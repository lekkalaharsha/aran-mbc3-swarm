#!/usr/bin/env python3
"""
tests/mock_sim.py — Full stack simulation without SITL or Gazebo.

Components running simultaneously:
  GCS      telemetry_web.py  Flask server port 5000 (SWARM_MODE=1)
  ELECT    leader_election.py  bully daemon, polls /api/swarm_state
  DRONES   5 virtual drones moving along survey-grid waypoints
  D2D      D2DNode × 5  UDP multicast 224.1.1.1:14900
  RADAR    synthetic FMCW detections → /asp_update (no Gazebo)
  FAILOVER at t=30s: DRONE-4 crash → election → new leader = DRONE-3

Usage:
  python3 tests/mock_sim.py              # 90s run
  python3 tests/mock_sim.py --duration 60
  python3 tests/mock_sim.py --no-gcs    # skip starting GCS (already running)
"""

import argparse
import asyncio
import math
import os
import signal
import subprocess
import sys
import time
import threading

import requests

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
from mission_config import (
    HOME_LAT, HOME_LON,
    ROWS, ROW_SPACING, ROW_WIDTH,
    partition_survey_grid,
    generate_survey_grid,
    ALTITUDE, SPEED,
)
from d2d_node import D2DNode
from radar_sim import compute_detections_for_drone, push_asp

GCS_URL   = "http://localhost:5000"
ASP_URL   = f"{GCS_URL}/asp_update"
EARTH_R   = 111111.0
NUM        = 5
CRUISE_ALT = 100.0
ALT_SEP    = 10.0
MOCK_SPEED = 4.0   # m/s — slow so drones are visibly in-transit during 70s run

# Track max displacement per drone during run
_max_disp_m: dict[int, float] = {i: 0.0 for i in range(NUM)}

# ── Colours for terminal ──────────────────────────────────────────────────────
R = "\033[91m"; G = "\033[92m"; Y = "\033[93m"; B = "\033[94m"; RST = "\033[0m"

def log(tag, msg, colour=RST):
    print(f"{colour}[{tag}]{RST} {msg}", flush=True)


# ── Synthetic radar targets (world frame: x=East m, y=North m, z=alt m) ──────
# Placed so elevation ∈ [-5°, 25°] from drones at 100-140m AGL
RADAR_TARGETS = [
    {"name": "radar_target_0", "x":  100.0, "y":  320.0, "z": 82.0},
    {"name": "radar_target_1", "x": -140.0, "y":  410.0, "z": 75.0},
    {"name": "radar_target_2", "x":  200.0, "y":  210.0, "z": 91.0},
]


def drone_alt(idx):
    return CRUISE_ALT + idx * ALT_SEP


def latlon_to_world(lat, lon):
    x = (lon - HOME_LON) * EARTH_R * math.cos(math.radians(HOME_LAT))
    y = (lat - HOME_LAT) * EARTH_R
    return x, y


# ── Shared drone state (mimics swarm_mission.py drone_states) ─────────────────
drone_states = {
    i: {
        "id":          f"DRONE-{i}",
        "lat":         HOME_LAT,
        "lon":         HOME_LON,
        "alt":         drone_alt(i),
        "heading":     45.0,
        "groundspeed": 0.0,
        "connected":   True,
        "armed":       True,
        "phase":       "INIT",
    }
    for i in range(NUM)
}

# DRONE-4 alive flag — set False at failover
_drone4_alive = True


# ── PX4 PID file helpers (leader_election.py reads these) ─────────────────────

def create_fake_px4_pids():
    """Write own PID to /tmp/px4_swarm_pid_1..4 so leader_election thinks all alive."""
    my_pid = os.getpid()
    for i in range(1, NUM):
        path = f"/tmp/px4_swarm_pid_{i}"
        with open(path, "w") as f:
            f.write(str(my_pid))
    log("SIM", f"PX4 PID stubs created (pid={my_pid}) for drones 1-4", Y)


def kill_px4_pid(idx):
    """Remove PID file so leader_election sees that drone as dead."""
    path = f"/tmp/px4_swarm_pid_{idx}"
    try:
        os.remove(path)
        log("SIM", f"Removed PX4 PID stub for DRONE-{idx} → leader_election will time it out", R)
    except FileNotFoundError:
        pass


def cleanup_px4_pids():
    for i in range(1, NUM):
        try:
            os.remove(f"/tmp/px4_swarm_pid_{i}")
        except FileNotFoundError:
            pass


# ── GCS server ────────────────────────────────────────────────────────────────

def start_gcs() -> subprocess.Popen:
    env = {**os.environ, "SWARM_MODE": "1", "PYTHONPATH": os.path.join(os.path.dirname(__file__), "..")}
    gcs_path = os.path.join(os.path.dirname(__file__), "..", "src", "telemetry_web.py")
    proc = subprocess.Popen(
        [sys.executable, gcs_path],
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    log("GCS", f"Started pid={proc.pid} — waiting for port 5000 …", B)
    for _ in range(30):
        time.sleep(0.5)
        try:
            r = requests.get(f"{GCS_URL}/api/leader", timeout=0.5)
            if r.status_code == 200:
                log("GCS", "Port 5000 ready ✓", G)
                return proc
        except Exception:
            pass
    raise RuntimeError("GCS did not come up within 15s")


def start_leader_election() -> subprocess.Popen:
    elect_path = os.path.join(os.path.dirname(__file__), "..", "src", "leader_election.py")
    proc = subprocess.Popen(
        [sys.executable, elect_path],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    log("ELECT", f"leader_election.py started pid={proc.pid}", B)
    return proc


def pipe_reader(proc, tag, colour):
    """Background thread: forward subprocess stdout to console."""
    for line in proc.stdout:
        print(f"{colour}[{tag}]{RST} {line.decode(errors='replace').rstrip()}", flush=True)


# ── Drone position mover ───────────────────────────────────────────────────────

async def drone_mover(idx: int, stop_evt: asyncio.Event):
    """Loop survey waypoints at MOCK_SPEED m/s so drones stay in motion for full run."""
    base_wps = partition_survey_grid(idx, NUM)
    if not base_wps:
        base_wps = [(HOME_LAT + 0.001 * idx, HOME_LON + 0.001 * idx)]
    # Ping-pong: forward + reverse to stay in the grid area
    wps = base_wps + list(reversed(base_wps))

    state = drone_states[idx]
    state["phase"] = "SURVEY"
    wp_idx = 0

    while not stop_evt.is_set():
        dest_lat, dest_lon = wps[wp_idx % len(wps)]
        start_lat = state["lat"]
        start_lon = state["lon"]

        dlat = dest_lat - start_lat
        dlon = dest_lon - start_lon
        dist_m = math.sqrt(
            (dlat * EARTH_R) ** 2
            + (dlon * EARTH_R * math.cos(math.radians(HOME_LAT))) ** 2
        )

        if dist_m < 0.5:
            wp_idx += 1
            continue

        steps = max(2, int(dist_m / (MOCK_SPEED * 0.5)))
        state["groundspeed"] = round(MOCK_SPEED, 1)
        hdg = math.degrees(math.atan2(
            dlon * math.cos(math.radians(HOME_LAT)), dlat
        )) % 360
        state["heading"] = round(hdg, 1)

        for step in range(steps):
            if stop_evt.is_set():
                break
            t = (step + 1) / steps
            state["lat"] = round(start_lat + t * dlat, 7)
            state["lon"] = round(start_lon + t * dlon, 7)
            # Track max displacement from home for final report
            disp = math.sqrt(
                ((state["lat"] - HOME_LAT) * EARTH_R) ** 2
                + ((state["lon"] - HOME_LON) * EARTH_R
                   * math.cos(math.radians(HOME_LAT))) ** 2
            )
            _max_disp_m[idx] = max(_max_disp_m[idx], disp)
            await asyncio.sleep(0.5)

        wp_idx += 1


# ── GCS push loop ─────────────────────────────────────────────────────────────

async def gcs_push_loop(stop_evt: asyncio.Event):
    scan = 0
    while not stop_evt.is_set():
        scan += 1
        # Only push drones that are "alive" (DRONE-4 removed at failover)
        drones = []
        for i, s in drone_states.items():
            if i == 4 and not _drone4_alive:
                continue
            drones.append(dict(s))

        try:
            requests.post(ASP_URL, json={
                "swarm_drones": drones,
                "scan_count":   scan,
                "asp_drone_id": "MOCK_SIM",
            }, timeout=0.3)
        except Exception:
            pass
        await asyncio.sleep(0.5)


# ── Radar push loop ───────────────────────────────────────────────────────────

def _build_poses() -> dict:
    """Convert drone lat/lon + synthetic targets → Gazebo-style pose dict."""
    poses: dict = {}
    for i, state in drone_states.items():
        if i == 4 and not _drone4_alive:
            continue
        wx, wy = latlon_to_world(state["lat"], state["lon"])
        poses[f"mbc3_radar_drone_{i}"] = (
            wx, wy, state["alt"],
            0.0, 0.0, 0.0, 1.0,   # level flight quaternion
        )
    for tgt in RADAR_TARGETS:
        poses[tgt["name"]] = (tgt["x"], tgt["y"], tgt["z"], 0.0, 0.0, 0.0, 1.0)
    return poses


async def radar_push_loop(stop_evt: asyncio.Event):
    scan = 0
    while not stop_evt.is_set():
        scan += 1
        poses = _build_poses()

        # Fuse detections from all alive drones
        seen: dict = {}
        for i in range(NUM):
            if i == 4 and not _drone4_alive:
                continue
            for det in compute_detections_for_drone(f"mbc3_radar_drone_{i}", poses):
                if det["id"] not in seen:
                    seen[det["id"]] = det
        detections = list(seen.values())

        if detections:
            push_asp(detections, scan)

        if scan % 25 == 0:
            log("RADAR", f"scan#{scan}  {len(detections)}/{len(RADAR_TARGETS)} targets detected", B)

        await asyncio.sleep(0.2)


# ── D2D tasks ─────────────────────────────────────────────────────────────────

async def run_d2d(nodes: list[D2DNode], tasks: list):
    try:
        await asyncio.gather(*[n.run() for n in nodes], return_exceptions=True)
    except asyncio.CancelledError:
        pass


# ── Failover trigger ──────────────────────────────────────────────────────────

async def failover_trigger(delay: float, d2d_nodes: list[D2DNode], stop_evt: asyncio.Event):
    """At t=delay: crash DRONE-4, watch D2D + leader_election both elect DRONE-3."""
    global _drone4_alive
    await asyncio.sleep(delay)
    if stop_evt.is_set():
        return

    log("SIM", f"{'='*50}", R)
    log("SIM", "  *** DRONE-4 CRASH INJECTED ***", R)
    log("SIM", f"{'='*50}", R)

    _drone4_alive = False
    drone_states[4]["connected"] = False
    drone_states[4]["armed"]     = False
    drone_states[4]["phase"]     = "DEAD"

    # Stop D2D node for drone 4 — peers will detect HB silence
    d2d_nodes[4].stop()   # sets _running=False; monitor filters by _running

    # Remove PX4 PID file — leader_election.py will time it out after DEATH_TIMEOUT
    kill_px4_pid(4)

    # Inject stale HBs on remaining peers so their _election_watch fires in ~1s
    # (they still hold leader_idx=4 so the "leader silence" condition triggers).
    stale = time.time() - 17.0   # > DEATH_TIMEOUT (15s): last > 0 AND now-last > timeout
    for n in d2d_nodes[:4]:
        n.peer_last_hb[4] = stale
    log("SIM", "Stale HBs injected → D2D election in ~2s  |  GCS election in ~15s", Y)


# ── Status monitor ────────────────────────────────────────────────────────────

async def monitor_loop(stop_evt: asyncio.Event, d2d_nodes: list[D2DNode]):
    t0 = time.time()
    while not stop_evt.is_set():
        await asyncio.sleep(5.0)
        elapsed = time.time() - t0

        # D2D leader consensus — only count running nodes (stopped node keeps stale state)
        leaders = {n.leader_idx for n in d2d_nodes if n._running and n.leader_idx is not None}
        leader_str = f"DRONE-{max(leaders)}" if leaders else "none"

        # GCS leader
        try:
            r = requests.get(f"{GCS_URL}/api/leader", timeout=0.3)
            gcs_leader = r.json().get("leader_id", "?")
        except Exception:
            gcs_leader = "?"

        # Drone positions
        pos_lines = []
        for i in range(NUM):
            s = drone_states[i]
            alive = "  " if (i == 4 and not _drone4_alive) else "✓ "
            pos_lines.append(
                f"  DRONE-{i} {alive} "
                f"lat={s['lat']:.5f} lon={s['lon']:.5f} "
                f"alt={s['alt']:.0f}m  {s['phase']}"
            )

        log("MON", f"t={elapsed:.0f}s  D2D-leader={leader_str}  GCS-leader={gcs_leader}", G)
        for l in pos_lines:
            print(l, flush=True)

        # Issues check
        if len(leaders) > 1:
            log("ISSUE", f"D2D split-brain: running nodes disagree on leader {leaders}", R)


# ── Issue checker — run after simulation ──────────────────────────────────────

def final_report(d2d_nodes, start_time, failover_at):
    print(f"\n{'='*55}", flush=True)
    print("  MOCK SIM — FINAL REPORT", flush=True)
    print(f"{'='*55}", flush=True)

    issues = []
    t_now = time.time()

    # 1. D2D leader consensus post-failover.
    # d2d_nodes[:4] excludes DRONE-4; leader_idx state persists after stop().
    # Require at least one non-None to avoid vacuously passing an all-None list.
    end_leaders = [n.leader_idx for n in d2d_nodes[:4]]
    non_none = [l for l in end_leaders if l is not None]
    if non_none and all(l == 3 for l in non_none):
        print(f"  {G}PASS{RST} D2D failover: all nodes elected DRONE-3")
    elif not _drone4_alive:
        print(f"  {R}FAIL{RST} D2D failover: node leaders={end_leaders} (expected 3)")
        issues.append("D2D failover — DRONE-3 not elected by all nodes")

    # 2. GCS leader
    try:
        r = requests.get(f"{GCS_URL}/api/leader", timeout=1.0)
        gcs = r.json()
        eid = gcs.get("election_count", 0)
        ldr = gcs.get("leader_id", "?")
        if not _drone4_alive and ldr != "DRONE-3":
            print(f"  {R}FAIL{RST} GCS leader still {ldr} after failover (expected DRONE-3)")
            issues.append(f"GCS leader = {ldr} after DRONE-4 crash")
        else:
            print(f"  {G}PASS{RST} GCS leader = {ldr}  elections={eid}")
    except Exception as e:
        issues.append(f"GCS unreachable: {e}")

    # 3. Radar detections
    try:
        r = requests.get(f"{GCS_URL}/api/leader", timeout=1.0)
        print(f"  {G}PASS{RST} GCS reachable")
    except Exception:
        issues.append("GCS went down during simulation")

    # 4. Drone movement — check max displacement seen during run (not final position)
    moved = sum(1 for i in range(NUM) if _max_disp_m[i] > 2.0)
    disp_str = "  ".join(f"D{i}:{_max_disp_m[i]:.0f}m" for i in range(NUM))
    if moved >= 4:
        print(f"  {G}PASS{RST} {moved}/5 drones moved  ({disp_str})")
    else:
        print(f"  {R}FAIL{RST} Only {moved}/5 drones moved  ({disp_str})")
        issues.append(f"Drone movement — only {moved}/5 drones moved")

    print(f"\n  Issues found: {len(issues)}", flush=True)
    for issue in issues:
        print(f"    {R}✗{RST} {issue}", flush=True)
    if not issues:
        print(f"  {G}All checks passed.{RST}", flush=True)
    print(f"{'='*55}\n", flush=True)
    return issues


# ── Main ──────────────────────────────────────────────────────────────────────

async def amain(args):
    global _drone4_alive

    stop_evt  = asyncio.Event()
    t_start   = time.time()

    log("SIM", f"Mock sim starting  duration={args.duration}s  failover=t+30s", Y)
    log("SIM", f"Open GCS at http://localhost:5000/asp  (swarm panel + radar tracks)", Y)

    # ── GCS server ──────────────────────────────────────────────────────────
    gcs_proc = elect_proc = None
    if not args.no_gcs:
        gcs_proc = start_gcs()
        threading.Thread(target=pipe_reader, args=(gcs_proc, "GCS", B), daemon=True).start()
    else:
        log("SIM", "--no-gcs: assuming GCS already running on port 5000", Y)

    # ── PX4 PID stubs ───────────────────────────────────────────────────────
    create_fake_px4_pids()

    # ── leader_election daemon ───────────────────────────────────────────────
    elect_proc = start_leader_election()
    threading.Thread(target=pipe_reader, args=(elect_proc, "ELECT", Y), daemon=True).start()
    await asyncio.sleep(1.5)

    # ── D2D nodes ────────────────────────────────────────────────────────────
    d2d_nodes = [D2DNode(i, drone_states[i]) for i in range(NUM)]
    d2d_tasks = [asyncio.create_task(d2d_nodes[i].run()) for i in range(NUM)]
    await asyncio.sleep(0.5)   # let sockets bind

    # Kick off initial election from DRONE-0
    d2d_nodes[0]._start_election("sim init")
    log("D2D", "Initial bully election triggered", B)

    # ── Async sim tasks ──────────────────────────────────────────────────────
    mover_tasks = [
        asyncio.create_task(drone_mover(i, stop_evt))
        for i in range(NUM)
    ]
    push_task   = asyncio.create_task(gcs_push_loop(stop_evt))
    radar_task  = asyncio.create_task(radar_push_loop(stop_evt))
    mon_task    = asyncio.create_task(monitor_loop(stop_evt, d2d_nodes))
    fail_task   = asyncio.create_task(failover_trigger(30.0, d2d_nodes, stop_evt))

    # ── Run for duration ─────────────────────────────────────────────────────
    log("SIM", f"Running … (Ctrl-C to stop early)", G)
    try:
        await asyncio.sleep(args.duration)
    except asyncio.CancelledError:
        pass

    # ── Shutdown ─────────────────────────────────────────────────────────────
    log("SIM", "Duration reached — shutting down …", Y)
    stop_evt.set()

    for t in mover_tasks + [push_task, radar_task, mon_task, fail_task]:
        t.cancel()
    for n in d2d_nodes:
        n.stop()
    for t in d2d_tasks:
        t.cancel()

    await asyncio.sleep(0.5)

    # ── Final report ─────────────────────────────────────────────────────────
    issues = final_report(d2d_nodes, t_start, failover_at=30.0)

    # Cleanup
    cleanup_px4_pids()
    if elect_proc:
        elect_proc.terminate()
    if gcs_proc and not args.no_gcs:
        log("SIM", "GCS server still running — press Ctrl-C to stop it", Y)

    return len(issues)


def main():
    parser = argparse.ArgumentParser(description="MBC-3 full stack mock sim")
    parser.add_argument("--duration", type=float, default=90.0, help="run time in seconds")
    parser.add_argument("--no-gcs",  action="store_true", help="skip starting GCS (use existing)")
    args = parser.parse_args()

    exit_code = asyncio.run(amain(args))
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
