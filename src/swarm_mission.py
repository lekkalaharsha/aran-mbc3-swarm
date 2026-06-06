#!/usr/bin/env python3
"""
swarm_mission.py — 5-drone parallel swarm mission with failure redistribution.

Phase 1: All 5 drones arm + climb concurrently.
Phase 2: All 5 drones execute their sector mission in parallel.
         If a drone fails mid-flight, its remaining waypoints are:
           1. Logged + printed to console.
           2. Distributed to adjacent active drones via D2D REASSIGN.
           3. Active drones pick up extra rows after their own sector completes.
Phase 3: All active drones RTL and land.

Mission redistribution (G3/G5):
  - leader sends REASSIGN via D2DNode.send_reassign()
  - receiving drone queues extra WPs in EXTRA_WPS[idx]
  - drone checks queue after its own sector → uploads + flies extra WPs
"""

import asyncio
import atexit
import math
import os
import subprocess
import sys
import threading
import time
from typing import Optional

import requests
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from d2d_node import D2DNode
from mission_config import HOME_LAT, HOME_LON, SPEED
from mpc_controller import haversine
from mission_config_swarm import (
    SWARM_NUM_DRONES,
    drone_alt,
    generate_drone_wps,
    compute_redistribution,
    DRONE_TARGET,
)

MAVSDK_SERVER = os.path.expanduser(
    "~/.local/lib/python3.12/site-packages/mavsdk/bin/mavsdk_server"
)

NUM_DRONES    = SWARM_NUM_DRONES
BASE_UDP      = 14540
BASE_GRPC     = 50050

# ── Process registry — cleaned up on any exit (normal, crash, Ctrl+C) ─────────
_mavsdk_procs: list = []

def _kill_mavsdk_servers() -> None:
    for p in _mavsdk_procs:
        try:
            p.terminate()
        except Exception:
            pass

atexit.register(_kill_mavsdk_servers)
MISSION_SPEED = 15.0    # m/s waypoint speed
CLIMB_SPEED   = 2.0     # m/s MPC_TKO_SPEED
ARM_TIMEOUT   = 90.0
GCS_URL       = "http://localhost:5000/asp_update"
EVENT_URL     = "http://localhost:5000/event_push"

# ── Shared state ──────────────────────────────────────────────────────────────
GCS_TRACK_URL = "http://localhost:5000/api/track_state"

drone_states: dict[int, dict] = {
    i: {
        "id":          f"DRONE-{i}",
        "lat":         0.0, "lon": 0.0, "alt": 0.0,
        "alt_amsl":    0.0,
        "heading":     0.0, "groundspeed": 0.0,
        "connected":   False, "armed": False,
        "flight_mode": "---", "phase": "INIT",
        "wp_current":  0, "wp_total": 0,
    }
    for i in range(NUM_DRONES)
}

# WP lists per drone: keys are drone indices, values are (lat,lon) lists
_drone_wps: dict[int, list] = {}        # primary sector WPs

# Extra WPs redistributed from failed drones: set by redistribution logic
EXTRA_WPS: dict[int, list[tuple]] = {i: [] for i in range(NUM_DRONES)}
EXTRA_WPS_LOCK = threading.Lock()

FAILED_DRONES: set[int] = set()
ACTIVE_DRONES: set[int] = set(range(NUM_DRONES))
_state_lock = threading.Lock()


def log(idx, msg):  print(f"[DRONE-{idx}] {msg}", flush=True)
def banner(msg):    print(f"\n{'='*55}\n  {msg}\n{'='*55}", flush=True)


# ── GCS push ──────────────────────────────────────────────────────────────────
def _push_loop():
    scan = 0
    _fail_count = 0
    while True:
        time.sleep(0.5)
        scan += 1
        drones = [dict(s) for s in drone_states.values()]
        try:
            requests.post(GCS_URL, json={
                "swarm_drones": drones,
                "scan_count":   scan,
                "asp_drone_id": "SWARM_MISSION",
            }, timeout=0.3)
            if _fail_count > 0:
                print(f"[PUSH] GCS reconnected after {_fail_count} failures", flush=True)
            _fail_count = 0
        except Exception as e:
            _fail_count += 1
            if _fail_count == 1:
                print(f"[PUSH] GCS unreachable — {e}", flush=True)
            elif _fail_count % 60 == 0:
                print(f"[PUSH] GCS still unreachable ({_fail_count} misses, ~{_fail_count//2}s)", flush=True)
        if scan % 30 == 0:
            info = [(s["id"], f"{s['alt']:.0f}m", s["phase"]) for s in drone_states.values()]
            print(f"[PUSH] #{scan}: {info}", flush=True)


# ── mavsdk servers ────────────────────────────────────────────────────────────
def start_mavsdk_servers() -> list:
    global _mavsdk_procs
    procs = []
    for i in range(NUM_DRONES):
        p = subprocess.Popen(
            [MAVSDK_SERVER, "-p", str(BASE_GRPC + i), f"udpin://0.0.0.0:{BASE_UDP + i}"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        print(f"[SWARM] mavsdk_server drone {i}: grpc={BASE_GRPC+i} udp={BASE_UDP+i} pid={p.pid}", flush=True)
        procs.append(p)
    _mavsdk_procs = procs  # register for atexit cleanup
    time.sleep(2)
    return procs


# ── Mission builders ──────────────────────────────────────────────────────────
def _make_item(lat, lon, alt, fly_through=True, loiter=0.0, land=False) -> MissionItem:
    return MissionItem(
        latitude_deg=lat, longitude_deg=lon,
        relative_altitude_m=alt, speed_m_s=MISSION_SPEED,
        is_fly_through=fly_through,
        gimbal_pitch_deg=float("nan"), gimbal_yaw_deg=float("nan"),
        camera_action=MissionItem.CameraAction.NONE,
        loiter_time_s=loiter, camera_photo_interval_s=0.0,
        acceptance_radius_m=15.0, yaw_deg=float("nan"),
        camera_photo_distance_m=0.0,
        vehicle_action=(MissionItem.VehicleAction.LAND if land else MissionItem.VehicleAction.NONE),
    )


def build_primary_plan(idx: int) -> tuple[MissionPlan, list[tuple]]:
    """
    Build mission plan for drone idx's assigned sector.
    Returns (MissionPlan, survey_wps_latlon).
    survey_wps_latlon is stored so we can compute redistribution offsets.
    """
    alt = drone_alt(idx)
    survey_wps = generate_drone_wps(idx)
    _drone_wps[idx] = survey_wps

    items: list[MissionItem] = []

    # WP0: climb to cruise altitude above home
    items.append(_make_item(HOME_LAT, HOME_LON, alt, fly_through=True))

    # Survey sector rows
    for i, (lat, lon) in enumerate(survey_wps):
        is_last = (i == len(survey_wps) - 1)
        items.append(_make_item(lat, lon, alt, fly_through=not is_last))

    # Secondary target orbit (if assigned)
    tgt = DRONE_TARGET.get(idx)
    if tgt:
        items.append(_make_item(
            tgt["lat"], tgt["lon"],
            tgt.get("orbit_altitude_m", alt),
            fly_through=False, loiter=float(tgt.get("orbit_duration_s", 10)),
        ))

    # RTL: return to home + land
    items.append(_make_item(HOME_LAT, HOME_LON, alt, fly_through=False, land=True))

    return MissionPlan(items), survey_wps


def build_extra_plan(idx: int, extra_wps: list[tuple]) -> MissionPlan:
    """Build a mini-plan from redistributed waypoints, ending with RTL."""
    alt = drone_alt(idx)
    items: list[MissionItem] = []
    for i, (lat, lon) in enumerate(extra_wps):
        items.append(_make_item(lat, lon, alt, fly_through=(i < len(extra_wps) - 1)))
    items.append(_make_item(HOME_LAT, HOME_LON, alt, fly_through=False, land=True))
    return MissionPlan(items)


# ── Telemetry streaming ───────────────────────────────────────────────────────
async def _stream_position(drone, idx):
    async for pos in drone.telemetry.position():
        drone_states[idx].update({
            "lat":      pos.latitude_deg,
            "lon":      pos.longitude_deg,
            "alt":      round(pos.relative_altitude_m, 1),
            "alt_amsl": round(pos.absolute_altitude_m, 1),
            "connected": True,
        })


async def _stream_velocity(drone, idx):
    import math
    async for vel in drone.telemetry.velocity_ned():
        spd = math.sqrt(vel.north_m_s**2 + vel.east_m_s**2)
        hdg = math.degrees(math.atan2(vel.east_m_s, vel.north_m_s)) % 360
        drone_states[idx].update({
            "groundspeed": round(spd, 1),
            "heading": round(hdg, 0),
        })


async def _stream_armed(drone, idx):
    async for armed in drone.telemetry.armed():
        drone_states[idx]["armed"] = armed


# ── Phase 1: connect + arm + climb ───────────────────────────────────────────
async def arm_and_climb(drone, idx) -> bool:
    log(idx, f"Connecting grpc:{BASE_GRPC + idx} ...")
    await drone.connect()
    async for state in drone.core.connection_state():
        if state.is_connected:
            drone_states[idx]["connected"] = True
            log(idx, "Connected ✓")
            break

    for fn, hz in [(drone.telemetry.set_rate_position, 2.0),
                   (drone.telemetry.set_rate_velocity_ned, 2.0)]:
        try:
            await fn(hz)
        except Exception:
            pass

    drone_states[idx]["phase"] = "HEALTH"
    t0 = asyncio.get_event_loop().time()
    async for h in drone.telemetry.health():
        elapsed = asyncio.get_event_loop().time() - t0
        if h.is_global_position_ok and h.is_local_position_ok and h.is_armable:
            log(idx, f"Health OK ({elapsed:.1f}s)")
            break
        if elapsed > ARM_TIMEOUT:
            log(idx, f"Health TIMEOUT after {ARM_TIMEOUT}s")
            return False
        await asyncio.sleep(0.5)

    try:
        await drone.param.set_param_float("MPC_TKO_SPEED", CLIMB_SPEED)
    except Exception:
        pass

    drone_states[idx]["phase"] = "ARMING"
    for attempt in range(1, 4):
        try:
            await drone.action.arm()
            log(idx, "Armed ✓")
            break
        except Exception as e:
            log(idx, f"Arm {attempt}/3: {e}")
            await asyncio.sleep(2.0)
    else:
        log(idx, "Arm FAILED")
        return False

    target_alt = drone_alt(idx)
    try:
        await drone.action.set_takeoff_altitude(target_alt)
    except Exception:
        pass

    drone_states[idx]["phase"] = "CLIMB"
    log(idx, f"Takeoff → {target_alt:.0f}m AGL")
    await drone.action.takeoff()
    return True


# ── Phase 2: run mission with redistribution ──────────────────────────────────
async def run_mission(
    drone, idx: int, plan: MissionPlan, d2d: D2DNode,
    survey_wps: list[tuple],
) -> None:
    """
    Upload and fly the primary mission plan.
    On failure: redistribute remaining WPs to active peers via D2D REASSIGN.
    On success: fly any extra WPs assigned via redistribution, then land.
    """
    try:
        drone_states[idx]["phase"] = "UPLOAD"
        log(idx, "Uploading mission ...")
        expected = len(plan.mission_items)
        uploaded = False
        for _attempt in range(1, 5):
            try:
                await drone.mission.upload_mission(plan)
                await asyncio.sleep(3.0)
                dl = await asyncio.wait_for(drone.mission.download_mission(), timeout=8.0)
                if len(dl.mission_items) == expected:
                    log(idx, f"Plan verified — {expected} items (attempt {_attempt})")
                    uploaded = True
                    break
                log(idx, f"Upload mismatch {len(dl.mission_items)}/{expected} — retry {_attempt}")
            except Exception as e:
                log(idx, f"Upload attempt {_attempt}/4 failed: {e}")
            await asyncio.sleep(3.0)
        if not uploaded:
            raise RuntimeError(f"Mission upload failed after 4 attempts — {expected} items unconfirmed")

        # Wait at cruise altitude before starting
        target_alt = drone_alt(idx)
        async for pos in drone.telemetry.position():
            if pos.relative_altitude_m >= target_alt * 0.90:
                log(idx, f"At {pos.relative_altitude_m:.1f}m — starting mission")
                break
            await asyncio.sleep(0.5)

        drone_states[idx]["phase"] = "SURVEY"

        # rMADER P4: broadcast sector trajectory before committing — check inter-drone conflicts
        t_cur = time.time() + 5.0
        traj_wps, traj_etas = [], []
        prev_lat, prev_lon = HOME_LAT, HOME_LON
        for wp_lat, wp_lon in survey_wps:
            t_cur += haversine(prev_lat, prev_lon, wp_lat, wp_lon) / MISSION_SPEED
            traj_wps.append((wp_lat, wp_lon, drone_alt(idx)))
            traj_etas.append(t_cur)
            prev_lat, prev_lon = wp_lat, wp_lon
        safe = await d2d.commit_trajectory(traj_wps, traj_etas)
        if not safe:
            log(idx, "rMADER: sector conflict unresolved after retries — proceeding with caution")

        await drone.mission.start_mission()
        log(idx, "Mission STARTED")

        # Track progress
        async for progress in drone.mission.mission_progress():
            drone_states[idx]["wp_current"] = progress.current
            drone_states[idx]["wp_total"]   = progress.total
            pct = int(100 * progress.current / progress.total) if progress.total else 0
            drone_states[idx]["phase"] = f"WP {progress.current}/{progress.total}"
            if progress.current % 2 == 0:
                log(idx, f"  WP {progress.current}/{progress.total}  ({pct}%)")
            if progress.current == progress.total:
                log(idx, "Primary mission complete ✓")
                break

    except Exception as exc:
        log(idx, f"FAILED: {exc}")
        with _state_lock:
            FAILED_DRONES.add(idx)
            ACTIVE_DRONES.discard(idx)

        # Compute remaining WPs from this drone's survey plan
        last_wp = drone_states[idx]["wp_current"]
        active = [i for i in ACTIVE_DRONES]  # snapshot
        redistrib = compute_redistribution(idx, last_wp, survey_wps, active)

        # Send REASSIGN via D2D (leader broadcasts to each target drone)
        for target_idx, wps in redistrib.items():
            log(idx, f"  → D2D REASSIGN {len(wps)} WPs to DRONE-{target_idx}")
            d2d.send_reassign(target_idx, [(lat, lon) for lat, lon in wps])
            # WPs delivered exclusively via D2D callback (_make_cb closure).
            # Do NOT also write directly to EXTRA_WPS — in SITL multicast loopback
            # fires the callback immediately, so a direct write duplicates every WP.
        # Push redistribution event to dashboard
        try:
            dist_str = ", ".join(f"D{k}:{len(v)}" for k, v in redistrib.items())
            requests.post(EVENT_URL, json={
                "msg":  f"[REDISTRIB] DRONE-{idx} failed WP {last_wp}/{len(survey_wps)}. → {dist_str}",
                "kind": "redistrib",
            }, timeout=0.5)
        except Exception:
            pass

        drone_states[idx]["phase"] = "FAILED"
        return

    # ── Check for redistributed extra WPs ─────────────────────────────────
    with EXTRA_WPS_LOCK:
        extra = list(EXTRA_WPS.get(idx, []))
        EXTRA_WPS[idx] = []

    if extra:
        log(idx, f"Executing {len(extra)} redistributed WPs from failed drone(s)")
        drone_states[idx]["phase"] = "EXTRA_WPS"
        extra_plan = build_extra_plan(idx, extra)
        try:
            await drone.mission.upload_mission(extra_plan)
            await drone.mission.start_mission()
            async for progress in drone.mission.mission_progress():
                drone_states[idx]["wp_current"] = progress.current
                drone_states[idx]["wp_total"]   = progress.total
                if progress.current == progress.total:
                    log(idx, "Extra WPs complete ✓")
                    break
        except Exception as e:
            log(idx, f"Extra WP execution failed: {e}")

    # ── Wait for landing ───────────────────────────────────────────────────
    drone_states[idx]["phase"] = "LANDING"
    log(idx, "Waiting for landing ...")
    try:
        from mavsdk.telemetry import LandedState
        async for landed in drone.telemetry.landed_state():
            if landed == LandedState.ON_GROUND:
                log(idx, "Landed ✓")
                drone_states[idx]["phase"] = "LANDED"
                break
            await asyncio.sleep(1.0)
    except Exception:
        drone_states[idx]["phase"] = "LANDED"


# ── Follow loop ───────────────────────────────────────────────────────────────

async def _follow_loop(drone, idx: int, d2d: D2DNode) -> None:
    """Poll GCS /api/track_state every 1 s.
    When this drone is assigned, override nav with repeated goto_location calls.
    goto_location takes absolute (AMSL) altitude — use alt_amsl from telemetry.
    """
    loop = asyncio.get_running_loop()
    _prev_phase = "INIT"
    while True:
        await asyncio.sleep(1.0)
        try:
            data = await loop.run_in_executor(
                None,
                lambda: requests.get(GCS_TRACK_URL, timeout=0.5).json(),
            )
        except Exception:
            continue

        if not data.get("active") or data.get("drone_idx") != idx:
            if drone_states[idx]["phase"] == "FOLLOW":
                drone_states[idx]["phase"] = _prev_phase
            continue

        if data.get("stale"):
            # Target disappeared — hover, don't chase ghost position
            continue

        lat  = data["lat"]
        lon  = data["lon"]
        amsl = drone_states[idx].get("alt_amsl", 0.0)
        if amsl < 5.0:
            # Drone not airborne yet — skip
            continue

        if drone_states[idx]["phase"] != "FOLLOW":
            _prev_phase = drone_states[idx]["phase"]
            drone_states[idx]["phase"] = "FOLLOW"
            log(idx, f"FOLLOW mode → {data['target_id']}  {lat:.6f},{lon:.6f}")

        safe = await d2d.commit_trajectory([(lat, lon, amsl)], [time.time() + 2.0])
        if not safe:
            log(idx, "rMADER: follow conflict — holding position this cycle")
            continue

        try:
            await drone.action.goto_location(lat, lon, amsl, float("nan"))
        except Exception as e:
            log(idx, f"goto_location failed — {e}")


RTL_BATTERY_PCT = 20.0   # auto-RTL threshold (matches proposal §2.11)


async def _stream_battery(drone, idx) -> None:
    """Auto-RTL when battery drops below RTL_BATTERY_PCT while armed and airborne.
    Retries on stream disconnect — battery monitoring must never die silently."""
    while True:
        try:
            async for batt in drone.telemetry.battery():
                pct = (batt.remaining_percent or 0.0) * 100.0
                drone_states[idx]["battery_pct"] = round(pct, 1)
                phase = drone_states[idx].get("phase", "INIT")
                if (
                    pct < RTL_BATTERY_PCT
                    and drone_states[idx].get("armed")
                    and phase not in {"RTL", "LANDING", "LANDED", "FAILED",
                                      "INIT", "STANDBY", "HEALTH"}
                ):
                    log(idx, f"LOW BATTERY {pct:.0f}% — auto RTL")
                    drone_states[idx]["phase"] = "RTL"
                    try:
                        await drone.action.return_to_launch()
                    except Exception as e:
                        log(idx, f"RTL on low battery failed: {e}")
                    return   # done — drone is RTL-ing
        except asyncio.CancelledError:
            raise
        except Exception as e:
            log(idx, f"battery stream error — {e}; retrying in 3s")
            await asyncio.sleep(3.0)


async def _stream_health(drone, idx) -> None:
    """HOLD on GPS loss. Saves pre-denial phase and restores it on GPS recovery.
    Retries on stream disconnect — GPS monitoring must never die silently."""
    _pre_deny_phase: str = ""
    while True:
        try:
            async for health in drone.telemetry.health():
                gps_ok = health.is_global_position_ok
                phase  = drone_states[idx].get("phase", "INIT")
                armed  = drone_states[idx].get("armed", False)
                if (
                    not gps_ok and armed
                    and phase not in {"INIT", "STANDBY", "ARMING", "CLIMB", "HEALTH",
                                      "LANDED", "FAILED", "GPS-DENIED", "RTL", "LANDING"}
                ):
                    log(idx, "GPS DENIED — HOLD (EKF baro+IMU fallback)")
                    _pre_deny_phase = phase
                    drone_states[idx]["phase"] = "GPS-DENIED"
                    try:
                        await drone.action.hold()
                    except Exception as e:
                        log(idx, f"hold() on GPS deny failed: {e}")
                elif gps_ok and phase == "GPS-DENIED":
                    restored = _pre_deny_phase or "SURVEY"
                    log(idx, f"GPS RESTORED — resuming phase {restored}")
                    drone_states[idx]["phase"] = restored
        except asyncio.CancelledError:
            raise
        except Exception as e:
            log(idx, f"health stream error — {e}; retrying in 3s")
            await asyncio.sleep(3.0)


# ── AI command executor ────────────────────────────────────────────────────────
async def _ai_cmd_loop(drones: list) -> None:
    """Poll GCS /api/pending_commands every 2 s and execute validated AI deltas."""
    loop = asyncio.get_running_loop()
    while True:
        await asyncio.sleep(2.0)
        try:
            data = await loop.run_in_executor(
                None,
                lambda: requests.get(
                    "http://localhost:5000/api/pending_commands", timeout=0.5
                ).json(),
            )
        except Exception:
            continue
        for delta in data.get("commands", []):
            action  = delta.get("action")
            targets = delta.get("target_drones", []) or list(range(NUM_DRONES))
            params  = delta.get("params", {})
            print(f"[AI-EXEC] {action} → drones={targets}  params={params}", flush=True)
            for idx in targets:
                if idx not in range(NUM_DRONES):
                    continue
                if idx in FAILED_DRONES:
                    continue
                try:
                    if action == "rtl_drone":
                        await drones[idx].action.return_to_launch()
                        drone_states[idx]["phase"] = "RTL"
                        print(f"[AI-EXEC] DRONE-{idx}: RTL triggered", flush=True)
                    elif action == "loiter":
                        await drones[idx].action.hold()
                        drone_states[idx]["phase"] = "HOLD"
                        print(f"[AI-EXEC] DRONE-{idx}: HOLD triggered", flush=True)
                    elif action == "abort_mission":
                        await drones[idx].mission.cancel_mission()
                        await drones[idx].action.hold()
                        drone_states[idx]["phase"] = "ABORT"
                        print(f"[AI-EXEC] DRONE-{idx}: ABORT triggered", flush=True)
                    elif action == "resume_mission":
                        await drones[idx].mission.start_mission()
                        drone_states[idx]["phase"] = "SURVEY"
                        print(f"[AI-EXEC] DRONE-{idx}: RESUME triggered", flush=True)
                    elif action == "change_speed":
                        spd = float(params.get("speed_ms", MISSION_SPEED))
                        await drones[idx].action.set_maximum_speed(spd)
                        print(f"[AI-EXEC] DRONE-{idx}: speed capped to {spd}m/s", flush=True)
                    elif action == "change_altitude":
                        print(f"[AI-EXEC] DRONE-{idx}: change_altitude requires mission rebuild "
                              f"— not supported mid-flight. Use RTL+relaunch.", flush=True)
                    elif action == "orbit_target":
                        print(f"[AI-EXEC] DRONE-{idx}: orbit_target mid-swarm not supported "
                              f"(use isr_lidar_mpc.py for single-drone recon).", flush=True)
                    elif action == "reassign_sector":
                        print(f"[AI-EXEC] DRONE-{idx}: reassign_sector not executable at runtime "
                              f"— sectors are fixed in mission upload.", flush=True)
                except Exception as e:
                    print(f"[AI-EXEC] DRONE-{idx}: {action} failed — {e}", flush=True)


# ── Main ──────────────────────────────────────────────────────────────────────
async def main():
    banner("ARAN MBC-3 — 5-DRONE SWARM MISSION WITH REDISTRIBUTION")
    alts = "/".join(f"{drone_alt(i):.0f}" for i in range(NUM_DRONES))
    print(f"Altitudes: {alts} m AGL  |  Speed: {MISSION_SPEED} m/s", flush=True)
    print(f"Redistribution: enabled — failed drone WPs → adjacent active drones", flush=True)

    procs = start_mavsdk_servers()

    threading.Thread(target=_push_loop, daemon=True).start()
    print("[SWARM] GCS push thread started → http://localhost:5000/asp", flush=True)

    drones = [
        System(mavsdk_server_address="localhost", port=BASE_GRPC + i)
        for i in range(NUM_DRONES)
    ]

    # Build per-drone mission plans
    plans_and_wps = [build_primary_plan(i) for i in range(NUM_DRONES)]
    mission_plans = [pw[0] for pw in plans_and_wps]
    drone_survey_wps = [pw[1] for pw in plans_and_wps]

    for i in range(NUM_DRONES):
        n_items = len(mission_plans[i].mission_items)
        n_rows  = len(drone_survey_wps[i])
        tgt     = DRONE_TARGET.get(i)
        tname   = tgt["name"] if tgt else "RTL only"
        print(f"  DRONE-{i}: {n_items} WPs | {n_rows} survey pts | alt={drone_alt(i):.0f}m | target={tname}", flush=True)

    # ── Phase 1: arm + climb ───────────────────────────────────────────────
    banner("PHASE 1 — ARM + CLIMB ALL 5 DRONES")
    results = await asyncio.gather(
        *[arm_and_climb(drones[i], i) for i in range(NUM_DRONES)],
        return_exceptions=True,
    )

    pre_failed = [i for i, r in enumerate(results) if r is not True]
    for i in pre_failed:
        FAILED_DRONES.add(i)
        ACTIVE_DRONES.discard(i)
        print(f"[SWARM] DRONE-{i}: arm/climb failed — excluded from mission", flush=True)

    active = list(ACTIVE_DRONES)
    if not active:
        print("[SWARM] All drones failed arm — aborting", flush=True)
        return

    # Telemetry streaming for all drones
    for i in range(NUM_DRONES):
        asyncio.create_task(_stream_position(drones[i], i))
        asyncio.create_task(_stream_velocity(drones[i], i))
        asyncio.create_task(_stream_armed(drones[i], i))
        asyncio.create_task(_stream_battery(drones[i], i))
        asyncio.create_task(_stream_health(drones[i], i))

    # D2D nodes — must start before follow loops (follow loops take d2d ref)
    d2d_nodes = [D2DNode(i, drone_states[i]) for i in range(NUM_DRONES)]
    # BUG-3 fix: register REASSIGN callbacks so hardware deployments (separate processes)
    # deliver redistributed WPs via D2D receipt, not shared-memory write.
    for i in range(NUM_DRONES):
        def _make_cb(idx):
            def _cb(wps):
                with EXTRA_WPS_LOCK:
                    EXTRA_WPS[idx].extend([(float(w[0]), float(w[1])) for w in wps])
                print(f"[D2D-RECV] DRONE-{idx}: +{len(wps)} WPs via REASSIGN callback", flush=True)
            return _cb
        d2d_nodes[i].on_reassign(_make_cb(i))
    for i in range(NUM_DRONES):
        asyncio.create_task(d2d_nodes[i].run())
    print("[SWARM] D2D multicast nodes running → 224.1.1.1:14900", flush=True)

    # Follow loops — poll GCS track_state, execute goto_location when assigned
    for i in range(NUM_DRONES):
        asyncio.create_task(_follow_loop(drones[i], i, d2d_nodes[i]))
    print("[SWARM] Follow loops started — poll GCS /api/track_state @ 1 Hz", flush=True)

    # AI command executor — poll GCS /api/pending_commands, execute rtl/loiter/abort
    asyncio.create_task(_ai_cmd_loop(drones))
    print("[SWARM] AI command executor started — poll GCS /api/pending_commands @ 2 Hz", flush=True)

    # Wait for all active drones at altitude
    print("[SWARM] Waiting for drones to reach cruise altitudes ...", flush=True)
    while True:
        at_alt = [i for i in active if drone_states[i]["alt"] >= drone_alt(i) * 0.90]
        alts_str = [f"{int(drone_states[i]['alt'])}/{drone_alt(i):.0f}m" for i in active]
        print(f"[SWARM]   At altitude: {len(at_alt)}/{len(active)}  {alts_str}", flush=True)
        if len(at_alt) >= len(active):
            break
        await asyncio.sleep(10)

    # ── Phase 2: parallel mission with redistribution ──────────────────────
    banner(f"PHASE 2 — PARALLEL MISSION ({len(active)} drones, redistribution ON)")
    for i in [j for j in range(NUM_DRONES) if j in FAILED_DRONES]:
        print(f"[SWARM] DRONE-{i}: skipped (arm failed)", flush=True)

    await asyncio.gather(
        *[
            run_mission(drones[i], i, mission_plans[i], d2d_nodes[i], drone_survey_wps[i])
            for i in active
        ],
        return_exceptions=True,
    )

    banner("ALL DRONES COMPLETE")
    summary = {
        "landed":    [i for i in range(NUM_DRONES) if drone_states[i]["phase"] == "LANDED"],
        "failed":    list(FAILED_DRONES),
        "remaining": [i for i in range(NUM_DRONES)
                      if drone_states[i]["phase"] not in ("LANDED", "FAILED", "INIT")],
    }
    print(f"[SWARM] Summary: {summary}", flush=True)

    for node in d2d_nodes:
        node.stop()
    for p in procs:
        p.terminate()


if __name__ == "__main__":
    asyncio.run(main())
