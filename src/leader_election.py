#!/usr/bin/env python3
"""
leader_election.py — Bully-style leader election for MBC-3 swarm.

Polls /api/swarm_state (from swarm_monitor.py) to get real-time drone liveness.
Election rule: highest-index connected drone wins (drone_4 > drone_3 > ... > drone_0).
Highest-index drones can be killed via kill_drone.sh without losing the Gazebo
world (drone_0 owns the world and must stay alive).

On leader failure:
  - Detects within 2s (swarm_monitor grace period)
  - Elects next highest connected drone
  - POSTs new leader to /api/leader  →  radar_sim.py updates instantly
  - Emits 'leader' socket.io event   →  ASP page shows live election status

Usage:
    python3 src/leader_election.py

Demo:
    1. Run swarm: MBC3_MODE=1 bash swarm_launch.sh
    2. Confirm leader = DRONE-4 on ASP
    3. Kill: bash kill_drone.sh 4
    4. ASP shows election → new leader = DRONE-3
    5. Radar tracks continue from DRONE-3's position
"""

import json
import os
import sys
import time

import requests

GCS_URL        = "http://localhost:5000"
SWARM_URL      = f"{GCS_URL}/api/swarm_state"
LEADER_GET_URL = f"{GCS_URL}/api/leader"
LEADER_PUT_URL = f"{GCS_URL}/api/leader"

NUM_DRONES     = 5
POLL_HZ        = 2.0
# SITL MAVSDK oscillations drop all 5 drones for ~10s then recover.
# Real drone deaths (kill_drone.sh) are permanent.
# 15s timeout ignores oscillation blips; detects real kills reliably.
DEATH_TIMEOUT  = 15.0


def _px4_process_alive(idx: int) -> bool:
    """Check if PX4 SITL instance idx is alive via PID file written by swarm_launch.sh.

    os.kill(pid, 0) sends no signal — it just checks process existence.
    Faster and more accurate than pgrep: no fork/exec, exact PID match,
    immune to pattern collisions (e.g. -i 1 matching -i 10).
    """
    if idx == 0:
        return True   # instance 0 owns Gazebo world, never killed
    try:
        with open(f"/tmp/px4_swarm_pid_{idx}") as f:
            pid = int(f.read().strip())
        os.kill(pid, 0)   # raises ProcessLookupError if dead
        return True
    except (FileNotFoundError, ProcessLookupError):
        return False
    except Exception:
        return True       # fail-safe: assume alive if check errors


def _drone_model(idx: int) -> str:
    return f"mbc3_radar_drone_{idx}"


def _drone_id(idx: int) -> str:
    return f"DRONE-{idx}"


def get_swarm_state() -> list[dict]:
    """Poll /api/swarm_state — returns list of drone dicts from swarm_monitor."""
    try:
        r = requests.get(SWARM_URL, timeout=0.5)
        return r.json().get("swarm_drones", [])
    except Exception:
        return []


# Per-drone last-seen timestamps — updated whenever connected=True is observed.
# Election uses these rather than the instantaneous connected flag so that
# SITL MAVSDK oscillation blips (~10s all-offline) don't trigger false elections.
_drone_last_online: dict[int, float] = {0: time.time()}  # drone 0 owns Gazebo world, always alive


def update_liveness(drones: list[dict], now: float) -> None:
    """Record timestamp whenever a drone reports connected=True AND its PX4 process is alive.

    The process check prevents MAVSDK oscillation from refreshing a killed
    drone's timestamp — without it, a killed drone briefly re-appears as
    connected during oscillation cycles, resetting its DEATH_TIMEOUT.
    """
    for d in drones:
        drone_id  = d.get("id", "")
        connected = d.get("connected", False)
        if not connected:
            continue
        try:
            idx = int(drone_id.split("-")[1])
            if _px4_process_alive(idx):
                _drone_last_online[idx] = now
        except (IndexError, ValueError):
            pass


def is_alive(idx: int, now: float) -> bool:
    """Drone alive if seen connected within DEATH_TIMEOUT seconds."""
    return (now - _drone_last_online.get(idx, 0.0)) < DEATH_TIMEOUT


def elect(now: float) -> int | None:
    """
    Bully election using liveness timestamps (not instantaneous connected flag).
    Returns highest-index alive drone, or None if all timed out.
    """
    winner = None
    for idx in range(NUM_DRONES):
        if is_alive(idx, now):
            if winner is None or idx > winner:
                winner = idx
    return winner


def post_leader(idx: int, election_count: int) -> None:
    """POST new leader to GCS so radar_sim.py and ASP update."""
    payload = {
        "leader_id":       _drone_id(idx),
        "leader_model":    _drone_model(idx),
        "since":           time.time(),
        "election_count":  election_count,
    }
    try:
        requests.post(LEADER_PUT_URL, json=payload, timeout=0.5)
    except Exception:
        pass


def main() -> None:
    print("[ELECT] Leader election daemon started", flush=True)
    print(f"[ELECT] Rule: highest-index connected drone wins", flush=True)
    print(f"[ELECT] Polling {SWARM_URL} at {POLL_HZ} Hz", flush=True)

    current_leader: int | None = None
    election_count = 0
    interval = 1.0 / POLL_HZ
    cycle = 0

    print(f"[ELECT] DEATH_TIMEOUT={DEATH_TIMEOUT}s (ignores SITL blips <{DEATH_TIMEOUT}s)", flush=True)

    while True:
        t0   = time.time()
        now  = t0
        cycle += 1

        drones = get_swarm_state()
        update_liveness(drones, now)      # stamp drones seen connected this cycle
        winner = elect(now)               # liveness-based election

        if winner != current_leader:
            if winner is None:
                print(f"[ELECT] All drones timed out — no leader", flush=True)
            else:
                election_count += 1
                if current_leader is None:
                    print(f"[ELECT] Initial leader: {_drone_id(winner)}", flush=True)
                else:
                    if winner > current_leader:
                        reason = f"{_drone_id(current_leader)} outranked"
                    else:
                        reason = f"{_drone_id(current_leader)} silent >{DEATH_TIMEOUT}s"
                    print(
                        f"[ELECT] *** ELECTION #{election_count} ***  "
                        f"{reason} → {_drone_id(winner)} is new leader",
                        flush=True,
                    )

            current_leader = winner
            if winner is not None:
                post_leader(winner, election_count)

        # Periodic status every 10 cycles
        if cycle % 20 == 0:
            alive = [_drone_id(i) for i in range(NUM_DRONES) if is_alive(i, now)]
            ldr   = _drone_id(current_leader) if current_leader is not None else "NONE"
            print(
                f"[ELECT] leader={ldr}  alive={alive}  elections={election_count}",
                flush=True,
            )

        elapsed = time.time() - t0
        time.sleep(max(0.0, interval - elapsed))


if __name__ == "__main__":
    main()
