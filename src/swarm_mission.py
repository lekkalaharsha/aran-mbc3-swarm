#!/usr/bin/env python3
"""
swarm_mission.py — 5-drone sequential swarm mission.

Phase 1: All 5 drones arm + climb to CRUISE_ALT concurrently.
Phase 2: Each drone executes the same survey + orbit mission ONE BY ONE.
Phase 3: Each drone RTLs and lands after its mission.

Uses dedicated mavsdk_server per drone (gRPC 50050-50054) for isolated control.
Pushes live positions to GCS /asp_update for ASP display.
"""

import asyncio
import os
import subprocess
import sys
import threading
import time

import requests
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from d2d_node import D2DNode
from mission_config import (
    HOME_LAT, HOME_LON,
    TARGET_LAT, TARGET_LON,
    generate_survey_grid,
    ROWS, ALTITUDE, SPEED,
)

MAVSDK_SERVER = os.path.expanduser(
    "~/.local/lib/python3.12/site-packages/mavsdk/bin/mavsdk_server"
)

NUM_DRONES   = 5
BASE_UDP     = 14540
BASE_GRPC    = 50050
CRUISE_ALT   = 100.0    # m AGL (demo: 100m; production: use ALTITUDE=500m)
CLIMB_SPEED  = 2.0      # m/s MPC_TKO_SPEED
MISSION_SPEED = 15.0    # m/s waypoint speed
ARM_TIMEOUT  = 90.0
GCS_URL      = "http://localhost:5000/asp_update"

# Shared state updated by drone coroutines
drone_states = {
    i: {
        "id":          f"DRONE-{i}",
        "lat":         0.0,
        "lon":         0.0,
        "alt":         0.0,
        "heading":     0.0,
        "groundspeed": 0.0,
        "connected":   False,
        "armed":       False,
        "flight_mode": "---",
        "phase":       "INIT",
    }
    for i in range(NUM_DRONES)
}


def log(idx, msg): print(f"[DRONE-{idx}] {msg}", flush=True)
def banner(msg):   print(f"\n{'='*55}\n  {msg}\n{'='*55}", flush=True)


# ── GCS push ──────────────────────────────────────────────────────────────
def push_loop():
    scan = 0
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
        except Exception:
            pass
        if scan % 30 == 0:
            info = [(s["id"], f"{s['alt']:.0f}m", s["phase"]) for s in drone_states.values()]
            print(f"[PUSH] #{scan}: {info}", flush=True)


# ── mavsdk_server ─────────────────────────────────────────────────────────
def start_mavsdk_servers():
    procs = []
    for i in range(NUM_DRONES):
        p = subprocess.Popen(
            [MAVSDK_SERVER, "-p", str(BASE_GRPC + i), f"udpin://0.0.0.0:{BASE_UDP + i}"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        print(f"[SWARM] mavsdk_server drone {i}: grpc={BASE_GRPC+i} udp={BASE_UDP+i} pid={p.pid}", flush=True)
        procs.append(p)
    time.sleep(2)
    return procs


# ── Mission items (same for every drone) ─────────────────────────────────
def build_mission() -> MissionPlan:
    """Survey grid + primary target approach at CRUISE_ALT."""
    items = []

    # First WP: climb to cruise alt above home before survey
    items.append(MissionItem(
        latitude_deg=HOME_LAT, longitude_deg=HOME_LON,
        relative_altitude_m=CRUISE_ALT, speed_m_s=MISSION_SPEED,
        is_fly_through=True,
        gimbal_pitch_deg=float("nan"), gimbal_yaw_deg=float("nan"),
        camera_action=MissionItem.CameraAction.NONE,
        loiter_time_s=0.0, camera_photo_interval_s=0.0,
        acceptance_radius_m=15.0, yaw_deg=float("nan"),
        camera_photo_distance_m=0.0,
        vehicle_action=MissionItem.VehicleAction.NONE,
    ))

    # Survey grid
    waypoints = generate_survey_grid()
    for i, (lat, lon) in enumerate(waypoints):
        is_last = (i == len(waypoints) - 1)
        items.append(MissionItem(
            latitude_deg=lat, longitude_deg=lon,
            relative_altitude_m=CRUISE_ALT, speed_m_s=MISSION_SPEED,
            is_fly_through=not is_last,
            gimbal_pitch_deg=float("nan"), gimbal_yaw_deg=float("nan"),
            camera_action=MissionItem.CameraAction.NONE,
            loiter_time_s=0.0, camera_photo_interval_s=0.0,
            acceptance_radius_m=15.0, yaw_deg=float("nan"),
            camera_photo_distance_m=0.0,
            vehicle_action=MissionItem.VehicleAction.NONE,
        ))

    # Primary target flyover
    items.append(MissionItem(
        latitude_deg=TARGET_LAT, longitude_deg=TARGET_LON,
        relative_altitude_m=CRUISE_ALT, speed_m_s=MISSION_SPEED,
        is_fly_through=False,
        gimbal_pitch_deg=float("nan"), gimbal_yaw_deg=float("nan"),
        camera_action=MissionItem.CameraAction.NONE,
        loiter_time_s=5.0, camera_photo_interval_s=0.0,
        acceptance_radius_m=10.0, yaw_deg=float("nan"),
        camera_photo_distance_m=0.0,
        vehicle_action=MissionItem.VehicleAction.NONE,
    ))

    # Return to home and land
    items.append(MissionItem(
        latitude_deg=HOME_LAT, longitude_deg=HOME_LON,
        relative_altitude_m=CRUISE_ALT, speed_m_s=MISSION_SPEED,
        is_fly_through=False,
        gimbal_pitch_deg=float("nan"), gimbal_yaw_deg=float("nan"),
        camera_action=MissionItem.CameraAction.NONE,
        loiter_time_s=0.0, camera_photo_interval_s=0.0,
        acceptance_radius_m=10.0, yaw_deg=float("nan"),
        camera_photo_distance_m=0.0,
        vehicle_action=MissionItem.VehicleAction.LAND,
    ))

    return MissionPlan(items)


# ── Position streaming ─────────────────────────────────────────────────────
async def stream_position(drone, idx):
    """Continuously update shared state from position + velocity telemetry."""
    import math
    async for pos in drone.telemetry.position():
        drone_states[idx]["lat"] = pos.latitude_deg
        drone_states[idx]["lon"] = pos.longitude_deg
        drone_states[idx]["alt"] = round(pos.relative_altitude_m, 1)
        if pos.latitude_deg != 0.0 or pos.longitude_deg != 0.0:
            drone_states[idx]["connected"] = True
        drone_states[idx]["flight_mode"] = drone_states[idx]["phase"]


async def stream_velocity(drone, idx):
    """Stream groundspeed + heading from velocity_ned."""
    import math
    async for vel in drone.telemetry.velocity_ned():
        spd = math.sqrt(vel.north_m_s**2 + vel.east_m_s**2)
        hdg = math.degrees(math.atan2(vel.east_m_s, vel.north_m_s)) % 360
        drone_states[idx]["groundspeed"] = round(spd, 1)
        drone_states[idx]["heading"]     = round(hdg, 0)


async def stream_armed(drone, idx):
    """Stream armed status."""
    async for armed in drone.telemetry.armed():
        drone_states[idx]["armed"] = armed


# ── Phase 1: connect + arm + takeoff ─────────────────────────────────────
async def arm_and_climb(drone, idx) -> bool:
    grpc_port = BASE_GRPC + idx
    log(idx, f"Connecting grpc:{grpc_port} ...")

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

    # Health — skip home_position_ok (not set on slave SITL instances)
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

    try:
        await drone.action.set_takeoff_altitude(CRUISE_ALT)
    except Exception:
        pass

    drone_states[idx]["phase"] = "CLIMB"
    log(idx, f"Takeoff → {CRUISE_ALT}m")
    await drone.action.takeoff()
    return True


# ── Phase 2: run mission ──────────────────────────────────────────────────
async def run_mission(drone, idx, mission_plan: MissionPlan) -> None:
    drone_states[idx]["phase"] = "MISSION_UPLOAD"
    log(idx, "Uploading mission plan ...")
    await drone.mission.upload_mission(mission_plan)
    log(idx, f"Mission uploaded ({len(mission_plan.mission_items)} items)")

    # Wait to be at cruise altitude before starting
    async for pos in drone.telemetry.position():
        if pos.relative_altitude_m >= CRUISE_ALT * 0.90:
            log(idx, f"At cruise alt {pos.relative_altitude_m:.1f}m — starting mission")
            break
        await asyncio.sleep(0.5)

    drone_states[idx]["phase"] = "SURVEY"
    await drone.mission.start_mission()
    log(idx, "Mission STARTED")

    # Wait for mission completion
    async for progress in drone.mission.mission_progress():
        pct = int(100 * progress.current / progress.total) if progress.total else 0
        if progress.current != progress.total:
            drone_states[idx]["phase"] = f"WP {progress.current}/{progress.total}"
            if progress.current % 2 == 0:
                log(idx, f"  WP {progress.current}/{progress.total}  ({pct}%)")
        else:
            log(idx, "Mission complete ✓")
            drone_states[idx]["phase"] = "LANDING"
            break

    # Mission last item has VehicleAction.LAND at HOME — drone is already landing.
    # Wait for landed
    async for landed in drone.telemetry.landed_state():
        from mavsdk.telemetry import LandedState
        if landed == LandedState.ON_GROUND:
            log(idx, "Landed ✓")
            drone_states[idx]["phase"] = "LANDED"
            break
        await asyncio.sleep(1.0)


# ── Main ──────────────────────────────────────────────────────────────────
async def main():
    banner("ARAN MBC-3 — 5-DRONE SEQUENTIAL SWARM MISSION")
    print(f"Cruise: {CRUISE_ALT}m AGL  |  Speed: {MISSION_SPEED}m/s  |  Climb: {CLIMB_SPEED}m/s", flush=True)

    procs = start_mavsdk_servers()

    # GCS push thread
    threading.Thread(target=push_loop, daemon=True).start()
    print("[SWARM] Position push → GCS /asp_update started", flush=True)

    # Create drone objects
    drones = [
        System(mavsdk_server_address="localhost", port=BASE_GRPC + i)
        for i in range(NUM_DRONES)
    ]

    # Build mission (same for all drones)
    mission_plan = build_mission()
    print(f"[SWARM] Mission: {len(mission_plan.mission_items)} waypoints (survey + target + RTL)", flush=True)

    # ── Phase 1: arm + climb all concurrently ──────────────────────────
    banner("PHASE 1 — ARM + CLIMB ALL 5 DRONES")
    results = await asyncio.gather(
        *[arm_and_climb(drones[i], i) for i in range(NUM_DRONES)],
        return_exceptions=True,
    )
    failed = [i for i, r in enumerate(results) if r is not True]
    if failed:
        print(f"[SWARM] WARNING: drones {failed} failed arm/climb — continuing with rest", flush=True)

    # Start telemetry streaming for all drones concurrently
    pos_tasks = (
        [asyncio.create_task(stream_position(drones[i], i)) for i in range(NUM_DRONES)] +
        [asyncio.create_task(stream_velocity(drones[i], i)) for i in range(NUM_DRONES)] +
        [asyncio.create_task(stream_armed(drones[i], i))    for i in range(NUM_DRONES)]
    )

    # Start D2D nodes — one per drone, broadcast HB + run bully election
    d2d_nodes = [D2DNode(i, drone_states[i]) for i in range(NUM_DRONES)]
    d2d_tasks = [asyncio.create_task(d2d_nodes[i].run()) for i in range(NUM_DRONES)]
    print("[SWARM] D2D multicast nodes started — 224.1.1.1:14900", flush=True)

    # Wait for all to reach cruise alt
    print("[SWARM] Waiting for all drones to reach cruise altitude ...", flush=True)
    while True:
        at_alt = [i for i in range(NUM_DRONES) if drone_states[i]["alt"] >= CRUISE_ALT * 0.90]
        alts = [str(int(drone_states[i]["alt"])) + "m" for i in range(NUM_DRONES)]
        print(f"[SWARM]   At altitude: {len(at_alt)}/5  alts={alts}", flush=True)
        if len(at_alt) >= NUM_DRONES - len(failed):
            break
        await asyncio.sleep(10)

    # ── Phase 2: mission one by one ─────────────────────────────────────
    banner("PHASE 2 — SEQUENTIAL MISSION (drone by drone)")
    for i in range(NUM_DRONES):
        if i in failed:
            print(f"[SWARM] DRONE-{i}: skipped (arm failed)", flush=True)
            continue
        banner(f"DRONE-{i} MISSION START ({i+1}/{NUM_DRONES})")
        await run_mission(drones[i], i, mission_plan)
        print(f"[SWARM] DRONE-{i} complete — next drone in 5s", flush=True)
        await asyncio.sleep(5)

    banner("ALL DRONES MISSION COMPLETE")
    for node in d2d_nodes:
        node.stop()
    for task in d2d_tasks:
        task.cancel()
    for p in procs:
        p.terminate()


if __name__ == "__main__":
    asyncio.run(main())
