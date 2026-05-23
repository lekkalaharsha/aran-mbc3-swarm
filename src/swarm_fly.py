#!/usr/bin/env python3
"""
swarm_fly.py — Arm + takeoff + monitor all 5 MBC-3 swarm drones.

Uses one dedicated mavsdk_server process per drone — avoids udpin port-sharing
that causes telemetry mixing across drones.

Also pushes drone positions to GCS /asp_update so ASP page shows 5 separate
drone markers with correct lat/lon/alt. Replaces swarm_monitor for position data.

Architecture:
    PX4 instance i → udpin:14540+i → mavsdk_server (grpc:50050+i) → swarm_fly
                                                                   → GCS /asp_update
"""

import asyncio
import os
import subprocess
import sys
import time
import threading

import requests
from mavsdk import System

MAVSDK_SERVER = os.path.expanduser(
    "~/.local/lib/python3.12/site-packages/mavsdk/bin/mavsdk_server"
)
TARGET_ALT   = 100.0
CLIMB_SPEED  = 2.0
NUM_DRONES   = 5
BASE_UDP     = 14540
BASE_GRPC    = 50050
ARM_TIMEOUT  = 90.0
GCS_URL      = "http://localhost:5000/asp_update"
PUSH_HZ      = 2.0

# Shared state — updated by each drone coroutine
drone_states = {
    i: {
        "id":        f"DRONE-{i}",
        "lat":       0.0,
        "lon":       0.0,
        "alt":       0.0,
        "connected": False,
    }
    for i in range(NUM_DRONES)
}


def log(idx: int, msg: str) -> None:
    print(f"[DRONE-{idx}] {msg}", flush=True)


def start_mavsdk_servers() -> list:
    procs = []
    for i in range(NUM_DRONES):
        cmd = [
            MAVSDK_SERVER,
            "-p", str(BASE_GRPC + i),
            f"udpin://0.0.0.0:{BASE_UDP + i}",
        ]
        p = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print(f"[SWARM FLY] mavsdk_server drone {i}: grpc={BASE_GRPC+i} udp={BASE_UDP+i} pid={p.pid}", flush=True)
        procs.append(p)
    time.sleep(2)
    return procs


def push_loop() -> None:
    """Background thread: push all drone positions to GCS every 0.5s."""
    interval = 1.0 / PUSH_HZ
    scan = 0
    while True:
        time.sleep(interval)
        scan += 1
        drones = [dict(s) for s in drone_states.values()]
        payload = {
            "swarm_drones": drones,
            "scan_count":   scan,
            "asp_drone_id": "SWARM",
        }
        try:
            requests.post(GCS_URL, json=payload, timeout=0.3)
        except Exception:
            pass
        if scan % 20 == 0:
            connected = sum(1 for s in drone_states.values() if s["connected"])
            alts = [f"{s['alt']:.0f}m" for s in drone_states.values()]
            print(f"[PUSH] #{scan}: {connected}/5 connected  alts={alts}", flush=True)


async def fly_drone(idx: int) -> None:
    grpc_port = BASE_GRPC + idx
    drone = System(mavsdk_server_address="localhost", port=grpc_port)
    log(idx, f"Connecting grpc:{grpc_port} ...")
    await drone.connect()

    async for state in drone.core.connection_state():
        if state.is_connected:
            drone_states[idx]["connected"] = True
            log(idx, "Connected ✓")
            break

    for fn, hz in [
        (drone.telemetry.set_rate_position,     2.0),
        (drone.telemetry.set_rate_velocity_ned, 2.0),
    ]:
        try:
            await fn(hz)
        except Exception:
            pass

    # Health check (skip home_position_ok — not sent by slave SITL instances)
    log(idx, "Waiting for health checks...")
    import asyncio as _a
    t0 = _a.get_event_loop().time()
    async for health in drone.telemetry.health():
        elapsed = _a.get_event_loop().time() - t0
        gps = health.is_global_position_ok
        loc = health.is_local_position_ok
        arm = health.is_armable
        if gps and loc and arm:
            log(idx, f"Health OK ({elapsed:.1f}s)")
            break
        if elapsed > ARM_TIMEOUT:
            log(idx, f"TIMEOUT: gps={gps} local={loc} armable={arm}")
            return
        if int(elapsed) % 15 == 0 and int(elapsed) > 0 and elapsed % 15 < 0.5:
            log(idx, f"  {elapsed:.0f}s: gps={gps} local={loc} armable={arm}")
        await asyncio.sleep(0.5)

    for pname, pval in [("MPC_TKO_SPEED", CLIMB_SPEED)]:
        try:
            await drone.param.set_param_float(pname, float(pval))
            log(idx, f"  {pname}={pval} ✓")
        except Exception:
            pass

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
        return

    try:
        await drone.action.set_takeoff_altitude(TARGET_ALT)
    except Exception:
        pass
    log(idx, f"Takeoff → {TARGET_ALT}m at {CLIMB_SPEED}m/s")
    await drone.action.takeoff()

    # Monitor + stream position
    prev = 0.0
    reached = False
    last_log_t = 0.0
    async for pos in drone.telemetry.position():
        alt = pos.relative_altitude_m
        # Update shared state for push_loop
        drone_states[idx]["lat"] = pos.latitude_deg
        drone_states[idx]["lon"] = pos.longitude_deg
        drone_states[idx]["alt"] = round(alt, 1)

        if not reached:
            if alt - prev >= 20.0:
                log(idx, f"Alt: {alt:.1f}m / {TARGET_ALT}m")
                prev = alt
            if alt >= TARGET_ALT * 0.95:
                log(idx, f"TARGET REACHED: {alt:.1f}m ✓")
                reached = True

        # Log loiter heartbeat every 60s
        now_t = asyncio.get_event_loop().time()
        if reached and (now_t - last_log_t) >= 60.0:
            log(idx, f"Loiter alt={alt:.1f}m  lat={pos.latitude_deg:.6f} lon={pos.longitude_deg:.6f}")
            last_log_t = now_t


async def main() -> None:
    print(f"[SWARM FLY] target={TARGET_ALT}m  climb={CLIMB_SPEED}m/s  drones={NUM_DRONES}", flush=True)

    procs = start_mavsdk_servers()

    # Push thread — sends all 5 positions to GCS
    t = threading.Thread(target=push_loop, daemon=True)
    t.start()
    print("[SWARM FLY] Position push loop started → GCS /asp_update", flush=True)

    try:
        tasks = [asyncio.create_task(fly_drone(i)) for i in range(NUM_DRONES)]
        await asyncio.gather(*tasks, return_exceptions=True)
    finally:
        for p in procs:
            p.terminate()


if __name__ == "__main__":
    asyncio.run(main())
