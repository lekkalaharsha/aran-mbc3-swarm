"""
swarm_monitor.py — Connect to 5 PX4 SITL drones, push positions to ASP GCS.

Each drone on udpin://0.0.0.0:1454N (N=0..4).
Pushes to http://localhost:5000/asp_update every 0.5s with all 5 drone positions.

Usage:
    python3 src/swarm_monitor.py
"""
import asyncio
import math
import os
import time
import requests
from mavsdk import System

GCS_ASP_URL = "http://localhost:5000/asp_update"
NUM_DRONES  = 5
BASE_PORT   = 14540
PUSH_HZ     = 2.0  # ASP update rate

# Shared state — one entry per drone
drone_states = {
    i: {
        "id":          f"DRONE-{i}",
        "lat":         0.0,
        "lon":         0.0,
        "alt":         0.0,
        "heading":     0.0,
        "groundspeed": 0.0,
        "armed":       False,
        "connected":   False,
        "flight_mode": "---",
    }
    for i in range(NUM_DRONES)
}


async def monitor_drone(idx: int):
    """Connect to drone idx and stream telemetry into drone_states.

    Runs _conn() continuously so connected status updates on both
    connect and disconnect — required for graceful degradation display.
    """
    port  = BASE_PORT + idx
    drone = System()
    await drone.connect(system_address=f"udpin://0.0.0.0:{port}")

    print(f"  [SWARM] Drone {idx}: connecting on port {port}...")

    async def _conn():
        async for state in drone.core.connection_state():
            was = drone_states[idx]["connected"]
            drone_states[idx]["connected"] = state.is_connected
            if state.is_connected and not was:
                print(f"  [SWARM] Drone {idx}: connected ✓", flush=True)
                # Reduced rates vs single-drone: 5 drones × streams saturates
                # the MAVSDK callback queue. 2 Hz position is plenty for ASP display.
                for fn, hz in [
                    (drone.telemetry.set_rate_position,     2.0),
                    (drone.telemetry.set_rate_velocity_ned, 2.0),
                ]:
                    try:
                        await fn(hz)
                    except Exception:
                        pass
            elif not state.is_connected and was:
                print(f"  [SWARM] Drone {idx}: DISCONNECTED — ASP will hide marker", flush=True)

    async def _pos():
        async for p in drone.telemetry.position():
            drone_states[idx]["lat"] = p.latitude_deg
            drone_states[idx]["lon"] = p.longitude_deg
            drone_states[idx]["alt"] = round(p.relative_altitude_m, 1)

    async def _vel():
        async for v in drone.telemetry.velocity_ned():
            vn = v.north_m_s
            ve = v.east_m_s
            drone_states[idx]["groundspeed"] = round(math.sqrt(vn**2 + ve**2), 1)

    async def _heading():
        async for h in drone.telemetry.heading():
            drone_states[idx]["heading"] = round(h.heading_deg, 1)

    async def _armed():
        async for a in drone.telemetry.armed():
            drone_states[idx]["armed"] = a

    async def _mode():
        async for m in drone.telemetry.flight_mode():
            drone_states[idx]["flight_mode"] = str(m).replace("FlightMode.", "")

    await asyncio.gather(
        _conn(), _pos(), _vel(), _heading(), _armed(), _mode(),
        return_exceptions=True
    )


async def push_loop():
    """Push all drone positions to ASP GCS every 0.5s."""
    interval = 1.0 / PUSH_HZ
    scan_count = 0
    while True:
        await asyncio.sleep(interval)
        scan_count += 1

        # Build drone positions list
        drones = [
            {
                "id":          s["id"],
                "lat":         s["lat"],
                "lon":         s["lon"],
                "alt":         s["alt"],
                "heading":     s["heading"],
                "groundspeed": s["groundspeed"],
                "armed":       s["armed"],
                "connected":   s["connected"],
                "flight_mode": s["flight_mode"],
            }
            for s in drone_states.values()
        ]

        payload = {
            "asp_tracks":   [],        # radar tracks (from radar_fusion, empty for now)
            "swarm_drones": drones,    # all 5 drone positions
            "scan_count":   scan_count,
            "asp_drone_id": "SWARM",
        }

        connected = sum(1 for s in drone_states.values() if s["connected"])
        if scan_count % 10 == 0:
            print(f"  [SWARM] Push #{scan_count}: {connected}/5 connected  "
                  f"alts={[s['alt'] for s in drone_states.values()]}", flush=True)

        try:
            requests.post(GCS_ASP_URL, json=payload, timeout=0.3)
        except Exception:
            pass


async def run():
    print("  [SWARM] Monitor starting — connecting to 5 drones on ports 14540-14544", flush=True)
    print("  [SWARM] ASP GCS: http://localhost:5000/asp", flush=True)

    tasks = [asyncio.create_task(monitor_drone(i)) for i in range(NUM_DRONES)]
    tasks.append(asyncio.create_task(push_loop()))
    await asyncio.gather(*tasks, return_exceptions=True)


if __name__ == "__main__":
    asyncio.run(run())
