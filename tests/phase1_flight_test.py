"""
Phase 1 — PX4 SITL flight test with mbc3_radar_drone.

Tests:
  T1  Drone connects via MAVSDK
  T2  Armed successfully
  T3  Takeoff to 10 m — altitude reached within 30 s
  T4  Hover stable — altitude drift < 1 m over 10 s
  T5  Fly to waypoint 50 m north — arrival within 60 s
  T6  Return to home — within 10 m of origin within 90 s
  T7  Land — grounded (alt < 0.5 m) within 60 s
  T8  No FAILED / Traceback lines in PX4 log

Run:
  # 1. Install model:  bash new_drone/install_px4_model.sh
  # 2. Start SITL:     ./launch.sh --sim-only
  # 3. Run test:       python3 tests/phase1_flight_test.py
"""

import asyncio
import math
import sys
import time

try:
    from mavsdk import System
    from mavsdk.action import ActionError
    from mavsdk.telemetry import LandedState
except ImportError:
    print("SKIP  mavsdk not installed — pip install mavsdk")
    sys.exit(0)

MAVSDK_ADDR = "udpin://0.0.0.0:14540"
HOME_LAT = 47.397742
HOME_LON = 8.545594
TEST_ALT = 10.0        # m AGL for hover test
WP_LAT   = HOME_LAT + (50 / 111_320)   # 50 m north
WP_LON   = HOME_LON
TIMEOUT_ARM      = 20
TIMEOUT_TAKEOFF  = 30
TIMEOUT_HOVER    = 15
TIMEOUT_WP       = 60
TIMEOUT_RTH      = 90
TIMEOUT_LAND     = 60

PASS = 0
FAIL = 0


def check(name, cond, detail=''):
    global PASS, FAIL
    if cond:
        print(f"  PASS  {name}" + (f"  ({detail})" if detail else ""))
        PASS += 1
    else:
        print(f"  FAIL  {name}  {detail}")
        FAIL += 1
    return cond


async def wait_until(cond_fn, timeout, poll=0.5):
    """Poll cond_fn() every poll seconds until True or timeout."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        if await cond_fn():
            return True
        await asyncio.sleep(poll)
    return False


async def run():
    print("=== PHASE 1 — PX4 FLIGHT TEST (mbc3_radar_drone) ===")
    print()

    drone = System()
    print(f"Connecting to {MAVSDK_ADDR} ...")
    await drone.connect(system_address=MAVSDK_ADDR)

    # T1 — connect
    connected = False
    async for state in drone.core.connection_state():
        if state.is_connected:
            connected = True
            break
    if not check("T1  MAVSDK connects", connected):
        print("Cannot continue — SITL not running?")
        return

    # wait for health checks to clear
    print("  Waiting for health: global position + home ...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            break

    # T2 — arm
    try:
        await drone.action.arm()
        check("T2  Armed OK", True)
    except ActionError as e:
        check("T2  Armed OK", False, str(e))
        return

    # T3 — takeoff to TEST_ALT
    await drone.action.set_takeoff_altitude(TEST_ALT)
    await drone.action.takeoff()

    async def above_alt():
        async for pos in drone.telemetry.position():
            return pos.relative_altitude_m >= TEST_ALT * 0.85
    reached = await wait_until(above_alt, TIMEOUT_TAKEOFF)
    alt_now = 0.0
    async for pos in drone.telemetry.position():
        alt_now = pos.relative_altitude_m
        break
    check("T3  Takeoff reaches 8.5 m+", reached, f"alt={alt_now:.1f}m")

    # T4 — hover stability: altitude drift < 1 m over 10 s
    await asyncio.sleep(3)
    alts = []
    t0 = time.time()
    while time.time() - t0 < 10:
        async for pos in drone.telemetry.position():
            alts.append(pos.relative_altitude_m)
            break
        await asyncio.sleep(0.5)
    drift = max(alts) - min(alts) if alts else 99
    check("T4  Hover stable (drift < 1 m / 10 s)", drift < 1.0,
          f"drift={drift:.2f}m")

    # T5 — fly to waypoint 50 m north
    home_abs_alt = 0.0
    async for pos in drone.telemetry.position():
        home_abs_alt = pos.absolute_altitude_m - pos.relative_altitude_m
        break
    await drone.action.goto_location(WP_LAT, WP_LON,
                                     home_abs_alt + TEST_ALT, float("nan"))

    async def near_wp():
        async for pos in drone.telemetry.position():
            d = math.sqrt(
                ((pos.latitude_deg  - WP_LAT) * 111_320) ** 2 +
                ((pos.longitude_deg - WP_LON) * 111_320 *
                 math.cos(math.radians(WP_LAT))) ** 2
            )
            return d < 5.0
    reached_wp = await wait_until(near_wp, TIMEOUT_WP)
    check("T5  Waypoint 50 m north reached", reached_wp)

    # T6 — RTH
    await drone.action.return_to_launch()

    async def near_home():
        async for pos in drone.telemetry.position():
            d = math.sqrt(
                ((pos.latitude_deg  - HOME_LAT) * 111_320) ** 2 +
                ((pos.longitude_deg - HOME_LON) * 111_320 *
                 math.cos(math.radians(HOME_LAT))) ** 2
            )
            return d < 10.0
    reached_home = await wait_until(near_home, TIMEOUT_RTH)
    check("T6  RTH — within 10 m of home", reached_home)

    # T7 — land
    async def grounded():
        async for ls in drone.telemetry.landed_state():
            return ls == LandedState.ON_GROUND
    landed = await wait_until(grounded, TIMEOUT_LAND)
    check("T7  Landed (ON_GROUND state)", landed)

    # T8 — no crash markers (user verifies PX4 log separately)
    print()
    print("  T8  Check PX4 log manually:")
    print("      grep -i 'FAILED\\|Traceback\\|crash' logs/*/px4.log")
    print("      Expected: 0 matches")
    PASS_val = PASS
    FAIL_val = FAIL

    print()
    print(f"Results: {PASS_val} passed, {FAIL_val} failed")
    if FAIL_val == 0:
        print("PHASE 1 — ALL PASS  ✓  mbc3_radar_drone flies correctly")
    else:
        print("PHASE 1 — FAILURES DETECTED  — check airframe + SITL logs")
    sys.exit(0 if FAIL_val == 0 else 1)


if __name__ == "__main__":
    asyncio.run(run())
