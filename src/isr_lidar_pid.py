"""
Aran Technologies — ISR Mission + LiDAR PID Avoidance  [v11]
Full autonomous mission: Survey -> Obstacle Avoidance -> Target Acq -> Orbit -> RTL

v11 Bug Fixes:
  - push_to_gcs(): sectors[] was missing from POST payload — GCS sector
    overlay was permanently frozen at init values; now sent every tick
  - push_to_gcs(): TOCTOU race on avoidance_state["last_wp"] — check and
    index happened in separate expressions across a thread boundary; fixed by
    capturing a single snapshot before building the payload
  - avoidance_loop(): after timeout/climb escape activates, the normal
    detour path still ran every 50 Hz tick, spamming conflicting horizontal
    goto_location commands during the climb; guarded with early `continue`
    when timeout_active is True
  - _compute_eta(): wp_current is a mission-item index (home WP at index 0),
    but was used directly as a 0-based index into the survey waypoints array,
    skipping one extra waypoint per ETA calculation; corrected to
    max(wp_current - 1, 0)
  - telemetry_tracker(): used asyncio.gather with no exception handling; a
    single MAVSDK stream error cancelled all four telemetry coroutines
    permanently mid-flight; each stream now runs in an independent retry loop
  - _bearing_to_nearest() / _compute_sectors() [CRITICAL]: both returned /
    indexed bearings in sensor-relative frame (0 = drone forward axis) but
    best_escape_bearing() and compute_avoidance_waypoint() expect world-absolute
    frame (0 = North). The detour waypoint was projected in the wrong absolute
    direction at any heading except North. Fixed by adding drone_state["heading"]
    to convert sensor-frame angles to world frame before any downstream use.

v10 Bug Fixes:
  - avoidance_state["count"] now increments once per obstacle event (not every
    50 Hz tick); the count was previously inflated by ~50x per second of contact
  - _compute_eta() now uses live drone_state["groundspeed"] instead of the
    constant SPEED=50 m/s; falls back to SPEED only before telemetry arrives

v10 Refactor:
  - All shared coordinates / grid constants / generate_survey_grid() moved to
    mission_config.py — no more silent divergence between mission and GCS map

v9 Changes vs v7:
  - Bearing bug fixed: escape direction fed directly to compute_avoidance_waypoint
    (no extra +90deg perpendicular rotation that previously offset detour ~90deg)
  - Left/right escape selection: picks the side with more sector clearance
  - Avoidance timeout: if obstacle persists >AVOIDANCE_TIMEOUT_S, drone climbs
    CLIMB_ESCAPE_M metres to fly over instead of hovering indefinitely
  - AvoidancePID gain scheduling: update_speed() called each avoidance tick
  - haversine() imported from pid_controller (no duplication)
  - Mission ETA telemetry: remaining WP distance / speed pushed to GCS
  - GPS health (HDOP proxy via health flags) pushed to GCS
  - Reconnect counter pushed to GCS
  - /pid_tune POST endpoint supported via GCS_URL scheme
"""
import asyncio
import math
import sys
import threading
import time
import requests
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan
from mavsdk.action import OrbitYawBehavior
from mavsdk.telemetry import LandedState

from pid_controller import (
    AvoidancePID, AltitudePID, OrbitPID,
    compute_avoidance_waypoint, best_escape_bearing, haversine
)

# ── All shared mission constants come from one place ──────
from mission_config import (
    HOME_LAT, HOME_LON,
    ALTITUDE, SPEED,
    ROWS, ROW_SPACING, ROW_WIDTH,
    TARGET_LAT, TARGET_LON,
    ORBIT_RADIUS, ORBIT_SPEED, ORBIT_ALTITUDE, ORBIT_DURATION,
    SECONDARY_TARGETS, NO_FLY_ZONES, LOITER_WAYPOINTS,
    ALTITUDE_STEP, GRID_ALTITUDE_STEPS,
    generate_survey_grid, generate_all_sweeps, get_nfz_exclusion_check,
)

try:
    from gz.transport13 import Node
    from gz.msgs10.laserscan_pb2 import LaserScan
    GZ_AVAILABLE = True
except ImportError:
    GZ_AVAILABLE = False
    print("  Warning  gz-transport not found — running in LiDAR-SIM mode")
    print("     Install: sudo apt install python3-gz-transport13 python3-gz-msgs10\n")


# ══════════════════════════════════════════════════════════
#  LIDAR CONFIG  (local to mission script only)
# ══════════════════════════════════════════════════════════
LIDAR_TOPIC         = "/lidar_360/scan"
LIDAR_WARN_DIST     = 25.0
LIDAR_AVOID_DIST    = 15.0
LIDAR_POLL_HZ       = 50
AVOIDANCE_OFFSET_M  = 50.0
AVOIDANCE_HOLD_S    = 2.0
SAFE_RESUME_DIST    = 22.0

DEBOUNCE_COUNT      = 3
MEDIAN_WINDOW       = 5
SECTOR_COUNT        = 8

# v9: avoidance escalation — climb if obstacle persists
AVOIDANCE_TIMEOUT_S = 10.0   # seconds before climb escape
CLIMB_ESCAPE_M      = 15.0   # metres to climb above current alt

# GCS dashboard endpoint
GCS_URL = "http://localhost:5000/lidar_update"


# ══════════════════════════════════════════════════════════
#  SHARED STATE
# ══════════════════════════════════════════════════════════
lidar_state = {
    "nearest_dist":    float("inf"),
    "nearest_bearing": 0.0,
    "raw_ranges":      [],
    "scan_count":      0,
    "dist_history":    [],
    "filtered_dist":   float("inf"),
    "debounce":        0,
    "sectors":         [float("inf")] * SECTOR_COUNT,
}

avoidance_state = {
    "active":         False,
    "count":          0,
    "last_wp":        None,
    "escape_side":    "---",
    "timeout_active": False,
}

drone_state = {
    "lat":         HOME_LAT,
    "lon":         HOME_LON,
    "alt":         0.0,
    "abs_alt":     0.0,
    "heading":     0.0,
    "groundspeed": 0.0,
    "gps_ok":      False,
    "reconnects":  0,
}

mission_state = {
    "wp_current":   0,
    "wp_total":     0,
    "eta_seconds":  None,
}

mission_done    = asyncio.Event()
abort_avoidance = asyncio.Event()


# ══════════════════════════════════════════════════════════
#  GCS PUSH
# ══════════════════════════════════════════════════════════
def push_to_gcs():
    try:
        wp_cur = mission_state["wp_current"]
        wp_tot = mission_state["wp_total"]
        eta    = mission_state["eta_seconds"]

        # BUG FIX: last_wp is written by the asyncio loop and read here from a
        # daemon thread.  Capture a single snapshot to avoid a TOCTOU race where
        # last_wp is truthy on the `if` check but becomes None before indexing.
        last_wp_snap = avoidance_state["last_wp"]

        payload = {
            "nearest_dist":     lidar_state["nearest_dist"],
            "nearest_bearing":  lidar_state["nearest_bearing"],
            "scan_count":       lidar_state["scan_count"],
            "avoidance_active": avoidance_state["active"],
            "avoidance_count":  avoidance_state["count"],
            "escape_side":      avoidance_state["escape_side"],
            "timeout_active":   avoidance_state["timeout_active"],
            "detour_lat":       last_wp_snap[0] if last_wp_snap else None,
            "detour_lon":       last_wp_snap[1] if last_wp_snap else None,
            # BUG FIX: sectors were missing from the payload — the GCS
            # sector-clearance overlay was permanently frozen at init values.
            "sectors":          list(lidar_state["sectors"]),
            "alert_msg":        (f"OBSTACLE {lidar_state['nearest_dist']:.1f}m "
                                 f"@ {lidar_state['nearest_bearing']:.0f}deg "
                                 f"ESCAPE:{avoidance_state['escape_side']}")
                                 if avoidance_state["active"] else "",
            "groundspeed":      drone_state["groundspeed"],
            "gps_ok":           drone_state["gps_ok"],
            "reconnects":       drone_state["reconnects"],
            "eta_seconds":      eta,
            "wp_current":       wp_cur,
            "wp_total":         wp_tot,
        }
        requests.post(GCS_URL, json=payload, timeout=0.2)
    except Exception:
        pass


def start_gcs_push_loop():
    while True:
        push_to_gcs()
        time.sleep(0.2)


# ══════════════════════════════════════════════════════════
#  HELPERS
# ══════════════════════════════════════════════════════════
def banner(msg):
    print(f"\n{'='*60}")
    print(f"  {msg}")
    print(f"{'='*60}")

def log(msg):       print(f"  > {msg}")
def log_warn(msg):  print(f"  WARNING  {msg}")
def log_alert(msg): print(f"  ALERT    {msg}")


def _bearing_to_nearest(ranges, angle_min, angle_increment):
    """Return (min_dist_m, world_bearing_deg) to the nearest valid obstacle.

    BUG FIX (Critical): previously returned a sensor-relative bearing
    (0 = drone's forward axis) because the raw scan angles are in the sensor
    frame.  best_escape_bearing() and compute_avoidance_waypoint() both expect
    a world-absolute bearing (0 = North, clockwise).  Adding drone_state heading
    rotates the sensor-frame angle into the world frame.
    """
    valid = [(i, r) for i, r in enumerate(ranges)
             if r > 0.05 and not math.isinf(r) and not math.isnan(r)]
    if not valid:
        return float("inf"), 0.0
    idx, min_dist = min(valid, key=lambda x: x[1])
    sensor_bearing_rad = angle_min + idx * angle_increment
    sensor_bearing_deg = math.degrees(sensor_bearing_rad) % 360
    world_bearing_deg  = (sensor_bearing_deg + drone_state["heading"]) % 360
    return min_dist, world_bearing_deg


def _compute_sectors(ranges, angle_min, angle_increment):
    """Build 8-sector clearance map in WORLD frame (sector 0 = 0-45° = N-NE).

    BUG FIX (Critical): previously indexed sectors by sensor-relative bearing,
    so sector 0 was 'directly ahead of the drone', not 'North'.
    best_escape_bearing() then picked a world-frame escape bearing whose sector
    index was looked up in a sensor-frame array — wrong sector chosen whenever
    the drone wasn't heading North.  Adding drone heading rotates each ray into
    the world frame before sector binning.
    """
    sector_size = 360.0 / SECTOR_COUNT
    sectors = [float("inf")] * SECTOR_COUNT
    heading = drone_state["heading"]
    for i, r in enumerate(ranges):
        if r <= 0.05 or math.isinf(r) or math.isnan(r):
            continue
        sensor_deg  = math.degrees(angle_min + i * angle_increment) % 360
        world_deg   = (sensor_deg + heading) % 360
        s = int(world_deg / sector_size) % SECTOR_COUNT
        if r < sectors[s]:
            sectors[s] = r
    return sectors


def _median_filter(dist):
    h = lidar_state["dist_history"]
    h.append(dist)
    if len(h) > MEDIAN_WINDOW:
        h.pop(0)
    sorted_h = sorted(h)
    return sorted_h[len(sorted_h) // 2]


def _compute_eta(wp_current, wp_total, waypoints):
    """Estimate seconds to survey completion based on remaining WP distance.

    mission_items layout: [home_wp, survey_wp_0, survey_wp_1, ..., survey_wp_N-1]
    progress.current is 0-based index into that list.
    survey waypoints array is 0-based and offset by 1 vs mission items.

    BUG FIX: previously used wp_current directly as a waypoints[] index,
    but wp_current=1 already means the first survey WP is in progress — the
    correct survey slice index is max(wp_current - 1, 0).
    """
    if wp_current >= wp_total or not waypoints:
        return 0
    survey_idx = max(wp_current - 1, 0)   # subtract home WP offset
    remaining_wps = waypoints[survey_idx:]
    if len(remaining_wps) < 2:
        return 0
    total_dist = sum(
        haversine(remaining_wps[i][0], remaining_wps[i][1],
                  remaining_wps[i+1][0], remaining_wps[i+1][1])
        for i in range(len(remaining_wps)-1)
    )
    # Uses live telemetry groundspeed; falls back to config SPEED only before
    # first telemetry frame arrives.
    spd = drone_state.get("groundspeed") or SPEED
    return int(total_dist / spd) if spd > 0 else None


# ══════════════════════════════════════════════════════════
#  LIDAR READER
# ══════════════════════════════════════════════════════════
async def lidar_gz_reader():
    log(f"LiDAR 360: subscribing to {LIDAR_TOPIC} via gz-transport...")
    node = Node()
    scan_queue = asyncio.Queue(maxsize=1)
    loop = asyncio.get_event_loop()

    def on_scan(msg):
        try:
            while not scan_queue.empty():
                try: scan_queue.get_nowait()
                except asyncio.QueueEmpty: break
            loop.call_soon_threadsafe(scan_queue.put_nowait, msg)
        except Exception:
            pass

    node.subscribe(LaserScan, LIDAR_TOPIC, on_scan)
    log("LiDAR 360: subscriber active")

    while True:
        msg = await scan_queue.get()
        ranges = list(msg.ranges)
        if not ranges:
            continue
        min_dist, bearing = _bearing_to_nearest(ranges, msg.angle_min, msg.angle_step)
        sectors  = _compute_sectors(ranges, msg.angle_min, msg.angle_step)
        filtered = _median_filter(min_dist)
        lidar_state.update({
            "nearest_dist":    min_dist,
            "nearest_bearing": bearing,
            "filtered_dist":   filtered,
            "raw_ranges":      ranges,
            "sectors":         sectors,
        })
        lidar_state["scan_count"] += 1


async def lidar_sim_reader():
    """
    Simulated 360deg LiDAR reader for use when gz-transport is not available.

    v12 upgrade: instead of a single hardcoded obstacle, the sim reader can
    load and replay any scenario from scenarios.json.  Set SIM_SCENARIO to a
    scenario name to run it, or leave as None for the legacy single-obstacle
    behaviour.  Multiple simultaneous events are supported by merging all
    active events into a combined range array each tick.

    Also checks NO_FLY_ZONES each tick and synthesises a 360deg wall of range=2m
    around the drone if it is inside a no-fly zone, forcing immediate avoidance.
    """
    import random, json, os

    SIM_SCENARIO = None   # set to a scenario name string to replay it, e.g. "urban_canyon"

    log("LiDAR 360 [SIM]: running in SIMULATED mode (no gz-transport)")

    # ── load scenario if requested ────────────────────────────────────
    scenario_events = []
    scenario_pid    = None
    if SIM_SCENARIO:
        scenario_file = os.path.join(os.path.dirname(__file__), "scenarios.json")
        try:
            with open(scenario_file) as f:
                sc_data = json.load(f)
            match = next((s for s in sc_data["scenarios"] if s["name"] == SIM_SCENARIO), None)
            if match:
                scenario_events = match["events"]
                scenario_pid    = match.get("pid_gains")
                log(f"LiDAR-SIM: loaded scenario '{SIM_SCENARIO}' "
                    f"({len(scenario_events)} events)")
                if scenario_pid:
                    avoid_pid_gains = scenario_pid
                    log(f"LiDAR-SIM: scenario PID overrides — "
                        f"Kp={scenario_pid['kp']} Ki={scenario_pid['ki']} Kd={scenario_pid['kd']}")
            else:
                log_warn(f"LiDAR-SIM: scenario '{SIM_SCENARIO}' not found — using legacy sim")
        except Exception as e:
            log_warn(f"LiDAR-SIM: could not load scenarios.json — {e}")

    sim_start = asyncio.get_event_loop().time()

    while True:
        now_rel = asyncio.get_event_loop().time() - sim_start
        fake_ranges = [float("inf")] * 360
        ANGLE_INC   = math.radians(1.0)

        if scenario_events:
            # ── scenario-driven multi-obstacle injection ──────────────
            active_events = [
                ev for ev in scenario_events
                if ev["start_s"] <= now_rel < ev["start_s"] + ev["duration_s"]
            ]
            for ev in active_events:
                bearing_deg = ev["bearing_deg"]
                dist_m      = ev["dist_m"] + random.uniform(-0.2, 0.2)
                spread_deg  = 12   # angular width of obstacle blob
                center_idx  = int(bearing_deg) % 360
                for offset in range(-spread_deg // 2, spread_deg // 2 + 1):
                    idx = (center_idx + offset) % 360
                    if dist_m < fake_ranges[idx]:
                        fake_ranges[idx] = dist_m
        else:
            # ── legacy single-obstacle after 8s ──────────────────────
            if now_rel >= 8.0:
                for idx in range(35, 56):
                    fake_ranges[idx] = 12.0 + random.uniform(-0.3, 0.3)
                if not hasattr(lidar_sim_reader, "_legacy_logged"):
                    lidar_sim_reader._legacy_logged = True
                    log_warn("LiDAR-SIM: injecting obstacle at 12m, bearing 45deg")

        # ── no-fly zone wall synthesis ────────────────────────────────
        try:
            from mission_config import get_nfz_exclusion_check
            nfz_inside, nfz_name, nfz_dist = get_nfz_exclusion_check(
                drone_state["lat"], drone_state["lon"]
            )
            if nfz_inside:
                log_alert(f"NFZ BREACH: {nfz_name} — synthesising 360deg obstacle wall")
                fake_ranges = [2.0] * 360   # hard wall in every direction
        except Exception:
            pass

        # ── update shared lidar state ─────────────────────────────────
        min_dist, bearing = _bearing_to_nearest(fake_ranges, 0.0, ANGLE_INC)
        sectors  = _compute_sectors(fake_ranges, 0.0, ANGLE_INC)
        filtered = _median_filter(min_dist)
        lidar_state.update({
            "nearest_dist":    min_dist,
            "nearest_bearing": bearing,
            "filtered_dist":   filtered,
            "sectors":         sectors,
        })
        lidar_state["scan_count"] += 1
        await asyncio.sleep(1.0 / LIDAR_POLL_HZ)


async def lidar_reader():
    if GZ_AVAILABLE:
        await lidar_gz_reader()
    else:
        await lidar_sim_reader()


# ══════════════════════════════════════════════════════════
#  TELEMETRY TRACKER
# ══════════════════════════════════════════════════════════
async def telemetry_tracker(drone):
    """
    BUG FIX: previously used asyncio.gather(_pos(), _hdg(), _vel(), _health())
    with no error handling.  If any one stream raised (e.g. MAVSDK disconnect),
    gather propagated the exception and cancelled all other coroutines, silently
    killing all telemetry for the rest of the flight.  Each stream now runs in
    its own independent retry loop so a transient disconnect in one stream
    cannot bring down the others.
    """
    async def _resilient(name, coro_factory, retry_s=1.0):
        while True:
            try:
                await coro_factory()
            except asyncio.CancelledError:
                raise
            except Exception as e:
                log_warn(f"Telemetry stream '{name}' error — {str(e)[:80]}; retrying in {retry_s:.0f}s")
                await asyncio.sleep(retry_s)

    async def _pos():
        async for p in drone.telemetry.position():
            drone_state["lat"]     = p.latitude_deg
            drone_state["lon"]     = p.longitude_deg
            drone_state["alt"]     = p.relative_altitude_m
            drone_state["abs_alt"] = p.absolute_altitude_m

    async def _hdg():
        async for h in drone.telemetry.heading():
            drone_state["heading"] = h.heading_deg

    async def _vel():
        async for v in drone.telemetry.velocity_ned():
            drone_state["groundspeed"] = math.sqrt(v.north_m_s**2 + v.east_m_s**2)

    async def _health():
        async for h in drone.telemetry.health():
            drone_state["gps_ok"] = h.is_global_position_ok

    await asyncio.gather(
        _resilient("position", _pos),
        _resilient("heading",  _hdg),
        _resilient("velocity", _vel),
        _resilient("health",   _health),
    )


# ══════════════════════════════════════════════════════════
#  AVOIDANCE LOOP  (v9 — bearing fix + left/right + timeout)
# ══════════════════════════════════════════════════════════
async def avoidance_loop(drone):
    avoid_pid = AvoidancePID(safe_distance=LIDAR_AVOID_DIST + 2.0)
    interval  = 1.0 / LIDAR_POLL_HZ
    debounce  = 0
    avoidance_start_time = None

    log(f"Avoidance loop started — 360deg scan {LIDAR_POLL_HZ}Hz "
        f"debounce={DEBOUNCE_COUNT} timeout={AVOIDANCE_TIMEOUT_S}s")

    while not mission_done.is_set():
        await asyncio.sleep(interval)
        if abort_avoidance.is_set():
            continue

        dist    = lidar_state["filtered_dist"]
        bearing = lidar_state["nearest_bearing"]
        sectors = lidar_state["sectors"]

        avoid_pid.update_speed(drone_state["groundspeed"])

        # WARNING zone
        if LIDAR_AVOID_DIST < dist <= LIDAR_WARN_DIST:
            log_warn(f"LiDAR WARNING  obstacle={dist:.1f}m  bearing={bearing:.0f}deg")
            # BUG FIX: reset debounce here so a fresh obstacle entry starts
            # counting from zero, not carrying over a partial count from a
            # previous event that cleared through the WARNING zone.
            debounce = 0
            avoidance_state["timeout_active"] = False
            continue

        # CLEAR zone (hysteresis)
        if dist > SAFE_RESUME_DIST:
            if avoidance_state["active"]:
                avoidance_state["active"]         = False
                avoidance_state["timeout_active"] = False
                avoidance_start_time              = None
                debounce = 0
                log("Obstacle cleared (hysteresis) — resuming mission")
                try:
                    await drone.mission.start_mission()
                except Exception as e:
                    log_warn(f"Mission resume error: {e}")
            elif dist > LIDAR_WARN_DIST:
                debounce = 0
            avoid_pid.reset()
            continue

        # AVOIDANCE zone — debounce
        if dist <= LIDAR_AVOID_DIST:
            debounce += 1
            if debounce < DEBOUNCE_COUNT:
                log_warn(f"Avoidance zone — debounce {debounce}/{DEBOUNCE_COUNT}  dist={dist:.1f}m")
                continue

            # v9: check avoidance timeout
            now = asyncio.get_event_loop().time()
            if avoidance_state["active"] and avoidance_start_time is not None:
                elapsed = now - avoidance_start_time
                if elapsed > AVOIDANCE_TIMEOUT_S and not avoidance_state["timeout_active"]:
                    avoidance_state["timeout_active"] = True
                    climb_alt = drone_state["abs_alt"] + CLIMB_ESCAPE_M
                    log_alert(f"AVOIDANCE TIMEOUT {elapsed:.1f}s — escalating to CLIMB +{CLIMB_ESCAPE_M}m")
                    try:
                        await drone.action.goto_location(
                            drone_state["lat"], drone_state["lon"],
                            climb_alt, float("nan")
                        )
                    except Exception as e:
                        log_warn(f"Climb escape error: {e}")
                    continue

            # BUG FIX: once climb escape is active, skip the normal detour
            # path entirely.  Without this guard, every 50 Hz tick re-issues a
            # horizontal goto_location that fights the ongoing climb command.
            if avoidance_state["timeout_active"]:
                continue

            # v9: smart left/right escape
            esc_bearing, esc_side, esc_clearance = best_escape_bearing(
                sectors, drone_state["heading"], bearing
            )
            lateral_offset = avoid_pid.compute_correction(dist)

            det_lat, det_lon = compute_avoidance_waypoint(
                drone_state["lat"],
                drone_state["lon"],
                drone_state["heading"],
                esc_bearing,
                offset_m=max(lateral_offset, AVOIDANCE_OFFSET_M)
            )

            if not avoidance_state["active"]:
                # Increment once on transition into new avoidance event only
                avoidance_state["count"] += 1
                avoidance_start_time = now

            avoidance_state["active"]      = True
            avoidance_state["last_wp"]     = (det_lat, det_lon)
            avoidance_state["escape_side"] = esc_side

            log_alert(
                f"OBSTACLE  dist={dist:.1f}m  bearing={bearing:.0f}deg  "
                f"escape={esc_side}({esc_bearing:.0f}deg)  clearance={esc_clearance:.1f}m  "
                f"[event #{avoidance_state['count']}]"
            )
            log(f"Detour WP: {det_lat:.6f}, {det_lon:.6f}")

            try:
                await drone.mission.pause_mission()
            except Exception:
                pass

            abs_alt = drone_state["abs_alt"]
            await drone.action.goto_location(det_lat, det_lon, abs_alt, float("nan"))

            hold_start = asyncio.get_event_loop().time()
            while asyncio.get_event_loop().time() - hold_start < AVOIDANCE_HOLD_S:
                await asyncio.sleep(0.1)
                if lidar_state["filtered_dist"] >= SAFE_RESUME_DIST:
                    log("Path clear during hold — exiting early")
                    break

    log("Avoidance loop exiting — mission complete")


# ══════════════════════════════════════════════════════════
#  MAIN MISSION
# ══════════════════════════════════════════════════════════
async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    banner("ARAN TECHNOLOGIES — ISR MISSION + LiDAR PID AVOIDANCE v11")
    log("Connecting to drone...")

    reconnects = 0
    async for state in drone.core.connection_state():
        if state.is_connected:
            log("Drone connected")
            break
        else:
            reconnects += 1
            drone_state["reconnects"] = reconnects
            log_warn(f"Reconnect attempt #{reconnects}")

    log("Waiting for GPS lock...")
    home_abs_alt = 0.0  # captured below
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            drone_state["gps_ok"] = True
            log("GPS locked")
            # BUG FIX: capture home absolute altitude now so Phase 3 can
            # compute the correct orbit altitude regardless of any climb-escape
            # that may have altered abs_alt during the survey.
            async for pos in drone.telemetry.position():
                home_abs_alt = pos.absolute_altitude_m
                break
            break

    # ── PRE-FLIGHT: No-fly zone fence check ──────────────
    banner("PRE-FLIGHT — NFZ BOUNDARY CHECK")
    nfz_abort = False
    for nfz in NO_FLY_ZONES:
        inside, _, dist = get_nfz_exclusion_check(HOME_LAT, HOME_LON)
        status = "  [BREACH]" if inside else f"  CLEAR — {dist:.0f}m to boundary"
        log(f"  {nfz['name']:40s}{status}")
        if inside:
            nfz_abort = True
            log_alert(f"HOME INSIDE NFZ '{nfz['name']}' — aborting mission")
    if nfz_abort:
        raise RuntimeError("Pre-flight NFZ check failed — cannot launch")
    log("All NFZ checks passed — launch authorised")

    # ── PHASE 1: Survey mission upload ───────────────────
    banner("PHASE 1 — SURVEY GRID UPLOAD")
    waypoints = generate_survey_grid()
    log(f"Generated {len(waypoints)} survey waypoints across {ROWS} rows")

    # Build loiter WP lookup: {after_wp_index: loiter_dict}
    loiter_map = {lw["after_wp_index"]: lw for lw in LOITER_WAYPOINTS}
    if loiter_map:
        log(f"Loiter waypoints: {len(loiter_map)} surveillance hold points injected")

    mission_items = []
    mission_items.append(MissionItem(
        latitude_deg=HOME_LAT, longitude_deg=HOME_LON,
        relative_altitude_m=ALTITUDE, speed_m_s=SPEED,
        is_fly_through=True, gimbal_pitch_deg=-90.0, gimbal_yaw_deg=0.0,
        camera_action=MissionItem.CameraAction.START_VIDEO,
        loiter_time_s=0.0, camera_photo_interval_s=1.0,
        acceptance_radius_m=12.0, yaw_deg=float("nan"),
        camera_photo_distance_m=0.0,
        vehicle_action=MissionItem.VehicleAction.NONE
    ))

    for i, (lat, lon) in enumerate(waypoints):
        is_last = (i == len(waypoints) - 1)
        mission_items.append(MissionItem(
            latitude_deg=lat, longitude_deg=lon,
            relative_altitude_m=ALTITUDE, speed_m_s=SPEED,
            is_fly_through=not is_last,
            gimbal_pitch_deg=-90.0, gimbal_yaw_deg=0.0,
            camera_action=MissionItem.CameraAction.STOP_VIDEO if is_last
                          else MissionItem.CameraAction.NONE,
            loiter_time_s=0.0, camera_photo_interval_s=1.0,
            acceptance_radius_m=12.0, yaw_deg=float("nan"),
            camera_photo_distance_m=0.0,
            vehicle_action=MissionItem.VehicleAction.NONE
        ))
        # Inject loiter waypoint after this survey WP if configured
        if i in loiter_map:
            lw = loiter_map[i]
            log(f"  Injecting loiter '{lw['name']}' after WP {i} "
                f"({lw['loiter_time_s']}s  pitch={lw['gimbal_pitch']}deg)")
            mission_items.append(MissionItem(
                latitude_deg=lw["lat"], longitude_deg=lw["lon"],
                relative_altitude_m=lw["altitude_m"], speed_m_s=SPEED * 0.4,
                is_fly_through=False,
                gimbal_pitch_deg=lw["gimbal_pitch"], gimbal_yaw_deg=0.0,
                camera_action=MissionItem.CameraAction.NONE,
                loiter_time_s=float(lw["loiter_time_s"]),
                camera_photo_interval_s=0.5,
                acceptance_radius_m=5.0, yaw_deg=float("nan"),
                camera_photo_distance_m=0.0,
                vehicle_action=MissionItem.VehicleAction.NONE
            ))

    mission_plan = MissionPlan(mission_items)
    await drone.mission.set_return_to_launch_after_mission(False)
    await drone.mission.upload_mission(mission_plan)
    log("Mission uploaded")

    log("Verifying mission on autopilot...")
    await asyncio.sleep(3)
    async for progress in drone.mission.mission_progress():
        log(f"Mission verified — {progress.total} waypoints confirmed")
        mission_state["wp_total"] = progress.total
        break

    gcs_thread = threading.Thread(target=start_gcs_push_loop, daemon=True)
    gcs_thread.start()
    log("GCS LiDAR push: started -> http://localhost:5000")

    lidar_task     = asyncio.create_task(lidar_reader())
    telem_task     = asyncio.create_task(telemetry_tracker(drone))
    avoidance_task = asyncio.create_task(avoidance_loop(drone))
    log("LiDAR reader / Telemetry tracker / Avoidance loop: started")

    # ── PHASE 2: Execute survey ───────────────────────────
    banner("PHASE 2 — EXECUTING ISR SURVEY (LiDAR ACTIVE)")
    log("Waiting for IMU to settle (5s)...")
    await asyncio.sleep(5)

    log("Arming drone...")
    await drone.action.arm()
    await asyncio.sleep(2)

    log("Starting survey mission...")
    await drone.mission.start_mission()

    last_wp = -1
    async for progress in drone.mission.mission_progress():
        total   = progress.total
        current = progress.current
        mission_state["wp_current"] = current
        mission_state["wp_total"]   = total
        mission_state["eta_seconds"] = _compute_eta(current, total, waypoints)

        if current != last_wp:
            last_wp = current
            pct = int((current / total) * 100) if total > 0 else 0
            bar = "=" * (pct // 5) + "-" * (20 - pct // 5)
            print(f"\r  Survey: WP {current:02d}/{total:02d}  [{bar}] {pct:3d}%"
                  f"  |  Avoided: {avoidance_state['count']}"
                  f"  ETA: {mission_state['eta_seconds']}s",
                  end="", flush=True)

        if current == total:
            print()
            log("Survey complete")
            break

    mission_done.set()
    abort_avoidance.set()
    await asyncio.sleep(2)

    banner(f"SURVEY SUMMARY — {avoidance_state['count']} obstacle(s) avoided")
    log(f"Total LiDAR scans: {lidar_state['scan_count']}")

    # ── Shared orbit helper ───────────────────────────────
    async def _do_orbit_phase(target, phase_label, home_abs_alt):
        """Fly to a target dict and orbit it using OrbitPID."""
        t_lat   = target["lat"]
        t_lon   = target["lon"]
        t_r     = target["orbit_radius_m"]
        t_spd   = target["orbit_speed_ms"]
        t_alt   = target["orbit_altitude_m"]
        t_dur   = target["orbit_duration_s"]
        t_name  = target.get("name", phase_label)

        banner(f"{phase_label} — {t_name}")
        log(f"Target: {t_lat:.6f}, {t_lon:.6f}")
        goto_alt = home_abs_alt + t_alt

        await drone.action.goto_location(t_lat, t_lon, goto_alt, float("nan"))
        log("Flying to target...")
        async for pos in drone.telemetry.position():
            dist = haversine(pos.latitude_deg, pos.longitude_deg, t_lat, t_lon)
            print(f"\r  Distance to {t_name}: {dist:.1f}m     ", end="", flush=True)
            if dist < 25.0:
                print()
                log("Arrived at target")
                break

        # In-flight NFZ check before committing to orbit
        nfz_inside, nfz_name, _ = get_nfz_exclusion_check(t_lat, t_lon)
        if nfz_inside:
            log_alert(f"TARGET INSIDE NFZ '{nfz_name}' — skipping orbit, continuing mission")
            return

        opid = OrbitPID(target_radius=t_r)
        log(f"Orbit: radius={t_r}m  speed={t_spd}m/s  duration={t_dur}s")
        await drone.action.do_orbit(
            radius_m=t_r,
            velocity_ms=t_spd,
            yaw_behavior=OrbitYawBehavior.HOLD_FRONT_TO_CIRCLE_CENTER,
            latitude_deg=t_lat,
            longitude_deg=t_lon,
            absolute_altitude_m=goto_alt
        )
        log("Orbit started — camera locked on target")
        for i in range(t_dur, 0, -1):
            async for pos in drone.telemetry.position():
                correction, current_r = opid.compute_correction(
                    pos.latitude_deg, pos.longitude_deg, t_lat, t_lon
                )
                break
            bar = "=" * (i * 20 // t_dur) + "-" * (20 - i * 20 // t_dur)
            print(f"\r  Orbit: [{bar}] {i:3d}s  radius={current_r:.1f}m  "
                  f"err={correction:+.2f}m", end="", flush=True)
            await asyncio.sleep(1)
        print()
        log(f"Orbit complete — {t_name}")

    # ── PHASE 3: Primary target orbit ─────────────────────
    primary_target = {
        "name":             "PRIMARY TARGET",
        "lat":              TARGET_LAT,
        "lon":              TARGET_LON,
        "orbit_radius_m":   ORBIT_RADIUS,
        "orbit_speed_ms":   ORBIT_SPEED,
        "orbit_altitude_m": ORBIT_ALTITUDE,
        "orbit_duration_s": ORBIT_DURATION,
    }
    await _do_orbit_phase(primary_target, "PHASE 3 — PRIMARY TARGET ACQUIRED", home_abs_alt)

    # ── PHASE 4: Secondary ISR targets (sorted by priority) ──
    sorted_secondaries = sorted(SECONDARY_TARGETS, key=lambda t: t.get("priority", 99))
    for i, sec in enumerate(sorted_secondaries, start=1):
        label = f"PHASE 4.{i} — SECONDARY TARGET {i}/{len(sorted_secondaries)}"
        await _do_orbit_phase(sec, label, home_abs_alt)

    # ── PHASE 5: RTL ──────────────────────────────────────
    banner("PHASE 5 — RETURN TO LAUNCH")
    log("Initiating RTL...")
    await drone.action.return_to_launch()

    async for state in drone.telemetry.landed_state():
        if state == LandedState.ON_GROUND:
            log("Landed safely")
            break

    lidar_task.cancel()
    telem_task.cancel()
    avoidance_task.cancel()
    await asyncio.gather(lidar_task, telem_task, avoidance_task,
                         return_exceptions=True)

    total_targets = 1 + len(SECONDARY_TARGETS)
    banner("FULL ISR + LiDAR MISSION COMPLETE v12")
    log(f"Survey -> {avoidance_state['count']} obstacle(s) avoided -> "
        f"{total_targets} target(s) acquired & orbited -> RTL")
    log("Aran Technologies — Ready for IAF MBC-3 Mission")


# ══════════════════════════════════════════════════════════
#  ENTRY POINT
# ══════════════════════════════════════════════════════════
if __name__ == "__main__":
    print()
    print("  +==================================================+")
    print("  |   ARAN TECHNOLOGIES — ISR + LiDAR PID DEMO      |")
    print("  |   v12 — Multi-Target | NFZ Fencing | Scenarios   |")
    print("  |   Gain Scheduling | Avoidance Timeout | ETA      |")
    print("  |   PX4 SITL + Gazebo Harmonic + MAVSDK Python     |")
    print("  +==================================================+")
    print()
    if not GZ_AVAILABLE:
        print("  [SIM MODE] gz-transport not installed")
        print("  Install: sudo apt install python3-gz-transport13 python3-gz-msgs10\n")
    asyncio.run(run())