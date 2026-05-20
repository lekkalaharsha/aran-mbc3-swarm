"""
Aran Technologies — ISR Mission + LiDAR MPC Avoidance  [v12-MPC-v4]
Full autonomous mission: Survey -> Obstacle Avoidance -> Target Acq -> Orbit -> RTL
"""
import asyncio
import math
import sys
import threading
import time
import requests
from mavsdk import System
import os as _os
from mavsdk.mission import MissionItem, MissionPlan
from mavsdk.action import OrbitYawBehavior
from mavsdk.telemetry import LandedState
import random, json, os

from mpc_controller import (
    AvoidanceMPC, AltitudeMPC, OrbitMPC,
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
    MAP_RESOLUTION_M, MAP_SAVE_PATH,
    RACING_MODE,
    RACING_LIDAR_WARN_DIST, RACING_LIDAR_AVOID_DIST,
    RACING_SAFE_RESUME_DIST, RACING_AVOIDANCE_OFFSET,
    RACING_MAX_SPEED, RACING_MAX_ACCEL,
    RACING_ACCEPTANCE_RADIUS,
    generate_survey_grid, generate_all_sweeps, get_nfz_exclusion_check,
)
from mapping_3d import MapBuilder

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
# Radar panel topics for mbc3_radar_drone (gz-transport).
# Each panel publishes on the explicit topic set in the xacro.
# Override via ISR_RADAR_TOPIC_A/B/C/D/E/F env vars for non-default worlds.
# 6 panels at 60° spacing × 60° H-FOV = seamless 360° coverage, zero gaps.
# Panel yaw offsets in body frame (CCW from forward):
#   A=0°, B=60°, C=120°, D=180°, E=240°, F=300°
_RADAR_TOPICS = {
    "A": _os.environ.get("ISR_RADAR_TOPIC_A", "/radar_A/scan"),
    "B": _os.environ.get("ISR_RADAR_TOPIC_B", "/radar_B/scan"),
    "C": _os.environ.get("ISR_RADAR_TOPIC_C", "/radar_C/scan"),
    "D": _os.environ.get("ISR_RADAR_TOPIC_D", "/radar_D/scan"),
    "E": _os.environ.get("ISR_RADAR_TOPIC_E", "/radar_E/scan"),
    "F": _os.environ.get("ISR_RADAR_TOPIC_F", "/radar_F/scan"),
}
_PANEL_YAWS_DEG = {"A": 0.0, "B": 60.0, "C": 120.0, "D": 180.0, "E": 240.0, "F": 300.0}

# Racing mode overrides: scale avoidance distances for 30–60 m/s operation.
# At these speeds the drone covers 30 m in under 1 second — standard 15/25 m
# thresholds leave no room to react.  Values come from RACING_* in mission_config.
if RACING_MODE:
    LIDAR_WARN_DIST    = RACING_LIDAR_WARN_DIST   # 40 m
    LIDAR_AVOID_DIST   = RACING_LIDAR_AVOID_DIST  # 25 m
    AVOIDANCE_OFFSET_M = RACING_AVOIDANCE_OFFSET  # 80 m
    SAFE_RESUME_DIST   = RACING_SAFE_RESUME_DIST  # 35 m
else:
    LIDAR_WARN_DIST    = 25.0
    LIDAR_AVOID_DIST   = 15.0
    AVOIDANCE_OFFSET_M = 50.0
    SAFE_RESUME_DIST   = 22.0

# BUG FIX: LIDAR_POLL_HZ was 200 in v12-MPC-v1.  At 200 Hz the avoidance loop
# sleeps only 5 ms per tick, hammering the asyncio event loop and starving the
# four telemetry coroutines.  DEBOUNCE_COUNT=3 was originally tuned for 50 Hz
# (3 × 20 ms = 60 ms debounce window); at 200 Hz this collapsed to 15 ms,
# far too short to suppress sensor noise spikes.  Reverted to 50 Hz.
LIDAR_POLL_HZ       = 50

AVOIDANCE_HOLD_S    = 2.0

DEBOUNCE_COUNT      = 3       # 3 × (1/50 Hz) = 60 ms debounce window
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
    "active":              False,
    "count":               0,
    "last_wp":             None,
    "escape_side":         "---",
    "timeout_active":      False,
    # Scenario PID override: written by lidar_sim_reader when a scenario
    # carries pid_gains; consumed once by avoidance_loop on its first tick.
    "scenario_pid_override": None,
}

drone_state = {
    "lat":         HOME_LAT,
    "lon":         HOME_LON,
    "alt":         0.0,
    "abs_alt":     0.0,
    "heading":     0.0,
    "groundspeed": 0.0,
    "vn_ms":       0.0,   # NED north velocity component (m/s)
    "ve_ms":       0.0,   # NED east velocity component (m/s)
    "gps_ok":      False,
    "reconnects":  0,
}

mission_state = {
    "wp_current":    0,
    "wp_total":      0,
    "eta_seconds":   None,
    # Written by run() at each mission phase transition so push_to_gcs can
    # forward it to the GCS; lets the dashboard distinguish primary-orbit
    # LOITER from SEC-1/2/3 orbits (all report HOLD in PX4 flight mode).
    "mission_phase": "STANDBY",
}

mission_done    = asyncio.Event()
abort_avoidance = asyncio.Event()

# BUG-B FIX: MapBuilder was imported but never instantiated — 3D mapping was
# completely non-functional.  Single instance shared by both lidar readers.
map_builder     = MapBuilder()

# Dynamic runtime state — populated by push_to_gcs() from GCS response.
# _dyn_lock guards access across the daemon push thread and asyncio coroutines.
_dyn_lock     = threading.Lock()
dynamic_state = {
    "pending_events": [],   # [{expire_at, bearing_deg, dist_m}] for lidar_sim_reader
}


# ══════════════════════════════════════════════════════════
#  GCS PUSH
# ══════════════════════════════════════════════════════════
def _safe_float(v, fallback=0.0):
    """Replace inf/nan with fallback — json.dumps rejects non-finite floats."""
    try:
        return fallback if (math.isinf(v) or math.isnan(v)) else float(v)
    except Exception:
        return fallback


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
            "nearest_dist":     _safe_float(lidar_state["nearest_dist"], 9999.9),
            "nearest_bearing":  _safe_float(lidar_state["nearest_bearing"], 0.0),
            "scan_count":       lidar_state["scan_count"],
            "avoidance_active": avoidance_state["active"],
            "avoidance_count":  avoidance_state["count"],
            "escape_side":      avoidance_state["escape_side"],
            "timeout_active":   avoidance_state["timeout_active"],
            "detour_lat":       last_wp_snap[0] if last_wp_snap else None,
            "detour_lon":       last_wp_snap[1] if last_wp_snap else None,
            # BUG FIX: sectors were missing from the payload — the GCS
            # sector-clearance overlay was permanently frozen at init values.
            "sectors":          [_safe_float(s, 9999.9) for s in lidar_state["sectors"]],
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
            # BUG FIX (telemetry_web.py): PX4 reports HOLD for every do_orbit
            # call, so the GCS flight-mode mapper cannot distinguish primary vs
            # secondary orbits.  Push the explicit phase string so the GCS
            # phase list and target queue correctly advance through SEC-1/2/3.
            "mission_phase":    mission_state["mission_phase"],
            "map_stats":        map_builder.stats(),
            # ASP tracks — extracted from fused radar scan
            "asp_tracks":       _extract_asp_tracks(
                                    lidar_state["raw_ranges"],
                                    drone_state["lat"],
                                    drone_state["lon"],
                                    drone_state["alt"],
                                ) if lidar_state["raw_ranges"] else [],
            "asp_drone_id":     "DRONE-L",
        }
        resp = requests.post(GCS_URL, json=payload, timeout=0.2)
        if resp.ok:
            cmds = resp.json().get("commands", {})
            if any(cmds.get(k) for k in ("nfz_queue", "target_queue",
                                          "config_updates", "event_queue")):
                _apply_dynamic_commands(cmds)
    except Exception:
        pass


def _apply_dynamic_commands(cmds):
    """Apply dynamic commands received from GCS in the /lidar_update response.

    Called from push_to_gcs() (daemon thread) at most once per 0.2s push cycle.
    Mutations to NO_FLY_ZONES and SECONDARY_TARGETS are safe because both are
    module-level lists imported by reference — appends are visible everywhere.
    """
    global LIDAR_WARN_DIST, LIDAR_AVOID_DIST, AVOIDANCE_OFFSET_M, SAFE_RESUME_DIST

    for nfz in cmds.get("nfz_queue", []):
        NO_FLY_ZONES.append(nfz)
        log(f"[DYN] NFZ added: {nfz.get('name','?')}  "
            f"lat={nfz.get('lat',0):.4f} lon={nfz.get('lon',0):.4f} "
            f"r={nfz.get('radius_m',0):.0f}m")

    for target in cmds.get("target_queue", []):
        SECONDARY_TARGETS.append(target)
        log(f"[DYN] Target queued: {target.get('name','?')}  "
            f"lat={target.get('lat',0):.4f} lon={target.get('lon',0):.4f}  "
            f"priority={target.get('priority',99)}")

    cfg = cmds.get("config_updates", {})
    if cfg:
        if "LIDAR_WARN_DIST"  in cfg: LIDAR_WARN_DIST    = float(cfg["LIDAR_WARN_DIST"])
        if "LIDAR_AVOID_DIST" in cfg: LIDAR_AVOID_DIST   = float(cfg["LIDAR_AVOID_DIST"])
        if "AVOIDANCE_OFFSET" in cfg: AVOIDANCE_OFFSET_M = float(cfg["AVOIDANCE_OFFSET"])
        if "SAFE_RESUME_DIST" in cfg: SAFE_RESUME_DIST   = float(cfg["SAFE_RESUME_DIST"])
        log(f"[DYN] Config patched: {cfg}")

    for ev in cmds.get("event_queue", []):
        expire_at = time.time() + float(ev.get("duration_s", 5.0))
        with _dyn_lock:
            dynamic_state["pending_events"].append({
                "expire_at":   expire_at,
                "bearing_deg": float(ev.get("bearing_deg", 0.0)),
                "dist_m":      float(ev.get("dist_m", 10.0)),
            })
        log(f"[DYN] Event injected: bearing={ev.get('bearing_deg',0):.0f}deg  "
            f"dist={ev.get('dist_m',10):.1f}m  duration={ev.get('duration_s',5):.1f}s")


def start_gcs_push_loop():
    while True:
        push_to_gcs()
        # NEW-3: reduced from 0.2s (5Hz) to 0.4s (2.5Hz).
        # requests.post() from this daemon thread competes with asyncio telemetry
        # callbacks — cutting push rate halves the contention.
        time.sleep(0.4)


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


def _extract_asp_tracks(raw_ranges, drone_lat, drone_lon, drone_alt,
                        min_range=5.0, max_range=4900.0, gap_deg=5):
    """Cluster 360° fused scan into ASP tracks.

    Groups consecutive valid returns into clusters, computes centroid
    bearing + mean range per cluster, converts to lat/lon using haversine.
    Returns list of track dicts for /asp_update payload.
    """
    tracks = []
    in_cluster = False
    cluster_bearings = []
    cluster_ranges   = []
    track_id = 1

    for deg in range(361):  # +1 to close wrap-around
        idx = deg % 360
        r   = raw_ranges[idx] if raw_ranges else float('inf')
        valid = (min_range < r < max_range
                 and not math.isinf(r) and not math.isnan(r))

        if valid:
            if not in_cluster:
                in_cluster = True
                cluster_bearings = []
                cluster_ranges   = []
            cluster_bearings.append(deg)
            cluster_ranges.append(r)
        else:
            if in_cluster and len(cluster_ranges) >= 2:
                bearing = sum(cluster_bearings) / len(cluster_bearings)
                rng     = sum(cluster_ranges)   / len(cluster_ranges)
                # Convert bearing + range to lat/lon
                brng_rad = math.radians(bearing)
                R_EARTH  = 6371000.0
                d_lat = (rng * math.cos(brng_rad)) / R_EARTH
                d_lon = (rng * math.sin(brng_rad)) / (
                    R_EARTH * math.cos(math.radians(drone_lat)))
                t_lat = drone_lat + math.degrees(d_lat)
                t_lon = drone_lon + math.degrees(d_lon)
                tracks.append({
                    "id":           f"TRK-{track_id:03d}",
                    "lat":          round(t_lat, 6),
                    "lon":          round(t_lon, 6),
                    "range_m":      round(rng, 1),
                    "bearing_deg":  round(bearing % 360, 1),
                    "alt_m":        round(drone_alt, 1),
                    "velocity_ms":  0.0,
                    "confidence":   min(1.0, len(cluster_ranges) / 10.0),
                    "width_deg":    len(cluster_bearings),
                    "timestamp":    time.time(),
                })
                track_id += 1
            in_cluster = False

    return tracks


def _bearing_to_nearest(ranges, angle_min, angle_increment):
    """Return (min_dist_m, world_bearing_deg) to the nearest valid obstacle.
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
    """
    if not waypoints:
        return 0
    # survey ceiling: home_wp (index 0) + N survey WPs
    survey_wp_total = len(waypoints) + 1
    if wp_current >= survey_wp_total:
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
    # Clamp min speed to SPEED to avoid huge ETA during loiter (groundspeed ≈ 0)
    spd = max(drone_state.get("groundspeed") or SPEED, SPEED)
    return int(total_dist / spd) if spd > 0 else None


# ══════════════════════════════════════════════════════════
#  LIDAR READER
# ══════════════════════════════════════════════════════════
LIDAR_TOPIC_DISCOVER_S = 8.0   # seconds to wait for first scan before discovery


def _discover_radar_topics():
    """
    Run 'gz topic -l' and return dict of panel_id → topic for any
    radar_A/B/C/D scan topics found.  Fallback when default topics
    yield no messages (e.g. non-default world name).
    """
    import subprocess
    found = {}
    try:
        result = subprocess.run(
            ["gz", "topic", "-l"],
            capture_output=True, text=True, timeout=5
        )
        for line in result.stdout.splitlines():
            for pid in ("A", "B", "C", "D", "E", "F"):
                if f"radar_{pid}/scan" in line and pid not in found:
                    found[pid] = line.strip()
    except Exception:
        pass
    return found


async def radar_gz_reader():
    """Subscribe to 4 FMCW radar panels and fuse into 360° range array.

    Panels A/B/C/D each cover ±27.5° (55° H-FOV). Combined they give
    4×55°=220° of coverage with 35° gaps between panels. Gap slots stay
    at float('inf') — avoidance treats them as clear, same as a 2D LiDAR
    with no return. Fusion takes the minimum range across overlapping bins.
    """
    topics = dict(_RADAR_TOPICS)
    node = Node()
    loop = asyncio.get_running_loop()
    notify_queue = asyncio.Queue(maxsize=1)
    panel_data = {}   # panel_id → latest LaserScan msg (asyncio-thread only)

    def make_callback(panel_id):
        def on_scan(msg):
            def _update():
                panel_data[panel_id] = msg
                while not notify_queue.empty():
                    try: notify_queue.get_nowait()
                    except Exception: break
                try: notify_queue.put_nowait(panel_id)
                except Exception: pass
            loop.call_soon_threadsafe(_update)
        return on_scan

    for pid, topic in topics.items():
        node.subscribe(LaserScan, topic, make_callback(pid))
        log(f"Radar: panel {pid} → {topic}")
    log("Radar 360: all panel subscribers active")

    got_first = False
    discovery_done = False

    while True:
        try:
            await asyncio.wait_for(notify_queue.get(), timeout=LIDAR_TOPIC_DISCOVER_S)
        except asyncio.TimeoutError:
            if not got_first and not discovery_done:
                log_warn(f"Radar: no scan after {LIDAR_TOPIC_DISCOVER_S:.0f}s "
                         "— running gz topic discovery...")
                discovered = await asyncio.get_running_loop().run_in_executor(
                    None, _discover_radar_topics
                )
                discovery_done = True
                for pid, dtopic in discovered.items():
                    if dtopic != topics.get(pid):
                        log(f"Radar: panel {pid} discovered → {dtopic}")
                        topics[pid] = dtopic
                        node.subscribe(LaserScan, dtopic, make_callback(pid))
                if not discovered:
                    log_warn("Radar: discovery found no radar topics — "
                             "avoidance disabled until scans arrive")
            continue

        got_first = True

        # Fuse all available panels into a 360-element range array (1° bins)
        fused = [float("inf")] * 360
        for pid, msg in panel_data.items():
            yaw_rad = math.radians(_PANEL_YAWS_DEG[pid])
            for i, r in enumerate(msg.ranges):
                panel_bearing = msg.angle_min + i * msg.angle_step
                abs_deg = math.degrees(yaw_rad + panel_bearing) % 360
                idx = int(round(abs_deg)) % 360
                if r < fused[idx]:
                    fused[idx] = r

        angle_min  = -math.pi
        angle_step = math.radians(1.0)
        min_dist, bearing = _bearing_to_nearest(fused, angle_min, angle_step)
        sectors  = _compute_sectors(fused, angle_min, angle_step)
        filtered = _median_filter(min_dist)
        lidar_state.update({
            "nearest_dist":    min_dist,
            "nearest_bearing": bearing,
            "filtered_dist":   filtered,
            "raw_ranges":      fused,
            "sectors":         sectors,
        })
        lidar_state["scan_count"] += 1
        map_builder.ingest(fused, angle_min, angle_step, drone_state)


async def lidar_sim_reader():
    """
    Simulated 360deg LiDAR reader for use when gz-transport is not available.

    instead of a single hardcoded obstacle, the sim reader can
    load and replay any scenario from scenarios.json.  Set SIM_SCENARIO to a
    scenario name to run it, or leave as None for the legacy single-obstacle
    behaviour.  Multiple simultaneous events are supported by merging all
    active events into a combined range array each tick.

    Also checks NO_FLY_ZONES each tick and synthesises a 360deg wall of range=2m
    around the drone if it is inside a no-fly zone, forcing immediate avoidance.
    """
    SIM_SCENARIO = _os.environ.get("ISR_SIM_SCENARIO") or None

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

                    avoidance_state["scenario_pid_override"] = scenario_pid
                    log(f"LiDAR-SIM: scenario PID overrides — "
                        f"Kp={scenario_pid['kp']} Ki={scenario_pid['ki']} Kd={scenario_pid['kd']}")
            else:
                log_warn(f"LiDAR-SIM: scenario '{SIM_SCENARIO}' not found — using legacy sim")
        except Exception as e:
            log_warn(f"LiDAR-SIM: could not load scenarios.json — {e}")

    sim_start = asyncio.get_running_loop().time()

    while True:
        now_rel = asyncio.get_running_loop().time() - sim_start
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

        # ── dynamic event injection (POST /inject_event via GCS) ────
        # BUG-4 FIX: replaced threading.Lock() with GIL-atomic list operations.
        # lidar_sim_reader is an asyncio coroutine — acquiring a threading.Lock()
        # blocks the OS thread synchronously, freezing the entire event loop
        # (telemetry, avoidance, lidar) if _apply_dynamic_commands() holds _dyn_lock.
        # list.copy() and dict-key assignment are each GIL-atomic in CPython,
        # making the lock unnecessary for these sub-millisecond operations.
        now_wall     = time.time()
        snapshot     = dynamic_state["pending_events"].copy()
        still_active = [e for e in snapshot if e["expire_at"] > now_wall]
        dynamic_state["pending_events"] = still_active
        active_dyn   = still_active
        for ev in active_dyn:
            ev_dist  = ev["dist_m"] + random.uniform(-0.2, 0.2)
            center   = int(ev["bearing_deg"]) % 360
            for offset in range(-6, 7):
                idx = (center + offset) % 360
                if ev_dist < fake_ranges[idx]:
                    fake_ranges[idx] = ev_dist

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
        map_builder.ingest(fake_ranges, 0.0, ANGLE_INC, drone_state)
        await asyncio.sleep(1.0 / LIDAR_POLL_HZ)


async def lidar_reader():
    if GZ_AVAILABLE:
        await radar_gz_reader()
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
            vn = v.north_m_s
            ve = v.east_m_s
            drone_state["groundspeed"] = math.sqrt(vn ** 2 + ve ** 2)
            # BUG-5 FIX (propagated): store individual NED velocity components so
            # AltitudeMPC and AvoidanceMPC can build accurate initial states.
            drone_state["vn_ms"] = vn
            drone_state["ve_ms"] = ve

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
    avoid_mpc = AvoidanceMPC(safe_distance=LIDAR_AVOID_DIST + 2.0)
    interval  = 1.0 / LIDAR_POLL_HZ
    debounce  = 0
    avoidance_start_time = None

    _override_applied = False

    log(f"Avoidance loop started — 360deg scan {LIDAR_POLL_HZ}Hz "
        f"debounce={DEBOUNCE_COUNT} timeout={AVOIDANCE_TIMEOUT_S}s")

    while not mission_done.is_set():
        await asyncio.sleep(interval)
        if abort_avoidance.is_set():
            continue

        dist    = lidar_state["filtered_dist"]
        bearing = lidar_state["nearest_bearing"]
        sectors = lidar_state["sectors"]

        # Consume scenario PID override lazily — lidar_sim_reader may write it
        # after this loop starts (async race), so we check each tick until applied.
        if not _override_applied:
            override = avoidance_state.pop("scenario_pid_override", None)
            if override:
                _override_applied = True
                avoid_mpc.set_gains(
                    kp=override.get("kp"), ki=override.get("ki"), kd=override.get("kd")
                )
                log(f"Avoidance MPC: scenario gains noted — "
                    f"Kp={override.get('kp')} Ki={override.get('ki')} Kd={override.get('kd')} "
                    f"(NOTE: kp/ki/kd are stub-only in MPC; cost weights unchanged. "
                    f"To tune MPC aggressiveness edit W_OBS_SLOW/W_OBS_FAST in mpc_controller.py)")

        avoid_mpc.update_speed(drone_state["groundspeed"])

        # WARNING zone
        if LIDAR_AVOID_DIST < dist <= LIDAR_WARN_DIST:
            log_warn(f"LiDAR WARNING  obstacle={dist:.1f}m  bearing={bearing:.0f}deg")

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
            avoid_mpc.reset()
            continue

        # AVOIDANCE zone — debounce
        if dist <= LIDAR_AVOID_DIST:
            debounce += 1
            if debounce < DEBOUNCE_COUNT:
                log_warn(f"Avoidance zone — debounce {debounce}/{DEBOUNCE_COUNT}  dist={dist:.1f}m")
                continue

            # v9: check avoidance timeout
            now = asyncio.get_running_loop().time()
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


            if avoidance_state["timeout_active"]:
                continue

            # v9: smart left/right escape
            esc_bearing, esc_side, esc_clearance = best_escape_bearing(
                sectors, drone_state["heading"], bearing
            )

            lateral_offset = avoid_mpc.compute_correction(
                dist,
                drone_heading_deg = drone_state["heading"],
                vn_ms             = drone_state["vn_ms"],
                ve_ms             = drone_state["ve_ms"],
            )

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

            hold_start = asyncio.get_running_loop().time()
            while asyncio.get_running_loop().time() - hold_start < AVOIDANCE_HOLD_S:
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
    # FIX: udp:// (sender-binds mode) conflicts with PX4 SITL when a prior
    # run left the socket open — "bind error: Address in use" silently breaks
    # the MAVLink MISSION_COUNT exchange and causes upload_mission() to confirm
    # 0 items on every attempt.  udpin:// (receiver-binds mode) does not bind
    # a sender socket, so it survives restarts without the port-in-use race.
    # NOTE: udpin:// requires an explicit host — bare ":14540" is invalid.
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    banner("ARAN TECHNOLOGIES — ISR MISSION + LiDAR MPC AVOIDANCE v12-MPC-v5")
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

    # Throttle telemetry rates — default PX4 SITL rates flood the MAVSDK
    # callback queue ("User callback queue slow") causing arm/mission failures.
    for _rate_call, _hz in [
        (drone.telemetry.set_rate_position,            5.0),
        (drone.telemetry.set_rate_velocity_ned,        5.0),
        (drone.telemetry.set_rate_health,              2.0),
        (drone.telemetry.set_rate_landed_state,        2.0),
        (drone.telemetry.set_rate_in_air,              2.0),
        (drone.telemetry.set_rate_attitude_euler,      5.0),
        (drone.telemetry.set_rate_attitude_quaternion, 5.0),
        (drone.telemetry.set_rate_imu,                 5.0),
    ]:
        try:
            await _rate_call(_hz)
        except Exception:
            pass

    log("Waiting for GPS lock...")
    home_abs_alt = 0.0  # captured below
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            drone_state["gps_ok"] = True
            log("GPS locked")

            async for pos in drone.telemetry.position():
                home_abs_alt = pos.absolute_altitude_m
                break
            break

    # ── PRE-FLIGHT: No-fly zone fence check ──────────────
    banner("PRE-FLIGHT — NFZ BOUNDARY CHECK")
    nfz_abort = False

    for nfz in NO_FLY_ZONES:
        dist = haversine(HOME_LAT, HOME_LON, nfz["lat"], nfz["lon"])
        inside = dist < nfz["radius_m"]
        status = "  [BREACH]" if inside else f"  CLEAR — {dist:.0f}m to boundary"
        log(f"  {nfz['name']:40s}{status}")
        if inside:
            nfz_abort = True
            log_alert(f"HOME INSIDE NFZ '{nfz['name']}' — aborting mission")
    if nfz_abort:
        raise RuntimeError("Pre-flight NFZ check failed — cannot launch")
    log("All NFZ checks passed — launch authorised")

    # ── Pre-upload channel health check ──────────────────────────────
    # Verify the MAVLink mission channel is actually responding before we build
    # or upload anything.  If upload_mission() fires over a broken UDP socket
    # (e.g. "bind error: Address in use" from a stale prior run), it appears to
    # succeed from MAVSDK's side but PX4 never receives the MISSION_COUNT frame,
    # so mission_progress() opens and closes immediately at total=0 every time.
    # download_mission() does a synchronous round-trip — if it times out the
    # channel is broken and we fail fast with an actionable error message rather
    # than burning all 5 upload attempts.
    CHANNEL_CHECK_TIMEOUT_S = 12.0
    log("Verifying mission upload channel (MAVLink round-trip)…")
    try:
        dl_result = await asyncio.wait_for(
            drone.mission.download_mission(),
            timeout=CHANNEL_CHECK_TIMEOUT_S,
        )
        existing = len(dl_result.mission_items)
        log(f"  Upload channel OK — autopilot reports {existing} existing item(s)")
    except asyncio.TimeoutError:
        raise RuntimeError(
            "Mission upload channel not responding before first upload attempt.\n"
            "Most likely cause: stale UDP socket from a previous run.\n"
            "Fix:  pkill -f isr_lidar_mpc  OR  use  ./launch.sh  (handles cleanup automatically)"
        )
    except Exception as e:
        log_warn(f"  download_mission() check raised {type(e).__name__}: {e} — continuing anyway")

    # ── Wait for PX4 mission manager to finish initialising ──────────
    # After GPS lock the autopilot still needs a few seconds to set up its
    # internal mission storage.  Uploading immediately causes the MAVLink
    # MISSION_COUNT → MISSION_ITEM_INT exchange to race with that init,
    # producing a progress stream that closes at total=0 on every attempt.
    # Raised to 15 s (from 5 s in v4) — v5 field testing showed slow-boot
    # SITL hosts need the extra headroom before the mission store accepts writes.
    MISSION_MGR_SETTLE_S = 15.0
    log(f"Waiting {MISSION_MGR_SETTLE_S:.0f}s for PX4 mission manager to initialise…")
    await asyncio.sleep(MISSION_MGR_SETTLE_S)

    # Poll for responsiveness (up to 6 × 2 s = 12 s extra) before uploading.
    # download_mission() does a synchronous MAVLink round-trip — success means
    # the mission store is accepting commands.
    MAX_READY_ATTEMPTS = 6
    log("Verifying mission manager responsiveness…")
    for attempt in range(1, MAX_READY_ATTEMPTS + 1):
        try:
            dl_result = await drone.mission.download_mission()
            existing  = len(dl_result.mission_items)
            log(f"  Mission manager responsive (attempt {attempt}/{MAX_READY_ATTEMPTS}) "
                f"— {existing} existing item(s) on autopilot")
            break
        except Exception as e:
            if attempt == MAX_READY_ATTEMPTS:
                raise RuntimeError(
                    f"PX4 mission manager not responsive after {MISSION_MGR_SETTLE_S:.0f}s "
                    f"+ {MAX_READY_ATTEMPTS} checks: {e}"
                ) from e
            log_warn(f"  Mission manager not ready "
                     f"(attempt {attempt}/{MAX_READY_ATTEMPTS}) — retrying in 2s")
            await asyncio.sleep(2)

    # Clear any stale mission so PX4's item-count starts from zero — avoids
    # a mismatch where a previously uploaded partial mission confuses the
    # confirmed_total check in the upload loop below.
    try:
        await drone.mission.clear_mission()
        log("Cleared any previous mission from autopilot")
        await asyncio.sleep(1)
    except Exception as e:
        log_warn(f"Could not clear previous mission (non-critical): {e}")

    # ── PHASE 1: Survey mission upload ───────────────────
    banner("PHASE 1 — SURVEY GRID UPLOAD")
    waypoints = generate_survey_grid()
    log(f"Generated {len(waypoints)} survey waypoints across {ROWS} rows")

    # Build loiter WP lookup: {after_wp_index: loiter_dict}
    # B11 FIX: loiter WPs are calibrated for ISR (30m AGL) with fixed altitudes of
    # 40-50m. At MBC3_MODE cruise (500m), those altitudes would force a 450-460m
    # descent + climb per loiter point (~22 min extra). Skip them in MBC3_MODE.
    if ALTITUDE >= 200.0:
        loiter_map = {}
        log("MBC3_MODE: loiter WPs skipped — ISR altitudes (40-50m) incompatible with 500m cruise")
    else:
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
        acceptance_radius_m=RACING_ACCEPTANCE_RADIUS if RACING_MODE else 12.0, yaw_deg=float("nan"),
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
            acceptance_radius_m=RACING_ACCEPTANCE_RADIUS if RACING_MODE else 12.0, yaw_deg=float("nan"),
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

    # ── Upload with retry ─────────────────────────────────────────────
    # Two-stage confirmation on every attempt:
    #   Stage 1 — download_mission() round-trip (fast, deterministic).
    #             Added in v5: if the item count matches we trust it immediately
    #             without waiting for the progress stream to stabilise.
    #   Stage 2 — mission_progress() stream (fallback for older MAVSDK builds
    #             where download_mission() may not reflect the latest upload).
    # BUG FIX (v4): settle window and retry ceiling raised so slow SITL hosts
    # don't exhaust attempts before the mission store finishes initialising.
    MAX_UPLOAD_ATTEMPTS = 8     # raised from 5 — slow SITL hosts need headroom
    UPLOAD_SETTLE_S     = 10.0  # raised from 5.0 — more time for PX4 store init
    RETRY_BACKOFF_S     = 5.0   # raised from 3.0 — longer gap between attempts
    expected_total      = len(mission_items)
    confirmed_total     = 0

    for attempt in range(1, MAX_UPLOAD_ATTEMPTS + 1):
        log(f"Uploading mission — attempt {attempt}/{MAX_UPLOAD_ATTEMPTS} "
            f"({expected_total} items)...")
        try:
            await drone.mission.upload_mission(mission_plan)
            log("  upload_mission() call completed")
        except Exception as e:
            log_warn(f"  upload_mission() raised {type(e).__name__}: {e}")
            if attempt == MAX_UPLOAD_ATTEMPTS:
                raise RuntimeError(
                    f"Mission upload API call failed on all {MAX_UPLOAD_ATTEMPTS} attempts"
                ) from e
            await asyncio.sleep(RETRY_BACKOFF_S)
            continue

        # Give PX4 time to ACK and begin storing items before we verify.
        await asyncio.sleep(UPLOAD_SETTLE_S)

        # ── Stage 1: download-back verification (v5 addition) ────────
        # Faster and more deterministic than the progress stream.
        download_ok = False
        try:
            dl = await asyncio.wait_for(
                drone.mission.download_mission(),
                timeout=10.0,
            )
            confirmed_total = len(dl.mission_items)
            if confirmed_total == expected_total:
                log(f"  Download-back confirmed {confirmed_total}/{expected_total} items — upload OK")
                download_ok = True
            else:
                log_warn(f"  Download-back mismatch: "
                         f"uploaded {expected_total}, autopilot stores {confirmed_total}")
                # Clear stale partial mission before next attempt
                if attempt < MAX_UPLOAD_ATTEMPTS:
                    try:
                        await drone.mission.clear_mission()
                        log("  Cleared partial mission — will retry upload")
                        await asyncio.sleep(1)
                    except Exception as ce:
                        log_warn(f"  clear_mission() failed: {ce}")
        except asyncio.TimeoutError:
            log_warn("  download_mission() timed out during verification — falling back to progress stream")
        except Exception as e:
            log_warn(f"  download_mission() verification failed ({type(e).__name__}: {e}) "
                     f"— falling back to progress stream")

        if download_ok:
            break

        # ── Stage 2: mission_progress() stream (original fallback) ───
        confirmed_total = 0
        first_ok_time   = None
        CONFIRM_TIMEOUT = 20.0
        STABLE_WINDOW_S = 2.0
        t_start = asyncio.get_running_loop().time()

        progress_aiter = drone.mission.mission_progress().__aiter__()
        upload_ok = False

        while True:
            elapsed = asyncio.get_running_loop().time() - t_start
            if elapsed >= CONFIRM_TIMEOUT:
                log_warn(f"  Confirmation timeout at {confirmed_total}/{expected_total} items")
                break

            try:
                progress = await asyncio.wait_for(
                    progress_aiter.__anext__(), timeout=STABLE_WINDOW_S
                )
                confirmed_total = progress.total
                now = asyncio.get_running_loop().time()

                if confirmed_total == expected_total:
                    if first_ok_time is None:
                        first_ok_time = now
                    stable_s = now - first_ok_time
                    log(f"  Mission items on autopilot: {confirmed_total}/{expected_total} "
                        f"(stable {stable_s:.1f}s / {STABLE_WINDOW_S}s)")
                    if stable_s >= STABLE_WINDOW_S:
                        upload_ok = True
                        break
                else:
                    first_ok_time = None
                    log(f"  Mission items on autopilot: {confirmed_total}/{expected_total} "
                        f"— fluctuating, waiting...")

            except asyncio.TimeoutError:
                if confirmed_total == expected_total:
                    log(f"  mission_progress stream settled at "
                        f"{confirmed_total}/{expected_total} — confirmed")
                    upload_ok = True
                    break
                log(f"  Still waiting for mission_progress frame… "
                    f"({elapsed:.0f}s / {CONFIRM_TIMEOUT:.0f}s)")

            except StopAsyncIteration:
                # MAVSDK closes the mission_progress stream once PX4 has
                # fully stored the mission.  StopAsyncIteration is NOT a
                # subclass of Exception and leaks through asyncio.wait_for
                # — it must be caught explicitly.
                if confirmed_total == expected_total:
                    log(f"  mission_progress stream closed — "
                        f"{confirmed_total}/{expected_total} items confirmed")
                    upload_ok = True
                    break
                # Stream closed at 0 items — this is the SITL timing race.
                # Fall through to retry the upload.
                log_warn(f"  mission_progress closed at {confirmed_total}/{expected_total} "
                         f"— PX4 mission manager not ready yet (attempt {attempt})")
                break

        if upload_ok:
            log(f"Mission verified — {confirmed_total} waypoints confirmed by autopilot")
            break

        if attempt < MAX_UPLOAD_ATTEMPTS:
            wait_s = UPLOAD_SETTLE_S + (attempt * RETRY_BACKOFF_S)
            log(f"  Retrying upload in {wait_s:.0f}s…  (attempt {attempt} of {MAX_UPLOAD_ATTEMPTS})")
            await asyncio.sleep(wait_s)
        else:
            raise RuntimeError(
                f"Mission upload failed after {MAX_UPLOAD_ATTEMPTS} attempts — "
                f"autopilot confirmed {confirmed_total}/{expected_total} items. "
                f"Check PX4 SITL connection and restart."
            )

    mission_state["wp_total"] = confirmed_total

    gcs_thread = threading.Thread(target=start_gcs_push_loop, daemon=True)
    gcs_thread.start()
    log("GCS LiDAR push: started -> http://localhost:5000")

    lidar_task     = asyncio.create_task(lidar_reader())
    telem_task     = asyncio.create_task(telemetry_tracker(drone))
    avoidance_task = asyncio.create_task(avoidance_loop(drone))
    log("LiDAR reader / Telemetry tracker / Avoidance loop: started")

    # ── PHASE 2: Execute survey ───────────────────────────
    banner("PHASE 2 — EXECUTING ISR SURVEY (LiDAR ACTIVE)")
    mission_state["mission_phase"] = "SURVEY"

    log("Waiting for pre-arm checks to pass...")
    ARM_TIMEOUT = 60.0
    t_arm = asyncio.get_running_loop().time()
    async for health in drone.telemetry.health():
        elapsed = asyncio.get_running_loop().time() - t_arm

        gps_ok     = health.is_global_position_ok
        home_ok    = health.is_home_position_ok
        armable_ok = health.is_armable
        try:
            local_ok = health.is_local_position_ok
        except AttributeError:
            local_ok = True

        log(f"  Health: gps={gps_ok}  home={home_ok}  local={local_ok}"
            f"  armable={armable_ok}  ({elapsed:.1f}s)")

        if gps_ok and home_ok and local_ok and armable_ok:
            break
        if elapsed > ARM_TIMEOUT:
            raise RuntimeError(
                f"Pre-arm health check timeout after {ARM_TIMEOUT:.0f}s — "
                f"gps={gps_ok} home={home_ok} local={local_ok} armable={armable_ok}. "
                f"Check SITL preflight checks."
            )
        await asyncio.sleep(0.5)
    log("Pre-arm checks passed")

    log("Arming drone...")
    for _arm_attempt in range(1, 4):
        try:
            await drone.action.arm()
            break
        except Exception as _arm_err:
            if _arm_attempt == 3:
                raise
            log_warn(f"  Arm attempt {_arm_attempt} failed: {_arm_err} — retrying in 2s")
            await asyncio.sleep(2.0)

    # ── Set takeoff altitude ──
    # NEW-2 FIX: param.Param(drone) uses an internal _channel attribute removed
    # in MAVSDK ≥ 1.4. drone.action.set_takeoff_altitude() uses the stable
    # action plugin API and is sufficient for SITL climb.
    try:
        await drone.action.set_takeoff_altitude(ALTITUDE)
        log(f"  Takeoff altitude set to {ALTITUDE}m")
    except Exception as e:
        log_warn(f"set_takeoff_altitude failed (non-critical): {e}")

    log("Commanding takeoff...")
    await drone.action.takeoff()

    # ── Wait for TAKING_OFF transition ────────────
    TAKEOFF_ACCEPT_TIMEOUT = 15.0
    t_accept = asyncio.get_running_loop().time()
    log("Waiting for takeoff acceptance (TAKING_OFF state)...")
    async for state in drone.telemetry.landed_state():
        elapsed = asyncio.get_running_loop().time() - t_accept
        if state == LandedState.TAKING_OFF or state == LandedState.IN_AIR:
            log(f"  Takeoff accepted — state={state}  ({elapsed:.1f}s)")
            break
        if elapsed > TAKEOFF_ACCEPT_TIMEOUT:
            log_warn(f"PX4 did not begin takeoff within {TAKEOFF_ACCEPT_TIMEOUT:.0f}s")
            log_warn(f"Last state={state} — attempting direct goto_location climb")
            # Fallback: use goto_location directly
            await drone.action.goto_location(HOME_LAT, HOME_LON, home_abs_alt + ALTITUDE, float("nan"))
            break
        await asyncio.sleep(0.1)

    # ── Climb to cruise altitude with adaptive logic ─────────────────
    ALT_THRESHOLD   = max(5.0, ALTITUDE * 0.04)  # 2% of target: 5m@30m, 20m@500m
    # Scale climb timeout: 90s for 30m, ~400s for 500m at ~1.4 m/s actual rate
    CLIMB_TIMEOUT   = max(90.0, ALTITUDE / 1.2)
    CLIMB_STALL_CHECK = 15.0
    target_alt      = ALTITUDE
    t_climb         = asyncio.get_running_loop().time()
    last_print      = t_climb
    last_alt        = 0.0
    stall_warned    = False
    force_goto_used = False

    log(f"Climbing to {target_alt:.0f} m (threshold: within {ALT_THRESHOLD:.0f} m)...")

    while True:
        await asyncio.sleep(0.5)
        elapsed_climb = asyncio.get_running_loop().time() - t_climb
        alt = drone_state["alt"]
        
        # Progress indicator every 3 seconds
        if asyncio.get_running_loop().time() - last_print >= 3.0:
            last_print = asyncio.get_running_loop().time()
            climb_rate = (alt - last_alt) / 3.0 if last_alt > 0 else 0
            log(f"  alt={alt:.1f}m / {target_alt:.0f}m  ({elapsed_climb:.1f}s)  climb_rate={climb_rate:.2f}m/s")
            last_alt = alt
        
        # Check for climb stall
        if not stall_warned and elapsed_climb > CLIMB_STALL_CHECK and alt < 15.0:
            stall_warned = True
            log_warn(f"Climb stalled at {alt:.1f}m after {elapsed_climb:.1f}s — low climb rate")
            log_warn("Attempting recovery: sending force takeoff command...")
            try:
                await drone.action.takeoff()
                await asyncio.sleep(2.0)
            except Exception as e:
                log_warn(f"Force takeoff failed: {e}")
        
        # If still low after 30 seconds, use goto_location as backup
        if not force_goto_used and elapsed_climb > 30.0 and alt < 20.0:
            force_goto_used = True
            log_warn(f"Still low at {alt:.1f}m after 30s — switching to goto_location climb")
            try:
                await drone.action.goto_location(HOME_LAT, HOME_LON, home_abs_alt + target_alt, float("nan"))
                log("  goto_location command sent")
            except Exception as e:
                log_warn(f"goto_location fallback failed: {e}")
        
        # Success condition
        if alt >= target_alt - ALT_THRESHOLD:
            log(f"Cruise altitude reached — alt={alt:.1f}m  ({elapsed_climb:.1f}s)")
            break
        
        # Timeout condition - don't crash, just warn and continue
        if elapsed_climb > CLIMB_TIMEOUT:
            log_warn(f"⚠️  Climb incomplete: reached {alt:.1f}m / {target_alt:.0f}m after {CLIMB_TIMEOUT:.0f}s")
            log_warn(f"  Continuing mission at current altitude — obstacle avoidance active")
            break

    # Continue with mission after climb
    # BUG FIX: Two issues caused persistent HOLD after takeoff:
    #
    # 1. Missing set_current_mission_item(0) — after upload + takeoff, PX4's
    #    internal mission cursor is undefined.  Calling start_mission() without
    #    first resetting the cursor leaves PX4 unsure where to begin and it
    #    stays in HOLD.  Explicitly reset to item 0 before the first attempt.
    #
    # 2. Single retry insufficient — PX4 SITL sometimes needs 3-4 start_mission()
    #    nudges over ~15s before accepting the mode switch (MAVLink round-trip
    #    timing during SITL init).  Replaced with a rolling retry every 4s.
    #
    # 3. Hard crash on timeout removed — if PX4 is in HOLD but the drone is
    #    airborne at a reasonable altitude, crashing here aborts a flight that
    #    could otherwise succeed.  Instead: warn and continue; the survey
    #    mission_progress loop is the authoritative completion signal.
    log("Starting mission — resetting mission cursor and switching to MISSION mode...")
    try:
        await drone.mission.set_current_mission_item(0)
        log("  Mission cursor reset to item 0")
    except Exception as e:
        log_warn(f"  set_current_mission_item(0) failed (non-critical): {e}")

    await drone.mission.start_mission()

    from mavsdk.telemetry import FlightMode
    FMODE_TIMEOUT   = 40.0    # extended — SITL can be slow post-takeoff
    RETRY_INTERVAL  = 4.0    # re-issue start_mission() every 4s if still HOLD
    t_fmode         = asyncio.get_running_loop().time()
    last_retry_t    = t_fmode
    mission_started = False

    async for fmode in drone.telemetry.flight_mode():
        elapsed_fm = asyncio.get_running_loop().time() - t_fmode
        mode_name  = str(fmode)
        log(f"  Flight mode: {mode_name}  ({elapsed_fm:.1f}s)")

        if fmode == FlightMode.MISSION:
            log("MISSION mode confirmed — survey underway")
            mission_started = True
            break

        # Rolling retry every RETRY_INTERVAL seconds
        now = asyncio.get_running_loop().time()
        if now - last_retry_t >= RETRY_INTERVAL:
            last_retry_t = now
            log(f"  HOLD persists at {elapsed_fm:.1f}s — re-issuing start_mission()...")
            try:
                await drone.mission.set_current_mission_item(0)
                await drone.mission.start_mission()
            except Exception as e:
                log_warn(f"  start_mission() retry error: {e}")

        if elapsed_fm > FMODE_TIMEOUT:
            log_warn(
                f"PX4 still in {mode_name} after {FMODE_TIMEOUT:.0f}s — "
                f"alt={drone_state['alt']:.1f}m. "
                f"Continuing anyway; mission_progress will confirm execution."
            )
            break

    if not mission_started:
        # Last-ditch attempt before entering the progress loop
        try:
            await drone.mission.set_current_mission_item(0)
            await drone.mission.start_mission()
            await asyncio.sleep(2.0)
            log("  Final start_mission() issued — entering survey progress loop")
        except Exception as e:
            log_warn(f"  Final start_mission() failed: {e}")

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
        """Fly to a target dict and orbit it using OrbitMPC."""
        t_lat   = target["lat"]
        t_lon   = target["lon"]
        t_r     = target["orbit_radius_m"]
        t_spd   = target["orbit_speed_ms"]
        t_alt   = target["orbit_altitude_m"]
        t_dur   = target["orbit_duration_s"]
        t_name  = target.get("name", phase_label)

        banner(f"{phase_label} — {t_name}")
        log(f"Target: {t_lat:.6f}, {t_lon:.6f}")
        # B12 FIX: secondary orbit configs have ISR-specific altitudes (70-100m AGL).
        # In MBC3_MODE (ALTITUDE=500m), using those raw values forces a 400-430m
        # descent per orbit. Cap to cruise altitude so we never descend below survey alt.
        goto_alt = home_abs_alt + max(t_alt, ALTITUDE)

        # set_maximum_speed removed — not available in this MAVSDK version.
        # PX4 SITL uses MPC_XY_VEL_MAX (15 m/s) set in airframe.

        # FIX-2: fly to orbit entry point (North of target at orbit radius) instead
        # of target centre.  When goto_location goes to the centre, do_orbit starts
        # at radius=0 and spirals outward — the drone never reaches the commanded
        # radius during the dwell window.  Starting at the orbit circle edge means
        # do_orbit locks onto the correct radius immediately.
        from mpc_controller import project_waypoint as _project_wp
        entry_lat, entry_lon = _project_wp(t_lat, t_lon, 0.0, t_r)
        await drone.action.goto_location(entry_lat, entry_lon, goto_alt, float("nan"))
        log(f"Flying to orbit entry point ({t_r:.0f}m North of target)...")

        t_approach = asyncio.get_running_loop().time()
        APPROACH_TIMEOUT_S = 180.0   # raised: 40 m/s × 180s = 7.2 km max range
        async for pos in drone.telemetry.position():
            dist = haversine(pos.latitude_deg, pos.longitude_deg, t_lat, t_lon)
            print(f"\r  Distance to {t_name}: {dist:.1f}m  "
                  f"(entry at {t_r:.0f}m)     ", end="", flush=True)
            # Arrived when within 1.3× orbit radius — on or near the orbit circle
            if dist <= t_r * 1.3:
                print()
                log(f"At orbit entry — dist={dist:.1f}m  target_r={t_r:.0f}m")
                break
            if asyncio.get_running_loop().time() - t_approach > APPROACH_TIMEOUT_S:
                print()
                log_warn(f"Approach timeout ({APPROACH_TIMEOUT_S:.0f}s) — "
                         f"still {dist:.0f}m from {t_name}, proceeding to orbit")
                break
            await asyncio.sleep(0.1)

        # In-flight NFZ check before committing to orbit
        nfz_inside, nfz_name, _ = get_nfz_exclusion_check(t_lat, t_lon)
        if nfz_inside:
            log_alert(f"TARGET INSIDE NFZ '{nfz_name}' — skipping orbit, continuing mission")
            return

        opid = OrbitMPC(target_radius=t_r)
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
            # Use drone_state cache (populated by telemetry_tracker) — avoids
            # opening a new position subscription every second of the countdown
            # which leaks N gRPC streams and floods the MAVSDK callback queue.
            radial_err, current_r = opid.compute_correction(
                drone_state["lat"], drone_state["lon"], t_lat, t_lon
            )
            bar = "=" * (i * 20 // t_dur) + "-" * (20 - i * 20 // t_dur)
            print(f"\r  Orbit: [{bar}] {i:3d}s  radius={current_r:.1f}m  "
                  f"radial_err={radial_err:+.2f}m", end="", flush=True)
            await asyncio.sleep(1)
        print()
        log(f"Orbit complete — {t_name}")

    # ── PHASE 3: Primary target orbit ─────────────────────
    mission_state["mission_phase"] = "LOITER"
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
    # BUG-3 FIX: replaced frozen sorted_secondaries snapshot with live re-evaluation.
    # Old code: sorted_secondaries captured before the loop; targets appended by
    # _apply_dynamic_commands() mid-loop (while SEC-1 is orbiting) were in
    # SECONDARY_TARGETS but not in the snapshot — never visited.
    # New code: re-reads SECONDARY_TARGETS after each orbit; visited set prevents
    # re-flying targets already completed.
    visited_targets = set()
    sec_idx = 0
    while True:
        remaining = sorted(
            [t for t in SECONDARY_TARGETS if id(t) not in visited_targets],
            key=lambda t: t.get("priority", 99)
        )
        if not remaining:
            break
        sec = remaining[0]
        visited_targets.add(id(sec))
        sec_idx += 1
        mission_state["mission_phase"] = f"SEC-{sec_idx}"
        label = f"PHASE 4.{sec_idx} — SECONDARY TARGET {sec_idx}"
        await _do_orbit_phase(sec, label, home_abs_alt)

    # ── PHASE 5: RTL ──────────────────────────────────────
    mission_state["mission_phase"] = "RTL"
    banner("PHASE 5 — RETURN TO LAUNCH")
    log("Saving 3D occupancy map before RTL...")
    try:
        saved = map_builder.save(MAP_SAVE_PATH)
        log(f"3D map saved — raw: {saved['raw_pcd']}  voxel: {saved['voxel_pcd']}")
    except Exception as e:
        log_warn(f"Map save failed (non-critical): {e}")
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
    banner("FULL ISR + LiDAR MPC MISSION COMPLETE v12-MPC-v5")
    log(f"Survey -> {avoidance_state['count']} obstacle(s) avoided -> "
        f"{total_targets} target(s) acquired & orbited -> RTL")
    log("Aran Technologies — Ready for IIT Panel Demo")


# ══════════════════════════════════════════════════════════
#  ENTRY POINT
# ══════════════════════════════════════════════════════════
if __name__ == "__main__":
    print()
    print("  +==================================================+")
    print("  |   ARAN TECHNOLOGIES — ISR + LiDAR MPC DEMO      |")
    print("  |   v12-MPC-v5 — Upload Fix + Arm/Takeoff/Climb   |")
    print("  |   Multi-Target | NFZ Fencing | Scenarios         |")
    print("  |   Weight Scheduling | Avoidance Timeout | ETA    |")
    print("  |   PX4 SITL + Gazebo Harmonic + MAVSDK Python     |")
    print("  +==================================================+")
    print()
    if not GZ_AVAILABLE:
        print("  [SIM MODE] gz-transport not installed")
        print("  Install: sudo apt install python3-gz-transport13 python3-gz-msgs10\n")
    asyncio.run(run())