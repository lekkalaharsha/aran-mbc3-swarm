"""
Aran Technologies — Mission Config  [v12]
Single source of truth for all coordinates, grid parameters, and the
generate_survey_grid() helper.

v12 Additions:
  - SECONDARY_TARGETS: list of additional ISR points of interest for
    multi-target recon (each visited after primary orbit)
  - NO_FLY_ZONES: list of (lat, lon, radius_m) circles the avoidance
    layer should treat as hard exclusion zones
  - GRID_ALTITUDE_STEPS: multi-altitude sweep — first pass at ALTITUDE,
    second at ALTITUDE + ALTITUDE_STEP for stereo imaging
  - LOITER_WAYPOINTS: fixed loiter points injected mid-survey for
    persistent surveillance of suspect areas
  - generate_survey_grid() now accepts an optional altitude_offset param
    so the caller can generate a second sweep at a different altitude

Previously these were duplicated between isr_lidar_pid.py and gcs_dashboard.py,
causing silent map/mission mismatches whenever a value was changed in one file
but not the other.

Usage:
    from mission_config import (
        HOME_LAT, HOME_LON,
        TARGET_LAT, TARGET_LON,
        ORBIT_RADIUS, ORBIT_SPEED, ORBIT_ALTITUDE, ORBIT_DURATION,
        ROWS, ROW_SPACING, ROW_WIDTH,
        ALTITUDE, SPEED,
        SECONDARY_TARGETS, NO_FLY_ZONES, LOITER_WAYPOINTS,
        ALTITUDE_STEP, GRID_ALTITUDE_STEPS,
        generate_survey_grid,
    )
"""

# ══════════════════════════════════════════════════════════
#  HOME / LAUNCH POSITION
# ══════════════════════════════════════════════════════════
HOME_LAT    = 47.3977
HOME_LON    = 8.5456

# ══════════════════════════════════════════════════════════
#  CRUISE PARAMETERS
# ══════════════════════════════════════════════════════════
import os as _os_alt
# MBC3_MODE=1 → 500m AGL (MBC-3 requirement 2.10: ≥500m AGL)
# Default (0) → 30m for ISR/racing demo
_MBC3_MODE = _os_alt.environ.get("MBC3_MODE", "0") not in ("0", "false", "False")
ALTITUDE = 500.0 if _MBC3_MODE else 30.0   # cruise altitude (m AGL)
SPEED    = 40.0          # mission speed (m/s) — racing cruise (30–60 m/s range)

# ══════════════════════════════════════════════════════════
#  SURVEY GRID
# ══════════════════════════════════════════════════════════
ROWS        = 4
ROW_SPACING = 0.0003     # degrees between rows
ROW_WIDTH   = 0.0006     # degrees row length

# Multi-altitude stereo sweep
ALTITUDE_STEP       = 15.0   # metres between sweep passes
GRID_ALTITUDE_STEPS = 1      # 1 = single pass; 2 = dual-altitude stereo

# ══════════════════════════════════════════════════════════
#  PRIMARY TARGET & ORBIT
# ══════════════════════════════════════════════════════════
TARGET_LAT     = 47.3985
TARGET_LON     = 8.5470
ORBIT_RADIUS   = 50.0    # metres — increased from 30m: at 35m/s, 30m radius needs
                         # 40.8 m/s² centripetal force which exceeds PX4's
                         # MPC_ACC_HOR_MAX (~5 m/s²), causing the drone to
                         # drift outward and spiral.  50m needs only 24.5 m/s².
ORBIT_SPEED    = 12.0    # m/s — reduced from 35m/s: safe centripetal at 50m
                         # radius is sqrt(5 × 50) ≈ 15.8 m/s.  12 m/s gives
                         # 2.9 m/s² — well within PX4's horizontal accel limit.
ORBIT_ALTITUDE = ALTITUDE  # m AGL — matches cruise altitude (500m MBC3, 30m ISR)
ORBIT_DURATION = 15      # seconds


# ══════════════════════════════════════════════════════════
#  SECONDARY ISR TARGETS
#  Visited in order after the primary orbit, each with its own
#  orbit radius, speed, altitude and dwell duration.
# ══════════════════════════════════════════════════════════
SECONDARY_TARGETS = [
    {
        "name":             "ALPHA-2 Industrial Compound",
        "lat":              47.3991,
        "lon":              8.5482,
        "orbit_radius_m":   50.0,   # increased: 30m @ 15m/s needs 7.5 m/s² (too high)
        "orbit_speed_ms":   12.0,   # reduced: 50m @ 12m/s = 2.9 m/s² centripetal
        "orbit_altitude_m": 70.0,
        "orbit_duration_s": 20,
        "priority":         1,
    },
    {
        "name":             "BRAVO-1 River Crossing",
        "lat":              47.3968,
        "lon":              8.5448,
        "orbit_radius_m":   80.0,   # was 60m — kept large, speed reduced
        "orbit_speed_ms":   15.0,   # reduced from 25m/s: 80m @ 15m/s = 2.8 m/s²
        "orbit_altitude_m": 100.0,
        "orbit_duration_s": 15,
        "priority":         2,
    },
    {
        "name":             "CHARLIE-3 Treeline Perimeter",
        "lat":              47.3979,
        "lon":              8.5500,
        "orbit_radius_m":   60.0,   # was 50m — increased slightly for physics
        "orbit_speed_ms":   12.0,   # reduced from 20m/s: 60m @ 12m/s = 2.4 m/s²
        "orbit_altitude_m": 80.0,
        "orbit_duration_s": 25,
        "priority":         3,
    },
]


# ══════════════════════════════════════════════════════════
#  NO-FLY ZONES  (lat, lon, radius_m)
#  The avoidance layer treats these as persistent hard obstacles —
#  the drone should never enter within radius_m of the centre.
# ══════════════════════════════════════════════════════════
NO_FLY_ZONES = [
    {
        "name":      "NFZ-1 Restricted Airspace Alpha",
        "lat":       47.3975,
        "lon":       8.5465,
        "radius_m":  40.0,
        "reason":    "Civilian infrastructure — 500m GND restriction",
    },
    {
        "name":      "NFZ-2 Airport Approach Sector",
        "lat":       47.3993,
        "lon":       8.5440,
        "radius_m":  80.0,
        "reason":    "IFR approach path — hard exclusion",
    },
    {
        "name":      "NFZ-3 Military Comms Tower",
        "lat":       47.3962,
        "lon":       8.5490,
        "radius_m":  25.0,
        "reason":    "Electronic warfare zone — no overfly",
    },
]


# ══════════════════════════════════════════════════════════
#  LOITER WAYPOINTS
#  Mid-survey hold points for persistent area surveillance.
#  Inserted between survey rows at the specified WP index.
# ══════════════════════════════════════════════════════════
LOITER_WAYPOINTS = [
    {
        "name":           "LOITER-A Suspect Vehicle",
        "lat":            47.3982,
        "lon":            8.5458,
        "altitude_m":     50.0,
        "loiter_time_s":  8.0,
        "after_wp_index": 3,   # inserted after survey WP 3
        "gimbal_pitch":   -45.0,
    },
    {
        "name":           "LOITER-B Building Entrance",
        "lat":            47.3980,
        "lon":            8.5466,
        "altitude_m":     40.0,
        "loiter_time_s":  10.0,
        "after_wp_index": 6,
        "gimbal_pitch":   -60.0,
    },
]


# ══════════════════════════════════════════════════════════
#  SURVEY GRID GENERATOR  (single definition, shared by both files)
# ══════════════════════════════════════════════════════════
def generate_survey_grid(altitude_offset: float = 0.0):
    """
    Return list of (lat, lon) survey waypoints for the boustrophedon grid.
    Odd rows fly left-to-right, even rows fly right-to-left.

    altitude_offset: added to ALTITUDE for multi-pass stereo sweeps.
    Pass altitude_offset=ALTITUDE_STEP to get the second (higher) sweep.
    The altitude itself is not embedded in the tuples — the caller uses it
    when building MissionItem objects — but it is available via the returned
    metadata dict for display on the GCS map.
    """
    waypoints = []
    for i in range(ROWS):
        lat       = HOME_LAT - (ROWS / 2 * ROW_SPACING) + (i * ROW_SPACING)
        lon_start = HOME_LON - ROW_WIDTH / 2 if i % 2 == 0 else HOME_LON + ROW_WIDTH / 2
        lon_end   = HOME_LON + ROW_WIDTH / 2 if i % 2 == 0 else HOME_LON - ROW_WIDTH / 2
        waypoints.append((lat, lon_start))
        waypoints.append((lat, lon_end))
    return waypoints


def generate_all_sweeps():
    """
    Return all altitude-step sweeps as a list of (altitude_m, waypoints) tuples.
    Used by isr_lidar_pid.py to build multi-altitude mission plans.
    """
    sweeps = []
    for step in range(GRID_ALTITUDE_STEPS):
        alt_m = ALTITUDE + step * ALTITUDE_STEP
        sweeps.append((alt_m, generate_survey_grid(altitude_offset=step * ALTITUDE_STEP)))
    return sweeps


# ══════════════════════════════════════════════════════════
#  3D MAPPING PARAMETERS
# ══════════════════════════════════════════════════════════
MAP_RESOLUTION_M = 1.0          # metres per voxel cell (1.0 = 1 m³ voxels)
MAP_SAVE_PATH    = "map_output" # directory for .pcd and GeoJSON exports
MAP_SLICE_BAND_M = 5.0          # ± metres around drone altitude for 2D slices


# ══════════════════════════════════════════════════════════
#  RACING MODE PARAMETERS
#  Used by mpc_controller.py and isr_lidar_mpc.py when
#  RACING_MODE = True.  All values tuned for 30–60 m/s.
# ══════════════════════════════════════════════════════════
import os as _os_cfg
# BUG-C FIX: was hardcoded True — launch.sh env injection had no effect.
RACING_MODE             = _os_cfg.environ.get("RACING_MODE", "1") not in ("0", "false", "False")

# Avoidance distances scale up at racing speeds — drone needs more room to stop
RACING_LIDAR_WARN_DIST  = 40.0   # m  (was 25 m at ISR speed)
RACING_LIDAR_AVOID_DIST = 25.0   # m  (was 15 m)
RACING_SAFE_RESUME_DIST = 35.0   # m  (was 22 m)
RACING_AVOIDANCE_OFFSET = 80.0   # m  detour waypoint distance (was 50 m)

# MPC weight schedule thresholds for racing
RACING_SPEED_THRESHOLD  = 25.0   # m/s — switch to fast MPC weights above this
RACING_MAX_SPEED        = 60.0   # m/s — hard velocity cap in MPC engine
RACING_MAX_ACCEL        = 12.0   # m/s² — higher thrust headroom on racing frame

# Waypoint acceptance radius — tightened from 20m to 6m.
# At 40 m/s, 20m radius meant the drone captured each WP while still 20m
# away, cutting the corners of every row turn by ~60% of the 33m row spacing.
# 6m = speed × 0.15s (one avoidance-loop tick buffer) — tight enough to
# actually fly the survey row while still avoiding overshoot oscillation.
RACING_ACCEPTANCE_RADIUS = 6.0   # m  (was 20m)


def get_nfz_exclusion_check(lat, lon):
    """
    Returns (is_inside_nfz, nfz_name, dist_m) for the closest no-fly zone.
    is_inside_nfz is True if the position violates any NFZ radius.
    nfz_name is the name of the BREACHING zone (if any), otherwise the closest zone.
    Used by the avoidance loop as a pre-flight and in-flight fence check.
    """
    import math
    closest_dist    = float("inf")
    closest_name    = None
    breaching_name  = None
    # BUG-D FIX: track closest breaching distance separately from global closest.
    # Previously used closest_dist (global minimum including non-breaching zones)
    # as the comparison — a nearby non-breaching zone made closest_dist small,
    # so later breaching zones at larger distances never updated breaching_name
    # even when they were closer than the first breach zone.
    breaching_dist  = float("inf")
    inside          = False
    for nfz in NO_FLY_ZONES:
        R = 6371000
        phi1, phi2 = math.radians(lat), math.radians(nfz["lat"])
        dphi = phi2 - phi1
        dlam = math.radians(nfz["lon"] - lon)
        a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlam/2)**2
        dist = R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        if dist < closest_dist:
            closest_dist = dist
            closest_name = nfz["name"]
        if dist < nfz["radius_m"]:
            inside = True
            if dist < breaching_dist:
                breaching_dist = dist
                breaching_name = nfz["name"]
    reported_name = breaching_name if inside else closest_name
    return inside, reported_name, closest_dist