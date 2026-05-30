"""
mission_config_swarm.py — 5-drone swarm sector layout + redistribution helpers.

Each drone owns 2 contiguous survey rows (10 rows total). Altitudes are
staggered 10 m apart for deconfliction. When a drone fails mid-mission,
its remaining waypoints are redistributed to adjacent active drones via D2D REASSIGN.

Usage:
    from mission_config_swarm import (
        DRONE_SECTORS, DRONE_TARGET,
        drone_alt, generate_drone_wps,
        compute_redistribution,
    )
"""

from mission_config import (
    HOME_LAT, HOME_LON,
    TARGET_LAT, TARGET_LON,
    SECONDARY_TARGETS,
)

# ── Swarm grid ────────────────────────────────────────────────────────────────
SWARM_NUM_DRONES = 5
SWARM_ROWS       = 10          # 2 rows × 5 drones
ROW_SPACING_DEG  = 0.0003      # ~33 m between rows
ROW_WIDTH_DEG    = 0.0006      # ~53 m row length

# ── Altitudes: staggered 10 m for deconfliction (G6) ─────────────────────────
BASE_ALT = 100.0    # m AGL for drone 0
ALT_SEP  =  10.0    # m per drone index


def drone_alt(idx: int) -> float:
    """Cruise altitude for drone idx: 100, 110, 120, 130, 140 m AGL."""
    return BASE_ALT + idx * ALT_SEP


# ── Explicit sector assignment (contiguous rows for physical coverage) ────────
# Each drone owns 2 rows. Row 0 = northernmost, row 9 = southernmost.
DRONE_SECTORS: dict[int, list[int]] = {
    0: [0, 1],    # north sector
    1: [2, 3],    # north-center
    2: [4, 5],    # center
    3: [6, 7],    # south-center
    4: [8, 9],    # south sector
}

# ── Per-drone secondary target after survey ───────────────────────────────────
DRONE_TARGET: dict[int, dict | None] = {
    0: SECONDARY_TARGETS[0],                # ALPHA-2 Industrial Compound
    1: SECONDARY_TARGETS[1],                # BRAVO-1 River Crossing
    2: {                                    # center drone → primary target
        "name":             "PRIMARY-TGT",
        "lat":              TARGET_LAT,
        "lon":              TARGET_LON,
        "orbit_radius_m":   50.0,
        "orbit_speed_ms":   12.0,
        "orbit_altitude_m": drone_alt(2),
        "orbit_duration_s": 20,
        "priority":         0,
    },
    3: SECONDARY_TARGETS[2],                # CHARLIE-3 Treeline Perimeter
    4: None,                                # drone 4 RTLs directly after survey
}


# ── Waypoint generators ────────────────────────────────────────────────────────

def generate_row(row_idx: int) -> tuple[tuple, tuple]:
    """Return (start_latlon, end_latlon) for one survey row (boustrophedon)."""
    lat = HOME_LAT - (SWARM_ROWS / 2.0 * ROW_SPACING_DEG) + row_idx * ROW_SPACING_DEG
    if row_idx % 2 == 0:
        return (lat, HOME_LON - ROW_WIDTH_DEG / 2), (lat, HOME_LON + ROW_WIDTH_DEG / 2)
    else:
        return (lat, HOME_LON + ROW_WIDTH_DEG / 2), (lat, HOME_LON - ROW_WIDTH_DEG / 2)


def generate_drone_wps(drone_idx: int) -> list[tuple[float, float]]:
    """Ordered (lat, lon) survey waypoints for this drone's assigned rows."""
    wps: list[tuple[float, float]] = []
    for row_idx in DRONE_SECTORS[drone_idx]:
        start, end = generate_row(row_idx)
        wps.extend([start, end])
    return wps


def all_sector_wps() -> dict[int, list[tuple[float, float]]]:
    """Return {drone_idx: [(lat, lon), ...]} for all 5 drones."""
    return {i: generate_drone_wps(i) for i in range(SWARM_NUM_DRONES)}


# ── Redistribution helper ─────────────────────────────────────────────────────

def compute_redistribution(
    failed_idx: int,
    last_completed_wp: int,
    plan_wps: list[tuple[float, float]],
    active_drones: list[int],
) -> dict[int, list[tuple[float, float]]]:
    """
    After drone `failed_idx` fails at WP `last_completed_wp`, distribute
    its remaining survey waypoints to `active_drones` round-robin.

    Args:
        failed_idx:          Drone index that failed (0-4).
        last_completed_wp:   Index of last WP the drone REACHED (0-based, exclusive).
        plan_wps:            Full (lat, lon) WP list from the failed drone's mission.
        active_drones:       List of drone indices still active (not failed).

    Returns:
        {recipient_drone_idx: [(lat, lon), ...]} — additional WPs per drone.
        Returns {} if nothing to redistribute or no active drones.
    """
    remaining = plan_wps[last_completed_wp:]
    if not remaining or not active_drones:
        return {}

    result: dict[int, list[tuple[float, float]]] = {i: [] for i in active_drones}
    # Prefer adjacent drones: sort active_drones by how close their sector is
    sector_center = (DRONE_SECTORS[failed_idx][0] + DRONE_SECTORS[failed_idx][-1]) / 2.0
    sorted_active = sorted(
        active_drones,
        key=lambda i: abs(
            (DRONE_SECTORS[i][0] + DRONE_SECTORS[i][-1]) / 2.0 - sector_center
        ),
    )

    for k, wp in enumerate(remaining):
        recipient = sorted_active[k % len(sorted_active)]
        result[recipient].append(wp)

    n_total = len(remaining)
    dist_str = ", ".join(f"DRONE-{i}:{len(result[i])}" for i in sorted_active)
    print(
        f"[REDISTRIB] DRONE-{failed_idx} failed at WP {last_completed_wp}/{len(plan_wps)}. "
        f"{n_total} remaining WPs → {dist_str}",
        flush=True,
    )
    return {k: v for k, v in result.items() if v}
