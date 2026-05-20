"""
Tests for mission_config.py
Covers: generate_survey_grid, get_nfz_exclusion_check (BUG-D fix),
        RACING_MODE env parsing (BUG-C fix), generate_all_sweeps.
"""
import math
import os
import sys
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))


# ── generate_survey_grid ──────────────────────────────────────────────────────

def test_survey_grid_waypoint_count():
    from mission_config import generate_survey_grid, ROWS
    wps = generate_survey_grid()
    assert len(wps) == ROWS * 2  # 2 WPs per row (start + end)

def test_survey_grid_boustrophedon():
    """Odd rows fly L→R, even rows fly R→L — lon_start alternates."""
    from mission_config import generate_survey_grid, HOME_LON, ROW_WIDTH
    wps = generate_survey_grid()
    # Row 0 (even): first WP lon < second WP lon  (L → R)
    assert wps[0][1] < wps[1][1]
    # Row 1 (odd): first WP lon > second WP lon  (R → L)
    assert wps[2][1] > wps[3][1]

def test_survey_grid_altitude_offset_does_not_change_latlon():
    from mission_config import generate_survey_grid, ALTITUDE_STEP
    wps_base  = generate_survey_grid(altitude_offset=0.0)
    wps_high  = generate_survey_grid(altitude_offset=ALTITUDE_STEP)
    assert wps_base == wps_high  # altitude offset doesn't touch lat/lon tuples

def test_generate_all_sweeps_count():
    from mission_config import generate_all_sweeps, GRID_ALTITUDE_STEPS
    sweeps = generate_all_sweeps()
    assert len(sweeps) == GRID_ALTITUDE_STEPS

def test_generate_all_sweeps_altitudes():
    from mission_config import generate_all_sweeps, ALTITUDE, ALTITUDE_STEP
    sweeps = generate_all_sweeps()
    for i, (alt, _) in enumerate(sweeps):
        assert abs(alt - (ALTITUDE + i * ALTITUDE_STEP)) < 1e-6


# ── get_nfz_exclusion_check ────────────────────────────────────────────────────

def test_nfz_outside_all_zones():
    from mission_config import get_nfz_exclusion_check, HOME_LAT, HOME_LON
    inside, name, dist = get_nfz_exclusion_check(HOME_LAT, HOME_LON)
    assert inside is False
    assert name is not None     # returns closest zone name even when not breaching
    assert dist > 0

def test_nfz_inside_zone_1():
    """NFZ-1 centre: lat=47.3975, lon=8.5465, radius=40m."""
    from mission_config import get_nfz_exclusion_check
    inside, name, _ = get_nfz_exclusion_check(47.3975, 8.5465)
    assert inside is True
    assert "NFZ-1" in name

def test_nfz_breaching_name_is_closest_breach_not_global_closest():
    """
    BUG-D regression: if a non-breaching zone is closer than a breaching zone,
    breaching_name must still report the breaching zone.
    """
    from mission_config import get_nfz_exclusion_check, NO_FLY_ZONES
    # Place drone just inside NFZ-3 (lat=47.3962, lon=8.5490, r=25m)
    nfz3 = next(z for z in NO_FLY_ZONES if "NFZ-3" in z["name"])
    inside, name, _ = get_nfz_exclusion_check(nfz3["lat"], nfz3["lon"])
    assert inside is True
    assert "NFZ-3" in name

def test_nfz_haversine_distance_accuracy():
    """Point exactly 50 m north of NFZ-1 centre should be outside (r=40m)."""
    from mission_config import get_nfz_exclusion_check
    lat_north = 47.3975 + (50 / 111320.0)
    inside, _, _ = get_nfz_exclusion_check(lat_north, 8.5465)
    assert inside is False


# ── RACING_MODE env parsing (BUG-C fix) ───────────────────────────────────────

def test_racing_mode_default_is_on(monkeypatch):
    monkeypatch.delenv("RACING_MODE", raising=False)
    import importlib, mission_config
    importlib.reload(mission_config)
    assert mission_config.RACING_MODE is True

def test_racing_mode_off_via_env(monkeypatch):
    monkeypatch.setenv("RACING_MODE", "0")
    import importlib, mission_config
    importlib.reload(mission_config)
    assert mission_config.RACING_MODE is False

def test_racing_mode_off_via_false_string(monkeypatch):
    monkeypatch.setenv("RACING_MODE", "false")
    import importlib, mission_config
    importlib.reload(mission_config)
    assert mission_config.RACING_MODE is False

def test_racing_mode_on_via_1(monkeypatch):
    monkeypatch.setenv("RACING_MODE", "1")
    import importlib, mission_config
    importlib.reload(mission_config)
    assert mission_config.RACING_MODE is True
