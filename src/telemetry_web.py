"""
Aran Technologies — GCS Dashboard v13

v13 Bug Fixes (post-release patch):
  - CRITICAL: Jinja2 HTML-escaping broke all three server-injected JSON blobs
    (secondary_targets_json, nfz_zones_json, loiter_waypoints_json). Jinja2
    auto-escapes plain string variables, turning every " into &quot; — the JS
    constants SECONDARY_TARGETS_INIT / NFZ_ZONES_INIT / LOITER_WPS_INIT were
    syntactically invalid, crashing the page on load and disabling every v13
    feature (secondary targets, NFZ overlay, loiter markers, target panel, NFZ
    monitor panel). Fixed by adding | safe to all three template variables.

  - NFZ and TARGETS map buttons had inverted initial state: nfzVisible=true and
    secTargetsVisible=true at init but the HTML buttons lacked the `active` CSS
    class. First click hid the layers (correct behaviour) yet turned the button
    ON — completely backwards UX. Fixed by adding `active` to both button elements.

  - nearest_bearing not guarded against inf/nan in emit_loop: nearest_dist was
    already clamped with min(..., 9999.0) but nearest_bearing was not. A LiDAR
    driver returning float('inf') or float('nan') for bearing when no obstacle is
    in range would raise OverflowError / ValueError from round(), crashing
    emit_loop and freezing the GCS. Fixed with an isfinite() guard + clamp.

  - scenario_list() used os.path.dirname(__file__) without os.path.abspath(),
    which resolves to '' (empty string) when __file__ is a relative path,
    causing FileNotFoundError for scenarios.json. Fixed with abspath().

v13 Additions:
  - Secondary ISR target markers on map (cyan diamonds, orbit rings, tooltips)
  - No-fly zone overlay: red exclusion circles with labels, toggleable via NFZ button
  - Loiter waypoint markers on map (camera icon, loiter time tooltip)
  - ISR Targets panel (right column): live status for primary + all secondary targets,
    shows active orbit, distance from drone, and priority order
  - /scenario_list GET endpoint: returns all available scenario names for remote
    selection without restarting the server
  - Mission phase list expanded: SURVEY-SEC-1/2/3 sub-phases for secondary orbits
  - NFZ breach alert: dedicated header indicator turns red when drone enters any NFZ
  - Map legend updated with NFZ and loiter symbols
  - emit_loop now pushes secondary_targets, nfz_zones, loiter_waypoints into payload
    so JS never needs hardcoded coordinates — all from mission_config.py
  - Footer and title bumped to v13

"""
import asyncio
import math
import threading
import time
from collections import deque
from datetime import datetime
from flask import Flask, render_template_string, request, jsonify, Response
from flask_socketio import SocketIO
from mavsdk import System

# ── All shared mission constants come from one place ──────
from mission_config import (
    HOME_LAT, HOME_LON,
    TARGET_LAT, TARGET_LON,
    ORBIT_RADIUS,
    ROWS, ROW_SPACING, ROW_WIDTH,
    SECONDARY_TARGETS, NO_FLY_ZONES, LOITER_WAYPOINTS,
    generate_survey_grid,
)

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

SURVEY_WAYPOINTS = generate_survey_grid()

data = {
    "lat": HOME_LAT, "lon": HOME_LON, "alt": 0.0,
    "groundspeed": 0.0, "battery": 100.0,
    "flight_mode": "---", "armed": False,
    "heading": 0.0, "connected": False,
    "elapsed": "00:00:00",
    "mission_phase": "STANDBY",
    "wp_current": 0, "wp_total": 0,
    "vspeed": 0.0,
    "gps_ok":    False,
    "reconnects": 0,
    "eta_seconds": None,
}

lidar_data = {
    "nearest_dist":     999.0,
    "nearest_bearing":  0.0,
    "scan_count":       0,
    "avoidance_active": False,
    "avoidance_count":  0,
    "detour_lat":       None,
    "detour_lon":       None,
    "alert_msg":        "",
    "escape_side":      "---",
    "timeout_active":   False,
    "sectors":          [999.0] * 8,
}

# 3D map live stats (populated by POST /map_update from isr_lidar_mpc)
map_data = {
    "voxel_count":     0,
    "resolution_m":    1.0,
    "raw_point_count": 0,
    "scan_count":      0,
    "bounds":          None,
    "alt_range_m":     None,
    "geojson_slice":   None,   # populated by /map_slice GET
}

# PID controllers (live-tunable via /pid_tune)
pid_gains = {
    "avoidance": {"kp": 1.5, "ki": 0.0, "kd": 0.6, "output_limit": 15.0},
    "altitude":  {"kp": 1.2, "ki": 0.1, "kd": 0.4, "output_limit": 3.0},
    "orbit":     {"kp": 0.8, "ki": 0.05,"kd": 0.3, "output_limit": 5.0},
}

start_time   = datetime.now()
trail        = deque(maxlen=300)
flight_log   = deque(maxlen=10000)

# Dynamic command queues — populated by REST endpoints, consumed once by the
# mission script via the JSON response to POST /lidar_update.  The mission
# script applies them immediately (NFZ append, target append, config patch,
# event injection) without needing a separate HTTP server on its side.
_dyn_cmd_lock    = threading.Lock()
dynamic_commands = {
    "nfz_queue":      [],   # [{name,lat,lon,radius_m,reason}]
    "target_queue":   [],   # [{name,lat,lon,orbit_*,priority}]
    "config_updates": {},   # {LIDAR_WARN_DIST|LIDAR_AVOID_DIST|AVOIDANCE_OFFSET|SAFE_RESUME_DIST: float}
    "event_queue":    [],   # [{bearing_deg,dist_m,duration_s}]
}

# BUG-A FIX: timestamp of last mission_phase push from isr_lidar_mpc.
# _mode() checks this before overwriting — prevents MAVSDK HOLD mapping
# from reverting SEC-1/2/3 phases written by the mission script.
_phase_state = {"push_time": 0.0}


# ── LiDAR update endpoint ──────────────────────────────────────────
@app.route("/lidar_update", methods=["POST"])
def lidar_update():
    payload = request.get_json(silent=True) or {}
    lidar_data["nearest_dist"]     = payload.get("nearest_dist",     999.0)
    lidar_data["nearest_bearing"]  = payload.get("nearest_bearing",  0.0)
    lidar_data["scan_count"]       = payload.get("scan_count",       0)
    lidar_data["avoidance_active"] = payload.get("avoidance_active", False)
    lidar_data["avoidance_count"]  = payload.get("avoidance_count",  0)
    lidar_data["detour_lat"]       = payload.get("detour_lat",       None)
    lidar_data["detour_lon"]       = payload.get("detour_lon",       None)
    lidar_data["alert_msg"]        = payload.get("alert_msg",        "")
    lidar_data["escape_side"]      = payload.get("escape_side",      "---")
    lidar_data["timeout_active"]   = payload.get("timeout_active",   False)
    # BUG FIX: sectors were never read from the POST payload — sector overlay
    # on the GCS map was permanently frozen at the [999.0]*8 init value.
    if "sectors" in payload:
        lidar_data["sectors"] = payload["sectors"]
    if "groundspeed" in payload: data["groundspeed"] = payload["groundspeed"]
    if "gps_ok"      in payload: data["gps_ok"]      = payload["gps_ok"]
    if "reconnects"  in payload: data["reconnects"]  = payload["reconnects"]
    if "eta_seconds" in payload: data["eta_seconds"] = payload["eta_seconds"]
    # BUG FIX: mission_phase pushed by isr_lidar_mpc.push_to_gcs() was never
    # written into data[], so SEC-1/2/3 phases were invisible to the GCS
    # frontend — the phase panel stayed frozen at "LOITER" for all secondary
    # orbits (PX4 reports HOLD for every do_orbit call).
    if "mission_phase" in payload:
        data["mission_phase"] = payload["mission_phase"]
        _phase_state["push_time"] = datetime.now().timestamp()
    # BUG FIX: wp_current / wp_total pushed from isr_lidar_mpc were never
    # written into data[] — waypoint progress bar only updated from MAVSDK
    # stream which stops during orbit/RTL phases. Now kept in sync from push.
    if "wp_current"  in payload: data["wp_current"]  = payload["wp_current"]
    if "wp_total"    in payload: data["wp_total"]     = payload["wp_total"]
    # 3D map stats piggybacked on the lidar_update payload
    if "map_stats" in payload:
        ms = payload["map_stats"]
        map_data["voxel_count"]     = ms.get("voxel_count",     0)
        map_data["resolution_m"]    = ms.get("resolution_m",    1.0)
        map_data["raw_point_count"] = ms.get("raw_point_count", 0)
        map_data["scan_count"]      = ms.get("scan_count",      0)
        map_data["bounds"]          = ms.get("bounds")
        map_data["alt_range_m"]     = ms.get("alt_range_m")

    # Drain the dynamic command queues into the response so the mission script
    # can apply them on the next push cycle without a separate channel.
    with _dyn_cmd_lock:
        cmds = {
            "nfz_queue":      list(dynamic_commands["nfz_queue"]),
            "target_queue":   list(dynamic_commands["target_queue"]),
            "config_updates": dict(dynamic_commands["config_updates"]),
            "event_queue":    list(dynamic_commands["event_queue"]),
        }
        dynamic_commands["nfz_queue"].clear()
        dynamic_commands["target_queue"].clear()
        dynamic_commands["config_updates"].clear()
        dynamic_commands["event_queue"].clear()
    return jsonify({"ok": True, "commands": cmds})


# ── 3D map endpoints ───────────────────────────────────────────────
@app.route("/map_update", methods=["POST"])
def map_update():
    """Receive 3D map stats from isr_lidar_mpc push_to_gcs()."""
    payload = request.get_json(silent=True) or {}
    stats   = payload.get("map_stats", {})
    map_data["voxel_count"]     = stats.get("voxel_count",     0)
    map_data["resolution_m"]    = stats.get("resolution_m",    1.0)
    map_data["raw_point_count"] = stats.get("raw_point_count", 0)
    map_data["scan_count"]      = stats.get("scan_count",      0)
    map_data["bounds"]          = stats.get("bounds")
    map_data["alt_range_m"]     = stats.get("alt_range_m")
    return jsonify({"ok": True})


@app.route("/map_slice", methods=["GET"])
def map_slice():
    """
    Return a GeoJSON slice of the 3D occupancy map at the drone's current
    altitude ± MAP_SLICE_BAND_M.  Called by the GCS Leaflet frontend to
    draw the real-time obstacle heatmap overlay.

    Query params:
        alt_min  (float, optional) — override lower bound in metres AGL
        alt_max  (float, optional) — override upper bound in metres AGL
    """
    drone_alt = data.get("alt", 50.0)
    alt_min   = float(request.args.get("alt_min", drone_alt - 5.0))
    alt_max   = float(request.args.get("alt_max", drone_alt + 5.0))

    cached = map_data.get("geojson_slice")
    if cached:
        return jsonify(cached)
    return jsonify({"type": "FeatureCollection", "features": [],
                    "meta": {"alt_min": alt_min, "alt_max": alt_max,
                             "voxel_count": map_data["voxel_count"]}})


@app.route("/map_stats", methods=["GET"])
def map_stats_endpoint():
    """Return current 3D map statistics for the GCS info panel."""
    return jsonify({
        "ok":            True,
        "voxel_count":   map_data["voxel_count"],
        "resolution_m":  map_data["resolution_m"],
        "point_count":   map_data["raw_point_count"],
        "scan_count":    map_data["scan_count"],
        "bounds":        map_data["bounds"],
        "alt_range_m":   map_data["alt_range_m"],
    })


# ── PID tune endpoint ──────────────────────────────────────────────
@app.route("/pid_tune", methods=["POST"])
def pid_tune():
    payload = request.get_json(silent=True) or {}
    controller = payload.get("controller")
    if controller not in pid_gains:
        return jsonify({"ok": False, "error": f"Unknown controller: {controller}"}), 400
    for key in ("kp", "ki", "kd", "output_limit"):
        if key in payload:
            pid_gains[controller][key] = float(payload[key])
    socketio.emit("pid_gains", pid_gains)
    return jsonify({"ok": True, "gains": pid_gains[controller]})


@app.route("/pid_gains", methods=["GET"])
def get_pid_gains():
    return jsonify(pid_gains)


# ── Scenario list endpoint ─────────────────────────────────────────
@app.route("/scenario_list", methods=["GET"])
def scenario_list():
    import json, os
    sc_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "scenarios.json")
    try:
        with open(sc_file) as f:
            d = json.load(f)
        names = [{"name": s["name"], "description": s["description"],
                  "events": len(s["events"])} for s in d.get("scenarios", [])]
        return jsonify({"ok": True, "scenarios": names, "count": len(names)})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500


# ── NFZ status endpoint ────────────────────────────────────────────
@app.route("/nfz_status", methods=["GET"])
def nfz_status():
    from mission_config import get_nfz_exclusion_check
    lat = data["lat"]
    lon = data["lon"]
    inside, name, dist = get_nfz_exclusion_check(lat, lon)
    return jsonify({"inside_nfz": inside, "closest_nfz": name,
                    "distance_m": round(dist, 1)})


# ── CSV flight log download ────────────────────────────────────────
@app.route("/download_log")
def download_log():
    if not flight_log:
        return "No flight data yet.", 404
    lines = ["timestamp,lat,lon,alt_m,groundspeed_ms,heading_deg,battery_pct,"
             "flight_mode,armed,lidar_dist_m,lidar_bearing_deg,avoidance_events\n"]
    for row in flight_log:
        lines.append(",".join(str(v) for v in row) + "\n")
    return Response(
        "".join(lines),
        mimetype="text/csv",
        headers={"Content-Disposition": "attachment; filename=flight_log.csv"}
    )


# ── Dynamic mission control endpoints ─────────────────────────────────

@app.route("/add_nfz", methods=["POST"])
def add_nfz():
    """Queue a no-fly zone to be applied by the mission script on next push cycle.

    Body: {"lat": float, "lon": float, "radius_m": float, "name": str, "reason": str}
    """
    payload = request.get_json(silent=True) or {}
    if "lat" not in payload or "lon" not in payload:
        return jsonify({"ok": False, "error": "lat and lon required"}), 400
    nfz = {
        "name":     payload.get("name",     f"DYN-NFZ-{int(time.time())}"),
        "lat":      float(payload["lat"]),
        "lon":      float(payload["lon"]),
        "radius_m": float(payload.get("radius_m", 50.0)),
        "reason":   payload.get("reason",   "Dynamic GCS injection"),
    }
    # BUG-1 FIX: append to GCS-process list so emit_loop reflects new NFZ on map.
    # GCS and mission script are separate OS processes with separate NO_FLY_ZONES
    # copies — queuing for mission script only left GCS map showing stale zones.
    NO_FLY_ZONES.append(nfz)
    with _dyn_cmd_lock:
        dynamic_commands["nfz_queue"].append(nfz)
    socketio.emit("dynamic_nfz", nfz)
    return jsonify({"ok": True, "nfz": nfz,
                    "note": "Will be applied by mission script on next push cycle (~0.2s)"})


@app.route("/add_target", methods=["POST"])
def add_target():
    """Queue an ISR target to orbit after the current secondary sequence.

    Body: {"lat": float, "lon": float, "name": str, "orbit_radius_m": float,
           "orbit_speed_ms": float, "orbit_altitude_m": float,
           "orbit_duration_s": int, "priority": int}
    """
    payload = request.get_json(silent=True) or {}
    if "lat" not in payload or "lon" not in payload:
        return jsonify({"ok": False, "error": "lat and lon required"}), 400
    target = {
        "name":             payload.get("name",             f"DYN-TGT-{int(time.time())}"),
        "lat":              float(payload["lat"]),
        "lon":              float(payload["lon"]),
        "orbit_radius_m":   float(payload.get("orbit_radius_m",   50.0)),
        "orbit_speed_ms":   float(payload.get("orbit_speed_ms",   12.0)),
        "orbit_altitude_m": float(payload.get("orbit_altitude_m", 50.0)),
        "orbit_duration_s": int(payload.get("orbit_duration_s",   15)),
        "priority":         int(payload.get("priority",           99)),
    }
    # BUG-2 FIX: append to GCS-process list so emit_loop shows new target on map.
    # Same process-isolation issue as BUG-1 for NO_FLY_ZONES.
    SECONDARY_TARGETS.append(target)
    with _dyn_cmd_lock:
        dynamic_commands["target_queue"].append(target)
    socketio.emit("dynamic_target", target)
    return jsonify({"ok": True, "target": target,
                    "note": "Will be appended to SECONDARY_TARGETS on next push cycle"})


@app.route("/config_update", methods=["POST"])
def config_update():
    """Patch live mission config values in the running mission script.

    Allowed keys: LIDAR_WARN_DIST, LIDAR_AVOID_DIST, AVOIDANCE_OFFSET, SAFE_RESUME_DIST
    Body: {"LIDAR_WARN_DIST": 30.0, "LIDAR_AVOID_DIST": 20.0}
    """
    payload = request.get_json(silent=True) or {}
    allowed = {"LIDAR_WARN_DIST", "LIDAR_AVOID_DIST", "AVOIDANCE_OFFSET", "SAFE_RESUME_DIST"}
    updates = {k: float(v) for k, v in payload.items() if k in allowed}
    if not updates:
        return jsonify({"ok": False,
                        "error": f"No valid keys. Allowed: {sorted(allowed)}"}), 400
    with _dyn_cmd_lock:
        dynamic_commands["config_updates"].update(updates)
    return jsonify({"ok": True, "updates": updates,
                    "note": "Will be applied by mission script on next push cycle"})


@app.route("/inject_event", methods=["POST"])
def inject_event():
    """Inject a timed LiDAR obstacle event into the running sim reader.

    Only effective in LiDAR-SIM mode (no gz-transport). For real LiDAR,
    use a physical obstacle or Gazebo model spawn.

    Body:
        bearing_deg  : float  — obstacle bearing (default 0.0)
        dist_m       : float  — obstacle distance in metres (default 10.0)
        duration_s   : float  — how long obstacle persists (default 5.0)
        frame        : str    — "sensor" (default) or "world"
                                "sensor": 0° = drone forward, clockwise
                                "world":  0° = North, clockwise (converted to sensor frame)

    BUG-5 FIX: bearing_deg was silently sensor-relative with no documentation.
    Operators using map/compass bearings (0=North) got obstacles injected in the
    wrong sector.  Added optional frame param; "world" converts to sensor frame
    using current drone heading from data["heading"].
    """
    payload = request.get_json(silent=True) or {}
    bearing = float(payload.get("bearing_deg", 0.0))
    frame   = payload.get("frame", "sensor")
    if frame == "world":
        bearing = (bearing - data.get("heading", 0.0)) % 360
    event = {
        "bearing_deg": bearing,
        "dist_m":      float(payload.get("dist_m",     10.0)),
        "duration_s":  float(payload.get("duration_s",  5.0)),
    }
    with _dyn_cmd_lock:
        dynamic_commands["event_queue"].append(event)
    return jsonify({"ok": True, "event": event, "frame_used": frame,
                    "note": "Active in SIM mode only — injected into lidar_sim_reader"})


RETRY_DELAY = 3.0


def _gcs_print(msg):
    ts = datetime.now().strftime("%H:%M:%S")
    print(f"  [{ts}] {msg}")


async def _stream(name, coro_factory, retry_delay=RETRY_DELAY):
    while True:
        try:
            await coro_factory()
        except Exception as e:
            err = str(e).splitlines()[0][:120]
            _gcs_print(f"stream '{name}' error — {err}")
            _gcs_print(f"retrying '{name}' in {retry_delay:.0f}s...")
            data["connected"] = False
            await asyncio.sleep(retry_delay)


async def telemetry_loop():
    drone = System()
    await drone.connect(system_address="udp://:14540")
    _gcs_print("Connecting to drone (GCS will retry on disconnect)...")

    async def _watch_connection():
        async for state in drone.core.connection_state():
            data["connected"] = state.is_connected
            if state.is_connected:
                for _fn, _hz in [
                    (drone.telemetry.set_rate_position,            5.0),
                    (drone.telemetry.set_rate_velocity_ned,        5.0),
                    (drone.telemetry.set_rate_battery,             1.0),
                    (drone.telemetry.set_rate_health,              2.0),
                    (drone.telemetry.set_rate_landed_state,        2.0),
                    (drone.telemetry.set_rate_in_air,              2.0),
                    (drone.telemetry.set_rate_attitude_euler,      5.0),
                    (drone.telemetry.set_rate_attitude_quaternion, 5.0),
                    (drone.telemetry.set_rate_imu,                 5.0),
                ]:
                    try:
                        await _fn(_hz)
                    except Exception:
                        pass
                _gcs_print("Drone connected!")
            else:
                _gcs_print("Drone disconnected — streams will auto-retry")

    async def _pos():
        async for p in drone.telemetry.position():
            data["lat"] = p.latitude_deg
            data["lon"] = p.longitude_deg
            data["alt"] = round(p.relative_altitude_m, 2)
            if data["armed"]:
                trail.append((p.latitude_deg, p.longitude_deg))

    async def _vel():
        async for v in drone.telemetry.velocity_ned():
            data["groundspeed"] = round(math.sqrt(v.north_m_s**2 + v.east_m_s**2), 2)
            data["vspeed"]      = round(v.down_m_s * -1, 2)

    async def _bat():
        async for b in drone.telemetry.battery():
            data["battery"] = round(b.remaining_percent * 100, 1)

    async def _mode():
        async for m in drone.telemetry.flight_mode():
            fm = str(m).replace("FlightMode.", "")
            data["flight_mode"] = fm
            # BUG-A FIX: only write mission_phase from MAVSDK stream if the
            # mission script hasn't pushed a more specific phase recently.
            # Without this guard, HOLD→"LOITER" overwrites "SEC-1"/"SEC-2"/"SEC-3"
            # pushed by isr_lidar_mpc every 0.2s, making secondary orbits
            # permanently invisible on the GCS phase panel.
            if datetime.now().timestamp() - _phase_state["push_time"] > 5.0:
                if fm == "TAKEOFF":            data["mission_phase"] = "TAKEOFF"
                elif fm == "MISSION":          data["mission_phase"] = "SURVEY"
                elif fm == "HOLD":             data["mission_phase"] = "LOITER"
                elif fm == "RETURN_TO_LAUNCH": data["mission_phase"] = "RTL"
                elif fm == "LAND":             data["mission_phase"] = "LANDING"

    async def _armed():
        async for a in drone.telemetry.armed():
            data["armed"] = a
            if not a:
                data["mission_phase"] = "STANDBY"

    async def _heading():
        async for h in drone.telemetry.heading():
            data["heading"] = round(h.heading_deg, 1)

    async def _mission_prog():
        async for p in drone.mission.mission_progress():
            data["wp_current"] = p.current
            data["wp_total"]   = p.total

    async def _health():
        async for h in drone.telemetry.health():
            data["gps_ok"] = h.is_global_position_ok

    await asyncio.gather(
        _watch_connection(),
        _stream("position",         _pos),
        _stream("velocity",         _vel),
        _stream("battery",          _bat),
        _stream("flight_mode",      _mode),
        _stream("armed",            _armed),
        _stream("heading",          _heading),
        _stream("mission_progress", _mission_prog),
        _stream("health",           _health),
    )


def start_telemetry():
    asyncio.run(telemetry_loop())


def emit_loop():
    while True:
        elapsed = datetime.now() - start_time
        data["elapsed"] = str(elapsed).split(".")[0]

        flight_log.append((
            datetime.now().strftime("%H:%M:%S"),
            round(data["lat"], 6), round(data["lon"], 6),
            data["alt"], data["groundspeed"], data["heading"],
            data["battery"], data["flight_mode"], data["armed"],
            # BUG FIX: nearest_dist can be float('inf') when no obstacle is
            # detected from a real LiDAR scan. round(float('inf'), 1) raises
            # OverflowError in some Python versions, crashing emit_loop and
            # freezing the entire GCS. Clamp to a sentinel value first.
            round(min(lidar_data["nearest_dist"], 9999.0), 1),
            # BUG FIX: nearest_bearing can also be inf/nan if the LiDAR driver
            # sends a degenerate value when no obstacle is in range. Apply the
            # same sentinel clamp to avoid OverflowError / ValueError from round().
            round(min(lidar_data["nearest_bearing"], 9999.0) if math.isfinite(lidar_data["nearest_bearing"]) else 0.0, 1),
            lidar_data["avoidance_count"],
        ))

        payload = dict(data)
        payload["trail"]              = list(trail[-150:])
        payload["lidar"]              = dict(lidar_data)
        payload["survey_waypoints"]   = SURVEY_WAYPOINTS
        payload["home_lat"]           = HOME_LAT
        payload["home_lon"]           = HOME_LON
        payload["target_lat"]         = TARGET_LAT
        payload["target_lon"]         = TARGET_LON
        payload["orbit_radius"]       = ORBIT_RADIUS
        payload["pid_gains"]          = pid_gains
        payload["secondary_targets"]  = SECONDARY_TARGETS
        payload["nfz_zones"]          = NO_FLY_ZONES
        payload["loiter_waypoints"]   = LOITER_WAYPOINTS
        payload["map"] = {
            "voxel_count":   map_data["voxel_count"],
            "resolution_m":  map_data["resolution_m"],
            "point_count":   map_data["raw_point_count"],
            "scan_count":    map_data["scan_count"],
            "alt_range_m":   map_data["alt_range_m"],
        }
        socketio.emit("telemetry", payload)
        time.sleep(0.4)


HTML = r"""
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<title>Aran Technologies — ISR GCS v13</title>

<script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.6.1/socket.io.min.js"></script>
<script src="https://cdnjs.cloudflare.com/ajax/libs/Chart.js/4.4.0/chart.umd.min.js"></script>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<link href="https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Rajdhani:wght@400;600;700&display=swap" rel="stylesheet">

<style>
:root {
  --bg:      #060b14;
  --panel:   #080f1c;
  --card:    #0b1525;
  --border:  #0e2340;
  --accent:  #00c8ff;
  --accent2: #00ff9d;
  --warn:    #ffb300;
  --danger:  #ff3d3d;
  --dim:     #1e4060;
  --text:    #c8dff0;
  --textdim: #4a7a9b;
  --mono:    'Share Tech Mono', monospace;
  --sans:    'Rajdhani', sans-serif;
}
* { margin:0; padding:0; box-sizing:border-box; }
html,body { width:100%; height:100%; overflow:hidden; background:var(--bg); color:var(--text); font-family:var(--sans); }
body::after {
  content:''; position:fixed; inset:0; pointer-events:none; z-index:9999;
  background:repeating-linear-gradient(0deg,transparent,transparent 2px,rgba(0,0,0,0.06) 2px,rgba(0,0,0,0.06) 4px);
}

.hdr {
  height:48px; display:flex; align-items:center; justify-content:space-between;
  padding:0 16px;
  background:linear-gradient(90deg,#060b14,#0b1a30,#060b14);
  border-bottom:1px solid var(--accent);
  box-shadow:0 0 20px rgba(0,200,255,0.15);
}
.hdr-left  { display:flex; align-items:center; gap:12px; }
.hdr-logo  { font-family:var(--sans); font-weight:700; font-size:1.1rem; color:var(--accent); letter-spacing:4px; }
.hdr-mission { font-family:var(--mono); font-size:0.65rem; color:var(--warn); letter-spacing:2px; }
.hdr-right { display:flex; align-items:center; gap:16px; font-family:var(--mono); font-size:0.63rem; color:var(--textdim); }
.hdr-right span { display:flex; align-items:center; gap:6px; }
.dot { width:7px; height:7px; border-radius:50%; background:var(--danger); }
.dot.on  { background:var(--accent2); box-shadow:0 0 8px var(--accent2); animation:pulse 1.5s infinite; }
.dot.warn{ background:var(--warn);    box-shadow:0 0 8px var(--warn); }
@keyframes pulse { 0%,100%{opacity:1} 50%{opacity:0.4} }

.layout { display:grid; grid-template-columns:240px 1fr 270px; height:calc(100vh - 48px - 28px); }
.lpanel,.rpanel { background:var(--panel); display:flex; flex-direction:column; gap:8px; padding:10px 8px; overflow-y:auto; overflow-x:hidden; }
.lpanel { border-right:1px solid var(--border); }
.rpanel { border-left:1px solid var(--border); }

.card { background:var(--card); border:1px solid var(--border); border-radius:4px; padding:10px 12px; position:relative; overflow:hidden; flex-shrink:0; }
.card::before { content:''; position:absolute; top:0; left:0; right:0; height:1px; background:linear-gradient(90deg,transparent,var(--accent),transparent); opacity:0.4; }
.ctitle { font-family:var(--mono); font-size:0.58rem; letter-spacing:3px; color:var(--accent); text-transform:uppercase; margin-bottom:8px; padding-bottom:5px; border-bottom:1px solid var(--border); }
.metric { display:flex; justify-content:space-between; align-items:baseline; padding:3px 0; border-bottom:1px solid #0a1828; }
.metric:last-child { border-bottom:none; }
.mlabel { font-family:var(--mono); font-size:0.6rem; color:var(--textdim); }
.mval   { font-family:var(--mono); font-size:0.95rem; color:var(--accent); font-weight:bold; }
.munit  { font-family:var(--mono); font-size:0.55rem; color:var(--dim); margin-left:2px; }

.armed-badge { text-align:center; padding:5px; border-radius:3px; font-family:var(--mono); font-weight:bold; font-size:0.8rem; letter-spacing:3px; margin-bottom:8px; }
.armed-badge.armed    { background:#001a00; color:var(--accent2); border:1px solid var(--accent2); box-shadow:0 0 10px rgba(0,255,157,0.2); }
.armed-badge.disarmed { background:#1a0000; color:var(--danger); border:1px solid var(--danger); }

.bat-bar-bg { background:#050e1a; border:1px solid var(--border); border-radius:2px; height:12px; overflow:hidden; margin-top:6px; }
.bat-bar    { height:100%; border-radius:2px; transition:width 0.6s, background 0.6s; }
.bat-label  { font-family:var(--mono); font-size:0.6rem; color:var(--textdim); margin-top:3px; text-align:right; }

.gauges-row  { display:flex; justify-content:space-around; align-items:center; padding:4px 0; }
.compass-wrap{ display:flex; justify-content:center; padding:4px 0; }

.phases { display:flex; flex-direction:column; gap:4px; }
.phase-item { display:flex; align-items:center; gap:8px; padding:5px 8px; border-radius:3px; border:1px solid var(--border); font-family:var(--mono); font-size:0.62rem; color:var(--textdim); background:var(--bg); transition:all 0.3s; }
.phase-item.active { color:var(--accent2); border-color:var(--accent2); background:#001a0d; box-shadow:0 0 10px rgba(0,255,157,0.1); }
.phase-item.done   { color:var(--dim); border-color:#0a1828; }
.phase-dot { width:6px; height:6px; border-radius:50%; background:var(--border); flex-shrink:0; }
.phase-item.active .phase-dot { background:var(--accent2); box-shadow:0 0 6px var(--accent2); }
.phase-item.done   .phase-dot { background:var(--dim); }

.wp-bar-bg { background:#050e1a; border:1px solid var(--border); border-radius:2px; height:10px; overflow:hidden; margin-top:4px; }
.wp-bar    { height:100%; background:linear-gradient(90deg,var(--accent),var(--accent2)); border-radius:2px; transition:width 0.6s; }
.wp-label  { display:flex; justify-content:space-between; font-family:var(--mono); font-size:0.58rem; color:var(--textdim); margin-top:3px; }

.center { display:flex; flex-direction:column; background:var(--bg); }
.map-wrap { flex:1; position:relative; overflow:hidden; }

.leaflet-container { background:#040d1a !important; }
.leaflet-tile { filter:brightness(0.52) saturate(0.35) hue-rotate(190deg); }
.leaflet-control-zoom a { background:#0b1525 !important; color:var(--accent) !important; border:1px solid var(--border) !important; font-family:var(--mono) !important; }
.leaflet-control-attribution { background:rgba(6,11,20,0.75) !important; color:var(--textdim) !important; font-size:0.48rem !important; }
.leaflet-tooltip {
  background:rgba(6,11,20,0.9) !important; border:1px solid #0e2340 !important;
  color:#4a7a9b !important; font-family:'Share Tech Mono',monospace !important;
  font-size:0.55rem !important; padding:2px 7px !important; border-radius:2px !important;
}

.alert-banner {
  position:absolute; top:8px; left:50%; transform:translateX(-50%);
  background:rgba(255,61,61,0.15); border:1px solid var(--danger);
  padding:5px 18px; border-radius:3px;
  font-family:var(--mono); font-size:0.65rem; color:var(--danger);
  letter-spacing:2px; white-space:nowrap;
  display:none; z-index:1000; pointer-events:none;
  animation:alertflash 0.5s infinite;
}
@keyframes alertflash { 0%,100%{opacity:1;border-color:var(--danger)} 50%{opacity:0.5;border-color:transparent} }

.map-controls {
  position:absolute; top:8px; left:8px; z-index:1000;
  display:flex; gap:6px; flex-direction:column;
}
.map-btn {
  background:rgba(8,15,28,0.92); border:1px solid var(--border);
  color:var(--textdim); font-family:var(--mono); font-size:0.58rem;
  padding:4px 8px; border-radius:3px; cursor:pointer; letter-spacing:1px;
  transition:all 0.2s;
}
.map-btn:hover    { border-color:var(--accent); color:var(--accent); }
.map-btn.active   { border-color:var(--accent2); color:var(--accent2); background:rgba(0,255,157,0.07); }
.map-btn.danger   { border-color:var(--danger); color:var(--danger); }

.map-legend {
  position:absolute; bottom:8px; right:8px; z-index:1000;
  background:rgba(6,11,20,0.88); border:1px solid var(--border);
  border-radius:4px; padding:7px 10px;
  font-family:var(--mono); font-size:0.53rem; color:var(--textdim);
  pointer-events:none; line-height:2;
}
.legend-row { display:flex; align-items:center; gap:7px; }
.ls { width:22px; height:3px; border-radius:2px; flex-shrink:0; }
.ls-dashed { border-top:2px dashed; height:0; }

.chart-strip { height:155px; padding:8px 12px; background:#060c18; border-top:1px solid var(--border); }
.chart-title { font-family:var(--mono); font-size:0.58rem; color:var(--accent); letter-spacing:2px; margin-bottom:5px; }

.obstacle-bar-bg { background:#050e1a; border:1px solid var(--border); border-radius:2px; height:10px; overflow:hidden; margin-top:4px; }
.obstacle-bar    { height:100%; border-radius:2px; transition:width 0.3s, background 0.3s; }
.lidar-bearing-wrap { display:flex; justify-content:center; margin:6px 0; }

.sector-row { display:grid; grid-template-columns:repeat(8,1fr); gap:2px; margin:6px 0; }
.sector-cell {
  height:16px; border-radius:2px; background:#0a1828;
  border:1px solid #0e2340; font-family:var(--mono); font-size:0.42rem;
  color:#4a7a9b; display:flex; align-items:center; justify-content:center;
  transition:background 0.3s, color 0.3s;
}

.pid-row { display:flex; gap:4px; align-items:center; margin-bottom:4px; flex-wrap:wrap; }
.pid-label { font-family:var(--mono); font-size:0.55rem; color:var(--textdim); width:22px; }
.pid-input {
  background:#050e1a; border:1px solid var(--border); color:var(--accent);
  font-family:var(--mono); font-size:0.62rem; padding:2px 4px;
  border-radius:2px; width:52px; text-align:center;
}
.pid-input:focus { outline:none; border-color:var(--accent); }
.pid-send {
  background:rgba(0,200,255,0.1); border:1px solid var(--accent);
  color:var(--accent); font-family:var(--mono); font-size:0.55rem;
  padding:3px 8px; border-radius:2px; cursor:pointer; letter-spacing:1px;
}
.pid-send:hover { background:rgba(0,200,255,0.2); }
.pid-ctrl-select {
  background:#050e1a; border:1px solid var(--border); color:var(--accent);
  font-family:var(--mono); font-size:0.6rem; padding:2px 4px; border-radius:2px;
  width:100%; margin-bottom:6px;
}

.log-wrap { flex:1; overflow-y:auto; font-family:var(--mono); font-size:0.6rem; line-height:1.9; min-height:80px; }
.log-wrap::-webkit-scrollbar { width:3px; }
.log-wrap::-webkit-scrollbar-thumb { background:var(--dim); }
.le { border-bottom:1px solid #080f1c; padding:1px 0; }
.lt { color:var(--dim); }
.lm { color:#5a8aaa; }
.lok     { color:var(--accent2); }
.lwarn   { color:var(--warn); }
.ldanger { color:var(--danger); }

.footer { height:28px; display:flex; align-items:center; justify-content:space-between; padding:0 16px; background:#040912; border-top:1px solid var(--border); font-family:var(--mono); font-size:0.55rem; color:var(--dim); letter-spacing:1px; }

.drone-icon { width:20px; height:20px; position:relative; }
.drone-body {
  width:12px; height:12px;
  background:rgba(0,200,255,0.2);
  border:2px solid #00c8ff;
  border-radius:50%;
  box-shadow:0 0 14px #00c8ff, 0 0 28px rgba(0,200,255,0.4);
  animation:dronebeat 1s infinite;
  position:absolute; top:4px; left:4px;
}
.drone-arrow {
  width:0; height:0;
  border-left:4px solid transparent;
  border-right:4px solid transparent;
  border-bottom:8px solid #00c8ff;
  position:absolute; top:0; left:6px;
  filter:drop-shadow(0 0 3px #00c8ff);
}
@keyframes dronebeat { 0%,100%{box-shadow:0 0 14px #00c8ff,0 0 28px rgba(0,200,255,0.3)} 50%{box-shadow:0 0 22px #00c8ff,0 0 44px rgba(0,200,255,0.6)} }

.detour-icon {
  width:12px; height:12px;
  background:#ffb300; border:2px solid #fff; border-radius:50%;
  box-shadow:0 0 10px #ffb300;
  animation:detourpulse 1s infinite;
}
@keyframes detourpulse { 0%,100%{box-shadow:0 0 6px #ffb300} 50%{box-shadow:0 0 18px #ffb300,0 0 32px rgba(255,179,0,0.5)} }

.nfz-breach-badge {
  display:none; padding:3px 10px; border-radius:3px; font-family:var(--mono);
  font-size:0.58rem; letter-spacing:2px; color:var(--danger);
  border:1px solid var(--danger); background:rgba(255,61,61,0.12);
  animation:alertflash 0.8s infinite;
}
.nfz-breach-badge.active { display:inline-block; }

.target-row {
  display:flex; align-items:center; gap:7px; padding:5px 8px;
  border:1px solid var(--border); border-radius:3px; margin-bottom:4px;
  background:var(--bg); font-family:var(--mono); font-size:0.58rem;
  color:var(--textdim); transition:all 0.3s; cursor:default;
}
.target-row.active { border-color:#aa44ff; color:#cc88ff; background:rgba(170,68,255,0.07); box-shadow:0 0 8px rgba(170,68,255,0.15); }
.target-row.done   { color:var(--dim); border-color:#0a1828; }
.target-dot { width:6px; height:6px; border-radius:50%; background:var(--border); flex-shrink:0; }
.target-row.active .target-dot { background:#aa44ff; box-shadow:0 0 6px #aa44ff; animation:pulse 1.5s infinite; }
.target-row.done   .target-dot { background:var(--dim); }
.target-dist { margin-left:auto; color:var(--accent); font-size:0.6rem; }
</style>
</head>
<body>

<div class="hdr">
  <div class="hdr-left">
    <div class="hdr-logo">&#11041; Aran Technologies</div>
    <span style="color:var(--dim)">|</span>
    <div class="hdr-mission">ISR GCS v13 — 50m/s &#9889; 100m AGL — 360&#176; LiDAR | 4 TARGETS | 3 NFZ | 24 SCENARIOS</div>
  </div>
  <div class="hdr-right">
    <span><div class="dot" id="conn-dot"></div><span id="conn-txt">OFFLINE</span></span>
    <span><div class="dot" id="gps-dot"></div><span id="gps-txt">GPS</span></span>
    <span>RECONNECTS: <b id="reconnect-count" style="color:var(--warn)">0</b></span>
    <span>ELAPSED: <b id="elapsed" style="color:var(--text)">00:00:00</b></span>
    <span>PHASE: <b id="hdr-phase" style="color:var(--warn)">STANDBY</b></span>
    <span id="lidar-hdr" style="color:var(--textdim)">360&#176; LiDAR: --</span>
    <span class="nfz-breach-badge" id="nfz-breach-hdr">&#9888; NFZ BREACH</span>
    <span style="color:var(--accent2)">OSM &#183; LEAFLET</span>
  </div>
</div>

<div class="layout">

<!-- LEFT PANEL -->
<div class="lpanel">
  <div class="card">
    <div class="ctitle">&#9656; Vehicle Status</div>
    <div class="armed-badge disarmed" id="armed-badge">DISARMED</div>
    <div class="metric">
      <span class="mlabel">FLIGHT MODE</span>
      <span class="mval" id="flight-mode" style="font-size:0.75rem">---</span>
    </div>
    <div class="metric">
      <span class="mlabel">HEADING</span>
      <span><span class="mval" id="hdg">0.0</span><span class="munit">&#176;</span></span>
    </div>
    <div class="metric">
      <span class="mlabel">GPS</span>
      <span class="mval" id="gps-status" style="font-size:0.72rem;color:var(--textdim)">---</span>
    </div>
  </div>

  <div class="card">
    <div class="ctitle">&#9656; Gauges</div>
    <div class="gauges-row">
      <div style="text-align:center">
        <canvas id="altGauge" width="100" height="100"></canvas>
        <div style="font-family:var(--mono);font-size:0.55rem;color:var(--textdim);margin-top:2px">ALTITUDE</div>
      </div>
      <div style="text-align:center">
        <canvas id="spdGauge" width="100" height="100"></canvas>
        <div style="font-family:var(--mono);font-size:0.55rem;color:var(--textdim);margin-top:2px">SPEED</div>
      </div>
    </div>
  </div>

  <div class="card">
    <div class="ctitle">&#9656; Power</div>
    <div class="metric">
      <span class="mlabel">BATTERY</span>
      <span><span class="mval" id="bat-pct">100.0</span><span class="munit">%</span></span>
    </div>
    <div class="bat-bar-bg"><div class="bat-bar" id="bat-bar" style="width:100%;background:var(--accent2)"></div></div>
    <div class="bat-label" id="bat-label">NOMINAL</div>
  </div>

  <div class="card">
    <div class="ctitle">&#9656; Compass</div>
    <div class="compass-wrap"><canvas id="compass" width="130" height="130"></canvas></div>
  </div>
</div>

<!-- CENTER -->
<div class="center">
  <div class="map-wrap" id="mapWrap">
    <div class="alert-banner" id="alert-banner">&#9888; OBSTACLE DETECTED</div>

    <div class="map-controls">
      <button class="map-btn active" id="btn-follow" onclick="toggleFollow()">&#9654; FOLLOW</button>
      <button class="map-btn" id="btn-sectors" onclick="toggleSectors()">&#9671; SECTORS</button>
      <button class="map-btn active" id="btn-nfz" onclick="toggleNFZ()">&#9940; NFZ</button>
      <button class="map-btn active" id="btn-targets" onclick="toggleSecTargets()">&#11210; TARGETS</button>
      <a href="/download_log" class="map-btn" style="text-decoration:none;display:block;text-align:center">&#8681; CSV LOG</a>
    </div>

    <div class="map-legend">
      <div class="legend-row"><div class="ls" style="background:#00c8ff"></div>Drone trail</div>
      <div class="legend-row"><div class="ls ls-dashed" style="border-color:rgba(255,179,0,0.7);width:22px"></div>LiDAR warn 25m</div>
      <div class="legend-row"><div class="ls ls-dashed" style="border-color:rgba(255,61,61,0.8);width:22px"></div>LiDAR avoid 15m</div>
      <div class="legend-row"><div class="ls ls-dashed" style="border-color:rgba(0,255,157,0.6);width:22px"></div>Survey grid</div>
      <div class="legend-row"><div class="ls ls-dashed" style="border-color:rgba(170,68,255,0.7);width:22px"></div>Primary orbit</div>
      <div class="legend-row"><div class="ls ls-dashed" style="border-color:rgba(100,180,255,0.7);width:22px"></div>Secondary orbits</div>
      <div class="legend-row"><div class="ls" style="background:rgba(255,61,61,0.4)"></div>No-fly zones</div>
      <div class="legend-row"><div class="ls" style="background:#ffb300"></div>Detour / Loiter WP</div>
      <div class="legend-row"><div class="ls" style="background:rgba(0,200,255,0.3)"></div>LiDAR sectors</div>
    </div>
    <div id="leaflet-map" style="width:100%;height:100%;"></div>
  </div>

  <div class="chart-strip">
    <div class="chart-title">&#9672; ALTITUDE (m) &amp; GROUNDSPEED (m/s) — LIVE</div>
    <canvas id="lineChart" style="height:110px"></canvas>
  </div>
</div>

<!-- RIGHT PANEL -->
<div class="rpanel">
  <div class="card">
    <div class="ctitle">&#9656; Mission Phase</div>
    <div class="phases">
      <div class="phase-item" data-phase="STANDBY">  <div class="phase-dot"></div>STANDBY</div>
      <div class="phase-item" data-phase="TAKEOFF">  <div class="phase-dot"></div>TAKEOFF</div>
      <div class="phase-item" data-phase="SURVEY">   <div class="phase-dot"></div>ISR SURVEY GRID</div>
      <div class="phase-item" data-phase="LOITER">   <div class="phase-dot"></div>PRIMARY TARGET ORBIT</div>
      <div class="phase-item" data-phase="SEC-1">    <div class="phase-dot"></div>SEC TARGET &#945;&#8202;ALPHA-2</div>
      <div class="phase-item" data-phase="SEC-2">    <div class="phase-dot"></div>SEC TARGET &#946;&#8202;BRAVO-1</div>
      <div class="phase-item" data-phase="SEC-3">    <div class="phase-dot"></div>SEC TARGET &#947;&#8202;CHARLIE-3</div>
      <div class="phase-item" data-phase="RTL">      <div class="phase-dot"></div>RETURN TO LAUNCH</div>
      <div class="phase-item" data-phase="LANDING">  <div class="phase-dot"></div>LANDING</div>
    </div>
  </div>

  <div class="card">
    <div class="ctitle">&#9656; Waypoint Progress</div>
    <div class="metric">
      <span class="mlabel">CURRENT WP</span>
      <span class="mval"><span id="wp-cur">0</span> / <span id="wp-tot">0</span></span>
    </div>
    <div class="metric">
      <span class="mlabel">ETA</span>
      <span><span class="mval" id="eta-display" style="font-size:0.75rem;color:var(--warn)">---</span><span class="munit">s</span></span>
    </div>
    <div class="wp-bar-bg"><div class="wp-bar" id="wp-bar" style="width:0%"></div></div>
    <div class="wp-label"><span>START</span><span id="wp-pct">0%</span><span>COMPLETE</span></div>
  </div>

  <div class="card">
    <div class="ctitle" id="lidar-hdr-panel">&#9656; 360&#176; LiDAR: CLEAR</div>
    <div class="metric">
      <span class="mlabel">NEAREST OBS</span>
      <span><span class="mval" id="lidar-dist" style="color:var(--accent2)">---</span><span class="munit">m</span></span>
    </div>
    <div class="metric">
      <span class="mlabel">BEARING</span>
      <span><span class="mval" id="lidar-bearing">---</span><span class="munit">&#176;</span></span>
    </div>
    <div class="metric">
      <span class="mlabel">ESCAPE SIDE</span>
      <span class="mval" id="escape-side" style="font-size:0.72rem;color:var(--warn)">---</span>
    </div>
    <div class="metric">
      <span class="mlabel">SCAN COUNT</span>
      <span class="mval" id="lidar-scans" style="font-size:0.75rem">0</span>
    </div>
    <div class="metric">
      <span class="mlabel">EVENTS</span>
      <span class="mval" id="lidar-events" style="color:var(--warn)">0</span>
    </div>
    <div style="margin-top:6px;font-family:var(--mono);font-size:0.52rem;color:var(--textdim);margin-bottom:2px">PROXIMITY BAR</div>
    <div class="obstacle-bar-bg">
      <div class="obstacle-bar" id="obstacle-bar" style="width:100%;background:var(--accent2)"></div>
    </div>
    <div style="font-family:var(--mono);font-size:0.52rem;color:var(--textdim);margin:5px 0 2px">8&#215;45&#176; SECTOR CLEARANCE</div>
    <div class="sector-row" id="sector-row">
      <div class="sector-cell" id="s0">N</div>
      <div class="sector-cell" id="s1">NE</div>
      <div class="sector-cell" id="s2">E</div>
      <div class="sector-cell" id="s3">SE</div>
      <div class="sector-cell" id="s4">S</div>
      <div class="sector-cell" id="s5">SW</div>
      <div class="sector-cell" id="s6">W</div>
      <div class="sector-cell" id="s7">NW</div>
    </div>
    <div class="lidar-bearing-wrap">
      <canvas id="bearingCanvas" width="100" height="100"></canvas>
    </div>
    <div id="avoid-status" style="text-align:center;font-family:var(--mono);font-size:0.65rem;
         color:var(--accent2);padding:4px;border:1px solid var(--border);border-radius:3px;margin-top:4px">
      CLEAR
    </div>
    <div id="timeout-status" style="display:none;text-align:center;font-family:var(--mono);font-size:0.6rem;
         color:var(--warn);padding:3px;border:1px solid var(--warn);border-radius:3px;margin-top:4px">
      CLIMB ESCAPE ACTIVE
    </div>
  </div>

  <div class="card">
    <div class="ctitle">&#9656; Navigation</div>
    <div class="metric"><span class="mlabel">LATITUDE</span>  <span class="mval" id="lat" style="font-size:0.72rem">---</span></div>
    <div class="metric"><span class="mlabel">LONGITUDE</span> <span class="mval" id="lon" style="font-size:0.72rem">---</span></div>
    <div class="metric"><span class="mlabel">V/SPEED</span>   <span><span class="mval" id="vspd">0.0</span><span class="munit">m/s</span></span></div>
  </div>

  <div class="card">
    <div class="ctitle">&#9656; ISR Target Queue</div>
    <div id="target-list">
      <div class="target-row active" id="tgt-primary">
        <div class="target-dot"></div>
        <span>PRIMARY</span>
        <span class="target-dist" id="tgt-primary-dist">---m</span>
      </div>
      <div class="target-row" id="tgt-sec-0">
        <div class="target-dot"></div>
        <span style="font-size:0.54rem">ALPHA-2 Industrial</span>
        <span class="target-dist" id="tgt-sec-0-dist">---m</span>
      </div>
      <div class="target-row" id="tgt-sec-1">
        <div class="target-dot"></div>
        <span style="font-size:0.54rem">BRAVO-1 River Cross</span>
        <span class="target-dist" id="tgt-sec-1-dist">---m</span>
      </div>
      <div class="target-row" id="tgt-sec-2">
        <div class="target-dot"></div>
        <span style="font-size:0.54rem">CHARLIE-3 Treeline</span>
        <span class="target-dist" id="tgt-sec-2-dist">---m</span>
      </div>
    </div>
    <div style="font-family:var(--mono);font-size:0.52rem;color:var(--textdim);margin-top:6px">
      TARGETS ACQUIRED: <span id="tgt-acquired" style="color:var(--accent2)">0</span> / 4
    </div>
  </div>

  <div class="card">
    <div class="ctitle">&#9656; No-Fly Zone Monitor</div>
    <div id="nfz-list" style="display:flex;flex-direction:column;gap:3px">
      <div class="metric">
        <span class="mlabel">NFZ-1 INFRA</span>
        <span class="mval" id="nfz-0-status" style="font-size:0.62rem;color:var(--accent2)">CLEAR</span>
      </div>
      <div class="metric">
        <span class="mlabel">NFZ-2 APPROACH</span>
        <span class="mval" id="nfz-1-status" style="font-size:0.62rem;color:var(--accent2)">CLEAR</span>
      </div>
      <div class="metric">
        <span class="mlabel">NFZ-3 COMMS</span>
        <span class="mval" id="nfz-2-status" style="font-size:0.62rem;color:var(--accent2)">CLEAR</span>
      </div>
    </div>
  </div>

  <div class="card">
    <div class="ctitle">&#9656; PID Live Tune</div>
    <select class="pid-ctrl-select" id="pid-ctrl-select">
      <option value="avoidance">Avoidance PID</option>
      <option value="altitude">Altitude PID</option>
      <option value="orbit">Orbit PID</option>
    </select>
    <div class="pid-row">
      <span class="pid-label">Kp</span>
      <input class="pid-input" id="pid-kp" type="number" step="0.1" value="1.5">
      <span class="pid-label">Ki</span>
      <input class="pid-input" id="pid-ki" type="number" step="0.01" value="0.0">
    </div>
    <div class="pid-row">
      <span class="pid-label">Kd</span>
      <input class="pid-input" id="pid-kd" type="number" step="0.1" value="0.6">
      <span class="pid-label">Lim</span>
      <input class="pid-input" id="pid-lim" type="number" step="1" value="15">
    </div>
    <button class="pid-send" onclick="sendPIDTune()">&#9654; SEND GAINS</button>
    <div id="pid-status" style="font-family:var(--mono);font-size:0.55rem;color:var(--textdim);margin-top:4px"></div>
  </div>

  <div class="card" style="flex:1;display:flex;flex-direction:column;min-height:0">
    <div class="ctitle">&#9656; System Log</div>
    <div class="log-wrap" id="log">
      <div class="le"><span class="lt">[INIT] </span><span class="lm">GCS v13 — multi-target | NFZ | loiter WPs</span></div>
      <div class="le"><span class="lt">[INIT] </span><span class="lm">Connecting SITL udp://:14540</span></div>
    </div>
  </div>
</div>
</div>

<div class="footer">
  <span>ARAN TECHNOLOGIES PVT LTD — ISR DEMO BUILD v13.0</span>
  <span>NIRMAAN INCUBATION &#183; IIT HYDERABAD &#183; 2026</span>
  <span>LiDAR: 360&#176; PID v13 | 4 Targets | 3 NFZ | 24 Scenarios | LEAFLET + OSM</span>
</div>

<script>
const socket = io();

// Coordinates injected from server — single source of truth via mission_config.py
const HOME_LAT           = {{ home_lat }};
const HOME_LON           = {{ home_lon }};
const TARGET_LAT_JS      = {{ target_lat }};
const TARGET_LON_JS      = {{ target_lon }};
const ORBIT_RADIUS_M     = {{ orbit_radius }};
const SECONDARY_TARGETS_INIT = {{ secondary_targets_json | safe }};
const NFZ_ZONES_INIT         = {{ nfz_zones_json | safe }};
const LOITER_WPS_INIT        = {{ loiter_waypoints_json | safe }};
const LIDAR_RANGE    = 25.0;
const SECTOR_LABELS  = ['N','NE','E','SE','S','SW','W','NW'];

// ══════════════════════════════════════════════════════════
//  MAP INIT
// ══════════════════════════════════════════════════════════
const map = L.map('leaflet-map', { center:[HOME_LAT,HOME_LON], zoom:17, zoomControl:true });
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
  attribution:'&copy; OpenStreetMap', maxZoom:20
}).addTo(map);

let followMode = true;
let sectorsVisible = false;

function toggleFollow(){
  followMode = !followMode;
  const btn = document.getElementById('btn-follow');
  btn.classList.toggle('active', followMode);
  btn.textContent = followMode ? '\u25BA FOLLOW' : '\u25A0 FREE';
}
function toggleSectors(){
  sectorsVisible = !sectorsVisible;
  document.getElementById('btn-sectors').classList.toggle('active', sectorsVisible);
  if(!sectorsVisible){ clearSectorOverlay(); }
}

const homeIcon = L.divIcon({
  html:`<div style="position:relative;width:16px;height:16px">
    <div style="position:absolute;top:7px;left:0;width:16px;height:2px;background:#ffb300;box-shadow:0 0 4px #ffb300"></div>
    <div style="position:absolute;top:0;left:7px;width:2px;height:16px;background:#ffb300;box-shadow:0 0 4px #ffb300"></div>
  </div>`,
  iconSize:[16,16], iconAnchor:[8,8], className:''
});
L.marker([HOME_LAT,HOME_LON], {icon:homeIcon})
  .bindTooltip('HOME / BASE ALPHA', {permanent:true,direction:'right',offset:[10,0]})
  .addTo(map);

const targetIcon = L.divIcon({
  html:`<div style="width:14px;height:14px;border:2px solid #aa44ff;border-radius:50%;
    background:rgba(170,68,255,0.25);box-shadow:0 0 10px #aa44ff"></div>`,
  iconSize:[14,14], iconAnchor:[7,7], className:''
});
L.marker([TARGET_LAT_JS,TARGET_LON_JS], {icon:targetIcon})
  .bindTooltip('ISR TARGET', {permanent:true,direction:'right',offset:[10,0]})
  .addTo(map);

L.circle([TARGET_LAT_JS,TARGET_LON_JS], {
  radius:ORBIT_RADIUS_M, color:'#aa44ff', weight:1.5, dashArray:'6 5',
  fillColor:'#aa44ff', fillOpacity:0.05
}).bindTooltip('ORBIT ZONE 45m', {direction:'top'}).addTo(map);

let surveyLayer = null, surveyDrawn = false;
function drawSurveyGrid(wps){
  if(surveyLayer){ map.removeLayer(surveyLayer); }
  surveyLayer = L.layerGroup();
  L.polyline(wps.map(([la,lo])=>[la,lo]), {
    color:'#00ff9d', weight:1.5, opacity:0.5, dashArray:'4 4'
  }).addTo(surveyLayer);
  const lats=wps.map(w=>w[0]), lons=wps.map(w=>w[1]);
  L.rectangle([[Math.min(...lats),Math.min(...lons)],[Math.max(...lats),Math.max(...lons)]],
    {color:'#00ff9d',weight:1,opacity:0.2,fill:true,fillOpacity:0.04}).addTo(surveyLayer);
  const wpIcon = L.divIcon({
    html:`<div style="width:7px;height:7px;border-radius:50%;background:#00ff9d;border:1px solid #fff;box-shadow:0 0 5px #00ff9d"></div>`,
    iconSize:[7,7], iconAnchor:[3,3], className:''
  });
  wps.forEach(([la,lo],i) => {
    L.marker([la,lo],{icon:wpIcon}).bindTooltip(`WP ${i+1}`,{direction:'top',offset:[0,-5]}).addTo(surveyLayer);
  });
  surveyLayer.addTo(map);
}

function makeDroneHtml(headingDeg){
  return `<div class="drone-icon" style="transform:rotate(${headingDeg}deg)">
    <div class="drone-arrow"></div>
    <div class="drone-body"></div>
  </div>`;
}
const droneIcon = L.divIcon({ html:makeDroneHtml(0), iconSize:[20,20], iconAnchor:[10,10], className:'' });
const droneMarker = L.marker([HOME_LAT,HOME_LON], {icon:droneIcon, zIndexOffset:1000, opacity:0});
let droneOnMap = false;

const trailLine = L.polyline([], {color:'#00c8ff',weight:2,opacity:0.7}).addTo(map);

const lidarWarnRing = L.circle([HOME_LAT,HOME_LON], {
  radius:25, color:'#ffb300', weight:1.5, dashArray:'4 4',
  fillColor:'#ffb300', fillOpacity:0.0, opacity:0
});
const lidarAvoidRing = L.circle([HOME_LAT,HOME_LON], {
  radius:15, color:'#ff3d3d', weight:1.5, dashArray:'3 3',
  fillColor:'#ff3d3d', fillOpacity:0.0, opacity:0
});

let obstacleRayLayer=null, obstacleEndDot=null, detourMarker=null, detourLine=null;
let sectorPolygons=[];

function clearObstacleRay(){
  if(obstacleRayLayer){ map.removeLayer(obstacleRayLayer); obstacleRayLayer=null; }
  if(obstacleEndDot)  { map.removeLayer(obstacleEndDot);   obstacleEndDot=null; }
}
function clearDetour(){
  if(detourMarker){ map.removeLayer(detourMarker); detourMarker=null; }
  if(detourLine)  { map.removeLayer(detourLine);   detourLine=null; }
}
function clearSectorOverlay(){
  sectorPolygons.forEach(p => map.removeLayer(p));
  sectorPolygons=[];
}

const detourLeafIcon = L.divIcon({
  html:'<div class="detour-icon"></div>',
  iconSize:[12,12], iconAnchor:[6,6], className:''
});

function drawSectorOverlay(lat, lon, sectors){
  clearSectorOverlay();
  if(!sectors || !sectors.length) return;
  const sectorSize = 360.0 / sectors.length;
  sectors.forEach((dist, s) => {
    const startBear = s * sectorSize;
    const endBear   = startBear + sectorSize;
    const pts = [[lat,lon]];
    for(let a=startBear; a<=endBear; a+=5){
      const rad = a * Math.PI / 180;
      pts.push([
        lat + (Math.min(dist,25)/111320)*Math.cos(rad),
        lon + (Math.min(dist,25)/(111320*Math.cos(lat*Math.PI/180)))*Math.sin(rad)
      ]);
    }
    pts.push([lat,lon]);
    let col = '#00ff9d'; let alpha = 0.06;
    if(dist <= 15)      { col='#ff3d3d'; alpha=0.15; }
    else if(dist <= 25) { col='#ffb300'; alpha=0.10; }
    const poly = L.polygon(pts, {
      color:col, weight:1, opacity:0.6,
      fillColor:col, fillOpacity:alpha
    }).addTo(map);
    sectorPolygons.push(poly);
  });
}

// ══════════════════════════════════════════════════════════
//  SECONDARY TARGETS, NFZ ZONES, LOITER WPs — drawn once from first telemetry
// ══════════════════════════════════════════════════════════
let staticLayersDrawn = false;
let nfzVisible = true;
let secTargetsVisible = true;
let nfzLayers = [];
let secTargetLayers = [];
let loiterLayers = [];

function toggleNFZ(){
  nfzVisible = !nfzVisible;
  document.getElementById('btn-nfz').classList.toggle('active', nfzVisible);
  nfzLayers.forEach(l => nfzVisible ? map.addLayer(l) : map.removeLayer(l));
}
function toggleSecTargets(){
  secTargetsVisible = !secTargetsVisible;
  document.getElementById('btn-targets').classList.toggle('active', secTargetsVisible);
  secTargetLayers.forEach(l => secTargetsVisible ? map.addLayer(l) : map.removeLayer(l));
  loiterLayers.forEach(l => secTargetsVisible ? map.addLayer(l) : map.removeLayer(l));
}

function drawStaticLayers(d){
  if(staticLayersDrawn) return;
  staticLayersDrawn = true;

  // ── No-fly zones ──────────────────────────────────────
  (d.nfz_zones || []).forEach((nfz, i) => {
    const circle = L.circle([nfz.lat, nfz.lon], {
      radius: nfz.radius_m,
      color: '#ff3d3d', weight: 1.5, dashArray: '5 4',
      fillColor: '#ff3d3d', fillOpacity: 0.07, opacity: 0.8
    }).bindTooltip(`&#9940; ${nfz.name}`, {direction: 'top', permanent: false});
    const label = L.marker([nfz.lat, nfz.lon], {
      icon: L.divIcon({
        html: `<div style="font-family:'Share Tech Mono',monospace;font-size:0.45rem;color:#ff3d3d;
               white-space:nowrap;text-shadow:0 0 4px rgba(255,61,61,0.8);pointer-events:none">
               &#9940; NFZ-${i+1}</div>`,
        iconSize: [60, 14], iconAnchor: [30, 7], className: ''
      }), zIndexOffset: -100
    });
    nfzLayers.push(circle, label);
    circle.addTo(map); label.addTo(map);
  });
  addLog(`NFZ overlays drawn — ${(d.nfz_zones||[]).length} zones`, 'lwarn');

  // ── Secondary ISR targets ─────────────────────────────
  const secColors = ['#64b4ff', '#88ddff', '#aaeeff'];
  (d.secondary_targets || []).forEach((tgt, i) => {
    const col = secColors[i % secColors.length];
    const secIcon = L.divIcon({
      html: `<div style="width:12px;height:12px;border:2px solid ${col};
             transform:rotate(45deg);background:rgba(100,180,255,0.2);
             box-shadow:0 0 8px ${col}"></div>`,
      iconSize: [12,12], iconAnchor: [6,6], className: ''
    });
    const marker = L.marker([tgt.lat, tgt.lon], {icon: secIcon})
      .bindTooltip(`${tgt.name || 'SEC-'+i}<br>&#9711; ${tgt.orbit_radius_m}m  ${tgt.orbit_duration_s}s`,
                   {direction: 'right', offset: [10,0]});
    const ring = L.circle([tgt.lat, tgt.lon], {
      radius: tgt.orbit_radius_m,
      color: col, weight: 1.2, dashArray: '6 5',
      fillColor: col, fillOpacity: 0.03, opacity: 0.6
    }).bindTooltip(`Orbit zone ${tgt.orbit_radius_m}m`, {direction: 'top'});
    secTargetLayers.push(marker, ring);
    marker.addTo(map); ring.addTo(map);
  });
  if((d.secondary_targets||[]).length)
    addLog(`Secondary targets drawn — ${d.secondary_targets.length} objectives`, 'lok');

  // ── Loiter waypoints ─────────────────────────────────
  (d.loiter_waypoints || []).forEach((lw, i) => {
    const loiterIcon = L.divIcon({
      html: `<div style="width:13px;height:13px;border:2px solid #ffb300;border-radius:2px;
             background:rgba(255,179,0,0.2);box-shadow:0 0 7px #ffb300;
             display:flex;align-items:center;justify-content:center;
             font-size:7px;color:#ffb300">&#9673;</div>`,
      iconSize: [13,13], iconAnchor: [6,6], className: ''
    });
    const lm = L.marker([lw.lat, lw.lon], {icon: loiterIcon})
      .bindTooltip(`${lw.name}<br>Hold: ${lw.loiter_time_s}s  alt:${lw.altitude_m}m`,
                   {direction: 'top', offset: [0,-8]});
    loiterLayers.push(lm);
    lm.addTo(map);
  });
  if((d.loiter_waypoints||[]).length)
    addLog(`Loiter WPs drawn — ${d.loiter_waypoints.length} surveillance holds`, 'lok');
}

// ══════════════════════════════════════════════════════════
//  HAVERSINE (JS) for target distance display
// ══════════════════════════════════════════════════════════
function haversineM(lat1,lon1,lat2,lon2){
  const R=6371000, toR=Math.PI/180;
  const dphi=(lat2-lat1)*toR, dlam=(lon2-lon1)*toR;
  const a=Math.sin(dphi/2)**2+Math.cos(lat1*toR)*Math.cos(lat2*toR)*Math.sin(dlam/2)**2;
  return R*2*Math.atan2(Math.sqrt(a),Math.sqrt(1-a));
}

// ══════════════════════════════════════════════════════════
//  TARGET QUEUE + NFZ PANEL UPDATERS
// ══════════════════════════════════════════════════════════
let targetsAcquired = 0;

function updateTargetPanel(d){
  const lat=d.lat, lon=d.lon;
  const phase=d.mission_phase||'STANDBY';

  // Primary target
  const pDist = haversineM(lat,lon, d.target_lat, d.target_lon);
  document.getElementById('tgt-primary-dist').textContent = pDist<9999 ? pDist.toFixed(0)+'m' : '---';
  const pRow = document.getElementById('tgt-primary');
  if(phase==='LOITER'){ pRow.classList.add('active'); pRow.classList.remove('done'); }
  else if(['SEC-1','SEC-2','SEC-3','RTL','LANDING'].includes(phase)){ pRow.classList.remove('active'); pRow.classList.add('done'); }
  else { pRow.classList.remove('active','done'); }

  // Secondary targets
  (d.secondary_targets||[]).forEach((tgt,i) => {
    const el = document.getElementById('tgt-sec-'+i);
    const distEl = document.getElementById('tgt-sec-'+i+'-dist');
    if(!el) return;
    const sDist = haversineM(lat,lon, tgt.lat, tgt.lon);
    distEl.textContent = sDist.toFixed(0)+'m';
    const activePhase = 'SEC-'+(i+1);
    if(phase===activePhase){ el.classList.add('active'); el.classList.remove('done'); }
    else if(PHASE_ORDER.indexOf(phase)>PHASE_ORDER.indexOf(activePhase)){ el.classList.remove('active'); el.classList.add('done'); }
    else { el.classList.remove('active','done'); }
  });

  // Acquired count
  const acq = ['LOITER','SEC-1','SEC-2','SEC-3','RTL','LANDING'].filter(p=>PHASE_ORDER.indexOf(phase)>=PHASE_ORDER.indexOf(p)).length;
  document.getElementById('tgt-acquired').textContent = Math.min(acq, 4);
}

function updateNFZPanel(d){
  const lat=d.lat, lon=d.lon;
  let anyBreach = false;
  (d.nfz_zones||[]).forEach((nfz,i) => {
    const el = document.getElementById('nfz-'+i+'-status');
    if(!el) return;
    const dist = haversineM(lat,lon, nfz.lat, nfz.lon);
    const margin = dist - nfz.radius_m;
    if(margin < 0){
      el.textContent = 'BREACH'; el.style.color = 'var(--danger)';
      anyBreach = true;
    } else if(margin < 30){
      el.textContent = Math.round(margin)+'m'; el.style.color = 'var(--warn)';
    } else {
      el.textContent = 'CLEAR'; el.style.color = 'var(--accent2)';
    }
  });
  const badge = document.getElementById('nfz-breach-hdr');
  badge.classList.toggle('active', anyBreach);
  if(anyBreach && !prevNFZBreach) addLog('NFZ BREACH DETECTED — avoidance active', 'ldanger');
  if(!anyBreach && prevNFZBreach) addLog('NFZ cleared — resuming normal flight', 'lwarn');
  prevNFZBreach = anyBreach;
}
function drawGauge(id,value,max,unit,color){
  const c=document.getElementById(id),cx=c.getContext('2d'),W=c.width,R=W/2,r=R-10;
  cx.clearRect(0,0,W,W);
  cx.beginPath();cx.arc(R,R,r,0.75*Math.PI,2.25*Math.PI);cx.strokeStyle='#0b1828';cx.lineWidth=8;cx.lineCap='round';cx.stroke();
  const end=0.75*Math.PI+Math.min(value/max,1)*1.5*Math.PI;
  cx.beginPath();cx.arc(R,R,r,0.75*Math.PI,end);cx.strokeStyle=color;cx.lineWidth=8;cx.lineCap='round';cx.stroke();
  cx.fillStyle='#c8dff0';cx.font='bold 18px Share Tech Mono';cx.textAlign='center';cx.textBaseline='middle';cx.fillText(value.toFixed(1),R,R-4);
  cx.fillStyle='#4a7a9b';cx.font='9px Share Tech Mono';cx.fillText(unit,R,R+14);
}

function drawCompass(deg){
  const c=document.getElementById('compass'),cx=c.getContext('2d'),W=c.width,R=W/2,r=R-8;
  cx.clearRect(0,0,W,W);
  cx.beginPath();cx.arc(R,R,r,0,2*Math.PI);cx.strokeStyle='#0e2340';cx.lineWidth=2;cx.stroke();cx.fillStyle='#060b14';cx.fill();
  for(let i=0;i<36;i++){
    const a=i*10*Math.PI/180,len=i%9===0?10:5;
    cx.beginPath();cx.moveTo(R+Math.cos(a)*(r-len),R+Math.sin(a)*(r-len));cx.lineTo(R+Math.cos(a)*(r-2),R+Math.sin(a)*(r-2));
    cx.strokeStyle=i%9===0?'#1e4060':'#0e2340';cx.lineWidth=1;cx.stroke();
  }
  [['N',270,'#ff5555'],['E',0,'#4a7a9b'],['S',90,'#4a7a9b'],['W',180,'#4a7a9b']].forEach(([l,a,col])=>{
    const rad=(a-90)*Math.PI/180;
    cx.fillStyle=col;cx.font='bold 10px Share Tech Mono';cx.textAlign='center';cx.textBaseline='middle';
    cx.fillText(l,R+Math.cos(rad)*(r-18),R+Math.sin(rad)*(r-18));
  });
  const a=(deg-90)*Math.PI/180;
  cx.save();cx.translate(R,R);cx.rotate(a);
  cx.beginPath();cx.moveTo(0,-(r-22));cx.lineTo(-5,10);cx.lineTo(5,10);cx.closePath();
  cx.fillStyle='#00c8ff';cx.shadowColor='#00c8ff';cx.shadowBlur=12;cx.fill();cx.restore();
  cx.beginPath();cx.arc(R,R,4,0,2*Math.PI);cx.fillStyle='#00c8ff';cx.fill();
  cx.fillStyle='#c8dff0';cx.font='bold 11px Share Tech Mono';cx.textAlign='center';cx.textBaseline='middle';cx.fillText(Math.round(deg)+'°',R,R+28);
}

function drawBearing(deg,dist){
  const c=document.getElementById('bearingCanvas'),cx=c.getContext('2d'),W=c.width,R=W/2,r=R-8;
  cx.clearRect(0,0,W,W);
  cx.beginPath();cx.arc(R,R,r,0,2*Math.PI);cx.strokeStyle='#0e2340';cx.lineWidth=1.5;cx.stroke();cx.fillStyle='#060b14';cx.fill();
  const col=dist<=6?'#ff3d3d':dist<=15?'#ffb300':'#00ff9d';
  const a=(deg-90)*Math.PI/180;
  cx.save();cx.translate(R,R);cx.rotate(a);
  cx.beginPath();cx.moveTo(0,-(r-10));cx.lineTo(-4,8);cx.lineTo(4,8);cx.closePath();
  cx.fillStyle=col;cx.shadowColor=col;cx.shadowBlur=8;cx.fill();cx.restore();
  cx.beginPath();cx.arc(R,R,3,0,2*Math.PI);cx.fillStyle=col;cx.fill();
  cx.fillStyle='#4a7a9b';cx.font='8px Share Tech Mono';cx.textAlign='center';cx.textBaseline='middle';cx.fillText(Math.round(deg)+'°',R,R+20);
}

// ══════════════════════════════════════════════════════════
//  LINE CHART
// ══════════════════════════════════════════════════════════
const altHistory=[],spdHistory=[],labels=[];
const chart=new Chart(document.getElementById('lineChart').getContext('2d'),{
  type:'line',
  data:{labels,datasets:[
    {label:'Alt (m)',   data:altHistory,borderColor:'#00c8ff',backgroundColor:'rgba(0,200,255,0.08)',borderWidth:1.5,pointRadius:0,fill:true,tension:0.4},
    {label:'Spd (m/s)',data:spdHistory,borderColor:'#00ff9d',backgroundColor:'rgba(0,255,157,0.05)',borderWidth:1.5,pointRadius:0,fill:true,tension:0.4}
  ]},
  options:{
    responsive:true,maintainAspectRatio:false,animation:false,
    plugins:{legend:{labels:{color:'#4a7a9b',font:{size:9,family:'Share Tech Mono'}}}},
    scales:{x:{display:false},y:{grid:{color:'#0b1828'},ticks:{color:'#4a7a9b',font:{size:8,family:'Share Tech Mono'}}}}
  }
});

// ══════════════════════════════════════════════════════════
//  PHASE + LOG
// ══════════════════════════════════════════════════════════
const PHASE_ORDER=['STANDBY','TAKEOFF','SURVEY','LOITER','SEC-1','SEC-2','SEC-3','RTL','LANDING'];
let prevPhase='',prevArmed=false,prevAlt=0,prevAvoidCount=0,logCount=0;
let connectedLogged=false;  // BUG FIX: dedicated flag so "connected" logs exactly once
let prevNFZBreach = false;

function updatePhase(phase){
  if(phase===prevPhase)return; prevPhase=phase;
  document.getElementById('hdr-phase').textContent=phase;
  const phaseIdx=PHASE_ORDER.indexOf(phase);
  document.querySelectorAll('.phase-item').forEach(el=>{
    const elIdx=PHASE_ORDER.indexOf(el.dataset.phase);
    el.classList.remove('active','done');
    if(el.dataset.phase===phase)el.classList.add('active');
    else if(elIdx<phaseIdx)el.classList.add('done');
  });
  addLog('Mission phase: '+phase, phase==='LOITER'?'lwarn':'lok');
}

function addLog(msg,type='lm'){
  const log=document.getElementById('log');
  const div=document.createElement('div'); div.className='le';
  div.innerHTML=`<span class="lt">[${new Date().toTimeString().slice(0,8)}] </span><span class="${type}">${msg}</span>`;
  log.appendChild(div); log.scrollTop=log.scrollHeight; logCount++;
}

// ══════════════════════════════════════════════════════════
//  SECTOR PANEL UPDATE
// ══════════════════════════════════════════════════════════
function updateSectorPanel(sectors){
  if(!sectors) return;
  sectors.forEach((dist,s) => {
    const el = document.getElementById('s'+s);
    if(!el) return;
    const val = dist > 100 ? '' : dist.toFixed(0);
    el.textContent = SECTOR_LABELS[s] + (val ? ' '+val : '');
    if(dist <= 15)      { el.style.background='rgba(255,61,61,0.25)';  el.style.color='#ff3d3d'; el.style.borderColor='#ff3d3d'; }
    else if(dist <= 25) { el.style.background='rgba(255,179,0,0.18)'; el.style.color='#ffb300'; el.style.borderColor='#ffb300'; }
    else                { el.style.background='rgba(0,255,157,0.07)'; el.style.color='#4a7a9b'; el.style.borderColor='#0e2340'; }
  });
}

// ══════════════════════════════════════════════════════════
//  MAP UPDATE
// ══════════════════════════════════════════════════════════
function updateMap(d){
  const lat=d.lat, lon=d.lon, lidar=d.lidar;

  if(!droneOnMap && d.connected){
    droneMarker.setOpacity(1).addTo(map);
    lidarWarnRing.setStyle({fillOpacity:0.03,opacity:1}).addTo(map);
    lidarAvoidRing.setStyle({fillOpacity:0.06,opacity:1}).addTo(map);
    droneOnMap = true;
    addLog('Live position acquired — drone marker active','lok');
  }

  droneMarker.setLatLng([lat,lon]);
  droneMarker.setIcon(L.divIcon({
    html: makeDroneHtml(d.heading),
    iconSize:[20,20], iconAnchor:[10,10], className:''
  }));

  if(followMode && droneOnMap){
    map.panTo([lat,lon], {animate:true, duration:0.3});
  }

  lidarWarnRing.setLatLng([lat,lon]);
  lidarAvoidRing.setLatLng([lat,lon]);

  if(lidar.nearest_dist<=15){
    lidarAvoidRing.setStyle({color:'#ff3d3d',fillOpacity:0.14,weight:2.5});
    lidarWarnRing.setStyle({color:'#ffb300',fillOpacity:0.08,weight:2});
  } else if(lidar.nearest_dist<=25){
    lidarWarnRing.setStyle({color:'#ffb300',fillOpacity:0.07,weight:2});
    lidarAvoidRing.setStyle({color:'#ff3d3d',fillOpacity:0.06,weight:1.5});
  } else {
    lidarWarnRing.setStyle({color:'#ffb300',fillOpacity:0.03,weight:1.5});
    lidarAvoidRing.setStyle({color:'#ff3d3d',fillOpacity:0.06,weight:1.5});
  }

  clearObstacleRay();
  if(lidar.nearest_dist < 25){
    const bearRad = lidar.nearest_bearing * Math.PI / 180;
    const rayM    = Math.min(lidar.nearest_dist, 25);
    const dLat    = (rayM / 111320) * Math.cos(bearRad);
    const dLon    = (rayM / (111320 * Math.cos(lat * Math.PI/180))) * Math.sin(bearRad);
    const endLat  = lat + dLat, endLon = lon + dLon;
    const rayCol  = lidar.nearest_dist<=15 ? '#ff3d3d' : '#ffb300';
    obstacleRayLayer = L.polyline([[lat,lon],[endLat,endLon]], {
      color:rayCol, weight:2, opacity:0.85, dashArray:'4 3'
    }).addTo(map);
    obstacleEndDot = L.circleMarker([endLat,endLon], {
      radius:5, color:rayCol, fillColor:rayCol, fillOpacity:1, weight:1
    }).addTo(map);
  }

  if(d.trail && d.trail.length>1){
    trailLine.setLatLngs(d.trail.map(([lt,ln])=>[lt,ln]));
  }

  if(lidar.detour_lat && lidar.detour_lon){
    if(!detourMarker){
      detourMarker = L.marker([lidar.detour_lat,lidar.detour_lon], {icon:detourLeafIcon,zIndexOffset:900})
        .bindTooltip('DETOUR WP',{direction:'top',offset:[0,-8]}).addTo(map);
    } else { detourMarker.setLatLng([lidar.detour_lat,lidar.detour_lon]); }
    if(detourLine) map.removeLayer(detourLine);
    detourLine = L.polyline([[lat,lon],[lidar.detour_lat,lidar.detour_lon]], {
      color:'#ffb300', weight:1.5, opacity:0.65, dashArray:'5 4'
    }).addTo(map);
  } else { clearDetour(); }

  if(!surveyDrawn && d.survey_waypoints && d.survey_waypoints.length){
    drawSurveyGrid(d.survey_waypoints);
    surveyDrawn=true;
    addLog('Survey grid drawn — '+d.survey_waypoints.length+' waypoints','lok');
  }

  if(sectorsVisible && lidar.sectors){
    drawSectorOverlay(lat, lon, lidar.sectors);
  } else if(!sectorsVisible && sectorPolygons.length){
    clearSectorOverlay();
  }
}

// ══════════════════════════════════════════════════════════
//  LIDAR PANEL UPDATE
// ══════════════════════════════════════════════════════════
function updateLidarPanel(lidar){
  const dist=lidar.nearest_dist, bearing=lidar.nearest_bearing, active=lidar.avoidance_active;
  const distEl=document.getElementById('lidar-dist');
  distEl.textContent=dist>100?'CLEAR':dist.toFixed(1);
  distEl.style.color=dist<=15?'var(--danger)':dist<=25?'var(--warn)':'var(--accent2)';
  document.getElementById('lidar-bearing').textContent=dist>100?'---':Math.round(bearing);
  document.getElementById('lidar-scans').textContent=lidar.scan_count.toLocaleString();
  document.getElementById('lidar-events').textContent=lidar.avoidance_count;
  document.getElementById('escape-side').textContent=lidar.escape_side||'---';
  document.getElementById('lidar-hdr-panel').textContent='360\u00b0 LiDAR: '+(dist>100?'CLEAR':dist.toFixed(1)+'m');
  document.getElementById('lidar-hdr').textContent='360\u00b0 LiDAR: '+(dist>100?'CLEAR':dist.toFixed(1)+'m');

  const bar=document.getElementById('obstacle-bar');
  bar.style.width=(Math.min(dist/LIDAR_RANGE,1)*100)+'%';
  bar.style.background=dist<=15?'var(--danger)':dist<=25?'var(--warn)':'var(--accent2)';

  drawBearing(dist>100?0:bearing, dist);
  updateSectorPanel(lidar.sectors);

  const statusEl=document.getElementById('avoid-status');
  if(active){
    statusEl.textContent='\u26A0 AVOIDING OBSTACLE'; statusEl.style.color='var(--danger)';
    statusEl.style.borderColor='var(--danger)'; statusEl.style.background='rgba(255,61,61,0.08)';
  } else if(dist<=25){
    statusEl.textContent='WARNING \u2014 OBSTACLE NEAR'; statusEl.style.color='var(--warn)';
    statusEl.style.borderColor='var(--warn)'; statusEl.style.background='rgba(255,179,0,0.06)';
  } else {
    statusEl.textContent='CLEAR'; statusEl.style.color='var(--accent2)';
    statusEl.style.borderColor='var(--border)'; statusEl.style.background='transparent';
  }

  const tEl=document.getElementById('timeout-status');
  tEl.style.display = lidar.timeout_active ? 'block' : 'none';

  const banner=document.getElementById('alert-banner');
  if(active){
    banner.style.display='block';
    banner.textContent=`\u26A0 OBSTACLE \u2014 AVOIDANCE ACTIVE \u2014 BEARING ${Math.round(bearing)}\u00b0  DIST ${dist.toFixed(1)}m  ESCAPE: ${lidar.escape_side||'---'}`;
  } else { banner.style.display='none'; }
}

// ══════════════════════════════════════════════════════════
//  PID LIVE TUNE
// ══════════════════════════════════════════════════════════
const pidDefaults = {
  avoidance: {kp:1.5, ki:0.0, kd:0.6, output_limit:15.0},
  altitude:  {kp:1.2, ki:0.1, kd:0.4, output_limit:3.0},
  orbit:     {kp:0.8, ki:0.05,kd:0.3, output_limit:5.0}
};

document.getElementById('pid-ctrl-select').addEventListener('change', function(){
  const g = pidDefaults[this.value];
  document.getElementById('pid-kp').value  = g.kp;
  document.getElementById('pid-ki').value  = g.ki;
  document.getElementById('pid-kd').value  = g.kd;
  document.getElementById('pid-lim').value = g.output_limit;
});

function sendPIDTune(){
  const ctrl = document.getElementById('pid-ctrl-select').value;
  const payload = {
    controller:   ctrl,
    kp:           parseFloat(document.getElementById('pid-kp').value),
    ki:           parseFloat(document.getElementById('pid-ki').value),
    kd:           parseFloat(document.getElementById('pid-kd').value),
    output_limit: parseFloat(document.getElementById('pid-lim').value),
  };
  fetch('/pid_tune', {
    method:'POST',
    headers:{'Content-Type':'application/json'},
    body: JSON.stringify(payload)
  }).then(r=>r.json()).then(data=>{
    const st = document.getElementById('pid-status');
    if(data.ok){
      st.style.color='var(--accent2)';
      st.textContent=`Sent ${ctrl}: Kp=${payload.kp} Ki=${payload.ki} Kd=${payload.kd}`;
    } else {
      st.style.color='var(--danger)';
      st.textContent='Error: '+data.error;
    }
  }).catch(()=>{
    document.getElementById('pid-status').textContent='Request failed';
  });
}

socket.on('pid_gains', gains => {
  const ctrl = document.getElementById('pid-ctrl-select').value;
  const g = gains[ctrl];
  if(g){
    pidDefaults[ctrl] = g;
    document.getElementById('pid-kp').value  = g.kp;
    document.getElementById('pid-ki').value  = g.ki;
    document.getElementById('pid-kd').value  = g.kd;
    document.getElementById('pid-lim').value = g.output_limit;
  }
});

// ══════════════════════════════════════════════════════════
//  MAIN SOCKET HANDLER
// ══════════════════════════════════════════════════════════
socket.on('telemetry', d => {
  const dot=document.getElementById('conn-dot'), txt=document.getElementById('conn-txt');
  if(d.connected){ dot.classList.add('on'); txt.textContent='ONLINE'; }
  else { dot.classList.remove('on'); txt.textContent='OFFLINE'; }

  const gpsDot=document.getElementById('gps-dot');
  // BUG FIX: HTML header element is id="gps-txt" but code was looking up
  // id="gps-status" which silently resolved to the left-panel card element,
  // leaving the header GPS text frozen at "GPS" forever.
  const gpsTxt=document.getElementById('gps-txt');
  const gpsPanelTxt=document.getElementById('gps-status');
  if(d.gps_ok){
    gpsDot.className='dot on';
    if(gpsTxt)     { gpsTxt.textContent='GPS OK';     gpsTxt.style.color='var(--accent2)'; }
    if(gpsPanelTxt){ gpsPanelTxt.textContent='GPS OK'; gpsPanelTxt.style.color='var(--accent2)'; }
  } else {
    gpsDot.className='dot warn';
    if(gpsTxt)     { gpsTxt.textContent='GPS SEARCH';     gpsTxt.style.color='var(--warn)'; }
    if(gpsPanelTxt){ gpsPanelTxt.textContent='GPS SEARCH'; gpsPanelTxt.style.color='var(--warn)'; }
  }

  document.getElementById('reconnect-count').textContent = d.reconnects||0;
  document.getElementById('elapsed').textContent=d.elapsed;
  document.getElementById('hdg').textContent=d.heading.toFixed(1);
  document.getElementById('lat').textContent=d.lat.toFixed(6)+'\u00b0';
  document.getElementById('lon').textContent=d.lon.toFixed(6)+'\u00b0';
  document.getElementById('vspd').textContent=d.vspeed.toFixed(1);
  document.getElementById('flight-mode').textContent=d.flight_mode;

  const etaEl=document.getElementById('eta-display');
  if(d.eta_seconds != null && d.eta_seconds > 0){
    const m=Math.floor(d.eta_seconds/60), s=d.eta_seconds%60;
    etaEl.textContent = m>0 ? `${m}m ${s}s` : `${s}s`;
  } else { etaEl.textContent='---'; }

  const badge=document.getElementById('armed-badge');
  badge.textContent=d.armed?'ARMED':'DISARMED';
  badge.className='armed-badge '+(d.armed?'armed':'disarmed');
  if(d.armed&&!prevArmed) addLog('Vehicle ARMED — motors active','lwarn');
  if(!d.armed&&prevArmed) addLog('Vehicle DISARMED — mission ended','lok');
  prevArmed=d.armed;

  const pct=d.battery;
  document.getElementById('bat-pct').textContent=pct.toFixed(1);
  document.getElementById('bat-bar').style.width=pct+'%';
  document.getElementById('bat-bar').style.background=pct>50?'var(--accent2)':pct>20?'var(--warn)':'var(--danger)';
  document.getElementById('bat-label').textContent=pct>50?'NOMINAL':pct>20?'LOW':'CRITICAL';

  drawGauge('altGauge',d.alt,100,'m AGL','#00c8ff');
  drawGauge('spdGauge',d.groundspeed,60,'m/s','#00ff9d');
  drawCompass(d.heading);

  labels.push(new Date().toTimeString().slice(3,8));
  altHistory.push(d.alt); spdHistory.push(d.groundspeed);
  if(labels.length>80){ labels.shift(); altHistory.shift(); spdHistory.shift(); }
  chart.update();

  const cur=d.wp_current, tot=d.wp_total;
  document.getElementById('wp-cur').textContent=cur;
  document.getElementById('wp-tot').textContent=tot;
  const wpPct=tot>0?Math.round(cur/tot*100):0;
  document.getElementById('wp-bar').style.width=wpPct+'%';
  document.getElementById('wp-pct').textContent=wpPct+'%';

  updatePhase(d.mission_phase||'STANDBY');
  updateMap(d);
  updateLidarPanel(d.lidar);
  drawStaticLayers(d);
  updateTargetPanel(d);
  updateNFZPanel(d);

  if(d.lidar.avoidance_count>prevAvoidCount){
    addLog(`OBSTACLE AVOIDED #${d.lidar.avoidance_count}  dist=${d.lidar.nearest_dist.toFixed(1)}m  side=${d.lidar.escape_side}`,'ldanger');
    prevAvoidCount=d.lidar.avoidance_count;
  }
  if(d.alt>5&&prevAlt<=5)  addLog('Airborne — alt '+d.alt.toFixed(1)+'m','lok');
  if(d.alt<2&&prevAlt>=2&&d.armed) addLog('Approaching ground...','lwarn');
  prevAlt=d.alt;
  if(d.connected && !connectedLogged){ connectedLogged=true; addLog('Drone connected — telemetry active','lok'); }
});

drawCompass(0);
drawGauge('altGauge',0,100,'m AGL','#00c8ff');
drawGauge('spdGauge',0,60,'m/s','#00ff9d');
drawBearing(0,999);
setTimeout(()=>map.invalidateSize(),300);

// Draw static layers immediately from server-injected data (no socket wait)
drawStaticLayers({
  nfz_zones:          NFZ_ZONES_INIT,
  secondary_targets:  SECONDARY_TARGETS_INIT,
  loiter_waypoints:   LOITER_WPS_INIT,
  lat: HOME_LAT, lon: HOME_LON,
  target_lat: TARGET_LAT_JS, target_lon: TARGET_LON_JS,
});
</script>
</body>
</html>
"""


@app.route("/")
def index():
    import json
    return render_template_string(
        HTML,
        home_lat=HOME_LAT,
        home_lon=HOME_LON,
        target_lat=TARGET_LAT,
        target_lon=TARGET_LON,
        orbit_radius=ORBIT_RADIUS,
        secondary_targets_json=json.dumps(SECONDARY_TARGETS),
        nfz_zones_json=json.dumps(NO_FLY_ZONES),
        loiter_waypoints_json=json.dumps(LOITER_WAYPOINTS),
    )


_emit_loop_started = False
_emit_loop_lock    = threading.Lock()


@socketio.on("connect")
def on_connect():
    global _emit_loop_started
    with _emit_loop_lock:
        if not _emit_loop_started:
            _emit_loop_started = True
            socketio.start_background_task(emit_loop)


if __name__ == "__main__":
    t = threading.Thread(target=start_telemetry, daemon=True)
    t.start()
    print("Aran GCS v13.0 running at http://localhost:5000")
    print("  Coordinates served from mission_config.py — single source of truth")
    print("  POST /lidar_update    — push LiDAR + mission state")
    print("  POST /pid_tune        — live gain update")
    print("  GET  /pid_gains       — current PID gains")
    print("  GET  /download_log    — CSV flight log")
    print("  GET  /scenario_list   — all 24 scenario names + event counts")
    print("  GET  /nfz_status      — live NFZ proximity for current drone position")
    print("  GET  /map_slice       — live 2D voxel GeoJSON at current altitude")
    print("  GET  /map_stats       — 3D occupancy grid statistics")
    print(f"  Secondary targets: {len(SECONDARY_TARGETS)} | NFZ zones: {len(NO_FLY_ZONES)} | Loiter WPs: {len(LOITER_WAYPOINTS)}")
    socketio.run(app, host="0.0.0.0", port=5000, debug=False, allow_unsafe_werkzeug=True)