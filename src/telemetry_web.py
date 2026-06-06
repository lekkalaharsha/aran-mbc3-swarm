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
import hmac
import asyncio
import math
import os
import threading
import time
from collections import deque
from datetime import datetime
from flask import Flask, render_template, request, jsonify, Response
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
socketio = SocketIO(app,
    cors_allowed_origins=["http://localhost:5000", "http://127.0.0.1:5000"],
    async_mode="threading")

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
# _shared_lock serialises all reads/writes of `data` and `lidar_data`; prevents
# torn reads between the Flask request thread and the socketio emit_loop thread.
_shared_lock     = threading.Lock()
# Watchdog: mission script must POST /lidar_update within 10s or GCS shows STALE.
_mission_alive   = {"last_push": 0.0}
# Optional GCS command auth — set GCS_TOKEN env var to require a token header.
GCS_TOKEN = os.environ.get("GCS_TOKEN", "")
if not GCS_TOKEN:
    print(
        "WARNING: GCS_TOKEN not set — all POST endpoints are unauthenticated. "
        "Set GCS_TOKEN in launch.sh for field/hardware deployments.",
        flush=True,
    )
GCS_BIND_HOST = os.environ.get("GCS_HOST", "127.0.0.1")

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

# ── ASP (Air Situation Picture) state ─────────────────────────────
asp_data = {
    "tracks":        [],      # radar detection tracks
    "swarm_drones":  [],      # all 5 drone positions from swarm_monitor
    "scan_count":    0,
    "last_update":   0.0,
    "drone_ids":     [],
    "track_log":     deque(maxlen=50000),
}


def _check_auth():
    """Return a 403 Response if GCS_TOKEN is set and the request header is wrong."""
    if not GCS_TOKEN:
        return None
    token = request.headers.get("X-GCS-Token", "")
    if not hmac.compare_digest(token.encode(), GCS_TOKEN.encode()):
        return jsonify({"ok": False, "error": "Unauthorized — set X-GCS-Token header"}), 403
    return None


# ── LiDAR update endpoint ──────────────────────────────────────────
@app.route("/lidar_update", methods=["POST"])
def lidar_update():
    payload = request.get_json(silent=True) or {}
    _mission_alive["last_push"] = time.time()
    with _shared_lock:
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
        if "sectors" in payload:
            lidar_data["sectors"] = payload["sectors"]
        if "groundspeed"       in payload: data["groundspeed"]  = payload["groundspeed"]
        if "gps_ok"            in payload: data["gps_ok"]       = payload["gps_ok"]
        if "reconnects"        in payload: data["reconnects"]   = payload["reconnects"]
        if "eta_seconds"       in payload: data["eta_seconds"]  = payload["eta_seconds"]
        if "drone_lat"         in payload: data["lat"]          = payload["drone_lat"]
        if "drone_lon"         in payload: data["lon"]          = payload["drone_lon"]
        if "drone_alt"         in payload: data["alt"]          = payload["drone_alt"]
        if "drone_heading"     in payload: data["heading"]      = payload["drone_heading"]
        if "drone_armed"       in payload: data["armed"]        = payload["drone_armed"]
        if "drone_flight_mode" in payload: data["flight_mode"]  = payload["drone_flight_mode"]
        if "drone_battery"     in payload: data["battery"]      = payload["drone_battery"]
        if "drone_vspeed"     in payload: data["vspeed"]      = payload["drone_vspeed"]
        if "mission_phase" in payload:
            data["mission_phase"] = payload["mission_phase"]
            _phase_state["push_time"] = datetime.now().timestamp()
        if "wp_current" in payload: data["wp_current"] = payload["wp_current"]
        if "wp_total"   in payload: data["wp_total"]   = payload["wp_total"]
        # Snapshot for socketio.emit after releasing the lock
        _asp_emit = None
        if "asp_tracks" in payload:
            tracks = payload["asp_tracks"]
            asp_data["tracks"]      = tracks
            asp_data["scan_count"] += 1
            asp_data["last_update"] = time.time()
            for t in tracks:
                asp_data["track_log"].append({
                    "time":        datetime.now().strftime("%H:%M:%S.%f")[:-3],
                    "id":          t.get("id","?"),
                    "lat":         t.get("lat",0),
                    "lon":         t.get("lon",0),
                    "range_m":     t.get("range_m",0),
                    "bearing_deg": t.get("bearing_deg",0),
                    "alt_m":       t.get("alt_m",0),
                    "velocity_ms": t.get("velocity_ms",0),
                    "confidence":  t.get("confidence",0),
                    "drone_id":    payload.get("asp_drone_id","DRONE-L"),
                })
            _asp_emit = {
                "tracks":      list(asp_data["tracks"]),
                "scan_count":  asp_data["scan_count"],
                "last_update": asp_data["last_update"],
                "drone":       {"lat": data["lat"], "lon": data["lon"],
                                "alt": data["alt"], "heading": data["heading"]},
            }
    if _asp_emit:
        socketio.emit("asp", _asp_emit)

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


# ── ASP update endpoint ────────────────────────────────────────────
@app.route("/asp_update", methods=["POST"])
def asp_update():
    payload = request.get_json(silent=True) or {}
    # Only update tracks if caller explicitly sent the key — guards against
    # swarm_monitor and other payloads that omit radar data clearing real tracks.
    with _shared_lock:
        if "asp_tracks" in payload or "tracks" in payload:
            tracks = payload["asp_tracks"] if "asp_tracks" in payload else payload.get("tracks", [])
            asp_data["tracks"] = tracks
            asp_data["scan_count"] += 1
            asp_data["last_update"] = time.time()
            for t in tracks:
                asp_data["track_log"].append({
                    "time":        datetime.now().strftime("%H:%M:%S.%f")[:-3],
                    "id":          t.get("id", "?"),
                    "lat":         t.get("lat", 0),
                    "lon":         t.get("lon", 0),
                    "range_m":     t.get("range_m", 0),
                    "bearing_deg": t.get("bearing_deg", 0),
                    "alt_m":       t.get("alt_m", 0),
                    "velocity_ms": t.get("velocity_ms", 0),
                    "confidence":  t.get("confidence", 0),
                    "drone_id":    payload.get("asp_drone_id", "DRONE-L"),
                })
        drone_id = payload.get("asp_drone_id")
        if drone_id and drone_id not in asp_data["drone_ids"]:
            asp_data["drone_ids"].append(drone_id)
        if "swarm_drones" in payload:
            asp_data["swarm_drones"] = payload["swarm_drones"]
        _asp_snap = {
            "tracks":       list(asp_data["tracks"]),
            "swarm_drones": list(asp_data["swarm_drones"]),
            "scan_count":   asp_data["scan_count"],
            "last_update":  asp_data["last_update"],
            "drone":        {"lat": data["lat"], "lon": data["lon"],
                             "alt": data["alt"], "heading": data["heading"]},
            "drone_ids":    list(asp_data["drone_ids"]),
        }
    socketio.emit("asp", _asp_snap)
    return jsonify({"ok": True})


@app.route("/asp_download")
def asp_download():
    """Download full ASP track log as CSV."""
    import csv, io
    buf = io.StringIO()
    w = csv.DictWriter(buf, fieldnames=[
        "time","id","lat","lon","range_m","bearing_deg",
        "alt_m","velocity_ms","confidence","drone_id"])
    w.writeheader()
    for row in list(asp_data["track_log"]):   # snapshot — deque mutates concurrently at 5 Hz
        w.writerow(row)
    return Response(buf.getvalue(), mimetype="text/csv",
                    headers={"Content-Disposition":
                             "attachment; filename=asp_track_log.csv"})


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
    auth_err = _check_auth()
    if auth_err: return auth_err
    payload = request.get_json(silent=True) or {}
    controller = payload.get("controller")
    if controller not in pid_gains:
        return jsonify({"ok": False, "error": f"Unknown controller: {controller}"}), 400
    for key in ("kp", "ki", "kd", "output_limit"):
        if key in payload:
            try:
                pid_gains[controller][key] = float(payload[key])
            except (ValueError, TypeError):
                return jsonify({"ok": False, "error": f"{key} must be numeric"}), 400
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


# ── Drone state API — used by asp_bridge.py for radar→lat/lon conversion ──
@app.route("/api/drone_state", methods=["GET"])
def api_drone_state():
    return jsonify({
        "lat":     data["lat"],
        "lon":     data["lon"],
        "alt":     data["alt"],
        "heading": data["heading"],
        "connected": data.get("connected", False),
    })


# ── Swarm state API — used by leader_election.py ──────────────────
@app.route("/api/swarm_state", methods=["GET"])
def api_swarm_state():
    """Return all 5 drone connected/position states from swarm_monitor pushes."""
    return jsonify({
        "swarm_drones": asp_data.get("swarm_drones", []),
        "timestamp":    asp_data.get("last_update", 0.0),
    })


# ── Leader election API — written by leader_election.py ───────────
_leader_state = {
    "leader_id":    "DRONE-0",
    "leader_model": "mbc3_radar_drone_0",
    "since":        0.0,
    "election_count": 0,
}

@app.route("/api/leader", methods=["GET"])
def api_leader_get():
    return jsonify(_leader_state)

@app.route("/api/leader", methods=["POST"])
def api_leader_post():
    payload = request.get_json(silent=True) or {}
    if "leader_id" in payload:
        _leader_state.update(payload)
        socketio.emit("leader", _leader_state)   # live update to ASP page
    return jsonify({"ok": True})


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
    auth_err = _check_auth()
    if auth_err: return auth_err
    payload = request.get_json(silent=True) or {}
    if "lat" not in payload or "lon" not in payload:
        return jsonify({"ok": False, "error": "lat and lon required"}), 400
    try:
        nfz = {
            "name":     payload.get("name",     f"DYN-NFZ-{int(time.time())}"),
            "lat":      float(payload["lat"]),
            "lon":      float(payload["lon"]),
            "radius_m": float(payload.get("radius_m", 50.0)),
            "reason":   payload.get("reason",   "Dynamic GCS injection"),
        }
    except (ValueError, TypeError) as e:
        return jsonify({"ok": False, "error": f"Invalid numeric field: {e}"}), 400
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
    auth_err = _check_auth()
    if auth_err: return auth_err
    payload = request.get_json(silent=True) or {}
    if "lat" not in payload or "lon" not in payload:
        return jsonify({"ok": False, "error": "lat and lon required"}), 400
    try:
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
    except (ValueError, TypeError) as e:
        return jsonify({"ok": False, "error": f"Invalid numeric field: {e}"}), 400
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
    auth_err = _check_auth()
    if auth_err: return auth_err
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
    auth_err = _check_auth()
    if auth_err: return auth_err
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
        try:
            now = datetime.now()
            elapsed = now - start_time

            # Atomic snapshot — prevents torn reads when lidar_update() is
            # writing from the Flask thread concurrently.
            with _shared_lock:
                data["elapsed"] = str(elapsed).split(".")[0]
                data_snap  = dict(data)
                lidar_snap = dict(lidar_data)

            data_age_s   = time.time() - _mission_alive["last_push"]
            mission_alive = _mission_alive["last_push"] > 0 and data_age_s < 10.0

            flight_log.append((
                now.strftime("%H:%M:%S"),
                round(data_snap["lat"], 6), round(data_snap["lon"], 6),
                data_snap["alt"], data_snap["groundspeed"], data_snap["heading"],
                data_snap["battery"], data_snap["flight_mode"], data_snap["armed"],
                round(min(lidar_snap["nearest_dist"], 9999.0), 1),
                round(min(lidar_snap["nearest_bearing"], 9999.0) if math.isfinite(lidar_snap["nearest_bearing"]) else 0.0, 1),
                lidar_snap["avoidance_count"],
            ))

            payload = data_snap
            payload["trail"]              = list(trail)[-150:]
            payload["lidar"]              = lidar_snap
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
            payload["mission_alive"]      = mission_alive
            payload["data_age_s"]         = round(data_age_s, 1) if _mission_alive["last_push"] > 0 else None
            payload["map"] = {
                "voxel_count":   map_data["voxel_count"],
                "resolution_m":  map_data["resolution_m"],
                "point_count":   map_data["raw_point_count"],
                "scan_count":    map_data["scan_count"],
                "alt_range_m":   map_data["alt_range_m"],
            }
            socketio.emit("telemetry", payload)
        except Exception as e:
            _gcs_print(f"emit_loop error — {e}")
        time.sleep(0.4)




@app.route("/")
def index():
    import json
    return render_template(
        "telemetry.html",
        home_lat=HOME_LAT,
        home_lon=HOME_LON,
        target_lat=TARGET_LAT,
        target_lon=TARGET_LON,
        orbit_radius=ORBIT_RADIUS,
        secondary_targets_json=json.dumps(SECONDARY_TARGETS),
        nfz_zones_json=json.dumps(NO_FLY_ZONES),
        loiter_waypoints_json=json.dumps(LOITER_WAYPOINTS),
    )


@app.route("/asp")
def asp_page():
    return render_template("asp.html", home_lat=HOME_LAT, home_lon=HOME_LON)


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
    # SWARM_MODE=1: skip MAVSDK connection — swarm_monitor.py owns all drone ports.
    # Avoids two mavsdk_server processes competing on port 14540 → connection oscillation.
    if os.environ.get("SWARM_MODE", "0") != "1":
        t = threading.Thread(target=start_telemetry, daemon=True)
        t.start()
    else:
        _gcs_print("SWARM_MODE: MAVSDK telemetry disabled — swarm_monitor owns drone ports")
    print("Aran Technologies — MBC-3 GCS running at http://localhost:5000")
    print("  Coordinates served from mission_config.py — single source of truth")
    print("  POST /lidar_update    — push LiDAR + mission state")
    print("  POST /pid_tune        — live gain update")
    print("  GET  /pid_gains       — current PID gains")
    print("  GET  /download_log    — CSV flight log")
    print("  GET  /scenario_list   — scenario list (gz-transport active, sim mode disabled)")
    print("  GET  /nfz_status      — live NFZ proximity for current drone position")
    print("  GET  /map_slice       — live 2D voxel GeoJSON at current altitude")
    print("  GET  /map_stats       — 3D occupancy grid statistics")
    print(f"  Secondary targets: {len(SECONDARY_TARGETS)} | NFZ zones: {len(NO_FLY_ZONES)} | Loiter WPs: {len(LOITER_WAYPOINTS)}")
    socketio.run(app, host=GCS_BIND_HOST, port=5000, debug=False, allow_unsafe_werkzeug=True)