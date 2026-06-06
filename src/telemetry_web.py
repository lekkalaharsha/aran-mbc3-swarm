"""
Aran Technologies — GCS Dashboard v13
FastAPI + python-socketio rewrite (was Flask + flask-socketio).

Run:  uvicorn src.telemetry_web:socket_app --host 127.0.0.1 --port 5000
 Or:  python3 src/telemetry_web.py
"""
import asyncio
import csv
import hmac
import io
import json
import math
import os
import time
from collections import deque
from contextlib import asynccontextmanager
from datetime import datetime

import socketio
import uvicorn
from fastapi import Depends, FastAPI, HTTPException, Request
from fastapi.responses import JSONResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from mavsdk import System

from mission_config import (
    HOME_LAT, HOME_LON,
    TARGET_LAT, TARGET_LON,
    ORBIT_RADIUS,
    ROWS, ROW_SPACING, ROW_WIDTH,
    SECONDARY_TARGETS, NO_FLY_ZONES, LOITER_WAYPOINTS,
    generate_survey_grid,
)

SURVEY_WAYPOINTS = generate_survey_grid()

# ── Shared state ──────────────────────────────────────────────────────────────
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

map_data = {
    "voxel_count":     0,
    "resolution_m":    1.0,
    "raw_point_count": 0,
    "scan_count":      0,
    "bounds":          None,
    "alt_range_m":     None,
    "geojson_slice":   None,
}

pid_gains = {
    "avoidance": {"kp": 1.5, "ki": 0.0, "kd": 0.6, "output_limit": 15.0},
    "altitude":  {"kp": 1.2, "ki": 0.1, "kd": 0.4, "output_limit": 3.0},
    "orbit":     {"kp": 0.8, "ki": 0.05,"kd": 0.3, "output_limit": 5.0},
}

start_time  = datetime.now()
trail       = deque(maxlen=300)
flight_log  = deque(maxlen=10000)

dynamic_commands = {
    "nfz_queue":      [],
    "target_queue":   [],
    "config_updates": {},
    "event_queue":    [],
}

_phase_state = {"push_time": 0.0}
_mission_alive = {"last_push": 0.0}

asp_data = {
    "tracks":        [],
    "swarm_drones":  [],
    "scan_count":    0,
    "last_update":   0.0,
    "drone_ids":     [],
    "track_log":     deque(maxlen=50000),
}

_leader_state = {
    "leader_id":      "DRONE-0",
    "leader_model":   "mbc3_radar_drone_0",
    "since":          0.0,
    "election_count": 0,
}

_shared_lock  = asyncio.Lock()
_dyn_cmd_lock = asyncio.Lock()

GCS_TOKEN = os.environ.get("GCS_TOKEN", "")
if not GCS_TOKEN:
    print(
        "WARNING: GCS_TOKEN not set — all POST endpoints are unauthenticated. "
        "Set GCS_TOKEN in launch.sh for field/hardware deployments.",
        flush=True,
    )
GCS_BIND_HOST = os.environ.get("GCS_HOST", "127.0.0.1")

RETRY_DELAY = 3.0


# ── Auth dependency ───────────────────────────────────────────────────────────
async def check_auth(request: Request):
    if not GCS_TOKEN:
        return
    token = request.headers.get("X-GCS-Token", "")
    if not hmac.compare_digest(token.encode(), GCS_TOKEN.encode()):
        raise HTTPException(status_code=403, detail="Unauthorized — set X-GCS-Token header")


# ── Helpers ───────────────────────────────────────────────────────────────────
def _gcs_print(msg: str):
    print(f"  [{datetime.now().strftime('%H:%M:%S')}] {msg}")


async def _get_body(request: Request) -> dict:
    try:
        return await request.json()
    except Exception:
        return {}


# ── Telemetry streams ─────────────────────────────────────────────────────────
async def _stream(name, coro_factory, retry_delay=RETRY_DELAY):
    while True:
        try:
            await coro_factory()
        except Exception as e:
            err = str(e).splitlines()[0][:120]
            _gcs_print(f"stream '{name}' error — {err}; retrying in {retry_delay:.0f}s")
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


async def emit_loop():
    while True:
        await asyncio.sleep(0.4)
        try:
            now = datetime.now()
            elapsed = now - start_time

            async with _shared_lock:
                data["elapsed"] = str(elapsed).split(".")[0]
                data_snap  = dict(data)
                lidar_snap = dict(lidar_data)

            data_age_s    = time.time() - _mission_alive["last_push"]
            mission_alive = _mission_alive["last_push"] > 0 and data_age_s < 10.0

            flight_log.append((
                now.strftime("%H:%M:%S"),
                round(data_snap["lat"], 6), round(data_snap["lon"], 6),
                data_snap["alt"], data_snap["groundspeed"], data_snap["heading"],
                data_snap["battery"], data_snap["flight_mode"], data_snap["armed"],
                round(min(lidar_snap["nearest_dist"], 9999.0), 1),
                round(min(lidar_snap["nearest_bearing"], 9999.0)
                      if math.isfinite(lidar_snap["nearest_bearing"]) else 0.0, 1),
                lidar_snap["avoidance_count"],
            ))

            payload = dict(data_snap)
            payload["trail"]             = list(trail)[-150:]
            payload["lidar"]             = lidar_snap
            payload["survey_waypoints"]  = SURVEY_WAYPOINTS
            payload["home_lat"]          = HOME_LAT
            payload["home_lon"]          = HOME_LON
            payload["target_lat"]        = TARGET_LAT
            payload["target_lon"]        = TARGET_LON
            payload["orbit_radius"]      = ORBIT_RADIUS
            payload["pid_gains"]         = pid_gains
            payload["secondary_targets"] = SECONDARY_TARGETS
            payload["nfz_zones"]         = NO_FLY_ZONES
            payload["loiter_waypoints"]  = LOITER_WAYPOINTS
            payload["mission_alive"]     = mission_alive
            payload["data_age_s"]        = (round(data_age_s, 1)
                                            if _mission_alive["last_push"] > 0 else None)
            payload["map"] = {
                "voxel_count":  map_data["voxel_count"],
                "resolution_m": map_data["resolution_m"],
                "point_count":  map_data["raw_point_count"],
                "scan_count":   map_data["scan_count"],
                "alt_range_m":  map_data["alt_range_m"],
            }
            await sio.emit("telemetry", payload)
        except Exception as e:
            _gcs_print(f"emit_loop error — {e}")


# ── App lifecycle ─────────────────────────────────────────────────────────────
@asynccontextmanager
async def lifespan(app: FastAPI):
    tasks = [asyncio.create_task(emit_loop())]
    if os.environ.get("SWARM_MODE", "0") != "1":
        tasks.append(asyncio.create_task(telemetry_loop()))
    else:
        _gcs_print("SWARM_MODE: MAVSDK telemetry disabled — swarm_monitor owns drone ports")
    yield
    for t in tasks:
        t.cancel()
        try:
            await t
        except asyncio.CancelledError:
            pass


# ── FastAPI + Socket.IO setup ─────────────────────────────────────────────────
sio = socketio.AsyncServer(
    async_mode="asgi",
    cors_allowed_origins=["http://localhost:5000", "http://127.0.0.1:5000"],
)

app = FastAPI(lifespan=lifespan)

_base = os.path.dirname(os.path.abspath(__file__))
templates = Jinja2Templates(directory=os.path.join(_base, "templates"))
templates.env.filters["tojson"] = lambda v: json.dumps(v, ensure_ascii=False)
app.mount("/static", StaticFiles(directory=os.path.join(_base, "static")), name="static")

socket_app = socketio.ASGIApp(sio, app)


# ── Routes ────────────────────────────────────────────────────────────────────
@app.post("/lidar_update")
async def lidar_update(request: Request):
    payload = await _get_body(request)
    _mission_alive["last_push"] = time.time()
    _asp_emit = None
    async with _shared_lock:
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
        if "drone_vspeed"      in payload: data["vspeed"]       = payload["drone_vspeed"]
        if "mission_phase" in payload:
            data["mission_phase"] = payload["mission_phase"]
            _phase_state["push_time"] = datetime.now().timestamp()
        if "wp_current" in payload: data["wp_current"] = payload["wp_current"]
        if "wp_total"   in payload: data["wp_total"]   = payload["wp_total"]
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
        await sio.emit("asp", _asp_emit)

    if "map_stats" in payload:
        ms = payload["map_stats"]
        map_data["voxel_count"]     = ms.get("voxel_count",     0)
        map_data["resolution_m"]    = ms.get("resolution_m",    1.0)
        map_data["raw_point_count"] = ms.get("raw_point_count", 0)
        map_data["scan_count"]      = ms.get("scan_count",      0)
        map_data["bounds"]          = ms.get("bounds")
        map_data["alt_range_m"]     = ms.get("alt_range_m")

    async with _dyn_cmd_lock:
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
    return {"ok": True, "commands": cmds}


@app.post("/asp_update")
async def asp_update(request: Request):
    payload = await _get_body(request)
    async with _shared_lock:
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
        snap = {
            "tracks":       list(asp_data["tracks"]),
            "swarm_drones": list(asp_data["swarm_drones"]),
            "scan_count":   asp_data["scan_count"],
            "last_update":  asp_data["last_update"],
            "drone":        {"lat": data["lat"], "lon": data["lon"],
                             "alt": data["alt"], "heading": data["heading"]},
            "drone_ids":    list(asp_data["drone_ids"]),
        }
    await sio.emit("asp", snap)
    return {"ok": True}


@app.get("/asp_download")
async def asp_download():
    buf = io.StringIO()
    w = csv.DictWriter(buf, fieldnames=[
        "time","id","lat","lon","range_m","bearing_deg",
        "alt_m","velocity_ms","confidence","drone_id"])
    w.writeheader()
    for row in list(asp_data["track_log"]):
        w.writerow(row)
    return Response(
        content=buf.getvalue(),
        media_type="text/csv",
        headers={"Content-Disposition": "attachment; filename=asp_track_log.csv"},
    )


@app.post("/map_update")
async def map_update(request: Request):
    payload = await _get_body(request)
    stats = payload.get("map_stats", {})
    map_data["voxel_count"]     = stats.get("voxel_count",     0)
    map_data["resolution_m"]    = stats.get("resolution_m",    1.0)
    map_data["raw_point_count"] = stats.get("raw_point_count", 0)
    map_data["scan_count"]      = stats.get("scan_count",      0)
    map_data["bounds"]          = stats.get("bounds")
    map_data["alt_range_m"]     = stats.get("alt_range_m")
    return {"ok": True}


@app.get("/map_slice")
async def map_slice(request: Request):
    drone_alt = data.get("alt", 50.0)
    alt_min = float(request.query_params.get("alt_min", drone_alt - 5.0))
    alt_max = float(request.query_params.get("alt_max", drone_alt + 5.0))
    cached = map_data.get("geojson_slice")
    if cached:
        return cached
    return {"type": "FeatureCollection", "features": [],
            "meta": {"alt_min": alt_min, "alt_max": alt_max,
                     "voxel_count": map_data["voxel_count"]}}


@app.get("/map_stats")
async def map_stats_endpoint():
    return {
        "ok":           True,
        "voxel_count":  map_data["voxel_count"],
        "resolution_m": map_data["resolution_m"],
        "point_count":  map_data["raw_point_count"],
        "scan_count":   map_data["scan_count"],
        "bounds":       map_data["bounds"],
        "alt_range_m":  map_data["alt_range_m"],
    }


@app.post("/pid_tune")
async def pid_tune(request: Request, _=Depends(check_auth)):
    payload = await _get_body(request)
    controller = payload.get("controller")
    if controller not in pid_gains:
        return JSONResponse({"ok": False, "error": f"Unknown controller: {controller}"}, status_code=400)
    for key in ("kp", "ki", "kd", "output_limit"):
        if key in payload:
            try:
                pid_gains[controller][key] = float(payload[key])
            except (ValueError, TypeError):
                return JSONResponse({"ok": False, "error": f"{key} must be numeric"}, status_code=400)
    await sio.emit("pid_gains", pid_gains)
    return {"ok": True, "gains": pid_gains[controller]}


@app.get("/pid_gains")
async def get_pid_gains():
    return pid_gains


@app.get("/scenario_list")
async def scenario_list():
    sc_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "scenarios.json")
    try:
        with open(sc_file) as f:
            d = json.load(f)
        names = [{"name": s["name"], "description": s["description"],
                  "events": len(s["events"])} for s in d.get("scenarios", [])]
        return {"ok": True, "scenarios": names, "count": len(names)}
    except Exception as e:
        return JSONResponse({"ok": False, "error": str(e)}, status_code=500)


@app.get("/nfz_status")
async def nfz_status():
    from mission_config import get_nfz_exclusion_check
    inside, name, dist = get_nfz_exclusion_check(data["lat"], data["lon"])
    return {"inside_nfz": inside, "closest_nfz": name, "distance_m": round(dist, 1)}


@app.get("/api/drone_state")
async def api_drone_state():
    return {
        "lat":       data["lat"],
        "lon":       data["lon"],
        "alt":       data["alt"],
        "heading":   data["heading"],
        "connected": data.get("connected", False),
    }


@app.get("/api/swarm_state")
async def api_swarm_state():
    return {
        "swarm_drones": asp_data.get("swarm_drones", []),
        "timestamp":    asp_data.get("last_update", 0.0),
    }


@app.get("/api/leader")
async def api_leader_get():
    return _leader_state


@app.post("/api/leader")
async def api_leader_post(request: Request):
    payload = await _get_body(request)
    if "leader_id" in payload:
        _leader_state.update(payload)
        await sio.emit("leader", _leader_state)
    return {"ok": True}


@app.get("/download_log")
async def download_log():
    if not flight_log:
        return Response("No flight data yet.", status_code=404, media_type="text/plain")
    lines = ["timestamp,lat,lon,alt_m,groundspeed_ms,heading_deg,battery_pct,"
             "flight_mode,armed,lidar_dist_m,lidar_bearing_deg,avoidance_events\n"]
    for row in flight_log:
        lines.append(",".join(str(v) for v in row) + "\n")
    return Response(
        content="".join(lines),
        media_type="text/csv",
        headers={"Content-Disposition": "attachment; filename=flight_log.csv"},
    )


@app.post("/add_nfz")
async def add_nfz(request: Request, _=Depends(check_auth)):
    payload = await _get_body(request)
    if "lat" not in payload or "lon" not in payload:
        return JSONResponse({"ok": False, "error": "lat and lon required"}, status_code=400)
    try:
        nfz = {
            "name":     payload.get("name",     f"DYN-NFZ-{int(time.time())}"),
            "lat":      float(payload["lat"]),
            "lon":      float(payload["lon"]),
            "radius_m": float(payload.get("radius_m", 50.0)),
            "reason":   payload.get("reason",   "Dynamic GCS injection"),
        }
    except (ValueError, TypeError) as e:
        return JSONResponse({"ok": False, "error": f"Invalid numeric field: {e}"}, status_code=400)
    NO_FLY_ZONES.append(nfz)
    async with _dyn_cmd_lock:
        dynamic_commands["nfz_queue"].append(nfz)
    await sio.emit("dynamic_nfz", nfz)
    return {"ok": True, "nfz": nfz, "note": "Will be applied by mission script on next push cycle (~0.2s)"}


@app.post("/add_target")
async def add_target(request: Request, _=Depends(check_auth)):
    payload = await _get_body(request)
    if "lat" not in payload or "lon" not in payload:
        return JSONResponse({"ok": False, "error": "lat and lon required"}, status_code=400)
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
        return JSONResponse({"ok": False, "error": f"Invalid numeric field: {e}"}, status_code=400)
    SECONDARY_TARGETS.append(target)
    async with _dyn_cmd_lock:
        dynamic_commands["target_queue"].append(target)
    await sio.emit("dynamic_target", target)
    return {"ok": True, "target": target, "note": "Will be appended to SECONDARY_TARGETS on next push cycle"}


@app.post("/config_update")
async def config_update(request: Request, _=Depends(check_auth)):
    payload = await _get_body(request)
    allowed = {"LIDAR_WARN_DIST", "LIDAR_AVOID_DIST", "AVOIDANCE_OFFSET", "SAFE_RESUME_DIST"}
    updates = {k: float(v) for k, v in payload.items() if k in allowed}
    if not updates:
        return JSONResponse({"ok": False, "error": f"No valid keys. Allowed: {sorted(allowed)}"}, status_code=400)
    async with _dyn_cmd_lock:
        dynamic_commands["config_updates"].update(updates)
    return {"ok": True, "updates": updates, "note": "Will be applied by mission script on next push cycle"}


@app.post("/inject_event")
async def inject_event(request: Request, _=Depends(check_auth)):
    payload = await _get_body(request)
    bearing = float(payload.get("bearing_deg", 0.0))
    frame   = payload.get("frame", "sensor")
    if frame == "world":
        bearing = (bearing - data.get("heading", 0.0)) % 360
    event = {
        "bearing_deg": bearing,
        "dist_m":      float(payload.get("dist_m",    10.0)),
        "duration_s":  float(payload.get("duration_s", 5.0)),
    }
    async with _dyn_cmd_lock:
        dynamic_commands["event_queue"].append(event)
    return {"ok": True, "event": event, "frame_used": frame,
            "note": "Active in SIM mode only — injected into lidar_sim_reader"}


@app.get("/")
async def index(request: Request):
    return templates.TemplateResponse(request, "telemetry.html", {
        "home_lat":               HOME_LAT,
        "home_lon":               HOME_LON,
        "target_lat":             TARGET_LAT,
        "target_lon":             TARGET_LON,
        "orbit_radius":           ORBIT_RADIUS,
        "secondary_targets_json": json.dumps(SECONDARY_TARGETS),
        "nfz_zones_json":         json.dumps(NO_FLY_ZONES),
        "loiter_waypoints_json":  json.dumps(LOITER_WAYPOINTS),
    })


@app.get("/asp")
async def asp_page(request: Request):
    return templates.TemplateResponse(request, "asp.html", {
        "home_lat": HOME_LAT,
        "home_lon": HOME_LON,
    })


if __name__ == "__main__":
    print("Aran Technologies — MBC-3 GCS (FastAPI)")
    print(f"  http://{GCS_BIND_HOST}:5000")
    print("  POST /lidar_update  POST /pid_tune  GET /download_log")
    print(f"  Secondary targets: {len(SECONDARY_TARGETS)} | NFZ zones: {len(NO_FLY_ZONES)}")
    uvicorn.run(socket_app, host=GCS_BIND_HOST, port=5000)
