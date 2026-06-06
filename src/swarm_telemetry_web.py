"""
swarm_telemetry_web.py — MBC-3 Swarm Command Center Dashboard
Port: 5000 (replaces telemetry_web.py when running swarm tests)
FastAPI + python-socketio rewrite (was Flask + flask-socketio).

COMPONENTS:
  1. Drone Status Grid   — 5 color-coded cards: phase/alt/speed/WP/armed
  2. Leaflet Map         — all 5 drone markers, color trails, sector polygons,
                          NFZ circles, radar target markers, redistribution arrows
  3. AERIS-10 Radar Panel— 6-sector polar SVG per drone (A-F panels, 60° each)
                          green=clear, red=active detection, panel label + count
  4. Event Log           — redistribution events, drone failures, phase changes

DATA SOURCES (same endpoints as telemetry_web.py for drop-in compatibility):
  POST /asp_update       ← swarm_mission.py pushes swarm_drones + radar tracks
  POST /event_push       ← swarm_mission.py pushes redistribution/failure events
  POST /lidar_update     ← isr_lidar_mpc.py (ignored in swarm mode, kept for compat)
  GET  /                 ← main dashboard HTML
  GET  /api/state        ← JSON state snapshot

USAGE:
  Run as replacement for telemetry_web.py during swarm tests:
    python3 src/swarm_telemetry_web.py
  Or launch via swarm_launch.sh (replace telemetry_web.py line).

  Single-drone launch.sh still uses telemetry_web.py.
"""

import asyncio
import hmac
import json
import os
import time
from collections import deque
from contextlib import asynccontextmanager
from datetime import datetime

import socketio
import uvicorn
from fastapi import Depends, FastAPI, HTTPException, Request
from fastapi.responses import JSONResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates

from mission_config import (
    HOME_LAT, HOME_LON,
    TARGET_LAT, TARGET_LON,
    SECONDARY_TARGETS, NO_FLY_ZONES,
)
from mission_ai import MissionAI, MODEL as _ai_model
from mission_config_swarm import (
    SWARM_NUM_DRONES,
    DRONE_SECTORS,
    drone_alt,
    generate_drone_wps,
)

_ai = MissionAI()
_ai_cmd_queue: deque = deque(maxlen=10)
_ai_cmd_log:   deque = deque(maxlen=20)

GCS_TOKEN = os.environ.get("GCS_TOKEN", "")
if not GCS_TOKEN:
    print(
        "WARNING: GCS_TOKEN not set — all POST endpoints are unauthenticated. "
        "Set GCS_TOKEN in launch.sh for field/hardware deployments.",
        flush=True,
    )

GCS_BIND_HOST = os.environ.get("GCS_HOST", "127.0.0.1")

# ── Drone colors (one per drone index) ───────────────────────────────────────
DRONE_COLORS = ["#3498db", "#2ecc71", "#e74c3c", "#f39c12", "#9b59b6"]
DRONE_NAMES  = [f"DRONE-{i}" for i in range(SWARM_NUM_DRONES)]

# ── Shared state ──────────────────────────────────────────────────────────────
swarm_state = {
    i: {
        "id":          f"DRONE-{i}",
        "lat":         HOME_LAT + (i - 2) * 0.0001,
        "lon":         HOME_LON + (i - 2) * 0.0001,
        "alt":         0.0,
        "heading":     0.0,
        "groundspeed": 0.0,
        "connected":   False,
        "armed":       False,
        "phase":       "STANDBY",
        "wp_current":  0,
        "wp_total":    0,
        "color":       DRONE_COLORS[i],
    }
    for i in range(SWARM_NUM_DRONES)
}

radar_tracks = []
radar_scan_count = 0
radar_last_update = 0.0
RADAR_STALE_S = 3.0
_known_track_ids: set = set()
_track_cmd: dict = {}

# Panel activity per drone: {drone_idx: {panel_letter: {count, last_range_m, ts}}}
panel_state = {
    i: {p: {"count": 0, "last_range_m": 0.0, "ts": 0.0} for p in "ABCDEF"}
    for i in range(SWARM_NUM_DRONES)
}
PANEL_DECAY_S = 3.0

event_log: deque = deque(maxlen=200)
_state_lock = asyncio.Lock()

_leader_state = {
    "leader_id":      "DRONE-0",
    "leader_model":   "mbc3_radar_drone_0",
    "since":          0.0,
    "election_count": 0,
}

start_time = datetime.now()


# ── Auth dependency ───────────────────────────────────────────────────────────
async def check_auth(request: Request):
    if not GCS_TOKEN:
        return
    token = request.headers.get("X-GCS-Token", "")
    if not hmac.compare_digest(token.encode(), GCS_TOKEN.encode()):
        raise HTTPException(status_code=403, detail="Unauthorized — set X-GCS-Token header")


# ── Helpers ───────────────────────────────────────────────────────────────────
async def _get_body(request: Request) -> dict:
    try:
        return await request.json()
    except Exception:
        return {}


def _bearing_to_panel(bearing_deg: float) -> str:
    az = bearing_deg % 360.0
    if az < 30 or az >= 330:  return "A"
    if 30  <= az < 90:        return "B"
    if 90  <= az < 150:       return "C"
    if 150 <= az < 210:       return "D"
    if 210 <= az < 270:       return "E"
    return "F"


def _push_event(msg: str, kind: str = "info"):
    ts = datetime.now().strftime("%H:%M:%S")
    event_log.appendleft({"ts": ts, "msg": msg, "kind": kind})


# ── Socket.IO push loop ───────────────────────────────────────────────────────
async def _emit_loop():
    while True:
        await asyncio.sleep(0.5)
        try:
            now = time.time()
            async with _state_lock:
                drones = [dict(swarm_state[i]) for i in range(SWARM_NUM_DRONES)]
                tracks = list(radar_tracks) if (now - radar_last_update) < RADAR_STALE_S else []
                panels_snap = {}
                for i in range(SWARM_NUM_DRONES):
                    panels_snap[i] = {}
                    for p in "ABCDEF":
                        ps = panel_state[i][p]
                        active = (now - ps["ts"]) < PANEL_DECAY_S and ps["count"] > 0
                        panels_snap[i][p] = {
                            "active":  active,
                            "count":   ps["count"],
                            "range_m": ps["last_range_m"],
                        }
                evts = list(event_log)[:20]

            elapsed = datetime.now() - start_time
            h, rem  = divmod(int(elapsed.total_seconds()), 3600)
            m, s    = divmod(rem, 60)

            await sio.emit("swarm_update", {
                "drones":     drones,
                "tracks":     tracks,
                "panels":     panels_snap,
                "events":     evts,
                "scan_count": radar_scan_count,
                "uptime":     f"{h:02d}:{m:02d}:{s:02d}",
                "ts":         now,
            })
        except Exception as e:
            print(f"[SWARM-GCS] emit_loop error — {e}", flush=True)


# ── App lifecycle ─────────────────────────────────────────────────────────────
@asynccontextmanager
async def lifespan(app: FastAPI):
    task = asyncio.create_task(_emit_loop())
    yield
    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass


# ── FastAPI + Socket.IO setup ─────────────────────────────────────────────────
sio = socketio.AsyncServer(async_mode="asgi", cors_allowed_origins="*")

app = FastAPI(lifespan=lifespan)

_base = os.path.dirname(os.path.abspath(__file__))
templates = Jinja2Templates(directory=os.path.join(_base, "templates"))
templates.env.filters["tojson"] = lambda v: json.dumps(v, ensure_ascii=False)
app.mount("/static", StaticFiles(directory=os.path.join(_base, "static")), name="static")

socket_app = socketio.ASGIApp(sio, app)


# ── HTTP endpoints ─────────────────────────────────────────────────────────────
@app.post("/asp_update")
async def asp_update(request: Request, _=Depends(check_auth)):
    global radar_scan_count, radar_tracks, radar_last_update, _known_track_ids
    payload = await _get_body(request)

    async with _state_lock:
        if "swarm_drones" in payload:
            for d in payload["swarm_drones"]:
                try:
                    idx = int(d["id"].split("-")[-1])
                except Exception:
                    continue
                if 0 <= idx < SWARM_NUM_DRONES:
                    swarm_state[idx].update({
                        "lat":         d.get("lat",         swarm_state[idx]["lat"]),
                        "lon":         d.get("lon",         swarm_state[idx]["lon"]),
                        "alt":         round(d.get("alt",   0.0), 1),
                        "heading":     d.get("heading",     0.0),
                        "groundspeed": round(d.get("groundspeed", 0.0), 1),
                        "connected":   d.get("connected",   False),
                        "armed":       d.get("armed",       False),
                        "phase":       d.get("phase",       d.get("flight_mode", "---")),
                        "wp_current":  d.get("wp_current",  0),
                        "wp_total":    d.get("wp_total",    0),
                    })
                    old_phase = swarm_state[idx].get("_last_phase", "")
                    new_phase = swarm_state[idx]["phase"]
                    if new_phase not in (old_phase, "---") and new_phase != "":
                        _push_event(f"DRONE-{idx} → {new_phase}", "phase")
                        swarm_state[idx]["_last_phase"] = new_phase

        if "asp_tracks" in payload or "tracks" in payload:
            tracks = payload.get("asp_tracks", payload.get("tracks", []))
            radar_tracks = tracks
            radar_scan_count += 1
            radar_last_update = time.time()
            now = radar_last_update
            new_ids  = {t['id'] for t in tracks if t.get('id')} - _known_track_ids
            lost_ids = _known_track_ids - {t['id'] for t in tracks if t.get('id')}
            for tid in new_ids:
                t = next((x for x in tracks if x.get('id') == tid), {})
                _push_event(
                    f"▲ CONTACT ACQUIRED: {tid}  R={t.get('range_m',0)}m"
                    f"  Az={t.get('bearing_deg',0):.0f}°",
                    "contact",
                )
            for tid in lost_ids:
                _push_event(f"▼ CONTACT LOST: {tid}", "contact_lost")
            _known_track_ids.clear()
            _known_track_ids.update(t['id'] for t in tracks if t.get('id'))
            for i in range(SWARM_NUM_DRONES):
                for p in "ABCDEF":
                    panel_state[i][p]["count"] = 0
            for t in tracks:
                panel = _bearing_to_panel(t.get("bearing_deg", 0.0))
                rng   = t.get("range_m", 0.0)
                drone_id_str = t.get("drone_id", "DRONE-0")
                try:
                    didx = int(drone_id_str.split("-")[-1])
                except Exception:
                    didx = 0
                if 0 <= didx < SWARM_NUM_DRONES:
                    panel_state[didx][panel]["count"]        += 1
                    panel_state[didx][panel]["last_range_m"]  = round(rng, 1)
                    panel_state[didx][panel]["ts"]            = now

    return {"ok": True}


@app.post("/event_push")
async def event_push(request: Request, _=Depends(check_auth)):
    payload = await _get_body(request)
    msg  = payload.get("msg",  "event")
    kind = payload.get("kind", "info")
    _push_event(msg, kind)
    return {"ok": True}


@app.post("/lidar_update")
async def lidar_update():
    return {"ok": True, "commands": {}}


@app.post("/api/track")
async def api_track(request: Request):
    global _track_cmd
    data = await _get_body(request)
    if data.get("stop"):
        _track_cmd = {}
        _push_event("■ TRACKING STOPPED", "info")
        return {"ok": True}
    target_id = data.get("target_id")
    drone_idx = int(data.get("drone_idx", 0))
    if not target_id:
        return JSONResponse({"error": "missing target_id"}, status_code=400)
    async with _state_lock:
        t = next((x for x in radar_tracks if x.get("id") == target_id), None)
    if not t:
        return JSONResponse({"error": "target not in current scan"}, status_code=404)
    _track_cmd = {
        "active":    True,
        "target_id": target_id,
        "drone_idx": drone_idx,
        "lat":       t["lat"],
        "lon":       t["lon"],
        "ts":        time.time(),
    }
    _push_event(f"▶ TRACKING {target_id} → DRONE-{drone_idx}", "contact")
    return {"ok": True}


@app.get("/api/track_state")
async def api_track_state():
    if not _track_cmd.get("active"):
        return {"active": False}
    target_id = _track_cmd.get("target_id")
    async with _state_lock:
        t = next((x for x in radar_tracks if x.get("id") == target_id), None)
    if t:
        _track_cmd.update({"lat": t["lat"], "lon": t["lon"], "ts": time.time()})
    return {
        "active":    True,
        "target_id": target_id,
        "drone_idx": _track_cmd.get("drone_idx", 0),
        "lat":       _track_cmd.get("lat",   0.0),
        "lon":       _track_cmd.get("lon",   0.0),
        "stale":     t is None,
    }


@app.post("/api/ai_command")
async def api_ai_command(request: Request, _=Depends(check_auth)):
    data = await _get_body(request)
    cmd  = data.get("command", "").strip()
    if not cmd:
        return JSONResponse({"ok": False, "error": "empty command"}, status_code=400)
    async with _state_lock:
        drones_snap = {i: dict(swarm_state[i]) for i in range(SWARM_NUM_DRONES)}
    delta  = _ai.parse_command(cmd, swarm_state=drones_snap)
    ok, reason = _ai.validate(delta)
    ts = datetime.now().strftime("%H:%M:%S")
    _ai_cmd_log.appendleft({"ts": ts, "cmd": cmd, "delta": delta, "ok": ok, "reason": reason})
    if ok:
        _ai_cmd_queue.append(delta)
        _push_event(
            f"[AI/{delta.get('source','?')}] {delta.get('action')} → "
            f"D{delta.get('target_drones',[])} ({int(delta.get('confidence',0)*100)}%)", "ai"
        )
    else:
        _push_event(f"[AI REJECT] {reason}", "warn")
    return {"ok": ok, "delta": delta, "reason": reason}


@app.get("/api/pending_commands")
async def api_pending_commands(_=Depends(check_auth)):
    cmds = []
    while _ai_cmd_queue:
        cmds.append(_ai_cmd_queue.popleft())
    return {"commands": cmds}


@app.get("/api/state")
async def api_state():
    async with _state_lock:
        drones  = [dict(swarm_state[i]) for i in range(SWARM_NUM_DRONES)]
        tracks  = list(radar_tracks)
        panels  = {i: dict(panel_state[i]) for i in range(SWARM_NUM_DRONES)}
        events  = list(event_log)
    return {
        "drones":     drones,
        "tracks":     tracks,
        "panels":     panels,
        "events":     events,
        "scan_count": radar_scan_count,
        "uptime_s":   round((datetime.now() - start_time).total_seconds()),
    }


@app.get("/api/swarm_state")
async def api_swarm_state():
    """leader_election.py polls this to get drone liveness."""
    async with _state_lock:
        drones = [dict(swarm_state[i]) for i in range(SWARM_NUM_DRONES)]
    return {"swarm_drones": drones, "timestamp": time.time()}


@app.get("/api/leader")
async def api_leader_get():
    return _leader_state


@app.post("/api/leader")
async def api_leader_post(request: Request):
    payload = await _get_body(request)
    if "leader_id" in payload:
        _leader_state.update(payload)
        await sio.emit("leader", _leader_state)
        _push_event(
            f"LEADER → {payload['leader_id']} "
            f"(election #{payload.get('election_count', 0)})", "phase"
        )
    return {"ok": True}


# ── Mission geometry for map ──────────────────────────────────────────────────
def _build_map_geometry() -> dict:
    sectors = []
    for i in range(SWARM_NUM_DRONES):
        wps = generate_drone_wps(i)
        if wps:
            sectors.append({
                "drone_idx": i,
                "color":     DRONE_COLORS[i],
                "wps":       [[lat, lon] for lat, lon in wps],
            })
    nfz = [
        {"name": z["name"], "lat": z["lat"], "lon": z["lon"], "r": z["radius_m"]}
        for z in NO_FLY_ZONES
    ]
    sec_targets = [
        {"name": t["name"], "lat": t["lat"], "lon": t["lon"]}
        for t in SECONDARY_TARGETS
    ]
    return {
        "home":        [HOME_LAT, HOME_LON],
        "primary":     [TARGET_LAT, TARGET_LON],
        "sectors":     sectors,
        "nfz":         nfz,
        "sec_targets": sec_targets,
        "colors":      DRONE_COLORS,
        "drone_alts":  [drone_alt(i) for i in range(SWARM_NUM_DRONES)],
    }


# ── Main page ─────────────────────────────────────────────────────────────────
@app.get("/")
async def index(request: Request):
    geo = _build_map_geometry()
    return templates.TemplateResponse(request, "swarm_telemetry.html", {
        "colors":   DRONE_COLORS,
        "geo":      geo,
        "n_drones": SWARM_NUM_DRONES,
        "ai_model": _ai_model,
    })


# ── Entry point ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print(f"[SWARM-GCS] Swarm Command Center → http://{GCS_BIND_HOST}:5000", flush=True)
    print("[SWARM-GCS] Endpoints: /asp_update  /event_push  /api/state", flush=True)
    uvicorn.run(socket_app, host=GCS_BIND_HOST, port=5000)
