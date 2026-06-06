"""
swarm_telemetry_web.py — MBC-3 Swarm Command Center Dashboard
Port: 5000 (replaces telemetry_web.py when running swarm tests)

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

import hmac as _hmac_web
import math
import os
import threading
import time
from collections import deque
from datetime import datetime

from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO

from mission_config import (
    HOME_LAT, HOME_LON,
    TARGET_LAT, TARGET_LON,
    SECONDARY_TARGETS, NO_FLY_ZONES,
)
from mission_ai import MissionAI
from mission_config_swarm import (
    SWARM_NUM_DRONES,
    DRONE_SECTORS,
    drone_alt,
    generate_drone_wps,
)

app    = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

_ai = MissionAI()
_ai_cmd_queue: deque = deque(maxlen=10)   # validated deltas for swarm_mission to consume
_ai_cmd_log:   deque = deque(maxlen=20)   # history for dashboard display

GCS_TOKEN = os.environ.get("GCS_TOKEN", "")
if not GCS_TOKEN:
    print(
        "WARNING: GCS_TOKEN not set — all POST endpoints are unauthenticated. "
        "Set GCS_TOKEN in launch.sh for field/hardware deployments.",
        flush=True,
    )

GCS_BIND_HOST = os.environ.get("GCS_HOST", "127.0.0.1")


def _check_auth():
    """Return 403 response if GCS_TOKEN is set and request header doesn't match."""
    if not GCS_TOKEN:
        return None
    token = request.headers.get("X-GCS-Token", "")
    if not _hmac_web.compare_digest(token.encode(), GCS_TOKEN.encode()):
        return jsonify({"ok": False, "error": "Unauthorized — set X-GCS-Token header"}), 403
    return None

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

radar_tracks = []          # latest radar target list from asp_bridge / radar_sim
radar_scan_count = 0
radar_last_update = 0.0    # epoch of most recent track push (for stale detection)
RADAR_STALE_S = 3.0        # emit empty tracks if no update within this window
_known_track_ids: set = set()  # IDs seen in last scan — drives contact/lost events
_track_cmd: dict = {}          # active follow command: {active, target_id, drone_idx, lat, lon}

# Panel activity per drone: {drone_idx: {panel_letter: {count, last_range_m, ts}}}
panel_state = {
    i: {p: {"count": 0, "last_range_m": 0.0, "ts": 0.0} for p in "ABCDEF"}
    for i in range(SWARM_NUM_DRONES)
}
PANEL_DECAY_S = 3.0  # panel goes clear if no detection for this many seconds

# Event log
event_log: deque = deque(maxlen=200)
_state_lock = threading.Lock()

start_time = datetime.now()


# ── Helpers ───────────────────────────────────────────────────────────────────
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


# ── HTTP endpoints ─────────────────────────────────────────────────────────────
@app.route("/asp_update", methods=["POST"])
def asp_update():
    auth_err = _check_auth()
    if auth_err:
        return auth_err
    global radar_scan_count, radar_tracks, radar_last_update, _known_track_ids
    payload = request.get_json(silent=True) or {}

    # ── Swarm drone positions ──────────────────────────────────────────────
    if "swarm_drones" in payload:
        with _state_lock:
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
                    # Log phase changes
                    old_phase = swarm_state[idx].get("_last_phase", "")
                    new_phase = swarm_state[idx]["phase"]
                    if new_phase not in (old_phase, "---") and new_phase != "":
                        _push_event(f"DRONE-{idx} → {new_phase}", "phase")
                        swarm_state[idx]["_last_phase"] = new_phase

    # ── Radar tracks ──────────────────────────────────────────────────────
    if "asp_tracks" in payload or "tracks" in payload:
        tracks = payload.get("asp_tracks", payload.get("tracks", []))
        with _state_lock:
            radar_tracks = tracks
            radar_scan_count += 1
            radar_last_update = time.time()
            now = radar_last_update
            # Contact acquired / lost events
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
            # Reset scan counts so count = hits in this scan, not lifetime total
            for i in range(SWARM_NUM_DRONES):
                for p in "ABCDEF":
                    panel_state[i][p]["count"] = 0
            # Update panel state — attribute each track to a panel + all drones
            for t in tracks:
                panel = _bearing_to_panel(t.get("bearing_deg", 0.0))
                rng   = t.get("range_m", 0.0)
                # For single-radar sim: attribute to leader drone (idx 0)
                # In future: attribute per drone_id field
                drone_id_str = t.get("drone_id", "DRONE-0")
                try:
                    didx = int(drone_id_str.split("-")[-1])
                except Exception:
                    didx = 0
                if 0 <= didx < SWARM_NUM_DRONES:
                    panel_state[didx][panel]["count"]        += 1
                    panel_state[didx][panel]["last_range_m"]  = round(rng, 1)
                    panel_state[didx][panel]["ts"]            = now

    return jsonify({"ok": True})


@app.route("/event_push", methods=["POST"])
def event_push():
    """Receive redistribution / failure events from swarm_mission.py."""
    auth_err = _check_auth()
    if auth_err:
        return auth_err
    payload = request.get_json(silent=True) or {}
    msg  = payload.get("msg",  "event")
    kind = payload.get("kind", "info")
    _push_event(msg, kind)
    return jsonify({"ok": True})


@app.route("/lidar_update", methods=["POST"])
def lidar_update():
    """Compatibility stub — single-drone lidar data ignored in swarm mode."""
    return jsonify({"ok": True, "commands": {}})


@app.route("/api/track", methods=["POST"])
def api_track():
    """GCS operator assigns a drone to follow a radar target.
    POST {target_id, drone_idx}  — start tracking
    POST {stop: true}            — stop tracking
    """
    global _track_cmd
    data = request.get_json(silent=True) or {}
    if data.get("stop"):
        _track_cmd = {}
        _push_event("■ TRACKING STOPPED", "info")
        return jsonify({"ok": True})
    target_id = data.get("target_id")
    drone_idx = int(data.get("drone_idx", 0))
    if not target_id:
        return jsonify({"error": "missing target_id"}), 400
    with _state_lock:
        t = next((x for x in radar_tracks if x.get("id") == target_id), None)
    if not t:
        return jsonify({"error": "target not in current scan"}), 404
    _track_cmd = {
        "active":    True,
        "target_id": target_id,
        "drone_idx": drone_idx,
        "lat":       t["lat"],
        "lon":       t["lon"],
        "ts":        time.time(),
    }
    _push_event(f"▶ TRACKING {target_id} → DRONE-{drone_idx}", "contact")
    return jsonify({"ok": True})


@app.route("/api/track_state", methods=["GET"])
def api_track_state():
    """swarm_mission.py polls this to get current follow target position."""
    if not _track_cmd.get("active"):
        return jsonify({"active": False})
    target_id = _track_cmd.get("target_id")
    with _state_lock:
        t = next((x for x in radar_tracks if x.get("id") == target_id), None)
    if t:
        _track_cmd.update({"lat": t["lat"], "lon": t["lon"], "ts": time.time()})
    return jsonify({
        "active":    True,
        "target_id": target_id,
        "drone_idx": _track_cmd.get("drone_idx", 0),
        "lat":       _track_cmd.get("lat",   0.0),
        "lon":       _track_cmd.get("lon",   0.0),
        "stale":     t is None,
    })


@app.route("/api/ai_command", methods=["POST"])
def api_ai_command():
    """Operator submits natural language command → MissionAI parses → queued for swarm_mission."""
    auth_err = _check_auth()
    if auth_err:
        return auth_err
    data = request.get_json(silent=True) or {}
    cmd  = data.get("command", "").strip()
    if not cmd:
        return jsonify({"ok": False, "error": "empty command"}), 400
    with _state_lock:
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
    return jsonify({"ok": ok, "delta": delta, "reason": reason})


@app.route("/api/pending_commands", methods=["GET"])
def api_pending_commands():
    """swarm_mission.py polls this every 2 s to execute AI-generated deltas."""
    auth_err = _check_auth()
    if auth_err:
        return auth_err
    cmds = []
    while _ai_cmd_queue:
        cmds.append(_ai_cmd_queue.popleft())
    return jsonify({"commands": cmds})


@app.route("/api/state")
def api_state():
    with _state_lock:
        drones  = [dict(swarm_state[i]) for i in range(SWARM_NUM_DRONES)]
        tracks  = list(radar_tracks)
        panels  = {i: dict(panel_state[i]) for i in range(SWARM_NUM_DRONES)}
        events  = list(event_log)
    return jsonify({
        "drones":      drones,
        "tracks":      tracks,
        "panels":      panels,
        "events":      events,
        "scan_count":  radar_scan_count,
        "uptime_s":    round((datetime.now() - start_time).total_seconds()),
    })


# ── Socket.IO push loop ───────────────────────────────────────────────────────
def _emit_loop():
    while True:
        time.sleep(0.5)
        try:
            now = time.time()
            with _state_lock:
                drones  = [dict(swarm_state[i]) for i in range(SWARM_NUM_DRONES)]
                # Emit empty tracks if radar_sim has gone silent
                tracks  = list(radar_tracks) if (now - radar_last_update) < RADAR_STALE_S else []
                # Build panel snapshot with active/inactive flag
                panels_snap = {}
                for i in range(SWARM_NUM_DRONES):
                    panels_snap[i] = {}
                    for p in "ABCDEF":
                        ps = panel_state[i][p]
                        active = (now - ps["ts"]) < PANEL_DECAY_S and ps["count"] > 0
                        panels_snap[i][p] = {
                            "active":   active,
                            "count":    ps["count"],
                            "range_m":  ps["last_range_m"],
                        }
                evts = list(event_log)[:20]

            elapsed = datetime.now() - start_time
            h, rem  = divmod(int(elapsed.total_seconds()), 3600)
            m, s    = divmod(rem, 60)

            socketio.emit("swarm_update", {
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


# ── Mission geometry for map ──────────────────────────────────────────────────
def _build_map_geometry():
    """Pre-compute sector polygons, NFZ circles, and target markers for JS."""
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
        "home":       [HOME_LAT, HOME_LON],
        "primary":    [TARGET_LAT, TARGET_LON],
        "sectors":    sectors,
        "nfz":        nfz,
        "sec_targets": sec_targets,
        "colors":     DRONE_COLORS,
        "drone_alts": [drone_alt(i) for i in range(SWARM_NUM_DRONES)],
    }


# ── HTML template ─────────────────────────────────────────────────────────────

@app.route("/")
def index():
    geo = _build_map_geometry()
    from mission_ai import MODEL as _ai_model
    return render_template(
        "swarm_telemetry.html",
        colors=DRONE_COLORS,
        geo=geo,
        n_drones=SWARM_NUM_DRONES,
        ai_model=_ai_model,
    )


# ── Entry point ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    import threading
    t = threading.Thread(target=_emit_loop, daemon=True)
    t.start()
    print(f"[SWARM-GCS] Swarm Command Center → http://{GCS_BIND_HOST}:5000", flush=True)
    print("[SWARM-GCS] Endpoints: /asp_update  /event_push  /api/state", flush=True)
    socketio.run(app, host=GCS_BIND_HOST, port=5000, debug=False,
                 allow_unsafe_werkzeug=True)
