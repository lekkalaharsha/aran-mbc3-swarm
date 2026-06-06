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

from flask import Flask, render_template_string, request, jsonify
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
_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>MBC-3 Swarm Command Center</title>
<link rel="stylesheet" href="/static/leaflet.css"/>
<script src="/static/leaflet.js"></script>
<script src="/static/socket.io.min.js"></script>
<link href="https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Rajdhani:wght@400;600;700&display=swap" rel="stylesheet">
<style>
:root{
  --bg:#060b14;--panel:#080f1c;--card:#0b1525;--border:#0e2340;
  --accent:#00c8ff;--accent2:#00ff9d;--warn:#ffb300;--danger:#ff3d3d;
  --dim:#1e4060;--text:#c8dff0;--textdim:#4a7a9b;
  --mono:'Share Tech Mono','Courier New',monospace;
}
*{box-sizing:border-box;margin:0;padding:0}
html,body{width:100%;height:100%;overflow:hidden;background:var(--bg);color:var(--text);font-family:var(--mono);font-size:12px}
body::after{content:'';position:fixed;inset:0;pointer-events:none;z-index:9999;
  background:repeating-linear-gradient(0deg,transparent,transparent 2px,rgba(0,0,0,0.06) 2px,rgba(0,0,0,0.06) 4px)}

header{
  background:linear-gradient(90deg,#060b14,#0b1a30,#060b14);
  border-bottom:1px solid var(--accent);box-shadow:0 0 20px rgba(0,200,255,0.15);
  padding:6px 14px;display:flex;align-items:center;gap:16px;flex-shrink:0
}
header h1{font-size:14px;color:var(--accent);letter-spacing:3px;font-family:'Rajdhani','Share Tech Mono',monospace;font-weight:700}
#uptime{color:var(--textdim);font-size:11px}
#scan_badge{background:var(--card);border:1px solid var(--border);border-radius:3px;padding:2px 8px;font-size:11px;color:var(--text)}
#conn_status{font-size:11px}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.4}}

/* Drone cards */
#drone_grid{display:grid;grid-template-columns:repeat(5,1fr);gap:6px;padding:6px 10px;flex-shrink:0}
.drone_card{
  background:var(--card);border:2px solid var(--border);border-radius:4px;
  padding:8px 10px;transition:border-color .3s;position:relative;overflow:hidden
}
.drone_card::before{content:'';position:absolute;top:0;left:0;right:0;height:1px;
  background:linear-gradient(90deg,transparent,var(--accent),transparent);opacity:.3}
.drone_card.armed{border-color:var(--dc);box-shadow:0 0 10px rgba(0,200,255,.1)}
.drone_card.failed{border-color:var(--danger);background:#0d0505;opacity:.75}
.dc_header{display:flex;align-items:center;gap:6px;margin-bottom:5px}
.dc_dot{width:9px;height:9px;border-radius:50%;background:var(--dc);box-shadow:0 0 6px var(--dc)}
.dc_id{font-weight:bold;font-size:12px;letter-spacing:1px}
.dc_phase{font-size:9px;color:var(--textdim);margin-left:auto;letter-spacing:1px}
.dc_row{display:flex;justify-content:space-between;font-size:10px;color:var(--textdim);margin-top:2px;border-bottom:1px solid #0a1828;padding:2px 0}
.dc_row:last-of-type{border-bottom:none}
.dc_val{color:var(--accent)}
.wp_bar{background:#050e1a;border:1px solid var(--border);border-radius:2px;height:4px;margin-top:5px;overflow:hidden}
.wp_fill{height:100%;border-radius:2px;background:linear-gradient(90deg,var(--accent),var(--accent2));transition:width .5s}
.dot_conn{display:inline-block;width:6px;height:6px;border-radius:50%;margin-left:4px}
.dot_conn.ok{background:var(--accent2);box-shadow:0 0 5px var(--accent2);animation:pulse 1.5s infinite}
.dot_conn.bad{background:var(--danger)}

/* Main panels */
#main_row{display:grid;grid-template-columns:1fr 340px;gap:6px;padding:0 10px 6px;flex:1;min-height:0}
#map_wrap{position:relative;border-radius:4px;overflow:hidden;border:1px solid var(--border)}
#map{width:100%;height:100%}
.leaflet-tile{filter:brightness(.52) saturate(.35) hue-rotate(190deg)}
.leaflet-control-zoom a{background:var(--card)!important;color:var(--accent)!important;border:1px solid var(--border)!important}
.leaflet-tooltip{background:rgba(6,11,20,.9)!important;border:1px solid var(--border)!important;color:var(--textdim)!important;
  font-family:var(--mono)!important;font-size:.55rem!important;padding:2px 7px!important;border-radius:2px!important}

/* Radar panel */
#radar_panel{background:var(--panel);border:1px solid var(--border);border-radius:4px;display:flex;flex-direction:column;min-height:0}
.rp_header{padding:7px 10px;border-bottom:1px solid var(--border);font-size:11px;color:var(--accent);letter-spacing:2px}
.drone_tabs{display:flex;border-bottom:1px solid var(--border)}
.dtab{flex:1;padding:5px 0;text-align:center;font-size:11px;cursor:pointer;border:none;background:none;
  color:var(--textdim);border-bottom:2px solid transparent;transition:all .2s;letter-spacing:1px}
.dtab.active{color:var(--text);border-bottom-color:var(--dc)}
.dtab:hover{color:var(--accent)}
#radar_svg_wrap{padding:10px;display:flex;justify-content:center}
svg#polar{display:block}
#panel_stats{padding:0 10px 8px;display:grid;grid-template-columns:repeat(3,1fr);gap:4px}
.ps_cell{background:#050e1a;border:1px solid var(--border);border-radius:3px;padding:5px 7px;text-align:center}
.ps_cell.active{background:rgba(255,61,61,.08);border-color:var(--danger)}
.ps_lbl{font-size:9px;color:var(--textdim);letter-spacing:1px}
.ps_cnt{font-size:14px;font-weight:bold}
.ps_rng{font-size:9px;color:var(--textdim)}
#radar_tgt_list{flex:1;overflow-y:auto;padding:6px 10px;border-top:1px solid var(--border)}
#radar_tgt_list::-webkit-scrollbar{width:3px}
#radar_tgt_list::-webkit-scrollbar-thumb{background:var(--dim)}
.tgt_row{display:flex;justify-content:space-between;align-items:center;font-size:10px;padding:2px 0;border-bottom:1px solid var(--card)}
.tgt_id{color:var(--accent)}
.tgt_track_btn{background:none;border:1px solid var(--dim);color:var(--textdim);
  border-radius:2px;padding:1px 5px;font-size:9px;cursor:pointer;font-family:var(--mono);
  letter-spacing:1px;transition:all .2s}
.tgt_track_btn:hover{border-color:var(--warn);color:var(--warn)}
.tgt_track_btn.tracking{border-color:var(--warn);color:var(--warn);
  background:rgba(255,179,0,.1);animation:pulse 1.2s infinite}
#stop_track_btn{display:none;background:rgba(255,179,0,.12);border:1px solid var(--warn);
  color:var(--warn);border-radius:3px;padding:3px 10px;font-family:var(--mono);
  font-size:10px;cursor:pointer;letter-spacing:1px;margin-left:auto}
#stop_track_btn.visible{display:block}

/* Event log */
#event_wrap{grid-column:1/-1;background:var(--panel);border:1px solid var(--border);border-radius:4px;
  padding:5px 10px;max-height:80px;overflow-y:auto}
#event_wrap::-webkit-scrollbar{width:3px}
#event_wrap::-webkit-scrollbar-thumb{background:var(--dim)}
#event_log{display:flex;flex-direction:column;gap:1px}
.ev_row{display:flex;gap:8px;font-size:10px}
.ev_ts{color:var(--dim);flex-shrink:0}
.ev_msg.redistrib{color:var(--warn)}
.ev_msg.failure{color:var(--danger)}
.ev_msg.phase{color:var(--textdim)}
.ev_msg.info{color:var(--accent2)}
.ev_msg.contact{color:#ffcc00;font-weight:bold}
.ev_msg.contact_lost{color:#7a5a00}
.ev_msg.ai{color:#b07aff}
.ev_msg.warn{color:var(--warn)}

/* AI command panel */
#ai_panel{padding:5px 10px 6px;flex-shrink:0;background:var(--panel);
  border:1px solid var(--border);border-top:1px solid rgba(176,122,255,.4);
  border-radius:0 0 4px 4px}
.ai_header{font-size:10px;color:#b07aff;letter-spacing:2px;margin-bottom:4px}
#ai_form{display:flex;gap:6px;margin-bottom:4px}
#ai_input{flex:1;background:var(--card);border:1px solid var(--border);
  color:var(--text);font-family:var(--mono);font-size:11px;
  padding:4px 8px;border-radius:2px;outline:none}
#ai_input:focus{border-color:#b07aff}
#ai_submit{background:rgba(176,122,255,.12);border:1px solid #b07aff;
  color:#b07aff;font-family:var(--mono);font-size:10px;
  padding:4px 12px;cursor:pointer;border-radius:2px;letter-spacing:1px}
#ai_submit:hover{background:rgba(176,122,255,.28)}
#ai_submit:disabled{opacity:.4;cursor:default}
#ai_response{font-size:10px;min-height:16px;color:var(--textdim)}

/* Contact acquired banner */
#contact_banner{
  position:fixed;top:54px;left:50%;transform:translateX(-50%);
  background:rgba(6,11,20,.96);border:1px solid #ffcc00;border-radius:4px;
  padding:7px 22px;font-size:13px;color:#ffcc00;letter-spacing:2px;
  z-index:10000;pointer-events:none;opacity:0;transition:opacity .25s;
  text-align:center;white-space:nowrap;box-shadow:0 0 20px rgba(255,204,0,.3)
}
#contact_banner.visible{opacity:1}
@keyframes contact_blink{0%,100%{opacity:1}50%{opacity:.3}}
#contact_banner.visible{opacity:1;animation:contact_blink .5s ease 3}

/* Radar sweep */
@keyframes radar_sweep{from{transform:rotate(0deg)}to{transform:rotate(360deg)}}
#polar_sweep{transform-origin:130px 130px;animation:radar_sweep 5s linear infinite}
#polar_sweep line{stroke:rgba(0,200,255,.55);stroke-width:1.5;stroke-linecap:round}
#polar_sweep line.glow{stroke:rgba(0,200,255,.12);stroke-width:10}
</style>
</head>
<body>
<div id="contact_banner"></div>
<header>
  <h1>&#11041; Aran Technologies &nbsp;|&nbsp; MBC-3 SWARM COMMAND CENTER</h1>
  <span id="uptime" style="color:var(--textdim);font-size:11px">00:00:00</span>
  <span id="scan_badge">RADAR 0 scans</span>
  <span id="conn_status" style="color:var(--danger);font-size:11px">&#9679; CONNECTING</span>
</header>

<div id="drone_grid"></div>

<div id="main_row">
  <div id="map_wrap"><div id="map"></div></div>

  <div id="radar_panel">
    <div class="rp_header" style="display:flex;align-items:center">
      <span>◈ AERIS-10 RADAR — 6-PANEL FMCW</span>
      <button id="stop_track_btn" onclick="stopTracking()">■ STOP TRACK</button>
    </div>
    <div class="drone_tabs" id="radar_tabs"></div>
    <div id="radar_svg_wrap">
      <svg id="polar" width="260" height="260" viewBox="0 0 260 260">
        <g id="polar_bg"></g>
        <g id="polar_wedges"></g>
        <g id="polar_dots"></g>
        <g id="polar_sweep">
          <line class="glow" x1="130" y1="130" x2="130" y2="22"/>
          <line x1="130" y1="130" x2="130" y2="22"/>
        </g>
        <g id="polar_labels"></g>
      </svg>
    </div>
    <div id="panel_stats"></div>
    <div id="radar_tgt_list"></div>
  </div>
</div>

<div style="padding:0 10px 0;flex-shrink:0">
  <div id="event_wrap">
    <div id="event_log"></div>
  </div>
</div>

<div id="ai_panel">
  <div class="ai_header">&#11042; LLM TACTICAL ENGINE &nbsp;|&nbsp; {{ ai_model }}</div>
  <div id="ai_form">
    <input type="text" id="ai_input" placeholder="OPERATOR COMMAND — e.g. RTL DRONE-2, orbit ALPHA-2, abort mission..." autocomplete="off"/>
    <button id="ai_submit" onclick="submitAICmd()">SEND</button>
  </div>
  <div id="ai_response">READY</div>
</div>

<script>
const COLORS  = {{ colors|tojson }};
const GEO     = {{ geo|tojson }};
const N       = {{ n_drones }};
const PANELS  = ['A','B','C','D','E','F'];
const PANEL_ANGLES = {A:0, B:60, C:120, D:180, E:240, F:300}; // centre deg CCW from North

// ── Map ──────────────────────────────────────────────────────────────────────
const map = L.map('map', {zoomControl:true}).setView(GEO.home, 15);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',
  {attribution:'OSM', maxZoom:19}).addTo(map);

const droneMarkers = {};
const droneTrails  = {};
const trailPoints  = {};

function makeDroneIcon(idx) {
  const c = COLORS[idx];
  return L.divIcon({
    html: `<div style="background:${c};width:14px;height:14px;border-radius:50%;
                border:2px solid white;box-shadow:0 0 6px ${c}"></div>`,
    iconSize:[14,14], iconAnchor:[7,7], className:''
  });
}

// Create drone markers at home positions
GEO.home && [0,1,2,3,4].forEach(i => {
  const lat = GEO.home[0] + (i-2)*0.0001;
  const lon = GEO.home[1] + (i-2)*0.0001;
  droneMarkers[i] = L.marker([lat,lon], {icon:makeDroneIcon(i)})
    .bindTooltip(`DRONE-${i}`, {permanent:false}).addTo(map);
  droneTrails[i]  = L.polyline([], {color:COLORS[i], weight:2, opacity:0.7}).addTo(map);
  trailPoints[i]  = [];
});

// Sector polygons (survey rows per drone)
GEO.sectors.forEach(s => {
  if (!s.wps || s.wps.length < 2) return;
  L.polyline(s.wps, {color:s.color, weight:2, dashArray:'5,5', opacity:0.5}).addTo(map);
  s.wps.forEach(([la,lo]) =>
    L.circleMarker([la,lo],{radius:3,color:s.color,fillOpacity:1}).addTo(map));
});

// NFZ circles
GEO.nfz.forEach(z => {
  L.circle([z.lat,z.lon], {radius:z.r, color:'#f85149', fillColor:'#f85149',
    fillOpacity:.08, dashArray:'6,4', weight:1.5}).addTo(map)
   .bindTooltip(`NFZ: ${z.name}`);
});

// Home marker
L.marker(GEO.home, {icon:L.divIcon({
  html:'<div style="font-size:18px">🏠</div>',iconSize:[20,20],iconAnchor:[10,10],className:''
})}).bindTooltip('HOME').addTo(map);

// Primary target
L.marker(GEO.primary, {icon:L.divIcon({
  html:'<div style="font-size:16px">🎯</div>',iconSize:[16,16],iconAnchor:[8,8],className:''
})}).bindTooltip('PRIMARY TARGET').addTo(map);

// Radar target markers
let targetMarkers = [];
function updateTargets(tracks) {
  targetMarkers.forEach(m => map.removeLayer(m));
  targetMarkers = [];
  tracks.forEach(t => {
    if (!t.lat || !t.lon) return;
    const m = L.circleMarker([t.lat,t.lon],
      {radius:6, color:'#ff6b35', fillColor:'#ff6b35', fillOpacity:.8, weight:1.5})
      .bindTooltip(`${t.id} R=${t.range_m}m`).addTo(map);
    targetMarkers.push(m);
  });
}

// ── Drone grid ───────────────────────────────────────────────────────────────
function buildDroneGrid() {
  const g = document.getElementById('drone_grid');
  g.innerHTML = '';
  for (let i=0; i<N; i++) {
    const c = COLORS[i];
    g.innerHTML += `<div class="drone_card" id="dc_${i}" style="--dc:${c}">
      <div class="dc_header">
        <div class="dc_dot"></div>
        <span class="dc_id">DRONE-${i}</span>
        <span class="dc_phase" id="dc_phase_${i}">STANDBY</span>
        <span class="dot_conn bad" id="dc_conn_${i}"></span>
      </div>
      <div class="dc_row"><span>ALT</span><span class="dc_val" id="dc_alt_${i}">0m</span></div>
      <div class="dc_row"><span>SPD</span><span class="dc_val" id="dc_spd_${i}">0m/s</span></div>
      <div class="dc_row"><span>WP</span><span class="dc_val" id="dc_wp_${i}">0/0</span></div>
      <div class="wp_bar"><div class="wp_fill" id="dc_wpbar_${i}" style="width:0%"></div></div>
    </div>`;
  }
}
buildDroneGrid();

function updateDroneCards(drones) {
  drones.forEach((d,i) => {
    const card    = document.getElementById(`dc_${i}`);
    const failed  = d.phase==='FAILED';
    card.className = 'drone_card' + (d.armed?' armed':'') + (failed?' failed':'');

    document.getElementById(`dc_phase_${i}`).textContent = d.phase||'---';
    document.getElementById(`dc_alt_${i}`).textContent   = `${d.alt}m`;
    document.getElementById(`dc_spd_${i}`).textContent   = `${d.groundspeed}m/s`;
    const wc = d.wp_current||0, wt = d.wp_total||0;
    document.getElementById(`dc_wp_${i}`).textContent    = `${wc}/${wt}`;
    const pct = wt>0 ? Math.round(100*wc/wt) : 0;
    document.getElementById(`dc_wpbar_${i}`).style.width = pct+'%';
    const conn = document.getElementById(`dc_conn_${i}`);
    conn.className = 'dot_conn '+(d.connected?'ok':'bad');

    // Update map marker
    if (d.lat && d.lon && Math.abs(d.lat)>0.001) {
      droneMarkers[i].setLatLng([d.lat,d.lon]);
      droneMarkers[i].setTooltipContent(`DRONE-${i} | ${d.phase} | ${d.alt}m`);
      // Trail
      trailPoints[i].push([d.lat,d.lon]);
      if (trailPoints[i].length>150) trailPoints[i].shift();
      droneTrails[i].setLatLngs(trailPoints[i]);
    }
  });
}

// ── Radar polar SVG ──────────────────────────────────────────────────────────
let selectedDrone = 0;

function buildRadarTabs() {
  const tb = document.getElementById('radar_tabs');
  tb.innerHTML = '';
  for (let i=0;i<N;i++) {
    tb.innerHTML += `<button class="dtab${i===0?' active':''}"
      style="--dc:${COLORS[i]}" onclick="selectDrone(${i})" id="rtab_${i}">D${i}</button>`;
  }
}
buildRadarTabs();

function selectDrone(idx) {
  selectedDrone = idx;
  document.querySelectorAll('.dtab').forEach((t,i)=>{
    t.className='dtab'+(i===idx?' active':'');
  });
}

const RADAR_DISPLAY_MAX_M = 300; // range at which dot reaches outer ring

function drawRadarPolar(panelData, tracks) {
  const cx=130, cy=130, R=110, rInner=20;
  const pd = panelData[selectedDrone] || {};

  // ── Background rings ──────────────────────────────────────────────
  let bgHtml = '';
  [0.25,0.5,0.75,1.0].forEach(f=>{
    bgHtml += `<circle cx="${cx}" cy="${cy}" r="${R*f}" fill="none"
      stroke="#0e2340" stroke-width="0.7" stroke-dasharray="3,3"/>`;
  });
  // Cross hairs
  bgHtml += `<line x1="${cx-R}" y1="${cy}" x2="${cx+R}" y2="${cy}"
    stroke="#0e2340" stroke-width="0.5"/>`;
  bgHtml += `<line x1="${cx}" y1="${cy-R}" x2="${cx}" y2="${cy+R}"
    stroke="#0e2340" stroke-width="0.5"/>`;
  // Range labels
  [75,150,225,300].forEach((m,i)=>{
    bgHtml += `<text x="${cx+4}" y="${cy-(R*(i+1)*0.25)+3}"
      font-size="7" fill="#1e4060" font-family="Courier New">${m}m</text>`;
  });
  document.getElementById('polar_bg').innerHTML = bgHtml;

  // ── Wedges ────────────────────────────────────────────────────────
  let wedgeHtml = '';
  PANELS.forEach(p=>{
    const startDeg = PANEL_ANGLES[p]-30, endDeg = PANEL_ANGLES[p]+30;
    const active   = pd[p] && pd[p].active;
    const col      = active ? '#ff3d3d' : '#080f1c';
    const stroke   = active ? '#ff3d3d' : '#0e2340';
    const path     = svgArc(cx,cy,R,rInner,startDeg,endDeg);
    const rng      = pd[p] ? pd[p].range_m : 0;
    wedgeHtml += `<path d="${path}" fill="${col}" fill-opacity="${active?0.55:0.4}"
      stroke="${stroke}" stroke-width="1"/>`;
    const la = Math.PI/2 - (PANEL_ANGLES[p]*Math.PI/180);
    const lr = R*0.72;
    const lx = cx+Math.cos(la)*lr, ly = cy-Math.sin(la)*lr;
    const tc = active ? '#ffffff' : '#1e4060';
    wedgeHtml += `<text x="${lx}" y="${ly+4}" text-anchor="middle"
      font-family="Courier New" font-size="11" fill="${tc}"
      font-weight="${active?'bold':'normal'}">${p}</text>`;
    if (active && rng>0) {
      wedgeHtml += `<text x="${lx}" y="${ly+14}" text-anchor="middle"
        font-family="Courier New" font-size="8" fill="#ff9090">${rng}m</text>`;
    }
  });
  // Center disc
  wedgeHtml += `<circle cx="${cx}" cy="${cy}" r="${rInner}" fill="#060b14" stroke="#0e2340"/>`;
  wedgeHtml += `<text x="${cx}" y="${cy+4}" text-anchor="middle" font-size="9"
    font-family="Courier New" fill="#00c8ff">D${selectedDrone}</text>`;
  // North tick
  wedgeHtml += `<line x1="${cx}" y1="${cy-rInner}" x2="${cx}" y2="${cy-R-4}"
    stroke="#00ff9d" stroke-width="1" stroke-dasharray="2,2"/>`;
  wedgeHtml += `<text x="${cx}" y="${cy-R-7}" text-anchor="middle" font-size="8"
    font-family="Courier New" fill="#00ff9d">N</text>`;
  document.getElementById('polar_wedges').innerHTML = wedgeHtml;

  // ── Target dots ───────────────────────────────────────────────────
  let dotsHtml = '';
  (tracks || []).forEach(t => {
    const rng = t.range_m || 0;
    const brg = t.bearing_deg || 0;
    if (rng <= 0) return;
    const r = Math.min(
      rInner + (rng / RADAR_DISPLAY_MAX_M) * (R - rInner),
      R - 5
    );
    const toRad = d => (90 - d) * Math.PI / 180;
    const angle = toRad(brg);
    const dx = cx + r * Math.cos(angle);
    const dy = cy - r * Math.sin(angle);
    // Outer ring (echo)
    dotsHtml += `<circle cx="${dx.toFixed(1)}" cy="${dy.toFixed(1)}" r="7"
      fill="none" stroke="#ff3d3d" stroke-width="0.8" opacity="0.45"/>`;
    // Solid blip
    dotsHtml += `<circle cx="${dx.toFixed(1)}" cy="${dy.toFixed(1)}" r="3.5"
      fill="#ff3d3d" stroke="#ffaaaa" stroke-width="0.6"/>`;
    // Label
    const label = (t.id||'?').replace('TGT_','T');
    dotsHtml += `<text x="${(dx+9).toFixed(1)}" y="${(dy+3).toFixed(1)}"
      font-size="8" fill="#ffaaaa" font-family="Courier New">${label}</text>`;
  });
  document.getElementById('polar_dots').innerHTML = dotsHtml;
}

function svgArc(cx,cy,R,rInner,startDeg,endDeg) {
  // startDeg/endDeg: degrees CW from North
  const toRad = d => (90-d)*Math.PI/180;
  const s1=toRad(startDeg), s2=toRad(endDeg);
  const x1=cx+R*Math.cos(s1),     y1=cy-R*Math.sin(s1);
  const x2=cx+R*Math.cos(s2),     y2=cy-R*Math.sin(s2);
  const ix1=cx+rInner*Math.cos(s1),iy1=cy-rInner*Math.sin(s1);
  const ix2=cx+rInner*Math.cos(s2),iy2=cy-rInner*Math.sin(s2);
  return `M${ix1} ${iy1} L${x1} ${y1} A${R} ${R} 0 0 0 ${x2} ${y2} L${ix2} ${iy2} A${rInner} ${rInner} 0 0 1 ${ix1} ${iy1} Z`;
}

function updatePanelStats(panelData) {
  const pd = panelData[selectedDrone] || {};
  const el = document.getElementById('panel_stats');
  el.innerHTML = PANELS.map(p => {
    const d      = pd[p] || {};
    const active = d.active;
    return `<div class="ps_cell${active?' active':''}">
      <div class="ps_lbl">Panel ${p}</div>
      <div class="ps_cnt" style="color:${active?'#f85149':'#3fb950'}">${d.count||0}</div>
      <div class="ps_rng">${d.range_m>0 ? d.range_m+'m' : '—'}</div>
    </div>`;
  }).join('');
}

// ── Follow / tracking ─────────────────────────────────────────────────────────
let _trackingId = null;

function trackTarget(targetId) {
  _trackingId = targetId;
  document.getElementById('stop_track_btn').classList.add('visible');
  fetch('/api/track', {
    method: 'POST',
    headers: {'Content-Type':'application/json'},
    body: JSON.stringify({target_id: targetId, drone_idx: 0}),
  }).then(r => r.json()).then(d => {
    if (d.error) { alert('Track error: ' + d.error); _trackingId = null; }
  }).catch(()=>{ _trackingId = null; });
}

function stopTracking() {
  _trackingId = null;
  document.getElementById('stop_track_btn').classList.remove('visible');
  fetch('/api/track', {
    method: 'POST',
    headers: {'Content-Type':'application/json'},
    body: JSON.stringify({stop: true}),
  });
}

function updateTargetList(tracks) {
  const el  = document.getElementById('radar_tgt_list');
  const rel = tracks.filter(t=>t).slice(0,10);
  el.textContent = '';
  if (!rel.length) {
    const empty = document.createElement('div');
    empty.style.cssText = 'color:#4a7a9b;font-size:10px;padding:4px';
    empty.textContent = 'No targets';
    el.append(empty);
    return;
  }
  for (const t of rel) {
    const tid        = t.id || '?';
    const isTracking = tid === _trackingId;
    const row = document.createElement('div');
    row.className = 'tgt_row';
    const idSpan = document.createElement('span');
    idSpan.className = 'tgt_id';
    idSpan.textContent = tid;
    const rSpan = document.createElement('span');
    rSpan.textContent = `R=${t.range_m||0}m`;
    const azSpan = document.createElement('span');
    azSpan.textContent = `Az=${(t.bearing_deg||0).toFixed(0)}°`;
    const btn = document.createElement('button');
    btn.className = `tgt_track_btn${isTracking?' tracking':''}`;
    btn.textContent = isTracking ? '▶ ON' : 'TRACK';
    btn.addEventListener('click', () => trackTarget(tid));
    row.append(idSpan, rSpan, azSpan, btn);
    el.append(row);
  }
}

// ── Event log ────────────────────────────────────────────────────────────────
function updateEventLog(events) {
  const el = document.getElementById('event_log');
  el.textContent = '';
  for (const e of events) {
    const kindClass = e.kind==='redistrib'?'redistrib':e.kind==='failure'?'failure':e.kind;
    const row = document.createElement('div');
    row.className = 'ev_row';
    const ts = document.createElement('span');
    ts.className = 'ev_ts';
    ts.textContent = e.ts;
    const msg = document.createElement('span');
    msg.className = `ev_msg ${kindClass}`;
    msg.textContent = e.msg;
    row.append(ts, msg);
    el.append(row);
  }
}

// ── Contact banner ────────────────────────────────────────────────────────────
let _lastContactMsg = '';
let _bannerTimer = null;
function showContactBanner(msg) {
  const b = document.getElementById('contact_banner');
  b.textContent = '⚠ ' + msg;
  b.classList.add('visible');
  clearTimeout(_bannerTimer);
  _bannerTimer = setTimeout(() => b.classList.remove('visible'), 4500);
}

// ── Socket.IO ────────────────────────────────────────────────────────────────
const socket = io();
socket.on('connect',    ()=>{ document.getElementById('conn_status').textContent='● LIVE'; document.getElementById('conn_status').style.color='#3fb950'; });
socket.on('disconnect', ()=>{ document.getElementById('conn_status').textContent='● OFFLINE'; document.getElementById('conn_status').style.color='#f85149'; });

socket.on('swarm_update', d => {
  document.getElementById('uptime').textContent     = d.uptime||'--';
  document.getElementById('scan_badge').textContent = `RADAR ${d.scan_count||0} scans`;

  // Contact banner — fire on first event if it's a new contact
  const evts = d.events || [];
  const latest = evts[0];
  if (latest && latest.kind === 'contact' && latest.msg !== _lastContactMsg) {
    _lastContactMsg = latest.msg;
    showContactBanner(latest.msg);
  }

  updateDroneCards(d.drones || []);
  updateTargets(d.tracks || []);
  drawRadarPolar(d.panels || {}, d.tracks || []);
  updatePanelStats(d.panels || {});
  updateTargetList(d.tracks || []);
  updateEventLog(d.events || []);
});

// ── AI command panel ─────────────────────────────────────────────────────────
async function submitAICmd() {
  const inp  = document.getElementById('ai_input');
  const resp = document.getElementById('ai_response');
  const btn  = document.getElementById('ai_submit');
  const cmd  = inp.value.trim();
  if (!cmd) return;
  btn.disabled = true;
  resp.textContent = 'PROCESSING...';
  resp.style.color = 'var(--warn)';
  try {
    const r = await fetch('/api/ai_command', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({command: cmd}),
    });
    const d = await r.json();
    resp.textContent = '';
    resp.style.color = '';
    if (d.ok) {
      const delta = d.delta;
      const src   = delta.source || '?';
      const conf  = ((delta.confidence || 0) * 100).toFixed(0);
      const tgts  = (delta.target_drones || []).map(i => `D${i}`).join(',') || 'ALL';
      const sp1 = document.createElement('span'); sp1.style.color='var(--accent2)'; sp1.textContent='▶ '+String(delta.action||'');
      const sp2 = document.createElement('span'); sp2.style.color='var(--text)';    sp2.textContent=' '+tgts;
      const sp3 = document.createElement('span'); sp3.style.color='var(--textdim)'; sp3.textContent=' '+String(delta.reasoning||'');
      const sp4 = document.createElement('span'); sp4.style.color='var(--dim)';     sp4.textContent=' ['+src+' '+conf+'%]';
      resp.append(sp1, sp2, sp3, sp4);
    } else {
      const sp = document.createElement('span'); sp.style.color='var(--danger)'; sp.textContent='✗ REJECTED: '+String(d.reason||'');
      resp.append(sp);
    }
    inp.value = '';
  } catch(e) {
    resp.textContent = '';
    const sp = document.createElement('span'); sp.style.color='var(--danger)'; sp.textContent='✗ GCS ERROR';
    resp.append(sp);
  }
  btn.disabled = false;
}
document.addEventListener('DOMContentLoaded', () => {
  document.getElementById('ai_input').addEventListener('keydown', e => {
    if (e.key === 'Enter') submitAICmd();
  });
});
</script>
</body>
</html>"""


@app.route("/")
def index():
    import json
    geo = _build_map_geometry()
    from mission_ai import MODEL as _ai_model
    return render_template_string(
        _HTML,
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
