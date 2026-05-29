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

import math
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
from mission_config_swarm import (
    SWARM_NUM_DRONES,
    DRONE_SECTORS,
    drone_alt,
    generate_drone_wps,
)

app    = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

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
    global radar_scan_count, radar_tracks
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
            now = time.time()
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
    payload = request.get_json(silent=True) or {}
    msg  = payload.get("msg",  "event")
    kind = payload.get("kind", "info")
    _push_event(msg, kind)
    return jsonify({"ok": True})


@app.route("/lidar_update", methods=["POST"])
def lidar_update():
    """Compatibility stub — single-drone lidar data ignored in swarm mode."""
    return jsonify({"ok": True, "commands": {}})


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
        now = time.time()
        with _state_lock:
            drones  = [dict(swarm_state[i]) for i in range(SWARM_NUM_DRONES)]
            tracks  = list(radar_tracks)
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
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.7.5/socket.io.min.js"></script>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{background:#0d1117;color:#e6edf3;font-family:'Courier New',monospace;font-size:13px;height:100vh;display:flex;flex-direction:column}
header{background:#161b22;border-bottom:1px solid #30363d;padding:6px 14px;display:flex;align-items:center;gap:16px;flex-shrink:0}
header h1{font-size:15px;color:#58a6ff;letter-spacing:1px}
#uptime{color:#7d8590;font-size:11px}
#scan_badge{background:#21262d;border:1px solid #30363d;border-radius:4px;padding:2px 8px;font-size:11px}

/* Drone cards */
#drone_grid{display:grid;grid-template-columns:repeat(5,1fr);gap:6px;padding:6px 10px;flex-shrink:0}
.drone_card{background:#161b22;border:2px solid #30363d;border-radius:6px;padding:8px 10px;transition:border-color .3s}
.drone_card.armed{border-color:var(--dc)}
.drone_card.failed{border-color:#f85149;background:#1a0c0c;opacity:.7}
.dc_header{display:flex;align-items:center;gap:6px;margin-bottom:5px}
.dc_dot{width:9px;height:9px;border-radius:50%;background:var(--dc)}
.dc_id{font-weight:bold;font-size:12px}
.dc_phase{font-size:10px;color:#7d8590;margin-left:auto}
.dc_row{display:flex;justify-content:space-between;font-size:10px;color:#7d8590;margin-top:2px}
.dc_val{color:#e6edf3}
.wp_bar{background:#21262d;border-radius:2px;height:4px;margin-top:4px;overflow:hidden}
.wp_fill{height:100%;border-radius:2px;background:var(--dc);transition:width .5s}
.dot_conn{display:inline-block;width:6px;height:6px;border-radius:50%;margin-left:4px}
.dot_conn.ok{background:#3fb950}
.dot_conn.bad{background:#f85149}

/* Main panels */
#main_row{display:grid;grid-template-columns:1fr 340px;gap:6px;padding:0 10px 6px;flex:1;min-height:0}
#map_wrap{position:relative;border-radius:6px;overflow:hidden;border:1px solid #30363d}
#map{width:100%;height:100%}

/* Radar panel */
#radar_panel{background:#161b22;border:1px solid #30363d;border-radius:6px;display:flex;flex-direction:column;min-height:0}
.rp_header{padding:7px 10px;border-bottom:1px solid #30363d;font-size:12px;color:#58a6ff;letter-spacing:.5px}
.drone_tabs{display:flex;border-bottom:1px solid #30363d}
.dtab{flex:1;padding:5px 0;text-align:center;font-size:11px;cursor:pointer;border:none;background:none;color:#7d8590;border-bottom:2px solid transparent}
.dtab.active{color:#e6edf3;border-bottom-color:var(--dc)}
.dtab:hover{color:#e6edf3}
#radar_svg_wrap{padding:10px;display:flex;justify-content:center}
svg#polar{display:block}
#panel_stats{padding:0 10px 8px;display:grid;grid-template-columns:repeat(3,1fr);gap:4px}
.ps_cell{background:#21262d;border-radius:4px;padding:5px 7px;text-align:center}
.ps_cell.active{background:#1a2a1a;border:1px solid #3fb950}
.ps_lbl{font-size:10px;color:#7d8590}
.ps_cnt{font-size:14px;font-weight:bold}
.ps_rng{font-size:9px;color:#7d8590}
#radar_tgt_list{flex:1;overflow-y:auto;padding:6px 10px;border-top:1px solid #30363d}
.tgt_row{display:flex;justify-content:space-between;font-size:10px;padding:2px 0;border-bottom:1px solid #21262d}
.tgt_id{color:#58a6ff}

/* Event log */
#event_wrap{grid-column:1/-1;background:#161b22;border:1px solid #30363d;border-radius:6px;padding:5px 10px;max-height:80px;overflow-y:auto}
#event_log{display:flex;flex-direction:column;gap:1px}
.ev_row{display:flex;gap:8px;font-size:10px}
.ev_ts{color:#7d8590;flex-shrink:0}
.ev_msg.redistrib{color:#f0883e}
.ev_msg.failure{color:#f85149}
.ev_msg.phase{color:#7d8590}
.ev_msg.info{color:#3fb950}
</style>
</head>
<body>
<header>
  <h1>⬡ MBC-3 SWARM COMMAND CENTER</h1>
  <span id="uptime">00:00:00</span>
  <span id="scan_badge">RADAR 0 scans</span>
  <span id="conn_status" style="color:#f85149;font-size:11px">● CONNECTING</span>
</header>

<div id="drone_grid"></div>

<div id="main_row">
  <div id="map_wrap"><div id="map"></div></div>

  <div id="radar_panel">
    <div class="rp_header">◈ AERIS-10 RADAR — 6-PANEL FMCW</div>
    <div class="drone_tabs" id="radar_tabs"></div>
    <div id="radar_svg_wrap">
      <svg id="polar" width="260" height="260" viewBox="0 0 260 260"></svg>
    </div>
    <div id="panel_stats"></div>
    <div id="radar_tgt_list"></div>
  </div>
</div>

<div style="padding:0 10px 6px;flex-shrink:0">
  <div id="event_wrap">
    <div id="event_log"></div>
  </div>
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

function drawRadarPolar(panelData) {
  const svg   = document.getElementById('polar');
  const cx=130, cy=130, R=110, rInner=20;
  const pd    = panelData[selectedDrone] || {};

  // Build wedge paths
  const wedges = PANELS.map(p => {
    const startDeg = PANEL_ANGLES[p]-30;
    const endDeg   = PANEL_ANGLES[p]+30;
    const active   = pd[p] && pd[p].active;
    const col      = active ? '#f85149' : '#1f2937';
    const stroke   = active ? '#f85149' : '#30363d';
    // Convert: SVG 0° = right, CCW. Map: 0° = North (top), CW.
    const s = svgArc(cx,cy,R,rInner, startDeg, endDeg);
    const count = pd[p] ? pd[p].count : 0;
    const rng   = pd[p] ? pd[p].range_m : 0;
    return {p, active, col, stroke, path:s, count, rng};
  });

  let html = '';
  // Range rings
  [0.25,0.5,0.75,1.0].forEach(f=>{
    html += `<circle cx="${cx}" cy="${cy}" r="${R*f}" fill="none"
      stroke="#30363d" stroke-width="0.5" stroke-dasharray="3,3"/>`;
  });
  // Wedges
  wedges.forEach(w=>{
    html += `<path d="${w.path}" fill="${w.col}" fill-opacity="${w.active?0.7:0.3}"
      stroke="${w.stroke}" stroke-width="1.2"/>`;
    // Panel label
    const la = Math.PI/2 - (PANEL_ANGLES[w.p]*Math.PI/180);
    const lr = R*0.72;
    const lx = cx+Math.cos(la)*lr, ly = cy-Math.sin(la)*lr;
    const tc = w.active ? '#ffffff' : '#7d8590';
    html += `<text x="${lx}" y="${ly+4}" text-anchor="middle"
      font-family="Courier New" font-size="11" fill="${tc}" font-weight="${w.active?'bold':'normal'}"
      >${w.p}</text>`;
    if (w.active && w.rng>0) {
      html += `<text x="${lx}" y="${ly+14}" text-anchor="middle"
        font-family="Courier New" font-size="8" fill="#f85149">${w.rng}m</text>`;
    }
  });
  // Center
  html += `<circle cx="${cx}" cy="${cy}" r="${rInner}" fill="#161b22" stroke="#30363d"/>`;
  html += `<text x="${cx}" y="${cy+4}" text-anchor="middle" font-size="9"
    font-family="Courier New" fill="#58a6ff">D${selectedDrone}</text>`;
  // North indicator
  html += `<line x1="${cx}" y1="${cy-rInner}" x2="${cx}" y2="${cy-R-4}"
    stroke="#3fb950" stroke-width="1" stroke-dasharray="2,2"/>`;
  html += `<text x="${cx}" y="${cy-R-7}" text-anchor="middle" font-size="8"
    font-family="Courier New" fill="#3fb950">N</text>`;

  svg.innerHTML = html;
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

function updateTargetList(tracks) {
  const el  = document.getElementById('radar_tgt_list');
  const rel = tracks.filter(t=>t).slice(0,10);
  if (!rel.length) { el.innerHTML='<div style="color:#7d8590;font-size:10px;padding:4px">No targets</div>'; return; }
  el.innerHTML = rel.map(t=>
    `<div class="tgt_row">
      <span class="tgt_id">${t.id||'?'}</span>
      <span>R=${t.range_m||0}m</span>
      <span>Az=${t.bearing_deg||0}°</span>
      <span style="color:#7d8590">${t.drone_id||''}</span>
    </div>`
  ).join('');
}

// ── Event log ────────────────────────────────────────────────────────────────
function updateEventLog(events) {
  const el = document.getElementById('event_log');
  el.innerHTML = events.map(e => {
    const kindClass = e.kind==='redistrib'?'redistrib':e.kind==='failure'?'failure':e.kind;
    return `<div class="ev_row"><span class="ev_ts">${e.ts}</span>
      <span class="ev_msg ${kindClass}">${e.msg}</span></div>`;
  }).join('');
}

// ── Socket.IO ────────────────────────────────────────────────────────────────
const socket = io();
socket.on('connect',    ()=>{ document.getElementById('conn_status').textContent='● LIVE'; document.getElementById('conn_status').style.color='#3fb950'; });
socket.on('disconnect', ()=>{ document.getElementById('conn_status').textContent='● OFFLINE'; document.getElementById('conn_status').style.color='#f85149'; });

socket.on('swarm_update', d => {
  document.getElementById('uptime').textContent     = d.uptime||'--';
  document.getElementById('scan_badge').textContent = `RADAR ${d.scan_count||0} scans`;

  updateDroneCards(d.drones || []);
  updateTargets(d.tracks || []);
  drawRadarPolar(d.panels || {});
  updatePanelStats(d.panels || {});
  updateTargetList(d.tracks || []);
  updateEventLog(d.events || []);
});
</script>
</body>
</html>"""


@app.route("/")
def index():
    import json
    geo = _build_map_geometry()
    return render_template_string(
        _HTML,
        colors=DRONE_COLORS,
        geo=geo,
        n_drones=SWARM_NUM_DRONES,
    )


# ── Entry point ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    import threading
    t = threading.Thread(target=_emit_loop, daemon=True)
    t.start()
    print("[SWARM-GCS] Swarm Command Center → http://localhost:5000", flush=True)
    print("[SWARM-GCS] Endpoints: /asp_update  /event_push  /api/state", flush=True)
    socketio.run(app, host="0.0.0.0", port=5000, debug=False,
                 allow_unsafe_werkzeug=True)
