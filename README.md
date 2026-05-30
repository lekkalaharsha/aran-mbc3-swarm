# Aran Technologies — MBC-3 ISR Swarm Drone System

![IAF MBC-3](https://img.shields.io/badge/IAF-Mehar%20Baba%20Competition%203-1a237e?style=for-the-badge)
![Phase I](https://img.shields.io/badge/Phase%20I-Ready-brightgreen?style=for-the-badge)
![Mission](https://img.shields.io/badge/Mission-Verified%20Exit%200-brightgreen?style=for-the-badge)
![PX4 SITL](https://img.shields.io/badge/PX4-SITL-0d47a1?style=for-the-badge)
![ROS2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-22314e?style=for-the-badge)
![Gazebo Harmonic](https://img.shields.io/badge/Gazebo-Harmonic-f57c00?style=for-the-badge)
![Python](https://img.shields.io/badge/Python-3.10+-3776ab?style=for-the-badge&logo=python&logoColor=white)

> **Mehar Baba Competition-3 (MBC-3) — Indian Air Force**
> Aran Technologies | aranrobotics@gmail.com | Phase I — New Delhi, July 2026

---

## Mission Demo — Verified 30 May 2026

Single-drone ISR simulation: autonomous survey grid → primary target orbit → RTL.
**Layout:** Gazebo 3D view (left) · GCS dashboard (right) · 1920×1080

```
┌─────────────────────────┬─────────────────────────┐
│                         │   GCS DASHBOARD          │
│    GAZEBO 3D SIM        │  ● Armed  ▲ 30m AGL     │
│    [Hexacopter]         │  Phase: SURVEY           │
│    Survey grid →        │  WP: 7 / 11              │
│    Orbit 50m radius     │  Speed: 12 m/s           │
│                         │  Bat: 87%                │
└─────────────────────────┴─────────────────────────┘
```

| Phase | Result | Detail |
|-------|--------|--------|
| PHASE 1 — Mission upload | ✅ Pass | 11 / 11 waypoints |
| PHASE 2 — ISR survey | ✅ Pass | 11 WPs · 0 avoidances |
| PHASE 3 — PRIMARY orbit | ✅ Pass | 50.0 m radius · locked ±0.5 m |
| PHASE 5 — RTL + map save | ✅ Pass | Landed · 3D map saved |

> **Demo video (4 min · 1920×1080 · 24 MB)**

https://github.com/lekkalaharsha/aran-mbc3-swarm/releases/download/v1.0.0-single-drone/mbc3_single_drone_demo.mp4

> Re-record locally:
> ```bash
> sudo apt install -y wmctrl
> bash record_single_drone.sh
> # → ~/Documents/aran_mbc/mbc3_single_drone_demo.mp4
> ```

---

## Table of Contents

1. [System Overview](#1-system-overview)
2. [File Structure](#2-file-structure)
3. [Quick Start](#3-quick-start)
4. [Phase 0 Demo Pipeline](#4-phase-0-demo-pipeline)
5. [Architecture](#5-architecture)
6. [Drone Model](#6-drone-model)
7. [Swarm Mission](#7-swarm-mission)
8. [Leader Election](#8-leader-election)
9. [AERIS-10 Radar + ROS2 Fusion](#9-aeris-10-radar--ros2-fusion)
10. [GCS Dashboards](#10-gcs-dashboards)
11. [Single-Drone ISR Mode](#11-single-drone-isr-mode)
12. [Single-Drone Demo Recording](#12-single-drone-demo-recording)
13. [Mission Phases](#13-mission-phases)
14. [Configuration Reference](#14-configuration-reference)
15. [Dependencies](#15-dependencies)
16. [Demo Videos](#16-demo-videos)

---

## 1. System Overview

Full autonomous ISR (Intelligence, Surveillance, Reconnaissance) drone system built for the IAF MBC-3 competition. Two operating modes:

**Swarm mode (Phase 0 demo)** — 5 hexacopter drones execute parallel sector survey with FMCW radar, leader election, and failure redistribution.

**Single-drone ISR mode** — One drone with 360° LiDAR MPC avoidance, multi-target orbit, NFZ hard fences, and SDF-defined ISR targets.

```
  [PX4 SITL ×5 + Gazebo Harmonic]
         │ MAVLink UDP 14540–14544
         ▼
  [swarm_mission.py]  ◄── sector WPs, failure redistribution
  [leader_election.py]◄── bully election, /api/leader POST
  [radar_sim.py]      ◄── pose-based FMCW radar simulation
         │ POST /asp_update (5 Hz)
         ▼
  [swarm_telemetry_web.py]   ── 5-drone GCS, port 5000
         │ SocketIO (2 Hz)
         ▼
  [Browser — Swarm Command Center]
         ├── 5-drone status grid + trail map
         ├── AERIS-10 6-panel polar radar display
         └── Event log (redistribution / failures)

  ─────────────────────────────────────────
  ROS2 Jazzy workspace (~/ros2_ws)
  [aeris10_driver]  → /radar/scan  →  [radar_fusion/detection_node]
                                            └── /radar/targets (JSON)
```

![MBC-3 Swarm — 5× mbc3_radar_drone in Gazebo Harmonic](images/Screenshot%202026-05-22%20205131.png)

---

## 2. File Structure

```
.
├── src/
│   ├── swarm_mission.py         5-drone parallel mission + redistribution
│   ├── swarm_telemetry_web.py   Swarm GCS dashboard (military theme, port 5000)
│   ├── swarm_monitor.py         Drone liveness monitor → /asp_update
│   ├── leader_election.py       Bully-style leader election (DEATH_TIMEOUT=15s)
│   ├── isr_lidar_mpc.py         Single-drone ISR: survey→avoid→orbit→RTL
│   ├── telemetry_web.py         Single-drone GCS v13 (port 5000)
│   ├── mission_config.py        All mission constants (coords, NFZ, targets)
│   ├── mission_config_swarm.py  Swarm sector layout + redistribution helpers
│   ├── radar_sim.py             Pose-based FMCW radar simulator
│   ├── asp_bridge.py            AERIS-10 → GCS ASP bridge
│   ├── d2d_node.py              Drone-to-drone messaging (REASSIGN, status)
│   ├── mpc_controller.py        MPC engine (AvoidanceMPC / OrbitMPC / AltitudeMPC)
│   ├── pid_controller.py        Legacy PID (reference / fallback)
│   ├── scenarios.json           LiDAR scenario registry (real ISR targets in worlds/*.sdf)
│   └── static/                  Locally-served JS/CSS (no CDN on demo day)
│       ├── leaflet.js / .css
│       ├── socket.io.min.js
│       └── chart.umd.min.js
│
├── aeris10_driver/              ROS2 package — USB AERIS-10 FMCW radar driver
│   └── aeris10_driver/aeris10_usb.py   (sim_mode: 15 scatter pts, 200m target)
│
├── radar_fusion/                ROS2 package — detection node (sklearn RF gate)
│   └── detection_node           ≥9 hits → confirmed target → /radar/targets
│
├── new_drone/
│   ├── mbc3_radar_drone.sdf     Hexacopter SDF (source of truth)
│   ├── mbc3_radar_drone.xacro   URDF/xacro equivalent
│   ├── airframe/4601_gz_mbc3_radar_drone  PX4 airframe params
│   └── install_px4_model.sh     Installs SDF + airframe to PX4 build
│
├── worlds/                      4 SDF Gazebo worlds (copied to PX4)
│
├── swarm_launch.sh              5-drone SITL launcher (PX4 + Gazebo + GCS + mission)
├── launch.sh                    Single-drone ISR launcher (STEP 1–6, set -m process groups)
├── record_demo.sh               Swarm demo recorder (300s → ~/mbc3_phase0_demo.mp4)
├── record_single_drone.sh       Single-drone ISR recorder (240s, Gazebo+GCS side-by-side → mbc3_single_drone_demo.mp4)
├── setup_ws.sh                  Build ros2_ws (radar_fusion + aeris10_driver)
├── kill_drone.sh                Kill one PX4 instance (triggers redistribution)
│
└── tools/
    ├── pre_demo_check.sh        7/7 pre-flight checks (aeris10 → detection → targets)
    └── kill_drone_sim.sh        Kill specific SITL drone by index
```

---

## 3. Quick Start

### Prerequisites

```bash
# Ubuntu 24.04, ROS2 Jazzy, Gazebo Harmonic assumed installed
# PX4-Autopilot built at ~/PX4-Autopilot

pip install mavsdk flask flask-socketio requests numpy scipy scikit-learn
pip install colcon-common-extensions  # for ros2_ws build
```

### Build ROS2 workspace

```bash
bash setup_ws.sh
# Builds aeris10_driver + radar_fusion into ~/ros2_ws
```

### Pre-flight check (7 items)

```bash
bash tools/pre_demo_check.sh
# Checks: colcon, sklearn, PX4 bin, SDF model, aeris10 sim mode,
#         detection_node launch, /radar/targets JSON output
```

### Swarm launch (competition mode)

```bash
MBC3_MODE=1 bash swarm_launch.sh
# Launches: 5× PX4 SITL + Gazebo + swarm GCS + swarm_mission + leader_election
# GCS → http://localhost:5000
```

### Kill one drone (failover demo)

```bash
bash tools/kill_drone_sim.sh 2   # kills DRONE-2, triggers redistribution
```

### Single-drone ISR launch

```bash
./launch.sh                          # with Gazebo GUI (default)
./launch.sh --headless               # headless — no Gazebo window
```

### Single-drone demo recording (automated)

```bash
bash record_single_drone.sh
# → ~/Documents/aran_mbc/mbc3_single_drone_demo.mp4  (240s, 1920×1080)
# Layout: Gazebo left (960px) | GCS Firefox right (960px)
# Covers: arm → survey → PRIMARY orbit → RTL  (secondary orbits skipped)
# Requires: wmctrl  →  sudo apt install -y wmctrl
# See section 12 for full timeline
```

---

## 4. Phase 0 Demo Pipeline

End-to-end verified 2026-05-29. Produces a 5-minute competition submission video.

```bash
# Step 1 — build workspace
bash setup_ws.sh

# Step 2 — verify 7/7 checks pass
bash tools/pre_demo_check.sh

# Step 3 — record demo video (automated, no interaction needed)
bash record_demo.sh
# → ~/mbc3_phase0_demo.mp4  (300s, 1920×1080)
#   T+0s:   swarm launches, 5 drones arm + climb
#   T+90s:  sector survey active, radar detections on ASP
#   T+150s: DRONE-2 killed → leader re-elected → sector redistributed
#   T+300s: recording stops
```

**Submit to IAF portal:**
- Video: `~/mbc3_phase0_demo.mp4`
- Vision doc: `competition/Final_Vision_Document_for_MBC_3_22Apr26.pdf`
- Form: `competition/Registration_form_MBC_3_final.pdf`

---

## 5. Architecture

### Swarm data flow

```
PX4 SITL ×5  ──MAVLink──►  swarm_mission.py
                                 │ drone positions (5 Hz)
                                 ▼
                         swarm_monitor.py  ──POST /asp_update──►  swarm_telemetry_web.py
                                                                         │ SocketIO
radar_sim.py ──POST /asp_update──►  swarm_telemetry_web.py              ▼
                                                                   Browser GCS
leader_election.py ──POST /api/leader──►  swarm_telemetry_web.py
```

### Thread / process model

| Process | Threads |
|---------|---------|
| `swarm_mission.py` | 5 asyncio coroutines (one per drone) + D2D thread |
| `leader_election.py` | Single polling loop (2s interval) |
| `swarm_telemetry_web.py` | Flask + `_emit_loop` daemon thread (0.5 Hz) |
| `radar_sim.py` | Single loop (10 Hz) |

### Shared state (thread safety)

- `swarm_state` — guarded by `_state_lock` (RLock)
- `panel_state` — guarded by same lock; count resets each scan
- `event_log` — `deque(maxlen=200)`, appendleft is GIL-atomic

### System diagram

![MBC-3 System Architecture](images/mbc3_radar_drone.svg)

---

## 6. Drone Model

Custom 6-arm hexacopter `mbc3_radar_drone`.

| Parameter | Value |
|-----------|-------|
| Arms | 6 |
| Motor config | X-hex |
| SDF mass | ~3.8 kg |
| Battery | 6S LiPo |
| Frame | Carbon fibre (competition build) |
| Payload | AERIS-10 FMCW radar module |
| SDF source | `new_drone/mbc3_radar_drone.sdf` |
| PX4 airframe | `4601_gz_mbc3_radar_drone` |

### Assembly reference


| Top (orthographic) | Front |
|---|---|
| ![Top view](images/assembly_reference/back%20view%20.jpeg) | ![Front view](images/assembly_reference/side%20view.jpeg) |

| 3/4 Front | Top (perspective) | Isometric |
|---|---|---|
| ![3/4 front](images/assembly_reference/WhatsApp%20Image%202026-05-26%20at%206.31.59%20PM.jpeg) | ![Top perspective](images/assembly_reference/WhatsApp%20Image%202026-05-26%20at%206.32.00%20PM.jpeg) | ![Isometric](images/assembly_reference/WhatsApp%20Image%202026-05-26%20at%206.32.01%20PM.jpeg) |

**Install model into PX4:**

```bash
bash new_drone/install_px4_model.sh
```

**After any airframe change**, copy to both locations and clear EEPROM:

```bash
cp new_drone/airframe/4601_gz_mbc3_radar_drone \
   ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/
cp new_drone/airframe/4601_gz_mbc3_radar_drone \
   ~/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/airframes/
chmod +x ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4601_gz_mbc3_radar_drone
chmod +x ~/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/airframes/4601_gz_mbc3_radar_drone
rm -f ~/PX4-Autopilot/build/px4_sitl_default/rootfs/eeprom/parameters
```

**SITL pre-arm params** (already in airframe file):

| Param | Value | Reason |
|-------|-------|--------|
| `CBRK_SUPPLY_CHK` | 894281 | gz-sim-linearbattery-plugin not installed → disables supply check |
| `EKF2_ABL_LIM` | 0.8 | Default 0.4 too tight at SITL startup; raised to clear arm check |

---

## 7. Swarm Mission

`src/swarm_mission.py` — 5 asyncio coroutines running concurrently.

### Sector layout

Each drone surveys 2 contiguous rows from a 10-row boustrophedon grid. Altitudes staggered 10m apart for deconfliction.

| Drone | Rows | Altitude AGL |
|-------|------|-------------|
| DRONE-0 | 0–1 | 30m |
| DRONE-1 | 2–3 | 40m |
| DRONE-2 | 4–5 | 50m |
| DRONE-3 | 6–7 | 60m |
| DRONE-4 | 8–9 | 70m |

### Failure redistribution

When a drone fails (disconnects or enters FAILED phase):

1. Leader detects failure via `/api/swarm_state` polling
2. `compute_redistribution()` splits remaining waypoints across adjacent live drones
3. Leader sends `REASSIGN` message via `D2DNode`
4. Receiving drones queue extra WPs in `EXTRA_WPS[idx]`
5. Extra WPs uploaded and flown after own sector completes
6. GCS receives `POST /event_push` — redistribution event shown in event log

---

## 8. Leader Election

`src/leader_election.py` — bully-style election.

**Rule:** Highest-index connected drone wins. `DRONE-4 > DRONE-3 > ... > DRONE-0`.
DRONE-0 is never killed in demo (owns the Gazebo world).

**Timing:**
- Poll interval: 2s
- `DEATH_TIMEOUT`: 15s — drone declared dead after 15s no heartbeat
- Election completes: <2s after detection

**On election:**
1. POSTs new leader to `swarm_telemetry_web.py /api/leader`
2. `radar_sim.py` polls `/api/leader` and switches to new leader's pose
3. ASP screen shows `RADAR LEADER → DRONE-N (election #K)`

---

## 9. AERIS-10 Radar + ROS2 Fusion

### aeris10_driver

ROS2 package. Publishes FMCW radar detections on `/radar/scan`.

**Sim mode** (no hardware): generates 15 scatter points + one 200m rotating target at 10 Hz. Activated by `sim_mode: true` in `aeris10_driver/config/aeris10_driver.yaml`.

```bash
ros2 launch aeris10_driver aeris10_driver.launch.py
```

### radar_fusion / detection_node

Consumes `/radar/scan`, applies sklearn Random Forest gate (≥9 hits = confirmed target), publishes `/radar/targets` as JSON.

```bash
ros2 run radar_fusion detection_node
```

### End-to-end test

```bash
bash tools/pre_demo_check.sh   # check 7 → aeris10 sim → detection → /radar/targets
```

---

## 10. GCS Dashboards

Both GCS servers run on port 5000. Only one runs at a time.

### Swarm Command Center (`swarm_telemetry_web.py`)

Military-theme dashboard matching ISR GCS v13.

| Panel | Content |
|-------|---------|
| Header | Mission uptime, radar scan count, connection status |
| Drone grid | 5 cards — phase, alt, speed, WP progress bar, connection dot |
| Leaflet map | 5 drone markers + trails, sector polygons, NFZ circles, radar targets |
| AERIS-10 radar | 6-panel polar SVG per drone (A–F, 60° each) — hits-per-scan + range |
| Event log | Redistribution events, failures, phase changes |

**REST endpoints:**

| Method | Path | Description |
|--------|------|-------------|
| `POST` | `/asp_update` | Drone positions + radar tracks from swarm_mission / radar_sim |
| `POST` | `/event_push` | Redistribution / failure events from swarm_mission |
| `GET` | `/api/state` | JSON snapshot of all drone + radar state |
| `GET` | `/api/leader` | Current leader drone |
| `POST` | `/api/leader` | Update leader (from leader_election.py) |

### ISR GCS v13 (`telemetry_web.py`)

Single-drone dashboard. Port 5000.

| Panel | Content |
|-------|---------|
| Left | Vehicle status, altitude/speed gauges, battery, compass |
| Center | Leaflet map (trail, survey grid, NFZ circles, orbit rings, sector overlay), live chart |
| Right | Swarm status, mission phase list, WP progress, 360° LiDAR panel, target queue, PID live tune, system log |
| ASP screen | `/asp` — radar track table + map, swarm markers, CSV download |

**REST endpoints:**

| Method | Path | Description |
|--------|------|-------------|
| `POST` | `/lidar_update` | Push LiDAR + mission state (5 Hz from isr_lidar_mpc) |
| `POST` | `/asp_update` | Radar tracks |
| `POST` | `/pid_tune` | Live PID gain update |
| `GET` | `/pid_gains` | Current gains |
| `GET` | `/download_log` | CSV flight log |
| `GET` | `/asp_download` | CSV radar track log |
| `POST` | `/add_nfz` | Inject dynamic NFZ |
| `POST` | `/add_target` | Add ISR target to queue |
| `POST` | `/inject_event` | Inject timed LiDAR obstacle (sim mode) |
| `GET` | `/scenario_list` | All 24 scenario names |

### Static assets

JS/CSS libraries served locally — no CDN required on demo day:

```
src/static/leaflet.js       145 KB
src/static/leaflet.css       15 KB
src/static/socket.io.min.js  49 KB
src/static/chart.umd.min.js 196 KB
```

---

## 11. Single-Drone ISR Mode

`src/isr_lidar_mpc.py` + `launch.sh`

```bash
./launch.sh                          # LiDAR sim mode
./launch.sh --scenario urban_canyon  # named scenario
./launch.sh --headless               # no Gazebo GUI
./launch.sh --gcs-only               # GCS only (drone already running)
ISR_SIM_SCENARIO=urban_canyon python3 src/isr_lidar_mpc.py
```

### MPC avoidance pipeline (50 Hz)

```
LiDAR scan → median filter → debounce (3 hits)
    → best_escape_bearing() — world-frame sector analysis
    → AvoidanceMPC.compute_correction() — penetration-weighted offset
    → compute_avoidance_waypoint() — project along escape bearing
    → drone.action.goto_location()
    → [timeout >10s] → climb +15m escape
```

### MPC cost function

```
J = Σ_k [ Q_track·‖pos_err‖² + Q_vel·‖vel_err‖² + obs_penalty ]
  + Σ_k [ R_input·‖u[k]‖² + R_delta·‖Δu[k]‖² ]
  + Q_terminal·‖pos_err(N)‖²
```

**State vector:** `x = [north, east, down, vn, ve, vd]` (NED)
**Control input:** `u = [an, ae, ad]` (m·s⁻²)
**Solver:** L-BFGS-B via `scipy.optimize.minimize`

### Scenario system

Scenarios in `scenarios.json` are inactive — real ISR targets defined in `worlds/mbc3_isr_targets.sdf`.

```json
{
  "name": "urban_canyon",
  "description": "...",
  "events": [
    { "start_s": 3.0, "duration_s": 12.0, "bearing_deg": 45.0, "dist_m": 10.0 }
  ]
}
```

---

## 12. Single-Drone Demo Recording

Automated recorder — launches `launch.sh` (Gazebo GUI), polls GCS, opens Firefox, tiles windows side-by-side, records ISR mission from arm to RTL.

```bash
# Prerequisite (one-time)
sudo apt install -y wmctrl

bash record_single_drone.sh
# → ~/Documents/aran_mbc/mbc3_single_drone_demo.mp4  (240s, 1920×1080, ~24 MB)
```

**Layout:** Gazebo 3D view (left 960px) | GCS dashboard in Firefox (right 960px)

**Short demo mode** — secondary orbits (ALPHA-2, BRAVO-1, CHARLIE-3) are skipped via `MBC3_SKIP_SECONDARY=1`. To re-enable them, unset that variable or run `launch.sh` directly.

**Timeline:**

| Time | Event |
|------|-------|
| T+0s | `launch.sh` starts — PX4 SITL + Gazebo GUI + GCS |
| T+~30s | Drone armed, climbing to 30m AGL |
| T+~45s | PHASE 2 — Survey grid (11 WPs, 0 avoidances) |
| T+~90s | PHASE 3 — PRIMARY orbit (50m radius, locked ±0.5m) |
| T+~180s | PHASE 5 — RTL, 3D map saved |
| T+240s | Recording stops |

**GCS dashboard shows during recording:**
- Drone position on Leaflet map (lat/lon trail)
- Armed/disarmed status, flight mode, battery %
- Altitude, climb rate, groundspeed, heading
- Mission phase indicator (STANDBY → SURVEY → LOITER → RTL)
- LiDAR sector overlay, NFZ boundaries, target markers
- Mission STALE watchdog badge (orange, if push stops)

**Prerequisites:** `DISPLAY` set (run from desktop terminal, not SSH without X forwarding). ffmpeg at `~/.local/bin/ffmpeg`. wmctrl installed.

**Preview / submit:**
```bash
xdg-open ~/Documents/aran_mbc/mbc3_single_drone_demo.mp4

# Submit to IAF portal:
#   Video:  ~/Documents/aran_mbc/mbc3_single_drone_demo.mp4
#   Docs:   competition/Final_Vision_Document_for_MBC_3_22Apr26.pdf
#   Form:   competition/Registration_form_MBC_3_final.pdf
```

**Verified run:** 2026-05-30 — FULL ISR + LiDAR MPC MISSION COMPLETE v12-MPC-v5 (exit 0)  
**Short demo verified:** 2026-05-30 — 240s, 24 MB, Gazebo + GCS side-by-side (exit 0)  
**Release:** [v1.0.0-single-drone](https://github.com/lekkalaharsha/aran-mbc3-swarm/releases/tag/v1.0.0-single-drone)

---

## 13. Mission Phases

### Swarm

| Phase | All drones |
|-------|-----------|
| `STANDBY` | Pre-arm, port allocation |
| `TAKEOFF` | Concurrent climb to sector altitude |
| `SURVEY` | Parallel boustrophedon sectors |
| `EXTRA` | Redistributed WPs from failed drone |
| `RTL` | All active drones return + land |

### Single-drone ISR

| Phase | Description |
|-------|-------------|
| `STANDBY` | Pre-arm, NFZ check, mission upload |
| `TAKEOFF` | Climb to survey altitude |
| `SURVEY` | Boustrophedon grid + LiDAR avoidance |
| `LOITER` | Primary target orbit (45m radius, 90m AGL, 30s) |
| `SEC-1/2/3` | Secondary ISR target orbits |
| `RTL` | Return-to-launch → land |

---

## 14. Configuration Reference

All mission constants in `src/mission_config.py`.

| Constant | Default | Description |
|----------|---------|-------------|
| `HOME_LAT / HOME_LON` | 47.3977, 8.5456 | Launch / RTL position |
| `ALTITUDE` | 50.0 m | Survey cruise altitude AGL |
| `SPEED` | 50.0 m/s | Mission speed |
| `ROWS` | 4 | Survey grid rows |
| `ROW_SPACING` | 0.0003° | ~33m row spacing |
| `ROW_WIDTH` | 0.0006° | ~47m row length |
| `TARGET_LAT / LON` | 47.3985, 8.5470 | Primary ISR target |
| `ORBIT_RADIUS` | 45 m | Primary orbit radius |
| `ORBIT_ALTITUDE` | 90 m AGL | Primary orbit altitude |
| `ORBIT_DURATION` | 30 s | Primary orbit dwell |

Swarm constants in `src/mission_config_swarm.py`:

| Constant | Description |
|----------|-------------|
| `SWARM_NUM_DRONES` | 5 |
| `DRONE_SECTORS` | Row assignments per drone index |
| `drone_alt(i)` | Returns staggered altitude for drone i |
| `generate_drone_wps(i)` | Returns sector waypoints for drone i |
| `compute_redistribution(failed, active, remaining_wps)` | Returns redistribution map |

LiDAR constants (local to `isr_lidar_mpc.py`):

| Constant | Default | Description |
|----------|---------|-------------|
| `LIDAR_WARN_DIST` | 25 m | Warning threshold |
| `LIDAR_AVOID_DIST` | 15 m | Avoidance trigger |
| `AVOIDANCE_OFFSET_M` | 50 m | Minimum detour distance |
| `DEBOUNCE_COUNT` | 3 | Consecutive hits before avoidance |
| `AVOIDANCE_TIMEOUT_S` | 10 s | Time before climb escalation |
| `CLIMB_ESCAPE_M` | 15 m | Altitude gain on climb escape |

---

## 15. Dependencies

### Python

| Package | Version | Role |
|---------|---------|------|
| `mavsdk` | ≥1.4 | PX4 / MAVLink interface |
| `flask` | ≥3.0 | GCS web server |
| `flask-socketio` | ≥5.3 | Real-time telemetry push |
| `requests` | ≥2.31 | Mission → GCS HTTP push |
| `numpy` | ≥1.26 | MPC matrix ops |
| `scipy` | ≥1.12 | L-BFGS-B optimiser |
| `scikit-learn` | ≥1.4 | RF gate in radar_fusion detection_node |

### System

| Package | Role |
|---------|------|
| PX4-Autopilot (built) | SITL flight stack |
| Gazebo Harmonic | Simulation environment |
| ROS2 Jazzy | radar_fusion + aeris10_driver |
| colcon | ROS2 workspace build |
| ffmpeg (static `~/.local/bin/ffmpeg`) | Demo recording |

### Install

```bash
pip install mavsdk flask flask-socketio requests numpy scipy scikit-learn
pip install colcon-common-extensions
# Gazebo Python bindings (optional — real LiDAR)
sudo apt install python3-gz-transport13 python3-gz-msgs10
```

---

## 16. Demo Videos

Videos are **not tracked in git** (`.gitignore: *.mp4`) — stored locally only.

### Single-drone ISR demo (Phase I submission)

```bash
# Requires wmctrl (one-time): sudo apt install -y wmctrl
bash record_single_drone.sh
```

| Property | Value |
|----------|-------|
| Output | `~/Documents/aran_mbc/mbc3_single_drone_demo.mp4` |
| Duration | 240s (4 min) |
| Resolution | 1920×1080 |
| Layout | Gazebo 3D left (960px) \| GCS Firefox right (960px) |
| Size | ~24 MB |
| Phases covered | Survey → PRIMARY orbit → RTL (secondary orbits skipped) |
| Verified | 2026-05-30, exit 0 |

### Swarm demo (5 drones)

```bash
bash record_demo.sh
```

| Property | Value |
|----------|-------|
| Output | `~/mbc3_phase0_demo.mp4` |
| Duration | 300s (5 min) |
| Resolution | 1920×1080 |
| Phases covered | 5-drone parallel survey + drone kill + leader election + ASP |

### Re-record from scratch

```bash
# Kill everything first
pkill -9 -f "bin/px4"; pkill -9 -f "gz sim"; pkill -9 -f "telemetry_web"
pkill -9 -f "isr_lidar_mpc"; pkill -9 -f "mavsdk_server"; pkill -9 -f ffmpeg

# Then record
bash record_single_drone.sh   # single-drone ISR
# or
bash record_demo.sh           # swarm
```
