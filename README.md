# Aran Technologies — MBC-3 Collaborative Drone Radar System

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

Single-drone simulation: autonomous survey grid → primary target orbit → RTL.

| Phase | Result | Detail |
|-------|--------|--------|
| PHASE 1 — Mission upload | ✅ Pass | 11 / 11 waypoints |
| PHASE 2 — Survey | ✅ Pass | 11 WPs · 0 avoidances |
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
4. [Phase I Demo Pipeline](#4-phase-i-demo-pipeline)
5. [Architecture](#5-architecture)
6. [Drone Model](#6-drone-model)
7. [Swarm Mission](#7-swarm-mission)
8. [Leader Election](#8-leader-election)
9. [AERIS-10 Radar + ROS2 Fusion](#9-aeris-10-radar--ros2-fusion)
10. [GCS Dashboards](#10-gcs-dashboards)
11. [Single-Drone Mission Mode](#11-single-drone-mission-mode)
12. [Demo Recording](#12-demo-recording)
13. [Mission Phases](#13-mission-phases)
14. [Configuration Reference](#14-configuration-reference)
15. [Dependencies](#15-dependencies)
16. [Demo Videos](#16-demo-videos)

---

## 1. System Overview

Full autonomous collaborative drone radar system built for the IAF MBC-3 competition. Two operating modes:

**Swarm mode** — 5 hexacopter drones execute parallel sector survey with FMCW radar, leader election, and failure redistribution. Each drone carries 6 × AERIS-10 FMCW radar panels providing full 360° coverage.

**Single-drone mode** — One drone with MPC obstacle avoidance, multi-target orbit sequencing, NFZ enforcement, and SDF-defined mission targets.

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
│   ├── swarm_telemetry_web.py   Swarm GCS dashboard (port 5000)
│   ├── swarm_monitor.py         Drone liveness monitor → /asp_update
│   ├── leader_election.py       Bully-style leader election (DEATH_TIMEOUT=15s)
│   ├── isr_lidar_mpc.py         Single-drone mission: survey→avoid→orbit→RTL
│   ├── telemetry_web.py         MBC-3 GCS (port 5000)
│   ├── mission_config.py        All mission constants (coords, NFZ, targets)
│   ├── mission_config_swarm.py  Swarm sector layout + redistribution helpers
│   ├── radar_sim.py             Pose-based FMCW radar simulator
│   ├── asp_bridge.py            AERIS-10 → GCS ASP bridge
│   ├── d2d_node.py              Drone-to-drone messaging (REASSIGN, status)
│   ├── mpc_controller.py        MPC engine (AvoidanceMPC / OrbitMPC / AltitudeMPC)
│   ├── pid_controller.py        PID fallback controller
│   ├── scenarios.json           Inactive — ISR targets defined in worlds/*.sdf
│   └── static/                  Locally-served JS/CSS (no CDN required on demo day)
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
├── launch.sh                    Single-drone launcher (STEP 1–6, set -m process groups)
├── record_demo.sh               Swarm demo recorder (300s)
├── record_single_drone.sh       Single-drone demo recorder (240s, Gazebo+GCS)
├── setup_ws.sh                  Build ros2_ws (radar_fusion + aeris10_driver)
├── kill_drone.sh                Kill one PX4 instance (triggers redistribution)
│
└── tools/
    ├── pre_demo_check.sh        7/7 pre-flight checks
    └── kill_drone_sim.sh        Kill specific SITL drone by index
```

---

## 3. Quick Start

### Prerequisites

```bash
# Ubuntu 24.04, ROS2 Jazzy, Gazebo Harmonic assumed installed
# PX4-Autopilot built at ~/PX4-Autopilot

pip install mavsdk flask flask-socketio requests numpy scipy scikit-learn
pip install colcon-common-extensions
```

### Build ROS2 workspace

```bash
bash setup_ws.sh
# Builds aeris10_driver + radar_fusion into ~/ros2_ws
```

### Pre-flight check

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

### Single-drone launch

```bash
./launch.sh                # with Gazebo GUI (default)
./launch.sh --headless     # headless — no Gazebo window
```

### Single-drone demo recording

```bash
bash record_single_drone.sh
# → ~/Documents/aran_mbc/mbc3_single_drone_demo.mp4  (240s, 1920×1080)
```

---

## 4. Phase I Demo Pipeline

End-to-end verified 2026-05-30.

```bash
# Step 1 — build workspace
bash setup_ws.sh

# Step 2 — verify all checks pass
bash tools/pre_demo_check.sh

# Step 3 — record demo video (automated)
bash record_demo.sh
# → ~/mbc3_swarm_demo.mp4  (300s, 1920×1080)
#   T+0s:   swarm launches, 5 drones arm + climb
#   T+90s:  sector survey active, radar detections on ASP
#   T+150s: DRONE-2 killed → leader re-elected → sector redistributed
#   T+300s: recording stops
```

**Submit to IAF:**
- Video: `~/mbc3_single_drone_demo.mp4`
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

### System diagram

![MBC-3 System Architecture](images/mbc3_radar_drone.svg)

---

## 6. Drone Model

Custom 6-arm hexacopter `mbc3_radar_drone`. All values derived from `new_drone/mbc3_radar_drone.sdf` and `new_drone/airframe/4601_gz_mbc3_radar_drone`.

### Geometry & mass

| Parameter | Value | Source |
|-----------|-------|--------|
| Configuration | 6-arm hexacopter, X-hex flat | SDF / airframe |
| Arm length | 360 mm | `CA_ROTOR0_PX` |
| Motor layout | 0° CCW, 60° CW, 120° CCW, 180° CW, 240° CCW, 300° CW | airframe |
| SDF base mass | 3.834 kg | `base_link` |
| Motor bell mass | 6 × 72.3 g = 433 g | `motor_bell_[0-5]` |
| **Total AUW** | **4.267 kg** | base + 6 × motor bells |
| Prop diameter | 276 mm (≈ 10.9 in) | SDF rotor radius 0.138 m |

### Motor & propulsion

| Parameter | Value | Source |
|-----------|-------|--------|
| Thrust coefficient kT | 2.800 × 10⁻⁵ N/(rad/s)² | `motorConstant` |
| Max rotor speed | 838 rad/s (~8,000 RPM) | `maxRotVelocity` |
| **Max thrust per motor** | **19.66 N** | kT × 838² |
| **Total max thrust** | **117.9 N** | 6 × 19.66 N |
| **Thrust-to-weight ratio** | **2.82** | 117.9 / (4.267 × 9.81) |
| Hover throttle (empirical) | 0.63 | flight log `13_58_15.ulg` |

### Battery & endurance

| Parameter | Value |
|-----------|-------|
| Battery | 6S LiPo, 10,000 mAh |
| Nominal voltage | 22.2 V |
| Est. hover endurance | ~32 min |

### Identifiers

| Parameter | Value |
|-----------|-------|
| Radar payload | AERIS-10 FMCW — 6 panels × 60° = 360° coverage |
| Frame material | Carbon fibre (competition build) |
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

---

## 7. Swarm Mission

`src/swarm_mission.py` — 5 asyncio coroutines running concurrently.

### Sector layout

Each drone surveys 2 contiguous rows from a 10-row boustrophedon grid. Altitudes staggered 10 m apart for deconfliction.

| Drone | Rows | Altitude AGL |
|-------|------|-------------|
| DRONE-0 | 0–1 | 30 m |
| DRONE-1 | 2–3 | 40 m |
| DRONE-2 | 4–5 | 50 m |
| DRONE-3 | 6–7 | 60 m |
| DRONE-4 | 8–9 | 70 m |

### Failure redistribution

When a drone fails (disconnects or enters FAILED phase):

1. Leader detects failure via `/api/swarm_state` polling
2. `compute_redistribution()` splits remaining waypoints across adjacent live drones
3. Leader sends `REASSIGN` message via `D2DNode`
4. Receiving drones queue extra WPs and fly them after their own sector completes
5. GCS receives `POST /event_push` — event shown in event log

---

## 8. Leader Election

`src/leader_election.py` — bully-style election.

**Rule:** Highest-index connected drone wins. `DRONE-4 > DRONE-3 > ... > DRONE-0`.

**Timing:**
- Poll interval: 2 s
- `DEATH_TIMEOUT`: 15 s — drone declared dead after 15 s no heartbeat
- Election completes: < 2 s after detection

**On election:**
1. POSTs new leader to `swarm_telemetry_web.py /api/leader`
2. `radar_sim.py` polls `/api/leader` and switches to new leader's pose
3. ASP screen shows `RADAR LEADER → DRONE-N (election #K)`

---

## 9. AERIS-10 Radar + ROS2 Fusion

### aeris10_driver

ROS2 package. Publishes FMCW radar detections on `/radar/scan`.

**Sim mode** (no hardware): generates 15 scatter points + one 200 m rotating target at 10 Hz. Activated by `sim_mode: true` in `aeris10_driver/config/aeris10_driver.yaml`.

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
bash tools/pre_demo_check.sh   # aeris10 sim → detection → /radar/targets
```

---

## 10. GCS Dashboards

Both GCS servers run on port 5000. Only one runs at a time.

### Swarm Command Center (`swarm_telemetry_web.py`)

| Panel | Content |
|-------|---------|
| Header | Mission uptime, radar scan count, connection status |
| Drone grid | 5 cards — phase, altitude, speed, WP progress, connection status |
| Leaflet map | 5 drone markers + trails, sector polygons, NFZ circles, radar targets |
| AERIS-10 radar | 6-panel polar display per drone (A–F, 60° each) — hits-per-scan + range |
| Event log | Redistribution events, failures, phase changes |

### MBC-3 GCS (`telemetry_web.py`)

Single-drone dashboard. Port 5000.

| Panel | Content |
|-------|---------|
| Left | Vehicle status, altitude/speed gauges, battery, compass |
| Center | Leaflet map (trail, survey grid, NFZ circles, orbit rings, sector overlay), live chart |
| Right | Mission phase list, WP progress, 360° avoidance panel, target queue, system log |
| ASP screen | `/asp` — radar track table + map, swarm markers, CSV download |

---

## 11. Single-Drone Mission Mode

`src/isr_lidar_mpc.py` + `launch.sh`

```bash
./launch.sh            # with Gazebo GUI
./launch.sh --headless # headless — no Gazebo window
```

### MPC avoidance pipeline (50 Hz)

```
Sensor scan → median filter → debounce (3 hits)
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

---

## 12. Demo Recording

Automated recorder — launches Gazebo GUI, opens Firefox GCS, tiles windows side-by-side, records mission from arm to RTL.

```bash
sudo apt install -y wmctrl   # one-time prerequisite

bash record_single_drone.sh
# → ~/Documents/aran_mbc/mbc3_single_drone_demo.mp4  (240s, 1920×1080, ~24 MB)
```

**Layout:** Gazebo 3D view (left 960px) | GCS dashboard in Firefox (right 960px)

**Timeline:**

| Time | Event |
|------|-------|
| T+0s | `launch.sh` starts — PX4 SITL + Gazebo + GCS |
| T+~30s | Drone armed, climbing to 30 m AGL |
| T+~45s | PHASE 2 — Survey grid (11 WPs) |
| T+~90s | PHASE 3 — PRIMARY orbit (50 m radius, locked ±0.5 m) |
| T+~180s | PHASE 5 — RTL, 3D map saved |
| T+240s | Recording stops |

**GCS shows during recording:**
- Drone position on Leaflet map (lat/lon trail)
- Armed status, flight mode, battery %
- Altitude, climb rate, groundspeed, heading
- Mission phase (STANDBY → SURVEY → LOITER → RTL)
- Avoidance sector overlay, NFZ boundaries, target markers

**Verified:** 2026-05-30 — full mission complete (exit 0)
**Release:** [v1.0.0-single-drone](https://github.com/lekkalaharsha/aran-mbc3-swarm/releases/tag/v1.0.0-single-drone)

---

## 13. Mission Phases

### Swarm

| Phase | Description |
|-------|-------------|
| `STANDBY` | Pre-arm, port allocation |
| `TAKEOFF` | Concurrent climb to sector altitude |
| `SURVEY` | Parallel boustrophedon sectors with FMCW radar active |
| `EXTRA` | Redistributed waypoints from failed drone |
| `RTL` | All active drones return and land |

### Single-drone

| Phase | Description |
|-------|-------------|
| `STANDBY` | Pre-arm, NFZ check, mission upload |
| `TAKEOFF` | Climb to survey altitude |
| `SURVEY` | Boustrophedon grid + MPC obstacle avoidance |
| `LOITER` | Primary target orbit (45 m radius, 90 m AGL, 30 s) |
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
| `ROW_SPACING` | 0.0003° | ~33 m row spacing |
| `ROW_WIDTH` | 0.0006° | ~47 m row length |
| `TARGET_LAT / LON` | 47.3985, 8.5470 | Primary target |
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

Avoidance constants (local to `isr_lidar_mpc.py`):

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
| ffmpeg (`~/.local/bin/ffmpeg`) | Demo recording |

### Install

```bash
pip install mavsdk flask flask-socketio requests numpy scipy scikit-learn
pip install colcon-common-extensions
sudo apt install python3-gz-transport13 python3-gz-msgs10
```

---

## 16. Demo Videos

Videos are **not tracked in git** (`.gitignore: *.mp4`) — stored locally only.

### Single-drone demo (Phase I submission)

```bash
sudo apt install -y wmctrl
bash record_single_drone.sh
```

| Property | Value |
|----------|-------|
| Output | `~/Documents/aran_mbc/mbc3_single_drone_demo.mp4` |
| Duration | 240 s (4 min) |
| Resolution | 1920×1080 |
| Layout | Gazebo 3D left (960px) \| GCS Firefox right (960px) |
| Size | ~24 MB |
| Phases covered | Survey → PRIMARY orbit → RTL |
| Verified | 2026-05-30, exit 0 |

### Swarm demo (5 drones)

```bash
bash record_demo.sh
```

| Property | Value |
|----------|-------|
| Output | `~/mbc3_swarm_demo.mp4` |
| Duration | 300 s (5 min) |
| Resolution | 1920×1080 |
| Phases covered | 5-drone parallel survey + drone kill + leader election + ASP |

### Re-record from scratch

```bash
pkill -9 -f "bin/px4"; pkill -9 -f "gz sim"; pkill -9 -f "telemetry_web"
pkill -9 -f "isr_lidar_mpc"; pkill -9 -f "mavsdk_server"; pkill -9 -f ffmpeg

bash record_single_drone.sh   # single-drone
# or
bash record_demo.sh           # swarm
```
