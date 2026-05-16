# Aran Technologies — ISR Mission + LiDAR MPC Avoidance
## v12-MPC | PX4 SITL + Gazebo Harmonic + MAVSDK Python

> **Nirmaan Incubation — IIT Hyderabad Demo Build**

---

## Table of Contents
1. [System Overview](#1-system-overview)
2. [File Structure](#2-file-structure)
3. [Quick Start](#3-quick-start)
4. [Architecture](#4-architecture)
5. [Mission Phases](#5-mission-phases)
6. [Bug Fixes — This Release](#6-bug-fixes--this-release)
7. [Configuration Reference](#7-configuration-reference)
8. [Scenario System](#8-scenario-system)
9. [GCS Dashboard](#9-gcs-dashboard)
10. [MPC Controller](#10-mpc-controller)
11. [PID Controller (Legacy)](#11-pid-controller-legacy)
12. [Dependencies](#12-dependencies)
13. [Troubleshooting](#13-troubleshooting)

---

## 1. System Overview

Full autonomous ISR (Intelligence, Surveillance, Reconnaissance) drone mission stack built on PX4 SITL, Gazebo Harmonic, and MAVSDK Python.

```
  [PX4 SITL + Gazebo]
         │ MAVLink UDP:14540
         ▼
  [isr_lidar_mpc.py]  ◄──── LiDAR scan (gz-transport or sim)
         │                   AvoidanceMPC / OrbitMPC / AltitudeMPC
         │ POST /lidar_update (5 Hz)
         ▼
  [telemetry_web.py]  ──── MAVSDK telemetry streams
         │ SocketIO (2.5 Hz)
         ▼
  [Browser GCS v13]   ──── Leaflet map, sector overlay, target panel
```

**Key capabilities:**
- 360° LiDAR obstacle avoidance with MPC (replaces legacy PID)
- Boustrophedon survey grid with multi-altitude stereo sweeps
- 4-target ISR orbit sequence (1 primary + 3 secondary)
- 3 No-Fly Zone hard fences (pre-flight + in-flight checks)
- 2 mid-survey loiter waypoints for persistent surveillance
- 24 named simulation scenarios (urban canyon, NFZ breach, etc.)
- Live GCS dashboard with Leaflet map, sector visualiser, log panel

---

## 2. File Structure

```
.
├── isr_lidar_mpc.py      Main mission script (survey → avoid → orbit → RTL)
├── mpc_controller.py     MPC engine + AvoidanceMPC / OrbitMPC / AltitudeMPC
├── pid_controller.py     Legacy PID stack (kept for reference / fallback)
├── mission_config.py     Single source of truth: coords, grid, NFZ, targets
├── telemetry_web.py      Flask/SocketIO GCS dashboard (v13)
├── scenarios.json        24 named LiDAR simulation scenarios
├── launch.sh             Orchestration script (SITL → GCS → Mission)
└── logs/                 Auto-created; timestamped per-run log files
```

---

## 3. Quick Start

### Prerequisites
```bash
# System
sudo apt install python3 python3-pip build-essential
# Gazebo Harmonic: https://gazebosim.org/docs/harmonic/install
# PX4-Autopilot: git clone https://github.com/PX4/PX4-Autopilot.git --recursive

# Python packages
pip install mavsdk flask flask-socketio requests numpy scipy
```

### Launch (full stack)
```bash
# Standard launch — LiDAR sim mode
./launch.sh

# With a named scenario
./launch.sh --scenario urban_canyon

# Headless (no Gazebo GUI)
./launch.sh --headless --scenario nfz_breach

# GCS only (drone already running)
./launch.sh --gcs-only

# SITL + Gazebo only, no mission
./launch.sh --sim-only

# Skip pip checks for fast re-launch
./launch.sh --no-deps
```

### Manual launch (split terminals)
```bash
# Terminal 1 — GCS
python3 telemetry_web.py
# → http://localhost:5000

# Terminal 2 — Mission (with optional scenario)
ISR_SIM_SCENARIO=urban_canyon python3 isr_lidar_mpc.py
```

---

## 4. Architecture

### Control Stack (v12-MPC)

| Component | Class | Role |
|-----------|-------|------|
| Obstacle avoidance | `AvoidanceMPC` | Lateral detour waypoint magnitude |
| Orbit radius hold | `OrbitMPC` | Radial error → acceleration cmd |
| Altitude hold | `AltitudeMPC` | Vertical acceleration cmd |
| MPC engine | `MPCEngine` | L-BFGS-B finite-horizon QP solver |

**State vector:** `x = [north, east, down, vn, ve, vd]` (NED, m / m·s⁻¹)  
**Control input:** `u = [an, ae, ad]` (acceleration, m·s⁻²)  
**Dynamics:** `x[k+1] = A·x[k] + B·u[k]` (ZOH discrete, dt=0.02 s)

### Avoidance Pipeline (50 Hz)
```
LiDAR scan → median filter → debounce (3 hits)
    → best_escape_bearing() (world-frame sector analysis)
    → AvoidanceMPC.compute_correction() (penetration-weighted offset)
    → compute_avoidance_waypoint() (project along escape bearing)
    → drone.action.goto_location()
    → [timeout >10s] → climb +15m escape
```

### Bearing Frame (Critical)
All bearings are **world-absolute** (0 = North, clockwise). The LiDAR sensor returns bearings in **sensor-relative** frame (0 = drone forward). Conversion: `world_bearing = (sensor_bearing + drone_heading) % 360`. This is applied in both `_bearing_to_nearest()` and `_compute_sectors()`.

### Shared State (Thread Safety)
- `lidar_state` — written by `lidar_gz_reader` / `lidar_sim_reader` (asyncio), read by `avoidance_loop` and `push_to_gcs` (thread)
- `avoidance_state` — written by `avoidance_loop`, read by `push_to_gcs`
- `drone_state` — written by `telemetry_tracker`, read everywhere
- GCS `data` / `lidar_data` — written by Flask endpoint thread, read by `emit_loop`

---

## 5. Mission Phases

| Phase | Label | Description |
|-------|-------|-------------|
| 1 | `STANDBY` | Pre-arm, NFZ fence check, mission upload |
| 2 | `SURVEY` | Boustrophedon grid, LiDAR avoidance active |
| 3 | `LOITER` | Primary target orbit (ORBIT_RADIUS=45m, 90m AGL) |
| 4.1–4.3 | `SEC-1/2/3` | Secondary ISR targets, sorted by priority |
| 5 | `RTL` | Return-to-launch → land |

**Loiter waypoints** are injected mid-survey at configurable WP indices (see `LOITER_WAYPOINTS` in `mission_config.py`).

---

## 6. Bug Fixes — This Release

### BUG-01 — `ISR_SIM_SCENARIO` env var ignored (Critical)
**File:** `isr_lidar_mpc.py` → `lidar_sim_reader()`  
**Symptom:** `./launch.sh --scenario urban_canyon` had no effect; the sim reader always ran the legacy single-obstacle hardcode.  
**Root cause:** `SIM_SCENARIO = None` was hardcoded; `os.environ.get("ISR_SIM_SCENARIO")` was never called.  
**Fix:** `SIM_SCENARIO = os.environ.get("ISR_SIM_SCENARIO") or None`

---

### BUG-02 — Scenario PID override lost due to async race (High)
**File:** `isr_lidar_mpc.py` → `avoidance_loop()`  
**Symptom:** `scenario_pid_override` was always `None` when popped at loop entry because `lidar_sim_reader` (a concurrent coroutine) hadn't run yet.  
**Root cause:** `avoidance_state.pop("scenario_pid_override", None)` executed at `avoidance_loop` startup, before `lidar_sim_reader` had a chance to write the value.  
**Fix:** Moved the pop inside the main `while` loop with a `_override_applied` guard — consumed lazily on the first tick where it's present.

---

### BUG-03 — `mission_phase` never written to GCS `data` dict (Critical)
**File:** `telemetry_web.py` → `lidar_update()` endpoint  
**Symptom:** GCS phase panel stayed frozen at `LOITER` for all secondary orbits (SEC-1/2/3). Target queue never advanced past primary target. Phase list never showed SEC phases.  
**Root cause:** `isr_lidar_mpc.push_to_gcs()` sends `"mission_phase"` in the POST body, but `lidar_update()` only read `groundspeed`, `gps_ok`, `reconnects`, `eta_seconds` from the payload — `mission_phase` was silently dropped.  
**Fix:** Added `if "mission_phase" in payload: data["mission_phase"] = payload["mission_phase"]`

---

### BUG-04 — `wp_current` / `wp_total` not updated from mission push (Medium)
**File:** `telemetry_web.py` → `lidar_update()` endpoint  
**Symptom:** Waypoint progress bar on GCS stopped updating during orbit/RTL phases because the MAVSDK `mission_progress` stream stops emitting once the survey completes.  
**Root cause:** `push_to_gcs()` sends `wp_current` and `wp_total` in every POST, but `lidar_update()` never wrote them into `data[]`.  
**Fix:** Added `if "wp_current" in payload: data["wp_current"] = payload["wp_current"]` and `wp_total` equivalent.

---

### BUG-05 — `predict_trajectory()` used stale `_u_prev` for horizon (Low)
**File:** `mpc_controller.py` → `MPCEngine.predict_trajectory()`  
**Symptom:** Predicted trajectory visualisation showed a path inconsistent with the actual MPC solve — all steps k>0 used the previous solve's first action instead of propagating the current optimal control.  
**Root cause:** `U = np.tile(self._u_prev, (self.N, 1)); U[0] = u_opt` — only the first row was from the current solve; rows 1..N used the previous `_u_prev`.  
**Fix:** `U = np.tile(u_opt, (self.N, 1))` — tiles the current optimal action across the full horizon for a consistent predicted path.

---

### BUG-06 — `banner()` padding wrong when message has spaces (Low)
**File:** `launch.sh` → `banner()`  
**Symptom:** Banner box lines misaligned for multi-word messages (e.g. `STEP 3 — Starting PX4 SITL`).  
**Root cause:** `${#1}` measures the length of only the **first word** of `$*`, so the right-padding was too wide by `len(remaining_words)` characters.  
**Fix:** Captured `local msg="$*"` and used `${#msg}` for correct total-message-length padding.

---

### Previously Fixed (v10–v12, documented in source)

| ID | File | Fix |
|----|------|-----|
| v11-A | `pid_controller.py` | `_Tt` recomputed on kp-only gain update |
| v11-B | `isr_lidar_mpc.py` | `sectors[]` added to GCS POST payload |
| v11-C | `isr_lidar_mpc.py` | TOCTOU race on `avoidance_state["last_wp"]` |
| v11-D | `isr_lidar_mpc.py` | Climb escape: early `continue` guards horizontal goto spam |
| v11-E | `isr_lidar_mpc.py` | `_compute_eta()` home-WP index offset |
| v11-F | `isr_lidar_mpc.py` | `telemetry_tracker()` independent retry loops |
| v11-G | `isr_lidar_mpc.py` | Bearing frame: sensor→world conversion in `_bearing_to_nearest()` + `_compute_sectors()` |
| v10-A | `isr_lidar_mpc.py` | `avoidance_count` increments once per event, not per tick |
| v10-B | `isr_lidar_mpc.py` | ETA uses live groundspeed, not constant SPEED |
| v10-C | `pid_controller.py` | Back-calculation anti-windup: removed spurious `* dt` |
| v10-D | `pid_controller.py` | `best_escape_bearing()`: removed double-add of drone heading |
| v10-E | `pid_controller.py` | `compute_avoidance_waypoint()`: removed spurious +90° rotation |
| v13-A | `telemetry_web.py` | Jinja2 `| safe` on all three server-injected JSON blobs |
| v13-B | `telemetry_web.py` | NFZ / targets buttons: added `active` CSS class at init |
| v13-C | `telemetry_web.py` | `nearest_bearing` inf/nan guard in `emit_loop` |
| v13-D | `telemetry_web.py` | `scenario_list()`: `abspath(__file__)` to fix relative-path CWD |
| v13-E | `isr_lidar_mpc.py` | NFZ fence check: per-zone haversine instead of global closest |
| v13-F | `isr_lidar_mpc.py` | Scenario PID override written to shared state, not local variable |

---

## 7. Configuration Reference

All mission constants live in `mission_config.py` — edit once, applies everywhere.

| Constant | Default | Description |
|----------|---------|-------------|
| `HOME_LAT / HOME_LON` | 47.3977, 8.5456 | Launch / RTL position |
| `ALTITUDE` | 50.0 m | Survey cruise altitude AGL |
| `SPEED` | 50.0 m/s | Mission speed |
| `ROWS` | 4 | Survey grid rows |
| `ROW_SPACING` | 0.0003° | Spacing between rows (~33m) |
| `ROW_WIDTH` | 0.0006° | Row length (~47m) |
| `TARGET_LAT / LON` | 47.3985, 8.5470 | Primary ISR target |
| `ORBIT_RADIUS` | 45 m | Primary orbit radius |
| `ORBIT_ALTITUDE` | 90 m AGL | Primary orbit altitude |
| `ORBIT_DURATION` | 30 s | Primary orbit dwell time |
| `ALTITUDE_STEP` | 15 m | Multi-altitude sweep step |
| `GRID_ALTITUDE_STEPS` | 1 | 1=single pass, 2=stereo |

LiDAR constants are local to `isr_lidar_mpc.py`:

| Constant | Default | Description |
|----------|---------|-------------|
| `LIDAR_WARN_DIST` | 25 m | Warning zone threshold |
| `LIDAR_AVOID_DIST` | 15 m | Avoidance trigger distance |
| `AVOIDANCE_OFFSET_M` | 50 m | Minimum detour waypoint distance |
| `DEBOUNCE_COUNT` | 3 | Consecutive hits before avoidance activates |
| `AVOIDANCE_TIMEOUT_S` | 10 s | Time before climb escalation |
| `CLIMB_ESCAPE_M` | 15 m | Altitude gain on timeout |

---

## 8. Scenario System

Scenarios are defined in `scenarios.json`. Each scenario has named timed events that inject obstacles at specified bearings and distances.

```json
{
  "name": "urban_canyon",
  "description": "...",
  "pid_gains": { "kp": 2.2, "ki": 0.0, "kd": 0.8 },
  "events": [
    { "start_s": 3.0, "duration_s": 12.0, "bearing_deg": 45.0, "dist_m": 10.0 }
  ]
}
```

**Activate via launch script:**
```bash
./launch.sh --scenario urban_canyon
```

**Activate manually:**
```bash
ISR_SIM_SCENARIO=urban_canyon python3 isr_lidar_mpc.py
```

**List all scenarios:**
```
GET http://localhost:5000/scenario_list
```

---

## 9. GCS Dashboard

Served at **http://localhost:5000**

### REST Endpoints

| Method | Path | Description |
|--------|------|-------------|
| `GET` | `/` | Main dashboard HTML |
| `POST` | `/lidar_update` | Push LiDAR + mission state (from mission script) |
| `POST` | `/pid_tune` | Live gain update: `{"controller":"avoidance","kp":2.0,...}` |
| `GET` | `/pid_gains` | Current PID/MPC gains |
| `GET` | `/download_log` | CSV flight log download |
| `GET` | `/scenario_list` | All scenario names + event counts |
| `GET` | `/nfz_status` | Live NFZ proximity for current drone position |

### Map Layers (toggleable)
- **Survey grid** — boustrophedon waypoints (green dashed)
- **Drone trail** — last 150 position fixes (cyan)
- **Obstacle ray** — bearing + distance to nearest obstacle
- **Sector overlay** — 8-sector LiDAR clearance map
- **Detour waypoint** — current avoidance target (amber pulse)
- **NFZ circles** — red exclusion zones with labels
- **Secondary targets** — cyan diamonds with orbit rings
- **Loiter waypoints** — camera surveillance hold markers

---

## 10. MPC Controller

### MPCEngine
Finite-horizon QP solver using `scipy.optimize.minimize` (L-BFGS-B).

**Cost function:**
```
J = Σ_k [ Q_track * ||pos_err||² + Q_vel * ||vel_err||² + obs_penalty ]
  + Σ_k [ R_input * ||u[k]||² + R_delta * ||Δu[k]||² ]
  + Q_terminal * ||pos_err(N)||²
```

**Speed scheduling (AvoidanceMPC):**

| Speed | Q_track | W_obs | N | Description |
|-------|---------|-------|---|-------------|
| < 20 m/s | 1.5 | 80 | 10 | Soft gains, hover/approach |
| ≥ 20 m/s | 2.8 | 200 | 8 | Aggressive, 50 m/s cruise |

### API Compatibility
All MPC wrappers expose the same interface as their PID predecessors:
```python
avoid_mpc.update_speed(groundspeed)
offset_m = avoid_mpc.compute_correction(nearest_dist)
avoid_mpc.set_gains(kp=..., ki=..., kd=...)  # stub — logs, does not map to weights
```

---

## 11. PID Controller (Legacy)

`pid_controller.py` is retained for reference. Contains three controllers:
- `AvoidancePID` — gain-scheduled Kp/Kd, back-calculation anti-windup
- `OrbitPID` — radial error → speed correction
- `AltitudePID` — altitude error → climb rate

Import these from `pid_controller` if you need to revert to PID avoidance.

---

## 12. Dependencies

| Package | Version | Role |
|---------|---------|------|
| `mavsdk` | ≥1.4 | PX4 / MAVLink autopilot interface |
| `flask` | ≥3.0 | GCS web server |
| `flask-socketio` | ≥5.3 | Real-time telemetry push |
| `requests` | ≥2.31 | Mission → GCS HTTP push |
| `numpy` | ≥1.26 | MPC matrix ops |
| `scipy` | ≥1.12 | L-BFGS-B optimiser |
| `gz-transport13` | optional | Real LiDAR from Gazebo (apt package) |
| `gz-msgs10` | optional | Protobuf LaserScan messages |

Install optional Gazebo Python bindings:
```bash
sudo apt install python3-gz-transport13 python3-gz-msgs10
```

---

## 13. Troubleshooting

**Mission refuses to arm — "No valid mission available"**  
→ PX4 needs `start_mission()` called before `arm()`. This is handled automatically. If it still fails, check PX4 log for GPS lock.

**Scenario has no effect**  
→ Confirm `ISR_SIM_SCENARIO` is set in the process environment (not just exported in the shell before launch). The launch script injects it via `env ISR_SIM_SCENARIO=... python3 isr_lidar_mpc.py`.

**GCS phase panel stuck at LOITER**  
→ This was BUG-03 (fixed). Confirm you are running the patched `telemetry_web.py`.

**Sector overlay frozen on GCS**  
→ Was fixed in v11 (sectors[] missing from POST payload) and v13 (sectors never read from POST in `lidar_update`). Both fixes are in this release.

**NFZ labels misidentified in pre-flight log**  
→ Was fixed in v13 (per-zone haversine vs global `get_nfz_exclusion_check`). Each zone now logs its own distance correctly.

**banner() box misaligned in terminal**  
→ Was BUG-06 (fixed). `${#1}` → `${#msg}` with `local msg="$*"`.

**`nearest_bearing` OverflowError crashing emit_loop**  
→ Was fixed in v13. `isfinite()` guard added before `round()`.
