# Aran Technologies — Session Summary
## ISR Mission + LiDAR MPC Avoidance v12-MPC-v5

> Nirmaan Incubation — IIT Hyderabad Demo Build

---

## Project Overview

Full autonomous ISR (Intelligence, Surveillance, Reconnaissance) drone stack.  
**Stack:** PX4 SITL + Gazebo Harmonic + MAVSDK Python + Flask/SocketIO GCS

```
[PX4 SITL + Gazebo]
       │ MAVLink UDP:14540
       ▼
[isr_lidar_mpc.py]  ◄── 360° LiDAR (gz-transport or sim)
       │                 AvoidanceMPC / OrbitMPC / AltitudeMPC
       │ POST /lidar_update (5 Hz)
       ▼
[telemetry_web.py]  ──── MAVSDK telemetry streams
       │ SocketIO (2.5 Hz)
       ▼
[Browser GCS v13]   ──── Leaflet map, sector overlay, target panel
```

---

## File Structure

```
Aran_Technologies_v1/
├── isr_lidar_mpc.py      Main mission: Survey → Avoidance → Orbit → RTL
├── mpc_controller.py     MPCEngine + AvoidanceMPC / OrbitMPC / AltitudeMPC
├── pid_controller.py     Legacy PID stack (reference / fallback)
├── mission_config.py     Single config: coords, grid, NFZ, targets, racing params
├── telemetry_web.py      Flask/SocketIO GCS dashboard (v13)
├── mapping_3d.py         3D voxel map builder (PointCloudAccumulator + VoxelGrid)
├── scenarios.json        24 named LiDAR sim scenarios
├── launch.sh             Orchestration: SITL → GCS → Mission
├── testcode.py           Dev scratch / test harness
├── architecture.drawio   System architecture diagram
├── bugs.md               Full bug register (this session)
├── section_summarize.md  This file
└── logs/                 Timestamped run logs (auto-created)
```

---

## Mission Phases

| Phase | Label | Description |
|-------|-------|-------------|
| 1 | `STANDBY` | Pre-arm, NFZ fence check, mission upload (8 retries) |
| 2 | `SURVEY` | Boustrophedon grid, 360° LiDAR avoidance active at 50 Hz |
| 3 | `LOITER` | Primary target orbit (50 m radius, 30 m AGL) |
| 4.1–4.N | `SEC-1/2/3` | Secondary ISR targets sorted by priority |
| 5 | `RTL` | 3D map save → Return-to-launch → land |

---

## MPC Controller Summary

| Controller | Class | Purpose |
|-----------|-------|---------|
| Obstacle avoidance | `AvoidanceMPC` | Lateral detour waypoint magnitude |
| Orbit radius hold | `OrbitMPC` | Radial error → acceleration cmd |
| Altitude hold | `AltitudeMPC` | Vertical acceleration cmd |
| Core engine | `MPCEngine` | L-BFGS-B finite-horizon QP (scipy) |

**State:** `x = [n, e, d, vn, ve, vd]` NED  
**Input:** `u = [an, ae, ad]` m/s²  
**AvoidanceMPC speed tiers:**

| Speed | N | W_obs | Description |
|-------|---|-------|-------------|
| < 25 m/s | 12 | 80 | Standard ISR |
| 25–45 m/s | 6 | 500 | Racing |
| ≥ 45 m/s | 4 | 1000 | Ultra-aggressive |

---

## Work Done This Session

### 1. Code Review + Bug Audit
Reviewed all 8 source files. Found 11 bugs total across two passes.

### 2. Bug Fixes — Batch 1 (commit `8c811e9`)
Six pre-existing bugs fixed:

| ID | Severity | File | Fix |
|----|----------|------|-----|
| BUG-A | Critical | `telemetry_web.py` | Phase race: MAVSDK stream overwrote SEC-1/2/3. Added `_phase_state` timestamp guard in `_mode()`. |
| BUG-B | High | `isr_lidar_mpc.py` | `MapBuilder` never instantiated. Wired ingest into both lidar readers; map_stats pushed to GCS; PCD saved at RTL. |
| BUG-C | High | `mission_config.py` | `RACING_MODE = True` hardcoded. Now reads `os.environ`. `./launch.sh` env injection works. |
| BUG-D | Medium | `mission_config.py` | Wrong NFZ reported in breach alert. Fixed by tracking `breaching_dist` separately from global `closest_dist`. |
| BUG-E | Medium | `mapping_3d.py` | `MapBuilder.ingest()` mutated accumulator without lock. Now holds `accumulator._lock` explicitly. |
| BUG-F | Low | `isr_lidar_mpc.py` | Final banner showed `v12-MPC-v4`. Corrected to `v12-MPC-v5`. |

### 3. .gitignore + Repo Cleanup (commit `8c811e9`)
- Added `.gitignore` covering `__pycache__/`, `*.pyc`, `logs/`, `map_output/`, `*.pcd`, `*.log`, `.env`, `.claude/`, Zone.Identifier files.
- Untracked 17 existing `*:Zone.Identifier` files from git index.

### 4. Dynamic Mission Control Feature (branch `feature/dynamic-mission-control`, commit `5ab4597`, merged `4ca6f70`)
Four new GCS endpoints enabling runtime mission modification without restart:

| Endpoint | What it does |
|----------|-------------|
| `POST /add_nfz` | Add no-fly zone mid-flight |
| `POST /add_target` | Queue ISR target for orbit |
| `POST /config_update` | Patch `LIDAR_WARN_DIST`, `LIDAR_AVOID_DIST`, `AVOIDANCE_OFFSET`, `SAFE_RESUME_DIST` live |
| `POST /inject_event` | Inject timed obstacle into sim LiDAR reader |

**Command channel architecture:** GCS queues commands in `dynamic_commands{}` → drained atomically on each `POST /lidar_update` → returned in JSON response body → mission script's `_apply_dynamic_commands()` applies within 0.2s. No extra HTTP server or port required.

### 5. Bug Audit — Batch 2 (found in new feature, not yet fixed)
Five bugs found in the dynamic mission control code. See `bugs.md` for full details.

| ID | Severity | Status | Problem |
|----|----------|--------|---------|
| BUG-1 | Critical | 🔴 OPEN | New NFZ not shown on GCS map (process isolation) |
| BUG-2 | Critical | 🔴 OPEN | New target not shown on GCS map (process isolation) |
| BUG-3 | High | 🔴 OPEN | Mid-loop targets never visited (frozen snapshot) |
| BUG-4 | Medium | 🔴 OPEN | `threading.Lock` blocks asyncio event loop |
| BUG-5 | Low | 🔴 OPEN | `inject_event` bearing frame undocumented |

---

## Git History

```
4ca6f70  merge: feature/dynamic-mission-control → main
5ab4597  feat: dynamic mission control — NFZ/target/config/event injection
8c811e9  fix: resolve 6 bugs across mission, GCS, and mapping stack
680fd3b  current cahges
```

---

## Key Configuration (`mission_config.py`)

| Constant | Value | Note |
|----------|-------|------|
| `HOME_LAT / HOME_LON` | 47.3977, 8.5456 | Zurich SITL origin |
| `ALTITUDE` | 30 m AGL | Survey cruise |
| `SPEED` | 40 m/s | Racing cruise |
| `ORBIT_RADIUS` | 50 m | Primary orbit |
| `ORBIT_SPEED` | 12 m/s | Safe centripetal at 50 m |
| `RACING_MODE` | env `RACING_MODE` | Default `1` (on) |
| `LIDAR_AVOID_DIST` | 25 m (racing) | Avoidance trigger |
| `LIDAR_WARN_DIST` | 40 m (racing) | Warning zone |
| `AVOIDANCE_OFFSET_M` | 80 m (racing) | Detour distance |

---

## Next Actions (priority order)

1. **Fix BUG-1 + BUG-2** — append to GCS-process lists in `add_nfz()` / `add_target()` (2 lines each)
2. **Fix BUG-3** — replace frozen `sorted_secondaries` snapshot with live loop
3. **Fix BUG-4** — replace `threading.Lock` in async context with GIL-atomic list copy
4. **Fix BUG-5** — document bearing frame; optionally add `frame: "sensor"|"world"` param
5. **Test end-to-end** — `./launch.sh --scenario iit_panel_demo` with GCS open
6. **IIT panel demo prep** — confirm `iit_panel_demo` scenario runs cleanly in headless mode

---

## GCS REST API Reference

| Method | Path | Description |
|--------|------|-------------|
| `GET` | `/` | GCS dashboard |
| `POST` | `/lidar_update` | Mission script → GCS push (returns queued commands) |
| `POST` | `/pid_tune` | Live gain update |
| `GET` | `/pid_gains` | Current gains |
| `GET` | `/download_log` | CSV flight log |
| `GET` | `/scenario_list` | All scenario names |
| `GET` | `/nfz_status` | Live NFZ proximity |
| `GET` | `/map_stats` | 3D voxel map statistics |
| `GET` | `/map_slice` | GeoJSON slice at drone altitude |
| `POST` | `/add_nfz` | *(new)* Add NFZ mid-flight |
| `POST` | `/add_target` | *(new)* Queue ISR target |
| `POST` | `/config_update` | *(new)* Patch live config |
| `POST` | `/inject_event` | *(new)* Inject sim LiDAR obstacle |
