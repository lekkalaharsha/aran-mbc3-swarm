# Aran Technologies — Session Summary
## ISR Mission + LiDAR MPC Avoidance v12-MPC-v5
## Last updated: 2026-05-16

> Nirmaan Incubation — IIT Hyderabad Demo Build

---

## Project Overview

Full autonomous ISR drone stack.  
**Stack:** PX4 SITL + Gazebo Harmonic + MAVSDK Python + Flask/SocketIO GCS

```
[PX4 SITL + Gazebo Harmonic]
       │ MAVLink UDP:14540
       ▼
[isr_lidar_mpc.py]  ◄── 360° LiDAR gz_x500_lidar_2d
       │                 /world/default/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan
       │ POST /lidar_update (2.5 Hz)
       ▼
[telemetry_web.py]  ──── MAVSDK telemetry streams
       │ SocketIO
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
├── scenarios.json        24 named LiDAR sim scenarios (LF line endings)
├── launch.sh             Orchestration: SITL → GCS → Mission
├── bugs.md               Full bug register (20 bugs, 16 fixed, 4 in-branch)
├── section_summarize.md  This file
├── CODING_RULES.md       Branch policy, logic-change rules, merge checklist
├── .gitattributes        Enforces LF line endings on all text files
└── logs/ map_output/     Auto-created at runtime
```

---

## Current Git State (2026-05-16)

### Branch: `fix/approach-orbit-queue` (ACTIVE — not yet merged)
Contains FIX-1 through FIX-4. Needs verified exit-0 run then merge to main.

### Main branch log
```
5d34b0f  merge: fix/lidar-topic-discovery → main
d7cf336  fix: correct LiDAR topic for gz_x500_lidar_2d model
44db1a8  docs: add CODING_RULES.md
d29f048  fix: 4 runtime issues found during smoke test (NEW-1 through NEW-4)
6d6a758  fix: resolve 5 dynamic-mission-control bugs (BUG-1 through BUG-5)
4ca6f70  merge: feature/dynamic-mission-control → main
5ab4597  feat: dynamic mission control — NFZ/target/config/event injection
8c811e9  fix: resolve 6 bugs across mission, GCS, and mapping stack
```

---

## Mission Phases

| Phase | Label | Description |
|-------|-------|-------------|
| 1 | `STANDBY` | Pre-arm, NFZ fence check, mission upload (8 retries) |
| 2 | `SURVEY` | Boustrophedon grid, 360° LiDAR avoidance at 50 Hz |
| 3 | `LOITER` | Primary target orbit |
| 4.1–4.N | `SEC-1/2/3` | Secondary ISR targets sorted by priority (live re-evaluation) |
| 5 | `RTL` | 3D map save → Return-to-launch → land |

---

## Last Verified Run (2026-05-16 — commit 5d34b0f)

```
Scenario:          iit_panel_demo
LiDAR model:       gz_x500_lidar_2d
LiDAR scans:       4384  ✅
Avoidances:        2  ✅
Survey WPs:        11/11  ✅
Phase 3 PRIMARY:   orbit complete  ✅
Phase 4.1 ALPHA-2: orbit complete  ✅
Phase 4.2 BRAVO-1: orbit complete  ✅
Phase 4.3 CHARLIE: orbit complete (approach timeout hit)  ✅
3D map saved:      raw + voxel .pcd  ✅
Exit code:         0  ✅
```

**Known issue in this run:** All 3 secondary targets hit 120s approach timeout because
`goto_location` defaults to ~2 m/s in SITL. FIX-1+FIX-2 on branch `fix/approach-orbit-queue`
address this.

---

## MPC Controller Summary

| Controller | Class | Purpose |
|-----------|-------|---------|
| Obstacle avoidance | `AvoidanceMPC` | Lateral detour waypoint magnitude |
| Orbit radius hold | `OrbitMPC` | Radial error → acceleration cmd |
| Altitude hold | `AltitudeMPC` | Vertical acceleration cmd |
| Core engine | `MPCEngine` | L-BFGS-B finite-horizon QP (scipy) |

**AvoidanceMPC speed tiers:**

| Speed | N | W_obs | Description |
|-------|---|-------|-------------|
| < 25 m/s | 12 | 80 | Standard ISR |
| 25–45 m/s | 6 | 500 | Racing |
| ≥ 45 m/s | 4 | 1000 | Ultra-aggressive |

---

## Bug Summary

| Batch | Count | Status |
|-------|-------|--------|
| Batch 1 (code review) | 6 | All fixed — commit `8c811e9` |
| Batch 2 (dynamic mission) | 5 | All fixed — commit `6d6a758` |
| Batch 3 (smoke test) | 5 | All fixed — commits `d29f048`, `d7cf336` |
| Batch 4 (perf/quality) | 4 | In branch `fix/approach-orbit-queue` — NOT merged |
| **Total** | **20** | **16 fixed, 4 in-branch** |

See `bugs.md` for full details.

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

## LiDAR Configuration

| Setting | Value |
|---------|-------|
| PX4 model | `gz_x500_lidar_2d` (airframe 4013) |
| Topic | `/world/default/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan` |
| Override env var | `ISR_LIDAR_TOPIC` |
| Auto-discovery | `_discover_lidar_topic()` — runs `gz topic -l` if no scan in 8s |
| Sensor range | 0.1–30 m, 1080 rays, ±135° horizontal |
| Update rate | 30 Hz |
| Poll rate in code | 50 Hz (LIDAR_POLL_HZ) |

---

## GCS REST API

| Method | Path | Description |
|--------|------|-------------|
| `GET` | `/` | GCS dashboard |
| `POST` | `/lidar_update` | Mission → GCS push (returns queued commands) |
| `POST` | `/pid_tune` | Live gain update |
| `GET` | `/pid_gains` | Current gains |
| `GET` | `/download_log` | CSV flight log |
| `GET` | `/scenario_list` | All 24 scenario names |
| `GET` | `/nfz_status` | Live NFZ proximity |
| `GET` | `/map_slice` | Live 2D voxel GeoJSON |
| `GET` | `/map_stats` | 3D occupancy grid stats |
| `POST` | `/add_nfz` | Add NFZ mid-flight |
| `POST` | `/add_target` | Queue ISR target |
| `POST` | `/config_update` | Patch live avoidance config |
| `POST` | `/inject_event` | Inject sim LiDAR obstacle (`frame: sensor/world`) |

---

## Next Actions (priority order)

1. **Complete + merge `fix/approach-orbit-queue`** — run to verified exit 0, then merge
2. **Check approach speed actually improved** — confirm no timeouts with FIX-1 (`set_maximum_speed`)
3. **Check orbit cold-start fixed** — confirm radius starts near commanded value with FIX-2
4. **Update README.md** — section 6 (bug fixes) is out of date; add Batch 3 + 4
5. **IIT panel demo dry-run** — screen-record GCS at `http://localhost:5000`

---

## Launch Commands

```bash
# Standard demo launch
./launch.sh --headless --scenario iit_panel_demo

# With GUI (if display available)
./launch.sh --scenario iit_panel_demo

# LiDAR model override (default is gz_x500_lidar_2d)
PX4_MAKE_MODEL=gz_x500_lidar_2d ./launch.sh --headless

# GCS only (SITL already running)
./launch.sh --gcs-only

# Custom LiDAR topic (non-default world/vehicle)
ISR_LIDAR_TOPIC=/world/my_world/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan \
  ./launch.sh --headless
```
