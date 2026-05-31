# S05 — PX4 SITL + Launch Orchestration

**Status:** ✅ Operational | Phase 1 flight test in progress

---

## File Map

| File | Purpose |
|------|---------|
| `launch.sh` | Full stack orchestration (all steps) |
| `new_drone/install_px4_model.sh` | Install SDF + airframe into PX4 |
| `new_drone/airframe/4601_gz_mbc3_radar_drone` | PX4 airframe params |

---

## Launch Sequence

```
STEP 1  Dependency checks (python3, make, gz, packages)
STEP 2  Verify mission files (syntax, imports, scenario)
STEP 3  PX4 SITL + Gazebo (make px4_sitl gz_mbc3_radar_drone)
STEP 3.5 Radar fusion (ros_gz_bridge + detection + fusion nodes) — optional
STEP 4  GCS dashboard (telemetry_web.py)
STEP 5  ISR mission (isr_lidar_mpc.py)
STEP 6  Live status monitor (PX4/GCS/Mission/Radar health)
```

---

## Common Launch Commands

```bash
# Full stack
./launch.sh

# SITL + Gazebo only (no GCS, no mission)
./launch.sh --sim-only

# With specific scenario
./launch.sh --scenario mbc3_iaf_demo

# Headless (no Gazebo GUI)
./launch.sh --headless --scenario mbc3_iaf_demo

# Skip radar fusion
./launch.sh --no-radar

# Build PX4 only (no launch)
./launch.sh --build-only
```

---

## Flags

| Flag | Effect |
|------|--------|
| `--sim-only` | Start PX4+Gazebo, skip GCS and mission |
| `--gcs-only` | Start GCS only (SITL already running) |
| `--headless` | No Gazebo GUI (server mode) |
| `--no-radar` | Skip radar fusion nodes |
| `--radar-mode single\|swarm` | Single or 5-drone detection |
| `--build-only` | Compile PX4 only |
| `--clean-logs` | Delete old logs before start |
| `--scenario NAME` | Inject ISR_SIM_SCENARIO env |
| `--px4-dir PATH` | Override PX4 directory |

---

## Env Overrides

| Var | Default | Effect |
|-----|---------|--------|
| `PX4_DIR` | `~/PX4-Autopilot` | PX4 path |
| `PX4_MAKE_MODEL` | `gz_mbc3_radar_drone` | Gazebo model |
| `RACING_MODE` | `1` | Enable racing avoidance params |
| `ROS2_WS` | `~/ros2_ws` | ROS2 workspace for radar |
| `RADAR_MODE` | `single` | `single` or `swarm` |
| `ROS2_DISTRO` | auto | ROS2 distro name |
| `GCS_TOKEN` | `` (unset) | When set: enforces `X-GCS-Token` header on all GCS mutation endpoints (`/add_nfz`, `/add_target`, `/pid_tune`, `/config_update`, `/inject_event`) |

---

## PX4 Airframe Params (4601_gz_mbc3_radar_drone)

| Param | Value | Note |
|-------|-------|------|
| MPC_THR_HOVER | 0.72 | Hover at 72% throttle |
| MPC_THR_MIN | 0.06 | Minimum throttle |
| MPC_THR_MAX | 0.95 | Maximum throttle |
| MPC_XY_VEL_MAX | 15.0 m/s | Horizontal speed limit |
| EKF2_AID_MASK | 1 | GPS fusion |
| BAT1_N_CELLS | 6 | 6S battery |
| BAT1_CAPACITY | 10000 mAh | |

---

## Common Errors + Fixes

| Error | Fix |
|-------|-----|
| `PX4 server already running for instance 0` | `pkill -9 -f px4; pkill -9 -f "gz sim"; sleep 2` |
| `Permission denied: ./launch.sh` | `chmod +x launch.sh` |
| `mbc3_radar_drone model not installed` | `bash new_drone/install_px4_model.sh` |
| `Socket closed` in MAVSDK | PX4 crashed — check `tail -20 logs/*/px4.log` |
| Drone sinks into ground | Check `<pose>` in SDF model element |

---

## Process Group Safety (`set -m`)

`launch.sh` uses `set -m` (bash job control). This assigns each background job its own process group (PGID == PID). The `cleanup()` trap sends `kill -SIGTERM -- "-${pid}"` (negative PID = kill entire process group) so PX4 grandchildren, Gazebo render threads, and Python subprocesses all terminate cleanly rather than becoming orphans.

---

## Open Tasks

- [x] Process group kill — cleanup() now kills full subtree
- [ ] Phase I flight test — presentations New Delhi, 13–24 July 2026
- [ ] Verify `isr_lidar_mpc.py` runs on mbc3_radar_drone
