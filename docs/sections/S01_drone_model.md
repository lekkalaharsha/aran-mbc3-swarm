# S01 — Drone Model (mbc3_radar_drone)

**Branch:** `feature/drone-visualization`  
**Parent:** `feature/6-panel-radar-coverage`  
**Status:** 🔄 In progress — landing gear spawn fix

---

## Overview

MBC-3 hexacopter with 6× FMCW radar panels. Custom PX4 SITL model.  
AUW: 3.834 kg | 6S 10 Ah | ~32 min endurance | 6-motor redundancy

---

## File Map

| File | Purpose |
|------|---------|
| `new_drone/mbc3_radar_drone.sdf` | **Source of truth** — edit this |
| `new_drone/model.config` | Gazebo model manifest (SDF version 1.11) |
| `new_drone/install_px4_model.sh` | Installs SDF → PX4 models dir |
| `new_drone/airframe/4601_gz_mbc3_radar_drone` | PX4 flight params |
| `new_drone/mbc3_radar_drone.xacro` | XACRO source (documentation only) |

---

## Install Workflow

```bash
# After any SDF edit:
bash new_drone/install_px4_model.sh
# Copies: mbc3_radar_drone.sdf → model.sdf → ~/PX4-Autopilot/Tools/simulation/gz/models/mbc3_radar_drone/
```

---

## Motor Layout (FRD, arm_len=0.360 m)

| Motor | Arm | Angle | Spin | CA_ROTOR_PX | CA_ROTOR_PY |
|-------|-----|-------|------|-------------|-------------|
| 0 | 0 | 0° | CCW | +0.360 | 0.000 |
| 1 | 1 | 60° | CW | +0.180 | -0.312 |
| 2 | 2 | 120° | CCW | -0.180 | -0.312 |
| 3 | 3 | 180° | CW | -0.360 | 0.000 |
| 4 | 4 | 240° | CCW | -0.180 | +0.312 |
| 5 | 5 | 300° | CW | +0.180 | +0.312 |

---

## Key Physics Params

| Param | Value | How to change |
|-------|-------|---------------|
| motorConstant | 1.740e-5 N·s²/rad² | Edit SDF MulticopterMotorModel |
| maxRotVelocity | 838 rad/s | Edit SDF — recalc MPC_THR_HOVER |
| MPC_THR_HOVER | 0.72 | = sqrt(mass×9.81/6 / motorConstant) / maxRotVelocity |
| Spawn pose | z=0.135 m | `<pose>` in SDF model element |
| Mass | 3.834 kg | From SDF base_link inertial |

**MPC_THR_HOVER recalc:**
```
hover_F     = 3.834 × 9.81 / 6 = 6.26 N
hover_omega = sqrt(6.26 / 1.74e-5) = 600 rad/s
MPC_THR_HOVER = 600 / 838 = 0.716 → set 0.72
```

---

## Radar Panel Layout

| Panel | Yaw offset | Coverage |
|-------|-----------|---------|
| A | 0° (forward) | ±30° H |
| B | 60° | ±30° H |
| C | 120° | ±30° H |
| D | 180° (rear) | ±30° H |
| E | 240° | ±30° H |
| F | 300° | ±30° H |

6 × 60° = seamless 360°, zero gaps.  
Topics: `/radar_A/scan` … `/radar_F/scan`  
Type: `gpu_lidar` + `<lidar>` block (Gazebo Harmonic)

---

## SDF Fixes Applied (2026-05-19)

| Fix | Description |
|-----|-------------|
| FIX-01 | LiftDrag → `motor_bell_N` (not prop_N frame) |
| FIX-02 | gpu_lidar `<ray>` → `<lidar>` block |
| FIX-03 | Rangefinder `type="ray"` → `type="gpu_lidar"` |
| FIX-04 | LiftDrag `<forward>` hardcoded `1 0 0` all rotors |
| FIX-05 | `maxRotVelocity` 1000 → 838 rad/s |
| FIX-06 | motor_3 pose Y jitter (4.41e-17 → 0) |
| FIX-07 | Radar min range 2m → 0.5m |
| FIX-08 | Radar noise 0.8m → 0.1m |
| FIX-09 | PosePublisher 50Hz → 100Hz |
| FIX-10 | odom_frame namespaced |
| FIX-11 | `visualize=false` rangefinder + optical flow |
| FIX-12 | Baro stddev 0.1 → 0.2m |
| FIX-13 | motor_4 pose Y jitter cleaned |
| FIX-LG | Spawn pose `z=0.135` — landing gear on ground (IN PROGRESS) |

---

## Open Tasks

- [ ] Verify full visuals via `gz sim mbc3_radar_drone.sdf` (standalone)
- [ ] Confirm landing gear touching ground (FIX-LG)
- [ ] Generate full-visual SDF from XACRO and compare
- [ ] Merge → `feature/6-panel-radar-coverage` after visual confirmed
