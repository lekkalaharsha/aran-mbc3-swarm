# S02 — Radar Fusion (ROS2 Package)

**Branch:** `feature/6-panel-radar-coverage`  
**Parent:** `main`  
**Status:** ✅ Complete — 40/40 tests pass

---

## Overview

ROS2 ament_python package. Two-node pipeline:  
`detection_node` (per drone) → `fusion_node` (leader only)

```
/radar_A-F/scan/points  (PointCloud2, gz→ROS2 bridge)
        ↓
detection_node  — filter → cluster → RF gate → TF → /drone_X/radar/targets
        ↓
fusion_node     — spatial merge → Kalman predict/update → /swarm/tracks + /swarm/asp
```

---

## File Map

| File | Purpose |
|------|---------|
| `radar_fusion/radar_fusion/detection_node.py` | Per-drone: 6-panel subscribe, cluster, RF gate, TF |
| `radar_fusion/radar_fusion/rf_classifier.py` | Layer 2: Random Forest (synthetic training) |
| `radar_fusion/radar_fusion/kalman_tracker.py` | 6-state Kalman (px py pz vx vy vz), NN assoc |
| `radar_fusion/radar_fusion/fusion_node.py` | Leader: 5-drone fusion → ASP + situation |
| `radar_fusion/config/radar_fusion.yaml` | All tunable params |
| `radar_fusion/launch/radar_fusion.launch.py` | `mode:=single` or `mode:=swarm` |
| `radar_fusion/test_unit.py` | Standalone tests (T1–T11, no ROS needed) |
| `radar_fusion/package.xml` | ROS2 deps |
| `radar_fusion/setup.py` | Entry points + install_requires |

---

## Panel Layout (matches mbc3_radar_drone)

| Panel | Yaw (body CCW) | PANEL_ROT | Detection topic |
|-------|---------------|-----------|----------------|
| A | 0° | Identity | `/radar_A/scan/points` |
| B | 60° | R_z(60°) | `/radar_B/scan/points` |
| C | 120° | R_z(120°) | `/radar_C/scan/points` |
| D | 180° | R_z(180°) | `/radar_D/scan/points` |
| E | 240° | R_z(240°) | `/radar_E/scan/points` |
| F | 300° | R_z(300°) | `/radar_F/scan/points` |

---

## Key Params (radar_fusion.yaml)

| Param | Value | Note |
|-------|-------|------|
| cluster_radius | 2.5 m | greedy clustering merge distance |
| min_cluster_hits | 3 | minimum points to form target |
| min_range | 20 m | software gate (avoids self-detection) |
| max_range | 5000 m | AWR1843 max |
| el_gate_deg | 25° | max elevation (filters zenith) |
| el_min_deg | -5° | min elevation (filters ground) |
| gate_m | 10 m | Kalman NN association gate |
| track_ttl_s | 3.0 s | prune stale tracks |
| merge_dist_m | 5.0 m | multi-drone same-target collapse |
| intercept_range_m | 500 m | INTERCEPT decision threshold |

---

## Published Topics

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/{ns}/radar/targets` | String JSON | 5 Hz | Per-drone confirmed targets |
| `/{ns}/radar/detections` | MarkerArray | 5 Hz | RViz markers per drone |
| `/swarm/tracks` | String JSON | 2 Hz | Fused track list with velocity |
| `/swarm/asp` | MarkerArray | 2 Hz | Air Situation Picture (RViz) |
| `/swarm/situation` | String JSON | 2 Hz | Tactical picture for LLM |

---

## Bugs Fixed (2026-05-19)

| ID | Severity | Fix |
|----|----------|-----|
| BUG-RF-1 | 🔴 | `makedirs('')` crash on flat model_path |
| BUG-RF-2 | 🔴 | `linalg.inv(S)` → `linalg.solve` |
| BUG-RF-3 | 🟠 | filter `tf_ok=False` targets before fusion |
| BUG-RF-4 | 🟠 | `time.time()` → `self.get_clock().now()` |
| BUG-RF-5 | 🟡 | `/5.0` → `/len(self._drone_ids)` |
| BUG-RF-6 | 🟡 | `_points` threading.Lock |
| BUG-RF-7 | 🟡 | `_raw` threading.Lock |
| BUG-RF-8 | 🟠 | `setup.py` missing scikit-learn, joblib |
| ISSUE-RF-1 | ⚪ | Q dt-scaled |
| ISSUE-RF-2 | ⚪ | RF training gap at hits=8 |

---

## Test Suite

```bash
python3 radar_fusion/test_unit.py
# Must pass: 40/40
# No ROS2 required — ROS2 deps mocked via unittest.mock
```

---

## Launch

```bash
# Single drone (no swarm)
ros2 launch radar_fusion radar_fusion.launch.py mode:=single use_sim_time:=true

# Full swarm (5 drones + fusion node)
ros2 launch radar_fusion radar_fusion.launch.py mode:=swarm use_sim_time:=true
```

---

## Open Tasks

- [ ] Integrate with Phase 2 ASP GCS web display
- [ ] Test with live Gazebo radar topics (requires ros_gz_bridge)
- [ ] Add CFAR pre-filter (Layer 1 — currently approximated by clustering)
