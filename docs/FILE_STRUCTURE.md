# Project File Structure — Aran MBC-3

Root: `~/Documents/aran_mbc/`

---

## Root files

| File | Purpose |
|------|---------|
| `launch.sh` | **Single-drone demo launcher** — starts PX4 SITL, Gazebo, AERIS-10 driver, radar_fusion, GCS, then runs `isr_lidar_mpc.py` |
| `swarm_launch.sh` | **5-drone swarm launcher** — starts 5× PX4 SITL instances, swarm GCS, radar_sim, then `swarm_mission.py` |
| `fly_demo.sh` | Lightweight flight demo (no radar, no GCS) for quick airframe tests |
| `kill_drone.sh` | Kills all PX4/Gazebo/Python processes — emergency stop |
| `record_demo.sh` | Screen recorder for demo video — ffmpeg static binary, two terminal windows |
| `setup_ws.sh` | One-shot workspace setup — installs ROS2 deps, colcon builds, pip installs |
| `README.md` | Project overview, setup guide, architecture, phase 0 demo steps |
| `MBC3_MASTER.md` | Competition master doc — phase checklist, scoring rubric, team roles |
| `.gitignore` | Ignores `__pycache__`, `*.pyc`, `logs/`, `map_output/` |
| `.gitattributes` | Enforces LF line endings on all text files |

---

## `src/` — Python flight stack

| File | Purpose |
|------|---------|
| `swarm_mission.py` | **Main swarm entry point** — arms/climbs 5 drones, runs parallel sector missions, redistribution on failure, follow-target loop |
| `swarm_telemetry_web.py` | **Swarm GCS dashboard** (Flask/SocketIO, port 5000) — 5-drone command center, radar polar display, contact alerts, track command endpoint |
| `telemetry_web.py` | Single-drone GCS dashboard — used with `launch.sh`, v13 MPC display |
| `mission_config.py` | **Protected constants** — HOME_LAT/LON, ALTITUDE, SPEED, NFZ coords, orbit params. Do not edit without team sign-off |
| `mission_config_swarm.py` | Swarm-specific config — per-drone altitudes, sector WP generator, redistribution logic, DRONE_TARGET map |
| `d2d_node.py` | Drone-to-drone UDP multicast (224.1.1.1:14900) — heartbeat, bully election, RADAR track share, REASSIGN ACK, follow-target state |
| `radar_sim.py` | Gazebo pose → 6-panel FOV geometry → detection list → pushes to GCS at 5 Hz |
| `leader_election.py` | Standalone leader election HTTP service (pre-D2D, legacy) |
| `asp_bridge.py` | Bridges AERIS-10 ROS2 `/radar_*/scan/points` topics → GCS `/asp_update` HTTP |
| `asp_fake_targets.py` | Injects synthetic radar tracks into GCS for bench testing without Gazebo |
| `isr_lidar_mpc.py` | **Single-drone flight controller** — MPC position control, LIDAR avoidance, scenario execution, phase state machine |
| `isr_lidar_pid.py` | Legacy PID-based single-drone controller (superseded by MPC) |
| `mpc_controller.py` | L-BFGS-B finite-horizon MPC engine — speed-scheduled gains, 5-step horizon |
| `pid_controller.py` | PID controller used by `isr_lidar_pid.py` |
| `mapping_3d.py` | Point-cloud voxel mapper — aggregates LIDAR returns → `map_output/*.pcd` |
| `swarm_fly.py` | Simple swarm formation flyer (no redistribution, pre-phase 6) |
| `swarm_monitor.py` | Console monitor — streams drone_states from GCS API, no flight control |
| `mission_ai.py` | AI mission planner stub — generates WP sequences from target descriptions |
| `scenarios.json` | **Protected** — timed scenario event list for `isr_lidar_mpc.py` playback |
| `static/` | Local JS/CSS assets (no CDN needed on demo day) |
| `static/leaflet.js` | Leaflet map library (145 KB) |
| `static/leaflet.css` | Leaflet styles (15 KB) |
| `static/socket.io.min.js` | Socket.IO client (49 KB) |
| `static/chart.umd.min.js` | Chart.js (196 KB) |

---

## `aeris10_driver/` — ROS2 radar driver package

| File | Purpose |
|------|---------|
| `aeris10_driver/driver_node.py` | ROS2 node — reads AERIS-10 USB serial, publishes 6× `sensor_msgs/PointCloud2` topics (`/radar_A` … `/radar_F`) |
| `aeris10_driver/aeris10_usb.py` | Low-level USB/serial protocol parser for AERIS-10 FMCW radar |
| `config/aeris10_driver.yaml` | Driver params — port, baud, sim_mode flag, publish_hz |
| `launch/aeris10_driver.launch.py` | ROS2 launch file — starts `driver_node` with config |
| `package.xml` / `setup.py` / `setup.cfg` | ROS2 Python package metadata |

---

## `radar_fusion/` — ROS2 detection + tracking package

| File | Purpose |
|------|---------|
| `radar_fusion/detection_node.py` | ROS2 node — subscribes to 6 panel topics, clusters returns, publishes `/radar/targets` (`RadarTargetArray`) |
| `radar_fusion/fusion_node.py` | Multi-source fusion node — merges tracks from multiple drones (G4) |
| `radar_fusion/kalman_tracker.py` | Kalman filter tracker — maintains track state (pos/vel) across scans |
| `radar_fusion/rf_classifier.py` | RF signature classifier — distinguishes drone / vehicle / bird by RCS |
| `config/radar_fusion.yaml` | Fusion params — cluster radius, min hits, track timeout |
| `launch/radar_fusion.launch.py` | ROS2 launch file — starts detection + fusion nodes |
| `test_unit.py` | Unit tests for Kalman tracker and clustering logic |

---

## `new_drone/` — Drone model (SDF / URDF / CAD)

| File | Purpose |
|------|---------|
| `mbc3_radar_drone.sdf` | **Source of truth** — Gazebo SDF model: hexacopter + AERIS-10 + LIDAR + IMU |
| `mbc3_exact_v3.sdf` | Exact geometry v3 (aerodynamically tuned, used in Phase 6 SITL) |
| `mbc3_radar_drone.xacro` | URDF Xacro — used for RViz visualisation and URDF export |
| `mbc3_radar_drone.urdf` / `mbc3_radar_drone_full.urdf` | URDF exports from xacro |
| `mbc3_radar_drone.step` | Full assembly STEP file for CAD reference |
| `10in_Prop_CW.step` | 10-inch propeller STEP geometry |
| `model.sdf` / `model.config` | Gazebo model entry point (symlinked into PX4 models dir) |
| `model.dae` | Gazebo visual mesh (Collada) |
| `airframe/4601_gz_mbc3_radar_drone` | PX4 airframe mixer file — installed to `~/.local/share/px4/…` |
| `install_px4_model.sh` | Copies SDF + airframe into PX4 SITL model directory |
| `mbc3_redesign_v2.sdf` | Earlier redesign iteration (archived) |
| `mbc3_radar_drone_1.sdf` | Single-instance variant for single-drone SITL |

---

## `worlds/` — Gazebo world files

| File | Purpose |
|------|---------|
| `mbc3_radar_targets.sdf` | Static target world — fixed `radar_target_N` box models for radar testing |
| `mbc3_isr_targets.sdf` | ISR static world — targets + terrain for survey testing |
| `mbc3_radar_moving.sdf` | Moving target world — targets with `<plugin>` velocity actors |
| `mbc3_isr_moving.sdf` | Full ISR world with moving actors and terrain features |

---

## `docs/` — Documentation

| File | Purpose |
|------|---------|
| `CODING_RULES.md` | **Team rules** — branch policy, protected files, commit format, bug register |
| `bugs.md` | Bug register — all filed bugs with ID, severity, status, root cause |
| `architecture.drawio` | System architecture diagram (draw.io XML) |
| `d2d.md` | D2D protocol spec — message types, election algorithm, timing |
| `drone_analysis.md` | Drone performance analysis — thrust, hover power, endurance calcs |
| `migration_ubuntu_laptop.md` | Dev environment migration guide — Ubuntu 24.04 + ROS2 Jazzy setup |
| `session_handoff.md` | AI session handoff notes |
| `section_summarize.md` | Section content index for proposal writing |
| `FILE_STRUCTURE.md` | This file |
| `sections/S01_drone_model.md` | Drone model technical section (for proposal) |
| `sections/S02_radar_fusion.md` | Radar fusion section |
| `sections/S03_isr_mission.md` | ISR mission section |
| `sections/S04_gcs.md` | GCS section |
| `sections/S05_px4_launch.md` | PX4 launch section |
| `sections/S06_mbc3_competition.md` | Competition context section |
| `sections/S07_testing.md` | Testing section |
| `sdf_archive/` | Archived SDF model versions (pre-Phase 6) |

---

## `competition/` — MBC-3 submission documents

| File | Purpose |
|------|---------|
| `MBC3_Proposal.pdf` | Submitted competition proposal |
| `Final_Vision_Document_for_MBC_3_22Apr26.pdf` | Final vision document (April 2026) |
| `Registration_form_MBC_3_final.pdf` | Submitted registration form |
| `doc2_products_tech.md` | Products + tech section source (Markdown) |
| `doc3_competitions.md` | Competitions section source |
| `doc4_additional.md` | Additional section source |
| `generate_proposal_pdf.py` | Script to compile Markdown sections → PDF via pandoc |

---

## `tests/` — Test suite

| File | Purpose |
|------|---------|
| `test_d2d_protocol.py` | Unit tests for D2D heartbeat, election, ACK logic |
| `test_mission_config.py` | Unit tests for WP generation, NFZ boundary checks |
| `phase1_flight_test.py` | Integration test — arm + climb in SITL, checks altitude reached |
| `mock_sim.py` | Mock Gazebo + MAVSDK for offline unit testing |
| `testcode.py` | Scratch test file (not part of CI) |

---

## `tools/` — Dev utilities

| File | Purpose |
|------|---------|
| `pre_demo_check.sh` | **Pre-flight checklist** — 7 checks: ROS2 pkgs, AERIS driver, detection_node, radar topics, GCS reachable, SITL lock files |
| `kill_drone_sim.sh` | Kills simulator processes only (not GCS) |
| `gen_drone_svg.py` | Generates `images/mbc3_radar_drone.svg` architecture diagram |
| `sdf_to_step.py` | Converts SDF geometry → STEP for CAD import |
| `win_scripts/apply_redesign.ps1` | Windows PowerShell: applies v2 redesign patches to SDF |
| `win_scripts/make_exact_v3.ps1` | Windows PowerShell: builds exact v3 SDF geometry |

---

## `images/` — Reference images

| Path | Purpose |
|------|---------|
| `Screenshot 2026-05-22 205131.png` | Gazebo SITL screenshot (used in README) |
| `mbc3_radar_drone.svg` | Architecture SVG diagram (used in README) |
| `assembly_reference/` | Physical drone assembly photos — side, back, 3D renders |

---

## `meshes/` — Gazebo mesh assets

| File | Purpose |
|------|---------|
| `10in_Prop_CW.dae` | Collada mesh for 10-inch CW propeller (visual only) |

---

## `map_output/` — Runtime output

| File | Purpose |
|------|---------|
| `raw_cloud_*.pcd` | Raw LIDAR point cloud captured during flight |
| `voxel_map_*.pcd` | Voxel-downsampled map built by `mapping_3d.py` |

---

## `logs/` — Runtime logs

Populated at runtime by `launch.sh` and `swarm_launch.sh`. Git-ignored.

---

## Key relationships

```
launch.sh
  └─ PX4 SITL + Gazebo (world: mbc3_radar_targets.sdf)
  └─ aeris10_driver (ROS2) ──► /radar_{A-F}/scan/points
  └─ radar_fusion  (ROS2) ──► /radar/targets
  └─ asp_bridge.py         ──► POST /asp_update ──► telemetry_web.py (GCS :5000)
  └─ isr_lidar_mpc.py      ◄── MAVSDK gRPC :50051

swarm_launch.sh
  └─ 5× PX4 SITL (ports 14540-14544)
  └─ radar_sim.py          ──► POST /asp_update ──► swarm_telemetry_web.py (GCS :5000)
  └─ swarm_mission.py      ◄── MAVSDK gRPC :50050-50054
       └─ D2DNode           ──► UDP multicast 224.1.1.1:14900
       └─ _follow_loop      ◄── GET /api/track_state ◄── operator clicks TRACK in GCS
```
