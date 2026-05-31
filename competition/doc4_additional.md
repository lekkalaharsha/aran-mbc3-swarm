# Document 4 — Additional Information
**Aran Technologies | MBC-3 Registration | Max 300 words**

---

## Simulation Demonstration — Phase I Ready

A fully verified five-drone swarm simulation is complete as of 30 May 2026. The simulation operates on ROS2 Jazzy and Gazebo Harmonic on Ubuntu 24.04 and demonstrates all core MBC-3 technical requirements: real-time Air Situation Picture generation, graceful degradation on drone loss, autonomous bully-protocol leader election, and sector reassignment across surviving drones. A complete swarm verification test — including live drone elimination and recovery — confirms 344 radar tracks logged with all four surviving drones sustaining full mission continuity.

A single-drone ISR demonstration video is recorded and prepared for Phase I submission: a 4-minute, 1920×1080 recording presenting Gazebo 3D simulation and GCS dashboard side-by-side across survey grid (11 waypoints), primary target orbit (50 m radius, locked ±0.5 m), and RTL with 3D occupancy map save. Mission exit verified clean (v12-MPC-v5, 30 May 2026).

## Indigenisation Breakdown

| Sub-system | Indigenous Content |
|---|---|
| Hexacopter airframe | Indigenous design and local fabrication |
| Antenna panels (6 × per drone) | Indigenous PCB design |
| AI software stack (CFAR, RF, LLM) | 100% indigenous (Python / C++, open-source models) |
| GCS dashboard (Flask / SocketIO) | 100% indigenous |
| ROS2 swarm coordination stack | 100% indigenous |
| Flight controller firmware | ESP32-based indigenous design |
| Compute module (Jetson) | COTS — imported |
| Radar frontend (AWR1843) | COTS — imported |
| Mesh radio (Doodle Labs) | COTS — imported |

Estimated indigenisation: **≥ 55%** by mission-criticality weighting — meets MBC-3 §2.25 requirement.

## Team

Aran Technologies is a defence-focused engineering team with demonstrated capability in embedded flight systems, autonomous mission software, ROS2 swarm middleware, edge AI inference, and defence product market research validated through direct engagement with Indian defence end-users. Point of Contact: L. Harsha Vardhan Naidu — `aranrobotics@gmail.com` | +91 72888 40612.

## Phase I Readiness

Simulation stack, GCS dashboard, and graceful degradation demonstration are prepared for live presentation at New Delhi, 13–24 July 2026.

---

*Word count: ~285 | Limit: 300*
