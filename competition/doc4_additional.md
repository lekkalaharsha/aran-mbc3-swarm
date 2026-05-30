# Document 4 — Additional Information
**Aran Technologies | MBC-3 Registration | Max 300 words**

---

## Simulation Demonstration — Ready for Phase I

A fully functional five-drone swarm simulation is complete and verified as of 30 May 2026. The simulation runs on ROS2 Jazzy + Gazebo Harmonic on Ubuntu 24.04 and demonstrates all core MBC-3 requirements: real-time ASP generation, graceful degradation on drone loss, autonomous leader election, and sector reassignment. The full swarm test (including live drone kill and recovery) passes all checks with 344 radar tracks and 4/4 surviving drones operational.

A single-drone ISR demo video is recorded and ready for Phase I submission: 4-minute recording (1920×1080), showing Gazebo 3D simulation and GCS dashboard side-by-side across survey grid (11 waypoints), primary target orbit (50 m radius, locked ±0.5 m), and RTL with 3D map save. Mission exit verified clean (v12-MPC-v5, 30 May 2026).

## Indigenisation Breakdown

| Sub-system | Indigenous Content |
|---|---|
| Hexacopter airframe | Indigenous design + local fabrication |
| Antenna panels (4 × per drone) | Indigenous PCB design |
| AI software stack (CFAR, RF, LLM) | 100% indigenous (Python/C++, open-source models) |
| GCS dashboard (Flask) | 100% indigenous |
| ROS2 swarm coordination stack | 100% indigenous |
| Flight controller firmware | ESP32-based indigenous design |
| Compute (Jetson) | COTS — imported |
| Radar frontend (AWR1843) | COTS — imported |
| Mesh radio (Doodle Labs) | COTS — imported |

Estimated indigenisation: **≥ 55%** by mission-criticality weighting — meets MBC-3 §2.25 requirement.

## Team

Cross-disciplinary team with expertise in embedded systems, autonomous flight, ROS2 middleware, edge AI, and defence product market research. Active participation in Nirmaan IIT Madras incubation programme. Point of Contact: L. Harsha Vardhan Naidu — `aranrobotics@gmail.com` | +91 72888 40612.

## Phase I Readiness

Simulation stack, GCS dashboard, and graceful degradation demo are ready for live presentation at New Delhi (13–24 July 2026).

---

*Word count: ~265 | Limit: 300*
