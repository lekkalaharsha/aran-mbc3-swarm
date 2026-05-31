# Document 4 — Additional Information
**Aran Technologies | MBC-3 Registration | Max 300 words**

---

## Phase I Testing — Completed

Five-drone swarm simulation verified on ROS2 Jazzy / Gazebo Harmonic: ASP generation, bully-protocol leader election, graceful degradation, sector reassignment — 344 tracks logged, four surviving drones sustaining full continuity. Pre-recorded 4-min ISR video (1920×1080, v12-MPC-v5) ready: 11-WP survey, 50 m orbit ±0.5 m, RTL, 3D map save. Broadband and drone-to-ground comms tested and operational.

## Indigenisation Breakdown

| Sub-system | Indigenous Content |
|---|---|
| Hexacopter airframe | Indigenous design and local fabrication |
| Antenna panels (6 × per drone) | Indigenous PCB design |
| AI software stack (CFAR, RF, LLM) | 100% indigenous (Python / C++, open-source models) |
| GCS dashboard (Flask / SocketIO) | 100% indigenous |
| ROS2 swarm coordination stack | 100% indigenous |
| Flight controller firmware | Custom STM32 PCB — designed from scratch |
| Compute module (Jetson) | COTS — imported |
| Radar frontend (AWR1843) | COTS — imported |
| Mesh radio (Doodle Labs) | COTS — imported |

Estimated indigenisation: **≥ 55%** by mission-criticality weighting — meets MBC-3 §2.25 requirement.

## Team

Aran Technologies is a three-member engineering startup building indigenous defence UAS systems from the ground up.

**Kishore Udhayakumar** | Founder & CEO — Mechanical Engineering. Product development, mechanical design, thermal engineering, business strategy, and defence market engagement.

**Vegi Tejo Bhargav** | Co-Founder & CTO — Mechanical Engineering. Project execution, hexacopter airframe fabrication, additive manufacturing, and hardware integration.

**L. Harsha Vardhan Naidu** | Product Head, Electronics — EEE. Autonomous mission stack: PX4 SITL, ROS2 swarm middleware, GCS dashboard, STM32 FC firmware, and three-layer AI pipeline (CFAR → RF → LLM).

Point of Contact: aranrobotics@gmail.com | +91 72888 40612

---

*Word count: ~295 | Limit: 300*
