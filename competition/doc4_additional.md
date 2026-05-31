# Document 4 — Additional Information
**Aran Technologies | MBC-3 Registration | Max 300 words**

---

## Phase I Testing — Completed

All Phase I systems have been tested and verified. The five-drone swarm simulation on ROS2 Jazzy and Gazebo Harmonic has demonstrated all core MBC-3 requirements: real-time ASP generation, bully-protocol leader election, graceful degradation on drone loss, and sector reassignment — 344 radar tracks logged with four surviving drones sustaining full mission continuity.

A pre-recorded 4-minute single-drone ISR mission video (1920×1080, v12-MPC-v5) is ready for Phase I presentation: 11-waypoint survey, primary target orbit locked at 50 m radius ±0.5 m, RTL, and 3D occupancy map save — verified clean exit. Broadband and drone-to-ground communication links have been tested and confirmed operational.

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

**Kishore Udhayakumar** | Founder & CEO — Mechanical Engineering. Leads product development, mechanical design, thermal engineering, and business strategy. Drives go-to-market planning and direct engagement with Indian defence end-users.

**Vegi Tejo Bhargav** | Co-Founder & CTO — Mechanical Engineering. Manages project execution, hexacopter airframe fabrication, additive manufacturing, procurement, and hardware integration roadmap.

**L. Harsha Vardhan Naidu** | Product Head, Electronics — EEE. Designed the complete autonomous mission stack: PX4 SITL pipeline, ROS2 swarm middleware, GCS dashboard, STM32 flight controller firmware, and three-layer onboard AI pipeline (CFAR → Random Forest → LLM).

Point of Contact: aranrobotics@gmail.com | +91 72888 40612

## Phase I Readiness

Simulation stack, GCS dashboard, graceful degradation demonstration, and pre-recorded mission video are complete and ready for live presentation at New Delhi, 13–24 July 2026.

---

*Word count: ~295 | Limit: 300*
