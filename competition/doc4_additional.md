# Document 4 — Additional Information
**Aran Technologies | MBC-3 Registration | Max 300 words**

---

## Simulation Demonstration — Phase I Ready

A verified five-drone swarm simulation demonstrates all core MBC-3 requirements on ROS2 Jazzy and Gazebo Harmonic: real-time ASP generation, bully-protocol leader election, graceful degradation on drone loss, and sector reassignment — 344 radar tracks logged with four surviving drones sustaining full mission continuity.

Single-drone ISR video prepared for Phase I: 4-minute 1920×1080 recording covering survey grid (11 WPs), primary target orbit (50 m radius, locked ±0.5 m), and RTL with 3D occupancy map save. Mission exit verified clean (v12-MPC-v5).

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

Simulation stack, GCS dashboard, and graceful degradation demonstration are prepared for live presentation at New Delhi, 13–24 July 2026.

---

*Word count: ~295 | Limit: 300*
