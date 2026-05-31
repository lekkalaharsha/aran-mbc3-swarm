# Document 2 — Short Brief on Previous and Current Products / Tech Development
**Aran Technologies | MBC-3 Registration | Max 300 words**

---

## Indigenous Flight Controller — Custom STM32 Platform

Custom STM32 PCB flight controller: FreeRTOS at 250 Hz, Kalman-filter IMU (MPU6050), barometer altitude hold (BMP280), GPS position hold and RTH (NEO-6M), compass body-frame control (QMC5883L), dual-loop PID (outer angle, inner rate). Schematic, PCB layout, and firmware 100% indigenously developed.

## AERIS-10 Open-Source Radar — Active Contributor

Aran Technologies contributes to AERIS-10 (PLFM_RADAR), an open-source 10.5 GHz pulse-LFM phased array radar system. Contributions span FPGA signal-processing RTL (Verilog), STM32 firmware, and Python GUI. This collaboration directly informs Aran Technologies' radar payload architecture for the MBC-3 mission.

## ISR Mission Stack — Verified Phase I Demo Build (v12-MPC-v5)

Aran Technologies developed a full autonomous ISR mission stack on PX4 SITL and Gazebo Harmonic using MAVSDK Python. Core capabilities include 360° LiDAR obstacle avoidance driven by a Model Predictive Controller (L-BFGS-B finite-horizon QP solver), boustrophedon survey grid generation, multi-target ISR orbit sequencing, No-Fly Zone hard fencing, and a live Flask/SocketIO Ground Control Station streaming altitude, heading, battery state, and mission phase at 2.5 Hz.

Verified on 30 May 2026 (v12-MPC-v5): 11-waypoint survey, 50 m orbit ±0.5 m, RTL, 3D map save. Pre-recorded 4-min demo video (1920×1080) ready for Phase I. Single-drone foundation of the MBC-3 swarm submission.

## MBC-3 Collaborative Swarm — Current Development

Five-drone hexacopter swarm operating as a distributed airborne radar, incorporating a three-layer onboard AI pipeline. Full simulation verified on ROS2 Jazzy and Gazebo Harmonic with graceful degradation, bully-protocol leader election, and real-time ASP generation.

---

*Word count: ~295 | Limit: 300*
