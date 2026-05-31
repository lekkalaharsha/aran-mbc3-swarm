# Document 2 — Short Brief on Previous and Current Products / Tech Development
**Aran Technologies | MBC-3 Registration | Max 300 words**

---

Aran Technologies is a defence-focused UAS development team with demonstrated capability across flight controller hardware, autonomous mission software, and AI-driven surveillance systems.

## Indigenous Flight Controller — Custom STM32 Platform

Aran Technologies designed and fabricated an autonomous flight controller on a fully custom STM32 PCB. Operating on FreeRTOS at 250 Hz, the system integrates a Kalman-filter IMU pipeline (MPU6050), barometer altitude hold (BMP280), GPS position hold and RTH (NEO-6M), and compass-referenced body-frame position control (QMC5883L). A dual-loop PID architecture — outer angle loop, inner rate loop — drives the motor mixer. All schematic design, PCB layout, and firmware are indigenously developed.

## AERIS-10 Open-Source Radar — Active Contributor

Aran Technologies contributes to AERIS-10 (PLFM_RADAR), an open-source 10.5 GHz pulse-LFM phased array radar system. Contributions span FPGA signal-processing RTL (Verilog), STM32 firmware, and Python GUI. This collaboration directly informs Aran Technologies' radar payload architecture for the MBC-3 mission.

## ISR Mission Stack — Verified Phase I Demo Build (v12-MPC-v5)

Aran Technologies developed a full autonomous ISR mission stack on PX4 SITL and Gazebo Harmonic using MAVSDK Python. Core capabilities include 360° LiDAR obstacle avoidance driven by a Model Predictive Controller (L-BFGS-B finite-horizon QP solver), boustrophedon survey grid generation, multi-target ISR orbit sequencing, No-Fly Zone hard fencing, and a live Flask/SocketIO Ground Control Station streaming altitude, heading, battery state, and mission phase at 2.5 Hz.

The stack (v12-MPC-v5) was verified on 30 May 2026 with a full clean exit: 11-waypoint survey, primary target orbit locked at 50 m radius ±0.5 m, RTL, and 3D occupancy map save. A 4-minute demonstration video (Gazebo and GCS side-by-side, 1920×1080) is prepared for Phase I submission. This stack forms the single-drone foundation of the MBC-3 swarm submission.

## MBC-3 Collaborative Swarm — Current Development

Five-drone hexacopter swarm operating as a distributed airborne radar, incorporating a three-layer onboard AI pipeline. Full simulation verified on ROS2 Jazzy and Gazebo Harmonic with graceful degradation, bully-protocol leader election, and real-time ASP generation.

---

*Word count: ~295 | Limit: 300*
