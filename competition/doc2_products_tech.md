# Document 2 — Short Brief on Previous and Current Products / Tech Development
**Aran Technologies | MBC-3 Registration | Max 300 words**

---

## Indigenous Flight Controller — Custom STM32 Platform

Custom STM32 PCB flight controller: FreeRTOS at 250 Hz, Kalman-filter IMU (MPU6050), barometer altitude hold (BMP280), GPS position hold and RTH (NEO-6M), compass body-frame control (QMC5883L), dual-loop PID (outer angle, inner rate). Schematic, PCB layout, and firmware 100% indigenously developed.

## AERIS-10 Open-Source Radar — Active Contributor

Aran Technologies contributes to AERIS-10 (PLFM_RADAR), an open-source 10.5 GHz pulse-LFM phased array radar system. Contributions span FPGA signal-processing RTL (Verilog), STM32 firmware, and Python GUI. This collaboration directly informs Aran Technologies' radar payload architecture for the MBC-3 mission.

## ISR Mission Stack — Verified Phase I Demo Build (v12-MPC-v5)

Aran Technologies developed a full autonomous ISR mission stack on PX4 SITL and Gazebo Harmonic using MAVSDK Python. Core capabilities: LiDAR obstacle avoidance (MPC, L-BFGS-B solver), boustrophedon survey, multi-target ISR orbit sequencing, No-Fly Zone fencing, and a FastAPI Ground Control Station at 2.5 Hz. Verified 30 May 2026: 11-waypoint survey, 50 m orbit ±0.5 m, RTL, 3D map save. Single-drone demo video (1920×1080) ready.

## MBC-3 Collaborative Swarm — Simulation-Verified

Five-drone hexacopter swarm operating as a distributed airborne radar with a three-layer onboard AI pipeline. Fully verified on ROS2 Jazzy and Gazebo Harmonic: graceful degradation, bully-protocol leader election, rMADER distributed trajectory deconfliction (0.3 s commitment window, HMAC-signed UDP multicast on 224.1.1.1:14900), and real-time Air Situation Picture generation. Five-drone swarm demo video (1920×1080, leader failover sequence) ready for Phase I submission.

---

*Word count: ~298 | Limit: 300*
