# Document 2 — Short Brief on Previous and Current Products / Tech Development
**Aran Technologies | MBC-3 Registration | Max 300 words**

---

Aran Technologies is a defence-focused UAS development team with hands-on experience across flight controller hardware, autonomous mission software, and AI-driven surveillance systems.

## ESP32 Autonomous Flight Controller

We designed and built a complete custom flight controller from scratch on ESP32-WROOM, running FreeRTOS dual-core architecture at 250 Hz. The system integrates a Kalman-filter IMU pipeline (MPU6050), barometer-based altitude hold (BMP280), GPS position hold and Return-to-Home (NEO-6M), and compass-referenced body-frame position control (QMC5883L). A dual-loop PID architecture (outer angle loop, inner rate loop) drives the motor mixer for an X-frame quadcopter. This project demonstrated indigenous design of flight-critical embedded systems with multi-sensor fusion.

## ISR Mission Stack with LiDAR MPC Avoidance — Nirmaan / IIT Madras Demo

Developed a full autonomous ISR mission stack on PX4 SITL + Gazebo Harmonic using MAVSDK Python. Key capabilities include 360° LiDAR obstacle avoidance driven by a Model Predictive Controller (MPC with L-BFGS-B finite-horizon QP solver), boustrophedon survey grid, multi-target ISR orbit sequences, No-Fly Zone enforcement, and a live Flask/SocketIO Ground Control Station dashboard. This was the IIT Madras Nirmaan incubation programme demonstration build.

## MBC-3 Collaborative Swarm — Current Development

Five-drone hexacopter swarm functioning as a distributed airborne radar with a three-layer onboard AI pipeline. Full simulation stack verified on ROS2 Jazzy + Gazebo Harmonic with graceful degradation, leader election, and real-time ASP generation.

---

*Word count: ~265 | Limit: 300*
