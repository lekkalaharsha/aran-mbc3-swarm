# Document 1 — Proposal for MBC-3
**Aran Technologies | MBC-3 Registration | Max 500 words**

---

## Collaborative FMCW Radar Swarm with Onboard AI for Aerial Surveillance

### Problem Statement

Modern aerial surveillance demands persistent, wide-area coverage against low-observable targets at low cost. A five-drone swarm functioning as a distributed FMCW radar — a micro-AWACS architecture — addresses this requirement by distributing aperture, providing redundancy, and enabling autonomous track management without ground-station intervention.

### Proposed Solution

We propose a five-hexacopter swarm, each drone carrying four 24 GHz FMCW radar panels (TI AWR1843, 90° H-FOV per panel) providing full 360° coverage. A three-layer onboard AI pipeline processes raw radar data through to tactical decisions autonomously.

**Layer 1 — Signal Processing:** The AWR1843 onboard DSP performs range-Doppler FFT and CFAR detection in under 10 ms per scan cycle, outputting candidate detections with range, azimuth, radial velocity, and SNR.

**Layer 2 — Target Classification:** A Random Forest model on each drone's Jetson compute classifies candidates as real targets or clutter in under 50 ms, using SNR, velocity, range-rate, and estimated RCS. Only confirmed detections propagate to Layer 3.

**Layer 3 — LLM Tactical Engine:** Llama 3.2 3B on the leader drone and Gemma 2B on each soldier receive structured JSON situation reports and output tactical commands — track reassignment, formation reallocation, sector reorientation, and alert generation — operating within edge-compute budget by triggering only on Layer 2 confirmations.

### Swarm Architecture

The swarm operates under a priority hierarchy: Ground Station > Leader Drone > Soldier autonomous LLM. On leader heartbeat loss (>2 s), the highest-battery soldier self-elects as new leader via a bully election protocol and assumes radar fusion and ASP publishing within 2 seconds. On soldier loss, the leader LLM reassigns orphaned track IDs to the nearest active drone. Full Air Situation Picture (ASP) continuity is maintained with three or more drones operational, satisfying the graceful degradation requirement.

A Flask-based GCS displays a consolidated real-time ASP at 2.5 Hz on a single browser screen with track table, polar radar view, leader identity, decision log, and timestamped JSON session recording.

### Platform

Each hexacopter carries: TI AWR1843BOOST radar panels × 4, Pixhawk 6C flight controller, VectorNav VN-100 IMU, Doodle Labs AES-128 mesh radio, and Jetson edge compute (AGX Orin 64 GB on leader; Orin NX 16 GB on soldiers). AUW ≤ 4.1 kg. Operational altitude: 500 m AGL minimum. Endurance: ~32 min (6S 10,000 mAh). Auto-RTH on link loss or low battery. GNSS-denied operation via EKF2 fusing optical flow (60 Hz), VN-100 (400 Hz), and barometer (50 Hz).

### Indigenisation

The entire intelligence layer — CFAR software, Random Forest classifier, LLM engine, ROS2 swarm stack, GCS dashboard, custom antenna PCB design, and hexacopter airframe fabrication — is 100% indigenously developed. Weighted indigenous content: ≥55% by mission-criticality, satisfying MBC-3 §2.25.

### Phase I Deliverable

A verified five-drone Gazebo Harmonic simulation demonstrating real-time ASP generation, drone loss triggering LLM track reallocation, and ASP continuity — live at New Delhi, 13–24 July 2026.

---

*Word count: ~490 | Limit: 500*
