# Document 1 — Proposal for MBC-3
**Aran Technologies | MBC-3 Registration | Max 500 words**

---

## Collaborative FMCW Radar Swarm with Onboard AI for Aerial Surveillance

### Problem Statement

Contemporary aerial surveillance requires persistent, wide-area detection of low-observable targets at operationally sustainable cost. Single-platform airborne radar is vulnerable to attrition and cannot provide distributed coverage. Aran Technologies addresses this requirement through a five-drone swarm configured as a distributed FMCW radar — a micro-AWACS architecture — that distributes aperture across platforms, provides mission redundancy, and enables autonomous track management independent of continuous ground-station control.

### Proposed Solution

Aran Technologies proposes a five-hexacopter swarm, each platform carrying six 24 GHz FMCW radar panels (TI AWR1843, 60° H-FOV per panel) providing full 360° coverage per drone. A three-layer onboard AI pipeline autonomously processes raw radar data through to tactical decision output.

**Layer 1 — Signal Processing:** The AWR1843 onboard DSP performs range-Doppler FFT and CFAR detection within 10 ms per scan cycle, producing candidate detections with range, azimuth, radial velocity, and SNR.

**Layer 2 — Target Classification:** A Random Forest classifier executing on each drone's Jetson compute module classifies candidates as confirmed targets or clutter within 50 ms, using SNR, velocity, range-rate, and estimated RCS. Only confirmed detections propagate to Layer 3.

**Layer 3 — LLM Tactical Engine:** Llama 3.2 3B (leader drone) and Gemma 2B (soldier drones) receive structured JSON situation reports and generate tactical commands — track reassignment, formation reallocation, sector reorientation, and threat alerts — operating within edge-compute budget and triggering exclusively on Layer 2 confirmations.

### Swarm Architecture and Resilience

Command hierarchy: Ground Station > Leader Drone > Soldier autonomous mode. On leader heartbeat loss exceeding 2 seconds, the highest-battery soldier self-elects as leader via bully election protocol, assuming radar fusion and ASP publication within 2 seconds. On soldier loss, the leader LLM redistributes orphaned track IDs to the nearest active drone. Full Air Situation Picture continuity is sustained with three or more drones operational, satisfying the MBC-3 graceful degradation requirement.

A Flask-based Ground Control Station displays a consolidated real-time ASP at 2.5 Hz — track table, polar radar display, leader identity, decision log, and timestamped JSON recording — on a single browser screen with no external dependency.

### Platform Specification

Each hexacopter carries: TI AWR1843BOOST radar panels ×6, indigenous STM32 flight controller (custom PCB), VectorNav VN-100 IMU, Doodle Labs AES-128 mesh radio, and Jetson edge compute (AGX Orin 64 GB on leader; Orin NX 16 GB on soldier drones). AUW ≤ 4.3 kg. Operational altitude: 500 m AGL minimum. Endurance: approximately 32 minutes (6S, 10,000 mAh). Automatic RTH on data-link loss or critical battery. GNSS-denied resilience via EKF2 fusing optical flow (60 Hz), VN-100 IMU (400 Hz), and barometer (50 Hz).

### Indigenisation

The complete intelligence layer — CFAR software, Random Forest classifier, LLM tactical engine, ROS2 swarm coordination stack, GCS dashboard, custom antenna PCB design, and hexacopter airframe fabrication — is 100% indigenously developed. Weighted indigenous content: ≥ 55% by mission-criticality, satisfying MBC-3 §2.25.

### Phase I Deliverable

A verified five-drone Gazebo Harmonic SITL simulation demonstrating real-time ASP generation, drone-loss triggering LLM track reallocation, and ASP continuity across surviving drones — available for live demonstration at New Delhi, 13–24 July 2026.

---

*Word count: ~480 | Limit: 500*
