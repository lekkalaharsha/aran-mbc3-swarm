# Document 1 — Proposal for MBC-3
**Aran Technologies | MBC-3 Registration | Max 500 words**

---

## Collaborative FMCW Radar Swarm with Onboard AI for Aerial Surveillance

### Problem Statement

Contemporary aerial surveillance requires persistent, wide-area detection of low-observable targets at operationally sustainable cost. Single-platform airborne radar is vulnerable to attrition and cannot provide distributed coverage. Aran Technologies addresses this requirement through a five-drone swarm configured as a distributed FMCW radar — a micro-AWACS architecture — that distributes aperture across platforms, provides mission redundancy, and enables autonomous track management independent of continuous ground-station control.

### Proposed Solution

Aran Technologies proposes a five-hexacopter swarm, each platform carrying six 24 GHz FMCW radar panels (TI AWR1843, 60° H-FOV per panel) providing full 360° coverage per drone. A three-layer onboard AI pipeline autonomously processes raw radar data through to tactical decision output.

**Layer 1 — Signal Processing:** AWR1843 DSP performs range-Doppler FFT and CFAR detection within 10 ms, outputting range, azimuth, radial velocity, and SNR.

**Layer 2 — Classification:** Random Forest on Jetson classifies candidates as targets or clutter within 50 ms using SNR, velocity, range-rate, and RCS. Only confirmed detections reach Layer 3.

**Layer 3 — LLM Tactical Engine:** Llama 3.2 3B (leader) and Gemma 2B (soldiers) receive JSON situation reports and generate tactical commands — track reassignment, formation reallocation, sector reorientation, and threat alerts — triggering only on Layer 2 confirmations.

### Swarm Architecture and Resilience

Command hierarchy: Ground Station > Leader Drone > Soldier autonomous mode. On leader heartbeat loss exceeding 2 seconds, the highest-battery soldier self-elects as leader via bully election protocol, assuming radar fusion and ASP publication within 2 seconds. On soldier loss, the leader LLM redistributes orphaned track IDs to the nearest active drone. Full Air Situation Picture continuity is sustained with three or more drones operational, satisfying the MBC-3 graceful degradation requirement.

A Flask-based Ground Control Station displays a consolidated real-time ASP at 2.5 Hz — track table, polar radar display, leader identity, decision log, and timestamped JSON recording — on a single browser screen with no external dependency.

### Platform Specification

Each hexacopter carries: TI AWR1843BOOST radar panels ×6, indigenous STM32 flight controller (custom PCB), Doodle Labs AES-128 mesh radio, and Jetson edge compute (AGX Orin 64 GB on leader; Orin NX 16 GB on soldiers). AUW ≤ 4.3 kg. Endurance: ~32 min (6S, 10,000 mAh). Auto RTH on data-link loss or low battery. GNSS-denied resilience via EKF2 fusing optical flow, IMU, and barometer.

### Indigenisation

The complete intelligence layer — CFAR software, Random Forest classifier, LLM tactical engine, ROS2 swarm coordination stack, GCS dashboard, custom antenna PCB design, and hexacopter airframe fabrication — is 100% indigenously developed. Weighted indigenous content: ≥ 55% by mission-criticality, satisfying MBC-3 §2.25.

### Phase I Deliverable

Phase I testing complete: five-drone SITL simulation verified (ASP generation, LLM track reallocation, graceful degradation), broadband and drone-to-ground comms tested, pre-recorded 4-min ISR demo video (1920×1080) ready — live demonstration available at New Delhi, 13–24 July 2026 (github.com/lekkalaharsha/aran-mbc3-swarm).

---

*Word count: ~493 | Limit: 500*
