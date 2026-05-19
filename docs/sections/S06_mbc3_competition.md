# S06 — MBC-3 Competition (IAF Mehar Baba Competition-3)

**Status:** Phase 0 submitted | Phase I (Gazebo sim demo) in development

---

## Proposal Summary

5-drone swarm functioning as distributed airborne radar (micro-AWACS).  
3-layer AI pipeline: CFAR → RF classifier → LLM tactical engine.

---

## Competition Requirements vs Implementation

| Requirement | Target | Implementation | Status |
|-------------|--------|---------------|--------|
| Min 5 VTOL UAS | 5 hex | 5× mbc3_radar_drone | 🔲 Phase 4 |
| 360° FOV | 360° | 6× panels × 60° = 360° | ✅ done |
| Range 2-5 km | AWR1843 | gpu_lidar 0.5-5km sim | ✅ done |
| Range res 120 m | 4 GHz BW | 0.12m resolution set | ✅ done |
| Velocity 10-40 m/s | Doppler | RF classifier features | 🔲 Phase 3 |
| Multi-target ≥ 5 | 10+ | Kalman fusion 10+ | ✅ done |
| Revisit < 10s | 10 Hz radar | 10 Hz update rate | ✅ done |
| Op height ≥ 500m AGL | 500m | Configurable altitude | ✅ done |
| Endurance ≥ 30 min | 32 min | 6S 10Ah @3.834kg | ✅ done |
| Graceful degradation | ≥3 drones | LLM track realloc | 🔲 Phase 6 |
| Self-healing | <2s leader elect | Heartbeat monitor | 🔲 Phase 6 |
| Swarm split-merge | 5s re-fusion | LLM command | 🔲 Phase 7 |
| Day and night | FMCW | Light-independent | ✅ done |
| ASP single screen | 2.5 Hz | Flask ASP GCS | 🔲 Phase 2 |
| Auto-RTH | Pixhawk | Pixhawk 6C | ✅ done |
| GNSS-denied | EKF2 flow | Sensors in SDF | ✅ sensor |
| Encrypted link | AES-128 | Doodle Labs (HW) | HW only |
| Min manpower 2 | 2 ops | launch.sh auto checks | ✅ done |
| Indigenisation ≥50% | ~60% | Software 100% indigenous | ✅ done |

---

## 7-Phase Development Plan

| Phase | Branch | Goal | Status |
|-------|--------|------|--------|
| 1 | `test/phase1-px4-flight` | Custom drone flies (hover/WP/RTL) | 🔄 In progress |
| 2 | `test/phase2-radar-web` | Radar → ASP browser display | 🔲 |
| 3 | `test/phase3-radar-avoidance` | Radar replaces LiDAR avoidance | 🔲 |
| 4 | `test/phase4-multi-sequential` | 5 drones, one-at-a-time | 🔲 |
| 5 | `test/phase5-multi-radar` | 5 simultaneous + radar avoidance | 🔲 |
| 6 | `test/phase6-leader-failover` | Leader election on failure | 🔲 |
| 7 | `test/phase7-llm` | SLM/LLM tactical engine | 🔲 |

**Rule:** Phase N+1 does not start until Phase N test script passes.

---

## Phase I Demo Requirements (Gazebo sim)

From proposal section 5:
1. Five-drone Gazebo sim showing real-time ASP generation
2. Drone loss → LLM track reallocation
3. ASP continuity with ≥4 remaining drones

Covered by: Phases 2 + 4 + 5 + 6 + 7

---

## Note on Panel Count

Proposal says "4 panels at 0/90/180/270°" but implementation uses **6 panels at 60° spacing**.  
6 panels is better (wider overlap, zero gaps). Update compliance table text in proposal for Phase I submission.

---

## Open Tasks

- [ ] Phase 1: complete flight test (test/phase1-px4-flight)
- [ ] Phase 2: ASP GCS (test/phase2-radar-web)
- [ ] Update proposal compliance table: 4 panels → 6 panels
- [ ] IIT Hyderabad demo dry-run (existing ISR stack)
