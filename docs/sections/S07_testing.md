# S07 — Testing & QA

**Status:** Unit tests 40/40 ✅ | Phase tests in progress

---

## Test Files

| File | Covers | Run without ROS? |
|------|--------|-----------------|
| `radar_fusion/test_unit.py` | radar_fusion pipeline (T1-T11) | ✅ yes |
| `tests/phase1_flight_test.py` | PX4 flight (T1-T7) | ❌ needs SITL |
| `tests/test_mission_config.py` | mission_config.py functions | ✅ yes |

---

## radar_fusion Unit Tests (40/40)

```bash
python3 radar_fusion/test_unit.py
```

| Test | Covers |
|------|--------|
| T1 | Panel rotation matrices A-F (60° spacing, orthonormal) |
| T2 | Kalman create, persist, prune (TTL) |
| T3 | Multi-drone spatial merge (5 drones → 1 track) |
| T4 | Target separation (no false merge) |
| T5 | Elevation gate logic |
| T6 | Track ID stable across 10 cycles |
| T7 | Kalman near-singular S — linalg.solve survives |
| T8 | Q dt-scaled — P grows with dt, zero gain at dt=0 |
| T9 | tf_ok=False targets dropped (BUG-RF-3) |
| T10 | RF makedirs flat path no crash (BUG-RF-1) |
| T11 | RF classifier: clear target=1, clear clutter=0 |

---

## Phase 1 Flight Test (phase1_flight_test.py)

```bash
# Terminal 1
./launch.sh --sim-only

# Terminal 2 (after PX4 ready)
python3 tests/phase1_flight_test.py
```

| Test | Criteria |
|------|---------|
| T1 | MAVSDK connects |
| T1b | GPS ok + home set within 30s |
| T2 | Armed successfully |
| T3 | Takeoff reaches 8.5m+ within 30s |
| T4 | Hover: altitude drift < 1m over 10s |
| T5 | Waypoint 50m north reached within 60s |
| T6 | RTH within 10m of home within 90s |
| T7 | Landed (ON_GROUND state) within 60s |
| T8 | No FAILED/Traceback in PX4 log (manual check) |

---

## Merge Checklist (run before merging any branch)

```bash
# 1. Syntax check all modified Python
python3 -m py_compile src/isr_lidar_mpc.py src/mpc_controller.py src/mapping_3d.py

# 2. Module self-tests
cd src && python3 mpc_controller.py && python3 mapping_3d.py

# 3. radar_fusion unit tests (if radar_fusion changed)
python3 radar_fusion/test_unit.py

# 4. Headless mission run
./launch.sh --headless --scenario mbc3_iaf_demo

# 5. Check log
grep -i "FAILED\|Traceback\|RuntimeError" logs/$(ls logs/ | tail -1)/mission.log
```

---

## Branch Merge Rule

```
New task branch → merge to PARENT branch (not main)

Example:
  fix/landing-gear-spawn → feature/drone-visualization
  feature/drone-visualization → feature/6-panel-radar-coverage
  test/phase1-px4-flight → feature/6-panel-radar-coverage
  feature/6-panel-radar-coverage → main  (after full checklist)
```

**Never merge directly to main from a sub-task branch.**

---

## Open Tasks

- [ ] Phase 1 test: T1b fix (health timeout) — committed
- [ ] Run full phase1_flight_test.py after PX4 stable
- [ ] Add phase test results to S06 status table
- [ ] Add test for radar_gz_reader 6-panel fusion (integration)
