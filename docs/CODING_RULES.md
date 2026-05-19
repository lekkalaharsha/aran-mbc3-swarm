# Aran Technologies — Coding Rules
## ISR Mission + FMCW Radar Swarm + MBC-3 Competition Stack

---

## 1. Branch Policy

| Action | Rule |
|--------|------|
| Bug fix | Create `fix/<short-description>` branch |
| New feature | Create `feature/<short-description>` branch |
| Testing / experiments | Create `test/<short-description>` branch |
| Config / tooling | Create `chore/<short-description>` branch |
| MBC-3 phase test | Create `test/phase<N>-<name>` branch (one per phase) |
| Drone model / visuals | Create `feature/drone-visualization` branch |

**Never commit directly to `main`.** All changes go through a branch → test → merge flow.

**Merge target rule: merge to PARENT branch, not main.**
```
fix/landing-gear → feature/drone-visualization    (not main)
feature/drone-visualization → feature/6-panel     (not main)
test/phase1-px4-flight → feature/6-panel          (not main)
feature/6-panel-radar-coverage → main             (after full checklist)
```

```
main  ←  merge only after verified pass
 └── fix/bug-name
 └── feature/new-capability
 └── test/experiment-name
 └── test/phase1-px4-flight       ← MBC-3 Phase 1
 └── test/phase2-radar-web        ← MBC-3 Phase 2
 └── test/phase3-radar-avoidance  ← MBC-3 Phase 3
 └── test/phase4-multi-sequential ← MBC-3 Phase 4
 └── test/phase5-multi-radar      ← MBC-3 Phase 5
 └── test/phase6-leader-failover  ← MBC-3 Phase 6
 └── test/phase7-llm              ← MBC-3 Phase 7
 └── feature/drone-visualization  ← SDF/model visual work
```

---

## 2. Merge to Main — Checklist

Before merging any branch into `main`, ALL of the following must pass:

- [ ] `python3 -m py_compile` passes on every modified `.py` file
- [ ] Module self-tests pass (`python3 mpc_controller.py`, `python3 mapping_3d.py`)
- [ ] `./launch.sh --headless --scenario iit_panel_demo` runs without `FAILED` exit
- [ ] Survey completes all WPs (progress reaches 100%)
- [ ] GCS dashboard reachable at `http://localhost:5000` during run
- [ ] No new `Traceback` or `RuntimeError` in mission log
- [ ] `bugs.md` updated — any new bugs found are documented

---

## 3. Logic Change Policy

**Core flight logic must NOT be changed unless explicitly approved.**

Core logic includes:
- `avoidance_loop()` — debounce, escape bearing, detour WP, climb escape
- `MPCEngine` cost function and solver settings
- `_bearing_to_nearest()` / `_compute_sectors()` — bearing frame conversion
- Mission phase sequence (SURVEY → LOITER → SEC-N → RTL)
- NFZ fence check logic
- `generate_survey_grid()` waypoint generation

### When logic change IS allowed:
1. **Bug confirmed in `bugs.md`** with root cause documented
2. **High-priority issue** — crash, safety risk, demo blocker
3. **Explicit user instruction** — user directly asks for the change
4. **Performance issue** with measurable evidence (e.g. callback queue overflow)

### When AI (Claude) must NOT change logic:
- Refactoring "for clarity" without a functional reason
- Changing control parameters (gains, thresholds, timeouts) without evidence they are wrong
- Replacing algorithms (e.g. MPC → PID or vice versa) without instruction
- Restructuring async/thread architecture without a confirmed race condition

---

## 4. What AI Can Change Without Asking

- Bug fixes with a documented root cause in `bugs.md`
- Syntax errors, import errors, typos
- Log message wording
- Comment text
- File mode / line ending fixes (CRLF → LF)
- Dependency version pins
- `.gitignore` / `.gitattributes` / `CODING_RULES.md`

---

## 5. What AI Must Ask Before Changing

- Any constant in `mission_config.py` (coordinates, speeds, radii, altitudes)
- Any MPC cost weight or horizon length in `mpc_controller.py`
- Avoidance thresholds (`LIDAR_AVOID_DIST`, `DEBOUNCE_COUNT`, `AVOIDANCE_TIMEOUT_S`, etc.)
- The GCS HTML/JS frontend in `telemetry_web.py`
- `launch.sh` launch sequence steps or PX4 make target
- Scenario events in `scenarios.json`

---

## 6. Commit Message Format

```
<type>: <short summary> (<scope if needed>)

<body — what changed and WHY, not just what>

Co-Authored-By: Claude Sonnet 4.6 <noreply@anthropic.com>
```

Types: `fix` `feat` `chore` `test` `docs` `refactor`

- Subject line ≤ 72 chars
- Body explains the **why**, not the what
- Reference bug ID if applicable (`BUG-3`, `NEW-1`, etc.)

---

## 7. Bug Register

All bugs go in `bugs.md` with:
- ID, severity, file, status (`🔴 OPEN` / `✅ FIXED`)
- Problem description
- Root cause
- Fix applied (or proposed fix if open)

**Do not fix a bug without logging it first.**

---

## 8. File Ownership

| File | Owner / Purpose | Change frequency |
|------|-----------------|-----------------|
| `mission_config.py` | Mission parameters — single source of truth | Low — ask before changing |
| `isr_lidar_mpc.py` | Main mission logic | Medium — bugs only |
| `mpc_controller.py` | Control algorithms | Low — bugs only |
| `pid_controller.py` | Legacy reference | Rarely — do not delete |
| `telemetry_web.py` | GCS dashboard | Medium — UI/endpoint bugs |
| `mapping_3d.py` | 3D mapping | Low |
| `scenarios.json` | Sim test scenarios | Low — ask before changing |
| `launch.sh` | Orchestration | Low — bugs only |

---

## 9. Line Endings

All files must use **LF** (Unix) line endings. `.gitattributes` enforces this.  
If a file gets CRLF-corrupted: `sed -i 's/\r//' <file>` before committing.

---

## 10. No Silent Logic Drift

If any change — even a "small fix" — alters observable flight behaviour (trajectory, avoidance response, orbit radius, phase sequence), it must be:

1. Documented in `bugs.md` or commit body
2. Tested with `./launch.sh --headless --scenario iit_panel_demo`
3. Reviewed before merge to `main`

---

## 11. SDF-First Drone Model Workflow

**Source of truth:** `new_drone/mbc3_radar_drone.sdf`

```
Edit mbc3_radar_drone.sdf
        ↓
bash new_drone/install_px4_model.sh
        ↓  (copies SDF → model.sdf → PX4 models dir)
./launch.sh --sim-only   OR   gz sim mbc3_radar_drone.sdf
```

- XACRO (`mbc3_radar_drone.xacro`) is documentation only — SDF is authoritative
- After any SDF edit: copy updated file to `Downloads/drone/mbc3_radar_drone_fixed.sdf` as backup
- All SDF visual/physics changes go on `feature/drone-visualization` branch

### Motor param changes require MPC_THR_HOVER recalc:
```
hover_F     = mass_kg × 9.81 / 6
hover_omega = sqrt(hover_F / motorConstant)
MPC_THR_HOVER = hover_omega / maxRotVelocity
```
Current values (2026-05-19): mass=3.834 kg, motorConstant=1.74e-5, maxRotVelocity=838 → **MPC_THR_HOVER=0.72**

---

## 12. MBC-3 Phase Development Rules

Each phase gets:
- Its own `test/phase<N>-<name>` branch
- A test script `tests/phase<N>_<name>_test.py`
- Pass criteria defined before coding starts
- Phase does NOT merge to main until all tests pass

Phase order is strict — do not start Phase N+1 until Phase N test script passes.

---

## 13. radar_fusion Package Rules

| File | Rule |
|------|------|
| `radar_fusion/radar_fusion/kalman_tracker.py` | bugs only — Kalman math is verified |
| `radar_fusion/radar_fusion/rf_classifier.py` | ask before changing training data |
| `radar_fusion/radar_fusion/detection_node.py` | ask before changing panel layout |
| `radar_fusion/radar_fusion/fusion_node.py` | ask before changing gate/TTL params |
| `radar_fusion/config/radar_fusion.yaml` | ask before changing — affects live fusion |

Before merging any radar_fusion change:
- `python3 radar_fusion/test_unit.py` must pass 40/40
- No new BUG-RF entries left OPEN in bugs.md

---

## 14. Updated File Ownership

| File | Owner / Purpose | Change rule |
|------|-----------------|-------------|
| `src/mission_config.py` | Mission parameters | Ask before changing |
| `src/isr_lidar_mpc.py` | Main mission logic | Bugs only |
| `src/mpc_controller.py` | MPC algorithm | Bugs only |
| `src/telemetry_web.py` | ISR GCS dashboard | Ask for frontend changes |
| `src/mapping_3d.py` | 3D voxel map | Low — bugs only |
| `src/scenarios.json` | Sim scenarios | Ask before changing |
| `launch.sh` | Full stack orchestration | Bugs only |
| `new_drone/mbc3_radar_drone.sdf` | Drone physics+sensors | `feature/drone-visualization` branch |
| `new_drone/airframe/4601_gz_mbc3_radar_drone` | PX4 params | Recalc hover after motor changes |
| `radar_fusion/` | ROS2 detection+fusion | See Section 13 |
| `docs/bugs.md` | Bug register | Log before fixing |
