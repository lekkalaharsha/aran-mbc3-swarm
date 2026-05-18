# Aran Technologies — Coding Rules
## ISR Mission + LiDAR MPC Avoidance Stack

---

## 1. Branch Policy

| Action | Rule |
|--------|------|
| Bug fix | Create `fix/<short-description>` branch |
| New feature | Create `feature/<short-description>` branch |
| Testing / experiments | Create `test/<short-description>` branch |
| Config / tooling | Create `chore/<short-description>` branch |

**Never commit directly to `main`.** All changes go through a branch → test → merge flow.

```
main  ←  merge only after verified pass
 └── fix/bug-name
 └── feature/new-capability
 └── test/experiment-name
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
