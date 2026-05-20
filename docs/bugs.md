# Aran Technologies — Bug Register
## ISR Mission + LiDAR MPC Avoidance v12-MPC-v5

---

## STATUS KEY
| Symbol | Meaning |
|--------|---------|
| ✅ FIXED | Patched and committed |
| 🔴 OPEN | Not yet fixed |

---

## BATCH 1 — Pre-existing bugs (found in code review, fixed in commit `8c811e9`)

---

### BUG-A | CRITICAL | Phase race — `_mode()` overwrites SEC phases
**File:** `telemetry_web.py` — `_mode()` stream + `lidar_update()` endpoint  
**Status:** ✅ FIXED

**Problem:**  
MAVSDK `_mode()` telemetry stream maps `HOLD → "LOITER"` and writes `data["mission_phase"]`.  
This overwrote the `"SEC-1"` / `"SEC-2"` / `"SEC-3"` values pushed by `isr_lidar_mpc.push_to_gcs()` via `POST /lidar_update`.  
Result: GCS phase panel stayed frozen at `LOITER` for all secondary orbits. PX4 reports `HOLD` for every `do_orbit()` call so the flight-mode mapper had no way to distinguish primary vs secondary phases.

**Root cause:** Two competing writers, no arbitration.

**Fix applied:**  
Added `_phase_state = {"push_time": 0.0}`. Updated on every successful `mission_phase` POST from the mission script. `_mode()` only writes `mission_phase` if `now - push_time > 5.0s` — MAVSDK telemetry only takes over when the mission script stops pushing.

---

### BUG-B | HIGH | `MapBuilder` imported but never used — 3D mapping dead
**File:** `isr_lidar_mpc.py`  
**Status:** ✅ FIXED

**Problem:**  
`from mapping_3d import MapBuilder` was present at the top of the file but no `MapBuilder` instance was ever created. Both `lidar_gz_reader()` and `lidar_sim_reader()` never called `.ingest()`. The 3D occupancy map was completely non-functional. `push_to_gcs()` never pushed `map_stats`. The GCS `/map_stats` endpoint always returned zeros.

**Fix applied:**  
- Added `map_builder = MapBuilder()` module-level global.  
- Wired `map_builder.ingest(ranges, ...)` into both lidar readers after each scan.  
- Added `"map_stats": map_builder.stats()` to every GCS push payload.  
- Added `map_builder.save(MAP_SAVE_PATH)` before RTL.

---

### BUG-C | HIGH | `RACING_MODE` env var injected by `launch.sh` but never read
**File:** `mission_config.py`  
**Status:** ✅ FIXED

**Problem:**  
`launch.sh` injects `env "RACING_MODE=${RACING_MODE}"` but `mission_config.py` hardcoded `RACING_MODE = True` and never called `os.environ`. The env var had zero effect. Impossible to disable racing mode without editing source.

**Fix applied:**  
```python
import os as _os_cfg
RACING_MODE = _os_cfg.environ.get("RACING_MODE", "1") not in ("0", "false", "False")
```

---

### BUG-D | MEDIUM | Wrong NFZ name reported in breach alert
**File:** `mission_config.py` — `get_nfz_exclusion_check()`  
**Status:** ✅ FIXED

**Problem:**  
```python
if dist < nfz["radius_m"]:
    inside = True
    if breaching_name is None or dist < closest_dist:   # BUG
        breaching_name = nfz["name"]
```
`closest_dist` tracks the global minimum distance including non-breaching zones. If a nearby non-breaching zone reduced `closest_dist` to a small value, later breaching zones with larger distances would never update `breaching_name` — even if they were closer than the first breaching zone. Alert printed wrong NFZ name.

**Fix applied:**  
Added separate `breaching_dist = float("inf")`. Breach candidates compared against `breaching_dist` only:
```python
if dist < breaching_dist:
    breaching_dist = dist
    breaching_name = nfz["name"]
```

---

### BUG-E | MEDIUM | `MapBuilder.ingest()` bypasses accumulator lock — thread race
**File:** `mapping_3d.py` — `MapBuilder.ingest()`  
**Status:** ✅ FIXED

**Problem:**  
```python
if pts:
    self.accumulator._pts.extend(pts)   # no _lock held
    self.accumulator._count += 1        # no _lock held
    self.grid.ingest_points(pts)
```
`MapBuilder._ingest_lock` serialized `ingest()` calls against each other but NOT against `accumulator.points()` and `accumulator.reset()` which both acquire `accumulator._lock`. Concurrent read or reset from another thread (e.g. `save()` at RTL) could corrupt `_pts`.

**Fix applied:**  
```python
with self.accumulator._lock:
    self.accumulator._pts.extend(pts)
    self.accumulator._count += 1
self.grid.ingest_points(pts)
```

---

### BUG-F | LOW | Version string mismatch in final mission banner
**File:** `isr_lidar_mpc.py` — line in `run()` post-RTL  
**Status:** ✅ FIXED

**Problem:**  
`banner("FULL ISR + LiDAR MPC MISSION COMPLETE v12-MPC-v4")` while entry-point header and all other references said `v12-MPC-v5`.

**Fix applied:** Changed to `v12-MPC-v5`.

---

---

## BATCH 2 — Dynamic mission control bugs (found after `feature/dynamic-mission-control` merge, commit `4ca6f70`)

---

### BUG-1 | CRITICAL | Dynamic NFZ never appears on GCS map
**File:** `telemetry_web.py` — `add_nfz()` + `emit_loop()`  
**Status:** ✅ FIXED

**Problem:**  
`add_nfz()` appends only to `dynamic_commands["nfz_queue"]`. The queue is drained into the `POST /lidar_update` response and applied by the mission script — which appends to its process-local copy of `NO_FLY_ZONES`.

However `emit_loop()` pushes:
```python
payload["nfz_zones"] = NO_FLY_ZONES   # GCS process startup snapshot
```
The GCS process and mission script are **separate OS processes**. Each imported their own `NO_FLY_ZONES` list at startup. Appending in the mission script process has no effect on the GCS process's copy. The operator never sees the new NFZ on the map.

**Fix:**  
In `add_nfz()`, also append to the GCS process's `NO_FLY_ZONES`:
```python
NO_FLY_ZONES.append(nfz)                       # update GCS copy → map shows it
with _dyn_cmd_lock:
    dynamic_commands["nfz_queue"].append(nfz)   # queue for mission script
```

---

### BUG-2 | CRITICAL | Dynamic target never appears on GCS map
**File:** `telemetry_web.py` — `add_target()` + `emit_loop()`  
**Status:** ✅ FIXED

**Problem:** Same root cause as BUG-1. `emit_loop()` sends:
```python
payload["secondary_targets"] = SECONDARY_TARGETS   # GCS process startup snapshot
```
Targets appended via `add_target()` are invisible on the GCS map and target panel.

**Fix:**  
```python
SECONDARY_TARGETS.append(target)                    # update GCS copy → map shows it
with _dyn_cmd_lock:
    dynamic_commands["target_queue"].append(target) # queue for mission script
```

---

### BUG-3 | HIGH | Targets added mid-orbit sequence never visited
**File:** `isr_lidar_mpc.py` — `run()` secondary orbit loop  
**Status:** ✅ FIXED

**Problem:**  
```python
sorted_secondaries = sorted(SECONDARY_TARGETS, key=lambda t: t.get("priority", 99))
for i, sec in enumerate(sorted_secondaries, start=1):
    await _do_orbit_phase(sec, label, home_abs_alt)   # each takes 20-30s
```
`sorted_secondaries` is a **frozen snapshot** taken before the loop. If `_apply_dynamic_commands()` appends to `SECONDARY_TARGETS` while SEC-1 is orbiting, the new target is in `SECONDARY_TARGETS` but not in `sorted_secondaries`. It is never visited.

**Fix:** Re-evaluate `SECONDARY_TARGETS` after each orbit phase:
```python
visited = set()
while True:
    remaining = sorted(
        [t for t in SECONDARY_TARGETS if id(t) not in visited],
        key=lambda t: t.get("priority", 99)
    )
    if not remaining:
        break
    sec = remaining[0]
    visited.add(id(sec))
    i = len(visited)
    mission_state["mission_phase"] = f"SEC-{i}"
    label = f"PHASE 4.{i} — SECONDARY TARGET {i}"
    await _do_orbit_phase(sec, label, home_abs_alt)
```

---

### BUG-4 | MEDIUM | `threading.Lock` acquired inside asyncio coroutine — blocks event loop
**File:** `isr_lidar_mpc.py` — `lidar_sim_reader()`  
**Status:** ✅ FIXED

**Problem:**  
```python
with _dyn_lock:    # threading.Lock() — BLOCKING call inside async coroutine
    dynamic_state["pending_events"] = [...]
    active_dyn = list(dynamic_state["pending_events"])
```
`lidar_sim_reader` is an asyncio coroutine running on the event loop thread. A `threading.Lock()` blocks the OS thread synchronously. If the GCS daemon thread holds `_dyn_lock` inside `_apply_dynamic_commands()`, the asyncio event loop freezes — killing telemetry tracking, avoidance loop, and the lidar reader simultaneously.

`_dyn_lock` is held for sub-millisecond operations only, so in practice this rarely triggers. But it is structurally wrong.

**Fix option A — GIL-atomic list copy (simplest):**  
```python
# list.copy() and list-comprehension assignment are each GIL-atomic in CPython.
# No explicit lock needed for these operations.
now_wall      = time.time()
snapshot      = dynamic_state["pending_events"].copy()
still_active  = [e for e in snapshot if e["expire_at"] > now_wall]
dynamic_state["pending_events"] = still_active
active_dyn    = still_active
```

**Fix option B — asyncio.Lock (correct for multi-impl Python):**  
Use `asyncio.Lock()` for the coroutine side; keep `threading.Lock()` for the daemon thread; use `loop.run_in_executor()` to bridge.

---

### BUG-5 | LOW | `inject_event` bearing frame undocumented — likely wrong for operators
**File:** `telemetry_web.py` — `inject_event()` endpoint  
**Status:** ✅ FIXED

**Problem:**  
`bearing_deg` is applied as a **sensor-relative** index into `fake_ranges` (0° = drone forward, clockwise). The endpoint docstring does not state this. Operators familiar with map bearings (0° = North) will inject obstacles in the wrong sector.

**Fix:** Document in the endpoint response and docstring. Optionally accept a `frame` param:
```python
frame = payload.get("frame", "sensor")   # "sensor" or "world"
bearing = float(payload.get("bearing_deg", 0.0))
if frame == "world":
    bearing = (bearing - drone_state["heading"]) % 360
```
`drone_state` must be imported or accessed from the shared state dict.

---

## BATCH 2 — Reviewer false positive (not a bug)

**Claim:** "`global` declaration ignored — assignments create local variables."  
**Verdict:** Incorrect. Python's `global` statement at function top correctly covers all assignments in that function scope. `LIDAR_WARN_DIST = float(...)` after `global LIDAR_WARN_DIST` modifies the module-level variable. Verified against Python language reference §7.12.

---

## BATCH 3 — Runtime issues found during smoke test (commit `d29f048`)

---

### NEW-1 | HIGH | `gz_x500` model has no LiDAR — 0 scans, avoidance blind
**File:** `launch.sh`  
**Status:** ✅ FIXED (commit `d29f048`, LiDAR topic also fixed in `d7cf336`)

**Problem:** Default `PX4_MAKE_MODEL=gz_x500` has no LiDAR sensor. `lidar_gz_reader()` subscribed to `/lidar_360/scan` which never existed. Zero scans received, avoidance loop ran blind for every flight.

**Fix:** Changed default to `gz_x500_lidar_2d`. LiDAR topic also corrected (see NEW-5).

---

### NEW-2 | MEDIUM | `param.Param(drone)` broken in MAVSDK ≥ 1.4
**File:** `isr_lidar_mpc.py`  
**Status:** ✅ FIXED (commit `d29f048`)

**Problem:** `param.Param(drone)` uses internal `_channel` attribute removed in MAVSDK ≥ 1.4. Every run printed `'System' object has no attribute 'channel'`.

**Fix:** Removed broken param block. Kept `drone.action.set_takeoff_altitude()` via stable action API.

---

### NEW-3 | MEDIUM | MAVSDK callback queue overflow at 50 Hz LiDAR
**File:** `isr_lidar_mpc.py`  
**Status:** ✅ FIXED (commit `d29f048`)

**Problem:** Tight `async for pos in drone.telemetry.position()` loop in `_do_orbit_phase` approach consumed every ~50 Hz GPS fix, saturating MAVSDK's gRPC callback queue (hit size 38). Also GCS push at 5 Hz added daemon thread contention.

**Fix:** Added `asyncio.sleep(0.1)` inside approach loop (10 Hz). Reduced GCS push 5 Hz → 2.5 Hz.

---

### NEW-4 | MEDIUM | `_do_orbit_phase` approach loop — no timeout
**File:** `isr_lidar_mpc.py`  
**Status:** ✅ FIXED (commit `d29f048`)

**Problem:** `goto_location` approach had no timeout. CHARLIE-3 (350m away) hung the entire mission indefinitely.

**Fix:** Added 120s `APPROACH_TIMEOUT_S` with warning log and graceful continue to orbit.

---

### NEW-5 | HIGH | Wrong LiDAR topic — `/lidar_360/scan` never published
**File:** `isr_lidar_mpc.py`  
**Status:** ✅ FIXED (commit `d7cf336`, branch `fix/lidar-topic-discovery` merged `5d34b0f`)

**Problem:** `LIDAR_TOPIC = "/lidar_360/scan"` was hardcoded. `gz_x500_lidar_2d` publishes on:
`/world/default/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan`
Topic pattern confirmed from `GZBridge.cpp` source.

**Fix:**
- Updated `LIDAR_TOPIC` to correct path
- `ISR_LIDAR_TOPIC` env var for override
- `_discover_lidar_topic()` auto-discovers via `gz topic -l` if no scan within 8s

**Verified:** 4384 scans received, 2 avoidances triggered, full mission exit 0.

---

## BATCH 4 — Performance / quality fixes (branch `fix/approach-orbit-queue`, NOT YET MERGED)

---

### FIX-1 | HIGH | `goto_location` transit speed ~1-2 m/s — approach timeouts on all targets
**File:** `isr_lidar_mpc.py` — `_do_orbit_phase()`  
**Status:** 🔵 IN BRANCH (fix/approach-orbit-queue — needs merge after test pass)

**Problem:** `goto_location` ignores mission speed. PX4 SITL defaults to ~2 m/s. ALPHA-2 (49m), BRAVO-1 (258m), CHARLIE-3 (243m) all hit 120s timeout. Orbits executed from wrong position.

**Fix:** `await drone.action.set_maximum_speed(SPEED)` before each `goto_location`.

---

### FIX-2 | MEDIUM | Orbit cold-start at radius=0 — never reaches commanded radius
**File:** `isr_lidar_mpc.py` — `_do_orbit_phase()`  
**Status:** 🔵 IN BRANCH (fix/approach-orbit-queue — needs merge after test pass)

**Problem:** Drone arrives within 25m of target centre, then `do_orbit` starts. PX4 begins orbit at radius=0 and spirals outward. In 15-25s dwell the drone only reaches 30-45m of a 50-80m commanded radius.

**Fix:** Fly to orbit entry point (`project_waypoint(t_lat, t_lon, 0.0, t_r)`) before calling `do_orbit`. Arrival threshold changed from `dist < 25m` to `dist <= t_r × 1.3`. Approach timeout raised 120s → 180s.

---

### FIX-3 | LOW | `Queue.put_nowait` race — "Exception in callback" log spam
**File:** `isr_lidar_mpc.py` — `lidar_gz_reader()` `on_scan` callback  
**Status:** 🔵 IN BRANCH (fix/approach-orbit-queue — needs merge after test pass)

**Problem:** Queue drain ran in gz callback thread, put_nowait via `call_soon_threadsafe`. Race: queue could refill between drain and put → `QueueFull` exception logged on every scan burst.

**Fix:** Wrap drain+put in single closure scheduled on the asyncio loop via `call_soon_threadsafe(_drain_and_put)`.

---

### FIX-4 | LOW | `scenarios.json` CRLF line endings
**File:** `scenarios.json`  
**Status:** 🔵 IN BRANCH (fix/approach-orbit-queue — needs merge after test pass)

**Problem:** Windows CRLF corrupted `scenarios.json`, causing git CRLF warnings on every commit.

**Fix:** `sed -i 's/\r//' scenarios.json`. `.gitattributes` prevents recurrence.

---

## Batch 10 — Swarm monitor bugs (2026-05-20, test/phase2-radar-web)

### B10-1 | HIGH | `connection_state()` breaks before connected — drone never shows on ASP
**File:** `src/swarm_monitor.py` — `monitor_drone()`
**Status:** ✅ FIXED

**Problem:** `break` was outside the `if state.is_connected` block — it always broke after
the first event regardless of connection status. If the first event had `is_connected=False`,
`drone_states[idx]["connected"]` stayed False forever. `updateSwarmDrones` filters `!d.connected`
so the drone marker never appeared on the ASP map.

**Fix:** Moved `break` inside `if state.is_connected`. Also redesigned to run `_conn()` as a
continuous task in the gather so disconnects also update the connected flag (required for req 2.14).

### B10-2 | MEDIUM | `VAR=val (compound_cmd)` invalid bash syntax in swarm_launch.sh
**File:** `swarm_launch.sh` line 166
**Status:** ✅ FIXED

**Problem:** `MBC3_MODE="${MBC3_MODE}" (cd ... && python3 swarm_monitor.py)` — variable
assignments before compound commands are not valid bash syntax. With `set -euo pipefail`
this causes undefined behaviour (either syntax error or swarm_monitor not inheriting the var).

**Fix:** `(cd ... && env MBC3_MODE="${MBC3_MODE}" python3 swarm_monitor.py)`

---

## Batch 11 — MBC3_MODE mission time bugs (2026-05-20, test/phase2-radar-web)

### B11 | CRITICAL | Loiter WPs at 40-50m descend drone 450-460m from 500m cruise
**File:** `src/isr_lidar_mpc.py` — loiter_map build (line ~1002)
**Status:** ✅ FIXED

**Problem:** `LOITER_WAYPOINTS` have `altitude_m: 50.0` (LOITER-A) and `40.0` (LOITER-B).
In MBC3_MODE (ALTITUDE=500m), PX4 executes these mission items literally — descend to 50m,
loiter 8s, then next WP at 500m → climb back. Two loiter points × 2 × (450m / 1.4 m/s) ≈ 22 min
wasted. These WPs are ISR (camera surveillance) features, not valid at 500m radar altitude.

**Fix:** When ALTITUDE >= 200.0, set loiter_map = {} to skip injection entirely.

### B12 | HIGH | Secondary orbit altitudes (70-100m) force 400-430m descent from 500m cruise
**File:** `src/isr_lidar_mpc.py` — `_do_orbit_phase()` line ~1450
**Status:** ✅ FIXED

**Problem:** `SECONDARY_TARGETS` orbit altitudes are ISR-specific (70m, 100m, 80m).
`goto_alt = home_abs_alt + t_alt` → in MBC3_MODE, each secondary orbit approach descends
400-430m. With ~3.0 m/s descent and 180s approach timeout, each orbit adds 3-5 min.

**Fix:** `goto_alt = home_abs_alt + max(t_alt, ALTITUDE)` — orbit altitude floored at cruise
altitude. Works for both modes: ISR (max(100, 30)=100 ✓), MBC3 (max(100, 500)=500 ✓).

---

## Batch 12 — Moving target altitude + radar pipeline (2026-05-20, test/phase2-radar-web)

### B12-1 | CRITICAL | Moving targets free-fall despite kinematic=true
**Files:** `worlds/mbc3_radar_moving.sdf`, `worlds/mbc3_isr_moving.sdf`
**Status:** ✅ FIXED

**Problem:** Targets spawned at z=500m (500m AGL) fall to z=-17249m in 62s of simulation.
`<kinematic>true</kinematic>` is insufficient in Gazebo Harmonic DART physics — gravity still
applies to kinematic links unless explicitly disabled. Targets free-fall at ~9.8 m/s².
Even with `kinematic=true`, elevation angle from drone (-90° after falling) puts all targets
outside the radar elevation gate (-5° to +25°) → zero detections.

**Fix:** Added `<gravity>false</gravity>` to every target link in both world SDF files
(5 targets × 2 worlds = 10 link definitions). Verified targets hold at z=500m.

### B12-2 | HIGH | Radar lidar sensors need rendering context (OGRE2)
**Finding:** `lidar` sensor type in Gazebo Harmonic uses OGRE2 rendering engine for ray
casting even for CPU lidar. Without DISPLAY set (headless, WSL launched from PowerShell),
render thread hangs at `Waiting for init` → radar panel topics never publish.
Sensors DO work when Gazebo is launched from user's WSL terminal (WSLg provides display).

**Workaround:** `src/radar_sim.py` — pose-based radar simulator that reads model positions
from `gz topic -e` (physics topics, no rendering), computes 6-panel FOV geometry,
pushes detections to ASP. Works in any environment. Verified: 5/5 targets detected.

---

## Open bug count: 0 | In-branch (not merged): 4 | Fixed: 22 | Total: 26

**Next action:** Test 5-drone swarm: `MBC3_MODE=1 HEADLESS=1 bash swarm_launch.sh`
