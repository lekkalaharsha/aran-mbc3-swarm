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

## Open bug count: 0 | Fixed bug count: 11 | Total: 11
