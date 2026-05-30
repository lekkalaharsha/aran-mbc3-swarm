# S03 — ISR Mission Core

**Branch:** `feature/6-panel-radar-coverage`  
**Parent:** `main`  
**Status:** ✅ Stable — last verified exit 0 on 2026-05-16

---

## Overview

Main autonomous ISR mission. MAVSDK Python + MPC avoidance + LiDAR (transitioning to radar).

---

## File Map

| File | Purpose | Change rule |
|------|---------|-------------|
| `src/isr_lidar_mpc.py` | Main mission logic | bugs only |
| `src/mpc_controller.py` | MPCEngine + 3 controllers | bugs only |
| `src/mission_config.py` | Single config source | ask before changing |
| `src/pid_controller.py` | Legacy PID (reference) | do not delete |
| `src/mapping_3d.py` | 3D voxel map | low |
| `src/scenarios.json` | 24 sim scenarios | ask before changing |

---

## Mission Phase Sequence

```
STANDBY
  → SURVEY     boustrophedon grid (ROWS × 2 WPs)
  → LOITER     primary orbit at orbit_lat/lon, r=ORBIT_RADIUS
  → SEC-1      approach + orbit secondary target 1
  → SEC-2      approach + orbit secondary target 2
  → SEC-3      approach + orbit secondary target 3
  → RTL        return to home
```

**Do NOT change phase sequence without explicit approval.**

---

## Key Config Values (src/mission_config.py)

| Constant | Value |
|----------|-------|
| HOME_LAT/LON | 47.3977, 8.5456 |
| ALTITUDE | 30 m AGL |
| SPEED | 40 m/s |
| ORBIT_RADIUS | 50 m |
| ORBIT_SPEED | 12 m/s |
| LIDAR_AVOID_DIST | 25 m |
| AVOIDANCE_OFFSET_M | 80 m |

---

## Avoidance Logic (DO NOT TOUCH)

`avoidance_loop()` in `isr_lidar_mpc.py`:
1. Debounce: `DEBOUNCE_COUNT` scans below `LIDAR_AVOID_DIST`
2. Escape bearing: 180° + offset away from obstacle
3. Detour WP: insert temporary WP `AVOIDANCE_OFFSET_M` away
4. Resume: return to mission after detour
5. Climb escape: if all sectors blocked, climb `CLIMB_ESCAPE_M`

---

## Radar Integration (Phase 3)

Current: avoidance uses 360° LiDAR (`isr_lidar_mpc.py` → `radar_gz_reader`)  
Phase 3 target: replace LiDAR with 6-panel radar fusion  
Branch: `test/phase3-radar-avoidance` (not started)

Existing `radar_gz_reader()` in `isr_lidar_mpc.py` already subscribes to 6 panels A-F and fuses into 360° array. Phase 3 connects this to avoidance_loop properly.

---

## Bugs Fixed (in branch, not merged)

| ID | Fix |
|----|-----|
| FIX-1 | `set_maximum_speed(SPEED)` before each `goto_location` |
| FIX-2 | Pre-fly to orbit entry point (N of target at r) |
| FIX-3 | Queue race in gz callback — drain+put atomically |
| FIX-4 | scenarios.json CRLF → LF |

All in `fix/approach-orbit-queue` — must be merged before Phase 3.

---

## Open Tasks

- [ ] Merge `fix/approach-orbit-queue` (FIX-1 to FIX-4) to parent branch
- [ ] Verify exit 0 on `iit_panel_demo` after merge
- [ ] Phase 3: integrate radar fusion into avoidance_loop
</content>
</invoke>