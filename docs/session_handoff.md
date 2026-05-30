# Session Handoff — 2026-05-29

## What we did this session

### 1. PX4 SITL — built and installed
- Cleared 13 GB of old logs to free disk (was 4.7 GB free, needed ~8 GB for build)
- Fixed CMakeLists.txt airframe registration (`t4601` tab-misparsed to literal `t`)
- Ran `install_px4_model.sh` — installed `mbc3_radar_drone` model + airframe into PX4
- Built PX4 SITL: `~/PX4-Autopilot/build/px4_sitl_default/bin/px4` (57 MB) ✓
- Copied 4 custom worlds to `~/PX4-Autopilot/Tools/simulation/gz/worlds/`

### 2. Pre-arm fixes (drone was stuck armable=False)
Two SITL-specific pre-arm failures fixed in `new_drone/airframe/4601_gz_mbc3_radar_drone`:

| Failure | Param | Value |
|---|---|---|
| `system power unavailable` | `CBRK_SUPPLY_CHK` | `894281` |
| `High Accelerometer Bias` | `EKF2_ABL_LIM` | `0.8` |

Drone now arms successfully — `armable=True` confirmed in logs.

**After any airframe change, always run:**
```bash
cp new_drone/airframe/4601_gz_mbc3_radar_drone ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4601_gz_mbc3_radar_drone
cp new_drone/airframe/4601_gz_mbc3_radar_drone ~/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/airframes/4601_gz_mbc3_radar_drone
chmod +x ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4601_gz_mbc3_radar_drone
chmod +x ~/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/airframes/4601_gz_mbc3_radar_drone
rm -f ~/PX4-Autopilot/build/px4_sitl_default/rootfs/eeprom/parameters
```

### 3. Climb stall fix (drone hovered at 19.8m instead of 30m)
**Root cause:** `goto_location(HOME_LAT, HOME_LON, ...)` fallback in `src/isr_lidar_mpc.py` fired when the drone was already at HOME horizontal position → PX4 declared waypoint reached instantly → levelled off at 19.8m.

**Fix applied in `src/isr_lidar_mpc.py`:**
- Replaced the time-based `elapsed > 30s AND alt < 20m` trigger with rate-based stall detection: `climb_rate < 0.05 m/s` for 12+ seconds
- When rescue fires, uses `drone_state["lat"] + 0.00045` (≈50m north) instead of HOME — gives PX4 a real horizontal target so it climbs while navigating
- Both `goto_location` call sites fixed (takeoff fallback at line ~1290 + climb rescue at line ~1336)

**Status at session end:** Fix applied, `launch.sh` restarted, monitor running. **Verify in next session.**

### 4. record_demo.sh updated for swarm
`record_demo.sh` now:
1. Kills stale processes
2. Opens gnome-terminal with `swarm_launch.sh` (5-drone SITL + Gazebo + GCS)
3. Polls `http://localhost:5000` up to 300s for GCS
4. Opens Firefox to swarm GCS dashboard
5. Records 300s via x11grab → `~/mbc3_phase0_demo.mp4`
6. Kills DRONE-2 at T+150s (leader failover demo)

---

## State at session end

| Component | Status |
|---|---|
| PX4 binary | ✓ Built at `~/PX4-Autopilot/build/px4_sitl_default/bin/px4` |
| mbc3_radar_drone model | ✓ Installed in PX4 |
| pre-arm (CBRK + EKF2) | ✓ Fixed and verified — drone arms |
| Climb stall | ✓ Fixed in code, **re-test needed** |
| record_demo.sh | ✓ Ready for swarm recording |
| Phase 0 video | `~/mbc3_phase0_demo.mp4` exists (old fly_demo.sh version, 12M) |
| Deadline | **31 May 2026** — 2 days |

---

## What to do next

### Priority 1 — Verify climb fix
```bash
bash launch.sh
```
Expected: drone arms, climbs continuously to 30m without WARNING messages, reaches `Cruise altitude reached — alt=29.Xm`. If `goto_location rescue` fires at all, it should be at a genuine stall, not at 19.8m.

### Priority 2 — Record swarm demo video
Once climb is verified:
```bash
bash record_demo.sh
```
This takes ~8 min total (3 min swarm startup + 5 min recording). Watch for:
- `GCS reachable at T+XXs` 
- Firefox opens to `http://localhost:5000`
- `DRONE-2 killed` at T+150s
- `~/mbc3_phase0_demo.mp4` saved

### Priority 3 — Submit to IAF portal (deadline 31 May 2026)
Files to submit:
- `~/mbc3_phase0_demo.mp4`
- `competition/Final_Vision_Document_for_MBC_3_22Apr26.pdf`
- `competition/Registration_form_MBC_3_final.pdf`

---

## Key file paths

| File | Purpose |
|---|---|
| `src/isr_lidar_mpc.py` | Single-drone ISR mission — climb fix here |
| `src/swarm_mission.py` | 5-drone swarm mission |
| `new_drone/airframe/4601_gz_mbc3_radar_drone` | PX4 airframe params (source of truth) |
| `launch.sh` | Single-drone demo launcher (972 lines, STEP 1–6) |
| `swarm_launch.sh` | 5-drone swarm launcher |
| `record_demo.sh` | Screen recorder for demo video |
| `tools/pre_demo_check.sh` | Pipeline validator (7/7 checks) |
| `docs/bugs.md` | Bug register — add before fixing |

## Known open items (not fixed this session)
- `Radar=OFF` in launch.sh STEP 6 — `ros_gz_bridge` not installed, `aeris10_driver` exits; radar avoidance falls back to LiDAR-only mode. Not blocking for demo.
- `WARNING  Still low at Xm` stall check at 15s, alt < 15m — fires a `takeoff()` retry; harmless but noisy.
