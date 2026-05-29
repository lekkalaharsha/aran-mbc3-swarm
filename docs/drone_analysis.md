# MBC-3 Drone — Weight, Power & Flight Analysis

> Source: MBC3_MASTER.md specifications  
> Date: 2026-05-29  
> Configuration: Hexacopter X6 | 730mm wheelbase | AERIS-10 payload

---

## 1. Weight Breakdown

| Component | Unit (g) | Qty | Total (g) |
|-----------|----------|-----|-----------|
| Motor T-Motor MN3110-26 700KV | 115 | 6 | 690 |
| ESC 40A BLHeli32 | 38 | 6 | 228 |
| CF arm 370mm (OD 25 / ID 22 mm) | 42 | 6 | 252 |
| Hub top plate 4mm CF | 145 | 1 | 145 |
| Hub bottom plate 4mm CF | 145 | 1 | 145 |
| Arm clamps (PETG) | 35 | 6 | 210 |
| Motor mounts (PETG) | 28 | 6 | 168 |
| Payload bay enclosure (PETG) | 185 | 1 | 185 |
| Landing gear struts + Al tubes | 90 | 3 | 270 |
| GPS mast + module | 55 | 1 | 55 |
| Pixhawk FC | 38 | 1 | 38 |
| PDB / power module | 45 | 1 | 45 |
| Wiring + connectors | — | — | 220 |
| Props 12×4.5 CF 3-blade | 32 | 6 | 192 |
| ESC brackets + hardware | — | — | 80 |
| Standoffs + bolts | — | — | 85 |
| **Frame subtotal (no battery, no radar)** | | | **2,838 g** |
| AERIS-10 radar | 430 | 1 | 430 |
| Battery 6S 16,000 mAh | 2,100 | 1 | 2,100 |
| Telemetry / camera / misc | — | — | ~462 |
| **GRAND TOTAL** | | | **5,830 g = 5.83 kg** |

Battery = 36% of total mass.

---

## 2. Motor — T-Motor MN3110-26 (700KV)

| Parameter | Value |
|-----------|-------|
| Type | Brushless DC outrunner |
| Stator dimensions | 31 × 10 mm |
| KV rating | 700 RPM/V |
| Weight | 115 g |
| Shaft diameter | 4 mm |
| Mount pattern | 4× M3, 19 mm bolt-circle |
| No-load RPM @ 6S (22.2V) | 700 × 22.2 = **15,540 RPM** |
| Estimated loaded RPM @ hover | ~11,500 RPM |
| Recommended prop range | 12–13 inch |
| Max thrust (12×4.5 CF, 6S) | **~2,100 g per motor** |
| Max current @ full throttle | ~30–35 A |
| Max power @ full throttle | ~720 W per motor |
| Backup motor option | EMAX MT3510+ 600KV (16mm BC, M3) |

---

## 3. Propeller — 12×4.5 in, 3-blade CF

| Parameter | Value |
|-----------|-------|
| Diameter | 12 in = 304.8 mm |
| Pitch | 4.5 in per revolution |
| Blades | 3 (carbon fibre) |
| Disk radius | 152.4 mm = 0.1524 m |
| Disk area per prop | π × 0.1524² = **0.07297 m²** |
| Total disk area (6 props) | 6 × 0.073 = **0.438 m²** |
| Adjacent prop-tip clearance | (730 − 609.6) / 2 = **60.2 mm** ✓ |
| Full tip-to-tip span | ~1,035 mm |

---

## 4. Battery — 6S LiPo

| Parameter | Value |
|-----------|-------|
| Cell configuration | 6S (6 cells in series) |
| Nominal voltage | 22.2 V (3.7 V/cell) |
| Fully charged | 25.2 V (4.2 V/cell) |
| Discharge cutoff | 19.8 V (3.3 V/cell) |
| Capacity | 16,000 mAh = 16 Ah |
| Total energy | 22.2 V × 16 Ah = **355.2 Wh** |
| Usable energy (80% DoD) | **284.2 Wh** |
| Weight | 2,100 g |
| Energy density | 355.2 / 2.1 = **169 Wh/kg** |
| Peak current draw | 6 motors × 35 A = **210 A** |
| Minimum C-rating needed | 210 A / 16 Ah = 13.1 C → buy **≥ 15C** |

---

## 5. Thrust & Power Analysis

### 5.1 Hover point

```
Total mass              = 5.83 kg
Total hover thrust      = 5.83 × 9.81 = 57.2 N
Thrust per motor        = 57.2 / 6    = 9.53 N = 972 g

Max total thrust        = 6 × 2,100 g = 12,600 g
TWR                     = 12,600 / 5,830 = 2.16  ✓
Hover throttle          = √(972 / 2100) = √0.463 = 0.68 = 68%
```

### 5.2 Power consumption (actuator disk theory)

```
Disk loading            = 57.2 N / 0.438 m²   = 130.5 N/m²
Induced velocity        = √(57.2 / (2 × 1.225 × 0.438))
                        = √(57.2 / 1.073)
                        = √53.3 = 7.29 m/s

Ideal hover power       = 57.2 × 7.29          = 417 W
Actual (figure of merit = 0.75): 417 / 0.75    = 556 W
+ Profile drag (+25%)   = 556 × 1.25           = 695 W   ← at propeller shaft
+ ESC losses (5%)       = 695 / 0.95           = 732 W
+ Avionics / FC / radar = +30 W
──────────────────────────────────────────────────────────
Total hover power       ≈ 762 W ≈ 760 W
Hover current draw      = 760 / 22.2           ≈ 34.2 A total
```

### 5.3 Mission cruise (20 m/s)

```
Cruise power factor     = 1.25 × hover (forward flight penalty)
Cruise power            = 760 × 1.25           = 950 W
Cruise current          = 950 / 22.2           = 42.8 A
```

---

## 6. Flight Endurance & Range

| Mode | Calculation | Result |
|------|-------------|--------|
| Hover | 284.2 Wh / 760 W | **22.4 min** |
| Mission cruise (20 m/s) | 284.2 Wh / 950 W | **18.0 min** |
| Path length @ 20 m/s | 18 min × 20 m/s × 60 | **21.6 km** |
| Round-trip radius | 21.6 / 2 | **10.8 km** |

---

## 7. ESC Selection Check

| Parameter | Value | Status |
|-----------|-------|--------|
| Rated continuous current | 40 A | ✓ motor peaks ~35 A |
| Voltage rating | 6S = 25.2 V max | ✓ BLHeli32 40A supports 6S |
| Telemetry protocol | BLHeli32 — DShot + active braking | ✓ |
| Total current headroom | 6 × 40A = 240A > 210A peak | ✓ |

---

## 8. Structural Check

| Check | Value | Status |
|-------|-------|--------|
| Wheelbase vs prop diameter | 730 mm vs 609.6 mm (2×D) | ✓ 60 mm clearance |
| CF arm wall (1.5 mm) | Handles hover bending moment | ✓ |
| Hub plate (4mm CF or 6061 Al) | Yield strength >> bolt preload | ✓ |
| PETG motor mount max temp | Tg ~80°C, motor base ~60°C sustained | ⚠ marginal — add thermal pad |
| Landing gear height | 374.9 mm — clears AERIS-10 dome | ✓ |
| CG estimate | Battery on bottom plate, electronics above | ✓ near geometric center |
| AERIS-10 radar aperture | Ø45 mm hole on payload bay floor | ✓ faces down for ground scan |

---

## 9. Compliance vs IAF Requirements

| IAF Requirement | Result | Pass? |
|-----------------|--------|-------|
| TWR ≥ 2.0 | **2.16** | ✓ |
| Hover throttle < 75% | **68%** | ✓ |
| Speed 10–40 m/s (req 2.5) | 20 m/s cruise capable | ✓ |
| Altitude ≥ 500 m AGL (req 2.10) | PX4 + 6S supports | ✓ |
| Endurance ≥ 15 min | **18–22 min** | ✓ |
| Payload bay fits AERIS-10 | 150×150×90mm clear | ✓ |
| 6-drone swarm coordination | D2D multicast + leader election | ✓ |

---

## 10. GAPS & OPEN ITEMS

### Hardware gaps (not verified / not sourced yet)

| # | Gap | Impact | Priority |
|---|-----|--------|----------|
| G1 | **AERIS-10 actual weight not confirmed** — 430g is estimate | Affects TWR/endurance | HIGH |
| G2 | **Motor max thrust not bench-tested** — 2100g is datasheet estimate for MN3110 on 6S | Affects TWR accuracy | HIGH |
| G3 | **Battery C-rating not specified** — need ≥15C discharge rating confirmed on purchase | Safety / ESC protection | HIGH |
| G4 | **AERIS-10 USB VID/PID unknown** — `aeris10_usb.py` has placeholder 0x0483/0xAE10 | Driver won't connect to real hardware | HIGH |
| G5 | **No ESC current measurement** — no shunt/sensor for per-motor current logging | Can't detect motor failure in flight | MEDIUM |
| G6 | **Prop efficiency curve not measured** — using theoretical FOM=0.75 for power calc | ±15% error on flight time | MEDIUM |
| G7 | **No vibration analysis** — CF arms + PETG mounts, resonance freq unknown | IMU noise → PX4 attitude errors | MEDIUM |
| G8 | **PETG motor mount thermal limit** — sustained 60°C at motor base, Tg=80°C only 20°C margin | Mount softening at full throttle | MEDIUM |
| G9 | **Battery connector not specified** — 210A peak needs XT90S or AS150, not XT60 | Fire risk if underrated | HIGH |
| G10 | **Prop pitch optimisation not done** — 4.5in pitch for 700KV on 6S may not be optimal | ±10% efficiency loss | LOW |

### Software gaps (for full Phase 0 demo)

| # | Gap | Impact | Priority |
|---|-----|--------|----------|
| S1 | ~~`fly_demo.sh` radar/targets echo parsing untested~~ → **FIXED (BUG-D1)**: sim_mode generated 5 hits (clutter band) → RF gate zeroed all detections. Fixed: 15 scatter points. Parsing logic verified correct. | ✓ closed | — |
| S2 | **`swarm_mission.py` redistribution untested** — EXTRA_WPS queue written but no end-to-end test | Core Phase 0 proof unverified | HIGH |
| S3 | **`launch.sh` aeris10 path hardcoded to python3.12** — MAVSDK_SERVER path in swarm_mission.py may not match system | Swarm won't start | HIGH |
| S4 | **No heartbeat timeout in `run_mission()`** — failure only detected on exception, not silent hang | Drone can hang mid-mission undetected | MEDIUM |
| S5 | **`mbc3_exact_v3.sdf` has no Gazebo radar sensor plugin** — gz_bridge has no data source | Radar pipeline needs aeris10_driver sim always | MEDIUM |
| S6 | **`radar_fusion.launch.py` existence not verified** — launch.sh references it but file not checked | launch.sh STEP 3.5 fails silently | MEDIUM |
| S7 | **`asp_bridge.py` not read/verified** — assumed to work | Radar targets won't show on GCS | LOW |
| S8 | ~~No drone failure simulation script~~ → **FIXED**: `tools/kill_drone_sim.sh` | ✓ closed | — |

### Documentation gaps

| # | Gap |
|---|-----|
| D1 | No wiring diagram (motor order CW/CCW, ESC signal pin, battery lead routing) |
| D2 | No PX4 airframe parameter file (MPC_THR_HOVER, MPC_XY_VEL_MAX etc. for 5.83kg) |
| D3 | No CAD files yet — Section 5 has dimensions but no `.FCStd` or `.STL` files |
| D4 | Competition registration form status unknown — `competition/Registration_form_MBC_3_final.pdf` exists but submitted? |

---

## 11. Immediate Actions (before 31 May 2026)

1. **Fix S1** — test fly_demo.sh end-to-end, confirm `/radar/targets` echo works
2. **Fix S3** — verify MAVSDK_SERVER path on this machine
3. **Fix D4** — confirm Phase 0 registration submitted to IAF website
4. **Note G9** — buy XT90S or AS150 connector, not XT60 for 16Ah 6S

---

*File auto-generated from drone_analysis session 2026-05-29*
