# MBC-3 Submission Checklist — Aran Technologies / Boson Motors
**Contact:** L. Harsha Vardhan Naidu | aranrobotics@gmail.com | +91 72888 40612  
**Phase I presentations:** New Delhi, 13–24 July 2026

---

## Phase 0 — Registration (SUBMITTED ✅ — 31 May 2026)

| # | Item | File | Status |
|---|------|------|--------|
| 1 | Registration form | `Registration_form_MBC_3_final.pdf` | ✅ Submitted |
| 2 | Proposal (Doc 1, ≤500 words) | `doc1_proposal.md` → `MBC3_Proposal.pdf` | ✅ Submitted |
| 3 | Products & Tech brief (Doc 2, ≤300 words) | `doc2_products_tech.md` | ✅ Submitted |
| 4 | Competitions / forums (Doc 3, ≤300 words) | `doc3_competitions.md` | ✅ Submitted |
| 5 | Additional info (Doc 4, ≤300 words) | `doc4_additional.md` | ✅ Submitted |
| 6 | Vision document | `Final_Vision_Document_for_MBC_3_22Apr26.pdf` | ✅ Submitted |

---

## Phase I — Presentation Package (Due: July 2026)

| # | Item | File | Status |
|---|------|------|--------|
| 1 | Single-drone ISR demo video (4 min) | `../mbc3_single_drone_demo.mp4` (24 MB, 1920×1080) | ✅ Ready |
| 2 | Swarm demo video (5 drone, 5 min) | Run `bash record_demo.sh` → `~/mbc3_phase0_demo.mp4` | ⬜ Not yet recorded |
| 3 | Vision document (updated if needed) | `Final_Vision_Document_for_MBC_3_22Apr26.pdf` | ✅ Ready |
| 4 | Registration form | `Registration_form_MBC_3_final.pdf` | ✅ Ready |
| 5 | Proposal PDF | `MBC3_Proposal.pdf` | ✅ Ready |
| 6 | Live simulation demo (Gazebo + GCS) | Run `bash launch.sh` on presentation day | ✅ Verified exit-0 |
| 7 | Pre-flight check pass (7/7) | Run `bash tools/pre_demo_check.sh` | ✅ Verified |

---

## Demo Videos — Recording Commands

```bash
# Single-drone ISR demo (4 min, Gazebo left + GCS right)
# Requires: wmctrl  →  sudo apt install -y wmctrl
bash ~/Documents/aran_mbc/record_single_drone.sh
# Output: ~/Documents/aran_mbc/mbc3_single_drone_demo.mp4

# Swarm demo (5 drones, 5 min)
bash ~/Documents/aran_mbc/record_demo.sh
# Output: ~/mbc3_phase0_demo.mp4
```

---

## Verified Simulation Runs (2026-05-30)

### Single-drone ISR
```
PHASE 1 — Survey upload       ✅  11/11 WPs
PHASE 2 — ISR survey          ✅  11/11 WPs, 0 avoidances
PHASE 3 — PRIMARY orbit       ✅  radius locked 50.0m ±0.5m
PHASE 5 — RTL                 ✅  landed, map saved
```
Release tag: `v1.0.0-single-drone`

### Swarm (5 drones)
- 344 radar tracks processed
- Leader kill → bully election → new leader in <2s
- 4/4 surviving drones operational post-kill

---

## Pre-Presentation Day Checklist

```
[ ] sudo apt install -y wmctrl          # window tiler for demo recording
[ ] bash tools/pre_demo_check.sh        # 7/7 checks must pass
[ ] df -h ~/Documents/aran_mbc/         # confirm ≥10 GB free (logs accumulate)
[ ] bash record_single_drone.sh         # re-record if demo updated
[ ] xdg-open mbc3_single_drone_demo.mp4 # verify video quality
[ ] Check GCS at http://localhost:5000   # all telemetry cards live
[ ] Kill all processes after dry run     # bash kill_drone.sh
```

---

## Key Paths

| Resource | Path |
|----------|------|
| Project root | `~/Documents/aran_mbc/` |
| Single-drone launcher | `launch.sh` |
| Swarm launcher | `swarm_launch.sh` |
| Demo video (single) | `mbc3_single_drone_demo.mp4` |
| Mission log | `logs/mission.log` |
| Bug register | `docs/bugs.md` |
| GCS (single) | `http://localhost:5000` |
| GCS (swarm) | `http://localhost:5001` |

---

*Last updated: 2026-05-30*
