#!/usr/bin/env bash
# ============================================================
#  kill_drone.sh — Terminate one PX4 SITL instance mid-flight
#  Proves MBC-3 req 2.14 (graceful degradation).
#
#  Usage:
#    bash kill_drone.sh 1          # kill drone 1 (safe — keeps Gazebo alive)
#    bash kill_drone.sh 2          # kill drone 2
#    bash kill_drone.sh 3
#    bash kill_drone.sh 4
#
#  Effect on ASP (Phase 2 — req 2.14):
#    swarm_monitor detects disconnect → connected=False
#    ASP marker disappears within 1-2s
#    Status bar shows 4/5 DRONES (amber) instead of 5/5 (green)
#
#  Effect on leader election (Phase 6):
#    Kill the current radar leader (default DRONE-4) to trigger election.
#    leader_election.py detects failure within 2s → Bully algorithm →
#    next highest-index drone becomes radar leader → RADAR LEADER badge
#    on ASP updates → radar_sim.py switches to new leader's position.
#
#  Phase 6 demo sequence:
#    bash kill_drone.sh 4   # kills current leader DRONE-4
#    # ASP: RADAR LEADER changes DRONE-4 → DRONE-3 (amber badge)
#    bash kill_drone.sh 3   # kills DRONE-3
#    # ASP: RADAR LEADER → DRONE-2  (election #2)
#    # Radar tracks continue — proves MBC-3 graceful degradation
#
#  WARNING: Do NOT kill instance 0 — it owns the Gazebo world.
#           Killing instance 0 also kills the world and all other drones.
# ============================================================
set -euo pipefail

RED='\033[0;31m'; GRN='\033[0;32m'; YLW='\033[0;33m'; CYN='\033[0;36m'; RST='\033[0m'

INSTANCE=${1:-}

if [[ -z "${INSTANCE}" ]]; then
    echo -e "${YLW}Usage: bash kill_drone.sh <instance>  (1..4)${RST}"
    echo -e "  1 = DRONE-1  port 14541"
    echo -e "  2 = DRONE-2  port 14542"
    echo -e "  3 = DRONE-3  port 14543"
    echo -e "  4 = DRONE-4  port 14544"
    exit 1
fi

if [[ "${INSTANCE}" == "0" ]]; then
    echo -e "${RED}ERROR: Do not kill instance 0 — it owns the Gazebo world.${RST}"
    echo -e "${YLW}Kill instances 1-4 only. Instance 0 is the world anchor.${RST}"
    exit 1
fi

if ! [[ "${INSTANCE}" =~ ^[1-4]$ ]]; then
    echo -e "${RED}ERROR: Instance must be 1, 2, 3, or 4.${RST}"
    exit 1
fi

echo -e "${CYN}[KILL] Terminating DRONE-${INSTANCE} (PX4 instance ${INSTANCE}, port $((14540 + INSTANCE)))...${RST}"

# PX4 instance N is identified by the -i N flag in the process args
if pkill -SIGTERM -f "bin/px4.*-i ${INSTANCE}" 2>/dev/null; then
    echo -e "${GRN}[KILL] DRONE-${INSTANCE} terminated ✓${RST}"
    echo -e "${YLW}[KILL] ASP will show 4/5 DRONES within 1-2s${RST}"
    echo -e "${YLW}[KILL] Remaining drones continue mission — req 2.14 ✓${RST}"
else
    # Try SIGKILL if SIGTERM didn't find it
    if pkill -SIGKILL -f "bin/px4.*-i ${INSTANCE}" 2>/dev/null; then
        echo -e "${GRN}[KILL] DRONE-${INSTANCE} killed (SIGKILL) ✓${RST}"
    else
        echo -e "${RED}[KILL] Process not found — is the swarm running?${RST}"
        echo -e "  Check: pgrep -a -f 'bin/px4.*-i ${INSTANCE}'"
        exit 1
    fi
fi
