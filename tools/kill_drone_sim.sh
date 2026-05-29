#!/usr/bin/env bash
# kill_drone_sim.sh — Simulate mid-flight drone failure for redistribution demo.
#
# Kills the PX4 SITL process + MAVSDK server for one drone instance.
# This drops the MAVLink heartbeat, causing MAVSDK to raise a gRPC exception
# inside run_mission() → redistribution logic in swarm_mission.py fires.
#
# USAGE:
#   bash ~/aran_mbc/tools/kill_drone_sim.sh [DRONE_IDX] [DELAY_SEC] [LOG_DIR]
#
#   DRONE_IDX  : 0–4  (default: 2 — center drone, max impact on coverage)
#   DELAY_SEC  : seconds to wait after script starts (default: 30)
#   LOG_DIR    : path to swarm session log dir to tail REDISTRIB lines (optional)
#
# EXAMPLES:
#   bash kill_drone_sim.sh               # kill DRONE-2 after 30s
#   bash kill_drone_sim.sh 1 20          # kill DRONE-1 after 20s
#   bash kill_drone_sim.sh 3 45 ~/aran_mbc/logs/swarm_20260529_120000
#
# DEMO SEQUENCE:
#   T1: bash ~/aran_mbc/swarm_launch.sh
#   T2: bash ~/aran_mbc/tools/kill_drone_sim.sh 2 30
#   Watch T1 for:  "[REDISTRIB] DRONE-2 failed at WP X/Y ..."
#                  "[DRONE-1] Executing N redistributed WPs ..."

set -eo pipefail

DRONE_IDX="${1:-2}"
DELAY_SEC="${2:-30}"
LOG_DIR="${3:-}"

BASE_GRPC=50050
PID_FILE="/tmp/px4_swarm_pid_${DRONE_IDX}"

RED='\033[0;31m'; YLW='\033[0;33m'; GRN='\033[0;32m'; CYN='\033[0;36m'; RST='\033[0m'

banner() { echo -e "\n${CYN}══════════════════════════════════════════${RST}"; echo -e "${CYN}  $1${RST}"; echo -e "${CYN}══════════════════════════════════════════${RST}"; }
ok()     { echo -e "${GRN}[$(date +%H:%M:%S)] ✓  $*${RST}"; }
warn()   { echo -e "${YLW}[$(date +%H:%M:%S)] ⚠  $*${RST}"; }
err()    { echo -e "${RED}[$(date +%H:%M:%S)] ✗  $*${RST}"; }
info()   { echo -e "${CYN}[$(date +%H:%M:%S)]    $*${RST}"; }

banner "DRONE FAILURE SIMULATOR — MBC-3 Redistribution Test"
echo ""
info "Target drone  : DRONE-${DRONE_IDX}"
info "Kill delay    : ${DELAY_SEC}s from now"
info "PID file      : ${PID_FILE}"
info "MAVSDK gRPC   : port $((BASE_GRPC + DRONE_IDX))"
echo ""
info "This will simulate hardware failure mid-flight."
info "Expected response: swarm_mission.py redistributes DRONE-${DRONE_IDX}'s"
info "remaining waypoints to adjacent active drones via D2D REASSIGN."
echo ""

# ── Sanity checks ─────────────────────────────────────────────────────────────
if (( DRONE_IDX < 0 || DRONE_IDX > 4 )); then
    err "DRONE_IDX must be 0–4, got ${DRONE_IDX}"
    exit 1
fi

if [[ ! -f "${PID_FILE}" ]]; then
    err "PID file not found: ${PID_FILE}"
    err "Is swarm_launch.sh running? It writes /tmp/px4_swarm_pid_N at startup."
    exit 1
fi

PX4_PID=$(cat "${PID_FILE}")
if ! kill -0 "${PX4_PID}" 2>/dev/null; then
    err "PX4 process PID ${PX4_PID} (from ${PID_FILE}) is not running."
    err "Either swarm already finished, or drone ${DRONE_IDX} already died."
    exit 1
fi
ok "DRONE-${DRONE_IDX} PX4 process alive: PID ${PX4_PID}"

# ── Countdown ─────────────────────────────────────────────────────────────────
echo ""
info "Killing DRONE-${DRONE_IDX} in ${DELAY_SEC} seconds ..."
info "Ctrl-C to abort."
echo ""

for (( t=DELAY_SEC; t>0; t-- )); do
    if (( t <= 5 )); then
        printf "\r${RED}  KILL in %2ds ...${RST}" "${t}"
    else
        printf "\r${YLW}  Kill in %2ds ...${RST}" "${t}"
    fi
    sleep 1
done
echo ""
echo ""

# ── Kill sequence ─────────────────────────────────────────────────────────────
banner "KILLING DRONE-${DRONE_IDX}"

# 1. Kill PX4 SITL process (drops MAVLink heartbeat immediately)
echo -e "${RED}[$(date +%H:%M:%S)] Sending SIGKILL to PX4 PID ${PX4_PID} ...${RST}"
if kill -9 "${PX4_PID}" 2>/dev/null; then
    ok "PX4 SITL process ${PX4_PID} killed"
else
    warn "PX4 PID ${PX4_PID} already gone"
fi

# 2. Kill MAVSDK server for this drone (holds gRPC port 5005N)
MAVSDK_GRPC_PORT=$((BASE_GRPC + DRONE_IDX))
echo -e "${RED}[$(date +%H:%M:%S)] Killing mavsdk_server on gRPC port ${MAVSDK_GRPC_PORT} ...${RST}"
if pkill -9 -f "mavsdk_server.*-p ${MAVSDK_GRPC_PORT}" 2>/dev/null; then
    ok "mavsdk_server (port ${MAVSDK_GRPC_PORT}) killed"
else
    # Try by UDP port (swarm_launch binds udpin://0.0.0.0:1454N)
    UDP_PORT=$((14540 + DRONE_IDX))
    if pkill -9 -f "mavsdk_server.*${UDP_PORT}" 2>/dev/null; then
        ok "mavsdk_server (udp ${UDP_PORT}) killed"
    else
        warn "mavsdk_server not found — MAVSDK may detect failure via heartbeat timeout (~5s)"
    fi
fi

# 3. Remove PID file so script can be re-run cleanly
rm -f "${PID_FILE}"
ok "PID file removed: ${PID_FILE}"

echo ""
ok "DRONE-${DRONE_IDX} KILLED at $(date +%H:%M:%S)"
echo ""
echo -e "${YLW}  Waiting for redistribution response from swarm_mission.py ...${RST}"
echo -e "${YLW}  MAVSDK detects heartbeat loss in ~3–5s, then redistribution fires.${RST}"
echo ""

# ── Monitor redistribution output ────────────────────────────────────────────
# If LOG_DIR provided, tail the swarm_mission log for redistribution lines.
# Otherwise, watch stdout of the running swarm_launch process.
MISSION_LOG=""
if [[ -n "${LOG_DIR}" ]] && [[ -d "${LOG_DIR}" ]]; then
    MISSION_LOG="${LOG_DIR}/swarm_mission.log"
fi

# Auto-detect latest swarm session log if not given
if [[ -z "${MISSION_LOG}" ]]; then
    LATEST_LOG_DIR=$(ls -td ~/aran_mbc/logs/swarm_* 2>/dev/null | head -1 || true)
    if [[ -n "${LATEST_LOG_DIR}" ]]; then
        MISSION_LOG="${LATEST_LOG_DIR}/swarm_mission.log"
    fi
fi

if [[ -n "${MISSION_LOG}" ]] && [[ -f "${MISSION_LOG}" ]]; then
    info "Tailing ${MISSION_LOG} for redistribution events (Ctrl-C to stop):"
    echo ""
    timeout 60 tail -f "${MISSION_LOG}" 2>/dev/null | \
        grep --line-buffered -E "REDISTRIB|FAILED|EXTRA_WPS|redistributed|REASSIGN|run_mission|DRONE-${DRONE_IDX}" \
        | while IFS= read -r line; do
            echo -e "  ${GRN}>>>${RST} ${line}"
        done || true
    echo ""
    ok "Monitor complete."
else
    info "No mission log found. Watch swarm_launch.sh terminal for:"
    echo ""
    echo -e "  ${GRN}[REDISTRIB] DRONE-${DRONE_IDX} failed at WP X/Y. N WPs → DRONE-A:n, DRONE-B:m${RST}"
    echo -e "  ${GRN}[DRONE-A]   Executing N redistributed WPs from failed drone(s)${RST}"
    echo -e "  ${GRN}[DRONE-B]   Executing M redistributed WPs from failed drone(s)${RST}"
    echo ""
    info "If you see those lines, redistribution is working correctly."
fi

echo ""
banner "SIMULATION COMPLETE"
echo -e "  Drone killed  : DRONE-${DRONE_IDX}"
echo -e "  Time          : $(date +%H:%M:%S)"
echo -e "  Next step     : Verify swarm_launch terminal shows REDISTRIB lines"
echo -e "  Then          : Record demo video showing redistribution in action"
echo ""
