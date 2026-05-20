#!/usr/bin/env bash
# ============================================================
#  Aran Technologies — MBC-3 Swarm Launch Script
#  Launches 5 PX4 SITL instances in one Gazebo world.
#
#  Instance ports (MAVSDK connects via udpin://0.0.0.0:1454N):
#    Drone 0: 14540  |  Drone 1: 14541  |  Drone 2: 14542
#    Drone 3: 14543  |  Drone 4: 14544
#
#  Usage:
#    MBC3_MODE=1 bash swarm_launch.sh
#    bash swarm_launch.sh           # ISR mode (30m)
# ============================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_DIR="${PX4_DIR:-${HOME}/PX4-Autopilot}"
PX4_BIN="${PX4_DIR}/build/px4_sitl_default/bin/px4"
PX4_ETC="${PX4_DIR}/build/px4_sitl_default/etc"
MODEL="mbc3_radar_drone"
MBC3_MODE="${MBC3_MODE:-0}"
HEADLESS="${HEADLESS:-0}"

RED='\033[0;31m'; GRN='\033[0;32m'; YLW='\033[0;33m'; CYN='\033[0;36m'; RST='\033[0m'
log()   { echo -e "${CYN}[SWARM]${RST} $*"; }
ok()    { echo -e "${GRN}[SWARM] ✓${RST} $*"; }
warn()  { echo -e "${YLW}[SWARM] ⚠${RST} $*"; }
err()   { echo -e "${RED}[SWARM] ✗${RST} $*"; exit 1; }

# ── World selection ──────────────────────────────────────────
if [[ "${MBC3_MODE}" == "1" ]]; then
    export PX4_GZ_WORLD="${PX4_GZ_WORLD:-mbc3_radar_moving}"
    ALTITUDE=500
else
    export PX4_GZ_WORLD="${PX4_GZ_WORLD:-mbc3_isr_moving}"
    ALTITUDE=30
fi

# ── Spawn positions: 5m grid ─────────────────────────────────
# x,y,z,roll,pitch,yaw
POSES=(
    "0,0,0.135,0,0,0"
    "5,0,0.135,0,0,0"
    "10,0,0.135,0,0,0"
    "0,5,0.135,0,0,0"
    "0,10,0.135,0,0,0"
)

PIDS=()
SESSION_DIR="${SCRIPT_DIR}/logs/swarm_$(date +%Y%m%d_%H%M%S)"
mkdir -p "${SESSION_DIR}"

cleanup() {
    log "Shutdown — stopping all processes..."
    for pid in "${PIDS[@]}"; do
        kill -SIGTERM "${pid}" 2>/dev/null || true
    done
    sleep 2
    pkill -f "bin/px4" 2>/dev/null || true
    pkill -f "gz sim"  2>/dev/null || true
    pkill -f "swarm_monitor" 2>/dev/null || true
    pkill -f "telemetry_web" 2>/dev/null || true
    ok "Shutdown complete"
}
trap cleanup EXIT
trap 'echo ""; err "Interrupted"' INT TERM

echo -e "\n${CYN}╔══════════════════════════════════════════════════╗"
echo    "║   ARAN TECHNOLOGIES — MBC-3 SWARM LAUNCH          ║"
echo    "║   5× mbc3_radar_drone  |  PX4 SITL + Gazebo       ║"
echo -e "╚══════════════════════════════════════════════════╝${RST}\n"
log "World: ${PX4_GZ_WORLD}  |  Altitude: ${ALTITUDE}m  |  Session: ${SESSION_DIR}"

# ── Kill stale ───────────────────────────────────────────────
log "Killing stale processes..."
pkill -9 -f "bin/px4"   2>/dev/null || true
pkill -9 -f "gz sim"    2>/dev/null || true
pkill -9 -f "swarm_mon" 2>/dev/null || true
pkill -9 -f "telemetry_web" 2>/dev/null || true
sleep 2
ok "Clean"

# ── Instance 0: start Gazebo + first drone ───────────────────
log "Launching Instance 0 (starts Gazebo world)..."
PX4_LOG_0="${SESSION_DIR}/px4_0.log"
[[ "${HEADLESS}" == "1" ]] && export HEADLESS=1

(
    cd "${PX4_DIR}"
    make px4_sitl gz_${MODEL}
) >> "${PX4_LOG_0}" 2>&1 &
PIDS+=($!)
log "Instance 0 PID: ${PIDS[0]}  |  log: ${PX4_LOG_0}"

# Wait for Gazebo world ready
log "Waiting for Gazebo world (instance 0)..."
waited=0
while (( waited < 120 )); do
    sleep 1; (( waited++ )) || true
    gz_world=$(gz topic -l 2>/dev/null | grep -m1 "^/world/.*/clock" | sed 's/\/world\///g; s/\/clock//g' || true)
    if [[ -n "${gz_world}" ]]; then
        ok "Gazebo world ready: ${gz_world} (${waited}s)"
        break
    fi
    if (( waited % 15 == 0 )); then
        log "  Still waiting for Gazebo... (${waited}s)"
    fi
done
[[ -z "${gz_world}" ]] && err "Gazebo world not ready after 120s"

# Extra settle time for gz_bridge
sleep 5

# ── Instances 1–4: attach to existing Gazebo ─────────────────
for i in 1 2 3 4; do
    POSE="${POSES[$i]}"
    WORK_DIR="/tmp/px4_swarm_${i}"
    LOG="${SESSION_DIR}/px4_${i}.log"

    log "Launching Instance ${i} at pose (${POSE})..."
    mkdir -p "${WORK_DIR}"

    (
        cd "${WORK_DIR}"
        export PX4_SIM_MODEL="gz_${MODEL}"
        export PX4_GZ_MODEL_POSE="${POSE}"
        # Don't set PX4_GZ_WORLD — px4-rc.gzsim will auto-detect running world
        unset PX4_GZ_WORLD
        "${PX4_BIN}" "${PX4_ETC}" -i "${i}"
    ) >> "${LOG}" 2>&1 &
    PIDS+=($!)
    log "Instance ${i} PID: ${PIDS[-1]}  |  log: ${LOG}"
    sleep 3  # stagger spawns to avoid gz service race
done

ok "All 5 instances launched"

# Wait for all to show "Ready for takeoff"
log "Waiting for all drones to be ready..."
READY_COUNT=0
waited=0
while (( READY_COUNT < 5 && waited < 120 )); do
    sleep 2; (( waited+=2 )) || true
    READY_COUNT=0
    for i in 0 1 2 3 4; do
        logf="${SESSION_DIR}/px4_${i}.log"
        if grep -q "Ready for takeoff\|Startup script returned successfully" "${logf}" 2>/dev/null; then
            (( READY_COUNT++ )) || true
        fi
    done
    log "  Ready: ${READY_COUNT}/5  (${waited}s)"
done
ok "${READY_COUNT}/5 drones ready"

# ── GCS Dashboard ────────────────────────────────────────────
log "Starting GCS dashboard..."
GCS_LOG="${SESSION_DIR}/gcs.log"
(cd "${SCRIPT_DIR}/src" && python3 telemetry_web.py) >> "${GCS_LOG}" 2>&1 &
PIDS+=($!)
sleep 2
ok "GCS live → http://localhost:5000  |  ASP → http://localhost:5000/asp"

# ── Swarm Monitor ────────────────────────────────────────────
log "Starting swarm monitor (connects to all 5 drones)..."
SWARM_LOG="${SESSION_DIR}/swarm.log"
(cd "${SCRIPT_DIR}/src" && env MBC3_MODE="${MBC3_MODE}" python3 swarm_monitor.py) >> "${SWARM_LOG}" 2>&1 &
PIDS+=($!)

echo ""
echo -e "${GRN}╔══════════════════════════════════════════════════╗${RST}"
echo -e "${GRN}║   SWARM RUNNING — 5 drones active                ║${RST}"
echo -e "${GRN}║   GCS:  http://localhost:5000                     ║${RST}"
echo -e "${GRN}║   ASP:  http://localhost:5000/asp                 ║${RST}"
echo -e "${GRN}╚══════════════════════════════════════════════════╝${RST}"
echo ""
log "Session logs: ${SESSION_DIR}/"
log "Press Ctrl-C to stop all"

# Keep alive until Ctrl-C
wait "${PIDS[0]}" || true
