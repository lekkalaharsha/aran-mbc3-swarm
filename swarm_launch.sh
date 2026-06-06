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

# ── Spawn positions: 10m grid ──────────────────────────────────────
# x,y,z,roll,pitch,yaw
# z=0.195: skid bottom sits at z=-0.186 from base_link (S3 LG fix: 145→200mm clearance)
# → spawn z must be ≥0.186. 0.195 gives 9mm clearance above ground.
POSES=(
    "0,0,0.195,0,0,0"
    "10,0,0.195,0,0,0"
    "20,0,0.195,0,0,0"
    "0,10,0.195,0,0,0"
    "0,20,0.195,0,0,0"
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
pkill -9 -f "bin/px4"      2>/dev/null || true
pkill -9 -f "gz sim"       2>/dev/null || true
pkill -9 -f "swarm_mon"    2>/dev/null || true
pkill -9 -f "telemetry_web" 2>/dev/null || true
pkill -9 -f "mavsdk_server" 2>/dev/null || true  # stale server holds UDP 14540-14544
rm -f /tmp/px4_swarm_pid_*                        # stale PID files confuse leader_election
# Remove stale PX4 working dirs so airframe param defaults take effect on each boot
# (PX4 SITL stores params in /tmp/px4_swarm_N/rootfs/eeprom; stale files override airframe)
rm -rf /tmp/px4_swarm_1 /tmp/px4_swarm_2 /tmp/px4_swarm_3 /tmp/px4_swarm_4
sleep 2
ok "Clean"

# ── Instance 0: start Gazebo + first drone ───────────────────
log "Launching Instance 0 (starts Gazebo world)..."
PX4_LOG_0="${SESSION_DIR}/px4_0.log"
[[ "${HEADLESS}" == "1" ]] && export HEADLESS=1

(
    cd "${PX4_DIR}"
    PX4_GZ_MODEL_POSE="${POSES[0]}" make px4_sitl gz_${MODEL}
) >> "${PX4_LOG_0}" 2>&1 &
PIDS+=($!)
echo "${PIDS[0]}" > /tmp/px4_swarm_pid_0
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
    echo "${PIDS[-1]}" > /tmp/px4_swarm_pid_${i}
    log "Instance ${i} PID: ${PIDS[-1]}  |  log: ${LOG}"
    sleep 8  # stagger spawns — enough for gz service + model load to complete
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

# ── Shared D2D HMAC key — generated once per session, exported to all children ──
# All drone processes must share the same key for inter-drone message authentication.
export D2D_HMAC_KEY="${D2D_HMAC_KEY:-$(openssl rand -hex 32)}"
log "D2D_HMAC_KEY set ($(echo "${D2D_HMAC_KEY}" | wc -c | tr -d ' ')  hex chars)"

# ── GCS Dashboard (swarm_telemetry_web.py — 5-drone map + radar panel) ───────
log "Starting Swarm Command Center dashboard..."
GCS_LOG="${SESSION_DIR}/gcs.log"
(cd "${SCRIPT_DIR}/src" && python3 swarm_telemetry_web.py) >> "${GCS_LOG}" 2>&1 &
PIDS+=($!)
sleep 2
ok "Swarm GCS live → http://localhost:5000"
ok "  Map: 5 drone markers + trails + sector polygons + NFZ"
ok "  Radar: 6-panel AERIS-10 polar SVG per drone"
ok "  Events: redistribution + failure log"

# ── Swarm Mission — arm + climb all 5, then sequential mission one by one ──
# Starts mavsdk_server per drone (grpc 50050-50054) for isolated control.
# Pushes positions to GCS /asp_update directly — no separate swarm_monitor needed.
log "Starting swarm_mission (5 drones: arm→climb→survey→orbit→land sequentially)..."
MISSION_LOG="${SESSION_DIR}/swarm_mission.log"
(cd "${SCRIPT_DIR}/src" && env MBC3_MODE="${MBC3_MODE}" D2D_HMAC_KEY="${D2D_HMAC_KEY}" python3 -u swarm_mission.py) >> "${MISSION_LOG}" 2>&1 &
PIDS+=($!)
sleep 3
ok "Swarm mission started  |  log: ${MISSION_LOG}"

# ── Swarm Monitor — telemetry only (no MAVSDK commands) ─────────────

# ── Leader Election — Bully algorithm, highest-index connected drone wins ─
log "Starting leader election daemon..."
ELECT_LOG="${SESSION_DIR}/election.log"
(cd "${SCRIPT_DIR}/src" && python3 -u leader_election.py) >> "${ELECT_LOG}" 2>&1 &
PIDS+=($!)
sleep 1
ok "Leader election daemon running  →  initial leader: pending first election"

# ── Radar Sim — pose-based target detection (no rendering required) ──
log "Starting radar_sim (headless radar detection)..."
RADAR_SIM_LOG="${SESSION_DIR}/radar_sim.log"
(cd "${SCRIPT_DIR}/src" && env PX4_GZ_WORLD="${PX4_GZ_WORLD}" python3 -u radar_sim.py) >> "${RADAR_SIM_LOG}" 2>&1 &
PIDS+=($!)
sleep 1
ok "Radar sim → ASP tracks at http://localhost:5000/asp"

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
