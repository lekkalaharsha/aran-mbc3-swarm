#!/usr/bin/env bash
# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  Aran Technologies — ISR Mission Launch Script  v2                      ║
# ║  Sequence: PX4 SITL → GCS Dashboard → Mission                          ║
# ║  Compatible with Ubuntu 22.04 / 24.04  |  Python 3.10+  |  Bash 5+     ║
# ╚══════════════════════════════════════════════════════════════════════════╝

set -euo pipefail

# ══════════════════════════════════════════════════════════
#  COLOURS & LOGGING
# ══════════════════════════════════════════════════════════
if [[ -t 1 ]]; then
    RED='\033[0;31m'; GRN='\033[0;32m'; YLW='\033[0;33m'
    CYN='\033[0;36m'; WHT='\033[1;37m'; DIM='\033[0;2m'; RST='\033[0m'
else
    RED=''; GRN=''; YLW=''; CYN=''; WHT=''; DIM=''; RST=''
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="${SCRIPT_DIR}/logs"
TS="$(date +%Y%m%d_%H%M%S)"
LOG_FILE="${LOG_DIR}/launch_${TS}.log"
mkdir -p "${LOG_DIR}"

log()      { echo -e "${WHT}[$(date +%H:%M:%S)]  $*${RST}"    | tee -a "${LOG_FILE}"; }
log_ok()   { echo -e "${GRN}[$(date +%H:%M:%S)] ✓  $*${RST}" | tee -a "${LOG_FILE}"; }
log_warn() { echo -e "${YLW}[$(date +%H:%M:%S)] ⚠  $*${RST}" | tee -a "${LOG_FILE}"; }
log_err()  { echo -e "${RED}[$(date +%H:%M:%S)] ✗  $*${RST}" | tee -a "${LOG_FILE}"; }
log_info() { echo -e "${DIM}[$(date +%H:%M:%S)]    $*${RST}"  | tee -a "${LOG_FILE}"; }

banner() {
    echo -e "\n${CYN}╔══════════════════════════════════════════════════════╗${RST}" | tee -a "${LOG_FILE}"
    printf "${CYN}║${WHT}  %-50s${CYN}║${RST}\n" "$1" | tee -a "${LOG_FILE}"
    echo -e "${CYN}╚══════════════════════════════════════════════════════╝${RST}" | tee -a "${LOG_FILE}"
}

# ══════════════════════════════════════════════════════════
#  DEFAULTS
# ══════════════════════════════════════════════════════════
PX4_DIR="${PX4_DIR:-${HOME}/PX4-Autopilot}"
# gz_x500_lidar_2d: x500 quad with 360° 2D LiDAR — required for real obstacle avoidance.
# gz_x500: standard quad WITHOUT LiDAR — isr_lidar_mpc falls back to SIM mode (0 real scans).
# Override at runtime: PX4_MAKE_MODEL=gz_x500 ./launch.sh
PX4_MAKE_DIR="px4_sitl"
PX4_MAKE_MODEL="${PX4_MAKE_MODEL:-gz_x500_lidar_2d}"
PYTHON="${PYTHON:-python3}"
GCS_PORT=5000
GCS_READY_TIMEOUT=20   # seconds Flask has to bind
PX4_READY_TIMEOUT=120  # seconds SITL+Gazebo has to boot (first build can be slow)

# Racing mode: inject env var so mission script loads RACING_* params
RACING_MODE="${RACING_MODE:-1}"

# Strings that confirm PX4 SITL is fully up and accepting connections
PX4_READY_PATTERNS=(
    "Ready for takeoff"
    "commander] Ready"
    "mavlink started"
    "Startup script returned successfully"
    "home position set"
)

OPT_SCENARIO=""
OPT_NO_DEPS=false
OPT_HEADLESS=false
OPT_CLEAN_LOGS=false
OPT_SIM_ONLY=false
OPT_GCS_ONLY=false

PID_PX4=""
PID_GCS=""
PID_MISSION=""

# ══════════════════════════════════════════════════════════
#  USAGE
# ══════════════════════════════════════════════════════════
usage() {
    echo -e "
${WHT}Usage:${RST}  ${GRN}./launch.sh${RST} [OPTIONS]

${WHT}Options:${RST}
  ${CYN}--scenario NAME${RST}    Named LiDAR sim scenario from scenarios.json
  ${CYN}--headless${RST}         Launch Gazebo in server/no-GUI mode (HEADLESS=1)
  ${CYN}--sim-only${RST}         Start PX4 + Gazebo only — skip GCS and mission
  ${CYN}--gcs-only${RST}         Start GCS only (SITL already running separately)
  ${CYN}--no-deps${RST}          Skip Python package installation checks
  ${CYN}--clean-logs${RST}       Delete previous log files before starting
  ${CYN}--px4-dir PATH${RST}     Override PX4-Autopilot path (default: ~/PX4-Autopilot)
  ${CYN}--help${RST}             Show this help and exit

${WHT}Environment overrides:${RST}
  PX4_DIR    Path to PX4-Autopilot  (default: ~/PX4-Autopilot)
  PYTHON     Python interpreter     (default: python3)

${WHT}Launch sequence:${RST}
  1. Dependency + file checks
  2. make px4_sitl gz_x500_lidar_2d  (waits for real ready signal in log)
  3. telemetry_web.py GCS dashboard  (waits for TCP port 5000 to bind)
  4. isr_lidar_mpc.py ISR mission    (foreground, Ctrl-C to abort all)

${WHT}Examples:${RST}
  ${GRN}./launch.sh${RST}
  ${GRN}./launch.sh --scenario urban_canyon${RST}
  ${GRN}./launch.sh --headless --scenario nfz_breach${RST}
  ${GRN}./launch.sh --sim-only${RST}
  ${GRN}./launch.sh --gcs-only${RST}
"
}

# ══════════════════════════════════════════════════════════
#  PARSE ARGS
# ══════════════════════════════════════════════════════════
while [[ $# -gt 0 ]]; do
    case "$1" in
        --scenario)   OPT_SCENARIO="$2"; shift 2 ;;
        --headless)   OPT_HEADLESS=true;  shift   ;;
        --sim-only)   OPT_SIM_ONLY=true;  shift   ;;
        --gcs-only)   OPT_GCS_ONLY=true;  shift   ;;
        --no-deps)    OPT_NO_DEPS=true;   shift   ;;
        --clean-logs) OPT_CLEAN_LOGS=true; shift  ;;
        --px4-dir)    PX4_DIR="$2";       shift 2 ;;
        --help|-h)    usage; exit 0 ;;
        *) log_err "Unknown option: $1"; usage; exit 1 ;;
    esac
done

# ══════════════════════════════════════════════════════════
#  CLEANUP / SIGNAL TRAPS
# ══════════════════════════════════════════════════════════
cleanup() {
    echo ""
    banner "SHUTDOWN — stopping all processes"
    local -a pids=("${PID_MISSION}" "${PID_GCS}" "${PID_PX4}")
    local -a names=("Mission" "GCS" "PX4 SITL")
    for i in "${!pids[@]}"; do
        local pid="${pids[$i]}"
        local name="${names[$i]}"
        if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
            log "Stopping ${name} (PID ${pid})…"
            kill -SIGTERM "${pid}" 2>/dev/null || true
            local w=0
            while kill -0 "${pid}" 2>/dev/null && (( w < 8 )); do
                sleep 0.5; (( w++ )) || true
            done
            if kill -0 "${pid}" 2>/dev/null; then
                log_warn "${name} ignored SIGTERM — sending SIGKILL"
                kill -SIGKILL "${pid}" 2>/dev/null || true
            else
                log_ok "${name} stopped"
            fi
        fi
    done
    # Kill any stray Gazebo / PX4 child processes not caught above
    pkill -f "gz sim"        2>/dev/null || true
    pkill -f "gzserver"      2>/dev/null || true
    pkill -f "px4.*sitl"     2>/dev/null || true
    # Kill stale MAVSDK server — holds UDP :14540 and causes upload failures on re-launch
    pkill -f "mavsdk_server" 2>/dev/null || true
    log_info "All logs saved to: ${LOG_DIR}/"
    log_ok   "Shutdown complete"
}

trap cleanup EXIT
trap 'echo ""; log_err "Interrupted (Ctrl-C)"; exit 130'  INT
trap 'echo ""; log_err "Terminated (SIGTERM)";  exit 143' TERM

# ══════════════════════════════════════════════════════════
#  HEADER
# ══════════════════════════════════════════════════════════
clear
echo -e "${CYN}"
cat << 'EOF'
  +══════════════════════════════════════════════════════+
  ║   ARAN TECHNOLOGIES — ISR MISSION LAUNCH SCRIPT     ║
  ║   v13-RACE  |  PX4 SITL + Gazebo Harmonic + MAVSDK ║
  ║   Nirmaan Incubation — IIT Hyderabad Demo Build     ║
  +══════════════════════════════════════════════════════+
EOF
echo -e "${RST}"
log_info "Working directory : ${SCRIPT_DIR}"
log_info "Log file          : ${LOG_FILE}"
log_info "Python            : ${PYTHON}"
log_info "PX4 directory     : ${PX4_DIR}"
log_info "PX4 make target   : make ${PX4_MAKE_DIR} ${PX4_MAKE_MODEL}"
log_info "Racing mode       : RACING_MODE=${RACING_MODE}"
[[ -n "${OPT_SCENARIO}" ]] && log_info "Scenario          : ${OPT_SCENARIO}"
echo ""

# ══════════════════════════════════════════════════════════
#  OPTIONAL: CLEAN OLD LOGS
# ══════════════════════════════════════════════════════════
if [[ "${OPT_CLEAN_LOGS}" == true ]]; then
    log "Cleaning old logs…"
    find "${LOG_DIR}" -name "*.log" ! -name "launch_${TS}.log" -delete 2>/dev/null || true
    log_ok "Old logs removed"
fi

# ══════════════════════════════════════════════════════════
#  STEP 1: DEPENDENCY CHECKS
# ══════════════════════════════════════════════════════════
banner "STEP 1 — Dependency checks"

DEPS_OK=true

check_cmd() {
    local cmd="$1" hint="$2"
    if command -v "${cmd}" &>/dev/null; then
        log_ok "${cmd}  →  $(command -v "${cmd}")"
    else
        log_err "${cmd} not found — ${hint}"
        DEPS_OK=false
    fi
}

check_cmd "${PYTHON}" "sudo apt install python3"
check_cmd "make"      "sudo apt install build-essential"

if [[ "${OPT_GCS_ONLY}" == false ]]; then
    check_cmd "gz" "Install Gazebo Harmonic: https://gazebosim.org/docs/harmonic/install"
fi

[[ "${DEPS_OK}" == false ]] && { log_err "Fix the missing tools above and re-run"; exit 1; }

# Python packages
if [[ "${OPT_NO_DEPS}" == false ]]; then
    log "Checking Python packages…"
    REQUIRED=(mavsdk flask flask_socketio requests numpy scipy)
    MISSING=()
    for pkg in "${REQUIRED[@]}"; do
        if "${PYTHON}" -c "import ${pkg}" &>/dev/null 2>&1; then
            log_ok "  ${pkg}"
        else
            log_warn "  ${pkg} — not found"
            MISSING+=("${pkg}")
        fi
    done
    if (( ${#MISSING[@]} > 0 )); then
        log "Installing missing packages: ${MISSING[*]}"
        "${PYTHON}" -m pip install --quiet "${MISSING[@]}" \
            && log_ok "Packages installed successfully" \
            || { log_err "pip install failed. Run manually: pip install ${MISSING[*]}"; exit 1; }
    fi

    # numpy: required by mapping_3d.py fast path (falls back to pure Python if absent)
    if "${PYTHON}" -c "import numpy" &>/dev/null 2>&1; then
        log_ok "  numpy (mapping_3d fast path enabled)"
    else
        log_warn "  numpy not found — mapping_3d will use slower pure-Python path"
        log_info "  Install: pip install numpy"
    fi

    # gz-transport: determines real LiDAR vs SIM mode
    if "${PYTHON}" -c "from gz.transport13 import Node" &>/dev/null 2>&1; then
        log_ok "  gz-transport13 (real LiDAR mode available)"
    else
        log_warn "  gz-transport13 not found — will run in LiDAR-SIM mode"
        log_info "  To enable real LiDAR: sudo apt install python3-gz-transport13 python3-gz-msgs10"
    fi
fi

# ══════════════════════════════════════════════════════════
#  STEP 2: VERIFY MISSION FILES
# ══════════════════════════════════════════════════════════
banner "STEP 2 — Verify mission files"

REQUIRED_FILES=(
    isr_lidar_mpc.py
    mpc_controller.py
    mission_config.py
    telemetry_web.py
    mapping_3d.py
    scenarios.json
)
FILES_OK=true
for f in "${REQUIRED_FILES[@]}"; do
    if [[ -f "${SCRIPT_DIR}/${f}" ]]; then
        log_ok "${f}"
    else
        log_err "${f} — NOT FOUND in ${SCRIPT_DIR}"
        FILES_OK=false
    fi
done
[[ "${FILES_OK}" == false ]] && { log_err "Missing files — cannot continue"; exit 1; }

# Auto-patch telemetry_web.py if allow_unsafe_werkzeug is missing
# (flask-socketio ≥ 5.3 raises RuntimeError without this flag)
if ! grep -q "allow_unsafe_werkzeug" "${SCRIPT_DIR}/telemetry_web.py"; then
    log_warn "telemetry_web.py: allow_unsafe_werkzeug=True not found — patching…"
    sed -i \
        's/socketio\.run(\(.*\)debug=False\(.*\))/socketio.run(\1debug=False\2, allow_unsafe_werkzeug=True)/' \
        "${SCRIPT_DIR}/telemetry_web.py" \
        && log_ok "  telemetry_web.py patched successfully" \
        || {
            log_err "  Auto-patch failed — add allow_unsafe_werkzeug=True to socketio.run() manually"
            exit 1
        }
fi
log_ok "telemetry_web.py werkzeug flag confirmed"

# Pre-create map output directory (mapping_3d.py will also create it, but doing
# it here avoids a permission surprise at end-of-mission when RTL is in progress)
MAP_OUT_DIR="${SCRIPT_DIR}/map_output"
if [[ ! -d "${MAP_OUT_DIR}" ]]; then
    mkdir -p "${MAP_OUT_DIR}" \
        && log_ok "Created map output directory: ${MAP_OUT_DIR}" \
        || log_warn "Could not create map output directory — mapping_3d will retry at runtime"
else
    log_ok "Map output directory exists: ${MAP_OUT_DIR}"
fi

# Syntax check mapping_3d.py
log "Pre-flight syntax check: mapping_3d.py…"
if ! "${PYTHON}" -m py_compile "${SCRIPT_DIR}/mapping_3d.py" 2>&1 | tee -a "${LOG_FILE}"; then
    log_err "Syntax error in mapping_3d.py — aborting"
    exit 1
fi
log_ok "mapping_3d.py syntax OK"

# Import check — confirms mapping_3d.py is importable from the mission working dir.
# py_compile only checks syntax; it does NOT verify the file is on sys.path when
# isr_lidar_mpc.py runs.  This catches the "file exists but isn't in SCRIPT_DIR" case.
log "Import check: mapping_3d importable from ${SCRIPT_DIR}…"
if ! (cd "${SCRIPT_DIR}" && "${PYTHON}" -c "import mapping_3d" 2>&1 | tee -a "${LOG_FILE}"); then
    log_err "mapping_3d is not importable from ${SCRIPT_DIR}"
    log_info "Make sure mapping_3d.py is in the same directory as isr_lidar_mpc.py"
    log_info "Expected path: ${SCRIPT_DIR}/mapping_3d.py"
    exit 1
fi
log_ok "mapping_3d import OK"

# Validate scenario name against scenarios.json
if [[ -n "${OPT_SCENARIO}" ]]; then
    log "Validating scenario '${OPT_SCENARIO}'…"
    if "${PYTHON}" - <<PYEOF
import json, sys
with open("${SCRIPT_DIR}/scenarios.json") as f:
    d = json.load(f)
names = [s["name"] for s in d["scenarios"]]
if "${OPT_SCENARIO}" not in names:
    print("  ERROR: scenario not found. Available scenarios:")
    for n in names:
        print(f"    {n}")
    sys.exit(1)
PYEOF
    then
        log_ok "Scenario '${OPT_SCENARIO}' confirmed in scenarios.json"
    else
        exit 1
    fi
fi

# ══════════════════════════════════════════════════════════
#  STEP 3: PX4 SITL + GAZEBO
# ══════════════════════════════════════════════════════════
if [[ "${OPT_GCS_ONLY}" == false ]]; then
    banner "STEP 3 — PX4 SITL  (make ${PX4_MAKE_DIR} ${PX4_MAKE_MODEL})"

    if [[ ! -d "${PX4_DIR}" ]]; then
        log_err "PX4 directory not found: ${PX4_DIR}"
        log_info "Clone it:  git clone https://github.com/PX4/PX4-Autopilot.git --recursive"
        log_info "Or set:    export PX4_DIR=/your/path  and re-run"
        exit 1
    fi

    # Verify the make target exists before launching in background
    log "Checking make target '${PX4_MAKE_DIR} ${PX4_MAKE_MODEL}' exists…"
    if ! (cd "${PX4_DIR}" && make list_config_targets 2>/dev/null | grep -q "${PX4_MAKE_DIR}"); then
        log_warn "Could not verify target via list_config_targets — proceeding anyway"
    else
        log_ok "Make target confirmed"
    fi

    PX4_LOG="${LOG_DIR}/px4_${TS}.log"

    [[ "${OPT_HEADLESS}" == true ]] && export HEADLESS=1 && log_info "Headless mode enabled (no Gazebo GUI)"

    log "Launching: make ${PX4_MAKE_DIR} ${PX4_MAKE_MODEL}"
    (
        cd "${PX4_DIR}"
        make ${PX4_MAKE_DIR} ${PX4_MAKE_MODEL}
    ) >> "${PX4_LOG}" 2>&1 &
    PID_PX4=$!
    log_info "PX4 PID: ${PID_PX4}  |  log: ${PX4_LOG}"
    log_info "Tail PX4 log: tail -f ${PX4_LOG}"

    # ── Wait for a real ready signal in the log ──────────────────────────
    # We do NOT use /dev/udp — UDP has no handshake and always returns true
    # immediately, causing false "ready" detection while make is still
    # compiling or Gazebo is still loading the world.
    log "Waiting for PX4 SITL ready signal (timeout: ${PX4_READY_TIMEOUT}s)…"
    log_info "Watching for: ${PX4_READY_PATTERNS[*]}"
    waited=0
    px4_ready=false

    while (( waited < PX4_READY_TIMEOUT )); do
        sleep 1; (( waited++ )) || true

        # Bail out immediately if make exited with an error
        if ! kill -0 "${PID_PX4}" 2>/dev/null; then
            log_err "PX4 make process exited after ${waited}s — checking log for errors"
            echo ""
            log_err "Last 20 lines of ${PX4_LOG}:"
            tail -20 "${PX4_LOG}" 2>/dev/null | while IFS= read -r line; do
                log_err "  ${line}"
            done
            echo ""
            log_info "Common fixes:"
            log_info "  Wrong target?  →  cd ${PX4_DIR} && make list_config_targets"
            log_info "  Missing deps?  →  bash ${PX4_DIR}/Tools/setup/ubuntu.sh"
            log_info "  Gazebo not found? →  source /opt/ros/jazzy/setup.bash"
            exit 1
        fi

        # Check log for any ready pattern
        for pattern in "${PX4_READY_PATTERNS[@]}"; do
            if grep -q "${pattern}" "${PX4_LOG}" 2>/dev/null; then
                px4_ready=true
                log_ok "PX4 ready  (matched: '${pattern}'  after ${waited}s)"
                break 2
            fi
        done

        # Progress indicator every 15s
        if (( waited % 15 == 0 )); then
            log_info "  Still waiting for PX4…  (${waited}/${PX4_READY_TIMEOUT}s)"
            # Show last meaningful line from log to indicate progress
            # last_line must be initialised before use — set -u treats an unset
            # variable as an error even when guarded by [[ -n ... ]] on the same line.
            last_line=""
            last_line="$(grep -v "^$" "${PX4_LOG}" 2>/dev/null | tail -1 || true)"
            [[ -n "${last_line}" ]] && log_info "  PX4 log tail: ${last_line}"
        fi
    done

    if [[ "${px4_ready}" == false ]]; then
        log_err "PX4 SITL did not signal ready within ${PX4_READY_TIMEOUT}s"
        log_err "Last 20 lines of ${PX4_LOG}:"
        tail -20 "${PX4_LOG}" 2>/dev/null | while IFS= read -r line; do
            log_err "  ${line}"
        done
        exit 1
    fi

    log_ok "PX4 SITL + Gazebo fully initialised"

    if [[ "${OPT_SIM_ONLY}" == true ]]; then
        log_ok "SIM-ONLY mode — GCS and mission will not start"
        log_ok "Press Ctrl-C to stop PX4 + Gazebo"
        wait "${PID_PX4}" || true
        exit 0
    fi
fi

# ══════════════════════════════════════════════════════════
#  STEP 4: GCS DASHBOARD
# ══════════════════════════════════════════════════════════
banner "STEP 4 — GCS Dashboard  (telemetry_web.py)"

# Free port 5000 if held by a stale process
if (echo >/dev/tcp/127.0.0.1/${GCS_PORT}) &>/dev/null 2>&1; then
    log_warn "Port ${GCS_PORT} is already in use — attempting to free it"
    # Try fuser first (psmisc), fall back to lsof
    fuser -k "${GCS_PORT}/tcp" 2>/dev/null \
        || { lsof -ti tcp:"${GCS_PORT}" 2>/dev/null | xargs -r kill -9 || true; }
    sleep 1
    if (echo >/dev/tcp/127.0.0.1/${GCS_PORT}) &>/dev/null 2>&1; then
        log_err "Port ${GCS_PORT} still in use after kill attempt"
        log_info "Free it manually: sudo lsof -ti tcp:${GCS_PORT} | xargs kill -9"
        exit 1
    fi
    log_ok "Port ${GCS_PORT} freed"
fi

GCS_LOG="${LOG_DIR}/gcs_${TS}.log"
(
    cd "${SCRIPT_DIR}"
    "${PYTHON}" telemetry_web.py
) >> "${GCS_LOG}" 2>&1 &
PID_GCS=$!
log_info "GCS PID: ${PID_GCS}  |  log: ${GCS_LOG}"

# Wait for Flask to bind on TCP port 5000
log "Waiting for GCS to bind on port ${GCS_PORT} (timeout: ${GCS_READY_TIMEOUT}s)…"
waited=0
gcs_ready=false

while (( waited < GCS_READY_TIMEOUT * 2 )); do
    sleep 0.5; (( waited++ )) || true

    # Fail fast if Flask exited immediately (e.g. import error, port conflict)
    if ! kill -0 "${PID_GCS}" 2>/dev/null; then
        log_err "GCS process died — full log:"
        cat "${GCS_LOG}" | while IFS= read -r line; do log_err "  ${line}"; done
        echo ""
        log_info "Common fixes:"
        log_info "  Werkzeug error?  →  ensure allow_unsafe_werkzeug=True in socketio.run()"
        log_info "  Import error?    →  pip install flask flask-socketio"
        log_info "  Port conflict?   →  sudo lsof -ti tcp:5000 | xargs kill -9"
        exit 1
    fi

    # TCP port bind check (reliable for HTTP servers)
    if (echo >/dev/tcp/127.0.0.1/${GCS_PORT}) &>/dev/null 2>&1; then
        gcs_ready=true
        break
    fi
done

if [[ "${gcs_ready}" == false ]]; then
    log_err "GCS did not bind on port ${GCS_PORT} within ${GCS_READY_TIMEOUT}s"
    cat "${GCS_LOG}" | while IFS= read -r line; do log_err "  ${line}"; done
    exit 1
fi

log_ok "GCS dashboard is live → http://localhost:${GCS_PORT}"
log_info "Open in browser to monitor the ISR mission"

if [[ "${OPT_GCS_ONLY}" == true ]]; then
    log_ok "GCS-ONLY mode — press Ctrl-C to stop"
    wait "${PID_GCS}" || true
    exit 0
fi

# ══════════════════════════════════════════════════════════
#  STEP 5: ISR MISSION
# ══════════════════════════════════════════════════════════
banner "STEP 5 — ISR Mission  (isr_lidar_mpc.py)"

# ── Kill any stale isr_lidar_mpc process from a prior run ────────────────
# A previous run that was Ctrl-C'd or crashed may leave a Python process still
# holding the MAVSDK gRPC connection and, more critically, the UDP socket on
# port 14540.  When the new process calls drone.connect("udpin://:14540") while
# the old one still owns the port, the MAVLink MISSION_COUNT exchange silently
# fails — upload_mission() appears to succeed but mission_progress() closes
# immediately at total=0 on every attempt.
# We pkill here (before launching the new process) and give the OS 2 s to
# release the socket before the new Python instance takes over.
log "Checking for stale mission processes on UDP :14540…"
if pkill -f "isr_lidar_mpc" 2>/dev/null; then
    log_warn "Killed stale isr_lidar_mpc process — waiting 2s for socket release"
    sleep 2
else
    log_ok "No stale mission process found"
fi
# Also evict any lingering MAVSDK gRPC server that may be holding the port
pkill -f "mavsdk_server" 2>/dev/null && sleep 1 || true

MISSION_LOG="${LOG_DIR}/mission_${TS}.log"

# Build optional scenario env injection
SCENARIO_ENV=""
if [[ -n "${OPT_SCENARIO}" ]]; then
    SCENARIO_ENV="ISR_SIM_SCENARIO=${OPT_SCENARIO}"
    log_info "Injecting scenario env: ${SCENARIO_ENV}"
fi

# ── Pre-launch syntax check (catches indentation / SyntaxError instantly) ──
log "Pre-flight syntax check: isr_lidar_mpc.py…"
if ! (cd "${SCRIPT_DIR}" && "${PYTHON}" -m py_compile isr_lidar_mpc.py 2>&1 | tee -a "${LOG_FILE}"); then
    log_err "Syntax error in isr_lidar_mpc.py — aborting before PX4/GCS start"
    log_info "Fix the error above and re-run: ./launch.sh"
    exit 1
fi
log_ok "Syntax check passed"

log "Launching isr_lidar_mpc.py…"
(
    cd "${SCRIPT_DIR}"
    # env prefix only added when SCENARIO_ENV is non-empty
    ${SCENARIO_ENV:+env "${SCENARIO_ENV}"} \
        env "RACING_MODE=${RACING_MODE}" \
        "${PYTHON}" isr_lidar_mpc.py 2>&1 \
        | tee -a "${MISSION_LOG}"
) &
PID_MISSION=$!
log_info "Mission PID: ${PID_MISSION}  |  log: ${MISSION_LOG}"

# ══════════════════════════════════════════════════════════
#  STEP 6: LIVE STATUS MONITOR
# ══════════════════════════════════════════════════════════
banner "STEP 6 — Mission in progress"
echo -e "${DIM}  GCS dashboard: http://localhost:${GCS_PORT}${RST}"
echo -e "${DIM}  Press Ctrl-C at any time to abort and cleanly stop all processes${RST}"
echo ""

# Background thread: print a process-health line every 10 seconds
(
    while kill -0 "${PID_MISSION}" 2>/dev/null; do
        sleep 10

        px4_s="${DIM}N/A${RST}"
        [[ -n "${PID_PX4}" ]] && {
            kill -0 "${PID_PX4}" 2>/dev/null \
                && px4_s="${GRN}UP${RST}" \
                || px4_s="${RED}DOWN${RST}"
        }

        kill -0 "${PID_GCS}"     2>/dev/null \
            && gcs_s="${GRN}UP${RST}"      || gcs_s="${YLW}DOWN${RST}"
        kill -0 "${PID_MISSION}" 2>/dev/null \
            && mis_s="${GRN}RUNNING${RST}" || mis_s="${YLW}DONE${RST}"

        echo -e "${DIM}[$(date +%H:%M:%S)]${RST}  PX4=${px4_s}  GCS=${gcs_s}  Mission=${mis_s}" \
            | tee -a "${LOG_FILE}"
    done
) &
MONITOR_PID=$!

# Foreground: block until the mission script finishes
# BUG FIX: "wait PID || true" always sets $? to 0, masking real Python exit
# codes (e.g. exit 1 on SyntaxError / unhandled exception).  Capture the real
# exit status with "&& / ||" so the log correctly says FAILED vs SUCCESS.
wait "${PID_MISSION}" && MISSION_EXIT=0 || MISSION_EXIT=$?
kill "${MONITOR_PID}" 2>/dev/null || true

echo ""
if (( MISSION_EXIT == 0 )); then
    log_ok "Mission completed successfully (exit 0)"
elif (( MISSION_EXIT == 130 )); then
    log_warn "Mission aborted by user (Ctrl-C / SIGINT)"
else
    log_err "Mission FAILED — exit code ${MISSION_EXIT}"
    log_err "Check mission log: ${MISSION_LOG}"
    log_info "Hint: 'python3 -m py_compile isr_lidar_mpc.py' to catch syntax errors before launch"
fi

# ══════════════════════════════════════════════════════════
#  DONE — EXIT trap runs cleanup() automatically
# ══════════════════════════════════════════════════════════
echo ""
banner "Mission finished — logs"
log_info "  PX4     → ${LOG_DIR}/px4_${TS}.log"
log_info "  GCS     → ${LOG_DIR}/gcs_${TS}.log"
log_info "  Mission → ${LOG_DIR}/mission_${TS}.log"
log_info "  Launch  → ${LOG_FILE}"
log_info "  3D Map  → ${MAP_OUT_DIR}/"