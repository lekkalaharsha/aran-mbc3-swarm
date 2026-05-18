#!/usr/bin/env bash
# ============================================================
#  install_px4_model.sh — Install mbc3_radar_drone into PX4
#  Run this once from the repo root before using launch.sh.
#
#  Prerequisites (Ubuntu 24.04):
#    sudo apt install ros-jazzy-xacro
#    source /opt/ros/jazzy/setup.bash
#
#  Usage:
#    PX4_DIR=~/PX4-Autopilot bash new_drone/install_px4_model.sh
# ============================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_DIR="${PX4_DIR:-${HOME}/PX4-Autopilot}"
MODEL_NAME="mbc3_radar_drone"
MODEL_DEST="${PX4_DIR}/Tools/simulation/gz/models/${MODEL_NAME}"
AIRFRAME_DEST="${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes"
AIRFRAME_FILE="4601_gz_${MODEL_NAME}"
XACRO_FILE="${SCRIPT_DIR}/mbc3_radar_drone.xacro"
URDF_FILE="${SCRIPT_DIR}/mbc3_radar_drone.urdf"
SDF_FILE="${SCRIPT_DIR}/model.sdf"

RED='\033[0;31m'; GRN='\033[0;32m'; YLW='\033[0;33m'; RST='\033[0m'
ok()   { echo -e "${GRN}✓  $*${RST}"; }
warn() { echo -e "${YLW}⚠  $*${RST}"; }
err()  { echo -e "${RED}✗  $*${RST}"; exit 1; }

echo "================================================="
echo "  MBC-3 Radar Drone — PX4 Model Installer"
echo "================================================="
echo "  PX4 dir   : ${PX4_DIR}"
echo "  Model dest : ${MODEL_DEST}"
echo ""

# ── Check PX4 dir ────────────────────────────────────────────
[[ -d "${PX4_DIR}" ]] || err "PX4 directory not found: ${PX4_DIR}. Set PX4_DIR env var."

# ── Check dependencies ───────────────────────────────────────
for cmd in xacro gz; do
    command -v "${cmd}" &>/dev/null \
        || err "${cmd} not found. Source: source /opt/ros/jazzy/setup.bash"
done
ok "xacro and gz available"

# ── Step 1: xacro → URDF ─────────────────────────────────────
echo ""
echo "==> Step 1: xacro → URDF (use_ros2_control:=false for PX4 mode)"
xacro "${XACRO_FILE}" \
    use_ros2_control:=false \
    drone_ns:="" \
    > "${URDF_FILE}" \
    && ok "URDF written: ${URDF_FILE}" \
    || err "xacro failed — check xacro syntax"

# ── Step 2: URDF → SDF ───────────────────────────────────────
echo ""
echo "==> Step 2: URDF → SDF (gz sdf -p)"
gz sdf -p "${URDF_FILE}" > "${SDF_FILE}" \
    && ok "SDF written: ${SDF_FILE}" \
    || err "gz sdf conversion failed"

# ── Step 3: Install model ────────────────────────────────────
echo ""
echo "==> Step 3: Installing model to ${MODEL_DEST}"
mkdir -p "${MODEL_DEST}"
cp "${SCRIPT_DIR}/model.config" "${MODEL_DEST}/"
cp "${SDF_FILE}"                "${MODEL_DEST}/"
ok "Model installed"

# ── Step 4: Install airframe ─────────────────────────────────
echo ""
echo "==> Step 4: Installing airframe ${AIRFRAME_FILE}"
cp "${SCRIPT_DIR}/airframe/${AIRFRAME_FILE}" "${AIRFRAME_DEST}/"
chmod +x "${AIRFRAME_DEST}/${AIRFRAME_FILE}"
ok "Airframe installed"

# Register airframe in CMakeLists.txt if not already present
CMAKEFILE="${AIRFRAME_DEST}/CMakeLists.txt"
if [[ -f "${CMAKEFILE}" ]] && ! grep -q "${AIRFRAME_FILE}" "${CMAKEFILE}"; then
    # Insert after the last gz_x500_flow entry (last of the gz_x500 series)
    sed -i "/4021_gz_x500_flow/a \\t${AIRFRAME_FILE}" "${CMAKEFILE}" \
        && ok "Registered in CMakeLists.txt" \
        || warn "Auto-register failed — add '\\t${AIRFRAME_FILE}' to ${CMAKEFILE} manually"
    # Invalidate cmake cache so PX4 picks up new airframe on next build
    rm -f "${PX4_DIR}/build/px4_sitl_default/CMakeCache.txt" && ok "CMake cache cleared"
else
    ok "CMakeLists.txt already up-to-date"
fi

# ── Done ─────────────────────────────────────────────────────
echo ""
echo "================================================="
ok "Install complete."
echo ""
echo "  Launch with:"
echo "    PX4_MAKE_MODEL=gz_${MODEL_NAME} ./launch.sh"
echo ""
echo "  Or set in launch.sh:"
echo "    PX4_MAKE_MODEL=\"gz_${MODEL_NAME}\""
echo "================================================="
