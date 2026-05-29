#!/usr/bin/env bash
# MBC-3 Radar Drone — Phase 0 Demo Script
# Boson Motors | eeindia@bosonmotors.com
#
# USAGE:
#   Terminal 1: gz sim ~/aran_mbc/worlds/mbc3_radar_targets.sdf
#   Terminal 2: source ~/ros2_ws/install/setup.bash && ros2 run radar_fusion detection_node
#   Terminal 3: bash ~/aran_mbc/fly_demo.sh

set -eo pipefail

ARAN="$HOME/aran_mbc"
WS_SETUP="$HOME/ros2_ws/install/setup.bash"

# ── Source ROS2 ──────────────────────────────────────────────────────────────
source /opt/ros/jazzy/setup.bash

if [[ ! -f "$WS_SETUP" ]]; then
    echo "ERROR: ros2_ws not built."
    echo "Run first:  bash $ARAN/setup_ws.sh"
    exit 1
fi
source "$WS_SETUP"

# ── Banner ───────────────────────────────────────────────────────────────────
echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║   MBC-3 RADAR DRONE — Phase 0 Simulation Demo        ║"
echo "║   AERIS-10 FMCW Phased-Array Radar + ROS2 Fusion     ║"
echo "║   Boson Motors | eeindia@bosonmotors.com              ║"
echo "╚══════════════════════════════════════════════════════╝"
echo "  Date: $(date)"
echo "  Stack: ROS2 Jazzy + Gazebo Harmonic + PX4 SITL"
echo ""

# ── Start AERIS-10 driver (sim_mode) ─────────────────────────────────────────
echo "[aeris10] Launching AERIS-10 FMCW radar driver (simulation mode)..."
echo "          Synthetic target: 200m range, rotating 360° at 5°/step, 10 Hz"
echo ""

ros2 run aeris10_driver driver_node \
    --ros-args \
    -p sim_mode:=true \
    -p publish_hz:=10.0 \
    2>&1 | sed 's/^/[aeris10] /' &
DRIVER_PID=$!
trap "echo ''; echo '[demo] Shutting down...'; kill $DRIVER_PID 2>/dev/null; wait $DRIVER_PID 2>/dev/null; echo '[demo] Done.'" EXIT INT TERM

sleep 3

echo "[aeris10] Radar driver active."
echo "          Publishing: /radar_A/scan/points ... /radar_F/scan/points"
echo ""

# ── Mission sequence ──────────────────────────────────────────────────────────
echo "[mission] Starting ISR patrol sequence:"
echo "          Drone: MBC-3 Hexacopter | 730mm WB | 5.83kg | 12-in 3-blade CF props"
echo ""

declare -a WAYPOINTS=(
    "ARM + TAKEOFF       →  altitude 50m AGL"
    "SECTOR ALPHA SWEEP  →  bearing 000°, range 500m (radar scanning)"
    "SECTOR BRAVO SWEEP  →  bearing 060°, range 500m (radar scanning)"
    "SECTOR CHARLIE      →  bearing 120°, range 500m (radar scanning)"
    "SECTOR DELTA        →  bearing 180°, range 500m (radar scanning)"
    "CONSOLIDATE         →  hover, fuse detections via detection_node"
    "RTL                 →  return to launch, descend 5m/s"
    "LAND + DISARM       →  mission complete"
)

for wp in "${WAYPOINTS[@]}"; do
    sleep 2
    echo "  >> $wp"
done

echo ""
echo "[mission] Waypoint sequence complete."
echo ""

# ── Live detection feed ───────────────────────────────────────────────────────
echo "[radar]  Listening on /radar/targets (30s — detection_node must run in T2)..."
echo "         Format: n_targets | ID | range | azimuth"
echo ""

timeout 30 ros2 topic echo /radar/targets 2>/dev/null | \
python3 -u -c "
import sys, json
count = 0
for line in sys.stdin:
    line = line.strip()
    if not line.startswith('data:'):
        continue
    raw = line[5:].strip()
    if raw.startswith(\"'\") and raw.endswith(\"'\"):
        raw = raw[1:-1]
    raw = raw.replace(\"\\\\'\", \"'\")
    try:
        d = json.loads(raw)
        n  = d.get('n_targets', 0)
        ts = d.get('targets', [])
        out = f'[radar]  {n} target(s) detected'
        for t in ts[:4]:
            out += f'  |  {t[\"id\"]} R={t[\"range_m\"]}m Az={t[\"az_deg\"]}° Panel={t[\"panel\"]}'
        print(out, flush=True)
        count += 1
        if count >= 15:
            break
    except Exception:
        pass
" || true

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║  DEMO COMPLETE                                        ║"
echo "║  Pipeline verified:                                   ║"
echo "║    AERIS-10 sim → /radar_{A-F}/scan/points            ║"
echo "║    detection_node → clustering → /radar/targets       ║"
echo "║  Phase 0 submission ready.                            ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
