#!/usr/bin/env bash
# setup_ws.sh — Create ~/ros2_ws and build MBC-3 ROS2 packages
# Run once before using fly_demo.sh or the demo pipeline.
set -eo pipefail

ARAN="$HOME/aran_mbc"
WS="$HOME/ros2_ws"

echo "=== MBC-3 ros2_ws setup ==="

# ── Create workspace ──────────────────────────────────────────────────────────
mkdir -p "$WS/src"
echo "[1/4] Created $WS/src"

# ── Symlink packages ──────────────────────────────────────────────────────────
for pkg in radar_fusion aeris10_driver; do
    target="$WS/src/$pkg"
    if [[ -L "$target" ]]; then
        echo "[2/4] $pkg symlink already exists — skip"
    elif [[ -d "$target" ]]; then
        echo "[2/4] $pkg directory already exists — skip"
    else
        ln -s "$ARAN/$pkg" "$target"
        echo "[2/4] Linked $pkg → $ARAN/$pkg"
    fi
done

# ── Install Python deps ───────────────────────────────────────────────────────
echo "[3/4] Checking Python dependencies..."
pip install --quiet pyusb 2>/dev/null || \
    (sudo apt-get install -y -q python3-usb 2>/dev/null && echo "pyusb installed via apt") || \
    echo "  WARNING: pyusb not installed (sim_mode works without it)"

# ── Build ─────────────────────────────────────────────────────────────────────
echo "[4/4] Building packages (colcon)..."
source /opt/ros/jazzy/setup.bash
cd "$WS"
colcon build \
    --packages-select radar_fusion aeris10_driver \
    --symlink-install \
    2>&1 | tail -20

echo ""
echo "=== Build complete ==="
echo "Source the workspace:"
echo "  source $WS/install/setup.bash"
echo ""
echo "Then run the demo:"
echo "  T1: gz sim $ARAN/worlds/mbc3_radar_targets.sdf"
echo "  T2: source $WS/install/setup.bash && ros2 run radar_fusion detection_node"
echo "  T3: bash $ARAN/fly_demo.sh"
