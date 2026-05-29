#!/usr/bin/env bash
# setup_ws.sh — Create ~/ros2_ws and build MBC-3 ROS2 packages
# Run once before using fly_demo.sh or the demo pipeline.
set -eo pipefail

ARAN="$HOME/aran_mbc"
WS="$HOME/ros2_ws"

echo "=== MBC-3 ros2_ws setup ==="

# ── colcon ───────────────────────────────────────────────────────────────────
if ! command -v colcon &>/dev/null; then
    echo "[0/5] Installing colcon..."
    sudo apt-get install -y -q python3-colcon-common-extensions
else
    echo "[0/5] colcon already installed — skip"
fi

# ── Create workspace ──────────────────────────────────────────────────────────
mkdir -p "$WS/src"
echo "[1/5] Created $WS/src"

# ── Symlink packages ──────────────────────────────────────────────────────────
for pkg in radar_fusion aeris10_driver; do
    target="$WS/src/$pkg"
    if [[ -L "$target" ]]; then
        echo "[2/5] $pkg symlink already exists — skip"
    elif [[ -d "$target" ]]; then
        echo "[2/5] $pkg directory already exists — skip"
    else
        ln -s "$ARAN/$pkg" "$target"
        echo "[2/5] Linked $pkg → $ARAN/$pkg"
    fi
done

# ── Install Python deps ───────────────────────────────────────────────────────
echo "[3/5] Checking Python dependencies..."

# pyusb (USB hardware driver — sim_mode works without it)
pip install --quiet --break-system-packages pyusb 2>/dev/null || \
    (sudo apt-get install -y -q python3-usb 2>/dev/null && echo "  pyusb installed via apt") || \
    echo "  WARNING: pyusb not installed (sim_mode works without it)"

# scikit-learn + joblib — required by radar_fusion/rf_classifier.py (Layer 2 gate)
if ! python3 -c "import sklearn, joblib" &>/dev/null; then
    echo "  Installing scikit-learn + joblib..."
    pip install --quiet --break-system-packages scikit-learn joblib
    echo "  scikit-learn + joblib installed"
else
    echo "  scikit-learn + joblib already installed — skip"
fi

# ── Build ─────────────────────────────────────────────────────────────────────
echo "[4/5] Building packages (colcon)..."
source /opt/ros/jazzy/setup.bash
cd "$WS"
colcon build \
    --packages-select radar_fusion aeris10_driver \
    --symlink-install \
    2>&1 | tail -20

# ── Verify ────────────────────────────────────────────────────────────────────
echo "[5/5] Verifying install..."
source "$WS/install/setup.bash"
if ros2 pkg list | grep -q radar_fusion && ros2 pkg list | grep -q aeris10_driver; then
    echo "  radar_fusion    ✓"
    echo "  aeris10_driver  ✓"
else
    echo "  WARNING: one or more packages not found in ros2 pkg list"
fi

echo ""
echo "=== Build complete ==="
echo ""
echo "Demo commands:"
echo "  T1 (optional): gz sim $ARAN/worlds/mbc3_radar_targets.sdf"
echo "  T2:  source $WS/install/setup.bash && ros2 run radar_fusion detection_node"
echo "  T3:  bash $ARAN/fly_demo.sh"
echo ""
echo "Or run the full pipeline in one step:"
echo "  bash $ARAN/tools/pre_demo_check.sh"
