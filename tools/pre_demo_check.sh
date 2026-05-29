#!/usr/bin/env bash
# pre_demo_check.sh — Verify the Phase 0 demo pipeline is working.
# Run this before recording to confirm all three components produce output.
#
# What it tests:
#   1. ros2_ws built and packages present
#   2. aeris10_driver starts in sim_mode and publishes panel topics
#   3. detection_node starts and publishes /radar/targets
#   4. /radar/targets carries valid JSON with at least 1 detection
#
# Usage:  bash ~/aran_mbc/tools/pre_demo_check.sh
set -eo pipefail

ARAN="$HOME/aran_mbc"
WS_SETUP="$HOME/ros2_ws/install/setup.bash"
PASS=0; FAIL=0

ok()   { echo "  ✓  $*"; ((PASS++)); }
fail() { echo "  ✗  $*"; ((FAIL++)); }
head() { echo ""; echo "── $* ──"; }

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║   MBC-3 Phase 0 — Pre-Demo Pipeline Check            ║"
echo "╚══════════════════════════════════════════════════════╝"

# ── Step 1: Environment ───────────────────────────────────────────────────────
head "1. Environment"

source /opt/ros/jazzy/setup.bash

if [[ -f "$WS_SETUP" ]]; then
    source "$WS_SETUP"
    ok "ros2_ws built ($WS_SETUP)"
else
    fail "ros2_ws not built — run: bash $ARAN/setup_ws.sh"
    echo ""; echo "Cannot continue without built workspace. Run setup_ws.sh first."; exit 1
fi

if ros2 pkg list 2>/dev/null | grep -q "^radar_fusion$"; then
    ok "radar_fusion package found"
else
    fail "radar_fusion not in ros2 pkg list"
fi

if ros2 pkg list 2>/dev/null | grep -q "^aeris10_driver$"; then
    ok "aeris10_driver package found"
else
    fail "aeris10_driver not in ros2 pkg list"
fi

if python3 -c "import sklearn, joblib" 2>/dev/null; then
    ok "scikit-learn + joblib available"
else
    fail "scikit-learn or joblib missing — run: pip install --break-system-packages scikit-learn joblib"
fi

# ── Step 2: Start aeris10_driver in sim_mode ──────────────────────────────────
head "2. AERIS-10 driver (sim_mode)"

ros2 run aeris10_driver driver_node \
    --ros-args -p sim_mode:=true -p publish_hz:=10.0 \
    2>/dev/null &
DRIVER_PID=$!

sleep 3

# Check at least one panel topic is publishing
TOPIC_FOUND=0
for panel in A B C D E F; do
    if ros2 topic hz /radar_${panel}/scan/points --window 5 2>/dev/null | \
       grep -q "average rate"; then
        TOPIC_FOUND=1; break
    fi
done

# Faster check: just list topics
if ros2 topic list 2>/dev/null | grep -q "/radar_A/scan/points"; then
    ok "Panel topics publishing (/radar_{A-F}/scan/points)"
    TOPIC_FOUND=1
fi

if [[ "$TOPIC_FOUND" -eq 0 ]]; then
    fail "No radar panel topics found"
fi

# ── Step 3: Start detection_node ──────────────────────────────────────────────
head "3. detection_node"

ros2 run radar_fusion detection_node 2>/dev/null &
DET_PID=$!

sleep 4

# Check /radar/targets is being published
if ros2 topic list 2>/dev/null | grep -q "^/radar/targets$"; then
    ok "/radar/targets topic exists"
else
    fail "/radar/targets not found (detection_node may have crashed)"
fi

# ── Step 4: Read one detection ────────────────────────────────────────────────
head "4. Detection output"

DETECTION=$(timeout 8 ros2 topic echo /radar/targets --once 2>/dev/null | \
    grep "^data:" | head -1 || true)

if [[ -z "$DETECTION" ]]; then
    fail "No message received on /radar/targets in 8s"
else
    # Extract JSON from data: '...'
    RAW="${DETECTION#data: }"
    RAW="${RAW#\'}"
    RAW="${RAW%\'}"
    N_TGTS=$(python3 -c "import json; d=json.loads('$RAW'); print(d.get('n_targets',0))" 2>/dev/null || echo "0")

    if [[ "$N_TGTS" -gt 0 ]]; then
        ok "Received detection: $N_TGTS target(s)"
        python3 -c "
import json, sys
raw = sys.argv[1]
d = json.loads(raw)
for t in d.get('targets', []):
    print(f'     → {t[\"id\"]}  R={t[\"range_m\"]}m  Az={t[\"az_deg\"]}°  Panel={t[\"panel\"]}')
" "$RAW" 2>/dev/null || true
    else
        fail "Message received but n_targets=0 (pipeline gap)"
    fi
fi

# ── Cleanup ───────────────────────────────────────────────────────────────────
kill $DRIVER_PID $DET_PID 2>/dev/null || true
wait 2>/dev/null || true

# ── Summary ───────────────────────────────────────────────────────────────────
echo ""
echo "══════════════════════════════════════════════════════"
echo "  PASS: $PASS   FAIL: $FAIL"
if [[ "$FAIL" -eq 0 ]]; then
    echo "  ✅  Pipeline verified — ready to record demo video"
else
    echo "  ❌  Fix the above failures before recording"
fi
echo "══════════════════════════════════════════════════════"
echo ""
