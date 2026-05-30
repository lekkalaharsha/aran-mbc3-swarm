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

ARAN="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_SETUP="$HOME/ros2_ws/install/setup.bash"
PASS=0; FAIL=0

ok()   { echo "  ✓  $*"; PASS=$((PASS+1)); }
fail() { echo "  ✗  $*"; FAIL=$((FAIL+1)); }
section() { echo ""; echo "── $* ──"; }

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║   MBC-3 Phase 0 — Pre-Demo Pipeline Check            ║"
echo "╚══════════════════════════════════════════════════════╝"

# ── Step 1: Environment ───────────────────────────────────────────────────────
section "1. Environment"

source /opt/ros/jazzy/setup.bash

if [[ -f "$WS_SETUP" ]]; then
    source "$WS_SETUP"
    ok "ros2_ws built ($WS_SETUP)"
else
    fail "ros2_ws not built — run: bash $ARAN/setup_ws.sh"
    echo ""; echo "Cannot continue without built workspace. Run setup_ws.sh first."; exit 1
fi

PKGS=$(ros2 pkg list 2>/dev/null || true)
if echo "$PKGS" | grep -q "^radar_fusion$"; then
    ok "radar_fusion package found"
else
    fail "radar_fusion not in ros2 pkg list"
fi

if echo "$PKGS" | grep -q "^aeris10_driver$"; then
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
section "2. AERIS-10 driver (sim_mode)"

ros2 run aeris10_driver driver_node \
    --ros-args -p sim_mode:=true -p publish_hz:=10.0 \
    2>/dev/null &
DRIVER_PID=$!

# Poll until /radar_A/scan/points appears (up to 10 s)
TOPIC_FOUND=0
for i in $(seq 1 20); do
    if ros2 topic list 2>/dev/null | grep -q "/radar_A/scan/points"; then
        TOPIC_FOUND=1; break
    fi
    sleep 0.5
done

if [[ "$TOPIC_FOUND" -eq 1 ]]; then
    ok "Panel topics publishing (/radar_{A-F}/scan/points)"
else
    fail "No radar panel topics found after 10 s"
fi

# ── Step 3: Start detection_node ──────────────────────────────────────────────
section "3. detection_node"

ros2 run radar_fusion detection_node 2>/dev/null &
DET_PID=$!

# Poll until /radar/targets appears (up to 12 s)
TARGETS_FOUND=0
for i in $(seq 1 24); do
    if ros2 topic list 2>/dev/null | grep -q "^/radar/targets$"; then
        TARGETS_FOUND=1; break
    fi
    sleep 0.5
done

if [[ "$TARGETS_FOUND" -eq 1 ]]; then
    ok "/radar/targets topic exists"
else
    fail "/radar/targets not found after 12 s (detection_node may have crashed)"
fi

# ── Step 4: Read detections — sample with --once loop (continuous echo exits early) ─
section "4. Detection output"

BEST_N=0
BEST_RAW=""
for i in $(seq 1 15); do
    LINE=$(timeout 3 ros2 topic echo /radar/targets --once --full-length 2>/dev/null | grep "^data:" || true)
    [[ -z "$LINE" ]] && continue
    RAW="${LINE#data: }"; RAW="${RAW#\'}"; RAW="${RAW%\'}"
    N=$(printf '%s' "$RAW" | python3 -c "import sys,json; print(json.loads(sys.stdin.read()).get('n_targets',0))" 2>/dev/null || echo 0)
    if [[ "$N" -gt "$BEST_N" ]]; then
        BEST_N=$N; BEST_RAW="$RAW"
    fi
    [[ "$BEST_N" -gt 0 ]] && break
done

if [[ "$BEST_N" -gt 0 ]]; then
    ok "Detection confirmed: $BEST_N target(s)"
    printf '%s' "$BEST_RAW" | python3 -c "
import sys, json
d = json.loads(sys.stdin.read())
for t in d.get('targets', [])[:4]:
    print(f\"  → {t['id']}  R={t['range_m']}m  Az={t['az_deg']}°  Panel={t['panel']}\")
" 2>/dev/null || true
elif [[ -z "$BEST_RAW" ]]; then
    fail "No messages received on /radar/targets in 15 attempts"
else
    fail "15 messages sampled — all had n_targets=0 (detection_node not clustering)"
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
