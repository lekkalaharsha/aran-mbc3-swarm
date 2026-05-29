#!/usr/bin/env bash
# record_demo.sh — MBC-3 Phase 0 Demo Video Recorder
# Boson Motors | eeindia@bosonmotors.com
#
# Produces: ~/mbc3_phase0_demo.mp4  (~70 seconds)
#
# USAGE (run from your desktop terminal):
#   bash ~/Documents/aran_mbc/record_demo.sh
#
# Layout:
#   Left window  — radar_fusion detection_node  (live detections 5 Hz)
#   Right window — fly_demo.sh                  (mission + live targets)
set -eo pipefail

ARAN="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_SETUP="$HOME/ros2_ws/install/setup.bash"
OUT="$HOME/mbc3_phase0_demo.mp4"
DISPLAY_VAR="${DISPLAY:-:1}"
DURATION=70

export PATH="$HOME/.local/bin:$PATH"

RED='\033[0;31m'; GRN='\033[0;32m'; YLW='\033[0;33m'; CYN='\033[0;36m'; RST='\033[0m'
log()  { echo -e "${CYN}[record]${RST} $*"; }
ok()   { echo -e "${GRN}[record] ✓${RST} $*"; }
err()  { echo -e "${RED}[record] ✗${RST} $*"; exit 1; }

echo ""
echo -e "${CYN}╔══════════════════════════════════════════════════════╗"
echo    "║   MBC-3 Phase 0 Demo — Video Recorder                ║"
echo    "║   Boson Motors | eeindia@bosonmotors.com              ║"
echo -e "╚══════════════════════════════════════════════════════╝${RST}"
echo ""

# ── Preflight ─────────────────────────────────────────────────────────────────
source /opt/ros/jazzy/setup.bash
[[ -f "$WS_SETUP" ]] || err "ros2_ws not built. Run: bash $ARAN/setup_ws.sh"
source "$WS_SETUP"

command -v ffmpeg &>/dev/null || err "ffmpeg not found at $HOME/.local/bin/ffmpeg — already downloaded, check PATH"

SCREEN=$(xrandr 2>/dev/null | grep " connected primary" | grep -oP '\d+x\d+' | head -1)
SCREEN="${SCREEN:-1920x1080}"
W=$(echo "$SCREEN" | cut -dx -f1)
H=$(echo "$SCREEN" | cut -dx -f2)
log "Display: $DISPLAY_VAR  Resolution: ${W}x${H}"

# ── Window 1 — detection_node (left half) ────────────────────────────────────
log "Opening detection_node window (left)..."

DETECT_CMD="bash -c '\
source /opt/ros/jazzy/setup.bash; \
source $WS_SETUP; \
echo \"\"; \
echo \" ╔══════════════════════════════════════════════════════╗\"; \
echo \" ║  AERIS-10 FMCW Radar — detection_node               ║\"; \
echo \" ║  MBC-3 | Boson Motors | eeindia@bosonmotors.com      ║\"; \
echo \" ╚══════════════════════════════════════════════════════╝\"; \
echo \"\"; \
ros2 run radar_fusion detection_node; \
exec bash'"

gnome-terminal \
    --title="T1: detection_node" \
    --geometry="100x35+0+0" \
    -- bash -c "$DETECT_CMD" &

sleep 2

# ── Window 2 — fly_demo.sh (right half) ──────────────────────────────────────
log "Opening fly_demo.sh window (right)..."

DEMO_CMD="bash -c '\
sleep 3; \
bash $ARAN/fly_demo.sh; \
echo \"\"; \
echo \" Demo complete — window closes in 10s\"; \
sleep 10'"

gnome-terminal \
    --title="T2: fly_demo.sh" \
    --geometry="100x35+960+0" \
    -- bash -c "$DEMO_CMD" &

log "Waiting 8s for windows + pipeline to settle..."
sleep 8

# ── Start ffmpeg recording ────────────────────────────────────────────────────
log "Recording ${DURATION}s → $OUT"
echo ""
echo -e "${YLW}  ► RECORDING — do not move or cover the terminal windows${RST}"
echo ""

ffmpeg -y \
    -f x11grab \
    -r 30 \
    -s "${W}x${H}" \
    -i "${DISPLAY_VAR}.0+0,0" \
    -vf "scale=${W}:${H}" \
    -c:v libx264 \
    -preset medium \
    -crf 20 \
    -pix_fmt yuv420p \
    -t "$DURATION" \
    "$OUT" \
    2>/dev/null &
FFMPEG_PID=$!

# ── Progress bar ─────────────────────────────────────────────────────────────
for i in $(seq 1 $DURATION); do
    sleep 1
    pct=$(( i * 100 / DURATION ))
    bar=$(printf '#%.0s' $(seq 1 $((i * 40 / DURATION))))
    printf "\r  [%-40s] %3d%%  (%ds / ${DURATION}s)" "$bar" "$pct" "$i"
done
echo ""

wait $FFMPEG_PID 2>/dev/null || true

# ── Done ──────────────────────────────────────────────────────────────────────
echo ""
if [[ -f "$OUT" ]] && [[ $(stat -c%s "$OUT") -gt 100000 ]]; then
    SIZE=$(du -h "$OUT" | cut -f1)
    ok "Saved: $OUT  ($SIZE)"
    echo ""
    echo -e "${GRN}  Preview:${RST}  xdg-open $OUT"
    echo ""
    echo -e "${GRN}  Submit to IAF MBC-3 portal:${RST}"
    echo "    Video:  $OUT"
    echo "    Docs:   $ARAN/competition/Final_Vision_Document_for_MBC_3_22Apr26.pdf"
    echo "    Form:   $ARAN/competition/Registration_form_MBC_3_final.pdf"
else
    err "Recording failed or too small — check ffmpeg and DISPLAY"
fi
echo ""
