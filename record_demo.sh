#!/usr/bin/env bash
# record_demo.sh — MBC-3 Phase 0 Demo Video Recorder
# Boson Motors | eeindia@bosonmotors.com
#
# Produces: ~/mbc3_phase0_demo.mp4  (~60 seconds)
#
# USAGE (run from your desktop terminal, NOT via Claude Code):
#   bash ~/Documents/aran_mbc/record_demo.sh
#
# Layout:
#   Left pane  — radar_fusion detection_node  (live detections at 5 Hz)
#   Right pane — fly_demo.sh                  (mission sequence + live targets)
set -eo pipefail

ARAN="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_SETUP="$HOME/ros2_ws/install/setup.bash"
OUT="$HOME/mbc3_phase0_demo.mp4"
DISPLAY_VAR="${DISPLAY:-:1}"
DURATION=70   # seconds to record (demo ~45s + buffer)

RED='\033[0;31m'; GRN='\033[0;32m'; YLW='\033[0;33m'; CYN='\033[0;36m'; RST='\033[0m'
log()  { echo -e "${CYN}[record]${RST} $*"; }
ok()   { echo -e "${GRN}[record] ✓${RST} $*"; }
warn() { echo -e "${YLW}[record] ⚠${RST} $*"; }
err()  { echo -e "${RED}[record] ✗${RST} $*"; exit 1; }

echo ""
echo -e "${CYN}╔══════════════════════════════════════════════════════╗"
echo    "║   MBC-3 Phase 0 Demo — Video Recorder                ║"
echo    "║   Boson Motors | eeindia@bosonmotors.com              ║"
echo -e "╚══════════════════════════════════════════════════════╝${RST}"
echo ""

# ── Preflight ─────────────────────────────────────────────────────────────────
export PATH="$HOME/.local/bin:$PATH"
source /opt/ros/jazzy/setup.bash

[[ -f "$WS_SETUP" ]] || err "ros2_ws not built. Run: bash $ARAN/setup_ws.sh"
source "$WS_SETUP"

# ── Install deps ─────────────────────────────────────────────────────────────
if ! command -v ffmpeg &>/dev/null; then
    log "Installing ffmpeg..."
    sudo apt-get install -y -q ffmpeg
    ok "ffmpeg installed"
fi

if ! command -v tmux &>/dev/null; then
    log "Installing tmux..."
    sudo apt-get install -y -q tmux
    ok "tmux installed"
fi

# ── Screen geometry ───────────────────────────────────────────────────────────
SCREEN=$(xrandr 2>/dev/null | grep " connected primary" | grep -oP '\d+x\d+' | head -1)
SCREEN="${SCREEN:-1920x1080}"
log "Display: $DISPLAY_VAR  Resolution: $SCREEN"

# ── tmux session layout ───────────────────────────────────────────────────────
SESSION="mbc3_demo"
tmux kill-session -t "$SESSION" 2>/dev/null || true

# Create session with large window (matches 1920×1080 at ~12pt font)
tmux new-session -d -s "$SESSION" -x 220 -y 50

# Left pane — detection_node
tmux send-keys -t "$SESSION:0.0" \
    "source /opt/ros/jazzy/setup.bash && source $WS_SETUP && \
     echo '' && \
     echo '  ┌─────────────────────────────────────────────────────┐' && \
     echo '  │  AERIS-10 FMCW Radar — detection_node               │' && \
     echo '  │  MBC-3 | Boson Motors | eeindia@bosonmotors.com      │' && \
     echo '  └─────────────────────────────────────────────────────┘' && \
     echo '' && \
     ros2 run radar_fusion detection_node" Enter

# Split right — fly_demo.sh
tmux split-window -h -t "$SESSION:0.0"
tmux send-keys -t "$SESSION:0.1" \
    "sleep 4 && bash $ARAN/fly_demo.sh" Enter

# Set pane widths (50/50)
tmux select-layout -t "$SESSION" even-horizontal

# ── Open tmux in a maximised gnome-terminal ───────────────────────────────────
log "Opening demo terminal..."
gnome-terminal \
    --geometry=220x52 \
    --title="MBC-3 Phase 0 Demo — Boson Motors" \
    -- bash -c "tmux attach -t $SESSION; bash" &
TERM_PID=$!

log "Waiting 6s for terminal + nodes to settle..."
sleep 6

# ── Start ffmpeg recording ────────────────────────────────────────────────────
log "Recording ${DURATION}s to: $OUT"
echo ""
echo -e "${YLW}  ► Recording started — do not move windows${RST}"
echo ""

ffmpeg -y \
    -f x11grab \
    -r 30 \
    -s "$SCREEN" \
    -i "${DISPLAY_VAR}.0" \
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

# ── Wait for ffmpeg to finish ─────────────────────────────────────────────────
wait $FFMPEG_PID 2>/dev/null || true

# ── Cleanup ───────────────────────────────────────────────────────────────────
tmux kill-session -t "$SESSION" 2>/dev/null || true

echo ""
if [[ -f "$OUT" ]]; then
    SIZE=$(du -h "$OUT" | cut -f1)
    ok "Recording saved: $OUT  ($SIZE)"
    echo ""
    echo -e "${GRN}  Next steps:${RST}"
    echo "  1. Preview:   vlc $OUT  (or xdg-open $OUT)"
    echo "  2. Submit to IAF MBC-3 portal with:"
    echo "     • $OUT  (this video)"
    echo "     • $ARAN/competition/Final_Vision_Document_for_MBC_3_22Apr26.pdf"
    echo "     • $ARAN/competition/Registration_form_MBC_3_final.pdf"
else
    err "Recording file not found — ffmpeg may have failed"
fi
echo ""
