#!/usr/bin/env bash
# record_single_drone.sh — MBC-3 Single-Drone ISR Demo Video Recorder
# Boson Motors | eeindia@bosonmotors.com
#
# Produces: ~/mbc3_single_drone_demo.mp4  (~8 minutes)
#
# USAGE (run from your desktop terminal):
#   bash ~/Documents/aran_mbc/record_single_drone.sh
#
# Sequence:
#   T+0s:    launch.sh --headless starts (PX4 SITL + GCS + mission)
#   T+~30s:  drone arms and climbs to 30m
#   T+~60s:  PHASE 2 survey (11 WPs)
#   T+~120s: PHASE 3 PRIMARY orbit
#   T+~180s: PHASE 4.1/4.2/4.3 secondary orbits
#   T+~360s: PHASE 5 RTL — MISSION COMPLETE
#   T+480s:  recording stops
set -eo pipefail

ARAN="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="$HOME/mbc3_single_drone_demo.mp4"
DISPLAY_VAR="${DISPLAY:-:1}"
DURATION=480   # 8 minutes — covers full ISR mission + RTL

export PATH="$HOME/.local/bin:$PATH"

RED='\033[0;31m'; GRN='\033[0;32m'; YLW='\033[0;33m'; CYN='\033[0;36m'; RST='\033[0m'
log()  { echo -e "${CYN}[record]${RST} $*"; }
ok()   { echo -e "${GRN}[record] ✓${RST} $*"; }
warn() { echo -e "${YLW}[record] ⚠${RST} $*"; }
err()  { echo -e "${RED}[record] ✗${RST} $*"; exit 1; }

echo ""
echo -e "${CYN}╔══════════════════════════════════════════════════════╗"
echo    "║   MBC-3 Single-Drone ISR Demo — Video Recorder       ║"
echo    "║   Boson Motors | eeindia@bosonmotors.com              ║"
echo -e "╚══════════════════════════════════════════════════════╝${RST}"
echo ""

# ── Preflight ─────────────────────────────────────────────────────────────────
command -v ffmpeg &>/dev/null || err "ffmpeg not found — check PATH (~/.local/bin/ffmpeg)"

SCREEN=$(xrandr 2>/dev/null | grep " connected primary" | grep -oP '\d+x\d+' | head -1)
SCREEN="${SCREEN:-1920x1080}"
W=$(echo "$SCREEN" | cut -dx -f1)
H=$(echo "$SCREEN" | cut -dx -f2)
log "Display: $DISPLAY_VAR  Resolution: ${W}x${H}"

# ── Kill stale processes ───────────────────────────────────────────────────────
log "Clearing stale processes..."
pkill -9 -f "bin/px4"         2>/dev/null || true
pkill -9 -f "gz sim"          2>/dev/null || true
pkill -9 -f "telemetry_web"   2>/dev/null || true
pkill -9 -f "isr_lidar_mpc"   2>/dev/null || true
pkill -9 -f "radar_sim"       2>/dev/null || true
pkill -9 -f "mavsdk_server"   2>/dev/null || true
sleep 2
ok "Clean"

# ── Window 1 — launch.sh (full ISR stack) ────────────────────────────────────
log "Opening single-drone ISR launch terminal..."

LAUNCH_CMD="bash -c '\
export DISPLAY=${DISPLAY_VAR}; \
export PATH=$HOME/.local/bin:\$PATH; \
bash ${ARAN}/launch.sh --headless; \
echo \"\"; echo \" Mission ended — window closes in 30s\"; sleep 30'"

gnome-terminal \
    --title="MBC-3 Single-Drone ISR Mission" \
    --geometry="120x40+0+0" \
    -- bash -c "$LAUNCH_CMD" &

log "Waiting 20s for PX4 + GCS startup..."
sleep 20

# ── Poll for GCS ──────────────────────────────────────────────────────────────
log "Polling http://localhost:5000 (timeout 120s — PX4 needs ~15s after script starts)..."
GCS_UP=0
for i in $(seq 1 60); do
    sleep 2
    if curl -sf http://localhost:5000/ -o /dev/null 2>/dev/null; then
        GCS_UP=1
        ok "GCS reachable at T+$((i*2 + 20))s"
        break
    fi
    if (( i % 10 == 0 )); then
        log "  Still waiting for GCS... ($((i*2 + 20))s)"
    fi
done

if [[ "$GCS_UP" -eq 0 ]]; then
    warn "GCS not reachable — opening browser anyway (may show loading)"
fi

# ── Open Firefox to GCS dashboard ────────────────────────────────────────────
log "Opening GCS dashboard in Firefox..."
firefox --new-window "http://localhost:5000" &
sleep 5

# ── Settle ────────────────────────────────────────────────────────────────────
log "Settling 5s before recording..."
sleep 5

# ── Start ffmpeg ──────────────────────────────────────────────────────────────
log "Recording ${DURATION}s → $OUT"
echo ""
echo -e "${YLW}  ► RECORDING ${DURATION}s — do not move or cover windows${RST}"
echo -e "${YLW}  ► Watch GCS at http://localhost:5000${RST}"
echo ""
echo "  Mission phases:"
echo "    T+~30s   PHASE 2 — Survey (11 WPs)"
echo "    T+~90s   PHASE 3 — PRIMARY orbit (50m radius)"
echo "    T+~120s  PHASE 4.1 — ALPHA-2 orbit"
echo "    T+~180s  PHASE 4.2 — BRAVO-1 orbit"
echo "    T+~240s  PHASE 4.3 — CHARLIE-3 orbit"
echo "    T+~300s  PHASE 5 — RTL + map save"
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

# ── Progress bar ──────────────────────────────────────────────────────────────
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
if [[ -f "$OUT" ]] && [[ $(stat -c%s "$OUT") -gt 500000 ]]; then
    SIZE=$(du -h "$OUT" | cut -f1)
    ok "Saved: $OUT  ($SIZE)"
    echo ""
    echo -e "${GRN}  Preview:${RST}  xdg-open $OUT"
    echo ""
    echo -e "${GRN}  Submit to IAF MBC-3 portal:${RST}"
    echo "    Video:  $OUT"
    echo "    Docs:   ${ARAN}/competition/Final_Vision_Document_for_MBC_3_22Apr26.pdf"
    echo "    Form:   ${ARAN}/competition/Registration_form_MBC_3_final.pdf"
else
    err "Recording failed or too small — check ffmpeg and DISPLAY ($DISPLAY_VAR)"
fi
echo ""
