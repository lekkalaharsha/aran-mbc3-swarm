#!/usr/bin/env bash
# record_single_drone.sh — MBC-3 Single-Drone ISR Demo Video Recorder
# Aran Technologies | aranrobotics@gmail.com
#
# Produces: ~/Documents/aran_mbc/mbc3_single_drone_demo.mp4  (~4 minutes)
#
# Layout: Gazebo left (960x1080) | GCS Firefox right (960x1080)
#
# USAGE (run from your desktop terminal):
#   bash ~/Documents/aran_mbc/record_single_drone.sh
#
# Requires: wmctrl  →  sudo apt install -y wmctrl
#
# Sequence:
#   T+0s:    launch.sh starts (PX4 SITL + Gazebo GUI + GCS + mission)
#   T+~30s:  drone arms and climbs to 30m
#   T+~60s:  PHASE 2 survey (11 WPs)
#   T+~120s: PHASE 3 PRIMARY orbit (50m radius)
#   T+~180s: PHASE 5 RTL — MISSION COMPLETE (secondary orbits skipped)
#   T+240s:  recording stops
set -eo pipefail

ARAN="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="${ARAN}/mbc3_single_drone_demo.mp4"
DISPLAY_VAR="${DISPLAY:-:1}"
DURATION=240   # 4 minutes — survey + primary orbit + RTL only

export PATH="$HOME/.local/bin:$PATH"

RED='\033[0;31m'; GRN='\033[0;32m'; YLW='\033[0;33m'; CYN='\033[0;36m'; RST='\033[0m'
log()  { echo -e "${CYN}[record]${RST} $*"; }
ok()   { echo -e "${GRN}[record] ✓${RST} $*"; }
warn() { echo -e "${YLW}[record] ⚠${RST} $*"; }
err()  { echo -e "${RED}[record] ✗${RST} $*"; exit 1; }

echo ""
echo -e "${CYN}╔══════════════════════════════════════════════════════╗"
echo    "║   MBC-3 Single-Drone ISR Demo — Video Recorder       ║"
echo    "║   Aran Technologies | aranrobotics@gmail.com              ║"
echo -e "╚══════════════════════════════════════════════════════╝${RST}"
echo ""

# ── Preflight ─────────────────────────────────────────────────────────────────
command -v ffmpeg  &>/dev/null || err "ffmpeg not found — check PATH (~/.local/bin/ffmpeg)"
command -v wmctrl  &>/dev/null || err "wmctrl not found — install: sudo apt install -y wmctrl"

SCREEN=$(xrandr 2>/dev/null | grep " connected primary" | grep -oP '\d+x\d+' | head -1)
SCREEN="${SCREEN:-1920x1080}"
W=$(echo "$SCREEN" | cut -dx -f1)
H=$(echo "$SCREEN" | cut -dx -f2)
HALF=$(( W / 2 ))
log "Display: $DISPLAY_VAR  Resolution: ${W}x${H}  Half: ${HALF}px"

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

# ── Launch ISR stack (Gazebo GUI, secondary orbits skipped) ──────────────────
log "Opening single-drone ISR launch terminal..."

LAUNCH_CMD="bash -c '\
export DISPLAY=${DISPLAY_VAR}; \
export PATH=$HOME/.local/bin:\$PATH; \
export MBC3_SKIP_SECONDARY=1; \
bash ${ARAN}/launch.sh; \
echo \"\"; echo \" Mission ended — window closes in 30s\"; sleep 30'"

gnome-terminal \
    --title="MBC-3 ISR Mission Log" \
    --geometry="100x30+0+0" \
    -- bash -c "$LAUNCH_CMD" &

log "Waiting 25s for PX4 + Gazebo + GCS startup..."
sleep 25

# ── Poll for GCS ──────────────────────────────────────────────────────────────
log "Polling http://localhost:5000 (timeout 90s)..."
GCS_UP=0
for i in $(seq 1 45); do
    sleep 2
    if curl -sf http://localhost:5000/ -o /dev/null 2>/dev/null; then
        GCS_UP=1
        ok "GCS reachable at T+$((i*2 + 25))s"
        break
    fi
    if (( i % 10 == 0 )); then
        log "  Still waiting for GCS... ($((i*2 + 25))s)"
    fi
done
[[ "$GCS_UP" -eq 0 ]] && warn "GCS not reachable — opening browser anyway"

# ── Open Firefox to GCS dashboard ────────────────────────────────────────────
log "Opening GCS dashboard in Firefox..."
firefox --new-window "http://localhost:5000" &
sleep 6

# ── Position windows: Gazebo left, Firefox right ──────────────────────────────
log "Positioning windows (Gazebo left ${HALF}x${H}, Firefox right ${HALF}x${H})..."

# Remove window decorations that shift geometry, then tile
# wmctrl: -e gravity,x,y,w,h  (gravity 0 = default)
for attempt in 1 2 3; do
    sleep 2
    # Gazebo window — title contains "Gazebo" or "gz"
    wmctrl -r "Gazebo" -e "0,0,0,${HALF},${H}" 2>/dev/null || \
    wmctrl -r "gz"     -e "0,0,0,${HALF},${H}" 2>/dev/null || true
    # Firefox GCS window
    wmctrl -r "localhost:5000" -e "0,${HALF},0,${HALF},${H}" 2>/dev/null || \
    wmctrl -r "Mozilla Firefox" -e "0,${HALF},0,${HALF},${H}" 2>/dev/null || true
done
ok "Windows positioned"

# ── Settle before recording ────────────────────────────────────────────────────
log "Settling 3s before recording..."
sleep 3

# ── Start ffmpeg ──────────────────────────────────────────────────────────────
log "Recording ${DURATION}s → $OUT"
echo ""
echo -e "${YLW}  ► RECORDING ${DURATION}s — Gazebo left | GCS right — do not move windows${RST}"
echo ""
echo "  Mission phases (short demo — secondary orbits skipped):"
echo "    T+~30s   PHASE 2 — Survey (11 WPs)"
echo "    T+~90s   PHASE 3 — PRIMARY orbit (50m radius)"
echo "    T+~180s  PHASE 5 — RTL + map save"
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
