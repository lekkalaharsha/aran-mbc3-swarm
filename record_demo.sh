#!/usr/bin/env bash
# record_demo.sh — MBC-3 Phase 0 Swarm Demo Video Recorder
# Aran Technologies | aranrobotics@gmail.com
#
# Produces: ~/mbc3_phase0_demo.mp4  (~4 minutes)
#
# USAGE (run from your desktop terminal):
#   bash ~/Documents/aran_mbc/record_demo.sh
#
# Layout:
#   Full-screen GCS dashboard at http://localhost:5000 (Firefox)
#   + swarm terminal visible on left half of screen
#   At T+120s: kill DRONE-2 to demonstrate leader failover / redistribution
set -eo pipefail

ARAN="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="$HOME/mbc3_phase0_demo.mp4"
DISPLAY_VAR="${DISPLAY:-:1}"
DURATION=300   # 5 minutes — startup(90s) + mission(120s) + failover+redistrib(90s)

export PATH="$HOME/.local/bin:$PATH"

RED='\033[0;31m'; GRN='\033[0;32m'; YLW='\033[0;33m'; CYN='\033[0;36m'; RST='\033[0m'
log()  { echo -e "${CYN}[record]${RST} $*"; }
ok()   { echo -e "${GRN}[record] ✓${RST} $*"; }
warn() { echo -e "${YLW}[record] ⚠${RST} $*"; }
err()  { echo -e "${RED}[record] ✗${RST} $*"; exit 1; }

echo ""
echo -e "${CYN}╔══════════════════════════════════════════════════════╗"
echo    "║   MBC-3 Phase 0 Demo — Swarm Video Recorder          ║"
echo    "║   Aran Technologies | aranrobotics@gmail.com              ║"
echo -e "╚══════════════════════════════════════════════════════╝${RST}"
echo ""

# ── Preflight ─────────────────────────────────────────────────────────────────
command -v ffmpeg &>/dev/null || err "ffmpeg not found — check PATH"

SCREEN=$(xrandr 2>/dev/null | grep " connected primary" | grep -oP '\d+x\d+' | head -1)
SCREEN="${SCREEN:-1920x1080}"
W=$(echo "$SCREEN" | cut -dx -f1)
H=$(echo "$SCREEN" | cut -dx -f2)
log "Display: $DISPLAY_VAR  Resolution: ${W}x${H}"

# ── Kill any stale swarm processes ────────────────────────────────────────────
log "Clearing stale processes..."
pkill -9 -f "bin/px4"       2>/dev/null || true
pkill -9 -f "gz sim"        2>/dev/null || true
pkill -9 -f "telemetry_web" 2>/dev/null || true
pkill -9 -f "swarm_mission" 2>/dev/null || true
pkill -9 -f "leader_election" 2>/dev/null || true
pkill -9 -f "radar_sim"     2>/dev/null || true
pkill -9 -f "mavsdk_server" 2>/dev/null || true
rm -f /tmp/px4_swarm_pid_*
rm -rf /tmp/px4_swarm_1 /tmp/px4_swarm_2 /tmp/px4_swarm_3 /tmp/px4_swarm_4
sleep 2
ok "Clean"

# ── Window 1 (left) — swarm_launch.sh ────────────────────────────────────────
log "Opening swarm launch terminal (left half)..."

SWARM_CMD="bash -c '\
export DISPLAY=${DISPLAY_VAR}; \
export PATH=$HOME/.local/bin:\$PATH; \
bash ${ARAN}/swarm_launch.sh; \
echo \"\"; echo \" Swarm stopped — window closes in 30s\"; sleep 30'"

gnome-terminal \
    --title="T1: MBC-3 Swarm Launch" \
    --geometry="120x40+0+0" \
    -- bash -c "$SWARM_CMD" &

log "Waiting 15s for swarm to start GCS (flask needs PX4 instance 0 + Gazebo up)..."
sleep 15

# ── Window 2 (right) — kill DRONE-2 after 120s ───────────────────────────────
log "Scheduling DRONE-2 kill at T+120s (leader failover demo)..."

KILL_CMD="bash -c '\
export DISPLAY=${DISPLAY_VAR}; \
echo \"\"; \
echo \" ╔══════════════════════════════════════════════════════╗\"; \
echo \" ║  Leader Failover Demo — DRONE-2 will be killed       ║\"; \
echo \" ║  Watch GCS for redistribution + new leader election  ║\"; \
echo \" ╚══════════════════════════════════════════════════════╝\"; \
echo \"\"; \
echo \" Waiting 150s for swarm to reach cruise altitude...\"; \
bash ${ARAN}/tools/kill_drone_sim.sh 2 150; \
echo \"\"; echo \" Failover demo complete.\"; sleep 60'"

gnome-terminal \
    --title="T2: Leader Failover Demo" \
    --geometry="90x20+960+600" \
    -- bash -c "$KILL_CMD" &

# ── Wait for GCS to be reachable ─────────────────────────────────────────────
log "Polling http://localhost:5000 (up to 300s — swarm needs ~90s to start)..."
GCS_UP=0
for i in $(seq 1 150); do
    sleep 2
    if curl -sf http://localhost:5000/ -o /dev/null 2>/dev/null; then
        GCS_UP=1
        ok "GCS reachable at T+$((i*2))s"
        break
    fi
    if (( i % 15 == 0 )); then
        log "  Still waiting for GCS... ($((i*2))s)"
    fi
done

if [[ "$GCS_UP" -eq 0 ]]; then
    warn "GCS not reachable after 300s — opening browser anyway (may show loading)"
fi

# ── Open Firefox to GCS dashboard ────────────────────────────────────────────
log "Opening GCS dashboard in Firefox..."
firefox --new-window "http://localhost:5000" &
sleep 5

# ── Brief settle before recording ────────────────────────────────────────────
log "Settling 5s before recording starts..."
sleep 5

# ── Start ffmpeg recording ────────────────────────────────────────────────────
log "Recording ${DURATION}s → $OUT"
echo ""
echo -e "${YLW}  ► RECORDING ${DURATION}s — do not move or cover windows${RST}"
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
if [[ -f "$OUT" ]] && [[ $(stat -c%s "$OUT") -gt 500000 ]]; then
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
