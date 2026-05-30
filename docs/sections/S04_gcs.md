# S04 — GCS Dashboards

**Status:** ISR GCS ✅ done | ASP GCS ✅ done (swarm_telemetry_web.py)

---

## ISR GCS (telemetry_web.py) — EXISTING

Flask/SocketIO server. Browser at `http://localhost:5000`.

| Feature | Status |
|---------|--------|
| Leaflet map + drone position | ✅ |
| Sector overlay (LiDAR/radar) | ✅ |
| Target panel (secondary ISR) | ✅ |
| NFZ display | ✅ |
| Mission phase indicator | ✅ |
| 2.5 Hz telemetry refresh | ✅ |

Change rule: ask before changing HTML/JS frontend.

---

## ASP GCS (swarm_telemetry_web.py) — COMPLETE

`src/swarm_telemetry_web.py` — Flask/SocketIO server port 5000. Military theme GCS dashboard for 5-drone swarm.

| Feature | Status |
|---------|--------|
| 5-drone position table | ✅ |
| Radar polar display (6-panel FOV) | ✅ |
| Contact alerts + track table | ✅ |
| Leader identity indicator | ✅ |
| Follow-target mode (`/api/track_state`) | ✅ |
| D2D health per drone | ✅ |
| 2.5 Hz SocketIO refresh | ✅ |
| Thread-safe shared state (`_shared_lock`) | ✅ |
| GCS_TOKEN auth on mutation endpoints | ✅ |
| CORS restricted to localhost | ✅ |
| Mission watchdog + STALE badge | ✅ |

**Input:** `POST /asp_update` from `radar_sim.py`, `POST /lidar_update` from each drone's mission script  
**Branch:** `main`

---

## Open Tasks

- [x] Phase 2: build ASP GCS Flask app
- [x] Connect to radar tracks SocketIO push
- [x] Sector map polar overlay
- [ ] Phase 7: LLM decision log integration
