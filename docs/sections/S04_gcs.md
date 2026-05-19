# S04 — GCS Dashboards

**Status:** ISR GCS ✅ done | ASP GCS 🔲 Phase 2 (not started)

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

## ASP GCS (to be built — Phase 2)

Separate Flask app. MBC-3 requirement: consolidated ASP on single browser screen.

**Required features:**
| Feature | Description |
|---------|-------------|
| Track table | ID, range, azimuth, velocity, source drones |
| Sector map | 360° polar display with track positions |
| Leader identity | Which drone is currently leader |
| Decision log | LLM tactical outputs (Phase 7) |
| JSON session recording | Timestamped log per session |
| 2.5 Hz refresh | SocketIO push from `/swarm/tracks` |

**Input:** `/swarm/tracks` + `/swarm/situation` from `fusion_node`  
**Branch:** `test/phase2-radar-web` (not started)

---

## Open Tasks

- [ ] Phase 2: build ASP GCS Flask app
- [ ] Connect to `/swarm/tracks` SocketIO push
- [ ] Sector map Leaflet overlay
- [ ] Session recording to JSON
