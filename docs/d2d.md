# MBC-3 Drone-to-Drone (D2D) Communication
## Aran Technologies — Design, Implementation & Gap Analysis

---

## 1. Why D2D

Current architecture routes ALL inter-drone coordination through GCS (Flask/5000):

```
        GCS (Flask/5000)
       ↑↑↑↑↑     ↓↓↓↓↓
   DRONE-0..4   DRONE-0..4
  (push pos)  (poll leader)
```

**Problem:** GCS goes down → swarm goes blind. Not suitable for IAF deployment.

**D2D goal:** Drones communicate directly. GCS receives telemetry only — not a control path. Swarm is autonomous if GCS goes offline.

---

## 2. Transport Layer

| Property | Choice | Reason |
|---|---|---|
| Protocol | UDP multicast | No routing needed ≤5 nodes; single socket serves all |
| Group | `224.1.1.1:14900` | Link-local multicast, works on LAN + loopback SITL |
| Source port | `15000 + drone_idx` | Identifies sender without parsing payload |
| MTU | 4096 bytes (soft) | Fits under Ethernet MTU with headroom |
| Rate limit | 10 Hz max | Avoids saturation on tactical radio |
| Encryption | HMAC-SHA256 (demo) / AES-128-GCM (production) | Pre-shared swarm key |

**SITL:** All 5 coroutines on same host. `IP_MULTICAST_LOOP=1` ensures loopback delivery. No routing config needed.

**Real hardware:** Same code, bind to tactical radio interface (RFD900x in broadcast mode, or Silvus SC4200 MANET).

---

## 3. Message Format

```json
{
  "src": "DRONE-4",
  "idx": 4,
  "t":   1748293847.312,
  "type": "HB",
  "sig": "a3f8c2d1",
  "...payload fields..."
}
```

All fields compact (short keys) for radio MTU budget.

---

## 4. Message Types

### HB — Heartbeat @ 2 Hz (all drones)
```json
{
  "type": "HB",
  "lat":  47.3977,
  "lon":  8.5456,
  "alt":  498.3,
  "spd":  19.8,
  "hdg":  137.0,
  "arm":  true,
  "pha":  "SURVEY",
  "bat":  78.2,
  "wp_cur": 4,
  "wp_tot": 9,
  "ldr":  false
}
```
Recipients: update peer table, drive election, ASP markers without GCS.

### LEAD — Leader keepalive @ 0.2 Hz (leader only)
```json
{
  "type": "LEAD",
  "ldr":  4,
  "eid":  2
}
```

### ELECT — Bully election trigger (any drone on leader silence)
```json
{
  "type": "ELECT",
  "cand": 3,
  "eid":  3,
  "why":  "DRONE-4 silent 15s"
}
```

### RADAR — Radar track share @ 5 Hz (leader + all active radars)
```json
{
  "type": "RADAR",
  "tracks": [
    {"id":"TGT_1","lat":47.401,"lon":8.547,"alt":12.3,"rng":842,"conf":0.91}
  ],
  "scan": 4421
}
```

### REASSIGN — Mission reallocation (leader → specific drone)
```json
{
  "type":    "REASSIGN",
  "target":  2,
  "reason":  "DRONE-4 failed at WP 6/9",
  "wps":     [[47.3985, 8.5470], [47.3988, 8.5475]],
  "priority":"HIGH",
  "eid":     3
}
```
**Requires ACK.** Sender retries 3× at 500ms. No ACK after 3 → escalate to GCS.

### ACK — Acknowledge critical messages
```json
{
  "type": "ACK",
  "ref_type": "REASSIGN",
  "ref_eid":  3,
  "ok":       true
}
```

---

## 5. Network Topology

```
DRONE-0 ←──────────────→ DRONE-1
   ↑  ↘               ↗  ↑
   │    ──→ DRONE-4 ←──   │
   ↓  ↗               ↘  ↓
DRONE-3 ←──────────────→ DRONE-2

All nodes: multicast group 224.1.1.1:14900
```

---

## 6. Bully Election Algorithm

```
Normal:
  DRONE-4 sends HB every 500ms
  All peers reset DRONE-4 liveness timer on each HB

DRONE-4 fails:
  DRONE-3 detects: last_hb[4] > DEATH_TIMEOUT (15s)
  DRONE-3 sends: ELECT{cand=3, eid=3}

  DRONE-2 receives (idx=2 < 3): no counter-bid
  DRONE-1 receives (idx=1 < 3): no counter-bid
  DRONE-0 receives (idx=0 < 3): no counter-bid

  After ELECTION_RACE_S (2s): DRONE-3 self-declares
  DRONE-3 sends: LEAD{ldr=3, eid=3}
  All peers accept → POST /api/leader → radar_sim switches

Election latency: 15s timeout + 2s race = ~17s total
```

**Improvement (GAP-2):** elect on score, not index:
```
score = idx × 0.3 + battery × 0.4 + stability × 0.3
```
Prevents electing a low-battery or damaged drone as leader.

---

## 7. Implementation: `src/d2d_node.py`

**Classes:**
- `_D2DProtocol(asyncio.DatagramProtocol)` — UDP datagram handler
- `D2DNode` — main node class

**D2DNode key attributes:**
```python
idx:           int               # drone index 0-4
state:         dict              # shared ref to drone_states[idx]
peer_last_hb:  dict[int, float]  # last HB time per peer
peer_state:    dict[int, dict]   # latest HB fields per peer
leader_idx:    Optional[int]     # current elected leader
election_id:   int               # monotonically increasing
```

**D2DNode async loops:**
```
run()
  ├─ _hb_loop()         2 Hz HB broadcast
  ├─ _lead_loop()       0.2 Hz LEAD keepalive (leader only)
  └─ _election_watch()  1 Hz: check liveness, resolve elections
  recv via _D2DProtocol.datagram_received() → _handle()
```

**Integration in `swarm_mission.py`:**
```python
d2d_nodes = [D2DNode(i, drone_states[i]) for i in range(NUM_DRONES)]
d2d_tasks = [asyncio.create_task(d2d_nodes[i].run()) for i in range(NUM_DRONES)]
```

On leader change: D2DNode POSTs `/api/leader` → `radar_sim.py` and GCS update. Existing interface unchanged.

---

## 8. LLM / SLM Operator Interface

### What LLM is NOT good for
WP reassignment is a **math problem** (nearest-neighbour assignment). Deterministic algorithm is faster, correct, verifiable:
```python
def reassign_greedy(failed_wps, alive_drones):
    for wp in failed_wps:
        nearest = min(alive_drones, key=lambda d: haversine(d.pos, wp))
        nearest.extra_wps.append(wp)
```

### What LLM IS good for: Operator Intent Translation

```
Operator: "enemy vehicles near sector C, prioritize thermal imaging of river"

LLM interprets:
  → elevate BRAVO-1 River Crossing to priority 1
  → insert thermal pass at 200m AGL
  → reroute nearest available drone
  → update ASP with new threat zone

Output: structured mission delta JSON → D2D REASSIGN
```

This is genuinely novel — no deterministic algorithm can parse natural language mission intent.

### SLM Options

| Model | Where | Latency | Cost |
|---|---|---|---|
| Claude Haiku 4.5 | GCS + internet | ~500ms | ₹0.01/decision |
| Ollama + Phi-3 Mini 3.8B | GCS local CPU | 2–5s | free |
| LLaMA 3.2 1B | RPi 5 on drone | 5–15s | free, embedded |
| Rule-based fallback | always | <1ms | free |

**Recommendation:** Claude Haiku 4.5 for demo. Rule-based fallback when API unavailable.

### Prompt Design

```
System:
  You are a swarm mission controller for 5 UAVs.
  Respond ONLY with valid JSON. No explanation.
  Never assign drones in mission_context.failed_drones.
  Validate all coordinates are within mission_bounds.

User:
  Mission context: {mission_context}
  Swarm state: {swarm_state}
  Operator command: "{operator_input}"
  
  Respond: {"action": "REASSIGN|PRIORITIZE|ABORT|EXTEND",
            "assignments": [...], "reason": "..."}
```

### Output Validation (critical — GAP-5)

```python
def validate_llm_plan(plan, mission_bounds, alive_drones, failed_drones):
    for a in plan["assignments"]:
        assert a["drone"] not in failed_drones      # can't assign dead drone
        assert a["drone"] in alive_drones
        for wp in a["waypoints"]:
            assert in_bounds(wp, mission_bounds, margin_m=500)
            assert not in_any_nfz(wp)
    return True
```

If validation fails → reject LLM output → use greedy fallback.

---

## 9. Revised Full Architecture

```
STRATEGIC LAYER (slow, LLM, 500ms–2s):
  Operator natural language
       ↓
  Claude Haiku 4.5 / Phi-3 Mini
       ↓
  validate_llm_plan()
       ↓
  Structured mission delta

TACTICAL LAYER (fast, deterministic, <1ms):
  D2D HB → failure detect (15s timeout)
       ↓
  greedy WP reassign + battery check
       ↓
  D2D REASSIGN (with ACK)
       ↓
  MAVSDK mission re-upload
       ↓
  drone_states phase = "COVERING-DRONE-N"

SENSOR LAYER (continuous):
  All 5 radars active → D2D RADAR @ 5Hz → leader fuses → ASP
```

---

## 10. Gap Analysis

### GAP-1 🔴 Sequential mission undermines swarm concept
- **Problem:** 5 drones cover same area one at a time. Single drone can do this.
- **Fix:** Partition survey grid into N slices. Drone i covers slice i. Parallel execution.
- **Effort:** 2 days
- **Impact:** Transforms demo from "5 drones wait in line" to "true swarm parallel ISR"

### GAP-2 🔴 Leader elected by index, not capability
- **Problem:** Highest-index drone wins even if battery=5%, damaged, at edge of range.
- **Fix:** Election score = `idx×0.3 + battery×0.4 + stability×0.3`
- **Effort:** 4 hours

### GAP-3 🔴 No ACK on critical D2D messages
- **Problem:** REASSIGN/LEAD packet drop (5–15% on real radio) → drone misses new mission silently.
- **Fix:** ACK message type + 3× retry at 500ms intervals.
- **Effort:** 3 hours

### GAP-4 🟡 Only leader radar feeds ASP — 4 radars idle
- **Problem:** 5-drone swarm with 1 active radar = no coverage gain over single drone.
- **Fix:** All drones broadcast RADAR tracks via D2D. Leader fuses: cluster within 20m = same target. Confidence += 0.1 per confirming drone.
- **Effort:** 1 day
- **Impact:** Multi-drone radar fusion — genuine swarm sensing capability.

### GAP-5 🔴 No LLM output validation
- **Problem:** LLM can hallucinate waypoints outside mission area, assign dead drones.
- **Fix:** Validate all LLM outputs before execution. Reject + fallback on failure.
- **Effort:** 2 hours

### GAP-6 🔴 No altitude separation for parallel flight
- **Problem:** Parallel mission → possible collision at survey grid boundaries.
- **Fix:** Drone i flies at `CRUISE_ALT + i×10m`. Zero code beyond one line. No collision.
- **Effort:** 30 minutes

### GAP-7 🟠 No battery model
- **Problem:** SITL drones fly forever. Reallocation ignores battery. LLM gives impossible assignments.
- **Fix:** Simulate drain: `bat = 100 - (elapsed_s / FLIGHT_TIME_S) × 100`. Broadcast in HB.
- **Effort:** 3 hours

### GAP-8 🟠 No D2D authentication
- **Problem:** Adversary injects fake ELECT/REASSIGN → hijacks swarm leader / redirects drones.
- **Fix:** HMAC-SHA256 on every message. Pre-shared swarm key via env var `SWARM_KEY`.
- **Effort:** 1 hour

### GAP-9 🟡 Radar targets are static
- **Problem:** Fixed Gazebo targets never move. ISR demo shows 5 stationary dots.
- **Fix:** Circular/linear motion in `radar_sim.py`:
  ```python
  tx = base_x + 50 * math.cos(time.time() * 0.1)
  ty = base_y + 50 * math.sin(time.time() * 0.1)
  ```
- **Effort:** 2 hours

### GAP-10 🟡 LLM has no memory between incidents
- **Problem:** Incident 2 LLM call has no knowledge of incident 1. May reassign already-dead drone.
- **Fix:** Persist `mission_context` dict across calls. Include all prior failures and reassignments.
- **Effort:** 1 hour

### GAP-11 🟠 D2D assumes all drones hear each other
- **Problem:** 500m formation spread → edge drones may not receive each other's multicast.
- **Fix (demo):** Not an issue on loopback. Fix for real hardware: leader acts as relay for drones that miss peer messages.
- **Effort:** 1 day (real hardware only)

### GAP-12 🟠 No RadCom implementation
- **Concept:** 6-panel FMCW array used for sensing AND D2D via slow-time phase modulation.
- **Status:** Design only. Implementation requires custom radar firmware (TI AWR1843 DSP).
- **For submission:** Include as Phase II innovation claim. Data rate: 1–5 kbps slow-time.

---

## 11. Priority Build Order (9 days to May 31)

| Day | Task | Gap |
|---|---|---|
| Day 1 | Altitude separation (30 min) | G6 |
| Day 1 | Parallel mission partition | G1 |
| Day 2 | Multi-drone radar fusion via D2D RADAR | G4 |
| Day 2 | ACK for REASSIGN/LEAD | G3 |
| Day 3 | `src/mission_ai.py` — Claude Haiku operator interface | — |
| Day 3 | LLM validation layer | G5 |
| Day 4 | Battery model + broadcast | G7 |
| Day 4 | Moving radar targets | G9 |
| Day 5 | LLM mission context memory | G10 |
| Day 5 | HMAC auth on D2D | G8 |
| Day 6–7 | Demo video recording (3 clips) | — |
| Day 8 | Merge chain: phase6 → phase2 → feature/6-panel → main | — |
| Day 9 | IAF registration + submission | — |

---

## 12. Real-World Hardware Path

| Layer | SITL | Real drone |
|---|---|---|
| Transport | UDP multicast loopback | RFD900x broadcast / Silvus SC4200 MANET |
| D2D code | unchanged | unchanged (same UDP socket API) |
| Range | localhost | RFD900x: 40km LOS / WiFi 5G: 2km LOS |
| Bandwidth | unlimited | RFD900x: 115kbps shared |
| Encryption | HMAC demo | AES-128-GCM + ECDH key exchange |
| Compute | GCS x86 | Companion: RPi 5 (Phi-3 Mini 3.8B local SLM) |

---

## 13. MBC-3 Submission Claims

> *"The swarm uses FANET-style UDP multicast with Bully leader election. GCS is telemetry display only — swarm is fully autonomous if GCS goes offline. Meets req 2.14 (graceful degradation) at both drone-failure and ground-station-failure level."*

> *"6-panel fixed FMCW array provides simultaneous 360° azimuth at 100 Hz with zero moving parts — eliminating scan-rate-induced track breaks and mechanical failure modes."*

> *"Unlike mechanically-scanned sensors, the fixed panel array enables Dual-Function Radar Communication (DFRC) via slow-time phase modulation — inter-drone coordination over the radar channel without a separate RF link (Phase II)."*

> *"LLM operator interface translates natural language mission commands into structured mission deltas. Tactical reallocation on drone failure uses deterministic greedy assignment (<1ms) — LLM handles strategic intent, not real-time optimization."*

---

## 14. Files

| File | Role |
|---|---|
| `src/d2d_node.py` | D2DNode class — HB, LEAD, ELECT, RADAR, REASSIGN |
| `src/swarm_mission.py` | Integrates D2DNode per drone coroutine |
| `src/mission_ai.py` | LLM operator interface (Claude Haiku / Phi-3 fallback) |
| `src/leader_election.py` | GCS-polling fallback election (runs in parallel) |
| `src/radar_sim.py` | Pose-based radar + multi-drone track fusion (planned) |
| `docs/d2d.md` | This file |

---

*Last updated: 2026-05-22 — Aran Technologies MBC-3 Phase 0*
