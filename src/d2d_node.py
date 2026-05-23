#!/usr/bin/env python3
"""
d2d_node.py — UDP multicast drone-to-drone communication for MBC-3 swarm.

Each drone runs one D2DNode. Messages are broadcast to multicast group
224.1.1.1:14900 and received by all peers simultaneously.

SITL: all 5 coroutines on same host. IP_MULTICAST_LOOP=1 ensures loopback
delivery. Each node binds source port 15000+idx so peers can identify sender
without parsing the payload.

Real hardware: same code, swap interface to a tactical radio bound to
the multicast group. No code changes needed.

Message types
─────────────
  HB    @ 2 Hz   — position, armed, phase (every drone)
  LEAD  @ 0.2 Hz — leader keepalive (leader only)
  ELECT @ event  — bully nomination (any drone detecting leader silence)
  RADAR @ 5 Hz   — radar track share (leader only, optional)

Integration
───────────
  swarm_mission.py creates D2DNode per drone, runs via asyncio.create_task.
  On leader change, D2DNode POSTs to /api/leader — same interface as
  leader_election.py, so radar_sim.py and GCS need no changes.
"""

import asyncio
import json
import socket
import struct
import time
from typing import Callable, Optional

import requests

D2D_GROUP       = "224.1.1.1"
D2D_PORT        = 14900
SRC_BASE        = 15000        # drone i sends/receives on port 15000+i
TTL             = 4            # multicast hops — covers LAN + one router hop
MAX_PACKET      = 4096

HB_HZ           = 2.0         # heartbeat rate
LEAD_HZ         = 0.2         # leader keepalive (every 5s)
DEATH_TIMEOUT   = 15.0        # seconds of HB silence → peer considered dead
ELECTION_RACE_S = 2.0         # bully race window before winner self-declares

GCS_LEADER_URL  = "http://localhost:5000/api/leader"


class _D2DProtocol(asyncio.DatagramProtocol):
    """asyncio UDP datagram handler — routes received packets to D2DNode."""

    def __init__(self, node: "D2DNode"):
        self._node = node

    def datagram_received(self, data: bytes, addr) -> None:
        self._node._handle(data, addr)

    def error_received(self, exc) -> None:
        pass

    def connection_lost(self, exc) -> None:
        pass


class D2DNode:
    """
    Drone-to-drone communication node.

    Usage:
        node = D2DNode(idx=2, state=drone_states[2])
        node.on_leader_change(lambda ldr, eid: print(f"new leader: {ldr}"))
        asyncio.create_task(node.run())
    """

    def __init__(self, idx: int, state: dict, gcs_url: str = GCS_LEADER_URL):
        self.idx     = idx
        self.id      = f"DRONE-{idx}"
        self.state   = state       # shared ref to drone_states[idx] — read in _hb()
        self._gcs    = gcs_url

        self.peer_last_hb:     dict[int, float] = {}
        self.peer_state:       dict[int, dict]  = {}   # latest HB fields per peer
        self.peer_radar_tracks: dict[int, list] = {}   # G4: latest RADAR tracks per peer

        self.leader_idx:  Optional[int] = None
        self.election_id: int           = 0

        self._candidate:      Optional[int] = None
        self._candidate_time: float         = 0.0
        self._running:        bool          = False

        self._on_leader_change: list[Callable] = []
        self._transport:  Optional[asyncio.DatagramTransport] = None
        self._send_sock:  Optional[socket.socket]             = None

    # ── Public API ────────────────────────────────────────────────────

    def on_leader_change(self, cb: Callable[[int, int], None]) -> None:
        """Register callback fired on every leader change: cb(leader_idx, election_id)."""
        self._on_leader_change.append(cb)

    def broadcast_radar(self, tracks: list, scan: int) -> None:
        """Leader calls this to share radar tracks with all peers over D2D."""
        self._send({"type": "RADAR", "tracks": tracks, "scan": scan})

    def get_fused_tracks(self, own_tracks: list) -> list:
        """G4: Merge own radar tracks with all received peer tracks (dedup by id)."""
        seen: dict = {}
        for t in own_tracks:
            seen[t["id"]] = t
        for peer_tracks in self.peer_radar_tracks.values():
            for t in peer_tracks:
                if t.get("id") and t["id"] not in seen:
                    seen[t["id"]] = t
        return list(seen.values())

    # ── Socket factories ──────────────────────────────────────────────

    @staticmethod
    def _make_send_sock(idx: int) -> socket.socket:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, TTL)
        s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)  # loopback for SITL
        s.bind(("", SRC_BASE + idx))
        return s

    @staticmethod
    def _make_recv_sock() -> socket.socket:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except AttributeError:
            pass
        s.bind(("", D2D_PORT))
        mreq = struct.pack("4sL", socket.inet_aton(D2D_GROUP), socket.INADDR_ANY)
        s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        return s

    # ── Send ──────────────────────────────────────────────────────────

    def _send(self, payload: dict) -> None:
        payload.update({"src": self.id, "idx": self.idx, "t": time.time()})
        try:
            data = json.dumps(payload, separators=(",", ":")).encode()
            if len(data) <= MAX_PACKET and self._send_sock:
                self._send_sock.sendto(data, (D2D_GROUP, D2D_PORT))
        except Exception:
            pass

    def _hb(self) -> None:
        self._send({
            "type": "HB",
            "lat":  self.state.get("lat",         0.0),
            "lon":  self.state.get("lon",         0.0),
            "alt":  self.state.get("alt",         0.0),
            "spd":  self.state.get("groundspeed", 0.0),
            "hdg":  self.state.get("heading",     0.0),
            "arm":  self.state.get("armed",       False),
            "pha":  self.state.get("phase",       "INIT"),
            "ldr":  self.leader_idx == self.idx,
        })

    def _announce_leader(self) -> None:
        self._send({"type": "LEAD", "ldr": self.idx, "eid": self.election_id})

    def _send_elect(self, candidate: int, reason: str = "") -> None:
        self._send({
            "type": "ELECT",
            "cand": candidate,
            "eid":  self.election_id + 1,
            "why":  reason,
        })

    # ── Receive + dispatch ────────────────────────────────────────────

    def _handle(self, data: bytes, addr) -> None:
        try:
            msg = json.loads(data)
        except Exception:
            return

        src = msg.get("idx")
        if src is None or src == self.idx:
            return   # ignore own messages

        mtype = msg.get("type")

        if mtype == "HB":
            self.peer_last_hb[src] = msg.get("t", time.time())
            self.peer_state[src]   = msg

        elif mtype == "LEAD":
            ldr = msg.get("ldr")
            eid = msg.get("eid", 0)
            if ldr is not None and eid >= self.election_id:
                self._set_leader(ldr, eid)

        elif mtype == "ELECT":
            cand = msg.get("cand", -1)
            eid  = msg.get("eid",  0)
            self._on_elect_msg(cand, eid)

        elif mtype == "RADAR":
            # G4: store peer radar tracks; leader will fuse these
            self.peer_radar_tracks[src] = msg.get("tracks", [])

    # ── Leader management ─────────────────────────────────────────────

    def _set_leader(self, ldr_idx: int, eid: int) -> None:
        changed = ldr_idx != self.leader_idx
        self.leader_idx  = ldr_idx
        self.election_id = max(self.election_id, eid)
        if changed:
            print(
                f"[D2D-{self.idx}] LEADER → DRONE-{ldr_idx}  election#{eid}",
                flush=True,
            )
            self._post_leader_gcs(ldr_idx, eid)
            for cb in self._on_leader_change:
                try:
                    cb(ldr_idx, eid)
                except Exception:
                    pass

    def _post_leader_gcs(self, ldr_idx: int, eid: int) -> None:
        try:
            requests.post(self._gcs, json={
                "leader_id":      f"DRONE-{ldr_idx}",
                "leader_model":   f"mbc3_radar_drone_{ldr_idx}",
                "since":          time.time(),
                "election_count": eid,
                "source":         "D2D",
            }, timeout=0.5)
        except Exception:
            pass

    # ── Bully election ────────────────────────────────────────────────

    def _start_election(self, reason: str) -> None:
        print(f"[D2D-{self.idx}] Election triggered — {reason}", flush=True)
        self._candidate      = self.idx
        self._candidate_time = time.time()
        self._send_elect(self.idx, reason)

    def _on_elect_msg(self, candidate: int, eid: int) -> None:
        if self.idx > candidate:
            # I outrank the candidate — counter-nominate myself
            self._send_elect(self.idx, f"outbid DRONE-{candidate}")
            self._candidate      = self.idx
            self._candidate_time = time.time()
        elif self._candidate is None or candidate > self._candidate:
            self._candidate      = candidate
            self._candidate_time = time.time()

    # ── Async loops ───────────────────────────────────────────────────

    async def _hb_loop(self) -> None:
        interval = 1.0 / HB_HZ
        while self._running:
            self._hb()
            await asyncio.sleep(interval)

    async def _lead_loop(self) -> None:
        interval = 1.0 / LEAD_HZ
        while self._running:
            if self.leader_idx == self.idx:
                self._announce_leader()
            await asyncio.sleep(interval)

    async def _election_watch(self) -> None:
        """Detect leader silence → trigger bully election → resolve winner."""
        while self._running:
            await asyncio.sleep(1.0)
            now = time.time()

            # Check leader liveness via HB timestamps
            if self.leader_idx is not None and self.leader_idx != self.idx:
                last = self.peer_last_hb.get(self.leader_idx, 0.0)
                if last > 0 and (now - last) > DEATH_TIMEOUT:
                    self._start_election(
                        f"DRONE-{self.leader_idx} silent {now-last:.0f}s"
                    )

            # Resolve race window — highest candidate after ELECTION_RACE_S wins
            if self._candidate is not None:
                if (now - self._candidate_time) >= ELECTION_RACE_S:
                    winner          = self._candidate
                    self._candidate = None
                    if winner == self.idx:
                        self.election_id += 1
                        self._set_leader(self.idx, self.election_id)
                        self._announce_leader()
                        print(
                            f"[D2D-{self.idx}] Won election #{self.election_id}"
                            f" — I am RADAR LEADER",
                            flush=True,
                        )

    # ── Entry point ───────────────────────────────────────────────────

    async def run(self) -> None:
        """Start D2D node. Call via asyncio.create_task(node.run())."""
        self._running   = True
        self._send_sock = self._make_send_sock(self.idx)
        recv_sock       = self._make_recv_sock()

        loop = asyncio.get_event_loop()
        self._transport, _ = await loop.create_datagram_endpoint(
            lambda: _D2DProtocol(self),
            sock=recv_sock,
        )
        print(
            f"[D2D-{self.idx}] Node up  "
            f"mcast={D2D_GROUP}:{D2D_PORT}  src={SRC_BASE+self.idx}",
            flush=True,
        )

        try:
            await asyncio.gather(
                self._hb_loop(),
                self._lead_loop(),
                self._election_watch(),
                return_exceptions=True,
            )
        finally:
            self.stop()

    def stop(self) -> None:
        self._running = False
        if self._transport:
            self._transport.close()
            self._transport = None
        if self._send_sock:
            self._send_sock.close()
            self._send_sock = None
