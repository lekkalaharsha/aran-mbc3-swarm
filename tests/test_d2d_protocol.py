#!/usr/bin/env python3
"""
tests/test_d2d_protocol.py — Standalone D2D protocol test (no SITL, no Gazebo).

Spawns 5 virtual drones on localhost using UDP multicast loopback.
Tests: HB propagation, bully election, leader failover, REASSIGN+ACK, RADAR fusion.

Run:
    python3 tests/test_d2d_protocol.py            # full suite (~40 s)
    python3 tests/test_d2d_protocol.py --quick    # HB + election only (~20 s)
    python3 tests/test_d2d_protocol.py --test election_failover
"""

import argparse
import asyncio
import sys
import os
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
from d2d_node import D2DNode, DEATH_TIMEOUT, ELECTION_RACE_S

# ── Shared mock state dicts ────────────────────────────────────────────────────

def _mock_state(idx: int) -> dict:
    return {
        "lat":         17.450 + idx * 0.001,
        "lon":         78.380 + idx * 0.001,
        "alt":         100.0  + idx * 10.0,
        "groundspeed": 12.0,
        "heading":     45.0,
        "armed":       True,
        "phase":       "SURVEY",
    }

STATES = {i: _mock_state(i) for i in range(5)}

# ── Helpers ────────────────────────────────────────────────────────────────────

PASS = "\033[92m PASS\033[0m"
FAIL = "\033[91m FAIL\033[0m"

def result(name: str, ok: bool, detail: str = "") -> bool:
    tag = PASS if ok else FAIL
    print(f"  [{tag} ] {name}" + (f"  — {detail}" if detail else ""), flush=True)
    return ok

async def _boot_nodes(indices: list[int], gcs_url: str = "http://localhost:9999/api/leader") -> list[D2DNode]:
    """Create and start D2DNode instances for given drone indices."""
    nodes = [D2DNode(i, STATES[i], gcs_url=gcs_url) for i in indices]
    tasks = [asyncio.create_task(n.run()) for n in nodes]
    await asyncio.sleep(0.3)           # allow sockets to bind
    return nodes, tasks

def _stop(nodes, tasks):
    for n in nodes:
        n.stop()
    for t in tasks:
        t.cancel()

# ── Tests ──────────────────────────────────────────────────────────────────────

async def test_heartbeat():
    """All 5 nodes exchange HBs and update peer_last_hb within 2 s."""
    print("\n[1] Heartbeat propagation", flush=True)
    nodes, tasks = await _boot_nodes(list(range(5)))
    await asyncio.sleep(2.5)     # 2 Hz → at least 5 cycles

    all_ok = True
    for n in nodes:
        peers_seen = len(n.peer_last_hb)
        ok = peers_seen == 4
        all_ok &= result(
            f"DRONE-{n.idx} sees peers",
            ok,
            f"{peers_seen}/4 peers in peer_last_hb",
        )

    _stop(nodes, tasks)
    return all_ok


async def test_election_basic():
    """Highest-index drone wins initial election within race window."""
    print("\n[2] Basic bully election", flush=True)
    nodes, tasks = await _boot_nodes(list(range(5)))
    await asyncio.sleep(1.5)

    # Manually trigger election from DRONE-0
    nodes[0]._start_election("test trigger")
    await asyncio.sleep(ELECTION_RACE_S + 0.5)

    winners = [n.leader_idx for n in nodes]
    # DRONE-4 (highest idx) should win
    all_ok = all(w == 4 for w in winners if w is not None)
    result(
        "DRONE-4 wins election",
        all_ok,
        f"seen leaders: {set(winners)}",
    )

    _stop(nodes, tasks)
    return all_ok


async def test_election_failover():
    """Kill leader (DRONE-4), next highest (DRONE-3) takes over within timeout."""
    print("\n[3] Leader failover", flush=True)
    nodes, tasks = await _boot_nodes(list(range(5)))
    await asyncio.sleep(1.5)

    # Make DRONE-4 the leader
    nodes[0]._start_election("prime leader")
    await asyncio.sleep(ELECTION_RACE_S + 0.5)

    prev_leader = nodes[0].leader_idx
    ok1 = result("Initial leader is DRONE-4", prev_leader == 4, f"got {prev_leader}")

    # Kill DRONE-4
    print("  [*] Stopping DRONE-4 (simulating crash)…", flush=True)
    nodes[4].stop()
    tasks[4].cancel()
    # Force stale timestamp so _election_watch triggers immediately
    stale = time.time() - DEATH_TIMEOUT - 2.0
    for n in nodes[:4]:
        n.peer_last_hb[4] = stale   # last > 0 AND now-last > DEATH_TIMEOUT

    # Wait for failover: DEATH_TIMEOUT check runs every 1 s, race = ELECTION_RACE_S
    wait = DEATH_TIMEOUT + ELECTION_RACE_S + 3.0
    print(f"  [*] Waiting up to {wait:.0f}s for failover…", flush=True)
    deadline = time.time() + wait
    while time.time() < deadline:
        await asyncio.sleep(1.0)
        new_leaders = [n.leader_idx for n in nodes[:4]]
        # wait until all remaining nodes have elected a non-crashed leader
        if all(l is not None and l != 4 for l in new_leaders):
            break

    new_leaders = [n.leader_idx for n in nodes[:4]]
    ok2 = result(
        "All agree on new leader (DRONE-3)",
        all(l == 3 for l in new_leaders if l is not None),
        f"leaders: {new_leaders}",
    )

    _stop(nodes[:4], tasks[:4])
    return ok1 and ok2


async def test_reassign_ack():
    """Leader sends REASSIGN to DRONE-1; DRONE-1 ACKs within 1 s."""
    print("\n[4] REASSIGN + ACK", flush=True)
    nodes, tasks = await _boot_nodes(list(range(5)))
    await asyncio.sleep(1.0)

    # Force DRONE-4 as leader in all nodes
    for n in nodes:
        n._set_leader(4, 1)

    mock_wps = [{"lat": 17.451, "lon": 78.381, "alt": 110.0}]
    mid = nodes[4].send_reassign(target_idx=1, waypoints=mock_wps)
    await asyncio.sleep(1.5)

    ack_info = nodes[4].pending_acks.get(mid, {})
    acked = ack_info.get("acked", False)
    ok = result("DRONE-1 ACKed REASSIGN", acked, f"msg#{mid} acked={acked}")

    _stop(nodes, tasks)
    return ok


async def test_radar_fusion():
    """Leader shares RADAR tracks; all nodes receive and fuse them."""
    print("\n[5] RADAR track sharing + fusion", flush=True)
    nodes, tasks = await _boot_nodes(list(range(5)))
    await asyncio.sleep(0.5)

    for n in nodes:
        n._set_leader(4, 1)

    mock_tracks = [
        {"id": "TRK-001", "lat": 17.452, "lon": 78.382, "alt": 50.0, "speed": 5.0},
        {"id": "TRK-002", "lat": 17.453, "lon": 78.383, "alt": 60.0, "speed": 0.0},
    ]
    nodes[4].broadcast_radar(mock_tracks, scan=42)
    await asyncio.sleep(0.8)

    all_ok = True
    for n in nodes[:4]:
        received = n.peer_radar_tracks.get(4, [])
        ok = len(received) == 2
        all_ok &= result(
            f"DRONE-{n.idx} received tracks",
            ok,
            f"got {len(received)} tracks",
        )

    # Test fusion
    own_new = [{"id": "TRK-003", "lat": 17.454, "lon": 78.384, "alt": 70.0}]
    fused = nodes[0].get_fused_tracks(own_new)
    ok_fuse = result(
        "DRONE-0 fusion gives 3 unique tracks",
        len(fused) == 3,
        f"fused count={len(fused)}",
    )
    all_ok &= ok_fuse

    _stop(nodes, tasks)
    return all_ok


async def test_no_self_loop():
    """Nodes must not process their own multicast packets."""
    print("\n[6] No self-loop", flush=True)
    nodes, tasks = await _boot_nodes([0])
    await asyncio.sleep(2.0)

    ok = len(nodes[0].peer_last_hb) == 0
    result("DRONE-0 ignores own HBs", ok, f"peer_last_hb={nodes[0].peer_last_hb}")

    _stop(nodes, tasks)
    return ok


# ── Runner ─────────────────────────────────────────────────────────────────────

SUITE = {
    "heartbeat":          test_heartbeat,
    "election_basic":     test_election_basic,
    "election_failover":  test_election_failover,
    "reassign_ack":       test_reassign_ack,
    "radar_fusion":       test_radar_fusion,
    "no_self_loop":       test_no_self_loop,
}

QUICK = ["heartbeat", "election_basic", "no_self_loop"]


async def main(args):
    if args.test:
        run = [args.test]
    elif args.quick:
        run = QUICK
    else:
        run = list(SUITE.keys())

    print(f"\n{'='*55}", flush=True)
    print(f"  MBC-3 D2D Protocol Test  —  {len(run)} test(s)", flush=True)
    print(f"{'='*55}", flush=True)
    print("  No SITL or Gazebo required — UDP multicast loopback", flush=True)

    results = {}
    for name in run:
        fn = SUITE.get(name)
        if fn is None:
            print(f"\nUnknown test: {name}. Available: {list(SUITE)}", flush=True)
            sys.exit(1)
        results[name] = await fn()

    passed = sum(results.values())
    total  = len(results)
    print(f"\n{'='*55}", flush=True)
    print(f"  Result: {passed}/{total} passed", flush=True)
    if passed < total:
        failed = [k for k, v in results.items() if not v]
        print(f"  Failed: {failed}", flush=True)
    print(f"{'='*55}\n", flush=True)
    sys.exit(0 if passed == total else 1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="MBC-3 D2D protocol test suite")
    parser.add_argument("--quick",   action="store_true", help="HB + election only")
    parser.add_argument("--test",    metavar="NAME",      help="run single test by name")
    args = parser.parse_args()
    asyncio.run(main(args))
