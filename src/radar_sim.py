#!/usr/bin/env python3
"""
radar_sim.py — Pose-based FMCW radar simulator (no rendering required).

Reads drone + target positions from Gazebo dynamic_pose/info via gz CLI.
Computes 6-panel radar detections from geometry. Pushes to ASP GCS.

Use when lidar sensors don't publish (headless, no DISPLAY):
    python3 src/radar_sim.py

Real lidar sensors work when launched from WSL terminal with GUI (WSLg):
    MBC3_MODE=1 ./launch.sh
"""

import json
import math
import os
import re
import subprocess
import sys
import time

import requests

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mission_config import HOME_LAT, HOME_LON

GCS_URL      = "http://localhost:5000/asp_update"
STATE_URL    = "http://localhost:5000/api/drone_state"
LEADER_URL   = "http://localhost:5000/api/leader"
WORLD        = os.environ.get("PX4_GZ_WORLD", "mbc3_radar_moving")
EARTH_R      = 111111.0   # m per degree lat

# Active radar leader — updated by _refresh_leader() from leader_election.py
_leader_model = "mbc3_radar_drone_0"   # default: instance 0

# Radar parameters matching mbc3_radar_drone SDF
PANEL_HALF_FOV = math.radians(30.0)   # ±30° per panel
PANEL_CENTERS  = [math.radians(i * 60) for i in range(6)]   # 0°,60°,...,300°
RADAR_MIN_M    = 20.0
RADAR_MAX_M    = 5000.0
EL_MAX_DEG     =  25.0
EL_MIN_DEG     =  -5.0
NOISE_SIGMA_M  =   0.10


# ── Pose parsing ──────────────────────────────────────────────────────────────

def get_poses() -> dict:
    """Call gz topic -n 1 and parse gz.msgs.Pose_V → {name: (x,y,z,qx,qy,qz,qw)}."""
    try:
        proc = subprocess.run(
            ['gz', 'topic', '-e', '-n', '1',
             '-t', f'/world/{WORLD}/dynamic_pose/info'],
            capture_output=True, text=True, timeout=2.0,
        )
        return _parse_pose_v(proc.stdout)
    except Exception:
        return {}


def _parse_pose_v(text: str) -> dict:
    """Parse gz.msgs.Pose_V protobuf text representation."""
    poses: dict = {}
    cur: dict   = {}
    depth       = 0
    in_pos      = False
    in_ori      = False

    for raw in text.splitlines():
        line = raw.strip()
        if line == 'pose {':
            cur = {}; depth = 1; in_pos = False; in_ori = False
        elif depth == 1 and line == '}':
            if 'name' in cur:
                poses[cur['name']] = (
                    cur.get('x', 0.0), cur.get('y', 0.0), cur.get('z', 0.0),
                    cur.get('qx', 0.0), cur.get('qy', 0.0),
                    cur.get('qz', 0.0), cur.get('qw', 1.0),
                )
            depth = 0; in_pos = False; in_ori = False
        elif depth == 1:
            if line.startswith('name:'):
                cur['name'] = line.split(':', 1)[1].strip().strip('"')
            elif line == 'position {'  : in_pos = True;  in_ori = False
            elif line == 'orientation {': in_ori = True; in_pos = False
            elif line == '}'            : in_pos = False; in_ori = False
            elif in_pos:
                m = re.match(r'([xyz]):\s*([-\d.eE+]+)', line)
                if m:
                    cur[m.group(1)] = float(m.group(2))
            elif in_ori:
                m = re.match(r'([wxyz]):\s*([-\d.eE+]+)', line)
                if m:
                    cur['q' + m.group(1)] = float(m.group(2))
    return poses


# ── Geometry ──────────────────────────────────────────────────────────────────

def _quat_inv_rotate(qx, qy, qz, qw, vx, vy, vz):
    """Rotate vector by conjugate (inverse) quaternion — world → body frame."""
    cx, cy, cz, cw = -qx, -qy, -qz, qw
    tx = 2 * (cy * vz - cz * vy)
    ty = 2 * (cz * vx - cx * vz)
    tz = 2 * (cx * vy - cy * vx)
    return (
        vx + cw * tx + cy * tz - cz * ty,
        vy + cw * ty + cz * tx - cx * tz,
        vz + cw * tz + cx * ty - cy * tx,
    )


def _in_fov(az_body_rad: float) -> bool:
    """True if az_body falls within any of the 6 panel sectors."""
    for c in PANEL_CENTERS:
        diff = abs(((az_body_rad - c + math.pi) % (2 * math.pi)) - math.pi)
        if diff <= PANEL_HALF_FOV:
            return True
    return False


# ── Detection engine ──────────────────────────────────────────────────────────

def _refresh_leader() -> None:
    """Update _leader_model from leader_election.py via GCS API."""
    global _leader_model
    try:
        r = requests.get(LEADER_URL, timeout=0.3)
        model = r.json().get("leader_model", _leader_model)
        if model != _leader_model:
            print(f"[SIM] Leader switched: {_leader_model} → {model}", flush=True)
            _leader_model = model
    except Exception:
        pass   # keep using cached value


def compute_detections(poses: dict) -> list:
    if _leader_model not in poses:
        return []

    dx, dy, dz, qx, qy, qz, qw = poses[_leader_model]
    detections = []

    for name, (tx, ty, tz, *_) in poses.items():
        if not name.startswith('radar_target_'):
            continue

        wx, wy, wz = tx - dx, ty - dy, tz - dz
        rng = math.sqrt(wx ** 2 + wy ** 2 + wz ** 2)

        if rng < RADAR_MIN_M or rng > RADAR_MAX_M:
            continue

        bx, by, bz = _quat_inv_rotate(qx, qy, qz, qw, wx, wy, wz)
        az  = math.atan2(by, bx)                            # CCW from forward
        el  = math.degrees(math.asin(bz / rng)) if rng > 0 else 0.0

        if el > EL_MAX_DEG or el < EL_MIN_DEG:
            continue
        if not _in_fov(az):
            continue

        # Tiny deterministic noise per target
        _m = re.search(r'\d+$', name)
        seed = (int(_m.group()) if _m else 0) * 7 + int(time.time() * 10) % 100
        rng += (seed % 21 - 10) * NOISE_SIGMA_M * 0.1

        detections.append({
            'id':        name.replace('radar_target_', 'TGT_'),
            'range_m':   round(rng, 1),
            'az_deg':    round(math.degrees(az), 1),
            'el_deg':    round(el, 1),
            'pos':       [round(tx, 2), round(ty, 2), round(tz, 2)],
            'vel':       [0.0, 0.0, 0.0],
            'tf_ok':     True,
            'hits':      8,
            'n_obs':     8,
            'simulated': True,
        })

    return detections


# ── ASP push ──────────────────────────────────────────────────────────────────

def _world_to_latlon(wx: float, wy: float) -> tuple:
    lat = HOME_LAT + wy / EARTH_R
    lon = HOME_LON + wx / (EARTH_R * math.cos(math.radians(HOME_LAT)))
    return round(lat, 7), round(lon, 7)


def push_asp(detections: list, scan_count: int) -> None:
    asp_tracks = []
    for d in detections:
        lat, lon = _world_to_latlon(d['pos'][0], d['pos'][1])
        asp_tracks.append({
            'id':          d['id'],
            'lat':         lat,
            'lon':         lon,
            'range_m':     d['range_m'],
            'bearing_deg': d['az_deg'],
            'alt_m':       round(d['pos'][2], 1),
            'velocity_ms': 0.0,
            'confidence':  0.85,
        })
    try:
        requests.post(GCS_URL, json={
            'asp_tracks':   asp_tracks,
            'scan_count':   scan_count,
            'asp_drone_id': 'SIM_RADAR',
        }, timeout=0.3)
    except Exception:
        pass


# ── Main loop ─────────────────────────────────────────────────────────────────

def main() -> None:
    print(f"[SIM] Radar simulator  world={WORLD}  HOME=({HOME_LAT},{HOME_LON})", flush=True)
    print(f"[SIM] Polling Gazebo poses → 6-panel FOV check → ASP at {GCS_URL}", flush=True)

    scan = 0
    while True:
        t0 = time.time()

        _refresh_leader()
        poses      = get_poses()
        detections = compute_detections(poses) if poses else []
        push_asp(detections, scan)
        scan += 1

        if scan % 10 == 0:
            n_tgts = sum(1 for k in poses if k.startswith('radar_target_'))
            if _leader_model in poses:
                dx, dy, dz = poses[_leader_model][:3]
                print(f"[SIM] #{scan}: {len(detections)}/{n_tgts} detected  "
                      f"leader={_leader_model}  pos=({dx:.0f},{dy:.0f},{dz:.0f})m",
                      flush=True)
            else:
                print(f"[SIM] #{scan}: leader {_leader_model} not in Gazebo — "
                      f"election pending?", flush=True)

        # 5 Hz — subtract processing time
        elapsed = time.time() - t0
        sleep   = max(0.0, 0.2 - elapsed)
        time.sleep(sleep)


if __name__ == '__main__':
    main()
