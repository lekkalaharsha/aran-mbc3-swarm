#!/usr/bin/env python3
"""
asp_bridge.py — Forward radar detections → ASP GCS map.

Subscribes to ROS2 topics from the radar_fusion pipeline and POSTs
track data to http://localhost:5000/asp_update so the ASP browser
map shows moving target detections in real time.

Single-drone mode: subscribes to /radar/targets (detection_node output).
Swarm mode:        subscribes to /swarm/tracks  (fusion_node output).

Coordinate conversion:
  tf_ok=True  → pos[x,y,z] is world ENU (meters from Gazebo origin = HOME_LAT/LON)
  tf_ok=False → uses range_m + az_deg + drone GPS from /api/drone_state
                az_deg is CCW from body forward; combined with compass heading
                to produce a world bearing.

Usage:
    source /opt/ros/jazzy/setup.bash
    python3 src/asp_bridge.py
"""

import json
import math
import os
import sys
import time

import requests
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Pull HOME coords from mission_config so they stay in sync with mission
sys.path.insert(0, os.path.dirname(__file__))
from mission_config import HOME_LAT, HOME_LON

GCS_URL   = "http://localhost:5000/asp_update"
STATE_URL = "http://localhost:5000/api/drone_state"
EARTH_R   = 111111.0   # metres per degree latitude (approx)


def _world_to_latlon(wx: float, wy: float) -> tuple[float, float]:
    """World ENU (x=East, y=North, metres from Gazebo origin) → lat/lon."""
    lat = HOME_LAT + wy / EARTH_R
    lon = HOME_LON + wx / (EARTH_R * math.cos(math.radians(HOME_LAT)))
    return lat, lon


def _range_az_to_latlon(range_m: float, az_deg: float,
                         drone_lat: float, drone_lon: float,
                         drone_heading: float) -> tuple[float, float]:
    """
    Convert range + body-frame az to lat/lon.

    az_deg is CCW from drone forward (atan2(cy, cx) in base_link).
    drone_heading is compass bearing (CW from North, 0-360).
    World bearing = heading - az_deg (compass CW minus body CCW).
    """
    world_bearing = math.radians((drone_heading - az_deg) % 360)
    d_north = range_m * math.cos(world_bearing)
    d_east  = range_m * math.sin(world_bearing)
    lat = drone_lat + d_north / EARTH_R
    lon = drone_lon + d_east / (EARTH_R * math.cos(math.radians(drone_lat)))
    return lat, lon


class AspBridge(Node):

    def __init__(self):
        super().__init__('asp_bridge')

        # Cached drone state — refreshed every push cycle from telemetry_web
        self._drone_lat     = HOME_LAT
        self._drone_lon     = HOME_LON
        self._drone_alt     = 0.0
        self._drone_heading = 0.0

        # Latest tracks — written by ROS2 callbacks, read by push timer
        self._tracks: list[dict] = []
        self._scan_count = 0

        # Single-drone detection output
        self.create_subscription(String, '/radar/targets', self._on_single, 10)
        # Swarm fusion output (used when fusion_node is running)
        self.create_subscription(String, '/swarm/tracks', self._on_swarm, 10)

        self.create_timer(0.5, self._push)   # 2 Hz push to ASP
        self.get_logger().info(
            f'ASP bridge ready  HOME=({HOME_LAT},{HOME_LON})  '
            f'→ {GCS_URL}'
        )

    # ── ROS2 callbacks ────────────────────────────────────────────────────────

    def _on_single(self, msg: String) -> None:
        """Detection node single-drone: /radar/targets"""
        try:
            self._tracks = json.loads(msg.data).get('targets', [])
        except Exception:
            pass

    def _on_swarm(self, msg: String) -> None:
        """Fusion node: /swarm/tracks (swarm mode)"""
        try:
            self._tracks = json.loads(msg.data).get('tracks', [])
        except Exception:
            pass

    # ── Drone state polling ───────────────────────────────────────────────────

    def _refresh_drone_state(self) -> None:
        try:
            r = requests.get(STATE_URL, timeout=0.2)
            s = r.json()
            # Only update if GCS has a valid GPS fix (lat != 0)
            if s.get('lat'):
                self._drone_lat     = s['lat']
                self._drone_lon     = s['lon']
                self._drone_alt     = s.get('alt', self._drone_alt)
                self._drone_heading = s.get('heading', self._drone_heading)
        except Exception:
            pass   # use cached values

    # ── Coordinate conversion ─────────────────────────────────────────────────

    def _track_to_asp(self, t: dict) -> dict | None:
        range_m = t.get('range_m', 0.0)
        az_deg  = t.get('az_deg', 0.0)
        tf_ok   = t.get('tf_ok', False)
        pos     = t.get('pos', [0.0, 0.0, 0.0])

        if tf_ok and any(pos):
            lat, lon = _world_to_latlon(pos[0], pos[1])
            alt = self._drone_alt + pos[2]
        else:
            if range_m < 1.0:
                return None   # no valid measurement
            lat, lon = _range_az_to_latlon(
                range_m, az_deg,
                self._drone_lat, self._drone_lon, self._drone_heading,
            )
            alt = self._drone_alt   # altitude unknown without TF

        vel = t.get('vel', [0.0, 0.0, 0.0])
        speed = math.sqrt(vel[0]**2 + vel[1]**2)
        n_obs = t.get('n_obs', t.get('hits', 1))

        return {
            'id':        t.get('id', '?'),
            'lat':       round(lat, 7),
            'lon':       round(lon, 7),
            'alt':       round(alt, 1),
            'range_m':   round(range_m, 1),
            'az_deg':    round(az_deg, 1),
            'speed_ms':  round(speed, 1),
            'conf':      round(min(1.0, n_obs / 8.0), 2),
            'tf_ok':     tf_ok,
        }

    # ── Push cycle ────────────────────────────────────────────────────────────

    def _push(self) -> None:
        self._refresh_drone_state()
        self._scan_count += 1

        asp_tracks = [
            asp for t in self._tracks
            if (asp := self._track_to_asp(t)) is not None
        ]

        payload = {
            'asp_tracks':   asp_tracks,
            'scan_count':   self._scan_count,
            'asp_drone_id': 'RADAR',
        }

        try:
            requests.post(GCS_URL, json=payload, timeout=0.3)
        except Exception:
            pass

        if self._scan_count % 20 == 0:
            self.get_logger().info(
                f'Push #{self._scan_count}: {len(asp_tracks)} track(s)  '
                f'drone=({self._drone_lat:.4f},{self._drone_lon:.4f})'
                f'  hdg={self._drone_heading:.0f}°'
            )


def main() -> None:
    rclpy.init()
    node = AspBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
