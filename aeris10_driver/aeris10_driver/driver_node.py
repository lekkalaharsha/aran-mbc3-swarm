"""
AERIS-10 ROS2 driver node.

Reads FMCW phased-array data from AERIS-10 via USB and publishes
PointCloud2 on six panel topics to feed radar_fusion/detection_node.

Topic mapping (matches detection_node subscription pattern):
  /[ns]/radar_A/scan/points  … /[ns]/radar_F/scan/points

Panel assignment — 6 × 60° azimuth sectors (CCW from drone +X):
  A:   0° ± 30°  →  [-30°,  +30°]
  B:  60° ± 30°  →  [+30°,  +90°]
  C: 120° ± 30°  →  [+90°, +150°]
  D: 180° ± 30°  →  [+150°,+180°] ∪ [-180°,-150°]
  E: 240° ± 30°  →  [-150°,  -90°]
  F: 300° ± 30°  →  [-90°,  -30°]

Points are expressed in each panel's LOCAL sensor frame so that
detection_node's PANEL_ROT rotation is correct.

sim_mode=true: generates synthetic rotating target for bench testing.
"""

import math
import struct
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2

from aeris10_driver.aeris10_usb import Aeris10USB, RadarFrame, AERIS10_VID, AERIS10_PID

# Panel centre azimuths (deg) and their base_link rotation matrices
# R_panel = R_z(centre_deg) — same as PANEL_ROT in detection_node.py
_S3 = math.sqrt(3) / 2

PANELS: dict[str, dict] = {
    'A': {'az_min': -30.0, 'az_max':  30.0,
          'R': np.array([[ 1.,   0.,  0.],
                         [ 0.,   1.,  0.],
                         [ 0.,   0.,  1.]], dtype=float)},
    'B': {'az_min':  30.0, 'az_max':  90.0,
          'R': np.array([[ 0.5, -_S3, 0.],
                         [ _S3,  0.5, 0.],
                         [ 0.,   0.,  1.]], dtype=float)},
    'C': {'az_min':  90.0, 'az_max': 150.0,
          'R': np.array([[-0.5, -_S3, 0.],
                         [ _S3, -0.5, 0.],
                         [ 0.,   0.,  1.]], dtype=float)},
    'D': {'az_min': 150.0, 'az_max': 210.0,  # wraps ±180°
          'R': np.array([[-1.,   0.,  0.],
                         [ 0.,  -1.,  0.],
                         [ 0.,   0.,  1.]], dtype=float)},
    'E': {'az_min': 210.0, 'az_max': 270.0,
          'R': np.array([[-0.5,  _S3, 0.],
                         [-_S3, -0.5, 0.],
                         [ 0.,   0.,  1.]], dtype=float)},
    'F': {'az_min': 270.0, 'az_max': 330.0,
          'R': np.array([[ 0.5,  _S3, 0.],
                         [-_S3,  0.5, 0.],
                         [ 0.,   0.,  1.]], dtype=float)},
}

# Precompute R^T (base_link → panel frame)
for _p in PANELS.values():
    _p['RT'] = _p['R'].T


def _az_to_panel(az_deg: float) -> str:
    """Map azimuth (−180…+180 or 0…360) to panel letter."""
    az = az_deg % 360.0   # normalise to [0, 360)
    for name, p in PANELS.items():
        if p['az_min'] <= az < p['az_max']:
            return name
    return 'A'   # ≥330° wraps to A


def _make_cloud(points: list[tuple], frame_id: str, stamp) -> PointCloud2:
    """Build PointCloud2 from list of (x, y, z) tuples."""
    header = Header()
    header.frame_id = frame_id
    header.stamp    = stamp
    return pc2.create_cloud_xyz32(header, points)


class Aeris10DriverNode(Node):

    def __init__(self):
        super().__init__('aeris10_driver')

        self.declare_parameter('drone_ns',       '')
        self.declare_parameter('vid',            AERIS10_VID)
        self.declare_parameter('pid',            AERIS10_PID)
        self.declare_parameter('min_range_m',    1.0)
        self.declare_parameter('max_range_m',    5000.0)
        self.declare_parameter('min_power_dBm', -80.0)
        self.declare_parameter('reconnect_s',    2.0)
        self.declare_parameter('publish_hz',     20.0)
        self.declare_parameter('sim_mode',       False)

        ns              = self.get_parameter('drone_ns').value
        vid             = self.get_parameter('vid').value
        pid             = self.get_parameter('pid').value
        self._min_rng   = self.get_parameter('min_range_m').value
        self._max_rng   = self.get_parameter('max_range_m').value
        self._min_pwr   = self.get_parameter('min_power_dBm').value
        self._reconn_s  = self.get_parameter('reconnect_s').value
        self._sim       = self.get_parameter('sim_mode').value
        hz              = self.get_parameter('publish_hz').value

        prefix = f'/{ns}' if ns else ''

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._pubs: dict[str, rclpy.publisher.Publisher] = {}
        for panel in 'ABCDEF':
            topic = f'{prefix}/radar_{panel}/scan/points'
            self._pubs[panel] = self.create_publisher(PointCloud2, topic, qos)
            self.get_logger().info(f'Publishing: {topic}')

        # Buffer: panel → list of (x,y,z) in panel-local frame
        self._buf: dict[str, list] = {p: [] for p in 'ABCDEF'}
        self._buf_lock = threading.Lock()

        self._usb = Aeris10USB(vid=vid, pid=pid)
        self._reader_thread = threading.Thread(
            target=self._reader_loop, daemon=True)
        self._reader_thread.start()

        self.create_timer(1.0 / hz, self._publish)

        self.get_logger().info(
            f'AERIS-10 driver started — '
            f'{"SIM MODE" if self._sim else f"USB VID={vid:#06x} PID={pid:#06x}"} '
            f'ns="{ns}" hz={hz}'
        )

    # ── USB reader thread ─────────────────────────────────────────────────────

    def _reader_loop(self):
        if self._sim:
            self._sim_loop()
            return

        while rclpy.ok():
            if not self._usb.connected:
                ok = self._usb.connect()
                if not ok:
                    self.get_logger().warn(
                        'AERIS-10 not found — retrying in '
                        f'{self._reconn_s:.0f}s',
                        throttle_duration_sec=10.0,
                    )
                    time.sleep(self._reconn_s)
                    continue
                self.get_logger().info('AERIS-10 connected')

            try:
                frame = self._usb.read_frame()
                self._ingest(frame)
            except IOError as e:
                self.get_logger().error(f'USB read error: {e} — reconnecting')
                self._usb.disconnect()
                time.sleep(self._reconn_s)

    def _ingest(self, frame: RadarFrame):
        """
        Classify each return into a panel and rotate from base_link into
        that panel's local sensor frame, then buffer.
        """
        per_panel: dict[str, list] = {p: [] for p in 'ABCDEF'}

        for ret in frame.returns:
            if ret.range_m < self._min_rng or ret.range_m > self._max_rng:
                continue
            if ret.power_dBm < self._min_pwr:
                continue

            az  = math.radians(ret.az_deg)
            el  = math.radians(ret.el_deg)
            rng = ret.range_m

            # Spherical → Cartesian in base_link
            cos_el = math.cos(el)
            bx = rng * cos_el * math.cos(az)
            by = rng * cos_el * math.sin(az)
            bz = rng * math.sin(el)

            panel = _az_to_panel(ret.az_deg)
            per_panel[panel].append((bx, by, bz))

        if not any(per_panel.values()):
            return

        # Rotate base_link → panel-local frame
        with self._buf_lock:
            for p, pts in per_panel.items():
                if not pts:
                    continue
                RT  = PANELS[p]['RT']
                arr = np.array(pts, dtype=float)        # (N, 3)
                loc = (RT @ arr.T).T                    # (N, 3) panel frame
                self._buf[p] = [
                    (float(loc[i, 0]), float(loc[i, 1]), float(loc[i, 2]))
                    for i in range(len(loc))
                ]

    # ── Simulation mode ───────────────────────────────────────────────────────

    def _sim_loop(self):
        """
        Synthetic rotating target at 200m range for bench testing.
        Generates 15 scatter points per cycle: satisfies min_cluster_hits=3 (detection_node)
        and the RF gate's 9-hit threshold for "real target" classification.
        """
        az_deg = 0.0
        rng_base = 200.0
        el_base  = math.radians(5.0)
        while rclpy.ok():
            az     = math.radians(az_deg)
            cos_el = math.cos(el_base)
            bx = rng_base * cos_el * math.cos(az)
            by = rng_base * cos_el * math.sin(az)
            bz = rng_base * math.sin(el_base)

            panel = _az_to_panel(az_deg)
            RT    = PANELS[panel]['RT']

            # 15 scatter points within 0.4m — all cluster together, pass RF gate (needs ≥9 hits)
            pts = []
            for _ in range(15):
                noise = np.random.normal(0.0, 0.2, 3)
                pt = RT @ (np.array([bx, by, bz]) + noise)
                pts.append((float(pt[0]), float(pt[1]), float(pt[2])))

            with self._buf_lock:
                for p in 'ABCDEF':
                    self._buf[p] = []
                self._buf[panel] = pts

            az_deg = (az_deg + 5.0) % 360.0
            time.sleep(0.1)

    # ── Publish timer ─────────────────────────────────────────────────────────

    def _publish(self):
        stamp = self.get_clock().now().to_msg()
        with self._buf_lock:
            snap = {p: list(v) for p, v in self._buf.items()}
            # Clear after publish so stale points don't accumulate
            for p in 'ABCDEF':
                self._buf[p] = []

        for panel, pts in snap.items():
            frame_id = f'aeris10_panel_{panel}'
            cloud = _make_cloud(pts, frame_id, stamp)
            self._pubs[panel].publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = Aeris10DriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
