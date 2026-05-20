"""
Radar Detection Node — MBC-3 Layer 1 + basic clustering.

Per drone: subscribes to 6 FMCW radar panel scans (PointCloud2),
filters NaN/inf no-hit rays, transforms panel-frame points to base_link,
clusters valid hits into targets, publishes detections as MarkerArray + JSON.

Pipeline:
  /[ns]/radar_A/scan  ┐
  /[ns]/radar_B/scan  │
  /[ns]/radar_C/scan  ├── filter → panel→base_link TF → cluster → world TF
  /[ns]/radar_D/scan  │       → /[ns]/radar/detections  (MarkerArray)
  /[ns]/radar_E/scan  │       → /[ns]/radar/targets     (String JSON)
  /[ns]/radar_F/scan  ┘
"""

import json
import math
import threading
import time

import numpy as np
import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs  # noqa: F401 — registers do_transform_point

import sensor_msgs_py.point_cloud2 as pc2

from radar_fusion.rf_classifier import RFTargetClassifier


# Panel rotation matrices: radar_frame_X → base_link
# 6 panels at 60° spacing (CCW from drone forward), matching mbc3_radar_drone xacro.
# R_z(θ) = [[cosθ,-sinθ,0],[sinθ,cosθ,0],[0,0,1]]
#   A:   0°  identity
#   B:  60°  R_z(60°)
#   C: 120°  R_z(120°)
#   D: 180°  R_z(180°)
#   E: 240°  R_z(240°)
#   F: 300°  R_z(300°)
_S3 = math.sqrt(3) / 2   # sin(60°) = cos(30°) ≈ 0.8660

PANEL_ROT: dict[str, np.ndarray] = {
    'A': np.array([[ 1.,   0.,  0.],
                   [ 0.,   1.,  0.],
                   [ 0.,   0.,  1.]], dtype=float),
    'B': np.array([[ 0.5, -_S3, 0.],
                   [ _S3,  0.5, 0.],
                   [ 0.,   0.,  1.]], dtype=float),
    'C': np.array([[-0.5, -_S3, 0.],
                   [ _S3, -0.5, 0.],
                   [ 0.,   0.,  1.]], dtype=float),
    'D': np.array([[-1.,   0.,  0.],
                   [ 0.,  -1.,  0.],
                   [ 0.,   0.,  1.]], dtype=float),
    'E': np.array([[-0.5,  _S3, 0.],
                   [-_S3, -0.5, 0.],
                   [ 0.,   0.,  1.]], dtype=float),
    'F': np.array([[ 0.5,  _S3, 0.],
                   [-_S3,  0.5, 0.],
                   [ 0.,   0.,  1.]], dtype=float),
}

VEL_GATE_M = 20.0   # max displacement per cycle to count as same target

PANEL_COLORS = {
    'A': (1.0, 0.2, 0.2, 0.9),   # red     — 0°
    'B': (1.0, 0.6, 0.1, 0.9),   # orange  — 60°
    'C': (0.6, 1.0, 0.2, 0.9),   # lime    — 120°
    'D': (0.2, 1.0, 0.4, 0.9),   # green   — 180°
    'E': (0.0, 0.9, 0.9, 0.9),   # cyan    — 240°
    'F': (0.8, 0.2, 1.0, 0.9),   # magenta — 300°
}


class DetectionNode(Node):

    def __init__(self):
        super().__init__('radar_detection_node')

        # Parameters
        self.declare_parameter('drone_ns',          '')
        self.declare_parameter('cluster_radius',    2.5)
        self.declare_parameter('min_cluster_hits',  3)
        self.declare_parameter('max_range',         5000.0)
        self.declare_parameter('min_range',         20.0)   # B2: raised from 2m — avoids self-detection
        self.declare_parameter('el_gate_deg',       25.0)   # B3: max elevation — filters zenith
        self.declare_parameter('el_min_deg',       -5.0)   # B3: min elevation — filters ground
        self.declare_parameter('marker_scale',      2.0)
        self.declare_parameter('publish_hz',        5.0)
        self.declare_parameter('use_rf_gate',       True)
        self.declare_parameter('rf_model_path',     '')

        ns               = self.get_parameter('drone_ns').value
        self.cluster_r   = self.get_parameter('cluster_radius').value
        self.min_hits    = self.get_parameter('min_cluster_hits').value
        self.max_range   = self.get_parameter('max_range').value
        self.min_range   = self.get_parameter('min_range').value
        self._el_gate    = self.get_parameter('el_gate_deg').value
        self._el_min     = self.get_parameter('el_min_deg').value
        self.marker_scale = self.get_parameter('marker_scale').value

        use_rf   = self.get_parameter('use_rf_gate').value
        rf_path  = self.get_parameter('rf_model_path').value or None
        self._rf = RFTargetClassifier(model_path=rf_path) if use_rf else None
        if use_rf:
            self.get_logger().info('Layer 2 RF gate: ACTIVE')

        prefix = f'/{ns}' if ns else ''
        self._source_frame = f'{ns}/base_link' if ns else 'base_link'

        # TF2
        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # B1: panel rotations are hardcoded from xacro (see PANEL_ROT above)

        # Velocity estimation: previous world-frame centroids
        self._prev_world: list[dict] = []

        # Storage: latest valid points per panel (already in base_link frame)
        self._points: dict[str, list] = {p: [] for p in 'ABCDEF'}
        self._points_lock = threading.Lock()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        for panel in 'ABCDEF':
            topic = f'{prefix}/radar_{panel}/scan/points'
            self.create_subscription(
                PointCloud2, topic,
                lambda msg, p=panel: self._on_scan(msg, p),
                qos,
            )
            self.get_logger().info(f'Subscribed: {topic}')

        self.det_pub   = self.create_publisher(
            MarkerArray, f'{prefix}/radar/detections', 10)
        self.track_pub = self.create_publisher(
            String, f'{prefix}/radar/targets', 10)

        hz = self.get_parameter('publish_hz').value
        self.create_timer(1.0 / hz, self._publish)

        self.get_logger().info(
            f'Detection node ready — ns="{ns}" '
            f'cluster_r={self.cluster_r}m min_hits={self.min_hits} '
            f'min_range={self.min_range}m el=[{self._el_min}°,{self._el_gate}°]'
        )

    # ── Scan callback ─────────────────────────────────────────────────────────

    def _on_scan(self, msg: PointCloud2, panel: str):
        """
        Decode PointCloud2 → range/elevation filter → transform to base_link.
        B2: min_range=20m avoids self-detection.
        B3: el_gate filters ground and zenith returns.
        B1: rotate from panel sensor frame into base_link before storing.
        """
        raw = []
        try:
            for p in pc2.read_points(msg, field_names=('x', 'y', 'z'),
                                     skip_nans=True):
                x, y, z = float(p[0]), float(p[1]), float(p[2])
                if math.isinf(x) or math.isinf(y) or math.isinf(z):
                    continue
                r = math.sqrt(x*x + y*y + z*z)
                if r < self.min_range or r > self.max_range:
                    continue
                # B3: elevation gate in sensor frame
                el = math.degrees(math.asin(z / r)) if r > 0 else 0.0
                if el > self._el_gate or el < self._el_min:
                    continue
                raw.append((x, y, z))
        except Exception as e:
            self.get_logger().debug(f'Scan decode error panel {panel}: {e}')

        if not raw:
            with self._points_lock:
                self._points[panel] = []
            return

        # B1: rotate from sensor frame → base_link using hardcoded panel rotation
        R   = PANEL_ROT[panel]
        pts = np.array(raw, dtype=float)       # (N, 3) in sensor frame
        pts_base = (R @ pts.T).T               # (N, 3) in base_link frame
        rotated = [
            (float(pts_base[i, 0]),
             float(pts_base[i, 1]),
             float(pts_base[i, 2]),
             float(np.linalg.norm(pts_base[i])))
            for i in range(len(pts_base))
        ]
        with self._points_lock:
            self._points[panel] = rotated

    # ── Clustering ────────────────────────────────────────────────────────────

    def _cluster(self, points: dict) -> list[dict]:
        """Greedy clustering in base_link frame across all panels."""
        all_pts = []
        for panel, pts in points.items():
            for x, y, z, r in pts:
                all_pts.append((x, y, z, r, panel))

        if not all_pts:
            return []

        used = [False] * len(all_pts)
        clusters = []

        for i, (xi, yi, zi, ri, pi) in enumerate(all_pts):
            if used[i]:
                continue
            members = [(xi, yi, zi, ri, pi)]
            used[i] = True
            for j in range(i + 1, len(all_pts)):
                if used[j]:
                    continue
                xj, yj, zj, rj, pj = all_pts[j]
                if math.sqrt((xi-xj)**2 + (yi-yj)**2 + (zi-zj)**2) <= self.cluster_r:
                    members.append((xj, yj, zj, rj, pj))
                    used[j] = True

            if len(members) < self.min_hits:
                continue

            cx = sum(m[0] for m in members) / len(members)
            cy = sum(m[1] for m in members) / len(members)
            cz = sum(m[2] for m in members) / len(members)
            rng = math.sqrt(cx*cx + cy*cy + cz*cz)
            az  = math.degrees(math.atan2(cy, cx))
            el  = math.degrees(math.asin(cz / rng)) if rng > 0 else 0.0

            panel_votes: dict[str, int] = {}
            for m in members:
                panel_votes[m[4]] = panel_votes.get(m[4], 0) + 1
            dom_panel = max(panel_votes, key=panel_votes.get)

            ranges    = [m[3] for m in members]
            lats      = [math.sqrt(m[0]**2 + m[1]**2) for m in members]
            range_std = float(np.std(ranges)) if len(ranges) > 1 else 0.0
            spread_xy = float(np.std(lats))   if len(lats)   > 1 else 0.0

            clusters.append({
                'cx': cx, 'cy': cy, 'cz': cz,
                'range_m':   round(rng, 2),
                'az_deg':    round(az, 1),
                'el_deg':    round(el, 1),
                'hits':      len(members),
                'panel':     dom_panel,
                'range_std': round(range_std, 3),
                'spread_xy': round(spread_xy, 3),
            })

        return clusters

    # ── TF2: base_link → world ────────────────────────────────────────────────

    def _to_world(self, x: float, y: float, z: float, stamp) -> tuple:
        """Transform point from base_link → world. Returns (x, y, z, ok)."""
        ps = PointStamped()
        ps.header.stamp    = stamp
        ps.header.frame_id = self._source_frame
        ps.point.x = x
        ps.point.y = y
        ps.point.z = z
        try:
            pw = self._tf_buffer.transform(
                ps, 'world',
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
            return pw.point.x, pw.point.y, pw.point.z, True
        except TransformException as e:
            self.get_logger().warn(
                f'TF {self._source_frame}→world: {e}',
                throttle_duration_sec=5.0,
            )
            return x, y, z, False

    # ── Velocity estimation ───────────────────────────────────────────────────

    def _match_velocity(self, wx: float, wy: float, wz: float, now: float) -> list:
        """Nearest-neighbour match to previous cycle → [vx, vy, vz] m/s."""
        best_dist = VEL_GATE_M
        best_prev = None
        for p in self._prev_world:
            px, py, pz = p['pos']
            d = math.sqrt((wx-px)**2 + (wy-py)**2 + (wz-pz)**2)
            if d < best_dist:
                best_dist = d
                best_prev = p
        if best_prev is None:
            return [0.0, 0.0, 0.0]
        dt = now - best_prev['t']
        if dt <= 0:
            return [0.0, 0.0, 0.0]
        px, py, pz = best_prev['pos']
        return [
            round((wx - px) / dt, 2),
            round((wy - py) / dt, 2),
            round((wz - pz) / dt, 2),
        ]

    # ── Publish ───────────────────────────────────────────────────────────────

    def _publish(self):
        with self._points_lock:
            points_snap = {k: list(v) for k, v in self._points.items()}
        clusters = self._cluster(points_snap)

        # Layer 2 — RF gate: filter clutter, pass confirmed targets
        if self._rf is not None and clusters:
            labels   = self._rf.predict(clusters)
            n_before = len(clusters)
            clusters = [c for c, lbl in zip(clusters, labels) if lbl == 1]
            n_after  = len(clusters)
            if n_before > n_after:
                self.get_logger().info(
                    f'RF gate: {n_after}/{n_before} clusters confirmed as targets'
                )

        stamp    = self.get_clock().now().to_msg()
        now      = self.get_clock().now().nanoseconds * 1e-9

        ma = MarkerArray()
        del_m = Marker()
        del_m.action = Marker.DELETEALL
        del_m.ns = 'radar_detections'
        ma.markers.append(del_m)

        tracks: list[dict] = []
        new_prev: list[dict] = []

        for idx, c in enumerate(clusters):
            wx, wy, wz, tf_ok = self._to_world(c['cx'], c['cy'], c['cz'], stamp)
            vel = self._match_velocity(wx, wy, wz, now)
            new_prev.append({'pos': [wx, wy, wz], 't': now})

            w_range = math.sqrt(wx*wx + wy*wy + wz*wz)
            w_az    = math.degrees(math.atan2(wy, wx))
            w_el    = math.degrees(math.asin(wz / w_range)) if w_range > 0 else 0.0

            color = PANEL_COLORS.get(c['panel'], (1.0, 1.0, 1.0, 0.8))

            sphere = Marker()
            sphere.header.stamp    = stamp
            sphere.header.frame_id = 'world'
            sphere.ns = 'radar_detections'; sphere.id = idx * 2
            sphere.type = Marker.SPHERE; sphere.action = Marker.ADD
            sphere.pose.position.x = wx
            sphere.pose.position.y = wy
            sphere.pose.position.z = wz
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = self.marker_scale
            sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = color
            sphere.lifetime.sec = 1
            ma.markers.append(sphere)

            text = Marker()
            text.header.stamp    = stamp
            text.header.frame_id = 'world'
            text.ns = 'radar_detections'; text.id = idx * 2 + 1
            text.type = Marker.TEXT_VIEW_FACING; text.action = Marker.ADD
            text.pose.position.x = wx
            text.pose.position.y = wy
            text.pose.position.z = wz + self.marker_scale + 0.5
            text.pose.orientation.w = 1.0
            text.scale.z = 1.5
            text.color.r = text.color.g = text.color.b = 1.0
            text.color.a = 1.0
            text.text = (
                f"TGT_{idx+1:02d}\n"
                f"R={w_range:.0f}m  Az={w_az:.0f}°\n"
                f"Panel {c['panel']}  Hits={c['hits']}"
                + ('' if tf_ok else '  [no TF]')
            )
            text.lifetime.sec = 1
            ma.markers.append(text)

            tracks.append({
                'id':       f'TGT_{idx+1:02d}',
                'range_m':  round(w_range, 2),
                'az_deg':   round(w_az, 1),
                'el_deg':   round(w_el, 1),
                'hits':     c['hits'],
                'panel':    c['panel'],
                'pos':      [round(wx, 2), round(wy, 2), round(wz, 2)],
                'vel':      vel,
                'tf_ok':    tf_ok,
                'timestamp': now,
            })

        self._prev_world = new_prev
        self.det_pub.publish(ma)

        track_msg = String()
        track_msg.data = json.dumps({'n_targets': len(tracks), 'targets': tracks})
        self.track_pub.publish(track_msg)

        if tracks:
            self.get_logger().info(
                f'Detected {len(tracks)} target(s): '
                + ', '.join(
                    f"{t['id']} R={t['range_m']:.0f}m Az={t['az_deg']:.0f}°"
                    + ('' if t['tf_ok'] else '[no TF]')
                    for t in tracks
                )
            )


def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
