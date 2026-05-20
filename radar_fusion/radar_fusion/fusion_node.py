"""
Radar Fusion Node — MBC-3 Leader aggregation.

Subscribes to /drone_X/radar/targets (JSON) from all 5 drones.
Runs Kalman track manager: spatial merge → predict → NN assoc → update → TTL prune.
Publishes:
  /swarm/asp        MarkerArray — fused Air Situation Picture (world frame)
  /swarm/tracks     String JSON — consolidated track list with velocity
  /swarm/situation  String JSON — tactical picture for leader LLM
"""

import json
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from radar_fusion.kalman_tracker import TrackManager


_DEFAULT_DRONES = ['drone_L', 'drone_S1', 'drone_S2', 'drone_S3', 'drone_S4']


class FusionNode(Node):

    def __init__(self):
        super().__init__('radar_fusion_node')

        # Parameters
        self.declare_parameter('merge_dist_m',     5.0)
        self.declare_parameter('gate_m',           10.0)
        self.declare_parameter('track_ttl_s',      3.0)
        self.declare_parameter('publish_hz',       2.0)
        self.declare_parameter('q_pos',            1.0)
        self.declare_parameter('q_vel',            0.5)
        self.declare_parameter('r_pos',            5.0)
        # B6: configurable INTERCEPT threshold
        self.declare_parameter('intercept_range_m', 500.0)
        # B7: configurable drone ID list (comma-separated)
        self.declare_parameter('drone_ids', ','.join(_DEFAULT_DRONES))

        self._tracker = TrackManager(
            merge_dist_m = self.get_parameter('merge_dist_m').value,
            gate_m       = self.get_parameter('gate_m').value,
            ttl_s        = self.get_parameter('track_ttl_s').value,
            q_pos        = self.get_parameter('q_pos').value,
            q_vel        = self.get_parameter('q_vel').value,
            r_pos        = self.get_parameter('r_pos').value,
        )

        self._intercept_range = self.get_parameter('intercept_range_m').value

        # B7: drone IDs from parameter
        drone_ids_str = self.get_parameter('drone_ids').value
        self._drone_ids: list[str] = [d.strip() for d in drone_ids_str.split(',') if d.strip()]

        # Raw targets per drone — {drone_id: [target_dict, ...]}
        self._raw: dict[str, list] = {d: [] for d in self._drone_ids}
        self._raw_lock = threading.Lock()
        # Track IDs published last cycle — for DELETE marker cleanup
        self._prev_ids: set[str] = set()
        # Per-drone last-seen wall time — for swarm health
        self._drone_last_seen: dict[str, float] = {d: time.time() for d in self._drone_ids}

        for drone in self._drone_ids:
            self.create_subscription(
                String,
                f'/{drone}/radar/targets',
                lambda msg, d=drone: self._on_tracks(msg, d),
                10,
            )
            self.get_logger().info(f'Subscribed: /{drone}/radar/targets')

        self.asp_pub       = self.create_publisher(MarkerArray, '/swarm/asp',       10)
        self.track_pub     = self.create_publisher(String,      '/swarm/tracks',    10)
        self.situation_pub = self.create_publisher(String,      '/swarm/situation', 10)

        hz = self.get_parameter('publish_hz').value
        self.create_timer(1.0 / hz, self._fuse_and_publish)

        self.get_logger().info('Fusion node ready — Kalman tracker active')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _on_tracks(self, msg: String, drone_id: str) -> None:
        try:
            data = json.loads(msg.data)
            # Drop targets where TF failed — position is in base_link, not world frame.
            valid = [t for t in data.get('targets', []) if t.get('tf_ok', True)]
            with self._raw_lock:
                self._raw[drone_id] = valid
            self._drone_last_seen[drone_id] = time.time()
        except Exception:
            pass

    # ── Fuse & publish ────────────────────────────────────────────────────────

    def _fuse_and_publish(self) -> None:
        now = time.time()
        with self._raw_lock:
            raw_snap = {k: list(v) for k, v in self._raw.items()}
        self._tracker.update(raw_snap, now)
        fused = self._tracker.get_fused_tracks()

        stamp = self.get_clock().now().to_msg()
        ma    = MarkerArray()

        # Delete markers for tracks that were pruned this cycle
        current_ids = {t['id'] for t in fused}
        for dropped_id in (self._prev_ids - current_ids):
            self._delete_marker(ma, dropped_id)
        self._prev_ids = current_ids

        for t in fused:
            self._add_track_markers(ma, stamp, t)

        self.asp_pub.publish(ma)

        out = String()
        out.data = json.dumps({
            'n_fused':  len(fused),
            'tracks':   fused,
            'timestamp': now,
        })
        self.track_pub.publish(out)

        sit = String()
        sit.data = json.dumps(self._build_situation(fused, now))
        self.situation_pub.publish(sit)

        if fused:
            self.get_logger().info(
                f'ASP: {len(fused)} track(s): '
                + ', '.join(
                    f"{t['id']} R={t['range_m']:.0f}m "
                    f"Az={t['az_deg']:.0f}° "
                    f"v={t['vel']}"
                    for t in fused
                )
            )

    # ── Situation builder ─────────────────────────────────────────────────────

    def _build_situation(self, fused: list[dict], now: float) -> dict:
        health = {
            d: (now - self._drone_last_seen[d]) < 2.0
            for d in self._drone_ids
        }
        n_alive = sum(health.values())

        if any(t.get('range_m', float('inf')) < self._intercept_range for t in fused):
            decision = 'INTERCEPT'
        elif fused:
            decision = 'TRACK'
        else:
            decision = 'HOLD'

        return {
            'n_tracks':         len(fused),
            'tracks':           fused,
            'swarm_health':     health,
            'n_drones_alive':   n_alive,
            'decision_required': decision,
            'timestamp':        now,
        }

    # ── Marker helpers ────────────────────────────────────────────────────────

    @staticmethod
    def _track_hash(track_id: str) -> int:
        """Stable int ID for marker ns from FUSED_NNN string."""
        return int(track_id.split('_')[-1])

    def _delete_marker(self, ma: MarkerArray, track_id: str) -> None:
        m = Marker()
        m.ns     = 'asp'
        m.id     = self._track_hash(track_id) * 2
        m.action = Marker.DELETE
        ma.markers.append(m)
        m2 = Marker()
        m2.ns     = 'asp'
        m2.id     = self._track_hash(track_id) * 2 + 1
        m2.action = Marker.DELETE
        ma.markers.append(m2)

    def _add_track_markers(self, ma: MarkerArray, stamp, t: dict) -> None:
        mid  = self._track_hash(t['id'])
        n_dr = len(t['sources'])
        ratio = min(n_dr / max(len(self._drone_ids), 1), 1.0)

        sphere = Marker()
        sphere.header.stamp    = stamp
        sphere.header.frame_id = 'world'
        sphere.ns     = 'asp'
        sphere.id     = mid * 2
        sphere.type   = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position.x = t['pos'][0]
        sphere.pose.position.y = t['pos'][1]
        sphere.pose.position.z = t['pos'][2]
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 3.0
        sphere.color.r = 1.0 - ratio
        sphere.color.g = ratio
        sphere.color.b = 0.1
        sphere.color.a = 0.9
        sphere.lifetime.sec = 2
        ma.markers.append(sphere)

        vx, vy, vz = t['vel']
        speed = (vx**2 + vy**2 + vz**2) ** 0.5

        lbl = Marker()
        lbl.header.stamp    = stamp
        lbl.header.frame_id = 'world'
        lbl.ns     = 'asp'
        lbl.id     = mid * 2 + 1
        lbl.type   = Marker.TEXT_VIEW_FACING
        lbl.action = Marker.ADD
        lbl.pose.position.x = t['pos'][0]
        lbl.pose.position.y = t['pos'][1]
        lbl.pose.position.z = t['pos'][2] + 4.0
        lbl.pose.orientation.w = 1.0
        lbl.scale.z = 2.0
        lbl.color.r = lbl.color.g = lbl.color.b = 1.0
        lbl.color.a = 1.0
        lbl.text = (
            f"{t['id']}\n"
            f"R={t['range_m']:.0f}m  Az={t['az_deg']:.0f}°\n"
            f"v={speed:.1f}m/s  obs={t['n_obs']}\n"
            f"src:{','.join(t['sources'])}"
        )
        lbl.lifetime.sec = 2
        ma.markers.append(lbl)


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
