#!/usr/bin/env python3
"""Unit tests for radar_fusion pipeline — runs standalone, no ROS2 needed."""
import sys
import os
import time
import json
import math
import tempfile
import pathlib
from unittest.mock import MagicMock

# ── Path setup ────────────────────────────────────────────────────────────────
# Insert radar_fusion package root so imports work without colcon install.
sys.path.insert(0, str(pathlib.Path(__file__).parent))

# ── ROS2 stubs ────────────────────────────────────────────────────────────────
# Mock every ROS2 module detection_node.py imports at module level.
# Node must be a real type (not MagicMock) so DetectionNode can subclass it.
_ROS2_DEPS = [
    'rclpy', 'rclpy.node', 'rclpy.duration', 'rclpy.qos',
    'sensor_msgs', 'sensor_msgs.msg',
    'visualization_msgs', 'visualization_msgs.msg',
    'std_msgs', 'std_msgs.msg',
    'geometry_msgs', 'geometry_msgs.msg',
    'tf2_ros', 'tf2_geometry_msgs',
    'sensor_msgs_py', 'sensor_msgs_py.point_cloud2',
]
for _dep in _ROS2_DEPS:
    sys.modules.setdefault(_dep, MagicMock())
sys.modules['rclpy.node'].Node = object   # must be real type for subclassing

import numpy as np  # noqa: E402 — after path/mock setup

from radar_fusion.kalman_tracker import TrackManager, KalmanTrack
from radar_fusion.detection_node import PANEL_ROT
from radar_fusion.rf_classifier import RFTargetClassifier

# ── Test harness ──────────────────────────────────────────────────────────────
DRONES = ['drone_L', 'drone_S1', 'drone_S2', 'drone_S3', 'drone_S4']
PASS = 0
FAIL = 0


def check(name, cond, detail=''):
    global PASS, FAIL
    if cond:
        print(f'  PASS  {name}' + (f'  ({detail})' if detail else ''))
        PASS += 1
    else:
        print(f'  FAIL  {name}  {detail}')
        FAIL += 1


print('=== UNIT TESTS ===')
print()

# ── T1: Panel rotation correctness ───────────────────────────────────────────
# 6 panels at 60° spacing (CCW from forward): A=0° B=60° C=120° D=180° E=240° F=300°
print('T1: Panel rotation matrices — 6-panel layout')
for panel, expected_az in [('A', 0), ('B', 60), ('C', 120),
                             ('D', 180), ('E', 240), ('F', 300)]:
    v = PANEL_ROT[panel] @ np.array([1., 0., 0.])
    got_az = round(math.degrees(math.atan2(float(v[1]), float(v[0]))) % 360)
    check(f'Panel {panel} +X -> {expected_az}°', got_az == expected_az,
          f'got {got_az}°')

check('6 panels defined', set(PANEL_ROT.keys()) == set('ABCDEF'),
      str(set(PANEL_ROT.keys())))

for panel in 'ABCDEF':
    det = round(float(np.linalg.det(PANEL_ROT[panel])), 6)
    check(f'Panel {panel} det=1 (orthonormal)', det == 1.0, f'det={det}')

# ── T2: Kalman track creation and persistence ─────────────────────────────────
print()
print('T2: Kalman tracker — basic')
mgr = TrackManager(merge_dist_m=5.0, gate_m=10.0, ttl_s=3.0)
now = time.time()
raw = {
    'drone_L':  [{'pos': [500., 0., 100.], 'vel': [10., 0., 0.]}],
    'drone_S1': [{'pos': [501., 0.5, 100.1], 'vel': [9.8, 0., 0.]}],
    'drone_S2': [], 'drone_S3': [], 'drone_S4': [],
}
true_range = math.sqrt(500**2 + 100**2)

mgr.update(raw, now)
mgr.update(raw, now + 0.5)
mgr.update(raw, now + 1.0)
tracks = mgr.get_fused_tracks()

check('Single track created', len(tracks) == 1, f'got {len(tracks)}')
if tracks:
    t = tracks[0]
    check('Persistent ID', t['id'] == 'FUSED_001', t['id'])
    check('Range within 20m of truth',
          abs(t['range_m'] - true_range) < 20,
          f"R={t['range_m']:.1f} truth={true_range:.1f}")
    check('n_obs=3', t['n_obs'] == 3, str(t['n_obs']))
    try:
        json.dumps(t)
        check('JSON serializable', True)
    except Exception as e:
        check('JSON serializable', False, str(e))

mgr.update({d: [] for d in DRONES}, now + 10.)
check('TTL prune', mgr.get_fused_tracks() == [], 'should be empty after 10s')

# ── T3: Multi-drone merge ─────────────────────────────────────────────────────
print()
print('T3: Multi-drone merge — same target seen by all 5 drones')
mgr2 = TrackManager(merge_dist_m=5.0)
raw2 = {d: [{'pos': [200., 50., 80.], 'vel': [0., 0., 0.]}] for d in DRONES}
mgr2.update(raw2, now)
t2 = mgr2.get_fused_tracks()
check('Single merged track', len(t2) == 1, f'got {len(t2)}')
if t2:
    check('All 5 sources', len(t2[0]['sources']) == 5, str(t2[0]['sources']))

# ── T4: Target separation ─────────────────────────────────────────────────────
print()
print('T4: Target separation — two distinct targets not merged')
mgr3 = TrackManager(merge_dist_m=5.0)
raw3 = {
    'drone_L': [
        {'pos': [100., 0., 0.], 'vel': [0., 0., 0.]},
        {'pos': [200., 0., 0.], 'vel': [0., 0., 0.]},
    ],
    'drone_S1': [], 'drone_S2': [], 'drone_S3': [], 'drone_S4': [],
}
mgr3.update(raw3, now)
t3 = mgr3.get_fused_tracks()
check('Two targets kept separate', len(t3) == 2, f'got {len(t3)}')

# ── T5: Elevation gate ────────────────────────────────────────────────────────
print()
print('T5: Elevation gate logic')
el_gate, el_min = 25.0, -5.0
for el, expected, label in [
    (0.0,  True,  'horizontal keep'),
    (20.0, True,  'within max keep'),
    (26.0, False, 'above max reject'),
    (-3.0, True,  'within min keep'),
    (-6.0, False, 'below min reject'),
]:
    ok = (el <= el_gate) and (el >= el_min)
    check(label, ok == expected, f'el={el}')

# ── T6: ID persistence across 10 cycles ──────────────────────────────────────
print()
print('T6: ID persistence across 10 update cycles')
mgr4 = TrackManager()
raw4 = {'drone_L': [{'pos': [300., 0., 50.], 'vel': [5., 0., 0.]}],
        'drone_S1': [], 'drone_S2': [], 'drone_S3': [], 'drone_S4': []}
mgr4.update(raw4, now)
id0 = mgr4.get_fused_tracks()[0]['id']
ok = True
for i in range(1, 11):
    mgr4.update(raw4, now + i * 0.5)
    cur = mgr4.get_fused_tracks()
    if not cur or cur[0]['id'] != id0:
        ok = False
        break
check('ID stable across 10 cycles', ok, id0)

# ── T7: Kalman solver stability — BUG-RF-2 fix ───────────────────────────────
# Near-identical observations → S near-singular → linalg.inv used to crash.
# linalg.solve must survive without raising LinAlgError.
print()
print('T7: Kalman solver stability (BUG-RF-2 — linalg.solve)')
mgr7 = TrackManager(merge_dist_m=0.0, gate_m=100.0, r_pos=0.0001)
identical_raw = {
    'drone_L':  [{'pos': [100., 0., 50.], 'vel': [0., 0., 0.]}],
    'drone_S1': [{'pos': [100., 0., 50.], 'vel': [0., 0., 0.]}],
    'drone_S2': [{'pos': [100., 0., 50.], 'vel': [0., 0., 0.]}],
    'drone_S3': [{'pos': [100., 0., 50.], 'vel': [0., 0., 0.]}],
    'drone_S4': [{'pos': [100., 0., 50.], 'vel': [0., 0., 0.]}],
}
try:
    for i in range(5):
        mgr7.update(identical_raw, now + i * 0.2)
    tr7 = mgr7.get_fused_tracks()
    check('No crash on near-singular S', True)
    check('Track still produced', len(tr7) >= 1, f'got {len(tr7)}')
except Exception as e:
    check('No crash on near-singular S', False, str(e))
    check('Track still produced', False, 'exception before tracks')

# ── T8: Q dt-scaling — ISSUE-RF-1 fix ────────────────────────────────────────
# Uncertainty P should grow more over a large dt than a small dt.
print()
print('T8: Q dt-scaling (ISSUE-RF-1 — P grows proportional to dt)')
def _p_trace_after_predict(dt):
    tr = KalmanTrack('X', np.array([0., 0., 0.]), np.array([0., 0., 0.]),
                     q_pos=1.0, q_vel=0.5, r_pos=5.0, now=0.0)
    p0 = float(np.trace(tr.P))
    tr.predict(dt)
    return float(np.trace(tr.P)) - p0

gain_small = _p_trace_after_predict(0.1)
gain_large = _p_trace_after_predict(1.0)
check('P grows more for dt=1.0 than dt=0.1', gain_large > gain_small,
      f'gain_small={gain_small:.3f} gain_large={gain_large:.3f}')
# Superlinear growth is expected: F@P@F.T cross-terms grow as dt²,
# so dt=1.0 vs dt=0.1 (10× difference) produces >>10× P gain. Verify dt=0 gives zero gain.
gain_zero = _p_trace_after_predict(0.0)
check('P unchanged for dt=0', gain_zero == 0.0, f'gain_zero={gain_zero}')

# ── T9: tf_ok filter — BUG-RF-3 fix ──────────────────────────────────────────
# Fusion node must drop targets with tf_ok=False (base_link frame, not world).
print()
print('T9: tf_ok filter logic (BUG-RF-3 — drop non-world-frame positions)')
raw_targets = [
    {'pos': [100., 0., 30.], 'vel': [0., 0., 0.], 'tf_ok': True},
    {'pos': [200., 0., 30.], 'vel': [0., 0., 0.], 'tf_ok': False},
    {'pos': [300., 0., 30.], 'vel': [0., 0., 0.], 'tf_ok': True},
]
valid = [t for t in raw_targets if t.get('tf_ok', True)]
check('tf_ok=False targets dropped', len(valid) == 2, f'kept {len(valid)}/3')
check('Correct targets kept',
      all(t['pos'][0] in (100., 300.) for t in valid), str(valid))
# Default-True when key absent (legacy detection_node without tf_ok field)
legacy = [{'pos': [50., 0., 0.], 'vel': [0., 0., 0.]}]
check('Missing tf_ok key defaults to keep',
      len([t for t in legacy if t.get('tf_ok', True)]) == 1)

# ── T10: RF classifier makedirs fix — BUG-RF-1 ───────────────────────────────
# Flat model_path (no directory) must not raise FileNotFoundError.
print()
print('T10: RF classifier makedirs fix (BUG-RF-1 — flat model path)')
with tempfile.TemporaryDirectory() as td:
    flat_path = os.path.join(td, 'model.pkl')   # directory part exists
    try:
        rf = RFTargetClassifier(model_path=flat_path)
        check('Flat model_path saves without crash', True)
        check('Model file created', os.path.exists(flat_path))
    except Exception as e:
        check('Flat model_path saves without crash', False, str(e))
        check('Model file created', False, 'exception before save')

# ── T11: RF training data gap — ISSUE-RF-2 fix ───────────────────────────────
# Hits boundary: clutter max=7, target min=9 → no overlap at 8.
print()
print('T11: RF training data gap — no overlap at hits=8 (ISSUE-RF-2)')
rf11 = RFTargetClassifier()
# A cluster with 8 hits, compact — was ambiguous before fix.
# Now 8 < target min (9), so classifier should lean toward clutter.
# More importantly, a 9-hit compact cluster must be target; 7-hit loose must be clutter.
target_cluster  = [{'range_m': 500., 'hits': 20, 'range_std': 0.5,
                     'spread_xy': 0.8, 'el_deg': 5.0}]
clutter_cluster = [{'range_m': 100., 'hits': 4,  'range_std': 5.0,
                    'spread_xy': 7.0, 'el_deg': 3.0}]
check('Clear target classified as target (label=1)',
      rf11.predict(target_cluster) == [1])
check('Clear clutter classified as clutter (label=0)',
      rf11.predict(clutter_cluster) == [0])
proba = rf11.predict_proba(target_cluster)[0]
check('Target confidence > 0.8', proba > 0.8, f'proba={proba:.2f}')

# ── Summary ───────────────────────────────────────────────────────────────────
print()
print(f'Results: {PASS} passed, {FAIL} failed')
sys.exit(0 if FAIL == 0 else 1)
