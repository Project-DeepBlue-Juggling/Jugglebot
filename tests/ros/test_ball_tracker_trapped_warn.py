"""FEAT-50: the Tracker node logs (log-only) a throttled WARN for a marker dwelling under the Hand."""
from __future__ import annotations

import time
from types import SimpleNamespace
from unittest.mock import MagicMock

from jugglebot.ball_tracker_node import BallTrackerNode
from jugglebot.tracking import trapped_ball


def _node():
    node = BallTrackerNode()
    node._logger = MagicMock()
    node.get_logger = lambda: node._logger
    return node


def _poses():
    pose = SimpleNamespace(pose=SimpleNamespace(
        position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)))
    return SimpleNamespace(bodies=[SimpleNamespace(name='Platform', pose=pose)])


def _frame(t, label=''):
    # world z = initial height + local z -> Platform-frame (16, 22, -26)
    m = SimpleNamespace(position=SimpleNamespace(
        x=16.0, y=22.0, z=trapped_ball.INITIAL_HEIGHT_MM - 26.0), label=label)
    sec = int(t)
    return SimpleNamespace(markers=[m], stamp=SimpleNamespace(
        sec=sec, nanosec=int((t - sec) * 1e9)))


def _drive(node, seconds, t0=1000.0):
    for i in range(int(seconds * 300)):
        node._on_mocap(_frame(t0 + i / 300.0))


def _under_warnings(node):
    return [c for c in node._logger.warning.call_args_list if 'under the Hand' in c[0][0]]


def test_warns_and_throttles_to_one_hertz():
    node = _node()
    node._on_rigid_body_poses(_poses())
    node._on_hand_telemetry(SimpleNamespace(pos_meas=2.6))
    _drive(node, 2.5)
    w = _under_warnings(node)
    assert len(w) == 3, w        # ~0.3 s, ~1.3 s, ~2.3 s
    assert 'Hand at 2.60 rev' in w[0][0][0] and '(16, 22, -26) mm' in w[0][0][0]


def test_labelled_marker_is_ignored():
    node = _node()
    node._on_rigid_body_poses(_poses())
    node._on_hand_telemetry(SimpleNamespace(pos_meas=2.6))
    for i in range(600):
        node._on_mocap(_frame(1000.0 + i / 300.0, label='Ball Butler - 1'))
    assert _under_warnings(node) == []


def test_no_hand_telemetry_or_pose_or_stale_means_silence():
    node = _node()
    node._on_rigid_body_poses(_poses())
    _drive(node, 1.5)                       # no telemetry yet
    assert _under_warnings(node) == []
    node = _node()
    node._on_hand_telemetry(SimpleNamespace(pos_meas=2.6))
    _drive(node, 1.5)                       # no Platform pose yet
    assert _under_warnings(node) == []
    node = _node()
    node._on_rigid_body_poses(_poses())
    node._on_hand_telemetry(SimpleNamespace(pos_meas=2.6))
    node._trapped_hand_mono = time.monotonic() - 5.0   # stale
    _drive(node, 1.5)
    assert _under_warnings(node) == []
    node._logger.warning.assert_not_called()
