"""mocap_node and the robot frame (phase 3 of bb-calibration-one-attempt).

The node fits the frame from the labelled ``Base - n`` markers at ~5 Hz,
publishes a latched ``robot_frame`` PoseStamped on first fit / change, sets
``MocapDataMulti.aligned`` from the fit, reports it on mocap/status, and applies
it to the static world -> platform_start TF.
"""
from __future__ import annotations

import math
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from jugglebot import mocap_status as ms
from jugglebot import robot_frame as rf

PTS = {'Base - 1': (275.0, -0.8, -44.3), 'Base - 2': (61.0, 106.4, -27.7),
       'Base - 3': (-0.2, -38.4, -2.5), 'Base - 4': (-174.9, -0.6, -43.9),
       'Base - 5': (-0.1, -1.3, -2.5), 'Base - 6': (-0.5, 73.5, -1.4),
       'Base - 7': (0.8, 453.7, -34.6)}
Z0 = 20.0


def _Rz(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def _rows(R=None, t=(0.0, 0.0, 0.0), skip=()):
    R = np.eye(3) if R is None else R
    return [(n, *(R @ np.array(p) + t), 0.0) for n, p in PTS.items() if n not in skip]


def _node():
    import jugglebot.mocap_node as mn
    iface = MagicMock()
    iface.is_receiving.return_value = True
    iface.is_aligned = False
    iface.get_qtm_sync_status.return_value = {'synced': True}
    iface.get_ball_butler_markers_base_frame.return_value = np.zeros((5, 4))
    with patch.object(mn, 'MocapInterface', return_value=iface):
        node = mn.MocapNode(solve_inline=True)
    node._robot_tracker = rf.RobotFrameTracker(rf.RobotFrameTemplate(
        labels=tuple(PTS), points_mm=np.array(list(PTS.values()))))
    node._platform_z_mm = Z0
    return node


def _feed(node, rows, ns=1_000_000_000):
    node._feed_robot_frame(rows, ns)


def test_identity_publishes_identity_and_the_todays_tf():
    node = _node()
    _feed(node, _rows())
    msg = node.pub_robot_frame.published[-1]
    assert (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z) == (0.0, 0.0, 0.0)
    assert msg.pose.orientation.w == 1.0 and msg.header.frame_id == 'world'
    tf = node.static_tf_broadcaster.sent[-1].transform
    assert (tf.translation.x, tf.translation.y, tf.translation.z) == (0.0, 0.0, Z0)
    assert tf.rotation.w == 1.0
    node.mocap.set_aligned.assert_called_with(True)


def test_rotated_translated_frame_round_trips_and_moves_the_tf():
    node = _node()
    R, t = _Rz(7.0), np.array([12.0, -30.0, 5.0])
    _feed(node, _rows(R, t))
    msg = node.pub_robot_frame.published[-1]
    got = rf.RobotFrame.from_pose(
        (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z),
        (msg.pose.orientation.x, msg.pose.orientation.y,
         msg.pose.orientation.z, msg.pose.orientation.w))
    assert np.allclose(got.R, R, atol=1e-6) and np.allclose(got.t, t, atol=1e-6)
    tf = node.static_tf_broadcaster.sent[-1].transform
    exp = R @ np.array([0, 0, Z0]) + t
    assert np.allclose([tf.translation.x, tf.translation.y, tf.translation.z], exp, atol=1e-6)


def test_publishes_on_first_fit_and_change_only():
    node = _node()
    _feed(node, _rows(), 1_000_000_000)
    _feed(node, _rows(), 1_200_000_000)
    assert len(node.pub_robot_frame.published) == 1
    # Window medoid keeps the median; feed enough moved fits to move the frame.
    for k in range(12):
        _feed(node, _rows(t=(5.0, 0.0, 0.0)), 1_400_000_000 + k * 200_000_000)
    assert len(node.pub_robot_frame.published) == 2
    assert node.pub_robot_frame.published[-1].pose.position.x == pytest.approx(5.0, abs=1e-6)


def test_no_labelled_base_markers_is_unaligned_and_silent():
    node = _node()
    _feed(node, [])
    assert not node.pub_robot_frame.published
    node.mocap.set_aligned.assert_called_with(False)
    node.mocap.is_aligned = False
    node._publish_mocap_status()
    kv = {k.key: k.value for k in node.pub_mocap_status.published[-1].values}
    assert kv[ms.KEY_ROBOT_FRAME_OK] == '0'
    assert kv[ms.KEY_ROBOT_FRAME_REASON] == rf.REASON_TOO_FEW_MARKERS
    assert kv[ms.KEY_ALIGNED] == '0'


def test_status_reports_a_good_fit():
    node = _node()
    _feed(node, _rows())
    node._publish_mocap_status()
    kv = {k.key: k.value for k in node.pub_mocap_status.published[-1].values}
    assert kv[ms.KEY_ROBOT_FRAME_OK] == '1'
    assert kv[ms.KEY_ROBOT_FRAME_N_MATCHED] == '7'
    assert float(kv[ms.KEY_ROBOT_FRAME_RMS_MM]) < 1e-6
    assert kv[ms.KEY_ROBOT_FRAME_REASON] == ''


def test_feed_cadence_is_about_five_hz():
    node = _node()
    node.mocap.drain_frames.return_value = []
    fed = 0
    for k in range(300):                       # 300 frames in 1 s
        ns = 1_000_000_000 + k * 3_333_333
        if node._robot_feed_due(ns):
            fed += 1
            node._robot_last_t = ns * 1e-9
    assert 4 <= fed <= 6


def test_missing_template_leaves_the_frame_unknown(tmp_path):
    import jugglebot.mocap_node as mn
    node = _node()
    node._robot_tracker = None
    node._load_robot_frame(str(tmp_path / 'nope.json'))
    assert node._robot_tracker is None
    assert node._robot_feed_due(1_000_000_000) is False
    node._publish_mocap_status()
    kv = {k.key: k.value for k in node.pub_mocap_status.published[-1].values}
    assert kv[ms.KEY_ROBOT_FRAME_OK] == '0' and kv[ms.KEY_ROBOT_FRAME_REASON] == 'NO_TEMPLATE'


def _log_capture(node):
    lines = []
    for lvl in ('info', 'warn', 'warning', 'error'):
        setattr(node.get_logger(), lvl, lambda m, *a, _l=lvl, **k: lines.append((_l, m)))
    return lines


def _kv(node):
    node._publish_mocap_status()
    return {k.key: k.value for k in node.pub_mocap_status.published[-1].values}


def test_one_bad_feed_among_good_ones_changes_nothing():
    node = _node()
    for k in range(10):
        _feed(node, _rows(), 1_000_000_000 + k * 200_000_000)
    lines = _log_capture(node)
    node.mocap.set_aligned.reset_mock()
    _feed(node, [], 3_000_000_000)
    node.mocap.set_aligned.assert_called_with(True)
    assert lines == []
    assert _kv(node)[ms.KEY_ROBOT_FRAME_OK] == '1'


def test_window_of_bad_feeds_flips_unknown_with_one_warn():
    node = _node()
    for k in range(10):
        _feed(node, _rows(), 1_000_000_000 + k * 200_000_000)
    lines = _log_capture(node)
    for k in range(11):
        _feed(node, [], 3_000_000_000 + k * 200_000_000)
    node.mocap.set_aligned.assert_called_with(False)
    warns = [m for lvl, m in lines if lvl in ('warn', 'warning')]
    assert warns == ['Robot frame UNKNOWN: TOO_FEW_MARKERS']
    assert _kv(node)[ms.KEY_ROBOT_FRAME_OK] == '0'


def test_unchanged_frame_is_republished_every_five_seconds():
    node = _node()
    t0 = 1_000_000_000
    for k in range(56):                      # 11 s at 5 Hz
        _feed(node, _rows(), t0 + k * 200_000_000)
    n = len(node.pub_robot_frame.published)
    assert n == 3, n
    assert len(node.static_tf_broadcaster.sent) >= 3


# ── move observability: log lines and status keys ────────────────────────────

def _logs(node):
    out = {'info': [], 'warn': []}
    lg = node.get_logger()
    lg.info = lambda m, *a, **k: out['info'].append(m)
    lg.warn = lambda m, *a, **k: out['warn'].append(m)
    return out


def test_settled_info_then_moved_warn_with_magnitude_and_status_keys():
    node = _node()
    out = _logs(node)
    ns = 1_000_000_000
    for k in range(12):
        _feed(node, _rows(), ns + k * 200_000_000)
    settled = [m for m in out['info'] if m.startswith('Robot frame settled: ')]
    assert len(settled) == 1 and 'origin (0.0, 0.0, 0.0) mm, yaw 0.00 deg' in settled[0]
    kv = {k.key: k.value for k in node._robot_frame_status_values()}
    assert kv[ms.KEY_ROBOT_FRAME_SETTLED] == '1' and kv[ms.KEY_ROBOT_FRAME_MOVE_COUNT] == '0'
    assert kv[ms.KEY_ROBOT_FRAME_LAST_MOVE_S] == '0.000'
    assert kv[ms.KEY_ROBOT_FRAME_ORIGIN_MM] == '0.000,0.000,0.000'
    assert kv[ms.KEY_ROBOT_FRAME_YAW_DEG] == '0.000'
    t0 = ns + 3_000_000_000
    for k in range(30):
        _feed(node, _rows(t=(20.0, 0.0, 0.0)), t0 + k * 200_000_000)
    assert len(out['warn']) == 1
    w = out['warn'][0]
    assert w.startswith('Robot frame MOVED: 20.0 mm / 0.0 deg -> origin (20.0, 0.0, 0.0) mm')
    assert '(was (0.0, 0.0, 0.0) mm, 0.00 deg)' in w
    kv = {k.key: k.value for k in node._robot_frame_status_values()}
    assert kv[ms.KEY_ROBOT_FRAME_SETTLED] == '1' and kv[ms.KEY_ROBOT_FRAME_MOVE_COUNT] == '1'
    assert float(kv[ms.KEY_ROBOT_FRAME_LAST_MOVE_MM]) == pytest.approx(20.0, abs=0.1)
    assert float(kv[ms.KEY_ROBOT_FRAME_LAST_MOVE_S]) > 4.0
    assert kv[ms.KEY_ROBOT_FRAME_ORIGIN_MM] == '20.000,0.000,0.000'


def test_status_keys_without_a_frame():
    node = _node()
    kv = {k.key: k.value for k in node._robot_frame_status_values()}
    assert kv[ms.KEY_ROBOT_FRAME_SETTLED] == '0' and kv[ms.KEY_ROBOT_FRAME_ORIGIN_MM] == ''
    assert kv[ms.KEY_ROBOT_FRAME_YAW_DEG] == '' and kv[ms.KEY_ROBOT_FRAME_LAST_MOVE_MM] == '0.000'
