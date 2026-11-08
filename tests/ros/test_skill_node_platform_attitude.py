"""The report-only Platform attitude line of the frame check (owner, 2026-11-08).

After the 2026-11-07 Sitting a re-seat left the Platform about 2.7 deg off its
seat and the position-only frame check passed it. `_platform_attitude` reports
the mocap Platform body's tilt and yaw in the robot frame; the node logs it at
each frame check with the change since the previous check of the launch.
Report only: it refuses nothing."""

from __future__ import annotations

import math
from unittest.mock import MagicMock

import numpy as np
import pytest

from jugglebot import skill_node as sn
from jugglebot.robot_frame import RobotFrame
from tests.ros.test_skill_node import _frame_ready, _node_with_client


def _quat(axis, deg):
    """(qx, qy, qz, qw) for a rotation of ``deg`` about unit ``axis``."""
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    h = math.radians(deg) / 2.0
    return tuple(a * math.sin(h)) + (math.cos(h),)


def _samples(q, n=30, now=100.0):
    return [(now - 0.01 * i,) + tuple(q) for i in range(n)]


def test_identity_is_level_and_unturned():
    att = sn._platform_attitude(_samples((0.0, 0.0, 0.0, 1.0)), 100.0)
    assert att.samples == 30
    for v in (att.tilt_deg, att.about_x_deg, att.about_y_deg, att.yaw_deg):
        assert abs(v) < 1e-9


def test_yaw_and_each_tilt_axis_are_read_with_their_sign():
    att = sn._platform_attitude(_samples(_quat((0, 0, 1), 2.4)), 100.0)
    assert att.yaw_deg == pytest.approx(2.4, abs=1e-9)
    assert att.tilt_deg == pytest.approx(0.0, abs=1e-9)
    att = sn._platform_attitude(_samples(_quat((0, 1, 0), 1.3)), 100.0)
    assert att.about_y_deg == pytest.approx(1.3, abs=1e-9)
    assert att.tilt_deg == pytest.approx(1.3, abs=1e-9)
    assert att.about_x_deg == pytest.approx(0.0, abs=1e-9)
    att = sn._platform_attitude(_samples(_quat((1, 0, 0), -0.7)), 100.0)
    assert att.about_x_deg == pytest.approx(-0.7, abs=1e-9)
    assert att.tilt_deg == pytest.approx(0.7, abs=1e-9)


def test_robot_frame_rotation_is_removed():
    """A body that turned WITH the robot frame reads unturned in it."""
    q = _quat((0, 0, 1), 17.0)
    R = RobotFrame.from_pose((0.0, 0.0, 0.0), q).R
    att = sn._platform_attitude(_samples(q), 100.0, R)
    assert att.yaw_deg == pytest.approx(0.0, abs=1e-6)
    assert att.tilt_deg == pytest.approx(0.0, abs=1e-6)


def test_sign_flipped_quaternions_average_correctly():
    q = _quat((0, 0, 1), 3.0)
    flipped = tuple(-c for c in q)
    mixed = _samples(q, 15) + _samples(flipped, 15, now=99.99)
    att = sn._platform_attitude(mixed, 100.0)
    assert att.yaw_deg == pytest.approx(3.0, abs=1e-6)


def test_stale_too_few_or_nonfinite_samples_are_not_measurable():
    q = (0.0, 0.0, 0.0, 1.0)
    assert sn._platform_attitude(_samples(q, 30, now=50.0), 100.0) is None
    assert sn._platform_attitude(_samples(q, sn._FRAME_CHECK_MIN_SAMPLES - 1), 100.0) is None
    bad = _samples((float('nan'), 0.0, 0.0, 1.0), 30)
    assert sn._platform_attitude(bad, 100.0) is None


def test_line_reports_the_change_since_the_previous_check():
    before = sn._platform_attitude(_samples(_quat((0, 0, 1), 0.66)), 100.0)
    after = sn._platform_attitude(_samples(_quat((0, 0, 1), 3.05)), 100.0)
    first = sn._attitude_line(before)
    assert 'first check this launch' in first and 'report only' in first
    line = sn._attitude_line(after, before)
    assert 'yaw +3.05 deg' in line and 'yaw +2.39 deg' in line
    assert 'not measurable' in sn._attitude_line(None, before)


def test_node_logs_the_attitude_at_each_frame_check_and_never_refuses():
    node, _client = _node_with_client(frame=False)
    _frame_ready(node)
    now = node._frame_clock()
    for _ in range(sn._FRAME_CHECK_MIN_SAMPLES + 5):
        node._platform_mocap_quat.append((now,) + _quat((0, 0, 1), 0.66))
    node._logger = MagicMock()
    node.get_logger = lambda: node._logger
    assert node._frame_check_error() == ''
    lines = [c[0][0] for c in node._logger.debug.call_args_list]
    first = [s for s in lines if 'Platform attitude' in s]
    assert len(first) == 1 and 'first check this launch' in first[0]

    # A re-seat turns the body 2.39 deg: the next check says so, still passes.
    node._platform_mocap_quat.clear()
    for _ in range(sn._FRAME_CHECK_MIN_SAMPLES + 5):
        node._platform_mocap_quat.append((now,) + _quat((0, 0, 1), 3.05))
    node._logger.reset_mock()
    assert node._frame_check_error() == ''
    lines = [c[0][0] for c in node._logger.debug.call_args_list]
    second = [s for s in lines if 'Platform attitude' in s]
    assert len(second) == 1 and 'yaw +2.39 deg' in second[0]


def test_node_attitude_without_orientation_samples_is_a_line_not_a_refusal():
    node, _client = _node_with_client(frame=False)
    _frame_ready(node)
    node._logger = MagicMock()
    node.get_logger = lambda: node._logger
    assert node._frame_check_error() == ''
    lines = [c[0][0] for c in node._logger.debug.call_args_list]
    assert any('Platform attitude: not measurable' in s for s in lines)
