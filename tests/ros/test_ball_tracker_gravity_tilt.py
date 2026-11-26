"""FEAT-27: `ball_tracker_node`'s `gravity_world_xy_mmps2` parameter.

Config default (`hw.TRACKING_GRAVITY_WORLD_XY_MMPS2`, [0, 0]) reaches the
tracker; `ros2 param set` switches it live (the Sitting A/B of the measured
(43, 83) mm/s^2) and refuses anything but two finite numbers. The fit
arithmetic itself is `tests/tracking/test_flight_fit_gravity_tilt.py`.
"""
from __future__ import annotations

import math

from jugglebot.ball_tracker_node import BallTrackerNode
from tests.ros.conftest import _MockParameter


def test_parameter_defaults_to_the_config_value_and_reaches_the_tracker():
    import jugglebot.hardware_config as hw
    node = BallTrackerNode()
    want = tuple(float(v) for v in hw.TRACKING_GRAVITY_WORLD_XY_MMPS2)
    assert tuple(node.get_parameter('gravity_world_xy_mmps2').value) == want
    assert node._tracker.gravity_world_xy_mmps2 == want == (0.0, 0.0)


def test_param_set_switches_the_tracker_live_and_back():
    node = BallTrackerNode()
    res = node.set_parameters(
        [_MockParameter([43.0, 83.0], name='gravity_world_xy_mmps2')])
    assert all(r.successful for r in res)
    assert node._tracker.gravity_world_xy_mmps2 == (43.0, 83.0)
    node.set_parameters([_MockParameter([0.0, 0.0], name='gravity_world_xy_mmps2')])
    assert node._tracker.gravity_world_xy_mmps2 == (0.0, 0.0)


def test_param_set_refuses_a_malformed_vector_and_keeps_the_old_one():
    node = BallTrackerNode()
    node.set_parameters([_MockParameter([43.0, 83.0], name='gravity_world_xy_mmps2')])
    for bad in ([1.0], [1.0, 2.0, 3.0], [math.nan, 0.0], 'up'):
        res = node.set_parameters([_MockParameter(bad, name='gravity_world_xy_mmps2')])
        assert not res[0].successful, bad
        assert node._tracker.gravity_world_xy_mmps2 == (43.0, 83.0)
