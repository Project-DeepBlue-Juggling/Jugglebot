"""Unit 3a: the tracker's landing plane follows the latched `robot_frame`.

The plane is the WORLD z of the robot-frame point (0, 0, landing_z_mm).
Identity -> today's number bit for bit; no message -> today's number.
"""
from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np

from jugglebot.ball_tracker_node import BallTrackerNode
from jugglebot.motion.skills import sites
from jugglebot.robot_frame import RobotFrame


def _pose(pos, quat=(0.0, 0.0, 0.0, 1.0)):
    return SimpleNamespace(pose=SimpleNamespace(
        position=SimpleNamespace(x=pos[0], y=pos[1], z=pos[2]),
        orientation=SimpleNamespace(x=quat[0], y=quat[1], z=quat[2], w=quat[3])))


def _node():
    node = BallTrackerNode()
    node._logger = MagicMock()
    node.get_logger = lambda: node._logger
    return node


def _roll_quat(deg):
    h = np.radians(deg) / 2.0
    return (float(np.sin(h)), 0.0, 0.0, float(np.cos(h)))


def test_no_message_plane_is_catch_cup_z_exactly():
    node = _node()
    assert node._landing_z == float(sites.CATCH_CUP_Z_MM)
    assert node._tracker.landing_z == float(sites.CATCH_CUP_Z_MM)


def test_identity_pose_is_bit_identical_and_silent():
    node = _node()
    node._on_robot_frame(_pose((0.0, 0.0, 0.0)))
    assert node._landing_z == float(sites.CATCH_CUP_Z_MM)
    assert node._tracker.landing_z == float(sites.CATCH_CUP_Z_MM)
    node._logger.info.assert_not_called()


def test_translation_and_roll_move_the_plane():
    node = _node()
    node.get_parameter  # parameter default is CATCH_CUP_Z_MM; pin z=830 below
    node._landing_z_robot = 830.0
    node._on_robot_frame(_pose((10.0, -20.0, 5.0)))
    assert node._tracker.landing_z == 835.0
    assert node._logger.info.call_count == 1

    q = _roll_quat(1.0)
    node._on_robot_frame(_pose((10.0, -20.0, 5.0), q))
    ref = RobotFrame.from_pose((10.0, -20.0, 5.0), q).to_world((0, 0, 830.0))[2]
    assert abs(node._tracker.landing_z - ref) < 1e-9
    assert node._landing_z == node._tracker.landing_z


def test_declared_parameter_overrides_the_default_robot_z():
    node = _node()
    node._on_set_parameters([SimpleNamespace(name='landing_z_mm', value=800.0)])
    assert node._tracker.landing_z == 800.0
    node._on_robot_frame(_pose((0.0, 0.0, 3.0)))
    assert node._tracker.landing_z == 803.0


def test_bad_quaternion_is_ignored_with_a_warning():
    node = _node()
    node._on_robot_frame(_pose((0.0, 0.0, 9.0), (0.0, 0.0, 0.0, 0.0)))
    assert node._tracker.landing_z == float(sites.CATCH_CUP_Z_MM)
    node._logger.warning.assert_called_once()
