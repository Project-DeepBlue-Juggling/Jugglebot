"""Tests for jugglebot.throw_director_node.ThrowDirectorNode.

Service handler logic — target resolution + inverse-ballistics call +
delegation to bb/send_throw_command.  ROS2 is mocked via tests/ros/conftest.py.
"""

from __future__ import annotations

import math

import pytest

from geometry_msgs.msg import Point
from jugglebot_interfaces.msg import (
    BallButlerCalibrationResult,
    RigidBodyPose,
    RigidBodyPoses,
)
from jugglebot_interfaces.srv import ThrowAtTarget


def _calibration(x_mm=0.0, y_mm=0.0, z_mm=0.0, yaw_offset_rad=0.0, success=True):
    msg = BallButlerCalibrationResult()
    msg.position_mm = Point(x=x_mm, y=y_mm, z=z_mm)
    msg.yaw_offset_rad = yaw_offset_rad
    msg.success = success
    return msg


def _rigid_bodies(named_positions):
    """Build a RigidBodyPoses msg from a dict {name: (x, y, z)} in mm."""
    msg = RigidBodyPoses()
    for name, (x, y, z) in named_positions.items():
        body = RigidBodyPose()
        body.name = name
        # body.pose.pose.position — three deep, matches the real msg structure
        body.pose.pose.position = Point(x=x, y=y, z=z)
        msg.bodies.append(body)
    return msg


@pytest.fixture
def node():
    from jugglebot.throw_director_node import ThrowDirectorNode
    return ThrowDirectorNode()


@pytest.fixture
def calibrated_node(node):
    node._on_bb_calibration(_calibration(x_mm=0.0, y_mm=0.0, z_mm=0.0,
                                         yaw_offset_rad=0.0))
    return node


class TestPreflight:
    def test_no_calibration_yet_returns_failure(self, node):
        req = ThrowAtTarget.Request()
        req.target_name = 'Catching_Cone'
        res = node._svc_throw_at_target(req, ThrowAtTarget.Response())
        assert res.success is False
        assert 'calibration' in res.message.lower()

    def test_unknown_target_returns_failure_with_known_list(self, calibrated_node):
        calibrated_node._on_rigid_bodies(_rigid_bodies({'Other_Body': (100, 0, 0)}))
        req = ThrowAtTarget.Request()
        req.target_name = 'Catching_Cone'
        res = calibrated_node._svc_throw_at_target(req, ThrowAtTarget.Response())
        assert res.success is False
        assert 'Catching_Cone' in res.message
        # Lists known bodies so the operator can fix the dropdown / QTM label
        assert 'Other_Body' in res.message

    def test_empty_target_list_lists_none(self, calibrated_node):
        req = ThrowAtTarget.Request()
        req.target_name = 'Catching_Cone'
        res = calibrated_node._svc_throw_at_target(req, ThrowAtTarget.Response())
        assert res.success is False
        assert '<none>' in res.message


class TestSuccessPath:
    def test_typical_throw_succeeds_and_invokes_can_node_client(self, calibrated_node):
        # Cone 1.2 m in front, slightly to the left, at BB origin Z.
        calibrated_node._on_rigid_bodies(_rigid_bodies({
            'Catching_Cone': (1200.0, 50.0, 0.0),
        }))
        # Spy on the throw service client
        client = calibrated_node._throw_client
        called_requests = []
        orig_call_async = client.call_async

        def _spy(req):
            called_requests.append(req)
            return orig_call_async(req)
        client.call_async = _spy

        req = ThrowAtTarget.Request()
        req.target_name = 'Catching_Cone'
        res = calibrated_node._svc_throw_at_target(req, ThrowAtTarget.Response())

        assert res.success is True
        assert 'Catching_Cone' in res.message
        assert 0.0 < res.throw_speed_mps <= 5.0
        assert math.radians(12.0) <= res.pitch_rad <= math.radians(85.0)
        assert res.predicted_tof_s > 0
        # Default delay applied (caller passed 0.0)
        assert res.throw_delay_s == pytest.approx(1.0)

        # bb/send_throw_command received an encoded request with the same
        # yaw/pitch/speed the director returned
        assert len(called_requests) == 1
        bb_req = called_requests[0]
        assert bb_req.yaw_angle_rad == pytest.approx(res.yaw_rad)
        assert bb_req.pitch_angle_rad == pytest.approx(res.pitch_rad)
        assert bb_req.throw_speed == pytest.approx(res.throw_speed_mps)
        assert bb_req.throw_time == pytest.approx(res.throw_delay_s)

    def test_caller_specified_delay_is_used(self, calibrated_node):
        calibrated_node._on_rigid_bodies(_rigid_bodies({
            'Catching_Cone': (1200.0, 0.0, 0.0),
        }))
        req = ThrowAtTarget.Request()
        req.target_name = 'Catching_Cone'
        req.throw_delay_s = 2.5
        res = calibrated_node._svc_throw_at_target(req, ThrowAtTarget.Response())
        assert res.success is True
        assert res.throw_delay_s == pytest.approx(2.5)

    def test_yaw_offset_applied_in_world_to_bb_local(self, calibrated_node):
        """With yaw_offset=+90°, a target on world +Y appears on BB-local +X."""
        # Re-calibrate this node fixture with a 90° yaw offset
        calibrated_node._on_bb_calibration(_calibration(
            x_mm=0.0, y_mm=0.0, z_mm=0.0, yaw_offset_rad=math.pi / 2))
        # Cone at world (+y) → BB local (+x).
        calibrated_node._on_rigid_bodies(_rigid_bodies({
            'Catching_Cone': (0.0, 1200.0, 0.0),
        }))
        req = ThrowAtTarget.Request()
        req.target_name = 'Catching_Cone'
        res = calibrated_node._svc_throw_at_target(req, ThrowAtTarget.Response())
        assert res.success is True
        # BB-local position field is populated and matches the transform
        assert res.target_position_bb_local_mm.x == pytest.approx(1200.0, abs=1e-6)
        assert res.target_position_bb_local_mm.y == pytest.approx(0.0, abs=1e-6)


class TestInfeasible:
    def test_target_too_far_returns_failure_with_solver_message(self, calibrated_node):
        # 100 m in front: no pitch satisfies the 5 m/s speed cap
        calibrated_node._on_rigid_bodies(_rigid_bodies({
            'Catching_Cone': (100_000.0, 0.0, 0.0),
        }))
        req = ThrowAtTarget.Request()
        req.target_name = 'Catching_Cone'
        res = calibrated_node._svc_throw_at_target(req, ThrowAtTarget.Response())
        assert res.success is False
        assert 'Inverse-ballistics failed' in res.message
        assert 'No feasible trajectory' in res.message

    def test_can_node_client_unavailable_returns_failure(self, calibrated_node):
        # Simulate can_node being down: client.service_is_ready() returns False
        calibrated_node._throw_client._ready = False
        calibrated_node._on_rigid_bodies(_rigid_bodies({
            'Catching_Cone': (1200.0, 0.0, 0.0),
        }))
        req = ThrowAtTarget.Request()
        req.target_name = 'Catching_Cone'
        res = calibrated_node._svc_throw_at_target(req, ThrowAtTarget.Response())
        assert res.success is False
        assert 'can_node' in res.message.lower()


class TestCacheUpdates:
    def test_rigid_body_cache_updates_per_message(self, calibrated_node):
        calibrated_node._on_rigid_bodies(_rigid_bodies({'A': (1, 2, 3)}))
        assert calibrated_node._target_positions_mm['A'] == (1, 2, 3)
        calibrated_node._on_rigid_bodies(_rigid_bodies({'A': (4, 5, 6), 'B': (7, 8, 9)}))
        assert calibrated_node._target_positions_mm['A'] == (4, 5, 6)
        assert calibrated_node._target_positions_mm['B'] == (7, 8, 9)

    def test_unsuccessful_calibration_ignored(self, node):
        node._on_bb_calibration(_calibration(success=False))
        assert node._bb_position_mm is None
