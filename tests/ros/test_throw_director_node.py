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
    """A throw_director_node with aim-correction DISABLED so the test
    assertions exercise the raw transform / solver only.  Use the
    `node_with_correction` fixture (or set `_aim_correction_matrix`
    manually) to exercise the correction path."""
    from jugglebot.throw_director_node import ThrowDirectorNode
    n = ThrowDirectorNode()
    # Force identity regardless of what's installed at share/jugglebot/resources/
    n._aim_correction_matrix = None
    n._aim_correction_source = '(disabled by test)'
    return n


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
        # yaw/pitch/speed the director returned, with announcement suppressed
        # so the director's own (solver-correct) announcement is the only one.
        assert len(called_requests) == 1
        bb_req = called_requests[0]
        assert bb_req.yaw_angle_rad == pytest.approx(res.yaw_rad)
        assert bb_req.pitch_angle_rad == pytest.approx(res.pitch_rad)
        assert bb_req.throw_speed == pytest.approx(res.throw_speed_mps)
        assert bb_req.throw_time == pytest.approx(res.throw_delay_s)
        assert bb_req.suppress_announcement is True

    def test_publishes_throw_announcement_with_solver_landing_geometry(self, calibrated_node):
        """The director's announcement should report landing_position = the
        actual target (cone position in world frame), and predicted_tof_sec
        matching the solver — not predict_throw's platform-plane projection."""
        cone_world = (1200.0, 50.0, 0.0)
        calibrated_node._on_rigid_bodies(_rigid_bodies({'Catching_Cone': cone_world}))

        req = ThrowAtTarget.Request()
        req.target_name = 'Catching_Cone'
        res = calibrated_node._svc_throw_at_target(req, ThrowAtTarget.Response())

        pub = calibrated_node.throw_announcement_pub
        assert len(pub.published) == 1, "expected exactly one ThrowAnnouncement"
        ann = pub.published[0]

        # Landing position is the actual cone position, not a projection
        assert ann.landing_position.x == pytest.approx(cone_world[0])
        assert ann.landing_position.y == pytest.approx(cone_world[1])
        assert ann.landing_position.z == pytest.approx(cone_world[2])
        # ToF matches the solver's
        assert ann.predicted_tof_sec == pytest.approx(res.predicted_tof_s)
        # Target identified by name so a multi-target session can match
        assert ann.target_id == 'Catching_Cone'
        assert ann.thrower_name == 'ball_butler'

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


class TestAimCorrection:
    """The 2D affine aim correction applied between global_to_bb_local
    and solve_throw_local."""

    def test_identity_when_no_matrix_loaded(self, node):
        """No correction loaded → input passes through unchanged."""
        assert node._aim_correction_matrix is None
        x_c, y_c = node._apply_aim_correction(1234.0, 5678.0)
        assert x_c == 1234.0
        assert y_c == 5678.0

    def test_matrix_applied_correctly(self, node):
        # Pure-translation matrix: shift by (+100, -50)
        node._aim_correction_matrix = ((1.0, 0.0, 100.0), (0.0, 1.0, -50.0))
        x_c, y_c = node._apply_aim_correction(1000.0, 500.0)
        assert x_c == pytest.approx(1100.0)
        assert y_c == pytest.approx(450.0)

    def test_full_affine_applied(self, node):
        # 90° rotation + translation: (x, y) → (-y, x) + (10, 20)
        node._aim_correction_matrix = ((0.0, -1.0, 10.0), (1.0, 0.0, 20.0))
        x_c, y_c = node._apply_aim_correction(100.0, 200.0)
        assert x_c == pytest.approx(-200.0 + 10.0)
        assert y_c == pytest.approx(100.0 + 20.0)

    def test_correction_feeds_solver_via_service(self, calibrated_node):
        """When correction is loaded, the solver receives the corrected
        BB-local position (not the raw one)."""
        # Pick a matrix that shifts target by (+200, 0) in BB-local frame
        calibrated_node._aim_correction_matrix = (
            (1.0, 0.0, 200.0), (0.0, 1.0, 0.0))
        # Cone at world (1000, 0, 0) with no BB yaw offset → BB-local (1000, 0)
        calibrated_node._on_rigid_bodies(_rigid_bodies({
            'Catching_Cone': (1000.0, 0.0, 0.0),
        }))
        req = ThrowAtTarget.Request()
        req.target_name = 'Catching_Cone'
        res = calibrated_node._svc_throw_at_target(req, ThrowAtTarget.Response())

        assert res.success is True
        # Solver got the corrected target → solver yaw is now toward (1200, 0)
        # which is straight ahead → yaw ≈ 0.
        assert res.target_position_bb_local_mm.x == pytest.approx(1200.0)
        assert res.target_position_bb_local_mm.y == pytest.approx(0.0)
        # The message should surface the correction delta
        assert 'aim corr' in res.message

    def test_no_correction_note_when_disabled(self, calibrated_node):
        """With correction off, the success message has no '[aim corr: ...]' tag."""
        assert calibrated_node._aim_correction_matrix is None
        calibrated_node._on_rigid_bodies(_rigid_bodies({
            'Catching_Cone': (1000.0, 0.0, 0.0),
        }))
        req = ThrowAtTarget.Request()
        req.target_name = 'Catching_Cone'
        res = calibrated_node._svc_throw_at_target(req, ThrowAtTarget.Response())
        assert res.success is True
        assert 'aim corr' not in res.message

    def test_load_from_file(self, node, tmp_path):
        """A valid JSON file at the given absolute path is loaded."""
        import json
        path = tmp_path / 'mat.json'
        path.write_text(json.dumps({
            'matrix': [[0.5, 0.0, 10.0],
                       [0.0, 0.5, 20.0],
                       [0.0, 0.0, 1.0]],
            'n_pairs': 12,
            'provenance': {'warning': 'test fixture — not real'},
        }))
        node._aim_correction_matrix = None
        node._load_aim_correction(str(path))
        assert node._aim_correction_matrix is not None
        assert node._aim_correction_matrix[0] == (0.5, 0.0, 10.0)
        assert node._aim_correction_matrix[1] == (0.0, 0.5, 20.0)

    def test_missing_file_falls_back_to_identity(self, node):
        node._aim_correction_matrix = None
        node._load_aim_correction('/nonexistent/path/missing.json')
        assert node._aim_correction_matrix is None  # identity (no correction)

    def test_malformed_matrix_falls_back_to_identity(self, node, tmp_path):
        """A bad matrix shape doesn't crash the node — falls back to identity
        so the rest of the system keeps working."""
        import json
        path = tmp_path / 'bad.json'
        path.write_text(json.dumps({'matrix': [[1, 2]]}))  # wrong shape
        node._aim_correction_matrix = None
        node._load_aim_correction(str(path))
        assert node._aim_correction_matrix is None


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
