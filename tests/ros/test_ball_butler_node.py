"""Tests for jugglebot.ball_butler_node.BallButlerNode.

Service handler logic — target resolution + inverse-ballistics call +
delegation to the bb/throw action.  ROS2 is mocked via tests/ros/conftest.py.
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
from jugglebot_interfaces.srv import BallButlerThrow


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
    """A ball_butler_node with aim-correction DISABLED so the test
    assertions exercise the raw transform / solver only.  Use the
    `node_with_correction` fixture (or set `_aim_correction_matrix`
    manually) to exercise the correction path."""
    from jugglebot.ball_butler_node import BallButlerNode
    n = BallButlerNode()
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
        req = BallButlerThrow.Request()
        req.target_name = 'Catching_Cone'
        res = node._svc_throw_at_target(req, BallButlerThrow.Response())
        assert res.success is False
        assert 'calibration' in res.message.lower()

    def test_unknown_target_returns_failure_with_known_list(self, calibrated_node):
        calibrated_node._on_rigid_bodies(_rigid_bodies({'Other_Body': (100, 0, 0)}))
        req = BallButlerThrow.Request()
        req.target_name = 'Catching_Cone'
        res = calibrated_node._svc_throw_at_target(req, BallButlerThrow.Response())
        assert res.success is False
        assert 'Catching_Cone' in res.message
        # Lists known bodies so the operator can fix the dropdown / QTM label
        assert 'Other_Body' in res.message

    def test_empty_target_list_lists_none(self, calibrated_node):
        req = BallButlerThrow.Request()
        req.target_name = 'Catching_Cone'
        res = calibrated_node._svc_throw_at_target(req, BallButlerThrow.Response())
        assert res.success is False
        assert '<none>' in res.message


class TestSuccessPath:
    def test_typical_throw_succeeds_and_invokes_throw_action(self, calibrated_node):
        # Cone 1.2 m in front, slightly to the left, at BB origin Z.
        calibrated_node._on_rigid_bodies(_rigid_bodies({
            'Catching_Cone': (1200.0, 50.0, 0.0),
        }))
        # Spy on the throw action client's goal send.
        action = calibrated_node._throw_action
        sent_goals = []
        orig_send = action.send_goal_async

        def _spy(goal):
            sent_goals.append(goal)
            return orig_send(goal)
        action.send_goal_async = _spy

        req = BallButlerThrow.Request()
        req.target_name = 'Catching_Cone'
        res = calibrated_node._svc_throw_at_target(req, BallButlerThrow.Response())

        assert res.success is True
        assert 'Catching_Cone' in res.message
        assert 0.0 < res.throw_speed_mps <= 5.0
        assert math.radians(12.0) <= res.pitch_rad <= math.radians(85.0)
        assert res.predicted_tof_s > 0
        # Default delay applied (caller passed 0.0).  Default is the
        # `throw_delay_s` ROS param (declared at 2.5 s — gives the slow
        # pitch axis time to settle before the hand fires).
        from jugglebot.ball_butler_node import _DEFAULT_THROW_DELAY_S
        assert res.throw_delay_s == pytest.approx(_DEFAULT_THROW_DELAY_S)

        # bb/throw received a goal with the same yaw/pitch/speed the node
        # returned, with announcement suppressed so the node's own
        # (solver-correct) announcement is the only one.
        assert len(sent_goals) == 1
        bb_goal = sent_goals[0]
        assert bb_goal.yaw_angle_rad == pytest.approx(res.yaw_rad)
        assert bb_goal.pitch_angle_rad == pytest.approx(res.pitch_rad)
        assert bb_goal.throw_speed == pytest.approx(res.throw_speed_mps)
        # The firmware throw_time is pulled EARLIER than the announced delay by
        # the constant command->release actuator latency, so the ball actually
        # leaves at the announced time (res.throw_delay_s itself is unchanged).
        import jugglebot.hardware_config as hw
        release_latency_s = hw.BB_OP_THROW_RELEASE_LATENCY_MS / 1000.0
        assert release_latency_s > 0.0
        assert bb_goal.throw_time == pytest.approx(
            res.throw_delay_s - release_latency_s
        )
        assert bb_goal.suppress_announcement is True

    def test_publishes_throw_announcement_with_solver_landing_geometry(self, calibrated_node):
        """The node's announcement should report landing_position = the
        actual target (cone position in world frame), and predicted_tof_sec
        matching the solver — not predict_throw's platform-plane projection."""
        cone_world = (1200.0, 50.0, 0.0)
        calibrated_node._on_rigid_bodies(_rigid_bodies({'Catching_Cone': cone_world}))

        req = BallButlerThrow.Request()
        req.target_name = 'Catching_Cone'
        res = calibrated_node._svc_throw_at_target(req, BallButlerThrow.Response())

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
        req = BallButlerThrow.Request()
        req.target_name = 'Catching_Cone'
        req.throw_delay_s = 2.5
        res = calibrated_node._svc_throw_at_target(req, BallButlerThrow.Response())
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
        req = BallButlerThrow.Request()
        req.target_name = 'Catching_Cone'
        res = calibrated_node._svc_throw_at_target(req, BallButlerThrow.Response())
        assert res.success is True
        # BB-local position field is populated and matches the transform
        assert res.target_position_bb_local_mm.x == pytest.approx(1200.0, abs=1e-6)
        assert res.target_position_bb_local_mm.y == pytest.approx(0.0, abs=1e-6)


class TestPointTargetExtension:
    """Phase-7 BallButlerThrow point-target extension: use_target_point skips QTM and
    aims at a caller-supplied world point; aim_only commands speed 0 (the 7a fast-path)."""

    def test_use_target_point_skips_qtm_lookup(self, calibrated_node):
        # No rigid bodies received at all — the QTM lookup would fail, but
        # use_target_point bypasses it entirely.
        action = calibrated_node._throw_action
        sent_goals = []
        orig_send = action.send_goal_async

        def _spy(goal):
            sent_goals.append(goal)
            return orig_send(goal)
        action.send_goal_async = _spy

        req = BallButlerThrow.Request()
        req.use_target_point = True
        req.target_point_global_mm = Point(x=1200.0, y=50.0, z=0.0)
        res = calibrated_node._svc_throw_at_target(req, BallButlerThrow.Response())

        assert res.success is True
        assert res.target_position_global_mm.x == pytest.approx(1200.0)
        assert res.target_position_global_mm.y == pytest.approx(50.0)
        assert len(sent_goals) == 1
        assert sent_goals[0].throw_speed > 0.0        # a real throw (not aim_only)

    def test_aim_only_commands_zero_speed_and_no_announcement(self, calibrated_node):
        action = calibrated_node._throw_action
        sent_goals = []
        orig_send = action.send_goal_async

        def _spy(goal):
            sent_goals.append(goal)
            return orig_send(goal)
        action.send_goal_async = _spy

        req = BallButlerThrow.Request()
        req.use_target_point = True
        req.aim_only = True
        req.target_point_global_mm = Point(x=1200.0, y=50.0, z=0.0)
        res = calibrated_node._svc_throw_at_target(req, BallButlerThrow.Response())

        assert res.success is True
        assert len(sent_goals) == 1
        assert sent_goals[0].throw_speed == pytest.approx(0.0)   # aim-only: no ball flies
        assert sent_goals[0].throw_time == pytest.approx(0.0)
        # No ThrowAnnouncement for an aim-only command (nothing to catch).
        assert len(calibrated_node.throw_announcement_pub.published) == 0

    def test_unknown_name_still_fails_when_not_use_target_point(self, calibrated_node):
        # The default path (use_target_point false) still requires a known QTM target.
        req = BallButlerThrow.Request()
        req.target_name = 'Nonexistent'
        res = calibrated_node._svc_throw_at_target(req, BallButlerThrow.Response())
        assert res.success is False
        assert 'not in latest rigid_body_poses' in res.message


class TestInfeasible:
    def test_target_too_far_returns_failure_with_solver_message(self, calibrated_node):
        # 100 m in front: no pitch satisfies the 5 m/s speed cap
        calibrated_node._on_rigid_bodies(_rigid_bodies({
            'Catching_Cone': (100_000.0, 0.0, 0.0),
        }))
        req = BallButlerThrow.Request()
        req.target_name = 'Catching_Cone'
        res = calibrated_node._svc_throw_at_target(req, BallButlerThrow.Response())
        assert res.success is False
        assert 'Inverse-ballistics failed' in res.message
        assert 'No feasible trajectory' in res.message

    def test_throw_action_unavailable_returns_failure(self, calibrated_node):
        # Simulate the throw action provider (teensy_bridge_node) being down:
        # action.server_is_ready() returns False.
        calibrated_node._throw_action._server_ready = False
        calibrated_node._on_rigid_bodies(_rigid_bodies({
            'Catching_Cone': (1200.0, 0.0, 0.0),
        }))
        req = BallButlerThrow.Request()
        req.target_name = 'Catching_Cone'
        res = calibrated_node._svc_throw_at_target(req, BallButlerThrow.Response())
        assert res.success is False
        assert 'bb/throw' in res.message


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
        req = BallButlerThrow.Request()
        req.target_name = 'Catching_Cone'
        res = calibrated_node._svc_throw_at_target(req, BallButlerThrow.Response())

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
        req = BallButlerThrow.Request()
        req.target_name = 'Catching_Cone'
        res = calibrated_node._svc_throw_at_target(req, BallButlerThrow.Response())
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

    def test_load_with_invert_inverts_the_matrix(self, node, tmp_path):
        """`invert=True` loads the mathematical inverse — applying the
        inverted matrix to a point and then the original to the result
        should return the original point."""
        import json
        # Pick a matrix with non-trivial 2x2 + translation
        mat = [[2.0, 0.0, 10.0],
               [0.0, 0.5, -5.0],
               [0.0, 0.0,  1.0]]
        path = tmp_path / 'm.json'
        path.write_text(json.dumps({'matrix': mat}))
        node._aim_correction_matrix = None
        node._load_aim_correction(str(path), invert=True)
        assert node._aim_correction_matrix is not None
        # Inverse of [[2,0,10],[0,0.5,-5]] is [[0.5,0,-5],[0,2,10]]
        (a, b, tx), (c, d, ty) = node._aim_correction_matrix
        assert (a, b, tx) == pytest.approx((0.5, 0.0, -5.0))
        assert (c, d, ty) == pytest.approx((0.0, 2.0, 10.0))


class TestInvertAffine3x3:
    """Module-level helper: invert a 3x3 affine matrix in pure Python."""

    def test_inverts_identity_to_identity(self):
        from jugglebot.ball_butler_node import _invert_affine_3x3
        I = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        inv = _invert_affine_3x3(I)
        for r in range(3):
            for c in range(3):
                assert inv[r][c] == pytest.approx(I[r][c])

    def test_inverts_pure_translation(self):
        from jugglebot.ball_butler_node import _invert_affine_3x3
        T = [[1.0, 0.0, 10.0], [0.0, 1.0, -5.0], [0.0, 0.0, 1.0]]
        inv = _invert_affine_3x3(T)
        # Inverse of pure translation just negates the offsets
        assert inv[0][2] == pytest.approx(-10.0)
        assert inv[1][2] == pytest.approx(5.0)
        # Linear part still identity
        assert (inv[0][0], inv[0][1]) == pytest.approx((1.0, 0.0))
        assert (inv[1][0], inv[1][1]) == pytest.approx((0.0, 1.0))

    def test_inverts_rotation_plus_translation_correctly(self):
        """A·A^-1 ≈ I (numerically)."""
        from jugglebot.ball_butler_node import _invert_affine_3x3
        import math
        ang = math.radians(20.0)
        A = [[math.cos(ang), -math.sin(ang), 100.0],
             [math.sin(ang),  math.cos(ang),  50.0],
             [0.0, 0.0, 1.0]]
        Ainv = _invert_affine_3x3(A)
        # Multiply A @ Ainv manually (3x3)
        prod = [[sum(A[i][k] * Ainv[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
        I = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        for r in range(3):
            for c in range(3):
                assert prod[r][c] == pytest.approx(I[r][c], abs=1e-9)

    def test_raises_on_singular_matrix(self):
        from jugglebot.ball_butler_node import _invert_affine_3x3
        S = [[1.0, 2.0, 0.0], [2.0, 4.0, 0.0], [0.0, 0.0, 1.0]]  # rows linearly dependent
        with pytest.raises(ValueError, match='singular'):
            _invert_affine_3x3(S)


class TestThrowDelayParam:
    """The `throw_delay_s` ROS param sets the default delay for the throw."""

    def test_default_is_value_of_module_constant(self, calibrated_node):
        from jugglebot.ball_butler_node import _DEFAULT_THROW_DELAY_S
        assert calibrated_node._default_throw_delay_s == pytest.approx(
            _DEFAULT_THROW_DELAY_S)

    def test_caller_zero_uses_default(self, calibrated_node):
        from jugglebot.ball_butler_node import _DEFAULT_THROW_DELAY_S
        calibrated_node._on_rigid_bodies(_rigid_bodies({
            'Catching_Cone': (1200.0, 0.0, 0.0),
        }))
        req = BallButlerThrow.Request()
        req.target_name = 'Catching_Cone'
        req.throw_delay_s = 0.0
        res = calibrated_node._svc_throw_at_target(req, BallButlerThrow.Response())
        assert res.success is True
        assert res.throw_delay_s == pytest.approx(_DEFAULT_THROW_DELAY_S)


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
