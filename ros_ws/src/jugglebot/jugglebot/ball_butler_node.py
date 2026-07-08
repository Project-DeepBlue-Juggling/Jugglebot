"""ROS2 node: high-level Ball Butler commanding.

Single owner of Cartesian-target → joint-space throws for BallButler. Two
modes of operation:

  1. Single-shot aim-and-throw at a named QTM rigid body via
     ``bb/throw_at_target`` (BallButlerThrow.srv) — looks up the target by
     name in the latest ``/rigid_body_poses``, transforms into BB local
     frame, applies optional 2D affine aim correction, solves inverse
     ballistics, and sends the ``bb/throw`` action on teensy_bridge_node
     (whose result carries the firmware's terminal throw outcome).

  2. Accuracy-calibration session via ``bb/start_accuracy_calibration``
     (std_srvs/Trigger) — generates a 5×5 grid in the global frame,
     randomises the firing order (logged seed), and drives BB through 50
     throws using the same per-throw logic as (1), waiting on
     ``bb/heartbeat`` between throws for position-settle + reload.

Time-signal note: all timing flows through can_node's existing pipeline
(host ROS clock = CLOCK_REALTIME = Unix epoch; BB Teensy clock is TimeSync-
locked to the same epoch via bus.broadcast_time).  This node does not
introduce any new clocks.
"""
from __future__ import annotations

import json
import math
import os
import random
from datetime import datetime
from pathlib import Path
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from geometry_msgs.msg import Point, Vector3
from std_srvs.srv import Trigger

from ament_index_python.packages import get_package_share_directory

from jugglebot_interfaces.msg import (
    BallButlerCalibrationResult,
    BallButlerHeartbeat,
    RigidBodyPoses,
    ThrowAnnouncement,
)
from jugglebot_interfaces.srv import BallButlerThrow
from jugglebot_interfaces.action import BallButlerThrowCmd

import jugglebot.hardware_config as hw
from jugglebot.can.throw_ballistics import (
    ThrowSolution,
    global_to_bb_local,
    solve_throw_local,
)
from jugglebot.protocol_config import BallButlerStates


# Default release delay (s) from the bb/throw_at_target service call to the
# actual ball release.  Pitch is the slow axis: trap-traj caps at 1 rev/s
# (360°/s) with 0.5 rev/s² accel, so a 60° pitch change takes ~1.6 s of pure
# motion + settling.  The BB firmware does NOT check yaw/pitch settled before
# firing the hand — if the hand wind-up finishes while pitch is still
# traversing, the ball gets released at the wrong angle.  Default 2.5 s
# leaves ~2 s for axis motion + ~0.5 s margin; tune via the `throw_delay_s`
# ROS param (or the service request field, which still wins when > 0).
_DEFAULT_THROW_DELAY_S = 2.5

# Gravity (mm/s²) — for landing-velocity decay in published ThrowAnnouncement.
_G_MMPS2 = hw.GRAVITY_MPS2 * 1000.0

# Default location of the 2D affine correction matrix (BB-local frame).
# The file maps desired-landing → commanded-target so that, when run
# through the BB hardware, the ball lands at the desired position.
# The default file is packaged as a share/jugglebot/resources/ artifact.
_DEFAULT_AIM_CORRECTION_FILE = 'throw_affine_correction.json'


def _invert_affine_3x3(m):
    """Invert a 3x3 affine matrix (last row [0, 0, 1]).

    Returns the 3x3 inverse as a list of lists.  Raises ``ValueError`` if
    the upper-left 2x2 is singular.  Standalone module-level helper so the
    node code stays compact and the math is easy to unit-test.
    """
    a, b, tx = m[0]
    c, d, ty = m[1]
    det = a * d - b * c
    if abs(det) < 1e-12:
        raise ValueError(f"Affine matrix is singular (det={det:.2e})")
    inv_det = 1.0 / det
    # 2x2 inverse times -translation
    a2 =  d * inv_det
    b2 = -b * inv_det
    c2 = -c * inv_det
    d2 =  a * inv_det
    tx2 = -(a2 * tx + b2 * ty)
    ty2 = -(c2 * tx + d2 * ty)
    return [[a2, b2, tx2], [c2, d2, ty2], [0.0, 0.0, 1.0]]


def generate_grid_targets(
    center: Tuple[float, float, float],
    size: Tuple[float, float],
    divisions: Tuple[int, int],
) -> list[Tuple[float, float, float]]:
    """Regular grid of (x, y, z) targets in the global frame, row-by-row."""
    cx, cy, cz = center
    sx, sy = size
    dx, dy = divisions
    step_x = sx / (dx - 1) if dx > 1 else 0.0
    step_y = sy / (dy - 1) if dy > 1 else 0.0
    start_x = cx - sx / 2
    start_y = cy - sy / 2
    return [
        (start_x + i * step_x, start_y + j * step_y, cz)
        for j in range(dy)
        for i in range(dx)
    ]


class BallButlerNode(Node):
    """High-level BallButler commander.

    Owns Cartesian-target → joint-space IK for both single-shot throws
    (``bb/throw_at_target``) and the accuracy-calibration volley
    (``bb/start_accuracy_calibration``).
    """

    # Accuracy-calibration state machine phases
    _CAL_PHASE_IDLE = 0
    _CAL_PHASE_AWAITING_POSITION = 1
    _CAL_PHASE_WAITING_RELOAD = 2

    def __init__(self):
        super().__init__('ball_butler_node')

        # Latest cache of rigid-body poses from mocap_node, keyed by name.
        # Stored as world-frame mm tuples (x, y, z).
        self._target_positions_mm: Dict[str, Tuple[float, float, float]] = {}

        # BB calibration (latched).  None until first /bb/calibration_result.
        self._bb_position_mm: Optional[Tuple[float, float, float]] = None
        self._bb_yaw_offset_rad: float = 0.0

        # Latest heartbeat (used by the accuracy-calibration state machine)
        self._bb_state: Optional[int] = None
        self._bb_yaw_deg: float = 0.0
        self._bb_pitch_deg: float = 0.0
        self._last_commanded_yaw_deg: float = 0.0
        self._last_commanded_pitch_deg: float = 0.0

        # ── Throw release delay (s) ──
        # See _DEFAULT_THROW_DELAY_S docstring at module scope for why the
        # default is 2.5 s (pitch is the slow axis; firmware doesn't gate
        # the hand on axis-settled).  Caller can also override per-call
        # via the service request's throw_delay_s field.
        self.declare_parameter('throw_delay_s', _DEFAULT_THROW_DELAY_S)
        self._default_throw_delay_s = float(
            self.get_parameter('throw_delay_s').value)

        # ── Affine aim-correction (2D) ──
        # Default: DISABLED.  The shipped session 1+2 matrix is in WORLD frame
        # and encodes biases (rotation, translation, scale) that subsequent
        # bug fixes (QTM cal, global_to_bb_frame rotation, FK s/d/l offsets,
        # pitch_z_offset) have already corrected in our code path.  Applying
        # it makes throws worse, not better — confirmed in the first live
        # session.  Enable explicitly when a fresh, BB-local calibration grid
        # is available.
        self.declare_parameter('apply_aim_correction', False)
        self.declare_parameter('aim_correction_file', _DEFAULT_AIM_CORRECTION_FILE)
        # `invert_aim_correction`: if True, the loaded matrix is inverted at
        # load time.  Diagnostic switch for testing whether the matrix is in
        # the wrong direction (caller-suggested fix when throws got worse).
        self.declare_parameter('invert_aim_correction', False)
        self._aim_correction_matrix: Optional[Tuple[Tuple[float, float, float],
                                                    Tuple[float, float, float]]] = None
        self._aim_correction_source: str = '(disabled)'
        if self.get_parameter('apply_aim_correction').value:
            self._load_aim_correction(
                self.get_parameter('aim_correction_file').value,
                invert=bool(self.get_parameter('invert_aim_correction').value))

        # ── Accuracy-calibration session params ──
        self.declare_parameter('calib_grid_center_mm', [0.0, 0.0, 750.0])
        self.declare_parameter('calib_grid_size_mm', [1000.0, 1000.0])
        self.declare_parameter('calib_grid_divisions', [5, 5])
        self.declare_parameter('calib_throws_per_cell', 2)
        self.declare_parameter('calib_random_seed', 42)
        self.declare_parameter('calib_throw_delay_s', 2.5)
        self.declare_parameter('calib_yaw_tolerance_deg', 0.5)
        self.declare_parameter('calib_pitch_tolerance_deg', 0.5)
        self.declare_parameter('calib_settle_time_s', 0.1)
        self.declare_parameter('calib_min_reload_heartbeats', 3)
        self.declare_parameter('calib_tracking_keepalive_s', 0.5)
        self.declare_parameter(
            'calib_session_dir',
            os.path.join(os.path.expanduser('~'), 'bb_calibration_sessions'))

        # Accuracy-calibration runtime state
        self._cal_active: bool = False
        self._cal_cancelled: bool = False
        self._cal_phase: int = self._CAL_PHASE_IDLE
        self._cal_throws: list[Tuple[int, Tuple[float, float, float]]] = []
        self._cal_idx: int = 0
        self._cal_position_reached_t: Optional[float] = None
        self._cal_reload_heartbeats: int = 0
        self._cal_keepalive_timer: Optional[rclpy.timer.Timer] = None
        self._cal_session_path: Optional[Path] = None

        # ── Subscriptions ───────────────────────────────────────
        self.create_subscription(
            RigidBodyPoses, 'rigid_body_poses', self._on_rigid_bodies, 10)

        # BB calibration is published with latched QoS by mocap_node; subscribe
        # with matching transient-local durability so we pick it up on startup.
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            BallButlerCalibrationResult, 'bb/calibration_result',
            self._on_bb_calibration, latched)

        # Heartbeat: only used by the accuracy-calibration state machine,
        # but always subscribed so we have current state available.
        self.create_subscription(
            BallButlerHeartbeat, 'bb/heartbeat', self._on_heartbeat, 20)

        # ── Action client (bb/throw — replaces bb/send_throw_command) ──
        # Fire-and-forget from the throw pipeline's view (we don't block on the
        # result), but the firmware's terminal outcome is logged when it arrives,
        # via the goal-response → result callback chain.
        self._throw_action = ActionClient(self, BallButlerThrowCmd, 'bb/throw')

        # ── Publisher: ThrowAnnouncement (own, not can_node's) ──
        # We send suppress_announcement=True on bb/throw so the bridge / can_node
        # skips its predict_throw-based announcement (which uses the platform
        # default catch height; wrong for off-plane targets like the cone).  We
        # publish here using the solver's actual target z and serial-chain ToF.
        self.throw_announcement_pub = self.create_publisher(
            ThrowAnnouncement, 'throw_announcements', 10)

        # ── Service servers ─────────────────────────────────────
        self.create_service(
            BallButlerThrow, 'bb/throw_at_target', self._svc_throw_at_target)
        self.create_service(
            Trigger, 'bb/start_accuracy_calibration',
            self._svc_start_accuracy_calibration)
        self.create_service(
            Trigger, 'bb/cancel_accuracy_calibration',
            self._svc_cancel_accuracy_calibration)

        self.get_logger().info(
            'BallButler node ready (bb/throw_at_target, '
            'bb/start_accuracy_calibration). '
            f'Aim correction: {self._aim_correction_source}'
        )

    # ─────────────────────────────────────────────────────────────────
    # Subscription handlers
    # ─────────────────────────────────────────────────────────────────

    def _on_rigid_bodies(self, msg: RigidBodyPoses):
        """Cache the latest world-frame position of every named rigid body."""
        for body in msg.bodies:
            p = body.pose.pose.position
            # QTM publishes in mm via mocap_node (per its docstring); store as-is.
            self._target_positions_mm[body.name] = (p.x, p.y, p.z)

    def _on_bb_calibration(self, msg: BallButlerCalibrationResult):
        """Store BB calibration so global→local transforms work."""
        if not msg.success:
            return
        self._bb_position_mm = (msg.position_mm.x, msg.position_mm.y, msg.position_mm.z)
        self._bb_yaw_offset_rad = float(msg.yaw_offset_rad)
        self.get_logger().info(
            f'BB calibration received: pos={self._bb_position_mm}, '
            f'yaw_offset={math.degrees(self._bb_yaw_offset_rad):.2f}°')

    def _on_heartbeat(self, msg: BallButlerHeartbeat):
        """Cache current BB state; drive the calibration FSM when active."""
        self._bb_state = msg.state
        self._bb_yaw_deg = msg.yaw_deg
        self._bb_pitch_deg = msg.pitch_deg
        if self._cal_active:
            self._handle_calibration_heartbeat(msg)

    # ─────────────────────────────────────────────────────────────────
    # Aim correction (2D affine on BB-local target)
    # ─────────────────────────────────────────────────────────────────

    def _load_aim_correction(self, file_path_or_name: str,
                             invert: bool = False) -> None:
        """Load the 2D affine correction matrix from a JSON file.

        Looks up the file in this order:
          1. As an absolute or relative path (if it exists).
          2. As a basename inside `share/jugglebot/resources/`.

        ``invert``: if True, the loaded matrix is mathematically inverted
        before storage (3x3 affine inverse).  Useful as a diagnostic switch
        when a matrix's direction is suspect.

        On any failure, falls back to identity (no correction) and logs
        the reason — never raises, so the node still starts.
        """
        candidates = [file_path_or_name]
        try:
            share_dir = get_package_share_directory('jugglebot')
            candidates.append(os.path.join(share_dir, 'resources', file_path_or_name))
        except Exception:
            pass

        path = next((p for p in candidates if os.path.isfile(p)), None)
        if path is None:
            self.get_logger().warning(
                f'Aim-correction file not found ({file_path_or_name}); '
                f'proceeding without correction (identity).')
            return

        try:
            with open(path, 'r') as f:
                data = json.load(f)
            mat = data.get('matrix')
            if mat is None or len(mat) < 2 or any(len(row) != 3 for row in mat[:2]):
                raise ValueError("missing or malformed 'matrix' (need 2x3 or 3x3)")

            # Pad to 3x3 for inversion if needed
            mat3 = [
                [float(mat[0][0]), float(mat[0][1]), float(mat[0][2])],
                [float(mat[1][0]), float(mat[1][1]), float(mat[1][2])],
                [0.0, 0.0, 1.0],
            ]
            if invert:
                mat3 = _invert_affine_3x3(mat3)

            self._aim_correction_matrix = (
                (mat3[0][0], mat3[0][1], mat3[0][2]),
                (mat3[1][0], mat3[1][1], mat3[1][2]),
            )
            self._aim_correction_source = path + (' (inverted)' if invert else '')
            prov = data.get('provenance', {})
            warn = prov.get('warning')
            self.get_logger().info(
                f'Aim correction loaded from {path}'
                f'{" (INVERTED)" if invert else ""} '
                f'(n_pairs={data.get("n_pairs", "?")}).')
            if warn:
                self.get_logger().warning(f'Aim correction warning: {warn}')
        except Exception as e:
            self.get_logger().error(
                f'Failed to load aim correction from {path}: {e}. '
                f'Proceeding without correction.')
            self._aim_correction_matrix = None

    def _apply_aim_correction(self, x_l: float, y_l: float) -> Tuple[float, float]:
        """Apply the loaded 2D affine to a BB-local (x, y) target.

        Returns (x, y) unchanged when no correction is loaded.
        """
        m = self._aim_correction_matrix
        if m is None:
            return x_l, y_l
        (a, b, tx), (c, d, ty) = m
        return a * x_l + b * y_l + tx, c * x_l + d * y_l + ty

    # ─────────────────────────────────────────────────────────────────
    # Core throw logic — shared by named-target service and calibration
    # ─────────────────────────────────────────────────────────────────

    def _throw_at_global_point(
        self,
        x_g: float,
        y_g: float,
        z_g: float,
        target_id: str,
        throw_delay_s: Optional[float] = None,
        throw: bool = True,
    ) -> Tuple[bool, str, Optional[ThrowSolution], float]:
        """Solve IK and send a throw command for a global-frame target.

        Returns (ok, message, solution, delay_s). When ``throw`` is False the
        speed is sent as 0 (track-only / aim-only).
        """
        if self._bb_position_mm is None:
            return False, 'BB calibration not yet received', None, 0.0

        x_l, y_l, z_l = global_to_bb_local(
            x_g, y_g, z_g,
            bb_position_mm=self._bb_position_mm,
            yaw_offset_rad=self._bb_yaw_offset_rad,
        )
        x_cmd, y_cmd = self._apply_aim_correction(x_l, y_l)

        try:
            sol = solve_throw_local(x_cmd, y_cmd, z_l)
        except ValueError as e:
            return False, f'Inverse-ballistics failed: {e}', None, 0.0

        self._last_commanded_yaw_deg = math.degrees(sol.yaw_rad)
        self._last_commanded_pitch_deg = math.degrees(sol.pitch_rad)

        delay_s = (
            float(throw_delay_s)
            if (throw_delay_s is not None and throw_delay_s > 0)
            else self._default_throw_delay_s
        )

        if not self._throw_action.server_is_ready():
            return False, 'bb/throw action server not available', sol, delay_s

        goal = BallButlerThrowCmd.Goal()
        goal.yaw_angle_rad = float(sol.yaw_rad)
        goal.pitch_angle_rad = float(sol.pitch_rad)
        goal.throw_speed = float(sol.speed_mps) if throw else 0.0
        # Compensate the hand's constant command→release actuator latency: command the
        # throw this much EARLIER so the ball ARRIVES at the requested (pos, time),
        # rather than ~44 ms late. The announced landing time below is left equal to the
        # true arrival (now + delay_s + tof) — after compensation the actual release
        # lands at now + delay_s, so the announcement is correct. The latency is constant
        # across the workspace (no range/speed/pitch dependence), so one value suffices.
        release_latency_s = hw.BB_OP_THROW_RELEASE_LATENCY_MS / 1000.0
        goal.throw_time = float(max(0.0, delay_s - release_latency_s)) if throw else 0.0
        # Suppress the predict_throw-based announcement (platform default catch
        # height); we publish our own using the solver's actual target z + ToF.
        goal.suppress_announcement = True
        # Fire-and-forget: don't block the throw pipeline on the result. The
        # firmware outcome (relayed CMD_RESULT) is logged when it arrives.
        send_future = self._throw_action.send_goal_async(goal)
        send_future.add_done_callback(
            lambda fut, tid=target_id: self._on_throw_goal_response(fut, tid))

        if throw:
            self._publish_throw_announcement(
                target_name=target_id,
                target_global_mm=(x_g, y_g, z_g),
                sol=sol,
                delay_s=delay_s,
            )

        correction_note = ''
        if self._aim_correction_matrix is not None:
            dx, dy = x_cmd - x_l, y_cmd - y_l
            if abs(dx) > 0.5 or abs(dy) > 0.5:
                correction_note = f' [aim corr: Δxy=({dx:+.0f},{dy:+.0f}) mm BB-local]'

        verb = 'Throwing at' if throw else 'Aiming at'
        msg = (
            f'{verb} {target_id or f"({x_g:.0f},{y_g:.0f},{z_g:.0f}) mm"}: '
            f'yaw={math.degrees(sol.yaw_rad):.1f}°, '
            f'pitch={math.degrees(sol.pitch_rad):.1f}°, '
            f'speed={sol.speed_mps:.2f} m/s, delay={delay_s:.2f} s, '
            f'predicted ToF={sol.tof_s:.3f} s.{correction_note}'
        )
        return True, msg, sol, delay_s

    # ── bb/throw result chain (async, non-blocking — logs the firmware outcome) ──

    def _on_throw_goal_response(self, future, target_id: str):
        """Goal accepted/rejected by the bridge action server; chain to the result."""
        try:
            goal_handle = future.result()
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'bb/throw goal send failed: {e}')
            return
        if not goal_handle.accepted:
            self.get_logger().warn(
                f'bb/throw goal REJECTED for {target_id or "point"} '
                '(a throw is already outstanding?)')
            return
        goal_handle.get_result_async().add_done_callback(
            lambda fut, tid=target_id: self._on_throw_result(fut, tid))

    def _on_throw_result(self, future, target_id: str):
        """Firmware terminal outcome (relayed CMD_RESULT) — log success/reason."""
        try:
            result = future.result().result
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'bb/throw result error: {e}')
            return
        tgt = target_id or 'point'
        if result.success:
            self.get_logger().info(f'bb/throw OK for {tgt}: {result.message}')
        else:
            self.get_logger().warn(f'bb/throw NOT thrown for {tgt}: {result.message}')

    # ─────────────────────────────────────────────────────────────────
    # Single-shot service: bb/throw_at_target
    # ─────────────────────────────────────────────────────────────────

    def _svc_throw_at_target(self, req: BallButlerThrow.Request,
                             res: BallButlerThrow.Response) -> BallButlerThrow.Response:
        # Stage 1: BB calibration must be present.
        if self._bb_position_mm is None:
            res.success = False
            res.message = ('BB calibration not yet received — calibrate from '
                           'the GUI first.')
            return res

        # Stage 2: resolve the aim point. Either a caller-supplied world-frame point
        # (use_target_point — the Phase-7 reload path, which computes the catch point
        # itself and skips QTM) or a QTM rigid-body lookup by name. Default-zero fields
        # keep every existing target_name caller on the lookup path.
        if req.use_target_point:
            p = req.target_point_global_mm
            x_g, y_g, z_g = float(p.x), float(p.y), float(p.z)
            target_id = req.target_name or 'point'
        else:
            target = self._target_positions_mm.get(req.target_name)
            if target is None:
                known = sorted(self._target_positions_mm.keys()) or ['<none>']
                res.success = False
                res.message = (
                    f"Target '{req.target_name}' not in latest rigid_body_poses. "
                    f"Known: {', '.join(known)}.")
                return res
            x_g, y_g, z_g = target
            target_id = req.target_name

        res.target_position_global_mm = Point(x=x_g, y=y_g, z=z_g)

        # Reject if a calibration session is currently driving throws.
        if self._cal_active:
            res.success = False
            res.message = 'Accuracy calibration in progress — cancel it first'
            return res

        # Stages 3-5: world → BB local, aim correction, IK, send command. aim_only
        # commands speed 0 (track/aim geometry only, no ball leaves) — the Phase-7a
        # frame-convention verification fast-path; it also suppresses the throw
        # announcement (no ball => nothing to catch).
        ok, msg, sol, delay_s = self._throw_at_global_point(
            x_g, y_g, z_g,
            target_id=target_id,
            throw_delay_s=req.throw_delay_s,
            throw=not req.aim_only,
        )

        # Echo back the *commanded* (post-correction) BB-local position so
        # callers can see where the solver actually aimed.
        if sol is not None:
            x_l, y_l, z_l = global_to_bb_local(
                x_g, y_g, z_g,
                bb_position_mm=self._bb_position_mm,
                yaw_offset_rad=self._bb_yaw_offset_rad,
            )
            x_cmd, y_cmd = self._apply_aim_correction(x_l, y_l)
            res.target_position_bb_local_mm = Point(x=x_cmd, y=y_cmd, z=z_l)
            res.yaw_rad = float(sol.yaw_rad)
            res.pitch_rad = float(sol.pitch_rad)
            res.throw_speed_mps = float(sol.speed_mps)
            res.throw_delay_s = float(delay_s)
            res.predicted_tof_s = float(sol.tof_s)

        res.success = ok
        res.message = msg
        self.get_logger().info(msg) if ok else self.get_logger().warning(msg)
        return res

    def _publish_throw_announcement(self, target_name: str,
                                    target_global_mm: Tuple[float, float, float],
                                    sol: ThrowSolution, delay_s: float) -> None:
        """Publish a ThrowAnnouncement with solver-correct landing geometry.

        Unlike can_node's predict_throw-based announcement (which uses the
        platform default catch height), this uses the solver's actual target z
        and serial-chain-aware ToF — so cone catches are correlated against
        the *actual* expected arrival time at the cone, not when the ball
        would have crossed the platform catch plane.
        """
        x_g, y_g, z_g = target_global_mm

        # Velocity at release (in world frame).  The solver yaws in BB local
        # frame; convert to world by adding yaw_offset.
        world_yaw = sol.yaw_rad + self._bb_yaw_offset_rad
        cos_p = math.cos(sol.pitch_rad)
        sin_p = math.sin(sol.pitch_rad)
        v_mmps = sol.speed_mps * 1000.0
        vx = v_mmps * cos_p * math.cos(world_yaw)
        vy = v_mmps * cos_p * math.sin(world_yaw)
        vz = v_mmps * sin_p
        # Landing velocity: horizontal unchanged, vertical decays under g.
        vz_land = vz - _G_MMPS2 * sol.tof_s

        now = self.get_clock().now()
        throw_time = now + rclpy.time.Duration(seconds=delay_s)
        landing_time = throw_time + rclpy.time.Duration(seconds=sol.tof_s)

        ann = ThrowAnnouncement()
        ann.header.stamp = now.to_msg()
        ann.header.frame_id = 'world'
        ann.thrower_name = 'ball_butler'
        ann.initial_position = Point(x=self._bb_position_mm[0],
                                     y=self._bb_position_mm[1],
                                     z=self._bb_position_mm[2])
        ann.initial_velocity = Vector3(x=vx, y=vy, z=vz)
        ann.target_id = target_name
        ann.throw_time = throw_time.to_msg()
        ann.predicted_tof_sec = float(sol.tof_s)
        ann.landing_position = Point(x=x_g, y=y_g, z=z_g)
        ann.landing_velocity = Vector3(x=vx, y=vy, z=vz_land)
        ann.landing_time = landing_time.to_msg()

        self.throw_announcement_pub.publish(ann)

    # ─────────────────────────────────────────────────────────────────
    # Accuracy-calibration session
    # ─────────────────────────────────────────────────────────────────

    def _svc_start_accuracy_calibration(self, request, response):
        if self._cal_active:
            response.success = False
            response.message = 'Accuracy calibration already in progress'
            return response
        if self._bb_position_mm is None:
            response.success = False
            response.message = 'BB calibration not yet received — calibrate first'
            return response

        center = tuple(self.get_parameter('calib_grid_center_mm').value)
        size = tuple(self.get_parameter('calib_grid_size_mm').value)
        divisions = tuple(int(d) for d in
                          self.get_parameter('calib_grid_divisions').value)
        throws_per_cell = int(self.get_parameter('calib_throws_per_cell').value)
        seed = int(self.get_parameter('calib_random_seed').value)

        targets = generate_grid_targets(center, size, divisions)

        # Pre-filter for reachability so we don't waste time mid-session.
        reachable: list[Tuple[int, Tuple[float, float, float]]] = []
        skipped: list[Tuple[int, Tuple[float, float, float], str]] = []
        for idx, t in enumerate(targets):
            x_l, y_l, z_l = global_to_bb_local(
                *t,
                bb_position_mm=self._bb_position_mm,
                yaw_offset_rad=self._bb_yaw_offset_rad,
            )
            x_cmd, y_cmd = self._apply_aim_correction(x_l, y_l)
            try:
                solve_throw_local(x_cmd, y_cmd, z_l)
            except ValueError as e:
                skipped.append((idx, t, str(e)))
                continue
            reachable.append((idx, t))

        if not reachable:
            response.success = False
            response.message = (
                f'All {len(targets)} grid targets unreachable — '
                f'check geometry / limits / BB calibration'
            )
            return response

        # Build randomised schedule (throws_per_cell × each reachable cell).
        schedule: list[Tuple[int, Tuple[float, float, float]]] = []
        for cell_idx, t in reachable:
            schedule.extend([(cell_idx, t)] * throws_per_cell)
        rng = random.Random(seed)
        rng.shuffle(schedule)

        # Persist a metadata JSON for the analysis script.
        session_dir = Path(self.get_parameter('calib_session_dir').value)
        session_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        self._cal_session_path = session_dir / f'accuracy_session_{stamp}.json'
        metadata = {
            'timestamp': stamp,
            'seed': seed,
            'grid_center_mm': list(center),
            'grid_size_mm': list(size),
            'grid_divisions': list(divisions),
            'throws_per_cell': throws_per_cell,
            'n_targets_total': len(targets),
            'n_targets_reachable': len(reachable),
            'n_throws_scheduled': len(schedule),
            'bb_mocap_position_mm': list(self._bb_position_mm),
            'bb_yaw_offset_rad': float(self._bb_yaw_offset_rad),
            'aim_correction_loaded': self._aim_correction_matrix is not None,
            'aim_correction_source': self._aim_correction_source,
            'skipped_targets': [
                {'cell_idx': i, 'target_mm': list(t), 'reason': r}
                for i, t, r in skipped
            ],
            'schedule': [
                {'throw_idx': k, 'cell_idx': ci, 'target_mm': list(t)}
                for k, (ci, t) in enumerate(schedule)
            ],
        }
        with open(self._cal_session_path, 'w') as f:
            json.dump(metadata, f, indent=2)

        self._cal_throws = schedule
        self._cal_idx = 0
        self._cal_active = True
        self._cal_cancelled = False
        self._cal_phase = self._CAL_PHASE_IDLE

        self.get_logger().info(
            f'Accuracy calibration starting: {len(schedule)} throws to '
            f'{len(reachable)}/{len(targets)} reachable cells (seed={seed}). '
            f'Metadata: {self._cal_session_path}'
        )
        if skipped:
            self.get_logger().warning(
                f'{len(skipped)} unreachable targets skipped (see session metadata)'
            )

        self._aim_at_current_calibration_throw()

        response.success = True
        response.message = (
            f'Accuracy calibration started: {len(schedule)} throws. '
            f'Metadata at {self._cal_session_path}'
        )
        return response

    def _svc_cancel_accuracy_calibration(self, request, response):
        if not self._cal_active:
            response.success = False
            response.message = 'No accuracy calibration in progress'
            return response
        self._cal_cancelled = True
        completed = self._cal_idx
        total = len(self._cal_throws)
        self._stop_calibration()
        response.success = True
        response.message = f'Cancelled after {completed}/{total} throws'
        self.get_logger().info(response.message)
        return response

    def _aim_at_current_calibration_throw(self) -> None:
        if self._cal_cancelled or not self._cal_active:
            return
        if self._cal_idx >= len(self._cal_throws):
            self._complete_calibration()
            return

        cell_idx, (x_g, y_g, z_g) = self._cal_throws[self._cal_idx]
        self.get_logger().info(
            f'Calibration throw {self._cal_idx + 1}/{len(self._cal_throws)} '
            f'(cell {cell_idx}): aiming at ({x_g:.0f}, {y_g:.0f}, {z_g:.0f}) mm'
        )

        ok, msg, _sol, _delay = self._throw_at_global_point(
            x_g, y_g, z_g,
            target_id=f'cell_{cell_idx}',
            throw=False,  # aim only; firmware throw happens once position settles
        )
        if not ok:
            self.get_logger().error(
                f'Calibration throw {self._cal_idx + 1} skipped: {msg}'
            )
            # Skip this throw rather than abort the whole session.
            self._cal_idx += 1
            self._aim_at_current_calibration_throw()
            return

        self._cal_phase = self._CAL_PHASE_AWAITING_POSITION
        self._cal_position_reached_t = None
        self._start_calibration_keepalive()

    def _start_calibration_keepalive(self) -> None:
        """Re-send aim commands periodically to dodge firmware tracking timeout."""
        self._stop_calibration_keepalive()
        interval = float(
            self.get_parameter('calib_tracking_keepalive_s').value)
        self._cal_keepalive_timer = self.create_timer(
            interval, self._calibration_keepalive_tick)

    def _stop_calibration_keepalive(self) -> None:
        if self._cal_keepalive_timer is not None:
            self._cal_keepalive_timer.cancel()
            self._cal_keepalive_timer = None

    def _calibration_keepalive_tick(self) -> None:
        if (self._cal_phase != self._CAL_PHASE_AWAITING_POSITION
                or self._cal_cancelled
                or self._cal_idx >= len(self._cal_throws)):
            self._stop_calibration_keepalive()
            return
        cell_idx, (x_g, y_g, z_g) = self._cal_throws[self._cal_idx]
        self._throw_at_global_point(
            x_g, y_g, z_g,
            target_id=f'cell_{cell_idx}',
            throw=False,
        )

    def _execute_calibration_throw(self) -> None:
        if self._cal_cancelled or not self._cal_active:
            return
        self._stop_calibration_keepalive()

        cell_idx, (x_g, y_g, z_g) = self._cal_throws[self._cal_idx]
        delay_s = float(self.get_parameter('calib_throw_delay_s').value)
        ok, msg, _sol, _delay = self._throw_at_global_point(
            x_g, y_g, z_g,
            target_id=f'cell_{cell_idx}',
            throw_delay_s=delay_s,
            throw=True,
        )
        if not ok:
            self.get_logger().error(f'Calibration throw failed: {msg}')
        self._cal_idx += 1
        self._cal_phase = self._CAL_PHASE_WAITING_RELOAD
        self._cal_reload_heartbeats = 0

    def _handle_calibration_heartbeat(self, msg: BallButlerHeartbeat) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9

        if self._cal_phase == self._CAL_PHASE_AWAITING_POSITION:
            # If BB drops to IDLE we lost tracking — re-aim.
            if msg.state == BallButlerStates.IDLE:
                self.get_logger().warning(
                    'BB returned to IDLE while awaiting position; re-aiming'
                )
                self._aim_at_current_calibration_throw()
                return

            yaw_tol = float(self.get_parameter('calib_yaw_tolerance_deg').value)
            pitch_tol = float(self.get_parameter('calib_pitch_tolerance_deg').value)
            reached = (
                abs(msg.yaw_deg - self._last_commanded_yaw_deg) <= yaw_tol
                and abs(msg.pitch_deg - self._last_commanded_pitch_deg) <= pitch_tol
            )
            if reached:
                if self._cal_position_reached_t is None:
                    self._cal_position_reached_t = now
                elif (now - self._cal_position_reached_t
                      >= float(self.get_parameter('calib_settle_time_s').value)):
                    self._execute_calibration_throw()
            else:
                self._cal_position_reached_t = None

        elif self._cal_phase == self._CAL_PHASE_WAITING_RELOAD:
            if msg.state == BallButlerStates.RELOADING:
                self._cal_reload_heartbeats += 1
                return
            min_reloads = int(
                self.get_parameter('calib_min_reload_heartbeats').value)
            if self._cal_reload_heartbeats < min_reloads:
                return
            if msg.state not in (BallButlerStates.RELOADING,
                                 BallButlerStates.THROWING):
                if self._cal_idx >= len(self._cal_throws):
                    self._complete_calibration()
                else:
                    self._aim_at_current_calibration_throw()

    def _complete_calibration(self) -> None:
        self.get_logger().info(
            f'Accuracy calibration COMPLETE: {self._cal_idx} throws done. '
            f'Metadata at {self._cal_session_path}'
        )
        self._stop_calibration()

    def _stop_calibration(self) -> None:
        self._stop_calibration_keepalive()
        self._cal_active = False
        self._cal_phase = self._CAL_PHASE_IDLE
        self._cal_throws = []
        self._cal_idx = 0
        self._cal_position_reached_t = None
        self._cal_reload_heartbeats = 0


def main(args=None):
    rclpy.init(args=args)
    node = BallButlerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
