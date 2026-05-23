"""ROS2 node: aim-and-throw Ball Butler at a named QTM rigid body.

Exposes a single service ``bb/throw_at_target`` that:

  1. Looks up the requested target by name in the most recent
     ``/rigid_body_poses`` message (cached locally).
  2. Transforms the target into BB's local frame using the latched
     ``/bb/calibration_result`` (BB position + yaw offset).
  3. Solves inverse-ballistics for ``(yaw, pitch, speed, tof)`` via
     ``throw_ballistics.solve_throw_local``.  Aborts with ``success=false``
     and a descriptive message if no feasible solution exists or the
     target is out of range.
  4. Calls the existing ``bb/send_throw_command`` service on can_node,
     which encodes + sends the CAN throw frame *and* publishes a
     ``ThrowAnnouncement`` for downstream correlation (e.g. the catching
     cone).

This node is intentionally thin: it owns the target-resolution +
solver-invocation glue and nothing else.  CAN transport and throw
announcement publishing remain on can_node.

Time-signal note: all timing flows through can_node's existing pipeline
(host ROS clock = CLOCK_REALTIME = Unix epoch; BB Teensy clock is TimeSync-
locked to the same epoch via bus.broadcast_time).  This director does not
introduce any new clocks.
"""
from __future__ import annotations

import math
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from geometry_msgs.msg import Point

from jugglebot_interfaces.msg import BallButlerCalibrationResult, RigidBodyPoses
from jugglebot_interfaces.srv import SendBallButlerCommand, ThrowAtTarget

from jugglebot.can.throw_ballistics import (
    ThrowSolution,
    global_to_bb_local,
    solve_throw_local,
)


# Default release delay (s) — leaves time for the CAN send + BB to schedule.
# Matches the archived ball_butler_node's `seconds_to_throw_in` default.
_DEFAULT_THROW_DELAY_S = 1.0


class ThrowDirectorNode(Node):
    """Aim Ball Butler at a named QTM rigid body and throw."""

    def __init__(self):
        super().__init__('throw_director_node')

        # Latest cache of rigid-body poses from mocap_node, keyed by name.
        # Stored as world-frame mm tuples (x, y, z).
        self._target_positions_mm: Dict[str, Tuple[float, float, float]] = {}

        # BB calibration (latched).  None until first /bb/calibration_result.
        self._bb_position_mm: Optional[Tuple[float, float, float]] = None
        self._bb_yaw_offset_rad: float = 0.0

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

        # ── Service client (call bb/send_throw_command on can_node) ──
        self._throw_client = self.create_client(
            SendBallButlerCommand, 'bb/send_throw_command')

        # ── Service server (what the GUI calls) ─────────────────
        self.create_service(ThrowAtTarget, 'bb/throw_at_target', self._svc_throw_at_target)

        self.get_logger().info('Throw director ready (bb/throw_at_target).')

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

    # ─────────────────────────────────────────────────────────────────
    # Service handler
    # ─────────────────────────────────────────────────────────────────

    def _svc_throw_at_target(self, req: ThrowAtTarget.Request,
                             res: ThrowAtTarget.Response) -> ThrowAtTarget.Response:
        # Stage 1: BB calibration must be present.
        if self._bb_position_mm is None:
            res.success = False
            res.message = ('BB calibration not yet received — calibrate from '
                           'the GUI first.')
            return res

        # Stage 2: target must be visible in QTM.
        target = self._target_positions_mm.get(req.target_name)
        if target is None:
            known = sorted(self._target_positions_mm.keys()) or ['<none>']
            res.success = False
            res.message = (
                f"Target '{req.target_name}' not in latest rigid_body_poses. "
                f"Known: {', '.join(known)}.")
            return res

        x_g, y_g, z_g = target
        res.target_position_global_mm = Point(x=x_g, y=y_g, z=z_g)

        # Stage 3: world → BB local
        x_l, y_l, z_l = global_to_bb_local(
            x_g, y_g, z_g,
            bb_position_mm=self._bb_position_mm,
            yaw_offset_rad=self._bb_yaw_offset_rad,
        )
        res.target_position_bb_local_mm = Point(x=x_l, y=y_l, z=z_l)

        # Stage 4: inverse ballistics
        try:
            sol: ThrowSolution = solve_throw_local(x_l, y_l, z_l)
        except ValueError as e:
            res.success = False
            res.message = f'Inverse-ballistics failed: {e}'
            return res

        # Stage 5: send throw command to BB via can_node.  We use a default
        # release delay if the caller didn't specify one (or specified 0).
        delay_s = float(req.throw_delay_s) if req.throw_delay_s > 0 else _DEFAULT_THROW_DELAY_S

        if not self._throw_client.service_is_ready():
            res.success = False
            res.message = 'bb/send_throw_command service not available (can_node down?).'
            return res

        bb_req = SendBallButlerCommand.Request()
        bb_req.yaw_angle_rad = float(sol.yaw_rad)
        bb_req.pitch_angle_rad = float(sol.pitch_rad)
        bb_req.throw_speed = float(sol.speed_mps)
        bb_req.throw_time = float(delay_s)

        # Fire-and-forget: we don't block on can_node's response.  can_node
        # publishes the ThrowAnnouncement synchronously in its own service
        # handler before returning, so by the time it returns, the
        # downstream cone correlation already has the announcement.
        self._throw_client.call_async(bb_req)

        # Fill response.
        res.success = True
        res.message = (
            f'Throwing at {req.target_name}: yaw={math.degrees(sol.yaw_rad):.1f}°, '
            f'pitch={math.degrees(sol.pitch_rad):.1f}°, '
            f'speed={sol.speed_mps:.2f} m/s, delay={delay_s:.2f} s, '
            f'predicted ToF={sol.tof_s:.3f} s.')
        res.yaw_rad = float(sol.yaw_rad)
        res.pitch_rad = float(sol.pitch_rad)
        res.throw_speed_mps = float(sol.speed_mps)
        res.throw_delay_s = float(delay_s)
        res.predicted_tof_s = float(sol.tof_s)

        self.get_logger().info(res.message)
        return res


def main(args=None):
    rclpy.init(args=args)
    node = ThrowDirectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
