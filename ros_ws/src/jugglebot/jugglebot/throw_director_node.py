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
from geometry_msgs.msg import Point, Vector3

from jugglebot_interfaces.msg import (
    BallButlerCalibrationResult,
    RigidBodyPoses,
    ThrowAnnouncement,
)
from jugglebot_interfaces.srv import SendBallButlerCommand, ThrowAtTarget

import jugglebot.hardware_config as hw
from jugglebot.can.throw_ballistics import (
    ThrowSolution,
    global_to_bb_local,
    solve_throw_local,
)


# Default release delay (s) — leaves time for the CAN send + BB to schedule.
# Matches the archived ball_butler_node's `seconds_to_throw_in` default.
_DEFAULT_THROW_DELAY_S = 1.0

# Gravity (mm/s²) — for landing-velocity decay in published ThrowAnnouncement.
_G_MMPS2 = hw.GRAVITY_MPS2 * 1000.0


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

        # ── Publisher: ThrowAnnouncement (own, not can_node's) ──
        # The director sends suppress_announcement=True on bb/send_throw_command
        # so can_node skips its predict_throw-based announcement (which uses
        # the platform default catch height; wrong for off-plane targets like
        # the cone).  We publish here using the solver's actual target z and
        # serial-chain-aware ToF.
        self.throw_announcement_pub = self.create_publisher(
            ThrowAnnouncement, 'throw_announcements', 10)

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
        # Tell can_node not to publish a predict_throw-based announcement
        # (which would use the platform default catch height).  We publish
        # our own below using the solver's actual target z + serial-chain
        # ToF.
        bb_req.suppress_announcement = True

        # Fire-and-forget: we don't block on can_node's response.
        self._throw_client.call_async(bb_req)

        # Publish our own ThrowAnnouncement with the correct landing geometry.
        self._publish_throw_announcement(
            target_name=req.target_name,
            target_global_mm=(x_g, y_g, z_g),
            sol=sol,
            delay_s=delay_s,
        )

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
