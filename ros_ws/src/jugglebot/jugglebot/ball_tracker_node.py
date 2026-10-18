"""ROS2 wrapper for the tracking/ subpackage.

Subscribes to:
  - mocap_data (MocapDataMulti) — ALL mocap markers at ~200 Hz
    (labelled and unlabelled; the matcher excludes the robot's own bodies)
  - throw_announcements (ThrowAnnouncement) — Ball Butler throw events

Publishes:
  - balls (BallStateArray) — all tracked balls at mocap rate
"""

import rclpy
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
import numpy as np

from rcl_interfaces.msg import SetParametersResult

from jugglebot_interfaces.msg import (
    MocapDataMulti,
    ThrowAnnouncement,
    BallStateArray,
    BallState,
)
from geometry_msgs.msg import Point, PoseStamped, Vector3
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from builtin_interfaces.msg import Time

import jugglebot.hardware_config as hw
from jugglebot.motion.skills import sites
from jugglebot.tracking.matcher import BallTracker, parse_label_prefixes
from jugglebot.tracking.ball import Ball
from jugglebot.ball_possession import announced_source, require_identity_fields
from jugglebot.robot_frame import RobotFrame


class BallTrackerNode(Node):
    def __init__(self):
        super().__init__('ball_tracker_node')
        # Every published track carries its announcement's (source,
        # throw_time) — the two-ball association key. An un-rebuilt
        # jugglebot_interfaces would make `_ball_to_msg` raise on the first
        # publish; refuse here, once, with the fix in the message.
        require_identity_fields(BallState)
        # Detail lines (per-throw announcements, config dump) log at DEBUG; no
        # .debug( call here is per-frame.
        self.get_logger().set_level(LoggingSeverity.DEBUG)

        # The catch plane the tracker predicts landings AT — the skill stack's
        # ONE definition (`motion.skills.sites.CATCH_CUP_Z_MM`, plan §
        # "Owner decisions" 2026-09-13), imported rather than restated so this
        # node and every skill terminal aimed at a CATCH agree on the same
        # number by construction. Was the hand-computed
        # `GEOM_INITIAL_HEIGHT_MM + JB_OP_DEFAULT_ACTIVE_Z_MM +
        # HAND_CATCH_OFFSET_MM` (809.08 mm at the time; 812.98 mm since the
        # 2026-09-27 kinematic calibration) until this change — the FSM's own
        # catch plane moves with it (accepted by the owner; R4 re-points the
        # coordinator's copies at this same definition, plan § "Key
        # architectural boundaries").
        #
        # Phase 3 (bb-calibration-one-attempt): that number is a ROBOT-FRAME
        # height. The tracker itself stays in the mocap (world) frame -- gravity
        # is along world -z there -- so the plane it is handed is the WORLD z of
        # the robot-frame point (0, 0, z_robot), from the latched `robot_frame`
        # topic (see `_on_robot_frame`). Declared parameter `landing_z_mm` (the
        # robot-frame z) overrides the config default.
        self.declare_parameter('landing_z_mm', float(sites.CATCH_CUP_Z_MM))
        self._landing_z_robot = float(
            self.get_parameter('landing_z_mm').value)
        # Before any `robot_frame` arrives the plane is the robot-frame number
        # itself (identity transform) -- today's behaviour.
        self._landing_z = self._landing_z_robot
        self._robot_frame = None

        # R5 (brief_S.md step 2, 2026-09-30): a declared node parameter, not
        # just the config default baked into the constructor -- the R5 lob
        # block (an un-announced human throw feeding two-ball columns) needs
        # the runsheet to turn this on for one block without a relaunch
        # (`ros2 param set .../ball_tracker_node detect_human_throws true`),
        # and off again after. Config default unchanged (`hw.
        # TRACKING_DETECT_HUMAN_THROWS`, False) -- an un-set launch tracks
        # exactly as before this parameter existed.
        self.declare_parameter('detect_human_throws',
                               bool(hw.TRACKING_DETECT_HUMAN_THROWS))
        detect_human_throws = bool(
            self.get_parameter('detect_human_throws').value)

        self._tracker = BallTracker(
            dt=hw.TRACKING_MOCAP_DT_S,
            landing_z=self._landing_z,
            match_threshold_base_mm=hw.TRACKING_MATCH_THRESHOLD_BASE_MM,
            parabolic_min_frames=hw.TRACKING_MIN_MATCHES_TO_CONFIRM,
            # 15 frames ~ 50 ms: mocap_node publishes every QTM frame (~300/s)
            # since 2026-10-10 (logbook 2026-10-10-mocap-frame-queue); 10 frames
            # meant ~50 ms at the old ~200 msgs/s and would now be ~33 ms.
            missed_frames_to_lose=15,
            max_frames_without_measurement=hw.TRACKING_MAX_FRAMES_WITHOUT_MEASUREMENT,
            process_noise=hw.TRACKING_PROCESS_NOISE,
            measurement_noise=hw.TRACKING_MEASUREMENT_NOISE,
            announced_gate_mm=hw.TRACKING_ANNOUNCED_GATE_MM,
            excluded_label_prefixes=parse_label_prefixes(
                hw.TRACKING_EXCLUDED_LABEL_PREFIXES),
            detect_human_throws=detect_human_throws,
            flight_fit_min_samples=hw.TRACKING_FLIGHT_FIT_MIN_SAMPLES,
            flight_fit_residual_mm=hw.TRACKING_FLIGHT_FIT_RESIDUAL_MM,
            flight_fit_freeze_above_plane_mm=hw.TRACKING_FLIGHT_FIT_FREEZE_ABOVE_PLANE_MM,
            # One DEBUG line per announced-ball confirmation / demotion
            # (matcher, 2026-10-14): python-logging DEBUG does not reach
            # launch.log under rclpy, the node's own logger does.
            debug_log=self.get_logger().debug,
        )
        # Live-updatable (mirrors `teensy_bridge_node`'s
        # `hand_torque_ff_gain` pattern, `_on_set_parameters`): the tracker's
        # own flag is a plain mutable attribute (`BallTracker.
        # detect_human_throws`), so a later `ros2 param set` takes effect on
        # the next mocap frame with no node restart.
        self.add_on_set_parameters_callback(self._on_set_parameters)

        # Subscribers
        self._mocap_sub = self.create_subscription(
            MocapDataMulti, 'mocap_data', self._on_mocap, 10)
        self._announcement_sub = self.create_subscription(
            ThrowAnnouncement, 'throw_announcements', self._on_announcement, 10)
        # mocap_node's latched robot-frame fit (TRANSIENT_LOCAL depth 1).
        self._robot_frame_sub = self.create_subscription(
            PoseStamped, 'robot_frame', self._on_robot_frame,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.get_logger().debug(
            "landing plane: no robot_frame yet, using robot z "
            f"{self._landing_z_robot:.1f} mm as world z (identity)")

        # Publisher
        self._balls_pub = self.create_publisher(BallStateArray, 'balls', 10)

        # Frame-stamp source tracking (2026-09-20, see _on_mocap): each of
        # these two transitions logs exactly ONCE, ever — not once per
        # occurrence — so a flapping clock sync doesn't spam the log.
        self._mocap_stamp_announced = False  # logged "stamped at the source"
        self._mocap_fallback_announced = False  # logged the fall-back warning
        self._mocap_had_stamps = False  # currently/ever in the stamped regime

        self.get_logger().info(
            f"Ball tracker up (landing plane z={self._landing_z:.0f} mm, "
            f"detect_human_throws={detect_human_throws})")
        self.get_logger().debug(
            f"BallTrackerNode ready: landing_z={self._landing_z:.1f}mm, "
            f"dt={hw.TRACKING_MOCAP_DT_S*1000:.1f}ms, "
            f"announced_gate={hw.TRACKING_ANNOUNCED_GATE_MM:.0f}mm, "
            f"excluded_labels={parse_label_prefixes(hw.TRACKING_EXCLUDED_LABEL_PREFIXES)}, "
            f"detect_human_throws={bool(hw.TRACKING_DETECT_HUMAN_THROWS)}, "
            f"flight_fit(min_samples={hw.TRACKING_FLIGHT_FIT_MIN_SAMPLES}, "
            f"residual={hw.TRACKING_FLIGHT_FIT_RESIDUAL_MM:.0f}mm, "
            f"freeze_above_plane={hw.TRACKING_FLIGHT_FIT_FREEZE_ABOVE_PLANE_MM:.0f}mm)")

    def _on_announcement(self, msg: ThrowAnnouncement):
        """Handle throw announcement from Ball Butler."""
        # Extract throw time
        throw_time = msg.throw_time.sec + msg.throw_time.nanosec * 1e-9
        current_time = self.get_clock().now().nanoseconds * 1e-9
        if throw_time < 1.0:
            throw_time = current_time  # Immediate throw

        initial_position = np.array([
            msg.initial_position.x,
            msg.initial_position.y,
            msg.initial_position.z,
        ])
        initial_velocity = np.array([
            msg.initial_velocity.x,
            msg.initial_velocity.y,
            msg.initial_velocity.z,
        ])

        # Pre-computed landing state from announcement
        landing_pos = np.array([
            msg.landing_position.x,
            msg.landing_position.y,
            msg.landing_position.z,
        ])
        landing_vel = np.array([
            msg.landing_velocity.x,
            msg.landing_velocity.y,
            msg.landing_velocity.z,
        ])
        landing_time_s = msg.landing_time.sec + msg.landing_time.nanosec * 1e-9

        # Destination comes from the announcement's target_id field.
        # Ball Butler sets this when throwing to a specific robot.
        # Empty target_id = unassigned (won't be caught).
        destination = msg.target_id if msg.target_id else ""

        ball_id = self._tracker.handle_announcement(
            initial_position=initial_position,
            initial_velocity=initial_velocity,
            throw_time=throw_time,
            source=announced_source(msg.thrower_name),
            destination=destination,
            landing_position=landing_pos if landing_time_s > 0 else None,
            landing_velocity=landing_vel if landing_time_s > 0 else None,
            landing_time=landing_time_s if landing_time_s > 0 else None,
        )

        delay = throw_time - current_time
        self.get_logger().debug(
            f"Ball {ball_id} announced by '{msg.thrower_name}', "
            f"throw in {delay:.2f}s")

    def _on_mocap(self, msg: MocapDataMulti):
        """Process mocap frame: forward EVERY marker to the tracker, publish.

        Every marker in the frame goes through — labelled and unlabelled alike
        — with its label alongside, and the matcher drops the ones belonging to
        the robot's own rigid bodies (`BallTracker.eligible_markers`).

        This node used to forward only markers with an EMPTY label. That
        silently blinded the tracker whenever the mocap system labelled the
        ball, which it does routinely: on 2026-09-15 QTM's AIM model claimed
        the flying ball as `Ball Butler - 1` on all 13 self-tosses (rising
        717 -> 1491 mm on armA-050, 716 -> 2002 mm on armA-090), so ZERO balls
        were ever CONFIRMED and every catch ended `NO_LANDING`. Which label
        QTM picks is a property of its model file, not of the ball, so the
        tracker must not depend on it.

        The frame is stamped at CALLBACK time only as a fallback. Measured
        2026-09-20 on bag ~/Desktop/rosbags/2026-09-18_16-16-17: under
        sitting load this callback runs late and unevenly, jittering the
        ballistic fit's sample times enough (10 ms -> 30 mm at 3 m/s, past
        the fit's 12 mm residual gate) that the fit never converges — 34/60
        throws in that bag left the learner no row. `msg.stamp` carries the
        QTM frame time through `MocapInterface`'s QTM<->ROS offset
        instead, so the fit sees the time the frame was actually captured
        at, not when this callback happened to run.
        """
        stamp_ns = msg.stamp.sec * 1_000_000_000 + msg.stamp.nanosec
        if stamp_ns != 0:
            current_time = stamp_ns * 1e-9
            if not self._mocap_stamp_announced:
                self._mocap_stamp_announced = True
                self.get_logger().debug("mocap frames stamped at the source")
            self._mocap_had_stamps = True
        else:
            current_time = self.get_clock().now().nanoseconds * 1e-9
            if self._mocap_had_stamps and not self._mocap_fallback_announced:
                self._mocap_fallback_announced = True
                self.get_logger().warning(
                    "mocap frame stamp fell back to callback time "
                    "(QTM clock sync not established)")

        markers = []
        labels = []
        for data in msg.markers:
            markers.append(np.array([
                data.position.x,
                data.position.y,
                data.position.z,
            ]))
            labels.append(data.label or "")

        # Run tracker
        balls = self._tracker.process_frame(markers, current_time, labels)

        # Publish all active balls
        if balls:
            out = BallStateArray()
            for ball in balls:
                ball_msg = self._ball_to_msg(ball)
                if ball_msg is not None:
                    out.balls.append(ball_msg)
            if out.balls:
                self._balls_pub.publish(out)

    def _ball_to_msg(self, ball: Ball) -> BallState:
        """Convert internal Ball to ROS2 BallState message."""
        msg = BallState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base"
        msg.id = ball.id
        msg.status = int(ball.status)
        msg.tracking = int(ball.tracking)
        msg.source = ball.source
        msg.destination = ball.destination

        msg.position = Point(
            x=float(ball.position[0]),
            y=float(ball.position[1]),
            z=float(ball.position[2]),
        )
        msg.velocity = Vector3(
            x=float(ball.velocity[0]),
            y=float(ball.velocity[1]),
            z=float(ball.velocity[2]),
        )

        msg.landing_position = Point(
            x=float(ball.landing_position[0]),
            y=float(ball.landing_position[1]),
            z=float(ball.landing_position[2]),
        )
        msg.landing_velocity = Vector3(
            x=float(ball.landing_velocity[0]),
            y=float(ball.landing_velocity[1]),
            z=float(ball.landing_velocity[2]),
        )

        msg.landing_from_fit = bool(ball.landing_from_fit)

        # The announcement's own throw_time, verbatim: with ``source`` it is
        # the identity key every consumer correlates a release on
        # (`ball_possession.match_announced_track`). Zero (left default) for
        # an un-announced (parabolic) track, which has no identity.
        if ball.throw_time > 0:
            msg.throw_time = Time()
            msg.throw_time.sec = int(ball.throw_time)
            msg.throw_time.nanosec = int((ball.throw_time % 1) * 1e9)

        # Convert absolute landing_time to ROS2 Time
        if ball.landing_time > 0:
            msg.time_at_land = Time()
            msg.time_at_land.sec = int(ball.landing_time)
            msg.time_at_land.nanosec = int((ball.landing_time % 1) * 1e9)

        return msg

    def _on_robot_frame(self, msg: PoseStamped):
        """Re-derive the tracker's landing plane from the robot frame.

        The plane is a horizontal WORLD plane through the robot-frame point
        (0, 0, z_robot): ``(R @ (0,0,z) + t)[2]``. A real plane through the
        robot frame is tilted by the (sub-degree) rotation of R; across the
        +-125 mm columns sites a 1 degree tilt moves the true plane by
        ~2 mm, well inside the 20 mm landing threshold, so a horizontal plane
        is kept (the tracker's ballistics are z-only). Identity -> bit-for-bit
        today's number. One INFO per change over 0.5 mm, never per message.
        """
        p, q = msg.pose.position, msg.pose.orientation
        try:
            frame = RobotFrame.from_pose(
                (p.x, p.y, p.z), (q.x, q.y, q.z, q.w))
        except ValueError as exc:
            self.get_logger().warning(
                f"robot_frame ignored (bad pose: {exc}); landing plane "
                f"stays at world z {self._landing_z:.1f} mm")
            return
        self._robot_frame = frame
        self._apply_landing_plane()

    def _apply_landing_plane(self):
        frame = self._robot_frame
        if frame is None:
            z_world = self._landing_z_robot
        else:
            z_world = float(frame.to_world((0.0, 0.0, self._landing_z_robot))[2])
        if not np.isfinite(z_world):
            return
        old = self._landing_z
        self._landing_z = z_world
        self._tracker.landing_z = z_world
        if frame is not None and abs(z_world - old) > 0.5:
            yaw = float(np.degrees(np.arctan2(frame.R[1, 0], frame.R[0, 0])))
            self.get_logger().info(
                f"landing plane: robot z {self._landing_z_robot:.1f} mm -> "
                f"world z {z_world:.1f} mm (robot frame origin "
                f"({frame.t[0]:.1f}, {frame.t[1]:.1f}, {frame.t[2]:.1f}) mm, "
                f"yaw {yaw:.2f} deg)")

    def _on_set_parameters(self, params):
        """ROS2 param-set live-update hook (R5, mirrors
        `teensy_bridge_node._on_set_parameters`'s pattern): only
        ``detect_human_throws`` is handled here; every other declared
        parameter on this node passes through unchanged. Pushes straight
        into the tracker's own mutable flag (`BallTracker.
        detect_human_throws`), so it takes effect on the very next mocap
        frame -- no node restart needed for the runsheet to turn the lob
        block on and off. One INFO line per change (operator-visible
        event), never per frame.
        """
        for param in params:
            if param.name == 'landing_z_mm':
                self._landing_z_robot = float(param.value)
                self._apply_landing_plane()
                continue
            if param.name != 'detect_human_throws':
                continue
            value = bool(param.value)
            if value != self._tracker.detect_human_throws:
                self.get_logger().info(
                    f"detect_human_throws -> {value} (ros2 param set)")
            self._tracker.detect_human_throws = value
        return SetParametersResult(successful=True)

    def destroy_node(self):
        self.get_logger().debug("Shutting down BallTrackerNode.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BallTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
