"""ROS2 wrapper for the tracking/ subpackage.

Subscribes to:
  - mocap_data (MocapDataMulti) — unlabelled markers at ~200 Hz
  - throw_announcements (ThrowAnnouncement) — Ball Butler throw events

Publishes:
  - balls (BallStateArray) — all tracked balls at mocap rate
"""

import rclpy
from rclpy.node import Node
import numpy as np

from jugglebot_interfaces.msg import (
    MocapDataMulti,
    ThrowAnnouncement,
    BallStateArray,
    BallState,
)
from geometry_msgs.msg import Point, Vector3
from builtin_interfaces.msg import Time

import jugglebot.hardware_config as hw
from jugglebot.tracking.matcher import BallTracker
from jugglebot.tracking.ball import Ball


class BallTrackerNode(Node):
    def __init__(self):
        super().__init__('ball_tracker_node')

        # Tracking config from hardware_config
        self._landing_z = hw.GEOM_INITIAL_HEIGHT_MM + 160.0  # Default catch plane

        self._tracker = BallTracker(
            dt=hw.TRACKING_MOCAP_DT_S,
            landing_z=self._landing_z,
            match_threshold_base_mm=hw.TRACKING_MATCH_THRESHOLD_BASE_MM,
            parabolic_min_frames=hw.TRACKING_MIN_MATCHES_TO_CONFIRM,
            missed_frames_to_lose=10,
            max_frames_without_measurement=hw.TRACKING_MAX_FRAMES_WITHOUT_MEASUREMENT,
            process_noise=hw.TRACKING_PROCESS_NOISE,
            measurement_noise=hw.TRACKING_MEASUREMENT_NOISE,
            min_height_above_landing_mm=hw.TRACKING_MIN_HEIGHT_ABOVE_LANDING_MM,
        )

        # Subscribers
        self._mocap_sub = self.create_subscription(
            MocapDataMulti, 'mocap_data', self._on_mocap, 10)
        self._announcement_sub = self.create_subscription(
            ThrowAnnouncement, 'throw_announcements', self._on_announcement, 10)

        # Publisher
        self._balls_pub = self.create_publisher(BallStateArray, 'balls', 10)

        self.get_logger().info(
            f"BallTrackerNode ready: landing_z={self._landing_z:.1f}mm, "
            f"dt={hw.TRACKING_MOCAP_DT_S*1000:.1f}ms")

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
            source=msg.thrower_name or "ball_butler",
            destination=destination,
            landing_position=landing_pos if landing_time_s > 0 else None,
            landing_velocity=landing_vel if landing_time_s > 0 else None,
            landing_time=landing_time_s if landing_time_s > 0 else None,
        )

        delay = throw_time - current_time
        self.get_logger().info(
            f"Ball {ball_id} announced by '{msg.thrower_name}', "
            f"throw in {delay:.2f}s")

    def _on_mocap(self, msg: MocapDataMulti):
        """Process mocap frame: extract unlabelled markers, run tracker, publish."""
        current_time = self.get_clock().now().nanoseconds * 1e-9

        # Extract unlabelled marker positions
        markers = []
        for data in msg.markers:
            if not data.label:  # Unlabelled markers only
                markers.append(np.array([
                    data.position.x,
                    data.position.y,
                    data.position.z,
                ]))

        # Run tracker
        balls = self._tracker.process_frame(markers, current_time)

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

        # Convert absolute landing_time to ROS2 Time
        if ball.landing_time > 0:
            msg.time_at_land = Time()
            msg.time_at_land.sec = int(ball.landing_time)
            msg.time_at_land.nanosec = int((ball.landing_time % 1) * 1e9)

        return msg

    def destroy_node(self):
        self.get_logger().info("Shutting down BallTrackerNode.")
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
