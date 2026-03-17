"""ROS2 wrapper for the catch coordinator.

Subscribes to:
  - balls (BallStateArray) — tracked balls from ball_tracker_node

Publishes:
  - catch/dynamic_target (DynamicTargetCommand) — forwarded by
    motion_bridge_node to the control process via IPC

Receives accept/reject feedback from the control process via IPC SUB
on the telemetry address (TOPIC_DYN_FEEDBACK).

Clock domain conversion: ROS2 landing_time → perf_counter arrival_time.
"""
from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
import numpy as np

from jugglebot_interfaces.msg import BallStateArray, DynamicTargetCommand
from geometry_msgs.msg import Point, Quaternion, Vector3

import jugglebot.hardware_config as hw
from jugglebot.tracking.ball import Ball, BallStatus, TrackingConfidence
from jugglebot.catch_coordinator import CatchCoordinator


class CatchCoordinatorNode(Node):
    def __init__(self):
        super().__init__('catch_coordinator_node')

        # Clock offset: perf_counter - ros2_time (re-measured periodically)
        self._ros_to_perf_offset = self._measure_clock_offset()
        self._clock_offset_history: list[float] = [self._ros_to_perf_offset]

        # Coordinator (pure Python policy)
        self._coordinator = CatchCoordinator(
            robot_name="jugglebot",
            initial_height_mm=hw.GEOM_INITIAL_HEIGHT_MM,
            landing_z_offset_mm=160.0,
            catch_angle_limit_deg=30.0,
        )

        # Publisher: dynamic target → motion_bridge_node → IPC → control loop
        self._dyn_target_pub = self.create_publisher(
            DynamicTargetCommand, 'catch/dynamic_target', 10)

        # Subscriber: tracked balls
        self._balls_sub = self.create_subscription(
            BallStateArray, 'balls', self._on_balls, 10)

        # IPC SUB for feedback (connects to control process telemetry PUB)
        self._feedback_ipc = _FeedbackIPC()

        # Poll for IPC feedback at 50 Hz
        self._feedback_timer = self.create_timer(0.02, self._poll_feedback)

        # Re-measure clock offset every 30s to track drift
        self._clock_timer = self.create_timer(30.0, self._refresh_clock_offset)

        # Track which ball we last submitted a target for
        self._last_submitted_ball_id: int | None = None
        self._last_arrival_time: float = 0.0

        self.get_logger().info(
            f"CatchCoordinatorNode ready: "
            f"ros_to_perf_offset={self._ros_to_perf_offset:.6f}s")

    def _measure_clock_offset(self) -> float:
        """Measure offset between perf_counter and ROS2 wall clock."""
        offsets = []
        for _ in range(10):
            t_perf = time.perf_counter()
            t_ros = self.get_clock().now().nanoseconds / 1e9
            offsets.append(t_perf - t_ros)
        return float(np.median(offsets))

    def _refresh_clock_offset(self):
        """Periodically re-measure clock offset to track drift."""
        new_offset = self._measure_clock_offset()
        self._clock_offset_history.append(new_offset)
        # Keep last 20 measurements (10 minutes at 30s interval)
        if len(self._clock_offset_history) > 20:
            self._clock_offset_history.pop(0)
        self._ros_to_perf_offset = float(np.median(self._clock_offset_history))

    def _on_balls(self, msg: BallStateArray):
        """Process ball state updates: select best ball and send dynamic target."""
        current_time = self.get_clock().now().nanoseconds / 1e9

        # Convert ROS2 messages to Ball objects for the coordinator
        balls = [self._msg_to_ball(b) for b in msg.balls]

        # Run coordinator policy
        cmd = self._coordinator.update(balls, current_time)
        if cmd is None:
            return

        # Convert landing_time from ROS2 clock → perf_counter clock
        arrival_time_perf = cmd.landing_time + self._ros_to_perf_offset

        # Publish typed message for bridge to forward via IPC
        out = DynamicTargetCommand()
        out.target_pos = Point(
            x=float(cmd.target_pos[0]),
            y=float(cmd.target_pos[1]),
            z=float(cmd.target_pos[2]),
        )
        out.target_quat = Quaternion(
            w=float(cmd.target_quat[0]),
            x=float(cmd.target_quat[1]),
            y=float(cmd.target_quat[2]),
            z=float(cmd.target_quat[3]),
        )
        out.target_vel = Vector3(
            x=float(cmd.target_vel[0]),
            y=float(cmd.target_vel[1]),
            z=float(cmd.target_vel[2]),
        )
        out.arrival_time = arrival_time_perf
        self._dyn_target_pub.publish(out)

        self._last_submitted_ball_id = cmd.ball_id
        self._last_arrival_time = arrival_time_perf

    def _poll_feedback(self):
        """Check for accept/reject feedback from the motion planner."""
        fb = self._feedback_ipc.recv()
        if fb is None:
            return

        ball_id = self._last_submitted_ball_id
        if ball_id is None:
            return

        # Correlate by arrival_time (approximate match)
        fb_arrival = fb.get('arrival_time', 0.0)
        if abs(fb_arrival - self._last_arrival_time) > 0.1:
            return  # Stale feedback, ignore

        if fb.get('accepted', False):
            self._coordinator.report_acceptance(ball_id)
            self.get_logger().debug(f"Ball {ball_id}: target accepted")
        else:
            violations = fb.get('violations', [])
            self._coordinator.report_rejection(ball_id)
            self.get_logger().info(
                f"Ball {ball_id}: target rejected — {', '.join(violations)}")

    @staticmethod
    def _msg_to_ball(msg) -> Ball:
        """Convert a ROS2 BallState message to an internal Ball object."""
        landing_time = msg.time_at_land.sec + msg.time_at_land.nanosec * 1e-9

        return Ball(
            id=msg.id,
            status=BallStatus(msg.status),
            tracking=TrackingConfidence(msg.tracking),
            source=msg.source,
            destination=msg.destination,
            position=np.array([msg.position.x, msg.position.y, msg.position.z]),
            velocity=np.array([msg.velocity.x, msg.velocity.y, msg.velocity.z]),
            landing_position=np.array([
                msg.landing_position.x, msg.landing_position.y, msg.landing_position.z,
            ]),
            landing_velocity=np.array([
                msg.landing_velocity.x, msg.landing_velocity.y, msg.landing_velocity.z,
            ]),
            landing_time=landing_time,
        )

    def destroy_node(self):
        self.get_logger().info("Shutting down CatchCoordinatorNode.")
        self._feedback_ipc.close()
        super().destroy_node()


class _FeedbackIPC:
    """Lightweight IPC receiver for dynamic target accept/reject feedback.

    Connects a SUB socket to the control process's telemetry PUB address.
    Only subscribes to TOPIC_DYN_FEEDBACK — does not interfere with the
    bridge's telemetry subscription.
    """

    def __init__(self):
        import zmq
        from jugglebot.motion.ipc import TELEMETRY_ADDR, TOPIC_DYN_FEEDBACK, _unpack
        self._unpack = _unpack
        self._zmq = zmq

        self._ctx = zmq.Context()
        self._sub = self._ctx.socket(zmq.SUB)
        self._sub.connect(TELEMETRY_ADDR)
        self._sub.setsockopt(zmq.SUBSCRIBE, TOPIC_DYN_FEEDBACK)
        self._sub.setsockopt(zmq.RCVTIMEO, 0)
        self._sub.setsockopt(zmq.RCVHWM, 64)

    def recv(self) -> dict | None:
        try:
            frames = self._sub.recv_multipart(flags=self._zmq.NOBLOCK)
            _, msg = self._unpack(frames)
            return msg
        except self._zmq.Again:
            return None

    def close(self):
        self._sub.close()
        self._ctx.term()


def main(args=None):
    rclpy.init(args=args)
    node = CatchCoordinatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
