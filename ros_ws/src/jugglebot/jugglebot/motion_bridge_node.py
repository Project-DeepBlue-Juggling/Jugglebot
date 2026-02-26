"""ROS2 bridge node for the standalone motion control process.

Translates between the ROS2 topic/service world and the ZeroMQ IPC
layer used by the control process.

ROS2 → IPC:
  - PlatformPoseCommand (from spacemouse, catch nodes, etc.) → TargetState
  - RobotState (from CAN node) → MotorFeedback
  - control_mode_topic (from orchestrator) → ModeCommand

IPC → ROS2:
  - Telemetry (from control loop) → leg_lengths_topic (for CAN node)
"""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from jugglebot_interfaces.msg import PlatformPoseCommand, RobotState

from jugglebot.motion.ipc import (
    BridgeIPC,
    make_mode_command,
    make_motor_feedback,
    make_target_state,
)


class MotionBridgeNode(Node):
    """Bridges ROS2 topics to/from the standalone control process via ZeroMQ."""

    def __init__(self):
        super().__init__('motion_bridge')

        # ------------------------------------------------------------------
        # IPC connection
        # ------------------------------------------------------------------
        self.ipc = BridgeIPC()
        self.get_logger().info("IPC bridge initialised "
                               "(command → tcp://localhost:5555, "
                               "telemetry ← tcp://localhost:5556)")

        # ------------------------------------------------------------------
        # ROS2 subscriptions (ROS2 → IPC)
        # ------------------------------------------------------------------

        # Platform pose commands from spacemouse, catch planner, etc.
        self.create_subscription(
            PlatformPoseCommand, 'platform_pose_topic',
            self._on_pose_command, 10)

        # Robot state from CAN node (motor positions/velocities/currents)
        self.create_subscription(
            RobotState, 'robot_state',
            self._on_robot_state, 10)

        # Control mode from orchestrator
        self.create_subscription(
            String, 'control_mode_topic',
            self._on_control_mode, 10)

        # ------------------------------------------------------------------
        # ROS2 publishers (IPC → ROS2)
        # ------------------------------------------------------------------

        # Leg lengths for the CAN node (position mode)
        self._leg_pub = self.create_publisher(
            Float64MultiArray, 'leg_lengths_topic', 10)

        # Leg torques for the CAN node (torque mode)
        self._torque_pub = self.create_publisher(
            Float64MultiArray, 'leg_torques_topic', 10)

        # ------------------------------------------------------------------
        # Timer to poll IPC telemetry
        # ------------------------------------------------------------------
        self._poll_timer = self.create_timer(0.002, self._poll_telemetry)  # 500 Hz

        # ------------------------------------------------------------------
        # State tracking
        # ------------------------------------------------------------------
        self._current_control_mode = ''
        self._active_publisher = ''  # which source is allowed (matches control_mode)

    # ------------------------------------------------------------------
    # ROS2 → IPC callbacks
    # ------------------------------------------------------------------

    def _on_pose_command(self, msg: PlatformPoseCommand) -> None:
        """Forward platform pose commands to the control process."""
        # Gate: only forward if the publisher matches the active control mode
        publisher = msg.publisher
        if publisher != self._current_control_mode:
            return

        pose = msg.pose_stamped.pose
        pos = [pose.position.x, pose.position.y, pose.position.z]
        quat = [pose.orientation.w, pose.orientation.x,
                pose.orientation.y, pose.orientation.z]

        target = make_target_state(pos=pos, rot_quat=quat)
        self.ipc.send_target(target)

    def _on_robot_state(self, msg: RobotState) -> None:
        """Extract motor feedback from RobotState and forward to control process."""
        if not msg.motor_states:
            return

        # Extract leg motor data (axes 0-5)
        n = min(6, len(msg.motor_states))
        positions = [0.0] * 6
        velocities = [0.0] * 6
        currents = [0.0] * 6

        for i in range(n):
            ms = msg.motor_states[i]
            positions[i] = float(ms.pos_estimate)
            velocities[i] = float(ms.vel_estimate)
            currents[i] = float(ms.iq_measured)

        fb = make_motor_feedback(
            positions=positions,
            velocities=velocities,
            currents=currents,
        )
        self.ipc.send_motor_feedback(fb)

    def _on_control_mode(self, msg: String) -> None:
        """Translate control mode changes to IPC mode commands."""
        mode = msg.data
        prev = self._current_control_mode
        self._current_control_mode = mode

        if mode in ('SPACEMOUSE', 'SHELL',
                   'TORQUE_SPACEMOUSE', 'TORQUE_SHELL'):
            # Strip TORQUE_ prefix for publisher gating
            self._active_publisher = mode.replace('TORQUE_', '')
            if prev != mode:
                cmd = make_mode_command('enable')
                self.ipc.send_mode_command(cmd)
                self.get_logger().info(f"Sent 'enable' to control process "
                                       f"(mode: {mode})")
        elif mode == 'ERROR':
            cmd = make_mode_command('estop')
            self.ipc.send_mode_command(cmd)
            self.get_logger().warning("Sent 'estop' to control process")
        elif mode == '' or mode is None:
            self._active_publisher = ''
            if prev and prev not in ('', 'ERROR'):
                cmd = make_mode_command('disable')
                self.ipc.send_mode_command(cmd)
                self.get_logger().info("Sent 'disable' to control process")

    # ------------------------------------------------------------------
    # IPC → ROS2 polling
    # ------------------------------------------------------------------

    def _poll_telemetry(self) -> None:
        """Poll telemetry from the control process and publish to ROS2."""
        telem = self.ipc.recv_telemetry()
        if telem is None:
            return

        # Publish leg positions to the CAN node (always, for both modes)
        leg_msg = Float64MultiArray()
        leg_msg.data = telem.get('leg_pos', [0.0] * 6)
        self._leg_pub.publish(leg_msg)

        # Publish commanded torques to the CAN node
        torques = telem.get('cmd_torques')
        if torques:
            torque_msg = Float64MultiArray()
            torque_msg.data = torques
            self._torque_pub.publish(torque_msg)

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def on_shutdown(self) -> None:
        """Clean up IPC on node shutdown."""
        # Send disable to control process
        try:
            cmd = make_mode_command('disable')
            self.ipc.send_mode_command(cmd)
        except Exception:
            pass
        self.ipc.close()
        self.get_logger().info("IPC bridge closed")


def main(args=None):
    rclpy.init(args=args)
    node = MotionBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
