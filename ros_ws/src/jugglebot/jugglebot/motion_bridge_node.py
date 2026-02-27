"""ROS2 bridge node for the standalone motion control process.

Translates between the ROS2 topic/service world and the ZeroMQ IPC
layer used by the control process.

ROS2 → IPC:
  - PlatformPoseCommand (from spacemouse, catch nodes, etc.) → TargetState
  - control_mode_topic (from orchestrator) → ModeCommand

IPC → ROS2:
  - Telemetry (from control loop) → leg_lengths_topic (18 values:
    6 pos_rev + 6 vel_ff_rps + 6 torque_ff_Nm for CAN node's set_input_pos)
"""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from jugglebot_interfaces.msg import PlatformPoseCommand

from jugglebot.motion.ipc import (
    BridgeIPC,
    make_mode_command,
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

        # Control mode from orchestrator
        self.create_subscription(
            String, 'control_mode_topic',
            self._on_control_mode, 10)

        # ------------------------------------------------------------------
        # ROS2 publishers (IPC → ROS2)
        # ------------------------------------------------------------------

        # Leg commands for the CAN node (18 values: pos + vel_ff + torque_ff)
        self._leg_pub = self.create_publisher(
            Float64MultiArray, 'leg_lengths_topic', 10)

        # Feedforward torques (monitoring/diagnostics only)
        self._torque_pub = self.create_publisher(
            Float64MultiArray, 'leg_torques_diagnostic', 10)

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

    def _on_control_mode(self, msg: String) -> None:
        """Translate control mode changes to IPC mode commands."""
        mode = msg.data
        prev = self._current_control_mode
        self._current_control_mode = mode

        if mode in ('SPACEMOUSE', 'SHELL'):
            self._active_publisher = mode
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

        positions = telem.get('leg_pos', [0.0] * 6)
        velocities = telem.get('leg_vel', [0.0] * 6)
        torques = telem.get('cmd_torques', [0.0] * 6)

        # Publish unified command to CAN node: 6 pos + 6 vel_ff + 6 torque_ff
        leg_msg = Float64MultiArray()
        leg_msg.data = list(positions) + list(velocities) + list(torques)
        self._leg_pub.publish(leg_msg)

        # Publish feedforward torques on diagnostic topic (monitoring only)
        ff_torques = telem.get('ff_torques')
        if ff_torques:
            diag_msg = Float64MultiArray()
            diag_msg.data = ff_torques
            self._torque_pub.publish(diag_msg)

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
