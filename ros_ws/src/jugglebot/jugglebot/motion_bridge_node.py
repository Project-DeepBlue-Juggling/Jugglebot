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
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from std_msgs.msg import Float64MultiArray, String
from jugglebot_interfaces.msg import PlatformPoseCommand, RobotState

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
                               "(command → tcp://127.0.0.1:5555, "
                               "telemetry ← tcp://127.0.0.1:5556)")

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

        # Gravity offset from orchestrator (levelling result)
        self.create_subscription(
            Float64MultiArray, 'gravity_offset',
            self._on_gravity_offset, 10)

        # Robot state from CAN node (motor feedback for control loop)
        self.create_subscription(
            RobotState, 'robot_state',
            self._on_robot_state, 10)

        # ------------------------------------------------------------------
        # ROS2 publishers (IPC → ROS2)
        # ------------------------------------------------------------------

        # Leg commands for the CAN node (18 values: pos + vel_ff + torque_ff)
        self._leg_pub = self.create_publisher(
            Float64MultiArray, 'leg_lengths_topic', 10)

        # Feedforward torques (monitoring/diagnostics only)
        self._torque_pub = self.create_publisher(
            Float64MultiArray, 'leg_torques_diagnostic', 10)

        # Per-leg tracking errors (mm)
        self._tracking_error_pub = self.create_publisher(
            Float64MultiArray, 'motion/tracking_error', 10)

        # Diagnostics: condition number, workspace status, workspace speed scale
        self._diagnostics_pub = self.create_publisher(
            DiagnosticStatus, 'motion/diagnostics', 10)

        # ------------------------------------------------------------------
        # Timer to poll IPC telemetry
        # ------------------------------------------------------------------
        self._poll_timer = self.create_timer(0.002, self._poll_telemetry)  # 500 Hz

        # ------------------------------------------------------------------
        # State tracking
        # ------------------------------------------------------------------
        self._current_control_mode = ''
        self._active_publisher = ''  # which source is allowed (matches control_mode)
        self._control_loop_enabled = False  # gate for leg command publishing

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

        if mode in ('SPACEMOUSE', 'SHELL', 'GUI'):
            self._active_publisher = mode
            if prev != mode:
                cmd = make_mode_command('enable')
                self.ipc.send_mode_command(cmd)
                self._control_loop_enabled = True
                self.get_logger().info(f"Sent 'enable' to control process "
                                       f"(mode: {mode})")
        elif mode == 'LEVELLING':
            # LEVELLING uses the CAN node's profiled gentle-move commands,
            # not the motion planner.  Do NOT enable the control loop.
            self._active_publisher = mode
            if prev in ('SPACEMOUSE', 'SHELL', 'GUI'):
                cmd = make_mode_command('disable')
                self.ipc.send_mode_command(cmd)
                self._control_loop_enabled = False
                self.get_logger().info(
                    "Disabled control loop for LEVELLING mode")
            self.get_logger().info(
                "LEVELLING mode — control loop stays disabled")
        elif mode == 'ERROR':
            cmd = make_mode_command('estop')
            self.ipc.send_mode_command(cmd)
            self._control_loop_enabled = False
            self.get_logger().warning("Sent 'estop' to control process")
        elif mode == '' or mode is None:
            self._active_publisher = ''
            if prev and prev not in ('', 'ERROR'):
                cmd = make_mode_command('disable')
                self.ipc.send_mode_command(cmd)
                self._control_loop_enabled = False
                self.get_logger().info("Sent 'disable' to control process")

    def _on_gravity_offset(self, msg: Float64MultiArray) -> None:
        """Forward gravity offset to the control process via IPC."""
        cmd = make_mode_command(
            'set_gravity_offset', tilt_x=msg.data[0], tilt_y=msg.data[1])
        self.ipc.send_mode_command(cmd)
        self.get_logger().info(
            f"Sent gravity offset to control process: "
            f"[{msg.data[0]:.4f}, {msg.data[1]:.4f}] rad")

    def _on_robot_state(self, msg: RobotState) -> None:
        """Forward motor feedback from CAN node to the control process."""
        states = msg.motor_states
        if len(states) < 6:
            return
        # First 6 entries are the leg motors
        fb = {
            'pos': [states[i].pos_estimate for i in range(6)],
            'vel': [states[i].vel_estimate for i in range(6)],
            'cur': [states[i].iq_measured for i in range(6)],
        }
        self.ipc.send_motor_feedback(fb)

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
        # Only when control loop is enabled — forwarding commands from a
        # disabled loop would send stale zeros to the motors.
        if self._control_loop_enabled:
            leg_msg = Float64MultiArray()
            leg_msg.data = list(positions) + list(velocities) + list(torques)
            self._leg_pub.publish(leg_msg)

        # Publish feedforward torques on diagnostic topic (monitoring only)
        ff_torques = telem.get('ff_torques')
        if ff_torques:
            diag_msg = Float64MultiArray()
            diag_msg.data = ff_torques
            self._torque_pub.publish(diag_msg)

        # Publish per-leg tracking errors (mm)
        tracking_error = telem.get('tracking_error_mm')
        if tracking_error:
            err_msg = Float64MultiArray()
            err_msg.data = list(tracking_error)
            self._tracking_error_pub.publish(err_msg)

        # Publish diagnostics as named key-value pairs
        cond = telem.get('cond_number')
        if cond is not None:
            ws_status = telem.get('workspace_status', 'unknown')
            ws_scale = telem.get('workspace_speed_scale', 1.0)
            traj_state = telem.get('traj_state', 'unknown')
            traj_progress = telem.get('traj_progress')
            fault = telem.get('fault_state')

            diag_msg = DiagnosticStatus()
            diag_msg.name = 'motion/control_loop'
            diag_msg.hardware_id = 'motion_planner'

            # Set level based on workspace/fault status
            if fault:
                diag_msg.level = DiagnosticStatus.ERROR
                diag_msg.message = f'FAULT: {fault}'
            elif ws_status == 'hard':
                diag_msg.level = DiagnosticStatus.ERROR
                diag_msg.message = 'Workspace hard limit'
            elif ws_status == 'soft':
                diag_msg.level = DiagnosticStatus.WARN
                diag_msg.message = 'Workspace soft limit'
            else:
                diag_msg.level = DiagnosticStatus.OK
                diag_msg.message = 'OK'

            diag_msg.values = [
                KeyValue(key='cond_number', value=f'{cond:.1f}'),
                KeyValue(key='workspace_status', value=str(ws_status)),
                KeyValue(key='workspace_speed_scale', value=f'{ws_scale:.3f}'),
                KeyValue(key='traj_state', value=str(traj_state)),
            ]
            if traj_progress is not None:
                diag_msg.values.append(
                    KeyValue(key='traj_progress', value=f'{traj_progress:.3f}'))
            if fault:
                diag_msg.values.append(
                    KeyValue(key='fault_state', value=str(fault)))

            self._diagnostics_pub.publish(diag_msg)

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
