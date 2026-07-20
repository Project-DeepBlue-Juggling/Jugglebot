"""ROS2 bridge node for the motor guard process.

Translates between the ROS2 topic/service world and the ZeroMQ IPC
layer used by the motor guard.

ROS2 → IPC:
  - control_mode_topic (from orchestrator) → ESTOP on ERROR only
  - robot_state (from CAN node) → MotorFeedback

IPC → ROS2:
  - Telemetry (from motor guard) → leg_lengths_topic (18 values:
    6 pos_rev + 6 vel_ff_rps + 6 torque_ff_Nm for CAN node's set_input_pos)

Motor-guard enable/disable is owned by ``HardwarePlant`` (in run_mpc.py), not
this bridge.  Launching run_mpc.py enables the guard; Ctrl-C disables it.
This bridge only forwards ESTOP on ERROR mode and gates topic publishing
based on whether the current control_mode permits motor motion.
"""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from std_msgs.msg import Float64MultiArray, String
from jugglebot_interfaces.msg import RobotState

from jugglebot.motion.ipc import (
    BridgeIPC,
    make_mode_command,
    make_motor_feedback,
)


class MotionBridgeNode(Node):
    """Bridges ROS2 topics to/from the motor guard process via ZeroMQ."""

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

        # Control mode from orchestrator
        self.create_subscription(
            String, 'control_mode_topic',
            self._on_control_mode, 10)

        # Robot state from CAN node (motor feedback for motor guard)
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
        self._motor_guard_enabled = False  # gate for leg command publishing

    # ------------------------------------------------------------------
    # ROS2 → IPC callbacks
    # ------------------------------------------------------------------

    # Modes that permit motor_guard telemetry to be forwarded to CAN.
    # STANDBY is included because run_mpc.py may be running and driving the
    # platform; the motor guard's own enable state is owned by HardwarePlant.
    _ACTIVE_MODES = frozenset(
        {'STANDBY', 'SPACEMOUSE', 'GUI', 'CATCH'})

    def _on_control_mode(self, msg: String) -> None:
        """Translate control mode changes to IPC mode commands.

        The only IPC mode command this bridge sends is ``estop`` on ERROR.
        Enable/disable is owned by ``HardwarePlant`` (run_mpc.py).  The local
        ``_motor_guard_enabled`` flag gates ``leg_lengths_topic`` publishing
        based on whether the orchestrator's mode permits motor motion.
        """
        mode = msg.data
        prev = self._current_control_mode
        self._current_control_mode = mode

        self._motor_guard_enabled = mode in self._ACTIVE_MODES

        if mode == 'ERROR' and prev != 'ERROR':
            cmd = make_mode_command('estop')
            self.ipc.send_mode_command(cmd)
            self.get_logger().warning("Sent 'estop' to motor guard")

    def _on_robot_state(self, msg: RobotState) -> None:
        """Forward motor feedback from CAN node to the motor guard."""
        states = msg.motor_states
        if len(states) < 6:
            return
        # First 6 entries are the leg motors
        fb = make_motor_feedback(
            positions=[states[i].pos_estimate for i in range(6)],
            velocities=[states[i].vel_estimate for i in range(6)],
            currents=[states[i].iq_measured for i in range(6)],
        )
        self.ipc.send_motor_feedback(fb)

    # ------------------------------------------------------------------
    # IPC → ROS2 polling
    # ------------------------------------------------------------------

    def _poll_telemetry(self) -> None:
        """Poll telemetry from the motor guard and publish to ROS2."""
        telem = self.ipc.recv_telemetry()
        if telem is None:
            return

        positions = telem.get('leg_pos')
        velocities = telem.get('leg_vel')
        torques = telem.get('leg_torques')

        # Publish unified command to CAN node: 6 pos + 6 vel_ff + 6 torque_ff
        # Only when motor guard is enabled AND telemetry contains valid motor
        # commands.  Refusing to publish when keys are missing prevents sending
        # default zeros which would command full retraction.
        if (self._motor_guard_enabled
                and positions is not None
                and velocities is not None
                and torques is not None):
            leg_msg = Float64MultiArray()
            leg_msg.data = list(positions) + list(velocities) + list(torques)
            self._leg_pub.publish(leg_msg)

        # Publish feedforward torques on diagnostic topic (monitoring only)
        if torques:
            diag_msg = Float64MultiArray()
            diag_msg.data = torques
            self._torque_pub.publish(diag_msg)

        # Publish per-leg tracking errors (mm)
        tracking_error = telem.get('tracking_error_mm')
        if tracking_error:
            err_msg = Float64MultiArray()
            err_msg.data = list(tracking_error)
            self._tracking_error_pub.publish(err_msg)

        # Publish diagnostics as named key-value pairs.
        #
        # INVARIANT: a fault / ESTOP signal MUST NOT be gated behind an
        # optional monitoring field.  Pre-2026-05-19 this block was
        # gated solely on ``cond_number is not None``.  motor_guard's
        # ESTOP-path telemetry (`_publish_fault_telemetry`) deliberately
        # omits ``cond_number``, so a ``GuardMode.ESTOP`` was NEVER
        # published here — the fault was invisible to operator / GUI /
        # rosbag for the entire fault (root cause of a 68 s blind
        # window; see
        # logbook/2026-05-19-findingb-motor-guard-estop-latch-observability.md).
        # Gate on "is there anything worth reporting" — a fault OR a
        # cond reading — never on the monitoring field alone.
        cond = telem.get('cond_number')
        fault = telem.get('fault_state')
        if cond is not None or fault:
            ws_status = telem.get('workspace_status', 'unknown')
            ws_scale = telem.get('workspace_speed_scale', 1.0)

            diag_msg = DiagnosticStatus()
            diag_msg.name = 'motion/motor_guard'
            diag_msg.hardware_id = 'motor_guard'

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
                KeyValue(key='workspace_status', value=str(ws_status)),
                KeyValue(key='workspace_speed_scale', value=f'{ws_scale:.3f}'),
            ]
            # cond_number is monitoring-only — include it when the
            # telemetry path carries it (absent on the ESTOP path), at
            # the head of the list to preserve the historical ordering.
            if cond is not None:
                diag_msg.values.insert(
                    0, KeyValue(key='cond_number', value=f'{cond:.1f}'))
            if fault:
                diag_msg.values.append(
                    KeyValue(key='fault_state', value=str(fault)))

            self._diagnostics_pub.publish(diag_msg)

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def on_shutdown(self) -> None:
        """Clean up IPC on node shutdown."""
        # Send disable to motor guard
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
