"""ROS2 bridge node for MPC target setting.

Subscribes to all ROS2 input sources (spacemouse, GUI, shell, catch
coordinator) and forwards target commands to the MPC process via ZeroMQ.

The MPC process receives targets on :5558 and uses them as the tracking
reference for its optimisation.  This node is the single gateway between
ROS2 input topics and the MPC — it converts message types, enforces
mode-gating (only the active source's targets are forwarded), and handles
quaternion → rotation-vector conversion.

ROS2 → ZMQ (:5558):
  - control_mode_topic  (String)               → MPC mode
  - platform_pose_topic (PlatformPoseCommand)   → MPC target (SPACEMOUSE/GUI/SHELL)
  - catch/dynamic_target (DynamicTargetCommand) → MPC target (CATCH)
  - gravity_offset (Float64MultiArray)         → stored correction applied to all targets

No motion planning is done here — this node is a pure translator.  The
gravity offset (from levelling) is composed into every outgoing target's
orientation via rotation-matrix multiplication so the MPC sees corrected
references transparently.
"""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from jugglebot_interfaces.msg import PlatformPoseCommand, DynamicTargetCommand

from jugglebot.motion.ipc import (
    MpcBridgeIPC,
    make_mpc_target,
    make_mpc_mode,
)
from jugglebot.motion.ik_solver import (
    quat_to_rot_matrix,
    rot_matrix_to_rotvec,
    rotvec_to_rot_matrix,
)


class MpcBridgeNode(Node):
    """Bridges ROS2 input topics to MPC target commands via ZeroMQ.

    Mode-gating logic:
      - SPACEMOUSE/GUI/SHELL modes forward ``platform_pose_topic`` messages
        whose ``publisher`` field matches the active mode.
      - CATCH mode forwards ``catch/dynamic_target`` messages.
      - All other modes (LEVELLING, ERROR, empty) send a ``disabled`` mode
        message and suppress all target forwarding.
    """

    # Modes that accept PlatformPoseCommand targets
    _POSE_MODES = frozenset({'SPACEMOUSE', 'SHELL', 'GUI'})
    # Modes that accept DynamicTargetCommand targets
    _CATCH_MODES = frozenset({'CATCH'})
    # Union of all modes where the MPC should be active
    _ACTIVE_MODES = _POSE_MODES | _CATCH_MODES

    def __init__(self):
        super().__init__('mpc_bridge')

        # ZMQ publisher for MPC targets and mode
        self._ipc = MpcBridgeIPC()
        self.get_logger().info(
            "MPC target bridge initialised "
            "(targets → tcp://127.0.0.1:5558)")

        # Current active mode from orchestrator
        self._current_mode = ''

        # Gravity correction rotation matrix (identity = no correction).
        # Set by the orchestrator after levelling.  Applied to every
        # outgoing target orientation via rotation-matrix composition.
        self._gravity_correction: np.ndarray = np.eye(3)

        # ── ROS2 subscriptions ────────────────────────────────────

        # Control mode from orchestrator (determines which source is active)
        self.create_subscription(
            String, 'control_mode_topic',
            self._on_control_mode, 10)

        # Gravity offset from orchestrator (levelling result)
        self.create_subscription(
            Float64MultiArray, 'gravity_offset',
            self._on_gravity_offset, 10)

        # Platform pose from spacemouse / GUI / shell
        self.create_subscription(
            PlatformPoseCommand, 'platform_pose_topic',
            self._on_platform_pose, 10)

        # Dynamic target from catch coordinator
        self.create_subscription(
            DynamicTargetCommand, 'catch/dynamic_target',
            self._on_catch_target, 10)

    # ------------------------------------------------------------------
    # Mode handling
    # ------------------------------------------------------------------

    def _on_control_mode(self, msg: String) -> None:
        """Forward mode transitions to the MPC process."""
        mode = msg.data
        if mode == self._current_mode:
            return

        prev = self._current_mode
        self._current_mode = mode

        if mode in self._ACTIVE_MODES:
            self._ipc.send_mode(make_mpc_mode(mode.lower()))
            self.get_logger().info(f"MPC mode → {mode.lower()}")
        elif prev in self._ACTIVE_MODES:
            # Transition from active → inactive: tell MPC to stop tracking
            self._ipc.send_mode(make_mpc_mode('disabled'))
            self.get_logger().info("MPC mode → disabled")

    # ------------------------------------------------------------------
    # Gravity offset
    # ------------------------------------------------------------------

    def _on_gravity_offset(self, msg: Float64MultiArray) -> None:
        """Store the gravity correction from levelling.

        The orchestrator publishes [tilt_x, tilt_y] in radians — the
        measured tilt error.  The correction is the inverse rotation
        (negate the angles) so that commanding "zero tilt" actually
        counter-tilts the platform to true level.
        """
        tilt_x, tilt_y = msg.data[0], msg.data[1]
        correction_rotvec = np.array([-tilt_x, -tilt_y, 0.0])
        self._gravity_correction = rotvec_to_rot_matrix(correction_rotvec)
        self.get_logger().info(
            f"Gravity correction set: tilt=[{tilt_x:.4f}, {tilt_y:.4f}] rad")

    def _apply_gravity_correction(self, rotvec: list) -> list:
        """Compose gravity correction into a target rotation vector.

        Returns the corrected rotation vector: R_corrected = R_gravity @ R_target.
        """
        R_target = rotvec_to_rot_matrix(np.array(rotvec))
        R_corrected = self._gravity_correction @ R_target
        rv = rot_matrix_to_rotvec(R_corrected)
        return [float(rv[0]), float(rv[1]), float(rv[2])]

    # ------------------------------------------------------------------
    # Target forwarding
    # ------------------------------------------------------------------

    def _on_platform_pose(self, msg: PlatformPoseCommand) -> None:
        """Convert PlatformPoseCommand to MPC target (spacemouse/GUI/shell).

        Only forwards when:
          1. The current mode is a pose-accepting mode (SPACEMOUSE/GUI/SHELL)
          2. The message's publisher field matches the active mode
        """
        if self._current_mode not in self._POSE_MODES:
            return

        source = msg.publisher.upper()
        if source != self._current_mode:
            return

        # Extract position (mm, relative to active pose)
        pos = msg.pose_stamped.pose.position
        ori = msg.pose_stamped.pose.orientation

        # Quaternion (w, x, y, z) → rotation vector [rx, ry, rz]
        rotvec = _quat_msg_to_rotvec(ori.w, ori.x, ori.y, ori.z)
        rotvec = self._apply_gravity_correction(rotvec)

        target_pose = [pos.x, pos.y, pos.z,
                       rotvec[0], rotvec[1], rotvec[2]]

        # Spacemouse/GUI/shell targets: arrive ASAP, hold at target
        self._ipc.send_target(make_mpc_target(
            target_pose=target_pose,
            source=source.lower(),
        ))

    def _on_catch_target(self, msg: DynamicTargetCommand) -> None:
        """Convert DynamicTargetCommand to MPC target (catch coordinator).

        Only forwards when the current mode is CATCH.
        """
        if self._current_mode not in self._CATCH_MODES:
            return

        # Position (mm, relative to active pose)
        catch_rotvec = _quat_msg_to_rotvec(
            msg.target_quat.w, msg.target_quat.x,
            msg.target_quat.y, msg.target_quat.z)
        catch_rotvec = self._apply_gravity_correction(catch_rotvec)
        target_pose = [
            msg.target_pos.x, msg.target_pos.y, msg.target_pos.z,
            catch_rotvec[0], catch_rotvec[1], catch_rotvec[2],
        ]

        # Velocity at arrival (linear only; angular = 0)
        target_twist = [
            msg.target_vel.x, msg.target_vel.y, msg.target_vel.z,
            0.0, 0.0, 0.0,
        ]

        self._ipc.send_target(make_mpc_target(
            target_pose=target_pose,
            arrival_time=msg.arrival_time,
            target_twist=target_twist,
            source='catch',
        ))

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def on_shutdown(self) -> None:
        """Send disable and close IPC on node shutdown."""
        try:
            self._ipc.send_mode(make_mpc_mode('disabled'))
        except Exception:
            pass
        self._ipc.close()
        self.get_logger().info("MPC target bridge closed")


# ----------------------------------------------------------------------
# Helpers
# ----------------------------------------------------------------------

def _quat_msg_to_rotvec(w: float, x: float, y: float, z: float) -> list:
    """Convert quaternion (w, x, y, z) → rotation vector [rx, ry, rz].

    Uses the production ``quat_to_rot_matrix`` and ``rot_matrix_to_rotvec``
    from ``ik_solver.py`` to ensure consistency with the rest of the motion
    pipeline.
    """
    rot_mat = quat_to_rot_matrix(w, x, y, z)
    rotvec = rot_matrix_to_rotvec(rot_mat)
    return [float(rotvec[0]), float(rotvec[1]), float(rotvec[2])]


# ----------------------------------------------------------------------
# Entry point
# ----------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = MpcBridgeNode()

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
