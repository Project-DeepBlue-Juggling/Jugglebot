"""ROS2 bridge node for MPC target setting.

Subscribes to the ROS2 pose input sources (spacemouse, GUI) and forwards
target commands to the MPC process via ZeroMQ.

The MPC process receives targets on :5558 and uses them as the tracking
reference for its optimisation.  This node is the single gateway between
ROS2 input topics and the MPC — it converts message types, enforces
mode-gating (only the active source's targets are forwarded), and handles
quaternion → rotation-vector conversion.

ROS2 → ZMQ (:5558):
  - control_mode_topic  (String)               → MPC mode
  - platform_pose_topic (PlatformPoseCommand)   → MPC target (SPACEMOUSE/GUI)
  - gravity_offset (Float64MultiArray)         → stored correction applied to all targets

No motion planning is done here — this node is a pure translator.  The
gravity offset (from levelling) is composed into every outgoing target's
orientation so the MPC sees corrected references transparently.  That
transform is **not** implemented here: it lives in
``jugglebot.motion.levelling`` and is shared with ``trajectory_node`` under
contract C-LEVEL-1 (``ros_ws/docs/levelling_frame.md``).  Until 2026-07-25
this node carried a verbatim second copy of it; two implementations of a
normative transform is how a contract drifts, and the composition is not
commutative, so a re-derived copy is a silent frame error.
"""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from jugglebot_interfaces.msg import PlatformPoseCommand

from jugglebot.motion.ipc import (
    MpcBridgeIPC,
    SessionMetadataPull,
    make_mpc_target,
    make_mpc_mode,
)
from jugglebot.motion import levelling
from jugglebot.motion.ik_solver import (
    quat_to_rot_matrix,
    rot_matrix_to_rotvec,
)


class MpcBridgeNode(Node):
    """Bridges ROS2 input topics to MPC target commands via ZeroMQ.

    Mode-gating logic:
      - SPACEMOUSE/GUI modes forward ``platform_pose_topic`` messages
        whose ``publisher`` field matches the active mode.
      - All other modes (LEVELLING, ERROR, empty) send a ``disabled`` mode
        message and suppress all target forwarding.
    """

    # Modes that accept PlatformPoseCommand targets
    _POSE_MODES = frozenset({'SPACEMOUSE', 'GUI'})
    # Modes where the MPC should be active
    _ACTIVE_MODES = _POSE_MODES

    def __init__(self):
        super().__init__('mpc_bridge')

        # ZMQ publisher for MPC targets and mode
        self._ipc = MpcBridgeIPC()
        self._session_pull = SessionMetadataPull()
        self.get_logger().info(
            "MPC target bridge initialised "
            "(targets → tcp://127.0.0.1:5558, session ← tcp://127.0.0.1:5560)")

        # Current active mode from orchestrator
        self._current_mode = ''

        # Gravity correction rotation matrix (identity = no correction).
        # Set by the orchestrator after levelling.  Applied to every
        # outgoing target orientation (C-LEVEL-1 ingest B1) — this node has
        # exactly one pose surface, so B1 is the whole enumeration here.
        self._gravity_correction: np.ndarray = levelling.identity_correction()

        # ── ROS2 subscriptions ────────────────────────────────────

        # Control mode from orchestrator (determines which source is active)
        self.create_subscription(
            String, 'control_mode_topic',
            self._on_control_mode, 10)

        # Gravity offset from orchestrator (levelling result)
        self.create_subscription(
            Float64MultiArray, 'gravity_offset',
            self._on_gravity_offset, 10)

        # Platform pose from spacemouse / GUI
        self.create_subscription(
            PlatformPoseCommand, 'platform_pose_topic',
            self._on_platform_pose, 10)

        # Poll for session metadata from MPC process (1 Hz is plenty)
        self.create_timer(1.0, self._poll_session_metadata)

    # ------------------------------------------------------------------
    # Session metadata (for hardware diagnosis log correlation)
    # ------------------------------------------------------------------

    def _poll_session_metadata(self) -> None:
        """Check for session-start messages from the MPC process."""
        msg = self._session_pull.recv()
        if msg is not None and msg.get('type') == 'session_start':
            csv_filename = msg.get('csv_filename', 'unknown')
            self.get_logger().info(
                f"MPC session started: {csv_filename}")

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
        measured tilt error.  The sign convention that turns it into the
        counter-tilting correction lives in ``motion/levelling.py``
        (C-LEVEL-1's single shared implementation).
        """
        tilt_x, tilt_y = msg.data[0], msg.data[1]
        self._gravity_correction = levelling.correction_from_offset(
            tilt_x, tilt_y)
        self.get_logger().info(
            f"Gravity correction set: tilt=[{tilt_x:.4f}, {tilt_y:.4f}] rad")

    # ------------------------------------------------------------------
    # Target forwarding
    # ------------------------------------------------------------------

    def _on_platform_pose(self, msg: PlatformPoseCommand) -> None:
        """Convert PlatformPoseCommand to MPC target (spacemouse/GUI).

        Only forwards when:
          1. The current mode is a pose-accepting mode (SPACEMOUSE/GUI)
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

        # C-LEVEL-1 ingest B1: a wire target is EXTERNAL ⇒ corrected exactly once,
        # rotation only.  Rebuilt as plain floats because the ZMQ payload is
        # msgpack-serialised (a numpy scalar would not encode).
        corrected = levelling.correct_pose(
            [pos.x, pos.y, pos.z, rotvec[0], rotvec[1], rotvec[2]],
            self._gravity_correction)
        target_pose = [float(v) for v in corrected]

        # Spacemouse/GUI targets: arrive ASAP, hold at target
        self._ipc.send_target(make_mpc_target(
            target_pose=target_pose,
            source=source.lower(),
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
        self._session_pull.close()
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
