"""
This ROS2 node reads the state of a 3Dconnexion SpaceMouse and publishes the pose of the platform to the
'platform_pose_topic' topic while SPACEMOUSE is the active control mode (per 'control_mode_topic').
Outside that mode it does nothing — no timer, no open device, nothing read.

Idle-CPU lifecycle (2026-09-14): outside SPACEMOUSE mode this node holds no timer and the device is
closed; its only live entity is the control_mode_topic subscription. Entering SPACEMOUSE mode creates the
100 Hz tick timer (the device then opens on the first tick, via the usual rate-limited _ensure_open());
leaving it destroys the timer and closes the device. This replaced an always-on 100 Hz
`while rclpy.ok(): spin_once` loop that cost 7-8% of a core with no device connected at all (2026-09-13
pidstat: live launch; 5.9% standalone on an idle Jetson) — the cost was the timer + spin_once loop itself, not HID scanning
(pyspacemouse.open() with no device measured 0.36 ms).

The sole consumer is trajectory_node._on_platform_pose (the follower), which only accepts this topic in a
follower (pose-accepting) mode whose 'publisher' field matches the active mode — so this node publishing
(or not) outside SPACEMOUSE mode was already inert downstream; the mode gate here just stops the wasted
work at the source instead of discarding it one hop later. If this node goes silent entirely (e.g.
crashed) while SPACEMOUSE is active, trajectory_node's own `follower_input_loss_s` (0.4 s) staleness check
is the backstop — it stops the follower rather than run on a stale target.

Connection handling mirrors the validated mocap-handler contract: the node stays alive whether or not the
SpaceMouse is present, retries quietly with a state-transition + throttled-recurring WARNING, and
auto-connects when the device is plugged in. On a mid-session unplug *while SpaceMouse is the active
control mode*, it commands the platform smoothly back to the ACTIVE pose (0,0,170,0,0,0) — the follower
profiles that move, not a step command — so the platform never freezes at whatever pose the stick last
commanded. (SpaceMouse mode is only used for qualitative testing, so a known safe home on loss-of-device
is the most robust behaviour.)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from jugglebot_interfaces.msg import PlatformPoseCommand
from geometry_msgs.msg import PoseStamped

import quaternion  # numpy quaternion
import pyspacemouse
import math
import time
import jugglebot.hardware_config as hw

class SpaceMouseHandler(Node):
    # Auto-reconnect cadence. NOT the 100 Hz publish rate: pyspacemouse.open()
    # enumerates HID devices and emits bare print()s — hammering it every tick
    # would spam stdout. (There is no unbounded-hang failure mode here, unlike
    # the TCP mocap path, so no asyncio.wait_for-style bound is needed.)
    _RECONNECT_INTERVAL_S = 2.0

    # rclpy-throttled "SpaceMouse not connected" WARNING cadence: logs
    # immediately on the first failure, then <= once per this period while the
    # device is absent. A reliable, ongoing declaration without per-tick spam.
    # Only ever fires while SPACEMOUSE is the active mode — the tick that
    # calls _ensure_open() exists nowhere else.
    _OUTAGE_WARN_THROTTLE_S = 30.0

    def __init__(self):
        super().__init__('spacemouse_handler')

        # Subscribe to control_mode_topic to see if spacemouse is enabled.
        # This subscription is the node's only live entity outside SPACEMOUSE
        # mode — no timer, no open device, until the first mode message.
        self.subscription = self.create_subscription(String, 'control_mode_topic', self.control_mode_callback, 10)
        self.spacemouse_enabled = False

        # Create the platform-pose publisher, but NOT the timer: the timer
        # is mode-scoped (created/destroyed by control_mode_callback) so an
        # idle session (SpaceMouse never selected) costs nothing.
        self.publisher_ = self.create_publisher(PlatformPoseCommand, 'platform_pose_topic', 10)
        self.timer = None

        # Connection state. The node never self-terminates on a missing
        # device — it retries quietly and connects automatically when the
        # SpaceMouse appears. The device is deliberately NOT opened here:
        # the control mode is unknown at construction time (the first
        # control_mode_topic message arrives within ~100 ms of the
        # orchestrator's 10 Hz stream), and opening it before SPACEMOUSE is
        # even selected is exactly the always-on cost this lifecycle removes.
        self._is_open = False
        self._last_open_attempt = 0.0  # time.monotonic(); gates reconnect rate

    # ──────────────────────────────────────────────────────────────────────
    #  Connection management
    # ──────────────────────────────────────────────────────────────────────

    def _ensure_open(self) -> bool:
        """Rate-limited, guarded (re)connect to the SpaceMouse.

        Returns True if a device is open. Logs only on state transitions:
        one INFO on (re)connect, one rclpy-throttled WARNING while absent.
        """
        if self._is_open:
            return True
        now = time.monotonic()
        if now - self._last_open_attempt < self._RECONNECT_INTERVAL_S:
            return False
        self._last_open_attempt = now
        try:
            opened = bool(pyspacemouse.open())
        except Exception:
            opened = False
        if opened:
            self._is_open = True
            self.get_logger().info("SpaceMouse connected.")
            return True
        self.get_logger().warning(
            "SpaceMouse not connected — retrying in background; will "
            "connect automatically when it is plugged in",
            throttle_duration_sec=self._OUTAGE_WARN_THROTTLE_S,
        )
        return False

    def _handle_disconnect(self):
        """Transition connected → disconnected: close + one WARNING.

        Only ever called from the 100 Hz tick, which only exists while
        SPACEMOUSE is the active mode, so the platform-homing the caller does
        right after this always applies. This logs one WARNING per disconnect
        *event* (distinct from the throttled "still absent" line in
        _ensure_open), mirroring the mocap _on_qtm_disconnect contract.
        """
        if not self._is_open:
            return
        self._is_open = False
        try:
            pyspacemouse.close()
        except Exception:
            pass
        self.get_logger().warning(
            "SpaceMouse disconnected — homing platform to ACTIVE; "
            "reconnecting in background"
        )
        # Attempt a prompt first reconnect on the next tick (then rate-limited).
        self._last_open_attempt = 0.0

    # ──────────────────────────────────────────────────────────────────────
    #  Publishing
    # ──────────────────────────────────────────────────────────────────────

    def publish_pose(self):
        """Read the SpaceMouse and publish the platform pose at 100 Hz.

        Exists only while SPACEMOUSE is the active mode (the timer is
        created/destroyed by control_mode_callback), so every tick here is
        "in mode" — the old not-in-mode drain-the-HID-buffer branch is gone;
        closing the device on mode-exit already discards whatever the
        hidraw buffer was holding (the buffer is per open()).

        Disconnected behaviour:
          - always retry (rate-limited) and keep the node alive;
          - while not open, hold the platform at ACTIVE (the follower profiles
            the move home and holds it). Publishing ACTIVE every tick is
            idempotent, survives a follower restart, and is continuous with
            reconnect (stick-at-rest == ACTIVE).
        """
        if not self._is_open:
            self._ensure_open()
            if not self._is_open:
                self._publish_pose_from_axes(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                return
            # Just reconnected — fall through and read the device.

        try:
            state = pyspacemouse.read()
        except Exception:
            # SpaceMouse was actively flying the platform — close, warn, and
            # send it smoothly home so it doesn't freeze at the last stick pose.
            self._handle_disconnect()
            self._publish_pose_from_axes(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            return
        if state is None:
            return

        self._publish_pose_from_axes(state.x, state.y, state.z,
                                     state.roll, state.pitch, state.yaw)

    def _publish_pose_from_axes(self, sx, sy, sz, sroll, spitch, syaw):
        """Build + publish a PlatformPoseCommand from raw SpaceMouse axes.

        All-zero axes == the ACTIVE pose (0, 0, JB_OP_DEFAULT_ACTIVE_Z_MM,
        identity) by construction, so the disconnect-home path reuses this
        exact transform — no new frame assumptions, and continuous with a
        reconnect where the stick is at rest.
        """
        # Set the multipliers for each axis (mm, deg)
        xy_mult = hw.SPACEMOUSE_XY_MULT_MM
        z_mult = hw.SPACEMOUSE_Z_MULT_MM
        pitch_roll_mult = hw.SPACEMOUSE_PITCH_ROLL_MULT_DEG
        yaw_mult = hw.SPACEMOUSE_YAW_MULT_DEG

        # Set the offset in z to put baseline position at ~midspan of robot
        z_offset = hw.JB_OP_DEFAULT_ACTIVE_Z_MM

        # Initialise PlatformPoseCommand object
        message = PlatformPoseCommand()
        pose_stamped = PoseStamped()

        # Get the current time
        current_time = self.get_clock().now().to_msg()

        # Apply multipliers and convert to radians
        # Not sure why I need negatives out the front of pitch and yaw, but this works!
        roll = math.radians(sroll * pitch_roll_mult)
        pitch = math.radians(-spitch * pitch_roll_mult)
        yaw = math.radians(-syaw * yaw_mult)

        # Convert orientation from Euler angles to quaternions
        q_roll = quaternion.from_rotation_vector([0, roll, 0])
        q_pitch = quaternion.from_rotation_vector([pitch, 0, 0])
        q_yaw = quaternion.from_rotation_vector([0, 0, yaw])

        quaternion_ori = q_yaw * q_roll * q_pitch

        # Construct the pose message
        pose_stamped.pose.position.x = sx * xy_mult
        pose_stamped.pose.position.y = sy * xy_mult
        pose_stamped.pose.position.z = sz * z_mult + z_offset
        pose_stamped.pose.orientation.x = quaternion_ori.x
        pose_stamped.pose.orientation.y = quaternion_ori.y
        pose_stamped.pose.orientation.z = quaternion_ori.z
        pose_stamped.pose.orientation.w = quaternion_ori.w

        # Set the time stamp
        pose_stamped.header.stamp = current_time
        pose_stamped.header.frame_id = 'platform_start'

        message.pose_stamped = pose_stamped
        message.publisher = 'SPACEMOUSE'

        self.publisher_.publish(message)

    def control_mode_callback(self, msg):
        """Enter/exit SPACEMOUSE mode — idempotent against repeated identical
        messages from the orchestrator's 10 Hz control_mode_topic stream.

        Entering creates the 100 Hz tick timer and resets the reconnect
        limiter so the first open attempt (on the very next tick) is
        immediate rather than waiting out a stale _RECONNECT_INTERVAL_S from
        a previous session. Leaving destroys the timer and closes the device
        if open — the node then holds no timer and touches no HID device
        again until SPACEMOUSE is selected.
        """
        if msg.data == 'SPACEMOUSE' and not self.spacemouse_enabled:
            self.get_logger().info('Spacemouse enabled')
            self.spacemouse_enabled = True
            self._last_open_attempt = 0.0
            self.timer = self.create_timer(0.01, self.publish_pose)

        elif msg.data != 'SPACEMOUSE' and self.spacemouse_enabled:
            self.get_logger().info('Spacemouse disabled')
            self.spacemouse_enabled = False
            self.destroy_timer(self.timer)
            self.timer = None
            if self._is_open:
                self._is_open = False
                try:
                    pyspacemouse.close()
                except Exception:
                    pass

    #########################################################################################################
    #                                          Utility Functions                                            #
    #########################################################################################################

    def on_shutdown(self):
        """Handle node shutdown."""
        self.get_logger().info("Shutting down SpacemouseHandler...")
        try:
            if self._is_open:
                pyspacemouse.close()
        except Exception as e:
            self.get_logger().error(f"Error during node shutdown: {e}")



def main(args=None):
    rclpy.init(args=args)
    node = SpaceMouseHandler()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
