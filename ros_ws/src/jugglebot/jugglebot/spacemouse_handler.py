"""
This ROS2 node reads the state of a 3Dconnexion SpaceMouse and publishes the pose of the platform to the 'platform_pose' topic.
The node subscribes to the 'control_state' topic to see if the spacemouse is the chosen control method.
If the spacemouse is enabled, the node reads the state of the spacemouse and publishes the pose of the platform.
Otherwise, the node does nothing.

Connection handling mirrors the validated mocap-handler contract: the node
stays alive whether or not the SpaceMouse is present, retries quietly with a
state-transition + throttled-recurring WARNING, and auto-connects when the
device is plugged in. On a mid-session unplug *while SpaceMouse is the active
control mode*, it commands the platform smoothly back to the ACTIVE pose
(0,0,170,0,0,0) — the MPC profiles that move — so the platform never freezes
at whatever pose the stick last commanded. (SpaceMouse mode is only used for
qualitative testing, so a known safe home on loss-of-device is the most
robust behaviour.)
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
    _OUTAGE_WARN_THROTTLE_S = 30.0

    def __init__(self):
        super().__init__('spacemouse_handler')

        # Subscribe to control_mode_topic to see if spacemouse is enabled
        self.subscription = self.create_subscription(String, 'control_mode_topic', self.control_mode_callback, 10)
        self.spacemouse_enabled = False

        # Create a publisher for the platform pose and a timer to publish it
        self.publisher_ = self.create_publisher(PlatformPoseCommand, 'platform_pose_topic', 10)
        self.timer = self.create_timer(0.01, self.publish_pose)

        # Connection state. The node never self-terminates on a missing
        # device — it retries quietly and connects automatically when the
        # SpaceMouse appears.
        self._is_open = False
        self._last_open_attempt = 0.0  # time.monotonic(); gates reconnect rate

        # One non-blocking attempt at startup (logs the throttled WARNING
        # immediately if the device is absent).
        self._ensure_open()

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

        The platform-homing is handled by the caller (it only applies when
        SpaceMouse is the active control mode). This logs one WARNING per
        disconnect *event* (distinct from the throttled "still absent" line
        in _ensure_open), mirroring the mocap _on_qtm_disconnect contract.
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

        Disconnected behaviour:
          - always retry (rate-limited) and keep the node alive;
          - if SpaceMouse is the active control mode, hold the platform at
            ACTIVE (the MPC profiles the move home and holds it). Publishing
            ACTIVE every tick is idempotent, survives an MPC restart, and is
            continuous with reconnect (stick-at-rest == ACTIVE).
        """
        if not self._is_open:
            self._ensure_open()
            if not self._is_open:
                if self.spacemouse_enabled:
                    self._publish_pose_from_axes(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                return
            # Just reconnected — fall through and read the device.

        if not self.spacemouse_enabled:
            # Not the active control method — drain the HID buffer, publish
            # nothing. A read failure here means the device went away.
            try:
                pyspacemouse.read()
            except Exception:
                self._handle_disconnect()
            return

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
        # If the incoming state calls for the spacemouse, enable it
        if msg.data == 'SPACEMOUSE' and not self.spacemouse_enabled:
            self.get_logger().info('Spacemouse enabled')
            self.spacemouse_enabled = True

        elif msg.data != 'SPACEMOUSE' and self.spacemouse_enabled:
            self.get_logger().info('Spacemouse disabled')
            self.spacemouse_enabled = False

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
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
