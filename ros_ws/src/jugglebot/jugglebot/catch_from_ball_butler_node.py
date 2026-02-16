"""
Catch-from-Ball-Butler node.

Moves Jugglebot's platform to catch a ball thrown by Ball Butler.
Sequence:
  1. Enter state -> platform moves to default pose, hand primes to top of stroke
  2. User calls /bb/throw_now -> Ball Butler throws and publishes ThrowAnnouncement
  3. This node receives announcement -> computes landing pose from announcement fields
  4. Platform adjusts position & orientation; hand catch trajectory is sent to Teensy
  5. After catch window passes -> transition back to STANDBY_ACTIVE
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from jugglebot_interfaces.msg import ThrowAnnouncement, PlatformPoseCommand, RobotState
from jugglebot_interfaces.srv import SetHandTrajCmd, SetFloat
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
import numpy as np
from scipy.spatial.transform import Rotation as R
from typing import Optional
from enum import Enum, auto
import threading


class _State(Enum):
    IDLE = auto()
    PRIMING = auto()
    WAITING_FOR_THROW = auto()
    CATCHING = auto()
    DONE = auto()


class CatchFromBallButlerNode(Node):
    # Platform geometry
    INITIAL_PLAT_HEIGHT = 565.0        # mm — platform base height from ground
    CATCH_Z_PLATFORM_FRAME = 170.0     # mm — default z in platform_start frame
    CATCH_PLANE_GLOBAL_Z = 735.0       # mm — 565 + 170, z-plane for landing calcs

    # Hand priming
    BALL_TOP_STROKE_POSITION_REVS = 9.858  # revs — hand prime position (top of stroke)

    # Catch constraints
    CATCH_ANGLE_LIMIT_DEG = 30.0       # max platform tilt angle
    XY_RANGE_OF_MOTION = 400.0         # mm — max distance from centre in any XY direction

    # Timing
    TIME_AFTER_CATCH_TO_TRANSITION = 2.0   # seconds after landing_time to transition to STANDBY
    WAITING_FOR_THROW_TIMEOUT = 120.0       # seconds without announcement before aborting

    def __init__(self):
        super().__init__('catch_from_ball_butler_node')

        self.callback_group = ReentrantCallbackGroup()

        # Internal state
        self._state = _State.IDLE
        self._state_lock = threading.Lock()
        self._landing_time_sec = None      # absolute ROS time (seconds) when ball arrives
        self._activation_time_sec = None   # when we entered WAITING_FOR_THROW

        # ---- Subscriptions ----
        self.control_mode_sub = self.create_subscription(
            String, 'control_mode_topic', self._control_mode_callback, 10)

        self.throw_announcement_sub = self.create_subscription(
            ThrowAnnouncement, '/throw_announcements', self._throw_announcement_callback, 10,
            callback_group=self.callback_group)

        self.robot_state_sub = self.create_subscription(
            RobotState, 'robot_state', self._robot_state_callback, 10,
            callback_group=self.callback_group)

        # ---- Publishers ----
        self.platform_pose_pub = self.create_publisher(PlatformPoseCommand, 'platform_pose_topic', 10)
        self.active_command_pub = self.create_publisher(String, 'active_command', 10)

        # ---- Service clients ----
        self.set_hand_traj_client = self.create_client(SetHandTrajCmd, 'set_hand_traj_cmd')
        self.smooth_move_hand_client = self.create_client(SetFloat, 'smooth_move_hand')

        # ---- State tracking ----
        self._last_known_hand_pos = None
        self._hand_state_lock = threading.Lock()

        # ---- Default pose ----
        self._default_pose = Pose()
        self._default_pose.position.x = 0.0
        self._default_pose.position.y = 0.0
        self._default_pose.position.z = self.CATCH_Z_PLATFORM_FRAME
        self._default_pose.orientation.x = 0.0
        self._default_pose.orientation.y = 0.0
        self._default_pose.orientation.z = 0.0
        self._default_pose.orientation.w = 1.0

        # ---- Timers ----
        self._platform_pub_timer = self.create_timer(0.01, self._publish_platform_pose)  # 100 Hz
        self._watchdog_timer = self.create_timer(0.1, self._watchdog_tick)  # 10 Hz

        # Pose to publish (updated when catch data arrives)
        self._current_target_pose = None

        self.get_logger().info("CatchFromBallButlerNode initialised.")

    # =====================================================================
    #  Subscription callbacks
    # =====================================================================

    def _control_mode_callback(self, msg: String):
        """Enable/disable this node based on the active control mode."""
        should_prime = False
        with self._state_lock:
            if msg.data == 'CATCH_FROM_BALL_BUTLER' and self._state == _State.IDLE:
                self.get_logger().info("CATCH_FROM_BALL_BUTLER activated — priming hand.")
                self._state = _State.PRIMING
                self._current_target_pose = self._default_pose
                should_prime = True

            elif msg.data != 'CATCH_FROM_BALL_BUTLER' and self._state != _State.IDLE:
                self.get_logger().info("CATCH_FROM_BALL_BUTLER deactivated.")
                self._reset()

        # Call _prime_hand outside the lock to avoid blocking timers during wait_for_service
        if should_prime:
            self._prime_hand()

    def _robot_state_callback(self, msg: RobotState):
        """Track hand motor position for priming status checks."""
        with self._hand_state_lock:
            self._last_known_hand_pos = msg.motor_states[6].pos_estimate

    def _throw_announcement_callback(self, msg: ThrowAnnouncement):
        """Handle incoming ThrowAnnouncement from Ball Butler."""
        with self._state_lock:
            if self._state != _State.WAITING_FOR_THROW:
                return

            if msg.thrower_name != "ball_butler":
                return

            self.get_logger().info("ThrowAnnouncement received from ball_butler — computing catch.")
            self._state = _State.CATCHING

        # --- Extract landing data ---
        landing_pos = msg.landing_position    # global frame, mm
        landing_vel = msg.landing_velocity    # global frame, mm/s
        landing_time_msg = msg.landing_time   # builtin_interfaces/Time

        landing_time_sec = landing_time_msg.sec + landing_time_msg.nanosec * 1e-9
        self._landing_time_sec = landing_time_sec

        # --- Check feasibility: XY range ---
        if abs(landing_pos.x) > self.XY_RANGE_OF_MOTION or abs(landing_pos.y) > self.XY_RANGE_OF_MOTION:
            self.get_logger().error(
                f"Landing position ({landing_pos.x:.0f}, {landing_pos.y:.0f}) mm "
                f"exceeds XY range of motion ({self.XY_RANGE_OF_MOTION:.0f} mm). Aborting catch.")
            self._transition_to_standby()
            return

        # --- Calculate platform orientation from landing velocity ---
        orientation = self._calculate_catch_orientation(landing_vel)
        if orientation is None:
            self.get_logger().error("Ball arrives at too steep an angle. Aborting catch.")
            self._transition_to_standby()
            return

        # --- Build platform pose ---
        catch_pose = Pose()
        catch_pose.position.x = landing_pos.x
        catch_pose.position.y = landing_pos.y
        catch_pose.position.z = self.CATCH_Z_PLATFORM_FRAME
        catch_pose.orientation = orientation

        self._current_target_pose = catch_pose

        # --- Command hand catch trajectory ---
        now_sec = self.get_clock().now().nanoseconds / 1e9
        event_delay = landing_time_sec - now_sec

        if event_delay <= 0:
            self.get_logger().error(
                f"Landing time is in the past (event_delay={event_delay:.2f}s). Aborting catch.")
            self._transition_to_standby()
            return

        if event_delay < 0.3:
            self.get_logger().warn(
                f"Event delay is very short ({event_delay:.2f}s). May miss catch.")

        # Compute ball speed at landing (m/s)
        speed_at_landing_mps = np.linalg.norm(
            [landing_vel.x, landing_vel.y, landing_vel.z]) / 1000.0

        # Clamp velocity to valid range for Teensy
        speed_at_landing_mps = max(0.3, min(7.0, speed_at_landing_mps))

        self.get_logger().info(
            f"Sending catch trajectory: event_delay={event_delay:.2f}s, "
            f"speed={speed_at_landing_mps:.2f} m/s")

        self._send_catch_trajectory(event_delay, speed_at_landing_mps)

    # =====================================================================
    #  Orientation calculation (same maths as catch_thrown_ball_node)
    # =====================================================================

    def _calculate_catch_orientation(self, landing_vel) -> Optional[Quaternion]:
        """
        Calculate platform orientation to be normal to the ball's velocity vector.
        Returns None if the angle exceeds CATCH_ANGLE_LIMIT_DEG.
        """
        reference_vector = np.array([0.0, 0.0, -1.0])
        velocity_vector = np.array([landing_vel.x, landing_vel.y, landing_vel.z])

        norm = np.linalg.norm(velocity_vector)
        if norm == 0:
            self.get_logger().warn("Landing velocity is zero — using identity orientation.")
            return Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        velocity_norm = velocity_vector / norm
        dot_product = np.dot(reference_vector, velocity_norm)

        # Clamp for numerical safety
        dot_product = np.clip(dot_product, -1.0, 1.0)
        angle_rad = np.arccos(dot_product)
        angle_deg = np.degrees(angle_rad)

        if angle_deg > self.CATCH_ANGLE_LIMIT_DEG:
            self.get_logger().warn(
                f"Ball arrives at {angle_deg:.1f}° from vertical "
                f"(limit: {self.CATCH_ANGLE_LIMIT_DEG:.0f}°).")
            return None

        rotation_axis = np.cross(reference_vector, velocity_norm)
        axis_norm = np.linalg.norm(rotation_axis)
        if axis_norm < 1e-9:
            # Velocity is (anti-)parallel to reference — no tilt needed
            return Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        rotation_axis_norm = rotation_axis / axis_norm
        rotation = R.from_rotvec(angle_rad * rotation_axis_norm)
        q = rotation.as_quat()  # [x, y, z, w]

        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

    # =====================================================================
    #  Hand motor commands
    # =====================================================================

    def _prime_hand(self):
        """Move hand to top of stroke via smooth-move, then transition to WAITING_FOR_THROW."""
        if not self.smooth_move_hand_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("smooth_move_hand service not available. Cannot prime hand.")
            self._transition_to_standby()
            return

        req = SetFloat.Request()
        req.data = self.BALL_TOP_STROKE_POSITION_REVS

        future = self.smooth_move_hand_client.call_async(req)
        future.add_done_callback(self._prime_hand_done)

    def _prime_hand_done(self, future):
        """Callback when smooth-move completes."""
        try:
            result = future.result()
            if result.success:
                self.get_logger().info("Hand primed. Waiting for throw announcement.")
                with self._state_lock:
                    self._state = _State.WAITING_FOR_THROW
                    self._activation_time_sec = self.get_clock().now().nanoseconds / 1e9
            else:
                self.get_logger().error(f"Hand priming failed: {result.message}")
                self._transition_to_standby()
        except Exception as e:
            self.get_logger().error(f"Hand priming service call failed: {e}")
            self._transition_to_standby()

    def _send_catch_trajectory(self, event_delay: float, speed_mps: float):
        """Send catch trajectory command to the Teensy via set_hand_traj_cmd service."""
        if not self.set_hand_traj_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("set_hand_traj_cmd service not available.")
            return

        req = SetHandTrajCmd.Request()
        req.event_delay = float(event_delay)
        req.event_vel = float(speed_mps)
        req.traj_type = 1  # catch

        future = self.set_hand_traj_client.call_async(req)
        future.add_done_callback(self._catch_traj_done)

    def _catch_traj_done(self, future):
        """Callback when catch trajectory command completes."""
        try:
            result = future.result()
            if result.success:
                self.get_logger().info("Catch trajectory armed on Teensy.")
            else:
                self.get_logger().error(f"Catch trajectory failed: {result.message}")
        except Exception as e:
            self.get_logger().error(f"Catch trajectory service call failed: {e}")

    # =====================================================================
    #  Platform pose publishing
    # =====================================================================

    def _publish_platform_pose(self):
        """Timer callback: publish the current target platform pose at 100 Hz."""
        with self._state_lock:
            if self._state == _State.IDLE or self._current_target_pose is None:
                return

        msg = PlatformPoseCommand()
        pose_stamped = PoseStamped()
        pose_stamped.pose = self._current_target_pose
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        msg.pose_stamped = pose_stamped
        msg.publisher = 'CATCH_FROM_BALL_BUTLER'

        self.platform_pose_pub.publish(msg)

    # =====================================================================
    #  Watchdog / state transitions
    # =====================================================================

    def _watchdog_tick(self):
        """10 Hz timer: check for post-catch transition and WAITING timeout."""
        with self._state_lock:
            state = self._state

        now_sec = self.get_clock().now().nanoseconds / 1e9

        if state == _State.CATCHING and self._landing_time_sec is not None:
            if now_sec > self._landing_time_sec + self.TIME_AFTER_CATCH_TO_TRANSITION:
                self.get_logger().info("Catch window elapsed. Returning to STANDBY_ACTIVE.")
                self._transition_to_standby()

        elif state == _State.WAITING_FOR_THROW and self._activation_time_sec is not None:
            elapsed = now_sec - self._activation_time_sec
            if elapsed > self.WAITING_FOR_THROW_TIMEOUT:
                self.get_logger().warn(
                    f"No ThrowAnnouncement received for {elapsed:.0f}s. Returning to STANDBY_ACTIVE.")
                self._transition_to_standby()

    def _transition_to_standby(self):
        """Publish command to return to STANDBY_ACTIVE and reset internal state."""
        msg = String()
        msg.data = "standby_active_cmd"
        self.active_command_pub.publish(msg)
        self._reset()

    def _reset(self):
        """Reset internal state to IDLE."""
        with self._state_lock:
            self._state = _State.IDLE
        self._current_target_pose = None
        self._landing_time_sec = None
        self._activation_time_sec = None


def main(args=None):
    rclpy.init(args=args)
    node = CatchFromBallButlerNode()

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    try:
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Shutting down.")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
