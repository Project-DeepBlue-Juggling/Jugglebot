#!/usr/bin/env python3
"""
hoop_sinker_node.py

This node computes a throw plan for the Jugglebot to throw a ball into a hoop.
It subscribes to the position of the hoop and the platform, and publishes
commands to move the platform and execute the throw.
"""
import math
import threading
import time
from dataclasses import dataclass, field
from typing import Optional, Tuple, List

import numpy as np
from tf_transformations import quaternion_from_euler
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time
from jugglebot_interfaces.msg import (
    PlatformPoseCommand, SetTrapTrajLimitsMessage, RobotState, ThrowDebug, RigidBodyPoses
)
from jugglebot_interfaces.srv import SetString, SetHandTrajCmd


# ─────────────────────────────────────────────────────────────
#  Constants
# ─────────────────────────────────────────────────────────────
G = 9.806
MAX_PLATFORM_R = 0.100
PLATFORM_Z = 0.165            # m (frame platform_start)
MAX_THROW_HEIGHT = 2.0        # m
MAX_RELEASE_SPEED = 5.8       # m s⁻¹
MAX_TILT = math.radians(15)   # DEG about the x and y axes
BALL_PEAK_ABOVE_CATCH_POS = 50e-3  # m (ball peak above catch pos)

# The distance from the platform COM to the throw position
RELEASE_OFFSET = 0.12455  # (m) {NOTE HARDCODED for now... Not sure how to best communicate this from the Teensy...}

# ─────────────────────────────────────────────────────────────
#  Data classes
# ─────────────────────────────────────────────────────────────
@dataclass
class ThrowPlan:
    feasible: bool = False
    platform_xy: Tuple[float, float] = (0., 0.)
    roll_pitch: Tuple[float, float] = (0., 0.)
    v0: Tuple[float, float, float] = (0., 0., 0.)
    tof: float = 0.
    throw_height: float = 0.
    horiz_range_mm: float = 0.
    cone_xyz: Tuple[float, float, float] = (0., 0., 0.)

# ─────────────────────────────────────────────────────────────
#  Utility functions
# ─────────────────────────────────────────────────────────────
def solve_orientation_launch(
        cone_xyz: np.ndarray,
        max_tilt_deg: float = 15.0,
        coarse_step_deg: float = 1.0,
        fine_window_deg: float = 1.0,
        fine_step_deg: float = 0.1,
        dz_apex_step: float = 0.05,
) -> Tuple[Optional[Tuple[float, float, np.ndarray, float]], float]:
    """
    Find roll, pitch, v0_vec, t_flight (min ‖v0‖) or None if impossible,
    and return elapsed computation time.

    * Two-pass grid search: coarse 1 °, then fine 0.1 ° in a ±1 ° window.
    * If **no** solution exists within ±max_tilt_deg using the
      nominal apex (cone_z + BALL_PEAK_ABOVE_CATCH_POS),
      the solver raises the apex in dz_apex_step increments until a
      feasible solution is found **or** the peak would exceed MAX_THROW_HEIGHT.
    * Speed capped at MAX_RELEASE_SPEED.
    """
    start_time = time.time()
    timeout_time = 0.5 # seconds
    xc, yc, zc = cone_xyz
    best = None
    best_speed = float('inf')

    # --------------------------------------------------------------
    # Helper that evaluates one roll/pitch pair for a *given* apex.
    # Returns (roll, pitch, v0_vec, tof) or None.
    # --------------------------------------------------------------
    def try_orientation(roll: float, pitch: float, min_z_peak: float):
        s_r, c_r = math.sin(roll), math.cos(roll)
        s_p, c_p = math.sin(pitch), math.cos(pitch)

        # body-Z in world
        u = np.array([s_p * c_r,
                    -s_r,
                    c_r * c_p])

        release = np.array([0., 0., PLATFORM_Z]) + u * RELEASE_OFFSET
        dx, dy = xc - release[0], yc - release[1]
        dz     = zc - release[2]
        Rh     = math.hypot(dx, dy)
        uh     = math.hypot(u[0], u[1])
        uz     = u[2]

        # ---------- bearing alignment check --------------------------------
        if Rh > 1e-6:
            v_hat = np.array([dx, dy]) / Rh
            if uh < 1e-6:                      # vertical launch can't reach lateral cone
                return None
            u_hat = np.array([u[0], u[1]]) / uh
            if np.dot(v_hat, u_hat) < math.cos(math.radians(1.0)):
                return None                    # heading off by >1°
        else:
            if uh > 0.01:                      # need (almost) vertical launch
                return None

        # ---------- speed from horizontal component (choose the safer axis) --
        # horizontal distance and unit vectors already computed
        if Rh < 1e-6:            # cone directly above/below the release
            return None          # (we reject vertical launches elsewhere)

        # analytic solution using Rh *and* uh
        denom = (uz / uh) * Rh - dz           # must be > 0
        if denom <= 0:
            return None

        s = math.sqrt( G * Rh * Rh / (2 * uh * uh * denom) )

        if s > MAX_RELEASE_SPEED:
            return None

        t = Rh / (uh * s)                     # horizontal flight time

        # ---------- apex constraint -----------------------------------------
        z_peak = release[2] + (s * uz) ** 2 / (2 * G)
        if z_peak < min_z_peak:
            return None

        return (roll, pitch, u * s, t), s

    # --------------------------------------------------------------
    # Two-pass search inside a loop that raises the apex only if
    # absolutely necessary.
    # --------------------------------------------------------------
    tilt_max = math.radians(max_tilt_deg)
    deg_coarse = np.deg2rad(np.arange(-max_tilt_deg,
                                      max_tilt_deg + 1e-6,
                                      coarse_step_deg))

    dz_extra = 0.0
    while True:
        if time.time() - start_time > timeout_time:
            break

        z_peak_nominal = zc + BALL_PEAK_ABOVE_CATCH_POS + dz_extra
        found_any = False

        # ------------ coarse pass 1° --------------------------------
        coarse_best = None
        coarse_speed = float('inf')
        for roll in deg_coarse:
            for pitch in deg_coarse:
                res = try_orientation(roll, pitch, z_peak_nominal)
                if res is not None:
                    found_any = True
                    sol, spd = res
                    if spd < coarse_speed:
                        coarse_best, coarse_speed = sol, spd

        if found_any:
            # ------------ fine pass 0.1° around the best coarse -----
            roll_c, pitch_c, _, _ = coarse_best
            def clamp_deg(val):
                return max(-tilt_max, min( tilt_max, val))

            fine_rolls = np.deg2rad(
                np.arange(math.degrees(clamp_deg(roll_c - math.radians(fine_window_deg))),
                          math.degrees(clamp_deg(roll_c + math.radians(fine_window_deg))) + 1e-6,
                          fine_step_deg))
            fine_pitchs = np.deg2rad(
                np.arange(math.degrees(clamp_deg(pitch_c - math.radians(fine_window_deg))),
                          math.degrees(clamp_deg(pitch_c + math.radians(fine_window_deg))) + 1e-6,
                          fine_step_deg))

            for roll in fine_rolls:
                for pitch in fine_pitchs:
                    res = try_orientation(roll, pitch, z_peak_nominal)
                    if res is not None:
                        sol, spd = res
                        if spd < best_speed:
                            best, best_speed = sol, spd
            # Success: break the dz_extra loop
            break

        # Nothing feasible at this apex → raise it and retry
        dz_extra += dz_apex_step
        if PLATFORM_Z + dz_extra > MAX_THROW_HEIGHT:
            break   # give up – no solution even with max height

    elapsed_time = time.time() - start_time
    return best, elapsed_time
# ─────────────────────────────────────────────────────────────
#  Node class
# ─────────────────────────────────────────────────────────────
class HoopSinkerNode(Node):
    # --------------------- init ------------------------------------------------
    def __init__(self):
        super().__init__('hoop_sinker_node')

        # Create a callback group to allow for concurrent execution of callbacks
        self.callback_group = ReentrantCallbackGroup()

        # ── comms
        self.platform_pub = self.create_publisher(PlatformPoseCommand, 'platform_pose_topic', 10)
        self.trap_limit_pub = self.create_publisher(SetTrapTrajLimitsMessage, 'set_leg_trap_traj_limits', 10)
        self.throw_debug_pub = self.create_publisher(ThrowDebug, 'throw_debug', 10)

        self.rigid_body_poses_sub = self.create_subscription(RigidBodyPoses, 'rigid_body_poses', self.rigid_body_poses_callback, 20,
                                                 callback_group=self.callback_group)
        self.control_sub = self.create_subscription(String, 'control_mode_topic', self.control_mode_callback, 10,
                                                     callback_group=self.callback_group)
        # Subscribe to /robot_state to know where the hand currently is
        self.robot_state_subscription = self.create_subscription(RobotState, 'robot_state', self.robot_state_callback, 10,
                                                                 callback_group=self.callback_group)


        # Initialize the services
        self.move_srv = self.create_service(Trigger, 'move_platform_for_throw', self.handle_move_request,
                                             callback_group=self.callback_group)
        self.throw_srv = self.create_service(Trigger, 'execute_throw', self.handle_throw_request,
                                             callback_group=self.callback_group)
        self.move_and_throw_srv = self.create_service(Trigger, 'move_and_execute_throw', self.handle_move_throw_request,
                                                     callback_group=self.callback_group)

        # Set up a service client to put the hand into the correct control mode
        self.hand_control_mode_client = self.create_client(SetString, 'set_hand_state',
                                                           callback_group=self.callback_group)
        
        # Set up a service client to send the hand trajectory
        self.hand_traj_client = self.create_client(SetHandTrajCmd, 'set_hand_traj_cmd',
                                                    callback_group=self.callback_group)

        # ── state
        self.enabled = False
        self.plan: ThrowPlan = ThrowPlan()
        self.platform_pose_mocap: Optional[PoseStamped] = None
        self.state_lock = threading.Lock()
        self.hand_state_lock = threading.Lock()
        self.last_known_hand_state = None

        self._last_plan_print = None # tuple of rounded fields

        self.throw_count = 0 # To keep track of how many throws have been made this session

        # defaults
        self.default_trap = dict(vel_limit=10., acc_limit=10., dec_limit=10.)

        self.get_logger().info('HoopSinkerNode initialised.')

    # --------------------- ROS callbacks --------------------------------------
    def control_mode_callback(self, msg: String):
        if msg.data == 'HOOP_SINKER' and not self.enabled:
            self.enabled = True
            self.get_logger().info('Hoop Sinker ENABLED')
            self.set_trap_traj_limits(**self.default_trap)
        elif msg.data != 'HOOP_SINKER' and self.enabled:
            self.get_logger().info('Hoop Sinker DISABLED')
            self.enabled = False

    def robot_state_callback(self, msg: RobotState):
        '''Callback function for the robot_state topic'''
        with self.hand_state_lock:
            # Extract the hand position from the message
            self.last_known_hand_state = msg.motor_states[6]

    def rigid_body_poses_callback(self, msg: RigidBodyPoses):
        """Callback for unified rigid body poses. Extracts Catching_Cone and Platform poses."""
        if not self.enabled:
            return

        # Extract relevant bodies from the message
        for body in msg.bodies:
            if body.name == "Catching_Cone":
                self._handle_cone_pose(body.pose)
            elif body.name == "Platform":
                with self.state_lock:
                    self.platform_pose_mocap = body.pose

    def _handle_cone_pose(self, pose: PoseStamped):
        """Process catching cone pose to update throw plan."""
        # mm → m
        cone_xyz = np.array([pose.pose.position.x,
                            pose.pose.position.y,
                            pose.pose.position.z]) / 1000.0

        # Check that we have a valid cone position (anything other than nan)
        if np.isnan(cone_xyz).any():
            with self.state_lock:
                self.plan.feasible = False
            self.get_logger().warn('Invalid cone position', throttle_duration_sec=1.0)
            return

        sol, calc_time = solve_orientation_launch(cone_xyz)
        if sol is None:
            with self.state_lock:
                self.plan.feasible = False
            self.get_logger().warn('No orientation-only throw plan', throttle_duration_sec=1.0)
            return
        
        # Log the time taken to solve the orientation
        self.get_logger().info(f'Solved orientation in {calc_time:.2f}s', throttle_duration_sec=1.0)

        roll, pitch, v0, tof = sol
        throw_h   = v0[2]**2 / (2*G)
        horiz_mm  = 1000.0 * math.hypot(cone_xyz[0], cone_xyz[1])

        with self.state_lock:
            self.plan = ThrowPlan(
                feasible=True,
                platform_xy=(0.0, 0.0),
                roll_pitch=(roll, pitch),
                v0=tuple(v0),
                tof=tof,
                throw_height=throw_h,
                horiz_range_mm=horiz_mm,
                cone_xyz=tuple(cone_xyz))

        # Conditional logging to avoid flooding
        sig_figs_to_round = 0
        plan_tuple = (
            round(math.degrees(roll), sig_figs_to_round),
            round(math.degrees(pitch), sig_figs_to_round),
            round(np.linalg.norm(v0), sig_figs_to_round),
            round(tof, sig_figs_to_round),
            round(throw_h, sig_figs_to_round),
            round(cone_xyz[0]*1000, sig_figs_to_round),
            round(cone_xyz[1]*1000, sig_figs_to_round)
        )

        if plan_tuple != self._last_plan_print:
            self.get_logger().info(
                f'Plan: roll={math.degrees(roll):.2f}°, pitch={math.degrees(pitch):.2f}°, '
                f'|v0|={np.linalg.norm(v0):.2f} m/s, TOF={tof:.2f}s, '
                f'h={throw_h:.2f}m, range=({cone_xyz[0]:.2f}, {cone_xyz[1]:.2f}, {cone_xyz[2]:.2f}) m',
                throttle_duration_sec=1.0)
            self._last_plan_print = plan_tuple

    # --------------------- Services -------------------------------------------
    def handle_move_request(self, _, res):
        if not self.enabled:
            res.success, res.message = False, 'Node not enabled.'
            return res
        if not self.plan.feasible:
            res.success, res.message = False, 'No feasible plan.'
            return res

        roll, pitch = self.plan.roll_pitch
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, 0.0)
        
        # Log the position and orientation (as roll and pitch)
        self.get_logger().info(
            f'Platform move: x={self.plan.platform_xy[0]:.2f} m, '
            f'y={self.plan.platform_xy[1]:.2f} m, '
            f'roll={math.degrees(roll):.2f}°, pitch={math.degrees(pitch):.2f}°, '
            f'quat: ({qx:.2f}, {qy:.2f}, {qz:.2f}, {qw:.2f})')

        cmd = PlatformPoseCommand()
        cmd.pose_stamped.header.frame_id = 'platform_start'
        cmd.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        cmd.pose_stamped.pose.position.x = self.plan.platform_xy[0] * 1000.0
        cmd.pose_stamped.pose.position.y = self.plan.platform_xy[1] * 1000.0
        cmd.pose_stamped.pose.position.z = PLATFORM_Z * 1000.0
        cmd.pose_stamped.pose.orientation.w = qw
        cmd.pose_stamped.pose.orientation.x = qx
        cmd.pose_stamped.pose.orientation.y = qy
        cmd.pose_stamped.pose.orientation.z = qz
        cmd.publisher = 'HOOP_SINKER'

        self.platform_pub.publish(cmd)
        res.success, res.message = True, 'Platform move sent.'
        return res

    def handle_throw_request(self, _, res):
        if not (self.enabled and self.plan.feasible):
            res.success, res.message = False, 'Not ready.'
            return res

        # Set the delay for when the throw should occur
        throw_delay = 2.0 # seconds

        # Get the throw velocity from the plan
        v0 = self.plan.v0
        # Calculate the magnitude of the throw velocity
        v0_mag = math.sqrt(v0[0]**2 + v0[1]**2 + v0[2]**2)

        # Publish the throw debug message
        debug_msg = ThrowDebug()
        now = self.get_clock().now()
        throw_time = now + Duration(seconds=throw_delay)
        debug_msg.stamp = throw_time.to_msg()
        debug_msg.roll_cmd = self.plan.roll_pitch[0]
        debug_msg.pitch_cmd = self.plan.roll_pitch[1]
        debug_msg.v0_mag_cmd = v0_mag
        self.throw_debug_pub.publish(debug_msg)

        # Create the service request
        req = SetHandTrajCmd.Request()
        req.event_delay = throw_delay
        req.event_vel = v0_mag
        req.traj_type = 0 # 0 = throw, 1 = catch, 2 = full

        self.future = self.hand_traj_client.call_async(req)
        
        # Once the service is called, we need to wait for the result
        self.future.add_done_callback(self.throw_request_callback)

        # Log that the throw was sent
        self.get_logger().info(f"Throw {self.throw_count} sent at {throw_time} s")
        self.throw_count += 1

        res.success = True
        res.message = f'Throw executed (TOF={self.plan.tof:.2f}s)'
        return res
    
    def handle_move_throw_request(self, _, res):
        if not self.enabled:
            res.success, res.message = False, 'Node not enabled.'
            return res
        if not self.plan.feasible:
            res.success, res.message = False, 'No feasible plan.'
            return res

        # Move the platform first
        move_res = self.handle_move_request(_, res)
        if not move_res.success:
            return move_res

        # Then execute the throw
        throw_res = self.handle_throw_request(_, res)
        if not throw_res.success:
            return throw_res

        res.success = True
        res.message = 'Move and throw executed.'
        return res
    
    def throw_request_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Throw executed successfully.')
            else:
                self.get_logger().error(f'Throw execution failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Exception while calling throw service: {e}')
        finally:
            # Reset the future to None to avoid memory leaks
            self.future = None

    # --------------------- helpers --------------------------------------------
    def set_trap_traj_limits(self, vel_limit, acc_limit, dec_limit):
        m = SetTrapTrajLimitsMessage()
        m.trap_vel_limit = float(vel_limit)
        m.trap_acc_limit = float(acc_limit)
        m.trap_dec_limit = float(dec_limit)
        self.trap_limit_pub.publish(m)

    # --------------------- Shutdown -------------------------------------------
    def on_shutdown(self):
        self.get_logger().info('HoopSinkerNode shutdown.')


# ─────────────────────────────────────────────────────────────
#  Main
# ─────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = HoopSinkerNode()

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
