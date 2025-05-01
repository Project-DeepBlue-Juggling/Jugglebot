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
import open3d as o3d
import quaternion
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time
from jugglebot_interfaces.msg import (
    PlatformPoseCommand, HandInputPosMsg, SetTrapTrajLimitsMessage, RobotState
)
from jugglebot_interfaces.srv import SetString
from .hand_trajectory_generator import HandTrajGenerator

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

# get the throw-pos offset once; no parameters are changed afterwards
_throw_sim = HandTrajGenerator()
RELEASE_OFFSET = _throw_sim.dist_from_plat_COM_to_throw_pos   # (m)

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
def clamp_to_circle(x: float, y: float, r_max: float) -> Tuple[float, float]:
    r = math.hypot(x, y)
    if r == 0.0 or r <= r_max:
        return x, y
    scale = r_max / r
    return x * scale, y * scale


def solve_orientation_launch(cone_xyz: np.ndarray) -> Optional[Tuple[float, float, np.ndarray, float]]:
    """
    Return (roll, pitch, v0, tof) or None if no solution.
    Platform is fixed at (0,0,PLATFORM_Z) with yaw = 0.
    Roll is about +X, pitch about +Y (ROS RPY convention).
    The projectile is launched along body +Z, starting
    RELEASE_OFFSET metres above the platform origin.
    Its apex must be exactly BALL_PEAK_ABOVE_CATCH_POS above the cone.
    """
    xc, yc, zc = cone_xyz

    best = None
    best_speed = float('inf')

    # Coarse grid of candidate orientations (1° steps)
    degs = np.deg2rad(np.arange(-15, 16, 1))
    for roll in degs:
        s_r, c_r = math.sin(roll), math.cos(roll)
        for pitch in degs:
            s_p, c_p = math.sin(pitch), math.cos(pitch)

            # Body +Z in world coordinates (direction cosine of tool frame Z)
            u = np.array([
                s_p * c_r,                    # x
               -s_r,                          # y
                c_p * c_r                     # z
            ])
            # release position in world
            release = np.array([0., 0., PLATFORM_Z]) + u * RELEASE_OFFSET

            # horizontal and vertical deltas
            dx, dy = xc - release[0], yc - release[1]
            dz     = zc - release[2]
            Rh     = math.hypot(dx, dy)

            # desired apex height
            z_peak_des = zc + BALL_PEAK_ABOVE_CATCH_POS
            if z_peak_des <= release[2]:          # apex below release → impossible
                continue

            # fix v0_z from apex condition:  z_peak = z_rel + v0z² / (2g)
            v0z = math.sqrt(2 * G * (z_peak_des - release[2]))

            # time of flight from quadratic  zc = z_rel + v0z t - ½ g t²
            a = 0.5 * G
            b = -v0z
            c = dz
            disc = b*b - 4*a*c
            if disc <= 0:
                continue
            t1 = (-b + math.sqrt(disc)) / (2*a)    # positive root
            if t1 <= 0:
                continue

            # horizontal speed needed
            vx = dx / t1
            vy = dy / t1
            speed = math.sqrt(vx*vx + vy*vy + v0z*v0z)
            if speed > MAX_RELEASE_SPEED:
                continue

            # choose minimal-speed solution
            if speed < best_speed:
                best_speed = speed
                best = (roll, pitch, np.array([vx, vy, v0z]), t1)

    return best

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
        self.hand_traj_pub = self.create_publisher(HandInputPosMsg, 'hand_trajectory', 10)
        self.trap_limit_pub = self.create_publisher(SetTrapTrajLimitsMessage, 'set_leg_trap_traj_limits', 10)

        self.cone_sub = self.create_subscription(PoseStamped, 'catching_cone_pose_mocap', self.cone_callback, 20,
                                                 callback_group=self.callback_group)
        self.control_sub = self.create_subscription(String, 'control_mode_topic', self.control_mode_callback, 10,
                                                     callback_group=self.callback_group)
        self.plat_pose_sub = self.create_subscription(PoseStamped, 'platform_pose_mocap', self.platform_pose_callback, 50,
                                                      callback_group=self.callback_group)
        # Subscribe to /robot_state to know where the hand currently is
        self.robot_state_subscription = self.create_subscription(RobotState, 'robot_state', self.robot_state_callback, 10,
                                                                 callback_group=self.callback_group)

        self.move_srv = self.create_service(Trigger, 'move_platform_for_throw', self.handle_move_request,
                                             callback_group=self.callback_group)
        self.throw_srv = self.create_service(Trigger, 'execute_throw', self.handle_throw_request,
                                             callback_group=self.callback_group)

        # Set up a service client to put the hand into the correct control mode
        self.hand_control_mode_client = self.create_client(SetString, 'set_hand_state',
                                                           callback_group=self.callback_group)

        # ── state
        self.enabled = False
        self.plan: ThrowPlan = ThrowPlan()
        self.platform_pose_mocap: Optional[PoseStamped] = None
        self.state_lock = threading.Lock()
        self.hand_state_lock = threading.Lock()
        self.last_known_hand_state = None

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

    def cone_callback(self, msg: PoseStamped):
        if not self.enabled:
            return

        # mm → m
        cone_xyz = np.array([msg.pose.position.x,
                            msg.pose.position.y,
                            msg.pose.position.z]) / 1000.0

        sol = solve_orientation_launch(cone_xyz)
        if sol is None:
            with self.state_lock:
                self.plan.feasible = False
            self.get_logger().warn('No orientation-only throw plan', throttle_duration_sec=1.0)
            return

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

        self.get_logger().info(
            f'Plan: roll={math.degrees(roll):.1f}°, pitch={math.degrees(pitch):.1f}°, '
            f'|v0|={np.linalg.norm(v0):.2f} m/s, TOF={tof:.2f}s '
            f'h={throw_h:.2f}m, range=({cone_xyz[0]*1000:.2f}, {cone_xyz[1]*1000:.2f}) mm',
            throttle_duration_sec=1.0)

    def platform_pose_callback(self, msg: PoseStamped):
        if not self.enabled:
            return
        with self.state_lock:
            self.platform_pose_mocap = msg

    # --------------------- Services -------------------------------------------
    def handle_move_request(self, _, res):
        if not self.enabled:
            res.success, res.message = False, 'Node not enabled.'
            return res
        if not self.plan.feasible:
            res.success, res.message = False, 'No feasible plan.'
            return res

        roll, pitch = self.plan.roll_pitch
        q = quaternion.as_float_array(
            quaternion.from_euler_angles(roll, pitch, 0.0))
        
        # Log the position and orientation (as roll and pitch)
        self.get_logger().info(
            f'Platform move: x={self.plan.platform_xy[0]:.2f} m, '
            f'y={self.plan.platform_xy[1]:.2f} m, '
            f'roll={roll:.2f} rad, pitch={pitch:.2f} rad')

        cmd = PlatformPoseCommand()
        cmd.pose_stamped.header.frame_id = 'platform_start'
        cmd.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        cmd.pose_stamped.pose.position.x = self.plan.platform_xy[0] * 1000.0
        cmd.pose_stamped.pose.position.y = self.plan.platform_xy[1] * 1000.0
        cmd.pose_stamped.pose.position.z = PLATFORM_Z * 1000.0
        cmd.pose_stamped.pose.orientation.w = q[0]
        cmd.pose_stamped.pose.orientation.x = q[1]
        cmd.pose_stamped.pose.orientation.y = q[2]
        cmd.pose_stamped.pose.orientation.z = q[3]
        cmd.publisher = 'HOOP_SINKER'

        self.platform_pub.publish(cmd)
        res.success, res.message = True, 'Platform move sent.'
        return res

    def handle_throw_request(self, _, res):
        if not (self.enabled and self.plan.feasible):
            res.success, res.message = False, 'Not ready.'
            return res

        # Smoothly move the hand to the starting position (0 revs) (0.1 is chosen instead because hand doesn't sit still at 0 revs)
        result_outcome, result_message = self.smooth_move_to_target(0.1, duration=2.0)
        if not result_outcome:
            res.success = False
            res.message = result_message
            return res
        self.get_logger().info(result_message)
        time.sleep(0.5)

        ht = HandTrajGenerator(
            throw_height_m=self.plan.throw_height,
            throw_range_mm=self.plan.horiz_range_mm)
        t_cmd, x, v, tor = ht.get_full_trajectory()

        # Convert time_cmd to builtin_interfaces/Time 
        time_cmd_ros = [
            (self.get_clock().now() + Duration(seconds=int(t), nanoseconds=int((t % 1) * 1e9))).to_msg() 
            for t in t_cmd
            ]

        # Publish the full trajectory
        self.publish_hand_trajectory(time_cmd_ros, x, v, tor)

        res.success = True
        res.message = f'Throw executed (TOF={self.plan.tof:.2f}s)'
        return res

    # --------------------- helpers --------------------------------------------
    def set_trap_traj_limits(self, vel_limit, acc_limit, dec_limit):
        m = SetTrapTrajLimitsMessage()
        m.trap_vel_limit = float(vel_limit)
        m.trap_acc_limit = float(acc_limit)
        m.trap_dec_limit = float(dec_limit)
        self.trap_limit_pub.publish(m)

    def publish_hand_trajectory(self, time_cmd, pos, vel, tor, timeout=5.0):
        '''
        Publish the generated trajectory after ensuring the hand is in CLOSED_LOOP_CONTROL mode.

        Parameters:
        - time_cmd: Commanded time sequence for the trajectory, as ROS2 time (builtin_interfaces/Time)
        - pos: Position commands
        - vel: Velocity commands
        - tor: Torque commands
        - timeout: Maximum time to wait for the hand to enter CLOSED_LOOP_CONTROL mode (in seconds)
        '''
        try:
            # Check if the hand is already in CLOSED_LOOP_CONTROL mode
            if not self.last_known_hand_state or self.last_known_hand_state.current_state != 8:
                self.get_logger().info("Hand not in CLOSED_LOOP_CONTROL. Requesting state change...")
                
                # Create and send the service request to change the hand state
                request = SetString.Request()
                request.data = "CLOSED_LOOP_CONTROL"
                # Wait for the service to be available
                self.hand_control_mode_client.wait_for_service(timeout_sec=1.0)
                self.hand_control_mode_client.call_async(request)
                
                # Wait until the hand is in CLOSED_LOOP_CONTROL mode or until timeout
                start_time = self.get_clock().now()
                while True:
                    with self.hand_state_lock:
                        current_state = self.last_known_hand_state.current_state if self.last_known_hand_state else None
                    
                    if current_state == 8:
                        self.get_logger().info("Hand is now in CLOSED_LOOP_CONTROL mode.")
                        break
                    
                    if (self.get_clock().now() - start_time).nanoseconds / 1e9 > timeout:
                        self.get_logger().error("Timeout waiting for hand to enter CLOSED_LOOP_CONTROL mode.")
                        return
                    
                    time.sleep(0.1)

            # Publish the full trajectory
            msg = HandInputPosMsg()
            msg.time_cmd = time_cmd
            msg.input_pos = pos
            msg.vel_ff = vel
            msg.torque_ff = tor
            self.hand_traj_pub.publish(msg)
            self.get_logger().info(f"Published generated trajectory. Number of points: {len(pos)}")

        except Exception as e:
            self.get_logger().error(f"Error occurred while publishing trajectory: {e}")

    def smooth_move_to_target(self, target_pos, duration=2.0):
        '''Move the hand smoothly to a target position'''
        result_outcome = None
        result_message = None

        try:
            # Log this
            self.get_logger().info(f"Moving hand to target position {target_pos} revs.")

            with self.hand_state_lock:
                # Check that we know where the hand currently is and that it's stationary (within a tolerance)
                vel_tol = 0.2 # {rev/s}
                if self.last_known_hand_state is None:
                    result_outcome = False
                    result_message = "Hand state is unknown. Please wait for the robot to report its position."
                    return result_outcome, result_message
                
                elif np.abs(self.last_known_hand_state.vel_estimate) > vel_tol:
                    result_outcome = False
                    result_message = f"Hand is not stationary. Please wait for the hand to stop moving. Current velocity: {self.last_known_hand_state.vel_estimate:.2f} rev/s"
                    return result_outcome, result_message
                    
            # Check that we've gotten a valid target position
            if target_pos < 0:
                result_outcome = False
                result_message = f"Target position must be non-negative. Target position received: {target_pos}"
                return result_outcome, result_message
            
            elif target_pos > 11.0:
                result_outcome = False
                result_message = f"Target position must be less than 11.0 revs. Target position received: {target_pos}"
                return result_outcome, result_message
                
            # Get the trajectory
            time_cmd, pos, vel, tor = self.get_smooth_move_traj(target_pos, duration=duration)

            # Publish the trajectory for can_interface_node to follow
            self.publish_hand_trajectory(time_cmd, pos, vel, tor)

            # Wait for the hand to reach the target position
            timeout = duration * 2.0
            start_time = self.get_clock().now()
            while True:
                with self.hand_state_lock:
                    current_pos = self.last_known_hand_state.pos_estimate if self.last_known_hand_state else None
                
                if current_pos is not None and np.abs(current_pos - target_pos) < 0.05:
                    break
                
                if (self.get_clock().now() - start_time).nanoseconds / 1e9 > timeout:
                    return False, "Timeout waiting for hand to reach target position."
                

            result_outcome = True
            result_message = f"Smoothly moving hand to target position {target_pos:.2f} revs. Trajectory published."
            return result_outcome, result_message

        except Exception as e:
            result_outcome = False
            result_message = f"Exception occurred: {e}"
            return result_outcome, result_message

    def get_smooth_move_traj(self, target_pos, duration=2.0):
        '''Get the trajectory for smoothly moving the hand to a target position'''
        with self.hand_state_lock:
            # Get the current position
            current_pos = self.last_known_hand_state.pos_estimate

        hand_traj_gen = HandTrajGenerator()

        # Get the trajectory
        time_cmd, pos, vel, acc, tor = hand_traj_gen.get_smooth_move_trajectory(start_pos=current_pos, 
                                                                                    target_pos=target_pos, 
                                                                                    duration=duration)
        
        # Convert time_cmd to builtin_interfaces/Time 
        time_cmd_ros = [
            (self.get_clock().now() + Duration(seconds=int(t), nanoseconds=int((t % 1) * 1e9))).to_msg() 
            for t in time_cmd
            ]

        return time_cmd_ros, pos, vel, tor

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
