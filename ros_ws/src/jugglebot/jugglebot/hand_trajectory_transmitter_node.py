'''This node will read the requested trajectory from the appropriate file and publish the trajectory to the topic /hand_trajectory'''

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from jugglebot_interfaces.srv import SendHandTrajectory
from jugglebot_interfaces.msg import HandTrajectoryPointMessage, LegsTargetReachedMessage, SetTrapTrajLimitsMessage
import numpy as np
np.bool = np.bool_ # Fixes issue on importing pandas
import pandas as pd
import time
import os
import can
import struct
import quaternion
from .hand_trajectory_generator import HandTrajGenerator

class HandTrajectoryTransmitter(Node):
    OPCODE_WRITE = 0x01  # For writing arbitrary parameters to the ODrive
    AXIS_ID = 6 # The CAN ID of the hand motor
    HAND_SPOOL_EFFECTIVE_RADIUS = 5.134 # {mm} Measured experimentally. (stroke / [revs_to_move_stroke * 2 * pi])
    LINEAR_GAIN = 1000 / (np.pi * HAND_SPOOL_EFFECTIVE_RADIUS * 2) # {rev/m}

    ARBITRARY_PARAMETER_IDS = {
        "input_pos"    : 383,
        "input_vel"    : 384,
        "input_torque" : 385,
    }

    def __init__(self, bus_name='can0', bitrate=1000000, bus_type='socketcan'):
        super().__init__('hand_trajectory_transmitter')

        # Set up a service to trigger closing the node
        self.end_session_service = self.create_service(Trigger, 'end_session', self.end_session)

        # Find the package directory
        self.pkg_dir = get_package_share_directory('jugglebot')

        # Initialize the hand trajectory generator
        self.hand_traj_gen = HandTrajGenerator()

        # Initialize the parameters for the CAN bus that will be used by setup_can_bus to initialise the bus itself
        self._can_bus_name = bus_name
        self._can_bitrate = bitrate
        self._can_bus_type = bus_type
        self.bus = None

        # Initialize parameters related to the throw
        self.samples_to_wait_before_moving_to_catch_pose = 10 # Num samples to wait after hand has started decelerating before moving to catch pose
        self.arm_x_span = 240.0 # {mm} Total span between catch and throw. ASSUMES SYMMETRIC THROW ABOUT Y-Z PLANE
        self.columns_x_span = 100.0 # {mm} Total span between the two columns. ASSUMES SYMMETRIC THROWS ABOUT Y-Z PLANE
        self.throw_duration = None # {s} Duration of the throw
        self.plat_height = 170.0 # {mm} Height of the platform when throwing/catching (above its lowest position)
        self.throw_pose = None
        self.catch_pose = None
        self.moving_to_catch = False # Has the platform already been commanded to move to the catch pose (in this throw)?

        self.g = 9.81 # {m/s^2} Acceleration due to gravity

        # Set up a publisher to publish the hand trajectory
        self.hand_trajectory_publisher = self.create_publisher(HandTrajectoryPointMessage, 'hand_trajectory', 10)

        # Set up publisher to publish the platform pose
        self.platform_pose_publisher = self.create_publisher(PoseStamped, 'platform_pose_topic', 10)

        # Set up a subscriber to receive the platform pose
        self.platform_pose_subscription = self.create_subscription(PoseStamped, 'platform_pose_topic', self.platform_pose_callback, 10)
        self.plat_current_pos = [0.0, 0.0] # x, y pos of the plat COM in mm (don't need z as we assume throw and catch occur at same height)

        # Set up a service to read the trajectory from the file and publish it to the topic
        self.send_trajectory_service = self.create_service(SendHandTrajectory,
                                                           'send_trajectory',
                                                           self.send_trajectory,
                                                           callback_group=MutuallyExclusiveCallbackGroup())
        
        # Set up a service to throw continuously
        self.throw_continuous_service = self.create_service(SendHandTrajectory,
                                                            'throw_continuous',
                                                            self.throw_continuous,
                                                            callback_group=MutuallyExclusiveCallbackGroup())

        # Set up a service to throw two columns
        self.throw_two_columns_service = self.create_service(SendHandTrajectory,
                                                            'throw_columns',
                                                            self.throw_two_columns,
                                                            callback_group=MutuallyExclusiveCallbackGroup())

        # Set up a service to throw one ball to a nominated location
        self.throw_to_service = self.create_service(SendHandTrajectory,
                                                    'throw_to',
                                                    self.throw_to,
                                                    callback_group=MutuallyExclusiveCallbackGroup())
        
        # Set up a publisher to the set_trap_traj_limits topic
        self.set_trap_traj_limits_publisher = self.create_publisher(SetTrapTrajLimitsMessage, 'leg_trap_traj_limits', 10)
        
        # Subscribe to the legs_target_reached topic to check if the platform has reached the calibration pose
        self.legs_target_reached = [False] * 6
        self.legs_target_reached_subscription = self.create_subscription(LegsTargetReachedMessage,
                                                                         'target_reached',
                                                                         self.legs_target_reached_callback,
                                                                         10)


        # Initialize a service client to prepare the hand motor for throwing
        self.prepare_hand_for_throw_client = self.create_client(Trigger, 'prepare_hand_for_throw')

        # Initialize a flag indicating whether the hand motor is ready to throw
        self.is_hand_ready_to_throw = False

        # Set up the CAN bus
        self.setup_can_bus()

    def sleep(self, duration_sec, get_now=time.perf_counter):
        '''Sleep using Jean-Marc's method'''

        now = get_now()
        end = now + duration_sec
        while now < end:
            now = get_now()

    def legs_target_reached_callback(self, msg):
        '''Handles the target_reached message'''

        self.legs_target_reached[0] = msg.leg0_has_arrived
        self.legs_target_reached[1] = msg.leg1_has_arrived
        self.legs_target_reached[2] = msg.leg2_has_arrived
        self.legs_target_reached[3] = msg.leg3_has_arrived
        self.legs_target_reached[4] = msg.leg4_has_arrived
        self.legs_target_reached[5] = msg.leg5_has_arrived

    #########################################################################################################
    #                                               CAN Bus                                                 #
    #########################################################################################################

    def setup_can_bus(self):
        self.bus = can.Bus(channel=self._can_bus_name, bustype=self._can_bus_type, bitrate=self._can_bitrate)

    def send_message(self, param_name, param_value):
        '''Send an arbitrary parameter to the ODrive'''
        
        # First make sure the bus is initialized
        if not self.bus:
            # If the bus hasn't been initialized, return
            self.get_logger().warn("CAN bus not initialized! Message not sent.")
            return

        # Get the endpoint ID for the parameter
        endpoint_id = self.ARBITRARY_PARAMETER_IDS[param_name]

        # Pack the data into the correct format
        data = struct.pack('<BHBf', self.OPCODE_WRITE, endpoint_id, 0, param_value)

        # Get the hex code for the message being sent
        command_id = 0x04  # Command ID for RxSdo 

        # Create the CAN message
        arbitration_id = (self.AXIS_ID << 5) | command_id

        msg = can.Message(arbitration_id=arbitration_id, dlc=8, is_extended_id=False, data=data, is_remote_frame=False)

        try:
            self.bus.send(msg)
            # self.get_logger().debug(f"CAN message for {param_name} sent to axisID {self.AXIS_ID}")
            # self.get_logger().info(f"msg: {msg} for {param_name} with value {param_value}")
        except Exception as e:
            # Log that the message couldn't be sent
            self.get_logger().warn(f"CAN message for {param_name} NOT sent to axisID {self.AXIS_ID}! Error: {e}")

    def send_trap_traj_limits(self, vel_limit, acc_limit, dec_limit):
        '''Send the trapezoidal trajectory limits to the ODrive'''
        # Create the message
        msg = SetTrapTrajLimitsMessage()
        msg.trap_vel_limit = vel_limit
        msg.trap_acc_limit = acc_limit
        msg.trap_dec_limit = dec_limit

        # Publish the message
        self.set_trap_traj_limits_publisher.publish(msg)

    #########################################################################################################
    #                                            Platform Pose                                              #
    #########################################################################################################

    def send_platform_pose(self, pose):
        '''Publish the platform pose to the platform_pose_topic'''
        # Initialize and populate a PoseStamped message
        msg = PoseStamped()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.pose.position.x = pose[0]
        msg.pose.position.y = pose[1]
        msg.pose.position.z = pose[2]
        
        msg.pose.orientation.x = pose[3]
        msg.pose.orientation.y = pose[4]
        msg.pose.orientation.z = pose[5]
        msg.pose.orientation.w = pose[6]

        self.platform_pose_publisher.publish(msg)

    def platform_pose_callback(self, msg):
        '''Callback to receive the current platform pose.
        Assumes the platform has stopped moving. ie. won't be correct until the platform has arrived.'''

        self.plat_current_pos = [msg.pose.position.x, msg.pose.position.y]

    #########################################################################################################
    #                                           Helper Functions                                            #
    #########################################################################################################

    def move_platform_to_pose_and_await_arrival(self, pose):
        '''Move the platform to the desired pose and wait for it to arrive'''
        # Send the platform pose to the platform
        self.send_platform_pose(pose)

        # Reset the target_reached flags, since these only update at 10 Hz (set by timer in can_bus_handler_node)
        self.legs_target_reached = [False] * 6

        # Wait for the platform to arrive at the calibration pose
        while not all (self.legs_target_reached):
            self.get_logger().info(f'Waiting for platform to reach throw pose. Status: {self.legs_target_reached}',
                                   throttle_duration_sec=1.0)
            time.sleep(0.1)

    def prepare_hand_for_throw(self):
        '''Call the service to put the hand in closed loop control will appropriate input and control modes'''
        request = Trigger.Request()
        future = self.prepare_hand_for_throw_client.call_async(request)
        future.add_done_callback(self.prepare_hand_for_throw_callback)

    def calculate_throw_catch_pose(self):
        '''Based on the chosen arm x span and throw duration, calculate the catch and throw poses
        Assumes platform z height is constant'''

        # Calculate the throw velocity
        throw_velocity = self.throw_duration * self.g / 2

        # Convert throw velocity to mm/s
        throw_velocity *= 1000

        # Calculate the throw angle {rad}
        throw_angle = np.arcsin(self.arm_x_span / (throw_velocity * self.throw_duration))

        # Convert the orientation part of the pose to a quaternion
        throw_q = quaternion.from_euler_angles(0, -throw_angle, 0)
        catch_q = quaternion.from_euler_angles(0, throw_angle, 0)

        # Construct the throw pose
        self.throw_pose = np.array([self.arm_x_span / 2, 0, self.plat_height,
                                    throw_q.x, throw_q.y, throw_q.z, throw_q.w])

        # Construct the catch pose
        self.catch_pose = np.array([-self.arm_x_span / 2, 0, self.plat_height,
                                    catch_q.x, catch_q.y, catch_q.z, catch_q.w])


    #########################################################################################################
    #                                       Trajectory (File-Based)                                         #
    #########################################################################################################


    def get_traj_file_path(self, sample_rate):
        '''Get the path to the trajectory file based on the sample rate'''
        # Construct the path to the trajectory file
        if sample_rate == 5:
            traj_file_path = os.path.join(self.pkg_dir, 'resources', '0.5s_500Hz_throw.csv')
            self.throw_duration = 0.5

        elif sample_rate == 6:
            traj_file_path = os.path.join(self.pkg_dir, 'resources', '0.6s_500Hz_throw.csv')
            self.throw_duration = 0.6

        elif sample_rate == 7:
            traj_file_path = os.path.join(self.pkg_dir, 'resources', '0.7s_500Hz_throw.csv')
            self.throw_duration = 0.7

        elif sample_rate == 8:
            traj_file_path = os.path.join(self.pkg_dir, 'resources', '0.8s_500Hz_throw.csv')
            self.throw_duration = 0.8

        elif sample_rate == 9:
            traj_file_path = os.path.join(self.pkg_dir, 'resources', '0.9s_500Hz_throw.csv')
            self.throw_duration = 0.9

        elif sample_rate == 10:
            traj_file_path = os.path.join(self.pkg_dir, 'resources', '1.0s_500Hz_throw.csv')
            self.throw_duration = 1.0

        else:
            traj_file_path = None
            self.get_logger().error(f"Sample rate {sample_rate} not supported. Trajectory file not found.")

        return traj_file_path

    async def send_trajectory(self, request, response):
        '''First reads the entire contents of the trajectory file, then publishes each point to the topic'''
        
        # Get the path to the trajectory file
        traj_file_path = self.get_traj_file_path(request.sample_rate)

        if traj_file_path is None:
            response.success = False
            response.message = "Trajectory file not found."
            return response

        # Check if the hand is ready to throw
        if not self.is_hand_ready_to_throw:
            self.prepare_hand_for_throw()
            response.success = False
            response.message = "Hand not ready to throw. Preparing hand for throw..."
            return response

        # Calculate the throw and catch poses
        self.calculate_throw_catch_pose()

        # # Move the platform to the throw pose and await arrival
        # self.move_platform_to_pose_and_await_arrival(self.throw_pose)

        # Reset the 'moving_to_catch' flag
        self.moving_to_catch = False

        # Read the trajectory from the file
        trajectory = pd.read_csv(traj_file_path)

        # Extract the relevant data from the trajectory
        time_cmd = trajectory.iloc[0].values
        pos = trajectory.iloc[1].values * self.LINEAR_GAIN
        vel = trajectory.iloc[2].values * self.LINEAR_GAIN
        tor = trajectory.iloc[4].values

        start_time = time.perf_counter()

        # Publish each point in the trajectory
        for i in range(len(time_cmd)):
            msg = HandTrajectoryPointMessage()
            msg.first_command = False
            msg.last_command = False
            
            if i == 0:
                msg.first_command = True
            elif i == len(time_cmd) - 1:
                msg.last_command = True

            msg.stamp = self.get_clock().now().to_msg() # Timestamp of when this message was sent
            msg.time = time_cmd[i]
            msg.pos = pos[i]
            msg.vel = vel[i]
            msg.tor = tor[i]
            self.hand_trajectory_publisher.publish(msg)

            '''If the hand has started decelerating (with positive velocity), the ball has been thrown 
            and we're ready to move to the catch pose'''
            # if self.moving_to_catch == False:
            #     if vel[i] > 0 and vel[i] < vel[i - self.samples_to_wait_before_moving_to_catch_pose]:
            #         self.send_platform_pose(self.catch_pose)
            #         self.moving_to_catch = True

            time_since_start = time.perf_counter() - start_time
            time_to_wait = time_cmd[i] - time_since_start
            if time_to_wait > 0:
                self.sleep(time_to_wait)

            # Send the data to the ODrive
            self.send_message("input_pos", pos[i])
            self.send_message("input_vel", vel[i])
            self.send_message("input_torque", tor[i])

        end_time = time.perf_counter()
        self.get_logger().info(f"Trajectory took {end_time - start_time} seconds to send.")

        response.success = True
        response.message = "Trajectory sent successfully."
        return response

    def throw_and_move_to_catch_one_ball(self, trajectory_file_path, catch_pose):
        # Read the trajectory from the file
        trajectory = pd.read_csv(trajectory_file_path)

        # Extract the relevant data from the trajectory
        time_cmd = trajectory.iloc[0].values
        pos = trajectory.iloc[1].values * self.LINEAR_GAIN
        vel = trajectory.iloc[2].values * self.LINEAR_GAIN
        tor = trajectory.iloc[4].values

        # Reset the 'moving_to_catch' flag
        self.moving_to_catch = False

        start_time = time.perf_counter()

        # Publish each point in the trajectory
        for i in range(len(time_cmd)):
            # msg = HandTrajectoryPointMessage() # Commented out for SPEED (temporarily)
            # msg.first_command = False
            # msg.last_command = False
            
            # if i == 0:
            #     msg.first_command = True
            # elif i == len(time_cmd) - 1:
            #     msg.last_command = True

            # msg.stamp = self.get_clock().now().to_msg() # Timestamp of when this message was sent
            # msg.time = time_cmd[i]
            # msg.pos = pos[i]
            # msg.vel = vel[i]
            # msg.tor = tor[i]
            # self.hand_trajectory_publisher.publish(msg)

            '''If the hand has started decelerating (with positive velocity), the ball has been thrown 
            and we're ready to move to the catch pose'''
            if self.moving_to_catch == False:
                if vel[i] > 0 and vel[i] < vel[i - self.samples_to_wait_before_moving_to_catch_pose]:
                    self.send_platform_pose(catch_pose)
                    self.moving_to_catch = True

            time_since_start = time.perf_counter() - start_time
            time_to_wait = time_cmd[i] - time_since_start
            if time_to_wait > 0:
                self.sleep(time_to_wait)

            # Send the data to the ODrive
            self.send_message("input_pos", pos[i])
            self.send_message("input_vel", vel[i])
            self.send_message("input_torque", tor[i])

    def follow_trajectory_from_file(self, trajectory_file_path):
        '''Move the hand along the trajectory specified in the file'''
        # Read the trajectory from the file
        trajectory = pd.read_csv(trajectory_file_path)

        # Extract the relevant data from the trajectory
        time_cmd = trajectory.iloc[0].values
        pos = trajectory.iloc[1].values * self.LINEAR_GAIN
        vel = trajectory.iloc[2].values * self.LINEAR_GAIN
        tor = trajectory.iloc[4].values

        start_time = time.perf_counter()

        # Publish each point in the trajectory
        for i in range(len(time_cmd)):
            time_since_start = time.perf_counter() - start_time
            time_to_wait = time_cmd[i] - time_since_start

            if time_to_wait > 0:
                self.sleep(time_to_wait)

            # Send the data to the ODrive
            self.send_message("input_pos", pos[i])
            self.send_message("input_vel", vel[i])
            self.send_message("input_torque", tor[i])

    #########################################################################################################
    #                                   Trajectory (Generated on Demand)                                    #
    #########################################################################################################

    def follow_generated_trajectory(self, time_cmd, pos, vel, tor, pose_to_move_to_after_throwing=None):
        '''Follow the trajectory generated by the hand trajectory generator
        If pos_to_move_to_after_throwing is provided, move to that position after the throw'''

        if pose_to_move_to_after_throwing is not None:
            moving_after_throw = True
        else:
            moving_after_throw = False

        # Set throw_time to None for now. If the trajectory being followed is a throw, this will update with the actual throw time
        throw_time = None 

        # Reset the 'moving_to_catch' flag
        self.moving_to_catch = False
        start_time = time.perf_counter()
        # Publish each point in the trajectory
        for i in range(len(time_cmd)):
            msg = HandTrajectoryPointMessage()
            msg.first_command = False
            msg.last_command = False
            
            if i == 0:
                msg.first_command = True
            elif i == len(time_cmd) - 1:
                msg.last_command = True

            msg.stamp = self.get_clock().now().to_msg() # Timestamp of when this message was sent
            msg.time = time_cmd[i]
            msg.pos = pos[i]
            msg.vel = vel[i]
            msg.tor = tor[i]
            self.hand_trajectory_publisher.publish(msg)

            '''If the hand has started decelerating (with positive velocity), the ball has been thrown 
            and we're ready to move to the catch pose'''
            if self.moving_to_catch == False:
                if vel[i] > 0 and vel[i] < vel[i - self.samples_to_wait_before_moving_to_catch_pose]:
                    if moving_after_throw:
                        self.send_platform_pose(pose_to_move_to_after_throwing)
                        pass

                    self.moving_to_catch = True
                    throw_time = time.perf_counter()

            time_since_start = time.perf_counter() - start_time
            time_to_wait = time_cmd[i] - time_since_start
            if time_to_wait > 0:
                self.sleep(time_to_wait)

            # Send the data to the ODrive
            self.send_message("input_pos", pos[i])
            self.send_message("input_vel", vel[i])
            self.send_message("input_torque", tor[i])

        end_time = time.perf_counter()
        # self.get_logger().info(f"Trajectory took {end_time - start_time} seconds to send.")

        return throw_time

    def get_traj(self, throw_range_mm=0.0, throw_height_m=0.6, linear_gain_factor=1.0):
        start_time = time.perf_counter()

        # Note that throw_range here is range that the platform covers, not the hand! (Hand/ball range is dealt with by the generator)
        self.hand_traj_gen.set_throw_parameters(throw_height=throw_height_m, throw_range=throw_range_mm)

        # Get the trajectory
        time_cmd, pos, vel, tor = self.hand_traj_gen.get_full_trajectory()

        # Multiply the position and velocity by the linear gain to convert from m to rev
        pos *= self.LINEAR_GAIN * linear_gain_factor
        vel *= self.LINEAR_GAIN * linear_gain_factor

        end_time = time.perf_counter()
        
        # Log the time taken to generate the trajectory in milliseconds
        self.get_logger().info(f"Trajectory generation took {(end_time - start_time) * 1000} ms.")

        return time_cmd, pos, vel, tor

    def calc_throw_catch_angle_quaternion(self, throw_angle, throw_pos, catch_pos):
        '''Given the throw angle (from vertical), throw position and catch position,
        calculate the quaternion that represents the throw angle'''

        # Log the throw and catch positions
        self.get_logger().info(f"Throw position: {throw_pos}, Catch position: {catch_pos}")

        # If the throw is straight up (throw angle = 0), the quaternion is the identity quaternion
        if throw_angle == 0:
            throw_quat = quaternion.one
            catch_quat = quaternion.one
        
        else:
            # Extract positions
            x0, y0 = throw_pos
            x1, y1 = catch_pos
            z0, z1 = 0, 0 # Catch and throw positions are assumed to be at the same height

            # Calculate the displacement vector
            dx = x1 - x0
            dy = y1 - y0
            dz = z1 - z0
            displacement = np.array([dx, dy, dz])

            # Calculate the magnitude of the displacement vector
            disp_mag = np.linalg.norm(displacement)

            # Calculate the throw velocity components
            v_x = disp_mag * np.sin(throw_angle) * (dx / disp_mag)
            v_y = disp_mag * np.sin(throw_angle) * (dy / disp_mag)
            v_z = disp_mag * np.cos(throw_angle)
            v = np.array([v_x, v_y, v_z])

            # Normalize the throw velocity vector to get the normal vector
            n = v / np.linalg.norm(v)

            # Extract the components of the normal vector
            n_x, n_y, n_z = n
            
            # Calculate the axis of rotation
            a_x = -n_y / np.sqrt(n_x**2 + n_y**2)
            a_y = n_x / np.sqrt(n_x**2 + n_y**2)
            a_z = 0

            # Calculate the angle of rotation
            angle = np.arccos(n_z)

            # Calculate the quaternion
            q_w = np.cos(angle / 2)
            q_x = a_x * np.sin(angle / 2)
            q_y = a_y * np.sin(angle / 2)
            q_z = a_z * np.sin(angle / 2)

            throw_quat = quaternion.quaternion(q_w, q_x, q_y, q_z)
            catch_quat = throw_quat.inverse()

        return throw_quat, catch_quat

    async def throw_two_columns(self, request, response):
        '''Juggle two balls in a columns pattern'''
        # Check if the hand is ready to throw
        if not self.is_hand_ready_to_throw:
            self.prepare_hand_for_throw()
            response.success = False
            response.message = "Hand not ready to throw. Preparing hand for throw..."
            return response
        
        # Handle the throw height input
        throw_height = request.throw_height
        if throw_height <= 0:
            throw_height = 0.7

        # Handle the first throw height input
        first_throw_height = request.first_throw_height
        if first_throw_height <= 0:
            first_throw_height = throw_height

        # Handle the "time_to_wait_after_throwing" input (if provided).
        if request.time_to_wait_after_throwing > 0:
            time_to_sleep_after_throwing = request.time_to_wait_after_throwing
        else:
            # Default to 0.003
            time_to_sleep_after_throwing = 0.003

        # Handle the trap traj limits input 
        self.send_trap_traj_limits(vel_limit=request.leg_properties[0], 
                                   acc_limit=request.leg_properties[1], 
                                   dec_limit=request.leg_properties[2])
        
        # Handle num_throws input
        num_throws = 1
        if request.num_throws > 0:
            num_throws = request.num_throws

        # Handle the linear gain input
        linear_gain_factor = request.linear_gain_factor
        # Truncate the linear gain so that it can't be more than 1. Also if it is less than or equal to 0, set it to 1.
        if linear_gain_factor > 1 or linear_gain_factor <= 0:
            linear_gain_factor = 1

        # Handle the delay percentage input
        delay_percentage = request.delay_percentage
        if delay_percentage <= 0 or delay_percentage > 1:
            delay_percentage = 1.0

        # Handle the beat gain factor input
        beat_gain_factor = request.beat_gain_factor
        if beat_gain_factor <= 0:
            beat_gain_factor = 1.0

        # Handle the pattern width input
        pattern_width = request.pattern_width
        if pattern_width <= 0:
            pattern_width = self.columns_x_span

        # Set the parameters and get the trajectory for the first throw
        self.hand_traj_gen.set_throw_parameters(throw_height=first_throw_height)
        t_throw1, pos_throw1, vel_throw1, tor_throw1, air_time1 = self.hand_traj_gen.get_throw_trajectory()

        # Set the parameters of the throw/catch and get both trajectories
        self.hand_traj_gen.set_throw_parameters(throw_height=throw_height)
        t_throw, pos_throw, vel_throw, tor_throw, air_time = self.hand_traj_gen.get_throw_trajectory()
        t_catch, pos_catch, vel_catch, tor_catch = self.hand_traj_gen.get_catch_trajectory()   

        # Calculate the minimum duration the hand will be holding the ball for (if throw happens immediately after catch)
        minimum_hold_duration = self.hand_traj_gen.catch_duration_s + self.hand_traj_gen.throw_duration_s
        beat_duration = ((self.hand_traj_gen.air_time_s + minimum_hold_duration) / 2) #* beat_gain_factor

        self.get_logger().info(f"Beat duration: {beat_duration} s")

        # Create list of times when throws will happen
        throw_times = [beat_duration * i for i in range(num_throws * 2)] # num_throws is for each column

        '''Since the first throw is higher than other throws, the beat duration for all subsequent throws can be
        shifted by the difference between the two air times'''
        # Shift the beat times for all but the first throw
        for i in range(1, len(throw_times)):
            throw_times[i] += (air_time1 - air_time) * delay_percentage

        # Initialize list of times when catches will happen
        catch_times_theoretical = []
        catch_times_actual = [] # Will be populated as throws are made

        # Log the air time
        self.get_logger().info(f"Air time: {air_time} s. For first throw: {air_time1} s")

        # Apply the linear gain to the position and velocity
        pos_throw1 *= self.LINEAR_GAIN * 0.97 # Manually found 0.97 to be best gain factor for first throw
        vel_throw1 *= self.LINEAR_GAIN * 0.97
        pos_throw *= self.LINEAR_GAIN * linear_gain_factor
        vel_throw *= self.LINEAR_GAIN * linear_gain_factor
        pos_catch *= self.LINEAR_GAIN * linear_gain_factor
        vel_catch *= self.LINEAR_GAIN * linear_gain_factor

        throw_duration = t_throw[-1] # {s} Duration of the throw movement
        catch_duration = t_catch[-1] # {s} Duration of the catch movement

        # Log these durations
        self.get_logger().info(f"Throw duration: {throw_duration} s")
        self.get_logger().info(f"Catch duration: {catch_duration} s")
        
        # Initialize the throw and catch poses
        left_pose = np.array([-pattern_width / 2, 0, self.plat_height, 0.0, 0.0, 0.0, 1.0])
        right_pose = np.array([pattern_width / 2, 0, self.plat_height, 0.0, 0.0, 0.0, 1.0])

        # Move the platform to the left pose and await arrival
        self.move_platform_to_pose_and_await_arrival(left_pose)

        # Initialize counter for how many throws have been made (in total)
        throw_counter = 0

        start_time = time.perf_counter()
        time_of_last_throw = None

        for throw_num in range(num_throws):
            # Wait until it's time to throw the first ball
            time_to_wait = (throw_times[throw_counter] - (time.perf_counter() - start_time)) * beat_gain_factor
            # time_to_wait = 0
            if time_to_wait > 0:
                self.sleep(time_to_wait)
            
            # Throw the first ball
            if throw_num == 0:
                first_throw_time = self.follow_generated_trajectory(t_throw1, pos_throw1, vel_throw1, tor_throw1)
                catch_times_theoretical.append([1, air_time1 + first_throw_time - start_time])
                ball1_air_time = air_time1
            else:
                first_throw_time = self.follow_generated_trajectory(t_throw, pos_throw, vel_throw, tor_throw)
                catch_times_theoretical.append([1, air_time + first_throw_time - start_time])
                ball1_air_time = air_time

            throw_counter += 1

            # Log this throw time
            self.get_logger().info(f"First throw time: {first_throw_time - start_time}")

            # Log the difference between this throw and the last
            if time_of_last_throw is not None:
                self.get_logger().info(f"Time between throws: {first_throw_time - time_of_last_throw}. Throw number: {throw_num}")
            
            time_of_last_throw = first_throw_time

            # Wait briefly before moving to the right pose
            self.sleep(time_to_sleep_after_throwing)
            self.send_platform_pose(right_pose)

            if throw_num == 0:
                # Lower the hand to prepare to throw the second ball
                self.follow_generated_trajectory(t_catch, pos_catch, vel_catch, tor_catch)
            else:
                # Wait until it's time to catch the second ball
                time_to_wait = air_time - (time.perf_counter() - second_throw_time)
                # Log the time to wait
                self.get_logger().info(f"Time to wait before catching second ball: {time_to_wait}")
                if time_to_wait > 0:
                    self.sleep(time_to_wait)
                
                catch_times_actual.append([2, time.perf_counter() - start_time])
                # Catch the second ball
                self.follow_generated_trajectory(t_catch, pos_catch, vel_catch, tor_catch)

            # Wait until it's time to throw the second ball
            # if throw_num == 0:
            time_to_wait = (throw_times[throw_counter] - (time.perf_counter() - start_time)) * beat_gain_factor
            # else: 
            #     time_to_wait = 0
            # Log the time to wait
            self.get_logger().info(f"Time to wait before throwing second ball: {time_to_wait}")
            if time_to_wait > 0:
                self.sleep(time_to_wait)

            # Throw the second ball
            second_throw_time = self.follow_generated_trajectory(t_throw, pos_throw, vel_throw, tor_throw)
            catch_times_theoretical.append([2, air_time + second_throw_time - start_time])
            throw_counter += 1

            # Log this throw time
            self.get_logger().info(f"Second throw time: {second_throw_time - start_time}")

            # Log the difference between this throw and the last
            self.get_logger().info(f"Time between throws: {second_throw_time - time_of_last_throw}. Throw number: {throw_num}")
            time_of_last_throw = second_throw_time

            # Wait briefly before moving to the left pose
            self.sleep(time_to_sleep_after_throwing)
            self.send_platform_pose(left_pose)

            # Wait until it's time to catch the first ball
            time_to_wait = ball1_air_time - (time.perf_counter() - first_throw_time)
            self.get_logger().info(f"Time to wait before catching first ball: {time_to_wait}")
            if time_to_wait > 0:
                self.sleep(time_to_wait)

            catch_times_actual.append([1, time.perf_counter() - start_time])
            # Catch the first ball
            self.follow_generated_trajectory(t_catch, pos_catch, vel_catch, tor_catch)

        # log the throw and catch times
        for i in range(len(catch_times_actual)):
            self.get_logger().info(f"Throw time {i + 1}: {throw_times[i]:.3f}")
            self.get_logger().info(f"Theoretical catch time {i + 1}: {catch_times_theoretical[i][1]:.3f}")
            self.get_logger().info(f"Actual catch time {i + 1}: {catch_times_actual[i][1]:.3f}\n")

        response.success = True
        response.message = "Two columns throw successful."
        return response

    async def throw_to(self, request, response):
        '''First generates the trajectory, then sends each command over the CAN bus'''

        # Check if the hand is ready to throw
        if not self.is_hand_ready_to_throw:
            self.prepare_hand_for_throw()
            response.success = False
            response.message = "Hand not ready to throw. Preparing hand for throw..."
            return response
        
        # Ensure the target_position is an array with two elements
        if len(request.target_pos) != 2:
            response.success = False
            response.message = "Target position must have two elements."
            return response

        # Handle the linear gain input
        linear_gain_factor = request.linear_gain_factor
        # Truncate the linear gain so that it can't be more than 1. Also if it is less than or equal to 0, set it to 1.
        if linear_gain_factor > 1 or linear_gain_factor <= 0:
            linear_gain_factor = 1

        # Handle the throw height input
        throw_height = request.throw_height
        if throw_height <= 0:
            throw_height = 0.6

        # Extract the target position we're throwing to
        catch_position = [request.target_pos[0],
                           request.target_pos[1]] # Tuple of (posX, posY) to move the COM of the platform to {mm}

        '''Check if the target position is within the range of the platform
        For now keep it simple and assume any catch position greater than 300 mm is uncatchable
        If the throw is not catchable, the platform will still throw it, but it won't attempt to catch'''

        if np.linalg.norm(catch_position) > 300:
            throw_is_catchable = False
            self.get_logger().warn("Throw is not catchable. Platform will not attempt to catch.")
        else:
            throw_is_catchable = True


        # Get the position we're throwing from
        throw_position = [self.plat_current_pos[0], self.plat_current_pos[1]]

        # Log the target position
        self.get_logger().info(f"Throwing to position: {catch_position} from: {self.plat_current_pos}")

        # Calculate the range of the platform during the throw
        throw_range = np.sqrt((catch_position[0] - self.plat_current_pos[0]) ** 2 + (catch_position[1] - self.plat_current_pos[1]) ** 2)

        # Log the throw range
        self.get_logger().info(f"Throw range: {throw_range} mm")

        # Get the trajectory
        time_cmd, pos, vel, tor = self.get_traj(throw_range_mm=throw_range, 
                                                linear_gain_factor=linear_gain_factor, 
                                                throw_height_m=throw_height)

        # Get the angle of the throw
        throw_angle = self.hand_traj_gen.get_throw_angle() # Angle of the throw from vertical {rad}

        self.get_logger().info(f"Throw angle (deg): {np.rad2deg(throw_angle)}")

        # Get the throw velocity and log it
        throw_velocity = self.hand_traj_gen.get_throw_velocity()
        self.get_logger().info(f"Throw velocity (m/s): {throw_velocity:.3f}")

        # Calculate the pose of the platform for the throw
        throw_ori, catch_ori = self.calc_throw_catch_angle_quaternion(throw_angle, throw_position, catch_position)

        throw_pose = np.array([throw_position[0], throw_position[1], self.plat_height, 
                               throw_ori.x, throw_ori.y, throw_ori.z, throw_ori.w])
        
        # Calculate the pose of the platform for the catch
        catch_pose = np.array([catch_position[0], catch_position[1], self.plat_height,
                                catch_ori.x, catch_ori.y, catch_ori.z, catch_ori.w])

        # Log the poses
        # self.get_logger().info(f"Throw pose: {throw_pose}")
        # self.get_logger().info(f"Catch pose: {catch_pose}")

        # Move the platform to the throw pose and await arrival (important to let the platform orient correctly)
        self.move_platform_to_pose_and_await_arrival(throw_pose)

        _ = self.follow_generated_trajectory(time_cmd, pos, vel, tor, catch_pose if throw_is_catchable else None)

        # If the throw wasn't catchable, end by returning the platform to the default position
        if not throw_is_catchable:
            self.move_platform_to_pose_and_await_arrival([0.0, 0.0, self.plat_height, 0.0, 0.0, 0.0, 1.0])

        response.success = True
        response.message = "Trajectory sent successfully."
        return response

    async def throw_continuous(self, request, response):
        '''Throw back and forth continuously, with a user-specified number of throws and break duration'''
        # Check if the hand is ready to throw
        if not self.is_hand_ready_to_throw:
            self.prepare_hand_for_throw()
            response.success = False
            response.message = "Hand not ready to throw. Preparing hand for throw..."
            return response
        
        # Handle the throw height input
        throw_height = request.throw_height
        if throw_height <= 0:
            throw_height = 0.6

        # Handle the "time_to_wait_after_throwing" input (if provided).
        # For this method this is actually used to set the time to wait after *catching*
        if request.time_to_wait_after_throwing > 0:
            time_to_sleep_after_catching = request.time_to_wait_after_throwing
        else:
            # Default to 0.0
            time_to_sleep_after_catching = 0.0

        # Handle num_throws input
        num_throws = 1
        if request.num_throws > 0:
            num_throws = request.num_throws

        # Handle the linear gain input
        linear_gain_factor = request.linear_gain_factor
        # Truncate the linear gain so that it can't be more than 1. Also if it is less than or equal to 0, set it to 1.
        if linear_gain_factor > 1 or linear_gain_factor <= 0:
            linear_gain_factor = 1

        # Set the parameters of the throw/catch
        self.hand_traj_gen.set_throw_parameters(throw_height=throw_height, throw_range=0.0)

        # Get the throw trajectory
        t_throw, pos_throw, vel_throw, tor_throw, air_time = self.hand_traj_gen.get_throw_trajectory()

        # Log the air time
        self.get_logger().info(f"Air time: {air_time} s")

        # Get the catch trajectory
        t_catch, pos_catch, vel_catch, tor_catch = self.hand_traj_gen.get_catch_trajectory()

        # Apply the linear gain to the position and velocity
        pos_throw *= self.LINEAR_GAIN * linear_gain_factor
        vel_throw *= self.LINEAR_GAIN * linear_gain_factor
        pos_catch *= self.LINEAR_GAIN * linear_gain_factor
        vel_catch *= self.LINEAR_GAIN * linear_gain_factor

        for _ in range(num_throws):
            # Throw the ball
            last_throw_time = self.follow_generated_trajectory(t_throw, pos_throw, vel_throw, tor_throw)

            # Wait until it's time to catch the ball
            time_to_wait = air_time - (time.perf_counter() - last_throw_time)
            if time_to_wait > 0:
                self.sleep(time_to_wait)
            
            # Catch the ball
            self.follow_generated_trajectory(t_catch, pos_catch, vel_catch, tor_catch)

            # Wait briefly before throwing the ball again
            self.sleep(time_to_sleep_after_catching)

        response.success = True
        response.message = "Continuous throw successful."
        return response

    #########################################################################################################
    #                                            Node Management                                            #
    #########################################################################################################

    def prepare_hand_for_throw_callback(self, future):
        '''Callback function for the prepare_hand_for_throw service'''
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
        else:
            self.is_hand_ready_to_throw = response.success
            self.get_logger().info(f"Hand is ready to throw: {self.is_hand_ready_to_throw}")

    def end_session(self, request, response):
        # The method that's called when a user clicks "End Session" in the GUI
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = HandTrajectoryTransmitter()
    try:
        executor = MultiThreadedExecutor()
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass
    finally:
        node.get_logger().info("Shutting down...")
        node.destroy_node()

        # Shut down the CAN bus
        if node.bus is not None:
            try:
                node.bus.shutdown()
                node.get_logger().info('CAN bus closed')
            except Exception as e:
                node.get_logger().error(f'Error when closing CAN bus: {e}')

        rclpy.shutdown()


if __name__ == '__main__':
    main()