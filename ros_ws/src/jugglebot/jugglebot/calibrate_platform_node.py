'''This node will run through a calibration routine for the platform, wherein it is moved
to a series of poses, waiting at each for a designated amount of time before continuing.'''

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from std_msgs.msg import String, Float32MultiArray, Int8MultiArray
from jugglebot_interfaces.msg import PlatformPoseCommand, SetTrapTrajLimitsMessage, LegsTargetReachedMessage, RobotState
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
import time
from .calibrate_platform_pattern_generator import PosePatternGenerator
import numpy as np

class CalibratePlatformNode(Node):
    def __init__(self):
        super().__init__('calibrate_platform_node')

        # Initialize state variables
        self.shutdown_flag = False
        self.enabled = False
        self.legs_target_reached = [False] * 6

        self.trap_traj_limits = {
            'vel_limit': 10, # rev/s
            'acc_limit': 10, # rev/s^2
            'dec_limit': 10 # rev/s^2
        }

        # Initialize a service to trigger closing the node
        self.service = self.create_service(Trigger, 'end_session', self.end_session)

        # Initialize publishers
        self.platform_pose_publisher = self.create_publisher(PlatformPoseCommand, 'platform_pose_topic', 10)
        self.set_trap_traj_limits_publisher = self.create_publisher(SetTrapTrajLimitsMessage, 'set_leg_trap_traj_limits', 10)
        self.settled_leg_lengths_publisher = self.create_publisher(Float32MultiArray, 'settled_leg_lengths', 10)
        self.settled_platform_poses_publisher = self.create_publisher(PoseArray, 'settled_platform_poses', 10)

        # Initialize subscribers
        self.control_mode_subscriber = self.create_subscription(String, 'control_mode_topic', self.control_mode_callback, 10)
        self.legs_target_reached_subscriber = self.create_subscription(LegsTargetReachedMessage, 'platform_target_reached',
                                                                       self.legs_target_reached_callback, 10)
        self.legs_state_subscriber = self.create_subscription(Int8MultiArray, 'leg_state_topic', self.legs_state_callback, 10)

        # The following subscribers are used to publish hardware feedback 'snapshots' so that it's easier to pick out the data we need
        self.robot_state_subscriber = self.create_subscription(RobotState, 'robot_state', self.robot_state_callback, 10)
        self.platform_pose_mocap_subscriber = self.create_subscription(PoseStamped, 'platform_pose_mocap', self.platform_pose_mocap_callback, 10)

        # Initialize service servers
        self.start_calibration_server = self.create_service(Trigger, 'start_calibration', self.start_calibration,
                                                             callback_group=MutuallyExclusiveCallbackGroup())

        self.test_radii = [0.0, 150.0, 180.0] # Radii to move the platform to {mm}
        self.test_height_min = 130.0 # Minimum height to test. Measured from platform lowest position {mm}
        self.test_height_max = 200.0 # Maximum height to test. Measured from platform lowest position {mm}
        self.pts_at_each_radius = 4  # Number of points to test at each radius

        self.time_at_each_pose = 2.0 # seconds to wait AFTER the platform has arrived at the pose
        pattern_iterations = 1 # Number of times to repeat the pattern

        self.leg_lengths = [None] * 6 # Variable to store the leg lengths right now. This is used to publish the leg lengths once each movement has settled
        self.leg_states = [None] * 6 # Variable to store the leg states. Used to  inform whether to save (publish) this pose or not
        self.pose_meas = Pose() # Variable to store the mocap pose of the platform
        self.pose_cmd = Pose() # Variable to store the commanded pose of the platform

        # Initialize the pose pattern generator
        self.pose_pattern_generator = PosePatternGenerator(self.test_radii, self.pts_at_each_radius,
                                                              self.test_height_min, self.test_height_max,
                                                              height_increments=4, orientations_per_position=3, tilt_angle_limits=15.0,
                                                              iterations=pattern_iterations,
                                                              logger=self.get_logger())

    #########################################################################################################
    #                                            Main Method                                                #
    #########################################################################################################

    async def start_calibration(self, request, response):
        '''Runs the calibration routine'''

        if not self.enabled:
            response.success = False
            response.message = 'Calibrate Platform node not enabled. Please enable the node before starting calibration.'
            return response

        start_time = time.perf_counter()

        # Generate the poses
        # poses = self.pose_pattern_generator.generate_poses(pose_type='test_poses')
        # poses = self.pose_pattern_generator.generate_poses(pose_type='dummy_square')
        # poses = self.pose_pattern_generator.generate_poses(pose_type='sad_face')
        # poses = self.pose_pattern_generator.generate_poses(pose_type='flat_grid')
        # poses = self.pose_pattern_generator.generate_poses(pose_type='happy_face')
        # poses = self.pose_pattern_generator.generate_poses(pose_type='angled_grid')
        poses = self.pose_pattern_generator.generate_poses(pose_type='random_sample_angled_grid')

        for i, pose in enumerate(poses):
            self.get_logger().info(f'Moving to pose {i}: {pose.pose.position.x:.3f}, {pose.pose.position.y:.3f}, {pose.pose.position.z:.3f}, '
                                   f' with orientation: {pose.pose.orientation.x:.3f}, {pose.pose.orientation.y:.3f}, {pose.pose.orientation.z:.3f}, {pose.pose.orientation.w:.3f}')
            
            # Move the platform to the pose and await arrival
            self.move_to_calibration_pose_and_await_arrival(pose)
            time.sleep(self.time_at_each_pose) # Wait for the nominated duration

            # Check if any leg state is non-zero. If so, skip this pose
            if any(self.leg_states):
                self.get_logger().info(f'Leg state is non-zero. Skipping pose {i}.')
                # Empty the leg states
                self.leg_states = [None] * 6
                continue

            # Now that the platform has arrived at the pose, publish the leg lengths and measured platform pose
            self.publish_settled_leg_lengths()
            self.publish_settled_platform_poses()
    
        # Return to the home pose
        home_pose = PoseStamped()
        home_pose.header.frame_id = 'platform_start'
        home_pose.header.stamp = self.get_clock().now().to_msg()
        home_pose.pose.position.x = 0.0
        home_pose.pose.position.y = 0.0
        home_pose.pose.position.z = (self.test_height_min + self.test_height_max) / 2
        home_pose.pose.orientation.x = 0.0
        home_pose.pose.orientation.y = 0.0
        home_pose.pose.orientation.z = 0.0
        home_pose.pose.orientation.w = 1.0

        self.move_to_calibration_pose_and_await_arrival(home_pose)

        end_time = time.perf_counter()

        self.get_logger().info(f'Calibration complete. Time taken: {end_time - start_time:.2f} seconds')

        response.success = True
        response.message = 'Calibration complete'
        return response

    #########################################################################################################
    #                                           Helper Methods                                              #
    #########################################################################################################

    def move_to_calibration_pose_and_await_arrival(self, pose):
        ''' Move the platform to the nominated pose and await arrival'''

        # Reset the target_reached flags, since these only update at 10 Hz (set by timer in can_bus_handler_node)
        self.legs_target_reached = [False] * 6

        # Create the message
        pose_msg = PlatformPoseCommand()
        pose_msg.pose_stamped = pose
        pose_msg.pose_stamped.header.frame_id = 'platform_start'
        pose_msg.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_msg.publisher = 'CALIBRATE_PLATFORM'

        self.pose_cmd = pose_msg.pose_stamped.pose

        # Publish the pose
        self.platform_pose_publisher.publish(pose_msg)

        # Give the robot a second to move
        time.sleep(1.0)

        # Wait for the platform to arrive at the calibration pose
        while not all (self.legs_target_reached):
            not_arrived_indices = [i for i, reached in enumerate(self.legs_target_reached) if not reached]
            self.get_logger().info(f'Waiting for platform to arrive at pose. Waiting on legs: {not_arrived_indices}',
                                   throttle_duration_sec=1.0)
            time.sleep(0.1) 

    def legs_target_reached_callback(self, msg):
        '''Handles the target_reached message'''

        self.legs_target_reached[0] = msg.leg0_has_arrived
        self.legs_target_reached[1] = msg.leg1_has_arrived
        self.legs_target_reached[2] = msg.leg2_has_arrived
        self.legs_target_reached[3] = msg.leg3_has_arrived
        self.legs_target_reached[4] = msg.leg4_has_arrived
        self.legs_target_reached[5] = msg.leg5_has_arrived

    def set_trap_traj_limits(self, vel_limit, acc_limit, dec_limit):
        """
        Set the trapezoidal trajectory limits for the legs (vel, acc, dec).

        Ensure all are floats before publising
        """
        # Create a message object
        message = SetTrapTrajLimitsMessage()
        message.trap_vel_limit = float(vel_limit)
        message.trap_acc_limit = float(acc_limit)
        message.trap_dec_limit = float(dec_limit)

        self.set_trap_traj_limits_publisher.publish(message)

        # Log the new limits
        self.get_logger().info(f"Trap traj limits set to: vel = {vel_limit}, acc = {acc_limit}, dec = {dec_limit}")

    def publish_settled_leg_lengths(self):
        '''Publish the leg lengths once they have settled'''

        # Create the message
        msg = Float32MultiArray()
        msg.data = self.leg_lengths

        # Publish the message
        self.settled_leg_lengths_publisher.publish(msg)

    def publish_settled_platform_poses(self):
        '''Publish the platform pose once it has settled'''

        # Construct the PoseArray message, consisting of the command pose followed by the measured pose
        pose_array_msg = PoseArray()
        pose_array_msg.header.frame_id = 'platform_start'
        pose_array_msg.header.stamp = self.get_clock().now().to_msg()
        pose_array_msg.poses = [self.pose_cmd, self.pose_meas]

        # Publish the message
        self.settled_platform_poses_publisher.publish(pose_array_msg)

    #########################################################################################################
    #                                       Subscription Callbacks                                          #
    #########################################################################################################

    def control_mode_callback(self, msg):
        # If the incoming state calls for this node, enable it
        if msg.data == 'CALIBRATE_PLATFORM' and not self.enabled:
            self.get_logger().info('Calibrate Platform node enabled')
            self.enabled = True

            # Set the trapezoidal trajectory limits for the legs
            self.set_trap_traj_limits(**self.trap_traj_limits)

        elif msg.data != 'CALIBRATE_PLATFORM' and self.enabled:
            self.get_logger().info('Calibrate Platform node disabled')
            self.enabled = False

    def robot_state_callback(self, msg):
        '''
        Handles the robot state message, extracting the measured leg lengths
        '''
        # Leg length for motor 0 is at: msg.motor_states[0].pos_estimate
        for i in range(6):
            self.leg_lengths[i] = msg.motor_states[i].pos_estimate

    def platform_pose_mocap_callback(self, msg):
        '''Handles the mocap pose message'''
        self.pose_meas = msg.pose

    def legs_state_callback(self, msg):
        '''
        Handles the leg state message, extracting the leg states.
        If any leg state is non-zero, the platform is unable to truly attempt to move to the pose.
        These poses will be ignored and not published (hence not saved to the CSV file).
        '''
        # Leg state for motor 0 is at: msg.data[0]
        for i in range(6):
            self.leg_states[i] = np.abs(msg.data[i]) # Make all error states +1.

    #########################################################################################################
    #                                          Node Management                                              #
    #########################################################################################################

    def end_session(self, request, response):
        """Service callback to end the session from the GUI"""
        self.get_logger().info("End session requested. Shutting down...")
        response.success = True
        response.message = "Session ended. Shutting down node."
        self.shutdown_flag = True
        return response

    def on_shutdown(self):
        """
        Cleanup method called when the node is shutting down.
        """
        self.get_logger().info("Shutting down CalibratePlatformNode...")
        # Perform any necessary cleanup here
        # Destroy publishers, subscribers, services, timers, etc., if needed


def main(args=None):
    rclpy.init(args=args)
    node = CalibratePlatformNode()

    try:
        executor = MultiThreadedExecutor()
        while rclpy.ok() and not node.shutdown_flag:
            rclpy.spin_once(node, executor=executor)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Shutting down.")
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
