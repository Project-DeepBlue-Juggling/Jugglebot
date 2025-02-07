'''This node will run through a calibration routine for the platform, wherein it is moved
to a series of poses, waiting at each for a designated amount of time before continuing.'''

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from std_msgs.msg import String
from jugglebot_interfaces.msg import PlatformPoseMessage, SetTrapTrajLimitsMessage, LegsTargetReachedMessage
from geometry_msgs.msg import PoseStamped
import numpy as np
import quaternion
import time
from .calibrate_platform_pattern_generator import PosePatternGenerator

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
        self.platform_pose_publisher = self.create_publisher(PlatformPoseMessage, 'platform_pose_topic', 10)
        self.set_trap_traj_limits_publisher = self.create_publisher(SetTrapTrajLimitsMessage, 'set_leg_trap_traj_limits', 10)

        # Initialize subscribers
        self.control_mode_subscriber = self.create_subscription(String, 'control_mode_topic', self.control_mode_callback, 10)
        self.legs_target_reached_subscriber = self.create_subscription(LegsTargetReachedMessage, 'platform_target_reached',
                                                                       self.legs_target_reached_callback, 10)
        

        # Initialize service servers
        self.start_calibration_server = self.create_service(Trigger, 'start_calibration', self.start_calibration,
                                                             callback_group=MutuallyExclusiveCallbackGroup())

        self.test_radii = [0.0, 100.0]#, 200.0] # Radii to move the platform to {mm}
        self.test_angles = [0.0, 5.0, 10.0] # Tilt angles to move the platform to {degrees}
        self.test_height = 170.0 # Height to move the platform to {mm}
        self.pts_at_each_radius = 4 # Number of points to test at each radius
        self.angles_at_each_pt = 17 # Number of angles to test at each point (flat, then 8 combinations for 5 and 10 degrees)

        self.time_at_each_pose = 2.0 # seconds to wait AFTER the platform has arrived at the pose

        # Initialize the pose pattern generator
        self.pose_pattern_generator = PosePatternGenerator(self.test_angles, self.test_radii, self.pts_at_each_radius,
                                                              self.angles_at_each_pt, self.test_height)

    async def start_calibration(self, request, response):
        '''Runs the calibration routine'''

        if not self.enabled:
            response.success = False
            response.message = 'Calibrate Platform node not enabled. Please enable the node before starting calibration.'
            return response

        start_time = time.perf_counter()

        # Generate the poses
        # poses = self.pose_pattern_generator.generate_poses()
        poses = self.pose_pattern_generator.generate_sad_face_poses()

        for pose in poses:
            self.get_logger().info(f'Moving to pose: {pose.pose.position.x:.3f}, {pose.pose.position.y:.3f}, {pose.pose.position.z:.3f}, '
                                   f' with orientation: {pose.pose.orientation.x:.3f}, {pose.pose.orientation.y:.3f}, {pose.pose.orientation.z:.3f}, {pose.pose.orientation.w:.3f}')
            
            # Move the platform to the pose and await arrival
            self.move_to_calibration_pose_and_await_arrival(pose)
            time.sleep(self.time_at_each_pose) # Wait for the nominated duration

    
        # Return to the home pose
        home_pose = PoseStamped()
        home_pose.pose.position.x = 0.0
        home_pose.pose.position.y = 0.0
        home_pose.pose.position.z = self.test_height
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
        pose_msg = PlatformPoseMessage()
        pose_msg.pose_stamped = pose
        pose_msg.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_msg.publisher = 'CALIBRATE_PLATFORM'

        # Publish the pose
        self.platform_pose_publisher.publish(pose_msg)

        # Give the robot a second to move
        time.sleep(1.0)

        # Wait for the platform to arrive at the calibration pose
        while not all (self.legs_target_reached):
            self.get_logger().info(f'Waiting for platform to reach calibration pose. Status: {self.legs_target_reached}',
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

    #########################################################################################################
    #                                       Control Mode Callback                                           #
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
