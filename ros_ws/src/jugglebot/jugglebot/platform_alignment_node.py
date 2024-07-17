import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from geometry_msgs.msg import PoseStamped, Quaternion
from action_msgs.msg import GoalStatus
from std_srvs.srv import Trigger
from jugglebot_interfaces.action import AutoLevelPlatformAction
from jugglebot_interfaces.msg import LegsTargetReachedMessage, SetMotorVelCurrLimitsMessage
from jugglebot_interfaces.srv import GetTiltReadingService

import quaternion
import time
import numpy as np

class PlatformAlignmentNode(Node):
    def __init__(self):
        super().__init__('platform_alignment_node')

        # Set up a service to trigger closing the node
        self.service = self.create_service(Trigger, 'end_session', self.end_session)

        # Set up a service client to get the tilt sensor reading
        self.get_tilt_reading_client = self.create_client(GetTiltReadingService, 'get_platform_tilt')

        # Set up an action client to conduct the auto-calibration process
        self.auto_level_client = ActionServer(self,
                                              AutoLevelPlatformAction,
                                              'auto_level_platform',
                                              self.execute_auto_calibrate_callback,
                                              goal_callback=self.accept_auto_calibrate_callback,
                                              cancel_callback=self.cancel_auto_calibrate_callback)
        
        self.platform_pose_publisher = self.create_publisher(PoseStamped, 'platform_pose_topic', 10)
        self.platform_tilt_offset = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0) # Offset to apply to the platform orientation to level it

        # Set up a publisher for the motor speeds
        self.motor_speed_publisher = self.create_publisher(SetMotorVelCurrLimitsMessage, 'set_motor_vel_curr_limits', 10)

        # Set up a publisher for the pose offset
        self.pose_offset_publisher = self.create_publisher(Quaternion, 'pose_offset_topic', 10)

        # Subscribe to the legs_target_reached topic to check if the platform has reached the calibration pose
        self.legs_target_reached = [False] * 6
        self.legs_target_reached_subscription = self.create_subscription(LegsTargetReachedMessage,
                                                                         'target_reached',
                                                                         self.legs_target_reached_callback,
                                                                         10)

        # Initialize the offset to apply to the inclinometer readings to level the platform. Found experimentally
        self.INCLINOMETER_OFFSET = [-0.6, -0.2] # tiltX, tiltY (deg)


    def legs_target_reached_callback(self, msg):
        '''Handles the target_reached message'''

        self.legs_target_reached[0] = msg.leg0_has_arrived
        self.legs_target_reached[1] = msg.leg1_has_arrived
        self.legs_target_reached[2] = msg.leg2_has_arrived
        self.legs_target_reached[3] = msg.leg3_has_arrived
        self.legs_target_reached[4] = msg.leg4_has_arrived
        self.legs_target_reached[5] = msg.leg5_has_arrived



    #########################################################################################################
    #                                         Alignment Methods                                             #
    #########################################################################################################


    def accept_auto_calibrate_callback(self, goal_request):
        '''Accept the auto-calibrate goal'''
        return GoalResponse.ACCEPT

    def cancel_auto_calibrate_callback(self, goal_handle):
        '''Cancel the auto-calibrate goal'''
        self.get_logger().info('Goal cancelled')
        return CancelResponse.ACCEPT

    def execute_auto_calibrate_callback(self, goal_handle):
        '''Execute the auto-calibrate goal'''

        # Start by setting conservative movement speeds
        self.set_legs_vel_limit(leg_vel_limit=2.5)
        
        # Now move to the calibration pose
        self.move_to_calibration_pose_and_await_arrival()
        time.sleep(0.5) # Add a short break just to be sure that the motors have stopped moving

        # Get the platform tilt
        self.get_tilt_reading(last_reading=False, goal_handle=goal_handle)

        # Return the goal handle. NOT SURE ABOUT THIS??
        return goal_handle


    def feedback_callback(self, feedback_msg):
        '''Handles the feedback from the action server'''
        tiltX, tiltY = feedback_msg.feedback.tilt

        self.get_logger().info(f'Feedback received! TiltX: {tiltX}, TiltY: {tiltY}')

    def alignment_done_callback(self, future):
        '''Handles the completion of the action server'''
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal was rejected...')
            return

        self.get_logger().info('Goal accepted...')
        self._goal_handle = goal_handle

        # Wait for the result
        future = goal_handle.get_result_async()
        future.add_done_callback(self.alignment_result_callback)

    def alignment_result_callback(self, future):
        '''Handles the result of the action server'''
        result = future.result().result
        status = future.result().status

        tiltX, tiltY = result.tilt

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Alignment process succeeded. Final tilt angles: {tiltX:.3f}, {tiltY:.3f}')
        else:
            self.get_logger().info(f'Alignment process failed with status: {status}')
    
    #########################################################################################################
    #                                         Auxiliary Methods                                             #
    #########################################################################################################

    def set_legs_vel_limit(self, leg_vel_limit):
        '''Set the absolute velocity limits for the motors'''
        # Construct the message
        msg = SetMotorVelCurrLimitsMessage()
        msg.legs_vel_limit = leg_vel_limit # {rev/s}

        # Publish the message
        self.motor_speed_publisher.publish(msg)

    def move_to_calibration_pose_and_await_arrival(self):
        ''' Move the platform to the calibration pose'''

        # Reset the target_reached flags, since these only update at 10 Hz (set by timer in can_bus_handler_node)
        self.legs_target_reached = [False] * 6

        # Construct the pose message
        pose_stamped = PoseStamped()
        pose_stamped.pose.position.x = 0.0
        pose_stamped.pose.position.y = 0.0
        pose_stamped.pose.position.z = 170.0
        pose_stamped.pose.orientation.x = 0.0
        pose_stamped.pose.orientation.y = 0.0
        pose_stamped.pose.orientation.z = 0.0
        pose_stamped.pose.orientation.w = 1.0

        # Set the time stamp
        pose_stamped.header.stamp = self.get_clock().now().to_msg()

        # Publish the pose
        self.platform_pose_publisher.publish(pose_stamped)

        # Wait for the platform to arrive at the calibration pose
        while not all (self.legs_target_reached):
            self.get_logger().info(f'Waiting for platform to reach calibration pose. Status: {self.legs_target_reached}',
                                   throttle_duration_sec=1.0)
            time.sleep(0.1)

    def get_tilt_reading(self, last_reading, goal_handle):
        '''Get the tilt reading from the tilt sensor. Different response handlers are used for different readings'''
        # Call the tilt sensor service
        tilt_request = GetTiltReadingService.Request()
        future = self.get_tilt_reading_client.call_async(tilt_request)

        if not last_reading:
            future.add_done_callback(lambda future: self.process_intermediate_tilt_reading(future, goal_handle))
        else:
            future.add_done_callback(lambda future: self.process_last_tilt_reading(future, goal_handle))

    def process_intermediate_tilt_reading(self, future, goal_handle):
        '''Process the tilt reading from the tilt sensor'''
        response = future.result()
        if response is not None:
            tiltX, tiltY = response.tilt_readings

            # Publish this tilt reading as feedback
            feedback = AutoLevelPlatformAction.Feedback()
            feedback.tilt = [tiltX, tiltY]
            goal_handle.publish_feedback(feedback)

            # Add the inclinometer offset to the tilt readings
            tiltX, tiltY = self.add_inclinometer_offset(tiltX, tiltY)

            # Convert the tilt readings to a quaternion offset and publish the result
            self.convert_tilt_to_quat_and_publish(tiltX, tiltY)
            
            # Now move the platform back to the calibration pose
            self.move_to_calibration_pose_and_await_arrival()
            time.sleep(0.5) # Add a short break just to be sure that the motors have stopped moving

            # Get the final tilt reading
            self.get_tilt_reading(last_reading=True, goal_handle=goal_handle)

        else:
            self.get_logger().error('Service call failed')

    def process_last_tilt_reading(self, future, goal_handle):
        '''Process the tilt reading from the tilt sensor'''
        response = future.result()
        if response is not None:
            tiltX, tiltY = response.tilt_readings

            # Subtract the inclinometer offset from the tilt readings so the result tilt is for the platform
            tiltX, tiltY = self.subtract_inclinometer_offset(tiltX, tiltY)

            # Publish this tilt reading as the result
            result = AutoLevelPlatformAction.Result()
            result.tilt = [tiltX, tiltY]

            goal_handle.succeed()
            goal_handle.set_result(result)

            # Finally, return the leg velocity limit to 50 rev/s
            self.set_legs_vel_limit(leg_vel_limit=50.0)

        else:
            self.get_logger().error('Service call failed')

    def convert_tilt_to_quat_and_publish(self, tiltX, tiltY):
        ''' Convert tilt readings to a quaternion offset to apply to the platform orientation to level it'''
        # Convert orientation from Euler angles to quaternions. -ve used since we're reversing the direction of the measured tilt
        q_roll = quaternion.from_rotation_vector([-tiltX, 0, 0])
        q_pitch = quaternion.from_rotation_vector([0, -tiltY, 0])

        tilt_offset_quat = q_roll * q_pitch
        
        # Get the current offset as a numpy quaternion
        current_offset = quaternion.quaternion(self.platform_tilt_offset.w, 
                                                self.platform_tilt_offset.x, 
                                                self.platform_tilt_offset.y, 
                                                self.platform_tilt_offset.z)

        # Calculate the new offset by multiplying the current offset by the tilt offset quaternion
        platform_tilt_offset = tilt_offset_quat * current_offset

        # Store the new offset as a ROS2 quaternion
        self.platform_tilt_offset = Quaternion(x=platform_tilt_offset.x,
                                                y=platform_tilt_offset.y,
                                                z=platform_tilt_offset.z,
                                                w=platform_tilt_offset.w)

        # Publish the new offset
        self.pose_offset_publisher.publish(self.platform_tilt_offset)

    def add_inclinometer_offset(self, tiltX, tiltY):
        '''Add the inclinometer offset to the tilt readings'''
        # Make sure to convert the offset values to radians (from degrees)
        tiltX += np.deg2rad(self.INCLINOMETER_OFFSET[0])
        tiltY += np.deg2rad(self.INCLINOMETER_OFFSET[1])

        return tiltX, tiltY
    
    def subtract_inclinometer_offset(self, tiltX, tiltY):
        '''Subtract the inclinometer offset from the tilt readings'''
        # Make sure to convert the offset values to radians (from degrees)
        tiltX -= np.deg2rad(self.INCLINOMETER_OFFSET[0])
        tiltY -= np.deg2rad(self.INCLINOMETER_OFFSET[1])

        return tiltX, tiltY

    #########################################################################################################
    #                                           End Session                                                 #
    #########################################################################################################

    def end_session(self, request, response):
        # The method that's called when a user clicks "End Session" in the GUI
        raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = PlatformAlignmentNode()

    try:
        rclpy.spin(node)
    except RuntimeError:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down...")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
