'''This node is used exclusively for calibrating the platform. It is not used during normal operation of the robot.
The main feature of this node is that it allows for commanding the pose of the platform using euler angles'''

import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped, Quaternion
from std_srvs.srv import Trigger
import quaternion

class PlatformPoseCommandNode(Node):
    def __init__(self):
        super().__init__('platform_pose_command_node')

        # Set up a service to trigger closing the node
        self.service = self.create_service(Trigger, 'end_session', self.end_session)

        # Initialize a publisher to publish the platform pose
        self.pose_publisher = self.create_publisher(PoseStamped, 'platform_pose_topic', 10)

        # Subscribe to the calibration_pose_euler topic to receive the desired platform pose
        self.calibration_pose_subscription = self.create_subscription(Float32MultiArray, 
                                                                      'pose_command_euler', 
                                                                      self.calibration_pose_callback, 
                                                                      10)

    def calibration_pose_callback(self, msg):
        # Extract the pose from the message
        pose = msg.data # The pose is a 6-element array [x, y, z, roll, pitch, yaw]
        # Note that the incoming pose angles are in DEGREES

        # Extract the position data from the message
        position = pose[:3]

        # Extract the euler angles from the message
        euler_angles = pose[3:]

        # Log the position and euler angles
        # self.get_logger().info(f"Received pose: {position}, {euler_angles}")

        # Convert the angles to radians
        euler_angles = np.deg2rad(euler_angles)

        # Log the converted angles
        # self.get_logger().info(f"Angles in rad: {euler_angles}")

        q_roll = quaternion.from_rotation_vector([euler_angles[0], 0, 0])
        q_pitch = quaternion.from_rotation_vector([0, euler_angles[1], 0])
        q_yaw = quaternion.from_rotation_vector([0, 0, euler_angles[2]])

        q = q_roll * q_pitch * q_yaw

        # Convert the euler angles to a quaternion
        # q = quaternion.from_euler_angles(euler_angles[0], euler_angles[1], euler_angles[2])

        # Log the converted quaternion
        # self.get_logger().info(f"Converted quaternion: {q}")

        # Initialize and populate a PoseStamped message
        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.header.stamp = self.get_clock().now().to_msg()
        
        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]
        pose.pose.position.z = position[2]
        
        pose.pose.orientation.x = q.x
        pose.pose.orientation.y = q.y
        pose.pose.orientation.z = q.z
        pose.pose.orientation.w = q.w

        # Publish the pose for the robot to move to
        self.pose_publisher.publish(pose)


    def end_session(self, request, response):
        # The method that's called when a user clicks "End Session" in the GUI
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = PlatformPoseCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass
    finally:
        node.get_logger().info("Shutting down...")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
