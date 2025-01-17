"""
This ROS2 node reads the state of a 3Dconnexion SpaceMouse and publishes the pose of the platform to the 'platform_pose' topic.
The node subscribes to the 'control_state' topic to see if the spacemouse is the chosen control method.
If the spacemouse is enabled, the node reads the state of the spacemouse and publishes the pose of the platform.
Otherwise, the node does nothing.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from jugglebot_interfaces.msg import PlatformPoseMessage
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
import quaternion  # numpy quaternion
import pyspacemouse
import math

class SpaceMouseHandler(Node):
    def __init__(self):
        super().__init__('spacemouse_handler')

        # Initialise shutdown flag
        self.shutdown_flag = False

        # Set up a service to trigger closing the node
        self.service = self.create_service(Trigger, 'end_session', self.end_session)

        # Subscribe to control_mode_topic to see if spacemouse is enabled
        self.subscription = self.create_subscription(String, 'control_mode_topic', self.control_mode_callback, 10)
        self.spacemouse_enabled = False

        # Create a publisher for the platform pose and a timer to publish it
        self.publisher_ = self.create_publisher(PlatformPoseMessage, 'platform_pose_topic', 10)
        self.timer = self.create_timer(0.01, self.publish_pose)

        """Initialize and open the SpaceMouse."""
        self._is_open = pyspacemouse.open()

        if not self._is_open:
            raise ConnectionError("Failed to connect to the SpaceMouse.")

    def publish_pose(self):
        """
        Read the state of the SpaceMouse and publish this state to the 'platform_pose' topic
        Start by checking if the spacemouse is the chosen control method
        """
        if not self.spacemouse_enabled:
            pyspacemouse.read()  # Read the state of the spacemouse to clear the buffer
            return

        # Set the multipliers for each axis (mm, deg)
        xy_mult = 150.0  # mm
        z_mult = 140.0   # mm
        pitch_roll_mult = 30.0  # deg
        yaw_mult = 10.0  # deg

        # Set the offset in z to put baseline position at ~midspan of robot
        z_offset = 170   # mm

        # Initialise PlatformPoseMessage object
        message = PlatformPoseMessage()
        pose_stamped = PoseStamped() 

        # Get the current time
        current_time = self.get_clock().now().to_msg()

        # Read the state of the spacemouse
        state = pyspacemouse.read()

        # Apply multipliers and convert to radians
        # Not sure why I need negatives out the front of pitch and yaw, but this works!
        roll = math.radians(state.roll * pitch_roll_mult)
        pitch = math.radians(-state.pitch * pitch_roll_mult)
        yaw = math.radians(-state.yaw * yaw_mult)

        # Convert orientation from Euler angles to quaternions
        q_roll = quaternion.from_rotation_vector([0, roll, 0])
        q_pitch = quaternion.from_rotation_vector([pitch, 0, 0])
        q_yaw = quaternion.from_rotation_vector([0, 0, yaw])

        quaternion_ori = q_yaw * q_roll * q_pitch
        
        # Construct the pose message
        pose_stamped.pose.position.x = state.x * xy_mult
        pose_stamped.pose.position.y = state.y * xy_mult
        pose_stamped.pose.position.z = state.z * z_mult + z_offset
        pose_stamped.pose.orientation.x = quaternion_ori.x
        pose_stamped.pose.orientation.y = quaternion_ori.y
        pose_stamped.pose.orientation.z = quaternion_ori.z
        pose_stamped.pose.orientation.w = quaternion_ori.w

        # Set the time stamp
        pose_stamped.header.stamp = current_time

        message.pose_stamped = pose_stamped
        message.publisher = 'spacemouse'

        self.publisher_.publish(message)

    def control_mode_callback(self, msg):
        # If the incoming state calls for the spacemouse, enable it
        if msg.data == 'spacemouse' and not self.spacemouse_enabled:
            self.get_logger().info('Spacemouse enabled')
            self.spacemouse_enabled = True

        elif msg.data != 'spacemouse' and self.spacemouse_enabled:
            self.get_logger().info('Spacemouse disabled')
            self.spacemouse_enabled = False
    
    #########################################################################################################
    #                                          Utility Functions                                            #
    #########################################################################################################

    def on_shutdown(self):
        """Handle node shutdown."""
        self.get_logger().info("Shutting down SpacemouseHandler...")
        try:
            # Close the spacemouse
            pyspacemouse.close()
        except Exception as e:
            self.get_logger().error(f"Error during node shutdown: {e}")

    def end_session(self, request, response):
        """Service callback to end the session from the GUI"""
        self.get_logger().info("End session requested. Shutting down...")
        response.success = True
        response.message = "Session ended. Shutting down node."
        self.shutdown_flag = True
        return response
        

def main(args=None):
    rclpy.init(args=args)
    node = SpaceMouseHandler()

    try:
        while rclpy.ok() and not node.shutdown_flag:
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()