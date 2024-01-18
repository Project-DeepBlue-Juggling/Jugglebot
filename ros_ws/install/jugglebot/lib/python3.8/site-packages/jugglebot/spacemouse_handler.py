import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from std_srvs.srv import Trigger
import quaternion  # numpy quaternion
import pyspacemouse
import math

class SpaceMouseHandler(Node):
    def __init__(self):
        super().__init__('spacemouse_handler')

        # Set up a service to trigger closing the node
        self.service = self.create_service(Trigger, 'end_session', self.end_session)

        # Subscribe to control_state_topic to see if spacemouse is enabled
        self.subscription = self.create_subscription(String, 'control_state_topic', self.control_state_callback, 10)
        self.subscription  # prevent unused variable warning
        self.spacemouse_enabled = False

        # Create a publisher for the platform pose and a timer to publish it
        self.publisher_ = self.create_publisher(Pose, 'platform_pose_topic', 10)
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
        yaw_mult = 25.0  # deg

        # Set the offset in z to put baseline position at ~midspan of robot
        z_offset = 140   # mm

        # Initialise pose object
        pose = Pose() 

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
        pose.position.x = state.x * xy_mult
        pose.position.y = state.y * xy_mult
        pose.position.z = state.z * z_mult + z_offset
        pose.orientation.x = quaternion_ori.x
        pose.orientation.y = quaternion_ori.y
        pose.orientation.z = quaternion_ori.z
        pose.orientation.w = quaternion_ori.w

        self.publisher_.publish(pose)

    def control_state_callback(self, msg):
        # If the incoming state calls for the spacemouse, enable it
        if msg.data == 'spacemouse' and not self.spacemouse_enabled:
            self.get_logger().info('Spacemouse enabled')
            self.spacemouse_enabled = True

        elif msg.data != 'spacemouse' and self.spacemouse_enabled:
            self.get_logger().info('Spacemouse disabled')
            self.spacemouse_enabled = False
            
        
    def end_session(self, request, response):
        # The method that's called when a user clicks "End Session" in the GUI
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = SpaceMouseHandler()
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

if __name__ == "__main__":
    main()