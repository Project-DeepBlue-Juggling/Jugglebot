import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from jugglebot_interfaces.msg import RobotStateMessage
from geometry_msgs.msg import Point, Vector3
from builtin_interfaces.msg import Time

class StateManagerNode(Node):
    def __init__(self):
        super().__init__('state_manager_node')

        # Define the initial states
        self.initial_states()

        # Subscribe to relevant Topics
        self.control_state_sub = self.create_subscription(String,'control_state_topic', self.control_state_callback, 10)
        self.robot_state_sub = self.create_subscription(RobotStateMessage,'robot_state_topic', self.robot_state_callback, 10)

        # Initialize publishers to relevant Topics
        self.control_state_pub = self.create_publisher(String, 'control_state_topic',10)
        self.robot_state_pub = self.create_publisher(RobotStateMessage, 'robot_state_topic',10)

        # Set up a service to trigger closing the node
        self.service = self.create_service(Trigger, 'end_session', self.end_session)

        # Set up a timer to periodically publish the robot state
        self.publish_timer = self.create_timer(0.1, self.publish_states)

    def control_state_callback(self, msg):
        # Update the control state
        self.control_state = msg.data

    def robot_state_callback(self, msg):
        # Update the robot state
        self.robot_state = msg

    def publish_states(self):
        # Publish the robot state
        self.robot_state_pub.publish(self.robot_state)

        control_state_msg = String()
        control_state_msg.data = self.control_state
        self.control_state_pub.publish(control_state_msg)

    def initial_states(self):
        '''Set up the initial robot state'''

        # Start with control state
        self.control_state = 'gui'

        # Now the robot state
        self.robot_state = RobotStateMessage()
        self.robot_state.is_homed = False
        self.robot_state.status = 'waiting'

        # Since the robot is 'waiting', all other values are irrelevant
        self.robot_state.next_ball_id = 0
        self.robot_state.next_ball_position = Point(x=0.0, y=0.0, z=0.0)
        self.robot_state.next_ball_velocity = Vector3(x=0.0, y=0.0, z=0.0)
        self.robot_state.next_action_time = Time(sec=0, nanosec=0)

    def end_session(self, request, response):
        # The method that's called when a user clicks "End Session" in the GUI
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = StateManagerNode()
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
