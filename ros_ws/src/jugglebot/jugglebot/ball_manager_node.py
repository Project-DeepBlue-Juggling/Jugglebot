import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from jugglebot_interfaces.msg import HandStateList, BallStateMessage
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from builtin_interfaces.msg import Time

class BallManagerNode(Node):
    def __init__(self):
        super().__init__('ball_manager_node')

        # Subscribe to relevant Topics
        self.hand_state_sub = self.create_subscription(HandStateList,'hand_state_topic', self.hand_state_callback, 10) # To determine if the ball is thrown

        # Initialize publishers to relevant Topics
        self.ball_state_pub = self.create_publisher(BallStateMessage, 'ball_state_topic',10)

        # Set up a service to trigger closing the node
        self.service = self.create_service(Trigger, 'end_session', self.end_session)

        # Set up a timer to periodically publish the ball state
        self.publish_timer = self.create_timer(0.1, self.publish_ball_state)

        # Class variables
        self.is_active = False
        self.time_offset = 0.0  # {s} The time offset between the hand actually throwing and when it *should* have thrown
        self.velocity = 0.0 # {m/s} The velocity of the ball. Used to determine if it has been thrown or not
        self.is_held = False  # Whether the ball is currently being held by the hand

        self.g = 9.81  # m/s^2
        self.lowest_catch_pos = -0.0  # {m} The lowest position the hand can catch a ball at (from throw pos). Would ideally be based on the hand's workspace
        self.ball_path = []

        self.throw_time = None  # {s} The time the ball was thrown
        self.throw_pos = None   # {m} The position the ball was thrown from (relative to hand throw/catch plane)
        self.throw_vel = None   # {m/s} The velocity the ball was thrown at

    #########################################################################################################
    #                                             Calculations                                              #
    #########################################################################################################

    def calculate_ball_path(self):
        # This method will calculate the path of the ball and publish it to the ball_state_topic

        # Initialize the path and time
        path = []
        time = 0.0         # {s} The time since the ball was thrown
        time_delta = 0.05  # {s} The time between each point in the path

        # Get the properties of the start of the throw
        initial_position = self.throw_pos
        initial_velocity = self.throw_vel
        initial_time = self.throw_time

        # Initialize a variable to track the z position
        z_pos = initial_position.z

        while z_pos > self.lowest_catch_pos:
            point = PoseStamped()
    
            point.pose.position.x = initial_position.x + initial_velocity.x * time
            point.pose.position.y = initial_position.y + initial_velocity.y * time
            point.pose.position.z = initial_position.z + initial_velocity.z * time - 0.5 * self.g * time**2

            # Convert m to mm. JANKY!!!
            point.pose.position.x *= 1000
            point.pose.position.y *= 1000
            point.pose.position.z *= 1000

            real_time = time + initial_time

            pt_time = Time()
            pt_time.sec = int(real_time)
            pt_time.nanosec = int((real_time - pt_time.sec) * 1e9)

            point.header.stamp = pt_time

            # Add the point to the path
            path.append(point)

            # Update the z position
            z_pos = point.pose.position.z

            # Update the time by the chosen delta
            time += time_delta

        # Store the ball path
        self.ball_path = path

    def get_current_ball_in_flight_position_as_point_stamped(self):
        # This method will return the current position of the ball, based on the current time

        # Initialize the message
        point_msg = PointStamped()
        
        # Get the current time in seconds
        current_time = self.get_clock().now().nanoseconds / 1e9

        # Apply the time offset
        # current_time -= self.time_offset

        # Convert the current time to a Time message
        current_time_msg = Time()
        current_time_msg.sec = int(current_time)
        current_time_msg.nanosec = int((current_time - current_time_msg.sec) * 1e9)

        time_in_flight = current_time - self.throw_time

        # Calculate the position of the ball
        point_msg.point.x = self.throw_pos.x + self.throw_vel.x * time_in_flight
        point_msg.point.y = self.throw_pos.y + self.throw_vel.y * time_in_flight
        point_msg.point.z = self.throw_pos.z + self.throw_vel.z * time_in_flight - 0.5 * self.g * time_in_flight**2

        # Convert m to mm
        point_msg.point.x *= 1000
        point_msg.point.y *= 1000
        point_msg.point.z *= 1000

        # Set the time stamp
        point_msg.header.stamp = current_time_msg

        return point_msg


     
    #########################################################################################################
    #                                              Publishing                                               #
    #########################################################################################################

    def publish_ball_state(self):
        # Check to see whether this node is active
        if not self.is_active:
            return
        
        # Get the current ball position
        current_pos = self.get_current_ball_in_flight_position_as_point_stamped()

        # Publish the ball state
        ball_state_msg = BallStateMessage()
        ball_state_msg.path = self.ball_path
        ball_state_msg.current_pos = current_pos

        self.ball_state_pub.publish(ball_state_msg)

        # self.get_logger().info(f"Ball position: {current_pos.point.x:.2f}, {current_pos.point.y:.2f}, {current_pos.point.z:.2f}")

    #########################################################################################################
    #                                 Node Management (callbacks etc.)                                      #
    #########################################################################################################

    def hand_state_callback(self, msg):
        # Get the action of the first state in the list
        action = msg.states[0].action.data

        # If the action is 'throw', set the ball's flight in motion
        if action == 'throw':
            self.is_active = True
            self.throw_pos = msg.states[0].pos
            self.throw_vel = msg.states[0].vel
            self.throw_time  = msg.states[0].time.sec + msg.states[0].time.nanosec / 1e9

            current_time_sec = self.get_clock().now().nanoseconds / 1e9

            self.time_offset = current_time_sec - self.throw_time
            # self.get_logger().info(f"Time offset: {self.time_offset:.2f}")

            self.calculate_ball_path()

        else:
            self.is_active = False

    def end_session(self, request, response):
        # The method that's called when a user clicks "End Session" in the GUI
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = BallManagerNode()
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
