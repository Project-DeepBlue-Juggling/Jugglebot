import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from jugglebot_interfaces.msg import HandStateList, BallStateMessage
from geometry_msgs.msg import PointStamped, PoseStamped
from builtin_interfaces.msg import Time
import quaternion  # numpy quaternion
import math

class BallManagerNode(Node):
    def __init__(self):
        super().__init__('ball_manager_node')

        # Subscribe to relevant Topics
        self.hand_pose_sub = self.create_subscription(PoseStamped,'hand_pose_topic', self.hand_pose_callback, 10) # To determine if the ball is held

        # Initialize publishers to relevant Topics
        self.ball_state_pub = self.create_publisher(BallStateMessage, 'ball_state_topic',10)

        # Set up a service to trigger closing the node
        self.service = self.create_service(Trigger, 'end_session', self.end_session)

        # Set up a timer to periodically publish the ball state
        self.publish_timer = self.create_timer(0.1, self.publish_ball_state)

        # Class variables
        self.velocity = 0.0 # {m/s} The velocity of the ball. Used to determine if it has been thrown or not
        self.time_of_flight = 0.0  # {s} The time the ball has been in the air for

            # Flags
        self.is_held = True  # Whether the ball is currently being held by the hand
        self.received_position = False  # Whether the ball has received a position yet

        self.last_hand_pos = {
            'pos': None,  # The last position of the hand
            'ori': None,  # The last orientation of the hand
            'time': None  # The time that the position was published
        }  

        self.g = 9.81  # m/s^2
        self.lowest_catch_pos = -0.5  # {m} The lowest position the hand can catch a ball at (from throw pos). Would ideally be based on the hand's workspace
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

        # Log the current time and throw time
        # self.get_logger().info(f"Current time: {current_time:.2f}, Throw time: {self.throw_time:.2f}")

        # Convert the current time to a Time message
        current_time_msg = Time()
        current_time_msg.sec = int(current_time)
        current_time_msg.nanosec = int((current_time - current_time_msg.sec) * 1e9)

        self.time_of_flight = current_time - self.throw_time

        # If the time of flight exceeds 1.5 seconds, the ball has either been caught or dropped (no throws should last this long... yet)
        if self.time_of_flight > 1.5:
            self.is_held = True # Just put the ball back in the hand for now
            self.velocity = 0.0
            return self.get_last_hand_pos_as_point_stamped()

        # Calculate the position of the ball
        point_msg.point.x = self.throw_pos.x / 1000 + self.throw_vel[0] * self.time_of_flight
        point_msg.point.y = self.throw_pos.y / 1000 + self.throw_vel[1] * self.time_of_flight
        point_msg.point.z = self.throw_pos.z / 1000 + self.throw_vel[2] * self.time_of_flight - 0.5 * self.g * self.time_of_flight**2

        # Calculate the ball velocity. The only velocity that changes is the z velocity, so don't need to re-calculate the whole thing
        vel_z = self.throw_vel[2] - self.g * self.time_of_flight

        # Now calculate the velocity scalar
        self.velocity = (self.throw_vel[0]**2 + self.throw_vel[1]**2 + vel_z**2)**0.5

        # Convert m to mm
        point_msg.point.x *= 1000
        point_msg.point.y *= 1000
        point_msg.point.z *= 1000

        # Set the time stamp
        point_msg.header.stamp = current_time_msg

        # Logging things
        self.get_logger().info(f"Current ball position: {point_msg.point.x:.2f}, {point_msg.point.y:.2f}, {point_msg.point.z:.2f}")
        self.get_logger().info(f"Throw position: {self.throw_pos.x:.2f}, {self.throw_pos.y:.2f}, {self.throw_pos.z:.2f}")
        self.get_logger().info(f"Current ball velocity: {self.velocity:.2f} m/s")
        self.get_logger().info(f"Time of flight: {self.time_of_flight:.2f} s")
        self.get_logger().info("--------------------")

        return point_msg

    def calculate_hand_velocity(self, new_pos_stamped, last_pos_stamped):
        '''Calculate the velocity of the hand based on the change in position and the time delta'''

        # Convert the points to tuples
        new_pos = (new_pos_stamped['pos'].x, new_pos_stamped['pos'].y, new_pos_stamped['pos'].z)
        last_pos = (last_pos_stamped['pos'].x, last_pos_stamped['pos'].y, last_pos_stamped['pos'].z)

        # Calculate the change in position
        squared_diffs = [(a - b) ** 2 for a, b in zip(new_pos, last_pos)]
        
        # Now calculate the euclidean distance
        distance = math.sqrt(sum(squared_diffs))  # {mm}

        # Calculate the time delta
        delta_time = (new_pos_stamped['time'] - last_pos_stamped['time']) * 1000 # {ms}

        # Check that the time delta is not zero
        if delta_time == 0:
            self.get_logger().error('Time delta is zero')
            return 0
        
        # Log the things
        # self.get_logger().info(f"New position: {new_pos}, Last position: {last_pos}")
        # self.get_logger().info(f"Distance: {distance:.2f} mm, Delta time: {delta_time:.2f} ms")
        # self.get_logger().info(f"Hand velocity: {distance / delta_time:.2f} m/s")
        # self.get_logger().info("--------------------")

        # Finally, return the velocity
        return distance / delta_time

    def ori_and_scalar_vel_to_vel_vector(self, ori, scalar_vel):
        '''Convert the orientation and scalar velocity to a 3D velocity vector'''

        # Convert the orientation to a quaternion
        quat = quaternion.quaternion(ori.w, ori.x, ori.y, ori.z)

        # Get the velocity quaternion
        vel_quat = quat * quaternion.quaternion(0, 0, 0, scalar_vel) * quat.conj()

        # Convert the velocity quaternion to a vector
        vel_vector = [vel_quat.x, vel_quat.y, vel_quat.z]

        return vel_vector

    def get_last_hand_pos_as_point_stamped(self):
        '''Convert the last hand position to a PointStamped message'''

        # Initialize the message
        point_msg = PointStamped()

        # Convert the position to mm
        point_msg.point = self.last_hand_pos['pos']

        # Set the time stamp
        point_msg.header.stamp.sec = int(self.last_hand_pos['time'])
        point_msg.header.stamp.nanosec = int((self.last_hand_pos['time'] - point_msg.header.stamp.sec) * 1e9)

        return point_msg

    #########################################################################################################
    #                                              Publishing                                               #
    #########################################################################################################

    def publish_ball_state(self):
        # Check to see whether this node is active
        if not self.received_position:
            return

        if self.is_held:
            # If the ball is being held, its position is coincident with the hand's (perfect hold assumption)
            current_pos = self.get_last_hand_pos_as_point_stamped()

        else: # If ball is in the air
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

    def hand_pose_callback(self, msg):
        '''Calculate the hand velocity and use it to determine if the ball has been thrown.
        Assume that holding is perfect. Then if hand_vel > ball_vel, the ball is still being held (and ball_vel = hand_vel).
        Once the hand slows down, the ball has been thrown.
        '''

        # Get the hand position and orientation
        hand_pos = msg.pose.position
        hand_ori = msg.pose.orientation

        # Get the time that this message was published (in seconds)
        hand_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9

        # If this is the first hand position we've received, we can't calculate the velocity
        if self.last_hand_pos['pos'] is None:
            self.last_hand_pos['pos'] = hand_pos
            self.last_hand_pos['ori'] = hand_ori
            self.last_hand_pos['time'] = hand_time

            self.received_position = True  # We've received a position now
            return
        
        new_hand_pos_stamped = {
            'pos': hand_pos,
            'ori': hand_ori,
            'time': hand_time
        }
        
        # Now we can calculate the velocity of the hand
        hand_vel = self.calculate_hand_velocity(new_hand_pos_stamped, self.last_hand_pos)

        # Log the velocities
        self.get_logger().info(f"Hand velocity: {hand_vel:.2f} m/s, Ball velocity: {self.velocity:.2f} m/s. Is held? {self.is_held}")

        # Compare the hand velocity against ball velocity
        if hand_vel > self.velocity and self.is_held:
            self.velocity = hand_vel # Update the ball velocity to match the hand velocity
            
        elif hand_vel < self.velocity and self.is_held: # If the hand has slowed down and the ball is held, the ball has been thrown
            self.is_held = False

            # Update the throw properties now that the ball has been thrown
            self.throw_pos = self.last_hand_pos['pos']
            self.throw_vel = self.ori_and_scalar_vel_to_vel_vector(self.last_hand_pos['ori'], self.velocity)
            self.throw_time = self.last_hand_pos['time']

        # Update the last hand position
        self.last_hand_pos = new_hand_pos_stamped

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
