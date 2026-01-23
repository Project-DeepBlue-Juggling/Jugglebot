"""
This node will move the platform to "catch" a ball that is predicted to land within the robot's range of motion.
Currently the catch is a basic movement to the predicted landing site, with the platform oriented to be normal 
to the ball's velocity vector.
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from jugglebot_interfaces.msg import BallStateArray, PlatformPoseCommand
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
import numpy as np
from typing import Optional
from scipy.spatial.transform import Rotation as R

class CatchThrownBallNode(Node):
    def __init__(self):
        super().__init__('catch_thrown_ball_node')

        # Initialize state variables
        self.shutdown_flag = False

        # Initialize a service to trigger closing the node
        self.service = self.create_service(Trigger, 'end_session', self.end_session)

        # Subscribe to control_mode_topic to see if this control mode is enabled
        self.control_mode_sub = self.create_subscription(String, 'control_mode_topic', self.control_mode_callback, 10)
        self.catch_thrown_ball_enabled = False

        # Subscribe to /balls to know where balls will land
        self.balls_sub = self.create_subscription(BallStateArray, '/balls', self.balls_callback, 10)

        # Initialize a publisher for the platform pose
        self.platform_pose_publisher = self.create_publisher(PlatformPoseCommand, 'platform_pose_topic', 10)

        # Initialize a timer to publish the platform pose
        self.pub_timer = self.create_timer(0.01, self.publish_catch_pose)
        # Initialize a timer to check the landing state. If the ball has landed, reset the landing state
        self.check_for_landing_timer = self.create_timer(0.1, self.check_whether_landed)
        # Initialize a timer to determine whether the robot ought to move back to the default pose
        self.return_to_default_timer = self.create_timer(0.1, self.return_to_default_pose_if_appropriate)

        # Initialise various variables
        self.xy_range_of_motion = 400.0   # mm (ie. XXX mm in any direction along the catch plane)
        self.initial_plat_height = 565.0  # mm. The initial height of the platform from the base plane
        self.catch_height = self.initial_plat_height + 170  # mm. The height of the catch plane from the base plane 
        self.catch_angle_limit_deg = 30.0         # deg. The maximum angle the robot can move to catch the ball, wrt the z-axis
        self.minimum_time_to_attempt_catch = 0.5  # s. The minimum time before the ball lands to attempt a catch
        self.time_after_catching_to_return_to_default = 1.0  # s. The time after catching the ball to return to the default pose
        self.last_catch_time = None  # The time at which the last catch was attempted

        # Initialise the landing state 
        self.landing_state = {"id": None,  # The id of the ball being caught. Set by ball_prediction_node
                              "pos": None, # The predicted landing position of the ball
                              "vel": None, # The predicted landing velocity of the ball
                              "time_to_land": None, # The time remaining before the ball lands (s)
                              "time_at_land": None, # The time at which the ball will land (ROS2 Time)
                              "catching": None}    # Flag to indicate if the robot is catching the ball
        
        # Initialise the default 'active' pose
        self.default_pose = Pose()
        self.default_pose.position.x = 0.0
        self.default_pose.position.y = 0.0
        self.default_pose.position.z = 170.0 # Note that this is in platform frame
        self.default_pose.orientation.x = 0.0
        self.default_pose.orientation.y = 0.0
        self.default_pose.orientation.z = 0.0
        self.default_pose.orientation.w = 1.0

    def control_mode_callback(self, msg):
        # If the incoming state calls for the catch_thrown_ball_node, enable it
        if msg.data == 'CATCH_THROWN_BALL_NODE' and not self.catch_thrown_ball_enabled:
            self.get_logger().info('catch_thrown_ball_node enabled')
            self.catch_thrown_ball_enabled = True

        elif msg.data != 'CATCH_THROWN_BALL_NODE' and self.catch_thrown_ball_enabled:
            self.get_logger().info('catch_thrown_ball_node disabled')
            self.catch_thrown_ball_enabled = False

    def balls_callback(self, msg: BallStateArray):
        """
        Callback to handle incoming ball state data.
        Process the ball data to determine if the robot can catch a ball.
        If ball is deemed catchable, compile the landing state (pos, vel, time_at_land) 
        and save it for use in another method

        Args:
            msg: BallStateArray message containing ball state data.
        """
        # Check if the catch_thrown_ball control mode is enabled
        if not self.catch_thrown_ball_enabled:
            return
        
        # Check if the ball has already been deemed uncatchable
        if self.landing_state["catching"] is False:
            return
        
        # Check that there's only one ball being tracked
        if len(msg.balls) != 1:
            self.get_logger().warn("More than one ball detected. This node can only handle one ball at a time.", throttle_duration_sec=1.0)
            return
        
        # Extract the relevant data from the message
        ball = msg.balls[0]
        id = ball.id
        landing_position = ball.landing_position
        landing_velocity = ball.landing_velocity
        time_at_land = ball.time_at_land

        # Calculate how much time remains before the ball lands
        time_to_land_s = time_at_land.sec - self.get_clock().now().to_msg().sec
        time_to_land_ns = time_at_land.nanosec - self.get_clock().now().to_msg().nanosec
        time_to_land = time_to_land_s + time_to_land_ns * 1e-9

        # Log the landing position
        self.get_logger().info(f"Predicted landing position: [{landing_position.x}, {landing_position.y}, {landing_position.z}]"
                               f" in {time_to_land:.2f} seconds.")

        # Check if the ball is catchable based on the landing position and time to land
        catchable = self.check_preliminary_catch_feasibility(id, landing_position, time_to_land)
        if not catchable:
            return
        
        # Compile the landing state
        self.landing_state["id"] = id
        self.landing_state["pos"] = landing_position
        self.landing_state["vel"] = landing_velocity
        self.landing_state["time_to_land"] = time_to_land
        self.landing_state["time_at_land"] = time_at_land
        self.landing_state["catching"] = True # Set the catching flag to True

    #########################################################################################################
    #                                         Auxiliary Methods                                             #
    #########################################################################################################

    def calculate_catch_pose(self) -> Optional[Pose]:
        """
        Calculate the pose the robot should move to in order to catch the ball.
        """        
        # Check if the landing state has been set
        if self.landing_state["id"] is None:
            # self.get_logger().warn("No landing state set. Cannot calculate catch pose.", throttle_duration_sec=2.0)
            return None
        
        # Check that the ball has not been previously deemed uncatchable
        if self.landing_state["catching"] is False:
            return None

        # Calculate the catch orientation based on the landing velocity
        catch_orientation = self.calculate_catch_orientation()
        
        if catch_orientation is None:
            return None
        
        # Calculate the catch pose
        catch_pose = Pose()
        catch_pose.position.x = self.landing_state["pos"].x
        catch_pose.position.y = self.landing_state["pos"].y
        catch_pose.position.z = self.catch_height - self.initial_plat_height # Convert to the platform frame
        catch_pose.orientation = catch_orientation

        return catch_pose

    def calculate_catch_orientation(self) -> Optional[Quaternion]:
        """
        Calculate the orientation the robot should move to in order to catch the ball, based on the landing velocity.
        If the ball will land at an angle greater than catch_angle_limit_deg, deem it uncatchable and return None.
        Otherwise, return the orientation the robot should move to in order to catch the ball as a quaternion.
        """
        reference_vector = np.array([0.0, 0.0, -1.0])  # The reference vector is the z-axis
        velocity_vector = np.array([self.landing_state["vel"].x, self.landing_state["vel"].y, self.landing_state["vel"].z])

        # Normalize the velocity vector
        norm = np.linalg.norm(velocity_vector)
        if norm == 0:
            self.get_logger().warn("Cannot normalize a zero vector.", throttle_duration_sec=2.0)
            return None
        
        velocity_vector_norm = velocity_vector / norm

        # Compute the dot product of the reference vector and the velocity vector
        dot_product = np.dot(reference_vector, velocity_vector_norm)

        # Compute the angle between the reference vector and the velocity vector
        angle_rad = np.arccos(dot_product)
        angle_deg = np.degrees(angle_rad)

        # Check if the angle is within the catch angle limit
        if angle_deg > self.catch_angle_limit_deg:
            self.get_logger().warn(f"Ball {self.landing_state['id']} will land at an angle of {angle_deg} deg. Cannot catch.", throttle_duration_sec=2.0)
            # Update the landing state to indicate that the ball is not being caught
            self.landing_state["catching"] = False
            return None
        
        # Compute the rotation axis (to be normal to the ball's velocity vector)
        rotation_axis = np.cross(reference_vector, velocity_vector_norm)
        rotation_axis_norm = rotation_axis / np.linalg.norm(rotation_axis)

        # Compute the quaternion representing the orientation
        rotation = R.from_rotvec(angle_rad * rotation_axis_norm)
        quaternion = rotation.as_quat()

        return Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])

    def check_preliminary_catch_feasibility(self, id, catch_pos, time_to_land) -> bool:
        """
        Check if the ball is catchable based purely on the timing and landing position (range of motion)
        More comprehensive checks (eg. angle of platform) will be done downstream as appropriate.

        Args:
            catch_pos: The predicted landing position of the ball
            time_to_land: The time remaining before the ball lands

        Returns:
            bool: True if the ball passes these checks, False otherwise
        """

        # If the ball is a new one (id doesn't match the last saved id), check that there's enough time to attempt a catch
        if id != self.landing_state["id"]:
            # Check that there's enough time to attempt a catch
            if time_to_land < self.minimum_time_to_attempt_catch:
                self.get_logger().warn(f"Time to land is too short to attempt a catch. Time to land: {time_to_land}", throttle_duration_sec=1.0)
                # Reset the landing state
                self.reset_landing_state()
                return False
        
        # Check if the predicted landing site is within the robot's range of motion
        if abs(catch_pos.x) > self.xy_range_of_motion or abs(catch_pos.y) > self.xy_range_of_motion:
            self.get_logger().warn(f"Ball landing pos is out of range. Landing pos: "
                                   f"[{catch_pos.x}, {catch_pos.y}, {catch_pos.z}]", throttle_duration_sec=1.0)
            return False
        
        return True

    def reset_landing_state(self):
        """
        Reset the landing state.
        """
        self.landing_state["id"] = None
        self.landing_state["pos"] = None
        self.landing_state["vel"] = None
        self.landing_state["time_to_land"] = None
        self.landing_state["time_at_land"] = None
        self.landing_state["catching"] = None

    #########################################################################################################
    #                                           Timer Methods                                               #
    #########################################################################################################

    def publish_catch_pose(self):
        """
        Publish the pose the robot should move to in order to catch the ball.
        """
        # Check if the catch_thrown_ball control mode is enabled
        if not self.catch_thrown_ball_enabled:
            return

        # Calculate the catch pose
        catch_pose = self.calculate_catch_pose()
        
        if catch_pose is None:
            return

        # Construct the PlatformPoseCommand
        message = PlatformPoseCommand()
        pose_stamped = PoseStamped()
        pose_stamped.pose = catch_pose

        # Log the catch pose
        self.get_logger().info(f"Moving to catch pose: [{catch_pose.position.x}, {catch_pose.position.y}, {catch_pose.position.z}]"
                               f", [{catch_pose.orientation.x}, {catch_pose.orientation.y}, {catch_pose.orientation.z}, {catch_pose.orientation.w}]")

        # Set the time stamp
        pose_stamped.header.stamp = self.get_clock().now().to_msg()

        message.pose_stamped = pose_stamped
        message.publisher = 'CATCH_THROWN_BALL_NODE'

        self.platform_pose_publisher.publish(message)

    def check_whether_landed(self):
        """
        Check if the ball has landed. If so, reset the landing state.
        """
        if self.landing_state["id"] is None:
            return
        
        # Check if the ball has landed
        if self.get_clock().now().to_msg().sec > self.landing_state["time_at_land"].sec:
            self.get_logger().info(f"Ball {self.landing_state['id']} has landed.")

            # Update the last catch time
            self.last_catch_time = self.get_clock().now().to_msg()

            # Reset the landing state
            self.reset_landing_state()

    def return_to_default_pose_if_appropriate(self):
        """
        Check if the robot should return to the default pose.
        This is done after a certain time has elapsed since the last catch
        and if there is not a new ball to catch.
        """

        # Check if the last catch time has been set
        if self.last_catch_time is None:
            return
        
        # Check that there isn't a new ball to catch
        if self.landing_state["id"] is not None:
            return
        
        # Check if the time since the last catch is greater than the time after catching to return to the default pose
        time_since_catch_s = self.get_clock().now().to_msg().sec - self.last_catch_time.sec
        time_since_catch_ns = self.get_clock().now().to_msg().nanosec - self.last_catch_time.nanosec
        time_since_catch = time_since_catch_s + time_since_catch_ns * 1e-9

        if time_since_catch > self.time_after_catching_to_return_to_default:
            # Publish the default pose
            message = PlatformPoseCommand()
            pose_stamped = PoseStamped()
            pose_stamped.pose = self.default_pose
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            message.pose_stamped = pose_stamped
            message.publisher = 'CATCH_THROWN_BALL_NODE'

            self.platform_pose_publisher.publish(message)

            # Reset the last catch time
            self.last_catch_time = None

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
        self.get_logger().info("Shutting down CatchThrownBallNode...")


def main(args=None):
    rclpy.init(args=args)
    node = CatchThrownBallNode()

    try:
        while rclpy.ok() and not node.shutdown_flag:
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Shutting down.")
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
