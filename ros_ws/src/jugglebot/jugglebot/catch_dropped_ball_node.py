"""
This node will move the platform to always be underneath a tracked ball.
The platform COM will stay at a fixed height and will always be horizontal.
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from std_srvs.srv import Trigger
from jugglebot_interfaces.msg import MocapDataMulti, PlatformPoseMessage, HandInputPosMsg, RobotState
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped
import numpy as np
import threading
from collections import deque
from typing import Optional
import time
from .hand_trajectory_generator import HandTrajGenerator

class CatchDroppedBallNode(Node):
    # Define some constants
    HAND_SPOOL_EFFECTIVE_RADIUS = 5.134 # {mm} Measured experimentally. (stroke / [revs_to_move_stroke * 2 * pi])
    LINEAR_GAIN = 1000 / (np.pi * HAND_SPOOL_EFFECTIVE_RADIUS * 2) # {rev/m}

    # Position of the ball when the hand is at the top of the stroke. Measured experimentally.
    BALL_Z_COORDINATE_AT_TOP_OF_STROKE = 828.2 # {mm}. (at "TOP_POSITION" revs)
    BALL_TOP_STROKE_POSITION_REVS = 9.858 # {revs}
    MAX_HAND_POSITION = 11.1 # {revs} The maximum position the hand can move to (really around 11.4, but good to be safe)

    def __init__(self):
        super().__init__('catch_dropped_ball_node')

        # Create a callback group to allow for reentrant callbacks
        self.callback_group = ReentrantCallbackGroup()

        # Initialize locks
        self.state_lock = threading.Lock()
        self.mocap_lock = threading.Lock()

        # Initialize the hand trajectory generator
        self.hand_traj_gen = HandTrajGenerator()

        # Initialize a service to trigger closing the node
        self.service = self.create_service(Trigger, 'end_session', self.end_session)

        # Subscribe to control_mode_topic to see if this control mode is enabled
        self.control_mode_sub = self.create_subscription(String, 'control_mode_topic', self.control_mode_callback, 10)
        self.is_enabled = False

        # Subscribe to /mocap_data to know where the ball is, and a variables to store the latest position
        self.mocap_data_sub = self.create_subscription(MocapDataMulti, 'mocap_data', self.mocap_data_callback, 10,
                                                       callback_group=self.callback_group)
        self.last_mocap_data_time = None
        self.number_of_marker_frames_to_save = 300
        self.marker_coordinates = deque(maxlen=self.number_of_marker_frames_to_save) # Store the last n marker coordinates {mm}

        # Subscribe to the robot state
        self.robot_state_sub = self.create_subscription(RobotState, 'robot_state', self.robot_state_callback, 10,
                                                        callback_group=self.callback_group)
        self.last_known_hand_state = None

        # Initialize a publisher for the platform pose and hand trajectory
        self.platform_pose_publisher = self.create_publisher(PlatformPoseMessage, 'platform_pose_topic', 10)
        self.hand_trajectory_publisher = self.create_publisher(HandInputPosMsg, 'hand_trajectory', 10)

        # Initialize a service to prime the hand for catching a ball
        self.prime_hand_service = self.create_service(Trigger, 'prime_hand', self.prime_hand_callback)

        # Initialize timers
        self.pub_timer = self.create_timer(0.01, self.publish_platform_pose) # To publish platform pose
        self.clear_ball_pos_timer = self.create_timer(0.1, self.clear_ball_pos_record) # To clear the ball position if it hasn't been updated in a while

        # Initialise various variables
        self.xy_range_of_motion = 400.0   # mm (ie. XXX mm in any direction along the catch plane)
        self.initial_plat_height = 565.0  # mm. The initial height of the platform COM from the base plane
        self.catch_height = self.initial_plat_height + 170  # mm. The height of the catch plane from the base plane. This is where the platform COM will be.
        self.time_after_losing_ball_to_return_to_default = 1.0  # s. The time after losing the ball to return to the default pose
        
        self.MOCAP_OFFSET = [-4.5, -70.0] # mm. THIS SHOULDN'T BE NECESSARY!! Applied in mocap_data_callback

        # Initialise the default 'active' pose
        self.default_pose = Pose()
        self.default_pose.position.x = 0.0
        self.default_pose.position.y = 0.0
        self.default_pose.position.z = 170.0 # Note that this is in platform frame
        self.default_pose.orientation.x = 0.0
        self.default_pose.orientation.y = 0.0
        self.default_pose.orientation.z = 0.0
        self.default_pose.orientation.w = 1.0

    #########################################################################################################
    #                                      Subscription Callbacks                                           #
    #########################################################################################################

    def control_mode_callback(self, msg):
        # If the incoming state calls for the catch_dropped_ball_node, enable it
        if msg.data == 'CATCH_DROPPED_BALL_NODE' and not self.is_enabled:
            self.get_logger().info('catch_dropped_ball_node enabled')
            self.is_enabled = True

        elif msg.data != 'CATCH_DROPPED_BALL_NODE' and self.is_enabled:
            self.get_logger().info('catch_dropped_ball_node disabled')
            self.is_enabled = False

    def mocap_data_callback(self, msg: MocapDataMulti):
        """
        Callback to handle incoming mocap data.
        Assume that there is only ever one 'unlabelled marker' at a time.

        Args:
            msg: MocapDataMulti message containing mocap data.
        """
        with self.mocap_lock:
            # Assume there will only be one unlabelled marker at a time
            if len(msg.unlabelled_markers) != 1:
                return
            
            # Get the ball position
            position = msg.unlabelled_markers[0].position

        # Apply the offset
        position.x += self.MOCAP_OFFSET[0]
        position.y += self.MOCAP_OFFSET[1]
        
        # Update the ball position
        # If the ball is at or below the catch plane, or if its x/y coordinates are out of reach, don't update the ball position
        if position.z > self.catch_height and self.check_whether_pos_is_reachable(np.array([position.x, position.y])):
            self.marker_coordinates.append([position.x, position.y, position.z]) 

        # Update the time of the last mocap data
        self.last_mocap_data_time = time.time()

    def robot_state_callback(self, msg: RobotState):
        """
        Callback to handle incoming robot state data.
        Simply updates the last known hand state.

        Args:
            msg: RobotState message containing robot state data.
        """
        with self.state_lock:
            self.last_known_hand_state = msg.motor_states[6]

    #########################################################################################################
    #                                         Service Callbacks                                             #
    #########################################################################################################

    def prime_hand_callback(self, request, response):
        """
        Service callback to prime the hand for catching a ball.
        This will move the hand to the top of the stroke and await a dropped ball.
        """
        # First, smooth move the hand to the top of the stroke
        result_outcome, result_message = self.smooth_move_to_target(self.BALL_TOP_STROKE_POSITION_REVS, duration=2.0)

        # If any errors occurred, return the error message
        if not result_outcome:
            response.success = False
            response.message = result_message
            return response
        
        # Now wait for the hand to reach the top of its stroke
        timeout_duration = 5.0 # {s}
        position_tolerance = 0.5 # {revs}
        start_time = self.get_clock().now()
        
        while True:
            with self.state_lock:
                current_pos = self.last_known_hand_state.pos_estimate if self.last_known_hand_state else None 

            if current_pos is not None and np.abs(current_pos - self.BALL_TOP_STROKE_POSITION_REVS) < position_tolerance:
                self.get_logger().info("Hand has reached the top of its stroke.")
                break

            if (self.get_clock().now() - start_time).nanoseconds / 1e9 > timeout_duration:
                response.success = False
                response.message = "Timeout reached while waiting for the hand to reach the top of its stroke."
                return response
            
            time.sleep(0.1)
        
        # Now wait for the ball to be dropped
        ball_dropped = False
        ball_drop_time = None

        while not ball_dropped:
            ball_dropped, drop_height, ball_drop_time = self.detect_ball_drop()

            self.get_logger().info("Waiting for ball to drop...", throttle_duration_sec=2.0)
            time.sleep(0.1)

        # Convert the drop height to m
        drop_height_m = drop_height / 1000

        # Generate the hand trajectory to catch the ball
        time_cmd, pos, vel, tor = self.get_drop_catch_trajectory(drop_height_m, ball_drop_time)

        # Publish the hand trajectory
        self.publish_hand_trajectory(time_cmd, pos, vel, tor)

        response.success = True
        response.message = f"Ball has been dropped from height: {drop_height:.2f} mm. Hand moving to catch the ball."
        return response

    #########################################################################################################
    #                                Hand Trajectory (Generated on Demand)                                  #
    #########################################################################################################

    def publish_hand_trajectory(self, time_cmd, pos, vel, tor):
        """
        Publish a generated hand trajectory.

        Args:
            time_cmd: Commanded time sequence for the trajectory, as ROS2 time (builtin_interfaces/Time)
            pos: Position commands
            vel: Velocity commands
            tor: Torque commands
        """
        try:
            message = HandInputPosMsg()
            message.time_cmd = time_cmd
            message.input_pos = pos
            message.vel_ff = vel
            message.torque_ff = tor

            self.hand_trajectory_publisher.publish(message)

        except Exception as e:
            self.get_logger().error(f"Error publishing hand trajectory: {e}")

    def smooth_move_to_target(self, target_pos, duration=2.0):
        """
        Generate a hand trajectory to move smoothly to a target position.

        Args:
            target_pos: The target position to move to
            duration: The time it should take to move to the target position
        """
        result_outcome = None
        result_message = None

        vel_tolerance = 0.2 # {rev/s} How close the hand velocity needs to be to zero to be considered stationary

        try:
            self.get_logger().info(f"Moving hand to target position: {target_pos} revs")

            with self.state_lock:
                # Check that we know where the hand currently is and that it's stationary (within a tolerance)
                if self.last_known_hand_state is None:
                    result_outcome = False
                    result_message = "Hand state is unknown. Cannot move hand."
                    return result_outcome, result_message
                
                elif np.abs(self.last_known_hand_state.vel_estimate) > vel_tolerance:
                    result_outcome = False
                    result_message = f"Hand is not stationary. Current velocity: {self.last_known_hand_state.vel_estimate:.2f} rev/s"
                    return result_outcome, result_message
                
            # Check that we've gotten a valid target position
            if not 0 <= target_pos <= self.MAX_HAND_POSITION:
                result_outcome = False
                result_message = f"Invalid target position: {target_pos} revs"
                return result_outcome, result_message
            
            # Generate the trajectory
            time_cmd, pos, vel, tor = self.get_smooth_move_trajectory(target_pos, duration)

            # Check that the trajectory generation was successful
            if time_cmd is None:
                result_outcome = False
                result_message = "Error generating hand trajectory"
                return result_outcome, result_message

            # Publish the trajectory
            self.publish_hand_trajectory(time_cmd, pos, vel, tor)

            result_outcome = True
            result_message = f"Hand moving to target position: {target_pos} revs"

            return result_outcome, result_message
        
        except Exception as e:
            result_outcome = False
            result_message = f"Error moving hand to target position: {e}"
            return result_outcome, result_message

    def get_drop_catch_trajectory(self, drop_height_m, drop_time):
        """
        Generate a hand trajectory for catching a dropped ball.

        Args:
            drop_height_m: The height from which the ball is dropped (m)
            drop_time: The time it takes for the ball to drop (s)
        """
        start_time = time.perf_counter() # For timing how long trajectory generation takes

        # Set the parameters for the hand trajectory generation, and get the trajectory
        self.hand_traj_gen.set_flight_parameters(throw_height=drop_height_m)
        time_cmd, pos, vel, tor = self.hand_traj_gen.get_catch_trajectory()

        # Get the time (relative to the start of the catch movement) and position of the catch
        catch_time_from_start_of_hand_movement, catch_pos = self.hand_traj_gen.get_catch_time_and_position()

        # Multiply the position and velocity commands by the linear gain to convert from m to rev
        catch_pos *= self.LINEAR_GAIN
        pos = [p * self.LINEAR_GAIN for p in pos]
        vel = [v * self.LINEAR_GAIN for v in vel]

        # Calculate how long it will take for the ball to reach the catch position from when it was dropped
        time_to_catch_from_drop = np.sqrt(2 * drop_height_m / 9.81)

        # Calculate when the hand needs to start moving to intercept the ball at the catch position
        time_to_start_moving = time_to_catch_from_drop - catch_time_from_start_of_hand_movement # {s}

        # Shift the time commands to start at the catch time
        time_cmd = [t + time_to_start_moving for t in time_cmd]

        # Put the time commands into ROS2 time by adding them to the 'time of drop'
        time_cmd_ros = [(drop_time + Duration(seconds=t)).to_msg() for t in time_cmd]

        end_time = time.perf_counter()

        # Log the time taken to generate the trajectory
        self.get_logger().info(f"Time taken to generate trajectory: {(end_time - start_time)*1000:.2f} ms")

        return time_cmd_ros, pos, vel, tor

    def get_smooth_move_trajectory(self, target_pos, duration=2.0):
        """
        Generate a hand trajectory to move smoothly to a target position.

        Args:
            target_pos: The target position to move to
            duration: The time it should take to move to the target position
        """
        # Get the current position of the hand
        with self.state_lock:
            if self.last_known_hand_state is None:
                return None, None, None, None
            
            current_pos = self.last_known_hand_state.pos_estimate

        # Generate the trajectory
        time_cmd, pos, vel, acc, tor = self.hand_traj_gen.get_smooth_move_trajectory(start_pos=current_pos,
                                                                                target_pos=target_pos, 
                                                                                duration=duration)
        
        # Convert time_cmd to builtin_interfaces/Time, offset by the current time
        time_cmd_ros = [
            (self.get_clock().now() + Duration(seconds=t)).to_msg() for t in time_cmd
             ]

        return time_cmd_ros, pos, vel, tor

    #########################################################################################################
    #                                         Auxiliary Methods                                             #
    #########################################################################################################

    def detect_ball_drop(self, threshold=4.0):
        """
        Detect when the ball has been dropped.
        This works by checking if the z coordinate of the ball changes by more than a threshold (in mm).

        Args:
            threshold: The threshold for detecting a ball

        Returns:
            dropped: True if the ball has been dropped, False otherwise
            drop_height: The height from which the ball was dropped (mm)
            drop_time: The time at which the ball was dropped (ROS2 time)
        """

        drop_height = None # The height from which the ball was dropped {mm}
        drop_time = None # The time at which the ball was dropped {ROS2 time}

        with self.mocap_lock:
            # Ensure we have at least two coordinates to compare
            if len(self.marker_coordinates) < 2:
                return False, None, None
            
            # Get the first and last coordinates
            z_first = self.marker_coordinates[0][2]
            z_last = self.marker_coordinates[-1][2]
        
        # Check if the difference is greater than the threshold
        if np.abs(z_last - z_first) > threshold:
            # Calculate how high the ball is dropping from the top of the stroke
            drop_height = z_first - self.BALL_Z_COORDINATE_AT_TOP_OF_STROKE # {mm}

            # Get the time at which the ball was dropped
            drop_time = self.get_clock().now()

            self.get_logger().info(f"Ball dropped from height: {drop_height:.2f} mm at time: {drop_time.to_msg()}")
            return True, drop_height, drop_time
        else:
            return False, None, None

    def get_catch_pose(self) -> Optional[Pose]:
        """
        Get the pose the robot should move to in order to stay under the ball.
        If no ball position is known, put the platform at the origin of the catch plane.
        """

        # Get the catch pose
        catch_pose = Pose()

        # If we know where the ball is, put the platform COM at the same x, y coordinates
        if self.marker_coordinates[0] is not None:
            catch_pose.position.x = self.marker_coordinates[-1][0]
            catch_pose.position.y = self.marker_coordinates[-1][1]
        else:
            catch_pose.position.x = 0.0
            catch_pose.position.y = 0.0       

        catch_pose.position.z = self.catch_height - self.initial_plat_height # Convert to the platform frame
        catch_pose.orientation = self.default_pose.orientation

        return catch_pose

    def check_whether_pos_is_reachable(self, pos: np.ndarray) -> bool:
        """
        Check whether a given position is reachable by the platform.
        """
        return np.linalg.norm(pos[0:2]) <= self.xy_range_of_motion

    #########################################################################################################
    #                                           Timer Methods                                               #
    #########################################################################################################

    def publish_platform_pose(self):
        """
        Publish the pose the robot should move to in order to stay under the ball.
        """
        # Check if this control mode is enabled
        if not self.is_enabled:
            return

        # Get the pose that we need the platform to move to in order to stay under the ball
        catch_pose = self.get_catch_pose()

        # Construct the PlatformPoseMessage
        message = PlatformPoseMessage()
        pose_stamped = PoseStamped()
        pose_stamped.pose = catch_pose

        # Set the time stamp
        pose_stamped.header.stamp = self.get_clock().now().to_msg()

        message.pose_stamped = pose_stamped
        message.publisher = 'CATCH_DROPPED_BALL_NODE'

        self.get_logger().info(f"Publishing platform pose: {message.pose_stamped.pose.position.x}, {message.pose_stamped.pose.position.y}, {message.pose_stamped.pose.position.z}", throttle_duration_sec=0.2)

        self.platform_pose_publisher.publish(message)

    def clear_ball_pos_record(self):
        """
        Clear the ball position record if it hasn't been updated in a while.
        """
        if (self.last_mocap_data_time is not None and 
            time.time() - self.last_mocap_data_time > self.time_after_losing_ball_to_return_to_default):
            self.marker_coordinates.clear()

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


def main(args=None):
    rclpy.init(args=args)
    node = CatchDroppedBallNode()

    # Use a MultiThreadedExecutor to allow for concurrent execution of callbacks
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)

    try:
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Shutting down.")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
