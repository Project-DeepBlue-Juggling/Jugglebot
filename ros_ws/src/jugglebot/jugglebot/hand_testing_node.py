'''
This node is used to generate and publish different hand trajectories. Available trajectories include:
- Throwing a ball to a specified height, with either just the throw or both the throw and catch
- Catching a ball that has been thrown to a specified height
- Smoothly moving the hand to a specified position
'''

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from builtin_interfaces.msg import Time
from std_srvs.srv import Trigger
from jugglebot_interfaces.msg import HandInputPosMsg, RobotState, MocapDataMulti
from jugglebot_interfaces.srv import SetFloat, SetString
import numpy as np
import threading
from collections import deque
import time
from .hand_trajectory_generator import HandTrajGenerator

class HandTestingNode(Node):
    # Position of the ball when the hand is at the top of the stroke. Measured experimentally.
    BALL_Z_COORDINATE_AT_TOP_OF_STROKE = 828.2 # {mm}. (at "TOP_POSITION" revs)
    BALL_TOP_STROKE_POSITION_REVS = 9.858 # {revs}

    def __init__(self):
        super().__init__('hand_testing_node')

        # Create a callback group to allow for concurrent execution of callbacks
        self.callback_group = ReentrantCallbackGroup()

        # Initialize locks
        self.state_lock = threading.Lock()
        self.mocap_lock = threading.Lock()

        # Set up a service to trigger closing the node
        self.end_session_service = self.create_service(Trigger, 'end_session', self.end_session)

        # Initialize the hand trajectory generator
        self.hand_traj_gen = HandTrajGenerator()

        # Set up a service client to put the hand into the correct control mode
        self.hand_control_mode_client = self.create_client(SetString, 'set_hand_state', callback_group=self.callback_group)

        # Set up a publisher to publish the hand trajectory
        self.hand_trajectory_publisher = self.create_publisher(HandInputPosMsg, 'hand_trajectory', 10)

        # Subscribe to /robot_state to know where the hand currently is
        self.robot_state_subscription = self.create_subscription(RobotState, 'robot_state', self.robot_state_callback, 10,
                                                                 callback_group=self.callback_group)
        
        # Subscribe to /mocap_data to know when to catch a ball
        self.mocap_data_subscription = self.create_subscription(MocapDataMulti, 'mocap_data', self.mocap_data_callback, 10,
                                                                callback_group=self.callback_group)

        # Initialize variables to store data from the subscriptions
        self.last_known_hand_state = None
        self.number_of_marker_frames_to_save = 300
        self.marker_coordinates = deque(maxlen=self.number_of_marker_frames_to_save) # Coordinates are in mm

        # Initialize service servers
        # Don't need to use callback_group for these services since they should only be called one at a time
        self.throw_service = self.create_service(SetFloat, 'throw', self.throw_callback)
        self.smoothly_move_to_target_service = self.create_service(SetFloat, 'smoothly_move_to_target', self.smoothly_move_to_target_callback)
        self.prime_hand_service = self.create_service(Trigger, 'prime_hand', self.prime_hand_callback)

    #########################################################################################################
    #                                           Service Callbacks                                           #
    #########################################################################################################

    def throw_callback(self, request, response):
        '''Callback function for the throw service'''

        throw_height = request.data

        if throw_height < 0:
            response.success = False
            response.message = f"Throw height must be non-negative. Throw height received: {throw_height}"
            return response
        
        elif throw_height > 1.8:
            response.success = False
            response.message = f"Throw height must be less than 1.8 m. Throw height received: {throw_height}"
            return response

        # Get the trajectory
        time_cmd, pos, vel, tor = self.get_throw_catch_traj(throw_height_m=throw_height)

        # Publish the trajectory for can_interface_node to follow
        self.publish_generated_trajectory(time_cmd, pos, vel, tor)

        response.success = True
        response.message = f"Throw height set to {throw_height} m. Throwing trajectory published. Throw should begin imminently."
        return response

    def smoothly_move_to_target_callback(self, request, response):
        '''Callback function for the smoothly_move_to_target service'''
        target_pos = request.data

        # Move the hand smoothly to the target position
        response.success, response.message = self.smooth_move_to_target(target_pos)

        response.success = True
        response.message = f"Smoothly moving hand to target position {target_pos} revs. Trajectory published."
        return response

    def prime_hand_callback(self, request, response):
        '''
        Prime the hand for catching a dropped ball. This involves 'listening' to /mocap_data and 'watching' for a ball to drop.
        '''
        # First, smoothly move the hand to the top of the stroke
        response.success, response.message = self.smooth_move_to_target(self.BALL_TOP_STROKE_POSITION_REVS)

        # If any errors occurred, return immediately
        if not response.success:
            return response
        
        # Wait for the hand to reach the top of the stroke
        timeout_duration = 5.0   # {s}
        position_tolerance = 0.5 # {revs}

        start_time = self.get_clock().now()
        while True:
            with self.state_lock:
                current_pos = self.last_known_hand_state.pos_estimate if self.last_known_hand_state else None
            
            if current_pos is not None and np.abs(current_pos - self.BALL_TOP_STROKE_POSITION_REVS) < position_tolerance:
                self.get_logger().info("Hand has reached the top of the stroke.")
                break
            
            if (self.get_clock().now() - start_time).nanoseconds / 1e9 > timeout_duration:
                response.success = False
                response.message = "Timeout waiting for hand to reach top of stroke."
                return response
            
            time.sleep(0.1)

        # Now, wait for the ball to drop
        ball_dropped = False
        drop_time = None
        while not ball_dropped:
            ball_dropped, drop_height, drop_time = self.detect_ball_drop()
            self.get_logger().info("Waiting for ball to drop...", throttle_duration_sec=2.0)
            time.sleep(0.1)

        # Convert the drop height to m
        drop_height_m = drop_height / 1000 # {m}

        # Generate the catch trajectory
        time_cmd, pos, vel, tor = self.get_drop_catch_traj(drop_height_m=drop_height_m, drop_time=drop_time)
        
        # Log the trajectory
        # self.get_logger().info(f"Generated catch trajectory: {time_cmd}, {pos}, {vel}, {tor}")

        # Publish the trajectory for can_interface_node to follow
        self.publish_generated_trajectory(time_cmd, pos, vel, tor)

        response.success = True
        response.message = f"Ball has been dropped from a height of {drop_height:.2f} mm. Catch trajectory published."
        return response


    #########################################################################################################
    #                                            Topic Callbacks                                            #
    #########################################################################################################

    def robot_state_callback(self, msg):
        '''Callback function for the robot_state topic'''
        with self.state_lock:
            # Extract the hand position from the message
            self.last_known_hand_state = msg.motor_states[6]

    def mocap_data_callback(self, msg):
        '''Callback function for the mocap_data topic'''
        with self.mocap_lock:
            # Assume there will only be one unlabelled marker, which is the ball
            if len(msg.unlabelled_markers) != 1:
                return
            
            self.marker_coordinates.append(msg.unlabelled_markers[0].position)

    #########################################################################################################
    #                                   Trajectory (Generated on Demand)                                    #
    #########################################################################################################
    
    def publish_generated_trajectory(self, time_cmd, pos, vel, tor, timeout=5.0):
        '''
        Publish the generated trajectory after ensuring the hand is in CLOSED_LOOP_CONTROL mode.

        Parameters:
        - time_cmd: Commanded time sequence for the trajectory, as ROS2 time (builtin_interfaces/Time)
        - pos: Position commands
        - vel: Velocity commands
        - tor: Torque commands
        - timeout: Maximum time to wait for the hand to enter CLOSED_LOOP_CONTROL mode (in seconds)
        '''
        try:
            # Check if the hand is already in CLOSED_LOOP_CONTROL mode
            if not self.last_known_hand_state or self.last_known_hand_state.current_state != 8:
                self.get_logger().info("Hand not in CLOSED_LOOP_CONTROL. Requesting state change...")
                
                # Create and send the service request to change the hand state
                request = SetString.Request()
                request.data = "CLOSED_LOOP_CONTROL"
                self.hand_control_mode_client.call_async(request)
                
                # Wait until the hand is in CLOSED_LOOP_CONTROL mode or until timeout
                start_time = self.get_clock().now()
                while True:
                    with self.state_lock:
                        current_state = self.last_known_hand_state.current_state if self.last_known_hand_state else None
                    
                    if current_state == 8:
                        self.get_logger().info("Hand is now in CLOSED_LOOP_CONTROL mode.")
                        break
                    
                    if (self.get_clock().now() - start_time).nanoseconds / 1e9 > timeout:
                        self.get_logger().error("Timeout waiting for hand to enter CLOSED_LOOP_CONTROL mode.")
                        return
                    
                    time.sleep(0.1)

            # Publish the full trajectory
            msg = HandInputPosMsg()
            msg.time_cmd = time_cmd
            msg.input_pos = pos
            msg.vel_ff = vel
            msg.torque_ff = tor
            self.hand_trajectory_publisher.publish(msg)
            self.get_logger().info("Published generated trajectory.")

        except Exception as e:
            self.get_logger().error(f"Error occurred while publishing trajectory: {e}")

    def get_throw_catch_traj(self, throw_height_m=0.3, linear_gain_factor=1.0):
        '''Get the trajectory of the full throw/catch motion'''
        start_time = time.perf_counter()

        self.hand_traj_gen.set_flight_parameters(throw_height=throw_height_m)

        # Get the trajectory
        time_cmd, pos, vel, tor = self.hand_traj_gen.get_full_trajectory()

        end_time = time.perf_counter()
        
        # Log the time taken to generate the trajectory in milliseconds
        self.get_logger().info(f"Trajectory generation took {(end_time - start_time) * 1000} ms.")

        return time_cmd, pos, vel, tor

    def get_drop_catch_traj(self, drop_height_m, drop_time, linear_gain_factor=1.0):
        '''Get the trajectory of the catch motion'''
        start_time = time.perf_counter()

        self.hand_traj_gen.set_flight_parameters(throw_height=drop_height_m)

        # Get the trajectory
        time_cmd, pos, vel, tor = self.hand_traj_gen.get_catch_trajectory()

        # Get the time (relative to the start of the catch movement) and position of the catch event
        catch_time_from_start_of_hand_movement, catch_pos = self.hand_traj_gen.get_catch_time_and_position()

        # Calculate how long it will take for the ball to reach the catch position from when it was dropped
        time_to_catch_from_drop = np.sqrt(8 * drop_height_m / 9.81) # {s} # NOTE NOTE NOTE Should be "2 * " not "8 * "!!!

        # Calculate when the hand needs to start moving to intercept the ball at the catch position
        time_to_start_moving = time_to_catch_from_drop - catch_time_from_start_of_hand_movement # {s}

        offset = 0.0 # {s} 

        # Shift the time commands to start at the catch time
        time_cmd = [t + time_to_start_moving - offset for t in time_cmd]

        # Put these time commands into ROS2 time by adding them to the drop time
        time_cmd_ros = [(drop_time + Duration(seconds=t)).to_msg() for t in time_cmd]

        # Log some things
        # self.get_logger().info(f'Command times: {time_cmd_ros}')
        self.get_logger().info(f'Catch time from start of hand movement: {catch_time_from_start_of_hand_movement:.2f} s')
        self.get_logger().info(f'Time to catch from drop: {time_to_catch_from_drop:.2f} s')
        self.get_logger().info(f'Time to start moving: {time_to_start_moving:.2f} s')

        end_time = time.perf_counter()
        
        # Log the time taken to generate the trajectory in milliseconds
        self.get_logger().info(f"Trajectory generation took {(end_time - start_time) * 1000} ms.")

        return time_cmd_ros, pos, vel, tor

    def smooth_move_to_target(self, target_pos, duration=2.0):
        '''Move the hand smoothly to a target position'''
        result_outcome = None
        result_message = None

        try:
            # Log this
            self.get_logger().info(f"Moving hand to target position {target_pos} revs.")

            with self.state_lock:
                # Check that we know where the hand currently is and that it's stationary (within a tolerance)
                vel_tol = 0.2 # {rev/s}
                if self.last_known_hand_state is None:
                    result_outcome = False
                    result_message = "Hand state is unknown. Please wait for the robot to report its position."
                    return result_outcome, result_message
                
                elif np.abs(self.last_known_hand_state.vel_estimate) > vel_tol:
                    result_outcome = False
                    result_message = f"Hand is not stationary. Please wait for the hand to stop moving. Current velocity: {self.last_known_hand_state.vel_estimate:.2f} rev/s"
                    return result_outcome, result_message
                    
            # Check that we've gotten a valid target position
            if target_pos < 0:
                result_outcome = False
                result_message = f"Target position must be non-negative. Target position received: {target_pos}"
                return result_outcome, result_message
            
            elif target_pos > 11.1:
                result_outcome = False
                result_message = f"Target position must be less than 11.1 revs. Target position received: {target_pos}"
                return result_outcome, result_message
                
            # Get the trajectory
            time_cmd, pos, vel, tor = self.get_smooth_move_traj(target_pos, duration=duration)

            # Publish the trajectory for can_interface_node to follow
            self.publish_generated_trajectory(time_cmd, pos, vel, tor)

            result_outcome = True
            result_message = f"Smoothly moving hand to target position {target_pos:.2f} revs. Trajectory published."
            return result_outcome, result_message

        except Exception as e:
            result_outcome = False
            result_message = f"Exception occurred: {e}"
            return result_outcome, result_message

    def get_smooth_move_traj(self, target_pos, duration=2.0):
        '''Get the trajectory for smoothly moving the hand to a target position'''
        with self.state_lock:
            # Get the current position
            current_pos = self.last_known_hand_state.pos_estimate

        # Get the trajectory
        time_cmd, pos, vel, acc, tor = self.hand_traj_gen.get_smooth_move_trajectory(start_pos=current_pos, 
                                                                                    target_pos=target_pos, 
                                                                                    duration=duration)
        
        # Convert time_cmd to builtin_interfaces/Time 
        time_cmd_ros = [
            (self.get_clock().now() + Duration(seconds=t)).to_msg() 
            for t in time_cmd
            ]

        return time_cmd_ros, pos, vel, tor


    def detect_ball_drop(self, threshold=2.0):
        '''
        Detect when the ball has been dropped
        This works by checking if the z coordinate of the ball changes by more than a threshold (in mm)

        Args:
        - threshold: The threshold for detecting a drop in the z coordinate of the ball

        Returns:
        - drop_detected: True if the ball has been dropped, False otherwise
        - drop_height: The height the ball has dropped from, if it has been dropped
        - drop_time: The time at which the ball was dropped (ROS2 time), if it has been dropped
        '''
        drop_height = None # {mm}
        drop_time = None

        with self.mocap_lock:
            # Ensure we have at least two coordinates to compare
            if len(self.marker_coordinates) < 2:
                return False, None, None
            
            # Get the first and last z coordinates
            z_first = self.marker_coordinates[0].z
            z_last = self.marker_coordinates[-1].z

        # Check if the difference is greater than the threshold
        if np.abs(z_last - z_first) > threshold:            
            # Calculate how high the ball is dropping from the top of the stroke
            drop_height = z_first - self.BALL_Z_COORDINATE_AT_TOP_OF_STROKE # {mm}

            # Get the time at which the ball was dropped
            drop_time = self.get_clock().now()

            self.get_logger().info(f"Ball has been dropped from a height of {drop_height:.2f} mm. at time {drop_time}")
            return True, drop_height, drop_time
        else:
            return False, None, None


    #########################################################################################################
    #                                            Node Management                                            #
    #########################################################################################################

    def end_session(self, request, response):
        # The method that's called when a user clicks "End Session" in the GUI
        self.get_logger().info("End session service called. Shutting down node.")
        rclpy.shutdown()
        response.success = True
        response.message = "Session ended. Node shutting down."
        return response

def main(args=None):
    rclpy.init(args=args)
    node = HandTestingNode()
    
    # Use MultiThreadedExecutor to allow concurrent execution of callbacks
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    try:
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received. Shutting down...")
    except Exception as e:
        node.get_logger().error(f"Exception in executor: {e}")
    finally:
        executor.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()