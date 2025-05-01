import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from jugglebot_interfaces.msg import (
    CanTrafficReportMessage,
    LegsTargetReachedMessage,
    SetMotorVelCurrLimitsMessage,
    SetTrapTrajLimitsMessage,
    HandTelemetryMessage,
    RobotState,
    HandInputPosMsg
)
from jugglebot_interfaces.srv import (
    ODriveCommandService, 
    GetTiltReadingService, 
    ActivateOrDeactivate,
    SetString
)
from jugglebot_interfaces.action import HomeMotors
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from .can_interface import CANInterface
import numpy as np


class CanInterfaceNode(Node):
    def __init__(self):
        super().__init__('can_interface_node')

        # Initialize the shutdown flag
        self.shutdown_flag = False

        # Initialize the CANInterface
        self.can_handler = CANInterface(logger=self.get_logger())

        #### Initialize service servers ####
        self.encoder_search_service = self.create_service(Trigger, 'encoder_search', self.run_encoder_search)
        self.end_session_service = self.create_service(Trigger, 'end_session', self.end_session)
        self.odrive_command_service = self.create_service(
            ODriveCommandService, 'odrive_command', self.odrive_command_callback)
        self.report_tilt_service = self.create_service(GetTiltReadingService, 'get_platform_tilt', self.report_tilt)
        self.gently_activate_or_deactivate_service = self.create_service(ActivateOrDeactivate,
                                                                        'activate_or_deactivate',
                                                                        self.activate_or_deactivate_callback)
        self.set_hand_state_service = self.create_service(SetString, 'set_hand_state', self.set_hand_state_callback)

        #### Initialize actions ####
        self.home_robot_action = ActionServer(self, HomeMotors, 'home_motors', self.home_robot)

        #### Initialize subscribers ####
        self.leg_pos_sub = self.create_subscription(Float64MultiArray, 'leg_lengths_topic', self.handle_platform_movement, 10)
        self.hand_traj_sub = self.create_subscription(HandInputPosMsg, 'hand_trajectory', self.handle_hand_movement, 10)
        self.motor_vel_curr_limits_sub = self.create_subscription(SetMotorVelCurrLimitsMessage, 'set_motor_vel_curr_limits',
                                                                  self.motor_vel_curr_limits_callback, 10
        )
        self.motor_trap_traj_limits_sub = self.create_subscription(SetTrapTrajLimitsMessage, 'set_leg_trap_traj_limits',
                                                                   self.motor_trap_traj_limits_callback, 10
        )
        self.control_mode_sub = self.create_subscription(String, 'control_mode_topic', self.control_mode_callback, 10)
        self.base_pose_sub = self.create_subscription(PoseStamped, 'base_pose_mocap', self.base_pose_callback, 10)

        #### Initialize publishers ####
            # Hardware data publishers
        self.robot_state_publisher = self.create_publisher(RobotState, 'robot_state', 10)
        self.can_traffic_publisher = self.create_publisher(CanTrafficReportMessage, 'can_traffic', 10)
        self.hand_telemetry_publisher = self.create_publisher(HandTelemetryMessage, 'hand_telemetry', 10)
        self.platform_target_reached_publisher = self.create_publisher(LegsTargetReachedMessage, 'platform_target_reached', 10)

        # Initialize target positions and `target reached` flags - for knowing whether the entire platform has reached its target pose
        self.legs_target_position = [None] * 6
        self.legs_target_reached = [False] * 6

        # Initialize a dictionary for hand trajectory data
        self.hand_traj_data = {
            'cmd_time': [],
            'pos': [],
            'vel_ff': [],
            'torque_ff': []
        }

        # Initialize a variable to track whether the robot was stowed due to an error
        # This prevents the usual shutdown routine from attempting to stow the robot again
        self.stowed_due_to_error = False

        # Initialize timers
        self.platform_target_reached_timer = self.create_timer(0.1, self.check_platform_target_reached_status)
        self.timer_canbus = self.create_timer(timer_period_sec=0.001, callback=self._poll_can_bus)
        self.robot_state_timer = self.create_timer(timer_period_sec=0.005, callback=self.get_and_publish_robot_state)
        self.hand_traj_timer = self.create_timer(timer_period_sec=0.001, callback=self.hand_traj_timer_callback)

        # Initialize the number of axes
        self.num_axes = 7 # 6 leg motors + 1 hand motor

        # Register callbacks with CANInterface
        self.can_handler.register_callback('can_traffic', self.publish_can_traffic)
        self.can_handler.register_callback('hand_telemetry', self.publish_hand_telemetry)

    #########################################################################################################
    #                                     Interfacing with the CAN bus                                      #
    #########################################################################################################

    def _poll_can_bus(self):
        """Polls the CAN bus to check for new updates"""
        self.can_handler.fetch_messages()

        # If there's a stored error, but no current error, clear the stored error
        if self.can_handler.last_known_state['error'] != [] and not (self.can_handler.fatal_error or self.can_handler.fatal_can_error):
            # Log this
            self.get_logger().info(f"Error cleared: {self.can_handler.last_known_state['error']}")
            self.can_handler.update_local_state({'error': []})

    def motor_vel_curr_limits_callback(self, msg):
        """Set the absolute velocity and current limits for all motors."""
        try:
            # Extract the limits from the message
            leg_vel_limit = msg.legs_vel_limit
            leg_curr_limit = msg.legs_curr_limit
            hand_vel_limit = msg.hand_vel_limit
            hand_curr_limit = msg.hand_curr_limit

            self.can_handler.set_absolute_vel_curr_limits(
                leg_vel_limit=leg_vel_limit,
                leg_current_limit=leg_curr_limit,
                hand_vel_limit=hand_vel_limit,
                hand_current_limit=hand_curr_limit,
            )
        except Exception as e:
            self.get_logger().error(f"Error setting motor velocity and current limits: {e}")

    def motor_trap_traj_limits_callback(self, msg):
        """Set the trapezoidal trajectory limits for all leg motors."""
        try:
            # Extract the limits from the message
            leg_vel_limit = msg.trap_vel_limit
            leg_acc_limit = msg.trap_acc_limit
            leg_dec_limit = msg.trap_dec_limit

            self.can_handler.set_all_legs_trap_traj_vel_acc_limits(
                velocity_limit=leg_vel_limit,
                acceleration_limit=leg_acc_limit,
                deceleration_limit=leg_dec_limit,
            )
            
        except Exception as e:
            self.get_logger().error(f"Error setting trapezoidal trajectory limits: {e}")

    #########################################################################################################
    #                                         Generic Callbacks                                             #
    #########################################################################################################

    def base_pose_callback(self, msg):
        """
        Callback for the base pose. Used to determine whether the global coordinate system is still valid.
        If the base pose is measured to be more than <pos_thresh> mm or <ori_thresh> degrees from the origin, the global coordinate
        system is invalid and an error is raised in robot_state.
        """
        # Set the thresholds for position and orientation
        pos_thresh = 5.0  # mm
        ori_thresh = 2.0  # degrees

        error_msg = "Base is not at the origin! Global coordinate system is invalid."

        try:
            # Extract the base pose
            base_pose = msg.pose

            # Check if the base is at the origin
            dist_from_origin = np.linalg.norm([base_pose.position.x, base_pose.position.y, base_pose.position.z])
            angle_from_origin = np.arccos(base_pose.orientation.w) * 2 * 180 / np.pi

            if dist_from_origin > pos_thresh or angle_from_origin > ori_thresh:
                # Determine whether the error is due to position or orientation
                if dist_from_origin > pos_thresh:
                    log_msg = f"Base is {dist_from_origin:.2f} mm from the origin! Global coordinate system is invalid."
                if angle_from_origin > ori_thresh:
                    log_msg = f"Base is {angle_from_origin:.2f} degrees from the origin! Global coordinate system is invalid."

                self.get_logger().warning(log_msg, throttle_duration_sec=10.0)

                # If this error hasn't already been appended to the error list, append it
                if error_msg not in self.can_handler.last_known_state['error']:
                    # self.can_handler.last_known_state['error'].append(error_msg)
                    # self.can_handler.fatal_error = True
                    pass

            else:
                # If the base is at the origin (ie. QTM Transformation has been applied), clear the error
                if error_msg in self.can_handler.last_known_state['error']:
                    # self.can_handler.last_known_state['error'].remove(error_msg)
                    # self.can_handler.fatal_error = False
                    pass

        except Exception as e:
            self.get_logger().error(f"Error in base pose callback: {e}")

    #########################################################################################################
    #                                        Commanding Jugglebot                                           #
    #########################################################################################################

    def control_mode_callback(self, msg):
        """Handle changes in the control mode."""
        try:
            # Construct flags for whether the legs and hand are in CLOSED_LOOP_CONTROL mode
            latest_states = self.latest_axis_states()
            legs_closed_loop = all(state == 8 for state in latest_states[:6])
            hand_closed_loop = latest_states[6] == 8

            if msg.data == 'ERROR':
                self.get_logger().error("Error state detected. Returning to stow position.")
                self.gently_move_platform_to_setpoint(0.0, deactivating=True)
                self.stowed_due_to_error = True

            # Control modes that should have the hand in IDLE mode (to be quieter)
            elif (msg.data == 'SPACEMOUSE' or 
                msg.data == 'LEVEL_PLATFORM_NODE' or 
                msg.data == 'CATCH_THROWN_BALL_NODE' or
                msg.data == 'SHELL' or
                msg.data == 'CALIBRATE_PLATFORM'):
                self.get_logger().info(f'Control mode: {msg.data} enabled')

                # Put the legs into CLOSED_LOOP_CONTROL mode if they aren't already
                if not legs_closed_loop:
                    self.can_handler.set_requested_state_for_all_legs(requested_state='CLOSED_LOOP_CONTROL')

                # Put the hand into IDLE mode since it isn't necessary for spacemouse or levelling (if it isn't already)
                if hand_closed_loop:
                    self.can_handler.set_requested_state(axis_id=6, requested_state='IDLE')
            
            # Control modes that should have all axes in CLOSED_LOOP_CONTROL mode
            elif (msg.data == 'CATCH_DROPPED_BALL_NODE' or
                  msg.data == 'HOOP_SINKER'):
                # Put all axes into CLOSED_LOOP_CONTROL mode if they aren't already
                if not legs_closed_loop or not hand_closed_loop:
                    self.can_handler.set_requested_state_for_all_axes(requested_state='CLOSED_LOOP_CONTROL')

            elif msg.data == '':
                # If no control mode is selected, do nothing
                pass

            else:
                self.get_logger().warning(f"Unknown control mode: {msg.data}. Returning to stow position.")
                self.gently_move_platform_to_setpoint(0.0, deactivating=True)

        except Exception as e:
            self.get_logger().error(f"Error in control_mode_callback: {e}")

    def run_encoder_search(self, request, response):
        """Service callback to run the encoder search."""

        self.get_logger().info("Encoder search initiated.")

        try:
            success = self.can_handler.run_encoder_search()
            response.success = success
            if success:
                response.message = 'Encoder search complete!'
            else:
                response.message = 'Error running encoder search.'

        except Exception as e:
            self.get_logger().error(f"Error running encoder search: {e}")
            response.success = False
            response.message = f"Error running encoder search: {e}"

        return response

    def home_robot(self, goal_handle):
        """Action server callback to home the robot."""
        try:
            # Start the robot homing. Home all axes.
            success = self.can_handler.home_robot(axes_to_home=list(range(self.num_axes))) # True if homing was successful, False otherwise

            self.can_handler.update_state_on_teensy({'is_homed': success})

            if success:
                self.get_logger().info("Robot homing complete.")
                goal_handle.succeed()
            else:
                # Log is sent by the CANInterface. No need to re-log.
                goal_handle.abort()
    
            result = HomeMotors.Result()
            result.success = success
            return result
            
        except Exception as e:
            self.get_logger().error(f"Error homing robot: {e}")
            goal_handle.abort()

    def handle_platform_movement(self, msg):
        """Handle movement commands for the platform (ie all 6 legs)"""
        try:
            # Extract the leg lengths
            motor_positions = msg.data

            # Store these positions as the target positions
            self.legs_target_position = motor_positions

            for axis_id, setpoint in enumerate(motor_positions):
                self.can_handler.send_position_target(axis_id=axis_id, setpoint=setpoint)
                # self.get_logger().debug(f'Motor {axis_id} commanded to setpoint {setpoint}')

        except Exception as e:
            self.get_logger().error(f"Error in handle_platform_movement: {e}")

    def handle_hand_movement(self, msg):
        """
        Handle movement commands for the hand.
        This command should be a trajectory command, which should consist of a list of 
        command times, positions, velocity feedforward values, and torque feedforward values.
        
        These lists will then be saved to the hand_traj_data dictionary (overwriting any existing data).
        The hand_traj_timer_callback will then send the next command in the trajectory at the appropriate time.
        """
        try:
            # Check that the hand is in CLOSED_LOOP_CONTROL mode
            if self.latest_axis_states()[6] != 8:
                self.get_logger().warning("Hand is not in CLOSED_LOOP_CONTROL mode. Ignoring trajectory command.")
                return

            # Save the trajectory data to the dictionary
            self.hand_traj_data['cmd_time'] = msg.time_cmd
            self.hand_traj_data['pos'] = msg.input_pos
            self.hand_traj_data['vel_ff'] = msg.vel_ff
            self.hand_traj_data['torque_ff'] = msg.torque_ff

        except Exception as e:
            self.get_logger().error(f"Error in handle_hand_movement: {e}")

    def hand_traj_timer_callback(self):
        """Send the next hand trajectory command over the CAN bus if the time is appropriate to do so."""
        try:
            # If the trajectory data is empty, do nothing
            if len(self.hand_traj_data['cmd_time']) == 0:
                return
            
            # Get the current time
            current_time = self.get_clock().now().nanoseconds / 1e9

            # Put the trajectory in a more readable format
            traj = self.hand_traj_data
                    
            # Next target time is cmd_time[0], but this needs to be converted to seconds
            cmd_time = traj['cmd_time'][0].sec + traj['cmd_time'][0].nanosec / 1e9

            if current_time >= cmd_time:
                # Command the hand to the setpoint
                input_pos = traj['pos'][0]
                vel_ff = traj['vel_ff'][0]
                torque_ff = traj['torque_ff'][0]

                self.can_handler.send_position_target(axis_id=6, setpoint=input_pos, vel_ff=vel_ff, torque_ff=torque_ff)
                # self.get_logger().info(f'Hand commanded: setpoint={input_pos:.4f}, vel_ff={vel_ff:.4f}, torque_ff={torque_ff:.4f} at time {current_time:.4f}')

                # Remove the first element from the trajectory data
                for key in ('cmd_time', 'pos', 'vel_ff', 'torque_ff'):
                    traj[key].pop(0)

        except Exception as e:
            self.get_logger().error(f"Error in hand_traj_timer_callback: {e}")

    def activate_or_deactivate_callback(self, request, response):
        """
        Bring the robot to an active or inactive state in a controlled manner.
        If the request is to activate, the platform will slowly move to the ACTIVE state (~mid-position, with all motors
        in CLOSED_LOOP_CONTROL. If the chosen control mode doesn't need the hand, it will be put into IDLE in the control_mode_callback.)
        If the request is to deactivate, the platform will slowly move to the INACTIVE state (platform stowed with all motors in IDLE).
        """
        try:
            if request.command == 'activate':
                setpoint = 2.229 # Mid-position (revs)
                deactivating = False
            elif request.command == 'deactivate':
                setpoint = 0.0
                deactivating = True
            else:
                response.success = False
                response.message = f"Invalid request: {request.data}"
                return response
            
            # Gently bring the robot to the setpoint
            result = self.gently_move_platform_to_setpoint(setpoint, deactivating=deactivating)

            if result == True:
                response.success = True
                response.message = f"Platform {request.command}d successfully."            
            else:
                response.success = False
                response.message = f"Error {request.command}ing platform."
            
            return response
        
        except Exception as e:
            self.get_logger().error(f"Error in activate_or_deactivate_callback: {e}")
            response.success = False
            response.message = f"Error in activate_or_deactivate_callback: {e}"
            return response
            
    def gently_move_platform_to_setpoint(self, setpoint, deactivating=True) -> str:
        """
        Gently bring the robot to the nominated setpoint.

        Args:
            setpoint: The setpoint to which the robot should move.
            deactivating: Whether the robot is being deactivated (True) or activated (False)
        """
        try:
            # If deactivating and all legs are already in IDLE mode, with all legs at a position less than 0.1 revs, leave them as-is
            if (deactivating and self.all_legs_in_state(target_state=1)):
                latest_motor_positions = self.latest_motor_positions() 
                if all(pos < 0.1 for pos in latest_motor_positions[:6]):
                    self.get_logger().info("Legs are already deactivated. No need to move.")
                    return True

            # Otherwise, ensure that the legs are in CLOSED_LOOP_CONTROL mode (8) so that we can move them to the setpoint
            if not self.all_legs_in_state(target_state=8):
                self.get_logger().info("Putting all axes into CLOSED_LOOP_CONTROL mode.")
                self.can_handler.set_requested_state_for_all_legs(requested_state='CLOSED_LOOP_CONTROL')

            time.sleep(0.1)

            # Start by lowering the max speed
            self.can_handler.set_absolute_vel_curr_limits(leg_vel_limit=2.5)
            time.sleep(0.1)

            # Command all legs to move to the setpoint
            for axis_id in range(6):
                self.can_handler.send_position_target(axis_id=axis_id, setpoint=setpoint)
                time.sleep(0.001)

            # Wait briefly for the motors to start moving
            time.sleep(0.5)
            self.can_handler.fetch_messages()

            # Wait until all motors have reached their setpoints
            ''' Note that this may have issues if the hand is doing something while the legs are being 'gently moved', but
                this is an edge case that shouldn't happen so it's okay for now.'''
            while not self.all_axes_trajectory_done():
                self.can_handler.fetch_messages()
                time.sleep(0.01)

            # Let things settle for a second
            time.sleep(1.0)

            if deactivating:
                # Put all motors into IDLE mode
                self.can_handler.set_requested_state_for_all_axes(requested_state='IDLE')
            else:
                # Return the motors to their normal operating vel/accel values
                self.can_handler.setup_odrives(requested_state='CLOSED_LOOP_CONTROL')

            return True

        except Exception as e:
            self.get_logger().error(f"Error during 'move to setpoint' process: {e}")
            return False

    def odrive_command_callback(self, request, response):
        """Service callback for ODrive commands."""
        command = request.command

        try:
            if command == 'clear_errors':
                self.can_handler.clear_errors()
                response.success = True
                response.message = 'ODrive errors cleared.'
                # self.is_fatal = False  # Reset the fatal flag now that errors have been cleared
                self.get_logger().info("ODrive errors cleared.")

            elif command == 'reboot_odrives':
                self.can_handler.reboot_odrives()
                response.success = True
                response.message = 'ODrives rebooted.'
                self.get_logger().info("ODrives rebooted.")

                # Update the homing status on the Teensy, as homing will need to be re-run
                self.can_handler.update_state_on_teensy({'is_homed': False,
                                                        'levelling_complete': False,
                                                        'pose_offset_rad': [0.0, 0.0]})

                # Print the current robot state
                self.get_logger().info(f"Robot state after reboot: {self.can_handler.last_known_state}")

            else:
                response.success = False
                response.message = f'Invalid command: {command}'
                self.get_logger().warning(f"Received invalid ODrive command: {command}")

        except Exception as e:
            self.get_logger().error(f"Error in odrive_command_callback: {e}")
            response.success = False
            response.message = f"Error executing command {command}: {e}"

        return response

    def check_platform_target_reached_status(self):
        """Check whether the robot has reached its target and publish the result."""
        try:
            # Check if there are set target positions
            if (
                None in self.legs_target_position
                or None in self.latest_motor_positions()[:6]
                or None in self.latest_motor_velocities()[:6]
            ):
                return

            # Set thresholds
            position_threshold = 0.01  # Revs (approx 0.7 mm with the 22 mm string spool)
            velocity_threshold = 0.1  # Revs/s

            # Check whether each leg has reached its target and is stationary
            for i, (setpoint, meas_pos, meas_vel) in enumerate(
                zip(
                    self.legs_target_position,
                    self.latest_motor_positions()[:6],
                    self.latest_motor_velocities()[:6],
                )
            ):
                if (
                    abs(meas_pos - setpoint) < position_threshold
                    and abs(meas_vel) < velocity_threshold
                ):
                    self.legs_target_reached[i] = True
                else:
                    self.legs_target_reached[i] = False

            # Publish the result
            msg = LegsTargetReachedMessage()
            msg.leg0_has_arrived = self.legs_target_reached[0]
            msg.leg1_has_arrived = self.legs_target_reached[1]
            msg.leg2_has_arrived = self.legs_target_reached[2]
            msg.leg3_has_arrived = self.legs_target_reached[3]
            msg.leg4_has_arrived = self.legs_target_reached[4]
            msg.leg5_has_arrived = self.legs_target_reached[5]

            self.platform_target_reached_publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error checking platform target reached status: {e}")

    #########################################################################################################
    #                                   Interfacing with the ROS Network                                    #
    #########################################################################################################

    def report_tilt(self, request, response):
        """Service callback to report the platform tilt."""
        try:
            tilt_x, tilt_y, tilt_quat = self.can_handler.get_tilt_sensor_reading()
            response.tilt_xy = [tilt_x, tilt_y]
            response.tilt_quat = tilt_quat
            self.get_logger().info(f"Reported tilt readings: X={tilt_x}, Y={tilt_y}")
        except Exception as e:
            self.get_logger().error(f"Error reporting tilt: {e}")
            response.tilt_xy = [None, None]

        return response

    def publish_can_traffic(self, can_traffic_data):
        """Publish CAN traffic reports."""
        try:
            msg = CanTrafficReportMessage()
            msg.received_count = can_traffic_data['received_count']
            msg.report_interval = can_traffic_data['report_interval']

            self.can_traffic_publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing CAN traffic: {e}")

    def publish_hand_telemetry(self, hand_telemetry_data):
        """Publish hand telemetry data."""
        try:
            msg = HandTelemetryMessage()
            msg.timestamp = self.get_clock().now().to_msg()
            msg.position = hand_telemetry_data['position']
            msg.velocity = hand_telemetry_data['velocity']

            self.hand_telemetry_publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing hand telemetry: {e}")

    def update_teensy_and_local_state_from_topic(self, msg):
        """Update the Teensy with the robot state.
        Also update the local copy of the robot state."""

        try:
            # Update the state on the Teensy. Note no error field is passed here
            self.can_handler.update_state_on_teensy({
                'is_homed': msg.is_homed,
                'levelling_complete': msg.levelling_complete,
                'pose_offset_rad': msg.pose_offset_rad,
            })

            # Update the local copy of the robot state
            self.can_handler.update_local_state({
                'is_homed': msg.is_homed,
                'levelling_complete': msg.levelling_complete,
                'pose_offset_rad': msg.pose_offset_rad,
                'error': msg.error,
            })

        except Exception as e:
            self.get_logger().error(f"Error updating state on Teensy: {e}")

    #########################################################################################################
    #                                       Related to Motor States                                         #
    #########################################################################################################

    def get_and_publish_robot_state(self):
        """Publish the motor states."""
        try:
            msg = RobotState()
            msg.timestamp = self.get_clock().now().to_msg()
            msg.motor_states = self.can_handler.get_motor_states()

            # Get the last known state from can_handler
            state = self.can_handler.last_known_state

            # Update error field if appropriate
            if self.can_handler.undervoltage_error:
                msg.error.append("Undervoltage detected. Was the E-stop hit?")
            elif self.can_handler.fatal_error:
                msg.error.append(
                f"Fatal issue with one or more ODrive(s): {state['error']}"
                )
            elif self.can_handler.fatal_can_error:
                msg.error.append("Fatal issue with CAN bus.")
            else:
                msg.error = []

            # Update the general robot state
            msg.encoder_search_complete=state['encoder_search_complete']
            msg.is_homed=state['is_homed']
            msg.levelling_complete=state['levelling_complete']
            msg.pose_offset_rad=state['pose_offset_rad']
            msg.pose_offset_quat=state['pose_offset_quat']

            self.robot_state_publisher.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Error publishing motor states: {e}")

    # Get the latest motor states (eg. IDLE, CLOSED_LOOP_CONTROL, etc.)
    def latest_axis_states(self):
        """Get the latest axis states."""
        return [motor.current_state for motor in self.can_handler.last_motor_states]
    
    # Check whether all motors are in a chosen specific state
    def all_axes_in_state(self, target_state: int):
        """Check if all axes are in the specific state."""
        return all(motor.current_state == target_state for motor in self.can_handler.last_motor_states)
    
    def all_axes_trajectory_done(self):
        """Check if all axes have completed their trajectories."""
        return all(motor.trajectory_done for motor in self.can_handler.last_motor_states)
    
    def all_legs_in_state(self, target_state: int):
        """Check if all legs are in the specific state."""
        return all(motor.current_state == target_state for motor in self.can_handler.last_motor_states[:6])

    def latest_motor_positions(self):
        """Get the latest motor positions."""
        return [motor.pos_estimate for motor in self.can_handler.last_motor_states]
    
    def latest_motor_velocities(self):
        """Get the latest motor velocities."""
        return [motor.vel_estimate for motor in self.can_handler.last_motor_states]
    
    def latest_motor_iqs(self):
        """Get the latest motor IQs."""
        return [motor.iq_measured for motor in self.can_handler.last_motor_states]

    def set_hand_state_callback(self, request, response):
        """Service callback to set the hand state."""
        try:
            # Extract the hand state from the request
            hand_state = request.data

            # Put the hand into the requested state
            self.can_handler.set_requested_state(axis_id=6, requested_state=hand_state)

            response.success = True
            response.message = f"Hand state set to {hand_state}"
            return response

        except Exception as e:
            self.get_logger().error(f"Error setting hand state: {e}")
            response.success = False
            response.message = f"Error setting hand state: {e}"
            return response

    #########################################################################################################
    #                                          Utility Functions                                            #
    #########################################################################################################

    def on_shutdown(self):
        """Handle node shutdown."""
        self.get_logger().info("Shutting down CanInterfaceNode...")
        try:
            if not self.stowed_due_to_error:
                self.gently_move_platform_to_setpoint(0.0, deactivating=True) # Deactivate the robot
    
            self.can_handler.set_requested_state(axis_id=6, requested_state='IDLE') # Put the hand into IDLE mode
            self.can_handler.shutdown()
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
    node = CanInterfaceNode()

    try:
        while rclpy.ok() and not node.shutdown_flag:
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
