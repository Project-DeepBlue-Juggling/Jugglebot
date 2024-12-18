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
    RobotState
)
from jugglebot_interfaces.srv import ODriveCommandService, GetTiltReadingService, ActivateOrDeactivate
from jugglebot_interfaces.action import HomeMotors
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger
from .can_interface import CANInterface


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
            ODriveCommandService, 'odrive_command', self.odrive_command_callback
        )
        self.report_tilt_service = self.create_service(GetTiltReadingService, 'get_platform_tilt', self.report_tilt)
        self.gently_activate_or_deactivate_service = self.create_service(ActivateOrDeactivate,
                                                                        'activate_or_deactivate',
                                                                        self.activate_or_deactivate_callback)

        #### Initialize actions ####
        self.home_robot_action = ActionServer(self, HomeMotors, 'home_motors', self.home_robot)

        #### Initialize subscribers ####
        self.motor_pos_subscription = self.create_subscription(
            Float64MultiArray, 'leg_lengths_topic', self.handle_movement, 10
        )
        self.motor_vel_curr_limits_subscription = self.create_subscription(
            SetMotorVelCurrLimitsMessage, 'set_motor_vel_curr_limits', self.motor_vel_curr_limits_callback, 10
        )
        self.motor_trap_traj_limits_subscription = self.create_subscription(
            SetTrapTrajLimitsMessage, 'set_leg_trap_traj_limits', self.motor_trap_traj_limits_callback, 10
        )
        self.control_mode_subscription = self.create_subscription(
            String, 'control_mode_topic', self.control_mode_callback, 10
        )

        #### Initialize publishers ####
            # Hardware data publishers
        self.robot_state_publisher = self.create_publisher(RobotState, 'robot_state', 10)
        self.can_traffic_publisher = self.create_publisher(CanTrafficReportMessage, 'can_traffic', 10)
        # self.hand_telemetry_publisher = self.create_publisher(HandTelemetryMessage, 'hand_telemetry', 10) # Keeping this commented out in case we need it later
        self.platform_target_reached_publisher = self.create_publisher(LegsTargetReachedMessage, 'platform_target_reached', 10)

        # Initialize target positions and `target reached` flags - for knowing whether the entire platform has reached its target pose
        self.legs_target_position = [None] * 6
        self.legs_target_reached = [False] * 6

        # Initialize timers
        self.platform_target_reached_timer = self.create_timer(0.1, self.check_platform_target_reached_status)
        self.timer_canbus = self.create_timer(timer_period_sec=0.001, callback=self._poll_can_bus)
        self.robot_state_timer = self.create_timer(timer_period_sec=0.01, callback=self.get_and_publish_robot_state)

        # Initialize the number of axes
        self.num_axes = 7 # 6 leg motors + 1 hand motor

        # # Register callbacks with CANInterface
        self.can_handler.register_callback('can_traffic', self.publish_can_traffic)
        # self.can_handler.register_callback('hand_telemetry', self.publish_hand_telemetry)

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

            self.can_handler.set_trap_traj_vel_acc_limits(
                velocity_limit=leg_vel_limit,
                acceleration_limit=leg_acc_limit,
                deceleration_limit=leg_dec_limit,
            )
            
        except Exception as e:
            self.get_logger().error(f"Error setting trapezoidal trajectory limits: {e}")

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

            if msg.data == 'spacemouse' or msg.data == 'level_platform_node':
                self.get_logger().info(f'Control mode: {msg.data} enabled')

                # Put the legs into CLOSED_LOOP_CONTROL mode if they aren't already
                if not legs_closed_loop:
                    self.can_handler.set_requested_state_for_all_legs(requested_state='CLOSED_LOOP_CONTROL')

                # Put the hand into IDLE mode since it isn't necessary for spacemouse or levelling (if it isn't already)
                if hand_closed_loop:
                    self.can_handler.set_requested_state(axis_id=6, requested_state='IDLE')
            
            elif msg.data == 'shell':
                # Put the legs into CLOSED_LOOP_CONTROL mode if they aren't already
                if not legs_closed_loop:
                    self.can_handler.set_requested_state_for_all_legs(requested_state='CLOSED_LOOP_CONTROL')

                # Put the hand into IDLE mode since it isn't controlled by the shell (if it isn't already)
                if hand_closed_loop:
                    self.can_handler.set_requested_state(axis_id=6, requested_state='IDLE')

            elif msg.data == '':
                # If no control mode is selected, do nothing
                pass

            else:
                self.get_logger().warning(f"Unknown control mode: {msg.data}. Putting all axes into IDLE.")
                self.can_handler.set_requested_state_for_all_axes(requested_state='IDLE')

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
            # Start the robot homing
            success = self.can_handler.home_robot() # True if homing was successful, False otherwise

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

    def handle_movement(self, msg):
        """Handle movement commands for the robot."""
        try:
            # Extract the leg lengths
            motor_positions = msg.data

            # Store these positions as the target positions
            self.legs_target_position = motor_positions

            for axis_id, setpoint in enumerate(motor_positions):
                self.can_handler.send_position_target(axis_id=axis_id, setpoint=setpoint)
                # self.get_logger().debug(f'Motor {axis_id} commanded to setpoint {setpoint}')

        except Exception as e:
            self.get_logger().error(f"Error in handle_movement: {e}")

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
            # If deactivating and all axes are already in IDLE mode, with all legs at a position less than 0.1 revs, leave them as-is
            if (deactivating and self.all_axes_in_state(target_state=1)):
                latest_motor_positions = self.latest_motor_positions() 
                if all(pos < 0.1 for pos in latest_motor_positions[:6]):
                    self.get_logger().info("Robot is already deactivated. No need to move.")
                    return True

            # Otherwise, ensure that the legs are in CLOSED_LOOP_CONTROL mode (8)
            if not self.all_axes_in_state(target_state=8):
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

    # def publish_hand_telemetry(self, hand_telemetry_data):
    #     """Publish hand telemetry data."""
    #     try:
    #         msg = HandTelemetryMessage()
    #         msg.timestamp = self.get_clock().now().to_msg()
    #         msg.position = hand_telemetry_data['position']
    #         msg.velocity = hand_telemetry_data['velocity']

    #         self.hand_telemetry_publisher.publish(msg)
    #     except Exception as e:
    #         self.get_logger().error(f"Error publishing hand telemetry: {e}")

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
    
    # Check is all motors are in a chosen specific state
    def all_axes_in_state(self, target_state: int):
        """Check if all axes are in the specific state."""
        return all(motor.current_state == target_state for motor in self.can_handler.last_motor_states)
    
    def all_axes_trajectory_done(self):
        """Check if all axes have completed their trajectories."""
        return all(motor.trajectory_done for motor in self.can_handler.last_motor_states)
    
    def latest_motor_positions(self):
        """Get the latest motor positions."""
        return [motor.pos_estimate for motor in self.can_handler.last_motor_states]
    
    def latest_motor_velocities(self):
        """Get the latest motor velocities."""
        return [motor.vel_estimate for motor in self.can_handler.last_motor_states]
    
    def latest_motor_iqs(self):
        """Get the latest motor IQs."""
        return [motor.iq_measured for motor in self.can_handler.last_motor_states]

    #########################################################################################################
    #                                          Utility Functions                                            #
    #########################################################################################################

    def on_shutdown(self):
        """Handle node shutdown."""
        self.get_logger().info("Shutting down CanInterfaceNode...")
        try:
            self.gently_move_platform_to_setpoint(0.0, deactivating=True) # Deactivate the robot
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
