import time
import rclpy
from rclpy.node import Node
from jugglebot_interfaces.srv import ODriveCommandService, GetTiltReadingService
from jugglebot_interfaces.msg import (RobotStateMessage, 
                                      CanTrafficReportMessage, 
                                      LegsTargetReachedMessage, 
                                      SetMotorVelCurrLimitsMessage,
                                      HandTelemetryMessage
                                      )
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
from .can_handler import CANHandler

class CANBusHandlerNode(Node):
    def __init__(self):
        super().__init__('can_bus_handler_node')
        
        self.can_handler = CANHandler(logger=self.get_logger())

        # Set up a subscriber and publisher for the robot state
        self.robot_state_publisher = self.create_publisher(RobotStateMessage, 'robot_state_topic', 10)
        self.robot_state_subscription = self.create_subscription(RobotStateMessage, 'robot_state_topic', self.robot_state_callback, 10)

        # Set up a service to trigger the homing of the robot
        self.home_service = self.create_service(Trigger, 'home_robot', self.home_robot)

        # Initialize a parameter to track whether the robot has been homed or not
        self.is_homed = False

        # Initialize a parameter to track whether the robot is in a fatal state
        self.is_fatal = False

        # Set up a service to trigger closing the node
        self.end_session_service = self.create_service(Trigger, 'end_session', self.end_session)

        # Set up a service to respond to commands to the ODrives
        self.odrive_command_service = self.create_service(ODriveCommandService, 'odrive_command', self.odrive_command_callback)

        # Subscribe to relevant topics
        self.motor_pos_subscription = self.create_subscription(Float64MultiArray, 'leg_lengths_topic', self.handle_movement, 10)

        # Initialise service server to report the measured tilt readings
        self.report_tilt_service = self.create_service(GetTiltReadingService, 'get_platform_tilt', self.report_tilt)

        # Initialize a service to prepare the hand for a throw
        self.prepare_hand_for_throw_service = self.create_service(Trigger, 'prepare_hand_for_throw', self.prepare_hand_for_throw)

        # Initialize a variable to track the target positions of the legs
        self.legs_target_position = [None] * 6

        # Subscribe to absolute motor vel/curr limits topic
        self.motor_vel_curr_limits_subscription = self.create_subscription(SetMotorVelCurrLimitsMessage,
                                                                           'set_motor_vel_curr_limits',
                                                                           self.motor_vel_curr_limits_callback,
                                                                           10)

        # Set up timer and publisher for whether the legs have reached their targets
        self.platform_target_reached_publisher = self.create_publisher(LegsTargetReachedMessage, 'target_reached', 10)
        self.legs_target_reached = [False] * 6
        self.platform_target_reached_timer = self.create_timer(0.1, self.check_platform_target_reached_status)

        # Initialize publishers for hardware data
        self.position_publisher       = self.create_publisher(Float64MultiArray, 'motor_positions', 10)
        self.velocity_publisher       = self.create_publisher(Float64MultiArray, 'motor_velocities', 10)
        self.iq_publisher             = self.create_publisher(Float64MultiArray, 'motor_iqs', 10)
        self.can_traffic_publisher    = self.create_publisher(CanTrafficReportMessage, 'can_traffic', 10)
        self.hand_telemetry_publisher = self.create_publisher(HandTelemetryMessage, 'hand_telemetry', 10)

        # Initialize variables to store the last received data
        self.last_motor_positions = None
        self.last_motor_velocities = None
        self.last_motor_iqs = None

        # Register callbacks
        self.can_handler.register_callback('motor_positions', self.publish_motor_positions)
        self.can_handler.register_callback('motor_velocities', self.publish_motor_velocities)
        self.can_handler.register_callback('motor_iqs', self.publish_motor_iqs)
        self.can_handler.register_callback('can_traffic', self.publish_can_traffic)
        self.can_handler.register_callback('hand_telemetry', self.publish_hand_telemetry)

        # Initialize a timer to poll the CAN bus
        self.timer_canbus = self.create_timer(timer_period_sec=0.001, callback=self._poll_can_bus)


    #########################################################################################################
    #                                     Interfacing with the CAN bus                                      #
    #########################################################################################################

    def _poll_can_bus(self):
        # Polls the CAN bus to check for new updates
        self.can_handler.fetch_messages()
        if self.can_handler.fatal_issue:
            self._on_fatal_issue()

    def _on_fatal_issue(self):
        self.get_logger().fatal("Fatal issue detected. Robot will be unresponsive until errors are cleared", throttle_duration_sec=3.0)
        self.is_fatal = True

    def motor_vel_curr_limits_callback(self, msg):
        '''Set the absolute velocity and current limits for all motors'''
        # Extract the limits from the message
        leg_vel_limit = msg.legs_vel_limit
        leg_curr_limit = msg.legs_curr_limit
        hand_vel_limit = msg.hand_vel_limit
        hand_curr_limit = msg.hand_curr_limit

        # If any of these values are 0, don't update that value. This allows for updating only the values that are needed
        if leg_vel_limit != 0:
            self.can_handler.set_absolute_vel_curr_limits(leg_vel_limit=leg_vel_limit)
        if leg_curr_limit != 0:
            self.can_handler.set_absolute_vel_curr_limits(leg_curr_limit=leg_curr_limit)
        if hand_vel_limit != 0:
            self.can_handler.set_absolute_vel_curr_limits(hand_vel_limit=hand_vel_limit)
        if hand_curr_limit != 0:
            self.can_handler.set_absolute_vel_curr_limits(hand_curr_limit=hand_curr_limit)

    #########################################################################################################
    #                                        Commanding Jugglebot                                           #
    #########################################################################################################

    def home_robot(self, request, response):
        try:
            self.can_handler.home_robot()
            response.success = True

            # Publish that the robot is now homed to the robot_state_topic
            homed_msg = RobotStateMessage()
            homed_msg.is_homed = True
            self.robot_state_publisher.publish(homed_msg)

        except Exception as e:
            self.get_logger().error(f'Error in home_robot: {str(e)}')
            response.success = False

        return response

    def handle_movement(self, msg):
        # Check if the robot is homed
        if not self.is_homed:
            self.get_logger().warning('Robot not homed. Home robot before commanding it to move.', once=True)
            return
        
        if self.can_handler.fatal_can_issue:
            self._on_fatal_issue()
            return

        # Extract the leg lengths
        motor_positions = msg.data

        # Store these positions as the target positions
        self.legs_target_position = motor_positions

        for axis_id, data in enumerate(motor_positions):
            self.can_handler.send_position_target(axis_id=axis_id, setpoint=data)
            # self.get_logger().debug(f'Motor {axis_id} commanded to setpoint {data}')

    def shutdown_robot(self):
        # Send the robot home and put all motors in IDLE
        # Start by lowering the max speed
        self.can_handler.set_absolute_vel_curr_limits(leg_vel_limit=2.5)
        time.sleep(1.0)

        # Command all motors to move home
        for axis_id in range(6):
            self.can_handler.send_position_target(axis_id=axis_id, setpoint=0.0)
            time.sleep(0.001)

        # Add a short break to make sure all motors have started moving and movement flags have updated
        time.sleep(0.5)
        self.can_handler.fetch_messages()

        # Check to make sure all motors have finished their movements
        while not all(self.can_handler.trajectory_done):
            self.can_handler.fetch_messages()
            time.sleep(0.01)

        time.sleep(1.0)  # Add a short break to make sure all motors have stopped moving

        # Put motors into IDLE
        self.can_handler.set_leg_odrive_state(requested_state='IDLE')
        self.can_handler.set_requested_state(axis_id=6, requested_state='IDLE') # Also put the hand motor into IDLE

    def odrive_command_callback(self, request, response):
        command = request.command

        if command == 'clear_errors':
            self.can_handler.clear_errors()
            response.success = True
            response.message = 'ODrive errors cleared'
            self.is_fatal = False  # Reset the fatal flag now that errors have been cleared

        elif command == 'reboot_odrives':
            self.can_handler.reboot_odrives()
            response.success = True
            response.message = 'ODrives rebooted'
            self.is_homed = False  # Reset the homed flag

        else:
            response.success = False
            response.message = 'Invalid command:' + str(request)

        return response

    def check_platform_target_reached_status(self):
        '''Check whether the robot has reached its target and publish the result.
        The target is considered reached if all motors are within a certain tolerance of their setpoints'''

        # Check if there are any target positions set, or if the last motor positions or velocities are missing
        if None in self.legs_target_position or None in self.last_motor_positions or None in self.last_motor_velocities:
            return

        # Set thresholds
        position_threshold = 0.01 # Revs (approx 0.7 mm with the 22 mm string spool)
        velocity_threshold = 0.1 # Revs/s

        # Check whether each leg has reached its target and is stationary

        for i, (setpoint, meas_pos, meas_vel) in enumerate(zip(self.legs_target_position, 
                                                               self.last_motor_positions, 
                                                               self.last_motor_velocities)):
            
            if abs(meas_pos - setpoint) < position_threshold and abs(meas_vel) < velocity_threshold:
                self.legs_target_reached[i] = True

            else :
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

    def prepare_hand_for_throw(self, request, response):
        '''Prepare the hand for a throw by putting the hand motor in closed loop control,
        setting the input mode to PASSTHROUGH, and setting the control mode to POSITION_CONTROL'''

        # Put the hand motor in closed loop control
        self.can_handler.set_requested_state(axis_id=6, requested_state='CLOSED_LOOP_CONTROL')

        # Set the control mode to POSITION_CONTROL and the input mode to PASSTHROUGH
        self.can_handler.set_control_mode(axis_id=6, control_mode='POSITION_CONTROL', input_mode='PASSTHROUGH')

        response.success = True
        response.message = 'Hand prepared for throw'

        return response

    #########################################################################################################
    #                                   Interfacing with the ROS Network                                    #
    #########################################################################################################
    
    def report_tilt(self, request, response):
        # Call the method in the CANHandler to get the tilt readings
        response.tilt_readings = self.can_handler.get_tilt_sensor_reading()

        return response

    def robot_state_callback(self, msg):
        if msg.is_homed and not self.is_homed: # If the robot has just been homed, update the flag
            self.is_homed = True

    def publish_motor_positions(self, position_data):
        motor_positions = Float64MultiArray()
        motor_positions.data = position_data

        # Store the last received positions
        self.last_motor_positions = position_data[:6] # Only store the leg positions for now

        self.position_publisher.publish(motor_positions)

    def publish_motor_velocities(self, velocity_data):
        motor_velocities = Float64MultiArray()
        motor_velocities.data = velocity_data

        # Store the last received velocities 
        self.last_motor_velocities = velocity_data[:6] # Only store the leg velocities for now

        self.velocity_publisher.publish(motor_velocities)

    def publish_motor_iqs(self, iq_data):
        motor_iqs = Float64MultiArray()
        motor_iqs.data = iq_data

        # Store the last received IQs
        self.last_motor_iqs = iq_data[:6] # Only store the leg IQs for now

        self.iq_publisher.publish(motor_iqs)

    def publish_can_traffic(self, can_traffic_data):
        msg = CanTrafficReportMessage()
        msg.received_count = can_traffic_data['received_count']
        msg.report_interval = can_traffic_data['report_interval']

        self.can_traffic_publisher.publish(msg)

    def publish_hand_telemetry(self, hand_telemetry_data):
        msg = HandTelemetryMessage()
        msg.timestamp = self.get_clock().now().to_msg()
        msg.position = hand_telemetry_data['position']
        msg.velocity = hand_telemetry_data['velocity']

        self.hand_telemetry_publisher.publish(msg)

    #########################################################################################################
    #                                          Managing the Node                                            #
    #########################################################################################################

    def on_shutdown(self):
        # Cleanup code goes here
        self.shutdown_robot()
        self.can_handler.close()

    def end_session(self, request, response):
        # The method that's called when a user clicks "End Session" in the GUI
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = CANBusHandlerNode()

    try:
        rclpy.spin(node)
    except RuntimeError:
        pass
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass
    finally:
        node.get_logger().info("Shutting down...")
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()