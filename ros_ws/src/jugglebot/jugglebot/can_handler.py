"""
CANHandler Class
----------------

This class is designed to interface with a CAN bus using the python-can library. 
It's specifically tailored for handling communications for a robot powered by six BLDC motors, controlled by ODrive controllers. 
The class provides functionalities for sending commands to the motors, handling periodic status messages from the ODrive controllers, 
and managing error states and encoder estimates.

Key Features:
-------------
1. Sending Commands:
   - The class can be used to send specific commands over the CAN bus to different motors. 

2. Handling Periodic Messages:
   - The ODrives send periodic status messages that are handled by this class. These messages include heartbeats and encoder estimates.

3. Error Handling:
   - The class listens for error messages from the ODrives, interpreting them and taking appropriate actions as defined 
        (e.g., stopping the robot, logging error information, etc.).

4. Encoder Estimates:
   - It collects and stores encoder estimates for the motors. These estimates are stored in a memory-efficient manner, 
        using a circular buffer to hold the last hour's worth of data (at a rate of 100 Hz per motor).

Usage:
------
- Instantiate the class with the desired CAN bus name and baud rate.
- Use the instance to send commands to the motors.
- The instance automatically handles incoming messages from the ODrives, including error messages and encoder estimates.
- Retrieve the encoder estimates when needed for analysis or debugging.
- Monitor for any errors and ensure appropriate actions are taken if any error occurs.
"""

import os
from ament_index_python.packages import get_package_share_directory
import can
import cantools
import struct
import time

class CANHandler:
    # Create a dictionary of commands that might be called over the CAN bus by external scripts
    COMMANDS = {
        "heartbeat_message"   : 0x001,
        "get_motor_error"     : 0x003,
        "get_encoder_error"   : 0x004,
        "get_encoder_estimate": 0x009,
        "set_controller_mode" : 0x00B,
        "set_input_pos"       : 0x00C,
        "set_input_vel"       : 0x00D,
        "set_requested_state" : 0x007,
        "set_vel_curr_limits" : 0x00F,
        "set_traj_vel_limit"  : 0x011,
        "set_traj_acc_limits" : 0x012,
        "get_iq"              : 0x014, 
        "reboot_odrives"      : 0x016,
        "clear_errors"        : 0x018,
        "set_linear_count"    : 0x019,
        "get_controller_error": 0x01D,
    }

    # Create the reverse dictionary to go from ID to name; mostly for logging human-readable errors
    COMMAND_ID_TO_NAME = {v: k for k, v in COMMANDS.items()}

    # Set absolute limit for how far the motors can turn. This is intended as a backup if prior error-checking fails
    _MOTOR_MAX_POSITION = 4.2  # Revs

    def __init__(self, logger, bus_name='can0', bitrate=1000000, bus_type='socketcan'):
        # Find the package directory
        pkg_dir = get_package_share_directory('jugglebot')
        
        # Construct the full filepath to the dbc file
        dbc_file_path = os.path.join(pkg_dir, 'resources', 'ODrive_CAN.dbc')

        # Now import the ODrive dbc file
        self.db = cantools.database.load_file(dbc_file_path)

        # Initialize the logger, to get access to the ROS network (for logging purposes only)
        self.ROS_logger = logger

        # Initialize the parameters for the CAN bus that will be used by setup_can_bus to initialise the bus itself
        self._can_bus_name = bus_name
        self._can_bitrate = bitrate
        self._can_bus_type = bus_type
        self.bus = None

        # Create a dictionary to map command IDs to their corresponding handler
        # This allows for easy addition of new commands and corresponding handlers
        self.command_handlers = {
            self.COMMANDS["heartbeat_message"]   : self._handle_heartbeat,
            self.COMMANDS["get_motor_error"]     : self._handle_motor_error,
            self.COMMANDS["get_encoder_error"]   : self._handle_encoder_error,
            self.COMMANDS["get_controller_error"]: self._handle_controller_error,
            self.COMMANDS["get_encoder_estimate"]: self._handle_encoder_estimates,
            self.COMMANDS["get_iq"]              : self._handle_iq_readings,
        }

        # Create a dictionary of callbacks that are used in the ROS2 node
        self.callbacks = {
            'motor_positions' : None,
            'motor_iqs'       : None,
            'motor_velocities': None,
            'can_traffic'     : None,
        }

        # Initialize the number of axes that the robot has
        self.num_axes = 8  # 6 for the legs, 1 for the hand. 1 Extra because I don't know how to disable the extra axis on the ODrive

        # Initialize buffers for all variables that are being sent out to the ROS network
        self.position_buffer = [None] * self.num_axes
        self.velocity_buffer = [None] * self.num_axes
        self.iq_buffer       = [None] * self.num_axes

        # We need to know current motor iqs for some methods (eg. homing) so initialize a list for these values
        self.iq_values = [None] * self.num_axes

        # Initialize a flag that triggers whenever any fatal errors are detected
        self.fatal_issue = False

        # Initialize a variable to track when the last heartbeat message came in. Determines whether the robot is responding
        self.last_heartbeat_time = None
        self.fatal_can_issue = False  # Has the bus completely failed and can't be restored?

        # Initialize the current state for each axis
        self.axis_states = [None] * self.num_axes
        self.is_done_moving = [None] * self.num_axes

        self.setup_can_bus()
        self.wait_for_heartbeat()

    #########################################################################################################
    #                                            CAN Bus Management                                         #
    #########################################################################################################

    def setup_can_bus(self):
        self.bus = can.Bus(channel=self._can_bus_name, bustype=self._can_bus_type, bitrate=self._can_bitrate)
        self.flush_bus() # Flush the bus now that it's been set up

    def wait_for_heartbeat(self):
        # Wait for the heartbeat message to verify that the bus is operational
        self.ROS_logger.info('Waiting for heartbeat from Jugglebot...')
        time_to_wait = 3  # Seconds

        start_time = time.time()

        while time.time() - start_time < time_to_wait:
            self.fetch_messages()

            if self.last_heartbeat_time and time.time() - self.last_heartbeat_time < 1.0:
                # If a heartbeat message has been received in the last second
                self.ROS_logger.info('Heartbeat received!')
                return

            time.sleep(0.1)

        # If no heartbeat was detected, attempt to restore the connection
        self.ROS_logger.info('No heartbeat detected.')
        self.attempt_to_restore_can_connection()

    def close(self):
        if self.bus is not None:
            try:
                self.bus.shutdown()
                self.ROS_logger.info('CAN bus closed')
            except Exception as e:
                self.ROS_logger.error(f'Error when closing CAN bus: {e}')

    def flush_bus(self):
        # Flushes the CAN bus to remove any stale messages
        # Read messages with a non-blocking call until there are no more
        while not (self.bus.recv(timeout=0) is None):
            pass # We're just discarding messages, so we don't need to do anything with them

    def attempt_to_restore_can_connection(self):
        # If the bus isn't responding, try to re-establish the connection
        max_retries = 3  # Maximum number of reconnection attempts
        retry_delay = 5  # Delay in seconds between retries
        reconnect_timeout = 1  # How long to wait for the heartbeat to come in before considering the connection dead

        self.ROS_logger.info('Attempting to restore CAN connection...')

        for attempt in range(max_retries):
            rx_working = 0  # Flag to indicate if receiving messages is working
            tx_working = 0
            try:
                # Attempt to close the existing connection if it's still open
                if self.bus is not None:
                    self.bus.shutdown()

                # Reinitialize the CAN bus connection
                self.setup_can_bus()

                time.sleep(0.1)
                self.fetch_messages()  # See if there are any new messages

                # Wait for heartbeat message to check that receiving messages is working
                start_time = time.time()
                while time.time() - start_time < reconnect_timeout:
                    self.fetch_messages() # Get any recent messages

                    if not self.last_heartbeat_time:
                        # If no heartbeat message has ever been received
                        self.ROS_logger.warning('No heartbeat message has been received yet. Is Jugglebot on?', 
                                                throttle_duration_sec=2.0)

                    elif time.time() - self.last_heartbeat_time < reconnect_timeout:
                        self.ROS_logger.info(f"Reading from CAN bus successful on attempt {attempt + 1}")
                        rx_working = 1
                        break
                    
                # Now check that sending messages is working by sending a dummy value
                msg = can.Message(arbitration_id=0x999, dlc=8, is_extended_id=False, data=[0, 0, 0, 0, 0, 0, 0, 0])
                try:
                    self.bus.send(msg)
                    self.ROS_logger.info(f"Writing to CAN bus successful on attempt {attempt + 1}")
                    tx_working = 1
                except Exception as e:
                    self.ROS_logger.warn(f"Failed to write to CAN bus on attempt {attempt + 1}: {e}")

                # If both reading and writing are working, we're done
                if rx_working and tx_working:
                    self.ROS_logger.info('CAN bus connection restored!')
                    self.fatal_can_issue = False
                    return
                
            except Exception as e:
                self.ROS_logger.warn(f"Failed to reconnect CAN bus on attempt {attempt + 1}: {e}")
            
            time.sleep(retry_delay)  # Wait before retrying

        self.ROS_logger.error(f"Failed to restore CAN bus connection after {max_retries} attempts. Closing CAN bus...")
        self.close()
        self.fatal_can_issue = True  # Report the fatal CAN issue so that the handler node can stop attempting to send messages

    #########################################################################################################
    #                                         ROS2 Callback Management                                      #
    #########################################################################################################

    def register_callback(self, data_type, callback):
        if data_type in self.callbacks:
            self.callbacks[data_type] = callback

    def _trigger_callback(self, data_type, data):
        if self.callbacks[data_type]:
            self.callbacks[data_type](data)

    #########################################################################################################
    #                                             Sending messages                                          #
    #########################################################################################################

    def _send_message(self, axis_id, command_name, data=None, error_descriptor='Not described', rtr_bit=False):
        if not self.bus:
            # If the bus hasn't been initialized, return
            return
        
        # Get the hex code for the message being sent
        command_id = self.COMMANDS[command_name]

        # Create the CAN message
        arbitration_id = (axis_id << 5) | command_id

        if data is not None:
            msg = can.Message(arbitration_id=arbitration_id, dlc=8, is_extended_id=False, data=data, is_remote_frame=rtr_bit)
        else:
            # For messages that require no data to be sent (eg. get encoder estimates)
            msg = can.Message(arbitration_id=arbitration_id, dlc=8, is_extended_id=False, is_remote_frame=rtr_bit)

        try:
            self.bus.send(msg)
            self.ROS_logger.debug(f"CAN message for {error_descriptor} sent to axisID {axis_id}")
        except Exception as e:
            # Log that the message couldn't be sent
            self.ROS_logger.warn(f"CAN message for {error_descriptor} NOT sent to axisID {axis_id}! Error: {e}")

            # If the buffer is full, the CAN bus is probably having issue. Try to re-establish it
            if "105" in str(e):
                # "No buffer space available" is error code 105
                self.attempt_to_restore_can_connection()

    def send_position_target(self, axis_id, setpoint, max_position=_MOTOR_MAX_POSITION, min_position=0.0):
        # Commands the given motor to move to the designated setpoint (after checking its magnitude)

        if setpoint > max_position or setpoint < min_position:
            setpoint = max(min_position, min(setpoint, max_position))

            self.ROS_logger.warning(f'Setpoint of {setpoint:.2f} is outside allowable bounds and has been clipped')

        setpoint = -1 * setpoint  # Since -ve is extension

        data = self.db.encode_message(f'Axis{axis_id}_Set_Input_Pos', 
                                {'Input_Pos': setpoint, 'Vel_FF': 0.0, 'Torque_FF': 0.0})
        
        self._send_message(axis_id=axis_id, command_name="set_input_pos", data=data, error_descriptor="position target")

    def _set_control_mode(self, axis_id, control_mode, input_mode):
        # Sets the control and input mode for the nominated axis. Is called a few times so easier to have a standalone function
        data = self.db.encode_message(f'Axis{axis_id}_Set_Controller_Mode', 
                                    {'Control_Mode': control_mode, 'Input_Mode':input_mode})
        
        command_name = "set_controller_mode"

        self._send_message(axis_id=axis_id, command_name=command_name, data=data, error_descriptor="Setting control mode")

    def _set_trap_traj_vel_acc(self, axis_id, vel_limit, acc_limit):
        # Start with the velocity limit
        data = self.db.encode_message(f'Axis{axis_id}_Set_Traj_Vel_Limit', 
                                    {'Traj_Vel_Limit':vel_limit})
        
        command_name = "set_traj_vel_limit"

        self._send_message(axis_id=axis_id, command_name=command_name, data=data, error_descriptor="Trap traj vel limit")

        # Now update the acceleration limit
        data = self.db.encode_message(f'Axis{axis_id}_Set_Traj_Accel_Limits', 
                                    {'Traj_Accel_Limit': acc_limit, 'Traj_Decel_Limit': acc_limit * 0.7}) # Am definitely going to forget about that 0.7 ...
        
        command_name = "set_traj_acc_limits"

        self._send_message(axis_id=axis_id, command_name=command_name, data=data, error_descriptor="Trap traj acc limit")

    def _set_requested_state(self, axis_id, requested_state):
        data = self.db.encode_message(f'Axis{axis_id}_Set_Axis_State', 
                                    {'Axis_Requested_State': requested_state})
        
        command_name = "set_requested_state"

        self._send_message(axis_id=axis_id, command_name=command_name, data=data, error_descriptor="Setting requested state")

    def clear_errors(self):
        self.ROS_logger.info('Clearing ODrive errors...')
        command_name = "clear_errors"

        # Clear errors for all axes
        for axisID in range(6):
            self._send_message(axis_id=axisID, command_name=command_name, error_descriptor="Clearing errors")

        # Reset fatal issue flag
        self.fatal_issue = False

    def reboot_odrives(self):
        self.ROS_logger.info("Rebooting ODrives...")
        command_name = "reboot_odrives"
        # Reboots all ODrives connected on the bus
        for axisID in range(6):
            self._send_message(axis_id=axisID, command_name=command_name, error_descriptor="Rebooting ODrives")

        time.sleep(10)  # To give the ODrives plenty of time to reboot
        self.ROS_logger.info("ODrives rebooted!")
        
    def home_robot(self):
        # Start by resetting the motor limits
        self.setup_odrives()

        # Runs all the motors until the physical limit reached (current spikes)
        self.ROS_logger.info("Homing motors...")
        homing_speed = 1.0 # Go real slow
        current_limit = 5.0  # Found experimentally
        current_limit_headroom = 3.0  # Headroom for limit of how high the current should go above "current_limit"

        for axisID in range(6):
            # Put the axis in velocity control mode
            # self._set_requested_state(axisID, requested_state='CLOSED_LOOP_CONTROL')  # 'setup_odrives' already does this
            self._set_control_mode(axis_id=axisID, control_mode='VELOCITY_CONTROL', input_mode='VEL_RAMP')

            # Set absolute current/velocity limit. Don't use the 'set_absolute_vel_curr_limits' method as it sets all axes simultaneously
            data = self.db.encode_message(f'Axis{axisID}_Set_Limits',{'Current_Limit':current_limit + current_limit_headroom,
                                                                 'Velocity_Limit':homing_speed*2})
            
            self._send_message(axis_id=axisID, command_name="set_vel_curr_limits", data=data)

            # Start the motor moving at the prescribed speed
            data = self.db.encode_message(f'Axis{axisID}_Set_Input_Vel',
                                        {'Input_Vel': homing_speed, 'Input_Torque_FF': 0.0})
            
            self._send_message(axis_id=axisID, command_name="set_input_vel", data=data, error_descriptor="Setting input vel")

            time.sleep(0.5)  # Give enough time for the motor to start moving

            # Stuff for an exponential average filter
            avg_weight = 0.7
            moving_avg = 0

            while True:
                if self.fatal_issue:
                    # eg. overcurrent
                    self.ROS_logger.fatal("FATAL ISSUE! Stopping process")
                    break

                # Fetch any recent messages on the CAN bus
                self.fetch_messages()
                
                # Get the current for this motor
                current = self.iq_values[axisID]

                # Accumulate in exponential moving average
                moving_avg = moving_avg * avg_weight + current * (1 - avg_weight)

                # Check if the limit has been reached
                if abs(moving_avg) >= current_limit:
                    self._set_requested_state(axisID, requested_state='IDLE')

                    # Set the encoder position to be a little more than 0 (so that 0 is a bit off the end-stop. +ve is contraction)
                    data = self.db.encode_message(f'Axis{axisID}_Set_Linear_Count',
                                             {'Position': 1200})  # Recall encoder has 8192 CPR
                    
                    self._send_message(axis_id=axisID, command_name="set_linear_count", data=data, error_descriptor="Set encoder pos in homing")
                    self.ROS_logger.info(f"Motor {axisID} homed!")
                    break

        # Now that all motors have been homed, reset them to have desired properties
        self.setup_odrives()

    #########################################################################################################
    #                                            Managing ODrives                                           #
    #########################################################################################################

    def setup_odrives(self):
        # Call each of the "set" methods with their default values
        self.ROS_logger.info('Setting up ODrives with default values...')

        self.set_absolute_vel_curr_limits()
        self.set_control_and_input_mode()
        self.set_trap_traj_vel_acc_limits()
        self.set_odrive_state()

    def set_absolute_vel_curr_limits(self, current_limit=20.0, velocity_limit=50.0):
        # Set the absolute velocity and current limits. If these values are exceeded, the ODrive will throw an error
        for axis_id in range(6):
            data = self.db.encode_message(f'Axis{axis_id}_Set_Limits', 
                                        {'Velocity_Limit':velocity_limit, 'Current_Limit': current_limit})
                
            self._send_message(axis_id=axis_id, command_name="set_vel_curr_limits", data=data, error_descriptor='Current/Vel Limits')
            time.sleep(0.005)

        self.ROS_logger.info(f'Absolute limits set. Velocity limit: {velocity_limit} rev/s, Current limit: {current_limit}')
        
    def set_control_and_input_mode(self, control_mode='POSITION_CONTROL', input_mode='TRAP_TRAJ'):
        ''' Set the control and input modes for the ODrives. 

        control_mode options include:
         - 'TORQUE_CONTROL' (not currently used for Jugglebot)
         - 'VELOCITY_CONTROL'
         - 'POSITION_CONTROL'

        input_mode options include:
         - 'TRAP_TRAJ'
         - 'PASSTHROUGH'
         - 'VEL_RAMP'
         - etc. See ODrive.Controller.InputMode in the ODrive docs for more details
        '''
        for axis_id in range(6):
            self._set_control_mode(axis_id=axis_id, control_mode=control_mode, input_mode=input_mode)
            time.sleep(0.005)

        self.ROS_logger.info(f'Control mode set to: {control_mode}, input mode set to: {input_mode}')

    def set_trap_traj_vel_acc_limits(self, velocity_limit=10.0, acceleration_limit=10.0):
        # Set the velocity and acceleration limits while in trap_traj control mode
        for axis_id in range(6):
            self._set_trap_traj_vel_acc(axis_id=axis_id, vel_limit=velocity_limit, acc_limit=acceleration_limit)
            time.sleep(0.005)

        self.ROS_logger.info(f'Trap traj limits set. Vel: {velocity_limit}, Acc: {acceleration_limit}')

    def set_odrive_state(self, requested_state='CLOSED_LOOP_CONTROL'):
        # Set the state of the ODrives. Options are "IDLE" or "CLOSED_LOOP CONTROL" (there are others, but Jugglebot doesn't use them)
        for axis_id in range(6):
            self._set_requested_state(axis_id=axis_id, requested_state=requested_state)    
            time.sleep(0.005)

        self.ROS_logger.info(f'ODrive state changed to {requested_state}')
    
    #########################################################################################################
    #                                       Fetching and handling messages                                  #
    #########################################################################################################

    def fetch_messages(self):
        # This method is designed to be called in a loop from the main script.
        # It checks for new messages on the CAN bus and processes them if there are any.
        while True:
            # Perform a non-blocking read from the CAN bus
            message = self.bus.recv(timeout=0)
            if message is not None:
                self.handle_message(message)
            else:
                break

    def handle_message(self, message):
        # This method processes a single CAN message.
        # It extracts the important information and delegates to the appropriate handler method.
        
        # The arbitration_id contains encoded information about the axis and command.
        arbitration_id = message.arbitration_id

        # Check if the message is for the CAN traffic report
        if arbitration_id == 0x20:
            self._handle_CAN_traffic_report(message)
            return
        
        # Extract the axis ID from the arbitration ID by right-shifting by 5 bits.
        axis_id = arbitration_id >> 5
        
        # Extract the command ID from the arbitration ID by masking with 0x1F (binary 00011111).
        command_id = arbitration_id & 0x1F

        # Retrieve the handler function for this command ID from the dictionary.
        handler = self.command_handlers.get(command_id)
        if handler:
            # If a handler was found, call it with the axis ID and message data.
            handler(axis_id, message.data)
        else:
            # If no handler was found, log a warning.
            self.ROS_logger.warn(f"No handler for command ID {command_id} on axis {axis_id}. Arbitration ID: {arbitration_id}")

    def _handle_heartbeat(self, axis_id, data):
        # Extract information from the heartbeat message
        axis_error = data[0]  # Not going to do anything with this for now as none of these errors are useful for me
        axis_current_state = data[4]
        motor_error_flag = (data[5] >> 0) & 0x01  # Extract bit 0
        encoder_error_flag = (data[6] >> 0) & 0x01  # Extract bit 0
        controller_error_flag = (data[7] >> 0) & 0x01  # Extract bit 0
        trajectory_done_flag = (data[7] >> 7) & 0x01  # Extract bit 7

        # Update the current state information for this axis
        self.axis_states[axis_id] = axis_current_state
        self.is_done_moving[axis_id] = trajectory_done_flag

        # Check for any errors and set the error_detected flag if any are found
        if motor_error_flag or encoder_error_flag or controller_error_flag:
            self.fatal_issue = True  # Will be important for stopping the robot as soon as any errors are detected
            self.ROS_logger.fatal('Fatal ODrive issue. Searching for more detail...')

            # Now call specific error handlers to get more info about the nature of the error
            if motor_error_flag:
                self._request_error_info(axis_id, self.COMMANDS["get_motor_error"] )
            if encoder_error_flag:
                self._request_error_info(axis_id, self.COMMANDS["get_encoder_error"] )
            if controller_error_flag:
                self._request_error_info(axis_id, self.COMMANDS["get_controller_error"])
            
            # Request the next message so that we can see what the actual error is.
            # This seems to be working stably in testing, though it doesn't feel 100% robust
            time.sleep(0.01)
            self.fetch_messages()

        # Record the time when this message came through
        self.last_heartbeat_time = time.time()

    def _request_error_info(self, axis_id, command_id):
        # Construct a message to request more details about the specific error
        # The arbitration ID includes the axis ID and the command ID
        arbitration_id = (axis_id << 5) | command_id
        message = can.Message(arbitration_id=arbitration_id, is_remote_frame=True, is_extended_id=False)

        # Send the message over the CAN bus
        self.bus.send(message)

        # Get the command name for logging purposes
        command_name = self.COMMAND_ID_TO_NAME.get(command_id, f"Unknown command {command_id}")

        # Log the issue
        self.ROS_logger.error(f"Error detected on axis {axis_id}, requesting more information with command {command_name}.")

    def _handle_motor_error(self, axis_id, message_data):
        # Convert the 8-byte array into a 64-bit integer
        error_code = int.from_bytes(message_data, byteorder="little", signed=False)

        # Dictionary mapping error codes to error messages
        MOTOR_ERRORS = {
            0x1: "PHASE_RESISTANCE_OUT_OF_RANGE: The measured motor phase resistance is outside of the plausible range.",
            0x2: "PHASE_INDUCTANCE_OUT_OF_RANGE: The measured motor phase inductance is outside of the plausible range.",
            0x8: "DRV_FAULT: The gate driver chip reported an error.",
            0x10: "CONTROL_DEADLINE_MISSED",
            0x80: "MODULATION_MAGNITUDE: The bus voltage was insufficient to push the requested current through the motor.",
            0x400: "CURRENT_SENSE_SATURATION: The current sense circuit saturated.",
            0x1000: "CURRENT_LIMIT_VIOLATION: The motor current exceeded the limit.",
            0x10000: "MODULATION_IS_NAN",
            0x20000: "MOTOR_THERMISTOR_OVER_TEMP: The motor thermistor measured a high temperature.",
            0x40000: "FET_THERMISTOR_OVER_TEMP: The inverter thermistor measured a high temperature.",
            0x80000: "TIMER_UPDATE_MISSED: A timer update event was missed.",
            0x100000: "CURRENT_MEASUREMENT_UNAVAILABLE: The phase current measurement is not available.",
            0x200000: "CONTROLLER_FAILED: The motor was disarmed because the underlying controller failed.",
            0x400000: "I_BUS_OUT_OF_RANGE: The DC current exceeded the configured hard limits.",
            0x800000: "BRAKE_RESISTOR_DISARMED: An attempt was made to run the motor while the brake resistor was disarmed.",
            0x1000000: "SYSTEM_LEVEL: The motor was disarmed because of a system level error.",
            0x2000000: "BAD_TIMING: The main control loop got out of sync.",
            0x4000000: "UNKNOWN_PHASE_ESTIMATE: The controller did not get a valid angle input.",
            0x8000000: "UNKNOWN_PHASE_VEL: The motor controller did not get a valid phase velocity input.",
            0x10000000: "UNKNOWN_TORQUE: The motor controller did not get a valid torque input.",
            0x20000000: "UNKNOWN_CURRENT_COMMAND: The current controller did not get a valid current setpoint.",
            0x40000000: "UNKNOWN_CURRENT_MEASUREMENT: The current controller did not get a valid current measurement.",
            0x80000000: "UNKNOWN_VBUS_VOLTAGE: The current controller did not get a valid Vbus voltage measurement.",
            0x100000000: "UNKNOWN_VOLTAGE_COMMAND: The current controller did not get a valid voltage setpoint.",
            0x200000000: "UNKNOWN_GAINS: The current controller gains were not configured.",
            0x400000000: "CONTROLLER_INITIALIZING: Internal value used while the controller is not ready.",
            0x800000000: "UNBALANCED_PHASES: The motor phases are not balanced.",
        }

        # Check if the error code is in the dictionary
        if error_code in MOTOR_ERRORS:
            # Log the corresponding error message
            self.ROS_logger.error(f"Error on axis {axis_id}: {MOTOR_ERRORS[error_code]}")
        else:
            # If the error code is not recognized, log a default message
            self.ROS_logger.error(f"Unknown motor error with code {error_code} on axis {axis_id}")

    def _handle_encoder_error(self, axis_id, message_data):
        # Convert the 4-byte array into a 32-bit integer
        error_code = int.from_bytes(message_data, byteorder="little", signed=False)

        # Dictionary mapping error codes to error messages
        ENCODER_ERRORS = {
            0x1: "UNSTABLE_GAIN",
            0x2: "CPR_POLEPAIRS_MISMATCH: Confirm you have entered the correct count per rotation (CPR) for your encoder. Check motor pole pairs value and consider increasing config.calib_scan_distance.",
            0x4: "NO_RESPONSE: Confirm that your encoder is plugged into the right pins on the ODrive board.",
            0x8: "UNSUPPORTED_ENCODER_MODE",
            0x10: "ILLEGAL_HALL_STATE: An invalid state can be caused by noise or a hardware fault. Consider adding 22nF capacitors between the encoder A,B,Z pins and ground.",
            0x20: "INDEX_NOT_FOUND_YET: Check that your encoder model has an index pulse and is connected to pin Z on your ODrive.",
            0x40: "ABS_SPI_TIMEOUT",
            0x80: "ABS_SPI_COM_FAIL",
            0x100: "ABS_SPI_NOT_READY",
            0x200: "HALL_NOT_CALIBRATED_YET",
        }

        # Check if the error code is in the dictionary
        if error_code in ENCODER_ERRORS:
            # Log the corresponding error message
            self.ROS_logger.error(f"Encoder error on axis {axis_id}: {ENCODER_ERRORS[error_code]}")
        else:
            # If the error code is not recognized, log a default message
            self.ROS_logger.error(f"Unknown encoder error with code {error_code} on axis {axis_id}")

    def _handle_controller_error(self, axis_id, message_data):
        # Convert the 4-byte array into a 32-bit integer
        error_code = int.from_bytes(message_data, byteorder="little", signed=False)

        # Dictionary mapping error codes to error messages
        CONTROLLER_ERRORS = {
            0x1: "OVERSPEED: Motor speed exceeded allowed limits. Consider increasing config.vel_limit or config.vel_limit_tolerance, or set config.enable_overspeed_error to False.",
            0x2: "INVALID_INPUT_MODE: The config.input_mode setting was set to an invalid value. Ensure it matches one of the available values in InputMode.",
            0x4: "UNSTABLE_GAIN: The motor.config.bandwidth is too high, leading to an unstable controller. Consider reducing it.",
            0x8: "INVALID_MIRROR_AXIS: An invalid axis_to_mirror was selected. Ensure your configuration is correct.",
            0x10: "INVALID_LOAD_ENCODER: An invalid load_encoder_axis was selected. Ensure your configuration is correct.",
            0x20: "INVALID_ESTIMATE: The encoder estimation module declined to output a linear position or velocity value.",
            0x40: "INVALID_CIRCULAR_RANGE: The encoder estimation module declined to output a circular position value.",
            0x80: "SPINOUT_DETECTED: Discrepancy between motor mechanical and electrical power, potentially due to slipping encoder or incorrect encoder offset calibration. Check encoder attachment and wiring.",
        }

        # Check if the error code is in the dictionary
        if error_code in CONTROLLER_ERRORS:
            # Log the corresponding error message
            self.ROS_logger.error(f"Controller error on axis {axis_id}: {CONTROLLER_ERRORS[error_code]}")
        else:
            # If the error code is not recognized, log a default message
            self.ROS_logger.error(f"Unknown controller error with code {error_code} on axis {axis_id}")

    def _handle_encoder_estimates(self, axis_id, data):
        # Start by unpacking the data, which is a 32-bit float
        pos_estimate, vel_estimate = struct.unpack_from('<ff', data)

        # Add the data to the appropriate buffer
        self.position_buffer[axis_id] = pos_estimate
        self.velocity_buffer[axis_id] = vel_estimate

        if all(self.position_buffer):
            # If the buffer is full, send it off!
            complete_pos_data = self.position_buffer.copy() # Copy so that data doesn't change before being sent away
            self._trigger_callback('motor_positions', complete_pos_data)
            self.position_buffer = [None] * self.num_axes  # Reset the buffer

            # If position buffer is full, then velocity buffer will also be full
            complete_vel_data = self.velocity_buffer.copy()
            self._trigger_callback('motor_velocities', complete_vel_data)
            self.velocity_buffer = [None] * self.num_axes  # Reset the buffer

    def _handle_iq_readings(self, axis_id, data):
        # Start by unpacking the data, which is a 32-bit float
        iq_setpoint, iq_measured = struct.unpack_from('<ff', data)

        # Add the data to the buffer
        self.iq_buffer[axis_id] = iq_measured

        if all(self.iq_buffer):
            # If the buffer is full, send the data to the ROS network
            complete_iq_data = self.iq_buffer.copy()
            self._trigger_callback('motor_iqs', complete_iq_data)

            # Store these values
            self.iq_values = complete_iq_data

            # Reset the buffer
            self.iq_buffer = [None] * self.num_axes

    def _handle_CAN_traffic_report(self, message):
        # Unpack the data into a 32-bit integer
        received_count = message.data[0] + (message.data[1] << 8)
        report_interval = message.data[2] + (message.data[3] << 8)

        traffic_report = {
            'received_count': received_count,
            'report_interval': report_interval,
        }

        # Log the traffic report
        self.ROS_logger.debug(f"CAN traffic report: {received_count} messages received in the last {report_interval} ms.")

        # Trigger the callback
        self._trigger_callback('can_traffic', traffic_report)