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
        (e.g., stopping the robot, printing error information, etc.).

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

import can
import cantools
import struct
import time
from collections import deque

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

    # Create the reverse dictionary to go from ID to name; mostly for printing human-readable errors
    COMMAND_ID_TO_NAME = {v: k for k, v in COMMANDS.items()}

    def __init__(self, bus_name='can0', bitrate=1000000, bus_type='socketcan'):
        # Import the ODrive dbc file
        self.db = cantools.database.load_file("ODrive_CAN.dbc")

        # Initialize the parameters for the CAN bus that will be used by __enter__ to initialise the bus itself
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

        # Initialize a flag that triggers whenever any errors are detected
        self.fatal_issue = False

        # Initialize the current state for each axis
        self.axis_states = [None] * 6
        self.is_done_moving = [None] * 6

        # Initialize the dictionary for storing encoder estimate and IQ values
        # Buffer size is set to 0.5 hr at 100 Hz (100 samples/sec * 60 sec/min * 60 min/hr)
        self.position_estimates = {axis_id: deque(maxlen=30*60*100) for axis_id in range(6)}
        self.velocity_estimates = {axis_id: deque(maxlen=30*60*100) for axis_id in range(6)}
        self.iq_values = {axis_id: deque(maxlen=30*60*100) for axis_id in range(6)}

    # Define methods that will permit compatibility with a context manager
    def __enter__(self):
        self.bus = can.Bus(channel=self._can_bus_name, bustype=self._can_bus_type, bitrate=self._can_bitrate)
        self.flush_bus() # Flush the bus now that it's been set up

        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if self.bus is not None:
            self.bus.shutdown()

    def flush_bus(self):
        # Flushes the CAN bus to remove any stale messages
        # Read messages with a non-blocking call until there are no more
        while not (self.bus.recv(timeout=0) is None):
            pass # We're just discarding messages, so we don't need to do anything with them

    # For sending messages

    def send_message(self, axis_id, command_name, data=None, error_descriptor='Not described', rtr_bit=False):
        # Get the hex code for the message being sent
        command_id = self.COMMANDS[command_name]

        # Create the CAN message
        arbitration_id = (axis_id << 5) | command_id

        if data is not None:
            msg = can.Message(arbitration_id=arbitration_id, dlc=8, is_extended_id=False, data=data, is_remote_frame=rtr_bit)
        else:
            msg = can.Message(arbitration_id=arbitration_id, dlc=8, is_extended_id=False, is_remote_frame=rtr_bit)

        try:
            self.bus.send(msg)
        except Exception as e:
            print(f"CAN message for {error_descriptor} NOT sent to axisID {axis_id}! Error: {e}")

    def set_control_mode(self, axis_id, control_mode, input_mode):
        # Sets the control and input mode for the nominated axis. Is called a few times so easier to have a standalone function
        data = self.db.encode_message(f'Axis{axis_id}_Set_Controller_Mode', 
                                    {'Control_Mode': control_mode, 'Input_Mode':input_mode})
        
        command_name = "set_controller_mode"

        self.send_message(axis_id=axis_id, command_name=command_name, data=data, error_descriptor="Setting control mode")

    def set_trap_traj_vel_acc(self, axis_id, vel_limit, acc_limit):
        # Start with the velocity limit
        data = self.db.encode_message(f'Axis{axis_id}_Set_Traj_Vel_Limit', 
                                    {'Traj_Vel_Limit':vel_limit})
        
        command_name = "set_traj_vel_limit"

        self.send_message(axis_id=axis_id, command_name=command_name, data=data, error_descriptor="Trap traj vel limit")

        # Now update the acceleration limit
        data = self.db.encode_message(f'Axis{axis_id}_Set_Traj_Accel_Limits', 
                                    {'Traj_Accel_Limit': acc_limit, 'Traj_Decel_Limit': acc_limit})
        
        command_name = "set_traj_acc_limits"

        self.send_message(axis_id=axis_id, command_name=command_name, data=data, error_descriptor="Trap traj acc limit")

    def set_requested_state(self, axis_id, requested_state):
        data = self.db.encode_message(f'Axis{axis_id}_Set_Axis_State', 
                                    {'Axis_Requested_State': requested_state})
        
        command_name = "set_requested_state"

        self.send_message(axis_id=axis_id, command_name=command_name, data=data)

    def clear_errors(self):
        print("Clearing errors...")
        command_name = "clear_errors"
        # Clear errors for all axes
        for axisID in range(6):
            self.send_message(axis_id=axisID, command_name=command_name, error_descriptor="Clearing errors")

    def reboot_odrives(self):
        print("Rebooting ODrives...")
        command_name = "reboot_odrives"
        # Reboots all ODrives connected on the bus
        for axisID in range(6):
            self.send_message(axis_id=axisID, command_name=command_name, error_descriptor="Rebooting ODrives")

        time.sleep(10)  # To give the ODrives plenty of time to reboot
        print("ODrives rebooted!")
        
    # For fetching and handling messages

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
        
        # Extract the axis ID from the arbitration ID by right-shifting by 5 bits.
        axis_id = arbitration_id >> 5
        
        # Extract the command ID from the arbitration ID by masking with 0x1F (binary 00011111).
        command_id = arbitration_id & 0x1F

        # Retrieve the handler function for this command ID from the dictionary.
        handler = self.command_handlers.get(command_id)
        if handler:
            # If a handler was found, call it with the axis ID and message data.
            handler(axis_id, message.data)

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

            # Now call specific error handlers to get more info about the nature of the error
            if motor_error_flag:
                self._request_error_info(axis_id, self.COMMANDS["get_motor_error"] )
            if encoder_error_flag:
                self._request_error_info(axis_id, self.COMMANDS["get_encoder_error"] )
            if controller_error_flag:
                self._request_error_info(axis_id, self.COMMANDS["get_controller_error"])
            
            # Request the next message so that we can see what the actual error is.
            # This seems to be working stably in testing, though it doesn't seem 100% robust
            time.sleep(0.05)
            self.fetch_messages()

    def _request_error_info(self, axis_id, command_id):
        # Construct a message to request more details about the specific error
        # The arbitration ID includes the axis ID and the command ID
        arbitration_id = (axis_id << 5) | command_id
        message = can.Message(arbitration_id=arbitration_id, is_remote_frame=True, is_extended_id=False)

        # Send the message over the CAN bus
        self.bus.send(message)

        # Get the command name for printing purposes
        command_name = self.COMMAND_ID_TO_NAME.get(command_id, f"Unknown command {command_id}")

        # Add a comment for the user to know what's going on
        print(f"Error detected on axis {axis_id}, requesting more information with command {command_name}.")

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
            # Print the corresponding error message
            print(f"Error on axis {axis_id}: {MOTOR_ERRORS[error_code]}")
        else:
            # If the error code is not recognized, print a default message
            print(f"Unknown motor error with code {error_code} on axis {axis_id}")

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
            # Print the corresponding error message
            print(f"Encoder error on axis {axis_id}: {ENCODER_ERRORS[error_code]}")
        else:
            # If the error code is not recognized, print a default message
            print(f"Unknown encoder error with code {error_code} on axis {axis_id}")

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
            # Print the corresponding error message
            print(f"Controller error on axis {axis_id}: {CONTROLLER_ERRORS[error_code]}")
        else:
            # If the error code is not recognized, print a default message
            print(f"Unknown controller error with code {error_code} on axis {axis_id}")

    def _handle_encoder_estimates(self, axis_id, data):
        # Start by unpacking the data, which is a 32-bit float
        pos_estimate, vel_estimate = struct.unpack_from('<ff', data)

        # I (currently) don't need both pos and vel estimates, so keep pos and replace vel with a timestamp
        # Could do all three, but I'm wary of hammering my RAM with so much data being handled at once
        current_time = time.time()

        # Append the estimates to the corresponding motor's deque
        # Deque will automatically remove the oldest element if the limit is exceeded
        self.position_estimates[axis_id].append((pos_estimate, current_time))
        self.velocity_estimates[axis_id].append((vel_estimate, current_time))

    def get_position_estimates(self):
        # Create a dictionary to hold the estimates for all axes
        pos_estimates = {}
        timestamps = {}

        # Check if all deques in encoder_estimates are empty
        if all(len(deque) ==0 for deque in self.position_estimates.values()):
            return pos_estimates, timestamps

        # Iterate over all axes
        for axis_id, estimates in self.position_estimates.items():
            # Extract the pos and vel estimates into separate dictionaries
            pos_estimates[axis_id], timestamps[axis_id] = zip(*estimates)

        return pos_estimates, timestamps
    
    def get_velocity_estimates(self):
        # Create a dictionary to hold the estimates for all axes
        vel_estimates = {}
        timestamps = {}

        # Check if all deques in encoder_estimates are empty
        if all(len(deque) ==0 for deque in self.velocity_estimates.values()):
            return vel_estimates, timestamps

        # Iterate over all axes
        for axis_id, estimates in self.position_estimates.items():
            # Extract the pos and vel estimates into separate dictionaries
            vel_estimates[axis_id], timestamps[axis_id] = zip(*estimates)

        return vel_estimates, timestamps
    
    def _handle_iq_readings(self, axis_id, data):
        # Start by unpacking the data, which is a 32-bit float
        iq_setpoint, iq_measured = struct.unpack_from('<ff', data)

        # Append the estimates to the corresponding motor's deque
        # Deque will automatically remove the oldest element if the limit is exceeded
        self.iq_values[axis_id].append((iq_setpoint, iq_measured))

    def get_iq_values(self):
        # Create a dictionary to hold the estimates for all axes
        iq_setpoints = {}
        iq_measured = {}

        # Check if all deques in iq_values are empty
        if all(len(deque) ==0 for deque in self.iq_values.values()):
            return iq_setpoints, iq_measured

        # Iterate over all axes
        for axis_id, values in self.iq_values.items():
            # Extract the setpoint and measured values into separate dictionaries
            iq_setpoints[axis_id], iq_measured[axis_id] = zip(*values)

        return iq_setpoints, iq_measured

