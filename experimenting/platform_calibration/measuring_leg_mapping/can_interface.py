"""
CANInterface Class
----------------
This class is designed to interface with a CAN bus using the python-can library. 
It's specifically tailored for handling communications for a robot powered by six BLDC motors, controlled by ODrive Pro controllers. 
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


NOTE that this version of can_interface has been modified for the leg mapping procedure.

**** THIS VERSION OF can_interface SHOULD NOT BE USED FOR ACTUAL JUGGLEBOT OPERATION!!! ****
"""

import os
import struct
import threading
import time
import math
from typing import Callable, Dict, List, Optional

import can
import cantools
from jugglebot_interfaces.msg import MotorStateSingle

class CANInterface:
    """
    Interface for handling CAN bus communication with ODrive controllers.
    """
    # Create a dictionary of ODrive-related commands and their corresponding IDs
    COMMANDS = {
        "heartbeat_message"       : 0x01,
        "get_error"               : 0x03,
        "RxSdo"                   : 0x04, # Write to arbitrary parameter
        "TxSdo"                   : 0x05, # Read from arbitrary parameter
        "set_requested_state"     : 0x07,
        "get_encoder_estimate"    : 0x09,
        "set_controller_mode"     : 0x0b,
        "set_input_pos"           : 0x0c,
        "set_input_vel"           : 0x0d,
        "set_vel_curr_limits"     : 0x0f,
        "set_traj_vel_limit"      : 0x11,
        "set_traj_acc_limits"     : 0x12,
        "get_iq"                  : 0x14,
        "get_temps"               : 0x15, 
        "reboot_odrives"          : 0x16,
        "get_bus_voltage_current" : 0x17,
        "clear_errors"            : 0x18,
        "set_absolute_position"   : 0x19,
        "set_pos_gain"            : 0x1a,
        "set_vel_gains"           : 0x1b,
    }

    ARBITRARY_PARAMETER_IDS = {
        "commutation_mapper.pos_abs"  : 488, # NaN until encoder search is complete
    }

    OPCODE_READ  = 0x00  # For reading arbitrary parameters from the ODrive
    OPCODE_WRITE = 0x01  # For writing arbitrary parameters to the ODrive

    # Create the reverse dictionary to go from ID to name; mostly for logging human-readable errors
    COMMAND_ID_TO_NAME = {v: k for k, v in COMMANDS.items()}

    # Set absolute limit for how far the motors can turn. This is intended as a backup if prior error-checking fails
    _LEG_MOTOR_MAX_POSITION  = 4.2  # Revs
    _HAND_MOTOR_MAX_POSITION = 11.1 # Revs

    # Set the limits for trapezoidal trajectory control (used only for the legs)
    _DEFAULT_TRAP_TRAJ_LIMITS = {'vel_limit': 15.0, 'acc_limit': 30.0, 'dec_limit': 30.0} # rev/s, rev/s^2
    # Set the absolute limits for the motors
    _DEFAULT_VEL_CURR_LIMITS = {'leg_vel_limit': 50.0, 'leg_curr_limit': 20.0, 
                                'hand_vel_limit': 1000.0, 'hand_curr_limit': 50.0} # rev/s, A

    def __init__(
        self,
        logger,
        odrive_num_being_tested: int,
        bus_name: str = 'can0',
        bitrate: int = 1000000,
        interface: str = 'socketcan',
    ):
        # Find the package directory
        folder_dir = os.path.dirname(os.path.abspath(__file__))
        
        # Construct the full filepath to the dbc file
        dbc_file_path = os.path.join(folder_dir, 'ODrive_Pro.dbc')

        # Now import the ODrive dbc file
        self.db = cantools.database.load_file(dbc_file_path)

        # Initialize the logger, to get access to the ROS network (for logging purposes only)
        self.ROS_logger = logger

        # Initialize the ODrive number being tested
        self.odrive_num_being_tested = odrive_num_being_tested

        # Initialize the parameters for the CAN bus that will be used by setup_can_bus to initialise the bus itself
        self._can_bus_name = bus_name
        self._can_bitrate = bitrate
        self._interface = interface
        self.bus = None

        # Create a dictionary to map command IDs to their corresponding handler
        # This allows for easy addition of new commands and corresponding handlers
        self.command_handlers: Dict[int, Callable[[int, bytes], None]] = {
            self.COMMANDS["heartbeat_message"]      : self._handle_heartbeat,
            self.COMMANDS["get_error"]              : self._handle_error,
            self.COMMANDS["get_encoder_estimate"]   : self._handle_encoder_estimates,
            self.COMMANDS["get_iq"]                 : self._handle_iq_readings,
            self.COMMANDS["get_temps"]              : self._handle_temp_readings,
            self.COMMANDS["get_bus_voltage_current"]: self._handle_bus_voltage_current_readings,
            self.COMMANDS["TxSdo"]                  : self._handle_arbitrary_parameter_read,
        }

        # Create a dictionary of callbacks that are used in the ROS2 node
        self.callbacks: Dict[str, Optional[Callable]] = {
            'can_traffic'   : None,
            # 'hand_telemetry': None,
        }

        # Initialize the number of axes that the robot has
        self.num_axes = 1

        # Initialize the motor states and the data lock for these states
        self.motor_states = [MotorStateSingle() for _ in range(self.num_axes)]
        self._motor_states_lock = threading.Lock()

        # Initialize a variable to track the position of the linear slider
        self.linear_slider_position = None
        self._linear_slider_lock = threading.Lock()

        # Initialize a local copy of the motor states so that we don't need to lock the data every time we access it
        self.last_motor_states = [MotorStateSingle() for _ in range(self.num_axes)]

        # Flags and variables for tracking errors
        self.fatal_error: bool = False
        self.undervoltage_error: bool = False # eg. me hitting the E-stop
        self.fatal_can_error: bool = False

        # Variable to store the result of checking whether the encoder search is complete
        self._received_encoder_search_feedback_on_axes = [None] * 6 # Only the legs need to do this, as the hand uses the on-board encoder

        '''
        Thread lock for thread safety
        A re-entrant lock is used to allow the same thread to acquire the lock multiple times without causing a deadlock.
        This is necessary for clearing errors after an error has been detected, as the `send_message` method needs access
        to the lock that `fetch_messages` is using.
        '''
        self._can_lock = threading.RLock()

        # Initialize the tilt readings that will come in from the SCL3300 sensor (via the Teensy)
        self.tilt_sensor_reading = None 

        # Initialize CAN arbitration IDs for messages to/from the Teensy
        # See the Teensy firmware for a note on "safe" IDs to not conflict with the ODrives
        self.linear_slider_CAN_ID = 0x7E0

        # Initialize error-related parameters
        self.max_reset_attempts_after_soft_error = 1 # Number of times to try resetting after encountering a soft error before throwing a fatal error
        self.reset_attempts_after_soft_error = 0 # Number of times the robot has tried to reset after encountering a soft error
        self._last_error_log_times = {} # Axis ID to error code to last log time
        self.error_log_throttle_duration_sec = 10.0 # Throttle duration for error logging. This is per axis, per error, so it doesn't
                                                    # need to be called often

        # Initialize leg and hand motor absolute limits
        self.leg_motor_abs_limits = {'velocity_limit': self._DEFAULT_VEL_CURR_LIMITS['leg_vel_limit'], 
                                     'current_limit': self._DEFAULT_VEL_CURR_LIMITS['leg_curr_limit']}
        self.hand_motor_abs_limits = {'velocity_limit': self._DEFAULT_VEL_CURR_LIMITS['hand_vel_limit'], 
                                      'current_limit': self._DEFAULT_VEL_CURR_LIMITS['hand_curr_limit']}

        # Initialize leg trapezoidal trajectory limits
        self.leg_trap_traj_limits = {'vel_limit': self._DEFAULT_TRAP_TRAJ_LIMITS['vel_limit'],
                                     'acc_limit': self._DEFAULT_TRAP_TRAJ_LIMITS['acc_limit'],
                                     'dec_limit': self._DEFAULT_TRAP_TRAJ_LIMITS['dec_limit']}


        # Set up the CAN bus to establish communication with the robot
        self.setup_can_bus()

    #########################################################################################################
    #                                            CAN Bus Management                                         #
    #########################################################################################################

    def setup_can_bus(self):
        """
        Initializes the CAN bus.
        """
        self.bus = can.Bus(
            channel=self._can_bus_name,
            interface=self._interface,
            bitrate=self._can_bitrate
        )
        self.flush_bus() # Flush the bus now that it's been set up

    def close(self):
        """
        Closes the CAN bus connection.
        """
        if self.bus is not None:
            try:
                self.bus.shutdown()
                self.ROS_logger.info('CAN bus closed')
            except Exception as e:
                self.ROS_logger.error(f'Error when closing CAN bus: {e}')

    def flush_bus(self):
        """
        Flushes the CAN bus to remove any stale messages.
        """
        # Flushes the CAN bus to remove any stale messages
        # Read messages with a non-blocking call until there are no more
        if self.bus:
            while self.bus.recv(timeout=0):
                pass # We're just discarding messages, so we don't need to do anything with them

    def attempt_to_restore_can_connection(self):
        """
        Attempts to restore the CAN bus connection after a failure.
        """
        # If the bus isn't responding, try to re-establish the connection
        max_retries = 3  # Maximum number of reconnection attempts
        retry_delay = 5  # Delay in seconds between retries
        reconnect_timeout = 1  # How long to wait for the heartbeat to come in before considering the connection dead

        self.ROS_logger.info('Attempting to restore CAN connection...')

        for attempt in range(1, max_retries + 1):
            rx_working = False
            tx_working = False
            try:
                # Close existing connection if open
                if self.bus is not None:
                    self.close()

                # Reinitialize the CAN bus
                self.setup_can_bus()

                time.sleep(0.1)
                # self.fetch_messages()

                # Wait for heartbeat messages
                start_time = time.time()
                while time.time() - start_time < reconnect_timeout:
                    # self.fetch_messages()
                    if all(self.received_heartbeats.values()):
                        self.ROS_logger.info(f"CAN bus read successful on attempt {attempt}")
                        rx_working = True
                        break
                    time.sleep(0.1)

                # Test sending a message
                msg = can.Message(
                    arbitration_id=0x999,
                    dlc=8,
                    is_extended_id=False,
                    data=[0] * 8,
                )
                try:
                    with self._can_lock:
                        self.bus.send(msg)
                    self.ROS_logger.info(f"CAN bus write successful on attempt {attempt}")
                    tx_working = True
                except Exception as e:
                    self.ROS_logger.warning(f"Failed to write to CAN bus on attempt {attempt}: {e}")

                if rx_working and tx_working:
                    self.ROS_logger.info('CAN bus connection restored!')
                    self.fatal_can_error = False
                    return
            except Exception as e:
                self.ROS_logger.warning(f"Failed to reconnect CAN bus on attempt {attempt}: {e}")

            time.sleep(retry_delay)  # Wait before retrying

        self.ROS_logger.error(f"Failed to restore CAN bus connection after {max_retries} attempts. Closing CAN bus...")
        self.close()
        self.fatal_can_error = True  # Report the fatal CAN issue so that the handler node can stop attempting to send messages

    #########################################################################################################
    #                                         ROS2 Callback Management                                      #
    #########################################################################################################

    def register_callback(self, data_type: str, callback: Callable):
        """
        Registers a callback for a specific data type.

        Args:
            data_type (str): The type of data (e.g., 'leg_positions').
            callback (Callable): The callback function.
        """
        if data_type in self.callbacks:
            self.callbacks[data_type] = callback

    def _trigger_callback(self, data_type: str, data):
        """
        Triggers the registered callback for a data type.

        Args:
            data_type (str): The type of data.
            data: The data to pass to the callback.
        """
        if self.callbacks.get(data_type):
            try:
                self.callbacks[data_type](data)
                self.ROS_logger.debug(f"Callback triggered for data type: {data_type}")
            except Exception as e:
                self.ROS_logger.error(f"Error in callback for {data_type}: {e}")

    #########################################################################################################
    #                                             Sending messages                                          #
    #########################################################################################################

    def _send_message(self, axis_id, command_name, data=None, error_descriptor='Not described', rtr_bit=False):
        """
        Sends a message over the CAN bus to a specific axis.

        Args:
            axis_id: The axis ID.
            command_name: The command name.
            data: The data to send.
            error_descriptor: Description for error logging.
            rtr_bit: Whether to set the RTR bit.

        Raises:
            Exception: If sending the message fails.
        """

        if not self.bus:
            # If the bus hasn't been initialized, return
            return

        # Get the hex code for the message being sent
        command_id = self.COMMANDS[command_name]

        if command_id is None:
            self.ROS_logger.error(f"Invalid command name in _send_message: {command_name}")
            return

        # Create the CAN message
        arbitration_id = (axis_id << 5) | command_id

        if data is not None:
            msg = can.Message(arbitration_id=arbitration_id, dlc=8, is_extended_id=False, data=data, is_remote_frame=rtr_bit)
        else:
            # For messages that require no data to be sent (eg. get encoder estimates)
            msg = can.Message(arbitration_id=arbitration_id, dlc=8, is_extended_id=False, is_remote_frame=rtr_bit)

        try:
            with self._can_lock:
                self.bus.send(msg)
            # self.ROS_logger.info(f"CAN message for {error_descriptor} sent to axisID {axis_id}")
            # self.ROS_logger.info(f"msg: {msg} for axis {axis_id} with command {command_name}")
        except can.CanError as e:
            self.ROS_logger.warn(f"CAN message for {error_descriptor} NOT sent to axisID {axis_id}! Error: {e}")

            # If the buffer is full, the CAN bus is probably having issue. Try to re-establish it
            if "105" in str(e):
                # "No buffer space available" is error code 105
                self.attempt_to_restore_can_connection()
        
        except Exception as e:
            self.ROS_logger.error(f"Error sending message to axis {axis_id} for {error_descriptor}: {e}")
            raise

    def send_arbitrary_parameter(
        self,
        axis_id: int,
        param_name: str,
        param_value: float,
        param_type: str = 'f', # Default to float
        op_code: int = OPCODE_WRITE  # Default to OPCODE_WRITE
    ) -> None:
        """
        Sends an arbitrary parameter to the ODrive.

        Args:
            axis_id (int): The axis ID to send the parameter to.
            param_name (str): The name of the parameter to set.
            param_value (float): The value to set for the parameter.
            param_type (str, optional): The format character for struct packing (default is 'f' for float).
            op_code (int, optional): The operation code (0x00 for read, 0x01 for write).

        Raises:
            ValueError: If the parameter name is invalid.
            Exception: If sending the message fails.
        """
        try:
            # Get the endpoint ID for the parameter
            endpoint_id = self.ARBITRARY_PARAMETER_IDS.get(param_name)
            if endpoint_id is None:
                raise ValueError(f"Invalid parameter name: {param_name}")

            # Pack the data into the correct format
            data = struct.pack('<BHB' + param_type, op_code, endpoint_id, 0, param_value)

            # Send the message
            self._send_message(
                axis_id=axis_id,
                command_name="RxSdo",
                data=data,
                error_descriptor=f"arbitrary parameter: {endpoint_id}"
            )
            # self.ROS_logger.info(f"Arbitrary parameter '{param_name}' sent to axis {axis_id} with value {param_value}")
        except Exception as e:
            self.ROS_logger.error(f"Failed to send arbitrary parameter '{param_name}' to axis {axis_id}: {e}")
            raise

    def send_position_target(
        self,
        axis_id: int,
        setpoint: float,
        vel_ff: int = 0, # These are ints because that's what the ODrive uses
        torque_ff: int = 0,
        min_position: float = 0.0
    ) -> None:
        """
        Commands the given motor to move to the designated setpoint after checking its magnitude.

        Args:
            axis_id (int): The axis ID to command.
            setpoint (float): The desired position setpoint in revolutions.
            vel_ff (float, optional): The velocity feedforward term.
            torque_ff (float, optional): The torque feedforward term.
            min_position (float, optional): The minimum allowable position.

        Raises:
            Exception: If sending the message fails.
        """
        try:
            # Check and clip setpoint to allowable bounds
            if axis_id < 6:
                max_position = self._LEG_MOTOR_MAX_POSITION
            elif axis_id == 6:
                max_position = self._HAND_MOTOR_MAX_POSITION
            else:
                raise ValueError(f"Invalid axis ID: {axis_id}")

            if setpoint > max_position or setpoint < min_position:
                self.ROS_logger.warning(
                    f"Setpoint of {setpoint:.2f} for axis {axis_id} is outside allowable bounds "
                    f"({min_position}, {max_position}) and has been clipped."
                )
                setpoint = max(min_position, min(setpoint, max_position))

            # Invert setpoint for legs since -ve is extension
            if axis_id != 6:
                setpoint = -setpoint

            # Log the setpoing, vell_ff, and torque_ff
            # self.ROS_logger.info(f"Position target set for axis {axis_id}: {setpoint:.4f} revs, Vel_FF = {vel_ff:.4f}, Torque_FF = {torque_ff:.4f}")

            data = self.db.encode_message(
                f'Axis{axis_id}_Set_Input_Pos',
                {'Input_Pos': setpoint, 'Vel_FF': vel_ff, 'Torque_FF': torque_ff}
            )

            self._send_message(
                axis_id=axis_id,
                command_name="set_input_pos",
                data=data,
                error_descriptor="position target"
            )
            # self.ROS_logger.debug(f"Position target set for axis {axis_id}: {setpoint:.2f} revs")
        except Exception as e:
            self.ROS_logger.error(f"Failed to send position target to axis {axis_id}: {e}")
            raise

    def set_control_mode(
        self,
        axis_id: int,
        control_mode: str,
        input_mode: str
    ) -> None:
        ''' Set the control and input modes for the chosen axis_id.

        Args:
        axis_id (int): The axis ID to configure.
        control_mode (str): The desired control mode (e.g., 'POSITION_CONTROL').
        input_mode (str): The desired input mode (e.g., 'TRAP_TRAJ').

        Raises:
            Exception: If sending the message fails.

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
        try:
            data = self.db.encode_message(
                f'Axis{axis_id}_Set_Controller_Mode',
                {'Control_Mode': control_mode, 'Input_Mode': input_mode}
            )

            self._send_message(
                axis_id=axis_id,
                command_name="set_controller_mode",
                data=data,
                error_descriptor="setting control mode"
            )
            self.ROS_logger.debug(
                f"Control mode set for axis {axis_id}: Control Mode = {control_mode}, Input Mode = {input_mode}"
            )
        except Exception as e:
            self.ROS_logger.error(f"Failed to set control mode for axis {axis_id}: {e}")
            raise

    def _set_trap_traj_vel_acc(
        self,
        axis_id: int,
        vel_limit: float,
        acc_limit: float,
        dec_limit: float
    ) -> None:
        """
        Sets the trapezoidal trajectory velocity and acceleration limits for a specific axis.

        Args:
            axis_id (int): The axis ID to configure.
            vel_limit (float): The velocity limit.
            acc_limit (float): The acceleration limit.
            dec_limit (float): The deceleration limit.

        Raises:
            Exception: If sending the message fails.
        """
        try:
            # Set velocity limit
            data_vel = self.db.encode_message(
                f'Axis{axis_id}_Set_Traj_Vel_Limit',
                {'Traj_Vel_Limit': vel_limit}
            )
            self._send_message(
                axis_id=axis_id,
                command_name="set_traj_vel_limit",
                data=data_vel,
                error_descriptor="trapezoidal trajectory velocity limit"
            )

            # Set acceleration and deceleration limits
            data_acc = self.db.encode_message(
                f'Axis{axis_id}_Set_Traj_Accel_Limits',
                {'Traj_Accel_Limit': acc_limit, 'Traj_Decel_Limit': dec_limit}
            )
            self._send_message(
                axis_id=axis_id,
                command_name="set_traj_acc_limits",
                data=data_acc,
                error_descriptor="trapezoidal trajectory acceleration limits"
            )

            self.ROS_logger.debug(
                f"Trapezoidal trajectory limits set for axis {axis_id}: "
                f"Velocity Limit = {vel_limit}, Acceleration Limit = {acc_limit}, Deceleration Limit = {dec_limit}"
            )
        except Exception as e:
            self.ROS_logger.error(f"Failed to set trapezoidal trajectory limits for axis {axis_id}: {e}")
            raise

    def set_requested_state(
        self,
        axis_id: int,
        requested_state: str
    ) -> None:
        """
        Sets the requested state for the specified axis.

        Args:
            axis_id (int): The axis ID to set the state for.
            requested_state (str): The requested state (e.g., 'CLOSED_LOOP_CONTROL').

        Raises:
            Exception: If sending the message fails.
        """
        try:
            data = self.db.encode_message(
                f'Axis{axis_id}_Set_Axis_State',
                {'Axis_Requested_State': requested_state}
            )

            self._send_message(
                axis_id=axis_id,
                command_name="set_requested_state",
                data=data,
                error_descriptor="setting requested state"
            )
            self.ROS_logger.debug(f"Requested state '{requested_state}' set for axis {axis_id}")
        except Exception as e:
            self.ROS_logger.error(f"Failed to set requested state for axis {axis_id}: {e}")
            raise

    def clear_errors(self) -> None:
        """
        Clears errors on all axes.
        Includes a short delay to give the ODrives time to process the command.

        Raises:
            Exception: If sending the message fails.
        """
        try:
            self.ROS_logger.info('Clearing ODrive errors...')
            command_name = "clear_errors"

            for axis_id in range(self.num_axes):
                self._send_message(
                    axis_id=axis_id,
                    command_name=command_name,
                    error_descriptor="clearing errors"
                )

            # Reset error flags
            self.fatal_error = False
            self.undervoltage_error = False

            # Clear the disarm reasons
            for motor in self.motor_states:
                motor.disarm_reason = 0

        except Exception as e:
            self.ROS_logger.error(f"Failed to clear errors: {e}")
            raise

    def reboot_odrives(self) -> None:
        """
        Reboots all ODrives connected on the bus.

        Raises:
            Exception: If sending the message fails.
        """
         ### NOTE: Would be good to add a check to see if the ODrives have actually responded to the reboot command
        try:
            self.ROS_logger.info("Rebooting ODrives...")
            command_name = "reboot_odrives"

            for axis_id in range(self.num_axes):
                self._send_message(
                    axis_id=axis_id,
                    command_name=command_name,
                    error_descriptor="rebooting ODrives"
                )
            time.sleep(10)  # Allow time for ODrives to reboot
            self.ROS_logger.info("ODrives rebooted.")
        except Exception as e:
            self.ROS_logger.error(f"Failed to reboot ODrives: {e}")
            raise
        
    def home_robot(self, axis_to_home, reset_zero_position=True) -> bool:
        """
        Homes all motors by moving them to their end stops.

        Args:
            axes_to_home (List): List of axes to home.

        Raises:
            Exception: If homing fails.
        """
        try:
            # Runs all the motors until the physical limit reached (current spikes)
            self.ROS_logger.info(f"Homing axis: {axis_to_home}")

            # Parameters for homing the motors
            leg_homing_speed = 1.5 # Go real slow
            leg_current_limit = 5.0  # Found experimentally

            current_limit_headroom = 0.0  # Headroom for limit of how high the current can go above "current_limit"

            # Run the leg downwards until the end-stop is hit
            homed = self.run_motor_until_current_limit(axis_id=axis_to_home, homing_speed=leg_homing_speed, current_limit=leg_current_limit,
                                            current_limit_headroom=current_limit_headroom)
        
            if not homed:
                self.ROS_logger.error(f"Motor {axis_to_home} homing failed!")
                return False

            if reset_zero_position:    
                # Set the encoder position to be a little more than 0 (so that 0 is a bit off the end-stop. +ve is contraction)
                data = self.db.encode_message(f'Axis{axis_to_home}_Set_Absolute_Position',
                                            {'Position': 0.1})  # Unit is revolutions
            
                self._send_message(axis_id=axis_to_home, command_name="set_absolute_position", data=data, error_descriptor="Set encoder pos in homing")
                self.ROS_logger.info(f"Motor {axis_to_home} encoder position reset!")

            self.ROS_logger.info(f"Motor {axis_to_home} homed!")
                
            return True
        
        except Exception as e:
            self.ROS_logger.error(f"Homing failed: {e}")
            return False

    def run_motor_until_current_limit(
            self,
            axis_id: int,
            homing_speed: float,
            current_limit: float,
            current_limit_headroom: float
        ) -> bool:
        """
        Runs the motor until the current limit is reached. This is used during homing.

        Args:
            axis_id (int): The axis ID to command.
            homing_speed (float): The speed at which to run the motor.
            current_limit (float): The current limit to detect end stop.
            current_limit_headroom (float): Additional current limit headroom.

        Raises:
            Exception: If homing fails.
        """
        try:
            # Ensure the axis is in CLOSED_LOOP_CONTROL mode
            self.set_requested_state(axis_id, requested_state='CLOSED_LOOP_CONTROL')
            
            # Set the hand to velocity control mode
            self.set_control_mode(axis_id=axis_id, control_mode='VELOCITY_CONTROL', input_mode='VEL_RAMP')
            
            # Set absolute current/velocity limit. Don't use the 'set_absolute_vel_curr_limits' method as it sets all axes simultaneously
            data = self.db.encode_message(f'Axis{axis_id}_Set_Limits',{'Current_Limit':current_limit + current_limit_headroom,
                                                                    'Velocity_Limit':abs(homing_speed*2)})
            
            self._send_message(axis_id=axis_id, command_name="set_vel_curr_limits", data=data)
            time.sleep(0.01)

            # Start the axis moving at the prescribed speed
            data = self.db.encode_message(f'Axis{axis_id}_Set_Input_Vel',
                                        {'Input_Vel': homing_speed, 'Input_Torque_FF': 0.0})
            
            self._send_message(axis_id=axis_id, command_name="set_input_vel", data=data, error_descriptor="Setting input vel")
            self.ROS_logger.info(f"Motor {axis_id} moving at {homing_speed:.2f} rev/s")

            # Stuff for an exponential average filter
            avg_weight = 0.7
            moving_avg = 0

            while True:
                if self.fatal_error:
                    # eg. overcurrent
                    self.ROS_logger.fatal("FATAL ISSUE! Stopping homing")
                    return False

                # Fetch any recent messages on the CAN bus
                # self.fetch_messages()
                
                # Get the current for this motor
                with self._motor_states_lock:
                    current = self.motor_states[axis_id].iq_measured

                # Magic logging line. Without this, current detection doesn't work immediately.
                # self.ROS_logger.info(f"Current reading for axis {axis_id}: {current}")

                if current is None:
                    self.ROS_logger.warn(f"Current reading for axis {axis_id} is None. Skipping...")
                    continue

                # Accumulate in exponential moving average
                moving_avg = moving_avg * avg_weight + current * (1 - avg_weight)

                # Check if the limit has been reached. If it has, put the axis into IDLE
                if abs(moving_avg) >= current_limit:
                    self.set_requested_state(axis_id, requested_state='IDLE')
                    return True
                
                time.sleep(0.005)
                
        except Exception as e:
            self.ROS_logger.error(f"Failed to run motor until current limit for axis {axis_id}: {e}")
            raise

    #########################################################################################################
    #                                            Managing ODrives                                           #
    #########################################################################################################
        
    def run_encoder_search(self, axis_to_run, attempt: int = 0) -> bool:
        """
        Runs the encoder index search procedure for the legs.
        This isn't necessary for the hand because we're using the on-board absolute encoder.
        If there's an error, it clears the errors and retries.
        If the search fails after one retry, it raises an exception.

        Args:
            axis_to_run: The axis ID to run the encoder search on.
            attempt (int, optional): The current attempt number.

        Raises:
            Exception: If encoder search fails after retries.
        """
        try:
            axisID = axis_to_run
            self.ROS_logger.info("Running encoder index search...")
            self.set_requested_state(axisID, requested_state='ENCODER_INDEX_SEARCH')

            # Check whether the leg being run has entered the ENCODER_INDEX_SEARCH state
            start_time = time.time()
            timeout_duration = 3.0 # seconds
            
            while True:
                elapsed = time.time() - start_time
                if elapsed > timeout_duration:
                    self.ROS_logger.error("Encoder index search failed. Timed out.")
                    return False

                with self._motor_states_lock:
                    current_state = self.motor_states[axisID].current_state
                    
                if current_state == 6:
                    break
                else:
                    self.ROS_logger.info(f"Axis {axisID} current state: {current_state}")

                time.sleep(0.05)

            self.ROS_logger.info(f"Axis {axisID} entered ENCODER_INDEX_SEARCH state. Waiting for success...")

            # Wait for the leg to present "Success" (0) in the procedure result
            start_time = time.time()
            timeout_duration = 5.0 # seconds

            while True:
                with self._motor_states_lock:
                    procedure_result = self.motor_states[axisID].procedure_result

                if procedure_result == 0:
                    # self.ROS_logger.info(f"Axis {axisID} encoder index search successful.")
                    break

                # Check for any fatal errors
                if self.fatal_error:
                    self.ROS_logger.error("Fatal error detected during encoder index search.")
                    return False

                # Check for timeout
                if time.time() - start_time > timeout_duration:
                    if attempt > 1:
                        self.ROS_logger.error("Encoder index search failed after retrying.")
                        return False
                    else:
                        self.ROS_logger.error("Encoder index search timed out. Clearing errors and retrying...")
                        self.clear_errors()
                        return self.run_encoder_search(axis_to_run, attempt=attempt+1)

                time.sleep(0.1)

            return True

        except Exception as e:
            self.ROS_logger.error(f"Encoder search failed: {e}")
            raise

    def set_absolute_vel_curr_limits(
    self,
    leg_current_limit: Optional[float] = 0.0,
    leg_vel_limit: Optional[float] = 0.0,
    hand_current_limit: Optional[float] = 0.0,
    hand_vel_limit: Optional[float] = 0.0
) -> None:
        """
        Sets the absolute velocity and current limits for the ODrives.

        Args:
            leg_current_limit (Optional[float]): The current limit for leg motors.
            leg_vel_limit (Optional[float]): The velocity limit for leg motors.
            hand_current_limit (Optional[float]): The current limit for the hand motor.
            hand_vel_limit (Optional[float]): The velocity limit for the hand motor.

        Raises:
            ValueError: If any provided limit is negative.
            Exception: If sending the message fails.
        """
        try:
            # Update limits if provided
            if leg_current_limit != 0.0:
                if leg_current_limit < 0:
                    raise ValueError("Leg current limit must be non-negative")
                self.leg_motor_abs_limits['current_limit'] = leg_current_limit
            if leg_vel_limit != 0.0:
                if leg_vel_limit < 0:
                    raise ValueError("Leg velocity limit must be non-negative")
                self.leg_motor_abs_limits['velocity_limit'] = leg_vel_limit
            if hand_current_limit != 0.0:
                if hand_current_limit < 0:
                    raise ValueError("Hand current limit must be non-negative")
                self.hand_motor_abs_limits['current_limit'] = hand_current_limit
            if hand_vel_limit != 0.0:
                if hand_vel_limit < 0:
                    raise ValueError("Hand velocity limit must be non-negative")
                self.hand_motor_abs_limits['velocity_limit'] = hand_vel_limit

            # Set limits for all axes
            for axis_id in range(self.num_axes):
                if axis_id < 6:
                    data = self.db.encode_message(
                        f'Axis{axis_id}_Set_Limits',
                        {
                            'Velocity_Limit': self.leg_motor_abs_limits['velocity_limit'],
                            'Current_Limit': self.leg_motor_abs_limits['current_limit']
                        }
                    )
                else:
                    data = self.db.encode_message(
                        f'Axis{axis_id}_Set_Limits',
                        {
                            'Velocity_Limit': self.hand_motor_abs_limits['velocity_limit'],
                            'Current_Limit': self.hand_motor_abs_limits['current_limit']
                        }
                    )
                self._send_message(
                    axis_id=axis_id,
                    command_name="set_vel_curr_limits",
                    data=data,
                    error_descriptor='current/velocity limits'
                )
                time.sleep(0.005)

            self.ROS_logger.info(
                f"Absolute limits set. Leg Vel: {self.leg_motor_abs_limits['velocity_limit']} rev/s, "
                f"Leg Curr: {self.leg_motor_abs_limits['current_limit']} A, "
                f"Hand Vel: {self.hand_motor_abs_limits['velocity_limit']} rev/s, "
                f"Hand Curr: {self.hand_motor_abs_limits['current_limit']} A"
            )
        except ValueError as ve:
            self.ROS_logger.error(f"Invalid limit value: {ve}")
            raise
        except Exception as e:
            self.ROS_logger.error(f"Failed to set absolute velocity and current limits: {e}")
            raise
        
    #########################################################################################################
    #                                       Fetching and handling messages                                  #
    #########################################################################################################

    def fetch_messages(self, max_messages=50):
        """
        Fetches messages from the CAN bus and handles them.
        Fetches up to max_messages messages at a time.
        If fewer messages are available, it will return after processing all of them.
        """
        # This method is designed to be called in a loop from the main script.
        # It checks for new messages on the CAN bus and processes them if there are any.
        with self._can_lock:
            try:
                for _ in range(max_messages):
                    # Perform a non-blocking read of the CAN bus
                    message = self.bus.recv(timeout=0)
                    if message is not None:
                        self.handle_message(message)
                    else:
                        break
            except Exception as e:
                self.ROS_logger.error(f"Error fetching messages: {e}")

    def handle_message(self, message):
        """
        Processes a single CAN message.

        Args:
            message: The CAN message to process.
        """
        # The arbitration_id contains encoded information about the axis and command.
        arbitration_id = message.arbitration_id

        # Check if the message is for the CAN traffic report
        if arbitration_id == self.linear_slider_CAN_ID:
            self._handle_linear_slider_msg(message)

        else: # If the message is from the ODrives
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
                self.ROS_logger.warning(
                    f"No handler for command ID {command_id} on axis {axis_id}. Arbitration ID: {arbitration_id}. Message data: {message.data}"
                    )

    def _handle_heartbeat(self, axis_id: int, data: bytes):
        """
        Handles heartbeat messages from ODrive axes.
        Publishes HeartbeatMsg for each heartbeat received.
        """
        try:
            # Extract information from the heartbeat message
            # axis_error = int.from_bytes(data[0:4], byteorder='little') # Not using this because we listen to the error messages separately
            axis_current_state = data[4]
            procedure_result = data[5]
            trajectory_done_flag = (data[6] >> 0) & 0x01  # Extract bit 0 from byte 6

            # If trajectory_done_flag is 1 or 0, convert to boolean. Otherwise, set to False and log a warning.
            if trajectory_done_flag == 1:
                trajectory_done_flag = True
            elif trajectory_done_flag == 0:
                trajectory_done_flag = False
            else:
                trajectory_done_flag = False
                self.ROS_logger.warning(f"Invalid trajectory_done_flag value: {trajectory_done_flag}")

            # Update the current state information for this axis
            with self._motor_states_lock:
                self.motor_states[axis_id].current_state = axis_current_state
                self.motor_states[axis_id].procedure_result = procedure_result
                self.motor_states[axis_id].trajectory_done = trajectory_done_flag

        except Exception as e:
            self.ROS_logger.error(f"Error handling heartbeat message for axis {axis_id}: {e}")

    def _handle_error(self, axis_id, message_data):
        """
        Handles error messages from an axis.
    
        Args:
            axis_id: The axis ID.
            message_data: The message data.
        """
        try:
            # Ignore any errors coming from ODrives that we aren't currently testing
            if axis_id != self.odrive_num_being_tested:
                return

            # First split the error data into its constituent parts
            active_errors = struct.unpack_from('<I', message_data, 0)[0]  # 4-byte integer
            disarm_reason = struct.unpack_from('<I', message_data, 4)[0]

            # Update the motor state with the error
            with self._motor_states_lock:
                self.motor_states[axis_id].active_errors = active_errors
                self.motor_states[axis_id].disarm_reason = disarm_reason

            # Dictionary mapping error codes to error messages
            ERRORS = {
                1: "INITIALIZING",
                2: "SYSTEM_LEVEL",
                4: "TIMING_ERROR",
                8: "MISSING_ESTIMATE",
                16: "BAD_CONFIG",
                32: "DRV_FAULT",
                64: "MISSING_INPUT",
                256: "DC_BUS_OVER_VOLTAGE",
                512: "DC_BUS_UNDER_VOLTAGE",
                1024: "DC_BUS_OVER_CURRENT",
                2048: "DC_BUS_OVER_REGEN_CURRENT",
                4096: "CURRENT_LIMIT_VIOLATION",
                8192: "MOTOR_OVER_TEMP",
                16384: "INVERTER_OVER_TEMP",
                32768: "VELOCITY_LIMIT_VIOLATION",
                65536: "POSITION_LIMIT_VIOLATION",
                16777216: "WATCHDOG_TIMER_EXPIRED",
                33554432: "ESTOP_REQUESTED",
                67108864: "SPINOUT_DETECTED",
                134217728: "BRAKE_RESISTOR_DISARMED",
                268435456: "THERMISTOR_DISCONNECTED",
                1073741824: "CALIBRATION_ERROR"
            }

            # Initialize conditions to check for various error states
            no_active_errors = all(motor.active_errors == 0 for motor in self.motor_states)
            no_disarm_reasons = all(motor.disarm_reason == 0 for motor in self.motor_states)
            any_disarmed = any(motor.disarm_reason != 0 for motor in self.motor_states)
            any_in_closed_loop_control = any(motor.current_state == 8 for motor in self.motor_states)

            # Log these conditions
            # self.ROS_logger.info(f"Axis: {axis_id}. Active errors: {active_errors}, Disarm reasons: {disarm_reason}, "
            #                         f"No active errors: {no_active_errors}, No disarm reasons: {no_disarm_reasons}, "
            #                         f"Any disarmed: {any_disarmed}, Any in closed loop control: {any_in_closed_loop_control}")
            
            # If there are no active errors and the disarm reason is 0 for all axes, there's nothing to report
            if no_active_errors and no_disarm_reasons:
                # Clear any existing error flags
                self.undervoltage_error = False
                self.fatal_error = False
                return

            # If there are any axes disarmed but no axes in CLOSED_LOOP_CONTROL, try clearing the errors
            if any_disarmed and not any_in_closed_loop_control:
                # Check that we haven't already tried clearing the errors
                if self.reset_attempts_after_soft_error < self.max_reset_attempts_after_soft_error:
                    try:
                        self.clear_errors()
                        # Wait a bit
                        time.sleep(0.5)
                        self.ROS_logger.info("Cleared errors due to disarmed axes.")
                        self.reset_attempts_after_soft_error += 1
                    except Exception as e:
                        self.ROS_logger.error(f"Failed to clear errors: {e}")

                else:
                    self.ROS_logger.error("Exceeded the maximum number of reset attempts after soft error.", throttle_duration_sec=1.0)
                    self.fatal_error = True
                    self._update_state_with_disarm_errors()

            # If there are any axes disarmed AND any axes are in CLOSED_LOOP_CONTROL mode (state 8), then report an error
            if any_disarmed and any_in_closed_loop_control:
                self.fatal_error = True
                self._update_state_with_disarm_errors()

                # Log the error
                self.ROS_logger.error(f"One or more axes are disarmed while in CLOSED_LOOP_CONTROL mode!", throttle_duration_sec=1.0)
            
            # If there are any active errors, set the flag
            if not no_active_errors:
                self.fatal_error = True

            # If the error is UNDERVOLTAGE, set the flag
            if active_errors & 512:
                self.undervoltage_error = True

            # If the disarm reason is DC_BUS_UNDER_VOLTAGE and there are no active errors, clear the flags and errors
            if disarm_reason & 512 and no_active_errors:
                self.undervoltage_error = False
                self.fatal_error = False
                try:
                    self.clear_errors()
                except Exception as e:
                    self.ROS_logger.error(f"Failed to clear undervoltage error: {e}")
                self.ROS_logger.info("Undervoltage disarm reason cleared as there are no active errors.")

            ###########################################################################
            #           Log all errors with per-axis per-error throttling             #
            ###########################################################################
            
            # Initialize per-axis log times if they don't exist
            if axis_id not in self._last_error_log_times:
                self._last_error_log_times[axis_id] = {}

            current_time = time.time()

            for error_code, error_message in ERRORS.items():
                if active_errors & error_code:
                    last_log_time = self._last_error_log_times[axis_id].get(error_code, 0)
                    if current_time - last_log_time > self.error_log_throttle_duration_sec:
                        self.ROS_logger.error(f"Active error on axis {axis_id}: {error_message}")
                        self._last_error_log_times[axis_id][error_code] = current_time

                elif disarm_reason & error_code:
                    last_log_time = self._last_error_log_times[axis_id].get(error_code, 0)
                    if current_time - last_log_time > self.error_log_throttle_duration_sec:
                        self.ROS_logger.error(f"Disarm reason on axis {axis_id}: {error_message}")
                        self._last_error_log_times[axis_id][error_code] = current_time

        except Exception as e:
            self.ROS_logger.error(f"Error handling error message for axis {axis_id}: {e}")

    def _handle_encoder_estimates(self, axis_id, data):
        """
        Handles encoder estimate messages.

        Args:
            axis_id: The axis ID.
            data: The message data.
        """
        try:
            # Start by unpacking the data, which is two 32-bit floats
            pos_estimate, vel_estimate = struct.unpack_from('<ff', data)

            if axis_id < 6:
                # Invert the position and velocity estimates for the legs since we want +ve to be upwards
                pos_estimate = -pos_estimate
                vel_estimate = -vel_estimate

            # Update the motor state with the encoder estimates. 
            with self._motor_states_lock:
                self.motor_states[axis_id].pos_estimate = pos_estimate
                self.motor_states[axis_id].vel_estimate = vel_estimate
            
        except Exception as e:
            self.ROS_logger.error(f"Failed to handle encoder estimates for axis {axis_id}: {e}")

    def _handle_iq_readings(self, axis_id, data):
        """
        Handles IQ readings messages.

        Args:
            axis_id: The axis ID.
            data: The message data.
        """
        try:
            # Start by unpacking the data, which is two 32-bit floats
            iq_setpoint, iq_measured = struct.unpack_from('<ff', data)

            # Update the motor state with the IQ readings
            with self._motor_states_lock:
                self.motor_states[axis_id].iq_setpoint = iq_setpoint
                self.motor_states[axis_id].iq_measured = iq_measured

        except Exception as e:
            self.ROS_logger.error(f"Failed to handle IQ readings for axis {axis_id}: {e}")

    def _handle_temp_readings(self, axis_id, data):
        """
        Handles temperature readings messages.

        Args:
            axis_id: The axis ID.
            data: The message data.
        """
        try:
            # Start by unpacking the data, which is two 32-bit floats
            fet_temp, motor_temp = struct.unpack_from('<ff', data)

            # Update the motor state with the temperature readings
            with self._motor_states_lock:
                self.motor_states[axis_id].fet_temp = fet_temp
                self.motor_states[axis_id].motor_temp = motor_temp

        except Exception as e:
            self.ROS_logger.error(f"Failed to handle temperature readings for axis {axis_id}: {e}")

    def _handle_bus_voltage_current_readings(self, axis_id, data):
        """
        Handles bus voltage and current readings messages.

        Args:
            axis_id: The axis ID.
            data: The message data.
        """
        try:
            # Start by unpacking the data, which is two 32-bit floats
            bus_voltage, bus_current = struct.unpack_from('<ff', data)

            # Update the motor state with the bus voltage and current readings
            with self._motor_states_lock:
                self.motor_states[axis_id].bus_voltage = bus_voltage
                self.motor_states[axis_id].bus_current = bus_current

        except Exception as e:
            self.ROS_logger.error(f"Failed to handle bus voltage and current readings for axis {axis_id}: {e}")

    def _handle_arbitrary_parameter_read(self, axis_id, data, param_type='f'):
        """
        Handles arbitrary parameter read messages.
        Currently assumes that the parameter is a 32-bit float.

        Args:
            axis_id: The axis ID.
            data: The message data.
        """
        # self.ROS_logger.info(f"Handling arbitrary parameter read for axis {axis_id}")
        try:
            # Start by unpacking the data, which is a 32-bit float
            _, endpoint_id, _, parameter_value = struct.unpack_from('<BHB' + param_type, data)

            # If the endpoint ID matches that for commutation_mapper.pos_abs, update self._received_encoder_search_feedback_on_axes
            if endpoint_id == self.ARBITRARY_PARAMETER_IDS['commutation_mapper.pos_abs']:
                # Record True if the parameter_value is a number, False if it's NaN
                if not math.isnan(parameter_value):
                    self._received_encoder_search_feedback_on_axes[axis_id] = True
                else:
                    self._received_encoder_search_feedback_on_axes[axis_id] = False
            else:
                # Log the parameter value
                self.ROS_logger.info(f"Unknown arbitrary parameter read on axis {axis_id}: {parameter_value} with endpoint ID {endpoint_id}")

        except Exception as e:
            self.ROS_logger.error(f"Failed to handle arbitrary parameter read for axis {axis_id}: {e}")    

    def _handle_linear_slider_msg(self, msg):
        """
        Handles position feedback from the linear slider.

        Assumes msg.data is a byte array of length 4 containing a little-endian 
        32-bit float representing the position in mm.

        Args:
            message: The CAN message.
        """
        try:
            if len(msg.data) != 4:
                print("Unexpected message length:", len(msg.data))
                return

            # Unpack the 4 bytes into a float (little-endian format)
            new_position, = struct.unpack('<f', msg.data)
            
            # Update the instance attribute
            with self._linear_slider_lock:
                self.linear_slider_position = new_position
        
        except Exception as e:
            self.ROS_logger.error(f"Failed to handle CAN traffic report: {e}")

    #########################################################################################################
    #                                       Utility functions                                               #
    #########################################################################################################

    def get_motor_states(self) -> List[MotorStateSingle]:
        """
        Returns the motor states for all axes and copies the current states to the last_motor_states attribute
        to avoid race conditions.

        Returns:
            A list of MotorStateSingle objects.
        """
        with self._motor_states_lock:
            self.last_motor_states = self.motor_states.copy()
        
        return self.last_motor_states

    def get_linear_slider_position(self) -> float:
        """
        Returns the current position of the linear slider in mm.
        """
        with self._linear_slider_lock:
            return self.linear_slider_position

    def shutdown(self):
        """
        Shuts down the interface, ensuring all resources are cleaned up.
        """
        try:
            self.close()
            self.ROS_logger.info("CANInterface shutdown completed.")
        except Exception as e:
            self.ROS_logger.error(f"Error during shutdown: {e}")