"""
CANHandler Class
----------------

This class is designed to interface with a CAN bus using the python-can library OVER USB. This version has been adapted from the
ROS2 version of the class.

It's specifically tailored for handling communications for a robot powered by BLDC motors, controlled by ODrive Pro controllers. 
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
import can
import cantools
import struct
import time
# from .ruckig_trajectory_generator import RuckigTrajectoryGenerator
from ruckig import InputParameter, OutputParameter, Result, Ruckig
from copy import copy

class CANHandler:
    # Create a dictionary of commands that might be called over the CAN bus by external scripts
    COMMANDS = {
        "heartbeat_message"     : 0x001,
        "get_error"             : 0x003,
        "set_requested_state"   : 0x007,
        "get_encoder_estimate"  : 0x009,
        "set_controller_mode"   : 0x00b,
        "set_input_pos"         : 0x00c,
        "set_input_vel"         : 0x00d,
        "set_vel_curr_limits"   : 0x00f,
        "set_traj_vel_limit"    : 0x011,
        "set_traj_acc_limits"   : 0x012,
        "get_iq"                : 0x014, 
        "reboot_odrives"        : 0x016,
        "clear_errors"          : 0x018,
        "set_absolute_position" : 0x019,
        "set_pos_gain"          : 0x01a,
        "set_vel_gains"         : 0x01b,
    }

    # Create the reverse dictionary to go from ID to name; mostly for logging human-readable errors
    COMMAND_ID_TO_NAME = {v: k for k, v in COMMANDS.items()}

    # Set absolute limit for how far the leg motors can turn. This is intended as a backup if prior error-checking fails
    _LEG_MOTOR_MAX_POSITION = 4.2  # Revs

    def __init__(self, bus_name='can0', bitrate=1000000, bus_type='socketcan'):
       
        # Construct the full filepath to the dbc file
        dbc_file_path = 'ODrive_Pro.dbc'

        # Now import the ODrive dbc file
        self.db = cantools.database.load_file(dbc_file_path)

        # Initialize the parameters for the CAN bus that will be used by setup_can_bus to initialise the bus itself
        self._can_bus_name = bus_name
        self._can_bitrate = bitrate
        self._can_bus_type = bus_type
        self.bus = None

        # Create a dictionary to map command IDs to their corresponding handler
        # This allows for easy addition of new commands and corresponding handlers
        self.command_handlers = {
            self.COMMANDS["heartbeat_message"]   : self._handle_heartbeat,
            self.COMMANDS["get_error"]           : self._handle_error,
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
        self.num_axes = 7  # 6 for the legs, 1 for the hand.

        # Initialize buffers for all variables that are being sent out to the ROS network
        self.position_buffer = [None] * self.num_axes
        self.velocity_buffer = [None] * self.num_axes
        self.iq_meas_buffer  = [None] * self.num_axes
        self.iq_set_buffer   = [None] * self.num_axes

        # Initialize a local list for these variables
        self.pos_values     = [None] * self.num_axes
        self.vel_values     = [None] * self.num_axes
        self.iq_meas_values = [None] * self.num_axes
        self.iq_set_values  = [None] * self.num_axes

        # Initialize a flag that triggers whenever any fatal errors are detected
        self.fatal_issue = False

        # Initialize a variable to track when the last heartbeat message came in. Determines whether the robot is responding
        self.last_heartbeat_time = None
        self.fatal_can_issue = False  # Has the bus completely failed and can't be restored?

        # Initialize the current state for each axis
        self.axis_states = [None] * self.num_axes
        self.is_done_moving = [None] * self.num_axes
        self.procedure_result = [None] * self.num_axes


        # Properties for the hand:

        # Initialize the default gains for the hand motor
        self.default_hand_gains = {'pos_gain': 18.0, 'vel_gain': 0.008, 'vel_integrator_gain': 0.01}
        self.catch_gains = {'pos_gain': 0.0, 'vel_gain': 0.0, 'vel_integrator_gain': 0.0}
        self.throw_gains = {'pos_gain': 35.0, 'vel_gain': 0.007, 'vel_integrator_gain': 0.07} # From Jon
        
        self.current_hand_gains = self.default_hand_gains
        self.hand_stroke = 8.82 # Revs. This is the full stroke of the hand motor and SHOULD BE UPDATED IF THE HAND IS CHANGED

        # self.hand_trajectory_generator = RuckigTrajectoryGenerator(dof=1, cycle_time=0.01)


        # Set default constraints for the hand trajectory generator
        # self.hand_trajectory_generator.set_constraints(max_velocity=5.0, max_acceleration=10.0, max_jerk=50.0)

        # Finally, set up the CAN bus and establish communication with the robot
        self.setup_can_bus()
        self.wait_for_heartbeat()
        # self.run_encoder_search() # TEMPORARY. Don't do this while testing the hand

    #########################################################################################################
    #                                            CAN Bus Management                                         #
    #########################################################################################################

    def setup_can_bus(self):
        self.bus = can.Bus(channel=self._can_bus_name, bustype=self._can_bus_type, bitrate=self._can_bitrate)
        self.flush_bus() # Flush the bus now that it's been set up

    def wait_for_heartbeat(self):
        # Wait for the heartbeat message to verify that the bus is operational
        print('Waiting for heartbeat from Jugglebot...')
        time_to_wait = 3  # Seconds

        start_time = time.time()

        while time.time() - start_time < time_to_wait:
            self.fetch_messages()

            if self.last_heartbeat_time and time.time() - self.last_heartbeat_time < 1.0:
                # If a heartbeat message has been received in the last second
                print('Heartbeat received!')
                return

            time.sleep(0.1)

        # If no heartbeat was detected, attempt to restore the connection
        print('No heartbeat detected.')
        self.attempt_to_restore_can_connection()

    def close(self):
        if self.bus is not None:
            try:
                self.bus.shutdown()
                print('CAN bus closed')
            except Exception as e:
                print(f'Error when closing CAN bus: {e}')

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

        print('Attempting to restore CAN connection...')

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
                        print('No heartbeat message has been received yet. Is Jugglebot on?')

                    elif time.time() - self.last_heartbeat_time < reconnect_timeout:
                        print(f"Reading from CAN bus successful on attempt {attempt + 1}")
                        rx_working = 1
                        break
                    
                # Now check that sending messages is working by sending a dummy value
                msg = can.Message(arbitration_id=0x999, dlc=8, is_extended_id=False, data=[0, 0, 0, 0, 0, 0, 0, 0])
                try:
                    self.bus.send(msg)
                    print(f"Writing to CAN bus successful on attempt {attempt + 1}")
                    tx_working = 1
                except Exception as e:
                    print(f"Failed to write to CAN bus on attempt {attempt + 1}: {e}")

                # If both reading and writing are working, we're done
                if rx_working and tx_working:
                    print('CAN bus connection restored!')
                    self.fatal_can_issue = False
                    return
                
            except Exception as e:
                print(f"Failed to reconnect CAN bus on attempt {attempt + 1}: {e}")
            
            time.sleep(retry_delay)  # Wait before retrying

        print(f"Failed to restore CAN bus connection after {max_retries} attempts. Closing CAN bus...")
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

    def send_arbitrary_message(self, axis_id, msg_name, setpoint, rtr_bit=False):
        ''' Send an arbitrary message over the CAN bus'''
        # Construct a dictionary to map the message name to the corresponding command ID
        arbitrary_message_ids = {
            'set_input_pos': 383,
            'set_input_vel': 384,
            'set_input_torque': 385,
        }

        arbitration_id = (axis_id << 5) | 0x04 # 0x04: RxSdo, to send arbitrary parameters

        message_id = arbitrary_message_ids[msg_name]

        # Construct the data to be sent
        data = struct.pack('<BHB' + 'f', 0x01, message_id, 0, -setpoint) # Data are all floats ('f') being written (0x01)

        # Create the CAN message
        msg = can.Message(arbitration_id=arbitration_id, dlc=8, is_extended_id=False, data=data, is_remote_frame=rtr_bit)

        try:
            self.bus.send(msg)
            # print(f"Arbitrary message sent for axis {axis_id} with message {msg_name} and setpoint {setpoint:.2f}")
        except Exception as e:
            print(f"Arbitrary message for axis {axis_id} NOT sent! Error: {e}")

    def _send_message(self, axis_id, command_name, data=None, error_descriptor='Not described', rtr_bit=False):
        if not self.bus:
            # If the bus hasn't been initialized, return
            return
        
        # If there is a fatal issue with any ODrive, don't send any more messages
        if self.fatal_issue:
            print("Fatal issue detected. Stopping further messages.")
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
            # print(f"msg: {msg} for axis {axis_id} with command {command_name}")
        except Exception as e:
            # Log that the message couldn't be sent
            print(f"CAN message for {error_descriptor} NOT sent to axisID {axis_id}! Error: {e}")

            # If the buffer is full, the CAN bus is probably having issue. Try to re-establish it
            if "105" in str(e):
                # "No buffer space available" is error code 105
                self.attempt_to_restore_can_connection()

    def send_position_target(self, axis_id, setpoint, max_position=_LEG_MOTOR_MAX_POSITION, min_position=0.0):
        # Commands the given motor to move to the designated setpoint (after checking its magnitude)

        # Log the setpoint
        # print(f"Setting position target for motor {axis_id} to {setpoint:.2f}")

        if axis_id < 6 and setpoint > max_position or setpoint < min_position:
            print(f'Setpoint of {setpoint:.2f} is outside allowable bounds and has been clipped')
            setpoint = max(min_position, min(setpoint, max_position))

        # if axis_id != 6:
        setpoint = -1 * setpoint  # Since -ve is extension for the legs. Same for hand CURRENTLY (TEMPORARY)

        data = self.db.encode_message(f'Axis{axis_id}_Set_Input_Pos', 
                                {'Input_Pos': setpoint, 'Vel_FF': 0.0, 'Torque_FF': 0.0})
        
        self._send_message(axis_id=axis_id, command_name="set_input_pos", data=data, error_descriptor="position target")
        print(f"Motor {axis_id} moving to {setpoint:.2f} revs")

    def _set_control_mode(self, axis_id, control_mode, input_mode):
        ''' Set the control and input modes for the chosen axis_id. 

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
        print('Clearing ODrive errors...')
        command_name = "clear_errors"

        # Clear errors for all axes
        for axisID in range(6):
            self._send_message(axis_id=axisID, command_name=command_name, error_descriptor="Clearing errors")

        # Reset fatal issue flag
        self.fatal_issue = False

    def reboot_odrives(self):
        print("Rebooting ODrives...")
        command_name = "reboot_odrives"
        # Reboots all ODrives connected on the bus
        for axisID in range(6):
            self._send_message(axis_id=axisID, command_name=command_name, error_descriptor="Rebooting ODrives")

        time.sleep(10)  # To give the ODrives plenty of time to reboot
        print("ODrives rebooted!")
        
    def home_robot(self):
        # Start by resetting the motor limits
        self.setup_odrives()

        # Runs all the motors until the physical limit reached (current spikes)
        print("Homing motors...")

        leg_homing_speed = 1.0 # Go real slow
        leg_current_limit = 5.0  # Found experimentally

        hand_direction = -1 # +1 is upwards +ve, -1 is downwards +ve
        hand_homing_speed = hand_direction * 3.0 # Higher kV motor, so can go faster (Negative is currently upwards)
        hand_current_limit = 5.0  # Found experimentally

        current_limit_headroom = 3.0  # Headroom for limit of how high the current can go above "current_limit"

        for axisID in range(self.num_axes):
            if axisID != 6:
                continue # Only homing the hand for now
                # Run the axis until an end-stop is hit
                self.run_motor_until_current_limit(axis_id=axisID, homing_speed=leg_homing_speed, current_limit=leg_current_limit,
                                                  current_limit_headroom=current_limit_headroom)
                # Set the encoder position to be a little more than 0 (so that 0 is a bit off the end-stop. +ve is contraction)
                data = self.db.encode_message(f'Axis{axisID}_Set_Absolute_Position',
                                            {'Position': 0.1})  # Unit is revolutions
                
                self._send_message(axis_id=axisID, command_name="set_absolute_position", data=data, error_descriptor="Set encoder pos in homing")
                print(f"Motor {axisID} homed!")
            
            elif axisID == 6: # ie the Hand
                # Ensure the hand gains are set to the correct values
                self.set_hand_gains(**self.default_hand_gains)

                # Run the hand upwards until the current limit is reached
                self.run_motor_until_current_limit(axis_id=axisID, homing_speed=hand_homing_speed, current_limit=hand_current_limit,
                                                    current_limit_headroom=current_limit_headroom)

                # Store this position
                upper_endstop = self.pos_values[axisID]

                time.sleep(0.5)
                
                # Run the hand downwards until the current limit is reached
                self.run_motor_until_current_limit(axis_id=axisID, homing_speed= -1 * hand_homing_speed, current_limit=hand_current_limit,
                                                    current_limit_headroom=current_limit_headroom)
                # Store this position
                lower_endstop = self.pos_values[axisID]

                # Set the encoder position to be a little more than 0 (so that 0 is a bit off the end-stop)
                data = self.db.encode_message(f'Axis{axisID}_Set_Absolute_Position',
                                                {'Position': hand_direction * -0.1})  # Unit is revolutions
                
                self._send_message(axis_id=axisID, command_name="set_absolute_position", data=data, error_descriptor="Set encoder pos in homing")

                # Store the hand_stroke for later use
                self.hand_stroke = abs(upper_endstop - lower_endstop) # Unit is revolutions

                print(f"Hand homed! Hand stroke is {self.hand_stroke:.2f} revs")

        # Now that all motors have been homed, reset them to have desired properties
        self.setup_odrives() 

    def run_motor_until_current_limit(self, axis_id, homing_speed, current_limit, current_limit_headroom):
        ''' Run the motor until the current limit is reached. This is used in all homing sequences '''
        # Ensure the axis is in CLOSED_LOOP_CONTROL mode
        self._set_requested_state(axis_id, requested_state='CLOSED_LOOP_CONTROL')
        
        # Set the hand to velocity control mode
        self._set_control_mode(axis_id=axis_id, control_mode='VELOCITY_CONTROL', input_mode='VEL_RAMP')
        
        # Set absolute current/velocity limit. Don't use the 'set_absolute_vel_curr_limits' method as it sets all axes simultaneously
        data = self.db.encode_message(f'Axis{axis_id}_Set_Limits',{'Current_Limit':current_limit + current_limit_headroom,
                                                                 'Velocity_Limit':abs(homing_speed*2)})
        
        self._send_message(axis_id=axis_id, command_name="set_vel_curr_limits", data=data)

        # Start the axis moving at the prescribed speed
        data = self.db.encode_message(f'Axis{axis_id}_Set_Input_Vel',
                                    {'Input_Vel': homing_speed, 'Input_Torque_FF': 0.0})
        
        self._send_message(axis_id=axis_id, command_name="set_input_vel", data=data, error_descriptor="Setting input vel")
        print(f"Motor {axis_id} moving at {homing_speed:.2f} rev/s")

        # Stuff for an exponential average filter
        avg_weight = 0.7
        moving_avg = 0

        while True:
            if self.fatal_issue:
                # eg. overcurrent
                print("FATAL ISSUE! Stopping homing")
                break

            # Fetch any recent messages on the CAN bus
            self.fetch_messages()
            
            # Get the current for this motor
            current = self.iq_meas_values[axis_id]

            # Accumulate in exponential moving average
            moving_avg = moving_avg * avg_weight + current * (1 - avg_weight)

            # Check if the limit has been reached. If it has, put the axis into IDLE
            if abs(moving_avg) >= current_limit:
                self._set_requested_state(axis_id, requested_state='IDLE')
                return

    def run_encoder_search(self):
        '''
        Runs the necessary encoder search procedure to find the index of the encoder. This was done automatically
        on power up for the ODrive v3.6's but now (AFAIK) must be done manually. Note that this doesn't need to be run
        for the hand motor, since we're using the on-board absolute encoder.
        '''

        for axisID in range(self.num_axes):
            if axisID == 6: # Don't need to do this for the hand, since we're using the on-board encoder
                self.procedure_result[axisID] = 0  # Set the procedure result to 0 so that the while loop doesn't get stuck
            else:
                self._set_requested_state(axisID, requested_state='ENCODER_INDEX_SEARCH')

        # Wait for all axes to present "Success" (0) in the procedure result
        start_time = time.time()
        timeout_duration = 3.0 # seconds

        while not all([result == 0 for result in self.procedure_result]):
            self.fetch_messages()
            time.sleep(0.1)
            if time.time() - start_time > timeout_duration:
                print("Timeout: Encoder index search still ongoing.")

        print("Encoder index search complete! Jugglebot is ready to be homed.")

    #########################################################################################################
    #                                              Hand Commands                                            #
    #########################################################################################################

    def prepare_for_catch(self):
        ''' Prepare the hand for catching a ball by moving the hand to the top of its stroke and putting the
        hand motor into the correct control/input mode, with correct gains'''

        # Set the hand to position control mode
        self._set_control_mode(axis_id=6, control_mode='POSITION_CONTROL', input_mode='POS_FILTER')

        # Move the hand to the top of its stroke
        setpoint = self.hand_stroke - 0.5
        self.send_position_target(axis_id=6, setpoint=setpoint, max_position=self.hand_stroke)

        # Wait until the hand has reached the top of its stroke
        while abs(self.pos_values[6] - setpoint) > 0.01 * setpoint:  # Exit loop when pos value is within 1% of setpoint
            self.fetch_messages()
            time.sleep(0.1)

        # Set the gains for the hand
        self.set_hand_gains(**self.catch_gains)

        time.sleep(0.5)

        # Now wait for the catch
        self.wait_for_catch()

    def wait_for_catch(self):
        ''' Wait for the hand to catch the ball, then once a catch has been detected, move the hand to the bottom of its stroke,
        with steadily increasing gains to prepare for the throw'''
        
        setpoint = 1.0
        self.send_position_target(axis_id=6, setpoint=setpoint)

        # Wait for the hand motor vel_estimate to spike, indicating a catch
        vel_threshold = -0.1  # rev/s. Recall -ve is downwards

        while self.vel_values[6] > vel_threshold:
            self.fetch_messages()
            time.sleep(0.1)

        print('Catch detected!')

        # Now that the catch has been detected, move the hand to near the bottom of its stroke while steadily ramping the gains to their throw values

        # # Ramp the gains to the throw values
        # self.ramp_hand_gains(**self.throw_gains)

    def set_hand_gains(self, pos_gain, vel_gain, vel_integrator_gain):
        ''' Set the gains for the hand motor. These change throughout the throw/catch cycle'''
        
        # First set the position gain
        data = self.db.encode_message('Axis6_Set_Pos_Gain', {'Pos_Gain': pos_gain})
        self._send_message(axis_id=6, command_name='set_pos_gain', data=data, error_descriptor='Setting hand gains')

        # Now set the velocity and integrator gains
        data = self.db.encode_message('Axis6_Set_Vel_Gains', {'Vel_Gain': vel_gain, 'Vel_Integrator_Gain': vel_integrator_gain})
        self._send_message(axis_id=6, command_name='set_vel_gains', data=data, error_descriptor='Setting hand gains')

        # Store the current gains so that we always know what they are
        self.current_hand_gains = {'pos_gain': pos_gain, 'vel_gain': vel_gain, 'vel_integrator_gain': vel_integrator_gain}
    
    def ramp_hand_gains(self, pos_gain, vel_gain, vel_integrator_gain, ramp_duration=1.0, num_steps=10):
        ''' Ramp the hand gains from their current values to the desired values over the specified duration'''

        # Fetch the initial gains
        init_gains = self.current_hand_gains

        # Calculate the step size for each gain
        pos_step = (pos_gain - init_gains['pos_gain']) / num_steps
        vel_step = (vel_gain - init_gains['vel_gain']) / num_steps
        vel_integrator_step = (vel_integrator_gain - init_gains['vel_integrator_gain']) / num_steps

        # Calculate the time to wait between each step
        wait_time = ramp_duration / num_steps

        # Loop through the steps, updating the gains each time
        for step in range(num_steps):
            new_pos_gain = init_gains['pos_gain'] + pos_step * (step + 1)
            new_vel_gain = init_gains['vel_gain'] + vel_step * (step + 1)
            new_vel_integrator_gain = init_gains['vel_integrator_gain'] + vel_integrator_step * (step + 1)

            # print(f'Ramping hand gains: Pos Gain: {new_pos_gain}, Vel Gain: {new_vel_gain}, Vel Integrator Gain: {new_vel_integrator_gain}')

            self.set_hand_gains(pos_gain=new_pos_gain, vel_gain=new_vel_gain, vel_integrator_gain=new_vel_integrator_gain)
            time.sleep(wait_time)

    def throw_ball(self):
        ''' Throw the ball by moving the hand to near the bottom of its stroke, then launch the ball using
        the Ruckig trajectory'''

        print('Preparing to throw ball')

        # Ensure the gains are set to the default move value before being sent to the bottom position
        self.set_hand_gains(**self.default_hand_gains)

        # Ensure the hand motor is in CLOSED_LOOP_CONTROL mode with POS_FILTER input mode
        self._set_requested_state(axis_id=6, requested_state='CLOSED_LOOP_CONTROL')
        self._set_control_mode(axis_id=6, control_mode='POSITION_CONTROL', input_mode='PASSTHROUGH')

        # Now move to near the bottom of the stroke
        self.send_position_target(axis_id=6, setpoint=1.0)

        # Wait for the hand to reach the bottom of its stroke
        # while abs(self.pos_values[6] - 0.1) > 0.01 * -0.1:  # Exit loop when pos value is within 1% of setpoint. -setpoint since upwards is -ve.
        #     self.fetch_messages()
        #     time.sleep(0.1)

        time.sleep(2.0)

        print('Hand at bottom of stroke. Preparing to throw...')

        ''' Now that the hand is at the bottom of its stroke, we're ready to implement the throw trajectory'''
        # Stary by setting the throw gains
        self.set_hand_gains(**self.throw_gains)

        # Set the throw target position to be near the upper end-stop
        throw_target_pos = self.hand_stroke - 3.0 # Be conservative for now
        
        otg = Ruckig(1, 0.01) # DoFs, control cycle time
        inp = InputParameter(1)
        out = OutputParameter(1)

        # Set input parameters
        inp.current_position = [self.pos_values[6]]
        inp.current_velocity = [self.vel_values[6]]
        inp.current_acceleration = [0.0]

        inp.target_position = [throw_target_pos]
        inp.target_velocity = [0.0]
        inp.target_acceleration = [0.0]

        inp.max_velocity = [5.0]
        inp.max_acceleration = [10.0]
        inp.max_jerk = [50.0]

        inp.min_velocity = [-10.0]
        inp.min_acceleration = [-15.0]

        print(f'Generating hand trajectory move hand to {throw_target_pos:.2f} revs')

        first_output, out_list = None, []
        res = Result.Working

        while res == Result.Working:
            res = otg.update(inp, out)
            out_list.append(copy(out))

            # Send the output to the hand motor
            print(f'Sending hand to {out.new_position[0]:.2f} revs')
            self.send_position_target(axis_id=6, setpoint=out.new_position[0])

            out.pass_to_input(inp)

            if not first_output:
                first_output = copy(out)

        print(f'Calculation duration: {first_output.calculation_duration:0.1f} [us]')
        print(f'Trajectory duration: {first_output.trajectory.duration:0.3f} [s]')


        # # Start by setting the trajectory constraints. We can have higher minimum vel/accel due to the ball no longer being in the hand
        # self.hand_trajectory_generator.set_constraints(max_velocity=5.0, max_acceleration=10.0, max_jerk=50.0,
        #                                                min_velocity=-10.0, min_acceleration=-15.0) # Go slow for now

        # # Put the hand into PASSTHROUGH position control
        # self._set_control_mode(axis_id=6, control_mode='POSITION_CONTROL', input_mode='PASSTHROUGH')

        # # Now generate the trajectory
        # print(f'Generating hand trajectory move hand to {throw_target_pos:.2f} revs')
        # trajectory_done = self.hand_trajectory_generator.generate_trajectory(target_position=throw_target_pos, target_velocity=0.0)

        # print(f'Trajectory generated: {trajectory_done}')


    def hand_control_loop(self):
        ''' Update the hand control loop to move the hand to the next point in the trajectory generator'''

        # Fetch the current hand position
        current_pos = self.pos_values[6]
        current_vel = self.vel_values[6]

        # Update the current state of the trajectory generator
        # self.hand_trajectory_generator.update_current_state(position=current_pos, velocity=current_vel)

        # Retrieve the next trajectory point
        # next_pos, next_vel, _ = self.hand_trajectory_generator.get_trajectory()  # Not doing anything with accel

        # print(f'Next pos: {next_pos:.2f}, Next vel: {next_vel:.2f}')

        # # Send the next position target to the hand motor
        # self.send_position_target(axis_id=6, setpoint=next_pos)

    #########################################################################################################
    #                                            Managing ODrives                                           #
    #########################################################################################################

    def setup_odrives(self):
        # Call each of the "set" methods with their default values
        print('Setting up Leg ODrives with default values...')

        self.set_absolute_vel_curr_limits()
        self.set_legs_control_and_input_mode()
        self.set_trap_traj_vel_acc_limits()
        self.set_leg_odrive_state()

    def set_absolute_vel_curr_limits(self, leg_current_limit=20.0, leg_vel_limit=50.0, hand_current_limit=50.0, hand_vel_limit=1000.0):
        # Set the absolute velocity and current limits. If these values are exceeded, the ODrive will throw an error
        for axis_id in range(self.num_axes):
            if axis_id < 6:
                data = self.db.encode_message(f'Axis{axis_id}_Set_Limits', 
                                            {'Velocity_Limit':leg_vel_limit, 'Current_Limit': leg_current_limit})
                    
                self._send_message(axis_id=axis_id, command_name="set_vel_curr_limits", data=data, error_descriptor='Current/Vel Limits')
                time.sleep(0.005)

            if axis_id == 6:
                data = self.db.encode_message(f'Axis{axis_id}_Set_Limits', 
                                            {'Velocity_Limit':hand_vel_limit, 'Current_Limit': hand_current_limit})
                    
                self._send_message(axis_id=axis_id, command_name="set_vel_curr_limits", data=data, error_descriptor='Current/Vel Limits')
                time.sleep(0.005)
        

        print(f'Absolute limits set. Leg vel lim: {leg_vel_limit} rev/s, Leg current lim: {leg_current_limit} A.'
                             f' Hand vel lim: {hand_vel_limit} rev/s, Hand current lim: {hand_current_limit} A')
        
    def set_legs_control_and_input_mode(self, control_mode='POSITION_CONTROL', input_mode='TRAP_TRAJ'):
        for axis_id in range(6):
            self._set_control_mode(axis_id=axis_id, control_mode=control_mode, input_mode=input_mode)
            time.sleep(0.005)

        print(f'Legs control mode set to: {control_mode}, input mode set to: {input_mode}')

    def set_trap_traj_vel_acc_limits(self, velocity_limit=10.0, acceleration_limit=10.0):
        # Set the velocity and acceleration limits while in trap_traj control mode
        for axis_id in range(6):
            self._set_trap_traj_vel_acc(axis_id=axis_id, vel_limit=velocity_limit, acc_limit=acceleration_limit)
            time.sleep(0.005)

        print(f'Legs trap traj limits set. Vel: {velocity_limit}, Acc: {acceleration_limit}')

    def set_leg_odrive_state(self, requested_state='CLOSED_LOOP_CONTROL'):
        # Set the state of the ODrives. Options are "IDLE" or "CLOSED_LOOP_CONTROL" (there are others, but Jugglebot doesn't use them)
        for axis_id in range(6):
            self._set_requested_state(axis_id=axis_id, requested_state=requested_state)    
            time.sleep(0.005)

        print(f'Leg ODrives state changed to {requested_state}')
    
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
            print(f"No handler for command ID {command_id} on axis {axis_id}. Arbitration ID: {arbitration_id}")

    def _handle_heartbeat(self, axis_id, data):
        # Extract information from the heartbeat message
        axis_error = data[0]
        axis_current_state = data[4]
        procedure_result = data[5]
        trajectory_done_flag = (data[6] >> 0) & 0x01  # Extract bit 0 from byte 6

        # Update the current state information for this axis
        self.axis_states[axis_id] = axis_current_state
        self.is_done_moving[axis_id] = trajectory_done_flag
        self.procedure_result[axis_id] = procedure_result

        # Check for any errors and set the error_detected flag if any are found
        if axis_error:
            self.fatal_issue = True  # Will be important for stopping the robot as soon as any errors are detected
            print(f'Fatal ODrive issue: {axis_error}. More info coming soon...')

        # Record the time when this message came through
        self.last_heartbeat_time = time.time()

    def _handle_error(self, axis_id, message_data):
        # First split the error data into its constituent parts
        active_errors = struct.unpack_from('<I', message_data, 0)[0] # 4-byte integer
        disarm_reason = struct.unpack_from('<I', message_data, 4)[0]

        # Convert the 4-byte array into a 32-bit integer
        error_code = int.from_bytes(message_data, byteorder="little", signed=False)

        # If the error_code is 0, ignore it (no error)
        if error_code == 0:
            return

        # Dictionary mapping error codes to error messages
        ERRORS = {
        1: "INITIALIZING: The system is initializing or reconfiguring.",
        2: "SYSTEM_LEVEL: Unexpected system error such as memory corruption, stack overflow, etc. Indicates a firmware bug.",
        4: "TIMING_ERROR: An internal hard timing requirement was violated, usually due to computational overload.",
        8: "MISSING_ESTIMATE: Necessary position, velocity, or phase estimate was invalid. Possible uncalibrated encoder or axis not homed.",
        16: "BAD_CONFIG: Invalid or incomplete ODrive configuration. Check axis configuration settings.",
        32: "DRV_FAULT: The gate driver chip reported an error, possibly indicating hardware damage.",
        64: "MISSING_INPUT: No value provided for input parameters, typically due to unregistered RC PWM pulses.",
        256: "DC_BUS_OVER_VOLTAGE: DC voltage exceeded the configured limit. Verify brake resistor connections and settings.",
        512: "DC_BUS_UNDER_VOLTAGE: DC voltage fell below the configured limit. Check power connections and settings.",
        1024: "DC_BUS_OVER_CURRENT: Excessive DC current drawn at motor or board level.",
        2048: "DC_BUS_OVER_REGEN_CURRENT: Excessive DC regeneration current. Check if brake resistor can handle the current.",
        4096: "CURRENT_LIMIT_VIOLATION: Motor current exceeded the hard maximum limit. Adjust current limits.",
        8192: "MOTOR_OVER_TEMP: Motor temperature exceeded its configured upper limit.",
        16384: "INVERTER_OVER_TEMP: Inverter temperature exceeded its configured upper limit.",
        32768: "VELOCITY_LIMIT_VIOLATION: Estimated velocity exceeds the configured limit.",
        65536: "POSITION_LIMIT_VIOLATION: Position exceeds configured limits.",
        16777216: "WATCHDOG_TIMER_EXPIRED: Watchdog timer expired without being reset.",
        33554432: "ESTOP_REQUESTED: Emergency stop requested via external input.",
        67108864: "SPINOUT_DETECTED: Discrepancy detected between electrical and mechanical power, indicating a spinout.",
        134217728: "BRAKE_RESISTOR_DISARMED: Brake resistor was disarmed due to another error, typically undervoltage.",
        268435456: "THERMISTOR_DISCONNECTED: Thermistor enabled but disconnected, detected by abnormal voltage readings.",
        1073741824: "CALIBRATION_ERROR: Calibration procedure failed; see detailed error or procedure result."
    }

        # Check if the reason for disarming is in the dictionary
        if disarm_reason in ERRORS:
            # Log the corresponding error message
            # print(f"Error on axis {axis_id}: {ERRORS[disarm_reason]}")
            pass
        else:
            # If the error code is not recognized, log a default message
            # print(f"Unknown error on axis {axis_id}. Active errors: {active_errors}, Disarm reason: {disarm_reason}")
            pass

    def _handle_encoder_estimates(self, axis_id, data):
        # Start by unpacking the data, which is a 32-bit float
        pos_estimate, vel_estimate = struct.unpack_from('<ff', data)

        # Add the data to the appropriate buffer
        # self.position_buffer[axis_id] = - pos_estimate # NEGATIVE FOR TESTING
        # self.velocity_buffer[axis_id] = - vel_estimate

        # For testing with only the hand axis connected, just set all buffer values to the hand motor values
        self.position_buffer = [-pos_estimate] * self.num_axes
        self.velocity_buffer = [-vel_estimate] * self.num_axes


        # Print these values for debugging
        # print(f"Axis {axis_id} - Position: {pos_estimate:.2f}, Velocity: {vel_estimate:.2f}")

        if all(val is not None for val in self.position_buffer):
            # If the buffer is full, send it off!
            self.pos_values = self.position_buffer.copy() # Copy so that data doesn't change before being sent away
            self._trigger_callback('motor_positions', self.pos_values)
            self.position_buffer = [None] * self.num_axes  # Reset the buffer

            # If position buffer is full, then velocity buffer will also be full
            self.vel_values = self.velocity_buffer.copy()
            self._trigger_callback('motor_velocities', self.vel_values)
            self.velocity_buffer = [None] * self.num_axes  # Reset the buffer

    def _handle_iq_readings(self, axis_id, data):
        # Start by unpacking the data, which is a 32-bit float
        iq_setpoint, iq_measured = struct.unpack_from('<ff', data)

        # Add the data to the buffer
        # self.iq_meas_buffer[axis_id] = -iq_measured
        # self.iq_set_buffer[axis_id] = -iq_setpoint

        # For testing with only the hand axis connected, just set all buffer values to the hand motor values
        self.iq_meas_buffer = [-iq_measured] * self.num_axes
        self.iq_set_buffer = [-iq_setpoint] * self.num_axes

        # Print these values for debugging
        # print(f"Axis {axis_id} - IQ Setpoint: {iq_setpoint:.2f}, IQ Measured: {iq_measured:.2f}, IQ Buffer: {self.iq_buffer}")

        if all(val is not None for val in self.iq_meas_buffer):
            # If the buffer is full, send the data to the ROS network
            self.iq_meas_values = self.iq_meas_buffer.copy()
            self.iq_set_values = self.iq_set_buffer.copy()
            self._trigger_callback('motor_iqs', self.iq_meas_values)

            # Log a confirmation that these values have been saved, and print them
            # print(f"Motor currents: {self.iq_values}")

            # Reset the buffer
            self.iq_meas_buffer = [None] * self.num_axes

    def _handle_CAN_traffic_report(self, message):
        # Unpack the data into a 32-bit integer
        received_count = message.data[0] + (message.data[1] << 8)
        report_interval = message.data[2] + (message.data[3] << 8)

        traffic_report = {
            'received_count': received_count,
            'report_interval': report_interval,
        }

        # Trigger the callback
        self._trigger_callback('can_traffic', traffic_report)