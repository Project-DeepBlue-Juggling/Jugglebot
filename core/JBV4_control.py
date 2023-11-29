import can
import cantools
import time
import threading
import matplotlib.pyplot as plt
import numpy as np
from can_handler import CANHandler
from spacemouse_handler import SpaceMouseHandler
from stewart_platform import StewartPlatform
from moving_average_filter import MovingAverageFilter
from pattern_creator import PatternCreator


fig = plt.figure()
ax1 = fig.add_subplot(211)
ax2 = fig.add_subplot(212)
fig.subplots_adjust(hspace=0.5)

sp_fig = plt.figure()
sp_ax = sp_fig.add_subplot(121, projection='3d')
legs_ax = sp_fig.add_subplot(122)

db = cantools.database.load_file("ODrive_CAN.dbc")

plot_list = []
plot_list2 = []

MAX_POSITION = 4.2  # The most extended the actuators should ever be (revolutions)

# Initialize bespoke classes
spacemouse = SpaceMouseHandler()
stewart_platform = StewartPlatform(sp_ax)
filter = MovingAverageFilter(window_size=100)  # 100 seems to work well with time.sleep(0.001) in the main loop
pattern = PatternCreator

# Use a context manager to ensure resources are freed and to better handle crashes/CTRL-C kills
with CANHandler(bus_name='can0', bitrate=1000000, bus_type='socketcan') as handler:
    def run_setup(current_limit=20.0, velocity_limit=10.0, accel_limit=10.0, num_motors=6):
        print("Setting motor limits...")

        # Set the current limit
        for axisID in range(num_motors):
            # Set the absolute velocity limit a little higher than what will be used by the trap traj controller
            data = db.encode_message(f'Axis{axisID}_Set_Limits', 
                                    {'Velocity_Limit':velocity_limit * 1.2, 'Current_Limit': current_limit})
            
            handler.send_message(axis_id=axisID, command_name="set_vel_curr_limits", data=data, error_descriptor='Current/Vel Limits')
            time.sleep(0.005)

        # Set the input mode to be position control with trapezoidal trajectory
        for axisID in range(num_motors):
            # handler.set_control_mode(axis_id=axisID, control_mode='POSITION_CONTROL', input_mode='PASSTHROUGH')
            handler.set_control_mode(axis_id=axisID, control_mode='POSITION_CONTROL', input_mode='TRAP_TRAJ')
            time.sleep(0.005)

        # Set the velocity and acceleration limits
        for axisID in range(num_motors):
            handler.set_trap_traj_vel_acc(axis_id=axisID, vel_limit=velocity_limit, acc_limit=accel_limit)
            time.sleep(0.005)

        #  Put the motors in closed loop control mode
        for axisID in range(num_motors):
            handler.set_requested_state(axisID, 'CLOSED_LOOP_CONTROL')
            # handler.set_requested_state(axisID, 'IDLE')
            time.sleep(0.005)

    def send_position_target(axisID, setpoint, max_position=MAX_POSITION, min_position=0.0):
        """
        Sends a position target message over CAN for a given axisID.

        Parameters:
            axisID (int): The axis ID for the motor.
            setpoint (float): The position target.
        """
        
        if setpoint > max_position or setpoint < min_position:
            print(f"Setpoint of {setpoint:.2f} is outside of allowable bounds and has been truncated.")

        setpoint = max(min_position, min(setpoint, max_position))  # Truncate the setpoint if it exceeds the allowable values

        setpoint = -1 * setpoint  # Since -ve is extension

        data = db.encode_message(f'Axis{axisID}_Set_Input_Pos', 
                                {'Input_Pos': setpoint, 'Vel_FF': 0.0, 'Torque_FF': 0.0})
        
        handler.send_message(axis_id=axisID, command_name="set_input_pos", data=data, error_descriptor="position target")
    
    def start_periodic_setpoint_msg(initial_setpoint, period, max_position=MAX_POSITION):
        """
        NEED TO UPDATE!!! OLD!!!

        Starts the transmission of the position target message over CAN for all axisIDs.

        Parameters:
            initial_setpoint (float): The initial position target.
            period (float): The period at which messages should be sent (in seconds)
            max_position (float): The highest allowable position for the setpoint (revolutions)
        """

        # Start by ensuring the setpoint is within allowable bounds
        setpoint = initial_setpoint

        if setpoint > max_position or setpoint < 0:
            print(f"Setpoint of {setpoint:.2f} is outside of allowable bounds and has been truncated.")

        setpoint = max(0, min(setpoint, max_position))  # Truncate the setpoint if it exceeds the allowable values

        setpoint = -1 * setpoint  # Since -ve is extension

        tasks = [] # To be populated with the periodic "tasks" that send the messages
        messages = []  # The list of messages that will be sent. Useful as it means we don't need to reconfigure arb_ids again later

        for axis_id in range(6):
            arbitration_id = (axis_id << 5) | 0x00C # 0x00C is "Set input pos"
            data = db.encode_message(f'Axis{axis_id}_Set_Input_Pos', 
                                {'Input_Pos': setpoint, 'Vel_FF': 0.0, 'Torque_FF': 0.0})
            
            # Create the CAN message
            msg = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=False)

        #     task = bus.send_periodic(msg, period=period)

        #     if not isinstance(task, can.ModifiableCyclicTaskABC):
        #         print(f"This interface doesn't seem to support modification for axisID {axis_id}")
        #         task.stop()
        #     else:
        #         messages.append(msg)
        #         tasks.append(task)

        # return messages, tasks
        
    def home_sequence(num_motors=6):
        # Start by setting the motor limits
        run_setup()

        # Runs all the motors until the physical limit reached (current spikes)
        print("Homing motors...")
        homing_speed = 1.0 # Go real slow
        current_limit = 5.0  # Found experimentally
        current_limit_headroom = 3.0  # Headroom for limit of how high the current should go above "current_limit"

        for axisID in range(num_motors):
            # Put the axis in velocity control mode
            handler.set_requested_state(axisID, requested_state='CLOSED_LOOP_CONTROL')
            handler.set_control_mode(axis_id=axisID, control_mode='VELOCITY_CONTROL', input_mode='VEL_RAMP')

            # Set absolute current limit
            data = db.encode_message(f'Axis{axisID}_Set_Limits',{'Current_Limit':current_limit + current_limit_headroom,
                                                                 'Velocity_Limit':homing_speed*2})
            
            handler.send_message(axis_id=axisID, command_name="set_vel_curr_limits", data=data)

            # Start the motor moving at the prescribed speed
            data = db.encode_message(f'Axis{axisID}_Set_Input_Vel',
                                    {'Input_Vel': homing_speed, 'Input_Torque_FF': 0.0})
            
            handler.send_message(axis_id=axisID, command_name="set_input_vel", data=data)

            time.sleep(0.5)  # Give enough time for the motor to start moving

            # Stuff for an exponential average filter
            avg_weight = 0.7
            moving_avg = 0

            while True:
                if handler.fatal_issue:
                    print("FATAL ISSUE! Stopping process")
                    break
                # Fetch any recent messages on the CAN bus
                handler.fetch_messages()
                
                # Get the current for the relevant motor
                current, _ = handler.get_iq_values()
                current = current[axisID][-1]  # Get just the last value of the current readings, to add to the moving avg filter

                # Accumulate in exponential moving average
                moving_avg = moving_avg * avg_weight + current * (1 - avg_weight)

                # Debugging
                # print(f"Current = {current:.2f}, moving avg = {moving_avg:.2f}")
                # plot_list.append(current)  # For plotting

                # Check if the limit has been reached
                if abs(moving_avg) >= current_limit:
                    handler.set_requested_state(axisID, requested_state='IDLE')
                    print(f"Motor {axisID} homed!")

                    # Set the encoder position to be a little more than 0 (so that 0 is a bit off the end-stop. +ve is contraction)
                    data = db.encode_message(f'Axis{axisID}_Set_Linear_Count',
                                             {'Position': 1200})  # Recall encoder has 8192 CPR
                    
                    handler.send_message(axis_id=axisID, command_name="set_linear_count", data=data, error_descriptor="Set encoder pos in homing")
                    break

        # Now that all motors have been homed, reset them to have desired properties
        run_setup()

    def move_slowly_to_position(pos_setpoint):
        ''' Move all the motors to a set position slowly. Exit the function once the motors are in position.
        Is a little buggy right now. Maybe change to be "move until encoder position = 0"?
        '''
        
        # Set a low speed
        run_setup(velocity_limit=2.0)
        
        # Command all motors to move to the desired position
        for axisID in range(6):
            send_position_target(axisID=axisID, setpoint=pos_setpoint)
            time.sleep(0.001)

        # Add a short break to make sure commands have sent and movement flags have been updated
        time.sleep(0.5)
        handler.fetch_messages()

        # Check to make sure all the motors have finished their movements
        while not all(handler.is_done_moving):
            handler.fetch_messages()
            time.sleep(0.001)

        # Return the motor velocity to what it was before
        run_setup()

    def get_spacemouse_pose():
        # Take in the spacemouse data (-1 < data < 1) and map it to the relevant range(s)
        raw_data = spacemouse.read()
        x, y, z, r, p, yaw = raw_data

        # Set up the multipliers for each axis (mm, deg) 
        # This is equivalent to setting the limits for the platform movement (as controlled by the spacemouse)
        xy_mult = 150
        z_mult = 150
        pitch_roll_mult = 30
        yaw_mult = 30

        x = x * xy_mult
        y = y * xy_mult
        z = z * z_mult
        p = p * pitch_roll_mult
        r = r * pitch_roll_mult
        yaw = yaw * yaw_mult

        return [x, y, z, p, r, yaw]

    def convert_mm_to_rev(input_lengths):
        # Convert the input lengths from mm into revs
        spool_dia = 22 # mm
        mm_to_rev = 1 / (spool_dia * np.pi)

        converted_lengths = [length * mm_to_rev + MAX_POSITION / 2 for length in input_lengths]

        return converted_lengths

    def reorder_axes_for_CAN(input_position_commands):
        '''Reorders the axis commands to align with the nodes of the CAN bus.
        This is needed because the Stewart Platform class assumes node IDs in certain (relative) positions
        and remapping the outputs is simpler than remodelling the SP class'''
        
        # Define the schema for mapping SP nodes to CAN nodes
        schema = [1, 4, 5, 2, 3, 0]

        CAN_position_commands = [input_position_commands[schema.index(i)] for i in range(6)]

        return CAN_position_commands

    def send_sinusoidal_position_with_canHandler(t_start, t_current, freq=0.5, amplitude=2.0):
        # Move the motors in a sinusoidal pattern.
        # Takes in the start time and current time and returns the position for the current time
        # Freq and amplitude control the shape of the sinusoid being created. Units are Hz and Revs, respectively

        # Calculate the elapsed time
        t_elapsed = t_current - t_start

        # Calculate the desired position using the sine wave formula
        position = amplitude * (1 - np.cos(2 * np.pi * freq * t_elapsed))

        # print(f'Position = {position:.2f}')

        return [position] * 6 # Cast the position value to all 6 motors

    def straight_throw(throw_pos):
        ''' A temporary method to test throwing perfectly vertically.
        throw_pos is the position that the motors will move to when throwing'''
        run_setup(velocity_limit=40.0, accel_limit=300.0)
        for axisID in range(6):
            send_position_target(axisID=axisID, setpoint=throw_pos)

    handler.clear_errors()
    # handler.reboot_odrives()

    '''Uncomment the following two lines if the platform doesn't return to pos=0 after the "end" command is sent.
    Not sure why this happens sometimes...'''
    # move_slowly_to_position(pos_setpoint=0.0)
    # raise ValueError

    axis_of_interest = 0  # For plotting etc.

    home_sequence()

    frequency_for_pattern = 0.2

    spacemouse_or_pattern = 1  # 1 for spacemouse, 0 for pattern
    
    duration = 20  # Seconds
    spacemouse_data = [[] for _ in range(6)]
    spacemouse_timestamps = []

    # Set a flag to check if the loop should still be running
    keep_running = True
    
    # Initialize a list to store the commanded pose
    commanded_pose = [0] * 6

    def check_for_commands():
        global keep_running, commanded_pose
        while True:
            user_input = input()
            if user_input.strip().lower() == "end":
                keep_running = False
                break
            elif user_input.strip().lower() == "throw":
                straight_throw(throw_pos=4.0)
                # break
            else:
                try:
                    # Attempt to parse the input as a list of numbers
                    new_pose = list(map(float, user_input.split(',')))
                    if len(new_pose) == 6:
                        commanded_pose = new_pose
                        print(f'New pose command received: {commanded_pose}')
                    else:
                        print("Please enter six numbers separated by commas")
                except ValueError:
                    print("Invalid input. Please enter numbers only, separated by commas")


    # Start the thread to check for user input
    end_thread = threading.Thread(target=check_for_commands)
    end_thread.start()

    # Move the platform to the mid-position
    # move_slowly_to_position(pos_setpoint=MAX_POSITION / 2)

    while keep_running:
        handler.fetch_messages()
        time.sleep(0.01)
    
    print("Receiving spacemouse inputs and moving motors!")
    start_time = time.time()
    # while time.time() - start_time < duration:
    # while keep_running:
    #     if handler.fatal_issue:
    #         print("FATAL ISSUE! Stopping process")
    #         break
        
    #     if spacemouse_or_pattern:  # If using spacemouse control
    #         # Get the pose and update the stewart platform
    #         raw_pose = get_spacemouse_pose()
    #         pose = filter.update_and_filter(raw_pose)

    #     else:  # If using pattern control
    #         # leg_lengths_revs = send_sinusoidal_position_with_canHandler(t_start=start_time, t_current=time.time(), freq=frequency_for_pattern)
    #         # pose = pattern.flat_circle_path(height=MAX_POSITION / 2, radius=80, time_start=start_time, freq=frequency_for_pattern)
    #         pose = pattern.conical_path(height_of_platform=MAX_POSITION / 2, cone_height= 80, radius=80, time_start=start_time, freq=frequency_for_pattern)
    #         # pose = commanded_pose
        
    #     stewart_platform.update_pose(pose)

    #     # # Calculate the leg lengths for this pose, then convert to revs
    #     leg_lengths_mm = stewart_platform.leg_lengths
    #     leg_lengths_revs = convert_mm_to_rev(leg_lengths_mm[:6])  # The last "leg" is the hand string, which we don't care about here

    #     # Sort the leg lengths to match the layout of the CAN node IDs
    #     leg_lengths_final = reorder_axes_for_CAN(leg_lengths_revs)

    #     spacemouse_timestamps.append(time.time())

    #     # print(["{:.2f}".format(length) for length in leg_positions])
    #     for axis, data in enumerate(leg_lengths_final):
    #         send_position_target(axisID=axis, setpoint=data)
    #         spacemouse_data[axis].append(-1 * data)

    #     # stewart_platform.update_pose(pose)
    #     # stewart_platform.plot_platform()
    #     handler.fetch_messages()
    #     time.sleep(0.001)

    pos_estimates, enc_timestamps = handler.get_position_estimates()
    vel_estimates, vel_timestamps = handler.get_velocity_estimates()
    _, iq_estimates = handler.get_iq_values()

    # Subtract the starting time from each timestamp value, to "normalize" the timestamps
    spacemouse_timestamps = [t - start_time for t in spacemouse_timestamps]
    enc_timestamps = [t - start_time for t in enc_timestamps[axis_of_interest]]


    # End by slowly sending all motors back to position = 0
    move_slowly_to_position(pos_setpoint=0.0)

    # Put all axes in IDLE state
    for axisID in range(6):
        handler.set_requested_state(axis_id=axisID, requested_state='IDLE')

    end_thread.join()

    ax1.plot(enc_timestamps, pos_estimates[axis_of_interest], label="Encoder Estimates")
    ax1.plot(enc_timestamps, vel_estimates[axis_of_interest], label="Velocity Estimates")
    # ax1.plot(spacemouse_timestamps, spacemouse_data[axis_of_interest], label="Commanded Positions")
    ax2.plot(iq_estimates[axis_of_interest])
    ax1.legend(loc='upper right')
    ax1.set_title(f'Actual vs. Commanded Motor Positions for Axis {axis_of_interest}')
    ax2.set_title(f"Iq Current for axis {axis_of_interest}")
    ax2.set_ylabel("Measured Current (A)")

plt.close(fig=sp_fig)
plt.show()







