'''
This script is used to tune the hand motor. It is used to find the optimal values for eg. torque_lim and vel_limit

Input position is from 0 to 2.5, where 0 is fully lowered and 2.5 is the peak.
This input position must be adjusted based on the 'home position', which is the position at the peak of the hand's stroke.
'''

import odrive
from odrive.enums import *
import time
import numpy as np
import math
import matplotlib.pyplot as plt

def home_sequence():
    current_lim = 7.5  # Current limit to detect hard stop
    home_pos_offset = 0.05
    
    mot.requested_state = 8  # Put in closed loop control
    mot.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ  # Set input mode to trapezoidal trajectory

    # Set a decent velocity and torque limit
    set_vel_torque_lims(vel_lim=1.0, torque_lim=10.0)

    # Set up trajectory control
    set_vel_acc(1.0, 1.0)  # Go real slow
    mot.controller.move_incremental(50, False)  # +ve is raising
    print("Running")

    while True:
        # Print the current
        # print(f"Current: {mot.motor.current_control.Iq_measured:.2f} A")

        if abs(mot.motor.current_control.Iq_measured) >= current_lim:  # If hard stop detected (current has spiked)
            top_pos = mot.encoder.pos_estimate      

            # Put the motor into passthrough mode
            mot.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
            
            # Set the target position to a low value
            mot.controller.input_pos = 0.0

            mot.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ  # Set input mode to trapezoidal trajectory

            # The above is done to ensure the setpoint actually changes to 0.0

            # Return the position we're at
            return top_pos - home_pos_offset  # Offset to give a bit of headroom

def set_vel_acc(vel, acc):
    mot.trap_traj.config.vel_limit = vel
    mot.trap_traj.config.accel_limit = acc
    mot.trap_traj.config.decel_limit = mot.trap_traj.config.accel_limit  # Set decel to equal accel

def set_vel_torque_lims(vel_lim, torque_lim):
    mot.controller.config.vel_limit = vel_lim
    mot.controller.config.vel_limit_tolerance = 1000
    mot.motor.config.torque_lim = torque_lim

def set_input_pos(pos):
    # Compensates for the home position when setting the input position.
    # Can only be run after home_sequence() has been run
    mot.controller.input_pos = pos + pos_adjustment

def catch_ball(catch_time, throw_vel):
    # Enter position control mode
    mot.controller.config.input_mode = INPUT_MODE_POS_FILTER  # Set input mode to position filter

    # Set a low velocity and torque limit 
    set_vel_torque_lims(vel_lim=throw_vel / revs_to_m, torque_lim=0.01)

    while time.time() < catch_time - 0.3:
        # Do nothing
        pass

    set_input_pos(0.0)

def throw_ball():
    # Enter position control mode
    mot.requested_state = 8  # Closed loop control
    mot.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ  # Set input mode to trapezoidal trajectory

    # Set a high velocity and acceleration limit
    set_vel_acc(100.0, 1500.0)
    set_vel_torque_lims(vel_lim=100.0, torque_lim=100.0)

    set_input_pos(1.8)

    # While the motor is moving, keep a record of its velocity
    while True:
        # Record the velocity and current time
        vel_buffer.append(mot.encoder.vel_estimate)
        pos_buffer.append(mot.encoder.pos_estimate)
        time_buffer.append(time.time())

        if abs(mot.encoder.pos_estimate - mot.controller.input_pos) < 0.1:
            break

        # time.sleep(0.01)

def plot_throw():
    # Plot the velocity and position of the motor during the throw on the same axes, with velocity on the left y-axis and position on the right y-axis
    fig, ax1 = plt.subplots()

    color = 'tab:red'
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Velocity (m/s)', color=color)
    ax1.plot(time_buffer, vel_buffer, color=color)
    ax1.tick_params(axis='y', labelcolor=color)

    ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis
    color = 'tab:blue'
    ax2.set_ylabel('Position (m)', color=color)  # we already handled the x-label with ax1
    ax2.plot(time_buffer, np.array(pos_buffer) * revs_to_m, color=color)
    ax2.tick_params(axis='y', labelcolor=color)

def calculate_throw_params(throw_vel):
    # Calculates how high the ball should go based on the throw velocity, assuming a vertical throw
    throw_height = (throw_vel**2) / (2 * 9.81)
    air_time = throw_vel / 9.81 * 2 # Time to reach peak and come back down

    return throw_height, air_time
    
def wait_until_reached_setpoint():
    while True:
        if abs(mot.encoder.pos_estimate - mot.controller.input_pos) < 0.05:
            break
        # time.sleep(0.1)

'''
---------------------------- Main ----------------------------
'''

# Connect to ODrive
od = odrive.find_any()
mot = od.axis0

# Set up buffers to store data
vel_buffer = []
pos_buffer = []
time_buffer = []

# Geometry values
spool_rad = 0.02 # Radius of the spool {m}
revs_to_m = 2 * spool_rad * math.pi  # Conversion factor from revolutions to meters

# Homing
top_pos = home_sequence()
pos_adjustment = top_pos - 2.5  # Calculate the adjustment to the input position. This must be added to all position setpoint commands
print(f"top_pos: {top_pos:.2f}, pos_adjustment: {pos_adjustment:.2f}")

# Move the hand down
# print("Moving to bottom")
set_input_pos(0.0)  # Move to lowest position

set_vel_torque_lims(vel_lim=5.0, torque_lim=1.0)
wait_until_reached_setpoint()

# print(f"Current pos: {mot.encoder.pos_estimate:.2f}, Setpoint pos: {mot.controller.pos_setpoint:.2f}, Input pos: {mot.controller.input_pos:.2f}")

# Throw
print("Throwing ball")
throw_ball()  # Throw the ball

# Calculate throw parameters
# Get the max velocity and the index at that point
vel_buffer = np.array(vel_buffer) * revs_to_m  # Convert to m/s
max_vel_idx = np.argmax(vel_buffer)
throw_vel = vel_buffer[max_vel_idx] 
throw_time = time_buffer[max_vel_idx]
print(f"Throw velocity: {throw_vel:.2f} m/s")
throw_height, air_time = calculate_throw_params(throw_vel)
print(f"Throw height: {throw_height:.2f} m \nAir time: {air_time:.2f} s")

# Catch
# print("Catching ball")
# catch_time = throw_time + air_time
# catch_ball(catch_time, throw_vel=throw_vel)  # Catch the ball

# wait_until_reached_setpoint()

mot.requested_state = 1  # Stop axis (1 = IDLE)

plot_throw()  # Plot the velocity of the motor
plt.show() # Show the plot