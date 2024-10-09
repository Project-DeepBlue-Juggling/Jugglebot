import can
import cantools
import time
import math
import matplotlib.pyplot as plt
from ruckig import InputParameter, Trajectory, Result, Ruckig
from can_handler import CANHandler

def move_to_start_pos():
    # Set the hand gains to be the default values
    can_handler.set_hand_gains(pos_gain=18, vel_gain=0.008, vel_integrator_gain=0.01)

    # Put the motor into closed-loop control with POS_FILTER control mode
    can_handler._set_requested_state(axis_id=6, requested_state='CLOSED_LOOP_CONTROL')
    time.sleep(0.05)
    can_handler._set_control_mode(axis_id=6, control_mode='POSITION_CONTROL', input_mode='POS_FILTER')
    time.sleep(0.05)

    # Set the target position
    target_pos = lowest_pos
    can_handler.send_position_target(axis_id=6, setpoint=target_pos, max_position=throw_stroke)

    print(f'Start position: {target_pos:.2f}')

    # Wait for the motor to reach the target position
    while True:
        can_handler.fetch_messages()
        pos = can_handler.pos_values[6]
        # print(pos)
        if pos is not None:
            if abs(pos - target_pos) < 0.02:
                break
            time.sleep(0.01)

    
    # Now set the hand gains to be the throwing values
    can_handler.set_hand_gains(pos_gain=35, vel_gain=0.007, vel_integrator_gain=0.07)

    # And put the axis in PASSTHROUGH mode
    can_handler._set_control_mode(axis_id=6, control_mode='POSITION_CONTROL', input_mode='PASSTHROUGH')

def shutdown():
    # Put the motor into idle mode
    can_handler._set_requested_state(axis_id=6, requested_state='IDLE')

    # Close the CAN handler
    can_handler.close()

# All of this code will run on the hand motor, with axis id = 6

# Initialize the CAN handler
can_handler = CANHandler()

spool_dia_mm = 11.7
linear_gain = 1000 / (math.pi * spool_dia_mm)  # rev / m

stroke = 8.8    # revs (actually 8.867)
throw_stroke = 6 # revs
lowest_pos = (stroke - throw_stroke) / 2 # revs
highest_pos = lowest_pos + throw_stroke  # revs

throw_pos = (lowest_pos + highest_pos) * 0.7  # revs.

target_throw_height = 0.8 # m
throw_vel = math.sqrt(2 * 9.81 * target_throw_height) * linear_gain # rev/s

print(f'Linear gain: {linear_gain:.2f} rev/m, highest pos: {highest_pos:.2f} revs\n')
print(f'Throw position: {throw_pos:.2f} revs, velocity: {throw_vel:.2f} rev/s, height: {target_throw_height:.2f} m\n')

# For converting acceleration into torque
total_inertia_ref_to_hand = 0.37 # kg

# Set kinematic limits
max_vel = 25.0 * linear_gain # rev/s
max_accel = 150.0 * linear_gain # rev/s^2
max_jerk = 2300.0 * linear_gain # rev/s^3

print(f'Max vel: {max_vel:.2f} rev/s, max accel: {max_accel:.2f} rev/s^2, max jerk: {max_jerk:.2f} rev/s^3\n')

# Move to the start position
move_to_start_pos()
time.sleep(1.0)

# Set initial conditions
x0 = can_handler.pos_values[6]
v0 = 0.0
a0 = 0.0
t0 = 0.0
dt = 0.01

# Create lists for tracking (theoretical [th] and experimental [ex])
t = [t0]
x_th = [x0]
v_th = [v0]
a_th = [a0]
iq_set = [0.0]

x_ex = [x0]
v_ex = [can_handler.vel_values[6]]
iq_ex = [can_handler.iq_meas_values[6]]

# Create the Ruckig object
otg = Ruckig(1, dt)
accel_inp = InputParameter(1)
decel_inp = InputParameter(1)

accel_trajectory = Trajectory(1)
decel_trajectory = Trajectory(1)

# Acceleration phase
accel_inp.current_position = [x0]
accel_inp.current_velocity = [v0]
accel_inp.current_acceleration = [a0]

accel_inp.target_position = [throw_pos]
accel_inp.target_velocity = [throw_vel]
accel_inp.target_acceleration = [0.0]

accel_inp.max_velocity = [max_vel] # m/s
accel_inp.max_acceleration = [max_accel]
accel_inp.max_jerk = [max_jerk] # m/s^3

# Calculate the trajectory in an offline manner
accel_result = otg.calculate(accel_inp, accel_trajectory)

# Check to make sure we have a valid input
if accel_result == Result.ErrorInvalidInput:
    shutdown()
    raise Exception("Invalid input - Accel phase")

pred_throw_pos, pred_throw_vel, pred_throw_acc = accel_trajectory.at_time(accel_trajectory.duration)

# Deceleration phase
decel_inp.current_position = [pred_throw_pos[0]]
decel_inp.current_velocity = [pred_throw_vel[0]]
decel_inp.current_acceleration = [pred_throw_acc[0]]

decel_inp.target_position = [highest_pos]
decel_inp.target_velocity = [0.0]
decel_inp.target_acceleration = [0.0]

decel_inp.max_velocity = [max_vel * 1.5] # m/s
decel_inp.max_acceleration = [max_accel * 4] # We can decelerate faster than we can accelerate since no ball inertia
decel_inp.max_jerk = [max_jerk * 5] # m/s^3

# Calculate the trajectory in an offline manner
decel_result = otg.calculate(decel_inp, decel_trajectory)

if decel_result == Result.ErrorInvalidInput:
    shutdown()
    raise Exception("Invalid input - Decel phase")

print(f'Accel trajectory duration: {accel_trajectory.duration:.4f} s, Decel trajectory duration: {decel_trajectory.duration:.4f} s')

# Get some info about the position extrema of the trajectory
print(f'Accel position extrema are {accel_trajectory.position_extrema[0]}, Decel extrema are {decel_trajectory.position_extrema[0]}\n')

throw_steps = 0
released = False
extra_time = 0.2 # We don't know the throw duration right now, so we'll just wait for a bit
return_step = 0
air_time = 0.0
delay_time = 0.0
first_wait_step = True

# Step through the trajectory and send the position commands, as well as logging the data
frames = int((accel_trajectory.duration + decel_trajectory.duration + extra_time )/ dt)

for i in range(frames):
    # Get the most up-to-date data on what the motor is actually doing
    can_handler.fetch_messages()
    x_ex.append(can_handler.pos_values[6])
    v_ex.append(can_handler.vel_values[6])
    iq_ex.append(can_handler.iq_meas_values[6])
    iq_set.append(can_handler.iq_set_values[6])

    # If we're still inside the trajectory, get the theoretical values
    if i * dt < accel_trajectory.duration:
        # Get the new position, velocity, and acceleration
        new_pos, new_vel, new_acc = accel_trajectory.at_time(i * dt)
        throw_steps += 1

    elif i * dt < accel_trajectory.duration + decel_trajectory.duration:
        # Accel phase is over, now we're in the decel phase
        traj_time = i * dt - accel_trajectory.duration
        new_pos, new_vel, new_acc = decel_trajectory.at_time(traj_time)


    # # Once we reach the end of the trajectory, wait for air_time before carrying out the trajectory in reverse (to catch - dodgily)
    # elif i * dt < accel_trajectory.duration + air_time + delay_time:
    #     # Do nothing
    #     if first_wait_step:
    #         # Set the hand gains low for the catch
    #         # can_handler.set_hand_gains(pos_gain=0.1, vel_gain=0.0001, vel_integrator_gain=0.01)
    #         first_wait_step = False
    #         pass
    #     return_step = i # Get the last step index before we start returning
    #     pass

    # elif i * dt < 2 * accel_trajectory.duration + air_time:
    #     # Get the new position, velocity, and acceleration
    #     traj_time = accel_trajectory.duration - (i - return_step) * dt
    #     # print(traj_time)
    #     # print(f'Return step = {return_step}. i = {i}. Time = {traj_time:.4f} s')
    #     new_pos, new_vel, new_acc = accel_trajectory.at_time(traj_time)
    #     new_vel[0] = -new_vel[0]

    else:
        # We're done. Set target vel and accel to 0
        new_pos = [highest_pos]
        new_vel = [0.0]
        new_acc = [0.0]
        pass

    # Log the data
    v_th.append(new_vel[0])
    a_th.append(new_acc[0])
    x_th.append(new_pos[0])
    t.append(i * dt)

    # Send the position command
    can_handler.send_arbitrary_message(axis_id=6, msg_name='set_input_pos', setpoint=new_pos[0])

    # Send the velocity FF command
    can_handler.send_arbitrary_message(axis_id=6, msg_name='set_input_vel', setpoint=new_vel[0])

    # Convert the acceleration into torque and send it
    torque = new_acc[0] * total_inertia_ref_to_hand * spool_dia_mm / 2000 # Nm
    # can_handler.send_arbitrary_message(axis_id=6, msg_name='set_input_torque', setpoint=-torque)
    
    time.sleep(dt)


max_v_th = max(v_th)
max_v_ex = max(v_ex)

print(f'Maximum theoretical velocity: {max_v_th:.2f} rev/s')
print(f'Maximum actual velocity: {max_v_ex:.2f} rev/s')

# Calculate throw stats
g = 9.81 # m/s^2
height = (max_v_th / linear_gain) ** 2 / (2 * g)
print(f'Throw height above release point: {height:.2f} m')

throw_duration = 2 * math.sqrt(2 * height / g)
print(f'Throw duration: {throw_duration:.2f} s')


shutdown()

# Plot the results on separate subfigures
plt.figure()

plt.subplot(4, 1, 1)
plt.title('Two-stage throw test.\nRed line denotes end of accel stage')
plt.plot(t, x_th)
plt.plot(t, x_ex)
plt.legend(['Theoretical', 'Experimental'])
plt.ylabel('Position\n(revs)')
plt.axvline(x=throw_steps * dt, color='red', linestyle='dotted')
plt.grid()

plt.subplot(4, 1, 2)
plt.plot(t, v_th)
plt.plot(t, v_ex)
plt.legend(['Theoretical', 'Experimental'])
plt.ylabel('Velocity\n(revs / s)')
plt.axvline(x=throw_steps * dt, color='red', linestyle='dotted')
plt.grid()

plt.subplot(4, 1, 3)
plt.plot(t, a_th)
plt.ylabel('Acceleration\n(revs / s^2)')
plt.axvline(x=throw_steps * dt, color='red', linestyle='dotted')
plt.grid()

plt.subplot(4, 1, 4)
plt.plot(t, iq_set)
plt.plot(t, iq_ex)
plt.legend(['Setpoint', 'Measured'])
plt.ylabel('Iq')
plt.axvline(x=throw_steps * dt, color='red', linestyle='dotted')
plt.grid()
plt.xlabel('Time (s)')


plt.show()