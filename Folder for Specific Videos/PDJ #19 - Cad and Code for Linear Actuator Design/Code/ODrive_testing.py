import odrive
import time
import numpy as np
import math


def boot():
    # Run just after booting ODrive
    legs[1].requested_state = 3  # 3 is FULL_CALIBRATION_SEQUENCE (necessary after booting)
    time.sleep(25)


def home_sequence():
    leg = legs[1]
    leg.requested_state = 8  # Put in closed loop control

    # Set up trajectory control
    set_vel_acc(1.0, 1.0, leg)  # Go real slow
    leg.controller.move_incremental(50, False)  # +ve value is contraction for axis1
    print("Running")

    while True:
        if abs(leg.motor.current_control.Iq_measured) >= 6:  # If hard stop detected (current has spiked)
            leg.requested_state = 1  # Stop axis (1 = IDLE)
            leg.controller.input_pos = leg.encoder.pos_estimate  # Reset target position to current position
            time.sleep(1)
            leg.requested_state = 8  # Return to closed loop control
            leg.controller.move_incremental(-0.2, False)  # Move a small amount to give a slight buffer
            time.sleep(1)  # Give enough time for the motor to move to the buffer position
            home_pos = leg.encoder.pos_estimate  # Keep a record of the current encoder reading. This will be "0".
            return home_pos


def measure_current():
    # Used to figure out the standard running current, which informs the lower bound for the homing current limit
    leg = legs[1]
    leg.requested_state = 8
    current = []

    # Set up trajectory control
    leg.trap_traj.config.vel_limit = 1.0  # Go nice and slow
    leg.trap_traj.config.accel_limit = 10.0
    leg.trap_traj.config.decel_limit = leg.trap_traj.config.accel_limit  # Set deceleration to equal acceleration

    start_pos = leg.encoder.pos_estimate
    turns_to_move = 3.0
    leg.controller.move_incremental(turns_to_move, False)
    print("Running")

    while leg.encoder.pos_estimate < start_pos + turns_to_move - 0.2:
        current.append(leg.motor.current_control.Iq_measured)

    print("Max current: ", max(abs(np.array(current))))
    # print(current)


def set_vel_acc(vel, acc, which_leg):
    # NEED TO UPDATE TO WORK WITH MULTIPLE LEGS
    which_leg.trap_traj.config.vel_limit = vel
    which_leg.trap_traj.config.accel_limit = acc
    which_leg.trap_traj.config.decel_limit = which_leg.trap_traj.config.accel_limit  # Set decel to equal accel


def move_straight():
    leg = legs[1]
    zero_pos = home_pos
    set_vel_acc(60.0, 600.0, leg)
    # leg.controller.move_incremental(-4.0, False)  # Different way to do it. This invokes rounding errors though
    leg.controller.input_pos = zero_pos - 2.0


def move_continuous():
    leg = legs[1]
    zero_pos = home_pos - 2.2

    direction = 1  # Start with +ve. This is contraction for axis1
    amplitude = 4   # Rotations of motor

    vel = 10.0  # Velocity {turns/sec}
    accel = 400.0  # Accel {turns/sec^2}
    cycles = 2000  # Number of cycles to run for

    leg.controller.input_pos = zero_pos + amplitude / 2  # Move to end (contracted) pos to begin
    set_vel_acc(vel, accel, leg)
    cycle = 0
    while cycle <= cycles:
        if abs(leg.controller.input_pos - leg.encoder.pos_estimate) < 0.01:
            print("direction change")
            direction = direction * -1
            # leg.controller.move_incremental(direction * amplitude, False)
            leg.controller.input_pos = zero_pos + direction * amplitude / 2
            cycle += 1
            print(cycle)


def move_faster():
    leg = legs[1]

    zero_pos = home_pos - 2.2

    direction = 1  # Start with -ve. This is extension for axis1
    amplitude = 4   # Rotations
    initial_vel = 1.0
    final_vel = 60.0
    accel = 600.0
    cycles_per_speed = 3
    cycle = 0

    leg.controller.input_pos = zero_pos + amplitude / 2  # Move to end (contracted) pos to begin

    set_vel_acc(initial_vel, accel, leg)

    vel = initial_vel
    while vel <= final_vel:
        if abs(leg.controller.input_pos - leg.encoder.pos_estimate) < 0.01:
            print("direction change")
            direction = direction * -1
            leg.controller.move_incremental(direction * amplitude, False)
            cycle += 1

        if cycle == cycles_per_speed:
            vel += 2.0
            print("Speed change. New vel: ", vel)
            set_vel_acc(vel, accel, leg)
            if vel == final_vel:
                cycles_per_speed = 10
            cycle = 0


od = odrive.find_any()

legs = [od.axis0, od.axis1]

# boot()  # COMMENT OUT WHEN ODRIVE IS ALREADY ON

for leg in legs:
    # Make sure initial values are set correctly. This isn't strictly necessary if the ODrive config was saved, but
    # it doesn't hurt.
    leg.motor.config.current_lim = 30.0
    set_vel_acc(60.0, 10.0, leg)
    leg.controller.config.input_mode = 5  # Make sure we're in trapezoidal trajectory mode


# Uncomment any of the movement modes that you want to use:
# measure_current()
home_pos = home_sequence()
# move_faster()
# move_straight()
move_continuous()

# At the end, return tested leg(s) to IDLE if they've completed their movement(s)
leg = legs[1]
while True:
    if abs(leg.encoder.pos_estimate - leg.controller.input_pos) < 0.01:
        leg.requested_state = 1
        print("Home: ", home_pos)
        print("Motor pos: ", legs[1].controller.pos_setpoint)
        # Check for errors
        print(leg.error)
        break
    else:
        time.sleep(0.1)

