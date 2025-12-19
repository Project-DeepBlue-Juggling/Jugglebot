# This script will run a motor from an ODrive Pro

import odrive
import time
import odrive.enums
import odrive.utils
import math
import numpy as np

# Connect to ODrive
print("Finding an ODrive...")
odrv = odrive.find_any()
print("ODrive found: ", odrv.serial_number)


# Assume that the ODrive is already configured for AS5047P encoders

def set_motor_position(motor, position, offset=0.0):
    """
    Set the position of the motor.
    :param motor: ODrive motor object (e.g., odrv.axis0 or odrv.axis1)
    :param position: Desired position in revolutions (float)
    :param offset: Offset to be added to the position (float)
    """
    motor.controller.input_pos = position + offset  # Set the position in revolutions

def set_axis_input_mode(motor, mode):
    """
    Set the input mode for the motor axis.
    :param motor: ODrive motor object (e.g., odrv.axis0 or odrv.axis1)
    :param mode: Desired input mode (e.g., odrive.enums.INPUT_MODE_PASSTHROUGH)
    """
    motor.controller.config.input_mode = mode
    print(f"Input mode set to {mode}")

def set_axis_gains(motor, pos_gain=30.0, vel_gain=0.05, vel_integrator_gain=0.00):
    """
    Set the position and velocity gains for the motor axis.
    :param motor: ODrive motor object (e.g., odrv.axis0 or odrv.axis1)
    :param pos_gain: Position gain (float)
    :param vel_gain: Velocity gain (float)
    :param vel_integrator_gain: Velocity integrator gain (float)
    """
    motor.controller.config.pos_gain = pos_gain
    motor.controller.config.vel_gain = vel_gain
    motor.controller.config.vel_integrator_gain = vel_integrator_gain
    print(f"Gains set: pos_gain={pos_gain}, vel_gain={vel_gain}, vel_integrator_gain={vel_integrator_gain}")

def set_trap_traj_limits(motor, vel_limit=1.0, accel_limit=1.0):
    """
    Set the trajectory limits for the motor axis.
    :param motor: ODrive motor object (e.g., odrv.axis0 or odrv.axis1)
    :param vel_limit: Velocity limit in revolutions per second (float)
    :param accel_limit: Acceleration limit in revolutions per second squared (float)
    """
    motor.trap_traj.config.vel_limit = vel_limit
    motor.trap_traj.config.accel_limit = accel_limit
    motor.trap_traj.config.decel_limit = accel_limit
    # print(f"Trajectory limits set: vel_limit={vel_limit}, accel_limit={accel_limit}")

def get_motor_position(motor):
    """
    Get the current position of the motor.
    :param motor: ODrive motor object (e.g., odrv.axis0 or odrv.axis1)
    :return: Current position in revolutions (float)
    """
    position = motor.encoder.pos_estimate
    return position

def M1_test():
    """
    Test function for M1 motor.
    """
    test_drv = odrv.axis0
    # Get the current position of the motor. This will be used to offset all position commands
    start_pos = -3.020303249359131
    # start_pos = test_drv.pos_estimate
    # print(f"Starting position of M1: {start_pos} revs")
    # return

    test_drv.config.anticogging.enabled = False

    # Set the input mode to PASSTHROUGH for M1
    set_axis_input_mode(test_drv, odrive.enums.INPUT_MODE_PASSTHROUGH)

    # Set the position and velocity gains for M1
    set_axis_gains(test_drv, pos_gain=25.0, vel_gain=0.5, vel_integrator_gain=0.8)

    # Put the motor in closed loop control state
    test_drv.requested_state = odrive.enums.AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(1)  # Allow some time for the motor to stabilize

    # Loop through positions in 1 degree increments from 0 to 30
    sleep_time = 0.8 # seconds between commands
    # print("Moving in 1 degree increments from 0 to 30")
    # for pos_deg in range(0, 31):  # 0 to 30
    #     set_motor_position(test_drv, pos_deg * (1/360), start_pos)
    #     time.sleep(sleep_time)
    #     # current_pos = get_motor_position(test_drv)
    #     # print(f"Target: {pos_deg}°, Actual: {(current_pos - start_pos) * 360:.2f}°")

    # Move back from 30 to 0 in 5 degree increments
    print("Moving in 5 degree increments from 30 back to 0")
    for pos_deg in range(30, -1, -5):  # 30 to 0, decrementing by 5
        set_motor_position(test_drv, pos_deg * (1/360), start_pos)
        time.sleep(sleep_time)
        # current_pos = get_motor_position(test_drv)
        # print(f"Target: {pos_deg}°, Actual: {(current_pos - start_pos) * 360:.2f}°")

    for pos_deg in range(30, -1, -5):  # 30 to 0, decrementing by 5
        set_motor_position(test_drv, pos_deg/360 - 1, start_pos)
        time.sleep(sleep_time)
        # current_pos = get_motor_position(test_drv)
        # print(f"Target: {pos_deg}°, Actual: {(current_pos - start_pos) * 360:.2f}°")


def M0_test():
    """
    Test function for M0 motor.
    """
    test_drv = odrv.axis0
    # Get the current position of the motor. This will be used to offset all position commands
    # start_pos = -0.07930180430412292
    start_pos = test_drv.encoder.pos_estimate
    print(f"Starting position of M0: {start_pos} revs")

    # Set the input mode to TRAP_TRAJ for M0
    set_axis_input_mode(test_drv, odrive.enums.INPUT_MODE_TRAP_TRAJ)

    # Set the trajectory limits for M0
    set_trap_traj_limits(test_drv, vel_limit=5.0, accel_limit=10.0)

    # Set the position and velocity gains for M0
    # set_axis_gains(test_drv, pos_gain=100.0, vel_gain=0.05, vel_integrator_gain=0.0)

    # Put the motor in closed loop control state
    test_drv.requested_state = odrive.enums.AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(1)  # Allow some time for the motor to stabilize

    # Move 2 revolutions forward
    print("Moving M0 forward by 2 revolutions")
    set_motor_position(test_drv, 2, start_pos)
    time.sleep(3)  # Allow some time for the motor to move

    # test_drv.requested_state = 1
    # start_time = time.time()
    # duration = 5.0  # seconds to run the loop
    # while time.time() - start_time < duration:
    #     # Get the current position of the motor as I move it by hand
    #     current_pos = get_motor_position(test_drv)
    #     print(f"Current position of M0: {current_pos} revs")
    #     time.sleep(0.1)
    # time.sleep(3)  # Allow some time for the motor to move
    # test_drv.requested_state = odrive.enums.AXIS_STATE_CLOSED_LOOP_CONTROL

    # Move 2 revolutions backward
    print("Moving M0 backward by 2 revolutions")
    set_motor_position(test_drv, -2, start_pos)
    time.sleep(3)  # Allow some time for the motor to move

# M0_test()
M1_test()

odrv.axis0.requested_state = odrive.enums.AXIS_STATE_IDLE
