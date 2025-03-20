#!/usr/bin/env python3
import time
import csv
import logging
import signal
import sys
import threading
from can_interface import CANInterface
import numpy as np

# -------------------------------
# Configuration parameters
# -------------------------------
MAX_POSITION_REV = 4.1       # Maximum position in revolutions
NUM_CYCLES = 5               # Number of cycles to run (full extension, back to home)
TIME_TO_WAIT = 2             # Time to wait after arriving at each target point

TRAP_VEL = 1.8                # Trap trajectory velocity limit (rev/s)
TRAP_ACC = 5.0               # Acceleration limit (rev/s^2)
TRAP_DEC = 5.0               # Deceleration limit (rev/s^2)

ODRIVE_NUM = 0               # The number of the ODrive that's being used to run the actuator
ACTUTATOR_NUM = 4            # The number of the actuator that's being used

SAMPLING_RATE = 1000          # Sampling rate for logging (Hz)

# -------------------------------
# Setup logger
# -------------------------------
logger = logging.getLogger("ActuatorLogger")
logger.setLevel(logging.INFO)  # Set to DEBUG for more verbose logging
ch = logging.StreamHandler()
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
ch.setFormatter(formatter)
logger.addHandler(ch)

# Global handles for cleanup
can_intf = None
cleaned_up = False  # Flag to prevent duplicate cleanup
running = True

# Global variables for logging threads
motor_logging_active = True
linear_slider_logging_active = True
motor_log = []      # List of tuples: (motor_timestamp, motor_position)
linear_slider_log = []    # List of tuples: (slider_timestamp, slider_position)
commanded_positions = []  # List of commanded positions: (timestamp, position)
can_fetch_active = True

# -------------------------------
# Functions
# -------------------------------

def cleanup():
    """Closes the serial port and CAN bus connection if open. Idempotent."""
    global can_intf, cleaned_up
    if cleaned_up:
        return

    # Ensure the axis is in IDLE mode before closing the connection
    try:
        if can_intf is not None:
            can_intf.set_requested_state(axis_id=ODRIVE_NUM, requested_state="IDLE")
            logger.info("Actuator set to IDLE mode.")
    except Exception as e:
        logger.error(f"Error setting IDLE mode: {e}")

    if can_intf is not None:
        try:
            can_intf.close()
            logger.info("CAN bus connection closed.")
        except Exception as e:
            logger.error(f"Error closing CAN bus connection: {e}")
    cleaned_up = True

def signal_handler(sig, frame):
    logger.info("Signal received. Exiting gracefully...")
    global motor_logging_active, linear_slider_logging_active, can_fetch_active
    motor_logging_active = False
    linear_slider_logging_active = False
    can_fetch_active = False
    cleanup()
    sys.exit(0)

# Register signal handlers for graceful exit
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

def motor_logging_thread():
    """
    Continuously logs motor positions
    """
    global motor_logging_active, motor_log, can_intf
    while motor_logging_active:
        ts = time.time()
        try:
            motor_states = can_intf.get_motor_states()
            motor_state = motor_states[ODRIVE_NUM]
            motor_pos = motor_state.pos_estimate
        except Exception as e:
            logger.error(f"Error reading motor state: {e}")
            motor_pos = None
        motor_log.append((ts, motor_pos))

        time.sleep(1/SAMPLING_RATE)

def linear_slider_logging_thread():
    """
    Continuously logs linear slider positions
    """
    global linear_slider_logging_active, linear_slider_log, can_intf
    while linear_slider_logging_active:
        ts = time.time()
        try:
            slider_pos = can_intf.get_linear_slider_position()
        except Exception as e:
            logger.error(f"Error reading slider position: {e}")
            slider_pos = None
        linear_slider_log.append((ts, slider_pos))

        time.sleep(1/SAMPLING_RATE)

def can_fetch():
    """
    Continuously fetches motor states from the CAN bus.
    """
    global can_fetch_active, can_intf
    while can_fetch_active:
        try:
            can_intf.fetch_messages()
        except Exception as e:
            logger.error(f"Error fetching motor state: {e}")

        time.sleep(0.0001)

def merge_and_save_logs(csv_filename="log_data.csv"):
    """
    Merges the motor and linear slider logs (by index) and saves to a CSV file with headings:
    "motor_timestamp", "motor_position", "slider_timestamp", "slider_position"
    """
    global motor_log, linear_slider_log, commanded_positions
    max_len = max(len(motor_log), len(linear_slider_log))
    with open(csv_filename, "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["motor_timestamp", "motor_position", "slider_timestamp", "slider_position", 
                         "commanded_timestamp", "commanded_position"])
        for i in range(max_len):
            # Extract the motor positions
            if i < len(motor_log):
                m_ts, m_val = motor_log[i]
            else:
                m_ts, m_val = "", ""
            # Extract the linear slider positions
            if i < len(linear_slider_log):
                s_ts, s_val = linear_slider_log[i]
            else:
                s_ts, s_val = "", ""
            # Extract the commanded positions
            if i < len(commanded_positions):
                c_ts, c_val = commanded_positions[i]
            else:
                c_ts, c_val = "", ""
            
            writer.writerow([m_ts, m_val, s_ts, s_val, c_ts, c_val])
    logger.info(f"Recorded data saved to {csv_filename}")

def move_and_await_arrival(setpoint, minimum_duration=0.5):
    """
    Commands the actuator to move to the specified setpoint and waits for the actuator to reach
    the target position. The minimum_duration parameter ensures the actuator at least starts moving
    before the function returns. (trajectory_done flag might otherwise be True immediately, from the 
    previous movement)
    """
    global can_intf
    try:
        can_intf.set_control_mode(axis_id=ODRIVE_NUM, input_mode="TRAP_TRAJ", control_mode="POSITION_CONTROL")
        can_intf.set_requested_state(axis_id=ODRIVE_NUM, requested_state="CLOSED_LOOP_CONTROL")
        time.sleep(0.5)  # Allow time for mode switch
        can_intf.send_position_target(axis_id=ODRIVE_NUM, setpoint=setpoint)

        # Wait for the actuator to reach the target position
        has_arrived = False
        start_time = time.time()
        while not (has_arrived and time.time() - start_time >= minimum_duration):
            motor_states = can_intf.get_motor_states()
            motor_state = motor_states[ODRIVE_NUM]
            has_arrived = motor_state.trajectory_done
            time.sleep(0.01)
        logger.info(f"Actuator {ODRIVE_NUM} has reached the target position.")
    except Exception as e:
        logger.error(f"Failed to command move to position: {e}")

def move_sinusoidal(min_position, max_position, duration_per_cycle=5.0, num_cycles=2):
    """
    Commands the actuator to move sinusoidally between the specified min and max positions
    """
    global can_intf, commanded_positions
    try:
        can_intf.set_control_mode(axis_id=ODRIVE_NUM, input_mode="TRAP_TRAJ", control_mode="POSITION_CONTROL")
        can_intf.set_requested_state(axis_id=ODRIVE_NUM, requested_state="CLOSED_LOOP_CONTROL")
        time.sleep(0.1)  # Allow time for mode switch
        start_time = time.time()
        cycle_num = 0
        while cycle_num < num_cycles:
            t = time.time() - start_time
            setpoint = min_position + (max_position - min_position) * 0.5 * (1 - np.cos(2*np.pi*t/duration_per_cycle))
            can_intf.send_position_target(axis_id=ODRIVE_NUM, setpoint=setpoint)

            # Record these commanded positions
            commanded_positions.append((time.time(), setpoint))
            time.sleep(0.01)
            if t > duration_per_cycle:
                cycle_num += 1
                start_time = time.time()

        logger.info(f"Sinusoidal movement completed.")
    except Exception as e:
        logger.error(f"Failed to command sinusoidal movement: {e}")

def move_to_stages(num_cycles=2):
    """
    Commands the actuator to move to a series of predefined stages.
    Between each stage, the actuator will first fully extend.
    At each stage, the actuator will wait for TIME_TO_WAIT seconds.
    """
    global can_intf
    stages = [value * 4.1 for value in [0.25, 0.5, 0.75, 0]] # Stages to move to (in revolutions)
    cycle = 0
    while cycle < num_cycles:
        for stage in stages:
            move_and_await_arrival(setpoint=MAX_POSITION_REV)
            time.sleep(TIME_TO_WAIT)

            logger.info(f"Commanding actuator to move to stage {stage}...")
            move_and_await_arrival(setpoint=stage)
            logger.info(f"Waiting for {TIME_TO_WAIT} seconds...")
            time.sleep(TIME_TO_WAIT)
        
        cycle += 1

def run_test():
    global can_intf, motor_logging_active, can_fetch_active, linear_slider_logging_active
    # -------------------------------
    # Initialize CAN bus connections
    # -------------------------------

    try:
        # Initialize CAN interface instance
        can_intf = CANInterface(logger, odrive_num_being_tested=ODRIVE_NUM)
        logger.info("CAN interface initialized.")
    except Exception as e:
        logger.error(f"Failed to initialize CAN interface: {e}")
        return

    # -------------------------------
    # Start background threads (logging and CAN message fetching)
    # -------------------------------
    motor_thread = threading.Thread(target=motor_logging_thread, daemon=True)
    slider_thread = threading.Thread(target=linear_slider_logging_thread, daemon=True)
    can_fetch_thread = threading.Thread(target=can_fetch, daemon=True)
    can_fetch_thread.start()
    logger.info("Background logging threads started.")

    # -------------------------------
    # Main procedure
    # -------------------------------
    try:
        # Step 0: Run the encoder search on the actuator
        # logger.info(f"Running encoder search on actuator {ODRIVE_NUM}...")
        # if not can_intf.run_encoder_search(ODRIVE_NUM):
        #     logger.error("Encoder search failed. Exiting.")
        #     return
        # logger.info("Encoder search completed successfully.")
        
        # Step 1: Home the actuator
        logger.info(f"Starting homing procedure for actuator {ODRIVE_NUM}...")
        if not can_intf.home_robot(ODRIVE_NUM, reset_zero_position=True):
            logger.error("Homing failed. Exiting.")
            return
        logger.info("Homing completed successfully.")


        # Step 2: Wait for TIME_TO_WAIT seconds to get a clear starting point
        logger.info(f"Waiting for {TIME_TO_WAIT} seconds...")
        time.sleep(TIME_TO_WAIT/3.0)

        # Start logging threads. Starting now ensures that both logs will start at position '0'
        motor_thread.start()
        slider_thread.start()

        time.sleep(TIME_TO_WAIT * 2/3.0)

        # Step 3: Set trajectory limits and command the actuator to move to max position
        logger.info("Setting trap trajectory limits...")
        try:
            can_intf._set_trap_traj_vel_acc(axis_id=ODRIVE_NUM,
                                            vel_limit=TRAP_VEL,
                                            acc_limit=TRAP_ACC,
                                            dec_limit=TRAP_DEC)
        except Exception as e:
            logger.error(f"Failed to set trajectory limits: {e}")
            return
        
        cycle_num = 0

        while cycle_num < NUM_CYCLES:
            logger.info(f"Commanding actuator {ODRIVE_NUM} to move to max position: {MAX_POSITION_REV} revs.")
            try:
                move_and_await_arrival(setpoint=MAX_POSITION_REV)
                logger.info(f"Waiting for {TIME_TO_WAIT} seconds...")
                time.sleep(TIME_TO_WAIT)
                            
            except Exception as e:
                logger.error(f"Failed to command move to max position: {e}")
                return

            # Step 4: Command the actuator to return to home (0 rev)
            logger.info(f"Commanding actuator {ODRIVE_NUM} to home, without resetting encoder position.")
            try:
                can_intf.home_robot(ODRIVE_NUM, reset_zero_position=False)
                logger.info(f"Waiting for {TIME_TO_WAIT} seconds...")
                time.sleep(TIME_TO_WAIT)
                
            except Exception as e:
                logger.error(f"Failed to command return-to-home: {e}")
                return

            cycle_num += 1
            logger.info(f"Cycle {cycle_num} completed.")

        # Step 5: Command the actuator to move sinusoidally between min and max
        sinusoid_min = 0.1
        sinusoid_max = 4.1
        logger.info(f"Commanding actuator to move sinusoidally between {sinusoid_min} and {sinusoid_max} revs.")
        try:
            move_and_await_arrival(setpoint=sinusoid_min)
            move_sinusoidal(min_position=sinusoid_min, max_position=sinusoid_max)
        except Exception as e:
            logger.error(f"Failed to command sinusoidal movement: {e}")
            return
        
        # Step 6: Command the actuator to move to a series of stages
        move_to_stages(num_cycles=2)

        # Step 7: Do the sinusoidal and 'stages' movements again, after waiting 5 seconds.
        # During this time, I will place a mass on the end of the actuator.
        # We also need to reduce the max speeds of the actuator to prevent damage.
        # try:
        #     logger.info("Waiting for 5 seconds before repeating sinusoidal and stage movements...")
        #     time.sleep(5)
        #     logger.info("Reducing trap trajectory limits...")
        #     can_intf._set_trap_traj_vel_acc(axis_id=ODRIVE_NUM,
        #                                     vel_limit=1.0,
        #                                     acc_limit=2.0,
        #                                     dec_limit=2.0)
            
        #     logger.info(f"Commanding actuator to move sinusoidally between {sinusoid_min} and {sinusoid_max} revs.")
        #     move_and_await_arrival(setpoint=sinusoid_min)
        #     move_sinusoidal(min_position=sinusoid_min, max_position=sinusoid_max, duration_per_cycle=10.0)

        #     # Command the actuator to move to a series of stages
        #     move_to_stages(num_cycles=2)
        # except Exception as e:
        #     logger.error(f"Failed to command weighted sinusoidal and stage movements: {e}")
        #     return

    except Exception as main_e:
        logger.error(f"An error occurred: {main_e}")

    finally:
        # Signal logging threads to stop
        motor_logging_active = False
        can_fetch_active = False
        motor_thread.join(timeout=1)
        slider_thread.join(timeout=1)
        can_fetch_thread.join(timeout=1)
        # Save logs to CSV before cleaning up
        merge_and_save_logs(csv_filename=f"{ACTUTATOR_NUM}_data.csv")
        cleanup()

def read_from_slider():
    """If just wanting to get a real-time readout of the linear slider position"""
    global can_intf, running
    
    try:
        # Initialize CAN interface
        can_intf = CANInterface(logger, odrive_num_being_tested=0)
        logger.info("CAN interface initialized")
        
        # Start a message fetching thread
        fetch_thread = threading.Thread(target=can_fetch, daemon=True)
        fetch_thread.start()
        
        # Main loop - read slider position every 10ms
        while running:
            try:
                slider_pos = can_intf.get_linear_slider_position()
                logger.info(f"Linear slider position: {slider_pos:.3f} mm")
            except Exception as e:
                logger.error(f"Error reading slider position: {e}")
            
            time.sleep(0.01)  # 10ms delay
            
    except Exception as e:
        logger.error(f"An error occurred: {e}")
    finally:
        if can_intf:
            can_intf.close()
        running = False

if __name__ == '__main__':
    # Choose one or the other. Comment out the one you're not using.
    
    # run_test()
    read_from_slider()


