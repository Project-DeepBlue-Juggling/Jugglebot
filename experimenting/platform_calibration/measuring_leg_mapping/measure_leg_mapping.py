#!/usr/bin/env python3
import serial
import time
import csv
import logging
import signal
import sys
import threading
from can_interface import CANInterface

# -------------------------------
# Configuration parameters
# -------------------------------
SERIAL_PORT = '/dev/ttyACM0'   # Change as needed
BAUD_RATE = 115200
MAX_POSITION_REV = 4.1       # Maximum position in revolutions
NUM_CYCLES = 5               # Number of cycles to run (full extension, back to home)
TIME_TO_WAIT = 3             # Time to wait after arriving at each target point
SAMPLING_INTERVAL = 0.01     # 100 Hz sampling (0.01 seconds), for motor 
TRAP_VEL = 1.0               # Trap trajectory velocity limit (rev/s)
TRAP_ACC = 2.0               # Acceleration limit (rev/s^2)
TRAP_DEC = 2.0               # Deceleration limit (rev/s^2)

ODRIVE_NUM = 5               # The number of the ODrive that's being used to run the actuator
ACTUTATOR_NUM = 0            # The number of the actuator that's being used

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
ser = None
can_intf = None
cleaned_up = False  # Flag to prevent duplicate cleanup


# Global variables for logging threads
motor_logging_active = True
arduino_logging_active = True
motor_log = []      # List of tuples: (motor_timestamp, motor_position)
arduino_log = []    # List of tuples: (arduino_timestamp, arduino_position)
can_fetch_active = True

# -------------------------------
# Functions
# -------------------------------

def cleanup():
    """Closes the serial port and CAN bus connection if open. Idempotent."""
    global ser, can_intf, cleaned_up
    if cleaned_up:
        return

    # Ensure the axis is in IDLE mode before closing the connection
    try:
        if can_intf is not None:
            can_intf.set_requested_state(axis_id=ODRIVE_NUM, requested_state="IDLE")
            logger.info("All axes set to IDLE mode.")
    except Exception as e:
        logger.error(f"Error setting IDLE mode: {e}")

    logger.info("Cleaning up: closing serial and CAN bus connections...")
    if ser is not None:
        try:
            ser.flush()
            if ser.is_open:
                ser.close()
                logger.info("Serial connection closed.")
        except Exception as e:
            logger.error(f"Error closing serial connection: {e}")
    if can_intf is not None:
        try:
            can_intf.close()
            logger.info("CAN bus connection closed.")
        except Exception as e:
            logger.error(f"Error closing CAN bus connection: {e}")
    cleaned_up = True

def signal_handler(sig, frame):
    logger.info("Signal received. Exiting gracefully...")
    global motor_logging_active, arduino_logging_active, can_fetch_active
    motor_logging_active = False
    arduino_logging_active = False
    can_fetch_active = False
    cleanup()
    sys.exit(0)

# Register signal handlers for graceful exit
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

def robust_readline(ser, timeout=0.1, max_chars=1024, do_logging=False):
    """
    Reads from the serial port one byte at a time until a newline is encountered,
    a maximum number of characters is reached, or a timeout occurs.
    """
    start_time = time.time()
    raw_bytes = bytearray()
    while True:
        if ser.in_waiting > 0:
            c = ser.read(1)
            if c:
                raw_bytes += c
                # Break if newline is received or maximum length reached.
                if c == b'\n' or len(raw_bytes) >= max_chars:
                    break
        if time.time() - start_time > timeout:
            break
    line = raw_bytes.decode('utf-8', errors='replace').strip()
    if do_logging:
        if raw_bytes:
            logger.info(f"Robust read: raw data = {raw_bytes}")
        else:
            logger.info("Robust read: no data read.")
    return line

def motor_logging_thread():
    """
    Continuously logs motor positions with timestamps.
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
        time.sleep(SAMPLING_INTERVAL)

def arduino_logging_thread():
    """
    Continuously logs Arduino readings with timestamps.
    """
    global arduino_logging_active, arduino_log, ser
    while arduino_logging_active:
        ts = time.time()
        line = robust_readline(ser, timeout=0.01)
        if line:
            try:
                value = float(line)
            except ValueError:
                logger.warning(f"Invalid Arduino reading: '{line}'")
                value = None
            arduino_log.append((ts, value))
        # A short sleep to yield CPU time
        time.sleep(0.01)

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

        time.sleep(0.01)

def merge_and_save_logs(csv_filename="log_data.csv"):
    """
    Merges the motor and Arduino logs (by index) and saves to a CSV file with headings:
    "motor_timestamp", "motor_position", "arduino_timestamp", "arduino_position"
    """
    global motor_log, arduino_log
    max_len = max(len(motor_log), len(arduino_log))
    with open(csv_filename, "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["motor_timestamp", "motor_position", "arduino_timestamp", "arduino_position"])
        for i in range(max_len):
            if i < len(motor_log):
                m_ts, m_val = motor_log[i]
            else:
                m_ts, m_val = "", ""
            if i < len(arduino_log):
                a_ts, a_val = arduino_log[i]
            else:
                a_ts, a_val = "", ""
            writer.writerow([m_ts, m_val, a_ts, a_val])
    logger.info(f"Recorded data saved to {csv_filename}")

def move_and_await_arrival(axis_id, setpoint, minimum_duration=0.5):
    """
    Commands the actuator to move to the specified setpoint and waits for the actuator to reach
    the target position. The minimum_duration parameter ensures the actuator at least starts moving
    before the function returns. (trajectory_done flag might otherwise be True immediately, from the 
    previous movement)
    """
    global can_intf
    try:
        can_intf.set_control_mode(axis_id=axis_id, input_mode="TRAP_TRAJ", control_mode="POSITION_CONTROL")
        can_intf.set_requested_state(axis_id=axis_id, requested_state="CLOSED_LOOP_CONTROL")
        time.sleep(0.1)  # Allow time for mode switch
        can_intf.send_position_target(axis_id=axis_id, setpoint=setpoint)

        # Wait for the actuator to reach the target position
        has_arrived = False
        start_time = time.time()
        while not (has_arrived and time.time() - start_time >= minimum_duration):
            motor_states = can_intf.get_motor_states()
            motor_state = motor_states[axis_id]
            has_arrived = motor_state.trajectory_done
            time.sleep(0.01)
        logger.info(f"Actuator {axis_id} has reached the target position.")
    except Exception as e:
        logger.error(f"Failed to command move to position: {e}")

def main():

    global ser, can_intf, motor_logging_active, can_fetch_active, arduino_logging_active
    # -------------------------------
    # Initialize serial and CAN bus connections
    # -------------------------------
    try:
        # Initialize serial connection to Arduino/Teensy with a short timeout.
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        # Disable DTR to prevent Arduino auto-reset when opening the port.
        ser.setDTR(False)
        # A short delay to let the serial connection settle.
        time.sleep(0.1)
        # Flush any stale data in the buffer.
        ser.reset_input_buffer()
        logger.info(f"Serial connection established on {SERIAL_PORT} at {BAUD_RATE} baud.")
    except Exception as e:
        logger.error(f"Failed to open serial port: {e}")
        arduino_logging_active = False
        # return

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
    arduino_thread = threading.Thread(target=arduino_logging_thread, daemon=True)
    can_fetch_thread = threading.Thread(target=can_fetch, daemon=True)
    motor_thread.start()
    arduino_thread.start()
    can_fetch_thread.start()
    logger.info("Background logging threads started.")

    # -------------------------------
    # Main procedure
    # -------------------------------
    try:
        # Step 0: Run the encoder search on the actuator
        # logger.info(f"Running encoder search on actuator {ODRIVE_NUM}...")
        # if not can_intf.run_encoder_search([ODRIVE_NUM]):
        #     logger.error("Encoder search failed. Exiting.")
        #     return
        # logger.info("Encoder search completed successfully.")

        # Step 1: Home the actuator
        logger.info(f"Starting homing procedure for actuator {ODRIVE_NUM}...")
        if not can_intf.home_robot(ODRIVE_NUM, reset_zero_position=False):
            logger.error("Homing failed. Exiting.")
            return
        logger.info("Homing completed successfully.")

        # Step 2: Wait for 5 seconds to get a clear starting point
        logger.info(f"Waiting for {TIME_TO_WAIT} seconds...")
        time.sleep(TIME_TO_WAIT)

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
                move_and_await_arrival(axis_id=ODRIVE_NUM, setpoint=MAX_POSITION_REV)
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

    except Exception as main_e:
        logger.error(f"An error occurred: {main_e}")

    finally:
        # Signal logging threads to stop
        motor_logging_active = False
        can_fetch_active = False
        motor_thread.join(timeout=1)
        arduino_thread.join(timeout=1)
        can_fetch_thread.join(timeout=1)
        # Save logs to CSV before cleaning up
        merge_and_save_logs(csv_filename=f"{ACTUTATOR_NUM}_test_data.csv")
        cleanup()

if __name__ == '__main__':
    main()
