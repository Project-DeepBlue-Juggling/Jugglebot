# The state of a single motor.
# Data types have been copied from what ODrive uses

# Errors
uint32 active_errors  # Any active errors
uint32 disarm_reason  # Is the axis disarmed? Why?

# General info
uint8 current_state     # Current state of the ODrive (eg. 1 is IDLE)
uint8 procedure_result  # Usually 0 (SUCCESS), 1 (BUSY) or 2 (CANCELLED)
bool trajectory_done    # Done with the last command?

# Position-related
float32 pos_estimate  # Estimated position of the motor {rev}
float32 vel_estimate  # Estimated velocity of the motor {rev/s}

# Motor current
float32 iq_setpoint  # {A}
float32 iq_measured  # {A}

# Temperatures
float32 fet_temp    # Temp of the on-board MOSFETs {deg C}
float32 motor_temp  # Temp of the motor (if a thermistor is installed) {deg C}

# Bus voltage, current
float32 bus_voltage  # Voltage on the bus as seen by this ODrive {V}
float32 bus_current  # Current sourced/sunk by this ODrive {A}
