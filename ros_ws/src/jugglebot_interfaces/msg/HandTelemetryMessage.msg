# For reporting telemetry of the hand. This is useful as a standalone topic
# as the hand operates at a higher command rate than the legs

builtin_interfaces/Time timestamp  # Timestamp

float64 pos_cmd  # Commanded input_pos {rev}
float64 vel_ff_cmd # Commanded vel_ff {rev/s}
float64 tor_ff_cmd # Commanded torque_ff {Nm}

float64 pos_meas # Estimated position from the ODrive {rev}
float64 vel_meas # Estimated velocity from the ODrive {rev/s}
float64 iq_meas  # Measured current in the hand motor {A}
