# For reporting telemetry of the hand. This is useful as a standalone topic
# as the hand operates at a higher command rate than the legs

builtin_interfaces/Time timestamp  # Timestamp

float64 pos_cmd  # Commanded input_pos {rev}
float64 vel_ff_cmd # Commanded vel_ff {rev/s}
float64 tor_ff_cmd # Commanded torque_ff {Nm}

float64 pos_meas # Estimated position from the ODrive {rev}
float64 vel_meas # Estimated velocity from the ODrive {rev/s}
float64 iq_meas  # Measured current in the hand motor {A}

# Hand ball-present sensor (switch on the hand ODrive's G02, polled by the
# can-bridge over an SDO pair). TRI-STATE, not a bare bool: ball_held is
# meaningless unless ball_held_valid is true — boot, staleness and an
# un-anchored bridge clock are all UNKNOWN, never "no ball".
# See plans/archived/hand-ball-sensor.md § Architecture (normative).
bool ball_held        # Debounced verdict (only meaningful while ball_held_valid)
bool ball_held_raw    # Raw per-sample bit (carry-flicker diagnostics)
bool ball_held_valid  # Fresh + endpoint-verified + link-fresh; false = UNKNOWN
builtin_interfaces/Time ball_held_stamp  # Bridge clock of the last good reply: wall-epoch
                                         # only while ball_held_valid; bridge-boot-relative
                                         # before the wall anchor lands (TIME_SYNCED clear)

# BUG-134: bridge wall clock at the can-bridge's telemetry tick that produced
# pos_meas/vel_meas (same clock and convention as /leg_cmd_executed and the Platform
# IMU). `timestamp` is unchanged: the Jetson ROS clock at PUBLISH, 12 to 14 ms later.
# Valid only when the bridge reports TIME_SYNCED and the stamp is within 1 s of the
# Jetson clock; when false this field is zero and readers use `timestamp`. The
# encoder sample itself is 0 to 10 ms older than the tick. APPENDED at the end.
builtin_interfaces/Time bridge_stamp
bool bridge_stamp_valid
