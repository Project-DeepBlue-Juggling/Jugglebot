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
# See plans/active/hand-ball-sensor.md § Architecture (normative).
bool ball_held        # Debounced verdict (only meaningful while ball_held_valid)
bool ball_held_raw    # Raw per-sample bit (carry-flicker diagnostics)
bool ball_held_valid  # Fresh + endpoint-verified + link-fresh; false = UNKNOWN
builtin_interfaces/Time ball_held_stamp  # Bridge clock of the last good reply: wall-epoch
                                         # only while ball_held_valid; bridge-boot-relative
                                         # before the wall anchor lands (TIME_SYNCED clear)
