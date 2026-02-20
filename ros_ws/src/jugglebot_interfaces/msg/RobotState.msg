# For distributing the state of the entire robot all at once

builtin_interfaces/Time timestamp
MotorStateSingle[] motor_states # states of each motor individually
string[] error                    # Human-readable error descriptions (for logging/display)

# Typed error flags — orchestrator uses these for error classification instead of parsing strings.
# Set by the CAN node from its internal motor state tracker flags.
bool has_fatal_odrive_error       # Fatal ODrive error (axis disarmed in CLOSED_LOOP, exceeded reset attempts, etc.)
bool has_fatal_can_error          # CAN bus disconnection or restoration failure
bool has_undervoltage             # DC bus undervoltage detected (e.g., E-stop hit)

bool encoder_search_complete # Has the encoder search been run on every leg?
bool is_homed                # Have the ODrives been homed since the last bootup?
bool levelling_complete      # Has the platform been levelled since the last bootup?

float32[] pose_offset_rad                 # Pose offset in radians ([tiltX, tiltY])
geometry_msgs/Quaternion pose_offset_quat # Pose offset as a quaternion


# Pose offset is given as both rad and quat because the rad format is half the data and hence can more easily be sent to the
# Teensy over the CAN bus.