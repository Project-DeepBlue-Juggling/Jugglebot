# For distributing the state of the entire robot all at once

builtin_interfaces/Time timestamp
MotorStateSingle[] motor_states # states of each motor individually
string[] error                    # Any current errors with the robot as a whole? (eg. power/CAN issues)

bool encoder_search_complete # Has the encoder search been run on every leg?
bool is_homed                # Have the ODrives been homed since the last bootup?
bool levelling_complete      # Has the platform been levelled since the last bootup?

float32[] pose_offset_rad                 # Pose offset in radians ([tiltX, tiltY])
geometry_msgs/Quaternion pose_offset_quat # Pose offset as a quaternion


# Pose offset is given as both rad and quat because the rad format is half the data and hence can more easily be sent to the
# Teensy over the CAN bus.