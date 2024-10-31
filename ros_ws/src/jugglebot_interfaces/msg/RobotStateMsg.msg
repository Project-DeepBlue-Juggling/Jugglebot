# To communicate and record the current state of the robot.
# Serves as a way to broadcast the current robot state

bool is_homed                             # Have the ODrives been homed since the last bootup?

bool levelling_complete                   # Has the platform been levelled since the last bootup?
float32[] pose_offset_rad                 # Pose offset in radians ([tiltX, tiltY])
geometry_msgs/Quaternion pose_offset_quat # Pose offset as a quaternion

string error                              # Any errors to report?

# Pose offset is given as both rad and quat because the rad format is half the data and hence can more easily be sent to the
# Teensy over the CAN bus.  




# Can this topic maybe be entirely done away with????