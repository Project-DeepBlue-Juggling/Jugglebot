# Result of Ball Butler mocap calibration (circle-fit on 5 fiducial markers).
# Published as a latched topic on bb/calibration_result after each successful calibration.

geometry_msgs/Point position_mm    # BB global position [x, y, z] (mm) — yaw axis intersection
float64 yaw_offset_rad             # Offset from BB local yaw to global frame (rad)
float64 yaw_offset_std_deg         # Uncertainty on yaw offset (degrees)
float64 axis_tilt_deg              # Rotation axis tilt from vertical (degrees)
bool success                       # True if calibration succeeded
string message                     # Human-readable status / error description
