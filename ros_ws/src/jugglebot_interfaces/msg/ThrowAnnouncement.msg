# Announcement published by throwers (ball_butler, jugglebot, etc.) before throw
# Used by ball_prediction_node to match detected projectiles to known throws

std_msgs/Header header
string thrower_name                    # Source: "ball_butler", "jugglebot", etc.
geometry_msgs/Point initial_position   # Position at throw time (mm)
geometry_msgs/Vector3 initial_velocity # Velocity at throw time (mm/s)
builtin_interfaces/Time throw_time     # When the throw will occur (use header.stamp if immediate)
float64 predicted_tof_sec              # Predicted time of flight (sec)
geometry_msgs/Point landing_position   # Predicted position at landing plane (mm, global frame)
geometry_msgs/Vector3 landing_velocity # Predicted velocity at landing plane (mm/s, global frame)
builtin_interfaces/Time landing_time   # Predicted ROS2 Time when ball crosses landing plane
