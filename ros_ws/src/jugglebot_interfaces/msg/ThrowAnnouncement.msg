# Announcement published by throwers (ball_butler, jugglebot, etc.) before throw
# Used by ball_prediction_node to match detected projectiles to known throws

std_msgs/Header header
string thrower_name                    # Source: "ball_butler", "jugglebot", etc.
geometry_msgs/Point initial_position   # Position at throw time (mm)
geometry_msgs/Vector3 initial_velocity # Velocity at throw time (mm/s)
string target_id                       # Target this throw is aimed at (empty if none)
builtin_interfaces/Time throw_time     # When the throw will occur (use header.stamp if immediate)
