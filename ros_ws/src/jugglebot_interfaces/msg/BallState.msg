# Comprehensive state of a single tracked ball in flight
# Published as part of BallStateArray on /balls topic
# Only balls confirmed to be in projectile motion are published

std_msgs/Header header
uint32 id                              # Unique monotonic ball ID (never reused in a session)

# Source information
string source                          # Source of ball: "human_throw", "ball_butler", etc.

# Current state (from Kalman Filter estimate, or predicted from launch state if not yet matched)
geometry_msgs/Point position           # Current estimated position (mm)
geometry_msgs/Vector3 velocity         # Current estimated velocity (mm/s)

# Target information
string target_id                       # Assigned target ID (empty string if none/unknown)
geometry_msgs/Point target_position    # Target position at time of assignment (mm)

# Predicted landing state (where ball crosses target z-plane, or default z-plane if no target)
geometry_msgs/Point landing_position   # Predicted (x, y, z) at landing (mm)
geometry_msgs/Vector3 landing_velocity # Predicted velocity at landing (mm/s)
builtin_interfaces/Time time_at_land   # Predicted ROS2 Time when ball will land
