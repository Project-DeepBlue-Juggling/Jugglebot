# Comprehensive state of a single tracked ball
# Published as part of BallStateArray on /balls topic

std_msgs/Header header
uint32 id                              # Unique monotonic ball ID (never reused in a session)

# Lifecycle and tracking
uint8 status                           # BallStatus: 0=TO_BE_THROWN, 1=IN_FLIGHT, 2=CAUGHT, 3=DROPPED, 4=UNKNOWN
uint8 tracking                         # TrackingConfidence: 0=ANNOUNCED, 1=CONFIRMED
string source                          # Source of ball: "human_throw", "ball_butler", etc.
string destination                     # Robot this ball is heading to (empty = unassigned)

# Current state (from Kalman Filter estimate, or predicted from launch state if not yet matched)
geometry_msgs/Point position           # Current estimated position (mm)
geometry_msgs/Vector3 velocity         # Current estimated velocity (mm/s)

# Predicted landing state (where ball crosses target z-plane, or default z-plane if no target)
geometry_msgs/Point landing_position   # Predicted (x, y, z) at landing (mm)
geometry_msgs/Vector3 landing_velocity # Predicted velocity at landing (mm/s)
builtin_interfaces/Time time_at_land   # Predicted ROS2 Time when ball will land
