# Comprehensive state of a single tracked ball
# Published as part of BallStateArray on /balls topic

std_msgs/Header header
uint32 id                              # Unique monotonic ball ID (never reused in a session)

# Lifecycle and tracking
uint8 status                           # BallStatus: 0=TO_BE_THROWN, 1=IN_FLIGHT, 2=CAUGHT, 3=DROPPED, 4=UNKNOWN
uint8 tracking                         # TrackingConfidence: 0=ANNOUNCED, 1=CONFIRMED
string source                          # Source of ball: "human_throw", "ball_butler", etc.
string destination                     # Robot this ball is heading to (empty = unassigned)

# Identity: the announcement this track was minted for (2026-10-04). The tracker mints ONE track
# per ThrowAnnouncement and copies its throw_time here verbatim; `source` is that announcement's
# thrower_name (empty -> "ball_butler"). (source, throw_time) is the key a consumer correlates a
# scheduled release on (jugglebot/ball_possession.py::match_announced_track) -- the only exact
# one, since mocap markers carry no identity. Zero for an un-announced (parabolic) detection.
builtin_interfaces/Time throw_time     # Announced release instant (ROS time); 0 = not announced

# Current state (from Kalman Filter estimate, or predicted from launch state if not yet matched)
geometry_msgs/Point position           # Current estimated position (mm)
geometry_msgs/Vector3 velocity         # Current estimated velocity (mm/s)

# Predicted landing state (where ball crosses target z-plane, or default z-plane if no target)
geometry_msgs/Point landing_position   # Predicted (x, y, z) at landing (mm)
geometry_msgs/Vector3 landing_velocity # Predicted velocity at landing (mm/s)
builtin_interfaces/Time time_at_land   # Predicted ROS2 Time when ball will land
bool landing_from_fit                  # True: landing came from the converged gravity-fixed
                                       # batch fit; False: Kalman/announcement extrapolation
