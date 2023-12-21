# Custom message for relaying the state of the robot
# The robot can be homed, empty or holding.
# If empty or holding, the 

bool is_homed
string status  # eg. 'holding', 'empty', 'waiting'
int32 next_ball_id
geometry_msgs/Point next_ball_position
geometry_msgs/Vector3 next_ball_velocity
builtin_interfaces/Time next_action_time