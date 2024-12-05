# For relaying the predicted landing point of a ball
# Where 'landing' is when the ball crosses a given z-plane

std_msgs/Header header # Standard header with timestamp and frame_id
geometry_msgs/Point position # Predicted position where the ball will cross the chosen z-plane
float32 time_to_land   # Time until the ball crosses the z-plane {sec}
int32 marker_id       # Unique ID of the tracked marker