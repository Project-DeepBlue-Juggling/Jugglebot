# For relaying the predicted landing point of a ball
# Where 'landing' is when the ball crosses a given z-plane

std_msgs/Header header # Standard header with timestamp and frame_id
geometry_msgs/Point landing_position # Predicted position where the ball will cross the chosen z-plane
geometry_msgs/Vector3 landing_velocity # Predicted velocity that the ball will have when crossing the chosen z-plane
float32 time_to_land   # Time until the ball crosses the z-plane {sec}
