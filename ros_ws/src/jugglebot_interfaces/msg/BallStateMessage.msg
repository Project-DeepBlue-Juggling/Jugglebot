# For structuring the state of the ball throughout its path

geometry_msgs/PoseStamped[] path        # The path of the ball (while in the air). Must be PoseStamped to match other paths
geometry_msgs/PointStamped current_pos  # The current position of the ball (with a timestamp for good measure)