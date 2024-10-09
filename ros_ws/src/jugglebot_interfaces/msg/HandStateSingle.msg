# A single hand state

std_msgs/String action       # What is the hand doing? (catch or throw)
geometry_msgs/Point pos      # Hand position at this state
geometry_msgs/Vector3 vel    # Hand velocity at this state
builtin_interfaces/Time time # Time that this state should occur