# For sending a single step of the hand trajectory

builtin_interfaces/Time stamp  # Timestamp
bool first_command # Whether this command is the start of the trajectory
bool last_command # Whether this command is the last one
float64 time # Time of command
float64 pos  # Position command
float64 vel  # Velocity command
float64 tor  # Torque command