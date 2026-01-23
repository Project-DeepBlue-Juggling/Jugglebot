# A single target that balls can be thrown to
# Published by target_tracker_node on /targets topic

std_msgs/Header header
string id                              # Unique target ID (e.g., "catching_cone", "jugglebot_catch")
geometry_msgs/Point position           # Current position of the target (mm)
