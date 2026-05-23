# Result of correlating a catching-cone catch event with a throw announcement.
# Published by catch_correlation_node on cone/timing_result; consumed by the GUI.

std_msgs/Header header
bool matched                                     # True if a throw was matched to this catch
string thrower_name                              # Matched throw source ("ball_butler", "jugglebot")
builtin_interfaces/Time predicted_landing_time   # Predicted landing time of the matched throw
builtin_interfaces/Time actual_catch_time        # Wall-clock instant the cone detected the catch
float64 timing_error_ms                          # actual - predicted; >0 = ball arrived late
