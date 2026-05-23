# Catch event from the instrumented catching cone — a precisely-timed
# ball-landing impact detected by the cone's piezo contact sensor.
# Decoded from a CATCH_EVENT CAN frame (ID 0x7E0) by can_node.

std_msgs/Header header
builtin_interfaces/Time catch_time   # Wall-clock instant of the impact
uint8 sequence                       # Monotonic per-event counter (wraps at 256)
bool time_synced                     # False => catch_time is NOT comparable to wall time
bool retrigger_suppressed            # Ring/bounce edges seen within the post-impact report-delay window
                                     # (~30 ms). Later bounces are silently dropped by the ISR refractory.
