# For reporting several tracked marker positions and residuals

MocapDataSingle[] markers # The marker(s) being tracked.
bool aligned              # True when the mocap base body is within alignment thresholds

# The QTM frame time this packet of markers was captured at, expressed on the
# ROS clock through MocapInterface's smoothed QTM<->ROS offset
# (qtm_timestamp_to_ros_ns / latest_frame_ros_ns in mocap_interface.py). Zero
# (sec = nanosec = 0) when the offset has not been established yet (startup,
# or just after a QTM restart resets the sync).
#
# Measured 2026-09-20 on bag ~/Desktop/rosbags/2026-09-18_16-16-17: stamping
# frames at CALLBACK time (ball_tracker_node.get_clock().now()) instead of at
# the source lets sitting-load jitter into the ballistic fit's sample times —
# at 3 m/s a 10 ms error is 30 mm, past the fit's 12 mm residual gate, so the
# fit never converges (live /balls carried a landing for only 33/60 flights
# vs. 54/60 replaying the same bag with receipt times). This field lets the
# tracker fit against QTM's own clock instead of the callback's.
builtin_interfaces/Time stamp