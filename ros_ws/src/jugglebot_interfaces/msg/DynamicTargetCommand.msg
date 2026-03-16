# Dynamic target command from catch coordinator to motion bridge.
# The bridge forwards this to the control process via IPC.

geometry_msgs/Point target_pos         # [x, y, z] platform offset from home (mm)
geometry_msgs/Quaternion target_quat   # Orientation (w, x, y, z)
geometry_msgs/Vector3 target_vel       # [vx, vy, vz] linear velocity at target (mm/s)
float64 arrival_time                   # Absolute arrival time (perf_counter seconds)
