# Dynamic target command from the catch coordinator.
# Consumed by trajectory_node as a timed catch target while the catch-armed latch
# (trajectory/arm_catch) is raised (the RELOAD action's reactive-catch trigger);
# historically forwarded to the MPC process via mpc_bridge_node / IPC.

geometry_msgs/Point target_pos         # [x, y, z] platform offset from home (mm)
geometry_msgs/Quaternion target_quat   # Orientation (w, x, y, z)
geometry_msgs/Vector3 target_vel       # [vx, vy, vz] linear velocity at target (mm/s)
float64 arrival_time                   # Absolute arrival time (perf_counter seconds)
