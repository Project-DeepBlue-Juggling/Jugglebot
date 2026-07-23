# Dynamic target command from the catch coordinator.
# Consumed by trajectory_node as a timed catch target while the catch-armed latch
# (trajectory/arm_catch) is raised (the RELOAD action's reactive-catch trigger);
# historically forwarded to the MPC process via mpc_bridge_node / IPC.
#
# FRAME (normative, hardware-verified 2026-07-23): target_pos is STOW-relative —
# z = 0 is the stow plane (GEOM_INITIAL_HEIGHT above the floor), z ~= 170 is the
# active pose. This matches the trajectory pose convention AND the (dormant) MPC
# frame, so consumers use it VERBATIM: adding any active-z lift double-counts the
# active height and drives the catch reach out of leg stroke.

geometry_msgs/Point target_pos         # [x, y, z] platform offset from STOW pose (mm)
geometry_msgs/Quaternion target_quat   # Orientation (w, x, y, z)
geometry_msgs/Vector3 target_vel       # [vx, vy, vz] linear velocity at target (mm/s)
float64 arrival_time                   # Absolute arrival time (perf_counter seconds)
