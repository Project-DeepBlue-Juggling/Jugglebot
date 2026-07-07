# Accept/reject feedback for a dynamic/timed target (trajectory_node → consumers).
# Replaces the dormant MPC process's ZMQ :5559 TargetFeedbackPub for the catch path:
# catch_coordinator_node subscribes to trajectory/target_feedback and, on a
# rejection, drives its feasibility blacklist (blacklist semantics preserved).

bool accepted            # True iff the target was accepted (plan installed).
string code              # Feasibility / service code (OK on accept; the reject code otherwise).
string reason            # Human-readable reject reason ('' on accept).
float64 arrival_time     # The target's absolute arrival time (perf_counter seconds) — the
                         # correlation key the coordinator matches against its last submission.
string source            # Target source: 'catch' (catch/dynamic_target) or 'timed' (service).
