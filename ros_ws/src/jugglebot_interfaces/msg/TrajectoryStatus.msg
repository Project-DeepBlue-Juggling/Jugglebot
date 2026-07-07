# Trajectory generator status (trajectory/status, 5 Hz).
# Replaces the Phase-1 diagnostic_msgs/DiagnosticStatus stand-in with a typed
# message once the interface package is rebuilt for the Phase-2 services.

bool streaming                    # True iff the emitter is publishing knot frames.
string mode                       # Current control mode (STANDBY / TRAJECTORY / ...).
string plan_kind                  # Active plan kind: 'hold' / 'move' / 'none'.
float64 plan_time_remaining_s     # Time until the active plan reaches its terminal hold (s).
uint64 seq                        # Monotonic emitted-frame counter.
float64 max_emit_gap_ms           # Max inter-tick emitter gap over the last window (ms).
string last_rejection             # Most recent rejection reason ('' if none).
