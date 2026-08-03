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
bool gravity_correction_loaded    # True once a /gravity_offset message has been
                                  # applied IN THIS PROCESS (contract C-LEVEL-1,
                                  # ros_ws/docs/levelling_frame.md). The correction
                                  # is per-process and in-memory: /gravity_offset is
                                  # VOLATILE with one latched push per orchestrator
                                  # boot and there is no re-request path, so a
                                  # relaunch silently reverts the commanded frame to
                                  # the un-levelled one while the Teensy-persisted
                                  # RobotState.levelling_complete still reads True.
                                  # This flag is the ONLY observation of the node
                                  # that actually APPLIES the correction; the toss's
                                  # REJECTED_NOT_LEVELLED gate consumes it. It says
                                  # "a correction is loaded", NOT "the correction is
                                  # non-identity" — a genuinely level machine
                                  # publishes a zero offset and must not be refused.

bool tilt_map_loaded              # True iff a pose-dependent residual tilt map is
                                  # LOADED (contract C-LEVEL-2,
                                  # ros_ws/docs/levelling_frame.md). Loaded, NOT
                                  # applied: while gravity_correction_loaded is
                                  # false the map is DORMANT — residuals are
                                  # defined relative to a fresh level reference,
                                  # so ingest applies identity until `level` runs.
                                  # "Applied" = tilt_map_loaded AND
                                  # gravity_correction_loaded, read together.
                                  # Refinement, never a gate: an absent map is
                                  # silent and an invalid one is refused whole,
                                  # and neither blocks a throw —
                                  # REJECTED_NOT_LEVELLED keys on
                                  # gravity_correction_loaded and only on that.
string tilt_map_version           # "<captured.date>-<sha256(content)[:8]>" of the
                                  # loaded map, hashed over the calibration
                                  # numbers only (schema version + both grid axes
                                  # + the residual grids), so two files with
                                  # identical calibration report the same string
                                  # and an edit to one node changes it. '' when no
                                  # map is loaded. A non-empty version with
                                  # gravity_correction_loaded=false names a map
                                  # that is loaded but dormant (not yet applied).
