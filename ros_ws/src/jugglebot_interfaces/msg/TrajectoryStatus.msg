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
float64 leg_vel_limit_mmps        # LIVE session leg-space limits — the values the
float64 leg_acc_limit_mmps2       # feasibility gate is enforcing RIGHT NOW, i.e.
float64 leg_jerk_limit_mmps3      # the YAML working point as ramped by any
                                  # trajectory/set_limits calls this session
                                  # (each clamped to its YAML hard ceiling).
                                  # Published so acceptance pre-checks downstream
                                  # (the toss sequencer's closed-form reach bound)
                                  # judge against the SAME limits the planner will
                                  # enforce at t_release, instead of a stale
                                  # module copy of the YAML default — a ramp-down
                                  # otherwise passes a goal the planner then
                                  # refuses MID-FLIGHT, and a ramp-up is refused
                                  # pre-throw for no physical reason. 0.0 = field
                                  # absent (old publisher/bag): consumers fall
                                  # back to the YAML-default copies, i.e. exactly
                                  # the pre-field behaviour.

string tilt_map_version           # "<captured.date>-<sha256(content)[:8]>" of the
                                  # loaded map, hashed over the calibration
                                  # numbers only (schema version + both grid axes
                                  # + the residual grids), so two files with
                                  # identical calibration report the same string
                                  # and an edit to one node changes it. '' when no
                                  # map is loaded. A non-empty version with
                                  # gravity_correction_loaded=false names a map
                                  # that is loaded but dormant (not yet applied).

# ── Unified 7-DoF cycle observability (plan Phase 4) ──────────────────────────
# `plan_kind` above cannot answer "is a unified cycle running": CyclePlan.kind is
# deliberately 'move' (trajectory_node gates its is-a-move-still-running checks on
# it), so a cycle is indistinguishable from a go_to_pose on that field alone. These
# five are the discriminator and the numbers a unified sitting watches.
bool cycle_active                 # True iff the ACTIVE plan is a 7-channel CyclePlan,
                                  # i.e. the hand is on the 40 Hz stream. False on every
                                  # legacy plan, so a bag from a legacy session is
                                  # byte-identical to a pre-field one apart from the
                                  # default.
float64 cycle_plan_wall_ms        # Wall time of the LAST trajectory/plan_cycle callback
                                  # (ms) — the measured number the split budget (core
                                  # <= 50 ms, total <= 250 ms) is judged against. 0.0
                                  # before the first call.
float64 cycle_hand_peak_rev       # Peak commanded hand position over the active cycle
                                  # plan (rev, ODrive absolute). 0.0 when none.
float64 cycle_hand_peak_vel_rps   # Peak |commanded hand velocity| over it (rev/s). Read
                                  # against JB_TRAJ_HAND_VEL_LIMIT_RPS; validate_cycle
                                  # already refused anything past it, so a number at the
                                  # cap here is a plan running at its ceiling, never a
                                  # breach.
float64 cycle_supersede_deadline_s  # Seconds until the ACTIVE cycle plan must have been
                                  # superseded, or 0.0 when it is rest-terminal (and so
                                  # cliff-safe) or absent. NEGATIVE means the deadline
                                  # has PASSED with the plan still installed: from that
                                  # instant the emitter's u1/v1 come from the terminal
                                  # HOLD, so the final 25 ms segment — the release
                                  # stroke itself — is commanded to a stop. No firmware
                                  # guard fires on that (the error is well inside
                                  # MAX_LEAD_HAND_REV and MAX_DEVIATION_HAND_REV), so
                                  # this field and the node's ERROR line are the only
                                  # trace it leaves.
