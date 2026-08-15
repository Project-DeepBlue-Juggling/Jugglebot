# ADR-0012: Port the 500 Hz Hermite/Taylor interpolator ladder verbatim from `motor_guard.py`

- **Status**: Accepted
- **Date**: 2026-06-02 (captured); decision made 2026-05-31
- **Deciders**: Harrison + Claude
- **Related**: [ADR-0001](0001-offload-can-and-interpolator-from-jetson.md), [parent plan](../../plans/archived/2026-08-15%20teensy-can-offload.md), [firmware handoff](../../plans/archived/2026-07-05%20HANDOFF-teensy-can-offload-firmware-wip.md) D5 / D9 / D10

## Context

The current 500 Hz interpolator lives in
[`motor_guard.py`](../../ros_ws/src/jugglebot/jugglebot/motion/motor_guard.py)
and implements a graceful-degradation ladder that handles a wide spread of
MPC behaviour:

- **Hermite path.** When the next MPC waypoint (`cmd_next`) is known,
  cubic Hermite between u₀ and u₁ with C¹ continuity via the u₂ lookahead.
  This is the nominal path.
- **Taylor extrapolation path.** When `cmd_next` is absent (older MPC
  payload or solver failure), cubic Taylor extrapolate from the last
  base point + velocity + acceleration + EMA-filtered jerk, for up to
  50 ms (`MAX_EXTRAP_DT_S`).
- **Velocity decay path.** Past `MAX_EXTRAP_DT_S`, ramp the boundary
  velocity linearly to zero over 60 ms (`EXTRAP_DECAY_DT_S`). Positions
  *continue from the boundary using the decaying velocity*; they do not
  hold.
- **E-STOP / link-fault.** Separately, a staleness watchdog
  (`MPC_CMD_STALENESS_S = 250 ms`) latches an E-STOP if no MPC update
  arrives at all.

Each path was tuned and validated against real catching/throwing data on
the bench over many sessions. The exact decay shape, jerk filter
constants, and threshold values reflect that history.

## Decision

Port the ladder **verbatim** from `motor_guard.py` to the Teensy's C++
`leg_interp.cpp`. Match the math at the algorithm level, the threshold
values at the constant level, and the *branching structure* at the
control-flow level (`cmd_next`-presence first, then `dt` thresholds).

Validation is via an offline cross-reference harness
([`tools/probes/teensy_link_profiling/hermite_xref/`](../../tools/probes/teensy_link_profiling/hermite_xref/))
that feeds recorded MPC trajectories through both the Python
`MotorGuard` and a Python mirror of the Teensy C++ implementation and
asserts <1e-6 rev divergence on the position/velocity output. The xref
currently shows **0.0 rev divergence** on synthetic + recorded inputs.

## Consequences

**Positive:**

- **No new dynamics to debug on the bench.** The hardest part of the
  port — the math itself — is already validated.
- **Tracked-clamp behaviour preserved.** `MAX_LEAD_REV` and per-leg
  stroke clamps are ported alongside the math, so safety properties
  travel with the interpolator.
- **Existing logbook entries remain authoritative.** Investigations
  that found and fixed bugs in `motor_guard.py` (e.g. friction-FF limit
  cycle, velocity-decay drift) carry forward unchanged.

**Negative:**

- **Friction feedforward is NOT ported on the first pass.** The
  Stribeck-with-smooth-gate friction term in `motor_guard._compute_friction_ff_Nm`
  is a separate concern (torque, not position/velocity) and requires
  its own validation pass. Until ported, the torque path on the Teensy
  is gravity+inertia only. Tracked as handoff D9; needs bench
  measurement to confirm tracking is acceptable without friction FF,
  or follow-up port.
- **Float32 on the Teensy vs float64 in Python.** The Python xref runs
  in float64 to verify the algorithm. On the Teensy, the ISR uses
  float32 for cycle count. The plan accepts "within a tolerance that
  doesn't affect tracking" — bench validation needed to set the
  tolerance and confirm it's met.
- **Two safety clamps now exist in defence-in-depth.** Both
  `motor_guard.py`'s lead-clamp (Jetson-side, still in the bridge for
  now) and the Teensy's lead-clamp (port destination) enforce
  `MAX_LEAD_REV`. Acceptable redundancy; to be reconciled when
  `motor_guard.py` is slimmed during the Phase-10 bridge work.

**Neutral:**

- **The interpolator runs in a bare timer ISR above the FreeRTOS
  syscall ceiling**, not as a FreeRTOS task. Means the ISR cannot be
  delayed by any RTOS critical section — see
  [ADR-0009](0009-freertos-tsandmann-port.md) for the rationale.

## Alternatives considered

- **Reimplement from scratch using a different interpolation scheme
  (quintic, B-spline, etc.).** Rejected. Every change to the
  interpolator must be re-validated on the bench, and the current
  Hermite scheme is tuned for the project's specific catching dynamics.
  Algorithmic improvements should come from informed bench observation,
  not from a fresh-eyes port.
- **Run the interpolator on the Jetson and just stream 500 Hz CAN
  commands over UDP to the Teensy.** Rejected: doesn't get the
  hard-real-time guarantee that's the entire point of
  [ADR-0001](0001-offload-can-and-interpolator-from-jetson.md). The
  Jetson's Linux scheduler jitter would leak into the CAN setpoint
  stream.
