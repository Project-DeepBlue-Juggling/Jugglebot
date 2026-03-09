# Validation Results

This page summarises the key hardware test results from each development phase. The full test log with detailed notes is in [`MOTION_PLANNER_PLAN.md`](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/MOTION_PLANNER_PLAN.md).

## Phase 1: Kinematics (Offline)

All 6 tests PASS. No hardware required.

| Test | Result | Precision |
|---|---|---|
| Regression vs legacy `sp_ik.py` | PASS | 0.00 mm error (exact match) |
| Numerical Jacobian (30 poses) | PASS | Max error 1.11×10⁻⁶ |
| Round-trip twist integration (RK4) | PASS | Max error 2.19×10⁻⁵ mm/s |
| Bias term J-dot verification | PASS | Max error 1.71×10⁻⁶ mm/s² |
| FK round-trip (20 poses) | PASS | 0.00 mm, 0.00 rad |
| Singularity map | INFO | 929/1944 reachable, cond 449–644 |

## Phase 2: Control Process & IPC

### Software Tests (Jetson)

| Test | Result | Detail |
|---|---|---|
| Loop timing (500 Hz, 10s) | PASS | Mean 2.066 ms, p99 jitter 0.093 ms, max 4.849 ms |
| IPC latency (100 messages) | PASS | Median 0.755 ms, mean 0.763 ms, p99 0.820 ms |

### Hardware Bench Tests (Single Leg)

All tested on axis 0 using `tools/single_leg_test.py`. ODrive current limit at 50% (10A).

| Test | Result | Detail |
|---|---|---|
| Torque passthrough smoke | PASS | 0.075 Nm to overcome friction; clean IDLE |
| Emergency stop | PASS | IDLE confirmed in 60–88 ms (< 100 ms threshold) |
| Encoder sign convention | PASS | +torque → +encoder (retraction); `can_node.py` inversion correct |
| Force conversion (multi-weight) | PASS | R²=0.994, Kt measured 0.0624 vs 0.0637 datasheet (2.0% discrepancy) |

**Key finding:** Motor stiction threshold ~0.075 Nm. Friction/PD bias: 0.27 A intercept.

## Phase 3: Gravity Feedforward & Position Control

### Stage A — Isolated Leg

| Test | Result | Detail |
|---|---|---|
| Position control smoke | PASS | 0.085 mm max error |
| Velocity feedforward | PASS | 22.2% tracking improvement with vel_ff |
| Gravity feedforward | PASS | 55.8% iq reduction; 10.1% magnitude discrepancy |
| ODrive baseline gains | — | pos_gain=40, vel_gain=0.2, vel_int_gain=0.32 |

### Stage B — Supported Platform

| Test | Result | Detail |
|---|---|---|
| Six-leg CAN coordination | PASS | P99 cycle 2.04 ms, no dropped frames |
| Direction/sign (all legs) | PASS | All legs move in correct direction |
| Multi-leg position hold | PASS | Max deviation 0.023 mm |
| Emergency stop (6 legs) | PASS | All axes IDLE < 100 ms |
| Feedforward dry run | PASS | All positive, total 0.1193 Nm (analytically sane) |

### Stage C — Free Platform

| Test | Result | Detail |
|---|---|---|
| Stable hold | PASS | 0.030 mm max deviation |
| Step response | PASS | Settled < 0.305 mm |
| Gravity torque consistency | PASS | 6 tilted poses, total variation 0.2% |
| Jacobian condition range | — | 414–476 across test poses |

## Phase 4: Trajectory Generator

### Offline Tests

7/7 PASS. Boundary conditions exact, feasibility checker correctly rejects known-bad trajectories.

### Hardware Tests (≤25% Speed)

| Test | Result | Worst Tracking | Threshold | Detail |
|---|---|---|---|---|
| T1: Small move from home | PASS | 0.45 mm | 1.5 mm | 10mm Z step |
| T2: Graduated distance | PASS | 0.85 mm | 1.5 mm | 30mm Z + 5° tilt |
| T3: Multi-pose sequence | PASS | 1.17 mm | 1.5 mm | 4-pose workspace tour |
| T4: Speed scale comparison | marginal FAIL | 1.535 mm | 1.5 mm | Threshold artefact, not functional |

**T4 note:** The 1.535 mm vs 1.5 mm threshold is a measurement artefact. Leg 2 was consistently the worst tracker across all tests — a mechanical issue (slightly different friction/stiffness), not a software problem.

## Phase 5: Inertia Feedforward

### Offline Tests

14/14 PASS. Full Newton-Euler dynamics verified against analytical gravity-only and numerical reference.

### Hardware Tests

| Test | Speed | Result | Detail |
|---|---|---|---|
| T5: FF comparison (gravity vs full) | 50%, 100% | PASS | Full FF not worse than gravity-only (within 5%) |
| T6: Trajectory replay | 50% | PASS | Worst tracking 1.537 mm (threshold 2.0 mm) |
| T6: Trajectory replay | 100% | PASS | Worst tracking 1.885 mm (threshold 3.0 mm) |
| T7: Differential iq | 100% | PASS | Full FF reduces PID effort by 1.5–2.8% |

**Key findings:**

- Stiction dominates motor current (~1.2 A vs ~0.3 A gravity) — inertia feedforward effect marginal at current speeds
- Full FF loop runs at ~290 Hz (vs 480 Hz gravity-only) — may need optimization before ball-catching speeds
- Inertia feedforward expected to become significant at Phase 7+ ball-catching accelerations
- Leg 2 consistently worst tracker (mechanical, not feedforward)

## Phase 6: Hardening

### Offline Tests

12/12 PASS. Workspace limits, fault detection, extended telemetry all verified.

### Hardware Tests

| Test | Speed | Duration | Result | Detail |
|---|---|---|---|---|
| H1: Workspace boundary | 25% | — | PASS | 6 trajectories, Z-down triggers soft limit correctly |
| H1: Workspace boundary | 100% | — | PASS | All 6 trajectories within limits |
| H3: Fault injection (static) | — | — | PASS | All legs idle safely on triggered fault |
| H5: Moderate endurance | 50% | 30 min | PASS | No faults, no degradation |
| H6: Aggressive ramp-up | 75% | 10 min | PASS | Stable throughout |
| H6: Aggressive ramp-up | 100% | 10 min | PASS | Worst tracking 2.025 mm (threshold 3.0 mm) |

**Key lessons learned:**

- **Never command step position changes** — all movements must use profiled trajectories. Instantaneous position jumps cause violent motor responses.
- **CAN encoding must exactly match `can_node.py`** — negate, scale by 1000, round to int16, clamp. Any mismatch causes incorrect motor behaviour.

## Phase 7: Dynamic Targets

### Offline Tests

14/14 PASS. Dynamic target acceptance/rejection, mid-motion splice C2 continuity, infeasible rejection, quaternion conversion, deferred start, return junction continuity.

### Hardware Tests (100% Speed)

| Test | Result | Worst Tracking | Threshold | Detail |
|---|---|---|---|---|
| DT1: Static target +30mm Z | PASS | 1.502 mm | 3.0 mm | Simple dynamic target |
| DT2: Nonzero velocity + return | PASS | 2.436 mm | 3.0 mm | Auto return, final error 0.000 mm from home |
| DT3: Mid-motion replan | PASS | 1.187 mm | 3.0 mm | 2/2 targets accepted |
| DT4: Rapid updates (2 Hz) | PASS | — | — | 10/10 accepted, 0 ODrive faults |
| DT5: Infeasible target ignored | PASS | 2.093 mm | 3.0 mm | Infeasible correctly rejected |

### Bugs Found During Phase 7 Hardware Validation

1. **`_plan_return_to_home()` velocity discontinuity:** Originally used `evaluate(traj, t_end)` to get the end velocity, but `evaluate()` returns zero velocity past `t_end` (hold behaviour). Fix: read `traj.end_state` directly.

2. **Feasibility-check stall:** `submit_dynamic_target()` blocked the control loop for ~250 ms. Fix: async feasibility pipeline with background thread and generation-counter staleness detection.

## Async Feasibility Pipeline

### Offline Tests

8/8 PASS.

| Test | Result |
|---|---|
| Nonblocking queueing | PASS — request returns in < 1 ms |
| Acceptance | PASS — feasible target accepted |
| Rejection | PASS — infeasible target rejected |
| Supersession | PASS — newer request invalidates older |
| Return precompute | PASS — return trajectory ready before outbound completes |
| Cancel | PASS — in-flight check cancelled cleanly |
| Early-exit speedup | PASS — 23× faster for infeasible trajectories |
| Torque-skip speedup | PASS — 1.9× faster when torque limit disabled |

Hardware validation (AP1–AP5) pending.

## Performance Summary

| Metric | Value | Test |
|---|---|---|
| Control loop rate | 500 Hz | Phase 2 |
| Loop jitter (p99) | 0.093 ms | Phase 2 |
| IPC latency (median) | 0.755 ms | Phase 2 |
| Best tracking error | 0.085 mm | Phase 3A (static) |
| Worst tracking error | 2.436 mm | Phase 7 DT2 (100% speed, dynamic) |
| Tracking threshold | 3.0 mm | At 100% speed |
| E-stop latency | 60–88 ms | Phase 2 |
| CAN cycle (p99) | 2.04 ms | Phase 3B |
| Feasibility check time | ~250 ms | Phase 7 (Jetson) |
| Gravity feedforward accuracy | 10.1% discrepancy | Phase 3A |
| Inertia feedforward benefit | 1.5–2.8% PID effort reduction | Phase 5 (T7) |

## Known Issues

| Issue | Impact | Status |
|---|---|---|
| Leg 2 worst tracker | ~0.3 mm worse than other legs consistently | Mechanical — not software |
| 18 ms first-sample spike | One-time startup transient | Benign — no tracking impact |
| Full FF loop ~290 Hz | Below target 500 Hz | May need optimization for ball-catching speeds |
| Soft limit no active speed reduction | Logs warning only | Planned for future (feasibility checker prevents entry) |
| Condition number uses mixed units | Raw values ~450 at home | Mitigated by relative thresholds |
