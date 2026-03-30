# Motor Command Safety

This page describes the defense-in-depth safety system that prevents the Stewart platform from making dangerous movements — sudden slams, overspeed, or commands issued without current motor feedback.

**Source files:**

- [control_loop.py](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/ros_ws/src/jugglebot/jugglebot/motion/control_loop.py) — slew limiter, fault checks
- [trajectory_manager.py](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/ros_ws/src/jugglebot/jugglebot/motion/trajectory_manager.py) — lead-time gate
- [can_node.py](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/ros_ws/src/jugglebot/jugglebot/can_node.py) — per-command step check

## Motivation

On 2026-03-11, the control loop commanded the active pose as a step discontinuity after LEVELLING finished, slamming the platform down and breaking an actuator. The root cause: the control loop was continuously commanding the active position `(0,0,0,0,0,0)` while LEVELLING moved the platform to a different position. When LEVELLING released control, the control loop's stale active-pose command was applied as a step — 200+ mm of instantaneous travel.

This incident exposed a gap: while the system had workspace limits and trajectory feasibility checking, there was no velocity-aware rate limiting on the final motor commands. The safety system described here closes that gap with multiple independent layers.

## Defense in Depth

Safety checks are applied at three independent layers. Each layer catches different failure modes, and any single layer is sufficient to prevent dangerous motion.

```
                    Planning Time
                    ─────────────
  1. Trajectory feasibility checker (rejects unsafe trajectories)
  2. Lead-time gate (rejects targets arriving too soon)

                    Runtime — Control Loop
                    ───────────────────────
  3. Motor feedback gate (no feedback → no commands)
  4. Feedback staleness check (old feedback → no commands)
  5. Motor overspeed check (feedback velocity too high → ESTOP)
  6. Tracking error check (commanded vs actual divergence → warning)
  7. Slew rate limiter (clamps rate of position change vs feedback)
  8. Sustained slew fault (clamping too long → warning)

                    Runtime — CAN Node
                    ───────────────────
  9. Per-command step check (rejects any single command > 0.2 rev)
```

## Slew Rate Limiter

The primary safety mechanism. Runs every control cycle (500 Hz) in the control loop, after motor commands are computed and before telemetry is published.

### How It Works

Each cycle, the limiter compares the commanded motor position against the actual motor position (from encoder feedback). If the difference exceeds the maximum allowed change per cycle, the command is clamped.

```python
MAX_SLEW_RATE_REV_PER_S = 9.5   # ~667 mm/s, 200mm in 0.3s
max_delta = MAX_SLEW_RATE_REV_PER_S * dt  # per-cycle limit

delta = commanded_pos - actual_pos
if any(|delta| > max_delta):
    clamped_delta = clip(delta, -max_delta, max_delta)
    commanded_pos = actual_pos + clamped_delta
    scale = |clamped_delta| / |delta|  # per-leg ratio
    vel_ff *= scale           # proportional to clamping
    # torque_ff unchanged — gravity comp must be preserved
```

### Key Behaviours

| Situation | Response |
|---|---|
| Normal trajectory (deltas well under limit) | Transparent — commands pass through unchanged |
| Step discontinuity (e.g., stale active-pose command) | Clamped to safe rate (~667 mm/s) |
| Sustained clamping (>0.5s) | Warning logged; cycle count derived from loop rate |
| No motor feedback available | All commands suppressed |
| Motor feedback stale (>100ms) | All commands suppressed |

### Why Clamp Instead of Reject?

Rejecting a large command would leave the ODrives holding their last position, which is safe but doesn't recover. Clamping allows the platform to move toward the target at a safe rate, which handles benign cases (e.g., enable at a non-active position) while still catching pathological ones via the sustained-clamping fault.

### Velocity and Torque Feedforward During Clamping

When the slew limiter clamps a position command:

- **vel_ff** is scaled proportionally to the clamping ratio per leg (`|clamped_delta| / |original_delta|`). This keeps the feedforward direction correct while reducing magnitude to match the reduced travel.
- **torque_ff** is left **unchanged**. The gravity compensation torque is still physically correct (the platform's weight hasn't changed) and zeroing it caused oscillation during testing — the ODrive's PID had to rediscover the gravity load from scratch each cycle the limiter was active.

## Motor Feedback Gate

The slew limiter requires current motor positions to function. Without feedback, it cannot verify that commands are safe.

**Rule: No feedback → no commands.**

The control loop tracks three feedback states:

| State | Condition | Effect |
|---|---|---|
| No feedback received | `_has_motor_fb == False` | All commands suppressed |
| Feedback stale | Age > `MOTOR_FB_STALENESS_S` (100ms) | All commands suppressed |
| Feedback current | Age ≤ 100ms | Normal operation |

When commands are suppressed, the ODrives hold their last commanded position via internal PID. This is safe — a stale position command means "stay where you are."

### Feedback Data Path

Motor feedback flows from the ODrive encoders through the full system:

```
ODrive encoders → CAN bus → can_node.py → /robot_state (100 Hz)
    → motion_bridge_node.py → IPC (TOPIC_MOTOR_FB)
    → control_loop.py _on_motor_feedback()
```

The bridge extracts `pos_estimate`, `vel_estimate`, and `iq_measured` from the first 6 entries of the `/robot_state` message (the leg motors) and forwards them to the control loop.

## Motor Overspeed Check

If any motor's feedback velocity exceeds 110% of the ODrive's configured trap velocity limit (`ODRIVE_TRAP_VEL_LIMIT_RPS`), the control loop triggers an immediate ESTOP.

```python
MAX_MOTOR_VEL_RPS = ODRIVE_TRAP_VEL_LIMIT_RPS * 1.1

if any(|feedback_velocity| > MAX_MOTOR_VEL_RPS):
    → ESTOP (fault: motor_overspeed)
```

This catches hardware failures (broken encoder, controller runaway) that could cause dangerous motion. The 10% headroom avoids false positives during normal high-speed trajectories.

## Tracking Error Check

The control loop computes per-leg tracking error: the difference between commanded and actual motor positions, converted to millimetres. If any leg exceeds `MAX_TRACKING_ERROR_MM` (10 mm), a warning is logged identifying the worst leg and error magnitude.

```python
if any(tracking_error_mm > MAX_TRACKING_ERROR_MM):
    → log warning (worst leg index, error in mm)
```

Large tracking errors are logged but do **not** trigger ESTOP. The slew limiter already rate-limits motor commands, so the motors will naturally catch up. Freezing the platform (ESTOP) would be worse than allowing recovery, especially during manual spacemouse control where the operator can move back to reduce error.

## Lead-Time Gate

Dynamic targets (ball-catching) must arrive with sufficient lead time for the trajectory planner to compute and verify a feasible path. Targets with less than `MIN_LEAD_TIME_S` (300ms) of lead time are rejected.

```python
MIN_LEAD_TIME_S = 0.3

duration = arrival_time - t_now
if duration < MIN_LEAD_TIME_S:
    → reject (log warning, continue current trajectory)
```

This applies to both the async production path (`request_dynamic_target`) and the test-only synchronous helper (`submit_dynamic_target_sync` in `tests/helpers.py`). The 300ms minimum accounts for feasibility checking time (~250ms on Jetson) plus a small margin.

## CAN Node Step Check

The CAN node (`can_node.py`) independently validates every motor command before sending it to the ODrives. If any single command changes position by more than `JB_OP_MAX_POSITION_STEP_REV` (0.2 rev ≈ 14.3 mm), the command is rejected.

```python
if abs(new_pos - last_pos) > JB_OP_MAX_POSITION_STEP_REV:
    → reject command, log warning
```

### Why Keep Both the Slew Limiter and CAN Step Check?

They catch different failure modes:

| Layer | Catches |
|---|---|
| **Slew limiter** (control loop) | Velocity-aware rate limiting against actual motor position. Handles sustained high-speed commands that individually pass the step check but collectively are too fast |
| **CAN step check** (CAN node) | Last-resort guard at a different layer. Catches commands from any source (not just the control loop), race conditions on mode transitions, and potential bugs in the slew limiter itself |

The CAN step check was confirmed working during the DEACTIVATE race condition: when the control loop briefly publishes stale active-pose commands before its IPC `disable` message arrives, the CAN node rejects them. At 500 Hz, the per-command step check allows up to 100 rev/s effective velocity — far too fast for safety — which is why the slew limiter is needed. But as a last-resort catch-all at a different layer, the step check has near-zero cost (6 float comparisons per command) and provides valuable defense in depth.

## FAULT Chain

When any safety check triggers an ESTOP in the control loop, the following sequence occurs:

```
1. Control loop → mode = ESTOP
   → _zero_outputs()              # all commands go to zero
   → cancel active trajectory
   → publish fault telemetry (diagnostic fields only, no motor commands)

2. ODrives hold last commanded position
   (position control fail-safe — no drift, no free-spin)

3. Bridge publishes fault in /motion/diagnostics

4. Orchestrator detects fault
   → forces FAULT state
   → publishes ERROR on control_mode_topic

5. CAN node receives ERROR
   → _gently_move_to_setpoint(0.0, deactivating=True)
   → TRAP_TRAJ profiled stow to stow position
```

All faults ultimately lead to a gentle, profiled stow. The platform never free-spins or drops under gravity.

## Telemetry

The slew limiter's state is included in the control loop's IPC telemetry:

| Field | Type | Description |
|---|---|---|
| `slew_limited` | bool | `True` if the limiter clamped commands this cycle |
| `tracking_error_mm` | float[6] | Per-leg tracking error (mm) |
| `fault_state` | string | Fault description if any (e.g., `slew_limit_sustained`) |

These are published on `/motion/diagnostics` by the bridge and recorded in rosbag for post-incident analysis.

## Constants

All safety constants are defined as module-level values in `control_loop.py` (not in config files), since they are consumed only there:

| Constant | Value | Rationale |
|---|---|---|
| `MAX_SLEW_RATE_REV_PER_S` | 9.5 | ~667 mm/s — 200mm of travel takes ≥0.3s |
| `SLEW_FAULT_DURATION_S` | 0.5 | Converted to cycles at runtime (`int(0.5 * rate_hz)`) |
| `MAX_MOTOR_VEL_RPS` | `ODRIVE_TRAP_VEL_LIMIT_RPS × 1.1` | 10% above configured ODrive velocity limit |
| `MAX_TRACKING_ERROR_MM` | 10.0 | ~3.5% of full stroke (280mm) |
| `MOTOR_FB_STALENESS_S` | 0.1 | 100ms ≈ 50 control cycles at 500 Hz |
| `MIN_LEAD_TIME_S` | 0.3 | In `trajectory_manager.py` — matches feasibility check time + margin |
| `JB_OP_MAX_POSITION_STEP_REV` | 0.2 | In `hardware_config` — CAN node step check |

## Verification

### Offline Tests

8 tests in `motion/tests/test_safety.py`:

| # | Test | Validates |
|---|---|---|
| 1 | Slew clamp | Large step clamped to max_delta per cycle |
| 2 | Slew transparency | Normal trajectory deltas pass through unchanged |
| 3 | Sustained slew fault | >250 cycles clamping → ESTOP |
| 4 | No feedback suppresses | `_has_motor_fb=False` → no commands |
| 5 | Stale feedback suppresses | 200ms-old timestamp → no commands |
| 6 | Motor overspeed | Feedback velocity > limit → ESTOP |
| 7 | Tracking error | Error > 10mm → warning logged |
| 8 | Lead-time gate | Dynamic target with <0.3s lead → rejected |

Tests 1-7 require pyzmq/msgpack (Jetson-only); test 8 always runs.

```bash
python -m jugglebot.motion.tests.test_safety
```

### Recommended Hardware Tests

| # | Test | What to verify |
|---|---|---|
| H1 | LEVELLING → ACTIVATE handoff | Platform does not slam; slew limiter ramps smoothly |
| H2 | Spacemouse session | `slew_limited` never appears in telemetry during normal use |
| H3 | Trajectory at 50% speed | Tracking error stays well under 10mm threshold |
