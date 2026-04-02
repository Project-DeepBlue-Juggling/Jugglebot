# Motor Command Safety

This page describes the defense-in-depth safety system that prevents the Stewart platform from making dangerous movements — sudden slams, overspeed, or commands issued without current motor feedback.

**Source files:**

- [motor_guard.py](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/ros_ws/src/jugglebot/jugglebot/motion/motor_guard.py) — safety checks, max-deviation, bounded extrapolation
- [can_node.py](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/ros_ws/src/jugglebot/jugglebot/can_node.py) — per-command step check

!!! note "Replaces slew limiter"
    The old `control_loop.py` used a per-cycle **slew rate limiter** as its primary safety mechanism. The motor guard replaces this with a **max-deviation check** at MPC command arrival plus **bounded quadratic extrapolation** between commands. The slew limiter was removed because the motor guard receives pre-computed motor commands from the MPC — there is no IK or trajectory evaluation that could produce step discontinuities. The max-deviation check catches the dangerous case (corrupted or stale MPC command far from actual position), while the interpolator ensures smooth inter-sample motion.

## Motivation

On 2026-03-11, the control loop commanded the active pose as a step discontinuity after LEVELLING finished, slamming the platform down and breaking an actuator. The root cause: the control loop was continuously commanding the active position `(0,0,0,0,0,0)` while LEVELLING moved the platform to a different position. When LEVELLING released control, the control loop's stale active-pose command was applied as a step — 200+ mm of instantaneous travel.

The MPC-native architecture eliminates this class of failure: the MPC replans from measured state every 20 ms, so it can never command a position far from where the platform actually is. The motor guard's max-deviation check provides a safety net against MPC solver failures or corrupted IPC messages.

## Defense in Depth

Safety checks are applied at two independent layers. Each layer catches different failure modes.

```
                    Runtime — Motor Guard (500 Hz)
                    ────────────────────────────────
  1. Motor feedback gate (no feedback → no commands)
  2. Feedback staleness check (old feedback → no commands)
  3. Motor overspeed check (feedback velocity too high → ESTOP)
  4. Max deviation check (MPC command too far from actual → ESTOP)
  5. Workspace check (extension or condition number out of bounds → ESTOP)
  6. NaN/Inf rejection (non-finite values in command or feedback → reject)
  7. MPC staleness check (no command in 200ms → ESTOP)
  8. IPC heartbeat (no messages in 500ms → ESTOP)
  9. Bounded extrapolation (velocity decay after 40ms, stop after 100ms)
 10. Stroke clamping (position clamped to hard limits every cycle)

                    Runtime — CAN Node
                    ───────────────────
 11. Per-command step check (rejects any single command > 0.3 rev)
```

## Max Deviation Check

The primary safety mechanism. Runs once per MPC command arrival (50 Hz), comparing the incoming MPC position against the actual motor position from encoder feedback.

```python
MAX_DEVIATION_REV = 0.5   # ~36 mm at standard spool radius

deviation = abs(mpc_commanded_pos - actual_motor_pos)
if any(deviation > MAX_DEVIATION_REV):
    → ESTOP (fault: max_deviation)
```

### Why Check at Arrival, Not Every Cycle?

The deviation between the incoming MPC position and the *current* motor feedback is checked once when the command arrives, not every 500 Hz cycle. Rationale: the interpolator only extrapolates from the MPC command, so per-cycle drift is bounded by `vel_ff × dt` (small). Checking at arrival catches the dangerous case: a corrupted or stale MPC command that jumps far from the actual motor position.

### Why 0.5 rev?

At the standard spool radius (~71.5 mm/rev), 0.5 rev ≈ 36 mm. This is generous enough to tolerate:

- Normal MPC step sizes (0.19 rev at max velocity)
- Interpolation lag between MPC command and motor feedback
- Timing jitter in ZMQ message delivery

But tight enough to catch a crashed MPC sending garbage positions.

## Bounded Extrapolation

The interpolator (`_interpolate_and_send`) extrapolates from the last MPC command using quadratic interpolation. Without bounds, at max velocity (9.5 rev/s), a missed MPC command could drift ~1.9 rev (~136 mm) before the 200 ms staleness E-stop.

**Two-phase approach:**

1. **Normal extrapolation** (0–40 ms): quadratic extrapolation `pos = base + vel·dt + ½·acc·dt²`
2. **Velocity decay** (40–100 ms): velocity decays linearly to zero. Position follows a parabolic coast-down (C0-continuous in velocity — no step discontinuity)

Worst-case travel at max velocity: ~0.665 rev (~47.5 mm).

### Stroke Clamping

Every interpolation cycle, `_commanded_pos_rev` is clamped against stroke hard limits (in rev-space). When clamping activates:

- `vel_ff` is zeroed for the clamped leg (prevents velocity loop fighting position clamp)
- `torque_ff` is zeroed for the clamped leg (prevents gravity/inertia push against the limit)
- A warning is logged, but no E-stop (the MPC should recover on the next command)

## Motor Feedback Gate

Safety checks require current motor positions to function. Without feedback, commands cannot be verified as safe.

**Rule: No feedback → no commands.**

The motor guard tracks three feedback states:

| State | Condition | Effect |
|---|---|---|
| No feedback received | `_has_motor_fb == False` | All commands suppressed |
| Feedback stale | Age > `MOTOR_FB_STALENESS_S` (150 ms) | All commands suppressed |
| Feedback current | Age ≤ 150 ms | Normal operation |

When commands are suppressed, the ODrives hold their last commanded position via internal PID. This is safe — a stale position command means "stay where you are."

### Feedback Data Path

Motor feedback flows from the ODrive encoders through the full system:

```
ODrive encoders → CAN bus → can_node.py → /robot_state (100 Hz)
    → motion_bridge_node.py → IPC (TOPIC_MOTOR_FB) → ZMQ :5555
    → motor_guard _on_motor_feedback()
```

The bridge extracts `pos_estimate`, `vel_estimate`, and `iq_measured` from the first 6 entries of the `/robot_state` message (the leg motors) and forwards them to the motor guard.

**NaN validation:** Motor feedback is validated for NaN/Inf values on arrival. Corrupt feedback is silently dropped — `NaN > threshold` evaluates to `False` in numpy, so without this check, corrupt feedback would silently bypass overspeed and max-deviation E-stop checks.

## Motor Overspeed Check

If any motor's feedback velocity exceeds 110% of the ODrive's configured trap velocity limit (`ODRIVE_TRAP_VEL_LIMIT_RPS`), the motor guard triggers an immediate ESTOP.

```python
MAX_MOTOR_VEL_RPS = ODRIVE_TRAP_VEL_LIMIT_RPS * 1.1

if any(|feedback_velocity| > MAX_MOTOR_VEL_RPS):
    → ESTOP (fault: motor_overspeed)
```

This catches hardware failures (broken encoder, controller runaway) that could cause dangerous motion. The 10% headroom avoids false positives during normal high-speed trajectories.

## MPC Staleness Check

If the motor guard has not received an MPC command for 200 ms (10× the expected 20 ms period), it triggers an E-stop. This catches:

- MPC process crash
- ZMQ connection loss
- MPC solver stuck in an infinite loop

The 200 ms timeout provides generous headroom for occasional long IPOPT solves while still catching a dead MPC quickly.

## Workspace Check

On each MPC command arrival, the motor guard evaluates workspace limits:

1. Convert commanded positions (rev) to leg extensions (mm)
2. Compute Jacobian condition number at the commanded pose
3. Call `check_workspace_limits(extensions, cond, limits)`

| Result | Action |
|---|---|
| `OK` | Command accepted |
| `SOFT_LIMIT` | Warning logged, command accepted |
| `HARD_LIMIT` | E-stop |

Workspace limits: soft = 15 mm from endpoints, hard = 5 mm. Condition number (normalized Jacobian): soft = 1.5× home, hard = 2.0× home (~5-12/~7-16).

## CAN Node Step Check

The CAN node (`can_node.py`) independently validates every motor command before sending it to the ODrives. If any single command changes position by more than `JB_OP_MAX_POSITION_STEP_REV` (0.3 rev ≈ 21.5 mm), the command is rejected.

```python
if abs(new_pos - last_pos) > JB_OP_MAX_POSITION_STEP_REV:
    → reject command, log warning
```

### Why Keep Both Max-Deviation and CAN Step Check?

They catch different failure modes:

| Layer | Catches |
|---|---|
| **Max-deviation** (motor guard) | Corrupted MPC command far from actual position. Checks against actual encoder position, not just previous command |
| **CAN step check** (CAN node) | Last-resort guard at a different layer. Catches commands from any source (not just the motor guard), race conditions on mode transitions, and potential bugs in the interpolator |

## FAULT Chain

When any safety check triggers an ESTOP in the motor guard, the following sequence occurs:

```
1. Motor guard → mode = ESTOP
   → _trigger_estop(reason)
   → zero all outputs (pos, vel_ff, torque_ff)
   → reset MPC interpolation state
   → publish fault telemetry

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

Safety-relevant fields in the motor guard's telemetry:

| Field | Type | Description |
|---|---|---|
| `tracking_error_mm` | float[6] | Per-leg tracking error (mm) — informational, no E-stop |
| `workspace_status` | string | OK / SOFT_LIMIT / HARD_LIMIT |
| `cond_number` | float | Current Jacobian condition number |
| `fault_state` | string | Fault description if any (e.g., `max_deviation`) |

These are published on `/motion/diagnostics` by the bridge and recorded in rosbag for post-incident analysis.

## Constants

Safety constants are defined as module-level values in `motor_guard.py`:

| Constant | Value | Rationale |
|---|---|---|
| `MAX_MOTOR_VEL_RPS` | `ODRIVE_TRAP_VEL_LIMIT_RPS × 1.1` | 10% above configured ODrive velocity limit |
| `MAX_DEVIATION_REV` | 0.5 | ~36 mm — catches runaway MPC, tolerates normal jitter |
| `MPC_CMD_STALENESS_S` | 0.2 | 10× the MPC period (20 ms) |
| `MOTOR_FB_STALENESS_S` | 0.15 | ~50% headroom over 100 Hz feedback (10 ms period) |
| `IPC_HEARTBEAT_TIMEOUT_S` | 0.5 | Catches bridge crash or network loss |
| `MAX_EXTRAP_DT_S` | 0.04 | 2× MPC period before velocity decay begins |
| `EXTRAP_DECAY_DT_S` | 0.06 | Velocity decays to zero over this duration |
| `JB_OP_MAX_POSITION_STEP_REV` | 0.3 | In `hardware_config` — CAN node step check |

## Verification

### Offline Tests

28 tests in `tests/motion/test_motor_guard.py`:

| # | Test | Validates |
|---|---|---|
| 1 | Loop timing (500 Hz jitter) | Mean dt and p99 jitter within bounds |
| 2 | IPC latency (ZMQ round-trip) | Median latency and delivery rate |
| 3 | Force conversion (round-trip) | mm/rev conversion consistency |
| 4 | MPC enable | Enable command activates guard |
| 5 | MPC command flow | Commands forwarded correctly |
| 6 | MPC staleness E-stop | No command in 200 ms → ESTOP |
| 7 | Disable clears state | Disable resets all MPC state |
| 8 | Workspace hard limit E-stop | Out-of-bounds extension → ESTOP |
| 9 | Torque passthrough | Torque values forwarded unchanged |
| 10 | Zeros when no feedforward | Missing FF fields → zero |
| 11 | No feedback suppresses | No motor feedback → no commands |
| 12 | Stale feedback suppresses | Old feedback → no commands |
| 13 | Motor overspeed E-stop | High velocity → ESTOP |
| 14 | Max deviation E-stop | Large position jump → ESTOP |
| 15 | NaN command rejection | Non-finite command → rejected |
| 16 | NaN feedback rejection | Non-finite feedback → dropped |
| 17 | IPC heartbeat E-stop | No messages in 500 ms → ESTOP |
| 18 | Interpolation output | Linear extrapolation produces correct values |
| 19 | Vel_ff finite-difference | Velocity computed from consecutive extensions |
| 20-24 | Bounded extrapolation | Decay, coast-down, stroke clamp |
| 25 | Enable idempotent | Double enable is no-op |
| 26 | Bridge disable+enable | Clears and resets state correctly |

```bash
pytest tests/motion/test_motor_guard.py -v
```

### Recommended Hardware Tests

| # | Test | What to verify |
|---|---|---|
| H1 | Mode transition handoff | Platform does not slam on ACTIVATE; max-deviation catches stale commands |
| H2 | Spacemouse session | Tracking error stays low during normal use |
| H3 | MPC staleness | Kill MPC process; motor guard E-stops within 200 ms |
| H4 | Bounded extrapolation | Introduce artificial MPC delay; verify smooth coast-down |
