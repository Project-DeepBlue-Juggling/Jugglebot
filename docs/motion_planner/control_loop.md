# Motor Guard

This page describes the 500 Hz motor guard process — what happens every 2 ms, how IPC messages are handled, and how the guard protects the hardware.

**Source files:**

- [motor_guard.py](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/ros_ws/src/jugglebot/jugglebot/motion/motor_guard.py) (~860 lines)
- [ipc.py](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/ros_ws/src/jugglebot/jugglebot/motion/ipc.py) (~375 lines)

!!! note "Replaces control_loop.py"
    The motor guard replaced `control_loop.py` as part of the MPC-native refactor. The old control loop computed IK, dynamics, trajectories, and smoothing at 500 Hz. The motor guard is a pure **interpolator + safety monitor** — all motion planning is done by the MPC at 50 Hz. See [MPC_NATIVE_REFACTOR_PLAN.md](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/MPC_NATIVE_REFACTOR_PLAN.md) for the full migration rationale.

## Architecture

The motor guard runs as a **standalone Python process**, separate from ROS2. It communicates with the ROS2 bridge node and the MPC process via ZeroMQ IPC. This separation ensures deterministic timing — the guard maintains 500 Hz with sub-millisecond jitter, independent of ROS2 executor scheduling.

```python
# Entry point
python -m jugglebot.motion.motor_guard --rate 500 --log-level INFO
```

### Role in the Pipeline

The motor guard sits between the MPC solver and the CAN hardware:

```
MPC (50 Hz)
    → HardwarePlant.command()
    → ZMQ :5557
    → motor_guard (500 Hz)    ← quadratic interpolation + safety checks
    → ZMQ :5556
    → motion_bridge_node
    → CAN node
    → ODrives (PASSTHROUGH mode)
```

The MPC outputs pre-computed motor commands (position in rev, velocity in rev/s, torque in Nm) at 50 Hz. The motor guard **does not** compute IK, dynamics, or trajectories — it only:

1. **Interpolates** between 50 Hz MPC commands at 500 Hz for smooth motor output
2. **Validates** every command against safety checks
3. **Forwards** approved commands to the bridge

## The Main Loop

Every cycle follows this sequence:

```
  1. Record cycle timing
  2. Process IPC messages (MPC commands, mode commands, motor feedback)
  3. Check safety (heartbeat, staleness, overspeed)
  4. Interpolate and send (quadratic extrapolation from last MPC command)
  5. Periodic logging (every 5 seconds)
  6. Sleep for remainder of cycle
```

### Step 1: Timing

Each cycle's wall-clock duration is recorded in a `LoopStats` object (sliding window of 10,000 samples). This provides real-time statistics:

| Metric | Typical Value |
|---|---|
| Mean dt | 2.066 ms |
| Std deviation | 0.055 ms |
| 99th percentile | 2.16 ms |
| P99 jitter | 0.093 ms |
| Max dt | 4.849 ms |

### Step 2: Process IPC

All pending messages from IPC sockets are drained and dispatched:

| Topic | Handler | Action |
|---|---|---|
| `mpccmd` | `_on_mpc_command()` | Validate and latch new MPC command for interpolation |
| `mode` | `_on_mode_command()` | Handle enable/disable/estop |
| `motorfb` | `_on_motor_feedback()` | Store encoder positions/velocities/currents |

Mode commands are processed **before** data messages (the mode socket is drained first), ensuring an E-stop arrives before any MPC updates in the same batch.

### Step 3: Safety Checks

Runs every cycle regardless of new commands:

| Check | Condition | Action |
|---|---|---|
| IPC heartbeat | No messages for > 500 ms | E-stop |
| MPC command staleness | No MPC command for > 200 ms | E-stop |
| Motor feedback staleness | No feedback for > 150 ms | Suppress commands |
| Motor overspeed | Any motor > 110% of trap velocity limit | E-stop |

### Step 4: Interpolate and Send

This is the core computation. The motor guard **quadratically extrapolates** from the last MPC command:

```python
dt = t_now - mpc_base_timestamp

# Quadratic extrapolation (position, velocity, torque)
pos = base_pos + vel * dt + 0.5 * acc * dt**2
vel_ff = vel + acc * dt
torque_ff = base_torque  # unchanged between MPC commands
```

This upsamples 50 Hz MPC commands to 500 Hz smooth ramps. Each 2 ms cycle sends an incremental position step to the ODrive with matching `vel_ff`. The ODrive sees small position deltas with velocity feedforward that acts as the *slope* of the ramp — not a persistent push that causes overshoot (see Phase 1B findings).

**Bounded extrapolation:** If the MPC command is late (> 40 ms), velocity decays linearly to zero over 60 ms. Position follows a parabolic coast-down (C0-continuous in velocity). Worst-case travel at max velocity: ~0.665 rev (~47.5 mm).

**Stroke clamping:** Every cycle, interpolated positions are clamped against stroke hard limits. When a leg is clamped, its `vel_ff` and `torque_ff` are zeroed to prevent the velocity loop from fighting the position clamp.

### Step 5: Publish Telemetry

A telemetry message is published every cycle (when safety checks pass) containing:

| Field | Content |
|---|---|
| `leg_pos` | 6 motor positions (rev) |
| `leg_vel` | 6 motor velocities (rev/s) |
| `leg_torques` | 6 motor torques (Nm) |
| `dt` | Actual cycle time (s) |
| `cond_number` | Current Jacobian condition number |
| `workspace_status` | OK / SOFT_LIMIT / HARD_LIMIT |
| `tracking_error_mm` | 6 per-leg tracking errors (mm) |
| `fault_state` | Fault description if any |

## Guard Modes

```
                 enable
  DISABLED ──────────────► ENABLED
      ▲                       │
      │      disable          │  fault / heartbeat loss
      +───────────────────────+──────► ESTOP
                                        │
                              disable   │
                  ◄─────────────────────+
```

| Mode | Outputs | Transitions |
|---|---|---|
| `DISABLED` | All zero | → `ENABLED` on `enable` command |
| `ENABLED` | Active interpolation | → `DISABLED` on `disable`; → `ESTOP` on fault/heartbeat |
| `ESTOP` | All zero | → `DISABLED` on `disable` (requires explicit recovery) |

### Enable Sequence

When transitioning from `DISABLED` to `ENABLED`, the motor guard **waits for the first MPC command** before sending any output. This differs from the old control loop which seeded the active pose on enable. No motor commands are published during the window between enable and the first MPC command.

**Enable idempotency:** Receiving an enable command when already ENABLED is a no-op (no state reset). This handles the case where both the motion bridge (:5555) and HardwarePlant (:5557) send enable commands — the ordering is nondeterministic, but the end state is always correct.

### E-stop

All safety violations funnel through a single `_trigger_estop()` method that:

1. Sets mode to ESTOP
2. Zeros all outputs
3. Resets MPC interpolation state
4. Records the fault reason
5. Publishes fault telemetry

This eliminates the scattered E-stop logic that existed in the old control loop.

## MPC Command Validation

When an MPC command arrives (50 Hz), several checks run before it is accepted as the new interpolation base:

| Check | Action |
|---|---|
| NaN/Inf in position, velocity, or torque | Reject command |
| Max deviation from actual motor position | E-stop if > 0.5 rev (~36 mm) |
| Workspace hard limit (leg extension or condition number) | E-stop |
| Workspace soft limit | Log warning |

If a command passes validation, it becomes the new interpolation base: position, velocity, acceleration, and torque are latched, and the interpolation timestamp resets.

**Velocity computation:** `vel_ff` is computed from consecutive MPC extensions using the deterministic MPC period (`dt = 0.02s`), not wall-clock time. This makes the velocity estimate immune to ZMQ delivery jitter.

**Acceleration computation:** `acc` is computed from three consecutive extensions: `acc = (u_curr - 2*u_prev + u_prev_prev) / dt^2`. This enables quadratic interpolation that eliminates the 50 Hz position ripple that linear interpolation would produce during accelerating trajectories.

## IPC Layer Detail

### Three-Port Design

The motor guard communicates on three ZeroMQ ports:

| Port | Direction | Content |
|---|---|---|
| `:5555` | SUB ← Bridge | Mode commands (enable/disable/estop), motor feedback |
| `:5557` | SUB ← HardwarePlant | MPC commands (pos, vel, torque, extensions, pose) |
| `:5556` | PUB → Bridge | Telemetry (motor commands + diagnostics) |

### Socket Design

The motor guard has **three** SUB sockets:

| Socket | Port | Topics | Mode | Purpose |
|---|---|---|---|---|
| Data socket | :5555 | `motorfb` | CONFLATE (latest only) | Motor feedback — only the most recent matters |
| Mode socket | :5555 | `mode` | Ordered (every message) | Safety-critical commands must not be dropped |
| MPC socket | :5557 | `mpccmd` | CONFLATE (latest only) | MPC commands — only the latest matters |

Mode messages are drained first to ensure safety-critical commands are processed before data.

### Message Serialization

All messages are serialized with **msgpack** — a compact binary format. Each ZeroMQ frame consists of:

```
Frame 0: topic (bytes, e.g., b'mpccmd')
Frame 1: msgpack-encoded dict
```

## Timing Statistics

The `LoopStats` class tracks cycle timing with a 10,000-sample sliding window:

```python
stats = LoopStats(target_dt_s=0.002)  # 500 Hz
stats.record(actual_dt)

print(stats.summary())
# "mean=2.066ms std=0.055ms p99=2.160ms max=4.849ms jitter_p99=0.093ms n=4843"
```

Properties: `mean`, `std`, `percentile_99`, `max`, `jitter_99` (p99 of |dt - target|), `count`.

A summary is logged every 5 seconds during operation.

## What Was Removed (vs control_loop.py)

The following components existed in `control_loop.py` but are **not** in the motor guard — the MPC handles all of these:

| Component | Replacement |
|---|---|
| Position IK (`pose_to_leg_lengths`) | MPC outputs leg extensions directly |
| Velocity IK (`twist_to_leg_velocities`) | HardwarePlant computes vel_ff |
| Dynamics (`gravity_to_motor_torques`) | HardwarePlant computes torque_ff via Newton-Euler |
| StreamSmoother | MPC produces smooth trajectories |
| TrajectoryManager | MPC IS the trajectory planner |
| FeasibilityWorker | MPC constraints handle feasibility |
| Quintic polynomial evaluation | MPC doesn't use quintics |
| Direct-target mode | Replaced by MPC target-setting |
| Workspace speed scaling | MPC respects its own constraints |
| Per-cycle slew limiter | Replaced by max-deviation check at command arrival |

## API Reference

### MotorGuard

```python
class MotorGuard:
    def __init__(self,
                 rate_hz: float = 500,
                 geom: StewartGeometry | None = None,
                 ipc: MotorGuardIPC | None = None)

    def run(self) -> None     # blocks until stop() or signal
    def stop(self) -> None    # sets _running = False
```

### IPC Classes

**MotorGuardIPC** (used by motor_guard.py):

| Method | Description |
|---|---|
| `recv_all()` → list | Non-blocking drain of all pending messages |
| `send_telemetry(msg)` | Publish telemetry to bridge |
| `seconds_since_last_recv` | Heartbeat watchdog property |
| `close()` | Clean shutdown |

**BridgeIPC** (used by motion_bridge_node.py):

| Method | Description |
|---|---|
| `send_mode_command(msg)` | Send mode command to motor guard |
| `send_motor_feedback(msg)` | Forward encoder data to motor guard |
| `recv_telemetry()` → dict or None | Receive latest telemetry |
| `close()` | Clean shutdown |
