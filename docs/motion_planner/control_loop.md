# Control Loop

This page describes the 500 Hz control process — what happens every 2 ms, how IPC messages are handled, and how the loop transitions between modes.

**Source files:**

- [control_loop.py](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/ros_ws/src/jugglebot/jugglebot/motion/control_loop.py) (~690 lines)
- [ipc.py](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/ros_ws/src/jugglebot/jugglebot/motion/ipc.py) (~375 lines)

## Architecture

The control loop runs as a **standalone Python process**, separate from ROS2. It communicates with the ROS2 bridge node via ZeroMQ IPC. This separation ensures deterministic timing — the loop maintains 500 Hz with 0.093 ms p99 jitter on the Jetson, independent of ROS2 executor scheduling.

```python
# Entry point
python -m jugglebot.motion.control_loop --rate 500 --log-level INFO
```

## The Main Loop

Every cycle follows this sequence:

```
  1. Record cycle timing
  2. Process IPC messages (targets, modes, feedback)
  3. Poll async feasibility results
  4. Check heartbeat (E-stop if no messages > 0.5s)
  5. Compute motor commands
  6. Check workspace limits
  7. Slew limit + safety checks (see Motor Command Safety)
  8. Publish telemetry (if safety checks pass)
  9. Periodic logging (every 5 seconds)
 10. Sleep for remainder of cycle
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

All pending messages from both IPC sockets are drained and dispatched:

| Topic | Handler | Action |
|---|---|---|
| `target` | `_on_target()` | Store latest pose/twist/accel for direct-target mode |
| `mode` | `_on_mode_command()` | Handle enable/disable/estop/feedforward/gravity/fault |
| `traj` | `_on_trajectory()` | Create and submit quintic trajectory |
| `dyntgt` | `_on_dynamic_target()` | Queue for async feasibility check |
| `motorfb` | `_on_motor_feedback()` | Store encoder positions/velocities/currents |

Mode commands are processed **before** data messages (the mode socket is drained first), ensuring an E-stop arrives before any target updates in the same batch.

### Step 3: Poll Async Results

If a dynamic target feasibility check completed in the background thread, the result is polled via `poll_pending_result()`. If accepted, the trajectory is committed via `commit_async_trajectory()`.

### Step 4: Heartbeat

If no IPC message has been received for 0.5 seconds, the loop triggers an E-stop (`ipc_heartbeat_lost`). This catches cases where the bridge node crashes or the network connection drops.

### Step 5: Compute Motor Commands

This is the core computation. The behaviour depends on whether a trajectory is active:

**Trajectory mode** (state is `EXECUTING` or `RETURNING`):

```python
pos_rev, vel_ff_rps, torque_ff_Nm = traj_manager.evaluate(time.perf_counter())
```

The trajectory manager evaluates quintic polynomials and runs them through the full [kinematics](kinematics.md) and [dynamics](dynamics.md) pipeline internally.

**Direct-target mode** (no active trajectory):

```python
# 1. Apply gravity correction if set
effective_rot = gravity_correction @ target_rot

# 2. Position IK
extensions_mm = pose_to_leg_lengths(target_pos, effective_rot, geom)
pos_rev = extensions_mm_to_revs(extensions_mm, geom)

# 3. Velocity IK
leg_vel = twist_to_leg_velocities(target_twist, target_pos, effective_rot, geom)
vel_ff_rps = leg_velocities_to_motor_velocities(leg_vel, geom)

# 4. Feedforward torques
torque_ff_Nm = compute_full_feedforward_torques(
    target_pos, effective_rot, target_twist, target_accel, geom, params
)
```

### Step 6: Workspace Check

After computing motor commands, the loop verifies workspace limits:

1. Convert commanded positions (rev) to leg extensions (mm)
2. Compute Jacobian condition number at current pose
3. Call `check_workspace_limits(extensions, cond, limits)`

If the result is `HARD_LIMIT`: cancel trajectory, E-stop, set fault state.
If the result is `SOFT_LIMIT`: log warning; in direct-target mode, scale the smoother velocity and acceleration limits by `speed_scale` to slow the platform as it approaches hard boundaries.

### Step 7: Slew Limit + Safety Checks

After workspace checks, the `_slew_limit()` method runs the [motor command safety](safety.md) checks. This is the final gate before telemetry is published. It verifies:

1. Motor feedback is available and current (not stale)
2. No motor is overspeeding
3. No leg has excessive tracking error
4. The rate of position change vs actual motor position is within bounds

If any check fails, commands are suppressed (or an ESTOP is triggered for critical faults like motor overspeed). On ESTOP, the control loop publishes **fault telemetry** (diagnostic fields only, no motor commands) so the bridge can report the fault to the orchestrator. Normal telemetry (with motor commands) is only published when `_slew_limit()` returns `True`.

See [Motor Command Safety](safety.md) for full details.

### Step 8: Publish Telemetry

A telemetry message is published every cycle (when safety checks pass) containing:

| Field | Content |
|---|---|
| `leg_pos` | 6 motor positions (rev) |
| `leg_vel` | 6 motor velocities (rev/s) |
| `cmd_torques` | 6 motor torques (Nm) |
| `dt` | Actual cycle time (s) |
| `ff_torques` | 6 feedforward torques (Nm) |
| `traj_state` | Trajectory state string |
| `traj_progress` | Trajectory completion fraction |
| `cond_number` | Current Jacobian condition number |
| `workspace_status` | OK / SOFT_LIMIT / HARD_LIMIT |
| `workspace_speed_scale` | Speed scale from workspace limits |
| `tracking_error_mm` | 6 per-leg tracking errors (mm) |
| `slew_limited` | Whether slew limiter clamped commands this cycle |
| `fault_state` | Fault description if any |

## Control Modes

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

| Mode | Outputs | Trajectories | Transitions |
|---|---|---|---|
| `DISABLED` | All zero | Cancelled | → `ENABLED` on `enable` command |
| `ENABLED` | Active computation | Active | → `DISABLED` on `disable`; → `ESTOP` on fault/heartbeat |
| `ESTOP` | All zero | Cancelled | → `DISABLED` on `disable` (requires explicit recovery) |

### Enable Sequence

When transitioning from `DISABLED` to `ENABLED`:

1. `_seed_home_pose()` initializes all outputs to the home pose
2. The trajectory manager's hold pose is set to home
3. Target state variables are set to home
4. The control process immediately publishes home-pose commands

This ensures the ODrives receive a valid position command on the first cycle after enable, rather than zeros (which would command the motors to the zero-extension position).

### Gravity Correction

The `set_gravity_offset` mode command applies a rotation correction to all IK computations. This is used for levelling — if the base isn't perfectly level, the gravity correction rotates the coordinate frame so "down" in the motion planner matches actual gravity.

The correction is a rotation matrix constructed from `tilt_x` and `tilt_y` angles (radians), pre-multiplied before all rotation matrices in IK computations.

## IPC Layer Detail

### Two-Socket Design

The control process subscribes to the command channel (tcp://localhost:5555) with **two** sockets:

**Data socket** — for `target` and `motorfb` topics:

- `CONFLATE=1`: only the most recent message is kept in the buffer
- Purpose: at 500 Hz target updates, we only care about the latest one

**Mode socket** — for `mode`, `traj`, and `dyntgt` topics:

- No CONFLATE: every message is delivered in order
- `RCVHWM=64`: high-water mark prevents unbounded queue growth
- Purpose: an `estop` command must never be dropped or overwritten

Both sockets are polled non-blocking (`zmq.NOBLOCK`) every cycle. Mode messages are drained first to ensure safety-critical commands are processed before data.

### Message Serialization

All messages are serialized with **msgpack** — a compact binary format. Each ZeroMQ frame consists of:

```
Frame 0: topic (bytes, e.g., b'target')
Frame 1: msgpack-encoded dict
```

Every message dict includes a `ts` field (timestamp from `time.time()`) for debugging.

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

## API Reference

### ControlLoop

```python
class ControlLoop:
    def __init__(self,
                 target_rate_hz: float = 500,
                 geom: StewartGeometry | None = None,
                 ipc: ControlProcessIPC | None = None)

    def run(self) -> None     # blocks until stop() or signal
    def stop(self) -> None    # sets _running = False
```

### IPC Classes

**ControlProcessIPC** (used by control_loop.py):

| Method | Description |
|---|---|
| `recv_all()` → list | Non-blocking drain of all pending messages |
| `send_telemetry(msg)` | Publish telemetry to bridge |
| `seconds_since_last_recv` | Heartbeat watchdog property |
| `close()` | Clean shutdown |

**BridgeIPC** (used by motion_bridge_node.py):

| Method | Description |
|---|---|
| `send_target(msg)` | Send pose target to control process |
| `send_mode_command(msg)` | Send mode command |
| `send_trajectory_command(msg)` | Send trajectory |
| `send_dynamic_target(msg)` | Send dynamic target |
| `send_motor_feedback(msg)` | Forward encoder data |
| `recv_telemetry()` → dict or None | Receive latest telemetry |
| `close()` | Clean shutdown |
