# Trajectory Planning

This page covers how smooth motions are planned — from the quintic polynomial math to feasibility checking to the trajectory manager that handles dynamic targets and mid-motion replanning.

**Source files:**

- [quintic.py](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/ros_ws/src/jugglebot/jugglebot/motion/quintic.py) — polynomial solver, trajectory dataclass, evaluation
- [motor_commands.py](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/ros_ws/src/jugglebot/jugglebot/motion/motor_commands.py) — Cartesian → motor command mapping
- [feasibility.py](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/ros_ws/src/jugglebot/jugglebot/motion/feasibility.py) — feasibility checking, convenience constructors
- [trajectory_manager.py](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/ros_ws/src/jugglebot/jugglebot/motion/trajectory_manager.py) — execution state machine, async pipeline
- [feasibility_worker.py](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/ros_ws/src/jugglebot/jugglebot/motion/feasibility_worker.py) — background worker process

## Concepts

A **trajectory** is a time-parameterized path: at any time $t$, it tells you the platform's position, velocity, and acceleration. The motion planner evaluates the trajectory every 2 ms and maps the result to motor commands through the [kinematics](kinematics.md) and [dynamics](dynamics.md) pipeline.

Trajectories are **quintic polynomials** — 5th-degree polynomials in time. A quintic has 6 coefficients per degree of freedom, which is exactly enough to satisfy 6 boundary conditions: position, velocity, and acceleration at both the start and end of the motion.

## Quintic Polynomial Solver

**Function:** `solve_quintic_1d(x0, v0, a0, xf, vf, af, duration)` → 6 coefficients

For each of the 6 Cartesian degrees of freedom `[x, y, z, rx, ry, rz]`, a separate quintic is solved. The polynomial uses **normalized time** $\tau = t / T$ where $\tau \in [0, 1]$:

$$p(\tau) = c_0 + c_1 \tau + c_2 \tau^2 + c_3 \tau^3 + c_4 \tau^4 + c_5 \tau^5$$

The 6 boundary conditions are:

| Condition | At | Equation |
|---|---|---|
| Start position | $\tau = 0$ | $p(0) = x_0$ |
| Start velocity | $\tau = 0$ | $p'(0) / T = v_0$ |
| Start acceleration | $\tau = 0$ | $p''(0) / T^2 = a_0$ |
| End position | $\tau = 1$ | $p(1) = x_f$ |
| End velocity | $\tau = 1$ | $p'(1) / T = v_f$ |
| End acceleration | $\tau = 1$ | $p''(1) / T^2 = a_f$ |

Using normalized time improves numerical conditioning (all time values are in [0,1] regardless of trajectory duration).

### Why Quintic?

- **Smooth acceleration:** A quintic has a continuous acceleration profile (unlike trapezoidal velocity profiles which have discontinuous acceleration). This matters because discontinuous acceleration means instantaneous jumps in force, which cause vibration.
- **6 boundary conditions:** Exactly matches what we need for C2 continuity (position + velocity + acceleration at both ends).
- **Simple evaluation:** Horner's method evaluates a 5th-degree polynomial in 5 multiplications and 5 additions — trivial computational cost at 500 Hz.

## Creating Trajectories

**Function:** `create_trajectory(start_pose, start_twist, start_accel, end_pose, end_twist, end_accel, duration, t_start, speed_scale)` → `QuinticTrajectory`

This solves 6 independent quintics (one per DoF) and packages them into a frozen dataclass:

```python
@dataclass(frozen=True)
class QuinticTrajectory:
    coeffs: np.ndarray          # (6, 6) — row per DoF, columns [c0..c5]
    duration: float             # effective duration after speed scaling
    t_start: float              # absolute start time (control loop clock)
    speed_scale: float          # applied speed factor
    start_state: np.ndarray     # (18,) original [pose, twist, accel]
    end_state: np.ndarray       # (18,) original [pose, twist, accel]
    original_duration: float    # duration before speed scaling
```

### Speed Scaling

Speed scaling stretches the trajectory in time while preserving its shape:

- Duration: $T_{\text{effective}} = T / \text{scale}$ (lower scale = slower = longer duration)
- Velocities: scaled by `speed_scale` (lower scale = lower velocities)
- Accelerations: scaled by `speed_scale²`

For example, a 1-second trajectory at `speed_scale=0.25` becomes a 4-second trajectory with 1/4 the velocity and 1/16 the acceleration.

### Convenience Constructor

**Function:** `make_rest_to_rest(end_pose, duration, speed_scale, start_pose, t_start)` → `QuinticTrajectory`

Creates a trajectory with zero velocity and acceleration at both ends. If `start_pose` is omitted, defaults to home `[0, 0, 0, 0, 0, 0]`.

```python
traj = make_rest_to_rest(
    end_pose=np.array([0, 0, 30, 0, 0, 0]),  # 30mm above home
    duration=1.0,
    speed_scale=0.5
)
```

## Evaluating Trajectories

**Function:** `evaluate(traj, t)` → (pose, twist, accel) — each a (6,) array

Given an absolute time `t`, evaluates the quintic polynomials and their derivatives using Horner's method.

Three time regions:

| Region | Behaviour |
|---|---|
| $t < t_{\text{start}}$ | Returns start state (deferred start — platform holds) |
| $t_{\text{start}} \leq t \leq t_{\text{end}}$ | Evaluates polynomials, returns interpolated state |
| $t > t_{\text{end}}$ | Returns end **pose** with **zero** twist and accel (hold at target) |

!!! warning "Evaluate at t_end returns zero velocity"
    `evaluate()` past the end time returns zeros for twist and acceleration (hold behaviour). **Never use `evaluate(traj, t_end)` to read boundary conditions** — use `traj.end_state` directly. This caused a real bug: `_plan_return_to_home()` originally used `evaluate()` to get the end velocity for the return trajectory, which gave zeros and created a velocity discontinuity.

### Jerk Evaluation

**Function:** `evaluate_jerk(traj, t)` → (6,) jerk values

Computes the 3rd derivative of the quintic (6 coefficients → 3 non-trivial jerk coefficients). Used by the feasibility checker for jerk limit enforcement.

Jerk is checked in **Cartesian space only** (mm/s³ translational, rad/s³ rotational). Per-leg jerk is not checked — the Jacobian condition number crudely constraint guards against poses where smooth Cartesian motion would map to jerky leg motion.

## Cartesian to Motor Commands

**Function:** `cartesian_to_motor_commands(pose, twist, accel, geom, dynamics_params, feedforward_enabled, gravity_correction)` → (pos_rev, vel_ff_rps, torque_ff_Nm)

This convenience function runs the full pipeline from a Cartesian state to motor commands:

1. Convert rotation vector → rotation matrix
2. Apply gravity correction (levelling) if set
3. Position IK → leg extensions → motor revolutions
4. Velocity IK → leg velocities → motor velocities
5. Full feedforward torques (gravity + inertia + reflected motor)

This is used internally by `TrajectoryManager.evaluate()`.

## Feasibility Checking

**Function:** `check_feasibility(traj, geom, dynamics_params, ...)` → `FeasibilityResult`

Before any trajectory is executed on hardware, it must pass a feasibility check. The checker samples the trajectory at evenly-spaced time points (default: 200) and verifies all constraints at each sample.

### Constraints Checked

| Constraint | Default Limit | Source |
|---|---|---|
| Leg extension | 5 mm from each endpoint | Hardware stroke with safety margin |
| Motor velocity | 15 rev/s | `hardware_config` |
| Motor acceleration | 30 rev/s² | `hardware_config` |
| Jacobian condition number | 2.0 × home cond (~900) | Singularity avoidance |
| Motor feedforward torque | None (disabled by default) | ODrive current limits |
| Translational jerk | 50,000 mm/s³ | Mechanical smoothness |
| Rotational jerk | 400 rad/s³ | Mechanical smoothness |

### FeasibilityResult

```python
@dataclass
class FeasibilityResult:
    feasible: bool                    # pass/fail
    peak_leg_vel_rps: np.ndarray      # (6,) per-leg peak velocity
    peak_leg_accel_rps2: np.ndarray   # (6,) per-leg peak acceleration
    max_extension_mm: np.ndarray      # (6,) per-leg max extension
    min_extension_mm: np.ndarray      # (6,) per-leg min extension
    peak_condition_number: float
    peak_torque_ff_Nm: np.ndarray     # (6,) per-leg peak torque
    peak_jerk_trans: float            # peak translational jerk
    peak_jerk_rot: float              # peak rotational jerk
    violations: list                  # human-readable violation strings
    n_samples: int
```

### Early Exit Mode

For binary search over trajectory durations, `early_exit=True` returns immediately on the first constraint violation — up to **23× faster** for infeasible trajectories, since it doesn't need to evaluate the remaining samples.

### Torque-Skip Optimization

If `torque_limit_Nm=None` (default), the dynamics model is not evaluated at each sample point. This gives a **~1.9× speedup** since the dynamics computation is the most expensive part of each sample.

## Finding Minimum Duration

**Function:** `find_min_feasible_duration(start_pose, start_twist, start_accel, end_pose, end_twist, end_accel, geom, params, ...)` → float or None

Given start and end states, find the shortest trajectory duration that passes feasibility. Uses binary search:

1. Check if `max_T` (default 5.0 s) is feasible. If not, return `None` (the move is fundamentally infeasible).
2. Binary search between `min_T` (0.2 s) and `max_T` with 8 bisections.
3. Each bisection creates a trajectory and runs `check_feasibility(early_exit=True)`.
4. Returns the upper bound after bisection (guaranteed feasible).

This is used by dynamic target handling to automatically select an appropriate trajectory speed.

## The Trajectory Manager

**Class:** `TrajectoryManager` — the state machine that manages trajectory execution, dynamic targets, and mid-motion replanning.

### States

```
    IDLE ──submit()──► EXECUTING ──complete──► COMPLETE
      ▲                    |                      |
      |                    |  (nonzero end vel)   |
      |                    ▼                      |
      +───────────── RETURNING ◄───────────────────+
                     (auto return to home)
```

| State | Meaning | Output |
|---|---|---|
| `IDLE` | No trajectory active. Holding at home. | Hold pose commands |
| `EXECUTING` | Running a forward trajectory. | Evaluate quintic at current time |
| `RETURNING` | Auto-returning to home after a non-zero-velocity target. | Evaluate return quintic |
| `COMPLETE` | Trajectory finished. Holding at end pose. | Hold pose commands |

### Initialization

```python
from jugglebot.motion.trajectory_manager import TrajectoryManager
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.dynamics import DynamicsParams

geom = StewartGeometry()
params = DynamicsParams.from_config()
manager = TrajectoryManager(geom, params)

# CRITICAL: Set hold pose before submitting trajectories
manager.set_hold_pose(np.array([0, 0, 0, 0, 0, 0]))

# For offline tests with synthetic time, inject a clock:
# t = [0.0]
# manager = TrajectoryManager(geom, params, clock=lambda: t[0])
# Default clock is time.perf_counter (correct for hardware).
```

### Per-Cycle Evaluation

```python
pos_rev, vel_ff_rps, torque_ff_Nm = manager.evaluate(t_now)
```

Called every control cycle (500 Hz). Returns motor commands for all 6 legs. Behaviour depends on current state — see the state table above.

### Submitting Trajectories

```python
traj = make_rest_to_rest(end_pose, duration=1.0)
result = check_feasibility(traj, geom, params)
if result.feasible:
    manager.submit(traj)
```

`submit()` can be called during `EXECUTING` or `RETURNING` for mid-motion replanning. The caller must ensure the new trajectory starts from the current state for C2 continuity (position, velocity, and acceleration all continuous at the splice point).

### Live Properties

| Property | Type | Description |
|---|---|---|
| `state` | `TrajectoryState` | Current state |
| `progress` | `float` | 0.0 to 1.0, updated each `evaluate()` call |
| `time_remaining` | `float` | Seconds remaining in active trajectory |
| `current_pose_6dof` | `np.ndarray` | Latest evaluated [x, y, z, rx, ry, rz] |
| `home_pose` | `np.ndarray` | Read-only copy of home pose |

## Dynamic Targets

Dynamic targets allow external systems (like a ball predictor) to send target states on the fly. The trajectory manager automatically plans a feasible trajectory, checks it, and splices it into the current motion.

### Target Specification

A dynamic target has four fields:

| Field | Type | Description |
|---|---|---|
| `target_pos` | (3,) | Target position [x, y, z] in mm |
| `target_quat` | (4,) | Target orientation [w, x, y, z] quaternion |
| `target_vel` | (3,) | Target linear velocity [vx, vy, vz] in mm/s |
| `arrival_time` | float | Absolute time to arrive (perf_counter clock) |

Angular velocity is always zero (the platform doesn't spin to catch a ball).

### Async Pipeline (Non-Blocking)

The async pipeline uses a dedicated **worker process** (not a thread — bypasses the GIL entirely) for feasibility checking. Communication uses two `multiprocessing.Pipe` channels: one for feasibility/splice results, one for deceleration results.

```python
# 1. Queue the request (non-blocking, ~0 ms)
manager.request_dynamic_target(
    target_pos, target_quat, target_vel, arrival_time, t_now
)

# 2. Poll each control cycle (~0 ms per call)
result = manager.poll_pending_result()
if result is not None:
    if result['accepted']:
        # 3. Commit (queues a splice re-check — does NOT commit immediately)
        manager.commit_async_trajectory(result)
        # The splice re-check runs in the background; when it passes,
        # poll_pending_result() handles the commit internally.
```

The worker process runs `check_feasibility()` without blocking the 500 Hz loop. A **generation counter** prevents stale results from being committed if a newer target supersedes an in-progress check.

!!! note "Splice re-check"
    `commit_async_trajectory()` does not commit immediately. It plans the new trajectory from a splice point 200 ms in the future, then sends it back to the worker process for a feasibility re-check. This closes the defense-in-depth gap where the start state may have drifted between the initial check and the commit. The old trajectory continues executing until the re-check passes.

!!! note "Synchronous path (test-only)"
    For offline tests with frozen clocks, `submit_dynamic_target_sync()` in `tests/helpers.py` provides a blocking synchronous wrapper. This is not used in production.

### Return to Home

When a target has nonzero velocity (the platform is moving when it arrives), the manager automatically plans a return-to-home trajectory. The return uses `find_min_feasible_duration()` to find the shortest feasible duration, then adds a 20% safety margin.

The return trajectory is **precomputed in the background** while the outbound trajectory is still executing. When the outbound completes, the return trajectory is ready to start immediately.

### Arrival Time and Duration

The trajectory duration is simply `arrival_time - t_now`. The trajectory starts immediately and uses the full requested duration — a far-future arrival time produces a slow trajectory, a near-future one produces a fast trajectory. Feasibility checking rejects durations that are too short (the move physically can't be done that fast).

!!! note "Deferred start (removed)"
    An earlier implementation included deferred-start logic: when `arrival_time` was far in the future (more than `min_feasible_duration + 2.0s`), the trajectory start was deferred so the platform would hold in place, then move at a reasonable speed. This was removed in commit `1695946` because it caused position discontinuities at the deferred start moment. The current approach — starting immediately with the full duration — is simpler and avoids the discontinuity. Re-implementing deferred start with proper splice continuity at the hold-to-move transition is a candidate for future work.

    The `evaluate()` method does support holding at the start pose before `t_start` (used by the realtime restamping mechanism), so the infrastructure for deferred start still exists if needed.

### Timing Compensation

On real hardware, the feasibility check takes ~250 ms. If we assign `t_start = t_now` before the check, the trajectory starts 250 ms in the past by the time the check finishes. The `TrajectoryManager` measures elapsed time using its injected clock function and automatically shifts `t_start` forward by the elapsed duration. By default the clock is `time.perf_counter` (wall-clock), which is correct for hardware. Offline tests inject a synthetic clock so the restamp delta is always zero (since the clock doesn't advance during computation).

```python
# Hardware (default) — uses time.perf_counter:
manager = TrajectoryManager(geom, params)

# Offline tests — synthetic clock, restamp is harmless:
t = [10.0]
manager = TrajectoryManager(geom, params, clock=lambda: t[0])
```

For the async pipeline, timing compensation happens at commit time — `commit_async_trajectory()` plans from a splice point 200 ms in the future, so the current trajectory continues uninterrupted while the splice is re-checked.

## API Reference

### Trajectory Functions

| Function | Input | Output |
|---|---|---|
| `solve_quintic_1d(x0,v0,a0, xf,vf,af, T)` | Boundary conditions, duration | (6,) coefficients |
| `create_trajectory(...)` | Start/end state, duration, speed_scale | `QuinticTrajectory` |
| `evaluate(traj, t)` | Trajectory, time | (pose, twist, accel) |
| `evaluate_jerk(traj, t)` | Trajectory, time | (6,) jerk |
| `rescale_trajectory(traj, scale, t_start)` | Trajectory, new scale | New `QuinticTrajectory` |
| `check_feasibility(traj, geom, params, ...)` | Trajectory, limits | `FeasibilityResult` |
| `make_rest_to_rest(end_pose, duration, ...)` | End pose, duration | `QuinticTrajectory` |
| `find_min_feasible_duration(...)` | Start/end state, limits | float or None |
| `cartesian_to_motor_commands(...)` | Cartesian state, geom, params | (pos_rev, vel_ff, torque_ff) |

### TrajectoryManager Methods

| Method | Blocking? | Description |
|---|---|---|
| `set_hold_pose(pose_6dof)` | No | Set idle/complete hold position |
| `set_gravity_correction(rot)` | No | Apply levelling correction |
| `set_feedforward_enabled(bool)` | No | Toggle feedforward torques |
| `submit(traj, is_decel)` | No | Submit pre-checked trajectory |
| `evaluate(t)` | No | Get motor commands at time t |
| `cancel()` | No | Cancel active trajectory |
| `request_dynamic_target(...)` | No | Queue for background feasibility check |
| `poll_pending_result()` | No | Check if async check is done |
| `commit_async_trajectory(result)` | No | Queue splice re-check for checked trajectory |
| `poll_precomputed_decel()` | No | Check if decel trajectory is ready |
| `restamp_active_trajectory(elapsed)` | No | Shift t_start forward |
| `shutdown()` | Yes (2s timeout) | Stop worker process |
