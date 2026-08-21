# Control Loop

This page describes the 50 Hz simulation loop, the target source abstraction, and how the hand coordinator integrates with MPC planning.

**Source files:**

- `sim/main.py` — entry point, loop functions, target source adapters
- `sim/hand/coordinator.py` — hand state machine + ball lifecycle
- `sim/hand/trajectory.py` — catch/throw hand trajectories
- `sim/input/sim_control.py` — pause/step/speed controls

## Loop Architecture

The simulation runs at a fixed **50 Hz** control rate (`CONTROL_DT = 0.02 s`). Each control step:

1. Reads the full plant state from MuJoCo sensors.
2. Queries the active `TargetSource` for the current target.
3. Processes ball spawning and hand commands.
4. Calls `mpc.solve()` with the target pose, arrival time, and twist.
5. Applies the resulting leg command to the plant.
6. Steps MuJoCo physics by `CONTROL_DT`.
7. Checks for ball capture.
8. Logs telemetry.

### Viewer Loop vs Headless Loop

Two loop implementations exist:

- **`run_mpc_headless()`** — No viewer. Runs `n_steps = duration / CONTROL_DT` iterations without real-time pacing. Used for batch testing and CI.
- **`run_mpc_with_viewer()`** — MuJoCo passive viewer with real-time pacing, pause/step/speed controls, automatic looping (if the source supports `reset()`), and optional custom rendering (horizon preview, ball trajectory).

Both loops share the same core logic via `_mpc_solve()` and `_execute_hand_cmd()` helper functions.

### Real-Time Pacing (Viewer Mode)

The viewer loop uses an accumulated wall-clock budget to maintain real-time playback:

```python
wall_budget += CONTROL_DT * sleep_factor
elapsed = time.monotonic() - start_wall
sleep_time = wall_budget - elapsed
if sleep_time > 0:
    time.sleep(sleep_time)
elif sleep_time < -0.5:
    # Fallen behind (e.g., speed change) — resync
    start_wall = time.monotonic()
    wall_budget = 0.0
```

The `sleep_factor` comes from the `SimController` (default 1.0, adjustable with Up/Down arrow keys). The resync threshold (-0.5 s) prevents runaway catch-up after pausing or speed changes.

## Target Sources

The `TargetSource` protocol defines a single required method:

```python
class TargetSource(Protocol):
    def update(self, sim_time: float, state: PlantState) -> TargetCommand: ...
```

A `TargetCommand` bundles everything the loop needs for one control step:

| Field | Type | Purpose |
|---|---|---|
| `target_pose` | `(6,)` ndarray | Target `[x,y,z,rx,ry,rz]` for MPC |
| `arrival_time` | `float` or `None` | Deadline; None = ASAP |
| `target_twist` | `(6,)` or `None` | Desired twist at arrival; None = zero |
| `hand_cmd` | various or `None` | Hand command this step |
| `ball_spawn` | `BallSpawn` or `None` | Ball to spawn this step |

Note: the `DynamicTarget` that feeds into `TargetCommand` has a `settle_margin_s` field (default 0.1 s). The coordinator subtracts this from the ball's `arrival_time` to set the MPC's deadline, so the platform arrives and settles **before** the ball gets there. See [Hand & Ball Physics — DynamicTarget](hand_and_ball.md#dynamictarget) for the full field list.

### Adapter Hierarchy

Each input mode has a dedicated adapter that converts its domain-specific logic into `TargetCommand`:

| Adapter | Input Mode | Controller | Features |
|---|---|---|---|
| `StaticTargetSource` | `--pose`, `--sequence` | — | Time-triggered pose schedule |
| `WaypointTargetSource` | `--trajectory T1..T6` | — | Waypoint list with arrival times |
| `InteractiveTargetSource` | `--spacemouse`, `--keyboard` | — | Continuous ASAP targets |
| `CatchTargetSource` | `--catch DT1..DT8` | `HandCoordinator` | Scripted catch sequences with ball physics |
| `ThrowCatchTargetSource` | `--throw-catch TC1..TC4` | `HandCoordinator` | Throw → catch cycle with ball release |
| `InteractiveCatchSource` | `--interactive-catch` | `HandCoordinator` | User-spawned balls, dynamic feasibility |
| `ContinuousThrowCatchSource` | `--juggle` | `ContinuousThrowCatchController` | Self-throw-catch loop with parameter tuning |
| `ContinuousThrowCatchSource` | `--cycle-time` | `TossLoopController` | Toss loop with quintic Hermite platform motion |

Note: `ContinuousThrowCatchSource` is a generic wrapper that adapts any controller with `update()`, `reset()`, and viewer lifecycle methods to the `TargetSource` protocol. The `--juggle` and `--cycle-time` modes use different controllers inside the same wrapper.

### Optional Lifecycle Methods

Target sources may optionally implement:

| Method | Purpose |
|---|---|
| `reset()` | Enables viewer looping — source resets to initial state |
| `close()` | Cleanup (e.g., close input devices) |
| `key_callback(keycode)` | Viewer keyboard events |
| `should_step() -> bool` | Pause gate (replaces default SimController) |
| `sleep_factor -> float` | Speed multiplier for real-time pacing |
| `render(viewer)` | Custom per-frame rendering |
| `notify_capture(sim_time)` | Ball capture notification |
| `print_summary()` | End-of-run statistics |

These are detected via `hasattr()` at runtime — no base class required.

## Hand Coordination

The hand actuator is independent of the MPC. The MPC controls the 6 platform legs; the hand is commanded by the `HandCoordinator` state machine.

### Hand State Machine

```
IDLE → PRIMING → APPROACHING → HOLDING → CAUGHT → RETURNING → IDLE
                                  ↓
                              (timeout → RETURNING)
```

For throw-catch sequences, additional states:

```
IDLE → APPROACHING_THROW → THROWING → TRANSITIONING → HOLDING → CAUGHT → ...
```

### Hand Commands

The `_execute_hand_cmd()` function processes the `hand_cmd` field from `TargetCommand`:

| Command Type | Action |
|---|---|
| `BallRelease(velocity_mms)` | Release ball from kinematic hold with specified velocity |
| `HandCatchSequence` / `HandThrowSequence` | Install as active sequence (sampled each step) |
| `float` / `int` | Direct position command (mm) |
| `'prime'` | Move hand to prime position (**~335 mm**; ~323 mm until 2026-08-21) |
| `'home'` | Move hand to bottom of travel (0 mm) |

Active hand sequences (`HandCatchSequence`, `HandThrowSequence`) are sampled at the current simulation time each step. When the sequence returns `None` (complete), the active sequence is cleared.

### Ball Lifecycle

Ball management flows through the `MuJoCoPlant.ball_manager`:

1. **Spawn:** `plant.spawn_ball(position_mm, velocity_mms)` teleports the ball and sets its velocity. Alternatively, `ball_manager.spawn_in_hand()` places the ball in the hand for throw sequences.

2. **Flight:** MuJoCo physics simulates the ball under gravity. The ball manager runs capture detection every physics substep inside `plant.step()`.

3. **Capture:** When the ball enters the hand's capture zone (checked every physics substep), the ball manager latches the capture flag. The control loop harvests it via `plant.check_and_capture()` and notifies the target source.

4. **Hold:** After capture, the ball is held kinematically in the hand (position updated each substep to follow the hand).

## Toss Loop Adapter

The toss loop (`--cycle-time`, `TossLoopController` in `sim/input/toss_loop.py`) is a specialized target source for continuous-motion juggling. It differs from other adapters in several ways:

**Bypasses HandCoordinator.** The toss loop manages hand sequences and ball lifecycle directly, without the `HandCoordinator` state machine. The coordinator's post-catch return-to-home flow (which takes ~2 s) is incompatible with tight cycle timing (~480 ms hold time).

**Quintic Hermite platform reference.** Instead of returning a static target with `arrival_time`, the toss loop evaluates a quintic Hermite spline at the current sim time and returns the result as an ASAP target. The MPC sees a smoothly moving reference and tracks it with uniform cost weighting. See [Variable Horizon — Toss Loop Bypass](variable_horizon.md#toss-loop-bypass) for the motivation.

**Time-driven targets.** Platform target transitions are driven by the cycle clock, not by ball capture detection. When the ball is captured, only the ball state changes (kinematic hold); the platform reference continues along its pre-planned spline uninterrupted. This eliminates the target-switch jerk that occurs when platform motion is coupled to capture events.

**Pre-planned cycles.** The next cycle is planned at startup and at each cycle transition (not at capture time). Positions alternate deterministically, so the hand position at the next throw prelude can be predicted in advance.

**Data flow:**

```
_quintic_interp()                          (module-level in toss_loop.py)
    → interpolated pose (ASAP, no arrival_time)
        → TossLoopController.update()
            → ContinuousThrowCatchSource.update()
                → TargetCommand(target_pose, arrival_time=None)
                    → mpc.solve(state, pose)   [uniform tracking weight]
```

## Telemetry

Each control step produces a `StepRecord` logged to CSV:

| Field | Source |
|---|---|
| `time` | Simulation time |
| `ref_pose`, `ref_twist` | Target from TargetCommand |
| `actual_pose`, `actual_twist` | PlantState sensors |
| `cmd_extensions` | MPC output (u[0]) |
| `actual_extensions` | PlantState sensors |
| `leg_velocities` | PlantState sensors |
| `hand_cmd_mm`, `hand_pos_mm`, `hand_vel_mmps` | Hand state |
| `solve_time_ms` | Wall-clock IPOPT solve duration |
| `solve_status` | IPOPT return status string |
| `cost` | Optimal objective value |
| `constraint_violation` | Max constraint residual |

The optional live dashboard (`--dashboard`) broadcasts each record via Server-Sent Events (SSE) to a browser-based visualization at `http://localhost:8082`.

## Viewer Controls

Available in all viewer-mode MPC runs:

| Key | Action |
|---|---|
| Space | Pause / unpause simulation |
| Right arrow | Step one frame (while paused) |
| Up arrow | Speed × 2 (max 16×) |
| Down arrow | Speed × 0.5 (min 0.0625×) |
| R | Reset speed to 1× |

Interactive modes add their own keys (B for ball spawn, T for throw, etc.) via the `key_callback` composition system. Multiple key callbacks are combined with `_combine_key_callbacks()`.

## Test Sequences

Pre-defined test sequences are available via command-line flags. For detailed descriptions and what each sequence validates, see [Hand & Ball Physics — Test Sequences](hand_and_ball.md#test-sequences).
