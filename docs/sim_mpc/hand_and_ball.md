# Hand & Ball Physics

This page documents the hand trajectory system, ballistic planning, throw-catch coordination, and ball capture detection. The hand actuator is independent of the MPC — it follows its own trajectory profiles ported from the Teensy firmware.

**Source files:**

- `sim/hand/trajectory.py` — Catch/throw 3-segment trajectories, quintic smooth-move
- `sim/hand/ballistics.py` — Inverse ballistics, orientation, hand offset
- `sim/hand/planner.py` — Throw-catch plan generation (12-step pipeline)
- `sim/hand/coordinator.py` — State machine orchestrating platform + hand + ball
- `sim/hand/feasibility.py` — Coarse-horizon MPC reachability check
- `sim/ball/manager.py` — Ball spawn, capture detection, kinematic hold, release
- `sim/ball_butler/sim.py` — Ball Butler throw simulator (pure-kinematic)

## Hand Trajectory Profiles

Both catch and throw trajectories use the same 3-segment structure: **accelerate → constant-velocity hold → decelerate**. This profile is a direct port of the Teensy firmware's `Trajectory.h`, which runs on hardware to command the hand motor.

### Constants (from `hardware_config.yaml`)

| Constant | Value | Meaning |
|---|---|---|
| `HAND_STROKE_M` | 0.355 m | Physical hand stroke |
| `STROKE_MARGIN_M` | 0.02 m | Safety margin at each end |
| `CATCH_VEL_RATIO` | 0.6 | Hand velocity = 60% of ball speed |
| `INERTIA_RATIO` | 0.747 | Decel/accel duration ratio (asymmetric profile) |
| `CATCH_VEL_HOLD_PCT` | 0.10 | 10% of effective stroke at constant velocity |
| `THROW_VEL_HOLD_PCT` | 0.05 | 5% of effective stroke at constant velocity |
| `MAX_EVENT_VEL_MPS` | 7.0 m/s | Maximum ball/hand speed |
| `MIN_EVENT_VEL_MPS` | 0.3 m/s | Minimum ball/hand speed |

The effective stroke is `HAND_STROKE_M - 2 × STROKE_MARGIN_M = 0.315 m` (315 mm).

### Catch Trajectory (`HandCatchTrajectory`)

The hand moves **downward** to match the incoming ball's velocity, reducing the impact force:

```
Segment 1 (accel):   0 → vC over t_acc,  travels accel_stroke
Segment 2 (hold):    constant vC,         travels vel_hold_stroke
Segment 3 (decel):   vC → 0 over t_dec,  decelerates to rest
```

Where `vC = -CATCH_VEL_RATIO × event_vel_mps` (negative = downward).

The `INERTIA_RATIO` (0.747) controls the asymmetry between acceleration and deceleration. The code uses `irC = 1/INERTIA_RATIO`:

- `t_acc = 2 / (irC + 1) × accel_stroke / |vC|`
- `t_dec = t_acc × irC` (i.e., `t_acc / INERTIA_RATIO`)

This means deceleration takes ~1.34× as long as acceleration (`1/0.747 ≈ 1.34`) — the hand decelerates more gently than it accelerates, because it must absorb the ball's energy without bouncing it out.

**Timeline convention:** `t = 0` is the **midpoint of the velocity-hold phase** — the moment the ball is expected to arrive. The trajectory starts at a negative time (`start_time < 0`) and ends at a positive time.

### Throw Trajectory (`HandThrowTrajectory`)

The hand moves **upward** to eject the ball:

```
Segment 1 (accel):   0 → v_throw over t_acc
Segment 2 (hold):    constant v_throw (ball released at end of this segment)
Segment 3 (decel):   v_throw → 0 over t_dec
```

**Timeline convention:** `t = 0` is the moment of **ball release** (end of velocity-hold phase). The trajectory starts at a negative time and ends at a positive time (deceleration after release).

The throw uses `THROW_VEL_HOLD_PCT = 0.05` (5% of stroke for velocity hold, vs 10% for catch), giving a shorter constant-velocity phase since precision at release matters more than cushioning.

### Smooth Move (`HandSmoothMove`)

A quintic S-curve used as a "prelude" to position the hand before catch/throw:

$$s(\tau) = 10\tau^3 - 15\tau^4 + 6\tau^5, \quad \tau = t/T$$

Duration `T` is computed so peak acceleration stays within `MAX_SMOOTH_MOVE_HAND_ACCEL_RPS2 = 100 rev/s²`, with a minimum of 50 ms.

### Sequences (`HandCatchSequence`, `HandThrowSequence`)

A sequence combines a smooth-move **prelude** with the catch/throw trajectory, anchored to absolute simulation time:

```
[--- prelude (smooth move to start position) ---][--- trajectory (3-segment profile) ---]
```

The `try_create()` static method validates timing feasibility: the prelude must fit before the trajectory starts (with a 20 ms safety gap). If the ball arrives too soon for the hand to reach its starting position, the catch/throw is infeasible.

## Ballistics

The `ballistics.py` module provides inverse ballistics for throw-catch planning.

### Inverse Ballistics (`compute_launch_velocity`)

Given release position, target position, and flight time, computes the required launch velocity (no drag, constant gravity):

$$\vec{v}_0 = \frac{\vec{r}_{target} - \vec{r}_{release}}{t_{flight}} + \frac{1}{2} g \cdot t_{flight} \cdot \hat{z}$$

### Arrival Velocity (`compute_arrival_velocity`)

Velocity at catch given launch velocity and flight time:

$$\vec{v}_{arrival} = \vec{v}_0 + \vec{g} \cdot t_{flight}$$

### Platform Orientation (`compute_orientation`)

Computes the rotation vector that aligns the platform's +Z axis with a target direction (launch direction for throw, negative arrival direction for catch):

1. Compute angle between `[0, 0, 1]` and the target direction.
2. If angle > 30° → raise `ValueError` (workspace limit).
3. Compute rotation axis via cross product.
4. Return `axis × angle` as rotation vector.

### Hand Offset (`compute_hand_offset_mm`)

The ball sits at an offset from the platform centroid along the platform's local Z axis. The offset depends on the hand position:

```
offset = HAND_AXIS_BOTTOM_OFFSET_MM + hand_pos_mm + BALL_SEAT_OFFSET_MM
       = -129.0 + hand_pos_mm + 44.4
```

Where:

- `-129.0 mm`: hand body origin is 129 mm below the platform centroid.
- `hand_pos_mm`: current hand stroke position (mm from physical bottom).
- `44.4 mm`: distance from hand body origin to ball COM when seated (40 mm to hand opening site + 4.4 mm cone rim clearance).

This offset is used to compute the platform centroid position from the desired ball position: `centroid = ball_pos - offset × platform_Z`.

## Throw-Catch Planner

The `ThrowCatchPlanner` converts user-level inputs (throw position, catch position, throw time, catch time) into a complete `ThrowCatchPlan` via a 14-step pipeline. Step numbers match the `--- N. ... ---` comments in `planner.py`:

### Planning Pipeline

1. **Validate inputs** — throw_time ≥ 0, catch_time > throw_time.
2. **Compute launch velocity** — inverse ballistics from throw/catch positions and flight time (world-frame ball velocity).
3. **Hand-relative throw velocity** — subtract platform velocity (if continuous motion) to get the velocity the hand must impart: `v_hand_rel = v_launch - v_platform`.
4. **Throw orientation** — rotation vector aligning platform +Z with the hand-relative launch direction.
5. **Throw speed check** — project hand-relative velocity onto platform Z to get scalar hand throw speed. Must be ≤ 7.0 m/s and non-negative (negative means platform velocity overshoots launch velocity).
6. **Build throw hand trajectory** — `HandThrowSequence` anchored to throw_time.
7. **Throw centroid** — platform centroid = throw_pos - hand_offset × platform_Z_throw. The hand offset uses the release position from the throw trajectory.
8. **Arrival velocity** — compute ball velocity at catch time (launch velocity + gravity), then subtract platform catch velocity to get hand-relative catch velocity.
9. **Catch orientation** — rotation vector aligning platform +Z with **negative** hand-relative arrival velocity (face into incoming ball).
10. **Catch centroid** — platform centroid = catch_pos - hand_offset × platform_Z_catch. The hand offset uses `_HAND_CATCH_X5_MM`, the stroke position where the ball meets the hand during the catch velocity-hold phase.
11. **Build catch hand trajectory** — `HandCatchSequence` with event velocity (projection of relative velocity onto platform Z) clamped to [0.3, 7.0] m/s.
12. **Build DynamicTargets** — throw target (at throw time) and catch target (at catch time). Both include `settle_margin_s = 0.1` for the coordinator flow (set to 0 in continuous-motion mode). The toss loop ignores `arrival_time` and `settle_margin_s`, reading only `pose_6dof` and `arrival_twist` to build its quintic Hermite reference.
13. **Build BallSpawn** — records the throw position and launch velocity at throw_time, for spawning the ball into the simulation when the throw begins.
14. **Timing feasibility** — throw hand trajectory must end before catch prelude starts. Raises `ValueError` if insufficient transit time.

### `_HAND_CATCH_X5_MM`

This is the hand position (in mm from physical bottom) where the ball contacts the hand during the catch velocity-hold phase. It's derived from the Teensy firmware's segment geometry:

```python
_x5_m = _TOTAL_STROKE_M - (_TOTAL_STROKE_M - CATCH_VEL_HOLD_PCT * _TOTAL_STROKE_M) * INERTIA_RATIO / (1.0 + INERTIA_RATIO)
_HAND_CATCH_X5_MM = STROKE_MARGIN_M * 1000.0 + _x5_m * 1000.0
```

Where `_TOTAL_STROKE_M = HAND_STROKE_M - 2 × STROKE_MARGIN_M = 0.315 m` (the effective stroke in metres).

This position determines the hand offset used for computing the catch platform centroid — ensuring the ball arrives at the correct position along the hand axis.

### `ThrowCatchPlan` Dataclass

| Field | Type | Purpose |
|---|---|---|
| `throw_target` | `DynamicTarget` | Platform pose + timing for throw |
| `catch_target` | `DynamicTarget` | Platform pose + timing for catch |
| `throw_hand_seq` | `HandThrowSequence` | Hand trajectory for throw |
| `catch_hand_seq` | `HandCatchSequence` | Hand trajectory for catch |
| `ball_release_time` | `float` | Absolute sim time of ball release |
| `ball_release_vel_mms` | `(3,)` ndarray | Ball velocity at release (mm/s) |
| `ball_spawn` | `BallSpawn` | Ball spawn data (position + velocity at throw_time) |

## Hand Coordinator State Machine

The `HandCoordinator` orchestrates platform motion, hand trajectories, and ball lifecycle. It receives `DynamicTarget` commands and returns per-step targets for the MPC and hand commands for the plant.

### `DynamicTarget`

| Field | Default | Purpose |
|---|---|---|
| `pose_6dof` | — | Target `[x, y, z, rx, ry, rz]` (mm, rad) |
| `arrival_time` | `None` | Absolute sim time; None = ASAP |
| `arrival_twist` | `None` | Twist at arrival; None = hold |
| `hold_duration` | 0.5 s | Time to hold at catch pose waiting for ball |
| `event_vel_mps` | `None` | Ball speed for hand trajectory computation |
| `settle_margin_s` | 0.1 s | Platform arrives early by this amount to settle |
| `mode` | `'catch'` | `'catch'` or `'throw'` |

The `settle_margin_s` field shifts the MPC's deadline earlier than the actual ball arrival, giving the platform time to come to rest at the target pose before the ball arrives.

### Catch-Only Flow

```
IDLE → PRIMING → APPROACHING → HOLDING → CAUGHT → RETURNING → IDLE
                                  ↓
                              (timeout → RETURNING)
```

1. **PRIMING** — hand moves to prime position (~323 mm, top of effective stroke minus margin).
2. **APPROACHING** — MPC drives platform to catch pose. Hand catch sequence is queued and starts playing when the prelude time arrives.
3. **HOLDING** — at catch pose, hand trajectory playing, waiting for ball. Times out after `hold_duration` if no capture.
4. **CAUGHT** — ball captured. Hand retracts to bottom (`'home'`).
5. **RETURNING** — MPC drives platform back to active pose. After 1.0 s, transitions to IDLE.

### Throw-Catch Flow

```
IDLE → APPROACHING_THROW → THROWING → TRANSITIONING → HOLDING → CAUGHT → ...
```

1. **APPROACHING_THROW** — MPC drives to throw pose. Throw hand sequence queued. When `ball_release_time` arrives, returns `BallRelease` command.
2. **THROWING** — ball released. Waits for throw hand trajectory to finish deceleration.
3. **TRANSITIONING** — MPC drives from throw pose to catch pose. Catch hand sequence queued. Re-checks catch feasibility (throw may have run long).
4. **HOLDING** → **CAUGHT** → **RETURNING** — same as catch-only flow.

### Hold Timeout (Deferred)

The hold timeout is **deferred by one step**: when `sim_time >= hold_end_time`, `_hold_expired` is set to `True` but the transition doesn't happen until the **next** `update()` call. This gives the main loop one last chance to call `notify_capture()` on the same sim step — preventing a race where the ball is captured and the hold times out simultaneously.

## Ball Capture Detection

The `BallManager` runs capture detection every **physics substep** (typically 2000 Hz), not just at the 50 Hz control rate. This prevents missing fast-moving balls that pass through the hand between control steps.

### Contact-Based Capture

Capture uses **MuJoCo's contact solver**, not a geometric zone check. Each substep, the ball manager iterates over `data.ncon` active contacts looking for pairs involving the ball geom and any hand collision geom:

```python
for i in range(data.ncon):
    c = data.contact[i]
    if ball_geom in (c.geom1, c.geom2) and hand_geom in (c.geom1, c.geom2):
        # Capture triggered
```

When a ball-hand contact pair is found, the ball transitions to kinematic hold. Capture is latched (`_capture_pending`) and polled by the control loop via `check_and_capture()`.

The hand model uses soft contact physics (`solref` with a long time constant) so the contact solver produces a cushioned interaction. The ball and hand geoms use `contype=3, conaffinity=3` (bits 0 and 1 = ground + hand).

### Post-Release Anti-Re-Capture

After a ball is released (thrown), two mechanisms prevent immediate re-capture:

1. **Collision disable + separation requirement** (`_must_exit_zone`): at release, ball-hand collision is **disabled** (`contype=1`, ground-only) so the contact solver cannot trigger re-capture. The ball must reach **100 mm separation** from the hand body before collision is re-enabled. This handles the case where the ball is thrown along the hand axis and remains geometrically close to the hand for several substeps after release.
2. **Cooldown timer** (`_release_cooldown = 30`): a 30-substep guard as a secondary safety net.

### `spawn_in_hand()`

For throw sequences, the ball is placed directly at the hand opening site in kinematic hold:

```python
ball_manager.spawn_in_hand()  # Ball tracks hand site exactly until release
```

This differs from `spawn(position, velocity)` which teleports the ball to an arbitrary position in free flight. `spawn_in_hand()` enables the throw flow where the ball is held, carried by the hand trajectory, and released with a specified velocity.

### Kinematic Hold

When held, the ball's `qpos` and `qvel` are **overwritten every physics substep** to match the hand opening site exactly. No MuJoCo weld constraint or contact physics — the ball is purely kinematic during hold. This is simpler and avoids contact solver artifacts.

## Ball Butler Simulator

The `BallButlerSim` (in `sim/ball_butler/sim.py`) models the physical Ball Butler throw machine as a **pure-kinematic throw source** — no MuJoCo bodies, just ballistic trajectory computation.

### Ballistics Solver

Ported from the production Ball Butler firmware:

1. **Yaw solve** — given target `(x, y)` and the off-axis offset of the release point, solves for the yaw angle.
2. **Pitch sweep** — sweeps pitch angle to find the trajectory that minimizes horizontal landing velocity at the target, subject to joint limits.
3. Returns `(yaw, pitch, speed, time_of_flight)`.

### Configuration

Geometry from `hardware_config.yaml`: yaw/pitch offsets, release position, joint limits (pitch min/max, yaw max), throw speed/height limits.

### Integration

Used in interactive catch mode (`--interactive-catch --bb`): pressing `T` triggers a Ball Butler throw instead of a simple ball drop. The `BallButlerSim.compute_throw()` returns a `BallSpawn` that the catch system processes normally.

## Feasibility Checking

The `FeasibilityChecker` predicts whether the MPC can reach a target pose in time. See [Variable Horizon — Feasibility Checking](variable_horizon.md#feasibility-checking) for the two-stage algorithm.

### Orientation Error: Geodesic Angle

The feasibility checker computes orientation error as the **geodesic angle** between two rotations, not a simple rotation vector difference:

$$\theta = \arccos\!\left(\frac{\mathrm{tr}(R_1^T R_2) - 1}{2}\right)$$

This is the correct metric for rotational distance (the shortest-path angle between two orientations on SO(3)). A simple `||rv_1 - rv_2||` would undercount error for large rotations because rotation vectors don't compose linearly.

### Velocity Limit Override

The feasibility checker's coarse MPC uses `max_leg_vel_mmps = 1000 mm/s` (close to hardware max ~1060), **not** the main MPC's conservative 300 mm/s. This means the feasibility check evaluates reachability at the platform's full physical capability, while the online MPC uses a lower velocity limit for smooth operation. The two limits serve different purposes:

- **Feasibility (1000 mm/s):** "Can the platform physically get there in time?"
- **Online MPC (300 mm/s):** "Can it get there smoothly within the control budget?"

## Test Sequences

Pre-defined test sequences are available via command-line flags. Each exercises specific aspects of the system:

### Trajectory Tests (`--trajectory`)

| Name | Description |
|---|---|
| T1 | Linear Z translation (active → z+50 → active), 1 s segments |
| T2 | Circular XY orbit at z=50 mm, 80 mm radius, 2 s period |
| T3 | Multi-axis translation + tilt, 1.5 s segments |
| T4 | Fast point-to-point (~400 ms transit) |
| T5 | Grand tour — 10 poses spanning the workspace |
| T6 | Extreme tour — large sweeping motions |

### Catch Tests (`--catch`)

| Name | Description |
|---|---|
| DT1 | Lateral offset + horizontal drift — exercises tilt computation |
| DT2 | Rapid succession — two catches at different XY positions |
| DT3 | Edge of workspace — ball near ~120 mm lateral reach limit |
| DT4 | Hardcoded fast drop — tests hand timing at higher speeds |
| DT5 | Moderate speed catch with angled approach |
| DT6 | Throw test — arrive with nonzero velocity, then decelerate (no ball) |
| DT7 | Catch then throw — catch a ball, hold, then throw downward |
| DT8 | Deliberate miss — ball at x=300 mm (outside reachable workspace), tests graceful failure |
| BB1-BB4 | Ball Butler throws at various targets (requires `--bb`) |

### Throw-Catch Tests (`--throw-catch`)

| Name | Description |
|---|---|
| TC1 | Vertical self-toss — throw up from [0,0,800], catch at same position, ~1 s flight |
| TC2 | Angled lateral traverse — 400 mm horizontal separation, ~1.15 s flight |
| TC3 | Fast self-toss — short 0.6 s flight time, tests timing precision |
| TC4 | Long arc — 1.4 s flight (6.86 m/s, near max throw speed) |
