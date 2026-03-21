# Throw-Catch Pipeline Plan

Implement a throw-catch cycle in the MuJoCo simulation: the platform moves to a
throw pose, the hand ejects the ball along the platform's local Z axis, the ball
flies ballistically, the platform moves to a catch pose, and the hand catches the
ball.  The platform is **stationary** during both throw and catch — all ball
velocity comes from the hand.

Ball release uses the existing **weld constraint** approach (disable weld + set
ball velocity).  Contact-based release is a separate future piece of work.

---

## User API

The user commands a single throw-catch cycle by specifying four values:

```python
plan_throw_catch(
    throw_pos_mm: np.ndarray,    # (3,) ball release position (world XYZ, mm)
    catch_pos_mm: np.ndarray,    # (3,) ball catch position (world XYZ, mm)
    throw_time: float,           # absolute sim time of ball release
    catch_time: float,           # absolute sim time of ball arrival at catch
)
```

Everything else — orientations, hand trajectories, ball velocity, platform poses,
timing margins — is computed from these four values.

---

## Physics Summary

### Ballistic inverse

Given `throw_pos`, `catch_pos`, and flight time `T = catch_time - throw_time`:

```
v_launch = (catch_pos - throw_pos) / T  +  [0, 0, -½ g T]
```

The ball must leave the hand with velocity `v_launch`.  At catch time, the ball
arrives with velocity:

```
v_arrival = v_launch + [0, 0, -g T]
```

### Platform orientation

The hand is rigidly mounted along the platform's local +Z axis.  The ball is
ejected along this axis.  Therefore the platform must be tilted so that its +Z
axis aligns with the launch velocity direction:

- **Throw orientation**: rotate platform +Z to align with `normalize(v_launch)`
- **Catch orientation**: rotate platform +Z to align with `normalize(-v_arrival)`
  (hand opening faces into the incoming ball)

Both use the same rotation-vector computation (axis-angle from `[0,0,1]` to
target direction).  The existing `_compute_catch_orientation()` already does this
for catch (with `reference = [0,0,-1]`); we generalise to a shared utility.

### Platform centroid position

The ball position is at the **hand opening**, which is offset from the platform
centroid along the tilted Z axis:

```
platform_centroid = ball_pos - hand_offset_mm * platform_z_hat
```

Where `platform_z_hat = R @ [0, 0, 1]` and `hand_offset_mm` depends on the hand
position along its stroke at the moment of interest.

The hand offset formula (from hardware_config + Trajectory.h):

```
hand_offset_mm = HAND_AXIS_BOTTOM_OFFSET_MM + hand_pos_mm + CONE_HEIGHT_MM
```

Constants:
- `HAND_AXIS_BOTTOM_OFFSET_MM = -129.0` (rail bottom in platform-local Z)
- `CONE_HEIGHT_MM = 109.0` (distance from hand body origin to cone opening)

So: `hand_offset_mm = hand_pos_mm - 20.0`

For **throw release**: hand is at `x2` position (end of constant-velocity hold
in the throw trajectory).  `x2` depends on throw speed — computed by
`HandThrowTrajectory`.

For **catch**: hand is at prime position (~323 mm from physical bottom).
`hand_offset = 323 - 20 = 303 mm`.  The existing value of 64.78 mm in
`_compute_catch_target()` uses a different derivation (x5 from the catch
trajectory's acceleration phase), which gives the hand position at the moment of
ball contact during the downward catch motion, not the prime position.  We keep
the existing catch offset logic unchanged; it is correct for its purpose.

### Hand throw trajectory (from Teensy Trajectory.h)

Three-segment profile matching production `calcThrow()`:

1. **Accelerate** (0 → v_throw): duration `t_acc = 2/(IR+1) * accel_stroke / v`,
   where `accel_stroke = 0.95 * 315 mm` and `IR = INERTIA_RATIO = 0.747`
2. **Constant velocity hold** (v_throw): duration `t_vel = vel_hold / v`,
   where `vel_hold = 0.05 * 315 = 15.75 mm`
3. **Decelerate** (v_throw → 0): duration `t_dec = t_acc * IR`

The ball is released at the end of phase 2 (transition from hold to deceleration).
At this instant the hand is at peak velocity, giving the most predictable launch.

Position milestones (mm from bottom of effective stroke):
- `x1 = 0.5 * v * t_acc` (end of acceleration)
- `x2 = x1 + vel_hold` (end of velocity hold = **release point**)
- `x3 = x2 + v * t_dec - 0.5 * (v/t_dec) * t_dec²` (end of deceleration)

The hand position at release (from physical bottom of stroke):
`hand_release_pos_mm = STROKE_MARGIN_MM + x2`

### Timing

The user's `throw_time` is when the ball is released.  Working backwards:

```
throw_trajectory_start = throw_time - t_acc - t_vel
smooth_move_end        = throw_trajectory_start
smooth_move_start      = smooth_move_end - smooth_move_duration
platform_arrive_by     = smooth_move_start - settle_margin
```

The platform must be at the throw pose before the hand throw trajectory begins.

Similarly for catch, the existing `HandCatchSequence` handles the timing: the
platform must be at catch pose before the catch trajectory's prelude begins.

Between throw and catch, the platform transitions from throw pose to catch pose.
The available transit time is approximately:

```
transit_budget = catch_platform_deadline - throw_time
```

If this is too short for the MPC to traverse the distance, the throw-catch is
infeasible.

---

## Phase Summary

| Phase | Description | Key Outcome |
|-------|-------------|-------------|
| **0** | Rename `sim/catch/` → `sim/hand/` | Terminology reflects both catch and throw |
| **1** | `HandThrowTrajectory` + `HandThrowSequence` | Port of Teensy `calcThrow()`, timing math |
| **2** | Ballistics + `ThrowCatchPlanner` | Inverse ballistics, orientation, offset, plan generation |
| **3** | Coordinator updates | New states, throw flow, ball release signaling |
| **4** | Main loop integration | Release wiring, throw-catch scripted sequences |
| **5** | Tests | Unit + integration tests |

---

## Phase 0: Rename `sim/catch/` → `sim/hand/`

Rename the package and all class prefixes to reflect that it handles both catching
and throwing.

### Renames

| Old | New |
|-----|-----|
| `sim/catch/` | `sim/hand/` |
| `CatchCoordinator` | `HandCoordinator` |
| `CatchPhase` | `HandPhase` |
| `CatchEvent` | `HandEvent` |
| `sim/catch/coordinator.py` | `sim/hand/coordinator.py` |
| `sim/catch/hand_trajectory.py` | `sim/hand/trajectory.py` |
| `sim/catch/feasibility.py` | `sim/hand/feasibility.py` |
| `sim/catch/__init__.py` | `sim/hand/__init__.py` |
| `HandCatchSequence` | Keep name (it's specifically a catch sequence) |
| `HandCatchTrajectory` | Keep name (it's specifically a catch trajectory) |

### Files requiring import updates

- `sim/main.py` — imports from `catch.*`
- `sim/input/scripted.py` — imports `DynamicTarget`, `BallSpawn`
- `sim/input/interactive_catch.py` — imports coordinator
- `sim/input/sim_control.py` — if it imports from catch
- `sim/controller/__init__.py` — if it re-exports
- `sim/tests/test_mpc_dynamic.py` — imports coordinator + targets
- `sim/tests/test_ball.py` — if it imports from catch
- `sim/tests/test_variable_horizon.py` — if it imports from catch

### Approach

Use `git mv sim/catch sim/hand`, then find-and-replace all import paths.  Run
the full test suite to verify no breakage.  Single commit, no functional changes.

---

## Phase 1: `HandThrowTrajectory` + `HandThrowSequence`

Port the Teensy `calcThrow()` to Python, mirroring the existing
`HandCatchTrajectory` / `HandCatchSequence` pattern.

### 1A. `HandThrowTrajectory` (in `sim/hand/trajectory.py`)

```python
class HandThrowTrajectory:
    """3-segment throw trajectory matching Teensy buildThrow().

    Moves the hand upward: accelerate → constant-velocity hold → decelerate.
    Ball is released at the end of the velocity-hold phase (t = t_acc + t_vel).

    Timeline: t=0 is the moment of ball release (end of velocity-hold phase).

    Parameters
    ----------
    throw_speed_mps : float
        Desired ejection speed (m/s).  Clamped to [MIN, MAX].
    start_pos_mm : float
        Hand position at trajectory start (mm from physical bottom).
        Default: bottom of effective stroke (STROKE_MARGIN_MM).
    """

    def __init__(self, throw_speed_mps: float, start_pos_mm: float | None = None): ...

    @property
    def release_time(self) -> float:
        """Always 0.0 (by timeline convention)."""

    @property
    def release_pos_mm(self) -> float:
        """Hand position at ball release (mm from physical bottom)."""

    @property
    def release_speed_mps(self) -> float:
        """Hand speed at release = throw_speed_mps (clamped)."""

    @property
    def start_time(self) -> float:
        """Time before release when trajectory starts (negative)."""

    @property
    def end_time(self) -> float:
        """Time after release when trajectory ends (positive)."""

    def sample(self, t: float) -> float:
        """Hand position (mm) at time t relative to ball release."""
```

**Internal math** (from Teensy `calcThrow`):

```python
accel_stroke = (1 - THROW_VEL_HOLD_PCT) * TOTAL_STROKE   # 0.95 * 0.315 m
vel_hold     = THROW_VEL_HOLD_PCT * TOTAL_STROKE           # 0.05 * 0.315 m

t_acc = 2 / (INERTIA_RATIO + 1) * accel_stroke / v_throw
t_vel = vel_hold / v_throw
t_dec = t_acc * INERTIA_RATIO

throwA = v_throw / t_acc          # acceleration (positive = upward)
throwD = -throwA / INERTIA_RATIO  # deceleration (negative = slowing)

x1 = 0.5 * throwA * t_acc²       # end of accel (m from effective bottom)
x2 = x1 + v_throw * t_vel         # end of vel hold = release point

# Timeline: t=0 at release → trajectory starts at -(t_acc + t_vel)
```

**Velocity method** (needed for ball release velocity verification):

```python
def velocity_at(self, t: float) -> float:
    """Hand velocity (m/s, positive = upward) at time t relative to release."""
```

### 1B. `HandThrowSequence` (in `sim/hand/trajectory.py`)

Analogous to `HandCatchSequence`: smooth-move prelude + throw trajectory.

```python
class HandThrowSequence:
    """Complete throw sequence: smooth-move to start → throw trajectory.

    Anchored to absolute sim time via ``release_time``.

    Parameters
    ----------
    throw_speed_mps : float
        Desired ejection speed (m/s).
    release_time : float
        Absolute sim time when ball should be released.
    current_pos_mm : float
        Current hand position (mm from physical bottom).
    """

    def __init__(self, throw_speed_mps, release_time, current_pos_mm): ...

    @property
    def prelude_start_time(self) -> float:
        """Absolute sim time when smooth-move begins."""

    @property
    def release_time(self) -> float:
        """Absolute sim time of ball release."""

    @property
    def end_time(self) -> float:
        """Absolute sim time when throw trajectory ends."""

    def sample(self, sim_time: float) -> float | None:
        """Hand position (mm) at sim_time.  None after sequence ends."""

    @staticmethod
    def try_create(throw_speed_mps, release_time, current_pos_mm, current_time
                   ) -> HandThrowResult:
        """Create with feasibility check (enough time for smooth-move prelude)."""
```

### 1C. Max throw speed utility

```python
def max_throw_speed_mps() -> float:
    """Maximum ejection speed achievable within the hand's stroke and accel limits.

    From v² = 2 * a * d where a = throwA and d = x2 (acceleration + hold distance).
    """
```

This is needed for feasibility checking in Phase 2.

### Files changed
- `sim/hand/trajectory.py` — add `HandThrowTrajectory`, `HandThrowSequence`,
  `HandThrowResult`, `max_throw_speed_mps()`

---

## Phase 2: Ballistics + `ThrowCatchPlanner`

### 2A. Ballistics module (in `sim/hand/ballistics.py`, new file)

```python
def compute_launch_velocity(
    release_pos_mm: np.ndarray,   # (3,) world
    target_pos_mm: np.ndarray,    # (3,) world
    flight_time_s: float,
) -> np.ndarray:
    """Ballistic inverse: required velocity at release for ball to reach target.

    Returns (3,) velocity in mm/s.
    Assumes no drag, constant gravity = [0, 0, -9806] mm/s².
    """
    dt = flight_time_s
    return (target_pos_mm - release_pos_mm) / dt - np.array([0, 0, 0.5 * 9806.0 * dt])


def compute_arrival_velocity(
    launch_vel_mms: np.ndarray,   # (3,)
    flight_time_s: float,
) -> np.ndarray:
    """Velocity at catch time given launch velocity.

    Returns (3,) velocity in mm/s.
    """
    return launch_vel_mms + np.array([0, 0, -9806.0 * flight_time_s])


def compute_orientation(target_direction: np.ndarray) -> np.ndarray:
    """Rotation vector to align platform +Z with target_direction.

    Returns (3,) rotation vector.  Returns zeros if direction is already +Z.
    Raises ValueError if angle exceeds 30 degrees (workspace limit).
    """
    # Same axis-angle math as existing _compute_catch_orientation(),
    # but generalised: reference = [0, 0, 1], target = normalize(target_direction)


def compute_hand_offset_mm(hand_pos_mm: float) -> float:
    """Offset from platform centroid to hand opening along platform local Z.

    hand_pos_mm: hand position in mm from physical bottom of stroke.
    Returns: offset in mm (positive = above centroid).

    Formula: HAND_AXIS_BOTTOM_OFFSET_MM + hand_pos_mm + CONE_HEIGHT_MM
    """
    return -129.0 + hand_pos_mm + 109.0
```

### 2B. `ThrowCatchPlanner` (in `sim/hand/planner.py`, new file)

This is the central planner that converts user input into a sequence of
`DynamicTarget`s and hand trajectories.

```python
@dataclass
class ThrowCatchPlan:
    """Complete plan for a throw-catch cycle."""
    throw_target: DynamicTarget     # MPC target: go to throw pose
    catch_target: DynamicTarget     # MPC target: go to catch pose
    throw_hand_seq: HandThrowSequence
    catch_hand_seq: HandCatchSequence
    ball_release_time: float        # absolute sim time to disable weld
    ball_release_vel_mms: np.ndarray  # (3,) ball velocity at release
    ball_spawn: BallSpawn | None    # for sim visualisation (spawn at release)


class ThrowCatchPlanner:
    """Plans a throw-catch cycle from user-specified ball positions and times.

    Parameters
    ----------
    platform_height_mm : float
        Home position Z (574.3 mm).
    """

    def __init__(self, platform_height_mm: float = 574.3): ...

    def plan(
        self,
        throw_pos_mm: np.ndarray,    # (3,) ball release position
        catch_pos_mm: np.ndarray,    # (3,) ball catch position
        throw_time: float,
        catch_time: float,
        current_hand_pos_mm: float = 0.0,
        current_time: float = 0.0,
    ) -> ThrowCatchPlan:
        """Compute a complete throw-catch plan.

        Raises ValueError if infeasible (throw speed exceeds hand limits,
        tilt exceeds workspace, insufficient transit time, etc.).
        """
```

**`plan()` algorithm:**

1. **Validate inputs**: `catch_time > throw_time`, positions within plausible range.

2. **Compute launch velocity**:
   ```python
   v_launch = compute_launch_velocity(throw_pos_mm, catch_pos_mm, flight_time)
   ```

3. **Compute throw orientation**:
   ```python
   throw_rv = compute_orientation(v_launch)   # tilt +Z toward launch direction
   ```

4. **Compute required hand throw speed**:
   The hand ejects along platform +Z.  The launch velocity projected onto the
   platform +Z axis gives the hand speed:
   ```python
   platform_z = R_throw @ [0, 0, 1]
   hand_throw_speed_mps = np.dot(v_launch, platform_z) / 1000.0  # mm/s → m/s
   ```
   Since we've aligned platform +Z with `v_launch`, this equals `|v_launch| / 1000`.
   Validate: `hand_throw_speed_mps <= max_throw_speed_mps()`.

5. **Build throw hand trajectory**:
   ```python
   throw_hand = HandThrowSequence(hand_throw_speed_mps, throw_time, current_hand_pos_mm)
   ```

6. **Compute throw platform centroid**:
   ```python
   x2_mm = throw_hand.throw_trajectory.release_pos_mm  # hand pos at release
   throw_offset = compute_hand_offset_mm(x2_mm)
   throw_centroid = throw_pos_mm - throw_offset * platform_z_throw
   throw_pose_6dof = [centroid_x, centroid_y, centroid_z - platform_height, rv_x, rv_y, rv_z]
   ```

7. **Compute arrival velocity and catch orientation**:
   ```python
   v_arrival = compute_arrival_velocity(v_launch, flight_time)
   catch_rv = compute_orientation(-v_arrival)  # face into incoming ball
   ```

8. **Compute catch platform centroid**:
   Use existing catch offset (x5-based, from `HandCatchTrajectory`):
   ```python
   catch_offset = compute_hand_offset_mm(catch_hand_x5_mm)
   catch_centroid = catch_pos_mm - catch_offset * platform_z_catch
   catch_pose_6dof = [centroid_x, centroid_y, centroid_z - platform_height, rv_x, rv_y, rv_z]
   ```

9. **Build catch hand trajectory**:
   ```python
   event_vel_mps = np.linalg.norm(v_arrival) / 1000.0
   catch_hand = HandCatchSequence(event_vel_mps, catch_time, throw_hand.end_pos_mm)
   ```
   Note: the catch hand trajectory starts from wherever the hand ends up after
   the throw (not from prime position).  The smooth-move prelude handles the
   transition.

10. **Build DynamicTargets**:
    ```python
    throw_target = DynamicTarget(
        pose_6dof=throw_pose_6dof,
        arrival_time=throw_hand.prelude_start_time - settle_margin,
        arrival_twist=None,          # platform stationary
        hold_duration=0.0,           # no hold — throw immediately
        event_vel_mps=hand_throw_speed_mps,
        mode='throw',                # new field — signals throw flow
    )
    catch_target = DynamicTarget(
        pose_6dof=catch_pose_6dof,
        arrival_time=catch_time,
        arrival_twist=None,          # platform stationary
        hold_duration=0.5,
        event_vel_mps=event_vel_mps,
        mode='catch',                # default
    )
    ```

11. **Build BallSpawn** (for sim visualisation):
    ```python
    ball_spawn = BallSpawn(
        position_mm=throw_pos_mm,
        velocity_mms=v_launch,
        spawn_time=throw_time,       # ball "appears" at release
    )
    ```

12. **Feasibility checks** (raise `ValueError` on failure):
    - Throw speed within hand limits
    - Throw/catch orientations within 30-degree tilt limit
    - Platform centroid positions within workspace (IK check)
    - Transit time (throw → catch) sufficient for MPC to traverse the distance
    - Hand timing budgets (smooth-move preludes fit within available time)

### Files created
- `sim/hand/ballistics.py`
- `sim/hand/planner.py`

### Files changed
- `sim/hand/__init__.py` — export new classes

---

## Phase 3: Coordinator Updates

### 3A. Add `mode` field to `DynamicTarget`

```python
@dataclass
class DynamicTarget:
    pose_6dof: np.ndarray
    arrival_time: float | None = None
    arrival_twist: np.ndarray | None = None
    hold_duration: float = 0.5
    event_vel_mps: float | None = None
    settle_margin_s: float = 0.1
    mode: str = 'catch'              # 'catch' or 'throw'
```

The `mode` field replaces the `arrival_twist` check for distinguishing throw
from catch.  The platform is stationary in both modes, so `arrival_twist` is
always None/zero.  The old DT6/DT7 non-zero `arrival_twist` throw tests will be
replaced with new planner-based tests.

### 3B. New `HandPhase` states

```python
class HandPhase(Enum):
    IDLE = auto()
    APPROACHING_THROW = auto()   # Platform moving to throw pose
    THROWING = auto()            # Hand executing throw trajectory
    TRANSITIONING = auto()       # Platform moving from throw to catch pose
    APPROACHING_CATCH = auto()   # Platform moving to catch pose
    HOLDING = auto()             # At catch pose, waiting for ball
    CAUGHT = auto()              # Ball captured, retracting hand
    RETURNING = auto()           # Returning to home
```

This replaces the current `PRIMING → APPROACHING → HOLDING/THROWING →
DECELERATING → RETURNING` flow.  The old `PRIMING` state (hand to prime) is
folded into `APPROACHING_CATCH` (the smooth-move prelude handles hand motion).
`DECELERATING` is removed (platform is always stationary at throw/catch).

### 3C. `HandCoordinator.update()` flow for throw-catch

The coordinator accepts a `ThrowCatchPlan` (from the planner) in addition to
raw `DynamicTarget` + `BallSpawn` pairs:

```python
def submit_throw_catch(self, plan: ThrowCatchPlan) -> None:
    """Queue a throw-catch plan."""
    self._throw_catch_plan = plan
    # Internally, queue the throw target as the first target
    self._targets.append((plan.throw_target, None))
    # Catch target is queued after throw completes
```

**State transitions:**

```
IDLE
  → submit_throw_catch(plan)
  → APPROACHING_THROW
      MPC target: plan.throw_target
      Hand cmd: plan.throw_hand_seq (smooth-move to start, then throw)

APPROACHING_THROW
  → platform at throw pose AND sim_time >= throw_hand_seq.prelude_start_time
  → THROWING
      Hand is executing throw trajectory (sampled each step)
      Wait for ball_release_time

THROWING
  → sim_time >= plan.ball_release_time
  → emit BallRelease(velocity=plan.ball_release_vel_mms)
  → TRANSITIONING
      MPC target: plan.catch_target (arrival_time = catch_time)
      Hand cmd: plan.catch_hand_seq begins (smooth-move from post-throw to prime)

TRANSITIONING
  → platform settled at catch pose (or arrival_time reached)
  → APPROACHING_CATCH
      Hand executing catch trajectory

APPROACHING_CATCH
  → sim_time >= catch_time - hold_margin
  → HOLDING

HOLDING
  → ball captured (notify_capture called)
  → CAUGHT

CAUGHT
  → hand_cmd = 'home'
  → RETURNING

RETURNING
  → returned near home
  → IDLE
```

### 3D. Ball release signaling

Add a new hand command type for ball release:

```python
@dataclass
class BallRelease:
    """Signal to release the ball with a specific velocity."""
    velocity_mms: np.ndarray   # (3,) world-frame ball velocity
```

The coordinator emits this as `hand_cmd` at the release instant.  The main loop
dispatches on type (like existing `HandCatchSequence` / `str` / `float`
dispatch).

### 3E. Backward compatibility

The existing `submit_target()` API continues to work for catch-only sequences
(DT1-DT5, DT8).  The coordinator detects `mode='catch'` and follows the
existing catch-only flow (APPROACHING_CATCH → HOLDING → CAUGHT → RETURNING),
bypassing the throw states entirely.

### Files changed
- `sim/hand/coordinator.py` — new states, throw-catch flow, BallRelease signal

---

## Phase 4: Main Loop Integration

### 4A. Handle `BallRelease` in `_execute_hand_cmd()`

```python
def _execute_hand_cmd(plant, hand_cmd, ...):
    ...
    if isinstance(hand_cmd, BallRelease):
        plant.release_ball(hand_cmd.velocity_mms)
    ...
```

`MuJoCoPlant.release_ball()` already exists as `BallManager.release()` — it
disables the weld constraint and sets ball velocity.  We just need to make sure
it's exposed through the plant interface.

### 4B. Handle `HandThrowSequence` in `_execute_hand_cmd()`

```python
if isinstance(hand_cmd, HandThrowSequence):
    active_throw_seq = hand_cmd   # track active throw sequence

# In the step loop, sample active throw sequence:
if active_throw_seq is not None:
    pos = active_throw_seq.sample(sim_time)
    if pos is not None:
        plant.command_hand(pos)
    else:
        active_throw_seq = None
```

This mirrors the existing `active_hand_seq` pattern for `HandCatchSequence`.
Both types can share the same active-sequence tracking (only one is active at a
time).

### 4C. `CatchTargetSource` / `InteractiveCatchSource` updates

The `CatchTargetSource` adapter wraps the coordinator for the main loop.  It
needs to handle the new `BallRelease` hand command by passing it through in
`TargetCommand.hand_cmd`.

The `TargetCommand` dataclass already accepts arbitrary hand commands
(`str | HandCatchSequence | float | None`).  Extend the type hint to include
`BallRelease` and `HandThrowSequence`.

### 4D. New scripted test sequences

Replace DT6 (throw-only) and DT7 (catch-then-throw) with planner-based
throw-catch sequences:

```python
def make_DT6() -> ...:
    """DT6: Throw-catch — throw ball up, catch it as it comes back down.

    Throw from [0, 0, 700] upward, ball arcs and lands at [0, 0, 700].
    Vertical throw (no tilt), ~0.5s flight time.
    """
    planner = ThrowCatchPlanner()
    plan = planner.plan(
        throw_pos_mm=np.array([0.0, 0.0, 700.0]),
        catch_pos_mm=np.array([0.0, 0.0, 700.0]),
        throw_time=1.0,
        catch_time=2.0,
    )
    ...

def make_DT7() -> ...:
    """DT7: Angled throw-catch — throw at an angle, catch at offset position.

    Tests platform tilt for both throw and catch orientations.
    """
    planner = ThrowCatchPlanner()
    plan = planner.plan(
        throw_pos_mm=np.array([0.0, 0.0, 750.0]),
        catch_pos_mm=np.array([50.0, -30.0, 700.0]),
        throw_time=1.0,
        catch_time=2.5,
    )
    ...
```

Additional test scenarios:
- **DT9: Fast throw-catch** — short flight time (~0.5s), tests timing precision
- **DT10: High arc** — long flight time (~2s), tests slow throw + high catch

### 4E. `--throw-catch` CLI flag

Add `--throw-catch DT6|DT7|DT9|DT10` to `sim/main.py`, similar to existing
`--catch DT1|DT2|...`.

### Files changed
- `sim/main.py` — `_execute_hand_cmd()`, hand sequence tracking, CLI flag
- `sim/input/scripted.py` — new DT6/DT7/DT9/DT10 using planner

---

## Phase 5: Tests

### 5A. Unit tests — `sim/tests/test_throw_trajectory.py` (new)

```
TestHandThrowTrajectory:
  - test_vertical_throw_3mps        — positions/velocities match Teensy math
  - test_release_at_t_zero          — sample(0.0) == release_pos_mm
  - test_velocity_at_release        — velocity_at(0.0) == throw_speed
  - test_starts_at_bottom           — sample(start_time) == start_pos_mm
  - test_deceleration_to_zero       — velocity_at(end_time) ≈ 0
  - test_speed_clamping             — speeds outside [0.3, 7.0] are clamped
  - test_max_throw_speed            — max_throw_speed_mps() returns correct value

TestHandThrowSequence:
  - test_smooth_move_prelude        — transitions from current to throw start
  - test_release_timing             — release_time matches input
  - test_feasibility_check          — try_create() rejects insufficient time budget
  - test_end_position               — hand stops after deceleration
```

### 5B. Unit tests — `sim/tests/test_ballistics.py` (new)

```
TestBallisticInverse:
  - test_vertical_throw             — throw straight up, catch at same XY
  - test_angled_throw               — throw at 45 degrees
  - test_round_trip                 — launch_vel → arrival_vel → inverse matches
  - test_short_flight               — 0.3s flight
  - test_long_flight                — 3.0s flight

TestOrientation:
  - test_vertical_no_tilt           — upward launch → zero rotation
  - test_angled_tilt                — 20° off vertical → correct rotation vector
  - test_exceeds_limit              — > 30° raises ValueError
  - test_catch_orientation          — -v_arrival gives correct catch tilt

TestHandOffset:
  - test_at_prime                   — matches existing 303 mm value
  - test_at_throw_release           — correct for typical x2 position
  - test_at_home                    — hand at 0 → offset = -20 mm
```

### 5C. Unit tests — `sim/tests/test_planner.py` (new)

```
TestThrowCatchPlanner:
  - test_vertical_throw_catch       — symmetric throw/catch, zero tilt
  - test_angled_throw_catch         — both poses tilted correctly
  - test_platform_centroids         — centroid positions account for hand offset
  - test_ball_release_velocity      — matches ballistic inverse
  - test_catch_arrival_velocity     — consistent with launch + gravity
  - test_infeasible_speed           — rejects throw requiring > max hand speed
  - test_infeasible_tilt            — rejects > 30° approach angle
  - test_infeasible_transit         — rejects when transit time too short
  - test_hand_timing_chain          — throw seq ends before catch seq starts
```

### 5D. Integration tests — `sim/tests/test_mpc_dynamic.py` (extended)

```
TestDT6ThrowCatch:
  - test_platform_reaches_throw_pose    — position error < 5 mm at throw time
  - test_ball_released_with_velocity    — ball velocity matches plan
  - test_ball_flight_matches_ballistic  — ball lands near catch_pos (< 10 mm)
  - test_platform_reaches_catch_pose    — position error < 5 mm at catch time
  - test_ball_captured                  — weld activated during catch
  - test_returns_home                   — platform returns near home after cycle

TestDT7AngledThrowCatch:
  - test_throw_tilt                     — platform tilted at throw time
  - test_catch_tilt                     — platform tilted at catch time
  - test_ball_captured                  — catch succeeds despite angled flight
```

### Files created
- `sim/tests/test_throw_trajectory.py`
- `sim/tests/test_ballistics.py`
- `sim/tests/test_planner.py`

### Files changed
- `sim/tests/test_mpc_dynamic.py` — new DT6/DT7 integration tests

---

## Implementation Order

```
Phase 0: Rename sim/catch → sim/hand
    git mv + import updates + test suite green
    Single commit, no functional changes

Phase 1: HandThrowTrajectory + HandThrowSequence
    1A. HandThrowTrajectory (port of calcThrow)
    1B. HandThrowSequence (smooth-move + throw)
    1C. max_throw_speed_mps()
    → test_throw_trajectory.py green

Phase 2: Ballistics + ThrowCatchPlanner
    2A. ballistics.py (inverse, orientation, offset)
    2B. planner.py (ThrowCatchPlan, ThrowCatchPlanner)
    → test_ballistics.py + test_planner.py green

Phase 3: Coordinator updates
    3A. DynamicTarget.mode field
    3B. New HandPhase states
    3C. Throw-catch state machine flow
    3D. BallRelease signaling
    3E. Backward compat for catch-only sequences
    → existing catch tests still pass

Phase 4: Main loop integration
    4A-4B. _execute_hand_cmd handles BallRelease + HandThrowSequence
    4C. TargetCommand type hints
    4D. New scripted DT6/DT7/DT9/DT10
    4E. --throw-catch CLI flag
    → test_mpc_dynamic.py integration tests green

Phase 5: Full test suite
    → All tests pass (existing + new)
```

Phases 1 and 2 are independent and can be developed in parallel.
Phase 3 depends on both (needs HandThrowSequence from 1 and ThrowCatchPlan from 2).
Phase 4 depends on Phase 3.

---

## Risk Assessment

### Low risk
- **Ballistic inverse**: simple closed-form formula, easy to validate
- **HandThrowTrajectory**: direct port of Teensy code, well-understood math
- **Orientation computation**: reuses existing catch orientation logic

### Medium risk
- **Hand timing chain**: the throw sequence must end, hand must transit to catch
  position, and catch sequence must start — all within the flight time.  Short
  flight times may be infeasible due to hand smooth-move durations.
- **Platform transit time**: the MPC must move from throw pose to catch pose
  during ball flight.  If the poses are far apart and flight time is short, the
  MPC may not arrive in time.  The feasibility check should catch this, but the
  transit-time estimate is approximate.
- **Weld release timing**: the ball velocity is set discretely at one instant.
  The 20ms MPC timestep means up to ±10ms timing jitter on release.  For a 3 m/s
  throw, this is ±30mm position error at the catch end — likely acceptable.

### Low risk (deferred)
- **Contact-based release**: deliberately excluded from this plan.  The weld
  approach is sufficient for validating the throw-catch pipeline.  Contact
  physics is a separate, self-contained upgrade.

---

## Implementation Results (2026-03-20)

All phases implemented and tested.  **206 tests pass** (160 existing + 46 new).

### Phase 0: Rename — DONE
- `git mv sim/catch sim/hand` + `hand_trajectory.py` → `trajectory.py`
- Class renames: `CatchCoordinator` → `HandCoordinator`, `CatchPhase` → `HandPhase`, `CatchEvent` → `HandEvent`
- All import paths updated across 8 files
- 160/160 tests pass — no functional changes

### Phase 1: HandThrowTrajectory + HandThrowSequence — DONE
- Added to `sim/hand/trajectory.py`: `HandThrowTrajectory`, `HandThrowSequence`, `HandThrowResult`, `max_throw_speed_mps()`
- Timeline convention: t=0 at ball release (end of velocity-hold phase)
- Constants: `THROW_VEL_HOLD_PCT = 0.05`, `STROKE_MARGIN_MM = 20.0`
- 3 m/s throw: t_acc=0.114s, t_vel=0.005s, t_dec=0.085s, release at 207mm from bottom

### Phase 2: Ballistics + ThrowCatchPlanner — DONE
- New file `sim/hand/ballistics.py`: `compute_launch_velocity()`, `compute_arrival_velocity()`, `compute_orientation()`, `compute_hand_offset_mm()`, `rodrigues()`
- New file `sim/hand/planner.py`: `ThrowCatchPlanner`, `ThrowCatchPlan` dataclass
- Orientation uses same axis-angle approach as existing catch code, generalised to both throw and catch directions
- Hand offset formula: `HAND_AXIS_BOTTOM_OFFSET_MM + hand_pos_mm + CONE_HEIGHT_MM = hand_pos_mm - 20.0`

### Phase 3: Coordinator Updates — DONE
- Added `mode: str = 'catch'` field to `DynamicTarget`
- Added `BallRelease` dataclass for ball release signaling
- New `HandPhase` states: `APPROACHING_THROW`, `TRANSITIONING`, `APPROACHING_CATCH`
- Added `submit_throw_catch(plan)` method
- Backward compatibility: `is_catch_mode` checks both `mode` field and legacy `arrival_twist` — existing DT1-DT8 tests unchanged

### Phase 4: Main Loop Integration — DONE
- `_execute_hand_cmd()` handles `BallRelease` (calls `ball_manager.release()`) and `HandThrowSequence`
- Added `BallManager.spawn_in_hand()` method for welding ball at start of throw
- New `ThrowCatchTargetSource` adapter wraps `ThrowCatchPlan` for the MPC loop
- Scripted sequences: `TC1` (vertical), `TC2` (angled), `TC3` (fast 0.6s flight), `TC4` (near-max 1.4s)
- `--throw-catch TC1|TC2|TC3|TC4` CLI flag added

### Phase 5: Tests — DONE (46 new tests)
- `sim/tests/test_throw_trajectory.py` — 18 tests (HandThrowTrajectory + HandThrowSequence)
- `sim/tests/test_ballistics.py` — 15 tests (inverse ballistics, orientation, hand offset, Rodrigues)
- `sim/tests/test_planner.py` — 13 tests (planner pipeline, feasibility, consistency)

### Key Discoveries

1. **Max throw speed is 7.0 m/s** (clamped by `MAX_EVENT_VEL_MPS`).  For symmetric throw/catch at the same height, `v = 0.5 * g * T`, so max flight time ≈ 1.43s.  Longer flights need higher speed and are infeasible.  This is a fundamental physical constraint of the hand mechanism.

2. **Throw-catch sequence naming**: Used `TC1-TC4` instead of overwriting existing `DT6/DT7` since those legacy tests exercise the throw-with-velocity and catch-then-throw-with-velocity paths (non-zero `arrival_twist`), which is a different flow from the planner-based throw-catch.

3. **Ball lifecycle**: For throw-catch, the ball starts welded in the hand (`spawn_in_hand()`), gets released via `BallRelease` at throw time, flies ballistically, and is caught by the standard capture mechanism.  This is different from catch-only sequences where the ball is spawned free in the air.

4. **Integration tests**: The new `TC1-TC4` sequences are defined and can be run via `--throw-catch TC1`, but full MPC integration tests (like the existing `TestDT*` classes) are not yet added to `test_mpc_dynamic.py`.  The unit tests validate the math pipeline completely; MPC integration tests should be added when running the actual sim confirms the flow works end-to-end.

### Items for Further Investigation

- **MPC integration tests**: Add `TestTC1ThrowCatch` etc. to `test_mpc_dynamic.py` after confirming the visual sim works correctly with `--throw-catch TC1`
- **Weld release timing jitter**: The 20ms control timestep means the ball release happens on the nearest step boundary, not exactly at `throw_time`.  With a 3 m/s throw, this is ±30mm position error at the catch end.  May need sub-step release or interpolation for high-precision throws.
- **Throw from non-home position**: Currently `spawn_in_hand()` is called once at startup.  For repeated throw-catch cycles, the ball needs to be re-welded after catch — this requires a `catch → re-weld → throw` flow that isn't implemented yet.
- **Contact-based release**: The weld release approach works but is discrete.  A contact-based approach (hand pushes ball out) would give smoother, more physically realistic release dynamics.
