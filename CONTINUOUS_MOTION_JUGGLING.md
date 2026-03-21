# Continuous-Motion Juggling

Design document for transitioning Jugglebot from stop-and-throw/catch to continuous
platform motion during juggling.

---

## Context

### What Jugglebot is

Jugglebot is a **Stewart platform** — a 6-DoF parallel robot where six linear
actuators (legs) connect a fixed base to a movable platform.  By extending and
retracting the legs, the platform can translate in XYZ and rotate in roll/pitch/yaw.

Mounted on top of the platform is a **linear hand** — a single-axis actuator that
slides a conical ball cup along the platform's local Z axis.  The hand has a 355 mm
stroke (315 mm effective after margins) and can throw a ball upward at up to 7.0 m/s.
The hand catches by descending at 90% of the incoming ball's speed, then decelerating.

**Home position:** platform centroid at Z = 574.3 mm above base, legs partially
extended (~648 mm).  All poses are expressed relative to home: `[x, y, z, rx, ry, rz]`
in mm and radians, where `[0,0,0,0,0,0]` = home.

**Coordinate system:** X = lateral, Y = forward/back, Z = up.  Gravity = -Z.

### Simulation environment

The simulation uses **MuJoCo** for physics, running at 50 Hz control rate with
multiple physics substeps per control step.  Entry point: `sim/main.py`.

Key existing modes:
- `--throw-catch TC1..TC4` — single throw-catch cycle (one-shot, for testing)
- `--juggle` — continuous throw-catch loop (existing, uses flight_time as knob)
- `--interactive-catch` — spawn balls on demand with keyboard

The sim uses a **TargetSource** protocol: any input module provides an `update()`
method that returns a `TargetCommand` each control step.  `TargetCommand` contains:
- `target_pose` — 6-DoF platform target for the MPC
- `arrival_time` — when to arrive (None = ASAP)
- `target_twist` — velocity at arrival (None = stop)
- `hand_cmd` — hand trajectory / position / release command
- `ball_spawn` — spawn a ball this step (or None)

The MPC solver receives the target and plans smooth platform motion to reach it,
respecting leg stroke, velocity, and acceleration limits.

### Throw-catch pipeline (existing)

The throw-catch cycle is already implemented across several modules:

- **`sim/hand/planner.py`** — `ThrowCatchPlanner`: given throw position, catch
  position, throw time, and catch time, computes everything: launch velocity
  (inverse ballistics), platform orientations (tilt hand toward target), hand
  trajectories (throw and catch sequences), platform centroid positions (offset
  from ball position by hand geometry), and timing chain validation.

- **`sim/hand/trajectory.py`** — `HandThrowTrajectory` / `HandCatchTrajectory`:
  3-segment acceleration profiles (accelerate → constant-velocity hold → decelerate),
  ported from the Teensy firmware.  `HandThrowSequence` / `HandCatchSequence` add
  a smooth-move prelude to position the hand before the main trajectory starts.

- **`sim/hand/coordinator.py`** — `HandCoordinator`: state machine that sequences
  throw and catch events.  Manages hand priming, trajectory execution, ball capture
  notification, and return-to-home.  Accepts a `ThrowCatchPlan` via
  `submit_throw_catch()` and drives the hand through the full cycle.

- **`sim/hand/ballistics.py`** — inverse ballistics (`compute_launch_velocity`),
  arrival velocity, orientation computation, hand offset geometry.

- **`sim/ball/manager.py`** — `BallManager`: ball spawn (teleport + set velocity),
  proximity-based capture detection (runs every physics substep), kinematic hold
  (ball locked to hand frame), release (unlock + set velocity + cooldown).

- **`sim/input/continuous_throw_catch.py`** — `ContinuousThrowCatchController`:
  existing continuous loop that repeatedly calls `ThrowCatchPlanner` and manages
  the cycle state machine (STARTUP → CYCLE_ACTIVE → BETWEEN_CYCLES → DROPPED).
  Currently parameterised by throw/catch XY positions, ball Z offset, and flight
  time.  Invoked via `--juggle`.

### What's missing / what this plan changes

The existing `--juggle` mode uses **flight_time** and **absolute positions** as its
control knobs.  It also uses fixed `_APPROACH_TIME = 2.0 s` and
`_INTER_CYCLE_PAUSE = 0.3 s` constants that are not derived from any timing model.

This plan introduces a **timing model** based on `cycle_time` and `hold_ratio`,
where the rhythm of the toss cycle is the primary control parameter.  It also
removes the requirement that the platform be stationary during throws and catches,
enabling continuous motion.

---

## Phase Summary

| Phase | Description | Complexity | Key work |
|-------|-------------|------------|----------|
| **A** | Vertical toss-to-self (stationary platform) | Low | Refactor `--juggle` parameterisation: swap `flight_time` → `cycle_time + hold_ratio`, derive timing from model instead of fixed constants. All infrastructure exists. |
| **B** | Toss between positions (platform moves) | Low–moderate | Add position selection logic and workspace envelope check. `ThrowCatchPlanner` already handles different positions with tilt; MPC already tracks dynamic targets with deadlines. |
| **C** | Velocity-at-events (continuous motion) | Moderate | New physics: ballistic equations account for platform velocity, settle margin removed, hand trajectories fire while platform moves. First phase requiring new thinking rather than rearranging existing pieces. |
| **D** | Orbit optimisation | High (if needed) | Fourier orbit optimiser + time-varying MPC tracking weights. Only pursue if Phase C reveals excessive jerk from MPC-planned transits. May be skipped entirely. |

---

## Motivation

The current throw-catch pipeline requires the platform to arrive at each event pose
and **settle for 100 ms** before the hand acts.  For repeated tosses this produces
aggressive start-stop acceleration profiles that waste energy, increase jerk, and
stress the hardware.

Human jugglers keep their hands moving continuously; the catch, carry, and throw are
all part of one fluid motion.  This document describes how to replicate that approach,
starting with the simplest possible pattern and building up.

### Design priorities (in order)

1. **Low per-leg jerk** — smooth leg motion = hardware longevity + mechanical quiet.
2. **Accurate event execution** — throws and catches must hit their pose and velocity
   targets.
3. **Energy efficiency** — continuous motion avoids start-stop waste.

---

## Pattern: Single-Ball Toss-to-Self

Jugglebot repeatedly throws a single ball upward, catches it, and throws again.
Early phases throw straight up from a fixed position; later phases toss the ball
between different positions around the workspace.

This is the simplest pattern that exercises all the core capabilities needed for
continuous-motion juggling: throwing while moving, catching while moving, ballistic
prediction, and the full throw-catch hand trajectory pipeline.  It requires only a
refactor of the existing `--juggle` mode (no new infrastructure), is entirely
self-paced (no external timing to synchronise with), and failure recovery is trivial
(drop → respawn → resume).

---

## Timing Model

### The hold_ratio knob

Two control parameters set the rhythm of the toss cycle:

```
cycle_time  = air_time + hold_time
hold_ratio  = hold_time / cycle_time        (default 0.4)
air_time    = (1 - hold_ratio) × cycle_time
hold_time   = hold_ratio × cycle_time
```

`cycle_time` is the primary free parameter.  `hold_ratio` defaults to 0.4 (the ball
spends slightly more time in the air than in the hand) but can be adjusted.

Given `cycle_time` and `hold_ratio`, the throw speed and height are fully determined
for a vertical toss:

```
v_launch = g × air_time / 2                 (symmetric: throw speed = catch speed)
height   = g × air_time² / 8
```

### What happens during each phase

The hand trajectory system defines rigid timing for the throw and catch.  Each is a
3-segment profile (accelerate → constant-velocity hold → decelerate) whose duration
depends on the ball speed.  These durations eat into the air and hold phases:

**Air phase** — ball is in flight, hand is empty:
1. **Throw post-release** — hand decelerating after ball ejection (ends near top of stroke)
2. **Free** — hand repositioning to catch start (near top of stroke for vertical toss ≈ 0 ms)
3. **Catch pre-arrival** — hand accelerating downward before ball arrives

**Hold phase** — ball is in the hand:
1. **Catch post-arrival** — hand decelerating with ball + 100 ms end hold (ends near bottom)
2. **Dwell** — ball sitting in hand, platform free to reposition
3. **Throw pre-release** — hand accelerating upward toward release (starts near bottom)

The **dwell time** is the slack in the hold phase after subtracting the hand trajectory
durations.  It must be non-negative for the cycle to be feasible.

```
dwell = hold_time − catch_post_arrival − throw_pre_release
```

Note: for a vertical toss, the throw ends near the top of the stroke and the catch
starts near the top, so there is no smooth-move prelude needed during air time.
Similarly, the catch ends near the bottom and the throw starts near the bottom.
This symmetry means dwell is the only timing constraint.

### Timing budget

For a vertical toss at `hold_ratio = 0.4`, computed from the actual hand trajectory
code (`sim/hand/trajectory.py`):

| cycle_time | air_time | hold_time | v_launch | height  | dwell   | status     |
|------------|----------|-----------|----------|---------|---------|------------|
| 0.8 s      | 0.48 s   | 0.32 s    | 2.35 m/s | 282 mm  | −93 ms  | INFEASIBLE |
| 0.9 s      | 0.54 s   | 0.36 s    | 2.65 m/s | 357 mm  | −18 ms  | INFEASIBLE |
| **1.0 s**  | 0.60 s   | 0.40 s    | 2.94 m/s | 441 mm  | 50 ms   | OK (tight) |
| **1.2 s**  | 0.72 s   | 0.48 s    | 3.53 m/s | 635 mm  | 171 ms  | OK         |
| 1.5 s      | 0.90 s   | 0.60 s    | 4.41 m/s | 993 mm  | 333 ms  | OK         |
| 2.0 s      | 1.20 s   | 0.80 s    | 5.88 m/s | 1765 mm | 575 ms  | OK         |

**Minimum feasible cycle_time at hold_ratio = 0.4: ~0.93 s** (dwell → 0).

The infeasibility below 0.93 s is driven by the hand trajectory durations: the catch
post-arrival phase alone takes 229 ms at 2.94 m/s (mostly the 100 ms end-of-profile
hold), and the throw pre-release takes 122 ms.  Together these consume 351 ms of the
400 ms hold time, leaving only 50 ms of dwell.

### Hand trajectory durations (reference)

These are derived from the Teensy-ported trajectory code and scale as ~1/v:

| v (m/s) | throw total | pre-release | post-release | catch total | pre-arrival | post-arrival |
|---------|-------------|-------------|-------------|-------------|-------------|-------------|
| 2.94    | 209 ms      | 122 ms      | 87 ms       | 326 ms      | 98 ms       | 229 ms      |
| 3.53    | 174 ms      | 102 ms      | 72 ms       | 288 ms      | 81 ms       | 207 ms      |
| 4.41    | 139 ms      | 81 ms       | 58 ms       | 251 ms      | 65 ms       | 186 ms      |
| 5.88    | 104 ms      | 61 ms       | 44 ms       | 213 ms      | 49 ms       | 164 ms      |

**Pre-release / pre-arrival** = time before the event (ball release / ball arrival)
when the hand trajectory starts.  This eats into the preceding phase.

**Post-release / post-arrival** = time after the event when the hand trajectory
finishes.  This eats into the following phase.

### Recommended starting point

**cycle_time = 1.2 s, hold_ratio = 0.4**

- 0.72 s air time, 0.48 s hold time — comfortable margins
- 3.53 m/s throw speed — well within 7.0 m/s hand max
- 635 mm throw height — visible, moderate
- 171 ms dwell — enough slack for timing jitter and drop recovery

---

## Ball Lifecycle (toss-to-self)

The ball cycles through four states: **Dwelling → Throwing → In Flight → Catching →
Dwelling → ...**

1. **Dwelling** — ball in hand at bottom of stroke, platform at rest or repositioning.
   Duration = dwell time (the slack from the timing budget).
2. **Throwing** — hand accelerates upward; ball released at throw speed at the end of
   the velocity-hold phase.  Ball becomes free-flying.
3. **In flight** — ball follows ballistic arc (no drag).  Hand decelerates, then
   repositions for catch.  Platform may transit to a new position.
4. **Catching** — hand descends at 90% of ball speed, captures ball via proximity
   detection, decelerates to rest.

**Drop recovery:** if the ball falls below ground Z, it is despawned and respawned in
the hand.  The cycle restarts from Dwelling.  No synchronisation is needed because the
robot sets its own pace.

---

## Implementation Phases

### Phase A: Vertical toss-to-self (stationary platform)

**Goal:** the robot stands still at home and tosses a ball straight up and down,
repeatedly, governed by the `cycle_time` / `hold_ratio` timing model.

**What exists:** `ContinuousThrowCatchController` in
`sim/input/continuous_throw_catch.py` already loops throw-catch cycles via the
`--juggle` CLI flag.  It uses `JuggleParams` (throw/catch XY, ball Z, flight_time)
and fixed timing constants (`_APPROACH_TIME = 2.0 s`, `_INTER_CYCLE_PAUSE = 0.3 s`).

**What changes:** refactor `ContinuousThrowCatchController` (or create a new
`TossLoopController` alongside it) to:

1. **Replace `flight_time` with `cycle_time` + `hold_ratio`.**  Derive air_time,
   hold_time, throw speed, and catch speed from these two parameters.

2. **Compute timing from the model, not fixed constants.**  Replace
   `_APPROACH_TIME` and `_INTER_CYCLE_PAUSE` with values derived from the timing
   budget:
   - `throw_time = last_catch_end + dwell`  (catch_end = when catch trajectory
     finishes, dwell = remaining hold time after accounting for throw pre-release)
   - `catch_time = throw_time + air_time`
   - No arbitrary pause — the timing model drives everything.

3. **Validate feasibility** at startup and when parameters change.  Compute
   minimum cycle_time for the given hold_ratio and reject infeasible combinations
   with a clear error message.

4. **For Phase A specifically**, throw_pos = catch_pos = a point directly above
   the hand at home position (Z = platform_height + hand_offset, X = Y = 0).
   The platform does not move.

**Implementation sketch** (key method on the controller):

```python
def _compute_cycle_timing(self, catch_end_time: float) -> tuple[float, float]:
    """From the end of the previous catch, compute next throw and catch times.

    Returns (throw_time, catch_time).
    """
    air_time = (1.0 - self._hold_ratio) * self._cycle_time
    hold_time = self._hold_ratio * self._cycle_time

    # How much of hold_time is consumed by catch post-arrival + throw pre-release?
    v = _GRAVITY * air_time / 2.0
    throw_traj = HandThrowTrajectory(v)
    throw_pre_release = -throw_traj.start_time   # positive duration

    # Throw starts at: catch_end + dwell + (throw pre-release is BEFORE throw_time)
    # So throw_time = catch_end + (hold_time - throw_pre_release)
    # But wait: catch_end already includes catch post-arrival.
    # The hold phase started at ball arrival, but catch_end is later.
    # We need: throw_time = ball_arrival + hold_time
    # Since air_time = throw_time_next - throw_time_prev... it's cleanest to
    # anchor on the ball events:
    #   ball_arrival = previous throw_time + air_time
    #   next throw_time = ball_arrival + hold_time
    #   next catch_time = next throw_time + air_time

    # Simplification: cycle_time = time between successive throws
    next_throw_time = self._last_throw_time + self._cycle_time
    next_catch_time = next_throw_time + air_time
    return next_throw_time, next_catch_time
```

**CLI:** `--toss-loop [cycle_time]` (or refactor `--juggle` to accept these params).
Optional `--hold-ratio 0.4`.

**Validates:**
- The hold_ratio timing schema and feasibility bounds
- Repeated throw-catch cycles (currently one-shot or manually parameterised)
- Ball spawn/capture/release looping
- Drop recovery (respawn and resume)
- Accurate cycle timing (measure actual vs. intended cycle_time)

**Does not require:** platform motion, MPC changes, new ballistics.

**Success criteria:** the robot tosses a ball straight up 10+ times in a row at
cycle_time = 1.2 s without a drop.  Actual cycle timing matches intended to within
±20 ms.

---

### Phase B: Toss between positions (platform moves)

**Goal:** throw the ball from position A to position B; the platform transits during
air time and catches at B; then throw from B to C; and so on.

Same timing model — the ball spends `air_time` in flight and `hold_time` in the
hand.  But now the platform must move between events.

**Cycle:**

```
1. Ball in hand at position A, platform oriented for throw toward B
2. Throw from A — ball arcs toward B (ThrowCatchPlanner computes orientation + velocity)
3. Platform transits A → B during air_time
     └─ MPC tracks catch pose as a DynamicTarget with arrival_time = catch_time
4. Catch at B — platform has arrived, hand catches ball
5. Dwell — ball in hand, pick next position C
6. Throw from B toward C, repeat
```

**Key considerations:**

- **Platform transit feasibility:** the platform has `air_time` to get from throw
  pose to catch pose.  For a 1.2 s cycle (air_time = 0.72 s) with 120 mm lateral
  spacing, the required peak velocity is ~377 mm/s — well within the MPC's 1000 mm/s
  throw-catch velocity limit.  The MPC's variable-resolution horizon with deadline
  tracking handles this naturally.

- **Platform tilt:** when throw_pos ≠ catch_pos, the ball arc is not vertical.
  `ThrowCatchPlanner` already computes the required platform tilt (align local Z
  with launch/catch velocity direction).  Tilts beyond 30° are rejected.

- **Position selection:** start with a scripted sequence (e.g. alternate between
  two positions ±60 mm from centre).  Graduate to random positions within a safe
  workspace envelope.

- **hold_ratio interaction:** hold_ratio indirectly controls transit difficulty.
  Lower hold_ratio → more air_time → easier transit.  Higher hold_ratio → less
  air_time → platform must move faster.

**New code:**
- Position sequence generator (scripted + random modes)
- Modify `_plan_next_cycle` to use a new catch position each cycle
- Workspace envelope check (max lateral distance given air_time and velocity limits)

**Success criteria:** 10+ consecutive catch-throw cycles alternating between two
positions 120 mm apart.

---

### Phase C: Velocity-at-events (continuous motion)

**Goal:** the platform is **moving** when it throws and catches.  Its velocity
contributes to the ball's launch/catch dynamics.

This is the core goal of continuous-motion juggling.  In Phases A and B, the platform
arrives and settles before the hand acts (100 ms settle margin).  In Phase C, the
platform passes through the event position with a designed velocity.

**Physics change — platform velocity enters the ballistic equation:**

```
v_ball = v_hand_relative + v_platform

v_hand_relative = hand throw speed along platform local Z
v_platform      = platform velocity at throw time (in world frame)
```

The `ThrowCatchPlanner` currently assumes the platform is stationary (v_platform = 0).
Phase C modifies it to accept an optional `platform_twist` at throw and catch time:

- **Throw:** the ball's world-frame velocity at release is the hand's ejection speed
  (along tilted platform Z) **plus** the platform's velocity.  If the platform is
  rising, the ball gets a free velocity boost — the hand can throw slower.

- **Catch:** the relative velocity between ball and hand is what matters.  If the
  platform descends as the ball arrives, the relative speed is reduced, softening
  the catch.

**MPC change — pass-through targets:**

The MPC's `DynamicTarget` already supports `arrival_twist` (velocity at arrival
time).  In Phases A-B this is None (arrive and stop).  In Phase C, it becomes
non-None: the MPC plans a trajectory that passes through the event pose at the
specified velocity, rather than decelerating to zero.

The `_SETTLE_MARGIN_S = 0.1` in the planner (which forces the platform to arrive
100 ms early and hold still) is removed or made conditional.

**New code:**
- `ThrowCatchPlanner.plan()` gains optional `throw_platform_twist` and
  `catch_platform_twist` parameters
- Ballistic computation accounts for platform velocity contribution
- MPC targets use `arrival_twist` to specify pass-through velocity
- Transit planner computes what platform velocity is achievable/desirable at events

**Success criteria:** continuous tossing with measurable platform velocity at event
times.  Ball trajectories match predictions (actual landing position within 10 mm
of predicted).

---

### Phase D: Orbit optimisation (if needed)

Only pursue this if Phase C reveals that MPC-planned transits produce excessive
per-leg jerk.  The MPC's built-in smoothness costs may be sufficient.

**If needed:** build a Fourier-parameterised periodic orbit between event positions,
optimised offline for minimum peak per-leg jerk.  The orbit is 2D in the XZ plane
(lateral oscillation + vertical oscillation), with the optimiser free to add small
contributions in other DoFs:

```
x(t) = A_x sin(ωt) + a₂ sin(2ωt) + ...
z(t) = A_z cos(ωt) + a₂ cos(2ωt) + ...     (cosine: Z peaks at throw, dips at catch)
```

The orbit provides a time-varying reference trajectory.  The MPC tracks it with
time-varying weights: high near events (position and velocity must be accurate),
low during transit (deviations are tolerable).

This is the most complex component in the plan and may turn out to be unnecessary.

---

## How to Run (once implemented)

```bash
# Phase A: vertical toss, default 1.2s cycle, 0.4 hold ratio
python sim/main.py --toss-loop 1.2

# Phase A: adjust hold ratio
python sim/main.py --toss-loop 1.2 --hold-ratio 0.4

# Phase A: faster cycle (tighter timing)
python sim/main.py --toss-loop 1.0

# Phase B: toss between positions (positions selected automatically)
python sim/main.py --toss-loop 1.2 --lateral-spacing 120

# Existing mode (preserved for compatibility)
python sim/main.py --juggle
```

---

## Open Questions

1. **Refactor vs. new controller** — Should Phase A refactor
   `ContinuousThrowCatchController` in place, or create a new `TossLoopController`
   alongside it?  The existing controller has useful infrastructure (state machine,
   viewer integration, drop recovery) but its parameterisation (flight_time, XY
   positions) is different from the timing model.  Refactoring is cleaner but risks
   breaking the existing `--juggle` mode during development.

2. **Position selection strategy (Phase B)** — How should catch positions be chosen?
   Options: fixed alternating (left-right-left), random within a safe envelope, or
   a smooth parametric path.  The choice affects how predictable the motion looks
   and how much the platform must accelerate.

3. **Platform velocity contribution (Phase C)** — How much of the launch velocity
   should come from the platform vs. the hand?  A 50/50 split reduces hand wear but
   requires faster platform motion.  Need to find the sweet spot based on actual
   MPC tracking performance.

4. **hold_ratio tuning** — 0.4 is a starting point.  Lower values (more air time)
   give the platform more transit time but require faster throws and taller arcs.
   Higher values (more hold time) give more dwell but less transit time.  The optimal
   value may depend on the workspace distance between events.

5. **Drop recovery with motion (Phase B+)** — For Phase A (stationary), just respawn.
   For Phase B+ (moving platform), should the platform stop and reset to home, or
   continue its motion and respawn the ball on the next dwell phase?
