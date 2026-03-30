# MPC Velocity Tracking Problem Scope

## Problem Statement

The MPC cannot achieve nonzero target velocities at arrival.  When
`target_twist` is provided (e.g. DT6: arrive at z=60mm with vz=-200mm/s at
t=1.0s), the platform reaches the target *position* accurately but arrives
with near-zero velocity (~0 mm/s instead of -200 mm/s).

This blocks the **throw** phase of continuous juggling, where the platform
must release the ball at a precise downward velocity.

---

## Observed Behaviour

### DT6 (standalone throw)

Target: `pose=[0,0,60]`, `arrival_twist=[0,0,-200,0,0,0]`, `arrival_time=1.0s`

```
t=0.80  vz=  0.0 mm/s  z=60.0mm   (already settled at target)
t=0.90  vz= -0.4 mm/s  z=60.0mm
t=1.00  vz=  0.1 mm/s  z=60.0mm   <-- should be -200 mm/s
t=1.10  vz= -0.0 mm/s  z=60.0mm
```

The platform reaches z=60mm by ~t=0.4s and holds there.  It never begins
the -200 mm/s downward motion because position tracking overwhelms velocity
tracking in the cost function.

### DT7 (catch then throw)

Same issue in the throw phase.  Peak downward velocity during the throw
window is ~-23 mm/s (target: -150 mm/s).

---

## Root Cause Analysis

Five interacting factors prevent velocity tracking.  Understanding all five
is essential — fixing any single factor in isolation is insufficient.

### 1. Position reference is a point attractor

`_build_reference()` sets `ref_traj[:] = target_pose` at **all** horizon
nodes.  Every node's position cost pulls toward z=60mm.  To achieve
vz=-200mm/s at the deadline, the platform must be *moving through* z=60,
which requires that it approach from above (~z=160mm at t=0.5s).  But the
position cost at every earlier node pulls it back down to z=60, so the
platform converges early and holds.

The position reference is designed for **convergent targets** (arrive and
hold) — it has no concept of a trajectory that *passes through* a point.

### 2. Twist reference is zero before the deadline

```python
# _build_reference(), line 617-621
for k in range(N + 1):
    if t_nodes[k] >= time_budget:
        twist_traj[k] = tw
```

For DT6 with time_budget=1.0s and cumulative_times=[0, 0.02, ..., 0.85,
1.10, 1.35], only **nodes 9 and 10** (t=1.10s, 1.35s) get `twist_ref =
[-200]`.  Nodes 1-8 have `twist_ref = [0]`, which means the velocity cost
`Q_vel_lin * (velocity - 0)^2` actively **penalises** any platform
motion during the approach phase.

### 3. Cost weight imbalance: 10,000x ratio

| Cost term | Weight | 1mm pos error | 200mm/s vel error |
|-----------|--------|---------------|-------------------|
| Position  | Q_pos=10.0 | 10 * 1 = **10** | -- |
| Velocity  | Q_vel_lin=0.001 | -- | 0.001 * 40000 = **40** |
| Terminal pos | Qf_pos=50.0 | 50 * 1 = **50** | -- |
| Terminal vel | *(none)* | -- | 0.001 * 40000 = **40** |

At first glance velocity cost (40) exceeds position cost (10) for a full
200mm/s error vs 1mm position error.  But the position cost grows
quadratically too — a 5mm error costs 250, while the velocity cost for
even a 50mm/s improvement (150mm/s error) only saves 40-22.5=17.5.  The
solver finds that it's cheaper to hold position perfectly and accept the
full velocity error, because the position error from accelerating creates
more cost than the velocity improvement saves.

The absence of a **terminal velocity cost** means the MPC has no extra
incentive at the final node to match velocity — it uses the same weak
Q_vel_lin=0.001 weight everywhere.

### 4. Variable-resolution horizon creates a gap at the deadline

The default dt_schedule creates cumulative times:
```
[0, 0.02, 0.04, 0.06, 0.08, 0.10, 0.35, 0.60, 0.85, 1.10, 1.35]
 fine tier (5x20ms)              coarse tier (5x250ms)
```

With a 1.0s deadline, the deadline falls **between node 8 (t=0.85) and
node 9 (t=1.10)** — a 250ms gap with no intermediate node.  The MPC has
no decision variable AT the deadline.  It optimises for node 8 (0.15s
before) and node 9 (0.10s after), both of which see conflicting objectives.

The finite-difference velocity at node 9 is `(p[9] - p[8]) / 0.25`.  To
achieve -200mm/s, `p[9]` must be 50mm below `p[8]`.  But the position cost
pulls both toward z=60.  The solver compromises by putting both near z=60,
yielding approximately zero velocity.

### 5. Receding horizon replans from actual state every tick

At each 50Hz control step, the MPC replans from the *current measured
state*.  By t=0.4s, the platform has already converged to z=60mm with zero
velocity.  From that state, the MPC sees a 1.35s horizon.  The deadline is
at time_budget=0.6s — well within the horizon.  But the solver's plan at
every replanning step is: "hold at z=60 (zero position error, low cost)"
rather than "move away from z=60 to build velocity (creates position error,
high cost)."  The receding-horizon structure reinforces the hold decision at
every tick.

---

## Why Simple Fixes Don't Work

### Approach A: Ramp twist reference before deadline

Replace the step function with a linear ramp over 0.5s before the deadline:
```python
elif time_to_deadline < ramp_s:
    frac = 1.0 - time_to_deadline / ramp_s
    twist_traj[k] = frac * tw
```

**Result:** No improvement.  The velocity cost at ramped nodes (e.g. node 8
with twist_ref=-140mm/s) IS significant (~20 for 140mm/s error)
but the position cost at the same node for any deviation from z=60 is even
larger.  The solver still holds at z=60 rather than build velocity.

### Approach B: Add terminal velocity cost

Add `Qf_vel_lin=0.1` (100x the stage weight) to the terminal node cost.

**Result:** No improvement.  The terminal node is at t=1.35s (node 10),
not at the deadline (t=1.0s).  The MPC can satisfy the terminal velocity
cost by planning motion between nodes 9-10, while still holding at z=60
during nodes 1-8.  The deadline passes with zero velocity.

### Approach C: Offset position reference along twist direction

Set `ref_traj[k] = target_pose - tw * time_to_deadline` for nodes before
the deadline and `ref_traj[k] = target_pose + tw * time_past_deadline` for
nodes after.

**Result:** Creates the correct approach trajectory — platform starts at
z~145mm and descends at ~55mm/s.  But the position reference converges to
z=60 at the deadline, and the position cost still dominates.  The platform
brakes from -55mm/s to 0mm/s in the 150ms before the deadline and settles
at z=60.  The velocity at t=1.0s remains ~0.

### Combined A+B+C

All three changes together produce the same outcome: a descent trajectory
that brakes to zero at the deadline.  The fundamental issue is that
**position and velocity targets at the same spatiotemporal point are
physically contradictory** for a system that cannot instantaneously change
velocity.

---

## What a Real Fix Requires

The core architecture assumes convergent targets (arrive and hold).  A
throw mode needs one or more of these structural changes:

### Option 1: Trajectory-based reference (not point-based)

Instead of `ref_traj[:] = target_pose`, compute a reference trajectory
that's kinematically consistent with arriving at `target_pose` with
`target_twist` at the deadline.  For nodes before the deadline:

```
ref_pos(t) = target_pose + target_twist * (t - deadline)
```

This is a straight-line extrapolation backward from the arrival state.
The position cost then *helps* velocity tracking instead of fighting it,
because hitting the reference at each node requires the correct velocity.

**Risk:** The extrapolated reference at early nodes may be outside the
workspace (e.g. z=60 + 200*0.5 = z=160mm at t=0.5s).  Need workspace
clamping or soft activation.

### Option 2: Reduce position cost near deadline for throw targets

When `target_twist != 0`, reduce or zero the position cost weight at nodes
near the deadline.  Let velocity tracking dominate close to arrival.

**Risk:** Position accuracy degrades for throw targets.  The ball release
point may shift.  Need to quantify acceptable position error at release.

### Option 3: Add velocity as a hard constraint

Instead of a soft cost, add an equality or inequality constraint on the
finite-difference velocity at the deadline node:

```
(p[k_deadline] - p[k_deadline-1]) / dt = target_twist  (equality)
```

**Risk:** Finding `k_deadline` exactly is impossible with the variable-
resolution schedule (the deadline may fall between nodes).  Interpolation
or a dedicated "deadline node" would be needed.

### Option 4: Dual-phase horizon

Split the horizon into two regions:
1. **Before deadline:** position reference follows the approach trajectory
   (Option 1); velocity cost uses ramped twist reference
2. **After deadline:** position cost is turned off or redirected to a
   deceleration trajectory; velocity cost continues tracking

This is the most architecturally clean solution but requires the most code.

---

## Relevant Code Locations

| File | Function | Lines | Role |
|------|----------|-------|------|
| `controller/mpc.py` | `_build_problem()` | 213-244 | Cost function (position, velocity, terminal) |
| `controller/mpc.py` | `_build_reference()` | 586-623 | Reference trajectory construction |
| `controller/mpc.py` | `_build_reference()` | 586-623 | Reference trajectory + twist construction |
| `controller/params.py` | `MPCParams` | 68-79 | Q_pos, Q_vel_lin, Qf_pos weights |
| `sim/input/scripted.py` | `make_DT6()`, `make_DT7()` | 602-640 | Test scenarios |
| `sim/tests/test_mpc_dynamic.py` | `TestDT6ThrowVelocity` | — | Velocity tracking tests |

## Related Bugs

- **B-02** (MPC_BUGS.md): Lead time utilization — the MPC converges to the
  target position early and holds, rather than using lead time to build velocity
- **B-09** (MPC_BUGS.md): Terminal cost has no velocity component

## Test Coverage

The following tests document the current (broken) behaviour with weak
assertions that will pass even without the fix.  Tighten these when the
fix lands:

- `TestDT6ThrowVelocity::test_velocity_direction` — asserts `vz < 10`
  (should be `vz < -100` when fixed)
- `TestDT7CatchThenThrow::test_throw_phase_activates` — asserts throw
  phase activates, does not check velocity magnitude
