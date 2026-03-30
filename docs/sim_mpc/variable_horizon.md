# Variable-Resolution Horizon

This page explains the variable-timestep horizon that lets the MPC handle both immediate tracking and timed targets (ball catches).

**Source files:**

- `sim/controller/params.py` — `dt_schedule`
- `sim/controller/mpc.py` — `_build_reference()`

## The Problem

Ball catching requires the MPC to plan over two timescales simultaneously:

- **Near-term (0-100 ms):** Fine position control — the platform is already near the target and needs precise tracking with smooth commands.
- **Far-term (100 ms - 1.5 s):** Gross motion planning — the platform needs to traverse a large distance to reach the catch pose. Fine temporal resolution here is wasted compute.

A uniform 20 ms timestep would need 67 nodes for a 1.35 s horizon — too many variables for IPOPT to solve within the 20 ms budget. A uniform 250 ms timestep would lose the fine control needed for accurate catching.

## The Solution: Two-Tier Schedule

The default `dt_schedule` has 10 steps split into two tiers:

```
Fine tier:   5 × 20 ms  = 100 ms    (steps 0-4)
Coarse tier: 5 × 250 ms = 1250 ms   (steps 5-9)
```

Total: 10 steps, 1.35 s horizon, 180 decision variables.

The fine tier provides precise control in the immediate future. The coarse tier extends the planning horizon to see approaching deadlines without increasing the NLP size.

### Cumulative Times

The horizon nodes occur at these absolute times from the current step:

| Node | Time (ms) | Tier |
|---|---|---|
| 0 | 0 | (current state — parameter) |
| 1 | 20 | Fine |
| 2 | 40 | Fine |
| 3 | 60 | Fine |
| 4 | 80 | Fine |
| 5 | 100 | Fine |
| 6 | 350 | Coarse |
| 7 | 600 | Coarse |
| 8 | 850 | Coarse |
| 9 | 1100 | Coarse |
| 10 | 1350 | Coarse |

### Tier Boundary Treatment

The fine-to-coarse transition (step 4→5) creates an asymmetry: the command delta `u[4]→u[5]` straddles two different timescales. The cost function handles this with:

- **Smoothness and acceleration costs:** Use the geometric mean `√(dt_fine × dt_coarse) = √(0.02 × 0.25) ≈ 0.071 s` as the effective interval. This splits the penalty evenly in log-space between the tiers.
- **Rate limits:** Use the physical interval `dt_schedule[k-1]` (the previous step's duration). See [MPC_BUGS.md](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/sim/MPC_BUGS.md) B-01 for a discussion of this choice.

## Actuator Dynamics Across Tiers

The actuator lag model uses the exact exponential: `α = 1 - exp(-dt/τ)`.

| Tier | dt | α (τ=30ms) | Meaning |
|---|---|---|---|
| Fine | 20 ms | 0.487 | Actuator reaches ~49% of the command-actual gap each step |
| Coarse | 250 ms | 0.9998 | Actuator essentially converges fully in one step |

The exact form is critical here. Forward Euler (`α = dt/τ`) gives `α = 8.33` for the coarse tier — wildly unstable. The exponential form is unconditionally stable for any positive dt.

## Reference Construction

The `_build_reference()` method constructs two `(N+1, 6)` arrays from the target:

### Pose Reference

All N+1 nodes are set to the target pose. The MPC finds the optimal path via its cost function and constraints — no intermediate waypoints are imposed. This "constant reference" approach works because:

- The MPC's own dynamics prevent instantaneous jumps (actuator lag constraint).
- The smoothness cost penalizes jerky transitions.
- The terminal cost (`Qf`) provides deadline pull toward the target at the horizon end.

### Twist Reference

- **ASAP targets** (`arrival_time = None`): All twist references are zero. The MPC drives to the target as fast as constraints allow.
- **Timed targets with nonzero arrival twist:** Twist reference is zero for nodes before the deadline and `target_twist` at/after the deadline. This lets the MPC plan its own velocity profile during approach, then match the desired velocity at arrival (used for throw targets).

## Tracking Cost Weighting

All tracking costs use **uniform weight** across horizon nodes — there is no per-node urgency multiplier. An earlier version of the MPC used an urgency ramp that scaled tracking weights from a low base to a high peak near the deadline, but this was removed because it caused premature arrival, target-switch jerk, and complicated interactions with the terminal cost.

The current design relies on two mechanisms to handle timed targets:

- **Event-based callers** (catch sequences, toss loop): the reference trajectory itself encodes timing. The MPC simply tracks the moving reference with uniform cost.
- **Flat-reference callers** (static targets, waypoint sequences): the terminal cost (`Qf_pos`, `Qf_ori`) provides deadline pull toward the target at the horizon end, while the stage costs (`Q_pos`, `Q_ori`) apply uniform pressure at all nodes.

## Feasibility Checking

The `FeasibilityChecker` (in `sim/hand/feasibility.py`) uses a separate coarse-horizon MPC to predict whether the controller can reach a target in time. It runs a one-shot solve with:

- Uniform 0.1 s timesteps × 10 nodes = 1.0 s horizon
- ASAP mode (uniform tracking weight)
- Higher velocity limit (1000 mm/s)
- No warm-start

### Two-Stage Check

1. **IK pre-filter:** Reject targets whose leg extensions fall outside `[0, stroke_mm]` with 2 mm tolerance. This catches geometrically impossible poses (costs ~0.1 ms).

2. **Coarse MPC solve:** Solve with constant target reference. Find the horizon node closest to the deadline. If the predicted pose at that node is within tolerance (5 mm position, 5.7° orientation), the target is feasible (costs ~5 ms). Orientation error uses the **geodesic angle** `arccos((tr(R₁ᵀ R₂) - 1) / 2)`, not a rotation vector norm — see [Hand & Ball Physics — Geodesic Angle](hand_and_ball.md#orientation-error-geodesic-angle).

### Limitations

The feasibility checker uses a coarse uniform-timestep horizon (10 x 0.1 s) while the real controller uses the two-tier variable-resolution schedule. This structural difference means the checker is slightly optimistic — see [MPC_BUGS.md](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/sim/MPC_BUGS.md) B-03 for details.

## Toss Loop Bypass

The continuous toss loop (`--cycle-time`, `TossLoopController`) computes a time-varying platform reference via quintic Hermite interpolation and sends it as an ASAP target each step, rather than using static endpoint targets with arrival deadlines.

### Quintic Hermite Solution

The toss loop replaces the static endpoint target with a **continuous motion stream**: a quintic Hermite spline evaluated at the current simulation time. The spline matches position, velocity, and acceleration (all zero) at segment boundaries, giving C2 continuity.

- All targets are ASAP (`arrival_time = None`), with uniform tracking weight across all nodes.
- The MPC simply tracks the moving reference with its standard cost function.
- The platform arrives at each event pose on time (because the reference itself is correctly timed) and at the correct velocity (because the reference encodes the velocity).
- No discontinuities at target transitions — the reference is smooth everywhere.

See [Usage — Toss Loop](usage.md#toss-loop-continuous-motion-juggling) for CLI usage and [Control Loop — Toss Loop Adapter](control_loop.md#toss-loop-adapter) for the data flow.
