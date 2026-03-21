# Variable-Resolution Horizon

This page explains the variable-timestep horizon and the urgency system that lets the MPC handle both immediate tracking and timed targets (ball catches).

**Source files:**

- `sim/controller/params.py` — `dt_schedule`, urgency parameters
- `sim/controller/mpc.py` — `_build_reference()`, `_compute_urgency()`

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
- The urgency system controls *when* the tracking cost kicks in.

### Twist Reference

- **ASAP targets** (`arrival_time = None`): All twist references are zero. The MPC drives to the target as fast as constraints allow.
- **Timed targets with nonzero arrival twist:** Twist reference is zero for nodes before the deadline and `target_twist` at/after the deadline. This lets the MPC plan its own velocity profile during approach, then match the desired velocity at arrival (used for throw targets).

## Urgency System

The urgency system controls how strongly the MPC tracks the reference at each horizon node. This is the mechanism that converts "arrive at time T" into optimizer behavior.

### ASAP Mode (arrival_time = None)

All nodes get `urgency = 1.0`. The MPC pushes toward the target uniformly — there's no timing constraint, so full tracking pressure everywhere is appropriate.

### Timed Mode

Each node's urgency ramps from `urgency_base` (0.05) to `urgency_max` (10.0) as the node's absolute time approaches the deadline:

```python
time_to_deadline = max(arrival_time - t_node, 0)
ramp = max(0, 1 - time_to_deadline / urgency_ramp_s)   # 0 far, 1 at deadline
urgency = urgency_base + (urgency_max - urgency_base) * ramp
```

The ramp window is `urgency_ramp_s = 0.5 s`.

### Why Low Base Urgency?

With a constant reference (all nodes = target), high uniform urgency would penalize early nodes for being far from the target even though they physically can't be there yet. The low base (0.05) lets the MPC choose its own trajectory for "how to get there", while the ramp and terminal cost ensure accurate arrival.

### Urgency Timeline Example

Target: arrive at t = 1.0 s from now. Ramp window: 0.5 s.

| Node | Time from now | Time to deadline | Ramp | Urgency |
|---|---|---|---|---|
| 1 | 0.02 s | 0.98 s | 0.0 | 0.05 |
| 2 | 0.04 s | 0.96 s | 0.0 | 0.05 |
| ... | ... | ... | ... | ... |
| 5 | 0.10 s | 0.90 s | 0.0 | 0.05 |
| 6 | 0.35 s | 0.65 s | 0.0 | 0.05 |
| 7 | 0.60 s | 0.40 s | 0.2 | 2.04 |
| 8 | 0.85 s | 0.15 s | 0.7 | 7.00 |
| 9 | 1.10 s | 0.0 s | 1.0 | 10.0 |
| 10 | 1.35 s | 0.0 s | 1.0 | 10.0 |

Nodes 9 and 10 (past the deadline) see full urgency, creating strong pull toward the target. Nodes 1-6 see minimal urgency, allowing the MPC to plan an efficient approach path.

## Feasibility Checking

The `FeasibilityChecker` (in `sim/hand/feasibility.py`) uses a separate coarse-horizon MPC to predict whether the controller can reach a target in time. It runs a one-shot solve with:

- Uniform 0.1 s timesteps × 10 nodes = 1.0 s horizon
- ASAP mode (no urgency ramp)
- Higher velocity limit (1000 mm/s)
- No warm-start

### Two-Stage Check

1. **IK pre-filter:** Reject targets whose leg extensions fall outside `[0, stroke_mm]` with 2 mm tolerance. This catches geometrically impossible poses (costs ~0.1 ms).

2. **Coarse MPC solve:** Solve with constant target reference. Find the horizon node closest to the deadline. If the predicted pose at that node is within tolerance (5 mm position, 5.7° orientation), the target is feasible (costs ~5 ms). Orientation error uses the **geodesic angle** `arccos((tr(R₁ᵀ R₂) - 1) / 2)`, not a rotation vector norm — see [Hand & Ball Physics — Geodesic Angle](hand_and_ball.md#orientation-error-geodesic-angle).

### Limitations

The feasibility checker uses ASAP mode while the real controller uses the urgency ramp. This makes the checker slightly optimistic — see [MPC_BUGS.md](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/sim/MPC_BUGS.md) B-03 for details.
