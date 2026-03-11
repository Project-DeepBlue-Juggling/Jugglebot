# Workspace Safety

This page covers how the motion planner keeps the platform within safe operating limits — leg extension bounds, singularity avoidance, and fault detection.

**Source file:** [workspace.py](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/ros_ws/src/jugglebot/jugglebot/motion/workspace.py) (~300 lines)

## Overview

The workspace safety system operates at two levels:

1. **Planning time** — The [feasibility checker](trajectory.md#feasibility-checking) rejects trajectories that would violate constraints before they execute.
2. **Runtime** — The [control loop](control_loop.md) checks workspace limits every cycle (500 Hz) and can slow down or abort if the platform approaches danger.

Both levels use the same margin constants, so a trajectory that passes feasibility will not trigger runtime limits under normal conditions.

## Leg Extension Limits

The legs have a physical stroke of 280 mm (0 = fully retracted, 280 = fully extended). Safety margins keep the platform away from the mechanical endpoints:

```
0 mm                                                          280 mm
|----|----|------------------------------------------|----|-----|
     ^    ^                                          ^    ^
  HARD  SOFT                                      SOFT  HARD
  5mm   15mm                                      15mm  5mm
```

| Boundary | Distance from Endpoint | Behaviour |
|---|---|---|
| **Hard limit** | 5 mm | Trajectory aborts, E-stop triggered |
| **Soft limit** | 15 mm | Speed ramp-down begins |
| **OK zone** | 15–265 mm | Normal operation |

### Soft Limit Ramp-Down

When a leg enters the soft zone (between 5 mm and 15 mm from an endpoint), the speed scale factor ramps linearly from 1.0 at the soft boundary to 0.0 at the hard boundary:

$$\text{scale} = \frac{\text{distance to hard boundary}}{\text{soft margin} - \text{hard margin}} = \frac{\text{distance to hard}}{10 \text{ mm}}$$

The minimum scale across all 6 legs is used. This gradual slowdown gives the platform a chance to decelerate before hitting the hard limit.

## Condition Number Limits

The Jacobian condition number indicates proximity to singular configurations (poses where the platform loses a degree of freedom). High condition numbers mean small Cartesian motions require large or uneven leg motions.

| Threshold | Value | Behaviour |
|---|---|---|
| Home condition | ~450 | Normal baseline (mixed mm/rad units) |
| Soft limit | 1.5× home (~675) | Speed ramp-down begins |
| Hard limit | 2.0× home (~900) | Trajectory aborts, E-stop triggered |

The soft limit uses the same linear ramp-down as leg extensions, applied between the soft and hard thresholds.

### Why Relative Thresholds?

The raw condition number is ~450 at home due to the Jacobian's mixed units (mm/s and rad/s). This number is not inherently meaningful — it just reflects the unit convention. Using relative thresholds (multiples of the home value) makes the limits independent of unit choices and directly meaningful: "2× worse than home" is a clear, physical threshold.

## WorkspaceLimits Configuration

```python
from jugglebot.motion.workspace import WorkspaceLimits
from jugglebot.motion.geometry import StewartGeometry

geom = StewartGeometry()
limits = WorkspaceLimits.from_geometry(geom)
```

The factory method precomputes the home condition number and derives all thresholds:

| Field | Value | Derivation |
|---|---|---|
| `leg_stroke_mm` | 280 | From geometry |
| `leg_soft_min_mm` | 15 | `LEG_SOFT_MARGIN_MM` |
| `leg_soft_max_mm` | 265 | `stroke - LEG_SOFT_MARGIN_MM` |
| `leg_hard_min_mm` | 5 | `LEG_HARD_MARGIN_MM` |
| `leg_hard_max_mm` | 275 | `stroke - LEG_HARD_MARGIN_MM` |
| `cond_home` | ~450 | Computed at home pose |
| `cond_soft` | ~675 | `COND_SOFT_FACTOR × cond_home` |
| `cond_hard` | ~900 | `COND_HARD_FACTOR × cond_home` |

## Runtime Checking

**Function:** `check_workspace_limits(extensions_mm, cond_number, limits)` → `WorkspaceCheck`

Called every control cycle by the control loop. Returns:

```python
@dataclass
class WorkspaceCheck:
    status: WorkspaceStatus      # OK, SOFT_LIMIT, or HARD_LIMIT
    leg_extensions_mm: np.ndarray
    cond_number: float
    speed_scale: float           # 1.0 if OK, <1.0 if soft, 0.0 if hard
    violations: list             # human-readable descriptions
```

### What Happens on Violation

| Status | Control Loop Action |
|---|---|
| `OK` | Normal operation |
| `SOFT_LIMIT` | Log warning with speed_scale. (Future: adaptive speed reduction) |
| `HARD_LIMIT` | Cancel active trajectory, E-stop, set fault state |

Currently, the soft limit triggers a log warning but does not actively slow the trajectory. The feasibility checker prevents trajectories from entering soft zones under normal conditions, so the runtime soft limit is a secondary safety net for unexpected situations (disturbances, modelling errors).

## Additional Workspace Functions

### Check Leg Extensions

**Function:** `check_leg_extensions(extensions_mm, geom)` → (all_valid, states)

Quick check whether all legs are within the hard limits:

- `all_valid`: `True` if every leg is within bounds
- `states`: (6,) int8 array — `0` = OK, `-1` = underextended, `+1` = overextended

Uses the same 5 mm hard margins as `check_workspace_limits()`.

### Check Reachability

**Function:** `check_reachability(pos, rot, geom)` → bool

Given a platform pose, compute leg extensions via IK and check if all legs are within stroke. Returns `True` if the pose is reachable.

### Compute Condition Number

**Function:** `compute_condition_number(pos, rot, geom)` → float

Compute the Jacobian condition number at a given pose. Calls `compute_jacobian()` and returns `np.linalg.cond(J)`.

### Map Singularities

**Function:** `map_singularities(geom, z_range, xy_range, tilt_max_deg, resolution)` → dict

Sweep the workspace on a grid and compute reachability and condition number at each point. Returns a dictionary with:

- `poses` — list of (pos, rotvec) tuples
- `extensions` — per-pose leg extensions
- `condition_numbers` — per-pose condition numbers (inf if unreachable)
- `reachable` — boolean flags
- `summary` — statistics (total, reachable, unreachable, ill-conditioned, cond min/max/mean/median)

Used during Phase 1 to characterize the workspace. Results: 929/1944 poses reachable, condition range 449–644.

## Motor Command Safety

In addition to workspace limits, the control loop enforces a set of runtime safety checks on every motor command — slew rate limiting against actual motor feedback, overspeed detection, tracking error faults, and feedback staleness gating. These are documented separately in [Motor Command Safety](safety.md).

## Agreement Between Planning and Runtime

A critical design invariant: the feasibility checker and the runtime workspace monitor use **the same margins**. Both use `LEG_HARD_MARGIN_MM = 5` mm as the hard boundary. This means:

- A trajectory that passes `check_feasibility()` will not trigger a hard limit during runtime (barring disturbances).
- The runtime monitor is a safety net, not a primary control mechanism.

If these margins ever diverge, trajectories could pass feasibility but trigger runtime aborts, or vice versa.

## Workspace Characteristics

From the Phase 1 singularity map (8×8×8×4×4 grid = 8192 poses):

| Metric | Value |
|---|---|
| Total poses sampled | 1944 |
| Reachable | 929 (47.8%) |
| Unreachable | 1015 (52.2%) |
| Condition range (reachable) | 449 – 644 |
| Home condition | ~450 |

The workspace is roughly a truncated cone — wider near home height, narrowing at extreme Z positions and tilts.

## Verification

| Test | Result |
|---|---|
| H1: Workspace boundary (25% and 100% speed) | PASS — all 6 trajectories, Z-down correctly triggers soft limit |
| H3: Fault injection (static) | PASS — all legs idle safely on triggered fault |
| H5: 30-min moderate endurance (50%) | PASS |
| H6: 10-min aggressive (75% + 100%) | PASS — worst tracking 2.025 mm (threshold 3.0 mm) |

See [Validation Results](results.md) for details.
