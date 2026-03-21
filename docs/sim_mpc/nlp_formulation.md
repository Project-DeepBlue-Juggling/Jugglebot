# NLP Formulation

This page describes the nonlinear program (NLP) that the MPC solves every control step. The NLP is built once at controller construction and reused with updated parameters each solve.

**Source files:**

- `sim/controller/mpc.py` — NLP construction (`_build_problem`) and solve logic
- `sim/controller/params.py` — all weight and constraint parameters

## Decision Variables

The NLP has three sets of decision variables per horizon step, for a total of `18 * N` scalar variables (N = 10 by default → 180 variables):

| Variable | Count | Range | Meaning |
|---|---|---|---|
| `u[0..N-1]` | 6 per step | `[0, stroke_mm]` | Commanded leg extensions (mm, home-relative) |
| `q[1..N]` | 6 per step | `[0, stroke_mm]` | Actual leg extensions after actuator lag |
| `p[1..N]` | 6 per step | workspace bounds | Platform pose `[x, y, z, rx, ry, rz]` (mm, rad) |

Note the indexing: `q[0]` and `p[0]` are **parameters** (current state), not decision variables. The solver decides what to command (`u`), predicts what the actuators will actually do (`q`), and finds the corresponding platform pose (`p`).

### Variable Layout in Memory

Variables are packed into a single flat vector `W` of length `18N`:

```
W = [ u[0](6) | u[1](6) | ... | u[N-1](6) |     ← commands
      q[1](6) | q[2](6) | ... | q[N](6)   |     ← actual extensions
      p[1](6) | p[2](6) | ... | p[N](6)   ]     ← platform poses
```

Index arithmetic: `u[k]` starts at offset `6k`, `q[k]` at `6N + 6(k-1)`, `p[k]` at `12N + 6(k-1)`.

### Variable Bounds

| Variable | Lower | Upper | Rationale |
|---|---|---|---|
| `u[k]` | 0 | `stroke_mm` (280) | Full physical stroke range |
| `q[k]` | 0 | `stroke_mm` (280) | Actuator can't be outside stroke |
| `p[k] x, y` | -200 mm | +200 mm | Generous workspace envelope |
| `p[k] z` | -50 mm | +300 mm | Below home is limited; stroke caps ~275 |
| `p[k] rx, ry, rz` | -0.3 rad | +0.3 rad | ~17 degrees — beyond this, Rodrigues linearization degrades |

## Parameters (Updated Each Solve)

The NLP has a fixed parameter vector `P` that changes every solve call:

| Parameter | Size | Source |
|---|---|---|
| `p_init` | 6 | Current platform pose from sensors |
| `q_init` | 6 | Current actual leg extensions from sensors |
| `u_prev` | 6 | Previous step's commanded extensions |
| `u_prev_prev` | 6 | Two steps ago (for acceleration smoothness) |
| `p_ref[0..N]` | 6 × (N+1) | Reference pose at each horizon node |
| `twist_ref[0..N]` | 6 × (N+1) | Reference twist at each horizon node |
| `urgency[1..N]` | N | Per-node tracking weight multiplier |

Total: `24 + 12(N+1) + N` scalars (with N=10: 156 parameters).

## Cost Function

The cost is a weighted sum of tracking, velocity, effort, smoothness, and acceleration terms. All terms are summed over the horizon nodes.

### Tracking Cost (nodes k = 1..N)

```
J_track = Σ_{k=1}^{N-1} urgency[k] * ( Q_pos * ||err_pos[k]||² + Q_ori * ||err_ori[k]||² )
        + urgency[N] * ( Qf_pos * ||err_pos[N]||² + Qf_ori * ||err_ori[N]||² )
```

Where `err[k] = p[k] - p_ref[k]`. The terminal node (k = N) uses heavier weights (`Qf_pos = 50` vs `Q_pos = 10`) to pull the solution toward the target at the horizon end.

The `urgency` multiplier modulates tracking weight per node — see [Variable Horizon](variable_horizon.md) for details.

### Velocity Tracking Cost (nodes k = 1..N)

```
J_vel = Σ_{k=1}^{N} urgency[k] * ( Q_vel_lin * ||dp_lin/dt - twist_ref_lin[k]||²
                                   + Q_vel_ang * ||dp_ang/dt - twist_ref_ang[k]||² )
```

Velocity is estimated by finite difference: `dp/dt = (p[k] - p[k-1]) / dt_k`. This penalizes deviation from the desired twist at arrival, used for throw trajectories where the platform needs nonzero velocity at the target.

### Control Effort (nodes k = 0..N-1)

```
J_effort = Σ R * ||u[k] - home_ext||²
```

Penalizes deviation of the command from home extensions. This is a small regularizer (`R = 1e-4`) that prevents the solver from commanding extreme positions without strong tracking incentive.

### Smoothness Cost (nodes k = 0..N-1)

```
J_smooth = Σ (S / dt_smooth) * ||du[k]||²
```

Where `du[k] = u[k] - u[k-1]` (with `u[-1] = u_prev` from the previous solve). The `1/dt` normalization makes fine and coarse steps comparable — a 6 mm/s command rate costs the same whether it's spread over 20 ms or 250 ms.

At tier boundaries (fine-to-coarse transition), `dt_smooth` uses the geometric mean `√(dt_fine × dt_coarse)` to split the penalty evenly between the two tiers.

### Acceleration Smoothness (nodes k = 0..N-1)

```
J_accel = Σ (A / dt_smooth) * ||ddu[k]||²
```

Where `ddu[k] = du[k] - du[k-1]` is the second difference of commands. This penalizes jerky changes in command rate. The `1/dt` normalization (rather than the physically correct `1/dt³`) prevents coarse-tier jerk from being almost free while keeping fine-tier oscillation expensive.

## Constraints

### 1. Actuator Dynamics (6N equality constraints)

```
q[k+1] = q[k] + (u[k] - q[k]) × α_k
```

Where `α_k = 1 - exp(-dt_k / τ)` is the exact first-order lag response for timestep `dt_k` with time constant `τ` (default 30 ms).

This models the actuator as a first-order system tracking the commanded position. The exact exponential form (rather than forward Euler `α = dt/τ`) is critical for the coarse tier, where `dt/τ = 0.25/0.03 ≈ 8.33` would make forward Euler violently unstable.

At the default schedule: `α_fine = 0.487` (20 ms step), `α_coarse = 0.9998` (250 ms step — nearly full convergence).

### 2. IK Consistency (6N equality constraints)

```
ik_ext_i(p[k]) = q[k]_i + home_ext_i     for i = 0..5
```

Where `ik_ext_i(p)` is the CasADi symbolic inverse kinematics for leg `i`:

```
ik_ext_i(p) = ||pos + height + R(rv) × plat_node_i - base_node_i|| - init_len_i
```

This ensures every predicted platform pose `p[k]` is kinematically consistent with the predicted leg extensions `q[k]`. The rotation matrix `R(rv)` is computed via the Rodrigues formula with a regularized denominator (`ε = 1e-20`) for numerical stability near zero rotation.

### 3. Rate Limits (6N inequality constraints)

```
|u[k] - u[k-1]| ≤ v_max × dt_rate
```

Where `v_max` is `max_leg_vel_mmps` (default 300 mm/s, raised to 1000 mm/s for catch modes) and `dt_rate` is the physical time available for the transition.

## Symbolic IK

The IK is implemented in CasADi's symbolic algebra (`SX` type) so that IPOPT can compute exact Jacobians and Hessians via automatic differentiation.

### Rodrigues Formula

The rotation vector `[rx, ry, rz]` is converted to a 3×3 rotation matrix using Rodrigues' formula:

```
θ = ||rv||
K = skew(rv)
R = I + (sin θ / θ) × K + ((1 - cos θ) / θ²) × K²
```

The denominator is regularized: `θ² + 1e-20` ensures `sin θ / θ → 1` and `(1 - cos θ) / θ² → 0.5` as `θ → 0`, avoiding division by zero at the home orientation.

### Leg Vector Geometry

For each leg `i`:

```
plat_world_i = position + [0, 0, init_height] + R × plat_node_i
leg_vec_i = plat_world_i - base_node_i
extension_i = ||leg_vec_i|| - init_length_i
```

The `init_length_i` values are the geometric leg lengths at home (before home extension offset).

## Warm-Starting

IPOPT supports warm-starting from a previous solution, dramatically reducing solve time (typically 1-3 ms warm vs 10-15 ms cold).

### Shift Strategy

Each solve, the previous optimal solution is shifted by one timestep:

```
u_warm[k] = u_opt[k+1]     for k = 0..N-2,  u_warm[N-1] = u_opt[N-1]
q_warm[k] = q_opt[k+1]     for k = 0..N-2,  q_warm[N-1] = q_opt[N-1]
p_warm[k] = p_opt[k+1]     for k = 0..N-2,  p_warm[N-1] = p_opt[N-1]
```

The last node is duplicated (held constant). Lagrange multipliers (`lam_g`, `lam_x`) are also carried forward.

### Cold Start

When no previous solution exists (first solve or after reset), the initial guess is constructed by:

1. Interpolating the platform pose linearly from current to target, using cumulative time as the interpolation parameter.
2. Computing IK-consistent leg extensions for each interpolated pose via numerical IK.
3. Clamping commands to the stroke margin bounds.

## Solver Configuration

| IPOPT Option | Value | Rationale |
|---|---|---|
| `max_iter` | 200 (sim: 500) | Enough for convergence; higher in sim where timing isn't real-time |
| `max_cpu_time` | 0.018 s | 90% of 20 ms budget (sim uses 2.0 s) |
| `tol` | 1e-4 | Adequate for mm-scale positioning |
| `warm_start_init_point` | yes | Exploit previous solution |
| `warm_start_bound_push` | 1e-8 | Tight bounds for warm-start |
| `print_level` | 0 | Silent operation |

## Failure Handling

When IPOPT fails to converge (max iterations, infeasibility, timeout):

1. **Consecutive failure count < 10:** Apply the first step of the shifted previous solution (clamped to margin bounds). This provides a gracefully-degrading trajectory.
2. **Count >= 10 but previous command exists:** Hold last command (zero delta). Safe because position control holds the platform in place.
3. **No previous command at all:** Command all legs to the margin position (lowest feasible). This is the absolute fallback on first-solve failure.

The consecutive failure counter resets to zero on any successful solve.
