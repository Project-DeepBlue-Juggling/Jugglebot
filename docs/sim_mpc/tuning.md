# Tuning Guide

This page documents the MPC's tunable parameters, their effects, and guidance for adjusting them.

**Source file:** `controller/params.py`

## Parameter Reference

All parameters live in the `MPCParams` dataclass. Defaults are set for Jugglebot's workspace and a 50 Hz control rate.

### Horizon

| Parameter | Default | Units | Effect |
|---|---|---|---|
| `dt_schedule` | `(0.02,) × 5 + (0.25,) × 5` | seconds | Timestep at each horizon node |

Derived properties:

| Property | Value | Meaning |
|---|---|---|
| `N` | 10 | Number of prediction steps |
| `dt_fine` | 0.02 s | Control loop period |
| `horizon_s` | 1.35 s | Total prediction horizon |
| `cumulative_times` | `[0, 0.02, ..., 1.35]` | Absolute time at each node |

**Tuning notes:**
- More fine steps → better near-term tracking, more variables, slower solve.
- More coarse steps → longer planning horizon, marginal solve cost increase.
- The control loop rate should match `dt_fine`. If you change one, change the other.

### Actuator Model

| Parameter | Default | Units | Effect |
|---|---|---|---|
| `tau` | 0.03 | seconds | First-order lag time constant |

This models the delay between commanding a leg extension and the actuator reaching it. The value should match the real actuator response:

- **Too low** (τ < actual): MPC assumes actuators are faster than they are → overshoot, oscillation.
- **Too high** (τ > actual): MPC is overly cautious → sluggish response, wasted travel time.
- **Correct**: Predicted extensions match actual → smooth tracking.

With the MuJoCo model, τ should match the PD actuator response in the MJCF. With real hardware, measure the step response and fit a first-order model.

### Tracking Weights

| Parameter | Default | Units | Effect |
|---|---|---|---|
| `Q_pos` | 10.0 | per mm² | Stage position tracking cost |
| `Q_ori` | 1000.0 | per rad² | Stage orientation tracking cost |
| `Qf_pos` | 50.0 | per mm² | Terminal position tracking cost |
| `Qf_ori` | 5000.0 | per rad² | Terminal orientation tracking cost |
| `Q_vel_lin` | 0.001 | per (mm/s)² | Linear velocity tracking cost |
| `Q_vel_ang` | 0.1 | per (rad/s)² | Angular velocity tracking cost |

**Weight rationale:**

The position/orientation ratio `Q_ori / Q_pos = 100` means 1 mm position error costs the same as `1/√100 = 0.1 rad ≈ 5.7°` orientation error. This makes 1 mm ≈ 1° in cost terms, which is a reasonable balance for catching.

Terminal weights are 5× the stage weights, providing extra pull toward the target at the horizon end.

Velocity weights are intentionally low. Higher values degrade position tracking without improving velocity accuracy — the actuator lag τ is the bottleneck, not the cost weight.

**Tuning guidance:**
- If the platform consistently undershoots the target → increase `Qf_pos` / `Qf_ori`.
- If the platform oscillates near the target → decrease `Q_pos` / `Q_ori` or increase smoothness `S`.
- If throw velocity at release is inaccurate → increase `Q_vel_lin` (but watch position tracking).

### Control Cost

| Parameter | Default | Units | Effect |
|---|---|---|---|
| `R` | 1e-4 | per mm² | Control effort (deviation from active pose) |
| `S` | 0.02 | — | Command rate smoothness (Δu/dt) |
| `A` | 0.004 | — | Command acceleration smoothness (ΔΔu/dt) |

**Effect of each:**

- **R (effort):** Penalizes commands far from the active pose. Very small — just enough to regularize the NLP and break ties. Increasing it makes the platform reluctant to move far from the active pose.
- **S (smoothness):** Penalizes rapid command changes. Higher values → smoother but slower motion. This is the primary knob for trading speed vs smoothness.
- **A (acceleration):** Penalizes jerky command changes (second derivative). Prevents high-frequency oscillation in the command signal. Increasing it makes transitions smoother but reduces the platform's ability to make sharp corrections.

**Tuning guidance:**
- If commands oscillate or chatter → increase `S` or `A`.
- If the platform is too sluggish → decrease `S`.
- If the platform tracks well but the commands look noisy → increase `A` while keeping `S` unchanged.

### Constraints

| Parameter | Default | Units | Effect |
|---|---|---|---|
| `stroke_mm` | 280.0 | mm | Physical leg stroke (hard bound on u and q) |
| `stroke_margin_mm` | 5.0 | mm | Safety margin from each end |
| `max_leg_vel_mmps` | 300.0 | mm/s | Maximum leg command rate |

**Notes:**
- `stroke_margin_mm` only affects the cold-start and failure fallback clamping, not the NLP bounds. The NLP bounds use the full `[0, stroke_mm]` range.
- `max_leg_vel_mmps` defaults to 300 mm/s for the **online MPC** (conservative for smooth pose tracking). The **feasibility checker** (`sim/hand/feasibility.py`) builds its own `MPCParams` with `max_leg_vel_mmps=1000` mm/s (close to hardware max ~1060) to evaluate reachability at the platform's full physical capability. The online MPC itself always uses the configured value — it does not dynamically switch velocity limits between modes.

### Solver Options

| Parameter | Default | Effect |
|---|---|---|
| `max_iter` | 200 | Maximum IPOPT iterations |
| `max_cpu_time` | 0.018 s | CPU time limit (90% of control period) |
| `tol` | 1e-4 | Convergence tolerance |
| `warm_start` | True | Use previous solution as initial guess |
| `print_level` | 0 | IPOPT verbosity (0 = silent) |

Additional IPOPT options set in the NLP construction (not exposed in `MPCParams`):

| Option | Value | Effect |
|---|---|---|
| `ipopt.sb` | `'yes'` | Suppress IPOPT startup banner |
| `ipopt.warm_start_mult_bound_push` | `1e-8` | Tight dual variable bounds for warm-start (matches `warm_start_bound_push`) |

**Simulation overrides:** `main.py` sets `max_iter=500` and `max_cpu_time=2.0` because the simulation isn't real-time constrained. These would need to revert for real hardware.

### Failure Handling

| Parameter | Default | Effect |
|---|---|---|
| `max_consecutive_failures` | 10 | After this many failures, switch from "apply shifted solution" to "hold last command" |

## Typical Solve Performance

With default parameters and warm-starting:

| Scenario | Solve Time (warm) | Solve Time (cold) |
|---|---|---|
| Static hold (at target) | 0.5-1.5 ms | 8-15 ms |
| Moderate motion | 1-3 ms | 10-20 ms |
| Aggressive catch maneuver | 3-8 ms | 15-40 ms |
| Infeasible target | 5-18 ms (hits max_iter) | 15-50 ms |

Warm-starting provides a 5-10× speedup on typical solves. The first solve after a `reset()` is always cold.

## Common Tuning Scenarios

### Scenario: Platform overshoots the target

The platform reaches the target but oscillates around it.

1. Increase `S` (smoothness) — e.g., `S=0.05`.
2. If oscillation is high-frequency, also increase `A` — e.g., `A=0.01`.
3. If oscillation persists, check that `tau` matches the actual actuator response.

### Scenario: Platform doesn't reach catch targets in time

The platform arrives late for timed targets.

1. Increase `Qf_pos` / `Qf_ori` to strengthen deadline pull from the terminal cost.
2. Increase `max_leg_vel_mmps` if the rate limits are binding.
3. Decrease `S` (smoothness) to allow more aggressive command changes.
4. Check feasibility: if the coarse MPC says "feasible" but the real MPC misses, see B-02 and B-03 in [MPC_BUGS.md](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/sim/MPC_BUGS.md).

### Scenario: Solver frequently fails

IPOPT returns "Maximum_Iterations_Exceeded" or "Restoration_Failed".

1. Check `max_cpu_time` — is the solver hitting the time limit? If so, increase it (sim only).
2. Try `max_iter=500` to see if convergence just needs more steps.
3. Inspect warm-start: after a reset or abrupt reference change, the first few solves may be cold. This is normal.
4. Check for infeasible targets — the NLP may be genuinely infeasible if the target is outside the workspace.

### Scenario: Jerky motion at the fine/coarse tier boundary

Motion is smooth within each tier but has a visible kink where fine meets coarse.

1. Increase `A` (acceleration smoothness) — this specifically targets tier-boundary jerk.
2. Consider adding a third intermediate tier (e.g., 3 × 20 ms + 3 × 80 ms + 4 × 250 ms).
3. See B-01 in [MPC_BUGS.md](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/sim/MPC_BUGS.md) for the rate limit tightness issue at the boundary.
