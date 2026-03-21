# MPC Bug Tracker

Issues identified through code review of the CasADi NMPC implementation.
Each entry has a severity, status, and space for implementation notes.

**Files in scope:**
`sim/controller/mpc.py`, `sim/controller/params.py`, `sim/main.py`,
`sim/hand/coordinator.py`, `sim/hand/feasibility.py`, `sim/plant/mujoco_plant.py`

---

## Summary Table

| ID   | Severity | Status | Title                                                        |
|------|----------|--------|--------------------------------------------------------------|
| B-01 | Medium   | Open   | Rate limits overly tight at tier boundary                    |
| B-02 | Medium   | Open   | Urgency ramp wastes lead time for distant deadlines          |
| B-03 | Medium   | Open   | Feasibility checker is optimistic (ASAP vs timed mode)       |
| B-04 | Low      | Open   | Fallback on target=None causes abrupt home reference         |
| B-05 | Low      | Open   | Warm-start shift assumes control rate matches dt_schedule[0] |
| B-06 | Low      | Open   | Failure fallback degrades silently over consecutive failures |
| B-07 | Low      | Open   | No validation on dt_schedule (zero-division possible)        |
| B-08 | Info     | Open   | Telemetry logs pre-step state with current-step command      |
| B-09 | Info     | Open   | Urgency × terminal cost interaction may over-commit          |
| B-10 | Info     | Open   | Cold start interpolates toward final ref only                |
| B-11 | Info     | Open   | Numerical vs symbolic IK convention mismatch (not a bug)     |
| B-12 | Info     | Open   | WaypointTargetSource skips waypoints at their trigger time   |

---

## B-01: Rate limits overly tight at tier boundary

**Severity:** Medium
**Status:** Open
**File:** `sim/controller/mpc.py` — lines 248, 317-321

### Description

Rate limits for the u[k-1] to u[k] transition use `dt_schedule[k-1]` (the
*previous* step's duration). At the tier boundary (k=5, the first coarse
step), this means the u[4] to u[5] delta is constrained by `dt_schedule[4]
= 0.02s`, even though the coarse step u[5] controls a 250ms interval.

The actuator has 250ms to execute this command delta, but the rate limit
only allows `v_max * 0.02` of movement — **12.5x tighter than the coarse
tier's actual capacity**. This forces the optimizer to plan sluggish
transitions across the tier boundary.

### Observed Impact

The optimizer may report higher-than-necessary cost near the tier boundary
and produce suboptimal coarse-tier trajectories, particularly for fast
dynamic targets that require large command deltas near the fine/coarse
boundary.

### Suggested Fix

At the tier boundary, use the coarse step's dt (or the geometric mean, as
done for smoothness cost) for the rate limit:

```python
# Option A: use the interval being *commanded* (dt_schedule[k])
dt_rate = dt_schedule[0] if k == 0 else dt_schedule[k]

# Option B: geometric mean at boundaries (mirrors smoothness logic)
dt_rate = dt_schedule[0] if k == 0 else math.sqrt(dt_schedule[k-1] * dt_schedule[k])
```

### Fix Notes

_None yet._

---

## B-02: Urgency ramp wastes lead time for distant deadlines

**Severity:** Medium
**Status:** Open
**File:** `sim/controller/mpc.py` — lines 600-632

### Description

When `arrival_time` is far in the future (e.g. 5s away with a 1.35s
horizon), every node's `time_to_deadline` is large, so all urgency values
equal `urgency_base = 0.05`. This makes the tracking cost nearly zero — the
MPC has almost no incentive to begin moving toward the target.

The ramp only kicks in when the deadline is within `urgency_ramp_s = 0.5s`
of a horizon node. With a 1.35s horizon, the MPC won't see urgency above
base until the deadline is ~0.85s away. For targets requiring significant
motion (e.g. full-stroke catch), this wastes the first few hundred
milliseconds of available lead time.

### Observed Impact

The platform may "wait and then rush" toward timed targets. This is
especially problematic for catch targets with tight timing margins, where
the MPC has 0.5-1.0s of lead time and should begin moving immediately.

### Suggested Fix

Consider a two-tier urgency: a modest floor (e.g. 0.3) for all nodes when
a timed target is active, with the ramp boosting further near the
deadline. Alternatively, use a longer ramp window or start the ramp from
the horizon edge rather than from each node's absolute time.

### Fix Notes

_None yet._

---

## B-03: Feasibility checker is optimistic (ASAP vs timed mode)

**Severity:** Medium
**Status:** Open
**File:** `sim/hand/feasibility.py` — lines 148-156

### Description

The coarse MPC solve uses `arrival_time=None` (ASAP mode), which applies
uniform urgency = 1.0 to all nodes. This makes the feasibility checker
more aggressive than the real controller, which will use the urgency ramp
with a timed target (urgency_base = 0.05 far from the deadline).

A target that the feasibility checker declares reachable may not be reached
in time by the actual controller, because the actual controller's low
urgency means it starts moving later.

### Observed Impact

False-positive feasibility: targets may pass the feasibility check but be
missed by the actual MPC. This is partially mitigated by the
`settle_margin_s` (100ms early arrival), but the margin may not be
sufficient for aggressive targets.

### Relationship

This is compounded by B-02. If the urgency ramp issue is fixed, this
discrepancy shrinks.

### Suggested Fix

Either:
1. Pass the actual `arrival_time` to the coarse solve so urgency behavior
   matches, or
2. Tighten the feasibility tolerances to compensate for the controller's
   sluggish start, or
3. Accept the optimism and rely on the settle margin (document the
   assumption).

### Fix Notes

_None yet._

---

## B-04: Fallback target on coordinator idle causes abrupt home reference

**Severity:** Low
**Status:** Open
**File:** `sim/main.py` — line 359 (CatchTargetSource.update),
         line 412 (ThrowCatchTargetSource.update),
         line 443 (InteractiveCatchSource.update),
         line 487 (ContinuousThrowCatchSource.update)

### Description

When the hand coordinator returns `target=None` (no active target), all
target source adapters substitute `np.zeros(6)` as the MPC target pose.
If the platform is at a non-zero pose when this transition occurs, the MPC
suddenly receives a home reference with ASAP urgency (uniform 1.0), and
will try to return aggressively.

### Observed Impact

A step discontinuity in the MPC reference at the moment the coordinator
finishes a sequence. The platform may jerk toward home if it was at an
offset pose. In practice this usually occurs after a catch/throw cycle when
the platform is near home anyway, but edge cases (infeasible target abort
mid-motion) could produce aggressive returns.

### Suggested Fix

When `target is None`, use the platform's current pose as the reference
(hold in place) instead of zeros. Or have the coordinator explicitly return
a "return to home" target with a reasonable arrival time.

### Fix Notes

_None yet._

---

## B-05: Warm-start shift assumes control rate matches dt_schedule[0]

**Severity:** Low
**Status:** Open
**File:** `sim/controller/mpc.py` — lines 531-555

### Description

`_shift_warm_start` shifts the previous optimal solution by exactly one
index position (u[1]→u[0], u[2]→u[1], ...). This is valid only when
the control loop emits one command per fine timestep (`dt_schedule[0]`).

Currently the control loop runs at 50Hz (20ms) matching `dt_schedule[0] =
0.02s`, so this is correct. However, if:
- the control rate changes independently of the schedule, or
- MPC solves are skipped (e.g. due to timeouts exceeding one control
  period),

the one-index shift no longer corresponds to the elapsed time, producing
poor warm-starts and potentially slow convergence.

### Observed Impact

None currently. This is a latent fragility.

### Suggested Fix

Couple `CONTROL_DT` to `params.dt_fine` or add an assertion. For
skipped-solve robustness, shift by `n = round(elapsed / dt_fine)` steps.

### Fix Notes

_None yet._

---

## B-06: Failure fallback degrades silently over consecutive failures

**Severity:** Low
**Status:** Open
**File:** `sim/controller/mpc.py` — lines 638-673

### Description

On solver failure, the fallback applies the first step of the shifted
previous solution. After N consecutive failures, the same solution has
been shifted N times — each shift discards the last node and duplicates
the penultimate one. After ~10 failures (0.2s at 50Hz), the entire
warm-start is filled with identical values and the "apply first step"
fallback is meaningless.

The `max_consecutive_failures = 10` guard eventually switches to
hold-last-command, but the degradation from shift 1 to shift 10 is
not logged or monitored.

### Observed Impact

During a burst of failures, commands degrade smoothly into holding the
same stale value, which may appear as the platform freezing. The hold
behavior itself is safe, but the gradual degradation is invisible to
diagnostics.

### Suggested Fix

Log the consecutive failure count in the diagnostics dict. Consider
switching to hold-last-command after 2-3 failures rather than 10, since
the shifted solution is essentially worthless after a few shifts.

### Fix Notes

_None yet._

---

## B-07: No validation on dt_schedule (zero-division possible)

**Severity:** Low
**Status:** Open
**File:** `sim/controller/params.py` (entire class),
         `sim/controller/mpc.py` — lines 263, 276, 287

### Description

If a zero timestep enters `dt_schedule`:
- `S / dt_smooth` and `A / dt_smooth` divide by zero in the cost function
  (lines 263, 276).
- `alpha_k = 1 - exp(-0/tau) = 0`, freezing the actuator dynamics.
- `v_max * dt_rate = 0`, making rate limits impossible to satisfy.

There is no validation in `MPCParams.__init__` or `_build_problem()`.

### Observed Impact

None currently — the default schedule has no zeros. A configuration error
would cause a cryptic CasADi or numpy error rather than a clear message.

### Suggested Fix

Add a `__post_init__` check in `MPCParams`:

```python
def __post_init__(self):
    if any(dt <= 0 for dt in self.dt_schedule):
        raise ValueError("dt_schedule must contain only positive values")
```

### Fix Notes

_None yet._

---

## B-08: Telemetry logs pre-step state with current-step command

**Severity:** Info
**Status:** Open
**File:** `sim/main.py` — lines 686-698 (headless), 800-814 (viewer)

### Description

The loop reads `state = plant.get_state()` *before* calling `mpc.solve()`
and `plant.step()`, then logs that state alongside the command and solve
diagnostics. The logged tracking error (ref vs actual) reflects the state
*before* the command was applied, not after.

This is standard for control-system logging (the command was computed
*from* this state), but analysis tools that compare ref_pose vs actual_pose
on the same row are seeing a one-step-lagged comparison.

### Observed Impact

Tracking error metrics are one control step behind. At 50Hz this is a 20ms
lag — negligible for summary statistics, but could matter for transient
analysis or debugging fast dynamics.

### Fix Notes

_None yet. This may be intentional — document the convention._

---

## B-09: Urgency times terminal cost interaction may over-commit

**Severity:** Info
**Status:** Open
**File:** `sim/controller/mpc.py` — lines 218-224

### Description

The terminal cost uses heavier weights (`Qf_pos = 50` vs `Q_pos = 10`).
The urgency multiplier also applies to the terminal node. Near the
deadline, `urgency_max = 10` makes the effective terminal weight
`Qf_pos * urgency_max = 500`, which is 50x the base tracking weight.

This may cause the optimizer to over-prioritize reaching the terminal
pose, producing aggressive solutions that violate smoothness preferences.

### Observed Impact

Not yet observed in practice. May manifest as jerky terminal approach
for timed targets with tight deadlines.

### Suggested Fix

Consider exempting the terminal cost from the urgency multiplier, or
reducing `urgency_max` to account for the terminal weight already being
5x the stage weight.

### Fix Notes

_None yet._

---

## B-10: Cold start interpolates toward final reference only

**Severity:** Info
**Status:** Open
**File:** `sim/controller/mpc.py` — lines 487-511

### Description

`_cold_start` interpolates from the current pose toward `ref_traj[-1]`
(the last reference node). Currently `_build_reference` sets all nodes
to the same target pose, so this is correct.

If the reference construction changes in the future to include
intermediate waypoints or trajectory-shaped references, the cold start
would ignore them and always aim at the final node.

### Observed Impact

None currently. This is a maintenance note.

### Fix Notes

_None yet._

---

## B-11: Numerical vs symbolic IK convention difference

**Severity:** Info (Not a bug)
**Status:** Open
**File:** `sim/controller/mpc.py` — lines 75-93 (symbolic), 513-529 (numerical)

### Description

The symbolic IK (`_build_symbolic_ik`) returns IK-convention extensions:
`norm(leg_vec) - init_len`. The numerical IK (`_numerical_ik`) returns
home-relative extensions: `norm(leg_vec) - init_len - home_ext`.

The NLP constraint accounts for this: `ik_ext = q + home_ext` (line 295).
The cold start uses `_numerical_ik` to initialize `q[k]`, which is
home-relative — matching the constraint's convention. So everything is
consistent.

However, the two IK functions look like they compute different things,
which is a maintenance trap. A future developer might "fix" one to match
the other and introduce a real bug.

### Suggested Fix

Add a docstring to `_numerical_ik` clarifying it returns home-relative
extensions, and a comment in `_build_symbolic_ik` noting it returns
IK-convention (not home-relative).

### Fix Notes

_None yet._

---

## B-12: WaypointTargetSource skips waypoints at their trigger time

**Severity:** Info
**Status:** Open
**File:** `sim/main.py` — lines 294-299

### Description

The `while` loop in `WaypointTargetSource.update()` advances `_idx` past
any waypoint whose arrival time has been reached:

```python
while (self._idx + 1 < len(self._waypoints)
       and sim_time >= self._waypoints[self._idx][1]):
    self._idx += 1
```

This means the MPC immediately targets the *next* waypoint once the
current one's time passes. The MPC never receives the just-passed waypoint
as a reference to settle onto — it jumps ahead.

### Observed Impact

For well-spaced waypoints this is correct (plan ahead, don't linger). For
tightly-spaced waypoints, the platform may appear to skip poses. This is
a design choice, not a bug, but could confuse users who expect the
platform to visit each waypoint.

### Fix Notes

_None yet. This is the intended behavior for trajectory sequences._

---

## Appendix: Review Metadata

- **Reviewed:** 2026-03-21
- **Reviewer:** Claude (code review)
- **Codebase state:** `refactor` branch, commit `da58f9f`
- **Scope:** MPC controller, main loop, hand coordinator, feasibility checker
