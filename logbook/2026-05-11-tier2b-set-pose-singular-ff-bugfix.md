---
title: HardwarePlant set_pose — once-only singular-FF warning (Tier 2b bugfix)
type: bugfix
date: 2026-05-11
status: resolved
phase: "mpc-sadpath-coverage-tiers-1-3 — Phase 5 (bugfix)"
related_plan: "mpc-sadpath-coverage-tiers-1-3.md"
related_entries:
  - 2026-05-11-tier2b-hardware-plant-telemetry-ff
  - 2026-05-11-tier1c-input-fuzz-bugfix
files_changed:
  - controller/hardware_plant.py
  - tests/sim/test_hardware_plant_failure_paths.py
  - logbook/2026-05-11-tier2b-set-pose-singular-ff-bugfix.md
  - logbook/INDEX.md
commits:
  - 7867589
subsystem:
  - controller
  - hardware-plant
  - safety
tags:
  - bugfix
  - safety
  - watchdog
  - hardware-plant
  - set-pose
  - feedforward
  - same-session-fix
---

# HardwarePlant set_pose — once-only singular-FF warning (Tier 2b bugfix)

## Summary

Same-session bugfix for the T-U-T2b-6 finding from
[logbook/2026-05-11-tier2b-hardware-plant-telemetry-ff.md](2026-05-11-tier2b-hardware-plant-telemetry-ff.md)
(commit `3e71ce5`).

**Bug.** `HardwarePlant.set_pose()` silently propagates an all-zero
feedforward torque to `_ff_torque_buf` whenever
`dynamics.compute_full_feedforward_torques` catches a `LinAlgError`
from `np.linalg.solve(J.T, W_total)` and returns `np.zeros(6)`
(`dynamics.py:341–344`).  Asymmetric to `get_state()`'s parallel
singular-Jacobian handler at `hardware_plant.py:737–740` which
emits a once-only 'Jacobian singular' warning on its twist-solve
path.  Operator gets no signal that the FF model has failed at a
near-singular workspace boundary; PID then has to handle gravity +
inertia alone.

**Fix.** In `HardwarePlant.set_pose()` after the
`cartesian_to_motor_commands` call, detect the all-zero
`torque_ff_Nm` symptom (the gravity wrench is non-zero in any
pose with `feedforward_enabled=True`, so all-zero reliably
indicates the singular fallback fired).  Emit a once-only
warning via a new `_singular_ff_warned` flag; reset to False on
recovery (mirrors the `_jacobian_singular_warned` once-only
pattern in `get_state()`).

**Verification.**  T-U-T2b-6 was xfail-strict in commit `3e71ce5`;
this commit removes the xfail marker.  Suite count:
**1252 passed + 1 xfailed** (`pytest tests/ -q`, run 2026-05-11
post-bugfix) — +1 from the previous 1251/2 (T-U-T2b-6 now passes;
only the inherited T-U-T1a-4 xfail remains).

## Motivation

[`2026-05-11-tier2b-hardware-plant-telemetry-ff.md`](2026-05-11-tier2b-hardware-plant-telemetry-ff.md)
Discussion §*"T-U-T2b-6 — the surfaced bug"* documents the trace
and the rationale for fixing in the same session.  Per CLAUDE.md's
*"Fix surfaced bugs in the same session when diagnosis is clear"*
rule:

> When a test commit (or audit) surfaces a real bug AND the
> diagnosis is clear AND the fix is small/well-scoped, address it
> in the same session — not as a deferred end-of-plan obligation.

All three criteria held for T-U-T2b-6:

* **Real bug** — silent failure on a safety-critical path
  (singular-J FF fallback with no operator notification).
* **Clear diagnosis** — the LinAlgError catch is visible at
  `dynamics.py:341–344`; the missing handler in `set_pose` is the
  gap; the asymmetry to `get_state()`'s pattern at `:737–740`
  identifies the right fix shape.
* **Small fix** — adds a flag init + ~12 lines in `set_pose`;
  removes the `@pytest.mark.xfail` marker from one test.

The bugfix lands as the immediate follow-up commit to the Phase 5
test commit (mirrors the Phase 3 → Phase 3 bugfix arc — cf.
[`logbook/2026-05-11-tier1c-input-fuzz-bugfix.md`](2026-05-11-tier1c-input-fuzz-bugfix.md)).

## Design

### Symptom detection over cause detection

The bug surface is `dynamics.py:341–344`'s silent `except
LinAlgError: return np.zeros(6)`.  Two fix shapes were considered:

(α) **Detect cause** — refactor `dynamics.py` to NOT catch
LinAlgError; let it propagate; `set_pose` catches and warns.
Clean fail-fast layering but changes the `dynamics.py` contract;
other call sites of `compute_full_feedforward_torques` may depend
on the silent-zero behaviour (the same try/except pattern exists
at `dynamics.py:256` for a different function — fan-out unclear).
(β) **Detect symptom** — check if `torque_ff_Nm` is all-zero in
`set_pose` after the call.  Once-only flag + reset-on-recovery
mirrors `get_state()`'s pattern.  Localised; layering preserved
(dynamics stays pure math; hardware_plant owns the safety logging).

**(β) selected** — confirmed by the user via AskUserQuestion.
Rationale:

* The singular-FF detection becomes a `set_pose`-specific concern,
  matching the symmetry argument with `get_state()`'s singular-J
  handler.  Both surfaces (read state, write commands) hit the
  same kind of workspace event; both should warn.
* The dynamics module stays a pure-math layer with no logging
  side-effects — a reusable property across sim and hardware.
* The false-positive surface is small: the gravity wrench is
  non-zero in any pose with `feedforward_enabled=True`, so
  `_last_ff_torque_max_Nm == 0.0` reliably indicates the singular
  fallback fired.  A degenerate pose where torque_ff is genuinely
  all-zero (some symmetry?) is implausible in practice; if it
  does occur, the cost is one spurious WARN record per such
  episode (no behaviour change).

### Reusing the existing once-only flag pattern

`_jacobian_singular_warned` at `hardware_plant.py:218` (init) +
`:740` (set) is the precedent.  The bugfix adds an exactly
parallel `_singular_ff_warned`:

* Init: at the end of the FK/Jacobian state block in
  `HardwarePlant.__init__` (immediately after
  `_jacobian_singular_warned = False`), with a comment that
  cross-references the get_state() parallel.
* Set: at the end of `set_pose()` after the FF computation —
  `if self._last_ff_torque_max_Nm == 0.0: warn-and-set` else
  `reset-to-False`.  Edge-triggered (matching the
  `_telem_stale_warned` and `_jacobian_singular_warned`
  patterns).

### Symptom = `_last_ff_torque_max_Nm == 0.0`, not `torque_ff_Nm`

The detector reuses the already-computed `self._last_ff_torque_max_Nm =
float(self._ff_abs_scratch.max())` — the max-abs reduction was
already done as a diagnostic.  Reading `_last_ff_torque_max_Nm == 0.0`
avoids a second pass over the torque vector.

Equivalent semantics: `_last_ff_torque_max_Nm == 0.0`
⇔ `np.array_equal(torque_ff_Nm, np.zeros(6))` (since max-abs is
zero iff every element is zero, for finite inputs).  Non-finite
inputs (NaN/Inf) would set `_last_ff_torque_max_Nm` to NaN/Inf,
so the comparison `== 0.0` reliably distinguishes the all-zero
case.

## Implementation

### controller/hardware_plant.py — two additions

**1. `__init__` — flag initialisation (after `_jacobian_singular_warned = False`):**

```python
# Once-only warning flag for the set_pose() singular-FF fallback.
# dynamics.compute_full_feedforward_torques silently catches
# LinAlgError from np.linalg.solve(J.T, W_total) and returns
# np.zeros(6).  set_pose() detects the all-zero symptom and emits
# a once-only warning, mirroring _jacobian_singular_warned for
# the parallel handler in get_state()'s twist-solve path
# (:737–740).  Reset to False when set_pose sees a non-zero
# torque_ff again (the platform left the singular region).
self._singular_ff_warned = False
```

**2. `set_pose` — detector at the end (after `_has_ff_torque = True`):**

```python
# Singular-FF detection.  dynamics.compute_full_feedforward_torques
# silently catches LinAlgError on np.linalg.solve(J.T, W_total)
# and returns np.zeros(6) — leaving the operator with no signal
# that the FF model has failed at this pose.  Mirror the
# once-only Jacobian-singular warning in get_state()'s
# twist-solve path (:737–740): emit on the all-zero edge, reset
# when FF recovers.  The gravity wrench is non-zero in any pose
# with feedforward_enabled=True, so an all-zero torque_ff
# reliably indicates the singular fallback fired.
if self._last_ff_torque_max_Nm == 0.0:
    if not self._singular_ff_warned:
        logger.warning(
            "set_pose: torque_ff is all-zero (gravity wrench "
            "expected to be non-zero) — Jacobian likely singular "
            "at this pose; FF defaulted to zeros (see "
            "dynamics.py:341–344)"
        )
        self._singular_ff_warned = True
else:
    self._singular_ff_warned = False
```

### tests/sim/test_hardware_plant_failure_paths.py — remove xfail marker

The `@pytest.mark.xfail(strict=True, ...)` decorator is removed
from `TestSetPoseFfSingular.test_t2b_6_set_pose_singular_ff_warns_once`;
the test docstring is updated to reference this bugfix entry as
the source of the unmarking.

## Verification

Each cited count carries the (date, exact pytest invocation, result)
triple per the workflow rule on test-count claims.

### Pre-bugfix baseline (Phase 5 test commit)

* `pytest tests/ -q`, run 2026-05-11 against SHA `d0b3c89`
  (Phase 5 test commit + SHA backfill): **1251 passed + 2 xfailed
  in 351.06 s.**  T-U-T2b-6 in the xfailed pool.

### Post-bugfix target test

* `pytest tests/sim/test_hardware_plant_failure_paths.py::TestSetPoseFfSingular -v`,
  run 2026-05-11: **1 passed in 1.28 s.**  T-U-T2b-6 now passes
  (without the xfail marker), confirming `_singular_ff_warned` is
  set and one once-only WARN record is emitted on the all-zero
  symptom.

### Module-isolated re-run (Phase 4 + Phase 5 combined)

* `pytest tests/sim/test_hardware_plant_failure_paths.py -q`, run
  2026-05-11 post-bugfix: **19 passed in 6.57 s.**  All Phase 4
  tests + 9 Phase 5 tests (T-U-T2b-6 no longer xfail) pass.

### Hot-loop allocation contract — post-bugfix regression check

* `pytest tests/sim/test_hot_loop_allocation_contract.py
  --hypothesis-profile=ci-deep --hypothesis-seed=0 -q`, run
  2026-05-11 post-bugfix: **3 passed in 16.48 s.**  The two new
  lines in `set_pose` (a scalar compare + a boolean assign) are
  zero-allocation; budget unchanged.

### Full-suite gate (post-bugfix)

* `pytest tests/ -q`, run 2026-05-11 post-bugfix: **1252 passed
  + 1 xfailed in 345.51 s.**  T-U-T2b-6 promotes from xfail to
  pass; T-U-T1a-4 remains as the sole xfail (permanent,
  CasADi 3.7.2 structural limitation).

## Discussion

### Why the symptom detector won't false-positive

The fix detects `_last_ff_torque_max_Nm == 0.0` AND
`feedforward_enabled` (the early `if not self._enable_torque_ff:
return` path skips the detector entirely).  The remaining surface
is: every `set_pose` call with `feedforward_enabled=True` and a
finite torque_ff returned by `cartesian_to_motor_commands`.

The torque components have three sources:

1. **Gravity wrench** — `compute_gravity_wrench(rot, params)` at
   `dynamics.py:333`.  The platform has mass; gravity always
   produces a non-zero wrench in world frame; the J^{-T}
   decomposition distributes it across the legs.  At ANY pose, at
   least some leg torque is required to support the platform —
   so torque_platform is non-zero.
2. **Inertia wrench** — `compute_inertia_wrench(rot, twist, accel,
   params)` at `:337`.  Zero when twist=accel=0; non-zero
   otherwise.
3. **Reflected motor inertia** — gated on
   `skip_reflected_inertia=False`.  Production code uses
   `skip_reflected_inertia=True` (hardware_plant.py:816), so
   this is always zero for the hardware path.

The aggregate `torque_ff_Nm` is the sum.  Gravity alone guarantees
non-zero unless:

* Numerical underflow somewhere in the dynamics chain (implausible
  — gravity is ~9.81 m/s² × platform mass, well within float64 range).
* `np.linalg.solve(J.T, W_total)` catches LinAlgError → returns
  zeros (THE bug surface; what the detector catches).
* The platform mass and platform_height are zero (config bug —
  would surface elsewhere).

So the detector's false-positive rate is structurally zero on
the well-formed-config path.  An ill-formed config that produces
zero gravity is a separate issue that would fire other warnings
before reaching `set_pose`.

### Why edge-triggered (not per-tick)

The MPC runs at 40 Hz.  If the platform spends N ticks at a
singular pose (e.g., near a workspace boundary on a fast move),
N WARN records per second would saturate the log.  Edge-triggered
(once-only-per-streak, reset on recovery) follows the established
pattern for `_jacobian_singular_warned` and `_telem_stale_warned`
in this file.  Operator gets one signal per singular *event*,
not per tick.

### Hot-loop allocation impact

The two new lines (`if self._last_ff_torque_max_Nm == 0.0:`,
`self._singular_ff_warned = True/False`) are pure scalar ops on
already-existing instance fields.  No new allocations; no new
ndarrays.  The hot-loop allocation contract test
(`test_hot_loop_allocation_contract.py`) re-ran post-bugfix
(`pytest tests/sim/test_hot_loop_allocation_contract.py
--hypothesis-profile=ci-deep --hypothesis-seed=0 -q`, 2026-05-11,
3 passed in 16.48 s) — budget unchanged.

The `logger.warning(...)` call on the singular edge IS an
allocation, but it fires at most once per singular *streak*
(amortised effectively zero per tick) and the existing
`_jacobian_singular_warned` precedent at `:738–740` makes the
same trade.

### Rollback discipline

Per the plan's *"Production-code changes triggered by tests"*
subsection: this bugfix is reverting-independent of the Phase 5
test commit (`3e71ce5`).

* Reverting this commit alone: T-U-T2b-6 returns to failing
  (because the `xfail(strict=True)` marker is also removed in
  this commit).  Symptom: the suite reports 1 failure on
  T-U-T2b-6 — the same fingerprint the bug originally produced.
  To restore xfail-strict status post-revert, re-add the marker
  (one-line change).
* Reverting Phase 5 test commit alone: removes the test surface
  entirely; the bugfix in `set_pose` remains but is untested.
* Reverting both: clean state pre-Phase-5.

The independence is intentional — protects rollback granularity
while honouring the same-session-fix discipline that prevented
the bug from being deferred to a later session.

### Why this bug existed in the first place

Two structural reasons it took until Phase 5 to surface:

1. **The asymmetry was easy to miss.**  `get_state()` and
   `set_pose()` are in the same file, but the FF computation
   inside `set_pose` is delegated to `cartesian_to_motor_commands`
   → `compute_full_feedforward_torques` (across two modules).
   The LinAlgError catch is at `dynamics.py:341–344`, far from
   `hardware_plant.py:737–740` where the parallel warning lives.
   A reviewer looking at either file in isolation would not see
   the gap.
2. **The silent-zero fallback is the safer behaviour.**  Zero
   torque_ff is fail-safe (defer to PID); NaN/Inf would be
   dangerous.  The fallback exists for sound reasons; the only
   missing piece was the operator notification.  Reviewers
   reading the dynamics code likely saw the silent-zero as
   correctness-preserving (which it is) without recognising
   that the *log* was the missing signal.

T-U-T2b-6's contribution is the **systematic check** —
hardware_plant tests that drive each safety-critical path through
its handler will catch this class of "fail-safe but
fail-silent" bugs by structure, not by review.

## Open Questions

* **Should `dynamics.py` also surface the LinAlgError (e.g., via
  logging) for visibility from non-hardware_plant callers?**
  Currently only `set_pose` is hooked up.  Other consumers of
  `compute_full_feedforward_torques` (the sim, the standalone
  trajectory tools) would silently get zeros without notification.
  Filed as a Phase 7 follow-up — the schema-completeness work is
  the natural home for "every output channel must surface its
  fail-states".

* **Does the reset-on-recovery semantic risk log saturation if
  the platform repeatedly traverses a workspace singularity at
  high frequency?**  The same concern applies to
  `_jacobian_singular_warned` — Phase 4's logbook open question
  flagged it then.  No change here; if either flag's log noise
  becomes a problem in production, the fix is a per-session
  reset semantic (warn once per *session*, not per *streak*).

## Related

* [logbook/2026-05-11-tier2b-hardware-plant-telemetry-ff.md](2026-05-11-tier2b-hardware-plant-telemetry-ff.md)
  — Phase 5 test commit; bug surface, design discussion (Discussion
  §"T-U-T2b-6 — the surfaced bug" / §"Why detect-symptom over
  detect-cause in the fix").
* [logbook/2026-05-11-tier1c-input-fuzz-bugfix.md](2026-05-11-tier1c-input-fuzz-bugfix.md)
  — Phase 3 → Phase 3 bugfix; same-session-fix arc precedent.
* [plans/archived/mpc-sadpath-coverage-tiers-1-3.md](../plans/archived/mpc-sadpath-coverage-tiers-1-3.md)
  — Plan 2 Phase 5 specification + "Production-code changes
  triggered by tests" subsection.
* [controller/hardware_plant.py](../controller/hardware_plant.py)
  — `_singular_ff_warned` init + `set_pose` detector.
* [ros_ws/src/jugglebot/jugglebot/motion/dynamics.py](../ros_ws/src/jugglebot/jugglebot/motion/dynamics.py)
  — `compute_full_feedforward_torques` silent LinAlgError catch
  at `:341–344` (the bug surface; unchanged this commit).
* [tests/sim/test_hardware_plant_failure_paths.py](../tests/sim/test_hardware_plant_failure_paths.py)
  — `TestSetPoseFfSingular.test_t2b_6_set_pose_singular_ff_warns_once`;
  xfail marker removed in this commit.
