---
title: MPC warm-start deadlock escape — failure-driven invalidation breaks chronic-CTE lockup
type: bugfix
date: 2026-05-20
status: resolved
phase: "off-plan / arc-spanning"
related_plan: ""
related_entries:
  - 2026-05-18-z30-solve-failure-motor-pos-none-watchdog-gap
  - 2026-05-18-hardware-bringup-t2b1-t2a1-cascade-validation
  - 2026-04-18-mpc-overhead-spikes-fallback-bursts
  - 2026-04-17-mpc-fallback-cmd-sawtooth-stutter
  - 2026-04-18-hold-fighting-motion-onset-jitter
  - 2026-04-20-k1-k6-reference-feasibility-resolution
sessions:
  - mpc_20260518_200137.csv
  - mpc_20260520_115857.csv
files_changed:
  - controller/mpc.py
  - tests/sim/test_mpc_static.py
  - logbook/2026-05-20-mpc-warmstart-deadlock-escape.md
  - logbook/INDEX.md
commits:
  - 67ae3da
subsystem:
  - controller
  - mpc
tags:
  - bugfix
  - warm-start
  - fallback
  - deadlock
  - offline-probe
  - contract-extension
---

# MPC warm-start deadlock escape — failure-driven invalidation breaks chronic-CTE lockup

## Summary

The hardware MPC could enter a self-sustaining `Maximum_CpuTime_Exceeded`
lockup. Once a few solves tripped the 22 ms CPU budget, IPOPT exited with
`iter_count = 0` on every subsequent tick (the warm-start vectors were
byproducts of failed solves and unusable as primal/dual init), the runner's
`t_ref` froze (it advances only on success), and the source presented the
bit-identical NLP every tick. The 2026-05-18 z=30 hardware session
(`temp/logs/mpc_20260518_200137.csv`) showed **1769 consecutive failures**
in this state. An offline production-faithful replay reproduces it within
0.5 pp (39.1 % probe success vs 39.6 % hardware; 1777-tick chronic run vs
1769 in the log).

**Fix**: at the top of `solve()`, when `_consecutive_failures >
max_consecutive_failures` (the same threshold that already gates
walk-forward), invalidate the warm-start vectors — mirrors the existing
W5 `warm_start_valid=False` block. Generalises the warm-start contract
from *structural-reference-shift-driven* to *also failure-driven*. Offline
result with the patch applied: **93.0 % success, longest CTE run 21 ticks
(84× reduction from 1777)**. Hardware re-run at the same target on
2026-05-20 (`temp/logs/mpc_20260520_115857.csv`): 2 isolated singleton
fallbacks, no chronic phase, final tracking error 0.041 mm.

## Context

The diagnosis arc started from an operator observation — *"MPC has never
produced clean hardware results, while the previously-used per-leg TRAP_TRAJ
had no jitter"* — and an architectural-reframe hypothesis (z=30 is
intrinsically ill-conditioned, MPC may be the wrong layer). The offline
DVFS A/B probe designed to test that, on a bare static-target reconstruction,
returned 100 % success in ~9 ms / 2 IPOPT iterations across every condition
tested (sleep vs busy pacer, isolated vs same-core-contended, freq
0.88 → 1.51 GHz). That decisively falsified the architectural concern but
also tripped the probe's pre-registered validity gate: a replay this clean
clearly wasn't reproducing the hardware failure.

The discrepancy localised to three production-path ingredients the bare-target
probe had omitted: **`ref_events`** (the K1–K6 quintic reference list),
**`boost_vel_weights=True`** (heavier velocity-tracking weights, which
`StaticTargetSource` sets unconditionally), and the **t_ref-freeze-on-fallback**
rule in `controller/runner.py`. A production-faithful probe with all three
restored — instantiating `StaticTargetSource(schedule=[(0.0, target)],
**src_kwargs)` and calling `source.update(t_ref, state); mpc.solve(state,
tc.target_pose, ref_events=tc.ref_events, boost_vel_weights=tc.boost_vel_weights,
t_now=t_ref, warm_start_valid=tc.warm_start_valid)` per the runner — reproduced
the chronic failure within 0.5 pp.

## Diagnosis

Three facts, all observable directly in the chronic-phase probe output, fully
specify the mechanism:

1. **`iter_count = 0` on every chronic tick.** IPOPT exits with
   `Maximum_CpuTime_Exceeded` *before* completing iteration 1. The ~25 ms
   solve time is burned entirely in IPOPT init / KKT factorisation /
   warm-start consumption — no iterative progress.
2. **`t_ref` is frozen at 29.750 s throughout the chronic phase.**
   `controller/runner.py:584-587` advances `t_ref` only on success-class
   statuses; every chronic-phase tick is fallback, so `t_ref` never advances.
   The source samples `ref_events` at a fixed `t_now`, making the MPC's
   optimisation problem **bit-identical every tick**.
3. **`warm_start_valid = True` on every chronic-phase tick.** Static target ⇒
   `target_changed = False` ⇒ `flat_target_to_events` is not recomputed ⇒
   `is_warm_start_invalidating` is never called ⇒ the runner-passed
   `warm_start_valid` stays True throughout. The MPC keeps reusing
   `_prev_w` / `_prev_lam_g` / `_prev_lam_x` / `_timeout_hint` — vectors
   that are byproducts of failed solves.

Together: a self-sustaining warm-start-corruption + t_ref-freeze deadlock.
The "trigger" is just one or two ticks creeping past the 22 ms cap as the
post-settle solve times drift upward (probe ticks 1196–1200 ran at
19–26 ms, right at the edge). Once a fallback fires:
`_consecutive_failures` increments, walk-forward emits the cached plan's
next step (preserving `_prev_w`). When the counter exceeds
`max_consecutive_failures = 3`, walk-forward stops and the loop enters
`hold_extrap`. **No code path in `hold_extrap` (or anywhere else after
fallback) resets the warm-start.** Escape is structurally impossible for
that solver instance — the only existing reset is `warm_start_valid=False`
from the caller, which won't fire for a static target.

## Fix

At the top of `solve()`, immediately after the existing W5
`warm_start_valid=False` block, add a second invalidation trigger that
mirrors its clearing list exactly:

```python
# Failure-driven warm-start invalidation (deadlock escape).
# Once walk-forward has been exhausted (_consecutive_failures >
# max_consecutive_failures), the fallback chain has moved into
# hold_extrap and the warm-start vectors are byproducts of
# repeatedly-failed solves.  Reusing them creates the self-sustaining
# CTE cascade where IPOPT exits with iter_count=0 before even one
# iteration completes.  Mirrors the warm_start_valid=False block above
# — same clearing, different trigger.
if self._consecutive_failures > self._params.max_consecutive_failures:
    if self._prev_w is not None or self._timeout_hint is not None:
        logger.warning(
            "MPC: %d consecutive failures > max=%d — invalidating "
            "warm-start (deadlock escape)",
            self._consecutive_failures,
            self._params.max_consecutive_failures,
        )
    self._prev_w = None
    self._prev_lam_g = None
    self._prev_lam_x = None
    self._timeout_hint = None
    self._timeout_lam_g = None
    self._timeout_lam_x = None
    self._fallback_step = 0
```

Properties of this patch:

- **No new `MPCParams` field.** Reuses the existing `max_consecutive_failures`
  threshold that already gates walk-forward, so there is exactly one knob
  to tune for both the "stop walk-forward" and "reset warm-start"
  semantics.
- **No behaviour change in the success path.** Trigger condition is purely
  on the failure counter; healthy operation is untouched. Hardware
  validation (2026-05-20) confirmed: 2400 ticks, 99.6 % success, zero
  spurious invalidations, log line never fired (no path was deep enough
  in failure to need it).
- **Same single canonical enforcement point** as W5. The "warm-start is
  invalid because the cached vectors no longer reflect a usable solution"
  decision lives in one place in `solve()`; this patch just extends the
  set of conditions under which it fires. Matches the codebase's
  contract-favouring discipline.
- **Edge-logged** (`if self._prev_w is not None or self._timeout_hint is
  not None`), so steady-state operation in deep fallback doesn't spam
  warnings every tick.

## Tests

`tests/sim/test_mpc_static.py::TestMPCSolverFailure::test_warmstart_invalidated_after_max_consecutive_failures`
— seeds an MPC's warm-start from a successful run, switches to a
restricted-budget MPC with `_INFEASIBLE_REF`, drives five consecutive
failures, and asserts that all four warm-start fields
(`_prev_w`, `_prev_lam_g`, `_prev_lam_x`, `_timeout_hint`) are `None`
after the threshold is exceeded. Regression guard.

Existing `test_consecutive_failures_increment` and
`test_escalation_fallback_to_hold` cover the "counter increments" and
"walk-forward → hold escalation" semantics and continue to pass — the
patch does not change them.

Full suite (`pytest tests/ -q`, run 2026-05-20): **1429/1429 pass in 428.40 s** (1 xfail; ci-fast hypothesis profile).

## Discussion

Three hypothesis-withdraw moments worth recording, since the institutional
record around the MPC's jitter arc has accumulated several entries built on
prior framings that this investigation displaced.

1. **The architectural reframe ("z=30 is intrinsically ill-conditioned;
   MPC may be the wrong layer") was wrong.** The recorded chronic-failure
   session was, as far as the institutional record knew, evidence the MPC
   architecture was failing on a hard pose. The offline probe showed the
   same NLP at z=30 is trivial (9 ms / 2 iters / 100 % across every
   condition tested) when given a bare static target. Climbing one level
   up: the difficulty wasn't in the NLP, it was in the *pipeline feeding
   it*. The architectural concern, which would have led to a substantial
   rework if accepted, was built on evidence that hadn't been fully
   diagnosed. This entry is the disposal of that concern.

2. **The DVFS-clock-starvation hypothesis was also wrong for this failure
   mode.** It IS a real mechanism — visible in the frequency sampling, the
   sleep pacer lets `schedutil` dip the core to 0.88 GHz — but on this NLP
   the solves are fast enough that DVFS isn't on the critical path. The
   busy-wait probe (a non-privileged clock-hold proxy) showed 100 %
   success on the bare-target reproduction regardless of pacer. There was
   no failure to differentiate, so the A/B was uninformative for that
   hypothesis specifically. DVFS may still matter at the margin in
   contended deployments, but it is not the cause of the chronic phase.

3. **The validity gate fired correctly on the first offline probe.** The
   pre-registered rule was *"if the offline reproduction shows ~100 %
   success when the hardware was 60 % failure, the probe is not faithful
   — report it, do not use it for further claims."* Following that rule
   rather than rationalising the surprising result led directly to the
   production-faithful version that did reproduce. The first probe wasn't
   *invalid* — it was *too easy* (missing production ingredients). Saying
   so honestly, rather than tweaking thresholds to declare success,
   surfaced the actual mechanism.

The contract framing matters. K1–K6 introduced `warm_start_valid` as a
*caller-side* hint for structural reference shifts ("the new ref differs
enough that the old warm-start is unusable"). This patch extends the same
enforcement point in `solve()` with a *self-detected* trigger ("we've been
failing past walk-forward's threshold, the cached vectors must be
unusable"). The clearing list and logging style are identical. The
contract becomes:

> Warm-start is invalidated when either the reference shifts structurally
> OR sustained failure indicates the cached vectors no longer reflect a
> usable solution.

This is the K1–K6 / W5 generalisation rather than a separate patch on top.
That distinction — *generalisation of an existing contract* vs *a new
contract* — is why this fix is small (+34 lines, 0 deletions, no new
`MPCParams` field, no new test class) rather than the substantial
architectural change the original framing pointed at.

**Relationship to the sibling 2026-05-18 watchdog-gap entry.** The
2026-05-18 z=30 session that supplied the recorded states for this
investigation is *also* covered by
[`2026-05-18-z30-solve-failure-motor-pos-none-watchdog-gap.md`](2026-05-18-z30-solve-failure-motor-pos-none-watchdog-gap.md),
which closes Finding 3 from a different angle: the
`motor_pos = None` path after CAN drop evades both telemetry-staleness
watchdogs in `HardwarePlant`. The two mechanisms live at different layers
and are independent. The sibling entry addresses the **hardware-plant /
telemetry-watchdog layer** (what happens after CAN drops). This entry
addresses the **solver / warm-start layer** (the chronic-CTE deadlock
that materialises *before* any CAN issue and reproduces fully offline
with no CAN involvement). Both contributed to the recorded session's
failure pattern; resolving both is necessary for confidence in z=30
operation. Reading the offline-probe arc here together with the
hardware-plant arc there gives the full picture.

**Residuals to track in a separate investigation.** Hardware validation on
2026-05-20 showed the chronic phase eliminated, but the operator
simultaneously reported "small but visible/audible jitter events" during
the z=170 → z=30 settle. Two of those events coincide with the two
isolated singleton fallbacks in the recording: the walk-forward path emits
`_prev_w[6k:6(k+1)]` rate-limited to the prior command, which produces
1-tick `cmd_ext` discontinuities of 2–3.5 mm at the points where the
fallbacks fired. Whether the remaining jitter events have the same root
cause (singleton fallbacks producing momentary command-stream
discontinuities) or a separate mechanism (the 7 `Solved_To_Acceptable_Level`
ticks producing different-quality output, or the K1–K6 stretched-segment
transition, or the 50 ms session-start overhead spike, or motion-onset
stiction) is the next investigation. The prior jitter-arc entries
(`2026-04-17-mpc-fallback-cmd-sawtooth-stutter`,
`2026-04-18-hold-fighting-motion-onset-jitter`,
`2026-04-18-move5-overshoot-stall-and-plant-collapse`,
`2026-04-18-mpc-overhead-spikes-fallback-bursts`) should be re-read
through the warm-start-deadlock lens, since some fraction of those
symptoms may turn out to have been this bug rather than the separate
causes they were attributed to.

## Verification

- **Offline probe**, production-faithful replay of `temp/logs/mpc_20260518_200137.csv`
  (2979 ticks, recorded `run_mpc.py --pose 0,0,30,0,0,0 --duration 600`):
  - **Baseline** (pre-patch): overall 39.1 % success, late-phase 0.0 %,
    longest CTE run **1777 ticks** (matches the logbook's 1769).
  - **Patched**: overall 93.0 % success, late-phase 89.8 %, longest CTE
    run **21 ticks** (84× reduction).
- **Hardware** (`run_mpc.py --pose 0,0,30,0,0,0 --duration 60`, 2026-05-20
  11:58): 2400 ticks, 2 isolated singleton `fallback(Maximum_CpuTime_Exceeded)`
  (0.08 %), zero `hold_extrap`, final tracking error 0.041 mm. Patch log
  line never fired (`_consecutive_failures` did not exceed 3, so the
  escape was dormant — confirms non-regression on the healthy path).
- **Full test suite** (`pytest tests/ -q`, run 2026-05-20): **1429/1429 pass in 428.40 s** (1 xfail; ci-fast hypothesis profile, includes the new regression test). Pre-flight run on the same patch (without the new test) earlier the same day produced one transient failure of `tests/sim/test_mpc_adversarial_sequences.py::TestScenario13_WalkForwardOldDir::test_ref_mid_run_survives_cpu_pressure`, which the test's own docstring documents as load-flaky on a busy Jetson and instructs to re-verify in isolation; re-run in isolation passed (`pytest <path> -v` → 1 passed in 3.90 s). The pre-commit suite above ran with that test in the suite and it passed.

## Related

- [logbook/2026-05-18-hardware-bringup-t2b1-t2a1-cascade-validation.md](2026-05-18-hardware-bringup-t2b1-t2a1-cascade-validation.md)
  Finding 3 — the recorded z=30 session this entry resolves.
- [logbook/2026-04-18-mpc-overhead-spikes-fallback-bursts.md](2026-04-18-mpc-overhead-spikes-fallback-bursts.md)
  — earlier investigation of solve-time overrun, candidate root cause of
  some of the same symptoms (now superseded for the chronic-phase case
  but still relevant for the per-spike behaviour).
- [logbook/2026-04-20-k1-k6-reference-feasibility-resolution.md](2026-04-20-k1-k6-reference-feasibility-resolution.md)
  — the W5 warm-start-invalidation contract this patch extends.
- Offline probe artefacts (not committed): `/tmp/mpc_dvfs_probe/`
  — `RESULTS.md`, `FINDINGS_EXP1.md`, `FINDINGS_EXP1B.md`,
  `probe_mpc_faithful.py`, per-cell CSV/summary files.
