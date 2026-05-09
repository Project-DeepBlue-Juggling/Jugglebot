---
title: Scheduler contract — Phase 3 enforcement of S4, S5, S6 + RuleBasedStateMachine
type: feature
date: 2026-05-09
status: resolved
phase: "mpc-tier0-contracts — Phase 3"
related_plan: "mpc-tier0-contracts.md"
related_entries:
  - 2026-05-09-scheduler-contract-phase-2-s1-s3-enforcement
  - 2026-05-09-scheduler-contract-phase-1-audit
  - 2026-04-20-k1-k6-reference-feasibility-resolution
  - 2026-04-23-hot-loop-zero-allocation-contract
files_changed:
  - controller/scheduler.py
  - controller/SCHEDULER_CONTRACT.md
  - tests/sim/test_scheduler_contract.py
  - plans/active/mpc-tier0-contracts.md
  - logbook/2026-05-09-scheduler-contract-phase-3-s4-s6-enforcement.md
  - logbook/INDEX.md
commits:
  - <filled-after-commit>
subsystem:
  - controller
  - mpc
tags:
  - contract
  - testing
  - safety
---

# Scheduler contract — Phase 3 enforcement of S4, S5, S6 + RuleBasedStateMachine

## Summary

Phase 3 of the [mpc-tier0-contracts plan](../plans/active/mpc-tier0-contracts.md):
implemented the three state-machine invariants S4 (internal quintic
feasibility), S5 (phase-transition C2 continuity), and S6 (sim_time
monotonicity) defined in
[controller/SCHEDULER_CONTRACT.md](../controller/SCHEDULER_CONTRACT.md).
Added a hypothesis ``RuleBasedStateMachine`` that exercises arbitrary
submit / cancel / update / replace / tick sequences and asserts S2 / S3
/ S5 / S6 invariants throughout.  Behaviour change: silent-skip of K2/K3
verification is now diagnosable at construction; segments built mid-
TRANSITIONING are structurally C2-continuous; backward ``sim_time`` at
``update()`` raises a loud ``ValueError`` with a pointer to the
documented ``clear()`` reset workflow.

## Motivation

[Phase 1](2026-05-09-scheduler-contract-phase-1-audit.md) drafted the
S1–S6 contract and surfaced the gaps to close.
[Phase 2](2026-05-09-scheduler-contract-phase-2-s1-s3-enforcement.md)
landed the input-domain enforcement (S1 / S2 / S3) at ``submit_event`` /
``replace_next_event``.  Phase 3 closes the *state-machine* subset:

- **S4** was already partially implemented (the ``strict_feasibility``
  toggle existed) but conditionally skipped when ``v_max_mmps`` or
  ``tau_s`` was unset, with no operator-visible diagnostic.
- **S5** was a side-effect of the segment-build pattern, not a stated
  invariant — a future contributor refactoring
  ``_update_approaching`` / ``_update_transitioning`` could silently
  break C0 continuity at the splice.
- **S6** was unenforced — the ``_last_sim_time`` bookkeeping landed in
  Phase 2 (for the S1 clock-skew check) was *tracked* but not *checked*
  for backward jumps.

Phase 3 also lands the property-based testing surface that Phase 2's
Discussion section flagged as a prerequisite: a ``RuleBasedStateMachine``
that walks arbitrary scheduler operations and asserts S1–S6 invariants.
Without Phase 2's enforcement, the random walks would generate
S1/S2/S3-violating states that the property assertions would falsely
interpret as scheduler bugs; with Phase 2 landed, the state machine can
trust the input-domain invariants and focus on the state-machine ones.

## Design

### S4 — surface the silent-skip path

The pre-Phase-3 ``_verify_segment_feasibility`` returned without action
when either ``v_max_mmps`` or ``tau_s`` was None.  Phase 3 adds a single
``WARNING`` at the constructor (``S4 unenforced: ...``) so the degraded
mode is visible in any session log without grepping the constructor
call.  The per-segment check stays silent on this path — the
construction warning has already covered the diagnostic and per-segment
spam would drown it.

The ``strict_feasibility`` constructor parameter stays opt-in (default
``False``, per user choice during Phase 3 design).  Production callers
keep the existing graceful-degradation behaviour; tests opt in
explicitly via ``strict_feasibility=True``.  The auto-detect path
(flipping default based on ``PYTEST_CURRENT_TEST``) was rejected as
magic — surprising for downstream test fixtures that explicitly
construct with ``v_max=None`` and don't expect their first segment-
build to raise.

### S5 — refactor the build helper to read ``_last_*``

The pre-Phase-3 ``_build_segment_to_event`` took ``start_pose`` /
``start_twist`` arguments and was called with two distinct conventions:

- ``_update_approaching`` passed plant pose / twist (live state).
- ``_update_transitioning`` passed ``self._last_pose`` /
  ``self._last_twist`` (segment-evaluated state).

Phase 3 refactors the helper to take only ``(sim_time, event)`` and
read from ``self._last_*`` directly.  This makes S5 enforced *by
construction*: every newly-built segment is structurally guaranteed to
start at the live motion state.

The IDLE→APPROACHING bootstrap path needed a small accommodation: at
``submit_event`` time, ``_last_*`` may be one tick stale (set by the
previous IDLE update) or zero (constructor defaults, before any update).
A new ``_approach_bootstrap_pending`` flag is set in ``submit_event``
when transitioning to APPROACHING, then consumed at the entry of
``_update_approaching`` to sync ``_last_*`` to plant state once before
the first build.  After the first build, ``_last_*`` is owned by
segment evaluation again.

The post-refinement path (``update_current_event`` mid-APPROACHING)
leaves the flag False, preserving segment-evaluated ``_last_*`` for
C2 continuity at the splice.

### S6 — backward-time guard at ``update()`` entry

At the top of ``update()``, after the existing finiteness check (landed
in Phase 2), Phase 3 adds a monotonicity check: if ``_last_sim_time``
is set and the new ``sim_time`` is below it by more than 1e-9 s, raise
``ValueError`` with a diagnostic message naming the operation, the
observed values, and the documented ``clear()`` reset workflow.  The
1e-9 s tolerance absorbs floating-point round-off in sim_time
arithmetic.

``clear()`` resets ``_last_sim_time = None`` (and the new
``_approach_bootstrap_pending`` flag) so the next ``update()`` after
a sim restart accepts any timestamp.

### RuleBasedStateMachine — generation strategy

Per user choice during Phase 3 design, the state machine's ``tick``
rule advances ``sim_time`` strictly forward by a positive ``dt``.  S6
monotonicity is enforced by construction during the random walk; the
backward-jump scenarios live in
[TestS6ClockMonotonicity](../tests/sim/test_scheduler_contract.py)
above the state machine, outside the generation surface.

The state machine asserts S2 (unique IDs), S3 (slot bound), S5 (segment
``t_start`` ≤ observed ``_last_sim_time`` — the residual evidence that
survives state advancement), and S6 (``_last_sim_time`` monotone non-
decreasing) as ``@invariant``s.

Two state-machine semantic gaps surfaced during the property walk —
``cancel_next`` during TRANSITIONING leaves the scheduler in an
inconsistent state, and ``begin_return`` during APPROACHING with both
slots filled silently overwrites ``_next_event``.  Both are
*orthogonal* to S4/S5/S6 (they're state-machine completeness gaps,
not contract invariants) and have been gated with ``@precondition``s
in the state machine and filed as Plan 2 follow-ups (``mpc-sadpath-
coverage-tiers-1-3.md``, Tier 1 state-machine completeness).

## Implementation

### controller/scheduler.py (modified)

- Constructor: added the ``S4 unenforced`` warning when either
  ``v_max_mmps`` or ``tau_s`` is None.  Added the
  ``_approach_bootstrap_pending`` instance flag (initialised False).
- ``submit_event``: when the new event becomes ``_current_event`` (and
  the phase transitions to APPROACHING), set
  ``_approach_bootstrap_pending = True`` so the next
  ``_update_approaching`` syncs ``_last_*`` to plant state once before
  the first segment build.
- ``clear``: also resets ``_last_sim_time = None`` and
  ``_approach_bootstrap_pending = False`` — the documented sim-restart
  reset path per S6.
- ``update``: added the S6 monotonicity check (1e-9 s tolerance) after
  the existing finiteness guard, before storing ``_last_sim_time``.
  The check runs only when ``_last_sim_time`` is non-None (first
  update accepts any timestamp).
- ``_update_approaching``: at the top of the segment-build branch,
  consume ``_approach_bootstrap_pending`` to sync ``_last_pose`` /
  ``_last_twist`` from the passed-in plant state.  ``_last_accel``
  stays at the zero baseline set by IDLE / the constructor (bootstrap
  starts from rest).  Subsequent calls trust ``_last_*`` as set by the
  previous tick's segment evaluation.
- ``_update_transitioning``: simplified the build call — the helper
  reads ``_last_*`` directly, no need to thread plant args.
- ``_build_segment_to_event``: refactored to take only
  ``(sim_time, event)``.  Reads from ``self._last_pose`` /
  ``_last_twist`` / ``_last_accel`` directly, making S5 enforced by
  construction.  Updated docstring lists the four splice paths the
  invariant covers (IDLE→APPROACHING bootstrap, mid-APPROACHING
  refinement, HOLDING→TRANSITIONING, mid-TRANSITIONING replace).

### controller/SCHEDULER_CONTRACT.md (modified)

Refreshed every ``scheduler.py:NNN`` line citation in the document to
point at the post-Phase-3 line numbers (the pre-Phase-3 numbers had
shifted by ~70 lines after Phase 2's input-domain helpers landed).
The Phase 1 audit lessons-learned identified line citations as a
common drift surface; Phase 2 also bumped them; this is the third
refresh in three phases — each one was caught during audit and
fixed before commit.

Added explicit pointers in the Background section to the new Phase 3
enforcement sites:
- S4 construction warning at [scheduler.py:225–235](../controller/scheduler.py)
- S5 segment-build enforcement at [scheduler.py:852–893](../controller/scheduler.py)
- S6 update() entry guard at [scheduler.py:459–484](../controller/scheduler.py)

### tests/sim/test_scheduler_contract.py (modified)

Phase 3 added 19 contract scenarios + 1 hypothesis state machine,
organised into four new classes:

| Class | Scope | Tests |
|-------|-------|-------|
| ``TestS4FeasibilityConstruction`` | Constructor S4-unenforced warning | 4 (v_max-missing, tau-missing, both-missing, both-provided silent) |
| ``TestS4FeasibilityEnforcement`` | Per-segment K2/K3 check | 3 (strict raises, lax warns, no-params silent) |
| ``TestS5SegmentLiveState`` | Splice continuity at every build | 6 (bootstrap, refinement, HOLDING→TRANSITIONING, replace mid-TRANSITIONING C2, replace mid-APPROACHING defers, RETURNING) |
| ``TestS6ClockMonotonicity`` | Backward-time enforcement | 6 (backward raises, equal/forward accepted, clear resets, message diagnostic, epsilon tolerance, failed update doesn't advance) |
| ``SchedulerStateMachine`` | Hypothesis property walk | 1 (TestCase exercises arbitrary submit/cancel/update/replace/tick sequences with S2/S3/S5/S6 invariants) |

The S5 splice-continuity tests assert C2 continuity (pose + twist +
accel match ``_last_*`` at the moment of build) per user choice during
Phase 3 design.  C2 is the natural scope given the contract explicitly
lists all three lines (``seg.p0 = _last_pose``, ``seg.v0 = _last_twist``,
``seg.a0 = _last_accel``) — testing only C0 would have left two of
those lines unverified.

A new ``scheduler_log`` fixture attaches a ``logging.Handler`` directly
to the ``controller.scheduler`` logger, replacing pytest's ``caplog``
which doesn't reliably capture from this project's logger configuration
(pytest's live-log goes to stderr via ``logging.lastResort``, bypassing
the propagation chain ``caplog`` hooks).  The direct attach removes
that ambiguity.

### plans/active/mpc-tier0-contracts.md (modified)

Phase 3 marked ``COMPLETE (2026-05-09)`` in both the summary table
and the detailed Phase 3 heading.

## Verification

### Existing tests — no behaviour-change impact (pre-audit prediction)

Pre-implementation audit of the three files that exercise the
scheduler:

1. [tests/sim/test_scheduler.py](../tests/sim/test_scheduler.py) — 24
   tests, all submit-then-update with monotonic time and a single plant
   pose throughout.  Predicted: zero S5/S6 collateral.
2. [tests/sim/test_scheduled_catch.py](../tests/sim/test_scheduled_catch.py)
   — sim catch-coordinator integration.  Predicted: ``scheduler.update``
   is called from inside ``ScheduledCoordinator.update`` with sim_time
   from the sim main loop (monotonic by construction); S6 cannot fire.
3. [sim/hand/scheduled_coordinator.py](../sim/hand/scheduled_coordinator.py)
   — production-shaped caller.  ``submit_event`` always uses fresh
   ``_next_event_id()``; ``update_current_event`` is the legitimate
   refinement path (out of S4/S5/S6 scope).

The audit also confirmed that **no** production caller currently
constructs ``EventScheduler`` directly with ``v_max_mmps``/``tau_s``
unset — the construction warning fires only in tests that don't pass
the parameters (already a documented test-only mode).

Confirmed empirically: ``pytest tests/sim/test_scheduler.py
tests/sim/test_scheduler_contract.py tests/sim/test_scheduled_catch.py
-q`` → **69 / 69 pass** with no test changes.

### Test results

- **Before Phase 3:** 1131 / 1131 pass (Phase 2 baseline).
- **After Phase 3:** **1151 / 1151 pass** — the +20 delta is exactly
  the 19 new contract scenarios + 1 ``SchedulerStateMachine`` TestCase
  (which counts as one test by ``RuleBasedStateMachine`` convention).
  Zero regressions.

(One transient flake of ``test_ref_mid_run_survives_cpu_pressure`` was
observed during a CPU-pressured full-suite run; the test passes
deterministically in isolation and on a re-run.  Not Phase-3-related.)

## Discussion

### Why S5 is enforced by construction rather than by assertion

Two designs were considered for S5 enforcement:

1. **Assert at the build site.** Keep the existing
   ``_build_segment_to_event(sim_time, start_pose, start_twist, event)``
   API; assert ``start_pose == self._last_pose`` and ``start_twist ==
   self._last_twist`` after construction.  Catches the violation but
   relies on every caller passing the right values.
2. **Make ``_last_*`` the only source.** Refactor the helper to take
   only ``(sim_time, event)`` and read ``_last_*`` directly.  Callers
   can't get it wrong by construction.  Chosen.

Option 2 won because the contract is *about* the relationship between
the segment and ``_last_*`` — making the helper read from there
directly is the contract.  Option 1 would let a future caller pass
plant pose / twist (the pre-Phase-3 ``_update_approaching`` convention)
without raising — the assertion would catch ``start_pose ≠ _last_pose``,
but the caller's intent ("trust the plant, not the cache") would be
the *real* bug, and an assertion saying "you broke the cache invariant"
wouldn't surface it.

The bootstrap-flag accommodation (``_approach_bootstrap_pending``) is
the structural alternative to "trust plant on the first build" — the
flag scopes the plant-trust to one specific transition rather than
making it the helper's default.  Future contributors who add a new
phase that needs plant-trust will need to add their own bootstrap
flag rather than rediscovering the per-call argument convention.

### Why backward sim_time is a hard raise rather than a clamp

Three options were considered for S6:

1. **Clamp** ``sim_time = max(sim_time, self._last_sim_time)``.  Lets
   downstream code keep computing.  Hides the bug — the segment's
   ``arrival = event.time - sim_time`` becomes physically inconsistent
   with the (now-clamped) sim_time, and the caller never learns their
   clock is wrong.
2. **Warn**.  Logs the violation but proceeds.  Better than clamp but
   still produces undefined arrival detection downstream.
3. **Raise**.  Forces the caller to either fix the timestamp or use
   ``clear()``.  Chosen.

Option 3 is consistent with how this project handles every other
caller-side input-shape error in the contract surface (S1 raises on
past-time events, S2 raises on duplicate IDs, S3 raises on slot
saturation).  A scheduler that silently accepted backward time would
produce confusing downstream symptoms (negative-duration quintics
clamped to 50 ms, spurious arrival firings, hand-notification
re-fires) days after the actual caller bug.

### Why C2 (not C0) for the S5 splice-continuity tests

The contract states:

    seg.p0 = self._last_pose
    seg.v0 = self._last_twist
    seg.a0 = self._last_accel

— three lines, all required.  Testing only C0 (``seg.p0 ==
_last_pose``) would have verified just one of the three, leaving the
twist and accel splice properties unenforced.

The plan's T-U-S5-1 row mentioned C0 only (``‖p(t_splice−) −
p(t_splice+)‖ < 1e-9``), but the contract scope is broader.  At Phase 3
design time we picked C2 — the natural scope given the contract
explicitly lists all three orders.  C0 would have been a defensive
position (the weakest invariant the contract permits); C2 verifies
what the contract actually says.

### What the state machine surfaced — and why we deferred it

Two state-machine semantic gaps surfaced during the property walk:

1. ``cancel_next`` while ``_phase == TRANSITIONING`` clears
   ``_next_event`` without resetting the phase, leaving the scheduler
   in TRANSITIONING with no event to transition to.  The next
   ``update()`` hits ``assert next_ev is not None`` in
   ``_update_transitioning``.
2. ``begin_return`` while both slots are filled silently overwrites
   ``_next_event`` (the ``begin_return`` body assigns
   ``self._next_event = ret_event`` without an S3 check).

Both are *real* state-machine semantic gaps, both are *orthogonal* to
S4/S5/S6 (they're about phase-transition completeness, not feasibility
or continuity or monotonicity).  Per user choice during Phase 3 design,
audit findings outside the S4/S5/S6 contract surface land in Plan 2
(the mpc-sadpath-coverage rollup, Tier 1 state-machine completeness)
rather than expanding Phase 3 scope.

For Phase 3, both paths are gated with ``@precondition``s in the state
machine — the property walk skips combinations that hit either path.
The targeted scenario tests for legitimate state-machine usage still
exercise the gated rules (e.g.,
``test_replace_next_mid_transitioning_splices_c2_continuous`` exercises
the TRANSITIONING phase through the replace API, just not the
cancel API).

The Plan 2 follow-ups will need to:
- Define what ``cancel_next`` should do during TRANSITIONING (revert
  to RETURNING is the most natural choice; reset to HOLDING is the
  alternative).
- Either teach ``begin_return`` to honour S3 (raise when both slots
  full) or document the overwrite as the documented behaviour and
  carve an S3 exemption.

### What landed without enforcement

S5's *evidence* invariant in the state machine (segment ``t_start`` ≤
observed ``_last_sim_time``) is weaker than the contract claim
(segment starts at ``_last_*``).  The full claim is impossible to
verify after the fact because ``_last_*`` advances via segment
evaluation between the build and the invariant check.  The targeted
scenario tests in
[TestS5SegmentLiveState](../tests/sim/test_scheduler_contract.py)
fill the gap by checking ``seg.p0 / v0 / a0`` against the
pre-build snapshot of ``_last_*`` directly — that's where the C2
claim is actually verified.  The state machine's ``@invariant``
catches the residual structural property (segment can't have been
built in the future).

## Open Questions

- **C2 scope when ``_last_accel`` is forced to zero.** The bootstrap
  path syncs ``_last_pose`` / ``_last_twist`` from plant but keeps
  ``_last_accel`` at zero (the IDLE / constructor baseline).  This is
  correct (we're starting from rest), but a future contributor adding
  a "warm start" path (e.g., resuming a paused approach) will need to
  thread ``_last_accel`` through the bootstrap as well.  The
  ``_approach_bootstrap_pending`` flag's contract is currently "sync
  pose+twist; accel stays at IDLE baseline" — make that explicit if
  the warm-start path lands.
- **State-machine gaps as Plan 2 work.** The two findings (cancel-mid-
  TRANSITIONING, begin-return-overwrite) are real bugs.  They're
  filed as Plan 2 follow-ups but the precondition gates in the state
  machine are a temporary suppression of the property test.  If Plan 2
  takes longer than expected, the gates risk drifting from "Plan 2
  follow-up" to "tribal knowledge" — re-audit at the Plan 2 kickoff
  to ensure they're still accurate.
- **Hypothesis profile selection.** The state machine runs at
  ``max_examples=50`` (the suite-default) which is coarse.  Phase 8 of
  this plan introduces ``ci-fast`` / ``ci-deep`` profiles; once those
  land, this state machine should run at ``ci-deep`` (1000 examples)
  in nightly to surface anything the 50-example walk missed.
