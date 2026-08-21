---
title: Scheduler — cancel_next during TRANSITIONING raises (Plan 2 Phase 0, bug 1 of 2)
type: bugfix
date: 2026-05-10
status: resolved
phase: "mpc-sadpath-coverage-tiers-1-3 — Phase 0"
related_plan: "mpc-sadpath-coverage-tiers-1-3.md"
related_entries:
  - 2026-05-09-scheduler-contract-phase-3-s4-s6-enforcement
  - 2026-05-09-scheduler-contract-phase-2-s1-s3-enforcement
  - 2026-05-09-scheduler-contract-phase-1-audit
files_changed:
  - controller/scheduler.py
  - controller/SCHEDULER_CONTRACT.md
  - tests/sim/test_scheduler_contract.py
  - plans/archived/mpc-sadpath-coverage-tiers-1-3.md
  - logbook/2026-05-10-scheduler-cancel-next-during-transitioning.md
  - logbook/INDEX.md
commits:
  - 167b9f8
subsystem:
  - controller
  - mpc
tags:
  - bugfix
  - testing
  - safety
  - state-machine
---

# Scheduler — cancel_next during TRANSITIONING raises (Plan 2 Phase 0, bug 1 of 2)

## Summary

First of two bugs closed under Plan 2 Phase 0 (the "Plan 1 deferred
``@precondition`` cleanup" carve-out).  Pre-fix, ``EventScheduler.cancel_next``
unconditionally cleared ``_next_event`` and ``_seg_next`` without
inspecting ``_phase``.  When called during ``TRANSITIONING`` —
the only phase where ``_next_event`` is the active *destination* of
an in-flight transition rather than a lookahead — the scheduler was
left in ``TRANSITIONING`` with no destination event, and the next
``update()`` asserted in ``_update_transitioning``
([scheduler.py:651](../controller/scheduler.py)).

The fix raises ``ValueError`` at ``cancel_next`` entry when
``_phase == TRANSITIONING``, consistent with the S2/S3 raise discipline
at ``submit_event``.  Callers who legitimately want to abort an
in-flight transition use ``clear()`` — the documented "tear it all
down" reset.  The ``@precondition`` gate at the
``SchedulerStateMachine.cancel_next`` rule
([test_scheduler_contract.py:999–1001](../tests/sim/test_scheduler_contract.py))
that suppressed this random-walk path is removed in the same commit;
the rule now exercises ``cancel_next`` in every phase and asserts the
new raise discipline.

## Motivation

[Plan 1 Phase 3](2026-05-09-scheduler-contract-phase-3-s4-s6-enforcement.md)
landed the S1–S6 contract enforcement and the ``RuleBasedStateMachine``
random walk.  During Phase 3 the random walk surfaced two state-machine
semantic gaps orthogonal to S4/S5/S6 — ``cancel_next`` mid-TRANSITIONING
(this bug) and ``begin_return`` overwrite (the next entry).  Both were
gated with ``@precondition``s and filed as Plan 2 follow-ups so the
contract enforcement work could ship.

[Plan 2](../plans/archived/mpc-sadpath-coverage-tiers-1-3.md) Phase 0
closes those gates as the documented exception to its "test additions
only" discipline — removing a ``@precondition`` requires fixing the
underlying bug in the same commit, otherwise the random walk would
immediately re-surface the broken state.

## Design

### Why raise rather than atomically transition

Two design alternatives were considered:

1. **Raise ``ValueError``.**  Treat ``cancel_next`` as an
   "operate on the lookahead slot" API and raise when the slot's
   semantics shift to "the active destination" (i.e., during
   TRANSITIONING).  Caller must explicitly choose ``clear()`` to abort
   the transition.

2. **Atomic phase transition.**  Have ``cancel_next`` flip the phase
   to ``HOLDING`` (with the old ``_current_event`` retained) or to
   ``RETURNING`` (return-to-active path).

Option 2 (atomic-to-HOLDING) is structurally unsafe.  ``_update_holding``
at [scheduler.py:629](../controller/scheduler.py) overwrites
``_last_pose = event.pose.copy()`` and ``_last_twist`` from the held
event's pose — but during TRANSITIONING the live ``_last_*`` is being
driven by the segment's mid-flight evaluated state, not the old current
event's pose.  Reverting to HOLDING would snap the reference back to
the old event's pose at the next tick, producing a position
discontinuity at the live mid-segment pose — a K1 (live-anchor)
violation on the first emitted event and a S5 (C0 continuity) violation
at the splice.  Atomic-to-RETURNING preserves C0 (the return segment is
built from ``_last_pose`` per
[scheduler.py:714–723](../controller/scheduler.py)) but conflates two
operations: the caller said "cancel my lookahead", not "begin return
to active".

Option 1 won.  The contract is "``cancel_next`` operates on a
lookahead slot; the slot's role during TRANSITIONING is structurally
different (it's the destination), so the API must refuse rather than
silently expand its semantics."  The caller who wants to abort an
in-flight transition has ``clear()`` already — a single, documented
reset path.

### Upstream impact

``cancel_next`` has zero production callers.  The ``sim/hand/scheduled_coordinator.py``
catch coordinator does not call it.  Test-side callers
([test_scheduler.py:199–215](../tests/sim/test_scheduler.py),
[test_scheduler_contract.py:375–382](../tests/sim/test_scheduler_contract.py))
exercise the API in APPROACHING / HOLDING phases only, where the new
guard is not triggered.  The ``SchedulerStateMachine.cancel_next``
rule was the *only* path that hit the bug, and it was already gated.

### Downstream impact

Inside ``cancel_next``, the early ``ValueError`` raise leaves
``_next_event`` and ``_seg_next`` intact, so subsequent ``update()``
calls continue the transition normally.  The hot-loop allocation
contract is unaffected — ``cancel_next`` is not on the hot loop.

## Implementation

### controller/scheduler.py (modified)

``cancel_next`` ([scheduler.py:405–428](../controller/scheduler.py))
gains a one-line phase guard at the top of the method:

```python
if self._phase == SchedulerPhase.TRANSITIONING:
    raise ValueError(
        "cancel_next during TRANSITIONING is invalid: "
        "_next_event is the active destination of the in-flight "
        "transition, not a lookahead.  Cancelling it would leave "
        "the scheduler in TRANSITIONING with no destination "
        "event, which subsequent update() calls would assert "
        "on.  Use clear() to abort the transition."
    )
```

The docstring is expanded to describe the new restriction and to
direct callers at ``clear()`` for the legitimate transition-abort
path.

### tests/sim/test_scheduler_contract.py (modified)

Added a new test class ``TestCancelNextStateMachine`` with four
scenarios:

| Test ID | Scenario | Asserts |
|---------|----------|---------|
| T-U-T0-1 | ``cancel_next`` during ``TRANSITIONING`` | Raises ``ValueError``; phase / next_event / _seg_next all unchanged after the raise; subsequent ``update()`` continues the transition normally |
| T-U-T0-2 (regression) | ``cancel_next`` during ``APPROACHING`` | Returns the cancelled event; ``next_event`` becomes ``None``; phase stays ``APPROACHING``; tick past arrival lands in ``HOLDING`` |
| (extra) | ``cancel_next`` during ``HOLDING`` (with queued lookahead) | Returns the cancelled event; ``next_event`` becomes ``None``; phase stays ``HOLDING`` |
| (extra) | ``cancel_next`` during ``IDLE`` | Returns ``None``; phase stays ``IDLE`` |

Removed the ``@precondition`` gate at the
[SchedulerStateMachine.cancel_next](../tests/sim/test_scheduler_contract.py)
rule.  The rule now wraps ``self.scheduler.cancel_next()`` in a
``try / except ValueError`` and asserts the raise message includes
``"cancel_next during TRANSITIONING"`` AND that the scheduler's phase
was ``TRANSITIONING`` at the time — so the random walk catches both
"didn't raise when it should have" and "raised in the wrong phase".

## Verification

### Pre-fix regression test

The new ``test_cancel_next_during_transitioning_raises`` was confirmed
to fail with a ``Failed: DID NOT RAISE <class 'ValueError'>`` error
when the production fix was stashed (``git stash push
controller/scheduler.py``) and re-run in isolation.  Re-applying the
fix made the test pass deterministically.

### Test results

- **Before this fix:** 1184 passing, ``SchedulerStateMachine``
  precondition-gated.
- **After this fix:** 1188 passing (+4 new ``TestCancelNextStateMachine``
  scenarios), ``SchedulerStateMachine.cancel_next`` rule un-gated.

ci-fast (``pytest tests/ -q``): pass (1187/1188; one transient flake on
``test_motor_guard.py::test_decay_boundary_continuity`` — a pre-existing
heap-state contamination flake noted in
[Plan 2 Working Note #5](../plans/archived/mpc-sadpath-coverage-tiers-1-3.md);
passes deterministically in module-level isolation).

ci-deep on the contract test file (``pytest
tests/sim/test_scheduler_contract.py --hypothesis-profile=ci-deep
--hypothesis-seed=0``): **64 / 64 pass in 105.20 s**.  The un-gated
``SchedulerStateMachine`` random walk at 1000 examples surfaces no
new ``cancel_next``-related state regressions.

## Discussion

### Why split this fix from begin_return

Plan 2 Phase 0 originally framed both bugs as a single "state-machine
completeness" pair.  Two reasons to split:

1. **Rollback granularity.**  Plan 1 (whose discipline Plan 2
   inherits) lands one logbook entry per concrete behaviour change so
   that a problematic change can be reverted independently.  These two
   bugs sit on opposite sides of the API surface — ``cancel_next`` is a
   pure removal API; ``begin_return`` is an event-construction API —
   and a rollback of one shouldn't be coupled to the other.
2. **Discussion-section coherence.**  Each bug has its own design-
   alternative analysis (raise vs atomic-transition for ``cancel_next``;
   raise vs new-error-class for ``begin_return``).  Bundling them into
   one entry would force two parallel decision trees into a single
   narrative, weakening the institutional memory.

### Why the state machine's ``cancel_next`` rule wraps in try/except rather than restricting the rule's preconditions

The pre-fix gate was a ``@precondition`` filtering the phase to
non-TRANSITIONING.  The post-fix rule could either:

1. **Drop the precondition entirely** and trust the scheduler to raise
   in the right phase.  Random walks expose any phase-handling
   regression.
2. **Keep a precondition** that lets the rule fire in *any* phase but
   always succeeds.  Doesn't catch the regression case
   ("``cancel_next`` raised but phase wasn't TRANSITIONING").

Option 1 won, with the additional refinement that the rule wraps the
call in ``try / except ValueError`` and asserts (a) the message
contains the new diagnostic substring, and (b) the scheduler was in
TRANSITIONING when the raise fired.  This guards against two future
regressions:

- A refactor that broadens the raise to phases where it shouldn't
  fire (e.g., raising during HOLDING) → assertion (b) catches it.
- A refactor that changes the diagnostic message in a way that would
  break ``logger.warning`` grep lines or session-log diagnosis →
  assertion (a) catches it.

The plain "drop precondition entirely" form would have caught only
the first regression class.

### What ``clear()`` versus ``cancel_next`` semantically means now

Post-fix, the API surface has a clean three-tier semantic split:

- **``cancel_next``** — clear the lookahead slot.  Preconditions:
  phase ∈ {IDLE, APPROACHING, HOLDING, RETURNING}.  Returns the
  cancelled event (or ``None`` if the slot was empty).  Phase
  unchanged.  Cheap.
- **``clear``** — full reset to IDLE at active pose.  No
  preconditions; always succeeds.  Wipes ``_current_event``,
  ``_next_event``, all segments, the bootstrap pending flag, and
  ``_last_sim_time``.  Documented S6 sim-restart workflow.
- **``begin_return``** — initiate return-to-active.  Phase-dependent
  semantics today (HOLDING → RETURNING; otherwise queue
  ``RETURN_TO_ACTIVE`` as next event).  Bug 2 of Plan 2 Phase 0
  fixes the slot-overflow path.

Pre-fix, ``cancel_next`` had a hidden fourth role ("silently
half-abort an in-flight transition") that didn't belong to it.
Post-fix the three roles are strictly disjoint and the contract is
stateable in one sentence each.

## Open Questions

- **Should ``clear()`` log when called from non-IDLE?**  The post-fix
  diagnostic message points at ``clear()`` as the "abort the
  transition" path.  ``clear()`` itself is silent today
  ([scheduler.py:430–447](../controller/scheduler.py)).  A future
  audit pass might want a single ``logger.info`` so an in-flight
  abort is visible in session logs without requiring a custom
  hand-written log call from the caller.  Out of scope for Phase 0.
- **State-machine rule docs.**  The
  ``SchedulerStateMachine.cancel_next`` rule's pre-fix comment block
  ("filed as a Plan 2 follow-up") was removed.  Nothing was added in
  its place beyond a one-line pointer at ``TestCancelNextStateMachine``.
  If a future state-machine semantic gap is identified and gated, the
  comment-block style should be re-introduced as the canonical "why
  this is gated" pattern.

## Related

- [logbook/2026-05-09-scheduler-contract-phase-3-s4-s6-enforcement.md](2026-05-09-scheduler-contract-phase-3-s4-s6-enforcement.md)
  — Phase 3 random-walk surfaced this bug.
- [controller/SCHEDULER_CONTRACT.md](../controller/SCHEDULER_CONTRACT.md)
  — S1–S6 normative spec.  This bug sits at the boundary of S3 and the
  state-machine semantics.
- [plans/archived/mpc-sadpath-coverage-tiers-1-3.md](../plans/archived/mpc-sadpath-coverage-tiers-1-3.md)
  — Plan 2 Phase 0 specification.
