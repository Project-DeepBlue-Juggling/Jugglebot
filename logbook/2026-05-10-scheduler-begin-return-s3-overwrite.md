---
title: Scheduler — begin_return raises S3 instead of overwriting _next_event (Plan 2 Phase 0, bug 2 of 2)
type: bugfix
date: 2026-05-10
status: resolved
phase: "mpc-sadpath-coverage-tiers-1-3 — Phase 0"
related_plan: "mpc-sadpath-coverage-tiers-1-3.md"
related_entries:
  - 2026-05-10-scheduler-cancel-next-during-transitioning
  - 2026-05-09-scheduler-contract-phase-3-s4-s6-enforcement
  - 2026-05-09-scheduler-contract-phase-2-s1-s3-enforcement
  - 2026-05-09-scheduler-contract-phase-1-audit
files_changed:
  - controller/scheduler.py
  - controller/SCHEDULER_CONTRACT.md
  - tests/sim/test_scheduler_contract.py
  - logbook/2026-05-10-scheduler-begin-return-s3-overwrite.md
  - logbook/INDEX.md
commits:
  - 239a35d
subsystem:
  - controller
  - mpc
tags:
  - bugfix
  - testing
  - safety
  - state-machine
  - contract
---

# Scheduler — begin_return raises S3 instead of overwriting _next_event (Plan 2 Phase 0, bug 2 of 2)

## Summary

Second of two bugs closed under Plan 2 Phase 0 (the "Plan 1 deferred
``@precondition`` cleanup" carve-out — see
[bug 1 of 2](2026-05-10-scheduler-cancel-next-during-transitioning.md)).
Pre-fix, ``EventScheduler.begin_return``'s third branch (the "queue
``RETURN_TO_ACTIVE`` as next event when not in HOLDING/IDLE" path)
silently assigned ``self._next_event = ret_event`` without inspecting
the slot's prior occupancy.  When called with both ``_current_event``
and ``_next_event`` already filled, the in-flight ``_next_event`` was
clobbered — bypassing S3's bounded-slot-set invariant and discarding
the legitimate caller-submitted lookahead (or the active TRANSITIONING
destination).

The fix routes the third branch through the existing
``_validate_slot_capacity(op_name='begin_return')`` helper — the same
helper ``submit_event`` uses for S3 enforcement
([scheduler.py:877–889](../controller/scheduler.py)) — raising
``ValueError`` with the canonical "S3 violation in begin_return: …"
message when both slots are occupied.  ``begin_return`` is now
documented as not a S3 bypass; callers must ``cancel_next()`` /
``clear()`` before requesting return when the lookahead slot holds
something they're not willing to lose.  The ``@precondition`` gate at
the ``SchedulerStateMachine.begin_return`` rule
([test_scheduler_contract.py:1236–1249](../tests/sim/test_scheduler_contract.py))
that suppressed this random-walk path is removed in the same commit;
the rule now exercises ``begin_return`` in every phase / slot-state
combination and asserts the S3 raise discipline.

## Motivation

The same Plan 1 Phase 3
[RuleBasedStateMachine](2026-05-09-scheduler-contract-phase-3-s4-s6-enforcement.md)
that surfaced bug 1 also surfaced this one.  Phase 3's
[Discussion](2026-05-09-scheduler-contract-phase-3-s4-s6-enforcement.md#what-the-state-machine-surfaced--and-why-we-deferred-it)
flagged it explicitly: "``begin_return`` while both slots are filled
silently overwrites ``_next_event`` (the ``begin_return`` body assigns
``self._next_event = ret_event`` without an S3 check)."

Plan 1 deferred both bugs to Plan 2 so contract enforcement could
ship.  [Plan 2 Phase 0](../plans/active/mpc-sadpath-coverage-tiers-1-3.md)
closes them as the documented exception to its "test additions only"
discipline — removing a ``@precondition`` requires fixing the
underlying bug in the same commit, otherwise the random walk would
immediately re-surface the broken state.

## Design

### Why raise per S3, not introduce a new error class

The chosen fix raises ``ValueError`` via the existing
``_validate_slot_capacity`` helper, producing a message of the form
"S3 violation in begin_return: both _current_event (id=…) and
_next_event (id=…) are occupied.  Call cancel_next() or clear()
before queuing more events."

A second design alternative was considered: introduce a dedicated
``SchedulerSlotSaturatedError`` (subclass of ``ValueError``) so the
S2/S3 boundary is type-distinguishable from input-shape errors at
``submit_event``.  Pros: caller-side ``except`` clauses can narrow
on intent.  Cons: breaks symmetry with the other S1–S6 enforcement
points (all use plain ``ValueError`` with a message-prefix
discriminator), would require adding the new class to the
``__all__`` of ``controller.scheduler``, and has no callers that
would benefit today (there are zero ``begin_return`` callers that
catch ``ValueError`` to distinguish failure modes).

The plan-author already pre-recommended the simpler default ("raise
per S3 (consistent with submit_event) — begin_return is not a
documented bypass for S3"), and the user's design choice during
implementation was to follow that recommendation.  S3 consistency
is the higher-leverage outcome: the contract's enforcement message
prefix ("Sx violation in op_name: …") is now the single canonical
substring that operator-side log-grepping needs to recognise.

### Why the helper, not a duplicate inline check

``_validate_slot_capacity`` already has the right shape: it returns
``None`` on success, raises ``ValueError`` with the canonical message
on the violating state, and takes ``op_name`` as a kwarg-only
parameter for the message body.  Calling it with
``op_name='begin_return'`` produces exactly the right message
("S3 violation in begin_return: …") without any code duplication.
A duplicate inline raise would have drifted in message format the
moment a future audit normalised the ``submit_event`` form.

### Upstream impact

``begin_return`` has exactly one production caller:
[sim/hand/scheduled_coordinator.py:310](../sim/hand/scheduled_coordinator.py).
It calls ``begin_return`` only when ``sched_out.phase ==
SchedulerPhase.HOLDING`` (line 304).  In HOLDING the new S3 guard is
not reachable — the first branch of ``begin_return`` (HOLDING + no
next) is taken, the third branch is never executed.  So the
production caller is unaffected.

The state-machine random walk's ``begin_return`` rule was the only
test-side path that hit the bug, and it was already gated.

### Downstream impact

Inside ``begin_return``, ``_validate_slot_capacity`` is invoked
*before* any state mutation in the third branch — so the raise leaves
``_current_event`` / ``_next_event`` / ``_seg_next`` /
``_pending_notification`` untouched.  Subsequent ``update()`` calls
proceed with the original lookahead/destination event intact.  The
hot-loop allocation contract is unaffected — ``begin_return`` is not
on the hot loop.

## Implementation

### controller/scheduler.py (modified)

``begin_return`` ([scheduler.py:762–807](../controller/scheduler.py))
gains one new line in the third branch (the "queue
``RETURN_TO_ACTIVE`` as next event" path):

```python
else:
    # Queue a return as the next event.  S3: refuse to overwrite
    # an existing _next_event — begin_return is not a documented
    # bypass for the bounded in-flight slot set.
    self._validate_slot_capacity(op_name='begin_return')
    ret_event = ScheduledEvent(...)
    self._next_event = ret_event
    self._seg_next = None
```

The docstring is expanded to document the S3 raise behaviour and to
direct callers at ``cancel_next()`` / ``clear()`` for the legitimate
slot-clearing path before requesting return.

### controller/SCHEDULER_CONTRACT.md (modified)

The S3 normative section gains one paragraph clarifying that
``begin_return`` is not a documented bypass — the third branch is
subject to the same bounded-slot-set invariant as ``submit_event``.
This pairs with the bug-1 ``cancel_next``-precondition paragraph
landed in the prior commit; both clarifications live under the new
"S3 — Phase preconditions on slot mutators" sub-heading.

### tests/sim/test_scheduler_contract.py (modified)

Added a new test class ``TestBeginReturnSlotCapacity`` with five
scenarios:

| Test ID | Scenario | Asserts |
|---------|----------|---------|
| T-U-T0-3 | ``begin_return`` during ``APPROACHING`` with both slots filled | Raises ``ValueError`` with "S3 violation in begin_return"; phase / both slots unchanged after the raise |
| T-U-T0-4 (regression) | ``begin_return`` during ``APPROACHING`` with only ``_current_event`` filled | Queues ``RETURN_TO_ACTIVE`` in next slot (existing behaviour preserved); phase unchanged |
| (extra) | ``begin_return`` during ``TRANSITIONING`` (next slot is the destination) | Raises per S3; state unchanged |
| (extra) | ``begin_return`` from ``HOLDING`` with no next | Flips to ``RETURNING`` (canonical happy path; first branch of ``begin_return``) |
| (extra) | ``begin_return`` from ``IDLE`` | No-op; phase stays ``IDLE`` |

Removed the ``@precondition`` gate at the
[SchedulerStateMachine.begin_return](../tests/sim/test_scheduler_contract.py)
rule.  The rule now wraps ``self.scheduler.begin_return()`` in a
``try / except ValueError`` and asserts the raise message includes
``"S3 violation in begin_return"`` AND that both
``active_event`` and ``next_event`` were non-None at raise time —
guarding both "didn't raise when it should have" and "raised in
the wrong slot-state" regression classes.

## Verification

### Pre-fix regression test

The new ``test_begin_return_with_both_slots_filled_raises`` was
confirmed to fail with ``Failed: DID NOT RAISE <class 'ValueError'>``
when the production fix was stashed (``git stash push
controller/scheduler.py``) and re-run in isolation.  Re-applying the
fix made the test pass deterministically.

### Test results

- **Before this fix:** 1188 passing (after [bug 1](2026-05-10-scheduler-cancel-next-during-transitioning.md)),
  ``SchedulerStateMachine.begin_return`` precondition-gated.
- **After this fix:** 1193 passing (+5 new ``TestBeginReturnSlotCapacity``
  scenarios), ``SchedulerStateMachine.begin_return`` rule un-gated.

ci-fast (``pytest tests/ -q``): pass (1192/1193; one transient flake
on ``test_motor_guard.py::test_decay_boundary_continuity`` — the
pre-existing heap-state contamination flake noted in
[Plan 2 Working Note #5](../plans/active/mpc-sadpath-coverage-tiers-1-3.md);
passes deterministically in module-level isolation).

ci-deep on the contract test file (``pytest
tests/sim/test_scheduler_contract.py --hypothesis-profile=ci-deep
--hypothesis-seed=0``): **69 / 69 pass in 97.40 s**.  The fully
un-gated ``SchedulerStateMachine`` random walk at 1000 examples
surfaces no new ``begin_return``-related state regressions —
both deferred ``@precondition`` gates are now closed and the random
walk runs against the canonical contract surface.

## Discussion

### Why ``begin_return`` has phase-dependent semantics at all

A reasonable redesign would be: make ``begin_return`` a thin wrapper
that always succeeds and always signals "I want a return queued",
deferring the slot-and-phase logic to the next ``update()``.  The
present three-branch shape (HOLDING + no next → flip; IDLE → no-op;
otherwise → queue) is a workable but not particularly clean
contract.

Phase 0's scope is *not* to redesign the API — it's to close the
deferred ``@precondition``s.  The fix matches the existing branch
shape and just plugs the S3 hole in the third branch.  A future
contract-level redesign (deserving its own plan) might unify
``begin_return``'s three branches into a single declarative form;
that's out of scope here.

### What ``begin_return``'s second branch actually means now

Post-fix, ``begin_return``'s third branch has the same slot-capacity
contract as ``submit_event`` — it's a "queue an event" operation,
just with an event the scheduler synthesises rather than the caller
providing.  The second branch (IDLE → no-op) survives unchanged
because there's nothing to queue (no current event, no transition
in flight, the platform is already at active pose).

This means the API surface is cleaner:

- **``submit_event``** — caller queues a caller-supplied event.
  S1 / S2 / S3 all enforced.
- **``begin_return``** — scheduler queues a synthesised
  ``RETURN_TO_ACTIVE`` event.  S3 enforced (the relevant invariant
  for a synthesised event with a placeholder ``time=0.0`` and
  fresh ``event_id``); S1 inapplicable (no caller-side time
  calculation); S2 inapplicable (the helper's ``_next_event_id()``
  guarantees uniqueness).

S3 is the only S-invariant ``begin_return`` could have legitimately
violated, and the fix closes that hole.

### State-machine semantic completeness reached

With both Plan 2 Phase 0 bugs fixed, the ``SchedulerStateMachine``'s
two ``@precondition`` gates are removed.  The random walk now
exercises:

- ``submit`` at every slot/phase combination — S1/S2/S3 enforcement
  surface (Phase 2).
- ``replace_next`` at every slot/phase combination — S1/S5
  enforcement surface (Phase 2 + Phase 3).
- ``cancel_next`` at every phase — S3 capacity-restore surface +
  the new "must not abort transitions silently" rule (Phase 0 bug 1).
- ``begin_return`` at every slot/phase combination — S3
  enforcement on synthesised ``RETURN_TO_ACTIVE`` events
  (Phase 0 bug 2).
- ``tick`` with positive ``dt`` only — S6 monotonicity by
  construction (Phase 3).

The contract surface that remains *un-walked* is intentional: the
state machine is a property test on contract enforcement, not a
property test on physics correctness.  Numerical contract surfaces
(K1–K6 reference feasibility, P1–P4 plant interface) live in their
own property tests with their own state-machine flavours.

### Why two separate logbook entries instead of one combined

Same rationale as bug 1's [Discussion](2026-05-10-scheduler-cancel-next-during-transitioning.md#why-split-this-fix-from-begin_return):
rollback granularity and Discussion-section coherence.  Each bug has
its own design-alternative analysis (raise-vs-atomic-transition
for ``cancel_next``; raise-vs-new-error-class for ``begin_return``),
its own enforcement site, and its own scope-of-impact rationale.
Bundling them would have forced two parallel decision trees into a
single narrative.

The two entries can be read independently — neither requires the
other for context — and the SCHEDULER_CONTRACT.md update under "S3 —
Phase preconditions on slot mutators" combines both clarifications
under a single normative sub-heading.

## Open Questions

- **Should ``begin_return`` be promoted to a state-machine semantic
  contract on its own?**  The bug 1 ``cancel_next`` fix elevated the
  "phase preconditions on slot mutators" sub-heading under S3.  The
  ``begin_return`` clarification reuses the same sub-heading.  A
  future plan that grows the slot-mutator surface (e.g., a multi-ball
  juggling planner with batch ``submit_events`` semantics) might
  prefer to graduate this to a first-class S7 invariant rather than
  an S3 sub-clause.  Out of scope for Phase 0.
- **The placeholder ``time=0.0`` on synthesised
  ``RETURN_TO_ACTIVE`` events.**  ``begin_return``'s third branch
  builds the event with ``time=0.0`` and a comment "will be computed
  when segment is built"
  ([scheduler.py:802](../controller/scheduler.py)).  This is
  surprising — every other ``ScheduledEvent`` has a meaningful
  ``time`` field (per S1).  S1 doesn't fire because ``begin_return``
  doesn't go through ``_validate_submission_time``, but a future
  audit pass might want to make the placeholder explicit (e.g., a
  ``time = sentinel_TBD`` constant) or to compute the actual arrival
  time at queue time rather than at segment-build time.  Out of
  scope for Phase 0.

## Related

- [logbook/2026-05-10-scheduler-cancel-next-during-transitioning.md](2026-05-10-scheduler-cancel-next-during-transitioning.md)
  — Bug 1 of 2, same plan phase.
- [logbook/2026-05-09-scheduler-contract-phase-3-s4-s6-enforcement.md](2026-05-09-scheduler-contract-phase-3-s4-s6-enforcement.md)
  — Phase 3 random-walk surfaced this bug.
- [controller/SCHEDULER_CONTRACT.md](../controller/SCHEDULER_CONTRACT.md)
  — S3 normative spec; this fix updates the "S3 — Phase preconditions
  on slot mutators" sub-section.
- [plans/active/mpc-sadpath-coverage-tiers-1-3.md](../plans/active/mpc-sadpath-coverage-tiers-1-3.md)
  — Plan 2 Phase 0 specification.
