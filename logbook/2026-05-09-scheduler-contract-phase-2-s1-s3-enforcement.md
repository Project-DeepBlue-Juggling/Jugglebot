---
title: Scheduler contract — Phase 2 enforcement of S1, S2, S3
type: feature
date: 2026-05-09
status: resolved
phase: "mpc-tier0-contracts — Phase 2"
related_plan: "mpc-tier0-contracts.md"
related_entries:
  - 2026-05-09-scheduler-contract-phase-1-audit
  - 2026-04-20-k1-k6-reference-feasibility-resolution
  - 2026-04-23-hot-loop-zero-allocation-contract
files_changed:
  - controller/scheduler.py
  - tests/sim/test_scheduler_contract.py
  - plans/archived/2026-05-10 mpc-tier0-contracts.md
  - logbook/2026-05-09-scheduler-contract-phase-2-s1-s3-enforcement.md
  - logbook/INDEX.md
commits:
  - 1316f0f
subsystem:
  - controller
  - mpc
tags:
  - contract
  - testing
  - safety
---

# Scheduler contract — Phase 2 enforcement of S1, S2, S3

## Summary

Phase 2 of the [mpc-tier0-contracts plan](../plans/archived/2026-05-10%20mpc-tier0-contracts.md):
implemented input-domain enforcement for the three input-shape /
state-collision invariants S1 (submission time), S2 (unique IDs), and
S3 (bounded slot set) defined in
[controller/SCHEDULER_CONTRACT.md](../controller/SCHEDULER_CONTRACT.md)
(landed Phase 1).  Behaviour change: previously-undefined inputs at
``submit_event`` and ``replace_next_event`` now raise ``ValueError``
loudly instead of silently degrading or overwriting in-flight state.

## Motivation

[Phase 1](2026-05-09-scheduler-contract-phase-1-audit.md) drafted the
S1–S6 contract and surfaced the gaps to close.  Phase 2 picks the
*input-domain* subset — the three invariants that can be enforced by
guards at a single API entry point (``submit_event`` /
``replace_next_event``) without touching the state machine.  Phase 3
will tackle S4–S6, which require state-machine work.

The split is intentional: input-domain enforcement is a thin slice that
introduces small, easy-to-review behaviour changes, lets us land tests
that pin those changes, and unblocks Plan 2's Tier 1 testing (which
references S1–S6 by name).

## Design

Three private validation helpers on ``EventScheduler`` evaluated in
order ``S1 → S2 → S3`` at the entry of ``submit_event``:

| Helper | Invariant | Rule |
|--------|-----------|------|
| ``_validate_submission_time`` | S1 | ``event.time`` MUST be finite (NaN / ±inf raise unconditionally — including before first ``update()``); ``event.time >= _last_sim_time − tau_grace_s`` (clock-skew comparison is no-op when ``_last_sim_time`` is None) |
| ``_validate_id_uniqueness`` | S2 | ``event.event_id`` not in ``{_current_event.id, _next_event.id}``; ``check_next=False`` for ``replace_next_event`` (next is being replaced) |
| ``_validate_slot_capacity`` | S3 | ``submit_event`` raises iff both slots already occupied |

``replace_next_event`` calls only the first two (S1 + partial-S2);
``update_current_event`` is unchanged (out of S1/S2/S3 scope per the
contract — refinement of an in-flight event is its own API path).

### Constructor signature

A new ``tau_grace_s: float | None = None`` parameter on
``EventScheduler.__init__`` controls the S1 clock-skew tolerance.
``None`` (the default) derives the value from ``cumulative_times[1] -
cumulative_times[0]`` (one MPC tick), matching the contract's "default
1 × control_dt" rule.  Negative values raise.

### Validation order — S1 → S2 → S3

The order is documented and tested
([test_scheduler_contract.py::TestValidationOrder](../tests/sim/test_scheduler_contract.py)).
Rationale: S1 is "is the input shape well-formed" (time is in the
addressable range); S2 is "does it conflict with current state" (id is
unique); S3 is "does it fit capacity" (slot space available).  When
multiple invariants are violated by the same event, reporting the
*most fundamental* violation first gives the caller the clearest
diagnostic.

### Error message format

Each error includes:

- The violating invariant tag (``S1 violation`` / ``S2 violation`` / ``S3 violation``)
- The operation that triggered it (``submit_event`` / ``replace_next_event``)
- The diagnostic values needed to reproduce the failure (event.time,
  last_sim_time, tau_grace_s for S1; event.event_id and conflicting
  slot for S2; both slot ids for S3)
- A pointer to the correct API for the failing intent ("Use
  update_current_event() for refinements; submit_event/
  replace_next_event are for distinct events" etc.)

## Implementation

### controller/scheduler.py (modified)

- Constructor: added ``tau_grace_s`` parameter with derived default
  (``cumulative_times[1] - cumulative_times[0]``) and
  finite-and-non-negative validation (NaN / ±inf / negative all raise
  ``ValueError``).  Added ``_tau_grace_s`` and ``_last_sim_time``
  instance state.
- ``submit_event``: prepended ``S1 → S2 → S3`` validation; removed the
  "silently overwrite ``_next_event``" branch (it's now blocked by S3).
  Cleaned the resulting branch: when ``_current_event`` is non-None,
  S3 has already verified ``_next_event`` is None, so the body is a
  pure "queue beyond current" assignment.  The pre-contract "replaced
  next event id=N with id=M" log line is gone (the path it logged is
  now an S3 raise).
- ``replace_next_event``: prepended ``S1 + partial-S2`` validation
  (current-event id only; next slot is being replaced).  S3 not
  applicable — replacement doesn't grow the slot set.
- ``_validate_submission_time``: rejects non-finite ``event.time``
  (NaN / ±inf) unconditionally — including before the first
  ``update()`` — then applies the clock-skew comparison only after
  ``_last_sim_time`` has been set.  This closes a real silent-bypass
  vector that the post-implementation audit caught: ``nan < threshold``
  is always False, so without the finiteness check NaN events would
  pass S1 and propagate into ``TargetCommand.target_pose``.
- ``update``: rejects non-finite ``sim_time`` (NaN / ±inf) before
  storing ``_last_sim_time``.  Closes the second silent-bypass vector
  the audit caught — a poisoned ``_last_sim_time`` would silently
  disable S1 for every subsequent submission.  Phase 3 will add the
  S6 monotonicity check here (compare-then-update); for Phase 2 we
  only validate finiteness and track the value.

### tests/sim/test_scheduler_contract.py (new)

Six test classes / 40 scenarios, organised by invariant.  The 33
contract scenarios were authored at implementation time; the 7
NaN/inf regression guards (one in ``TestTauGraceConstruction`` and
six in ``TestS1SubmissionTime``) were added during the
post-implementation ``/audit --unstaged`` pass to close two real
silent-bypass vectors the initial implementation missed (NaN
``event.time`` and NaN ``sim_time``):

| Class | Scope | Tests |
|-------|-------|-------|
| ``TestTauGraceConstruction`` | Constructor parameter handling | 6 (default derivation, override, zero, negative-rejection, NaN-rejection, short-cumulative-times fallback) |
| ``TestS1SubmissionTime`` | S1 enforcement | 14 (pre-update unconstrained, past-time raises, boundary inclusive, just-past-boundary raises, future accepted, replace_next_event also validates, update_current_event exempt, error-message diagnostic, NaN raises, +inf raises, −inf raises, NaN-before-first-update raises, NaN sim_time at update raises, +inf sim_time at update raises) |
| ``TestS2UniqueIds`` | S2 enforcement | 8 (current-id collision, next-id collision, distinct accepted, replace-next current-id raises, replace-next reusing-old-next-id accepted, replace-next fresh-id accepted, update_current_event matching-id legitimate, diagnostic message) |
| ``TestS3BoundedSlots`` | S3 enforcement | 7 (two accepted, third raises, third does not mutate state, after-cancel third accepted, after-clear third accepted, replace_next bypasses S3, diagnostic message) |
| ``TestValidationOrder`` | Precedence order | 3 (S1 > S2, S2 > S3, S1 > S3) |
| ``TestBehaviourChangeRegression`` | Pin the new loud-fail behaviour | 2 (silent-overwrite path now raises and preserves state, replace_next_event is the documented replacement path) |

### plans/archived/2026-05-10 mpc-tier0-contracts.md (modified)

Phase 2 marked ``COMPLETE (2026-05-09)`` in both the summary table
and the detailed Phase 2 heading.

## Verification

### Existing tests — no behaviour-change impact (pre-audit prediction)

Pre-implementation audit of [tests/sim/test_scheduler.py](../tests/sim/test_scheduler.py)
(7 classes / 24 tests) walked every submission and verified:

- Most tests submit BEFORE any ``update()`` call, so ``_last_sim_time``
  is None at submission time and S1 is a no-op.
- All event IDs are auto-generated via ``_next_event_id()`` (a process-
  wide monotonic counter), so S2 collisions are impossible by
  construction.
- The single ``replace_next_event`` test (``test_replace_next_during_transitioning``)
  uses ``time=0.8`` after ``update(0.15)``, so S1 is satisfied.
- No test deliberately exercises a 3rd submission when both slots are
  full.

Confirmed empirically: ``pytest tests/sim/test_scheduler.py -v`` →
**24 / 24 pass** with no test changes.

### External caller — sim/hand/scheduled_coordinator.py

Three submission-API call sites in the sim coordinator
(``submit_event``, ``update_current_event``, ``begin_return`` at
[sim/hand/scheduled_coordinator.py:147, 197, 310](../sim/hand/scheduled_coordinator.py))
are the surface area S1/S2/S3 governs.  The other three call sites
(``phase`` at :105, ``update`` at :242, ``clear`` at :332) are
out of S1/S2/S3 scope: ``phase`` is read-only, ``update`` is the
clock advance (Phase 3's S6 will check it there), and ``clear``
resets state without submitting events.

- ``submit_event`` always uses ``_next_event_id()`` (S2 collision-free)
  and submits with ``arrival_time - _SETTLE_MARGIN_S`` from ball
  physics, which is always future-dated (S1 satisfied).
- ``update_current_event`` is the legitimate refinement path (out of
  S1/S2/S3 scope).
- ``begin_return`` doesn't pass user events through the contract surface.

S3 is the one invariant that depends on higher-level orchestration
(does the sim ever queue 3 events deep?).  Confirmed empirically: the
sim test suite (including ``test_ball_butler_sim``, ``test_scheduled_catch``,
``test_post_catch``, ``test_multi_event``) passes without S3 firing.

### Test results

- **Before Phase 2:** 1091 / 1091 pass.
- **After Phase 2:** **1131 / 1131 pass** — the +40 delta is exactly
  the new ``test_scheduler_contract.py`` count (33 contract scenarios
  + 7 NaN/inf regression guards added during the post-implementation
  audit).  Zero regressions.

## Discussion

### Why S1 is no-op until first ``update()``

The contract permits this gap, and the implementation honours it: an
event submitted before any ``update()`` has been observed has no
reference clock.  Two design choices were considered:

1. **Refuse all submissions until first update** (require
   ``update()`` before ``submit_event()`` is callable).  Cleaner
   contract; breaks every existing test fixture that submits *then*
   updates (including the canonical sim catch coordinator pattern at
   [scheduled_coordinator.py:147](../sim/hand/scheduled_coordinator.py)
   where the catch event is submitted in the same tick its arrival
   becomes known).
2. **No-op until first update** (chosen).  Matches existing usage,
   acknowledges the bootstrap window honestly, accepts that
   pre-update submissions can have nonsensical times.

The pre-update-no-op path is marked in code with the explicit comment
``# S1 disabled until first update — see contract S1`` so a future
contributor doesn't think it's a bug to fix.

### Why ``update_current_event`` is exempt from S1

Submission is the act of *introducing* an event into the in-flight set;
refinement (``update_current_event``) is *modifying* an event already in
the set, by the same operator who submitted it.  The contract scopes
S1 to submission for two reasons:

1. **Different semantic.** Past-time-at-submission is "this event was
   stale when sent" (e.g., re-sent after a long pause).  Past-time-at-
   refinement is "the arrival prediction now points to the past" — a
   tracker / physics-update bug, not a clock-skew or stale-ref bug.
2. **Different recovery path.** The submission path's recovery is "use
   ``cancel`` / ``clear`` and resubmit fresh."  The refinement path's
   recovery is "the event is already ours; clamp its segment to
   ``_MIN_DURATION_S`` and let the MPC track it as best it can"
   (existing behaviour at [scheduler.py:791](../controller/scheduler.py)
   that S5 codifies as a degraded-but-acceptable mode).

The contract draws the line where the *operator's intent* differs.

### Why the S1 → S2 → S3 order

There were three plausible orderings:

- **Capacity first (S3 → S2 → S1):** report "no room" before bothering
  with id or time checks.  Rejected: capacity is the most contingent
  axis; reporting it first hides the deeper input-shape bug if both
  apply.
- **Identity first (S2 → S1 → S3):** id is the strongest unique
  identifier.  Rejected: time is even more fundamental — an event
  can have a fresh id and be a legal refinement, but a past time is
  a categorical "this didn't make sense at any moment" error.
- **Input-shape first (S1 → S2 → S3, chosen):** matches the
  "well-formed input?  → conflict-free state?  → fits capacity?"
  validation pyramid common in API design.  Reports the most
  fundamental violation first.

The order is pinned by [TestValidationOrder](../tests/sim/test_scheduler_contract.py)
so a future refactor that swaps the order trips the test.

### What landed without enforcement

S4 (internal quintic feasibility), S5 (segment-construction live-state),
and S6 (sim_time monotonicity) are NOT enforced in this phase.  Phase 3
covers them.  S5 is already an implementation invariant that the
existing code maintains; S4 is partially implemented (the toggle
exists, but the contract-required "warn once on missing v_max/tau"
diagnostic is Phase 3 work); S6's ``_last_sim_time`` is *tracked*
already (this phase needed it for S1) but not yet *checked* for
backward jumps.

Phase 3 will use the bookkeeping landed here as its starting point,
adding the S6 raise at the entry of ``update()``.

### Behaviour change — silent overwrite is gone

The largest behaviour change in this phase is at
[scheduler.py:262–287 (pre-Phase-2)](../controller/scheduler.py): the
"else" branch in ``submit_event`` previously routed a duplicate or
slot-saturating submission to ``_next_event``, silently overwriting
whatever was there.  Phase 2 makes both paths raise:

- Duplicate id matching either slot → S2.
- Both slots filled → S3.

The legitimate replacement path (``replace_next_event``) is preserved
and now also validates S1 + partial-S2.  No production caller in
this codebase relied on the silent overwrite; the test suite confirms
this empirically.

## Open Questions

- **S1 on ``update_current_event``.** The contract scopes S1 to
  ``submit_event`` and ``replace_next_event``, NOT
  ``update_current_event``.  This is documented intent in the
  Discussion section above, but worth a follow-up if a tracker bug
  ever produces past-time refinements that surprise an operator.
  The fall-back behaviour today (clamp to ``_MIN_DURATION_S``) is
  the same as pre-contract; S1 doesn't make it louder.
- **Tau-grace as a scheduler-level vs. plant-level parameter.**
  Phase 4 (``PLANT_INTERFACE_CONTRACT.md``) introduces a
  ``control_dt`` invariant on the plant.  This phase derives
  ``tau_grace_s`` from the scheduler's own ``cumulative_times`` instead.
  When Phase 4 lands, consider whether the scheduler should consume
  ``control_dt`` from the plant (one source of truth) instead of
  re-deriving from the MPC horizon schedule.  Captured for Phase 4
  follow-through.
- **State-machine property tests.**  Phase 2 ships scenario tests
  only.  Phase 3 will add a hypothesis ``RuleBasedStateMachine``
  that exercises arbitrary submit/cancel/update/replace sequences
  and asserts S1–S6 invariants throughout.  The Phase 2 enforcement
  is a *prerequisite* for that machine — without it, the random
  sequences would generate S1/S2/S3-violating states that the
  property assertions would falsely interpret as scheduler bugs.
