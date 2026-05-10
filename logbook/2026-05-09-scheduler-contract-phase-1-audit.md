---
title: Scheduler contract — Phase 1 audit and S1–S6 draft
type: feature
date: 2026-05-09
status: resolved
phase: "mpc-tier0-contracts — Phase 1"
related_plan: "mpc-tier0-contracts.md"
related_entries:
  - 2026-04-20-k1-k6-reference-feasibility-resolution
  - 2026-04-23-hot-loop-zero-allocation-contract
files_changed:
  - controller/SCHEDULER_CONTRACT.md
  - plans/archived/2026-05-10 mpc-tier0-contracts.md
  - logbook/2026-05-09-scheduler-contract-phase-1-audit.md
  - logbook/INDEX.md
commits:
  - 7b202ae
subsystem:
  - controller
  - mpc
tags:
  - contract
  - testing
  - docs
  - safety
---

# Scheduler contract — Phase 1 audit and S1–S6 draft

## Summary

Phase 1 of the [mpc-tier0-contracts plan](../plans/archived/2026-05-10%20mpc-tier0-contracts.md):
audited [controller/scheduler.py](../controller/scheduler.py) for invariants
the code currently relies on (explicit and implicit), then drafted
[controller/SCHEDULER_CONTRACT.md](../controller/SCHEDULER_CONTRACT.md)
following the structural template of
[REFERENCE_LAYER_CONTRACT.md](../controller/REFERENCE_LAYER_CONTRACT.md)
and [HOT_LOOP_CONTRACT.md](../controller/HOT_LOOP_CONTRACT.md).  No code
changes — Phases 2–3 land enforcement.

## Motivation

The 2026-05-08 cross-cutting MPC audit identified that **sad-path test
coverage in this codebase tracks contract coverage**: where K1–K6 and the
hot-loop allocation budget are formalised, ratchet-protected enforcement
and rich tests follow.  Where the contract is implicit — as in the
scheduler — the test suite has happy-path coverage (24 tests across 7 classes in
[tests/sim/test_scheduler.py](../tests/sim/test_scheduler.py)) but no
input-domain or state-machine invariants.  The scheduler routes catch
events, drives quintic-Hermite chaining, and feeds the MPC's reference
each tick — it is a safety-critical surface and warrants the same
contract treatment.

Phase 1 deliberately produces no code change; it is the discovery + design
pass that surfaces what the scheduler currently guarantees and what it
silently degrades on.  Phases 2 and 3 land enforcement against that
specification.

## Design

The contract follows the existing template: Background → Invariants →
Enforcement → Implementing-a-new-source template → Diagnosis → Related.
Six normative invariants:

| Invariant | Statement (one line) |
|-----------|----------------------|
| S1 | ``event.time >= last_sim_time − τ_grace`` at submission; past-time events raise ``ValueError`` |
| S2 | ``event.event_id`` MUST NOT collide with any in-flight event's ID at ``submit_event`` time |
| S3 | In-flight slot set bounded to 2 ``ScheduledEvent`` slots + 1 ``_QuinticSegment`` return slot; saturating ``submit_event`` raises |
| S4 | Every scheduler-built ``_QuinticSegment`` MUST pass ``_verify_segment_feasibility`` (K2/K3) |
| S5 | Every newly-built segment MUST start at the scheduler's recorded live motion state (C0 splice continuity) |
| S6 | ``update(sim_time)`` MUST satisfy ``sim_time >= last_sim_time``; backward jumps raise |

S1, S2, S3, and S6 are *new behaviour* on previously-undefined inputs —
each closes a class of silent caller-side failure mode that the audit
identified.  S4 *promotes* an existing implementation feature
(``_verify_segment_feasibility``, [scheduler.py:667](../controller/scheduler.py))
to contract status, and tightens its documented degraded modes (silent
skip when ``v_max_mmps`` / ``tau_s`` are unset → loud once-at-construction
warning).  S5 *codifies* an existing implementation invariant (live-state
segment construction at
[scheduler.py:644–665](../controller/scheduler.py)) so that future
refactors cannot silently drift away from it.

## Audit findings

The audit walked every public method on ``EventScheduler`` and every
internal segment-construction path, comparing documented behaviour
against actual code.  Five concrete gaps surfaced; each became an
invariant.

### Gap 1 — past-time events silently clamped

[scheduler.py:652](../controller/scheduler.py): ``duration =
max(event.time - sim_time, _MIN_DURATION_S)``.

A submission with ``event.time < sim_time`` produces a 50 ms segment
ending at the event's pose at ``sim_time + 0.05 s`` — the segment is
kinematically valid but operationally wrong: the event was supposed to
arrive in the past, and the scheduler silently re-aimed at a
near-future arrival.  No log, no warning.  Closed by **S1**.

### Gap 2 — duplicate IDs route to the wrong slot

[scheduler.py:262–287](../controller/scheduler.py).  ``submit_event``
branches on ``_current_event is None``; if a current event exists, the
"else" branch silently overwrites ``_next_event``.  This branch does
not check ``event.event_id`` — so a caller intending "refine event 42"
via the wrong API (``submit_event`` instead of
``update_current_event``) silently corrupts the next-event slot with
a duplicate of event 42's ID, producing two distinct events with the
same ID in the in-flight set.  Closed by **S2**.

### Gap 3 — three-deep submission silently truncated

The structural design intent is ``current + lookahead = 2 explicit
events``.  A third ``submit_event`` overwrites ``_next_event`` (the
same code path as Gap 2).  Pipeline overflow is invisible to the
caller.  Closed by **S3**.

### Gap 4 — K2/K3 check silently skipped on missing parameters

[scheduler.py:693–694](../controller/scheduler.py): ``if
self._v_max_mmps is None or self._tau_s is None: return``.

A scheduler constructed without one of these parameters silently
disables ``_verify_segment_feasibility``.  The skip is invisible to
the caller — there is no construction-time warning, and the test
fixture in [tests/sim/test_scheduler.py:56](../tests/sim/test_scheduler.py)
constructs without these parameters, so the test suite has been
exercising the unenforced path the entire time.  Closed by **S4** (now
requires production callers to provide both, with a once-at-construction
warning when missing).

### Gap 5 — backward sim_time undefined

``update(sim_time)`` does not validate against the previously-seen
``sim_time``.  A sim restart without ``clear()`` produces:

- Negative-duration segments (clamped via Gap 1's mechanism, but with
  no relationship to the original event timing)
- Spurious arrival detection (or missed arrivals) at the
  ``sim_time >= seg.t_end`` checks at
  [:417](../controller/scheduler.py),
  [:510](../controller/scheduler.py), and
  [:576](../controller/scheduler.py)
- Hand-notification re-fires on backward jumps past arrival times

Closed by **S6**.

### Non-gap — C0 continuity at segment splices

S5 codifies an invariant the implementation already maintains:
``_build_segment_to_event`` always reads ``self._last_pose /
_last_twist / _last_accel`` (live motion state) for segment start
boundary conditions.  This is preserved across all phase transitions
including ``replace_next_event`` mid-TRANSITIONING (which builds the
new segment from the live state at
[scheduler.py:503–504](../controller/scheduler.py)).  S5 is included
not because there's a current bug but because the invariant is
load-bearing for K1 (live-state anchor) on the FIRST emitted event of
every new segment, and a future contributor refactoring
``replace_next_event`` could silently break it.

## Discussion

### Why six invariants and not more

The audit also surfaced a candidate seventh invariant — *``_pending_notification``
queue depth: the scheduler has a single-slot notification queue; if two
notifications arrive in quick succession (e.g., ``submit_event`` followed
by an ``arrived`` transition on the same tick), the second silently
overwrites the first.* This is real, but:

1. It's not *safety*-critical — notifications drive the hand
   coordinator's UI/audio state, not motion.
2. The single-slot queue is *intentional* per the inline comment at
   [scheduler.py:232](../controller/scheduler.py) ("Pending hand
   notifications (consumed by caller after update)") — the design
   contract is that the caller drains every tick.
3. Lifting it into the formal contract would lock in that single-slot
   design when the natural fix (a small bounded deque) might be wanted
   later.

So this stays as a documented quirk, not an invariant.  If hand-
coordinator UX bugs ever trace back to it, S7 can be added later.

### Why S2 raises rather than dispatches to ``update_current_event``

A "smart" alternative would be: ``submit_event`` with a duplicate ID
silently dispatches to ``update_current_event`` ("the caller meant
refinement").  This was ruled out because:

- It conflates two semantic operations ("queue new" vs "refine
  existing").
- It makes the caller's intent invisible at the call site — the next
  reader can't tell if a refinement or a new event was meant.
- It encourages buggy callers to keep accidentally using the wrong API
  with no signal that anything is off.

A loud raise forces the caller to choose the right API.  This matches
the "favour contracts over patches" engineering philosophy.

### Why ``τ_grace`` is configurable

S1's strict form ``event.time >= last_sim_time`` would be brittle: even
benign clock skew between the event-source's reference and the
scheduler's would trip it.  The default ``τ_grace = 1 × control_dt``
gives a one-tick clock-skew allowance, which empirically is enough
margin for well-aligned sources without papering over real bugs.  Tests
that need stricter checking can construct with ``τ_grace = 0``.

### Why S4 keeps the strict-vs-warning toggle

The pre-contract code already had ``strict_feasibility`` as a
constructor parameter.  Keeping it preserves the production policy
("graceful degradation via W7 fallback") while making tests strict by
default.  The alternative — always raise — was ruled out because the
W7 fallback is the documented production-response to an infeasible
ref ([REFERENCE_LAYER_CONTRACT.md](../controller/REFERENCE_LAYER_CONTRACT.md)
"Stretch policy"), and crashing the juggling demo on a single
infeasible event would be worse than degrading.

### Phase 1 produces no code change — by design

The plan ([mpc-tier0-contracts.md](../plans/archived/2026-05-10%20mpc-tier0-contracts.md)
Phase 1) explicitly excludes code changes from this phase.  Reason:
*the contract document is the design artefact*.  Landing it
independently lets the user review the *spec* before any *enforcement*
locks in behaviour changes.  Phases 2 and 3 implement S1–S3 and
S4–S6 enforcement respectively, with their own logbook entries and
gates.

## Implementation

| Path | Change |
|------|--------|
| `controller/SCHEDULER_CONTRACT.md` | Created (~400 lines): Background, six S-invariants with Why + enforcement pointers, Enforcement table, new-source template, Diagnosis, Related |
| `plans/archived/2026-05-10 mpc-tier0-contracts.md` | Phase 1 status: `NOT STARTED` → `IN PROGRESS (started 2026-05-09)` → `COMPLETE (2026-05-09)` |
| `logbook/INDEX.md` | New entry row (this document) |

No production code changes.

## Verification

- ``pytest tests/ -q`` — full test suite green (this phase is doc-only;
  the run is a regression sanity check).
- Document cross-references checked: every linked file path
  (REFERENCE_LAYER_CONTRACT.md, HOT_LOOP_CONTRACT.md, scheduler.py
  line numbers, test files) resolves to a real artefact in the repo.
- The contract's S5 enforcement claim ("``_build_segment_to_event``
  always reads ``self._last_pose``...") was verified by reading
  [scheduler.py:644–665](../controller/scheduler.py); the assertion
  holds in the current implementation.
- The contract's S4 enforcement claim ("``_verify_segment_feasibility``
  is called from ``_build_segment_to_event`` and the return-to-active
  path") was verified by grep-confirming the two call sites at
  [scheduler.py:664](../controller/scheduler.py) and
  [scheduler.py:571](../controller/scheduler.py).

## Outcome

The scheduler now has a written contract analogous to K1–K6 and the
hot-loop budget.  Phases 2 (S1–S3) and 3 (S4–S6) of the same plan can
now build enforcement against this specification with property-based
hypothesis tests.

The five invariants that introduce new behaviour (S1, S2, S3, S6 raise
on previously-undefined inputs; S4 warns on previously-silent skip)
are scoped to caller-side bugs that produce malformed inputs.  No
correct caller exercises these inputs today, so enforcement should
land without observable behaviour change in production paths.

## Open questions

- *Hand-notification queue depth (potential S7).* Documented as a
  non-gap in the Discussion above.  Revisit if hand-coordinator UX
  bugs trace back to a missed notification.
- *S4 in tests vs production.* The contract requires test fixtures to
  use ``strict_feasibility=True``.  An audit pass over existing
  scheduler tests in Phase 3 may surface fixtures that need
  retrofitting.
- *Cross-contract overlap with K4.* Both K4 and S5 reference
  ``_MIN_DURATION_S = 0.05 s``.  K4 enforces it on
  ``ReferenceEvent`` lists at the ``make_feasible_events`` boundary;
  S5 references it as a clamp at scheduler-internal segment
  construction.  These are consistent (the same constant, the same
  rationale), but worth a sentence in the K1–K6 expansion in Phase 7
  noting the cross-contract shared bound.
