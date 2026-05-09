# Scheduler Contract (S1–S6)

This document is the **normative specification** of the invariants that
``EventScheduler`` (see [controller/scheduler.py](scheduler.py)) — and any
future variant — must satisfy.  It is the structural sibling of the K1–K6
[REFERENCE_LAYER_CONTRACT.md](REFERENCE_LAYER_CONTRACT.md) and the
[HOT_LOOP_CONTRACT.md](HOT_LOOP_CONTRACT.md), and uses the same RFC 2119
normative language (MUST, MUST NOT, SHOULD, MAY).

It exists so that future schedulers (juggling choreography, GUI input
paths, multi-ball coordination, hardware-driven catch sequences) can be
implemented without reintroducing the failure modes that Phase 1 of the
[mpc-tier0-contracts plan](../plans/active/mpc-tier0-contracts.md)
identified.

## Background

The MPC's reference is computed each tick from the scheduler's
``SchedulerOutput`` — a ``TargetCommand`` plus optional hand notification.
The scheduler owns the *temporal* contract: when do events fire, in what
order, and with what state-machine transitions.  K1–K6 owns the
*kinematic* contract on the ``ReferenceEvent`` list the scheduler emits;
this contract owns the temporal/state-machine contract on the events
themselves.

Pre-contract, the scheduler accepted a class of malformed inputs without
diagnostic loudness:

- **Past-time events** were silently clamped to ``_MIN_DURATION_S = 0.05 s``
  (see [scheduler.py:791](scheduler.py)).  An event submitted with
  ``time < sim_time`` produced a 50 ms quintic to the event's pose, which
  is feasible kinematically (K2/K3 may pass) but is operationally
  incorrect — the event was supposed to arrive in the past, and the
  scheduler silently re-aimed at a near-future arrival.
- **Duplicate event IDs** routed to the "else" branch in the
  pre-Phase-2 ``submit_event`` body: if ``_current_event is not None``,
  the new event silently replaced ``_next_event`` even when its
  ``event_id`` matched the current event's ID — a caller intending
  to refine the current event via the wrong API silently corrupted
  the next-event slot.  That branch was removed in Phase 2; the
  post-Phase-2 ``submit_event`` body is at
  [scheduler.py:276–319](scheduler.py).
- **Three-deep submission** had no guard.  Today there are only two
  explicit slots (``_current_event``, ``_next_event``) plus one implicit
  return slot (``_seg_return``); a third ``submit_event`` overwrites
  ``_next_event`` without warning.
- **K2/K3 feasibility check was conditionally skipped** when
  ``v_max_mmps`` or ``tau_s`` was unset
  (see [_verify_segment_feasibility (scheduler.py:832)](scheduler.py)) —
  silent degradation rather than a documented test-only mode.
- **Phase-transition splices** (e.g., ``replace_next_event`` mid-
  TRANSITIONING) preserved C0 continuity by virtue of the
  ``_last_pose / _last_twist / _last_accel`` cache, but this was a
  side-effect of the implementation rather than a stated invariant.
- **Backward ``sim_time``** in ``update()`` was not detected.  A sim
  restart without ``clear()`` produced negative-duration segments and
  undefined arrival detection.

The S1–S6 invariants below close that whole class of failures.

## The Invariants

Every ``EventScheduler`` instance and every event submitted to it MUST
satisfy:

### S1 — Submission time bound

Every event submitted via ``submit_event`` or ``replace_next_event`` MUST
satisfy

    event.time >= last_sim_time - τ_grace

where ``last_sim_time`` is the most recent ``sim_time`` passed to
``update()`` (or ``-∞`` if no ``update()`` has been called yet) and
``τ_grace`` is a per-instance constructor parameter with a documented
default of **1 × control_dt** (one tick of clock skew).

Additionally, ``event.time`` MUST be finite — NaN and ±inf are
structural input-shape errors that MUST raise ``ValueError`` at
submission, regardless of whether ``update()`` has been called.
The same finiteness requirement applies to ``sim_time`` passed to
``update()``: a non-finite ``sim_time`` would silently disable S1
(every subsequent comparison evaluates to False), so ``update()``
MUST raise ``ValueError`` at entry on non-finite input.  The same
applies to ``τ_grace`` itself, which is checked at construction.

Violations MUST raise ``ValueError`` at submission, not silently degrade
to a clamped-duration segment.

**Why.** Past-time events are caller bugs: either a stale event was
re-submitted, or the caller's clock is misaligned with the scheduler's.
Silently clamping the segment duration to ``_MIN_DURATION_S`` (the pre-
contract behaviour at [scheduler.py:791](scheduler.py)) hides the bug
and produces a segment whose endpoint pose is at the *wrong absolute
time* — useful for nothing.  A loud raise forces the caller to either
fix the timestamp or explicitly use ``cancel_next`` / ``clear`` to
discard the stale event.

The ``τ_grace`` allowance accommodates one-tick clock-skew between the
event-source's time reference and the scheduler's; ``τ_grace = 0``
would make the contract brittle against benign timing jitter.

### S2 — Unique event IDs in flight

For every ``submit_event(event)`` call, ``event.event_id`` MUST NOT match
the ``event_id`` of either ``_current_event`` or ``_next_event``.
Duplicate-ID submissions MUST raise ``ValueError``.

The legitimate refinement paths are:

- ``update_current_event(event)`` — refines the current event in place;
  REQUIRES ``event.event_id == _current_event.event_id`` (already
  validated at [scheduler.py:331–336](scheduler.py)).
- ``replace_next_event(event)`` — replaces the next event; the new
  event's ID may differ from the old next event's ID, but MUST NOT match
  the current event's ID (would create an ambiguous event lifecycle).

**Why.** Pre-contract, ``submit_event`` with a duplicate ID routed
silently to the "else" branch (replacing ``_next_event`` rather than
raising) — a caller intending "please refine event 42" via the wrong
API would clobber the next-event slot with a duplicate of the current
event, producing two distinct events with the same ID in the in-flight
set.  That branch was removed in Phase 2; the post-Phase-2
``submit_event`` body at [scheduler.py:276–319](scheduler.py) raises
on duplicate IDs (and on slot saturation, per S3) before any state
mutation.  Hand-notification
consumers, ID-based cancellation, and operator log-grepping all break
when IDs are non-unique.  A raise forces the caller to choose the
right API.

### S3 — Bounded in-flight slot set

The scheduler's in-flight slot set is structurally bounded.  Two slots
hold ``ScheduledEvent`` references and one slot holds a
``_QuinticSegment``:

- At most **1** ``_current_event`` (``ScheduledEvent``: active or being approached)
- At most **1** ``_next_event`` (``ScheduledEvent``: lookahead beyond the current)
- At most **1** ``_seg_return`` (``_QuinticSegment``: implicit return to active pose; built directly when HOLDING→RETURNING)

A ``submit_event`` call when both ``_current_event`` and ``_next_event``
are non-None MUST raise ``ValueError``.  The caller is responsible for
``cancel_next`` / ``clear`` before submitting beyond the bounded set.

The ``RETURN_TO_ACTIVE`` event queued by ``begin_return``
([scheduler.py:689–698](scheduler.py)) when not in HOLDING is a
legitimate use of the ``_next_event`` slot and counts toward the bound.

**Why.** The structural design intent of the scheduler is "one active
target plus one lookahead."  Pre-contract, a third submission silently
overwrote ``_next_event``, hiding the queue-saturation condition from
the caller.  A loud raise surfaces it.  Future variants that need a
deeper queue MUST change S3 and update this document; silent expansion
of the in-flight slot set is a contract violation.

### S4 — Internal quintic feasibility

Every ``_QuinticSegment`` constructed by the scheduler MUST be verified
against K2 (peak velocity ≤ β · v_max) and K3 (peak acceleration
≤ β · v_max / τ) via
[_verify_segment_feasibility (scheduler.py:806)](scheduler.py).

Two enforcement modes:

- ``strict_feasibility=True`` — infeasibility raises ``ValueError``.
  Tests MUST construct schedulers in this mode.
- ``strict_feasibility=False`` — infeasibility logs ``WARNING`` and
  continues.  Production constructors MAY use this mode for graceful
  degradation; the MPC will see the infeasible reference and fall back
  via [W7's hardening (REFERENCE_LAYER_CONTRACT.md, "Stretch policy")](REFERENCE_LAYER_CONTRACT.md).

Schedulers constructed without ``v_max_mmps`` or ``tau_s`` (today: the
"S4 silently skipped" mode at [scheduler.py:832–833](scheduler.py)) MUST
log a single ``WARNING`` at construction stating "S4 unenforced" and
SHOULD only be used in unit tests that do not exercise feasibility.
Production callers MUST provide both parameters.

**Why.** Pre-contract, the feasibility check was conditional on the
``v_max_mmps``/``tau_s`` parameters being supplied; an oversight in the
constructor call silently disabled K2/K3 enforcement.  The contract
treats unsupplied parameters as a documented test-only degradation
rather than a silent skip.

S4 is *defense in depth*.  The K1–K6 contract on ``make_feasible_events``
already guarantees that **TargetSource-emitted** events satisfy K2/K3.
The scheduler builds **its own** quintics from event boundaries, which
could in principle violate K2/K3 if the event's ``time`` field is set
too tight relative to the inter-event pose delta.  S4 catches that
class of caller error.

Scope: S4's check is per-axis on the **linear** workspace components
(see existing scope discussion in [scheduler.py:816–825](scheduler.py)).
Per-leg-velocity enforcement on quintic refs is out of scope; the MPC
itself enforces leg-velocity bounds at every horizon node.

### S5 — Phase-transition C0 continuity

Every newly-built ``_QuinticSegment`` MUST start at the scheduler's
recorded live motion state at segment-build time:

    seg.p0 = self._last_pose
    seg.v0 = self._last_twist
    seg.a0 = self._last_accel

at the moment ``_build_segment_to_event`` is called.  This guarantees
C0 (and operationally C1, since ``v0`` matches the previous segment's
end velocity) at every segment splice — including:

- IDLE → APPROACHING (segment built from the live plant state)
- HOLDING → TRANSITIONING (segment built from ``_last_pose`` /
  ``_last_twist``, which were updated to the current event's pose at
  arrival)
- TRANSITIONING → HOLDING (live motion state IS the current event's
  pose at arrival)
- TRANSITIONING + ``replace_next_event`` (segment rebuilt from the
  live motion state at the moment of replacement)
- HOLDING → RETURNING (return segment built from ``_last_pose`` —
  the current event's pose)

Phase handlers MUST update ``self._last_pose`` / ``self._last_twist`` /
``self._last_accel`` to the **evaluated segment state** at every
``update()`` call.  ``_last_accel`` MUST come from
``seg.evaluate_full(t)``'s acceleration return — not be set to zero
mid-segment.

**Why.** Pre-contract, C0 continuity was preserved as a side-effect of
the segment-build pattern, not a stated invariant.  A future contributor
who refactors ``replace_next_event`` to build the new segment from the
new event's previous-event-end-pose rather than the live motion state
would silently introduce a position discontinuity at the splice — the
reference would step away from the plant's current state, violating
K1 (live-anchor) on the FIRST emitted event of the new segment.  S5
codifies the invariant that prevents that drift.

A subtle edge case: ``replace_next_event`` to a near-arrival event
(``event.time - sim_time < _MIN_DURATION_S``) is a caller-side timing
bug analogous to S1.  S1 catches it at submission with the configured
``τ_grace``; S5 covers the segment-build path for events that pass S1
but produce ``duration == _MIN_DURATION_S`` after the
``max(event.time - sim_time, _MIN_DURATION_S)`` clamp at
[scheduler.py:791](scheduler.py).  Such segments still satisfy S5
(they start at live state), but their **end** state is the event's
pose at ``sim_time + 0.05 s``, which is not the event's intended
arrival time.  Callers MUST treat S5-compliant-but-S1-clamped segments
as a degraded mode and SHOULD log when ``duration == _MIN_DURATION_S``.

### S6 — Monotonic sim_time

Every call to ``update(sim_time, ...)`` MUST satisfy

    sim_time >= self._last_sim_time

across consecutive calls.  Backward ``sim_time`` MUST raise
``ValueError`` with a message naming the violation:

    "scheduler sim_time went backward: got X, last seen Y; call clear()
    before reusing this scheduler instance"

The legitimate sim-restart workflow is:

    scheduler.clear()       # resets phase, events, and _last_sim_time
    scheduler.update(t0)    # accepted at any t0

**Why.** A scheduler with backward ``sim_time`` produces:

- Negative-duration segments via the ``event.time - sim_time``
  computation at [scheduler.py:791](scheduler.py); these clamp to
  ``_MIN_DURATION_S`` per S5's edge case but with no relationship to
  the original event's intended timing.
- Undefined arrival detection: the ``sim_time >= seg.t_end`` check
  at [:417](scheduler.py), [:510](scheduler.py), [:576](scheduler.py)
  may fire spuriously or miss arrivals depending on how far backward
  the time jumped.
- Hand-notification ordering corruption: notifications are tied to
  phase transitions which fire at arrival times.  A backward jump
  past an arrival re-fires it.

A loud raise is the correct behaviour because the violation is a
caller-side bug (sim restart without ``clear()``, monotonic-clock
wraparound, multi-process clock-source swap).  Silent degradation
would produce confusing downstream symptoms days after the actual
bug.

## Enforcement

All six invariants are enforced in the canonical implementation:
[controller/scheduler.py](scheduler.py).  The enforcement points are:

| Invariant | Enforcement point | Mechanism |
|-----------|-------------------|-----------|
| S1 | ``submit_event`` / ``replace_next_event`` (entry) | Compare ``event.time`` to ``_last_sim_time``; raise on violation |
| S2 | ``submit_event`` (entry) | Check ``event.event_id`` against ``_current_event.event_id`` and ``_next_event.event_id``; raise on collision |
| S3 | ``submit_event`` (entry) | Check ``(_current_event, _next_event)`` slot occupancy; raise when both occupied |
| S4 | ``_verify_segment_feasibility`` (called from ``_build_segment_to_event`` and the return-to-active path) | Strict mode: raise; lax mode: log; unsupplied params: log once at construction |
| S5 | ``_build_segment_to_event`` (segment construction) | Always reads ``self._last_pose`` / ``_twist`` / ``_accel`` |
| S6 | ``update`` (entry) | Compare ``sim_time`` to ``self._last_sim_time``; raise on violation; update at end |

The constructor accepts a ``τ_grace`` parameter (default ``1 × control_dt``,
itself a constructor parameter that will be aligned with the P4 invariant
in the forthcoming ``PLANT_INTERFACE_CONTRACT.md`` (Phase 4 of the
[mpc-tier0-contracts plan](../plans/active/mpc-tier0-contracts.md)).

## Implementing a new event source

An *event source* is any caller of ``submit_event`` /
``update_current_event`` / ``replace_next_event``.  Examples:

- ``sim/hand/scheduled_coordinator.py:147`` — sim catch coordinator
- A future GUI choreography driver
- A future multi-ball juggling planner

Event sources MUST:

1. **Honour S1.** Compute ``event.time`` against the same time-base
   the scheduler is observing.  If the source has its own clock, it
   MUST resolve clock-skew before submission, not at the scheduler.
2. **Honour S2.** Use ``_next_event_id()`` (or an equivalent monotonic
   counter) for new events.  Use ``update_current_event`` for refinements
   of the current event, NOT ``submit_event`` with the same ID.
3. **Honour S3.** Check the scheduler's ``active_event`` /
   ``next_event`` properties before submitting; cancel or clear if the
   in-flight slot set is full.
4. **Trust S4–S6.** Sources do not need to know about quintic
   feasibility, motion-state continuity, or clock monotonicity — those
   are scheduler-internal invariants.  Sources only need to provide
   well-formed ``ScheduledEvent`` objects.

Template:

```python
class MyEventSource:
    def __init__(self, scheduler: EventScheduler):
        self._scheduler = scheduler

    def submit(self, event_type, pose, arrival_time, twist=None):
        # Honour S3: check slot occupancy before submitting
        if (self._scheduler.active_event is not None and
            self._scheduler.next_event is not None):
            self._scheduler.cancel_next()  # or raise — your policy

        # Honour S2: fresh ID
        event = ScheduledEvent(
            event_id=_next_event_id(),
            event_type=event_type,
            pose=pose,
            time=arrival_time,    # honour S1 — must be in the future
            twist=twist,
        )
        # Honour S1: scheduler will raise on past-time events
        try:
            self._scheduler.submit_event(event)
        except ValueError as exc:
            logger.warning("submit rejected: %s", exc)
            # caller-specific recovery

    def refine(self, event_id, new_pose):
        # Honour S2: refine via the right API
        if self._scheduler.active_event is None:
            return
        if self._scheduler.active_event.event_id != event_id:
            return  # stale refinement
        refined = ScheduledEvent(
            event_id=event_id,
            event_type=self._scheduler.active_event.event_type,
            pose=new_pose,
            time=self._scheduler.active_event.time,
        )
        self._scheduler.update_current_event(refined)
```

## Diagnosis

If the scheduler raises one of the S1–S6 ``ValueError`` exceptions in a
session log:

1. **S1 violation** — the caller submitted a stale event.  Check the
   event source's clock alignment and the caller's freshness logic.
   Common causes: re-submitting a cached event after a long pause;
   submitting from a thread whose clock is offset from the scheduler's.
2. **S2 violation** — the caller used ``submit_event`` for a refinement.
   Switch to ``update_current_event`` for refinements of the current
   event, ``replace_next_event`` for the next event.
3. **S3 violation** — the caller saturated the in-flight slot set.  Either
   the source is producing events faster than the scheduler can consume
   them (genuine pipeline overflow — back-pressure needed), or a stale
   ``_next_event`` was never cancelled (cancellation-path bug in the
   source).
4. **S4 violation** — the caller's chosen ``event.time`` is too tight
   for the inter-event pose delta to satisfy K2/K3.  Either widen the
   arrival window or accept that this catch/throw is infeasible and
   abort upstream.  Check the scheduler's ``v_max_mmps`` / ``tau_s``
   construction parameters match the MPC's; mismatch is a config bug.
5. **S5 violation** — should be impossible from caller-side input;
   indicates a scheduler-internal bug.  File against the scheduler.
6. **S6 violation** — sim-restart workflow missing a ``clear()``, or
   wall-clock source change between calls.  Audit the caller's lifecycle
   for every ``update()`` invocation.

## Related

- [REFERENCE_LAYER_CONTRACT.md](REFERENCE_LAYER_CONTRACT.md) — K1–K6
  contract on ``ReferenceEvent`` lists.  Orthogonal to this contract:
  K1–K6 governs *what* events the MPC tracks, S1–S6 governs *when and
  in what order* they are submitted.  S4 is the single point of overlap.
- [HOT_LOOP_CONTRACT.md](HOT_LOOP_CONTRACT.md) — hot-loop allocation
  budget.  ``EventScheduler.update`` runs on the hot loop and MUST
  respect the budget; S1–S6 enforcement adds two integer comparisons
  and an ID-set membership check per ``submit_event`` (which is NOT on
  the hot loop) plus one float comparison per ``update`` (S6) which is.
- ``PLANT_INTERFACE_CONTRACT.md`` (forthcoming, Phase 4 of the
  [mpc-tier0-contracts plan](../plans/active/mpc-tier0-contracts.md)) —
  P4 ``control_dt`` parameter; this contract's ``τ_grace`` default
  will derive from it.
- [controller/scheduler.py](scheduler.py) — canonical implementation.
- [tests/sim/test_scheduler_contract.py](../tests/sim/test_scheduler_contract.py)
  — enforcement tests (Phase 2/3 of the
  [mpc-tier0-contracts plan](../plans/active/mpc-tier0-contracts.md)).
- [plans/active/mpc-tier0-contracts.md](../plans/active/mpc-tier0-contracts.md)
  — phased implementation plan; this document is the Phase 1 deliverable.
- [logbook/2026-05-09-scheduler-contract-phase-1-audit.md](../logbook/2026-05-09-scheduler-contract-phase-1-audit.md)
  — Phase 1 audit and contract draft.
