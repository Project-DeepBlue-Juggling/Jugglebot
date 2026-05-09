"""Event scheduler for C2-continuous catch/throw/return sequencing.

Manages an ordered sequence of events (catch, throw, return-to-active),
performs pairwise quintic Hermite chaining, and outputs MPC-compatible
``TargetCommand`` with ``ref_events`` each control step.

Supports:
- Live target refinement (continuous catch updates from ball tracker)
- Event replacement (next event can be swapped mid-motion)
- Graceful cancellation (decelerate to hold at current position)

This module has NO sim or ROS2 dependencies — pure Python + numpy.

Architecture
------------
The scheduler maintains at most two events: the *current* event being
approached and an optional *next* event for lookahead.  A quintic Hermite
segment connects the motion start state to the current event, and (if a
next event exists) a second segment connects the current event to the
next.  The MPC receives sampled ``ref_events`` spanning its horizon.

Phases::

    IDLE ──submit──▶ APPROACHING ──arrive──▶ HOLDING
      ▲                                        │
      │                              (next exists)
      │                                        ▼
      ◀──arrive──── RETURNING ◀──cancel── TRANSITIONING
"""

from __future__ import annotations

import itertools
import logging
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Optional

import numpy as np

from .hermite import quintic_interp, quintic_interp_with_accel
from .target import ReferenceEvent, TargetCommand

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------


class EventType(Enum):
    """Types of scheduled motion events."""
    CATCH = auto()
    THROW = auto()
    RETURN_TO_ACTIVE = auto()


class SchedulerPhase(Enum):
    """Scheduler state machine phases."""
    IDLE = auto()           # At active pose, no events queued
    APPROACHING = auto()    # Moving toward current event
    HOLDING = auto()        # At current event pose, waiting / dwelling
    TRANSITIONING = auto()  # Moving from current event to next event
    RETURNING = auto()      # Returning to active pose


@dataclass
class ScheduledEvent:
    """A single motion event (catch, throw, or return-to-active).

    Parameters
    ----------
    event_id : int
        Unique monotonic identifier for tracking and cancellation.
    event_type : EventType
        What kind of event this is.
    pose : (6,) ndarray
        Target platform pose [x, y, z, rx, ry, rz] in mm / rad.
    time : float
        Absolute arrival time (seconds, same clock as sim_time).
    twist : (6,) ndarray or None
        Desired twist at arrival.  None = zero (hold at pose).
    hand_vel_mps : float or None
        Ball speed for hand trajectory calculation.  Only meaningful
        for CATCH and THROW events.
    metadata : dict
        Opaque bag for caller use (ball_id, source, etc.).
    """
    event_id: int
    event_type: EventType
    pose: np.ndarray
    time: float
    twist: np.ndarray | None = None
    hand_vel_mps: float | None = None
    metadata: dict = field(default_factory=dict)


@dataclass
class HandNotification:
    """Notification from scheduler to hand coordinator about phase changes.

    The hand coordinator uses these to trigger priming, catch/throw
    sequences, etc.
    """
    event: ScheduledEvent
    notification_type: str    # 'approaching', 'arrived', 'leaving', 'cancelled'


@dataclass
class SchedulerOutput:
    """Output of a single scheduler update step.

    Contains everything the control loop needs: a ``TargetCommand`` for the
    MPC, optional hand notification, and scheduler state info.
    """
    target_command: TargetCommand
    hand_notification: HandNotification | None = None
    active_event: ScheduledEvent | None = None
    next_event: ScheduledEvent | None = None
    phase: SchedulerPhase = SchedulerPhase.IDLE


# ---------------------------------------------------------------------------
# Quintic segment
# ---------------------------------------------------------------------------

@dataclass
class _QuinticSegment:
    """A single quintic Hermite segment between two boundary states."""
    p0: np.ndarray
    v0: np.ndarray
    a0: np.ndarray
    p1: np.ndarray
    v1: np.ndarray
    a1: np.ndarray
    t_start: float    # absolute time at segment start
    duration: float   # segment duration in seconds

    @property
    def t_end(self) -> float:
        return self.t_start + self.duration

    def evaluate(self, t_abs: float) -> tuple[np.ndarray, np.ndarray]:
        """Evaluate pose and twist at absolute time *t_abs*."""
        t_local = t_abs - self.t_start
        return quintic_interp(
            self.p0, self.v0, self.a0,
            self.p1, self.v1, self.a1,
            self.duration, t_local,
        )

    def evaluate_full(self, t_abs: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Evaluate pose, twist, and acceleration at absolute time *t_abs*."""
        t_local = t_abs - self.t_start
        return quintic_interp_with_accel(
            self.p0, self.v0, self.a0,
            self.p1, self.v1, self.a1,
            self.duration, t_local,
        )


# ---------------------------------------------------------------------------
# EventScheduler
# ---------------------------------------------------------------------------

_id_counter = itertools.count()


def _next_event_id() -> int:
    return next(_id_counter)


# Minimum segment duration to avoid degenerate quintics
_MIN_DURATION_S = 0.05

# How close (mm position, rad orientation) before we consider "arrived"
_ARRIVAL_POS_TOL_MM = 5.0
_ARRIVAL_ROT_TOL_RAD = 0.02  # ~1.1 degrees


class EventScheduler:
    """Manages event sequencing with C2-continuous quintic Hermite chaining.

    Parameters
    ----------
    active_pose : (6,) ndarray
        The ACTIVE pose (geometric centre, all legs equal).  Used as the
        return destination when no events are queued.
    cumulative_times : (N+1,) ndarray
        MPC horizon node times from ``MPCParams.cumulative_times``.
        Used to sample ``ref_events`` at the correct time offsets.
    return_duration : float
        Default duration for return-to-active segments (seconds).
    """

    def __init__(
        self,
        active_pose: np.ndarray,
        cumulative_times: np.ndarray,
        return_duration: float = 2.0,
        v_max_mmps: float | None = None,
        tau_s: float | None = None,
        strict_feasibility: bool = False,
        tau_grace_s: float | None = None,
    ) -> None:
        self._active_pose = np.asarray(active_pose, dtype=np.float64)
        self._cumulative_times = np.asarray(cumulative_times, dtype=np.float64)
        self._return_duration = return_duration
        self._zero6 = np.zeros(6, dtype=np.float64)
        # W4: K2/K3 defense-in-depth.  When v_max_mmps + tau_s are provided,
        # each constructed segment is verified against K2/K3 per
        # controller/REFERENCE_LAYER_CONTRACT.md.  No auto-stretching — catch/toss
        # deadlines are hard, so infeasibility is a scheduler bug to surface.
        self._v_max_mmps = v_max_mmps
        self._tau_s = tau_s
        self._strict_feasibility = strict_feasibility

        # S1: clock-skew tolerance for submission-time check.  When None,
        # derive from cumulative_times[1] - [0] (one MPC tick).  See
        # controller/SCHEDULER_CONTRACT.md S1.
        if tau_grace_s is None:
            if len(self._cumulative_times) >= 2:
                tau_grace_s = float(
                    self._cumulative_times[1] - self._cumulative_times[0]
                )
            else:
                tau_grace_s = 0.025  # fallback for empty schedules
        if not np.isfinite(tau_grace_s) or tau_grace_s < 0:
            raise ValueError(
                f"tau_grace_s must be finite and >= 0, got {tau_grace_s}"
            )
        self._tau_grace_s = float(tau_grace_s)

        # State
        self._phase = SchedulerPhase.IDLE
        self._current_event: ScheduledEvent | None = None
        self._next_event: ScheduledEvent | None = None

        # Active quintic segments
        self._seg_current: _QuinticSegment | None = None   # toward current event
        self._seg_next: _QuinticSegment | None = None       # toward next event
        self._seg_return: _QuinticSegment | None = None     # toward active pose

        # Cached state at last update (for replan continuity)
        self._last_pose = self._active_pose.copy()
        self._last_twist = self._zero6.copy()
        self._last_accel = self._zero6.copy()

        # Pending hand notifications (consumed by caller after update)
        self._pending_notification: HandNotification | None = None

        # S1/S6 bookkeeping: most recent sim_time observed via update().
        # None until first update() — S1 disabled, per SCHEDULER_CONTRACT.md.
        self._last_sim_time: float | None = None

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def phase(self) -> SchedulerPhase:
        return self._phase

    @property
    def active_event(self) -> ScheduledEvent | None:
        return self._current_event

    @property
    def next_event(self) -> ScheduledEvent | None:
        return self._next_event

    # ------------------------------------------------------------------
    # Event management
    # ------------------------------------------------------------------

    def submit_event(self, event: ScheduledEvent) -> None:
        """Add an event to the schedule.

        If no current event exists, this becomes the current event and the
        scheduler begins approaching it.  If a current event exists (and
        the next slot is empty), this becomes the next event.

        Raises
        ------
        ValueError
            If the event violates S1 (past-time submission), S2 (event_id
            collides with an in-flight event), or S3 (both slots already
            occupied).  See controller/SCHEDULER_CONTRACT.md.
        """
        # Contract enforcement.  Order: S1 (input shape) → S2 (state
        # collision) → S3 (capacity).  Each helper raises ValueError on
        # violation; composition produces a single first-fault report.
        self._validate_submission_time(event, op_name='submit_event')   # S1
        self._validate_id_uniqueness(
            event, op_name='submit_event', check_next=True,
        )                                                                # S2
        self._validate_slot_capacity(op_name='submit_event')             # S3

        if self._current_event is None:
            self._current_event = event
            self._phase = SchedulerPhase.APPROACHING
            # Segment will be built on next update() when we have sim_time
            self._seg_current = None
            self._pending_notification = HandNotification(
                event=event, notification_type='approaching',
            )
            logger.info(
                "Scheduler: new current event %s id=%d at t=%.3f",
                event.event_type.name, event.event_id, event.time,
            )
        else:
            # S3 has already verified _next_event is None at this point,
            # so this branch is purely the "queue beyond current" case.
            self._next_event = event
            self._seg_next = None  # will be rebuilt
            logger.info(
                "Scheduler: queued next event %s id=%d at t=%.3f",
                event.event_type.name, event.event_id, event.time,
            )

    def update_current_event(self, event: ScheduledEvent) -> None:
        """Refine the current event (live catch tracking update).

        The event_id should match the current event.  The quintic segment
        is marked stale and will be recomputed from the current motion
        state on the next ``update()`` call.
        """
        if self._current_event is None:
            logger.warning("update_current_event called with no current event")
            return
        if event.event_id != self._current_event.event_id:
            logger.warning(
                "update_current_event: id mismatch (got %d, expected %d)",
                event.event_id, self._current_event.event_id,
            )
            return
        self._current_event = event
        self._seg_current = None  # force replan on next update()

    def replace_next_event(self, event: ScheduledEvent) -> None:
        """Replace the next event.

        If currently transitioning toward the old next event, the segment
        is replanned from the current motion state to the new event.

        Raises
        ------
        ValueError
            If the event violates S1 (past-time submission) or partial S2
            (event_id matches the current event).  Replacement is allowed
            to reuse the OLD next event's id (it's a refinement of the
            same slot); collision is only checked against ``_current_event``.
            See controller/SCHEDULER_CONTRACT.md.
        """
        # S1 + partial S2 (current only — next is being replaced).  No
        # S3 check: replace doesn't grow the slot set.
        self._validate_submission_time(event, op_name='replace_next_event')
        self._validate_id_uniqueness(
            event, op_name='replace_next_event', check_next=False,
        )

        old = self._next_event
        self._next_event = event
        self._seg_next = None  # force rebuild

        if self._phase == SchedulerPhase.TRANSITIONING:
            # Mid-transition: will replan from current state on next update()
            logger.info(
                "Scheduler: mid-transition replacement, old id=%s new id=%d",
                old.event_id if old else None, event.event_id,
            )

    def cancel_next(self) -> ScheduledEvent | None:
        """Cancel the next event.  Returns the cancelled event or None."""
        cancelled = self._next_event
        self._next_event = None
        self._seg_next = None
        return cancelled

    def clear(self) -> None:
        """Cancel all events and return to IDLE at current position."""
        self._current_event = None
        self._next_event = None
        self._seg_current = None
        self._seg_next = None
        self._seg_return = None
        self._phase = SchedulerPhase.IDLE

    # ------------------------------------------------------------------
    # Per-step update
    # ------------------------------------------------------------------

    def update(
        self,
        sim_time: float,
        current_pose: np.ndarray,
        current_twist: np.ndarray,
    ) -> SchedulerOutput:
        """Advance scheduler state and produce MPC target for this step.

        Called at MPC rate (typically 40 Hz).

        Parameters
        ----------
        sim_time : float
            Current simulation / wall-clock time.
        current_pose : (6,) ndarray
            Current platform pose from plant state.
        current_twist : (6,) ndarray
            Current platform twist from plant state.

        Returns
        -------
        SchedulerOutput
            Contains ``TargetCommand`` for MPC plus scheduler metadata.
        """
        # S1/S6 bookkeeping: record the most recent observed sim_time so
        # subsequent submit_event/replace_next_event calls can validate
        # against it (S1).  Phase 3 will add the S6 monotonicity check
        # before this assignment; for Phase 2 we only track.  Reject
        # NaN / ±inf here — a poisoned _last_sim_time silently disables
        # S1 because every subsequent comparison evaluates to False.
        if not np.isfinite(sim_time):
            raise ValueError(
                f"sim_time must be finite, got {sim_time!r}"
            )
        self._last_sim_time = float(sim_time)

        # Don't clear _pending_notification here — submit_event() may have
        # set one that should be delivered on this update.  Each handler
        # either overwrites it or leaves the existing value.

        if self._phase == SchedulerPhase.IDLE:
            return self._update_idle(sim_time, current_pose, current_twist)
        elif self._phase == SchedulerPhase.APPROACHING:
            return self._update_approaching(sim_time, current_pose, current_twist)
        elif self._phase == SchedulerPhase.HOLDING:
            return self._update_holding(sim_time, current_pose, current_twist)
        elif self._phase == SchedulerPhase.TRANSITIONING:
            return self._update_transitioning(sim_time, current_pose, current_twist)
        elif self._phase == SchedulerPhase.RETURNING:
            return self._update_returning(sim_time, current_pose, current_twist)
        else:
            # Should never happen
            return self._make_hold_output(current_pose)

    # ------------------------------------------------------------------
    # Phase handlers
    # ------------------------------------------------------------------

    def _update_idle(
        self, sim_time: float, pose: np.ndarray, twist: np.ndarray,
    ) -> SchedulerOutput:
        """IDLE: hold at active pose.  If an event was submitted, we
        already transitioned to APPROACHING in submit_event()."""
        self._last_pose = pose.copy()
        self._last_twist = twist.copy()
        self._last_accel = self._zero6.copy()
        return self._make_hold_output(self._active_pose)

    def _update_approaching(
        self, sim_time: float, pose: np.ndarray, twist: np.ndarray,
    ) -> SchedulerOutput:
        """APPROACHING: moving toward current event via quintic segment."""
        event = self._current_event
        assert event is not None

        # Build segment if needed (first call or after replan)
        if self._seg_current is None:
            self._seg_current = self._build_segment_to_event(
                sim_time, pose, twist, event,
            )

        seg = self._seg_current

        # Check if we've arrived (past the segment end time)
        if sim_time >= seg.t_end:
            self._phase = SchedulerPhase.HOLDING
            self._last_pose = event.pose.copy()
            self._last_twist = (
                event.twist.copy() if event.twist is not None else self._zero6.copy()
            )
            self._last_accel = self._zero6.copy()
            self._pending_notification = HandNotification(
                event=event, notification_type='arrived',
            )
            logger.info(
                "Scheduler: arrived at event id=%d, entering HOLDING",
                event.event_id,
            )
            return self._update_holding(sim_time, pose, twist)

        # Evaluate quintic and build ref_events
        ref_pose, ref_twist = seg.evaluate(sim_time)
        p, v, a = seg.evaluate_full(sim_time)
        self._last_pose = p.copy()
        self._last_twist = v.copy()
        self._last_accel = a.copy()

        ref_events = self._sample_segments(sim_time, [seg])

        return SchedulerOutput(
            target_command=TargetCommand(
                target_pose=ref_pose,
                arrival_time=event.time,
                target_twist=event.twist,
                ref_events=ref_events,
            ),
            hand_notification=self._pop_notification(),
            active_event=self._current_event,
            next_event=self._next_event,
            phase=self._phase,
        )

    def _update_holding(
        self, sim_time: float, pose: np.ndarray, twist: np.ndarray,
    ) -> SchedulerOutput:
        """HOLDING: at current event pose.  Advance to next or return."""
        event = self._current_event
        assert event is not None

        # If next event exists, transition to it
        if self._next_event is not None:
            self._pending_notification = HandNotification(
                event=event, notification_type='leaving',
            )
            self._phase = SchedulerPhase.TRANSITIONING
            self._seg_next = None  # build on next update
            logger.info(
                "Scheduler: HOLDING → TRANSITIONING toward event id=%d",
                self._next_event.event_id,
            )
            return self._update_transitioning(sim_time, pose, twist)

        # No next event: hold at current event pose (MPC tracks it ASAP)
        self._last_pose = event.pose.copy()
        self._last_twist = (
            event.twist.copy() if event.twist is not None else self._zero6.copy()
        )
        self._last_accel = self._zero6.copy()

        return SchedulerOutput(
            target_command=TargetCommand(
                target_pose=event.pose,
                target_twist=event.twist,
            ),
            hand_notification=self._pop_notification(),
            active_event=self._current_event,
            next_event=self._next_event,
            phase=self._phase,
        )

    def _update_transitioning(
        self, sim_time: float, pose: np.ndarray, twist: np.ndarray,
    ) -> SchedulerOutput:
        """TRANSITIONING: moving from current event to next event."""
        next_ev = self._next_event
        assert next_ev is not None

        # Build segment if needed
        if self._seg_next is None:
            # Start from current motion state (handles mid-transition replan)
            self._seg_next = self._build_segment_to_event(
                sim_time, self._last_pose, self._last_twist, next_ev,
            )

        seg = self._seg_next

        # Check arrival
        if sim_time >= seg.t_end:
            # Promote next → current
            old_current = self._current_event
            self._current_event = next_ev
            self._next_event = None
            self._seg_current = None
            self._seg_next = None
            self._phase = SchedulerPhase.HOLDING
            self._last_pose = next_ev.pose.copy()
            self._last_twist = (
                next_ev.twist.copy() if next_ev.twist is not None else self._zero6.copy()
            )
            self._last_accel = self._zero6.copy()
            self._pending_notification = HandNotification(
                event=next_ev, notification_type='arrived',
            )
            logger.info(
                "Scheduler: arrived at next event id=%d, promoting to current",
                next_ev.event_id,
            )
            return self._update_holding(sim_time, pose, twist)

        # Evaluate
        ref_pose, ref_twist = seg.evaluate(sim_time)
        p, v, a = seg.evaluate_full(sim_time)
        self._last_pose = p.copy()
        self._last_twist = v.copy()
        self._last_accel = a.copy()

        ref_events = self._sample_segments(sim_time, [seg])

        return SchedulerOutput(
            target_command=TargetCommand(
                target_pose=ref_pose,
                arrival_time=next_ev.time,
                target_twist=next_ev.twist,
                ref_events=ref_events,
            ),
            hand_notification=self._pop_notification(),
            active_event=self._current_event,
            next_event=self._next_event,
            phase=self._phase,
        )

    def _update_returning(
        self, sim_time: float, pose: np.ndarray, twist: np.ndarray,
    ) -> SchedulerOutput:
        """RETURNING: moving back to active pose."""
        if self._seg_return is None:
            # Build return segment from current state
            duration = self._return_duration
            self._seg_return = _QuinticSegment(
                p0=self._last_pose.copy(),
                v0=self._last_twist.copy(),
                a0=self._last_accel.copy(),
                p1=self._active_pose.copy(),
                v1=self._zero6.copy(),
                a1=self._zero6.copy(),
                t_start=sim_time,
                duration=max(duration, _MIN_DURATION_S),
            )
            self._verify_segment_feasibility(self._seg_return)

        seg = self._seg_return

        # Check arrival
        if sim_time >= seg.t_end:
            self._current_event = None
            self._seg_return = None
            self._phase = SchedulerPhase.IDLE
            self._last_pose = self._active_pose.copy()
            self._last_twist = self._zero6.copy()
            self._last_accel = self._zero6.copy()
            logger.info("Scheduler: returned to active pose, entering IDLE")
            return self._make_hold_output(self._active_pose)

        ref_pose, ref_twist = seg.evaluate(sim_time)
        p, v, a = seg.evaluate_full(sim_time)
        self._last_pose = p.copy()
        self._last_twist = v.copy()
        self._last_accel = a.copy()

        ref_events = self._sample_segments(sim_time, [seg])

        return SchedulerOutput(
            target_command=TargetCommand(
                target_pose=ref_pose,
                ref_events=ref_events,
            ),
            hand_notification=self._pop_notification(),
            active_event=None,
            next_event=None,
            phase=self._phase,
        )

    # ------------------------------------------------------------------
    # Trigger return to active
    # ------------------------------------------------------------------

    def begin_return(self) -> None:
        """Signal the scheduler to return to active pose.

        Call this after a catch is confirmed, hand retraction completes,
        etc.  If called during HOLDING with no next event, begins the
        return.  If called during other phases, queues a RETURN_TO_ACTIVE
        as the next event.
        """
        if self._phase == SchedulerPhase.HOLDING and self._next_event is None:
            self._phase = SchedulerPhase.RETURNING
            self._seg_return = None  # build on next update
            self._pending_notification = HandNotification(
                event=self._current_event,
                notification_type='leaving',
            ) if self._current_event else None
            self._current_event = None
            logger.info("Scheduler: HOLDING → RETURNING to active pose")
        elif self._phase == SchedulerPhase.IDLE:
            pass  # already at active
        else:
            # Queue a return as next event
            ret_event = ScheduledEvent(
                event_id=_next_event_id(),
                event_type=EventType.RETURN_TO_ACTIVE,
                pose=self._active_pose.copy(),
                time=0.0,  # will be computed when segment is built
                twist=None,
            )
            self._next_event = ret_event
            self._seg_next = None

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    # ---- S1/S2/S3 input-domain validation ----
    # See controller/SCHEDULER_CONTRACT.md for normative spec.

    def _validate_submission_time(
        self, event: ScheduledEvent, *, op_name: str,
    ) -> None:
        """S1 enforcement.  Raises ``ValueError`` if event.time is non-
        finite (NaN / ±inf), or more than ``_tau_grace_s`` in the past
        relative to the most recent ``sim_time`` observed via
        ``update()``.

        The finiteness check is unconditional — NaN / inf are
        structural input-shape errors that MUST be rejected even
        before the first ``update()`` fires.  The clock-skew
        comparison is no-op until ``update()`` has been called at
        least once (``_last_sim_time is None``).
        """
        if not np.isfinite(event.time):
            raise ValueError(
                f"S1 violation in {op_name}: event.time={event.time!r} "
                f"is not finite.  Reject NaN / ±inf at submission to "
                f"prevent downstream NaN propagation into MPC targets."
            )
        if self._last_sim_time is None:
            return  # S1 disabled until first update — see contract S1
        threshold = self._last_sim_time - self._tau_grace_s
        if event.time < threshold:
            raise ValueError(
                f"S1 violation in {op_name}: event.time={event.time:.6f} "
                f"< last_sim_time - tau_grace_s = "
                f"{self._last_sim_time:.6f} - {self._tau_grace_s:.6f} = "
                f"{threshold:.6f}.  Past-time events are caller bugs; "
                f"cancel/clear stale events before resubmitting."
            )

    def _validate_id_uniqueness(
        self, event: ScheduledEvent, *, op_name: str, check_next: bool,
    ) -> None:
        """S2 enforcement.  Raises ``ValueError`` if ``event.event_id``
        matches an in-flight event's id.

        For ``submit_event`` (``check_next=True``), collision is checked
        against both ``_current_event`` and ``_next_event``.  For
        ``replace_next_event`` (``check_next=False``), only
        ``_current_event`` is checked — a replacement may legitimately
        reuse the OLD next event's id (it's a refinement of the same
        slot).  See contract S2.
        """
        if (self._current_event is not None
                and self._current_event.event_id == event.event_id):
            raise ValueError(
                f"S2 violation in {op_name}: event.event_id="
                f"{event.event_id} matches the current event's id.  "
                f"Use update_current_event() for refinements; "
                f"submit_event/replace_next_event are for distinct events."
            )
        if check_next and self._next_event is not None and (
                self._next_event.event_id == event.event_id):
            raise ValueError(
                f"S2 violation in {op_name}: event.event_id="
                f"{event.event_id} matches the next event's id.  "
                f"Use replace_next_event() with a fresh id, or "
                f"cancel_next() before reusing the id."
            )

    def _validate_slot_capacity(self, *, op_name: str) -> None:
        """S3 enforcement.  Raises ``ValueError`` if both
        ``_current_event`` and ``_next_event`` are already occupied.
        See contract S3.
        """
        if (self._current_event is not None
                and self._next_event is not None):
            raise ValueError(
                f"S3 violation in {op_name}: both _current_event "
                f"(id={self._current_event.event_id}) and _next_event "
                f"(id={self._next_event.event_id}) are occupied.  Call "
                f"cancel_next() or clear() before queuing more events."
            )

    def _build_segment_to_event(
        self,
        sim_time: float,
        start_pose: np.ndarray,
        start_twist: np.ndarray,
        event: ScheduledEvent,
    ) -> _QuinticSegment:
        """Build a quintic segment from current state to event."""
        duration = max(event.time - sim_time, _MIN_DURATION_S)
        end_twist = event.twist if event.twist is not None else self._zero6
        seg = _QuinticSegment(
            p0=start_pose.copy(),
            v0=start_twist.copy(),
            a0=self._last_accel.copy(),
            p1=event.pose.copy(),
            v1=end_twist.copy(),
            a1=self._zero6.copy(),  # zero accel at event boundary
            t_start=sim_time,
            duration=duration,
        )
        self._verify_segment_feasibility(seg)
        return seg

    def _verify_segment_feasibility(self, seg: _QuinticSegment) -> None:
        """Defense-in-depth K2/K3 check on scheduler-built quintic segments.

        The scheduler's internal quintic construction is BELIEVED feasible
        because event durations are computed from distance/v_max ratios.
        This method catches any deviation from that assumption.  No
        auto-stretching — the catch/toss coordinator has hard arrival
        deadlines and silent stretching would make the platform miss the
        ball.  Infeasibility is a scheduler bug to surface, not hide.

        Scope: this is a *linear-axis* (workspace x/y/z) check via
        ``segment_is_feasible``.  It catches translation-dominated K2/K3
        violations that any reasonable Stewart-platform Jacobian condition
        would also fail at the leg level.  Angular twists routed through
        the Jacobian (which couples translation and rotation into per-leg
        velocities) can still produce per-leg velocities exceeding
        ``v_max_mmps`` even when the linear check passes.  Per-leg-velocity
        enforcement on quintic refs would require sampling the segment in
        joint space, which is out of scope for this defensive check; the
        MPC itself enforces leg-velocity bounds at every horizon node.

        When ``strict_feasibility=True`` (tests), infeasibility raises.
        When False (production), it logs a WARNING and continues — the MPC
        will see the infeasible ref and degrade gracefully via W7's fallback
        hardening, but operators will want to know this happened.
        """
        if self._v_max_mmps is None or self._tau_s is None:
            return
        from .feasibility import segment_is_feasible
        feas, vr, ar = segment_is_feasible(
            seg.p0, seg.v0, seg.a0, seg.p1, seg.v1, seg.a1,
            seg.duration, self._v_max_mmps,
            self._v_max_mmps / self._tau_s, beta=0.85,
        )
        if not feas:
            msg = (
                f"EventScheduler: segment violates K2/K3 "
                f"(vr={vr:.2f} ar={ar:.2f}) — duration={seg.duration:.3f}s, "
                f"p0→p1 Δ={np.linalg.norm(seg.p1[:3] - seg.p0[:3]):.2f}mm, "
                f"|v0|={np.linalg.norm(seg.v0[:3]):.2f}, "
                f"|v1|={np.linalg.norm(seg.v1[:3]):.2f} mm/s"
            )
            if self._strict_feasibility:
                raise ValueError(msg)
            logger.warning(msg)

    def _sample_segments(
        self,
        sim_time: float,
        segments: list[_QuinticSegment],
    ) -> list[ReferenceEvent]:
        """Sample quintic segments at MPC horizon node times."""
        events: list[ReferenceEvent] = []
        for dt in self._cumulative_times:
            t_abs = sim_time + dt
            # Find the segment containing this time
            pose, twist, accel = None, None, None
            for seg in segments:
                if t_abs <= seg.t_end or seg is segments[-1]:
                    pose, twist, accel = seg.evaluate_full(t_abs)
                    break
            assert pose is not None
            events.append(ReferenceEvent(
                time=t_abs,
                pose=pose,
                twist=twist,
                accel=accel,
            ))
        return events

    def _pop_notification(self) -> HandNotification | None:
        """Consume and return the pending notification (if any)."""
        n = self._pending_notification
        self._pending_notification = None
        return n

    def _make_hold_output(self, pose: np.ndarray) -> SchedulerOutput:
        """Output for holding at a static pose (IDLE or end-of-return)."""
        return SchedulerOutput(
            target_command=TargetCommand(
                target_pose=pose.copy(),
            ),
            hand_notification=self._pop_notification(),
            active_event=self._current_event,
            next_event=self._next_event,
            phase=self._phase,
        )
