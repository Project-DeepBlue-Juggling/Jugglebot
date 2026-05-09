"""Phase 2 enforcement tests for SCHEDULER_CONTRACT.md S1, S2, S3.

These cover the input-domain invariants checked at ``submit_event`` /
``replace_next_event`` entry:

  * S1 — submission time bound (event.time >= last_sim_time − τ_grace)
  * S2 — unique event_id across in-flight events
  * S3 — bounded in-flight slot set (≤ 2 ScheduledEvent slots filled)

Phase 3 (forthcoming) will extend this file with hypothesis
``RuleBasedStateMachine`` invariants for S4–S6.
"""

from __future__ import annotations

import numpy as np
import pytest

from controller.scheduler import (
    EventScheduler,
    EventType,
    ScheduledEvent,
    _next_event_id,
)

# ---------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------

ACTIVE_POSE = np.zeros(6)
_DT = 0.025  # 40 Hz
_CUMULATIVE_TIMES = np.array([_DT * i for i in range(11)])  # 10 nodes


def _scheduler(**kwargs) -> EventScheduler:
    """Construct a scheduler with default fixture values, overridable."""
    return EventScheduler(
        active_pose=ACTIVE_POSE,
        cumulative_times=_CUMULATIVE_TIMES,
        **kwargs,
    )


def _event(
    time: float, *, event_id: int | None = None,
    pose: np.ndarray | None = None,
) -> ScheduledEvent:
    """Construct a minimal ScheduledEvent.  event_id auto-assigned if None."""
    if event_id is None:
        event_id = _next_event_id()
    if pose is None:
        pose = np.array([50.0, 0.0, 20.0, 0.0, 0.0, 0.0])
    return ScheduledEvent(
        event_id=event_id,
        event_type=EventType.CATCH,
        pose=np.asarray(pose, dtype=np.float64),
        time=float(time),
    )


# ---------------------------------------------------------------------
# Constructor — tau_grace_s
# ---------------------------------------------------------------------

class TestTauGraceConstruction:
    """T-U-S1-3 (constructor leg) — tau_grace_s parameter handling."""

    def test_default_tau_grace_derives_from_cumulative_times(self):
        """tau_grace_s=None → cumulative_times[1] − [0] (one MPC tick)."""
        sched = _scheduler()  # no override
        assert sched._tau_grace_s == pytest.approx(_DT)

    def test_explicit_tau_grace_overrides_default(self):
        sched = _scheduler(tau_grace_s=0.1)
        assert sched._tau_grace_s == 0.1

    def test_zero_tau_grace_accepted(self):
        """τ_grace = 0 makes S1 strict (no clock-skew tolerance).
        Permitted; primarily for tests."""
        sched = _scheduler(tau_grace_s=0.0)
        assert sched._tau_grace_s == 0.0

    def test_negative_tau_grace_rejected(self):
        with pytest.raises(ValueError, match="tau_grace_s"):
            _scheduler(tau_grace_s=-0.001)

    def test_nan_tau_grace_rejected(self):
        """NaN tau_grace_s would silently disable S1 (every comparison
        evaluates to False).  Constructor must reject.  Regression
        guard for the NaN-poison vector via cumulative_times input."""
        with pytest.raises(ValueError, match="tau_grace_s"):
            _scheduler(tau_grace_s=float('nan'))

    def test_default_when_cumulative_times_too_short(self):
        """Empty / single-element cumulative_times → fallback to 0.025."""
        sched = EventScheduler(
            active_pose=ACTIVE_POSE,
            cumulative_times=np.array([0.0]),
        )
        assert sched._tau_grace_s == pytest.approx(0.025)


# ---------------------------------------------------------------------
# S1 — Submission time bound
# ---------------------------------------------------------------------

class TestS1SubmissionTime:
    """T-U-S1-1, T-U-S1-2 — past-time submission enforcement."""

    def test_submission_before_first_update_unconstrained(self):
        """Until update() has been called, _last_sim_time is None and
        S1 cannot be evaluated.  Past-time events must be accepted —
        the contract makes this explicit (S1 is no-op until first update).
        """
        sched = _scheduler()
        assert sched._last_sim_time is None
        # Negative time, no update yet — must NOT raise
        sched.submit_event(_event(time=-100.0))

    def test_past_time_event_raises_after_update(self):
        sched = _scheduler(tau_grace_s=0.0)  # strict
        sched.update(1.0, ACTIVE_POSE, np.zeros(6))
        with pytest.raises(ValueError, match="S1 violation"):
            sched.submit_event(_event(time=0.5))

    def test_event_at_grace_boundary_accepted(self):
        """event.time == last_sim_time − τ_grace must be accepted
        (boundary is inclusive — strict-less-than in the check)."""
        sched = _scheduler(tau_grace_s=_DT)
        sched.update(1.0, ACTIVE_POSE, np.zeros(6))
        # Boundary: 1.0 − 0.025 = 0.975
        sched.submit_event(_event(time=0.975))

    def test_event_just_past_grace_raises(self):
        """One epsilon below the boundary triggers S1."""
        sched = _scheduler(tau_grace_s=_DT)
        sched.update(1.0, ACTIVE_POSE, np.zeros(6))
        # 0.975 − 1e-6 = 0.974999, just past the boundary
        with pytest.raises(ValueError, match="S1 violation"):
            sched.submit_event(_event(time=0.974999))

    def test_future_event_after_update_accepted(self):
        sched = _scheduler(tau_grace_s=_DT)
        sched.update(1.0, ACTIVE_POSE, np.zeros(6))
        sched.submit_event(_event(time=2.0))

    def test_replace_next_validates_s1(self):
        """S1 applies to replace_next_event too (per contract)."""
        sched = _scheduler(tau_grace_s=0.0)
        sched.submit_event(_event(time=1.0))
        sched.submit_event(_event(time=2.0))
        sched.update(1.5, ACTIVE_POSE, np.zeros(6))
        # 0.5 < 1.5 − 0 → S1 violation
        with pytest.raises(ValueError, match="S1 violation"):
            sched.replace_next_event(_event(time=0.5))

    def test_update_current_event_unaffected_by_s1(self):
        """update_current_event is OUTSIDE S1's domain (refinement of an
        already-in-flight event).  Past-time refinements are clamped at
        segment-build time, not raised at submission."""
        sched = _scheduler(tau_grace_s=0.0)
        ev = _event(time=2.0, event_id=42)
        sched.submit_event(ev)
        sched.update(1.5, ACTIVE_POSE, np.zeros(6))
        # update_current_event with a past-time refinement: no S1 raise
        refined = _event(time=0.5, event_id=42)
        sched.update_current_event(refined)
        assert sched.active_event.time == 0.5

    def test_s1_message_contains_diagnostic_values(self):
        """Error message must surface event.time, last_sim_time, and τ_grace
        so a debugging operator can immediately see the offset."""
        sched = _scheduler(tau_grace_s=0.05)
        sched.update(2.5, ACTIVE_POSE, np.zeros(6))
        with pytest.raises(ValueError) as excinfo:
            sched.submit_event(_event(time=1.0))
        msg = str(excinfo.value)
        assert "S1 violation" in msg
        assert "submit_event" in msg
        assert "1.0" in msg or "1.000000" in msg
        assert "2.5" in msg or "2.500000" in msg
        assert "0.05" in msg or "0.050000" in msg

    # --- NaN / ±inf regression guards (post-implementation audit) ---

    def test_nan_event_time_raises(self):
        """NaN event.time must raise S1 ValueError.  Pre-fix, NaN
        passed S1 because ``nan < threshold`` is always False — the
        event was installed and NaN propagated into TargetCommand."""
        sched = _scheduler()
        sched.update(1.0, ACTIVE_POSE, np.zeros(6))
        with pytest.raises(ValueError, match="S1 violation"):
            sched.submit_event(_event(time=float('nan')))

    def test_positive_inf_event_time_raises(self):
        sched = _scheduler()
        sched.update(1.0, ACTIVE_POSE, np.zeros(6))
        with pytest.raises(ValueError, match="S1 violation"):
            sched.submit_event(_event(time=float('inf')))

    def test_negative_inf_event_time_raises(self):
        sched = _scheduler()
        sched.update(1.0, ACTIVE_POSE, np.zeros(6))
        with pytest.raises(ValueError, match="S1 violation"):
            sched.submit_event(_event(time=float('-inf')))

    def test_nan_event_time_raises_before_first_update(self):
        """The finiteness check fires BEFORE the _last_sim_time-is-None
        early-return.  NaN is a structural input-shape error, not a
        clock-skew problem, so it must be rejected even in the
        bootstrap window before the first update()."""
        sched = _scheduler()
        assert sched._last_sim_time is None
        with pytest.raises(ValueError, match="S1 violation"):
            sched.submit_event(_event(time=float('nan')))

    def test_nan_sim_time_rejected_at_update(self):
        """NaN sim_time would poison _last_sim_time and silently
        disable S1 for all future calls (every subsequent comparison
        evaluates to False).  update() must reject at the boundary."""
        sched = _scheduler()
        with pytest.raises(ValueError, match="sim_time must be finite"):
            sched.update(float('nan'), ACTIVE_POSE, np.zeros(6))
        # State must be unchanged after the failed update
        assert sched._last_sim_time is None

    def test_inf_sim_time_rejected_at_update(self):
        sched = _scheduler()
        with pytest.raises(ValueError, match="sim_time must be finite"):
            sched.update(float('inf'), ACTIVE_POSE, np.zeros(6))
        assert sched._last_sim_time is None


# ---------------------------------------------------------------------
# S2 — Unique event IDs
# ---------------------------------------------------------------------

class TestS2UniqueIds:
    """T-U-S2-1, T-U-S2-2 — id-collision enforcement."""

    def test_submit_with_current_id_raises(self):
        sched = _scheduler()
        sched.submit_event(_event(time=1.0, event_id=42))
        with pytest.raises(ValueError, match="S2 violation"):
            sched.submit_event(_event(time=2.0, event_id=42))

    def test_submit_with_next_id_raises(self):
        """When _next_event already holds id N, submit_event with id=N
        must raise — even though the slot for it would be a third-deep
        position.  S2 fires before S3 has a chance to."""
        sched = _scheduler()
        sched.submit_event(_event(time=1.0, event_id=10))
        sched.submit_event(_event(time=2.0, event_id=20))
        # Slots: current=10, next=20.  Submit id=20 — collides with next.
        with pytest.raises(ValueError, match="S2 violation"):
            sched.submit_event(_event(time=3.0, event_id=20))

    def test_submit_with_unique_id_accepted(self):
        sched = _scheduler()
        sched.submit_event(_event(time=1.0, event_id=10))
        sched.submit_event(_event(time=2.0, event_id=20))
        # Both slots filled with distinct IDs: no S2 violation.

    def test_replace_next_with_current_id_raises(self):
        """replace_next_event with an id matching the CURRENT event must
        raise (partial S2 — current-only, since next is being replaced)."""
        sched = _scheduler()
        sched.submit_event(_event(time=1.0, event_id=42))
        sched.submit_event(_event(time=2.0, event_id=99))
        # Try to replace next with id=42 — collides with current
        with pytest.raises(ValueError, match="S2 violation"):
            sched.replace_next_event(_event(time=3.0, event_id=42))

    def test_replace_next_with_old_next_id_accepted(self):
        """Reusing the OLD next event's id in replace_next_event is the
        documented same-slot refinement pattern; must NOT raise."""
        sched = _scheduler()
        sched.submit_event(_event(time=1.0, event_id=42))
        sched.submit_event(_event(time=2.0, event_id=99))
        # Replace next, reusing id=99 — same-slot refinement
        sched.replace_next_event(_event(time=2.5, event_id=99))
        assert sched.next_event.event_id == 99
        assert sched.next_event.time == 2.5

    def test_replace_next_with_fresh_id_accepted(self):
        sched = _scheduler()
        sched.submit_event(_event(time=1.0, event_id=42))
        sched.submit_event(_event(time=2.0, event_id=99))
        sched.replace_next_event(_event(time=2.5, event_id=100))
        assert sched.next_event.event_id == 100

    def test_update_current_with_matching_id_accepted(self):
        """update_current_event REQUIRES the id match — S2 doesn't apply
        (it's the legitimate refinement path)."""
        sched = _scheduler()
        ev = _event(time=1.0, event_id=42)
        sched.submit_event(ev)
        refined = _event(time=1.05, event_id=42)
        sched.update_current_event(refined)
        assert sched.active_event.event_id == 42
        assert sched.active_event.time == 1.05

    def test_s2_message_contains_diagnostic_values(self):
        sched = _scheduler()
        sched.submit_event(_event(time=1.0, event_id=42))
        with pytest.raises(ValueError) as excinfo:
            sched.submit_event(_event(time=2.0, event_id=42))
        msg = str(excinfo.value)
        assert "S2 violation" in msg
        assert "42" in msg
        assert "submit_event" in msg


# ---------------------------------------------------------------------
# S3 — Bounded slot set
# ---------------------------------------------------------------------

class TestS3BoundedSlots:
    """T-U-S3-1 — slot-saturation enforcement."""

    def test_two_submissions_accepted(self):
        sched = _scheduler()
        sched.submit_event(_event(time=1.0))
        sched.submit_event(_event(time=2.0))
        assert sched.active_event is not None
        assert sched.next_event is not None

    def test_third_submission_raises(self):
        sched = _scheduler()
        sched.submit_event(_event(time=1.0))
        sched.submit_event(_event(time=2.0))
        with pytest.raises(ValueError, match="S3 violation"):
            sched.submit_event(_event(time=3.0))

    def test_third_submission_does_not_mutate_state(self):
        """A rejected submit_event must NOT have replaced or modified
        any in-flight event.  This is the critical safety property of
        S3 — it converts a previously-silent overwrite into a loud no-op.
        """
        sched = _scheduler()
        ev1 = _event(time=1.0, event_id=10)
        ev2 = _event(time=2.0, event_id=20)
        sched.submit_event(ev1)
        sched.submit_event(ev2)
        with pytest.raises(ValueError):
            sched.submit_event(_event(time=3.0, event_id=30))
        # Slots unchanged
        assert sched.active_event.event_id == 10
        assert sched.next_event.event_id == 20

    def test_after_cancel_next_third_submit_accepted(self):
        sched = _scheduler()
        sched.submit_event(_event(time=1.0, event_id=10))
        sched.submit_event(_event(time=2.0, event_id=20))
        sched.cancel_next()
        # next slot is now free; third submit_event allowed
        sched.submit_event(_event(time=3.0, event_id=30))
        assert sched.next_event.event_id == 30

    def test_after_clear_third_submit_accepted(self):
        sched = _scheduler()
        sched.submit_event(_event(time=1.0, event_id=10))
        sched.submit_event(_event(time=2.0, event_id=20))
        sched.clear()
        # Both slots free; submit goes to current
        sched.submit_event(_event(time=3.0, event_id=30))
        assert sched.active_event.event_id == 30
        assert sched.next_event is None

    def test_replace_next_does_not_check_s3(self):
        """replace_next_event substitutes for the existing next event;
        slot count doesn't increase, so S3 is not relevant."""
        sched = _scheduler()
        sched.submit_event(_event(time=1.0))
        sched.submit_event(_event(time=2.0))
        # Both slots filled — replace is OK (replaces, doesn't add)
        sched.replace_next_event(_event(time=2.5))

    def test_s3_message_contains_diagnostic_ids(self):
        sched = _scheduler()
        sched.submit_event(_event(time=1.0, event_id=10))
        sched.submit_event(_event(time=2.0, event_id=20))
        with pytest.raises(ValueError) as excinfo:
            sched.submit_event(_event(time=3.0, event_id=30))
        msg = str(excinfo.value)
        assert "S3 violation" in msg
        assert "10" in msg and "20" in msg
        assert "submit_event" in msg


# ---------------------------------------------------------------------
# Validation order — S1 → S2 → S3
# ---------------------------------------------------------------------

class TestValidationOrder:
    """When multiple invariants would be violated by the same event,
    the error reported MUST be the first in the order S1 → S2 → S3.

    Rationale: S1 is the input-shape check (time is well-formed); S2 is
    the state-collision check (id doesn't conflict); S3 is the capacity
    check (slot space available).  Reporting in this order gives the
    caller the most fundamental violation first."""

    def test_s1_takes_precedence_over_s2(self):
        """Past-time event whose id ALSO collides with current."""
        sched = _scheduler(tau_grace_s=0.0)
        sched.submit_event(_event(time=2.0, event_id=42))
        sched.update(3.0, ACTIVE_POSE, np.zeros(6))
        # event.time=1.0 (past) AND event_id=42 (collides)
        with pytest.raises(ValueError, match="S1 violation"):
            sched.submit_event(_event(time=1.0, event_id=42))

    def test_s2_takes_precedence_over_s3(self):
        """Slot-full event whose id ALSO collides."""
        sched = _scheduler()
        sched.submit_event(_event(time=1.0, event_id=10))
        sched.submit_event(_event(time=2.0, event_id=20))
        # Both slots full AND id=10 collides with current → S2 first
        with pytest.raises(ValueError, match="S2 violation"):
            sched.submit_event(_event(time=3.0, event_id=10))

    def test_s1_takes_precedence_over_s3(self):
        """Past-time event submitted while slots are full."""
        sched = _scheduler(tau_grace_s=0.0)
        sched.submit_event(_event(time=2.0, event_id=10))
        sched.submit_event(_event(time=3.0, event_id=20))
        sched.update(4.0, ACTIVE_POSE, np.zeros(6))
        # event.time=1.0 (past) AND slots full → S1 first
        with pytest.raises(ValueError, match="S1 violation"):
            sched.submit_event(_event(time=1.0, event_id=30))


# ---------------------------------------------------------------------
# Behaviour-change regression: previously-silent overwrite now raises
# ---------------------------------------------------------------------

class TestBehaviourChangeRegression:
    """Pre-Phase-2, ``submit_event`` with both slots full silently
    overwrote ``_next_event``.  Phase 2 makes this raise.  These tests
    pin the new behaviour so a future refactor cannot silently re-
    introduce the overwrite path.
    """

    def test_pre_contract_silent_overwrite_now_raises(self):
        """The legacy "silent replace" path on a full slot set is gone."""
        sched = _scheduler()
        sched.submit_event(_event(time=1.0, event_id=1))
        sched.submit_event(_event(time=2.0, event_id=2))
        # Pre-Phase-2: this would silently set _next_event = id=3.
        # Phase 2: must raise (S3 — slot capacity).
        with pytest.raises(ValueError, match="S3 violation"):
            sched.submit_event(_event(time=3.0, event_id=3))
        # Verify state is exactly as it was before the failed submit
        assert sched.next_event.event_id == 2
        assert sched.next_event.time == 2.0

    def test_replace_next_is_the_documented_replacement_path(self):
        """The contract directs callers wanting to swap the next event
        to use replace_next_event explicitly.  That path is preserved."""
        sched = _scheduler()
        sched.submit_event(_event(time=1.0, event_id=1))
        sched.submit_event(_event(time=2.0, event_id=2))
        sched.replace_next_event(_event(time=2.5, event_id=3))
        assert sched.next_event.event_id == 3
        assert sched.next_event.time == 2.5


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
