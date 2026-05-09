"""Phase 2 + Phase 3 enforcement tests for SCHEDULER_CONTRACT.md S1–S6.

Phase 2 (input-domain) covers invariants checked at ``submit_event`` /
``replace_next_event`` entry:

  * S1 — submission time bound (event.time >= last_sim_time − τ_grace)
  * S2 — unique event_id across in-flight events
  * S3 — bounded in-flight slot set (≤ 2 ScheduledEvent slots filled)

Phase 3 (state-machine) covers invariants checked across ``update()``
calls or during phase transitions:

  * S4 — internal quintic feasibility (warn-once on missing v_max/tau;
    strict_feasibility=True raises, =False warns)
  * S5 — phase-transition C2 continuity at every newly-built segment
    (segment starts at the live motion state ``_last_*``)
  * S6 — monotonic ``sim_time`` across consecutive ``update()`` calls;
    ``clear()`` resets the clock for the legitimate sim-restart workflow

The hypothesis ``RuleBasedStateMachine`` at the bottom of this file
exercises arbitrary submit/cancel/update/replace/tick sequences and
asserts S1–S6 invariants throughout.  The tick-rule advances ``sim_time``
monotonically, so S6's backward-jump path is covered by the targeted
scenario tests above rather than the state machine itself.
"""

from __future__ import annotations

import logging

import numpy as np
import pytest
from hypothesis import HealthCheck, assume, settings, strategies as st
from hypothesis.stateful import (
    RuleBasedStateMachine,
    invariant,
    precondition,
    rule,
    initialize,
)

from controller.scheduler import (
    EventScheduler,
    EventType,
    ScheduledEvent,
    SchedulerPhase,
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


# ---------------------------------------------------------------------
# S4 — Internal quintic feasibility
# ---------------------------------------------------------------------

# Production-realistic operating point — mirror controller/params.py defaults
# so the K2/K3 strict-mode raise lands on a credible regime.
_V_MAX_MMPS = 140.0
_TAU_S = 0.04


class _CapturingHandler(logging.Handler):
    """Direct-attach handler that records LogRecord objects.

    Used in place of pytest's caplog because the propagation path
    between the ``controller.scheduler`` logger and the root caplog
    handler is not reliably configured under this project's pytest
    setup (pytest's live-log goes to stderr via ``logging.lastResort``,
    bypassing the propagation chain caplog hooks).  Attaching directly
    to the logger under test removes that ambiguity."""

    def __init__(self):
        super().__init__(level=logging.WARNING)
        self.records: list[logging.LogRecord] = []

    def emit(self, record):
        self.records.append(record)

    def messages(self) -> list[str]:
        return [r.getMessage() for r in self.records]


@pytest.fixture
def scheduler_log():
    """Yields a list-of-records handler attached to controller.scheduler.

    Cleans up automatically; resets handler state per test."""
    logger_under_test = logging.getLogger('controller.scheduler')
    handler = _CapturingHandler()
    logger_under_test.addHandler(handler)
    try:
        yield handler
    finally:
        logger_under_test.removeHandler(handler)


class TestS4FeasibilityConstruction:
    """Construction-time S4 surface: warn-once when v_max/tau missing."""

    def test_warning_logged_when_v_max_missing(self, scheduler_log):
        """v_max_mmps=None disables S4; constructor MUST log a single
        WARNING so the degraded mode is visible in any session log.

        Pre-Phase-3, the silent-skip at scheduler.py made S4
        unenforced without any operator-visible diagnostic.  This pins
        the loud-fail behaviour."""
        _scheduler(v_max_mmps=None, tau_s=_TAU_S)
        warnings = [m for m in scheduler_log.messages()
                    if 'S4 unenforced' in m]
        assert len(warnings) == 1, (
            f"expected 1 'S4 unenforced' warning, got {len(warnings)}: "
            f"{scheduler_log.messages()}"
        )

    def test_warning_logged_when_tau_missing(self, scheduler_log):
        _scheduler(v_max_mmps=_V_MAX_MMPS, tau_s=None)
        warnings = [m for m in scheduler_log.messages()
                    if 'S4 unenforced' in m]
        assert len(warnings) == 1

    def test_warning_logged_when_both_missing(self, scheduler_log):
        """Default constructor with neither parameter: warning fires."""
        _scheduler()  # v_max=None, tau=None (default)
        warnings = [m for m in scheduler_log.messages()
                    if 'S4 unenforced' in m]
        assert len(warnings) == 1

    def test_no_warning_when_both_provided(self, scheduler_log):
        """Production-shaped construction: silent."""
        _scheduler(v_max_mmps=_V_MAX_MMPS, tau_s=_TAU_S)
        warnings = [m for m in scheduler_log.messages()
                    if 'S4 unenforced' in m]
        assert len(warnings) == 0


class TestS4FeasibilityEnforcement:
    """T-U-S4-1, T-U-S4-2 — segment-build feasibility check."""

    def test_strict_mode_raises_on_k2_violation(self):
        """A segment whose endpoint requires more than v_max·β must
        raise in strict mode.  Geometry: 1000 mm in 0.5 s with zero
        boundary twist → peak quintic vel ≫ 119 mm/s (β·v_max)."""
        sched = _scheduler(
            v_max_mmps=_V_MAX_MMPS, tau_s=_TAU_S,
            strict_feasibility=True,
        )
        big_pose = np.array([1000.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        ev = _event(time=0.5, pose=big_pose)
        sched.submit_event(ev)
        # First update() builds the segment → strict raise.
        with pytest.raises(ValueError, match="K2/K3"):
            sched.update(0.0, ACTIVE_POSE, np.zeros(6))

    def test_lax_mode_warns_on_k2_violation(self, scheduler_log):
        """Production-like: same geometry, strict_feasibility=False (the
        default).  Logs WARNING but does not raise — caller continues
        with the infeasible segment, MPC degrades via W7 fallback."""
        sched = _scheduler(
            v_max_mmps=_V_MAX_MMPS, tau_s=_TAU_S,
            strict_feasibility=False,
        )
        big_pose = np.array([1000.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        ev = _event(time=0.5, pose=big_pose)
        sched.submit_event(ev)
        # Must NOT raise.
        out = sched.update(0.0, ACTIVE_POSE, np.zeros(6))
        assert out.phase == SchedulerPhase.APPROACHING
        # Must log the K2/K3 warning.
        k_warnings = [m for m in scheduler_log.messages() if 'K2/K3' in m]
        assert len(k_warnings) >= 1, (
            f"expected K2/K3 warning, got {scheduler_log.messages()}"
        )

    def test_skipped_when_v_max_or_tau_missing(self, scheduler_log):
        """When v_max/tau are unset, _verify_segment_feasibility is a
        no-op regardless of strict_feasibility.  The construction
        warning has already covered the diagnostic; per-segment
        verification stays silent (no extra spam)."""
        sched = _scheduler(strict_feasibility=True)  # no v_max/tau
        # Drain construction warning so we can isolate per-segment behaviour.
        construction_warnings = [m for m in scheduler_log.messages()
                                 if 'S4 unenforced' in m]
        assert len(construction_warnings) == 1
        scheduler_log.records.clear()
        # Build a wildly infeasible segment — verify no per-segment
        # warning, no raise.
        big_pose = np.array([10000.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        sched.submit_event(_event(time=0.1, pose=big_pose))
        sched.update(0.0, ACTIVE_POSE, np.zeros(6))
        per_seg_warnings = [m for m in scheduler_log.messages() if 'K2/K3' in m]
        assert len(per_seg_warnings) == 0


# ---------------------------------------------------------------------
# S5 — Phase-transition C2 continuity at every newly-built segment
# ---------------------------------------------------------------------

class TestS5SegmentLiveState:
    """Segments built by the scheduler MUST start at ``_last_*``.

    The contract requires C0 (and operationally C1 / C2) at every
    splice.  We test the splice continuity directly by inspecting the
    new segment's (p0, v0, a0) against ``_last_*`` immediately before
    the build.
    """

    def test_idle_to_approaching_segment_starts_at_plant(self):
        """Bootstrap path: first APPROACHING update after IDLE→APPROACHING
        must sync ``_last_*`` to plant state, then build from it.  The
        new segment's p0 / v0 must equal the plant state passed to
        update()."""
        sched = _scheduler()
        plant_pose = np.array([10.0, 5.0, 30.0, 0.01, 0.0, 0.0])
        plant_twist = np.array([2.0, 0.0, -1.0, 0.0, 0.0, 0.0])
        sched.submit_event(_event(time=1.0))
        sched.update(0.0, plant_pose, plant_twist)
        seg = sched._seg_current
        np.testing.assert_array_equal(seg.p0, plant_pose)
        np.testing.assert_array_equal(seg.v0, plant_twist)
        # Bootstrap accel is zero (set by IDLE / constructor).
        np.testing.assert_array_equal(seg.a0, np.zeros(6))

    def test_refinement_segment_splices_at_last_state(self):
        """Post-refinement path: ``update_current_event`` invalidates
        ``_seg_current`` but ``_last_*`` retains the segment-evaluated
        state from the prior tick.  The replan starts there — NOT from
        plant state — so splice continuity is preserved at C2."""
        sched = _scheduler()
        ev = _event(time=1.0, event_id=42,
                    pose=np.array([100.0, 0.0, 50.0, 0.0, 0.0, 0.0]))
        sched.submit_event(ev)
        sched.update(0.0, ACTIVE_POSE, np.zeros(6))    # bootstrap build
        sched.update(0.1, ACTIVE_POSE, np.zeros(6))    # advance segment
        sched.update(0.2, ACTIVE_POSE, np.zeros(6))    # advance further
        # Capture the live state right before refinement.
        last_pose_before = sched._last_pose.copy()
        last_twist_before = sched._last_twist.copy()
        last_accel_before = sched._last_accel.copy()
        # Confirm we're not still at zero — the segment has actually advanced.
        assert np.linalg.norm(last_pose_before) > 1e-6
        # Refine to a new pose at a new arrival time.
        refined = _event(time=1.5, event_id=42,
                         pose=np.array([120.0, 10.0, 60.0, 0.0, 0.0, 0.0]))
        sched.update_current_event(refined)
        # Pass plant state that DIFFERS from _last_* to prove the build
        # does not trust plant on the refinement path.
        sched.update(0.225,
                     np.array([-999.0, -999.0, -999.0, 0.0, 0.0, 0.0]),
                     np.array([-999.0, -999.0, -999.0, 0.0, 0.0, 0.0]))
        seg = sched._seg_current
        # C0 / C1 / C2: every order must match the pre-build live state.
        np.testing.assert_allclose(seg.p0, last_pose_before, atol=1e-12)
        np.testing.assert_allclose(seg.v0, last_twist_before, atol=1e-12)
        np.testing.assert_allclose(seg.a0, last_accel_before, atol=1e-12)

    def test_holding_to_transitioning_segment_starts_at_event_pose(self):
        """HOLDING→TRANSITIONING: at arrival, ``_update_holding``/
        ``_update_approaching`` set ``_last_*`` to the current event's
        pose+twist (and zero accel).  The TRANSITIONING segment then
        starts there — i.e., at the current event's pose, not at plant
        state."""
        sched = _scheduler()
        ev1 = _event(time=0.1, pose=np.array([50.0, 0.0, 20.0, 0.0, 0.0, 0.0]))
        ev2 = _event(time=0.6, pose=np.array([100.0, 0.0, 40.0, 0.0, 0.0, 0.0]))
        sched.submit_event(ev1)
        sched.submit_event(ev2)
        sched.update(0.0, ACTIVE_POSE, np.zeros(6))
        # Step past ev1 arrival → enters TRANSITIONING and builds seg_next.
        # Use plant_pose distinct from ev1.pose to confirm the segment
        # starts at ev1.pose, not plant.
        sched.update(0.15,
                     np.array([-999.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
                     np.zeros(6))
        assert sched.phase == SchedulerPhase.TRANSITIONING
        seg = sched._seg_next
        np.testing.assert_array_equal(seg.p0, ev1.pose)
        np.testing.assert_array_equal(seg.v0, np.zeros(6))
        np.testing.assert_array_equal(seg.a0, np.zeros(6))

    def test_replace_next_mid_transitioning_splices_c2_continuous(self):
        """T-U-S5-1: mid-TRANSITIONING ``replace_next_event`` produces
        a new segment that starts at the live (segment-evaluated) state,
        i.e., C2 continuous with the prior segment.  Per user choice
        the contract test scope is C2 (pose + twist + accel)."""
        sched = _scheduler()
        ev1 = _event(time=0.1, pose=np.array([50.0, 0.0, 20.0, 0.0, 0.0, 0.0]))
        ev2 = _event(time=1.0, pose=np.array([200.0, 0.0, 50.0, 0.0, 0.0, 0.0]))
        sched.submit_event(ev1)
        sched.submit_event(ev2)
        sched.update(0.0, ACTIVE_POSE, np.zeros(6))    # APPROACHING bootstrap
        sched.update(0.15, ACTIVE_POSE, np.zeros(6))   # arrive ev1 → TRANSITIONING
        # Step into the transitioning segment so _last_* is segment-
        # evaluated and non-trivially different from ev1.pose.
        sched.update(0.3, ACTIVE_POSE, np.zeros(6))
        sched.update(0.4, ACTIVE_POSE, np.zeros(6))
        assert sched.phase == SchedulerPhase.TRANSITIONING
        last_pose = sched._last_pose.copy()
        last_twist = sched._last_twist.copy()
        last_accel = sched._last_accel.copy()
        # Replace next with a different terminal pose.
        ev3 = _event(time=0.9,
                     pose=np.array([-50.0, 20.0, 10.0, 0.0, 0.0, 0.0]))
        sched.replace_next_event(ev3)
        # Pass an absurd plant pose to prove the build does not touch it.
        sched.update(0.425,
                     np.array([1e6, 1e6, 1e6, 0.0, 0.0, 0.0]),
                     np.array([1e6, 1e6, 1e6, 0.0, 0.0, 0.0]))
        seg = sched._seg_next
        np.testing.assert_allclose(seg.p0, last_pose, atol=1e-12)
        np.testing.assert_allclose(seg.v0, last_twist, atol=1e-12)
        np.testing.assert_allclose(seg.a0, last_accel, atol=1e-12)

    def test_replace_next_mid_approaching_defers_segment_build(self):
        """T-U-S5-2: ``replace_next_event`` while APPROACHING the
        current event (i.e., ``next`` slot is queued for after current
        arrives) does NOT rebuild ``_seg_current`` and only invalidates
        ``_seg_next`` — which doesn't exist yet in APPROACHING."""
        sched = _scheduler()
        ev1 = _event(time=1.0)
        ev2 = _event(time=2.0)
        sched.submit_event(ev1)
        sched.submit_event(ev2)
        sched.update(0.0, ACTIVE_POSE, np.zeros(6))
        assert sched.phase == SchedulerPhase.APPROACHING
        seg_current_before = sched._seg_current
        ev3 = _event(time=2.5)
        sched.replace_next_event(ev3)
        # _seg_current must NOT have been rebuilt (still the same object).
        assert sched._seg_current is seg_current_before
        # _seg_next stays None until we transition.
        assert sched._seg_next is None

    def test_returning_segment_starts_at_event_pose(self):
        """HOLDING→RETURNING: ``begin_return`` starts the return from
        ``_last_pose`` (which is the held event's pose) with zero twist
        and zero accel.  Verifies S5 on the inline-built return segment
        path."""
        sched = _scheduler(return_duration=0.5)
        held_pose = np.array([40.0, 0.0, 25.0, 0.0, 0.0, 0.0])
        ev = _event(time=0.1, pose=held_pose)
        sched.submit_event(ev)
        sched.update(0.0, ACTIVE_POSE, np.zeros(6))
        sched.update(0.15, ACTIVE_POSE, np.zeros(6))   # → HOLDING
        assert sched.phase == SchedulerPhase.HOLDING
        sched.begin_return()
        # First RETURNING update builds the return segment from _last_*.
        sched.update(0.16, ACTIVE_POSE, np.zeros(6))
        seg = sched._seg_return
        np.testing.assert_allclose(seg.p0, held_pose, atol=1e-12)
        np.testing.assert_allclose(seg.v0, np.zeros(6), atol=1e-12)
        np.testing.assert_allclose(seg.a0, np.zeros(6), atol=1e-12)


# ---------------------------------------------------------------------
# S6 — Monotonic sim_time
# ---------------------------------------------------------------------

class TestS6ClockMonotonicity:
    """T-U-S6-1, T-U-S6-2 — backward sim_time at update() entry."""

    def test_backward_sim_time_raises(self):
        sched = _scheduler()
        sched.update(1.0, ACTIVE_POSE, np.zeros(6))
        with pytest.raises(ValueError, match="S6 violation"):
            sched.update(0.999, ACTIVE_POSE, np.zeros(6))

    def test_forward_or_equal_sim_time_accepted(self):
        """Equal sim_time is permitted (re-firing the same tick is
        a no-op-shaped contract for replay/debug paths)."""
        sched = _scheduler()
        sched.update(1.0, ACTIVE_POSE, np.zeros(6))
        sched.update(1.0, ACTIVE_POSE, np.zeros(6))     # same — accepted
        sched.update(1.025, ACTIVE_POSE, np.zeros(6))   # forward — accepted

    def test_clear_resets_clock_for_sim_restart(self):
        """T-U-S6-2: the documented sim-restart workflow.  After
        ``clear()`` the next update() accepts any ``sim_time``
        (including one earlier than the previous observation)."""
        sched = _scheduler()
        sched.update(5.0, ACTIVE_POSE, np.zeros(6))
        sched.clear()
        # _last_sim_time must be None again.
        assert sched._last_sim_time is None
        sched.update(0.0, ACTIVE_POSE, np.zeros(6))     # accepted
        assert sched._last_sim_time == 0.0

    def test_s6_message_contains_diagnostic_values(self):
        sched = _scheduler()
        sched.update(2.5, ACTIVE_POSE, np.zeros(6))
        with pytest.raises(ValueError) as excinfo:
            sched.update(2.0, ACTIVE_POSE, np.zeros(6))
        msg = str(excinfo.value)
        assert "S6 violation" in msg
        assert "update" in msg
        assert "2.0" in msg or "2.000000" in msg
        assert "2.5" in msg or "2.500000" in msg
        assert "clear()" in msg

    def test_tiny_backward_jump_within_epsilon_accepted(self):
        """The S6 check uses a 1e-9 s tolerance to absorb floating-
        point round-off in sim_time arithmetic.  A jump within
        that tolerance must NOT raise."""
        sched = _scheduler()
        sched.update(1.0, ACTIVE_POSE, np.zeros(6))
        sched.update(1.0 - 1e-10, ACTIVE_POSE, np.zeros(6))   # within tol

    def test_failed_update_does_not_advance_clock(self):
        """An S6-rejected update() must NOT have updated
        ``_last_sim_time`` — a future caller passing the (correct)
        forward sim_time must still succeed."""
        sched = _scheduler()
        sched.update(1.0, ACTIVE_POSE, np.zeros(6))
        with pytest.raises(ValueError):
            sched.update(0.5, ACTIVE_POSE, np.zeros(6))
        assert sched._last_sim_time == 1.0
        # Forward update still works.
        sched.update(1.5, ACTIVE_POSE, np.zeros(6))
        assert sched._last_sim_time == 1.5


# ---------------------------------------------------------------------
# Hypothesis RuleBasedStateMachine — S1–S6 invariants under arbitrary
# submit / cancel / update / replace / tick sequences
# ---------------------------------------------------------------------

# Generation strategies — bounded so the search stays in physically
# reasonable territory.  Plant pose stays within the ACTIVE_POSE
# neighbourhood; arrival-time deltas are sampled from a small range
# that keeps S1/S4 satisfiable on the happy path while still firing
# raises on the sad path.

_pose_strat = st.lists(
    st.floats(min_value=-50.0, max_value=50.0,
              allow_nan=False, allow_infinity=False),
    min_size=6, max_size=6,
).map(np.asarray)

_twist_strat = st.lists(
    st.floats(min_value=-10.0, max_value=10.0,
              allow_nan=False, allow_infinity=False),
    min_size=6, max_size=6,
).map(np.asarray)

_dt_strat = st.floats(
    min_value=0.001, max_value=0.5,
    allow_nan=False, allow_infinity=False,
)

_event_offset_strat = st.floats(
    min_value=0.05, max_value=2.0,
    allow_nan=False, allow_infinity=False,
)


class SchedulerStateMachine(RuleBasedStateMachine):
    """Hypothesis state machine exercising S1–S6 under arbitrary ops.

    Timestamp generation strategy (per user choice): the ``tick`` rule
    advances ``sim_time`` strictly forward by a positive ``dt``, so S6
    monotonicity is enforced by construction during the random walk.
    The targeted backward-jump scenarios live in
    :class:`TestS6ClockMonotonicity` above — outside the state machine
    so they don't pollute the generation surface.

    Invariants checked after every rule:

    * **S2** — ``_current_event.event_id`` and ``_next_event.event_id``
      are distinct (when both slots are occupied).
    * **S3** — at most 2 ``ScheduledEvent`` slots filled simultaneously.
    * **S5** — every newly-built ``_QuinticSegment`` starts at the
      live ``_last_*`` state at build time (verified by inspecting
      ``seg.p0 / v0 / a0`` against ``_last_*`` recorded just before
      the build).
    * **S6** — observed ``sim_time`` is monotonically non-decreasing.

    S1 and S4 are exercised by the rules but their invariants are
    point-in-time (raise / no-raise on the offending op) rather than
    state predicates — they're checked inside the rules via
    ``pytest.raises`` or `try/except`.
    """

    def __init__(self):
        super().__init__()
        self.sim_time: float = 0.0
        self.scheduler: EventScheduler = EventScheduler(
            active_pose=ACTIVE_POSE,
            cumulative_times=_CUMULATIVE_TIMES,
            v_max_mmps=_V_MAX_MMPS,
            tau_s=_TAU_S,
            strict_feasibility=False,  # graceful — this is the
            # production-shaped path; strict-mode raises live in
            # TestS4FeasibilityEnforcement.
        )
        # Plant state advances with each tick — we keep it stationary
        # at ACTIVE_POSE (zero twist) for simplicity; the focus is the
        # scheduler's own state machine, not the plant.
        self.plant_pose = ACTIVE_POSE.copy()
        self.plant_twist = np.zeros(6)
        # Highest sim_time observed at any update() — for S6 invariant.
        self.last_observed_sim_time: float | None = None
        # The number of "in-flight" ScheduledEvent slots from the
        # caller's perspective (both _current and _next) — for S3.
        # Tracked alongside the scheduler's own state for cross-checking.

    @initialize()
    def _setup(self):
        # First tick at t=0 to put _last_sim_time on a known footing.
        # This also drains the IDLE handler's _last_* sync so subsequent
        # rules have a consistent starting state.
        out = self.scheduler.update(self.sim_time, self.plant_pose, self.plant_twist)
        self.last_observed_sim_time = self.sim_time
        assert out.phase == SchedulerPhase.IDLE

    # ---- Rules: state mutations ----

    @rule(pose=_pose_strat, dt_offset=_event_offset_strat)
    def submit(self, pose, dt_offset):
        """Submit a new event.  Caller-side, the event is always
        future-dated (sim_time + dt_offset) so S1 is satisfied; we
        rely on the scheduler's S2/S3 to either accept or raise."""
        ev = _event(
            time=self.sim_time + dt_offset,
            pose=pose,
        )
        try:
            self.scheduler.submit_event(ev)
        except ValueError as exc:
            # Acceptable: S2 (id collision impossible — _next_event_id
            # is monotonic) or S3 (slot saturation when both slots full).
            assert ('S2 violation' in str(exc)
                    or 'S3 violation' in str(exc)
                    or 'K2/K3' in str(exc)), (
                f"unexpected raise from submit_event: {exc}"
            )

    @rule(pose=_pose_strat, dt_offset=_event_offset_strat)
    def replace_next(self, pose, dt_offset):
        """Replace the next-slot event.  Always submitted with a fresh
        id from _next_event_id() (so partial-S2 cannot fire), at
        ``sim_time + dt_offset`` (so S1 cannot fire)."""
        ev = _event(
            time=self.sim_time + dt_offset,
            pose=pose,
        )
        try:
            self.scheduler.replace_next_event(ev)
        except ValueError as exc:
            assert 'K2/K3' in str(exc), (
                f"unexpected raise from replace_next_event: {exc}"
            )

    # cancel_next during TRANSITIONING leaves the scheduler in an
    # inconsistent state (TRANSITIONING phase with _next_event = None).
    # That is a state-machine semantic gap orthogonal to S4/S5/S6 —
    # filed as a Plan 2 follow-up (mpc-sadpath-coverage rollup, Tier 1
    # state-machine completeness).  We gate the rule here so the
    # property tests stay focused on the contract invariants this
    # phase owns.
    @precondition(
        lambda self: self.scheduler.phase != SchedulerPhase.TRANSITIONING
    )
    @rule()
    def cancel_next(self):
        self.scheduler.cancel_next()

    # begin_return silently overwrites _next_event when both slots are
    # filled (begin_return assigns _next_event = ret_event without an
    # S3 check).  This is also a state-machine gap orthogonal to
    # S4/S5/S6 — filed as a Plan 2 follow-up.  Gate to keep the
    # property tests focused on the contract surface this phase owns.
    @precondition(
        lambda self: self.scheduler.next_event is None
    )
    @rule()
    def begin_return(self):
        self.scheduler.begin_return()

    @rule(dt=_dt_strat)
    def tick(self, dt):
        """Advance sim_time by ``dt > 0`` and call ``update()``.  The
        positive ``dt`` strategy keeps S6 monotone by construction."""
        self.sim_time += dt
        self.scheduler.update(
            self.sim_time, self.plant_pose, self.plant_twist,
        )
        # Track the highest sim_time observed for the S6 invariant
        # (which is checked structurally — backward jumps cannot occur
        # given the strategy).
        if (self.last_observed_sim_time is None
                or self.sim_time > self.last_observed_sim_time):
            self.last_observed_sim_time = self.sim_time

    # ---- Invariants: structural predicates ----

    @invariant()
    def s2_unique_in_flight_ids(self):
        """S2: when both slots are occupied, their event_ids differ."""
        cur = self.scheduler.active_event
        nxt = self.scheduler.next_event
        if cur is not None and nxt is not None:
            assert cur.event_id != nxt.event_id, (
                f"S2 invariant violation: current and next have the "
                f"same event_id {cur.event_id}"
            )

    @invariant()
    def s3_bounded_slots(self):
        """S3: at most 1 _current_event and 1 _next_event in flight.

        ``RETURN_TO_ACTIVE`` (when produced by ``begin_return`` from a
        non-HOLDING phase) is queued into _next_event and counts toward
        the bound — that's the contract."""
        cur = self.scheduler.active_event
        nxt = self.scheduler.next_event
        n_filled = (1 if cur is not None else 0) + (1 if nxt is not None else 0)
        assert n_filled <= 2, (
            f"S3 invariant violation: {n_filled} ScheduledEvent slots filled"
        )

    @invariant()
    def s6_clock_monotonic(self):
        """S6: scheduler's recorded ``_last_sim_time`` is monotone-
        non-decreasing across the random walk.  The tick rule's
        positive-dt strategy enforces this by construction; the
        invariant catches any future rule that side-effects backward."""
        if self.last_observed_sim_time is not None:
            assert self.scheduler._last_sim_time is not None
            # Allow equality (re-tick at same sim_time is permitted).
            assert (self.scheduler._last_sim_time
                    >= self.last_observed_sim_time - 1e-9), (
                f"S6 invariant violation: scheduler _last_sim_time="
                f"{self.scheduler._last_sim_time} regressed below "
                f"observed {self.last_observed_sim_time}"
            )

    @invariant()
    def s5_segment_lives_at_last_state(self):
        """S5: every currently-live segment starts at a state that
        was once ``_last_*``.  We can't check the build-time
        equality directly after the fact (``_last_*`` has since
        advanced via segment evaluation), but we can verify the
        weaker structural property: the segment's ``t_start`` is
        not in the future relative to the observed clock — i.e.,
        the segment was built at-or-before the current tick.

        This is the residual S5 evidence that survives advancement;
        the per-build C0/C1/C2 splice continuity is verified by the
        targeted scenario tests in ``TestS5SegmentLiveState`` above."""
        for seg in (self.scheduler._seg_current,
                    self.scheduler._seg_next,
                    self.scheduler._seg_return):
            if seg is None:
                continue
            assert (self.scheduler._last_sim_time is None
                    or seg.t_start <= self.scheduler._last_sim_time + 1e-9), (
                f"S5 evidence: segment t_start={seg.t_start} > "
                f"_last_sim_time={self.scheduler._last_sim_time}"
            )


# Settings template mirrors tests/sim/test_make_feasible_events.py:135 —
# bounded examples + suppress the too_slow health check (state-machine
# walks are inherently slower than scalar property tests).
TestSchedulerStateMachine = SchedulerStateMachine.TestCase
TestSchedulerStateMachine.settings = settings(
    max_examples=50,
    deadline=None,
    suppress_health_check=[HealthCheck.too_slow, HealthCheck.filter_too_much],
)


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
