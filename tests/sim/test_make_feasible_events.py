"""Tests for ``make_feasible_events`` — the K1–K6 reference-feasibility layer.

Two test layers:

**Property tests** (hypothesis-based): for random (p0, v0, p1, v1, v_max, T)
tuples, assert the returned events satisfy every K1–K6 invariant.  These are
the regression floor — if they fail, new contributors have silently re-introduced
a failure-mode class the W1–W11 cycle was built to prevent.

**Scenario tests**: every one of the 14 failure scenarios enumerated in the
investigation (see logbook/2026-04-19-bundle-a-*.md and the 223902 diagnosis)
gets a named test reproducing its geometry.  These are the *specific* cases
the contract is intended to defend.

See ``controller/REFERENCE_LAYER_CONTRACT.md`` for the K1–K6 normative spec.
"""

from __future__ import annotations

import numpy as np
import pytest
from hypothesis import given, strategies as st, settings, HealthCheck

from controller.target import make_feasible_events, ReferenceEvent
from controller.feasibility import (
    quintic_peak_vel_per_axis,
    quintic_peak_acc_per_axis,
    segment_is_feasible,
)

# Canonical plant/MPC params used across tests (mirror controller/params.py
# defaults so property tests exercise the production operating point).
V_MAX = 140.0          # mm/s
TAU = 0.04             # s
BETA = 0.85
A_MAX = V_MAX / TAU    # 3500 mm/s² — K3's physical limit
V_LIMIT = BETA * V_MAX  # 119.0 mm/s
A_LIMIT = BETA * A_MAX  # 2975.0 mm/s²


# ---------------------------------------------------------------------------
# K1–K6 assertion helpers
# ---------------------------------------------------------------------------

def _assert_K1(events, t_now, plant_pose, plant_twist, v_clamp):
    """K1 — first event anchored at t_now with live state (post-K6-clamp twist)."""
    assert events, "K1: must emit ≥1 event"
    e0 = events[0]
    assert abs(e0.time - t_now) < 1e-6, f"K1: first event at t={e0.time}, want {t_now}"
    np.testing.assert_allclose(e0.pose, plant_pose, atol=1e-6,
                               err_msg="K1: first event pose != plant pose")
    expected_twist = np.asarray(plant_twist, dtype=float).copy()
    expected_twist[:3] = np.clip(expected_twist[:3], -v_clamp, v_clamp)
    np.testing.assert_allclose(e0.twist, expected_twist, atol=1e-6,
                               err_msg="K1/K6: first event twist != clamped plant twist")


def _assert_K2_K3(events, v_max, tau, beta):
    """K2/K3 — each segment's peak |v| and |a| within β·limits."""
    for i in range(len(events) - 1):
        e0, e1 = events[i], events[i + 1]
        T = e1.time - e0.time
        assert T > 0, f"K4 prerequisite: event times not strictly increasing at i={i}"
        peak_v = quintic_peak_vel_per_axis(
            e0.pose, e0.twist, e0.accel, e1.pose, e1.twist, e1.accel, T,
        )
        peak_a = quintic_peak_acc_per_axis(
            e0.pose, e0.twist, e0.accel, e1.pose, e1.twist, e1.accel, T,
        )
        # 2% safety-margin slack to match _STRETCH_BS_SAFETY_MARGIN in target.py
        assert np.max(peak_v[:3]) <= beta * v_max * 1.02, (
            f"K2 violation at seg {i}: peak|v|={np.max(peak_v[:3]):.2f} > "
            f"{beta * v_max:.2f}"
        )
        assert np.max(peak_a[:3]) <= beta * (v_max / tau) * 1.02, (
            f"K3 violation at seg {i}: peak|a|={np.max(peak_a[:3]):.2f} > "
            f"{beta * (v_max / tau):.2f}"
        )


def _assert_K4(events, min_span=0.05):
    for i in range(len(events) - 1):
        span = events[i + 1].time - events[i].time
        assert span >= min_span - 1e-9, (
            f"K4 violation: span {span:.4f}s < {min_span:.4f}s between events {i},{i+1}"
        )


def _assert_K5(events):
    """K5 — single event per time stamp; no impulse across boundaries.
    Because make_feasible_events emits a list (not a graph), K5 is satisfied
    by construction *unless* two events end up at the same time (which would
    already have raised)."""
    for i in range(len(events) - 1):
        if abs(events[i + 1].time - events[i].time) < 1e-9:
            np.testing.assert_allclose(events[i].twist, events[i + 1].twist,
                                       err_msg="K5: coincident events twist disagree")


def _assert_K6(events, v_clamp):
    """K6 — each event's linear twist within ±v_clamp; angular unrestricted."""
    for i, ev in enumerate(events):
        lin = ev.twist[:3]
        assert np.all(np.abs(lin) <= v_clamp + 1e-6), (
            f"K6 violation at event {i}: |linear twist|={np.max(np.abs(lin)):.2f} > {v_clamp:.2f}"
        )


def _assert_K1_K6_compliant(events, t_now, plant_pose, plant_twist,
                             v_max=V_MAX, tau=TAU, beta=BETA):
    v_clamp = beta * v_max
    _assert_K1(events, t_now, plant_pose, plant_twist, v_clamp)
    _assert_K2_K3(events, v_max, tau, beta)
    _assert_K4(events)
    _assert_K5(events)
    _assert_K6(events, v_clamp)


# ---------------------------------------------------------------------------
# Property-based tests (hypothesis)
# ---------------------------------------------------------------------------

# Six-vector strategies — bounded so generated tests don't chase nonsense
_pose_strat = st.lists(st.floats(min_value=-200, max_value=200,
                                  allow_nan=False, allow_infinity=False),
                       min_size=6, max_size=6).map(np.asarray)
_twist_strat = st.lists(st.floats(min_value=-300, max_value=300,
                                   allow_nan=False, allow_infinity=False),
                        min_size=6, max_size=6).map(np.asarray)
_T_strat = st.floats(min_value=0.05, max_value=3.0, allow_nan=False)
_vmax_strat = st.floats(min_value=50.0, max_value=300.0, allow_nan=False)
_tau_strat = st.floats(min_value=0.01, max_value=0.15, allow_nan=False)


@given(p0=_pose_strat, v0=_twist_strat, p1=_pose_strat, v1=_twist_strat,
       T=_T_strat, v_max=_vmax_strat, tau=_tau_strat)
@settings(max_examples=150, deadline=None,
          suppress_health_check=[HealthCheck.too_slow])
def test_property_K1_K6_two_event_proposal(p0, v0, p1, v1, T, v_max, tau):
    """Random two-event proposals produce K1–K6 compliant events.

    This is the canonical regression — if any future change breaks the K1–K6
    contract on a random input, this test catches it.
    """
    t_now = 0.0
    proposal = [
        ReferenceEvent(time=t_now, pose=p0, twist=v0, accel=np.zeros(6)),
        ReferenceEvent(time=t_now + T, pose=p1, twist=v1, accel=np.zeros(6)),
    ]
    events, reason = make_feasible_events(
        p0, v0, proposal, t_now, v_max_mmps=v_max, tau_s=tau,
        max_stretch_ratio=8.0,  # generous to ensure feasibility across the strategy
    )
    # Property: if reason is None, fully K1–K6 compliant
    if reason is None:
        _assert_K1_K6_compliant(events, t_now, p0, v0, v_max=v_max, tau=tau)
    # Rejection: reason is a non-empty string
    else:
        assert isinstance(reason, str) and len(reason) > 0


@given(p0=_pose_strat, v0=_twist_strat, p1=_pose_strat, v1=_twist_strat,
       T=_T_strat, v_max=_vmax_strat, tau=_tau_strat)
@settings(max_examples=100, deadline=None,
          suppress_health_check=[HealthCheck.too_slow])
def test_property_no_stretch_rejects_deterministically(p0, v0, p1, v1, T, v_max, tau):
    """no_stretch=True either passes (K1–K6 via clamping alone) or rejects
    with a reason string and returns events unmodified (except K1 anchor + K6 clamp)."""
    t_now = 0.0
    proposal = [
        ReferenceEvent(time=t_now, pose=p0, twist=v0, accel=np.zeros(6)),
        ReferenceEvent(time=t_now + T, pose=p1, twist=v1, accel=np.zeros(6)),
    ]
    events, reason = make_feasible_events(
        p0, v0, proposal, t_now, v_max_mmps=v_max, tau_s=tau, no_stretch=True,
    )
    if reason is None:
        _assert_K1_K6_compliant(events, t_now, p0, v0, v_max=v_max, tau=tau)
    else:
        # K1/K5/K6 still enforced even on rejection — only K2/K3 may be violated.
        _assert_K1(events, t_now, p0, v0, beta * v_max)
        _assert_K6(events, beta * v_max)
        # Final event time is preserved (no stretching happened)
        assert abs(events[-1].time - (t_now + T)) < 1e-6, (
            "no_stretch: final event time must not move"
        )


beta = BETA  # (fixture for the above)


# ---------------------------------------------------------------------------
# Scenario tests — one per failure mode from the 14-item investigation table
# ---------------------------------------------------------------------------

class TestScenario01_Move5_PlantAheadSameDir:
    """Scenario 1: plant ahead of ref, ref accelerating same direction (Move 5)."""

    def test_move5_geometry(self):
        plant_pose = np.array([8.57, 34.0, 169.3, 0, 0, 0], dtype=float)
        plant_twist = np.array([0, 0, 80.0, 0, 0, 0], dtype=float)
        target = np.array([0, 0, 220.0, 0, 0, 0], dtype=float)
        t_now = 2.0
        proposal = [
            ReferenceEvent(time=t_now, pose=plant_pose.copy(),
                           twist=plant_twist.copy(), accel=np.zeros(6)),
            ReferenceEvent(time=t_now + 0.6, pose=target,
                           twist=np.zeros(6), accel=np.zeros(6)),
        ]
        events, reason = make_feasible_events(
            plant_pose, plant_twist, proposal, t_now,
            v_max_mmps=V_MAX, tau_s=TAU,
        )
        assert reason is None
        _assert_K1_K6_compliant(events, t_now, plant_pose, plant_twist)
        # K1 must have used the clamped live twist (80 mm/s < v_clamp=119, so unchanged)
        np.testing.assert_allclose(events[0].twist[:3], [0, 0, 80.0])


class TestScenario02_223902_DirectionReversal:
    """Scenario 2: plant ahead of ref, ref reverses direction (the 223902 failure)."""

    def test_223902_geometry(self):
        plant_pose = np.array([0, 0, 210, 0, 0, 0], dtype=float)
        plant_twist = np.array([0, 0, 60, 0, 0, 0], dtype=float)
        target = np.array([0, 0, 170, 0, 0, 0], dtype=float)
        t_now = 1.2
        proposal = [
            ReferenceEvent(time=t_now, pose=plant_pose.copy(),
                           twist=plant_twist.copy(), accel=np.zeros(6)),
            ReferenceEvent(time=t_now + 0.5, pose=target,
                           twist=np.zeros(6), accel=np.zeros(6)),
        ]
        events, reason = make_feasible_events(
            plant_pose, plant_twist, proposal, t_now,
            v_max_mmps=V_MAX, tau_s=TAU,
        )
        assert reason is None
        # Critical: the arrival was stretched from 0.5s to something longer
        # because the original peak |v_z| was 177 mm/s (above β·v_max=119).
        assert events[-1].time > t_now + 0.5, (
            "223902: segment should have been stretched"
        )
        _assert_K1_K6_compliant(events, t_now, plant_pose, plant_twist)

    def test_223902_catch_path_rejects(self):
        """Same geometry with no_stretch=True rejects with a reason containing
        the peak-vs-limit ratio (vr > 1.0)."""
        plant_pose = np.array([0, 0, 210, 0, 0, 0], dtype=float)
        plant_twist = np.array([0, 0, 60, 0, 0, 0], dtype=float)
        proposal = [
            ReferenceEvent(time=1.2, pose=plant_pose.copy(),
                           twist=plant_twist.copy(), accel=np.zeros(6)),
            ReferenceEvent(time=1.7, pose=np.array([0, 0, 170.0, 0, 0, 0]),
                           twist=np.zeros(6), accel=np.zeros(6)),
        ]
        events, reason = make_feasible_events(
            plant_pose, plant_twist, proposal, 1.2,
            v_max_mmps=V_MAX, tau_s=TAU, no_stretch=True,
        )
        assert reason is not None
        assert "peak_exceeds_no_stretch_allowed" in reason
        assert "vr=" in reason and "ar=" in reason


class TestScenario03_PeakVelExceedsVMax:
    """Scenario 3: quintic peak velocity > v_max (the deep cause of 1 and 2)."""

    def test_stretch_reduces_peak_below_limit(self):
        # Fast boundary-velocity plus short duration → guaranteed peak > v_max
        p0 = np.array([0, 0, 170, 0, 0, 0], dtype=float)
        v0 = np.array([100, 0, 0, 0, 0, 0], dtype=float)
        p1 = np.array([50, 0, 170, 0, 0, 0], dtype=float)
        v1 = np.array([-80, 0, 0, 0, 0, 0], dtype=float)
        proposal = [
            ReferenceEvent(time=0, pose=p0, twist=v0, accel=np.zeros(6)),
            ReferenceEvent(time=0.3, pose=p1, twist=v1, accel=np.zeros(6)),
        ]
        events, reason = make_feasible_events(
            p0, v0, proposal, 0.0, v_max_mmps=V_MAX, tau_s=TAU,
            max_stretch_ratio=8.0,
        )
        assert reason is None
        _assert_K2_K3(events, V_MAX, TAU, BETA)


class TestScenario04_PeakAccelExceedsBandwidth:
    """Scenario 4: quintic peak accel > v_max/tau (latent before W1)."""

    def test_large_pose_change_short_T_triggers_K3(self):
        # 100 mm move in 0.1 s with zero-vel boundaries forces high accel
        p0 = np.array([0, 0, 170, 0, 0, 0], dtype=float)
        v0 = np.zeros(6)
        p1 = np.array([0, 0, 270, 0, 0, 0], dtype=float)
        proposal = [
            ReferenceEvent(time=0, pose=p0, twist=v0, accel=np.zeros(6)),
            ReferenceEvent(time=0.1, pose=p1, twist=np.zeros(6), accel=np.zeros(6)),
        ]
        events, reason = make_feasible_events(
            p0, v0, proposal, 0.0, v_max_mmps=V_MAX, tau_s=TAU,
            max_stretch_ratio=20.0,
        )
        assert reason is None
        _assert_K2_K3(events, V_MAX, TAU, BETA)


class TestScenario05_StepDiscontinuityAtArrival:
    """Scenario 5: ref step-discontinuity (ZmqTargetSource pre-Bundle-A — fixed)."""

    def test_k1_anchor_prepended_if_caller_forgot(self):
        """If the caller passes a target-only event (no plant anchor), K1
        should auto-prepend an anchor at the live state."""
        plant_pose = np.array([0, 0, 170, 0, 0, 0], dtype=float)
        plant_twist = np.array([0, 0, 30, 0, 0, 0], dtype=float)
        proposal = [
            ReferenceEvent(time=1.0, pose=np.array([0, 0, 220, 0, 0, 0]),
                           twist=np.zeros(6), accel=np.zeros(6)),
        ]
        events, reason = make_feasible_events(
            plant_pose, plant_twist, proposal, t_now=0.5,
            v_max_mmps=V_MAX, tau_s=TAU,
        )
        assert reason is None
        # K1 anchor inserted at t=0.5
        assert events[0].time == pytest.approx(0.5)
        np.testing.assert_allclose(events[0].pose, plant_pose, atol=1e-6)

    def test_k1_anchor_inserted_when_first_event_in_past_with_matching_state(self):
        """Past-time first event must trigger K1 re-anchoring even when its
        pose/twist coincidentally match the live state.  Without symmetric
        ``abs(first.time - t_now) > 1e-6`` the past event slips through and
        becomes ref event 0 at ``time < t_now``.
        """
        cur_pose = np.array([0., 0., 170., 0., 0., 0.])
        cur_twist = np.zeros(6)
        t_now = 1.0
        proposal = [
            ReferenceEvent(time=0.5, pose=cur_pose.copy(),
                           twist=cur_twist.copy(), accel=np.zeros(6)),
            ReferenceEvent(time=2.0, pose=cur_pose + np.array([10, 0, 0, 0, 0, 0]),
                           twist=cur_twist.copy(), accel=np.zeros(6)),
        ]
        events, reason = make_feasible_events(
            cur_pose, cur_twist, proposal, t_now,
            v_max_mmps=V_MAX, tau_s=TAU,
        )
        assert reason is None
        assert abs(events[0].time - t_now) < 1e-6, (
            f"K1: anchor not at t_now (got {events[0].time})"
        )


class TestScenario06_WarmStartIncompatibility:
    """Scenario 6: addressed by W5 (warm_start_valid on TargetCommand) — not W1.
    Here we verify W1 at least doesn't paper over a big direction change."""

    def test_big_direction_change_events_are_feasible(self):
        """A large reversal produces K2/K3-compliant events even when stretched."""
        plant_pose = np.array([0, 0, 200, 0, 0, 0], dtype=float)
        plant_twist = np.array([0, 0, 100, 0, 0, 0], dtype=float)
        proposal = [
            ReferenceEvent(time=0, pose=plant_pose.copy(),
                           twist=plant_twist.copy(), accel=np.zeros(6)),
            ReferenceEvent(time=0.6, pose=np.array([0, 0, 100, 0, 0, 0]),
                           twist=np.zeros(6), accel=np.zeros(6)),
        ]
        events, reason = make_feasible_events(
            plant_pose, plant_twist, proposal, 0.0,
            v_max_mmps=V_MAX, tau_s=TAU, max_stretch_ratio=8.0,
        )
        assert reason is None
        _assert_K1_K6_compliant(events, 0.0, plant_pose, plant_twist)


class TestScenario07_RapidSuccessiveTargets:
    """Scenario 7: rapid successive target changes (W1 makes each *individual* ref K1–K6,
    W5 handles warm-start invalidation between them)."""

    def test_consecutive_calls_produce_independent_feasible_events(self):
        """Each make_feasible_events call is independent; no hidden cross-state."""
        plant_pose = np.array([0, 0, 170, 0, 0, 0], dtype=float)
        plant_twist = np.array([0, 0, 40, 0, 0, 0], dtype=float)
        for i in range(10):
            target = np.array([0, 0, 170 + 10 * i, 0, 0, 0], dtype=float)
            proposal = [
                ReferenceEvent(time=0, pose=plant_pose.copy(),
                               twist=plant_twist.copy(), accel=np.zeros(6)),
                ReferenceEvent(time=0.4, pose=target,
                               twist=np.zeros(6), accel=np.zeros(6)),
            ]
            events, reason = make_feasible_events(
                plant_pose, plant_twist, proposal, 0.0,
                v_max_mmps=V_MAX, tau_s=TAU,
            )
            assert reason is None
            _assert_K1_K6_compliant(events, 0.0, plant_pose, plant_twist)


class TestScenario08_TwistInconsistentAcrossBoundary:
    """Scenario 8: two events at the same time with inconsistent twists (K5 violation)."""

    def test_coincident_events_with_mismatched_twist_raises(self):
        plant_pose = np.zeros(6)
        plant_twist = np.zeros(6)
        proposal = [
            ReferenceEvent(time=0.5, pose=np.array([0, 0, 200, 0, 0, 0]),
                           twist=np.array([0, 0, 30, 0, 0, 0]), accel=np.zeros(6)),
            ReferenceEvent(time=0.5, pose=np.array([0, 0, 200, 0, 0, 0]),
                           twist=np.array([0, 0, -30, 0, 0, 0]), accel=np.zeros(6)),
        ]
        with pytest.raises(ValueError, match="K5"):
            make_feasible_events(
                plant_pose, plant_twist, proposal, 0.0,
                v_max_mmps=V_MAX, tau_s=TAU,
            )


class TestScenario09_StalePlantSnapshot:
    """Scenario 9: K1 always anchors at LIVE state, so any proposal-side
    staleness is overwritten by the current pose/twist we pass in."""

    def test_proposal_start_pose_is_ignored_uses_live(self):
        # Caller proposes an "old" plant pose — K1 should overwrite.
        live_pose = np.array([0, 0, 185, 0, 0, 0], dtype=float)
        live_twist = np.array([0, 0, 60, 0, 0, 0], dtype=float)
        stale_pose = np.array([0, 0, 170, 0, 0, 0], dtype=float)   # the bug
        stale_twist = np.array([0, 0, 0, 0, 0, 0], dtype=float)
        proposal = [
            ReferenceEvent(time=0, pose=stale_pose, twist=stale_twist, accel=np.zeros(6)),
            ReferenceEvent(time=0.5, pose=np.array([0, 0, 220, 0, 0, 0]),
                           twist=np.zeros(6), accel=np.zeros(6)),
        ]
        events, reason = make_feasible_events(
            live_pose, live_twist, proposal, 0.0,
            v_max_mmps=V_MAX, tau_s=TAU,
        )
        assert reason is None
        # First event should carry LIVE pose, not the stale proposal pose
        np.testing.assert_allclose(events[0].pose, live_pose, atol=1e-6)
        np.testing.assert_allclose(events[0].twist, live_twist, atol=1e-6)


class TestScenario10_ArrivalTimeInPast:
    """Scenario 10: caller provides an arrival_time < t_now."""

    def test_past_arrival_event_is_dropped(self):
        plant_pose = np.array([0, 0, 200, 0, 0, 0], dtype=float)
        plant_twist = np.zeros(6)
        # Proposal: event with time 0.5, but t_now=1.0 → in the past
        proposal = [
            ReferenceEvent(time=0.5, pose=np.array([0, 0, 205, 0, 0, 0]),
                           twist=np.zeros(6), accel=np.zeros(6)),
            ReferenceEvent(time=1.3, pose=np.array([0, 0, 210, 0, 0, 0]),
                           twist=np.zeros(6), accel=np.zeros(6)),
        ]
        events, reason = make_feasible_events(
            plant_pose, plant_twist, proposal, t_now=1.0,
            v_max_mmps=V_MAX, tau_s=TAU,
        )
        assert reason is None
        # First event at t_now; past event dropped
        assert events[0].time == pytest.approx(1.0)
        # Next event at or after 1.3 (may be stretched)
        assert events[-1].time >= 1.3 - 1e-9


class TestScenario11_TargetTwistExceedsVMax:
    """Scenario 11: caller-supplied target_twist > v_max (silent before W1)."""

    def test_oversized_target_twist_clamped_by_K6(self):
        plant_pose = np.zeros(6)
        plant_twist = np.zeros(6)
        proposal = [
            ReferenceEvent(time=0, pose=plant_pose.copy(),
                           twist=plant_twist.copy(), accel=np.zeros(6)),
            ReferenceEvent(time=0.5, pose=np.array([0, 0, 200, 0, 0, 0]),
                           twist=np.array([500, -500, 400, 0, 0, 0]),
                           accel=np.zeros(6)),
        ]
        events, reason = make_feasible_events(
            plant_pose, plant_twist, proposal, 0.0,
            v_max_mmps=V_MAX, tau_s=TAU, max_stretch_ratio=8.0,
        )
        # Terminal event linear twist clamped to ±β·v_max = ±119
        terminal_lin = events[-1].twist[:3]
        assert np.all(np.abs(terminal_lin) <= BETA * V_MAX + 1e-6)


class TestScenario12_ClampNoneInheritsFKNoise:
    """Scenario 12: K6 clamp is always applied by make_feasible_events,
    regardless of flat_target_to_events's legacy clamp_start_twist_mmps kwarg."""

    def test_fk_noise_spike_always_clamped(self):
        plant_pose = np.zeros(6)
        fk_spike = np.array([500, 0, 0, 0, 0, 0], dtype=float)  # simulated FK glitch
        target = np.array([50, 0, 0, 0, 0, 0], dtype=float)
        proposal = [
            ReferenceEvent(time=0, pose=plant_pose.copy(), twist=fk_spike.copy(),
                           accel=np.zeros(6)),
            ReferenceEvent(time=0.6, pose=target, twist=np.zeros(6), accel=np.zeros(6)),
        ]
        events, reason = make_feasible_events(
            plant_pose, fk_spike, proposal, 0.0,
            v_max_mmps=V_MAX, tau_s=TAU, max_stretch_ratio=8.0,
        )
        # Even with a 500 mm/s FK spike, the K1 anchor is clamped to β·v_max
        assert events[0].twist[0] == pytest.approx(BETA * V_MAX)


class TestScenario13_WalkForwardCommitsOldDir:
    """Scenario 13: fixed by W7 in _handle_failure, not W1. Here we only verify
    that the reference *does not* contribute to the failure — it's K2/K3-compliant."""

    def test_ref_is_clean_so_fallback_only_failure_mode_is_saturation(self):
        plant_pose = np.array([0, 0, 210, 0, 0, 0], dtype=float)
        plant_twist = np.array([0, 0, 60, 0, 0, 0], dtype=float)
        proposal = [
            ReferenceEvent(time=0, pose=plant_pose.copy(),
                           twist=plant_twist.copy(), accel=np.zeros(6)),
            ReferenceEvent(time=0.5, pose=np.array([0, 0, 170, 0, 0, 0]),
                           twist=np.zeros(6), accel=np.zeros(6)),
        ]
        events, _ = make_feasible_events(
            plant_pose, plant_twist, proposal, 0.0,
            v_max_mmps=V_MAX, tau_s=TAU,
        )
        _assert_K2_K3(events, V_MAX, TAU, BETA)


class TestScenario14_PlantVelOppositeTargetTwist:
    """Scenario 14: catch retargeting — plant moving opposite to requested target_twist."""

    def test_plant_downward_target_upward_gets_stretched(self):
        plant_pose = np.array([0, 0, 200, 0, 0, 0], dtype=float)
        plant_twist = np.array([0, 0, -80, 0, 0, 0], dtype=float)  # plant falling
        target = np.array([0, 0, 230, 0, 0, 0], dtype=float)
        target_twist = np.array([0, 0, 100, 0, 0, 0], dtype=float)  # want upward at arrival
        proposal = [
            ReferenceEvent(time=0, pose=plant_pose.copy(),
                           twist=plant_twist.copy(), accel=np.zeros(6)),
            ReferenceEvent(time=0.4, pose=target, twist=target_twist,
                           accel=np.zeros(6)),
        ]
        events, reason = make_feasible_events(
            plant_pose, plant_twist, proposal, 0.0,
            v_max_mmps=V_MAX, tau_s=TAU, max_stretch_ratio=8.0,
        )
        assert reason is None
        _assert_K1_K6_compliant(events, 0.0, plant_pose, plant_twist)

    def test_catch_path_rejects_infeasible_retarget(self):
        """Catch target with opposing plant velocity + no_stretch → reject."""
        plant_pose = np.array([0, 0, 200, 0, 0, 0], dtype=float)
        plant_twist = np.array([0, 0, -100, 0, 0, 0], dtype=float)
        target = np.array([0, 0, 250, 0, 0, 0], dtype=float)
        target_twist = np.array([0, 0, 120, 0, 0, 0], dtype=float)
        proposal = [
            ReferenceEvent(time=0, pose=plant_pose.copy(),
                           twist=plant_twist.copy(), accel=np.zeros(6)),
            ReferenceEvent(time=0.3, pose=target, twist=target_twist,
                           accel=np.zeros(6)),
        ]
        events, reason = make_feasible_events(
            plant_pose, plant_twist, proposal, 0.0,
            v_max_mmps=V_MAX, tau_s=TAU, no_stretch=True,
        )
        assert reason is not None, "Infeasible catch retarget must reject"


# ---------------------------------------------------------------------------
# Edge cases
# ---------------------------------------------------------------------------

class TestEdgeCases:
    def test_empty_proposal_emits_hold_anchor(self):
        plant_pose = np.array([0, 0, 170, 0, 0, 0], dtype=float)
        plant_twist = np.array([0, 0, 10, 0, 0, 0], dtype=float)
        events, reason = make_feasible_events(
            plant_pose, plant_twist, [], 0.0,
            v_max_mmps=V_MAX, tau_s=TAU,
        )
        assert reason is None
        assert len(events) == 1
        np.testing.assert_allclose(events[0].pose, plant_pose)

    def test_bad_v_max_raises(self):
        with pytest.raises(ValueError):
            make_feasible_events(np.zeros(6), np.zeros(6), [], 0.0,
                                 v_max_mmps=-1.0, tau_s=TAU)

    def test_bad_tau_raises(self):
        with pytest.raises(ValueError):
            make_feasible_events(np.zeros(6), np.zeros(6), [], 0.0,
                                 v_max_mmps=V_MAX, tau_s=0.0)

    def test_k4_drops_near_degenerate_event(self):
        """Two events within 50 ms: K4 drops the second."""
        plant_pose = np.zeros(6)
        proposal = [
            ReferenceEvent(time=0, pose=plant_pose, twist=np.zeros(6),
                           accel=np.zeros(6)),
            ReferenceEvent(time=0.01, pose=np.array([0, 0, 200, 0, 0, 0]),
                           twist=np.zeros(6), accel=np.zeros(6)),
            ReferenceEvent(time=0.5, pose=np.array([0, 0, 220, 0, 0, 0]),
                           twist=np.zeros(6), accel=np.zeros(6)),
        ]
        events, _ = make_feasible_events(
            plant_pose, np.zeros(6), proposal, 0.0,
            v_max_mmps=V_MAX, tau_s=TAU,
        )
        _assert_K4(events)

    def test_stretch_warning_threshold(self):
        """Stretches > 25% of requested duration log a WARNING.

        Attaches a direct handler to the target logger (caplog doesn't reliably
        intercept loggers whose propagation isn't in the root chain here)."""
        import logging
        records: list[logging.LogRecord] = []

        class _Capture(logging.Handler):
            def emit(self, record): records.append(record)

        target_logger = logging.getLogger('controller.target')
        handler = _Capture(level=logging.WARNING)
        target_logger.addHandler(handler)
        prev_level = target_logger.level
        target_logger.setLevel(logging.WARNING)
        try:
            plant_pose = np.array([0, 0, 200, 0, 0, 0], dtype=float)
            plant_twist = np.array([0, 0, 100, 0, 0, 0], dtype=float)
            proposal = [
                ReferenceEvent(time=0, pose=plant_pose.copy(),
                               twist=plant_twist.copy(), accel=np.zeros(6)),
                ReferenceEvent(time=0.3, pose=np.array([0, 0, 150, 0, 0, 0]),
                               twist=np.zeros(6), accel=np.zeros(6)),
            ]
            make_feasible_events(
                plant_pose, plant_twist, proposal, 0.0,
                v_max_mmps=V_MAX, tau_s=TAU, max_stretch_ratio=8.0,
            )
        finally:
            target_logger.removeHandler(handler)
            target_logger.setLevel(prev_level)

        assert any("stretched segment" in r.getMessage() for r in records), (
            f"Expected a WARNING log about stretch >25%. Got: "
            f"{[(r.levelname, r.getMessage()) for r in records]}"
        )
