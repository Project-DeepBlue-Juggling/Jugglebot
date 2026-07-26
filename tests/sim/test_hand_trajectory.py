"""Tests for hand catch trajectory generator (sim/hand/trajectory.py).

Validates that the Python port matches the Teensy Trajectory.h math for
buildCatch() and makeSmoothMove().
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from hand.trajectory import (
    HandCatchTrajectory,
    HandSmoothMove,
    HandCatchSequence,
    CATCH_VEL_RATIO,
    INERTIA_RATIO,
    CATCH_VEL_HOLD_PCT,
    HAND_STROKE_M,
    STROKE_MARGIN_M,
    END_PROFILE_HOLD_S,
    MAX_SMOOTH_MOVE_HAND_ACCEL_RPS2,
    QUINTIC_S2_MAX,
    QUINTIC_H_MAX,
    QUINTIC_H2_MAX,
    SMOOTH_MOVE_V0_DEADBAND_RPS,
    SMOOTH_MOVE_EXCURSION_MARGIN_REV,
    SMOOTH_MOVE_MIN_DURATION_S,
    HAND_MOTOR_MAX_POSITION_REVS,
    HAND_HOME_ABS_POS_REV,
    MIN_EVENT_VEL_MPS,
    MAX_EVENT_VEL_MPS,
    mm_to_rev,
    rev_to_mm,
    plan_smooth_move,
    smooth_move_duration_s,
    smooth_move_accel_limited_duration_s,
    smooth_move_max_duration_s,
    smooth_move_peak_accel_rps2,
    smooth_move_excursion_rev,
    quintic_s,
    quintic_h,
    quintic_h1,
    quintic_h2,
    _TOTAL_STROKE_M,
    _TOTAL_STROKE_MM,
    _LINEAR_GAIN,
)

# x3 — the throw stroke's end AND the catch trajectory's first sample.  Every
# prelude the fix is about starts or ends here.
_X3_REV = _TOTAL_STROKE_M * _LINEAR_GAIN                      # 9.9594031 rev
_CEIL_REV = HAND_MOTOR_MAX_POSITION_REVS - SMOOTH_MOVE_EXCURSION_MARGIN_REV

# The measured hand speed at ball release on 2026-07-25 — the worst v0 a hand
# command has ever landed on (temp/logs/toss_trace_2026-07-25_15-24-25.jsonl,
# max vel_meas 119.61 rev/s; the 15-06-38 trace reads 124.07).
_V0_AT_RELEASE_RPS = 119.6


# ── HandCatchTrajectory ──────────────────────────────────────────────────

class TestCatchTrajectoryShape:
    """Verify the 3-segment catch trajectory has correct shape."""

    @pytest.fixture
    def traj(self):
        return HandCatchTrajectory(event_vel_mps=3.0)

    def test_starts_at_rest(self, traj):
        """Position at start_time should equal start_pos_mm (no displacement)."""
        p_before = traj.sample(traj.start_time - 0.01)
        p_start = traj.sample(traj.start_time)
        assert abs(p_before - traj.start_pos_mm) < 0.01
        assert abs(p_start - traj.start_pos_mm) < 0.1

    def test_ends_at_rest(self, traj):
        """Position at end_time should equal end_pos_mm (trajectory complete)."""
        p_end = traj.sample(traj.end_time)
        p_after = traj.sample(traj.end_time + 1.0)
        assert abs(p_end - traj.end_pos_mm) < 0.1
        assert abs(p_after - traj.end_pos_mm) < 0.01

    def test_hand_moves_downward(self, traj):
        """Hand should move downward (decreasing position) during catch."""
        p_start = traj.sample(traj.start_time)
        p_mid = traj.sample(0.0)  # ball arrival
        p_end = traj.sample(traj.end_time)
        assert p_mid < p_start, "Hand should move down from start to mid"
        assert p_end < p_start, "Hand should move down from start to end"

    def test_total_displacement_matches_stroke(self, traj):
        """Total displacement should approximately equal the effective stroke."""
        displacement = traj.start_pos_mm - traj.end_pos_mm
        assert abs(displacement - _TOTAL_STROKE_MM) < 1.0, \
            f"Displacement {displacement:.1f} mm, expected ~{_TOTAL_STROKE_MM:.1f} mm"

    def test_catch_velocity_sign(self, traj):
        """Catch velocity should be negative (downward)."""
        assert traj.catch_velocity_mps < 0

    def test_catch_velocity_magnitude(self, traj):
        """Catch velocity = -CATCH_VEL_RATIO * event_vel."""
        expected = -CATCH_VEL_RATIO * 3.0
        assert abs(traj.catch_velocity_mps - expected) < 1e-6


class TestCatchTrajectoryTiming:
    """Verify timeline is centred on ball arrival."""

    def test_t0_is_midpoint_of_velocity_hold(self):
        """At t=0, hand should be in the constant-velocity phase."""
        traj = HandCatchTrajectory(event_vel_mps=3.0)
        # Sample around t=0 — velocity should be approximately constant
        dt = 0.001
        p_before = traj.sample(-dt)
        p_at = traj.sample(0.0)
        p_after = traj.sample(dt)

        # Numerical velocity at t=0
        v0 = (p_after - p_before) / (2.0 * dt)  # mm/s
        # Expected: CATCH_VEL_RATIO * event_vel * 1000 mm/s (negative)
        expected_v_mmps = -CATCH_VEL_RATIO * 3.0 * 1000.0
        assert abs(v0 - expected_v_mmps) / abs(expected_v_mmps) < 0.05, \
            f"Velocity at t=0: {v0:.0f} mm/s, expected ~{expected_v_mmps:.0f} mm/s"

    def test_start_time_is_negative(self):
        """Trajectory starts before ball arrival (negative start_time)."""
        traj = HandCatchTrajectory(event_vel_mps=3.0)
        assert traj.start_time < 0

    def test_end_time_is_positive(self):
        """Trajectory ends after ball arrival (positive end_time)."""
        traj = HandCatchTrajectory(event_vel_mps=3.0)
        assert traj.end_time > 0

    def test_duration_is_positive(self):
        traj = HandCatchTrajectory(event_vel_mps=3.0)
        assert traj.duration > 0
        assert abs(traj.duration - (traj.end_time - traj.start_time)) < 1e-9


class TestCatchTrajectoryEventVelRange:
    """Verify behaviour at min and max event velocities."""

    def test_min_event_vel(self):
        """Trajectory at min velocity (0.3 m/s) should still be valid."""
        traj = HandCatchTrajectory(event_vel_mps=0.3)
        assert traj.duration > 0
        disp = traj.start_pos_mm - traj.end_pos_mm
        assert abs(disp - _TOTAL_STROKE_MM) < 1.0

    def test_max_event_vel(self):
        """Trajectory at max velocity (7.0 m/s) should still be valid."""
        traj = HandCatchTrajectory(event_vel_mps=7.0)
        assert traj.duration > 0
        disp = traj.start_pos_mm - traj.end_pos_mm
        assert abs(disp - _TOTAL_STROKE_MM) < 1.0

    def test_clamping_below_min(self):
        """Values below min should be clamped."""
        traj_low = HandCatchTrajectory(event_vel_mps=0.1)
        traj_min = HandCatchTrajectory(event_vel_mps=0.3)
        # Same catch velocity since both are clamped to 0.3
        assert abs(traj_low.catch_velocity_mps - traj_min.catch_velocity_mps) < 1e-9

    def test_clamping_above_max(self):
        """Values above max should be clamped."""
        traj_high = HandCatchTrajectory(event_vel_mps=10.0)
        traj_max = HandCatchTrajectory(event_vel_mps=7.0)
        assert abs(traj_high.catch_velocity_mps - traj_max.catch_velocity_mps) < 1e-9

    def test_faster_vel_shorter_duration(self):
        """Higher event velocity should produce a shorter catch trajectory."""
        traj_slow = HandCatchTrajectory(event_vel_mps=1.0)
        traj_fast = HandCatchTrajectory(event_vel_mps=5.0)
        assert traj_fast.duration < traj_slow.duration


class TestCatchTrajectoryMonotonicity:
    """Verify hand position decreases monotonically during catch."""

    @pytest.mark.parametrize("event_vel", [0.5, 2.0, 5.0])
    def test_monotonic_descent(self, event_vel):
        """Hand position should never increase during the catch."""
        traj = HandCatchTrajectory(event_vel_mps=event_vel)
        n_samples = 200
        times = np.linspace(traj.start_time, traj.end_time, n_samples)
        positions = [traj.sample(t) for t in times]

        for i in range(1, len(positions)):
            assert positions[i] <= positions[i - 1] + 0.01, \
                f"Position increased at sample {i}: " \
                f"{positions[i]:.2f} > {positions[i-1]:.2f}"


# ── HandSmoothMove ───────────────────────────────────────────────────────

class TestSmoothMove:
    """Verify quintic S-curve smooth move."""

    def test_boundary_positions(self):
        """Start position at t=0, end position at t=duration."""
        sm = HandSmoothMove(100.0, 300.0)
        assert abs(sm.sample(0.0) - 100.0) < 0.01
        assert abs(sm.sample(sm.duration) - 300.0) < 0.01

    def test_zero_velocity_at_boundaries(self):
        """Velocity should be zero at start and end (quintic property)."""
        sm = HandSmoothMove(100.0, 300.0)
        dt = 0.0001
        # Start velocity
        v_start = (sm.sample(dt) - sm.sample(0.0)) / dt
        assert abs(v_start) < 10.0  # mm/s, should be near zero

        # End velocity
        v_end = (sm.sample(sm.duration) - sm.sample(sm.duration - dt)) / dt
        assert abs(v_end) < 10.0

    def test_zero_delta_instant(self):
        """Zero displacement → zero duration."""
        sm = HandSmoothMove(200.0, 200.0)
        assert sm.duration == 0.0
        assert sm.sample(0.0) == 200.0

    def test_monotonic_for_positive_delta(self):
        """Position should monotonically increase for upward move."""
        sm = HandSmoothMove(100.0, 300.0)
        n = 100
        times = np.linspace(0.0, sm.duration, n)
        positions = [sm.sample(t) for t in times]
        for i in range(1, len(positions)):
            assert positions[i] >= positions[i - 1] - 0.01

    def test_monotonic_for_negative_delta(self):
        """Position should monotonically decrease for downward move."""
        sm = HandSmoothMove(300.0, 100.0)
        n = 100
        times = np.linspace(0.0, sm.duration, n)
        positions = [sm.sample(t) for t in times]
        for i in range(1, len(positions)):
            assert positions[i] <= positions[i - 1] + 0.01

    def test_duration_scales_with_distance(self):
        """Longer moves should take longer."""
        sm_short = HandSmoothMove(100.0, 150.0)
        sm_long = HandSmoothMove(100.0, 300.0)
        assert sm_long.duration > sm_short.duration


# ── HandSmoothMove: the velocity-continuous prelude (C-HAND-1, firmware half) ──
#
# plans/active/hand-command-continuity.md Phase 4.  The firmware seeded every
# smooth move v = a = 0 from current_hand_position while current_hand_velocity
# sat declared and unread, so any command landing while the hand moved commanded
# a VELOCITY STEP.  These tests are the gate: they run in the sim mirror, and
# tests/firmware/test_hand_smooth_move_xref.py pins the mirror against the
# shipped Trajectory.h.

class TestQuinticShapeIdentities:
    """The two fixed shapes the profile decomposes into, and their landmarks.

    ``pos(tau) = x0 + delta*s(tau) + (v0*T)*h(tau)``.  Every number the duration
    bound and the excursion clamp use is a landmark of ``s`` or ``h``, so if a
    landmark constant drifts from the shape it names, both go wrong silently.
    """

    def test_h_vanishes_at_both_ends_so_v0_never_moves_the_endpoints(self):
        """The whole design rests on this: honouring v0 must not move the target.

        If ``h(1) != 0`` the prelude would land somewhere other than the main
        trajectory's first sample and the queue would resume with a position
        step — the exact defect the phase removes, reintroduced one level down.
        """
        assert quintic_h(0.0) == 0.0
        assert quintic_h(1.0) == 0.0
        assert quintic_h1(1.0) == 0.0        # and at rest there
        assert quintic_h2(1.0) == 0.0        # and with zero acceleration

    def test_h_is_non_negative_so_the_bulge_is_one_sided(self):
        """``h >= 0`` on [0, 1] is why the excursion check is one-sided."""
        for tau in np.linspace(0.0, 1.0, 2001):
            assert quintic_h(float(tau)) >= -1e-15

    def test_quintic_h_max_is_the_maximum_of_h(self):
        """16/81 at tau = 1/3 — the constant the excursion bound multiplies v0*T by."""
        taus = np.linspace(0.0, 1.0, 2_000_001)
        vals = [quintic_h(float(t)) for t in taus[::100]]
        assert QUINTIC_H_MAX == pytest.approx(16.0 / 81.0, abs=1e-8)
        assert quintic_h(1.0 / 3.0) == pytest.approx(16.0 / 81.0, abs=1e-15)
        assert max(vals) <= QUINTIC_H_MAX + 1e-8

    def test_quintic_h2_max_is_the_maximum_of_h_second_derivative(self):
        """``max|h''|`` at tau = (8 - sqrt(19))/15 — the constant in the duration bound."""
        tau_star = (8.0 - math.sqrt(19.0)) / 15.0
        assert tau_star == pytest.approx(0.2427401, abs=1e-6)
        assert abs(quintic_h2(tau_star)) == pytest.approx(QUINTIC_H2_MAX, abs=1e-6)
        for tau in np.linspace(0.0, 1.0, 20001):
            assert abs(quintic_h2(float(tau))) <= QUINTIC_H2_MAX + 1e-6

    def test_quintic_s2_max_is_the_maximum_of_s_second_derivative(self):
        """The pre-existing constant, re-pinned because the bound now pairs the two."""
        from hand.trajectory import quintic_s2
        vals = [abs(quintic_s2(float(t))) for t in np.linspace(0.0, 1.0, 20001)]
        assert max(vals) <= QUINTIC_S2_MAX + 1e-6
        assert QUINTIC_S2_MAX == pytest.approx(10.0 / math.sqrt(3.0), abs=1e-7)

    def test_decomposition_reproduces_the_coefficient_form(self):
        """``delta*s + u*h`` == ``u*tau + A tau^3 + B tau^4 + C tau^5``.

        The samplers use the shape form (it reduces exactly at ``u = 0``); the
        closed-form excursion derivation uses the coefficient form. They must be
        the same polynomial or the clamp bounds a different curve than the one
        emitted.
        """
        from hand.trajectory import _quintic_coeffs
        for delta, u in [(2.26, 43.2), (-3.83, 6.1), (0.0, -12.0), (9.96, 0.0)]:
            A, B, C = _quintic_coeffs(delta, u)
            for tau in np.linspace(0.0, 1.0, 501):
                tau = float(tau)
                shape = delta * quintic_s(tau) + u * quintic_h(tau)
                coeff = u * tau + A * tau ** 3 + B * tau ** 4 + C * tau ** 5
                assert shape == pytest.approx(coeff, abs=1e-12)


class TestSmoothMoveDurationBound:
    """The safety-critical line: the duration must bound the peak acceleration.

    Too loose and the prelude commands an acceleration the hand cannot produce,
    8-18 ms after a ball release at ~100 rev/s.  Too tight and the prelude eats a
    catch window that is only 115 ms wide at the shortest shipped flight.
    """

    def test_v0_zero_reduces_to_the_historical_formula_exactly(self):
        """Every existing caller is rest-to-rest; none may change by one bit."""
        for delta in (0.02, 0.10, 1.0, 2.2594, 3.8327, 9.9594, -9.9594):
            got = smooth_move_accel_limited_duration_s(delta, 0.0)
            want = math.sqrt(abs(delta) * QUINTIC_S2_MAX
                             / MAX_SMOOTH_MOVE_HAND_ACCEL_RPS2)
            assert got == want, delta
            # and the reported duration carries the floor, like the firmware
            assert smooth_move_duration_s(delta, 0.0) == max(want, 0.05)

    @pytest.mark.parametrize("v0", [
        -_V0_AT_RELEASE_RPS, -60.0, -24.63, -9.0, -6.01, 0.0, 6.01, 9.0,
        24.63, 60.0, _V0_AT_RELEASE_RPS])
    @pytest.mark.parametrize("delta", [-9.9594, -3.8327, -0.10, 0.0, 0.10,
                                       2.2594, 9.9594])
    def test_peak_acceleration_never_exceeds_the_limit(self, v0, delta):
        """The sweep the phase turns on: BOTH signs of v0, including 119.6 rev/s.

        Verified against the EXACT closed-form peak (itself checked against a
        400 000-sample brute force to 1.2e-9 relative,
        /tmp/probe_v0_quintic.py 2026-07-27), not against a sampling of the
        emitted profile — a sampled max can miss a narrow peak between samples,
        which is exactly the failure a too-loose bound would hide.
        """
        T = max(smooth_move_accel_limited_duration_s(delta, v0),
                SMOOTH_MOVE_MIN_DURATION_S)
        peak = smooth_move_peak_accel_rps2(delta, v0, T)
        assert peak <= MAX_SMOOTH_MOVE_HAND_ACCEL_RPS2 * (1 + 1e-9), (
            "T=%.6f gives peak |a| = %.3f rev/s^2" % (T, peak))

    def test_the_bound_is_tight_where_it_matters(self):
        """When v0 points AWAY from the target the bound is ~98.6 % tight.

        That is the case that needs the room, so a conservative-but-loose bound
        there would lengthen the prelude for nothing. ``s''`` peaks at
        tau = 0.2113 and ``h''`` at 0.2427, which is why the triangle inequality
        very nearly holds with equality.
        """
        # target above, hand moving down (v0 away from the target)
        delta, v0 = 1.0, -20.0
        T = smooth_move_accel_limited_duration_s(delta, v0)
        peak = smooth_move_peak_accel_rps2(delta, v0, T)
        assert peak / MAX_SMOOTH_MOVE_HAND_ACCEL_RPS2 > 0.98

    def test_the_duration_floor_can_only_reduce_the_peak(self):
        """``fmaxf(T, 0.05)`` must never make a profile harsher.

        The peak is monotone-decreasing in T (|a| <= |delta|*S2/T^2 +
        |v0|*H2/T), so raising T to the floor is always safe — but the floor is
        applied BEFORE the excursion check for exactly that reason, and a future
        edit reordering them would silently break this.
        """
        for delta, v0 in [(0.02, 0.0), (0.02, 7.0), (0.10, 8.0), (1.0, 12.0)]:
            T_raw = smooth_move_accel_limited_duration_s(delta, v0)
            if T_raw >= SMOOTH_MOVE_MIN_DURATION_S:
                continue
            assert (smooth_move_peak_accel_rps2(delta, v0, SMOOTH_MOVE_MIN_DURATION_S)
                    <= smooth_move_peak_accel_rps2(delta, v0, T_raw) + 1e-9)

    def test_a_nonzero_v0_always_lengthens_the_move(self):
        """Arresting v0 costs time; a shorter T at the same delta would raise |a|."""
        for delta in (0.0, 0.10, 2.2594, 9.9594):
            base = smooth_move_accel_limited_duration_s(delta, 0.0)
            for v0 in (6.01, 9.0, 20.0):
                assert smooth_move_accel_limited_duration_s(delta, v0) > base


class TestSmoothMoveExcursion:
    """The overshoot is CORRECT and must be allowed — but bounded by the stroke."""

    def test_excursion_matches_a_dense_sampling_of_the_emitted_profile(self):
        """The clamp must bound the curve actually emitted, not an approximation."""
        for start, target, v0 in [(_X3_REV, _X3_REV, 8.0),
                                  (_X3_REV - 0.02, _X3_REV, 7.0),
                                  (5.0, _X3_REV, -9.0),
                                  (1.0, 0.0, -7.5),
                                  (0.0, 5.0, 6.5)]:
            plan = plan_smooth_move(start, target, v0)
            samples = [plan.sample_rev(float(t))
                       for t in np.linspace(0.0, plan.duration_s, 20001)]
            assert min(samples) >= start + plan.excursion_lo_rev - 1e-6
            assert max(samples) <= start + plan.excursion_hi_rev + 1e-6

    def test_v0_away_from_the_target_overshoots_and_returns(self):
        """The behaviour the plan says must be ALLOWED, not suppressed."""
        # at the stroke top, still drifting upward: must go above and come back
        plan = plan_smooth_move(_X3_REV, _X3_REV, 8.0)
        assert plan.velocity_continuous
        assert plan.excursion_hi_rev > 0.0
        assert plan.sample_rev(plan.duration_s) == pytest.approx(_X3_REV, abs=1e-9)
        assert max(plan.sample_rev(float(t))
                   for t in np.linspace(0, plan.duration_s, 4001)) > _X3_REV

    def test_the_overshoot_is_bounded_by_v0_T_times_h_max(self):
        """``|v0|*T*16/81`` is the closed-form bound a host caller can size with.

        EXACT in the braking case (delta = 0), where the excursion's turning
        point is h's own maximum at tau = 1/3.  The generated
        ``QUINTIC_H_MAX = 0.19753086`` is 16/81 ROUNDED TO 8 DIGITS, so the
        agreement is to 2e-8 relative, not to machine precision — anything
        tighter would be pinning the rounding rather than the physics.
        """
        for v0 in (6.01, 8.0, 9.0):
            plan = plan_smooth_move(_X3_REV, _X3_REV, v0)
            exact = abs(v0) * plan.duration_s * (16.0 / 81.0)
            bound = abs(v0) * plan.duration_s * QUINTIC_H_MAX
            assert plan.excursion_hi_rev == pytest.approx(exact, rel=1e-12)
            assert plan.excursion_hi_rev == pytest.approx(bound, rel=1e-7)

    @pytest.mark.parametrize("v0", [-_V0_AT_RELEASE_RPS, -60.0, -20.0, -9.0,
                                    -6.01, 6.01, 9.0, 20.0, 60.0,
                                    _V0_AT_RELEASE_RPS])
    @pytest.mark.parametrize("start,target", [
        (_X3_REV, _X3_REV),              # braking at the stroke top
        (7.7004, _X3_REV),               # the measured mid-stroke truncation
        (_X3_REV, 6.1267),               # a catch descent prelude (x3 -> x5)
        (4.98, _X3_REV),                 # mid-prime re-dispatch
        (5.0, 0.0),                      # a SAFE_ABORT retract mid-descent
        (0.0, _X3_REV),                  # prime from the bottom
        (9.99, 0.0),                     # retract re-dispatched high and fast
        (1.23, 6.1267),                  # catch prelude near the bottom
    ])
    def test_the_planned_excursion_always_stays_inside_the_stroke(self, start,
                                                                  target, v0):
        """The end-stop assertion, over the whole reachable (start, target, v0) grid.

        The un-fixed system already reached 10.174-10.325 rev with as little as
        0.775 rev = 24.5 mm of headroom to the 11.1 rev overextension guard —
        which itself sits only 0.76 mm below the top of the 355 mm stroke.  Both
        bounds admit the endpoints, because a legal target (retract to 0.0) and a
        live mid-coast position can each already sit outside the band.

        The PHYSICAL assertions are the load-bearing ones and they are written
        against the physical limits — the overextension guard, and encoder zero as
        a bare ``0.0`` — NOT against the expressions ``plan_smooth_move`` itself
        evaluates.  An assertion that re-uses the code's own predicate is
        tautological on that side: it passes for any value of the clamp
        constants, including a floor moved a full rev below the bottom hard stop.
        (That is not hypothetical.  The floor shipped into this phase as
        ``HAND_HOME_ABS_POS_REV = -0.1`` rev — the bottom stop itself — and no
        assertion anywhere failed, because the grid had no case whose bulge
        landed between 0 and -0.1 rev.  ``(9.99, 0.0)`` and ``(1.23, 6.1267)``
        are here for exactly that: both are states ON a shipped profile, and both
        commanded troughs of -0.098 to -0.100 rev under the old floor.)
        """
        plan = plan_smooth_move(start, target, v0)
        peak = start + plan.excursion_hi_rev
        trough = start + plan.excursion_lo_rev
        # physical: never past the overextension guard, never below encoder zero
        assert peak <= max(HAND_MOTOR_MAX_POSITION_REVS, start, target) + 1e-9
        assert trough >= min(0.0, start, target) - 1e-3, plan
        # ...and the stop is a further 0.1 rev below that, with real margin now
        assert trough > HAND_HOME_ABS_POS_REV, plan
        # the clamp's own (tighter) working band
        assert peak <= max(_CEIL_REV, start, target) + 1e-9, plan

    def test_the_ceiling_is_converted_with_no_stroke_margin_term(self):
        """rev = mm/1000 * LINEAR_GAIN, no 20 mm inset.

        The sim CENTRES the 315 mm stroke in the 355 mm travel; the firmware
        starts it at zero.  Carrying the sim's 20 mm inset into the end-stop
        bound would put the ceiling 0.63 rev too high — 0.53 rev PAST the
        overextension guard, on a system already measured 0.775 rev from it.
        """
        assert HAND_MOTOR_MAX_POSITION_REVS == 11.1
        assert rev_to_mm(HAND_MOTOR_MAX_POSITION_REVS) == pytest.approx(351.08,
                                                                       abs=0.01)
        assert mm_to_rev(351.08) == pytest.approx(11.1, abs=1e-3)
        # the inset is NOT part of the mapping
        assert mm_to_rev(HAND_STROKE_M * 1000.0 - 2 * STROKE_MARGIN_M * 1000.0) \
            == pytest.approx(_X3_REV, abs=1e-9)


class TestSmoothMoveBranches:
    """Empty / velocity-continuous / fallback — and which one is reachable when."""

    def test_at_target_and_at_rest_is_still_empty(self):
        """Load-bearing, and just paid for.

        The prime was moved to the derived stroke top (9.9594 rev) and the catch
        arm gated to after the throw stroke so that a catch from rest opens with
        the smallest possible prelude.  Losing the empty branch re-introduces a
        prelude on every clean catch and silently undoes both.
        """
        plan = plan_smooth_move(_X3_REV, _X3_REV, 0.0)
        assert plan.empty
        assert plan.duration_s == 0.0
        assert plan.reason == 'at-target-and-at-rest'

    @pytest.mark.parametrize("v0", [0.0, 0.25, 1.82, 5.39,
                                    SMOOTH_MOVE_V0_DEADBAND_RPS])
    def test_dither_still_reads_as_at_rest(self, v0):
        """Measured parked |vel| on the signal the firmware reads.

        p99 = 5.39 rev/s at the top park / 1.82 at the bottom (2026-07-24), and
        the settle tail of every completed commanded move in the three
        2026-07-25 traces reads <= 0.25 rev/s in the +20..+70 ms window the
        gated catch arm lands in.  All of those must stay inside the dead-band:
        honouring them would command ~7 mm of excursion nobody asked for on a
        stationary hand.
        """
        assert plan_smooth_move(_X3_REV, _X3_REV, v0).empty
        assert plan_smooth_move(_X3_REV, _X3_REV, -v0).empty

    def test_the_deadband_sits_above_the_measured_dither(self):
        """5.39 rev/s p99 dither below, +9.2 rev/s slowest real traverse above."""
        assert 5.39 < SMOOTH_MOVE_V0_DEADBAND_RPS < 9.2

    def test_at_target_but_moving_brakes_instead_of_returning_empty(self):
        """The distinction that is the whole point of the phase.

        And it NARROWS the empty branch rather than widening it:
        ``Teensy_code.ino:472-475`` returns from the kind-3 handler BEFORE
        ``packedMsgs.clear()`` when the move is empty, so every case that used to
        return empty and is now moving produces a real trajectory instead — a
        strictly smaller hole in the only un-arm mechanism the Teensy offers.
        """
        for v0 in (6.01, 8.0, 20.0, 60.0, _V0_AT_RELEASE_RPS,
                   -6.01, -20.0, -_V0_AT_RELEASE_RPS):
            plan = plan_smooth_move(_X3_REV, _X3_REV, v0)
            assert not plan.empty, v0
            assert plan.duration_s >= SMOOTH_MOVE_MIN_DURATION_S, v0

    def test_a_brake_that_fits_is_velocity_continuous(self):
        """It starts at the live velocity, not at zero."""
        plan = plan_smooth_move(_X3_REV, _X3_REV, 8.0)
        assert plan.velocity_continuous
        assert plan.sample_vel_rps(0.0) == pytest.approx(8.0, abs=1e-9)
        assert plan.sample_vel_rps(plan.duration_s) == pytest.approx(0.0, abs=1e-9)
        # continuous in position too, at both seams
        assert plan.sample_rev(0.0) == pytest.approx(_X3_REV, abs=1e-12)
        assert plan.sample_rev(plan.duration_s) == pytest.approx(_X3_REV, abs=1e-9)

    def test_a_brake_that_cannot_fit_falls_back_and_says_so(self):
        """119.6 rev/s cannot be arrested inside the stroke at 100 rev/s^2.

        The physics, not a design choice: an accel-limited brake needs
        ``QUINTIC_H_MAX*H2*v0^2/a_max`` = 0.00778*v0^2 rev = **111 rev** of room
        at 119.6 rev/s, against 11.1 rev of total travel.  The fallback is
        today's rest-to-rest profile — never empty, never a refusal, and never a
        commanded magnitude the firmware could not already produce.
        """
        plan = plan_smooth_move(7.7004, _X3_REV, _V0_AT_RELEASE_RPS)
        assert not plan.velocity_continuous
        assert not plan.empty
        assert plan.reason == 'excursion-would-leave-the-stroke'
        assert plan.v0_seed_rps == 0.0
        assert plan.v0_measured_rps == _V0_AT_RELEASE_RPS
        # bit-identical to the pre-Phase-4 profile
        assert plan.duration_s == max(
            math.sqrt(abs(_X3_REV - 7.7004) * QUINTIC_S2_MAX
                      / MAX_SMOOTH_MOVE_HAND_ACCEL_RPS2),
            SMOOTH_MOVE_MIN_DURATION_S)
        assert plan.peak_accel_rps2 <= MAX_SMOOTH_MOVE_HAND_ACCEL_RPS2 * (1 + 1e-9)

    def test_the_fallback_is_never_empty_even_at_the_target(self):
        """At the target, moving fast, cannot fit: a hold, not nothing.

        This is the one combination that could have WIDENED the empty branch —
        fall back to rest-to-rest with delta ~ 0 and the old dead-band would
        return empty.  It must emit the floored hold instead.
        """
        plan = plan_smooth_move(_X3_REV, _X3_REV, _V0_AT_RELEASE_RPS)
        assert not plan.empty
        assert not plan.velocity_continuous
        assert plan.duration_s == pytest.approx(SMOOTH_MOVE_MIN_DURATION_S)
        assert plan.sample_rev(plan.duration_s / 2) == pytest.approx(_X3_REV,
                                                                     abs=1e-9)

    def test_the_retract_target_is_never_made_infeasible_by_the_lower_bound(self):
        """A SAFE_ABORT retract always lands on ``hand_retract_rev = 0.0``.

        It lands there for ANY floor value, because the bound is relaxed to
        ``min(FLOOR, start, target)`` and the target is one of the endpoints —
        which is why the original justification for giving the floor no margin
        ("a margin would put the shipped SAFE_ABORT target outside the band,
        making every abort retract infeasible by construction") was wrong, and
        why tightening the floor from the bottom hard stop to encoder zero cost
        nothing: the profiles that would have dived under simply take the
        rest-to-rest fallback, which is today's behaviour and lands on 0.0 too.
        """
        for v0 in (0.0, -3.0, -7.5, -30.0, -60.0):
            plan = plan_smooth_move(5.0, 0.0, v0)
            assert not plan.empty
            assert plan.sample_rev(plan.duration_s) == pytest.approx(0.0, abs=1e-9)
            assert plan.peak_accel_rps2 <= MAX_SMOOTH_MOVE_HAND_ACCEL_RPS2 * (1 + 1e-9)
            # and no sample of it is ever commanded below encoder zero
            n = 64
            assert min(plan.sample_rev(k * plan.duration_s / n)
                       for k in range(n + 1)) >= -1e-3, v0

    def test_the_floor_is_encoder_zero_not_the_bottom_hard_stop(self):
        """``Homing::HAND_ABS_POS_REV = -0.1`` rev IS the stop, not a bound.

        The axis homes DOWNWARD into it at -3 rev/s, so a clamp anchored there
        plans commanded travel onto the stop with zero allowance for the position
        loop's own undershoot (+0.186 rev measured on this profile family).  The
        host already refuses to command it: ``teensy_bridge_node`` rejects a
        smooth-move target below 0 and ``can/odrive.py`` clips a hand setpoint
        below 0 and warns — two enforcement points that must not disagree.

        Both cases below are states on a SHIPPED profile: 9.99 rev at -22.36
        rev/s sits on the full-stroke retract (peak -24.63 rev/s), and 1.23 rev
        at -11.96 rev/s on the descent to x5.  Under the -0.1 rev floor both were
        HONOURED, commanding -0.098 and -0.100 rev.
        """
        for start, target, v0 in ((9.99, 0.0, -22.36), (1.23, 6.1267, -11.96)):
            plan = plan_smooth_move(start, target, v0)
            trough = plan.start_rev + plan.excursion_lo_rev
            assert trough >= min(0.0, start, target) - 1e-3, plan
            # they now take the documented fallback rather than the stop
            assert not plan.velocity_continuous
            assert plan.reason == 'excursion-would-leave-the-stroke'

    def test_a_live_position_already_above_the_ceiling_is_still_servable(self):
        """A mid-coast reading above 10.6 rev most needs a continuous profile.

        If the ceiling were absolute rather than relaxed to admit the endpoints,
        this case would be infeasible by definition and always take the fallback
        — i.e. the fix would switch itself off exactly where the pre-fix system
        was measured (10.165-10.325 rev).
        """
        plan = plan_smooth_move(10.7, _X3_REV, -7.0)
        assert plan.velocity_continuous
        assert plan.sample_rev(plan.duration_s) == pytest.approx(_X3_REV, abs=1e-9)

    def test_the_continuity_band_is_narrow_and_that_is_reported(self):
        """How wide the affordable band actually is, at the two live geometries.

        Pinned because it is the operator's decision input: continuity is
        affordable only up to ~9 rev/s at the stroke top and ~21 rev/s from a
        mid-stroke freeze, and that ceiling is set by
        MAX_SMOOTH_MOVE_HAND_ACCEL_RPS2 = 100 rev/s^2 — a COMFORT limit, 19x
        below the 1908 rev/s^2 the throw profile itself commands at 3.93 m/s.

        These are the EXCURSION-limited figures.  The duration cap
        (:func:`smooth_move_max_duration_s`, pinned by
        ``test_an_honoured_prelude_never_outlasts_a_rest_to_rest_move``) binds
        first anywhere it is tighter, at 20.32 rev/s — so the effective mid-stroke
        band is 20.3, not 20.9.  Do not quote v_mid on its own.
        """
        k = QUINTIC_H_MAX * QUINTIC_H2_MAX / MAX_SMOOTH_MOVE_HAND_ACCEL_RPS2
        assert k == pytest.approx(0.0077832, rel=1e-4)
        v_top = math.sqrt((_CEIL_REV - _X3_REV) / k)
        v_mid = math.sqrt((HAND_MOTOR_MAX_POSITION_REVS - 7.7004) / k)
        assert v_top == pytest.approx(9.07, abs=0.05)
        assert v_mid == pytest.approx(20.90, abs=0.05)
        # and the boundary is real: just below fits, just above does not
        assert plan_smooth_move(_X3_REV, _X3_REV, v_top - 0.3).velocity_continuous
        assert not plan_smooth_move(_X3_REV, _X3_REV, v_top + 0.3).velocity_continuous

    def test_an_honoured_prelude_never_outlasts_a_rest_to_rest_move(self):
        """The SECOND cannot-fit test: duration, not just excursion.

        Arresting ``v0`` costs time as well as travel, and the duration grows
        linearly in ``|v0|`` while the excursion grows as ``v0**2`` — so
        mid-stroke, where there is room for a big bulge, the clamp alone lets a
        prelude run arbitrarily long.  Every host-side window that budgets for a
        commanded hand move is sized on the durations this firmware could already
        produce, and the binding one is
        ``catch_coordinator_node._PRIME_INFLIGHT_S = 1.2`` s (the anti-stutter
        window; a re-dispatch inside a live ascent is the 2026-07-23 stutter).

        The case that made it concrete: a PRIME dispatched into a live retract at
        the retract's own peak descent speed solves to 1.206 s — past the window
        outright, not merely past its 1.5x headroom.  It must now fall back.
        """
        cap = smooth_move_max_duration_s()
        # the cap IS the longest rest-to-rest move the stroke admits
        assert cap == pytest.approx(
            math.sqrt(HAND_MOTOR_MAX_POSITION_REVS * QUINTIC_S2_MAX
                      / MAX_SMOOTH_MOVE_HAND_ACCEL_RPS2), rel=1e-12)
        assert cap == pytest.approx(0.80054, abs=1e-4)

        prime = plan_smooth_move(5.04, _X3_REV, -24.62)
        assert not prime.velocity_continuous
        assert prime.reason == 'duration-would-outlast-a-rest-to-rest-move'
        assert prime.duration_s <= cap

        # no reachable (start, target, v0) is honoured above the cap
        for start, target in ((_X3_REV, _X3_REV), (7.7004, _X3_REV),
                              (5.0, 0.0), (0.0, _X3_REV), (4.98, _X3_REV)):
            for v0 in (-119.6, -60.0, -30.0, -20.0, -9.0, -6.01,
                       6.01, 9.0, 20.0, 30.0, 60.0, 119.6):
                plan = plan_smooth_move(start, target, v0)
                if plan.velocity_continuous and plan.v0_seed_rps != 0.0:
                    assert plan.duration_s <= cap + 1e-12, plan

    def test_the_deepest_honoured_brake_is_bounded_and_the_probe_knows_it(self):
        """What the bench instrument has to separate a brake from the descent by.

        ``tools/probes/hand_stroke_timeline._CATCH_DESC_ABOVE_X5_REV`` decides
        whether a downward command is the ARMED CATCH descent or a
        ``makeSmoothMove`` brake, and a brake read as the descent makes the
        end-stop ``peak`` row under-report — a false PASS on the row that guards
        the 11.1 rev limit.  Its threshold is sized against THIS number, so if the
        accel limit or the duration cap moves and this bound grows past
        ``x3 - (x5 + 0.33)`` = 3.502 rev, that separator must move with it.
        """
        cap = smooth_move_max_duration_s()
        v0_cap = MAX_SMOOTH_MOVE_HAND_ACCEL_RPS2 * cap / QUINTIC_H2_MAX
        deepest = v0_cap * cap * QUINTIC_H_MAX
        assert v0_cap == pytest.approx(20.32, abs=0.02)
        assert deepest == pytest.approx(3.213, abs=0.005)
        assert deepest < _X3_REV - (6.1267 + 0.33), (
            'the probe brake/descent separator no longer clears the deepest '
            'brake the firmware can honour')
        # and it is really reachable: honoured just under, fallback just over
        assert plan_smooth_move(_X3_REV, _X3_REV, -(v0_cap - 0.2)).velocity_continuous
        assert not plan_smooth_move(_X3_REV, _X3_REV,
                                    -(v0_cap + 0.2)).velocity_continuous


class TestSmoothMoveMmWrapperUnchanged:
    """The sim's own callers are rest-to-rest and must be untouched."""

    def test_default_start_velocity_reproduces_the_old_profile_bit_for_bit(self):
        """HandCatchSequence / HandThrowSequence do not track hand velocity."""
        for start, end in [(0.0, 335.0), (335.0, 100.0), (100.0, 300.0),
                           (244.0, 318.0)]:
            sm = HandSmoothMove(start, end)
            # The delta is differenced in the REV domain now (the firmware's own
            # frame) where the historical code differenced in mm, so the duration
            # can move by 1 ulp — 1.5e-16 s, nine orders of magnitude inside the
            # 3.2e-8 s xref tolerance.
            delta_rev = mm_to_rev(end) - mm_to_rev(start)
            want_T = max(math.sqrt(abs(delta_rev) * QUINTIC_S2_MAX
                                   / MAX_SMOOTH_MOVE_HAND_ACCEL_RPS2), 0.05)
            assert sm.duration == want_T
            historical = max(math.sqrt(abs((end - start) / 1000.0 * _LINEAR_GAIN)
                                       * QUINTIC_S2_MAX
                                       / MAX_SMOOTH_MOVE_HAND_ACCEL_RPS2), 0.05)
            assert sm.duration == pytest.approx(historical, rel=1e-15)
            assert sm.velocity_continuous
            for t in np.linspace(0.0, sm.duration, 201):
                tau = float(t) / sm.duration
                want = start + (end - start) * quintic_s(tau)
                assert sm.sample(float(t)) == pytest.approx(want, abs=1e-9)

    def test_a_moving_start_is_opt_in_and_honoured(self):
        sm = HandSmoothMove(300.0, 300.0, start_vel_mm_s=rev_to_mm(8.0))
        assert sm.duration > 0.0
        assert sm.velocity_continuous
        assert sm.sample_velocity(0.0) == pytest.approx(rev_to_mm(8.0), rel=1e-9)
        lo, hi = sm.excursion_mm()
        assert hi > 300.0 and lo == pytest.approx(300.0, abs=1e-6)
        assert sm.peak_accel_rps2 <= MAX_SMOOTH_MOVE_HAND_ACCEL_RPS2 * (1 + 1e-9)

    def test_catch_and_throw_sequences_are_unchanged(self):
        """The prelude the sim's sequences build must still be rest-to-rest."""
        seq = HandCatchSequence(event_vel_mps=3.0, arrival_time=5.0,
                                current_pos_mm=323.0)
        assert seq.prelude.start_vel_mm_s == 0.0
        assert seq.prelude.velocity_continuous


# ── HandCatchSequence ────────────────────────────────────────────────────

class TestCatchSequence:
    """Verify the full catch sequence (smooth-move prelude + catch)."""

    def test_sequence_timing_no_gaps(self):
        """Smooth-move should end exactly when catch trajectory starts."""
        seq = HandCatchSequence(
            event_vel_mps=3.0,
            arrival_time=2.0,
            current_pos_mm=323.0,
        )
        # Position should be continuous at the transition
        catch_start = seq.arrival_time + seq.catch_trajectory.start_time
        p_before = seq.sample(catch_start - 0.001)
        p_after = seq.sample(catch_start + 0.001)
        # Should be close (smooth-move ends at catch-trajectory start position)
        assert abs(p_before - p_after) < 2.0, \
            f"Discontinuity at prelude→catch transition: " \
            f"{p_before:.1f} → {p_after:.1f}"

    def test_returns_none_after_complete(self):
        """sample() returns None after the sequence ends."""
        seq = HandCatchSequence(
            event_vel_mps=3.0,
            arrival_time=2.0,
            current_pos_mm=323.0,
        )
        assert seq.sample(seq.end_time + 1.0) is None

    def test_before_prelude_returns_start(self):
        """Before prelude starts, returns the smooth-move start position."""
        seq = HandCatchSequence(
            event_vel_mps=3.0,
            arrival_time=5.0,  # far in the future
            current_pos_mm=323.0,
        )
        pos = seq.sample(0.0)
        assert pos is not None
        assert abs(pos - 323.0) < 1.0


class TestCatchSequenceFeasibility:
    """Verify timing budget check (try_create)."""

    def test_feasible_with_enough_time(self):
        """Should succeed when there's plenty of time."""
        result = HandCatchSequence.try_create(
            event_vel_mps=3.0,
            arrival_time=5.0,
            current_pos_mm=323.0,
            current_time=0.0,
        )
        assert result.feasible
        assert result.sequence is not None

    def test_infeasible_when_too_late(self):
        """Should fail when arrival is imminent (no time for smooth-move)."""
        result = HandCatchSequence.try_create(
            event_vel_mps=3.0,
            arrival_time=0.1,  # only 100ms away
            current_pos_mm=100.0,  # far from prime — needs long smooth-move
            current_time=0.0,
        )
        assert not result.feasible
        assert result.sequence is None
        assert 'Insufficient time' in result.reason

    def test_feasible_when_already_at_prime(self):
        """Should succeed even with short lead time if hand is near prime."""
        # If hand is already near the catch trajectory start, the
        # smooth-move is near-zero duration, so tight timing works.
        result = HandCatchSequence.try_create(
            event_vel_mps=3.0,
            arrival_time=1.0,
            current_pos_mm=335.0,  # already at top of effective stroke
            current_time=0.0,
        )
        assert result.feasible


class TestCatchTrajectoryTeensyMatch:
    """Cross-check against hand-computed Teensy values for event_vel=3.0 m/s."""

    def test_catch_velocity(self):
        """vC = -CATCH_VEL_RATIO * 3.0."""
        traj = HandCatchTrajectory(event_vel_mps=3.0)
        expected = -CATCH_VEL_RATIO * 3.0
        assert abs(traj.catch_velocity_mps - expected) < 1e-6

    def test_segment_durations(self):
        """Verify segment timing matches Teensy calcCatch() for v_throw=3.0."""
        # Manually compute:
        vC = -CATCH_VEL_RATIO * 3.0  # -1.08 m/s at catch_vel_ratio 0.6
        irC = 1.0 / INERTIA_RATIO     # 1/0.747 ≈ 1.3387
        totalStroke = _TOTAL_STROKE_M   # 0.315 m
        velH = CATCH_VEL_HOLD_PCT * totalStroke  # 0.0315 m
        accS = totalStroke - velH                  # 0.2835 m

        t_acc = -(2.0 / (irC + 1.0)) * accS / vC
        t_vel = -velH / vC
        t_dec = t_acc * irC

        traj = HandCatchTrajectory(event_vel_mps=3.0)

        # Total duration = t_acc + t_vel + t_dec + END_PROFILE_HOLD
        expected_duration = t_acc + t_vel + t_dec + END_PROFILE_HOLD_S
        assert abs(traj.duration - expected_duration) < 1e-6, \
            f"Duration {traj.duration:.4f}s, expected {expected_duration:.4f}s"

    def test_start_pos_is_top_of_effective_stroke(self):
        """Default start position = STROKE_MARGIN + TOTAL_STROKE in mm."""
        traj = HandCatchTrajectory(event_vel_mps=3.0)
        expected = (STROKE_MARGIN_M + _TOTAL_STROKE_M) * 1000.0
        assert abs(traj.start_pos_mm - expected) < 0.01
