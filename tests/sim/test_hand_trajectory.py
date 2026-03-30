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
    MIN_EVENT_VEL_MPS,
    MAX_EVENT_VEL_MPS,
    _TOTAL_STROKE_M,
    _TOTAL_STROKE_MM,
    _LINEAR_GAIN,
)


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
        vC = -CATCH_VEL_RATIO * 3.0  # -1.8 m/s
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
