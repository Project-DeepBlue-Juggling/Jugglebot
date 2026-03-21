"""Tests for hand throw trajectory generator (sim/hand/trajectory.py).

Tests HandThrowTrajectory, HandThrowSequence, and max_throw_speed_mps().
"""

from __future__ import annotations

import math
import os
import sys

import pytest
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from hand.trajectory import (
    HandThrowTrajectory,
    HandThrowSequence,
    HandThrowResult,
    max_throw_speed_mps,
    MIN_EVENT_VEL_MPS,
    MAX_EVENT_VEL_MPS,
    STROKE_MARGIN_MM,
    _TOTAL_STROKE_MM,
    INERTIA_RATIO,
    THROW_VEL_HOLD_PCT,
    _TOTAL_STROKE_M,
)


class TestHandThrowTrajectory:
    """Unit tests for the 3-segment throw trajectory."""

    def test_vertical_throw_3mps(self):
        """Positions and velocities match Teensy calcThrow() math at 3 m/s."""
        t = HandThrowTrajectory(3.0)

        # Check computed segment durations match Teensy formula
        accel_stroke = (1 - THROW_VEL_HOLD_PCT) * _TOTAL_STROKE_M
        vel_hold = THROW_VEL_HOLD_PCT * _TOTAL_STROKE_M
        expected_t_acc = 2.0 / (INERTIA_RATIO + 1.0) * accel_stroke / 3.0
        expected_t_vel = vel_hold / 3.0
        expected_t_dec = expected_t_acc * INERTIA_RATIO

        assert t.start_time == pytest.approx(-(expected_t_acc + expected_t_vel), abs=1e-6)
        assert t.end_time == pytest.approx(expected_t_dec, abs=1e-6)

    def test_release_at_t_zero(self):
        """sample(0.0) returns release_pos_mm."""
        t = HandThrowTrajectory(3.0)
        assert t.sample(0.0) == pytest.approx(t.release_pos_mm, abs=0.01)

    def test_velocity_at_release(self):
        """velocity_at(0.0) equals throw_speed."""
        t = HandThrowTrajectory(3.0)
        assert t.velocity_at(0.0) == pytest.approx(3.0, abs=1e-6)

    def test_starts_at_bottom(self):
        """sample(start_time) returns start_pos_mm."""
        t = HandThrowTrajectory(3.0)
        assert t.sample(t.start_time) == pytest.approx(t.start_pos_mm, abs=0.01)

    def test_deceleration_to_zero(self):
        """velocity_at(end_time) is approximately zero."""
        t = HandThrowTrajectory(3.0)
        assert t.velocity_at(t.end_time) == pytest.approx(0.0, abs=0.01)

    def test_speed_clamping_low(self):
        """Speeds below MIN are clamped."""
        t = HandThrowTrajectory(0.1)
        assert t.release_speed_mps == pytest.approx(MIN_EVENT_VEL_MPS)

    def test_speed_clamping_high(self):
        """Speeds above MAX are clamped."""
        t = HandThrowTrajectory(20.0)
        assert t.release_speed_mps == pytest.approx(MAX_EVENT_VEL_MPS)

    def test_max_throw_speed(self):
        """max_throw_speed_mps() returns MAX_EVENT_VEL_MPS."""
        assert max_throw_speed_mps() == MAX_EVENT_VEL_MPS

    def test_default_start_at_stroke_margin(self):
        """Default start position is at STROKE_MARGIN_MM."""
        t = HandThrowTrajectory(3.0)
        assert t.start_pos_mm == pytest.approx(STROKE_MARGIN_MM, abs=0.01)

    def test_custom_start_pos(self):
        """Custom start position is respected."""
        t = HandThrowTrajectory(3.0, start_pos_mm=100.0)
        assert t.start_pos_mm == pytest.approx(100.0)

    def test_end_pos_above_start(self):
        """End position is above start (hand moves upward)."""
        t = HandThrowTrajectory(3.0)
        assert t.end_pos_mm > t.start_pos_mm

    def test_release_pos_between_start_and_end(self):
        """Release position is between start and end."""
        t = HandThrowTrajectory(3.0)
        assert t.start_pos_mm < t.release_pos_mm < t.end_pos_mm

    def test_monotonic_position(self):
        """Position increases monotonically during throw (upward motion)."""
        t = HandThrowTrajectory(3.0)
        times = np.linspace(t.start_time, t.end_time, 100)
        positions = [t.sample(ti) for ti in times]
        for i in range(1, len(positions)):
            assert positions[i] >= positions[i-1] - 0.001, \
                f"Position decreased at t={times[i]:.4f}"


class TestHandThrowSequence:
    """Tests for the complete throw sequence (smooth-move + throw)."""

    def test_smooth_move_prelude(self):
        """Transitions from current position to throw start."""
        seq = HandThrowSequence(3.0, release_time=2.0, current_pos_mm=150.0)
        # Prelude should move from 150mm to throw start (STROKE_MARGIN_MM)
        assert seq.prelude_start_time < 2.0

    def test_release_timing(self):
        """release_time matches input."""
        seq = HandThrowSequence(3.0, release_time=5.0, current_pos_mm=0.0)
        assert seq.release_time == 5.0

    def test_feasibility_check_passes(self):
        """try_create() succeeds with sufficient time budget."""
        result = HandThrowSequence.try_create(
            throw_speed_mps=3.0,
            release_time=5.0,
            current_pos_mm=0.0,
            current_time=0.0,
        )
        assert result.feasible
        assert result.sequence is not None

    def test_feasibility_check_rejects(self):
        """try_create() rejects when time budget is insufficient."""
        result = HandThrowSequence.try_create(
            throw_speed_mps=3.0,
            release_time=0.05,  # Way too soon
            current_pos_mm=300.0,  # Far from start
            current_time=0.0,
        )
        assert not result.feasible
        assert result.sequence is None
        assert 'Insufficient' in result.reason

    def test_end_position(self):
        """Hand stops at end_pos_mm after throw."""
        seq = HandThrowSequence(3.0, release_time=2.0, current_pos_mm=0.0)
        assert seq.end_pos_mm > 0

    def test_sample_returns_none_after_end(self):
        """sample() returns None after sequence completes."""
        seq = HandThrowSequence(3.0, release_time=2.0, current_pos_mm=0.0)
        assert seq.sample(seq.end_time + 1.0) is None

    def test_sample_at_release(self):
        """sample(release_time) returns release position."""
        seq = HandThrowSequence(3.0, release_time=2.0, current_pos_mm=20.0)
        pos = seq.sample(2.0)
        assert pos == pytest.approx(seq.throw_trajectory.release_pos_mm, abs=0.1)
