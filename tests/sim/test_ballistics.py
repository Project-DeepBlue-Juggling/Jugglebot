"""Tests for ballistic utilities (sim/hand/ballistics.py).

Tests inverse ballistics, orientation computation, and hand offset.
"""

from __future__ import annotations

import pytest
import numpy as np

from hand.ballistics import (
    compute_launch_velocity,
    compute_arrival_velocity,
    compute_orientation,
    compute_hand_offset_mm,
    rodrigues,
)


class TestBallisticInverse:
    """Tests for compute_launch_velocity and compute_arrival_velocity."""

    def test_vertical_throw(self):
        """Throw straight up, catch at same XY — velocity is purely vertical."""
        v = compute_launch_velocity(
            np.array([0.0, 0.0, 800.0]),
            np.array([0.0, 0.0, 800.0]),
            flight_time_s=1.0,
        )
        assert v[0] == pytest.approx(0.0, abs=1.0)
        assert v[1] == pytest.approx(0.0, abs=1.0)
        assert v[2] > 0  # Must launch upward
        # For symmetric throw: v_z = 0.5 * g * T = 0.5 * 9806 * 1.0 = 4903
        assert v[2] == pytest.approx(4903.0, abs=1.0)

    def test_angled_throw(self):
        """Throw at an angle — has both horizontal and vertical components."""
        v = compute_launch_velocity(
            np.array([0.0, 0.0, 800.0]),
            np.array([100.0, 0.0, 800.0]),
            flight_time_s=1.0,
        )
        assert v[0] == pytest.approx(100.0, abs=1.0)  # dx/dt
        assert v[2] > 0  # Still launches upward for same-height catch

    def test_round_trip(self):
        """launch_vel → arrival_vel → inverse gives consistent velocities."""
        release = np.array([0.0, 0.0, 800.0])
        target = np.array([50.0, -30.0, 750.0])
        T = 1.5

        v_launch = compute_launch_velocity(release, target, T)
        v_arrival = compute_arrival_velocity(v_launch, T)

        # Verify arrival position matches target
        # pos = release + v_launch * T + 0.5 * g * T^2
        pos_check = release + v_launch * T + np.array([0, 0, -0.5 * 9806.0 * T**2])
        np.testing.assert_allclose(pos_check, target, atol=1.0)

    def test_short_flight(self):
        """Short flight time (0.3s) — higher launch speed needed."""
        v = compute_launch_velocity(
            np.array([0.0, 0.0, 800.0]),
            np.array([0.0, 0.0, 800.0]),
            flight_time_s=0.3,
        )
        # v_z = 0.5 * g * T for symmetric; shorter T = smaller v_z? No,
        # v_z = 0.5 * 9806 * 0.3 = 1470.9
        assert v[2] == pytest.approx(0.5 * 9806.0 * 0.3, abs=1.0)

    def test_long_flight(self):
        """Long flight time (3.0s) — higher launch speed needed."""
        v = compute_launch_velocity(
            np.array([0.0, 0.0, 800.0]),
            np.array([0.0, 0.0, 800.0]),
            flight_time_s=3.0,
        )
        assert v[2] == pytest.approx(0.5 * 9806.0 * 3.0, abs=1.0)


class TestOrientation:
    """Tests for compute_orientation."""

    def test_vertical_no_tilt(self):
        """Upward launch → zero rotation."""
        rv = compute_orientation(np.array([0.0, 0.0, 1.0]))
        np.testing.assert_allclose(rv, [0, 0, 0], atol=1e-6)

    def test_angled_tilt(self):
        """20° off vertical → correct rotation vector magnitude."""
        angle = np.radians(20)
        # Direction 20° off vertical in the X-Z plane
        direction = np.array([np.sin(angle), 0.0, np.cos(angle)])
        rv = compute_orientation(direction)
        assert np.linalg.norm(rv) == pytest.approx(angle, abs=1e-6)

    def test_exceeds_limit(self):
        """> 30° raises ValueError."""
        angle = np.radians(35)
        direction = np.array([np.sin(angle), 0.0, np.cos(angle)])
        with pytest.raises(ValueError, match="exceeds"):
            compute_orientation(direction)

    def test_catch_orientation(self):
        """-v_arrival gives correct catch tilt."""
        # Ball arriving at 20° from vertical
        angle = np.radians(15)
        v_arrival = np.array([np.sin(angle) * 5000, 0.0, -np.cos(angle) * 5000])
        rv = compute_orientation(-v_arrival)
        assert np.linalg.norm(rv) == pytest.approx(angle, abs=1e-3)

    def test_tilt_axis_perpendicular(self):
        """Tilt axis is perpendicular to both reference Z and target direction."""
        direction = np.array([1.0, 0.0, 2.0])
        rv = compute_orientation(direction)
        axis = rv / np.linalg.norm(rv)
        ref = np.array([0, 0, 1])
        d_hat = direction / np.linalg.norm(direction)
        # Axis should be perpendicular to both ref and d_hat
        assert abs(np.dot(axis, ref)) < 1e-6
        assert abs(np.dot(axis, d_hat)) < 1e-6


class TestHandOffset:
    """Tests for compute_hand_offset_mm.

    offset = HAND_AXIS_BOTTOM_OFFSET + hand_pos + BALL_SEAT_OFFSET
           = -129.0 + hand_pos + 44.4
    """

    def test_at_home(self):
        """Hand at 0mm → offset = -129 + 0 + 44.4 = -84.6 mm."""
        assert compute_hand_offset_mm(0.0) == pytest.approx(-84.6)

    def test_at_prime(self):
        """Hand at prime (~323mm) → offset = -129 + 323 + 44.4 = 238.4 mm."""
        assert compute_hand_offset_mm(323.0) == pytest.approx(238.4)

    def test_at_throw_release(self):
        """Correct for typical x2 position."""
        # At x2 ~207mm (from 3 m/s throw), offset = -129 + 207 + 44.4 = 122.4mm
        assert compute_hand_offset_mm(207.0) == pytest.approx(122.4)


class TestRodrigues:
    """Tests for the Rodrigues rotation helper."""

    def test_identity(self):
        """Zero rotation vector gives identity matrix."""
        R = rodrigues(np.zeros(3))
        np.testing.assert_allclose(R, np.eye(3), atol=1e-10)

    def test_90_deg_z(self):
        """90° rotation about Z axis."""
        rv = np.array([0.0, 0.0, np.pi / 2])
        R = rodrigues(rv)
        # Should map [1,0,0] → [0,1,0]
        result = R @ np.array([1, 0, 0])
        np.testing.assert_allclose(result, [0, 1, 0], atol=1e-10)
