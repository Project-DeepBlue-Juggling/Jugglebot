"""Tests for ballistics.py — landing prediction quadratic solver."""

import math

import numpy as np
import pytest

from jugglebot.tracking.ballistics import GRAVITY_MMPS2, predict_landing_state


class TestVerticalThrow:
    """Ball thrown straight up from above the landing plane."""

    def test_simple_drop(self):
        """Ball at rest above plane: lands at same XY, with downward velocity."""
        pos = np.array([0.0, 0.0, 1000.0])
        vel = np.array([0.0, 0.0, 0.0])
        landing_z = 734.3

        result = predict_landing_state(pos, vel, landing_z)
        assert result is not None
        landing_pos, landing_vel, ttl = result

        # Analytical: h = 0.5*g*t^2 => t = sqrt(2*dz/g)
        dz = 1000.0 - 734.3
        expected_ttl = math.sqrt(2.0 * dz / GRAVITY_MMPS2)
        assert ttl == pytest.approx(expected_ttl, rel=1e-6)

        # Lands at same XY
        assert landing_pos[0] == pytest.approx(0.0)
        assert landing_pos[1] == pytest.approx(0.0)
        assert landing_pos[2] == pytest.approx(landing_z)

        # Landing velocity: purely downward
        expected_vz = -GRAVITY_MMPS2 * expected_ttl
        assert landing_vel[2] == pytest.approx(expected_vz, rel=1e-6)

    def test_thrown_upward(self):
        """Ball thrown upward from above the landing plane.

        Should return the later crossing (descending), not the earlier
        (ascending) solution which would be negative or non-physical.
        """
        pos = np.array([0.0, 0.0, 800.0])
        vel = np.array([0.0, 0.0, 2000.0])  # Upward at 2 m/s
        landing_z = 734.3

        result = predict_landing_state(pos, vel, landing_z)
        assert result is not None
        _, landing_vel, ttl = result

        # Ball goes up first, then comes back down past landing_z
        assert ttl > 0
        # At landing, ball must be moving downward
        assert landing_vel[2] < 0


class TestAngledThrow:
    """Ball thrown at an angle — verify XY landing position."""

    def test_45_degree_throw(self):
        """Classic 45° throw from above landing plane."""
        speed = 3000.0  # mm/s
        angle = math.radians(45)
        pos = np.array([0.0, 0.0, 1000.0])
        vel = np.array([speed * math.cos(angle), 0.0, speed * math.sin(angle)])
        landing_z = 734.3

        result = predict_landing_state(pos, vel, landing_z)
        assert result is not None
        landing_pos, landing_vel, ttl = result

        # Verify analytically
        vx, vz = vel[0], vel[2]
        dz = pos[2] - landing_z
        # -0.5*g*t^2 + vz*t + dz = 0 => 0.5*g*t^2 - vz*t - dz = 0
        a = 0.5 * GRAVITY_MMPS2
        b = -vz
        c = -dz
        disc = b * b - 4 * a * c
        t_analytical = (-b + math.sqrt(disc)) / (2 * a)

        assert ttl == pytest.approx(t_analytical, rel=1e-6)
        assert landing_pos[0] == pytest.approx(vx * t_analytical, rel=1e-6)
        assert landing_pos[1] == pytest.approx(0.0)

        # Horizontal velocity unchanged
        assert landing_vel[0] == pytest.approx(vx, rel=1e-9)

    def test_xy_throw(self):
        """Throw with both X and Y components."""
        pos = np.array([100.0, -200.0, 900.0])
        vel = np.array([1500.0, -800.0, 500.0])
        landing_z = 734.3

        result = predict_landing_state(pos, vel, landing_z)
        assert result is not None
        landing_pos, landing_vel, ttl = result

        # Check XY: x_land = x0 + vx*t, y_land = y0 + vy*t
        assert landing_pos[0] == pytest.approx(pos[0] + vel[0] * ttl, rel=1e-6)
        assert landing_pos[1] == pytest.approx(pos[1] + vel[1] * ttl, rel=1e-6)
        # Horizontal velocity preserved
        assert landing_vel[0] == pytest.approx(vel[0], rel=1e-9)
        assert landing_vel[1] == pytest.approx(vel[1], rel=1e-9)


class TestEdgeCases:
    """Edge cases: below plane, no solution, already at plane."""

    def test_below_landing_plane_moving_down(self):
        """Ball already below the landing plane and moving down → no solution."""
        pos = np.array([0.0, 0.0, 700.0])
        vel = np.array([0.0, 0.0, -1000.0])  # Moving down
        landing_z = 734.3

        result = predict_landing_state(pos, vel, landing_z)
        # Ball is below and going further down — never reaches plane
        assert result is None

    def test_below_landing_plane_moving_up(self):
        """Ball below landing plane but moving up fast enough to cross it."""
        pos = np.array([0.0, 0.0, 700.0])
        vel = np.array([0.0, 0.0, 3000.0])  # Fast upward
        landing_z = 734.3

        result = predict_landing_state(pos, vel, landing_z)
        assert result is not None
        _, _, ttl = result
        assert ttl > 0

    def test_exactly_at_plane_moving_down(self):
        """Ball at landing plane moving down → no future positive crossing."""
        pos = np.array([0.0, 0.0, 734.3])
        vel = np.array([0.0, 0.0, -5000.0])
        landing_z = 734.3

        result = predict_landing_state(pos, vel, landing_z)
        # Already at the plane and moving away from it
        assert result is None

    def test_no_real_solution(self):
        """Ball far below plane with insufficient upward velocity."""
        pos = np.array([0.0, 0.0, 100.0])
        vel = np.array([0.0, 0.0, 100.0])  # Barely moving up
        landing_z = 10000.0  # Way above

        result = predict_landing_state(pos, vel, landing_z)
        assert result is None
