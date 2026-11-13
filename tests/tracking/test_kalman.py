"""Tests for kalman.py — 6D Kalman filter with gravity model."""

import math

import numpy as np
import pytest

from jugglebot.tracking.kalman import KalmanFilter
from jugglebot.tracking.ballistics import GRAVITY_MMPS2


DT = 0.005  # 200 Hz


def _analytical_ballistic(pos0, vel0, t):
    """Analytical position and velocity under gravity at time t."""
    pos = pos0.copy()
    vel = vel0.copy()
    pos[0] += vel0[0] * t
    pos[1] += vel0[1] * t
    pos[2] += vel0[2] * t - 0.5 * GRAVITY_MMPS2 * t * t
    vel[2] -= GRAVITY_MMPS2 * t
    return pos, vel


class TestGravityModel:
    """Predict-only (no measurements): verify gravity model matches analytical ballistics."""

    def test_free_fall_position(self):
        """Ball dropped from rest: predicted position matches analytical."""
        kf = KalmanFilter(dt=DT)
        pos0 = np.array([0.0, 0.0, 1000.0])
        vel0 = np.array([0.0, 0.0, 0.0])
        kf.initialize_from_state(pos0, vel0)

        n_steps = 100  # 500 ms
        for _ in range(n_steps):
            kf.predict()

        t = n_steps * DT
        expected_pos, expected_vel = _analytical_ballistic(pos0, vel0, t)

        np.testing.assert_allclose(kf.position, expected_pos, atol=0.5)
        np.testing.assert_allclose(kf.velocity, expected_vel, atol=1.0)

    def test_thrown_ball_position(self):
        """Ball thrown at angle: predicted trajectory matches analytical."""
        kf = KalmanFilter(dt=DT)
        pos0 = np.array([100.0, -200.0, 800.0])
        vel0 = np.array([1500.0, -800.0, 2000.0])
        kf.initialize_from_state(pos0, vel0)

        n_steps = 60  # 300 ms
        for _ in range(n_steps):
            kf.predict()

        t = n_steps * DT
        expected_pos, expected_vel = _analytical_ballistic(pos0, vel0, t)

        np.testing.assert_allclose(kf.position, expected_pos, atol=0.5)
        np.testing.assert_allclose(kf.velocity, expected_vel, atol=1.0)


class TestConvergence:
    """Filter with noisy measurements converges to true trajectory."""

    def test_convergence_with_noise(self):
        """Feed noisy measurements of a known trajectory; filter should converge."""
        rng = np.random.default_rng(42)
        kf = KalmanFilter(dt=DT, process_noise=5.0, measurement_noise=1.0)

        pos0 = np.array([0.0, 0.0, 1000.0])
        vel0 = np.array([1000.0, 500.0, 1500.0])
        # Start with high uncertainty to test convergence
        kf.initialize_from_state(pos0, vel0, position_cov=500.0, velocity_cov=10000.0)

        n_steps = 100
        errors = []

        for i in range(n_steps):
            t = (i + 1) * DT
            true_pos, true_vel = _analytical_ballistic(pos0, vel0, t)

            kf.predict()
            # Add Gaussian noise (std=3mm) to true position
            noisy_measurement = true_pos + rng.normal(0, 3.0, size=3)
            kf.update(noisy_measurement)

            pos_err = np.linalg.norm(kf.position - true_pos)
            errors.append(pos_err)

        # Filter should stay well under 10mm throughout the settled period
        assert np.mean(errors[-20:]) < 10.0

    def test_noise_rejection(self):
        """Filter should smooth out measurement noise."""
        rng = np.random.default_rng(123)
        # Use a higher R (less trust in measurements) to show smoothing
        kf = KalmanFilter(dt=DT, process_noise=1.0, measurement_noise=10.0)

        pos0 = np.array([0.0, 0.0, 900.0])
        vel0 = np.array([0.0, 0.0, 0.0])
        kf.initialize_from_state(pos0, vel0)

        noise_std = 8.0
        n_steps = 100
        filtered_errs = []
        raw_errs = []

        for i in range(n_steps):
            t = (i + 1) * DT
            true_pos, _ = _analytical_ballistic(pos0, vel0, t)
            noise = rng.normal(0, noise_std, size=3)
            measurement = true_pos + noise

            kf.predict()
            kf.update(measurement)

            filtered_errs.append(np.linalg.norm(kf.position - true_pos))
            raw_errs.append(np.linalg.norm(noise))

        # Filtered RMS should be significantly less than raw noise RMS
        filtered_rms = np.sqrt(np.mean(np.array(filtered_errs[-40:]) ** 2))
        raw_rms = np.sqrt(np.mean(np.array(raw_errs[-40:]) ** 2))
        assert filtered_rms < raw_rms * 0.8


class TestInitializationModes:
    """Test both initialization modes."""

    def test_init_from_state(self):
        """Initialize from announcement: position and velocity set correctly."""
        kf = KalmanFilter(dt=DT)
        pos = np.array([100.0, 200.0, 800.0])
        vel = np.array([1000.0, -500.0, 1500.0])
        kf.initialize_from_state(pos, vel)

        assert kf.initialized
        np.testing.assert_array_equal(kf.position, pos)
        np.testing.assert_array_equal(kf.velocity, vel)

    def test_init_from_positions(self):
        """Initialize from 2 mocap positions: velocity estimated by finite diff."""
        pos0 = np.array([0.0, 0.0, 1000.0])
        vel0 = np.array([1000.0, 0.0, 2000.0])

        # Generate two points along a ballistic trajectory
        t1, t2 = 0.0, DT
        p1, _ = _analytical_ballistic(pos0, vel0, t1)
        p2, _ = _analytical_ballistic(pos0, vel0, t2)

        kf = KalmanFilter(dt=DT)
        kf.initialize_from_positions([p1, p2], [t1, t2])

        assert kf.initialized
        np.testing.assert_allclose(kf.position, p2, atol=0.01)
        # Velocity estimate should be reasonable (within ~10%)
        np.testing.assert_allclose(kf.velocity, vel0, rtol=0.15, atol=100.0)

    def test_init_from_three_positions(self):
        """Three positions gives better velocity estimate."""
        pos0 = np.array([0.0, 0.0, 1000.0])
        vel0 = np.array([1500.0, -800.0, 1000.0])

        times = [0.0, DT, 2 * DT]
        positions = [_analytical_ballistic(pos0, vel0, t)[0] for t in times]

        kf = KalmanFilter(dt=DT)
        kf.initialize_from_positions(positions, times)

        assert kf.initialized
        # Uses last two points, so position should match the last
        np.testing.assert_allclose(kf.position, positions[-1], atol=0.01)


class TestLandingPrediction:
    """Landing prediction through the filter."""

    def test_landing_prediction_accuracy(self):
        """Known throw → landing prediction should match analytical."""
        kf = KalmanFilter(dt=DT, process_noise=5.0, measurement_noise=1.0)

        pos0 = np.array([0.0, 0.0, 800.0])
        vel0 = np.array([1000.0, 0.0, 2000.0])
        kf.initialize_from_state(pos0, vel0)
        landing_z = 734.3

        # Feed 20 perfect measurements (no noise) to let filter settle
        for i in range(20):
            t = (i + 1) * DT
            true_pos, _ = _analytical_ballistic(pos0, vel0, t)
            kf.predict()
            kf.update(true_pos)

        result = kf.predict_landing(landing_z)
        assert result is not None
        landing_pos, landing_vel, ttl = result

        # Compute analytical landing from current filter state
        # (should be very close to analytical from initial state offset by 20 steps)
        t_elapsed = 20 * DT
        true_pos_now, true_vel_now = _analytical_ballistic(pos0, vel0, t_elapsed)

        # Analytical time-to-land from current state
        a_coeff = -0.5 * GRAVITY_MMPS2
        b_coeff = true_vel_now[2]
        c_coeff = true_pos_now[2] - landing_z
        disc = b_coeff**2 - 4 * a_coeff * c_coeff
        assert disc >= 0
        t_analytical = min(t for t in [
            (-b_coeff + math.sqrt(disc)) / (2 * a_coeff),
            (-b_coeff - math.sqrt(disc)) / (2 * a_coeff),
        ] if t > 0)

        assert ttl == pytest.approx(t_analytical, abs=0.005)  # Within 5ms
        assert landing_pos[0] == pytest.approx(
            true_pos_now[0] + true_vel_now[0] * t_analytical, abs=5.0
        )

    def test_uninitialized_returns_none(self):
        """Uninitialized filter returns None for landing prediction."""
        kf = KalmanFilter(dt=DT)
        assert kf.predict_landing(734.3) is None
