"""6D Kalman filter for ball tracking — pure Python.

State: [x, y, z, vx, vy, vz] in mm and mm/s.
Process model: constant velocity + gravity control input.
Measurement: 3D position [x, y, z] from mocap markers.

Lifted and cleaned up from the retired kalman_filter.py (now
attic/ros-jugglebot-archived/).
"""
from __future__ import annotations

import numpy as np
from typing import Optional, Tuple

from .ballistics import GRAVITY_MMPS2, predict_landing_state


class KalmanFilter:
    """6D Kalman filter for a single ball under gravity.

    Two initialization modes:
      1. From announcement: initialize_from_state() with known pos+vel.
      2. From mocap: initialize_from_positions() with 2-3 marker positions.
    """

    # Measurement matrix: observe [x, y, z] from state [x, y, z, vx, vy, vz]
    H = np.array([
        [1, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
    ], dtype=np.float64)

    def __init__(
        self,
        dt: float,
        process_noise: float = 5.0,
        measurement_noise: float = 1.0,
    ):
        """
        Args:
            dt: Nominal time step between measurements (seconds).
            process_noise: Diagonal process noise variance.
            measurement_noise: Diagonal measurement noise variance.
        """
        self.dt = dt
        self.process_noise = process_noise
        self.measurement_noise = measurement_noise

        # State vector [x, y, z, vx, vy, vz]
        self.state = np.zeros(6)
        self.covariance = np.eye(6) * 500.0

        # Noise matrices
        self.Q = np.eye(6) * process_noise
        self.R = np.eye(3) * measurement_noise

        self.initialized = False

    def _build_F(self, dt: float) -> np.ndarray:
        """State transition matrix for a given dt."""
        F = np.eye(6)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt
        return F

    def _build_B(self, dt: float) -> np.ndarray:
        """Control input matrix mapping gravity acceleration to state."""
        B = np.zeros((6, 3))
        B[0, 0] = 0.5 * dt * dt
        B[1, 1] = 0.5 * dt * dt
        B[2, 2] = 0.5 * dt * dt
        B[3, 0] = dt
        B[4, 1] = dt
        B[5, 2] = dt
        return B

    def initialize_from_state(
        self,
        position: np.ndarray,
        velocity: np.ndarray,
        position_cov: float = 500.0,
        velocity_cov: float = 500.0,
    ):
        """Initialize from known position and velocity (e.g. throw announcement).

        Args:
            position: [x, y, z] mm.
            velocity: [vx, vy, vz] mm/s.
            position_cov: Initial position covariance (diagonal).
            velocity_cov: Initial velocity covariance (diagonal).
        """
        self.state[:3] = position
        self.state[3:] = velocity
        self.covariance = np.diag([
            position_cov, position_cov, position_cov,
            velocity_cov, velocity_cov, velocity_cov,
        ])
        self.initialized = True

    def initialize_from_positions(
        self,
        positions: list[np.ndarray],
        timestamps: list[float],
    ):
        """Initialize from 2+ sequential marker positions via finite difference.

        Args:
            positions: List of [x, y, z] arrays (mm), oldest first.
            timestamps: Corresponding times (seconds), oldest first.
        """
        if len(positions) < 2:
            raise ValueError("Need at least 2 positions to estimate velocity")

        # Use the most recent position
        self.state[:3] = positions[-1]

        # Finite-difference velocity from last two points, correcting for gravity
        p0, p1 = positions[-2], positions[-1]
        dt = timestamps[-1] - timestamps[-2]
        if dt <= 0:
            raise ValueError(f"Timestamps must be increasing, got dt={dt}")

        raw_vel = (p1 - p0) / dt
        # Correct z-velocity: finite-difference gives the average velocity
        # over [t0, t1], but we want vz at t1 (consistent with position=p1).
        #   raw_vz = Δz/dt = vz(t0) - 0.5*g*dt
        #   vz(t1) = vz(t0) - g*dt = raw_vz - 0.5*g*dt
        raw_vel[2] -= 0.5 * GRAVITY_MMPS2 * dt

        self.state[3:] = raw_vel
        self.covariance = np.diag([
            50.0, 50.0, 50.0,      # Tight position (measured)
            2000.0, 2000.0, 2000.0, # Wider velocity (estimated)
        ])
        self.initialized = True

    def predict(self, dt: Optional[float] = None):
        """Propagate state forward one step under gravity.

        Args:
            dt: Time step override. If None, uses self.dt.
        """
        if not self.initialized:
            return

        dt = dt if dt is not None else self.dt

        F = self._build_F(dt)
        B = self._build_B(dt)
        u = np.array([0.0, 0.0, -GRAVITY_MMPS2])

        self.state = F @ self.state + B @ u
        self.covariance = F @ self.covariance @ F.T + self.Q

    def update(self, measurement: np.ndarray):
        """Incorporate a position measurement [x, y, z].

        Args:
            measurement: Observed position [x, y, z] in mm.
        """
        if not self.initialized:
            return

        z = measurement  # 3-vector
        y = z - self.H @ self.state  # Innovation
        S = self.H @ self.covariance @ self.H.T + self.R  # Innovation covariance
        K = self.covariance @ self.H.T @ np.linalg.inv(S)  # Kalman gain

        self.state = self.state + K @ y
        I_KH = np.eye(6) - K @ self.H
        self.covariance = I_KH @ self.covariance

    @property
    def position(self) -> np.ndarray:
        """Current filtered position [x, y, z] mm."""
        return self.state[:3].copy()

    @property
    def velocity(self) -> np.ndarray:
        """Current filtered velocity [vx, vy, vz] mm/s."""
        return self.state[3:].copy()

    def predict_landing(self, landing_z: float) -> Optional[Tuple[np.ndarray, np.ndarray, float]]:
        """Predict when and where the ball will cross a horizontal plane.

        Delegates to ballistics.predict_landing_state() using current
        filtered state.

        Returns:
            (landing_pos, landing_vel, time_to_land) or None.
        """
        if not self.initialized:
            return None
        return predict_landing_state(self.position, self.velocity, landing_z)

    def reset(self):
        """Reset to uninitialized state."""
        self.state = np.zeros(6)
        self.covariance = np.eye(6) * 500.0
        self.initialized = False
