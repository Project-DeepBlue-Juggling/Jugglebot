# kalman_filter.py

import numpy as np
from typing import Optional, Tuple


class KalmanFilter:
    """
    A Kalman Filter for tracking a single marker in 3D space, predicting landing positions.

    Note units are mm and seconds.

    Attributes:
        state (np.ndarray): The state vector [x, y, z, vx, vy, vz].
        covariance (np.ndarray): The state covariance matrix.
        Q (np.ndarray): The process noise covariance matrix.
        R (np.ndarray): The measurement noise covariance matrix.
        initialized (bool): Flag indicating if the filter has been initialized.
    """

    def __init__(self, dt: float = 0.0033, process_noise: float = 1.0, measurement_noise: float = 3.0, logger=None):
        """
        Initializes the Kalman Filter with default parameters.

        Args:
            dt (float): Time step between measurements in seconds. Default is ~300 Hz.
            process_noise (float): Variance of the process noise.
            measurement_noise (float): Variance of the measurement noise.
            logger: Logger for debugging.
        """
        self.dt = dt  # Time step

        # State vector: [x, y, z, vx, vy, vz]
        self.state = np.zeros((6, 1))  # Initialized to zero; will be set on first measurement

        # State covariance matrix
        self.covariance = np.eye(6) * 500.0  # Large initial uncertainty

        # Initialize the state transition matrices
        self.update_state_transition_matrices()

        # Process noise covariance
        self.Q = np.eye(6) * process_noise

        # Measurement noise covariance
        self.R = np.eye(3) * measurement_noise

        # Flag to indicate if the filter has been initialized with the first measurement
        self.initialized = False

        # Logging setup
        self.logger = logger

    def update_state_transition_matrices(self):
        """
        Updates the state transition matrices F and B based on the current dt.
        """
        self.F = np.array([
            [1, 0, 0, self.dt, 0,    0],
            [0, 1, 0, 0,    self.dt, 0],
            [0, 0, 1, 0,    0,    self.dt],
            [0, 0, 0, 1,    0,    0],
            [0, 0, 0, 0,    1,    0],
            [0, 0, 0, 0,    0,    1]
        ])

    def initialize_with_state(self, initial_state: np.ndarray):
        """
        Initializes the filter with a given state vector.

        Args:
            initial_state (np.ndarray): The initial state vector [x, y, z, vx, vy, vz].
        """
        if initial_state.shape != (6, 1):
            self.logger.error("Initial state must be a 6x1 vector.")
            raise ValueError("Invalid initial state shape.")

        self.state = initial_state
        self.initialized = True
        self.logger.info(f"Filter initialized with state: {self.state.ravel()}")

    def predict(self, dt: Optional[float] = None):
        """
        Performs the prediction step of the Kalman Filter.

        Args:
            dt (float): Optional time step. If provided, updates the state transition matrices.
        """
        if not self.initialized:
            self.logger.warning("Filter not initialized. Prediction skipped.")
            return

        if dt is not None:
            self.dt = dt
            self.update_state_transition_matrices()

        # Gravity acceleration in mm/s^2
        gravity = -9810.0

        # Control input (acceleration due to gravity)
        u = np.array([[0], [0], [gravity]])

        # Control input matrix
        B = np.array([
            [0.5 * self.dt**2, 0, 0],
            [0, 0.5 * self.dt**2, 0],
            [0, 0, 0.5 * self.dt**2],
            [self.dt, 0, 0],
            [0, self.dt, 0],
            [0, 0, self.dt]
        ])

        # Predict the state
        self.state = self.F @ self.state + B @ u

        # Predict the covariance
        self.covariance = self.F @ self.covariance @ self.F.T + self.Q

        # self.logger.debug(f"Predicted state: {self.state.ravel()}")
        # self.logger.debug(f"Predicted covariance: {self.covariance}")

    def update(self, measurement: np.ndarray):
        """
        Performs the update step of the Kalman Filter with a new measurement.

        Args:
            measurement (np.ndarray): The measurement vector [x, y, z].
        """
        if measurement.shape != (3, 1):
            self.logger.error("Measurement must be a 3x1 vector.")
            raise ValueError("Invalid measurement shape.")

        if not self.initialized:
            self.logger.warning("Filter not initialized. Update skipped.")
            return

        # Compute the Kalman Gain
        S = self.H @ self.covariance @ self.H.T + self.R
        K = self.covariance @ self.H.T @ np.linalg.inv(S)

        # Update the state with the measurement
        y = measurement - (self.H @ self.state)
        self.state = self.state + K @ y

        # Update the covariance
        I = np.eye(self.F.shape[0])
        self.covariance = (I - K @ self.H) @ self.covariance

        # self.logger.debug(f"Updated state: {self.state.ravel()}")
        # self.logger.debug(f"Updated covariance: {self.covariance}")

    def predict_landing(self, ground_z: float = 0.0) -> Optional[Tuple[Tuple[float, float], float]]:
        """
        Predicts the landing position (x, y) and time when the marker will reach the specified ground z-height.

        Args:
            ground_z (float): The z-height plane representing the ground.

        Returns:
            Optional[Tuple[Tuple[float, float], float]]: The predicted landing position (x, y) and the time to land in seconds.
                                                         Returns None if prediction is not feasible.
        """
        if not self.initialized:
            self.logger.info("Filter not initialized. Cannot predict landing.")
            return None

        # Current state
        x, y, z, vx, vy, vz = self.state.ravel()

        # Solve for time when z + vz*t + 0.5*(-g)*t^2 = ground_z
        a = -0.5 * 9810.0  # 0.5 * gravity in mm/s^2
        b = vz
        c = z - ground_z

        discriminant = b**2 - 4 * a * c
        if discriminant < 0:
            self.logger.warning("No real solution for landing time.")
            return None

        sqrt_discriminant = np.sqrt(discriminant)
        t1 = (-b + sqrt_discriminant) / (2 * a)
        t2 = (-b - sqrt_discriminant) / (2 * a)

        # Choose the positive and smallest time
        landing_time = min(t for t in [t1, t2] if t > 0) if any(t > 0 for t in [t1, t2]) else None

        if landing_time is None:
            self.logger.warning("No positive landing time found.")
            return None

        # Predict landing positions
        landing_x = x + vx * landing_time
        landing_y = y + vy * landing_time

        # self.logger.info(f"Predicted landing at ({landing_x}, {landing_y}) mm in {landing_time:.2f} seconds.")

        return (landing_x, landing_y), landing_time

    def reset(self):
        """
        Resets the filter to its initial state.
        """
        self.state = np.zeros((6, 1))
        self.covariance = np.eye(6) * 500.0
        self.initialized = False
        self.previous_velocity = np.zeros((3, 1))
        self.logger.info("Kalman Filter has been reset.")

    # Initialize previous velocity
    previous_velocity = np.zeros((3, 1))

    # Measurement matrix
    H = np.array([
        [1, 0, 0, 0, 0, 0],  # x
        [0, 1, 0, 0, 0, 0],  # y
        [0, 0, 1, 0, 0, 0]   # z
    ])
