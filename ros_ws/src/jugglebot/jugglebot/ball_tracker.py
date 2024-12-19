'''
Class overview:
This class is responsible for tracking the ball in the air. It uses a kalman filter to estimate the position and velocity of the ball.
It also uses a linear regression model to classify whether the ball is in projectile motion or not.
The class has a buffer to store the last few measurements, and once it has enough data, it attempts to classify projectile motion.
If projectile motion is confirmed, the kalman filter is initialized with the initial state of the ball.
The class also has a method to predict the landing state (pos, vel, time) of the ball based on the kalman filter estimate.

One big assumption is that once the ball enters projectile motion, it won't stop until it lands.
This landing is based on the kalman filter estimate, so as soon as projectile motion is confirmed, the virtual 
ball won't stop 'existing' until it 'lands'.
'''

from collections import deque
from jugglebot_interfaces.msg import BallStateSingle
from builtin_interfaces.msg import Time
import numpy as np
from sklearn.linear_model import LinearRegression
from typing import Optional
from .kalman_filter import KalmanFilter

class BallTracker:
    def __init__(self, dt: float, ground_height: float = 0.0, logger=None, initial_data=None, node=None):
        self.logger = logger
        self.node = node

        # Parameters
        self.process_noise = 5.0
        self.measurement_noise = 1.0
        self.ground_z = ground_height
        self.landing_threshold = 20.0
        self.is_projectile_residual_threshold = 10.0
        self.is_projectile_a_threshold = 400.0
        self.buffer_duration = 0.1  # seconds to store for projectile motion classification
        self.expected_a = -0.5 * 9810.0

        self.current_position = None

        self.measurement_buffer = deque()
        self.latest_measurement: Optional[np.ndarray] = None  # Holds the latest measurement, or None if no new measurement.

        self.projectile_motion_confirmed = False
        self.previous_time = self.node.get_clock().now().nanoseconds * 1e-9

        self.kf = KalmanFilter(
            dt=dt,
            process_noise=self.process_noise,
            measurement_noise=self.measurement_noise,
            logger=self.logger
        )

        self.update_with_new_data(initial_data)

        # self.logger.info(f"BallTracker initialized.")

    def update_with_new_data(self, data):
        """
        Update the tracker with new data. This method should be called whenever new data is available.

        Args:
            data: The new data to update the tracker with. (np.ndarray)
        """
        # Only store the latest measurement
        if isinstance(data, np.ndarray):
            self.latest_measurement = np.array([[data[0]],
                                                [data[1]],
                                                [data[2]]])
            
            if self.current_position is None:
                self.current_position = self.latest_measurement
        else:
            self.latest_measurement = None

    def advance_tracker(self) -> Optional[BallStateSingle]:
        """
        Update the tracker, advancing it to the current timestep. 
        If there's a new measurement, add it to the buffer and check for projectile motion.
        If projectile motion is confirmed, predict the landing state and return it.

        If there is no new measurement and we were previously classed as being in projectile motion,
        just predict the landing state with the Kalman Filter.

        Returns:
            Optional[BallStateSingle]: The predicted landing state (pos, vel, time_to_land) of the ball.
        """
        current_time = self.node.get_clock().now().nanoseconds * 1e-9
        self.previous_time = current_time

        # Initialize landing state
        landing_state = None

        new_measurement = self.latest_measurement
        # Clear latest_measurement since we're consuming it now
        self.latest_measurement = None

        if not self.projectile_motion_confirmed:
            # Not in projectile motion yet
            if new_measurement is not None:
                # We got a new measurement: add it to the buffer
                self.measurement_buffer.append((new_measurement, current_time))

                # Also update the current state of this tracked object
                self.current_position = new_measurement.ravel()

                # Check if we have enough data to classify projectile motion
                if len(self.measurement_buffer) >= 3:
                    time_span = self.measurement_buffer[-1][1] - self.measurement_buffer[0][1]
                    if time_span >= self.buffer_duration:
                        # Attempt classification
                        if self.is_projectile_motion():
                            self.projectile_motion_confirmed = True
                            self.initialize_kalman_filter()
                            # self.logger.info("Projectile motion confirmed. KF initialized.")
                            # Do not trim since we now want full path
                        else:
                            # Not projectile yet, trim buffer to maintain buffer_duration window
                            self.trim_buffer_to_duration()
                    else:
                        # Not enough time span yet, just wait
                        self.trim_buffer_to_duration()
                else:
                    # Not enough points yet
                    self.trim_buffer_to_duration()
            else:
                # No new measurement and not projectile (yet). Just wait.
                self.trim_buffer_to_duration()
                pass
        else:
            # Projectile motion confirmed
            if new_measurement is not None:
                # Add measurement to buffer
                self.measurement_buffer.append((new_measurement, current_time))
                # Update KF with measurement
                self.kf.predict()
                self.kf.update(new_measurement)
            else:
                # No new measurement, just predict forward
                self.kf.predict()

            # Get the current state
            self.current_position = self.kf.get_current_position().ravel()

            # Check landing from KF state
            estimated_state = self.kf.state.ravel()
            est_z = estimated_state[2]
            if est_z <= self.ground_z + self.landing_threshold and self.projectile_motion_confirmed:
                # self.logger.info("Ball has landed (KF estimate). Resetting.")
                self.kf.reset()
                self.measurement_buffer.clear()
                self.projectile_motion_confirmed = False
                return None

            # Predict landing
            prediction = self.kf.predict_landing_state(self.ground_z)
            if prediction is not None:
                # If we have a prediction, publish it
                (landing_x, landing_y), (landing_velx, landing_vely, landing_velz), time_to_land = prediction
                landing_state = BallStateSingle()
                
                # Header
                landing_state.header.stamp = self.node.get_clock().now().to_msg()
                landing_state.header.frame_id = "base"
                # Position
                landing_state.landing_position.x = landing_x
                landing_state.landing_position.y = landing_y
                landing_state.landing_position.z = self.ground_z
                # Velocity
                landing_state.landing_velocity.x = landing_velx
                landing_state.landing_velocity.y = landing_vely
                landing_state.landing_velocity.z = landing_velz
                # Time at land - when the ball will cross the catch plane - as a ROS2 Time object
                # This is the current time + the time to land
                landing_state.time_at_land = Time()
                landing_state.time_at_land.sec = int(current_time + time_to_land)
                landing_state.time_at_land.nanosec = int((current_time + time_to_land) % 1 * 1e9)
                
            # Since projectile motion confirmed, we do not trim the buffer.

        return landing_state

    def is_projectile_motion(self) -> bool:
        if len(self.measurement_buffer) < 3:
            return False
        positions = np.array([p[0].ravel() for p in self.measurement_buffer])
        times = np.array([t[1] for t in self.measurement_buffer])
        times = times - times[0]

        z_positions = positions[:, 2]
        t = times.reshape(-1, 1)
        X = np.hstack((t**2, t, np.ones_like(t)))

        model = LinearRegression().fit(X, z_positions)
        z_pred = model.predict(X)

        residuals = z_positions - z_pred
        residuals_std = np.std(residuals)
        a = model.coef_[0]
        a_diff = np.abs(a - self.expected_a)

        return (residuals_std < self.is_projectile_residual_threshold) and (a_diff < self.is_projectile_a_threshold)

    def initialize_kalman_filter(self):
        if len(self.measurement_buffer) < 3:
            self.logger.warning("Not enough data to initialize KF.")
            return

        initial_position = self.measurement_buffer[-1][0]
        times = np.array([m[1] for m in self.measurement_buffer])
        times -= times[0]
        positions = np.array([m[0].ravel() for m in self.measurement_buffer])

        X_t = times.reshape(-1, 1)
        vx_model = LinearRegression().fit(X_t, positions[:, 0])
        vy_model = LinearRegression().fit(X_t, positions[:, 1])
        initial_vx = vx_model.coef_[0]
        initial_vy = vy_model.coef_[0]

        z_positions = positions[:, 2]
        X_z = np.vstack((times**2, times, np.ones_like(times))).T
        z_model = LinearRegression().fit(X_z, z_positions)
        a = z_model.coef_[0]
        b = z_model.coef_[1]
        last_time = times[-1]
        initial_vz = 2 * a * last_time + b

        initial_velocity = np.array([[initial_vx], [initial_vy], [initial_vz]])
        initial_state = np.vstack((initial_position, initial_velocity))
        self.kf.initialize_with_state(initial_state)
        # self.logger.info(f"KF initialized: position={initial_position.ravel()}, velocity={initial_velocity.ravel()}")

    def trim_buffer_to_duration(self):
        current_time = self.node.get_clock().now().nanoseconds * 1e-9
        while len(self.measurement_buffer) > 0 and \
              (current_time - self.measurement_buffer[0][1]) > self.buffer_duration:
            self.measurement_buffer.popleft()