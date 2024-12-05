# """ Note that base units for this node are in millimeters and seconds. """

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Header
# from geometry_msgs.msg import Point
# from jugglebot_interfaces.msg import MocapDataMulti, MocapDataSingle
# import numpy as np
# from sklearn.linear_model import LinearRegression
# from typing import List, Tuple
# from .kalman_filter import KalmanFilter
# import uuid


# class Track:
#     def __init__(self, initial_measurement, initial_time, node):
#         self.kf = KalmanFilter(
#             process_noise=node.process_noise,
#             measurement_noise=node.measurement_noise,
#             logger=node.get_logger()
#         )
#         self.measurement_buffer = [(initial_measurement, initial_time)]
#         self.buffer_duration = node.buffer_duration
#         self.last_update_time = initial_time
#         self.initialized = False
#         self.ball_in_flight = False
#         self.previous_time = None  # To calculate dt


# class BallPredictionNode(Node):
#     """
#     ROS2 Node for tracking markers using a Kalman Filter, classifying whether the motion is projectile motion, and
#     predicting landing positions.

#     Note: units are millimeters and seconds.
#     """

#     def __init__(self):
#         super().__init__('ball_prediction_node')

#         # Parameters
#         self.declare_parameter('process_noise', 1000000.0)
#         self.declare_parameter('measurement_noise', 3.0)
#         self.declare_parameter('residual_threshold', 10.0)
#         self.declare_parameter('ground_z', 0.0)  # mm
#         self.declare_parameter('landing_threshold', 20.0)  # mm above ground
#         self.declare_parameter('is_projectile_residual_threshold', 10.0)  # mm
#         self.declare_parameter('is_projectile_a_threshold', 400.0)  # mm/s^2
#         self.declare_parameter('min_z_variance', 5.0)  # mm

#         # Initialize parameters
#         self.process_noise = self.get_parameter('process_noise').get_parameter_value().double_value
#         self.measurement_noise = self.get_parameter('measurement_noise').get_parameter_value().double_value
#         self.residual_threshold = self.get_parameter('residual_threshold').get_parameter_value().double_value
#         self.ground_z = self.get_parameter('ground_z').get_parameter_value().double_value
#         self.landing_threshold = self.get_parameter('landing_threshold').get_parameter_value().double_value
#         self.is_projectile_residual_threshold = self.get_parameter('is_projectile_residual_threshold').get_parameter_value().double_value
#         self.is_projectile_a_threshold = self.get_parameter('is_projectile_a_threshold').get_parameter_value().double_value
#         self.min_z_variance = self.get_parameter('min_z_variance').get_parameter_value().double_value

#         # Subscriber and Publisher
#         self.subscription = self.create_subscription(
#             MocapDataMulti,
#             '/mocap_data',
#             self.mocap_data_callback,
#             10
#         )
#         self.landing_pub = self.create_publisher(
#             Point,
#             '/predicted_landings',
#             10
#         )

#         # Kalman filter for the ball
#         self.kf = KalmanFilter(
#             process_noise=self.process_noise,
#             measurement_noise=self.measurement_noise,
#             logger=self.get_logger()
#         )

#         # Flags and state
#         self.ball_in_flight = False

#         # Measurement buffer for initial velocity estimation
#         self.measurement_buffer: List[Tuple[np.ndarray, float]] = []
#         self.buffer_duration = 0.1  # seconds. This is used to inform whether the ball is in flight or not
#         self.previous_time = None  # To calculate dt

#         # Data structures to allow for multiple tracked objects
#         self.tracks = {} # Key: track_id, Value: track information
#         self.max_track_age = 0.5  # Seconds before deleting a track
#         self.track_id_counter = 0  # Counter for assigning unique track IDs

#         # Parameters for track association
#         self.association_distance_threshold = 100.0  # mm

#         self.get_logger().info("BallPredictionNode has been initialized.")

#     def mocap_data_callback(self, msg: MocapDataMulti):
#         """
#         Callback function for incoming mocap data.

#         Args:
#             msg (MocapDataMulti): The incoming motion capture data containing multiple markers.
#         """
#         # Check if there are any markers
#         if not msg.unlabelled_markers:
#             return

#         # Get current time in seconds
#         current_time = self.get_clock().now().nanoseconds * 1e-9

#         # Convert markers to measurements
#         measurements = []
#         for marker in msg.unlabelled_markers:
#             position = marker.position
#             measurement = np.array([[position.x], [position.y], [position.z]])
#             measurements.append(measurement)

#         # Update existing tracks with new measurements
#         unmatched_measurements = measurements.copy()
#         for track_id, track in list(self.tracks.items()):
#             # Predict the current position
#             if track.initialized:
#                 track.kf.predict(dt=current_time - track.previous_time)
#                 track.previous_time = current_time
#                 predicted_position = track.kf.state[:3].ravel()
#             else:
#                 # If not initialized, use the last measurement
#                 predicted_position = track.measurement_buffer[-1][0].ravel()

#             # Associate measurement to track
#             closest_measurement, distance = self.find_closest_measurement(predicted_position, unmatched_measurements)
#             if distance < self.association_distance_threshold:
#                 # Measurement matched to this track                
#                 for i, measurement in enumerate(unmatched_measurements):
#                     if np.array_equal(measurement, closest_measurement):
#                         del unmatched_measurements[i]
#                         break  # Exit the loop after removing the item
                    
#                 track.measurement_buffer.append((closest_measurement, current_time))
#                 track.last_update_time = current_time

#                 # Remove old measurements
#                 while track.measurement_buffer and (current_time - track.measurement_buffer[0][1]) > track.buffer_duration * 1.2:
#                     track.measurement_buffer.pop(0)

#                 if not track.initialized:
#                     # Check if we can initialize the Kalman Filter
#                     if (track.measurement_buffer[-1][1] - track.measurement_buffer[0][1]) >= self.buffer_duration:
#                         if self.is_projectile_motion(track):
#                             self.initialize_kalman_filter(track)
#                             track.ball_in_flight = True
#                             track.previous_time = current_time
#                         else:
#                             self.get_logger().info("Motion does not follow projectile motion.", throttle_duration_sec=1.0)
#                 else:
#                     # Update the Kalman Filter
#                     try:
#                         track.kf.update(closest_measurement)
#                         # Check if the ball has landed or deviated from projectile motion
#                         self.handle_track_state(track, closest_measurement, current_time)
#                     except Exception as e:
#                         self.get_logger().error(f"Error updating Kalman filter for track {track_id}: {e}")
#             else:
#                 # If track hasn't been updated for a while, delete it
#                 if (current_time - track.last_update_time) > self.max_track_age:
#                     self.get_logger().info(f"Deleting track {track_id} due to inactivity.")
#                     del self.tracks[track_id]

#         # Create new tracks for unmatched measurements
#         for measurement in unmatched_measurements:
#             self.create_new_track(measurement, current_time)

#     def find_closest_measurement(self, predicted_position, measurements):
#         """
#         Finds the closest measurement to the predicted position.

#         Args:
#             predicted_position (np.ndarray): The predicted position (x, y, z).
#             measurements (List[np.ndarray]): List of measurements.

#         Returns:
#             Tuple[np.ndarray, float]: Closest measurement and the distance to it.
#         """
#         min_distance = float('inf')
#         closest_measurement = None
#         for measurement in measurements:
#             distance = np.linalg.norm(predicted_position - measurement.ravel())
#             if distance < min_distance:
#                 min_distance = distance
#                 closest_measurement = measurement
#         return closest_measurement, min_distance

#     def create_new_track(self, measurement, current_time):
#         """
#         Creates a new track for an unmatched measurement.

#         Args:
#             measurement (np.ndarray): The measurement to start the new track.
#             current_time (float): The current timestamp.
#         """
#         track_id = str(uuid.uuid4())
#         new_track = Track(measurement, current_time, self)
#         self.tracks[track_id] = new_track
#         self.get_logger().info(f"Created new track {track_id}")

#     def handle_track_state(self, track, measurement, current_time):
#         """
#         Handles the state of the track after updating the Kalman Filter.

#         Args:
#             track (Track): The track being updated.
#             measurement (np.ndarray): The latest measurement.
#             current_time (float): The current timestamp.
#         """
#         z_position = measurement[2][0]

#         # Check if the ball has landed
#         if z_position <= self.ground_z + self.landing_threshold:
#             if track.ball_in_flight:
#                 self.get_logger().info("Ball has landed.")
#                 track.ball_in_flight = False
#                 # Reset the Kalman filter
#                 track.kf.reset()
#                 # Clear the measurement buffer
#                 track.measurement_buffer.clear()
#                 track.initialized = False
#         elif not self.is_projectile_motion(track):
#             self.get_logger().info("Motion deviated from projectile motion. Resetting Kalman filter.")
#             track.kf.reset()
#             track.measurement_buffer.clear()
#             track.initialized = False
#             track.ball_in_flight = False
#         else:
#             if not track.ball_in_flight:
#                 self.get_logger().info("Ball is in flight.")
#                 track.ball_in_flight = True

#             # Predict landing position
#             prediction = track.kf.predict_landing(self.ground_z)
#             if prediction is not None:
#                 (landing_x, landing_y), landing_time = prediction
#                 landing_point = Point()
#                 landing_point.x = landing_x
#                 landing_point.y = landing_y
#                 landing_point.z = self.ground_z
#                 self.landing_pub.publish(landing_point)


#     def is_projectile_motion(self, track) -> bool:
#         """
#         Check if the motion of the ball follows projectile motion.

#         Args:
#             track (Track): The track object containing the measurements.

#         Returns:
#             bool: True if the motion follows projectile motion, False otherwise.
#         """

#         self.get_logger().info(f"Number of datapoints: {len(track.measurement_buffer)}")

#         # Extract positions and times
#         positions = np.array([p[0].ravel() for p in track.measurement_buffer])
#         times = np.array([t[1] for t in track.measurement_buffer]) 
#         times = times - times[0]  # Normalize start time at 0

#         # Check variance in the z position (too little variance indicates the ball is very likely not in projectile motion)
#         z_variance = np.var(positions[:, 2])

#         # if z_variance < self.min_z_variance:
#         #     self.get_logger().info(f"Variance in z position is too low: {z_variance:.2f} mm. Not projectile motion.")
#         #     return False

#         # Fit a quadratic model to z(t)
#         z_positions = positions[:, 2] # z-axis positions
#         t = times.reshape(-1, 1)
#         X = np.hstack((t ** 2, t, np.ones_like(t)))

#         # Perform a linear regression to fit z = a*t^2 + b*t + c
#         model = LinearRegression()
#         model.fit(X, z_positions)
#         z_pred = model.predict(X)

#         # Calculate residuals
#         residuals = z_positions - z_pred
#         residuals_std = np.std(residuals)

#         # Get the coeffiecients of the model
#         a = model.coef_[0] # Quadratic coefficient

#         # Check if 'a' is close to the expected value
#         expected_a = -0.5 * 9810.0  # 0.5 * gravity in mm/s^2
#         a_diff = np.abs(a - expected_a)

#         # Get the thresholds for acceptable residuals and 'a' difference
#         res_thresh = self.get_parameter('is_projectile_residual_threshold').get_parameter_value().double_value
#         a_thresh = self.get_parameter('is_projectile_a_threshold').get_parameter_value().double_value

#         # Check if the residuals are within the threshold
#         if residuals_std < res_thresh*1000.0 and a_diff < a_thresh:
#             self.get_logger().info(f"Projectile motion detected with residual std: {residuals_std:.2f} mm.")
#             return True
#         else:
#             self.get_logger().info(f"Residual std: {residuals_std:.2f} mm, a_coefficient: {a} mm/s^2. Not projectile motion.")
#             return False

#     def initialize_kalman_filter(self, track):
#         """
#         Initializes the Kalman Filter using the collected measurements in the buffer.

#         Args:
#             track (Track): The track object containing the measurements.
#         """
#         # Use the last position as the initial position
#         initial_position = track.measurement_buffer[-1][0]

#         # Estimate initial velocity using linear regression on positions
#         times = np.array([t[1] for t in track.measurement_buffer])
#         times = times - times[0]  # Normalize time
#         positions = np.array([p[0].ravel() for p in track.measurement_buffer])

#         # Fit linear models to x(t) and y(t)
#         X = times.reshape(-1, 1)
#         vx_model = LinearRegression()
#         vy_model = LinearRegression()
#         vx_model.fit(X, positions[:, 0])
#         vy_model.fit(X, positions[:, 1])
#         initial_vx = vx_model.coef_[0]
#         initial_vy = vy_model.coef_[0]

#         # For z(t), the velocity is the derivative at the last point
#         t = times
#         z_positions = positions[:, 2]
#         t_squared = (t ** 2)
#         X_z = np.vstack((t_squared, t, np.ones_like(t))).T
#         z_model = LinearRegression()
#         z_model.fit(X_z, z_positions)
#         a = z_model.coef_[0]
#         b = z_model.coef_[1]
#         # Velocity at last time
#         last_time = t[-1]
#         initial_vz = 2 * a * last_time + b

#         initial_velocity = np.array([[initial_vx], [initial_vy], [initial_vz]])
#         initial_state = np.vstack((initial_position, initial_velocity))
#         self.kf.initialize_with_state(initial_state)
#         # self.get_logger().info(f"Kalman Filter initialized with position {initial_position.ravel()} and velocity {initial_velocity.ravel()}")

#         # Initialize the track
#         track.initialized = True

#     def destroy_node(self):
#         """
#         Clean up before shutting down the node.
#         """
#         self.get_logger().info("Shutting down BallPredictionNode.")
#         super().destroy_node()


# def main(args=None):
#     rclpy.init(args=args)
#     node = BallPredictionNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info("Keyboard interrupt received. Shutting down.")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()


""" Note that base units for this node are in millimeters and seconds. """

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from jugglebot_interfaces.msg import MocapDataMulti, MocapDataSingle
import numpy as np
from sklearn.linear_model import LinearRegression
from typing import List, Tuple
from .kalman_filter import KalmanFilter


class BallPredictionNode(Node):
    """
    ROS2 Node for tracking a single marker using a Kalman Filter, predicting landing positions.
    Note units are millimeters and seconds.
    """

    def __init__(self):
        super().__init__('ball_prediction_node')

        # Parameters
        self.declare_parameter('process_noise', 1000000.0)
        self.declare_parameter('measurement_noise', 3.0)
        self.declare_parameter('residual_threshold', 10.0)
        self.declare_parameter('ground_z', 0.0)  # mm
        self.declare_parameter('landing_threshold', 20.0)  # mm above ground
        self.declare_parameter('is_projectile_residual_threshold', 10.0)  # mm
        self.declare_parameter('is_projectile_a_threshold', 400.0)  # mm/s^2
        self.declare_parameter('min_z_variance', 5.0)  # mm

        # Initialize parameters
        self.process_noise = self.get_parameter('process_noise').get_parameter_value().double_value
        self.measurement_noise = self.get_parameter('measurement_noise').get_parameter_value().double_value
        self.residual_threshold = self.get_parameter('residual_threshold').get_parameter_value().double_value
        self.ground_z = self.get_parameter('ground_z').get_parameter_value().double_value
        self.landing_threshold = self.get_parameter('landing_threshold').get_parameter_value().double_value
        self.is_projectile_residual_threshold = self.get_parameter('is_projectile_residual_threshold').get_parameter_value().double_value
        self.is_projectile_a_threshold = self.get_parameter('is_projectile_a_threshold').get_parameter_value().double_value
        self.min_z_variance = self.get_parameter('min_z_variance').get_parameter_value().double_value

        # Subscriber and Publisher
        self.subscription = self.create_subscription(
            MocapDataMulti,
            '/mocap_data',
            self.mocap_data_callback,
            10
        )
        self.landing_pub = self.create_publisher(
            Point,
            '/predicted_landings',
            10
        )

        # Kalman filter for the ball
        self.kf = KalmanFilter(
            process_noise=self.process_noise,
            measurement_noise=self.measurement_noise,
            logger=self.get_logger()
        )

        # Flags and state
        self.ball_in_flight = False

        # Measurement buffer for initial velocity estimation
        self.measurement_buffer: List[Tuple[np.ndarray, float]] = []
        self.buffer_duration = 0.1  # seconds. This is used to inform whether the ball is in flight or not
        self.previous_time = None  # To calculate dt

        self.get_logger().info("BallPredictionNode has been initialized.")

    def mocap_data_callback(self, msg: MocapDataMulti):
        """
        Callback function for incoming mocap data.

        Args:
            msg (MocapDataMulti): The incoming motion capture data containing multiple markers.
        """
        # Check if there are any markers
        if not msg.unlabelled_markers:
            return

        # Assume only one marker (the ball)
        marker = msg.unlabelled_markers[0]
        position = marker.position

        # Convert to numpy array
        measurement = np.array([[position.x], [position.y], [position.z]])

        # Get current time in seconds
        current_time = self.get_clock().now().nanoseconds * 1e-9

        # Update the measurement buffer
        self.measurement_buffer.append((measurement, current_time))

        # Remove old measurements from the buffer
        while self.measurement_buffer and current_time - self.measurement_buffer[0][1] > self.buffer_duration*1.2:
            self.measurement_buffer.pop(0)

        if not self.kf.initialized:
            # Check whether we have enough data to classify the ball as in flight
            if (self.measurement_buffer[-1][1] - self.measurement_buffer[0][1]) >= self.buffer_duration:
                # Check if the motion follows projectile motion
                if self.is_projectile_motion():
                    # Initialize the Kalman filter with the collected data
                    self.initialize_kalman_filter()
                    self.ball_in_flight = True
                    self.previous_time = current_time
                else:
                    self.get_logger().info("Motion does not follow projectile motion.", throttle_duration_sec=1.0)

            else:
                self.get_logger().info("Not enough data to classify the ball as in flight.", throttle_duration_sec=1.0)

        else:
            # Calculate dt
            dt = current_time - self.previous_time
            # self.get_logger().info(f"dt: {dt:.3f} s")
            self.previous_time = current_time

            # Update the Kalman filter
            try:
                self.kf.predict(dt=dt)
                self.kf.update(measurement)

                # Get current estimated position
                estimated_state = self.kf.state

                # self.get_logger().info(f"Measurement: {measurement.ravel()} mm")
                # self.get_logger().info(f"Estimated state: {estimated_state.ravel()} mm")
                # self.get_logger().info(f"dt: {dt:.3f} s")

                # Check if the ball has landed
                if measurement[2] <= self.ground_z + self.landing_threshold:
                    if self.ball_in_flight:
                        self.get_logger().info("Ball has landed.")
                        self.ball_in_flight = False
                        # Reset the Kalman filter
                        self.kf.reset()
                        # Clear the measurement buffer
                        self.measurement_buffer.clear()
                elif not self.is_projectile_motion():
                    self.get_logger().info("Motion deviated from projectile motion. Resetting Kalman filter.")
                    self.kf.reset()
                    self.measurement_buffer.clear()
                    self.ball_in_flight = False
                else:
                    if not self.ball_in_flight:
                        self.get_logger().info("Ball is in flight.")
                        self.ball_in_flight = True

                    # Predict landing position
                    prediction = self.kf.predict_landing(self.ground_z)
                    if prediction is not None:
                        (landing_x, landing_y), landing_time = prediction
                        landing_point = Point()
                        landing_point.x = landing_x
                        landing_point.y = landing_y
                        landing_point.z = self.ground_z
                        self.landing_pub.publish(landing_point)
                        # self.get_logger().debug(f"Published landing position at ({landing_x}, {landing_y}, {self.ground_z}) mm.")
            except Exception as e:
                self.get_logger().error(f"Error updating Kalman filter: {e}")

    def is_projectile_motion(self) -> bool:
        """
        Check if the motion of the ball follows projectile motion.

        Returns:
            bool: True if the motion follows projectile motion, False otherwise.
        """

        self.get_logger().info(f"Number of datapoints: {len(self.measurement_buffer)}")

        # Extract positions and times
        positions = np.array([p[0].ravel() for p in self.measurement_buffer])
        times = np.array([t[1] for t in self.measurement_buffer]) 
        times = times - times[0]  # Normalize start time at 0

        # Check variance in the z position (too little variance indicates the ball is very likely not in projectile motion)
        z_variance = np.var(positions[:, 2])

        # if z_variance < self.min_z_variance:
        #     self.get_logger().info(f"Variance in z position is too low: {z_variance:.2f} mm. Not projectile motion.")
        #     return False

        # Fit a quadratic model to z(t)
        z_positions = positions[:, 2] # z-axis positions
        t = times.reshape(-1, 1)
        X = np.hstack((t ** 2, t, np.ones_like(t)))

        # Perform a linear regression to fit z = a*t^2 + b*t + c
        model = LinearRegression()
        model.fit(X, z_positions)
        z_pred = model.predict(X)

        # Calculate residuals
        residuals = z_positions - z_pred
        residuals_std = np.std(residuals)

        # Get the coeffiecients of the model
        a = model.coef_[0] # Quadratic coefficient

        # Check if 'a' is close to the expected value
        expected_a = -0.5 * 9810.0  # 0.5 * gravity in mm/s^2
        a_diff = np.abs(a - expected_a)

        # Get the thresholds for acceptable residuals and 'a' difference
        res_thresh = self.get_parameter('is_projectile_residual_threshold').get_parameter_value().double_value
        a_thresh = self.get_parameter('is_projectile_a_threshold').get_parameter_value().double_value

        # Check if the residuals are within the threshold
        if residuals_std < res_thresh*1000.0 and a_diff < a_thresh:
            self.get_logger().info(f"Projectile motion detected with residual std: {residuals_std:.2f} mm.")
            return True
        else:
            self.get_logger().info(f"Residual std: {residuals_std:.2f} mm, a_coefficient: {a} mm/s^2. Not projectile motion.")
            return False

    def initialize_kalman_filter(self):
        """
        Initializes the Kalman Filter using the collected measurements in the buffer.
        """
        # Use the last position as the initial position
        initial_position = self.measurement_buffer[-1][0]

        # Estimate initial velocity using linear regression on positions
        times = np.array([t[1] for t in self.measurement_buffer])
        times = times - times[0]  # Normalize time
        positions = np.array([p[0].ravel() for p in self.measurement_buffer])

        # Fit linear models to x(t) and y(t)
        X = times.reshape(-1, 1)
        vx_model = LinearRegression()
        vy_model = LinearRegression()
        vx_model.fit(X, positions[:, 0])
        vy_model.fit(X, positions[:, 1])
        initial_vx = vx_model.coef_[0]
        initial_vy = vy_model.coef_[0]

        # For z(t), the velocity is the derivative at the last point
        t = times
        z_positions = positions[:, 2]
        t_squared = (t ** 2)
        X_z = np.vstack((t_squared, t, np.ones_like(t))).T
        z_model = LinearRegression()
        z_model.fit(X_z, z_positions)
        a = z_model.coef_[0]
        b = z_model.coef_[1]
        # Velocity at last time
        last_time = t[-1]
        initial_vz = 2 * a * last_time + b

        initial_velocity = np.array([[initial_vx], [initial_vy], [initial_vz]])
        initial_state = np.vstack((initial_position, initial_velocity))
        self.kf.initialize_with_state(initial_state)
        self.get_logger().info(f"Kalman Filter initialized with position {initial_position.ravel()} and velocity {initial_velocity.ravel()}")


    def destroy_node(self):
        """
        Clean up before shutting down the node.
        """
        self.get_logger().info("Shutting down BallPredictionNode.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BallPredictionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
