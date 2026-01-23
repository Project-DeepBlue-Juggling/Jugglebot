# landing_analysis_node.py

import rclpy
from rclpy.node import Node
from jugglebot_interfaces.msg import MocapDataMulti, BallStateArray
import numpy as np
import matplotlib.pyplot as plt
from threading import Lock
from typing import List, Tuple


class LandingAnalysisNode(Node):
    def __init__(self):
        super().__init__('landing_analysis_node')

        # Declare and get ROS2 parameters with default values
        self.declare_parameter('ground_z', 1200.0)  # Ground plane in mm
        self.ground_z = self.get_parameter('ground_z').get_parameter_value().double_value

        self.declare_parameter('analysis_duration', 10.0)  # Duration in seconds for each analysis window
        self.analysis_duration = self.get_parameter('analysis_duration').get_parameter_value().double_value

        self.declare_parameter('z_velocity_threshold', -50.0)  # Minimum z-velocity in mm/s to consider landing
        self.z_velocity_threshold = self.get_parameter('z_velocity_threshold').get_parameter_value().double_value

        self.declare_parameter('min_predictions', 10)  # Minimum number of predictions required to plot a landing
        self.min_predictions = self.get_parameter('min_predictions').get_parameter_value().integer_value

        # State variables
        self.landed_instances: List[Tuple[np.ndarray, float, List[Tuple[float, float, float]]]] = []
        # Each instance: (final_landing_pos, landing_time, list_of_predictions)
        self.current_predictions: List[Tuple[float, float, float]] = []  # List of (prediction_time, x, y) tuples
        self.start_time = self.get_clock().now().nanoseconds * 1e-9  # Initial timestamp

        # Previous z-position and time for velocity calculation
        self.previous_z = None
        self.previous_time_z = None

        # Lock for thread-safe operations
        self.lock = Lock()

        # Subscribers
        self.mocap_sub = self.create_subscription(
            MocapDataMulti,
            '/mocap_data',
            self.mocap_callback,
            10
        )

        self.balls_sub = self.create_subscription(
            BallStateArray,
            '/balls',
            self.balls_callback,
            10
        )

        # Timer to define analysis duration (one-shot timer)
        self.analysis_timer = self.create_timer(self.analysis_duration, self.analysis_timer_callback)
        self.analysis_timer_active = True

        self.get_logger().info("LandingAnalysisNode initialized.")
        self.get_logger().info(f"Parameters - ground_z: {self.ground_z} mm, "
                               f"analysis_duration: {self.analysis_duration} s, "
                               f"z_velocity_threshold: {self.z_velocity_threshold} mm/s, "
                               f"min_predictions: {self.min_predictions}")

    def mocap_callback(self, msg: MocapDataMulti):
        """
        Callback to handle incoming mocap data.
        Detects when a ball crosses below the ground plane and is moving downwards.
        """
        with self.lock:
            if len(msg.unlabelled_markers) == 0:
                return  # No markers to process

            # Iterate through each marker (assuming multiple markers could be tracked)
            for marker in msg.unlabelled_markers:
                x, y, z = marker.position.x, marker.position.y, marker.position.z

                current_time = self.get_clock().now().nanoseconds * 1e-9

                # Compute z-velocity
                if self.previous_z is not None and self.previous_time_z is not None:
                    delta_time = current_time - self.previous_time_z
                    if delta_time > 0:
                        z_velocity = (z - self.previous_z) / delta_time
                    else:
                        z_velocity = 0.0
                else:
                    z_velocity = 0.0  # Cannot compute velocity yet

                # Update previous z and time
                self.previous_z = z
                self.previous_time_z = current_time

                # Check if the ball has landed
                if z <= self.ground_z and z_velocity <= self.z_velocity_threshold:
                    # Considered a valid landing
                    # self.get_logger().info(f"Ball landed at ({x:.2f}, {y:.2f}) mm at time {current_time - self.start_time:.2f} s with z-velocity {z_velocity:.2f} mm/s.")

                    # Collect predictions made before landing time
                    relevant_predictions = [pred for pred in self.current_predictions if pred[0] <= current_time]

                    # Check if the landing instance has enough predictions
                    if len(relevant_predictions) >= self.min_predictions:
                        # Store the landed instance
                        self.landed_instances.append((np.array([x, y]), current_time, relevant_predictions))
                        # self.get_logger().info(f"Stored landing instance {len(self.landed_instances)} with {len(relevant_predictions)} predictions.")
                    else:
                        # Discard the landing instance due to insufficient predictions
                        # self.get_logger().warn(f"Discarded landing instance at time {current_time - self.start_time:.2f} s due to insufficient predictions ({len(relevant_predictions)} < {self.min_predictions}).")
                        pass

                    # Remove these predictions from current_predictions to prevent reuse
                    self.current_predictions = [pred for pred in self.current_predictions if pred[0] > current_time]

    def balls_callback(self, msg: BallStateArray):
        """
        Callback to handle incoming ball state data.
        Stores all predictions with their reception time.
        """
        with self.lock:
            current_time = self.get_clock().now().nanoseconds * 1e-9
            for ball in msg.balls:
                # Extract predicted landing position
                px = ball.landing_position.x
                py = ball.landing_position.y
                # Store (prediction_time, x, y)
                self.current_predictions.append((current_time, px, py))
                # Note: Velocities and time_to_land can be stored if needed

    def analysis_timer_callback(self):
        """
        Callback triggered after the analysis duration.
        Processes all landed instances within the duration, plots errors, and resets storage.
        Blocks until the plot window is closed before starting the next analysis window.
        """
        # Stop the timer to prevent overlapping callbacks
        self.analysis_timer.cancel()
        self.analysis_timer_active = False

        with self.lock:
            if not self.landed_instances:
                self.get_logger().info("No valid landings detected in this analysis window.")
            else:
                self.get_logger().info(f"Analyzing {len(self.landed_instances)} landed instance(s).")

                # Prepare data for plotting
                plot_data = []
                for idx, (final_pos, landing_time, predictions) in enumerate(self.landed_instances, start=1):
                    pred_times = np.array([p[0] for p in predictions])
                    pred_xs = np.array([p[1] for p in predictions])
                    pred_ys = np.array([p[2] for p in predictions])

                    # Compute Euclidean errors between predictions and actual landing
                    final_x, final_y = final_pos
                    errors = np.sqrt((pred_xs - final_x)**2 + (pred_ys - final_y)**2)

                    # Relative time since analysis start
                    rel_times = pred_times - self.start_time

                    plot_data.append((rel_times, errors, idx))

            if plot_data:
                # Plot errors on the same axes with unique colors
                self.plot_errors(plot_data)
            else:
                self.get_logger().info("No valid landings to plot.")

            # Reset for next analysis window
            self.landed_instances = []
            self.current_predictions = []
            self.start_time = self.get_clock().now().nanoseconds * 1e-9

        # Restart the timer for the next analysis window
        self.analysis_timer = self.create_timer(self.analysis_duration, self.analysis_timer_callback)
        self.analysis_timer_active = True

    def plot_errors(self, plot_data: List[Tuple[np.ndarray, np.ndarray, int]]):
        """
        Plots the prediction errors for all landed instances.
        This function blocks until the plot window is closed.

        Args:
            plot_data: List of tuples containing relative times, errors, and instance index.
        """
        plt.figure(figsize=(10, 6))
        color_cycle = plt.rcParams['axes.prop_cycle'].by_key()['color']
        num_colors = len(color_cycle)

        for idx, (rel_times, errors, instance_num) in enumerate(plot_data):
            color = color_cycle[idx % num_colors]
            plt.plot(rel_times, errors, label=f'Landing {instance_num}', color=color)
            # Mark the landing time with a vertical dashed line
            if len(rel_times) > 0:
                landing_time = rel_times[-1]
                plt.axvline(landing_time, color=color, linestyle='--', label=f'Landing {instance_num} Time')

        plt.xlabel('Time since analysis start (s)')
        plt.ylabel('Prediction Error (mm)')
        plt.title('Landing Prediction Error Over Time')
        # plt.legend()
        plt.grid(True)

        self.get_logger().info("Displaying plot. Blocking until the plot window is closed.")
        plt.show()  # This will block execution until the plot window is closed
        self.get_logger().info("Plot window closed. Continuing to collect data.")

    def cleanup_dead_trackers(self):
        """
        Removes trackers that no longer meet the criteria for existence.

        Criteria for removal:
        - projectile_motion_confirmed == False
        - AND (measurement_buffer is empty OR last measurement older than buffer_duration)
        """
        with self.lock:
            dead_trackers = []
            current_time = self.get_clock().now().nanoseconds * 1e-9

            for tracker_id, tracker in self.trackers.items():
                if not tracker.projectile_motion_confirmed:
                    # Check if the tracker has no recent measurements
                    if len(tracker.measurement_buffer) == 0:
                        dead_trackers.append(tracker_id)
                    else:
                        last_measurement_time = tracker.measurement_buffer[-1][1]
                        time_since_last_measurement = current_time - last_measurement_time
                        if time_since_last_measurement > tracker.buffer_duration:
                            dead_trackers.append(tracker_id)
                # If projectile_motion_confirmed is True, the tracker will be removed once it lands

            # Remove dead trackers
            for tracker_id in dead_trackers:
                del self.trackers[tracker_id]
                self.get_logger().info(f"Removed dead tracker with ID {tracker_id}")

    def destroy_node(self):
        self.get_logger().info("Shutting down LandingAnalysisNode.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LandingAnalysisNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
