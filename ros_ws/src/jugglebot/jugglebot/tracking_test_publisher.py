""" This node runs a simulation of the tracking system by publishing simulated mocap data
of a ball in flight. It simulates occlusion and can be extended to support multiple markers.
"""
import rclpy
from rclpy.node import Node
from jugglebot_interfaces.msg import MocapDataMulti, MocapDataSingle
from geometry_msgs.msg import Point
from typing import Dict, Tuple
import numpy as np
import random


# Type aliases for clarity
Position = np.ndarray
Velocity = np.ndarray
PositionRange = Dict[str, Tuple[float, float]]
VelocityRange = Dict[str, Tuple[float, float]]


class Marker:
    """
    Represents a simulated marker following projectile motion.
    """

    def __init__(
        self,
        initial_position: Position,
        initial_velocity: Velocity,
        position_range: PositionRange,
        velocity_range: VelocityRange
    ):
        """
        Initializes a Marker with position and velocity.

        Args:
            initial_position (Position): Initial position [x, y, z] in mm.
            initial_velocity (Velocity): Initial velocity [vx, vy, vz] in mm/s.
            position_range (PositionRange): Ranges for x, y, z positions.
            velocity_range (VelocityRange): Ranges for vx, vy, vz velocities.
        """
        self.position_range = position_range
        self.velocity_range = velocity_range

        self.position = initial_position.copy()
        self.velocity = initial_velocity.copy()
        self.active = True  # Indicates if the marker is currently somewhere between thrown and landed
        self.occluded = False  # Indicates if the marker is "occluded"
        self.reset_scheduled = False  # Flag to prevent multiple reset timers

    def update(self, dt: float, gravity: float = -9810.0):
        """
        Updates the marker's position based on its velocity and gravity.

        Args:
            dt (float): Time step in seconds.
            gravity (float): Acceleration due to gravity in mm/s².
        """
        if not self.active:
            return

        # Update velocity with gravity affecting the z-axis
        self.velocity[2] += gravity * dt  # gravity is negative

        # Update position with velocity
        self.position += self.velocity * dt

    def simulate_occlusion(self, occlusion_probability: float = 0.05):
        """
        Simulates occlusion by randomly deactivating or reactivating the marker.

        Args:
            occlusion_probability (float): Probability of occlusion at each update.
        """
        if self.occluded:
            # Chance to recover from occlusion
            if random.random() < 0.1:
                self.occluded = False
        else:
            # Chance to become occluded
            if random.random() < occlusion_probability:
                self.occluded = True

    def reset(self) -> Tuple[Position, Velocity]:
        """
        Resets the marker to new random position and velocity within specified ranges.

        Returns:
            Tuple[Position, Velocity]: The new position and velocity of the marker.
        """
        # Assign new random position within the specified ranges
        self.position = np.array([
            random.uniform(*self.position_range['x']),
            random.uniform(*self.position_range['y']),
            random.uniform(*self.position_range['z'])
        ])

        # Assign new random velocity within the specified ranges
        self.velocity = np.array([
            random.uniform(*self.velocity_range['vx']),
            random.uniform(*self.velocity_range['vy']),
            random.uniform(*self.velocity_range['vz'])
        ])

        self.active = True
        self.occluded = False
        self.reset_scheduled = False

        return self.position, self.velocity


class TrackingTestPublisher(Node):
    """
    ROS2 Node that publishes simulated mocap data for testing purposes.
    """

    def __init__(self):
        super().__init__('tracking_test_publisher')

        # Publisher
        self.publisher_ = self.create_publisher(MocapDataMulti, '/mocap_data', 10)

        # Timer setup for publishing at 100 Hz
        timer_period = 1.0 / 100.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_mocap_data)

        # Simulation parameters
        self.gravity = -9810.0  # mm/s²
        self.dt = timer_period  # Time step
        self.reset_delay = 2.0  # seconds delay before resetting after landing

        # Define ranges for initial state position and velocity
        self.position_range: PositionRange = {
            'x': (0, 1000),      # mm
            'y': (0, 1000),      # mm
            'z': (500, 1500)     # mm (above ground)
        }
        self.velocity_range: VelocityRange = {
            'vx': (-1000, 1000),  # mm/s
            'vy': (-1000, 1000),  # mm/s
            'vz': (1000, 5000)     # mm/s (initially moving upwards)
        }

        # Initialize a single marker with random position and velocity
        initial_position = np.array([
            random.uniform(*self.position_range['x']),
            random.uniform(*self.position_range['y']),
            random.uniform(*self.position_range['z'])
        ])
        initial_velocity = np.array([
            random.uniform(*self.velocity_range['vx']),
            random.uniform(*self.velocity_range['vy']),
            random.uniform(*self.velocity_range['vz'])
        ])
        self.marker = Marker(
            initial_position, initial_velocity,
            self.position_range, self.velocity_range
        )
        self.get_logger().info(
            f"Initialized marker with position {initial_position} mm "
            f"and velocity {initial_velocity} mm/s."
        )

        # Timer for resetting the marker after landing
        self.reset_timer = None

        self.get_logger().info("TrackingTestPublisher has been initialized.")

    def publish_mocap_data(self):
        """
        Publishes MocapDataMulti messages with updated marker positions.
        """
        msg = MocapDataMulti()

        # Update marker state
        self.marker.update(self.dt, self.gravity)

        # Simulate occlusion
        # self.marker.simulate_occlusion()

        if self.marker.active and not self.marker.occluded:
            mocap_marker = MocapDataSingle()
            mocap_marker.position = Point()
            mocap_marker.position.x = float(self.marker.position[0])
            mocap_marker.position.y = float(self.marker.position[1])
            mocap_marker.position.z = float(self.marker.position[2])

            # Simulate residual as a small random value (noise) in mm
            mocap_marker.residual = float(random.uniform(0.5, 3.0))

            msg.unlabelled_markers.append(mocap_marker)

        # Check for landing
        if self.marker.position[2] <= 0 and self.marker.active and not self.marker.reset_scheduled:
            self.get_logger().info("Marker has landed.")
            self.marker.active = False
            self.marker.reset_scheduled = True
            # Schedule reset after reset_delay seconds
            self.reset_timer = self.create_timer(
                self.reset_delay,
                self.reset_marker
            )

        # Publish the message
        self.publisher_.publish(msg)
        self.get_logger().debug(f"Published MocapDataMulti with {len(msg.unlabelled_markers)} markers.")

    def reset_marker(self):
        """
        Resets the marker after the reset delay has elapsed.
        """
        new_pos, new_vel = self.marker.reset()
        self.get_logger().info(
            f"Resetting marker to new random initial state: "
            f"Position {new_pos} mm, Velocity {new_vel} mm/s."
        )
        # Destroy the reset timer as it's no longer needed
        if self.reset_timer is not None:
            self.reset_timer.cancel()
            self.reset_timer = None

    def destroy_node(self):
        """
        Clean up before shutting down the node.
        """
        self.get_logger().info("Shutting down TrackingTestPublisher.")
        # Cancel the reset timer if it's active
        if self.reset_timer is not None:
            self.reset_timer.cancel()
            self.reset_timer = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TrackingTestPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("TrackingTestPublisher node stopped manually.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
