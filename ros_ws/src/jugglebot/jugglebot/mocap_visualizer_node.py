""" This node is responsible for visualizing the mocap markers and predicted landings in RViz.
"""

import rclpy
from rclpy.node import Node
from jugglebot_interfaces.msg import MocapDataMulti, MocapDataSingle
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA
import uuid
import numpy as np

class MocapVisualizerNode(Node):
    """
    ROS2 Node that visualizes mocap markers and predicted landings in RViz.
    """

    def __init__(self):
        super().__init__('mocap_visualizer_node')

        # Subscribers
        self.mocap_subscriber = self.create_subscription(
            MocapDataMulti,
            '/mocap_data',
            self.mocap_data_callback,
            10
        )

        self.landing_subscriber = self.create_subscription(
            Point,
            '/predicted_landings',
            self.landing_callback,
            10
        )

        # Publisher for visualization markers
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            '/visualization_marker',
            10
        )

        # Store marker IDs to maintain consistency
        self.marker_id_map = {}  # Maps marker UUID to visualizer marker ID
        self.next_marker_id = 0  # Sequential ID counter
        
        self.marker_paths = {}  # Dictionary to store paths for each marker ID

        # Unique namespace for markers
        self.namespace = 'mocap_markers'

        # Parameters
        self.association_distance_threshold = 50.0  # mm

        self.get_logger().info("MocapVisualizerNode has been initialized.")

    def mocap_data_callback(self, msg: MocapDataMulti):
        """
        Callback to handle incoming mocap data and create visualization markers.

        Args:
            msg (MocapDataMulti): The incoming mocap data containing multiple markers.
        """
        marker_array = MarkerArray()

        for marker in msg.unlabelled_markers:
            # Convert position from mm to meters
            position_meters = Point()
            position_meters.x = marker.position.x / 1000.0
            position_meters.y = marker.position.y / 1000.0
            position_meters.z = marker.position.z / 1000.0

            # Assign a unique marker ID
            vis_id = self.get_unique_marker_id()

            # Create visualization marker
            vis_marker = Marker()
            vis_marker.header = Header()
            vis_marker.header.stamp = self.get_clock().now().to_msg()
            vis_marker.header.frame_id = 'map'

            vis_marker.ns = self.namespace
            vis_marker.id = vis_id
            vis_marker.type = Marker.SPHERE
            vis_marker.action = Marker.ADD

            vis_marker.pose.position = position_meters
            vis_marker.pose.orientation.x = 0.0
            vis_marker.pose.orientation.y = 0.0
            vis_marker.pose.orientation.z = 0.0
            vis_marker.pose.orientation.w = 1.0

            # Scale of the sphere (radius) in meters
            vis_marker.scale.x = 0.05  # 50 mm diameter -> 0.05 m
            vis_marker.scale.y = 0.05
            vis_marker.scale.z = 0.05

            # Color (e.g., red with some transparency)
            vis_marker.color = ColorRGBA()
            vis_marker.color.r = 1.0
            vis_marker.color.g = 0.0
            vis_marker.color.b = 0.0
            vis_marker.color.a = 0.8

            # Update path for this marker
            marker_id = vis_id  # Use vis_id or any unique identifier for the marker
            if marker_id not in self.marker_paths:
                self.marker_paths[marker_id] = []

            # Append the current position to the path
            self.marker_paths[marker_id].append(position_meters)

            # Keep only the last N positions to limit the length of the path
            N = 100
            if len(self.marker_paths[marker_id]) > N:
                self.marker_paths[marker_id].pop(0)

            # Create a line strip marker for the path
            path_marker = Marker()
            path_marker.header = vis_marker.header
            path_marker.ns = f"{self.namespace}_paths"
            path_marker.id = vis_id + 10000  # Offset ID to avoid conflicts
            path_marker.type = Marker.LINE_STRIP
            path_marker.action = Marker.ADD

            path_marker.scale.x = 0.1  # Line width in meters

            # Color (e.g., blue)
            path_marker.color = ColorRGBA()
            path_marker.color.r = 0.0
            path_marker.color.g = 0.0
            path_marker.color.b = 1.0
            path_marker.color.a = 0.8

            path_marker.points = self.marker_paths[marker_id]

            # Lifetime of the markers
            path_marker.lifetime = rclpy.duration.Duration(seconds=0.1).to_msg()
            vis_marker.lifetime = rclpy.duration.Duration(seconds=0.1).to_msg()

            marker_array.markers.append(path_marker)
            marker_array.markers.append(vis_marker)

        # Publish all markers at once
        self.marker_publisher.publish(marker_array)
        self.get_logger().debug(f"Published {len(marker_array.markers)} mocap markers to RViz.")

    def landing_callback(self, msg: Point):
        """
        Callback to handle predicted landing points and create visualization markers.

        Args:
            msg (Point): The predicted landing position.
        """
        # Convert position from mm to meters
        position_meters = Point()
        position_meters.x = msg.x / 1000.0
        position_meters.y = msg.y / 1000.0
        position_meters.z = msg.z / 1000.0

        landing_marker = Marker()
        landing_marker.header = Header()
        landing_marker.header.stamp = self.get_clock().now().to_msg()
        landing_marker.header.frame_id = 'map'

        landing_marker.ns = 'predicted_landings'
        landing_marker.id = self.get_unique_marker_id()
        landing_marker.type = Marker.CYLINDER
        landing_marker.action = Marker.ADD

        landing_marker.pose.position = position_meters
        landing_marker.pose.orientation.x = 0.0
        landing_marker.pose.orientation.y = 0.0
        landing_marker.pose.orientation.z = 0.0
        landing_marker.pose.orientation.w = 1.0

        # Scale of the cylinder in meters
        landing_marker.scale.x = 0.1  # 100 mm diameter -> 0.1 m
        landing_marker.scale.y = 0.1
        landing_marker.scale.z = 0.05  # 50 mm height -> 0.05 m

        # Color (e.g., green with some transparency)
        landing_marker.color = ColorRGBA()
        landing_marker.color.r = 0.0
        landing_marker.color.g = 1.0
        landing_marker.color.b = 0.0
        landing_marker.color.a = 0.8

        # Lifetime of the marker
        landing_marker.lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()

        # Create MarkerArray with a single marker
        marker_array = MarkerArray()
        marker_array.markers.append(landing_marker)

        # Publish the landing marker
        self.marker_publisher.publish(marker_array)
        self.get_logger().debug(f"Published predicted landing marker at ({position_meters.x}, {position_meters.y}, {position_meters.z}) m.")


    def get_unique_marker_id(self) -> int:
        """
        Generates a unique marker ID within the valid range.

        Returns:
            int: A unique marker ID.
        """
        if self.next_marker_id > 2147483647:
            self.next_marker_id = 0  # Reset to 0 to stay within int32 range

        unique_id = self.next_marker_id
        self.next_marker_id += 1
        return unique_id

    def destroy_node(self):
        """
        Clean up before shutting down the node.
        """
        self.get_logger().info("Shutting down MocapVisualizerNode.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MocapVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("MocapVisualizerNode stopped manually.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
