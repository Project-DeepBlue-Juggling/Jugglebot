"""
Target Tracker Node
-------------------
A ROS2 node that tracks targets (catching locations) from motion capture data

Key functionality:
- Subscribes to /rigid_body_poses for rigid body positions (e.g., Catching_Cone)
- Publishes target positions to /targets while valid data is available

Targets are locations that balls might want to be thrown to. Each target has:
- A unique ID (e.g., "catching_cone")
- A current position updated from mocap data
"""

import rclpy
from rclpy.node import Node
from jugglebot_interfaces.msg import RigidBodyPoses, Target, TargetArray
import numpy as np


class TargetTrackerNode(Node):
    def __init__(self):
        super().__init__('target_tracker_node')

        # Declare parameters for configurable target mappings
        # Maps rigid body names from mocap to target IDs
        self.declare_parameter('target_mappings', ['Catching_Cone:catching_cone'])
        
        # Parse target mappings: "RigidBodyName:target_id"
        self.target_mappings = {}
        mappings = self.get_parameter('target_mappings').get_parameter_value().string_array_value
        for mapping in mappings:
            if ':' in mapping:
                rigid_body_name, target_id = mapping.split(':', 1)
                self.target_mappings[rigid_body_name] = target_id
                self.get_logger().info(f"Target mapping: '{rigid_body_name}' -> '{target_id}'")
        
        # Store current target positions
        self.targets = {}  # target_id -> Target message
        
        # Subscriber for rigid body poses
        self.rigid_body_sub = self.create_subscription(
            RigidBodyPoses,
            '/rigid_body_poses',
            self.rigid_body_callback,
            10
        )
        
        # Publisher for targets
        self.targets_pub = self.create_publisher(TargetArray, '/targets', 10)
        
        # Timer to publish targets at regular intervals
        self.timer_frequency = 50  # Hz
        self.timer = self.create_timer(1.0 / self.timer_frequency, self.publish_targets)
        
        self.get_logger().info("TargetTrackerNode initialized.")

    def rigid_body_callback(self, msg: RigidBodyPoses):
        """
        Callback to handle incoming rigid body pose data.
        Updates target positions based on configured mappings.
        
        Args:
            msg: RigidBodyPoses message containing all tracked rigid bodies
        """
        for body in msg.bodies:
            if body.name in self.target_mappings:
                target_id = self.target_mappings[body.name]
                
                # Check if position data is valid (any NaN means the body is occluded/lost)
                pos = body.pose.pose.position
                is_valid = not (np.isnan(pos.x) or np.isnan(pos.y) or np.isnan(pos.z))
                
                if is_valid:
                    # Create or update target with valid position data
                    target = Target()
                    target.header = body.pose.header
                    target.id = target_id
                    target.position.x = pos.x
                    target.position.y = pos.y
                    target.position.z = pos.z
                    
                    self.targets[target_id] = target
                else:
                    # Invalid data (NaN) - remove target if it exists
                    if target_id in self.targets:
                        del self.targets[target_id]

    def publish_targets(self):
        """Timer callback to publish all current targets."""
        if len(self.targets) == 0:
            return
        
        msg = TargetArray()
        msg.targets = list(self.targets.values())
        self.targets_pub.publish(msg)

    def destroy_node(self):
        self.get_logger().info("Shutting down TargetTrackerNode.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TargetTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
