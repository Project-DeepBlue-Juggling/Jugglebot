"""
Holds the geometry for the robot and publishes the initial node positions to the 'get_robot_geometry' service
"""

from jugglebot_interfaces.srv import GetRobotGeometry
from std_srvs.srv import Trigger

import rclpy
from rclpy.node import Node
import numpy as np
import math


class RobotGeometry(Node):
    def __init__(self):
        super().__init__('robot_geometry')

        # Set up a service to get the robot geometry
        self.service = self.create_service(
            GetRobotGeometry, 'get_robot_geometry', self.handle_get_robot_geometry)
        
        # Initialize shutdown flag
        self.shutdown_flag = False
        # Set up a service to trigger closing the node
        self.service = self.create_service(Trigger, 'end_session', self.end_session)
        
        # Initialise the geometry parameters. Start with the platform
        # Nodes for the various elements
        self.base_nodes = np.zeros((6, 3))        # Base frame
        self.init_plat_nodes = np.zeros((6, 3))   # Platform frame
        self.init_arm_nodes = np.zeros((6, 3))    # Platform frame
        self.init_hand_nodes = np.zeros((3, 3))   # Platform frame
        self.new_plat_nodes = np.zeros((6, 3))    # Base frame
        self.new_arm_nodes = np.zeros((6, 3))     # Base frame
        self.new_hand_nodes = np.zeros((6, 3))    # Base frame

        self.init_leg_lengths = np.zeros((6, 1))

        ''' MAKE SURE ALL VALUES ARE FLOATS '''
        # All values have been updated to match the new platform (Jon's spaceframe design)

        self.initial_height = 565.0   # Dist. from the base plane (bottom joint of legs) to plat. in its lowest pos {mm}
        self.base_radius = 410.0      # Radius of base {mm}
        self.plat_radius = 219.075    # Radius of platform {mm}
        self.base_small_angle = 20.0  # Gamma2 on main sketch {deg}
        self.plat_small_angle = 8.6024446 # Lambda1 on main sketch {deg}
        self.plat_x_axis_offset = 154.3012223 # Lambda0 on main sketch {deg}
        self.leg_stroke = 280.0  # Stroke of leg {mm}

        # Relating to the hand/arm:
        self.arm_radius = 70.0  # Radius of opening where the ball comes in. Doesn't need to be exact.
        self.arm_height_from_platform = 210.25  # Height of opening where ball comes in from the platform leg nodes
        self.hand_stroke = 355.0  # Stroke of hand. DOES need to be ~exact. Used to inform overextensions etc.
        self.hand_radius = 35.0  # Radius of hand. Doesn't need to be exact

        self.start_pos = np.array([[0], [0], [self.initial_height]])

        # Build the platform
        self.build_platform()

    def build_platform(self):
        # Builds the stewart platform, calculating the initial positions of all nodes
        deg_to_rad = math.pi / 180

        # Define the angles to the nodes
        gamma2 = self.base_small_angle  # Angle between close base nodes {deg}
        gamma0 = 210 - gamma2 / 2  # Offset from horizontal
        gamma1 = 120 - gamma2  # Angle between far base nodes {deg}

        lambda1 = self.plat_small_angle  # Angle between close platform nodes {deg}
        lambda2 = 120 - lambda1  # Angle between far platform nodes {deg}
        lambda0 = self.plat_x_axis_offset  # Offset from x axis for platform nodes {deg} (measured in Onshape)

        # self.get_logger().info(f"Gamma0: {gamma0}, Gamma1: {gamma1}, Gamma2: {gamma2}")
        # self.get_logger().info(f"Lambda0: {lambda0}, Lambda1: {lambda1}, Lambda2: {lambda2}")

        base_node_angles = np.zeros((6, 1))  # Angles to each of the 6 base nodes {deg}
        plat_node_angles = np.zeros((6, 1))  # Angles to each of the 6 platform nodes {deg}

        # Find the nodes for the arm
        # Doesn't need to be perfectly realistic, so just assume arm nodes are equally spaced radially
        for node in range(6):
            # Find top three nodes first
            self.init_arm_nodes[node][0] = self.arm_radius * math.cos((np.pi * 2 / 3 * node))
            self.init_arm_nodes[node][1] = self.arm_radius * math.sin((np.pi * 2 / 3 * node))
            self.init_arm_nodes[node][2] = self.arm_height_from_platform

            if node > 2:
                # Then find the bottom three nodes
                self.init_arm_nodes[node][0] = self.init_arm_nodes[node - 3][0]
                self.init_arm_nodes[node][1] = self.init_arm_nodes[node - 3][1]
                self.init_arm_nodes[node][2] = self.init_arm_nodes[node - 3][2] - self.hand_stroke

        # Find the main 6 nodes for the base and platform
        for node in range(6):
            first_angle_index = int(math.floor(node / 2))
            second_angle_index = int(math.floor((node + 1) / 2))

            # self.get_logger().info(f"Node: {node}, First: {first_angle_index}, Second: {second_angle_index}")

            base_node_angles[node] = gamma0 + gamma1 * first_angle_index + gamma2 * second_angle_index
            plat_node_angles[node] = lambda0 + lambda1 * first_angle_index + lambda2 * second_angle_index

            # self.get_logger().info(f"ANGLES:\nNode: {node}, Base: {base_node_angles[node]}, Plat: {plat_node_angles[node]}")

            self.base_nodes[node][0] = self.base_radius * math.cos(base_node_angles[node] * deg_to_rad)
            self.base_nodes[node][1] = self.base_radius * math.sin(base_node_angles[node] * deg_to_rad)
            self.base_nodes[node][2] = 0

            self.init_plat_nodes[node][0] = self.plat_radius * math.cos(plat_node_angles[node] * deg_to_rad)
            self.init_plat_nodes[node][1] = self.plat_radius * math.sin(plat_node_angles[node] * deg_to_rad)
            self.init_plat_nodes[node][2] = 0

        # Find the nodes for the hand
        for node in range(3):
            self.init_hand_nodes[node][0] = self.hand_radius * math.cos(np.pi / 3 * (1 + 2 * node))
            self.init_hand_nodes[node][1] = self.hand_radius * math.sin(np.pi / 3 * (1 + 2 * node))
            self.init_hand_nodes[node][2] = 0

        # Log the base and platform nodes
        # self.get_logger().info(f"Base Nodes: \n{self.base_nodes}")
        # self.get_logger().info(f"Platform Nodes: \n{self.init_plat_nodes}")

        # Calculate the lengths of the legs in the initial state
        self.init_leg_lengths = np.linalg.norm(self.init_plat_nodes + self.start_pos.T - self.base_nodes, axis=1)

    def handle_get_robot_geometry(self, request, response):
        # Send all the node data
        response.start_pos = self.start_pos.flatten().tolist()
        response.base_nodes = self.base_nodes.flatten().tolist()
        response.init_plat_nodes = self.init_plat_nodes.flatten().tolist()
        response.init_arm_nodes = self.init_arm_nodes.flatten().tolist()
        response.init_hand_nodes = self.init_hand_nodes.flatten().tolist()
        response.init_leg_lengths = self.init_leg_lengths.flatten().tolist()
        response.leg_stroke = self.leg_stroke
        response.hand_stroke = self.hand_stroke

        return response

    #########################################################################################################
    #                                          Utility Functions                                            #
    #########################################################################################################

    def end_session(self, request, response):
        """Service callback to end the session from the GUI"""
        self.get_logger().info("End session requested. Shutting down...")
        response.success = True
        response.message = "Session ended. Shutting down node."
        self.shutdown_flag = True
        return response
        

def main(args=None):
    rclpy.init(args=args)
    node = RobotGeometry()

    try:
        while rclpy.ok() and not node.shutdown_flag:
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()