import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from geometry_msgs.msg import Pose, Point
from std_srvs.srv import Trigger
from jugglebot_interfaces.srv import GetRobotGeometry
from jugglebot_interfaces.msg import RobotPlotterMessage
import quaternion  # numpy quaternion

class PlatformPlotter(Node):
    def __init__(self):
        super().__init__('platform_plotter')
        # Set up service client to get robot geometry
        self.geometry_client = self.create_client(GetRobotGeometry, 'get_robot_geometry')
        
        while not self.geometry_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Waiting for "get_robot_geometry" service...')

        self.send_geometry_request()

        # Set up subscriber for platform pose
        self.subscription = self.create_subscription(Pose, 'platform_pose_topic', self.pose_callback, 10)
        self.subscription  # prevent unused variable warning

        # Set up publisher for the robot nodes
        self.node_location_publisher = self.create_publisher(RobotPlotterMessage, 'robot_plotter_topic', 10)

        # Set up a service to trigger closing the node
        self.service = self.create_service(Trigger, 'end_session', self.end_session)

        # Initialize flag to track whether geometry data has been received or not
        self.has_geometry_data = False

        # Initialize geometry terms to be populated with a call to the get_robot_geometry service
        self.start_pos = None         # Base frame
        self.base_nodes = None        # Base frame
        self.init_plat_nodes = None   # Platform frame
        self.init_arm_nodes = None    # Platform frame
        self.init_hand_nodes = None   # Platform frame

        self.init_leg_lengths = None  # Frameless
        self.leg_stroke = None

        self.new_plat_nodes = None    # Base frame
        self.new_arm_nodes = None     # Base frame
        self.new_hand_nodes = None    # Base frame

        # Initialize geometry to be populated with the subscription to platform_pose_topic
        self.ori_quat = None   # Orientation of the platform in the base frame (as a quaternion)

        self.axis_lims = None  # To keep the plot from constantly changing size

        # Set up Matplotlib
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')

        plt.ion()
        plt.show()

    def send_geometry_request(self):
        req = GetRobotGeometry.Request()
        self.future = self.geometry_client.call_async(req)
        self.future.add_done_callback(self.handle_geometry_response)

    def handle_geometry_response(self, future):
        response = future.result()
        if response is not None:
            # Store the geometry data after converting it to numpy arrays
            self.start_pos = np.array(response.start_pos).reshape(3, 1)
            self.base_nodes = np.array(response.base_nodes).reshape(7, 3)
            self.init_plat_nodes = np.array(response.init_plat_nodes).reshape(7, 3)
            self.init_arm_nodes = np.array(response.init_arm_nodes).reshape(6, 3)
            self.init_hand_nodes = np.array(response.init_hand_nodes).reshape(3, 3)
            self.init_leg_lengths = np.array(response.init_leg_lengths).reshape(7, 1)
            self.leg_stroke = response.leg_stroke

            ## Create axis limits based on the retrieved data
            # Define a multiplier to get the scale to look nice
            multiplier = 1.5

            # Extract x (column 0) and y (column 1) values
            x_values = self.base_nodes[:, 0]
            y_values = self.base_nodes[:, 1]

            # Find min and max, then apply the multiplier
            x_min = np.min(x_values) * multiplier
            x_max = np.max(x_values) * multiplier
            y_min = np.min(y_values) * multiplier
            y_max = np.max(y_values) * multiplier

            z_min = -0.2
            z_max = (self.start_pos[-1, 0] + self.leg_stroke) * multiplier

            # Store the results in an array called "axis_lims"
            self.axis_lims = np.array([[x_min, x_max], [y_min, y_max], [z_min, z_max]])

            # Report the receipt of data
            self.get_logger().info('Received geometry data!')

            # Record the receipt of data so that we know we've got it
            self.has_geometry_data = True
        else:
            self.get_logger().error('Exception while calling "get_robot_geometry" service')

    def pose_callback(self, msg):
        if self.has_geometry_data:
            # Clear the plot
            self.ax.clear()

            # Extract position data
            pos = np.array([[msg.position.x], [msg.position.y], [msg.position.z]])

            # Extract the orientation quaternion
            self.ori_quat = msg.orientation
            quaternion_ori = quaternion.quaternion(self.ori_quat.w, self.ori_quat.x, self.ori_quat.y, self.ori_quat.z)
            
            # Convert quaternion to 4x4 rotation matrix
            rot = quaternion.as_rotation_matrix(quaternion_ori)

            # Extract the 3x3 rotation matrix
            rot = rot[:3, :3]

            # Use this data to update the locations of all the platform nodes
            self.update_pose(pos, rot)

    def update_pose(self, pos, rot):
        # Calculate the positions of all nodes

        new_position = pos + self.start_pos

        # Update plat_nodes to reflect change in pose
        self.new_plat_nodes = (new_position + np.dot(rot, self.init_plat_nodes.T)).T

        # Update arm nodes
        self.new_arm_nodes = (new_position + np.dot(rot, self.init_arm_nodes.T)).T

        # Calculate the length of the hand string (leg 7)
        string_length = np.linalg.norm(self.new_plat_nodes[6] - self.base_nodes[6])

        # Update hand nodes
        change_in_string_length = string_length - self.init_leg_lengths[-1]
        self.unit_ori_vector = np.dot(rot, np.array([[0], [0], [1]]))
        self.new_hand_nodes = ((new_position + np.dot(rot, self.init_hand_nodes.T)) +
                               change_in_string_length * self.unit_ori_vector ).T
        self.hand_centroid = np.mean(self.new_hand_nodes, axis=0)

        # Send the data off to be plotted, or published (or both)
        self.publish_robot_nodes()
        # self.plot_platform()

    def publish_robot_nodes(self):
        # Publishes the locations of the robot nodes to robot_plotter_topic
        # Create a RobotPlotter message
        msg = RobotPlotterMessage()

        # Convert numpy arrays to Point messages and assign to the RobotPlotter message
        msg.base_nodes = [self.numpy_to_point(node) for node in self.base_nodes]
        msg.new_plat_nodes = [self.numpy_to_point(node) for node in self.new_plat_nodes]
        msg.new_arm_nodes = [self.numpy_to_point(node) for node in self.new_arm_nodes]
        msg.new_hand_nodes = [self.numpy_to_point(node) for node in self.new_hand_nodes]
        msg.orientation = self.ori_quat

        # Publish the message
        self.node_location_publisher.publish(msg)

    def numpy_to_point(self, numpy_array):
        # Converts a numpy array to a Point message
        point = Point()
        point.x = numpy_array[0]
        point.y = numpy_array[1]
        point.z = numpy_array[2]

        return point

    def plot_platform(self):
        # Plots the platform

        plat_points =  self.ax.scatter(self.new_plat_nodes[:6].T[0], self.new_plat_nodes[:6].T[1], 
                                       self.new_plat_nodes[:6].T[2], color='r')

        base_points = self.ax.scatter(self.base_nodes.T[0], self.base_nodes.T[1], self.base_nodes.T[2],
                                      color='b')

        # Plot the hexagons
        arm_hex_poly = Poly3DCollection([self.new_arm_nodes[3:]], alpha=0.3, facecolor='r')
        hand_hex_poly = Poly3DCollection([self.new_hand_nodes], alpha=0.8, facecolors='m')
        base_hex_poly = Poly3DCollection([self.base_nodes[:6]], alpha=0.3, facecolor='b')
        # plat_hex_poly = Poly3DCollection([self.new_plat_nodes[:6]], alpha=0.3, facecolor='r')

        self.ax.add_collection(arm_hex_poly)
        # self.ax.add_collection(plat_hex_poly)
        self.ax.add_collection(hand_hex_poly)
        self.ax.add_collection(base_hex_poly)

        # Plot the legs
        legs = []
        for leg in range(6):
            legs.append(self.ax.plot3D([self.base_nodes[leg][0], self.new_plat_nodes[leg][0]],
                                            [self.base_nodes[leg][1], self.new_plat_nodes[leg][1]],
                                            [self.base_nodes[leg][2], self.new_plat_nodes[leg][2]],
                                            'g')[0])

        # Plot the struts that connect the platform nodes to arm nodes
        struts = []
        for strut in range(6):
            temp = int(round(np.ceil(strut/2)) % 3)
            struts.append(self.ax.plot3D([self.new_plat_nodes[strut][0], self.new_arm_nodes[temp][0]],
                                              [self.new_plat_nodes[strut][1], self.new_arm_nodes[temp][1]],
                                              [self.new_plat_nodes[strut][2], self.new_arm_nodes[temp][2]],
                                              'k', linewidth=0.75)[0])

        # Plot the rods that connect the lower arm to the upper arm
        rods = []
        for rod in range(3):
            rods.append(self.ax.plot3D([self.new_arm_nodes[rod][0], self.new_arm_nodes[rod + 3][0]],
                                            [self.new_arm_nodes[rod][1], self.new_arm_nodes[rod + 3][1]],
                                            [self.new_arm_nodes[rod][2], self.new_arm_nodes[rod + 3][2]],
                                            'k', linewidth=0.75)[0])

        # Redraw the plot
        self.ax.set_xlim(self.axis_lims[0])
        self.ax.set_ylim(self.axis_lims[1])
        self.ax.set_zlim(self.axis_lims[2])
        plt.draw()
        plt.pause(0.001)

    def on_shutdown(self):
        # Close the plot etc. cleanly
        plt.ioff()
        plt.close()

    def end_session(self, request, response):
        # The method that's called when a user clicks "End Session" in the GUI
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = PlatformPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass
    finally:
        node.get_logger().info("Shutting down...")
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()