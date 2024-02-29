import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose
from std_srvs.srv import Trigger
from std_msgs.msg import Float64MultiArray, Int8MultiArray
from jugglebot_interfaces.srv import GetRobotGeometry
import quaternion  # numpy quaternion
import time  # To time how long the numeric solution(s) take

class SPInverseKinematics(Node):
    def __init__(self):
        super().__init__('hand_ik')

        # Set up a service to trigger closing the node
        self.service = self.create_service(Trigger, 'end_session', self.end_session)

        # Set up service client to get robot geometry
        self.geometry_client = self.create_client(GetRobotGeometry, 'get_robot_geometry')
        
        while not self.geometry_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Waiting for "get_robot_geometry" service...')

        self.send_geometry_request()

        self.subscription = self.create_subscription(PoseStamped, 'hand_pose_topic', self.pose_callback, 10)
        self.subscription  # Prevent "unused variable" warning

        # Set up a publisher to publish the platform pose
        self.platform_pose_publisher = self.create_publisher(Pose, 'platform_pose_topic', 10)

        # Initialize flag to track whether geometry data has been received or not
        self.has_geometry_data = False

        # Initialize geometry terms to be populated with a call to the get_robot_geometry service
        self.start_pos  = None        # Base frame
        self.base_nodes = None        # Base frame
        self.init_plat_nodes = None   # Platform frame
        self.init_arm_nodes  = None   # Platform frame
        self.init_hand_nodes = None   # Platform frame

        self.init_leg_lengths = None
        self.leg_stroke  = None
        self.hand_stroke = None # For checking if the hand string is overextended

        self.new_plat_nodes = None    # Base frame
        self.new_arm_nodes  = None    # Base frame
        self.new_hand_nodes = None    # Base frame

        # Initialize a variable to store the quaternion representation of the platform/hand orientation
        self.orientation = None

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
            self.init_leg_lengths = np.array(response.init_leg_lengths).reshape(7,)
            self.leg_stroke = response.leg_stroke
            self.hand_stroke = response.hand_stroke

            # Report the receipt of data
            self.get_logger().info('Received geometry data!')

            # Record the receipt of data so that we know we've got it
            self.has_geometry_data = True
        else:
            self.get_logger().error('Exception while calling "get_robot_geometry" service')

    def pose_callback(self, msg):
        if self.has_geometry_data:
            # Extract position data
            pos = np.array([[msg.pose.position.x], [msg.pose.position.y], [msg.pose.position.z]])  # Note that this is in base frame
            pos_adjusted = pos + self.start_pos  # Convert to platform (at lowest position) frame

            # Extract the orientation quaternion
            self.orientation = msg.pose.orientation
            quaternion_ori = quaternion.quaternion(self.orientation.w, self.orientation.x, self.orientation.y, self.orientation.z)
            
            # # Convert quaternion to 4x4 rotation matrix
            # rot = quaternion.as_rotation_matrix(quaternion_ori)

            # # Extract the 3x3 rotation matrix
            # rot = rot[:3, :3]

            # Log the position vector as a single line
            # self.get_logger().info(f"Position vector: {pos.flatten()}")

            # Log the orientation vector
            # self.get_logger().info(f"Orientation vector: {quaternion_ori}")

            # Now solve the hand IK to get the platform pose
            self.solve_hand_IK(pos_adjusted, quaternion_ori)

    def solve_hand_IK(self, pos, rot_quat):
        '''
        Applies the Newton-Raphson method to numerically solve for the position of the platform that will result in
        the hand being in the correct spot
        '''
        start_time = time.perf_counter_ns()
        
        # Rename variables to something a little more usable
        A_s = np.reshape(self.init_plat_nodes[6], (3, 1))   # Location of plat string attachment, in plat frame
        Q_A_s = np.quaternion(0, *A_s)  # Quaternion representation of A_s
        B_s = np.reshape(self.base_nodes[6], (3, 1))        # Location of base string attachment, in base frame
        e_o = np.quaternion(0, 0, 0, 1)  # Unit vector in z direction
        L_o = self.init_leg_lengths[6]  # Length of hand string at origin position
        h = pos # Desired hand position

        # Rotate the unit vector e_o to correspond to the hand orientation
        e_o = rot_quat * e_o * rot_quat.conjugate()

        # self.get_logger().info(f"e_o: {e_o}") # Log the rotated unit vector

        # Extract the rotated unit vector components
        e_o = np.array([e_o.x, e_o.y, e_o.z]).reshape(3, 1)

        def F(P):
            # Equation describing the position of the platform wrt other variables
            return P + (np.linalg.norm(P - B_s) - L_o) * e_o - h

        def dFdP(P):
            # Derivative of F wrt P
            return 1 + np.dot((P - B_s).reshape(1, 3), e_o) / np.linalg.norm(P - B_s)

        # Set some initial guess for P. For now, try just using the hand position since it will always be close
        # Note that P is the location of the platform string attachment in the BASE frame
        P_guess = h + np.mean(self.init_hand_nodes, axis=0).reshape(3, 1)

        # Tolerance for the solution
        tol = 1e-6

        # Maximum number of iterations
        max_iter = 100

        # Apply Newton-Raphson method
        for i in range(max_iter):
            F_val = F(P_guess)
            error = np.linalg.norm(F_val)
            if error < tol:
                # If the solution is close enough to the true answer, stop
                break
            # If solution is not close enough, update the guess
            P_guess = P_guess - F_val / dFdP(P_guess)

            if i == max_iter - 1:
                # self.get_logger().error(f"No suitable solution found after {max_iter} steps. Final numeric error = {error}")
                pass

        new_A_s = rot_quat * Q_A_s * rot_quat.conjugate()
        new_A_s = np.array([new_A_s.x, new_A_s.y, new_A_s.z]).reshape(3, 1)

        plat_pos = P_guess - new_A_s - self.start_pos  

        end_time = time.perf_counter_ns()

        # self.get_logger().info(f"Numeric solution took {(end_time - start_time) / 1e6} ms")
        
        # Publish the platform pose
        plat_pose_msg = Pose()
        plat_pose_msg.position.x = plat_pos[0][0]
        plat_pose_msg.position.y = plat_pos[1][0]
        plat_pose_msg.position.z = plat_pos[2][0]
        plat_pose_msg.orientation = self.orientation
        # plat_pose_msg.orientation.x = rot_quat.x
        # plat_pose_msg.orientation.y = rot_quat.y
        # plat_pose_msg.orientation.z = rot_quat.z
        # plat_pose_msg.orientation.w = rot_quat.w

        self.platform_pose_publisher.publish(plat_pose_msg)

    def end_session(self, request, response):
        # The method that's called when a user clicks "End Session" in the GUI
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = SPInverseKinematics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass
    finally:
        node.get_logger().info("Shutting down...")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
