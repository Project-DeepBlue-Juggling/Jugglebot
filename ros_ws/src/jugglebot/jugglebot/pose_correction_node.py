#!/usr/bin/env python3
'''
Uses one of two approaches to correct the pose of a platform based on motion capture data.

1. Apply closed-loop control to the platform pose commands.
Receive the platform pose from the mocap system and apply corrections to the pose commands.

2. Apply feedforward control to the platform pose commands.
Analyse a pre-recorded dataset of mocap data and apply corrections to real-time pose commands.
'''

import rclpy
from rclpy.node import Node, SetParametersResult
from std_srvs.srv import Trigger
from jugglebot_interfaces.msg import PlatformPoseMessage
from geometry_msgs.msg import PoseStamped, Pose
import numpy as np
from scipy.spatial.transform import Rotation as R
from .pose_correction_ff_class import PoseCorrectionFF

class PoseCorrectionNode(Node):
    def __init__(self):
        super().__init__('pose_correction_node')

        # Initialize state variables
        self.shutdown_flag = False
        # Initialize a service to trigger closing the node.
        self.service = self.create_service(Trigger, 'end_session', self.end_session)

        # Configure the method of pose correction: "feedforward" for feedforward control, "closed_loop" for closed-loop control
        self.method = "feedforward"  
        # method = "closed_loop" 

        """
        Initialize subscribers and publishers
        """
        # Subscribe to platform_pose_topic and platform_pose_mocap.
        self.pose_subscription = self.create_subscription(
            PlatformPoseMessage, 
            'platform_pose_topic', 
            self.pose_cmd_callback, 
            10
        )
        self.mocap_subscription = self.create_subscription(
            PoseStamped, 
            'platform_pose_mocap', 
            self.mocap_callback, 
            10
        )

        # Create a publisher for the corrected platform pose.
        self.corrected_pose_publisher = self.create_publisher(
            PlatformPoseMessage, 
            'platform_pose_corrected', 
            10
        )

        # Store the latest commanded and measured poses, and the publisher identifier.
        self.latest_cmd_pose = None         # type: Pose
        self.latest_measured_pose = None    # type: Pose
        self.latest_pose_publisher = None   # type: str
        
        # Method 1: Closed-loop control
        if self.method == "closed_loop":
            # Declare and initialize gain parameters.
            # These control the magnitude of the correction applied.
            self.declare_parameter('k_trans', 1.2)
            self.declare_parameter('k_rot', 1.5)
            self.k_trans = self.get_parameter('k_trans').value
            self.k_rot   = self.get_parameter('k_rot').value

            # Enable dynamic parameter updates.
            self.add_on_set_parameters_callback(self.parameter_callback)
                    
            # Create a timer to publish the corrected pose (e.g., 100 Hz).
            timer_period = 0.01  # seconds
            self.timer = self.create_timer(timer_period, self.publish_corrected_pose)
        
        # Method 2: Feedforward control
        elif self.method == "feedforward":
            self.ff_correction = PoseCorrectionFF()
            self.calc_times = []

        self.get_logger().info("PoseCorrectionNode initialized")

    #########################################################################################################
    #                                    Dynamic Parameter Callback                                         #
    #########################################################################################################

    def parameter_callback(self, params):
        """
        Callback triggered when parameters are dynamically updated.
        Updates k_trans and k_rot and logs the new values.
        """
        for param in params:
            if param.name == 'k_trans':
                self.k_trans = param.value
                self.get_logger().info(f"Dynamic parameter update: k_trans updated to {self.k_trans}")
            if param.name == 'k_rot':
                self.k_rot = param.value
                self.get_logger().info(f"Dynamic parameter update: k_rot updated to {self.k_rot}")
        return SetParametersResult(successful=True)

    #########################################################################################################
    #                                           Subscriptions                                               #
    #########################################################################################################

    def pose_cmd_callback(self, msg: PlatformPoseMessage):
        """Callback for the platform pose command topic."""
        self.latest_cmd_pose = msg.pose_stamped.pose
        self.latest_pose_publisher = msg.publisher

        # If using feedforward control, apply the correction immediately.
        if self.method == "feedforward":
            self.apply_feedforward_correction(self.latest_cmd_pose)
        # If using closed-loop control, the correction will be applied in the publish_corrected_pose method.

    def mocap_callback(self, msg: PoseStamped):
        """Callback for the motion capture pose topic."""
        self.latest_measured_pose = msg.pose

    #########################################################################################################
    #                                         Main Control Loops                                            #
    #########################################################################################################

    # Method 1: Closed-loop control
    def publish_corrected_pose(self):
        """Publish the corrected pose based on the latest commanded and measured poses."""
        if self.latest_cmd_pose is None or self.latest_measured_pose is None:
            return

        # Build 4x4 transforms from commanded and measured poses.
        T_des = self.pose_to_homogeneous(self.latest_cmd_pose)
        T_meas = self.pose_to_homogeneous(self.latest_measured_pose)

        # Calculate the error transform: T_err = T_meas^-1 * T_des.
        try:
            T_err = np.linalg.inv(T_meas) @ T_des
        except np.linalg.LinAlgError:
            self.get_logger().error("Singular matrix encountered when calculating pose error.")
            return

        # Log the current error (for debugging).
        # pose_err = self.homogeneous_to_pose(T_err)
        # self.get_logger().info(
        #     f"Pose error (trans): {pose_err.position.x:.2f}, {pose_err.position.y:.2f}, {pose_err.position.z:.2f}; "
        #     f"Rot (quat): {pose_err.orientation.x:.2f}, {pose_err.orientation.y:.2f}, "
        #     f"{pose_err.orientation.z:.2f}, {pose_err.orientation.w:.2f}",
        #     throttle_duration_sec=0.5
        # )

        # Compute a correction transform 
        T_corr = self.compute_correction_transform(T_err, self.k_trans, self.k_rot)

        # Log the correction transform (for debugging).
        # correction_pose = self.homogeneous_to_pose(T_corr)
        # self.get_logger().info(
        #     f"Pose correction (trans): {correction_pose.position.x:.2f}, "
        #     f"{correction_pose.position.y:.2f}, {correction_pose.position.z:.2f}; "
        #     f"Rot (quat): {correction_pose.orientation.x:.2f}, "
        #     f"{correction_pose.orientation.y:.2f}, {correction_pose.orientation.z:.2f}, "
        #     f"{correction_pose.orientation.w:.2f}",
        #     throttle_duration_sec=0.5
        # )

        # The new corrected desired pose: T_corrected = T_meas * T_corr.
        T_corrected = T_meas @ T_corr
        corrected_pose = self.homogeneous_to_pose(T_corrected)

        # Construct and publish the corrected pose message.
        corrected_pose_msg = PlatformPoseMessage()
        corrected_pose_msg.pose_stamped.header.frame_id = 'platform_start'
        corrected_pose_msg.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        corrected_pose_msg.pose_stamped.pose = corrected_pose
        corrected_pose_msg.publisher = self.latest_pose_publisher

        self.corrected_pose_publisher.publish(corrected_pose_msg)

    # Method 2: Feedforward control
    def apply_feedforward_correction(self, pose_cmd: Pose):
        """Apply feedforward corrections to the commanded pose based on pre-recorded mocap data."""
        # Compute the corrected pose using the feedforward correction class.
        elapsed_time, index, corrected_pose = self.ff_correction.compute_corrected_pose(pose_cmd)

        # Record the computation time
        self.calc_times.append(elapsed_time)

        # Construct and publish the corrected pose message.
        corrected_pose_msg = PlatformPoseMessage()
        corrected_pose_msg.pose_stamped.header.frame_id = 'platform_start'
        corrected_pose_msg.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        corrected_pose_msg.pose_stamped.pose = corrected_pose
        corrected_pose_msg.publisher = self.latest_pose_publisher
        self.corrected_pose_publisher.publish(corrected_pose_msg)
        # Log the corrected pose.
        # self.get_logger().info(f"Corrected pose: {corrected_pose.position.x}, {corrected_pose.position.y}, {corrected_pose.position.z};"
        #                         f" {corrected_pose.orientation.x}, {corrected_pose.orientation.y}, "
        #                         f"{corrected_pose.orientation.z}, {corrected_pose.orientation.w}")
        

    #########################################################################################################
    #                                          Correction Helpers                                           #
    #########################################################################################################

    def compute_correction_transform(self, T_err: np.ndarray, k_trans: float, k_rot: float) -> np.ndarray:
        """
        Compute a correction transform T_corr from the pose error T_err.
        This uses the logarithmic map of the rotation and scales the translation and rotation errors.
        """
        # Extract translation and rotation from the error.
        p_err = T_err[0:3, 3]
        R_err = T_err[0:3, 0:3]

        # Compute the rotation error as a rotation vector (log map).
        r_err = R.from_matrix(R_err)
        rotvec_err = r_err.as_rotvec()  # 3D rotation vector

        # Scale the translation and rotation error components.
        p_corr = k_trans * p_err
        rotvec_corr = k_rot * rotvec_err

        # Reconstruct the correction rotation matrix (exp map).
        R_corr = R.from_rotvec(rotvec_corr).as_matrix()

        # Build the homogeneous correction transform.
        T_corr = np.eye(4)
        T_corr[0:3, 0:3] = R_corr
        T_corr[0:3, 3] = p_corr
        return T_corr

    #########################################################################################################
    #                                          Auxiliary Methods                                            #
    #########################################################################################################

    def pose_to_homogeneous(self, pose: Pose) -> np.ndarray:
        """Convert a geometry_msgs/Pose to a 4x4 homogeneous transformation matrix."""
        translation = pose.position
        rotation = pose.orientation

        T = np.eye(4)
        T[0:3, 3] = [translation.x, translation.y, translation.z]
        T[0:3, 0:3] = self.quaternion_to_rotation_matrix(rotation)
        return T

    def quaternion_to_rotation_matrix(self, quaternion) -> np.ndarray:
        """Convert a quaternion to a 3x3 rotation matrix using scipy."""
        q = np.array([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        norm = np.linalg.norm(q)
        if norm < 1e-10:
            return np.eye(3)
        q /= norm
        r = R.from_quat(q)
        return r.as_matrix()

    def rotation_matrix_to_quaternion(self, rot: np.ndarray) -> np.ndarray:
        """Convert a 3x3 rotation matrix to a quaternion using scipy."""
        r = R.from_matrix(rot)
        return r.as_quat()

    def homogeneous_to_pose(self, T: np.ndarray) -> Pose:
        """Convert a 4x4 homogeneous transformation matrix to a geometry_msgs/Pose."""
        pose = Pose()
        translation = T[0:3, 3]
        pose.position.x = translation[0]
        pose.position.y = translation[1]
        pose.position.z = translation[2]
        R_mat = T[0:3, 0:3]
        q = self.rotation_matrix_to_quaternion(R_mat)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        return pose

    def compute_computation_time_statistics(self):
        """
        Compute and log the average, maximum, and standard deviation of the computation times.
        This is used for the feedforward method.
        """
        if len(self.calc_times) > 0:
            avg_time = np.mean(self.calc_times)
            max_time = np.max(self.calc_times)
            stdev_time = np.std(self.calc_times)
            self.get_logger().info("=====================")
            self.get_logger().info(f"Average computation time: {avg_time:.2f} ms")
            self.get_logger().info(f"Max computation time: {max_time:.2f} ms")
            self.get_logger().info(f"Standard deviation of computation times: {stdev_time:.2f} ms")
            self.get_logger().info("=====================")
        else:
            self.get_logger().info("No computation times recorded.")

    #########################################################################################################
    #                                          Node Management                                              #
    #########################################################################################################

    def end_session(self, request, response):
        """Service callback to end the session from the GUI."""
        self.get_logger().info("End session requested. Shutting down...")
        response.success = True
        response.message = "Session ended. Shutting down node."
        self.shutdown_flag = True
        return response

    def on_shutdown(self):
        """Cleanup method called when the node is shutting down."""
        self.get_logger().info("Shutting down PoseCorrectionNode...")

        if self.method == "feedforward":
            # Log the average, max and stdev of the computation times.
            self.compute_computation_time_statistics()

def main(args=None):
    rclpy.init(args=args)
    node = PoseCorrectionNode()

    try:
        while rclpy.ok() and not node.shutdown_flag:
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Shutting down.")
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
