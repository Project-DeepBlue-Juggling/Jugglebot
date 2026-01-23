#!/usr/bin/env python3

"""
PoseCorrectionNode: Applies pose corrections to platform commands.

Methods:
- 'feedforward' : corrects commands based on pre-recorded KD-tree errors.
- 'none'        : no correction, raw commands passed through.
"""

import rclpy
from rclpy.node import Node, SetParametersResult
from std_srvs.srv import Trigger
from jugglebot_interfaces.msg import PlatformPoseCommand, RigidBodyPoses
from geometry_msgs.msg import Pose
import numpy as np
from scipy.spatial.transform import Rotation as R
import joblib
import os
from ament_index_python import get_package_share_directory
from sklearn.neighbors import KDTree
import time

ROT_SCALE = 100.0  # mm/rad; must match the tree build
K_NEIGHBORS = 8    # Number of nearest samples to use for feedforward

class PoseCorrectionNode(Node):
    def __init__(self):
        super().__init__('pose_correction_node')

        self.shutdown_flag = False
    
        self.declare_parameter('correction_method', 'feedforward')
        self.method = self.get_parameter('correction_method').get_parameter_value().string_value

        self.pose_subscription = self.create_subscription(
            PlatformPoseCommand, 'platform_pose_topic', self.pose_cmd_callback, 10)
        self.mocap_subscription = self.create_subscription(
            RigidBodyPoses, 'rigid_body_poses', self.mocap_callback, 10)
        self.corrected_pose_publisher = self.create_publisher(
            PlatformPoseCommand, 'platform_pose_corrected', 10)

        self.latest_cmd_pose = None
        self.latest_pose_publisher = None
        self.latest_measured_pose = None

        self.calc_times = []

        if self.method == "feedforward":
            self.load_feedforward_model()

        elif self.method == "none":
            self.get_logger().info("No pose correction applied. Platform follows commanded poses.")

        else:
            self.get_logger().error(f"Unknown correction method '{self.method}'. Defaulting to 'none'.")
            self.method = "none"

        self.add_on_set_parameters_callback(self.parameter_callback)

        self.service = self.create_service(Trigger, 'end_session', self.end_session)
        self.get_logger().info("PoseCorrectionNode initialized.")

    def load_feedforward_model(self):
        """Load pre-computed KD-tree and associated calibration data."""
        try:
            calibration_file_name = 'angled_origin_calibration_144.joblib'#'calibration_kdtree.joblib'
            package_path = get_package_share_directory('jugglebot')
            calib_path = os.path.join(package_path, 'resources', calibration_file_name)
            data = joblib.load(calib_path)
            self.kdtree = data['tree']
            self.features = data['features']
            self.errors = data['errors']
            self.get_logger().info(f"Feedforward model loaded: {calib_path} "
                                   f"({self.features.shape[0]} samples)")
        except Exception as e:
            self.get_logger().error(f"Failed to load feedforward calibration data: {e}")
            self.method = "none"

    #########################################################################################################

    def pose_cmd_callback(self, msg: PlatformPoseCommand):
        """Callback for incoming pose commands."""
        self.latest_cmd_pose = msg.pose_stamped.pose
        self.latest_pose_publisher = msg.publisher

        if self.method == "feedforward":
            self.apply_feedforward_correction(self.latest_cmd_pose)

        elif self.method == "none":
            # Pass-through: no correction
            corrected_pose_msg = PlatformPoseCommand()
            corrected_pose_msg.pose_stamped.header = msg.pose_stamped.header
            corrected_pose_msg.pose_stamped.pose = self.latest_cmd_pose
            corrected_pose_msg.publisher = self.latest_pose_publisher
            self.corrected_pose_publisher.publish(corrected_pose_msg)

    def mocap_callback(self, msg: RigidBodyPoses):
        """Callback for mocap measurements. Extracts Platform pose."""
        for body in msg.bodies:
            if body.name == "Platform":
                self.latest_measured_pose = body.pose.pose
                break

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'correction_method':
                new_method = param.value
                if new_method == self.method:
                    continue  # no change

                if new_method == "feedforward":
                    self.load_feedforward_model()
                    self.method = "feedforward"
                    self.get_logger().info("Switched correction method to feedforward.")
                elif new_method == "none":
                    self.method = "none"
                    self.get_logger().info("Switched correction method to none (pass-through).")
                else:
                    self.get_logger().warn(f"Unknown method '{new_method}'; keeping previous method '{self.method}'.")

        return SetParametersResult(successful=True)

    #########################################################################################################

    def apply_feedforward_correction(self, pose_cmd: Pose):
        """Apply feedforward correction based on nearest calibration samples."""
        if not hasattr(self, 'kdtree'):
            self.get_logger().warn("Feedforward correction requested but no KD-tree loaded.")
            return

        start_time = time.perf_counter_ns()

        # Build query feature
        cmd_xyz = np.array([pose_cmd.position.x, pose_cmd.position.y, pose_cmd.position.z])
        cmd_quat = np.array([pose_cmd.orientation.x, pose_cmd.orientation.y,
                             pose_cmd.orientation.z, pose_cmd.orientation.w])
        rotvec = R.from_quat(cmd_quat).as_rotvec() * ROT_SCALE
        query_feat = np.hstack([cmd_xyz, rotvec]).reshape(1, -1)

        # Query KD-tree
        dists, idxs = self.kdtree.query(query_feat, k=K_NEIGHBORS)

        # Compute weighted average error
        dists = dists.flatten()
        idxs = idxs.flatten()
        weights = np.exp(-dists**2 / (2 * (dists.max()**2 + 1e-6)))
        weights /= weights.sum()
        weighted_error = (weights[:,None] * self.errors[idxs]).sum(axis=0)

        # Apply correction
        corrected_pose = Pose()
        corrected_pose.position.x = pose_cmd.position.x + weighted_error[0]
        corrected_pose.position.y = pose_cmd.position.y + weighted_error[1]
        corrected_pose.position.z = pose_cmd.position.z + weighted_error[2]

        correction_quat = weighted_error[3:]
        # Compose rotation: corrected_q = correction_q * commanded_q
        corrected_quat = self.quat_mult(correction_quat, cmd_quat)
        corrected_pose.orientation.x = corrected_quat[0]
        corrected_pose.orientation.y = corrected_quat[1]
        corrected_pose.orientation.z = corrected_quat[2]
        corrected_pose.orientation.w = corrected_quat[3]

        elapsed_time_ms = (time.perf_counter_ns() - start_time) / 1e6
        self.calc_times.append(elapsed_time_ms)

        # Debugging
        # for n, i in enumerate(idxs):
        #     cp = self.features[i][:3]
        #     err = self.errors[i][:3]
        #     self.get_logger().info(
        #         f"  nn{n}: pos={cp.round(1)}  err={err.round(1)}  d={dists[n]:.1f}"
        #     )

        # Publish corrected pose
        corrected_pose_msg = PlatformPoseCommand()
        corrected_pose_msg.pose_stamped.header.frame_id = 'platform_start'
        corrected_pose_msg.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        corrected_pose_msg.pose_stamped.pose = corrected_pose
        corrected_pose_msg.publisher = self.latest_pose_publisher
        self.corrected_pose_publisher.publish(corrected_pose_msg)

    #########################################################################################################

    def quat_mult(self, q1, q2):
        """
        Hamilton product of two quaternions.
        Note: q1 and q2 are assumed to be in the form [x, y, z, w].
        Returns [x, y, z, w].
        """
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2

        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2
        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        
        return np.array([x, y, z, w])

    #########################################################################################################

    def end_session(self, request, response):
        """Shutdown service handler."""
        self.get_logger().info("End session requested. Shutting down...")
        response.success = True
        response.message = "Session ended. Shutting down node."
        self.shutdown_flag = True
        return response

    def on_shutdown(self):
        """Cleanup on shutdown."""
        self.get_logger().info("Shutting down PoseCorrectionNode...")
        if len(self.calc_times) > 0:
            avg = np.mean(self.calc_times)
            mx  = np.max(self.calc_times)
            std = np.std(self.calc_times)
            self.get_logger().info(f"Feedforward timing (ms): avg={avg:.2f}, max={mx:.2f}, std={std:.2f}")

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