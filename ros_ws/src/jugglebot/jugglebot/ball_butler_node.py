from __future__ import annotations  # Enables modern type hint syntax on Python 3.8+

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import Point, PoseStamped, Vector3
from jugglebot_interfaces.msg import (
    MocapDataMulti, BallButlerHeartbeat, BallStateArray, 
    RigidBodyPoses, ThrowAnnouncement, Target, TargetArray
)
from jugglebot_interfaces.srv import SendBallButlerCommand
import math
import numpy as np
import json
import os
from typing import Optional
from ament_index_python.packages import get_package_share_directory
from jugglebot.ball_butler_states import BallButlerStates


# =========================================================================================
#                              RLS Error Model for Throw Correction
# =========================================================================================

class RLSErrorModel:
    """
    Recursive Least Squares (RLS) model for learning throw error corrections.
    
    Models error_x and error_y as linear functions of target position:
        error_x = a0 + a1*x + a2*y + a3*z
        error_y = b0 + b1*x + b2*y + b3*z
    
    Where (x, y, z) is the target position in BB's local frame (mm).
    The correction applied is the negative of the predicted error.
    
    Features are normalized by dividing by 1000 (typical scale in mm) for
    numerical stability.
    """
    
    # Number of features: [1, x, y, z] (bias + 3 position terms)
    N_FEATURES = 4
    # Number of outputs: [error_x, error_y]
    N_OUTPUTS = 2
    # Feature normalization scale (mm)
    FEATURE_SCALE = 1000.0
    
    def __init__(self, 
                 forgetting_factor: float = 0.98,
                 initial_covariance: float = 1000.0,
                 covariance_trace_limit: float = 1e6,
                 logger=None):
        """
        Initialize the RLS error model.
        
        Args:
            forgetting_factor: Lambda in (0, 1]. Higher = slower adaptation but more stable.
                               Typical values: 0.95-0.99
            initial_covariance: Diagonal value for initial P matrix. Higher = faster initial
                                learning but potentially unstable.
            covariance_trace_limit: Maximum allowed trace(P) before rescaling to prevent blow-up.
            logger: Optional ROS logger for debug output.
        """
        self.lambda_ = forgetting_factor
        self.initial_covariance = initial_covariance
        self.covariance_trace_limit = covariance_trace_limit
        self.logger = logger
        
        # Initialize weights: theta is (N_FEATURES, N_OUTPUTS)
        # Each column predicts one output (error_x, error_y)
        self.theta = np.zeros((self.N_FEATURES, self.N_OUTPUTS))
        
        # Initialize covariance matrix P (shared for both outputs)
        self.P = initial_covariance * np.eye(self.N_FEATURES)
        
        # Track number of updates for cold-start handling
        self.n_updates = 0
        self.min_updates_for_linear = 5  # Use bias-only until we have this many samples
    
    def _build_features(self, x: float, y: float, z: float) -> np.ndarray:
        """
        Build the feature vector for a given target position.
        
        Features are normalized for numerical stability.
        
        Args:
            x, y, z: Target position in BB's local frame (mm)
            
        Returns:
            Feature vector [1, x/scale, y/scale, z/scale]
        """
        return np.array([
            1.0,  # Bias term
            x / self.FEATURE_SCALE,
            y / self.FEATURE_SCALE,
            z / self.FEATURE_SCALE
        ])
    
    def predict(self, x: float, y: float, z: float) -> tuple[float, float]:
        """
        Predict the error correction to apply for a given target position.
        
        Returns the negative of the predicted error (i.e., the correction to add
        to the target position).
        
        Args:
            x, y, z: Target position in BB's local frame (mm)
            
        Returns:
            (correction_x, correction_y): Corrections to add to target position (mm)
        """
        phi = self._build_features(x, y, z)
        
        # If we don't have enough data, only use the bias term
        if self.n_updates < self.min_updates_for_linear:
            # Only use bias (first element of theta)
            predicted_error = self.theta[0, :]
        else:
            # Use full linear model
            predicted_error = phi @ self.theta
        
        # Return negative of predicted error as the correction
        return -predicted_error[0], -predicted_error[1]
    
    def update(self, x: float, y: float, z: float, 
               error_x: float, error_y: float) -> dict:
        """
        Update the model with an observed error.
        
        Args:
            x, y, z: Target position where the throw was aimed (mm)
            error_x, error_y: Observed error = predicted_landing - target (mm)
            
        Returns:
            Dict with update statistics for logging
        """
        phi = self._build_features(x, y, z)  # (N_FEATURES,)
        y_obs = np.array([error_x, error_y])  # (N_OUTPUTS,)
        
        # Prediction before update
        y_pred = phi @ self.theta  # (N_OUTPUTS,)
        innovation = y_obs - y_pred  # (N_OUTPUTS,)
        
        # RLS update equations
        # Kalman gain: k = P @ phi / (lambda + phi' @ P @ phi)
        P_phi = self.P @ phi  # (N_FEATURES,)
        denom = self.lambda_ + phi @ P_phi  # scalar
        k = P_phi / denom  # (N_FEATURES,)
        
        # Update weights: theta += k @ innovation'
        # k is (N_FEATURES,), innovation is (N_OUTPUTS,)
        self.theta += np.outer(k, innovation)
        
        # Update covariance: P = (P - k @ phi' @ P) / lambda
        self.P = (self.P - np.outer(k, P_phi)) / self.lambda_
        
        # Covariance trace limiting to prevent blow-up
        trace_P = np.trace(self.P)
        if trace_P > self.covariance_trace_limit:
            scale_factor = self.covariance_trace_limit / trace_P
            self.P *= scale_factor
            if self.logger:
                self.logger.debug(f"RLS: Rescaled P by {scale_factor:.4f} (trace was {trace_P:.0f})")
        
        self.n_updates += 1
        
        return {
            'innovation': innovation,
            'trace_P': np.trace(self.P),
            'n_updates': self.n_updates
        }
    
    def get_state(self) -> dict:
        """
        Get the model state for persistence.
        
        Returns:
            Dict containing all state needed to restore the model.
        """
        return {
            'theta': self.theta.tolist(),
            'P': self.P.tolist(),
            'n_updates': self.n_updates,
            'lambda': self.lambda_,
            'initial_covariance': self.initial_covariance
        }
    
    def set_state(self, state: dict):
        """
        Restore model state from a saved state dict.
        
        Args:
            state: Dict from get_state()
        """
        if 'theta' in state:
            self.theta = np.array(state['theta'])
        if 'P' in state:
            self.P = np.array(state['P'])
        if 'n_updates' in state:
            self.n_updates = state['n_updates']
        if 'lambda' in state:
            self.lambda_ = state['lambda']
        if 'initial_covariance' in state:
            self.initial_covariance = state['initial_covariance']
    
    def reset(self):
        """Reset the model to its initial state."""
        self.theta = np.zeros((self.N_FEATURES, self.N_OUTPUTS))
        self.P = self.initial_covariance * np.eye(self.N_FEATURES)
        self.n_updates = 0
    
    def get_bias(self) -> tuple[float, float]:
        """
        Get the current bias terms (constant offset component).
        
        Returns:
            (bias_x, bias_y): The bias terms for x and y error prediction.
        """
        return -self.theta[0, 0], -self.theta[0, 1]
    
    def get_coefficients_summary(self) -> str:
        """
        Get a human-readable summary of the model coefficients.
        
        Returns:
            Formatted string describing the model.
        """
        # Coefficients are scaled, so we show them as "per 1000mm"
        lines = [
            f"RLS Error Model ({self.n_updates} updates):",
            f"  error_x = {self.theta[0,0]:.1f} + {self.theta[1,0]:.3f}*(x/1000) + {self.theta[2,0]:.3f}*(y/1000) + {self.theta[3,0]:.3f}*(z/1000)",
            f"  error_y = {self.theta[0,1]:.1f} + {self.theta[1,1]:.3f}*(x/1000) + {self.theta[2,1]:.3f}*(y/1000) + {self.theta[3,1]:.3f}*(z/1000)",
            f"  Bias correction: ({-self.theta[0,0]:.1f}, {-self.theta[0,1]:.1f}) mm"
        ]
        return "\n".join(lines)

def wrap_pi(angle: float) -> float:
    """Wrap angle to (-pi, pi]."""
    a = (angle + math.pi) % (2.0 * math.pi) - math.pi
    return a if a != -math.pi else math.pi

def yaw_solve_thetas(x: float, y: float, s: float):
    """
    Return (theta1, theta2, chosen_theta). If no solution, returns (nan, nan, nan).

    Using: -x*sin(theta) + y*cos(theta) = s
    Solutions: theta = atan2(-x, y) ± arccos(s / r), r = hypot(x, y).
    """
    r = math.hypot(x, y)
    if r < abs(s) or r == 0.0:
        return float("nan"), float("nan"), float("nan")
    base = math.atan2(-x, y)
    # clip to avoid domain errors on boundary
    s_over_r = max(-1.0, min(1.0, s / r))
    delta = math.acos(s_over_r)
    t1 = wrap_pi(base + delta)
    t2 = wrap_pi(base - delta)
    chosen = t1 if abs(t1) <= abs(t2) else t2
    return t1, t2, chosen


# =========================================================================================
#                           Rotation Axis Calibration Functions
# =========================================================================================

def fit_plane(points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """
    Fit a plane to a set of 3D points using SVD.
    Returns: (centroid, normal vector)
    """
    centroid = np.mean(points, axis=0)
    centered = points - centroid
    _, _, Vt = np.linalg.svd(centered)
    normal = Vt[-1]  # Last row of Vt (smallest singular value)
    return centroid, normal


def fit_circle_3d(points: np.ndarray) -> tuple[np.ndarray, float, np.ndarray, float]:
    """
    Fit a circle to 3D points that lie approximately on a plane.
    
    Returns:
        center: 3D center of the circle
        radius: radius of the circle
        normal: normal vector of the plane containing the circle
        residual: RMS error of fit (mm)
    """
    # First fit a plane to get the normal
    centroid, normal = fit_plane(points)
    
    # Create local 2D coordinate system on the plane
    if abs(normal[0]) < 0.9:
        u = np.cross(normal, [1, 0, 0])
    else:
        u = np.cross(normal, [0, 1, 0])
    u = u / np.linalg.norm(u)
    v = np.cross(normal, u)
    v = v / np.linalg.norm(v)
    
    # Project points to 2D
    centered = points - centroid
    points_2d = np.column_stack([
        np.dot(centered, u),
        np.dot(centered, v)
    ])
    
    # Fit circle in 2D using algebraic method
    x, y = points_2d[:, 0], points_2d[:, 1]
    A = np.column_stack([x, y, np.ones(len(x))])
    b = x**2 + y**2
    
    result, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
    
    a_2d = result[0] / 2
    b_2d = result[1] / 2
    r_squared = result[2] + a_2d**2 + b_2d**2
    radius = np.sqrt(max(r_squared, 0))
    
    # Convert 2D center back to 3D
    center_3d = centroid + a_2d * u + b_2d * v
    
    # Calculate residual (RMS distance from circle)
    distances_to_center = np.sqrt((x - a_2d)**2 + (y - b_2d)**2)
    residual = np.sqrt(np.mean((distances_to_center - radius)**2))
    
    return center_3d, radius, normal, residual


def find_rotation_axis(marker_trajectories: dict[int, np.ndarray], 
                       min_points: int = 50) -> tuple[np.ndarray, np.ndarray, dict]:
    """
    Find the rotation axis from multiple marker trajectories.
    
    Args:
        marker_trajectories: dict mapping marker index to Nx3 array of positions
        min_points: minimum number of points required for reliable fit
    
    Returns:
        axis_point: a point on the axis (mm)
        axis_direction: unit vector along the axis
        quality_metrics: dict with fit quality information for each marker
    
    Raises:
        ValueError: if calibration fails (insufficient data, poor fit, etc.)
    """
    centers = []
    normals = []
    weights = []
    quality_metrics = {}
    
    for marker_idx, points in marker_trajectories.items():
        if len(points) < min_points:
            quality_metrics[marker_idx] = {
                'status': 'skipped',
                'reason': f'insufficient points ({len(points)} < {min_points})'
            }
            continue
        
        center, radius, normal, residual = fit_circle_3d(points)
        
        # Ensure consistent normal direction (all pointing same way)
        if len(normals) > 0 and np.dot(normal, normals[0]) < 0:
            normal = -normal
        
        centers.append(center)
        normals.append(normal)
        
        # Weight by number of points and quality of fit
        weight = len(points) / (residual + 0.1)
        weights.append(weight)
        
        quality_metrics[marker_idx] = {
            'status': 'ok',
            'n_points': len(points),
            'radius_mm': radius,
            'center': center,
            'normal': normal,
            'fit_residual_mm': residual
        }
    
    if len(centers) < 2:
        raise ValueError(f"Calibration failed: only {len(centers)} markers had sufficient data")
    
    centers = np.array(centers)
    normals = np.array(normals)
    weights = np.array(weights)
    weights = weights / weights.sum()  # Normalize
    
    # The axis direction is the weighted average of circle normals
    axis_direction = np.sum(normals * weights[:, np.newaxis], axis=0)
    axis_direction = axis_direction / np.linalg.norm(axis_direction)
    
    # The axis passes through all circle centers - use weighted average
    axis_point = np.sum(centers * weights[:, np.newaxis], axis=0)
    
    # Calculate how well centers align with the axis (quality check)
    max_center_deviation = 0.0
    for i, marker_idx in enumerate([k for k in marker_trajectories.keys() 
                                     if quality_metrics.get(k, {}).get('status') == 'ok']):
        center = quality_metrics[marker_idx]['center']
        v = center - axis_point
        parallel = np.dot(v, axis_direction) * axis_direction
        perp = v - parallel
        dist_from_axis = np.linalg.norm(perp)
        quality_metrics[marker_idx]['distance_from_axis_mm'] = dist_from_axis
        max_center_deviation = max(max_center_deviation, dist_from_axis)
    
    # Warn if centers don't align well (suggests non-rigid body or bad data)
    if max_center_deviation > 2.0:  # More than 2mm deviation
        raise ValueError(
            f"Calibration warning: circle centers deviate up to {max_center_deviation:.2f}mm from axis. "
            "This may indicate non-rigid motion or poor marker visibility."
        )
    
    return axis_point, axis_direction, quality_metrics


def find_axis_plane_intersection(axis_point: np.ndarray, 
                                  axis_direction: np.ndarray,
                                  plane_z: float) -> Optional[np.ndarray]:
    """
    Find where the axis intersects a horizontal plane at given Z height.
    
    Returns the intersection point, or None if axis is parallel to plane.
    """
    if abs(axis_direction[2]) < 1e-10:
        return None  # Axis parallel to horizontal plane
    
    t = (plane_z - axis_point[2]) / axis_direction[2]
    intersection = axis_point + t * axis_direction
    
    return intersection


def calculate_yaw_offset(marker3_positions: np.ndarray,
                         yaw_readings_deg: list[float],
                         axis_point: np.ndarray) -> tuple[float, float]:
    """
    Calculate the yaw offset between Ball Butler's local frame and global frame.
    
    Uses the final ~1 second of data (when BB is stationary) to compute the offset
    between Marker 3's global angle and the reported yaw angle.
    
    Args:
        marker3_positions: Nx3 array of Marker 3 positions (mm)
        yaw_readings_deg: list of yaw readings from heartbeat (degrees)
        axis_point: the rotation axis position (mm)
    
    Returns:
        yaw_offset_rad: the offset angle (radians) - add this to convert local yaw to global
        std_dev_deg: standard deviation of the offset measurements (degrees)
    
    Raises:
        ValueError: if insufficient data for reliable calculation
    """
    if len(marker3_positions) < 50:
        raise ValueError(f"Insufficient Marker 3 data for yaw offset calculation: {len(marker3_positions)} points")
    
    if len(yaw_readings_deg) < 5:
        raise ValueError(f"Insufficient yaw readings for offset calculation: {len(yaw_readings_deg)} readings")
    
    # Use the last ~1 second of marker data (200 points at 200Hz)
    n_marker_samples = min(200, len(marker3_positions))
    recent_marker3 = marker3_positions[-n_marker_samples:]
    
    # Use the last ~1 second of yaw readings (10 points at 10Hz)
    n_yaw_samples = min(10, len(yaw_readings_deg))
    recent_yaw = yaw_readings_deg[-n_yaw_samples:]
    
    # Calculate Marker 3's angle in global frame for each position
    marker3_angles_rad = []
    for pos in recent_marker3:
        dx = pos[0] - axis_point[0]
        dy = pos[1] - axis_point[1]
        angle = math.atan2(dy, dx) 
        marker3_angles_rad.append(angle)
    
    # Average the marker angles
    # Use circular mean to handle wrap-around
    sin_sum = sum(math.sin(a) for a in marker3_angles_rad)
    cos_sum = sum(math.cos(a) for a in marker3_angles_rad)
    avg_marker3_angle_rad = math.atan2(sin_sum, cos_sum)
    
    # Average the yaw readings
    avg_yaw_deg = sum(recent_yaw) / len(recent_yaw)
    avg_yaw_rad = math.radians(avg_yaw_deg)
    
    # The offset is the difference: when BB reports yaw_deg, Marker 3 is at marker3_angle in global frame
    # So: global_angle = local_yaw + offset
    # Therefore: offset = global_angle - local_yaw
    yaw_offset_rad = wrap_pi(avg_marker3_angle_rad - avg_yaw_rad)
    
    # Calculate standard deviation of marker angles (as quality metric)
    angle_variance = sum((a - avg_marker3_angle_rad)**2 for a in marker3_angles_rad) / len(marker3_angles_rad)
    marker_std_deg = math.degrees(math.sqrt(angle_variance))
    
    # Also calculate yaw reading variance
    yaw_variance = sum((y - avg_yaw_deg)**2 for y in recent_yaw) / len(recent_yaw)
    yaw_std_deg = math.sqrt(yaw_variance)
    
    # Combined uncertainty (rough estimate)
    combined_std_deg = math.sqrt(marker_std_deg**2 + yaw_std_deg**2)
    
    return yaw_offset_rad, combined_std_deg


class BallButlerNode(Node):
    def __init__(self):
        super().__init__('ball_butler_node')

        # ---------------- Parameters ----------------
        self.declare_parameter('yaw_s_offset', -90.31)  # mm
        self.declare_parameter('pitch_max_min_deg', [85.0, 12.0]) # [max, min] degrees
        self.declare_parameter('max_throw_speed', 5.0)  # m/s - maximum allowed throw speed
        self.declare_parameter('max_throw_height', 0.5)  # m - maximum height above BB the ball can reach
        self.declare_parameter('g_mps2', 9.81)  # m/s^2
        self.declare_parameter('test_target_xyz', [500.0, -500.0, -100.0])  # mm
        self.declare_parameter('bb_mocap_position', [-707.1, -149.4, 1724.6])  # mm, approx. Updates after running calibration.
        self.declare_parameter('bb_yaw_offset_rad', -0.053)  # radians, approx. Updates after running calibration.
        self.declare_parameter('throw_speed_multiplier', 1.0)  # Multiplier for tuning
        self.declare_parameter('rls_forgetting_factor', 0.995)  # RLS forgetting factor (0.95-1.0)

        # ---------------- State & services ----------------
        self.shutdown_flag = False
        self.tracking_enabled = True  # Start by tracking targets
        self.service = self.create_service(Trigger, 'end_session', self.end_session)
        self.send_ball_butler_command_client = self.create_client(SendBallButlerCommand, 'bb/send_throw_command')

        # Trigger service to aim at current param target
        self.aim_now_srv = self.create_service(Trigger, 'bb/aim_now', self.handle_aim_now)

        # Trigger to throw
        self.throw_now_srv = self.create_service(Trigger, 'bb/throw_now', self.handle_throw_now)

        # Trigger service to toggle tracking on/off
        self.toggle_tracking_srv = self.create_service(Trigger, 'bb/toggle_tracking', self.handle_toggle_tracking)

        # Trigger service to reset RLS error model
        self.reset_rls_srv = self.create_service(Trigger, 'bb/reset_rls_model', self.handle_reset_rls_model)
        
        # Trigger service to print RLS model summary
        self.print_rls_srv = self.create_service(Trigger, 'bb/print_rls_model', self.handle_print_rls_model)

        # Subscriber that triggers aim on publish
        self.aim_sub = self.create_subscription(Point, 'bb/aim_target', self.on_aim_target, 10)

        # ---------------- Location Calibration ----------------
        # Subscribe to `bb/heartbeat` to monitor state
        self.ball_butler_heartbeat_sub = self.create_subscription(BallButlerHeartbeat, 'bb/heartbeat', self.heartbeat_callback, 20)
        self.current_state = None
        self.last_state = None

        # Calibration data storage
        self.calibration_data: dict[int, list[np.ndarray]] = {}  # marker index -> list of positions
        self.calibration_yaw_readings: list[float] = []  # yaw_deg readings during calibration
        self.is_calibrating = False
        self.is_calibrated = False
        self.calibration_yaw_offset_uncertainty_limit_deg = 5.0  # Max std dev for yaw offset to accept calibration

        # Subscribe to rigid body poses (for catching cone) and ball butler markers topics
        self.target_positions_sub = self.create_subscription(TargetArray, '/targets', self.target_poses_callback, 20)
        self.ball_butler_sub = self.create_subscription(MocapDataMulti, 'bb/markers', self.ball_butler_marker_callback, 20)

        # Optional: log param changes (helps when tuning live)
        self.add_on_set_parameters_callback(self._on_params_set)

        # Initialize variable to track last target position
        self.last_target = None  # type: Point | None  # (x, y, z) in BB's local frame (mm)
        self.last_target_global = None  # type: tuple[float, float, float] | None  # (x, y, z) in global frame (mm)
        self.last_target_id = ""  # Target ID for throw announcements (e.g., "catching_cone")

        self.seconds_to_throw_in = 2.0  # Time from command to throw

        # ---------------- Throw Learning / Offset Tracking ----------------
        # Subscribe to ball states to learn from throw errors
        self.balls_sub = self.create_subscription(
            BallStateArray, '/balls', self.balls_callback, 10
        )
        
        # Publisher for throw announcements
        self.throw_announcement_pub = self.create_publisher(
            ThrowAnnouncement, '/throw_announcements', 10
        )
        
        # State for tracking throws by ball ID
        self.tracked_ball_id: int | None = None  # ID of ball we're tracking (from /balls)
        self.throw_announce_time: float | None = None  # When the throw was announced
        self.expected_throw_time: float | None = None  # When the ball should be released
        self.throw_timeout_sec = 5.0  # Max time to wait for ball after announce
        self.latest_landing_prediction: tuple[float, float] | None = None  # (x, y) in mm
        self.latest_time_at_land: float | None = None  # Unix time when ball will land
        self.landing_capture_margin = 0.05  # seconds - capture final prediction this close to landing
        
        # Load persistent config from file
        self.config_file = self._get_config_file_path()
        
        # Initialize RLS error model for throw correction
        rls_lambda = float(self.get_parameter('rls_forgetting_factor').value)
        self.error_model = RLSErrorModel(
            forgetting_factor=rls_lambda,
            initial_covariance=1000.0,
            covariance_trace_limit=1e6,
            logger=self.get_logger()
        )
        self._load_config()

    # ---- param change logger ----
    def _on_params_set(self, params):
        for p in params:
            self.get_logger().info(f"Param set: {p.name} = {p.value}")
        from rcl_interfaces.msg import SetParametersResult
        return SetParametersResult(successful=True)

    # =========================================================================================
    #                          Throw Learning / Offset Persistence
    # =========================================================================================
    def _get_config_file_path(self) -> str:
        """Get the path to the persistent config file."""
        try:
            # Try to use package share directory (for installed packages)
            package_dir = get_package_share_directory('jugglebot')
            resources_dir = os.path.join(package_dir, 'resources')
        except Exception:
            # Fallback: use source directory
            package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            resources_dir = os.path.join(package_dir, 'resources')
        
        os.makedirs(resources_dir, exist_ok=True)
        return os.path.join(resources_dir, 'ball_butler_config.json')

    def _load_config(self):
        """Load the persistent config from file."""
        if os.path.exists(self.config_file):
            try:
                with open(self.config_file, 'r') as f:
                    data = json.load(f)
                    
                    # Load RLS error model state
                    rls_state = data.get('rls_error_model', {})
                    if rls_state:
                        self.error_model.set_state(rls_state)
                        bias_x, bias_y = self.error_model.get_bias()
                        self.get_logger().info(
                            f"Loaded RLS error model - {self.error_model.n_updates} updates, "
                            f"bias correction: ({bias_x:.1f}, {bias_y:.1f}) mm"
                        )
            except Exception as e:
                self.get_logger().warn(f"Failed to load config: {e}. Using defaults.")
                self.error_model.reset()
        else:
            self.get_logger().info("No config file found. Starting with defaults.")

    def _save_config(self):
        """Save the current config to file for persistence."""
        try:
            # Load existing config to preserve other sections
            existing_data = {}
            if os.path.exists(self.config_file):
                try:
                    with open(self.config_file, 'r') as f:
                        existing_data = json.load(f)
                except Exception:
                    pass
            
            # Save RLS error model state
            existing_data['rls_error_model'] = self.error_model.get_state()
            
            # Also save a human-readable summary
            bias_x, bias_y = self.error_model.get_bias()
            existing_data['rls_error_model']['_summary'] = {
                'n_updates': self.error_model.n_updates,
                'bias_correction_x_mm': bias_x,
                'bias_correction_y_mm': bias_y,
                'description': 'RLS linear regression model for throw error correction. '
                               'Predicts error as function of target position (x, y, z).'
            }

            # Ensure metadata section exists
            if '_metadata' not in existing_data:
                existing_data['_metadata'] = {
                    'description': 'Persistent configuration for Ball Butler node',
                    'units': 'All distances in millimeters unless otherwise specified'
                }
            
            with open(self.config_file, 'w') as f:
                json.dump(existing_data, f, indent=2)
            
            bias_x, bias_y = self.error_model.get_bias()
            self.get_logger().info(
                f"Saved RLS model - {self.error_model.n_updates} updates, "
                f"bias correction: ({bias_x:.1f}, {bias_y:.1f}) mm"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to save config: {e}")

    def balls_callback(self, msg: BallStateArray):
        """
        Callback for ball state updates.
        
        Tracks balls by ID:
        1. If we announced a throw but haven't captured a ball ID yet, look for new balls
        2. Once we have a ball ID, track that specific ball's landing prediction
        3. When ball is about to land (time_at_land is imminent) or disappears, apply learning
        """
        current_time = self.get_clock().now().nanoseconds * 1e-9
        
        # Only process if we're expecting a ball (throw was announced)
        if self.throw_announce_time is None:
            return
        
        # Check if we should time out waiting for a ball
        if self.tracked_ball_id is None:
            elapsed = current_time - self.throw_announce_time
            if elapsed > self.throw_timeout_sec:
                self.get_logger().warn(
                    f"Throw tracking timed out - no ball matched after {elapsed:.1f}s"
                )
                self._reset_throw_tracking()
                return
        
        # Check for timeout on tracked ball (in case it disappears without landing)
        if self.tracked_ball_id is not None:
            elapsed = current_time - self.throw_announce_time
            if elapsed > self.throw_timeout_sec + 3.0:  # Extra time after expected throw
                self.get_logger().warn(
                    f"Ball {self.tracked_ball_id} tracking timed out after {elapsed:.1f}s"
                )
                if self.latest_landing_prediction is not None:
                    self._apply_throw_learning()
                self._reset_throw_tracking()
                return
        
        # No target means we can't do learning (but we can still track)
        if self.last_target_global is None:
            self.get_logger().warn("No global target set - cannot apply throw learning", throttle_duration_sec=5.0)
            return
        
        # Use the actual target height for validation (from last_target_global)
        target_z_mm = self.last_target_global[2]  # Target z in mm
        height_tolerance_mm = 250.0  # 250mm tolerance for height validation
        
        # Build set of ball IDs in this message
        current_ball_ids = {ball.id for ball in msg.balls}
        
        # If we're tracking a ball and it's no longer in the message, it has landed
        if self.tracked_ball_id is not None and self.tracked_ball_id not in current_ball_ids:
            self.get_logger().info(
                f"Ball {self.tracked_ball_id} no longer tracked (landed or lost)"
            )
            if self.latest_landing_prediction is not None:
                self._apply_throw_learning()
            else:
                self.get_logger().warn("Ball disappeared but no landing prediction was captured")
            self._reset_throw_tracking()
            return
        
        # Process each ball in the message
        for ball in msg.balls:
            # Only process balls from ball_butler
            if ball.source != "ball_butler":
                continue
            
            # Extract time_at_land as unix timestamp
            time_at_land = ball.time_at_land.sec + ball.time_at_land.nanosec * 1e-9
            
            # Validate landing height matches our target height
            predicted_z_mm = ball.landing_position.z  # Already in mm
            if abs(predicted_z_mm - target_z_mm) > height_tolerance_mm:
                continue
            
            # If we don't have a tracked ball yet, try to capture this one
            if self.tracked_ball_id is None and self.throw_announce_time is not None:
                # Only capture balls that appeared after we announced
                # The ball should have a reasonable time_to_land
                time_to_land = time_at_land - current_time
                if time_to_land > 0 and time_to_land < 3.0:  # Reasonable flight time
                    self.tracked_ball_id = ball.id
                    self.get_logger().info(
                        f"Captured ball ID {ball.id} for throw learning "
                        f"(landing in {time_to_land:.2f}s)"
                    )
            
            # Update prediction for our tracked ball
            if ball.id == self.tracked_ball_id:
                predicted_x_mm = ball.landing_position.x  # Already in mm
                predicted_y_mm = ball.landing_position.y  # Already in mm
                self.latest_landing_prediction = (predicted_x_mm, predicted_y_mm)
                self.latest_time_at_land = time_at_land
                
                # Check if ball is about to land
                time_to_land = time_at_land - current_time
                if time_to_land <= self.landing_capture_margin:
                    # self._apply_throw_learning()
                    self._reset_throw_tracking()
                    return

    def _apply_throw_learning(self):
        """Apply the learning update using the final landing prediction."""
        if self.latest_landing_prediction is None or self.last_target_global is None:
            return
        
        if self.last_target is None:
            self.get_logger().warn("No local target position stored - cannot update RLS model")
            return
        
        predicted_x_mm, predicted_y_mm = self.latest_landing_prediction
        target_x_mm, target_y_mm, target_z_mm = self.last_target_global
        
        # Get the target in BB's local frame for the RLS model
        local_x = self.last_target.x
        local_y = self.last_target.y
        local_z = self.last_target.z
        
        # Calculate error in GLOBAL frame: where ball went vs where we wanted it
        error_x_global = predicted_x_mm - target_x_mm  # positive = ball went too far in +x
        error_y_global = predicted_y_mm - target_y_mm  # positive = ball went too far in +y
        error_magnitude_mm = math.hypot(error_x_global, error_y_global)
        
        # Transform error from global frame to BB's local frame
        # This is just a rotation (no translation needed for a direction/error vector)
        yaw_offset = self.get_parameter('bb_yaw_offset_rad').get_parameter_value().double_value
        cos_offset = math.cos(yaw_offset)
        sin_offset = math.sin(yaw_offset)
        
        # Rotate error vector: same rotation as global_to_bb_frame but without translation
        error_x_local = error_x_global * cos_offset - error_y_global * sin_offset
        error_y_local = error_x_global * sin_offset + error_y_global * cos_offset
        
        # Get old correction for logging
        old_corr_x, old_corr_y = self.error_model.predict(local_x, local_y, local_z)
        
        # Update RLS model with observed error (in local frame)
        update_stats = self.error_model.update(
            local_x, local_y, local_z,
            error_x_local, error_y_local
        )
        
        # Get new correction for logging
        new_corr_x, new_corr_y = self.error_model.predict(local_x, local_y, local_z)
        
        ball_id_str = f"ball {self.tracked_ball_id}" if self.tracked_ball_id else "unknown ball"
        self.get_logger().info(
            f"RLS learning from {ball_id_str}: "
            f"predicted=({predicted_x_mm:.0f}, {predicted_y_mm:.0f})mm, "
            f"target=({target_x_mm:.0f}, {target_y_mm:.0f})mm, "
            f"error_global=({error_x_global:.1f}, {error_y_global:.1f})mm, "
            f"error_local=({error_x_local:.1f}, {error_y_local:.1f})mm ({error_magnitude_mm:.1f}mm), "
            f"correction: ({old_corr_x:.1f}, {old_corr_y:.1f}) → "
            f"({new_corr_x:.1f}, {new_corr_y:.1f})mm "
            f"[update #{update_stats['n_updates']}]"
        )
        
        # Save the updated config
        self._save_config()

    def _reset_throw_tracking(self):
        """Reset all throw tracking state."""
        self.tracked_ball_id = None
        self.throw_announce_time = None
        self.expected_throw_time = None
        self.latest_landing_prediction = None
        self.latest_time_at_land = None

    # ---- Trigger service handler ----
    def handle_aim_now(self, request, response):
        tgt = self.get_parameter('test_target_xyz').get_parameter_value().double_array_value
        try:
            self.aim_and_throw(float(tgt[0]), float(tgt[1]), float(tgt[2]))
            response.success = True
            response.message = f"Aimed at test_target_xyz={list(tgt)}"
        except Exception as e:
            response.success = False
            response.message = f"Failed: {e}"
        return response

    # ---- Topic callback ----
    def on_aim_target(self, msg: Point):
        self.get_logger().info(f"Received /bb/aim_target → x={msg.x:.0f}, y={msg.y:.0f}, z={msg.z:.0f}mm")
        self.aim_and_throw(float(msg.x), float(msg.y), float(msg.z))

    # =========================================================================================
    #                              Coordinate Transformations
    # =========================================================================================
    def global_to_bb_frame(self, x_global: float, y_global: float, z_global: float) -> tuple[float, float, float]:
        """
        Transform a point from global (mocap) coordinates to Ball Butler's local frame.
        
        Args:
            x_global, y_global, z_global: target position in global frame (mm)
        
        Returns:
            x_bb, y_bb, z_bb: target position in BB's local frame (mm)
        """
        # Get calibration parameters (bb_mocap_position is already in mm)
        bb_pos_mm = self.get_parameter('bb_mocap_position').get_parameter_value().double_array_value
        yaw_offset = self.get_parameter('bb_yaw_offset_rad').get_parameter_value().double_value
        
        # Step 1: Translate to BB origin (all in mm)
        dx = x_global - bb_pos_mm[0]
        dy = y_global - bb_pos_mm[1]
        dz = z_global - bb_pos_mm[2]
        
        # Step 2: Rotate from global frame to BB's local frame
        # We need to rotate by -yaw_offset to go from global to local
        cos_offset = math.cos(yaw_offset)
        sin_offset = math.sin(yaw_offset)
        
        x_bb = dx * cos_offset - dy * sin_offset
        y_bb = dx * sin_offset + dy * cos_offset
        z_bb = dz
        
        return x_bb, y_bb, z_bb

    # =========================================================================================
    #                                 IK + Ballistics
    # =========================================================================================
    def compute_command_for_target(self, x: float, y: float, z: float):
        """
        Given target (x,y,z) in mm (relative to launch point, in BB's local frame),
        returns (yaw_angle_rad, pitch_angle_rad, throw_speed_mm_s, time_of_flight_s).

        The solver finds the trajectory that minimizes horizontal landing velocity
        while respecting constraints on:
        - Maximum throw speed
        - Maximum trajectory height (above launch point)
        - Pitch angle limits

        Assumptions:
        - Launch point is origin (0,0,0).
        - Yaw aims the throw plane toward the XY projection of the target.
        - No air drag; uniform gravity along -Z.
        """
        s = float(self.get_parameter('yaw_s_offset').value)  # mm
        v_max = float(self.get_parameter('max_throw_speed').value) * 1000.0  # mm/s
        h_max = float(self.get_parameter('max_throw_height').value) * 1000.0  # mm
        g = float(self.get_parameter('g_mps2').value) * 1000.0  # mm/s^2

        # ---- Yaw from provided geometry solver ----
        _, _, yaw = yaw_solve_thetas(x, y, s)
        if not math.isfinite(yaw):
            raise ValueError(f"No yaw solution for target (x={x:.0f}, y={y:.0f}mm) with s={s:.1f}mm")

        # ---- Find optimal pitch/speed to minimize horizontal landing velocity ----
        R = math.hypot(x, y)  # Horizontal range to target

        # Special case: target directly above/below
        if R <= 1e-9:
            if z >= 0:
                raise ValueError("Target is directly above launch point. Cannot reach with projectile motion.")
            # Straight down - minimal horizontal velocity (zero!)
            pitch = -math.pi / 2
            v = math.sqrt(2 * g * (-z))  # Speed needed for free-fall
            if v > v_max:
                raise ValueError(f"Target too far below: need {v/1000:.2f} m/s, max is {v_max/1000:.2f} m/s")
            t = math.sqrt(-2 * z / g)
            # No peak height for downward throw
            return yaw, pitch, v, t, 0.0, h_max

        # Get pitch limits
        pitch_limits_deg = self.get_parameter('pitch_max_min_deg').get_parameter_value().double_array_value
        pitch_max_rad = math.radians(pitch_limits_deg[0])
        pitch_min_rad = math.radians(pitch_limits_deg[1])

        # Search for optimal pitch angle
        # For a given pitch θ, the required speed is:
        #   v² = g*R² / (R*sin(2θ) - z*(1 + cos(2θ)))
        # Horizontal landing velocity is v*cos(θ)
        # We want to minimize v*cos(θ) subject to constraints
        
        best_pitch = None
        best_speed = None
        best_h_vel = float('inf')  # Horizontal velocity to minimize
        best_h_peak = None  # Track peak height of best solution
        
        # Search over pitch angles (higher pitch = loftier throw = lower horizontal velocity)
        # Use fine search with 0.5 degree steps
        n_steps = int((pitch_max_rad - pitch_min_rad) / math.radians(0.5)) + 1
        
        for i in range(n_steps):
            pitch = pitch_min_rad + i * (pitch_max_rad - pitch_min_rad) / max(n_steps - 1, 1)
            
            cos_p = math.cos(pitch)
            sin_p = math.sin(pitch)
            
            # Skip near-vertical pitches (numerical issues)
            if abs(cos_p) < 1e-6:
                continue
            
            # Denominator for speed calculation
            # D = R*sin(2θ) - z*(1 + cos(2θ))
            sin_2p = 2 * sin_p * cos_p
            cos_2p = cos_p * cos_p - sin_p * sin_p
            D = R * sin_2p - z * (1 + cos_2p)
            
            # Need D > 0 for a valid trajectory
            if D <= 0:
                continue
            
            # Required speed
            v_squared = g * R * R / D
            if v_squared <= 0:
                continue
            v = math.sqrt(v_squared)
            
            # Check speed constraint
            if v > v_max:
                continue
            
            # Check max height constraint
            # Max height above launch point: h_peak = (v*sin(θ))² / (2*g)
            v_vertical = v * sin_p
            h_peak = v_vertical * v_vertical / (2 * g) if v_vertical > 0 else 0.0
            
            if h_peak > h_max:
                continue
            
            # This is a valid solution - compute horizontal landing velocity
            h_vel = v * cos_p
            
            if h_vel < best_h_vel:
                best_h_vel = h_vel
                best_pitch = pitch
                best_speed = v
                best_h_peak = h_peak
        
        if best_pitch is None:
            raise ValueError(
                f"No valid trajectory found for target (R={R:.0f}mm, z={z:.0f}mm). "
                f"Constraints: v_max={v_max/1000:.1f}m/s, h_max={h_max/1000:.1f}m, "
                f"pitch=[{pitch_limits_deg[1]:.0f}°, {pitch_limits_deg[0]:.0f}°]"
            )
        
        # Compute time of flight
        t = R / (best_speed * math.cos(best_pitch))
        
        return yaw, best_pitch, best_speed, t, best_h_peak, h_max

    def track_target(self, x: float, y: float, z: float, target_id: str = ""):
        """Compute yaw/pitch for (x,y,z) and send a track command (aim without throwing).
        
        Note: x, y, z are in BB's local frame (mm).
        The RLS error model predicts the correction to apply based on target position,
        so that tracking mode shows where the ball would actually land.
        
        Args:
            x, y, z: Target position in BB's local frame (mm)
            target_id: Optional target ID to store for subsequent throw
        """
        # Apply RLS-predicted correction to match actual throw behavior
        corr_x, corr_y = self.error_model.predict(x, y, z)
        x_corrected = x + corr_x
        y_corrected = y + corr_y
        
        try:
            yaw_rad, pitch_rad, v_mm_s, tof_s, h_peak_mm, h_max_mm = self.compute_command_for_target(x_corrected, y_corrected, z)
        except ValueError as e:
            self.get_logger().error(f"Track target error: {e}", throttle_duration_sec=2.0)
            return

        # Build track request (aim only, no throw)
        req = SendBallButlerCommand.Request()
        req.yaw_angle_rad = float(yaw_rad)
        req.pitch_angle_rad = float(pitch_rad)
        req.throw_speed = float(0.0)  # Track only, no throw
        req.throw_time = float(0.0)

        # Set last target position (original, uncorrected - for learning comparison)
        # and ID for potential subsequent throw
        self.last_target = Point(x=x, y=y, z=z)
        self.last_target_id = target_id

        # Send track command
        if self.send_ball_butler_command_client.wait_for_service(timeout_sec=5.0):
            future = self.send_ball_butler_command_client.call_async(req)
            future.add_done_callback(self.command_response_callback)
        else:
            self.get_logger().warn("Ball butler command service not available")

    def aim_and_throw(self, x: float, y: float, z: float, target_id: str = ""):
        """Compute yaw/pitch/speed/time for (x,y,z) and send the command.
        
        Note: x, y, z are in BB's local frame (mm).
        The RLS error model predicts the correction to apply based on target position.
        
        Args:
            x, y, z: Target position in BB's local frame (mm)
            target_id: Optional target ID for throw announcement
        """
        # Apply RLS-predicted correction to compensate for systematic errors
        corr_x, corr_y = self.error_model.predict(x, y, z)
        x_corrected = x + corr_x
        y_corrected = y + corr_y
        
        try:
            yaw_rad, pitch_rad, v_mm_s, tof_s, h_peak_mm, h_max_mm = self.compute_command_for_target(x_corrected, y_corrected, z)
        except ValueError as e:
            self.get_logger().error(f"IK/ballistics error: {e}")
            return
        
        multiplier = float(self.get_parameter('throw_speed_multiplier').value)
        actual_throw_speed_m_s = (v_mm_s / 1000.0) * multiplier  # Convert mm/s to m/s
        
        # Compute ACTUAL peak height after multiplier is applied
        # h_peak scales with v^2, so multiply by multiplier^2
        actual_h_peak_mm = h_peak_mm * multiplier * multiplier
        
        # Log trajectory details including peak height for debugging
        h_vel_m_s = actual_throw_speed_m_s * math.cos(pitch_rad)
        self.get_logger().info(
            f"Throw: pitch={math.degrees(pitch_rad):.1f}deg, speed={actual_throw_speed_m_s:.2f}m/s, "
            f"h_vel={h_vel_m_s:.2f}m/s, h_peak={actual_h_peak_mm/1000:.2f}m (max={h_max_mm/1000:.2f}m), ToF={tof_s:.2f}s"
        )
        
        # Warn if multiplier causes peak height to exceed limit
        if actual_h_peak_mm > h_max_mm:
            self.get_logger().warn(
                f"WARNING: throw_speed_multiplier={multiplier:.2f} causes peak height "
                f"({actual_h_peak_mm/1000:.2f}m) to exceed limit ({h_max_mm/1000:.2f}m)!"
            )

        # Build request (throw_speed is in m/s for the hardware interface)
        req = SendBallButlerCommand.Request()
        req.yaw_angle_rad = float(yaw_rad)
        req.pitch_angle_rad = float(pitch_rad)
        req.throw_speed = float(actual_throw_speed_m_s)
        req.throw_time = float(self.seconds_to_throw_in)

        # Set last target position (original, uncorrected - for learning comparison)
        self.last_target = Point(x=x, y=y, z=z)

        # Start tracking this throw for learning (by announce time, ball ID will be captured later)
        current_time = self.get_clock().now().nanoseconds * 1e-9
        self.throw_announce_time = current_time
        self.expected_throw_time = current_time + self.seconds_to_throw_in
        self.tracked_ball_id = None  # Will be captured when ball appears in /balls
        self.latest_landing_prediction = None
        self.latest_time_at_land = None

        # Send
        if self.send_ball_butler_command_client.wait_for_service(timeout_sec=5.0):
            future = self.send_ball_butler_command_client.call_async(req)
            future.add_done_callback(self.command_response_callback)
            
            # Publish throw announcement so ball_prediction_node can match this throw
            self._publish_throw_announcement(yaw_rad, pitch_rad, actual_throw_speed_m_s, target_id)
        else:
            self.get_logger().warn("Ball butler command service not available")
            self._reset_throw_tracking()

    def _publish_throw_announcement(self, yaw_rad: float, pitch_rad: float, speed_mps: float, target_id: str = ""):
        """
        Publish a throw announcement so ball_prediction_node can match this throw.
        
        Args:
            yaw_rad: Yaw angle in radians
            pitch_rad: Pitch angle in radians
            speed_mps: Throw speed in m/s
            target_id: Optional target ID
        """
        # Get BB position in mm
        bb_pos_mm = self.get_parameter('bb_mocap_position').get_parameter_value().double_array_value
        yaw_offset = self.get_parameter('bb_yaw_offset_rad').get_parameter_value().double_value
        
        # Calculate initial velocity components in BB's local frame (m/s)
        # Then convert to global frame
        cos_pitch = math.cos(pitch_rad)
        sin_pitch = math.sin(pitch_rad)
        cos_yaw = math.cos(yaw_rad + yaw_offset)
        sin_yaw = math.sin(yaw_rad + yaw_offset)
        
        # Velocity in global frame (mm/s for consistency with other messages)
        vx = speed_mps * cos_pitch * cos_yaw * 1000.0
        vy = speed_mps * cos_pitch * sin_yaw * 1000.0
        vz = speed_mps * sin_pitch * 1000.0
        
        # Build and publish announcement
        announcement = ThrowAnnouncement()
        announcement.header.stamp = self.get_clock().now().to_msg()
        announcement.header.frame_id = "base"
        announcement.thrower_name = "ball_butler"
        announcement.initial_position = Point(
            x=bb_pos_mm[0],
            y=bb_pos_mm[1],
            z=bb_pos_mm[2]
        )
        announcement.initial_velocity = Vector3(x=vx, y=vy, z=vz)
        announcement.target_id = target_id
        announcement.throw_time = (self.get_clock().now() + rclpy.time.Duration(seconds=self.seconds_to_throw_in)).to_msg()
        
        self.throw_announcement_pub.publish(announcement)
        self.get_logger().info(f"Published throw announcement targeting '{target_id}'")

    def aim_at_global_point(self, x_global: float, y_global: float, z_global: float, throw: bool = True, target_id: str = ""):
        """
        Aim (and optionally throw) at a point specified in global (mocap) coordinates.
        
        This is the main entry point for aiming at arbitrary points in space after calibration.
        
        Args:
            x_global, y_global, z_global: target position in global frame (mm)
            throw: if True, also execute throw; if False, just aim
            target_id: Optional target ID for throw announcement
        """
        # Always store the global target (for throw learning when throw_now is called)
        self.last_target_global = (x_global, y_global, z_global)
        self.last_target_id = target_id
        
        # Transform from global to BB local frame
        x_bb, y_bb, z_bb = self.global_to_bb_frame(x_global, y_global, z_global)

        if throw:
            self.aim_and_throw(x_bb, y_bb, z_bb, target_id=target_id)
        else:
            self.track_target(x_bb, y_bb, z_bb, target_id=target_id)

    # =========================================================================================
    #                           Calibrating Ball Butler Location
    # =========================================================================================
    def ball_butler_marker_callback(self, msg: MocapDataMulti):
        """
        Callback for ball butler marker data.
        
        Accumulates marker positions during CALIBRATING state for axis detection.
        """
        # Only process if in CALIBRATING state
        if self.current_state != BallButlerStates.CALIBRATING:
            return
        
        # Initialize calibration data structure if just started calibrating
        if not self.is_calibrating:
            self.is_calibrating = True
            self.calibration_data = {i: [] for i in range(5)}  # 5 markers
            self.calibration_yaw_readings = []
            self.get_logger().info("Calibration started - collecting marker data...")
        
        # Store each marker's position (if valid)
        for i, marker in enumerate(msg.markers):
            if i >= 5:
                break  # Only expect 5 markers
            
            pos = marker.position
            
            # Skip invalid/missing data (QTM reports NaN for missing markers)
            if np.isnan(pos.x) or np.isnan(pos.y) or np.isnan(pos.z):
                continue
            
            # Store position as numpy array
            self.calibration_data[i].append(np.array([pos.x, pos.y, pos.z]))

    def _finalize_calibration(self):
        """
        Process accumulated calibration data to find rotation axis and yaw offset,
        then update the relevant parameters.
        """
        if not self.calibration_data:
            self.get_logger().error("Calibration failed: no marker data collected")
            return False
        
        # Convert lists to numpy arrays
        marker_trajectories = {}
        total_points = 0
        all_z_values = []
        
        for marker_idx, positions in self.calibration_data.items():
            if len(positions) > 0:
                marker_trajectories[marker_idx] = np.array(positions)
                total_points += len(positions)
                all_z_values.extend([p[2] for p in positions])
                self.get_logger().info(
                    f"Marker {marker_idx + 1}: {len(positions)} valid samples collected"
                )
        
        if total_points == 0:
            self.get_logger().error("Calibration failed: no valid marker positions recorded")
            return False
        
        # Calculate average Z height of all markers
        avg_z = np.mean(all_z_values)
        self.get_logger().info(f"Average marker Z height: {avg_z:.2f} mm")
        
        try:
            # Find the rotation axis
            axis_point, axis_direction, quality_metrics = find_rotation_axis(
                marker_trajectories, min_points=50
            )
            
            # Log quality metrics
            for marker_idx, metrics in quality_metrics.items():
                if metrics['status'] == 'ok':
                    self.get_logger().info(
                        f"Marker {marker_idx + 1}: radius={metrics['radius_mm']:.1f}mm, "
                        f"fit_residual={metrics['fit_residual_mm']:.3f}mm, "
                        f"axis_deviation={metrics.get('distance_from_axis_mm', 0):.3f}mm"
                    )
                else:
                    self.get_logger().warn(
                        f"Marker {marker_idx + 1}: {metrics['status']} - {metrics.get('reason', '')}"
                    )
            
            # Find intersection with average Z plane
            intersection = find_axis_plane_intersection(axis_point, axis_direction, avg_z)
            
            if intersection is None:
                self.get_logger().error(
                    "Calibration failed: rotation axis is horizontal (parallel to XY plane)"
                )
                return False
            
            # Calculate axis tilt from vertical
            tilt_deg = np.degrees(np.arccos(abs(axis_direction[2])))
            
            # Log axis results
            self.get_logger().info("=" * 50)
            self.get_logger().info("AXIS CALIBRATION RESULTS")
            self.get_logger().info("=" * 50)
            self.get_logger().info(
                f"Rotation axis intersection at Z={avg_z:.1f}mm: "
                f"X={intersection[0]:.2f}, Y={intersection[1]:.2f}, Z={intersection[2]:.2f} mm"
            )
            self.get_logger().info(
                f"Axis direction: ({axis_direction[0]:.4f}, {axis_direction[1]:.4f}, {axis_direction[2]:.4f})"
            )
            self.get_logger().info(f"Axis tilt from vertical: {tilt_deg:.2f}°")
            
            # ---- Calculate yaw offset using Marker 3 ----
            marker3_idx = 2  # "Ball Butler - 3" is index 2 (0-indexed)
            
            if marker3_idx not in marker_trajectories or len(marker_trajectories[marker3_idx]) < 50:
                self.get_logger().error(
                    "Calibration failed: insufficient Marker 3 data for yaw offset calculation"
                )
                return False
            
            if len(self.calibration_yaw_readings) < 5:
                self.get_logger().error(
                    f"Calibration failed: insufficient yaw readings ({len(self.calibration_yaw_readings)})"
                )
                return False
            
            yaw_offset_rad, offset_std_deg = calculate_yaw_offset(
                marker_trajectories[marker3_idx],
                self.calibration_yaw_readings,
                intersection  # Use the axis intersection point
            )

            # If the offset uncertainty is too high, fail calibration
            if offset_std_deg > self.calibration_yaw_offset_uncertainty_limit_deg:
                self.get_logger().error(
                    f"Calibration failed: yaw offset uncertainty too high (±{offset_std_deg:.2f}° > "
                    f"limit of ±{self.calibration_yaw_offset_uncertainty_limit_deg}°)"
                )
                return False
            
            self.get_logger().info("-" * 50)
            self.get_logger().info("YAW OFFSET CALIBRATION RESULTS")
            self.get_logger().info("-" * 50)
            self.get_logger().info(
                f"Yaw offset: {math.degrees(yaw_offset_rad):.2f}° ({yaw_offset_rad:.4f} rad)"
            )
            self.get_logger().info(f"Offset uncertainty: ±{offset_std_deg:.2f}°")
            
            # ---- Update parameters ----
            new_position = [float(intersection[0]), float(intersection[1]), float(intersection[2])]
            
            # Get old values for comparison
            old_position = self.get_parameter('bb_mocap_position').get_parameter_value().double_array_value
            old_yaw_offset = self.get_parameter('bb_yaw_offset_rad').get_parameter_value().double_value
            
            # Set new parameter values
            from rclpy.parameter import Parameter
            self.set_parameters([
                Parameter('bb_mocap_position', Parameter.Type.DOUBLE_ARRAY, new_position),
                Parameter('bb_yaw_offset_rad', Parameter.Type.DOUBLE, float(yaw_offset_rad))
            ])
            
            self.get_logger().info("=" * 50)
            self.get_logger().info("PARAMETER UPDATES")
            self.get_logger().info("=" * 50)
            self.get_logger().info(
                f"bb_mocap_position: {list(old_position)} → {new_position}"
            )
            self.get_logger().info(
                f"bb_yaw_offset_rad: {old_yaw_offset:.4f} → {yaw_offset_rad:.4f} "
                f"({math.degrees(old_yaw_offset):.2f}° → {math.degrees(yaw_offset_rad):.2f}°)"
            )
            
            # Calculate and log the position change
            delta = np.array(new_position) - np.array(old_position)
            self.get_logger().info(
                f"Position change: ΔX={delta[0]:.2f}, ΔY={delta[1]:.2f}, ΔZ={delta[2]:.2f} mm "
                f"(total: {np.linalg.norm(delta):.2f} mm)"
            )
            
            self.get_logger().info("=" * 50)
            self.get_logger().info("CALIBRATION COMPLETE")
            self.get_logger().info("=" * 50)
            return True
            
        except ValueError as e:
            self.get_logger().error(f"Calibration failed: {e}")
            return False

    # =========================================================================================
    #                             Service callbacks & shutdown
    # =========================================================================================
    def heartbeat_callback(self, msg: BallButlerHeartbeat):
        """Callback to handle Ball Butler heartbeat messages."""
        # Store previous state before updating
        previous_state = self.current_state
        
        # If calibrating, record the yaw reading
        if self.is_calibrating:
            self.calibration_yaw_readings.append(msg.yaw_deg)
        
        # If the state hasn't changed, do nothing else
        if msg.state == self.current_state:
            return
        
        # Update the current state
        self.current_state = msg.state
        
        # Check if we just exited CALIBRATING state
        if previous_state == BallButlerStates.CALIBRATING and self.is_calibrating:
            if self.current_state == BallButlerStates.ERROR:
                # Abort calibration on error
                self.get_logger().warn("Calibration aborted: Ball Butler entered ERROR state")
                self.calibration_data = {}
                self.calibration_yaw_readings = []
                self.is_calibrating = False
            else:
                # Calibration routine completed - process the data
                self.get_logger().info(
                    f"Calibration routine finished (new state: {self.current_state}). "
                    "Processing marker data..."
                )
                calib_success = self._finalize_calibration()
                self.calibration_data = {}
                self.calibration_yaw_readings = []
                self.is_calibrating = False
                self.is_calibrated = calib_success
    
    def handle_toggle_tracking(self, request, response):
        """Toggle tracking on/off. When tracking is disabled, Ball Butler moves to idle position."""
        self.tracking_enabled = not self.tracking_enabled
        
        if self.tracking_enabled:
            response.message = "Tracking enabled"
            self.get_logger().info("Tracking enabled")
        else:
            response.message = "Tracking disabled - moving to idle position"
            self.get_logger().info("Tracking disabled - moving to idle position (pitch=90°)")
            self._move_to_idle_position()
        
        response.success = True
        return response

    def handle_reset_rls_model(self, request, response):
        """Reset the RLS error model to its initial state."""
        old_n_updates = self.error_model.n_updates
        old_bias_x, old_bias_y = self.error_model.get_bias()
        
        self.error_model.reset()
        self._save_config()
        
        response.success = True
        response.message = (
            f"RLS model reset. Cleared {old_n_updates} updates. "
            f"Old bias was ({old_bias_x:.1f}, {old_bias_y:.1f}) mm"
        )
        self.get_logger().info(response.message)
        return response
    
    def handle_print_rls_model(self, request, response):
        """Print a summary of the current RLS error model."""
        summary = self.error_model.get_coefficients_summary()
        self.get_logger().info(f"\n{summary}")
        
        response.success = True
        response.message = summary
        return response

    def _move_to_idle_position(self):
        """Move Ball Butler to idle position (pitch=90°, yaw=20°)."""
        req = SendBallButlerCommand.Request()
        req.yaw_angle_rad = float(0.0)  # A message with yaw=0 and pitch=0 is interpreted as idle
        req.pitch_angle_rad = float(0.0)  
        req.throw_speed = float(0.0)  # No throw
        req.throw_time = float(0.0)

        if self.send_ball_butler_command_client.wait_for_service(timeout_sec=5.0):
            future = self.send_ball_butler_command_client.call_async(req)
            future.add_done_callback(self.command_response_callback)
        else:
            self.get_logger().warn("Ball butler command service not available")

    def handle_throw_now(self, request, response):
        if self.last_target is None:
            response.success = False
            response.message = "No previous target to throw at. Use 'aim_now' or publish to 'bb/aim_target' first."
            return response

        try:
            # Ensure last_target_global is set for learning (in case tracking set it)
            if self.last_target_global is not None:
                self.get_logger().info(
                    f"Throwing at global target: ({self.last_target_global[0]:.0f}, "
                    f"{self.last_target_global[1]:.0f}, {self.last_target_global[2]:.0f})mm"
                )
            self.aim_and_throw(self.last_target.x, self.last_target.y, self.last_target.z, target_id=self.last_target_id)
            response.success = True
            response.message = f"Throw command sent to last target at x={self.last_target.x:.0f}, y={self.last_target.y:.0f}, z={self.last_target.z:.0f}mm"
        except Exception as e:
            response.success = False
            response.message = f"Failed to throw: {e}"
        return response

    def target_poses_callback(self, msg: TargetArray):
        """
        Callback for rigid body poses. Extracts Catching_Cone pose and tracks it with Ball Butler.
        
        Uses the calibrated transform to convert global cone position to BB frame.
        """
        # Only proceed if calibrated and tracking is enabled
        if not self.is_calibrated or not self.tracking_enabled:
            self.get_logger().warn(f"Received rigid body poses but Ball Butler isn't ready. Calibrated: {self.is_calibrated}, Tracking enabled: {self.tracking_enabled}" , throttle_duration_sec=5.0)
            return

        # Find the Catching_Cone body in the message
        for body in msg.targets:
            if body.id == "catching_cone":
                # Extract cone position (already in mm)
                cone_pos_mm = np.array([
                    body.position.x,
                    body.position.y,
                    body.position.z
                ])

                # Use the calibrated transform and track the target (all in mm)
                self.aim_at_global_point(cone_pos_mm[0], cone_pos_mm[1], cone_pos_mm[2], throw=False, target_id="catching_cone")
                break
    
    def command_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                # self.get_logger().info("Ball butler command sent successfully", throttle_duration_sec=0.5)
                pass
            else:
                self.get_logger().error(f"Ball butler command failed: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def end_session(self, request, response):
        self.get_logger().info("End session requested. Shutting down...")
        response.success = True
        response.message = "Session ended. Shutting down node."
        self.shutdown_flag = True
        return response

    def on_shutdown(self):
        self.get_logger().info("Shutting down BallButlerNode...")


def main(args=None):
    rclpy.init(args=args)
    node = BallButlerNode()
    try:
        while rclpy.ok() and not node.shutdown_flag:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Shutting down.")
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
