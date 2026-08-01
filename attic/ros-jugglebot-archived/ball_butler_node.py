from __future__ import annotations  # Enables modern type hint syntax on Python 3.8+

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Vector3
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from jugglebot_interfaces.msg import (
    MocapDataMulti, BallButlerHeartbeat,
    RigidBodyPoses, ThrowAnnouncement, Target, TargetArray
)
from jugglebot_interfaces.srv import SendBallButlerCommand
import math
import numpy as np
from typing import Optional
from jugglebot.ball_butler_states import BallButlerStates


def wrap_pi(angle: float) -> float:
    """Wrap angle to (-pi, pi]."""
    a = (angle + math.pi) % (2.0 * math.pi) - math.pi
    return a if a != -math.pi else math.pi


def yaw_solve_thetas(x: float, y: float, s: float):
    """
    Return (theta1, theta2, chosen_theta). If no solution, returns (nan, nan, nan).

    Using:
        y = r*sin(theta) + s*cos(theta)
        x = r*cos(theta) - s*sin(theta)

    Solutions:
        r = sqrt(x^2 + y^2 - s^2)
        theta = atan2(y, x) - arcsin(s / sqrt(x^2 + y^2))
    """
    hyp = math.hypot(x, y)
    r_sq = x * x + y * y - s * s
    if r_sq < 0.0 or hyp == 0.0:
        return float("nan"), float("nan"), float("nan")
    r = math.sqrt(r_sq)
    base = math.atan2(y, x)
    # clip to avoid domain errors on boundary
    s_over_hyp = max(-1.0, min(1.0, s / hyp))
    delta = math.asin(s_over_hyp)
    t1 = wrap_pi(base - delta)
    t2 = wrap_pi(base - math.pi + delta)  # second solution from arcsin
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
    if max_center_deviation > 3.0:  # More than 3mm deviation
        raise ValueError(
            f"Circle centers deviate up to {max_center_deviation:.2f}mm from axis. "
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
        # Kinematic offsets (mm)
        self.declare_parameter('yaw_s_offset', -105.65)  # Ball centroid to Stage 2 origin (measured in Onshape)
        self.declare_parameter('pitch_d_offset', 41.0)   # Distance behind origin to pitch axis, along BB's centerline, as measured in Onshape
        self.declare_parameter('release_l_position', 150.0) # 105.5 mm offset from pitch axis to ball pos at bottom of stroke + 142.5 mm const. release point from Trajectory.h
        self.declare_parameter('pitch_z_offset', 17.5)  # mm, vertical offset of pitch axis from BB origin (yaw axis), as measured in Onshape

        self.declare_parameter('pitch_max_min_deg', [85.0, 12.0]) # [max, min] degrees
        self.declare_parameter('yaw_max_min_deg', [0.0, 185.0])   # [max, min] degrees
        self.declare_parameter('max_throw_speed', 5.0)  # m/s - maximum allowed throw speed
        self.declare_parameter('max_throw_height', 0.5)  # m - maximum height above BB the ball can reach
        self.declare_parameter('g_mps2', 9.81)  # m/s^2
        self.declare_parameter('test_target_xyz', [500.0, -500.0, -100.0])  # mm
        self.declare_parameter('bb_mocap_position', [-707.1, -149.4, 1724.6])  # mm, approx. Updates after running calibration.
        self.declare_parameter('bb_yaw_offset_rad', -0.053)  # radians, approx. Updates after running calibration.
        self.declare_parameter('throw_speed_multiplier', 1.0)  # Multiplier for tuning

        self.declare_parameter('landing_time_offset', -120.0)  # ms to subtract from predicted landing time for timing the throw command (accounts for release timing and ball flight time to target)

        # Configure which target to aim at
        self.declare_parameter('target_name', 'catching_cone')  # Name of target to aim at (must match target_id from TargetTrackerNode)
        self.possible_targets = ['catching_cone', 'platform']  # Valid target IDs (must match TargetTrackerNode mappings)

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

        # tf2 for transforming targets from arbitrary frames into 'world'
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribe to rigid body poses (for catching cone) and ball butler markers topics
        self.target_positions_sub = self.create_subscription(TargetArray, '/targets', self.target_poses_callback, 20)
        self.ball_butler_sub = self.create_subscription(MocapDataMulti, 'bb/markers', self.ball_butler_marker_callback, 20)

        # Optional: log param changes (helps when tuning live)
        self.add_on_set_parameters_callback(self._on_params_set)

        # Initialize variable to track last target position
        self.last_target = None  # type: Point | None  # (x, y, z) in BB's local frame (mm)
        self.last_target_global = None  # type: tuple[float, float, float] | None  # (x, y, z) in global frame (mm)
        self.last_target_id = ""  # Target ID for throw announcements (e.g., "catching_cone")

        self.seconds_to_throw_in = 1.0  # Time from command to throw (reduced for throw_now responsiveness)

        # Publisher for throw announcements
        self.throw_announcement_pub = self.create_publisher(
            ThrowAnnouncement, '/throw_announcements', 10
        )

    # ---- param change logger ----
    def _on_params_set(self, params):
        for p in params:
            self.get_logger().info(f"Param set: {p.name} = {p.value}")
        from rcl_interfaces.msg import SetParametersResult
        return SetParametersResult(successful=True)

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
    def _transform_to_world(self, target: Target) -> tuple[float, float, float] | None:
        """
        Transform a target's position into the 'world' frame using tf2.

        If the target is already in the 'world' frame (or has no frame_id),
        the position is returned as-is. Otherwise, tf2 is used to look up
        the transform.

        Returns:
            (x, y, z) in mm in the world frame, or None if the transform
            is unavailable.
        """
        frame = target.header.frame_id
        if frame == 'world' or frame == '':
            return target.position.x, target.position.y, target.position.z

        # Build a stamped point for tf2
        pt = PointStamped()
        pt.header = target.header
        pt.point = target.position

        try:
            transformed = self.tf_buffer.transform(
                pt, 'world', timeout=rclpy.duration.Duration(seconds=0.1)
            )
            return transformed.point.x, transformed.point.y, transformed.point.z
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(
                f"Cannot transform target '{target.id}' from '{frame}' to 'world': {e}",
                throttle_duration_sec=2.0
            )
            return None

    def apply_learned_correction(self, x_global, y_global, z_global):
        """
        Apply learned affine correction to target coordinates.

        This corrects for systematic kinematic errors without fixing root cause.
        Call this BEFORE global_to_bb_frame().
        """
        # Affine transformation parameters (from calibration)
        a11, a12 = 0.865538, 0.121274
        a21, a22 = -0.116464, 1.040522
        tx, ty = 45.030, -182.559

        # Apply correction
        x_corrected = a11 * x_global + a12 * y_global + tx
        y_corrected = a21 * x_global + a22 * y_global + ty
        z_corrected = z_global  # Z is unaffected

        return x_corrected, y_corrected, z_corrected

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
        # We need to rotate by yaw_offset to go from global to local
        cos_offset = math.cos(yaw_offset)
        sin_offset = math.sin(yaw_offset)

        x_bb = dx * cos_offset + dy * sin_offset
        y_bb = -dx * sin_offset + dy * cos_offset
        z_bb = dz

        return x_bb, y_bb, z_bb

    # =========================================================================================
    #                                 IK + Ballistics
    # =========================================================================================
    def compute_command_for_target(self, x: float, y: float, z: float):
        """
        Given target (x,y,z) in mm (relative to BB origin / yaw axis, in BB's local frame),
        returns (yaw_angle_rad, pitch_angle_rad, throw_speed_mm_s, time_of_flight_s,
                 peak_height_mm, max_height_mm).

        The solver accounts for the full forward kinematics of the serial chain:
            yaw axis → pitch axis (offset d) → linear axis → release point (offset s)

        The ball releases at position (in cylindrical coords about yaw axis):
            h       = l * sin(phi)
            r       = sqrt(s^2 + (l*cos(phi) - d)^2)
            alpha   = theta + atan2(s, l*cos(phi) - d)

        The target (x, y) satisfies:
            x = A*cos(theta) - s*sin(theta)
            y = A*sin(theta) + s*cos(theta)
        where A = l*cos(phi) - d + R, and R is the throw range (release → target).

        Since A = sqrt(x^2 + y^2 - s^2), the yaw solution is independent of phi, l, d.
        The throw range R = A - l*cos(phi) + d varies with pitch.
        The throw height z_throw = z - l*sin(phi) also varies with pitch.

        The solver minimises horizontal landing velocity subject to speed, height,
        and pitch constraints.
        """
        s = float(self.get_parameter('yaw_s_offset').value)              # mm
        d = float(self.get_parameter('pitch_d_offset').value)            # mm
        l = float(self.get_parameter('release_l_position').value)        # mm
        v_max = float(self.get_parameter('max_throw_speed').value) * 1000.0   # mm/s
        h_max = float(self.get_parameter('max_throw_height').value) * 1000.0  # mm
        g = float(self.get_parameter('g_mps2').value) * 1000.0                # mm/s^2

        # ---- Yaw from geometry solver (independent of pitch, l, d) ----
        _, _, yaw = yaw_solve_thetas(x, y, s)
        if not math.isfinite(yaw):
            raise ValueError(f"No yaw solution for target (x={x:.0f}, y={y:.0f}mm) with s={s:.1f}mm")

        # Check yaw limits
        yaw_limits_deg = self.get_parameter('yaw_max_min_deg').get_parameter_value().double_array_value
        yaw_min_rad = math.radians(yaw_limits_deg[0])
        yaw_max_rad = math.radians(yaw_limits_deg[1])
        if yaw < yaw_min_rad or yaw > yaw_max_rad:
            raise ValueError(
                f"Yaw angle {math.degrees(yaw):.1f}° out of limits "
                f"[{yaw_limits_deg[1]:.1f}°, {yaw_limits_deg[0]:.1f}°]"
            )

        # ---- Horizontal "arm-line" distance from yaw axis to target ----
        # A = sqrt(x^2 + y^2 - s^2), the distance along the throw direction
        A_sq = x * x + y * y - s * s
        if A_sq < 0:
            raise ValueError(
                f"Target (x={x:.0f}, y={y:.0f}mm) is inside the s-offset circle (s={s:.1f}mm)"
            )
        A = math.sqrt(A_sq)

        # ---- Special case: target directly above/below ----
        if A <= 1e-9:
            if z >= 0:
                raise ValueError("Target is directly above launch point. Cannot reach with projectile motion.")
            pitch = -math.pi / 2
            v = math.sqrt(2 * g * (-z))
            if v > v_max:
                raise ValueError(f"Target too far below: need {v / 1000:.2f} m/s, max is {v_max / 1000:.2f} m/s")
            t = math.sqrt(-2 * z / g)
            return yaw, pitch, v, t, 0.0, h_max

        # ---- Find optimal pitch/speed ----
        pitch_limits_deg = self.get_parameter('pitch_max_min_deg').get_parameter_value().double_array_value
        pitch_max_rad = math.radians(pitch_limits_deg[0])
        pitch_min_rad = math.radians(pitch_limits_deg[1])

        best_pitch = None
        best_speed = None
        best_h_vel = float('inf')
        best_h_peak = None

        n_steps = int((pitch_max_rad - pitch_min_rad) / math.radians(0.5)) + 1

        for i in range(n_steps):
            pitch = pitch_min_rad + i * (pitch_max_rad - pitch_min_rad) / max(n_steps - 1, 1)

            cos_p = math.cos(pitch)
            sin_p = math.sin(pitch)

            if abs(cos_p) < 1e-6:
                continue

            # ---- Launch offset corrections ----
            # Throw range: horizontal distance from release point to target
            R = A - l * cos_p + d

            # Must be positive (target must be beyond release point)
            if R <= 0:
                continue

            # Throw height: vertical distance from release point to target
            z_throw = z - l * sin_p

            # ---- Projectile equation ----
            # v^2 = g * R^2 / (R * sin(2phi) - z_throw * (1 + cos(2phi)))
            sin_2p = 2 * sin_p * cos_p
            cos_2p = cos_p * cos_p - sin_p * sin_p
            D = R * sin_2p - z_throw * (1 + cos_2p)

            if D <= 0:
                continue

            v_squared = g * R * R / D
            if v_squared <= 0:
                continue
            v = math.sqrt(v_squared)

            if v > v_max:
                continue

            # Max height above RELEASE POINT: h_peak = (v*sin(phi))^2 / (2*g)
            v_vertical = v * sin_p
            h_peak = v_vertical * v_vertical / (2 * g) if v_vertical > 0 else 0.0

            if h_peak > h_max:
                continue

            # Horizontal landing velocity (minimise this)
            h_vel = v * cos_p

            if h_vel < best_h_vel:
                best_h_vel = h_vel
                best_pitch = pitch
                best_speed = v
                best_h_peak = h_peak

        if best_pitch is None:
            raise ValueError(
                f"No valid trajectory found for target (A={A:.0f}mm, z={z:.0f}mm). "
                f"Constraints: v_max={v_max / 1000:.1f}m/s, h_max={h_max / 1000:.1f}m, "
                f"pitch=[{pitch_limits_deg[1]:.0f}°, {pitch_limits_deg[0]:.0f}°]"
            )

        # Time of flight (using the pitch-corrected range)
        R_final = A - l * math.cos(best_pitch) + d
        t = R_final / (best_speed * math.cos(best_pitch))

        return yaw, best_pitch, best_speed, t, best_h_peak, h_max

    def track_target(self, x: float, y: float, z: float, target_id: str = ""):
        """Compute yaw/pitch for (x,y,z) and send a track command (aim without throwing).

        Note: x, y, z are in BB's local frame (mm).

        Args:
            x, y, z: Target position in BB's local frame (mm)
            target_id: Optional target ID to store for subsequent throw
        """
        try:
            yaw_rad, pitch_rad, v_mm_s, tof_s, h_peak_mm, h_max_mm = self.compute_command_for_target(x, y, z)
        except ValueError as e:
            self.get_logger().error(f"Track target error: {e}", throttle_duration_sec=2.0)
            return

        # Build track request (aim only, no throw)
        req = SendBallButlerCommand.Request()
        req.yaw_angle_rad = float(yaw_rad)
        req.pitch_angle_rad = float(pitch_rad)
        req.throw_speed = float(0.0)  # Track only, no throw
        req.throw_time = float(0.0)

        # Set last target position and ID for potential subsequent throw
        self.last_target = Point(x=x, y=y, z=z)
        self.last_target_id = target_id

        # Send track command
        if self.send_ball_butler_command_client.wait_for_service(timeout_sec=5.0):
            future = self.send_ball_butler_command_client.call_async(req)
            future.add_done_callback(self.command_response_callback)
        else:
            self.get_logger().warn("Ball butler command service not available")

    def aim_and_throw(
        self,
        x: float, y: float, z: float,
        target_id: str = "",
        throw_delay_sec: Optional[float] = None,
        land_time: Optional[float] = None,
        ):
        """Compute yaw/pitch/speed/time for (x,y,z) and send the command.

        Timing can be specified in one of three ways (checked in priority order):
            1. land_time (absolute ROS time in seconds) — back-calculates the
            throw delay using the predicted time-of-flight.
            2. throw_delay_sec — explicit delay before release.
            3. Neither — uses self.seconds_to_throw_in as default delay.

        Args:
            x, y, z: Target position in BB's local frame (mm)
            target_id: Optional target ID for throw announcement
            throw_delay_sec: Time delay before throwing (seconds)
            land_time: Absolute ROS time when ball should arrive at target (seconds).
                    Overrides throw_delay_sec if both are provided.
        """
        if throw_delay_sec is None:
            throw_delay_sec = self.seconds_to_throw_in

        try:
            yaw_rad, pitch_rad, v_mm_s, tof_s, h_peak_mm, h_max_mm = self.compute_command_for_target(x, y, z)
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
            f"Throw: pitch={math.degrees(pitch_rad):.1f}deg, yaw={math.degrees(yaw_rad):.1f}deg, speed={actual_throw_speed_m_s:.2f}m/s, "
            f"h_vel={h_vel_m_s:.2f}m/s, h_peak={actual_h_peak_mm/1000:.2f}m (max={h_max_mm/1000:.2f}m), ToF={tof_s:.2f}s"
        )

        # Warn if multiplier causes peak height to exceed limit
        if actual_h_peak_mm > h_max_mm:
            self.get_logger().warn(
                f"WARNING: throw_speed_multiplier={multiplier:.2f} causes peak height "
                f"({actual_h_peak_mm/1000:.2f}m) to exceed limit ({h_max_mm/1000:.2f}m)!"
            )

        # Compute throw delay
        if land_time is not None:
            now = self.get_clock().now().nanoseconds / 1e9  # Current ROS time in seconds
            throw_delay_sec = land_time - now - tof_s

            if throw_delay_sec < 0:
                self.get_logger().warn(
                    f"Landing time is in the past! "
                    f"land_time={land_time:.3f}, now={now:.3f}, tof={tof_s:.3f}, "
                    f"computed delay={throw_delay_sec:.3f}s — clamping to 0"
                )
                throw_delay_sec = 0.0

            self.get_logger().info(
                f"Land-time mode: land_time={land_time:.3f}, tof={tof_s:.3f}s, "
                f"delay={throw_delay_sec:.3f}s"
            )
        elif throw_delay_sec is None:
            throw_delay_sec = self.seconds_to_throw_in

        # Validate against CAN limit (delay must fit in 16-bit milliseconds for wraparound logic)
        if throw_delay_sec > 65.535:
            self.get_logger().error(
                f"Throw delay {throw_delay_sec:.3f}s exceeds CAN limit of 65.535s"
            )
            return

        # Build request (throw_speed is in m/s for the hardware interface)
        req = SendBallButlerCommand.Request()
        req.yaw_angle_rad = float(yaw_rad)
        req.pitch_angle_rad = float(pitch_rad)
        req.throw_speed = float(actual_throw_speed_m_s)
        req.throw_time = float(throw_delay_sec + self.get_parameter('landing_time_offset').get_parameter_value().double_value / 1000.0)

        # Set last target position
        self.last_target = Point(x=x, y=y, z=z)

        # Send
        if self.send_ball_butler_command_client.wait_for_service(timeout_sec=5.0):
            future = self.send_ball_butler_command_client.call_async(req)
            future.add_done_callback(self.command_response_callback)

            # Publish throw announcement so ball_prediction_node can match this throw
            self._publish_throw_announcement(yaw_rad, pitch_rad, actual_throw_speed_m_s, target_id, throw_delay_sec, tof_s)
        else:
            self.get_logger().warn("Ball butler command service not available")

    def _publish_throw_announcement(self, yaw_rad: float, pitch_rad: float, speed_mps: float,
                                    target_id: str = "", throw_delay_sec: float = 2.0, tof_s: float = 0.0):
        """
        Publish a throw announcement so ball_prediction_node can match this throw.

        Args:
            yaw_rad: Yaw angle in radians
            pitch_rad: Pitch angle in radians
            speed_mps: Throw speed in m/s
            target_id: Optional target ID
            throw_delay_sec: Time delay before throwing
            tof_s: Time of flight in seconds
        """
        # Get BB position in mm
        bb_pos_mm = self.get_parameter('bb_mocap_position').get_parameter_value().double_array_value
        yaw_offset = self.get_parameter('bb_yaw_offset_rad').get_parameter_value().double_value
        g_mps2 = float(self.get_parameter('g_mps2').value)

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

        # Landing velocity in global frame (mm/s)
        # Horizontal components are unchanged during flight; only vertical changes due to gravity
        vx_land = vx  # same as initial
        vy_land = vy  # same as initial
        vz_land = vz - (g_mps2 * 1000.0 * tof_s)  # v_z_land = v_z_initial - g * tof

        # Build and publish announcement
        now = self.get_clock().now()
        announcement = ThrowAnnouncement()
        announcement.header.stamp = now.to_msg()
        announcement.header.frame_id = "base"
        announcement.thrower_name = "ball_butler"
        announcement.initial_position = Point(
            x=bb_pos_mm[0],
            y=bb_pos_mm[1],
            z=bb_pos_mm[2]
        )
        announcement.initial_velocity = Vector3(x=vx, y=vy, z=vz)
        announcement.target_id = target_id
        announcement.target_position = Point(
            x=self.last_target_global[0] if self.last_target_global else 0.0,
            y=self.last_target_global[1] if self.last_target_global else 0.0,
            z=self.last_target_global[2] if self.last_target_global else 0.0
        )
        announcement.throw_time = (now + rclpy.time.Duration(seconds=throw_delay_sec)).to_msg()
        announcement.predicted_tof_sec = tof_s

        # Landing fields
        announcement.landing_position = Point(
            x=self.last_target_global[0] if self.last_target_global else 0.0,
            y=self.last_target_global[1] if self.last_target_global else 0.0,
            z=self.last_target_global[2] if self.last_target_global else 0.0
        )
        announcement.landing_velocity = Vector3(x=vx_land, y=vy_land, z=vz_land)
        announcement.landing_time = (
            now
            + rclpy.time.Duration(seconds=throw_delay_sec)
            + rclpy.time.Duration(seconds=tof_s)
        ).to_msg()

        self.throw_announcement_pub.publish(announcement)

    def aim_at_global_point(self, x_global: float, y_global: float, z_global: float,
                            throw: bool = True, target_id: str = "", throw_delay_sec: Optional[float] = None):
        """
        Aim (and optionally throw) at a point specified in global (mocap) coordinates.

        This is the main entry point for aiming at arbitrary points in space after calibration.

        Args:
            x_global, y_global, z_global: target position in global frame (mm)
            throw: if True, also execute throw; if False, just aim
            target_id: Optional target ID for throw announcement
            throw_delay_sec: Time delay before throwing (for throw=True only)
        """
        # Always store the global target
        self.last_target_global = (x_global, y_global, z_global)
        self.last_target_id = target_id

        # Apply learned correction to global coordinates before transforming to BB frame
        x_corr, y_corr, z_corr = self.apply_learned_correction(x_global, y_global, z_global)

        # Transform from global to BB local frame
        x_bb, y_bb, z_bb = self.global_to_bb_frame(x_corr, y_corr, z_corr)

        if throw:
            self.aim_and_throw(x_bb, y_bb, z_bb, target_id=target_id, throw_delay_sec=throw_delay_sec)
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

        # Add the vertical offset to the pitch axis
        z_offset = float(self.get_parameter('pitch_z_offset').value)
        avg_z += z_offset

        self.get_logger().info(f"Average marker Z height (including {z_offset} mm pitch vertical offset): {avg_z:.2f} mm")

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

        # Find the desired body in the message
        target_name = self.get_parameter('target_name').get_parameter_value().string_value

        # Check that the target name is valid
        if not target_name:
            self.get_logger().error("Target name parameter is empty. Please set 'target_name' to the desired rigid body name.")
            return
        if target_name not in self.possible_targets:
            self.get_logger().error(f"Target name '{target_name}' is not in the list of possible targets: {self.possible_targets}",
            throttle_duration_sec=5.0)
            return

        for body in msg.targets:
            if body.id == target_name:
                # Transform the target position into the 'world' frame
                world_pos = self._transform_to_world(body)
                if world_pos is None:
                    break

                # Use the calibrated transform and track the target (all in mm)
                self.aim_at_global_point(*world_pos, throw=False, target_id=target_name)
                break

    def command_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
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
