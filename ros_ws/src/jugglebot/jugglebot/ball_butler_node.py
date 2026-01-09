from __future__ import annotations  # Enables modern type hint syntax on Python 3.8+

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import Point, PoseStamped
from jugglebot_interfaces.msg import MocapDataMulti, BallButlerHeartbeat
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


class BallButlerNode(Node):
    def __init__(self):
        super().__init__('ball_butler_node')

        # ---------------- Parameters ----------------
        self.declare_parameter('yaw_s_offset', -0.1305)  # m
        self.declare_parameter('pitch_max_min_deg', [85.0, 0.0]) # [max, min] degrees
        self.declare_parameter('throw_speed', 2.5)  # m/s
        self.declare_parameter('g_mps2', 9.81)
        self.declare_parameter('throw_speed_max', 6.5)
        self.declare_parameter('test_target_xyz', [0.5, -0.5, -0.1])
        self.declare_parameter('bb_mocap_position', [-850.0, -178.3, 1764.0])  # mm (SKETCHY. TO BE AUTOMATED). OG: [-843.3, -178.3, 1764.0
        self.declare_parameter('throw_speed_multiplier', 1.025)  # Multiplier for tuning

        # ---------------- State & services ----------------
        self.shutdown_flag = False
        self.service = self.create_service(Trigger, 'end_session', self.end_session)
        self.send_ball_butler_command_client = self.create_client(SendBallButlerCommand, 'bb/send_throw_command')

        # Trigger service to aim at current param target
        self.aim_now_srv = self.create_service(Trigger, 'bb/aim_now', self.handle_aim_now)

        # Trigger to throw
        self.throw_now_srv = self.create_service(Trigger, 'bb/throw_now', self.handle_throw_now)

        # Subscriber that triggers aim on publish
        self.aim_sub = self.create_subscription(Point, 'bb/aim_target', self.on_aim_target, 10)

        # ---------------- Location Calibration ----------------
        # Subscribe to `bb/heartbeat` to monitor state
        self.ball_butler_heartbeat_sub = self.create_subscription(BallButlerHeartbeat, 'bb/heartbeat', self.heartbeat_callback, 20)
        self.current_state = None
        self.last_state = None

        # Calibration data storage: dict mapping marker index (0-4) to list of positions
        self.calibration_data: dict[int, list[np.ndarray]] = {}
        self.is_calibrating = False

        # Subscribe to cone pose and ball butler markers topics
        # self.cone_sub = self.create_subscription(PoseStamped, 'catching_cone_pose_mocap', self.cone_callback, 20)
        self.ball_butler_sub = self.create_subscription(MocapDataMulti, 'bb/markers', self.ball_butler_marker_callback, 20)

        # Optional: log param changes (helps when tuning live)
        self.add_on_set_parameters_callback(self._on_params_set)

        # Initialize variable to track last target position
        self.last_target = None  # type: Point | None

        self.seconds_to_throw_in = 2.0  # Time from command to throw

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
        self.get_logger().info(f"Received /bb/aim_target → x={msg.x:.3f}, y={msg.y:.3f}, z={msg.z:.3f}")
        self.aim_and_throw(float(msg.x), float(msg.y), float(msg.z))

    # =========================================================================================
    #                                 IK + Ballistics
    # =========================================================================================
    def compute_command_for_target(self, x: float, y: float, z: float):
        """
        Given target (x,y,z) in meters (relative to launch point),
        returns (yaw_angle_rad, pitch_angle_rad, throw_speed_mps, time_of_flight_s).

        Assumptions:
        - Launch point is origin (0,0,0).
        - Yaw aims the throw plane toward the XY projection of the target using the provided 's' geometry.
        - Throw speed is fixed (parameter).
        - No air drag; uniform gravity along -Z.
        """
        s = float(self.get_parameter('yaw_s_offset').value)
        v = float(self.get_parameter('throw_speed').value)
        g = float(self.get_parameter('g_mps2').value)

        # ---- Yaw from provided geometry solver ----
        _, _, yaw = yaw_solve_thetas(x, y, s)
        if not math.isfinite(yaw):
            raise ValueError(f"No yaw solution for target (x={x:.3f}, y={y:.3f}) with s={s:.3f}")

        # ---- Ballistic pitch for fixed speed to hit (R, z) ----
        # Heading-range R is distance in XY toward target, independent of yaw detail.
        R = math.hypot(x, y)

        # If R==0, we are directly above/below the origin. Special case:
        if R <= 1e-9:
            if z >= 0:
                raise ValueError("Target is directly above launch point. Cannot reach with projectile motion.")
            # Straight down shot
            pitch = -math.pi / 2  # -90 degrees
            t = math.sqrt(-2 * z / g)  # Free fall time
            return yaw, pitch, v, t

        # Standard projectile relation (no drag):
        # z = R*tan(pitch) - g*R^2 / (2*v^2 * cos^2(pitch))
        # Rearranging: g*R^2 / (2*v^2) * sec^2(pitch) - R*tan(pitch) + z = 0
        # Let u = tan(pitch), then sec^2(pitch) = 1 + tan^2(pitch) = 1 + u^2
        # So: g*R^2 / (2*v^2) * (1 + u^2) - R*u + z = 0
        # Rearranging: (g*R^2 / (2*v^2)) * u^2 - R*u + (g*R^2 / (2*v^2) + z) = 0

        a = g * R * R / (2 * v * v)
        b = -R
        c = a + z

        # Solve quadratic equation: a*u^2 + b*u + c = 0
        discriminant = b * b - 4 * a * c
        
        if discriminant < 0:
            raise ValueError(
                f"Target unreachable with speed {v:.3f} m/s. "
                f"Need higher speed or different target. R={R:.3f}, z={z:.3f}"
            )

        u1 = (-b + math.sqrt(discriminant)) / (2 * a)
        u2 = (-b - math.sqrt(discriminant)) / (2 * a)
        
        pitch1 = math.atan(u1)
        pitch2 = math.atan(u2)
        
        # Get pitch limits in radians
        pitch_limits_deg = self.get_parameter('pitch_max_min_deg').get_parameter_value().double_array_value
        pitch_max_rad = math.radians(pitch_limits_deg[0])
        pitch_min_rad = math.radians(pitch_limits_deg[1])
        
        # Check which angles are within limits
        pitch1_valid = pitch_min_rad <= pitch1 <= pitch_max_rad
        pitch2_valid = pitch_min_rad <= pitch2 <= pitch_max_rad
        
        if pitch1_valid and pitch2_valid:
            # Both valid, choose the lower angle
            pitch = pitch1 if abs(pitch1) <= abs(pitch2) else pitch2
        elif pitch1_valid:
            pitch = pitch1
        elif pitch2_valid:
            pitch = pitch2
        else:
            raise ValueError(
            f"Both pitch solutions out of range. "
            f"pitch1={math.degrees(pitch1):.1f}°, pitch2={math.degrees(pitch2):.1f}°, "
            f"limits=[{pitch_limits_deg[1]:.1f}°, {pitch_limits_deg[0]:.1f}°]"
            )
        
        # Time of flight
        cos_p = math.cos(pitch)
        if abs(cos_p) < 1e-9:
            raise ValueError("Computed pitch is too close to vertical.")
        
        t = R / (v * cos_p)

        return yaw, pitch, v, t

    def aim_no_throw(self, x: float, y: float, z: float):
        """Compute yaw/pitch/speed/time for (x,y,z) and send the command."""
        try:
            yaw_rad, pitch_rad, v_mps, tof_s = self.compute_command_for_target(x, y, z)
        except ValueError as e:
            self.get_logger().error(f"IK/ballistics error: {e}")
            return

        # Build request
        req = SendBallButlerCommand.Request()
        req.yaw_angle_rad = float(yaw_rad)
        req.pitch_angle_rad = float(pitch_rad)
        req.throw_speed = float(0.0)  # No throw
        req.throw_time = float(3.0)

        # Set last target position
        self.last_target = Point(x=x, y=y, z=z)

        # Send
        if self.send_ball_butler_command_client.wait_for_service(timeout_sec=5.0):
            future = self.send_ball_butler_command_client.call_async(req)
            future.add_done_callback(self.command_response_callback)
            # self.get_logger().info(
            #     f"Sent command → yaw={yaw_rad:.3f} rad, pitch={pitch_rad:.3f} rad "
            #     f"({math.degrees(pitch_rad):.1f}°), v={v_mps:.3f} m/s, t={tof_s:.3f} s"
            # )
        else:
            self.get_logger().warn("Ball butler command service not available")

    def aim_and_throw(self, x: float, y: float, z: float):
        """Compute yaw/pitch/speed/time for (x,y,z) and send the command."""
        try:
            yaw_rad, pitch_rad, v_mps, tof_s = self.compute_command_for_target(x, y, z)
        except ValueError as e:
            self.get_logger().error(f"IK/ballistics error: {e}")
            return
        
        multiplier = float(self.get_parameter('throw_speed_multiplier').value)

        # Build request
        req = SendBallButlerCommand.Request()
        req.yaw_angle_rad = float(yaw_rad)
        req.pitch_angle_rad = float(pitch_rad)
        req.throw_speed = float(v_mps * multiplier)
        req.throw_time = float(self.seconds_to_throw_in)

        # Set last target position
        self.last_target = Point(x=x, y=y, z=z)

        # Send
        if self.send_ball_butler_command_client.wait_for_service(timeout_sec=5.0):
            future = self.send_ball_butler_command_client.call_async(req)
            future.add_done_callback(self.command_response_callback)
            self.get_logger().info(
                f"Sent command → yaw={yaw_rad:.3f} rad, pitch={pitch_rad:.3f} rad "
                f"({math.degrees(pitch_rad):.1f}°), v={v_mps:.3f} m/s, t={tof_s:.3f} s"
            )
        else:
            self.get_logger().warn("Ball butler command service not available")

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
        Process accumulated calibration data to find rotation axis and update bb_mocap_position.
        """
        if not self.calibration_data:
            self.get_logger().error("Calibration failed: no marker data collected")
            return
        
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
            return
        
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
                return
            
            # Calculate axis tilt from vertical
            tilt_deg = np.degrees(np.arccos(abs(axis_direction[2])))
            
            # Log results
            self.get_logger().info("=" * 50)
            self.get_logger().info("CALIBRATION COMPLETE")
            self.get_logger().info("=" * 50)
            self.get_logger().info(
                f"Rotation axis intersection at Z={avg_z:.1f}mm: "
                f"X={intersection[0]:.2f}, Y={intersection[1]:.2f}, Z={intersection[2]:.2f} mm"
            )
            self.get_logger().info(
                f"Axis direction: ({axis_direction[0]:.4f}, {axis_direction[1]:.4f}, {axis_direction[2]:.4f})"
            )
            self.get_logger().info(f"Axis tilt from vertical: {tilt_deg:.2f}°")
            
            # Update the bb_mocap_position parameter
            new_position = [float(intersection[0]), float(intersection[1]), float(intersection[2])]
            
            # Get old position for comparison
            old_position = self.get_parameter('bb_mocap_position').get_parameter_value().double_array_value
            
            # Set new parameter value
            from rclpy.parameter import Parameter
            self.set_parameters([Parameter('bb_mocap_position', Parameter.Type.DOUBLE_ARRAY, new_position)])
            
            self.get_logger().info(
                f"Updated bb_mocap_position: {list(old_position)} → {new_position}"
            )
            
            # Calculate and log the change
            delta = np.array(new_position) - np.array(old_position)
            self.get_logger().info(
                f"Position change: ΔX={delta[0]:.2f}, ΔY={delta[1]:.2f}, ΔZ={delta[2]:.2f} mm "
                f"(total: {np.linalg.norm(delta):.2f} mm)"
            )
            
        except ValueError as e:
            self.get_logger().error(f"Calibration failed: {e}")

    # =========================================================================================
    #                             Service callbacks & shutdown
    # =========================================================================================
    def heartbeat_callback(self, msg: BallButlerHeartbeat):
        """Callback to handle Ball Butler heartbeat messages."""
        # Store previous state before updating
        previous_state = self.current_state
        
        # If the state hasn't changed, do nothing
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
                self.is_calibrating = False
            else:
                # Calibration routine completed - process the data
                self.get_logger().info(
                    f"Calibration routine finished (new state: {self.current_state}). "
                    "Processing marker data..."
                )
                self._finalize_calibration()
                self.calibration_data = {}
                self.is_calibrating = False
    
    def handle_throw_now(self, request, response):
        if self.last_target is None:
            response.success = False
            response.message = "No previous target to throw at. Use 'aim_now' or publish to 'bb/aim_target' first."
            return response

        try:
            self.aim_and_throw(self.last_target.x, self.last_target.y, self.last_target.z)
            response.success = True
            response.message = f"Throw command sent to last target at x={self.last_target.x:.3f}, y={self.last_target.y:.3f}, z={self.last_target.z:.3f}"
        except Exception as e:
            response.success = False
            response.message = f"Failed to throw: {e}"
        return response

    def cone_callback(self, msg: PoseStamped):
        # Convert BB mocap position from mm to m
        bb_pos_mm = self.get_parameter('bb_mocap_position').get_parameter_value().double_array_value
        bb_pos_m = np.array(bb_pos_mm) / 1000.0  # Convert to meters

        # Extract cone position from PoseStamped message. Note that the incoming position data is in mm
        cone_pos_m = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]) / 1000.0

        # Compute target relative to ball butler
        target_rel = cone_pos_m - bb_pos_m

        # self.get_logger().info(
        #     f"Cone detected at (mocap): x={cone_pos_m[0]:.3f}, y={cone_pos_m[1]:.3f}, z={cone_pos_m[2]:.3f} | "
        #     f"Relative to BB: x={target_rel[0]:.3f}, y={target_rel[1]:.3f}, z={target_rel[2]:.3f}"
        # )

        # Aim and send command
        self.aim_no_throw(target_rel[0], target_rel[1], target_rel[2])
    
    def command_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Ball butler command sent successfully", throttle_duration_sec=0.5)
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