import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import Point, PoseStamped
from jugglebot_interfaces.srv import SendBallButlerCommand
import math
import numpy as np


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


class BallButlerNode(Node):
    def __init__(self):
        super().__init__('ball_butler_node')

        # ---------------- Parameters ----------------
        self.declare_parameter('yaw_s_offset', -0.1305)  # m
        self.declare_parameter('pitch_max_min_deg', [85.0, 0.0]) # [max, min] degrees
        self.declare_parameter('throw_speed', 3.5)  # m/s
        self.declare_parameter('g_mps2', 9.81)
        self.declare_parameter('throw_speed_max', 6.5)
        self.declare_parameter('test_target_xyz', [0.5, -0.5, -0.1])
        self.declare_parameter('bb_mocap_position', [-850.0, -178.3, 1764.0])  # mm (SKETCHY. TO BE AUTOMATED). OG: [-843.3, -178.3, 1764.0
        self.declare_parameter('throw_speed_multiplier', 1.025)  # Multiplier for tuning

        # ---------------- State & services ----------------
        self.shutdown_flag = False
        self.service = self.create_service(Trigger, 'end_session', self.end_session)
        self.send_ball_butler_command_client = self.create_client(
            SendBallButlerCommand, 'send_ball_butler_command'
        )

        # NEW: trigger service to aim at current param target
        self.aim_now_srv = self.create_service(Trigger, 'aim_now', self.handle_aim_now)

        # Trigger to throw
        self.throw_now_srv = self.create_service(Trigger, 'throw_now', self.handle_throw_now)

        # NEW: subscriber that triggers aim on publish
        self.aim_sub = self.create_subscription(Point, 'bb/aim_target', self.on_aim_target, 10)

        # Subscribe to cone pose
        self.cone_sub = self.create_subscription(PoseStamped, 'catching_cone_pose_mocap', self.cone_callback, 20)

        # Optional: log param changes (helps when tuning live)
        self.add_on_set_parameters_callback(self._on_params_set)

        # Initialize variable to track last target position
        self.last_target = None  # type: Point | None

        self.seconds_to_throw_in = 2.0  # Time from command to throw

    # ---- NEW: param change logger (optional) ----
    def _on_params_set(self, params):
        for p in params:
            self.get_logger().info(f"Param set: {p.name} = {p.value}")
        from rcl_interfaces.msg import SetParametersResult
        return SetParametersResult(successful=True)

    # ---- NEW: Trigger service handler ----
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

    # ---- NEW: Topic callback ----
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
    #                             Service callbacks & shutdown
    # =========================================================================================
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
