import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import Point
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
        self.declare_parameter('yaw_s_offset', 0.0)
        self.declare_parameter('launch_pitch_deg', 75.0)
        self.declare_parameter('g_mps2', 9.81)
        self.declare_parameter('throw_speed_max', 6.5)
        self.declare_parameter('test_target_xyz', [0.5, -0.5, -0.1])

        # ---------------- State & services ----------------
        self.shutdown_flag = False
        self.service = self.create_service(Trigger, 'end_session', self.end_session)
        self.send_ball_butler_command_client = self.create_client(
            SendBallButlerCommand, 'send_ball_butler_command'
        )

        # NEW: trigger service to aim at current param target
        self.aim_now_srv = self.create_service(Trigger, 'aim_now', self.handle_aim_now)

        # NEW: subscriber that triggers aim on publish
        self.aim_sub = self.create_subscription(Point, 'bb/aim_target', self.on_aim_target, 10)

        # Optional: log param changes (helps when tuning live)
        self.add_on_set_parameters_callback(self._on_params_set)

        # Example: one-shot on startup (can remove if undesired)
        tgt = self.get_parameter('test_target_xyz').get_parameter_value().double_array_value
        if len(tgt) >= 3:
            self.aim_and_send(float(tgt[0]), float(tgt[1]), float(tgt[2]))
        else:
            self.get_logger().warn("Parameter 'test_target_xyz' must be [x,y,z]. Skipping demo send.")

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
            self.aim_and_send(float(tgt[0]), float(tgt[1]), float(tgt[2]))
            response.success = True
            response.message = f"Aimed at test_target_xyz={list(tgt)}"
        except Exception as e:
            response.success = False
            response.message = f"Failed: {e}"
        return response

    # ---- NEW: Topic callback ----
    def on_aim_target(self, msg: Point):
        self.get_logger().info(f"Received /bb/aim_target → x={msg.x:.3f}, y={msg.y:.3f}, z={msg.z:.3f}")
        self.aim_and_send(float(msg.x), float(msg.y), float(msg.z))

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
        - Pitch is fixed (parameter).
        - No air drag; uniform gravity along -Z.
        """
        s = float(self.get_parameter('yaw_s_offset').value)
        pitch_deg = float(self.get_parameter('launch_pitch_deg').value)
        g = float(self.get_parameter('g_mps2').value)
        vmax = float(self.get_parameter('throw_speed_max').value)

        # ---- Yaw from provided geometry solver ----
        _, _, yaw = yaw_solve_thetas(x, y, s)
        if not math.isfinite(yaw):
            raise ValueError(f"No yaw solution for target (x={x:.3f}, y={y:.3f}) with s={s:.3f}")

        # ---- Fixed pitch ----
        pitch = math.radians(pitch_deg)

        # ---- Ballistic speed for fixed pitch to hit (R, z) ----
        # Heading-range R is distance in XY toward target, independent of yaw detail.
        R = math.hypot(x, y)

        # If R==0, we are directly above/below the origin. With fixed pitch (not vertical),
        # there is generally no solution unless z == 0 and we accept t=0. Guard it:
        if R <= 1e-9:
            raise ValueError("Target is collinear above/below launch point (R≈0). No solution with fixed non-vertical pitch.")

        # Standard projectile relation (no drag):
        # z = R*tan(pitch) - g*R^2 / (2*v^2 * cos^2(pitch))
        # => v^2 = g*R^2 / (2*cos^2(pitch) * (R*tan(pitch) - z))
        cos_p = math.cos(pitch)
        tan_p = math.tan(pitch)
        denom = (R * tan_p - z)

        if denom <= 0.0 or abs(cos_p) < 1e-9:
            # Physically unreachable for this pitch (target too high for given angle, or pitch ~90°)
            raise ValueError(
                f"Target unreachable at fixed pitch={pitch_deg:.1f}°. Need denom=R*tan(pitch)-z > 0. "
                f"Got R={R:.3f}, z={z:.3f}, denom={denom:.3f}."
            )

        v2 = g * R * R / (2.0 * cos_p * cos_p * denom)
        if v2 <= 0.0:
            raise ValueError("Computed non-positive v^2; check inputs.")
        v = math.sqrt(v2)

        # Time of flight
        t = R / (v * cos_p)

        # Clamp to vmax if necessary (note: clamping means we won't exactly hit the target)
        if v > vmax:
            self.get_logger().warn(
                f"Required speed {v:.3f} m/s exceeds max {vmax:.3f} m/s. Clamping to max (trajectory will undershoot)."
            )
            v = vmax
            t = R / (v * cos_p)

        return yaw, pitch, v, t

    def aim_and_send(self, x: float, y: float, z: float):
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
        req.throw_speed = float(v_mps)
        req.throw_time = float(2.0)

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
    def command_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Ball butler command sent successfully")
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
