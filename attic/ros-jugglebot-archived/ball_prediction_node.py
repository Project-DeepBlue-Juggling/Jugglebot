"""
Ball Prediction Node
--------------------------------------
Tracks balls that have been announced via /throw_announcements.

Pipeline:
1. Thrower publishes ThrowAnnouncement (position, velocity, target)
2. Node matches incoming mocap markers to announced balls
3. Kalman filter tracks matched balls
4. Landing prediction published on /balls

Any mocap data that doesn't match an announced ball is ignored.
"""

import rclpy
from rclpy.node import Node
from jugglebot_interfaces.msg import (
    MocapDataMulti, BallStateArray, BallState, 
    TargetArray, ThrowAnnouncement
)
from geometry_msgs.msg import Point, Vector3
from builtin_interfaces.msg import Time
import numpy as np
from typing import Optional, Dict, List, Tuple
from .kalman_filter import KalmanFilter


class TrackedBall:
    """A ball that has been announced and matched with mocap data."""
    
    def __init__(self, ball_id: int, source: str, target_id: str,
                 target_position: Point, landing_z: float,
                 initial_position: np.ndarray, initial_velocity: np.ndarray,
                 dt: float, logger=None):
        self.ball_id = ball_id
        self.source = source
        self.target_id = target_id
        self.target_position = target_position
        self.landing_z = landing_z
        self.logger = logger
        
        # Kalman filter for tracking
        self.kf = KalmanFilter(
            dt=dt,
            process_noise=5.0,
            measurement_noise=1.0,
            logger=logger
        )
        
        # Initialize with announced state
        initial_state = np.array([
            [initial_position[0]],
            [initial_position[1]],
            [initial_position[2]],
            [initial_velocity[0]],
            [initial_velocity[1]],
            [initial_velocity[2]]
        ])
        self.kf.initialize_with_state(initial_state)
        
        # Track frames without measurement
        self.frames_without_measurement = 0
        self.max_frames_without_measurement = 200  # ~1.0s at 200Hz - survive blind periods
        self.total_measurements = 0
        
        # Track if this ball was promoted from an announcement (vs pure mocap discovery)
        # Announcement-based balls get more trust during blind periods
        self.from_announcement = True
        
        # Current state cache
        self.position = initial_position.copy()
        self.velocity = initial_velocity.copy()
        
        # For adaptive matching - track how fast the ball is moving
        self.speed = np.linalg.norm(initial_velocity)
    
    def get_match_threshold(self, base_threshold: float, dt: float) -> float:
        """Get adaptive match threshold based on ball speed and uncertainty."""
        # Faster balls need larger thresholds due to timing uncertainty
        # Allow ~10% of distance traveled per frame as tolerance
        speed_based = self.speed * dt * 0.15
        
        # After losing tracking, increase threshold significantly to re-acquire
        if self.frames_without_measurement > 10:
            # Exponentially increase threshold during blind periods
            # This helps re-acquire the ball when it reappears
            blind_factor = 1.0 + min(self.frames_without_measurement * 0.05, 3.0)
            uncertainty_based = 50.0 * blind_factor
        else:
            uncertainty_based = 20.0 * (1 + self.frames_without_measurement * 0.5)
        
        return max(base_threshold, speed_based, min(uncertainty_based, 400.0))
    
    def predict_position(self, dt: float) -> np.ndarray:
        """Predict where the ball will be in dt seconds."""
        gravity = -9810.0  # mm/s^2
        pos = self.position.copy()
        pos += self.velocity * dt
        pos[2] += 0.5 * gravity * dt * dt
        return pos
    
    def update(self, measurement: Optional[np.ndarray]):
        """Update with new measurement (or None if no match this frame)."""
        self.kf.predict()
        
        if measurement is not None:
            self.kf.update(measurement)
            self.frames_without_measurement = 0
            self.total_measurements += 1
        else:
            self.frames_without_measurement += 1
        
        # Update cached state
        self.position = self.kf.get_current_position().ravel()
        self.velocity = self.kf.get_current_velocity().ravel()
        self.speed = np.linalg.norm(self.velocity)
    
    def is_below_landing_plane(self) -> bool:
        """Check if ball has passed below landing plane."""
        # Consider ball landed if it's at or below the landing plane
        # Also check if it's near the plane and moving downward
        at_or_below = self.position[2] <= self.landing_z
        near_and_descending = (
            self.position[2] < self.landing_z + 50 and 
            self.velocity[2] < -1000  # Moving downward significantly
        )
        return at_or_below or near_and_descending
    
    def is_lost(self) -> bool:
        """Check if we've lost tracking.
        
        Announcement-based balls get more leeway during blind periods
        since we know the ball exists and should reappear.
        """
        # Base threshold
        max_frames = self.max_frames_without_measurement
        
        # Announcement-based balls with some measurements get extra leeway
        if self.from_announcement and self.total_measurements >= 3:
            max_frames = int(max_frames * 1.5)  # 50% more tolerance
        
        return self.frames_without_measurement > max_frames
    
    def to_ball_state(self, node) -> Optional[BallState]:
        """Create BallState message with landing prediction."""
        prediction = self.kf.predict_landing_state(self.landing_z)
        if prediction is None:
            if self.logger:
                self.logger.warn(
                    f"Ball {self.ball_id}: predict_landing_state returned None. "
                    f"pos=({self.position[0]:.0f}, {self.position[1]:.0f}, {self.position[2]:.0f}), "
                    f"vel=({self.velocity[0]:.0f}, {self.velocity[1]:.0f}, {self.velocity[2]:.0f}), "
                    f"landing_z={self.landing_z:.0f}, "
                    f"total_meas={self.total_measurements}, frames_no_meas={self.frames_without_measurement}"
                )
            return None
        
        (landing_x, landing_y), (vx, vy, vz), time_to_land = prediction
        
        # Sanity check: time_to_land should be positive
        if time_to_land <= 0:
            if self.logger:
                self.logger.warn(
                    f"Ball {self.ball_id}: negative time_to_land={time_to_land:.3f}s. "
                    f"pos_z={self.position[2]:.0f}, vel_z={self.velocity[2]:.0f}, landing_z={self.landing_z:.0f}"
                )
            return None
        
        msg = BallState()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = "base"
        msg.id = self.ball_id
        msg.source = self.source
        
        msg.position = Point(
            x=float(self.position[0]),
            y=float(self.position[1]),
            z=float(self.position[2])
        )
        msg.velocity = Vector3(
            x=float(self.velocity[0]),
            y=float(self.velocity[1]),
            z=float(self.velocity[2])
        )
        
        msg.target_id = self.target_id
        msg.target_position = self.target_position
        
        msg.landing_position = Point(x=landing_x, y=landing_y, z=self.landing_z)
        msg.landing_velocity = Vector3(x=vx, y=vy, z=vz)
        
        current_time = node.get_clock().now().nanoseconds * 1e-9
        land_time = current_time + time_to_land
        msg.time_at_land = Time()
        msg.time_at_land.sec = int(land_time)
        msg.time_at_land.nanosec = int((land_time % 1) * 1e9)
        
        return msg


class PendingBall:
    """An announced ball waiting to be matched with mocap data."""
    
    def __init__(self, ball_id: int, announcement: ThrowAnnouncement,
                 throw_time: float, target_position: Point, landing_z: float):
        self.ball_id = ball_id
        self.source = announcement.thrower_name
        self.target_id = announcement.target_id
        self.target_position = target_position
        self.landing_z = landing_z
        self.throw_time = throw_time  # When the ball will actually be thrown
        self.created_time = throw_time  # For timeout calculation
        
        self.initial_position = np.array([
            announcement.initial_position.x,
            announcement.initial_position.y,
            announcement.initial_position.z
        ])
        self.initial_velocity = np.array([
            announcement.initial_velocity.x,
            announcement.initial_velocity.y,
            announcement.initial_velocity.z
        ])
        
        # Matching state - track match quality over recent frames
        self.match_count = 0  # Total matches
        self.recent_matches = 0  # Matches in recent window
        self.frames_since_throw = 0  # Frames processed since throw became active
        self.last_matched_position: Optional[np.ndarray] = None
        self.match_distances: List[float] = []  # Recent match distances for quality assessment
        self.match_history: List[Tuple[float, np.ndarray]] = []  # (timestamp, position) for velocity estimation
    
    def is_active(self, current_time: float) -> bool:
        """Check if the throw time has passed (ball should be in the air)."""
        return current_time >= self.throw_time
    
    def time_since_throw(self, current_time: float) -> float:
        """Get time elapsed since throw."""
        return max(0.0, current_time - self.throw_time)
    
    def predict_position(self, current_time: float) -> np.ndarray:
        """Predict current position based on projectile motion since throw."""
        dt = current_time - self.throw_time
        if dt < 0:
            # Throw hasn't happened yet - return initial position
            return self.initial_position.copy()
        
        gravity = -9810.0  # mm/s^2
        pos = self.initial_position + self.initial_velocity * dt
        pos[2] += 0.5 * gravity * dt * dt
        return pos
    
    def predict_velocity(self, current_time: float) -> np.ndarray:
        """Predict current velocity."""
        dt = current_time - self.throw_time
        if dt < 0:
            return self.initial_velocity.copy()
        
        vel = self.initial_velocity.copy()
        vel[2] += -9810.0 * dt
        return vel
    
    def get_match_threshold(self, base_threshold: float, current_time: float) -> float:
        """
        Get adaptive match threshold based on time since throw.
        
        Early in the flight, we're more uncertain about exact timing,
        so allow a larger search radius. As time passes and we still
        haven't matched, also increase threshold.
        """
        dt = self.time_since_throw(current_time)
        speed = np.linalg.norm(self.initial_velocity)
        
        # Timing uncertainty: allow ~50ms of timing error worth of distance
        timing_uncertainty = speed * 0.05
        
        # Increase threshold if we haven't matched yet but frames are passing
        no_match_factor = 1.0 + min(self.frames_since_throw * 0.02, 1.0)
        
        return max(base_threshold, timing_uncertainty) * no_match_factor
    
    def record_match(self, position: np.ndarray, distance: float, timestamp: float):
        """Record a successful match with timestamp for velocity estimation."""
        self.match_count += 1
        self.recent_matches += 1
        self.last_matched_position = position.copy()
        self.match_distances.append(distance)
        self.match_history.append((timestamp, position.copy()))
        
        # Keep only recent data
        if len(self.match_distances) > 20:
            self.match_distances.pop(0)
        if len(self.match_history) > 20:
            self.match_history.pop(0)
    
    def record_no_match(self):
        """Record a frame without a match."""
        # Decay recent matches
        self.recent_matches = max(0, self.recent_matches - 1)
    
    def estimate_velocity(self) -> Optional[np.ndarray]:
        """
        Estimate current velocity from recent match history.
        
        Uses linear regression on recent positions to estimate velocity,
        accounting for gravity on the z-axis.
        
        Returns:
            Estimated velocity [vx, vy, vz] or None if insufficient data.
        """
        if len(self.match_history) < 3:
            return None
        
        # Use last few matches for velocity estimation
        recent = self.match_history[-5:]  # Up to 5 recent matches
        if len(recent) < 2:
            return None
        
        # Time span check
        t0 = recent[0][0]
        t_last = recent[-1][0]
        dt_total = t_last - t0
        if dt_total < 0.005:  # Need at least 5ms of data
            return None
        
        # Use central differences or linear fit for velocity
        # For simplicity, use the two endpoints and correct for gravity
        pos0 = recent[0][1]
        pos1 = recent[-1][1]
        
        dt = t_last - t0
        vel = (pos1 - pos0) / dt
        
        # The z-component needs gravity correction: observed dz = vz*dt + 0.5*g*dt^2
        # So vz_avg = dz/dt - 0.5*g*dt
        # We want the velocity at the END (most recent), so vz = vz_initial + g*dt
        # vz_initial = dz/dt - 0.5*g*dt  =>  vz_final = dz/dt + 0.5*g*dt
        gravity = -9810.0
        vel[2] += 0.5 * gravity * dt  # This gives velocity at the end point
        
        return vel
    
    def is_ready_to_promote(self, min_matches: int) -> bool:
        """
        Check if we have enough confidence to promote to tracked ball.
        
        Since we trust the throw announcement, we can promote quickly:
        - Just need a few matches to confirm the ball exists
        - Or if enough time has passed after throw, promote anyway
        """
        # Promote quickly with just a few matches
        if self.match_count >= min(3, min_matches):
            return True
        
        # If we had early matches but lost them (ball went out of volume),
        # still promote if we had reasonable confidence
        if self.match_count >= 2 and self.frames_since_throw > 30:
            # Check if avg match distance was good
            if self.match_distances and np.mean(self.match_distances) < 100:
                return True
        
        return False


class BallPredictionNode(Node):
    def __init__(self):
        super().__init__('ball_prediction_node')

        # Subscribers
        self.mocap_sub = self.create_subscription(
            MocapDataMulti, '/mocap_data', self.mocap_callback, 10)
        self.targets_sub = self.create_subscription(
            TargetArray, '/targets', self.targets_callback, 10)
        self.announcement_sub = self.create_subscription(
            ThrowAnnouncement, '/throw_announcements', self.announcement_callback, 10)
        
        # Publisher
        self.balls_pub = self.create_publisher(BallStateArray, '/balls', 10)

        # Timer - match mocap rate for best tracking
        self.dt = 1.0 / 200.0  # 200 Hz to match mocap
        self.timer = self.create_timer(self.dt, self.timer_callback)

        # State
        self.pending_balls: List[PendingBall] = []  # Announced, waiting for mocap match
        self.tracked_balls: Dict[int, TrackedBall] = {}  # Matched and being tracked
        self.next_ball_id = 1
        
        # Matching parameters - more lenient for robust matching
        self.match_threshold_base = 100.0  # mm - base distance threshold (can be adaptive)
        self.pending_timeout = 5.0  # seconds before pending ball expires after throw
        self.default_landing_z = 735.0  # mm
        self.min_matches_to_confirm = 3  # Reduced: trust announcements, just need quick confirmation
        self.min_height_above_landing = 50.0  # mm - ball must be this high above landing_z
        
        # Diagnostic tracking
        self.last_diagnostic_time = 0.0
        self.diagnostic_interval = 1.0  # Log diagnostics every N seconds
        
        # Current targets
        self.targets: Dict[str, dict] = {}

        self.get_logger().info(
            f"BallPredictionNode initialized. "
            f"match_threshold={self.match_threshold_base}mm, "
            f"min_matches={self.min_matches_to_confirm}"
        )

    def announcement_callback(self, msg: ThrowAnnouncement):
        """Handle throw announcement - creates a pending ball."""
        current_time = self.get_clock().now().nanoseconds * 1e-9
        
        # Get target info
        target_position = Point()
        landing_z = self.default_landing_z
        
        if msg.target_id and msg.target_id in self.targets:
            target_data = self.targets[msg.target_id]
            target_position = target_data["position"]
            landing_z = target_data["z"]
        
        ball_id = self.next_ball_id
        self.next_ball_id += 1
        
        # Extract throw time from message (or use current time if not set)
        throw_time = msg.throw_time.sec + msg.throw_time.nanosec * 1e-9
        if throw_time < 1.0:  # Assume unset if near zero
            throw_time = current_time  # Immediate throw
        
        pending = PendingBall(
            ball_id=ball_id,
            announcement=msg,
            throw_time=throw_time,
            target_position=target_position,
            landing_z=landing_z
        )
        self.pending_balls.append(pending)
        
        delay = throw_time - current_time
        # self.get_logger().info(
        #     f"Ball {ball_id} announced by '{msg.thrower_name}' "
        #     f"pos=({msg.initial_position.x:.0f}, {msg.initial_position.y:.0f}, {msg.initial_position.z:.0f}) "
        #     f"target='{msg.target_id or 'default'}' "
        #     f"throw in {delay:.2f}s"
        # )

    def mocap_callback(self, msg: MocapDataMulti):
        """Match mocap markers to pending/tracked balls."""
        if len(msg.markers) == 0:
            return
        
        current_time = self.get_clock().now().nanoseconds * 1e-9
        
        # Build list of marker positions
        markers = []
        for data in msg.markers:
            pos = np.array([data.position.x, data.position.y, data.position.z])
            markers.append(pos)
        
        # Track which markers have been used
        used_markers = set()
        
        # First: Update tracked balls with closest marker
        for ball_id, ball in self.tracked_balls.items():
            best_idx = None
            best_dist = float('inf')
            
            # Predict where ball should be
            predicted_pos = ball.predict_position(self.dt)
            
            # Get adaptive threshold for this ball
            threshold = ball.get_match_threshold(self.match_threshold_base, self.dt)
            
            for idx, marker_pos in enumerate(markers):
                if idx in used_markers:
                    continue
                
                dist = np.linalg.norm(marker_pos - predicted_pos)
                if dist < threshold and dist < best_dist:
                    best_dist = dist
                    best_idx = idx
            
            if best_idx is not None:
                used_markers.add(best_idx)
                marker_pos = markers[best_idx]
                
                # Log re-acquisition after blind period
                if ball.frames_without_measurement > 20:
                    self.get_logger().info(
                        f"Ball {ball_id} RE-ACQUIRED after {ball.frames_without_measurement} blind frames! "
                        f"predicted=({predicted_pos[0]:.0f}, {predicted_pos[1]:.0f}, {predicted_pos[2]:.0f}), "
                        f"marker=({marker_pos[0]:.0f}, {marker_pos[1]:.0f}, {marker_pos[2]:.0f}), "
                        f"dist={best_dist:.1f}mm"
                    )
                
                ball.update(marker_pos.reshape(3, 1))
            else:
                # Log when ball enters blind period
                if ball.frames_without_measurement == 20:
                    self.get_logger().info(
                        f"Ball {ball_id} entering blind period at "
                        f"pos=({ball.position[0]:.0f}, {ball.position[1]:.0f}, {ball.position[2]:.0f}), "
                        f"vel_z={ball.velocity[2]:.0f}mm/s"
                    )
                ball.update(None)  # No measurement this frame
        
        # Second: Try to match remaining markers to pending balls
        ready_to_promote = []
        
        for pending in self.pending_balls:
            # Skip if throw hasn't happened yet
            if not pending.is_active(current_time):
                continue
            
            # Increment frame counter for this pending ball
            pending.frames_since_throw += 1
            
            predicted_pos = pending.predict_position(current_time)
            predicted_z = predicted_pos[2]
            
            # Get adaptive threshold
            threshold = pending.get_match_threshold(self.match_threshold_base, current_time)
            
            best_idx = None
            best_dist = float('inf')
            
            for idx, marker_pos in enumerate(markers):
                if idx in used_markers:
                    continue
                
                # Check marker is above landing plane
                if marker_pos[2] < pending.landing_z + self.min_height_above_landing:
                    continue
                
                dist = np.linalg.norm(marker_pos - predicted_pos)
                if dist < threshold and dist < best_dist:
                    best_dist = dist
                    best_idx = idx
            
            if best_idx is not None:
                # Found a match this frame
                pending.record_match(markers[best_idx], best_dist, current_time)
                used_markers.add(best_idx)
                
                # Check if ready to promote
                if pending.is_ready_to_promote(self.min_matches_to_confirm):
                    ready_to_promote.append(pending)
            else:
                pending.record_no_match()
                
                # Log diagnostic if this pending ball has been active a while with no matches
                if pending.frames_since_throw == 50 and pending.match_count == 0:
                    self.get_logger().warn(
                        f"Ball {pending.ball_id}: No matches after {pending.frames_since_throw} frames. "
                        f"Predicted pos=({predicted_pos[0]:.0f}, {predicted_pos[1]:.0f}, {predicted_pos[2]:.0f}), "
                        f"threshold={threshold:.0f}mm, markers available={len(markers) - len(used_markers)}"
                    )
        
        # Promote pending balls that have enough matches
        for pending in ready_to_promote:
            self.pending_balls.remove(pending)
            marker_pos = pending.last_matched_position
            
            # Try to estimate velocity from actual observations, fall back to predicted
            estimated_vel = pending.estimate_velocity()
            if estimated_vel is not None:
                current_vel = estimated_vel
                vel_source = "estimated"
            else:
                current_vel = pending.predict_velocity(current_time)
                vel_source = "predicted"
            
            tracked = TrackedBall(
                ball_id=pending.ball_id,
                source=pending.source,
                target_id=pending.target_id,
                target_position=pending.target_position,
                landing_z=pending.landing_z,
                initial_position=marker_pos,
                initial_velocity=current_vel,
                dt=self.dt,
                logger=self.get_logger()
            )
            self.tracked_balls[pending.ball_id] = tracked
            
            avg_dist = np.mean(pending.match_distances) if pending.match_distances else 0
            self.get_logger().info(
                f"Ball {pending.ball_id} confirmed: {pending.match_count} matches in "
                f"{pending.frames_since_throw} frames (avg dist={avg_dist:.1f}mm) at "
                f"({marker_pos[0]:.0f}, {marker_pos[1]:.0f}, {marker_pos[2]:.0f}), "
                f"vel=({current_vel[0]:.0f}, {current_vel[1]:.0f}, {current_vel[2]:.0f}) [{vel_source}]"
            )
            
            # Verify the tracked ball was initialized correctly
            self.get_logger().debug(
                f"Ball {pending.ball_id} KF state after init: "
                f"pos=({tracked.position[0]:.0f}, {tracked.position[1]:.0f}, {tracked.position[2]:.0f}), "
                f"vel=({tracked.velocity[0]:.0f}, {tracked.velocity[1]:.0f}, {tracked.velocity[2]:.0f})"
            )

    def targets_callback(self, msg: TargetArray):
        """Update known targets."""
        for target in msg.targets:
            self.targets[target.id] = {
                "position": target.position,
                "z": target.position.z
            }

    def timer_callback(self):
        """Publish ball states and cleanup."""
        current_time = self.get_clock().now().nanoseconds * 1e-9
        
        # Expire old pending balls
        expired = []
        for p in self.pending_balls:
            time_since_throw = p.time_since_throw(current_time)
            if time_since_throw > self.pending_timeout:
                expired.append(p)
                self.get_logger().warn(
                    f"Ball {p.ball_id} expired without being matched. "
                    f"Got {p.match_count} matches in {p.frames_since_throw} frames over {time_since_throw:.1f}s"
                )
        
        for p in expired:
            self.pending_balls.remove(p)
        
        # Remove balls that have landed or been lost
        to_remove = []
        for ball_id, ball in self.tracked_balls.items():
            if ball.is_below_landing_plane():
                self.get_logger().info(
                    f"Ball {ball_id} landed at ({ball.position[0]:.0f}, {ball.position[1]:.0f}, {ball.position[2]:.0f}) "
                    f"({ball.total_measurements} measurements, {ball.frames_without_measurement} frames without)"
                )
                to_remove.append(ball_id)
            elif ball.is_lost():
                self.get_logger().info(
                    f"Ball {ball_id} lost at ({ball.position[0]:.0f}, {ball.position[1]:.0f}, {ball.position[2]:.0f}) "
                    f"after {ball.frames_without_measurement} frames without measurement "
                    f"({ball.total_measurements} total measurements)"
                )
                to_remove.append(ball_id)
        
        for ball_id in to_remove:
            del self.tracked_balls[ball_id]
        
        # Publish all tracked balls
        if self.tracked_balls:
            msg = BallStateArray()
            for ball in self.tracked_balls.values():
                ball_state = ball.to_ball_state(self)
                if ball_state is not None:
                    msg.balls.append(ball_state)
            
            if msg.balls:
                self.balls_pub.publish(msg)
        
        # Periodic diagnostic logging
        if current_time - self.last_diagnostic_time > self.diagnostic_interval:
            self.last_diagnostic_time = current_time
            if self.pending_balls:
                for p in self.pending_balls:
                    if p.is_active(current_time):
                        self.get_logger().debug(
                            f"Pending ball {p.ball_id}: {p.match_count}/{p.frames_since_throw} matches"
                        )

    def destroy_node(self):
        self.get_logger().info("Shutting down BallPredictionNode.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BallPredictionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
