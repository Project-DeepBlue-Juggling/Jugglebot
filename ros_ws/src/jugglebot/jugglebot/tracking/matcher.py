"""Ball tracking orchestrator — pure Python.

BallTracker is the main class that owns all Ball objects and drives:
  - Throw announcement handling (Ball Butler path)
  - Parabolic motion detection (human throw path)
  - Frame-to-frame marker association for tracked balls
  - Ball lifecycle transitions (status + cleanup)

No ROS2 dependency. The ROS2 node (ball_tracker_node.py) feeds data in
and reads results out.
"""
from __future__ import annotations

from typing import Dict, List, Optional, Tuple

import numpy as np

from .ball import Ball, BallStatus, TrackingConfidence, TERMINAL_STATUSES
from .ballistics import GRAVITY_MMPS2, predict_landing_state
from .kalman import KalmanFilter


# --- Parabolic detection helpers ---

class _MarkerTrack:
    """Internal: buffered unlabelled marker for parabolic detection."""
    __slots__ = ("positions", "timestamps")

    def __init__(self):
        self.positions: list[np.ndarray] = []
        self.timestamps: list[float] = []

    def add(self, pos: np.ndarray, t: float):
        self.positions.append(pos.copy())
        self.timestamps.append(t)
        # Keep last 6 frames max
        if len(self.positions) > 6:
            self.positions.pop(0)
            self.timestamps.pop(0)

    @property
    def last_position(self) -> np.ndarray:
        return self.positions[-1]

    @property
    def last_timestamp(self) -> float:
        return self.timestamps[-1]

    def __len__(self):
        return len(self.positions)


class BallTracker:
    """Owns all Ball objects and processes mocap frames.

    Usage:
        tracker = BallTracker(dt=0.005, landing_z=734.3)

        # Ball Butler path:
        tracker.handle_announcement(position, velocity, throw_time, ...)

        # Each mocap frame:
        balls = tracker.process_frame(marker_positions, current_time)
    """

    def __init__(
        self,
        dt: float = 0.005,
        landing_z: float = 734.3,
        match_threshold_base_mm: float = 100.0,
        gate_radius_mm: float = 50.0,
        parabolic_accel_threshold_mmps2: float = 3000.0,
        parabolic_min_frames: int = 3,
        missed_frames_to_lose: int = 10,
        caught_grace_s: float = 0.200,
        terminal_retention_s: float = 2.0,
        max_frames_without_measurement: int = 200,
        process_noise: float = 5.0,
        measurement_noise: float = 1.0,
        min_height_above_landing_mm: float = 50.0,
    ):
        self.dt = dt
        self.landing_z = landing_z
        self.match_threshold_base_mm = match_threshold_base_mm
        self.gate_radius_mm = gate_radius_mm
        self.parabolic_accel_threshold = parabolic_accel_threshold_mmps2
        self.parabolic_min_frames = parabolic_min_frames
        self.missed_frames_to_lose = missed_frames_to_lose
        self.caught_grace_s = caught_grace_s
        self.terminal_retention_s = terminal_retention_s
        self.max_frames_without_measurement = max_frames_without_measurement
        self.process_noise = process_noise
        self.measurement_noise = measurement_noise
        self.min_height_above_landing_mm = min_height_above_landing_mm

        # Active balls keyed by id
        self._balls: Dict[int, Ball] = {}
        # Kalman filters keyed by ball id (only for CONFIRMED balls)
        self._filters: Dict[int, KalmanFilter] = {}
        # Next monotonic ball id
        self._next_id = 1

        # Unlabelled marker buffer for parabolic detection
        # Keyed by a transient track id (not ball id)
        self._marker_tracks: Dict[int, _MarkerTrack] = {}
        self._next_track_id = 1

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def handle_announcement(
        self,
        initial_position: np.ndarray,
        initial_velocity: np.ndarray,
        throw_time: float,
        source: str = "ball_butler",
        destination: str = "",
        landing_position: Optional[np.ndarray] = None,
        landing_velocity: Optional[np.ndarray] = None,
        landing_time: Optional[float] = None,
    ) -> int:
        """Register an announced throw (Ball Butler path).

        Args:
            initial_position: [x,y,z] mm at throw time.
            initial_velocity: [vx,vy,vz] mm/s at throw time.
            throw_time: Absolute time when ball will be released.
            source: Thrower identifier.
            destination: Which robot should catch this ball.
            landing_position/velocity/time: Pre-computed from announcement.
                If None, computed from ballistics.

        Returns:
            Ball id.
        """
        ball_id = self._next_id
        self._next_id += 1

        ball = Ball(
            id=ball_id,
            status=BallStatus.TO_BE_THROWN,
            tracking=TrackingConfidence.ANNOUNCED,
            source=source,
            destination=destination,
            position=np.array(initial_position, dtype=np.float64),
            velocity=np.array(initial_velocity, dtype=np.float64),
            throw_time=throw_time,
        )

        # Pre-compute or use provided landing prediction
        if landing_position is not None and landing_velocity is not None and landing_time is not None:
            ball.landing_position = np.array(landing_position, dtype=np.float64)
            ball.landing_velocity = np.array(landing_velocity, dtype=np.float64)
            ball.landing_time = landing_time
        else:
            self._update_landing_prediction(ball)

        # Create Kalman filter initialized from announcement
        kf = KalmanFilter(
            dt=self.dt,
            process_noise=self.process_noise,
            measurement_noise=self.measurement_noise,
        )
        kf.initialize_from_state(
            np.array(initial_position, dtype=np.float64),
            np.array(initial_velocity, dtype=np.float64),
            position_cov=500.0,   # High uncertainty from announcement
            velocity_cov=500.0,
        )
        self._filters[ball_id] = kf
        self._balls[ball_id] = ball
        return ball_id

    def process_frame(
        self,
        marker_positions: List[np.ndarray],
        current_time: float,
    ) -> List[Ball]:
        """Process one mocap frame. Call at mocap rate (~200 Hz).

        Args:
            marker_positions: List of [x,y,z] arrays (mm) for unlabelled markers.
            current_time: Current time in seconds.

        Returns:
            List of all active (non-terminal or recently-terminal) balls.
        """
        used_markers = set()

        # 1. Time-based transitions (TO_BE_THROWN → IN_FLIGHT)
        self._check_throw_times(current_time)

        # 2. Predict all Kalman filters forward
        for ball_id, kf in self._filters.items():
            ball = self._balls[ball_id]
            if ball.status == BallStatus.IN_FLIGHT:
                kf.predict()

        # 3. Associate markers to existing CONFIRMED in-flight balls
        used_markers = self._associate_confirmed_balls(marker_positions, current_time)

        # 4. Try to match remaining markers to ANNOUNCED balls
        used_markers = self._match_announced_balls(marker_positions, current_time, used_markers)

        # 5. Detect parabolic motion in remaining markers (human throw path)
        self._detect_parabolic(marker_positions, current_time, used_markers)

        # 6. Lifecycle transitions (CAUGHT, DROPPED, UNKNOWN)
        self._check_lifecycle(current_time)

        # 7. Cleanup terminal balls past retention
        self._cleanup_terminal(current_time)

        return list(self._balls.values())

    @property
    def active_balls(self) -> List[Ball]:
        """All balls not yet cleaned up."""
        return list(self._balls.values())

    def get_ball(self, ball_id: int) -> Optional[Ball]:
        return self._balls.get(ball_id)

    # ------------------------------------------------------------------
    # Internal: Time-based transitions
    # ------------------------------------------------------------------

    def _check_throw_times(self, current_time: float):
        for ball in self._balls.values():
            if ball.status == BallStatus.TO_BE_THROWN and current_time >= ball.throw_time:
                ball.status = BallStatus.IN_FLIGHT
                ball.timestamp = current_time

    # ------------------------------------------------------------------
    # Internal: Frame-to-frame association (CONFIRMED balls)
    # ------------------------------------------------------------------

    def _associate_confirmed_balls(
        self,
        markers: List[np.ndarray],
        current_time: float,
    ) -> set:
        """Match markers to existing CONFIRMED in-flight balls using gated nearest-neighbour."""
        used = set()

        for ball_id, ball in self._balls.items():
            if ball.status != BallStatus.IN_FLIGHT or ball.tracking != TrackingConfidence.CONFIRMED:
                continue

            kf = self._filters.get(ball_id)
            if kf is None:
                continue

            predicted_pos = kf.position
            best_idx = None
            best_dist = float("inf")

            for idx, mpos in enumerate(markers):
                if idx in used:
                    continue
                dist = np.linalg.norm(mpos - predicted_pos)
                if dist < self.gate_radius_mm and dist < best_dist:
                    best_dist = dist
                    best_idx = idx

            if best_idx is not None:
                kf.update(markers[best_idx])
                used.add(best_idx)
                ball.frames_tracked += 1
                ball.frames_without_measurement = 0
                ball.last_seen = current_time
            else:
                ball.frames_without_measurement += 1

            # Update ball state from filter
            ball.position = kf.position
            ball.velocity = kf.velocity
            ball.timestamp = current_time

            # Refresh landing prediction
            self._update_landing_prediction(ball, from_filter=True)

        return used

    # ------------------------------------------------------------------
    # Internal: Announced ball matching
    # ------------------------------------------------------------------

    def _match_announced_balls(
        self,
        markers: List[np.ndarray],
        current_time: float,
        used: set,
    ) -> set:
        """Try to match markers to IN_FLIGHT/ANNOUNCED balls."""
        for ball_id, ball in self._balls.items():
            if ball.status != BallStatus.IN_FLIGHT:
                continue
            if ball.tracking != TrackingConfidence.ANNOUNCED:
                continue

            kf = self._filters.get(ball_id)
            if kf is None:
                continue

            # Predicted position from KF (which was initialized from announcement)
            predicted_pos = kf.position

            # Adaptive threshold: base + time scaling
            time_since_throw = max(0.0, current_time - ball.throw_time)
            speed = np.linalg.norm(ball.velocity)
            threshold = self.match_threshold_base_mm + speed * 0.05 + time_since_throw * 50.0
            threshold = min(threshold, 400.0)

            best_idx = None
            best_dist = float("inf")

            for idx, mpos in enumerate(markers):
                if idx in used:
                    continue
                # Must be above landing plane
                if mpos[2] < self.landing_z + self.min_height_above_landing_mm:
                    continue
                dist = np.linalg.norm(mpos - predicted_pos)
                if dist < threshold and dist < best_dist:
                    best_dist = dist
                    best_idx = idx

            if best_idx is not None:
                # Match found — promote to CONFIRMED
                kf.update(markers[best_idx])
                used.add(best_idx)
                ball.tracking = TrackingConfidence.CONFIRMED
                ball.frames_tracked += 1
                ball.frames_without_measurement = 0
                ball.last_seen = current_time
                ball.position = kf.position
                ball.velocity = kf.velocity
                ball.timestamp = current_time
                self._update_landing_prediction(ball, from_filter=True)
            else:
                ball.frames_without_measurement += 1

        return used

    # ------------------------------------------------------------------
    # Internal: Parabolic motion detection (human throw path)
    # ------------------------------------------------------------------

    def _detect_parabolic(
        self,
        markers: List[np.ndarray],
        current_time: float,
        used: set,
    ):
        """Buffer unmatched markers and detect parabolic motion."""
        remaining = [
            (idx, markers[idx]) for idx in range(len(markers))
            if idx not in used
        ]

        # Associate remaining markers with existing tracks (nearest-neighbour)
        matched_track_ids = set()
        for idx, mpos in remaining:
            best_track_id = None
            best_dist = float("inf")

            for track_id, track in self._marker_tracks.items():
                if track_id in matched_track_ids:
                    continue
                if not track.positions:
                    continue
                # Predict where this marker track should be
                dt = current_time - track.last_timestamp
                if dt <= 0 or dt > 0.1:  # Too old
                    continue
                dist = np.linalg.norm(mpos - track.last_position)
                if dist < self.gate_radius_mm * 2 and dist < best_dist:
                    best_dist = dist
                    best_track_id = track_id

            if best_track_id is not None:
                self._marker_tracks[best_track_id].add(mpos, current_time)
                matched_track_ids.add(best_track_id)
            else:
                # New track
                tid = self._next_track_id
                self._next_track_id += 1
                track = _MarkerTrack()
                track.add(mpos, current_time)
                self._marker_tracks[tid] = track

        # Check tracks for parabolic motion
        promote_ids = []
        for track_id, track in self._marker_tracks.items():
            if len(track) < self.parabolic_min_frames:
                continue
            if self._is_parabolic(track):
                promote_ids.append(track_id)

        for track_id in promote_ids:
            track = self._marker_tracks.pop(track_id)
            self._promote_to_ball(track, current_time)

        # Prune stale tracks (no update for > 50ms)
        stale = [
            tid for tid, t in self._marker_tracks.items()
            if current_time - t.last_timestamp > 0.05
        ]
        for tid in stale:
            del self._marker_tracks[tid]

    def _is_parabolic(self, track: _MarkerTrack) -> bool:
        """Check if a marker track exhibits gravity-consistent acceleration."""
        if len(track) < 3:
            return False

        # Use the last 3-4 points
        positions = track.positions[-4:]
        timestamps = track.timestamps[-4:]

        if len(positions) < 3:
            return False

        # Compute acceleration via finite differences
        accels = []
        for i in range(1, len(positions) - 1):
            dt1 = timestamps[i] - timestamps[i - 1]
            dt2 = timestamps[i + 1] - timestamps[i]
            if dt1 <= 0 or dt2 <= 0:
                return False

            v1 = (positions[i] - positions[i - 1]) / dt1
            v2 = (positions[i + 1] - positions[i]) / dt2
            dt_avg = 0.5 * (dt1 + dt2)
            accel = (v2 - v1) / dt_avg
            accels.append(accel)

        if not accels:
            return False

        avg_accel = np.mean(accels, axis=0)
        expected = np.array([0.0, 0.0, -GRAVITY_MMPS2])
        residual = np.linalg.norm(avg_accel - expected)

        return residual < self.parabolic_accel_threshold

    def _promote_to_ball(self, track: _MarkerTrack, current_time: float):
        """Create a new Ball from a parabolic marker track."""
        ball_id = self._next_id
        self._next_id += 1

        # Initialize KF from the track positions
        kf = KalmanFilter(
            dt=self.dt,
            process_noise=self.process_noise,
            measurement_noise=self.measurement_noise,
        )
        kf.initialize_from_positions(track.positions, track.timestamps)

        ball = Ball(
            id=ball_id,
            status=BallStatus.IN_FLIGHT,
            tracking=TrackingConfidence.CONFIRMED,
            source="human_throw",
            position=kf.position,
            velocity=kf.velocity,
            timestamp=current_time,
            frames_tracked=len(track),
            last_seen=current_time,
        )

        self._update_landing_prediction(ball, from_filter=True)
        # Human throws have no assigned destination — the coordinator
        # only catches balls explicitly aimed at it (via BB announcements).
        self._balls[ball_id] = ball
        self._filters[ball_id] = kf

    # ------------------------------------------------------------------
    # Internal: Lifecycle transitions
    # ------------------------------------------------------------------

    def _check_lifecycle(self, current_time: float):
        """Transition in-flight balls to terminal states."""
        for ball in self._balls.values():
            if ball.status != BallStatus.IN_FLIGHT:
                continue

            # Check: past expected landing time + grace → CAUGHT or UNKNOWN
            if ball.landing_time > 0 and current_time > ball.landing_time + self.caught_grace_s:
                if ball.tracking == TrackingConfidence.CONFIRMED:
                    if ball.frames_without_measurement >= self.missed_frames_to_lose:
                        # Marker disappeared around landing time → CAUGHT
                        ball.status = BallStatus.CAUGHT
                        ball.terminal_since = current_time
                    # If marker still visible past landing: let it continue
                    # (it might be bouncing — will eventually hit lost-track)
                else:
                    # ANNOUNCED-only ball past landing: UNKNOWN
                    if ball.frames_without_measurement > self.max_frames_without_measurement:
                        ball.status = BallStatus.UNKNOWN
                        ball.terminal_since = current_time

            # Check: lost tracking for too long → UNKNOWN
            if ball.frames_without_measurement >= self.max_frames_without_measurement:
                if ball.status not in TERMINAL_STATUSES:
                    ball.status = BallStatus.UNKNOWN
                    ball.terminal_since = current_time

    def _cleanup_terminal(self, current_time: float):
        """Remove terminal balls past retention period."""
        to_remove = [
            ball_id for ball_id, ball in self._balls.items()
            if ball.status in TERMINAL_STATUSES
            and ball.terminal_since is not None
            and current_time - ball.terminal_since > self.terminal_retention_s
        ]
        for ball_id in to_remove:
            del self._balls[ball_id]
            self._filters.pop(ball_id, None)

    # ------------------------------------------------------------------
    # Internal: Landing prediction
    # ------------------------------------------------------------------

    def _update_landing_prediction(self, ball: Ball, from_filter: bool = False):
        """Recompute landing prediction from ball's current state."""
        pos = ball.position
        vel = ball.velocity

        result = predict_landing_state(pos, vel, self.landing_z)
        if result is not None:
            landing_pos, landing_vel, ttl = result
            ball.landing_position = landing_pos
            ball.landing_velocity = landing_vel
            ball.landing_time = ball.timestamp + ttl
