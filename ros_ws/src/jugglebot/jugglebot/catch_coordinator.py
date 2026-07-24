"""Catch coordinator — pure Python catch policy.

Decides which ball to catch, computes the catch pose, and manages
a feasibility blacklist. No ROS2 dependency.

The ROS2 wrapper (catch_coordinator_node.py) feeds Ball objects in
and reads CatchCommand objects out, then sends them via IPC.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np
from scipy.spatial.transform import Rotation

from jugglebot.tracking.ball import Ball, BallStatus
# Single source of truth for the usable receive-tilt ceiling (12°) — shared with the
# sim-validated tilt_geometry.tilt_to_receive so the ROS catch path and the sim clamp
# agree. Pure Python (no ROS), so this keeps catch_coordinator ROS-free.
from jugglebot.motion.trajectory.tilt_geometry import MAX_TILT_DEG

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Output dataclass
# ---------------------------------------------------------------------------

@dataclass
class CatchCommand:
    """Output of the coordinator: what to send to the motion planner and hand."""
    ball_id: int
    target_pos: np.ndarray     # [x, y, z] platform offset from stow pose (mm)
    target_quat: np.ndarray    # [w, x, y, z] quaternion orientation
    target_vel: np.ndarray     # [vx, vy, vz] mm/s — always [0,0,0] for a catch
    landing_time: float        # Absolute landing time (ROS2 seconds)

    # Hand control — populated by the coordinator for the node to execute.
    # event_vel_mps is the ball speed at landing (m/s), clamped to Teensy range.
    # The node computes event_delay from landing_time and current time.
    arm_hand: bool = False         # True when hand catch trajectory should be armed
    event_vel_mps: float = 0.0     # Ball speed at landing, m/s (clamped [0.3, 7.0])


# Hand trajectory constants (Teensy limits)
_MIN_EVENT_VEL_MPS = 0.3
_MAX_EVENT_VEL_MPS = 7.0


# ---------------------------------------------------------------------------
# Blacklist entry
# ---------------------------------------------------------------------------

@dataclass
class _BlacklistEntry:
    ball_id: int
    landing_position_snapshot: np.ndarray  # Position when blacklisted
    rejection_count: int = 0


# ---------------------------------------------------------------------------
# Coordinator
# ---------------------------------------------------------------------------

class CatchCoordinator:
    """Stateless-ish catch policy. Call ``update()`` each cycle with current balls.

    Parameters
    ----------
    robot_name : str
        Only balls with ``destination == robot_name`` are considered.
    initial_height_mm : float
        Platform initial height (GEOM_INITIAL_HEIGHT_MM).
    landing_z_offset_mm : float
        Offset above initial height for the catch plane.
    hand_catch_offset_mm : float
        Height of hand catch point above platform centroid along local Z (HAND_CATCH_OFFSET_MM).
    catch_angle_limit_deg : float
        Maximum approach angle from vertical (degrees).
    blacklist_rejection_threshold : int
        Consecutive rejections before blacklisting a ball.
    blacklist_reeval_threshold_mm : float
        Landing position shift to remove a ball from the blacklist.
    min_lead_time_s : float
        Minimum time before landing to attempt a catch.
    xy_range_mm : float
        Maximum XY distance from origin for preliminary feasibility.
    """

    def __init__(
        self,
        robot_name: str = "jugglebot",
        initial_height_mm: float = 574.3,
        landing_z_offset_mm: float = 160.0,
        hand_catch_offset_mm: float = 64.78,
        catch_angle_limit_deg: float = 30.0,
        blacklist_rejection_threshold: int = 3,
        blacklist_reeval_threshold_mm: float = 50.0,
        min_lead_time_s: float = 0.3,
        xy_range_mm: float = 400.0,
    ):
        self.robot_name = robot_name
        self.initial_height_mm = initial_height_mm
        self.catch_z_offset_mm = landing_z_offset_mm
        self.hand_catch_offset_mm = hand_catch_offset_mm
        # Retained for API/back-compat; the receive-tilt is now CLAMPED (not gated) at
        # tilt_geometry.MAX_TILT_DEG in compute_catch_orientation, so this no longer
        # rejects steep arrivals.
        self.catch_angle_limit_rad = math.radians(catch_angle_limit_deg)
        self.blacklist_rejection_threshold = blacklist_rejection_threshold
        self.blacklist_reeval_threshold_mm = blacklist_reeval_threshold_mm
        self.min_lead_time_s = min_lead_time_s
        self.xy_range_mm = xy_range_mm

        # Blacklist: ball_id → _BlacklistEntry
        self._blacklist: Dict[int, _BlacklistEntry] = {}
        # Track consecutive rejections per ball (reset on acceptance)
        self._rejection_counts: Dict[int, int] = {}

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def update(
        self,
        balls: List[Ball],
        current_time: float,
        exclude_ids: Optional[set] = None,
    ) -> Optional[CatchCommand]:
        """Process current ball states and return a catch command (or None).

        Args:
            balls: All active balls from the tracker.
            current_time: Current ROS2 time in seconds.
            exclude_ids: Ball ids never eligible as a catch candidate — the node
                passes the ids already IN_FLIGHT at the catch-armed rising edge so a
                leftover track from a PRIOR attempt cannot drive this reload. Note:
                this snapshots at the arm edge and so CANNOT exclude the current
                ball's OWN corrupt split-track (it spawns during the countdown, after
                the snapshot) — the node's arrival-window arm-guard is the real
                protection for that class; this is defense-in-depth for prior-attempt
                leftovers.

        Returns:
            CatchCommand for the best catchable ball, or None if nothing to catch.
        """
        # Clean up blacklist entries for balls no longer in flight
        active_ids = {b.id for b in balls if b.status == BallStatus.IN_FLIGHT}
        stale = [bid for bid in self._blacklist if bid not in active_ids]
        for bid in stale:
            del self._blacklist[bid]
            self._rejection_counts.pop(bid, None)

        # Filter candidates
        candidates = []
        for ball in balls:
            if ball.status != BallStatus.IN_FLIGHT:
                continue
            if ball.destination != self.robot_name:
                continue
            if exclude_ids and ball.id in exclude_ids:
                continue
            if self._is_blacklisted(ball):
                continue
            if not self._preliminary_feasibility(ball, current_time):
                continue
            candidates.append(ball)

        if not candidates:
            return None

        # Select best: earliest landing time
        candidates.sort(key=lambda b: b.landing_time)
        best = candidates[0]

        # Compute catch pose
        return self._compute_catch_command(best)

    def report_rejection(self, ball_id: int):
        """Report that the motion planner rejected a dynamic target for this ball."""
        count = self._rejection_counts.get(ball_id, 0) + 1
        self._rejection_counts[ball_id] = count

        if count >= self.blacklist_rejection_threshold:
            # Find the ball's current landing position for the snapshot
            # (caller should provide this, but we store from last update)
            if ball_id not in self._blacklist:
                self._blacklist[ball_id] = _BlacklistEntry(
                    ball_id=ball_id,
                    landing_position_snapshot=np.zeros(3),  # Updated below
                )

    def report_rejection_with_position(self, ball_id: int, landing_position: np.ndarray):
        """Report rejection with the ball's current landing position."""
        count = self._rejection_counts.get(ball_id, 0) + 1
        self._rejection_counts[ball_id] = count

        if count >= self.blacklist_rejection_threshold:
            self._blacklist[ball_id] = _BlacklistEntry(
                ball_id=ball_id,
                landing_position_snapshot=landing_position.copy(),
                rejection_count=count,
            )

    def report_acceptance(self, ball_id: int):
        """Report that the motion planner accepted a dynamic target for this ball."""
        self._rejection_counts.pop(ball_id, None)

    # ------------------------------------------------------------------
    # Catch pose computation
    # ------------------------------------------------------------------

    def predicted_catch_command(
        self,
        landing_position: np.ndarray,
        landing_velocity: np.ndarray,
        landing_time: float,
    ) -> Optional[CatchCommand]:
        """The PRE-TILT catch command from an announced (not yet flying) throw.

        Same pose math as the reactive per-ball path (single-sourced through
        :meth:`_compute_catch_command` on a synthetic ball), driven from the
        announcement's solver-consistent landing prediction ~3.9 s before the ball
        lands — so the platform settles into the receive tilt during the throw
        countdown instead of reaching mid-flight (2026-07-23 session: the reactive
        reach was only ~95 % settled at contact). The receive-tilt DIRECTION is the
        BB→target azimuth, which is flight-invariant, and the magnitude saturates at
        the 12° clamp for real BB arrivals — which is why pre-tilting to the
        predicted arrival is safe. With JB_OP_RELOAD_PLATFORM_OPEN_LOOP off,
        mid-flight refinements supersede via C2 replans; under the default
        open-loop the node holds this pose for the whole flight.

        ``arm_hand`` is forced False: the hand arm must stay flight-gated (the
        Teensy fire is one-shot on the FIRST event time it receives — arming at
        announcement would bake ~3.9 s of countdown jitter into it; the release-time
        arm from the same announced landing_time is strictly tighter)."""
        ball = Ball(
            id=-1,
            status=BallStatus.IN_FLIGHT,
            tracking=None,
            source='announcement',
            destination=self.robot_name,
            position=np.asarray(landing_position, dtype=float),
            velocity=np.asarray(landing_velocity, dtype=float),
            landing_position=np.asarray(landing_position, dtype=float),
            landing_velocity=np.asarray(landing_velocity, dtype=float),
            landing_time=float(landing_time),
        )
        cmd = self._compute_catch_command(ball)
        if cmd is not None:
            cmd.arm_hand = False
        return cmd

    def _compute_catch_command(self, ball: Ball) -> Optional[CatchCommand]:
        """Compute the catch pose and hand parameters from a ball's landing state."""
        quat = self.compute_catch_orientation(ball.landing_velocity)
        if quat is None:
            return None

        # The hand catches the ball, not the platform centroid. The hand's catch
        # point is offset from the centroid along the platform's local Z axis by
        # hand_catch_offset_mm. We position the platform so that the hand (not the
        # centroid) is at the ball's landing position.
        #
        # Platform local Z after tilting = rotation applied to [0,0,1].
        rot = Rotation.from_quat([quat[1], quat[2], quat[3], quat[0]])  # scipy: xyzw
        platform_z = rot.apply(np.array([0.0, 0.0, 1.0]))

        # Ball landing position in base frame → desired hand position in base frame.
        # Platform centroid = hand position - offset * platform_z
        centroid_base = ball.landing_position - self.hand_catch_offset_mm * platform_z

        # Convert from base frame to STOW-relative coordinates (the trajectory pose
        # convention: 0 = the STOW plane at initial_height in base frame, ~170 =
        # active). NOTE the earlier comment here called (0, 0, initial_height) "the
        # active pose" — that mislabel seeded a consumer-side double-add of the active
        # height (fixed 2026-07-23; see CatchCommand's docstring: "offset from stow").
        target_pos = np.array([
            centroid_base[0],
            centroid_base[1],
            centroid_base[2] - self.initial_height_mm,
        ])

        # Hand: compute ball speed at landing, clamped to Teensy range
        speed_mps = float(np.linalg.norm(ball.landing_velocity)) / 1000.0
        speed_mps = max(_MIN_EVENT_VEL_MPS, min(_MAX_EVENT_VEL_MPS, speed_mps))

        return CatchCommand(
            ball_id=ball.id,
            target_pos=target_pos,
            target_quat=quat,
            target_vel=np.zeros(3),  # Stationary catch
            landing_time=ball.landing_time,
            arm_hand=True,
            event_vel_mps=speed_mps,
        )

    def compute_catch_orientation(
        self,
        landing_velocity: np.ndarray,
    ) -> Optional[np.ndarray]:
        """Compute quaternion [w,x,y,z] to orient the platform normal to the ball's
        arrival velocity (the collinear "receive" catch).

        The from-vertical tilt is CLAMPED to the usable receive-tilt ceiling
        (``tilt_geometry.MAX_TILT_DEG`` = 12° — the single source of truth shared with the
        sim-validated ``tilt_to_receive``), NOT rejected above it. Real BB arrivals are
        18–40° off vertical; a clamped tilt seats the ball better than a level cup even
        though the collinearity is only partial past the ceiling (the plan's design
        explicitly accepts non-collinearity beyond the clamp). Never returns None for a
        steep arrival — the axis direction is preserved; only the magnitude saturates.
        """
        v = landing_velocity
        norm = np.linalg.norm(v)
        if norm < 1e-6:
            # Near-zero velocity: identity orientation
            return np.array([1.0, 0.0, 0.0, 0.0])

        v_hat = v / norm
        reference = np.array([0.0, 0.0, -1.0])  # Platform Z-axis (downward)

        dot = np.clip(np.dot(reference, v_hat), -1.0, 1.0)
        angle = math.acos(dot)

        max_tilt_rad = math.radians(MAX_TILT_DEG)
        if angle > max_tilt_rad:
            logger.info(
                "Receive tilt %.1f° exceeds the %.1f° ceiling — clamping (partial "
                "collinear seat, ball still seated better than level).",
                math.degrees(angle), MAX_TILT_DEG)
            angle = max_tilt_rad

        if angle < 1e-9:
            # Nearly aligned: identity
            return np.array([1.0, 0.0, 0.0, 0.0])

        axis = np.cross(reference, v_hat)
        axis_norm = np.linalg.norm(axis)
        if axis_norm < 1e-9:
            # Anti-parallel: arbitrary 180° rotation
            return np.array([0.0, 1.0, 0.0, 0.0])

        axis = axis / axis_norm
        rot = Rotation.from_rotvec(angle * axis)
        # scipy returns [x, y, z, w] — convert to [w, x, y, z]
        q_xyzw = rot.as_quat()
        return np.array([q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]])

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _preliminary_feasibility(self, ball: Ball, current_time: float) -> bool:
        """Quick checks: time and XY range."""
        # Enough lead time?
        time_to_land = ball.landing_time - current_time
        if time_to_land < self.min_lead_time_s:
            return False

        # Within XY range?
        xy = ball.landing_position[:2]
        if abs(xy[0]) > self.xy_range_mm or abs(xy[1]) > self.xy_range_mm:
            return False

        return True

    def _is_blacklisted(self, ball: Ball) -> bool:
        """Check if a ball is blacklisted, with re-eval escape hatch."""
        entry = self._blacklist.get(ball.id)
        if entry is None:
            return False

        # Re-eval: if landing position shifted significantly, remove from blacklist
        shift = np.linalg.norm(ball.landing_position - entry.landing_position_snapshot)
        if shift > self.blacklist_reeval_threshold_mm:
            del self._blacklist[ball.id]
            self._rejection_counts.pop(ball.id, None)
            return False

        return True
