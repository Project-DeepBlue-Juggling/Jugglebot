"""Throw-catch planner: converts user-level throw/catch positions into a
complete sequence of platform targets and hand trajectories.

Usage::

    planner = ThrowCatchPlanner()
    plan = planner.plan(
        throw_pos_mm=np.array([0.0, 0.0, 700.0]),
        catch_pos_mm=np.array([0.0, 0.0, 700.0]),
        throw_time=1.0,
        catch_time=2.0,
    )
"""

from __future__ import annotations

import logging
from dataclasses import dataclass

import numpy as np

from hand.ballistics import (
    compute_launch_velocity,
    compute_arrival_velocity,
    compute_orientation,
    compute_hand_offset_mm,
    rodrigues,
)
from hand.coordinator import DynamicTarget, BallSpawn
from hand.trajectory import (
    HandThrowSequence,
    HandCatchSequence,
    HandCatchTrajectory,
    max_throw_speed_mps,
    INERTIA_RATIO,
    CATCH_VEL_HOLD_PCT,
    _TOTAL_STROKE_M,
    STROKE_MARGIN_M,
)

logger = logging.getLogger(__name__)

# x5 position for catch offset: position on the physical stroke (mm from bottom)
# where the ball meets the hand during the catch velocity-hold phase.
# _x5_m is in effective-stroke coords (0 = bottom margin); add STROKE_MARGIN
# to convert to physical stroke position.
_x5_m = _TOTAL_STROKE_M - (_TOTAL_STROKE_M - CATCH_VEL_HOLD_PCT * _TOTAL_STROKE_M) * INERTIA_RATIO / (1.0 + INERTIA_RATIO)
_HAND_CATCH_X5_MM = STROKE_MARGIN_M * 1000.0 + _x5_m * 1000.0

_PLATFORM_HEIGHT_MM = 574.3
_SETTLE_MARGIN_S = 0.1


@dataclass
class ThrowCatchPlan:
    """Complete plan for a throw-catch cycle."""
    throw_target: DynamicTarget
    catch_target: DynamicTarget
    throw_hand_seq: HandThrowSequence
    catch_hand_seq: HandCatchSequence
    ball_release_time: float
    ball_release_vel_mms: np.ndarray
    ball_spawn: BallSpawn | None


class ThrowCatchPlanner:
    """Plans a throw-catch cycle from user-specified ball positions and times.

    Parameters
    ----------
    platform_height_mm : float
        Home position Z (574.3 mm).
    """

    def __init__(self, platform_height_mm: float = _PLATFORM_HEIGHT_MM):
        self._height = platform_height_mm

    def plan(
        self,
        throw_pos_mm: np.ndarray,
        catch_pos_mm: np.ndarray,
        throw_time: float,
        catch_time: float,
        current_hand_pos_mm: float = 0.0,
        current_time: float = 0.0,
        throw_platform_twist: np.ndarray | None = None,
        catch_platform_twist: np.ndarray | None = None,
    ) -> ThrowCatchPlan:
        """Compute a complete throw-catch plan.

        Parameters
        ----------
        throw_platform_twist : (6,) array or None
            Platform [vx,vy,vz,wx,wy,wz] at throw time (mm/s, rad/s).
            When provided, the ball's launch velocity includes the platform
            velocity contribution, and settle margin is removed.
        catch_platform_twist : (6,) array or None
            Platform twist at catch time.  The hand catch speed is computed
            from the relative velocity between ball and platform.

        Raises ValueError if infeasible (throw speed exceeds hand limits,
        tilt exceeds workspace, insufficient transit time, etc.).
        """
        throw_pos_mm = np.asarray(throw_pos_mm, dtype=float)
        catch_pos_mm = np.asarray(catch_pos_mm, dtype=float)
        continuous = (throw_platform_twist is not None
                      or catch_platform_twist is not None)
        v_plat_throw = (np.asarray(throw_platform_twist, dtype=float)
                        if throw_platform_twist is not None
                        else np.zeros(6))
        v_plat_catch = (np.asarray(catch_platform_twist, dtype=float)
                        if catch_platform_twist is not None
                        else np.zeros(6))
        settle = 0.0 if continuous else _SETTLE_MARGIN_S

        # --- 1. Validate inputs ---
        if throw_time < 0:
            raise ValueError(f"throw_time must be non-negative (got {throw_time:.3f}s)")
        flight_time = catch_time - throw_time
        if flight_time <= 0:
            raise ValueError(f"catch_time must be after throw_time (got {flight_time:.3f}s)")

        # --- 2. Compute launch velocity (world-frame ball velocity) ---
        v_launch = compute_launch_velocity(throw_pos_mm, catch_pos_mm, flight_time)

        # --- 3. Compute hand-relative velocity at throw ---
        # v_ball = v_hand_relative + v_platform  =>  v_hand_rel = v_ball - v_platform
        v_hand_rel_throw = v_launch - v_plat_throw[:3]

        # --- 4. Compute throw orientation (platform +Z aligned with hand-relative vel) ---
        throw_rv = compute_orientation(v_hand_rel_throw)
        R_throw = rodrigues(throw_rv)
        platform_z_throw = R_throw @ np.array([0.0, 0.0, 1.0])

        # --- 5. Compute required hand throw speed (along platform Z) ---
        hand_throw_speed_mps = float(np.dot(v_hand_rel_throw, platform_z_throw)) / 1000.0
        if hand_throw_speed_mps < 0:
            raise ValueError(
                f"Hand-relative throw speed is negative ({hand_throw_speed_mps:.2f} m/s) "
                f"— platform velocity overshoots launch velocity"
            )
        max_speed = max_throw_speed_mps()
        if hand_throw_speed_mps > max_speed:
            raise ValueError(
                f"Required throw speed {hand_throw_speed_mps:.2f} m/s exceeds "
                f"maximum {max_speed:.2f} m/s"
            )

        # --- 6. Build throw hand trajectory ---
        throw_hand_seq = HandThrowSequence(
            hand_throw_speed_mps, throw_time, current_hand_pos_mm)

        # --- 7. Compute throw platform centroid ---
        release_pos_mm = throw_hand_seq.throw_trajectory.release_pos_mm
        throw_offset = compute_hand_offset_mm(release_pos_mm)
        throw_centroid = throw_pos_mm - throw_offset * platform_z_throw
        throw_pose = np.array([
            throw_centroid[0],
            throw_centroid[1],
            throw_centroid[2] - self._height,
            throw_rv[0], throw_rv[1], throw_rv[2],
        ])

        # --- 8. Compute arrival velocity and hand-relative catch velocity ---
        v_arrival = compute_arrival_velocity(v_launch, flight_time)
        v_hand_rel_catch = v_arrival - v_plat_catch[:3]

        # --- 9. Compute catch orientation (platform +Z faces into incoming relative vel) ---
        catch_rv = compute_orientation(-v_hand_rel_catch)
        R_catch = rodrigues(catch_rv)
        platform_z_catch = R_catch @ np.array([0.0, 0.0, 1.0])

        # --- 10. Compute catch platform centroid ---
        catch_hand_offset = compute_hand_offset_mm(_HAND_CATCH_X5_MM)
        catch_centroid = catch_pos_mm - catch_hand_offset * platform_z_catch
        catch_pose = np.array([
            catch_centroid[0],
            catch_centroid[1],
            catch_centroid[2] - self._height,
            catch_rv[0], catch_rv[1], catch_rv[2],
        ])

        # --- 11. Build catch hand trajectory ---
        # Catch speed = magnitude of relative velocity along platform Z
        event_vel_mps = abs(float(np.dot(v_hand_rel_catch, platform_z_catch))) / 1000.0
        event_vel_mps = max(0.3, min(7.0, event_vel_mps))
        catch_hand_seq = HandCatchSequence(
            event_vel_mps, catch_time, throw_hand_seq.end_pos_mm)

        # --- 12. Build DynamicTargets ---
        throw_arrival_twist = v_plat_throw.copy() if continuous else None
        catch_arrival_twist = v_plat_catch.copy() if continuous else None

        throw_target = DynamicTarget(
            pose_6dof=throw_pose,
            arrival_time=throw_time,
            arrival_twist=throw_arrival_twist,
            hold_duration=0.0,
            event_vel_mps=hand_throw_speed_mps,
            settle_margin_s=settle,
            mode='throw',
        )
        catch_target = DynamicTarget(
            pose_6dof=catch_pose,
            arrival_time=catch_time,
            arrival_twist=catch_arrival_twist,
            hold_duration=0.5,
            event_vel_mps=event_vel_mps,
            settle_margin_s=settle,
            mode='catch',
        )

        # --- 13. Build BallSpawn ---
        ball_spawn = BallSpawn(
            position_mm=throw_pos_mm.copy(),
            velocity_mms=v_launch.copy(),
            spawn_time=throw_time,
        )

        # --- 14. Feasibility: timing chain ---
        throw_end_abs = throw_hand_seq.end_time
        catch_prelude_start = catch_hand_seq.prelude_start_time
        if throw_end_abs > catch_prelude_start:
            raise ValueError(
                f"Throw ends at {throw_end_abs:.3f}s but catch prelude needs "
                f"to start at {catch_prelude_start:.3f}s — insufficient transit time"
            )

        return ThrowCatchPlan(
            throw_target=throw_target,
            catch_target=catch_target,
            throw_hand_seq=throw_hand_seq,
            catch_hand_seq=catch_hand_seq,
            ball_release_time=throw_time,
            ball_release_vel_mms=v_launch.copy(),
            ball_spawn=ball_spawn,
        )
