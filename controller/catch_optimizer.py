"""Catch height optimizer: choose catch Z to minimise platform jerk.

Given a ball landing at (x, y) with a particular approach velocity, the
catch coordinator must decide the platform Z at which to intercept.  This
module evaluates a grid of candidate Z heights, rejects those that violate
workspace limits (via an IK callable), and scores the remainder by the
analytical quintic jerk integral from the current pose to the candidate
catch pose.

The optimizer is a lightweight utility — no MPC solve, no sim dependencies.
It depends only on numpy, scipy (for rotation), and the quintic jerk
integral from ``controller.hermite``.

Typical usage::

    optimizer = CatchHeightOptimizer(
        ik_fn=plant.pose_to_extensions,
        stroke_mm=280.0,
    )
    best_z, jerk_score = optimizer.optimize(
        landing_xy_mm=ball.landing_position[:2],
        landing_velocity_mms=ball.landing_velocity,
        current_pose=current_pose_6dof,
        current_twist=current_twist_6dof,
        time_available=landing_time - sim_time,
    )
"""

from __future__ import annotations

import logging
import math
from typing import Callable, Optional

import numpy as np
from scipy.spatial.transform import Rotation

from .hermite import quintic_jerk_integral

logger = logging.getLogger(__name__)

# Defaults
_DEFAULT_N_CANDIDATES = 8
_DEFAULT_STROKE_MARGIN_MM = 5.0
_CATCH_ANGLE_LIMIT_RAD = math.radians(30.0)


def compute_catch_orientation(
    landing_velocity_mms: np.ndarray,
    angle_limit_rad: float = _CATCH_ANGLE_LIMIT_RAD,
) -> Optional[np.ndarray]:
    """Compute rotation vector to orient platform normal to ball velocity.

    Returns (3,) rotation vector or None if the approach angle exceeds
    the angle limit.

    This is the same algorithm as ``CatchCoordinator.compute_catch_orientation``
    but returns a rotation vector instead of a quaternion (matching the MPC's
    pose convention [x, y, z, rx, ry, rz]).
    """
    v = np.asarray(landing_velocity_mms, dtype=np.float64)
    norm = np.linalg.norm(v)
    if norm < 1e-6:
        return np.zeros(3)

    v_hat = v / norm
    reference = np.array([0.0, 0.0, -1.0])  # platform Z points down toward ball

    dot = np.clip(np.dot(reference, v_hat), -1.0, 1.0)
    angle = math.acos(dot)

    if angle > angle_limit_rad:
        return None

    if angle < 1e-9:
        return np.zeros(3)

    axis = np.cross(reference, v_hat)
    axis_norm = np.linalg.norm(axis)
    if axis_norm < 1e-9:
        # Anti-parallel
        return np.array([0.0, math.pi, 0.0])

    axis = axis / axis_norm
    return axis * angle


def compute_catch_pose(
    landing_xy_mm: np.ndarray,
    landing_z_mm: float,
    landing_velocity_mms: np.ndarray,
    hand_catch_offset_mm: float,
    initial_height_mm: float,
    angle_limit_rad: float = _CATCH_ANGLE_LIMIT_RAD,
) -> Optional[np.ndarray]:
    """Compute the full 6-DOF catch pose for a given catch Z.

    Returns (6,) pose [x, y, z, rx, ry, rz] in platform-active-relative
    coordinates (mm/rad), or None if the approach angle exceeds limits.

    Parameters
    ----------
    landing_xy_mm : (2,) XY landing position in base frame (mm).
    landing_z_mm : float
        Z coordinate of the catch plane in base frame (mm).
    landing_velocity_mms : (3,) ball velocity at landing (mm/s).
    hand_catch_offset_mm : float
        Offset from platform centroid to hand catch point along local Z (mm).
    initial_height_mm : float
        Platform height at stow/active pose (mm).
    """
    rotvec = compute_catch_orientation(landing_velocity_mms, angle_limit_rad)
    if rotvec is None:
        return None

    # Platform local Z after tilting
    rot = Rotation.from_rotvec(rotvec)
    platform_z = rot.apply(np.array([0.0, 0.0, 1.0]))

    # Hand is at (landing_xy, landing_z). Platform centroid is offset.
    hand_pos = np.array([landing_xy_mm[0], landing_xy_mm[1], landing_z_mm])
    centroid_base = hand_pos - hand_catch_offset_mm * platform_z

    # Convert to platform-active-relative coordinates
    target_pos = np.array([
        centroid_base[0],
        centroid_base[1],
        centroid_base[2] - initial_height_mm,
    ])

    return np.concatenate([target_pos, rotvec])


class CatchHeightOptimizer:
    """Choose catch Z to minimise platform jerk within workspace limits.

    Parameters
    ----------
    ik_fn : callable
        ``ik_fn(pose_6dof) -> (6,) extensions_mm``.  Typically
        ``plant.pose_to_extensions``.
    stroke_mm : float
        Total leg stroke (mm).
    hand_catch_offset_mm : float
        Offset from centroid to hand catch point along local Z (mm).
    initial_height_mm : float
        Platform height at ACTIVE pose (mm).
    stroke_margin_mm : float
        Safety margin from stroke endpoints (mm).
    n_candidates : int
        Number of Z candidates to evaluate.
    """

    def __init__(
        self,
        ik_fn: Callable[[np.ndarray], np.ndarray],
        stroke_mm: float,
        hand_catch_offset_mm: float = 64.78,
        initial_height_mm: float = 574.3,
        stroke_margin_mm: float = _DEFAULT_STROKE_MARGIN_MM,
        n_candidates: int = _DEFAULT_N_CANDIDATES,
    ) -> None:
        self._ik_fn = ik_fn
        self._stroke_mm = stroke_mm
        self._hand_catch_offset_mm = hand_catch_offset_mm
        self._initial_height_mm = initial_height_mm
        self._margin = stroke_margin_mm
        self._n_candidates = n_candidates

    def optimize(
        self,
        landing_xy_mm: np.ndarray,
        landing_velocity_mms: np.ndarray,
        current_pose: np.ndarray,
        current_twist: np.ndarray,
        time_available: float,
        z_range: Optional[tuple[float, float]] = None,
    ) -> Optional[tuple[np.ndarray, float]]:
        """Find the catch pose that minimises platform jerk.

        Parameters
        ----------
        landing_xy_mm : (2,) XY landing position in base frame (mm).
        landing_velocity_mms : (3,) ball velocity at landing (mm/s).
        current_pose : (6,) current platform pose [x,y,z,rx,ry,rz].
        current_twist : (6,) current platform twist.
        time_available : float
            Seconds until the ball arrives.
        z_range : (z_min, z_max) or None
            Base-frame Z range to search.  Defaults to a window around
            the nominal catch height.

        Returns
        -------
        (best_pose, jerk_score) or None if no feasible candidate exists.
        best_pose : (6,) catch pose [x, y, z, rx, ry, rz].
        jerk_score : float, integrated squared jerk of the best candidate.
        """
        if time_available <= 0:
            return None

        current_pose = np.asarray(current_pose, dtype=np.float64)
        current_twist = np.asarray(current_twist, dtype=np.float64)
        zero6 = np.zeros(6)

        # Default Z range: nominal catch height ± workspace margin
        if z_range is None:
            # Nominal: initial_height + a reasonable active-Z range
            nominal_z = self._initial_height_mm + self._hand_catch_offset_mm
            z_min = nominal_z - 100.0  # 100mm below nominal
            z_max = nominal_z + 100.0  # 100mm above nominal
        else:
            z_min, z_max = z_range

        z_candidates = np.linspace(z_min, z_max, self._n_candidates)

        best_pose = None
        best_jerk = float('inf')

        for z in z_candidates:
            # Compute full catch pose for this Z
            catch_pose = compute_catch_pose(
                landing_xy_mm=landing_xy_mm,
                landing_z_mm=z,
                landing_velocity_mms=landing_velocity_mms,
                hand_catch_offset_mm=self._hand_catch_offset_mm,
                initial_height_mm=self._initial_height_mm,
            )
            if catch_pose is None:
                continue  # Approach angle too steep

            # IK feasibility check
            try:
                extensions = self._ik_fn(catch_pose)
            except Exception:
                continue

            if np.any(extensions < -self._margin):
                continue
            if np.any(extensions > self._stroke_mm + self._margin):
                continue

            # Analytical jerk scoring: quintic from current state to catch pose
            jerk = quintic_jerk_integral(
                p0=current_pose,
                v0=current_twist,
                a0=zero6,
                p1=catch_pose,
                v1=zero6,  # stationary catch
                a1=zero6,
                duration=time_available,
            )

            if jerk < best_jerk:
                best_jerk = jerk
                best_pose = catch_pose

        if best_pose is None:
            return None

        return best_pose, best_jerk
