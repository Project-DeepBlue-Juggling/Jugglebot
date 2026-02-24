"""Workspace analysis and constraint checking for the Stewart platform.

Provides leg extension validation, reachability checks, and singularity
mapping.  All functions are pure Python + numpy (no ROS2 dependency).
"""

from __future__ import annotations

import numpy as np
from numpy.linalg import norm

from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.ik_solver import (
    compute_jacobian,
    compute_leg_vectors,
    pose_to_leg_lengths,
    rotvec_to_rot_matrix,
)

# Condition number threshold above which a pose is considered ill-conditioned.
# This matches the threshold specified in the MOTION_PLANNER_PLAN Phase 1.
ILL_CONDITION_THRESHOLD = 100.0


def check_leg_extensions(extensions_mm: np.ndarray,
                         geom: StewartGeometry
                         ) -> tuple[bool, np.ndarray]:
    """Check whether leg extensions are within stroke limits.

    Parameters
    ----------
    extensions_mm : (6,) ndarray — leg extensions in mm
    geom : StewartGeometry

    Returns
    -------
    all_valid : bool — True if every leg is within bounds
    states : (6,) int8 ndarray — per-leg state:
        0 = within bounds, -1 = underextended, +1 = overextended
    """
    states = np.zeros(6, dtype=np.int8)
    states[extensions_mm < 0.0] = -1
    states[extensions_mm > geom.leg_stroke_mm] = 1
    return bool(np.all(states == 0)), states


def compute_condition_number(pos: np.ndarray,
                             rot: np.ndarray,
                             geom: StewartGeometry) -> float:
    """Condition number of the Jacobian at a given pose.

    A large condition number indicates proximity to a singularity.
    Typical good values are < 10; the ill-conditioned threshold
    from the motion planner plan is 100.
    """
    J = compute_jacobian(pos, rot, geom)
    return float(np.linalg.cond(J))


def check_reachability(pos: np.ndarray,
                       rot: np.ndarray,
                       geom: StewartGeometry) -> bool:
    """Check whether a pose is reachable (all legs within stroke).

    Parameters
    ----------
    pos : (3,) ndarray — platform offset from home
    rot : (3,3) ndarray — rotation matrix

    Returns
    -------
    reachable : bool
    """
    extensions = pose_to_leg_lengths(pos, rot, geom)
    valid, _ = check_leg_extensions(extensions, geom)
    return valid


def map_singularities(geom: StewartGeometry,
                      z_range: tuple[float, float] = (-20.0, 200.0),
                      xy_range: float = 150.0,
                      tilt_max_deg: float = 15.0,
                      resolution: int = 8) -> dict:
    """Sweep the workspace and compute Jacobian condition numbers.

    Parameters
    ----------
    geom : StewartGeometry
    z_range : (min_z, max_z) — vertical offset range from home (mm)
    xy_range : float — maximum horizontal offset (mm) in each direction
    tilt_max_deg : float — maximum tilt angle (deg) to sample
    resolution : int — number of grid points per dimension (higher = finer)

    Returns
    -------
    dict with keys:
        'poses' : list of (pos, rotvec) tuples
        'extensions' : list of (6,) ndarrays
        'condition_numbers' : list of floats
        'reachable' : list of bools
        'ill_conditioned' : list of bools (condition > threshold AND reachable)
        'summary' : dict with aggregate statistics
    """
    z_vals = np.linspace(z_range[0], z_range[1], resolution)
    xy_vals = np.linspace(-xy_range, xy_range, resolution)
    tilt_vals_rad = np.deg2rad(np.linspace(-tilt_max_deg, tilt_max_deg,
                                           max(3, resolution // 2)))

    poses = []
    extensions_list = []
    cond_numbers = []
    reachable_flags = []
    ill_conditioned_flags = []

    for z in z_vals:
        for x in xy_vals:
            for y in xy_vals:
                for rx in tilt_vals_rad:
                    for ry in tilt_vals_rad:
                        pos = np.array([x, y, z])
                        rotvec = np.array([rx, ry, 0.0])
                        rot = rotvec_to_rot_matrix(rotvec)

                        ext = pose_to_leg_lengths(pos, rot, geom)
                        valid, _ = check_leg_extensions(ext, geom)

                        if valid:
                            cond = compute_condition_number(pos, rot, geom)
                        else:
                            cond = float('inf')

                        poses.append((pos.copy(), rotvec.copy()))
                        extensions_list.append(ext)
                        cond_numbers.append(cond)
                        reachable_flags.append(valid)
                        ill_conditioned_flags.append(
                            valid and cond > ILL_CONDITION_THRESHOLD)

    cond_arr = np.array(cond_numbers)
    reachable_arr = np.array(reachable_flags)
    finite_conds = cond_arr[reachable_arr]

    summary = {
        'total_poses': len(poses),
        'reachable': int(np.sum(reachable_arr)),
        'unreachable': int(np.sum(~reachable_arr)),
        'ill_conditioned': int(np.sum(ill_conditioned_flags)),
    }
    if len(finite_conds) > 0:
        summary['cond_min'] = float(np.min(finite_conds))
        summary['cond_max'] = float(np.max(finite_conds))
        summary['cond_mean'] = float(np.mean(finite_conds))
        summary['cond_median'] = float(np.median(finite_conds))

    return {
        'poses': poses,
        'extensions': extensions_list,
        'condition_numbers': cond_numbers,
        'reachable': reachable_flags,
        'ill_conditioned': ill_conditioned_flags,
        'summary': summary,
    }
