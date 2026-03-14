"""Workspace analysis and constraint checking for the Stewart platform.

Provides leg extension validation, reachability checks, singularity
mapping, and runtime workspace limit enforcement (Phase 6).

All functions are pure Python + numpy (no ROS2 dependency).
"""

from __future__ import annotations

import enum
from dataclasses import dataclass

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

# ---------------------------------------------------------------------------
# Workspace limit parameters (Phase 6)
# ---------------------------------------------------------------------------

# Leg extension soft/hard margins (mm from physical endpoints).
# Soft limit triggers speed degradation; hard limit triggers trajectory abort.
LEG_SOFT_MARGIN_MM = 15.0   # soft limit = [margin, stroke - margin]
LEG_HARD_MARGIN_MM = 5.0    # hard limit = [margin, stroke - margin]

# Condition number thresholds (relative to home).
# Computed once at startup: home cond ~450, so soft ~675, hard ~900.
COND_SOFT_FACTOR = 1.5      # soft = 1.5 * cond_home
COND_HARD_FACTOR = 2.0      # hard = 2.0 * cond_home (matches feasibility checker)


class WorkspaceStatus(enum.Enum):
    """Runtime workspace limit status."""
    OK = 'ok'              # all clear
    SOFT_LIMIT = 'soft'    # in soft-limit zone — speed should degrade
    HARD_LIMIT = 'hard'    # hard limit violated — trajectory must abort


@dataclass
class WorkspaceLimits:
    """Precomputed workspace limits (immutable after construction).

    Constructed once from geometry; used every control cycle.
    """
    leg_stroke_mm: float
    leg_soft_min_mm: float
    leg_soft_max_mm: float
    leg_hard_min_mm: float
    leg_hard_max_mm: float
    cond_home: float
    cond_soft: float
    cond_hard: float

    @staticmethod
    def from_geometry(geom: StewartGeometry) -> 'WorkspaceLimits':
        stroke = geom.leg_stroke_mm
        cond_home = float(np.linalg.cond(
            compute_jacobian(np.zeros(3), np.eye(3), geom)))
        return WorkspaceLimits(
            leg_stroke_mm=stroke,
            leg_soft_min_mm=LEG_SOFT_MARGIN_MM,
            leg_soft_max_mm=stroke - LEG_SOFT_MARGIN_MM,
            leg_hard_min_mm=LEG_HARD_MARGIN_MM,
            leg_hard_max_mm=stroke - LEG_HARD_MARGIN_MM,
            cond_home=cond_home,
            cond_soft=COND_SOFT_FACTOR * cond_home,
            cond_hard=COND_HARD_FACTOR * cond_home,
        )


@dataclass
class WorkspaceCheck:
    """Result of a single-cycle workspace limit check."""
    status: WorkspaceStatus
    leg_extensions_mm: np.ndarray   # (6,) current extensions
    cond_number: float
    speed_scale: float              # 1.0 if OK, <1.0 if soft, 0.0 if hard
    violations: list                # human-readable violation messages


def check_workspace_limits(extensions_mm: np.ndarray,
                           cond_number: float,
                           limits: WorkspaceLimits) -> WorkspaceCheck:
    """Runtime workspace limit check (called every control cycle).

    Parameters
    ----------
    extensions_mm : (6,) ndarray — current leg extensions
    cond_number : float — current Jacobian condition number
    limits : WorkspaceLimits — precomputed limits

    Returns
    -------
    WorkspaceCheck with status, speed_scale, and violation details.
    """
    violations = []
    status = WorkspaceStatus.OK
    speed_scale = 1.0

    # --- Leg extension limits ---
    for i in range(6):
        ext = extensions_mm[i]
        if ext < limits.leg_hard_min_mm or ext > limits.leg_hard_max_mm:
            status = WorkspaceStatus.HARD_LIMIT
            speed_scale = 0.0
            end = 'underextended' if ext < limits.leg_hard_min_mm else 'overextended'
            violations.append(f"Leg {i} HARD {end}: {ext:.1f} mm")
        elif ext < limits.leg_soft_min_mm or ext > limits.leg_soft_max_mm:
            if status != WorkspaceStatus.HARD_LIMIT:
                status = WorkspaceStatus.SOFT_LIMIT
            # Linear ramp-down: at soft boundary speed=1.0, at hard boundary speed=0.0
            margin = LEG_SOFT_MARGIN_MM - LEG_HARD_MARGIN_MM
            if margin <= 0.0:
                margin = 1.0  # Avoid division by zero; treat as hard limit
            if ext < limits.leg_soft_min_mm:
                dist = limits.leg_soft_min_mm - ext
            else:
                dist = ext - limits.leg_soft_max_mm
            leg_scale = max(0.0, 1.0 - dist / margin)
            speed_scale = min(speed_scale, leg_scale)
            end = 'underextended' if ext < limits.leg_soft_min_mm else 'overextended'
            violations.append(f"Leg {i} soft {end}: {ext:.1f} mm (scale={leg_scale:.2f})")

    # --- Condition number limits ---
    if cond_number > limits.cond_hard:
        status = WorkspaceStatus.HARD_LIMIT
        speed_scale = 0.0
        violations.append(
            f"Condition number HARD: {cond_number:.1f} > {limits.cond_hard:.1f}")
    elif cond_number > limits.cond_soft:
        if status != WorkspaceStatus.HARD_LIMIT:
            status = WorkspaceStatus.SOFT_LIMIT
        # Linear ramp-down in condition number band
        margin = limits.cond_hard - limits.cond_soft
        if margin <= 0.0:
            margin = 1.0  # Avoid division by zero; treat as hard limit
        dist = cond_number - limits.cond_soft
        cond_scale = max(0.0, 1.0 - dist / margin)
        speed_scale = min(speed_scale, cond_scale)
        violations.append(
            f"Condition number soft: {cond_number:.1f} > {limits.cond_soft:.1f} "
            f"(scale={cond_scale:.2f})")

    return WorkspaceCheck(
        status=status,
        leg_extensions_mm=extensions_mm,
        cond_number=cond_number,
        speed_scale=speed_scale,
        violations=violations,
    )


def check_leg_extensions(extensions_mm: np.ndarray,
                         geom: StewartGeometry
                         ) -> tuple[bool, np.ndarray]:
    """Check whether leg extensions are within workspace hard limits.

    Uses the same hard margins as check_workspace_limits() and
    check_feasibility() so that planning and runtime agree.

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
    hard_min = LEG_HARD_MARGIN_MM
    hard_max = geom.leg_stroke_mm - LEG_HARD_MARGIN_MM
    states = np.zeros(6, dtype=np.int8)
    states[extensions_mm < hard_min] = -1
    states[extensions_mm > hard_max] = 1
    return bool(np.all(states == 0)), states


def compute_condition_number(pos: np.ndarray,
                             rot: np.ndarray,
                             geom: StewartGeometry,
                             J: np.ndarray | None = None) -> float:
    """Condition number of the Jacobian at a given pose.

    A large condition number indicates proximity to a singularity.
    Typical good values are < 10; the ill-conditioned threshold
    from the motion planner plan is 100.

    Parameters
    ----------
    pos, rot, geom : same as ``compute_jacobian``
    J : (6,6) ndarray or None — pre-computed Jacobian (skips recomputation)
    """
    if J is None:
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
