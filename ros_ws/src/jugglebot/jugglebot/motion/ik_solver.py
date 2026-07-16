"""Stewart platform kinematic model — IK, Jacobian, FK.

Provides all kinematic computations needed for the motion planner:
  - Position IK:      pose → leg extensions (mm)
  - Velocity IK:      twist → leg velocities via Jacobian
  - Acceleration IK:  accel → leg accelerations via J and J̇
  - Position FK:      leg extensions → pose (numerical Newton-Raphson)

Pure Python + numpy.  No ROS2 dependency.

Coordinate conventions
----------------------
- **Base frame**: origin at base centre, z = 0 at base plane, +z up.
- **Platform frame** (body-fixed): origin at platform centre, aligned with
  the base frame when the platform is at its initial (active) pose.
- **Pose**: the *offset* of the platform centre from the active position,
  expressed in the base frame.  At active the offset is [0, 0, 0].
  The absolute platform centre is therefore ``pos + [0, 0, init_height]``.
- **Rotation matrix** ``R``: rotates vectors from the platform frame into
  the base frame.  Identity at active.
- **Twist**: ``[vx, vy, vz, ωx, ωy, ωz]`` in the base frame (mm/s, rad/s).
- **Leg extensions**: 0 mm = fully retracted, ``leg_stroke_mm`` = fully
  extended.  Positive extension → leg gets longer.

Jacobian convention
-------------------
``J`` (6×6) maps a platform twist to leg extension rates::

    q̇ = J · [v; ω]

Row *i* corresponds to leg *i*.  Columns 0-2 are the translational
sensitivity (dimensionless unit leg directions) and columns 3-5 are the
rotational sensitivity (mm / rad).
"""

from __future__ import annotations

import logging

import numpy as np
from numpy.linalg import norm

logger = logging.getLogger(__name__)

from jugglebot.motion.geometry import StewartGeometry

# ---------------------------------------------------------------------------
# Rotation utilities
# ---------------------------------------------------------------------------

def skew(v: np.ndarray) -> np.ndarray:
    """Skew-symmetric matrix of a 3-vector.  ``skew(v) @ u == cross(v, u)``."""
    return np.array([[0.0,  -v[2],  v[1]],
                     [v[2],  0.0,  -v[0]],
                     [-v[1], v[0],  0.0]])


def quat_to_rot_matrix(w: float, x: float, y: float, z: float) -> np.ndarray:
    """Quaternion (w, x, y, z) → 3×3 rotation matrix."""
    # Normalise to guard against drift
    n = np.sqrt(w*w + x*x + y*y + z*z)
    if n < 1e-12:
        raise ValueError(f"Zero or near-zero quaternion: ({w}, {x}, {y}, {z})")
    w, x, y, z = w/n, x/n, y/n, z/n

    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


def rot_matrix_to_quat(R: np.ndarray) -> tuple[float, float, float, float]:
    """3×3 rotation matrix → quaternion (w, x, y, z).

    Uses Shepperd's method for numerical stability.
    """
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return (w, x, y, z)


def rotvec_to_rot_matrix(rotvec: np.ndarray) -> np.ndarray:
    """Rotation vector (axis × angle in radians) → 3×3 rotation matrix.

    Uses the Rodrigues formula.
    """
    angle = norm(rotvec)
    if angle < 1e-12:
        return np.eye(3)
    k = rotvec / angle
    K = skew(k)
    return np.eye(3) + np.sin(angle) * K + (1.0 - np.cos(angle)) * (K @ K)


def rot_matrix_to_rotvec(R: np.ndarray) -> np.ndarray:
    """3×3 rotation matrix → rotation vector (axis × angle in radians)."""
    angle = np.arccos(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))
    if angle < 1e-8:
        return np.zeros(3)
    sin_angle = np.sin(angle)
    if abs(sin_angle) < 1e-8:
        # angle ≈ π — numerically unstable axis extraction.
        # Should never occur (platform tilt limited to 15°).
        logger.error("rot_matrix_to_rotvec: angle ≈ π (%.4f rad) — "
                     "axis extraction unstable, returning zeros", angle)
        return np.zeros(3)
    # Extract axis from skew-symmetric part of R
    axis = np.array([R[2, 1] - R[1, 2],
                     R[0, 2] - R[2, 0],
                     R[1, 0] - R[0, 1]]) / (2.0 * sin_angle)
    return axis * angle


# ---------------------------------------------------------------------------
# Position IK
# ---------------------------------------------------------------------------

def compute_leg_vectors(pos: np.ndarray,
                        rot: np.ndarray,
                        geom: StewartGeometry) -> np.ndarray:
    """Compute the 6 leg vectors from base nodes to platform nodes.

    Parameters
    ----------
    pos : (3,) ndarray — platform offset from stow pose [x, y, z] in mm
    rot : (3,3) ndarray — rotation matrix (platform → base)
    geom : StewartGeometry

    Returns
    -------
    leg_vectors : (6, 3) ndarray — L_i vectors in the base frame
    """
    # Platform node positions in world (base) frame
    # plat_world[i] = (pos + init_height) + R @ plat_nodes[i]
    platform_centre = pos + geom.init_height_vec  # (3,)
    plat_world = platform_centre + (rot @ geom.plat_nodes.T).T  # (6, 3)

    return plat_world - geom.base_nodes  # (6, 3)


def pose_to_leg_lengths(pos: np.ndarray,
                        rot: np.ndarray,
                        geom: StewartGeometry) -> np.ndarray:
    """Position IK: platform pose → leg extensions in mm.

    Parameters
    ----------
    pos : (3,) ndarray — platform offset from stow pose [x, y, z] in mm
    rot : (3,3) ndarray — rotation matrix (platform → base)
    geom : StewartGeometry

    Returns
    -------
    extensions_mm : (6,) ndarray — leg extensions, 0 = retracted.
        Values may be outside [0, stroke] — caller must check bounds.
    """
    leg_vecs = compute_leg_vectors(pos, rot, geom)
    abs_lengths = norm(leg_vecs, axis=1)
    return abs_lengths - geom.init_leg_lengths_mm


# ---------------------------------------------------------------------------
# Jacobian
# ---------------------------------------------------------------------------

def compute_jacobian(pos: np.ndarray,
                     rot: np.ndarray,
                     geom: StewartGeometry) -> np.ndarray:
    """Analytical Jacobian: maps platform twist → leg extension rates.

    ``q̇ = J @ [vx, vy, vz, ωx, ωy, ωz]``

    Parameters
    ----------
    pos : (3,) ndarray — platform offset from stow pose
    rot : (3,3) ndarray — rotation matrix (platform → base)
    geom : StewartGeometry

    Returns
    -------
    J : (6, 6) ndarray
    """
    leg_vecs = compute_leg_vectors(pos, rot, geom)              # (6, 3)

    # Unit leg directions and platform-node lever arms for all six legs at once.
    # (rot @ plat_nodes.T).T[i] == rot @ plat_nodes[i], so this is row-for-row the
    # per-leg `a_i_world` the loop form computed — just batched.
    l = leg_vecs / norm(leg_vecs, axis=1, keepdims=True)        # (6, 3) unit
    a_world = (rot @ geom.plat_nodes.T).T                       # (6, 3)

    J = np.empty((6, 6))
    J[:, :3] = l
    # Rotational columns are the row-wise cross product a_world × l. This used to be
    # `np.cross(a_i_world, l_i)` per leg, but np.cross routes through numpy's generic
    # __array_function__ / moveaxis dispatch, which profiled as ~68 % of the analytic
    # validate() wall time on the Jetson (three compute_jacobian calls per sample ×
    # ~dense samples/segment). The explicit component form below is arithmetic-only
    # (no dispatch) and numerically identical: bit-equal to np.cross on the same
    # batched operands, and ≤2.84e-14 (~1 ulp, from the batched matmul above) vs the
    # previous per-leg loop (measured 2026-07-16); it cut unshaped build_move
    # 736→234 ms and shaped 2680→1183 ms.
    ax, ay, az = a_world[:, 0], a_world[:, 1], a_world[:, 2]
    lx, ly, lz = l[:, 0], l[:, 1], l[:, 2]
    J[:, 3] = ay * lz - az * ly
    J[:, 4] = az * lx - ax * lz
    J[:, 5] = ax * ly - ay * lx

    return J


# ---------------------------------------------------------------------------
# Velocity IK
# ---------------------------------------------------------------------------

def twist_to_leg_velocities(twist: np.ndarray,
                            pos: np.ndarray,
                            rot: np.ndarray,
                            geom: StewartGeometry,
                            J: np.ndarray | None = None) -> np.ndarray:
    """Velocity IK: platform twist → leg extension rates (mm/s).

    Parameters
    ----------
    twist : (6,) ndarray — [vx, vy, vz, ωx, ωy, ωz] in base frame (mm/s, rad/s)
    pos, rot, geom : same as ``compute_jacobian``
    J : (6,6) ndarray or None — pre-computed Jacobian (skips recomputation)

    Returns
    -------
    q_dot : (6,) ndarray — leg extension velocities in mm/s
    """
    if J is None:
        J = compute_jacobian(pos, rot, geom)
    return J @ twist


# ---------------------------------------------------------------------------
# Jacobian time-derivative (numerical)
# ---------------------------------------------------------------------------

def _advance_pose(pos: np.ndarray,
                  rot: np.ndarray,
                  twist: np.ndarray,
                  dt: float) -> tuple[np.ndarray, np.ndarray]:
    """Advance a pose by a twist over a small timestep dt.

    Used internally for numerical differentiation of J.
    """
    v = twist[:3]
    omega = twist[3:]

    pos_new = pos + v * dt
    dR = rotvec_to_rot_matrix(omega * dt)
    rot_new = dR @ rot

    return pos_new, rot_new


def compute_jacobian_dot(pos: np.ndarray,
                         rot: np.ndarray,
                         twist: np.ndarray,
                         geom: StewartGeometry,
                         dt: float = 1e-7) -> np.ndarray:
    """Time-derivative of the Jacobian via central finite difference.

    J̇ ≈ (J(pose + twist·dt) − J(pose − twist·dt)) / (2·dt)

    Central difference is O(dt²) accurate vs O(dt) for forward difference.

    Parameters
    ----------
    pos : (3,) ndarray
    rot : (3,3) ndarray
    twist : (6,) ndarray — current platform twist
    geom : StewartGeometry
    dt : float — finite-difference half-step (default 1e-7 s)

    Returns
    -------
    J_dot : (6, 6) ndarray
    """
    pos_fwd, rot_fwd = _advance_pose(pos, rot, twist, dt)
    pos_bwd, rot_bwd = _advance_pose(pos, rot, twist, -dt)
    J_fwd = compute_jacobian(pos_fwd, rot_fwd, geom)
    J_bwd = compute_jacobian(pos_bwd, rot_bwd, geom)
    return (J_fwd - J_bwd) / (2.0 * dt)


# ---------------------------------------------------------------------------
# Acceleration IK
# ---------------------------------------------------------------------------

def accel_to_leg_accels(accel: np.ndarray,
                        twist: np.ndarray,
                        pos: np.ndarray,
                        rot: np.ndarray,
                        geom: StewartGeometry,
                        J: np.ndarray | None = None) -> np.ndarray:
    """Acceleration IK: platform acceleration → leg extension accelerations.

    ``q̈ = J·ẍ + J̇·ẋ``

    Parameters
    ----------
    accel : (6,) ndarray — platform acceleration [ax, ay, az, αx, αy, αz]
                           (mm/s², rad/s²)
    twist : (6,) ndarray — current platform twist (mm/s, rad/s)
    pos, rot, geom : same as ``compute_jacobian``
    J : (6,6) ndarray or None — pre-computed Jacobian (skips recomputation)

    Returns
    -------
    q_ddot : (6,) ndarray — leg extension accelerations in mm/s²
    """
    if J is None:
        J = compute_jacobian(pos, rot, geom)
    J_dot = compute_jacobian_dot(pos, rot, twist, geom)
    return J @ accel + J_dot @ twist


# ---------------------------------------------------------------------------
# Forward kinematics (numerical Newton-Raphson)
# ---------------------------------------------------------------------------

def leg_lengths_to_pose(extensions_mm: np.ndarray,
                        geom: StewartGeometry,
                        initial_guess: tuple[np.ndarray, np.ndarray] | None = None,
                        tol: float = 1e-10,
                        max_iter: int = 50) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Position FK: leg extensions → platform pose (Newton-Raphson).

    Solves for the pose ``(pos, R)`` such that
    ``pose_to_leg_lengths(pos, R, geom) ≈ extensions_mm``.

    Parameters
    ----------
    extensions_mm : (6,) ndarray — target leg extensions in mm
    geom : StewartGeometry
    initial_guess : optional (pos, rot) starting point.
        Defaults to active pose (zero offset, identity rotation).
    tol : convergence tolerance on max residual (mm)
    max_iter : maximum Newton iterations

    Returns
    -------
    pos : (3,) ndarray — platform offset from stow pose
    rot : (3, 3) ndarray — rotation matrix (platform → base)
    J : (6, 6) ndarray — Jacobian at the converged pose (from the last
        Newton iteration).  Callers can reuse this to avoid recomputing
        the Jacobian for twist/force decomposition.

    Raises
    ------
    RuntimeError
        If Newton-Raphson does not converge within *max_iter* iterations.
    """
    # Work directly with (pos, rot_matrix) to avoid rotvec ↔ rot_matrix
    # round-trips on each iteration.  Newton updates are applied as:
    #   pos += dx[:3],  rot = rotvec_to_rot_matrix(dx[3:]) @ rot
    if initial_guess is not None:
        pos_cur = initial_guess[0].copy()
        rot_cur = initial_guess[1].copy()
    else:
        pos_cur = np.zeros(3)
        rot_cur = np.eye(3)

    for iteration in range(max_iter):
        # Residual: actual − target leg extensions
        residual = pose_to_leg_lengths(pos_cur, rot_cur, geom) - extensions_mm

        if np.max(np.abs(residual)) < tol:
            # Compute Jacobian at the converged pose (not a stale one
            # from a prior iteration) so callers get the correct J.
            J = compute_jacobian(pos_cur, rot_cur, geom)
            leg_lengths_to_pose.last_iterations = iteration + 1
            return pos_cur, rot_cur, J

        J = compute_jacobian(pos_cur, rot_cur, geom)
        # Newton step: δx = -J⁻¹ · residual
        try:
            dx = np.linalg.solve(J, -residual)
        except np.linalg.LinAlgError:
            raise RuntimeError("FK failed: singular Jacobian at current pose")

        # Apply update: translate position, compose incremental rotation
        pos_cur += dx[:3]
        if norm(dx[3:]) > 1e-14:
            rot_cur = rotvec_to_rot_matrix(dx[3:]) @ rot_cur

    leg_lengths_to_pose.last_iterations = max_iter
    raise RuntimeError(
        f"FK did not converge after {max_iter} iterations "
        f"(max residual: {np.max(np.abs(residual)):.2e} mm)"
    )

# Diagnostic: iteration count from last successful call (zero-cost when unread)
leg_lengths_to_pose.last_iterations = 0
