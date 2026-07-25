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
from math import isfinite

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
#
# FK CONVERGENCE CRITERION — normative
# ------------------------------------
# ``leg_lengths_to_pose`` MUST NOT raise on a solve that has converged as far
# as double precision allows.  Its criterion therefore has three parts, and all
# three are load-bearing:
#
#   (1) a MIXED absolute/relative test, ``residual <= tol + rtol * scale``;
#   (2) a STAGNATION exit — a residual that has stopped improving *and* is
#       already physically meaningless has converged to round-off, so return;
#   (3) a GENUINE-DIVERGENCE path that still raises ``RuntimeError``.
#
# Part (3) is not optional: dropping it would trade a loud spurious failure for
# a silent wrong answer, which is strictly worse.  Losing (1) or (2) reopens the
# hardware failure below.
#
# (1) and (2) are evaluated in exactly one place — ``_fk_accepted`` — called by
# both the in-loop check and the post-loop honest-residual re-check.  Writing
# the predicate twice is how a criterion drifts: the post-loop copy is the path
# with the least coverage, so a hardening edit applied only in the loop would
# leave it accepting under a rule this block no longer describes.
#
# The threshold in (1) is derived from the CALLER'S TARGET, which makes the
# target's validity part of the criterion: ``rtol * inf`` is ``inf``, every
# ``<=`` against it is vacuously true, and FK would return its initial guess as
# a "converged" pose with an infinite residual.  ``leg_lengths_to_pose``
# therefore rejects a non-finite ``scale`` before deriving the threshold from
# it, with a ``RuntimeError`` — the same signal every caller's handler already
# routes on.  Whenever a future change makes the acceptance threshold depend on
# more of the input, validate that input the same way, at the same point.
#
# Why a relative term at all (measured 2026-07-25, plans/active/
# fk-convergence-tolerance.md Phase 0).  The residual is
# ``|leg_vector| - init_leg_length`` — a difference of two ~650-870 mm
# quantities — so it is a catastrophic-cancellation quantity whose achievable
# floor scales with the ABSOLUTE leg length ``L`` and with the conditioning of
# the Jacobian solve.  Measured over the 765-pose Phase 0 grid (cold start,
# tilt 0-15 deg, extensions 0-280 mm): floor p50 = 1.14e-13 mm,
# p95 = 4.32e-13 mm, max = 1.53e-10 mm, and ``floor <= 2.01 * eps * L *
# cond(J)`` bounded every sample.  A wider 13001-pose sweep at review time
# (tilt 0-18 deg, z 0-275 mm, |xy| <= 130 mm, incl. 4000 random poses) found a
# worse tail: p50 = 2.27e-13 mm, p95 = 3.44e-11 mm, MAX = 2.18e-10 mm, 85 poses
# above the Phase 0 max.  Quote the wider figure when sizing anything against
# "the worst floor" — the mechanism bound holds on both grids, but no grid
# bounds the tail, which is exactly why part (2) ships as a backstop.
# A FIXED absolute 1e-10 mm is therefore not reachable everywhere in the
# workspace, and whether it is reachable at a given pose is arbitrary.
# Consequences on real hardware:
#   * 37 of 10453 (0.354 %) ``/leg_setpoint_echo`` vectors from the 2026-07-25
#     15:17:48 session fail offline FK at the shipped 1e-10 default;
#   * ``trajectory_node``'s seed path refused to stream 26 times in 286 ms on
#     2026-07-24 09:08:55 (~/.ros/log/python3_198327_1784848076544.log) and
#     once on 2026-07-25 15:24:29
#     (~/.ros/log/python3_31420_1784956973167.log) — a hard "not streaming
#     until a valid state" refusal, which is a safety-relevant transition.
# The pose that was rejected in every one of those cases was accurate to
# <= 4.60e-13 mm and <= 8.20e-13 rad — 5.3e-11 of one leg encoder dead-band
# (8.607e-3 mm).  The residual was noise; the answer was exact.
#
# ``scale`` is the largest ABSOLUTE leg length implied by the *target*
# extensions, i.e. exactly the magnitude at which that cancellation happens.
# It is known before the loop and is bounded below by ``init_leg_lengths_mm``
# (~648 mm), so it cannot collapse toward zero near the stow pose — the
# low-extension failure mode a naive ``rtol * max|extension|`` would have.

#: Absolute term of the mixed convergence test (mm).  Default of the ``tol``
#: argument; kept at the historical value, so a caller passing an explicit
#: ``tol`` well above the round-off floor is unaffected on every solve that
#: converges within ``max_iter``.  (On EXHAUSTION such a caller does see a
#: change — see the post-loop re-check's note in the loop below.)
FK_DEFAULT_ATOL_MM = 1e-10

#: Relative term of the mixed convergence test, applied to the absolute leg
#: length.  At the working pose (scale ~803 mm) the whole gate is
#: 1e-10 + 1e-12 * 803 = 9.0e-10 mm — 5.9x the worst floor on the 765-pose
#: Phase 0 grid (1.53e-10 mm) and 4.1x the worst on the wider 13001-pose sweep
#: (2.18e-10 mm).  Size against the 4.1x.  Still 1e5 x tighter than the 1e-4 mm
#: the MPC hardware plant already ships as "far below anything physically
#: resolvable".  rtol=1e-13 was rejected: it gives 1.8e-10 mm, i.e. NEGATIVE
#: headroom against the wider sweep's tail, so every floor-limited solve there
#: would fall through to the stagnation exit and the "relative term handles the
#: predictable floor" guarantee would be void.
FK_DEFAULT_RTOL = 1e-12

#: Residual ceiling (mm) below which a stalled iteration counts as converged.
#: 100x below ``hardware_plant``'s shipped ``tol=1e-4`` mm and ~1.2e-4 of one
#: leg encoder dead-band, so accepting a residual at this ceiling can never be
#: a physically meaningful error — while still covering a round-off floor
#: ~4600x worse than the worst measured (2.18e-10 mm over 13001 poses; a future
#: geometry, BLAS or numpy change).  Deliberately an ABSOLUTE ceiling and not a
#: multiple of the caller's ``tol``: ``k * tol`` would give ``hardware_plant``
#: (tol=1e-4) a ceiling around 0.1 mm = 12 encoder dead-bands, so a
#: stalled-but-wrong solve there would be accepted as converged and fed to the
#: MPC's feedback loop.  The ceiling is what stops the stagnation exit from
#: swallowing a genuine divergence: an iteration stuck at a residual of
#: millimetres is above it and still raises.
FK_STALL_CEILING_MM = 1e-6

#: Minimum improvement factor per iteration.  A residual that did not shrink
#: to below ``FK_STALL_FACTOR`` x the previous iteration's has stopped making
#: progress.  Near a good root Newton gains orders of magnitude per step, and
#: at the round-off floor it gains nothing, so 0.5 sits in a very wide gap
#: between the two regimes.
FK_STALL_FACTOR = 0.5


def _residual_scale_mm(extensions_mm: np.ndarray,
                       geom: StewartGeometry) -> float:
    """Magnitude the FK residual's round-off floor scales with (mm).

    The largest absolute leg length implied by *extensions_mm* — i.e. the
    operand size of the ``|leg_vector| - init_leg_length`` cancellation inside
    :func:`pose_to_leg_lengths`.  Computed from the *target* extensions, so it
    is exact (the target IS the solution's leg lengths) and available before
    the first Newton step.
    """
    return float(np.max(np.abs(extensions_mm + geom.init_leg_lengths_mm)))


def _fk_accepted(res_max: float,
                 prev_res: float,
                 accept_mm: float,
                 stall_ceiling_mm: float,
                 stall_factor: float) -> bool:
    """THE FK acceptance criterion — the single enforcement point.

    Parts (1) and (2) of the *FK CONVERGENCE CRITERION* block above, in that
    order.  The ordering is load-bearing, not stylistic: because the mixed test
    is tried first and the stagnation exit can only fire below
    ``stall_ceiling_mm``, a caller whose ``tol`` is looser than the ceiling
    (``hardware_plant``'s ``tol=1e-4`` mm) can never reach the stagnation
    branch — the mixed test has always already accepted.  That is what makes
    the stagnation exit structurally unreachable for the 40 Hz MPC hot loop.

    *prev_res* is the residual of the pose BEFORE the most recent Newton step,
    and is ``+inf`` on the first pass, so the stagnation branch cannot fire
    until at least one step has been taken and judged.
    """
    if res_max <= accept_mm:
        return True
    return res_max <= stall_ceiling_mm and res_max > prev_res * stall_factor


def leg_lengths_to_pose(extensions_mm: np.ndarray,
                        geom: StewartGeometry,
                        initial_guess: tuple[np.ndarray, np.ndarray] | None = None,
                        tol: float = FK_DEFAULT_ATOL_MM,
                        max_iter: int = 50,
                        *,
                        rtol: float = FK_DEFAULT_RTOL,
                        stall_ceiling_mm: float = FK_STALL_CEILING_MM,
                        stall_factor: float = FK_STALL_FACTOR,
                        ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Position FK: leg extensions → platform pose (Newton-Raphson).

    Solves for the pose ``(pos, R)`` such that
    ``pose_to_leg_lengths(pos, R, geom) ≈ extensions_mm``.

    Convergence
    -----------
    See the *FK CONVERGENCE CRITERION* block above this function for the
    normative statement and the measurements behind the defaults.  A solve is
    accepted when **either**

    * ``max|residual| <= tol + rtol * scale``, where ``scale`` is the largest
      absolute leg length implied by *extensions_mm* (~648-930 mm over the
      envelope) — the mixed absolute/relative test; **or**
    * ``max|residual| <= stall_ceiling_mm`` **and** the residual failed to
      shrink below ``stall_factor`` x its previous value — the iteration has
      converged to the round-off floor and no further step can help.

    ``RuntimeError`` is raised only for a *genuine* failure: a residual still
    above both gates after *max_iter* iterations (an unsatisfiable extension
    set), or a singular Jacobian.

    Parameters
    ----------
    extensions_mm : (6,) ndarray — target leg extensions in mm
    geom : StewartGeometry
    initial_guess : optional (pos, rot) starting point.
        Defaults to active pose (zero offset, identity rotation).
    tol : **absolute** term of the convergence test (mm).  Historically the
        whole criterion; now the absolute half of a mixed test.  A caller
        passing a value well above the round-off floor (e.g.
        ``hardware_plant``'s ``tol=1e-4``) is unaffected by the relative term
        (1e-12 * 930 mm ≈ 9.3e-10 mm) and by the stagnation exit (which can
        only fire below ``stall_ceiling_mm`` = 1e-6 mm, i.e. only where the
        absolute test has already passed).
    max_iter : maximum Newton iterations
    rtol : relative term, multiplied by the absolute leg length.  Pass 0.0 for
        the historical absolute-only behaviour.
    stall_ceiling_mm : residual below which a stalled iteration is accepted as
        converged.  Pass 0.0 to disable the stagnation exit.
    stall_factor : required per-iteration improvement factor; a residual that
        did not shrink by at least this factor counts as stalled.

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
        If Newton-Raphson genuinely fails: the residual is still above both
        acceptance gates after *max_iter* iterations, or the Jacobian is
        singular, or *extensions_mm* contains a non-finite value.  The quoted
        residual is that of the pose actually being abandoned (see the note in
        the loop below).
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

    # Mixed-test threshold.  Computed once, outside the loop (~9 us): the
    # target extensions do not change, so neither does the scale.
    scale_mm = _residual_scale_mm(extensions_mm, geom)
    # Guard the THRESHOLD DERIVATION, which is the thing a non-finite target
    # breaks.  ``rtol * inf`` is ``inf``, which makes every ``<=`` below
    # vacuously true: FK would return its initial guess as a "converged" pose
    # with an infinite residual, in one iteration, silently — suppressing the
    # RuntimeError that is the only FK-failure signal any caller's handler
    # routes on (``hardware_plant``'s ``_fk_fail_count`` /
    # ``fk_convergence_failure`` e-stop, ``trajectory_node``'s refusal to
    # stream).  A non-finite ``pos_estimate`` is a live concern on this robot,
    # not hypothetical: teensy_bridge_node documents ``pos_rev is NaN until the
    # encoder is ready`` and guards it in two places.
    #
    # Checking the SCALE rather than the extensions costs one scalar test
    # (~0.06 us) instead of an array scan (~7 us) and allocates nothing, which
    # matters on the 40 Hz path — and it is exactly as strict: the scale is
    # ``max|ext + init_leg_lengths|``, so any inf in ``extensions_mm`` makes it
    # inf and any NaN makes it NaN.  It additionally catches a finite-but-
    # overflowing target, which is equally unusable.
    if not isfinite(scale_mm):
        leg_lengths_to_pose.last_iterations = 0
        raise RuntimeError(
            "FK failed: non-finite target extensions "
            f"(max|ext + init_leg_length| = {scale_mm})")
    accept_mm = tol + rtol * scale_mm
    prev_res = np.inf

    for iteration in range(max_iter):
        # Residual: actual − target leg extensions
        residual = pose_to_leg_lengths(pos_cur, rot_cur, geom) - extensions_mm
        res_max = np.max(np.abs(residual))

        # THE criterion — parts (1) and (2), in `_fk_accepted` so that this
        # check and the post-loop one cannot drift apart.  `<=` not `<` inside
        # it: at the round-off floor the residual can land exactly on the
        # threshold.
        if _fk_accepted(res_max, prev_res, accept_mm,
                        stall_ceiling_mm, stall_factor):
            # Compute Jacobian at the converged pose (not a stale one
            # from a prior iteration) so callers get the correct J.
            J = compute_jacobian(pos_cur, rot_cur, geom)
            leg_lengths_to_pose.last_iterations = iteration + 1
            return pos_cur, rot_cur, J
        prev_res = res_max

        J = compute_jacobian(pos_cur, rot_cur, geom)
        # Newton step: δx = -J⁻¹ · residual
        try:
            dx = np.linalg.solve(J, -residual)
        except np.linalg.LinAlgError:
            leg_lengths_to_pose.last_iterations = iteration + 1
            raise RuntimeError("FK failed: singular Jacobian at current pose")

        # Apply update: translate position, compose incremental rotation
        pos_cur += dx[:3]
        if norm(dx[3:]) > 1e-14:
            rot_cur = rotvec_to_rot_matrix(dx[3:]) @ rot_cur

    # The loop body evaluates the residual BEFORE stepping, so on exhaustion
    # the pose we are holding has been advanced once more than the last
    # residual we measured — its own residual has never been evaluated.  Two
    # consequences, both fixed here for the price of one cheap IK call on the
    # failure path only:
    #   * that final step's work would otherwise be computed and discarded;
    #   * the residual quoted in the error would be the one from BEFORE the
    #     step, which over-states the abandoned pose's actual residual by up
    #     to 3.00x (measured over the Phase 0 sweep).  A safety-relevant error
    #     message must describe the pose it is actually rejecting.
    #
    # NOTE this DOES change behaviour on exhaustion for every caller, including
    # ``hardware_plant``'s ``tol=1e-4`` one: a solve whose final step lands
    # within the caller's own tolerance now returns instead of raising.  For
    # that caller the practical effect is that such a tick no longer counts
    # toward ``_FK_FAIL_ESTOP_THRESHOLD`` and no longer falls back to the
    # previous tick's pose — it gets a fresh pose that satisfies the tolerance
    # it asked for, which is the more correct feedback and strictly fresher.
    # Pinned by ``test_exhaustion_returns_when_the_final_step_converges``.
    residual = pose_to_leg_lengths(pos_cur, rot_cur, geom) - extensions_mm
    res_max = np.max(np.abs(residual))
    leg_lengths_to_pose.last_iterations = max_iter
    if _fk_accepted(res_max, prev_res, accept_mm,
                    stall_ceiling_mm, stall_factor):
        return pos_cur, rot_cur, compute_jacobian(pos_cur, rot_cur, geom)
    raise RuntimeError(
        f"FK did not converge after {max_iter} iterations "
        f"(max residual: {res_max:.2e} mm, "
        f"acceptance threshold: {accept_mm:.2e} mm)"
    )

# Diagnostic: iteration count from last successful call (zero-cost when unread)
leg_lengths_to_pose.last_iterations = 0
