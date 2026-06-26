"""Offline CasADi optimiser for the periodic juggle-demo platform trajectory.

Solves, once, an NLP for the steady-state platform pose trajectory that
satisfies the throw and catch keyframe constraints. The default
objective is **leg-space jerk² + leg-velocity minimax + soft catch
colinearity** (cuts #4 + #5, 2026-05-24). Pose-jerk² is preserved as
an opt-in regulariser. The runtime player loads the result (a
:class:`JuggleTrajectory`) and plays it at 40 Hz with no solver in the
loop.

Discretisation
--------------
N quintic-Hermite knots over one period. Each knot stores
``(pose, twist, accel)`` — 18 floats; the segment between two knots is a
quintic-Hermite polynomial that is C2 across knot boundaries by
construction. The per-segment integral of squared jerk is computed
analytically (see :func:`controller.hermite.quintic_jerk_integral`), so
the objective is a low-degree polynomial in the decision variables —
IPOPT converges quickly.

Periodicity is enforced by indexing: segment ``k`` connects knot ``k`` to
knot ``(k+1) % N``; there are only ``N`` distinct knots, so the
boundary state at the period wrap is identical to knot 0 by definition.

Constraints
-----------
**Throw instant** (``t_rel == 0``, knot 0):

  - Platform centroid xyz equals ``pattern.throw_pose[:3]``; orientation
    (rx, ry, rz) is free, subject to the per-knot tilt cap.
  - Orientation-aware ball-release equality: the ball's release velocity
    ``v_ball = twist[0,:3] + R_throw @ [0, 0, throw_speed_mms]`` is set
    equal to the inverse-ballistic ``v_required = (catch_target −
    release_pos) / flight + [0, 0, 0.5 · g · flight]``. ``release_pos``
    and ``catch_target`` are the hand-opening *world* positions, derived
    from the platform's throw and catch orientations respectively — so
    a tilted platform shifts both endpoints and the optimiser may bank
    the throw to give the hand-axis a horizontal kick.

**Catch instant** (``t_rel == catch_offset_s``, mid-segment in general):

  - Platform centroid xyz equals ``pattern.catch_pose[:3]``.
  - Orientation, twist, and acceleration at catch are all free — the
    optimiser may lean into or out of the arriving ball, and may move
    its centroid at any velocity.

**Tilt** — every knot satisfies ``rx² + ry² ≤ tilt_limit_rad²`` when
``bank_locked_level=False`` (the default). With ``bank_locked_level=True``
the orientation DOFs are pinned to zero everywhere (opt-in legacy mode,
useful for A/B comparison against the level-locked first cut).

**Periodic** — built into the wraparound, no explicit constraint.

**Workspace** — simple xy and z box bounds on the pose at every knot.

**Leg velocity / acceleration limits** — *not* enforced in this cut;
those tie into the Phase 4 ``hardware_config.yaml`` raise and are added
in a later cut alongside a leg-space jerk objective and leg-velocity
equalisation.

Catch offset
------------
The matching rule is ``throw[k] ↔ catch[k+1]``: a ball thrown at throw
event ``k`` is caught at catch event ``k+1`` (the next catch event after
the throw, one platform-period later). For that to be ballistically
self-consistent::

    catch_offset_s = flight_time_s - platform_period_s.

For the default :class:`JugglePattern` (apex 1.3 m), this is
``1.03 − 0.625 ≈ 0.405 s``. The optimiser default uses this value;
``cfg.catch_offset_s`` overrides it.

Pure CasADi + NumPy. No ROS2, no MuJoCo, no hardware. Run offline on
any machine.
"""
from __future__ import annotations

import dataclasses
from pathlib import Path
from typing import Optional

import casadi as cs
import numpy as np

from controller.ballistics import TILT_LIMIT_RAD
from controller.demo.pattern import JugglePattern
from controller.demo.trajectory import JuggleTrajectory, build_analytic_oval
from controller.hermite import quintic_interp_with_accel
from jugglebot.motion.geometry import StewartGeometry

# Hand-stroke kinematics constants (mirroring sim/hand/trajectory.py
# and sim/hand/ballistics.py). Lifted here so the optimiser stays inside
# the controller/ boundary (no sim/ imports). These describe the
# hardware hand actuator and are not sim-specific.
_HAND_STROKE_MARGIN_MM = 20.0
_HAND_TOTAL_STROKE_MM = 315.0
_HAND_AXIS_BOTTOM_OFFSET_MM = -129.0   # from sim/hand/ballistics.py:22
_HAND_BALL_SEAT_OFFSET_MM = 44.4       # from sim/hand/ballistics.py:25
_HAND_INERTIA_RATIO = 0.747            # from sim/hand/trajectory.py:35
_HAND_THROW_VEL_HOLD_PCT = 0.05        # from sim/hand/trajectory.py:48
_HAND_CATCH_VEL_HOLD_PCT = 0.10        # from sim/hand/trajectory.py:37


def _hand_offset_at_release_mm(throw_speed_mps: float) -> float:
    """Platform-centroid → ball-COM offset (mm) at the throw release instant.

    Mirrors ``HandThrowTrajectory`` at ``t == release_time`` (end of the
    velocity-hold phase): slider position = stroke_margin + x1 +
    v_throw * t_vel where x1 is the acceleration-phase displacement.
    The ball offset is the slider position + the fixed seat-offset
    geometry. Independent of pose; depends only on ``throw_speed_mps``
    and the constants above.
    """
    accel_stroke_m = (1.0 - _HAND_THROW_VEL_HOLD_PCT) * (_HAND_TOTAL_STROKE_MM / 1000.0)
    vel_hold_m = _HAND_THROW_VEL_HOLD_PCT * (_HAND_TOTAL_STROKE_MM / 1000.0)
    t_acc = 2.0 / (_HAND_INERTIA_RATIO + 1.0) * accel_stroke_m / throw_speed_mps
    t_vel = vel_hold_m / throw_speed_mps
    throwA = throw_speed_mps / t_acc
    x1_m = 0.5 * throwA * t_acc * t_acc
    release_slider_mm = _HAND_STROKE_MARGIN_MM + (x1_m + throw_speed_mps * t_vel) * 1000.0
    return _HAND_AXIS_BOTTOM_OFFSET_MM + release_slider_mm + _HAND_BALL_SEAT_OFFSET_MM


def _hand_offset_at_arrival_mm() -> float:
    """Platform-centroid → ball-COM offset (mm) at the catch arrival instant.

    Independent of throw_speed (the catch-vel ratio cancels in the
    midpoint-of-velocity-hold formula). The slider descent from the
    top of stroke (335 mm) to arrival is ~137 mm for the standard hand
    kinematics, putting the slider at ~198 mm.
    """
    descent_m = 0.5 * (_HAND_TOTAL_STROKE_MM / 1000.0) * (
        2.0 * _HAND_INERTIA_RATIO / (_HAND_INERTIA_RATIO + 1.0)
        * (1.0 - _HAND_CATCH_VEL_HOLD_PCT)
        + _HAND_CATCH_VEL_HOLD_PCT
    )
    top_of_stroke_mm = _HAND_STROKE_MARGIN_MM + _HAND_TOTAL_STROKE_MM
    arrival_slider_mm = top_of_stroke_mm - descent_m * 1000.0
    return _HAND_AXIS_BOTTOM_OFFSET_MM + arrival_slider_mm + _HAND_BALL_SEAT_OFFSET_MM


@dataclasses.dataclass
class OptimizerConfig:
    """Parameters for :func:`optimise_juggle_trajectory`.

    Attributes
    ----------
    n_knots : int
        Number of quintic-Hermite knots per period. ``12`` is the
        empirical sweet spot for the leg-jerk² objective — coarse
        enough that IPOPT converges quickly (~40-65 iter, ~12 s
        wall), fine enough that the spline + dense sub-grid captures
        the trajectory's smoothness budget.
    n_samples : int
        Density of the resampled :class:`JuggleTrajectory` returned.
        Independent of ``n_knots`` — the resample uses the same
        quintic-Hermite segments the optimiser solved for, so dense
        resampling is exact (no approximation).
    bank_locked_level : bool
        If True, pins ``rx == ry == rz == 0`` and zero rotational
        twist/accel at every knot (opt-in legacy mode, useful for A/B
        comparison against the level-locked first cut). Default
        ``False`` — the optimiser banks the platform within
        ``tilt_limit_rad`` to reduce leg-jerk² and exploit the hand-
        axis tilt-induced horizontal velocity contribution at throw.
    catch_offset_s : float or None
        Catch event time within the period. ``None`` derives it from
        the pattern's ballistics (``flight_time_s − platform_period_s``).
    workspace_xy_mm : float
        Absolute box bound on platform x and y at every knot, mm.
    workspace_z_mm : tuple of (float, float)
        Lower / upper bound on platform z at every knot, mm.
    """
    n_knots: int = 12
    n_samples: int = 128
    bank_locked_level: bool = False
    tilt_limit_rad: float = TILT_LIMIT_RAD
    catch_offset_s: Optional[float] = None
    workspace_xy_mm: float = 200.0
    workspace_z_mm: tuple[float, float] = (0.0, 300.0)
    # ---- Objective weights ------------------------------------------
    # Leg-space jerk² is the primary smoothness term — penalises
    # actuator-current jerk via finite differences on the dense sub-
    # sampled IK output. Captures what the motors actually feel.
    # Default 1.0 (primary).
    leg_jerk_weight: float = 1.0
    # Pose-space jerk² was the Phase 2 cut #1 objective; preserved as
    # an optional regulariser. With banking + level-locked off, the
    # mixed-unit rotation (rad/s³) vs translation (mm/s³) made it
    # ~1e6× insensitive to orientation jerk, producing degenerate
    # constant-tilt solutions at low apex. Default 0.0 (off).
    pose_jerk_weight: float = 0.0
    # ---- Leg-velocity equalisation (cut #5) -------------------------
    # Number of sub-samples per knot segment for the dense leg-space
    # FD computation. Total sub-grid = ``n_knots * n_sub_per_segment``.
    # 4 is the empirical sweet spot for the default pattern (64 total
    # sub-points at N=16); raise for tighter jerk integration, lower
    # to speed up IPOPT.
    n_sub_per_segment: int = 4
    # Minimax slack: if ``leg_vel_equalize_weight > 0``, the NLP gains
    # a scalar ``v_max`` variable with constraints ``|leg_vel_i(t_m)|
    # ≤ v_max`` for every leg and sub-sample, and the objective gains
    # ``leg_vel_equalize_weight × v_max``. Drives ALL six legs to
    # share peak velocity (the "no leg works much harder than the
    # others" criterion). Default 0.01 (gentle nudge — strong enough
    # to equalise without dominating the leg-jerk² primary term).
    leg_vel_equalize_weight: float = 0.01
    # Hard cap on |leg_vel_i(t_m)| (mm/s); enforced as an inequality
    # at every sub-sample. ``None`` (default) disables. Useful when
    # the actuators have a known velocity ceiling and the optimiser
    # should respect it directly. Independent of the minimax slack.
    leg_vel_hard_cap_mms: Optional[float] = None
    # Soft penalty weight for catch colinearity — the hand axis at catch
    # is "encouraged" (not forced) to lie along the ball's arrival
    # velocity direction. Penalty term added to the objective:
    #   weight * ||v_ball_arrival × (R_catch @ [0,0,1])||²
    # Scale: pose-jerk² ≈ 1e9 for the default pattern; a 5° catch
    # mis-alignment at v ≈ 5000 mm/s gives ||v×ẑ||² ≈ 1.9e5, so the
    # weighting needed to be comparable is ~1e3–1e4. Default 1000 is
    # a gentle nudge that costs ~0.2% of pose-jerk for 5° miss; raise
    # for a tighter alignment, set to 0.0 to disable.
    catch_colinearity_weight: float = 1000.0
    # World-frame z of the platform centroid at the STOW pose, mm. The
    # optimiser works in STOW-relative pose coordinates but the ballistic
    # constraints need world frame; the only quantity that bridges the
    # frames is this offset. Default matches the StewartGeometry default
    # (574.3 mm); override via the caller for non-default geometries.
    platform_init_height_mm: float = 574.3
    # Soft penalty weight for THROW facing — mirror of the catch term.
    # The throw's HARD invariant is hand velocity == required launch
    # velocity (a resultant of platform twist + axial slider ejection +
    # tilt); this soft term only shapes the orientation so the hand axis
    # "faces" along the launch velocity. 'Facing soft' — never overrides
    # the hard velocity match. Default 1000 (matches catch); 0.0 disables.
    throw_colinearity_weight: float = 1000.0
    # Catch velocity-match target: the TOTAL hand-opening (cup) velocity
    # at the catch event is constrained colinear with the ball arrival
    # velocity and at this fraction of its speed. 0.7 ⇒ the cup moves at
    # 70% of ball speed in the ball's direction, leaving a 30% closing
    # velocity so the ball seats into the cup (a full 1.0 match gives zero
    # relative velocity → the ball never closes in → off-centre rim
    # contact). This is the user's "hand velocity colinear with ball
    # velocity, at 70%" invariant — the HARD constraint at catch.
    catch_vel_ratio: float = 0.7
    # The hand SLIDER's own descent speed as a fraction of the vertical
    # launch speed (throw_speed_mps, mirroring the throw stroke; ≈ the ball
    # arrival speed to within the small horizontal separation component) —
    # an internal detail of how the cup velocity is produced (slider +
    # platform twist = cup velocity). MUST match the sim hand model
    # (sim/hand/trajectory.py CATCH_VEL_RATIO) and hardware_config.yaml
    # teensy_trajectory.catch_vel_ratio (both 0.6) so the optimiser's
    # cup-velocity model matches what the sim/hardware hand actually does;
    # the platform twist supplies the remainder (cup_ratio − slider_ratio).
    # Drift risk noted; single-source contract owned by the hand overhaul.
    catch_slider_vel_ratio: float = 0.6


@dataclasses.dataclass
class OptimizerResult:
    """Return value of :func:`optimise_juggle_trajectory`.

    Attributes
    ----------
    trajectory : JuggleTrajectory
        The resampled periodic trajectory, ready for the runtime player.
    catch_offset_s : float
        The catch offset the NLP was solved for (in case it was derived).
    objective_value : float
        Final value of the composite objective at convergence
        (weighted sum: leg-jerk² + optional pose-jerk² + minimax
        ``v_max`` + catch-colinearity² penalty). Magnitude depends on
        the weights set in ``OptimizerConfig``; useful for comparing
        runs that share the same weighting and as a regression sanity
        check.
    knot_poses, knot_twists, knot_accels : np.ndarray
        Raw NLP knot decision values, shape ``(n_knots, 6)``.
    n_iter : int
        IPOPT iteration count (informational).
    """
    trajectory: JuggleTrajectory
    catch_offset_s: float
    objective_value: float
    knot_poses: np.ndarray
    knot_twists: np.ndarray
    knot_accels: np.ndarray
    n_iter: int


# --------------------------------------------------------------------------
# Symbolic quintic Hermite — CasADi mirrors of controller.hermite
# --------------------------------------------------------------------------

def _quintic_eval_sym(p0, v0, a0, p1, v1, a1, T, t):
    """CasADi-symbolic quintic Hermite eval mirroring
    :func:`controller.hermite.quintic_interp_with_accel`.

    Inputs are 1x6 ``cs.MX`` row expressions (or numpy arrays of length 6).
    Returns ``(pose, twist, accel)`` each a 1x6 ``cs.MX`` row.
    """
    s = t / T
    V0 = v0 * T
    V1 = v1 * T
    A0 = a0 * T * T
    A1 = a1 * T * T

    s2 = s * s
    s3 = s2 * s
    s4 = s3 * s
    s5 = s4 * s

    h0 = 1 - 10 * s3 + 15 * s4 - 6 * s5
    h1 = s - 6 * s3 + 8 * s4 - 3 * s5
    h2 = 0.5 * s2 - 1.5 * s3 + 1.5 * s4 - 0.5 * s5
    h3 = 10 * s3 - 15 * s4 + 6 * s5
    h4 = -4 * s3 + 7 * s4 - 3 * s5
    h5 = 0.5 * s3 - s4 + 0.5 * s5
    pose = h0 * p0 + h1 * V0 + h2 * A0 + h3 * p1 + h4 * V1 + h5 * A1

    inv_T = 1.0 / T
    dh0 = (-30 * s2 + 60 * s3 - 30 * s4) * inv_T
    dh1 = (1 - 18 * s2 + 32 * s3 - 15 * s4) * inv_T
    dh2 = (s - 4.5 * s2 + 6 * s3 - 2.5 * s4) * inv_T
    dh3 = (30 * s2 - 60 * s3 + 30 * s4) * inv_T
    dh4 = (-12 * s2 + 28 * s3 - 15 * s4) * inv_T
    dh5 = (1.5 * s2 - 4 * s3 + 2.5 * s4) * inv_T
    twist = dh0 * p0 + dh1 * V0 + dh2 * A0 + dh3 * p1 + dh4 * V1 + dh5 * A1

    inv_T2 = inv_T * inv_T
    ddh0 = (-60 * s + 180 * s2 - 120 * s3) * inv_T2
    ddh1 = (-36 * s + 96 * s2 - 60 * s3) * inv_T2
    ddh2 = (1 - 9 * s + 18 * s2 - 10 * s3) * inv_T2
    ddh3 = (60 * s - 180 * s2 + 120 * s3) * inv_T2
    ddh4 = (-24 * s + 84 * s2 - 60 * s3) * inv_T2
    ddh5 = (3 * s - 12 * s2 + 10 * s3) * inv_T2
    accel = ddh0 * p0 + ddh1 * V0 + ddh2 * A0 + ddh3 * p1 + ddh4 * V1 + ddh5 * A1

    return pose, twist, accel


def _casadi_rodrigues(rv):
    """CasADi-symbolic rotation matrix from a 3-vector rotation (Rodrigues).

    ``rv`` is a 1x3 or 3x1 ``cs.MX`` rotation vector (axis × angle, radians).
    Returns a 3x3 rotation matrix. Uses eps-padded denominators
    (``theta_sq + 1e-30``) so ``sin(θ)/θ`` and ``(1 − cos θ)/θ²``
    evaluate to their limits (1 and ½) at ``θ = 0``; the symbolic
    Jacobian stays finite there.

    Mirrors ``controller.mpc._casadi_rodrigues`` but inlined here to
    keep the optimiser's import graph self-contained.
    """
    # Normalise rv to a column vector for indexing.
    rv = cs.reshape(rv, 3, 1)
    rx = rv[0]
    ry = rv[1]
    rz = rv[2]
    theta_sq_raw = rx * rx + ry * ry + rz * rz
    # eps padding used in BOTH theta and theta_sq denominators so the
    # division is finite at theta=0. At theta=0 the limits are
    # sin(θ)/θ → 1 and (1−cos(θ))/θ² → 1/2, and using the padded
    # values recovers those limits to ~1e-15 (the small-angle Taylor
    # truncation error of the formula itself).
    theta_sq = theta_sq_raw + 1e-30
    theta = cs.sqrt(theta_sq)

    K = cs.vertcat(
        cs.horzcat(0.0, -rz, ry),
        cs.horzcat(rz, 0.0, -rx),
        cs.horzcat(-ry, rx, 0.0),
    )
    I3 = cs.MX.eye(3)
    # R = I + sin(θ)/θ K + (1-cos(θ))/θ² K²
    return I3 + cs.sin(theta) / theta * K + (1.0 - cs.cos(theta)) / theta_sq * K @ K


def _casadi_left_jacobian(rv):
    """CasADi-symbolic left Jacobian of SO(3): rotation-vector RATE → world ω.

    The optimiser parameterises orientation by a rotation vector and its
    twist is the rotation-vector *rate* (d/dt of the pose), NOT the physical
    angular velocity. The two coincide only near θ=0; at the demo's ~36°
    throw *rotation* (mostly the ~34° yaw, ~5 rad/s — the tilt itself stays
    ≤30°) they diverge enough to flip the sign of the lateral ω×r cup-velocity
    term. ``ω_world = J_l(φ) · φ̇`` with

        J_l = I + ((1−cosθ)/θ²) K + ((θ−sinθ)/θ³) K²,   K = skew(φ)

    converts the rate to the true world-frame angular velocity so the
    cup-velocity model matches the rigid-body physics. Eps-padded exactly
    like :func:`_casadi_rodrigues` so the θ=0 limits (½, 1/6) stay finite
    and the symbolic Jacobian is well-defined there.
    """
    rv = cs.reshape(rv, 3, 1)
    rx, ry, rz = rv[0], rv[1], rv[2]
    theta_sq = rx * rx + ry * ry + rz * rz + 1e-30
    theta = cs.sqrt(theta_sq)
    K = cs.vertcat(
        cs.horzcat(0.0, -rz, ry),
        cs.horzcat(rz, 0.0, -rx),
        cs.horzcat(-ry, rx, 0.0),
    )
    I3 = cs.MX.eye(3)
    c1 = (1.0 - cs.cos(theta)) / theta_sq
    c2 = (theta - cs.sin(theta)) / (theta * theta_sq)
    return I3 + c1 * K + c2 * (K @ K)


def _ik_extensions_sym(pos, R, geom: StewartGeometry):
    """CasADi-symbolic IK: ``(pos, R) → 6 leg extensions (mm)``.

    Mirrors :func:`jugglebot.motion.ik_solver.pose_to_leg_lengths`:

      platform_centre_world = pos + [0, 0, init_height_mm]
      plat_attach_i_world   = platform_centre_world + R @ plat_nodes[i]
      leg_vec_i             = plat_attach_i_world − base_nodes[i]
      ext_i                 = ||leg_vec_i|| − init_leg_lengths_mm[i]

    Parameters
    ----------
    pos : 3x1 ``cs.MX``
        Platform centroid offset from STOW (mm), as in
        ``pose_to_leg_lengths``.
    R : 3x3 ``cs.MX``
        Platform rotation matrix (from :func:`_casadi_rodrigues`).
    geom : StewartGeometry
        Source of the constant ``plat_nodes`` / ``base_nodes`` /
        ``init_leg_lengths_mm`` / ``init_height_mm`` arrays.

    Returns
    -------
    cs.MX of shape (6, 1)
        Leg extensions in mm, ordered legs 0..5.
    """
    init_height_offset = cs.DM(np.array([[0.0], [0.0], [geom.init_height_mm]]))
    centroid_world = pos + init_height_offset
    base = cs.DM(geom.base_nodes)            # (6, 3) constant
    plat = cs.DM(geom.plat_nodes)            # (6, 3) constant
    init_lengths = cs.DM(geom.init_leg_lengths_mm.reshape(6, 1))

    extensions = []
    for i in range(6):
        plat_node_i = plat[i, :].T            # (3, 1)
        base_node_i = base[i, :].T            # (3, 1)
        plat_world = centroid_world + R @ plat_node_i
        leg_vec = plat_world - base_node_i
        # +1e-30 guards sqrt at the degenerate origin; the gradient
        # is finite for ||leg_vec|| > 0 which always holds in
        # practice (the legs are never zero-length).
        leg_length = cs.sqrt(cs.dot(leg_vec, leg_vec) + 1e-30)
        extensions.append(leg_length - init_lengths[i])
    return cs.vertcat(*extensions)            # (6, 1)


def _quintic_jerk_integral_sym(p0, v0, a0, p1, v1, a1, T):
    """CasADi-symbolic analytical jerk² integral mirroring
    :func:`controller.hermite.quintic_jerk_integral`.

    Sum of per-dimension integrals over a quintic-Hermite segment.
    Returns a scalar ``cs.MX`` expression.
    """
    V0 = v0 * T
    V1 = v1 * T
    A0 = a0 * (T * T)
    A1 = a1 * (T * T)

    c0 = -60 * p0 - 36 * V0 - 9 * A0 + 60 * p1 - 24 * V1 + 3 * A1
    c1 = 360 * p0 + 192 * V0 + 36 * A0 - 360 * p1 + 168 * V1 - 24 * A1
    c2 = -360 * p0 - 180 * V0 - 30 * A0 + 360 * p1 - 180 * V1 + 30 * A1

    # Per-dimension squared-jerk integral, then sum across dims.
    per_dim = (c0 * c0
               + c0 * c1
               + (2 * c0 * c2 + c1 * c1) / 3.0
               + c1 * c2 / 2.0
               + c2 * c2 / 5.0)
    return cs.sum2(per_dim) / (T ** 5)


# --------------------------------------------------------------------------
# Public API
# --------------------------------------------------------------------------

def optimise_juggle_trajectory(pattern: JugglePattern,
                               cfg: Optional[OptimizerConfig] = None
                               ) -> OptimizerResult:
    """Solve the periodic NLP and return the optimised trajectory.

    Raises :class:`RuntimeError` if IPOPT fails to converge — that
    usually means the keyframe constraints are infeasible with the
    given pattern + workspace bounds. The leg-jerk² objective composed
    with the IK norm + Rodrigues rotation matrix is non-convex; IPOPT
    converges in ~40-65 iterations at the default ``n_knots=12`` from
    the analytic-oval warm start.
    """
    if cfg is None:
        cfg = OptimizerConfig()

    P = pattern.platform_period_s
    if cfg.catch_offset_s is None:
        catch_offset = pattern.flight_time_s - P
    else:
        catch_offset = cfg.catch_offset_s
    if not (0.0 < catch_offset < P):
        raise ValueError(
            f"catch_offset_s={catch_offset:.4f} not in (0, "
            f"platform_period_s={P:.4f}). For the default pattern the "
            f"ballistic value is flight_time − P = {pattern.flight_time_s - P:.4f}.")

    if cfg.n_knots < 4:
        raise ValueError("n_knots must be >= 4")
    if cfg.n_samples < cfg.n_knots:
        raise ValueError("n_samples must be >= n_knots")

    N = cfg.n_knots
    dt_knot = P / N
    flight = pattern.flight_time_s
    # StewartGeometry: needed by the leg-space jerk objective + leg-vel
    # bound (cuts #4 / #5). Defaults to the project's configured
    # geometry; constructed once per optimiser call (cheap).
    pattern_geom = StewartGeometry()

    # Hand-opening offsets at the throw release / catch arrival instants
    # (platform-local z above the centroid). Independent of pose; depend
    # only on throw_speed_mps (release) and on the fixed hand kinematics
    # (arrival).
    h_release_mm = _hand_offset_at_release_mm(pattern.throw_speed_mps)
    h_arrival_mm = _hand_offset_at_arrival_mm()
    g_mms2 = 9806.0

    # Locate the catch instant: which segment is it in, and at what
    # local time within that segment? Constant integers; no symbolic
    # uncertainty.
    k_catch = int(catch_offset // dt_knot)
    if k_catch >= N:
        k_catch = N - 1
    t_catch_local = catch_offset - k_catch * dt_knot

    # ---- NLP ---------------------------------------------------------
    opti = cs.Opti()
    poses = opti.variable(N, 6)
    twists = opti.variable(N, 6)
    accels = opti.variable(N, 6)

    # ---- Throw keyframe (knot 0): xyz pinned, orientation free -----
    #
    # The orientation-aware ball-release-velocity constraint: the ball
    # at release inherits ``v_platform + R_throw @ [0, 0, throw_speed]``
    # from the kinematic hold + the hand's vertical throw stroke. The
    # required release velocity is inverse-ballistics from the hand-
    # opening's world position at release to the hand-opening's world
    # position at the catch arrival. Both endpoints depend on platform
    # orientation, so the whole system is coupled — but the equations
    # are still smooth in the decision variables.
    opti.subject_to(poses[0, 0] == pattern.throw_pose[0])    # x
    opti.subject_to(poses[0, 1] == pattern.throw_pose[1])    # y
    opti.subject_to(poses[0, 2] == pattern.throw_pose[2])    # z

    R_throw = _casadi_rodrigues(poses[0, 3:6])
    centroid_throw_world = cs.vertcat(
        poses[0, 0],
        poses[0, 1],
        poses[0, 2] + cfg.platform_init_height_mm,
    )
    release_pos_world = centroid_throw_world + R_throw @ cs.vertcat(0.0, 0.0, h_release_mm)

    # Catch keyframe (mid-segment in general). Only the pose is
    # constrained; twist and accel at catch are free, so we discard
    # those quintic-eval outputs.
    pose_catch_sym, twist_catch_sym, _accel_catch_sym = _quintic_eval_sym(
        poses[k_catch, :], twists[k_catch, :], accels[k_catch, :],
        poses[(k_catch + 1) % N, :], twists[(k_catch + 1) % N, :],
        accels[(k_catch + 1) % N, :],
        dt_knot, t_catch_local)

    # ---- Catch keyframe: xyz pinned; twist + orientation FREE ------
    # The optimiser's freedom at catch is the whole point of this cut —
    # it can lean into / out of the ball, move with or against it, as
    # long as the platform centroid sits at the right xyz for the
    # arriving ball.
    opti.subject_to(pose_catch_sym[0, 0] == pattern.catch_pose[0])
    opti.subject_to(pose_catch_sym[0, 1] == pattern.catch_pose[1])
    opti.subject_to(pose_catch_sym[0, 2] == pattern.catch_pose[2])

    R_catch = _casadi_rodrigues(pose_catch_sym[0, 3:6])
    centroid_catch_world = cs.vertcat(
        pose_catch_sym[0, 0],
        pose_catch_sym[0, 1],
        pose_catch_sym[0, 2] + cfg.platform_init_height_mm,
    )
    catch_target_world = centroid_catch_world + R_catch @ cs.vertcat(0.0, 0.0, h_arrival_mm)

    # ---- Orientation-aware throw equality (3 equations) -------------
    # The hand-opening (cup) is offset r_hand = R_throw @ [0,0,h_release]
    # above the platform centroid, so its world velocity is the full rigid-
    # body expression
    #   v_cup = v_centroid + omega × r_hand + R_throw @ [0,0,throw_speed]
    # i.e. centroid translation + rotation of the offset (omega × r_hand) +
    # the slider extending along the hand axis. The released ball inherits
    # exactly v_cup (validated in sim: ball velocity == cup velocity to
    # within solver noise — see logbook 2026-06-26 contact integration).
    # The omega × r_hand term was previously omitted; with the old kinematic
    # velocity-override it was invisible, but under a faithful contact throw
    # it is the dominant lateral aim error (at the throw the platform yaws
    # at ~5 rad/s and r_hand ≈ 0.2 m, so omega × r_hand ≈ 0.2 m/s — the same
    # magnitude as the entire planned lateral velocity). Including it makes
    # the plan match what the physical cup actually imparts.
    # v_required = compute_launch_velocity(release_pos, target, flight)
    #            = (target - release) / flight + [0, 0, 0.5 * g * flight]
    v_required = (catch_target_world - release_pos_world) / flight + \
                 cs.vertcat(0.0, 0.0, 0.5 * g_mms2 * flight)
    r_hand_throw = R_throw @ cs.vertcat(0.0, 0.0, h_release_mm)
    # True world angular velocity from the rotvec rate (NOT the rate itself).
    omega_throw = _casadi_left_jacobian(poses[0, 3:6]) @ twists[0, 3:6].T
    v_ball_release = (twists[0, 0:3].T
                      + cs.cross(omega_throw, r_hand_throw)
                      + R_throw @ cs.vertcat(0.0, 0.0, pattern.throw_speed_mps * 1000.0))
    opti.subject_to(v_ball_release == v_required)

    # ---- HARD catch velocity-match (3 equations) -------------------
    # Invariant (user-stated, "must" at every event): the hand-opening
    # (cup) velocity is COLINEAR with the ball's arrival velocity and at
    # ``catch_vel_ratio`` (0.7) of its speed — so the cup moves *with* the
    # ball but 30% slower, giving a closing velocity that seats the ball
    # into the cup (a full 1.0 match → zero relative velocity → the ball
    # never closes in → off-centre rim contact). The cup velocity is the
    # resultant of the platform centroid twist + the catch-stroke slider
    # descending along the (tilted) hand axis at ``catch_slider_vel_ratio``
    # (0.6) of the vertical launch speed (≈ ball speed); the free platform
    # twist supplies the remainder of the 0.7×v_ball target exactly. The
    # vector equality enforces both colinearity and the 0.7 magnitude.
    # The cup velocity is the full rigid-body expression — centroid twist +
    # omega × r_hand (rotation of the hand-opening offset) + the catch-stroke
    # slider — matching the throw model above. (The omega × r_hand term was
    # previously omitted here too; it is included now so the planned cup
    # velocity equals what the physical cup imparts/receives.) Orientation
    # stays free here — only the soft facing penalty below shapes the cup to
    # point along the arrival.
    v_ball_arrival = v_required + cs.vertcat(0.0, 0.0, -g_mms2 * flight)
    catch_slider_speed = cfg.catch_slider_vel_ratio * pattern.throw_speed_mps * 1000.0
    r_hand_catch = R_catch @ cs.vertcat(0.0, 0.0, h_arrival_mm)
    omega_catch = _casadi_left_jacobian(pose_catch_sym[0, 3:6]) @ twist_catch_sym[0, 3:6].T
    v_hand_catch = (twist_catch_sym[0, 0:3].T
                    + cs.cross(omega_catch, r_hand_catch)
                    + R_catch @ cs.vertcat(0.0, 0.0, -catch_slider_speed))
    opti.subject_to(v_hand_catch == cfg.catch_vel_ratio * v_ball_arrival)

    # ---- Level-platform lock (opt-in for back-compat / comparison) --
    # Pre-relaxation default; useful for A/B comparing the optimiser
    # against the level-locked first cut.
    if cfg.bank_locked_level:
        for k in range(N):
            for j in (3, 4, 5):
                opti.subject_to(poses[k, j] == 0.0)
                opti.subject_to(twists[k, j] == 0.0)
                opti.subject_to(accels[k, j] == 0.0)
    else:
        # Tilt bound at every knot: rx² + ry² ≤ tilt_limit². rz (yaw)
        # is unbounded — yaw doesn't change the hand-axis direction
        # so it's purely cosmetic and the optimiser usually leaves it
        # at zero on its own. Keeps the bound a simple convex circle
        # rather than a more expensive full-tilt formula.
        for k in range(N):
            opti.subject_to(
                poses[k, 3] ** 2 + poses[k, 4] ** 2
                <= cfg.tilt_limit_rad ** 2)

    # Workspace bounds at every knot.
    for k in range(N):
        opti.subject_to(opti.bounded(
            -cfg.workspace_xy_mm, poses[k, 0], cfg.workspace_xy_mm))
        opti.subject_to(opti.bounded(
            -cfg.workspace_xy_mm, poses[k, 1], cfg.workspace_xy_mm))
        opti.subject_to(opti.bounded(
            cfg.workspace_z_mm[0], poses[k, 2], cfg.workspace_z_mm[1]))

    # ---- Objective construction --------------------------------------
    cost = 0

    # Pose-jerk² (legacy / optional regulariser; default weight 0).
    if cfg.pose_jerk_weight > 0.0:
        pose_jerk_cost = 0
        for k in range(N):
            j = (k + 1) % N
            pose_jerk_cost = pose_jerk_cost + _quintic_jerk_integral_sym(
                poses[k, :], twists[k, :], accels[k, :],
                poses[j, :], twists[j, :], accels[j, :],
                dt_knot)
        cost = cost + cfg.pose_jerk_weight * pose_jerk_cost

    # Leg-space jerk² (primary smoothness objective from cut #4).
    #
    # Pose-jerk² is unit-imbalanced — rotation jerk (rad/s³) is ~1e6×
    # smaller than translation jerk (mm/s³) in the same number, so
    # orientation is essentially free, producing degenerate constant-
    # tilt solutions at low apex. Leg-jerk² puts everything in mm
    # via the IK, so all six DOFs feed into a uniformly-weighted
    # actuator-current-jerk cost. Computed via finite differences on
    # a dense sub-sampled grid (``n_sub_per_segment`` per knot
    # segment); ``leg_extensions(t)`` is a nonlinear function of
    # pose(t) so the analytical quintic-jerk integral doesn't apply.
    M = cfg.n_sub_per_segment
    if M < 4:
        raise ValueError("n_sub_per_segment must be >= 4 (3rd central "
                         "difference needs ±2 neighbours)")
    M_total = N * M
    dt_sub = dt_knot / M

    # Sub-sample: at every sub-grid point compute pose(t) via quintic
    # Hermite from the bracketing knot states, then IK to leg
    # extensions. Pre-build the (M_total, 6) leg-extension matrix so
    # both the jerk objective AND the leg-velocity bound (cut #5) can
    # consume it without redoing the IK chain.
    leg_ext_per_sub: list = []   # list of (6, 1) MX expressions
    for k_seg in range(N):
        j_seg = (k_seg + 1) % N
        for m in range(M):
            t_local = m * dt_sub
            pose_sub, _, _ = _quintic_eval_sym(
                poses[k_seg, :], twists[k_seg, :], accels[k_seg, :],
                poses[j_seg, :], twists[j_seg, :], accels[j_seg, :],
                dt_knot, t_local)
            # pose_sub is (1, 6); slice as a 3x1 column vector.
            pos_sub = pose_sub[0, 0:3].T
            R_sub = _casadi_rodrigues(pose_sub[0, 3:6])
            leg_ext_per_sub.append(_ik_extensions_sym(pos_sub, R_sub, pattern_geom))

    # ---- Leg-stroke (position) bound --------------------------------
    # Every leg extension must stay within the physical stroke [0, stroke]
    # at every sub-grid point — beyond it the actuators saturate and the
    # trajectory is unplayable. Previously only leg *velocity* was bounded;
    # the platform workspace box implicitly kept positions in range, but
    # the velocity-matched catch (2026-06-26) reshapes the trajectory
    # enough to drive a leg past full stroke (~330 mm vs 280) without this
    # explicit bound. Cheap (reuses the leg_ext_per_sub already built for
    # the jerk objective); makes leg-stroke feasibility a hard contract.
    leg_stroke_mm = pattern_geom.leg_stroke_mm
    for m in range(M_total):
        for i in range(6):
            opti.subject_to(opti.bounded(0.0, leg_ext_per_sub[m][i], leg_stroke_mm))

    # Periodic 3rd central difference for jerk at each sub-grid point.
    # leg_jerk_i(m) ≈ (e[m+2] - 2 e[m+1] + 2 e[m-1] - e[m-2]) / (2 dt_sub³)
    if cfg.leg_jerk_weight > 0.0:
        leg_jerk_cost = 0
        inv_2dt3 = 1.0 / (2.0 * dt_sub ** 3)
        for m in range(M_total):
            mp2 = (m + 2) % M_total
            mp1 = (m + 1) % M_total
            mm1 = (m - 1) % M_total
            mm2 = (m - 2) % M_total
            jerk = (leg_ext_per_sub[mp2] - 2 * leg_ext_per_sub[mp1]
                    + 2 * leg_ext_per_sub[mm1] - leg_ext_per_sub[mm2]) * inv_2dt3
            # Sum over 6 legs (Riemann × dt_sub for an ∫dt approximation).
            leg_jerk_cost = leg_jerk_cost + cs.dot(jerk, jerk) * dt_sub
        cost = cost + cfg.leg_jerk_weight * leg_jerk_cost

    # ---- Leg-velocity bound (cut #5) --------------------------------
    # Periodic 1st central difference for leg velocity at each sub-grid
    # point. Used by both the hard cap and the minimax slack.
    if (cfg.leg_vel_hard_cap_mms is not None
            or cfg.leg_vel_equalize_weight > 0.0):
        inv_2dt = 1.0 / (2.0 * dt_sub)
        # Optional minimax slack: a scalar v_max bounds every leg-vel
        # sample, and v_max is minimised (with weight) — drives all 6
        # legs to share peak velocity (the "no leg works much harder
        # than the others" criterion).
        v_max_slack = None
        if cfg.leg_vel_equalize_weight > 0.0:
            v_max_slack = opti.variable()
            opti.subject_to(v_max_slack >= 0.0)
            cost = cost + cfg.leg_vel_equalize_weight * v_max_slack

        for m in range(M_total):
            mp1 = (m + 1) % M_total
            mm1 = (m - 1) % M_total
            vel = (leg_ext_per_sub[mp1] - leg_ext_per_sub[mm1]) * inv_2dt
            # vel is (6, 1); per-leg per-sample bound on absolute value.
            for i in range(6):
                if cfg.leg_vel_hard_cap_mms is not None:
                    cap = cfg.leg_vel_hard_cap_mms
                    opti.subject_to(opti.bounded(-cap, vel[i], cap))
                if v_max_slack is not None:
                    opti.subject_to(opti.bounded(
                        -v_max_slack, vel[i], v_max_slack))

    # Soft catch-colinearity penalty (optional).
    #
    # The user-stated invariant is: the hand throws the ball ALONG the
    # platform z-axis in the platform's frame; in world frame the ball's
    # velocity is ``v_plat + R @ [0,0,throw_speed]``. The mirror at catch
    # is that the ball arrives along the hand axis (in platform frame),
    # which means the world-frame ball-arrival velocity is parallel to
    # ``R_catch @ [0,0,1]``. This is a "should" not a "must" because the
    # passive cone tolerates some mis-alignment; encode it as a soft
    # penalty on the perpendicular component of v_ball_arrival.
    #
    # v_ball_arrival = v_required + [0, 0, −g·flight]  (gravity over flight)
    # hand_axis_catch_world = R_catch @ [0,0,1]
    # penalty = weight × ||v_ball_arrival × hand_axis_catch_world||²
    if cfg.catch_colinearity_weight > 0.0:
        # v_ball_arrival is defined above (hard catch velocity-match).
        hand_axis_catch = R_catch @ cs.vertcat(0.0, 0.0, 1.0)
        cross = cs.cross(v_ball_arrival, hand_axis_catch)
        cost = cost + cfg.catch_colinearity_weight * cs.dot(cross, cross)

    # Soft THROW facing penalty (mirror of catch). The hand axis at throw
    # is encouraged toward the launch-velocity direction so the cup
    # "faces" the throw. 'Facing soft' — the hard hand-velocity match
    # above always wins; this only shapes the otherwise-free orientation.
    if cfg.throw_colinearity_weight > 0.0:
        hand_axis_throw = R_throw @ cs.vertcat(0.0, 0.0, 1.0)
        cross_throw = cs.cross(v_required, hand_axis_throw)
        cost = cost + cfg.throw_colinearity_weight * cs.dot(cross_throw, cross_throw)

    opti.minimize(cost)

    # Warm start from the analytic oval (same period, same n_samples == N).
    init_traj = build_analytic_oval(pattern, n_samples=N)
    opti.set_initial(poses, init_traj.poses)
    opti.set_initial(twists, init_traj.twists)
    opti.set_initial(accels, init_traj.accels)

    opti.solver('ipopt', {'print_time': 0, 'ipopt.print_level': 0,
                          'ipopt.sb': 'yes'})
    sol = opti.solve()

    poses_opt = np.asarray(sol.value(poses))
    twists_opt = np.asarray(sol.value(twists))
    accels_opt = np.asarray(sol.value(accels))
    obj_value = float(sol.value(cost))
    n_iter = int(sol.stats().get('iter_count', 0))

    # Build the (resampled) JuggleTrajectory.
    traj = _resample_quintic_to_juggle_trajectory(
        P, poses_opt, twists_opt, accels_opt, cfg.n_samples)

    return OptimizerResult(
        trajectory=traj,
        catch_offset_s=catch_offset,
        objective_value=obj_value,
        knot_poses=poses_opt,
        knot_twists=twists_opt,
        knot_accels=accels_opt,
        n_iter=n_iter,
    )


def _resample_quintic_to_juggle_trajectory(period_s: float,
                                           poses: np.ndarray,
                                           twists: np.ndarray,
                                           accels: np.ndarray,
                                           n_samples: int
                                           ) -> JuggleTrajectory:
    """Resample N-knot quintic-Hermite trajectory to an n_samples-grid
    :class:`JuggleTrajectory`.

    The N knots define N quintic-Hermite segments (with wraparound for
    segment N-1). When ``n_samples`` is an integer multiple of
    ``n_knots`` (e.g. the default 128/16), each runtime sub-segment lies
    wholly within one optimiser knot segment and the runtime quintic
    Hermite *exactly* reproduces the optimiser's piecewise quintic.

    For non-multiple ratios, the runtime quintic still C2-matches the
    optimiser at every sample point (since the pose/twist/accel sampled
    here are continuous across knot boundaries) — but sub-segments
    *straddling* a knot boundary follow a single quintic where the
    optimiser had two pieces, so jerk profile inside such sub-segments
    differs from the optimiser's. Accuracy at the sample points is
    unaffected; downstream consumers that integrate or differentiate
    between samples should prefer the multiple-of relationship.
    """
    N = poses.shape[0]
    dt_knot = period_s / N
    poses_r = np.zeros((n_samples, 6))
    twists_r = np.zeros((n_samples, 6))
    accels_r = np.zeros((n_samples, 6))
    for m in range(n_samples):
        t = m * (period_s / n_samples)
        k = int(t // dt_knot)
        if k >= N:
            k = N - 1
        j = (k + 1) % N
        local = t - k * dt_knot
        pose, twist, accel = quintic_interp_with_accel(
            poses[k], twists[k], accels[k],
            poses[j], twists[j], accels[j],
            dt_knot, local)
        poses_r[m] = pose
        twists_r[m] = twist
        accels_r[m] = accel
    return JuggleTrajectory(period_s, poses_r, twists_r, accels_r)


# --------------------------------------------------------------------------
# .npz save / load helper
# --------------------------------------------------------------------------

def save_optimised_trajectory(result: OptimizerResult,
                              path: str | Path) -> None:
    """Write the optimised trajectory + metadata to a .npz file.

    Loadable with :meth:`JuggleTrajectory.load` for trajectory replay;
    the additional metadata (catch_offset_s, objective_value, knot
    arrays) is preserved for diagnostics but not consumed by the runtime
    player.
    """
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(
        path,
        period_s=result.trajectory.period_s,
        poses=result.trajectory.poses,
        twists=result.trajectory.twists,
        accels=result.trajectory.accels,
        catch_offset_s=result.catch_offset_s,
        objective_value=result.objective_value,
        knot_poses=result.knot_poses,
        knot_twists=result.knot_twists,
        knot_accels=result.knot_accels,
        n_iter=result.n_iter,
    )
