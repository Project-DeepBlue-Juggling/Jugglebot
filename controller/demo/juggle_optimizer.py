"""Offline CasADi optimiser for the periodic juggle-demo platform trajectory.

Solves, once, an NLP for the steady-state platform pose trajectory that
satisfies the throw and catch keyframe constraints with minimum integrated
pose-jerk. The runtime player loads the result (a :class:`JuggleTrajectory`)
and plays it at 40 Hz with no solver in the loop.

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
        Number of quintic-Hermite knots per period. ``16`` is a sensible
        baseline — coarse enough that IPOPT converges quickly, fine
        enough that the C2 spline between knots stays close to the
        underlying continuous optimal.
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
        ``tilt_limit_rad`` to reduce pose-jerk and exploit the hand-axis
        tilt-induced horizontal velocity contribution at throw.
    catch_offset_s : float or None
        Catch event time within the period. ``None`` derives it from
        the pattern's ballistics (``flight_time_s − platform_period_s``).
    workspace_xy_mm : float
        Absolute box bound on platform x and y at every knot, mm.
    workspace_z_mm : tuple of (float, float)
        Lower / upper bound on platform z at every knot, mm.
    """
    n_knots: int = 16
    n_samples: int = 128
    bank_locked_level: bool = False
    tilt_limit_rad: float = TILT_LIMIT_RAD
    catch_offset_s: Optional[float] = None
    workspace_xy_mm: float = 200.0
    workspace_z_mm: tuple[float, float] = (0.0, 300.0)
    # World-frame z of the platform centroid at the STOW pose, mm. The
    # optimiser works in STOW-relative pose coordinates but the ballistic
    # constraints need world frame; the only quantity that bridges the
    # frames is this offset. Default matches the StewartGeometry default
    # (574.3 mm); override via the caller for non-default geometries.
    platform_init_height_mm: float = 574.3


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
        Final value of the integrated pose-jerk² objective. Useful for
        comparing optimiser cuts and for the "optimiser-improves-on-
        analytic-oval" sanity test.
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
    given pattern + workspace bounds. The current first-cut formulation
    is convex-ish (quadratic objective in pose-knots, linear keyframe
    constraints, box bounds) and converges in ~20 iterations from the
    analytic-oval warm start.
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
    pose_catch_sym, _twist_catch_sym, _accel_catch_sym = _quintic_eval_sym(
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
    # v_ball_release = v_platform_at_throw + R_throw @ [0, 0, throw_speed_mms]
    # v_required = compute_launch_velocity(release_pos, target, flight)
    #            = (target - release) / flight + [0, 0, 0.5 * g * flight]
    v_required = (catch_target_world - release_pos_world) / flight + \
                 cs.vertcat(0.0, 0.0, 0.5 * g_mms2 * flight)
    v_ball_release = twists[0, 0:3].T + R_throw @ cs.vertcat(
        0.0, 0.0, pattern.throw_speed_mps * 1000.0)
    opti.subject_to(v_ball_release == v_required)

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

    # Objective: sum of analytical jerk integrals over all N segments
    # (segment k connects knot k -> knot (k+1) mod N).
    cost = 0
    for k in range(N):
        j = (k + 1) % N
        cost = cost + _quintic_jerk_integral_sym(
            poses[k, :], twists[k, :], accels[k, :],
            poses[j, :], twists[j, :], accels[j, :],
            dt_knot)
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
