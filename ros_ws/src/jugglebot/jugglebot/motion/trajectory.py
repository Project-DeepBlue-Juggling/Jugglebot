"""Quintic trajectory generation for the Stewart platform.

Phase 4 deliverables:
  - Quintic polynomial solver: 6 boundary conditions -> 6 coefficients
  - Trajectory evaluator: time t -> (pose, twist, accel) in Cartesian space
  - Leg-space mapper: Cartesian state -> motor commands (pos, vel_ff, torque_ff)
  - Feasibility checker: validate trajectory against kinematic/dynamic limits
  - Speed scaling: time-stretching to uniformly scale velocities/accelerations
  - Trajectory manager: state machine for trajectory execution

Pure Python + numpy.  No ROS2, no ZeroMQ dependency.

Coordinate conventions
----------------------
- **Pose**: ``[x, y, z, rx, ry, rz]`` where ``[x,y,z]`` is the platform
  offset from home in mm and ``[rx,ry,rz]`` is a rotation vector in radians.
- **Twist**: ``[vx, vy, vz, wx, wy, wz]`` in mm/s and rad/s.
- **Accel**: ``[ax, ay, az, alphax, alphay, alphaz]`` in mm/s^2 and rad/s^2.

Rotation is interpolated as quintic polynomials on rotation vector components.
This is valid for the platform's limited tilt range (<=15 degrees).
"""

from __future__ import annotations 

import enum
import logging
import time as _time
from dataclasses import dataclass, replace as _dc_replace
from typing import TYPE_CHECKING

import numpy as np

from jugglebot.motion.conversions import (
    extensions_mm_to_revs,
    leg_velocities_to_motor_velocities,
)
from jugglebot.motion.dynamics import (
    DynamicsParams,
    compute_full_feedforward_torques,
    gravity_to_motor_torques,
)
from jugglebot.motion.ik_solver import (
    pose_to_leg_lengths,
    rotvec_to_rot_matrix,
    twist_to_leg_velocities,
    accel_to_leg_accels,
)
from jugglebot.motion.workspace import compute_condition_number

if TYPE_CHECKING:
    from jugglebot.motion.geometry import StewartGeometry

logger = logging.getLogger(__name__)

# Minimum time (seconds) between now and a dynamic target's arrival_time.
# Targets closer than this are rejected — they leave insufficient margin
# for feasibility checking and safe trajectory execution.
MIN_LEAD_TIME_S = 0.3


# ---------------------------------------------------------------------------
# Quintic polynomial solver
# ---------------------------------------------------------------------------

def solve_quintic_1d(x0: float, v0: float, a0: float,
                     xf: float, vf: float, af: float,
                     duration: float) -> np.ndarray:
    """Solve for quintic polynomial coefficients (one scalar DoF).

    Computes the 6 coefficients of a quintic polynomial in *normalized time*
    ``tau = t / T`` (where T = duration) that satisfies boundary conditions
    on position, velocity, and acceleration at tau=0 and tau=1.

    Parameters
    ----------
    x0, v0, a0 : float
        Initial position, velocity, acceleration.
    xf, vf, af : float
        Final position, velocity, acceleration.
    duration : float
        Trajectory duration T in seconds.  Must be > 0.

    Returns
    -------
    coeffs : (6,) ndarray
        Coefficients ``[c0, c1, c2, c3, c4, c5]`` such that::

            p(tau) = c0 + c1*tau + c2*tau^2 + c3*tau^3 + c4*tau^4 + c5*tau^5

        with::

            velocity     = p'(tau) / T
            acceleration = p''(tau) / T^2
    """
    if duration <= 0:
        raise ValueError(f"duration must be > 0, got {duration}")

    T = duration
    T2 = T * T
    dx = xf - x0

    c0 = x0
    c1 = v0 * T
    c2 = 0.5 * a0 * T2
    c3 = 10.0 * dx - (6.0 * v0 + 4.0 * vf) * T - (1.5 * a0 - 0.5 * af) * T2
    c4 = -15.0 * dx + (8.0 * v0 + 7.0 * vf) * T + (1.5 * a0 - af) * T2
    c5 = 6.0 * dx - 3.0 * (v0 + vf) * T - 0.5 * (a0 - af) * T2

    return np.array([c0, c1, c2, c3, c4, c5])


# ---------------------------------------------------------------------------
# QuinticTrajectory data class
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class QuinticTrajectory:
    """6-DoF quintic trajectory in Cartesian space.

    Each of the 6 DoFs (x, y, z, rx, ry, rz) has an independent quintic
    polynomial.  All share the same duration and start time.

    Attributes
    ----------
    coeffs : (6, 6) ndarray
        Row i = DoF i, columns = [c0..c5] in normalized time.
    duration : float
        Effective duration in seconds (after speed scaling).
    t_start : float
        Absolute start time (seconds, control-loop clock).
    speed_scale : float
        Applied speed scale factor (1.0 = full speed).
    start_state : (18,) ndarray
        Original boundary conditions [pose(6), twist(6), accel(6)].
    end_state : (18,) ndarray
        Original boundary conditions [pose(6), twist(6), accel(6)].
    original_duration : float
        Duration before speed scaling.
    """
    coeffs: np.ndarray
    duration: float
    t_start: float
    speed_scale: float
    start_state: np.ndarray
    end_state: np.ndarray
    original_duration: float


# ---------------------------------------------------------------------------
# Trajectory creation
# ---------------------------------------------------------------------------

def create_trajectory(start_pose: np.ndarray,
                      start_twist: np.ndarray,
                      start_accel: np.ndarray,
                      end_pose: np.ndarray,
                      end_twist: np.ndarray,
                      end_accel: np.ndarray,
                      duration: float,
                      t_start: float = 0.0,
                      speed_scale: float = 1.0) -> QuinticTrajectory:
    """Create a 6-DoF quintic trajectory.

    Parameters
    ----------
    start_pose : (6,) ndarray
        ``[x, y, z, rx, ry, rz]`` in mm and radians.
    start_twist : (6,) ndarray
        ``[vx, vy, vz, wx, wy, wz]`` in mm/s and rad/s.
    start_accel : (6,) ndarray
        ``[ax, ay, az, alphax, alphay, alphaz]`` in mm/s^2 and rad/s^2.
    end_pose, end_twist, end_accel : (6,) ndarray
        Same conventions as start.
    duration : float
        Base trajectory duration in seconds (before speed scaling).
    t_start : float
        Absolute start time reference (seconds).
    speed_scale : float
        Speed scaling factor in (0.0, 1.0].  Stretches duration by
        ``1/speed_scale`` and scales boundary velocities by ``speed_scale``,
        accelerations by ``speed_scale**2``.

    Returns
    -------
    QuinticTrajectory
    """
    if duration <= 0:
        raise ValueError(f"duration must be > 0, got {duration}")
    if speed_scale <= 0 or speed_scale > 1.0:
        raise ValueError(f"speed_scale must be in (0, 1], got {speed_scale}")

    start_pose = np.asarray(start_pose, dtype=np.float64)
    start_twist = np.asarray(start_twist, dtype=np.float64)
    start_accel = np.asarray(start_accel, dtype=np.float64)
    end_pose = np.asarray(end_pose, dtype=np.float64)
    end_twist = np.asarray(end_twist, dtype=np.float64)
    end_accel = np.asarray(end_accel, dtype=np.float64)

    # Store original boundary conditions before scaling
    start_state = np.concatenate([start_pose, start_twist, start_accel])
    end_state = np.concatenate([end_pose, end_twist, end_accel])

    # Apply speed scaling: stretch duration, scale velocities and accelerations
    effective_duration = duration / speed_scale
    scaled_start_twist = start_twist * speed_scale
    scaled_start_accel = start_accel * (speed_scale ** 2)
    scaled_end_twist = end_twist * speed_scale
    scaled_end_accel = end_accel * (speed_scale ** 2)

    # Solve per-DoF quintic polynomials
    coeffs = np.empty((6, 6))
    for i in range(6):
        coeffs[i] = solve_quintic_1d(
            x0=start_pose[i], v0=scaled_start_twist[i], a0=scaled_start_accel[i],
            xf=end_pose[i], vf=scaled_end_twist[i], af=scaled_end_accel[i],
            duration=effective_duration,
        )

    return QuinticTrajectory(
        coeffs=coeffs,
        duration=effective_duration,
        t_start=t_start,
        speed_scale=speed_scale,
        start_state=start_state,
        end_state=end_state,
        original_duration=duration,
    )


def rescale_trajectory(traj: QuinticTrajectory,
                       new_speed_scale: float,
                       t_start: float | None = None) -> QuinticTrajectory:
    """Re-create a trajectory from its original boundary conditions
    with a different speed scale factor.

    Parameters
    ----------
    traj : QuinticTrajectory
        Original trajectory (only its stored boundary conditions are used).
    new_speed_scale : float
        New speed scale factor in (0.0, 1.0].
    t_start : float or None
        New start time.  Defaults to the original t_start.
    """
    s = traj.start_state
    e = traj.end_state
    return create_trajectory(
        start_pose=s[:6], start_twist=s[6:12], start_accel=s[12:18],
        end_pose=e[:6], end_twist=e[6:12], end_accel=e[12:18],
        duration=traj.original_duration,
        t_start=t_start if t_start is not None else traj.t_start,
        speed_scale=new_speed_scale,
    )


# ---------------------------------------------------------------------------
# Trajectory evaluation
# ---------------------------------------------------------------------------

def evaluate(traj: QuinticTrajectory,
             t: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Evaluate the trajectory at absolute time ``t``.

    Parameters
    ----------
    traj : QuinticTrajectory
    t : float
        Absolute time in seconds.

    Returns
    -------
    pose : (6,) ndarray — [x, y, z, rx, ry, rz] in mm, rad
    twist : (6,) ndarray — [vx, vy, vz, wx, wy, wz] in mm/s, rad/s
    accel : (6,) ndarray — [ax, ay, az, alphax, alphay, alphaz] in mm/s^2, rad/s^2

    Notes
    -----
    Before ``t_start``: returns start state.
    After ``t_start + duration``: returns end pose with zero twist/accel.
    """
    T = traj.duration
    t_end = traj.t_start + T

    if t <= traj.t_start:
        # Before trajectory start: return start state
        s = traj.start_state
        twist = s[6:12] * traj.speed_scale
        accel = s[12:18] * (traj.speed_scale ** 2)
        return s[:6].copy(), twist.copy(), accel.copy()

    if t >= t_end:
        # After trajectory end: hold at end pose, zero velocity/acceleration
        e = traj.end_state
        return e[:6].copy(), np.zeros(6), np.zeros(6)

    tau = (t - traj.t_start) / T
    c = traj.coeffs  # (6, 6)

    # Position: p(tau) = c0 + c1*tau + c2*tau^2 + c3*tau^3 + c4*tau^4 + c5*tau^5
    # Evaluate all 6 DoFs at once using Horner's method
    tau2 = tau * tau
    tau3 = tau2 * tau
    tau4 = tau3 * tau
    tau5 = tau4 * tau

    pose = c[:, 0] + c[:, 1]*tau + c[:, 2]*tau2 + c[:, 3]*tau3 + c[:, 4]*tau4 + c[:, 5]*tau5

    # Velocity: p'(tau) / T
    twist = (c[:, 1] + 2*c[:, 2]*tau + 3*c[:, 3]*tau2 + 4*c[:, 4]*tau3 + 5*c[:, 5]*tau4) / T

    # Acceleration: p''(tau) / T^2
    T2 = T * T
    accel = (2*c[:, 2] + 6*c[:, 3]*tau + 12*c[:, 4]*tau2 + 20*c[:, 5]*tau3) / T2

    return pose, twist, accel


def end_boundary_state(traj: QuinticTrajectory) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return the trajectory's end boundary conditions (pose, twist, accel).

    Unlike ``evaluate(traj, t_end)``, this returns the ORIGINAL boundary twist
    and acceleration, not zeros.
    """
    return (
        traj.end_state[:6].copy(),
        traj.end_state[6:12].copy(),
        traj.end_state[12:18].copy(),
    )


def evaluate_jerk(traj: QuinticTrajectory, t: float) -> np.ndarray:
    """Evaluate the Cartesian jerk (3rd derivative) at absolute time ``t``.

    Parameters
    ----------
    traj : QuinticTrajectory
    t : float
        Absolute time in seconds.

    Returns
    -------
    jerk : (6,) ndarray
        ``[jx, jy, jz, jrx, jry, jrz]`` in mm/s^3 and rad/s^3.

    Notes
    -----
    Jerk is computed in Cartesian space only.  Per-leg jerk is NOT checked
    because the Jacobian condition number constraint already guards against
    poses where smooth Cartesian motion maps to jerky leg motion.  If
    per-leg jerk becomes a concern, it would require differentiating through
    the Jacobian (J_dot contribution), which is deferred.

    Before ``t_start`` or after ``t_start + duration``: returns zeros
    (constant position or constant velocity has zero jerk).
    """
    T = traj.duration
    if t <= traj.t_start or t >= traj.t_start + T:
        return np.zeros(6)

    tau = (t - traj.t_start) / T
    c = traj.coeffs  # (6, 6)
    T3 = T * T * T

    # p'''(tau) / T^3 = 6*c3 + 24*c4*tau + 60*c5*tau^2
    jerk = (6.0 * c[:, 3] + 24.0 * c[:, 4] * tau + 60.0 * c[:, 5] * tau * tau) / T3
    return jerk


# ---------------------------------------------------------------------------
# Leg-space mapper
# ---------------------------------------------------------------------------

def cartesian_to_motor_commands(
    pose: np.ndarray,
    twist: np.ndarray,
    accel: np.ndarray,
    geom: StewartGeometry,
    dynamics_params: DynamicsParams,
    feedforward_enabled: bool = True,
    gravity_correction: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Convert a Cartesian state to motor commands.

    Maps the trajectory evaluator output to the ODrive command triple
    ``(input_pos, vel_ff, torque_ff)`` using IK and dynamics.

    Parameters
    ----------
    pose : (6,) ndarray — [x, y, z, rx, ry, rz] in mm, rad
    twist : (6,) ndarray — [vx, vy, vz, wx, wy, wz] in mm/s, rad/s
    accel : (6,) ndarray — [ax, ay, az, alphax, alphay, alphaz] in mm/s², rad/s²
    geom : StewartGeometry
    dynamics_params : DynamicsParams
    feedforward_enabled : bool
        Whether to include torque_ff (gravity + inertia).
    gravity_correction : (3,3) ndarray or None
        Pre-multiply rotation to align platform with gravity.

    Returns
    -------
    pos_rev : (6,) ndarray — motor positions in revolutions
    vel_ff_rps : (6,) ndarray — velocity feedforward in rev/s
    torque_ff_Nm : (6,) ndarray — torque feedforward in Nm
    """
    pos = pose[:3]
    rot = rotvec_to_rot_matrix(pose[3:6])
    if gravity_correction is not None:
        rot = gravity_correction @ rot

    # Position IK: pose -> leg extensions (mm) -> motor revolutions
    extensions_mm = pose_to_leg_lengths(pos, rot, geom)
    pos_rev = extensions_mm_to_revs(extensions_mm, geom)

    # Velocity IK: twist -> leg velocities (mm/s) -> motor rev/s
    vel_mm_s = twist_to_leg_velocities(twist, pos, rot, geom)
    vel_ff_rps = leg_velocities_to_motor_velocities(vel_mm_s, geom)

    # Torque feedforward: gravity + platform inertia + reflected motor inertia
    if feedforward_enabled:
        torque_ff_Nm = compute_full_feedforward_torques(
            pos, rot, twist, accel, geom, dynamics_params)
    else:
        torque_ff_Nm = np.zeros(6)

    return pos_rev, vel_ff_rps, torque_ff_Nm


# ---------------------------------------------------------------------------
# Feasibility checker
# ---------------------------------------------------------------------------

@dataclass
class FeasibilityResult:
    """Result of trajectory feasibility analysis."""
    feasible: bool
    peak_leg_vel_rps: np.ndarray
    peak_leg_accel_rps2: np.ndarray
    max_extension_mm: np.ndarray
    min_extension_mm: np.ndarray
    peak_condition_number: float
    peak_torque_ff_Nm: np.ndarray
    peak_jerk_trans: float        # peak translational jerk (mm/s^3)
    peak_jerk_rot: float          # peak rotational jerk (rad/s^3)
    violations: list
    n_samples: int


def check_feasibility(
    traj: QuinticTrajectory,
    geom: StewartGeometry,
    dynamics_params: DynamicsParams,
    vel_limit_rps: float | None = None,
    accel_limit_rps2: float | None = None,
    condition_limit: float | None = None,
    torque_limit_Nm: float | None = None,
    jerk_trans_limit: float | None = 50_000.0,
    jerk_rot_limit: float | None = 400.0,
    n_samples: int = 200,
    early_exit: bool = False,
    yield_interval: int = 0,
) -> FeasibilityResult:
    """Check whether a trajectory respects kinematic and dynamic limits.

    Evaluates the trajectory at ``n_samples`` evenly-spaced time points
    and checks each against the specified limits.

    Parameters
    ----------
    traj : QuinticTrajectory
    geom : StewartGeometry
    dynamics_params : DynamicsParams
    vel_limit_rps : float or None
        Maximum leg velocity in rev/s.  Defaults to 15.0 (from config).
    accel_limit_rps2 : float or None
        Maximum leg acceleration in rev/s^2.  Defaults to 30.0 (from config).
    condition_limit : float or None
        Maximum Jacobian condition number.  Defaults to 2.0 * cond(J_home),
        which accounts for the mixed mm/rad units giving raw cond ~450.
    torque_limit_Nm : float or None
        Maximum feedforward torque per motor.  If None, skip torque check.
    jerk_trans_limit : float or None
        Maximum translational Cartesian jerk in mm/s^3.  Defaults to 50000.
        If None, skip jerk check.  NOTE: per-leg jerk is NOT checked —
        the condition number constraint guards against poses where smooth
        Cartesian motion maps to jerky leg motion.
    jerk_rot_limit : float or None
        Maximum rotational Cartesian jerk in rad/s^3.  Defaults to 400.
        If None, skip jerk check.
    n_samples : int
        Number of evenly-spaced points to evaluate.
    early_exit : bool
        If True, return immediately on the first violation found.
        Peak values in the result will reflect only the samples evaluated
        before the violation.  Useful for binary-search where only the
        feasible/infeasible verdict matters.
    yield_interval : int
        If > 0, call ``time.sleep(0)`` every *yield_interval* samples to
        release the GIL so that other threads (e.g. the 500 Hz control
        loop) can run between chunks of computation.  Default 0 (no yield).

    Returns
    -------
    FeasibilityResult
    """
    import jugglebot.hardware_config as hw

    if vel_limit_rps is None:
        vel_limit_rps = float(hw.ODRIVE_TRAP_VEL_LIMIT_RPS)
    if accel_limit_rps2 is None:
        accel_limit_rps2 = float(hw.ODRIVE_TRAP_ACC_LIMIT_RPS2)
    if condition_limit is None:
        # Use relative threshold: 2x the condition number at home pose
        cond_home = compute_condition_number(np.zeros(3), np.eye(3), geom)
        condition_limit = 2.0 * cond_home

    violations = []
    times = np.linspace(traj.t_start, traj.t_start + traj.duration, n_samples)
    # Use workspace hard limits (5mm margin from each end) so that
    # feasibility agrees with the runtime workspace monitor.
    from jugglebot.motion.workspace import LEG_HARD_MARGIN_MM
    stroke_min = LEG_HARD_MARGIN_MM
    stroke_max = geom.leg_stroke_mm - LEG_HARD_MARGIN_MM

    # Accumulators
    peak_vel = np.zeros(6)
    peak_accel = np.zeros(6)
    max_ext = np.full(6, -np.inf)
    min_ext = np.full(6, np.inf)
    peak_cond = 0.0
    peak_torque = np.zeros(6)
    peak_jerk_t = 0.0  # peak translational jerk magnitude (mm/s^3)
    peak_jerk_r = 0.0  # peak rotational jerk magnitude (rad/s^3)

    check_jerk = (jerk_trans_limit is not None or jerk_rot_limit is not None)
    n_evaluated = 0

    for t in times:
        pose, twist, accel_cart = evaluate(traj, t)
        pos = pose[:3]
        rot = rotvec_to_rot_matrix(pose[3:6])
        n_evaluated += 1

        # Periodically release the GIL so the control loop can run
        if yield_interval > 0 and n_evaluated % yield_interval == 0:
            _time.sleep(0)

        # Leg extensions (mm) — check stroke limits
        extensions_mm = pose_to_leg_lengths(pos, rot, geom)
        max_ext = np.maximum(max_ext, extensions_mm)
        min_ext = np.minimum(min_ext, extensions_mm)

        # Leg velocities (rev/s)
        vel_mm_s = twist_to_leg_velocities(twist, pos, rot, geom)
        vel_rps = leg_velocities_to_motor_velocities(vel_mm_s, geom)
        peak_vel = np.maximum(peak_vel, np.abs(vel_rps))

        # Leg accelerations (rev/s^2)
        accel_mm_s2 = accel_to_leg_accels(accel_cart, twist, pos, rot, geom)
        accel_rps2 = accel_mm_s2 * geom.mm_to_rev
        peak_accel = np.maximum(peak_accel, np.abs(accel_rps2))

        # Jacobian condition number
        cond = compute_condition_number(pos, rot, geom)
        peak_cond = max(peak_cond, cond)

        # Full feedforward torques (gravity + inertia + reflected motor)
        # Skip when no torque limit — the result would never be checked.
        if torque_limit_Nm is not None:
            torque_Nm = compute_full_feedforward_torques(
                pos, rot, twist, accel_cart, geom, dynamics_params)
            peak_torque = np.maximum(peak_torque, np.abs(torque_Nm))

        # Cartesian jerk (3rd derivative)
        if check_jerk:
            jerk = evaluate_jerk(traj, t)
            jerk_t_mag = float(np.max(np.abs(jerk[:3])))
            jerk_r_mag = float(np.max(np.abs(jerk[3:])))
            peak_jerk_t = max(peak_jerk_t, jerk_t_mag)
            peak_jerk_r = max(peak_jerk_r, jerk_r_mag)

        # Early exit: check per-sample limits and break on first violation
        if early_exit:
            if (np.any(extensions_mm < stroke_min)
                    or np.any(extensions_mm > stroke_max)):
                violations.append("stroke_early_exit")
                break
            if np.any(np.abs(vel_rps) > vel_limit_rps):
                violations.append("velocity_early_exit")
                break
            if np.any(np.abs(accel_rps2) > accel_limit_rps2):
                violations.append("acceleration_early_exit")
                break
            if cond > condition_limit:
                violations.append("condition_early_exit")
                break
            if torque_limit_Nm is not None and np.any(
                    np.abs(peak_torque) > torque_limit_Nm):
                violations.append("torque_early_exit")
                break
            if jerk_trans_limit is not None and peak_jerk_t > jerk_trans_limit:
                violations.append("jerk_trans_early_exit")
                break
            if jerk_rot_limit is not None and peak_jerk_r > jerk_rot_limit:
                violations.append("jerk_rot_early_exit")
                break

    # Post-loop violation analysis (skipped when early_exit already found one)
    if not violations:
        if np.any(min_ext < stroke_min):
            bad_legs = np.where(min_ext < stroke_min)[0]
            violations.append(
                f"Underextended legs {bad_legs.tolist()}: "
                f"min extensions {min_ext[bad_legs].tolist()} mm "
                f"(limit {stroke_min:.1f} mm)")
        if np.any(max_ext > stroke_max):
            bad_legs = np.where(max_ext > stroke_max)[0]
            violations.append(
                f"Overextended legs {bad_legs.tolist()}: "
                f"max extensions {max_ext[bad_legs].tolist()} mm "
                f"(limit {stroke_max:.1f} mm)")

        if np.any(peak_vel > vel_limit_rps):
            bad_legs = np.where(peak_vel > vel_limit_rps)[0]
            violations.append(
                f"Velocity limit ({vel_limit_rps} rev/s) exceeded on legs "
                f"{bad_legs.tolist()}: peak {peak_vel[bad_legs].tolist()} rev/s")

        if np.any(peak_accel > accel_limit_rps2):
            bad_legs = np.where(peak_accel > accel_limit_rps2)[0]
            violations.append(
                f"Acceleration limit ({accel_limit_rps2} rev/s^2) exceeded on legs "
                f"{bad_legs.tolist()}: peak {peak_accel[bad_legs].tolist()} rev/s^2")

        if peak_cond > condition_limit:
            violations.append(
                f"Jacobian condition number ({peak_cond:.1f}) exceeds limit "
                f"({condition_limit:.1f})")

        if torque_limit_Nm is not None and np.any(peak_torque > torque_limit_Nm):
            bad_legs = np.where(peak_torque > torque_limit_Nm)[0]
            violations.append(
                f"Torque limit ({torque_limit_Nm} Nm) exceeded on legs "
                f"{bad_legs.tolist()}: peak {peak_torque[bad_legs].tolist()} Nm")

        if jerk_trans_limit is not None and peak_jerk_t > jerk_trans_limit:
            violations.append(
                f"Translational jerk limit ({jerk_trans_limit:.0f} mm/s^3) "
                f"exceeded: peak {peak_jerk_t:.0f} mm/s^3")
        if jerk_rot_limit is not None and peak_jerk_r > jerk_rot_limit:
            violations.append(
                f"Rotational jerk limit ({jerk_rot_limit:.0f} rad/s^3) "
                f"exceeded: peak {peak_jerk_r:.0f} rad/s^3")

    return FeasibilityResult(
        feasible=len(violations) == 0,
        peak_leg_vel_rps=peak_vel,
        peak_leg_accel_rps2=peak_accel,
        max_extension_mm=max_ext,
        min_extension_mm=min_ext,
        peak_condition_number=peak_cond,
        peak_torque_ff_Nm=peak_torque,
        peak_jerk_trans=peak_jerk_t,
        peak_jerk_rot=peak_jerk_r,
        violations=violations,
        n_samples=n_samples,
    )


# ---------------------------------------------------------------------------
# Convenience constructors
# ---------------------------------------------------------------------------

def make_rest_to_rest(
    end_pose: np.ndarray,
    duration: float,
    speed_scale: float = 1.0,
    start_pose: np.ndarray | None = None,
    t_start: float = 0.0,
) -> QuinticTrajectory:
    """Create a rest-to-rest quintic trajectory (zero twist/accel at both ends).

    Parameters
    ----------
    end_pose : (6,) ndarray — [x, y, z, rx, ry, rz] in mm, rad
    duration : float — base duration in seconds (before speed scaling)
    speed_scale : float — speed scaling factor in (0, 1]
    start_pose : (6,) ndarray or None — defaults to home (zeros)
    t_start : float — absolute start time reference (seconds)
    """
    zeros6 = np.zeros(6)
    sp = np.asarray(start_pose, dtype=np.float64) if start_pose is not None \
        else zeros6.copy()
    ep = np.asarray(end_pose, dtype=np.float64)
    return create_trajectory(
        start_pose=sp, start_twist=zeros6, start_accel=zeros6,
        end_pose=ep, end_twist=zeros6, end_accel=zeros6,
        duration=duration, t_start=t_start, speed_scale=speed_scale)


def find_min_feasible_duration(
    start_pose: np.ndarray,
    start_twist: np.ndarray,
    start_accel: np.ndarray,
    end_pose: np.ndarray,
    end_twist: np.ndarray,
    end_accel: np.ndarray,
    geom: StewartGeometry,
    dynamics_params: DynamicsParams,
    speed_scale: float = 1.0,
    min_T: float = 0.2,
    max_T: float = 5.0,
    n_bisections: int = 8,
    n_feas_samples: int = 50,
) -> float | None:
    """Binary search for the minimum feasible trajectory duration.

    Creates trajectories with progressively shorter durations and checks
    each against the full feasibility suite (stroke, velocity, acceleration,
    condition number, jerk).

    Parameters
    ----------
    start_pose, start_twist, start_accel : (6,) ndarray
        Start boundary conditions.
    end_pose, end_twist, end_accel : (6,) ndarray
        End boundary conditions.
    geom : StewartGeometry
    dynamics_params : DynamicsParams
    speed_scale : float
    min_T : float
        Minimum search bound (seconds).
    max_T : float
        Maximum search bound (seconds).  If even this is infeasible,
        returns None.
    n_bisections : int
        Number of binary search iterations.
    n_feas_samples : int
        Number of samples per feasibility check.

    Returns
    -------
    duration : float or None
        Shortest feasible duration found, or None if ``max_T`` is infeasible.
    """
    # First check: is max_T feasible?
    traj = create_trajectory(
        start_pose=start_pose, start_twist=start_twist, start_accel=start_accel,
        end_pose=end_pose, end_twist=end_twist, end_accel=end_accel,
        duration=max_T, speed_scale=speed_scale)
    result = check_feasibility(
        traj, geom, dynamics_params, n_samples=n_feas_samples, early_exit=True)
    if not result.feasible:
        return None

    lo, hi = min_T, max_T
    for _ in range(n_bisections):
        mid = (lo + hi) / 2.0
        traj = create_trajectory(
            start_pose=start_pose, start_twist=start_twist, start_accel=start_accel,
            end_pose=end_pose, end_twist=end_twist, end_accel=end_accel,
            duration=mid, speed_scale=speed_scale)
        result = check_feasibility(
            traj, geom, dynamics_params, n_samples=n_feas_samples, early_exit=True)
        if result.feasible:
            hi = mid
        else:
            lo = mid

    return hi


# ---------------------------------------------------------------------------
# Deceleration trajectory
# ---------------------------------------------------------------------------

def make_deceleration_trajectory(
    start_pose: np.ndarray,
    start_twist: np.ndarray,
    start_accel: np.ndarray,
    geom: 'StewartGeometry',
    vel_limit_rps: float | None = None,
    accel_limit_rps2: float | None = None,
    t_start: float = 0.0,
    duration_margin: float = 1.5,
) -> QuinticTrajectory | None:
    """Create a quintic that decelerates from *start_twist* to rest.

    The end pose is wherever the polynomial lands — the caller must run a
    feasibility check to verify the stopping point is within the workspace.

    Parameters
    ----------
    start_pose, start_twist, start_accel : (6,) ndarray
        Current Cartesian state.
    geom : StewartGeometry
    vel_limit_rps, accel_limit_rps2 : float or None
        Motor limits.  Defaults loaded from hardware_config.
    t_start : float
        Absolute start time for the trajectory.
    duration_margin : float
        Safety margin applied to the minimum computed duration.

    Returns
    -------
    QuinticTrajectory or None
        None if start_twist is effectively zero (no deceleration needed).
    """
    import jugglebot.hardware_config as hw

    if vel_limit_rps is None:
        vel_limit_rps = float(hw.ODRIVE_TRAP_VEL_LIMIT_RPS)
    if accel_limit_rps2 is None:
        accel_limit_rps2 = float(hw.ODRIVE_TRAP_ACC_LIMIT_RPS2)

    start_pose = np.asarray(start_pose, dtype=np.float64)
    start_twist = np.asarray(start_twist, dtype=np.float64)
    start_accel = np.asarray(start_accel, dtype=np.float64)

    # Convert Cartesian twist to motor velocities to find peak motor speed
    pos = start_pose[:3]
    rot = rotvec_to_rot_matrix(start_pose[3:6])
    vel_mm_s = twist_to_leg_velocities(start_twist, pos, rot, geom)
    vel_rps = leg_velocities_to_motor_velocities(vel_mm_s, geom)

    v_max_motor = np.max(np.abs(vel_rps))
    if v_max_motor < 1e-6:
        return None  # already at rest

    # Quintic deceleration from v0 to 0: peak acceleration = 10*v0/(3*T).
    # Solve for T: T_accel = 10 * v_max / (3 * accel_limit).
    # The peak velocity is bounded by v0 (starting velocity), which was
    # already validated by the outbound trajectory's feasibility check.
    t_accel = 10.0 * v_max_motor / (3.0 * accel_limit_rps2)

    T = max(t_accel, 0.1) * duration_margin

    # Estimate end pose: for a smooth deceleration the displacement is
    # approximately twist * T * 0.5 (average velocity over the interval).
    # The quintic solver produces the exact polynomial regardless — this
    # just sets the boundary condition.
    end_pose = start_pose + start_twist * T * 0.5

    try:
        return create_trajectory(
            start_pose=start_pose,
            start_twist=start_twist,
            start_accel=start_accel,
            end_pose=end_pose,
            end_twist=np.zeros(6),
            end_accel=np.zeros(6),
            duration=T,
            t_start=t_start,
        )
    except ValueError:
        return None


# ---------------------------------------------------------------------------
# Trajectory manager
# ---------------------------------------------------------------------------

class TrajectoryState(enum.Enum):
    IDLE = 'idle'
    EXECUTING = 'executing'
    RETURNING = 'returning'  # decelerating to stop after non-zero-velocity target
    COMPLETE = 'complete'


class TrajectoryManager:
    """Manages trajectory execution within the control loop.

    Lifecycle:
        1. Set a hold pose via ``set_hold_pose()`` (typically home).
        2. Submit a trajectory via ``submit()`` or ``submit_dynamic_target()``.
        3. Each control cycle, call ``evaluate(t)`` to get motor commands.
        4. When the trajectory completes:
           - Zero-velocity targets → COMPLETE (hold at target).
           - Non-zero-velocity targets → RETURNING (decelerate to stop).
        5. Submit a new trajectory at any time (mid-motion replanning).

    Phase 6 additions:
        - ``progress`` and ``time_remaining`` are now live (updated on evaluate).
        - ``current_pose_6dof`` exposes the latest evaluated Cartesian pose.

    Phase 7 additions:
        - ``submit_dynamic_target()`` for fire-and-forget target commanding.
        - ``RETURNING`` state for deceleration-to-stop after non-zero-velocity
          targets.
        - Mid-motion replanning: ``submit()`` and ``submit_dynamic_target()`` can
          be called during EXECUTING or RETURNING.
    """

    def __init__(self, geom: StewartGeometry, dynamics_params: DynamicsParams,
                 clock=None):
        import jugglebot.hardware_config as hw

        self.geom = geom
        self.dynamics_params = dynamics_params
        self._state = TrajectoryState.IDLE
        self._active_traj: QuinticTrajectory | None = None
        self._feedforward_enabled = True

        # Hold pose for IDLE/COMPLETE states
        self._hold_pose = np.zeros(6)  # [x,y,z,rx,ry,rz]
        self._hold_rot = np.eye(3)
        self._hold_pos_rev = np.zeros(6)
        self._hold_torque_ff = np.zeros(6)

        # Home pose (used as reference, no longer auto-returned to)
        self._home_pose = np.array(
            [0.0, 0.0, float(hw.JB_OP_DEFAULT_ACTIVE_Z_MM), 0.0, 0.0, 0.0])
        self._pending_decel = False  # True when trajectory end twist != 0
        self._held_at_end_cycles = 0  # cycles spent holding at end pose waiting for decel precompute

        # Gravity correction (applied before IK)
        self._gravity_correction: np.ndarray | None = None

        # Progress tracking (Phase 6)
        self._last_progress = 0.0
        self._last_time_remaining = 0.0
        self._current_pose = np.zeros(6)  # latest evaluated [x,y,z,rx,ry,rz]

        # Clock function for measuring elapsed time during expensive
        # operations (feasibility checks, binary search).  Defaults to
        # wall-clock time; offline tests can inject a synthetic clock so
        # that restamp deltas stay in the correct time domain.
        self._clock = clock or _time.perf_counter

        # --- Async feasibility pipeline ---
        # Feasibility checks run in a dedicated child process to bypass
        # the GIL.  Communication via multiprocessing.Pipe.
        from jugglebot.motion.feasibility_worker import FeasibilityWorkerProxy
        self._worker = FeasibilityWorkerProxy(
            geom, dynamics_params,
            max_restarts=hw.JB_OP_FEASIBILITY_WORKER_MAX_RESTARTS)
        self._async_generation = 0                    # monotonic counter
        self._pending_result: dict | None = None      # buffered feasibility result
        self._precomputed_decel: QuinticTrajectory | None = None
        self._decel_generation = 0

    @property
    def state(self) -> TrajectoryState:
        return self._state

    @property
    def progress(self) -> float:
        """Fraction of trajectory completed (0.0 to 1.0)."""
        return self._last_progress

    @property
    def time_remaining(self) -> float:
        """Seconds remaining in active trajectory, or 0.0."""
        return self._last_time_remaining

    @property
    def current_pose_6dof(self) -> np.ndarray:
        """Latest evaluated Cartesian pose [x,y,z,rx,ry,rz]."""
        return self._current_pose.copy()

    @property
    def home_pose(self) -> np.ndarray:
        """Home pose (reference position)."""
        return self._home_pose.copy()

    def set_feedforward_enabled(self, enabled: bool) -> None:
        """Toggle gravity feedforward for trajectory evaluation."""
        self._feedforward_enabled = enabled

    def set_gravity_correction(self, correction: np.ndarray) -> None:
        """Set gravity correction rotation matrix. Recomputes hold pose IK."""
        self._gravity_correction = correction
        self.set_hold_pose(self._hold_pose)

    def set_hold_pose(self, pose_6dof: np.ndarray) -> None:
        """Set the hold pose for IDLE/COMPLETE states.

        Parameters
        ----------
        pose_6dof : (6,) ndarray — [x, y, z, rx, ry, rz] in mm, rad
        """
        self._hold_pose = np.asarray(pose_6dof, dtype=np.float64)
        rot = rotvec_to_rot_matrix(self._hold_pose[3:6])
        if self._gravity_correction is not None:
            rot = self._gravity_correction @ rot
        self._hold_rot = rot
        pos = self._hold_pose[:3]
        extensions_mm = pose_to_leg_lengths(pos, rot, self.geom)
        self._hold_pos_rev = extensions_mm_to_revs(extensions_mm, self.geom)
        if self._feedforward_enabled:
            self._hold_torque_ff = gravity_to_motor_torques(
                pos, rot, self.geom, self.dynamics_params)
        else:
            self._hold_torque_ff = np.zeros(6)

    def submit(self, traj: QuinticTrajectory,
               is_decel: bool = False) -> None:
        """Submit a trajectory for execution.

        Can be called from any state, including during EXECUTING or
        RETURNING (mid-motion replanning).  The caller must ensure the
        trajectory starts from the correct current state at the splice
        time for C2 continuity.

        Parameters
        ----------
        traj : QuinticTrajectory
            Must have passed feasibility checking before submission.
        is_decel : bool
            If True, enter RETURNING state (deceleration to stop).
        """
        self._active_traj = traj
        self._state = TrajectoryState.RETURNING if is_decel \
            else TrajectoryState.EXECUTING
        self._pending_decel = False
        self._held_at_end_cycles = 0
        logger.info(
            f"Trajectory submitted ({'decel' if is_decel else 'target'}): "
            f"duration={traj.duration:.3f}s, "
            f"speed_scale={traj.speed_scale:.2f}")

    def restamp_active_trajectory(self, elapsed: float) -> None:
        """Shift the active trajectory's t_start forward by *elapsed* seconds.

        Call this after ``submit_dynamic_target()`` (or any operation that
        blocks the control loop) to compensate for wall-clock time consumed
        by the feasibility check.  The polynomial uses normalised time, so
        only the absolute time anchor changes — the trajectory shape is
        unaffected.
        """
        if self._active_traj is not None and elapsed > 0:
            self._active_traj = _dc_replace(
                self._active_traj,
                t_start=self._active_traj.t_start + elapsed,
            )

    def submit_dynamic_target(
        self,
        target_pos: np.ndarray,
        target_quat: np.ndarray,
        target_vel: np.ndarray,
        arrival_time: float,
        t_now: float,
    ) -> bool:
        """Submit a dynamic target command.

        Automatically handles:
        - Sampling current state for splice continuity
        - Quaternion -> rotation vector conversion
        - Feasibility checking (stroke, vel, accel, cond, jerk)
        - Queuing deceleration-to-stop if target velocity is non-zero

        The trajectory uses the full requested duration (arrival_time -
        t_now).  The caller controls the speed: a longer duration gives a
        slower, smoother trajectory; a shorter one gives a faster one.
        Feasibility checking rejects durations that are too short.

        Parameters
        ----------
        target_pos : (3,) ndarray — [x, y, z] in mm
        target_quat : (4,) ndarray — [w, x, y, z] quaternion
        target_vel : (3,) ndarray — [vx, vy, vz] in mm/s (linear only;
            angular velocity is always zero)
        arrival_time : float — absolute arrival time (same clock domain as t_now)
        t_now : float — current time (from the injected clock)

        Returns
        -------
        accepted : bool
            True if the target was accepted and a trajectory planned.
        """
        from jugglebot.motion.ik_solver import (
            quat_to_rot_matrix,
            rot_matrix_to_rotvec,
        )

        # Compute duration from absolute arrival time
        duration = arrival_time - t_now
        if duration < MIN_LEAD_TIME_S:
            logger.warning(
                f"Dynamic target rejected: lead time {duration:.3f}s "
                f"< {MIN_LEAD_TIME_S}s minimum")
            return False

        # Convert quaternion orientation to rotation vector
        w, x, y, z = target_quat
        rot_mat = quat_to_rot_matrix(w, x, y, z)
        rotvec = rot_matrix_to_rotvec(rot_mat)

        # Compose 6-DoF target pose and twist (angular velocity = 0)
        target_pose = np.array([
            target_pos[0], target_pos[1], target_pos[2],
            rotvec[0], rotvec[1], rotvec[2],
        ])
        target_twist = np.array([
            target_vel[0], target_vel[1], target_vel[2],
            0.0, 0.0, 0.0,
        ])

        # Measure wall-clock time consumed by trajectory creation and
        # feasibility checking so we can restamp t_start forward,
        # preventing a position jump on the first motor command.
        t_clock_start = self._clock()

        cur_pose, cur_twist, cur_accel = self._get_current_state(t_now)

        try:
            traj = create_trajectory(
                start_pose=cur_pose,
                start_twist=cur_twist,
                start_accel=cur_accel,
                end_pose=target_pose,
                end_twist=target_twist,
                end_accel=np.zeros(6),
                duration=duration,
                t_start=t_now,
            )
        except ValueError as e:
            logger.debug(f"Dynamic target rejected: {e}")
            return False

        # Feasibility check (reduced samples for inline speed).
        result = check_feasibility(
            traj, self.geom, self.dynamics_params, n_samples=50)
        if not result.feasible:
            logger.debug(
                f"Dynamic target rejected (infeasible): "
                f"{'; '.join(result.violations)}")
            return False

        # Shift t_start forward by wall-clock time consumed by this method
        # so the trajectory starts "now" rather than in the past.
        total_elapsed = self._clock() - t_clock_start
        if total_elapsed > 0:
            traj = _dc_replace(
                traj, t_start=traj.t_start + total_elapsed)

        # Accept: submit trajectory and flag deceleration if needed
        self.submit(traj)
        self._pending_decel = bool(np.linalg.norm(target_vel) > 1e-6)

        # Pre-compute deceleration in background if needed
        if self._pending_decel:
            self._start_decel_precompute(traj)

        return True

    def _get_current_state(
        self, t: float,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Get the current Cartesian state for trajectory planning.

        Returns
        -------
        pose : (6,) ndarray — [x, y, z, rx, ry, rz]
        twist : (6,) ndarray — [vx, vy, vz, wx, wy, wz]
        accel : (6,) ndarray — [ax, ay, az, alphax, alphay, alphaz]
        """
        if self._state in (TrajectoryState.EXECUTING, TrajectoryState.RETURNING):
            return evaluate(self._active_traj, t)
        else:
            # IDLE or COMPLETE: at hold pose, zero velocity/acceleration
            return self._hold_pose.copy(), np.zeros(6), np.zeros(6)

    def evaluate(self, t: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Evaluate the current target for the control loop.

        Parameters
        ----------
        t : float
            Current time in seconds (control loop clock).

        Returns
        -------
        pos_rev : (6,) ndarray — motor positions in revolutions
        vel_ff_rps : (6,) ndarray — velocity feedforward in rev/s
        torque_ff_Nm : (6,) ndarray — torque feedforward in Nm
        """
        if self._state in (TrajectoryState.IDLE, TrajectoryState.COMPLETE):
            self._current_pose = self._hold_pose.copy()
            return (self._hold_pos_rev.copy(),
                    np.zeros(6),
                    self._hold_torque_ff.copy())

        # EXECUTING or RETURNING
        traj = self._active_traj
        t_end = traj.t_start + traj.duration

        if t >= t_end:
            # Trajectory complete
            self._last_progress = 1.0
            self._last_time_remaining = 0.0
            end_pose = traj.end_state[:6]
            self._current_pose = end_pose.copy()

            if self._state == TrajectoryState.RETURNING:
                # Deceleration complete → IDLE at wherever we stopped
                self.set_hold_pose(end_pose)
                self._state = TrajectoryState.IDLE
                self._active_traj = None
                logger.info("Deceleration complete")
                return (self._hold_pos_rev.copy(),
                        np.zeros(6),
                        self._hold_torque_ff.copy())

            # EXECUTING complete
            if self._pending_decel:
                # Check for pre-computed deceleration trajectory (non-blocking)
                precomputed = self.poll_precomputed_decel()
                if precomputed is not None:
                    if self._held_at_end_cycles == 0:
                        # First cycle after outbound ends — use precomputed
                        # trajectory directly for C2 continuity (its start
                        # state matches the outbound's end state).
                        decel_traj = _dc_replace(precomputed, t_start=t)
                    else:
                        # Platform has been held at rest for multiple cycles.
                        # Create a new rest-to-rest deceleration at the
                        # precomputed duration since the platform is now
                        # physically stationary.
                        decel_traj = create_trajectory(
                            start_pose=end_pose,
                            start_twist=np.zeros(6),
                            start_accel=np.zeros(6),
                            end_pose=end_pose,
                            end_twist=np.zeros(6),
                            end_accel=np.zeros(6),
                            duration=precomputed.duration,
                            t_start=t,
                        )
                    self.submit(decel_traj, is_decel=True)
                    pose, twist, accel_cart = evaluate(self._active_traj, t)
                    self._current_pose = pose.copy()
                    return cartesian_to_motor_commands(
                        pose, twist, accel_cart,
                        self.geom, self.dynamics_params,
                        self._feedforward_enabled,
                        self._gravity_correction)
                else:
                    # Pre-computed deceleration not ready yet — hold at end
                    # pose and wait for next cycle.  Keep _pending_decel True
                    # and stay in EXECUTING so we re-enter this branch.
                    self._held_at_end_cycles += 1
                    self.set_hold_pose(end_pose)
                    return (self._hold_pos_rev.copy(),
                            np.zeros(6),
                            self._hold_torque_ff.copy())

            # Zero-velocity target or failed deceleration → COMPLETE
            self._state = TrajectoryState.COMPLETE
            self._active_traj = None
            self.set_hold_pose(end_pose)
            logger.info("Trajectory complete")
            return (self._hold_pos_rev.copy(),
                    np.zeros(6),
                    self._hold_torque_ff.copy())

        # Before trajectory starts: hold at start pose
        if t < traj.t_start:
            self._last_progress = 0.0
            self._last_time_remaining = t_end - t
            pose = traj.start_state[:6]
            self._current_pose = pose.copy()
            return cartesian_to_motor_commands(
                pose, np.zeros(6), np.zeros(6),
                self.geom, self.dynamics_params,
                self._feedforward_enabled,
                self._gravity_correction)

        # Mid-trajectory: evaluate at current time
        elapsed = t - traj.t_start
        self._last_progress = min(1.0, elapsed / traj.duration)
        self._last_time_remaining = max(0.0, t_end - t)

        pose, twist, accel_cart = evaluate(traj, t)
        self._current_pose = pose.copy()
        return cartesian_to_motor_commands(
            pose, twist, accel_cart,
            self.geom, self.dynamics_params,
            self._feedforward_enabled,
            self._gravity_correction)

    def cancel(self) -> None:
        """Cancel the active trajectory and transition to IDLE.

        The hold pose remains at whatever was last set (typically
        the end pose from a previous trajectory or home).
        """
        if self._state in (TrajectoryState.EXECUTING, TrajectoryState.RETURNING):
            logger.info(f"Trajectory cancelled (was {self._state.value})")
        self._active_traj = None
        self._state = TrajectoryState.IDLE
        self._pending_decel = False
        self._held_at_end_cycles = 0
        self._last_progress = 0.0
        self._last_time_remaining = 0.0
        # Discard any pending async results
        self._async_generation += 1
        self._pending_result = None
        self._precomputed_decel = None

    def shutdown(self) -> None:
        """Shut down the feasibility worker process."""
        self._worker.shutdown()

    def try_restart_worker(self) -> bool:
        """Attempt to restart the feasibility worker process.

        Returns True if restart succeeded, False if restart limit reached.
        """
        return self._worker.try_restart()

    # ------------------------------------------------------------------
    # Async feasibility pipeline
    # ------------------------------------------------------------------

    def request_dynamic_target(
        self,
        target_pos: np.ndarray,
        target_quat: np.ndarray,
        target_vel: np.ndarray,
        arrival_time: float,
        t_now: float,
    ) -> bool:
        """Queue a dynamic target for background feasibility checking.

        Non-blocking.  The control loop should call ``poll_pending_result()``
        each cycle to check for a completed result.

        Parameters
        ----------
        target_pos : (3,) ndarray — [x, y, z] in mm
        target_quat : (4,) ndarray — [w, x, y, z] quaternion
        target_vel : (3,) ndarray — [vx, vy, vz] in mm/s
        arrival_time : float — absolute arrival time (same clock domain as t_now)
        t_now : float — current time (from the injected clock)

        Returns
        -------
        queued : bool
            True if the request was queued for background processing.
            False if rejected immediately (bad duration, trajectory creation
            failure).  When False, no ``poll_pending_result()`` will follow.
        """
        from jugglebot.motion.ik_solver import (
            quat_to_rot_matrix,
            rot_matrix_to_rotvec,
        )

        duration = arrival_time - t_now
        if duration < MIN_LEAD_TIME_S:
            logger.warning(
                f"Dynamic target rejected: lead time {duration:.3f}s "
                f"< {MIN_LEAD_TIME_S}s minimum")
            return False

        # Convert quaternion to rotation vector
        w, x, y, z = target_quat
        rot_mat = quat_to_rot_matrix(w, x, y, z)
        rotvec = rot_matrix_to_rotvec(rot_mat)

        target_pose = np.array([
            target_pos[0], target_pos[1], target_pos[2],
            rotvec[0], rotvec[1], rotvec[2],
        ])
        target_twist = np.array([
            target_vel[0], target_vel[1], target_vel[2],
            0.0, 0.0, 0.0,
        ])

        # Sample current state on the main thread (thread-safe snapshot)
        cur_pose, cur_twist, cur_accel = self._get_current_state(t_now)

        try:
            traj = create_trajectory(
                start_pose=cur_pose,
                start_twist=cur_twist,
                start_accel=cur_accel,
                end_pose=target_pose,
                end_twist=target_twist,
                end_accel=np.zeros(6),
                duration=duration,
                t_start=t_now,
            )
        except ValueError as e:
            logger.debug(f"Dynamic target rejected: {e}")
            return False

        needs_decel = bool(np.linalg.norm(target_vel) > 1e-6)

        self._async_generation += 1
        self._pending_result = None
        request = {
            'type': 'feasibility',
            'traj': traj,
            'needs_decel': needs_decel,
            'arrival_time': arrival_time,
            'generation': self._async_generation,
            't_request': t_now,
        }
        if not self._worker.submit(request):
            logger.warning("Feasibility worker dead, rejecting dynamic target")
            return False
        return True

    def poll_pending_result(self) -> dict | None:
        """Check for a completed async feasibility result.

        Non-blocking.  Returns None if no result is ready.

        Returns
        -------
        result : dict or None
            If ready: ``{'accepted': bool, 'traj': QuinticTrajectory,
            'needs_decel': bool, 'generation': int}``

        Raises
        ------
        WorkerCrashed
            If the feasibility worker process has died.
        """
        # Return buffered result first (from a previous poll that
        # received a feasibility result while draining decel results)
        if self._pending_result is not None:
            result = self._pending_result
            self._pending_result = None
            return result

        from jugglebot.motion.feasibility_worker import WorkerCrashed
        # May raise WorkerCrashed — caller handles
        result = self._worker.poll()
        if result is None:
            return None

        if result['type'] == 'decel':
            # Decel results go to the precomputed slot
            if (result['generation'] == self._decel_generation
                    and result.get('traj') is not None):
                self._precomputed_decel = result['traj']
                logger.info(
                    "Deceleration pre-computed: duration=%.3fs",
                    result['traj'].duration)
            return None  # not a feasibility result

        # Feasibility result — check generation
        if result['generation'] != self._async_generation:
            return None  # stale
        return result

    def commit_async_trajectory(self, result: dict) -> bool:
        """Commit a trajectory from a completed async feasibility check.

        Re-creates the trajectory from the current state at commit time to
        eliminate the timing gap between request and commit.

        Parameters
        ----------
        result : dict
            From ``poll_pending_result()``.

        Returns
        -------
        accepted : bool
            True if the trajectory was committed.
        """
        # Stale result: a newer request has been submitted
        if result['generation'] != self._async_generation:
            logger.debug("Async result discarded (stale generation)")
            return False

        if not result['accepted']:
            return False

        original_traj = result['traj']
        arrival_time = result['arrival_time']
        t_now = self._clock()

        # Remaining duration to arrival
        remaining = arrival_time - t_now
        if remaining <= 0.01:
            logger.debug("Async commit: arrival_time passed during check")
            return False

        # Re-sample state at current time (eliminates request→commit gap)
        cur_pose, cur_twist, cur_accel = self._get_current_state(t_now)

        try:
            traj = create_trajectory(
                start_pose=cur_pose,
                start_twist=cur_twist,
                start_accel=cur_accel,
                end_pose=original_traj.end_state[:6],
                end_twist=original_traj.end_state[6:12],
                end_accel=original_traj.end_state[12:18],
                duration=remaining,
                t_start=t_now,
            )
        except ValueError as e:
            logger.debug(f"Async commit trajectory creation failed: {e}")
            return False

        # Submit the trajectory
        self.submit(traj)
        self._pending_decel = result['needs_decel']

        # Pre-compute deceleration in background if needed
        if result['needs_decel']:
            self._start_decel_precompute(traj)

        logger.info(
            f"Async trajectory committed: remaining={remaining:.3f}s, "
            f"latency={t_now - result['t_request']:.3f}s")
        return True

    def poll_precomputed_decel(self) -> QuinticTrajectory | None:
        """Check if a pre-computed deceleration trajectory is available.

        Non-blocking.  Returns None if not ready or not applicable.
        Drains the worker pipe first so decel results are routed to the
        precomputed slot even if ``poll_pending_result()`` hasn't been
        called recently.
        """
        # Drain any pending results from the pipe — decel results get
        # routed to self._precomputed_decel by poll_pending_result().
        try:
            while True:
                result = self._worker.poll()
                if result is None:
                    break
                if result['type'] == 'decel':
                    if (result['generation'] == self._decel_generation
                            and result.get('traj') is not None):
                        self._precomputed_decel = result['traj']
                        logger.info(
                            "Deceleration pre-computed: duration=%.3fs",
                            result['traj'].duration)
                elif result['type'] == 'feasibility':
                    # Buffer feasibility results for poll_pending_result()
                    if result['generation'] == self._async_generation:
                        self._pending_result = result
        except Exception:
            pass  # worker may be dead; caller handles via poll_pending_result

        ret = self._precomputed_decel
        if ret is not None:
            self._precomputed_decel = None
        return ret

    def _start_decel_precompute(self, outbound_traj: QuinticTrajectory) -> None:
        """Queue background pre-computation of the deceleration trajectory.

        Called on the main thread after committing an outbound trajectory
        that has nonzero end velocity.
        """
        end_state = outbound_traj.end_state
        self._decel_generation += 1
        self._precomputed_decel = None
        request = {
            'type': 'decel',
            'generation': self._decel_generation,
            'start_pose': end_state[:6].copy(),
            'start_twist': end_state[6:12].copy(),
            'start_accel': end_state[12:18].copy(),
        }
        self._worker.submit(request)

