"""Trajectory feasibility checking and convenience constructors.

Validates trajectories against kinematic and dynamic limits (stroke,
velocity, acceleration, condition number, torque, jerk) and provides
helper functions for common trajectory patterns.

Provides:
  - ``FeasibilityResult`` / ``check_feasibility``: limit validation
  - ``make_rest_to_rest``: zero-velocity-at-both-ends shorthand
  - ``find_min_feasible_duration``: binary search for shortest safe duration
  - ``make_deceleration_trajectory``: quintic decel-to-stop builder

Pure Python + numpy.  No ROS2, no ZeroMQ dependency.
"""

from __future__ import annotations

import time as _time
from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

from jugglebot.motion.conversions import leg_velocities_to_motor_velocities
from jugglebot.motion.dynamics import (
    DynamicsParams,
    compute_full_feedforward_torques,
)
from jugglebot.motion.ik_solver import (
    accel_to_leg_accels,
    compute_jacobian,
    pose_to_leg_lengths,
    rotvec_to_rot_matrix,
    twist_to_leg_velocities,
)
from jugglebot.motion.quintic import (
    QuinticTrajectory,
    create_trajectory,
    evaluate,
    evaluate_jerk,
)
from jugglebot.motion.workspace import compute_condition_number

if TYPE_CHECKING:
    from jugglebot.motion.geometry import StewartGeometry


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

        # Compute Jacobian once per sample — reused by velocity IK,
        # acceleration IK, condition number, and torque computation.
        J = compute_jacobian(pos, rot, geom)

        # Leg extensions (mm) — check stroke limits
        extensions_mm = pose_to_leg_lengths(pos, rot, geom)
        max_ext = np.maximum(max_ext, extensions_mm)
        min_ext = np.minimum(min_ext, extensions_mm)

        # Leg velocities (rev/s)
        vel_mm_s = twist_to_leg_velocities(twist, pos, rot, geom, J=J)
        vel_rps = leg_velocities_to_motor_velocities(vel_mm_s, geom)
        peak_vel = np.maximum(peak_vel, np.abs(vel_rps))

        # Leg accelerations (rev/s^2)
        accel_mm_s2 = accel_to_leg_accels(accel_cart, twist, pos, rot, geom, J=J)
        accel_rps2 = accel_mm_s2 * geom.mm_to_rev
        peak_accel = np.maximum(peak_accel, np.abs(accel_rps2))

        # Jacobian condition number
        cond = compute_condition_number(pos, rot, geom, J=J)
        peak_cond = max(peak_cond, cond)

        # Full feedforward torques (gravity + inertia + reflected motor)
        # Skip when no torque limit — the result would never be checked.
        if torque_limit_Nm is not None:
            torque_Nm = compute_full_feedforward_torques(
                pos, rot, twist, accel_cart, geom, dynamics_params, J=J)
            peak_torque = np.maximum(peak_torque, np.abs(torque_Nm))

        # Cartesian jerk (3rd derivative)
        if check_jerk:
            jerk = evaluate_jerk(traj, t)
            jerk_t_mag = float(np.linalg.norm(jerk[:3]))
            jerk_r_mag = float(np.linalg.norm(jerk[3:]))
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
