"""Quintic polynomial solver and trajectory evaluation.

Pure math layer — no IK, no dynamics, no hardware config dependencies.

Provides:
  - ``solve_quintic_1d``: 6 boundary conditions → 6 polynomial coefficients
  - ``QuinticTrajectory``: frozen dataclass for 6-DoF quintic trajectories
  - ``create_trajectory`` / ``rescale_trajectory``: construction + speed scaling
  - ``evaluate`` / ``end_boundary_state`` / ``evaluate_jerk``: time → state

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

from dataclasses import dataclass

import numpy as np


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
