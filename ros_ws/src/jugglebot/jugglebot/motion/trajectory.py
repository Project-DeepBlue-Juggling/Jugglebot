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
from dataclasses import dataclass
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

    Returns
    -------
    pos_rev : (6,) ndarray — motor positions in revolutions
    vel_ff_rps : (6,) ndarray — velocity feedforward in rev/s
    torque_ff_Nm : (6,) ndarray — torque feedforward in Nm
    """
    pos = pose[:3]
    rot = rotvec_to_rot_matrix(pose[3:6])

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
    n_samples: int = 200,
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
    n_samples : int
        Number of evenly-spaced points to evaluate.

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

    # Accumulators
    peak_vel = np.zeros(6)
    peak_accel = np.zeros(6)
    max_ext = np.full(6, -np.inf)
    min_ext = np.full(6, np.inf)
    peak_cond = 0.0
    peak_torque = np.zeros(6)

    for t in times:
        pose, twist, accel_cart = evaluate(traj, t)
        pos = pose[:3]
        rot = rotvec_to_rot_matrix(pose[3:6])

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
        torque_Nm = compute_full_feedforward_torques(
            pos, rot, twist, accel_cart, geom, dynamics_params)
        peak_torque = np.maximum(peak_torque, np.abs(torque_Nm))

    # Check violations
    stroke = geom.leg_stroke_mm
    if np.any(min_ext < 0.0):
        bad_legs = np.where(min_ext < 0.0)[0]
        violations.append(
            f"Underextended legs {bad_legs.tolist()}: "
            f"min extensions {min_ext[bad_legs].tolist()} mm")
    if np.any(max_ext > stroke):
        bad_legs = np.where(max_ext > stroke)[0]
        violations.append(
            f"Overextended legs {bad_legs.tolist()}: "
            f"max extensions {max_ext[bad_legs].tolist()} mm")

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

    return FeasibilityResult(
        feasible=len(violations) == 0,
        peak_leg_vel_rps=peak_vel,
        peak_leg_accel_rps2=peak_accel,
        max_extension_mm=max_ext,
        min_extension_mm=min_ext,
        peak_condition_number=peak_cond,
        peak_torque_ff_Nm=peak_torque,
        violations=violations,
        n_samples=n_samples,
    )


# ---------------------------------------------------------------------------
# Trajectory manager
# ---------------------------------------------------------------------------

class TrajectoryState(enum.Enum):
    IDLE = 'idle'
    EXECUTING = 'executing'
    COMPLETE = 'complete'


class TrajectoryManager:
    """Manages trajectory execution within the control loop.

    Lifecycle:
        1. Set a hold pose via ``set_hold_pose()`` (typically home).
        2. Submit a trajectory via ``submit()``.
        3. Each control cycle, call ``evaluate(t)`` to get motor commands.
        4. When the trajectory completes, state transitions to COMPLETE.
        5. Submit a new trajectory, or call ``cancel()`` to return to IDLE.

    Phase 6 additions:
        - ``progress`` and ``time_remaining`` are now live (updated on evaluate).
        - ``current_pose_6dof`` exposes the latest evaluated Cartesian pose.
    """

    def __init__(self, geom: StewartGeometry, dynamics_params: DynamicsParams):
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

        # Progress tracking (Phase 6)
        self._last_progress = 0.0
        self._last_time_remaining = 0.0
        self._current_pose = np.zeros(6)  # latest evaluated [x,y,z,rx,ry,rz]

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

    def set_feedforward_enabled(self, enabled: bool) -> None:
        """Toggle gravity feedforward for trajectory evaluation."""
        self._feedforward_enabled = enabled

    def set_hold_pose(self, pose_6dof: np.ndarray) -> None:
        """Set the hold pose for IDLE state.

        Parameters
        ----------
        pose_6dof : (6,) ndarray — [x, y, z, rx, ry, rz] in mm, rad
        """
        self._hold_pose = np.asarray(pose_6dof, dtype=np.float64)
        rot = rotvec_to_rot_matrix(self._hold_pose[3:6])
        self._hold_rot = rot
        pos = self._hold_pose[:3]
        extensions_mm = pose_to_leg_lengths(pos, rot, self.geom)
        self._hold_pos_rev = extensions_mm_to_revs(extensions_mm, self.geom)
        if self._feedforward_enabled:
            self._hold_torque_ff = gravity_to_motor_torques(
                pos, rot, self.geom, self.dynamics_params)
        else:
            self._hold_torque_ff = np.zeros(6)

    def submit(self, traj: QuinticTrajectory) -> None:
        """Submit a trajectory for execution.

        Parameters
        ----------
        traj : QuinticTrajectory
            Must have passed feasibility checking before submission.

        Raises
        ------
        RuntimeError
            If state is EXECUTING (Phase 4 restriction).
        """
        if self._state == TrajectoryState.EXECUTING:
            raise RuntimeError(
                "Cannot submit trajectory while executing "
                "(mid-motion re-planning deferred to Phase 7)")
        self._active_traj = traj
        self._state = TrajectoryState.EXECUTING
        logger.info(
            f"Trajectory submitted: duration={traj.duration:.3f}s, "
            f"speed_scale={traj.speed_scale:.2f}")

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
        if self._state == TrajectoryState.IDLE:
            self._current_pose = self._hold_pose.copy()
            return (self._hold_pos_rev.copy(),
                    np.zeros(6),
                    self._hold_torque_ff.copy())

        if self._state == TrajectoryState.COMPLETE:
            self._current_pose = self._hold_pose.copy()
            return (self._hold_pos_rev.copy(),
                    np.zeros(6),
                    self._hold_torque_ff.copy())

        # EXECUTING
        traj = self._active_traj
        t_end = traj.t_start + traj.duration

        if t >= t_end:
            # Trajectory complete — transition to COMPLETE
            self._state = TrajectoryState.COMPLETE
            self._last_progress = 1.0
            self._last_time_remaining = 0.0
            # Set hold pose to end pose
            end_pose = traj.end_state[:6]
            self.set_hold_pose(end_pose)
            self._current_pose = end_pose.copy()
            logger.info("Trajectory complete")
            return (self._hold_pos_rev.copy(),
                    np.zeros(6),
                    self._hold_torque_ff.copy())

        # Update progress tracking
        elapsed = t - traj.t_start
        self._last_progress = min(1.0, elapsed / traj.duration)
        self._last_time_remaining = max(0.0, t_end - t)

        # Evaluate trajectory at current time
        pose, twist, accel_cart = evaluate(traj, t)
        self._current_pose = pose.copy()
        return cartesian_to_motor_commands(
            pose, twist, accel_cart,
            self.geom, self.dynamics_params,
            self._feedforward_enabled)

    def cancel(self) -> None:
        """Cancel the active trajectory and transition to IDLE.

        The hold pose remains at whatever was last set (typically
        the end pose from a previous trajectory or home).
        """
        if self._state == TrajectoryState.EXECUTING:
            logger.info("Trajectory cancelled")
        self._active_traj = None
        self._state = TrajectoryState.IDLE
        self._last_progress = 0.0
        self._last_time_remaining = 0.0
