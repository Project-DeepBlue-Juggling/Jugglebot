"""Universal C2-continuous quintic smoother for all direct targets.

Every incoming target — regardless of source (spacemouse, GUI, shell) —
gets a quintic segment whose duration is computed from the motor-space
displacement and the system's velocity/acceleration limits.

C2 continuity is guaranteed at every splice: when a new target arrives,
the current segment is evaluated to get the exact (pose, twist, accel),
which becomes the start boundary of the new segment.  The end boundary
is always (target, twist=0, accel=0), so the platform smoothly
decelerates to rest if no further targets arrive.

No ROS2 or IPC dependency — pure Python + numpy.
"""

from __future__ import annotations

import numpy as np

from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.ik_solver import (
    compute_jacobian,
    pose_to_leg_lengths,
    rotvec_to_rot_matrix,
)
from jugglebot.motion.conversions import extensions_mm_to_revs
from jugglebot.motion.trajectory import solve_quintic_1d


class StreamSmoother:
    """Universal C2-continuous quintic smoother for all direct targets.

    Every incoming target — regardless of source (spacemouse, GUI, shell) —
    gets a quintic segment whose duration is computed from the motor-space
    displacement and the system's velocity/acceleration limits.

    C2 continuity is guaranteed at every splice: when a new target arrives,
    the current segment is evaluated to get the exact (pose, twist, accel),
    which becomes the start boundary of the new segment.  The end boundary
    is always (target, twist=0, accel=0), so the platform smoothly
    decelerates to rest if no further targets arrive.
    """

    T_MIN = 0.005   # 5 ms floor (prevents degenerate polynomials)
    T_MAX = 5.0     # 5 s ceiling

    def __init__(self,
                 geom: StewartGeometry,
                 vel_limit_rps: float,
                 accel_limit_rps2: float):
        self._geom = geom
        self._vel_limit = vel_limit_rps
        self._accel_limit = accel_limit_rps2

        self._coeffs = np.zeros((6, 6))     # quintic coefficients per DoF
        self._t_start = 0.0
        self._duration = 0.0
        self._pose = np.zeros(6)             # [x, y, z, rx, ry, rz]
        self._twist = np.zeros(6)
        self._accel = np.zeros(6)
        self._target = np.zeros(6)
        self._has_segment = False

    def set_limits(self, vel_limit_rps: float, accel_limit_rps2: float) -> None:
        """Update velocity and acceleration limits (e.g. when switching control modes)."""
        self._vel_limit = vel_limit_rps
        self._accel_limit = accel_limit_rps2

    def reset(self, pose_6dof: np.ndarray) -> None:
        """Reset to a known pose with zero derivatives.

        Called on enable, disable, E-stop, and trajectory→direct transitions.
        """
        self._pose = pose_6dof.copy()
        self._twist = np.zeros(6)
        self._accel = np.zeros(6)
        self._target = pose_6dof.copy()
        self._has_segment = False

    def set_target(self, pose_6dof: np.ndarray, t_now: float) -> None:
        """Accept a new target from any source.

        Evaluates the current segment at the splice point to preserve C2
        continuity, computes the minimum feasible duration from motor-space
        displacement and system limits, and builds a new quintic segment.
        """
        if self._has_segment:
            self._evaluate_at(t_now)  # update state to splice point

        duration = self._compute_duration(self._pose, pose_6dof)

        self._target = pose_6dof.copy()
        self._t_start = t_now
        self._duration = duration

        for i in range(6):
            self._coeffs[i] = solve_quintic_1d(
                self._pose[i], self._twist[i], self._accel[i],
                self._target[i], 0.0, 0.0, duration)

        self._has_segment = True

    def evaluate(self, t_now: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Evaluate at the current time.

        Returns (pose, twist, accel) — all (6,) ndarrays.
        Called every control cycle (500 Hz).
        """
        if not self._has_segment:
            return self._pose.copy(), self._twist.copy(), self._accel.copy()

        self._evaluate_at(t_now)
        return self._pose.copy(), self._twist.copy(), self._accel.copy()

    # -- internals ----------------------------------------------------------

    def _compute_duration(self,
                          current_pose: np.ndarray,
                          target_pose: np.ndarray) -> float:
        """Compute minimum feasible duration from motor-space displacement and velocity.

        Runs IK at both endpoints to get per-motor displacements in
        revolutions, then uses the quintic peak formulas:
          T_vel  = 15·d / (8·v_limit)       — displacement-limited
          T_acc  = sqrt(10·d / (3·a_limit))  — displacement-limited
          T_decel = 15·v / (8·a_limit)       — velocity-limited

        The velocity term ensures the quintic has enough time to arrest the
        current motor-space velocity without overshooting.  Without it,
        small-displacement splices at high velocity produce durations near
        T_MIN, causing the quintic coefficients to overshoot and oscillate.

        Takes the max across all 6 motors and clamps to [T_MIN, T_MAX].
        """
        cur_rot = rotvec_to_rot_matrix(current_pose[3:6])
        tgt_rot = rotvec_to_rot_matrix(target_pose[3:6])

        cur_ext = pose_to_leg_lengths(current_pose[:3], cur_rot, self._geom)
        tgt_ext = pose_to_leg_lengths(target_pose[:3], tgt_rot, self._geom)

        cur_rev = extensions_mm_to_revs(cur_ext, self._geom)
        tgt_rev = extensions_mm_to_revs(tgt_ext, self._geom)

        d_rev = np.abs(tgt_rev - cur_rev)
        d_max = np.max(d_rev)

        # Displacement-based terms
        if d_max < 1e-6:
            t_vel = 0.0
            t_acc = 0.0
        else:
            t_vel = 15.0 * d_max / (8.0 * self._vel_limit)
            t_acc = np.sqrt(10.0 * d_max / (3.0 * self._accel_limit))

        # Velocity-based term: ensure enough time to decelerate from the
        # current motor-space velocity to rest without exceeding accel limits.
        # J maps Cartesian twist (mm/s, rad/s) → leg extension rates (mm/s);
        # multiply by mm_to_rev to get rev/s.
        J = compute_jacobian(current_pose[:3], cur_rot, self._geom)
        motor_vel_rps = (J @ self._twist) * self._geom.mm_to_rev
        v_max = np.max(np.abs(motor_vel_rps))
        t_decel = 15.0 * v_max / (8.0 * self._accel_limit)

        return float(np.clip(max(t_vel, t_acc, t_decel), self.T_MIN, self.T_MAX))

    def _evaluate_at(self, t_now: float) -> None:
        """Evaluate quintic polynomials, updating internal state."""
        T = self._duration
        elapsed = t_now - self._t_start

        if elapsed >= T:
            # Segment complete: hold at target with zero derivatives
            self._pose = self._target.copy()
            self._twist = np.zeros(6)
            self._accel = np.zeros(6)
            return

        if elapsed <= 0:
            return  # before segment start, keep current state

        tau = elapsed / T
        tau2 = tau * tau
        tau3 = tau2 * tau
        tau4 = tau3 * tau
        tau5 = tau4 * tau
        c = self._coeffs

        self._pose = (c[:, 0] + c[:, 1] * tau + c[:, 2] * tau2
                       + c[:, 3] * tau3 + c[:, 4] * tau4 + c[:, 5] * tau5)
        self._twist = (c[:, 1] + 2 * c[:, 2] * tau + 3 * c[:, 3] * tau2
                        + 4 * c[:, 4] * tau3 + 5 * c[:, 5] * tau4) / T
        self._accel = (2 * c[:, 2] + 6 * c[:, 3] * tau
                        + 12 * c[:, 4] * tau2 + 20 * c[:, 5] * tau3) / (T * T)
