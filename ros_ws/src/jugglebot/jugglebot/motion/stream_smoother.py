"""Universal C2-continuous quintic smoother for all direct targets.

Every incoming target — regardless of source (spacemouse, GUI, shell) —
gets a quintic segment whose duration is computed from the motor-space
displacement and the system's velocity/acceleration limits.

C2 continuity is guaranteed at every splice: when a new target arrives,
the current segment is evaluated to get the exact (pose, twist, accel),
which becomes the start boundary of the new segment.  The end boundary
is always (target, twist=0, accel=0), so the platform smoothly
decelerates to rest if no further targets arrive.

Deferred splice: when a segment is already executing, the new segment
is scheduled to start DEFER_S seconds in the future.  The current
segment continues executing during the defer window, allowing velocity
to build.  At the splice point the new segment inherits the current
segment's mid-flight state, starting with meaningful velocity.  The
first target always splices immediately (no velocity to inherit).

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
from jugglebot.motion.quintic import solve_quintic_1d


class StreamSmoother:
    """Universal C2-continuous quintic smoother for all direct targets.

    Every incoming target — regardless of source (spacemouse, GUI, shell) —
    gets a quintic segment whose duration is computed from the motor-space
    displacement and the system's velocity/acceleration limits.

    C2 continuity is guaranteed at every splice.  Deferred splicing lets the
    current segment develop velocity before the new segment takes over.
    """

    T_MIN = 0.005   # 5 ms floor (prevents degenerate polynomials)
    T_MAX = 5.0     # 5 s ceiling
    HYSTERESIS_REV = 0.05  # motor-space threshold for target deduplication
    DEFER_S = 0.05  # deferred splice window (seconds)

    def __init__(self,
                 geom: StewartGeometry,
                 vel_limit_rps: float,
                 accel_limit_rps2: float):
        self._geom = geom
        self._vel_limit = vel_limit_rps
        self._accel_limit = accel_limit_rps2

        # Active segment state
        self._coeffs = np.zeros((6, 6))     # quintic coefficients per DoF
        self._t_start = 0.0
        self._duration = 0.0
        self._pose = np.zeros(6)             # [x, y, z, rx, ry, rz]
        self._twist = np.zeros(6)
        self._accel = np.zeros(6)
        self._target = np.zeros(6)
        self._has_segment = False

        # Pending segment (deferred splice)
        self._pending_coeffs = np.zeros((6, 6))
        self._pending_t_start = 0.0
        self._pending_duration = 0.0
        self._pending_target = np.zeros(6)
        self._pending_start_pose = np.zeros(6)
        self._pending_start_twist = np.zeros(6)
        self._pending_start_accel = np.zeros(6)
        self._has_pending = False

    def set_limits(self, vel_limit_rps: float, accel_limit_rps2: float) -> None:
        """Update velocity and acceleration limits (e.g. when switching control modes)."""
        self._vel_limit = vel_limit_rps
        self._accel_limit = accel_limit_rps2

    def reset(self, pose_6dof: np.ndarray) -> None:
        """Reset to a known pose with zero derivatives.

        Called on enable, disable, E-stop, and trajectory->direct transitions.
        """
        self._pose = pose_6dof.copy()
        self._twist = np.zeros(6)
        self._accel = np.zeros(6)
        self._target = pose_6dof.copy()
        self._has_segment = False
        self._has_pending = False

    def set_target(self, pose_6dof: np.ndarray, t_now: float) -> None:
        """Accept a new target from any source.

        Deferred splice: when a segment is already executing, the new
        quintic is scheduled to start DEFER_S seconds in the future.
        The current segment continues executing, allowing velocity to
        build.  The first target always splices immediately.

        If a pending segment already exists (target arrived during the
        current defer window), only the endpoint is updated — the start
        state and splice time are preserved.

        Target hysteresis: if the new target is within HYSTERESIS_REV of
        the effective target (pending or current) in motor space, the
        update is skipped.
        """
        # --- Hysteresis: skip if target barely changed ---
        effective_target = self._pending_target if self._has_pending else self._target
        if self._has_segment or self._has_pending:
            tgt_rot = rotvec_to_rot_matrix(pose_6dof[3:6])
            old_rot = rotvec_to_rot_matrix(effective_target[3:6])

            new_rev = extensions_mm_to_revs(
                pose_to_leg_lengths(pose_6dof[:3], tgt_rot, self._geom),
                self._geom)
            old_rev = extensions_mm_to_revs(
                pose_to_leg_lengths(effective_target[:3], old_rot, self._geom),
                self._geom)

            if np.max(np.abs(new_rev - old_rev)) < self.HYSTERESIS_REV:
                return  # target hasn't moved enough

        # --- Determine splice time ---
        if not self._has_segment and not self._has_pending:
            # First target: splice immediately (no velocity to inherit)
            splice_time = t_now
        elif not self._has_pending:
            # New defer window
            splice_time = t_now + self.DEFER_S
        else:
            # Already pending: keep existing splice time
            splice_time = self._pending_t_start

        # --- Compute start state (only once per defer window) ---
        if not self._has_pending:
            if self._has_segment:
                start_pose, start_twist, start_accel = self._peek_at(splice_time)
            else:
                start_pose = self._pose.copy()
                start_twist = self._twist.copy()
                start_accel = self._accel.copy()
            self._pending_start_pose = start_pose
            self._pending_start_twist = start_twist
            self._pending_start_accel = start_accel
            self._pending_t_start = splice_time

        # --- Compute duration and build pending quintic ---
        duration = self._compute_duration(
            self._pending_start_pose, pose_6dof,
            twist=self._pending_start_twist)

        self._pending_target = pose_6dof.copy()
        self._pending_duration = duration

        for i in range(6):
            self._pending_coeffs[i] = solve_quintic_1d(
                self._pending_start_pose[i],
                self._pending_start_twist[i],
                self._pending_start_accel[i],
                pose_6dof[i], 0.0, 0.0, duration)

        self._has_pending = True

        # --- First target: commit immediately ---
        if not self._has_segment:
            self._commit_pending()

    def evaluate(self, t_now: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Evaluate at the current time.

        Returns (pose, twist, accel) — all (6,) ndarrays.
        Called every control cycle (500 Hz).
        """
        # Promote pending segment if splice time has arrived
        if self._has_pending and t_now >= self._pending_t_start:
            self._commit_pending()

        if not self._has_segment:
            return self._pose.copy(), self._twist.copy(), self._accel.copy()

        self._evaluate_at(t_now)
        return self._pose.copy(), self._twist.copy(), self._accel.copy()

    # -- internals ----------------------------------------------------------

    def _commit_pending(self) -> None:
        """Promote the pending segment to active."""
        self._coeffs = self._pending_coeffs.copy()
        self._t_start = self._pending_t_start
        self._duration = self._pending_duration
        self._target = self._pending_target.copy()
        self._pose = self._pending_start_pose.copy()
        self._twist = self._pending_start_twist.copy()
        self._accel = self._pending_start_accel.copy()
        self._has_segment = True
        self._has_pending = False

    def _peek_at(self, t_now: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Evaluate the active quintic at *t_now* without modifying state.

        Used to compute the start state for a deferred splice while the
        active segment continues executing.
        """
        T = self._duration
        elapsed = t_now - self._t_start

        if elapsed >= T:
            return self._target.copy(), np.zeros(6), np.zeros(6)

        if elapsed <= 0:
            return self._pose.copy(), self._twist.copy(), self._accel.copy()

        tau = elapsed / T
        tau2 = tau * tau
        tau3 = tau2 * tau
        tau4 = tau3 * tau
        tau5 = tau4 * tau
        c = self._coeffs

        pose = (c[:, 0] + c[:, 1] * tau + c[:, 2] * tau2
                + c[:, 3] * tau3 + c[:, 4] * tau4 + c[:, 5] * tau5)
        twist = (c[:, 1] + 2 * c[:, 2] * tau + 3 * c[:, 3] * tau2
                 + 4 * c[:, 4] * tau3 + 5 * c[:, 5] * tau4) / T
        accel = (2 * c[:, 2] + 6 * c[:, 3] * tau
                 + 12 * c[:, 4] * tau2 + 20 * c[:, 5] * tau3) / (T * T)

        return pose, twist, accel

    def _compute_duration(self,
                          current_pose: np.ndarray,
                          target_pose: np.ndarray,
                          twist: np.ndarray | None = None) -> float:
        """Compute minimum feasible duration from motor-space displacement and velocity.

        Runs IK at both endpoints to get per-motor displacements in
        revolutions, then uses the quintic peak formulas:
          T_vel  = 15*d / (8*v_limit)       — displacement-limited
          T_acc  = sqrt(10*d / (3*a_limit))  — displacement-limited
          T_decel = 15*v / (8*a_limit)       — velocity-limited

        The velocity term ensures the quintic has enough time to arrest the
        current motor-space velocity without overshooting.  Without it,
        small-displacement splices at high velocity produce durations near
        T_MIN, causing the quintic coefficients to overshoot and oscillate.

        Takes the max across all 6 motors and clamps to [T_MIN, T_MAX].
        """
        if twist is None:
            twist = self._twist

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
        # J maps Cartesian twist (mm/s, rad/s) -> leg extension rates (mm/s);
        # multiply by mm_to_rev to get rev/s.
        J = compute_jacobian(current_pose[:3], cur_rot, self._geom)
        motor_vel_rps = (J @ twist) * self._geom.mm_to_rev
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
