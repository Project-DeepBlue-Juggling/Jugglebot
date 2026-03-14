"""Trajectory execution state machine and async feasibility pipeline.

Manages the lifecycle of trajectory execution within the control loop:
submit, evaluate each cycle, handle completion/deceleration, and
coordinate background feasibility checking via the worker process.

Pure Python + numpy.  No ROS2, no ZeroMQ dependency.
"""

from __future__ import annotations

import enum
import logging
import time as _time
from dataclasses import replace as _dc_replace
from typing import TYPE_CHECKING

import numpy as np

from jugglebot.motion.conversions import extensions_mm_to_revs
from jugglebot.motion.dynamics import (
    DynamicsParams,
    gravity_to_motor_torques,
)
from jugglebot.motion.feasibility import check_feasibility
from jugglebot.motion.ik_solver import (
    pose_to_leg_lengths,
    rotvec_to_rot_matrix,
)
from jugglebot.motion.motor_commands import cartesian_to_motor_commands
from jugglebot.motion.quintic import (
    QuinticTrajectory,
    create_trajectory,
    evaluate,
)

if TYPE_CHECKING:
    from jugglebot.motion.geometry import StewartGeometry

logger = logging.getLogger(__name__)

# Minimum time (seconds) between now and a dynamic target's arrival_time.
# Targets closer than this are rejected — they leave insufficient margin
# for feasibility checking and safe trajectory execution.
MIN_LEAD_TIME_S = 0.3

# Time (seconds) between "now" and the splice point when committing an
# async trajectory.  The old trajectory continues executing during this
# window while the re-created trajectory undergoes a feasibility re-check
# in the worker process.  200 ms gives ~4× headroom over typical 50-sample
# feasibility check time (~50 ms).
SPLICE_LEAD_S = 0.200


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
        2. Submit a trajectory via ``submit()`` or the async pipeline.
        3. Each control cycle, call ``evaluate(t)`` to get motor commands.
        4. When the trajectory completes:
           - Zero-velocity targets → COMPLETE (hold at target).
           - Non-zero-velocity targets → RETURNING (decelerate to stop).
        5. Submit a new trajectory at any time (mid-motion replanning).

    Phase 6 additions:
        - ``progress`` and ``time_remaining`` are now live (updated on evaluate).
        - ``current_pose_6dof`` exposes the latest evaluated Cartesian pose.

    Phase 7 additions:
        - ``request_dynamic_target()`` / ``poll_pending_result()`` /
          ``commit_async_trajectory()`` for non-blocking target commanding.
        - ``RETURNING`` state for deceleration-to-stop after non-zero-velocity
          targets.
        - Mid-motion replanning: ``submit()`` and the async pipeline can
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
        self._last_jacobian: np.ndarray | None = None  # cached from last evaluate()

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
        self._precomputed_decel: QuinticTrajectory | None = None
        self._decel_generation = 0
        self._pending_splice: dict | None = None      # splice awaiting re-check

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
    def last_jacobian(self) -> np.ndarray | None:
        """Jacobian from the most recent evaluate() call, or None if holding."""
        return self._last_jacobian

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

        Call this after any operation that blocks the control loop to
        compensate for wall-clock time consumed.  The polynomial uses
        normalised time, so only the absolute time anchor changes — the
        trajectory shape is unaffected.
        """
        if self._active_traj is not None and elapsed > 0:
            self._active_traj = _dc_replace(
                self._active_traj,
                t_start=self._active_traj.t_start + elapsed,
            )

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

        Side effects
        ------------
        Updates ``_last_jacobian`` with the Jacobian computed during
        ``cartesian_to_motor_commands`` (or None for hold paths).
        The control loop can read this to avoid recomputing J for
        the condition number check.
        """
        if self._state in (TrajectoryState.IDLE, TrajectoryState.COMPLETE):
            self._current_pose = self._hold_pose.copy()
            self._last_jacobian = None
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
                self._last_jacobian = None
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
                    pos_rev, vel_ff, torque_ff, J = cartesian_to_motor_commands(
                        pose, twist, accel_cart,
                        self.geom, self.dynamics_params,
                        self._feedforward_enabled,
                        self._gravity_correction)
                    self._last_jacobian = J
                    return pos_rev, vel_ff, torque_ff
                else:
                    # Pre-computed deceleration not ready yet — hold at end
                    # pose and wait for next cycle.  Keep _pending_decel True
                    # and stay in EXECUTING so we re-enter this branch.
                    self._held_at_end_cycles += 1
                    self.set_hold_pose(end_pose)
                    self._last_jacobian = None
                    return (self._hold_pos_rev.copy(),
                            np.zeros(6),
                            self._hold_torque_ff.copy())

            # Zero-velocity target or failed deceleration → COMPLETE
            self._state = TrajectoryState.COMPLETE
            self._active_traj = None
            self.set_hold_pose(end_pose)
            logger.info("Trajectory complete")
            self._last_jacobian = None
            return (self._hold_pos_rev.copy(),
                    np.zeros(6),
                    self._hold_torque_ff.copy())

        # Before trajectory starts: hold at start pose
        if t < traj.t_start:
            self._last_progress = 0.0
            self._last_time_remaining = t_end - t
            pose = traj.start_state[:6]
            self._current_pose = pose.copy()
            pos_rev, vel_ff, torque_ff, J = cartesian_to_motor_commands(
                pose, np.zeros(6), np.zeros(6),
                self.geom, self.dynamics_params,
                self._feedforward_enabled,
                self._gravity_correction)
            self._last_jacobian = J
            return pos_rev, vel_ff, torque_ff

        # Mid-trajectory: evaluate at current time
        elapsed = t - traj.t_start
        self._last_progress = min(1.0, elapsed / traj.duration)
        self._last_time_remaining = max(0.0, t_end - t)

        pose, twist, accel_cart = evaluate(traj, t)
        self._current_pose = pose.copy()
        pos_rev, vel_ff, torque_ff, J = cartesian_to_motor_commands(
            pose, twist, accel_cart,
            self.geom, self.dynamics_params,
            self._feedforward_enabled,
            self._gravity_correction)
        self._last_jacobian = J
        return pos_rev, vel_ff, torque_ff

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
        self._pending_splice = None  # supersede any in-flight splice re-check
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
        Only reads from the feasibility/splice pipe — deceleration
        results are on a separate pipe read by ``poll_precomputed_decel()``.

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
        from jugglebot.motion.feasibility_worker import WorkerCrashed
        # May raise WorkerCrashed — caller handles
        result = self._worker.poll()
        if result is None:
            return None

        if result['type'] == 'splice_recheck':
            self._handle_splice_result(result)
            return None  # handled internally

        # Feasibility result — check generation
        if result['generation'] != self._async_generation:
            return None  # stale
        return result

    def commit_async_trajectory(self, result: dict) -> bool:
        """Queue a splice re-check for a trajectory from an async result.

        Instead of committing immediately, plans the new trajectory from
        a future splice point (``SPLICE_LEAD_S`` ahead) and submits it
        to the feasibility worker for re-checking.  The currently-executing
        trajectory continues uninterrupted until the re-check passes and
        the splice point arrives.

        Parameters
        ----------
        result : dict
            From ``poll_pending_result()``.

        Returns
        -------
        queued : bool
            True if the splice re-check was queued successfully.
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

        # Plan the splice point ahead of now
        splice_time = t_now + SPLICE_LEAD_S

        # Need enough time after the splice for a meaningful trajectory
        if splice_time >= arrival_time - 0.01:
            logger.debug(
                "Async commit: insufficient time for splice "
                "(splice=%.3f, arrival=%.3f)", splice_time, arrival_time)
            return False

        # Sample the current trajectory at the splice point — this is
        # deterministic (polynomial evaluation), guaranteeing C2 continuity
        splice_pose, splice_twist, splice_accel = \
            self._get_current_state(splice_time)

        splice_duration = arrival_time - splice_time
        try:
            traj = create_trajectory(
                start_pose=splice_pose,
                start_twist=splice_twist,
                start_accel=splice_accel,
                end_pose=original_traj.end_state[:6],
                end_twist=original_traj.end_state[6:12],
                end_accel=original_traj.end_state[12:18],
                duration=splice_duration,
                t_start=splice_time,
            )
        except ValueError as e:
            logger.debug(f"Async commit trajectory creation failed: {e}")
            return False

        # Submit for feasibility re-check (non-blocking)
        request = {
            'type': 'splice_recheck',
            'traj': traj,
            'needs_decel': result['needs_decel'],
            'splice_time': splice_time,
            'generation': self._async_generation,
        }
        if not self._worker.submit(request):
            logger.warning("Feasibility worker dead, splice re-check failed")
            return False

        self._pending_splice = {
            'traj': traj,
            'needs_decel': result['needs_decel'],
            'splice_time': splice_time,
            'generation': self._async_generation,
        }

        logger.debug(
            "Splice re-check queued: splice_in=%.0fms, "
            "duration=%.3fs, latency=%.3fs",
            SPLICE_LEAD_S * 1000, splice_duration,
            t_now - result['t_request'])
        return True

    def _handle_splice_result(self, result: dict) -> None:
        """Handle a completed splice re-check from the feasibility worker.

        Called internally by ``poll_pending_result()``.  If the re-check passed and the
        splice point hasn't elapsed, commits the trajectory.  Otherwise
        discards it and the current trajectory continues uninterrupted.
        """
        splice = self._pending_splice
        if splice is None:
            return

        # Stale: a newer request has superseded this splice
        if result['generation'] != self._async_generation:
            logger.debug("Splice re-check discarded (stale generation)")
            self._pending_splice = None
            return

        if splice['generation'] != result['generation']:
            logger.debug("Splice re-check generation mismatch")
            self._pending_splice = None
            return

        if not result['accepted']:
            logger.warning(
                "Splice re-check FAILED feasibility — continuing current "
                "trajectory. Violations: %s",
                '; '.join(result.get('violations', [])))
            self._pending_splice = None
            return

        splice_time = splice['splice_time']
        t_now = self._clock()

        if t_now >= splice_time:
            logger.warning(
                "Splice re-check passed but splice time has elapsed "
                "(splice=%.3f, now=%.3f, overrun=%.0fms) — discarding",
                splice_time, t_now, (t_now - splice_time) * 1000)
            self._pending_splice = None
            return

        # Feasible and on time — commit the splice trajectory
        traj = splice['traj']
        self.submit(traj)
        self._pending_decel = splice['needs_decel']

        if splice['needs_decel']:
            self._start_decel_precompute(traj)

        logger.info(
            "Splice trajectory committed: splice_in=%.0fms, "
            "recheck=%.1fms",
            (splice_time - t_now) * 1000,
            result.get('check_elapsed', 0) * 1000)

        self._pending_splice = None

    def poll_precomputed_decel(self) -> QuinticTrajectory | None:
        """Check if a pre-computed deceleration trajectory is available.

        Non-blocking.  Returns None if not ready or not applicable.
        Reads from the dedicated decel pipe — independent of the
        feasibility/splice pipe used by ``poll_pending_result()``.
        """
        try:
            result = self._worker.poll_decel()
            if result is not None:
                if (result['generation'] == self._decel_generation
                        and result.get('traj') is not None):
                    self._precomputed_decel = result['traj']
                    logger.info(
                        "Deceleration pre-computed: duration=%.3fs",
                        result['traj'].duration)
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
