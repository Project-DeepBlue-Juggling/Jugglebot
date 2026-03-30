"""Shared test helpers for the motion subpackage.

Provides ``submit_dynamic_target_sync``, a synchronous wrapper around
TrajectoryManager's internal planning logic.  Production code uses the
async pipeline (request_dynamic_target / poll_pending_result /
commit_async_trajectory); this helper exists solely for offline tests
and tool scripts that run with frozen clocks and need immediate state
transitions.
"""

from __future__ import annotations

import logging
from dataclasses import replace as _dc_replace

import numpy as np

from jugglebot.motion.feasibility import check_feasibility
from jugglebot.motion.ik_solver import quat_to_rot_matrix, rot_matrix_to_rotvec
from jugglebot.motion.quintic import create_trajectory
from jugglebot.motion.trajectory_manager import MIN_LEAD_TIME_S, TrajectoryManager

logger = logging.getLogger(__name__)


def submit_dynamic_target_sync(
    mgr: TrajectoryManager,
    target_pos: np.ndarray,
    target_quat: np.ndarray,
    target_vel: np.ndarray,
    arrival_time: float,
    t_now: float,
) -> bool:
    """Synchronously plan and submit a dynamic target.

    Equivalent to the former ``TrajectoryManager.submit_dynamic_target()``
    method.  Blocks while running the feasibility check, then commits the
    trajectory immediately via ``mgr.submit()``.

    Parameters
    ----------
    mgr : TrajectoryManager
    target_pos : (3,) ndarray — [x, y, z] in mm
    target_quat : (4,) ndarray — [w, x, y, z] quaternion
    target_vel : (3,) ndarray — [vx, vy, vz] in mm/s
    arrival_time : float — absolute arrival time
    t_now : float — current time (from the injected clock)

    Returns
    -------
    accepted : bool
        True if the target was accepted and a trajectory planned.
    """
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
    # feasibility checking so we can restamp t_start forward.
    t_clock_start = mgr._clock()

    cur_pose, cur_twist, cur_accel = mgr._get_current_state(t_now)

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
        traj, mgr.geom, mgr.dynamics_params, n_samples=50)
    if not result.feasible:
        logger.debug(
            f"Dynamic target rejected (infeasible): "
            f"{'; '.join(result.violations)}")
        return False

    # Shift t_start forward by wall-clock time consumed
    total_elapsed = mgr._clock() - t_clock_start
    if total_elapsed > 0:
        traj = _dc_replace(traj, t_start=traj.t_start + total_elapsed)

    # Accept: submit trajectory and flag deceleration if needed
    mgr.submit(traj)
    mgr._pending_decel = bool(np.linalg.norm(target_vel) > 1e-6)

    # Pre-compute deceleration in background if needed
    if mgr._pending_decel:
        mgr._start_decel_precompute(traj)

    return True
