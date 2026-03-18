"""Feasibility checker for dynamic targets.

Uses a coarse-horizon MPC solve to predict whether the controller can
reach a target pose within the available time.  The coarse MPC has the
same N=10 steps but a larger dt (e.g. 0.1s → 1s lookahead), so it sees
the full approach while keeping the same NLP size and solve cost (~5ms).

A fast IK pre-filter rejects geometrically impossible targets (extensions
outside leg stroke) before the coarse solve.
"""

from __future__ import annotations

import logging

import numpy as np

import os, sys
_sim_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if _sim_dir not in sys.path:
    sys.path.insert(0, _sim_dir)

from controller.mpc import MPCController
from controller.params import MPCParams
from plant.interface import PlantState

logger = logging.getLogger(__name__)


class FeasibilityChecker:
    """Predicts whether the MPC can reach a target pose in time.

    Two-stage check:
      1. **IK pre-filter** — rejects targets whose leg extensions fall outside
         the stroke range [0, stroke_mm].  Costs ~0.1ms (two IK calls).
      2. **Coarse MPC solve** — builds a static reference at the target pose
         and solves with a wide-horizon dt.  If the predicted terminal pose
         is close to the target, the target is feasible.  Costs ~5ms.

    Parameters
    ----------
    plant : MuJoCoPlant
        Used for IK (pose → extensions) and geometry.
    coarse_dt : float
        Timestep for the coarse MPC horizon.  With N=10, this gives an
        N*coarse_dt lookahead (e.g. 0.1 → 1.0s).
    terminal_pos_tol_mm : float
        Maximum acceptable position error at the end of the coarse horizon.
    terminal_ori_tol_rad : float
        Maximum acceptable orientation error at the end of the coarse horizon.
    """

    def __init__(
        self,
        plant,
        coarse_dt: float = 0.1,
        terminal_pos_tol_mm: float = 5.0,
        terminal_ori_tol_rad: float = 0.10,  # ~5.7°
    ):
        self._plant = plant
        self._stroke_mm = plant.geom.leg_stroke_mm
        self._terminal_pos_tol = terminal_pos_tol_mm
        self._terminal_ori_tol = terminal_ori_tol_rad

        # Build coarse-horizon MPC — same formulation, wider timestep
        coarse_params = MPCParams(
            dt=coarse_dt,
            max_leg_vel_mmps=1000.0,
            max_cpu_time=0.5,   # generous budget for one-shot solve
            max_iter=300,
            warm_start=False,   # no warm-start for one-shot feasibility
            print_level=0,
        )
        self._coarse_mpc = MPCController.from_plant(coarse_params, plant)
        self._coarse_dt = coarse_dt
        self._horizon_s = coarse_params.N * coarse_dt

    def check(
        self,
        current_state: PlantState,
        target_pose: np.ndarray,
        time_available: float,
    ) -> tuple[bool, str]:
        """Check whether the target is feasible.

        Parameters
        ----------
        current_state : PlantState
            Current platform state.
        target_pose : (6,) array
            Target [x, y, z, rx, ry, rz] in mm/rad.
        time_available : float
            Seconds until the target must be reached.

        Returns
        -------
        feasible : bool
        reason : str
            '' if feasible, or a short explanation of why not.
        """
        target_pose = np.asarray(target_pose, dtype=float)

        # ---- Stage 1: IK pre-filter ----
        try:
            target_ext = self._plant.pose_to_extensions(target_pose)
        except Exception as e:
            return False, f'IK failed: {e}'

        margin = 2.0  # mm tolerance
        if np.any(target_ext < -margin):
            worst = float(target_ext.min())
            return False, f'extension below stroke: {worst:.1f} mm'
        if np.any(target_ext > self._stroke_mm + margin):
            worst = float(target_ext.max())
            return False, f'extension above stroke: {worst:.1f} mm'

        # ---- Stage 2: coarse MPC solve ----
        # The coarse MPC has a fixed horizon of N*coarse_dt seconds.
        # We check the predicted pose at the step closest to time_available.
        N = self._coarse_mpc.params.N
        ref_poses = np.tile(target_pose, (N + 1, 1))
        ref_twists = np.zeros_like(ref_poses)

        # Reset coarse MPC state (no warm-start carryover)
        self._coarse_mpc.reset()

        _, diag = self._coarse_mpc.solve(current_state, ref_poses, ref_twist=ref_twists)

        predicted = self._coarse_mpc.predicted_poses
        if predicted is None:
            return False, 'coarse MPC produced no prediction'

        # Find the horizon step closest to the deadline
        # step_k corresponds to time k * coarse_dt
        deadline_step = min(N, max(1, round(time_available / self._coarse_dt)))
        check_pose = predicted[deadline_step]

        pos_err = np.linalg.norm(check_pose[:3] - target_pose[:3])
        ori_err = np.linalg.norm(check_pose[3:] - target_pose[3:])

        if pos_err > self._terminal_pos_tol:
            return False, (
                f'coarse MPC pos error {pos_err:.1f} mm at step {deadline_step} '
                f'({deadline_step * self._coarse_dt:.1f}s) > {self._terminal_pos_tol:.1f} mm')
        if ori_err > self._terminal_ori_tol:
            return False, (
                f'coarse MPC ori error {np.degrees(ori_err):.1f} deg at step {deadline_step} '
                f'> {np.degrees(self._terminal_ori_tol):.1f} deg')

        return True, ''
