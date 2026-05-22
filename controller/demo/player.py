"""Runtime open-loop trajectory player for the juggling demo.

``TrajectoryPlayer`` is a pure function of time: given a trajectory-relative
time it evaluates the pre-computed ``JuggleTrajectory``, runs inverse
kinematics, and produces the leg-extension command (with leg-velocity
feedforward and one-/two-step lookahead) that a plant's ``command()``
expects. No solver, no feedback, no state beyond the trajectory.

It is a drop-in replacement for the MPC as the 40 Hz command source — see
``plans/active/bb-led-two-ball-juggle-demo.md`` §2.

Pure NumPy — no ROS2, no CasADi.
"""
from __future__ import annotations

from typing import NamedTuple

import numpy as np

from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.ik_solver import (
    compute_jacobian,
    pose_to_leg_lengths,
    rotvec_to_rot_matrix,
    twist_to_leg_velocities,
)

from controller.demo.trajectory import JuggleTrajectory


class PlayerCommand(NamedTuple):
    """One tick of player output.

    ``ext_mm``/``ext_next_mm``/``ext_next2_mm`` are STOW-relative leg
    extensions (mm) at t, t+dt, t+2dt — the exact lookahead lets the
    downstream 500 Hz interpolator do cubic Hermite rather than extrapolate.
    """
    pose: np.ndarray          # (6,) commanded platform pose [x,y,z,rx,ry,rz]
    twist: np.ndarray         # (6,) commanded platform twist
    ext_mm: np.ndarray        # (6,) leg extensions, mm
    vel_mm_s: np.ndarray      # (6,) leg extension velocities, mm/s
    ext_next_mm: np.ndarray   # (6,) leg extensions one control step ahead
    ext_next2_mm: np.ndarray  # (6,) leg extensions two control steps ahead


class TrajectoryPlayer:
    """Evaluate a ``JuggleTrajectory`` and emit leg-extension commands.

    The player is plant-agnostic: it produces a :class:`PlayerCommand`; the
    caller forwards it to whichever plant via ``plant.command(...)``.
    """

    def __init__(self, trajectory: JuggleTrajectory,
                 geom: StewartGeometry = None,
                 control_dt: float = 0.025):
        self._traj = trajectory
        self._geom = geom if geom is not None else StewartGeometry()
        self._dt = float(control_dt)

    def _ik(self, pose):
        """Pose 6-vector -> (extensions_mm, pos, rot_matrix)."""
        pos = np.asarray(pose[:3], dtype=float)
        rot = rotvec_to_rot_matrix(np.asarray(pose[3:], dtype=float))
        return pose_to_leg_lengths(pos, rot, self._geom), pos, rot

    def command_at(self, t_rel: float) -> PlayerCommand:
        """Player output for trajectory-relative time ``t_rel`` (seconds)."""
        pose, twist, _accel = self._traj.eval(t_rel)
        ext, pos, rot = self._ik(pose)

        # Leg-velocity feedforward from the platform twist. NOTE: the
        # trajectory's twist is the pose-vector time-derivative; for a
        # non-level platform the rotation-vector rate is not the angular
        # velocity. The demo's level-platform trajectories carry zero
        # orientation, so [vx,vy,vz,wx,wy,wz] == pose-rate exactly here;
        # the optimiser/Phase 3 must revisit this when banking is added.
        J = compute_jacobian(pos, rot, self._geom)
        vel = twist_to_leg_velocities(twist, pos, rot, self._geom, J=J)

        pose_n, _, _ = self._traj.eval(t_rel + self._dt)
        pose_n2, _, _ = self._traj.eval(t_rel + 2.0 * self._dt)
        ext_next = self._ik(pose_n)[0]
        ext_next2 = self._ik(pose_n2)[0]

        return PlayerCommand(pose=pose, twist=twist, ext_mm=ext, vel_mm_s=vel,
                             ext_next_mm=ext_next, ext_next2_mm=ext_next2)
