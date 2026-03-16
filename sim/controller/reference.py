"""Reference trajectory generation for the MPC controller.

Provides ``ReferenceGenerator`` — a uniform interface that the MPC loop calls
to obtain N+1 future reference states (pose + twist) spanning the prediction
horizon.

Factory constructors cover common use cases:
  - ``from_static_pose``  — constant pose, zero twist
  - ``from_waypoints``    — rest-to-rest quintic segments between waypoints
  - ``from_function``     — arbitrary callable ``fn(t) -> (pose, twist)``
"""

from __future__ import annotations

from typing import Callable

import numpy as np

# Import production quintic math (pure Python, no ROS2 dependency)
import os, sys
_repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(_repo_root, 'ros_ws', 'src', 'jugglebot'))

from jugglebot.motion.quintic import (
    QuinticTrajectory,
    create_trajectory,
    evaluate as quintic_evaluate,
)


class ReferenceGenerator:
    """Generates time-varying reference trajectories for the MPC horizon.

    Call :meth:`evaluate` each MPC step to get (N+1) reference poses and twists
    spanning the prediction horizon from ``t_start`` to ``t_start + N*dt``.
    """

    def __init__(
        self,
        fn: Callable[[float], tuple[np.ndarray, np.ndarray]],
        *,
        duration: float | None = None,
    ):
        """
        Parameters
        ----------
        fn : callable
            ``fn(t) -> (pose_6dof, twist_6dof)`` evaluated at any time *t*.
        duration : float or None
            Total trajectory duration (for informational / scheduling use).
            None means the reference is valid forever (e.g. static pose).
        """
        self._fn = fn
        self._duration = duration

    # ------------------------------------------------------------------
    # Factory constructors
    # ------------------------------------------------------------------

    @staticmethod
    def from_static_pose(pose: np.ndarray) -> ReferenceGenerator:
        """Constant pose reference with zero twist (static target)."""
        pose = np.asarray(pose, dtype=float).copy()
        twist = np.zeros(6)

        def _eval(t):
            return pose.copy(), twist.copy()

        return ReferenceGenerator(_eval, duration=None)

    @staticmethod
    def from_waypoints(
        waypoints: list[np.ndarray],
        durations: list[float],
        *,
        start_time: float = 0.0,
    ) -> ReferenceGenerator:
        """Rest-to-rest quintic segments between waypoints.

        Parameters
        ----------
        waypoints : list of (6,) arrays
            Sequence of poses ``[x, y, z, rx, ry, rz]`` (mm, rad).
            Must have at least 2 entries (start + end).
        durations : list of float
            Duration (seconds) for each segment.  ``len(durations) == len(waypoints) - 1``.
        start_time : float
            Absolute time of the first waypoint.
        """
        if len(waypoints) < 2:
            raise ValueError("Need at least 2 waypoints")
        if len(durations) != len(waypoints) - 1:
            raise ValueError(
                f"Need {len(waypoints)-1} durations for {len(waypoints)} waypoints, "
                f"got {len(durations)}"
            )

        # Build chained quintic segments (rest-to-rest: zero twist/accel at each waypoint)
        segments: list[QuinticTrajectory] = []
        t = start_time
        zeros = np.zeros(6)
        for i in range(len(durations)):
            seg = create_trajectory(
                start_pose=np.asarray(waypoints[i], dtype=float),
                start_twist=zeros,
                start_accel=zeros,
                end_pose=np.asarray(waypoints[i + 1], dtype=float),
                end_twist=zeros,
                end_accel=zeros,
                duration=durations[i],
                t_start=t,
            )
            segments.append(seg)
            t += durations[i]

        total_duration = sum(durations)

        def _eval(t):
            # Find active segment
            for seg in segments:
                seg_end = seg.t_start + seg.duration
                if t < seg_end:
                    pose, twist, _ = quintic_evaluate(seg, t)
                    return pose, twist
            # After all segments: hold final pose
            last = segments[-1]
            pose, twist, _ = quintic_evaluate(last, last.t_start + last.duration)
            return pose, twist

        return ReferenceGenerator(_eval, duration=total_duration)

    @staticmethod
    def from_function(
        fn: Callable[[float], tuple[np.ndarray, np.ndarray]],
        duration: float | None = None,
    ) -> ReferenceGenerator:
        """Arbitrary parametric reference.

        Parameters
        ----------
        fn : callable
            ``fn(t) -> (pose_6dof, twist_6dof)`` for any time *t*.
        duration : float or None
            Trajectory duration (informational).
        """
        return ReferenceGenerator(fn, duration=duration)

    # ------------------------------------------------------------------
    # Evaluation
    # ------------------------------------------------------------------

    def evaluate(
        self,
        t_start: float,
        dt: float,
        N: int,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Sample the reference at N+1 horizon steps.

        Parameters
        ----------
        t_start : float
            Current time (start of MPC horizon).
        dt : float
            MPC timestep (seconds).
        N : int
            Number of prediction steps.

        Returns
        -------
        poses : (N+1, 6) ndarray
            Reference poses at times ``t_start, t_start+dt, ..., t_start+N*dt``.
        twists : (N+1, 6) ndarray
            Reference twists at the same times.
        """
        poses = np.empty((N + 1, 6))
        twists = np.empty((N + 1, 6))
        for k in range(N + 1):
            t = t_start + k * dt
            p, tw = self._fn(t)
            poses[k] = p
            twists[k] = tw
        return poses, twists

    @property
    def duration(self) -> float | None:
        """Total trajectory duration, or None for infinite references."""
        return self._duration
