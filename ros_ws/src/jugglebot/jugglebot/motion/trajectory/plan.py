"""An immutable trajectory plan: ordered quintic segments + a terminal hold.

A ``TrajectoryPlan`` is a sequence of :class:`QuinticSegment`s that abut in time,
followed by an *implicit terminal hold*: past the final segment the plan returns
the final pose with zero twist and zero acceleration, forever. A ``HoldPlan`` is
the degenerate zero-segment case (hold a fixed pose from t=0).

``state_at(t)`` is the single sampling entry point — the emitter samples it at
``(t, t+dt, t+2·dt)`` each tick, and every replan seeds the next plan from
``old_plan.state_at(t_install)`` so pose/twist/accel are continuous (C2) across
the swap.

Pure Python + numpy. No ROS2 / repo-root imports.
"""

from __future__ import annotations

import numpy as np

from jugglebot.motion.trajectory.segment import POSE_DIM, QuinticSegment

_ZERO = np.zeros(POSE_DIM)


class TrajectoryPlan:
    """Immutable ordered segments + implicit terminal hold at the final pose."""

    __slots__ = ('segments', 'final_pose', '_starts', 'total_duration')

    def __init__(self, segments, final_pose):
        self.segments = tuple(segments)
        self.final_pose = np.asarray(final_pose, dtype=float)
        if self.final_pose.shape != (POSE_DIM,):
            raise ValueError(
                f"final_pose must be shape ({POSE_DIM},), "
                f"got {self.final_pose.shape}")
        # Cumulative segment start times (seconds since plan install).
        starts = []
        acc = 0.0
        for seg in self.segments:
            starts.append(acc)
            acc += seg.duration
        self._starts = tuple(starts)
        self.total_duration = acc

    def state_at(self, t: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Sample ``(pose, twist, accel)`` at ``t`` seconds since install.

        ``t <= 0`` returns the first segment's start BCs (or the final hold pose
        for a :class:`HoldPlan`). ``t >= total_duration`` returns the terminal
        hold (``final_pose``, zero twist, zero accel).
        """
        if not self.segments:
            # HoldPlan: fixed pose, at rest, for all time.
            return self.final_pose.copy(), _ZERO.copy(), _ZERO.copy()

        if t >= self.total_duration:
            return self.final_pose.copy(), _ZERO.copy(), _ZERO.copy()

        # Locate the active segment (t clamped to >= 0 inside eval()).
        idx = 0
        for i, start in enumerate(self._starts):
            if t >= start:
                idx = i
            else:
                break
        seg = self.segments[idx]
        return seg.eval(t - self._starts[idx])

    @property
    def kind(self) -> str:
        return 'hold' if not self.segments else 'move'


def HoldPlan(pose) -> TrajectoryPlan:
    """A degenerate plan that holds ``pose`` (zero twist/accel) for all time."""
    return TrajectoryPlan(segments=(), final_pose=pose)
