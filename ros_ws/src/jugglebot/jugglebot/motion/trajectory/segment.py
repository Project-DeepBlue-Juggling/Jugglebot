"""One quintic Hermite segment in 6-DoF pose space.

A ``QuinticSegment`` carries the pos/vel/accel boundary conditions at both ends
(per pose axis ``[x, y, z, rx, ry, rz]``; mm for translation, rad rotvec for
orientation; z is STOW-relative with 170.0 = ACTIVE) and a duration. ``eval(t)``
returns ``(pose, twist, accel)``, clamped past both ends (before start → the
start BCs; after end → the end pose with zero twist/accel is the *plan*'s job —
the segment itself holds the end BCs when ``t >= duration``, matching
``quintic_interp``'s clamp).

Matching pos/vel/accel at both ends is what makes every segment join — and every
mid-segment replan (which reads ``eval(t)`` to seed the next segment) — C2
continuous by construction. C2 in pose space bounds leg jerk (the binding
actuator constraint) through the smooth IK chain.

Pure Python + numpy. No ROS2 / repo-root imports.
"""

from __future__ import annotations

import numpy as np

from jugglebot.motion.trajectory.quintic import quintic_interp_with_accel

POSE_DIM = 6


class QuinticSegment:
    """A single C2 quintic segment between two 6-DoF pose boundary states."""

    __slots__ = ('p0', 'v0', 'a0', 'p1', 'v1', 'a1', 'duration')

    def __init__(self, p0, v0, a0, p1, v1, a1, duration: float):
        self.p0 = np.asarray(p0, dtype=float)
        self.v0 = np.asarray(v0, dtype=float)
        self.a0 = np.asarray(a0, dtype=float)
        self.p1 = np.asarray(p1, dtype=float)
        self.v1 = np.asarray(v1, dtype=float)
        self.a1 = np.asarray(a1, dtype=float)
        for name, vec in (('p0', self.p0), ('v0', self.v0), ('a0', self.a0),
                          ('p1', self.p1), ('v1', self.v1), ('a1', self.a1)):
            if vec.shape != (POSE_DIM,):
                raise ValueError(
                    f"QuinticSegment.{name} must be shape ({POSE_DIM},), "
                    f"got {vec.shape}")
        self.duration = float(duration)
        if self.duration <= 0.0:
            raise ValueError(
                f"QuinticSegment duration must be > 0 (got {self.duration})")

    def eval(self, t: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Sample ``(pose, twist, accel)`` at time ``t`` since segment start.

        ``t`` is clamped to ``[0, duration]`` inside ``quintic_interp_with_accel``,
        so a sample before the start returns the start BCs and a sample at/after
        the end returns the end BCs (pose ``p1``, twist ``v1``, accel ``a1``).
        """
        return quintic_interp_with_accel(
            self.p0, self.v0, self.a0, self.p1, self.v1, self.a1,
            self.duration, t)
