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

    def eval_pose_batch(self, ts: np.ndarray) -> np.ndarray:
        """Vectorised **pose-only** eval at many times ``ts`` → ``(M, 6)`` poses.

        This is the hot path for the SpaceMouse follower gate
        (``feasibility.validate_follow``): sampling one segment at ~300 points in a
        single numpy expression instead of a Python loop of :meth:`eval` calls is
        the whole reason that gate lands at ~1.5 ms instead of ~15 ms (the Python
        per-call overhead dominates). Only the pose is returned — the follower gate
        derives leg vel/acc/jerk by finite-differencing the sampled leg extensions,
        so it never needs the analytic twist/accel here.

        Each ``t`` is clamped to ``[0, duration]`` (matching :meth:`eval` via the
        underlying ``quintic_interp`` clamp), so an out-of-range knot sample returns
        the boundary pose.
        """
        T = self.duration
        # h-basis in normalised time s = t/T, evaluated on the whole vector at once.
        s = np.clip(np.asarray(ts, dtype=float) / T, 0.0, 1.0)      # (M,)
        V0 = self.v0 * T
        V1 = self.v1 * T
        A0 = self.a0 * (T * T)
        A1 = self.a1 * (T * T)
        s2 = s * s
        s3 = s2 * s
        s4 = s3 * s
        s5 = s4 * s
        h0 = 1 - 10 * s3 + 15 * s4 - 6 * s5
        h1 = s - 6 * s3 + 8 * s4 - 3 * s5
        h2 = 0.5 * s2 - 1.5 * s3 + 1.5 * s4 - 0.5 * s5
        h3 = 10 * s3 - 15 * s4 + 6 * s5
        h4 = -4 * s3 + 7 * s4 - 3 * s5
        h5 = 0.5 * s3 - s4 + 0.5 * s5
        return (h0[:, None] * self.p0 + h1[:, None] * V0 + h2[:, None] * A0
                + h3[:, None] * self.p1 + h4[:, None] * V1 + h5[:, None] * A1)
