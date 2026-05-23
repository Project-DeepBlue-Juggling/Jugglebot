"""Periodic platform-pose trajectory for the juggling demo.

``JuggleTrajectory`` stores one period of the platform pose as dense samples
of ``(pose, twist, accel)`` on a uniform time grid and evaluates it at any
time by C2-continuous quintic-Hermite interpolation, wrapping periodically.

``build_analytic_oval`` produces a smooth analytic oval directly from a
``JugglePattern``. The offline jerk optimiser (``juggle_optimizer.py``)
produces an optimised ``JuggleTrajectory`` via
:func:`optimise_juggle_trajectory`; ``build_analytic_oval`` remains
available as the un-optimised baseline and as the optimiser's warm-start
source.

Pure NumPy — no ROS2, no CasADi.
"""
from __future__ import annotations

import numpy as np

from controller.hermite import quintic_interp_with_accel
from controller.demo.pattern import JugglePattern


class JuggleTrajectory:
    """One period of platform pose, sampled on a uniform grid.

    Sample ``k`` (k = 0 .. N-1) is at time ``k * period_s / N``; sample ``N``
    wraps to sample 0. ``eval`` interpolates with a quintic Hermite between
    bracketing samples, so position, velocity and acceleration are all
    continuous, including across the period boundary.
    """

    def __init__(self, period_s, poses, twists, accels):
        poses = np.asarray(poses, dtype=float)
        twists = np.asarray(twists, dtype=float)
        accels = np.asarray(accels, dtype=float)
        n = poses.shape[0]
        if n < 2:
            raise ValueError("JuggleTrajectory needs at least 2 samples")
        for name, arr in (("poses", poses), ("twists", twists),
                          ("accels", accels)):
            if arr.shape != (n, 6):
                raise ValueError(
                    f"{name} must have shape ({n}, 6), got {arr.shape}")
        if period_s <= 0:
            raise ValueError("period_s must be positive")
        self.period_s = float(period_s)
        self.poses = poses
        self.twists = twists
        self.accels = accels
        self._n = n
        self._dt = self.period_s / n   # node spacing; node N == node 0

    @property
    def n_samples(self) -> int:
        return self._n

    def eval(self, t):
        """Evaluate at time ``t`` (seconds); ``t`` is wrapped into one period.

        Returns ``(pose, twist, accel)``, each a (6,) ndarray.
        """
        tau = float(t) % self.period_s
        i = int(tau // self._dt)
        if i >= self._n:          # guard a float-rounding edge at tau -> period
            i = self._n - 1
        j = (i + 1) % self._n
        local = tau - i * self._dt
        return quintic_interp_with_accel(
            self.poses[i], self.twists[i], self.accels[i],
            self.poses[j], self.twists[j], self.accels[j],
            self._dt, local)

    def save(self, path) -> None:
        """Write the trajectory to a .npz file."""
        np.savez(path, period_s=self.period_s, poses=self.poses,
                 twists=self.twists, accels=self.accels)

    @classmethod
    def load(cls, path) -> "JuggleTrajectory":
        """Load a trajectory written by :meth:`save`."""
        d = np.load(path)
        return cls(float(d["period_s"]), d["poses"], d["twists"], d["accels"])


def build_analytic_oval(pattern: JugglePattern,
                        n_samples: int = 128) -> JuggleTrajectory:
    """Build a smooth analytic oval trajectory from a ``JugglePattern``.

    The platform centre traces an ellipse in the x-y plane at constant
    height, level (zero orientation) throughout:

        x(t) = (separation/2) cos(w t)
        y(t) = oval_width     sin(w t)
        z(t) = active_z

    with ``w = 2 pi / platform_period``. The THROW point ``(+sep/2, 0)`` is
    reached at ``t = 0`` and the CATCH point ``(-sep/2, 0)`` at half-period.

    This is the un-optimised baseline: it exercises the player -> IK -> plant
    pipeline and is smooth (C-infinity), but it does not minimise leg jerk and
    does not bank the platform. The offline optimiser supersedes it.
    """
    period = pattern.platform_period_s
    w = 2.0 * np.pi / period
    x_amp = 0.5 * pattern.separation_mm
    y_amp = pattern.oval_width_mm
    z = pattern.active_z_mm

    k = np.arange(n_samples)
    t = k * (period / n_samples)
    c = np.cos(w * t)
    s = np.sin(w * t)

    poses = np.zeros((n_samples, 6))
    twists = np.zeros((n_samples, 6))
    accels = np.zeros((n_samples, 6))

    poses[:, 0] = x_amp * c
    poses[:, 1] = y_amp * s
    poses[:, 2] = z
    twists[:, 0] = -x_amp * w * s
    twists[:, 1] = y_amp * w * c
    accels[:, 0] = -x_amp * w * w * c
    accels[:, 1] = -y_amp * w * w * s

    return JuggleTrajectory(period, poses, twists, accels)
