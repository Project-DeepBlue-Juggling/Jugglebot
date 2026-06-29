"""Hardware-faithful noise model for the online juggle (plan §3, cross-cutting).

Two independent, seed-reproducible noise sources, on by default, exposed as
config knobs so any rung can sweep or disable them. Every rung's acceptance must
hold *under* the noise.

* **BB throw noise** (:meth:`JuggleNoise.perturb_throw`) — the Ball Butler's
  shot-to-shot variability. The spawned ball's initial conditions get Gaussian
  noise tied to one ``bb_throw_noise_frac`` knob (default 2%): per-component
  velocity σ = frac·|vel| (≈2% of the throw speed) and per-component position
  σ = frac·|throw displacement| (2% of how far the throw travels, origin →
  touch-down). This scatters the landing well beyond the tilt lever-arm, so the
  catch must *reach* the observed landing.

* **Tracking (observation) noise** (:meth:`JuggleNoise.observe`) — mocap/sensor
  noise: every *observed* ball position gets per-component Gaussian noise with
  σ = ``tracking_noise_mm`` (default 0.5 mm; σ, not half-range).

The velocity estimate used for re-planning must NOT be a raw finite difference of
the noisy positions — differencing amplifies the 0.5 mm noise into m/s-scale
velocity error. :class:`BallisticEstimator` is the explicit handler the plan
calls for: it least-squares-fits a *known-gravity ballistic arc* to a window of
noisy observations, returning a noise-averaged (pos, vel) at the latest sample.

Pure NumPy; no MuJoCo, no ROS2. Unit-agnostic where it can be (the estimator
takes gravity in the same units as the positions you feed it).
"""
from __future__ import annotations

import dataclasses

import numpy as np


@dataclasses.dataclass
class NoiseConfig:
    """The two §3 noise knobs (both ON by default; set 0 to disable one)."""
    bb_throw_noise_frac: float = 0.02     # BB initial-condition noise (fraction)
    tracking_noise_mm: float = 0.5        # observation noise σ (mm, per component)


class JuggleNoise:
    """Seed-reproducible source of the two §3 noise streams.

    A single instance owns one ``numpy`` ``Generator`` so an entire run (the BB
    throw perturbation AND every per-tick observation) is deterministic given the
    seed — the runner already threads ``seed`` through for this reason.
    """

    def __init__(self, cfg: "NoiseConfig | None" = None, seed: int = 0):
        self.cfg = cfg if cfg is not None else NoiseConfig()
        self.rng = np.random.default_rng(seed)

    def perturb_throw(self, pos_mm, vel_mms, displacement_mm):
        """Apply BB throw noise to a ball's nominal initial conditions.

        Parameters
        ----------
        pos_mm, vel_mms : (3,) array-like
            Nominal spawn position (mm) and velocity (mm/s).
        displacement_mm : (3,) array-like
            The throw's origin -> nominal touch-down displacement vector (mm);
            its magnitude anchors the position σ (= frac·|displacement|).

        Returns
        -------
        (pos_mm, vel_mms) : tuple of (3,) ndarray
            The perturbed initial conditions. With ``bb_throw_noise_frac == 0``
            this is the input unchanged.
        """
        frac = float(self.cfg.bb_throw_noise_frac)
        pos = np.asarray(pos_mm, dtype=float).reshape(3)
        vel = np.asarray(vel_mms, dtype=float).reshape(3)
        if frac <= 0.0:
            return pos.copy(), vel.copy()
        disp_mag = float(np.linalg.norm(np.asarray(displacement_mm, float)))
        speed = float(np.linalg.norm(vel))
        pos_n = pos + self.rng.normal(0.0, frac * disp_mag, size=3)
        vel_n = vel + self.rng.normal(0.0, frac * speed, size=3)
        return pos_n, vel_n

    def observe(self, pos_mm):
        """Return a noisy observation of a true ball position (mm, per-component
        Gaussian σ = ``tracking_noise_mm``). ``0`` σ returns the input copy."""
        sigma = float(self.cfg.tracking_noise_mm)
        pos = np.asarray(pos_mm, dtype=float).reshape(3)
        if sigma <= 0.0:
            return pos.copy()
        return pos + self.rng.normal(0.0, sigma, size=3)


class BallisticEstimator:
    """Noise-averaging (pos, vel) estimate from a window of noisy observations.

    A free-flying ball follows ``p(t) = p0 + v0·Δt + ½·g·Δt²`` exactly (g known).
    Subtracting the known ``½·g·Δt²`` term leaves a problem **linear** in
    ``(p0, v0)`` per axis, so a least-squares fit over the recent observations
    recovers a smooth position and velocity at the latest sample — the tracking
    noise averages down as ``σ/√N`` (position) and faster still for velocity
    (a fit, not a difference). This is the §3 "observe position, derive velocity,
    or a small filter" handler.

    Unit-agnostic: feed positions and ``gravity`` in consistent units (the
    runner feeds SI — metres, m/s², so the estimate drops straight into
    :func:`controller.demo.juggle_planner.ballistic_touchdown`).
    """

    def __init__(self, gravity, window: "int | None" = None):
        self.g = np.asarray(gravity, dtype=float).reshape(3)
        self.window = window
        self._t: "list[float]" = []
        self._p: "list[np.ndarray]" = []

    def reset(self) -> None:
        self._t.clear()
        self._p.clear()

    def add(self, t: float, pos) -> None:
        """Append one (time, noisy-position) observation."""
        self._t.append(float(t))
        self._p.append(np.asarray(pos, dtype=float).reshape(3))
        if self.window is not None and len(self._t) > self.window:
            self._t.pop(0)
            self._p.pop(0)

    @property
    def n(self) -> int:
        return len(self._t)

    def estimate(self):
        """Least-squares ``(pos, vel)`` at the latest observation time.

        Returns ``(pos, vel)`` 3-vectors. With a single sample velocity is zero
        (under-determined) and position is that sample; with two it is the plain
        finite difference; with more it is the noise-averaged ballistic fit.
        Raises ``ValueError`` if no samples have been added.
        """
        n = len(self._t)
        if n == 0:
            raise ValueError("BallisticEstimator.estimate() with no samples")
        t = np.asarray(self._t, dtype=float)
        P = np.asarray(self._p, dtype=float)          # (n, 3)
        t_ref = t[-1]
        dt = t - t_ref                                # latest sample at Δt = 0
        if n == 1:
            return P[-1].copy(), np.zeros(3)
        # Remove the known-gravity term, then linear LS for (p0, v0): the model
        # is P - ½gΔt² = p0 + v0·Δt, design matrix A = [1, Δt].
        rhs = P - 0.5 * np.outer(dt ** 2, self.g)     # (n, 3)
        A = np.column_stack([np.ones(n), dt])         # (n, 2)
        coef, *_ = np.linalg.lstsq(A, rhs, rcond=None)  # (2, 3): [p0; v0]
        p0, v0 = coef[0], coef[1]
        # Evaluate the fitted arc at the latest time (Δt = 0): pos = p0, the
        # velocity there is v0 + g·0 = v0.
        return p0.copy(), v0.copy()
