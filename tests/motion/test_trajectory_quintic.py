"""Quintic-copy cross-reference + closed-form peak-bound tests (Phase 1).

``jugglebot.motion.trajectory.quintic`` is a deliberate COPY of
``controller/hermite.py`` + ``controller/feasibility.py`` (the MVP trajectory
package must stay free of any ``controller`` import). These tests pin the copy to
the source: they assert **bit-for-bit** agreement with the ``controller``
originals across randomised boundary conditions, so the two can never silently
drift, and independently check the closed-form peak bounds against dense sampling.
"""

from __future__ import annotations

import numpy as np
import pytest

from jugglebot.motion.trajectory import quintic as tq

from controller import hermite as ch
from controller import feasibility as cf


def _rng_bcs(rng):
    p0 = rng.uniform(-100, 100, 6)
    v0 = rng.uniform(-50, 50, 6)
    a0 = rng.uniform(-20, 20, 6)
    p1 = rng.uniform(-100, 100, 6)
    v1 = rng.uniform(-50, 50, 6)
    a1 = rng.uniform(-20, 20, 6)
    return p0, v0, a0, p1, v1, a1


def test_interp_bit_for_bit_vs_controller():
    rng = np.random.default_rng(1)
    for _ in range(200):
        p0, v0, a0, p1, v1, a1 = _rng_bcs(rng)
        T = float(rng.uniform(0.1, 3.0))
        t = float(rng.uniform(0.0, T))
        pose_a, twist_a = tq.quintic_interp(p0, v0, a0, p1, v1, a1, T, t)
        pose_b, twist_b = ch.quintic_interp(p0, v0, a0, p1, v1, a1, T, t)
        assert np.array_equal(pose_a, pose_b)
        assert np.array_equal(twist_a, twist_b)


def test_interp_with_accel_bit_for_bit_vs_controller():
    rng = np.random.default_rng(2)
    for _ in range(200):
        p0, v0, a0, p1, v1, a1 = _rng_bcs(rng)
        T = float(rng.uniform(0.1, 3.0))
        t = float(rng.uniform(0.0, T))
        a = tq.quintic_interp_with_accel(p0, v0, a0, p1, v1, a1, T, t)
        b = ch.quintic_interp_with_accel(p0, v0, a0, p1, v1, a1, T, t)
        for x, y in zip(a, b):
            assert np.array_equal(x, y)


def test_jerk_integral_bit_for_bit_vs_controller():
    rng = np.random.default_rng(3)
    for _ in range(200):
        p0, v0, a0, p1, v1, a1 = _rng_bcs(rng)
        T = float(rng.uniform(0.1, 3.0))
        assert (tq.quintic_jerk_integral(p0, v0, a0, p1, v1, a1, T)
                == ch.quintic_jerk_integral(p0, v0, a0, p1, v1, a1, T))


def test_peak_bounds_bit_for_bit_vs_controller():
    rng = np.random.default_rng(4)
    for _ in range(150):
        p0, v0, a0, p1, v1, a1 = _rng_bcs(rng)
        T = float(rng.uniform(0.1, 3.0))
        assert np.array_equal(
            tq.quintic_peak_vel_per_axis(p0, v0, a0, p1, v1, a1, T),
            cf.quintic_peak_vel_per_axis(p0, v0, a0, p1, v1, a1, T))
        assert np.array_equal(
            tq.quintic_peak_acc_per_axis(p0, v0, a0, p1, v1, a1, T),
            cf.quintic_peak_acc_per_axis(p0, v0, a0, p1, v1, a1, T))


def test_peak_vel_bounds_dense_sampling():
    """Closed-form peak |dp/dt| ≥ every densely-sampled |dp/dt| (with margin)."""
    rng = np.random.default_rng(5)
    for _ in range(50):
        p0, v0, a0, p1, v1, a1 = _rng_bcs(rng)
        T = float(rng.uniform(0.2, 2.0))
        peak = tq.quintic_peak_vel_per_axis(p0, v0, a0, p1, v1, a1, T)
        sampled = np.zeros(6)
        for t in np.linspace(0, T, 400):
            _, twist = tq.quintic_interp(p0, v0, a0, p1, v1, a1, T, t)
            sampled = np.maximum(sampled, np.abs(twist))
        # Closed-form peak must bound the dense sample (small slack for the grid).
        assert np.all(peak >= sampled - 1e-6)
        assert np.all(peak <= sampled * 1.05 + 1e-6)


def test_peak_vel_raises_on_nonpositive_T():
    z = np.zeros(6)
    with pytest.raises(ValueError):
        tq.quintic_peak_vel_per_axis(z, z, z, z, z, z, 0.0)
    with pytest.raises(ValueError):
        tq.quintic_peak_acc_per_axis(z, z, z, z, z, z, -1.0)
