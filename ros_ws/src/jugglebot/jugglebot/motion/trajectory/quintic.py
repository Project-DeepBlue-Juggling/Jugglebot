"""Quintic Hermite interpolation + closed-form peak bounds (numpy-only).

**Copied**, deliberately, from ``controller/hermite.py`` and
``controller/feasibility.py`` — NOT imported. The MVP trajectory generator lives
inside the ROS2 package (``jugglebot.motion.trajectory``) and must stay free of
any repo-root / ``controller`` path dependence (``controller/`` is DORMANT and
must not be imported by ros_ws code, per the MVP plan). Copying the ~250 lines of
validated, closed-form math is the price of that boundary; the copy is pinned to
the source revision by the cross-reference test in
``tests/motion/test_trajectory_quintic.py`` (it asserts bit-for-bit agreement
with the ``controller`` originals so the two can never silently drift).

Provided:
  * ``quintic_interp`` / ``quintic_interp_with_accel`` — position/velocity(/accel)
    interpolation matching pos, vel, accel at both segment ends (C2 by
    construction at every join).
  * ``quintic_jerk_integral`` — analytic ∫ jerk² (segment smoothness score).
  * ``quintic_peak_vel_per_axis`` / ``quintic_peak_acc_per_axis`` — closed-form
    per-axis peak |dp/dt| and |d²p/dt²| over the segment (roots of the accel /
    jerk polynomial in [0,1] + endpoints), in physical units.

Pure Python + numpy. No sim, ROS2, or repo-root imports.
"""

from __future__ import annotations

import numpy as np


# ---------------------------------------------------------------------------
# Interpolation (copied verbatim from controller/hermite.py)
# ---------------------------------------------------------------------------

def quintic_interp(
    p0: np.ndarray,
    v0: np.ndarray,
    a0: np.ndarray,
    p1: np.ndarray,
    v1: np.ndarray,
    a1: np.ndarray,
    duration: float,
    t: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Quintic Hermite interpolation matching pos, vel, accel at both ends.

    The polynomial in normalised time s = t/T is::

        p(s) = h0·p0 + h1·T·v0 + h2·T²·a0 + h3·p1 + h4·T·v1 + h5·T²·a1

    Parameters
    ----------
    p0, p1 : (N,) start and end poses
    v0, v1 : (N,) start and end twists (real-time units, e.g. mm/s)
    a0, a1 : (N,) start and end accelerations (real-time units, e.g. mm/s²)
    duration : segment duration in seconds
    t : time since segment start (clamped to [0, duration])

    Returns
    -------
    pose : (N,) interpolated pose
    twist : (N,) interpolated twist (velocity in real-time units)
    """
    if duration <= 0:
        return p1.copy(), v1.copy()

    t = max(0.0, min(t, duration))
    T = duration
    s = t / T

    V0 = v0 * T
    V1 = v1 * T
    A0 = a0 * (T * T)
    A1 = a1 * (T * T)

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

    pose = h0 * p0 + h1 * V0 + h2 * A0 + h3 * p1 + h4 * V1 + h5 * A1

    inv_T = 1.0 / T
    dh0 = (-30 * s2 + 60 * s3 - 30 * s4) * inv_T
    dh1 = (1 - 18 * s2 + 32 * s3 - 15 * s4) * inv_T
    dh2 = (s - 4.5 * s2 + 6 * s3 - 2.5 * s4) * inv_T
    dh3 = (30 * s2 - 60 * s3 + 30 * s4) * inv_T
    dh4 = (-12 * s2 + 28 * s3 - 15 * s4) * inv_T
    dh5 = (1.5 * s2 - 4 * s3 + 2.5 * s4) * inv_T

    twist = dh0 * p0 + dh1 * V0 + dh2 * A0 + dh3 * p1 + dh4 * V1 + dh5 * A1

    return pose, twist


def quintic_interp_with_accel(
    p0: np.ndarray,
    v0: np.ndarray,
    a0: np.ndarray,
    p1: np.ndarray,
    v1: np.ndarray,
    a1: np.ndarray,
    duration: float,
    t: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Like :func:`quintic_interp` but also returns acceleration.

    Needed to read the current acceleration when replanning mid-segment
    (preserving C2 continuity across replan boundaries).

    Returns
    -------
    pose, twist, accel : (N,) each — real-time units.
    """
    if duration <= 0:
        return p1.copy(), v1.copy(), a1.copy()

    t = max(0.0, min(t, duration))
    T = duration
    s = t / T

    V0 = v0 * T
    V1 = v1 * T
    A0 = a0 * (T * T)
    A1 = a1 * (T * T)

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

    pose = h0 * p0 + h1 * V0 + h2 * A0 + h3 * p1 + h4 * V1 + h5 * A1

    inv_T = 1.0 / T
    dh0 = (-30 * s2 + 60 * s3 - 30 * s4) * inv_T
    dh1 = (1 - 18 * s2 + 32 * s3 - 15 * s4) * inv_T
    dh2 = (s - 4.5 * s2 + 6 * s3 - 2.5 * s4) * inv_T
    dh3 = (30 * s2 - 60 * s3 + 30 * s4) * inv_T
    dh4 = (-12 * s2 + 28 * s3 - 15 * s4) * inv_T
    dh5 = (1.5 * s2 - 4 * s3 + 2.5 * s4) * inv_T

    twist = dh0 * p0 + dh1 * V0 + dh2 * A0 + dh3 * p1 + dh4 * V1 + dh5 * A1

    inv_T2 = inv_T * inv_T
    ddh0 = (-60 * s + 180 * s2 - 120 * s3) * inv_T2
    ddh1 = (-36 * s + 96 * s2 - 60 * s3) * inv_T2
    ddh2 = (1 - 9 * s + 18 * s2 - 10 * s3) * inv_T2
    ddh3 = (60 * s - 180 * s2 + 120 * s3) * inv_T2
    ddh4 = (-24 * s + 84 * s2 - 60 * s3) * inv_T2
    ddh5 = (3 * s - 12 * s2 + 10 * s3) * inv_T2

    accel = ddh0 * p0 + ddh1 * V0 + ddh2 * A0 + ddh3 * p1 + ddh4 * V1 + ddh5 * A1

    return pose, twist, accel


def quintic_jerk_integral(
    p0: np.ndarray,
    v0: np.ndarray,
    a0: np.ndarray,
    p1: np.ndarray,
    v1: np.ndarray,
    a1: np.ndarray,
    duration: float,
) -> float:
    """Analytical integral of squared jerk over a quintic Hermite segment.

    Returns the sum over all dimensions of ∫₀ᵀ jerk²(t) dt.  Lower is smoother.
    """
    if duration <= 0:
        return float('inf')

    T = duration

    V0 = v0 * T
    V1 = v1 * T
    A0 = a0 * (T * T)
    A1 = a1 * (T * T)

    c0 = -60 * p0 - 36 * V0 - 9 * A0 + 60 * p1 - 24 * V1 + 3 * A1
    c1 = 360 * p0 + 192 * V0 + 36 * A0 - 360 * p1 + 168 * V1 - 24 * A1
    c2 = -360 * p0 - 180 * V0 - 30 * A0 + 360 * p1 - 180 * V1 + 30 * A1

    per_dim = (
        c0 * c0
        + c0 * c1
        + (2 * c0 * c2 + c1 * c1) / 3.0
        + c1 * c2 / 2.0
        + c2 * c2 / 5.0
    )

    return float(np.sum(per_dim)) / (T ** 5)


# ---------------------------------------------------------------------------
# Closed-form peak bounds (copied verbatim from controller/feasibility.py)
# ---------------------------------------------------------------------------

# Derivatives w.r.t. s — coefficients for dp/ds (a quartic in s):
_H0_PRIME = np.array([0.0,   0.0, -30.0,  60.0, -30.0])
_H1_PRIME = np.array([1.0,   0.0, -18.0,  32.0, -15.0])
_H2_PRIME = np.array([0.0,   1.0,  -4.5,   6.0,  -2.5])
_H3_PRIME = np.array([0.0,   0.0,  30.0, -60.0,  30.0])
_H4_PRIME = np.array([0.0,   0.0, -12.0,  28.0, -15.0])
_H5_PRIME = np.array([0.0,   0.0,   1.5,  -4.0,   2.5])

# Second derivatives — coefficients for d²p/ds² (a cubic in s):
_H0_DPRIME = np.array([0.0, -60.0,  180.0, -120.0])
_H1_DPRIME = np.array([0.0, -36.0,   96.0,  -60.0])
_H2_DPRIME = np.array([1.0,  -9.0,   18.0,  -10.0])
_H3_DPRIME = np.array([0.0,  60.0, -180.0,  120.0])
_H4_DPRIME = np.array([0.0, -24.0,   84.0,  -60.0])
_H5_DPRIME = np.array([0.0,   3.0,  -12.0,   10.0])

# Third derivatives — coefficients for d³p/ds³ (a quadratic in s):
_H0_TPRIME = np.array([-60.0,  360.0, -360.0])
_H1_TPRIME = np.array([-36.0,  192.0, -180.0])
_H2_TPRIME = np.array([ -9.0,   36.0,  -30.0])
_H3_TPRIME = np.array([ 60.0, -360.0,  360.0])
_H4_TPRIME = np.array([-24.0,  168.0, -180.0])
_H5_TPRIME = np.array([  3.0,  -24.0,   30.0])


def _poly_coeffs_ds(p0, v0, a0, p1, v1, a1, T):
    """Per-axis velocity polynomial coefficients w.r.t. s (ascending). (5, N)."""
    return (
        np.outer(_H0_PRIME, p0)
        + np.outer(_H1_PRIME, T * v0)
        + np.outer(_H2_PRIME, T * T * a0)
        + np.outer(_H3_PRIME, p1)
        + np.outer(_H4_PRIME, T * v1)
        + np.outer(_H5_PRIME, T * T * a1)
    )


def _poly_coeffs_dds(p0, v0, a0, p1, v1, a1, T):
    """Per-axis acceleration polynomial coefficients w.r.t. s. (4, N) cubic."""
    return (
        np.outer(_H0_DPRIME, p0)
        + np.outer(_H1_DPRIME, T * v0)
        + np.outer(_H2_DPRIME, T * T * a0)
        + np.outer(_H3_DPRIME, p1)
        + np.outer(_H4_DPRIME, T * v1)
        + np.outer(_H5_DPRIME, T * T * a1)
    )


def _poly_coeffs_ddds(p0, v0, a0, p1, v1, a1, T):
    """Per-axis jerk polynomial coefficients w.r.t. s (ascending)."""
    return (
        np.outer(_H0_TPRIME, p0)
        + np.outer(_H1_TPRIME, T * v0)
        + np.outer(_H2_TPRIME, T * T * a0)
        + np.outer(_H3_TPRIME, p1)
        + np.outer(_H4_TPRIME, T * v1)
        + np.outer(_H5_TPRIME, T * T * a1)
    )


def _evaluate_poly(coeffs_asc, s):
    """Evaluate a polynomial with ascending-order coefficients at scalar s."""
    return np.polynomial.polynomial.polyval(s, coeffs_asc)


def _roots_in_unit_interval(coeffs_asc, axis_i):
    """Real roots of the axis-i polynomial in [0, 1] (coeffs ascending, (deg+1, N))."""
    c = coeffs_asc[:, axis_i]
    while len(c) > 1 and abs(c[-1]) < 1e-14:
        c = c[:-1]
    if len(c) < 2:
        return np.empty(0)
    r = np.roots(c[::-1])
    real = r[np.abs(r.imag) < 1e-9].real
    return real[(real >= 0.0) & (real <= 1.0)]


def quintic_peak_vel_per_axis(p0, v0, a0, p1, v1, a1, T):
    """Peak |dp/dt| per axis over t ∈ [0, T]. Returns (N,) physical units.

    Raises ``ValueError`` when ``T <= 0`` (the per-axis scaling divides by T).
    """
    if T <= 0:
        raise ValueError(
            f"quintic_peak_vel_per_axis: T must be > 0 (got T={T!r}); "
            "the quintic peak-velocity formula is undefined for "
            "non-positive durations.")
    p0 = np.asarray(p0, dtype=float); v0 = np.asarray(v0, dtype=float)
    a0 = np.asarray(a0, dtype=float); p1 = np.asarray(p1, dtype=float)
    v1 = np.asarray(v1, dtype=float); a1 = np.asarray(a1, dtype=float)
    n = p0.shape[0]

    vel_coef = _poly_coeffs_ds(p0, v0, a0, p1, v1, a1, T)
    acc_coef = _poly_coeffs_dds(p0, v0, a0, p1, v1, a1, T)

    peak = np.empty(n)
    for i in range(n):
        roots = _roots_in_unit_interval(acc_coef, i)
        candidates_s = np.concatenate(([0.0, 1.0], roots))
        vals = np.array([
            _evaluate_poly(vel_coef[:, i], s) for s in candidates_s
        ]) / T
        peak[i] = np.max(np.abs(vals))
    return peak


def quintic_peak_acc_per_axis(p0, v0, a0, p1, v1, a1, T):
    """Peak |d²p/dt²| per axis over t ∈ [0, T]. Returns (N,) physical units.

    Raises ``ValueError`` when ``T <= 0`` (mirrors the peak-velocity guard).
    """
    if T <= 0:
        raise ValueError(
            f"quintic_peak_acc_per_axis: T must be > 0 (got T={T!r}); "
            "the quintic peak-acceleration formula is undefined for "
            "non-positive durations.")
    p0 = np.asarray(p0, dtype=float); v0 = np.asarray(v0, dtype=float)
    a0 = np.asarray(a0, dtype=float); p1 = np.asarray(p1, dtype=float)
    v1 = np.asarray(v1, dtype=float); a1 = np.asarray(a1, dtype=float)
    n = p0.shape[0]

    acc_coef = _poly_coeffs_dds(p0, v0, a0, p1, v1, a1, T)
    jrk_coef = _poly_coeffs_ddds(p0, v0, a0, p1, v1, a1, T)

    peak = np.empty(n)
    T2 = T * T
    for i in range(n):
        roots = _roots_in_unit_interval(jrk_coef, i)
        candidates_s = np.concatenate(([0.0, 1.0], roots))
        vals = np.array([
            _evaluate_poly(acc_coef[:, i], s) for s in candidates_s
        ]) / T2
        peak[i] = np.max(np.abs(vals))
    return peak
