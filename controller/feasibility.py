"""Closed-form peak-velocity / peak-acceleration bounds for quintic Hermite segments.

Used by ``make_feasible_events`` in :mod:`controller.target` to enforce the K2/K3
reference-feasibility invariants (see ``docs/reference_layer_contract.md``).

The quintic Hermite basis interpolates position, velocity, and acceleration at
both segment endpoints.  For a segment over duration ``T`` with boundary
conditions ``(p0, v0, a0)`` and ``(p1, v1, a1)``, the per-axis position is::

    p(s) = h0(s)·p0 + h1(s)·T·v0 + h2(s)·T²·a0
         + h3(s)·p1 + h4(s)·T·v1 + h5(s)·T²·a1,   s ∈ [0, 1]

where the basis ``{h_i}`` is C²-continuous with the standard pairing
``(h0, h1, h2) ↔ (p0, v0, a0)`` and ``(h3, h4, h5) ↔ (p1, v1, a1)``.

We expose two functions:

* :func:`quintic_peak_vel_per_axis` — returns per-axis ``max |dp/dt(t)|`` for
  ``t ∈ [0, T]``.  Critical points of ``dp/dt`` are roots of the quartic
  acceleration polynomial in ``s``; we compute them via ``np.roots`` and
  evaluate the velocity polynomial at each root in ``[0, 1]`` plus the
  endpoints.
* :func:`quintic_peak_acc_per_axis` — same structure, one derivative up.
  Critical points of ``d²p/dt²`` are roots of the cubic jerk polynomial.

Both return physical units (mm/s and mm/s² respectively) regardless of the
interior normalisation.
"""

from __future__ import annotations

import numpy as np


# Bumped manually whenever the closed-form quintic peak-vel / peak-acc math
# changes.  See ``controller/target.py::_REFERENCE_LAYER_VERSION`` for the
# AOT-hash rationale; the two constants are hashed together by
# ``controller/mpc.py::_nlp_source_hash``.
_FEASIBILITY_VERSION = 1


# ---------------------------------------------------------------------------
# Basis-function polynomial coefficients in s (ascending order: c0 + c1·s + …)
# ---------------------------------------------------------------------------
# Quintic Hermite basis (from controller/hermite.py, matching literature):
#   h0(s) = 1 - 10s³ + 15s⁴ - 6s⁵
#   h1(s) = s - 6s³ + 8s⁴ - 3s⁵
#   h2(s) = (s² - 3s³ + 3s⁴ - s⁵) / 2
#   h3(s) = 10s³ - 15s⁴ + 6s⁵
#   h4(s) = -4s³ + 7s⁴ - 3s⁵
#   h5(s) = (s³ - 2s⁴ + s⁵) / 2

# Derivatives w.r.t. s — coefficients for dp/ds (a quartic in s):
_H0_PRIME = np.array([0.0,   0.0, -30.0,  60.0, -30.0])          # h0'(s)
_H1_PRIME = np.array([1.0,   0.0, -18.0,  32.0, -15.0])          # h1'(s)
_H2_PRIME = np.array([0.0,   1.0,  -4.5,   6.0,  -2.5])          # h2'(s)
_H3_PRIME = np.array([0.0,   0.0,  30.0, -60.0,  30.0])          # h3'(s)
_H4_PRIME = np.array([0.0,   0.0, -12.0,  28.0, -15.0])          # h4'(s)
_H5_PRIME = np.array([0.0,   0.0,   1.5,  -4.0,   2.5])          # h5'(s)

# Second derivatives — coefficients for d²p/ds² (a cubic in s):
_H0_DPRIME = np.array([0.0, -60.0,  180.0, -120.0])              # h0''(s)
_H1_DPRIME = np.array([0.0, -36.0,   96.0,  -60.0])              # h1''(s)
_H2_DPRIME = np.array([1.0,  -9.0,   18.0,  -10.0])              # h2''(s)
_H3_DPRIME = np.array([0.0,  60.0, -180.0,  120.0])              # h3''(s)
_H4_DPRIME = np.array([0.0, -24.0,   84.0,  -60.0])              # h4''(s)
_H5_DPRIME = np.array([0.0,   3.0,  -12.0,   10.0])              # h5''(s)

# Third derivatives — coefficients for d³p/ds³ (a quadratic in s) — used as
# the "jerk polynomial" whose roots give accel-extrema.
_H0_TPRIME = np.array([-60.0,  360.0, -360.0])                    # h0'''(s)
_H1_TPRIME = np.array([-36.0,  192.0, -180.0])                    # h1'''(s)
_H2_TPRIME = np.array([ -9.0,   36.0,  -30.0])                    # h2'''(s)
_H3_TPRIME = np.array([ 60.0, -360.0,  360.0])                    # h3'''(s)
_H4_TPRIME = np.array([-24.0,  168.0, -180.0])                    # h4'''(s)
_H5_TPRIME = np.array([  3.0,  -24.0,   30.0])                    # h5'''(s)


def _poly_coeffs_ds(p0, v0, a0, p1, v1, a1, T):
    """Per-axis velocity polynomial coefficients w.r.t. s (ascending).

    Returns a (4+1, 6) array where column i is the axis-i quartic-in-s
    coefficients of ``dp/ds``.  Physical velocity is ``(dp/ds) / T``.
    """
    # Shape gymnastics: broadcast (5,) basis coeffs with (6,) boundary values.
    return (
        np.outer(_H0_PRIME, p0)
        + np.outer(_H1_PRIME, T * v0)
        + np.outer(_H2_PRIME, T * T * a0)
        + np.outer(_H3_PRIME, p1)
        + np.outer(_H4_PRIME, T * v1)
        + np.outer(_H5_PRIME, T * T * a1)
    )


def _poly_coeffs_dds(p0, v0, a0, p1, v1, a1, T):
    """Per-axis acceleration polynomial coefficients w.r.t. s.

    Physical acceleration is ``(d²p/ds²) / T²``.  Returns (3+1, 6) cubic.
    """
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
    """Evaluate polynomial with ascending-order coefficients at scalar s.

    coeffs_asc: shape (deg+1,)  →  value
    coeffs_asc: shape (deg+1, 6) →  (6,)
    """
    if coeffs_asc.ndim == 1:
        return np.polynomial.polynomial.polyval(s, coeffs_asc)
    out = np.empty(coeffs_asc.shape[1])
    for i in range(coeffs_asc.shape[1]):
        out[i] = np.polynomial.polynomial.polyval(s, coeffs_asc[:, i])
    return out


def _roots_in_unit_interval(coeffs_asc, axis_i):
    """Real roots of the axis-i polynomial in [0, 1].

    coeffs_asc is (deg+1, 6) ascending order; we use np.roots which needs
    descending order.
    """
    c = coeffs_asc[:, axis_i]
    # Strip leading zeros (high-order coeffs that are zero) to reduce degree
    # so np.roots doesn't produce spurious infinities.
    while len(c) > 1 and abs(c[-1]) < 1e-14:
        c = c[:-1]
    if len(c) < 2:
        return np.empty(0)
    r = np.roots(c[::-1])
    real = r[np.abs(r.imag) < 1e-9].real
    return real[(real >= 0.0) & (real <= 1.0)]


def quintic_peak_vel_per_axis(p0, v0, a0, p1, v1, a1, T):
    """Peak |dp/dt| per axis over t ∈ [0, T].

    Returns a (6,) array of physical-unit velocity-magnitude maxima.

    Method: extrema of velocity occur where the acceleration polynomial
    vanishes.  We find the roots of the accel cubic in [0, 1] and evaluate
    |dp/ds|/T at those roots and the endpoints.
    """
    p0 = np.asarray(p0, dtype=float); v0 = np.asarray(v0, dtype=float)
    a0 = np.asarray(a0, dtype=float); p1 = np.asarray(p1, dtype=float)
    v1 = np.asarray(v1, dtype=float); a1 = np.asarray(a1, dtype=float)

    vel_coef = _poly_coeffs_ds(p0, v0, a0, p1, v1, a1, T)    # (5, 6)
    acc_coef = _poly_coeffs_dds(p0, v0, a0, p1, v1, a1, T)   # (4, 6)

    peak = np.empty(6)
    for i in range(6):
        roots = _roots_in_unit_interval(acc_coef, i)
        candidates_s = np.concatenate(([0.0, 1.0], roots))
        vals = np.array([
            _evaluate_poly(vel_coef[:, i], s) for s in candidates_s
        ]) / T
        peak[i] = np.max(np.abs(vals))
    return peak


def quintic_peak_acc_per_axis(p0, v0, a0, p1, v1, a1, T):
    """Peak |d²p/dt²| per axis over t ∈ [0, T].  (6,) physical units."""
    p0 = np.asarray(p0, dtype=float); v0 = np.asarray(v0, dtype=float)
    a0 = np.asarray(a0, dtype=float); p1 = np.asarray(p1, dtype=float)
    v1 = np.asarray(v1, dtype=float); a1 = np.asarray(a1, dtype=float)

    acc_coef = _poly_coeffs_dds(p0, v0, a0, p1, v1, a1, T)   # (4, 6)
    jrk_coef = _poly_coeffs_ddds(p0, v0, a0, p1, v1, a1, T)  # (3, 6)

    peak = np.empty(6)
    T2 = T * T
    for i in range(6):
        roots = _roots_in_unit_interval(jrk_coef, i)
        candidates_s = np.concatenate(([0.0, 1.0], roots))
        vals = np.array([
            _evaluate_poly(acc_coef[:, i], s) for s in candidates_s
        ]) / T2
        peak[i] = np.max(np.abs(vals))
    return peak


def segment_is_feasible(
    p0: np.ndarray, v0: np.ndarray, a0: np.ndarray,
    p1: np.ndarray, v1: np.ndarray, a1: np.ndarray,
    T: float,
    v_max_mmps: float,
    a_max_mmps2: float,
    beta: float = 0.85,
) -> tuple[bool, float, float]:
    """Check K2 + K3 for a single quintic segment.

    Returns (feasible, worst_v_ratio, worst_a_ratio) where ratios are
    ``max_linear_peak / (beta * limit)``; values ≤ 1.0 imply feasibility.
    Only the linear components (indices 0..2) are checked — angular twist
    limits are not enforced at this layer (by design — see K6).
    """
    pv = quintic_peak_vel_per_axis(p0, v0, a0, p1, v1, a1, T)
    pa = quintic_peak_acc_per_axis(p0, v0, a0, p1, v1, a1, T)
    v_limit = beta * v_max_mmps
    a_limit = beta * a_max_mmps2
    vr = float(np.max(pv[:3]) / v_limit) if v_limit > 0 else np.inf
    ar = float(np.max(pa[:3]) / a_limit) if a_limit > 0 else np.inf
    return (vr <= 1.0 and ar <= 1.0), vr, ar
