"""BUG-146: direct unit test of ``cup_realize._clamped_spline_rates``.

The solver is otherwise exercised only through whole plans of ~31 Knots, where
an off-by-one at 3 or 4 Knots (the 3-Knot seam window is a live case) would go
unseen.  Reference: the dense n x n system, rows 0 and n-1 pinning the end
rates, interior rows ``v[k-1] + 4 v[k] + v[k+1] = 3 (p[k+1] - p[k-1]) / dt``,
solved with ``np.linalg.solve``.

Behaviour at n < 3 (asserted, not endorsed): n = 0 is empty, n = 1 returns
``[v_end]`` (``out[0] = v_start`` is overwritten by ``out[-1] = v_end``), n = 2
returns ``[v_start, v_end]``.
"""
from __future__ import annotations

import numpy as np
import pytest

from jugglebot.motion.trajectory.cup_realize import _clamped_spline_rates

NS = [3, 4, 5, 9, 31]
DTS = [0.0125, 0.025, 0.1, 1.0]


def _dense_reference(p, dt, v_start, v_end):
    n = len(p)
    a = np.zeros((n, n))
    b = np.zeros(n)
    a[0, 0] = 1.0
    b[0] = v_start
    a[-1, -1] = 1.0
    b[-1] = v_end
    for k in range(1, n - 1):
        a[k, k - 1:k + 2] = (1.0, 4.0, 1.0)
        b[k] = 3.0 * (p[k + 1] - p[k - 1]) / dt
    return np.linalg.solve(a, b)


def _hermite_second_derivs(p0, p1, v0, v1, h):
    """Second derivative of the Hermite cubic at its left and right ends."""
    d = p1 - p0
    left = 6.0 * d / h ** 2 - (4.0 * v0 + 2.0 * v1) / h
    right = -6.0 * d / h ** 2 + (2.0 * v0 + 4.0 * v1) / h
    return left, right


@pytest.mark.parametrize('n', NS)
@pytest.mark.parametrize('dt', DTS)
def test_matches_dense_reference_and_is_c2(n, dt):
    rng = np.random.default_rng(1000 * n + int(dt * 1e4))
    for _ in range(5):
        p = rng.normal(scale=0.5, size=n)
        v_start, v_end = rng.normal(scale=2.0, size=2)
        got = _clamped_spline_rates(p, dt, v_start, v_end)
        ref = _dense_reference(p, dt, v_start, v_end)
        assert got.shape == (n,)
        scale = max(1.0, float(np.max(np.abs(ref))))
        np.testing.assert_allclose(got, ref, rtol=1e-12, atol=1e-12 * scale)
        # end rates pinned exactly
        assert got[0] == v_start
        assert got[-1] == v_end
        # equal second derivative at every interior knot
        for k in range(1, n - 1):
            _, left_of_k = _hermite_second_derivs(
                p[k - 1], p[k], got[k - 1], got[k], dt)
            right_of_k, _ = _hermite_second_derivs(
                p[k], p[k + 1], got[k], got[k + 1], dt)
            assert abs(left_of_k - right_of_k) <= 1e-9 * max(
                1.0, abs(left_of_k))


def test_n3_single_interior_knot_closed_form():
    p = np.array([0.0, 1.0, 3.0])
    dt = 0.5
    got = _clamped_spline_rates(p, dt, 0.7, -0.3)
    # 0.7 + 4 v + (-0.3) = 3 * 3.0 / 0.5
    assert got[1] == pytest.approx((18.0 - 0.4) / 4.0, rel=1e-14)


def test_n0_is_empty():
    assert _clamped_spline_rates(np.zeros(0), 0.025, 1.0, 2.0).shape == (0,)


def test_n1_returns_v_end_only():
    got = _clamped_spline_rates(np.array([0.3]), 0.025, 1.0, 2.0)
    assert got.shape == (1,)
    assert got[0] == 2.0


def test_n2_returns_pinned_ends():
    got = _clamped_spline_rates(np.array([0.3, 0.9]), 0.025, 1.0, 2.0)
    np.testing.assert_array_equal(got, [1.0, 2.0])
