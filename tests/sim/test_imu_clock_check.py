"""Pure pieces of tools/probes/imu_clock_check.py (BUG-132): the lag estimator's
sign convention on a synthetic signal, and the PASS/FAIL/INSUFFICIENT verdict rule.
No bag, no capture, no ports or fixed files, so it is parallel-safe."""
from __future__ import annotations

import importlib.util
import os

import numpy as np
import pytest

_P = os.path.join(os.path.dirname(__file__), "..", "..", "tools", "probes", "imu_clock_check.py")
_spec = importlib.util.spec_from_file_location("imu_clock_check", _P)
icc = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(icc)


def _signal(early_ms, seed=3):
    """Reference rate w(tm) at 100 Hz and a 1 kHz 'IMU' whose stamps run early_ms early."""
    rng = np.random.default_rng(seed)
    tm = np.arange(0.0, 20.0, 0.01)
    f = rng.uniform(0.3, 6.0, (3, 6)); ph = rng.uniform(0, 6.28, (3, 6)); am = rng.uniform(0.3, 1.0, (3, 6))
    sig = lambda t: np.stack([(am[i] * np.sin(2 * np.pi * f[i] * t[:, None] + ph[i])).sum(1) for i in range(3)], 1)
    Rtrue = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1.0]])  # IMU frame differs from the reference frame
    w = sig(tm) @ Rtrue.T
    t = np.arange(0.0, 20.0, 0.001)
    # a sample taken at true time T is stamped T - early: stamp t carries the signal at t + early
    g = sig(t + early_ms / 1e3)
    return t, g, tm, w


@pytest.mark.parametrize("early_ms", [40.0, 0.0, -12.5])
def test_lag_sign_convention(early_ms):
    t, g, tm, w = _signal(early_ms)
    lag, r, _ = icc.estimate_lag(t, g, tm, w)
    assert abs(lag - (-early_ms)) < 1.0, lag
    assert r > 0.99


def test_verdict_rule():
    v = icc.verdict
    assert v(-2.2, 5.2, 5.0, 0.1)[:2] == ("PASS", 0)
    assert v(-4.2, 6.8, 5.0, 0.1)[:2] == ("PASS", 0)
    assert v(-48.8, -38.4, 5.0, 0.1)[:2] == ("FAIL", 1)
    assert v(-2.0, -3.0, 5.0, 0.1)[:2] == ("FAIL", 1)      # IMU leads the command: impossible
    assert v(-2.0, 12.0, 5.0, 0.1)[:2] == ("FAIL", 1)      # lag2 too large
    assert v(-6.0, 5.0, 5.0, 0.1)[:2] == ("FAIL", 1)       # lag1 beyond the threshold
    assert v(-6.0, 5.0, 8.0, 0.1)[:2] == ("PASS", 0)       # --max-abs-ms respected
    assert v(0.0, 5.0, 5.0, 0.001)[:2] == ("INSUFFICIENT_MOTION", 2)
    assert v(float("nan"), 5.0, 5.0, 0.1)[:2] == ("INSUFFICIENT_MOTION", 2)
    assert v(0.0, 5.0, 5.0, float("nan"))[:2] == ("INSUFFICIENT_MOTION", 2)
