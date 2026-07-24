"""Unit tests for the §3 hardware-faithful noise model (``sim.juggle_noise``).

Plan: plans/active/bb-online-juggle-tilt-rearchitecture.md §3 (cross-cutting
noise) + Phase 1 / Rung 1. Two independent, seed-reproducible noise sources
(BB throw initial-condition noise + ball-tracking observation noise) plus the
known-gravity ballistic least-squares estimator that turns the noisy
observations back into a smooth (pos, vel) for re-planning.
"""
import numpy as np
import pytest

from sim.juggle_noise import NoiseConfig, JuggleNoise, BallisticEstimator
from controller.demo.juggle_planner import GRAVITY, ballistic_touchdown


# ---- NoiseConfig defaults (the §3 knobs) --------------------------------
def test_noise_config_defaults_match_plan():
    cfg = NoiseConfig()
    assert cfg.bb_throw_noise_frac == pytest.approx(0.02)   # 2% BB throw noise
    assert cfg.tracking_noise_mm == pytest.approx(0.5)      # 0.5 mm tracking σ


# ---- JuggleNoise.perturb_throw (BB throw noise) -------------------------
def test_perturb_throw_zero_frac_is_identity():
    n = JuggleNoise(NoiseConfig(bb_throw_noise_frac=0.0), seed=0)
    pos = np.array([100.0, -50.0, 600.0])
    vel = np.array([250.0, 0.0, 3300.0])
    p, v = n.perturb_throw(pos, vel, displacement_mm=[150.0, 0.0, 250.0])
    np.testing.assert_allclose(p, pos)
    np.testing.assert_allclose(v, vel)


def test_perturb_throw_is_seed_reproducible():
    cfg = NoiseConfig(bb_throw_noise_frac=0.02)
    args = (np.zeros(3), np.array([0.0, 0.0, 3300.0]), [150.0, 0.0, 250.0])
    a = JuggleNoise(cfg, seed=7).perturb_throw(*args)
    b = JuggleNoise(cfg, seed=7).perturb_throw(*args)
    c = JuggleNoise(cfg, seed=8).perturb_throw(*args)
    np.testing.assert_allclose(a[0], b[0]); np.testing.assert_allclose(a[1], b[1])
    assert not np.allclose(a[0], c[0])      # different seed -> different draw


def test_perturb_throw_sigma_scales_with_frac_and_anchors():
    # velocity σ = frac·|vel|, position σ = frac·|displacement| (per component).
    frac = 0.05
    speed = 4000.0
    disp_mag = 300.0
    vel = np.array([speed, 0.0, 0.0])
    disp = np.array([disp_mag, 0.0, 0.0])
    n = JuggleNoise(NoiseConfig(bb_throw_noise_frac=frac), seed=3)
    P = np.empty((4000, 3)); V = np.empty((4000, 3))
    for i in range(4000):
        p, v = n.perturb_throw(np.zeros(3), vel, disp)
        P[i] = p; V[i] = v - vel        # velocity perturbation
    # empirical per-component σ ≈ frac·anchor (±10%)
    assert np.std(V[:, 0]) == pytest.approx(frac * speed, rel=0.10)
    assert np.std(P[:, 1]) == pytest.approx(frac * disp_mag, rel=0.10)


# ---- JuggleNoise.observe (tracking noise) -------------------------------
def test_observe_zero_sigma_is_identity():
    n = JuggleNoise(NoiseConfig(tracking_noise_mm=0.0), seed=0)
    pos = np.array([1.0, 2.0, 3.0])
    np.testing.assert_allclose(n.observe(pos), pos)


def test_observe_sigma_and_reproducible():
    sigma = 0.5
    n = JuggleNoise(NoiseConfig(tracking_noise_mm=sigma), seed=1)
    draws = np.array([n.observe(np.zeros(3)) for _ in range(5000)])
    assert np.std(draws[:, 0]) == pytest.approx(sigma, rel=0.10)
    # reproducible
    a = JuggleNoise(NoiseConfig(tracking_noise_mm=sigma), seed=1).observe(np.ones(3))
    b = JuggleNoise(NoiseConfig(tracking_noise_mm=sigma), seed=1).observe(np.ones(3))
    np.testing.assert_allclose(a, b)


# ---- BallisticEstimator -------------------------------------------------
def _arc(p0, v0, t):
    return p0 + v0 * t + 0.5 * GRAVITY * t ** 2


def test_estimator_recovers_exact_arc_without_noise():
    p0 = np.array([0.1, -0.2, 1.1]); v0 = np.array([0.3, 0.1, 0.5])
    est = BallisticEstimator(GRAVITY)
    ts = np.linspace(0.0, 0.3, 13)
    for t in ts:
        est.add(t, _arc(p0, v0, t))
    pos, vel = est.estimate()
    # estimate is at the LATEST sample time (ts[-1])
    np.testing.assert_allclose(pos, _arc(p0, v0, ts[-1]), atol=1e-9)
    np.testing.assert_allclose(vel, v0 + GRAVITY * ts[-1], atol=1e-9)


def test_estimator_single_and_two_samples():
    est = BallisticEstimator(GRAVITY)
    with pytest.raises(ValueError):
        est.estimate()
    est.add(0.0, np.array([1.0, 2.0, 3.0]))
    pos, vel = est.estimate()
    np.testing.assert_allclose(pos, [1.0, 2.0, 3.0])
    np.testing.assert_allclose(vel, [0.0, 0.0, 0.0])     # under-determined -> 0
    est.add(0.025, np.array([1.0, 2.0, 3.0]) + np.array([0.1, 0.0, -0.2]) * 0.025
            + 0.5 * GRAVITY * 0.025 ** 2)
    _, vel2 = est.estimate()
    assert np.isfinite(vel2).all()


def test_estimator_averages_down_tracking_noise():
    # The fit recovers velocity far better than a raw two-point finite difference
    # of the noisy positions — the whole point of the §3 estimator.
    p0 = np.array([0.0, 0.0, 1.2]); v0 = np.array([0.25, 0.0, 0.0])
    sigma = 0.0005   # 0.5 mm in metres
    rng = np.random.default_rng(0)
    n_trials = 200
    fit_err = np.empty(n_trials); diff_err = np.empty(n_trials)
    ts = np.linspace(0.0, 0.30, 13)
    for k in range(n_trials):
        est = BallisticEstimator(GRAVITY)
        noisy = [_arc(p0, v0, t) + rng.normal(0, sigma, 3) for t in ts]
        for t, p in zip(ts, noisy):
            est.add(t, p)
        _, vel = est.estimate()
        fit_err[k] = abs(vel[0] - v0[0])
        diff_err[k] = abs((noisy[-1][0] - noisy[-2][0]) / (ts[-1] - ts[-2]) - v0[0])
    assert np.mean(fit_err) < 0.25 * np.mean(diff_err)


def test_estimator_touchdown_consistent_with_planner():
    # Estimate -> ballistic_touchdown reproduces the true touch-down of a known arc.
    p0 = np.array([-0.1, 0.05, 1.15]); v0 = np.array([0.25, -0.1, 0.0])
    catch_z = 0.84
    est = BallisticEstimator(GRAVITY)
    ts = np.linspace(0.0, 0.20, 9)
    for t in ts:
        est.add(t, _arc(p0, v0, t))
    pos_e, vel_e = est.estimate()
    t_td, pos_td, _ = ballistic_touchdown(pos_e, vel_e, catch_z)
    # propagate the TRUE arc from the latest sample by t_td -> should hit catch_z
    true_now = _arc(p0, v0, ts[-1])
    true_vel = v0 + GRAVITY * ts[-1]
    landing = true_now + true_vel * t_td + 0.5 * GRAVITY * t_td ** 2
    assert landing[2] == pytest.approx(catch_z, abs=1e-6)
    np.testing.assert_allclose(landing[:2], pos_td[:2], atol=1e-6)
