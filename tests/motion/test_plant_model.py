"""f0, the Learner's global plant model (``motion/skills/plant_model.py``; plan
``plans/archived/learner-generalisation.md`` § 2.2 and § 5 T-U1, T-U2, T-U5, T-R2).

Real arithmetic throughout: synthetic plants with known theta, and the committed corpus copy for
the production-vs-probe pin.

Pin recipe (generated 2026-11-06 on the Jetson, worktree Jugglebot-p3 @ 0b60a7a6 + the P1 L1
unit): ``OPENBLAS_NUM_THREADS=1 python -m pytest tests/motion/test_plant_model.py -q -p
no:cacheprovider -k pin``; the literal values were printed by the same replay
(``PlantModel.theta()`` after the first 400 rows of ``temp/learn/jugglebot/memory.csv`` in file
order, read READ-ONLY through the probe's own v1 loader ``memory_regression/models.load``) and are
bit-identical to the probe prototype ``tools/probes/learner_replay.run_rls`` on the same rows
under the settings stated in ``test_pin_production_equals_the_probe_prototype_on_400_corpus_rows``.
"""
from __future__ import annotations

import copy
import math
import os
import sys

import numpy as np
import pytest

from jugglebot.motion.skills.plant_model import PlantModel, PlantModelConfig

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
CORPUS = os.path.join(REPO, 'temp', 'learn', 'jugglebot', 'memory.csv')


def _plant(theta, u, site_x, ball, nd_x=(0.0,), nd_y=(0.0,)):
    """True plant: theta = (K, b_x, b_y, s); per-ball offsets nd_x/nd_y indexed by ball - 1."""
    K, bx, by, s = theta
    dx = nd_x[ball - 1] if ball >= 1 else 0.0
    dy = nd_y[ball - 1] if ball >= 1 else 0.0
    return np.array([u[0] + bx + s * site_x + dx, u[1] + by + dy, K * u[2]])


def _cmd(rng):
    return np.array([rng.uniform(-0.02, 0.02), rng.uniform(-0.02, 0.02), rng.uniform(0.8, 1.3)])


# --------------------------------------------------------------------------- T-U1, T-R2
@pytest.mark.parametrize('offsets', [False, True])
def test_t_u1_forward_inverse_round_trip_for_random_theta_and_sites(offsets):
    rng = np.random.default_rng(1)
    cfg = PlantModelConfig(ball_offsets=offsets)
    for trial in range(20):
        m = PlantModel(cfg)
        # drive the model to a random, fully reported theta (rows from 3 Sites, both balls)
        th = (rng.uniform(0.9, 1.3), rng.uniform(-0.03, 0.03), rng.uniform(-0.03, 0.03),
              rng.uniform(-0.3, 0.3))
        ndx, ndy = (rng.uniform(-0.03, 0.03),), (rng.uniform(-0.03, 0.03),)
        for i in range(60):
            sx, ball = (-0.15, 0.0, 0.125)[i % 3], i % 2
            u = _cmd(rng)
            m.observe(u, _plant(th, u, sx, ball, ndx, ndy) + rng.normal(0, 0.002, 3), sx, ball, 'free_cup')
        # free_cup rows only: the carried kind's lateral law (BUG-138, '@carried') stays at prior
        assert [n for n in m.at_prior() if not n.endswith('@carried')] == []
        for _ in range(10):
            y = np.array([rng.uniform(-0.1, 0.1), rng.uniform(-0.1, 0.1), rng.uniform(0.5, 1.5)])
            sx, ball = rng.uniform(-0.3, 0.3), int(rng.integers(0, 2))
            np.testing.assert_allclose(m.forward(m.inverse(y, sx, ball, 'free_cup'), sx, ball, 'free_cup'), y, rtol=0, atol=1e-12)
            u = _cmd(rng)
            np.testing.assert_allclose(m.inverse(m.forward(u, sx, ball, 'free_cup'), sx, ball, 'free_cup'), u, rtol=0, atol=1e-12)
        if offsets:
            # Delta of ball 0 is zero: ball 0's map carries b and s only
            th_r = m.theta()
            u = _cmd(rng)
            y0 = m.forward(u, 0.05, 0, 'free_cup')
            np.testing.assert_allclose(y0, [u[0] + th_r[1] + th_r[3] * 0.05, u[1] + th_r[2], th_r[0] * u[2]],
                                       rtol=0, atol=1e-15)
            assert m.forward(u, 0.05, 1, 'free_cup')[0] - y0[0] == pytest.approx(th_r[4], abs=1e-15)


def test_t_r2_an_empty_model_is_exactly_the_identity():
    for offsets in (False, True):
        m = PlantModel(PlantModelConfig(ball_offsets=offsets))
        y = np.array([0.0123, -0.0456, 0.987654321])
        for sx, ball in ((0.0, 0), (-0.075, 1), (0.125, 0)):
            assert np.array_equal(m.inverse(y, sx, ball, 'free_cup'), y)
            assert np.array_equal(m.forward(y, sx, ball, 'free_cup'), y)
        th = m.theta()
        assert th[0] == 1.0 and np.all(th[1:] == 0.0)
        assert set(m.at_prior()) == set(m.param_names)
        for sx in (-0.3, -0.075, 0.0, 0.125, 0.3):
            assert m.offset_x(sx) == (0.0, True)
            assert np.array_equal(m.forward_rows(np.tile(y, (2, 1)), np.array([sx, sx]), 0, 'free_cup'), np.tile(y, (2, 1)))


# --------------------------------------------------------------------------- T-U2
def test_t_u2_rls_recovers_a_synthetic_theta_from_noisy_rows():
    rng = np.random.default_rng(7)
    th = (1.081, 0.0099, 0.0150, -0.0915)
    ndx, ndy = (0.012,), (-0.008,)
    m = PlantModel(PlantModelConfig(ball_offsets=True))
    sites = (-0.15, -0.075, 0.0, 0.075, 0.15)
    for i in range(800):
        sx, ball = sites[i % 5], (i // 5) % 2
        u = _cmd(rng)
        y = _plant(th, u, sx, ball, ndx, ndy) + rng.normal(0, [0.020, 0.020, 0.010])
        assert m.observe(u, y, sx, ball, 'free_cup')
    got = m.theta()
    # tolerances ~3-4 posterior sd at lambda 0.99 (an ~100-row memory for K and b)
    assert got[0] == pytest.approx(th[0], abs=0.004)
    assert got[1] == pytest.approx(th[1], abs=0.008)
    assert got[2] == pytest.approx(th[2], abs=0.008)
    assert got[3] == pytest.approx(th[3], abs=0.03)
    assert got[4] == pytest.approx(ndx[0], abs=0.005)
    assert got[5] == pytest.approx(ndy[0], abs=0.005)
    assert m.n_rejected == 0 and m.n_accepted == 800


def test_t_u2_tracks_a_slow_drift_in_K():
    rng = np.random.default_rng(3)
    m = PlantModel()
    n = 1500
    for i in range(n):
        K = 1.06 + 0.04 * i / (n - 1)        # +4 % over 1500 throws
        u = _cmd(rng)
        sx = (-0.075, 0.075)[i % 2]
        m.observe(u, _plant((K, 0.01, 0.0, -0.09), u, sx, 0) + rng.normal(0, [0.02, 0.02, 0.01]), sx, 0, 'free_cup')
    # lambda 0.99 lags a ramp by ~ slope x 1/(1 - lambda) = 2.7e-5 x 100 = 0.0027
    assert m.theta()[0] == pytest.approx(1.10, abs=0.006)


def _two_site_then_one_site(model):
    rng = np.random.default_rng(11)
    th = (1.08, 0.010, 0.012, -0.09)
    for i in range(200):                            # s is identified: two Sites
        sx = (-0.125, 0.125)[i % 2]
        u = _cmd(rng)
        model.observe(u, _plant(th, u, sx, 0) + rng.normal(0, [0.02, 0.02, 0.01]), sx, 0, 'free_cup')
    s_before, sd_before = model.theta()[3], model.sd()[3]
    for i in range(2000):                           # then a long single-Site stretch (the oval)
        u = _cmd(rng)
        model.observe(u, _plant(th, u, -0.075, 0) + rng.normal(0, [0.02, 0.02, 0.01]), -0.075, 0, 'free_cup')
    return s_before, sd_before


def test_t_u2_s_does_not_wind_up_during_a_long_single_site_stretch():
    """The probe's defect (2026-11-06): with ONE forgetting factor for every parameter, s's
    unexcited direction winds up (P grows by 1/lambda per row) during a single-Site stretch until
    s falls back to its prior (on the corpus after 410-1958 rows). s is never forgotten."""
    m = PlantModel()
    s_before, sd_before = _two_site_then_one_site(m)
    assert 's' not in m.at_prior()
    assert m.sd()[3] <= sd_before + 1e-12           # never forgotten: its sd can only shrink
    assert m.theta()[3] == pytest.approx(-0.09, abs=0.04)
    assert s_before != 0.0


def test_t_u2_an_uncertain_parameter_reports_its_prior():
    rng = np.random.default_rng(5)
    m = PlantModel(PlantModelConfig(ball_offsets=True))
    th = (1.08, 0.010, 0.012, -0.09)
    for i in range(40):        # one Site, ball 0 only: s is not identified, Delta_1 never seen
        u = _cmd(rng)
        m.observe(u, _plant(th, u, -0.075, 0) + rng.normal(0, [0.002, 0.002, 0.002]), -0.075, 0, 'free_cup')
    names = m.param_names
    # at ONE off-centre Site b_x and s are confounded (only b_x + s x is seen), so BOTH stay at
    # their prior: b_x's marginal sd keeps |x| sd_s = 22.5 mm > 15 mm. Axis 0 is then L2's.
    carried = {n for n in names if n.endswith('@carried')}     # no carried rows (BUG-138)
    assert set(m.at_prior()) == {'b_x', 's', 'delta1_x', 'delta1_y'} | carried
    raw, rep, sd = m.raw(), m.theta(), m.sd()
    i_s = names.index('s')
    assert raw[i_s] != 0.0 and rep[i_s] == 0.0      # a raw estimate exists; the prior is reported
    assert sd[i_s] > 0.5 * 0.30
    assert rep[names.index('delta1_x')] == 0.0 and rep[names.index('delta1_y')] == 0.0
    assert rep[0] == pytest.approx(1.08, abs=0.005)  # K left its prior


# --------------------------------------------------------------------------- per-query axis-0 offset
OFFSET_X_TRUE = 0.010 + (-0.09) * (-0.075) + 0.008     # 24.75 mm at x = -0.075


def _one_site(seed, n, site_x=-0.075, m=None):
    rng = np.random.default_rng(seed)
    m = PlantModel() if m is None else m
    for _ in range(n):
        u = _cmd(rng)
        y = np.array([u[0] + OFFSET_X_TRUE, u[1] + 0.012, 1.08 * u[2]]) + rng.normal(0, [0.020, 0.020, 0.010])
        m.observe(u, y, site_x, 0, 'free_cup')
    return m


def test_one_off_centre_site_applies_its_offset_though_b_x_and_s_stay_at_prior():
    """At ONE Site only b_x + s x is observed: b_x and s each stay uncertain (per-parameter rule:
    both at prior), but the offset AT that Site is determined and is applied (per-query rule:
    predictive sd sqrt(a^T P a) <= 0.5 x the prior predictive sd, 37.5 mm at x = -0.075). Measured
    2026-11-06: applied from the 1st row; after 30 rows of 20 mm-noise data the 10-seed mean
    |error| is ~3 mm (posterior sd ~4 mm)."""
    errs = []
    for seed in range(10):
        m = _one_site(seed, 30)
        assert {'b_x', 's'} <= set(m.at_prior())
        ox, at_prior = m.offset_x(-0.075)
        assert not at_prior
        errs.append(ox - OFFSET_X_TRUE)
        u = np.array([0.0, 0.0, 1.0])
        assert m.forward(u, -0.075, 0, 'free_cup')[0] == pytest.approx(ox, abs=1e-15)
        assert m.inverse(np.array([0.0, 0.0, 1.0]), -0.075, 0, 'free_cup')[0] == pytest.approx(-ox, abs=1e-15)
        assert 'applied axis-0 offset at x -0.075 m: %+.1f mm' % (1e3 * ox) in m.summary(-0.075)
    errs = np.abs(np.array(errs))
    assert errs.max() < 0.012 and errs.mean() < 0.005, errs * 1e3
    assert not _one_site(0, 1).offset_x(-0.075)[1]           # applied from the 1st row


def test_a_far_site_gets_no_axis0_correction_until_a_second_site_fixes_the_line():
    """The b / s split is unknown after one Site, so at a far Site (x = +0.125) the model applies
    the identity (its prior) rather than extrapolating a guessed slope; once rows from a second
    Site arrive, the far-Site prediction follows the fitted line b_x + s x."""
    m = _one_site(1, 60)
    assert m.offset_x(0.125) == (0.0, True)
    u = np.array([0.003, -0.002, 1.0])
    assert m.forward(u, 0.125, 0, 'free_cup')[0] == u[0]
    assert m.offset_x(0.0)[1] is True                       # even at x = 0 the split matters
    # a second Site with the true line b_x 0.018, s -0.09: offset +0.075 -> 0.01125
    rng = np.random.default_rng(2)
    for _ in range(60):
        u2 = _cmd(rng)
        y = np.array([u2[0] + 0.018 - 0.09 * 0.075, u2[1] + 0.012, 1.08 * u2[2]]) + rng.normal(0, [0.020, 0.020, 0.010])
        m.observe(u2, y, 0.075, 0, 'free_cup')
    ox, at_prior = m.offset_x(0.125)
    assert not at_prior
    raw = m.raw()
    assert ox == pytest.approx(raw[1] + raw[3] * 0.125, abs=1e-15)   # the fitted line
    assert ox == pytest.approx(0.018 - 0.09 * 0.125, abs=0.012)


@pytest.mark.parametrize('n', [60, 150, 300, 1500])
def test_bug137_a_long_single_site_memory_never_applies_an_offset_at_a_far_site(n):
    """BUG-137: b is now a random walk (it was forgotten by lambda 0.99 before
    BUG-139) and s is not forgotten, so in the forgetting filter's
    covariance I_ss keeps growing at one Site while I_bb saturates: s's sd fell from 0.24 (30 rows)
    to 0.096 (150) and 0.008 (1500) with no excitation, and from ~150 rows on an invented slope was
    applied at a far Site (orchestrator probe 2026-11-25, 10 seeds: 10/10 applied, -53..+60 mm
    against a true +6.75 mm). The decisions now read the unforgotten covariance, so at ANY length a
    single-Site Memory keeps b_x and s at prior and applies 0 at a far Site, while the Site itself
    is still corrected."""
    for seed in range(5):
        m = _one_site(seed, n)
        assert m.offset_x(0.125) == (0.0, True)
        assert m.offset_x(0.0) == (0.0, True)
        assert {'b_x', 's'} <= set(m.at_prior())
        th = m.theta()
        assert th[1] == 0.0 and th[3] == 0.0
        u = np.array([0.003, -0.002, 1.0])
        assert m.forward(u, 0.125, 0, 'free_cup')[0] == u[0]
        assert m.inverse(u, 0.125, 0, 'free_cup')[0] == u[0]
        ox, at_prior = m.offset_x(-0.075)                   # the Site itself: still applied
        assert not at_prior
        assert ox == pytest.approx(OFFSET_X_TRUE, abs=0.012)


def _second_site(m, n, seed):
    rng = np.random.default_rng(seed)
    for _ in range(n):
        u2 = _cmd(rng)
        y = np.array([u2[0] + 0.018 - 0.09 * 0.075, u2[1] + 0.012, 1.08 * u2[2]]) + rng.normal(0, [0.020, 0.020, 0.010])
        m.observe(u2, y, 0.075, 0, 'free_cup')


@pytest.mark.parametrize('n', [300, 1500])
def test_bug137_after_a_long_single_site_memory_a_second_site_still_fixes_the_line(n):
    """Multi-Site still learns: the unforgotten covariance does not hold the far Site at prior
    once a second Site has been observed; the applied offset there is the fitted line."""
    for seed in range(3):
        m = _one_site(seed, n)
        _second_site(m, 60, 100 + seed)
        ox, at_prior = m.offset_x(0.125)
        assert not at_prior
        raw = m.raw()
        assert ox == pytest.approx(raw[1] + raw[3] * 0.125, abs=1e-15)
        assert 's' not in m.at_prior() and 'b_x' not in m.at_prior()


def test_bug137_residue_the_slope_value_recovers_after_a_long_single_site_memory():
    """BUG-137 residue, fixed by BUG-139 (2026-11-27; was ``xfail(strict=True)``, assertion
    unchanged): with b forgotten by per-parameter lambda the VALUE of s was corrupted by the
    information that forgetting manufactured during the single-Site run, and after 60 second-Site
    rows the far-Site line was off by up to 27 mm (10 seeds, ``/tmp/probe_bug139.py``). With b a
    random walk it is off by at most 6.4 mm over the same 10 seeds."""
    for seed in range(6):
        m = _one_site(seed, 1500)
        _second_site(m, 60, 100 + seed)
        assert m.offset_x(0.125)[0] == pytest.approx(0.018 - 0.09 * 0.125, abs=0.012)


def test_bug139_the_slope_after_a_second_site_does_not_depend_on_the_single_site_length():
    """BUG-139: what a second Site teaches about s must not depend on how long the Memory sat at
    the first one (no information about s was received there, so none may be credited). 300 vs
    1500 single-Site rows, then the same 60 second-Site rows: the far-Site offset differs by under
    4 mm per seed and s's VALUE is within 0.06 /m of the truth (the residue test's 12 mm at
    x = +0.125 over the 0.2 m lever from the first Site). Measured 2026-11-27 on seeds 0-5: new
    <= 1.5 mm apart, s within 0.03 /m; old 0.2-22 mm apart (seed 2: +2.9 vs +24.9 mm)."""
    for seed in range(6):
        far, s = [], []
        for n in (300, 1500):
            m = _one_site(seed, n)
            _second_site(m, 60, 100 + seed)
            far.append(m.offset_x(0.125)[0])
            s.append(m.raw()[3])
        assert abs(far[1] - far[0]) < 0.004, (seed, far)
        assert s[1] == pytest.approx(-0.09, abs=0.06), (seed, s)


@pytest.mark.parametrize('n', [150, 1500])
def test_bug139_forgetting_never_holds_more_information_than_the_rows_gave(n):
    """The class invariant behind BUG-137 and BUG-139: forgetting may only REMOVE information, so
    the forgetting filter's covariance is never below the unforgotten one in PSD order
    (P - P_unforgotten >= 0), on every filter, and in particular s's sd is never below what the
    rows determine. Old per-parameter lambda broke it after ~150 single-Site rows (s's sd 0.096 at
    150 rows, 0.008 at 1500, against an unforgotten ~0.24). The invariant holds inside EACH
    release kind's lateral pair (BUG-138 x BUG-139, merged 2026-11-27): the carried kind gets its
    own single-Site stretch at its own law."""
    for seed in range(3):
        m = _one_site(seed, n)
        rng = np.random.default_rng(900 + seed)
        for _ in range(n):
            u = _cmd(rng)
            y = np.array([u[0] + 0.179 - 1.23 * 0.075, u[1] + 0.004, 1.08 * u[2]]) \
                + rng.normal(0, [0.020, 0.020, 0.010])
            m.observe(u, y, -0.075, 0, 'carried')
        assert m._lat['carried'].n_accepted > 0.9 * n
        filters = [m._fa] + [f for lat in m._lat.values() for f in (lat.fx, lat.fy)]
        for f in filters:
            assert np.linalg.eigvalsh(f.P - f.P_unforgotten).min() >= -1e-15
        for name in ('s', 's@carried'):
            i = m.param_names.index(name)
            assert m.sd()[i] >= m.sd_unforgotten()[i] - 1e-15


def test_bug139_a_lambda_on_a_multi_parameter_filter_is_refused():
    """The per-parameter lambda that manufactured information cannot come back by construction:
    the filter takes lambda only when it has one parameter (K), process noise q otherwise."""
    from jugglebot.motion.skills.plant_model import _Filter
    _Filter([1.0], [0.1], 1e-4, lam=0.99)
    _Filter([0.0, 0.0], [0.03, 0.3], 4e-4, q=[4e-8, 0.0])
    with pytest.raises(ValueError, match='1-parameter'):
        _Filter([0.0, 0.0], [0.03, 0.3], 4e-4, lam=0.99)


def test_bug139_q_is_derived_from_lambda_and_can_be_overridden():
    cfg = PlantModelConfig()
    assert cfg.q_b_m2 == pytest.approx((1 - 0.99) ** 2 * 0.020 ** 2 / 0.99, rel=1e-12)
    assert PlantModelConfig(forgetting=1.0).q_b_m2 == 0.0
    assert PlantModelConfig(process_noise_b_m2=1e-7).q_b_m2 == 1e-7
    for bad in (-1e-9, float('nan'), float('inf'), True, '1e-8'):
        with pytest.raises(ValueError, match='process_noise_b_m2'):
            PlantModelConfig(process_noise_b_m2=bad)


@pytest.mark.parametrize('axis', [0, 1])
def test_bug139_b_tracks_a_step_at_the_rate_lambda_gave(axis):
    """The random walk forgets b at lambda's rate: q = (1 - lam)^2 R / lam is the q whose steady
    gain is 1 - lam (``PlantModelConfig.q_b_m2``), an exponential memory of 1 / (1 - lam) = 100
    rows. After 600 two-Site rows a +20 mm step in b: the 10-seed mean estimate covers 63 % of the
    step in 100 rows +- a factor 1.5 (measured 2026-11-27: 101 rows on axis 0 and 99 on axis 1, the
    same as the old lambda's 101 / 99, ``/tmp/probe_bug139.py``), and b_y's steady posterior variance
    is (1 - lam) R within 5 %."""
    step = np.array([0.020, 0.0] if axis == 0 else [0.0, 0.020])
    curves = []
    for seed in range(10):
        rng = np.random.default_rng(500 + seed)
        m = PlantModel()
        for i in range(600):
            sx = (-0.075, 0.075)[i % 2]
            u = _cmd(rng)
            m.observe(u, _plant((1.08, 0.010, 0.012, -0.09), u, sx, 0) + rng.normal(0, [0.02, 0.02, 0.01]), sx, 0, 'free_cup')
        if axis == 1:
            assert m.sd()[2] ** 2 == pytest.approx(0.01 * 0.020 ** 2, rel=0.05)
        c = []
        for i in range(250):
            sx = (-0.075, 0.075)[i % 2]
            u = _cmd(rng)
            y = _plant((1.08, 0.010, 0.012, -0.09), u, sx, 0) + np.append(step, 0.0)
            m.observe(u, y + rng.normal(0, [0.02, 0.02, 0.01]), sx, 0, 'free_cup')
            c.append(m.raw()[1] - 0.010 if axis == 0 else m.raw()[2] - 0.012)
        curves.append(c)
    mean = np.mean(curves, axis=0)
    n63 = int(np.argmax(mean >= 0.632 * 0.020)) + 1
    assert mean.max() >= 0.632 * 0.020 and 67 <= n63 <= 150, n63


def test_bug137_unforgotten_covariance_is_the_lambda_one_filter_and_rejected_rows_leave_it_alone():
    """P_unforgotten of the default model is the covariance of the same accepted rows with no
    forgetting, i.e. the P of a lambda = 1 model fed identical rows (small-noise rows well inside
    the inlier band, so both models accept every row); and a rejected 300 mm lateral outlier leaves
    every P_unforgotten bit-identical."""
    rng = np.random.default_rng(7)
    m = PlantModel()
    m1 = PlantModel(PlantModelConfig(forgetting=1.0))
    for i in range(60):
        sx = (-0.075, 0.075)[i % 2]
        u = _cmd(rng)
        y = _plant((1.08, 0.01, 0.012, -0.09), u, sx, 0) + rng.normal(0, [0.002, 0.002, 0.001])
        assert m.observe(u, y, sx, 0, 'free_cup') is True
        assert m1.observe(u, y, sx, 0, 'free_cup') is True
    fc, fc1 = m._lat['free_cup'], m1._lat['free_cup']
    for f, f1 in ((m._fa, m1._fa), (fc.fx, fc1.fx), (fc.fy, fc1.fy)):
        np.testing.assert_allclose(f.P_unforgotten, f1.P, rtol=1e-9)
    every = (m._fa,) + tuple(g for lat in m._lat.values() for g in (lat.fx, lat.fy))
    before = [f.P_unforgotten.copy() for f in every]
    u = np.array([0.0, 0.0, 1.0])
    y = m.forward(u, 0.075, 0, 'free_cup') + np.array([0.0, 0.300, 0.0])
    assert m.observe(u, y, 0.075, 0, 'free_cup') is False
    for f, b in zip(every, before):
        assert np.array_equal(f.P_unforgotten, b)


# --------------------------------------------------------------------------- the inlier rule
def _converged(rng, K=1.08):
    m = PlantModel()
    for i in range(100):
        sx = (-0.075, 0.075)[i % 2]
        u = _cmd(rng)
        m.observe(u, _plant((K, 0.01, 0.012, -0.09), u, sx, 0) + rng.normal(0, [0.02, 0.02, 0.01]), sx, 0, 'free_cup')
    return m


def test_a_300_mm_lateral_outlier_is_rejected_and_counted():
    rng = np.random.default_rng(2)
    m = _converged(rng)
    th, n_acc, n_rej = m.theta().copy(), m.n_accepted, m.n_rejected
    u = np.array([0.0, 0.0, 1.0])
    y = m.forward(u, 0.075, 0, 'free_cup') + np.array([0.0, 0.300, 0.0])
    assert m.observe(u, y, 0.075, 0, 'free_cup') is False
    assert m.n_rejected == n_rej + 1 and m.n_accepted == n_acc
    assert 'axis 1' in m.last_rejection
    assert np.array_equal(m.theta(), th)


def test_the_sims_gain_1_23_is_learned_from_cold():
    """The apex band widens with K's posterior sd: a plant with gain 1.23 (the sim's) is outside
    the converged 9 % band about the prior K = 1, and must still be learned from the first row."""
    rng = np.random.default_rng(4)
    m = PlantModel()
    for i in range(20):
        u = _cmd(rng)
        m.observe(u, _plant((1.23, 0.0, 0.0, 0.0), u, 0.0, 0) + rng.normal(0, [0.005, 0.005, 0.005]), 0.0, 0, 'free_cup')
    assert m.n_rejected == 0
    assert m.theta()[0] == pytest.approx(1.23, abs=0.005)


def test_a_single_wild_apex_row_after_convergence_is_rejected():
    rng = np.random.default_rng(6)
    m = _converged(rng)
    th = m.theta().copy()
    u = np.array([0.0, 0.0, 1.0])
    y = m.forward(u, 0.075, 0, 'free_cup')
    y[2] = 1.25 * u[2]                    # 16 % above K = 1.08 (converged band 9 %)
    assert m.observe(u, y, 0.075, 0, 'free_cup') is False
    assert 'apex' in m.last_rejection
    assert np.array_equal(m.theta(), th)


# --------------------------------------------------------------------------- T-U5
@pytest.mark.parametrize('bad', [math.nan, math.inf, -math.inf])
def test_t_u5_every_non_finite_input_raises_value_error_by_name(bad):
    m = _converged(np.random.default_rng(8))
    u, y = np.array([0.0, 0.0, 1.0]), np.array([0.01, 0.0, 1.08])
    for k in range(3):
        ub = u.copy(); ub[k] = bad
        with pytest.raises(ValueError, match=r'\bu\b'):
            m.observe(ub, y, 0.0, 0, 'free_cup')
        with pytest.raises(ValueError, match=r'\bu\b'):
            m.forward(ub, 0.0, 0, 'free_cup')
        yb = y.copy(); yb[k] = bad
        with pytest.raises(ValueError, match=r'\by\b'):
            m.observe(u, yb, 0.0, 0, 'free_cup')
        with pytest.raises(ValueError, match='y_d'):
            m.inverse(yb, 0.0, 0, 'free_cup')
    for call in (lambda: m.observe(u, y, bad, 0, 'free_cup'), lambda: m.forward(u, bad, 0, 'free_cup'), lambda: m.inverse(y, bad, 0, 'free_cup')):
        with pytest.raises(ValueError, match='site_x'):
            call()
    with pytest.raises(ValueError, match='ball_id'):
        m.observe(u, y, 0.0, 1.5, 'free_cup')
    assert m.n_accepted == 100                    # nothing above changed the state


def test_t_u5_a_degenerate_update_raises_only_value_error_and_keeps_the_state():
    # extreme-but-finite rows: each either returns a bool or raises ValueError; the state stays finite
    m = PlantModel()
    rows = [([0.0, 0.0, 1e200], [0.0, 0.0, 1e200], 0.0), ([0.0, 0.0, 1e-300], [0.0, 0.0, 1e-300], 0.0),
            ([0.0, 0.0, 1.0], [0.0, 0.0, 1.0], 1e200), ([0.0, 0.0, 1.0], [0.0, 0.0, 1.0], 1e153),
            ([1e300, -1e300, 1.0], [1e300, -1e300, 1.0], 0.0), ([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], 0.0),
            ([0.0, 0.0, -1.0], [0.0, 0.0, -1.0], 0.0), ([0.0, 0.0], [0.0, 0.0, 1.0], 0.0),
            ('abc', [0.0, 0.0, 1.0], 0.0), ([0.0, 0.0, 1.0], [0.0, 0.0, 1.0], 1e308 * 10)]
    for u, y, sx in rows:
        try:
            assert m.observe(u, y, sx, 0, 'free_cup') in (True, False)
        except ValueError:
            pass
        assert np.all(np.isfinite(m.raw())) and np.all(np.isfinite(m.sd()))
        assert np.all(np.isfinite(m.forward([0.0, 0.0, 1.0], 0.0, 0, 'free_cup')))
    # the finiteness guard itself: a prior variance that overflows (sd0^2 = inf) makes the gain
    # inf/inf = nan; the update is discarded and a ValueError raised, never a numpy error
    with np.errstate(over='ignore'):
        g = PlantModel(PlantModelConfig(prior_sd_K=1e200))
    before = (g.raw().copy(), g.n_accepted)
    with pytest.raises(ValueError, match='not finite'):
        g.observe([0.0, 0.0, 1.0], [0.0, 0.0, 1.08], 0.0, 0, 'free_cup')
    assert np.array_equal(g.raw(), before[0]) and g.n_accepted == before[1]


def test_the_summary_names_the_parameters_still_at_their_prior():
    m = PlantModel()
    assert 'at prior: K, b_x, b_y, s' in m.summary()
    m2 = _converged(np.random.default_rng(9))
    # free_cup rows only: just the carried kind's lateral law (BUG-138) is left at its prior
    assert 'at prior: b_x@carried, b_y@carried, s@carried' in m2.summary()
    assert '100 accepted / 0 rejected' in m2.summary()


# --------------------------------------------------------------------------- production vs probe
# PlantModel.theta() after the first 400 corpus rows (generated 2026-11-06; recipe at the top).
# Re-pinned 2026-11-27 for BUG-139 (b forgotten by a random walk, the probe's '+pn'): K and the
# rejected rows are unchanged; b and Delta moved by <= 0.26 mm and s by 0.0083 per m (offsets
# model), b by <= 0.35 mm and s by 0.0008 per m (default). Was: OFFSETS [1.079487609514168,
# 0.00701510259549045, 0.00765143243009404, -0.11448831811970676, 0.04233194336535344,
# 0.05126206317651464], DEFAULT [1.079487609514168, 0.01005758462615316, 0.01115381863081433,
# -0.13023172853598408]. Generated by ``/tmp/probe_bug139_pin.py`` (the body of this test, printing).
PIN_THETA_OFFSETS = [1.079487609514168, 0.007032853070803385, 0.007635110614636432,
                     -0.10616568732048332, 0.04258676484666085, 0.05122841883854835]
PIN_THETA_DEFAULT = [1.079487609514168, 0.010403155186884177, 0.01113536814774612,
                     -0.12942244913900142]
PIN_REJECTED = [0, 55, 57, 72, 73, 114, 122, 180, 330, 333, 364, 396]


def test_pin_production_equals_the_probe_prototype_on_400_corpus_rows(monkeypatch):
    """The settings under which the design's rule and the probe prototype coincide: the probe's
    'mband+struct+pn' rule (model-relative band; s and Delta unforgotten; b forgotten by the
    random walk q = (1 - lambda)^2 R / lambda, BUG-139), lambda 0.99 on K, unbounded P,
    with the probe's forgetting on REJECTED rows disabled (the design forgets per accepted row
    only), and ``ball_offsets=True`` (the probe always fits Delta). The design's apex band
    max(0.09 K, 3 sd_K) equals the probe's fixed 9 % once K's sd is under 0.03 K, i.e. from the
    first accepted row on; the accept masks are asserted equal, so no row falls between them."""
    pytest.importorskip('pandas')   # the probe's loader
    for p in (os.path.join(REPO, 'tools', 'probes'),):
        if p not in sys.path:
            monkeypatch.syspath_prepend(p)
    import learner_replay as probe
    d = probe.mr.load(CORPUS).iloc[:400].reset_index(drop=True)
    d['ball1'] = (d.ball_id == 1).astype(float)
    d['inlier'] = probe.mr.inlier_mask(d).values
    monkeypatch.setattr(probe.Filt, 'forget_only', lambda self: None)
    hist, _sd, raw, acc, _nz = probe.run_rls(d, 0.99, False, 'mband+struct+pn')
    U = d[['u0', 'u1', 'u2_apex_m']].values
    Y = d[['y0', 'y1', 'y2_apex_m']].values
    models = (PlantModel(PlantModelConfig(ball_offsets=True)), PlantModel())
    accs = []
    for m in models:
        accs.append(np.array([m.observe(U[i], Y[i], float(d.x0.values[i]), int(d.ball_id.values[i]), 'free_cup')
                              for i in range(len(d))]))
    assert np.array_equal(accs[0], acc)
    assert np.flatnonzero(~acc).tolist() == PIN_REJECTED
    # probe hist columns: K, b_x, s, D1x, b_y, D1y, s_y
    h = hist[-1]
    probe_theta = np.array([h[0], h[1], h[4], h[2], h[3], h[5]])
    r = raw[-1]
    probe_raw = np.array([r[0], r[1], r[4], r[2], r[3], r[5]])
    # the corpus is v1 (free_cup rows only): the pin is the free_cup slice of param_names, and
    # the carried lateral law (BUG-138) is untouched, at its prior
    n6, n4 = len(PIN_THETA_OFFSETS), len(PIN_THETA_DEFAULT)
    np.testing.assert_allclose(models[0].theta()[:n6], probe_theta, rtol=0, atol=1e-9)
    np.testing.assert_allclose(models[0].raw()[:n6], probe_raw, rtol=0, atol=1e-9)
    np.testing.assert_allclose(models[0].theta()[:n6], PIN_THETA_OFFSETS, rtol=0, atol=1e-9)
    np.testing.assert_allclose(models[1].theta()[:n4], PIN_THETA_DEFAULT, rtol=0, atol=1e-9)
    for m in models:
        carried = [i for i, n in enumerate(m.param_names) if n.endswith('@carried')]
        assert len(carried) == (len(m.param_names) - 1) // 2
        assert np.all(m.raw()[carried] == 0.0)
        assert set(m.param_names[i] for i in carried) <= set(m.at_prior())
    assert np.array_equal(accs[1], acc)


# --------------------------------------------------------------------------- BUG-138: lateral law per release kind
#: The measured oval (carried) lateral fit on its own rows (2026-11-24 review): b_x +179 mm,
#: s +1.23 per m, flown at ONE Site (x = -0.075), so only the sum b_x + s x = +86.75 mm there is
#: meaningful. Its b_y and K are chosen to differ from Columns so every shared/split part shows.
OVAL = dict(K=1.095, b_x=0.179, s=1.23, b_y=0.030, site_x=-0.075)
COLUMNS = (1.08, 0.004, 0.012, -0.09)                # free_cup plant (K, b_x, b_y, s)
COLUMNS_SITES = (-0.0625, 0.0625, -0.125, 0.125)


def _columns_model(seed=21, n=400):
    """A Columns (free_cup) model: rows alternating over the Columns Sites."""
    rng = np.random.default_rng(seed)
    m = PlantModel()
    for i in range(n):
        sx = COLUMNS_SITES[i % len(COLUMNS_SITES)]
        u = _cmd(rng)
        assert m.observe(u, _plant(COLUMNS, u, sx, 0) + rng.normal(0, [0.02, 0.02, 0.01]), sx, 0,
                         release_kind='free_cup')
    return m


def _oval_rows(m, n, seed=22):
    rng = np.random.default_rng(seed)
    sx = OVAL['site_x']
    acc = 0
    for _ in range(n):
        u = _cmd(rng)
        y = np.array([u[0] + OVAL['b_x'] + OVAL['s'] * sx, u[1] + OVAL['b_y'], OVAL['K'] * u[2]])
        acc += m.observe(u, y + rng.normal(0, [0.02, 0.02, 0.01]), sx, 0, release_kind='carried')
    return acc


def test_bug138_carried_rows_never_move_the_columns_lateral_offset_and_k_is_shared():
    """BUG-138 (option 3, TKT-93): 600 oval rows (the measured carried law) added to a Columns
    Memory. The free_cup lateral law is a separate filter that a carried row never updates, so the
    free_cup applied offsets (axis 0 at every Columns Site, axis 1) and the lateral part of the
    free_cup command are BIT-IDENTICAL: tolerance 0, because any nonzero change can only be
    leakage (pooled, the same rows moved the +62.5 mm Site's offset by ~8 mm on the live Memory and
    by more here). K is shared by design: it moves toward the carried rows' gain."""
    m = _columns_model()
    sites = COLUMNS_SITES + (0.0, 0.03)
    before = [m.offset_x(sx, 'free_cup') for sx in sites]
    y_d = np.array([0.01, -0.02, 1.0])
    cmd_before = [m.inverse(y_d, sx, 0, 'free_cup') for sx in sites]
    K_before = m.theta()[0]
    assert K_before == pytest.approx(COLUMNS[0], abs=0.006)
    assert all(not p for _, p in before)                       # Columns is fitted: applied
    assert _oval_rows(m, 600) > 500                            # carried rows are accepted
    after = [m.offset_x(sx, 'free_cup') for sx in sites]
    assert after == before
    for sx, c0 in zip(sites, cmd_before):
        assert np.array_equal(m.inverse(y_d, sx, 0, 'free_cup')[:2], c0[:2])
        assert m.inverse(y_d, sx, 0, 'free_cup')[:2].tolist() == c0[:2].tolist()   # the default kind
    names = m.param_names
    th = m.theta()
    for n in ('b_x', 'b_y', 's'):
        assert th[names.index(n)] == pytest.approx(
            {'b_x': COLUMNS[1], 'b_y': COLUMNS[2], 's': COLUMNS[3]}[n], abs=0.03)
    # K is shared: the 600 carried rows pull it to their gain (lambda 0.99: ~100-row memory)
    assert m.theta()[0] == pytest.approx(OVAL['K'], abs=0.004)
    assert m.theta()[0] > K_before + 0.008


def test_bug138_the_carried_kind_learns_its_own_lateral_law():
    """The carried rows fit the carried lateral pair: the applied carried offset at the oval's
    Site is its own law's (+86.75 mm), b_y@carried is its own +30 mm, and (one Site only, the
    BUG-137 guard per kind) b_x@carried and s@carried stay at prior and a far Site gets the
    identity for the carried kind."""
    m = _columns_model()
    _oval_rows(m, 600)
    ox, p = m.offset_x(OVAL['site_x'], 'carried')
    assert not p
    assert ox == pytest.approx(OVAL['b_x'] + OVAL['s'] * OVAL['site_x'], abs=0.012)
    names, th = m.param_names, m.theta()
    assert th[names.index('b_y@carried')] == pytest.approx(OVAL['b_y'], abs=0.008)
    assert {'b_x@carried', 's@carried'} <= set(m.at_prior())
    assert not {'b_x', 'b_y', 's'} & set(m.at_prior())
    assert m.offset_x(0.0625, 'carried') == (0.0, True)
    u = np.array([0.003, -0.002, 1.0])
    y = m.forward(u, OVAL['site_x'], 0, 'carried')
    assert y[0] == pytest.approx(u[0] + ox, abs=1e-15)
    assert y[1] == pytest.approx(u[1] + th[names.index('b_y@carried')], abs=1e-15)
    assert y[2] == pytest.approx(m.theta()[0] * u[2], abs=1e-15)
    assert 'carried: b ' in m.summary(OVAL['site_x']) and '600 carried rows' in m.summary()


def test_bug138_a_carried_law_with_two_sites_recovers_its_own_slope():
    """Two carried Sites identify the carried b_x and s separately from the free_cup ones."""
    m = _columns_model()
    rng = np.random.default_rng(23)
    law = (1.08, 0.030, -0.005, 0.20)
    for i in range(400):
        sx = (-0.075, 0.075)[i % 2]
        u = _cmd(rng)
        assert m.observe(u, _plant(law, u, sx, 0) + rng.normal(0, [0.02, 0.02, 0.01]), sx, 0,
                         release_kind='carried')
    names, th = m.param_names, m.theta()
    assert th[names.index('b_x@carried')] == pytest.approx(law[1], abs=0.008)
    assert th[names.index('s@carried')] == pytest.approx(law[3], abs=0.06)
    assert th[names.index('b_y@carried')] == pytest.approx(law[2], abs=0.008)
    assert th[names.index('s')] == pytest.approx(COLUMNS[3], abs=0.03)


def test_bug138_an_empty_kind_is_the_identity_laterally_whatever_the_other_kind_learned():
    """Identity at an empty model for BOTH kinds; a model with only free_cup rows applies the
    identity laterally to a carried query (at prior) and K to its apex."""
    for offsets in (False, True):
        m = PlantModel(PlantModelConfig(ball_offsets=offsets))
        y = np.array([0.0123, -0.0456, 0.987654321])
        for kind in ('free_cup', 'carried'):
            for sx, ball in ((0.0, 0), (-0.075, 1), (0.0625, 0)):
                assert np.array_equal(m.inverse(y, sx, ball, kind), y)
                assert np.array_equal(m.forward(y, sx, ball, kind), y)
            assert m.offset_x(0.0625, kind) == (0.0, True)
        assert set(m.at_prior()) == set(m.param_names)
    m = _columns_model()
    K = m.theta()[0]
    y = np.array([0.0123, -0.0456, 0.9])
    for sx in COLUMNS_SITES + (-0.075,):
        u = m.inverse(y, sx, 0, 'carried')
        assert u[0] == y[0] and u[1] == y[1] and u[2] == pytest.approx(y[2] / K, rel=1e-15)
        f = m.forward(y, sx, 0, 'carried')
        assert f[0] == y[0] and f[1] == y[1]
        assert m.offset_x(sx, 'carried') == (0.0, True)
        np.testing.assert_array_equal(
            m.forward_rows(np.tile(y, (2, 1)), np.array([sx, sx]), 0, 'carried')[:, :2],
            np.tile(y[:2], (2, 1)))
    assert set(m.at_prior()) == {'b_x@carried', 'b_y@carried', 's@carried'}


def test_bug138_release_kind_is_validated():
    m = PlantModel()
    for bad in ('oval', None, 0):
        for call in (lambda: m.observe([0.0, 0.0, 1.0], [0.0, 0.0, 1.0], 0.0, 0, bad),
                     lambda: m.forward([0.0, 0.0, 1.0], 0.0, 0, bad),
                     lambda: m.inverse([0.0, 0.0, 1.0], 0.0, 0, bad),
                     lambda: m.offset_x(0.0, bad),
                     lambda: m.forward_rows(np.zeros((1, 3)), np.zeros(1), 0, bad)):
            with pytest.raises(ValueError, match='release_kind'):
                call()
    assert m.n_accepted == 0 and m.n_rejected == 0


def test_bug138_release_kind_is_required_on_every_map_and_update():
    """BUG-138 audit finding (2026-11-27): ``release_kind`` defaulted to 'free_cup', so a caller
    that forgot the kind silently read and wrote the Columns law. It is now REQUIRED on the write
    path (``observe``) and the command path (``inverse``, ``forward``, ``forward_rows``); omitting
    it is a TypeError and changes nothing. Only the read-out ``offset_x`` keeps a default."""
    m = _converged(np.random.default_rng(3))
    before = (m.raw().copy(), m.n_accepted, m.n_rejected)
    u = [0.0, 0.0, 1.0]
    for call in (lambda: m.observe(u, [0.0, 0.0, 1.08], 0.0, 0),
                 lambda: m.forward(u, 0.0, 0),
                 lambda: m.inverse(u, 0.0, 0),
                 lambda: m.forward_rows(np.zeros((1, 3)), np.zeros(1), 0)):
        with pytest.raises(TypeError, match='release_kind'):
            call()
    assert np.array_equal(m.raw(), before[0]) and (m.n_accepted, m.n_rejected) == before[1:]
    assert m.offset_x(0.0625) == m.offset_x(0.0625, 'free_cup')


def _cold_carried(offset_m, seed, n=300, site_x=-0.075):
    """A warm free_cup kind (300 Columns rows), then ``n`` carried rows at ONE Site whose axis-0
    landing is the command plus a CONSISTENT ``offset_m`` (20 mm shot noise)."""
    rng = np.random.default_rng(seed)
    m = PlantModel()
    for i in range(300):
        u = _cmd(rng)
        m.observe(u, np.array([u[0] + 0.005, u[1] - 0.005, 1.08 * u[2]])
                  + rng.normal(0, [0.020, 0.020, 0.010]), (-0.0625, 0.0625)[i % 2], 0, 'free_cup')
    acc = []
    for _ in range(n):
        u = _cmd(rng)
        acc.append(m.observe(u, np.array([u[0] + offset_m, u[1] + 0.004, 1.08 * u[2]])
                             + rng.normal(0, [0.020, 0.020, 0.010]), site_x, 0, 'carried'))
    return m, np.array(acc)


def test_bug138_a_cold_carried_kind_learns_an_offset_up_to_120_mm():
    """The lateral inlier gate (|innovation| < ``inlier_lateral_m`` = 100 mm against the kind's own
    law) is measured against the PRIOR (offset 0) while a kind has no rows. Measured 2026-11-27
    (5 seeds): a consistent offset of up to ~120 mm is still learned, because the first row whose
    noise brings it under 100 mm is accepted at a gain of ~0.6 on b and pulls the law within the
    gate; at 120 mm 294-300 of 300 rows are accepted and the applied offset ends within 2 mm."""
    for seed in range(5):
        m, acc = _cold_carried(0.120, seed)
        assert acc.sum() >= 290
        off, at_prior = m.offset_x(-0.075, 'carried')
        assert not at_prior and off == pytest.approx(0.120, abs=0.006)
        assert m.offset_x(0.0625, 'free_cup')[0] == pytest.approx(0.005, abs=0.012)


def test_bug138_a_cold_carried_kind_learns_the_ovals_measured_179_mm_offset():
    """The oval's own measured lateral law (2026-11-24 review: b_x +179 mm) as a consistent offset
    at a cold carried kind. Before BUG-153 (measured 2026-11-27, 5 seeds): 0 of 300 rows accepted,
    the kind stayed at its prior (identity laterally), and since only ACCEPTED rows enter the
    Learner's residual memory, nothing corrected the Throw (was ``xfail(strict=True)``). Fixed by
    run-length recovery (BUG-153, 2026-11-29): the 5th consecutive agreeing lateral rejection is
    accepted, which pulls the law within the gate, and the rest pass the ordinary gate."""
    for seed in range(5):
        m, acc = _cold_carried(0.179, seed)
        off, at_prior = m.offset_x(-0.075, 'carried')
        assert not at_prior and off == pytest.approx(0.179, abs=0.010)
        assert not acc[:4].any() and acc[4] and acc.sum() >= 290
        assert m.n_recovered == 1 and m._lat['carried'].n_recovered == 1


# --------------------------------------------------------------------------- BUG-153: run-length recovery
CARRIED_SITE = -0.075


def _warm_free_cup(seed=31):
    """A warm free_cup kind (300 Columns rows, K 1.08, offset +5/-5 mm) and a cold carried kind."""
    m, _acc = _cold_carried(0.0, seed, n=0)
    return m


def _carried(m, off_x, apex_ratio=1.08, off_y=0.004, kind='carried', site_x=CARRIED_SITE):
    """Observe one noise-free row of ``kind`` landing ``off_x`` / ``off_y`` [m] from its command."""
    u = np.array([0.01, -0.02, 1.0])
    return m.observe(u, np.array([u[0] + off_x, u[1] + off_y, apex_ratio * u[2]]), site_x, 0, kind)


def test_bug153_a_warm_kind_fed_one_wild_row_is_unchanged():
    """A warm kind: one 300 mm wild row is rejected and the normal rows after it give BIT-IDENTICAL
    parameters to the same rows without the wild one (the next accepted row resets the run, so a
    lone outlier never contributes). Five consecutive wild rows that disagree with each other
    (alternating +-300 mm) never recover either."""
    rng = np.random.default_rng(2)
    a = _converged(rng)
    b = copy.deepcopy(a)
    assert _carried(a, 0.0, off_y=0.300, kind='free_cup', site_x=0.075) is False
    assert len(a._lat['free_cup'].run) == 1
    for i in range(30):
        sx = (-0.075, 0.075)[i % 2]
        u = _cmd(rng)
        y = _plant((1.08, 0.01, 0.012, -0.09), u, sx, 0) + rng.normal(0, [0.02, 0.02, 0.01])
        assert a.observe(u, y, sx, 0, 'free_cup') == b.observe(u, y, sx, 0, 'free_cup')
    assert np.array_equal(a.raw(), b.raw()) and np.array_equal(a.theta(), b.theta())
    assert a.n_recovered == 0 and a._lat['free_cup'].run == []
    th = a.raw().copy()
    for i in range(8):
        assert _carried(a, 0.0, off_y=(0.300, -0.300)[i % 2], kind='free_cup', site_x=0.075) is False
    assert a.n_recovered == 0 and np.array_equal(a.raw(), th)


def test_bug153_four_agreeing_rejections_then_a_wild_one_reset_the_run():
    """4 rows at the oval's +179 mm (rejected, agreeing), then a wild row (+450 mm, > 100 mm from
    the run's mean): no recovery; the run restarts, so it takes 5 MORE agreeing rows, and the 5th
    of them is the one accepted. An apex-rejected row resets the run the same way."""
    for wild in (dict(off_x=0.450), dict(off_x=0.179, apex_ratio=1.40)):
        m = _warm_free_cup()
        for _ in range(4):
            assert _carried(m, 0.179) is False
        assert len(m._lat['carried'].run) == 4
        assert _carried(m, **wild) is False
        assert m.n_recovered == 0
        for _ in range(4):
            assert _carried(m, 0.179) is False
        assert _carried(m, 0.179) is True
        assert m.n_recovered == 1


def test_bug153_five_agreeing_rejections_recover_with_the_ordinary_update_of_the_fifth_row():
    """5 rows at +179 mm on a cold carried kind: rows 1-4 rejected (counted), row 5 accepted by the
    ORDINARY update (theta, P and the unforgotten P of K and the carried lateral filters exactly as
    ``_Filter.proposed`` gives for that row alone; the free_cup law untouched), and the next rows
    pass the ordinary gate."""
    m = _warm_free_cup()
    n_rej = m.n_rejected
    for _ in range(4):
        assert _carried(m, 0.179) is False
    assert m.n_rejected == n_rej + 4 and 'axis 0' in m.last_rejection
    before = copy.deepcopy(m)
    u = np.array([0.01, -0.02, 1.0])
    y = np.array([u[0] + 0.179, u[1] + 0.004, 1.08])
    px, py = before._phis(CARRIED_SITE, 0)
    lat = before._lat['carried']
    want = [f.proposed(p, z) for f, p, z in ((before._fa, np.array([1.0]), y[2]),
                                             (lat.fx, px, y[0] - u[0]), (lat.fy, py, y[1] - u[1]))]
    assert m.observe(u, y, CARRIED_SITE, 0, 'carried') is True
    got = (m._fa, m._lat['carried'].fx, m._lat['carried'].fy)
    for f, (th, P, Pu) in zip(got, want):
        assert np.array_equal(f.th, th) and np.array_equal(f.P, P) and np.array_equal(f.P_unforgotten, Pu)
    assert m.n_recovered == 1 and m.n_rejected == n_rej + 4 and m._lat['carried'].run == []
    assert m._lat['carried'].n_accepted == 1
    for f0, f1 in ((before._lat['free_cup'].fx, m._lat['free_cup'].fx),
                   (before._lat['free_cup'].fy, m._lat['free_cup'].fy)):
        assert np.array_equal(f0.th, f1.th) and np.array_equal(f0.P, f1.P)
    assert all(_carried(m, 0.179) for _ in range(20))
    assert m.n_recovered == 1
    assert m.offset_x(CARRIED_SITE, 'carried')[0] == pytest.approx(0.179, abs=0.010)
    assert '(1 by run-length recovery)' in m.summary()


def test_bug153_the_run_is_per_release_kind():
    """Rows of the other kind neither extend nor reset a kind's run: 4 carried rejections with
    free_cup rows between them (accepted ones, and a free_cup rejection with the SAME innovation)
    leave the carried run at 4; the 5th CARRIED rejection recovers; the free_cup rejection was a
    run of one for free_cup, reset by its next accepted row, and recovered nothing."""
    m = _warm_free_cup()
    for i in range(4):
        assert _carried(m, 0.179) is False
        assert _carried(m, 0.005, off_y=-0.005, kind='free_cup', site_x=0.0625) is True
        if i == 1:
            assert _carried(m, 0.179, kind='free_cup', site_x=CARRIED_SITE) is False
            assert len(m._lat['free_cup'].run) == 1
    assert len(m._lat['carried'].run) == 4 and m._lat['free_cup'].run == []
    assert m.n_recovered == 0
    assert _carried(m, 0.179) is True
    assert m._lat['carried'].n_recovered == 1 and m._lat['free_cup'].n_recovered == 0
