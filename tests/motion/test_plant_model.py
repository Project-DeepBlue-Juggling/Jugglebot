"""f0, the Learner's global plant model (``motion/skills/plant_model.py``; plan
``plans/active/learner-generalisation.md`` § 2.2 and § 5 T-U1, T-U2, T-U5, T-R2).

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
            m.observe(u, _plant(th, u, sx, ball, ndx, ndy) + rng.normal(0, 0.002, 3), sx, ball)
        assert m.at_prior() == []
        for _ in range(10):
            y = np.array([rng.uniform(-0.1, 0.1), rng.uniform(-0.1, 0.1), rng.uniform(0.5, 1.5)])
            sx, ball = rng.uniform(-0.3, 0.3), int(rng.integers(0, 2))
            np.testing.assert_allclose(m.forward(m.inverse(y, sx, ball), sx, ball), y, rtol=0, atol=1e-12)
            u = _cmd(rng)
            np.testing.assert_allclose(m.inverse(m.forward(u, sx, ball), sx, ball), u, rtol=0, atol=1e-12)
        if offsets:
            # Delta of ball 0 is zero: ball 0's map carries b and s only
            th_r = m.theta()
            u = _cmd(rng)
            y0 = m.forward(u, 0.05, 0)
            np.testing.assert_allclose(y0, [u[0] + th_r[1] + th_r[3] * 0.05, u[1] + th_r[2], th_r[0] * u[2]],
                                       rtol=0, atol=1e-15)
            assert m.forward(u, 0.05, 1)[0] - y0[0] == pytest.approx(th_r[4], abs=1e-15)


def test_t_r2_an_empty_model_is_exactly_the_identity():
    for offsets in (False, True):
        m = PlantModel(PlantModelConfig(ball_offsets=offsets))
        y = np.array([0.0123, -0.0456, 0.987654321])
        for sx, ball in ((0.0, 0), (-0.075, 1), (0.125, 0)):
            assert np.array_equal(m.inverse(y, sx, ball), y)
            assert np.array_equal(m.forward(y, sx, ball), y)
        th = m.theta()
        assert th[0] == 1.0 and np.all(th[1:] == 0.0)
        assert set(m.at_prior()) == set(m.param_names)
        for sx in (-0.3, -0.075, 0.0, 0.125, 0.3):
            assert m.offset_x(sx) == (0.0, True)
            assert np.array_equal(m.forward_rows(np.tile(y, (2, 1)), np.array([sx, sx]), 0), np.tile(y, (2, 1)))


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
        assert m.observe(u, y, sx, ball)
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
        m.observe(u, _plant((K, 0.01, 0.0, -0.09), u, sx, 0) + rng.normal(0, [0.02, 0.02, 0.01]), sx, 0)
    # lambda 0.99 lags a ramp by ~ slope x 1/(1 - lambda) = 2.7e-5 x 100 = 0.0027
    assert m.theta()[0] == pytest.approx(1.10, abs=0.006)


def _two_site_then_one_site(model):
    rng = np.random.default_rng(11)
    th = (1.08, 0.010, 0.012, -0.09)
    for i in range(200):                            # s is identified: two Sites
        sx = (-0.125, 0.125)[i % 2]
        u = _cmd(rng)
        model.observe(u, _plant(th, u, sx, 0) + rng.normal(0, [0.02, 0.02, 0.01]), sx, 0)
    s_before, sd_before = model.theta()[3], model.sd()[3]
    for i in range(2000):                           # then a long single-Site stretch (the oval)
        u = _cmd(rng)
        model.observe(u, _plant(th, u, -0.075, 0) + rng.normal(0, [0.02, 0.02, 0.01]), -0.075, 0)
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
        m.observe(u, _plant(th, u, -0.075, 0) + rng.normal(0, [0.002, 0.002, 0.002]), -0.075, 0)
    names = m.param_names
    # at ONE off-centre Site b_x and s are confounded (only b_x + s x is seen), so BOTH stay at
    # their prior: b_x's marginal sd keeps |x| sd_s = 22.5 mm > 15 mm. Axis 0 is then L2's.
    assert set(m.at_prior()) == {'b_x', 's', 'delta1_x', 'delta1_y'}
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
        m.observe(u, y, site_x, 0)
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
        assert m.forward(u, -0.075, 0)[0] == pytest.approx(ox, abs=1e-15)
        assert m.inverse(np.array([0.0, 0.0, 1.0]), -0.075, 0)[0] == pytest.approx(-ox, abs=1e-15)
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
    assert m.forward(u, 0.125, 0)[0] == u[0]
    assert m.offset_x(0.0)[1] is True                       # even at x = 0 the split matters
    # a second Site with the true line b_x 0.018, s -0.09: offset +0.075 -> 0.01125
    rng = np.random.default_rng(2)
    for _ in range(60):
        u2 = _cmd(rng)
        y = np.array([u2[0] + 0.018 - 0.09 * 0.075, u2[1] + 0.012, 1.08 * u2[2]]) + rng.normal(0, [0.020, 0.020, 0.010])
        m.observe(u2, y, 0.075, 0)
    ox, at_prior = m.offset_x(0.125)
    assert not at_prior
    raw = m.raw()
    assert ox == pytest.approx(raw[1] + raw[3] * 0.125, abs=1e-15)   # the fitted line
    assert ox == pytest.approx(0.018 - 0.09 * 0.125, abs=0.012)


# --------------------------------------------------------------------------- the inlier rule
def _converged(rng, K=1.08):
    m = PlantModel()
    for i in range(100):
        sx = (-0.075, 0.075)[i % 2]
        u = _cmd(rng)
        m.observe(u, _plant((K, 0.01, 0.012, -0.09), u, sx, 0) + rng.normal(0, [0.02, 0.02, 0.01]), sx, 0)
    return m


def test_a_300_mm_lateral_outlier_is_rejected_and_counted():
    rng = np.random.default_rng(2)
    m = _converged(rng)
    th, n_acc, n_rej = m.theta().copy(), m.n_accepted, m.n_rejected
    u = np.array([0.0, 0.0, 1.0])
    y = m.forward(u, 0.075, 0) + np.array([0.0, 0.300, 0.0])
    assert m.observe(u, y, 0.075, 0) is False
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
        m.observe(u, _plant((1.23, 0.0, 0.0, 0.0), u, 0.0, 0) + rng.normal(0, [0.005, 0.005, 0.005]), 0.0, 0)
    assert m.n_rejected == 0
    assert m.theta()[0] == pytest.approx(1.23, abs=0.005)


def test_a_single_wild_apex_row_after_convergence_is_rejected():
    rng = np.random.default_rng(6)
    m = _converged(rng)
    th = m.theta().copy()
    u = np.array([0.0, 0.0, 1.0])
    y = m.forward(u, 0.075, 0)
    y[2] = 1.25 * u[2]                    # 16 % above K = 1.08 (converged band 9 %)
    assert m.observe(u, y, 0.075, 0) is False
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
            m.observe(ub, y, 0.0, 0)
        with pytest.raises(ValueError, match=r'\bu\b'):
            m.forward(ub, 0.0, 0)
        yb = y.copy(); yb[k] = bad
        with pytest.raises(ValueError, match=r'\by\b'):
            m.observe(u, yb, 0.0, 0)
        with pytest.raises(ValueError, match='y_d'):
            m.inverse(yb, 0.0, 0)
    for call in (lambda: m.observe(u, y, bad, 0), lambda: m.forward(u, bad, 0), lambda: m.inverse(y, bad, 0)):
        with pytest.raises(ValueError, match='site_x'):
            call()
    with pytest.raises(ValueError, match='ball_id'):
        m.observe(u, y, 0.0, 1.5)
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
            assert m.observe(u, y, sx, 0) in (True, False)
        except ValueError:
            pass
        assert np.all(np.isfinite(m.raw())) and np.all(np.isfinite(m.sd()))
        assert np.all(np.isfinite(m.forward([0.0, 0.0, 1.0], 0.0, 0)))
    # the finiteness guard itself: a prior variance that overflows (sd0^2 = inf) makes the gain
    # inf/inf = nan; the update is discarded and a ValueError raised, never a numpy error
    with np.errstate(over='ignore'):
        g = PlantModel(PlantModelConfig(prior_sd_K=1e200))
    before = (g.raw().copy(), g.n_accepted)
    with pytest.raises(ValueError, match='not finite'):
        g.observe([0.0, 0.0, 1.0], [0.0, 0.0, 1.08], 0.0, 0)
    assert np.array_equal(g.raw(), before[0]) and g.n_accepted == before[1]


def test_the_summary_names_the_parameters_still_at_their_prior():
    m = PlantModel()
    assert 'at prior: K, b_x, b_y, s' in m.summary()
    m2 = _converged(np.random.default_rng(9))
    assert 'at prior: none' in m2.summary() and '100 accepted / 0 rejected' in m2.summary()


# --------------------------------------------------------------------------- production vs probe
# PlantModel.theta() after the first 400 corpus rows (generated 2026-11-06; recipe at the top).
PIN_THETA_OFFSETS = [1.079487609514168, 0.00701510259549045, 0.00765143243009404,
                     -0.11448831811970676, 0.04233194336535344, 0.05126206317651464]
PIN_THETA_DEFAULT = [1.079487609514168, 0.01005758462615316, 0.01115381863081433,
                     -0.13023172853598408]
PIN_REJECTED = [0, 55, 57, 72, 73, 114, 122, 180, 330, 333, 364, 396]


def test_pin_production_equals_the_probe_prototype_on_400_corpus_rows(monkeypatch):
    """The settings under which the design's rule and the probe prototype coincide: the probe's
    'mband+struct' rule (model-relative band; s and Delta unforgotten), lambda 0.99, unbounded P,
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
    hist, _sd, raw, acc, _nz = probe.run_rls(d, 0.99, False, 'mband+struct')
    U = d[['u0', 'u1', 'u2_apex_m']].values
    Y = d[['y0', 'y1', 'y2_apex_m']].values
    models = (PlantModel(PlantModelConfig(ball_offsets=True)), PlantModel())
    accs = []
    for m in models:
        accs.append(np.array([m.observe(U[i], Y[i], float(d.x0.values[i]), int(d.ball_id.values[i]))
                              for i in range(len(d))]))
    assert np.array_equal(accs[0], acc)
    assert np.flatnonzero(~acc).tolist() == PIN_REJECTED
    # probe hist columns: K, b_x, s, D1x, b_y, D1y, s_y
    h = hist[-1]
    probe_theta = np.array([h[0], h[1], h[4], h[2], h[3], h[5]])
    r = raw[-1]
    probe_raw = np.array([r[0], r[1], r[4], r[2], r[3], r[5]])
    np.testing.assert_allclose(models[0].theta(), probe_theta, rtol=0, atol=1e-9)
    np.testing.assert_allclose(models[0].raw(), probe_raw, rtol=0, atol=1e-9)
    np.testing.assert_allclose(models[0].theta(), PIN_THETA_OFFSETS, rtol=0, atol=1e-9)
    np.testing.assert_allclose(models[1].theta(), PIN_THETA_DEFAULT, rtol=0, atol=1e-9)
    assert np.array_equal(accs[1], acc)
