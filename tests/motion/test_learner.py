"""The Learner (``motion/skills/learner.py``; plan ``plans/active/learner-generalisation.md``
§§ 2.3-2.4 and § 5 T-U3, T-U4, T-U5, T-R2): residual memory L2 and the evidence cap L3 over the
global plant model f0 (``plant_model.py``, tested in ``test_plant_model.py``).

The old local-linear law's tests went with the old law (owner decision D1, deleted 2026-11-06;
the law survives frozen in ``tools/probes/learner_replay_old_law.py``). The ``Experience`` /
``Memory`` I/O tests live in ``test_memory.py`` (schema v2). Real arithmetic, no model mocks; an
Experience is a duck-typed namespace carrying the v2 field names.
"""
from __future__ import annotations

import dataclasses
import hashlib
import math
import types

import numpy as np
import pytest

from jugglebot.motion.skills.learner import Learner, LearnerConfig, ThrowContext
from jugglebot.motion.skills.plant_model import PlantModelConfig


def _ctx(site_x=0.0, rel=None, vel=(0.0, 0.0, 4.4), ball=0, kind='free_cup', dwell=0.4):
    rel = (site_x, 0.0, 0.4) if rel is None else rel
    return ThrowContext(site_xy_m=(site_x, 0.0), release_m=rel, release_vel_mps=vel, dwell_s=dwell,
                        ball_id=ball, release_kind=kind)


def _exp(ctx, u, y):
    return types.SimpleNamespace(site_xy_m=ctx.site_xy_m, release_m=ctx.release_m,
                                 release_vel_mps=ctx.release_vel_mps, dwell_s=ctx.dwell_s,
                                 seat_offset_m=ctx.seat_offset_m, u=np.asarray(u, dtype=float),
                                 y=np.asarray(y, dtype=float), t_abs_s=0.0, ball_id=ctx.ball_id,
                                 release_kind=ctx.release_kind, caught=True, migrated=False)


def _plant(u, site_x, extra=(0.0, 0.0, 0.0)):
    """True plant: K 1.08, b (10, 12) mm, s -0.09 per m, plus a context residual ``extra``."""
    return np.array([u[0] + 0.010 - 0.09 * site_x + extra[0], u[1] + 0.012 + extra[1],
                     1.08 * u[2] + extra[2]])


def _trained(n=60, extra=(0.008, -0.006, 0.005), ctx=None, seed=0):
    """A Learner whose f0 has converged on two Sites (plain rows) and whose memory holds ``n``
    rows at ``ctx`` (default: a carried Throw) that carry a context residual ``extra``."""
    rng = np.random.default_rng(seed)
    L = Learner()
    for i in range(80):
        sx = (-0.125, 0.125)[i % 2]
        c = _ctx(sx)
        u = np.array([0.0, 0.0, rng.uniform(0.8, 1.2)])
        L.observe(_exp(c, u, _plant(u, sx) + rng.normal(0, 0.002, 3)))
    ctx = _ctx(-0.075, rel=(-0.06, 0.0, 0.4), vel=(0.1, 0.0, 4.4), kind='carried') if ctx is None else ctx
    for i in range(n):
        u = np.array([0.0, 0.0, 0.9])
        L.observe(_exp(ctx, u, _plant(u, ctx.site_xy_m[0], extra) + rng.normal(0, 0.002, 3)))
    return L, ctx


Y_D = np.array([0.0, 0.0, 0.9])


# --------------------------------------------------------------------------- T-R2
def test_t_r2_an_empty_learner_returns_exactly_y_d_and_predicts_exactly_u():
    L = Learner()
    for ctx in (_ctx(), _ctx(-0.075, kind='carried', ball=1), _ctx(0.125, dwell=math.nan)):
        y_d = np.array([0.0123, -0.0045, 0.912345])
        assert np.array_equal(L.command(ctx, y_d), y_d)
        assert np.array_equal(L.model_command(ctx, y_d), y_d)
        u = np.array([0.003, -0.002, 0.87])
        assert np.array_equal(L.predict(ctx, u), u)
        assert np.array_equal(L.predict(ctx, u, y_d), u)
        assert L.evidence(ctx) == (0.0, 0)
        u_bar, D, d = L.local_model(ctx, y_d)
        assert np.array_equal(u_bar, y_d) and np.array_equal(D, np.eye(3)) and np.array_equal(d, y_d)


def test_the_learner_advertises_the_context_api():
    assert Learner.takes_context is True and Learner().takes_context is True


# --------------------------------------------------------------------------- T-U3
def test_t_u3_alpha_is_monotone_in_the_evidence_and_zero_at_empty_support():
    L, ctx = _trained(n=0)
    assert L.evidence(ctx) == (0.0, 0)
    assert np.array_equal(L.command(ctx, Y_D), L.model_command(ctx, Y_D))
    rng = np.random.default_rng(1)
    prev_sw, prev_step = 0.0, 0.0
    cfg = L.config
    for n in range(1, cfg.k + 5):
        u = np.array([0.0, 0.0, 0.9])
        L.observe(_exp(ctx, u, _plant(u, ctx.site_xy_m[0], (0.010, 0.0, 0.0)) + rng.normal(0, 1e-4, 3)))
        sw, used = L.evidence(ctx)
        assert used == min(n, cfg.k)
        assert sw >= prev_sw - 1e-12
        alpha = sw / (sw + cfg.w0)
        step = abs(L.command(ctx, Y_D)[0] - L.model_command(ctx, Y_D)[0])
        assert step >= prev_step - 2e-4          # monotone up to the 0.1 mm row noise
        assert 0.0 < alpha < 1.0
        prev_sw, prev_step = sw, step
    assert prev_sw == pytest.approx(cfg.k, rel=1e-9)   # k rows exactly at the query: w = 1 each


@pytest.mark.parametrize('other', [dict(ball=1), dict(kind='free_cup')])
def test_t_u3_rows_of_another_ball_or_release_kind_contribute_nothing(other):
    L, ctx = _trained(n=0)
    base = L.command(ctx, Y_D).copy()
    kw = dict(site_x=ctx.site_xy_m[0], rel=ctx.release_m, vel=ctx.release_vel_mps, ball=ctx.ball_id,
              kind=ctx.release_kind)
    kw.update(other)
    c_other = _ctx(**kw)
    # rows identical in physics, far off in landing (but inside the plant model's inlier band)
    for _ in range(20):
        u = np.array([0.0, 0.0, 0.9])
        L.observe(_exp(c_other, u, _plant(u, ctx.site_xy_m[0], (0.050, -0.050, 0.02))))
    assert L.evidence(ctx) == (0.0, 0)
    np.testing.assert_allclose(L.command(ctx, Y_D), L.model_command(ctx, Y_D), rtol=0, atol=0)
    assert L.evidence(c_other)[1] == L.config.k
    del base


def test_t_u3_rows_beyond_the_support_gate_contribute_nothing():
    """Rows just outside the gate (d^2 = (0.155 / 0.05)^2 = 9.61 > 9), interleaved with the
    query's rows and carrying a 50 mm residual, do not enter: the Learner's correction equals
    alpha r_hat computed by hand over the k most recent rows AT the query only."""
    def build(with_far):
        L = Learner()
        rng = np.random.default_rng(3)
        q = _ctx(0.0, kind='carried', vel=(0.1, 0.0, 4.4))
        far = _ctx(0.0, rel=(0.155, 0.0, 0.4), kind='carried', vel=(0.1, 0.0, 4.4))
        for i in range(30):
            u = np.array([0.0, 0.0, 0.9])
            L.observe(_exp(q, u, _plant(u, 0.0, (0.004, 0.003, 0.002)) + rng.normal(0, 0.002, 3)))
            if with_far and i % 4 == 0:
                # a far row on the SAME f0 (no extra residual for f0 to learn), wild in L2 terms
                # only if it entered: its landing is offset in a way f0 attributes to noise
                L.observe(_exp(far, u, _plant(u, 0.0, (0.004, 0.003, 0.002)) + np.array([0.05, -0.05, 0.0])))
        return L, q
    La, q = build(False)
    Lb, _ = build(True)
    ra = La.command(q, Y_D) - La.model_command(q, Y_D)
    rb = Lb.command(q, Y_D) - Lb.model_command(q, Y_D)
    # the far rows moved f0's b (they are real rows), so compare the residual layer through the
    # rows it used: same count, and the correction equals alpha r_hat recomputed by hand
    k, w0 = La.config.k, La.config.w0
    assert La.evidence(q)[1] == Lb.evidence(q)[1] == k
    for L, r in ((La, ra), (Lb, rb)):
        n = L._rows.n
        idx = [i for i in range(n) if L._rows.rel[i][0] == 0.0][-k:]
        res = np.array([L._rows.y[i] - L.model.forward(L._rows.u[i], 0.0, 0) for i in idx])
        sw = float(k)
        corr = sw / (sw + w0) * res.mean(0)
        expect = -np.array([corr[0], corr[1], corr[2] / L.model.theta()[0]])
        np.testing.assert_allclose(r, expect, rtol=0, atol=1e-12)


# --------------------------------------------------------------------------- T-U4
def test_t_u4_the_cap_holds_everywhere_about_the_model_command():
    rng = np.random.default_rng(5)
    cfg = LearnerConfig()
    for trial in range(30):
        extra = rng.uniform(-0.08, 0.08, 3)
        extra[2] = rng.uniform(-0.06, 0.06)
        L, ctx = _trained(n=int(rng.integers(1, 12)), extra=tuple(extra), seed=trial)
        for _ in range(5):
            y_d = np.array([rng.uniform(-0.02, 0.02), rng.uniform(-0.02, 0.02), rng.uniform(0.8, 1.2)])
            sw, _n = L.evidence(ctx)
            f = 1.0 - math.exp(-sw / cfg.w0)
            dlt = L.command(ctx, y_d) - L.model_command(ctx, y_d)
            assert abs(dlt[0]) <= cfg.c_max_xy_m * f + 1e-15
            assert abs(dlt[1]) <= cfg.c_max_xy_m * f + 1e-15
            assert abs(dlt[2]) <= cfg.c_max_apex_m * f + 1e-15


def _opposed(cfg, n):
    """A Learner whose f0 cannot absorb the query's residual: rows at the query context (carried)
    land +80/-80/+60 mm off, interleaved with rows at the SAME Site but release_kind free_cup that
    land the opposite way, so f0's b and K average them out and L2 sees the residual."""
    L, ctx = _trained(n=0)
    L = Learner(L.model, cfg)
    other = _ctx(ctx.site_xy_m[0], rel=ctx.release_m, vel=ctx.release_vel_mps, kind='free_cup')
    u = np.array([0.0, 0.0, 0.9])
    for _ in range(n):
        L.observe(_exp(other, u, _plant(u, ctx.site_xy_m[0], (-0.08, 0.08, -0.06))))
        L.observe(_exp(ctx, u, _plant(u, ctx.site_xy_m[0], (0.08, -0.08, 0.06))))
    assert L.model.n_rejected == 0
    return L, ctx


def test_t_u4_a_large_residual_is_held_at_the_cap_and_dense_evidence_reaches_c_max():
    """At the DEFAULT config (k 16, w0 2) full evidence gives the cap factor 1 - e^-8 = 0.9997:
    the cap reaches c_max (to 0.01 mm), in the direction that opposes the residual."""
    cfg = LearnerConfig()
    L, ctx = _opposed(cfg, cfg.k)
    sw = L.evidence(ctx)[0]
    assert sw == pytest.approx(cfg.k, rel=1e-12)
    f = 1.0 - math.exp(-sw / cfg.w0)
    c_max = np.array([cfg.c_max_xy_m, cfg.c_max_xy_m, cfg.c_max_apex_m])
    dlt = L.command(ctx, Y_D) - L.model_command(ctx, Y_D)
    np.testing.assert_allclose(np.abs(dlt), c_max * f, rtol=0, atol=1e-12)
    np.testing.assert_allclose(np.abs(dlt), c_max, rtol=0, atol=1.1e-5)
    assert np.sign(dlt[0]) == -1 and np.sign(dlt[1]) == 1 and np.sign(dlt[2]) == -1


def test_t_u4_zero_evidence_gives_exactly_the_model_command():
    L, ctx = _trained(n=10)
    elsewhere = _ctx(0.6, rel=(0.6, 0.3, 0.4), vel=(0.0, 0.0, 2.0), kind='carried')
    assert L.evidence(elsewhere) == (0.0, 0)
    for y_d in (Y_D, np.array([0.01, -0.02, 1.2])):
        assert np.array_equal(L.command(elsewhere, y_d), L.model_command(elsewhere, y_d))
        assert np.array_equal(L.predict(elsewhere, y_d), L.model.forward(y_d, 0.6, 0))


# --------------------------------------------------------------------------- local_model, read-only
def test_local_model_is_the_affine_map_of_predict():
    rng = np.random.default_rng(9)
    L, ctx = _trained(n=7)
    for _ in range(20):
        y_d = np.array([rng.uniform(-0.03, 0.03), rng.uniform(-0.03, 0.03), rng.uniform(0.7, 1.3)])
        u_bar, D, d = L.local_model(ctx, y_d)
        assert u_bar.shape == (3,) and D.shape == (3, 3) and d.shape == (3,)
        np.testing.assert_array_equal(u_bar, L.model_command(ctx, y_d))
        np.testing.assert_array_equal(D, np.diag([1.0, 1.0, L.model.theta()[0]]))
        for _ in range(5):
            u = u_bar + rng.uniform(-0.05, 0.05, 3)
            np.testing.assert_allclose(d + D @ (u - u_bar), L.predict(ctx, u, y_d), rtol=0, atol=1e-12)


def _state_hash(L):
    h = hashlib.sha256()
    R = L._rows
    for a in (R.rel, R.vel, R.dwell, R.u, R.y, R.site_x, R.ball, R.kind, R.accepted):
        h.update(np.ascontiguousarray(a).tobytes())
    m = L.model
    for f in (m._fa, m._fx, m._fy):
        h.update(f.th.tobytes())
        h.update(f.P.tobytes())
    h.update(m.theta().tobytes())
    h.update(repr((R.n, m.n_accepted, m.n_rejected, m.last_rejection)).encode())
    return h.hexdigest()


def test_queries_do_not_change_state():
    L, ctx = _trained(n=9)
    before = _state_hash(L)
    for _ in range(3):
        L.command(ctx, Y_D)
        L.model_command(ctx, Y_D)
        L.predict(ctx, Y_D)
        L.predict(ctx, Y_D, Y_D)
        L.local_model(ctx, Y_D)
        L.evidence(ctx)
        L.model.forward(Y_D, 0.0, 0)
        L.model.inverse(Y_D, 0.0, 0)
        L.model.summary()
    assert _state_hash(L) == before


# --------------------------------------------------------------------------- convergence (plan § 1.1)
def test_a_biased_plant_converges_within_a_handful_of_throws_with_the_two_throw_delay():
    """Columns-like: alternating Sites x = -/+0.125, plant K 1.08, b (10, 12) mm, s -0.09/m plus an
    (8, -6, 5) mm residual f0 cannot see; landing noise 5 mm per axis; row i reaches the Learner
    only after throw i+1 is commanded (the real loop's 2-throw delay). Measured 2026-11-06 over
    10 seeds (k 16, w0 2, per-query axis-0 offset): the mean |commanded landing error| per throw is
    <= 3.8 mm per axis from throw 6 on and settles at ~1.2-1.9 mm by throw 12 (77 mm in apex at
    throws 0 and 1)."""
    errs = []
    for seed in range(10):
        rng = np.random.default_rng(seed)
        L, pend, e = Learner(), [], []
        for i in range(14):
            sx = (-0.125, 0.125)[i % 2]
            ctx = _ctx(sx)
            u = L.command(ctx, Y_D)
            truth = _plant(u, sx, (0.008, -0.006, 0.005))
            e.append(truth - Y_D)
            pend.append(_exp(ctx, u, truth + rng.normal(0, 0.005, 3)))
            if len(pend) == 2:
                L.observe(pend.pop(0))
        errs.append(np.abs(np.array(e)))
    errs = np.array(errs)                        # (seed, throw, axis) [m]
    assert errs[:, 0, 2].mean() > 0.070          # the bias is real: the first Throw misses by ~77 mm
    late = errs[:, 6:, :].mean(axis=(0, 1))
    assert np.all(late < 0.004), late * 1e3


# --------------------------------------------------------------------------- T-U5
@pytest.mark.parametrize('bad', [math.nan, math.inf])
def test_t_u5_non_finite_inputs_raise_value_error_by_name(bad):
    L, ctx = _trained(n=3)
    for name, n in (('site_xy_m', 2), ('release_m', 3), ('release_vel_mps', 3), ('seat_offset_m', 2)):
        kw = dict(site_xy_m=(0.0, 0.0), release_m=(0.0, 0.0, 0.4), release_vel_mps=(0.0, 0.0, 4.4),
                  dwell_s=0.4, ball_id=0, release_kind='free_cup', seat_offset_m=(0.0, 0.0))
        v = [0.0] * n
        v[-1] = bad
        kw[name] = tuple(v)
        with pytest.raises(ValueError, match=name):
            ThrowContext(**kw)
    with pytest.raises(ValueError, match='dwell_s'):
        _ctx(dwell=math.inf)
    with pytest.raises(ValueError, match='release_kind'):
        _ctx(kind='thrown')
    with pytest.raises(ValueError, match='ball_id'):
        _ctx(ball=-1)
    y_bad = np.array([0.0, bad, 0.9])
    for call in (lambda: L.command(ctx, y_bad), lambda: L.model_command(ctx, y_bad),
                 lambda: L.local_model(ctx, y_bad)):
        with pytest.raises(ValueError, match='y_d'):
            call()
    with pytest.raises(ValueError, match=r'\bu\b'):
        L.predict(ctx, y_bad)
    with pytest.raises(ValueError, match='ctx'):
        L.command((0.0, 0.0, 0.0, 0.0), Y_D)
    before = _state_hash(L)
    for field in ('u', 'y', 'release_m', 'release_vel_mps', 'site_xy_m'):
        e = _exp(ctx, [0.0, 0.0, 0.9], [0.0, 0.0, 0.97])
        v = np.array(getattr(e, field), dtype=float)
        v[0] = bad
        setattr(e, field, v)
        with pytest.raises(ValueError, match=field):
            L.observe(e)
    with pytest.raises(ValueError, match='v2 field'):
        L.observe(types.SimpleNamespace(x=np.zeros(4), u=np.zeros(3), y=np.zeros(3)))
    assert _state_hash(L) == before                  # a refused row changes nothing


def test_a_dwell_of_nan_is_unknown_and_accepted():
    L = Learner()
    c = _ctx(0.0, dwell=math.nan)
    L.observe(_exp(c, [0.0, 0.0, 0.9], _plant([0.0, 0.0, 0.9], 0.0)))
    assert L.evidence(c)[1] == 1
    assert L.evidence(_ctx(0.0, dwell=0.4))[1] == 1  # h_d = inf: the dwell term is inert


def test_learner_config_rejects_bad_values():
    for kw in (dict(k=0), dict(k=2.0), dict(h_p=0.0), dict(h_v=math.nan), dict(w0=-1.0),
               dict(h_d=0.0), dict(h_d=math.nan), dict(c_max_xy_m=-0.01), dict(c_max_apex_m=math.inf)):
        with pytest.raises(ValueError):
            LearnerConfig(**kw)
    for kw in (dict(forgetting=0.0), dict(forgetting=1.01), dict(prior_sd_K=0.0), dict(n_balls=0)):
        with pytest.raises(ValueError):
            PlantModelConfig(**kw)


def test_a_row_the_plant_model_rejects_does_not_enter_the_residual_memory():
    L, ctx = _trained(n=5)
    ev, cmd = L.evidence(ctx), L.command(ctx, Y_D).copy()
    u = np.array([0.0, 0.0, 0.9])
    assert L.observe(_exp(ctx, u, _plant(u, ctx.site_xy_m[0]) + np.array([0.300, 0.0, 0.0]))) is False
    assert L.n_rows == 86                            # the row is kept (counted), not used
    assert L.evidence(ctx) == ev
    assert np.array_equal(L.command(ctx, Y_D), cmd)


def _closed_loop(ctxs, extras, n, noise, seed, cfg=None):
    """Fly ``n`` Throws cycling through ``ctxs`` with y_d = Y_D and the 2-throw delay; returns
    |commanded landing error| (n, 3) [m] (noise-free truth minus y_d) and the Learner."""
    rng = np.random.default_rng(seed)
    L, pend, e = Learner(config=cfg), [], []
    for i in range(n):
        ctx, extra = ctxs[i % len(ctxs)], extras[i % len(ctxs)]
        u = L.command(ctx, Y_D)
        truth = _plant(u, ctx.site_xy_m[0], extra)
        e.append(np.abs(truth - Y_D))
        pend.append(_exp(ctx, u, truth + rng.normal(0, noise, 3)))
        if len(pend) == 2:
            L.observe(pend.pop(0))
    return np.array(e), L


def test_a_fresh_learner_at_one_off_centre_site_corrects_the_whole_axis0_offset():
    """The oval case end to end: a fresh Learner flying ONLY Site x = -0.075 (carried), axis-0
    offset 24.75 mm (b_x 10 + s x 6.75 + residual 8), noise 5 mm, 2-throw delay. Until the
    per-query rule (2026-11-06 follow-up) b_x and s both stayed at their prior here and only
    alpha <= 2/3 of the offset was corrected: it settled at 8.7 mm. Measured 2026-11-06 (10 seeds):
    24.8 mm at throws 0-1, 4.9 at throw 2, ~1.2 mm from throw 12 on: what is left is the
    estimation noise of a 5 mm-noise plant (f0 and the residual each average a finite window),
    not a bias."""
    ctx = _ctx(-0.075, rel=(-0.06, 0.0, 0.4), vel=(0.1, 0.0, 4.4), kind='carried')
    errs = np.array([_closed_loop([ctx], [(0.008, -0.006, 0.005)], 24, 0.005, seed)[0] for seed in range(10)])
    assert errs[:, 0, 0].mean() > 0.024                       # the offset is real
    late = errs[:, 12:, :].mean(axis=(0, 1))
    assert np.all(late < 0.004), late * 1e3
    assert late[0] < 0.0025, late * 1e3                       # axis 0 carries no 1/3 floor


def test_a_persistent_residual_f0_cannot_carry_is_corrected_by_alpha_up_to_0_889():
    """Two release kinds at the SAME Site with opposite +-12 mm residuals: f0's b averages them
    away, so only L2 can correct them and it corrects alpha <= k / (k + w0) of each for ever.
    Default k 16 / w0 2: alpha 0.889, steady error ~(1 - alpha) 12 = 1.3 mm; the probe's predictor
    optimum k 8 / w0 4 (alpha 2/3) leaves ~4 mm, and either change alone (alpha 0.8) ~2.4 mm."""
    a = _ctx(-0.075, rel=(-0.06, 0.0, 0.4), vel=(0.1, 0.0, 4.4), kind='carried')
    b = _ctx(-0.075, rel=(-0.06, 0.0, 0.4), vel=(0.1, 0.0, 4.4), kind='free_cup')
    errs = np.array([_closed_loop([a, b], [(0.012, 0.012, 0.0), (-0.012, -0.012, 0.0)], 80, 0.002, seed)[0]
                     for seed in range(6)])
    late = errs[:, 40:, :2].mean()
    assert late < 0.0020, late * 1e3              # measured 2026-11-06: 1.34 mm (k 8 / w0 4: 3.97)


# ---- the kernel measures position on the release Site, never on the catch offset (2026-11-06)

def _with_seat(ctx, seat_xy):
    """``ctx`` as a same-Site fold whose ball was caught ``seat_xy`` off the Site: the release
    position and the seat offset move together (the executor's ``_latest_ctx``)."""
    rel = (ctx.release_m[0] + seat_xy[0], ctx.release_m[1] + seat_xy[1], ctx.release_m[2])
    return dataclasses.replace(ctx, release_m=rel, seat_offset_m=tuple(seat_xy))


def test_the_command_does_not_move_with_the_catch_offset():
    """The catch offset is the Catch aim, which follows the tracker's in-flight estimate. With it
    in the kernel a 10 mm aim shift moved the command past the 1 mm early-command tolerance on
    29 % of corpus queries (probe 2026-11-06), and the memory was partitioned by where each ball
    happened to land (the old law's 2026-10-12 ``h_seat`` fix). At a fixed release velocity the
    command, the evidence and the prediction are IDENTICAL for any offset."""
    ctx = _ctx(-0.075, kind='free_cup')
    L, _ = _trained(n=40, ctx=ctx)
    y_d = np.array([0.0, 0.0, 0.9])
    u0, ev0 = L.command(ctx, y_d), L.evidence(ctx)
    assert ev0[1] > 0
    for seat in ((0.010, 0.0), (-0.030, 0.020), (0.0, -0.030)):
        c = _with_seat(ctx, seat)
        np.testing.assert_array_equal(L.command(c, y_d), u0)
        assert L.evidence(c) == ev0
        np.testing.assert_array_equal(L.predict(c, u0, y_d), L.predict(ctx, u0, y_d))


def test_rows_caught_off_the_site_support_a_query_at_the_site_in_full():
    """Rows whose balls were caught up to 30 mm off the Site carry the same weight as rows
    caught on it: the offset does not thin the support."""
    ctx = _ctx(0.05)
    rng = np.random.default_rng(3)
    on, off = Learner(), Learner()
    for i in range(30):
        u = np.array([0.0, 0.0, 0.9])
        y = _plant(u, 0.05, (0.006, 0.0, 0.0)) + rng.normal(0.0, 0.002, 3)
        on.observe(_exp(ctx, u, y))
        off.observe(_exp(_with_seat(ctx, rng.uniform(-0.030, 0.030, 2)), u, y))
    assert on.evidence(ctx) == off.evidence(ctx)
    y_d = np.array([0.0, 0.0, 0.9])
    np.testing.assert_array_equal(on.command(ctx, y_d), off.command(ctx, y_d))


def test_a_different_site_or_release_height_still_separates_rows():
    """What the kernel DOES measure: the release Site's xy and the release height."""
    ctx = _ctx(-0.075)
    L, _ = _trained(n=40, ctx=ctx)
    far_site = dataclasses.replace(ctx, site_xy_m=(0.300, 0.0))    # no row within 3 bandwidths
    high = dataclasses.replace(ctx, release_m=(ctx.release_m[0], ctx.release_m[1],
                                               ctx.release_m[2] + 0.5))
    assert L.evidence(ctx)[1] > 0
    assert L.evidence(far_site)[0] < 1e-3 * L.evidence(ctx)[0] or L.evidence(far_site)[1] == 0
    assert L.evidence(high)[1] == 0
