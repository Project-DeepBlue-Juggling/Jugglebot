"""``motion/skills/{learner,memory}`` -- the memory-based command learner
(plan § 2.5) and its append-only CSV backing store (plan § 2.2).

Unmarked and parallel-safe: pure arithmetic plus file I/O confined to
``tmp_path``, nothing touches a shared path or a shared clock.

Plan: ``plans/active/two-ball-skill-stack.md`` § 2.2 / § 2.5. Hyperparameters:
owner decision 2026-09-13 (probe 2, ``probe_learner.py``, logbook slug
``2026-09-13-skill-stack-r3-learner-single-site``).
"""

from __future__ import annotations

import logging

import numpy as np
import pytest

from jugglebot.motion.skills import learner as lr
from jugglebot.motion.skills import memory as mem


# ---------------------------------------------------------------------------
# An INDEPENDENT reference implementation of plan § 2.5 steps 1-4, written
# from the paper's closed form via lstsq (never the module's np.linalg.solve
# call) so it exercises a genuinely different numerical path.
# ---------------------------------------------------------------------------

def _reference_command(x, y_d, X, U, Y, cfg):
    n = X.shape[0]
    if n < cfg.k_min:
        return y_d.copy()
    h_y = np.asarray(cfg.h_y, dtype=float)
    d2 = (((X - x) / cfg.h_x) ** 2).sum(axis=1) + (((Y - y_d) / h_y) ** 2).sum(axis=1)
    kk = min(cfg.k, n)
    idx = np.argsort(d2, kind='stable')[:kk]
    w = np.exp(-d2[idx])
    Xs, Us, Ys = X[idx], U[idx], Y[idx]
    u_bar = (w[:, None] * Us).sum(axis=0) / w.sum()
    Z = np.vstack([(Xs - x).T, (Us - u_bar).T, np.ones((1, kk))])
    Theta0 = np.hstack([np.zeros((3, 4)), np.eye(3), u_bar[:, None]])
    sw = np.sqrt(w)
    # Weighted ridge toward Theta0 as one augmented least-squares solve per row
    # (sqrt(w)-scaled data rows plus sqrt(gamma)*(I | Theta0) prior rows).
    Aaug = np.vstack([(Z * sw).T, np.sqrt(cfg.gamma) * np.eye(8)])
    Theta = np.zeros((3, 8))
    for r in range(3):
        baug = np.concatenate([sw * Ys[:, r], np.sqrt(cfg.gamma) * Theta0[r]])
        Theta[r] = np.linalg.lstsq(Aaug, baug, rcond=None)[0]
    D, d = Theta[:, 4:7], Theta[:, 7]
    # eq. 21's argmin as one augmented least-squares solve (D rows plus
    # sqrt(eta)*I rows toward u_bar).
    Aq = np.vstack([D, np.sqrt(cfg.eta) * np.eye(3)])
    bq = np.concatenate([y_d - d, np.zeros(3)])
    return u_bar + np.linalg.lstsq(Aq, bq, rcond=None)[0]


def test_closed_form_matches_an_independent_lstsq_reference():
    cfg = lr.LearnerConfig()
    rng = np.random.default_rng(42)
    for _trial in range(20):
        n = int(rng.integers(cfg.k_min, 41))  # n from k_min to 40
        x = rng.normal(scale=0.01, size=4)
        y_d = np.array([0.0, 0.0, 0.85]) + rng.normal(scale=0.02, size=3)
        X = rng.normal(scale=0.01, size=(n, 4))
        U = y_d + rng.normal(scale=0.02, size=(n, 3))
        Y = U + rng.normal(scale=0.01, size=(n, 3))
        u_mod = lr.command(x, y_d, X, U, Y, cfg)
        u_ref = _reference_command(x, y_d, X, U, Y, cfg)
        np.testing.assert_allclose(u_mod, u_ref, rtol=1e-9, atol=1e-9)


# ---------------------------------------------------------------------------
# Prior limit
# ---------------------------------------------------------------------------

def test_fewer_than_k_min_rows_returns_the_identity_prior_exactly():
    cfg = lr.LearnerConfig()
    y_d = np.array([0.1, -0.05, 0.9])
    for n in range(cfg.k_min):  # 0 rows, ..., k_min - 1 rows
        X = np.zeros((n, 4))
        U = np.zeros((n, 3))
        Y = np.zeros((n, 3))
        u = lr.command(np.zeros(4), y_d, X, U, Y, cfg)
        assert np.array_equal(u, y_d)
        assert u is not y_d, 'must be a COPY, not the caller\'s array'


# ---------------------------------------------------------------------------
# No wander under an exact, noise-free plant
# ---------------------------------------------------------------------------

def test_no_wander_when_the_plant_is_exact_and_memory_is_noise_free():
    """y = u exactly, and every past row already targeted y_d -- the learner
    must not drift the command away from y_d."""
    cfg = lr.LearnerConfig()
    y_d = np.array([0.1, -0.05, 0.9])
    rng = np.random.default_rng(0)
    n = 30
    X = rng.normal(scale=1e-3, size=(n, 4))   # varied query states
    U = np.tile(y_d, (n, 1))                  # every past command targeted y_d
    Y = U.copy()                              # exact plant: observed == commanded
    x = rng.normal(scale=1e-3, size=4)
    u = lr.command(x, y_d, X, U, Y, cfg)
    assert np.max(np.abs(u - y_d)) < 1e-9


# ---------------------------------------------------------------------------
# Corrects a bias
# ---------------------------------------------------------------------------

def test_corrects_an_affine_bias_within_six_throws():
    """Plant y = u + b, b = (0.03, -0.03, 0.09) m; iterate 6 throws from a
    cold memory, appending (x, u, y) after each. Measured 2026-09-13
    (``pytest tests/motion/test_learner.py -k corrects_an_affine_bias -q``,
    this exact scenario, LearnerConfig defaults k=16/k_min=2/h_x=0.01/
    h_y=(0.05,0.05,0.2)/gamma=1e-2/eta=0.2):

        throw 1: err_xy=42.426 mm, err_t=90.000 ms  (n=0 rows -> prior)
        throw 2: err_xy=42.426 mm, err_t=90.000 ms  (n=1 < k_min=2 -> prior)
        throw 3: err_xy= 7.510 mm, err_t=15.932 ms  (first correction, n=2)
        throw 4: err_xy= 4.070 mm, err_t= 8.633 ms
        throw 5: err_xy= 2.851 mm, err_t= 6.047 ms  (time error still > 5 ms)
        throw 6: err_xy= 2.221 mm, err_t= 4.711 ms  (both under 5 mm / 5 ms)

    So the plan's "in-band by throw 5" target (§ R3 gate) needs throw 6 at
    this eta; pinned with margin at throw 6, not throw 5.
    """
    cfg = lr.LearnerConfig()
    y_d = np.array([0.0, 0.0, 0.857])
    b = np.array([0.03, -0.03, 0.09])
    x = np.zeros(4)
    X = np.zeros((0, 4))
    U = np.zeros((0, 3))
    Y = np.zeros((0, 3))
    errs_xy_mm = []
    errs_t_ms = []
    for _throw in range(6):
        u = lr.command(x, y_d, X, U, Y, cfg)
        y = u + b
        errs_xy_mm.append(1e3 * np.linalg.norm((y - y_d)[:2]))
        errs_t_ms.append(1e3 * abs((y - y_d)[2]))
        X = np.vstack([X, x])
        U = np.vstack([U, u])
        Y = np.vstack([Y, y])
    # Monotone non-increasing once correction starts (throw 3 onward).
    assert all(a >= b - 1e-9 for a, b in zip(errs_xy_mm[2:], errs_xy_mm[3:]))
    assert all(a >= b - 1e-9 for a, b in zip(errs_t_ms[2:], errs_t_ms[3:]))
    # Measured 2.221 mm / 4.711 ms at throw 6 -- pinned with margin.
    assert errs_xy_mm[-1] < 5.0
    assert errs_t_ms[-1] < 5.0


# ---------------------------------------------------------------------------
# Determinism
# ---------------------------------------------------------------------------

def test_identical_inputs_are_bitwise_deterministic():
    cfg = lr.LearnerConfig()
    rng = np.random.default_rng(7)
    n = 25
    x = rng.normal(scale=0.01, size=4)
    y_d = np.array([0.0, 0.0, 0.85]) + rng.normal(scale=0.02, size=3)
    X = rng.normal(scale=0.01, size=(n, 4))
    U = y_d + rng.normal(scale=0.02, size=(n, 3))
    Y = U + rng.normal(scale=0.01, size=(n, 3))
    u1 = lr.command(x, y_d, X, U, Y, cfg)
    u2 = lr.command(x, y_d, X, U, Y, cfg)
    assert np.array_equal(u1, u2)


def test_equal_distance_neighbours_resolve_by_row_order():
    """With more equal-distance rows than k, only the FIRST k (stable-sort
    order) enter the fit -- rows past the cut must not move the result."""
    cfg = lr.LearnerConfig()  # k = 16
    x = np.zeros(4)
    y_d = np.array([0.0, 0.0, 0.85])
    n = 20
    X = np.zeros((n, 4))       # every row at the exact query state -> d2 == 0 for all
    Y = np.tile(y_d, (n, 1))   # every row at the exact target -> d2 == 0 for all
    rng = np.random.default_rng(3)
    U = y_d + rng.normal(scale=0.02, size=(n, 3))

    u_full = lr.command(x, y_d, X, U, Y, cfg)

    # Recompute using only the first k=16 rows directly: must match exactly.
    u_first_k = lr.command(x, y_d, X[:16], U[:16], Y[:16], cfg)
    assert np.array_equal(u_full, u_first_k)

    # Perturbing a row PAST the cut (index 16) must not change the result.
    U_perturbed = U.copy()
    U_perturbed[16] += 1.0
    u_perturbed = lr.command(x, y_d, X, U_perturbed, Y, cfg)
    assert np.array_equal(u_full, u_perturbed)

    # Perturbing a row INSIDE the cut (index 0) must change the result.
    U_changed = U.copy()
    U_changed[0] += 1.0
    u_changed = lr.command(x, y_d, X, U_changed, Y, cfg)
    assert not np.array_equal(u_full, u_changed)


# ---------------------------------------------------------------------------
# Input validation
# ---------------------------------------------------------------------------

def test_command_rejects_a_bad_x_shape():
    with pytest.raises(ValueError, match='x'):
        lr.command(np.zeros(3), np.zeros(3), np.zeros((0, 4)), np.zeros((0, 3)),
                   np.zeros((0, 3)))


def test_command_rejects_a_bad_y_d_shape():
    with pytest.raises(ValueError, match='y_d'):
        lr.command(np.zeros(4), np.zeros(2), np.zeros((0, 4)), np.zeros((0, 3)),
                   np.zeros((0, 3)))


def test_command_rejects_non_finite_query():
    with pytest.raises(ValueError, match='finite'):
        lr.command(np.array([0.0, 0.0, 0.0, float('nan')]), np.zeros(3),
                   np.zeros((0, 4)), np.zeros((0, 3)), np.zeros((0, 3)))


def test_a_command_that_cannot_be_computed_raises_rather_than_returning_nan():
    """Every neighbour 100 m from the target in y: the weights underflow to
    exactly 0, u_bar is 0/0, and the learner must refuse by name — a NaN
    command would otherwise travel on to a throw terminal."""
    cfg = lr.LearnerConfig()
    x = np.zeros(4)
    y_d = np.array([0.0, 0.0, 0.857])
    X = np.zeros((3, 4))
    U = np.tile(y_d, (3, 1))
    Y = U + np.array([100.0, 0.0, 0.0])
    with pytest.raises(ValueError, match='not finite'):
        lr.command(x, y_d, X, U, Y, cfg)


def test_command_rejects_mismatched_row_counts():
    with pytest.raises(ValueError, match='U'):
        lr.command(np.zeros(4), np.zeros(3), np.zeros((3, 4)), np.zeros((2, 3)),
                   np.zeros((3, 3)))


@pytest.mark.parametrize('field,value', [
    ('k', 1), ('k_min', 0), ('h_x', 0.0), ('h_x', -1.0), ('gamma', 0.0),
    ('gamma', -1e-3), ('eta', 0.0), ('eta', -1.0),
])
def test_learner_config_rejects_non_positive_values(field, value):
    kwargs = {field: value}
    if field == 'k':
        kwargs['k_min'] = 2  # keep k < k_min to trigger the k >= k_min check
    with pytest.raises(ValueError):
        lr.LearnerConfig(**kwargs)


def test_learner_config_rejects_a_bad_h_y():
    with pytest.raises(ValueError, match='h_y'):
        lr.LearnerConfig(h_y=(0.05, 0.05))
    with pytest.raises(ValueError, match='h_y'):
        lr.LearnerConfig(h_y=(0.05, 0.05, -0.2))


# ===========================================================================
# memory.Memory / memory.Experience / memory.memory_path
# ===========================================================================

def _exp(i: int) -> mem.Experience:
    return mem.Experience(
        x=np.array([0.01 * i, -0.02 * i, 0.0, 0.0]),
        u=np.array([0.0, 0.0, 0.85 + 0.001 * i]),
        y=np.array([0.001 * i, -0.001 * i, 0.85 + 0.0011 * i]),
        t_abs_s=1_700_000_000.0 + i, ball_id=i % 2, caught=(i % 3 != 0))


def test_experience_validates_shapes_and_finiteness():
    with pytest.raises(ValueError, match='x'):
        mem.Experience(x=np.zeros(3), u=np.zeros(3), y=np.zeros(3),
                        t_abs_s=0.0, ball_id=0, caught=True)
    with pytest.raises(ValueError, match='u'):
        mem.Experience(x=np.zeros(4), u=np.array([0.0, float('nan'), 0.0]),
                        y=np.zeros(3), t_abs_s=0.0, ball_id=0, caught=True)


def test_memory_path_builds_the_pinned_layout():
    assert mem.memory_path('/root', 'P1') == '/root/temp/learn/P1/memory.csv'


@pytest.mark.parametrize('bad_plant_id', ['', '..', '.', 'a/b', 'a/../b'])
def test_memory_path_rejects_unsafe_plant_ids(bad_plant_id):
    with pytest.raises(ValueError, match='plant_id'):
        mem.memory_path('/root', bad_plant_id)


def test_missing_file_is_an_empty_memory(tmp_path):
    m = mem.Memory(str(tmp_path / 'no_such' / 'memory.csv'))
    assert len(m) == 0
    X, U, Y = m.arrays()
    assert X.shape == (0, 4) and U.shape == (0, 3) and Y.shape == (0, 3)


def test_append_reload_round_trips_bitwise(tmp_path):
    path = str(tmp_path / 'plant1' / 'memory.csv')
    m = mem.Memory(path)
    exps = [_exp(i) for i in range(5)]
    # Precision case: a ROS-epoch t_abs_s that must round-trip exactly.
    exps[2] = mem.Experience(x=exps[2].x, u=exps[2].u, y=exps[2].y,
                              t_abs_s=1789263419.5 + 0.123456789,
                              ball_id=exps[2].ball_id, caught=exps[2].caught)
    for e in exps:
        m.append(e)
    assert len(m) == 5

    reloaded = mem.Memory(path)
    assert len(reloaded) == 5
    X0, U0, Y0 = m.arrays()
    X1, U1, Y1 = reloaded.arrays()
    np.testing.assert_array_equal(X0, X1)
    np.testing.assert_array_equal(U0, U1)
    np.testing.assert_array_equal(Y0, Y1)


def test_malformed_rows_are_dropped_with_a_warning_and_good_rows_kept_in_order(
        tmp_path, caplog):
    path = tmp_path / 'plant2' / 'memory.csv'
    path.parent.mkdir(parents=True)
    header = ','.join(mem._HEADER)
    good0 = _row_csv(_exp(0))
    good1 = _row_csv(_exp(1))
    short_row = '1.0,2.0,3.0'  # wrong column count
    nan_row = _row_csv(_exp(2)).replace(
        _fmt(_exp(2).x[0]), 'nan', 1)  # a non-finite numeric field
    text_row = ','.join(['abc'] * len(mem._HEADER))  # unparsable
    path.write_text('\n'.join([header, good0, short_row, nan_row, text_row, good1]) + '\n')

    caplog.set_level(logging.WARNING)
    m = mem.Memory(str(path))

    assert len(m) == 2  # only the two good rows survive
    X, U, Y = m.arrays()
    np.testing.assert_array_equal(X[0], _exp(0).x)
    np.testing.assert_array_equal(X[1], _exp(1).x)  # order preserved
    warnings = [r for r in caplog.records if r.levelno == logging.WARNING]
    assert len(warnings) == 3
    # Line numbers: header=1, good0=2, short_row=3, nan_row=4, text_row=5, good1=6.
    assert any('line 3' in r.message for r in warnings)
    assert any('line 4' in r.message for r in warnings)
    assert any('line 5' in r.message for r in warnings)


def _fmt(v: float) -> str:
    return mem._FLOAT_FMT % v


def _row_csv(exp: mem.Experience) -> str:
    return ','.join(
        [_fmt(v) for v in exp.x] + [_fmt(v) for v in exp.u] + [_fmt(v) for v in exp.y]
        + [_fmt(exp.t_abs_s), str(exp.ball_id), str(exp.caught)])


def test_memory_command_delegates_to_learner(tmp_path):
    path = str(tmp_path / 'plant3' / 'memory.csv')
    m = mem.Memory(path)
    y_d = np.array([0.0, 0.0, 0.85])
    x = np.zeros(4)
    cfg = lr.LearnerConfig()
    # Cold memory -> identity prior, matching learner.command directly.
    u_direct = lr.command(x, y_d, *m.arrays(), cfg)
    u_via_memory = m.command(x, y_d, cfg)
    np.testing.assert_array_equal(u_direct, u_via_memory)
