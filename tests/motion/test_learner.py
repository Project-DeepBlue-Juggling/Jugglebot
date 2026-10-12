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
    """Plan § 2.5 as amended 2026-10-04 (learner module docstring): support
    gate on the joint metric, the k MOST RECENT support rows, the anchor on
    the joint kernel, the fit on the STATE kernel only. As amended
    2026-10-12: the seat-offset half of the state has its own bandwidth
    ``h_seat``, and the ridge prior's intercept is ``u_bar`` plus the
    window's unweighted mean residual."""
    n = X.shape[0]
    if n < cfg.k_min:
        return y_d.copy()
    h_y = np.asarray(cfg.h_y, dtype=float)
    h_x = np.array([cfg.h_x, cfg.h_x, cfg.h_seat, cfg.h_seat])
    d2 = (((X - x) / h_x) ** 2).sum(axis=1) + (((Y - y_d) / h_y) ** 2).sum(axis=1)
    support = [i for i in range(n) if d2[i] <= cfg.support_d2]
    idx = np.array(support[max(0, len(support) - cfg.k):], dtype=int)
    if idx.size < cfg.k_min:
        return y_d.copy()
    kk = idx.size
    w = np.exp(-d2[idx])
    Xs, Us, Ys = X[idx], U[idx], Y[idx]
    u_bar = (w[:, None] * Us).sum(axis=0) / w.sum()
    v = np.exp(-(((Xs - x) / h_x) ** 2).sum(axis=1))
    Z = np.vstack([(Xs - x).T, (Us - u_bar).T, np.ones((1, kk))])
    r_bar = np.mean(Ys - Us, axis=0)
    Theta0 = np.hstack([np.zeros((3, 4)), np.eye(3), (u_bar + r_bar)[:, None]])
    sv = np.sqrt(v)
    # Weighted ridge toward Theta0 as one augmented least-squares solve per row
    # (sqrt(v)-scaled data rows plus sqrt(gamma)*(I | Theta0) prior rows).
    Aaug = np.vstack([(Z * sv).T, np.sqrt(cfg.gamma) * np.eye(8)])
    Theta = np.zeros((3, 8))
    for r in range(3):
        baug = np.concatenate([sv * Ys[:, r], np.sqrt(cfg.gamma) * Theta0[r]])
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
# The forward model never sees a sample chosen by its own outcome (A2,
# 2026-10-04 -- learner module docstring)
# ---------------------------------------------------------------------------

#: The 48 most recent P1 self-toss rows of ``temp/learn/jugglebot/memory.csv``
#: (x = (-0.05, 0, 0, 0), sittings 2026-10-02 18:47 .. 2026-10-03 00:26, real
#: hardware, uncurated -- one row lands > 100 mm off), copied as data:
#: (u0, u1, u_apex, y0, y1, y_apex) in mm, rounded to 0.1 mm. Mean residual
#: y - u = (+13.2, +8.3) mm laterally: this plant lands beyond its command in
#: +y, so a learner must command y BELOW the target. The pre-fix law (k
#: nearest in the joint state/OUTCOME metric) returned u_y = +2.8 mm on these
#: rows -- the wrong sign -- because its 16 neighbours were the 16 throws that
#: happened to land nearest the target (their mean landing IS the target, so
#: its step was ~0); the fixed law returns u_y = -3.2 mm (probe
#: ``scratchpad/s3/a2/fixture_probe.py``, 2026-10-04).
_P1_SELF_TOSS_ROWS_MM = [
    (-15.7, 1.5, 835.2, 29.3, 37.4, 918.4),
    (-15.7, 1.5, 835.2, 8.1, -2.3, 924.0),
    (-14.8, 1.4, 835.3, 15.3, 17.1, 919.6),
    (-14.8, 1.4, 835.3, -21.9, 5.8, 917.8),
    (-15.1, 2.2, 835.3, -26.6, 10.4, 910.9),
    (-15.1, 2.2, 835.3, 26.9, -5.4, 921.6),
    (-15.1, 2.2, 835.3, 26.4, 34.5, 929.5),
    (-15.1, 2.2, 835.3, -19.9, 18.4, 919.9),
    (-15.1, 2.2, 835.3, 45.1, 18.9, 934.5),
    (-15.1, 2.2, 835.3, 8.2, 42.9, 917.1),
    (-15.1, 2.2, 835.3, 22.6, 43.9, 915.9),
    (-15.1, 2.2, 835.3, 3.9, 13.5, 921.1),
    (-15.1, 2.2, 835.3, 48.7, 39.6, 919.8),
    (-15.1, 2.2, 835.3, 36.6, 11.8, 921.0),
    (-15.1, 2.2, 835.3, -13.7, 27.9, 921.4),
    (-15.1, 2.2, 835.3, -7.7, 27.6, 918.1),
    (-15.1, 2.2, 835.3, -4.4, 9.1, 922.4),
    (-15.1, 2.2, 835.3, -23.5, 72.5, 924.3),
    (-15.1, 2.2, 835.3, -335.3, -171.4, 813.3),
    (-15.1, 2.0, 835.7, 23.6, 8.3, 918.9),
    (-15.1, 2.0, 835.7, 15.0, 25.7, 923.9),
    (-15.1, 2.0, 835.7, 23.7, 9.1, 930.8),
    (-15.1, 2.0, 835.7, -16.8, 37.7, 923.2),
    (-15.1, 2.0, 835.7, 43.1, 27.3, 924.0),
    (-15.1, 2.0, 835.7, -38.0, 34.9, 924.8),
    (-15.1, 2.0, 835.7, 29.4, -3.4, 929.5),
    (-15.1, 2.0, 835.7, -8.2, 5.8, 929.9),
    (-15.1, 2.0, 835.7, 12.3, -33.0, 930.2),
    (-15.1, 2.0, 835.7, 10.6, 7.2, 920.7),
    (-15.1, 3.0, 819.4, -6.7, -2.2, 888.3),
    (-15.1, 3.0, 819.4, -5.5, 37.3, 893.1),
    (-15.1, 3.0, 819.4, -23.2, -11.5, 891.8),
    (-15.0, 2.4, 819.6, -0.3, -2.1, 899.6),
    (-15.0, 2.4, 819.6, -13.2, 8.3, 896.3),
    (-15.5, 3.0, 819.8, 13.2, -6.8, 888.8),
    (-15.5, 3.0, 819.8, -19.6, -4.8, 886.1),
    (-15.5, 3.0, 819.8, -13.7, -15.4, 893.1),
    (-15.5, 3.0, 819.8, -33.8, 4.0, 897.3),
    (-15.5, 3.0, 819.8, 39.8, 11.5, 903.4),
    (-15.5, 3.0, 819.8, -13.4, 26.5, 885.1),
    (-15.5, 3.0, 819.8, 6.3, -24.8, 894.8),
    (-15.5, 3.0, 819.8, 44.4, 39.9, 914.8),
    (-15.5, 3.0, 819.8, -14.8, -3.3, 902.0),
    (-15.5, 3.0, 819.8, -22.0, 56.9, 911.8),
    (-15.5, 3.0, 819.8, 29.8, 41.7, 897.6),
    (-15.5, 3.0, 819.8, 6.9, -10.4, 906.5),
    (-15.9, 4.1, 819.1, 7.4, -11.6, 887.0),
    (-15.9, 4.1, 819.1, 8.6, 10.9, 897.4),
]


def _p1_self_toss_memory():
    F = np.asarray(_P1_SELF_TOSS_ROWS_MM, dtype=float) / 1e3
    n = F.shape[0]
    return np.tile([-0.05, 0.0, 0.0, 0.0], (n, 1)), F[:, :3], F[:, 3:]


def test_real_p1_memory_commands_against_the_plants_measured_offset():
    """Regression for the 2026-10-02 sign flip: on the real rows the lateral
    command opposes the mean residual on BOTH axes (the plant lands beyond
    its command, so the command sits short of the target)."""
    X, U, Y = _p1_self_toss_memory()
    y_d = np.array([0.0, 0.0, 0.9])
    u = lr.command(X[0], y_d, X, U, Y, lr.LearnerConfig())
    residual = (Y - U).mean(axis=0)
    assert residual[0] > 0.0 and residual[1] > 0.0
    assert np.sign(u[0] - y_d[0]) == -np.sign(residual[0])
    assert np.sign(u[1] - y_d[1]) == -np.sign(residual[1])
    # Measured -3.2 mm (2026-10-04); the pre-fix law's +2.8 mm fails here.
    assert u[1] < -1.5e-3


def test_a_noisy_plant_at_one_state_is_not_frozen_by_its_own_luckiest_throws():
    """The mechanism, synthetic: 160 rows at one state, commands constant to
    ~1 mm, plant y = u + b + noise (sd 23 mm, b_y = +11 mm). The command
    must step against b. The pre-fix law took the 16 rows that landed
    nearest the target, saw a plant on target, and stepped ~0."""
    rng = np.random.default_rng(11)
    n = 160
    y_d = np.array([0.0, 0.0, 0.9])
    u0 = np.array([-0.015, 0.003, 0.82])
    b = np.array([0.015, 0.011, 0.08])
    X = np.zeros((n, 4))
    U = u0 + rng.normal(scale=[1e-3, 1e-3, 2e-3], size=(n, 3))
    Y = U + b + rng.normal(scale=[0.023, 0.023, 0.020], size=(n, 3))
    u = lr.command(np.zeros(4), y_d, X, U, Y, lr.LearnerConfig())
    # The ideal damped step from u0 is -(u0 + b - y_d)/(1 + eta) = -11.7 mm
    # in y; a 16-row window carries ~6 mm of noise, so pin half of it.
    assert u[1] - u0[1] < -0.5 * (u0[1] + b[1]) / 1.2


def test_an_identical_command_neighbourhood_has_exactly_the_identity_slope():
    """When the neighbours' commands do not vary the data carry no
    information on dy/du, and the slope IS the prior: D == I."""
    cfg = lr.LearnerConfig()
    rng = np.random.default_rng(5)
    n = 12
    x = np.zeros(4)
    Xs = np.zeros((n, 4))
    Us = np.tile([-0.015, 0.003, 0.82], (n, 1))
    Ys = Us + 0.01 + rng.normal(scale=0.02, size=(n, 3))
    d2s = rng.uniform(0.0, 2.0, size=n)
    u_bar, Theta = lr._local_fit(x, Xs, Us, Ys, d2s, cfg)
    np.testing.assert_allclose(Theta[:, 4:7], np.eye(3), rtol=0.0, atol=1e-12)
    np.testing.assert_allclose(u_bar, Us[0], rtol=0.0, atol=1e-15)


def test_a_well_spread_neighbourhood_still_identifies_a_true_slope():
    """The fix must not blunt learning the data support: a noise-free plant
    y = c + D_true (u - u0), D_true = diag(0.6, 0.6, 0.8), 40 rows with u
    spread +-0.10 m. The fitted D must move >= 70 % of the way from the
    prior I to D_true (measured 2026-10-04: 88 / 79 / 76 %; the pre-fix
    joint-kernel fit reached 55 / 58 / 63 %, because weighting by the
    outcome's distance down-weights exactly the rows that spread u)."""
    cfg = lr.LearnerConfig()
    rng = np.random.default_rng(1)
    D_true = np.diag([0.6, 0.6, 0.8])
    u0 = np.array([0.0, 0.0, 0.82])
    c = np.array([0.01, 0.01, 0.9])
    n = 40
    X = np.zeros((n, 4))
    U = u0 + rng.uniform(-1.0, 1.0, size=(n, 3)) * 0.10
    Y = c + (U - u0) @ D_true.T
    y_d = np.array([0.0, 0.0, 0.9])
    idx, d2 = lr._neighbourhood(np.zeros(4), y_d, X, Y, cfg)
    assert idx.size == cfg.k and list(idx) == list(range(n - cfg.k, n))
    _u_bar, Theta = lr._local_fit(np.zeros(4), X[idx], U[idx], Y[idx], d2[idx], cfg)
    frac = (1.0 - np.diag(Theta[:, 4:7])) / (1.0 - np.diag(D_true))
    assert np.all(frac >= 0.70), frac


# ---------------------------------------------------------------------------
# Thin state support (2026-10-12, U1: sitting 7b's support collapse)
# ---------------------------------------------------------------------------

def _overflying_plant(seed, X_row, n=16):
    """16 rows of a plant that flies 75 mm above its apex command (sitting
    7b: u_apex 0.875 -> apex 0.95), commands already converged on y_d."""
    rng = np.random.default_rng(seed)
    U = np.array([0.0, 0.0, 0.875]) + rng.normal(scale=[1e-3, 1e-3, 2e-3], size=(n, 3))
    Y = U + np.array([0.0, 0.0, 0.075]) + rng.normal(scale=[0.02, 0.02, 0.007], size=(n, 3))
    return np.tile(X_row, (n, 1)), U, Y


@pytest.mark.parametrize('seed', range(5))
def test_thin_state_support_falls_back_on_the_window_residual_not_the_identity(seed):
    """Every support row sits 25 mm from the query state: sum v = 16 e^-6.25
    = 0.031 (sitting 7b row 465: 0.032), so the ridge carries the fit onto
    its prior. The prior must be what those rows say the plant does (lands
    75 mm above command), not "apex = command": the command stays at the
    converged 0.875. Measured 2026-10-12 over seeds 0-4: 0.872-0.878; the
    identity prior gave 0.888-0.892, the upward walk that put row 467 at
    0.901 and its catch over the jerk limit."""
    X, U, Y = _overflying_plant(seed, [0.025, 0.0, 0.0, 0.0])
    y_d = np.array([0.0, 0.0, 0.95])
    cfg = lr.LearnerConfig()
    idx, _d2 = lr._neighbourhood(np.zeros(4), y_d, X, Y, cfg)
    assert idx.size == 16, 'every row is inside the support gate'
    u = lr.command(np.zeros(4), y_d, X, U, Y, cfg)
    assert abs(u[2] - 0.875) < 0.005, u


@pytest.mark.parametrize('seed', range(5))
def test_the_seat_offset_does_not_partition_the_memory(seed):
    """Rows at seat offset 0; a query from the far corner of the lateral
    authority (-20, +20) mm must get the command a query at offset 0 gets
    (pre-registered test on the real memory, 2026-10-12: no offset effect
    above noise on any channel). Measured: identical to 1e-6 m. With the
    pre-fix shared 10 mm bandwidth AND the identity prior, the corner query
    jumped +39..+43 mm in apex; the residual prior alone (``h_seat`` back at
    10 mm) holds it within 0.5 mm, so each fix is pinned on its own."""
    X, U, Y = _overflying_plant(seed, [-0.0625, 0.0, 0.0, 0.0])
    y_d = np.array([0.0, 0.0, 0.95])
    centre = np.array([-0.0625, 0.0, 0.0, 0.0])
    corner = np.array([-0.0625, 0.0, -0.02, 0.02])
    u_centre = lr.command(centre, y_d, X, U, Y)
    np.testing.assert_allclose(lr.command(corner, y_d, X, U, Y), u_centre,
                               rtol=0.0, atol=1e-4)
    shared = lr.LearnerConfig(h_seat=0.01)
    u_shared = lr.command(corner, y_d, X, U, Y, shared)
    assert abs(u_shared[2] - u_centre[2]) < 0.005, (u_shared, u_centre)


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


def test_the_window_is_the_most_recent_k_support_rows():
    """With more support rows than k, only the LAST k (most recent, row
    order = append order) enter the fit -- older rows must not move the
    result, whatever their distance (2026-10-04: the window is chosen by
    time, never by how close a row's outcome came to the target)."""
    cfg = lr.LearnerConfig()  # k = 16
    x = np.zeros(4)
    y_d = np.array([0.0, 0.0, 0.85])
    n = 20
    X = np.zeros((n, 4))       # every row at the exact query state
    Y = np.tile(y_d, (n, 1))   # every row at the exact target -> d2 == 0 for all
    rng = np.random.default_rng(3)
    U = y_d + rng.normal(scale=0.02, size=(n, 3))

    u_full = lr.command(x, y_d, X, U, Y, cfg)

    # Recompute using only the last k=16 rows directly: must match exactly.
    u_last_k = lr.command(x, y_d, X[4:], U[4:], Y[4:], cfg)
    assert np.array_equal(u_full, u_last_k)

    # Perturbing a row OLDER than the window (index 3) must not change it,
    # even when that moves the row's outcome nearer the target than any other.
    U_perturbed, Y_perturbed = U.copy(), Y.copy()
    U_perturbed[3] += 1.0
    Y_perturbed[3] = y_d
    assert np.array_equal(u_full, lr.command(x, y_d, X, U_perturbed, Y_perturbed, cfg))

    # Perturbing a row INSIDE the window (the newest) must change the result.
    U_changed = U.copy()
    U_changed[-1] += 1.0
    assert not np.array_equal(u_full, lr.command(x, y_d, X, U_changed, Y, cfg))


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


def test_no_row_inside_the_support_returns_the_identity_prior_exactly():
    """Every row 100 m from the target in y: none is inside the support gate,
    so the memory says nothing about this query and the command is the
    identity prior, exactly as for a cold memory (2026-10-04; before, the
    weights underflowed and the skill was refused)."""
    cfg = lr.LearnerConfig()
    x = np.zeros(4)
    y_d = np.array([0.0, 0.0, 0.857])
    X = np.zeros((3, 4))
    U = np.tile(y_d, (3, 1))
    Y = U + np.array([100.0, 0.0, 0.0])
    u = lr.command(x, y_d, X, U, Y, cfg)
    assert np.array_equal(u, y_d)
    assert u is not y_d


def test_a_command_that_cannot_be_computed_raises_rather_than_returning_nan():
    """A non-finite row inside the support makes u_bar inf: the learner must
    refuse by name -- a NaN command would otherwise travel on to a throw
    terminal."""
    cfg = lr.LearnerConfig()
    x = np.zeros(4)
    y_d = np.array([0.0, 0.0, 0.857])
    X = np.zeros((3, 4))
    U = np.tile(y_d, (3, 1))
    U[:, 0] = 1e308            # the weighted sum overflows to inf
    Y = np.tile(y_d, (3, 1))
    with np.errstate(over='ignore'), pytest.raises(ValueError, match='not finite'):
        lr.command(x, y_d, X, U, Y, cfg)


def test_command_rejects_mismatched_row_counts():
    with pytest.raises(ValueError, match='U'):
        lr.command(np.zeros(4), np.zeros(3), np.zeros((3, 4)), np.zeros((2, 3)),
                   np.zeros((3, 3)))


@pytest.mark.parametrize('field,value', [
    ('k', 1), ('k_min', 0), ('h_x', 0.0), ('h_x', -1.0), ('h_seat', 0.0),
    ('h_seat', -1.0), ('h_seat', float('inf')), ('gamma', 0.0),
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


# ── the physical flight band (2026-09-16) ──────────────────────────────────
#
# A row whose observed APEX is not a near-unit multiple of the COMMANDED
# one is not a row about its own throw — it is the next flight's landing,
# captured because the executor kept refreshing the observation after the
# ball had been caught and re-thrown. The executor freezes the observation at
# the crossing; this band is the belt-and-braces guard one level down, on
# BOTH of `Memory`'s paths, so a contaminated file already on disk (the ten
# `temp/learn/*-20260916` memories) cannot be read back into a learner.


def _band_exp(u_apex, y_apex):
    return mem.Experience(x=np.zeros(4), u=np.array([0.0, 0.0, u_apex]),
                           y=np.array([0.0, 0.0, y_apex]), t_abs_s=1.0,
                           ball_id=0, caught=True)


@pytest.mark.parametrize('u_apex,y_apex,want', [
    (0.9, 0.9, True),                # exact
    (0.9, 1.38, True),               # the measured 25 %-fast plant (armA-090)
    # The largest genuine ratio seen: 1.456 in FLIGHT, i.e. 2.12 in apex.
    (0.9, 0.9 * 1.456 ** 2, True),
    # armB-090 row 1 — the next flight. 2.605x in flight is 6.79x in apex.
    (0.9, 0.9 * 2.605 ** 2, False),
    # armB-090 row 3, 1.348x in flight = 1.82x in apex: INSIDE the band, and
    # deliberately so. Its 1.1554 s was the BEAT, not a flight: a ball sitting
    # in the cup has its "landing" predicted at ~now, so this contaminant's
    # ratio is beat/commanded and can sit arbitrarily close to 1. No ratio
    # band can separate that class — the executor's freeze at the crossing
    # does, by refusing any estimate sampled after the ball has landed.
    (0.9, 0.9 * 1.348 ** 2, True),
    (0.9, 0.9 * 0.2, False),         # under the band: a failed release
    (0.9, 2.56 * 0.9, True),         # the closed edges
    (0.9, 0.25 * 0.9, True),
])
def test_apex_in_band(u_apex, y_apex, want):
    assert mem.apex_in_band(u_apex, y_apex) is want


def test_the_apex_band_is_the_square_of_the_retired_flight_band():
    """A flight ratio IS the release-speed ratio and an apex goes as its
    SQUARE, so the band that admitted flights in (0.5, 1.6) x commanded
    (2026-09-16) admits apexes in (0.25, 2.56) x commanded — the same physical
    throws, re-expressed. Written down because the numbers look unrelated."""
    assert mem.APEX_RATIO_BAND == pytest.approx((0.5 ** 2, 1.6 ** 2))


def test_apex_in_band_without_a_commanded_apex_admits():
    """No commanded apex, no band: the guard's whole evidence is the RATIO,
    and with nothing to scale there is no evidence either way."""
    assert mem.apex_in_band(0.0, 5.0) is True
    assert mem.apex_in_band(float('nan'), 5.0) is True
    assert mem.apex_in_band(0.9, float('nan')) is False


def test_append_refuses_a_row_outside_the_band(tmp_path):
    path = str(tmp_path / 'memory.csv')
    m = mem.Memory(path)
    with pytest.raises(ValueError, match='another flight'):
        m.append(_band_exp(0.8569, 2.2317))
    assert len(m) == 0
    m.append(_band_exp(0.8569, 1.0597))
    assert len(m) == 1


def test_a_contaminated_row_is_dropped_on_load(tmp_path, caplog):
    """The quarantine that survives a restart: armB-090's real first three
    rows (2026-09-16), their flights re-expressed as the apexes those
    crossings imply, plus one genuine 1.53x row."""
    path = tmp_path / 'memory.csv'
    rows = [
        ','.join(mem._HEADER),
        '-0.05,0,0,0,0,0,0.9,-0.0031,0.0783,6.1078,'
        '1789540417.3557081,0,True',
        '-0.05,0,0,0,0,0,0.9,-0.0098,0.0967,6.1040,'
        '1789540418.5125885,0,True',
        '-0.05,0,0,0,0,0,0.9,-0.0170,0.0897,1.6369,'
        '1789540419.6694691,0,True',
        '-0.05,0,0,0,0,0,0.9,-0.0170,0.0897,1.3772,'
        '1789540419.6694691,0,True',
    ]
    path.write_text('\n'.join(rows) + '\n')
    with caplog.at_level(logging.WARNING):
        m = mem.Memory(str(path))
    # The two next-flight rows (6.8x commanded) are dropped. The 1.637 m row
    # is the BEAT-shaped contaminant at 1.82x — inside the physical band by
    # construction (see `test_apex_in_band`), so it survives the load: the
    # executor's freeze is what stops it being written in the first place.
    assert len(m) == 2
    _X, _U, Y = m.arrays()
    assert Y[0][2] == pytest.approx(1.6369)
    assert Y[1][2] == pytest.approx(1.3772)
    dropped = [r for r in caplog.records if 'another flight' in r.getMessage()]
    assert len(dropped) == 2


def test_a_pre_apex_flight_time_memory_is_REFUSED_not_reinterpreted(tmp_path):
    """2026-09-18: the third column changed MEANING (seconds of flight ->
    metres of apex) and its numbers still parse, so the header is the only
    thing that can tell the two schemas apart. A pre-2026-09-18 file must
    refuse loudly and name the quarantine convention — silently reading 0.857
    as an apex would teach the learner a plant that does not exist."""
    path = tmp_path / 'memory.csv'
    path.write_text(','.join(mem._LEGACY_FLIGHT_HEADER) + '\n'
                    + '-0.05,0,0,0,0,0,0.85688,-0.003,0.078,0.8721,'
                      '1789540417.3557081,0,True\n')
    with pytest.raises(ValueError, match='quarantine'):
        mem.Memory(str(path))


# ===========================================================================
# learner.predict / Memory.predict -- the FORWARD model (2026-10-13, wave 2
# unit A): the catch prior is where the plant lands from the command that was
# actually SENT (clipped), not the target y_d.
# ===========================================================================

_P1_X = np.array([-0.0625, 0.0, 0.0, 0.0])


def _biased_rows(n, u_rows, bias=(0.020, 0.0, 0.0), seed=0, noise=0.0):
    """``n`` rows at P1 whose landing is the command plus a constant ``bias``
    (m), oldest-first; ``u_rows(i)`` gives row ``i``'s command."""
    rng = np.random.default_rng(seed)
    X = np.tile(_P1_X, (n, 1))
    U = np.array([u_rows(i) for i in range(n)], dtype=float)
    Y = U + np.asarray(bias) + noise * rng.standard_normal((n, 3))
    return X, U, Y


def test_predict_on_an_empty_memory_is_the_identity_prior():
    u = np.array([0.004, -0.002, 0.87])
    y_hat = lr.predict(_P1_X, u, np.zeros((0, 4)), np.zeros((0, 3)),
                       np.zeros((0, 3)))
    np.testing.assert_array_equal(y_hat, u)
    y_hat[0] = 99.0                                   # a COPY
    assert u[0] == 0.004


def test_predict_below_k_min_support_is_the_identity_prior_and_never_raises():
    cfg = lr.LearnerConfig()
    X, U, Y = _biased_rows(cfg.k_min - 1, lambda i: [0.0, 0.0, 0.87])
    u = np.array([0.0, 0.0, 0.87])
    np.testing.assert_array_equal(lr.predict(_P1_X, u, X, U, Y, cfg), u)
    # Rows present but all at another site: outside the gate -> identity.
    X2, U2, Y2 = _biased_rows(8, lambda i: [0.0, 0.0, 0.87])
    X2[:, 0] = +0.0625
    np.testing.assert_array_equal(lr.predict(_P1_X, u, X2, U2, Y2, cfg), u)


def test_predict_a_constant_x_bias_lands_the_command_plus_the_bias():
    """Spread commands, every landing +20 mm in x: the forward model says
    ``u + 20 mm`` for a command inside the spread."""
    X, U, Y = _biased_rows(
        16, lambda i: [0.001 * (i % 5 - 2), 0.001 * (i % 3 - 1),
                       0.87 + 0.002 * (i % 4 - 1.5)])
    u = np.array([0.001, 0.0, 0.87])
    y_hat = lr.predict(_P1_X, u, X, U, Y, y_d=np.array([0.0, 0.0, 0.95]))
    np.testing.assert_allclose(y_hat, u + np.array([0.020, 0.0, 0.0]),
                               atol=5e-4)


def test_predict_a_pinned_command_predicts_the_pinned_landing_not_y_d():
    """Sitting 7's P1 shape: the learner wants -24 mm but the box pins the
    lateral command at its -4 mm edge, so every row has ``u_x = -4 mm`` and
    the ball lands +20 mm beyond it. The forward model predicts the PINNED
    landing (+16 mm), which is what the catch must aim at -- not y_d (0)."""
    X, U, Y = _biased_rows(16, lambda i: [-0.004, 0.0, 0.871])
    y_d = np.array([0.0, 0.0, 0.95])
    Y[:, 2] = 0.95                               # the apex is learned
    u = np.array([-0.004, 0.0, 0.871])
    y_hat = lr.predict(_P1_X, u, X, U, Y, y_d=y_d)
    np.testing.assert_allclose(y_hat, [0.016, 0.0, 0.95], atol=5e-4)
    assert abs(y_hat[0] - y_d[0]) > 0.015
    # The command the learner WANTS (unclipped) is well beyond the pin.
    u_want = lr.command(_P1_X, y_d, X, U, Y)
    assert u_want[0] < -0.010


def test_predict_is_the_forward_half_of_the_commands_own_fit():
    """With a well-spread noisy memory, ``predict`` at the learner's own
    (unclipped) command lands close to the target it was solving for --
    the two are the same local model (eq. 21's ridge leaves a small gap)."""
    X, U, Y = _biased_rows(
        16, lambda i: [0.003 * (i % 5 - 2), 0.003 * (i % 3 - 1),
                       0.87 + 0.01 * (i % 4 - 1.5)],
        bias=(0.012, -0.006, 0.08), seed=7, noise=0.002)
    y_d = np.array([0.0, 0.0, 0.95])
    u = lr.command(_P1_X, y_d, X, U, Y)
    y_hat = lr.predict(_P1_X, u, X, U, Y, y_d=y_d)
    np.testing.assert_allclose(y_hat, y_d, atol=0.006)


def test_predict_rejects_a_non_finite_command():
    with pytest.raises(ValueError, match='u'):
        lr.predict(_P1_X, [np.nan, 0.0, 0.9], np.zeros((0, 4)),
                   np.zeros((0, 3)), np.zeros((0, 3)))


def test_memory_predict_delegates_to_learner(tmp_path):
    m = mem.Memory(str(tmp_path / 'plant4' / 'memory.csv'))
    X, U, Y = _biased_rows(6, lambda i: [0.001 * i, 0.0, 0.87 + 0.001 * i])
    for i in range(6):
        m.append(mem.Experience(x=X[i], u=U[i], y=Y[i],
                                t_abs_s=1_700_000_000.0 + i, ball_id=0,
                                caught=True))
    u = np.array([0.002, 0.0, 0.872])
    y_d = np.array([0.0, 0.0, 0.89])
    np.testing.assert_array_equal(
        m.predict(_P1_X, u, y_d=y_d),
        lr.predict(_P1_X, u, *m.arrays(), lr.LearnerConfig(), y_d=y_d))
