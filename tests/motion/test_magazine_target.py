"""Magazine target: config loader strictness and the base/world/plan frame maps."""
from __future__ import annotations

import numpy as np
import pytest

from jugglebot import bb_base_frame as bf
from jugglebot.motion.skills import magazine_target as mt

R_OWNER = np.array([[-0.99867, 0.05141, 0.00500],
                    [-0.05148, -0.99855, -0.01567],
                    [0.00419, -0.01591, 0.99986]])
O_OWNER = np.array([-725.9, -280.1, 1665.8])


def _write(tmp_path, text):
    p = tmp_path / 'm.yaml'
    p.write_text(text)
    return str(p)


def test_committed_config_loads():
    cfg = mt.load_magazine_config(mt.default_config_path())
    assert np.allclose(cfg.target_base_mm, [550, 300, 750])
    assert cfg.flight_s == 0.90


def test_owner_point_world():
    w = mt.base_to_world_mm([550, 300, 750], R_OWNER, O_OWNER)
    assert np.allclose(w, [-1256, -620, 2413], atol=2.0)


def test_round_trips_and_offset():
    p = np.array([123.0, -45.0, 678.0])
    off = (7.5, -3.0)
    Ro = (lambda u, _s, vt: u @ vt)(*np.linalg.svd(R_OWNER))  # exactly orthonormal
    assert np.allclose(mt.world_to_base_mm(mt.base_to_world_mm(p, Ro, O_OWNER), Ro, O_OWNER), p)
    plan = mt.base_to_plan_mm(p, Ro, O_OWNER, off)
    world = mt.base_to_world_mm(p, Ro, O_OWNER)
    assert np.allclose(plan, world - [7.5, -3.0, 0.0])
    assert np.allclose(mt.plan_to_base_mm(plan, Ro, O_OWNER, off), p)


def test_convention_matches_canonical_base_frame():
    C, E, S = np.array([100., 200., 1500.]), np.array([100., 1200., 1500.]), np.array([-400., 200., 1500.])
    M = (E + S) / 2
    R, o = bf.canonical_base_frame(np.array([C, M, E, S]))
    # E is on base +x at its long-edge distance; S on +y
    assert np.allclose(mt.world_to_base_mm(E, R, o), [1000, 0, 0], atol=1e-6)
    assert np.allclose(mt.world_to_base_mm(S, R, o), [0, 500, 0], atol=1e-6)


@pytest.mark.parametrize('text', [
    '', 'flight_s: 0.9\n', 'target_base_mm: [1, 2, 3]\n',
    'target_base_mm: [1, 2]\nflight_s: 0.9\n',
    'target_base_mm: [1, 2, .nan]\nflight_s: 0.9\n',
    'target_base_mm: [1, 2, "a"]\nflight_s: 0.9\n',
    'target_base_mm: [1, 2, 3]\nflight_s: 0.4\n',
    'target_base_mm: [1, 2, 3]\nflight_s: 1.31\n',
    'target_base_mm: [1, 2, 3]\nflight_s: .nan\n',
    'target_base_mm: [1, 2, 3]\nflight_s: x\n',
])
def test_loader_is_strict(tmp_path, text):
    with pytest.raises(ValueError, match='MAGAZINE_CONFIG'):
        mt.load_magazine_config(_write(tmp_path, text))


def test_missing_file(tmp_path):
    with pytest.raises(ValueError, match='MAGAZINE_CONFIG'):
        mt.load_magazine_config(str(tmp_path / 'nope.yaml'))


def test_valid_custom(tmp_path):
    cfg = mt.load_magazine_config(_write(tmp_path, 'target_base_mm: [1, 2, 3]\nflight_s: 1\n'))
    assert cfg.flight_s == 1.0 and cfg.target_base_mm.shape == (3,)


# ── fit_descending_crossing (U2c) ───────────────────────────────────────

def _parabola(n=30, noise=0.0, t0=1.7e9):
    rng = np.random.RandomState(3)
    g = 9810.0
    ts = t0 + np.linspace(0.0, 0.8, n)
    tau = ts - t0
    z = 900.0 + 3000.0 * tau - 0.5 * g * tau ** 2
    pos = np.stack([100.0 + 200.0 * tau, -50.0 - 80.0 * tau, z], axis=1)
    pos += noise * rng.randn(*pos.shape)
    return ts, pos


def test_fit_crossing_is_exact_on_a_parabola():
    from jugglebot.motion.skills.magazine_target import fit_descending_crossing
    ts, pos = _parabola()
    zt = 700.0
    t0 = ts[0]
    g = 9810.0
    tc = (3000.0 + np.sqrt(3000.0 ** 2 + 2 * g * 200.0)) / g
    fit = fit_descending_crossing(ts, pos, zt)
    assert fit.t_cross_s - t0 == pytest.approx(tc, abs=1e-6)
    np.testing.assert_allclose(fit.xy_mm, [100.0 + 200.0 * tc, -50.0 - 80.0 * tc], atol=1e-3)


def test_fit_crossing_is_noise_robust_and_honours_t_from():
    from jugglebot.motion.skills.magazine_target import fit_descending_crossing
    ts, pos = _parabola(n=60, noise=2.0)
    g = 9810.0
    tc = (3000.0 + np.sqrt(3000.0 ** 2 + 2 * g * 200.0)) / g
    fit = fit_descending_crossing(ts, pos, 700.0, t_from_s=ts[0] + 0.1)
    assert fit.t_cross_s - ts[0] == pytest.approx(tc, abs=0.01)
    assert fit.n_samples < 60


def test_fit_crossing_refuses_too_few_or_unreachable():
    from jugglebot.motion.skills.magazine_target import fit_descending_crossing
    ts, pos = _parabola()
    with pytest.raises(ValueError, match='usable samples'):
        fit_descending_crossing(ts[:3], pos[:3], 700.0)
    with pytest.raises(ValueError):
        fit_descending_crossing(ts, pos, 5000.0)


def test_quat_to_rotation_round_trips_mocap_node_quaternion():
    from jugglebot.motion.skills.magazine_target import quat_to_rotation
    c, s = np.cos(0.4), np.sin(0.4)
    R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
    q = (0.0, 0.0, np.sin(0.2), np.cos(0.2))
    np.testing.assert_allclose(quat_to_rotation(*q), R, atol=1e-12)


# ── with_overrides (the console's on-the-fly nudge, 2026-10-13) ────────────

def _cfg():
    return mt.MagazineConfig(target_base_mm=np.array([550.0, 300.0, 750.0]),
                             flight_s=0.9, source_path='x.yaml')


def test_overrides_unset_sentinels_keep_the_file_values():
    for t, f in ((None, None), ([0.0, 0.0, 0.0], 0.0), ([0, 0, 0], None), ([], 0.0)):
        cfg = mt.with_overrides(_cfg(), target_base_mm=t, flight_s=f)
        np.testing.assert_array_equal(cfg.target_base_mm, [550.0, 300.0, 750.0])
        assert cfg.flight_s == 0.9 and cfg.source_path == 'x.yaml'


def test_overrides_replace_the_file_values_independently():
    cfg = mt.with_overrides(_cfg(), target_base_mm=[560.0, 290.0, 0.0], flight_s=None)
    np.testing.assert_array_equal(cfg.target_base_mm, [560.0, 290.0, 0.0])
    assert cfg.flight_s == 0.9
    cfg = mt.with_overrides(_cfg(), target_base_mm=None, flight_s=1.1)
    np.testing.assert_array_equal(cfg.target_base_mm, [550.0, 300.0, 750.0])
    assert cfg.flight_s == 1.1


@pytest.mark.parametrize('t, f', [
    ([1.0, 2.0], None), ([1.0, 2.0, float('nan')], None), (['a', 1, 2], None),
    (None, 0.4), (None, 1.31), (None, float('nan')),
])
def test_overrides_are_validated_like_the_file(t, f):
    with pytest.raises(ValueError, match='MAGAZINE_CONFIG'):
        mt.with_overrides(_cfg(), target_base_mm=t, flight_s=f)
