"""Tests for the kinematic-calibration fit (``tools/kincal_fit.py``).

Plan: ``plans/active/kinematic-calibration.md`` (§ 6 step 1: the fit is proven
offline on synthetic data with a known geometry error before any hardware
time). Lives in ``tests/sim/`` beside the other ``tools/`` tests
(``test_tilt_cal_analyse.py``), with the same ``sys.path`` pattern.

The synthetic truth is the config geometry plus a seeded perturbation of every
parameter group; the capture is that truth's exact IK with mocap and encoder
noise added. Pose-space accuracy on held-out rows is what the plan's § 8
criteria judge, so it is what the recovery tests assert on.

Pure numpy, seeded, files only under ``tmp_path`` — safe under xdist.
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np
import pytest
import yaml
from scipy.spatial.transform import Rotation

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
for _p in (os.path.join(_REPO_ROOT, 'tools'),
           os.path.join(_REPO_ROOT, 'ros_ws', 'src', 'jugglebot')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import kincal_fit as kf  # noqa: E402

NOM = kf.nominal_geometry()
H0 = kf.nominal_init_height_mm()


def _skew(p):
    return np.array([[0.0, -p[2], p[1]], [p[2], 0.0, -p[0]], [-p[1], p[0], 0.0]])


def _perturbed(rng, node_sd=1.2, L0_sd=3.0, k_rel=0.005, reg_deg=0.3):
    """A truth geometry inside the fit's gauge (the platform pattern's rigid
    motion projected out, as the fit's constraints require)."""
    t = NOM.copy()
    t.base = t.base + rng.normal(0.0, node_sd, (6, 3))
    d = rng.normal(0.0, node_sd, (6, 3)).ravel()
    A = np.vstack([np.hstack([np.eye(3), -_skew(p)]) for p in NOM.plat])
    coef, *_ = np.linalg.lstsq(A, d, rcond=None)
    t.plat = NOM.plat + (d - A @ coef).reshape(6, 3)
    t.L0 = t.L0 + rng.normal(0.0, L0_sd, 6)
    t.k = t.k * (1.0 + rng.normal(0.0, k_rel, 6))
    t.reg = rng.normal(0.0, math.radians(reg_deg), 3)
    return t


def _poses(rng, g, zs=(100, 150, 200, 250), per=16, tilt_deg=6.0):
    """Reachable poses across the § 5 region, every one tilted."""
    out = []
    for z in zs:
        n = 0
        while n < per:
            r = 380.0 * math.sqrt(rng.uniform())
            a = rng.uniform(0.0, 2.0 * math.pi)
            x, y = r * math.cos(a), r * math.sin(a)
            if abs(x) > 300.0 or abs(y) > 300.0:
                continue
            tv = np.r_[rng.uniform(-1.0, 1.0, 2) * math.radians(tilt_deg), 0.0]
            c = np.array([x, y, H0 + z])
            R = Rotation.from_rotvec(tv).as_matrix()
            ext = kf.leg_lengths(c[None], R[None], g.base, g.plat)[0] - g.L0
            if ext.min() < 5.0 or ext.max() > 275.0:
                continue
            out.append((c, R))
            n += 1
    return out


def _capture(rng, g, poses, *, mocap_sd=0.1, rev_sd=0.0005, roles=None,
             phase=None, repeat_group=None, direction=None, ids=None):
    n = len(poses)
    pos = np.array([c for c, _ in poses])
    R = np.array([R for _, R in poses])
    rev = kf.length_to_rev(kf.leg_lengths(pos, R, g.base, g.plat), g)
    R_m = R @ Rotation.from_rotvec(-g.reg).as_matrix()

    def col(v, default=''):
        return np.array(v if v is not None else [default] * n, dtype=object)

    return kf.Capture(
        pose_id=col(ids if ids is not None else ['p%d' % i for i in range(n)]),
        role=col(roles, 'fit'),
        pos=pos + rng.normal(0.0, mocap_sd, pos.shape),
        quat=kf.matrix_to_quat_wxyz(R_m),
        rev=rev + rng.normal(0.0, rev_sd, rev.shape),
        repeat_group=col(repeat_group), direction=col(direction),
        phase=col(phase))


# ── The model is the production IK ──────────────────────────────────────────

def test_leg_model_matches_production_ik():
    """The fit calibrates ``pose_to_leg_lengths``; if the two ever disagree the
    fitted numbers mean something else in the robot."""
    from jugglebot.motion.geometry import StewartGeometry
    from jugglebot.motion.ik_solver import pose_to_leg_lengths
    geom = StewartGeometry()
    rng = np.random.default_rng(3)
    for _ in range(20):
        pos = np.array([rng.uniform(-150, 150), rng.uniform(-150, 150),
                        rng.uniform(50, 200)])
        R = Rotation.from_rotvec(rng.uniform(-0.1, 0.1, 3)).as_matrix()
        prod = pose_to_leg_lengths(pos, R, geom)
        c = pos + np.array([0.0, 0.0, H0])
        mine = kf.leg_lengths(c[None], R[None], NOM.base, NOM.plat)[0] - NOM.L0
        np.testing.assert_allclose(mine, prod, atol=1e-9)


# ── Recovery of a known geometry error ──────────────────────────────────────

@pytest.fixture(scope='module')
def recovery():
    rng = np.random.default_rng(11)
    truth = _perturbed(rng)
    poses = _poses(rng, truth)
    roles = ['holdout' if i % 8 == 0 else 'fit' for i in range(len(poses))]
    cap = _capture(rng, truth, poses, roles=roles)
    return truth, cap, kf.analyse(cap)


def test_holdout_meets_the_pass_criteria(recovery):
    _, _, a = recovery
    h = a['pose_holdout']
    assert h['n'] >= 8
    assert h['pos_rms_mm'] < 0.5 and h['pos_max_mm'] < 1.0, h
    assert h['att_max_deg'] < 0.05, h
    for name in ('holdout_pos_rms', 'holdout_pos_max', 'holdout_att_max',
                 'nodes_plausible'):
        assert a['criteria'][name], (name, a['criteria'])
    assert a['pass']


def test_the_reference_geometry_fails_the_same_holdout(recovery):
    """Sanity: the injected error is large enough that the fit had work to do."""
    _, _, a = recovery
    assert a['pose_holdout_reference']['pos_rms_mm'] > 3.0


def test_leg_scales_recovered(recovery):
    truth, _, a = recovery
    fitted = kf.KinGeom.from_dict(a['geometry'])
    np.testing.assert_allclose(fitted.k, truth.k, rtol=1e-3)


def test_posterior_sd_is_honest():
    """Parameter-level recovery is NOT exact over a stroke-limited sweep: each
    L0 is near-degenerate with its base node's z (both act along a
    near-vertical leg) and the registration tilt with the six L0 — the fit
    freezes or trades them, and pose-space accuracy (what § 8 judges) is
    unaffected. What must hold is that the reported sd tells the truth: every
    fitted parameter lies within a few sd of the truth, so a report that says
    "pinned to 0.6 mm" means it.
    """
    rng = np.random.default_rng(41)
    truth = _perturbed(rng, reg_deg=0.0)
    cap = _capture(rng, truth, _poses(rng, truth))
    res = kf.fit(cap)
    err = res.geom.to_vector() - truth.to_vector()
    z = np.abs(err[res.free]) / res.sd[res.free]
    assert np.max(z) < 3.5, np.max(z)
    dp, _ = kf.pose_errors(cap, res.geom)
    assert math.sqrt(np.mean(dp ** 2)) < 0.3


def test_registration_yaw_is_identified(recovery):
    """Yaw is pinned by the pattern's gauge. Tilt is only weakly identified
    (near-degenerate with a per-leg L0 change over a stroke-limited sweep) and
    may be frozen — benign, because a constant attitude offset is what
    ``level`` absorbs; the pose-space criteria above hold either way."""
    truth, _, a = recovery
    fitted = kf.KinGeom.from_dict(a['geometry'])
    assert abs(fitted.reg[2] - truth.reg[2]) < math.radians(0.05)
    frozen = {f['param'] for f in a['frozen']}
    assert frozen <= {'reg.x', 'reg.y'}, frozen


def test_platform_pattern_stays_in_gauge(recovery):
    _, _, a = recovery
    d = np.array(a['geometry']['init_plat_nodes_mm']) - NOM.plat
    assert np.max(np.abs(d.sum(axis=0))) < 1e-3
    assert np.max(np.abs(np.cross(NOM.plat, d).sum(axis=0))) < 1e-1


def test_noise_free_fit_is_exact():
    """Pins the model algebra: no noise, no freezing -> zero pose error."""
    rng = np.random.default_rng(5)
    truth = _perturbed(rng)
    cap = _capture(rng, truth, _poses(rng, truth, per=12), mocap_sd=0.0,
                   rev_sd=0.0)
    # A tiny sigma so the data outweighs the CAD prior's (deliberate) pull.
    res = kf.fit(cap, freeze=False, sigma_mm=1e-5)
    assert res.max_mm < 1e-3
    dp, da = kf.pose_errors(cap.subset(np.arange(len(cap)) < 10), res.geom)
    assert dp.max() < 1e-3 and da.max() < 1e-4


def test_unidentifiable_parameters_are_frozen_and_named():
    """A single-height, level-only sweep cannot separate everything: the fit
    must say which parameters it froze instead of reporting a confident
    number the data never saw."""
    rng = np.random.default_rng(7)
    truth = _perturbed(rng)
    cap = _capture(rng, truth, _poses(rng, truth, zs=(170,), per=20,
                                      tilt_deg=0.0))
    res = kf.fit(cap)
    assert res.frozen
    names = kf.param_names()
    for name, _sd in res.frozen:
        assert not res.free[names.index(name)]


# ── Path dependence (§ 3) ───────────────────────────────────────────────────

def _repeat_cap(offsets_by_dir):
    pos, dirs = [], []
    for d, offs in offsets_by_dir.items():
        for o in offs:
            pos.append(np.array([100.0, 0.0, H0 + 170.0]) + np.asarray(o, float))
            dirs.append(d)
    n = len(pos)
    return kf.Capture(pose_id=np.array(['r'] * n, dtype=object),
                      role=np.array(['fit'] * n, dtype=object),
                      pos=np.array(pos), quat=np.tile([1.0, 0, 0, 0], (n, 1)),
                      rev=np.zeros((n, 6)),
                      repeat_group=np.array(['g1'] * n, dtype=object),
                      direction=np.array(dirs, dtype=object),
                      phase=np.array([''] * n, dtype=object))


@pytest.mark.parametrize('offsets, expected', [
    ({'A': [(0, 0, 0), (0.2, 0, 0)], 'B': [(0.5, 0, 0), (0.6, 0, 0)]}, 'STATIC'),
    ({'A': [(0, 0, 0), (0.2, 0, 0)], 'B': [(2.0, 0, 0), (2.1, 0, 0)]}, 'DIRECTIONAL'),
    ({'A': [(0, 0, 0), (1.8, 0, 0)], 'B': [(0.5, 0, 0), (0.1, 0, 0)]}, 'RANDOM'),
])
def test_repeat_verdict_thresholds(offsets, expected):
    worst, groups = kf.repeat_verdicts(_repeat_cap(offsets))
    assert worst == expected and groups[0]['verdict'] == expected


# ── Re-home (§ 4) and the per-session offsets-only re-fit ───────────────────

def _rehome_capture(rng, truth, shift):
    poses = _poses(rng, truth, per=6)
    post = truth.copy()
    post.L0 = post.L0 + shift
    ids = ['p%d' % i for i in range(len(poses))]
    pre = _capture(rng, truth, poses, phase=['pre_home'] * len(poses), ids=ids)
    aft = _capture(rng, post, poses, phase=['post_home'] * len(poses), ids=ids)
    return pre, aft


def _concat(a, b):
    return kf.Capture(**{f: np.concatenate([getattr(a, f), getattr(b, f)])
                         for f in ('pose_id', 'role', 'pos', 'quat', 'rev',
                                   'repeat_group', 'direction', 'phase')})


def test_rehome_detects_a_leg_zero_shift():
    rng = np.random.default_rng(13)
    truth = _perturbed(rng, reg_deg=0.0)
    shift = np.array([0.0, 0.0, 1.5, 0.0, 0.0, 0.0])
    pre, aft = _rehome_capture(rng, truth, shift)
    main = _capture(rng, truth, _poses(rng, truth, per=12))
    cap = _concat(_concat(main, pre), aft)
    res = kf.fit(cap)
    v = kf.rehome_verdict(cap, res)
    assert v['verdict'] == 'FAIL'
    dL0 = np.array(v['dL0_mm'])
    # The homing shift lengthens leg 2's model length at a given count: the
    # re-fit reports it as +1.5 mm on leg 2 and ~0 elsewhere.
    assert abs(dL0[2] - 1.5) < 0.3
    assert np.max(np.abs(np.delete(dL0, 2))) < 0.3


def test_rehome_passes_without_a_shift():
    rng = np.random.default_rng(17)
    truth = _perturbed(rng, reg_deg=0.0)
    pre, aft = _rehome_capture(rng, truth, np.zeros(6))
    main = _capture(rng, truth, _poses(rng, truth, per=12))
    cap = _concat(_concat(main, pre), aft)
    assert kf.rehome_verdict(cap, kf.fit(cap))['verdict'] == 'PASS'


def test_offsets_only_recovers_a_session_homing_shift():
    rng = np.random.default_rng(19)
    truth = _perturbed(rng)
    session = truth.copy()
    shift = np.array([0.8, -0.6, 0.0, 1.2, 0.0, -1.0])
    session.L0 = session.L0 + shift
    cap = _capture(rng, session, _poses(rng, session, per=3))
    a = kf.analyse(cap, offsets_only_from=truth)
    assert a['mode'] == 'offsets-only'
    got = np.array(a['geometry']['init_leg_lengths_mm']) - truth.L0
    np.testing.assert_allclose(got, shift, atol=0.3)
    fitted = [p['param'] for p in a['params'] if p['fitted']]
    assert fitted == ['L0[%d]' % i for i in range(6)]


# ── File formats and CLI ────────────────────────────────────────────────────

def test_capture_csv_round_trip(tmp_path):
    rng = np.random.default_rng(23)
    cap = _capture(rng, NOM, _poses(rng, NOM, per=2),
                   roles=['fit', 'holdout'] * 4,
                   repeat_group=['g', ''] * 4, direction=['A', ''] * 4,
                   phase=['pre_home', ''] * 4)
    path = str(tmp_path / 'cap.csv')
    kf.write_capture(path, cap)
    back = kf.read_capture(path)
    np.testing.assert_allclose(back.pos, cap.pos, atol=1e-4)
    np.testing.assert_allclose(back.rev, cap.rev, atol=1e-6)
    np.testing.assert_allclose(np.abs(back.quat), np.abs(cap.quat), atol=1e-8)
    assert list(back.role) == list(cap.role)
    assert list(back.phase) == list(cap.phase)


def test_capture_rejects_unknown_role_and_missing_columns(tmp_path):
    p = tmp_path / 'bad.csv'
    p.write_text('pose_id,role,x_mm\np0,fit,1\n')
    with pytest.raises(ValueError, match='missing columns'):
        kf.read_capture(str(p))
    rng = np.random.default_rng(29)
    cap = _capture(rng, NOM, _poses(rng, NOM, per=1), roles=['train'] * 4)
    kf.write_capture(str(p), cap)
    with pytest.raises(ValueError, match='unknown role'):
        kf.read_capture(str(p))


def test_cli_writes_report_and_a_reloadable_geometry(tmp_path):
    rng = np.random.default_rng(31)
    truth = _perturbed(rng)
    poses = _poses(rng, truth, per=10)
    cap = _capture(rng, truth, poses,
                   roles=['holdout' if i % 5 == 0 else 'fit'
                          for i in range(len(poses))])
    csv_path = str(tmp_path / 'kincal_capture_test.csv')
    kf.write_capture(csv_path, cap)
    out = tmp_path / 'out'
    assert kf.main([csv_path, '--out-dir', str(out)]) == 0
    for name in ('report.md', 'result.json', 'proposed_geometry.yaml'):
        assert (out / name).is_file()
    doc = yaml.safe_load((out / 'proposed_geometry.yaml').read_text())
    geom = kf.KinGeom.from_dict(doc['geometry'])
    assert geom.base.shape == (6, 3) and geom.L0.shape == (6,)
    # The proposed file is exactly what --offsets-only consumes.
    out2 = tmp_path / 'out2'
    assert kf.main([csv_path, '--offsets-only', '--geometry',
                    str(out / 'proposed_geometry.yaml'),
                    '--out-dir', str(out2)]) == 0
    assert 'offsets-only' in (out2 / 'report.md').read_text()
