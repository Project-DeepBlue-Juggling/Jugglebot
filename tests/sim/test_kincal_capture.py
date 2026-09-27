"""Tests for the kinematic-calibration capture's pure core
(``tests/hardware/kincal_capture.py``).

Plan: ``plans/active/kinematic-calibration.md`` (§ 5 the sweep, § 3 repeats,
§ 4 re-home, § 6 step 2). The end-to-end test flies the generated sweep on a
synthetic robot with a known geometry error — commanded revolutions from the
config IK, the platform wherever the TRUE geometry puts it — writes the CSV
through the tool's own writer and hands it to ``tools/kincal_fit.py``: the sweep
is judged by whether the fit it feeds passes § 8, not by its shape alone.

Pure numpy, seeded, files only under ``tmp_path`` — safe under xdist.
"""

from __future__ import annotations

import ast
import math
import os
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
for _p in (os.path.join(_REPO_ROOT, 'tools'),
           os.path.join(_REPO_ROOT, 'tests', 'hardware'),
           os.path.join(_REPO_ROOT, 'ros_ws', 'src', 'jugglebot')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import kincal_capture as kc  # noqa: E402
import kincal_fit as kf  # noqa: E402

REACH = kc.Reach()
STEPS = kc.generate_sweep(1, reach=REACH)
REC = [s.target for s in STEPS if s.kind == 'move' and s.target.record]


# ── The sweep's shape (§ 5, § 3, § 4) ───────────────────────────────────────

def test_sweep_content_matches_the_plan():
    s = kc.sequence_summary(STEPS)
    main = [t for t in REC if not t.repeat_group and not t.phase]
    assert len(main) == len(kc.Z_LEVELS_MM) * kc.POSES_PER_LEVEL
    assert s['holdout'] == kc.N_HOLDOUT
    assert s['repeat_groups'] == kc.N_REPEAT
    assert s['pre_home'] == s['post_home'] == kc.N_REHOME
    assert s['rehomes'] == 1
    assert s['eta_min'] <= 30.0            # owner's motion budget (Q11)


def test_every_recorded_pose_is_tilted_reachable_and_in_the_box():
    """§ 5.1: every pose tilted, as far as reach allows."""
    for t in REC:
        assert kc.TILT_MIN_DEG - 1e-9 <= t.tilt_deg <= kc.TILT_MAX_DEG + 1e-9
        assert abs(t.x) <= kc.BOX_MM + 1e-9 and abs(t.y) <= kc.BOX_MM + 1e-9
        assert 100.0 <= t.z <= 250.0
    assert REACH.ok(np.array([t.pose6 for t in REC])).all()
    assert kc.reach_problems(STEPS, REACH) == []


def test_sweep_reaches_toward_the_edge():
    """Edge-weighted: legs near their stroke ends separate the parameters.
    Judged against each pose's OWN reachable radius (its direction and tilt):
    at z = 250 a 10° tilt changes the reach by a factor of two."""
    for z in kc.Z_LEVELS_MM:
        lvl = [t for t in REC if t.z == z and not t.repeat_group and not t.phase]
        frac = np.array([math.hypot(t.x, t.y)
                         / REACH.max_radius(z, math.atan2(t.y, t.x), t.rv)
                         for t in lvl])
        # 0.9 · u^0.4 has median 0.68; the ±300 mm box clips some at z = 100.
        assert np.median(frac) > 0.55, (z, np.median(frac))


def test_repeat_groups_have_two_directions_twice():
    groups = {}
    for t in REC:
        if t.repeat_group:
            groups.setdefault(t.repeat_group, []).append(t)
    for grp, ts in groups.items():
        assert sorted(t.direction for t in ts) == ['A', 'A', 'B', 'B'], grp
        assert len(set(t.pose_id for t in ts)) == 1, grp
    # each repeat arrival is preceded by its own via pose, from opposite sides
    for i, s in enumerate(STEPS):
        t = s.target if s.kind == 'move' else None
        if t is not None and t.repeat_group:
            via = STEPS[i - 1].target
            assert not via.record and via.pose_id == t.pose_id + '_via'


def test_rehome_subset_is_the_same_poses_either_side_of_the_rehome():
    i = next(k for k, s in enumerate(STEPS) if s.kind == 'rehome')
    pre = [s.target.pose_id for s in STEPS[:i] if s.target.phase == 'pre_home']
    post = [s.target.pose_id for s in STEPS[i + 1:] if s.target.phase == 'post_home']
    assert pre == post and len(pre) == kc.N_REHOME


def test_generation_is_deterministic_by_seed():
    again = kc.generate_sweep(1, reach=REACH)
    assert [(s.kind, s.target) for s in again] == [(s.kind, s.target) for s in STEPS]
    other = kc.generate_sweep(2, reach=REACH)
    assert [s.target for s in other][:5] != [s.target for s in STEPS][:5]


def test_check_mode_is_short_and_tilted():
    steps = kc.generate_check(1, reach=REACH)
    s = kc.sequence_summary(steps)
    assert s['recorded'] == 8 and s['eta_min'] < 1.5
    assert s['tilt_deg_min'] >= kc.TILT_MIN_DEG - 1e-9


# ── Checks report every refusal at once ─────────────────────────────────────

def test_reach_problems_names_every_bad_move():
    far = kc.Target('far', 390.0, 0.0, 250.0, (0.1, 0.0, 0.0))
    far2 = kc.Target('far2', -390.0, 0.0, 250.0)
    steps = [kc.Step('move', far), kc.Step('move', far2)]
    probs = kc.reach_problems(steps, REACH)
    assert len(probs) == 3          # into far, far -> far2, far2 -> centre
    assert 'far' in probs[0] and 'far2' in probs[1]


def test_production_gate_accepts_the_capture_moves():
    """The first moves of the sweep through ``planner.build_move`` at the
    durations the capture requests (the full sweep runs in the dry-run)."""
    steps = STEPS[:4]
    assert kc.gate_problems(steps) == []


def test_move_duration_floors():
    a = kc.centre_target()
    assert kc.move_duration_s(a, a) == kc.MIN_MOVE_S
    b = kc.Target('b', 300.0, 0.0, 170.0)
    assert kc.move_duration_s(a, b) == pytest.approx(300.0 / kc.MOVE_SPEED_MMPS)
    c = kc.Target('c', 0.0, 0.0, 170.0, (math.radians(20.0), 0.0, 0.0))
    assert kc.move_duration_s(a, c) == pytest.approx(20.0 / kc.TILT_RATE_DEGPS)


def test_preflight_reports_every_problem_at_once():
    probs = kc.preflight_problems(
        link_kv={}, status_mode='STANDBY', robot_state_age_s=None,
        leg_errors=[0, 4, 0, 0, 0, 0], hand_rev=2.0, platform_age_s=None,
        service_ok=False)
    text = '\n'.join(probs)
    for needle in ('go_to_pose', 'wire', 'STANDBY', 'robot_state', 'axes [1]',
                   'hand at 2.00', 'never seen'):
        assert needle in text
    assert len(probs) == 7


def test_preflight_passes_a_ready_stack():
    kv = {'mpc_active': '1', 'fault_state': 'NONE', 'bridge_link': 'UP'}
    assert kc.preflight_problems(
        link_kv=kv, status_mode='TRAJECTORY', robot_state_age_s=0.01,
        leg_errors=[0] * 6, hand_rev=0.02, platform_age_s=0.01,
        service_ok=True) == []


@pytest.mark.parametrize('kind,remaining,done', [
    ('move', 1.2, False),     # in flight
    ('move', 0.0, True),      # a finished move stays 'move' (2026-09-27 sitting)
    ('hold', 0.0, True),
    ('none', 0.0, True),
    (None, None, False),      # no status yet
])
def test_move_finished_matches_the_node(kind, remaining, done):
    """``trajectory/status`` never flips a finished move to 'hold'; the node's
    own in-flight test is kind == 'move' AND time remaining > 0."""
    assert kc.move_finished(kind, remaining) is done


def test_node_in_flight_test_is_what_move_finished_mirrors():
    src = open(os.path.join(_REPO_ROOT, 'ros_ws', 'src', 'jugglebot', 'jugglebot',
                            'trajectory_node.py'), encoding='utf-8').read()
    i = src.index('def _active_move_in_flight')
    body = src[i:i + 900]
    assert "plan.kind != 'move'" in body
    assert '(plan.total_duration - (time.perf_counter() - t0)) > 0.0' in body


# ── Dwell reduction ─────────────────────────────────────────────────────────

def test_reduce_dwell_means_and_quaternion_sign():
    rng = np.random.default_rng(0)
    q = kf.matrix_to_quat_wxyz(Rotation.from_rotvec([0.1, -0.05, 0.0]).as_matrix()[None])[0]
    Q = np.tile(q, (40, 1))
    Q[::2] *= -1.0                  # the same attitude, both signs
    P = np.array([10.0, 20.0, 700.0]) + rng.normal(0, 0.05, (40, 3))
    V = np.tile(np.arange(6) * 0.5, (40, 1)) + rng.normal(0, 0.0003, (40, 6))
    d = kc.reduce_dwell(P, Q, V)
    assert d.problems == []
    np.testing.assert_allclose(d.quat, q if q[0] >= 0 else -q, atol=1e-12)
    np.testing.assert_allclose(d.pos, [10, 20, 700], atol=0.05)


def test_reduce_dwell_refuses_motion_and_starvation():
    P = np.linspace([0, 0, 700], [2, 0, 700], 40)          # 2 mm drift
    V = np.linspace(np.zeros(6), np.full(6, 0.01), 40)
    Q = np.tile([1.0, 0, 0, 0], (40, 1))
    probs = kc.reduce_dwell(P, Q, V).problems
    assert any('mocap spread' in p for p in probs)
    assert any('encoder spread' in p for p in probs)
    probs = kc.reduce_dwell(P[:3], Q[:3], V).problems
    assert any('mocap Platform samples' in p for p in probs)


# ── Frame sanity check ──────────────────────────────────────────────────────

def _exact_dwell(t):
    c = t.pose6[:3] + [0.0, 0.0, REACH.h0]
    R = kf._exp(np.asarray(t.rv))
    return kc.Dwell(c, kf.matrix_to_quat_wxyz(R[None])[0], REACH.revs(t.pose6),
                    40, 40, 0.1, 0.0005)


def test_frame_check_passes_a_consistent_dwell():
    assert kc.frame_problems(REC[0], _exact_dwell(REC[0]), REACH) == []


@pytest.mark.parametrize('fault,needle', [
    ('no_z_shift', 'from the commanded pose'),
    ('transposed', 'attitude'),
    ('xyzw_order', 'attitude'),
    ('leg_sign', 'encoder revs'),
])
def test_frame_check_catches_each_frame_error(fault, needle):
    t = REC[0]
    d = _exact_dwell(t)
    if fault == 'no_z_shift':
        d.pos = d.pos - [0.0, 0.0, REACH.h0]
    elif fault == 'transposed':
        R = kf.quat_wxyz_to_matrix(d.quat[None])[0]
        d.quat = kf.matrix_to_quat_wxyz(R.T[None])[0]
    elif fault == 'xyzw_order':
        d.quat = np.r_[d.quat[1:], d.quat[0]]
    elif fault == 'leg_sign':
        d.rev = -d.rev
    probs = kc.frame_problems(t, d, REACH)
    assert any(needle in p for p in probs), probs


# ── End to end: the sweep feeds a fit that passes § 8 ───────────────────────

def _perturbed(rng):
    t = kf.nominal_geometry().copy()
    nom = kf.nominal_geometry()
    # Base-joint heights near CAD (owner, 2026-09-27): the fit holds them.
    t.base = t.base + rng.normal(0.0, 1.2, (6, 3)) * [1.0, 1.0, 0.2 / 1.2]
    d = rng.normal(0.0, 1.2, (6, 3)).ravel()
    sk = lambda p: np.array([[0, -p[2], p[1]], [p[2], 0, -p[0]], [-p[1], p[0], 0]])
    A = np.vstack([np.hstack([np.eye(3), -sk(p)]) for p in nom.plat])
    coef, *_ = np.linalg.lstsq(A, d, rcond=None)
    t.plat = nom.plat + (d - A @ coef).reshape(6, 3)
    t.L0 = t.L0 + rng.normal(0.0, 3.0, 6)
    t.k = t.k * (1.0 + rng.normal(0.0, 0.005, 6))
    t.reg = rng.normal(0.0, math.radians(0.3), 3)
    return t


def _fly(steps, truth, rng, *, rehome_dL0=None):
    """Commanded revs from the config IK; the platform where ``truth`` puts
    them; the mocap reading with its registration and 0.1 mm noise."""
    rows = []
    for s in steps:
        if s.kind != 'move' or not s.target.record:
            continue
        t = s.target
        rev = REACH.revs(t.pose6)
        g = truth
        if rehome_dL0 is not None and t.phase == 'post_home':
            g = truth.copy()
            g.L0 = g.L0 + rehome_dL0
        c0 = t.pose6[:3] + [0.0, 0.0, REACH.h0]
        c, R = kf.forward(rev, g, c0, kf._exp(np.asarray(t.rv)))
        R_m = R @ kf._exp(-truth.reg)
        d = kc.Dwell(c + rng.normal(0.0, 0.1, 3), kf.matrix_to_quat_wxyz(R_m[None])[0],
                     rev + rng.normal(0.0, 0.0005, 6), 40, 40, 0.2, 0.001)
        rows.append((t, d))
    return rows


def test_sweep_feeds_a_fit_that_passes(tmp_path):
    rng = np.random.default_rng(7)
    truth = _perturbed(rng)
    rows = _fly(STEPS, truth, rng)
    assert kc.frame_problems(rows[0][0], rows[0][1], REACH) == []
    w = kc.CaptureWriter(str(tmp_path / 'cap.csv'), {'mocap_z_shift_mm': REACH.h0})
    for t, d in rows:
        w.write(kc.csv_row(t, d, hand_rev=0.0, uptime_ms='123'))
    w.close()
    cap = kf.read_capture(str(tmp_path / 'cap.csv'))
    assert len(cap) == len(REC)
    a = kf.analyse(cap)
    assert a['pass'], a['criteria']
    assert a['pose_holdout']['pos_rms_mm'] < 0.3
    assert a['pose_holdout_reference']['pos_rms_mm'] > 3.0
    assert a['repeat']['verdict'] == 'STATIC'
    assert a['rehome']['verdict'] == 'PASS'


def test_a_sloppy_rehome_is_caught(tmp_path):
    rng = np.random.default_rng(8)
    truth = _perturbed(rng)
    rows = _fly(STEPS, truth, rng, rehome_dL0=np.array([0, 2.5, 0, 0, -2.0, 0]))
    cap_path = str(tmp_path / 'cap.csv')
    w = kc.CaptureWriter(cap_path, {})
    for t, d in rows:
        w.write(kc.csv_row(t, d))
    w.close()
    a = kf.analyse(kf.read_capture(cap_path))
    assert a['rehome']['verdict'] == 'FAIL'
    assert a['rehome']['max_abs_dL0_mm'] > 1.5


# ── Hygiene ─────────────────────────────────────────────────────────────────

def test_pure_core_imports_no_ros_at_module_scope():
    src = open(kc.__file__, encoding='utf-8').read()
    for node in ast.parse(src).body:
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            names = ([a.name for a in node.names] if isinstance(node, ast.Import)
                     else [node.module or ''])
            for n in names:
                assert not n.startswith(('rclpy', 'jugglebot_interfaces',
                                         'diagnostic_msgs')), n


def test_csv_header_is_the_fit_contract_plus_extras():
    fit_cols = ['pose_id', 'role', 'x_mm', 'y_mm', 'z_mm', 'qw', 'qx', 'qy', 'qz'] \
        + ['rev_%d' % i for i in range(6)]
    assert all(c in kc.CSV_COLS for c in fit_cols)
    assert len(set(kc.CSV_COLS)) == len(kc.CSV_COLS)


# ── 2026-09-27 sitting: levelling allowance, stalls, planning refusals ──────

def test_reach_allows_for_the_node_levelling_correction():
    """z250_23 of the pre-fix seed-1 sweep: leg 1 at 272.4 mm nominal passed a
    5 mm margin, then the node's ~0.8 deg C-LEVEL-1 tilt put it at 275.2 mm and
    go_to_pose refused it WORKSPACE."""
    pose = np.array([20.9316, 76.4709, 250.0, -0.147246, -0.060055, 0.0])
    assert kc.Reach(level_allowance_deg=0.0).ok(pose)[0]
    assert not REACH.ok(pose)[0]


def test_every_sweep_pose_survives_a_levelling_correction_in_any_direction():
    rng = np.random.default_rng(0)
    for t in REC:
        R = kf._exp(np.asarray(t.rv))
        for psi in rng.uniform(0.0, 2.0 * math.pi, 4):
            a = math.radians(kc.LEVEL_ALLOWANCE_DEG)
            Rg = kf._exp(np.array([a * math.cos(psi), a * math.sin(psi), 0.0]))
            p = t.pose6.copy()
            p[3:] = Rotation.from_matrix(Rg @ R).as_rotvec()
            e = kc.Reach(level_allowance_deg=0.0).extensions(p)[0]
            assert np.all(e >= 0.0) and np.all(e <= REACH.stroke), t.pose_id


def test_tracking_check_passes_a_followed_dwell_and_catches_a_stall():
    t, prev = REC[30], REC[29]
    assert kc.tracking_problems(t, _exact_dwell(t), REACH) == []
    stalled = _exact_dwell(prev)     # legs still where the previous pose left them
    assert 'did not follow' in kc.tracking_problems(t, stalled, REACH)[0]


class _FakeRos:
    """Stands in for ``_Ros``: every move arrives exactly, except the FIRST
    request of each pose id in ``refuse`` (answered with ``code``; a group's vias
    share an id) and, from ``stall_at`` on, a platform frozen where it was."""

    def __init__(self, refuse=(), code='WORKSPACE', stall_at=None):
        self.refuse, self.code, self.stall_at = set(refuse), code, stall_at
        self.link_kv = {'mpc_active': '1', 'fault_state': 'NONE',
                        'bridge_link': 'UP'}
        self.hand_rev, self.uptime_ms = 0.0, 1
        self.at, self.stalled, self.requested = kc.centre_target(), False, []

    def preflight(self):
        return []

    def go(self, t, duration_s):
        self.requested.append(t.pose_id)
        if t.pose_id in self.refuse:
            self.refuse.discard(t.pose_id)
            raise kc.MoveRefused('go_to_pose refused %s: %s x' % (t.pose_id,
                                                                  self.code),
                                 self.code)
        if t.pose_id == self.stall_at:
            self.stalled = True
        if not self.stalled:
            self.at = t
        return 1.0

    def wait_arrival(self, planned_s, settle_s):
        pass

    def sample(self, window_s, z_shift_mm):
        return _exact_dwell(self.at)


def _run(monkeypatch, tmp_path, steps, fake):
    import types
    rclpy = types.SimpleNamespace(init=lambda: None, shutdown=lambda: None,
                                  create_node=lambda name: types.SimpleNamespace(
                                      destroy_node=lambda: None))
    monkeypatch.setitem(sys.modules, 'rclpy', rclpy)
    monkeypatch.setattr(kc, '_Ros', lambda node, timeout_s: fake)
    args = types.SimpleNamespace(mode='sweep', seed=1, timeout_s=1.0,
                                 settle_s=0.0, window_s=0.0)
    stem = str(tmp_path / 'cap')
    rc = kc.run_capture(steps, args, REACH, stem)
    import json
    with open(stem + '_meta.json') as fh:
        meta = json.load(fh)
    cap = kf.read_capture(stem + '.csv') if meta.get('rows') else None
    return rc, meta, cap


def _repeat_slice():
    """The main poses up to and including the first repeat group's four dwells."""
    i = next(j for j, s in enumerate(STEPS) if s.target.repeat_group)
    return STEPS[:i + 7]      # via, A, via, B, via, A, via, B


def test_a_refused_pose_is_skipped_and_the_capture_goes_on(monkeypatch, tmp_path):
    steps = STEPS[:12]
    bad = steps[4].target.pose_id
    rc, meta, cap = _run(monkeypatch, tmp_path, steps, _FakeRos(refuse={bad}))
    assert rc == 0 and meta['abort_reason'] is None
    assert [s['pose_id'] for s in meta['skipped']] == [bad]
    assert len(cap.pose_id) == 11 and bad not in list(cap.pose_id)


def test_a_refused_via_skips_its_dwell_only(monkeypatch, tmp_path):
    steps = _repeat_slice()
    i_via = next(j for j, s in enumerate(steps) if not s.target.record)
    fake = _FakeRos(refuse={steps[i_via].target.pose_id})
    rc, meta, cap = _run(monkeypatch, tmp_path, steps, fake)
    assert rc == 0
    orphan = steps[i_via + 1].target
    assert meta['skipped'] == [{'pose_id': orphan.pose_id, 'phase': '',
                                'repeat_group': orphan.repeat_group,
                                'problems': ['its via was refused']}]
    # The other three repeat dwells of the group were still recorded.
    assert int(np.sum(cap.repeat_group == orphan.repeat_group)) == 3


def test_a_non_planning_refusal_still_aborts(monkeypatch, tmp_path):
    steps = STEPS[:6]
    fake = _FakeRos(refuse={steps[2].target.pose_id}, code='STALE_STATE')
    rc, meta, _ = _run(monkeypatch, tmp_path, steps, fake)
    assert rc == 1 and 'STALE_STATE' in meta['abort_reason']


def test_too_many_refusals_abort(monkeypatch, tmp_path):
    steps = STEPS[:kc.MAX_REFUSALS + 3]
    fake = _FakeRos(refuse={s.target.pose_id for s in steps})
    rc, meta, _ = _run(monkeypatch, tmp_path, steps, fake)
    assert rc == 1 and 'planning refusals' in meta['abort_reason']


def test_a_stalled_platform_aborts_before_recording_it(monkeypatch, tmp_path):
    """The 2026-09-27 run 1 shape: the legs stop mid-sweep with no fault yet;
    mocap and encoders agree with each other, so only the command disagrees."""
    steps = STEPS[:10]
    stall = steps[5].target.pose_id
    rc, meta, cap = _run(monkeypatch, tmp_path, steps, _FakeRos(stall_at=stall))
    assert rc == 1 and 'did not follow' in meta['abort_reason']
    assert len(cap.pose_id) == 5 and stall not in list(cap.pose_id)


@pytest.mark.parametrize('kv,baseline,fires', [
    ({'sched_refused': '50'}, 50, False),       # steady: history, not this move
    ({'sched_refused': '55'}, 50, True),        # the 2026-09-27 13:12:20.6 shape
    ({'sched_refused': '55'}, None, False),     # bridge never published it
    ({}, 50, False),                            # key absent from this message
    ({'sched_refused': 'x'}, 50, False),
])
def test_sched_refused_names_the_latched_hold(kv, baseline, fires):
    why = kc.sched_refused_problem(kv, baseline)
    assert (why is not None) == fires
    if fires:
        assert 'REFUSED' in why and '50 -> 55' in why
