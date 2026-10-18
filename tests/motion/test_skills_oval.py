"""The one-ball oval pattern's fold + executor cases.

``OneBallPattern(roles='fixed')`` (plan two-ball-oval § 2.1/2.2): every throw
from ``sites[0]`` (T) to ``sites[1]`` (C), the loaded cup carried C -> T, one
STEADY window per cycle.  This file holds the compiler/fold cases and, below
the marker at the end, the executor cases.

Unmarked and parallel-safe: pure arithmetic.
"""

from __future__ import annotations

import numpy as np
import pytest

from types import SimpleNamespace

import dataclasses

from jugglebot.motion.skills import admissible as adm
from jugglebot.motion.skills import bump as bmp
from jugglebot.motion.skills import executor as ex
from jugglebot.motion.skills import schedule as sc
from jugglebot.motion.skills import sites as st
from jugglebot.motion.trajectory import ballistics_bc
from tests.motion.test_skills_executor import (
    LAND_VEL, T0_ABS, _carried_catch_idx, _fit_landing, _offset_dispatch,
    _offset_dispatch_with_apex_gain)

APEX_M = 0.9
DWELL_S = 0.30


@pytest.fixture
def cols():
    return st.columns_sites(150.0)


def _pat(sites, n_throws=3, **kw):
    base = dict(sites=tuple(sites), apex_m=APEX_M, dwell_s=DWELL_S,
                n_throws=n_throws)
    base.update(kw)
    return sc.OneBallPattern(**base)


def _reload(pattern, t0=1000.0):
    site = pattern.sites[0]
    vel = np.array([4800.0 * np.sin(np.radians(25.0)), 0.0,
                    -4800.0 * np.cos(np.radians(25.0))])
    return sc.compile_reload(site.catch_site_mm(), vel, t0 + 3.0, pattern, t0)


def _both(pattern):
    return (sc.compile_one_ball(pattern, t0_abs_s=0.0), _reload(pattern))


# (a) site / target roles ---------------------------------------------------

@pytest.mark.parametrize('compile_', ['one_ball', 'reload'])
@pytest.mark.parametrize('n', [2, 3, 5])
def test_fixed_roles_throw_from_T_to_C_and_catch_at_C(compile_, n, cols):
    t, c = cols
    pat = _pat((t, c), n_throws=n, roles='fixed')
    sched = (sc.compile_one_ball(pat, 0.0) if compile_ == 'one_ball'
             else _reload(pat))
    throws = [s for s in sched.skills if s.kind == sc.THROW]
    catches = [s for s in sched.skills if s.kind == sc.CATCH]
    assert throws and all(s.site is t and s.target is c for s in throws)
    # reload's held-catch prefix sits at site0; every catch the pattern
    # schedules (and every folded then_throw target) is C.
    pattern_catches = [s for s in catches if s.then_throw is not None
                       or s is catches[-1]]
    assert all(s.site is c for s in pattern_catches)
    assert all(s.then_throw.target is c for s in catches
               if s.then_throw is not None)


# (b) pattern label -----------------------------------------------------------

def test_fixed_roles_label_the_schedule_oval_1ball(cols):
    for sched in _both(_pat(cols, n_throws=3, roles='fixed')):
        assert sched.pattern == 'oval_1ball'


# (c) the cross-site fold -----------------------------------------------------

@pytest.mark.parametrize('n', [2, 3, 6])
def test_fixed_roles_fold_one_window_per_cycle(n, cols):
    t, c = cols
    fixed = sc.compile_one_ball(_pat((t, c), n_throws=n, roles='fixed'), 0.0)
    hop = sc.compile_one_ball(_pat((t, c), n_throws=n), 0.0)
    assert len(fixed.skills) == len(hop.skills) == n + 3
    folded = [s for s in fixed.skills if s.then_throw is not None]
    assert len(folded) == n - 1
    for s in folded:
        assert s.site is c
        assert s.then_throw.release_site is t
        assert s.then_throw.release_site_or(s.site) is t
        assert s.then_throw.target is c


# (d) same-site folds unchanged -----------------------------------------------

@pytest.mark.parametrize('nsites', [1, 2])
def test_same_site_folds_leave_release_site_none(nsites, cols):
    sites = cols[:nsites]
    sched = sc.compile_one_ball(_pat(sites, n_throws=4), 0.0)
    assert sched.pattern == ('self_toss' if nsites == 1 else 'hop')
    folded = [s for s in sched.skills if s.then_throw is not None]
    assert len(folded) == 3
    for s in folded:
        assert s.then_throw.release_site is None
        assert s.then_throw.release_site_or(s.site) is s.site


# (e) validation ---------------------------------------------------------------

def test_roles_must_be_a_known_value(cols):
    with pytest.raises(ValueError, match='roles'):
        _pat(cols, roles='oval')


@pytest.mark.parametrize('nsites', [1, 3])
def test_fixed_roles_need_exactly_two_sites(nsites, cols):
    """1 site and 3 sites both raise the ``roles='fixed' needs exactly 2
    sites`` ValueError from ``OneBallPattern.__post_init__`` (the 3-site tuple
    is the two columns sites plus a repeat of the first)."""
    sites = (tuple(cols) * 2)[:nsites]
    with pytest.raises(ValueError, match='roles'):
        _pat(sites, roles='fixed')


def test_roles_default_is_alternate(cols):
    assert _pat(cols).roles == 'alternate'


# (f) alternate output pinned ---------------------------------------------------

def test_alternate_hop_site_sequence_is_unchanged(cols):
    p1, p2 = cols
    sched = sc.compile_one_ball(_pat((p1, p2), n_throws=3), 0.0)
    assert [s.site.name for s in sched.skills] == [
        'P1', 'P1', 'P2', 'P1', 'P2', 'P2']
    assert [s.kind for s in sched.skills] == [
        sc.REST, sc.THROW, sc.CATCH, sc.CATCH, sc.CATCH, sc.REST]
    assert [s.then_throw.target.name for s in sched.skills
            if s.then_throw is not None] == ['P1', 'P2']


# --- executor cases land here ---

# --- executor cases (release-site threading, design_executor_threading) ---

SEP = 150.0


def _fixed(offset=(0.0, 20.0, 0.0), **kw):
    """The fixed-role fold dispatched against a fake installer: ``(sched, x,
    rows, T, C, k)`` with ``k`` the folded CATCH(C)+THROW(T) index."""
    t, c = st.columns_sites(SEP)
    sched, x, rows = _offset_dispatch(offset, sites=(t, c), roles='fixed', **kw)
    return sched, x, rows, t, c, _carried_catch_idx(sched)


def test_oval_carried_throw_releases_at_T_exactly_and_flies_nothing_back():
    t, c = st.columns_sites(SEP)
    sched, x = _offset_dispatch_with_apex_gain(
        (0.0, 20.0, 0.0), u_apex=0.75, sites=(t, c), roles='fixed')
    k = _carried_catch_idx(sched)
    assert sched.skills[k].site is c
    tt = x._catch_terminal(k, sched.skills[k], x.tracker(0)).then_throw
    assert tt.site_mm.tolist() == t.throw_site_mm().tolist()
    assert tt.target_mm.tolist() == c.catch_site_mm().tolist(), \
        'a release AT T has no offset to fly back (g = 1.2 must not shift it)'
    assert x._release_offset_m(k).tolist() == [0.0, 0.0]


def test_oval_learner_state_is_release_xy_plus_catch_offset():
    sched, x, _rows, t, c, k = _fixed()
    want = [t.cup_mm[0] / 1000.0, t.cup_mm[1] / 1000.0, 0.0, 0.020]
    assert x._u_cache[k][0].tolist() == pytest.approx(want, abs=1e-12)
    pend = [p for p in x._pending_outcomes if p.t_release_s == pytest.approx(
        float(sched.skills[k].then_throw.t_release_abs_s))]
    assert len(pend) == 1
    assert pend[0].x.tolist() == pytest.approx(want, abs=1e-12)
    assert pend[0].release_z_mm == pytest.approx(float(t.throw_site_mm()[2]))
    skill = sched.skills[k]
    moved = _fit_landing(
        pos_mm=np.asarray(c.catch_site_mm(), dtype=float)
        + np.array([0.0, 8.0, 0.0]),
        vel_mm_s=LAND_VEL.copy(), t_land_abs_s=float(skill.t_abs_s))
    x._catch_terminal(k, skill, moved)
    x._register_outcome(k, skill)
    assert x._pending_outcomes[-1].x[3] == pytest.approx(0.008, abs=1e-12)
    assert x._carried_release_mm[k].tolist() == t.throw_site_mm().tolist()


def test_oval_forward_prior_queries_the_row_state():
    t, c = st.columns_sites(SEP)
    seen = []

    def predict(xx, u, y_d):
        seen.append(np.asarray(xx, dtype=float).copy())
        return np.array([y_d[0], y_d[1], y_d[2]])
    learner = SimpleNamespace(
        command=lambda xx, y_d: np.array([y_d[0], y_d[1], y_d[2]]),
        predict=predict)
    sched, x, _r = _offset_dispatch((0.0, 20.0, 0.0), sites=(t, c),
                                    roles='fixed', learner=learner)
    k = _carried_catch_idx(sched)
    x._learned_prior_landing(k + 1, sched.skills[k + 1])
    assert seen, 'the forward prediction never ran'
    want = [t.cup_mm[0] / 1000.0, t.cup_mm[1] / 1000.0, 0.0, 0.020]
    assert seen[-1].tolist() == pytest.approx(want, abs=1e-12)


def test_oval_box_key_is_T_then_C(monkeypatch):
    t, c = st.columns_sites(SEP)
    calls = []

    def spy(boxes, pattern, site_pair, apex_m, *, release_site_xy_mm,
            target_site_xy_mm, **_kw):
        calls.append((pattern, tuple(site_pair), tuple(release_site_xy_mm),
                      tuple(target_site_xy_mm)))
        return None
    monkeypatch.setattr(adm, 'select', spy)
    _offset_dispatch((0.0, 20.0, 0.0), sites=(t, c), roles='fixed', boxes=[])
    assert ('oval_1ball', ('P1', 'P2'), tuple(t.cup_mm[:2]),
            tuple(c.cup_mm[:2])) in calls
    assert all(cl[1] == ('P1', 'P2') for cl in calls), calls


def test_oval_next_catch_prior_flies_from_T():
    sched, x, _rows, t, c, k = _fixed()
    nxt = sched.skills[k + 1]
    tt = sched.skills[k].then_throw
    pos = x._commanded_target_mm(tt.target, tt.y_d)
    from jugglebot.motion.skills import schedule as _sc
    fl = _sc.flight_s(float(tt.y_d[1]))
    lv = ballistics_bc.launch_velocity(t.throw_site_mm(), pos, fl)
    want = ballistics_bc.arrival_velocity(lv, fl)
    got = x._predicted_landing(k + 1, nxt)
    assert got.vel_mm_s.tolist() == pytest.approx(want.tolist(), abs=1e-9)
    hc = x._hand_corrected_landing(k + 1, nxt, 1.0)
    _p, vel2, _t = ballistics_bc.arrival_state_at_z(
        t.throw_site_mm(), lv, float(pos[2]))
    assert hc.vel_mm_s.tolist() == pytest.approx(
        np.asarray(vel2).tolist(), abs=1e-6)


def test_oval_carried_window_and_stop_rest_at_T():
    sched, x, _rows, t, c, k = _fixed()
    skill = sched.skills[k]
    term = x._catch_terminal(k, skill, x.tracker(0))
    assert term.rest_site_mm.tolist() == t.rest_site_mm().tolist()
    t_now = float(sched.skills[1].t_abs_s) + 0.05   # THROW 0 released
    stops = x.stop_terminals(t_now)
    assert [lb for lb, _t in stops] == ['after']
    assert stops[0][1].rest_site_mm.tolist() == t.rest_site_mm().tolist()


def test_stop_mid_carry_declines_the_before_rest_by_name():
    sched, x, _rows, t, c, k = _fixed()
    t_now = float(sched.skills[1].t_abs_s) + 0.05
    assert x.stop_terminals(t_now)
    assert x.stop_before_declined == ex.STOP_BEFORE_CARRIED
    # a same-site pending release (THROW 0) still offers 'before'
    stops = x.stop_terminals(float(sched.skills[1].dispatch_s()) + 0.01)
    assert [lb for lb, _t in stops] == ['before', 'after']
    assert x.stop_before_declined is None


def test_oval_outward_unit_counts_folded_release_sites():
    sched, _x, _rows, t, c, k = _fixed()
    skills = tuple(s for s in sched.skills if s.kind != sc.THROW)
    x2 = ex.SkillExecutor(dataclasses.replace(sched, skills=skills),
                          lambda *a, **kw: None)
    u = x2._outward_unit(skills[[s.then_throw is not None
                                 for s in skills].index(True)])
    assert u is not None and u.tolist() == pytest.approx([1.0, 0.0])


def test_bump_never_runs_on_a_carried_fold():
    assert bmp.bump_enabled_for('oval_1ball') is False
    sched, x, _rows, t, c, k = _fixed()
    x.catch_bump_enabled = True
    skill = sched.skills[k]
    term = x._catch_terminal(k, skill, x.tracker(0))
    x._live_catch = (k, term, 0.0)

    def apply(*a, **kw):
        raise AssertionError('apply must not be reached on a carried fold')
    dec = x._catch_bump_decide(float(skill.t_abs_s) - 0.5, apply, x.tracker(0))
    assert dec.reason == 'BUMP-CARRIED' and dec.terminal
