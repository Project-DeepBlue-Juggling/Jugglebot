"""JP-14 Centre and orbit (``centre_orbit``, IDEA-14, pattern-progression P9):
the Sites, the compile and its refusals.

Every existing Pattern compiling bit for bit is the golden in
``tests/motion/test_skills_beats.py`` plus the before/after byte-compare in
``logbook/2026-11-25-jp14-centre-and-orbit-compile.md``.  The sim gate runs
and the clearance numbers are in that entry; the node wiring is
``tests/ros/test_skill_node_centre_orbit.py``.

    pytest tests/motion/test_skills_centre_orbit.py -q -p no:cacheprovider
"""
from __future__ import annotations

import math

import numpy as np
import pytest

from jugglebot.motion.skills import nominate as nm
from jugglebot.motion.skills import schedule as sc
from jugglebot.motion.skills import sites as st
from jugglebot.motion.skills.segments import CATCH, REST, THROW


def _pattern(r=100.0, n=8, n_throws=None):
    ss = st.centre_orbit_sites(r, n)
    return sc.Pattern(sites=(ss[0], ss[1]), apex_m=0.95, dwell_s=0.27,
                      n_throws=2 * n + 1 if n_throws is None else n_throws,
                      orbit_sites=ss[1:])


def _throws(sched):
    """``[(ball, release Site name, target Site name, shadow)]`` in order."""
    out = []
    for sk in sched.skills:
        if sk.kind == THROW:
            out.append((sk.ball_id, sk.site.name, sk.target.name,
                        sk.shadow_landing))
        elif sk.kind == CATCH and sk.then_throw is not None:
            tt = sk.then_throw
            out.append((sk.ball_id, tt.release_site_or(sk.site).name,
                        tt.target.name, tt.shadow_landing))
    return out


@pytest.mark.parametrize('r,n', [(95.0, 6), (100.0, 8), (105.0, 12)])
def test_sites_centre_and_a_regular_orbit_from_the_feed_site(r, n):
    ss = st.centre_orbit_sites(r, n)
    assert [s.name for s in ss] == ['C'] + ['O%d' % (k + 1) for k in range(n)]
    assert np.allclose(ss[0].cup_mm[:2], 0.0)
    # O1 is where columns' P1 is: -x, the Site nearest Ball Butler
    assert np.allclose(ss[1].cup_mm[:2], [-r, 0.0])
    for a, b in zip(ss[1:], ss[2:] + ss[1:2]):
        assert math.hypot(*a.cup_mm[:2]) == pytest.approx(r)
        assert math.hypot(*(b.cup_mm[:2] - a.cup_mm[:2])) == pytest.approx(
            2.0 * r * math.sin(math.pi / n))
    # counter-clockwise: O2 is below O1 (angle pi + 2 pi / n)
    assert ss[2].cup_mm[1] < 0.0
    assert all(s.cup_mm[2] == st.CATCH_CUP_Z_MM for s in ss)


def test_the_working_point_chord_is_77_mm():
    ss = st.centre_orbit_sites(100.0, 8)
    assert math.hypot(*(ss[2].cup_mm[:2] - ss[1].cup_mm[:2])) == \
        pytest.approx(76.54, abs=0.01)


@pytest.mark.parametrize('n', [6, 8, 12])
def test_one_lap_ball2_visits_every_site_in_order_and_the_lap_closes(n):
    sched = sc.compile_columns(_pattern(n=n), t0_abs_s=10.0)
    assert sched.pattern == sc.CENTRE_ORBIT == 'centre_orbit'
    th = _throws(sched)
    assert len(th) == 2 * n + 1
    assert len(sched.skills) == 2 * n + 3     # n_throws + 2
    orbit = ['O%d' % (k + 1) for k in range(n)]
    b2 = [t for t in th if t[0] == 1]
    assert [t[1] for t in b2] == orbit                     # released in order
    assert [t[2] for t in b2] == orbit[1:] + orbit[:1]     # aimed one step on
    # Ball 1 never leaves the centre but for the Stop, which lands on Ball 2
    # held at O1 again: the lap is closed
    b1 = [t for t in th if t[0] == 0]
    assert all(t[1] == 'C' for t in b1)
    assert all(t[2] == 'C' and not t[3] for t in b1[:-1])
    assert b1[-1][2] == 'O1' and b1[-1][3] is True
    last_catch = [s for s in sched.skills if s.kind == CATCH][-1]
    assert (last_catch.ball_id, last_catch.site.name) == (1, 'O1')
    rest = sched.skills[-1]
    assert rest.kind == REST and rest.site.name == 'O1'


def test_every_cup_transit_is_the_radius():
    """The cup alternates C and an orbit Site, so every transit is Columns'
    at separation r: CATCH Sites alternate between C and the orbit."""
    sched = sc.compile_columns(_pattern(), t0_abs_s=10.0)
    names = [s.site.name for s in sched.skills if s.kind == CATCH]
    assert all((a == 'C') != (b == 'C') for a, b in zip(names, names[1:]))


def test_timing_is_columns_at_one_apex():
    sched = sc.compile_columns(_pattern(), t0_abs_s=10.0)
    a, b = st.columns_sites(100.0)
    col = sc.compile_columns(sc.Pattern(sites=(a, b), apex_m=0.95,
                                        dwell_s=0.27, n_throws=17),
                             t0_abs_s=10.0)
    assert (sched.flight_s, sched.beat_s, sched.transit_s, sched.dwell_s) == (
        col.flight_s, col.beat_s, col.transit_s, col.dwell_s)
    assert [s.t_abs_s for s in sched.skills] == [s.t_abs_s for s in col.skills]


def test_the_fed_hop_entry_launches_ball1_from_the_feed_site_to_the_centre():
    pat = _pattern()
    tm = sc.columns_ball_timing(0.95, 0.95, 0.27)
    feed = sc.LandingPrior(pos_mm=pat.sites[1].catch_site_mm(),
                           vel_mm_s=np.array([0.0, 0.0, -4000.0]),
                           t_land_abs_s=20.0 + tm.transits[1])
    sched = sc.compile_columns(pat, feed=feed, hop_entry=True)
    first = [s for s in sched.skills if s.kind == THROW][0]
    assert (first.site.name, first.target.name, first.entry_hop) == (
        'O1', 'C', True)
    assert sched.pattern == sc.CENTRE_ORBIT


def test_with_events_keeps_the_label():
    sched = sc.compile_columns(_pattern(), t0_abs_s=10.0)
    again = sched.with_events(sched.source.events)
    assert again.pattern == sc.CENTRE_ORBIT
    assert [s.t_abs_s for s in again.skills] == [s.t_abs_s for s in
                                                  sched.skills]


@pytest.mark.parametrize('r', [94.9, 90.0, 105.1, 150.0, float('nan')])
def test_a_radius_outside_the_range_is_refused_by_name(r):
    with pytest.raises(ValueError, match=st.ORBIT_RADIUS_OUT_OF_RANGE):
        st.centre_orbit_sites(r, 8)
    cfg = nm.NominationConfig(pattern='centre_orbit', apex_m=0.95,
                              separation_mm=r, dwell_s=0.27, orbit_n=8)
    with pytest.raises(ValueError, match=st.ORBIT_RADIUS_OUT_OF_RANGE):
        nm.build_pattern(cfg)


@pytest.mark.parametrize('n', [5, 13, 4, 7.5])
def test_a_step_count_outside_the_range_is_refused_by_name(n):
    with pytest.raises(ValueError, match=st.ORBIT_N_OUT_OF_RANGE):
        st.centre_orbit_sites(100.0, n)


def test_the_range_edges_compile():
    for r in st.ORBIT_RADIUS_RANGE_MM:
        for n in st.ORBIT_N_RANGE:
            assert sc.compile_columns(_pattern(r, n), t0_abs_s=10.0).pattern \
                == sc.CENTRE_ORBIT


def test_pattern_refuses_an_inconsistent_orbit():
    ss = st.centre_orbit_sites(100.0, 8)
    with pytest.raises(ValueError, match='must be sites'):
        sc.Pattern(sites=(ss[0], ss[2]), apex_m=0.95, dwell_s=0.27,
                   n_throws=17, orbit_sites=ss[1:])
    with pytest.raises(ValueError, match='phantom'):
        sc.Pattern(sites=(ss[0], ss[1]), apex_m=0.95, dwell_s=0.27,
                   n_throws=17, orbit_sites=ss[1:], phantom_balls=(0,))
    with pytest.raises(ValueError, match='one Apex'):
        sc.Pattern(sites=(ss[0], ss[1]), apex_m=0.95, dwell_s=0.27,
                   n_throws=17, orbit_sites=ss[1:], apex_m_ball_b=0.9)
    with pytest.raises(ValueError, match='centre Site'):
        sc.Pattern(sites=(ss[0], ss[1]), apex_m=0.95, dwell_s=0.27,
                   n_throws=17, orbit_sites=ss[1:] + ss[:1])


def test_nomination_plans_every_chord_and_the_even_stop():
    cfg = nm.NominationConfig(pattern='centre_orbit', apex_m=0.95,
                              separation_mm=100.0, dwell_s=0.27, orbit_n=10)
    pat = nm.build_pattern(cfg)
    assert pat.n_throws == 22 and len(pat.orbit_sites) == 10
    th = _throws(sc.compile_columns(pat, t0_abs_s=10.0))
    chords = {(t[1], t[2]) for t in th if t[0] == 1 and not t[3]}
    assert chords == {('O%d' % (k + 1), 'O%d' % ((k + 1) % 10 + 1))
                      for k in range(10)}
    assert th[-1] == (1, 'O1', 'C', True)      # Ball 2's Stop onto Ball 1
    assert pat.sites[0].name == 'C' and pat.sites[1].name == 'O1'
    assert nm.build_pattern(cfg, n_throws=40).n_throws == 40
    assert nm.workspace_ok(cfg)
    assert 'centre_orbit' in nm.NOMINATED_PATTERNS
    assert 'centre_orbit' in nm.COLUMNS_FED
    sched = nm.compile_schedule(dataclasses_replace(cfg, entry='hop'))
    assert sched.pattern == sc.CENTRE_ORBIT


def dataclasses_replace(cfg, **kw):
    import dataclasses
    return dataclasses.replace(cfg, **kw)


def test_columns_is_untouched_by_the_orbit_field():
    """``orbit_sites=None`` is Columns: same skills, same label."""
    a, b = st.columns_sites(100.0)
    p0 = sc.Pattern(sites=(a, b), apex_m=0.95, dwell_s=0.27, n_throws=9)
    p1 = sc.Pattern(sites=(a, b), apex_m=0.95, dwell_s=0.27, n_throws=9,
                    orbit_sites=None)
    s0 = sc.compile_columns(p0, t0_abs_s=10.0)
    s1 = sc.compile_columns(p1, t0_abs_s=10.0)
    assert s0.pattern == s1.pattern == 'columns'
    assert repr(s0.skills) == repr(s1.skills)
    assert s0.source.label is None
