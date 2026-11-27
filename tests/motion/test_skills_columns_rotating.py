"""JP-11 rotating Columns (``columns_rotating``, IDEA-12): the Sites, the
compile and its refusals (build unit U1; the node, nomination and sim-gate
wiring are U2's).

Plain Columns and every other Pattern compiling bit for bit with
``step_deg`` unset is the golden in ``tests/motion/test_skills_beats.py`` plus
the before/after Schedule dump in
``logbook/2026-11-27-jp11-rotating-columns-compile.md``.

    pytest tests/motion/test_skills_columns_rotating.py -q -p no:cacheprovider
"""
from __future__ import annotations

import math

import numpy as np
import pytest

from jugglebot.motion.skills import schedule as sc
from jugglebot.motion.skills import sites as st
from jugglebot.motion.skills.segments import CATCH, THROW

SEP = 125.0
R = SEP / 2.0
APEX, DWELL = 0.95, 0.27


def _pair():
    p1, p2 = st.columns_sites(SEP)
    return p2, p1            # fed layout: (held Site, Feed Site P1)


def _pattern(step=10.0, n=8, **kw):
    return sc.Pattern(sites=_pair(), apex_m=APEX, dwell_s=DWELL, n_throws=n,
                      step_deg=step, **kw)


def _feed(pattern):
    tm = sc.columns_ball_timing(pattern.apex_for(0), pattern.apex_for(1),
                                pattern.dwell_s)
    return sc.LandingPrior(pos_mm=pattern.sites[1].catch_site_mm(),
                           vel_mm_s=np.array([0.0, 0.0, -4000.0]),
                           t_land_abs_s=10.0 + tm.transits[1])


def _compile(pattern):
    return sc.compile_columns(pattern, feed=_feed(pattern), hop_entry=True)


def _throws(sched):
    """``[(ball, release Site, target Site)]`` in Throw order."""
    out = []
    for sk in sched.skills:
        if sk.kind == THROW:
            out.append((sk.ball_id, sk.site, sk.target))
        elif sk.kind == CATCH and sk.then_throw is not None:
            tt = sk.then_throw
            out.append((sk.ball_id, tt.release_site_or(sk.site), tt.target))
    return out


def _xy(site):
    return np.asarray(site.cup_mm[:2], dtype=float)


def _expected(k, step):
    """S(k) from the design formula: angle 180 + (k - 1) * (step + 180) deg."""
    a = math.radians(180.0 + (k - 1) * (step + 180.0))
    return R * np.array([math.cos(a), math.sin(a)])


def test_the_first_five_throws_at_ten_degrees_per_beat():
    sched = _compile(_pattern(10.0, n=8))
    th = _throws(sched)
    assert [b for b, _, _ in th] == [0, 1, 0, 1, 0, 1, 0, 1]
    # Throw 0 is the hop entry: leaves the Feed Site P1 (-62.5, 0), aimed S(2)
    assert th[0][1].name == 'P1'
    assert np.allclose(_xy(th[0][1]), [-R, 0.0])
    for k in range(1, 5):
        assert np.allclose(_xy(th[k][1]), _expected(k, 10.0), atol=1e-9), k
    for k in range(0, 5):
        assert np.allclose(_xy(th[k][2]), _expected(k + 2, 10.0), atol=1e-9), k
    # the numbers of the logbook walk: S(2) = (61.55, 10.85), S(3) = (-58.73, -21.38)
    assert np.allclose(_xy(th[0][2]), [61.5505, 10.8530], atol=1e-3)
    assert np.allclose(_xy(th[1][2]), [-58.7308, -21.3763], atol=1e-3)
    # a Throw leaves the Site its ball was caught at (Throw k+2 leaves S(k+2))
    for k in range(0, 4):
        assert th[k + 2][1].name == th[k][2].name


@pytest.mark.parametrize('step', [5.0, 10.0, 20.0])
def test_chord_and_transit_lengths_match_the_design_formulas(step):
    th = _throws(_compile(_pattern(step, n=12)))
    chord = 2.0 * R * math.sin(math.radians(step))
    transit = 2.0 * R * math.cos(math.radians(step) / 2.0)
    for k in range(1, 9):              # Throw 0 is the hop, the last is the Stop
        _, rel, tgt = th[k]
        assert np.linalg.norm(_xy(tgt) - _xy(rel)) == pytest.approx(chord)
        # the cup transit to the next Throw's Site (the other ball's)
        nxt = th[k + 1][1]
        assert np.linalg.norm(_xy(nxt) - _xy(rel)) == pytest.approx(transit)
    assert {round(2 * R * math.sin(math.radians(d)), 1)
            for d in (5.0, 10.0, 20.0)} == {10.9, 21.7, 42.8}


@pytest.mark.parametrize('n', [2, 3, 8, 9])
def test_the_stop_lands_where_the_previous_ball_is_held(n):
    th = _throws(_compile(_pattern(10.0, n=n)))
    assert th[-1][2].name == th[-2][2].name
    assert np.array_equal(th[-1][2].cup_mm, th[-2][2].cup_mm)
    sched = _compile(_pattern(10.0, n=n))
    stop = [sk for sk in sched.skills if sk.kind == THROW
            or (sk.kind == CATCH and sk.then_throw is not None)][-1]
    shadow = (stop.shadow_landing if stop.kind == THROW
              else stop.then_throw.shadow_landing)
    assert shadow is True


def test_hop_entry_is_anchored_at_the_feed_site():
    pat = _pattern(10.0)
    sched = _compile(pat)
    first = sched.skills[0]
    assert first.kind == THROW and first.entry_hop is True
    assert first.site is pat.sites[1] and first.site.name == 'P1'
    # ball 1's inflight Catch is at P1 too: Throw 1 leaves it
    th = _throws(sched)
    assert th[1][1].name == 'P1'
    assert sched.pattern == sc.COLUMNS_ROTATING == 'columns_rotating'


@pytest.mark.parametrize('step', [5.0, 10.0, 20.0])
def test_timing_is_field_identical_to_columns_at_the_same_apex_and_dwell(step):
    rot = _compile(_pattern(step, n=12))
    p1, p2 = st.columns_sites(SEP)
    col = _compile(sc.Pattern(sites=(p2, p1), apex_m=APEX, dwell_s=DWELL,
                              n_throws=12))
    assert (rot.flight_s, rot.beat_s, rot.transit_s, rot.dwell_s) == (
        col.flight_s, col.beat_s, col.transit_s, col.dwell_s)
    assert len(rot.skills) == len(col.skills)
    for a, b in zip(rot.skills, col.skills):
        assert (a.kind, a.ball_id, a.t_abs_s, a.window_s, a.lead_s) == (
            b.kind, b.ball_id, b.t_abs_s, b.window_s, b.lead_s)


@pytest.mark.parametrize('step', [None, 0, 0.0])
def test_no_step_is_plain_columns_bit_for_bit(step):
    p1, p2 = st.columns_sites(SEP)
    kw = dict(sites=(p2, p1), apex_m=APEX, dwell_s=DWELL, n_throws=9)
    base = _compile(sc.Pattern(**kw))
    got = _compile(sc.Pattern(step_deg=step, **kw))
    assert repr(got.skills) == repr(base.skills) and got.pattern == 'columns'


def test_site_names_are_unique_per_site():
    th = _throws(_compile(_pattern(10.0, n=40)))     # past one full turn
    where = {}
    for _, rel, tgt in th:
        for x in (rel, tgt):
            assert where.setdefault(x.name, tuple(x.cup_mm)) == tuple(x.cup_mm)
    # and no two names share a place within the run (a name per Site)
    assert len(set(where.values())) == len(where)
    assert 'P1' in where


@pytest.mark.parametrize('bad', [-5.0, 20.5, 45.0, float('nan'), float('inf')])
def test_a_step_outside_zero_to_twenty_is_refused_by_name(bad):
    with pytest.raises(ValueError, match='ROTATING_STEP_OUT_OF_RANGE'):
        _pattern(bad)


def test_twenty_is_the_top_of_the_range():
    _compile(_pattern(20.0))


def test_an_unequal_apex_pair_is_refused_by_name():
    with pytest.raises(ValueError, match='ROTATING_APEX_PAIR_UNEQUAL'):
        _pattern(10.0, apex_m_ball_b=0.9)


def test_a_phantom_ball_is_refused_by_name():
    with pytest.raises(ValueError, match='ROTATING_PHANTOM_REFUSED'):
        _pattern(10.0, phantom_balls=(0,))


def test_a_t0_start_is_refused_by_name():
    pat = _pattern(10.0)
    with pytest.raises(ValueError, match='ROTATING_NEEDS_HOP_ENTRY'):
        sc.compile_columns(pat, t0_abs_s=10.0)


def test_a_pair_that_is_not_a_columns_pair_is_refused_by_name():
    p1, _ = st.columns_sites(SEP)
    other = st.Site('X', np.array([30.0, 40.0, st.CATCH_CUP_Z_MM]))
    with pytest.raises(ValueError, match='ROTATING_NOT_A_PAIR'):
        sc.Pattern(sites=(other, p1), apex_m=APEX, dwell_s=DWELL, n_throws=4,
                   step_deg=10.0)


def test_rotating_with_an_orbit_is_refused_by_name():
    ss = st.centre_orbit_sites(100.0, 8)
    with pytest.raises(ValueError, match='ROTATING_WITH_ORBIT'):
        sc.Pattern(sites=(ss[0], ss[1]), apex_m=APEX, dwell_s=DWELL,
                   n_throws=10, orbit_sites=ss[1:], step_deg=10.0)
