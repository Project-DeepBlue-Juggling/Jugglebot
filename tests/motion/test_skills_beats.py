"""``motion/skills/schedule`` -- the per-beat event stream (plan
``learner-generalisation`` P4: D-P4-1..3; tests T-R1, T-U9, T-U10).

T-R1 is the Columns golden: ``tests/fixtures/columns_schedule_golden_2026-11-06.json``
was written by ``tools/probes/columns_golden.py --write`` on the tree BEFORE
``compile_columns`` became ``compile_beats(columns_events(...))`` (branch
p4-beat-events at 0b60a7a6) and is never regenerated: every case is recompiled
through the public compilers and compared field for field, exactly (floats as
``float.hex``).  A difference means the compiler moved, not the fixture.

Unmarked and parallel-safe: pure arithmetic, the fixture is read only.
"""

from __future__ import annotations

import dataclasses
import importlib.util
import json
import math
import os

import numpy as np
import pytest

from jugglebot.motion.skills import schedule as sc
from jugglebot.motion.skills import sites as st

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(
    __file__))))
_GOLDEN_PY = os.path.join(_REPO, 'tools', 'probes', 'columns_golden.py')


def _load_golden_module():
    spec = importlib.util.spec_from_file_location('columns_golden', _GOLDEN_PY)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


cg = _load_golden_module()

with open(cg.FIXTURE) as _fh:
    _GOLDEN = json.load(_fh)['cases']


# ── T-R1: the golden ─────────────────────────────────────────────────────────

def test_golden_fixture_has_every_case_of_the_matrix():
    """The fixture and the generator's matrix name the same cases, in order
    (a case silently dropped from either would leave it unpinned)."""
    assert [c['name'] for c in _GOLDEN] == [n for n, _ in cg.matrix()]
    assert len(_GOLDEN) == 227      # 201 first written + 26 added after the P4 audit


@pytest.mark.parametrize('case', _GOLDEN, ids=[c['name'] for c in _GOLDEN])
def test_columns_golden_field_for_field(case):
    """T-R1: today's compile of the case equals the pre-refactor compile in
    every field of every Skill / ThenThrow / LandingPrior / Schedule, floats
    bit for bit; a refused case refuses with the same message."""
    got = cg.result_of(case['inputs'])
    diff = cg.first_difference(case['result'], got)
    assert diff is None, 'case %r differs at %s: fixture %r, now %r' % (
        case['name'], diff[0], diff[1], diff[2])


def test_golden_serialiser_walks_every_schedule_field_but_source():
    """The field walk is by ``dataclasses.fields``: a Schedule field added
    later is a visible difference.  ``source`` is the one skipped field."""
    pat = cg.pattern_of(cg._inputs(n=3))
    s = cg.ser(sc.compile_columns(pat, t0_abs_s=10.0))
    names = {f.name for f in dataclasses.fields(sc.Schedule)}
    assert set(s) - {'__type__'} == names - {'source'}


# ── D-P4-1: the stream, the source, with_events ──────────────────────────────

def _pattern(n=6, apex=1.0, apex_b=None, dwell=0.27, phantom=(), sep=100.0):
    return sc.Pattern(sites=st.columns_sites(sep), apex_m=apex, dwell_s=dwell,
                      n_throws=n, phantom_balls=phantom, apex_m_ball_b=apex_b)


def _feed(pattern, t_land=1789263419.8125):
    return sc.LandingPrior(pos_mm=pattern.sites[1].catch_site_mm(),
                           vel_mm_s=np.array([310.0, -120.0, -4150.0]),
                           t_land_abs_s=t_land)


_SOURCE_CASES = [
    dict(pattern=dict(n=6), t0=10.0, feed=False, hop=False),
    dict(pattern=dict(n=1), t0=10.0, feed=False, hop=False),
    dict(pattern=dict(n=8, apex=1.05, apex_b=0.95), t0=1789263419.5,
         feed=False, hop=False),
    dict(pattern=dict(n=5, apex=0.9, apex_b=1.0, dwell=0.35), t0=None,
         feed=True, hop=True),
    dict(pattern=dict(n=4, phantom=(0,)), t0=None, feed=True, hop=True),
    dict(pattern=dict(n=5, phantom=(1,)), t0=None, feed=True, hop=False),
]


def _compile(case):
    pat = _pattern(**case['pattern'])
    if case['feed']:
        return pat, sc.compile_columns(pat, feed=_feed(pat),
                                       hop_entry=case['hop'])
    return pat, sc.compile_columns(pat, t0_abs_s=case['t0'])


@pytest.mark.parametrize('case', _SOURCE_CASES)
def test_compile_columns_source_is_its_columns_events(case):
    pat, sched = _compile(case)
    assert sched.source is not None
    assert sched.source.events == sc.columns_events(pat, hop_entry=case['hop'])
    assert len(sched.source.events) + 2 == len(sched.skills)


@pytest.mark.parametrize('case', _SOURCE_CASES)
def test_with_events_of_the_own_source_reproduces_the_schedule(case):
    _, sched = _compile(case)
    again = sched.with_events(sched.source.events)
    assert cg.ser(again) == cg.ser(sched)


def test_source_takes_no_part_in_equality_or_repr():
    f = {x.name: x for x in dataclasses.fields(sc.Schedule)}['source']
    assert f.compare is False and f.repr is False and f.default is None


def test_reload_and_rest_columns_carry_no_source():
    """Their lead-in is not in the stream, so recompiling from the columns
    tail's events could not reproduce them: ``source`` is None and
    ``with_events`` refuses by name."""
    for name in ('reload_columns n4', 'rest_columns n5'):
        inp = dict(cg.matrix())[name]
        sched = cg.compile_case(inp)
        assert sched.source is None
        with pytest.raises(ValueError, match=sc.BEAT_NOT_SUPPORTED):
            sched.with_events(())


def test_columns_events_keep_the_pattern_refusals():
    with pytest.raises(ValueError, match='leaves no transit'):
        sc.columns_events(_pattern(apex=0.3, dwell=0.6))
    with pytest.raises(ValueError, match='n_throws must be >= 1'):
        sc.columns_events(_pattern(n=0))


# ── T-U10: yaw is dormant; the carried Throw is refused ──────────────────────

def _ev(apex=0.9, dwell=0.27, ball=0, **kw):
    s = st.columns_sites(100.0)
    return sc.BeatEvent(throw_site=s[ball], catch_site=s[ball], apex_m=apex,
                        dwell_s=dwell, ball_id=ball, **kw)


def test_beat_event_refuses_a_non_zero_yaw_by_name():
    with pytest.raises(ValueError, match=sc.BEAT_YAW_NOT_SUPPORTED):
        _ev(yaw_rad=0.1)
    with pytest.raises(ValueError, match=sc.BEAT_YAW_NOT_SUPPORTED):
        _ev(yaw_rad=-1e-300)
    assert _ev(yaw_rad=0.0).yaw_rad == 0.0
    neg = _ev(yaw_rad=-0.0)
    assert neg.yaw_rad == 0.0 and math.copysign(1.0, neg.yaw_rad) == -1.0


def test_compile_beats_refuses_a_carried_release_by_name():
    events = list(sc.columns_events(_pattern(n=4, apex=0.9)))
    events[2] = dataclasses.replace(events[2], release_kind='carried')
    with pytest.raises(ValueError, match=sc.BEAT_CARRIED_NOT_SUPPORTED):
        sc.compile_beats(tuple(events), launch_s=0.4, rest_tail_s=0.3,
                         t0_abs_s=10.0)


# ── T-U9: the next-beat rule, an accept / refuse table ───────────────────────
#
# Every row is computed by hand: t_r = own + d, tau = inflight - t_r, all
# values dyadic so the arithmetic is exact.  `tau_min` rows pass 0.125 (exact)
# so "exactly at the floor" is a float equality, not a hope.

_F09 = sc.flight_s(0.9)
_NEXT_X = 1.25 + _F09                     # this Catch, own 1.0 + d 0.25 + f
_NEXT_Y = _NEXT_X - 0.125                 # exact (Sterbenz)
_NEXT_D_AT = _NEXT_Y - 1.5                # exact: fl(1.5 + d) == _NEXT_Y
_NEXT_D_BELOW = np.nextafter(_NEXT_Y, np.inf) - 1.5

_W = sc.MIN_WINDOW_S

# (row id, own, inflight, event kw, next_dwell, tau_min, ok, code, t_r, tau)
_TABLE = [
    ('ok', 1.0, 1.5, dict(dwell=0.25), None, None, True, sc.BEAT_OK,
     1.25, 0.25),
    ('dwell floor exactly', 1.0, 2.0, dict(dwell=_W), None, None, True,
     sc.BEAT_OK, 1.0 + _W, 2.0 - (1.0 + _W)),
    ('dwell one float below', 1.0, 2.0,
     dict(dwell=float(np.nextafter(_W, 0.0))), None, None, False,
     sc.BEAT_DWELL_TOO_SHORT, 1.0 + float(np.nextafter(_W, 0.0)),
     2.0 - (1.0 + float(np.nextafter(_W, 0.0)))),
    ('dwell too short', 1.0, 1.5, dict(dwell=0.0625), None, None, False,
     sc.BEAT_DWELL_TOO_SHORT, 1.0625, 0.4375),
    ('no transit (default tau_min)', 1.0, 1.3125, dict(dwell=0.25), None,
     None, False, sc.BEAT_NO_TRANSIT, 1.25, 0.0625),
    ('transit over default tau_min', 1.0, 1.375, dict(dwell=0.25), None,
     None, True, sc.BEAT_OK, 1.25, 0.125),
    ('transit exactly tau_min', 1.0, 1.375, dict(dwell=0.25), None, 0.125,
     True, sc.BEAT_OK, 1.25, 0.125),
    ('transit one float below tau_min', 1.0, float(np.nextafter(1.375, 0.0)),
     dict(dwell=0.25), None, 0.125, False, sc.BEAT_NO_TRANSIT, 1.25,
     float(np.nextafter(1.375, 0.0)) - 1.25),
    ('negative transit', 1.0, 1.0, dict(dwell=0.25), None, None, False,
     sc.BEAT_NO_TRANSIT, 1.25, -0.25),
    ('next transit ok', 1.0, 1.5, dict(dwell=0.25), 0.27, None, True,
     sc.BEAT_OK, 1.25, 0.25),
    ('next no transit', 1.0, 1.5, dict(dwell=0.25), 0.55, None, False,
     sc.BEAT_NEXT_NO_TRANSIT, 1.25, 0.25),
    ('next transit exactly tau_min', 1.0, 1.5, dict(dwell=0.25), _NEXT_D_AT,
     0.125, True, sc.BEAT_OK, 1.25, 0.25),
    ('next transit one float below', 1.0, 1.5, dict(dwell=0.25),
     float(_NEXT_D_BELOW), 0.125, False, sc.BEAT_NEXT_NO_TRANSIT, 1.25, 0.25),
]


@pytest.mark.parametrize('row', _TABLE, ids=[r[0] for r in _TABLE])
def test_next_beat_table(row):
    _, own, inflight, kw, nxt, tau_min, ok, code, t_r, tau = row
    ev = _ev(**kw)
    extra = {} if tau_min is None else {'tau_min_s': tau_min}
    dec = sc.next_beat(own, inflight, ev, next_dwell_s=nxt, **extra)
    assert (dec.ok, dec.code) == (ok, code), dec.detail
    assert dec.t_release_s == t_r
    assert dec.transit_s == tau
    assert (dec.detail == '') == ok


def test_next_beat_boundary_rows_are_what_they_claim():
    """The NEXT boundary rows' transit is exactly / one float under 0.125."""
    assert _NEXT_X - (1.5 + _NEXT_D_AT) == 0.125
    assert _NEXT_X - (1.5 + float(_NEXT_D_BELOW)) < 0.125
    assert float(np.nextafter(1.375, 0.0)) - 1.25 < 0.125


def test_next_beat_refuses_a_yaw_by_name():
    """A BeatEvent cannot be BUILT with a yaw; this forces one past the
    constructor (object.__setattr__ on the frozen instance) to pin that the
    rule refuses it too, first."""
    ev = _ev(dwell=0.01)
    object.__setattr__(ev, 'yaw_rad', 0.2)
    dec = sc.next_beat(1.0, 1.5, ev)
    assert (dec.ok, dec.code) == (False, sc.BEAT_YAW_NOT_SUPPORTED)


def test_default_tau_min_is_the_four_knot_window_floor():
    assert sc.MIN_WINDOW_S == pytest.approx(0.1, abs=1e-15)
    ev = _ev(dwell=0.25)
    assert sc.next_beat(1.0, 1.25 + 0.0999, ev).code == sc.BEAT_NO_TRANSIT
    assert sc.next_beat(1.0, 1.25 + 0.1001, ev).ok


# ── The general path of compile_beats ────────────────────────────────────────

def _six_throw_stream():
    """6 Throws at apex 1.0, dwell 0.27; Throws 3.. at apex 0.9 (no re-centring:
    a raw non-periodic stream)."""
    ev = list(sc.columns_events(_pattern(n=6, apex=1.0, dwell=0.27)))
    for k in range(3, 6):
        ev[k] = dataclasses.replace(ev[k], apex_m=0.9)
    return tuple(ev)


#: The recursion by hand for `_six_throw_stream` (relative clock):
#:   f1 = flight(1.0), f9 = flight(0.9), d = 0.27, beta = (f1 + d)/2
#:   r = [0, beta, 2 beta, 3 beta, 4 beta, (3 beta + f9) + d]
#:   c = [f1, beta + f1, 2 beta + f1, 3 beta + f9, 4 beta + f9, r5 + f9]
#: (r3, r4 keep the legacy form: their Dwell and the Catch they follow are the
#: opening ones; r5 follows c3, whose flight changed.)  Literal values:
_R6 = [0.0, 0.5866157228957298, 1.1732314457914597, 1.7598471686871895,
       2.3464628915829193, 2.8867277555835655]
_C6 = [0.9032314457914598, 1.4898471686871897, 2.0764628915829193,
       2.6167277555835655, 3.203343478479295, 3.7436083424799413]


def test_general_path_event_table_follows_the_recursion():
    f1, f9, d = sc.flight_s(1.0), sc.flight_s(0.9), 0.27
    beta = (f1 + d) / 2
    r = [0.0, beta, 2 * beta, 3 * beta, 4 * beta, (3 * beta + f9) + d]
    c = [f1, beta + f1, 2 * beta + f1, 3 * beta + f9, 4 * beta + f9,
         ((3 * beta + f9) + d) + f9]
    assert r == pytest.approx(_R6, abs=1e-12)
    assert c == pytest.approx(_C6, abs=1e-12)
    t0 = 10.0
    sched = sc.compile_beats(_six_throw_stream(), launch_s=0.4,
                             rest_tail_s=0.3, t0_abs_s=t0)
    sk = sched.skills
    assert [s.kind for s in sk] == ([sc.THROW] + [sc.CATCH] * 6 + [sc.REST])
    releases = [sk[0].t_abs_s] + [s.then_throw.t_release_abs_s
                                  for s in sk[1:6]]
    assert np.allclose(np.array(releases) - t0, _R6, atol=1e-12, rtol=0)
    # Skill k (1 <= k <= 5) carries Throw k; Skill 1 is ball 1's first Catch.
    catches = [s.t_abs_s - t0 for s in sk[2:7]]
    assert np.allclose(catches, _C6[:5], atol=1e-12, rtol=0)
    # The folded CATCH carrying Throw 3 carries the CHANGED Throw.
    tt3 = sk[3].then_throw
    assert tt3.y_d[1] == 0.9 and sk[3].ball_id == 1
    assert tt3.t_release_abs_s - t0 == pytest.approx(_R6[3], abs=1e-12)
    assert sk[2].then_throw.y_d[1] == 1.0
    # The windows are the differences: a CATCH's is the transit from the
    # release before it, a carried Throw's is its Dwell.
    rel = sorted(releases)
    for s in sk[1:7]:
        prev = max(x for x in rel if x < s.t_abs_s)
        assert s.window_s == pytest.approx(s.t_abs_s - prev, abs=1e-12)
    assert sk[5].window_s == pytest.approx(_C6[3] - _R6[4], abs=1e-12)
    # Every CATCH still gets the handoff lead; dispatch stays monotone.
    assert all(s.lead_s == sc.HANDOFF_LEAD_S for s in sk if s.kind == sc.CATCH)
    disp = [s.dispatch_s() for s in sk]
    assert all(b >= a - 1e-9 for a, b in zip(disp, disp[1:]))
    # The Stop is still the last Throw, cross-site, shadow_landing.
    assert sk[5].then_throw.shadow_landing
    assert sk[5].then_throw.target is sched.source.events[4].catch_site


def test_general_path_refuses_a_stream_with_no_transit_naming_the_numbers():
    ev = list(_six_throw_stream())
    ev[3] = dataclasses.replace(ev[3], dwell_s=0.6)
    with pytest.raises(ValueError) as exc:
        sc.compile_beats(tuple(ev), launch_s=0.4, rest_tail_s=0.3,
                         t0_abs_s=10.0)
    msg = str(exc.value)
    assert msg.startswith(sc.BEAT_NO_TRANSIT + ': Throw 3')
    assert 'transit -0.' in msg and 'under tau_min 0.1000 s' in msg


@pytest.mark.parametrize('apex,apex_b,dwell', [(0.9, None, 0.27),
                                               (1.05, 0.95, 0.27),
                                               (0.9, 1.0, 0.35)])
def test_a_periodic_stream_down_the_general_path_agrees_with_legacy(
        apex, apex_b, dwell):
    """A 2-periodic stream forced through the recursion (``general=True``)
    gives the legacy table to 1e-12 s.  NOT bit for bit (``i * beta`` vs a
    sum of Dwells and flights): which is why the compiler keeps the legacy
    expressions while the inputs are the opening ones."""
    ev = sc.columns_events(_pattern(n=20, apex=apex, apex_b=apex_b,
                                    dwell=dwell))
    pre, timing, d_cmd = sc._opening(ev, None)
    leg = sc._beat_times(ev, pre, timing, d_cmd)
    gen = sc._beat_times(ev, pre, timing, d_cmd, general=True)
    assert leg.first_deviation is None and gen.first_deviation == 0
    assert np.allclose(gen.t_r, leg.t_r, atol=1e-12, rtol=0)
    assert np.allclose(gen.t_c, leg.t_c, atol=1e-12, rtol=0)
    assert np.allclose(gen.windows, leg.windows, atol=1e-12, rtol=0)
    assert abs(gen.t_init - leg.t_init) <= 1e-12


def test_opening_pair_off_one_cycle_is_refused_by_name():
    ev = list(sc.columns_events(_pattern(n=4)))
    ev[1] = dataclasses.replace(ev[1], dwell_s=0.30)
    with pytest.raises(ValueError, match=sc.BEAT_MISMATCH):
        sc.compile_beats(tuple(ev), launch_s=0.4, rest_tail_s=0.3,
                         t0_abs_s=10.0)


def test_a_stream_the_compiler_does_not_build_is_refused_by_name():
    s = st.columns_sites(100.0)
    ev = list(sc.columns_events(_pattern(n=4)))
    bad_ball = list(ev)
    bad_ball[2] = dataclasses.replace(ev[2], ball_id=1)
    bad_stop = list(ev)
    bad_stop[3] = dataclasses.replace(ev[3], catch_site=s[1])
    for bad in (bad_ball, bad_stop, ()):
        with pytest.raises(ValueError, match=sc.BEAT_STREAM_INVALID):
            sc.compile_beats(tuple(bad), launch_s=0.4, rest_tail_s=0.3,
                             t0_abs_s=10.0)


# ── D-P4-3: columns_change ───────────────────────────────────────────────────

def _times(events):
    pre, timing, d_cmd = sc._opening(events, None)
    return sc._beat_times(events, pre, timing, d_cmd)


def _transits(bt):
    """tau of Throw i >= 1: the in-flight ball's Catch minus this release."""
    return [None] + [bt.t_c[i - 1] - bt.t_r[i] for i in range(1, len(bt.t_r))]


_CHANGES = [
    # (id, pattern kw, change kw, new (apex A, apex B, dwell))
    ('apex 1.0 -> 0.9', dict(apex=1.0), dict(apex_m=0.9), (0.9, 0.9, 0.27)),
    ('dwell 0.27 -> 0.32', dict(apex=1.0), dict(dwell_s=0.32),
     (1.0, 1.0, 0.32)),
    ('high-low 1.0 -> 1.05 / 0.95', dict(apex=1.0),
     dict(apex_m=1.05, apex_m_ball_b=0.95), (1.05, 0.95, 0.27)),
]


@pytest.mark.parametrize('change', _CHANGES, ids=[c[0] for c in _CHANGES])
@pytest.mark.parametrize('j', [2, 5, 6])
def test_columns_change_recentres_the_rhythm(change, j):
    _, pkw, ckw, (a0, a1, dnew) = change
    old = sc.columns_events(_pattern(n=12, dwell=0.27, **pkw))
    snapshot = tuple(old)
    new = sc.columns_change(old, j, **ckw)
    assert old == snapshot and len(new) == 12
    # Events before j are the same objects.
    assert all(new[k] is old[k] for k in range(j))
    # Events j.. carry the new Apexes and columns_ball_timing's Dwells, the
    # transition event j + 1 the re-centring Dwell.
    tm = sc.columns_ball_timing(a0, a1, dnew)
    f_old = sc.flight_s(1.0)
    beta_old = (f_old + 0.27) / 2
    b, b2 = j % 2, (j + 1) % 2
    d_trans = (tm.dwells[b2] + (tm.beat - beta_old)
               + ((tm.flights[b2] - tm.flights[b]) - (f_old - f_old)))
    for k in range(j, 12):
        assert new[k].apex_m == (a0, a1)[k % 2]
        want = d_trans if k == j + 1 else tm.dwells[k % 2]
        assert new[k].dwell_s == want
    # Every transit from Throw j + 1 on is the new uniform one (per ball:
    # Throw i's transit ends at ball (i - 1)'s Catch), and so is every
    # compiled CATCH window from CATCH j on.
    bt = _times(new)
    tau = _transits(bt)
    for i in range(j + 1, 12):
        assert tau[i] == pytest.approx(tm.transits[(i - 1) % 2], abs=1e-12)
    sched = sc.compile_beats(new, launch_s=0.4, rest_tail_s=0.3,
                             t0_abs_s=1789263419.5)
    for k in range(j, 11):     # windows[k] = CATCH k
        assert bt.windows[k] == pytest.approx(tm.transits[k % 2], abs=1e-12)
    # Every release from Throw j on is one new Beat after the last.
    for i in range(j + 1, 12):
        assert bt.t_r[i] - bt.t_r[i - 1] == pytest.approx(tm.beat, abs=1e-12)
    # The Stop is still the last event: cross-site, shadow_landing.
    stop = sched.skills[11].then_throw
    assert stop.shadow_landing and stop.target is new[10].catch_site
    assert new[11].catch_site is not new[11].throw_site


def test_columns_change_transition_dwell_literal_values():
    """Hand values: apex 1.0 -> 0.9 at dwell 0.27: d + (beta' - beta) =
    0.27 + (f(0.9) - f(1.0)) / 2 = 0.2468245705524581."""
    old = sc.columns_events(_pattern(n=12, apex=1.0, dwell=0.27))
    new = sc.columns_change(old, 5, apex_m=0.9)
    assert new[6].dwell_s == pytest.approx(0.2468245705524581, abs=1e-15)
    new = sc.columns_change(old, 5, dwell_s=0.32)
    assert new[6].dwell_s == pytest.approx(0.32 + 0.025, abs=1e-15)


@pytest.mark.parametrize('pkw,ckw,code', [
    (dict(apex=0.9), dict(dwell_s=0.9), sc.BEAT_NO_TRANSIT),
    (dict(apex=1.0), dict(apex_m=0.3), sc.BEAT_DWELL_TOO_SHORT),
])
def test_columns_change_refused_by_the_rule_leaves_the_stream(pkw, ckw, code):
    """dwell 0.9 at apex 0.9 leaves no transit; apex 1.0 -> 0.3 at dwell 0.27
    makes the transition Dwell 0.27 + (beta' - beta) = 0.0657 s, under the
    0.1 s floor."""
    old = sc.columns_events(_pattern(n=12, dwell=0.27, **pkw))
    snapshot = tuple(old)
    with pytest.raises(ValueError) as exc:
        sc.columns_change(old, 5, **ckw)
    assert str(exc.value).startswith(code + ': Throw ')
    assert old == snapshot and all(a is b for a, b in zip(old, snapshot))


@pytest.mark.parametrize('j', [0, 1, 12, 13, -1])
def test_columns_change_out_of_range_is_refused_by_name(j):
    old = sc.columns_events(_pattern(n=12))
    with pytest.raises(ValueError, match=sc.BEAT_CHANGE_RANGE):
        sc.columns_change(old, j, apex_m=0.9)


def test_columns_change_of_the_stop_alone_is_accepted():
    old = sc.columns_events(_pattern(n=12))
    new = sc.columns_change(old, 11, apex_m=0.9)
    assert new[:11] == old[:11] and new[11].apex_m == 0.9
    sched = sc.compile_beats(new, launch_s=0.4, rest_tail_s=0.3,
                             t0_abs_s=10.0)
    assert sched.skills[11].then_throw.shadow_landing


def test_columns_change_during_a_recentring_is_refused_then_accepted():
    """Throws j - 2 and j - 1 must be on one Beat: after a change at 4 the
    stream re-centres through Throw 5, so 6 and 7 are refused and 8 is the
    first changeable Throw again."""
    old = sc.columns_events(_pattern(n=14))
    once = sc.columns_change(old, 4, apex_m=0.9)
    for j in (5, 6, 7):
        with pytest.raises(ValueError, match=sc.BEAT_CHANGE_RANGE):
            sc.columns_change(once, j, dwell_s=0.30)
    twice = sc.columns_change(once, 8, dwell_s=0.30)
    assert twice[:8] == once[:8]


@pytest.mark.parametrize('ckw', [dict(apex_m=0.9), dict(dwell_s=0.32),
                                 dict(apex_m=1.05, apex_m_ball_b=0.95)])
@pytest.mark.parametrize('j', list(range(2, 12)))
def test_first_changed_skill_is_the_one_carrying_throw_j(ckw, j):
    """THE PREFIX PROPERTY the executor relies on: for a change at Throw j
    (2 <= j <= n - 1) Skills 0 .. j - 1 compile field-identical and Skill j
    -- the CATCH that carries Throw j -- is the first that differs.  Pinned
    at a fed, hop-entry wall-clock Schedule through with_events."""
    pat = _pattern(n=12, apex=1.0, dwell=0.27)
    sched = sc.compile_columns(pat, feed=_feed(pat), hop_entry=True)
    new = sched.with_events(sc.columns_change(sched.source.events, j, **ckw))
    a = [cg.ser(s) for s in sched.skills]
    b = [cg.ser(s) for s in new.skills]
    assert b[:j] == a[:j]
    assert b[j] != a[j]
    assert new.skills[j].then_throw is not None
    assert new.t0_abs_s == sched.t0_abs_s
