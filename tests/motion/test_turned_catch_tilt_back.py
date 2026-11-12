"""IDEA-22 (2026-11-12, axis changed 2026-11-13): turned Columns at 90 deg
tilts the Platform back at every auto-banked Catch, by a rotation about
Robot-frame +x (the Cup normal leans toward -y).

Owner, Sitting 2026-11-10 / TKT-62: "a default of ~3-5 deg tilt back from the
currently-computed catch orientation ... runtime configurable"; TKT-65
(2026-11-13): "let's go with 3 deg ... We need a rotation about +ve x, so that
the platform tip moves in -ve y" (the 2026-11-12 build rotated about +y and
leaned +x; superseded). The offset is
a Pattern property (``Pattern.catch_tilt_offset``, from
``nominate.turned_catch_tilt_offset``), carried on every auto-banked CATCH
Skill (``Skill.receive_tilt_offset``) and folded into the
``CatchTerminal.receive_tilt`` pin by ``segments.catch_receive_tilt`` (the one
derivation point the executor and the nomination share). Pinned here:

* the rule: 90 deg (either sign, within the tolerance) only, rotation about
  +x, Cup normal leaning -y; 45 deg, plain Columns and 0.0 give ``None``;
* the fold: computed tilt PLUS the offset, clamped to the 12 deg disc;
* the schedule: the Feed Catch stays pinned Level, every other Catch carries
  the offset, a recompiled tail keeps it, and 45 deg / plain Columns / a 0.0
  tilt-back compile today's schedule field for field (the Columns golden's
  serialiser);
* the flight and the probe agree: the executor's Catch terminal and the
  nomination's Catch terminal both carry the same pin.

Unmarked and parallel-safe: pure arithmetic plus one short planner walk.
"""

from __future__ import annotations

import importlib.util
import math
import os

import numpy as np
import pytest

from jugglebot.motion.skills import executor as ex
from jugglebot.motion.skills import nominate as nm
from jugglebot.motion.skills import schedule as sc
from jugglebot.motion.skills import segments as sg
from jugglebot.motion.trajectory import tilt_geometry as tg

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(
    __file__))))


def _golden_ser():
    path = os.path.join(_REPO, 'tools', 'probes', 'columns_golden.py')
    spec = importlib.util.spec_from_file_location('columns_golden_tb', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.ser


ser = _golden_ser()

C = nm.NominationConfig
TB3 = math.radians(3.0)
#: The 3 deg tilt-back as an (rx, ry) offset: a rotation about Robot-frame +x.
OFF3 = (TB3, 0.0)
VERTICAL = np.array([0.0, 0.0, -4350.5])
#: Ball 1's hop-entry arrival at 90 deg (the walk-through's), moving +y.
OBLIQUE = np.array([0.0, 142.0, -4350.5])


def _cfg(pattern='columns_turned', angle=90.0, tb=3.0):
    return C(pattern, 0.95, 125.0, 0.27,
             angle_deg=(angle if pattern == 'columns_turned' else 0.0),
             catch_tilt_back_deg=tb, entry='hop', feed_site='P1')


# ── the rule ────────────────────────────────────────────────────────────────

@pytest.mark.parametrize('angle', [90.0, -90.0, 90.4, 89.6])
def test_the_offset_is_a_rotation_about_plus_x_leaning_the_cup_normal_minus_y(
        angle):
    """TKT-65: "a rotation about +ve x, so that the platform tip moves in -ve
    y". ``cup_axis(+t, 0)`` is ``(0, -sin t, cos t)``."""
    off = nm.turned_catch_tilt_offset('columns_turned', angle, 3.0)
    assert off == OFF3
    axis = tg.cup_axis(*off)
    assert axis[0] == pytest.approx(0.0, abs=1e-15)
    assert axis[1] == pytest.approx(-math.sin(TB3))
    assert axis[2] == pytest.approx(math.cos(TB3))


@pytest.mark.parametrize('pattern, angle, tb', [
    ('columns_turned', 45.0, 3.0),
    ('columns_turned', 89.0, 3.0),
    ('columns_turned', 90.0, 0.0),
    ('columns', 0.0, 3.0),
    ('columns_highlow', 90.0, 3.0),
    ('hop', 90.0, 3.0),
])
def test_no_offset_off_the_90_deg_turned_pattern_or_at_zero(pattern, angle, tb):
    assert nm.turned_catch_tilt_offset(pattern, angle, tb) is None


# ── the fold (segments.catch_receive_tilt) ──────────────────────────────────

def test_no_offset_passes_the_pin_through_unchanged():
    assert sg.catch_receive_tilt(None, None, VERTICAL) is None
    assert sg.catch_receive_tilt((0.0, 0.0), None, VERTICAL) == (0.0, 0.0)


def _rule_offset(deg=3.0):
    """The offset the 90 deg rule produces (not a hand-typed tuple), so the
    fold tests below follow the rule's axis and sign."""
    return nm.turned_catch_tilt_offset('columns_turned', 90.0, deg)


def test_the_pin_is_the_computed_tilt_plus_the_offset():
    assert sg.catch_receive_tilt(None, _rule_offset(), VERTICAL) == OFF3
    base = np.asarray(tg.tilt_to_receive(OBLIQUE))
    assert base[0] > 0.0 and base[1] == 0.0   # the arrival's own bank, about +x
    got = sg.catch_receive_tilt(None, _rule_offset(), OBLIQUE)
    assert got == pytest.approx((base[0] + TB3, base[1]), abs=1e-15)


def test_the_folded_tilt_leans_the_cup_further_toward_minus_y():
    """For a representative 90 deg arrival (Ball 1's hop-entry Catch, moving
    +y), the folded pin's Cup axis sits further toward -y than the un-offset
    auto bank's, and no further in x."""
    base_axis = tg.cup_axis(*tg.tilt_to_receive(OBLIQUE))
    got_axis = tg.cup_axis(*sg.catch_receive_tilt(None, _rule_offset(),
                                                  OBLIQUE))
    assert got_axis[1] < base_axis[1] < 0.0
    assert got_axis[0] == pytest.approx(base_axis[0], abs=1e-15)


def test_the_sum_is_clamped_onto_the_12_deg_disc():
    # An arrival steep enough that the auto bank alone saturates at 12 deg,
    # leaning the Cup toward -y (an arrival moving +y): the offset cannot
    # push the commanded tilt past the ceiling.
    steep = np.array([0.0, 3000.0, -3000.0])
    got = sg.catch_receive_tilt(None, _rule_offset(), steep)
    assert math.degrees(math.hypot(*got)) == pytest.approx(tg.MAX_TILT_DEG)
    assert got[0] > 0.0
    # Direction kept: the clamp scales the raw sum, it does not rotate it.
    raw = np.asarray(tg.tilt_to_receive(steep)) + np.array(OFF3)
    assert got[0] * raw[1] - got[1] * raw[0] == pytest.approx(0.0, abs=1e-12)
    assert got[0] * raw[0] + got[1] * raw[1] > 0.0


def test_a_negative_tilt_back_leans_the_cup_axis_plus_y_through_the_fold():
    off = _rule_offset(-3.0)
    assert sg.catch_receive_tilt(None, off, VERTICAL) == (-TB3, 0.0)
    assert tg.cup_axis(*sg.catch_receive_tilt(None, off, VERTICAL))[1] > 0.0


def test_the_t0_initial_catch_of_a_90_deg_turned_pattern_carries_the_offset():
    pat = nm.build_pattern(_cfg())
    sch = sc.compile_columns(pat, t0_abs_s=1.0)
    first = next(s for s in sch.skills if s.kind == sg.CATCH)
    assert first.landing_prior is None and first.receive_tilt is None
    assert first.receive_tilt_offset == OFF3


def test_a_pinned_catch_cannot_also_take_an_offset():
    with pytest.raises(ValueError):
        sg.catch_receive_tilt((0.0, 0.0), OFF3, VERTICAL)
    site = sc.Site('P1', np.array([0.0, -62.5, 0.0]))
    with pytest.raises(ValueError):
        sc.Skill(kind=sg.CATCH, ball_id=0, site=site, t_abs_s=1.0,
                 window_s=0.3, receive_tilt=(0.0, 0.0),
                 receive_tilt_offset=OFF3)
    with pytest.raises(ValueError):
        sc.Skill(kind=sg.THROW, ball_id=0, site=site, t_abs_s=1.0,
                 window_s=0.3, receive_tilt_offset=OFF3)


# ── the schedule ────────────────────────────────────────────────────────────

def test_90_deg_every_auto_banked_catch_carries_the_offset_feed_stays_level():
    sch = nm.compile_schedule(_cfg())
    catches = [s for s in sch.skills if s.kind == sg.CATCH]
    feed = [s for s in catches if s.landing_prior is not None]
    assert len(feed) == 1
    assert feed[0].receive_tilt == (0.0, 0.0)
    assert feed[0].receive_tilt_offset is None
    rest = [s for s in catches if s.landing_prior is None]
    assert len(rest) == 4
    assert all(s.receive_tilt is None and s.receive_tilt_offset == OFF3
               for s in rest)
    # A recompiled tail (columns_change -> with_events) keeps the tilt-back.
    again = sch.with_events(sch.source.events)
    assert ser(again) == ser(sch)


@pytest.mark.parametrize('pattern, angle', [
    ('columns_turned', 45.0), ('columns', 0.0), ('columns_highlow', 0.0)])
def test_45_deg_and_straight_columns_compile_todays_schedule(pattern, angle):
    cfg = _cfg(pattern, angle, 3.0)
    if pattern == 'columns_highlow':
        cfg = C(pattern, 1.0, 100.0, 0.27, apex_m_ball2=0.9,
                catch_tilt_back_deg=3.0, entry='hop', feed_site='P1')
    today = C(cfg.pattern, cfg.apex_m, cfg.separation_mm, cfg.dwell_s,
              apex_m_ball2=cfg.apex_m_ball2, angle_deg=cfg.angle_deg,
              entry='hop', feed_site='P1')
    assert ser(nm.compile_schedule(cfg)) == ser(nm.compile_schedule(today))
    assert all(s.receive_tilt_offset is None
               for s in nm.compile_schedule(cfg).skills)


def test_a_zero_tilt_back_reproduces_todays_90_deg_schedule():
    """Param 0.0 = the schedule compiled with no ``catch_tilt_offset`` at all
    (the Pattern exactly as built before IDEA-22)."""
    cfg = _cfg(tb=0.0)
    pat = nm.build_pattern(cfg)
    assert pat.catch_tilt_offset is None
    legacy = sc.Pattern(sites=pat.sites, apex_m=pat.apex_m,
                        dwell_s=pat.dwell_s, n_throws=pat.n_throws)
    feed = sc.LandingPrior(pos_mm=pat.sites[1].catch_site_mm(),
                           vel_mm_s=VERTICAL, t_land_abs_s=12.0)
    got = sc.compile_columns(pat, feed=feed, hop_entry=True)
    want = sc.compile_columns(legacy, feed=feed, hop_entry=True)
    assert ser(got) == ser(want)


# ── the flight and the probe plan the same pin ──────────────────────────────

class _Installer:
    def __call__(self, kind, terminal, t_now_s, ball_id=None):
        return ex.InstallResult(True, 'OK', 'ok', 0.0, splice_k=0)


def _carried_catch(sch):
    return next(i for i, s in enumerate(sch.skills)
                if s.kind == sg.CATCH and s.landing_prior is None
                and s.then_throw is not None)


def test_the_executors_90_deg_catch_terminal_is_tilted_back_minus_y():
    sch = nm.compile_schedule(_cfg())
    x = ex.SkillExecutor(sch, _Installer(), tracker=lambda ball_id: None)
    i = _carried_catch(sch)
    sk = sch.skills[i]
    landing = ex.Landing(pos_mm=sk.site.catch_site_mm(), vel_mm_s=VERTICAL,
                         t_land_abs_s=float(sk.t_abs_s))
    term = x._catch_terminal(i, sk, landing)
    assert term.receive_tilt == OFF3
    assert tg.cup_axis(*term.receive_tilt)[1] < 0.0      # leans -y
    # The same Catch at 45 deg / tilt-back 0: today's auto-banked catch.
    sch0 = nm.compile_schedule(_cfg(tb=0.0))
    x0 = ex.SkillExecutor(sch0, _Installer(), tracker=lambda ball_id: None)
    assert x0._catch_terminal(i, sch0.skills[i], landing).receive_tilt is None


def test_the_nomination_plans_the_same_pin_the_executor_flies(monkeypatch):
    """The offline admissibility walk builds its Catch terminal through the
    same fold: the first auto-banked Catch reaches ``plan_segment`` with the
    computed tilt plus the offset as its receive pin."""
    seen = []
    real = sg.plan_segment

    def spy(kind, seed, term, *a, **k):
        if kind == sg.CATCH:
            seen.append(term.receive_tilt)
        return real(kind, seed, term, *a, **k)

    monkeypatch.setattr(nm.sg, 'plan_segment', spy)
    cfg = _cfg()
    sch = nm.compile_schedule(cfg)
    i = next(k for k, s in enumerate(sch.skills)
             if s.kind == sg.CATCH and s.landing_prior is None)
    lim = nm.sitting_limits(600.0, 8000.0, 200000.0, 4500.0)
    nm.plan_nominal(sch, lim, nm._default_geom(), nm.segment_config(cfg),
                    stop_after=i, proxy_seeds=False)
    assert seen[0] == (0.0, 0.0)                     # the Feed Catch, Level
    rx, ry = seen[-1]
    base = tg.tilt_to_receive(np.asarray(
        nm._landing(sch, i, sch.skills[i])[1], dtype=float))
    # + the tilt-back about +x; ry is the arrival's own bank, untouched.
    assert (rx, ry) == pytest.approx((base[0] + TB3, base[1]), abs=1e-12)
