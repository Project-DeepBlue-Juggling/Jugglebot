"""The human_catch CATCH's own catch window (PWAY-11 P2, U5a).

A ``compile_human_catch`` Schedule is ONE CATCH and no release, so the
MISSED_CATCH rule's ``t_open`` anchor (the latest scheduled release before the
landing, ``executor._latest_release_before``) does not exist and, before this
unit, the CATCH armed no :class:`executor._PendingCatch`: ``done`` was true the
tick the CATCH dispatched, about 0.5 s before touch-down, so no Catch verdict
was ever produced and the live re-aims never ran.

The fix is the narrow one (orchestrator decision 2026-11-18, option (b)): an
optional ``Skill.catch_open_abs_s``, set ONLY by ``compile_human_catch`` (to the
instant the plan is made), which ``_register_catch`` takes as the window's
start.  ``None`` (every other compile) leaves the release-derived rule exactly
as it was; the pins at the bottom of this file hold that.

Unmarked and parallel-safe: a fake installer, no solve.
"""

from __future__ import annotations

import inspect
import os

import numpy as np
import pytest

from jugglebot import ball_possession as bp
from jugglebot.motion.skills import executor as ex
from jugglebot.motion.skills import nominate as nm
from jugglebot.motion.skills import schedule as sc
from jugglebot.motion.skills import sites as st

LOOKAHEAD_S = 0.05            # skill_node._DISPATCH_LOOKAHEAD_S
ARRIVAL_VEL = np.array([0.0, 0.0, -4580.0])
T_NOW = 100.0
LEAD_S = 0.70


class _Installer:
    def __init__(self):
        self.calls = []

    def __call__(self, kind, terminal, t_now_s, ball_id=None):
        self.calls.append((kind, terminal, t_now_s, ball_id))
        return ex.InstallResult(True, 'OK', 'ok', 0.0, splice_k=0)


def _obs():
    # A CATCH with an empty cup on a ready machine (the human_catch shape).
    return ex.Observations(mocap_fresh=True, hand_fresh=True, levelled=True,
                           ball_evidence=bp.EVIDENCE_EMPTY,
                           in_trajectory_mode=True)


def _live(n_seated):
    """A LIVE 100 Hz window with ``n_seated`` raw-SEATED samples."""
    def window(t0, t1):
        return ex.SeatWindow(n_valid=int(round((t1 - t0) * 100)),
                             n_seated=n_seated, max_gap_s=0.01,
                             t_first_seated_s=(t1 - 0.2 if n_seated else None))
    return window


def _landing_mm(x=-50.0, y=0.0):
    return np.array([x, y, st.CATCH_CUP_Z_MM])


def _hc_schedule(t_now=T_NOW, lead_s=LEAD_S):
    return sc.compile_human_catch(_landing_mm(), ARRIVAL_VEL, t_now + lead_s,
                                  t_now, LOOKAHEAD_S)


def _executor(sch, seat_window, tracker=None, inst=None, **kw):
    inst = _Installer() if inst is None else inst
    x = ex.SkillExecutor(sch, inst, tracker=tracker,
                         observations=lambda t: _obs(),
                         observer=lambda ball_id, t: bp.EVIDENCE_EMPTY,
                         seat_window=seat_window, **kw)
    return x, inst


def _run(x, t_from, t_to, dt=0.025):
    lines = []
    t = t_from
    while t < t_to:
        lines.extend(x.tick(t))
        t += dt
    lines.extend(x.tick(t_to))
    return lines


# ── the field ────────────────────────────────────────────────────────────────

def test_compile_human_catch_opens_the_window_at_the_plan_instant():
    sch = _hc_schedule()
    (sk,) = sch.skills
    assert sk.catch_open_abs_s == pytest.approx(T_NOW, abs=1e-12)
    assert sk.catch_open_abs_s == sch.t0_abs_s


def test_only_a_catch_may_carry_a_window_start_and_it_must_be_finite():
    site = st.columns_sites(100.0)[0]
    with pytest.raises(ValueError, match='catch_open_abs_s'):
        sc.Skill(kind=sc.REST, ball_id=0, site=site, t_abs_s=1.0,
                 window_s=0.5, catch_open_abs_s=0.5)
    with pytest.raises(ValueError, match='catch_open_abs_s'):
        sc.Skill(kind=sc.CATCH, ball_id=0, site=site, t_abs_s=1.0,
                 window_s=0.5, catch_open_abs_s=float('nan'))


# ── the CATCH is kept alive to its verdict ──────────────────────────────────

def test_a_human_catch_arms_a_pending_catch_from_the_plan_instant():
    sch = _hc_schedule()
    L = T_NOW + LEAD_S
    x, inst = _executor(sch, _live(0))
    t_disp = sch.skills[0].dispatch_s()
    x.tick(t_disp)
    assert len(inst.calls) == 1 and inst.calls[0][0] == sc.CATCH
    assert len(x._pending_catches) == 1
    pend = x._pending_catches[0]
    assert pend.t_open_s == pytest.approx(T_NOW)
    assert pend.t_close_s == pytest.approx(
        L + ex.MISSED_CATCH_CLOSE_AFTER_LANDING_S)
    # every skill is dispatched, yet the Attempt is not over: the verdict is
    # still due (the defect this unit fixes: done was True here)
    assert len(x.dispatched) == len(sch.skills)
    assert not x.done
    t_decide = (L + ex.MISSED_CATCH_CLOSE_AFTER_LANDING_S
                + ex.MISSED_CATCH_DECIDE_LAG_S)
    _run(x, t_disp + 0.025, t_decide - 0.005)
    assert not x.done


def test_a_seated_cup_in_the_window_is_the_catch_verdict():
    sch = _hc_schedule()
    L = T_NOW + LEAD_S
    x, _inst = _executor(sch, _live(12))
    t_decide = (L + ex.MISSED_CATCH_CLOSE_AFTER_LANDING_S
                + ex.MISSED_CATCH_DECIDE_LAG_S)
    lines = _run(x, sch.skills[0].dispatch_s(), t_decide)
    assert x.done
    assert not x.attempt_ended
    assert x.end_code == ''                  # caught: nothing ended it
    assert any('CATCH-EVIDENCE-ABSTAIN' in ln and 'cup read SEATED' in ln
               for ln in lines), lines


def test_a_ball_that_never_arrives_is_missed_catch():
    sch = _hc_schedule()
    L = T_NOW + LEAD_S
    x, _inst = _executor(sch, _live(0))
    t_decide = (L + ex.MISSED_CATCH_CLOSE_AFTER_LANDING_S
                + ex.MISSED_CATCH_DECIDE_LAG_S)
    lines = _run(x, sch.skills[0].dispatch_s(), t_decide)
    assert x.attempt_ended
    assert x.end_code == ex.MISSED_CATCH
    assert any('MISSED_CATCH' in ln for ln in lines)
    # BUG-68: the miss is provisional for MISSED_CATCH_FAULT_SETTLE_S, then over
    assert not x.done
    _run(x, t_decide + 0.025, t_decide + ex.MISSED_CATCH_FAULT_SETTLE_S + 0.03)
    assert x.done
    assert x.end_code == ex.MISSED_CATCH


def test_a_later_fit_re_aims_the_live_human_catch():
    """The executor's own re-aim path (``_resend_live_catch``) runs because the
    CATCH's executor is still live after its dispatch."""
    sch = _hc_schedule()
    L = T_NOW + LEAD_S
    box = {'landing': ex.Landing(pos_mm=_landing_mm(), vel_mm_s=ARRIVAL_VEL,
                                 t_land_abs_s=L, from_fit=True)}
    x, inst = _executor(sch, _live(12), tracker=lambda b: box['landing'],
                        lateral_authority_m=0.10)
    t_disp = sch.skills[0].dispatch_s()
    x.tick(t_disp)
    assert len(inst.calls) == 1
    box['landing'] = ex.Landing(pos_mm=_landing_mm(-50.0, 40.0),
                                vel_mm_s=ARRIVAL_VEL, t_land_abs_s=L + 0.02,
                                from_fit=True)
    lines = x.tick(t_disp + 0.05)
    assert len(inst.calls) == 2, lines
    assert any('RESEND' in ln and 'RESEND-SKIPPED' not in ln for ln in lines)
    assert inst.calls[-1][1].landing_mm[1] == pytest.approx(40.0)
    assert not x.done


# ── regression pins: every other compile is untouched ───────────────────────

def _all_other_compiles():
    out = []
    for p in nm.NOMINATED_PATTERNS:
        out.append((p, nm.compile_schedule(
            nm.NominationConfig(p, 0.9, 100.0, 0.27))))
    out.append(('columns hop entry', nm.compile_schedule(
        nm.NominationConfig('columns', 0.9, 100.0, 0.27, entry='hop'))))
    site = st.columns_sites(150.0)[0]
    one = sc.OneBallPattern(sites=(site,), apex_m=1.0, dwell_s=0.27,
                            n_throws=1, floor_lift_s=sc.FLOOR_LIFT_S)
    out.append(('reload (Feed CATCH)', sc.compile_reload(
        site.catch_site_mm(), np.array([900.0, 0.0, -4200.0]), 100.0, one,
        95.0)))
    out.append(('reload_wait', sc.compile_reload_wait(
        site, sc.FLOOR_LIFT_S, 95.0)))
    return out


def test_no_other_compile_sets_the_window_start():
    for name, sch in _all_other_compiles():
        for sk in sch.skills:
            assert sk.catch_open_abs_s is None, (name, sk)


def test_only_compile_human_catch_sets_the_window_start_in_source():
    """A structural pin on top of the compiles above: the keyword is passed
    in ``motion/`` only by ``compile_human_catch`` (its literal) and
    ``_shifted`` (which moves it with every other absolute instant)."""
    src = inspect.getsource(sc)
    assert src.count('catch_open_abs_s=') == 2
    assert 'catch_open_abs_s=0.0' in inspect.getsource(sc.compile_human_catch)
    motion = os.path.dirname(os.path.dirname(os.path.abspath(sc.__file__)))
    for root, _dirs, files in os.walk(motion):
        for f in files:
            if f.endswith('.py') and f != 'schedule.py':
                with open(os.path.join(root, f)) as fh:
                    assert 'catch_open_abs_s=' not in fh.read(), f


def test_a_feed_catch_with_no_release_before_it_arms_nothing_as_on_main():
    """The reload's Feed CATCH (``landing_prior``, no release in its Schedule
    before its landing) stays UNARMED: option (a), anchoring on the prior,
    was rejected because it would change exactly this CATCH."""
    site = st.columns_sites(150.0)[0]
    one = sc.OneBallPattern(sites=(site,), apex_m=1.0, dwell_s=0.27,
                            n_throws=1, floor_lift_s=sc.FLOOR_LIFT_S)
    sch = sc.compile_reload(site.catch_site_mm(),
                            np.array([900.0, 0.0, -4200.0]), 100.0, one, 95.0)
    idx = next(i for i, sk in enumerate(sch.skills)
               if sk.kind == sc.CATCH and sk.landing_prior is not None)
    feed = sch.skills[idx]
    assert ex._latest_release_before(sch, feed.t_abs_s) is None
    x, _inst = _executor(sch, _live(0))
    x._register_catch(idx, feed, float(feed.t_abs_s))
    assert x._pending_catches == []


def test_a_release_anchored_catch_keeps_its_window_exactly():
    """A plain THROW -> CATCH: the window still opens 0.10 s after the release
    and closes 0.50 s after the landing (the field is None)."""
    p1, p2 = st.columns_sites(100.0)
    t_throw, L = 10.0, 10.8
    skills = (
        sc.Skill(kind=sc.THROW, ball_id=0, site=p1, t_abs_s=t_throw,
                 window_s=0.3, y_d=(np.zeros(2), 0.8), target=p2),
        sc.Skill(kind=sc.CATCH, ball_id=0, site=p2, t_abs_s=L, window_s=0.5),
    )
    sch = sc.Schedule(pattern='columns', skills=skills, flight_s=0.8,
                      beat_s=0.578, transit_s=0.278, dwell_s=0.30,
                      t0_abs_s=t_throw)
    x, _inst = _executor(sch, _live(0))
    x._register_catch(1, skills[1], L)
    (pend,) = x._pending_catches
    assert pend.t_open_s == pytest.approx(
        t_throw + ex.MISSED_CATCH_OPEN_AFTER_RELEASE_S)
    assert pend.t_close_s == pytest.approx(
        L + ex.MISSED_CATCH_CLOSE_AFTER_LANDING_S)
