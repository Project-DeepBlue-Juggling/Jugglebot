"""A clamp-shrunk Catch re-aim is judged on the Fit's demand, not on the
clamp's leftover (Sittings 2026-11-27 19:13 and 2026-11-28 12:21).

In two-ball Columns a Catch dispatched on the schedule's Site (no converged
Fit yet) was later asked by the Fit to move 16-36 mm OUTWARD.  The lateral
clamp (``catch_outward_authority_mm`` 10) shrank the move to exactly 10.0 mm,
and the re-aim fences then judged that 10.0 mm move WITHIN-TOLERANCE
(``moved_mm <= resend_pos_tol_mm`` 10.0) on both the re-send and the bump
path: the Cup never moved outward at all and six Feeds were lost.  The
authority was unusable whenever it was at or below the tolerance.

The contract pinned here, on BOTH re-aim paths (``_resend_live_catch``'s raw
re-solve and ``_catch_bump_decide``'s bump):

* a demand ABOVE the tolerance whose clamped move takes at least half the
  outward authority is re-aimed, for ANY authority (5, 10, 15 mm);
* a demand at or BELOW the tolerance (Fit noise) is still skipped
  WITHIN-TOLERANCE, whatever the clamp does to it;
* a Catch already committed AT the authority edge is not re-solved again for
  a demand the clamp pins to that same edge (the 2026-09-18 rule:
  a move that never reaches the platform spends no solve).
"""
from __future__ import annotations

import dataclasses
from types import SimpleNamespace

import numpy as np
import pytest

from jugglebot.motion.skills import executor as ex
from jugglebot.motion.skills import segments as sg
from jugglebot.motion.skills import sites as si
from tests.motion.test_skills_executor import (
    LAND_VEL, SEPARATION_MM, _FakeInstaller, _fit_landing, _schedule)

#: Ball 1's lateral (across) authority on hardware, and the inward one.
LATERAL_M = 0.030
INWARD_M = 0.035
TOL_MM = 10.0


@pytest.fixture(scope='module')
def sites():
    return si.columns_sites(SEPARATION_MM)


#: The Catch's Site: ``'neg'`` = P1 at x = -50 mm (outward is -x: the Site
#: whose Feeds were lost at the Sittings), ``'pos'`` = P2 at x = +50 mm
#: (outward is +x).  Every case runs at both.
SIDES = ('neg', 'pos')


def _outward_catch(sites, outward_mm, bump=False, side='neg'):
    """A CATCH at the ``side`` Site thrown from the other one (outward is
    away from the thrower), its first fit sitting exactly ON the schedule's
    Site, so the committed aim is the Site — the shape of the Sitting's
    ``source=schedule`` dispatches.
    Returns ``(x, inst, box, site_xyz, out_u, t_catch)``."""
    p1, p2 = sites
    catch_at, throw_from = (p1, p2) if side == 'neg' else (p2, p1)
    sch = _schedule(sites)
    sch = dataclasses.replace(sch, skills=tuple(
        dataclasses.replace(k, site=throw_from, target=catch_at)
        if k.kind == sg.THROW
        else dataclasses.replace(k, site=catch_at) for k in sch.skills))
    site = np.asarray(catch_at.catch_site_mm(), dtype=float)
    t_land = sch.skills[1].t_abs_s
    box = {'landing': _fit_landing(pos_mm=site.copy(), vel_mm_s=LAND_VEL,
                                   t_land_abs_s=t_land)}
    inst = _FakeInstaller()
    x = ex.SkillExecutor(sch, inst, tracker=lambda b: box['landing'],
                         lateral_authority_m=LATERAL_M,
                         catch_outward_authority_m=outward_mm / 1000.0,
                         catch_inward_authority_m=INWARD_M,
                         resend_pos_tol_mm=TOL_MM,
                         catch_bump_enabled=bump)
    x.tick(sch.skills[0].dispatch_s())
    t_catch = sch.skills[1].dispatch_s()
    x.tick(t_catch)
    assert x._live_catch is not None
    np.testing.assert_allclose(x._live_catch[1].landing_mm[:2], site[:2])
    out_u = x._outward_unit(sch.skills[1])
    assert out_u is not None
    # The geometry the parametrisation claims: outward is -x at the -x Site.
    sign = -1.0 if side == 'neg' else 1.0
    assert sign * site[0] > 0 and np.allclose(out_u, [sign, 0.0])
    return x, inst, box, site, out_u, t_catch


def _ask(box, site, out_u, demand_mm, t_land):
    box['landing'] = _fit_landing(
        pos_mm=site + np.array([out_u[0], out_u[1], 0.0]) * demand_mm,
        vel_mm_s=LAND_VEL, t_land_abs_s=t_land)


# (authority, demand, re-aimed?): demand above the 10 mm tolerance is taken
# at every authority; a noise-level demand is skipped at every authority
# (at 5 mm it is ALSO clamp-shrunk, to 5 mm: still noise, still skipped).
CASES = [
    (5.0, 25.0, True), (10.0, 25.0, True), (15.0, 25.0, True),
    (5.0, 8.0, False), (10.0, 8.0, False), (15.0, 8.0, False),
]


@pytest.mark.parametrize('side', SIDES)
@pytest.mark.parametrize('authority_mm,demand_mm,taken', CASES)
def test_resend_judges_the_demand_not_the_clamped_leftover(
        sites, authority_mm, demand_mm, taken, side):
    x, inst, box, site, out_u, t_catch = _outward_catch(
        sites, authority_mm, side=side)
    t_land = x._live_catch[1].t_land_s
    n = len(inst.calls)
    _ask(box, site, out_u, demand_mm, t_land)
    lines = x.tick(t_catch + 0.05)
    if taken:
        assert len(inst.calls) == n + 1, lines
        applied = min(authority_mm, demand_mm)
        np.testing.assert_allclose(
            inst.calls[-1][1].landing_mm[:2], site[:2] + out_u * applied,
            atol=1e-9)
        assert any(l.split(' ', 1)[1].startswith('RESEND skill 1')
                   for l in lines), lines
    else:
        assert len(inst.calls) == n, lines
        assert any('RESEND-SKIPPED WITHIN-TOLERANCE' in l for l in lines), lines


@pytest.mark.parametrize('side', SIDES)
@pytest.mark.parametrize('authority_mm,demand_mm,taken', CASES)
def test_bump_judges_the_demand_not_the_clamped_leftover(
        sites, authority_mm, demand_mm, taken, side):
    x, _inst, box, site, out_u, t_catch = _outward_catch(
        sites, authority_mm, bump=True, side=side)
    t_land = x._live_catch[1].t_land_s
    _ask(box, site, out_u, demand_mm, t_land)
    calls = []

    def apply(idx, ball_id, delta, t_c_abs, t_dispatch):
        calls.append(np.asarray(delta, dtype=float).copy())
        return SimpleNamespace(accepted=False, terminal=True,
                               code='PROBE-APPLIED')

    d = x._catch_bump_decide(t_land - 0.45, apply, None)
    if taken:
        assert d.reason == 'PROBE-APPLIED', d
        np.testing.assert_allclose(calls[0], out_u * min(authority_mm,
                                                         demand_mm), atol=1e-9)
    else:
        assert d.reason == 'WITHIN-TOLERANCE', d
        assert calls == []


@pytest.mark.parametrize('authority_mm', [5.0, 10.0, 15.0])
def test_a_catch_committed_at_the_edge_is_not_re_solved_for_the_same_edge(
        sites, authority_mm):
    """Once a re-aim has moved the Cup to the outward edge, a later Fit that
    still asks for more than the authority clamps to that SAME edge: zero
    applied move, so no further install (the solve-load guard of the
    2026-09-18 clamped-landing rule), and the one re-aim spent stays one."""
    x, inst, box, site, out_u, t_catch = _outward_catch(sites, authority_mm)
    t_land = x._live_catch[1].t_land_s
    _ask(box, site, out_u, 30.0, t_land)
    x.tick(t_catch + 0.05)
    n = len(inst.calls)
    _ask(box, site, out_u, 36.0, t_land)
    lines = x.tick(t_catch + 0.10)
    assert len(inst.calls) == n, lines
    assert x._resend_counts.get(1, 0) == 1


def test_a_small_step_toward_the_edge_from_near_it_is_not_re_solved(sites):
    """Committed 7 mm outward of a 10 mm edge (a converged Fit inside the
    authority), the Fit then asks 30 mm: the clamp leaves a 3 mm move, under
    half the authority — not worth a 25-130 ms solve, so it is skipped."""
    x, inst, box, site, out_u, t_catch = _outward_catch(sites, 10.0)
    t_land = x._live_catch[1].t_land_s
    _ask(box, site, out_u, 7.0, t_land)
    x.tick(t_catch + 0.05)            # 7 mm: noise, not re-aimed
    assert len(inst.calls) and x._resend_counts.get(1, 0) == 0
    # Re-commit at +7 mm by a fresh executor run would need a dispatch-time
    # fit there; emulate it directly on the live record instead.
    idx, term, t_last = x._live_catch
    term7 = dataclasses.replace(
        term, landing_mm=site + np.array([out_u[0], out_u[1], 0.0]) * 7.0)
    x._live_catch = (idx, term7, t_last)
    n = len(inst.calls)
    _ask(box, site, out_u, 30.0, t_land)
    lines = x.tick(t_catch + 0.10)
    assert len(inst.calls) == n, lines


# ── the other two clamps: behaviour unchanged ─────────────────────────────
# The across (lateral, 30 mm) and inward (35 mm) authorities are both wider
# than the tolerance, so a clamp-bitten move from the Site is already above
# it and was always re-aimed; and near their edge the half-authority floor is
# the tolerance itself (min(10, 15 or 17.5)), i.e. exactly the old rule.  In
# each case the outcome equals the pre-2026-11-28 rule: re-aimed iff the
# CLAMPED move exceeds the tolerance.

def _old_rule(applied_mm):
    return float(np.max(np.abs(applied_mm))) > TOL_MM


# (axis direction in units of out_u / across, committed offset, demand, label)
OTHER_CLAMPS = [
    ('across', 0.0, 40.0),    # from the Site: clamped 30, taken (old: taken)
    ('across', 22.0, 40.0),   # near the edge: clamped move 8, skipped (old too)
    ('inward', 0.0, 45.0),    # from the Site: clamped 35, taken (old: taken)
    ('inward', 28.0, 45.0),   # near the edge: clamped move 7, skipped (old too)
]


@pytest.mark.parametrize('side', SIDES)
@pytest.mark.parametrize('which,committed_mm,demand_mm', OTHER_CLAMPS)
def test_cross_axis_and_inward_clamps_keep_the_old_rule(
        sites, which, committed_mm, demand_mm, side):
    x, inst, box, site, out_u, t_catch = _outward_catch(sites, 10.0, side=side)
    t_land = x._live_catch[1].t_land_s
    if which == 'across':
        u = np.array([-out_u[1], out_u[0]])
        authority = LATERAL_M * 1000.0
    else:
        u = -np.asarray(out_u, dtype=float)
        authority = INWARD_M * 1000.0
    u3 = np.array([u[0], u[1], 0.0])
    if committed_mm:
        idx, term, t_last = x._live_catch
        x._live_catch = (idx, dataclasses.replace(
            term, landing_mm=site + u3 * committed_mm), t_last)
    n = len(inst.calls)
    box['landing'] = _fit_landing(pos_mm=site + u3 * demand_mm,
                                  vel_mm_s=LAND_VEL, t_land_abs_s=t_land)
    lines = x.tick(t_catch + 0.05)
    applied = u * (min(authority, demand_mm) - committed_mm)
    expect = _old_rule(applied)
    assert (len(inst.calls) == n + 1) == expect, lines
    if expect:
        np.testing.assert_allclose(inst.calls[-1][1].landing_mm[:2],
                                   site[:2] + u * min(authority, demand_mm),
                                   atol=1e-9)
