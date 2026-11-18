"""The ``human_catch`` nomination (PWAY-11 P2, U5b): I-ADMIT-2 covers the
Pattern like every other.

What is judged: ONE nominal CATCH at the wait Site, from the REST the machine
holds there, for a Ball arriving VERTICALLY at the policy's nominal arrival
(D9, a PROPOSED owner value: ``HumanCatchPolicy.nominal_apex_m`` = 1.9 m
absolute, 1.07 m above the 830 mm Catch plane, 4.58 m/s) with the LATEST lead
the policy still plans from (D3 ``first_lead_s`` 0.55 s, PROPOSED).  Zero
displacement: the Landing IS the wait Site, so every Leg stays still (peak
velocity / acceleration / jerk 0, margin 1.000) and what the planner gates is
the Hand's catch.  The reach of a displaced Landing is the P2 reach table, not
an accept rule (design section 2.4).

Real planner, no stub.  Unmarked, parallel-safe (about 3 solves).
"""

from __future__ import annotations

import dataclasses
import math

import numpy as np
import pytest

from jugglebot.motion.skills import human_catch as hcp
from jugglebot.motion.skills import nominate as nm
from jugglebot.motion.skills import schedule as sc
from jugglebot.motion.skills import sites as st

C = nm.NominationConfig
P4 = nm.sitting_limits(600.0, 8000.0, 200000.0, 4500.0)
GATE = nm.sitting_limits(350.0, 5000.0, 200000.0, 4500.0)


def _cfg(**kw):
    return C('human_catch', 0.9, 100.0, 0.27, **kw)


def test_human_catch_is_a_nominated_pattern():
    assert 'human_catch' in nm.NOMINATED_PATTERNS
    assert 'human_catch' not in nm.NOT_NOMINATED_PATTERNS
    assert nm.workspace_ok(_cfg())


def test_the_defaults_are_the_policys_not_a_second_copy():
    c = _cfg()
    d = {n: p.default for n, p in
         __import__('inspect').signature(hcp.HumanCatchPolicy).parameters.items()}
    assert tuple(c.hc_home_xy_mm) == tuple(d['home_xy_mm'])
    assert c.hc_first_lead_s == d['first_lead_s']
    assert c.hc_nominal_apex_m == d['nominal_apex_m']
    assert (c.hc_first_lead_s, c.hc_nominal_apex_m) == (0.55, 1.9)


def test_the_nominal_ball_is_vertical_at_4p58_ms_onto_the_wait_site():
    nm.clear_cache()
    sch = nm.compile_schedule(_cfg())
    (sk,) = sch.skills
    assert sch.pattern == 'human_catch' and sk.kind == sc.CATCH
    lp = sk.landing_prior
    np.testing.assert_allclose(lp.pos_mm, [-50.0, 0.0, st.CATCH_CUP_Z_MM])
    assert lp.vel_mm_s[0] == 0.0 and lp.vel_mm_s[1] == 0.0
    # D9: 1.9 m absolute is 1.07 m above the Catch plane: sqrt(2 g h) 4.58 m/s
    assert -lp.vel_mm_s[2] / 1000.0 == pytest.approx(
        math.sqrt(2 * 9.81 * 1.07), abs=0.01)
    assert -lp.vel_mm_s[2] / 1000.0 == pytest.approx(4.58, abs=0.005)
    # the LATEST planned lead, the dispatch look-ahead and the install lead
    # reserved ahead of touch-down; the window opens at the plan instant
    assert lp.t_land_abs_s - sch.t0_abs_s == pytest.approx(0.55)
    assert sk.window_s == pytest.approx(
        0.55 - nm.HUMAN_CATCH_DISPATCH_LEAD_S - sc.LEAD_S)
    assert sk.catch_open_abs_s == pytest.approx(sch.t0_abs_s)
    assert nm.HUMAN_CATCH_DISPATCH_LEAD_S == pytest.approx(0.05)


@pytest.mark.parametrize('limits', [P4, GATE], ids=['P4', 'GATE'])
def test_the_nominal_human_catch_is_viable_at_both_working_points(limits):
    nm.clear_cache()
    rep = nm.nominate(_cfg(), limits)
    assert rep.ok, rep.lines()
    assert rep.refusals == [] and rep.solves == 3       # one CATCH per rung
    assert [v.kind for v in rep.verdicts] == [sc.CATCH] * 3
    assert all(v.ok and v.code == 'OK' for v in rep.verdicts)
    # no displacement: every Leg stays still, the margin is the full 1.000
    assert all(v.margin == pytest.approx(1.0, abs=1e-6) for v in rep.verdicts)
    assert all(tuple(v.peaks) == (0.0, 0.0, 0.0) for v in rep.verdicts)
    s = rep.summary()
    assert s.startswith('nomination OK: human_catch:')
    assert 'PROPOSED D9' in s and 'Apex 1.90 m' in s and '0.55 s' in s


def test_the_nominal_arrival_is_a_parameter_not_a_constant():
    """D9 is the owner's to set: another Apex is another arrival speed and
    another cache key."""
    hi = _cfg(hc_nominal_apex_m=2.2)
    (sk,) = nm.compile_schedule(hi).skills
    assert -sk.landing_prior.vel_mm_s[2] / 1000.0 == pytest.approx(
        math.sqrt(2 * 9.81 * (2.2 - 0.83)), abs=0.01)         # 5.18 m/s
    assert hi.key() != _cfg().key()
    assert 'Apex 2.20 m' in nm.nominate(hi, P4).summary()


def test_the_wait_site_follows_the_home_parameter():
    cfg = _cfg(hc_home_xy_mm=(30.0, -20.0))
    pat = nm.build_pattern(cfg, 7)
    assert pat.n_throws == 7 and pat.sites[0].name == 'W'
    np.testing.assert_allclose(pat.sites[0].cup_mm,
                               [30.0, -20.0, st.CATCH_CUP_Z_MM])
    (sk,) = nm.compile_schedule(cfg).skills
    np.testing.assert_allclose(sk.landing_prior.pos_mm,
                               [30.0, -20.0, st.CATCH_CUP_Z_MM])


def test_a_lead_the_window_cannot_hold_is_a_compile_refusal_and_never_searched():
    nm.clear_cache()
    rep = nm.nominate(_cfg(hc_first_lead_s=0.30), P4)
    assert not rep.ok and rep.solves == 0 and rep.verdicts == []
    assert rep.lines()[0].startswith('COMPILE:')
    # apex / separation / dwell are no knob of this Pattern: no nearest search
    assert rep.nearest == [] and not rep.search_done and rep.searched == 0


def test_a_landing_offset_is_refused_for_human_catch():
    with pytest.raises(ValueError):
        nm.build_pattern(_cfg(landing_offset_mm=(10.0, 0.0)))


def test_a_wait_site_outside_the_policys_box_is_a_compile_refusal():
    rep = nm.nominate(_cfg(hc_home_xy_mm=(400.0, 0.0)), P4)
    assert not rep.ok and rep.solves == 0
    assert rep.lines()[0].startswith('COMPILE:')
