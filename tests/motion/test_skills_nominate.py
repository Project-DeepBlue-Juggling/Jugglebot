"""``motion/skills/nominate`` -- the nomination-time check (learner-
generalisation P3b, owner D8), with the REAL planner and gate.

Recipe (every pinned number below)
----------------------------------
Generated on the Jetson, 2026-11-06, worktree ``Jugglebot-p3`` (branch
``p3-online-admissibility``), venv python 3.8.10, ONE BLAS thread::

    OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python -m pytest \
        tests/motion/test_skills_nominate.py -q -p no:cacheprovider

and cross-checked against the walker probes recorded in
``logbook/2026-11-06-learner-generalisation-p3b-nomination-check.md``: the
same refusing Segments, codes, legs, margins and nearest-viable answers.
Margins are pinned to 1e-6 (``admissible.margin``, leg-only).  A nomination is
~0.6 s (18 solves at ~33 ms) on this box; a refused one plus its search
1.5-3 s.  The ``run_tests.sh`` gate exports ``OPENBLAS_NUM_THREADS=1``.

Limits: ``P4`` = the 2026-11-03 working point 600 / 8000 / 200000 mm/s^n,
Hand 4500 rev/s^2 (today's launch constants); ``GATE`` = the 2026-10-31
high-low finding's 350 / 5000 / 200000, Hand 4500
(``logbook/2026-10-31-pattern-progression-p4-high-low-columns.md``).

Unmarked: no ``serial`` (no wall-clock budget is claimed here; the measured
seconds are in the recipe) and no ``nightly`` (it gates goal acceptance).
"""

from __future__ import annotations

import dataclasses

import numpy as np
import pytest

from jugglebot.motion.skills import nominate as nm
from jugglebot.motion.skills import schedule as sc
from jugglebot.motion.skills import segments as sg

C = nm.NominationConfig
P4 = nm.sitting_limits(600.0, 8000.0, 200000.0, 4500.0)
GATE = nm.sitting_limits(350.0, 5000.0, 200000.0, 4500.0)

TOL = 1e-6


def _refused(rep, idx, frac):
    vs = [v for v in rep.verdicts
          if v.idx == idx and abs(v.flight_frac - frac) < 1e-12]
    assert len(vs) == 1
    return vs[0]


def _bad(rep):
    return sorted({(v.idx, v.code, v.leg) for v in rep.verdicts if not v.ok})


# ── the walker: fidelity to the P3a probe ────────────────────────────────────

def test_columns_at_p4_plans_every_segment_and_matches_the_p3a_probe_cell():
    """Columns 0.95 / 125 / 0.27, pose, at P4: every Segment planned and
    viable at every rung, and the steady catch-with-throw AT P2 (+62.5 mm)
    seeded from a carried release at P1 -- skill 2 under skill_node's default
    ``columns_feed_site='P1'`` (Ball 1 at P2) -- equals
    ``tools/probes/admissible_obviation/online_check_replay.py``'s
    ``Cell((62.5, 0), 0.95)`` at offset 0 (0.381994) to 1e-6.  It is the
    probe's shape: same Site, same incoming flight, same carried Throw; its
    seed differs only in Hand velocity (131.49 vs 132.54 rev/s: the walker
    aims every Throw at the CATCH point, the probe's seed at the release
    height), which leaves the leg margin identical.  The others need not
    coincide: skills 1 / 3 are the mirror catch at P1 (x-mirror is not a
    symmetry of the platform: 0.383161), skill 4 carries the Stop to the
    other Site, skill 5 is the standalone last CATCH the box never swept."""
    nm.clear_cache()
    rep = nm.nominate(C('columns', 0.95, 125.0, 0.27), P4, search=False)
    assert rep.ok, rep.lines()
    assert rep.refusals == [] and rep.solves == 18
    assert all(v.ok for v in rep.verdicts)
    assert sum(1 for v in rep.verdicts if v.code == 'REST') == 3
    for frac in nm.RUNGS:
        v = _refused(rep, 2, frac)
        assert (v.kind, v.ball, v.site, v.target) == ('CATCH', 'Ball 1', 'P2', 'P2')
        assert v.seed == 'release of skill 1'
        assert v.margin == pytest.approx(0.381994251, abs=TOL)
        assert _refused(rep, 1, frac).margin == pytest.approx(0.383161404, abs=TOL)
        assert _refused(rep, 5, frac).margin == pytest.approx(0.334698211, abs=TOL)
        assert _refused(rep, 0, frac).margin == pytest.approx(1.0, abs=TOL)
    w = rep.worst()
    assert (w.idx, w.flight_frac, w.binding) == (5, -0.20, 'jerk')
    # JB-185: a plain nomination (the goal accept's) walks no second figure and
    # prints the pre-JB-185 line byte for byte ...
    assert rep.aim_verdicts == [] and rep.aim_solves == 0
    assert rep.summary() == (
        'nomination OK: columns apex 0.95 m, separation 125 mm, dwell 0.27 s, '
        'pose; worst margin 0.335 (jerk, CATCH Ball 2 at P1, rung -20 %)')
    # ... and asking for it adds the second figure to the SAME cached report
    both = nm.nominate(C('columns', 0.95, 125.0, 0.27), P4, search=False,
                       aim=nm.LAUNCH_AIM)
    assert both is rep
    assert both.summary().startswith(
        'nomination OK: columns apex 0.95 m, separation 125 mm, dwell 0.27 s, '
        'pose; worst margin 0.335 at the nominal Sites '
        '(jerk, CATCH Ball 2 at P1, rung -20 %), ')
    assert 'with the Catch re-aim at its limit' in both.summary()


# ── the PHASE GATE: the 2026-10-31 high-low finding ─────────────────────────

def test_phase_gate_highlow_100_90_refused_at_125_names_separation_100():
    """1.00 / 0.90, dwell 0.27, pose, at GATE: refused at 125 with every
    refusal listed (the low catches LIMIT_VEL leg 2, the last CATCH LIMIT_ACC
    leg 1, all rungs alike), and the search names separation 100 and nothing
    else (apex / dwell steps all refused)."""
    nm.clear_cache()
    rep = nm.nominate(C('columns_highlow', 1.0, 125.0, 0.27, apex_m_ball2=0.9), GATE)
    assert not rep.ok
    assert _bad(rep) == [(1, 'LIMIT_VEL', 2), (3, 'LIMIT_VEL', 2),
                         (5, 'LIMIT_ACC', 1)]
    v1 = _refused(rep, 1, 0.0)
    assert (v1.ball, v1.site, v1.binding) == ('Ball 2', 'P1', 'acc')
    assert v1.t_s == pytest.approx(0.1875)
    assert v1.margin == pytest.approx(-0.173736530, abs=TOL)
    assert _refused(rep, 1, -0.20).margin == pytest.approx(-0.173752838, abs=TOL)
    v5 = _refused(rep, 5, 0.0)
    assert v5.t_s == pytest.approx(0.100)
    assert v5.margin == pytest.approx(-0.253243877, abs=TOL)
    lines = rep.lines()
    assert len(lines) == 3
    assert lines[0].startswith('skill 1 CATCH Ball 2 at P1 -> P1, rungs -20/+0/+10 %: '
                               'LIMIT_VEL leg 2 at t=0.188 s, margin -0.174 (acc)')
    assert lines[2].startswith('skill 5 CATCH Ball 2 at P1, rungs -20/+0/+10 %: '
                               'LIMIT_ACC leg 1 at t=0.100 s, margin -0.253 (acc)')
    # I-ADMIT-2: the Segments downstream of a refused one are planned from a
    # LABELLED seed (never skipped), the label naming the actual refusal.
    assert _refused(rep, 2, 0.0).ok
    assert _refused(rep, 2, 0.0).seed == (
        'PROXY: launch from rest at P1 (skill 1 refused LIMIT_VEL)')
    assert _refused(rep, 3, 0.0).seed == 'release of skill 2 [chain via PROXY]'
    assert _refused(rep, 4, 0.0).seed == (
        'PROXY: launch from rest at P1 (skill 3 refused LIMIT_VEL)')
    assert _refused(rep, 5, 0.0).seed == 'release of skill 4 [chain via PROXY]'
    assert lines[1].endswith('{seed: release of skill 2 [chain via PROXY]}')
    assert lines[2].endswith('{seed: release of skill 4 [chain via PROXY]}')
    assert [(p, v) for p, v, _ in rep.nearest] == [('separation', 100.0)]
    assert rep.summary() == (
        'nomination REFUSED: columns_highlow apex 1.00/0.90 m, separation 125 mm, '
        'dwell 0.27 s, pose; 3 refusals; nearest viable: separation 100 mm')
    # bounded: 3 full nominations, 78 solves (focus-Segment pre-checks included)
    assert (rep.searched, rep.search_solves) == (3, 78)


def test_phase_gate_highlow_105_95_refused_at_125_names_separation_100():
    """1.05 / 0.95 likewise: the low catches and the last CATCH refuse on
    MARGIN (inside the hard limit, outside 0.9 of it); separation 100 or a
    shorter dwell (0.22 s) is viable."""
    nm.clear_cache()
    rep = nm.nominate(C('columns_highlow', 1.05, 125.0, 0.27, apex_m_ball2=0.95), GATE)
    assert not rep.ok
    assert _bad(rep) == [(1, 'MARGIN', None), (3, 'MARGIN', None),
                         (5, 'MARGIN', None)]
    assert _refused(rep, 1, 0.0).margin == pytest.approx(-0.073450838, abs=TOL)
    assert _refused(rep, 1, 0.0).binding == 'vel'
    assert _refused(rep, 5, 0.0).margin == pytest.approx(-0.078540885, abs=TOL)
    assert _refused(rep, 5, 0.0).binding == 'acc'
    assert [(p, v) for p, v, _ in rep.nearest] == [('separation', 100.0),
                                                   ('dwell', 0.22)]
    assert rep.summary().endswith(
        '3 refusals; nearest viable: separation 100 mm (or dwell 0.22 s, shorter)')
    assert (rep.searched, rep.search_solves) == (2, 48)


@pytest.mark.parametrize('apex, apex2, worst', [(1.0, 0.9, 0.094482784),
                                                 (1.05, 0.95, 0.163118488)])
def test_phase_gate_highlow_viable_at_100(apex, apex2, worst):
    rep = nm.nominate(C('columns_highlow', apex, 100.0, 0.27, apex_m_ball2=apex2),
                      GATE)
    assert rep.ok, rep.lines()
    assert rep.worst().margin == pytest.approx(worst, abs=TOL)


# ── the oval dwell ladder ────────────────────────────────────────────────────

def test_oval_refused_dwell_names_the_first_viable_dwell_at_p4():
    """``oval_1ball`` 0.95 / 150, pose, carry_zvd false, at P4: dwell 0.30 is
    refused (the carried catch: LIMIT_JERK leg 0 at -20 %, MARGIN -0.034 at
    the nominal; viable at +10 %), and the search walks the dwell UP and names
    0.35 s (margin 0.123), and separation 100 mm."""
    nm.clear_cache()
    rep = nm.nominate(C('oval_1ball', 0.95, 150.0, 0.30), P4)
    assert not rep.ok
    v = _refused(rep, 2, -0.20)
    assert (v.code, v.leg, v.site, v.release) == ('LIMIT_JERK', 0, 'P2', 'P1')
    assert v.margin == pytest.approx(-0.271565794, abs=TOL)
    assert _refused(rep, 2, 0.0).code == 'MARGIN'
    assert _refused(rep, 2, 0.0).margin == pytest.approx(-0.033695992, abs=TOL)
    assert _refused(rep, 2, 0.10).ok
    # a dependant of a planned-then-refused producer names its actual refusal
    assert _refused(rep, 3, 0.0).seed == 'release of skill 2 (itself refused MARGIN)'
    assert _refused(rep, 3, -0.20).seed == (
        'PROXY: launch from rest at P1 (skill 2 refused LIMIT_JERK)')
    assert dict((p, v) for p, v, _ in rep.nearest) == {'separation': 100.0,
                                                       'dwell': 0.35}
    assert 'dwell 0.35 s, longer' in rep.summary()
    ok = nm.nominate(C('oval_1ball', 0.95, 150.0, 0.35), P4, search=False)
    assert ok.ok and ok.worst().margin == pytest.approx(0.123371798, abs=TOL)


def test_oval_carry_zvd_at_gate_limits_refuses_035_and_names_040():
    """``carry_zvd=True`` at GATE: dwell 0.35 refused (LIMIT_ACC leg 1 at
    t=0.863 s on every rung), first viable dwell 0.40 (margin 0.117), or
    separation 125 mm."""
    nm.clear_cache()
    rep = nm.nominate(C('oval_1ball', 0.95, 150.0, 0.35, carry_zvd=True), GATE)
    assert not rep.ok
    assert {(v.idx, v.code, v.leg) for v in rep.verdicts if not v.ok} == {
        (2, 'LIMIT_ACC', 1), (3, 'LIMIT_ACC', 1)}
    assert _refused(rep, 2, 0.0).margin == pytest.approx(-0.147113829, abs=TOL)
    assert dict((p, v) for p, v, _ in rep.nearest) == {'separation': 125.0,
                                                       'dwell': 0.40}
    ok = nm.nominate(C('oval_1ball', 0.95, 150.0, 0.40, carry_zvd=True), GATE,
                     search=False)
    assert ok.ok and ok.worst().margin == pytest.approx(0.116713586, abs=TOL)


# ── one nomination per GUI pre-fill (juggle-panel.js PATTERN_POINTS), P4 ────

@pytest.mark.parametrize('config, worst, solves', [
    (C('self_toss', 0.9, 0.0, 0.27), 1.0, 12),
    (C('hop', 0.9, 250.0, 0.27), 0.206879713, 18),
    (C('columns', 0.95, 125.0, 0.27), 0.334698211, 18),
    (C('columns_1ball', 0.95, 125.0, 0.27), 0.333152066, 21),
    (C('columns_1ball_fed', 0.95, 125.0, 0.27), 0.334698211, 18),
    (C('columns_turned', 0.95, 125.0, 0.27, angle_deg=45.0), 0.035540900, 18),
    (C('columns_highlow', 1.0, 100.0, 0.27, apex_m_ball2=0.9), 0.136236957, 18),
    (C('oval_1ball', 0.95, 150.0, 0.65), 0.123371798, 12),
    (C('oval_1ball_fed', 0.95, 150.0, 0.65), 0.123371798, 12),
], ids=lambda x: x.pattern if isinstance(x, C) else None)
def test_every_gui_prefill_nominates_at_p4(config, worst, solves):
    rep = nm.nominate(config, P4, search=False)
    assert rep.ok, rep.lines()
    assert rep.solves == solves
    assert rep.worst().margin == pytest.approx(worst, abs=TOL)
    assert rep.summary().startswith('nomination OK: %s' % config.pattern)


# ── a refused twin per family, one parameter moved ──────────────────────────

@pytest.mark.parametrize('ok_config, field, value, refusal', [
    # hop: dwell 0.27 -> 0.17, the carried catch LIMIT_JERK leg 3 at -20 %
    (C('hop', 0.9, 250.0, 0.27), 'dwell_s', 0.17, (2, -0.20, 'LIMIT_JERK', 3)),
    # self-toss: apex 0.9 -> 1.5, the Hand at +10 % (legs never move: margin 1)
    (C('self_toss', 0.9, 0.0, 0.27), 'apex_m', 1.5, (1, 0.10, 'HAND_LIMIT_ACC', None)),
    # turned Columns 45 deg: dwell 0.27 -> 0.17, LIMIT_JERK leg 4
    (C('columns_turned', 0.95, 125.0, 0.27, angle_deg=45.0), 'dwell_s', 0.17,
     (4, 0.0, 'LIMIT_JERK', 4)),
    # one-ball Columns: dwell 0.27 -> 0.17, LIMIT_JERK leg 0
    (C('columns_1ball', 0.95, 125.0, 0.27), 'dwell_s', 0.17,
     (5, 0.0, 'LIMIT_JERK', 0)),
], ids=['hop', 'self_toss', 'columns_turned', 'columns_1ball'])
def test_a_refused_twin_one_parameter_away(ok_config, field, value, refusal):
    assert nm.nominate(ok_config, P4, search=False).ok
    rep = nm.nominate(dataclasses.replace(ok_config, **{field: value}), P4,
                      search=False)
    assert not rep.ok
    idx, frac, code, leg = refusal
    v = _refused(rep, idx, frac)
    assert (v.ok, v.code, v.leg) == (False, code, leg)
    assert any(line.startswith('skill %d ' % idx) for line in rep.lines())


# ── COMPILE and NOT_NOMINATED ────────────────────────────────────────────────

@pytest.mark.parametrize('config, text', [
    (C('columns', 0.95, 125.0, 0.90), 'leaves no transit'),
    (C('columns_highlow', 1.5, 100.0, 0.27, apex_m_ball2=0.4), 'has no transit'),
    (C('columns', 0.95, 125.0, 0.02), 'below the 4-knot floor'),
    (C('self_toss', 0.9, 0.0, 0.07), 'below the 4-knot floor'),
], ids=['dwell_ge_flight', 'highlow_no_transit', 'columns_window', 'self_toss_window'])
def test_compile_refusals_carry_the_compilers_text(config, text):
    with pytest.raises(ValueError) as exc:
        nm.compile_schedule(config)
    rep = nm.nominate(config, P4, search=False)
    assert not rep.ok and rep.solves == 0 and rep.verdicts == []
    assert rep.lines() == ['COMPILE: %s' % exc.value]
    assert text in rep.lines()[0]


@pytest.mark.parametrize('pattern, why', [
    ('magazine', 'NOT_NOMINATED: formulaic'),
    ('reload', 'NOT_NOMINATED: needs a live landing')])
def test_magazine_and_reload_are_not_nominated_and_not_searched(pattern, why):
    rep = nm.nominate(C(pattern, 0.9, 0.0, 0.27), P4)
    assert not rep.ok and rep.solves == 0
    assert len(rep.lines()) == 1 and rep.lines()[0].startswith(why)
    assert rep.nearest == [] and not rep.search_done and rep.searched == 0


# ── the search: bounded, deterministic, never names a refused configuration ─

def test_search_is_deterministic_and_every_nearest_passes_a_fresh_full_nomination():
    config = C('columns_highlow', 1.05, 125.0, 0.27, apex_m_ball2=0.95)
    runs = []
    for _ in range(2):
        nm.clear_cache()
        rep = nm.nominate(config, GATE)
        runs.append((rep.nearest, rep.search_trace, rep.searched,
                     rep.search_solves))
    assert runs[0] == runs[1]
    trials = nm.search_trials(config)
    assert rep.searched <= sum(len(t) for t in trials.values())
    assert all(len(t) <= nm.SEARCH_STEPS * (2 if p == 'dwell' else 1)
               for p, t in trials.items())
    for param, val, _summary in rep.nearest:
        nm.clear_cache()
        field = {'apex': 'apex_m', 'separation': 'separation_mm',
                 'dwell': 'dwell_s'}[param]
        full = nm.nominate(dataclasses.replace(config, **{field: val}), GATE,
                           search=False)
        assert full.ok, (param, val, full.lines())


def test_search_grid_and_workspace_rule():
    trials = nm.search_trials(C('columns_highlow', 1.0, 125.0, 0.27,
                                apex_m_ball2=0.9))
    assert [(t.apex_m, t.apex_m_ball2) for _, t in trials['apex']] == [
        (0.95, 0.85), (0.9, 0.8), (0.85, 0.75), (0.8, 0.7)]
    assert [t.separation_mm for _, t in trials['separation']] == [
        100.0, 75.0, 50.0, 25.0]
    assert [(d, t.dwell_s) for d, t in trials['dwell']][:4] == [
        ('down', 0.22), ('up', 0.32), ('down', 0.17), ('up', 0.37)]
    oval = nm.search_trials(C('oval_1ball', 0.95, 150.0, 0.30))
    assert [(d, t.dwell_s) for d, t in oval['dwell']] == [
        ('up', 0.35), ('up', 0.4), ('up', 0.45), ('up', 0.5)]
    assert 'separation' not in nm.search_trials(C('self_toss', 0.9, 0.0, 0.27))
    assert nm.workspace_ok(C('hop', 0.9, 250.0, 0.27))
    assert nm.workspace_ok(C('hop', 0.9, 300.0, 0.27))
    assert not nm.workspace_ok(C('hop', 0.9, 325.0, 0.27))


# ── the cache ────────────────────────────────────────────────────────────────

def test_cache_returns_the_same_report_without_solving(monkeypatch):
    nm.clear_cache()
    config = C('oval_1ball', 0.95, 150.0, 0.65)
    first = nm.nominate(config, P4, search=False)
    calls = []
    real = sg.plan_segment

    def counting(*a, **k):
        calls.append(1)
        return real(*a, **k)

    monkeypatch.setattr(sg, 'plan_segment', counting)
    again = nm.nominate(config, nm.sitting_limits(600, 8000, 200000, 4500))
    assert again is first and calls == []
    other = nm.nominate(config, GATE, search=False)
    assert other is not first and len(calls) == other.solves == 12 and other.aim_solves == 0


def test_a_refused_cached_report_searches_once(monkeypatch):
    nm.clear_cache()
    config = C('columns_1ball', 0.95, 125.0, 0.17)
    rep = nm.nominate(config, P4, search=False)
    assert not rep.ok and not rep.search_done
    searched = nm.nominate(config, P4)
    assert searched is rep and rep.search_done
    calls = []
    real = sg.plan_segment
    monkeypatch.setattr(sg, 'plan_segment',
                        lambda *a, **k: calls.append(1) or real(*a, **k))
    assert nm.nominate(config, P4) is rep and calls == []


# ── build_pattern mirrors skill_node ─────────────────────────────────────────

def test_build_pattern_mirrors_skill_node_site_picks():
    cols = nm.build_pattern(C('columns', 0.95, 125.0, 0.27), n_throws=4)
    assert isinstance(cols, sc.Pattern) and cols.n_throws == 4
    assert [s.name for s in cols.sites] == ['P2', 'P1']      # (a_site, fed P1)
    fed2 = nm.build_pattern(C('columns', 0.95, 125.0, 0.27, feed_site='P2'))
    assert [s.name for s in fed2.sites] == ['P1', 'P2']
    assert fed2.n_throws == nm.short_n_throws('columns') == 5
    turned = nm.build_pattern(C('columns_turned', 0.95, 125.0, 0.27,
                                angle_deg=45.0, feed_site='P2@45'))
    assert [s.name for s in turned.sites] == ['P1@45', 'P2@45']
    hl = nm.build_pattern(C('columns_highlow', 1.0, 100.0, 0.27, apex_m_ball2=0.9))
    assert (hl.apex_for(0), hl.apex_for(1)) == (1.0, 0.9)
    assert nm.build_pattern(C('columns_1ball_fed', 0.95, 125.0, 0.27)).phantom_balls == (0,)
    one = nm.build_pattern(C('columns_1ball', 0.95, 125.0, 0.27))
    assert one.phantom_balls == (1,) and [s.name for s in one.sites] == ['P2', 'P1']
    st_ = nm.build_pattern(C('self_toss', 0.9, 0.0, 0.27))
    assert isinstance(st_, sc.OneBallPattern) and st_.n_throws == 3
    assert list(st_.sites[0].cup_mm[:2]) == [-50.0, 0.0] and st_.sites[0].name == 'P1'
    assert nm.build_pattern(C('oval_1ball', 0.95, 150.0, 0.65)).roles == 'fixed'
    assert nm.build_pattern(C('hop', 0.9, 250.0, 0.27)).roles == 'alternate'
    with pytest.raises(ValueError):
        nm.build_pattern(C('magazine', 0.9, 0.0, 0.27))


# ── I-ADMIT-2: nothing silently viable, nothing skipped ─────────────────────

def test_a_nomination_that_walks_nothing_is_refused_by_name():
    """Empty ``rungs`` walk no Segment: refused ``NOT_NOMINATED: nothing to
    plan``, never vacuously viable, and not searched."""
    rep = nm.nominate(C('columns', 0.95, 125.0, 0.27), P4, rungs=())
    assert not rep.ok and rep.verdicts == [] and rep.solves == 0
    assert rep.lines() == [nm.NOTHING_TO_PLAN] == ['NOT_NOMINATED: nothing to plan']
    assert not rep.search_done and rep.nearest == []


def test_a_segment_with_no_seed_is_reported_not_planned_by_name():
    """Self-toss apex 2.0 at P4, nominal rung: the launch THROW is refused
    INFEASIBLE before it plans, so it has no release, and the PROXY (the same
    launch from rest) refuses too: every dependant is ``NOT_PLANNED``, named,
    with the seed's actual refusal."""
    rep = nm.nominate(C('self_toss', 2.0, 0.0, 0.27), P4, rungs=(0.0,),
                      search=False)
    assert not rep.ok
    assert [(v.idx, v.code) for v in rep.verdicts] == [
        (0, 'REST'), (1, 'INFEASIBLE'), (2, 'NOT_PLANNED'), (3, 'NOT_PLANNED'),
        (4, 'NOT_PLANNED'), (5, 'REST')]
    assert [v.message for v in rep.verdicts[2:5]] == [
        'seed refused at skill 1 (INFEASIBLE)',
        'seed refused at skill 2 (NOT_PLANNED)',
        'seed refused at skill 3 (NOT_PLANNED)']
    assert rep.lines()[1] == ('skill 2 CATCH Ball 1 at P1 -> P1, rung 0 %: '
                              'NOT_PLANNED [seed refused at skill 1 (INFEASIBLE)]')


def test_a_seed_refused_on_the_envelope_is_labelled_with_the_envelope(monkeypatch):
    """A producer that planned but failed the throw envelope seeds its
    dependant from its own release, and the label names ``ENVELOPE:<bound>``
    (not MARGIN).  The envelope is forced to refuse every Throw; a fresh
    geometry keeps the forced result out of the module cache."""
    from types import SimpleNamespace
    from jugglebot.motion.geometry import StewartGeometry
    monkeypatch.setattr(nm.throw_envelope, 'evaluate',
                        lambda flight_s, speed: SimpleNamespace(
                            ok=False, bound='FORCED', detail='forced by the test'))
    rep = nm.nominate(C('columns', 0.95, 125.0, 0.27), P4, StewartGeometry(),
                      rungs=(0.0,), search=False)
    v0, v1 = rep.verdicts[0], rep.verdicts[1]
    assert (v0.idx, v0.code) == (0, 'ENVELOPE:FORCED')
    assert v1.seed == 'release of skill 0 (itself refused ENVELOPE:FORCED)'
    assert v1.solved and v1.code == 'ENVELOPE:FORCED'


# ── JB-185: the re-aim figure is display only ────────────────────────────────
#
# Recipe (2026-11-06, Jetson, venv python 3.8, ONE BLAS thread):
#   OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python -m pytest \
#       tests/motion/test_skills_nominate.py -q -p no:cacheprovider
# Pinned figures are the probe's at 600 / 8000 / 200000, Hand 4500 (to 1e-3).

_AIM_GRID = [
    (C('columns', 0.95, 125.0, 0.27), P4),                 # accepted
    (C('columns', 1.0, 170.0, 0.27), P4),                  # accepted, thin
    (C('hop', 0.90, 250.0, 0.27), P4),
    (C('oval_1ball', 0.95, 150.0, 0.30), P4),              # refused, searched
    (C('columns_highlow', 1.0, 125.0, 0.27, apex_m_ball2=0.9), GATE),  # refused
    (C('self_toss', 0.60, 0.0, 0.27), P4),                 # no Leg motion
]


def _outcome(rep):
    """Everything acceptance, the refusal lines and the search are made of,
    margins left out (the QP is deterministic only to ~1e-12 run to run, so
    they are compared with a tolerance below)."""
    return (rep.ok, rep.refusals, rep.nearest, rep.solves,
            [(v.idx, v.flight_frac, v.code, v.leg, v.t_s, v.seed)
             for v in rep.verdicts])


@pytest.mark.parametrize('config, limits', _AIM_GRID,
                         ids=[c.pattern + '-%g-%g' % (c.apex_m, c.separation_mm)
                              for c, _ in _AIM_GRID])
def test_reaim_figure_never_changes_accept_refuse_search_or_margins(config, limits):
    """Acceptance, every refusal line, the nearest-viable search and the
    nominal verdicts (margins bit for bit) are the same with the re-aim
    figure on (default) and off (``aim=None``, the pre-JB-185 path)."""
    nm.clear_cache()
    off = nm.nominate(config, limits)
    nm.clear_cache()
    on = nm.nominate(config, limits, aim=nm.LAUNCH_AIM)
    assert _outcome(on) == _outcome(off)
    assert [v.margin for v in on.verdicts] == pytest.approx(
        [v.margin for v in off.verdicts], abs=1e-9, nan_ok=True)
    if on.ok:
        assert on.summary().split('; worst margin')[0] \
            == off.summary().split('; worst margin')[0]
    else:
        assert on.summary() == off.summary() and on.aim_verdicts == []


def test_reaim_figure_is_a_floor_below_the_nominal_and_rounds_out_the_line():
    nm.clear_cache()
    rep = nm.nominate(C('columns', 1.0, 170.0, 0.27), P4, search=False,
                      aim=nm.LAUNCH_AIM)
    assert rep.ok
    assert rep.worst().margin == pytest.approx(0.211, abs=1e-3)
    assert rep.aim_worst().margin == pytest.approx(0.016, abs=1e-3)
    assert rep.aim_worst().margin < rep.worst().margin
    assert not rep.aim_below_zero()
    assert NEGATIVE not in rep.summary()
    assert rep.aim_solves == 2 * 6 and rep.solves == 18   # worst rung only, 2 signs
    # the plain wording of the operator's line
    assert ('worst margin 0.211 at the nominal Sites (jerk, CATCH Ball 1 at P2, '
            'rung -20 %), 0.016 with the Catch re-aim at its limit (') in rep.summary()


NEGATIVE = nm.AIM_NEGATIVE_TEXT


def test_reaim_below_zero_is_still_accepted_and_says_so():
    """Columns Apex 0.80 / Sep. 100 / Dwell 0.27 at the generated launch limits
    (the JB-185 sim case): nominal 0.060 (accepted, as before), re-aim figure
    negative, the one added sentence names the consequence."""
    nm.clear_cache()
    rep = nm.nominate(C('columns', 0.80, 100.0, 0.27), search=False,
                      aim=nm.LAUNCH_AIM)
    assert rep.ok and rep.refusals == []
    assert rep.worst().margin == pytest.approx(0.060, abs=1e-3)
    assert rep.aim_worst().margin < 0.0 and rep.aim_below_zero()
    assert rep.summary().endswith('. ' + NEGATIVE)
    assert 'An Attempt may end mid-Pattern on an admissibility refusal.' in rep.summary()


def test_one_site_pattern_plans_no_second_walk():
    """Self-toss (no outward direction, no Leg motion): the line is the old
    one, nothing was walked twice."""
    nm.clear_cache()
    rep = nm.nominate(C('self_toss', 0.60, 0.0, 0.27), P4, search=False,
                      aim=nm.LAUNCH_AIM)
    assert rep.ok and rep.aim_verdicts == [] and rep.aim_solves == 0
    assert rep.summary().startswith(
        'nomination OK: self_toss apex 0.60 m, dwell 0.27 s, pose; worst margin 1.000 (')
    assert 'at the nominal Sites' not in rep.summary()


def test_aim_authority_is_the_launch_table_and_the_clamp_is_the_executors():
    """No copied literals: the default authorities ARE
    ``executor.LAUNCH_LATERAL_AUTHORITY_MM`` and the displacement is what
    ``SkillExecutor._lateral_clip_mm`` returns for the corner ask: OUTWARD
    ``catch_outward_authority_mm`` along the Site's outward unit, ACROSS the
    lateral authority; the Learner's shift is the lateral box."""
    from jugglebot.motion.skills import executor as ex
    t = ex.LAUNCH_LATERAL_AUTHORITY_MM
    assert nm.LAUNCH_AIM == nm.AimAuthority(
        t['learner_lateral_authority_mm'], t['catch_outward_authority_mm'],
        t['catch_inward_authority_mm'])
    sched = nm.compile_schedule(C('columns', 0.95, 125.0, 0.27))
    sh = nm._AimShifts(sched, nm.LAUNCH_AIM, +1)
    p1 = sched.skills[1].site
    from jugglebot.motion.skills.executor import SkillExecutor
    u = SkillExecutor._outward_unit_at(sh.stub, p1)
    d = sh.catch_mm(p1)
    assert float(d[:2] @ u) == pytest.approx(t['catch_outward_authority_mm'])
    assert abs(float(d[0] * -u[1] + d[1] * u[0])) == pytest.approx(
        t['learner_lateral_authority_mm'])
    # the two signs are mirror corners across the axis
    d2 = nm._AimShifts(sched, nm.LAUNCH_AIM, -1).catch_mm(p1)
    assert float(d2[:2] @ u) == pytest.approx(float(d[:2] @ u))
    assert float(d2[0] * -u[1] + d2[1] * u[0]) == pytest.approx(
        -float(d[0] * -u[1] + d[1] * u[0]))
    assert np.allclose(sh.learner_mm(p1)[:2],
                       np.clip((u + np.array([-u[1], u[0]])) * 1e6,
                               -t['learner_lateral_authority_mm'],
                               t['learner_lateral_authority_mm']))


def test_the_aim_figure_is_not_part_of_the_cache_key_and_never_walks_twice(monkeypatch):
    """The plain nomination (goal accept) and the aim nomination share ONE
    cached report; a second ask for the same authorities solves nothing, a
    different authority re-walks only the figure; `has_aim_for` is a pure read."""
    nm.clear_cache()
    cfg = C('columns', 0.95, 125.0, 0.27)
    a = nm.nominate(cfg, P4, search=False, aim=nm.LAUNCH_AIM)
    other = nm.AimAuthority(30.0, 0.0, 35.0)
    assert a.has_aim_for(nm.LAUNCH_AIM) and not a.has_aim_for(other)
    calls = []
    real = nm.sg.plan_segment
    monkeypatch.setattr(nm.sg, 'plan_segment',
                        lambda *x, **k: calls.append(1) or real(*x, **k))
    assert nm.nominate(cfg, P4, search=False) is a
    assert nm.nominate(cfg, P4, search=False, aim=nm.LAUNCH_AIM) is a
    assert calls == []
    worst_before = a.aim_worst().margin
    assert nm.nominate(cfg, P4, search=False, aim=other) is a and calls
    assert a.has_aim_for(other) and a.aim_worst().margin > worst_before


def test_accept_path_performs_zero_second_walk_solves(monkeypatch):
    """Counting every `plan_segment` call: a plain `nominate` of an accepted
    config makes exactly the nominal walk's `solves` (12 at the gate limits),
    `aim_solves` stays 0, and a repeat is free."""
    nm.clear_cache()
    calls = []
    real = nm.sg.plan_segment
    monkeypatch.setattr(nm.sg, 'plan_segment',
                        lambda *x, **k: calls.append(1) or real(*x, **k))
    rep = nm.nominate(C('columns', 0.95, 125.0, 0.27), P4, search=False)
    assert rep.ok and len(calls) == rep.solves and rep.aim_solves == 0
    n = len(calls)
    nm.nominate(C('columns', 0.95, 125.0, 0.27), P4, search=False)
    assert len(calls) == n


def test_worst_rung_walk_equals_the_full_walk_on_the_probe_cases():
    """The choice behind AIM_FULL_WALK = False: on the four cases probed
    (2026-11-06) the one-rung walk returns the full walk's figure."""
    assert nm.AIM_FULL_WALK is False
    lim = nm.default_limits()
    for cfg, limits in ((C('columns', 1.0, 170.0, 0.27), P4),
                        (C('columns', 0.80, 100.0, 0.27), lim)):
        nm.clear_cache()
        rep = nm.nominate(cfg, limits, search=False)
        figs = []
        for full in (False, True):
            nm.add_aim_figure(rep, limits, nm._default_geom(), nm.RUNGS,
                              nm.LAUNCH_AIM, full=full)
            figs.append(rep.aim_worst().margin)
        assert figs[0] == pytest.approx(figs[1], abs=0.01)
