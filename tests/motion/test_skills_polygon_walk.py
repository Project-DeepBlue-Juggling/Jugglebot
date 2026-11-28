"""JP-5 polygon walk (IDEA-9, pattern-progression P7): the offline half.

One Ball fed by Ball Butler is thrown Site to Site around a regular N-gon
(``schedule.OneBallPattern(roles='polygon')``, ``sites.polygon_sites``), the
first Pattern whose Site changes every Throw.  It rides the one-ball compiler
(``compile_one_ball`` / ``compile_reload``): the two-ball ``BeatEvent`` stream
cannot express it (ball ids alternate, the Beat is half a cycle).

Recipe (every pinned number), 2026-11-18, venv python 3.8.10, ONE BLAS thread::

    OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python -m pytest \
        tests/motion/test_skills_polygon_walk.py -q -p no:cacheprovider

Margins come from ``/tmp/probe_nom.py`` (uncommitted): ``nominate.nominate`` at
the ``P4`` limits (600 / 8000 / 200000 mm/s^n, Hand 4500), pose planner, three
rungs, ``search=False``.  A square of edge 250 mm (the Hop's separation) is
INFEASIBLE: the square turns 90 degrees at every Throw where the Hop reverses.
"""
from __future__ import annotations

import math

import numpy as np
import pytest

from jugglebot.motion.skills import nominate as nm
from jugglebot.motion.skills import schedule as sc
from jugglebot.motion.skills import sites as st

C = nm.NominationConfig
P4 = nm.sitting_limits(600.0, 8000.0, 200000.0, 4500.0)


def _poly(n=4, edge=100.0, n_throws=9, **kw):
    return sc.OneBallPattern(sites=st.polygon_sites(n, edge), apex_m=0.9,
                             dwell_s=0.27, n_throws=n_throws, roles='polygon',
                             **kw)


# ── geometry ─────────────────────────────────────────────────────────────────

@pytest.mark.parametrize('n,edge', [(3, 100.0), (4, 100.0), (4, 125.0), (6, 80.0)])
def test_polygon_sites_are_a_regular_ngon_centred_on_the_platform(n, edge):
    sites = st.polygon_sites(n, edge)
    xy = np.array([s.cup_mm[:2] for s in sites])
    assert len(sites) == n and [s.name for s in sites] == ['P%d' % (k + 1) for k in range(n)]
    assert np.allclose(xy.mean(axis=0), 0.0, atol=1e-9)
    side = [np.linalg.norm(xy[(k + 1) % n] - xy[k]) for k in range(n)]
    assert np.allclose(side, edge)
    # P1 is where columns_sites puts its P1 (the Feed Site, -x)
    assert xy[0, 0] < 0 and abs(xy[0, 1]) < 1e-9
    assert all(s.cup_mm[2] == st.CATCH_CUP_Z_MM for s in sites)


def test_polygon_sites_refuse_what_is_not_a_polygon():
    with pytest.raises(ValueError):
        st.polygon_sites(2, 100.0)
    with pytest.raises(ValueError):
        st.polygon_sites(4, 0.0)


# ── the compile ──────────────────────────────────────────────────────────────

def test_square_visits_n_distinct_sites_in_order_and_ends_rest_terminal():
    pat = _poly(4, 100.0, 9)
    sch = sc.compile_one_ball(pat, 10.0)
    assert sch.pattern == sc.POLYGON_WALK == 'polygon_walk'
    sites = pat.sites
    throws = []
    for sk in sch.skills:
        if sk.kind == sc.THROW:
            throws.append((sk.site, sk.target))
        if sk.then_throw is not None:           # a folded carried Throw
            throws.append((sk.site, sk.then_throw.target))
    assert len(throws) == 9
    for i, (site, target) in enumerate(throws):
        assert site.name == sites[i % 4].name, i
        assert target.name == sites[(i + 1) % 4].name, i
    assert len({s.name for s, _ in throws}) == 4
    # opens and closes on a REST (every non-REST Segment is walked rest-terminal
    # by the nomination test below)
    assert sch.skills[0].kind == sc.REST and sch.skills[0].site.name == 'P1'
    assert sch.skills[-1].kind == sc.REST


def test_the_feed_enters_at_site_zero():
    pat = _poly(4, 100.0, 9)
    p0 = pat.sites[0]
    land = np.asarray(p0.catch_site_mm(), dtype=float)
    sch = sc.compile_reload(land, np.array([0.0, 0.0, -4000.0]), 11.0, pat, 5.0)
    assert sch.pattern == 'polygon_walk'
    catches = [s for s in sch.skills if s.kind == sc.CATCH]
    assert catches[0].site.name == 'P1'
    # the first Throw after the Feed leaves P1 for P2
    first_throw = next(s for s in sch.skills
                       if s.kind == sc.THROW or s.then_throw is not None)
    assert first_throw.site.name in ('P1', 'P2')


# ── nothing else moved ───────────────────────────────────────────────────────

def test_existing_labels_and_site_count_rules_are_unchanged():
    two = st.columns_sites(250.0)
    one = (two[0],)
    assert sc.compile_one_ball(sc.OneBallPattern(
        sites=one, apex_m=0.9, dwell_s=0.27, n_throws=3), 10.0).pattern == 'self_toss'
    assert sc.compile_one_ball(sc.OneBallPattern(
        sites=two, apex_m=0.9, dwell_s=0.27, n_throws=3), 10.0).pattern == 'hop'
    assert sc.compile_one_ball(sc.OneBallPattern(
        sites=two, apex_m=0.9, dwell_s=0.27, n_throws=3, roles='fixed'),
        10.0).pattern == 'oval_1ball'
    p0 = two[0]
    land = np.asarray(p0.catch_site_mm(), dtype=float)
    hop = sc.compile_reload(land, np.array([0.0, 0.0, -4000.0]), 11.0,
                            sc.OneBallPattern(sites=two, apex_m=0.9, dwell_s=0.27,
                                              n_throws=3), 5.0)
    assert hop.pattern == 'hop'
    # three Sites on any non-polygon role is still refused, and a polygon of two is
    three = st.polygon_sites(3, 100.0)
    for roles in ('alternate', 'fixed'):
        with pytest.raises(ValueError):
            sc.OneBallPattern(sites=three, apex_m=0.9, dwell_s=0.27, n_throws=3,
                              roles=roles)
    with pytest.raises(ValueError):
        sc.OneBallPattern(sites=two, apex_m=0.9, dwell_s=0.27, n_throws=3,
                          roles='polygon')
    with pytest.raises(ValueError):
        sc.OneBallPattern(sites=three, apex_m=0.9, dwell_s=0.27, n_throws=3,
                          roles='polygon', landing_offset_m=(0.01, 0.0))


# ── the nomination (the existing admissibility check, as it is) ──────────────

@pytest.mark.parametrize('edge,dwell,worst', [
    (100.0, 0.27, 0.188923959),
    # # RE-PINNED 2026-11-28 (BUG-159, I-PLAN-18): 0.199972183 -> 0.441335830 (the worst moved from CATCH
    # Ball 1 at P2, skill 10, to THROW Ball 1 at P1, skill 1)
    (125.0, 0.45, 0.441335830),
])
def test_the_square_nominates_and_every_segment_is_planned(edge, dwell, worst):
    rep = nm.nominate(C('polygon_walk', 0.9, edge, dwell), P4, search=False)
    assert rep.ok, rep.lines()
    w = rep.worst()
    assert w.margin == pytest.approx(worst, abs=1e-6)
    walked = [v for v in rep.verdicts if v.kind != sc.REST]
    assert walked and all(v.solved and v.ok for v in walked)
    # nine Throws: every Site is released from at least twice
    assert {v.site for v in walked} == {'P1', 'P2', 'P3', 'P4'}


def test_the_hops_250mm_edge_is_not_flown_by_a_square():
    rep = nm.nominate(C('polygon_walk', 0.9, 250.0, 0.27), P4, search=False)
    assert not rep.ok
    assert any('INFEASIBLE' in r for r in rep.refusals), rep.refusals


def test_polygon_n_selects_the_site_count():
    pat = nm.build_pattern(C('polygon_walk', 0.9, 100.0, 0.27, polygon_n=3))
    assert len(pat.sites) == 3 and pat.roles == 'polygon' and pat.n_throws == 7
    assert nm.workspace_ok(C('polygon_walk', 0.9, 100.0, 0.27))
    assert not nm.workspace_ok(C('polygon_walk', 0.9, 4000.0, 0.27))


# ── Memory: a first visit to a Site ──────────────────────────────────────────

def test_a_first_visit_to_a_site_commands_the_plant_model_inverse_not_a_memory_row():
    """Memory is keyed on release xy and velocity (learner.py, kernel d2 <= 9
    with h_p 0.05 m, h_v 0.15 m/s).  With NO rows the Learner commands exactly
    the commanded landing (the default PlantModel is the identity).  Against
    the 5264-row corpus (probe 2026-11-18, /tmp/probe_mem.py) a never-visited
    Site's evidence is ~0 (sum of weights <= 0.035, 0 rows at P3) and the
    command is the corpus-fitted plant model's inverse, 4-10 mm off in xy
    (inside the 20 mm lateral authority); that corpus run is not pinned here
    (the file changes every Sitting)."""
    from jugglebot.motion.skills import learner as lr
    from jugglebot.motion.trajectory import ballistics_bc as bal
    L = lr.Learner()
    sites = st.polygon_sites(4, 100.0)
    tf = sc.flight_s(0.9)
    for k in range(4):
        s, t = sites[k], sites[(k + 1) % 4]
        rel = s.throw_site_mm()
        lv = bal.launch_velocity(rel, t.catch_site_mm(), tf)
        ctx = lr.ThrowContext(site_xy_m=s.cup_mm[:2] / 1000.0,
                              release_m=rel / 1000.0,
                              release_vel_mps=np.asarray(lv) / 1000.0,
                              dwell_s=0.27, ball_id=0,
                              release_kind='carried' if k else 'free_cup')
        y_d = np.array([0.0, 0.0, 0.9])
        assert L.evidence(ctx) == (0.0, 0)
        assert np.allclose(L.command(ctx, y_d), y_d)
