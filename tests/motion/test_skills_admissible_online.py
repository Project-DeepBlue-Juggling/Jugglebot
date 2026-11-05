"""P3a command-time admissibility check: ``admissible.margin``,
``admissible.resolve_command`` and ``executor.check_segment``
(``plans/active/learner-generalisation.md`` P3a; design rows T-U7 and PIN).

Recipes (real planner everywhere except the scripted call-count table):

* **margin** -- the probe's own formula (``online_check_replay.margin``), and
  ``nan`` for a missing report.  Fed a synthetic report object: the formula is
  arithmetic, the real reports are covered by the PIN.
* **resolve_command call counts** -- a SCRIPTED ``verdict_fn`` (a dict from
  command to verdict, every call recorded), because "model refused", "confirm
  fails" and "unevaluated" are not reachable on the probe's cells: the nominal
  command passes on every one of them.  The real-planner side of T-U7 is the
  PIN's old-point half below.
* **PIN against the probe** -- the probe's part B (``online_check_replay.py``):
  a columns catch-with-throw cell at site (62.5, 0) mm, partner (-62.5, 0) mm,
  pattern Apex 0.95 and 1.05 m, Dwell 0.27 s, pose planner, seeded by the
  partner's launch THROW's release state; 48 candidates on rays of 10/20/30 mm
  x 8 directions.  Fixtures GENERATED ON THIS JETSON by the probe, 2026-11-05:
  ``python tools/probes/admissible_obviation/online_check_replay.py``
  (P4 point 600 / 8000 / 200000 / 4500: 48 pass outright) and the same with
  ``--skip-a --limits 350 5000 200000 4500`` (the OLD point: 33 pass, 15 are
  refused MARGIN and step).  What drives the real refusal at the old point is
  the leg velocity / acceleration headroom at 350 mm/s, 5000 mm/s^2: the
  offset landing makes the catch-with-throw move the Platform further.
  The cell's seed and timing are the probe's own ``Cell`` (IMPORTED, not
  rebuilt: the pin then shares one cell builder with the probe, so a drift in
  the probe's seeding fails here instead of two builders diverging silently);
  only the plan call is swapped for production ``executor.check_segment``
  (fresh origin at ``t_now_s = 0`` from that seed, the terminal's absolute
  times then ARE the probe's segment times).
* **check_segment through a record** -- ``tools/probes/admissible_obviation/
  loaded_budget.py``'s ``Chain`` recipe in process: a launch THROW from rest at
  the partner Site installed with ``install_segment``, then a catch-with-throw
  at the other Site dispatched ``schedule.HANDOFF_LEAD_S`` before that release
  (the splice snaps to the release knot and seeds post-release).
"""

from __future__ import annotations

import importlib.util
import json
import math
import os
import pickle

import numpy as np
import pytest

from jugglebot.motion import unified_cycle as uc
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.skills import admissible as adm
from jugglebot.motion.skills import executor as ex
from jugglebot.motion.skills import magazine_throw as mt
from jugglebot.motion.skills import schedule as sc
from jugglebot.motion.skills import segments as sg
from jugglebot.motion.skills import sites as st
from jugglebot.motion.trajectory import ballistics_bc as bal

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
_PROBE = os.path.join(_ROOT, 'tools', 'probes', 'admissible_obviation',
                      'online_check_replay.py')
_FIX = os.path.join(_ROOT, 'tests', 'fixtures')
_P4_FIXTURE = os.path.join(_FIX, 'online_check_replay_p4_jetson_2026-11-05.json')
_OLD_FIXTURE = os.path.join(
    _FIX, 'online_check_replay_old_point_jetson_2026-11-05.json')


def _load_probe():
    spec = importlib.util.spec_from_file_location('online_check_replay', _PROBE)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope='module')
def probe():
    return _load_probe()


# ---------------------------------------------------------------------------
# (1) margin
# ---------------------------------------------------------------------------

class _Report:
    def __init__(self, v, a, j):
        self.peak_leg_vel_mmps = v
        self.peak_leg_acc_mmps2 = a
        self.peak_leg_jerk_mmps3 = j


class _Limits:
    leg_vel_mmps = 600.0
    leg_acc_mmps2 = 8000.0
    leg_jerk_mmps3 = 200000.0


@pytest.mark.parametrize('peaks', [(300.0, 1000.0, 1000.0),
                                   (100.0, 7000.0, 1000.0),
                                   (100.0, 1000.0, 190000.0),
                                   (540.0, 7200.0, 180000.0)])
def test_margin_is_the_probe_formula(probe, peaks):
    rep = _Report(*peaks)
    assert adm.margin(rep, _Limits) == probe.margin(rep, _Limits)
    f = adm.MARGIN_FRAC
    want = min((f * 600.0 - peaks[0]) / (f * 600.0),
               (f * 8000.0 - peaks[1]) / (f * 8000.0),
               (f * 200000.0 - peaks[2]) / (f * 200000.0))
    assert adm.margin(rep, _Limits) == want
    # exactly at 0.9 L on the binding channel the margin is 0: admitted.
    assert peaks != (540.0, 7200.0, 180000.0) or adm.margin(rep, _Limits) == 0.0


def test_margin_is_nan_without_a_report():
    assert math.isnan(adm.margin(None, _Limits))


# ---------------------------------------------------------------------------
# (2) resolve_command: the call-count table (scripted verdicts)
# ---------------------------------------------------------------------------

def _key(u):
    return (round(float(u[0][0]), 9), round(float(u[0][1]), 9),
            round(float(u[1]), 9))


class _Script:
    """``verdict_fn`` from a table: an exact command -> verdict, plus a
    ``default`` for any other command (the stepped point).  Records calls."""

    def __init__(self, table, default=None):
        self.table = {_key(u): v for u, v in table}
        self.default = default
        self.calls = []

    def __call__(self, u):
        self.calls.append((np.array(u[0], dtype=float), float(u[1])))
        v = self.table.get(_key(u), self.default)
        assert v is not None, 'unscripted command %r' % (u,)
        return v


OK = adm.CommandVerdict(True, 0.20, 'OK', True)
U_MODEL = (np.zeros(2), 1.00)
U_CAND = (np.array([0.020, 0.0]), 1.10)


def test_a_passing_candidate_costs_one_call():
    fn = _Script([(U_CAND, OK)])
    res = adm.resolve_command(fn, U_CAND, U_MODEL)
    assert res.how == 'candidate' and res.solves == 1 and len(fn.calls) == 1
    assert np.array_equal(res.u[0], U_CAND[0]) and res.u[1] == U_CAND[1]
    assert res.verdicts == (OK,)


def test_a_refused_candidate_steps_in_three_calls_with_the_probe_arithmetic():
    refused = adm.CommandVerdict(False, -0.10, 'MARGIN', True)
    model = adm.CommandVerdict(True, 0.30, 'OK', True)
    stepped = adm.CommandVerdict(True, 0.01, 'OK', True)
    fn = _Script([(U_CAND, refused), (U_MODEL, model)], default=stepped)
    res = adm.resolve_command(fn, U_CAND, U_MODEL, backoff_m=0.001)
    assert res.how == 'stepped' and res.solves == 3 and len(fn.calls) == 3
    s = 0.30 / (0.30 + 0.10)                    # the margin's zero: 0.75
    s2 = s - 0.001 / 0.020                      # back off 1 mm of 20 mm
    assert res.u[0] == pytest.approx([0.020 * s2, 0.0], abs=1e-15)
    assert res.u[1] == pytest.approx(1.00 + s2 * 0.10, abs=1e-15)   # apex too
    assert res.verdicts == (refused, model, stepped)
    # the calls were candidate, model, stepped -- in that order
    assert fn.calls[1][1] == 1.00 and np.array_equal(fn.calls[1][0], np.zeros(2))


def test_no_usable_slope_halves():
    for m_u, m0 in ((float('nan'), 0.3), (0.05, 0.3), (-0.1, float('nan'))):
        refused = adm.CommandVerdict(False, m_u, 'X', True)
        model = adm.CommandVerdict(True, m0, 'OK', True)
        fn = _Script([(U_CAND, refused), (U_MODEL, model)], default=OK)
        res = adm.resolve_command(fn, U_CAND, U_MODEL, backoff_m=0.0)
        assert res.how == 'stepped' and res.solves == 3
        assert res.u[0] == pytest.approx([0.010, 0.0], abs=1e-15)
        assert res.u[1] == pytest.approx(1.05, abs=1e-15)


def test_a_failed_confirm_returns_the_model_command_and_never_a_fourth_call():
    refused = adm.CommandVerdict(False, -0.10, 'MARGIN', True)
    model = adm.CommandVerdict(True, 0.30, 'OK', True)
    fn = _Script([(U_CAND, refused), (U_MODEL, model)],
                 default=adm.CommandVerdict(False, -0.01, 'MARGIN', True))
    res = adm.resolve_command(fn, U_CAND, U_MODEL)
    assert res.how == 'model' and res.solves == 3 and len(fn.calls) == 3
    assert np.array_equal(res.u[0], U_MODEL[0]) and res.u[1] == U_MODEL[1]


def test_a_backoff_that_consumes_the_line_returns_the_model_in_two_calls():
    refused = adm.CommandVerdict(False, -0.10, 'MARGIN', True)
    model = adm.CommandVerdict(True, 0.001, 'OK', True)     # s ~ 0.0099
    fn = _Script([(U_CAND, refused), (U_MODEL, model)])
    res = adm.resolve_command(fn, U_CAND, U_MODEL, backoff_m=0.001)
    assert res.how == 'model' and res.solves == 2 and len(fn.calls) == 2


def test_a_refused_model_command_raises_after_two_calls():
    refused = adm.CommandVerdict(False, -0.10, 'MARGIN', True)
    fn = _Script([(U_CAND, refused),
                  (U_MODEL, adm.CommandVerdict(False, -0.02, 'MARGIN', True))])
    with pytest.raises(adm.AdmissibleError, match='model command'):
        adm.resolve_command(fn, U_CAND, U_MODEL)
    assert len(fn.calls) == 2


def test_an_unevaluated_check_fails_closed_after_one_call():
    fn = _Script([(U_CAND, adm.CommandVerdict(False, float('nan'),
                                              'SOLVER_UNAVAILABLE', False))])
    with pytest.raises(adm.AdmissibleError, match='SOLVER_UNAVAILABLE'):
        adm.resolve_command(fn, U_CAND, U_MODEL)
    assert len(fn.calls) == 1


def test_a_refused_candidate_equal_to_the_model_raises_after_one_call():
    fn = _Script([(U_MODEL, adm.CommandVerdict(False, -0.05, 'MARGIN', True))])
    with pytest.raises(adm.AdmissibleError, match='MARGIN'):
        adm.resolve_command(fn, (np.zeros(2), 1.00), U_MODEL)
    assert len(fn.calls) == 1


# ---------------------------------------------------------------------------
# (3) PIN: production check_segment + resolve_command reproduce the probe
# ---------------------------------------------------------------------------

_CELL_SITE_MM = (62.5, 0.0)


def _check(probe_mod, cell, xy_m, apex_m):
    """One production check of the probe's cell at offset ``xy_m`` (metres)
    and commanded Apex ``apex_m``: the probe's ``Cell.solve`` terminal, planned
    by ``executor.check_segment`` from a fresh origin at ``t_now_s = 0``."""
    off = np.asarray(xy_m, dtype=float) * 1000.0
    tau = sc.transit_s(cell.T, probe_mod.DWELL)
    landing = cell.site.catch_site_mm() + np.array([off[0], off[1], 0.0])
    then = sg.ThrowAfterCatch(t_release_s=tau + probe_mod.DWELL,
                              site_mm=cell.site.throw_site_mm(),
                              target_mm=landing, flight_s=sc.flight_s(apex_m))
    term = sg.CatchTerminal(
        landing_mm=landing,
        landing_vel_mm_s=bal.arrival_velocity(cell.takeoff, cell.T),
        t_land_s=tau, rest_site_mm=cell.site.rest_site_mm(), then_throw=then)
    return ex.check_segment(None, cell.seed, sg.CATCH, term, 0.0,
                            cfg=cell.cfg, limits=cell.limits, geom=cell.geom)


def _verdict(chk) -> adm.CommandVerdict:
    if not chk.planned:
        return adm.CommandVerdict(False, chk.margin, chk.code, True)
    ok = chk.margin >= 0.0
    return adm.CommandVerdict(ok, chk.margin, 'OK' if ok else 'MARGIN', True)


def _pin(probe_mod, fixture_path):
    fx = json.load(open(fixture_path))
    limits = probe_mod.asw._limits(*fx['limits'])
    cfg = sg.SegmentConfig(planner_mode=fx['mode'])
    geom = StewartGeometry()
    cells = {}
    out = []
    for row in fx['two_solve']:
        apex = float(row['apex'])
        if apex not in cells:
            cells[apex] = probe_mod.Cell(_CELL_SITE_MM, apex, limits, geom, cfg)
            assert cells[apex].ok, cells[apex].code
        cell = cells[apex]
        calls = []

        def verdict_fn(u, cell=cell, calls=calls):
            v = _verdict(_check(probe_mod, cell, u[0], u[1]))
            calls.append(v)
            return v

        r, ang = float(row['r']), math.radians(float(row['ang']))
        u_cand = (np.array([r * math.cos(ang), r * math.sin(ang)]) / 1000.0, apex)
        res = adm.resolve_command(verdict_fn, u_cand, (np.zeros(2), apex))
        assert res.solves == len(calls)
        out.append((row, res))
    return out


def test_pin_p4_point_every_candidate_passes_as_the_probe_found(probe):
    rows = _pin(probe, _P4_FIXTURE)
    assert len(rows) == 48
    for row, res in rows:
        cand = res.verdicts[0]
        assert cand.code == row['candidate_code'] == 'OK', row
        assert cand.margin == pytest.approx(row['candidate_m'], abs=1e-6), row
        assert res.how == 'candidate' and res.solves == 1


def test_pin_old_point_15_refused_candidates_step_in_three_solves(probe):
    """The probe stepped with the affinely PREDICTED candidate margin;
    production uses the candidate's REAL margin.  On these 15 they agree to
    0.003, so the stepped distance is pinned to 0.5 mm of the probe's, not
    exactly.  The candidate code and margin are pinned exactly (1e-6)."""
    rows = _pin(probe, _OLD_FIXTURE)
    assert len(rows) == 48
    stepped = 0
    for row, res in rows:
        cand = res.verdicts[0]
        assert cand.code == row['candidate_code'], row
        assert cand.margin == pytest.approx(row['candidate_m'], abs=1e-6), row
        if row['real_solves'] == 1:
            assert res.how == 'candidate' and res.solves == 1, row
            continue
        stepped += 1
        assert row['confirm_code'] == 'OK'
        assert res.how == 'stepped' and res.solves == 3, (row, res.verdicts)
        assert res.verdicts[2].margin >= 0.0
        dist_mm = float(np.linalg.norm(res.u[0])) * 1000.0
        assert dist_mm == pytest.approx(row['r'] * row['step_frac'], abs=0.5), row
    assert stepped == 15


# ---------------------------------------------------------------------------
# (4) check_segment through a record == the install's own plan
# ---------------------------------------------------------------------------

_P4_LIMITS = (600.0, 8000.0, 200000.0, 4500.0)
_APEX = 0.95
_DWELL = 0.27


@pytest.fixture(scope='module')
def chain(probe):
    asw = probe.asw
    limits = asw._limits(*_P4_LIMITS)
    geom = StewartGeometry()
    cfg = sg.SegmentConfig(planner_mode='pose')
    site = st.Site('S', np.array([62.5, 0.0, st.CATCH_CUP_Z_MM]))
    partner = st.Site('Q', np.array([-62.5, 0.0, st.CATCH_CUP_Z_MM]))
    T = sc.flight_s(_APEX)
    ok, code, _seed, takeoff = asw._throw_cell(
        partner, partner, T, asw.SINGLE_SITE_LAUNCH_S, limits, geom, cfg)
    assert ok, code
    t0 = 1000.0
    t_rel = t0 + asw.SINGLE_SITE_LAUNCH_S
    record, res, seg = ex.install_segment(
        None, asw._rest_state(partner.rest_site_mm()), sg.THROW,
        sg.ThrowTerminal(site_mm=partner.throw_site_mm(),
                         target_mm=partner.throw_site_mm(), flight_s=T,
                         t_release_s=t_rel),
        t0, lead_s=ex.LEAD_S, cfg=cfg, limits=limits, geom=geom)
    assert res.accepted, res.message
    return dict(limits=limits, geom=geom, cfg=cfg, site=site, T=T,
                v_arrival=bal.arrival_velocity(takeoff, T), t_rel=t_rel,
                record=record, warm=seg.warm_start)


def _catch_terminal(c, off_mm=(0.0, 0.0)):
    landing = c['site'].catch_site_mm() + np.array([off_mm[0], off_mm[1], 0.0])
    t_land = c['t_rel'] + sc.transit_s(c['T'], _DWELL)
    then = sg.ThrowAfterCatch(t_release_s=t_land + _DWELL,
                              site_mm=c['site'].throw_site_mm(),
                              target_mm=landing, flight_s=c['T'])
    return sg.CatchTerminal(landing_mm=landing, landing_vel_mm_s=c['v_arrival'],
                            t_land_s=t_land,
                            rest_site_mm=c['site'].rest_site_mm(),
                            then_throw=then)


def _both(c, term, t_now=None, **extra):
    t_now = c['t_rel'] - sc.HANDOFF_LEAD_S if t_now is None else t_now
    kw = dict(lead_s=ex.LEAD_S, cfg=c['cfg'], limits=c['limits'],
              geom=c['geom'], warm_start=c['warm'], **extra)
    before = pickle.dumps(c['record'], protocol=pickle.HIGHEST_PROTOCOL)
    chk = ex.check_segment(c['record'], None, sg.CATCH, term, t_now, **kw)
    assert pickle.dumps(c['record'], protocol=pickle.HIGHEST_PROTOCOL) == before
    _rec, res, seg = ex.install_segment(c['record'], None, sg.CATCH, term,
                                        t_now, **kw)
    return chk, res, seg


def test_check_segment_through_a_record_is_the_installs_own_plan(chain):
    record = chain['record']
    chk, res, seg = _both(chain, _catch_terminal(chain))
    assert res.accepted, res.message
    assert res.seeded_post_release          # the handoff snap was taken
    assert chk.planned and chk.code == 'OK', chk.message
    assert 'post-release' in chk.message
    assert chk.margin == pytest.approx(
        adm.margin(seg.meta.report, chain['limits']), abs=1e-9)
    assert chk.margin >= 0.0
    assert chk.plan_wall_s > 0.0
    assert chain['record'] is record


def test_check_segment_reports_the_planners_refusal_code(chain):
    chk, res, _seg = _both(chain, _catch_terminal(chain, (400.0, 0.0)))
    assert not res.accepted
    assert not chk.planned and chk.code == res.code, (chk, res)


def test_check_segment_shares_the_installs_origin_refusals(chain):
    term = _catch_terminal(chain)
    # A dispatch so late that the first knot LEAD_S ahead is still on the
    # launch THROW's record (its SETTLE tail runs to t0 + 0.7 s) but within
    # the 4-knot floor of the touch-down: the SPLICE branch's WINDOW_TOO_SHORT
    # from the shared origin derivation.
    late = float(term.t_land_s) - ex.LEAD_S - 0.05
    assert late + ex.LEAD_S < chain['record'].end_s
    chk, res, _seg = _both(chain, term, t_now=late)
    assert res.code == ex.WINDOW_TOO_SHORT
    assert not chk.planned and chk.code == ex.WINDOW_TOO_SHORT
    assert chk.message == res.message
    assert math.isnan(chk.margin)


def test_check_segment_refuses_a_magazine_by_name_and_a_seedless_origin(chain):
    chk = ex.check_segment(None, None, mt.MAGAZINE, object(), 0.0,
                           limits=chain['limits'])
    assert not chk.planned and chk.code == ex.MAGAZINE_NOT_CHECKABLE
    # No record and no seed: the install's own ValueError -> UNREACHABLE.
    term = _catch_terminal(chain)
    chk = ex.check_segment(None, None, sg.CATCH, term, chain['t_rel'],
                           limits=chain['limits'])
    _r, res, _s = ex.install_segment(None, None, sg.CATCH, term, chain['t_rel'],
                                     limits=chain['limits'])
    assert chk.code == res.code == ex.UNREACHABLE
    assert chk.message == res.message
