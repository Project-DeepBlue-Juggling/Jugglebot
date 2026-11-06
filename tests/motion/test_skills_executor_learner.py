"""P1 (learner-generalisation, 2026-11-06): the executor driven by a REAL
``learner.Learner`` + ``plant_model.PlantModel`` (rows fed through
``observe``), the context API (``takes_context``).

What is pinned here (unit L3 of the P1 design):

* the ONE context builder (``SkillExecutor._throw_context``) for a same-site
  Columns catch-throw, a carried (oval) fold and a launch THROW: every field,
  ``release_kind``, and the release velocity of the MODEL command (not of
  ``y_d``);
* the candidate is the Learner's own capped command -- the executor's fixed
  lateral clamp about ``y_d`` is NOT applied on top;
* the MODEL command reaches ``admissible.resolve_command``: a candidate the
  REAL planner refuses at the old working point (350 / 5000 / 200000 / 4500)
  resolves toward the model command's apex, not toward ``y_d``'s;
* the hop entry commands the model command and writes no row;
* the v2 ``Experience`` written at finalise carries the context's fields.

The old ``command(x, y_d)`` duck type is pinned by the unchanged fakes in
``test_skills_executor.py`` / ``test_skills_executor_online.py``.
"""

from __future__ import annotations

import importlib.util
import math
import os

import numpy as np
import pytest

from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.skills import admissible as adm
from jugglebot.motion.skills import executor as ex
from jugglebot.motion.skills import learner as lr
from jugglebot.motion.skills import memory as mem
from jugglebot.motion.skills import plant_model as pm
from jugglebot.motion.skills import schedule as sc
from jugglebot.motion.skills import segments as sg
from jugglebot.motion.skills import sites as st
from jugglebot.motion.skills.schedule import Schedule, Skill
from jugglebot.motion.trajectory import ballistics_bc as bal

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
_PROBE = os.path.join(_ROOT, 'tools', 'probes', 'admissible_obviation',
                      'online_check_replay.py')
_OLD_LIMITS = (350.0, 5000.0, 200000.0, 4500.0)
_APEX = 0.95
_SITE_MM = (62.5, 0.0)
_DWELL = 0.27
_K = 1.05                      # the plant gain the trained models learn
_FROZEN = pm.PlantModelConfig(prior_sd_b_m=1e-6, prior_sd_s_per_m=1e-6)


@pytest.fixture(scope='module')
def cell():
    spec = importlib.util.spec_from_file_location('online_check_replay', _PROBE)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    c = mod.Cell(_SITE_MM, _APEX, mod.asw.sitting_limits(*_OLD_LIMITS),
                 StewartGeometry(), sg.SegmentConfig(planner_mode='pose'))
    assert c.ok, c.code
    return c


class _Installer:
    def __init__(self):
        self.calls = []

    def __call__(self, kind, terminal, t_now_s, ball_id=None):
        self.calls.append((kind, terminal, t_now_s, ball_id))
        return ex.InstallResult(True, 'OK', 'ok', 0.0, splice_k=0)


class _OkChecker:
    def __init__(self):
        self.calls = []

    def __call__(self, kind, terminal, t_abs_s, *, ball_id):
        self.calls.append((kind, terminal, t_abs_s, ball_id))
        return adm.CommandVerdict(True, 0.5, 'OK', True)


class _RealChecker:
    """``check_segment`` of the stand-in from the cell's seed at t = 0 (the
    PIN recipe of ``test_skills_executor_online.py``)."""

    def __init__(self, cell):
        self.cell = cell
        self.calls = []

    def __call__(self, kind, terminal, t_abs_s, *, ball_id):
        c = self.cell
        chk = ex.check_segment(None, c.seed, kind, terminal, 0.0, cfg=c.cfg,
                               limits=c.limits, geom=c.geom)
        if not chk.planned:
            v = adm.CommandVerdict(False, chk.margin, chk.code, True)
        else:
            ok = chk.margin >= 0.0
            v = adm.CommandVerdict(ok, chk.margin, 'OK' if ok else 'MARGIN',
                                   True)
        self.calls.append(v)
        return v


def _exp(ctx, u, y, t=1e9, ball_id=None, kind=None):
    return mem.Experience(
        site_xy_m=ctx.site_xy_m, release_m=ctx.release_m,
        release_vel_mps=ctx.release_vel_mps, dwell_s=ctx.dwell_s,
        seat_offset_m=ctx.seat_offset_m, u=u, y=y, t_abs_s=t,
        ball_id=ctx.ball_id if ball_id is None else ball_id,
        release_kind=ctx.release_kind if kind is None else kind, caught=True)


def _gain_trained(model_cfg=None, n=40, k=_K):
    """A Learner whose plant model has learned apex gain ~``_K`` from rows of
    a far-away CARRIED context (they never enter a free-cup residual pool;
    the plant model is global)."""
    learner = lr.Learner(model=pm.PlantModel(model_cfg),
                         config=lr.LearnerConfig())
    far = lr.ThrowContext(site_xy_m=(0.3, 0.3), release_m=(0.3, 0.3, 0.86),
                          release_vel_mps=(1.0, 1.0, 1.0), dwell_s=0.5,
                          ball_id=0, release_kind='carried')
    for i in range(n):
        a = 0.85 + 0.005 * (i % 10)
        learner.observe(_exp(far, [0.0, 0.0, a], [0.0, 0.0, k * a], t=i))
    assert learner.model.theta()[0] == pytest.approx(k, abs=0.01)
    return learner


def _site():
    return st.Site('S', np.array([_SITE_MM[0], _SITE_MM[1], st.CATCH_CUP_Z_MM]))


def _catch_schedule(cell, release_site=None, pattern='columns'):
    s = cell.site
    tau = sc.transit_s(cell.T, _DWELL)
    tt = sc.ThenThrow(t_release_abs_s=tau + _DWELL,
                      y_d=(np.zeros(2), _APEX), target=s,
                      release_site=release_site)
    sk = Skill(kind=sg.CATCH, ball_id=1, site=s, t_abs_s=tau, window_s=0.25,
               then_throw=tt)
    return Schedule(pattern=pattern, skills=(sk,), flight_s=cell.T,
                    beat_s=cell.T, transit_s=tau, dwell_s=_DWELL, t0_abs_s=0.0)


def _landing(cell, dx_mm=0.0):
    tau = sc.transit_s(cell.T, _DWELL)
    return ex.Landing(
        pos_mm=cell.site.catch_site_mm() + np.array([dx_mm, 0.0, 0.0]),
        vel_mm_s=bal.arrival_velocity(cell.takeoff, cell.T),
        t_land_abs_s=tau, from_fit=True)


def _expected_vel_mps(release_mm, target_site, rel_site, u_model, apex_d):
    """The model command's planned release velocity, computed here from the
    closed form (commanded target + the release offset's fly-back)."""
    um_xy, um_apex = u_model
    r = np.asarray(release_mm)[:2] - np.asarray(rel_site.cup_mm)[:2]
    g = min(max(apex_d / um_apex, 0.5), 2.0)
    aim = (target_site.catch_site_mm()
           + np.array([um_xy[0] * 1e3, um_xy[1] * 1e3, 0.0])
           + np.array([r[0] * (1 - 1 / g), r[1] * (1 - 1 / g), 0.0]))
    return bal.launch_velocity(np.asarray(release_mm, dtype=float), aim,
                               sc.flight_s(um_apex)) / 1000.0


# ---------------------------------------------------------------------------
# the context
# ---------------------------------------------------------------------------

def test_the_context_of_a_same_site_columns_catch_throw(cell):
    learner = _gain_trained()
    sch = _catch_schedule(cell)
    x = ex.SkillExecutor(sch, _Installer(), tracker=lambda b: _landing(cell, 5.0),
                         learner=learner, checker=_OkChecker())
    x.tick(sch.skills[0].dispatch_s())
    assert not x.attempt_ended, x.end_message
    s = cell.site
    ctx = x._ctx_cache[0]
    release_mm = np.array([s.cup_mm[0] + 5.0, s.cup_mm[1], st.RELEASE_CUP_Z_MM])
    np.testing.assert_allclose(ctx.site_xy_m, np.asarray(s.cup_mm[:2]) / 1e3)
    np.testing.assert_allclose(ctx.release_m, release_mm / 1e3, atol=1e-12)
    np.testing.assert_allclose(ctx.seat_offset_m, [0.005, 0.0], atol=1e-12)
    assert ctx.release_kind == 'free_cup' and ctx.ball_id == 1
    assert ctx.dwell_s == pytest.approx(_DWELL)
    um = learner.model_command(ctx, [0.0, 0.0, _APEX])
    assert um[2] == pytest.approx(_APEX / _K, abs=0.01)     # f0^-1, not y_d
    want = _expected_vel_mps(release_mm, s, s, (um[:2], um[2]), _APEX)
    np.testing.assert_allclose(ctx.release_vel_mps, want, atol=1e-9)
    ident = _expected_vel_mps(release_mm, s, s, (np.zeros(2), _APEX), _APEX)
    assert abs(ctx.release_vel_mps[2] - ident[2]) > 0.05    # not y_d's
    # the row's question refreshes release + seat offset (a re-send's move)
    x._carried_release_mm[0] = release_mm + np.array([3.0, 0.0, 0.0])
    late = x._latest_ctx(0, ctx)
    np.testing.assert_allclose(late.release_m, (release_mm + [3, 0, 0]) / 1e3)
    np.testing.assert_allclose(late.seat_offset_m, [0.008, 0.0], atol=1e-12)
    np.testing.assert_allclose(late.release_vel_mps, ctx.release_vel_mps)


def test_the_context_of_a_carried_oval_fold(cell):
    learner = _gain_trained()
    q = st.Site('Q', np.array([-_SITE_MM[0], 0.0, st.CATCH_CUP_Z_MM]))
    sch = _catch_schedule(cell, release_site=q, pattern='oval')
    x = ex.SkillExecutor(sch, _Installer(), tracker=lambda b: _landing(cell, 5.0),
                         learner=learner, checker=_OkChecker())
    x.tick(sch.skills[0].dispatch_s())
    assert not x.attempt_ended, x.end_message
    ctx = x._ctx_cache[0]
    s = cell.site
    np.testing.assert_allclose(ctx.site_xy_m, np.asarray(q.cup_mm[:2]) / 1e3)
    np.testing.assert_allclose(ctx.release_m, q.throw_site_mm() / 1e3)
    np.testing.assert_allclose(ctx.seat_offset_m, [0.005, 0.0], atol=1e-12)
    assert ctx.release_kind == 'carried'
    um = learner.model_command(ctx, [0.0, 0.0, _APEX])
    want = _expected_vel_mps(q.throw_site_mm(), s, q, (um[:2], um[2]), _APEX)
    np.testing.assert_allclose(ctx.release_vel_mps, want, atol=1e-9)
    assert ctx.release_vel_mps[0] > 0.1          # flies Q -> S


def test_the_context_of_a_launch_throw():
    learner = _gain_trained()
    p1, _p2 = st.columns_sites(100.0)
    sk = Skill(kind=sg.THROW, ball_id=0, site=p1, t_abs_s=10.4, window_s=0.4,
               y_d=(np.zeros(2), _APEX), target=p1)
    sch = Schedule(pattern='self_toss', skills=(sk,),
                   flight_s=sc.flight_s(_APEX), beat_s=1.0, transit_s=0.6,
                   dwell_s=_DWELL, t0_abs_s=10.0)
    inst = _Installer()
    x = ex.SkillExecutor(sch, inst, learner=learner)
    x.tick(sk.dispatch_s())
    ctx = x._ctx_cache[0]
    np.testing.assert_allclose(ctx.site_xy_m, np.asarray(p1.cup_mm[:2]) / 1e3)
    np.testing.assert_allclose(ctx.release_m, p1.throw_site_mm() / 1e3)
    np.testing.assert_allclose(ctx.seat_offset_m, [0.0, 0.0])
    assert ctx.release_kind == 'free_cup' and ctx.ball_id == 0
    assert ctx.dwell_s == pytest.approx(_DWELL)
    um = learner.model_command(ctx, [0.0, 0.0, _APEX])
    want = _expected_vel_mps(p1.throw_site_mm(), p1, p1, (um[:2], um[2]), _APEX)
    np.testing.assert_allclose(ctx.release_vel_mps, want, atol=1e-9)
    # empty residual support: the command IS the model command
    term = inst.calls[0][1]
    assert term.flight_s == pytest.approx(sc.flight_s(um[2]), abs=1e-12)


# ---------------------------------------------------------------------------
# the candidate and the model command
# ---------------------------------------------------------------------------

def _residual_learner(ctx_of, dx_m, model_cfg=_FROZEN, n=16, k=_K):
    """A gain-trained Learner (offsets frozen at the identity prior) with
    ``n`` rows at the query context whose landings sit ``dx_m`` off f0: a
    residual only the memory sees, so the L3 cap is what binds."""
    learner = _gain_trained(model_cfg, k=k)
    ctx = ctx_of(learner)
    um = learner.model_command(ctx, [0.0, 0.0, _APEX])
    for i in range(n):
        u = np.array([um[0], um[1], um[2]])
        learner.observe(_exp(ctx, u, [u[0] + dx_m, u[1], k * u[2]],
                             t=100.0 + i))
    return learner


def test_the_candidate_is_the_learners_capped_command_with_no_fixed_clamp():
    p1, _p2 = st.columns_sites(100.0)
    sk = Skill(kind=sg.THROW, ball_id=0, site=p1, t_abs_s=10.4, window_s=0.4,
               y_d=(np.zeros(2), _APEX), target=p1)
    sch = Schedule(pattern='self_toss', skills=(sk,),
                   flight_s=sc.flight_s(_APEX), beat_s=1.0, transit_s=0.6,
                   dwell_s=_DWELL, t0_abs_s=10.0)

    def ctx_of(learner):
        probe = ex.SkillExecutor(sch, _Installer(), learner=learner)
        return probe._throw_context(sk, p1, p1.throw_site_mm(), p1, sk.y_d,
                                    (0.0, 0.0))[0]

    learner = _residual_learner(ctx_of, +0.012)
    inst = _Installer()
    x = ex.SkillExecutor(sch, inst, learner=learner, lateral_authority_m=0.001)
    x.tick(sk.dispatch_s())
    _x, u_dy, u_apex = x._u_cache[0]
    want = learner.command(x._ctx_cache[0], [0.0, 0.0, _APEX])
    np.testing.assert_allclose(u_dy, want[:2], atol=1e-12)
    assert u_apex == pytest.approx(want[2], abs=1e-12)
    assert u_dy[0] < -0.005                  # beyond the 1 mm fixed clamp
    assert x._learned_u == {0}


def test_the_model_command_reaches_resolve_command_on_the_real_planner(
        cell, monkeypatch):
    """Real planner at the old point, a real Learner with a residual only
    the memory sees: ``admissible.resolve_command`` is handed the Learner's
    capped candidate AND its MODEL command ``f0^-1(y_d)`` (apex ``y_d`` / K,
    K = 0.95), never ``y_d``.

    Why not "a refused candidate steps toward the model": with a consistent
    Learner the stand-in is planned at the PREDICTED landing, ``u + alpha
    r_hat``, so the candidate's stand-in landing (``alpha r_hat`` minus the
    capped correction) always lies between the target and the model
    command's (``alpha r_hat``), on the same side. A candidate the planner
    refuses on its landing therefore implies a model command refused
    further out (probed 2026-11-06: residuals -35..-70 mm, caps 30/60 mm,
    lateral authority 0-10 mm, every candidate passed, margins 0.035-0.076).
    The fallback ROUTING is pinned here by a spy; the step toward the model
    command on a real refusal is pinned on the envelope below."""
    sch = _catch_schedule(cell)

    def ctx_of(learner):
        probe = ex.SkillExecutor(sch, _Installer(), learner=learner)
        rel = cell.site.throw_site_mm()
        return probe._throw_context(sch.skills[0], cell.site, rel, cell.site,
                                    sch.skills[0].then_throw.y_d,
                                    (0.0, 0.0))[0]

    learner = _residual_learner(ctx_of, -0.040, k=0.95)
    seen = []
    real = ex.adm.resolve_command

    def spy(verdict_fn, u_cand, u_model, *a, **kw):
        seen.append((u_cand, u_model))
        return real(verdict_fn, u_cand, u_model, *a, **kw)

    monkeypatch.setattr(ex.adm, 'resolve_command', spy)
    chk = _RealChecker(cell)
    x = ex.SkillExecutor(sch, _Installer(), tracker=lambda b: _landing(cell),
                         learner=learner, checker=chk)
    x.tick(sch.skills[0].dispatch_s())
    assert not x.attempt_ended, x.end_message
    ctx = x._ctx_cache[0]
    cand = learner.command(ctx, [0.0, 0.0, _APEX])
    um = learner.model_command(ctx, [0.0, 0.0, _APEX])
    assert cand[0] == pytest.approx(um[0] + 0.030, abs=1e-4)   # at the cap
    (u_cand, u_model), = seen
    np.testing.assert_allclose(u_cand[0], cand[:2], atol=1e-12)
    np.testing.assert_allclose(u_model[0], um[:2], atol=1e-12)
    assert u_model[1] == pytest.approx(um[2], abs=1e-12)
    assert abs(u_model[1] - _APEX) > 0.02          # NOT y_d's apex
    assert chk.calls and chk.calls[0].evaluated


def test_an_envelope_refused_launch_candidate_falls_back_to_the_model_command(
        monkeypatch):
    """A plain THROW (envelope-only resolution): the envelope refuses the
    candidate's flight, so the MODEL command (apex ``y_d`` / K) is what is
    installed -- not ``y_d``'s apex."""
    p1, _p2 = st.columns_sites(100.0)
    sk = Skill(kind=sg.THROW, ball_id=0, site=p1, t_abs_s=10.4, window_s=0.4,
               y_d=(np.zeros(2), _APEX), target=p1)
    sch = Schedule(pattern='self_toss', skills=(sk,),
                   flight_s=sc.flight_s(_APEX), beat_s=1.0, transit_s=0.6,
                   dwell_s=_DWELL, t0_abs_s=10.0)

    def ctx_of(learner):
        probe = ex.SkillExecutor(sch, _Installer(), learner=learner)
        return probe._throw_context(sk, p1, p1.throw_site_mm(), p1, sk.y_d,
                                    (0.0, 0.0))[0]

    learner = _residual_learner(ctx_of, +0.012)
    ctx0 = ctx_of(learner)
    cand = learner.command(ctx0, [0.0, 0.0, _APEX])
    um = learner.model_command(ctx0, [0.0, 0.0, _APEX])
    assert abs(cand[0] - um[0]) > 0.005          # a real correction
    real = ex.throw_envelope.evaluate
    calls = []

    def env(flight_s, speed_mps):     # refuses the FIRST Throw it is shown
        calls.append((flight_s, speed_mps))
        if len(calls) == 1:
            return ex.throw_envelope.ThrowEnvelopeVerdict(False, 'TEST', 'x')
        return real(flight_s, speed_mps)

    monkeypatch.setattr(ex.throw_envelope, 'evaluate', env)
    inst = _Installer()
    x = ex.SkillExecutor(sch, inst, learner=learner, checker=_OkChecker())
    lines = x.tick(sk.dispatch_s())
    assert not x.attempt_ended, x.end_message
    _x, u_dy, u_apex = x._u_cache[0]
    np.testing.assert_allclose(u_dy, um[:2], atol=1e-12)
    assert u_apex == pytest.approx(um[2], abs=1e-12)
    assert abs(u_apex - _APEX) > 0.02
    assert inst.calls[0][1].flight_s == pytest.approx(sc.flight_s(um[2]))
    assert len(calls) == 2                      # the candidate, the model
    assert any('refused ENVELOPE:TEST' in ln and 'model command' in ln
               for ln in lines)


def test_the_hop_entry_commands_the_model_command_and_writes_no_row():
    learner = _gain_trained()
    p1, p2 = st.columns_sites(100.0)
    t_rel = 10.4
    sk = Skill(kind=sg.THROW, ball_id=0, site=p1, t_abs_s=t_rel, window_s=0.4,
               y_d=(np.zeros(2), _APEX), target=p2, entry_hop=True)
    sch = Schedule(pattern='columns', skills=(sk,),
                   flight_s=sc.flight_s(_APEX), beat_s=1.0, transit_s=0.6,
                   dwell_s=_DWELL, t0_abs_s=10.0)
    inst = _Installer()
    rows = []
    x = ex.SkillExecutor(sch, inst, learner=learner, checker=_OkChecker(),
                         lateral_authority_m=0.030,
                         on_experience=rows.append)
    lines = x.tick(sk.dispatch_s())
    ctx = x._ctx_cache[0]
    um = learner.model_command(ctx, [0.0, 0.0, _APEX])
    _x, u_dy, u_apex = x._u_cache[0]
    assert u_apex == pytest.approx(um[2], abs=1e-12)
    np.testing.assert_allclose(u_dy, np.clip(um[:2], -0.03, 0.03), atol=1e-12)
    assert inst.calls[0][1].flight_s == pytest.approx(sc.flight_s(um[2]))
    assert any('HOP-COMMAND skill 0' in ln for ln in lines)
    pend, = x._pending_outcomes
    assert pend.entry_hop
    # finalise it: a fitted landing at the target -> the report, no row
    land = ex.Landing(pos_mm=p2.catch_site_mm(),
                      vel_mm_s=np.array([0.0, 0.0, -4200.0]),
                      t_land_abs_s=t_rel + sc.flight_s(u_apex), from_fit=True)
    x.tracker = lambda b: land
    x.observer = lambda b, t: ex.CAUGHT_EVIDENCE
    t = sk.dispatch_s()
    while t < t_rel + 4.0:
        x.tick(t)
        t += 0.01
    assert not x._pending_outcomes
    assert rows == []


def test_the_v2_row_carries_the_context():
    learner = _gain_trained()
    p1, _p2 = st.columns_sites(100.0)
    t_rel = 10.4
    sk = Skill(kind=sg.THROW, ball_id=0, site=p1, t_abs_s=t_rel, window_s=0.4,
               y_d=(np.zeros(2), _APEX), target=p1)
    sch = Schedule(pattern='self_toss', skills=(sk,),
                   flight_s=sc.flight_s(_APEX), beat_s=1.0, transit_s=0.6,
                   dwell_s=_DWELL, t0_abs_s=10.0)
    rows = []
    x = ex.SkillExecutor(sch, _Installer(), learner=learner,
                         observer=lambda b, t: ex.CAUGHT_EVIDENCE,
                         on_experience=rows.append)
    x.tick(sk.dispatch_s())
    _x, u_dy, u_apex = x._u_cache[0]
    land = ex.Landing(pos_mm=p1.catch_site_mm() + np.array([4.0, -2.0, 0.0]),
                      vel_mm_s=np.array([0.0, 0.0, -4200.0]),
                      t_land_abs_s=t_rel + sc.flight_s(u_apex), from_fit=True)
    x.tracker = lambda b: land
    t = sk.dispatch_s()
    while t < t_rel + 4.0 and not rows:
        x.tick(t)
        t += 0.01
    row, = rows
    ctx = x._latest_ctx(0, x._ctx_cache[0])
    np.testing.assert_allclose(row.site_xy_m, ctx.site_xy_m)
    np.testing.assert_allclose(row.release_m, x._release_pos_mm(0) / 1e3)
    np.testing.assert_allclose(row.release_vel_mps, ctx.release_vel_mps)
    np.testing.assert_allclose(row.seat_offset_m, ctx.seat_offset_m)
    assert row.dwell_s == pytest.approx(_DWELL)
    assert row.release_kind == 'free_cup' and row.ball_id == 0
    assert not row.migrated and row.caught
    np.testing.assert_allclose(row.u, [u_dy[0], u_dy[1], u_apex])
    np.testing.assert_allclose(row.y[:2], [0.004, -0.002], atol=1e-12)
    # the row round-trips through the memory and the Learner learns from it
    n0 = learner.n_rows
    assert learner.observe(row) in (True, False)
    assert learner.n_rows == n0 + 1
    assert math.isfinite(row.t_abs_s)
