"""P3a: the executor's ONLINE admissibility path (``SkillExecutor(checker=)``,
``plans/active/learner-generalisation.md`` P3a, `motion/skills/INVARIANTS.md` I-ADMIT-1).

Recipe table (driver, working point, expected code):

=====================================  ===========================  ==========================================
driver                                 working point                expected
=====================================  ===========================  ==========================================
REAL planner: ``check_segment`` of     OLD 350 / 5000 / 200000 /     candidate (0, +10) mm passes (1 check);
the stand-in, fresh origin at t=0      4500, pose, Apex 0.95,       candidate (+30, 0) mm refused (LIMIT_VEL),
from the probe ``Cell``'s seed (the    Dwell 0.27, site (62.5, 0)   resolved off it (2-3 checks); y_d
partner launch THROW's release),       (``online_check_replay``)    (+30, 0) mm with no Learner refused ->
mapped to a verdict as that file's PIN does                              NO_ADMISSIBLE_COMMAND, nothing installed
scripted checker (records calls,       n/a                          plumbing: call counts, the stand-in's
answers a fixed verdict)                                            geometry, unevaluated -> refused
spy on ``throw_envelope.evaluate``     n/a (closed form)            called on the carried Throw, the launch
(optionally refusing one flight)                                    THROW and the hop; refusal -> step /
                                                                    model / NO_ADMISSIBLE_COMMAND
=====================================  ===========================  ==========================================

The executor's own installer is a recording fake throughout: what is under
test is which COMMAND reaches the installer, and whether one does at all.
The REAL-planner checker ignores the dispatch instant and plans the stand-in
from the probe cell's seed at ``t_now_s = 0`` (the schedule below is built on
the cell's own clock: catch at ``tau``, release at ``tau + DWELL``), exactly
the PIN against the probe in ``tests/motion/test_skills_admissible_online.py``.
"""

from __future__ import annotations

import dataclasses
import importlib.util
import math
import os

import numpy as np
import pytest

from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.skills import admissible as adm
from jugglebot.motion.skills import executor as ex
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


@pytest.fixture(scope='module')
def probe():
    spec = importlib.util.spec_from_file_location('online_check_replay', _PROBE)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope='module')
def cell(probe):
    limits = probe.asw._limits(*_OLD_LIMITS)
    c = probe.Cell(_SITE_MM, _APEX, limits, StewartGeometry(),
                   sg.SegmentConfig(planner_mode='pose'))
    assert c.ok, c.code
    return c


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------

class _Installer:
    def __init__(self):
        self.calls = []

    def __call__(self, kind, terminal, t_now_s, ball_id=None):
        self.calls.append((kind, terminal, t_now_s, ball_id))
        return ex.InstallResult(True, 'OK', 'ok', 0.0, splice_k=0)


class _RealChecker:
    """``check_segment`` of the stand-in from the cell's seed at t = 0."""

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
        self.calls.append((kind, terminal, t_abs_s, ball_id, v))
        return v


class _ScriptedChecker:
    def __init__(self, verdict=None, raises=None):
        self.verdict = verdict or adm.CommandVerdict(True, 0.5, 'OK', True)
        self.raises = raises
        self.calls = []

    def __call__(self, kind, terminal, t_abs_s, *, ball_id):
        self.calls.append((kind, terminal, t_abs_s, ball_id))
        if self.raises is not None:
            raise self.raises
        return self.verdict


class _Learner:
    """A fixed command; optionally a forward model ``y_hat = u + bias``."""

    def __init__(self, u, bias_xy=None, raise_value_error=False):
        self.u = np.asarray(u, dtype=float)
        self.bias = None if bias_xy is None else np.asarray(bias_xy, float)
        self.raise_value_error = raise_value_error
        self.calls = 0
        if bias_xy is None:
            self.local_model = None

    def command(self, x, y_d):
        self.calls += 1
        if self.raise_value_error:
            raise ValueError('neighbourhood weights underflowed to 0')
        return self.u.copy()

    def local_model(self, x, y_d):   # replaced by None when no bias
        return (self.u.copy(), np.eye(3),
                self.u + np.array([self.bias[0], self.bias[1], 0.0]))


def _site():
    return st.Site('S', np.array([_SITE_MM[0], _SITE_MM[1], st.CATCH_CUP_Z_MM]))


def _catch_schedule(cell, y_d_xy=(0.0, 0.0), shadow=False, release_site=None,
                    extra=()):
    """ONE catch-with-throw at S on the probe cell's clock (catch at ``tau``,
    release ``tau + DWELL``), aimed by the tracker (no previous release)."""
    s = cell.site
    tau = sc.transit_s(cell.T, cell_dwell())
    tt = sc.ThenThrow(t_release_abs_s=tau + cell_dwell(),
                      y_d=(np.asarray(y_d_xy, dtype=float), _APEX), target=s,
                      shadow_landing=shadow, release_site=release_site)
    sk = Skill(kind=sg.CATCH, ball_id=1, site=s, t_abs_s=tau, window_s=0.25,
               then_throw=tt)
    return Schedule(pattern='columns', skills=(sk,) + tuple(extra),
                    flight_s=cell.T, beat_s=cell.T, transit_s=tau,
                    dwell_s=cell_dwell(), t0_abs_s=0.0)


def cell_dwell():
    return 0.27


def _landing(cell, dx_mm=0.0):
    tau = sc.transit_s(cell.T, cell_dwell())
    return ex.Landing(
        pos_mm=cell.site.catch_site_mm() + np.array([dx_mm, 0.0, 0.0]),
        vel_mm_s=bal.arrival_velocity(cell.takeoff, cell.T),
        t_land_abs_s=tau, from_fit=True)


def _run_catch(cell, checker, learner=None, y_d_xy=(0.0, 0.0), **kw):
    sch = _catch_schedule(cell, y_d_xy=y_d_xy,
                          shadow=kw.pop('shadow', False))
    inst = _Installer()
    state = {'landing': _landing(cell)}
    x = ex.SkillExecutor(sch, inst, tracker=lambda b: state['landing'],
                         learner=learner, checker=checker, **kw)
    lines = x.tick(sch.skills[0].dispatch_s())
    return sch, inst, x, lines, state


def _then_throw(inst):
    kinds = [c[0] for c in inst.calls]
    assert kinds == [sg.CATCH], kinds
    return inst.calls[0][1].then_throw


def _commanded_xy_mm(cell, then):
    """The installed carried throw's commanded landing offset (mm) about the
    site's catch point (zero fly-back: the tracker landing sits on the site,
    so the release offset is zero)."""
    return (np.asarray(then.target_mm)[:2]
            - cell.site.catch_site_mm()[:2])


# ---------------------------------------------------------------------------
# (1) REAL planner at the OLD working point
# ---------------------------------------------------------------------------

def test_a_passing_candidate_is_installed_unchanged_after_one_check(cell):
    chk = _RealChecker(cell)
    _s, inst, x, lines, _st = _run_catch(
        cell, chk, learner=_Learner([0.0, 0.010, _APEX]))
    assert not x.attempt_ended, x.end_message
    assert len(chk.calls) == 1 and chk.calls[0][4].ok
    then = _then_throw(inst)
    np.testing.assert_allclose(_commanded_xy_mm(cell, then), [0.0, 10.0],
                               atol=1e-9)
    assert then.flight_s == pytest.approx(sc.flight_s(_APEX), abs=1e-12)
    assert any('ADMISSIBILITY skill 0' in ln and 'passed m=' in ln
               for ln in lines)


def test_an_out_of_margin_learner_command_is_never_installed_as_commanded(cell):
    """(+30, 0) mm at the old point: the stand-in is refused LIMIT_VEL (the
    leg velocity headroom at 350 mm/s), the model command passes, and the
    command installed is the stepped point (or the model) -- never +30 mm."""
    chk = _RealChecker(cell)
    _s, inst, x, lines, _st = _run_catch(
        cell, chk, learner=_Learner([0.030, 0.0, _APEX]))
    assert not x.attempt_ended, x.end_message
    verdicts = [c[4] for c in chk.calls]
    assert verdicts[0].code == 'LIMIT_VEL' and not verdicts[0].ok
    assert verdicts[1].ok                       # the model command
    assert len(verdicts) in (2, 3)
    got = _commanded_xy_mm(cell, _then_throw(inst))
    assert got[0] < 30.0 - 1.0 and abs(got[1]) < 1e-9, got
    if len(verdicts) == 3 and verdicts[2].ok:
        assert got[0] > 0.0                     # stepped toward the candidate
    _x, u_dy, _a = x._u_cache[0]
    np.testing.assert_allclose(u_dy * 1000.0, got, atol=1e-9)
    assert any('refused LIMIT_VEL' in ln and '->' in ln for ln in lines)


def test_a_refused_model_command_ends_the_attempt_before_any_install(cell):
    """No Learner, the schedule's own y_d (+30, 0) mm: candidate == model,
    refused on the real planner -> NO_ADMISSIBLE_COMMAND after ONE check."""
    chk = _RealChecker(cell)
    _s, inst, x, lines, _st = _run_catch(cell, chk, y_d_xy=(0.030, 0.0))
    assert x.attempt_ended and x.end_code == ex.NO_ADMISSIBLE_COMMAND
    assert inst.calls == []
    assert len(chk.calls) == 1
    assert 'LIMIT_VEL' in x.end_message


# ---------------------------------------------------------------------------
# (2) scripted checker: plumbing
# ---------------------------------------------------------------------------

@pytest.mark.parametrize('checker', [
    _ScriptedChecker(adm.CommandVerdict(False, float('nan'),
                                        'SOLVER_UNAVAILABLE', False)),
    _ScriptedChecker(raises=RuntimeError('service timed out')),
], ids=['unevaluated', 'raises'])
def test_an_unevaluated_check_ends_the_attempt_before_any_install(cell,
                                                                  checker):
    checker.calls = []
    _s, inst, x, _l, _st = _run_catch(cell, checker,
                                      learner=_Learner([0.01, 0.0, _APEX]))
    assert x.attempt_ended and x.end_code == ex.NO_ADMISSIBLE_COMMAND
    assert inst.calls == []
    assert 'check unavailable' in x.end_message
    assert len(checker.calls) == 1


def test_the_command_is_checked_once_and_a_resend_reuses_it(cell):
    chk = _ScriptedChecker()
    sch, inst, x, _l, state = _run_catch(cell, chk,
                                         learner=_Learner([0.004, 0.0, _APEX]))
    assert len(chk.calls) == 1 and len(inst.calls) == 1
    first = _commanded_xy_mm(cell, inst.calls[0][1].then_throw)
    state['landing'] = _landing(cell, dx_mm=12.0)
    x.tick(sch.skills[0].dispatch_s() + 0.05)
    assert len(inst.calls) == 2, 'the re-send did not fire'
    assert len(chk.calls) == 1                  # never re-checked
    _x, u_dy, _a = x._u_cache[0]
    np.testing.assert_allclose(u_dy * 1000.0, first, atol=1e-9)


def test_the_lateral_authority_clamps_the_candidate_before_the_check(cell):
    chk = _ScriptedChecker()
    _s, inst, _x, lines, _st = _run_catch(
        cell, chk, learner=_Learner([0.080, -0.080, _APEX]),
        lateral_authority_m=0.020)
    standin = chk.calls[0][1]
    np.testing.assert_allclose(
        standin.landing_mm[:2] - cell.site.catch_site_mm()[:2],
        [20.0, -20.0], atol=1e-9)
    np.testing.assert_allclose(_commanded_xy_mm(cell, _then_throw(inst)),
                               [20.0, -20.0], atol=1e-9)
    assert any('u=(+20.0, -20.0) mm' in ln for ln in lines)


def test_the_standin_is_planned_at_the_predicted_landing(cell):
    """A biased forward model (y_hat = u + (-15, +5) mm) moves the STAND-IN
    landing (and its same-site release, which rides it), not the command."""
    chk = _ScriptedChecker()
    _s, inst, _x, _l, _st = _run_catch(
        cell, chk, learner=_Learner([0.010, 0.0, _APEX],
                                    bias_xy=(-0.015, 0.005)))
    standin = chk.calls[0][1]
    want = cell.site.catch_site_mm()[:2] + np.array([-5.0, 5.0])
    np.testing.assert_allclose(standin.landing_mm[:2], want, atol=1e-9)
    np.testing.assert_allclose(standin.then_throw.site_mm[:2], want,
                               atol=1e-9)
    real = _landing(cell)
    assert standin.t_land_s == real.t_land_abs_s
    np.testing.assert_allclose(standin.landing_vel_mm_s, real.vel_mm_s)
    np.testing.assert_allclose(_commanded_xy_mm(cell, _then_throw(inst)),
                               [10.0, 0.0], atol=1e-9)


def test_a_carried_fold_standin_releases_at_the_release_site(cell):
    q = st.Site('Q', np.array([-_SITE_MM[0], 0.0, st.CATCH_CUP_Z_MM]))
    s = cell.site
    tau = sc.transit_s(cell.T, cell_dwell())
    tt = sc.ThenThrow(t_release_abs_s=tau + cell_dwell(),
                      y_d=(np.zeros(2), _APEX), target=s, release_site=q)
    sk = Skill(kind=sg.CATCH, ball_id=1, site=s, t_abs_s=tau, window_s=0.25,
               then_throw=tt)
    sch = Schedule(pattern='oval', skills=(sk,), flight_s=cell.T,
                   beat_s=cell.T, transit_s=tau, dwell_s=cell_dwell(),
                   t0_abs_s=0.0)
    chk = _ScriptedChecker()
    x = ex.SkillExecutor(sch, _Installer(), tracker=lambda b: _landing(cell),
                         learner=_Learner([0.010, 0.0, _APEX]), checker=chk)
    x.tick(sk.dispatch_s())
    standin = chk.calls[0][1]
    np.testing.assert_allclose(standin.then_throw.site_mm, q.throw_site_mm())
    np.testing.assert_allclose(standin.landing_mm[:2] - s.catch_site_mm()[:2],
                               [10.0, 0.0], atol=1e-9)


def test_a_learner_value_error_still_refuses_on_the_online_path(cell):
    chk = _ScriptedChecker()
    _s, inst, x, _l, _st = _run_catch(
        cell, chk, learner=_Learner([0.0, 0.0, _APEX], raise_value_error=True))
    assert x.end_code == ex.NO_ADMISSIBLE_COMMAND and inst.calls == []
    assert chk.calls == [] and 'finite' in x.end_message


def test_a_shadow_landing_carried_throw_bypasses_the_checker(cell):
    chk = _ScriptedChecker()
    learner = _Learner([0.030, 0.0, 1.2])
    _s, inst, _x, _l, _st = _run_catch(cell, chk, learner=learner,
                                       shadow=True)
    assert chk.calls == [] and learner.calls == 0
    then = _then_throw(inst)
    np.testing.assert_allclose(_commanded_xy_mm(cell, then), [0.0, 0.0])
    assert then.flight_s == pytest.approx(sc.flight_s(_APEX))


# ---------------------------------------------------------------------------
# (3) T-U8: the envelope (spy on throw_envelope.evaluate)
# ---------------------------------------------------------------------------

class _EnvSpy:
    """Wraps ``throw_envelope.evaluate``; refuses the flights in ``refuse``
    (``'all'`` = every flight) with bound ``TEST``."""

    def __init__(self, real, refuse=()):
        self.real = real
        self.refuse = refuse
        self.calls = []

    def __call__(self, flight_s, speed_mps):
        self.calls.append((float(flight_s), float(speed_mps)))
        if self.refuse == 'all' or any(abs(flight_s - f) < 1e-9
                                       for f in self.refuse):
            return ex.throw_envelope.ThrowEnvelopeVerdict(False, 'TEST', 'x')
        return self.real(flight_s, speed_mps)


@pytest.fixture
def env_spy(monkeypatch):
    def make(refuse=()):
        spy = _EnvSpy(ex.throw_envelope.evaluate, refuse)
        monkeypatch.setattr(ex.throw_envelope, 'evaluate', spy)
        return spy
    return make


_P1, _P2 = st.columns_sites(100.0)


def _throw_run(checker, learner=None, entry_hop=False, shadow=False,
               boxes=None):
    sk = Skill(kind=sg.THROW, ball_id=0, site=_P1, t_abs_s=10.4,
               window_s=0.4, y_d=(np.zeros(2), _APEX),
               target=_P2 if (entry_hop or shadow) else _P1,
               entry_hop=entry_hop, shadow_landing=shadow)
    sch = Schedule(pattern='self_toss', skills=(sk,),
                   flight_s=sc.flight_s(_APEX), beat_s=1.0, transit_s=0.6,
                   dwell_s=0.27, t0_abs_s=10.0)
    inst = _Installer()
    x = ex.SkillExecutor(sch, inst, learner=learner, checker=checker,
                         boxes=boxes)
    lines = x.tick(sk.dispatch_s())
    return inst, x, lines


def test_the_envelope_is_evaluated_on_the_carried_throw(cell, env_spy):
    spy = env_spy()
    _s, inst, _x, _l, _st = _run_catch(cell, _ScriptedChecker(),
                                       learner=_Learner([0.01, 0.0, _APEX]))
    then = _then_throw(inst)
    assert len(spy.calls) == 1
    flight, speed = spy.calls[0]
    assert flight == pytest.approx(sc.flight_s(_APEX), abs=1e-12)
    v = bal.launch_velocity(then.site_mm, then.target_mm, then.flight_s)
    assert speed == pytest.approx(float(np.linalg.norm(v)) / 1000.0,
                                  abs=1e-12)


def test_the_envelope_is_evaluated_on_a_launch_throw_and_no_check_is_solved(
        env_spy):
    spy = env_spy()
    chk = _ScriptedChecker()
    inst, x, lines = _throw_run(chk, learner=_Learner([0.004, 0.0, 0.9]))
    assert len(spy.calls) == 1 and chk.calls == []
    term = inst.calls[0][1]
    assert term.flight_s == pytest.approx(sc.flight_s(0.9))
    np.testing.assert_allclose(term.target_mm,
                               _P1.catch_site_mm() + [4.0, 0.0, 0.0])
    assert any('passed the envelope' in ln for ln in lines)


def test_a_hop_entry_takes_the_envelope_not_the_box_and_no_check(env_spy):
    """The hop's apex-only clip to the same-site box is REPLACED online: a
    box whose apex ceiling is 0.90 does not clip the learner's 0.93."""
    spy = env_spy()
    chk = _ScriptedChecker()
    box = adm.AdmissibleBox(
        site_pair=(_P1.name, _P1.name), apex_band_m=(0.9, 1.0),
        landing_xy_m=((-0.05, 0.05), (-0.05, 0.05)), apex_m=(0.3, 0.90),
        pattern='self_toss', release_site_xy_mm=tuple(_P1.cup_mm[:2]),
        target_site_xy_mm=tuple(_P1.cup_mm[:2]),
        limits={'leg_vel_mmps': 1.0, 'leg_acc_mmps2': 1.0,
                'leg_jerk_mmps3': 1.0, 'hand_acc_rps2': 1.0},
        gate_hash='0123456789ab', swept_at='2026-11-06', dwell_s=0.27,
        planner_mode='pose', carry_zvd=False)
    inst, x, _l = _throw_run(chk, learner=_Learner([0.0, 0.0, 0.93]),
                             entry_hop=True, boxes=[box])
    assert len(spy.calls) == 1 and chk.calls == []
    assert inst.calls[0][1].flight_s == pytest.approx(sc.flight_s(0.93))


def test_a_shadow_landing_throw_bypasses_the_envelope_and_the_checker(
        env_spy):
    spy = env_spy(refuse='all')
    chk = _ScriptedChecker()
    inst, x, _l = _throw_run(chk, learner=_Learner([0.03, 0.0, 1.2]),
                             shadow=True)
    assert spy.calls == [] and chk.calls == [] and len(inst.calls) == 1
    assert inst.calls[0][1].flight_s == pytest.approx(sc.flight_s(_APEX))


def test_an_envelope_refused_launch_throw_falls_back_to_the_model(env_spy):
    spy = env_spy(refuse=(sc.flight_s(1.2),))
    inst, x, lines = _throw_run(_ScriptedChecker(),
                                learner=_Learner([0.004, 0.0, 1.2]))
    assert len(spy.calls) == 2
    term = inst.calls[0][1]
    assert term.flight_s == pytest.approx(sc.flight_s(_APEX))
    np.testing.assert_allclose(term.target_mm, _P1.catch_site_mm())
    assert any('refused ENVELOPE:TEST' in ln and 'model command' in ln
               for ln in lines)


@pytest.mark.parametrize('entry_hop', [False, True], ids=['launch', 'hop'])
def test_an_envelope_refused_throw_and_model_end_the_attempt(env_spy,
                                                             entry_hop):
    env_spy(refuse='all')
    inst, x, _l = _throw_run(_ScriptedChecker(),
                             learner=_Learner([0.004, 0.0, 1.2]),
                             entry_hop=entry_hop)
    assert x.attempt_ended and x.end_code == ex.NO_ADMISSIBLE_COMMAND
    assert inst.calls == [] and 'ENVELOPE:TEST' in x.end_message


def test_an_apex_only_envelope_refused_candidate_resolves_to_the_model(
        cell, env_spy):
    """Candidate apex 1.2 refused by the envelope (no solve); the model's
    0.95 passes its check. An apex-only candidate has no xy line, so the
    1 mm back-off consumes it and the MODEL command is installed after ONE
    checker call (``admissible.resolve_command``'s apex-only rule)."""
    env_spy(refuse=(sc.flight_s(1.2),))
    chk = _ScriptedChecker()
    _s, inst, x, lines, _st = _run_catch(cell, chk,
                                         learner=_Learner([0.0, 0.0, 1.2]))
    assert not x.attempt_ended, x.end_message
    assert len(chk.calls) == 1
    then = _then_throw(inst)
    assert then.flight_s == pytest.approx(sc.flight_s(_APEX), abs=1e-12)
    assert any('refused ENVELOPE:TEST' in ln and 'model command' in ln
               for ln in lines)


def test_an_envelope_refused_carried_candidate_with_xy_steps(cell, env_spy):
    env_spy(refuse=(sc.flight_s(1.2),))
    chk = _ScriptedChecker()
    _s, inst, x, lines, _st = _run_catch(cell, chk,
                                         learner=_Learner([0.010, 0.0, 1.2]))
    assert not x.attempt_ended, x.end_message
    assert len(chk.calls) == 2                  # model + stepped
    then = _then_throw(inst)
    s2 = 0.5 - 0.001 / 0.010
    assert then.flight_s == pytest.approx(
        sc.flight_s(_APEX + s2 * (1.2 - _APEX)), abs=1e-9)
    np.testing.assert_allclose(_commanded_xy_mm(cell, then), [10.0 * s2, 0.0],
                               atol=1e-9)
    assert any('stepped to' in ln for ln in lines)


def test_an_envelope_refused_carried_candidate_and_model_end_the_attempt(
        cell, env_spy):
    env_spy(refuse='all')
    chk = _ScriptedChecker()
    _s, inst, x, _l, _st = _run_catch(cell, chk,
                                      learner=_Learner([0.010, 0.0, 1.2]))
    assert x.attempt_ended and x.end_code == ex.NO_ADMISSIBLE_COMMAND
    assert inst.calls == [] and chk.calls == []


# ---------------------------------------------------------------------------
# (4) checker=None keeps the box path
# ---------------------------------------------------------------------------

def test_no_checker_keeps_the_box_path_and_a_checker_ignores_the_box(env_spy):
    box = adm.AdmissibleBox(
        site_pair=(_P1.name, _P1.name), apex_band_m=(0.9, 1.0),
        landing_xy_m=((-0.002, 0.002), (-0.05, 0.05)), apex_m=(0.3, 1.5),
        pattern='self_toss', release_site_xy_mm=tuple(_P1.cup_mm[:2]),
        target_site_xy_mm=tuple(_P1.cup_mm[:2]),
        limits={'leg_vel_mmps': 1.0, 'leg_acc_mmps2': 1.0,
                'leg_jerk_mmps3': 1.0, 'hand_acc_rps2': 1.0},
        gate_hash='0123456789ab', swept_at='2026-11-06', dwell_s=0.27,
        planner_mode='pose', carry_zvd=False)
    spy = env_spy()
    learner = _Learner([0.010, 0.0, _APEX])
    inst, _x, _l = _throw_run(None, learner=learner, boxes=[box])
    np.testing.assert_allclose(inst.calls[0][1].target_mm,
                               _P1.catch_site_mm() + [2.0, 0.0, 0.0])
    assert spy.calls == []                      # no envelope on the box path
    inst, _x, _l = _throw_run(_ScriptedChecker(), learner=learner,
                              boxes=[box])
    np.testing.assert_allclose(inst.calls[0][1].target_mm,
                               _P1.catch_site_mm() + [10.0, 0.0, 0.0])


# ---------------------------------------------------------------------------
# (5) the EARLY command (the P3a logbook entry's Discussion): the eager form of the check
# ---------------------------------------------------------------------------
#
# Driver: a THROW of ball 1 at _P1, then the CATCH-with-throw of that ball
# (window = the flight, so its base is the release: the columns handoff
# shape). The catch's prior is the THROW's y_d landing (no tracker unless a
# test wires one), so the early rule (c) has a side-effect-free landing.
# Every checker here is scripted unless named REAL.

_FLIGHT = sc.flight_s(_APEX)
_T_REL = 10.0


class _SeqLearner:
    """``command`` answers ``us[i]`` on its i-th call (the last one after);
    optional ``predict`` = ``u + predict_bias`` (a learned prior)."""

    def __init__(self, us, predict_bias=None):
        self.us = [np.asarray(u, dtype=float) for u in us]
        self.n = 0
        self.local_model = None
        if predict_bias is not None:
            bias = np.array([predict_bias[0], predict_bias[1], 0.0])
            self.predict = lambda x, u, y_d: np.asarray(u, float) + bias

    def command(self, x, y_d):
        u = self.us[min(self.n, len(self.us) - 1)]
        self.n += 1
        return u.copy()


def _throw_catch_schedule(early_throw_t=None, handoff=True):
    """``handoff`` = the catch carries ``HANDOFF_LEAD_S``, as
    ``schedule._assign_leads`` gives a skill whose base is the release (a
    PINNED splice); ``False`` = the kind's own install lead."""
    s = _P1
    t_land = _T_REL + _FLIGHT
    y_d = (np.zeros(2), _APEX)
    skills = []
    if early_throw_t is not None:      # ball 0, never caught: an outcome row
        skills.append(Skill(kind=sg.THROW, ball_id=0, site=_P2,
                            t_abs_s=early_throw_t, window_s=0.4, y_d=y_d,
                            target=_P2))
    skills.append(Skill(kind=sg.THROW, ball_id=1, site=s, t_abs_s=_T_REL,
                        window_s=0.4, y_d=y_d, target=s))
    tt = sc.ThenThrow(t_release_abs_s=t_land + 0.27, y_d=y_d, target=s)
    skills.append(Skill(kind=sg.CATCH, ball_id=1, site=s, t_abs_s=t_land,
                        window_s=_FLIGHT, then_throw=tt,
                        lead_s=sc.HANDOFF_LEAD_S if handoff else None))
    return Schedule(pattern='self_toss', skills=tuple(skills),
                    flight_s=_FLIGHT, beat_s=1.0, transit_s=0.6, dwell_s=0.27,
                    t0_abs_s=9.0)


def _early_exec(checker, learner, sch=None, **kw):
    sch = _throw_catch_schedule() if sch is None else sch
    inst = _Installer()
    rows = []
    x = ex.SkillExecutor(sch, inst, learner=learner, checker=checker,
                         on_experience=rows.append, **kw)
    return sch, inst, x


def _dispatch_throws(sch, x):
    for sk in sch.skills:
        if sk.kind == sg.THROW:
            x.tick(sk.dispatch_s())
    assert not x.attempt_ended, x.end_message


def test_the_early_path_checks_before_the_dispatch_and_not_at_it(cell):
    """The checker runs ONCE, at a tick before ``dispatch_s``; the dispatch
    makes ZERO checker calls, installs the early-resolved command, and the
    cache / pending row carry the DISPATCH-time ``x`` (a tracker fit +5 mm
    off the prior at dispatch: same-site fold, so x[2] = +0.005 m)."""
    chk = _ScriptedChecker()
    learner = _SeqLearner([[0.0, 0.0, _APEX], [0.004, 0.0, _APEX]])
    state = {'landing': None}
    sch, inst, x = _early_exec(chk, learner,
                               tracker=lambda b: state['landing'])
    _dispatch_throws(sch, x)
    catch = sch.skills[-1]
    t_d = catch.dispatch_s()
    t_early = t_d - 0.2
    x.tick(t_early)
    assert len(chk.calls) == 1 and chk.calls[0][2] == t_early < t_d
    assert x.dispatched == {0}
    state['landing'] = ex.Landing(
        pos_mm=_P1.catch_site_mm() + np.array([5.0, 0.0, 0.0]),
        vel_mm_s=np.array([0.0, 0.0, -4000.0]),
        t_land_abs_s=float(catch.t_abs_s), from_fit=True)
    lines = x.tick(t_d)
    assert len(chk.calls) == 1                  # nothing checked at dispatch
    assert [c[0] for c in inst.calls] == [sg.THROW, sg.CATCH]
    _x, u_dy, u_apex = x._u_cache[1]
    np.testing.assert_allclose(u_dy, [0.004, 0.0], atol=1e-12)
    assert u_apex == _APEX
    np.testing.assert_allclose(_x[2:4], [0.005, 0.0], atol=1e-12)
    pend = [p for p in x._pending_outcomes if p.ball_id == 1][-1]
    np.testing.assert_allclose(pend.x[2:4], [0.005, 0.0], atol=1e-12)
    then = inst.calls[1][1].then_throw
    np.testing.assert_allclose(
        then.target_mm[:2] - _P1.catch_site_mm()[:2]
        - x._offset_flyback_mm(1, catch.then_throw.y_d, _APEX)[:2],
        [4.0, 0.0], atol=1e-9)
    assert any('ADMISSIBILITY-EARLY skill 1: checked 200 ms before dispatch'
               in ln and 'early resolution is sent' in ln for ln in lines)
    assert any('ADMISSIBILITY skill 1:' in ln for ln in lines)


def test_no_early_check_outside_the_window_or_at_the_dispatch_tick():
    chk = _ScriptedChecker()
    sch, _inst, x = _early_exec(chk, _SeqLearner([[0.0, 0.0, _APEX]]))
    _dispatch_throws(sch, x)
    t_d = sch.skills[-1].dispatch_s()
    x.tick(t_d - ex.EARLY_COMMAND_MAX_S - 0.01)
    assert chk.calls == []
    x.tick(t_d)                                 # (d) fails: synchronous
    assert len(chk.calls) == 1 and chk.calls[0][2] == t_d


def test_a_pending_outcome_due_before_the_dispatch_holds_the_early_check():
    """Rule (b), driven through the REAL outcome path (no hand-poked rows):
    ball 0's throw is never caught, so its row finalises
    ``CAUGHT_WINDOW_S`` after its scheduled landing -- placed 0.10 s before
    the catch's dispatch, inside the 0.30 s early window. No early check
    may run while that row is pending; the first one runs on the tick after
    it finalised, and still before the dispatch."""
    t_d = _T_REL + _FLIGHT - _FLIGHT - sc.HANDOFF_LEAD_S
    t_fin = t_d - 0.10
    t_rel0 = t_fin - ex.CAUGHT_WINDOW_S - _FLIGHT
    sch = _throw_catch_schedule(early_throw_t=t_rel0)
    assert sch.skills[-1].dispatch_s() == pytest.approx(t_d)
    chk = _ScriptedChecker()
    sch, _inst, x = _early_exec(chk, _SeqLearner([[0.0, 0.0, _APEX]]), sch)
    _dispatch_throws(sch, x)
    ball0 = [p for p in x._pending_outcomes if p.ball_id == 0]
    assert len(ball0) == 1
    assert x._outcome_window(ball0[0])[1] == pytest.approx(t_fin)
    t = t_d - ex.EARLY_COMMAND_MAX_S + 0.001
    first = None
    while t < t_d and first is None:
        pending_before = any(p.ball_id == 0 for p in x._pending_outcomes)
        x.tick(t)
        if chk.calls:
            first = t
            assert not pending_before, 'checked with ball 0 still pending'
        t += 0.025
    assert first is not None and t_fin <= first < t_d
    assert not any(p.ball_id == 0 for p in x._pending_outcomes)


@pytest.mark.parametrize('moved, rechecked', [
    ((0.0005, 0.0, 0.0), False),
    ((0.0, -0.0009, 0.0015), False),
    ((0.002, 0.0, 0.0), True),
    ((0.0, 0.0, 0.003), True),
], ids=['0.5mm', '0.9mm+apex1.5mm', '2mm', 'apex3mm'])
def test_a_candidate_that_moved_past_the_tolerance_is_rechecked(moved,
                                                                rechecked):
    u0 = np.array([0.010, 0.0, _APEX])
    u1 = u0 + np.asarray(moved)
    chk = _ScriptedChecker()
    sch, inst, x = _early_exec(chk, _SeqLearner([[0.0, 0.0, _APEX], u0, u1]))
    _dispatch_throws(sch, x)
    t_d = sch.skills[-1].dispatch_s()
    x.tick(t_d - 0.1)
    assert len(chk.calls) == 1
    lines = x.tick(t_d)
    assert len(chk.calls) == (2 if rechecked else 1)
    if rechecked:
        assert chk.calls[1][2] == t_d
    _x, u_dy, u_apex = x._u_cache[1]
    sent = u1 if rechecked else u0
    np.testing.assert_allclose(u_dy, sent[:2], atol=1e-12)
    assert u_apex == pytest.approx(sent[2], abs=1e-12)
    want = ('candidate moved (%+.1f, %+.1f) mm apex %+.1f mm since the early '
            'check -> re-checked at dispatch'
            % (moved[0] * 1e3, moved[1] * 1e3, moved[2] * 1e3))
    assert any(want in ln for ln in lines) == rechecked
    assert any('early resolution is sent' in ln
               for ln in lines) == (not rechecked)


def test_an_early_refusal_ends_the_attempt_at_the_dispatch():
    refuse = adm.CommandVerdict(False, -0.2, 'LIMIT_VEL', True)
    chk = _ScriptedChecker(refuse)
    sch, inst, x = _early_exec(chk, None)       # no Learner: cand == model
    _dispatch_throws(sch, x)
    t_d = sch.skills[-1].dispatch_s()
    x.tick(t_d - 0.15)
    assert len(chk.calls) == 1 and not x.attempt_ended
    lines = x.tick(t_d)
    assert x.attempt_ended and x.end_code == ex.NO_ADMISSIBLE_COMMAND
    assert len(chk.calls) == 1                  # re-raised, not re-checked
    assert [c[0] for c in inst.calls] == [sg.THROW]
    assert 'LIMIT_VEL' in x.end_message
    assert any(ln.startswith('%.3f END %s at skill 1' % (
        t_d, ex.NO_ADMISSIBLE_COMMAND)) for ln in lines)


def test_an_unexpected_early_error_never_escapes_the_tick():
    """A Learner raising ``KeyError`` on the EARLY call only: the early tick
    raises nothing and stores no entry; the dispatch then computes and checks
    the command synchronously and installs."""
    class _RaisesOnce(_SeqLearner):
        def command(self, x, y_d):
            if self.n == 1:                     # call 0 = the THROW; 1 = early
                self.n += 1
                raise KeyError('early read')
            return super().command(x, y_d)

    chk = _ScriptedChecker()
    sch, inst, x = _early_exec(chk, _RaisesOnce([[0.0, 0.0, _APEX]]))
    _dispatch_throws(sch, x)
    t_d = sch.skills[-1].dispatch_s()
    x.tick(t_d - 0.2)                           # must not raise
    assert x._early_by_idx == {} and chk.calls == []
    assert not x.attempt_ended
    x.tick(t_d)
    assert len(chk.calls) == 1 and chk.calls[0][2] == t_d    # synchronous
    assert [c[0] for c in inst.calls] == [sg.THROW, sg.CATCH]


def test_early_command_off_checks_only_at_the_dispatch():
    chk = _ScriptedChecker()
    sch, inst, x = _early_exec(chk, _SeqLearner([[0.0, 0.0, _APEX]]),
                               early_command=False)
    _dispatch_throws(sch, x)
    t_d = sch.skills[-1].dispatch_s()
    for dt in (0.29, 0.2, 0.1, 0.01):
        x.tick(t_d - dt)
    assert chk.calls == [] and x._early_by_idx == {}
    lines = x.tick(t_d)
    assert len(chk.calls) == 1 and chk.calls[0][2] == t_d
    assert not any('ADMISSIBILITY-EARLY' in ln for ln in lines)


def test_a_non_handoff_carried_catch_is_checked_only_at_its_dispatch():
    """Rule (e): a CATCH-with-throw on the kind's own lead (not a pinned
    splice: a feed catch, a first catch after a REST) is never checked
    early, with ``early_command=True``, through the whole window."""
    chk = _ScriptedChecker()
    sch = _throw_catch_schedule(handoff=False)
    assert sch.skills[-1].lead_s != pytest.approx(sc.HANDOFF_LEAD_S)
    sch, inst, x = _early_exec(chk, _SeqLearner([[0.0, 0.0, _APEX]]), sch)
    _dispatch_throws(sch, x)
    t_d = sch.skills[-1].dispatch_s()
    for dt in (0.29, 0.2, 0.1, 0.01):
        x.tick(t_d - dt)
    assert chk.calls == [] and x._early_by_idx == {}
    lines = x.tick(t_d)
    assert len(chk.calls) == 1 and chk.calls[0][2] == t_d
    assert [c[0] for c in inst.calls] == [sg.THROW, sg.CATCH]
    assert not any('ADMISSIBILITY-EARLY' in ln for ln in lines)


def test_no_checker_makes_no_early_resolution():
    sch, inst, x = _early_exec(None, _SeqLearner([[0.0, 0.0, _APEX]]))
    _dispatch_throws(sch, x)
    t_d = sch.skills[-1].dispatch_s()
    x.tick(t_d - 0.1)
    assert x._early_by_idx == {} and 1 not in x._u_cache
    x.tick(t_d)
    assert [c[0] for c in inst.calls] == [sg.THROW, sg.CATCH]


def _side_effect_state(x):
    return dict(
        fit_latch=dict(x._fit_latch_by_idx), hand=set(x._hand_corrected),
        notes=list(x._pending_notes), identity=set(x._identity_refused),
        resend_notes=set(x._resend_notes), learned=set(x._learned_u),
        cache=sorted(x._u_cache), release=sorted(x._carried_release_mm),
        caught=sorted(x._carried_catch_xy_mm), dispatched=set(x.dispatched),
        live=x._live_catch)


@pytest.mark.parametrize('aim', [ex.AIM_TRACKER, ex.AIM_SCHEDULE_HAND])
def test_an_early_command_has_no_side_effects(aim):
    """Two executors ticked identically, early command ON vs OFF: after the
    early tick every piece of state the dispatch's aim / clamp / Learner
    path writes is IDENTICAL -- ``_fit_latch_by_idx``, ``_hand_corrected``,
    ``_pending_notes``, the identity-refused set, the once-keys, the learned
    marks, the cache. The learned prior is 50 mm off the schedule landing
    with a 20 mm authority, so the clamp BITES (a ``note=True`` clamp would
    queue AIM-LATERAL-CLAMPED), and the tracker serves the WRONG ball's
    estimate (an identity-refused line on any tracker read)."""
    wrong = ex.Landing(pos_mm=_P2.catch_site_mm(),
                       vel_mm_s=np.array([0.0, 0.0, -4000.0]),
                       t_land_abs_s=_T_REL + _FLIGHT, from_fit=True)
    states, early = [], []
    for on in (True, False):
        chk = _ScriptedChecker()
        sch, _inst, x = _early_exec(
            chk, _SeqLearner([[0.0, 0.0, _APEX]], predict_bias=(0.05, 0.0)),
            tracker=lambda b: wrong, catch_aim_source=aim,
            launch_ratio=lambda b, t: 1.05, lateral_authority_m=0.020,
            early_command=on)
        _dispatch_throws(sch, x)
        x.tick(sch.skills[-1].dispatch_s() - 0.2)
        states.append(_side_effect_state(x))
        early.append((len(chk.calls), dict(x._early_by_idx)))
    assert early[0][0] == 1 and 1 in early[0][1]
    assert early[1] == (0, {})
    assert states[0] == states[1]


# ---------------------------------------------------------------------------
# (6) the early command end to end, REAL planner: two-ball columns
# ---------------------------------------------------------------------------

_T0_ABS = 100.0
_DT = float(ex.hw.JB_TRAJ_KNOT_DT_S)


def test_a_columns_schedule_runs_with_every_carried_command_checked_early():
    """``compile_columns`` (R5 point: 125 mm, Apex 0.95, Dwell 0.27, 6
    throws) -> ``SkillExecutor(checker=, early_command=True)`` ->
    ``install_segment`` / ``check_segment`` on ONE local record, at the R5
    sitting limits (350 / 5000 / 200000), a perfect analytic tracker.

    Every non-shadow CATCH-with-throw is checked EARLY (inside
    ``[dispatch - EARLY_COMMAND_MAX_S, dispatch)``), never at its dispatch --
    except Ball B's first catch, thrown before ``t0``: no prior, so rule (c)
    holds it to the synchronous check -- and every install is accepted. And the early check is the check the
    dispatch would have run: re-checking the same stand-in at the dispatch
    instant on the record the dispatch installs onto gives the SAME margin
    (a handoff catch's splice snaps to the head's release knot whatever the
    instant, ``_snap_to_release``, so the seed is the same)."""
    from jugglebot.motion.trajectory.limits import TrajectoryLimits
    limits = TrajectoryLimits.from_config(ex.hw).with_sitting_limits(
        leg_vel_mmps=350.0, leg_acc_mmps2=5000.0, leg_jerk_mmps3=200000.0,
        hand_acc_rps2=float(ex.hw.JB_TRAJ_HAND_ACC_LIMIT_RPS2))
    geom = StewartGeometry()
    sites = st.columns_sites(125.0)
    sched = sc.compile_columns(
        sc.Pattern(sites=sites, apex_m=_APEX, dwell_s=0.27, n_throws=6),
        _T0_ABS)
    arrival = np.array([0.0, 0.0, -0.5 * 9806.0 * sched.flight_s])
    landings = {}
    for sk in sched.skills:
        if sk.kind == sg.CATCH:
            landings.setdefault(sk.ball_id, []).append(ex.Landing(
                pos_mm=sk.site.catch_site_mm(), vel_mm_s=arrival.copy(),
                t_land_abs_s=float(sk.t_abs_s), from_fit=True))
    clock = {'t': _T0_ABS - 1.0}

    def tracker(ball_id):
        for land in landings.get(ball_id, ()):
            if land.t_land_abs_s > clock['t'] - 0.05:
                return land
        return None

    state = {'record': None}
    installs, checks = [], []

    def installer(kind, terminal, t_now_s, ball_id=0):
        rec = state['record']
        seed = ex._rest_state_at(sites[0].rest_site_mm()) if rec is None else None
        new_rec, res, _seg = ex.install_segment(
            rec, seed, kind, terminal, t_now_s, limits=limits, geom=geom)
        assert res.accepted, '%s: %s' % (res.code, res.message)
        installs.append((kind, t_now_s, rec))
        state['record'] = new_rec
        return res

    def checker(kind, terminal, t_abs_s, *, ball_id):
        rec = state['record']
        chk = ex.check_segment(rec, None, kind, terminal, t_abs_s,
                               limits=limits, geom=geom)
        checks.append((kind, terminal, t_abs_s, rec, chk))
        ok = chk.planned and chk.margin >= 0.0
        return adm.CommandVerdict(ok, chk.margin,
                                  'OK' if ok else chk.code, True)

    execu = ex.SkillExecutor(sched, installer, tracker=tracker,
                             learner=_Learner([0.0, 0.0, _APEX]),
                             checker=checker, early_command=True)
    t_end = max(sk.t_abs_s for sk in sched.skills) + 0.5
    t = clock['t']
    while t < t_end and not execu.attempt_ended:
        clock['t'] = t
        execu.tick(t)
        t += _DT / 10.0
    assert not execu.attempt_ended, execu.end_message
    carried = [i for i, sk in enumerate(sched.skills)
               if sk.kind == sg.CATCH and sk.then_throw is not None
               and not sk.then_throw.shadow_landing]
    assert len(carried) >= 4
    assert len(checks) == len(carried)          # one check each, all early
    assert sorted(execu._u_cache) == sorted(
        set(carried) | {i for i, sk in enumerate(sched.skills)
                        if sk.kind == sg.THROW
                        or (sk.kind == sg.CATCH and sk.then_throw)})
    disp = {}
    for i, (kind, t_now, rec) in zip(
            [r[0] for r in execu.results], installs):
        disp[i] = (t_now, rec)
    n_sync = n_early = 0
    for (kind, terminal, t_chk, rec, chk), i in zip(checks, carried):
        assert kind == sg.CATCH and chk.planned and chk.margin >= 0.0, chk
        t_disp, rec_disp = disp[i]
        d_s = sched.skills[i].dispatch_s()
        if ex._previous_release(sched, i, sched.skills[i].ball_id) is None:
            # Ball B's first catch: thrown before t0, so no prior exists --
            # rule (c) has no side-effect-free landing; checked at dispatch.
            assert t_chk == t_disp >= d_s
            n_sync += 1
            continue
        assert d_s - ex.EARLY_COMMAND_MAX_S <= t_chk < d_s <= t_disp + 1e-9
        n_early += 1
        assert rec_disp is rec, 'an install landed between check and dispatch'
        again = ex.check_segment(rec, None, kind, terminal, t_disp,
                                 limits=limits, geom=geom)
        assert again.planned
        assert again.margin == pytest.approx(chk.margin, abs=1e-9)
    assert n_sync == 1 and n_early == len(carried) - 1 >= 3


# ---------------------------------------------------------------------------
# (7) the stand-in by where the ball is next caught: hop (reflect), cross-site
#     asymmetric (envelope only). Follow-up to the 2026-11-06 sim hop run,
#     whose model command was refused UNVERIFIED on a stand-in moved 250 mm.
# ---------------------------------------------------------------------------

_HOP_P4_LIMITS = (600.0, 8000.0, 200000.0, 4500.0)


def _hop_catch_schedule(site, target, T, takeoff_site=None, tau=None):
    """ONE same-site fold at ``site`` whose carried Throw goes to ``target``
    (the hop's catch shape), on the probe-cell clock; the tracker's landing
    arrives along the hop from ``target`` (``takeoff_site``). ``tau`` (the
    landing instant) defaults to the Columns transit."""
    tau = sc.transit_s(T, cell_dwell()) if tau is None else float(tau)
    tt = sc.ThenThrow(t_release_abs_s=tau + cell_dwell(),
                      y_d=(np.zeros(2), _APEX), target=target)
    sk = Skill(kind=sg.CATCH, ball_id=1, site=site, t_abs_s=tau,
               window_s=0.25, then_throw=tt)
    sch = Schedule(pattern='hop', skills=(sk,), flight_s=T, beat_s=T,
                   transit_s=tau, dwell_s=cell_dwell(), t0_abs_s=0.0)
    src = target if takeoff_site is None else takeoff_site
    v0 = bal.launch_velocity(src.throw_site_mm(), site.catch_site_mm(), T)
    landing = ex.Landing(pos_mm=site.catch_site_mm(),
                         vel_mm_s=bal.arrival_velocity(v0, T),
                         t_land_abs_s=tau, from_fit=True)
    return sch, landing


def test_a_hop_catch_is_checked_on_its_own_segment_reflected():
    s = st.Site('S', np.array([125.0, 0.0, st.CATCH_CUP_Z_MM]))
    q = st.Site('Q', np.array([-125.0, 0.0, st.CATCH_CUP_Z_MM]))
    sch, landing = _hop_catch_schedule(s, q, sc.flight_s(_APEX))
    chk = _ScriptedChecker()
    inst = _Installer()
    u = np.array([0.010, 0.004, _APEX])
    x = ex.SkillExecutor(sch, inst, tracker=lambda b: landing, checker=chk,
                         learner=_Learner(u, bias_xy=(0.006, -0.002)))
    x.tick(sch.skills[0].dispatch_s())
    assert len(chk.calls) == 1
    standin = chk.calls[0][1]
    y_hat_mm = np.array([16.0, 2.0])               # u + bias
    want = s.catch_site_mm()[:2] - y_hat_mm         # reflected, at THIS site
    np.testing.assert_allclose(standin.landing_mm[:2], want, atol=1e-9)
    np.testing.assert_allclose(standin.then_throw.site_mm[:2], want,
                               atol=1e-9)           # release rides it
    fly = ex.SkillExecutor._flyback_for_offset_mm(-y_hat_mm, sch.skills[0]
                                                  .then_throw.y_d, _APEX)
    np.testing.assert_allclose(
        standin.then_throw.target_mm,
        q.catch_site_mm() + np.array([-10.0, -4.0, 0.0]) + fly, atol=1e-9)
    assert standin.t_land_s == landing.t_land_abs_s     # this Catch's clock
    np.testing.assert_allclose(standin.landing_vel_mm_s, landing.vel_mm_s)
    # the command sent is u itself, aimed at the real target
    then = inst.calls[0][1].then_throw
    np.testing.assert_allclose(then.target_mm[:2] - q.catch_site_mm()[:2],
                               [10.0, 4.0], atol=1e-9)


def test_an_asymmetric_cross_site_catch_is_admitted_on_the_envelope_alone(
        env_spy):
    s = st.Site('S', np.array([125.0, 0.0, st.CATCH_CUP_Z_MM]))
    q = st.Site('Q', np.array([-100.0, 0.0, st.CATCH_CUP_Z_MM]))
    sch, landing = _hop_catch_schedule(s, q, sc.flight_s(_APEX))
    spy = env_spy()
    chk = _ScriptedChecker()
    inst = _Installer()
    x = ex.SkillExecutor(sch, inst, tracker=lambda b: landing, checker=chk,
                         learner=_Learner([0.010, 0.0, _APEX]))
    lines = x.tick(sch.skills[0].dispatch_s())
    assert chk.calls == [] and len(spy.calls) == 1
    assert not x.attempt_ended and len(inst.calls) == 1
    assert any('ADMISSIBILITY skill 0: no stand-in for a cross-site Throw '
               '(S -> Q): envelope only' in ln for ln in lines)
    then = inst.calls[0][1].then_throw
    np.testing.assert_allclose(then.target_mm[:2] - q.catch_site_mm()[:2],
                               [10.0, 0.0], atol=1e-9)


def test_an_asymmetric_cross_site_envelope_refusal_still_ends_the_attempt(
        env_spy):
    s = st.Site('S', np.array([125.0, 0.0, st.CATCH_CUP_Z_MM]))
    q = st.Site('Q', np.array([-100.0, 0.0, st.CATCH_CUP_Z_MM]))
    sch, landing = _hop_catch_schedule(s, q, sc.flight_s(_APEX))
    env_spy(refuse='all')
    chk = _ScriptedChecker()
    inst = _Installer()
    x = ex.SkillExecutor(sch, inst, tracker=lambda b: landing, checker=chk,
                         learner=_Learner([0.010, 0.0, _APEX]))
    x.tick(sch.skills[0].dispatch_s())
    assert x.end_code == ex.NO_ADMISSIBLE_COMMAND
    assert inst.calls == [] and chk.calls == []


def test_a_hop_catch_model_command_passes_its_real_check(probe):
    """REAL planner, P4 point, Sites +-125 mm: the hop Catch at S (seeded by
    the post-release state of a launch at Q at t = 0, the probe cell's
    seed), the ball that launch threw landing at S one flight later, its
    carried Throw back to Q. The model command
    (y_d, no Learner) passes the reflected stand-in's check; the pre-fix
    stand-in (the landing moved to Q on this clock) was refused."""
    limits = probe.asw._limits(*_HOP_P4_LIMITS)
    c = probe.Cell((125.0, 0.0), _APEX, limits, StewartGeometry(),
                   sg.SegmentConfig(planner_mode='pose'))
    assert c.ok, c.code
    sch, landing = _hop_catch_schedule(c.site, c.partner, c.T, tau=c.T)
    chk = _RealChecker(c)
    inst = _Installer()
    x = ex.SkillExecutor(sch, inst, tracker=lambda b: landing, checker=chk)
    x.tick(sch.skills[0].dispatch_s())
    assert not x.attempt_ended, x.end_message
    assert len(chk.calls) == 1
    v = chk.calls[0][4]
    assert v.ok and v.margin >= 0.0, v
    np.testing.assert_allclose(chk.calls[0][1].landing_mm,
                               c.site.catch_site_mm(), atol=1e-9)
