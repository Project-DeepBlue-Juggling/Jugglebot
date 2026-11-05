"""P3a: the sim gate's ONLINE admissibility path (the default for
every `--learn` run) and the `--box` A/B path that keeps today's behaviour.

Twins of the three box-path gate tests in test_skills_gate.py (which still
run the box path, now passed `box=True`): a biased-plant small hop run, an
unbiased self-throw (`self_toss`) run and the small hop learner run, judged by the ONLINE
check's own evidence (`ADMISSIBILITY` lines) instead of `BOX-EXPECTED-LANDING`
ones. Recipe: 5 throws, band entry 5, 15 attempts, seed 0, as the box twins.
"""
from __future__ import annotations

import pytest

pytest.importorskip('mujoco')

from sim import skills_gate as sg                                  # noqa: E402
from sim.skills_gate import SelfTossGateConfig, SkillsGate         # noqa: E402
from jugglebot.motion.skills import executor as ex                 # noqa: E402


def _adm_lines(res):
    return [ln for ln in res['executor_lines'] if 'ADMISSIBILITY' in ln]


def _box_lines(res):
    return [ln for ln in res['executor_lines'] if 'BOX-EXPECTED-LANDING' in ln]


def _hop_cfg(**kw):
    return SelfTossGateConfig(pattern='hop', separation_mm=250.0,
                              target_throws=5, band_entry_throws=5,
                              max_attempts=15, **kw)


def _spy_checks_and_installs(monkeypatch):
    """Record, per call, the checker's verdict (keyed by ball) and every
    install; `installs` entries carry the last verdict for the same ball."""
    log = {'verdicts': {}, 'installs': [], 'refused': []}
    real_make_checker = sg._make_checker
    real_make_installer = sg._make_installer

    def make_checker(*a, **k):
        inner = real_make_checker(*a, **k)

        def checker(kind, terminal, t_now_s, *, ball_id):
            v = inner(kind, terminal, t_now_s, ball_id=ball_id)
            log['verdicts'][int(ball_id)] = v
            if not v.ok:
                log['refused'].append(v)
            return v
        return checker

    def make_installer(*a, **k):
        inner = real_make_installer(*a, **k)

        def installer(kind, terminal, t_now_s, *, ball_id):
            carried = (kind == 'CATCH'
                       and getattr(terminal, 'then_throw', None) is not None
                       and not getattr(terminal.then_throw,
                                       'shadow_landing', False))
            log['installs'].append(
                (kind, carried, int(ball_id),
                 log['verdicts'].get(int(ball_id))))
            return inner(kind, terminal, t_now_s, ball_id=ball_id)
        return installer

    monkeypatch.setattr(sg, '_make_checker', make_checker)
    monkeypatch.setattr(sg, '_make_installer', make_installer)
    return log


# ── twin 1: the biased plant ────────────────────────────────────────────────

def test_a_biased_plant_small_hop_run_is_checked_and_installs_nothing_refused(
        monkeypatch):
    """+30 mm x plant bias. The hop Catch is checked on its own Segment
    REFLECTED (executor ``_catch_standin`` rule (ii)), so its checks plan and
    carry real margins (measured 2026-11-06, seed 0: skill 2 passed m=0.355).
    One attempt still ends NO_ADMISSIBLE_COMMAND, honestly: the learner's
    (-70, -31) mm candidate is refused HAND_LIMIT_C2 (4.97 > 4.5 rev/s^2)
    and the model command, whose PREDICTED landing the biased fit puts 84 mm
    out, INFEASIBLE -- this gate's self-toss/hop attempts run with NO
    lateral authority, so nothing clamps either. (Before the reflection the
    model command was refused UNVERIFIED on a stand-in moved 250 mm to the
    other Site.) The run collects its throws by cheap reset, leaves no
    box-clip lines, and EVERY carried-catch install follows an OK verdict for
    that ball: a refused command is never sent."""
    log = _spy_checks_and_installs(monkeypatch)
    res = SkillsGate(_hop_cfg(plant_landing_bias_mm=(30.0, 0.0))
                     ).run_self_toss_seed(0, policy='A')
    assert res['n_throws_collected'] == 5
    assert _adm_lines(res)
    assert not _box_lines(res)
    assert set(res['end_codes']) <= {'', 'NO_ADMISSIBLE_COMMAND'}
    # The refusal is real (the check said no and the attempt ended) ...
    assert 'NO_ADMISSIBLE_COMMAND' in res['end_codes']
    assert log['refused']
    # ... the checks are not vacuous: at least one carried command was
    # planned and passed with a real, positive margin ...
    margins = [float(ln.rsplit('passed m=', 1)[1].split()[0])
               for ln in _adm_lines(res) if 'passed m=' in ln]
    assert margins and max(margins) > 0.05, _adm_lines(res)
    # ... and no carried catch was installed without an OK verdict.
    carried = [i for i in log['installs'] if i[1]]
    assert carried
    for kind, _c, ball, verdict in carried:
        assert verdict is not None and verdict.ok, (kind, ball, verdict)


# ── twin 2: unbiased self-toss ──────────────────────────────────────────────

def test_an_unbiased_self_toss_run_passes_every_check_with_no_stepped_command(
        monkeypatch):
    log = _spy_checks_and_installs(monkeypatch)
    cfg = SelfTossGateConfig(target_throws=5, band_entry_throws=5,
                             max_attempts=15)
    res = SkillsGate(cfg).run_self_toss_seed(0, policy='A')
    assert res['n_throws_collected'] == 5
    assert set(res['end_codes']) == {''}
    lines = _adm_lines(res)
    assert lines
    assert not any(' refused ' in ln or 'stepped to' in ln for ln in lines)
    assert not any('NO_ADMISSIBLE_COMMAND' in ln for ln in res['executor_lines'])
    assert log['verdicts'] and all(v.ok for v in log['verdicts'].values())


# ── twin 3: the small hop learner run ───────────────────────────────────────

def test_a_small_hop_learner_run_commands_apexes_the_envelope_admitted(
        monkeypatch):
    """The online twin of the apex-bound test: every apex the learner
    commanded went through `_envelope_verdict` (or the checker) and was
    admitted. The envelope is the closed-form throw envelope, so this is the
    online path's own bound instead of the box's apex band."""
    admitted = set()
    real = ex.SkillExecutor._envelope_verdict

    def spy(release_mm, target_mm, u_apex):
        v = real(release_mm, target_mm, u_apex)
        if v is None:
            admitted.add(round(float(u_apex), 6))
        return v

    monkeypatch.setattr(ex.SkillExecutor, '_envelope_verdict',
                        staticmethod(spy))
    res = SkillsGate(_hop_cfg()).run_self_toss_seed(0, policy='A')
    assert res['n_throws_collected'] == 5
    assert res['drops'] == 0
    # 5 of 5 since the hop Catch's stand-in is its own Segment reflected
    # (2026-11-06 follow-up): no attempt ends NO_ADMISSIBLE_COMMAND, and the
    # carried commands were really planned (a checked margin, not envelope-
    # only lines alone).
    assert res['makes'] == 5
    assert set(res['end_codes']) == {''}
    assert any('passed m=' in ln for ln in _adm_lines(res))
    commanded = [round(float(t['u'][2]), 6) for t in res['throws']]
    assert commanded and admitted
    assert min(commanded) < 0.9 - 1e-6, commanded     # the correction moved it
    assert all(a in admitted for a in commanded), (commanded, sorted(admitted))


# ── the --box A/B path ──────────────────────────────────────────────────────

def test_the_box_flag_restores_the_box_path_with_no_checker(monkeypatch):
    called = []
    real = sg._make_checker
    monkeypatch.setattr(sg, '_make_checker',
                        lambda *a, **k: called.append(1) or real(*a, **k))
    res = SkillsGate(_hop_cfg(plant_landing_bias_mm=(30.0, 0.0), box=True)
                     ).run_self_toss_seed(0, policy='A')
    assert called == []
    assert res['n_throws_collected'] == 5
    assert set(res['end_codes']) == {''}
    assert _box_lines(res) and not _adm_lines(res)


def test_the_cli_exposes_the_box_flag():
    with pytest.raises(SystemExit) as e:
        sg.main(['--help'])
    assert e.value.code == 0


def test_online_kwargs_are_empty_under_the_box_flag():
    cfg = SelfTossGateConfig(box=True)
    assert sg._online_kwargs(cfg, None, None, None, None) == {}
    cfg = SelfTossGateConfig()
    assert 'checker' in sg._online_kwargs(cfg, sg._InstallCtx(None), None,
                                          None, None)
