"""``sim/skills_gate.py --beat-change`` -- P4 (D-P4-7), a mid-pattern beat
change on the plain two-ball Columns gate.

**Marker policy: UNMARKED (per-commit)**, copied from ``test_skills_gate.py``:
the executor, the planner walk and the gate's criteria are on the
hardware-safety surface.  Cost is contained the same way: ONE seed and a
12-Throw stream (~9 s a run) per case; the five-seed 20-Throw sweeps run by
hand (``python sim/skills_gate.py --beat-change throw=6,apex=1.00``).

The real runs change to a configuration with a healthy nomination margin. An
Apex of 0.80 m is NOT one: at the gate's limits a STANDING Columns at 0.80 m
nominates with a 0.06 margin and then fails every seed at an install (measured
2026-11-06), and a change to it behaves the same (2 of 5 seeds). That is the
nomination's fidelity at a thin margin, not the change; see the P4 logbook
entry.

The criteria are pure over recorded data (:func:`sg._beat_criteria`), so the
fail-before cases corrupt a synthetic trace and watch each criterion fail;
one real run does the same by making the EXECUTOR request one Throw later
than the gate's independent recompute.
"""

from __future__ import annotations

import pytest

pytest.importorskip('mujoco')

import numpy as np                                                # noqa: E402

from jugglebot.motion.skills import executor as ex                # noqa: E402
from jugglebot.motion.skills import schedule as sk                # noqa: E402
from jugglebot.motion.skills import sites                         # noqa: E402

import sim.skills_gate as sg                                      # noqa: E402

_N = 12      # a short stream: Throws 0..11, change requested after Throw 5


def _run(spec: str, seed: int = 0, n_throws: int = _N):
    """One plain-gate trial with ``--beat-change spec``; returns
    ``(result, executors)`` (every executor the run built)."""
    cfg = sg.SkillsGateConfig(seeds=(seed,), n_throws=n_throws,
                              beat_change=sg.parse_beat_change(spec))
    built = []
    orig = ex.SkillExecutor.__init__

    def spy(self, *a, **k):
        orig(self, *a, **k)
        built.append(self)

    ex.SkillExecutor.__init__ = spy
    try:
        res = sg.SkillsGate(cfg).run_trial(seed)
    finally:
        ex.SkillExecutor.__init__ = orig
    return res, built


# ── the grammar and the argparse refusals ────────────────────────────────

def test_the_spec_grammar():
    assert sg.parse_beat_change('throw=6,apex=0.8') == dict(throw=6, apex=0.8)
    assert sg.parse_beat_change('throw=7,apex=1.0,dwell=0.3,apex2=0.9') == dict(
        throw=7, apex=1.0, dwell=0.3, apex2=0.9)
    assert sg._beat_request_kwargs(
        dict(throw=6, apex=1.0, dwell=0.3, apex2=0.9)) == dict(
        apex_m=1.0, apex_m_ball_b=0.9, dwell_s=0.3)


@pytest.mark.parametrize('spec', [
    'apex=0.8',                  # no throw
    'throw=6',                   # nothing to change
    'throw=2,apex=0.8',          # K < 3
    'throw=6,apex=0.8,apex=0.9',  # repeated key
    'throw=6,speed=1',           # unknown key
    'throw=6,apex=fast',         # not a number
    'throw=6,dwell=-0.3',        # not > 0
])
def test_a_bad_spec_is_refused_by_name(spec):
    with pytest.raises(ValueError, match='--beat-change'):
        sg.parse_beat_change(spec)


@pytest.mark.parametrize('argv, named', [
    (['--beat-change', 'throw=6,apex=0.8', '--learn'], '--learn'),
    (['--beat-change', 'throw=6,apex=0.8', '--reload'], '--reload'),
    (['--beat-change', 'throw=6,apex=0.8', '--nominate'], '--nominate'),
    (['--beat-change', 'throw=6,apex=0.8', '--one-ball', '--learn'],
     '--one-ball'),
    (['--beat-change', 'throw=2,apex=0.8'], 'K must be >= 3'),
    (['--beat-change', 'throw=20,apex=0.8'], 'at least 21 Throws'),
    (['--beat-change', 'throw=11,apex=0.8', '--n-throws', '11'],
     'at least 12 Throws'),
])
def test_the_argparse_refusals(argv, named, capsys):
    with pytest.raises(SystemExit) as e:
        sg.main(argv)
    assert e.value.code == 2
    assert named in capsys.readouterr().err


# ── the real runs, one seed each ─────────────────────────────────────────

@pytest.fixture(scope='module')
def apex_run():
    return _run('throw=6,apex=1.00')


def test_a_beat_change_run_binds_the_gates_checker(apex_run):
    _res, built = apex_run
    assert len(built) == 1 and callable(built[0].beat_checker)


def test_an_apex_change_flies_one_beat_later_and_passes_everything(apex_run):
    res, _ = apex_run
    b = res.beat
    assert (res.pump_clean and res.mirror_ok and res.flags_ok
            and res.all_installed and res.caught_all and res.no_drops
            and res.final_cup_ok), res
    assert b['request_ok'] and b['code'] == 'OK'
    # the request came after Throw 5; Throws 6 and 7 were already committed
    # (early-command window), so the first changeable Throw is 8
    assert b['from_throw'] == b['expected_from'] == 8
    assert b['change_applied'] and b['change_one_beat_later']
    assert b['release_instants_ok']
    meas = b['apex_measured_mm']
    # commanded 0.90 m then 1.00 m (the sim plant's gain is about 0.986)
    assert all(860 < m < 940 for m in meas[:8]) and all(m > 950 for m in meas[8:])
    assert res.passed and b['ok']


def test_a_dwell_change_moves_the_spans():
    res, _ = _run('throw=6,dwell=0.30')
    b = res.beat
    assert res.passed, b
    assert b['change_applied'] and b['release_instants_ok']
    assert b['dwell_spans_ok'] and 'transition_span_s' in b


def test_an_apex_and_dwell_change_together():
    res, _ = _run('throw=6,apex=1.00,dwell=0.30')
    b = res.beat
    assert res.passed, b
    assert b['change_applied'] and b['change_one_beat_later']
    assert b['dwell_spans_ok'] and b['apex_err_max_mm'] < 42.0


def test_a_change_with_no_transit_is_refused_by_name_and_the_attempt_flies_on():
    res, built = _run('throw=6,dwell=0.9')
    b = res.beat
    assert not b['request_ok'] and b['code'] == sk.BEAT_NO_TRANSIT
    assert b['change_refused_by_name'] and res.passed
    assert res.makes >= res.scheduled_catches and res.drops == 0


def test_mutation_the_executor_requesting_one_throw_later_fails_the_gate(
        monkeypatch):
    """Fail-before: the executor changes from Throw j + 1 while the gate's
    independent recompute says j -- ``change_applied`` and
    ``change_one_beat_later`` must both fail."""
    orig = ex.SkillExecutor.first_changeable_throw
    monkeypatch.setattr(
        ex.SkillExecutor, 'first_changeable_throw',
        lambda self, t: (None if orig(self, t) is None else orig(self, t) + 1))
    res, _ = _run('throw=6,apex=1.00')
    b = res.beat
    assert b['request_ok'] and b['from_throw'] == b['expected_from'] + 1
    assert not b['change_applied']
    assert not b['change_one_beat_later']
    assert not res.passed


def test_a_plain_run_reports_no_beat_fields():
    cfg = sg.SkillsGateConfig(seeds=(0,), n_throws=4,
                              report_path=None)
    res = sg.SkillsGate(cfg).run_trial(0)
    assert res.beat is None and 'beat' not in res.to_dict()


# ── the criteria, corrupted (fail-before, no sim) ────────────────────────

def _synthetic(spec_str: str, j: int = 8, gain: float = 0.986):
    spec = sg.parse_beat_change(spec_str)
    s0, s1 = sites.columns_sites(100.0, 0.0)
    pat = sk.Pattern(sites=(s0, s1), apex_m=0.9, dwell_s=0.27, n_throws=_N)
    sched0 = sk.compile_columns(pat, 10.0)
    kw = sg._beat_request_kwargs(spec)
    ref = sched0.with_events(sk.columns_change(sched0.source.events, j, **kw))
    rel = sg._release_times(ref)
    ev = ref.source.events
    releases = [dict(t=rel[n], ball=n % 2, z0_mm=100.0,
                     zmax_mm=100.0 + gain * 1000.0 * ev[n].apex_m)
                for n in range(_N)]
    captures = [(rel[n] - ev[n].dwell_s + 0.018, n % 2) for n in range(1, _N)]
    return spec, sched0, ref, dict(releases=releases, captures=captures)


def _score(spec, sched0, after, j, trace, from_throw=None):
    res = ex.BeatChange(True, 'OK', 'x', j if from_throw is None else from_throw)
    return sg._beat_criteria(spec=spec, sched0=sched0, result=res,
                             sched_after=after, expected_j=j, trace=trace)


def test_the_criteria_pass_on_a_faithful_trace():
    spec, s0, ref, tr = _synthetic('throw=6,apex=0.80,dwell=0.32')
    b = _score(spec, s0, ref, 8, tr)
    assert b['change_applied'] and b['change_one_beat_later']
    assert b['release_instants_ok'] and b['dwell_spans_ok'] and b['ok']


def test_mutation_change_one_beat_later_fails_when_a_throw_flew_the_old_apex():
    spec, s0, ref, tr = _synthetic('throw=6,apex=0.80')
    tr['releases'][8]['zmax_mm'] = 100.0 + 0.986 * 900.0     # Throw 8 old apex
    b = _score(spec, s0, ref, 8, tr)
    assert not b['change_one_beat_later'] and not b['ok']
    assert b['change_applied']


def test_mutation_change_one_beat_later_fails_when_the_prefix_moved():
    spec, s0, ref, tr = _synthetic('throw=6,dwell=0.32')
    tr['releases'][5]['t'] += 0.05
    assert not _score(spec, s0, ref, 8, tr)['change_one_beat_later']


def test_mutation_release_instants_fail_when_a_release_follows_the_old_beat():
    spec, s0, ref, tr = _synthetic('throw=6,dwell=0.32')
    old = sg._release_times(s0)
    tr['releases'][10]['t'] = old[10]
    b = _score(spec, s0, ref, 8, tr)
    assert not b['release_instants_ok'] and not b['ok']


def test_mutation_change_applied_fails_when_the_request_names_a_later_throw():
    spec, s0, ref, tr = _synthetic('throw=6,apex=0.80')
    assert not _score(spec, s0, ref, 8, tr, from_throw=9)['change_applied']
    # and when the Schedule was never replaced
    assert not _score(spec, s0, s0, 8, tr)['change_applied']


def test_mutation_dwell_spans_fail_when_the_new_dwell_is_not_flown():
    spec, s0, ref, tr = _synthetic('throw=6,dwell=0.32')
    tr['captures'] = [(t - 0.05 if i >= 9 else t, b)
                      for i, (t, b) in enumerate(tr['captures'])]
    assert not _score(spec, s0, ref, 8, tr)['dwell_spans_ok']


def test_mutation_the_refusal_criterion_fails_on_an_accepted_or_unnamed_change():
    spec = sg.parse_beat_change('throw=6,dwell=0.9')
    s0, s1 = sites.columns_sites(100.0, 0.0)
    sched0 = sk.compile_columns(
        sk.Pattern(sites=(s0, s1), apex_m=0.9, dwell_s=0.27, n_throws=_N), 10.0)
    tr = dict(releases=[], captures=[])

    def score(res, after):
        return sg._beat_criteria(spec=spec, sched0=sched0, result=res,
                                 sched_after=after, expected_j=8, trace=tr)

    named = ex.BeatChange(False, sk.BEAT_NO_TRANSIT, 'x', 8)
    assert score(named, sched0)['change_refused_by_name']
    # refused, but the Schedule object was replaced
    assert not score(named, object())['change_refused_by_name']
    # refused with a code that is not one of schedule.py's
    assert not score(ex.BeatChange(False, 'WHATEVER', 'x', 8),
                     sched0)['change_refused_by_name']
    assert not score(ex.BeatChange(False, ex.BEAT_TOO_LATE, 'x', None),
                     sched0)['change_refused_by_name']
    # the change was ACCEPTED: a refusal run must not pass
    acc = ex.BeatChange(True, 'OK', 'x', 8)
    out = score(acc, sched0)
    assert not out.get('change_refused_by_name', False) and not out['ok']
