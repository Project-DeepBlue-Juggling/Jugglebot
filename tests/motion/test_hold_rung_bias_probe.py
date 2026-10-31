"""CI guard for ``tools/probes/hold_rung_bias.py`` (pre-release-hold ladder read,
map JB-102, runbook ``tests/hardware/session_oval_rung1.md`` § 11).

The probe attributes throws to goals (launch log) and holds (``segment config``
lines) by itself and prints the next operator action, so its attribution, the
pre-registered verdict and the NEXT branch are tested on synthetic logs; the real
2026-10-30 launch log is a smoke test when it exists on the box.
"""

from __future__ import annotations

import importlib.util
import math
import os
import random

import pytest

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_PROBE = os.path.join(_REPO, 'tools', 'probes', 'hold_rung_bias.py')
_REAL_LOG = os.path.expanduser(
    '~/.ros/log/2026-10-30-16-17-11-746307-jetson-190841/launch.log')
_REAL_MEM = os.path.join(_REPO, 'temp', 'learn', 'jugglebot', 'memory.csv')
T0 = 1_800_000_000.0


@pytest.fixture(scope='module')
def probe():
    spec = importlib.util.spec_from_file_location('hold_rung_bias', _PROBE)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _line(t, node, level, msg):
    return '%.7f [%s-1] [%s] [%.9f] [%s]: %s' % (t, node, level, t, node, msg)


def _hold(t, h):
    return _line(t, 'trajectory_node', 'INFO',
                 'segment config: pre_release_hold_s=%.3f s, post_release_hold_s=0.050 s, '
                 'planner_mode default=pose (live set; applies from the next install)' % h)


def _goal(t, n, caught, pattern='oval_1ball', apex=0.95, end='done', logged=None):
    start = _line(t, 'skill_node', 'INFO', '%s started: %d throws at apex %.2f m (P1 -> P2) '
                  '· memory 100 rows' % (pattern, n, apex))
    if logged:  # (dwell_s, sep_mm): the new start-line fields
        start += ' · mocap frame offset 0.4 mm · dwell %.2f s · sep %g mm' % logged
    tail = {'done': 'done: %d/%d caught · apex 0.94–0.96 m' % (caught, n),
            'stop': 'stopped by the operator: %d/%d caught · apex 0.95–0.96 m' % (caught, n),
            }[end]
    return start, _line(t + 2.0 * n + 5, 'skill_node', 'INFO',
                        '%s %s' % (pattern, tail))


def _rows(t, n, mean_short, sd=6.0, seed=0, dy=0.0):
    """Memory rows (x0=-75 mm, u0=+5 mm) landing `mean_short` mm short on average."""
    rng = random.Random(seed)
    out = []
    for i in range(n):
        short = rng.gauss(mean_short, sd)
        out.append((-0.075, 0.005, 0.005 - short / 1000.0, 0.0, 0.0 + dy / 1000.0,
                    0.95, 0.95, T0 + t + 3 + 2.0 * i, True))
    return out


def _write(tmp_path, lines, rows):
    log = tmp_path / 'launch.log'
    log.write_text('\n'.join(lines) + '\n')
    mem = tmp_path / 'memory.csv'
    with open(mem, 'w') as fh:
        fh.write('x0,x1,x2,x3,u0,u1,u2_apex_m,y0,y1,y2_apex_m,t_abs_s,ball_id,caught\n')
        for x0, _u0, y0, _a, y1, u2, y2, t, c in rows:
            fh.write('%r,0,0,0,%r,0,%r,%r,%r,%r,%r,0,%s\n' % (x0, 0.005, u2, y0, y1, y2, t, c))
    return str(log), str(mem)


def _scenario(tmp_path, rung_short, n_rung=30, rung_hold=0.050):
    g1a, g1b = _goal(T0 + 10, 30, 30)
    # the goal REQUESTS 30 throws; an early stop leaves fewer rows
    g2a, g2b = _goal(T0 + 200, 30, n_rung, end='stop')
    lines = [_hold(T0 + 1, 0.100), g1a, g1b, _hold(T0 + 150, rung_hold), g2a, g2b]
    rows = _rows(10, 30, 3.0, seed=1) + _rows(200, n_rung, rung_short, seed=2)
    return _write(tmp_path, lines, rows)


def _run(probe, tmp_path, capsys, log, mem, extra=()):
    rc = probe.main(['--launch-log', log, '--memory', mem] + list(extra))
    assert rc == 0
    return capsys.readouterr().out


def test_attribution_and_stats(probe, tmp_path):
    log, mem = _scenario(tmp_path, 28.0)
    holds, goals = probe.parse_log(log)
    assert [h for _t, h in holds] == [0.100, 0.050]
    probe.attribute(goals, probe.load_memory(mem))
    assert [g['hold'] for g in goals] == [0.100, 0.050]
    assert [g['role'] for g in goals] == ['control', 'rung']
    assert [g['n'] for g in goals] == [30, 30]
    assert goals[0]['dwell'] == '0.45*'
    assert abs(sum(goals[0]['dx']) / 30 - 3.0) < 4
    assert abs(sum(goals[1]['dx']) / 30 - 28.0) < 4
    # sign convention: positive = short
    r = {'x0': '-0.075', 'u0': '0.005', 'y0': '-0.005', 'y1': '0', 'u1': '0',
         'y2_apex_m': '0.95', 'u2_apex_m': '0.95', 'caught': 'True'}
    assert probe.row_vals(r)[0] == pytest.approx(10.0)
    r.update(x0='0.075', u0='-0.005', y0='0.005')
    assert probe.row_vals(r)[0] == pytest.approx(10.0)


def test_confirmed_goes_to_fallback_0075(probe, tmp_path, capsys):
    log, mem = _scenario(tmp_path, 28.0)
    out = _run(probe, tmp_path, capsys, log, mem)
    _, goals = probe.parse_log(log)
    probe.attribute(goals, probe.load_memory(mem))
    ctl, rungs, cmp_, nxt = probe.decide(goals)
    c = cmp_[0.050]
    assert c['verdict'] == 'confirmed'
    assert 18 < c['delta'] < 32
    d = probe.delta_eff(c['delta'])
    assert 5 < d < 20
    pred = probe.hold0_prediction(c['delta'])
    assert pred > 55                      # the rule's fallback branch
    assert 'CONFIRMED' in out
    assert nxt.startswith('NEXT: ros2 param set /trajectory_node pre_release_hold_s 0.075')
    assert 'stops the ladder' in nxt
    assert out.strip().endswith(nxt)


def test_delta_eff_interpolation(probe):
    assert probe.delta_eff(44.5) == pytest.approx(0.0)
    assert probe.delta_eff(31.5) == pytest.approx(10.0)
    assert probe.delta_eff(26.9) == pytest.approx(15.0)
    assert probe.delta_eff(11.0) == pytest.approx(40.0)
    assert probe.hold0_prediction(26.9) == pytest.approx((102.5 + 81.3) / 2)
    assert probe.hold0_prediction(11.0) == pytest.approx(46.8)   # 0.025 branch


def test_confirmed_mid_branch_goes_to_0025(probe, tmp_path, capsys):
    # Delta ~ 11 mm over the control: delta_eff ~ 40 ms -> hold-0 prediction ~ 47 -> 0.025
    g1a, g1b = _goal(T0 + 10, 30, 30)
    g2a, g2b = _goal(T0 + 200, 30, 30)
    lines = [_hold(T0 + 1, 0.100), g1a, g1b, _hold(T0 + 150, 0.050), g2a, g2b]
    rows = _rows(10, 30, 3.0, sd=3.0, seed=1) + _rows(200, 30, 14.5, sd=3.0, seed=2)
    log, mem = _write(tmp_path, lines, rows)
    out = _run(probe, tmp_path, capsys, log, mem)
    assert 'CONFIRMED' in out
    assert 'NEXT: ros2 param set /trajectory_node pre_release_hold_s 0.025' in out


def test_no_effect_sets_hold_zero(probe, tmp_path, capsys):
    log, mem = _scenario(tmp_path, 5.0)
    out = _run(probe, tmp_path, capsys, log, mem)
    assert 'NO EFFECT' in out
    assert out.strip().splitlines()[-1].startswith(
        'NEXT: ros2 param set /trajectory_node pre_release_hold_s 0.0 ;')


def test_inconclusive_small_n(probe, tmp_path, capsys):
    log, mem = _scenario(tmp_path, 28.0, n_rung=12)
    out = _run(probe, tmp_path, capsys, log, mem)
    assert 'too few rows' in out or 'INCONCLUSIVE' in out or 'n=12' in out
    last = out.strip().splitlines()[-1]
    assert last.startswith('NEXT: ros2 param set /trajectory_node pre_release_hold_s 0.050')


def test_ladder_done_adopts_shortest_passing(probe, tmp_path, capsys):
    # control, 0.050 confirmed (fallback 0.075), 0.075 flown 30/30 with small bias.
    g = [_goal(T0 + 10, 30, 30), _goal(T0 + 200, 30, 30), _goal(T0 + 400, 30, 30)]
    lines = [_hold(T0 + 1, 0.100), g[0][0], g[0][1], _hold(T0 + 150, 0.050), g[1][0],
             g[1][1], _hold(T0 + 350, 0.075), g[2][0], g[2][1]]
    rows = (_rows(10, 30, 3.0, seed=1) + _rows(200, 30, 28.0, seed=2)
            + _rows(400, 30, 10.0, seed=3))
    log, mem = _write(tmp_path, lines, rows)
    out = _run(probe, tmp_path, capsys, log, mem)
    assert out.strip().splitlines()[-1].startswith('NEXT: ladder done - adopt hold 0.075')


def test_ladder_done_refuses_a_long_rung_and_names_direction(probe, tmp_path, capsys):
    # 0.050 lands 39.5 mm LONG (Delta < 0): |Delta| > 20, so it must NOT be adopted
    # (the signed rule adopted it on 2026-10-31); 0.075 lands 10 mm SHORT, adopted,
    # and the verdict names the direction.
    g = [_goal(T0 + 10, 30, 30), _goal(T0 + 200, 30, 30), _goal(T0 + 400, 30, 30)]
    lines = [_hold(T0 + 1, 0.100), g[0][0], g[0][1], _hold(T0 + 150, 0.050), g[1][0],
             g[1][1], _hold(T0 + 350, 0.075), g[2][0], g[2][1]]
    rows = (_rows(10, 30, 3.0, seed=1) + _rows(200, 30, -36.5, seed=2)
            + _rows(400, 30, 10.0, seed=3))
    log, mem = _write(tmp_path, lines, rows)
    out = _run(probe, tmp_path, capsys, log, mem)
    last = out.strip().splitlines()[-1]
    assert last.startswith('NEXT: ladder done - adopt hold 0.075')
    assert 'SHORT' in last
    assert probe.adopt.__doc__ and 'LONG' in probe.adopt.__doc__


def test_json_and_clean_errors(probe, tmp_path, capsys):
    log, mem = _scenario(tmp_path, 28.0)
    js = tmp_path / 'sub' / 'out.json'
    _run(probe, tmp_path, capsys, log, mem, ['--json', str(js)])
    assert js.exists()
    with pytest.raises(SystemExit) as e:
        probe.main(['--launch-log', str(tmp_path / 'nope.log'), '--memory', mem])
    assert e.value.code == 2
    with pytest.raises(SystemExit):
        probe.main(['--launch-log', log, '--memory', str(tmp_path / 'nope.csv')])


@pytest.mark.skipif(not (os.path.isfile(_REAL_LOG) and os.path.isfile(_REAL_MEM)),
                    reason='2026-10-30 launch log / memory not on this box')
def test_real_log_finds_the_five_oval_goals(probe):
    _holds, goals = probe.parse_log(_REAL_LOG)
    ovals = [g for g in goals if g['pattern'] == 'oval_1ball']
    assert len(ovals) == 5
    assert [(g['end_caught'], g['end_total']) for g in ovals] == [
        (50, 50), (5, 6), (6, 6), (50, 50), (40, 40)]
    assert [g['reload'] for g in ovals] == [False, False, True, False, False]
    assert all(g['hold'] == 0.100 for g in goals)
    assert all(not math.isnan(float(g['start'])) for g in goals)


def test_logged_dwell_is_used_not_inferred(tmp_path, probe):
    """A start line carrying ` · dwell X s · sep N mm` is parsed as logged:
    no `*` marker, and a 0.50 apex goal is not mislabelled by goal order."""
    lines = [_hold(T0 + 1, 0.100)]
    lines += list(_goal(T0 + 10, 30, 30, logged=(0.45, 150)))
    lines += list(_goal(T0 + 200, 30, 30, apex=1.00, logged=(0.50, 150)))
    lines += list(_goal(T0 + 400, 30, 30, apex=1.05, logged=(0.45, 150)))
    log, _mem = _write(tmp_path, lines, [])
    _holds, goals = probe.parse_log(log)
    assert [g['dwell_logged'] for g in goals] == [0.45, 0.50, 0.45]
    assert [g['sep_mm'] for g in goals] == [150.0] * 3

    probe.attribute(goals, [])
    assert [g['dwell'] for g in goals] == ['0.45', '0.50', '0.45']
