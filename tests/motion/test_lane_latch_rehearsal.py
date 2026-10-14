"""Pure-logic tests for tools/probes/lane_latch_rehearsal.py (no rclpy, no processes).

The probe is the sitting-8 dress rehearsal of lane-latch auto-recovery; its
verdict must report EVERY expectation at once, and a rehearsal that never
latched must FAIL (not pass vacuously). The ROS/OS layer is blind-tested on
the loaded Jetson by the --no-freeze pass (runsheet row)."""
from __future__ import annotations

import importlib.util
import os

import pytest

_P = os.path.join(os.path.dirname(__file__), '..', '..', 'tools', 'probes',
                  'lane_latch_rehearsal.py')
_spec = importlib.util.spec_from_file_location('lane_latch_rehearsal', _P)
m = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(m)


def S(t, bit=0, rec='idle', stage='none', dev=0.01, fault='NONE', ref=0):
    return dict(t=t, bit15=bit, lane_recovery=rec, stage=stage, live_dev=dev,
                fault_state=fault, sched_refused=ref)


RUN = dict(no_freeze=False, move_accepted=True, move_msg='OK', freeze_actual_ms=102.0,
           home_accepted=True, home_msg='ok')


def happy():
    s = [S(0.0), S(0.1), S(0.2, bit=1, ref=1), S(0.3, bit=1, ref=1),
         S(0.5, 1, 'in_progress', 'converge'), S(0.9, 1, 'in_progress', 'rest'),
         S(1.4, 1, 'in_progress', 'disarm'), S(1.9, 1, 'in_progress', 'arm'),
         S(2.3, 0, 'in_progress', 'confirm'), S(2.6, 0, 'done', 'confirm', dev=0.04),
         S(3.0, 0, 'done', 'confirm')]
    return s


def by(rows):
    return {r['expectation'].split(' (')[0]: r for r in rows}


def test_happy_path_all_pass():
    rows = m.build_verdict(happy(), RUN)
    assert all(r['status'] in ('PASS', 'INFO') for r in rows), m.format_table(rows)
    assert m.exit_code(rows) == 0


def test_failed_recovery_reported_with_stage_and_fallback():
    s = happy()[:8] + [S(2.5, 1, 'failed', 'arm')]
    rows = m.build_verdict(s, RUN)
    done = [r for r in rows if r['expectation'].startswith('recovery done')][0]
    assert done['status'] == 'FAIL' and 'failed at stage arm' in done['evidence']
    assert 'DEACTIVATE' in done['evidence']
    end = [r for r in rows if r['expectation'].startswith('bit 15 cleared')][0]
    assert end['status'] == 'FAIL'
    assert m.exit_code(rows) == 1


def test_fault_during_is_a_fail_row_with_real_fault_text():
    s = happy()
    s[6] = S(1.4, 1, 'in_progress', 'disarm', fault='SETPOINT_STALE')
    rows = m.build_verdict(s, RUN)
    f = [r for r in rows if r['expectation'].startswith('fault_state')][0]
    assert f['status'] == 'FAIL'
    assert 'SETPOINT_STALE' in f['evidence']
    assert 'run /recover (DEACTIVATE then ACTIVATE), then re-run' in f['evidence']
    assert m.exit_code(rows) == 1


def test_no_latch_fails_and_skips_recovery_rows():
    s = [S(i * 0.1) for i in range(30)]
    rows = m.build_verdict(s, RUN)
    r0 = [r for r in rows if r['expectation'].startswith('bit 15 (sched')][0]
    assert r0['status'] == 'FAIL'
    assert 'lengthen --freeze-ms or --dx-mm' in r0['evidence']
    assert any(r['status'] == 'SKIPPED' for r in rows)
    assert m.exit_code(rows) == 1


def test_stage_order_violation_and_live_deviation_and_slow_start():
    s = happy()
    s[5], s[6] = S(0.9, 1, 'in_progress', 'disarm'), S(1.4, 1, 'in_progress', 'rest')
    rows = m.build_verdict(s, RUN)
    assert [r for r in rows if r['expectation'].startswith('stages')][0]['status'] == 'FAIL'
    s = happy()
    s[8] = S(2.3, 0, 'in_progress', 'confirm', dev=0.15)
    rows = m.build_verdict(s, RUN)
    assert [r for r in rows if r['expectation'].startswith('live_dev')][0]['status'] == 'FAIL'
    s = happy()
    for x in s[4:]:
        x['t'] += 1.5                      # in_progress 1.8 s after the latch
    rows = m.build_verdict(s, RUN)
    assert [r for r in rows if r['expectation'].startswith('recovery in_progress')][0]['status'] == 'FAIL'


def test_go_home_refused_and_move_refused_rows():
    run = dict(RUN, home_accepted=False, home_msg='lane latched')
    rows = m.build_verdict(happy(), run)
    assert [r for r in rows if r['expectation'].startswith('go_home')][0]['status'] == 'FAIL'
    rows = m.build_verdict(happy(), dict(RUN, move_accepted=False, move_msg='TOO_FAST'))
    assert rows[0]['status'] == 'FAIL' and m.exit_code(rows) == 1


def test_no_freeze_pass_and_spurious_latch():
    run = dict(RUN, no_freeze=True, freeze_actual_ms=None)
    clean = [S(i * 0.1) for i in range(20)]
    rows = m.build_verdict(clean, run)
    assert m.exit_code(rows) == 0
    assert any(r['status'] == 'SKIPPED' for r in rows)
    rows = m.build_verdict(happy(), run)
    assert [r for r in rows if r['expectation'].startswith('no spurious')][0]['status'] == 'FAIL'


def test_preflight_lists_every_failure_at_once():
    args = m.parse_args([])
    snap = dict(link_age_s=3.0, bridge_link='LOST', fault_state='SETPOINT_STALE',
                sched_hold_latched='1', lane_recovery='failed', mode='STANDBY',
                traj_pids=[11, 12], have_skill_node=False, pose_known=False)
    bad = m.preflight(snap, args, [0.8] * 6)
    assert len(bad) == 9, bad
    ok = dict(snap, link_age_s=0.1, bridge_link='UP', fault_state='NONE',
              sched_hold_latched='0', lane_recovery='idle', mode='TRAJECTORY',
              traj_pids=[11], have_skill_node=True, pose_known=True)
    assert m.preflight(ok, args, [0.8] * 6) == []
    assert len(m.preflight(ok, args, [0.2] * 6)) == 1            # too gentle
    assert m.preflight(ok, m.parse_args(['--no-freeze']), [0.2] * 6) == []


def test_freeze_150_refused_and_defaults_ok():
    assert m.check_args(m.parse_args([])) == []
    assert m.check_args(m.parse_args(['--freeze-ms', '150']))
    assert not m.check_args(m.parse_args(['--freeze-ms', '149']))
    assert m.check_args(m.parse_args(['--freeze-ms', '100', '--at-ms', '1100']))
    assert m.main(['--freeze-ms', '250']) == 2


def test_default_move_exceeds_resume_tolerance_on_some_leg():
    pytest.importorskip('numpy')
    a = m.parse_args([])
    assert (a.dx_mm, a.move_s, a.at_ms) == (50.0, 1.2, 450.0)
    peak = m.leg_peak_rps(a.dx_mm, a.move_s)
    assert len(peak) == 6
    assert max(peak) > m.RESUME_TOL_VEL_LEG_RPS
    assert max(peak) == pytest.approx(0.40, abs=0.03)    # measured 2026-10-14
    assert max(m.leg_peak_rps(5.0, 1.2)) < m.RESUME_TOL_VEL_LEG_RPS


# Samples shaped like the 2026-10-14 21:58:43 live record (50 mm / 1.2 s, 100 ms freeze).
TONIGHT = [
    S(2.092, 0, 'idle', 'none', 0.0114, ref=0), S(2.192, 0, 'idle', 'none', 0.0146, ref=0),
    S(2.292, 0, 'idle', 'none', 0.0344, ref=0), S(2.392, 1, 'idle', 'none', 0.0596, ref=3),
    S(2.492, 1, 'idle', 'none', 0.1008, ref=7), S(2.593, 1, 'idle', 'none', 0.1288, ref=10),
    S(2.695, 1, 'in_progress', 'converge', 0.1577, ref=14),
    S(2.795, 1, 'in_progress', 'converge', 0.1412, ref=18),
    S(2.891, 1, 'in_progress', 'disarm', 0.0006, ref=23),
    S(2.992, 1, 'in_progress', 'disarm', 0.0011, ref=24),
    S(3.091, 1, 'in_progress', 'disarm', 0.0013, ref=24),
    S(3.197, 1, 'in_progress', 'confirm', 0.0016, ref=24),
    S(3.295, 0, 'in_progress', 'confirm', 0.0020, ref=24),
    S(3.393, 0, 'done', 'confirm', 0.0013, ref=24),
    S(3.494, 0, 'done', 'confirm', 0.0013, ref=24),
]


def test_tonights_record_all_pass_or_info():
    rows = m.build_verdict(TONIGHT, RUN)
    assert all(r['status'] in ('PASS', 'INFO') for r in rows), m.format_table(rows)
    assert m.exit_code(rows) == 0
    pk = [r for r in rows if r['expectation'].startswith('peak live_deviation')][0]
    assert pk['status'] == 'INFO'
    assert 'max 0.1577 rev at t=2.69' in pk['evidence']
    assert 'bridge escape 0.5, firmware trip 1.0' in pk['evidence']


def test_peak_while_latched_fails_at_bridge_escape():
    s = [dict(x) for x in TONIGHT]
    s[6]['live_dev'] = 0.5
    rows = m.build_verdict(s, RUN)
    pk = [r for r in rows if r['expectation'].startswith('peak live_deviation')][0]
    assert pk['status'] == 'FAIL'
    s[6]['live_dev'] = 0.49
    rows = m.build_verdict(s, RUN)
    assert [r for r in rows if r['expectation'].startswith('peak live_deviation')][0]['status'] == 'INFO'


def test_recovery_window_deviation_fails_only_inside_the_window():
    s = [dict(x) for x in TONIGHT]
    s[11]['live_dev'] = 0.12                 # confirm stage, inside converge-done -> DONE
    rows = m.build_verdict(s, RUN)
    r = [r for r in rows if r['expectation'].startswith('live_deviation <=')][0]
    assert r['status'] == 'FAIL' and '0.1200' in r['evidence']
    s = [dict(x) for x in TONIGHT]
    s[7]['live_dev'] = 0.9                   # still converging: the latched-peak row owns it
    rows = m.build_verdict(s, RUN)
    assert [r for r in rows if r['expectation'].startswith('live_deviation <=')][0]['status'] == 'PASS'


def test_no_latch_skips_recovery_window_row():
    rows = m.build_verdict([S(i * 0.1) for i in range(30)], RUN)
    by_ = {r['expectation'].split(' (')[0]: r for r in rows}
    assert [r for r in rows if r['expectation'].startswith('live_deviation <=')][0]['status'] == 'SKIPPED'
    assert [r for r in rows if r['expectation'].startswith('peak live_deviation')][0]['status'] == 'SKIPPED'
    assert by_ and m.exit_code(rows) == 1


def test_refused_move_skips_everything_with_reason_and_hint():
    msg = ('TOO_FAST TOO_FAST: requested duration 0.500s < minimal feasible 0.822s '
           'for this move at the current limits (planned 0.00 s)')
    run = dict(RUN, move_accepted=False, move_msg=msg, freeze_actual_ms=None,
               home_accepted=None)
    rows = m.build_verdict([S(i * 0.1) for i in range(20)], run)
    assert rows[0]['status'] == 'FAIL'
    assert 're-run with --move-s >= 1.02' in rows[0]['evidence']
    rest = {r['expectation']: r for r in rows[1:]}
    for k in ('freeze bounded (CONT sent)', 'bit 15 (sched_hold_latched) rose',
              'bit 15 cleared at the end', m.LIVE_DEV_PEAK_NAME,
              m.LIVE_DEV_RECOVERY_NAME):
        assert rest[k]['status'] == 'SKIPPED' and rest[k]['evidence'] == 'move refused'
    assert not any('did not latch' in r['evidence'] for r in rows)
    assert not any(r['status'] == 'FAIL' for r in rows[1:])
    assert m.exit_code(rows) == 1


def test_no_freeze_observation_pass_checks_deviation():
    run = dict(RUN, no_freeze=True, freeze_actual_ms=None)
    obs = lambda rows: [r for r in rows if r['expectation'].startswith('live_deviation <=')]
    clean = [S(i * 0.1, dev=0.0166 if i == 30 else 0.005) for i in range(56)]
    rows = m.build_verdict(clean, run)
    assert len(obs(rows)) == 1 and obs(rows)[0]['status'] == 'PASS'
    assert obs(rows)[0]['evidence'] == 'max 0.0166 rev over 56 samples'
    assert not any(r['expectation'].startswith('peak live_deviation') for r in rows)
    assert m.exit_code(rows) == 0
    bad = [dict(x) for x in clean]
    bad[10]['live_dev'] = 0.2
    rows = m.build_verdict(bad, run)
    assert obs(rows)[0]['status'] == 'FAIL' and m.exit_code(rows) == 1
    rows = m.build_verdict([S(0.0, dev=None)], run)
    assert obs(rows)[0]['status'] == 'FAIL' and 'no live_deviation' in obs(rows)[0]['evidence']


def test_peak_row_sees_pre_latch_spike():
    s = [dict(x) for x in TONIGHT]
    first = next(i for i, x in enumerate(s) if x['bit15'])
    assert first > 0
    s[first - 1]['live_dev'] = 0.6           # spike BEFORE bit 15 rose
    rows = m.build_verdict(s, RUN)
    pk = [r for r in rows if r['expectation'].startswith('peak live_deviation')][0]
    assert pk['status'] == 'FAIL' and '0.6000' in pk['evidence']
