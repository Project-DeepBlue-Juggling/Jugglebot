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
    assert m.check_args(m.parse_args(['--freeze-ms', '100', '--at-ms', '450']))
    assert m.main(['--freeze-ms', '250']) == 2


def test_default_move_exceeds_resume_tolerance_on_some_leg():
    pytest.importorskip('numpy')
    peak = m.leg_peak_rps(40.0, 0.5)
    assert len(peak) == 6
    assert max(peak) > m.RESUME_TOL_VEL_LEG_RPS
    assert max(peak) == pytest.approx(0.77, abs=0.05)    # measured 2026-10-13
    assert max(m.leg_peak_rps(5.0, 0.5)) < m.RESUME_TOL_VEL_LEG_RPS
