"""BUG-129 launch-series parser and runner (tools/probes/bug129_series_report.py,
bug129_launch_series.sh): synthetic log text, dumps and bag rows; the runner's dry run.
No ROS, no hardware, no real bag."""
from __future__ import annotations

import importlib.util
import pathlib
import re
import subprocess

REPO = pathlib.Path(__file__).resolve().parents[2]


def _load():
    spec = importlib.util.spec_from_file_location(
        'bug129_series_report', REPO / 'tools' / 'probes' / 'bug129_series_report.py')
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


R = _load()
ESC = '\x1b[2m'
RST = '\x1b[0m'

HEAD_A = (
    f'{ESC}14:30:01.000{RST} launch       shm clean: removed 42 stale Fast DDS files from /dev/shm\n'
    f'{ESC}14:30:01.001{RST} launch       bridge shared memory on: teensy_bridge_node uses the default Fast DDS transports\n'
    f'14:30:01.002 launch       recording bag -> /home/x/rosbags/2026-11-25_14-30-01\n'
    f'14:30:04.000 teensy       BRIDGE_FW_CHECK: OK - can-bridge v49 [install_skew=0]\n'
    f'14:30:04.100 teensy       stall watchdog on: timeout 150 ms, stacks to /x (BUG-129)\n'
    f'14:30:04.200 teensy       ERROR ODrive error on leg 3: active=[DC_BUS_UNDER_VOLTAGE]\n')
STALL = (f'{ESC}14:31:10.430{RST} teensy       WARN  stall watchdog: teensy_bridge_node stalled '
         '(largest gap 313.2 ms, timeout 150 ms); every Python thread\'s stack, taken mid-stall: '
         '/x/stall_teensy_bridge_node_2026-11-25T14-31-10_77_1.txt\n')
SOFT = ('14:32:00.000 teensy       WARN  stall watchdog: teensy_bridge_node stalled (largest gap 180.0 ms, '
        'timeout 150 ms); every Python thread\'s stack, taken mid-stall: (not written: cap of 5 reports reached)\n')

DUMP_P = '''# teensy_bridge_node stall #1
Thread 0x1 (most recent call first):
  File "/opt/ros/foxy/lib/python3.8/site-packages/rclpy/publisher.py", line 70 in publish
  File "/x/teensy_bridge_node.py", line 4712 in _publish_robot_state
Thread 0x2 (most recent call first):
  File "/usr/lib/python3.8/concurrent/futures/thread.py", line 78 in _worker
'''
DUMP_K = '''Thread 0x1 (most recent call first):
  File "/x/stall_watchdog.py", line 136 in _arm
  File "/x/stall_watchdog.py", line 178 in _tick
Thread 0x2 (most recent call first):
  File "/x/client.py", line 636 in recvfrom
  File "/x/client.py", line 640 in _drain_socket
'''
DUMP_O = '''Thread 0x1 (most recent call first):
  File "/usr/lib/python3.8/threading.py", line 890 in _bootstrap
'''


def _rows(gap_at=None, gap_ms=340.0, stamp_follows=True, n=400, t0_ns=1_700_000_000_000_000_000):
    """100 Hz /robot_state rows; one gap of gap_ms before index gap_at. stamp_follows=False
    makes it a recorder-side gap (header stamps stay 10 ms apart)."""
    rows, t, s = [], t0_ns, 1000.0
    for i in range(n):
        if gap_at is not None and i == gap_at:
            t += int(gap_ms * 1e6)
            s += gap_ms / 1e3 if stamp_follows else 0.010
        else:
            t += 10_000_000
            s += 0.010
        rows.append((t, s))
    return rows


def _meta(arm, maps, start):
    return {'fastrtps_maps': str(maps), 'start_epoch': str(start)}


def test_parse_log_stall_warn_with_ansi_and_expected_errors_ignored():
    p = R.parse_log(HEAD_A + STALL + SOFT)
    assert [s['gap_ms'] for s in p['stalls']] == [313.2, 180.0]
    assert p['stalls'][0]['path'].endswith('_77_1.txt') and p['stalls'][0]['hms'] == '14:31:10.430'
    assert p['stalls'][1]['path'].startswith('(not written')
    assert p['bag'] == '/home/x/rosbags/2026-11-25_14-30-01'
    assert p['errors_other'] == 0                      # the ODrive undervoltage ERROR is expected
    assert p['watchdog_on'] and p['shm_clean'] and p['shm_on'] and p['fw_ok'] and not p['shm_off']


def test_clean_launch_is_valid_with_no_stalls():
    start = 1_700_000_000
    rec = R.analyse_launch('A_01', 'A', HEAD_A, _meta('A', 12, start), _rows(), {})
    assert rec['valid'] and rec['invalid'] == []
    assert rec['stalls'] == [] and rec['p_stalls'] == 0 and rec['n_gaps_50'] == 0


def test_invalid_trial_reasons():
    # arm B whose log says shm ON, with fastrtps mapped, and an empty bag
    rec = R.analyse_launch('B_02', 'B', HEAD_A.replace('BRIDGE_FW_CHECK: OK', 'x'),
                           _meta('B', 3, 0), [], {})
    assert not rec['valid']
    why = ' | '.join(rec['invalid'])
    assert 'shared memory off' in why and 'manipulation check failed (B' in why
    assert 'BRIDGE_FW_CHECK' in why and 'empty /robot_state' in why


def test_manipulation_check_arm_a_needs_maps():
    rec = R.analyse_launch('A_01', 'A', HEAD_A, _meta('A', 0, 1_700_000_000), _rows(), {})
    assert not rec['valid'] and any('A has no fastrtps maps' in w for w in rec['invalid'])


def test_confirmed_P_stall_vs_recorder_side_gap():
    # bag log time of the gap end: row 200 -> t0 + 200*10 ms + 340 ms
    t0 = 1_700_000_000
    rows = _rows(gap_at=200, t0_ns=t0 * 10**9)
    warn_s = t0 + 200 * 0.010 + 0.34 + 0.2     # WARN is reported ~0.2 s after the gap ends
    hh = __import__('time').localtime(warn_s)
    hms = '%02d:%02d:%06.3f' % (hh.tm_hour, hh.tm_min, hh.tm_sec + (warn_s % 1))
    log = HEAD_A + STALL.replace('14:31:10.430', hms)
    path = '/x/stall_teensy_bridge_node_2026-11-25T14-31-10_77_1.txt'
    rec = R.analyse_launch('A_01', 'A', log, _meta('A', 9, t0), rows, {path: DUMP_P})
    assert rec['p_stalls'] == 1 and rec['stalls'][0]['confirmed'] and rec['stalls'][0]['shape'] == 'P'
    assert rec['max_gap_ms'] >= 330 and rec['n_gaps_50'] == 1
    # same WARN, but the stamps stayed 10 ms apart: the recorder dropped, not the bridge
    rows_rec = _rows(gap_at=200, stamp_follows=False, t0_ns=t0 * 10**9)
    rec2 = R.analyse_launch('A_01', 'A', log, _meta('A', 9, t0), rows_rec, {path: DUMP_P})
    assert rec2['p_stalls'] == 0 and rec2['recorder_gaps'] == 1
    assert rec2['stalls'][0]['confirmed'] is False
    # a K-shaped dump never counts as P
    rec3 = R.analyse_launch('A_01', 'A', log, _meta('A', 9, t0), rows, {path: DUMP_K})
    assert rec3['p_stalls'] == 0 and rec3['k_stalls'] == 1


def test_soft_warn_150_250_listed_separately():
    rec = R.analyse_launch('A_01', 'A', HEAD_A + SOFT, _meta('A', 9, 1_700_000_000), _rows(), {})
    assert len(rec['soft_warns']) == 1 and rec['stalls'] == [] and rec['p_stalls'] == 0


def test_classify_dump_shapes():
    assert R.classify_dump(DUMP_P) == 'P'
    assert R.classify_dump(DUMP_K) == 'K'
    assert R.classify_dump(DUMP_O) == 'O'
    assert R.classify_dump('') == 'O'


def test_verdict_rules():
    assert 'REFUTES' in R.verdict(12, 12, 3, 1)               # one P in B is decisive
    assert 'SUPPORTS' in R.verdict(15, 15, 4, 0) and 'significant (' in R.verdict(15, 15, 4, 0)
    assert 'NOT yet significant' in R.verdict(12, 12, 2, 0)
    assert 'POSITIVE CONTROL FAILED' in R.verdict(10, 10, 0, 0)
    assert 'rule of three' in R.verdict(15, 15, 0, 0)
    assert 'too few' in R.verdict(5, 5, 0, 0)
    assert 'INCONCLUSIVE' in R.verdict(12, 12, 1, 0)
    assert abs(R.fisher_one_sided(4, 15, 0, 15) - 0.0539) < 0.005     # C(15,4)/C(30,4)... one-sided


def test_render_report_has_row_per_launch_and_verdict():
    recs = [R.analyse_launch('A_01', 'A', HEAD_A, _meta('A', 9, 1_700_000_000), _rows(), {}),
            R.analyse_launch('B_02', 'B', HEAD_A, _meta('B', 3, 0), _rows(), {})]
    text = R.render_report('2026-11-25', recs, ['hdr'])
    assert '| A_01 | yes |' in text and '| B_02 | NO |' in text and '## Verdict' in text


def test_runner_dry_run_prints_plan_and_never_stops_a_process():
    sh = REPO / 'tools' / 'probes' / 'bug129_launch_series.sh'
    out = subprocess.run(['bash', str(sh), '--dry-run', '--stop-at', '23:00', '--launches-per-arm', '2',
                          '--main', str(REPO)], capture_output=True, text=True, timeout=60)
    assert out.returncode == 0, out.stderr
    assert 'DRY RUN' in out.stdout and 'auto_arm:=false' in out.stdout
    assert 'bridge_shm:=false' in out.stdout
    code = [ln for ln in sh.read_text().splitlines() if not ln.lstrip().startswith('#')]
    assert not any(re.search(r'kill\s+-(STOP|SIGSTOP|19)\b', ln) for ln in code)
    assert out.stdout.index('A_01') < out.stdout.index('B_02') < out.stdout.index('B_03') < out.stdout.index('A_04')


def test_runner_refuses_stop_at_inside_quiet_window():
    sh = REPO / 'tools' / 'probes' / 'bug129_launch_series.sh'
    out = subprocess.run(['bash', str(sh), '--dry-run', '--stop-at', '02:15', '--main', str(REPO)],
                         capture_output=True, text=True, timeout=60)
    assert out.returncode == 64 and 'refused' in out.stderr
