"""BUG-58: tools/probes/sched_probe.py parsers, CSV round trip, report windows."""
from __future__ import annotations

import importlib.util
import os

_P = os.path.join(os.path.dirname(__file__), '..', '..', 'tools', 'probes',
                  'sched_probe.py')
_spec = importlib.util.spec_from_file_location('sched_probe', _P)
sp = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(sp)

SCHED = """python3 (1234, #threads: 3)
-------------------------------------------------------------------
se.sum_exec_runtime                          :        100.500000
nr_switches                                  :                50
nr_voluntary_switches                        :                40
nr_involuntary_switches                      :                10
policy                                       :                 0
"""


def test_parse_sched_and_schedstat():
    d = sp.parse_sched(SCHED)
    assert d['nr_involuntary_switches'] == 10.0
    assert d['nr_voluntary_switches'] == 40.0
    assert d['nr_switches'] == 50.0
    assert d['sum_exec_runtime'] == 100.5   # kernel spells it se.sum_exec_runtime
    assert sp.parse_schedstat('1000 2000 3\n') == {
        'run_ns': 1000.0, 'wait_ns': 2000.0, 'timeslices': 3.0}
    assert sp.parse_schedstat('garbage') == {}
    assert sp.parse_sched('no colon lines') == {}


def test_sample_pid_tolerates_missing(tmp_path):
    assert sp.sample_pid(99999, proc=str(tmp_path)) == {}
    pd = tmp_path / '7'
    (pd / 'task' / '7').mkdir(parents=True)
    (pd / 'task' / '8').mkdir(parents=True)
    (pd / 'sched').write_text(SCHED)
    for t, n in (('7', 3), ('8', 4)):
        (pd / 'task' / t / 'sched').write_text(
            SCHED.replace('                10', '                %d' % n))
    d = sp.sample_pid(7, proc=str(tmp_path))
    assert d['task_ivcsw'] == 7.0 and 'run_ns' not in d


def test_record_csv_roundtrip(tmp_path):
    out = tmp_path / 'x.csv'
    out.write_text(
        'wall_time,a_1_task_ivcsw,a_1_sum_exec_runtime\n'
        '100.0,5.0,10.0\n'
        '100.05,,12.0\n'
        'wall_time,a_1_task_ivcsw\n'
        '100.1,9.0\n')
    rows = sp.read_csv(str(out))
    assert [t for t, _ in rows] == [100.0, 100.05, 100.1]
    assert rows[1][1] == {'a_1_sum_exec_runtime': 12.0}
    assert rows[2][1] == {'a_1_task_ivcsw': 9.0}


LOG = """\
1000.100 [trajectory_node] install_segment CATCH x solve 100.0 ms (cpu 60.0 ms, ivcsw 81, vcsw 3)
1000.200 [trajectory_node] install_segment THROW x solve 50.0 ms (cpu 40.0 ms, ivcsw 1, vcsw 3)
1000.300 [teensy_bridge_node] GC_PROBE teensy_bridge_node n=2 max_ms=12.500 total_ms=13.000 worst_gen=2 worst_wall=1000.1
noise line
"""


def test_parse_log_and_window_selection(tmp_path):
    solves, gcs = sp.parse_log(LOG.splitlines())
    assert len(solves) == 1 and solves[0]['ivcsw'] == 81   # THROW skipped
    assert gcs[0]['max_ms'] == 12.5
    rows = [(999.9 + 0.1 * i, {'p_1_task_ivcsw': float(i),
                                 'p_1_sum_exec_runtime': 1e6 * i})
            for i in range(10)]
    a, b = sp.window_rows(rows, 1000.0, 1000.1)
    assert a[0] <= 1000.0 and b[0] >= 1000.1
    csvp = tmp_path / 's.csv'
    with open(csvp, 'w') as f:
        f.write('wall_time,p_1_task_ivcsw,p_1_sum_exec_runtime\n')
        for t, d in rows:
            f.write('%r,%r,%r\n' % (t, d['p_1_task_ivcsw'],
                                    d['p_1_sum_exec_runtime']))
    logp = tmp_path / 'l.log'
    logp.write_text(LOG)
    import io
    buf = io.StringIO()
    sp.report(str(csvp), str(logp), out=buf)
    text = buf.getvalue()
    assert 'CATCH solve @1000.100' in text
    assert 'p:1' in text and 'ivcsw 1 ' in text
    assert 'GC_PROBE worst in window' in text and '12.500' in text


def test_report_with_no_sampler_rows(tmp_path):
    csvp = tmp_path / 'e.csv'
    csvp.write_text('wall_time\n')
    logp = tmp_path / 'l.log'
    logp.write_text(LOG)
    import io
    buf = io.StringIO()
    sp.report(str(csvp), str(logp), out=buf)
    assert 'no sampler rows' in buf.getvalue()


NEW_GC_LINE = (
    "1000.300 [trajectory_node] GC_PROBE trajectory_node n=1 max_ms=80.250 "
    "total_ms=80.250 worst_gen=2 worst_wall=1000.1 worst_collected=5000 "
    "worst_uncollectable=0 gc_count=3,4,5 gc_colls=100,9,2 gc_frozen=0 "
    "gc_objects0=29295")


def test_old_and_new_gc_lines_both_parse_and_report_detail():
    _, gcs = sp.parse_log(LOG.splitlines())
    assert gcs[0]['extra'] == {}                      # old log: no new fields
    _, gcs = sp.parse_log([NEW_GC_LINE])
    assert gcs[0]['max_ms'] == 80.25 and gcs[0]['n'] == 1
    assert gcs[0]['extra']['worst_collected'] == '5000'
    assert gcs[0]['extra']['gc_count'] == '3,4,5'
    solve = dict(t=1000.4, wall=300.0, cpu=1.0, ivcsw=0)
    new = sp.window_report([], solve, gcs)
    assert any('worst-line detail' in ln and 'gc_frozen=0' in ln for ln in new)
    old = sp.window_report([], solve, sp.parse_log(LOG.splitlines())[1])
    assert not any('worst-line detail' in ln for ln in old)
    assert any('GC_PROBE worst in window' in ln for ln in old)
