"""JB-149 scheduler sampler: CPU preemption vs whole-process pause, from OUTSIDE.

A live observer (reads /proc only; never commands anything).  Run beside the
launch.  ``record`` samples, at 20 Hz, every launch python process
(skill_node, its solver worker children, trajectory_node, teensy_bridge_node):
per pid ``/proc/<pid>/sched`` (nr_voluntary_switches, nr_involuntary_switches,
sum_exec_runtime, nr_switches), ``/proc/<pid>/schedstat`` (run_ns, wait_ns,
timeslices) and the involuntary switches summed over every thread
(``/proc/<pid>/task/*/sched``).  ``report`` joins that CSV with a launch log:
for each CATCH solve line it prints, per process, the involuntary-switch delta
and CPU-vs-wall over the solve window, plus the worst GC_PROBE pause logged in
that window.

Solve line pattern (the one tools/probes/solve_starvation_scan.py already
uses; emitted by trajectory_node's solver log):
    ``<epoch> ... install_segment CATCH ... solve <W> ms (cpu <C> ms, ivcsw <N>, vcsw <M>)``
The log line is written when the solve ends, so the window is [t - W, t].
GC line (jugglebot/gc_probe.py): ``<epoch> ... GC_PROBE <tag> n=.. max_ms=.. ... worst_wall=<epoch>``.

Motivating Item: map/bug-40-hz-setpoint-stream-starved-past-the-firmware-s-70-ms.md (JB-149).
Logbook: logbook/2026-11-03-jb149-gc-sched-probe.md

Usage (repo root)::

    python tools/probes/sched_probe.py record [--hz 20] [--out temp/probes/sched_probe_<ts>.csv]
    python tools/probes/sched_probe.py report <csv> <launch.log>
"""
from __future__ import annotations

import argparse
import csv
import os
import re
import signal
import sys
import time

PROC_NAMES = ('skill_node', 'trajectory_node', 'teensy_bridge_node')
SCHED_KEYS = ('nr_voluntary_switches', 'nr_involuntary_switches',
              'sum_exec_runtime', 'nr_switches')
PID_FIELDS = SCHED_KEYS + ('run_ns', 'wait_ns', 'timeslices', 'task_ivcsw')


def parse_sched(text):
    """``/proc/<pid>/sched`` text -> {key: float} for SCHED_KEYS present."""
    out = {}
    for ln in text.splitlines():
        if ':' not in ln:
            continue
        k, _, v = ln.partition(':')
        k = k.strip()
        if k == 'se.sum_exec_runtime':   # the kernel's spelling
            k = 'sum_exec_runtime'
        if k in SCHED_KEYS:
            try:
                out[k] = float(v.strip())
            except ValueError:
                pass
    return out


def parse_schedstat(text):
    p = text.split()
    if len(p) < 3:
        return {}
    try:
        return {'run_ns': float(p[0]), 'wait_ns': float(p[1]),
                'timeslices': float(p[2])}
    except ValueError:
        return {}


def _read(path):
    try:
        with open(path) as f:
            return f.read()
    except OSError:
        return None


def sample_pid(pid, proc='/proc'):
    """One pid's field dict (missing fields omitted, never raises)."""
    d = {}
    t = _read('%s/%d/sched' % (proc, pid))
    if t:
        d.update(parse_sched(t))
    t = _read('%s/%d/schedstat' % (proc, pid))
    if t:
        d.update(parse_schedstat(t))
    try:
        tids = os.listdir('%s/%d/task' % (proc, pid))
    except OSError:
        tids = []
    tot, seen = 0.0, False
    for tid in tids:
        t = _read('%s/%d/task/%s/sched' % (proc, pid, tid))
        if t:
            v = parse_sched(t).get('nr_involuntary_switches')
            if v is not None:
                tot += v
                seen = True
    if seen:
        d['task_ivcsw'] = tot
    return d


def _cmdline(pid, proc='/proc'):
    t = _read('%s/%d/cmdline' % (proc, pid))
    return (t or '').replace('\0', ' ').strip()


def _ppid(pid, proc='/proc'):
    t = _read('%s/%d/stat' % (proc, pid))
    if not t:
        return None
    try:
        return int(t.rsplit(')', 1)[1].split()[1])
    except (IndexError, ValueError):
        return None


def find_processes(proc='/proc'):
    """{pid: label}: the named nodes plus every descendant (solver workers)."""
    pids = {}
    for e in os.listdir(proc):
        if e.isdigit():
            pids[int(e)] = None
    roots = {}
    for pid in pids:
        cmd = _cmdline(pid, proc)
        if 'python' not in cmd:
            continue
        for n in PROC_NAMES:
            if n in cmd.split('/')[-1] or (' ' + n) in cmd or ('/' + n) in cmd:
                roots[pid] = n
                break
    ppids = {pid: _ppid(pid, proc) for pid in pids}
    labels = dict(roots)
    changed = True
    while changed:
        changed = False
        for pid, pp in ppids.items():
            if pid not in labels and pp in labels:
                labels[pid] = labels[pp] + '_child'
                changed = True
    return labels


def column_names(labels):
    return ['%s_%d_%s' % (lab, pid, f) for pid, lab in sorted(labels.items())
            for f in PID_FIELDS]


def record(out, hz=20.0, proc='/proc', duration=None):
    os.makedirs(os.path.dirname(out) or '.', exist_ok=True)
    stop = []
    signal.signal(signal.SIGINT, lambda *a: stop.append(1))
    signal.signal(signal.SIGTERM, lambda *a: stop.append(1))
    labels = {}
    rescan = 0
    period = 1.0 / hz
    t0 = time.time()
    # Columns are fixed per pid set; a new pid (worker respawn) starts a new
    # CSV block (a repeated header row), which read_csv() handles.
    with open(out, 'w', newline='') as f:
        w = csv.writer(f)
        header = None
        nxt = time.time()
        while not stop and (duration is None or time.time() - t0 < duration):
            if rescan <= 0:
                new = find_processes(proc)
                rescan = int(hz * 2)
                if new != labels or header is None:
                    labels = new
                    header = ['wall_time'] + column_names(labels)
                    w.writerow(header)
            rescan -= 1
            row = [repr(time.time())]
            for pid, lab in sorted(labels.items()):
                d = sample_pid(pid, proc)
                row += [repr(d[k]) if k in d else '' for k in PID_FIELDS]
            w.writerow(row)
            f.flush()
            nxt += period
            time.sleep(max(0.0, nxt - time.time()))
    print('wrote', out, file=sys.stderr)


def read_csv(path):
    """-> list of (wall_time, {col: float}) rows.  Tolerates repeated header
    blocks and empty cells (omitted)."""
    rows = []
    header = None
    with open(path, newline='') as f:
        for r in csv.reader(f):
            if not r:
                continue
            if r[0] == 'wall_time':
                header = r
                continue
            if header is None:
                continue
            try:
                t = float(r[0])
            except ValueError:
                continue
            d = {}
            for k, v in zip(header[1:], r[1:]):
                if v != '':
                    try:
                        d[k] = float(v)
                    except ValueError:
                        pass
            rows.append((t, d))
    return rows


SOLVE_RX = re.compile(
    r'^(\d+\.\d+) .*?install_segment CATCH .*?solve ([\d.]+) ms '
    r'\(cpu ([\d.]+) ms, ivcsw (\d+), vcsw (\d+)\)')
GC_RX = re.compile(r'^(\d+\.\d+) .*GC_PROBE (\w+) n=(\d+) max_ms=([\d.]+) ')


def parse_log(lines):
    solves, gcs = [], []
    for ln in lines:
        m = SOLVE_RX.search(ln)
        if m:
            solves.append(dict(t=float(m.group(1)), wall=float(m.group(2)),
                               cpu=float(m.group(3)), ivcsw=int(m.group(4))))
            continue
        m = GC_RX.search(ln)
        if m:
            gcs.append(dict(t=float(m.group(1)), tag=m.group(2),
                            n=int(m.group(3)), max_ms=float(m.group(4))))
    return solves, gcs


def window_rows(rows, t0, t1):
    """Rows bracketing [t0, t1]: last row <= t0 and first row >= t1 (or None)."""
    a = b = None
    for t, d in rows:
        if t <= t0:
            a = (t, d)
        if t >= t1 and b is None:
            b = (t, d)
    return a, b


def _by_proc(cols):
    out = {}
    for c in cols:
        m = re.match(r'(.+)_(\d+)_(\w+)$', c)
        if m:
            out.setdefault('%s:%s' % (m.group(1), m.group(2)), {})[m.group(3)] = c
    return out


def window_report(rows, solve, gcs):
    t1 = solve['t']
    t0 = t1 - solve['wall'] / 1e3
    a, b = window_rows(rows, t0, t1)
    lines = ['CATCH solve @%.3f wall %.1f ms cpu %.1f ms ivcsw(worker) %d' %
             (t1, solve['wall'], solve['cpu'], solve['ivcsw'])]
    if a is None or b is None:
        lines.append('  no sampler rows bracket this window (sampler not running?)')
    else:
        dt = b[0] - a[0]
        lines.append('  sampler window %.3f s' % dt)
        procs = _by_proc(set(a[1]) | set(b[1]))
        for p, f in sorted(procs.items()):
            def delta(field):
                ca = cb = f.get(field)
                if ca in a[1] and cb in b[1]:
                    return b[1][cb] - a[1][ca]
                return None
            iv = delta('task_ivcsw')
            if iv is None:
                iv = delta('nr_involuntary_switches')
            run = delta('sum_exec_runtime')
            wait = delta('wait_ns')
            lines.append('  %-28s ivcsw %s  cpu %s ms of %.1f ms wall  runq-wait %s ms' % (
                p, 'n/a' if iv is None else '%d' % iv,
                'n/a' if run is None else '%.1f' % run,
                dt * 1e3, 'n/a' if wait is None else '%.1f' % (wait / 1e6)))
    g = [x for x in gcs if t0 <= x['t'] <= t1 + 1.0]
    if g:
        w = max(g, key=lambda x: x['max_ms'])
        lines.append('  GC_PROBE worst in window (1 Hz lines, +1 s): %.3f ms (%s)' %
                     (w['max_ms'], w['tag']))
    else:
        lines.append('  GC_PROBE: no line in window (probe off, or none logged)')
    return lines


def report(csv_path, log_path, out=sys.stdout):
    rows = read_csv(csv_path)
    with open(log_path, errors='replace') as f:
        solves, gcs = parse_log(f)
    print('sampler rows %d, CATCH solves %d, GC_PROBE lines %d' %
          (len(rows), len(solves), len(gcs)), file=out)
    for s in solves:
        print('\n'.join(window_report(rows, s, gcs)), file=out)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    r = sub.add_parser('record')
    r.add_argument('--hz', type=float, default=20.0)
    r.add_argument('--out', default=None)
    r.add_argument('--duration', type=float, default=None)
    p = sub.add_parser('report')
    p.add_argument('csv')
    p.add_argument('log')
    a = ap.parse_args(argv)
    if a.cmd == 'record':
        out = a.out or 'temp/probes/sched_probe_%s.csv' % time.strftime('%Y%m%d_%H%M%S')
        record(out, a.hz, duration=a.duration)
    else:
        report(a.csv, a.log)


if __name__ == '__main__':
    main()
