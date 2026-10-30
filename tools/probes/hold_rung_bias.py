#!/usr/bin/env python3
"""Pre-release-hold ladder read: attributes throws to goals and holds BY ITSELF
and prints the next operator action (map JB-102, runbook
``tests/hardware/session_oval_rung1.md`` § 11).

    python -I tools/probes/hold_rung_bias.py [--latest | --launch-log PATH]
        [--memory temp/learn/jugglebot/memory.csv] [--json temp/probes/x.json]

What it reads, and why the operator sets nothing but the hold
-------------------------------------------------------------
* The launch log (``~/.ros/log/<dir>/launch.log``) gives the goals and the holds.
  A goal is ``skill_node: <pattern>[ (reload)] started: ...`` up to its
  ``done: / stopped by the operator: / ENDED (CODE)`` line (live: up to the last
  log line).  The hold in force for a goal is the last
  ``trajectory_node: segment config: pre_release_hold_s=<v> s, ...`` line before
  its start.  The start line logs the pattern, the throw count and (oval) the
  apex, then ``· dwell <d> s · sep <N> mm`` (the goal's effective values; used
  as logged).  An OLD log without those fields falls back to attributing dwell
  from the runbook order, printed as ``0.45*`` (the ``*`` = "from goal order,
  not logged").
* The single learner memory (``temp/learn/jugglebot/memory.csv``) gives the
  throws: a row belongs to the goal whose [start, end] window holds its
  ``t_abs_s`` (same epoch clock as the log).  The bias read is the plant residual
  y - u (measured minus commanded landing), which does not move when the learner
  adapts, so one shared memory is fine.

Convention: ``dx_short`` (mm) = -sign(u0 - x0) * (y0 - u0), POSITIVE = the ball
landed SHORT of the commanded landing (displaced back toward the release site
x0).  ``dy`` = y1 - u1 and ``dapex`` = y2 - u2 in mm.

Ladder = oval_1ball goals (not reload) with logged apex 0.95 and >= 20 throws
requested.  Control = those at hold 0.100 before the first other hold; a rung =
those at another hold (same hold pooled).  Pre-registered rule: Delta = mean
dx_short(rung) - mean dx_short(control); SE = sqrt(sd1^2/n1 + sd2^2/n2).
confirmed = Delta >= 10 mm AND |Delta| > 2 SE (and |d dy|, |d dapex| < 10 mm);
no effect = |Delta| < 10 mm with n >= 20 both and |d dy|, |d dapex| < 10 mm;
else inconclusive.  Hold-0 go/no-go interpolates the confirmed Delta on the
model's 0.050 row (delay 0/10/20/40 ms) and reads the hold-0 row at that delay.

Pure stdlib, deterministic.  Run with ``python -I``.
"""
import argparse
import csv
import glob
import json
import math
import os
import re
import sys
import time

DELAYS = (0.0, 10.0, 20.0, 40.0)
MODEL = {  # hold (s) -> predicted dx short (mm) at separation delay DELAYS
    0.100: (6.6, 4.4, 2.8, 1.1),
    0.075: (15.6, 10.8, 7.5, 3.4),
    0.050: (44.5, 31.5, 22.3, 11.0),
    0.025: (68.1, 50.5, 36.5, 18.3),
    0.0: (125.2, 102.5, 81.3, 46.8),
}
CONTROL_HOLD = 0.100
MIN_N = 20
OVAL = 'Oval (Sep. 150, Apex 0.95, Dwell 0.45, Throws 30)'

_LINE = re.compile(r'^(\d+\.\d+) \[[^\]]+\] \[(\w+)\] \[[\d.]+\] \[(\w+)\]: (.*)$')
_HOLD = re.compile(r'segment config: pre_release_hold_s=([\d.]+) s')
_START = re.compile(r'^(\w+)(?: \((reload)\))? started: (.*)$')
_END = re.compile(r'^(\w+)(?: \((reload)\))? (done:|stopped by the operator:|ENDED \((\w+)\))(.*)$')
_DWELL = re.compile(r' · dwell ([\d.]+) s')
_SEP = re.compile(r' · sep ([\d.]+) mm')
_NTHROW = re.compile(r'(\d+) throws at apex ([\d.]+) m')
_CAUGHT = re.compile(r'(\d+)/(\d+) caught')


def fail(msg):
    sys.stderr.write('hold_rung_bias: %s\n' % msg)
    sys.exit(2)


# ---------------------------------------------------------------- parsing
def parse_log(path):
    """-> (holds [(t, hold)], goals [dict]).  Goals are in log order."""
    holds, goals, open_goal, last_t = [], [], None, 0.0
    with open(path, errors='replace') as fh:
        for line in fh:
            if 'started:' not in line and 'segment config' not in line \
                    and 'caught' not in line and 'stopped by' not in line:
                m0 = _LINE.match(line)
                if m0:
                    last_t = float(m0.group(1))
                continue
            m = _LINE.match(line)
            if not m:
                continue
            t, node, msg = float(m.group(1)), m.group(3), m.group(4)
            last_t = t
            if node == 'trajectory_node':
                h = _HOLD.search(msg)
                if h:
                    holds.append((t, float(h.group(1))))
            elif node == 'skill_node':
                s = _START.match(msg)
                if s:
                    if open_goal is not None:
                        open_goal['end'] = t
                        open_goal['status'] = 'no end line'
                    n = _NTHROW.search(s.group(3))
                    d = _DWELL.search(s.group(3))
                    sp = _SEP.search(s.group(3))
                    open_goal = {
                        'start': t, 'end': None, 'pattern': s.group(1),
                        'reload': bool(s.group(2)),
                        'n_throws': int(n.group(1)) if n else None,
                        'apex_m': float(n.group(2)) if n else None,
                        'dwell_logged': (float(d.group(1)) if d else None),
                        'sep_mm': (float(sp.group(1)) if sp else None),
                        'status': 'running', 'end_caught': None, 'end_total': None}
                    goals.append(open_goal)
                    continue
                e = _END.match(msg)
                if e and open_goal is not None:
                    c = _CAUGHT.search(e.group(5))
                    open_goal['end'] = t
                    open_goal['status'] = ('done' if e.group(3) == 'done:' else
                                           'stopped' if e.group(3).startswith('stopped')
                                           else 'ended ' + e.group(4))
                    if c:
                        open_goal['end_caught'] = int(c.group(1))
                        open_goal['end_total'] = int(c.group(2))
                    open_goal = None
    for g in goals:
        if g['end'] is None:
            g['end'] = last_t
        h = [v for (t, v) in holds if t <= g['start']]
        g['hold'] = h[-1] if h else None
    return holds, goals


def load_memory(path):
    with open(path, newline='') as fh:
        return list(csv.DictReader(fh))


def mstats(vals):
    n = len(vals)
    if n == 0:
        return 0, float('nan'), float('nan')
    m = sum(vals) / n
    sd = math.sqrt(sum((v - m) ** 2 for v in vals) / (n - 1)) if n > 1 else 0.0
    return n, m, sd


def row_vals(r):
    x0, u0, y0 = float(r['x0']), float(r['u0']), float(r['y0'])
    d = u0 - x0
    sgn = (d > 0) - (d < 0)
    return (-sgn * (y0 - u0) * 1000.0,
            (float(r['y1']) - float(r['u1'])) * 1000.0,
            (float(r['y2_apex_m']) - float(r['u2_apex_m'])) * 1000.0,
            str(r['caught']).strip().lower() in ('true', '1'))


def attribute(goals, rows):
    """Fill each goal's rows/stat fields; also the order-based dwell and role."""
    apex_seq = 0
    seen_rung = False
    for g in goals:
        sel = [r for r in rows if g['start'] <= float(r['t_abs_s']) <= g['end']]
        vals = [row_vals(r) for r in sel]
        g['n'] = len(vals)
        g['dx'] = [v[0] for v in vals]
        g['dy'] = [v[1] for v in vals]
        g['dapex'] = [v[2] for v in vals]
        g['row_caught'] = sum(1 for v in vals if v[3])
        g['caught'] = g['end_caught'] if g['end_caught'] is not None else g['row_caught']
        g['total'] = g['end_total'] if g['end_total'] is not None else g['n']
        g['role'] = ''
        g['dwell'] = ''
        if g['pattern'] != 'oval_1ball' or g['reload']:
            continue
        apex = g['apex_m']
        logged = g.get('dwell_logged')
        if apex is not None and abs(apex - 0.95) < 0.005 and (g['n_throws'] or 0) >= MIN_N:
            g['dwell'] = '%.2f' % logged if logged is not None else '0.45*'
            if g['hold'] is not None and abs(g['hold'] - CONTROL_HOLD) < 1e-6:
                g['role'] = 'extra control' if seen_rung else 'control'
            else:
                g['role'] = 'rung'
                seen_rung = True
        elif apex is not None and apex > 0.95:
            apex_seq += 1
            g['role'] = 'apex'
            g['dwell'] = ('%.2f' % logged if logged is not None else
                          '0.50*' if apex_seq <= 2 else '0.45*')


def pooled(goals_):
    dx = [v for g in goals_ for v in g['dx']]
    dy = [v for g in goals_ for v in g['dy']]
    da = [v for g in goals_ for v in g['dapex']]
    return {'n': len(dx), 'dx': mstats(dx), 'dy': mstats(dy), 'da': mstats(da),
            'caught': sum(g['caught'] for g in goals_),
            'total': sum(g['total'] for g in goals_),
            'goals': len(goals_)}


# ---------------------------------------------------------------- verdicts
def compare(ctl, rung):
    n1, m1, s1 = ctl['dx']
    n2, m2, s2 = rung['dx']
    out = {'n_control': n1, 'n_rung': n2, 'delta': None, 'se': None,
           'ddy': None, 'ddapex': None, 'verdict': 'inconclusive'}
    if n1 < 2 or n2 < 2:
        return out
    delta = m2 - m1
    se = math.sqrt(s1 ** 2 / n1 + s2 ** 2 / n2)
    ddy = rung['dy'][1] - ctl['dy'][1]
    dda = rung['da'][1] - ctl['da'][1]
    out.update(delta=delta, se=se, ddy=ddy, ddapex=dda)
    if n1 < MIN_N or n2 < MIN_N or abs(ddy) >= 10 or abs(dda) >= 10:
        return out
    if delta >= 10 and abs(delta) > 2 * se:
        out['verdict'] = 'confirmed'
    elif abs(delta) < 10:
        out['verdict'] = 'no effect'
    return out


def interp(xs, ys, x):
    """Piecewise-linear y(x) on xs (either direction), clamped at the ends."""
    pts = sorted(zip(xs, ys))
    if x <= pts[0][0]:
        return pts[0][1]
    for (xa, ya), (xb, yb) in zip(pts, pts[1:]):
        if x <= xb:
            return ya + (yb - ya) * (x - xa) / (xb - xa)
    return pts[-1][1]


def delta_eff(delta):
    """Separation delay (ms) at which the 0.050 model row equals the measured Delta."""
    return interp(MODEL[0.050], DELAYS, delta)


def hold0_prediction(delta):
    return interp(DELAYS, MODEL[0.0], delta_eff(delta))


def fly(hold, k=30):
    return ('ros2 param set /trajectory_node pre_release_hold_s %s ; then %s x %d '
            '; then re-run this script' % ('%.3f' % hold if hold else '0.0', OVAL, k))


def hkey(h):
    return round(h, 3)


def adopt(ctl, rungs):
    """Shortest hold with Delta <= 20 mm, all caught of >= 30, sd <= control sd + 3."""
    best = None
    for h, p in rungs.items():
        if p['n'] < 30 or p['caught'] < p['total'] or p['total'] < 30:
            continue
        c = compare(ctl, p)
        if c['delta'] is None or c['delta'] > 20 or p['dx'][2] > ctl['dx'][2] + 3:
            continue
        if best is None or h < best:
            best = h
    return best


def done_line(ctl, rungs):
    h = adopt(ctl, rungs)
    if h is None:
        return ('NEXT: ladder done - no shorter hold qualified; keep hold 0.100, set '
                'pre_release_hold_s 0.100 and continue with the Apex goals')
    return ('NEXT: ladder done - adopt hold %s; set pre_release_hold_s 0.100 and '
            'continue with the Apex goals' % ('%.3f' % h if h else '0.0'))


def decide(goals):
    """-> (ctl pooled|None, {hold: pooled}, {hold: compare}, next_line)."""
    ctl_g = [g for g in goals if g['role'] == 'control']
    rung_g = [g for g in goals if g['role'] == 'rung']
    ctl = pooled(ctl_g) if ctl_g else None
    rungs = {}
    for h in sorted({hkey(g['hold']) for g in rung_g}, reverse=True):
        rungs[h] = pooled([g for g in rung_g if hkey(g['hold']) == h])
    cmp_ = {h: compare(ctl, p) for h, p in rungs.items()} if ctl else {}
    if ctl is None or ctl['n'] < MIN_N:
        return ctl, rungs, cmp_, ('NEXT: no control yet - at hold 0.100 run %s x 30 ; '
                                  'then re-run this script' % OVAL)
    r50 = rungs.get(0.050)
    if r50 is None or r50['n'] < MIN_N:
        if r50 is None:
            return ctl, rungs, cmp_, 'NEXT: ' + fly(0.050)
        return ctl, rungs, cmp_, 'NEXT: ' + fly(0.050, MIN_N - r50['n'] + 10)
    c50 = cmp_[0.050]
    target, stop = None, False
    if c50['verdict'] == 'no effect':
        target = 0.0
    elif c50['verdict'] == 'confirmed':
        pred = hold0_prediction(c50['delta'])
        if pred <= 35:
            target = 0.0
        elif pred <= 55:
            target = 0.025
        else:
            target, stop = 0.075, True
    else:
        if r50['n'] < 40:
            return ctl, rungs, cmp_, ('NEXT: inconclusive - ' + 'fly 20 more at 0.050 : '
                                      + fly(0.050, 20))
        target, stop = 0.075, True
    t = rungs.get(hkey(target))
    if t is None or t['n'] < 30:
        need = 30 if t is None else 30 - t['n']
        return ctl, rungs, cmp_, 'NEXT: ' + fly(target, need) + (
            '  (fallback; this stops the ladder)' if stop else '')
    if target == 0.025 and t['caught'] >= 0.9 * t['total']:
        z = rungs.get(0.0)
        if z is None or z['n'] < 30:
            return ctl, rungs, cmp_, 'NEXT: ' + fly(0.0, 30 if z is None else 30 - z['n'])
    return ctl, rungs, cmp_, done_line(ctl, rungs)


# ---------------------------------------------------------------- output
def fmt_ms(v):
    return '%+.1f' % v if v == v else '-'


def render(goals, ctl, rungs, cmp_, nxt, log_path, mem_path):
    out = ['launch log: %s' % log_path, 'memory:     %s' % mem_path,
           'dx_short in mm, POSITIVE = landed SHORT of the commanded landing '
           '(toward the release site); dwell * = inferred from goal order (old log lacking the dwell field).', '']
    out.append('%-3s %-8s %-18s %-6s %-6s %-5s %-5s %-10s %-14s %-8s %-8s %s' % (
        '#', 'start', 'pattern', 'hold', 'dwell', 'apex', 'rows', 'caught',
        'dx_short', 'dy', 'dapex', 'role / status'))
    for i, g in enumerate(goals, 1):
        t = time.strftime('%H:%M:%S', time.localtime(g['start']))
        _, mx, sx = mstats(g['dx'])
        _, my, _ = mstats(g['dy'])
        _, ma, _ = mstats(g['dapex'])
        name = g['pattern'] + (' (reload)' if g['reload'] else '')
        out.append('%-3d %-8s %-18s %-6s %-6s %-5s %-5d %-10s %-14s %-8s %-8s %s' % (
            i, t, name, '-' if g['hold'] is None else '%.3f' % g['hold'],
            g['dwell'] or '-', '-' if g['apex_m'] is None else '%.2f' % g['apex_m'],
            g['n'], '%d/%d' % (g['caught'], g['total']),
            '%s +-%.1f' % (fmt_ms(mx), sx) if g['n'] > 1 else '-',
            fmt_ms(my), fmt_ms(ma), (g['role'] + ' ' if g['role'] else '') + g['status']))
    out.append('')
    if ctl is not None:
        out.append('control (hold 0.100): n=%d  dx_short %s +-%.1f mm  caught %d/%d' % (
            ctl['n'], fmt_ms(ctl['dx'][1]), ctl['dx'][2], ctl['caught'], ctl['total']))
    for h, p in rungs.items():
        c = cmp_[h]
        if c['delta'] is None:
            out.append('rung %.3f: n=%d  (too few rows for a verdict)' % (h, p['n']))
            continue
        line = ('rung %.3f: n=%d  Delta %s mm  SE %.1f  d_dy %s  d_dapex %s  caught %d/%d  '
                'sd %.1f  -> %s' % (h, p['n'], fmt_ms(c['delta']), c['se'], fmt_ms(c['ddy']),
                                    fmt_ms(c['ddapex']), p['caught'], p['total'],
                                    p['dx'][2], c['verdict'].upper()))
        if h == 0.050 and c['verdict'] == 'confirmed':
            line += '  (delta_eff %.0f ms, hold-0 prediction %.0f mm)' % (
                delta_eff(c['delta']), hold0_prediction(c['delta']))
        out.append(line)
    out += ['', nxt]
    return '\n'.join(out)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--latest', action='store_true', help='newest ~/.ros/log/2026-*/launch.log')
    ap.add_argument('--launch-log')
    ap.add_argument('--memory', default=os.path.join('temp', 'learn', 'jugglebot', 'memory.csv'))
    ap.add_argument('--json')
    a = ap.parse_args(argv)
    if a.launch_log:
        log = a.launch_log
    elif a.latest:
        c = glob.glob(os.path.expanduser('~/.ros/log/2026-*/launch.log'))
        if not c:
            fail('no ~/.ros/log/2026-*/launch.log found')
        log = max(c, key=lambda p: (os.path.getmtime(p), p))
    else:
        fail('give --latest or --launch-log PATH')
    if not os.path.isfile(log):
        fail('launch log not found: %s' % log)
    if not os.path.isfile(a.memory):
        fail('memory file not found: %s' % a.memory)
    holds, goals = parse_log(log)
    if not goals:
        fail('no juggling goals ("... started:" lines) in %s' % log)
    attribute(goals, load_memory(a.memory))
    ctl, rungs, cmp_, nxt = decide(goals)
    print(render(goals, ctl, rungs, cmp_, nxt, log, a.memory))
    if a.json:
        d = os.path.dirname(a.json)
        if d:
            os.makedirs(d, exist_ok=True)
        with open(a.json, 'w') as fh:
            json.dump({'launch_log': log, 'memory': a.memory, 'holds': holds,
                       'goals': [{k: v for k, v in g.items() if k not in ('dx', 'dy', 'dapex')}
                                 for g in goals],
                       'rungs': {str(h): {'pooled_n': p['n'], 'compare': cmp_[h]}
                                 for h, p in rungs.items()},
                       'next': nxt}, fh, indent=1, sort_keys=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
