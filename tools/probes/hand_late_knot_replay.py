#!/usr/bin/env python3
"""Replay a late or dropped Hand setpoint frame through the firmware twin (BUG-111, BUG-89).

WHAT IT ANSWERS
    Did the can-bridge's 500 Hz Hand command step (position or velocity) across
    the release knot of a Throw when the setpoint frame carrying that knot was
    late or dropped, and would it have stepped with one knot of emit lead
    (``trajectory_op.emit_lead_knots`` 0 -> 1) or with the other options on the
    BUG-111 / BUG-89 Items?

HOW
    1. ``fit``: the bags carry no Hand knot topic, so the knot plan is rebuilt
       from the firmware's Hand command echo in a hand capture CSV
       (``kind == 'echo'``: cmd_pos, cmd_vel_ff at 10 ms, stamped at the interp
       tick that produced them).  The plan on the wire is a piecewise cubic
       Hermite through (q_j, v_j) at 25 ms knots, so the echo is LINEAR in the
       knot values: a weighted least-squares fit over a window of knots, with
       the echo samples inside the suspected hole (previous frame's cover end
       to the late frame's promotion) left out and then PREDICTED by the twin
       as the check.
    2. ``replay``: frames are rebuilt from the fitted knots (frame j carries
       knots j, j+1, j+2 and is stamped at knot j) and fed to the firmware's
       Python twin (``teensy_link_profiling/hermite_xref/teensy_interp.py``:
       ``sched_on_setpoint`` queue by t_start, ``SCHED_QUEUE_LEN`` 4,
       ``sched_step`` promote-newest-due then advance, the FW 37/39 Hand C1
       stop, the 20 ms Hand grace) at the firmware's 2 ms tick phase, with the
       arrival times read from the launch log's ``setpoint drain dropped`` WARN.
       Scenarios: as flown; emit lead 1 (every frame 25 ms earlier, same
       transport delay, newest-only bridge); lead 1 with the due-aware bridge
       rule; constant-velocity extrapolation in the grace instead of the
       continued cubic.  A sweep over the forwarded frame's lag gives the step
       size per ms of hole across the release knot.
    3. ``census``: every ``setpoint drain dropped`` WARN in the launch logs,
       bucketed into Attempts (skill ``started`` .. ``ENDED``/``done`` lines),
       with the hole each drop opens at lead 0 and what it would be at lead 1;
       with ``--bags``, the firmware's per-heartbeat frame-age census
       (``/link_status`` ``rx_age_*``) inside the same Attempt windows, which
       counts late frames that were NOT drops.
    4. ``scan``: step 1-2 (no plots) for EVERY drop line a Hand capture covers,
       with the plan's velocity and acceleration at the hole and whether a Hand
       ODrive fault followed within 0.3 s, so a step at a release knot that did
       NOT end in a fault is visible next to the two that did.
    ``--data-root`` points at the checkout whose gitignored temp/ holds the data.

ASSUMPTIONS (stated so they can be argued with)
    * The echo is the command the interp produced at its stamped tick
      (``telemetry.cpp hand_cmd_echo_uplink_step``, age-corrected stamp).
    * Frames other than the dropped / late pair arrived inside the previous
      frame's cover (checked: the flown replay must reproduce every echo
      sample, including the left-out ones).
    * Bridge drain -> firmware queue entry costs ``--transit-ms`` (default
      1.0 ms; the report sweeps it).
    * Emit lead 1 shifts every send 25 ms earlier and leaves every transport
      delay (emitter -> drain -> firmware) unchanged, so the same frames
      coalesce in the same drains.
    * Perfect tracking: the twin's lead clamp is fed the command itself, so
      the replay is the COMMAND only; the drive's response is not modelled.

OUTPUTS
    ``--out DIR`` (default ``temp/probes/hand_late_knot_replay/<stamp>/``):
    ``<event>_replay.csv`` (500 Hz command per scenario), ``<event>_*.png``,
    ``census.json``, and a JSON summary on stdout.

USAGE
    python tools/probes/hand_late_knot_replay.py replay --event 2026-11-20
    python tools/probes/hand_late_knot_replay.py replay --event 2026-11-18
    python tools/probes/hand_late_knot_replay.py scan \
        --logs 'temp/logs/launch_2026-11-18_*.log' 'temp/logs/launch_2026-11-20_*.log'
    python tools/probes/hand_late_knot_replay.py census \
        --logs 'temp/logs/launch_2026-11-18_*.log' 'temp/logs/launch_2026-11-20_*.log' \
        --bags ~/Desktop/rosbags/2026-11-20_12-49-25

Motivated by BUG-111 (Hand SPINOUT at the Throw peak after a late setpoint) and
the BUG-89 emit-lead decision; the 2026-11-18 debrief's scripts for step 1 were
lost, this is their committed replacement.
"""

from __future__ import annotations

import argparse
import contextlib
import csv
import datetime as _dt
import glob
import json
import os
import re
import sys

import numpy as np

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(REPO, 'tools', 'probes', 'teensy_link_profiling', 'hermite_xref'))
import teensy_interp as ti  # noqa: E402

T_US = 25000
T_S = T_US * 1e-6
TICK_US = 2000
HAND = ti.HAND_AXIS

# The two recorded events.  Numbers that come from the launch log are re-read
# from it (``parse_drop_line``); these are only the pointers.
EVENTS = {
    '2026-11-20': dict(capture='temp/hand_capture/hand_capture_20261120_124928.csv',
                       log='temp/logs/launch_2026-11-20_1.log', dropped_seq=50752),
    '2026-11-18': dict(capture='temp/hand_capture/hand_capture_20261118_103811.csv',
                       log='temp/logs/launch_2026-11-18_1.log', dropped_seq=2723),
}

_ANSI = re.compile(r'\x1b\[[0-9;]*m')
_DROP = re.compile(
    r'^(?P<hms>\d\d:\d\d:\d\d\.\d+) .*setpoint drain dropped (?P<n>\d+) older frame\(s\).*'
    r'dropped seqs? (?P<seqs>[\d.]+), stamp (?P<stamp>[\d.]+|n/a), lag (?P<dlag>[\d.]+|n/a)'
    r'(?: ms)? at drain; forwarded seq (?P<fseq>\d+), lag (?P<flag>[\d.]+|n/a)(?: ms)?; '
    r'total (?P<total>\d+) \((?P<more>\d+) more')
_ATT_START = re.compile(r'^(?P<hms>\d\d:\d\d:\d\d\.\d+) skill +(?:\S+ +)?(?P<pat>[a-z_0-9]+) '
                        r'(?:\([a-z]+\) )?started\b')
_ATT_END = re.compile(r'^(?P<hms>\d\d:\d\d:\d\d\.\d+) skill +(?:ERROR +|WARN +)?'
                      r'(?P<pat>[a-z_0-9]+) (?:ENDED|done)\b')


# ── log parsing ──────────────────────────────────────────────────────────────

def _clean_lines(path):
    with open(path, errors='replace') as fh:
        for i, line in enumerate(fh, 1):
            yield i, _ANSI.sub('', line.rstrip('\n'))


def parse_drop_line(log, dropped_seq):
    """The WARN naming ``dropped_seq`` -> dict (stamp s, lags ms, drain wall s)."""
    for i, line in _clean_lines(log):
        m = _DROP.match(line)
        if m and m.group('seqs').split('..')[-1] == str(dropped_seq):
            stamp = float(m.group('stamp'))
            dlag = float(m.group('dlag'))
            return dict(line=i, n=int(m.group('n')), dropped_seq=dropped_seq,
                        dropped_stamp=stamp, dropped_lag_ms=dlag,
                        fwd_seq=int(m.group('fseq')), fwd_lag_ms=float(m.group('flag')),
                        drain_wall=stamp + dlag * 1e-3, text=line)
    raise SystemExit('no drop line for seq %s in %s' % (dropped_seq, log))


def _log_date(path):
    m = re.search(r'(\d{4}-\d\d-\d\d)', os.path.basename(path))
    return m.group(1)


def _hms_epoch(date, hms):
    """Local wall clock of a launch-log line -> epoch s (the box's local zone)."""
    return _dt.datetime.strptime(date + ' ' + hms, '%Y-%m-%d %H:%M:%S.%f').timestamp()


def scan_log(path):
    date = _log_date(path)
    drops, attempts, open_att = [], [], None
    for i, line in _clean_lines(path):
        m = _DROP.match(line)
        if m:
            drops.append(dict(
                log=os.path.basename(path), line=i, t=_hms_epoch(date, m.group('hms')),
                n=int(m.group('n')), seqs=m.group('seqs'),
                stamp=None if m.group('stamp') == 'n/a' else float(m.group('stamp')),
                dlag=None if m.group('dlag') == 'n/a' else float(m.group('dlag')),
                flag=None if m.group('flag') == 'n/a' else float(m.group('flag')),
                total=int(m.group('total')), more=int(m.group('more'))))
            continue
        m = _ATT_START.match(line)
        if m and open_att is None:
            open_att = dict(log=os.path.basename(path), line=i, pattern=m.group('pat'),
                            t0=_hms_epoch(date, m.group('hms')))
            continue
        m = _ATT_END.match(line)
        if m and open_att is not None:
            open_att.update(t1=_hms_epoch(date, m.group('hms')), end_line=i,
                            end='done' if ' done' in line else 'ENDED')
            attempts.append(open_att)
            open_att = None
    return drops, attempts


def hole_ms(d, a, lead, rule):
    """Hole (ms) between the last forwarded frame's 2T cover end and the promotion
    of the next frame, for a drain that DROPPED ``d`` frames and forwarded one
    whose lead-0 lag at drain was ``a`` ms (transit and tick phase excluded).

    lead 0, newest-only (as flown): (d-1)T + a.
    lead 1, newest-only: the forwarded frame is played on its stamp unless it is
    still late: (d-1)T + max(0, a - T).
    lead 1, due-aware (newest due frame + every not-yet-due frame): the newest
    dropped frame is due with lag a, so the hole is max(0, (d-2)T + a) when
    d >= 2, and 0 when d == 1 (it lands inside the cover).
    """
    T = T_US * 1e-3
    if lead == 0:
        return (d - 1) * T + a
    if rule == 'newest':
        return (d - 1) * T + max(0.0, a - T)
    return 0.0 if d == 1 and a < T else max(0.0, (d - 2) * T + a)


# ── the echo and the knot fit ────────────────────────────────────────────────

def load_echo(capture, t0_us, t1_us):
    rows = []
    with open(capture, newline='') as fh:
        rd = csv.reader(fh)
        hdr = next(rd)
        ik, it = hdr.index('kind'), hdr.index('t_src_us')
        ip, iv, itq = (hdr.index('cmd_pos_rev'), hdr.index('cmd_vel_ff_rps'),
                       hdr.index('cmd_tor_ff'))
        for r in rd:
            if r[ik] != 'echo':
                continue
            t = int(r[it])
            if t0_us <= t <= t1_us:
                rows.append((t, float(r[ip]), float(r[iv]), float(r[itq])))
    rows.sort()
    return np.array(rows, dtype=float)


def _hermite_rows(t_us, origin_us, nk):
    """Design rows (pos, vel) of the knot vector [q_0..q_nk-1, v_0..v_nk-1]."""
    j = int((t_us - origin_us) // T_US)
    s = (t_us - origin_us - j * T_US) / T_US
    if j < 0 or j + 1 >= nk:
        return None
    s2, s3 = s * s, s * s * s
    rp, rv = np.zeros(2 * nk), np.zeros(2 * nk)
    rp[j], rp[j + 1] = 2 * s3 - 3 * s2 + 1, -2 * s3 + 3 * s2
    rp[nk + j], rp[nk + j + 1] = (s3 - 2 * s2 + s) * T_S, (s3 - s2) * T_S
    rv[j], rv[j + 1] = (6 * s2 - 6 * s) / T_S, (-6 * s2 + 6 * s) / T_S
    rv[nk + j], rv[nk + j + 1] = 3 * s2 - 4 * s + 1, 3 * s2 - 2 * s
    return rp, rv


def fit_knots(echo, origin_us, nk, exclude, w_pos=1e-3, w_vel=1e-2):
    """Least-squares knots from the echo samples not in ``exclude`` ((lo, hi) us)."""
    A, b, used = [], [], []
    for k, (t, p, v, _tq) in enumerate(echo):
        if any(lo <= t <= hi for lo, hi in exclude):
            continue
        if p <= 0.0 or p >= ti.HAND_MOTOR_MAX_POSITION - 1e-4:
            continue                       # the stroke clip zeroes vel_ff there
        rows = _hermite_rows(t, origin_us, nk)
        if rows is None:
            continue
        A += [rows[0] / w_pos, rows[1] / w_vel]
        b += [p / w_pos, v / w_vel]
        used.append(k)
    A, b = np.array(A), np.array(b)
    x, *_ = np.linalg.lstsq(A, b, rcond=None)
    q, v = x[:nk], x[nk:]
    res = (A @ x - b)
    return q, v, dict(n_used=len(used), used=used,
                      pos_rms=float(np.sqrt(np.mean((res[0::2] * w_pos) ** 2))),
                      pos_max=float(np.max(np.abs(res[0::2] * w_pos))),
                      vel_rms=float(np.sqrt(np.mean((res[1::2] * w_vel) ** 2))),
                      vel_max=float(np.max(np.abs(res[1::2] * w_vel))))


def plan_eval(q, v, origin_us, t_us):
    j = int((t_us - origin_us) // T_US)
    j = min(max(j, 0), len(q) - 2)
    s = (t_us - origin_us - j * T_US) / T_US
    return ti.span_eval(q[j], v[j], q[j + 1], v[j + 1], s)


def knot_accel_jumps(q, v):
    """a at the end of span j-1 and the start of span j, per interior knot j."""
    out = []
    for j in range(1, len(q) - 1):
        a_in = ti.span_eval(q[j - 1], v[j - 1], q[j], v[j], 1.0)[2]
        a_out = ti.span_eval(q[j], v[j], q[j + 1], v[j + 1], 0.0)[2]
        out.append((j, a_in, a_out))
    return out


# ── the twin replay ──────────────────────────────────────────────────────────

@contextlib.contextmanager
def constant_velocity_grace():
    """Counterfactual firmware: past span 2's end knot (s > 1) the Hand group
    continues at the end-knot velocity instead of the continued cubic.  Swaps
    the twin's ``span_eval`` (which play_eval, the C1 decision and the stop
    entry all call), so the grace, the C1 stop and the promotion check all see
    the constant-velocity state.  Legs carry zeros in this replay."""
    orig = ti.span_eval

    def cv(P0, V0, P1, V1, s):
        if s <= 1.0:
            return orig(P0, V0, P1, V1, s)
        return P1 + V1 * (s - 1.0) * T_S, V1, 0.0
    ti.span_eval = cv
    try:
        yield
    finally:
        ti.span_eval = orig


def replay(q, v, origin_us, arrivals, t_from_us, t_to_us, tick_phase_us, epoch_us,
           cv=False):
    """Run the twin.  ``arrivals``: {knot index j: firmware queue-entry us or None}.
    Times are absolute wall us; the twin runs on ``t - epoch_us`` so its float
    seconds keep microsecond precision."""
    it = ti.TeensyLegInterp([0.0] * ti.NUM_LEGS, [10.0] * ti.NUM_LEGS, leg_clamp='firmware')
    nk = len(q)
    z = [0.0] * ti.NUM_AXES
    frames = []
    for j, rcv in arrivals.items():
        if rcv is None or j + 2 >= nk:
            continue
        u = [list(z) for _ in range(6)]
        for col, val in zip(u, (q[j], q[j + 1], q[j + 2], v[j], v[j + 1], v[j + 2])):
            col[HAND] = float(val)
        frames.append((int(rcv), j, u))
    frames.sort()
    t = t_from_us - ((t_from_us - tick_phase_us) % TICK_US)
    out, fi, fb = [], 0, None
    with (constant_velocity_grace() if cv else contextlib.nullcontext()):
        while t <= t_to_us:
            while fi < len(frames) and frames[fi][0] <= t:
                rcv, j, u = frames[fi]
                stamp = origin_us + j * T_US
                it.sched_on_setpoint(stamp - epoch_us, rcv - epoch_us, *u, has_hand=True,
                                     hand_ff_gain=1.0)
                fi += 1
            r = it.tick_hand((t - epoch_us) * 1e-6, fb_rev=fb if fb is not None else 0.0)
            g = it._sg[1]
            if r is not None:
                fb = r[0]
                playing = (g.frame_t_start_us + epoch_us - origin_us) / T_US if g.active else -1
                out.append((t, r[0], r[1], it.hand_cmd_acc, it.sched_phase(1), playing,
                            it.sched_hand_c1_stops, it.sched_refused, it.sched_stops))
            t += TICK_US
    return np.array(out, dtype=float)


def summarize(run, base, t_lo, t_hi):
    """Command vs the on-time plan inside [t_lo, t_hi]."""
    t = np.intersect1d(run[:, 0], base[:, 0])
    t = t[(t >= t_lo) & (t <= t_hi)]
    r = run[np.isin(run[:, 0], t)]
    b = base[np.isin(base[:, 0], t)]
    dv = np.diff(r[:, 2])
    dvb = np.diff(b[:, 2])
    k = int(np.argmax(np.abs(dv - dvb)))
    return dict(
        peak_vel=float(r[:, 2].max()), peak_pos=float(r[:, 1].max()),
        plan_peak_vel=float(b[:, 2].max()), plan_peak_pos=float(b[:, 1].max()),
        max_dev_vel=float(np.max(np.abs(r[:, 2] - b[:, 2]))),
        max_dev_pos=float(np.max(np.abs(r[:, 1] - b[:, 1]))),
        worst_tick_step_vel=float(dv[k]), plan_tick_change_there=float(dvb[k]),
        worst_tick_t=float(r[k + 1, 0]),
        c1_stops=int(r[-1, 6] - r[0, 6]), refused=int(r[-1, 7] - r[0, 7]),
        stops=int(r[-1, 8] - r[0, 8]))


def load_echo_all(capture):
    """Every echo row of a capture (t_src_us, cmd_pos, cmd_vel_ff, cmd_tor_ff)."""
    return load_echo(capture, 0, 1 << 62)


def _arrivals(origin_us, nk, j_drop, n_drop, d, lead_us, rule, fwd_rcv, nom, tr):
    """{knot j: firmware queue entry us or None}.  Frames other than the drained
    pair arrive ``nom`` after their stamp (inside the previous cover)."""
    j_fwd = j_drop + n_drop
    a = {}
    for j in range(nk - 2):
        st = origin_us + j * T_US
        if j_drop <= j < j_fwd:
            if rule == 'due' and j == j_fwd - 1:
                a[j] = st + int(d['dropped_lag_ms'] * 1e3) + tr - lead_us
            else:
                a[j] = None
        elif j == j_fwd:
            a[j] = fwd_rcv - lead_us
        else:
            a[j] = st + nom - lead_us
    return a


def analyse_drop(echo_all, d, transit_ms=None, nominal_age_ms=6.0, before_knots=16,
                 after_knots=10, detail=False):
    """Fit the knots around one drop (``d``: a parsed drop line) and replay it.

    ``transit_ms`` None = infer the drain -> firmware transit from the echo
    samples inside the hole (the smallest of 0.5..10 ms whose flown replay
    matches them within 0.05 rev/s; 1.0 ms when no echo falls in the hole)."""
    stamp_us = int(round(d['dropped_stamp'] * 1e6))
    drain_us = int(round(d['drain_wall'] * 1e6))
    origin_us = stamp_us - before_knots * T_US
    nk = before_knots + after_knots
    m = (echo_all[:, 0] >= origin_us) & (echo_all[:, 0] <= origin_us + (nk - 1) * T_US)
    echo = echo_all[m]
    if len(echo) < 2 * nk:      # 10 ms echo against 25 ms knots: ~2.5 per knot
        return None
    tick_phase = int(np.median(echo[:, 0] % TICK_US))
    j_drop, n_drop = before_knots, d['n']
    j_fwd = j_drop + n_drop
    cover_end = origin_us + (j_drop - 1) * T_US + 2 * T_US
    hole = (cover_end, drain_us + 12000)           # left out of the fit, predicted
    q, v, fit = fit_knots(echo, origin_us, nk, [hole])
    j_cov = j_drop + 1
    jumps = [x for x in knot_accel_jumps(q, v) if abs(x[0] - j_cov) <= 3]
    jr, a_in, a_out = max(jumps, key=lambda x: abs(x[2] - x[1]))
    epoch = origin_us - 1_000_000
    t_from, t_to = origin_us + T_US, origin_us + (nk - 3) * T_US
    nom = int(nominal_age_ms * 1e3)
    fwd_base = stamp_us + n_drop * T_US + int(d['fwd_lag_ms'] * 1e3)
    win = (cover_end - 3 * T_US, cover_end + 4 * T_US)

    def run(lead_us, rule, tr_us, cv=False, fwd=None):
        fr = (fwd_base if fwd is None else fwd) + tr_us
        return replay(q, v, origin_us, _arrivals(origin_us, nk, j_drop, n_drop, d, lead_us,
                                                 rule, fr, nom, tr_us),
                      t_from, t_to, tick_phase, epoch, cv=cv)

    def hole_residual(r):
        ix = {int(t): i for i, t in enumerate(r[:, 0])}
        worst, n = 0.0, 0
        for (t, p, vv, _tq) in echo:
            if not (hole[0] <= t <= hole[1]):
                continue
            i = ix.get(int(t)) if int(t) in ix else ix.get(int(t) - 1, ix.get(int(t) + 1))
            if i is None:
                continue
            n += 1
            worst = max(worst, abs(r[i, 2] - vv))
        return worst, n

    base = replay(q, v, origin_us, {j: origin_us + j * T_US + nom for j in range(nk - 2)},
                  t_from, t_to, tick_phase, epoch)
    consistent, by_transit = [], []
    for tms in np.arange(0.0, 10.01, 0.5):
        rr = run(0, 'newest', int(tms * 1e3))
        w, n = hole_residual(rr)
        if n and w < 0.05:
            consistent.append(float(tms))
            s_ = summarize(rr, base, *win)
            by_transit.append(dict(transit_ms=float(tms), max_dev_vel=round(s_['max_dev_vel'], 2),
                                   step=round(s_['worst_tick_step_vel']
                                              - s_['plan_tick_change_there'], 2),
                                   at_ms=round((s_['worst_tick_t'] - cover_end) / 1e3, 3),
                                   peak_vel=round(s_['peak_vel'], 2), c1_stops=s_['c1_stops']))
    if transit_ms is None:
        transit_ms = consistent[0] if consistent else 1.0
    tr = int(transit_ms * 1e3)
    scen = {
        'on_time_plan': base,
        'flown_lead0_newest': run(0, 'newest', tr),
        'lead1_newest': run(T_US, 'newest', tr),
        'lead1_due_aware': run(T_US, 'due', tr),
        'lead0_cv_grace': run(0, 'newest', tr, cv=True),
    }
    summ = {k: summarize(r, base, *win) for k, r in scen.items()}
    pl_hole = plan_eval(q, v, origin_us, cover_end)
    res = dict(
        dropped_seq=d['dropped_seq'], dropped_stamp=d['dropped_stamp'],
        dropped_lag_ms=d['dropped_lag_ms'], fwd_seq=d['fwd_seq'], fwd_lag_ms=d['fwd_lag_ms'],
        n_dropped=n_drop, cover_end_wall=cover_end / 1e6, tick_phase_us=tick_phase,
        fit=dict((k, v_) for k, v_ in fit.items() if k != 'used'),
        transit_ms_used=transit_ms, transit_ms_consistent_with_echo=consistent,
        flown_by_consistent_transit=by_transit,
        echo_samples_in_hole=hole_residual(base)[1],
        plan_at_cover_end=dict(pos=pl_hole[0], vel=pl_hole[1], acc=pl_hole[2]),
        release_knot=dict(j=jr, t_rel_ms=(origin_us + jr * T_US - cover_end) / 1e3,
                          q=float(q[jr]), v=float(v[jr]), a_in=a_in, a_out=a_out),
        scenarios=summ)
    if not detail:
        return res
    ix = {int(t): i for i, t in enumerate(scen['flown_lead0_newest'][:, 0])}
    flown = scen['flown_lead0_newest']
    check = []
    for (t, p, vv, tq) in echo:
        i = ix.get(int(t), ix.get(int(t) - 1, ix.get(int(t) + 1)))
        if i is None:
            continue
        pl = plan_eval(q, v, origin_us, t)
        check.append(dict(t_rel_ms=round((t - cover_end) / 1e3, 3), echo_pos=round(p, 4),
                          echo_vel=vv, echo_tor=tq, twin_pos=round(flown[i, 1], 4),
                          twin_vel=round(flown[i, 2], 2), plan_pos=round(pl[0], 4),
                          plan_vel=round(pl[1], 2), in_hole=bool(hole[0] <= t <= hole[1])))
    sweep = []
    for lag in [0, 1, 2, 3, 4, 6, 8, 10, 12, 13, 14, 16, 20, 24, 26, 30]:
        fr = stamp_us + n_drop * T_US + int(lag * 1e3)
        row = dict(fwd_lag_ms=lag)
        for nm, lead, cv in (('lead0', 0, False), ('lead0_cv', 0, True), ('lead1', T_US, False)):
            s = summarize(run(lead, 'newest', tr, cv=cv, fwd=fr), base, *win)
            row[nm] = dict(max_dev_vel=round(s['max_dev_vel'], 2),
                           step=round(s['worst_tick_step_vel'] - s['plan_tick_change_there'], 2),
                           peak_vel=round(s['peak_vel'], 2), peak_pos=round(s['peak_pos'], 3),
                           c1_stops=s['c1_stops'], refused=s['refused'])
        sweep.append(row)
    res.update(knots=[dict(j=j, t_rel_ms=(origin_us + j * T_US - cover_end) / 1e3,
                           q=round(float(q[j]), 4), v=round(float(v[j]), 3)) for j in range(nk)],
               echo_check=check, sweep=sweep, _scen=scen, _echo=echo, _hole=hole, _win=win)
    return res


def run_event(name, out_dir, transit_ms, nominal_age_ms, data_root=REPO):
    ev = EVENTS[name]
    d = parse_drop_line(os.path.join(data_root, ev['log']), ev['dropped_seq'])
    echo_all = load_echo_all(os.path.join(data_root, ev['capture']))
    res = analyse_drop(echo_all, d, transit_ms, nominal_age_ms, detail=True)
    scen, echo, hole, win = (res.pop(k) for k in ('_scen', '_echo', '_hole', '_win'))
    cover_end = int(round(res['cover_end_wall'] * 1e6))
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, '%s_replay.csv' % name), 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['scenario', 't_wall_us', 't_rel_cover_end_ms', 'cmd_pos', 'cmd_vel',
                    'cmd_acc', 'sched_phase', 'playing_knot'])
        for k, r in scen.items():
            for row in r:
                w.writerow([k, int(row[0]), round((row[0] - cover_end) / 1e3, 3),
                            round(row[1], 5), round(row[2], 3), round(row[3], 1),
                            int(row[4]), round(row[5], 3)])
    _plot(name, out_dir, scen, echo, cover_end, win, hole)
    res.update(event=name, drop_line=dict(line=d['line'], text=d['text']),
               assumptions=dict(transit_ms=res['transit_ms_used'],
                                nominal_age_ms=nominal_age_ms))
    return res


def _capture_span(path):
    """(first, last) Jetson wall us of a capture's rows, from its head and tail
    (the echo itself is silent while the Hand is idle, so it cannot bound it)."""
    first = last = None
    with open(path, 'rb') as fh:
        fh.readline()
        p = fh.readline().split(b',')
        if len(p) > 3:
            first = int(float(p[1]) * 1e6)
        fh.seek(0, 2)
        fh.seek(max(0, fh.tell() - 4096))
        for line in fh.read().split(b'\n'):
            p = line.split(b',')
            if len(p) > 3:
                try:
                    last = int(float(p[1]) * 1e6)
                except ValueError:
                    pass
    return first, last


def run_scan(logs, out_dir, data_root=REPO, captures='temp/hand_capture/hand_capture_*.csv',
             transit_ms=None):
    """Every drop line of ``logs`` that a Hand capture covers: fit + replay, and
    whether a Hand fault followed within 0.3 s."""
    caps = []
    for c in sorted(glob.glob(os.path.join(data_root, captures))):
        f, l = _capture_span(c)
        if f is not None and l is not None:
            caps.append((f, l, c))
    paths = sorted({p for pat in logs for p in glob.glob(os.path.join(data_root, pat))})
    rows = []
    for p in paths:
        drops, _atts = scan_log(p)
        faults = [(i, t) for i, t in _hand_fault_lines(p)]
        cur, echo_all = None, None
        for dr in drops:
            if dr['stamp'] is None or dr['flag'] is None:
                continue
            st = int(dr['stamp'] * 1e6)
            cap = next((c for f, l, c in caps if f + 1e6 <= st <= l - 1e6), None)
            if cap is None:
                continue
            if cap != cur:
                cur, echo_all = cap, load_echo_all(cap)
            d = dict(dropped_seq=dr['seqs'], dropped_stamp=dr['stamp'], dropped_lag_ms=dr['dlag'],
                     fwd_seq=None, fwd_lag_ms=dr['flag'], n=dr['n'],
                     drain_wall=dr['stamp'] + dr['dlag'] * 1e-3)
            r = analyse_drop(echo_all, d, transit_ms)
            if r is None:
                continue
            fl = r['scenarios']['flown_lead0_newest']
            fault = next((dict(line=i, dt_s=round(t - d['drain_wall'], 3)) for i, t in faults
                          if 0 <= t - d['drain_wall'] <= 0.3), None)
            rows.append(dict(log=os.path.basename(p), line=dr['line'], seqs=dr['seqs'],
                             n=dr['n'], fwd_lag_ms=dr['flag'], capture=os.path.basename(cap),
                             plan_vel_at_cover_end=round(r['plan_at_cover_end']['vel'], 2),
                             plan_acc_at_cover_end=round(r['plan_at_cover_end']['acc'], 1),
                             transit_ms=r['transit_ms_used'],
                             transit_consistent=r['transit_ms_consistent_with_echo'],
                             fit_vel_max=round(r['fit']['vel_max'], 4),
                             flown_max_dev_vel_range=(
                                 [min(x['max_dev_vel'] for x in r['flown_by_consistent_transit']),
                                  max(x['max_dev_vel'] for x in r['flown_by_consistent_transit'])]
                                 if r['flown_by_consistent_transit'] else None),
                             flown_max_dev_vel=round(fl['max_dev_vel'], 2),
                             flown_step=round(fl['worst_tick_step_vel']
                                              - fl['plan_tick_change_there'], 2),
                             flown_peak_vel=round(fl['peak_vel'], 2),
                             plan_peak_vel=round(fl['plan_peak_vel'], 2),
                             flown_c1_stops=fl['c1_stops'], flown_refused=fl['refused'],
                             lead1_max_dev_vel=round(
                                 r['scenarios']['lead1_newest']['max_dev_vel'], 2),
                             lead1_due_max_dev_vel=round(
                                 r['scenarios']['lead1_due_aware']['max_dev_vel'], 2),
                             cv_max_dev_vel=round(
                                 r['scenarios']['lead0_cv_grace']['max_dev_vel'], 2),
                             hand_fault_within_0_3s=fault))
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, 'scan.json'), 'w') as fh:
        json.dump(rows, fh, indent=1)
    return rows


def _hand_fault_lines(path):
    date = _log_date(path)
    for i, line in _clean_lines(path):
        if 'ODrive error on hand' in line and 'disarm=[]' not in line:
            m = re.match(r'^(\d\d:\d\d:\d\d\.\d+) ', line)
            if m:
                yield i, _hms_epoch(date, m.group(1))


def _plot(name, out_dir, scen, echo, cover_end, win, hole):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        return
    fig, ax = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    for k, r in scen.items():
        m = (r[:, 0] >= win[0]) & (r[:, 0] <= win[1])
        x = (r[m, 0] - cover_end) / 1e3
        ls = '-' if k != 'on_time_plan' else '--'
        ax[0].plot(x, r[m, 2], ls, label=k, lw=1.2)
        ax[1].plot(x, r[m, 1], ls, label=k, lw=1.2)
    m = (echo[:, 0] >= win[0]) & (echo[:, 0] <= win[1])
    x = (echo[m, 0] - cover_end) / 1e3
    ax[0].plot(x, echo[m, 2], 'ko', ms=4, label='echo (flown)')
    ax[1].plot(x, echo[m, 1], 'ko', ms=4)
    for a in ax:
        a.axvspan((hole[0] - cover_end) / 1e3, (hole[1] - cover_end) / 1e3, color='0.9')
        a.axvline(0, color='0.5', lw=0.8)
        a.grid(alpha=0.3)
    ax[0].set_ylabel('Hand command velocity (rev/s)')
    ax[1].set_ylabel('Hand command position (rev)')
    ax[1].set_xlabel('ms from the last forwarded frame\'s cover end')
    ax[0].legend(fontsize=7)
    ax[0].set_title('%s: 500 Hz Hand command across the release knot (twin replay)' % name)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, '%s_command.png' % name), dpi=110)
    plt.close(fig)


# ── census ───────────────────────────────────────────────────────────────────

def scan_bag_rx_age(bag_dirs):
    from mcap_ros2.reader import read_ros2_messages
    rows = []
    for bd in bag_dirs:
        for f in sorted(glob.glob(os.path.join(os.path.expanduser(bd), '*.mcap'))):
            for m in read_ros2_messages(f, topics=['/link_status']):
                kv = {x.key: x.value for x in m.ros_msg.values}
                if 'rx_age_le25' not in kv:
                    continue
                try:
                    rows.append((m.log_time_ns * 1e-9,
                                 None if kv.get('rx_age_max_us') in (None, 'none')
                                 else int(kv['rx_age_max_us']),
                                 int(kv['rx_age_le25']), int(kv['rx_age_le50']),
                                 int(kv['rx_age_le70']), int(kv['rx_age_gt70'])))
                except ValueError:
                    continue
    return rows


def run_census(logs, out_dir, bags=(), data_root=REPO):
    paths = sorted({p for pat in logs for p in glob.glob(os.path.join(data_root, pat)
                                                            if not os.path.isabs(pat) else pat)})
    all_drops, all_atts = [], []
    for p in paths:
        dr, at = scan_log(p)
        all_drops += dr
        all_atts += at
    for d in all_drops:
        d['attempt'] = next((k for k, a in enumerate(all_atts)
                             if a['t0'] <= d['t'] <= a['t1']), None)
    in_att = [d for d in all_drops if d['attempt'] is not None]
    att_s = sum(a['t1'] - a['t0'] for a in all_atts)

    def pct(xs, ps=(0, 50, 90, 99, 100)):
        return {('p%d' % p): round(float(np.percentile(xs, p)), 1) for p in ps} if xs else {}
    fl = [d['flag'] for d in in_att if d['flag'] is not None]
    dl = [d['dlag'] for d in in_att if d['dlag'] is not None]
    holes = {}
    for lead, rule in ((0, 'newest'), (1, 'newest'), (1, 'due')):
        hs = [hole_ms(d['n'], d['flag'], lead, rule) for d in in_att if d['flag'] is not None]
        holes['lead%d_%s' % (lead, rule)] = dict(
            n=len(hs), n_hole_gt0=sum(h > 0 for h in hs), n_hole_gt_c1_grace_14=sum(h > 14 for h in hs),
            **pct([h for h in hs if h > 0]))
    per_att = []
    for k, a in enumerate(all_atts):
        ds = [d for d in in_att if d['attempt'] == k]
        per_att.append(dict(log=a['log'], line=a['line'], pattern=a['pattern'], end=a['end'],
                            minutes=round((a['t1'] - a['t0']) / 60, 2), drop_lines=len(ds),
                            frames_dropped=sum(d['n'] for d in ds),
                            suppressed_more=sum(d['more'] for d in ds),
                            fwd_lags=[d['flag'] for d in ds]))
    res = dict(
        logs=[os.path.basename(p) for p in paths], attempts=len(all_atts),
        attempt_minutes=round(att_s / 60, 1), drop_lines_total=len(all_drops),
        drop_lines_in_attempts=len(in_att),
        frames_dropped_in_attempts=sum(d['n'] for d in in_att),
        multi_frame_drains=sum(d['n'] > 1 for d in in_att),
        throttled_more_in_attempts=sum(d['more'] for d in in_att),
        dropped_lag_ms=pct(dl), forwarded_lag_ms=pct(fl),
        forwarded_lag_hist={'<=5': sum(x <= 5 for x in fl), '5-10': sum(5 < x <= 10 for x in fl),
                            '10-14': sum(10 < x <= 14 for x in fl),
                            '14-25': sum(14 < x <= 25 for x in fl), '>25': sum(x > 25 for x in fl)},
        holes=holes,
        attempts_with_drops=sum(1 for a in per_att if a['drop_lines']),
        drops_per_attempt=pct([a['frames_dropped'] for a in per_att]),
        per_attempt=per_att,
        drops=[dict((k, d[k]) for k in ('log', 'line', 'n', 'seqs', 'dlag', 'flag', 'more',
                                        'attempt')) for d in all_drops])
    if bags:
        rx = scan_bag_rx_age(bags)
        inw = [r for r in rx if any(a['t0'] <= r[0] <= a['t1'] for a in all_atts)]
        tot = [sum(r[i] for r in inw) for i in (2, 3, 4, 5)]
        mx = [r[1] for r in inw if r[1] is not None]
        res['bag_rx_age'] = dict(
            bags=list(bags), heartbeats_in_attempts=len(inw),
            frames_le25=tot[0], frames_25_50=tot[1], frames_50_70=tot[2], frames_gt70=tot[3],
            window_max_ms=pct([x / 1e3 for x in mx], (50, 90, 99, 100)),
            windows_max_gt25ms=sum(x > 25000 for x in mx))
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, 'census.json'), 'w') as fh:
        json.dump(res, fh, indent=1)
    return res


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    r = sub.add_parser('replay')
    r.add_argument('--event', choices=sorted(EVENTS), required=True)
    r.add_argument('--transit-ms', type=float, default=None,
                   help='bridge drain -> firmware queue entry (default: inferred from '
                        'the echo samples inside the hole, else 1.0)')
    r.add_argument('--nominal-age-ms', type=float, default=6.0,
                   help='firmware age of every other frame (inside the cover; default 6)')
    c = sub.add_parser('census')
    c.add_argument('--logs', nargs='+', required=True)
    c.add_argument('--bags', nargs='*', default=())
    sc = sub.add_parser('scan')
    sc.add_argument('--logs', nargs='+', required=True)
    sc.add_argument('--transit-ms', type=float, default=None)
    for p in (r, c, sc):
        p.add_argument('--data-root', default=REPO,
                       help='checkout whose temp/ holds the captures and logs (default: this one)')
        p.add_argument('--out', default=os.path.join(
            REPO, 'temp', 'probes', 'hand_late_knot_replay',
            _dt.datetime.now().strftime('%Y%m%d_%H%M%S')))
    a = ap.parse_args(argv)
    if a.cmd == 'replay':
        res = run_event(a.event, a.out, a.transit_ms, a.nominal_age_ms, a.data_root)
        with open(os.path.join(a.out, '%s_summary.json' % a.event), 'w') as fh:
            json.dump(res, fh, indent=1)
    elif a.cmd == 'scan':
        res = run_scan(a.logs, a.out, a.data_root, transit_ms=a.transit_ms)
    else:
        res = run_census(a.logs, a.out, a.bags, a.data_root)
        res = dict((k, v) for k, v in res.items() if k not in ('per_attempt', 'drops'))
    print(json.dumps(res, indent=1))


if __name__ == '__main__':
    main()
