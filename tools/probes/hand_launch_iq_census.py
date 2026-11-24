#!/usr/bin/env python3
"""Per-Throw Hand current census from full-rate Hand captures + launch logs (BUG-40/41/51).

Offline and read-only. For every ``temp/hand_capture/hand_capture_<ts>.csv`` it
  * finds the launch log (``~/.ros/log/<ts>-*/launch.log``) with the same start time,
  * parses Attempt dispatches (pattern, Apex, Dwell, turned angle from the
    nomination line) and the planner's ``install_segment THROW`` peak acceleration,
  * detects Hand strokes (velocity peak >= --min-peak-rps), splits each into an ASCENT
    (last rest before the peak -> the peak) and a BRAKE (peak -> back under 8 rev/s),
  * records the peak iq_measured / iq_setpoint of each phase and the jitter-robust
    integral of iq_measured over each phase (the input of the torque-constant fit),
  * flags strokes near a trapped-Ball stall (|iq_setpoint| >= 62 A with the Hand stopped,
    BUG-84) or a SPINOUT_DETECTED line, so the stats can exclude them.

Sampling: the capture's diag rows are the ODrive iq message (9 ms period since the
2026-11-08 odd-rate change, 10 ms before) as it arrived; /hand_telemetry in a bag carries
the same updates held on a 100 Hz grid. The capture is NOT a higher-rate iq source than
the bag: it adds iq_setpoint and loses nothing to the grid. A peak narrower than the
9-10 ms message period can fall between samples; ``n_asc`` and ``dt_asc_ms`` say how well
each ascent was sampled.

  python tools/probes/hand_launch_iq_census.py strokes   # writes temp/probes/hand_iq_strokes_<ts>.csv
  python tools/probes/hand_launch_iq_census.py report STROKES.csv
  python tools/probes/hand_launch_iq_census.py fit STROKES.csv
  python tools/probes/hand_launch_iq_census.py hold [BAGDIR ...]   # rest iq by ball_held (bags)

Requires numpy and pandas (the project venv); ``hold`` also mcap + mcap_ros2.
"""
from __future__ import annotations

import argparse
import glob
import os
import re
import sys
import time

import numpy as np
import pandas as pd

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
TEMP = os.environ.get('JUGGLEBOT_TEMP', os.path.join(REPO, 'temp'))  # a worktree has no temp/: point this at the main checkout's
CAP_DIR = os.path.join(TEMP, 'hand_capture')
OUT_DIR = os.environ.get('HAND_IQ_OUT', os.path.join(TEMP, 'probes'))
LOG_ROOT = os.path.expanduser('~/.ros/log')
BAG_ROOT = os.path.expanduser('~/Desktop/rosbags')

RE_DISPATCH = re.compile(
    r"goal dispatched \(pattern='(\w+)'(?:, apex_m=([\d.]+))?(?:, dwell_s=([\d.]+))?"
    r"(?:, num_cycles=(\d+))?(?:, separation_mm=([\d.]+))?")
RE_ANGLE = re.compile(r"columns_angle_deg=(\d+)|angle (\d+) deg|@(\d+)")
RE_THROW = re.compile(r"install_segment THROW Ball (\d): splice_k=(\d+).*?peaks hand acc (\d+) rev/s\^2")
ONE_BALL = ('self_toss', 'hop', 'oval_1ball', 'oval_1ball_fed', 'columns_1ball_fed')


def parse_log(path):
    """-> dict(dispatch=[...], throws=[(t, ball, splice_k, acc)], spinout=[t], limit_acc=[t])."""
    out = dict(dispatch=[], throws=[], spinout=[], limit_acc=[])
    pend = None
    last_angle, last_angle_t = 0.0, -1e9
    with open(path, errors='replace') as fh:
        for ln in fh:
            sp = ln.find(' ')
            try:
                t = float(ln[:sp])
            except ValueError:
                continue
            if 'skill_node' in ln and ('nominat' in ln):
                m = RE_ANGLE.search(ln)
                if m:
                    last_angle = float(next(g for g in m.groups() if g))
                    last_angle_t = t
            if pend is not None and pend['pattern'] == 'columns_turned' and t - pend['t'] < 10.0 and 'skill_node' in ln and 'nomination OK' in ln:
                m = RE_ANGLE.search(ln)
                if m:
                    pend['angle'] = float(next(g for g in m.groups() if g))
                    pend['angle_age_s'] = 0.0
                    pend = None
            m = RE_DISPATCH.search(ln)
            if m:
                pend = dict(t=t, pattern=m.group(1),
                            apex=float(m.group(2)) if m.group(2) else np.nan,
                            dwell=float(m.group(3)) if m.group(3) else np.nan,
                            cycles=int(m.group(4)) if m.group(4) else 0,
                            sep=float(m.group(5)) if m.group(5) else np.nan, angle=0.0, angle_age_s=np.nan)
                if m.group(1) == 'columns_turned':
                    pend['angle'], pend['angle_age_s'] = last_angle, t - last_angle_t
                out['dispatch'].append(pend)
                continue
            if 'install_segment THROW' in ln:
                m = RE_THROW.search(ln)
                if m:
                    out['throws'].append((t, int(m.group(1)), int(m.group(2)), float(m.group(3))))
            elif 'SPINOUT_DETECTED' in ln:
                out['spinout'].append(t)
            elif 'HAND_LIMIT_ACC' in ln:
                out['limit_acc'].append(t)
    return out


def find_log(cap_path):
    """The launch log whose directory start time is the latest one within 15 s before/after the capture's."""
    import datetime as dt
    stamp = re.search(r'hand_capture_(\d{8})_(\d{6})', cap_path)
    t0 = dt.datetime.strptime(stamp.group(1) + stamp.group(2), '%Y%m%d%H%M%S')
    best = None
    for d in glob.glob(os.path.join(LOG_ROOT, t0.strftime('%Y-%m-%d-') + '*')):
        try:
            td = dt.datetime.strptime(os.path.basename(d)[:19], '%Y-%m-%d-%H-%M-%S')
        except ValueError:
            continue
        if abs((td - t0).total_seconds()) <= 15 and os.path.exists(os.path.join(d, 'launch.log')):
            if best is None or abs((td - t0).total_seconds()) < best[0]:
                best = (abs((td - t0).total_seconds()), os.path.join(d, 'launch.log'))
    return best[1] if best else None


def strokes_of(cap_path, min_peak=60.0):
    d = pd.read_csv(cap_path)
    tl = d[d.kind == 'telem']
    dg = d[d.kind == 'diag'].dropna(subset=['iq_measured'])
    tw, v = tl.t_jetson_wall_s.values, tl.vel_rps.values
    gw, im, isp = dg.t_jetson_wall_s.values, dg.iq_measured.values, dg.iq_setpoint.values
    stall_t = []
    if len(gw):
        vi = np.interp(gw, tw, v)
        stall_t = gw[(np.abs(isp) >= 62.0) & (np.abs(vi) < 3.0)]
    # candidate peaks: local maxima of v above min_peak, separated by > 0.15 s
    idx = np.where((v[1:-1] >= min_peak) & (v[1:-1] >= v[:-2]) & (v[1:-1] > v[2:]))[0] + 1
    rows, last = [], -9.0
    for i in idx:
        if tw[i] - last < 0.15:
            continue
        last = tw[i]
        j = i
        while j > 0 and v[j] > 3.0 and tw[i] - tw[j] < 0.6:
            j -= 1
        k = i
        while k < len(v) - 1 and v[k] > 8.0 and tw[k] - tw[i] < 0.5:
            k += 1
        t0, tp, t1 = tw[j], tw[i], tw[k]
        a = (gw >= t0) & (gw <= tp + 0.010)
        b = (gw > tp + 0.010) & (gw <= t1)
        if a.sum() < 3:
            continue
        r = dict(cap=os.path.basename(cap_path), t_start=t0, t_peak=tp, t_end=t1, v_peak=v[i], T_asc=tp - t0,
                 n_asc=int(a.sum()), dt_asc_ms=1e3 * np.mean(np.diff(gw[a])) if a.sum() > 1 else np.nan,
                 asc_iq_meas=im[a].max(), asc_iq_set=np.nanmax(isp[a]),
                 I_asc=float(np.trapz(im[a], gw[a])), dv_asc=v[i] - v[j], T_asc_samp=gw[a][-1] - gw[a][0])
        if b.sum() >= 3:
            r.update(brk_iq_meas=im[b].min(), brk_iq_set=np.nanmin(isp[b]), n_brk=int(b.sum()),
                     I_brk=float(np.trapz(im[b], gw[b])), dv_brk=v[i] - v[k], T_brk_samp=gw[b][-1] - gw[b][0])
        r['stall_near'] = bool(len(stall_t) and np.min(np.abs(stall_t - tp)) < 1.5)
        rows.append(r)
    return rows


def cmd_strokes(a):
    allrows = []
    for cap in sorted(glob.glob(os.path.join(CAP_DIR, 'hand_capture_*.csv'))):
        lg = find_log(cap)
        rows = strokes_of(cap, a.min_peak)
        info = parse_log(lg) if lg else dict(dispatch=[], throws=[], spinout=[], limit_acc=[])
        disp = info['dispatch']
        spin = np.array(info['spinout'])
        # wall time of a stroke = telem wall time (same epoch as the log prefixes)
        for r in rows:
            r['log'] = os.path.basename(os.path.dirname(lg)) if lg else ''
            cur, idx_ = None, 0
            for n, dsp in enumerate(disp):
                if dsp['t'] <= r['t_start']:
                    cur, idx_ = dsp, n
            if cur is not None and r['t_start'] - cur['t'] <= a.window_s:
                r.update(pattern=cur['pattern'], apex=cur['apex'], dwell=cur['dwell'], sep=cur['sep'],
                         angle=cur['angle'], angle_age_s=cur['angle_age_s'], disp_t=cur['t'], disp_id='%s#%d' % (r['cap'], idx_),
                         balls=1 if cur['pattern'] in ONE_BALL else 2)
            else:
                r.update(pattern='(none)', balls=0, disp_id='')
            r['spinout_near'] = bool(len(spin) and np.min(np.abs(spin - r['t_peak'])) < 4.0)
        # stroke order inside an Attempt, planned acc for the first
        order = {}
        for r in rows:
            order[r['disp_id']] = order.get(r['disp_id'], 0) + 1
            r['k'] = order[r['disp_id']]
            r['plan_acc'] = np.nan
            if r['k'] == 1 and r['disp_id']:
                th = [x for x in info['throws'] if x[0] >= r['disp_t']]
                if th:
                    r['plan_acc'] = th[0][3]
        allrows += rows
        print('%s: %d strokes (%d attempts)' % (os.path.basename(cap), len(rows), len(order)), file=sys.stderr)
    df = pd.DataFrame(allrows)
    os.makedirs(OUT_DIR, exist_ok=True)
    out = a.out or os.path.join(OUT_DIR, 'hand_iq_strokes_%s.csv' % time.strftime('%Y%m%d_%H%M%S'))
    df.to_csv(out, index=False)
    print(out)


def _stats(x):
    x = np.asarray(x, float)
    x = x[~np.isnan(x)]
    if not len(x):
        return 'n=0'
    return 'n=%d med=%.1f p95=%.1f max=%.1f' % (len(x), np.median(x), np.percentile(x, 95), x.max())


def cmd_report(a):
    df = pd.read_csv(a.csv)
    df = df[(df.pattern != '(none)') & ~df.stall_near & ~df.spinout_near]
    df['apex_b'] = df.apex.round(2)
    for label, sub in (('LAUNCH (k==1)', df[df.k == 1]), ('ALL strokes', df)):
        print('\n== %s: peak ascent |iq| (A), measured / setpoint ==' % label)
        for key, g in sub.groupby(['balls', 'pattern', 'angle', 'apex_b']):
            print('balls=%d %-16s ang=%3.0f apex=%.2f  meas[%s]  set[%s]  n_asc_med=%d dt=%.1fms' % (
                key[0], key[1], key[2], key[3], _stats(g.asc_iq_meas), _stats(g.asc_iq_set),
                g.n_asc.median(), g.dt_asc_ms.median()))


def _impulse(gw, im, t_a, t_b):
    """Integral of iq_measured over [t_a, t_b] with the edges linearly interpolated (no edge loss)."""
    m = (gw > t_a) & (gw < t_b)
    tt = np.r_[t_a, gw[m], t_b]
    yy = np.r_[np.interp(t_a, gw, im), im[m], np.interp(t_b, gw, im)]
    return float(np.trapz(yy, tt))


def fit_rows(cap_path, delta_s, min_peak=70.0):
    """Per stroke: ascent and brake impulses of iq_measured, edges at the VELOCITY events shifted by delta_s
    (iq leads the encoder velocity: the window of iq that produced a velocity change ends earlier)."""
    d = pd.read_csv(cap_path)
    tl = d[d.kind == 'telem']
    dg = d[d.kind == 'diag'].dropna(subset=['iq_measured'])
    tw, v = tl.t_jetson_wall_s.values, tl.vel_rps.values
    gw, im = dg.t_jetson_wall_s.values, dg.iq_measured.values
    rows = []
    for r in strokes_of(cap_path, min_peak):
        i = int(np.argmin(np.abs(tw - r['t_peak'])))
        j = int(np.argmin(np.abs(tw - r['t_start'])))
        k = int(np.argmin(np.abs(tw - r['t_end'])))
        rows.append(dict(cap=r['cap'], t_peak=r['t_peak'], flag=r['stall_near'],
                         dv_a=v[i] - v[j], T_a=tw[i] - tw[j], I_a=_impulse(gw, im, tw[j] - delta_s, tw[i] - delta_s),
                         dv_b=v[i] - v[k], T_b=tw[k] - tw[i], I_b=_impulse(gw, im, tw[i] - delta_s, tw[k] - delta_s),
                         v_peak=v[i]))
    return rows


def cmd_fit(a):
    """c1 (A per rev/s^2) of the ascent (ball aboard) and the brake (ball released), per lag delta and per
    assumed c0 (the hold+friction offset, A): c1 = (impulse - c0*T) / dv. Prints the median and the stroke-to-stroke
    spread; the fit is NOT a free 2-parameter regression because dv and T barely vary across strokes."""
    df = pd.read_csv(a.csv)
    caps = sorted(set(df.cap))
    st = df[(df.pattern != '(none)') & ~df.stall_near & ~df.spinout_near]
    keep = set(zip(st.cap, st.t_peak.round(3)))
    for delta in (0.0, 0.005, 0.010, 0.015, 0.020):
        rows = []
        for c in caps:
            rows += [r for r in fit_rows(os.path.join(CAP_DIR, c), delta) if (r['cap'], round(r['t_peak'], 3)) in keep]
        g = pd.DataFrame(rows)
        g = g[(g.v_peak > 100) & (g.dv_b > 80) & (g.T_b > 0.02)]
        print('delta %.0f ms  n=%d' % (delta * 1e3, len(g)))
        for c0 in (0.0, 2.0, 4.0):
            ca = (g.I_a - c0 * g.T_a) / g.dv_a
            cb = -(g.I_b - c0 * g.T_b) / g.dv_b
            print('  c0=%.0f A  c1_ascent %.5f [IQR %.5f..%.5f]  c1_brake %.5f [IQR %.5f..%.5f]' % (
                c0, ca.median(), ca.quantile(.25), ca.quantile(.75), cb.median(), cb.quantile(.25), cb.quantile(.75)))
        g.to_csv(os.path.join(OUT_DIR, 'hand_iq_fit_delta%02d.csv' % int(delta * 1e3)), index=False)


def cmd_hold(a):
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
    dirs = a.bags or sorted(glob.glob(os.path.join(BAG_ROOT, '2026-11-*')))
    acc, slow = [], []
    for d in dirs:
        for f in sorted(glob.glob(os.path.join(d, '*.mcap'))):
            rows = []
            with open(f, 'rb') as fh:
                rd = make_reader(fh, decoder_factories=[DecoderFactory()])
                for _s, _c, _m, msg in rd.iter_decoded_messages(topics=['/hand_telemetry']):
                    ts = msg.timestamp
                    rows.append((ts.sec + ts.nanosec * 1e-9, msg.iq_meas, msg.vel_meas, msg.pos_meas,
                                 msg.ball_held, msg.ball_held_valid))
            if not rows:
                continue
            b = pd.DataFrame(rows, columns=['t', 'iq', 'v', 'pos', 'held', 'valid'])
            # stationary for >= 0.3 s: |v| < 0.3 over a 30-sample window
            still = (b.v.abs() < 0.3).rolling(30).min().fillna(0).astype(bool)
            s = b[still & b.valid]
            s = s.assign(bag=os.path.basename(d))
            acc.append(s)
            # slow-motion samples (Coulomb friction): 0.3 < |v| < 3 rev/s, |dv/dt| < 40 rev/s^2, ball state valid
            dv = b.v.diff().abs() / b.t.diff().clip(lower=1e-3)
            sl = b[(b.v.abs() > 0.3) & (b.v.abs() < 3.0) & (dv < 40) & b.valid & (b.pos > 0.0) & (b.pos < 10.0)]
            slow.append(sl.assign(bag=os.path.basename(d)))
            print(os.path.basename(d), len(b), len(s), file=sys.stderr)
    s = pd.concat(acc)
    s['posband'] = np.where((s.pos > 0.03) & (s.pos < 0.10), 'rest', 'other')
    for key, g in s.groupby(['held', 'posband']):
        print('ball_held=%s pos=%s  n=%d  iq median %.2f  IQR [%.2f, %.2f]  bags=%d' % (
            key[0], key[1], len(g), g.iq.median(), g.iq.quantile(.25), g.iq.quantile(.75), g.bag.nunique()))
    sl = pd.concat(slow)
    for key, g in sl.groupby([sl.held, np.sign(sl.v)]):
        print('SLOW ball_held=%s dir=%+d n=%d iq median %.2f IQR [%.2f, %.2f]' % (
            key[0], key[1], len(g), g.iq.median(), g.iq.quantile(.25), g.iq.quantile(.75)))
    out = os.path.join(OUT_DIR, 'hand_hold_iq_%s.csv' % time.strftime('%Y%m%d_%H%M%S'))
    os.makedirs(OUT_DIR, exist_ok=True)
    s.to_csv(out, index=False)
    print(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('strokes')
    p.add_argument('--min-peak', type=float, default=60.0)
    p.add_argument('--window-s', type=float, default=150.0)
    p.add_argument('--out')
    p.set_defaults(fn=cmd_strokes)
    for nm, fn in (('report', cmd_report), ('fit', cmd_fit)):
        p = sub.add_parser(nm)
        p.add_argument('csv')
        p.set_defaults(fn=fn)
    p = sub.add_parser('hold')
    p.add_argument('bags', nargs='*')
    p.set_defaults(fn=cmd_hold)
    a = ap.parse_args()
    a.fn(a)


if __name__ == '__main__':
    main()
