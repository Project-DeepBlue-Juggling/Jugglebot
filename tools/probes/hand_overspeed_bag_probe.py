#!/usr/bin/env python3
"""Per hand THROW-stroke overspeed: how much faster the hand and ball leave
than commanded.

This is the measurement behind the 2026-09-14 apex ladder
(`logbook/2026-09-14-skill-stack-r3-apex-ladder-prep.md`). The finding it
produced: the streamed hand lane sends zero torque
feedforward (``leg_interp.cpp:1021`` passes a literal ``0.0f``), so overspeed
beyond the command delay GROWS with commanded acceleration — unlike the
retired Platform-Teensy engine, whose ~68% torque feedforward held overspeed
flat across the whole commanded range (see ``agent_hand_overspeed.md``
working notes, folded into the logbook entry above).

TIMING CAVEAT — read before trusting any column with "(adv)" in its header
-----------------------------------------------------------------------
``/hand_telemetry`` is timing-unreliable by construction, discovered
2026-09-14. The can-bridge coalesces its 500 Hz hand command echo down to
one frame per 100 Hz telemetry tick
(``Teensy_code_canbridge/telemetry.cpp:230-280``), and
``teensy_bridge_node._on_hand_cmd_echo`` discards the echo's own
``t_bridge_us`` (``ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py:2157-2172``);
``_publish_hand_telemetry`` stamps the message with the JETSON's own 100 Hz
poll clock instead (``teensy_bridge_node.py:3486``). Consequence:
inter-sample intervals actually seen on the wire run 2-20+ ms, not a clean
10 ms — so any quantity derived from *consecutive-sample dt* is a sampling
artifact, not a measurement of the command or the plant.

  * PRIMARY (timing-robust) columns: cmd/meas peak rev/s, meas/cmd, peak
    |iq|, and the ball-channel columns (mocap-timed, independent of the
    telemetry clock). These are the columns to read.
  * ADVISORY (timing-sensitive) columns, headers suffixed ``(adv)``: fitted
    command delay, overspeed beyond delay, delay-compensated lag, mean ramp
    acceleration, and ESPECIALLY peak 10 ms commanded acceleration. Do not
    read "peak 10 ms commanded acceleration" as evidence of a real command
    surge — it is the coalescing artifact above, not a controller event.
    They are kept (and validated against the scratch prototype) because the
    delay/overspeed numbers were the ones that first surfaced the
    zero-torque-FF finding, but they describe telemetry jitter as much as
    plant behaviour.

WHAT IT MEASURES, PER STROKE
-----------------------------
A stroke is one commanded hand velocity peak above ``--min-peak-rps``
(default 60 rev/s). Columns, in the primary/advisory order above:

  ``t_peak``            commanded-peak time, ROS epoch (s)
  ``cmd_apex_m``         commanded apex height (m) = g*tof^2/8, from the
                         nearest ``/throw_announcements`` ``predicted_tof_sec``
                         within the match window; blank if none
  ``cmd_peak_rps``       commanded velocity at the peak (rev/s) — see
                         ``--cmd-source``
  ``meas_peak_rps``      measured velocity peak (rev/s), 3-point parabolic
                         refinement of the sampled maximum
  ``meas_over_cmd``      ``meas_peak_rps / cmd_peak_rps``
  ``peak_iq_a``          max |iq_meas| (A) over the stroke
  ``ball_v_mps``         ball launch speed (m/s), least-squares ballistic
                         fit of z(t) over the RISE (post-cup, pre-apex
                         samples only); blank if no usable ``/balls`` track
  ``ball_apex_m``        ball apex height above the release height (m)
  ``ball_over_cmd``      ``ball_v_mps / (cmd_peak_rps * mm_per_rev / 1000)``
  ``ball_over_meas_gain`` ``ball_v_mps / (meas_peak_rps * mm_per_rev / 1000)``
                         — "does the ball follow the encoder?"
  ``cmd_delay_ms_adv``          fitted pure command->measured delay (ms),
                         RMS-min grid search over 0-30 ms on the ramp
  ``overspeed_rps_adv``         max residual of measured vel over the
                         delay-shifted command, across the whole stroke
  ``lag_delay_comp_rev_adv``    max delay-compensated position lag
                         (pos_cmd delayed by the fitted delay, minus
                         pos_meas) on the up-ramp
  ``acc_mean_rps2_adv``         mean 10%-90%-of-peak ramp acceleration
  ``peak_cmd_acc10ms_rps2_adv`` max sample-to-sample commanded acceleration
                         on the ramp — READ AS SAMPLING ARTIFACT, see caveat

Strictly offline and read-only (opens ``.mcap`` only via ``mcap``/``mcap_ros2``,
no node, no socket, no hardware; needs no ROS2 runtime).

Usage
-----
    python tools/probes/hand_overspeed_bag_probe.py --bag <dir> [options]

    --bag PATH             rosbag directory (bare name resolves under
                            ~/Desktop/rosbags; a full/relative path is used
                            as-is)
    --since / --until      ROS epoch seconds window (optional)
    --cmd-source            telemetry (default) | announcement
    --mm-per-rev FLOAT      hand encoder gain, mm of tip travel per
                            revolution (default 32.567)
    --min-peak-rps FLOAT    stroke-detection threshold (default 60.0)
    --out PATH              CSV path (default
                            temp/probes/hand_overspeed_<bag basename>.csv)

Prints a markdown table to stdout and writes the same rows to ``--out``.
Deterministic: no randomness, rows sorted by ``t_peak``.

Validated 2026-09-14 (reproduces the scratch prototype's numbers, see
``logbook`` entry above for the finding these numbers support):
  * ``--bag ~/Desktop/rosbags/2026-09-13_22-57-18``: three streamed strokes
    reproduce cmd/meas/delay/overspeed/iq within tolerance of the scratch
    ``hand_overspeed_v2.py`` run (t~...4511.33/4521.96/4572.03).
  * ``--bag ~/Desktop/rosbags/2026-08-23_19-14-54 --cmd-source announcement
    --mm-per-rev 31.628``: Teensy-era tiers score meas/cmd ~0.95-1.02, peak
    iq 10-28 A.
"""

from __future__ import annotations

import argparse
import csv
import glob
import os
import sys
from datetime import datetime

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))          # tools/probes
_REPO = os.path.dirname(os.path.dirname(_HERE))              # repo root
OUT_DIR = os.path.join(_REPO, 'temp', 'probes')
DEFAULT_BAG_ROOT = os.path.expanduser('~/Desktop/rosbags')

G_MM_S2 = 9810.0     # mm/s^2
G_M_S2 = 9.81        # m/s^2

_MIN_STROKE_SEP_S = 0.3     # min separation between distinct strokes
_RAMP_BACK_S = 0.4          # how far back to look for the ramp start
_TAIL_S = 0.2                # how far past the peak the window extends
_ANN_MATCH_S = 0.3           # announcement<->stroke match window
_DELAY_GRID_S = np.arange(0.0, 0.0301, 0.001)   # 0-30 ms, 1 ms steps


def parse_bag_dir(spec: str) -> str:
    """Accept a bare bag name (resolved under ~/Desktop/rosbags) or a path."""
    if os.path.isdir(spec):
        return spec
    return os.path.join(DEFAULT_BAG_ROOT, spec)


def _reader(path):
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
    return make_reader(open(path, 'rb'), decoder_factories=[DecoderFactory()])


def read_bag(bag_dir, since=None, until=None):
    """Return (hand: list of tuples, ann: list of tuples, balls: {id: array})."""
    hand, ann, balls = [], [], {}
    for path in sorted(glob.glob(os.path.join(bag_dir, '*.mcap'))):
        r = _reader(path)
        topics = ['/hand_telemetry', '/throw_announcements', '/balls']
        for _schema, ch, m, msg in r.iter_decoded_messages(topics=topics):
            tl = m.log_time * 1e-9
            if since is not None and tl < since - 2.0:
                continue
            if until is not None and tl > until + 2.0:
                continue
            if ch.topic == '/hand_telemetry':
                ts = msg.timestamp.sec + msg.timestamp.nanosec * 1e-9
                hand.append((ts, tl, msg.pos_cmd, msg.vel_ff_cmd, msg.pos_meas,
                             msg.vel_meas, msg.iq_meas))
            elif ch.topic == '/throw_announcements':
                tt = msg.throw_time.sec + msg.throw_time.nanosec * 1e-9
                ann.append((tt, float(msg.initial_velocity.z),
                            float(msg.initial_position.z),
                            float(msg.predicted_tof_sec)))
            else:  # /balls
                for b in msg.balls:
                    balls.setdefault(int(b.id), []).append(
                        (tl, float(b.position.x), float(b.position.y),
                         float(b.position.z)))
    for k in balls:
        balls[k] = np.array(sorted(balls[k]))
    return hand, ann, balls


def choose_time_column(H):
    """Header timestamp if it looks like a real, regular clock; else log time."""
    ts, tl = H[:, 0], H[:, 1]
    dts = np.diff(ts)
    if len(dts) and np.median(dts) > 0.005 and \
            np.percentile(np.abs(dts - np.median(dts)), 90) < 0.004:
        return ts, 'header'
    return tl, 'log'


def dedupe(H):
    """Drop repeated samples (identical pos_cmd/vel_ff/pos_meas/vel_meas/iq)."""
    if len(H) < 2:
        return H
    keep = np.ones(len(H), bool)
    keep[1:] = np.any(np.diff(H[:, 2:7], axis=0) != 0, axis=1)
    return H[keep]


def find_peaks(t, vff, min_peak_rps, min_sep_s=_MIN_STROKE_SEP_S):
    pk = []
    for i in range(1, len(t) - 1):
        if vff[i] > min_peak_rps and vff[i] >= vff[i - 1] and vff[i] > vff[i + 1]:
            if not pk or t[i] - t[pk[-1]] > min_sep_s:
                pk.append(i)
            elif vff[i] > vff[pk[-1]]:
                pk[-1] = i
    return pk


def match_announcement(ann, t_peak, window_s=_ANN_MATCH_S):
    cands = [a for a in ann if abs(a[0] - t_peak) < window_s]
    if not cands:
        return None
    return min(cands, key=lambda a: abs(a[0] - t_peak))


def fit_ball(balls, t_peak, zr):
    """Best matching /balls track for the stroke at t_peak; ballistic fit
    over the rise. Returns dict(v_mps, apex_above_release_m) or None."""
    best = None
    for _bid, S in balls.items():
        sel = (S[:, 0] > t_peak - 0.05) & (S[:, 0] < t_peak + 1.6)
        if sel.sum() < 15:
            continue
        s = S[sel]
        if s[0, 0] > t_peak + 0.30 or np.ptp(s[:, 3]) < 200:
            continue
        ia = int(np.argmax(s[:, 3]))
        if ia < 8:
            continue
        if best is None or ia > best[1]:
            best = (s, ia)
    if best is None:
        return None
    s, ia = best
    rise = s[:ia + 1]
    ref_z = zr if zr is not None else float(rise[0, 3])
    keep = (rise[:, 3] > ref_z + 60) & (rise[:, 0] < rise[ia, 0] - 0.06)
    rs = rise[keep]
    if len(rs) < 6:
        return None
    tau = rs[:, 0] - t_peak
    y = rs[:, 3] + 0.5 * G_MM_S2 * tau ** 2
    A = np.vstack([np.ones_like(tau), tau]).T
    (_z0, v), *_ = np.linalg.lstsq(A, y, rcond=None)
    apex_mm = float(s[ia, 3])
    return dict(v_mps=float(v) / 1000.0,
                apex_above_release_m=(apex_mm - ref_z) / 1000.0)


def analyse_stroke(H, t, i, cmd_source, mm_per_rev, ann, balls):
    pc, vff, pm, vm, iq = H[:, 2], H[:, 3], H[:, 4], H[:, 5], H[:, 6]
    j0 = i
    while j0 > 0 and vff[j0] > 2.0 and t[i] - t[j0] < _RAMP_BACK_S:
        j0 -= 1
    j1 = i
    while j1 < len(H) - 1 and vff[j1] > 2.0 and t[j1] - t[i] < _TAIL_S:
        j1 += 1
    w = slice(j0, j1 + 1)
    tw = t[w]

    match = match_announcement(ann, t[i])
    cmd_peak = float(vff[i])
    if cmd_source == 'announcement' and match is not None:
        cmd_peak = abs(match[1]) / mm_per_rev  # mm/s / (mm/rev) -> rev/s

    # measured peak: 3-point parabolic refinement around the sampled max
    im = j0 + int(np.argmax(vm[w]))
    vpk = vm[im]
    if 0 < im < len(H) - 1:
        a, b, c = vm[im - 1], vm[im], vm[im + 1]
        den = a - 2 * b + c
        meas_peak = b - (a - c) ** 2 / (8 * den) if den < 0 else b
    else:
        meas_peak = vpk

    iq_pk = float(np.abs(iq[w]).max())

    # advisory: delay fit on the ramp (20-90% of the commanded peak)
    ramp = np.arange(j0, i + 1)
    ramp = ramp[vff[ramp] > 0.2 * cmd_peak] if cmd_peak > 0 else ramp
    if len(ramp) >= 2:
        best = (float('inf'), 0.0)
        for d in _DELAY_GRID_S:
            e = vm[ramp] - np.interp(t[ramp] - d, t, vff)
            s = float(np.sqrt(np.mean(e ** 2)))
            if s < best[0]:
                best = (s, float(d))
        delay_s = best[1]
    else:
        delay_s = 0.0
    res = vm[w] - np.interp(tw - delay_s, t, vff)
    overspeed = float(res.max())

    plag = np.interp(tw - delay_s, t, pc) - pm[w]
    up_n = i - j0 + 1
    lag_delay_comp = float(plag[:up_n].max()) if up_n > 0 else float('nan')

    up = np.arange(j0, i + 1)
    thr_lo = up[np.argmax(vff[up] >= 0.1 * cmd_peak)] if cmd_peak > 0 else j0
    thr_hi = up[np.argmax(vff[up] >= 0.9 * cmd_peak)] if cmd_peak > 0 else i
    acc_mean = 0.8 * cmd_peak / max(t[thr_hi] - t[thr_lo], 1e-3)

    ramp_w = slice(j0, i + 1)
    dv = np.diff(vff[ramp_w])
    dtt = np.diff(t[ramp_w])
    acc10 = float((dv / dtt).max()) if len(dv) else float('nan')

    cmd_apex_m = None
    zr = None
    if match is not None:
        tof = match[3]
        cmd_apex_m = G_M_S2 * tof ** 2 / 8.0
        zr = match[2]
    ball = fit_ball(balls, t[i], zr)

    row = dict(
        t_peak=round(float(t[i]), 3),
        cmd_apex_m=round(cmd_apex_m, 3) if cmd_apex_m is not None else '',
        cmd_peak_rps=round(cmd_peak, 1),
        meas_peak_rps=round(float(meas_peak), 1),
        meas_over_cmd=round(float(meas_peak) / cmd_peak, 3) if cmd_peak else '',
        peak_iq_a=round(iq_pk, 1),
        ball_v_mps='', ball_apex_m='', ball_over_cmd='', ball_over_meas_gain='',
        cmd_delay_ms_adv=round(delay_s * 1e3, 1),
        overspeed_rps_adv=round(overspeed, 1),
        lag_delay_comp_rev_adv=round(lag_delay_comp, 3),
        acc_mean_rps2_adv=round(acc_mean, 0),
        peak_cmd_acc10ms_rps2_adv=round(acc10, 0),
    )
    if ball is not None:
        v = ball['v_mps']
        row['ball_v_mps'] = round(v, 3)
        row['ball_apex_m'] = round(ball['apex_above_release_m'], 3)
        cmd_mps = cmd_peak * mm_per_rev / 1000.0
        meas_mps = float(meas_peak) * mm_per_rev / 1000.0
        row['ball_over_cmd'] = round(v / cmd_mps, 3) if cmd_mps else ''
        row['ball_over_meas_gain'] = round(v / meas_mps, 3) if meas_mps else ''
    return row


COLUMNS = ['t_peak', 'cmd_apex_m', 'cmd_peak_rps', 'meas_peak_rps',
           'meas_over_cmd', 'peak_iq_a', 'ball_v_mps', 'ball_apex_m',
           'ball_over_cmd', 'ball_over_meas_gain', 'cmd_delay_ms_adv',
           'overspeed_rps_adv', 'lag_delay_comp_rev_adv', 'acc_mean_rps2_adv',
           'peak_cmd_acc10ms_rps2_adv']

HEADERS = {
    't_peak': 't_peak (s)', 'cmd_apex_m': 'cmd apex (m)',
    'cmd_peak_rps': 'cmd peak (rev/s)', 'meas_peak_rps': 'meas peak (rev/s)',
    'meas_over_cmd': 'meas/cmd', 'peak_iq_a': 'peak |iq| (A)',
    'ball_v_mps': 'ball v (m/s)', 'ball_apex_m': 'ball apex (m)',
    'ball_over_cmd': 'ball/cmd', 'ball_over_meas_gain': 'ball/(meas*gain)',
    'cmd_delay_ms_adv': 'cmd delay (ms) (adv)',
    'overspeed_rps_adv': 'overspeed beyond delay (rev/s) (adv)',
    'lag_delay_comp_rev_adv': 'delay-comp lag (rev) (adv)',
    'acc_mean_rps2_adv': 'mean ramp accel (rev/s^2) (adv)',
    'peak_cmd_acc10ms_rps2_adv': 'peak 10ms cmd accel (rev/s^2) (adv)',
}


def print_markdown(rows):
    print('| ' + ' | '.join(HEADERS[c] for c in COLUMNS) + ' |')
    print('|' + '---|' * len(COLUMNS))
    for r in rows:
        print('| ' + ' | '.join(str(r[c]) for c in COLUMNS) + ' |')


def write_csv(rows, path):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=COLUMNS)
        w.writeheader()
        for r in rows:
            w.writerow(r)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__.split('\n\n')[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bag', required=True, help='rosbag directory')
    ap.add_argument('--since', type=float, default=None,
                     help='ROS epoch seconds, lower window bound')
    ap.add_argument('--until', type=float, default=None,
                     help='ROS epoch seconds, upper window bound')
    ap.add_argument('--cmd-source', choices=['telemetry', 'announcement'],
                     default='telemetry')
    ap.add_argument('--mm-per-rev', type=float, default=32.567)
    ap.add_argument('--min-peak-rps', type=float, default=60.0)
    ap.add_argument('--out', default=None, help='CSV output path')
    args = ap.parse_args(argv)

    bag_dir = parse_bag_dir(args.bag)
    if not os.path.isdir(bag_dir):
        print('ERROR: bag directory not found: %s' % bag_dir, file=sys.stderr)
        return 1

    hand, ann, balls = read_bag(bag_dir, since=args.since, until=args.until)
    if not hand:
        print('ERROR: no /hand_telemetry messages in %s' % bag_dir,
              file=sys.stderr)
        return 1
    H = np.array(hand)
    H = H[np.argsort(H[:, 1])]      # sort by log_time first (always monotonic)
    H = dedupe(H)
    t, tsrc = choose_time_column(H)

    if args.since is not None:
        keep = t >= args.since
        H, t = H[keep], t[keep]
    if args.until is not None:
        keep = t <= args.until
        H, t = H[keep], t[keep]

    pk = find_peaks(t, H[:, 3], args.min_peak_rps)
    rows = [analyse_stroke(H, t, i, args.cmd_source, args.mm_per_rev, ann, balls)
            for i in pk]
    rows.sort(key=lambda r: r['t_peak'])

    out = args.out
    if out is None:
        base = os.path.basename(os.path.normpath(bag_dir))
        out = os.path.join(OUT_DIR, 'hand_overspeed_%s.csv' % base)

    print('# hand_overspeed_bag_probe: %s (%s, time column: %s, %d strokes)'
          % (bag_dir, args.cmd_source, tsrc, len(rows)))
    print_markdown(rows)
    write_csv(rows, out)
    print('\nCSV: %s' % out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
