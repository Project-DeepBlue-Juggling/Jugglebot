#!/usr/bin/env python3
"""Carry-window metrics for an oval / hop sitting, straight from a rosbag + launch log.

Origin: the 2026-10-23 oval P2 measure-first review
(logbook/2026-10-23-oval-p2-banking-leak-not-reproduced.md); fills the
as-is column of the runsheet `tests/hardware/session_oval_rung1.md` § 6 (rows 22a, 23, 24, 27) from the
bag of 2026-10-23_11-06-46.

Per CARRY (scheduled landing time from /throw_announcements -> next Jugglebot
throw_time) it reports: platform bank (FK of commanded /leg_cmd_executed and
measured /robot_state; bank = arccos(R[2,2]), normal components asin(-n_y),
asin(n_x), BASE frame, no tilt map), commanded leg peak vel/acc/jerk, leg 1/2
iq peak/rms, seat-bit continuity (ball_held vs ball_held_raw), landing dx/dy
(skill_node "throw k/N ... landed dx, dy mm" lines), plus per-goal diag peaks,
emit gap and a LIMIT_* tally.

METHOD (preserve): leg vel/acc/jerk come from /leg_setpoint_echo POSITIONS with
h = 0.025 s taken from the KNOT INDEX, not log time: v = (x[i+1]-x[i-1])/2h,
a = 2nd difference/h^2, jerk = backward 3rd difference/h^3, no smoothing,
abs-max over 6 legs. (Label note: the 2026-10-23 report called the -3.25..+2.34 deg
range the "normal y-component"; by the code it is asin(n_x), the dominant tilt axis,
and is printed here as normal-x. The small axis is normal-y.) A commanded-plan quantity, comparable to the node's own
40 Hz tick-diff jerk.
REJECTED PATH: finite differences of /leg_cmd_executed velocity at "100 Hz".
That stream has a 219 ms gap and ~10 ms timestamp jitter; differencing it
gives 400-570k spurious jerk and a 22 mm/s rms vel-channel vs d(pos)/dt
mismatch. Do not use log-time differences of /leg_cmd_executed.

CAVEAT -- splice-seam artefacts: the echo stream restarts at re-aim installs,
so WHOLE-GOAL / throw-to-throw echo peaks contain seam artefacts (2026-10-23:
hop 568k, O2 305k and 83k) that are not real plan jerk. Compare on the
carry-window numbers (and the node's diag column), never the whole-goal echo
peaks (those are only printed in the per-goal block, marked "seam-prone").
Carry-window peaks can still hold one seam (O2 carry k=6, 83.1k).

TOUCH-DOWN CUP VELOCITY (per carry; "v_td" column and per-goal median): the mocap
`Platform` rigid body from /rigid_body_poses (body.name == 'Platform', stamp =
body.pose.header.stamp, ~300 Hz, WORLD mm), sorted by stamp, central difference
over +-15 ms (30 ms window, nearest samples via searchsorted) evaluated at the
the carry's scheduled touch-down time t_td: the skill_node "CATCH-AIM skill N: ...
landing=(x, y, z) mm t_land=T" line (launch log; only lines with landing x > 0, i.e. the
catch site, de-duplicated on (skill, round(T))) whose T lies in (l, l+0.1] after the
carry's /throw_announcements landing_time l -- it runs ~32 ms (19-49) after l, and the
cup is still decelerating there, so evaluating at l instead reads ~80 mm/s (pose 0.65)
and ~125 mm/s (pose 0.50) less negative. No such line -> '--'. v = (p[b]-p[a])/(t[b]-t[a]),
a = first sample >= t_td-15 ms, b = first sample >= t_td+15 ms. The world xy vector is rotated by +`--cup-yaw-deg`
(default 10.81, --cup-yaw-deg; the Sitting's platform-start frame, ~ the
robot frame, whose launch-log yaw is -10.7..-10.8 deg) to give (v_x, v_y) in the robot/base frame, mm/s.
Same recipe as the 2026-10-24 sitting review (its skid.py); the probe and the review agree
to the mm/s. Mocap is decoded once to <out>/plat.npy (the 675 MB bag is not re-read).
Missed catches are still reported (the cup moves regardless of the ball). Per-goal
number = median over that goal's carries. Only carries of 'oval' goals are listed.

Goal windows: parsed from the launch log (--goals auto, default). Regexes:
  DISPATCH  Juggle requested .* goal dispatched \\(pattern='(\\w+)'
  ACCEPT    skill_node\\]: New goal accepted: \\[([\\d ]+)\\]      (start = leading epoch)
  FINISH    skill_node\\]: Goal with ID \\[([\\d ]+)\\] finished with state (\\d+)
  LANDED    throw (\\d+)/(\\d+) (\\w+) .*?landed ([+-]\\d+), ([+-]\\d+) mm
A goal = ACCEPT paired with the FINISH of the same id, named by the latest
preceding DISPATCH pattern (+ '#k' per pattern; state != 4 is flagged). Fallback:
--windows name:epoch0:epoch1,... (alias --goals <spec>).

Cache: the decoded bag is pickled to <out>/bag.pkl, so re-runs are seconds.
Run (venv):  python tools/probes/oval_carry_bag_metrics.py <bag.mcap> \\
    --launch-log <launch.log> [--goals auto|SPEC] [--out temp/probes/oval_carry_<bagstamp>/]
Output: <out>/row27.md (also printed). Writes only under --out.
"""
from __future__ import annotations

import argparse
import os
import pickle
import re
import sys

for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path[:0] = [ROOT, os.path.join(ROOT, "ros_ws", "src", "jugglebot")]

import numpy as np  # noqa: E402

TOPICS = ['/throw_announcements', '/hand_telemetry', '/robot_state', '/leg_cmd_executed',
          '/leg_setpoint_echo', '/trajectory/status', '/trajectory/diagnostics']
H = 0.025     # echo knot period, s
RE_DISP = re.compile(r"Juggle requested .* goal dispatched \(pattern='(\w+)'")
RE_ACC = re.compile(r"skill_node\]: New goal accepted: \[([\d ]+)\]")
RE_FIN = re.compile(r"skill_node\]: Goal with ID \[([\d ]+)\] finished with state (\d+)")
RE_LAND = re.compile(r"throw (\d+)/(\d+) (\w+) .*?landed ([+-]\d+), ([+-]\d+) mm")
RE_LIM = re.compile(r"\b(LIMIT_\w+)")


def ts(s):
    return s['sec'] + s['nanosec'] * 1e-9


def tod(o):
    if hasattr(o, '__slots__'):
        return {s: tod(getattr(o, s)) for s in o.__slots__}
    if isinstance(o, (list, tuple)):
        return [tod(x) for x in o]
    return o.tolist() if hasattr(o, 'tolist') else o


def load_bag(path, cache):
    if os.path.exists(cache):
        return pickle.load(open(cache, 'rb'))
    from mcap_ros2.reader import read_ros2_messages     # same reader as gravity_ff_ab_extract.py
    d = {k: [] for k in TOPICS}
    for r in read_ros2_messages(path, topics=TOPICS):
        d[r.channel.topic].append((r.log_time_ns, tod(r.ros_msg)))
    pickle.dump(d, open(cache, 'wb'))
    return d


def load_platform(path, cache):
    """Mocap `Platform` body -> array [t_epoch, x, y, z] sorted by t (cached as .npy)."""
    if os.path.exists(cache):
        return np.load(cache)
    from mcap_ros2.reader import read_ros2_messages
    rows = []
    for r in read_ros2_messages(path, topics=['/rigid_body_poses']):
        for b in r.ros_msg.bodies:
            if b.name == 'Platform':
                p = b.pose.pose.position
                st = b.pose.header.stamp
                rows.append((st.sec + st.nanosec * 1e-9, p.x, p.y, p.z))
    arr = np.array(rows, float).reshape(-1, 4)
    arr = arr[np.argsort(arr[:, 0], kind='stable')]
    np.save(cache, arr)
    return arr


def cup_velocity(plat, t_epoch, yaw_deg, half=0.015):
    """(v_x, v_y) mm/s, robot frame, 30 ms central difference at t_epoch; None if no data."""
    if len(plat) < 2:
        return None
    tp = plat[:, 0]
    a, b = np.searchsorted(tp, t_epoch - half), np.searchsorted(tp, t_epoch + half)
    if b >= len(tp) or tp[b] == tp[a] or tp[b] - tp[a] > 0.1:
        return None
    v = (plat[b, 1:3] - plat[a, 1:3]) / (tp[b] - tp[a])
    ya = np.radians(yaw_deg)
    return (float(np.cos(ya) * v[0] - np.sin(ya) * v[1]), float(np.sin(ya) * v[0] + np.cos(ya) * v[1]))


RE_AIM = re.compile(r"CATCH-AIM skill (\d+): source=(\S+) landing=\(([-\d.]+), ([-\d.]+),.*t_land=([\d.]+)")


def parse_catch_aims(path):
    """-> sorted epoch t_land of the CATCH-AIM lines at the catch site (landing x > 0)."""
    ev = {}
    for line in open(path, errors='replace'):
        m = RE_AIM.search(line)
        if m and float(m.group(3)) > 0:
            ev[(int(m.group(1)), round(float(m.group(5))))] = float(m.group(5))
    return np.array(sorted(ev.values()), float)


def touchdown(aims, l_epoch):
    k = np.searchsorted(aims, l_epoch, side='right')
    return float(aims[k]) if k < len(aims) and aims[k] - l_epoch <= 0.1 else None


def parse_launch(path):
    """-> (goals [(name, t0, t1, state)], landed [(epoch, k, n, kind, dx, dy)], limit tally)."""
    pend, acc, goals, landed, lim, cnt = None, {}, [], [], {}, {}
    for line in open(path, errors='replace'):
        try:
            t = float(line.split(' ', 1)[0])
        except ValueError:
            continue
        for c in RE_LIM.findall(line):
            lim[c] = lim.get(c, 0) + 1
        m = RE_DISP.search(line)
        if m:
            pend = m.group(1)
        m = RE_ACC.search(line)
        if m:
            acc[m.group(1).split().__str__()] = (t, pend)
        m = RE_FIN.search(line)
        if m and m.group(1).split().__str__() in acc:
            t0, pat = acc.pop(m.group(1).split().__str__())
            cnt[pat] = cnt.get(pat, 0) + 1
            goals.append((f"{pat}#{cnt[pat]}", t0, t, int(m.group(2))))
        m = RE_LAND.search(line)
        if m:
            landed.append((t, int(m.group(1)), int(m.group(2)), m.group(3), int(m.group(4)), int(m.group(5))))
    return goals, landed, lim


def spec_goals(spec):
    return [(n, float(a), float(b), 4) for n, a, b in (s.split(':') for s in spec.split(','))]


def make_fk(geom):
    from jugglebot.motion.ik_solver import leg_lengths_to_pose

    def fk_series(t, rev):
        out, guess = [], None
        for ti, r in zip(t, rev):
            try:
                p, R, _ = leg_lengths_to_pose(np.asarray(r) / geom.mm_to_rev, geom, initial_guess=guess)
            except Exception:
                out.append([ti] + [np.nan] * 3)
                continue
            guess = (p, R)
            n = R @ np.array([0, 0, 1.0])
            out.append([ti, np.degrees(np.arccos(np.clip(n[2], -1, 1))),
                        np.degrees(np.arcsin(-n[1])), np.degrees(np.arcsin(n[0]))])
        return np.array(out)
    return fk_series


def W(t, a, b):
    return (t >= a) & (t <= b)


def runs_false(x, t, tref):
    """Runs of False in a bool series -> [(time-before-tref, duration)]."""
    r, i = [], 0
    while i < len(x):
        if not x[i]:
            k = i
            while k < len(x) and not x[k]:
                k += 1
            r.append((round(tref - t[i], 3), round(t[min(k, len(t) - 1)] - t[i], 3)))
            i = k
        else:
            i += 1
    return r


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('bag')
    ap.add_argument('--launch-log', required=True)
    ap.add_argument('--goals', '--windows', dest='goals', default='auto',
                    help="auto (parse launch log) or name:epoch0:epoch1,...")
    ap.add_argument('--out', default=None)
    ap.add_argument('--cup-yaw-deg', type=float, default=10.81,
                    help="world->robot xy rotation applied to the Platform velocity (default 10.81)")
    a = ap.parse_args()
    stamp = os.path.basename(os.path.dirname(os.path.abspath(a.bag))) or 'bag'
    out = a.out or os.path.join(ROOT, 'temp', 'probes', f'oval_carry_{stamp}')
    os.makedirs(out, exist_ok=True)
    d = load_bag(a.bag, os.path.join(out, 'bag.pkl'))
    plat = load_platform(a.bag, os.path.join(out, 'plat.npy'))
    aims = parse_catch_aims(a.launch_log)
    goals, landed, lim = parse_launch(a.launch_log)
    if a.goals != 'auto':
        goals = spec_goals(a.goals)
    if not goals:
        sys.exit('no goal windows found; pass --windows name:t0:t1,...')
    E0 = float(int(d['/leg_setpoint_echo'][0][0] / 1e9))      # bag-relative time origin (epoch s)

    from jugglebot.motion.geometry import StewartGeometry
    geom = StewartGeometry(); MMREV = np.asarray(geom.mm_to_rev); fk = make_fk(geom)
    # --- series
    e = d['/leg_setpoint_echo']
    te = np.array([t / 1e9 - E0 for t, _ in e]); xe = np.array([m['data'] for _, m in e]) / MMREV
    v = (xe[2:] - xe[:-2]) / (2 * H); tv = te[1:-1]
    ac = (xe[2:] - 2 * xe[1:-1] + xe[:-2]) / H ** 2; ta = te[1:-1]
    j = (xe[3:] - 3 * xe[2:-1] + 3 * xe[1:-2] - xe[:-3]) / H ** 3; tj = te[1:-2]
    leg = d['/leg_cmd_executed']
    tl = np.array([ts(m['header']['stamp']) for _, m in leg]); pl = np.array([m['position'] for _, m in leg])
    rs = d['/robot_state']
    tr = np.array([ts(m['timestamp']) for _, m in rs])
    pr = np.array([[s['pos_estimate'] for s in m['motor_states'][:6]] for _, m in rs])
    iq = np.array([[s['iq_measured'] for s in m['motor_states'][:6]] for _, m in rs])
    fet = np.array([[s['fet_temp'] for s in m['motor_states'][:6]] for _, m in rs])
    hh = d['/hand_telemetry']
    th = np.array([ts(m['timestamp']) - E0 for _, m in hh])
    held = np.array([m['ball_held'] for _, m in hh]); raw = np.array([m['ball_held_raw'] for _, m in hh])
    ann = [(ts(m['throw_time']) - E0, ts(m['landing_time']) - E0, m['thrower_name']) for _, m in d['/throw_announcements']]
    dg = d['/trajectory/diagnostics']; tdg = np.array([t / 1e9 - E0 for t, _ in dg])
    dgk = [{x['key']: x['value'] for x in m['values']} for _, m in dg]

    def peak3(a_, b_):
        m1, m2, m3 = W(tv, a_, b_), W(ta, a_, b_), W(tj, a_, b_)
        if m1.sum() < 3:
            return [np.nan] * 6
        return [np.abs(v[m1]).max(), np.abs(ac[m2]).max(), np.abs(j[m3]).max(),
                int(np.abs(v[m1]).max(0).argmax()), int(np.abs(ac[m2]).max(0).argmax()), int(np.abs(j[m3]).max(0).argmax())]

    def iqs(a_, b_):
        q = iq[W(tr - E0, a_, b_)]
        return (np.abs(q[:, 1]).max(), np.sqrt((q[:, 1] ** 2).mean()), np.abs(q[:, 2]).max(),
                np.sqrt((q[:, 2] ** 2).mean()), fet[W(tr - E0, a_, b_)][:, 1:3].max())

    carries, goalrows = [], []
    for name, g0, g1, state in goals:
        a0, b0 = g0 - E0, g1 - E0
        sel = lambda t: (t >= g0 - 0.3) & (t <= g1 + 0.3)
        mc, mr = sel(tl), sel(tr)
        cmdfk, measfk = fk(tl[mc] - E0, pl[mc]), fk(tr[mr] - E0, pr[mr])
        # goal-level (seam-prone echo peaks + node diag + emit gap)
        md = W(tdg, a0, b0)
        dvals = lambda k: [float(x[k]) for x, mm in zip(dgk, md) if mm]
        gaps = np.array(dvals('max_emit_gap_ms')) if md.any() else np.array([np.nan])
        pk = peak3(a0, b0)
        goalrows.append(dict(name=name, state=state, win=(round(a0, 1), round(b0, 1)), echo=pk[:3],
                             diag=[max(dvals(k)) if md.any() else np.nan for k in
                                   ('realized_peak_leg_vel_mmps', 'realized_peak_leg_acc_mmps2', 'realized_peak_leg_jerk_mmps3')],
                             gap=(float(np.median(gaps)), float(np.percentile(gaps, 99)), float(gaps.max())),
                             iq=iqs(a0, b0), nland=sum(1 for x in landed if g0 <= x[0] <= g1)))
        if not name.startswith('oval'):
            continue
        thr = sorted(x[0] for x in ann if x[2] == 'jugglebot' and a0 <= x[0] <= b0 + 0.5)
        land = sorted(x[1] for x in ann if a0 - 0.5 <= x[1] <= b0 + 0.5)
        for kk, tt in enumerate(thr):
            prior = [l for l in land if l < tt]
            if not prior:
                continue
            l = prior[-1]
            ms = np.where((th >= l - 0.15) & held)[0]
            s0 = th[ms[0]] if len(ms) else np.nan
            pk = peak3(l, tt)
            q = iqs(l, tt)

            def bk(X):
                m = W(X[:, 0], l, tt)
                return (X[m, 1].min(), X[m, 1].max(), X[m, 2].min(), X[m, 2].max(), X[m, 3].min(), X[m, 3].max())
            bc, bm = bk(cmdfk), bk(measfk)
            mm = W(th, s0, tt)
            after = np.where((th > tt) & (~held))[0]
            carries.append(dict(
                goal=name, k=kk, len=tt - l, steady=(tt - l) < 1.0, seat_lag_ms=round((s0 - l) * 1e3) if np.isfinite(s0) else None,
                vtd=(lambda t_td: cup_velocity(plat, t_td, a.cup_yaw_deg) if t_td else None)(touchdown(aims, l + E0)),
                bank_cmd=bc[1], floor=bc[0], bank_meas=bm[1], ny=(bc[2], bc[3]), nx=(bc[4], bc[5]),
                vmax=pk[0], amax=pk[1], jmax=pk[2], legs=pk[3:],
                iq1=(q[0], q[1]), iq2=(q[2], q[3]), fet=q[4],
                held_all=bool(held[mm].all()), held_falls_ms=None if not len(after) else round((th[after[0]] - tt) * 1e3),
                raw_runs=runs_false(raw[mm], th[mm], tt)))
    # per-carry landing dx/dy by goal
    L = []
    P = lambda *s: L.append(' '.join(str(x) for x in s))
    f = lambda x, n=0: f"{x:.{n}f}" if np.isfinite(x) else 'nan'
    P(f"# Carry metrics: {os.path.basename(a.bag)}\n")
    P(f"Goals (launch log, bag-relative t = epoch - {E0:.0f}):")
    P("| goal | window t | state | tracker lines |\n|---|---|---|---|")
    for g in goalrows:
        P(f"| {g['name']} | {g['win'][0]}-{g['win'][1]} | {g['state']} | {g['nland']} |")
    P("\n## Per carry (landing -> next Jugglebot throw)")
    vt = lambda c: f"{c['vtd'][0]:+.0f}, {c['vtd'][1]:+.0f}" if c['vtd'] else '--'
    P("| goal | k | len s | steady | seat lag ms | v_td x, y mm/s | bank cmd max (floor) / meas max deg | normal-y range deg | leg v / a / jerk | leg1 iq pk/rms | leg2 iq pk/rms | held all | raw dropouts (s before release, dur) |")
    P("|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    for c in carries:
        P(f"| {c['goal']} | {c['k']} | {c['len']:.2f} | {c['steady']} | {'--' if c['seat_lag_ms'] is None else c['seat_lag_ms']} | {vt(c)} | {c['bank_cmd']:.2f} ({c['floor']:.2f}) / {c['bank_meas']:.2f} | "
          f"{c['ny'][0]:+.2f}..{c['ny'][1]:+.2f} | {f(c['vmax'])} / {f(c['amax'])} / {f(c['jmax'] / 1e3, 1)}k | "
          f"{c['iq1'][0]:.2f}/{c['iq1'][1]:.2f} | {c['iq2'][0]:.2f}/{c['iq2'][1]:.2f} | {c['held_all']} | {c['raw_runs']} |")
    S = [c for c in carries if c['steady']]
    if S:
        A_ = lambda k: np.array([c[k] for c in S], float)
        mx = lambda x: f"{x.min():.2f} / {np.median(x):.2f} / {x.max():.2f}"
        P(f"\n## Aggregate, {len(S)} steady carries (< 1.0 s)")
        P(f"- bank commanded max, min / median / max: {mx(A_('bank_cmd'))}; measured-FK: {mx(A_('bank_meas'))}")
        P(f"- normal-y range: {min(c['ny'][0] for c in S):+.2f} .. {max(c['ny'][1] for c in S):+.2f}; "
          f"normal-x range {min(c['nx'][0] for c in S):+.2f} .. {max(c['nx'][1] for c in S):+.2f}")
        P(f"- leg vel median {np.median(A_('vmax')):.0f} max {A_('vmax').max():.0f}; acc median {np.median(A_('amax')):.0f} max {A_('amax').max():.0f}; "
          f"jerk median {np.median(A_('jmax')):,.0f} max {A_('jmax').max():,.0f}")
        i1 = np.array([c['iq1'] for c in S]); i2 = np.array([c['iq2'] for c in S])
        P(f"- leg1 iq pk median {np.median(i1[:, 0]):.2f} max {i1[:, 0].max():.2f}, rms mean {i1[:, 1].mean():.2f}; "
          f"leg2 iq pk median {np.median(i2[:, 0]):.2f} max {i2[:, 0].max():.2f}, rms mean {i2[:, 1].mean():.2f}")
        nd = sum(1 for c in carries if c['raw_runs'])
        P(f"- seat: ball_held unbroken in {sum(c['held_all'] for c in carries)}/{len(carries)} carries; "
          f"ball_held_raw dropouts in {nd}/{len(carries)}")
    P("\n## Per goal (seam-prone echo peaks; diag = node 40 Hz tick-diff, 5 Hz samples, a lower bound)")
    P("| goal | echo v/a/jerk (seam-prone) | diag v/a/jerk | emit gap p50/p99/max ms | leg1 iq pk/rms | leg2 iq pk/rms |\n|---|---|---|---|---|---|")
    for g in goalrows:
        P(f"| {g['name']} | {f(g['echo'][0])}/{f(g['echo'][1])}/{f(g['echo'][2] / 1e3, 1)}k | "
          f"{f(g['diag'][0])}/{f(g['diag'][1])}/{f(g['diag'][2] / 1e3, 1)}k | {g['gap'][0]:.1f}/{g['gap'][1]:.1f}/{g['gap'][2]:.1f} | "
          f"{g['iq'][0]:.2f}/{g['iq'][1]:.2f} | {g['iq'][2]:.2f}/{g['iq'][3]:.2f} |")
    P("\n## Touch-down cup velocity per goal (mocap Platform, robot frame, median over carries; mm/s)")
    P("| goal | n carries | n with v | v_x median | v_y median |\n|---|---|---|---|---|")
    for name in dict.fromkeys(c['goal'] for c in carries):
        vs = np.array([c['vtd'] for c in carries if c['goal'] == name and c['vtd']], float).reshape(-1, 2)
        med = (lambda x: f"{x:+.0f}") if len(vs) else (lambda x: '--')
        P(f"| {name} | {sum(c['goal'] == name for c in carries)} | {len(vs)} | "
          f"{med(np.median(vs[:, 0]) if len(vs) else 0)} | {med(np.median(vs[:, 1]) if len(vs) else 0)} |")
    P("\n## Landing dx/dy (tracker log lines, mm)")
    P("| goal | n | dx mean/sd | dy mean/sd |\n|---|---|---|---|")
    for name, g0, g1, _ in goals:
        r = [(x[4], x[5]) for x in landed if g0 <= x[0] <= g1]
        if r:
            r = np.array(r, float)
            P(f"| {name} | {len(r)} | {r[:, 0].mean():+.1f}/{r[:, 0].std(ddof=1):.1f} | {r[:, 1].mean():+.1f}/{r[:, 1].std(ddof=1):.1f} |")
    P(f"\n## LIMIT_* mentions (whole launch log, text mentions not distinct events)\n{lim or 'none'}")
    txt = '\n'.join(L)
    open(os.path.join(out, 'row27.md'), 'w').write(txt + '\n')
    print(txt)
    print(f"\nwrote {out}/row27.md")


if __name__ == '__main__':
    main()
