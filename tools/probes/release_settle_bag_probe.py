#!/usr/bin/env python3
"""Per-Throw release state and in-Cup settle measures from a bag (learner-efficacy E0).

WHAT IT CHARACTERISES
---------------------
For every Throw the Learner recorded (a Memory row) that a bag covers, two things:

1. The ingredients of the launch term g of the 2026-11-23 review: the ball's lateral
   launch velocity (linear fit to the Mocap ball marker 0.035 to 0.2 s after the
   announced release), the commanded Cup position and velocity at release (FK of
   `/leg_cmd_executed` plus the Hand command), and the Platform's measured normal,
   normal rate and centroid velocity at release (Mocap Platform markers). g itself is
   a within-group residual and is formed by the consumer, not here.
2. Settle measures: the ball marker's motion relative to the Cup BEFORE release. The
   marker is expressed in the Platform's Mocap frame (Kabsch on the Platform markers,
   per frame), reduced to a lateral offset from the Hand axis (a line fitted per bag),
   and turned to world axes at release. The raw offset carries a repeatable artefact
   of several mm that depends on Hand height (the cameras lose and regain parts of the
   ball behind the Cup rim and the Platform), so the measures are taken on the residual
   from the median trace of the Throws that share a trajectory (bag, Site, release
   kind, Dwell), indexed by time before release. A ball resting in the Cup with the
   Hand still needs no template: its residual is the offset minus its own median.
   Columns: `v<W>_x/_y` (velocity against the Cup, mm/s), `off<W>_x/_y` (mean offset,
   mm), `wig_<W>` (rms about the trend, mm), `spd_<W>` (path speed, mm/s), `c_<W>`
   (coverage), for the windows 35, a (-100..-35 ms), 100, 200, 500 ms before the
   announced release and p (0..+35 ms after it); `t_seat` (time since the ball
   seated, capped at 1.3 s), `from_rest`, `entry_*` (where the ball came in against
   the Hand axis at the Catch), `ax_lift_*` (lift off the seat), `sens_*` (Hand ball
   sensor), `t_vpk` (the ball's own peak upward velocity against the announced release).

Release reference: the announcement's `throw_time`, which sits on the 25 ms tick grid;
the ball's own peak upward velocity lands 2.7 ms before it. Memory's `t_abs_s` is the
plan's time and is up to 12.5 ms away, so it only matches a Memory row to its Throw.

Timing: every stream goes through `_bag_time.py` (Mocap envelope stamp minus err_us
minus 13 ms; `/hand_telemetry` via its bridge stamp or pairing with
`/leg_cmd_executed`). The per-body stamps of `/rigid_body_poses` are never read.

Motivation: `logbook/2026-11-25-learner-efficacy-e0-unsettled-ball-test.md`
(plan `plans/active/learner-efficacy.md`, Phase E0). Consuming test: none (a
run-on-demand harness; the analysis that consumed it is named in that entry).

USAGE
-----
    python tools/probes/release_settle_bag_probe.py extract  BAGDIR  [--out NPZ]
    python tools/probes/release_settle_bag_probe.py features NPZ     [--out CSV] [--traces NPZ]
    python tools/probes/release_settle_bag_probe.py run      BAGDIR          # both

Outputs go to `temp/probes/release_settle/` (series `<bag>.npz`, per-Throw records
`<bag>.csv`, residual traces `<bag>_traces.npz`). The directory is fixed and a rerun
overwrites a bag's files: the series are a cache of the bag, and the same input
gives the same output (there is no random element). Reads every
`temp/learn/*/memory_v2.csv` unless `--memory` names another glob. Run one process
at a time under `nice -n 15`; `extract` took 1 to 25 min per bag on the loaded box
(2026-11-25, 21 bags, 17 GB), `features` about 20 s per 340 Throws.
"""
from __future__ import annotations

import argparse
from array import array
import csv
import glob
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path[:0] = [os.path.join(REPO, 'tools', 'probes'), os.path.join(REPO, 'ros_ws', 'src', 'jugglebot')]
import _bag_time as bt  # noqa: E402

G = 9806.65                      # mm/s^2
OUT_DIR = os.path.join(REPO, 'temp', 'probes', 'release_settle')
MEM_GLOB = os.path.join(REPO, 'temp', 'learn', '*', 'memory_v2.csv')
PRE_S, POST_S = 1.6, 0.5         # series kept around each Memory row (log time)
LOOK_S = 1.3                     # in-Cup look-back before release
FLIGHT = (0.035, 0.2)            # launch-velocity fit window after release (the review's)
LAT_GATE_MM = 40.0               # ball-to-Hand-axis gate for an in-Cup sample (wide enough for an off-axis Catch)
AX_GATE_MM = 25.0                # ball-to-Cup axial gate for "seated"
HAND_MM_PER_REV = 32.567         # config/generated/hardware_config.py GEOM_HAND_MM_PER_REV
STAMP_SLACK_S = 0.015            # Hand telemetry stamp and encoder age allowance in the seated gate
REST_S = 1.0                     # seated this long before release = from rest
GRID = 1.0 / 300.0               # template grid (Mocap frame period)
WINDOWS = {'35': (-0.035, 0.0), 'a': (-0.100, -0.035), '100': (-0.100, 0.0),
           '200': (-0.200, 0.0), '500': (-0.500, 0.0), 'p': (0.0, 0.035)}
POST_LOOK_S = 0.045              # the series runs this far past release, for the 'p' window only
MEM_KEYS = ['site_x', 'site_y', 'rel_x', 'rel_y', 'rel_z', 'vel_x', 'vel_y', 'vel_z', 'dwell_s', 'seat_x',
            'seat_y', 'u0', 'u1', 'u2_apex_m', 'y0', 'y1', 'y2_apex_m', 't_abs_s', 'ball_id', 'release_kind',
            'caught']


def _st(s):
    return s.sec + s.nanosec * 1e-9


def read_memory(pattern=MEM_GLOB):
    rows = []
    for f in sorted(glob.glob(pattern)):
        plant = os.path.basename(os.path.dirname(f))
        with open(f) as fh:
            for r in csv.DictReader(fh):
                if r.get('t_abs_s'):
                    r['plant'] = plant
                    rows.append(r)
    rows.sort(key=lambda r: float(r['t_abs_s']))
    return rows


# ---------------------------------------------------------------- extract
def extract(bag, out, mem_glob=MEM_GLOB):
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
    mt = np.array([float(r['t_abs_s']) for r in read_memory(mem_glob)])

    def inwin(t):
        i = np.searchsorted(mt, t)
        return (i < len(mt) and mt[i] - t < PRE_S) or (i > 0 and t - mt[i - 1] < POST_S)

    topics = ['/leg_cmd_executed', '/hand_telemetry', '/mocap_data', '/throw_announcements', '/clock_diag']
    # flat double arrays, not lists of lists: a 2.4 GB bag holds millions of marker rows and this box has 7 GB
    cmd, ht, plat, unl, ann, clk = (array('d') for _ in range(6))
    labels = {}
    df = DecoderFactory()
    dec = {}
    for f in sorted(glob.glob(os.path.join(bag, '*.mcap'))):
        with open(f, 'rb') as fh:
            for s, c, m in make_reader(fh).iter_messages(topics=topics):
                lt = m.log_time / 1e9
                tp = c.topic
                if tp in ('/mocap_data', '/leg_cmd_executed', '/hand_telemetry') and not inwin(lt):
                    continue
                d = dec.get(s.id) or dec.setdefault(s.id, df.decoder_for(c.message_encoding, s))
                d = d(m.data)
                if tp == '/leg_cmd_executed':
                    cmd.extend([lt, _st(d.header.stamp)] + list(d.position)[:6])
                elif tp == '/hand_telemetry':
                    bs = getattr(d, 'bridge_stamp', None)
                    ht.extend([lt, _st(d.timestamp), d.pos_cmd, d.vel_ff_cmd, d.pos_meas, d.vel_meas, d.iq_meas,
                               float(d.ball_held), _st(bs) if bs is not None else 0.0,
                               float(bool(getattr(d, 'bridge_stamp_valid', False))),
                               float(getattr(d, 'ball_held_raw', False)), float(getattr(d, 'ball_held_valid', False)),
                               _st(d.ball_held_stamp) if hasattr(d, 'ball_held_stamp') else 0.0])
                elif tp == '/mocap_data':
                    ts = _st(d.stamp)
                    for mk in d.markers:
                        p = mk.position
                        if mk.label.startswith('Platform'):
                            k = labels.setdefault(mk.label, len(labels))
                            plat.extend([ts, k, p.x, p.y, p.z, mk.residual])
                        elif mk.label == '':
                            unl.extend([ts, p.x, p.y, p.z, mk.residual, lt])
                elif tp == '/throw_announcements':
                    ann.extend([lt, _st(d.header.stamp), 1.0 if d.thrower_name == 'jugglebot' else 0.0,
                                d.initial_position.x, d.initial_position.y, d.initial_position.z,
                                d.initial_velocity.x, d.initial_velocity.y, d.initial_velocity.z,
                                _st(d.throw_time), d.predicted_tof_sec,
                                d.landing_position.x, d.landing_position.y, d.landing_position.z,
                                _st(d.landing_time), 1.0 if d.target_id == 'jugglebot' else 0.0])
                elif tp == '/clock_diag':
                    v = {kv.key: kv.value for kv in d.values}
                    try:
                        clk.extend([lt, float(v['jetson_wall_us']) / 1e6, float(v['err_us']) / 1e6])
                    except Exception:
                        pass
    A = lambda x, w: np.frombuffer(x, float).reshape(-1, w) if len(x) else np.zeros((0, w))  # noqa: E731
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    np.savez_compressed(out, cmd=A(cmd, 8), ht=A(ht, 13), plat=A(plat, 6), unl=A(unl, 6), ann=A(ann, 16),
                        clk=A(clk, 3), plat_labels=np.array(sorted(labels, key=labels.get)),
                        bag=np.array(os.path.basename(os.path.normpath(bag))))
    print('extract', os.path.basename(os.path.normpath(bag)),
          {k: len(v) // w for k, (v, w) in dict(cmd=(cmd, 8), ht=(ht, 13), plat=(plat, 6), unl=(unl, 6), ann=(ann, 16), clk=(clk, 3)).items()}, flush=True)


# ---------------------------------------------------------------- geometry
def kabsch_all(P, tmpl):
    """Rotation and template-centroid position of every Platform frame (NaN where < 3 markers)."""
    n = len(P)
    R = np.full((n, 3, 3), np.nan)
    c = np.full((n, 3), np.nan)
    vis = np.isfinite(P).all(2)
    code = vis @ (1 << np.arange(P.shape[1]))
    for cd in np.unique(code):
        ok = vis[np.argmax(code == cd)]
        if ok.sum() < 3:
            continue
        idx = np.where(code == cd)[0]
        A = tmpl[ok] - tmpl[ok].mean(0)
        X = P[idx][:, ok]
        B = X - X.mean(1, keepdims=True)
        H = np.einsum('ki,nkj->nij', A, B)
        U, _, Vt = np.linalg.svd(H)
        V = np.transpose(Vt, (0, 2, 1))
        dd = np.sign(np.linalg.det(V @ np.transpose(U, (0, 2, 1))))
        Dm = np.zeros((len(idx), 3, 3))
        Dm[:, 0, 0] = Dm[:, 1, 1] = 1
        Dm[:, 2, 2] = dd
        Rm = V @ Dm @ np.transpose(U, (0, 2, 1))
        R[idx] = Rm
        c[idx] = X.mean(1) - Rm @ tmpl[ok].mean(0) + Rm @ tmpl.mean(0)
    return R, c


def locfit(t, y, t0, half=0.045, deg=2):
    """Value and slope at t0 of a polynomial through the samples in [t0 - half, t0 + 1 ms]."""
    m = (t >= t0 - half) & (t <= t0 + 0.001) & np.isfinite(y)
    if m.sum() < deg + 2:
        return np.nan, np.nan
    co = np.polyfit(t[m] - t0, y[m], deg)
    return co[-1], co[-2]


def slope2(t, r):
    """Least-squares slope of a 2-D series with one 3-sigma rejection; (vx, vy, rms about trend)."""
    ok = np.isfinite(r).all(1)
    t, r = t[ok], r[ok]
    if len(t) < 5 or np.ptp(t) < 0.012:
        return np.nan, np.nan, np.nan
    for _ in range(2):
        X = np.c_[np.ones_like(t), t]
        b = np.linalg.lstsq(X, r, rcond=None)[0]
        e = r - X @ b
        d = np.hypot(e[:, 0], e[:, 1])
        s = np.sqrt(np.mean(d ** 2)) + 1e-9
        keep = d < 3 * s
        if keep.all() or keep.sum() < 5:
            break
        t, r = t[keep], r[keep]
    return b[1, 0], b[1, 1], float(np.sqrt(np.mean(e ** 2) * 2))


# ---------------------------------------------------------------- features
def features(npz, out_csv, traces=None, mem_glob=MEM_GLOB):
    from jugglebot.motion.geometry import StewartGeometry
    from jugglebot.motion import ik_solver as ik, conversions as cv, unified_cycle as uc
    from scipy.spatial.transform import Rotation as Rot
    D = np.load(npz, allow_pickle=True)
    bag = str(D['bag']) if 'bag' in D.files else os.path.basename(npz).replace('.npz', '')
    cmd, ht, plat, unl, ann, clk = (D[k] for k in ['cmd', 'ht', 'plat', 'unl', 'ann', 'clk'])
    geom = StewartGeometry()
    clk = clk[np.argsort(clk[:, 1])] if len(clk) else clk
    err_at = (lambda tj: np.interp(tj, clk[:, 1], clk[:, 2])) if len(clk) else (lambda tj: np.zeros_like(tj))
    cmd = cmd[np.unique(cmd[:, 1], return_index=True)[1]]
    tc = cmd[:, 1]
    ht = ht[np.argsort(ht[:, 0])]
    th, _ = bt.robot_state_bridge_time(ht[:, 1], ht[:, 0], ht[:, 8], ht[:, 9] > .5, cmd[:, 1], cmd[:, 0])
    ok = np.isfinite(th)
    ht, th = ht[ok], th[ok]
    o = np.argsort(th, kind='stable')
    ht, th = ht[o], th[o]
    keep_ = np.r_[True, np.diff(th) > 1e-6]            # paired stamps repeat; np.interp needs increasing x
    ht, th = ht[keep_], th[keep_]
    has_raw = ht.shape[1] >= 13
    plat = plat.copy()
    unl = unl.copy()
    plat[:, 0] = bt.mocap_time_on_bridge_clock(plat[:, 0], err_at(plat[:, 0]))
    unl[:, 0] = bt.mocap_time_on_bridge_clock(unl[:, 0], err_at(unl[:, 0]))
    unl = unl[np.argsort(unl[:, 0], kind='stable')]
    tu = unl[:, 0]
    nl = len(D['plat_labels'])
    fr_t, fi = np.unique(plat[:, 0], return_inverse=True)
    P = np.full((len(fr_t), nl, 3), np.nan)
    P[fi, plat[:, 1].astype(int)] = plat[:, 2:5]
    full = np.isfinite(P).all((1, 2))
    tmpl = P[np.where(full)[0][0]]
    Rk, ck = kabsch_all(P, tmpl)
    # every unlabelled marker in the Platform frame
    ufi = np.clip(np.searchsorted(fr_t, tu), 0, len(fr_t) - 1)
    ufi = np.where((ufi > 0) & (np.abs(fr_t[ufi - 1] - tu) < np.abs(fr_t[ufi] - tu)), ufi - 1, ufi)
    same = np.abs(fr_t[ufi] - tu) < 1e-4
    Q = np.full((len(unl), 3), np.nan)
    Q[same] = np.einsum('nji,nj->ni', Rk[ufi[same]], unl[same, 1:4] - ck[ufi[same]])

    def fk(revs):
        ext = cv.revs_to_extensions_mm(np.atleast_2d(revs), geom)
        out, guess = [], None
        for e in ext:
            try:
                p, Rm, _ = ik.leg_lengths_to_pose(e, geom, initial_guess=guess)
                guess = (p, Rm)
                out.append(np.r_[p, Rot.from_matrix(Rm).as_rotvec()])
            except Exception:
                out.append(np.full(6, np.nan))
        return np.array(out)

    mem = [r for r in read_memory(mem_glob) if len(tc) and tc[0] - 5 < float(r['t_abs_s']) < tc[-1] + 5]
    J = ann[ann[:, 2] > 0.5]
    rows, series = [], []
    for r in mem:
        tm_ = float(r['t_abs_s'])
        if not len(J):
            break
        near = np.where(np.abs(J[:, 9] - tm_) < 0.5)[0]
        if not len(near):
            continue
        a = J[near[np.argmax(J[near, 0])]]            # the last announcement of this Throw
        trel = a[9]
        f = dict(bag=bag, plant=r['plant'], trel=trel, dtm=tm_ - trel)
        for k in MEM_KEYS:
            f[k] = r[k]
        # --- commanded Cup state at release
        mc = (tc >= trel - 0.06) & (tc <= trel + 0.002)
        mh = (th >= trel - 0.2) & (th <= trel + 0.1)
        if mc.sum() >= 4 and mh.sum() >= 8:
            pc = fk(cmd[mc, 2:8])
            hcmd = np.interp(tc[mc], th[mh], ht[mh, 2])
            cc = np.array([uc.cup_state_from_platform(p, h) for p, h in zip(pc, hcmd)])
            for j, ax in enumerate('xy'):
                f['cup%s_c' % ax], f['vcup%s_c' % ax] = locfit(tc[mc], cc[:, j], trel)
                f['p%s_c' % ax], f['vp%s_c' % ax] = locfit(tc[mc], pc[:, j], trel)
                f['r%s_c' % ax], f['wr%s_c' % ax] = locfit(tc[mc], pc[:, 3 + j], trel)
            # the same velocity with the Hand term taken from the commanded Hand velocity (see the logbook entry)
            h0 = float(np.interp(trel, th[mh], ht[mh, 2]))
            vh = float(np.interp(trel, th[mh], ht[mh, 3]))
            c0 = np.array([uc.cup_state_from_platform(p, h0) for p in pc])
            k_ = int(np.argmin(np.abs(tc[mc] - trel)))
            dcdh = (np.array(uc.cup_state_from_platform(pc[k_], h0 + 0.5)) - np.array(uc.cup_state_from_platform(pc[k_], h0 - 0.5)))
            for j, ax in enumerate('xy'):
                f['vcup%s_a' % ax] = locfit(tc[mc], c0[:, j], trel)[1] + vh * dcdh[j]
            f['vh_c'] = vh
        # --- measured Platform at release (Mocap)
        i0, i1 = np.searchsorted(fr_t, [trel - 0.035, trel + 0.002])
        if i1 - i0 > 8 and np.isfinite(ck[i0:i1, 0]).sum() > 8:
            for j, ax in enumerate('xy'):
                f['mn' + ax], f['mwn' + ax] = locfit(fr_t[i0:i1], Rk[i0:i1, j, 2], trel, 0.03)
                f['mc' + ax], f['mvc' + ax] = locfit(fr_t[i0:i1], ck[i0:i1, j], trel, 0.03)
        ir = min(np.searchsorted(fr_t, trel), len(fr_t) - 1)
        Rrel = Rk[ir] if abs(fr_t[ir] - trel) < 0.02 else np.full((3, 3), np.nan)
        # --- launch velocity (flight)
        u0, u1 = np.searchsorted(tu, [trel + FLIGHT[0], trel + 0.25])
        U = unl[u0:u1]
        tau = U[:, 0] - trel
        sel = np.zeros(len(U), bool)
        if len(U) >= 20:
            pred = a[3:6] + np.outer(tau, a[6:9])
            pred[:, 2] -= 0.5 * G * tau ** 2
            sel = np.linalg.norm(U[:, 1:4] - pred, axis=1) < 80
            for gate in (40, 15, 6):
                if sel.sum() < 12:
                    break
                tt = tau[sel]
                X = np.c_[np.ones_like(tt), tt, 0.5 * tt ** 2]
                cx = np.linalg.lstsq(X, U[sel, 1], rcond=None)[0]
                cy = np.linalg.lstsq(X, U[sel, 2], rcond=None)[0]
                cz = np.linalg.lstsq(X[:, :2], U[sel, 3] + 0.5 * G * tt ** 2, rcond=None)[0]
                Xa = np.c_[np.ones_like(tau), tau, 0.5 * tau ** 2]
                fit = np.c_[Xa @ cx, Xa @ cy, cz[0] + cz[1] * tau - 0.5 * G * tau ** 2]
                sel = np.linalg.norm(U[:, 1:4] - fit, axis=1) < gate
        s0 = sel & (tau < FLIGHT[1])
        f['fl_n'] = int(s0.sum())
        if s0.sum() >= 12:
            f['v0x'] = np.polyfit(tau[s0], U[s0, 1], 1)[0]
            f['v0y'] = np.polyfit(tau[s0], U[s0, 2], 1)[0]
            cz = np.polyfit(tau[s0], U[s0, 3] + 0.5 * G * tau[s0] ** 2, 1)
            f['v0z'], f['z0'] = cz[0], cz[1]
            f['x0'] = np.polyfit(tau[s0], U[s0, 1], 1)[1]
            f['y0f'] = np.polyfit(tau[s0], U[s0, 2], 1)[1]
        if s0.sum() >= 12:                             # the ball's own peak upward velocity, against the announced release
            v1 = slice(*np.searchsorted(tu, [trel - 0.07, trel + 0.06]))
            tb, Ub = tu[v1] - trel, unl[v1, 1:4]
            zf = f['z0'] + f['v0z'] * tb - .5 * G * tb ** 2
            mb = (np.hypot(Ub[:, 0] - f['x0'] - f['v0x'] * tb, Ub[:, 1] - f['y0f'] - f['v0y'] * tb) < 25) & (Ub[:, 2] < zf + 30) & (Ub[:, 2] > zf - 400)
            tb, zb = tb[mb], Ub[mb, 2]
            if len(tb) >= 25:
                vz = np.array([np.polyfit(tb[i - 3:i + 4], zb[i - 3:i + 4], 1)[0] for i in range(3, len(tb) - 3)])
                f['t_vpk'] = float(tb[3:-3][np.argmax(vz)])
                f['vz_pk'] = float(vz.max())
        # --- candidates for the in-Cup series (reduced after the Hand axis is known)
        w0, w1 = np.searchsorted(tu, [trel - LOOK_S, trel + POST_LOOK_S])
        f0, f1 = np.searchsorted(fr_t, [trel - LOOK_S, trel + POST_LOOK_S])
        mh2 = (th >= trel - LOOK_S - .1) & (th <= trel + .1)
        hpos = np.interp(tu[w0:w1], th[mh2], ht[mh2, 4]) if mh2.sum() > 8 else np.full(w1 - w0, np.nan)
        hvel = np.interp(tu[w0:w1], th[mh2], ht[mh2, 5]) if mh2.sum() > 8 else np.full(w1 - w0, np.nan)
        seed = None
        if s0.sum() >= 12:                             # the ball just before release, by back-extrapolation
            v1 = slice(*np.searchsorted(tu, [trel - 0.02, trel]))
            tb = tu[v1] - trel
            ext = np.c_[f['x0'] + f['v0x'] * tb, f['y0f'] + f['v0y'] * tb, f['z0'] + f['v0z'] * tb - .5 * G * tb ** 2]
            ms = (np.linalg.norm(unl[v1, 1:3] - ext[:, :2], axis=1) < 25) & (np.abs(unl[v1, 3] - ext[:, 2]) < 120)
            seed = np.c_[Q[v1][ms], ht_at(th, ht, tu[v1][ms])] if ms.any() else None
        series.append(dict(tau=tu[w0:w1] - trel, q=Q[w0:w1], hpos=hpos, hvel=hvel, nfr=f1 - f0, Rrel=Rrel, seed=seed,
                           tfr=fr_t[f0:f1] - trel))
        # --- Hand ball sensor
        ms_ = (th >= trel - LOOK_S) & (th <= trel - 0.02)
        if ms_.sum() > 10:
            held = ht[ms_, 7] > .5
            ts = th[ms_] - trel
            lows = np.where(~held)[0]
            f['sens_t_held'] = -ts[lows[-1]] if len(lows) else LOOK_S
            if has_raw:
                raw = ht[ms_, 10] > .5
                valid = ht[ms_, 11] > .5
                f['sens_valid_frac'] = valid.mean()
                m5 = ts > -0.5
                f['sens_flips_500'] = int(np.abs(np.diff(raw[m5].astype(int))).sum())
                f['sens_rawlow_200'] = float((~raw[ts > -0.2]).mean()) if (ts > -0.2).any() else np.nan
                rl = np.where(~raw)[0]
                f['sens_t_raw'] = -ts[rl[-1]] if len(rl) else LOOK_S
        rows.append(f)

    # --- Hand axis (a line in the Platform frame) and Cup height against Hand position, per bag
    seeds = [s['seed'] for s in series if s['seed'] is not None and len(s['seed'])]
    axis = None
    if len(seeds) >= 5:
        S = np.vstack(seeds)
        S = S[np.isfinite(S).all(1)]
        keep = np.ones(len(S), bool)
        for _ in range(4):
            A = np.c_[np.ones(keep.sum()), S[keep, 2]]
            bx = np.linalg.lstsq(A, S[keep, 0], rcond=None)[0]
            by = np.linalg.lstsq(A, S[keep, 1], rcond=None)[0]
            kz = HAND_MM_PER_REV / np.sqrt(1 + bx[1] ** 2 + by[1] ** 2)
            bz = np.array([kz, np.median(S[keep, 2] - kz * S[keep, 3])])
            e = np.hypot(S[:, 0] - bx[0] - bx[1] * S[:, 2], S[:, 1] - by[0] - by[1] * S[:, 2])
            keep = (e < 3 * np.median(e[keep]) + 1.0) & (np.abs(S[:, 2] - np.polyval(bz, S[:, 3])) < 80)
        # Cup height against Hand position: the offset comes from samples taken while the Hand is slow,
        # where the telemetry stamp and encoder age cannot bias it
        zr = []
        for s in series:
            dl = np.hypot(s['q'][:, 0] - bx[0] - bx[1] * s['q'][:, 2], s['q'][:, 1] - by[0] - by[1] * s['q'][:, 2])
            m = (dl < LAT_GATE_MM) & (np.abs(s['hvel']) < 3.0) & (s['tau'] > -0.4)
            zr.append(s['q'][m, 2] - kz * s['hpos'][m])
        zr = np.concatenate(zr) if zr else np.zeros(0)
        zr = zr[np.isfinite(zr)]
        if len(zr) >= 50:
            h, e = np.histogram(zr, bins=np.arange(zr.min() - 5, zr.max() + 10, 5.0))
            mode = e[np.argmax(h)] + 2.5
            bz = np.array([kz, np.median(zr[np.abs(zr - mode) < 20])])
        axis = (bx, by, bz)
    for f, s in zip(rows, series):
        f['nfr'] = s['nfr']
        s['lat'] = None
        if axis is None or not np.isfinite(s['Rrel']).all():
            continue
        bx, by, bz = axis
        q, tau = s['q'], s['tau']
        ahat = np.array([bx[1], by[1], 1.0])
        ahat /= np.linalg.norm(ahat)
        d = np.c_[q[:, 0] - bx[0] - bx[1] * q[:, 2], q[:, 1] - by[0] - by[1] * q[:, 2], np.zeros(len(q))]
        d = d - np.outer(d @ ahat, ahat)
        lat = np.linalg.norm(d, axis=1)
        dz = q[:, 2] - np.polyval(bz, s['hpos'])
        gate = AX_GATE_MM + STAMP_SLACK_S * np.abs(s['hvel']) * HAND_MM_PER_REV
        axial_ok = np.where(tau < 0, np.abs(dz) < 3 * gate, (dz > -3 * gate) & (dz < 250.0))
        cand = np.where((lat < LAT_GATE_MM) & np.isfinite(lat) & axial_ok)[0]
        if not len(cand):
            continue
        # one sample per frame: the nearest to the axis
        _, first = np.unique(np.round(tau[cand] / 1e-4).astype(np.int64), return_index=True)
        pick = []
        for g_ in np.split(np.arange(len(cand)), first[1:]):
            pick.append(cand[g_[np.argmin(lat[cand[g_]])]])
        pick = np.array(pick)
        f['amb_frames'] = int(len(cand) - len(pick))
        t_, dw, dz_, z_ = tau[pick], (d[pick] @ s['Rrel'].T)[:, :2], dz[pick], q[pick, 2]
        # seated run: back from release while the axial offset stays inside the gate and gaps stay short
        seated = np.abs(dz_) < gate[pick]
        npre = int((t_ < 0).sum())
        k = npre - 1
        if k < 0 or not seated[k] or t_[k] < -0.03:
            f['t_seat'] = 0.0
            k = max(k, 0)
        else:
            while k > 0 and seated[k - 1] and t_[k] - t_[k - 1] < 0.06:
                k -= 1
            f['t_seat'] = float(-t_[k]) if t_[k] > -(LOOK_S - 0.02) else LOOK_S
        run = slice(k, len(t_))
        # the Catch: where the ball came in against the Hand axis (above the rim, so unoccluded), and
        # how far it lifted off the seat afterwards beyond what the Hand telemetry stamp could explain
        if 0 < f['t_seat'] < LOOK_S and k >= 3:
            e_ = (t_[:k] >= t_[k] - 0.02) & (dz_[:k] > 10.0)
            if e_.sum() >= 3:
                f['entry_x'], f['entry_y'] = dw[:k][e_].mean(0)
                f['entry_r'] = float(np.hypot(f['entry_x'], f['entry_y']))
        mid = (np.arange(len(t_)) >= k) & (t_ > -f['t_seat'] + 0.03) & (t_ < -0.08)
        if mid.sum() >= 5:
            lift = dz_[mid] - (gate[pick][mid] - AX_GATE_MM)
            f['ax_lift_max'] = float(max(lift.max(), 0.0))
            f['ax_lift_rms'] = float(np.sqrt(np.mean((dz_[mid] - np.median(dz_[mid])) ** 2)))
        s['lat'] = (t_[run], dw[run], z_[run])
        for nm, (a_, b_) in WINDOWS.items():
            f['n_' + nm] = int(((t_[run] >= a_) & (t_[run] < b_)).sum())
        f['from_rest'] = int(f['t_seat'] >= REST_S)

    # --- templates per trajectory group, residuals and the measures
    grid = np.arange(-LOOK_S, POST_LOOK_S + GRID / 2, GRID)

    def on_grid(t, y):
        out = np.full((len(grid),) + y.shape[1:], np.nan)
        if len(t) < 2:
            return out
        j = np.clip(np.searchsorted(t, grid), 1, len(t) - 1)
        nearest = np.minimum(np.abs(t[j] - grid), np.abs(t[j - 1] - grid)) < 0.005
        w = ((grid - t[j - 1]) / np.maximum(t[j] - t[j - 1], 1e-9))[:, None]
        y2 = y.reshape(len(t), -1)
        val = y2[j - 1] * (1 - w) + y2[j] * w
        ok_ = nearest & (grid >= t[0]) & (grid <= t[-1]) & (t[j] - t[j - 1] < 0.02)
        out.reshape(len(grid), -1)[ok_] = val[ok_]
        return out

    key = lambda f: (f['site_x'], f['site_y'], f['release_kind'], f['dwell_s'])  # noqa: E731
    G_ = {}
    for i, (f, s) in enumerate(zip(rows, series)):
        if s['lat'] is not None:
            t_, dw, z_ = s['lat']
            s['g'] = on_grid(t_, np.c_[dw, z_])
            if not f.get('from_rest'):
                G_.setdefault(key(f), []).append(i)
    tr_out = {}
    for i, (f, s) in enumerate(zip(rows, series)):
        if s['lat'] is None:
            continue
        g_ = s['g']
        res = np.full((len(grid), 2), np.nan)
        still = np.zeros(len(grid), bool)
        members = [j for j in G_.get(key(f), []) if j != i]          # leave this Throw out of its own template
        f['tmpl_n'] = len(members)
        if len(members) >= 6:
            stack = np.stack([series[j]['g'] for j in members])
            cnt = np.isfinite(stack[:, :, 0]).sum(0)
            with np.errstate(all='ignore'):
                med = np.nanmedian(stack, axis=0)
                mad = np.nanmedian(np.abs(stack[:, :, :2] - med[None, :, :2]), axis=0).sum(1)
            good = (cnt >= 6) & (mad < 3 * np.nanmedian(mad[cnt >= 6]) + 0.05) & (np.abs(g_[:, 2] - med[:, 2]) < 8.0)
            res[good] = g_[good, :2] - med[good, :2]
        # a still Hand needs no template: the offset minus its own median
        zz = g_[:, 2]
        for a_ in range(0, len(grid) - 30, 15):
            sl = slice(a_, a_ + 30)
            if np.isfinite(zz[sl]).sum() >= 24 and np.nanmax(zz[sl]) - np.nanmin(zz[sl]) < 2.0:
                still[sl] = True
        if f.get('from_rest') and still.any():
            res[still] = g_[still, :2] - np.nanmedian(g_[still, :2], axis=0)
            f['still_frac_200'] = float(still[(grid >= -0.2) & (grid < 0)].mean())
        s['res'] = res
        tr_out[i] = np.c_[g_, res]
        for nm, (a_, b_) in WINDOWS.items():
            m = (grid >= max(a_, -f['t_seat'] + 0.02)) & (grid < b_)
            span = b_ - max(a_, -f['t_seat'] + 0.02)
            if span < 0.6 * (b_ - a_):
                continue
            okk = m & np.isfinite(res[:, 0])
            f['c_' + nm] = round(float(okk.sum() / max(m.sum(), 1)), 3)
            if okk.sum() < 0.5 * m.sum():
                continue
            vx, vy, rms = slope2(grid[okk], res[okk])
            f['v%s_x' % nm], f['v%s_y' % nm], f['wig_' + nm] = vx, vy, rms
            f['off%s_x' % nm], f['off%s_y' % nm] = np.nanmean(res[okk], axis=0)
            if okk.sum() >= 12:
                sm = np.c_[np.convolve(res[okk, 0], np.ones(5) / 5, 'valid'), np.convolve(res[okk, 1], np.ones(5) / 5, 'valid')]
                dt_ = np.convolve(grid[okk], np.ones(5) / 5, 'valid')
                dd = np.diff(sm, axis=0) / np.maximum(np.diff(dt_), 1e-4)[:, None]
                f['spd_' + nm] = float(np.sqrt(np.mean((dd ** 2).sum(1))))
    keys = []
    for f in rows:
        for k in f:
            if k not in keys:
                keys.append(k)
    os.makedirs(os.path.dirname(os.path.abspath(out_csv)), exist_ok=True)
    with open(out_csv, 'w', newline='') as fo:
        w = csv.DictWriter(fo, keys)
        w.writeheader()
        for f in rows:
            w.writerow(f)
    if traces:
        idx = sorted(tr_out)
        np.savez_compressed(traces, grid=grid, idx=np.array(idx), tr=np.stack([tr_out[i] for i in idx]) if idx else np.zeros((0, len(grid), 5)))
    n_ax = 0 if axis is None else 1
    print('features', bag, 'throws', len(rows), 'launch fit', sum('v0x' in f for f in rows), 'axis', n_ax,
          'in-Cup series', sum(s.get('lat') is not None for s in series),
          'from rest', sum(f.get('from_rest', 0) for f in rows), 'v100', sum('v100_x' in f for f in rows), flush=True)


def ht_at(th, ht, t):
    return np.interp(t, th, ht[:, 4])


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('mode', choices=['extract', 'features', 'run'])
    ap.add_argument('src')
    ap.add_argument('--out')
    ap.add_argument('--traces')
    ap.add_argument('--memory', default=MEM_GLOB)
    a = ap.parse_args()
    name = os.path.basename(os.path.normpath(a.src)).replace('.npz', '')
    if a.mode in ('extract', 'run'):
        npz = (a.out if a.mode == 'extract' and a.out else os.path.join(OUT_DIR, name + '.npz'))
        extract(a.src, npz, a.memory)
        if a.mode == 'extract':
            return
        a.src = npz
    features(a.src, a.out if a.mode == 'features' and a.out else os.path.join(OUT_DIR, name + '.csv'),
             a.traces or os.path.join(OUT_DIR, name + '_traces.npz'), a.memory)


if __name__ == '__main__':
    main()
