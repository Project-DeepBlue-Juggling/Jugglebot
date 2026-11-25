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
(plan `plans/active/learner-efficacy.md`, Phases E0 and E1). Consuming test:
`tests/sim/test_release_settle_blocks.py` (the `blocks` mode's pure functions on synthetic
rows); `extract` and `features` have none (run on demand).

USAGE
-----
    python tools/probes/release_settle_bag_probe.py extract  BAGDIR  [--out NPZ]
    python tools/probes/release_settle_bag_probe.py features NPZ     [--out CSV] [--traces NPZ]
    python tools/probes/release_settle_bag_probe.py run      BAGDIR          # both
    python tools/probes/release_settle_bag_probe.py blocks   CSV [CSV ...] [--attempt-gap-s 3.0]
                                                             [--skip-attempts N] [--out JSON]

BLOCKS (FEAT-67: the report FEAT-68's verdict is read from)
------
`blocks` reads the per-Throw CSVs of ONE Sitting and prints the launch scatter per block.
- Attempts: a new Attempt at a new bag or when consecutive `trel` are more than
  `--attempt-gap-s` apart (3.0 s). Within an Attempt Throws are at most about 2 s apart at
  Dwell 1.00; a hand-placed single Throw of block S follows the previous landing by the
  operator's 3 s count plus the goal's round trip, which can be under 6 s, so E0's 6 s rule
  (the 2026-11-23 review's, made for long running Attempts) could merge two S Attempts into
  one `first` + `D27` pair. On the E0 bag 2026-11-06 15:53 the two rules give the same 19
  Attempts (within-Attempt gaps 0.5 to 1.5 s, between Attempts 12 s or more).
- Blocks, by the goal and never by `from_rest` (which fires on running Throws at Dwell 1.00):
  `S` the only Throw of a one-Throw Attempt seated >= 1 s (`t_seat` or `sens_t_held`);
  `other` a one-Throw Attempt that was not; `first` Throw 1 of a longer Attempt (its count
  seated 1 s is printed: on the E0 bags a long Attempt can open on a Fed ball, not from
  rest); `D<nn>` Throw 2 or later by Memory's `dwell_s` rounded to 0.01 s.
  `--skip-attempts N` drops the first N Attempts (FEAT-68's uncounted warm-up).
- The figure: the sd per axis of the lateral launch velocity `v0x`, `v0y` about the mean of
  its cell (block, Site, release kind; one Site in FEAT-68, so the block mean), raw and
  Platform-corrected. The correction is ONE least-squares fit pooled over every Throw of
  the Sitting, both sides centred per cell, of `v0` on the commanded Cup velocity
  (`vcupx_a`, `vcupy_a`) and the Platform's measured Mocap centroid velocity, normal and
  normal rate (`mvcx`, `mvcy`, `mnx`, `mny`, `mwnx`, `mwny`): 8 slopes per axis. E0's
  two-stage ten-regressor fit (commanded Platform terms regressed out of the measured ones
  first) is not carried by 30 Throws per block; one pooled fit is, and the residual it
  leaves is what is left after the measured Platform motion and the command. Degrees of
  freedom are corrected exactly, not neglected: each Throw carries w = 1 - 1/n_cell - h
  (h the hat value of the cell-centred regressors), and a block's variance is
  sum(e^2) / sum(w), unbiased under equal noise. With FEAT-68's ~240 Throws and 8 slopes the
  loss is about 3 % of the variance; a cell of a few Throws (`first` on the E0 bags, 19 Throws in
  7 Sites) loses far more, which the plain rms would hide. A Throw missing a regressor
  enters the raw figure only and is counted.
- Uncertainty: a cluster bootstrap by Attempt (2000 resamples, seed 0), stratified by the
  set of blocks an Attempt holds, over the fitted residuals (see `blocks_report`), giving
  a 95 % interval on each block's pooled-axis sd and on its variance ratio (both axes
  pooled) against block S, or against `first` when there is no S.
- Verdict, on the corrected figure (FEAT-68, pre-registered 2026-11-25): Seating (S <= 6
  mm/s on both axes AND D27 >= 2x S's variance AND the ratios falling as Dwell grows),
  Separation (S >= 12 mm/s on both axes AND every Dwell block within 1.5x), else Mixed with
  the separation share (1 / ratio) and seating share per Dwell block. A deciding ratio
  whose interval straddles its threshold makes the clause UNDECIDED. No S: no verdict.
- D27 split in thirds of the Catch entry offset `entry_r`, corrected sd per third. The
  entry offset was NOT validated in E0: a lead, not a measure.
Same input, same output (fixed seed). `--out` writes the numbers as JSON; print only otherwise.

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


# ---------------------------------------------------------------- blocks (FEAT-67, learner-efficacy E1)
# Pure functions: rows in (dicts as csv.DictReader gives them), numbers out; no file I/O below this line
# until read_records / main.
ATTEMPT_GAP_S = 3.0              # a gap longer than this between consecutive Throws starts a new Attempt
SEATED_S = 1.0                   # block S needs the ball seated this long (Mocap t_seat or the Hand ball sensor)
BLOCK_REGRESSORS = ['vcupx_a', 'vcupy_a', 'mvcx', 'mvcy', 'mnx', 'mny', 'mwnx', 'mwny']
N_BOOT, BOOT_SEED = 2000, 0
SEATING_SD, SEATING_RATIO = 6.0, 2.0          # FEAT-68 criterion, pre-registered 2026-11-25
SEPARATION_SD, SEPARATION_RATIO = 12.0, 1.5


def _num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return np.nan


def _seated(r):
    return bool(_num(r.get('t_seat')) >= SEATED_S or _num(r.get('sens_t_held')) >= SEATED_S)


def segment_attempts(rows, gap_s=ATTEMPT_GAP_S):
    """Rows sorted by (bag, trel), the Attempt id of each (from 0) and its Throw index in the Attempt (from 1).

    A new Attempt starts at a new bag or when `trel` moves on by more than `gap_s` since the previous Throw."""
    rows = sorted(rows, key=lambda r: (str(r.get('bag', '')), _num(r.get('trel'))))
    att, idx = np.zeros(len(rows), int), np.zeros(len(rows), int)
    a, i, prev = -1, 0, None
    for j, r in enumerate(rows):
        t = _num(r.get('trel'))
        if prev is None or r.get('bag') != prev[0] or not (t - prev[1] <= gap_s):
            a, i = a + 1, 0
        i += 1
        att[j], idx[j], prev = a, i, (r.get('bag'), t)
    return rows, att, idx


def dwell_block(dwell_s):
    """`D<nn>` from Memory's dwell_s rounded to 0.01 s (0.27 -> D27, 1.0 -> D100)."""
    d = _num(dwell_s)
    return 'D%d' % int(round(d * 100)) if np.isfinite(d) else 'other'


def label_blocks(rows, gap_s=ATTEMPT_GAP_S, skip_attempts=0):
    """(rows sorted, Attempt id, Throw index, block label) by the goal, never by the probe's `from_rest` flag.

    S: the only Throw of a one-Throw Attempt, seated >= 1 s. other: a one-Throw Attempt that was not.
    first: Throw 1 of a longer Attempt. D<nn>: Throw 2 or later, by Memory's dwell_s. The first
    `skip_attempts` Attempts (a warm-up) are labelled `skipped`."""
    rows, att, idx = segment_attempts(rows, gap_s)
    size = np.bincount(att)[att] if len(att) else np.zeros(0, int)
    lab = []
    for r, a, i, n in zip(rows, att, idx, size):
        if a < skip_attempts:
            lab.append('skipped')
        elif n == 1:
            lab.append('S' if _seated(r) else 'other')
        elif i == 1:
            lab.append('first')
        else:
            lab.append(dwell_block(r.get('dwell_s')))
    return rows, att, idx, np.array(lab, dtype=object)


def centred_residuals(Y, X, cell):
    """Residuals of a fixed-effect fit: a mean per cell plus ONE set of slopes on X pooled over every row.

    Returns (e, w, rank): e the residual per row (n, Y columns); w the degrees of freedom each row carries,
    w_i = 1 - 1/n_cell(i) - h_ii (h the hat matrix of the cell-centred X), so that E[e_i^2] = sigma^2 w_i and
    sum(e^2) / sum(w) over any subset is the unbiased variance of that subset. X None or 0 columns: raw."""
    Y = np.asarray(Y, float)
    _, inv, cnt = np.unique(np.asarray(cell), return_inverse=True, return_counts=True)

    def centre(A):
        s = np.stack([np.bincount(inv, A[:, j], len(cnt)) for j in range(A.shape[1])], 1)
        return A - (s / cnt[:, None])[inv]
    e = centre(Y)
    w = 1.0 - 1.0 / cnt[inv]
    rank = 0
    if X is not None and np.asarray(X).shape[1]:
        Xc = centre(np.asarray(X, float))
        sc = Xc.std(0)
        Xc = Xc[:, sc > 0] / sc[sc > 0]                      # unit scale: Platform tilt is ~1e-3, Cup velocity ~1e2
        if Xc.shape[1]:
            U, s, _ = np.linalg.svd(Xc, full_matrices=False)
            rank = int((s > s[0] * 1e-8).sum()) if s[0] > 0 else 0
            U = U[:, :rank]
            e = e - U @ (U.T @ e)
            w = w - (U ** 2).sum(1)
    return e, w, rank


def block_variances(e, w, lab, blocks):
    """Per block, the unbiased variance per axis: sum(e^2) / sum(w); NaN below 2 degrees of freedom."""
    out = {}
    for b in blocks:
        m = lab == b
        dof = float(w[m].sum())
        out[b] = (e[m] ** 2).sum(0) / dof if dof >= 2 else np.full(e.shape[1], np.nan)
    return out


def _ratio_ref(blocks):
    return 'S' if 'S' in blocks else ('first' if 'first' in blocks else None)


def _dwell_blocks(blocks):
    return sorted([b for b in blocks if b.startswith('D') and b[1:].isdigit()], key=lambda b: int(b[1:]))


def verdict(s_sd, dwell):
    """FEAT-68's criterion on the Platform-corrected figure. s_sd: block S's sd per axis (mm/s) or None;
    dwell: [(label, variance ratio against S, bootstrap lo, hi)] sorted by Dwell.
    Returns (word, the clause that fired); word in Seating / Separation / Mixed / UNDECIDED / n/a."""
    if s_sd is None or not np.all(np.isfinite(s_sd)):
        return 'n/a', 'no settled block: verdict not available'
    sx, sy = s_sd
    shares = ', '.join('%s separation %.2f / seating %.2f' % (b, min(1.0, 1 / r), max(0.0, 1 - 1 / r))
                       for b, r, _, _ in dwell if np.isfinite(r) and r > 0)
    if max(sx, sy) <= SEATING_SD:
        d27 = [x for x in dwell if x[0] == 'D27']
        if not d27:
            return 'UNDECIDED', 'Seating clause: S <= %g mm/s per axis but there is no D27 block' % SEATING_SD
        _, r, lo, hi = d27[0]
        rs = [x[1] for x in dwell]
        falling = all(a >= b for a, b in zip(rs, rs[1:]))
        if lo < SEATING_RATIO < hi:
            return 'UNDECIDED', ('Seating clause: S %.1f/%.1f mm/s <= %g, but the D27/S variance ratio %.2f (95 %% %.2f..%.2f) '
                                 'straddles %g' % (sx, sy, SEATING_SD, r, lo, hi, SEATING_RATIO))
        if r >= SEATING_RATIO and falling:
            return 'Seating', ('S %.1f/%.1f mm/s <= %g AND D27/S %.2f (95 %% %.2f..%.2f) >= %g AND falling with Dwell (%s)'
                               % (sx, sy, SEATING_SD, r, lo, hi, SEATING_RATIO, ' >= '.join('%.2f' % x for x in rs)))
        why = 'D27/S %.2f < %g' % (r, SEATING_RATIO) if r < SEATING_RATIO else 'ratios not falling with Dwell (%s)' % ', '.join('%.2f' % x for x in rs)
        return 'Mixed', 'S %.1f/%.1f mm/s <= %g but %s; shares: %s' % (sx, sy, SEATING_SD, why, shares)
    if min(sx, sy) >= SEPARATION_SD:
        if not dwell:
            return 'UNDECIDED', 'Separation clause: S >= %g mm/s per axis but there is no Dwell block' % SEPARATION_SD
        txt = ', '.join('%s %.2f (95 %% %.2f..%.2f)' % x for x in dwell)
        if all(r <= SEPARATION_RATIO and not (lo < SEPARATION_RATIO < hi) for _, r, lo, hi in dwell):
            return 'Separation', 'S %.1f/%.1f mm/s >= %g AND every Dwell block within %gx S: %s' % (sx, sy, SEPARATION_SD, SEPARATION_RATIO, txt)
        if any(lo > SEPARATION_RATIO for _, _, lo, _ in dwell):
            return 'Mixed', 'S %.1f/%.1f mm/s >= %g but a Dwell block exceeds %gx S: %s; shares: %s' % (sx, sy, SEPARATION_SD, SEPARATION_RATIO, txt, shares)
        return 'UNDECIDED', 'Separation clause: S %.1f/%.1f mm/s >= %g, but a ratio straddles %g: %s' % (sx, sy, SEPARATION_SD, SEPARATION_RATIO, txt)
    return 'Mixed', 'S %.1f/%.1f mm/s is between %g and %g; shares: %s' % (sx, sy, SEATING_SD, SEPARATION_SD, shares or 'no Dwell block')


def blocks_report(rows, gap_s=ATTEMPT_GAP_S, skip_attempts=0, regressors=BLOCK_REGRESSORS, n_boot=N_BOOT, seed=BOOT_SEED):
    """The per-block launch-scatter report of one Sitting as a plain dict (see the module docstring, BLOCKS)."""
    rows, att, idx, lab = label_blocks(rows, gap_s, skip_attempts)
    keep = lab != 'skipped'
    n_skipped = int((~keep).sum())
    rows = [r for r, k in zip(rows, keep) if k]
    att, idx, lab = att[keep], idx[keep], lab[keep]
    order = ['S', 'first'] + _dwell_blocks(set(lab)) + ['other']
    blocks = [b for b in order if b in set(lab)]
    v = np.array([[_num(r.get('v0x')), _num(r.get('v0y'))] for r in rows], float).reshape(-1, 2)
    X = np.array([[_num(r.get(c)) for c in regressors] for r in rows], float).reshape(-1, len(regressors))
    cell = np.array(['%s|%.4f|%.4f|%s' % (b, _num(r.get('site_x')), _num(r.get('site_y')), r.get('release_kind', ''))
                     for b, r in zip(lab, rows)], dtype=object)
    fit = np.isfinite(v).all(1)
    cor = fit & np.isfinite(X).all(1)
    e_raw, w_raw, _ = centred_residuals(v[fit], None, cell[fit])
    e_c, w_c, rank = centred_residuals(v[cor], X[cor], cell[cor])
    var_raw = block_variances(e_raw, w_raw, lab[fit], blocks)
    var_c = block_variances(e_c, w_c, lab[cor], blocks)
    ref = _ratio_ref([b for b in blocks if b != 'other'])
    pooled = lambda vv: float(np.sqrt(np.mean(vv)))  # noqa: E731

    # Cluster bootstrap by Attempt, stratified by the set of blocks an Attempt holds (one-Throw S Attempts are
    # resampled among themselves, each Dwell's Attempts among themselves), so every block keeps its size. The
    # residuals and their degrees of freedom are fitted ONCE and the resample is over (e^2, w) per Throw, a
    # ratio estimator sum(e^2) / sum(w): refitting the cell means on a resample treats an Attempt's copies as
    # independent Throws, which drives a small block's variance low (on the E0 bag 2026-11-06 15:53 `first`,
    # 19 Throws in 7 cells, it put the point estimate at the top of its own interval).
    a_c = att[cor]
    lab_c = lab[cor]
    e2 = (e_c ** 2).mean(1)                          # both axes pooled
    strata = {}
    for a in np.unique(att):
        strata.setdefault(tuple(sorted(set(lab[att == a]))), []).append(a)
    bi = {b: i for i, b in enumerate(blocks)}
    li = np.array([bi[x] for x in lab_c], int)
    att_list = np.unique(att)
    # per Attempt and block: summed e^2 and summed w, so a resample is a weighted sum over Attempts
    se = np.zeros((len(att_list), len(blocks)))
    sw = np.zeros((len(att_list), len(blocks)))
    ai = np.searchsorted(att_list, a_c)
    np.add.at(se, (ai, li), e2)
    np.add.at(sw, (ai, li), w_c)
    rs = np.random.RandomState(seed)
    cnt = np.zeros((n_boot, len(att_list)))
    for key in sorted(strata):
        m = np.searchsorted(att_list, strata[key])
        for k in range(n_boot):
            cnt[k] += np.bincount(m[rs.randint(0, len(m), len(m))], minlength=len(att_list))
    with np.errstate(all='ignore'):
        dof_b = cnt @ sw
        vb = np.where(dof_b >= 2, (cnt @ se) / dof_b, np.nan)
    # a block without a point estimate (under 2 degrees of freedom) gets no interval either
    boot = {b: vb[:, bi[b]] if np.isfinite(np.mean(var_c[b])) else np.full(n_boot, np.nan) for b in blocks}

    def ci(x):
        x = x[np.isfinite(x)]
        return [float(q) for q in np.percentile(x, [2.5, 97.5])] if len(x) >= 20 else [np.nan, np.nan]

    out = dict(attempt_gap_s=gap_s, skip_attempts=skip_attempts, skipped_throws=n_skipped, n_throws=len(rows),
               n_attempts=int(len(np.unique(att))), regressors=list(regressors), regression_rank=rank,
               n_fit=int(fit.sum()), n_corrected=int(cor.sum()), n_raw_only=int((fit & ~cor).sum()),
               n_boot=n_boot, seed=seed, ratio_ref=ref, blocks={})
    for b in blocks:
        m = lab == b
        mc = lab_c == b
        rec = dict(n=int(m.sum()), n_fit=int((fit & m).sum()), n_corrected=int((cor & m).sum()),
                   attempts=int(len(np.unique(att[m]))), cells=int(len(set(cell[m]))),
                   sd_raw=[float(x) for x in np.sqrt(var_raw[b])], sd_corr=[float(x) for x in np.sqrt(var_c[b])],
                   sd_corr_pooled=pooled(var_c[b]), sd_corr_pooled_ci=[float(np.sqrt(x)) for x in ci(boot[b])],
                   max_abs_resid=float(np.abs(e_c[mc]).max()) if mc.any() else np.nan)
        if b == 'first':
            rec['n_seated'] = int(sum(_seated(r) for r, k in zip(rows, m) if k))
        if ref is not None and b != ref:
            rec['ratio_raw'] = float(np.mean(var_raw[b]) / np.mean(var_raw[ref]))
            rec['ratio_corr'] = float(np.mean(var_c[b]) / np.mean(var_c[ref]))
            rec['ratio_corr_ci'] = ci(boot[b] / boot[ref])
        out['blocks'][b] = rec
    s = out['blocks'].get('S')
    dw = [(b, out['blocks'][b]['ratio_corr'], *out['blocks'][b]['ratio_corr_ci']) for b in _dwell_blocks(blocks) if ref == 'S']
    out['verdict'], out['verdict_clause'] = verdict(s['sd_corr'] if s else None, dw)

    # D27 by where the ball came in at the Catch (entry_r: NOT validated in E0)
    out['d27_by_entry'] = []
    er = np.array([_num(r.get('entry_r')) for r in rows])[cor]
    m = np.where((lab_c == 'D27') & np.isfinite(er))[0]
    if len(m) >= 6:
        m = m[np.argsort(er[m], kind='stable')]
        for part in np.array_split(m, 3):
            dof = float(w_c[part].sum())
            vv = (e_c[part] ** 2).sum(0) / dof if dof >= 2 else np.full(2, np.nan)
            out['d27_by_entry'].append(dict(n=int(len(part)), entry_r_mm=[float(er[part].min()), float(er[part].max())],
                                            sd_corr=[float(x) for x in np.sqrt(vv)], sd_corr_pooled=pooled(vv)))
    return out


def format_blocks(rep):
    """The report as text."""
    f = lambda x: '%.1f' % x if x is not None and np.isfinite(x) else '-'  # noqa: E731
    f2 = lambda x: '%.2f' % x if x is not None and np.isfinite(x) else '-'  # noqa: E731
    L = ['blocks: %d Throws in %d Attempts (gap > %.1f s; %d Throws of the first %d Attempts skipped); launch fit %d, '
         'Platform-corrected %d, raw only %d (a regressor missing)'
         % (rep['n_throws'], rep['n_attempts'], rep['attempt_gap_s'], rep['skipped_throws'], rep['skip_attempts'],
            rep['n_fit'], rep['n_corrected'], rep['n_raw_only']),
         'correction: one pooled fit over the Sitting of v0 on %s (rank %d), both sides centred per (block, Site, release kind); '
         'sd in mm/s about the block mean, degrees of freedom corrected' % (', '.join(rep['regressors']), rep['regression_rank']),
         'ratios: variance (both axes pooled) against block %s; 95 %% interval from %d cluster-bootstrap resamples by Attempt (seed %d)'
         % (rep['ratio_ref'], rep['n_boot'], rep['seed']),
         '%-6s %4s %4s %4s %4s %5s  %-11s %-11s %-18s %-6s %-20s %s' % ('block', 'n', 'fit', 'corr', 'att', 'cells', 'raw sd x/y',
                                                                      'corr sd x/y', 'corr sd pooled', 'raw r', 'corr ratio (95 %)', 'max|e|')]
    for b, r in rep['blocks'].items():
        L.append('%-6s %4d %4d %4d %4d %5d  %-11s %-11s %-18s %-6s %-20s %s' % (
            b, r['n'], r['n_fit'], r['n_corrected'], r['attempts'], r['cells'], '%s/%s' % tuple(f(x) for x in r['sd_raw']),
            '%s/%s' % tuple(f(x) for x in r['sd_corr']), '%s (%s..%s)' % (f(r['sd_corr_pooled']), *map(f, r['sd_corr_pooled_ci'])),
            f2(r.get('ratio_raw')), '%s (%s..%s)' % (f2(r.get('ratio_corr')), *map(f2, r.get('ratio_corr_ci', [None, None])))
            if 'ratio_corr' in r else '-', f(r['max_abs_resid'])) + (' (seated 1 s: %d)' % r['n_seated'] if 'n_seated' in r else ''))
    L.append('VERDICT: %s -- %s' % (rep['verdict'], rep['verdict_clause']))
    if rep['d27_by_entry']:
        L.append('D27 by Catch entry offset, thirds of entry_r (NOT validated in E0; read as a lead, not a measure):')
        for p in rep['d27_by_entry']:
            L.append('  entry_r %s..%s mm  n %d  corr sd %s/%s  pooled %s' % (f(p['entry_r_mm'][0]), f(p['entry_r_mm'][1]), p['n'],
                                                                           f(p['sd_corr'][0]), f(p['sd_corr'][1]), f(p['sd_corr_pooled'])))
    else:
        L.append('D27 by Catch entry offset: fewer than 6 D27 Throws with an entry offset')
    return '\n'.join(L)


def _jsonable(x):
    if isinstance(x, dict):
        return {k: _jsonable(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_jsonable(v) for v in x]
    if isinstance(x, (float, np.floating)):
        return None if not np.isfinite(x) else float(x)
    if isinstance(x, np.integer):
        return int(x)
    return x


def read_records(paths):
    rows = []
    for p in paths:
        with open(p) as fh:
            rows.extend(csv.DictReader(fh))
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('mode', choices=['extract', 'features', 'run', 'blocks'])
    ap.add_argument('src', nargs='+')
    ap.add_argument('--out')
    ap.add_argument('--traces')
    ap.add_argument('--memory', default=MEM_GLOB)
    ap.add_argument('--attempt-gap-s', type=float, default=ATTEMPT_GAP_S)
    ap.add_argument('--skip-attempts', type=int, default=0)
    a = ap.parse_args()
    if a.mode == 'blocks':
        rep = blocks_report(read_records(a.src), a.attempt_gap_s, a.skip_attempts)
        rep['sources'] = [os.path.basename(p) for p in a.src]
        print(format_blocks(rep))
        if a.out:
            import json
            os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
            with open(a.out, 'w') as fo:
                json.dump(_jsonable(rep), fo, indent=1, sort_keys=True)
        return
    if len(a.src) != 1:
        ap.error('%s takes one source' % a.mode)
    a.src = a.src[0]
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
