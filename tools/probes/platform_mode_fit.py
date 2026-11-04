#!/usr/bin/env python3
"""Identify the Platform's lateral ring mode (natural frequency, damping, input
delay) from rosbags already on disk (JB-102, ``plans/active/carry-ring-zvd.md`` P0).

    python tools/probes/platform_mode_fit.py BAG_DIR [BAG_DIR ...]
        [--source encoder|mocap|both] [--axis x|y|both]
        [--model 6.0,0.17,0.015] [--out-dir temp/probes/platform_mode_fit]

Motivated by ``logbook/2026-11-04-sitting-debrief.md`` (one-ball oval long
Throws: the carry decelerates into the release knot, the legs carry no
acceleration feedforward, and the Platform rings at ~6 Hz past the command at
separation).  Its numbers become ``trajectory_op.platform_mode_*`` and underpin
the future ``tests/motion/test_cup_zvd.py``.

The model (the debrief's "Model A" form)
----------------------------------------
Per lateral axis the Platform x is a second-order servo following the delayed
command, with no acceleration feedforward::

    x'' = w^2 (x_c(t - tau) + c - x) + 2 zeta w (v_c(t - tau) - v)

so with ``e = x - x_c(t - tau) - c``:  ``e'' + 2 zeta w e' + w^2 e = -a_c(t - tau)``,
and what is measured is ``r = x - x_c(t) = [x_c(t - tau) - x_c(t)] + c + e``.
``x_c`` is the forward kinematics of ``/leg_cmd_executed`` (100 Hz, cubic spline);
``x`` is either the encoder FK of ``/robot_state`` ("encoder", what the legs see and
can correct) or the mocap ``Platform`` rigid body moved into the robot frame with
``/robot_frame`` and a fitted body-origin lever ("mocap", what the cup sees,
compliance beyond the motors included).

Releases are the ``/throw_announcements`` with thrower ``jugglebot``; the catch
before a release is the latest earlier announcement's predicted landing
(throw_time + tof, 0.2..0.8 s before it) plus ``--catch-lag`` (0.033 s), and the
inferred Dwell is release - catch, binned to 0.05 s.  A release is CARRIED when the commanded
Platform moved > 100 mm along x over that Dwell.

Estimators (both fit PER RELEASE, never on a median profile: the median smears
the ring and the debrief's median fit came out over-damped, 5.1 Hz / 0.76)
------------------------------------------------------------------------------
(a) free decay: the commanded-acceleration-quiet span (|a_c| < 800 mm/s^2 on the
    axis, --a-quiet; the carry peaks at 4000-6000) around the release; the window starts 20 ms after the span starts
    (past any input delay) and runs to the span end or release + 0.30 s, at
    least 60 ms.  The span is anchored 5 ms before the release.  ``r = c + e0*phi1 + v0*phi2`` (the damped free response); the
    window is labelled ``hold`` when the span starts >= 40 ms before the
    release (a pre-release hold, the 2026-10-31 rungs) else ``post``.
(b) forced response: catch .. release + 0.25 s (--forced-post), the full model above; nonlinear
    in (f, zeta, tau), linear in (c, e0, v0) (variable projection, multi-start).
Fits that rail on a bound, or explain < 50 % of the residual variance a
constant leaves, are counted and excluded from the distribution.

(c) prediction check: with the median encoder (f, zeta, tau) and the Model A
    reference (``--model``), simulate every carried release from its commanded
    trajectory (initial state fitted on catch .. release - 50 ms only, and
    zero-state at the catch) and compare the predicted with the measured excess
    velocity at separation, mean of (v - v_c) over release + 20..40 ms, per Dwell.
(d) mocap / encoder excess ratio per Dwell, and the averaged residual spectrum
    of the one-mode forced fit per source (a peak away from the fitted f = a
    second mode).

Output: ``<out-dir>/<YYYYmmdd-HHMMSS>.json`` and ``.txt``.  The bag extraction
(mcap deserialisation is minutes per bag) is cached under
``<out-dir>/cache/<bag>.npz``.  numpy / scipy / mcap_ros2 only; deterministic.
"""
import argparse
import glob
import json
import math
import os
import sys
import time

for _v in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_v, '1')

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
for _p in (ROOT, os.path.join(ROOT, 'ros_ws', 'src', 'jugglebot')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np  # noqa: E402
from scipy.interpolate import CubicSpline  # noqa: E402
from scipy.optimize import least_squares  # noqa: E402
from scipy.signal import cont2discrete, lfilter  # noqa: E402

DT = 1e-3                 # simulation grid (s)
A_QUIET = 800.0           # mm/s^2: commanded-acceleration-quiet threshold (--a-quiet)
HOLD_MIN = 0.040          # s of quiet before release for a "hold" window
FREE_MIN = 0.060          # s: shortest free-decay fit window
SEP = (0.020, 0.040)      # s after release: the separation window
F_BOUNDS = (1.5, 20.0)
Z_BOUNDS = (0.01, 0.95)
TAU_BOUNDS = (0.0, 0.060)
AXES = {'x': 0, 'y': 1}


# --------------------------------------------------------------- extraction
def _stamp(s):
    return s.sec + 1e-9 * s.nanosec


def extract(bag, cache_dir):
    """Bag -> dict of numpy arrays (cached)."""
    name = os.path.basename(os.path.normpath(bag))
    files = sorted(glob.glob(os.path.join(bag, '*.mcap')))
    if not files:
        raise SystemExit('platform_mode_fit: no .mcap in %s' % bag)
    key = sum(os.path.getsize(f) for f in files)
    cpath = os.path.join(cache_dir, name + '.npz')
    if os.path.exists(cpath):
        z = np.load(cpath, allow_pickle=False)
        if int(z['key']) == key:
            return {k: z[k] for k in z.files}
    from mcap_ros2.reader import read_ros2_messages
    cmd, meas, plat, frame, ann = [], [], [], [], []
    topics = ['/leg_cmd_executed', '/robot_state', '/rigid_body_poses',
              '/robot_frame', '/throw_announcements']
    for f in files:
        for m in read_ros2_messages(f, topics=topics):
            r, tp = m.ros_msg, m.channel.topic
            if tp == '/leg_cmd_executed':
                cmd.append([_stamp(r.header.stamp)] + list(r.position[:6]))
            elif tp == '/robot_state':
                meas.append([_stamp(r.timestamp)]
                            + [s.pos_estimate for s in r.motor_states[:6]])
            elif tp == '/rigid_body_poses':
                for b in r.bodies:
                    if b.name == 'Platform':
                        P, q = b.pose.pose.position, b.pose.pose.orientation
                        plat.append((_stamp(b.pose.header.stamp), P.x, P.y, P.z,
                                     q.w, q.x, q.y, q.z))
            elif tp == '/robot_frame':
                P, q = r.pose.position, r.pose.orientation
                frame.append((_stamp(r.header.stamp), P.x, P.y, P.z, q.w, q.x, q.y, q.z))
            else:
                v, p0, l = r.initial_velocity, r.initial_position, r.landing_position
                ann.append((_stamp(r.throw_time), float(r.thrower_name == 'jugglebot'),
                            v.x, v.y, v.z, p0.x, p0.y, p0.z, l.x, l.y, l.z,
                            r.predicted_tof_sec))
    out = dict(key=np.array(key), cmd=np.array(cmd).reshape(-1, 7),
               meas=np.array(meas).reshape(-1, 7), plat=np.array(plat).reshape(-1, 8),
               frame=np.array(frame).reshape(-1, 8), ann=np.array(ann).reshape(-1, 12))
    os.makedirs(cache_dir, exist_ok=True)
    np.savez(cpath, **out)
    return out


def _sorted_unique(a):
    a = a[np.argsort(a[:, 0], kind='stable')]
    keep = np.r_[True, np.diff(a[:, 0]) > 1e-6]
    return a[keep]


def _quat_R(w, x, y, z):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


# --------------------------------------------------------------------- FK
class FK:
    def __init__(self):
        from jugglebot.motion.geometry import StewartGeometry
        from jugglebot.motion.ik_solver import leg_lengths_to_pose
        self.geom = StewartGeometry()
        self._fk = leg_lengths_to_pose

    def __call__(self, rows):
        """rows (N, 7) [t, leg revs x6] -> (t, pos (N,3) mm, R (N,3,3)) of the solved ones."""
        ts, ps, Rs, g = [], [], [], None
        for r in rows:
            try:
                p, R, _ = self._fk(np.asarray(r[1:7]) / self.geom.mm_to_rev, self.geom,
                                   initial_guess=g)
            except Exception:  # noqa: BLE001 - a failed solve just drops the sample
                g = None
                continue
            g = (p, R)
            ts.append(r[0])
            ps.append(p)
            Rs.append(R)
        return np.array(ts), np.array(ps).reshape(-1, 3), np.array(Rs).reshape(-1, 3, 3)


# ----------------------------------------------------------------- releases
def releases(ann):
    """-> list of (t_rel, t_land_prev or nan); jugglebot throws, deduplicated."""
    if not len(ann):
        return []
    ann = ann[np.argsort(ann[:, 0])]
    # dedupe repeats of one throw (re-announcements) within 50 ms
    keep = np.r_[True, np.diff(ann[:, 0]) > 0.05]
    ann = ann[keep]
    land = ann[:, 0] + ann[:, 11]
    out = []
    for i in np.where(ann[:, 1] > 0.5)[0]:
        t = ann[i, 0]
        prev = [land[j] for j in range(len(ann)) if j != i and 0.2 <= t - land[j] <= 0.8]
        out.append((t, max(prev) if prev else float('nan')))
    return out


# -------------------------------------------------------------------- model
def _disc(f, zeta):
    w = 2 * math.pi * f
    numd, dend, _ = cont2discrete(([-1.0], [1.0, 2 * zeta * w, w * w]), DT, method='zoh')
    return np.ravel(numd), np.ravel(dend)


def free_basis(t, f, zeta):
    """Damped free response for e0 = 1 / v0 = 1 (zeta < 1), t from 0."""
    w = 2 * math.pi * f
    s = zeta * w
    wd = w * math.sqrt(max(1 - zeta * zeta, 1e-9))
    ex = np.exp(-s * t)
    return ex * (np.cos(wd * t) + s / wd * np.sin(wd * t)), ex * np.sin(wd * t) / wd


def forced(tgrid, acc_fn, f, zeta, tau):
    """Zero-state e on tgrid driven by -a_c(t - tau)."""
    b, a = _disc(f, zeta)
    return lfilter(b, a, acc_fn(tgrid - tau))


def _linsolve(cols, y):
    A = np.column_stack(cols)
    coef, *_ = np.linalg.lstsq(A, y, rcond=None)
    return coef, y - A @ coef


class Window:
    """One release, one axis, one source: command spline + measured samples."""

    def __init__(self, cs, tm, xm):
        ok = np.isfinite(tm) & np.isfinite(xm)     # mocap drops out as NaN when occluded
        tm, xm = tm[ok], xm[ok]
        self.cs, self.tm, self.xm = cs, tm, xm      # cs: CubicSpline of x_c
        self.r = xm - cs(tm)                        # measured - commanded

    def acc(self, t):
        return self.cs(t, 2)

    def forced_parts(self, t0, t1, f, zeta, tau):
        """-> (sample mask, model-without-linear-terms at samples, phi1, phi2)."""
        msk = (self.tm >= t0) & (self.tm <= t1)
        tg = np.arange(t0, t1 + DT, DT)
        e = forced(tg, self.acc, f, zeta, tau)
        ts = self.tm[msk]
        ef = np.interp(ts, tg, e)
        d = self.cs(ts - tau) - self.cs(ts)
        p1, p2 = free_basis(ts - t0, f, zeta)
        return msk, d + ef, p1, p2

    def fit_forced(self, t0, t1, fixed=None):
        def res(th):
            f, z, tau = th
            msk, m0, p1, p2 = self.forced_parts(t0, t1, f, z, tau)
            _, rr = _linsolve([np.ones_like(p1), p1, p2], self.r[msk] - m0)
            return rr
        if fixed is not None:
            rr = res(fixed)
            return dict(f=fixed[0], zeta=fixed[1], tau=fixed[2], rms=_rms(rr), n=len(rr))
        best = None
        for f0 in (4.5, 7.0):
            for z0 in (0.15, 0.4):
                try:
                    sol = least_squares(res, [f0, z0, 0.015],
                                        bounds=([F_BOUNDS[0], Z_BOUNDS[0], TAU_BOUNDS[0]],
                                                [F_BOUNDS[1], Z_BOUNDS[1], TAU_BOUNDS[1]]),
                                        x_scale=[1.0, 0.1, 0.01])
                except ValueError:
                    continue
                if best is None or sol.cost < best.cost:
                    best = sol
        if best is None:
            return None
        f, z, tau = best.x
        rr = best.fun
        msk = (self.tm >= t0) & (self.tm <= t1)
        y = self.r[msk]
        base = _rms(y - y.mean())
        return dict(f=f, zeta=z, tau=tau, rms=_rms(rr), base=base, n=len(rr),
                    railed=_railed(f, z, tau))

    def fit_free(self, t0, t1):
        msk = (self.tm >= t0) & (self.tm <= t1)
        ts, y = self.tm[msk] - t0, self.r[msk]
        if len(ts) < 6:
            return None

        def res(th):
            p1, p2 = free_basis(ts, th[0], th[1])
            return _linsolve([np.ones_like(p1), p1, p2], y)[1]
        best = None
        for f0 in (4.5, 7.0):
            for z0 in (0.15, 0.4):
                try:
                    sol = least_squares(res, [f0, z0], bounds=([F_BOUNDS[0], Z_BOUNDS[0]],
                                                               [F_BOUNDS[1], Z_BOUNDS[1]]),
                                        x_scale=[1.0, 0.1])
                except ValueError:
                    continue
                if best is None or sol.cost < best.cost:
                    best = sol
        if best is None:
            return None
        f, z = best.x
        return dict(f=f, zeta=z, rms=_rms(best.fun), base=_rms(y - y.mean()), n=len(y),
                    len_s=float(ts[-1]), railed=_railed(f, z, None))

    def predict_excess(self, t_catch, t_rel, f, zeta, tau, ic='data'):
        """Predicted and measured mean (v - v_c) over the separation window."""
        t1 = t_rel + SEP[1] + 0.005
        tg = np.arange(t_catch, t1 + DT, DT)
        e = forced(tg, self.acc, f, zeta, tau)
        d = self.cs(tg - tau) - self.cs(tg)
        p1, p2 = free_basis(tg - t_catch, f, zeta)
        model = d + e
        if ic == 'data':
            msk = (self.tm >= t_catch) & (self.tm <= t_rel - 0.05)
            if msk.sum() < 6:
                return None
            ts = self.tm[msk]
            cols = [np.ones(len(ts)), np.interp(ts, tg, p1), np.interp(ts, tg, p2)]
            coef, _ = _linsolve(cols, self.r[msk] - np.interp(ts, tg, model))
            model = model + coef[0] + coef[1] * p1 + coef[2] * p2
        pred = (np.interp(t_rel + SEP[1], tg, model)
                - np.interp(t_rel + SEP[0], tg, model)) / (SEP[1] - SEP[0])
        return pred

    def meas_excess(self, t_rel):
        a, b = t_rel + SEP[0], t_rel + SEP[1]
        if self.tm[0] > a or self.tm[-1] < b:
            return float('nan')
        return (np.interp(b, self.tm, self.r) - np.interp(a, self.tm, self.r)) / (b - a)


def _rms(v):
    return float(np.sqrt(np.mean(np.square(v)))) if len(v) else float('nan')


def _railed(f, z, tau):
    eps = 1e-3
    r = (f <= F_BOUNDS[0] + eps or f >= F_BOUNDS[1] - eps
         or z <= Z_BOUNDS[0] + eps or z >= Z_BOUNDS[1] - eps)
    if tau is not None:
        r = r or tau >= TAU_BOUNDS[1] - 1e-4
    return bool(r)


def _quiet_span(cs, t_rel, ax_lo, ax_hi):
    """Commanded-accel-quiet span containing release + 10 ms -> (start, end) or None."""
    tg = np.arange(ax_lo, ax_hi, DT)
    q = np.abs(cs(tg, 2)) < A_QUIET
    # anchored just BEFORE the release: the commanded post-release motion starts
    # with a ~-1100 mm/s^2 pulse at +10..+20 ms in every 2026-10-31/11-03 throw
    i = int(np.clip(round((t_rel - 0.005 - ax_lo) / DT), 0, len(tg) - 1))
    if not q[i]:
        return None
    lo = i
    while lo > 0 and q[lo - 1]:
        lo -= 1
    hi = i
    while hi < len(tg) - 1 and q[hi + 1]:
        hi += 1
    return tg[lo], tg[hi]


def _stats(vals):
    v = np.asarray([x for x in vals if x is not None and np.isfinite(x)], float)
    if not len(v):
        return dict(n=0)
    q1, med, q3 = np.percentile(v, [25, 50, 75])
    return dict(n=int(len(v)), median=float(med), iqr=[float(q1), float(q3)],
                mean=float(v.mean()), sd=float(v.std(ddof=1)) if len(v) > 1 else 0.0)


def _spectrum(resids, dt):
    """Averaged Hann periodogram of residual segments resampled on dt -> (freq, power)."""
    n = 512
    acc, cnt = np.zeros(n // 2 + 1), 0
    for ts, rr in resids:
        if len(ts) < 12:
            continue
        tg = np.arange(ts[0], ts[-1], dt)
        y = np.interp(tg, ts, rr)
        y = (y - y.mean()) * np.hanning(len(y))
        acc += np.abs(np.fft.rfft(y, n)) ** 2
        cnt += 1
    if not cnt:
        return None, None
    return np.fft.rfftfreq(n, dt), acc / cnt


def _peaks(fr, pw, lo=2.0, hi=None, k=3):
    if fr is None:
        return []
    hi = hi or fr[-1]
    m = (fr >= lo) & (fr <= hi)
    f, p = fr[m], pw[m]
    idx = [i for i in range(1, len(p) - 1) if p[i] >= p[i - 1] and p[i] >= p[i + 1]]
    idx.sort(key=lambda i: -p[i])
    tot = p.sum()
    return [(float(f[i]), float(p[i] / tot)) for i in idx[:k]]


# --------------------------------------------------------------- per bag
def mocap_in_robot(data, fk_meas):
    """Platform body -> robot frame, plus the fitted body-origin lever (c + R_fk d)."""
    plat = data['plat']
    plat = _sorted_unique(plat[np.all(np.isfinite(plat), axis=1)])
    fr = data['frame']
    if not len(plat) or not len(fr):
        return None, 'no mocap Platform body or /robot_frame'
    o = np.median(fr[:, 1:4], axis=0)
    q = np.median(fr[:, 4:8], axis=0)
    q /= np.linalg.norm(q)
    Rf = _quat_R(*q)
    pr = (plat[:, 1:4] - o) @ Rf          # = Rf^T (p - o) per row
    tf, pf, Rfk = fk_meas
    if len(tf) < 50:
        return None, 'too few encoder FK samples to fit the lever'
    # fit the lever on quasi-static samples only (|v| < 30 mm/s): during the carry
    # the mocap - encoder difference IS the structural compliance being measured
    sp = np.linalg.norm(np.gradient(pf, tf, axis=0), axis=1)
    st = (sp < 30.0) & np.r_[np.diff(tf) < 0.05, False] & np.r_[False, np.diff(tf) < 0.05]
    if st.sum() > 200:
        tf, pf, Rfk = tf[st], pf[st], Rfk[st]
    pm = np.column_stack([np.interp(tf, plat[:, 0], pr[:, k]) for k in range(3)])
    # pm - pf = c + R d  (linear in c, d)
    A = np.zeros((3 * len(tf), 6))
    for k in range(3):
        A[k::3, k] = 1.0
        A[k::3, 3:] = Rfk[:, k, :]
    y = (pm - pf).reshape(-1)
    sol, *_ = np.linalg.lstsq(A, y, rcond=None)
    res = (y - A @ sol).reshape(-1, 3)
    c, d = sol[:3], sol[3:]
    # remove the lever using the encoder orientation interpolated to the mocap stamps
    idx = np.clip(np.searchsorted(tf, plat[:, 0]), 0, len(tf) - 1)
    lever = np.einsum('nij,j->ni', Rfk[idx], d)
    pos = pr - c - lever
    info = dict(lever_mm=[float(v) for v in d], offset_mm=[float(v) for v in c],
                fit_rms_mm=[float(v) for v in np.sqrt(np.mean(res ** 2, axis=0))])
    return (plat[:, 0], pos), info


def analyse_bag(bag, args, fk, model_a):
    t_start = time.time()
    data = extract(bag, args.cache)
    cmd = _sorted_unique(data['cmd'])
    meas = _sorted_unique(data['meas'])
    rels = releases(data['ann'])
    out = dict(bag=bag, n_announced_releases=len(rels), windows=[], notes=[])
    if not rels or len(cmd) < 100:
        out['notes'].append('no jugglebot releases or no /leg_cmd_executed: nothing to fit')
        return out, []
    sources = ['encoder', 'mocap'] if args.source == 'both' else [args.source]
    axes = ['x', 'y'] if args.axis == 'both' else [args.axis]
    # FK of every sample inside the analysed spans
    spans = [(t - 0.85, t + 0.40) for t, _ in rels]

    def fk_spans(arr):
        sel = np.zeros(len(arr), bool)
        for a, b in spans:
            sel |= (arr[:, 0] >= a) & (arr[:, 0] <= b)
        return fk(arr[sel])
    fkc = fk_spans(cmd)
    fkm = fk_spans(meas)
    moc = None
    if 'mocap' in sources:
        moc, info = mocap_in_robot(data, fkm)
        out['mocap_frame'] = info
        if moc is None:
            out['notes'].append('mocap skipped: %s' % info)
    recs = []
    for t_abs, t_land in rels:
        # every per-release time is relative to the release: an absolute epoch
        # (~1.8e9 s) leaves the delay's finite-difference step below float resolution
        c_m = (fkc[0] >= t_abs - 0.85) & (fkc[0] <= t_abs + 0.40)
        tc = fkc[0][c_m] - t_abs
        if len(tc) < 60:
            continue
        pc = fkc[1][c_m]
        t_rel = 0.0
        t_catch = (t_land - t_abs + args.catch_lag) if np.isfinite(t_land) else -0.50
        dwell = t_rel - t_catch
        carry_mm = float(np.interp(t_rel, tc, pc[:, 0]) - np.interp(t_catch, tc, pc[:, 0]))
        rec = dict(t_rel=float(t_abs), dwell=round(round(dwell / 0.05) * 0.05, 2) if np.isfinite(t_land)
                   else None, dwell_raw=float(dwell), carry_mm=carry_mm,
                   carried=bool(abs(carry_mm) > 100.0), fits={})
        for ax in axes:
            k = AXES[ax]
            cs = CubicSpline(tc, pc[:, k])
            span = _quiet_span(cs, t_rel, tc[0], tc[-1])
            arms = float(np.sqrt(np.mean(cs(np.arange(t_catch, t_rel, DT), 2) ** 2)))
            rec['fits'].setdefault(ax, {})['acc_rms_carry'] = arms
            if span is not None:
                rec['fits'][ax]['quiet'] = [float(span[0] - t_rel), float(span[1] - t_rel)]
            for src in sources:
                if src == 'encoder':
                    m = (fkm[0] >= t_abs - 0.85) & (fkm[0] <= t_abs + 0.40)
                    tm, xm = fkm[0][m] - t_abs, fkm[1][m][:, k]
                elif moc is not None:
                    m = (moc[0] >= t_abs - 0.85) & (moc[0] <= t_abs + 0.40)
                    tm, xm = moc[0][m] - t_abs, moc[1][m][:, k]
                else:
                    continue
                if len(tm) < 30:
                    continue
                w = Window(cs, tm, xm)
                fr = dict(meas_excess=float(w.meas_excess(t_rel)))
                if span is not None:
                    a0 = span[0] + 0.020
                    a1 = min(span[1], t_rel + 0.30)
                    if a1 - a0 >= FREE_MIN:
                        fd = w.fit_free(a0, a1)
                        if fd:
                            fd['kind'] = 'hold' if span[0] <= t_rel - HOLD_MIN else 'post'
                            fr['free'] = fd
                if rec['carried']:
                    t0, t1 = t_catch, t_rel + args.forced_post
                    ff = w.fit_forced(t0, t1)
                    if ff:
                        msk = (w.tm >= t0) & (w.tm <= t1)
                        fr['forced'] = ff
                        # residual for the spectrum
                        _, m0, p1, p2 = w.forced_parts(t0, t1, ff['f'], ff['zeta'], ff['tau'])
                        _, rr = _linsolve([np.ones_like(p1), p1, p2], w.r[msk] - m0)
                        fr['_resid'] = (w.tm[msk], rr)
                    fr['_win'] = w
                rec['fits'][ax][src] = fr
        recs.append(rec)
    out['elapsed_s'] = round(time.time() - t_start, 1)
    return out, recs


def _good(fit, min_expl=0.5):
    return (fit is not None and not fit['railed']
            and fit['rms'] < (1 - min_expl) ** 0.5 * fit['base'])


def summarise(name, recs, sources, axes):
    """Per (axis, source): distributions of (a) and (b)."""
    s = {}
    for ax in axes:
        for src in sources:
            free = [r['fits'][ax][src]['free'] for r in recs
                    if src in r['fits'].get(ax, {}) and 'free' in r['fits'][ax][src]]
            forc = [r['fits'][ax][src]['forced'] for r in recs
                    if src in r['fits'].get(ax, {}) and 'forced' in r['fits'][ax][src]]
            row = {}
            for kind in ('hold', 'post'):
                fs = [f for f in free if f['kind'] == kind]
                g = [f for f in fs if _good(f)]
                row['free_' + kind] = dict(
                    n_windows=len(fs), n_good=len(g),
                    f=_stats([f['f'] for f in g]), zeta=_stats([f['zeta'] for f in g]),
                    rms_mm=_stats([f['rms'] for f in g]),
                    len_s=_stats([f['len_s'] for f in fs]))
            g = [f for f in forc if _good(f)]
            row['forced'] = dict(
                n_carries=len(forc), n_good=len(g),
                n_railed=sum(1 for f in forc if f['railed']),
                f=_stats([f['f'] for f in g]), zeta=_stats([f['zeta'] for f in g]),
                tau_ms=_stats([1e3 * f['tau'] for f in g]),
                rms_mm=_stats([f['rms'] for f in g]),
                base_rms_mm=_stats([f['base'] for f in g]))
            acc = [r['fits'][ax]['acc_rms_carry'] for r in recs
                   if r['carried'] and ax in r['fits']]
            row['acc_rms_carry_mm_s2'] = _stats(acc)
            s['%s/%s' % (ax, src)] = row
    return s


def main(argv=None):
    global A_QUIET
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('bags', nargs='+')
    ap.add_argument('--source', choices=('encoder', 'mocap', 'both'), default='both')
    ap.add_argument('--axis', choices=('x', 'y', 'both'), default='both')
    ap.add_argument('--catch-lag', type=float, default=0.033,
                    help='s added to the previous throw\'s predicted landing to get the catch '
                         'knot; 0.033 maps the 2026-11-03 landing->release clusters 0.433 / '
                         '0.380 s onto the logged Dwells 0.40 / 0.35')
    ap.add_argument('--forced-post', type=float, default=0.25,
                    help='s after release that the forced fit (b) runs to; at hold 0 the decel '
                         'ends AT the release, so a fit ending at +0.04 s sees almost none of '
                         'the free ring and its damping rails (measured 2026-11-04)')
    ap.add_argument('--a-quiet', type=float, default=A_QUIET,
                    help='mm/s^2: |commanded acceleration| below which a span counts as free decay')
    ap.add_argument('--model', default='6.0,0.17,0.015',
                    help='reference f_Hz,zeta,tau_s for the prediction check (Model A)')
    ap.add_argument('--out-dir', default=os.path.join(ROOT, 'temp', 'probes',
                                                      'platform_mode_fit'))
    args = ap.parse_args(argv)
    args.cache = os.path.join(args.out_dir, 'cache')
    A_QUIET = args.a_quiet
    os.makedirs(args.out_dir, exist_ok=True)
    model_a = tuple(float(v) for v in args.model.split(','))
    sources = ['encoder', 'mocap'] if args.source == 'both' else [args.source]
    axes = ['x', 'y'] if args.axis == 'both' else [args.axis]
    fk = FK()
    report = dict(generated=time.strftime('%Y-%m-%d %H:%M:%S'), argv=sys.argv[1:],
                  model_a=model_a, a_quiet_mm_s2=A_QUIET, catch_lag_s=args.catch_lag, forced_post_s=args.forced_post,
                  sep_window_s=SEP, bags=[])
    allrecs = []
    lines = []
    P = lines.append
    for bag in args.bags:
        info, recs = analyse_bag(bag, args, fk, model_a)
        info['summary'] = summarise(bag, recs, sources, axes)
        info['n_carried'] = sum(r['carried'] for r in recs)
        dw = {}
        for r in recs:
            if r['carried']:
                dw[r['dwell']] = dw.get(r['dwell'], 0) + 1
        info['carried_by_dwell'] = {str(k): v for k, v in sorted(dw.items(), key=str)}
        info['releases'] = [
            dict(t_rel=r['t_rel'], dwell=r['dwell'], dwell_raw=round(r['dwell_raw'], 3),
                 carry_mm=round(r['carry_mm'], 1), carried=r['carried'],
                 fits={ax: {k: ({kk: vv for kk, vv in v.items() if not kk.startswith('_')}
                                if isinstance(v, dict) else v)
                            for k, v in d.items()} for ax, d in r['fits'].items()})
            for r in recs]
        report['bags'].append(info)
        allrecs += [(bag, r) for r in recs]
        P('== %s  releases %d, carried %d %s  (%.0f s)' % (
            os.path.basename(os.path.normpath(bag)), len(recs), info['n_carried'],
            info['carried_by_dwell'], info.get('elapsed_s', 0)))
        if 'mocap_frame' in info and isinstance(info['mocap_frame'], dict):
            mf = info['mocap_frame']
            P('   mocap lever %s mm, fit rms %s mm' % (
                [round(v, 1) for v in mf['lever_mm']], [round(v, 2) for v in mf['fit_rms_mm']]))
        for n in info['notes']:
            P('   note: ' + n)
        for key, row in info['summary'].items():
            P('   %-11s carry |a_c| rms %s mm/s^2' % (
                key, _fmt(row['acc_rms_carry_mm_s2'], 0)))
            for kind in ('hold', 'post'):
                fr = row['free_' + kind]
                if fr['n_windows']:
                    P('     (a) free %-4s windows %3d good %3d  len %s s  f %s Hz  zeta %s  rms %s mm'
                      % (kind, fr['n_windows'], fr['n_good'], _fmt(fr['len_s'], 3),
                         _fmt(fr['f'], 2), _fmt(fr['zeta'], 3), _fmt(fr['rms_mm'], 2)))
            fo = row['forced']
            if fo['n_carries']:
                P('     (b) forced   carries %3d good %3d railed %3d  f %s Hz  zeta %s  tau %s ms'
                  '  rms %s mm (const-only %s)'
                  % (fo['n_carries'], fo['n_good'], fo['n_railed'], _fmt(fo['f'], 2),
                     _fmt(fo['zeta'], 3), _fmt(fo['tau_ms'], 1), _fmt(fo['rms_mm'], 2),
                     _fmt(fo['base_rms_mm'], 2)))

    # ---- (c) prediction check + (d) excess ratio / spectrum, pooled over bags
    pooled = {}
    for ax in axes:
        for src in sources:
            g = [r['fits'][ax][src]['forced'] for _, r in allrecs
                 if src in r['fits'].get(ax, {}) and _good(r['fits'][ax][src].get('forced'))]
            if g:
                pooled['%s/%s' % (ax, src)] = (float(np.median([f['f'] for f in g])),
                                               float(np.median([f['zeta'] for f in g])),
                                               float(np.median([f['tau'] for f in g])))
    report['pooled_forced_median'] = pooled
    P('')
    P('== pooled forced-fit medians (f Hz, zeta, tau s): %s' % {
        k: tuple(round(x, 3) for x in v) for k, v in pooled.items()})
    pred = {}
    for ax in axes:
        models = {'model_a': model_a}
        if '%s/encoder' % ax in pooled:
            models['encoder_fit'] = pooled['%s/encoder' % ax]
        for src in sources:
            groups = {}
            for bag, r in allrecs:
                fr = r['fits'].get(ax, {}).get(src)
                if not r['carried'] or not fr or '_win' not in fr:
                    continue
                w = fr['_win']
                t_catch = -r['dwell_raw']
                row = dict(meas=fr['meas_excess'])
                own = fr.get('forced')
                if _good(own):
                    row['own_fit_ic_data'] = w.predict_excess(t_catch, 0.0, own['f'],
                                                              own['zeta'], own['tau'], 'data')
                for mn, (f, z, tau) in models.items():
                    row[mn + '_ic_data'] = w.predict_excess(t_catch, 0.0, f, z, tau, 'data')
                    row[mn + '_ic_zero'] = w.predict_excess(t_catch, 0.0, f, z, tau, 'zero')
                gk = '%s|%s' % (os.path.basename(os.path.normpath(bag)), r['dwell'])
                groups.setdefault(gk, []).append(row)
            for gk, rows in sorted(groups.items()):
                key = '%s/%s %s' % (ax, src, gk)
                ks = sorted({k for q in rows for k in q})
                pred[key] = {k: _stats([q.get(k) for q in rows]) for k in ks}
    report['prediction'] = pred
    P('')
    P('== (c) excess velocity at separation (mean of v - v_c over +20..+40 ms), mm/s, mean +/- sd (n)')
    for key, d in pred.items():
        P('   %-44s meas %s' % (key, _fmt_m(d['meas'])))
        for k in sorted(d):
            if k != 'meas':
                P('       %-26s %s   pred - meas %+.1f' % (
                    k, _fmt_m(d[k]), d[k].get('mean', float('nan')) - d['meas'].get('mean', float('nan'))))
    # (d) ratio
    ratio = {}
    for ax in axes:
        for gk in sorted({k.split(' ', 1)[1] for k in pred if k.startswith(ax + '/')}):
            e = pred.get('%s/encoder %s' % (ax, gk), {}).get('meas', {})
            m = pred.get('%s/mocap %s' % (ax, gk), {}).get('meas', {})
            if e.get('n') and m.get('n') and abs(e['mean']) > 1e-6:
                ratio['%s %s' % (ax, gk)] = dict(encoder=e['mean'], mocap=m['mean'],
                                                 ratio=m['mean'] / e['mean'])
    report['mocap_encoder_excess_ratio'] = ratio
    P('')
    P('== (d) mocap / encoder excess ratio')
    for k, v in ratio.items():
        P('   %-30s encoder %+6.1f  mocap %+6.1f  ratio %.2f' % (k, v['encoder'], v['mocap'], v['ratio']))
    spec = {}
    for ax in axes:
        for src in sources:
            res = [r['fits'][ax][src]['_resid'] for _, r in allrecs
                   if '_resid' in r['fits'].get(ax, {}).get(src, {})
                   and _good(r['fits'][ax][src]['forced'])]
            dt = 0.010 if src == 'encoder' else 1.0 / 300.0
            fr, pw = _spectrum(res, dt)
            raw = [(w.tm[(w.tm >= -r['dwell_raw']) & (w.tm <= 0.04)],
                    w.r[(w.tm >= -r['dwell_raw']) & (w.tm <= 0.04)])
                   for _, r in allrecs
                   for w in [r['fits'].get(ax, {}).get(src, {}).get('_win')] if w is not None]
            fr2, pw2 = _spectrum(raw, dt)
            spec['%s/%s' % (ax, src)] = dict(
                n=len(res), resid_peaks=_peaks(fr, pw), error_peaks=_peaks(fr2, pw2),
                resid_rms_over_error_rms=(float(np.sqrt(pw.sum() / pw2.sum()))
                                          if fr is not None and fr2 is not None else None))
    report['spectrum'] = spec
    P('')
    P('== (d) spectra over the carry windows: [(Hz, share of 2 Hz+ power)] of the error r and of the one-mode residual')
    for k, v in spec.items():
        P('   %-11s n %3d  error %s  residual %s  resid/error rms %s' % (
            k, v['n'], [(round(a, 1), round(b, 2)) for a, b in v['error_peaks']],
            [(round(a, 1), round(b, 2)) for a, b in v['resid_peaks']],
            None if v['resid_rms_over_error_rms'] is None else round(v['resid_rms_over_error_rms'], 2)))
    stamp = time.strftime('%Y%m%d-%H%M%S')
    jp = os.path.join(args.out_dir, stamp + '.json')
    tp = os.path.join(args.out_dir, stamp + '.txt')
    with open(jp, 'w') as fh:
        json.dump(report, fh, indent=1, default=float)
    with open(tp, 'w') as fh:
        fh.write('\n'.join(lines) + '\n')
    print('\n'.join(lines))
    print('\nwrote %s\n      %s' % (jp, tp))
    return 0


def _fmt(st, nd):
    if not st or not st.get('n'):
        return '-'
    return ('%.*f [%.*f, %.*f]' % (nd, st['median'], nd, st['iqr'][0], nd, st['iqr'][1]))


def _fmt_m(st):
    if not st or not st.get('n'):
        return '-'
    return '%+6.1f +/- %5.1f (%d)' % (st['mean'], st['sd'], st['n'])


if __name__ == '__main__':
    sys.exit(main())
