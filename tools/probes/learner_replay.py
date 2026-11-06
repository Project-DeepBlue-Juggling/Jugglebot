#!/usr/bin/env python3
"""Replay the Learner memory corpus through the OLD law and a prototype of the plan's NEW law.

Question (``plans/active/learner-generalisation.md`` Phase 1, "Probe FIRST"): does the global plant
model f0 (apex gain K, lateral offset b, axis-0 centre pull s, per-ball offset Delta) fitted by
recursive least squares, plus the residual memory L2 and the evidence cap L3, predict the robot's
throws better than today's local-linear Learner, at a cold Site and in steady state, and with
which constants?

Everything is evaluated as a PREDICTOR (the corpus rows were flown under the old law's commands,
so a new law's landing cannot be observed): for row i, each law's forward model built only from
the rows available before it predicts the landing of the row's ACTUAL command u_i, scored against
the observed y_i.  Availability rule (the 2-throw delay): row i sees rows 0..i-2, plus row i-1
when i-1 belongs to an earlier Sitting (a Sitting = a run of rows with no gap > 1800 s).

Old law = ``learner_replay_old_law.py`` (the pre-P1 ``motion/skills/learner.py``, frozen with its bodies
unchanged on 2026-11-06 when that module was rewritten) CALLED, never re-implemented: ``learner.predict`` (gate
centred on the command, ``y_d=None``) for the predictor; ``learner.command`` with the Goal's
y_d recovered from the launch logs (lateral y_d = 0) for the command-replay fidelity check.

New law (prototype, plan sections 2.2-2.4):
  f0 forward  y_apex = K u_apex;  y_xy = u_xy + b + (s x_site, 0) + Delta_ball  (Delta_0 = 0)
  RLS with forgetting lambda, three independent filters (apex: K; axis 0: b_x, s, Delta_b,x;
  axis 1: b_y, Delta_b,y), Kalman form with measurement variance R, prior = identity
  (K = 1, rest 0) with a diagonal prior covariance P0; a parameter whose posterior sd exceeds
  ``FALLBACK_FRAC`` x its prior sd REPORTS its prior (empty memory -> the identity command).
  Forgetting is either plain exponential (P <- P/lambda) or BOUNDED (the same, then each
  diagonal capped at P0: no covariance wind-up in unexcited directions).
  L2: over the k most recent available rows with the same ball_id and release_kind inside the
  support d2 <= 9, w = exp(-d2), d2 = |dp|^2/h_p^2 + |dv|^2/h_v^2 + ddwell^2/h_d^2; the residual
  of each row is taken against the CURRENT f0; r_hat = weighted mean; alpha = sum w/(sum w + w0);
  correction = alpha r_hat, capped per axis at c_max (1 - exp(-sum w / w0)) (L3; apex cap in
  command space, i.e. K c_max_apex in landing space).

Derived state the corpus does not carry (stated, not guessed): release xy = site xy + seat
offset (x0 + x2, x1 + x3); release z = ``sites.RELEASE_CUP_Z_MM`` for every row (constant, so it
drops out of d2); planned release velocity from ``ballistics_bc.launch_velocity`` with flight time
``schedule.flight_s(u_apex)``, target z = ``sites.CATCH_CUP_Z_MM``, target xy = release xy for a
vertical throw and release xy + (separation, 0) for an oval carried throw (separation from the
Goal line, default 0.150 m); the query uses the row's actual command (a live command would use
f0^-1(y_d)); dwell from the Goal line when the launch log still exists, else UNKNOWN (an unknown
dwell contributes 0 to d2 and is counted); release_kind = 'carried' for oval rows (Site x0 =
-0.075 and/or the Goal line names an oval pattern), else 'free_cup'.

Usage (repo root; offline, read-only; single process, ~1-3 min with the sweep):
    OPENBLAS_NUM_THREADS=1 python tools/probes/learner_replay.py \
        --csv temp/probes/learner_replay/corpus_3276.csv --tag 3276
    ... --csv temp/probes/learner_replay/corpus_846.csv --tag 846
Outputs under temp/probes/learner_replay/: goals.csv, rows_<tag>.csv (per-row predictions),
theta_<tag>.csv (RLS trajectory), sweep_<tag>.csv, coldstart_<tag>.csv, batch_<tag>.csv,
report_<tag>.md, and report.md (all report_*.md concatenated).
"""
from __future__ import annotations

import argparse
import glob
import math
import os
import re
import sys
import time

import numpy as np
import pandas as pd

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(REPO, 'ros_ws', 'src', 'jugglebot'))
sys.path.insert(0, os.path.join(REPO, 'tools', 'probes', 'memory_regression'))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))   # the frozen old law beside this file

import learner_replay_old_law as lr                        # noqa: E402  (the OLD law, frozen 2026-11-06, called)
from jugglebot.motion.skills import schedule as sch        # noqa: E402
from jugglebot.motion.skills import sites as st            # noqa: E402
from jugglebot.motion.trajectory import ballistics_bc as bbc  # noqa: E402
import models as mr                                        # noqa: E402  (load / inlier rule of 2026-10-18)

OUT = os.path.join(REPO, 'temp', 'probes', 'learner_replay')
LOG_ROOT = os.path.expanduser('~/.ros/log')
OVAL_SITE_X = -0.075
OVAL_SEP_DEFAULT_M = 0.150
CLAMP_OLD_M = 0.0199                      # rows with |u_xy| >= this sat at the old 20 mm clamp
SUPPORT_D2 = 9.0
FALLBACK_FRAC = 0.5                       # sd > 0.5 x prior sd -> the parameter reports its prior
# prior (identity) and prior sd
PRIOR_SD = dict(K=0.10, b=0.030, s=0.30, D=0.030)
R_APEX = 0.010 ** 2                       # measurement variance (m^2); apex residual ~10 mm
R_LAT = 0.020 ** 2                        # lateral shot noise ~20 mm per axis
MBAND_APEX = 0.09                         # 'mband': |y2 / (K u2) - 1| < 9 % (1.0..1.2 about K = 1.085, centred)
LAMBDAS = (0.95, 0.97, 0.98, 0.99, 0.995, 0.998, 1.0)   # 0.95 / 0.97 extend the brief's grid
ERA_EDGES = (('2026-10-17', 'E1 <=10-16 (the 846 rows)'),)


# ----------------------------------------------------------------------------- launch logs
_GOAL_RE = re.compile(r"goal dispatched \(pattern='([^']+)'(.*)\)")
_START_RE = re.compile(r"\[skill_node\]: ([a-z_0-9]+) started: (.*)")


def _kv(text, key):
    m = re.search(key + r'=([0-9.]+)', text)
    return float(m.group(1)) if m else np.nan


def parse_goals(t_lo, t_hi):
    """(t, src, pattern, apex_m, dwell_s, sep_m, logdir) from every launch.log in the window."""
    rows = []
    dirs = sorted(glob.glob(os.path.join(LOG_ROOT, '2026-*-jetson-*')))
    for d in dirs:
        f = os.path.join(d, 'launch.log')
        if not os.path.isfile(f):
            continue
        try:
            mt = os.path.getmtime(f)
        except OSError:
            continue
        name = os.path.basename(d)
        try:   # the directory name is the launch's local start time
            t0 = pd.Timestamp(pd.to_datetime(name[:19], format='%Y-%m-%d-%H-%M-%S')).tz_localize(
                'Australia/Sydney').timestamp()
        except Exception:
            t0 = mt
        if mt < t_lo - 3600 or t0 > t_hi + 3600:
            continue
        with open(f, 'r', errors='replace') as fh:
            for line in fh:
                if 'goal dispatched' not in line and ' started: ' not in line:
                    continue
                try:
                    t = float(line.split(' ', 1)[0])
                except ValueError:
                    continue
                m = _GOAL_RE.search(line)
                if m:
                    rest = m.group(2)
                    rows.append((t, 'goal', m.group(1), _kv(rest, 'apex_m'), _kv(rest, 'dwell_s'),
                                 _kv(rest, 'separation_mm') / 1000.0, name))
                    continue
                m = _START_RE.search(line)
                if m:
                    rest = m.group(2)
                    a = re.search(r'at apex ([0-9.]+) m', rest)
                    dw = re.search(r'dwell ([0-9.]+) s', rest)
                    sp = re.search(r'sep ([0-9]+) mm', rest)
                    rows.append((t, 'started', m.group(1), float(a.group(1)) if a else np.nan,
                                 float(dw.group(1)) if dw else np.nan,
                                 float(sp.group(1)) / 1000.0 if sp else np.nan, name))
    g = pd.DataFrame(rows, columns=['t', 'src', 'pattern', 'apex_m', 'dwell_s', 'sep_m', 'logdir'])
    return g.sort_values('t').reset_index(drop=True)


def attach_goals(d, g, max_age_s=1200.0):
    """Per row: the last Goal (dispatched or started) before its release, its fields filled from a
    'started' line of the same Goal when the dispatch line lacks them."""
    n = len(d)
    pat = np.array([''] * n, dtype=object)
    apex = np.full(n, np.nan)
    dwell = np.full(n, np.nan)
    sep = np.full(n, np.nan)
    logdir = np.array([''] * n, dtype=object)
    gid = np.full(n, np.nan)
    if len(g):
        gt = g.t.values
        for i, t in enumerate(d.t_abs_s.values):
            j = np.searchsorted(gt, t, side='right') - 1
            # last 'goal' line, else last 'started' line
            jg = j
            while jg >= 0 and g.src.values[jg] != 'goal' and t - gt[jg] < max_age_s:
                jg -= 1
            if jg < 0 or t - gt[jg] >= max_age_s:
                jg = j
            if jg < 0 or t - gt[jg] >= max_age_s:
                continue
            pat[i] = g.pattern.values[jg]
            apex[i], dwell[i], sep[i] = g.apex_m.values[jg], g.dwell_s.values[jg], g.sep_m.values[jg]
            logdir[i] = g.logdir.values[jg]
            gid[i] = gt[jg]
            for k in range(jg + 1, j + 1):            # a 'started' line of the same Goal fills gaps
                if g.src.values[k] == 'started':
                    if np.isnan(apex[i]):
                        apex[i] = g.apex_m.values[k]
                    if np.isnan(dwell[i]):
                        dwell[i] = g.dwell_s.values[k]
                    if np.isnan(sep[i]):
                        sep[i] = g.sep_m.values[k]
    d['pattern'] = pat
    d['goal_apex'] = apex
    d['dwell'] = dwell
    d['sep'] = sep
    d['logdir'] = logdir
    d['goal_t'] = gid
    return d


# ----------------------------------------------------------------------------- derived state
def derive(d):
    # a Site is (x0, x1): the turned patterns put Sites off the x axis (x1 = +-0.0625, +-0.0442)
    d['site'] = ['%+.4f,%+.4f' % (a + 0.0, b + 0.0) for a, b in zip(d.x0.round(4), d.x1.round(4))]
    d['rel_x'] = d.x0 + d.x2
    d['rel_y'] = d.x1 + d.x3
    oval_pat = d.pattern.astype(str).str.startswith('oval')
    d['oval'] = (np.isclose(d.x0, OVAL_SITE_X)) | oval_pat
    d['kind'] = np.where(d.oval, 'carried', 'free_cup')
    sep = np.where(np.isfinite(d.sep) & d.oval, d.sep, OVAL_SEP_DEFAULT_M)
    vx = np.zeros(len(d))
    vz = np.zeros(len(d))
    for i in range(len(d)):
        a = float(d.u2_apex_m.values[i])
        T = sch.flight_s(a)
        p0 = np.array([d.rel_x.values[i], d.rel_y.values[i], st.RELEASE_CUP_Z_MM / 1000.0]) * 1000.0
        p1 = p0.copy()
        p1[2] = st.CATCH_CUP_Z_MM
        if d.oval.values[i]:
            p1[0] += sep[i] * 1000.0
        v = bbc.launch_velocity(p0, p1, T) / 1000.0
        vx[i], vz[i] = v[0], v[2]
    d['vx'] = vx
    d['vz'] = vz
    d['ball1'] = (d.ball_id == 1).astype(float)
    # availability (2-throw delay inside a Sitting)
    sit = d.sit.values
    avail = np.zeros(len(d), dtype=int)
    for i in range(1, len(d)):
        avail[i] = i if sit[i - 1] != sit[i] else i - 1
    d['avail'] = avail
    # throw index at a Site within a Sitting (0-based) and first appearances
    d['k_site_sit'] = d.groupby(['sit', 'site']).cumcount()
    d['first_site'] = ~d.site.duplicated()
    d['first_site_ball'] = ~d[['site', 'ball_id']].duplicated()
    d['inlier'] = mr.inlier_mask(d).values
    d['at_clamp_old'] = (d.u0.abs() >= CLAMP_OLD_M) | (d.u1.abs() >= CLAMP_OLD_M)
    d['at_clamp_old0'] = d.u0.abs() >= CLAMP_OLD_M
    d['at_clamp_old1'] = d.u1.abs() >= CLAMP_OLD_M
    return d


def eras(d):
    t_local = pd.to_datetime(d.t_abs_s, unit='s').dt.tz_localize('UTC').dt.tz_convert('Australia/Sydney')
    first_oval = d.index[d.oval].min() if d.oval.any() else len(d)
    e = np.array(['E1 09-28..10-16 (846-row corpus)'] * len(d), dtype=object)
    idx = np.arange(len(d))
    e[(idx >= 846)] = 'E2 10-17..oval'
    e[(idx >= first_oval)] = 'E3 oval era ..11-01'
    e[(t_local >= pd.Timestamp('2026-11-02', tz='Australia/Sydney')).values & (idx >= 846)] = 'E4 11-02..11-03'
    return e


# ----------------------------------------------------------------------------- batch f0 fit
def batch_fit(df):
    """OLS of f0 on rows ``df``: K through the origin; e0 = b_x + s x0 + D_x ball1; e1 = b_y + D_y ball1."""
    out = {}
    u2, y2 = df.u2_apex_m.values, df.y2_apex_m.values
    out['n'] = len(df)
    out['K'] = float(u2 @ y2 / (u2 @ u2)) if len(df) else np.nan
    out['apex_resid_mm'] = 1000 * float(np.sqrt(np.mean((y2 - out['K'] * u2) ** 2))) if len(df) else np.nan
    has_b1 = df.ball1.sum() > 0 and df.ball1.sum() < len(df)
    n_sites = df.site.nunique()
    cols0 = [np.ones(len(df))] + ([df.x0.values] if n_sites > 1 else []) + ([df.ball1.values] if has_b1 else [])
    X0 = np.column_stack(cols0)
    beta0 = np.linalg.lstsq(X0, df.e0.values, rcond=None)[0] if len(df) else [np.nan] * 3
    X1 = np.column_stack([np.ones(len(df))] + ([df.ball1.values] if has_b1 else []))
    beta1 = np.linalg.lstsq(X1, df.e1.values, rcond=None)[0] if len(df) else [np.nan] * 2
    out['b_x_mm'] = 1000 * beta0[0]
    out['s'] = beta0[1] if n_sites > 1 else np.nan
    out['D1x_mm'] = 1000 * beta0[-1] if has_b1 else np.nan
    out['b_y_mm'] = 1000 * beta1[0]
    out['D1y_mm'] = 1000 * beta1[1] if has_b1 else np.nan
    out['b_pool_x_mm'] = 1000 * float(df.e0.mean()) if len(df) else np.nan   # 2026-10-18 'M1 pooled const'
    out['b_pool_y_mm'] = 1000 * float(df.e1.mean()) if len(df) else np.nan
    b0 = df[df.ball1 == 0]
    if b0.site.nunique() > 1:
        out['s_ball0'] = float(np.polyfit(b0.x0.values, b0.e0.values, 1)[0])
    out['rank0'] = int(np.linalg.matrix_rank(X0))
    out['cols0'] = X0.shape[1]
    return out


def loso_coldstart(D, first_n=5):
    """Leave-one-Site-out: f0 batch-fitted on every OTHER Site's inliers, scored on the held-out
    Site's first ``first_n`` throws (all rows, file order) and on all its inlier rows."""
    rows = []
    for site in sorted(D.site.unique()):
        tr = D[(D.site != site) & D.inlier]
        te = D[D.site == site]
        f = batch_fit(tr)
        K = f['K']
        s = 0.0 if np.isnan(f['s']) else f['s']
        dx = 0.0 if np.isnan(f['D1x_mm']) else f['D1x_mm'] / 1000
        dy = 0.0 if np.isnan(f['D1y_mm']) else f['D1y_mm'] / 1000
        for label, sub in (('first%d' % first_n, te.head(first_n)), ('all_inliers', te[te.inlier])):
            p0 = sub.u0 + f['b_x_mm'] / 1000 + s * sub.x0 + dx * sub.ball1
            p1 = sub.u1 + f['b_y_mm'] / 1000 + dy * sub.ball1
            rows.append(dict(site=site, set=label, n=len(sub), K_train=K,
                             apex_f0_mm=_rms(sub.y2_apex_m - K * sub.u2_apex_m),
                             apex_identity_mm=_rms(sub.y2_apex_m - sub.u2_apex_m),
                             lat0_f0_mm=_rms(sub.y0 - p0), lat1_f0_mm=_rms(sub.y1 - p1),
                             lat0_identity_mm=_rms(sub.y0 - sub.u0), lat1_identity_mm=_rms(sub.y1 - sub.u1)))
    return pd.DataFrame(rows)


def _rms(x):
    x = np.asarray(x, dtype=float)
    x = x[np.isfinite(x)]
    return 1000 * float(np.sqrt(np.mean(x ** 2))) if len(x) else np.nan


def _med(x):
    x = np.asarray(x, dtype=float)
    x = x[np.isfinite(x)]
    return 1000 * float(np.median(np.abs(x))) if len(x) else np.nan


# ----------------------------------------------------------------------------- RLS f0
class Filt(object):
    """One RLS filter (Kalman form, forgetting lambda), identity prior, fallback to prior."""

    def __init__(self, prior, sd0, R, lam, bounded):
        self.th = np.array(prior, dtype=float)
        self.prior = np.array(prior, dtype=float)
        self.P0 = np.diag(np.asarray(sd0, dtype=float) ** 2)
        self.P = self.P0.copy()
        self.R, self.bounded = R, bounded
        self.lam = np.sqrt(np.outer(np.broadcast_to(np.asarray(lam, dtype=float), self.th.shape),
                                    np.broadcast_to(np.asarray(lam, dtype=float), self.th.shape)))
        self.sd0 = np.asarray(sd0, dtype=float)

    def innov(self, phi, z):
        P = self._fp()
        S = float(phi @ P @ phi) + self.R
        return z - float(phi @ self.th), S

    def _fp(self):
        P = self.P / self.lam
        if self.bounded:
            r = np.sqrt(np.minimum(1.0, np.diag(self.P0) / np.diag(P)))
            P = P * np.outer(r, r)
        return P

    def update(self, phi, z):
        P = self._fp()
        S = float(phi @ P @ phi) + self.R
        g = P @ phi / S
        self.th = self.th + g * (z - float(phi @ self.th))
        P = P - np.outer(g, phi @ P)
        self.P = 0.5 * (P + P.T)

    def forget_only(self):
        self.P = self._fp()

    def reported(self):
        sd = np.sqrt(np.diag(self.P))
        return np.where(sd > FALLBACK_FRAC * self.sd0, self.prior, self.th), sd


def run_rls(d, lam, bounded, rule, gate=4.0):
    # '<rule>+struct': the STRUCTURAL parameters (s, Delta) are not forgotten (lambda 1); K and b are
    lam_s = lam
    sy = rule.endswith('+sy')          # '+sy': an axis-1 slope s_y on the Site's y (radial-pull test)
    if sy:
        rule = rule[:-len('+sy')]
    if rule.endswith('+struct'):
        rule, lam_s = rule[:-len('+struct')], 1.0
    """Replay every row in file order. Returns theta_hist (n+1, 6) of REPORTED params after rows
    0..j-1 (cols K, b_x, s, D1x, b_y, D1y), sd_hist, raw_hist and the accepted mask."""
    n = len(d)
    fa = Filt([1.0], [PRIOR_SD['K']], R_APEX, lam, bounded)
    f0 = Filt([0.0, 0.0, 0.0], [PRIOR_SD['b'], PRIOR_SD['s'], PRIOR_SD['D']], R_LAT, [lam, lam_s, lam_s], bounded)
    f1 = Filt([0.0, 0.0, 0.0], [PRIOR_SD['b'], PRIOR_SD['s'], PRIOR_SD['D']], R_LAT, [lam, lam_s, lam_s], bounded)
    hist = np.zeros((n + 1, 7))
    sdh = np.zeros((n + 1, 7))
    raw = np.zeros((n + 1, 7))
    acc = np.zeros(n, dtype=bool)
    nz = np.zeros((n, 3))

    def snap(j):
        ra, sa = fa.reported()
        r0, s0 = f0.reported()
        r1, s1 = f1.reported()
        hist[j] = [ra[0], r0[0], r0[1], r0[2], r1[0], r1[2], r1[1]]
        sdh[j] = [sa[0], s0[0], s0[1], s0[2], s1[0], s1[2], s1[1]]
        raw[j] = [fa.th[0], f0.th[0], f0.th[1], f0.th[2], f1.th[0], f1.th[2], f1.th[1]]

    snap(0)
    u0, u1, u2 = d.u0.values, d.u1.values, d.u2_apex_m.values
    y0, y1, y2 = d.y0.values, d.y1.values, d.y2_apex_m.values
    x0, x1, b1 = d.x0.values, d.x1.values, d.ball1.values
    inl = d.inlier.values
    for i in range(n):
        pa, za = np.array([u2[i]]), y2[i]
        p0, z0 = np.array([1.0, x0[i], b1[i]]), y0[i] - u0[i]
        p1, z1 = np.array([1.0, x1[i] if sy else 0.0, b1[i]]), y1[i] - u1[i]
        ia, Sa = fa.innov(pa, za)
        i0, S0 = f0.innov(p0, z0)
        i1, S1 = f1.innov(p1, z1)
        nz[i] = [ia / math.sqrt(Sa), i0 / math.sqrt(S0), i1 / math.sqrt(S1)]
        if rule == 'none':
            ok = True
        elif rule == 'band':
            ok = bool(inl[i])
        elif rule == 'innov':
            ok = bool(np.all(np.abs(nz[i]) < gate))
        elif rule == 'mband':   # the 2026-10-18 band made relative to the CURRENT model
            Kc = fa.reported()[0][0]
            ok = (abs(i0) < 0.100 and abs(i1) < 0.100 and abs(y2[i] / (Kc * u2[i]) - 1.0) < MBAND_APEX)
        else:
            raise ValueError(rule)
        acc[i] = ok
        if ok:
            fa.update(pa, za)
            f0.update(p0, z0)
            f1.update(p1, z1)
        else:
            fa.forget_only()
            f0.forget_only()
            f1.forget_only()
        snap(i + 1)
    return hist, sdh, raw, acc, nz


def f0_predict(th, u0, u1, u2, x0, b1, x1=0.0):
    """th (..., 6) K, b_x, s, D1x, b_y, D1y -> predicted (y0, y1, y2)."""
    return (u0 + th[..., 1] + th[..., 2] * x0 + th[..., 3] * b1,
            u1 + th[..., 4] + th[..., 5] * b1 + th[..., 6] * x1,
            th[..., 0] * u2)


# ----------------------------------------------------------------------------- L2 + L3
def l2_run(d, th_q, acc, hp, hv, hd, w0, k, c_lat=0.040, c_apex=0.040):
    """Predictor f0 + capped alpha r_hat per row. th_q (n, 6) is the f0 each row is queried with."""
    n = len(d)
    U = d[['u0', 'u1', 'u2_apex_m']].values
    Y = d[['y0', 'y1', 'y2_apex_m']].values
    X0, B1, X1 = d.x0.values, d.ball1.values, d.x1.values
    Pp = d[['rel_x', 'rel_y']].values
    Vv = d[['vx', 'vz']].values
    Dw = d.dwell.values
    avail = d.avail.values
    key = d.ball_id.astype(str).values + '|' + d.kind.values
    pools = {}
    for kk in np.unique(key):
        idx = np.flatnonzero((key == kk) & acc)
        pools[kk] = idx
    corr = np.zeros((n, 3))
    sw_out = np.zeros(n)
    capped = np.zeros((n, 3), dtype=bool)
    nsup = np.zeros(n, dtype=int)
    for i in range(n):
        pool = pools[key[i]]
        m = np.searchsorted(pool, avail[i])   # rows j < avail[i]
        if m == 0:
            continue
        J = pool[:m]
        dp = Pp[J] - Pp[i]
        dv = Vv[J] - Vv[i]
        d2 = (dp * dp).sum(1) / hp ** 2 + (dv * dv).sum(1) / hv ** 2
        if hd is not None and np.isfinite(Dw[i]):
            dd = Dw[J] - Dw[i]
            d2 = d2 + np.where(np.isfinite(dd), dd * dd / hd ** 2, 0.0)
        sup = np.flatnonzero(d2 <= SUPPORT_D2)
        if sup.size == 0:
            continue
        sup = sup[-k:]
        Js = J[sup]
        w = np.exp(-d2[sup])
        th = th_q[i]
        r = np.empty((sup.size, 3))
        r[:, 0] = Y[Js, 0] - (U[Js, 0] + th[1] + th[2] * X0[Js] + th[3] * B1[Js])
        r[:, 1] = Y[Js, 1] - (U[Js, 1] + th[4] + th[5] * B1[Js] + th[6] * X1[Js])
        r[:, 2] = Y[Js, 2] - th[0] * U[Js, 2]
        sw = w.sum()
        rh = (w[:, None] * r).sum(0) / sw
        a = sw / (sw + w0)
        c = a * rh
        capf = 1.0 - math.exp(-sw / w0)
        lim = np.array([c_lat * capf, c_lat * capf, th[0] * c_apex * capf])
        capped[i] = np.abs(c) >= lim - 1e-12
        corr[i] = np.clip(c, -lim, lim)
        sw_out[i] = sw
        nsup[i] = sup.size
    return corr, sw_out, capped, nsup


# ----------------------------------------------------------------------------- old law
def old_law(d, with_command=True):
    """learner.predict (y_d=None) of the actual command, and learner.command for the Goal's y_d
    (lateral 0, apex from the launch log) clamped +-20 mm about y_d, over rows available."""
    cfg = lr.LearnerConfig()
    X = d[['x0', 'x1', 'x2', 'x3']].values
    U = d[['u0', 'u1', 'u2_apex_m']].values
    Y = d[['y0', 'y1', 'y2_apex_m']].values
    n = len(d)
    pred = np.full((n, 3), np.nan)
    cmd = np.full((n, 3), np.nan)
    for i in range(n):
        a = d.avail.values[i]
        pred[i] = lr.predict(X[i], U[i], X[:a], U[:a], Y[:a], cfg)
        yd_apex = d.goal_apex.values[i]
        if with_command and np.isfinite(yd_apex) and 'highlow' not in str(d.pattern.values[i]):
            y_d = np.array([0.0, 0.0, yd_apex])
            u = lr.command(X[i], y_d, X[:a], U[:a], Y[:a], cfg)
            u[:2] = y_d[:2] + np.clip(u[:2] - y_d[:2], -0.020, 0.020)
            cmd[i] = u
    return pred, cmd


# ----------------------------------------------------------------------------- scoring
def score(d, P, mask):
    """RMSE and median |e| (mm) per axis of predictions P (n,3) over ``mask``."""
    E = d[['y0', 'y1', 'y2_apex_m']].values - P
    m = mask & np.all(np.isfinite(P), axis=1)
    return dict(n=int(m.sum()),
                rms0=_rms(E[m, 0]), rms1=_rms(E[m, 1]), rmsA=_rms(E[m, 2]),
                med0=_med(E[m, 0]), med1=_med(E[m, 1]), medA=_med(E[m, 2]))


def fmt_score(s):
    return ('n=%d  ax0 %.1f/%.1f  ax1 %.1f/%.1f  apex %.1f/%.1f' %
            (s['n'], s['rms0'], s['med0'], s['rms1'], s['med1'], s['rmsA'], s['medA']))


def lam_tracking(d, hist):
    """RMS (over Sittings with >= 20 inliers) of RLS K at Sitting end minus the Sitting's batch K."""
    errs = []
    for s, g in d[d.inlier].groupby('sit'):
        if len(g) < 20:
            continue
        Kb = float(g.u2_apex_m @ g.y2_apex_m / (g.u2_apex_m @ g.u2_apex_m))
        j = int(d.index[d.sit == s].max()) + 1
        errs.append(hist[j, 0] - Kb)
    return 1000 * float(np.sqrt(np.mean(np.square(errs)))) if errs else np.nan, len(errs)


# ----------------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--csv', required=True)
    ap.add_argument('--tag', required=True)
    ap.add_argument('--no-sweep', action='store_true')
    ap.add_argument('--lam', type=float, default=None, help='force the f0 lambda (default: the variant-table pick)')
    ap.add_argument('--rule', default=None, help='force the f0 inlier rule (band, mband, innov, none, +struct)')
    ap.add_argument('--sweep-only', action='store_true',
                    help='only run the L2 grid for --sweep-k with the forced f0 (--rule, --lam); writes '
                         'sweep_<tag>_k<k>.csv, read by the main run')
    ap.add_argument('--sweep-k', type=int, default=16)
    args = ap.parse_args()
    t_start = time.time()
    os.makedirs(OUT, exist_ok=True)
    rep = []
    P = rep.append

    d = mr.load(args.csv)               # sorted by t; the corpus file is already in time order
    raw_order = pd.read_csv(args.csv)
    assert np.array_equal(raw_order.t_abs_s.values, d.t_abs_s.values), 'file order != time order'
    g = parse_goals(d.t_abs_s.min(), d.t_abs_s.max())
    g.to_csv(os.path.join(OUT, 'goals.csv'), index=False)
    d = attach_goals(d, g)
    d = derive(d)
    d['era'] = eras(d)
    n = len(d)
    if args.sweep_only:
        hist, _, _, acc, _ = run_rls(d, args.lam, True, args.rule)
        th_q = hist[d.avail.values]
        Uq = d[['u0', 'u1', 'u2_apex_m']].values
        p_f0 = np.column_stack(f0_predict(th_q, Uq[:, 0], Uq[:, 1], Uq[:, 2], d.x0.values, d.ball1.values, d.x1.values))
        em = d.inlier.values
        ssm = em & (d.k_site_sit.values >= 5)
        ovm = em & d.oval.values
        out = []
        for hp in (0.02, 0.05, 0.10):
            for hv in (0.15, 0.3, 0.6):
                for hd in (0.05, 0.1, 0.2):
                    for w0 in (1.0, 2.0, 4.0):
                        corr, sw, cap, ns = l2_run(d, th_q, acc, hp, hv, hd, w0, args.sweep_k)
                        s1, s2, s3 = score(d, p_f0 + corr, em), score(d, p_f0 + corr, ssm), score(d, p_f0 + corr, ovm)
                        out.append(dict(rule=args.rule, lam=args.lam, hp=hp, hv=hv, hd=hd, w0=w0, k=args.sweep_k,
                                        rms0=s1['rms0'], rms1=s1['rms1'], rmsA=s1['rmsA'], ss_rms0=s2['rms0'],
                                        ss_rms1=s2['rms1'], ss_rmsA=s2['rmsA'], oval_rms0=s3['rms0'],
                                        oval_rms1=s3['rms1'], oval_rmsA=s3['rmsA'],
                                        cap_lat=float(cap[em, :2].any(1).mean())))
        pd.DataFrame(out).to_csv(os.path.join(OUT, 'sweep_%s_k%d_lam%s_%s.csv' % (args.tag, args.sweep_k, args.lam, args.rule.replace('+', '-'))), index=False)
        print('sweep k=%d: %d configs in %.0f s' % (args.sweep_k, len(out), time.time() - t_start))
        return
    P('# learner_replay: corpus %s (%d rows, %d Sittings, %d band-inliers)\n' %
      (args.tag, n, d.sit.nunique(), int(d.inlier.sum())))
    P('Command: `OPENBLAS_NUM_THREADS=1 python tools/probes/learner_replay.py --csv %s --tag %s%s`, run %s.\n'
      % (os.path.relpath(args.csv, REPO), args.tag, ' --no-sweep' if args.no_sweep else '',
         time.strftime('%Y-%m-%d %H:%M')))

    # ---- D3: patterns / dwell coverage from the launch logs
    P('## D3. Pattern, dwell and oval rows from the launch logs\n')
    cov = d.groupby('sit').agg(start=('t_abs_s', 'min'), n=('x0', 'size'),
                               pat=('pattern', lambda s: ','.join(sorted(set(x for x in s if x))) or '?'),
                               pat_known=('pattern', lambda s: float(np.mean([bool(x) for x in s]))),
                               apex_known=('goal_apex', lambda s: float(np.isfinite(s).mean())),
                               dwell_known=('dwell', lambda s: float(np.isfinite(s).mean())),
                               oval_rows=('oval', 'sum'), x075=('x0', lambda s: int(np.isclose(s, OVAL_SITE_X).sum())))
    cov['start'] = pd.to_datetime(cov.start, unit='s').dt.tz_localize('UTC').dt.tz_convert('Australia/Sydney').dt.strftime('%m-%d %H:%M')
    cov.to_csv(os.path.join(OUT, 'sittings_%s.csv' % args.tag))
    P('Goal lines parsed: %d (%d dispatched, %d started) from %d launch logs.\n' %
      (len(g), int((g.src == 'goal').sum()), int((g.src == 'started').sum()), g.logdir.nunique()))
    P('Sittings with the pattern recovered for >= 90 %% of rows: %d / %d; apex: %d; dwell: %d. '
      'Rows: pattern %d, apex %d, dwell %d of %d.\n' %
      ((cov.pat_known >= 0.9).sum(), len(cov), (cov.apex_known >= 0.9).sum(), (cov.dwell_known >= 0.9).sum(),
       int((d.pattern != '').sum()), int(np.isfinite(d.goal_apex).sum()), int(np.isfinite(d.dwell).sum()), n))
    ov = d[np.isclose(d.x0, OVAL_SITE_X)]
    P('Rows at Site x0 = -0.075: %d, first %s; their logged patterns: %s. Rows whose logged pattern is '
      'oval*: %d (of which at x0 = -0.075: %d). release_kind carried = %d rows.\n' %
      (len(ov), pd.to_datetime(ov.t_abs_s.min(), unit='s').strftime('%Y-%m-%d') if len(ov) else '-',
       dict(ov.pattern.replace('', '?').value_counts()),
       int(d.pattern.astype(str).str.startswith('oval').sum()),
       int((d.pattern.astype(str).str.startswith('oval') & np.isclose(d.x0, OVAL_SITE_X)).sum()),
       int(d.oval.sum())))
    P('Per-Sitting table: `sittings_%s.csv`.\n' % args.tag)
    P('```\n' + cov.to_string(float_format=lambda v: '%.2f' % v) + '\n```\n')

    # ---- A: batch fit reproduction and eras
    P('## A. Batch f0 fit (OLS on band-inliers)\n')
    rows = []
    D = d[d.inlier]
    for label, sub in [('ALL', D)] + [(e, D[D.era == e]) for e in sorted(d.era.unique())] \
            + [('vertical (free_cup)', D[~D.oval]), ('oval (carried)', D[D.oval])]:
        if len(sub) < 5:
            continue
        f = batch_fit(sub)
        f['set'] = label
        rows.append(f)
    for site in sorted(D.site.unique()):
        sub = D[D.site == site]
        rows.append(dict(set='site %s' % site, n=len(sub),
                         K=float(sub.u2_apex_m @ sub.y2_apex_m / (sub.u2_apex_m @ sub.u2_apex_m)),
                         b_x_mm=1000 * sub.e0.mean(), b_y_mm=1000 * sub.e1.mean(),
                         balls=','.join(str(b) for b in sorted(sub.ball_id.unique()))))
    for pat, sub in D.groupby(D.pattern.replace('', '?')):
        rows.append(dict(set='pattern %s' % pat, n=len(sub),
                         K=float(sub.u2_apex_m @ sub.y2_apex_m / (sub.u2_apex_m @ sub.u2_apex_m)),
                         b_x_mm=1000 * sub.e0.mean(), b_y_mm=1000 * sub.e1.mean()))
    bt = pd.DataFrame(rows)
    bt.to_csv(os.path.join(OUT, 'batch_%s.csv' % args.tag), index=False)
    P('```\n' + bt.to_string(index=False, float_format=lambda v: '%.4f' % v) + '\n```\n')
    # per-Sitting K (drift)
    ks = D.groupby('sit').apply(lambda s: pd.Series(dict(
        n=len(s), K=float(s.u2_apex_m @ s.y2_apex_m / (s.u2_apex_m @ s.u2_apex_m)),
        b_x_mm=1000 * s.e0.mean(), b_y_mm=1000 * s.e1.mean())))
    ks.to_csv(os.path.join(OUT, 'sitting_K_%s.csv' % args.tag))
    ksb = ks[ks.n >= 20]
    tt = d.groupby('sit').t_abs_s.mean().loc[ksb.index] / 86400.0
    slope = np.polyfit(tt.values, ksb.K.values, 1)[0] if len(ksb) > 2 else np.nan
    P('Per-Sitting K (Sittings with >= 20 inliers, n=%d): min %.4f max %.4f sd %.4f; linear drift %+.5f /day '
      '(`sitting_K_%s.csv`).\n' % (len(ksb), ksb.K.min(), ksb.K.max(), ksb.K.std(), slope, args.tag))
    # D: confounding of Delta_1 with site
    ct = pd.crosstab(d.site, d.ball_id)
    both = ct[(ct.get(0, 0) > 0) & (ct.get(1, 0) > 0)] if 1 in ct.columns else ct.iloc[0:0]
    P('Sites flown with BOTH balls (the within-Site ball contrast that separates Delta_1 from the Site): %s.\n'
      % (', '.join('%s (%d/%d rows)' % (s, r[0], r[1]) for s, r in both.iterrows()) or 'none'))
    for s in both.index:
        sub = D[D.site == s]
        if sub.ball1.sum() >= 3 and (1 - sub.ball1).sum() >= 3:
            c0 = 1000 * (sub[sub.ball1 == 1].e0.mean() - sub[sub.ball1 == 0].e0.mean())
            c1 = 1000 * (sub[sub.ball1 == 1].e1.mean() - sub[sub.ball1 == 0].e1.mean())
            # same-Sitting contrast
            wc = []
            for _, gg in sub.groupby('sit'):
                if gg.ball1.sum() >= 2 and (1 - gg.ball1).sum() >= 2:
                    wc.append((1000 * (gg[gg.ball1 == 1].e0.mean() - gg[gg.ball1 == 0].e0.mean()),
                               1000 * (gg[gg.ball1 == 1].e1.mean() - gg[gg.ball1 == 0].e1.mean()), len(gg)))
            P('  Site %s: ball1 - ball0 = %+.1f / %+.1f mm (inliers, pooled); same-Sitting contrasts: %s\n' %
              (s, c0, c1, '; '.join('%+.1f/%+.1f (n=%d)' % w for w in wc) or 'none'))
    P('\n')

    # ---- apex level: does one gain hold over 0.8..1.4 m, and does the lateral offset scale with apex?
    bins = [0.0, 0.86, 0.90, 0.96, 1.10, 9.9]
    D2 = D.assign(abin=pd.cut(D.u2_apex_m, bins))
    lv = D2.groupby('abin').apply(lambda s: pd.Series(dict(
        n=len(s), K=float(s.u2_apex_m @ s.y2_apex_m / (s.u2_apex_m @ s.u2_apex_m)) if len(s) else np.nan,
        ratio_sd=float((s.y2_apex_m / s.u2_apex_m).std()) if len(s) > 1 else np.nan,
        e0_mm=1000 * s.e0.mean() if len(s) else np.nan, e1_mm=1000 * s.e1.mean() if len(s) else np.nan,
        ball1=float(s.ball1.mean()) if len(s) else np.nan)))
    P('### Apex level (band-inliers by commanded apex u2)\n```\n' + lv.to_string(float_format=lambda v: '%.4f' % v)
      + '\n```\n')
    if len(D):
        A = np.column_stack([D.u2_apex_m, np.ones(len(D))])
        kc = np.linalg.lstsq(A, D.y2_apex_m.values, rcond=None)[0]
        P('Affine apex fit y2 = %.4f u2 %+.1f mm (resid %.1f mm) vs through-origin K %.4f (resid %.1f mm).\n' %
          (kc[0], 1000 * kc[1], _rms(D.y2_apex_m - A @ kc), batch_fit(D)['K'], batch_fit(D)['apex_resid_mm']))
        lo_a, hi_a = D[D.u2_apex_m <= 0.96], D[D.u2_apex_m > 1.0]
        if len(hi_a) >= 10:
            Kl = float(lo_a.u2_apex_m @ lo_a.y2_apex_m / (lo_a.u2_apex_m @ lo_a.u2_apex_m))
            al = np.linalg.lstsq(np.column_stack([lo_a.u2_apex_m, np.ones(len(lo_a))]), lo_a.y2_apex_m.values,
                                 rcond=None)[0]
            P('Extrapolation to apex > 1.0 m (%d rows) from fits on apex <= 0.96 m (%d rows): gain K %.4f -> apex RMS '
              '%.1f mm (mean %+.1f); affine -> %.1f mm (mean %+.1f); identity %.1f mm.\n' %
              (len(hi_a), len(lo_a), Kl, _rms(hi_a.y2_apex_m - Kl * hi_a.u2_apex_m),
               1000 * (hi_a.y2_apex_m - Kl * hi_a.u2_apex_m).mean(),
               _rms(hi_a.y2_apex_m - al[0] * hi_a.u2_apex_m - al[1]),
               1000 * (hi_a.y2_apex_m - al[0] * hi_a.u2_apex_m - al[1]).mean(), _rms(hi_a.y2_apex_m - hi_a.u2_apex_m)))
            for ax in (0, 1):
                e = lo_a['e%d' % ax].values
                c_const = e.mean()
                th = float(lo_a.u2_apex_m.values @ e / (lo_a.u2_apex_m.values @ lo_a.u2_apex_m.values))
                eh = hi_a['e%d' % ax].values
                P('  lateral axis %d at apex > 1.0 m: constant offset (%.1f mm) -> RMS %.1f mm, mean resid %+.1f; '
                  'offset proportional to apex (aim angle, %.1f mm per m) -> RMS %.1f, mean resid %+.1f mm.\n' %
                  (ax, 1000 * c_const, _rms(eh - c_const), 1000 * (eh - c_const).mean(), 1000 * th,
                   _rms(eh - th * hi_a.u2_apex_m.values), 1000 * (eh - th * hi_a.u2_apex_m.values).mean()))
    # ---- y-axis Site dependence (P2(b) question): x-only s (plan) vs an own-axis slope on both axes
    def _lat_fit(tr, te, with_sy, with_ball):
        out = []
        for ax, xc in ((0, 'x0'), (1, 'x1')):
            cols = [np.ones(len(tr))] + ([tr[xc].values] if (ax == 0 or with_sy) else []) + \
                   ([tr.ball1.values] if with_ball else [])
            b = np.linalg.lstsq(np.column_stack(cols), tr['e%d' % ax].values, rcond=None)[0]
            ct = [np.ones(len(te))] + ([te[xc].values] if (ax == 0 or with_sy) else []) + \
                 ([te.ball1.values] if with_ball else [])
            out.append((b, te['e%d' % ax].values - np.column_stack(ct) @ b))
        return out
    turned = D[D.x1.abs() > 1e-6]
    if len(turned) >= 10:
        P('### Off-axis Sites (x1 != 0; %d inlier rows, Sittings %s)\n' % (len(turned), sorted(turned.sit.unique())))
        for with_sy in (False, True):
            for with_ball in (True, False):
                ins = _lat_fit(D, D, with_sy, with_ball)
                lo_r = [[], []]
                for site in sorted(turned.site.unique()):
                    r2 = _lat_fit(D[D.site != site], D[D.site == site], with_sy, with_ball)
                    lo_r[0].append(r2[0][1])
                    lo_r[1].append(r2[1][1])
                P('- s on %s, Delta_1 %s: axis-1 coefs %s; in-sample RMS %.1f / %.1f mm; leave-one-off-axis-Site-out '
                  'RMS on those Sites %.1f / %.1f mm\n' %
                  ('x AND y' if with_sy else 'x only', 'in' if with_ball else 'out',
                   np.round(1000 * ins[1][0], 1) if not with_sy else np.round(ins[1][0] * [1000, 1, 1000][:len(ins[1][0])], 3),
                   _rms(ins[0][1]), _rms(ins[1][1]), _rms(np.concatenate(lo_r[0])), _rms(np.concatenate(lo_r[1]))))
    # ---- LOSO cold start (batch)
    lo = loso_coldstart(d)
    lo.to_csv(os.path.join(OUT, 'loso_%s.csv' % args.tag), index=False)
    P('### Held-out Site (leave-one-Site-out batch f0; RMS mm)\n```\n' +
      lo.to_string(index=False, float_format=lambda v: '%.1f' % v) + '\n```\n')

    # ---- C/B: RLS variants
    P('## C. RLS f0 over the corpus (file order)\n')
    P('Prior: K=1 (sd %.2f), b=0 (sd %.0f mm), s=0 (sd %.2f /m), Delta=0 (sd %.0f mm); R: apex %.0f mm, lateral '
      '%.0f mm; fallback when posterior sd > %.1f x prior sd.\n' %
      (PRIOR_SD['K'], 1000 * PRIOR_SD['b'], PRIOR_SD['s'], 1000 * PRIOR_SD['D'], 1000 * math.sqrt(R_APEX),
       1000 * math.sqrt(R_LAT), FALLBACK_FRAC))
    av = d.avail.values
    U = d[['u0', 'u1', 'u2_apex_m']].values
    evalmask = d.inlier.values & (np.arange(n) >= 0)
    ss_mask = evalmask & (d.k_site_sit.values >= 5)
    lam_rows = []
    cache = {}
    variants = [(r, b) for r in ('band', 'mband') for b in (False, True)] + [('none', True), ('innov', True),
                                                                            ('band+struct', True), ('mband+struct', True),
                                                                            ('band+struct+sy', True)]
    for rule, bounded in variants:
        if True:
            for lam in LAMBDAS:
                hist, sdh, raw, acc, nz = run_rls(d, lam, bounded, rule)
                th_q = hist[av]
                p = np.column_stack(f0_predict(th_q, U[:, 0], U[:, 1], U[:, 2], d.x0.values, d.ball1.values, d.x1.values))
                s_all = score(d, p, evalmask)
                s_ss = score(d, p, ss_mask)
                trk, ntrk = lam_tracking(d, hist)
                fallbacks = int(((sdh[1:, 2] > FALLBACK_FRAC * PRIOR_SD['s']) &
                                 (np.maximum.accumulate(sdh[1:, 2] <= FALLBACK_FRAC * PRIOR_SD['s']))).sum())
                lam_rows.append(dict(rule=rule, bounded=bounded, lam=lam, accepted=int(acc.sum()),
                                     rms0=s_all['rms0'], rms1=s_all['rms1'], rmsA=s_all['rmsA'],
                                     ss_rms0=s_ss['rms0'], ss_rms1=s_ss['rms1'], ss_rmsA=s_ss['rmsA'],
                                     K_end=hist[-1, 0], b_x_end_mm=1000 * hist[-1, 1], s_end=hist[-1, 2],
                                     D1x_end_mm=1000 * hist[-1, 3], b_y_end_mm=1000 * hist[-1, 4],
                                     D1y_end_mm=1000 * hist[-1, 5], s_y_end=hist[-1, 6], K_track_mm=trk,
                                     s_fallback_rows_after_leaving=fallbacks,
                                     s_sd_max=float(sdh[1:, 2].max())))
                cache[(rule, bounded, lam)] = (hist, sdh, raw, acc, nz)
    lt = pd.DataFrame(lam_rows)
    lt.to_csv(os.path.join(OUT, 'rls_variants_%s.csv' % args.tag), index=False)
    P('Predictor RMS (mm) of f0 alone over band-inlier rows (all / steady state >= 6th throw at a Site in '
      'a Sitting); K_track = RMS of (RLS K at Sitting end - that Sitting\'s batch K) x 1000; '
      's_fallback = rows where s had left its prior and fell back (wind-up):\n')
    P('```\n' + lt.to_string(index=False, float_format=lambda v: '%.4f' % v) + '\n```\n')

    # choose: bounded, rule, lam by all-row predictor MSE sum
    lt['mse'] = lt.rms0 ** 2 + lt.rms1 ** 2 + lt.rmsA ** 2
    best = lt.sort_values('mse').iloc[0]
    rule_b, bnd_b = (args.rule, True) if args.rule else (best.rule, bool(best.bounded))
    lam_b = args.lam if args.lam is not None else float(best.lam)
    sub = lt[(lt.rule == rule_b) & (lt.bounded == bnd_b)].set_index('lam')
    P('Chosen f0 filter: rule=%s bounded=%s lambda=%.3f (lowest summed MSE). Neighbours at the same rule: %s.\n' %
      (rule_b, bnd_b, lam_b, ', '.join('lambda %.3f: %.2f/%.2f/%.2f' % (l, r.rms0, r.rms1, r.rmsA)
                                        for l, r in sub.iterrows())))
    hist, sdh, raw, acc, nz = cache[(rule_b, bnd_b, lam_b)] if (rule_b, bnd_b, lam_b) in cache else \
        run_rls(d, lam_b, bnd_b, rule_b)
    th_q = hist[av]
    names = ['K', 'b_x', 's', 'D1x', 'b_y', 'D1y', 's_y']
    tr = pd.DataFrame(hist[1:], columns=names)
    tr.insert(0, 't_abs_s', d.t_abs_s.values)
    tr.insert(0, 'row', np.arange(n))
    for c, nm in enumerate(names):
        tr['sd_' + nm] = sdh[1:, c]
        tr['raw_' + nm] = raw[1:, c]
    tr['accepted'] = acc
    tr['sit'] = d.sit.values
    tr.to_csv(os.path.join(OUT, 'theta_%s.csv' % args.tag), index=False)
    leave = {}
    for c, nm in enumerate(names):
        prior_v = [1.0, 0, 0, 0, 0, 0, 0][c]
        sd0 = [PRIOR_SD['K'], PRIOR_SD['b'], PRIOR_SD['s'], PRIOR_SD['D'], PRIOR_SD['b'], PRIOR_SD['D'], PRIOR_SD['s']][c]
        left = np.flatnonzero(sdh[1:, c] <= FALLBACK_FRAC * sd0)
        leave[nm] = int(left[0]) + 1 if left.size else None
    jumps = {nm: float(np.max(np.abs(np.diff(hist[:, c])))) for c, nm in enumerate(names)}
    jrow = {nm: int(np.argmax(np.abs(np.diff(hist[:, c])))) for c, nm in enumerate(names)}
    P('Theta at the end: K %.4f, b %+.1f / %+.1f mm, s %+.3f /m, Delta_1 %+.1f / %+.1f mm [s_y %+.3f] (`theta_%s.csv`).\n' %
      (hist[-1, 0], 1000 * hist[-1, 1], 1000 * hist[-1, 4], hist[-1, 2], 1000 * hist[-1, 3], 1000 * hist[-1, 5],
       hist[-1, 6], args.tag))
    P('Rows to leave the prior (first row index + 1 at which sd <= %.1f x prior sd): %s. Ball 1 first row: %s; '
      'second distinct Site first row: %s.\n' %
      (FALLBACK_FRAC, leave, int(np.flatnonzero(d.ball1.values)[0]) if d.ball1.any() else None,
       int(np.flatnonzero(d.site.values != d.site.values[0])[0])))
    P('Largest single-row jump of each REPORTED parameter (value, at row): %s.\n' %
      ', '.join('%s %.4g @%d' % (nm, jumps[nm], jrow[nm]) for nm in names))
    # gap behaviour
    gaps = np.flatnonzero(np.diff(d.sit.values) > 0) + 1
    dk = [hist[gi + 1, 0] - hist[gi, 0] for gi in gaps]
    P('Across the %d Sitting gaps: |Delta K| over the first row of the new Sitting: median %.4f, max %.4f '
      '(K has no time term: forgetting is per row, a gap costs nothing and does not inflate P).\n' %
      (len(gaps), float(np.median(np.abs(dk))) if dk else np.nan, float(np.max(np.abs(dk))) if dk else np.nan))
    # outliers with / without the inlier rule
    for rule in ('none', 'band', 'mband', 'innov', 'band+struct', 'mband+struct', 'band+struct+sy'):
        h2 = cache.get((rule, True, lam_b))
        if h2 is None:
            continue
        P('  rule %-12s: accepted %d / %d rows; end K %.4f b %+.1f/%+.1f mm s %+.3f D1 %+.1f/%+.1f mm; max |jump| K %.4f b_x %.1f mm\n'
          % (rule, int(h2[3].sum()), n, h2[0][-1, 0], 1000 * h2[0][-1, 1], 1000 * h2[0][-1, 4], h2[0][-1, 2],
             1000 * h2[0][-1, 3], 1000 * h2[0][-1, 5], np.max(np.abs(np.diff(h2[0][:, 0]))),
             1000 * np.max(np.abs(np.diff(h2[0][:, 1])))))
    # innov vs band agreement
    hb, hi = cache.get(('band', True, lam_b)), cache.get(('mband', True, lam_b))
    if hb is not None and hi is not None:
        P('  mband (model-relative band) vs band: both reject %d, band-only %d, innov-only %d; '
          'uncaught rows rejected by mband: %d / %d; clamped rows rejected: %d / %d.\n' %
          (int((~hb[3] & ~hi[3]).sum()), int((~hb[3] & hi[3]).sum()), int((hb[3] & ~hi[3]).sum()),
           int((~hi[3] & ~d.caught.values).sum()), int((~d.caught.values).sum()),
           int((~hi[3] & d.at_clamp_old.values).sum()), int(d.at_clamp_old.sum())))
    P('Uncaught rows: %d; band-outliers among them %d; band-outliers among caught %d.\n' %
      (int((~d.caught).sum()), int((~d.caught & ~d.inlier).sum()), int((d.caught & ~d.inlier).sum())))
    P('\n')

    # ---- B: predictors
    P('## B. Chronological replay as predictors (RMS / median |e| mm; band-inlier rows scored)\n')
    t0 = time.time()
    p_old, c_old = old_law(d)
    P('Old law (learner.predict, today\'s LearnerConfig defaults) computed in %.1f s.\n' % (time.time() - t0))
    p_id = U.copy()
    p_f0 = np.column_stack(f0_predict(th_q, U[:, 0], U[:, 1], U[:, 2], d.x0.values, d.ball1.values, d.x1.values))

    # L2 sweep
    base = dict(hp=0.05, hv=0.3, hd=0.1, w0=2.0, k=16)
    files = sorted(glob.glob(os.path.join(OUT, 'sweep_%s_k*.csv' % args.tag)))
    sweep_rows = pd.concat([pd.read_csv(f) for f in files]) if (files and not args.no_sweep) else pd.DataFrame()
    if len(sweep_rows):
        sweep_rows = sweep_rows[(sweep_rows.rule == rule_b) & np.isclose(sweep_rows.lam, lam_b)]
    t0 = time.time()
    sw_t = sweep_rows.copy()
    if len(sw_t):
        sw_t['mse'] = sw_t.rms0 ** 2 + sw_t.rms1 ** 2 + sw_t.rmsA ** 2
        sw_t = sw_t.sort_values('mse')
        sw_t.to_csv(os.path.join(OUT, 'sweep_%s.csv' % args.tag), index=False)
        P('(grid read from %s, f0 = %s lambda %.3f)\n' % (', '.join(os.path.basename(f) for f in files), rule_b, lam_b))
        bb = sw_t.iloc[0]
        base = dict(hp=bb.hp, hv=bb.hv, hd=bb.hd, w0=bb.w0, k=int(bb.k))
        P('L2 grid (%d configs, read in %.0f s). Best: %s -> %.2f/%.2f/%.2f mm; worst %.2f/%.2f/%.2f; spread of '
          'summed MSE best..median..worst %.1f..%.1f..%.1f mm^2 (`sweep_%s.csv`).\n' %
          (len(sw_t), time.time() - t0, base, bb.rms0, bb.rms1, bb.rmsA, sw_t.iloc[-1].rms0, sw_t.iloc[-1].rms1,
           sw_t.iloc[-1].rmsA, sw_t.mse.min(), sw_t.mse.median(), sw_t.mse.max(), args.tag))
        # one-at-a-time neighbours of the best
        for par, vals in (('hp', (0.02, 0.05, 0.10)), ('hv', (0.15, 0.3, 0.6)), ('hd', (0.05, 0.1, 0.2)),
                          ('w0', (1.0, 2.0, 4.0)), ('k', (8, 16, 32))):
            parts = []
            for v in vals:
                q = sw_t
                for p2, v2 in base.items():
                    q = q[np.isclose(q[p2], v if p2 == par else v2)]
                if len(q):
                    parts.append('%g: %.2f/%.2f/%.2f' % (v, q.iloc[0].rms0, q.iloc[0].rms1, q.iloc[0].rmsA))
            P('  %s -> %s\n' % (par, '; '.join(parts)))
    corr, sw, cap, ns = l2_run(d, th_q, acc, base['hp'], base['hv'], base['hd'], base['w0'], base['k'])
    corr_nod, _, _, _ = l2_run(d, th_q, acc, base['hp'], base['hv'], None, base['w0'], base['k'])
    p_l2 = p_f0 + corr
    preds = [('identity (y=u)', p_id), ('old law predict', p_old), ('f0 RLS', p_f0), ('f0 + L2 (chosen)', p_l2),
             ('f0 + L2 no dwell term', p_f0 + corr_nod)]
    for label, mask in (('all band-inlier rows', evalmask), ('steady state (>= 6th at Site in Sitting)', ss_mask),
                        ('oval (carried) rows', evalmask & d.oval.values),
                        ('vertical rows', evalmask & ~d.oval.values),
                        ('ALL rows incl. outliers', np.ones(n, dtype=bool))):
        P('**%s**\n' % label)
        for nm, pp in preds:
            P('- %-24s %s\n' % (nm, fmt_score(score(d, pp, mask))))
    for e in sorted(d.era.unique()):
        m = evalmask & (d.era.values == e)
        P('- era %s: old %s | f0+L2 %s\n' % (e, fmt_score(score(d, p_old, m)), fmt_score(score(d, p_l2, m))))
    # clamp / cap shares
    P('\nOld law at the +-20 mm clamp (|u| >= 19.9 mm on an axis; y_d,xy = 0): any axis %.1f %% (ax0 %.1f, ax1 %.1f) '
      'of all rows, %.1f %% of inliers; rows with |u_xy| > 20.0 mm (the 30 mm authority flown): %d. '
      'By era: %s.\n' %
      (100 * d.at_clamp_old.mean(), 100 * d.at_clamp_old0.mean(), 100 * d.at_clamp_old1.mean(),
       100 * d[d.inlier].at_clamp_old.mean(), int(((d.u0.abs() > 0.02001) | (d.u1.abs() > 0.02001)).sum()),
       ', '.join('%s %.0f %%' % (e, 100 * d[d.era == e].at_clamp_old.mean()) for e in sorted(d.era.unique()))))
    P('New law at its cap(x) (lateral c_max 40 mm, apex c_max 40 mm): lateral any axis %.2f %%, apex %.2f %% of '
      'inlier rows; |alpha r_hat| p50/p95/p99/max lateral %.1f/%.1f/%.1f/%.1f mm, apex (command space) '
      '%.1f/%.1f/%.1f/%.1f mm.\n' %
      (100 * cap[evalmask, :2].any(1).mean(), 100 * cap[evalmask, 2].mean(),
       *[1000 * np.percentile(np.abs(corr[evalmask, :2]).max(1), q) for q in (50, 95, 99, 100)],
       *[1000 * np.percentile(np.abs(corr[evalmask, 2] / th_q[evalmask, 0]), q) for q in (50, 95, 99, 100)]))
    need_a = np.abs(d.y2_apex_m.values - th_q[:, 0] * U[:, 2]) / th_q[:, 0]
    need_l = np.abs(d[['y0', 'y1']].values - p_f0[:, :2]).max(1)
    P('Correction each row NEEDED beyond f0 (single row, noise included): apex p50/p95/p99/max %.1f/%.1f/%.1f/%.1f mm '
      '(inliers), %.1f max (all rows); lateral %.1f/%.1f/%.1f/%.1f mm.\n' %
      (*[1000 * np.percentile(need_a[evalmask], q) for q in (50, 95, 99, 100)], 1000 * need_a.max(),
       *[1000 * np.percentile(need_l[evalmask], q) for q in (50, 95, 99, 100)]))
    # sum-of-w and support stats
    P('L2 support: rows with any support %.1f %%; sum w p50 %.2f; alpha p50 %.2f.\n' %
      (100 * (ns[evalmask] > 0).mean(), float(np.median(sw[evalmask])),
       float(np.median(sw[evalmask] / (sw[evalmask] + base['w0'])))))

    # implied command error (old: replayed command vs u_ideal; new: = predictor error laterally, / K in apex)
    yd_ok = np.isfinite(c_old[:, 2])
    y = d[['y0', 'y1', 'y2_apex_m']].values
    ydA = d.goal_apex.values
    u_ideal = np.column_stack([U[:, 0] - y[:, 0], U[:, 1] - y[:, 1], U[:, 2] * ydA / y[:, 2]])
    fid = np.abs(c_old - U)
    m = yd_ok & evalmask
    P('\nOld-law COMMAND replay (learner.command with y_d = (0, 0, Goal apex), +-20 mm clamp) on %d inlier rows with '
      'a logged apex: fidelity |u_replay - u_flown| median %.1f / %.1f / %.1f mm, within 1 mm on all axes %.0f %%. '
      'The old law\'s ACTUAL landing error vs target (y - y_d, observed): ax0 %.1f/%.1f ax1 %.1f/%.1f apex %.1f/%.1f '
      '(RMS/median mm, same rows).\n' %
      (int(m.sum()), *[1000 * np.median(fid[m, c]) for c in range(3)], 100 * np.mean(np.all(fid[m] < 0.001, 1)),
       _rms(y[m, 0]), _med(y[m, 0]), _rms(y[m, 1]), _med(y[m, 1]), _rms(y[m, 2] - ydA[m]), _med(y[m, 2] - ydA[m])))
    ce_old = c_old - u_ideal
    u_new = np.column_stack([0 - (p_l2[:, 0] - U[:, 0]), 0 - (p_l2[:, 1] - U[:, 1]),
                             (ydA - corr[:, 2]) / th_q[:, 0]])
    ce_new = u_new - u_ideal
    P('Implied COMMAND error |u_law - u_ideal| (u_ideal = u + (y_d - y) laterally, u y_d / y in apex), same rows: '
      'old law (replayed) %.1f / %.1f / %.1f mm RMS; new law (f0+L2) %.1f / %.1f / %.1f mm RMS.\n' %
      (_rms(ce_old[m, 0]), _rms(ce_old[m, 1]), _rms(ce_old[m, 2]), _rms(ce_new[m, 0]), _rms(ce_new[m, 1]),
       _rms(ce_new[m, 2])))

    # ---- cold start
    P('\n## B(1). Cold start (first throw at each Site / (Site, ball) in the corpus; error mm, y - prediction)\n')
    cs = []
    for flag in ('first_site', 'first_site_ball'):
        for i in np.flatnonzero(d[flag].values):
            r = dict(kind=flag, row=int(i), site=d.site.values[i], ball=int(d.ball_id.values[i]),
                     date=pd.to_datetime(d.t_abs_s.values[i], unit='s').strftime('%m-%d'),
                     inlier=bool(d.inlier.values[i]), K_used=th_q[i, 0])
            for nm, pp in (('id', p_id), ('old', p_old), ('f0', p_f0), ('l2', p_l2)):
                e = y[i] - pp[i]
                r[nm + '_ax0'], r[nm + '_ax1'], r[nm + '_apex'] = 1000 * e[0], 1000 * e[1], 1000 * e[2]
            cs.append(r)
    cs = pd.DataFrame(cs)
    cs.to_csv(os.path.join(OUT, 'coldstart_%s.csv' % args.tag), index=False)
    P('```\n' + cs.to_string(index=False, float_format=lambda v: '%.1f' % v) + '\n```\n')
    for flag in ('first_site', 'first_site_ball'):
        c = cs[(cs.kind == flag) & (cs.row > 0)]
        P('%s (excluding row 0, which has no data): |apex| mean identity %.1f, old %.1f, f0 %.1f, f0+L2 %.1f mm; '
          'max f0 %.1f; lateral RMS (both axes) identity %.1f, old %.1f, f0 %.1f, f0+L2 %.1f mm. n=%d.\n' %
          (flag, c.id_apex.abs().mean(), c.old_apex.abs().mean(), c.f0_apex.abs().mean(), c.l2_apex.abs().mean(),
           c.f0_apex.abs().max(),
           *[float(np.sqrt(np.mean(np.r_[c[nm + '_ax0'] ** 2, c[nm + '_ax1'] ** 2]))) for nm in ('id', 'old', 'f0', 'l2')],
           len(c)))
    for flag in ('first_site', 'first_site_ball'):
        c = cs[(cs.kind == flag) & (cs.row > 0)]
        P('%s: f0 cold-start |apex| < 15 mm in %d / %d; identity < 15 mm in %d / %d.\n' %
          (flag, int((c.f0_apex.abs() < 15).sum()), len(c), int((c.id_apex.abs() < 15).sum()), len(c)))
    # is the first throw of ANY Goal (familiar Site or not) an outlier of the plant? (cold-start floor)
    gpos = d.groupby('goal_t').cumcount().values
    has_goal = np.isfinite(d.goal_t.values)
    eA = 1000 * (y[:, 2] - p_f0[:, 2])
    for lab, mm in (('1st throw of a Goal', has_goal & (gpos == 0)), ('2nd', has_goal & (gpos == 1)),
                    ('3rd-5th', has_goal & (gpos >= 2) & (gpos < 5)), ('6th+', has_goal & (gpos >= 5))):
        mm = mm & evalmask
        P('- f0 apex error by throw-in-Goal, %-20s n=%4d  RMS %.1f  median %+.1f  mean %+.1f mm\n' %
          (lab, int(mm.sum()), _rms(eA[mm] / 1000), float(np.median(eA[mm])) if mm.any() else np.nan,
           float(np.mean(eA[mm])) if mm.any() else np.nan))
    # first 5 throws at a Site's first Sitting
    first_sit_rows = d.groupby('site').sit.transform('min') == d.sit
    m5 = first_sit_rows.values & (d.k_site_sit.values < 5) & (np.arange(n) > 0)
    for nm, pp in (('identity', p_id), ('old', p_old), ('f0', p_f0), ('f0+L2', p_l2)):
        P('- first 5 throws at each Site\'s first Sitting, %-8s %s\n' % (nm, fmt_score(score(d, pp, m5))))

    # rows csv
    out_rows = d[['t_abs_s', 'sit', 'era', 'site', 'ball_id', 'kind', 'pattern', 'goal_apex', 'dwell', 'caught',
                  'inlier', 'at_clamp_old', 'k_site_sit', 'avail']].copy()
    for nm, pp in (('old', p_old), ('f0', p_f0), ('l2', p_l2)):
        for c, ax in enumerate(('0', '1', 'A')):
            out_rows['%s_err%s_mm' % (nm, ax)] = 1000 * (y[:, c] - pp[:, c])
    out_rows['l2_sumw'] = sw
    out_rows['l2_cap_lat'] = cap[:, :2].any(1)
    out_rows['accepted'] = acc
    out_rows.to_csv(os.path.join(OUT, 'rows_%s.csv' % args.tag), index=False)
    P('\nRun time %.0f s.\n' % (time.time() - t_start))
    text = ''.join(s if s.endswith('\n') else s + '\n' for s in rep)
    with open(os.path.join(OUT, 'report_%s.md' % args.tag), 'w') as fh:
        fh.write(text)
    allr = sorted(glob.glob(os.path.join(OUT, 'report_*.md')))
    with open(os.path.join(OUT, 'report.md'), 'w') as fh:
        for f in allr:
            with open(f) as fi:
                fh.write(fi.read() + '\n\n')
    print(text)


if __name__ == '__main__':
    main()
