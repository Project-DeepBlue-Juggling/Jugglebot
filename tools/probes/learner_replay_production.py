#!/usr/bin/env python3
"""Replay the Learner memory corpus through the PRODUCTION Learner (plan test T-I1).

``tools/probes/learner_replay.py`` PROTOTYPED the law inside the probe (k = 8, w0 = 4) and produced
the phase's numbers.  This module measures the same quantities with the real classes:
``motion/skills/learner.Learner`` (production defaults k = 16, w0 = 2.0) fed row by row, with the
v1 corpus loaded the way the migration does it (``tools/migrate_memory_schema_v2.convert`` into a
TEMP v2 file, then ``memory.Memory(path).rows()``; nothing is written beside the input).

Method (the probe's, reused; function names cited):

* Predictor: row i is scored on ``Learner.predict(ctx_i, u_i)`` against ``y_i`` where the Learner has
  observed rows 0..i-2 plus row i-1 when it belongs to an earlier Sitting (probe ``derive``:
  ``avail``; Sitting = a run of rows with no ``t_abs_s`` gap > 1800 s, ``models.load``).  One Learner
  is fed incrementally through a one-row delay buffer.  The Learner observes EVERY row (the
  probe's RLS saw all rows); the plant model's own inlier rule decides acceptance.
* Scored rows ("inliers"): ``models.inlier_mask`` of 2026-10-18: |y - u| < 100 mm on both lateral
  axes and 1.0 < y_apex / u_apex < 1.2.
* Identity prior: prediction = u (probe ``p_id``).
* Cold start = the first appearance of each Site in the file (probe ``derive``: ``first_site``,
  Site = (x0, x1) rounded to 4 dp; probe "B(1)"), over every such row (the probe's two
  "excluding row 0" sets differ only by rows with no earlier data): reported over ALL first
  appearances and over those with at least one earlier row ACCEPTED by the model.  Apex = mean
  |y_apex - prediction|; lateral = RMS pooled over both axes (probe) and mean |e| over both axes.
* Steady state = the 6th and later throws at a Site within a Sitting (probe ``k_site_sit >= 5``),
  inliers only; RMS per axis via the probe's ``score``.
* At the cap = the command's lateral deviation from the model command reaches the cap
  c_max_xy x (1 - exp(-sum_w / w0)) on either lateral axis (probe ``l2_run``'s
  ``|c| >= lim - 1e-12``), sum_w from ``Learner.evidence``; share over inliers.
* y_d: a v1 row carries no target.  The probe used lateral y_d = 0 and the apex from the launch
  logs (not available here).  This replay uses y_d = (0, 0, u_apex): lateral exactly the probe's;
  the apex target only moves the recorded ``command`` / ``model_command`` apex (K-scaled), never
  the predictions nor the lateral cap test.

Usage (repo root, offline, read-only; single process):
    OPENBLAS_NUM_THREADS=1 python tools/probes/learner_replay_production.py \
        --csv temp/learn/jugglebot/memory.csv [--limit 3276] [--json OUT.json]
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import tempfile
import time

import numpy as np

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
for _p in (REPO, os.path.join(REPO, 'ros_ws', 'src', 'jugglebot'),
           os.path.join(REPO, 'tools')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import migrate_memory_schema_v2 as mig                          # noqa: E402
from jugglebot.motion.skills import memory as mem               # noqa: E402
from jugglebot.motion.skills.learner import Learner, ThrowContext  # noqa: E402

#: > 30 min between consecutive rows opens a new Sitting (``models.SITTING_GAP_S``).
SITTING_GAP_S = 1800.0
#: ``models.OUTLIER_LAT_M`` / ``RATIO_LO`` / ``RATIO_HI``.
OUTLIER_LAT_M = 0.100
RATIO_LO, RATIO_HI = 1.0, 1.2
#: Steady state: the 6th and later throw at a Site in a Sitting (``k_site_sit >= 5``).
STEADY_FROM = 5


def _rms(x) -> float:
    x = np.asarray(x, dtype=float)
    return float(np.sqrt(np.mean(x * x))) if x.size else float('nan')


def load_rows(csv_path: str, limit=None, out_v2=None):
    """Read the v1 ``csv_path`` read-only; migrate (the tool's own ``convert``) into a v2 file at
    ``out_v2`` (default: a temp dir, deleted on return); return (rows, migration stats).
    ``limit`` keeps the first N data rows of the v1 file (append-only corpus)."""
    with tempfile.TemporaryDirectory(prefix='learner_replay_') as tmp:
        src = csv_path
        if limit is not None:
            src = os.path.join(tmp, 'v1_head.csv')
            with open(csv_path, 'r', newline='') as fh, open(src, 'w', newline='') as out:
                for n, line in enumerate(fh):
                    if n > limit:           # line 0 is the header
                        break
                    out.write(line)
        text, stats = mig.convert(src)
        dst = out_v2 or os.path.join(tmp, 'memory_v2.csv')
        with open(dst, 'wb') as fh:
            fh.write(text.encode('utf-8'))
        rows = mem.Memory(dst).rows()
    return rows, stats


def _ctx(e) -> ThrowContext:
    return ThrowContext(site_xy_m=e.site_xy_m, release_m=e.release_m,
                        release_vel_mps=e.release_vel_mps, dwell_s=e.dwell_s,
                        ball_id=e.ball_id, release_kind=e.release_kind,
                        seat_offset_m=e.seat_offset_m)


def replay(rows, learner=None):
    """Replay ``rows`` (v2 Experiences in file order) as a predictor; returns per-row arrays."""
    n = len(rows)
    learner = Learner() if learner is None else learner
    t = np.array([r.t_abs_s for r in rows])
    sit = np.concatenate([[0], np.cumsum(np.diff(t) > SITTING_GAP_S)]) if n else np.zeros(0, int)
    U = np.array([r.u for r in rows]).reshape(n, 3)
    Y = np.array([r.y for r in rows]).reshape(n, 3)
    pred = np.zeros((n, 3))
    cmd = np.zeros((n, 3))
    mcmd = np.zeros((n, 3))
    at_cap = np.zeros(n, dtype=bool)
    n_acc_before = np.zeros(n, dtype=int)
    sw = np.zeros(n)
    pending = None
    for i, e in enumerate(rows):
        if pending is not None and sit[i - 1] != sit[i]:
            learner.observe(rows[pending])          # an earlier Sitting: no 2-throw delay
            pending = None
        ctx = _ctx(e)
        y_d = (0.0, 0.0, float(e.u[2]))
        pred[i] = learner.predict(ctx, e.u, y_d)
        mcmd[i] = learner.model_command(ctx, y_d)
        cmd[i] = learner.command(ctx, y_d)
        sw[i], _ = learner.evidence(ctx)
        lim = learner.config.c_max_xy_m * (1.0 - math.exp(-sw[i] / learner.config.w0))
        at_cap[i] = bool(sw[i] > 0.0 and np.any(np.abs(cmd[i, :2] - mcmd[i, :2]) >= lim - 1e-12))
        n_acc_before[i] = learner.model.n_accepted
        if pending is not None:
            learner.observe(rows[pending])
        pending = i
    return dict(learner=learner, sit=sit, U=U, Y=Y, pred=pred, command=cmd, model_command=mcmd,
                at_cap=at_cap, n_accepted_before=n_acc_before, sum_w=sw)


def headline(rows, rep) -> dict:
    """The headline numbers (see the module docstring for every definition)."""
    n = len(rows)
    U, Y, P, sit = rep['U'], rep['Y'], rep['pred'], rep['sit']
    e_id = Y - U
    inl = ((np.abs(Y[:, 0] - U[:, 0]) < OUTLIER_LAT_M) & (np.abs(Y[:, 1] - U[:, 1]) < OUTLIER_LAT_M)
           & (Y[:, 2] / U[:, 2] > RATIO_LO) & (Y[:, 2] / U[:, 2] < RATIO_HI))
    e_pr = Y - P
    site = ['%+.4f,%+.4f' % (round(r.site_xy_m[0], 4) + 0.0, round(r.site_xy_m[1], 4) + 0.0)
            for r in rows]
    seen = set()
    first = np.zeros(n, dtype=bool)
    for i, s in enumerate(site):
        if s not in seen:
            seen.add(s)
            first[i] = True
    k_in = {}
    k_site_sit = np.zeros(n, dtype=int)
    for i in range(n):
        key = (int(sit[i]), site[i])
        k_site_sit[i] = k_in.get(key, 0)
        k_in[key] = k_site_sit[i] + 1
    steady = inl & (k_site_sit >= STEADY_FROM)

    def cold(mask):
        out = {'n': int(mask.sum())}
        for nm, e in (('identity', e_id), ('production', e_pr)):
            m = mask
            out[nm] = dict(
                apex_mean_abs_mm=1e3 * float(np.mean(np.abs(e[m, 2]))) if m.any() else float('nan'),
                lat_rms_mm=1e3 * _rms(np.concatenate([e[m, 0], e[m, 1]])),
                lat_mean_abs_mm=1e3 * float(np.mean(np.abs(np.concatenate([e[m, 0], e[m, 1]]))))
                if m.any() else float('nan'))
        return out

    def axes(e, m):
        return dict(ax0=1e3 * _rms(e[m, 0]), ax1=1e3 * _rms(e[m, 1]), apex=1e3 * _rms(e[m, 2]))

    L = rep['learner']
    return dict(
        n_rows=n, n_inliers=int(inl.sum()), n_sittings=int(sit.max()) + 1 if n else 0,
        rms_all_inliers_mm=dict(identity=axes(e_id, inl), production=axes(e_pr, inl)),
        cold_start_all_first_appearances=cold(first),
        cold_start_with_earlier_accepted=cold(first & (rep['n_accepted_before'] > 0)),
        steady_state=dict(n=int(steady.sum()), identity_rms_mm=axes(e_id, steady),
                          production_rms_mm=axes(e_pr, steady)),
        share_at_cap_inliers=float(rep['at_cap'][inl].mean()) if inl.any() else float('nan'),
        share_at_cap_all=float(rep['at_cap'].mean()) if n else float('nan'),
        model_accepted=int(L.model.n_accepted), model_rejected=int(L.model.n_rejected),
        model_accepted_share=float(L.model.n_accepted) / max(1, L.model.n_accepted + L.model.n_rejected),
        K=float(L.model.theta()[0]), model_summary=L.model.summary())


def format_table(h: dict, mig_stats=None, seconds=None) -> str:
    def ax(d):
        return '%.1f / %.1f / %.1f' % (d['ax0'], d['ax1'], d['apex'])
    o = ['learner_replay_production: %d rows (%d inliers, %d Sittings)%s' % (
        h['n_rows'], h['n_inliers'], h['n_sittings'],
        '' if mig_stats is None else ', migration dropped %d' % mig_stats['dropped'])]
    o.append('RMS mm (axis 0 / axis 1 / apex), inliers')
    o.append('  all rows     identity %s | production %s' % (
        ax(h['rms_all_inliers_mm']['identity']), ax(h['rms_all_inliers_mm']['production'])))
    st = h['steady_state']
    o.append('  steady state identity %s | production %s  (n=%d)' % (
        ax(st['identity_rms_mm']), ax(st['production_rms_mm']), st['n']))
    for key, label in (('cold_start_all_first_appearances', 'all first appearances'),
                       ('cold_start_with_earlier_accepted', 'with earlier accepted row')):
        c = h[key]
        o.append('cold start, %s (n=%d)' % (label, c['n']))
        for nm in ('identity', 'production'):
            d = c[nm]
            o.append('  %-10s |apex| mean %.1f mm; lateral RMS %.1f mm, mean |e| %.1f mm' % (
                nm, d['apex_mean_abs_mm'], d['lat_rms_mm'], d['lat_mean_abs_mm']))
    o.append('share of rows at the lateral cap: %.2f %% of inliers (%.2f %% of all rows)' % (
        100 * h['share_at_cap_inliers'], 100 * h['share_at_cap_all']))
    o.append('model accepted %d / rejected %d (%.1f %%), K %.4f' % (
        h['model_accepted'], h['model_rejected'], 100 * h['model_accepted_share'], h['K']))
    o.append('model.summary(): ' + h['model_summary'])
    if seconds is not None:
        o.append('runtime %.1f s' % seconds)
    return '\n'.join(o)


def run(csv_path: str, limit=None, out_v2=None):
    """Load, replay, summarise. Returns (headline dict, migration stats, replay arrays, seconds)."""
    t0 = time.time()
    rows, stats = load_rows(csv_path, limit=limit, out_v2=out_v2)
    rep = replay(rows)
    h = headline(rows, rep)
    return h, stats, rep, time.time() - t0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--csv', required=True, help='a schema-v1 memory.csv (read only)')
    ap.add_argument('--limit', type=int, default=None, help='replay only the first N data rows')
    ap.add_argument('--json', default=None, help='also dump the headline dict here')
    args = ap.parse_args(argv)
    h, stats, _, secs = run(args.csv, limit=args.limit)
    print(format_table(h, stats, secs))
    if args.json:
        with open(args.json, 'w') as fh:
            json.dump(h, fh, indent=2, sort_keys=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
