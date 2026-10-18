"""Plant-error structure in the learner memory corpus: global model vs site-idiosyncratic.

Regresses ``temp/learn/jugglebot/memory.csv`` (one row per flown throw: release site
``x0..x3``, the learner's commanded landing ``u0,u1`` and commanded apex ``u2_apex_m``, the
measured landing ``y0,y1`` and apex ``y2_apex_m``, ``t_abs_s``, ``ball_id``, ``caught``) to
answer ONE question: is the plant error a few GLOBAL terms, or does each site need its own
memory?  Writes, under ``--out``:

  sittings.csv / sites.csv / sittings_bias.csv   the sittings table, the per-site and
                                                  per-sitting bias + apex-ratio tables
  apex_fit.csv                                    the apex-gain fits (through-origin, affine, offset)
  x0_slopes.csv                                   the x0-linear centre-pull fits (all rows / ball-0 / FE)
  loso.csv / chrono.csv / forward_chain.csv       out-of-sample RMSE per model: leave-one-site-out,
                                                  chronological split, forward chaining
  r2_ladder.csv                                   in-sample R2 of 1..51-parameter models
  fig_resid_vs_site.png, fig_apex_ratio.png       the two figures
  report.md                                       everything above, with the headline numbers
                                                  and a reproduction check against 2026-10-18

Pure numpy / pandas / matplotlib (Agg); no scipy, no repo imports, no robot.

Method notes (all in the report): analysis set = "inliers" (|e| < 100 mm both axes and apex
ratio in (1.0, 1.2); the rest are misses / bad mocap fits).  Standard errors cluster by
sitting.  u is the learner's OUTPUT, so the cross-section is closed-loop: pooled slopes of y
on u are NOT plant gains (the within-cell slope is the honest, if weak, estimate).

Motivating entry: logbook/2026-10-18-learner-generalisation-investigation.md (2026-10-18).
2026-10-18 result (846 rows, 825 inliers, 17 sittings): apex gain K = k^2 = 1.085 (global,
site-independent); shared lateral offset +9.9 mm (x) / +15.0 mm (y); an x0-linear centre pull
in axis 0 only (about -0.06 per m ball-0, -0.10 pooled); a +15..+18 mm ball_id=1 contrast
(confounded with site x0 = -0.0625); shot noise about 20 mm per axis, so no model reaches
below ~20-21 mm lateral RMSE.

    python tools/probes/memory_regression/regress.py \
        [--csv temp/learn/jugglebot/memory.csv] [--out temp/probes/memory_regression/]
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
sys.path[:0] = [HERE]
import models as M  # noqa: E402

DEFAULT_CSV = os.path.join('temp', 'learn', 'jugglebot', 'memory.csv')
DEFAULT_OUT = os.path.join('temp', 'probes', 'memory_regression')
CHRONO_CUT = 9            # train sittings <= 9, test > 9: halves the rows (410 / 415 on 2026-10-18)
FORWARD_FIRST = 4         # forward chaining predicts sittings >= 4 from all earlier ones
SPLIT_B = 5               # second split, as a stability check; sitting 5 is the 2026-10-04 sitting, the date the
                          # learner law was revised (task brief 2026-10-18; the revision is not recorded in the CSV)
# Plane-move groups: the catch plane was moved 830 -> 930 -> 830 mm around sittings 5-7. Provenance: the 2026-10-18
# investigation brief, NOT recorded in the CSV; the indices are this probe's own gap > 30 min sitting split, so they
# only mean the same sittings while the corpus keeps those 17 sittings in that order. Edit if the corpus changes.
PLANE_MOVE_GROUPS = (('sit <= 4', 0, 4), ('sit 5-7', 5, 7), ('sit >= 8', 8, 10 ** 6))

R = []                    # report lines (also printed)


def say(s: str = '') -> None:
    print(s)
    R.append(s)


def md_table(header, rows) -> None:
    say('| ' + ' | '.join(header) + ' |')
    say('|' + '---|' * len(header))
    for r in rows:
        say('| ' + ' | '.join(str(c) for c in r) + ' |')


def r2_of(y, groups) -> float:
    y = np.asarray(y, float)
    m = pd.Series(y).groupby(np.asarray(groups)).transform('mean').values
    return 1.0 - ((y - m) ** 2).sum() / ((y - y.mean()) ** 2).sum()


# ----------------------------------------------------------------------------- sections
def sec_sittings(c: M.Corpus, out: str) -> None:
    a = c.all
    say('## 1. Sittings (gap > 30 min; %d sittings)' % a.sit.nunique())
    rows, recs = [], []
    for s, g in a.groupby('sit'):
        sites = ','.join('%g' % x for x in sorted(set(g.x0.round(4))))
        rec = (s, g.date.min().strftime('%m-%d %H:%M'), sites, len(g),
               '%.3f-%.3f' % (g.u2_apex_m.min(), g.u2_apex_m.max()),
               '%.0f%%' % (100 * g.caught.mean()), '%.0f%%' % (100 * g.ball_id.mean()))
        rows.append(rec)
        recs.append(dict(sit=s, start_utc=rec[1], sites_x0=sites, n=len(g), u2_min=g.u2_apex_m.min(),
                         u2_max=g.u2_apex_m.max(), caught_frac=g.caught.mean(), ball1_frac=g.ball_id.mean()))
    md_table(['sit', 'start (UTC)', 'sites x0 [m]', 'n', 'u2 apex range [m]', 'caught', 'ball1 frac'], rows)
    pd.DataFrame(recs).to_csv(os.path.join(out, 'sittings.csv'), index=False)
    D = c.D
    cl = D.u1 <= -M.CLAMP_M
    say('')
    say('Learner clamp: u1 sits at -20 mm in %.0f%% of inlier rows (u0 at +-20 mm in %.0f%%). Where u1 is clamped e1 '
        'averages %+.1f mm vs %+.1f mm unclamped, so the y correction was censored there (true y bias >= ~25 mm).'
        % (100 * cl.mean(), 100 * (D.u0.abs() >= M.CLAMP_M).mean(), 1e3 * D.e1[cl].mean(), 1e3 * D.e1[~cl].mean()))
    say('')


def sec_bias(c: M.Corpus, out: str) -> dict:
    D = c.D
    say('## 2. Per-site / per-sitting bias and apex ratio (inliers, %d of %d rows; mm)' % (len(D), len(c.all)))
    lat_bad = (c.all.e0.abs() >= M.OUTLIER_LAT_M) | (c.all.e1.abs() >= M.OUTLIER_LAT_M)
    rat_bad = (c.all.ratio <= M.RATIO_LO) | (c.all.ratio >= M.RATIO_HI)
    say('Dropped: %d rows with |e| > 100 mm on either axis (misses / bad mocap fits), then %d more with apex ratio '
        'outside (%.1f, %.1f).' % (int(lat_bad.sum()), int((rat_bad & ~lat_bad).sum()), M.RATIO_LO, M.RATIO_HI))
    say('')
    say('Per site (pooled over sittings):')
    rows, recs = [], []
    for s, g in D.groupby('site'):
        clampf = (g.u1 <= -M.CLAMP_M).mean()
        rows.append(('%+g' % s, ','.join(map(str, sorted(set(g.ball_id)))), len(g),
                     '%+.1f / %.1f' % (1e3 * g.e0.mean(), 1e3 * g.e0.std()),
                     '%+.1f / %.1f' % (1e3 * g.e1.mean(), 1e3 * g.e1.std()),
                     '%.4f / %.4f' % (g.ratio.mean(), g.ratio.std()),
                     '%+.1f' % (1e3 * g.u0.mean()), '%+.1f' % (1e3 * g.u1.mean()), '%.0f%%' % (100 * clampf)))
        recs.append(dict(site_x0=s, balls=rows[-1][1], n=len(g), e0_mean_mm=1e3 * g.e0.mean(), e0_sd_mm=1e3 * g.e0.std(),
                         e1_mean_mm=1e3 * g.e1.mean(), e1_sd_mm=1e3 * g.e1.std(), ratio_mean=g.ratio.mean(),
                         ratio_sd=g.ratio.std(), u0_mean_mm=1e3 * g.u0.mean(), u1_mean_mm=1e3 * g.u1.mean(),
                         u1_clamped_frac=clampf))
    md_table(['site x0', 'ball', 'n', 'e0 mean/sd', 'e1 mean/sd', 'apex ratio mean/sd', 'u0 mean', 'u1 mean',
              'frac u1 at -20 mm'], rows)
    pd.DataFrame(recs).to_csv(os.path.join(out, 'sites.csv'), index=False)
    say('')
    say('Per sitting (pooled over sites):')
    rows, recs = [], []
    for s, g in D.groupby('sit'):
        sd = lambda v: v.std() if len(v) > 1 else float('nan')  # noqa: E731
        rows.append((s, '%+.1f / %.1f' % (1e3 * g.e0.mean(), 1e3 * sd(g.e0)),
                     '%+.1f / %.1f' % (1e3 * g.e1.mean(), 1e3 * sd(g.e1)),
                     '%.4f / %.4f' % (g.ratio.mean(), sd(g.ratio)), len(g)))
        recs.append(dict(sit=s, n=len(g), e0_mean_mm=1e3 * g.e0.mean(), e0_sd_mm=1e3 * sd(g.e0),
                         e1_mean_mm=1e3 * g.e1.mean(), e1_sd_mm=1e3 * sd(g.e1), ratio_mean=g.ratio.mean(),
                         ratio_sd=sd(g.ratio)))
    md_table(['sit', 'e0 mean/sd', 'e1 mean/sd', 'ratio mean/sd', 'n'], rows)
    pd.DataFrame(recs).to_csv(os.path.join(out, 'sittings_bias.csv'), index=False)
    say('')
    cell = D.site.astype(str) + '_' + D.sit.astype(str)
    res = {}
    say('Variance decomposition (R2 of factor on the quantity; noise = sd about site x sitting cell means):')
    rr = []
    for nm, v, k in [('e0 [mm]', D.e0 * 1e3, 'e0'), ('e1 [mm]', D.e1 * 1e3, 'e1'), ('ratio', D.ratio, 'ratio')]:
        cm = v - v.groupby(cell).transform('mean')
        res['noise_' + k] = float(cm.std())
        rr.append((nm, '%.3f' % v.std(), '%.3f' % r2_of(v, D.site), '%.3f' % r2_of(v, D.sit),
                   '%.3f' % r2_of(v, cell), '%.3f' % cm.std()))
    md_table(['quantity', 'total sd', 'R2 site', 'R2 sitting', 'R2 site x sitting (%d cells)' % cell.nunique(),
              'cell-demeaned sd'], rr)
    say('')
    return res


def apex_fits(c: M.Corpus, out: str, bi: dict) -> dict:
    D = c.D
    y, u = D.y2_apex_m.values, D.u2_apex_m.values
    res = {}
    say('## 3. Apex-gain fit')
    recs = []
    K = float(u @ y / (u @ u))
    r = y - K * u
    seK = float(np.sqrt(r.var() / (u @ u)))
    recs.append(dict(model='y2 = K u2 (through origin)', a=0.0, K=K, K_se=seK, resid_sd_mm=1e3 * r.std()))
    X = np.column_stack([np.ones(len(u)), u])
    b, se, rr = M.clse(X, y, D.sit.values)
    recs.append(dict(model='y2 = a + b u2 (affine)', a=b[0], K=b[1], K_se=se[1], resid_sd_mm=1e3 * rr.std()))
    off = float((y - u).mean())
    recs.append(dict(model='y2 = u2 + c (offset only)', a=off, K=1.0, K_se=float('nan'), resid_sd_mm=1e3 * (y - u).std()))
    pd.DataFrame(recs).to_csv(os.path.join(out, 'apex_fit.csv'), index=False)
    res['K'], res['K_se'], res['k'] = K, seK, float(np.sqrt(K))
    res['affine_a'], res['affine_b'] = b[0], b[1]
    say('')
    md_table(['model', 'intercept a [m]', 'gain', 'gain SE (origin: naive; affine: cluster)', 'resid sd [mm]'],
             [(r_['model'], '%+.4f' % r_['a'], '%.4f' % r_['K'], '%.4f' % r_['K_se'], '%.1f' % r_['resid_sd_mm'])
              for r_ in recs])
    say('')
    say('Single global gain: y2 = %.4f u2  (K = k^2, k = %.4f), residual %.1f mm. Per-site mean ratio spans '
        '%.4f-%.4f; R2 of site on ratio = %.2f.' % (K, np.sqrt(K), 1e3 * r.std(), D.groupby('site').ratio.mean().min(),
                                                     D.groupby('site').ratio.mean().max(), r2_of(D.ratio, D.site)))
    say('Gain vs offset: affine fit gives a = %+.3f m, gain %.3f; the offset-only model (+%.1f mm) has residual '
        '%.1f vs %.1f mm. Apex range is only %.3f-%.3f m, so the two are nearly degenerate; gain is favoured, not proven.'
        % (b[0], b[1], 1e3 * off, 1e3 * (y - u).std(), 1e3 * r.std(), u.min(), u.max()))
    cell = D.site.astype(str) + '_' + D.sit.astype(str)
    g = D.groupby(cell)
    du = D.u2_apex_m - g.u2_apex_m.transform('mean')
    dy = D.y2_apex_m - g.y2_apex_m.transform('mean')
    m = (g.u2_apex_m.transform('count') > 3).values
    bb, sb, _ = M.clse(du.values[m][:, None], dy.values[m], D.sit.values[m])
    res['within_slope'] = float(bb[0])
    say('Within-cell slope dy2/du2 = %.3f +- %.3f (cluster SE; gain model ~%.2f, offset model 1.0; within-cell u2 sd %.1f mm).'
        % (bb[0], sb[0], K, 1e3 * du[m].std()))
    bins = [(0, .83), (.83, .85), (.85, .87), (.87, .88), (.88, 2)]
    say('Ratio by commanded-apex bin: ' + ', '.join(
        '%.3f-%s: %.4f (n=%d)' % (lo, ('%.2f' % hi if hi < 2 else 'up'), D.ratio[(D.u2_apex_m >= lo) & (D.u2_apex_m < hi)].mean(),
                                  int(((D.u2_apex_m >= lo) & (D.u2_apex_m < hi)).sum())) for lo, hi in bins))
    t = ((D.t_abs_s - D.t_abs_s.mean()) / 86400.0).values
    Xt = np.column_stack([np.ones(len(t)), t])
    d_ratio = M.clse(Xt, D.ratio.values, D.sit.values)
    res['ratio_drift_day'] = float(d_ratio[0][1])
    say('Drift: ratio %+.5f/day (cluster SE %.5f) -> %+.4f over the %.0f-day span.'
        % (d_ratio[0][1], d_ratio[1][1], d_ratio[0][1] * np.ptp(t), np.ptp(t)))
    for k in ('e0', 'e1'):
        bd, sd_, _ = M.clse(Xt, D[k].values * 1e3, D.sit.values)
        res[k + '_drift_day'] = float(bd[1])
        say('Drift: %s %+.2f mm/day (cluster SE %.2f).' % (k, bd[1], sd_[1]))
    say('Plane-move check (sitting groups from PLANE_MOVE_GROUPS; a plane-height offset would show as a step in the '
        'middle group), mean y2-u2 and ratio:')
    for nm, mk in [(nm_, (D.sit >= lo_) & (D.sit <= hi_)) for nm_, lo_, hi_ in PLANE_MOVE_GROUPS]:
        say('  %-9s n=%3d  y2-u2 %.1f mm  ratio %.4f' % (nm, int(mk.sum()), 1e3 * (D.y2_apex_m - D.u2_apex_m)[mk].mean(),
                                                         D.ratio[mk].mean()))
    # joint (K, theta_x, theta_y) weighted LS
    sa = float(np.std(y - K * u))
    s0, s1 = bi['noise_e0'] / 1e3, bi['noise_e1'] / 1e3      # lateral weights = computed cell-demeaned sd [m]
    abar = float(u.mean())
    Xj = np.vstack([np.column_stack([u, 0 * u, 0 * u]) / sa,
                    np.column_stack([D.u0.values, 4 * u, 0 * u]) / s0,
                    np.column_stack([D.u1.values, 0 * u, 4 * u]) / s1])
    yj = np.r_[y / sa, D.y0.values / s0, D.y1.values / s1]
    bj, sj, _ = M.clse(Xj, yj, np.r_[D.sit.values, D.sit.values, D.sit.values])
    res['joint'] = (bj, sj)
    say('Joint 3-parameter global fit (y2 = K u2; y_i = K u_i + 4 u2 theta_i): K = %.4f +- %.4f (k = %.4f), '
        'theta_x = %.2f +- %.2f mrad, theta_y = %.2f +- %.2f mrad (=> %.1f / %.1f mm at the corpus mean apex a = %.3f m; '
        'lateral weights = cell-demeaned sd %.1f / %.1f mm).'
        % (bj[0], sj[0], np.sqrt(bj[0]), 1e3 * bj[1], 1e3 * sj[1], 1e3 * bj[2], 1e3 * sj[2],
           4 * abar * 1e3 * bj[1], 4 * abar * 1e3 * bj[2], abar, bi['noise_e0'], bi['noise_e1']))
    say('')
    return res


def lateral_fits(c: M.Corpus, out: str) -> dict:
    D = c.D
    n = len(D)
    res = {}
    say('## 4. Lateral structure')
    recs = []
    for i in (0, 1):
        b, se, _ = M.clse(np.ones((n, 1)), D['e%d' % i].values * 1e3, D.sit.values)
        res['off%d' % i], res['off%d_se' % i] = float(b[0]), float(se[0])
    say('Pooled offset (M1): x %+.2f mm (cluster SE %.1f), y %+.2f mm (SE %.1f); same sign at every site except '
        'x0 > 0 in axis 0.' % (res['off0'], res['off0_se'], res['off1'], res['off1_se']))
    say('')
    say('Site-linear offset G1: e_i = c + m x0 (c in mm, slope m dimensionless; FE = sitting fixed effects).  "ball-0" = rows with ball_id 0 only.')
    rows = []
    B0 = D[D.ball_id == 0]
    for nm, df, fe in [('all rows', D, False), ('all rows, sitting FE', D, True), ('ball-0', B0, False),
                       ('ball-0, sitting FE', B0, True), ('ball-0, sittings 0-4, FE', B0[B0.sit <= 4], True),
                       ('ball-0, sittings 5+, FE', B0[B0.sit >= 5], True)]:
        row = [nm, len(df)]
        for i in (0, 1):
            e = df['e%d' % i].values * 1e3
            X = (np.column_stack([M.dummies(df.sit), df.x0.values]) if fe
                 else np.column_stack([np.ones(len(df)), df.x0.values]))
            b, se, _ = M.clse(X, e, df.sit.values)
            row += ['%.1f' % b[0] if not fe else '(FE)', '%+.3f (%.3f)' % (b[-1] / 1e3, se[-1] / 1e3)]
            res['slope_%s_%d' % (nm, i)] = float(b[-1]) / 1e3      # mm per m -> m per m (dimensionless)
            res['slope_se_%s_%d' % (nm, i)] = float(se[-1]) / 1e3
            recs.append(dict(fit=nm, axis=i, n=len(df), x0_slope_m_per_m=b[-1] / 1e3, se=se[-1] / 1e3))
        rows.append(row)
    md_table(['rows', 'n', 'ax0 const [mm]', 'ax0 slope [m/m] (SE)', 'ax1 const [mm]', 'ax1 slope [m/m] (SE)'], rows)
    # the headline "x0 slope" is the ball-0 axis-0 slope; the pooled one is bigger and ball-confounded
    res['slope_b0_ax0'] = res['slope_ball-0, sitting FE_0']
    res['slope_all_ax0'] = res['slope_all rows_0']
    sl0, sl1 = res['slope_all_ax0'], res['slope_ball-0, sitting FE_1']
    se1 = res['slope_se_ball-0, sitting FE_1']
    se_ = res['slope_se_ball-0, sitting FE_0']
    e_, l_ = res['slope_ball-0, sittings 0-4, FE_0'], res['slope_ball-0, sittings 5+, FE_0']
    say('Axis-0 slope %+.3f (all rows): %s. Ball-0 sitting-FE slope %+.3f (SE %.3f); early sittings %+.3f vs later %+.3f '
        '(%s across periods). Axis-1 ball-0 slope %+.3f (SE %.3f): %s (x1 == 0 for ALL rows, so y-axis SITE dependence is '
        'unidentifiable).'
        % (sl0, 'negative: landing pulled toward the platform centre' if sl0 < 0 else
           'not a centre pull', res['slope_b0_ax0'], se_, e_, l_,
           'differ by more than 2 SE' if abs(e_ - l_) > 2 * se_ else 'stable within 2 SE', sl1, se1,
           'distinguishable from zero' if abs(sl1) > 2 * se1 else 'not distinguishable from zero'))
    say('')
    say('Ball contrast (ball_id 1 is thrown ONLY at site x0 = -0.0625: site and ball are perfectly confounded):')
    print_ct = pd.crosstab(D.site, D.ball_id)
    say('  rows by site x ball: ' + '; '.join('%+g: ball0=%d ball1=%d' % (s, r_.get(0, 0), r_.get(1, 0))
                                             for s, r_ in print_ct.iterrows()))
    for i in (0, 1):
        v = []
        for s, g in D.groupby('sit'):
            a_, b_ = g[g.ball_id == 1]['e%d' % i], g[g.ball_id == 0]['e%d' % i]
            if len(a_) >= 5 and len(b_) >= 5:
                v.append(1e3 * (a_.mean() - b_.mean()))
        v = np.array(v)
        if len(v) == 0:
            res['ball%d' % i], res['ball%d_n' % i] = float('nan'), 0
            say('  axis %d: no sitting with both balls (>= 5 rows each): within-sitting ball contrast skipped.' % i)
        else:
            res['ball%d' % i], res['ball%d_n' % i] = float(v.mean()), len(v)
            say('  axis %d: within-sitting ball1 - ball0 = %+.1f mm (SE %.1f, %d sittings, range %+.0f..%+.0f).'
                % (i, v.mean(), v.std(ddof=1) / np.sqrt(len(v)) if len(v) > 1 else float('nan'), len(v),
                   v.min(), v.max()))
        if D.ball_id.nunique() < 2:
            say('  axis %d: ball_id is constant in the inlier set: sitting-FE ball fit skipped (singular design).' % i)
            continue
        X = np.column_stack([M.dummies(D.sit), D.ball_id.values])
        bf, sf, _ = M.clse(X, D['e%d' % i].values * 1e3, D.sit.values)
        res['ball%d_fe' % i], res['ball%d_fe_se' % i] = float(bf[-1]), float(sf[-1])
        say('  axis %d: with sitting FE %+.1f mm (cluster SE %.1f).' % (i, bf[-1], sf[-1]))
    rs = D.ratio - D.ratio.groupby(D.sit).transform('mean')
    if D.ball_id.nunique() > 1:
        say('  Apex-ratio difference between balls (sitting-demeaned): %.4f (ratio sd %.4f).'
            % (rs[D.ball_id == 1].mean() - rs[D.ball_id == 0].mean(), D.ratio.std()))
    say('')
    say('Aim-angle vs geometric offset: e_i = c + s (a - abar), a = apex; aim model predicts s = c/abar (~12-17 mm/m), '
        'offset model s = 0.')
    rows, ses, preds = [], [], []
    for ax in (0, 1):
        for nm, cn in [('commanded', 'u2_apex_m'), ('observed', 'y2_apex_m')]:
            e = D['e%d' % ax].values * 1e3
            a = D[cn].values
            bb, se, _ = M.clse(np.column_stack([np.ones(n), a - a.mean()]), e, D.sit.values)
            ses.append(se[1])
            preds.append(abs(bb[0] / a.mean()))
            rows.append((ax, nm, '%.2f (%.2f)' % (bb[0], se[0]), '%+.1f (%.1f)' % (bb[1], se[1]), '%+.1f' % (bb[0] / a.mean())))
    md_table(['axis', 'apex regressor', 'c [mm]', 's [mm/m] (SE)', 'aim-model s = c/abar'], rows)
    nopow = all(sv > 3 * pv for sv, pv in zip(ses, preds))
    say('The apex range is %.3f-%.3f m (sd %.0f mm); s has SE %.0f-%.0f mm/m against an aim-model prediction of |s| '
        '%.0f-%.0f mm/m: %s' % (D.u2_apex_m.min(), D.u2_apex_m.max(), 1e3 * D.u2_apex_m.std(), min(ses), max(ses),
                               min(preds), max(preds),
                               'the apex-slope SE exceeds 3x the aim-model prediction on every row, so the test has no '
                               'power and the data cannot separate aim angle from a geometric offset.' if nopow else
                               'the SE is within 3x the prediction on at least one row, so the test is resolvable there; '
                               'compare s with 0 and with c/abar.'))
    say('')
    # closed-loop warning: pooled vs within-cell slope of y on u
    cell = D.site.astype(str) + '_' + D.sit.astype(str)
    say('Closed-loop caution (u is the learner output): slope of y on u')
    rows = []
    for i in (0, 1):
        u, y = D['u%d' % i], D['y%d' % i]
        bp, sp, _ = M.clse(np.column_stack([u.values, np.ones(n)]), y.values, D.sit.values)
        du, dy = u - u.groupby(cell).transform('mean'), y - y.groupby(cell).transform('mean')
        bw, sw, _ = M.clse(du.values[:, None], dy.values, D.sit.values)
        rows.append((i, '%.2f (%.2f)' % (bp[0], sp[0]), '%.2f (%.2f)' % (bw[0], sw[0]), '%.1f' % (1e3 * du.std()),
                     '%.1f' % (1e3 * dy.std())))
        res['slope_u_within_%d' % i] = float(bw[0])
        res['slope_u_within_se_%d' % i] = float(sw[0])
        res['slope_u_pooled_%d' % i] = float(bp[0])
    md_table(['axis', 'pooled slope (SE)', 'within-cell slope (SE)', 'within-cell u sd [mm]', 'within-cell y sd [mm]'], rows)
    say('Pooled slopes (%.2f / %.2f) are not plant gains (u is the learner output, closed loop). Within-cell u sd is '
        '%.1f / %.1f mm against y sd %.1f / %.1f mm; the within-cell slope %s 1 on axis 0 and %s 1 on axis 1 (2 SE).'
        % (res['slope_u_pooled_0'], res['slope_u_pooled_1'], float(rows[0][3]), float(rows[1][3]),
           float(rows[0][4]), float(rows[1][4]),
           'excludes' if abs(res['slope_u_within_0'] - 1) > 2 * res['slope_u_within_se_0'] else 'includes',
           'excludes' if abs(res['slope_u_within_1'] - 1) > 2 * res['slope_u_within_se_1'] else 'includes'))
    say('')
    pd.DataFrame(recs).to_csv(os.path.join(out, 'x0_slopes.csv'), index=False)
    return res


def sec_r2(c: M.Corpus, out: str) -> dict:
    D = c.D
    n = len(D)
    t = ((D.t_abs_s - D.t_abs_s.mean()) / 86400.0).values
    say('## 5. How much does a small global model capture? (in-sample R2 about the pooled mean)')
    recs, rows = [], []
    res = {}
    fams = [('pooled const', np.ones((n, 1))),
            ('const + x0-linear', np.column_stack([np.ones(n), D.x0.values])),
            ('const + ball + x0-linear', np.column_stack([np.ones(n), D.ball_id.values, D.x0.values])),
            ('+ linear time drift', np.column_stack([np.ones(n), D.ball_id.values, D.x0.values, t])),
            ('6 per-site constants', M.dummies(D.site)),
            ('17 sitting means', M.dummies(D.sit)),
            ('site + sitting additive', np.column_stack([M.dummies(D.site), M.dummies(D.sit)[:, 1:]])),
            ('site x sitting cells', M.dummies(D.site.astype(str) + '_' + D.sit.astype(str)))]
    for nm, X in fams:
        row = [nm, np.linalg.matrix_rank(X)]
        for i in (0, 1):
            e = D['e%d' % i].values * 1e3
            sst = ((e - e.mean()) ** 2).sum()
            b = np.linalg.lstsq(X, e, rcond=None)[0]
            r = e - X @ b
            k = np.linalg.matrix_rank(X)
            r2 = 1 - (r ** 2).sum() / sst
            adj = 1 - ((r ** 2).sum() / (n - k)) / (sst / (n - 1))
            row.append('%.3f / %.3f / %.2f' % (r2, adj, np.sqrt((r ** 2).sum() / (n - k))))
            recs.append(dict(model=nm, k=k, axis=i, r2=r2, adj_r2=adj, resid_sd_mm=np.sqrt((r ** 2).sum() / (n - k))))
            res[(nm, i)] = r2
        rows.append(row)
    md_table(['model', 'params', 'ax0 R2 / adj / resid sd [mm]', 'ax1 R2 / adj / resid sd [mm]'], rows)
    pd.DataFrame(recs).to_csv(os.path.join(out, 'r2_ladder.csv'), index=False)
    gl = ['const + ball + x0-linear', '+ linear time drift']
    say('In-sample R2 (ax0 / ax1): %s global %s vs 6 site constants %.3f / %.3f vs site x sitting cells %.3f / %.3f; the '
        'global model captures %.0f%% / %.0f%% of the 6-site-constant R2 with 3-4 parameters. Whether cells can be '
        'predicted ahead of time is the out-of-sample section.'
        % ('%d-%d parameter' % (min(np.linalg.matrix_rank(dict(fams)[g]) for g in gl),
                                max(np.linalg.matrix_rank(dict(fams)[g]) for g in gl)),
           ' / '.join('%.3f-%.3f' % (min(res[(g, i)] for g in gl), max(res[(g, i)] for g in gl)) for i in (0, 1)),
           res[('6 per-site constants', 0)], res[('6 per-site constants', 1)],
           res[('site x sitting cells', 0)], res[('site x sitting cells', 1)],
           100 * min(res[(g, 0)] for g in gl) / res[('6 per-site constants', 0)],
           100 * min(res[(g, 1)] for g in gl) / res[('6 per-site constants', 1)]))
    say('')
    return res


def oos(c: M.Corpus, out: str) -> dict:
    D = c.D
    names = list(M.SPEC)
    sites = sorted(D.site.unique())
    res = {}
    say('## 6. Global-model fits: out-of-sample RMSE [mm], axis 0 / axis 1')
    say('Models (per axis): M0 yhat = u; M1 pooled constant; M2 own-axis gain + b; M3 A u + A2 u_other + B x0 + b; M3x adds '
        'carried x2,x3; G1 const + x0-linear; P1..P4 aim-angle variants (landing offset 4 a theta, apex a). ORACLE rows '
        'fit the held-out data itself (an unreachable floor).')
    say('')
    # ---------- LOSO by site
    say('Leave-one-site-out (train on 5 sites, test the 6th; pooled over folds):')
    L, rows, recs = {}, [], []
    for i in (0, 1):
        for nm in names:
            P = np.full(len(D), np.nan)
            for s in sites:
                tr, te = D[D.site != s], (D.site == s).values
                P[te] = c.predict(D[te], i, nm, c.fit(tr, i, nm))
            L[(nm, i)] = P
        P = np.full(len(D), np.nan)
        for s in sites:
            te = (D.site == s).values
            P[te] = D.loc[te, 'u%d' % i] + (D.loc[te, 'y%d' % i] - D.loc[te, 'u%d' % i]).mean()
        L[('ORACLE per-site const (fit on the held-out site)', i)] = P
    for nm in [k[0] for k in L if k[1] == 0]:
        row = [nm]
        rec = dict(model=nm)
        for i in (0, 1):
            v = M.rmse_mm(D, i, L[(nm, i)])
            row.append('%.1f' % v)
            rec['ax%d' % i] = v
            res[('loso', nm, i)] = v
        worst = max(np.sqrt(np.mean([(1e3 * (D.loc[(D.site == s).values, 'y%d' % i].values
                                             - L[(nm, i)][(D.site == s).values])) ** 2 for i in (0, 1)])) for s in sites)
        for s in sites:
            m_ = (D.site == s).values
            rec['site%+g' % s] = np.sqrt(np.mean([(1e3 * (D.loc[m_, 'y%d' % i].values - L[(nm, i)][m_])) ** 2
                                                 for i in (0, 1)]))
        rec['worst_site_rms'] = worst
        rows.append(row)
        recs.append(rec)
    md_table(['model', 'ax0', 'ax1'], rows)
    pd.DataFrame(recs).to_csv(os.path.join(out, 'loso.csv'), index=False)
    ws = {s: np.sqrt(np.mean([(1e3 * (D.loc[(D.site == s).values, 'y%d' % i].values
                                      - L[('M1 pooled const', i)][(D.site == s).values])) ** 2 for i in (0, 1)]))
          for s in sites}
    worst_s = max(ws, key=ws.get)
    say('Per held-out site (M1 pooled const, rms of both axes): ' + ', '.join('%+g: %.1f' % (s, v) for s, v in ws.items())
        + ' -- worst is %+g%s.' % (worst_s, ' (a site where ball_id 1 is thrown)'
                                  if worst_s in set(D[D.ball_id == 1].site) else ''))
    say('')
    # ---------- chronological split(s)
    recs = []
    for cut in (CHRONO_CUT, SPLIT_B):
        tr, te = D[D.sit <= cut], D[D.sit > cut]
        say('Chronological: train sittings <= %d (n=%d), test sittings > %d (n=%d):' % (cut, len(tr), cut, len(te)))
        rows = []
        for nm in names + ['M1s per-site const (train period; pooled fallback)',
                           'ORACLE per-site const on test period', 'ORACLE per-site-per-sitting const (in-sample)']:
            row, rec = [nm], dict(model=nm, cut=cut)
            for i in (0, 1):
                if nm in M.SPEC:
                    p = c.predict(te, i, nm, c.fit(tr, i, nm))
                elif nm.startswith('M1s'):
                    e = tr['y%d' % i] - tr['u%d' % i]
                    p = te['u%d' % i].values + te.site.map(e.groupby(tr.site).mean()).fillna(e.mean()).values
                else:
                    e = te['y%d' % i] - te['u%d' % i]
                    keys = [te.site] if 'per-site const' in nm and 'sitting' not in nm else [te.site, te.sit]
                    p = te['u%d' % i].values + e.groupby(keys).transform('mean').values
                v = M.rmse_mm(te, i, p)
                row.append('%.2f' % v)
                rec['ax%d' % i] = v
                res[('chrono%d' % cut, nm, i)] = v
            rows.append(row)
            recs.append(rec)
        md_table(['model', 'ax0', 'ax1'], rows)
        say('')
    pd.DataFrame(recs).to_csv(os.path.join(out, 'chrono.csv'), index=False)
    # ---------- forward chaining
    say('Forward chaining (each sitting >= %d predicted from all earlier sittings), pooled RMSE:' % FORWARD_FIRST)
    fnames = names + ['M1s per-site const']
    sq, rows, recs = {}, [], []
    for nm in fnames:
        row, rec = [nm], dict(model=nm)
        for i in (0, 1):
            errs = []
            for s in range(FORWARD_FIRST, int(D.sit.max()) + 1):
                tr, te = D[D.sit < s], D[D.sit == s]
                if len(te) == 0:
                    continue
                if nm.startswith('M1s'):
                    e = tr['y%d' % i] - tr['u%d' % i]
                    p = te['u%d' % i].values + te.site.map(e.groupby(tr.site).mean()).fillna(e.mean()).values
                else:
                    p = c.predict(te, i, nm, c.fit(tr, i, nm))
                errs.append(te['y%d' % i].values - p)
            e = np.concatenate(errs)
            sq[(nm, i)] = e
            rec['ax%d' % i] = 1e3 * np.sqrt(np.mean(e ** 2))
            row.append('%.2f' % rec['ax%d' % i])
            res[('fwd', nm, i)] = rec['ax%d' % i]
        rows.append(row)
        recs.append(rec)
    md_table(['model', 'ax0', 'ax1'], rows)
    pd.DataFrame(recs).to_csv(os.path.join(out, 'forward_chain.csv'), index=False)
    say('')
    say('Paired forward-chain squared-error differences [mm^2] (naive SE; they ignore within-sitting dependence):')
    for a, b in [('M1 pooled const', 'M0 identity'), ('M1s per-site const', 'M1 pooled const'),
                 ('M3 affine A u + A2 u_other + B x0 + b', 'M1 pooled const'),
                 ('P3 const+aim-slope: u+c+4*theta*(a-abar)', 'M1 pooled const')]:
        parts = []
        for i in (0, 1):
            dd = (sq[(a, i)] ** 2 - sq[(b, i)] ** 2) * 1e6
            parts.append('ax%d %+.0f +- %.0f' % (i, dd.mean(), dd.std() / np.sqrt(len(dd))))
        say('  %s - %s: %s' % (a.split(' ')[0], b.split(' ')[0], '; '.join(parts)))
    gains = [float(c.fit(D, i, 'M2 own-axis gain+b')[0]) for i in (0, 1)]
    say('M2 own-axis gain A (full sample): %.2f / %.2f (ax0 / ax1). 2026-10-18 corpus note: an A below 1 here is '
        'closed-loop shrinkage (u anti-correlates with cell bias), not a plant property; it is policy-conditional and '
        'would change if the learner law changes.' % tuple(gains))
    say('')
    return res


def figures(c: M.Corpus, out: str, K: float, off_mm: float) -> None:
    D = c.D
    cm = plt.get_cmap('viridis')
    smax = max(int(D.sit.max()), 1)
    sites = sorted(D.site.unique())
    xp = {s: k for k, s in enumerate(sites)}
    fig, ax = plt.subplots(1, 2, figsize=(12, 4.6), sharey=True)
    rng = np.random.RandomState(0)
    for i in (0, 1):
        for s, g in D.groupby('sit'):
            jit = rng.uniform(-.18, .18, len(g))
            ax[i].scatter(g.site.map(xp).values + jit, 1e3 * g['e%d' % i], c=[cm(s / smax)] * len(g), s=9, alpha=.6, lw=0)
        mm = D.groupby('site')['e%d' % i].mean() * 1e3
        ax[i].plot([xp[s] for s in mm.index], mm.values, 'k_', ms=26, mew=2.5, label='site mean')
        ax[i].axhline(0, c='grey', lw=.7)
        ax[i].set_xticks(range(len(sites)))
        ax[i].set_xticklabels(['%+.4g' % s for s in sites])
        ax[i].set_xlabel('release site x0 [m] (x1 = 0 for all rows)')
        ax[i].set_title('lateral residual y%d - u%d  (inlier rows)' % (i, i))
        ax[i].set_ylim(-80, 100)
    ax[0].set_ylabel('y - u [mm]')
    ax[0].legend()
    sm = plt.cm.ScalarMappable(cmap=cm, norm=plt.Normalize(0, smax))
    fig.colorbar(sm, ax=ax, label='sitting index (0 = first, %d = last)' % smax)
    fig.savefig(os.path.join(out, 'fig_resid_vs_site.png'), dpi=130, bbox_inches='tight')
    plt.close(fig)
    fig, ax = plt.subplots(1, 2, figsize=(12, 4.6))
    for s in sites:
        g = D[D.site == s]
        ax[0].scatter(g.u2_apex_m, g.ratio, s=8, alpha=.45, label='%+.4g (n=%d)' % (s, len(g)), lw=0)
    ax[0].axhline(D.ratio.mean(), c='k', lw=.8)
    ax[0].set_xlabel('commanded apex u2 [m]')
    ax[0].set_ylabel('y2/u2')
    ax[0].legend(fontsize=7, markerscale=2)
    ax[0].set_title('apex ratio vs commanded apex, by site')
    for s, g in D.groupby('sit'):
        ax[1].scatter(g.u2_apex_m, 1e3 * (g.y2_apex_m - g.u2_apex_m), c=[cm(s / smax)] * len(g), s=8, alpha=.6, lw=0)
    uu = np.linspace(D.u2_apex_m.min(), D.u2_apex_m.max(), 10)
    ax[1].plot(uu, 1e3 * (K - 1) * uu, 'r-', label='gain model (K = %.3f): y - u = %.3f u' % (K, K - 1))
    ax[1].axhline(off_mm, c='b', ls='--', label='offset model: %+.1f mm' % off_mm)
    ax[1].legend(fontsize=7)
    ax[1].set_xlabel('commanded apex u2 [m]')
    ax[1].set_ylabel('y2 - u2 [mm]')
    ax[1].set_title('apex excess vs commanded (colour = sitting)')
    fig.savefig(os.path.join(out, 'fig_apex_ratio.png'), dpi=130, bbox_inches='tight')
    plt.close(fig)


# reference values from the 2026-10-18 run: (label, getter, expected, tolerance)
def reproduction_check(ap, lat, bi) -> list:
    chk = [
        ('apex gain K = k^2', ap['K'], 1.085, 0.004),
        ('pooled lateral offset x [mm]', lat['off0'], 9.9, 1.0),
        ('pooled lateral offset y [mm]', lat['off1'], 15.0, 1.0),
        ('ball1 - ball0 contrast, x [mm, within-sitting]', lat['ball0'], 15.2, 1.5),
        ('ball1 - ball0 contrast, y [mm, within-sitting]', lat['ball1'], 18.1, 1.5),
        ('x0 slope axis 0, ball-0 rows, sitting FE [m/m]', lat['slope_b0_ax0'], -0.058, 0.03),
        ('cell-demeaned shot noise e0 [mm]', bi['noise_e0'], 20.0, 1.5),
        ('cell-demeaned shot noise e1 [mm]', bi['noise_e1'], 20.0, 1.5),
    ]
    return chk


def main() -> int:
    ap_ = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap_.add_argument('--csv', default=DEFAULT_CSV, help='memory corpus (default %(default)s, relative to the repo root)')
    ap_.add_argument('--out', default=DEFAULT_OUT, help='output directory (default %(default)s)')
    args = ap_.parse_args()
    csv = args.csv if os.path.isabs(args.csv) else os.path.join(ROOT, args.csv)
    out = args.out if os.path.isabs(args.out) else os.path.join(ROOT, args.out)
    if not os.path.isfile(csv):
        print('memory corpus not found: %s' % csv, file=sys.stderr)
        return 2
    os.makedirs(out, exist_ok=True)
    c = M.Corpus(M.load(csv))
    a = c.all
    if len(a) == 0 or len(c.D) == 0:
        print('empty analysis set (%d rows, %d inliers) in %s: nothing to regress' % (len(a), len(c.D), csv),
              file=sys.stderr)
        return 2
    say('# Plant-error structure in memory.csv: global model vs site-idiosyncratic')
    say('')
    say('Data: %s (%d rows, %s .. %s UTC, %d sittings). Inliers: %d rows. e_i = y_i - u_i (axis 0 = x, 1 = y); '
        'apex ratio r = y2/u2. Cluster SEs are by sitting (%d clusters).'
        % (os.path.relpath(csv, ROOT), len(a), a.date.min().strftime('%Y-%m-%d'), a.date.max().strftime('%Y-%m-%d'),
           a.sit.nunique(), len(c.D), c.D.sit.nunique()))
    say('')
    sec_sittings(c, out)
    bi = sec_bias(c, out)
    ap = apex_fits(c, out, bi)
    lat = lateral_fits(c, out)
    sec_r2(c, out)
    oo = oos(c, out)
    figures(c, out, ap['K'], 1e3 * float((c.D.y2_apex_m - c.D.u2_apex_m).mean()))
    say('## 7. Reproduction check against the 2026-10-18 run')
    rows, nbad = [], 0
    for lab, val, exp, tol in reproduction_check(ap, lat, bi):
        ok = abs(val - exp) <= tol
        nbad += 0 if ok else 1
        rows.append((lab, '%.4g' % val, '%.4g +- %.2g' % (exp, tol), 'ok' if ok else 'CHECK'))
    md_table(['quantity', 'this run', '2026-10-18 reference', ''], rows)
    best, bn, bi_ = min((oo[('loso', n, i)], n, i) for n in M.SPEC for i in (0, 1) if n != 'M0 identity')
    say('')
    say('Best non-oracle LOSO RMSE on either axis: %.1f mm (%s, axis %d); cell-demeaned shot noise %.1f / %.1f mm '
        '(ax0 / ax1).' % (best, bn.split(' ')[0], bi_, bi['noise_e0'], bi['noise_e1']))
    say('Figures: fig_resid_vs_site.png (y-u per axis vs site, coloured by sitting, bars = site means); fig_apex_ratio.png '
        '(ratio vs commanded apex per site; y2-u2 vs u2 with the gain and offset curves).')
    say('')
    say('Reproduction: %s' % ('all %d checks within tolerance' % len(rows) if nbad == 0 else '%d CHECK(s) outside tolerance' % nbad))
    with open(os.path.join(out, 'report.md'), 'w') as fh:
        fh.write('\n'.join(R) + '\n')
    print('wrote %s' % out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
