#!/usr/bin/env python3
"""bb_residual_cap_evidence.py - evidence for/against a value of CONSTELLATION_MAX_RESIDUAL_MM.

What it measures
    How far the BB (Ball Butler) calibration's yaw offset kappa can be biased by a marker-template
    error that the template-residual cap does NOT catch, so the cap can be re-derived (rather than
    re-argued) after the marker template is rebuilt or the solver changes. Three stages (positional
    ``stage`` argument):
      decode : replay bags (``--bags``) through ``bb_calibration_replay.NodeSim`` and pickle each bag's
             rows and every sweep's raw solver inputs to ``<out>/<bagname>.pkl`` (the --pickles format).
             Minutes per bag. Unlike the 2026-10-16 decoder it does not keep every marker of every frame
             (``all_frames``): nothing here reads it and it was most of the 75 MB.
      prep : cut the solver inputs (frames, yaw samples, base heading) of the swept bags out of the
             decoded-sweep pickles into ``<out>/evening_inputs.pkl`` (a small scratch pickle that A
             and B read). Run it first, once per --out.
      A    : empirical kappa deviation vs template residual over every ACCEPTED sweep in --replay-dir
             plus every sweep in the pickles re-solved with the cap lifted (max_residual_mm=10) ->
             ``partA_rows.csv``, ``partA.txt``, ``partA_scatter.png``. Deviation = kappa minus the
             sitting reference (median kappa of that group's sweeps with residual <= 0.5 mm; see the
             reference rules below).
      B    : template-perturbation injection: re-solve the clean sweeps with each template marker
             displaced (tan/rad/z, +/-, 0.5..3 mm), the Q3/Q6 tangential pair, a 0.1 deg rotation and a
             0.75 % xy scale, and record the resulting yaw shift against the residual the solver
             reports -> ``partB_injections.csv``. Uses a 5-process Pool.
      C    : sensitivity tables, the worst-case slope (deg of yaw per mm of residual), plots and the
             cap table (worst-case vs observed vs sweeps refused at each cap) -> ``partC.txt``,
             ``partB_injection.png``. Reads A and B's CSVs from --out.

Inputs
    --replay-dir   a ``bb_calibration_replay.py`` output dir holding ``sweeps.csv`` (needs 24+ bags
                   to be useful; stage A uses the rows with verdict_intrinsic == ACCEPTED).
    --bags         (decode) calibration bag dirs, e.g. ``~/Desktop/rosbags/2026-10-16_20-30-57``.
    --pickles      one or more decoded-sweep pickles, produced by the ``decode`` stage:
                   ``P['bag']`` (bag name), ``P['rows']`` (list of replay-row dicts, each with
                   ``sweep_idx`` and ``verdict_p1``) and ``P['sweeps'][i]`` with keys ``frames``,
                   ``stamped``, ``yaw_samples``, ``base_points``, ``t_start``, ``t_end``,
                   ``row_index`` (index into ``P['rows']``).
    The marker template and base frame come from ``ros_ws/src/jugglebot/resources`` (the live ones).

DATA-SPECIFIC rules (kept explicit, edit or override for a new data set)
    * GROUP_RULES (below): replay rows are grouped by bag; pickle sweeps by the table. The 2026-10-16
      evening bags are split into E1 (20:31-20:34, constellation clean) and E2 (22:00-22:03, after the
      86 min BB-off deformed Q3-Q6).
    * REF_FALLBACK / ``--ref-group TARGET=SOURCE`` (repeatable): a group with no sweep at residual
      <= 0.5 mm borrows SOURCE's reference (E2 is referenced to E1, by construction). A daytime bag
      with no clean sweep borrows the nearest-in-time bag OF THE SAME DATE. Every row prints which one
      it used in the ``ref_src`` column ('own', 'E1 (no clean sweep)' / '--ref-group ...', 'nearest bag ...').
    * Stage B's sweep set (``--b-sweeps``, default: 20-30-57 sweeps 1-6 + 22-01-50 sweep 2 = the clean
      ones) and magnitudes (``--mags``).

Usage (repo root, project venv)
    python tools/probes/bb_residual_cap_evidence.py decode --bags BAGDIR... --out PKLDIR
    python tools/probes/bb_residual_cap_evidence.py prep --replay-dir R --pickles P1.pkl P2.pkl
    python tools/probes/bb_residual_cap_evidence.py A    --replay-dir R --pickles P1.pkl P2.pkl --out OUT
    python tools/probes/bb_residual_cap_evidence.py B    --out OUT [--b-sweeps BAG:1,2,3 ...] [--mags 1.0,3.0]
    python tools/probes/bb_residual_cap_evidence.py C    --out OUT
    ``--out`` defaults to ``temp/probes/bb_residual_cap_evidence/<timestamp>/``; pass the same dir to
    every stage of one run. Runtime: prep/A/C seconds; B ~6 min (1589 solves, 7 sweeps, 5 processes).

Result, 2026-10-17 (record: ``temp/probes/bb_calibration_replay/20261017-cap-evidence/``)
    A: 89 rows, |dev| vs residual slope -0.022 +/- 0.016 deg/mm, worst |dev| 0.160 deg (nothing grows with
    residual). B: worst single-marker slope 0.155 deg of yaw per mm of residual (Q3 z+), so a 1.5 mm cap
    bounds the undetected bias at ~0.23 deg worst-case; this underpinned raising the cap 1.0 -> 1.5 mm.

Provenance: logbook ``2026-10-17-bb-calibration-residual-cap-1p5``; plan
``plans/active/bb-calibration-one-attempt.md``.
"""
from __future__ import annotations

import argparse
import csv
import dataclasses
import datetime
import math
import os
import pickle
import sys
import time
from collections import defaultdict

import numpy as np

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
sys.path.insert(0, REPO)
import tools.probes.bb_calibration_replay as R  # noqa: E402
from jugglebot.bb_calibration import track_constellation, estimate_sweep_yaw_offset  # noqa: E402
from jugglebot.bb_base_frame import estimate_base_pose, wrap_deg  # noqa: E402

# --- DATA-SPECIFIC tables (2026-10-16 evening) -------------------------------------------------
# Pickle sweeps -> reference group: (bag, max sweep_idx) -> group; anything else in that bag -> default.
E1 = 'E1 10-16 20:31-20:34'
E2 = 'E2 10-16 22:00-22:03'
GROUP_RULES = [('2026-10-16_20-30-57', 6, E1)]
GROUP_DEFAULT = E2
# group -> group whose sitting reference it borrows (E2 deformed constellation has no clean sweep).
REF_FALLBACK = {E2: E1}
# Stage B default sweeps: the constellation-clean ones.
B_SWEEPS_DEFAULT = [('2026-10-16_20-30-57', i) for i in range(1, 7)] + [('2026-10-16_22-01-50', 2)]

SIMS = []


class Sim2(R.NodeSim):
    """NodeSim that keeps each sweep's raw solver inputs (ported from the 2026-10-16 probe_bb_phase2)."""

    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        self.sweeps = []
        SIMS.append(self)

    def _finalize(self, t_end):
        self.sweeps.append({
            't_start': self.t_start, 't_end': t_end,
            'frames': [(t, p.copy()) for t, p in self.frames],
            'calib_data': {i: [p.copy() for p in v] for i, v in self.calib_data.items()},
            'calib_times': {i: list(v) for i, v in self.calib_times.items()},
            'stamped': list(self.stamped), 'yaw_samples': list(self.yaw_samples),
            'yaw_readings': list(self.yaw_readings),
            'base_points': [(t, p.copy()) for t, p in self.base_points],
            'row_index': len(self.rows)})
        return super()._finalize(t_end)


def stage_decode(bags):
    R.NodeSim = Sim2
    tpl = template()
    base_model = R.load_base_frame(os.path.join(R.RES_DIR, 'bb_base_frame.json'))
    gate = R.GateState(base_model)
    rargs = argparse.Namespace(bb_yaw_source='auto', clip=None, clip_yaw_deg=None)
    for bd in bags:
        name = os.path.basename(os.path.normpath(bd))
        t0 = time.time()
        SIMS.clear()
        rows, _ = R.replay_bag(bd, rargs, tpl, base_model, gate, False)
        sim = SIMS[-1] if SIMS else None
        with open(os.path.join(OUT, name + '.pkl'), 'wb') as fh:
            pickle.dump({'bag': name, 'rows': rows, 'sweeps': sim.sweeps if sim else []}, fh, protocol=4)
        print(f'{name}: {len(rows)} sweeps, {time.time() - t0:.0f} s', flush=True)


A = argparse.Namespace()      # parsed args, set in main(); inherited by Pool workers (fork)
OUT = SMALL = None


def lt(t):
    return datetime.datetime.fromtimestamp(t).strftime('%H:%M:%S')


def template():
    return R.load_marker_template(os.path.join(R.RES_DIR, 'bb_marker_template.json'))


def yaw_src(sw):
    return (sw['stamped'], 'stamped') if len(sw['stamped']) >= R.MIN_STAMPED_YAW_SAMPLES else (sw['yaw_samples'], 'heartbeat')


def solve(frames, yaw, src, tpl):
    """cap-lifted solve -> (est, track) or refusal class."""
    try:
        trk = track_constellation(frames, tpl)
        est = estimate_sweep_yaw_offset(frames, yaw, tpl, yaw_source=src, track=trk, max_residual_mm=10.0)
    except ValueError as e:
        return R.refusal_class(str(e)), None
    return est, trk


def worst_pair(trk, est, yaw):
    mov = R.moving_mask(trk, yaw, est.lag_s)
    p = np.abs(trk.pair_residual_pose_binned_mm(mov))
    a, b = np.unravel_index(np.nanargmax(np.where(np.isfinite(p), p, -1)), p.shape)
    return f'Q{a+1}-Q{b+1}'


# ------------------------------------------------------------------ prep
def stage_prep():
    base_model = R.load_base_frame(os.path.join(R.RES_DIR, 'bb_base_frame.json'))
    out = []
    for pk in A.pickles:
        P = pickle.load(open(pk, 'rb'))
        for sw in P['sweeps']:
            row = P['rows'][sw['row_index']]
            yaw, src = yaw_src(sw)
            pose = estimate_base_pose(sw['base_points'], base_model.template_mm)
            out.append(dict(bag=P['bag'], sweep_idx=int(row['sweep_idx']), t_start=sw['t_start'],
                            frames=sw['frames'], yaw=yaw, src=src, heading=pose.heading_deg,
                            verdict_p1=row['verdict_p1']))
    pickle.dump(out, open(SMALL, 'wb'), protocol=4)
    print('prep:', len(out), 'sweeps ->', SMALL)


# ------------------------------------------------------------------ A
def evening_group(bag, idx):
    for b, mx, g in GROUP_RULES:
        if bag == b and idx <= mx:
            return g
    return GROUP_DEFAULT


def stage_A():
    tpl = template()
    rows = []
    for r in csv.DictReader(open(os.path.join(A.replay_dir, 'sweeps.csv'))):
        if r['verdict_intrinsic'] != 'ACCEPTED' or not r['kappa_deg']:
            continue
        rows.append(dict(source='181127', bag=r['bag'], sweep_idx=int(r['sweep_idx']), t_start=float(r['t_start']),
                         group=r['bag'], kappa=float(r['kappa_deg']), res=float(r['template_residual_mm']),
                         res_win=float(r['template_residual_window_mm']), worst_pair=r['worst_pair'],
                         epoch=r['epoch_id'], kappa_out=float(r['kappa_out_deg']) if r['kappa_out_deg'] else None,
                         kappa_ret=float(r['kappa_ret_deg']) if r['kappa_ret_deg'] else None,
                         verdict='ACCEPTED'))
    for sw in pickle.load(open(SMALL, 'rb')):   # all 14 evening sweeps from the cap-lifted solve, one footing
        est, trk = solve(sw['frames'], sw['yaw'], sw['src'], tpl)
        if trk is None:
            print('evening refused even cap-lifted', sw['bag'], sw['sweep_idx'], est)
            continue
        yaw_deg = math.degrees(est.yaw_offset_rad)
        yo, yr = R.yaw_by_direction(trk, sw['yaw'], tpl, est.lag_s, est.phi_raw_deg)
        rows.append(dict(source='221458-caplifted', bag=sw['bag'], sweep_idx=sw['sweep_idx'], t_start=sw['t_start'],
                         group=evening_group(sw['bag'], sw['sweep_idx']),
                         kappa=wrap_deg(yaw_deg - sw['heading']), res=est.template_residual_mm,
                         res_win=est.template_residual_window_mm, worst_pair=worst_pair(trk, est, sw['yaw']),
                         epoch='0', kappa_out=None if yo is None else wrap_deg(yo - sw['heading']),
                         kappa_ret=None if yr is None else wrap_deg(yr - sw['heading']),
                         verdict='ACCEPTED' if est.template_residual_mm <= 1.0 else 'CONSTELLATION_RESIDUAL'))
    # sitting reference: median kappa of the group's sweeps with residual <= 0.5 mm
    ref = {}
    for g in set(r['group'] for r in rows):
        k = [r['kappa'] for r in rows if r['group'] == g and r['res'] <= 0.5]
        ref[g] = (float(np.median(k)), len(k), 'own') if k else None
    for tgt, src in REF_FALLBACK.items():
        if tgt in ref and ref[tgt] is None and ref.get(src):
            ref[tgt] = (ref[src][0], ref[src][1], src.split()[0] + ' (no clean sweep)')
        elif tgt in ref and ref[tgt] is not None and src != tgt:
            pass   # the target has its own clean sweeps: keep 'own'
    # a daytime bag with no clean sweep: reference = the nearest-in-time bag of the SAME date that has one
    t_of = {g: np.median([r['t_start'] for r in rows if r['group'] == g]) for g in ref}
    for g in [g for g in ref if ref[g] is None]:
        day = g[:10]
        cands = [h for h in ref if ref[h] is not None and h[:10] == day and ref[h][2] == 'own']
        if cands:
            h = min(cands, key=lambda h: abs(t_of[h] - t_of[g]))
            ref[g] = (ref[h][0], ref[h][1], 'nearest bag ' + h[11:])
    ep = defaultdict(list)
    for r in rows:
        if r['res'] <= 0.5 and r['epoch'] != '':
            ep[r['epoch']].append(r['kappa'])
    for r in rows:
        g = ref[r['group']]
        if g is None:
            r.update(dev=None, ref_n=0, ref_src='none')
        else:
            r.update(dev=r['kappa'] - g[0], ref_n=g[1], ref_src=g[2], ref_kappa=g[0])
        r['dev_epoch'] = (r['kappa'] - float(np.median(ep[r['epoch']]))) if r['epoch'] in ep else None
    rows.sort(key=lambda r: r['t_start'])
    keys = ['source', 'bag', 'sweep_idx', 't_start', 'group', 'verdict', 'kappa', 'res', 'res_win', 'worst_pair',
            'epoch', 'ref_kappa', 'ref_n', 'ref_src', 'dev', 'dev_epoch', 'kappa_out', 'kappa_ret']
    with open(os.path.join(OUT, 'partA_rows.csv'), 'w', newline='') as fh:
        w = csv.DictWriter(fh, keys, extrasaction='ignore')
        w.writeheader()
        w.writerows(rows)
    report_A(rows)
    plot_A(rows)


def report_A(rows):
    L = []
    v = [r for r in rows if r['dev'] is not None]
    L.append(f'Part A: {len(rows)} sweeps with a kappa, {len(v)} with a sitting reference')
    single = [r for r in v if r['ref_n'] == 1]
    L.append(f'  sweeps whose sitting reference rests on ONE clean sweep: {len(single)}')
    L.append('  band (mm)   n   mean dev   SD     max|dev|  (deg)')
    for lo, hi in [(0, .3), (.3, .5), (.5, .7), (.7, .9), (.9, 1.0), (1.0, 1.3)]:
        d = np.array([r['dev'] for r in v if lo < r['res'] <= hi])
        if len(d):
            L.append(f'  {lo:.1f}-{hi:.1f}   {len(d):3d}  {d.mean():+.3f}    {d.std(ddof=1) if len(d) > 1 else float("nan"):.3f}  {np.abs(d).max():.3f}')
        else:
            L.append(f'  {lo:.1f}-{hi:.1f}     0')
    x = np.array([r['res'] for r in v]); y = np.array([r['dev'] for r in v])
    for name, yy in (('dev', y), ('|dev|', np.abs(y))):
        p, cov = np.polyfit(x, yy, 1, cov=True)
        L.append(f'  fit {name} = {p[1]:+.3f} + {p[0]:+.3f} * res   slope SE {math.sqrt(cov[0,0]):.3f} deg/mm  (t = {p[0]/math.sqrt(cov[0,0]):+.2f})')
    # excluding the single-reference sweeps (dev trivially ~0)
    vv = [r for r in v if r['ref_n'] >= 2]
    x2 = np.array([r['res'] for r in vv]); y2 = np.abs(np.array([r['dev'] for r in vv]))
    p, cov = np.polyfit(x2, y2, 1, cov=True)
    L.append(f'  fit |dev| (ref_n>=2, n={len(vv)}) slope {p[0]:+.3f} +/- {math.sqrt(cov[0,0]):.3f} deg/mm')
    hi = [r for r in v if r['res'] > 0.9]
    if hi:
        w = max(hi, key=lambda r: abs(r['dev']))
        L.append(f'  above 0.9 mm: n={len(hi)}, worst |dev| {abs(w["dev"]):.3f} deg ({w["bag"]} #{w["sweep_idx"]}, res {w["res"]:.2f})')
    de = [r['dev_epoch'] for r in rows if r['dev_epoch'] is not None]
    L.append(f'  epoch-level deviation: n={len(de)}, SD {np.std(de, ddof=1):.3f}, max|dev| {np.max(np.abs(de)):.3f}')
    L.append('  direction split, evening (kappa_out / kappa_ret / out-ret, deg):')
    for r in rows:
        if r['source'].startswith('221458') and r['kappa_out'] is not None:
            L.append(f'    {r["bag"][11:16]}#{r["sweep_idx"]:<2d} {lt(r["t_start"])} res {r["res"]:.2f}  {r["kappa_out"]:.3f} / {r["kappa_ret"]:.3f} / {r["kappa_out"]-r["kappa_ret"]:+.3f}  kappa {r["kappa"]:.3f} dev {r["dev"]:+.3f}')
    for gname in ('E1', 'E2'):
        s = [r['kappa_out'] - r['kappa_ret'] for r in rows if r['group'].startswith(gname) and r['kappa_out'] is not None]
        L.append(f'    {gname} out-ret mean {np.mean(s):+.3f} SD {np.std(s, ddof=1):.3f} (n {len(s)})')
    dd = [r['kappa_out'] - r['kappa_ret'] for r in rows if r['source'] == '181127' and r['kappa_out'] is not None]
    L.append(f'    daytime 181127 out-ret mean {np.mean(dd):+.3f} SD {np.std(dd, ddof=1):.3f} (n {len(dd)})')
    txt = '\n'.join(L)
    print(txt)
    open(os.path.join(OUT, 'partA.txt'), 'w').write(txt + '\n')


def plot_A(rows):
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    v = [r for r in rows if r['dev'] is not None]
    days = sorted(set(datetime.datetime.fromtimestamp(r['t_start']).strftime('%m-%d') if not r['group'].startswith('E')
                      else r['group'][:2] for r in v))
    cmap = plt.get_cmap('tab10')
    col = {d: cmap(i % 10) for i, d in enumerate(days)}
    fig, ax = plt.subplots(1, 3, figsize=(17, 5.2))
    for r in v:
        d = r['group'][:2] if r['group'].startswith('E') else datetime.datetime.fromtimestamp(r['t_start']).strftime('%m-%d')
        mk = 'x' if r['ref_n'] == 1 else ('s' if r['group'].startswith('E2') else 'o')
        ax[0].scatter(r['res'], r['dev'], color=col[d], marker=mk, s=28)
        ax[1].scatter(r['res'], abs(r['dev']), color=col[d], marker=mk, s=28)
        if r['dev_epoch'] is not None:
            ax[2].scatter(r['res'], r['dev_epoch'], color=col[d], marker=mk, s=28)
    for d in days:
        ax[0].scatter([], [], color=col[d], label=d if not d.startswith('E') else {'E1': '10-16 eve clean', 'E2': '10-16 22:00 deformed (ref=E1)'}[d])
    ax[0].scatter([], [], color='k', marker='x', label='sitting ref = 1 sweep')
    ax[0].legend(fontsize=7, loc='lower left')
    xs = sorted(v, key=lambda r: r['res'])
    run = np.maximum.accumulate([abs(r['dev']) for r in xs])
    ax[1].step([r['res'] for r in xs], run, where='post', color='k', lw=1, label='running max |dev|')
    ax[1].legend(fontsize=7)
    for a, t in zip(ax, ('kappa - sitting median (res<=0.5)', '|kappa - sitting median|', 'kappa - epoch median (res<=0.5)')):
        a.axvline(1.0, color='r', lw=1); a.axvline(0.9, color='orange', lw=1, ls='--')
        for h in (0.05, 0.20):
            a.axhline(h, color='grey', lw=.6, ls=':')
            if a is not ax[1]:
                a.axhline(-h, color='grey', lw=.6, ls=':')
        a.set_xlabel('template residual (pose-binned, mm)'); a.set_ylabel('deg'); a.set_title(t, fontsize=10)
        a.grid(alpha=.2)
    fig.suptitle(f'BB kappa deviation vs template residual, {len(v)} sweeps 2026-10-10..16 (red = cap 1.0, orange = advisory 0.90)', fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'partA_scatter.png'), dpi=130)


# ------------------------------------------------------------------ B
DIRS = ('tan+', 'tan-', 'rad+', 'rad-', 'z+', 'z-')
MAGS = (0.5, 1.0, 1.5, 2.0, 3.0)
MAGS_C = MAGS   # stage C's ratio columns assume the default magnitudes
_G = {}


def unit(p, d):
    az = math.atan2(p[1], p[0])
    u = {'tan': np.array([-math.sin(az), math.cos(az), 0.0]), 'rad': np.array([math.cos(az), math.sin(az), 0.0]),
         'z': np.array([0.0, 0.0, 1.0])}[d[:-1]]
    return u if d[-1] == '+' else -u


def perturbed(tpl, spec):
    T = tpl.points_mm.copy()
    kind = spec['kind']
    if kind == 'single':
        T[spec['k']] += spec['mag'] * unit(T[spec['k']], spec['dir'])
    elif kind == 'pair36':   # Q3 (idx 2) and Q6 (idx 5) tangential; s3/s6 = +/-1
        T[2] += spec['mag'] * spec['s3'] * unit(T[2], 'tan+')
        T[5] += spec['mag'] * spec['s6'] * unit(T[5], 'tan+')
    elif kind == 'rot':
        a = math.radians(spec['mag'])
        Rz = np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])
        T = T @ Rz.T
    elif kind == 'scale':
        T[:, :2] *= (1.0 + spec['mag'])
    return dataclasses.replace(tpl, points_mm=T)


def _init():
    _G['sw'] = {(s['bag'], s['sweep_idx']): s for s in pickle.load(open(SMALL, 'rb'))}
    _G['tpl'] = template()


def _work(spec):
    sw = _G['sw'][(spec['bag'], spec['sweep_idx'])]
    tpl = _G['tpl'] if spec['kind'] == 'base' else perturbed(_G['tpl'], spec)
    t0 = time.time()
    est, trk = solve(sw['frames'], sw['yaw'], sw['src'], tpl)
    out = dict(spec)
    out['dt_s'] = time.time() - t0
    if trk is None:
        out.update(refusal=est)
        return out
    out.update(refusal='', yaw_deg=math.degrees(est.yaw_offset_rad), res=est.template_residual_mm,
               res_win=est.template_residual_window_mm, worst_pair=worst_pair(trk, est, sw['yaw']),
               lag_ms=est.lag_s * 1e3, n_frames=est.n_frames)
    return out


def stage_B(which):
    from multiprocessing import Pool
    sweeps = list(which)
    specs = []
    for bag, i in sweeps:
        b = dict(bag=bag, sweep_idx=i)
        specs.append(dict(b, kind='base', k=-1, dir='', mag=0.0))
        for k in range(7):
            for d in DIRS:
                for m in A.mags:
                    specs.append(dict(b, kind='single', k=k, dir=d, mag=m))
        for s3, s6, nm in ((1, -1, 'opp'), (-1, 1, 'opp'), (1, 1, 'same'), (-1, -1, 'same')):
            for m in (0.5, 1.0, 1.5):
                specs.append(dict(b, kind='pair36', k=-1, dir=f'{nm}{s3:+d}{s6:+d}', mag=m, s3=s3, s6=s6))
        specs.append(dict(b, kind='rot', k=-1, dir='z', mag=0.1))
        specs.append(dict(b, kind='rot', k=-1, dir='z', mag=-0.1))
        specs.append(dict(b, kind='scale', k=-1, dir='xy', mag=0.0075))
        specs.append(dict(b, kind='scale', k=-1, dir='xy', mag=-0.0075))
    print(f'B: {len(specs)} solves on {len(sweeps)} sweeps', flush=True)
    t0 = time.time()
    res = []
    keys = ['bag', 'sweep_idx', 'kind', 'k', 'dir', 'mag', 's3', 's6', 'refusal', 'yaw_deg', 'dyaw_deg', 'res',
            'res_win', 'res_base', 'worst_pair', 'lag_ms', 'n_frames', 'dt_s']
    with Pool(5, initializer=_init) as pool:
        for n, o in enumerate(pool.imap_unordered(_work, specs, chunksize=4)):
            res.append(o)
            if n % 100 == 0:
                print(f'  {n}/{len(specs)} {time.time() - t0:.0f} s', flush=True)
    base = {(o['bag'], o['sweep_idx']): o for o in res if o['kind'] == 'base'}
    for o in res:
        b = base[(o['bag'], o['sweep_idx'])]
        o['res_base'] = b.get('res')
        o['dyaw_deg'] = (o['yaw_deg'] - b['yaw_deg']) if o.get('yaw_deg') is not None else None
    res.sort(key=lambda o: (o['bag'], o['sweep_idx'], o['kind'], o['k'], o['dir'], o['mag']))
    with open(os.path.join(OUT, 'partB_injections.csv'), 'w', newline='') as fh:
        w = csv.DictWriter(fh, keys, extrasaction='ignore')
        w.writeheader()
        w.writerows(res)
    print(f'B done: {len(res)} solves in {time.time() - t0:.0f} s; refusals {sum(1 for o in res if o["refusal"])}')


# ------------------------------------------------------------------ C
CAPS = (1.0, 1.25, 1.5, 2.0)


def stage_C():
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    B = list(csv.DictReader(open(os.path.join(OUT, 'partB_injections.csv'))))
    A = list(csv.DictReader(open(os.path.join(OUT, 'partA_rows.csv'))))
    for o in B:
        for f in ('mag', 'dyaw_deg', 'res', 'res_base', 'res_win'):
            o[f] = float(o[f]) if o.get(f) not in (None, '') else None
        o['k'] = int(o['k']); o['sweep_idx'] = int(o['sweep_idx'])
    L = []
    ok = [o for o in B if not o['refusal']]
    L.append(f'Part B: {len(B)} solves, {len(B) - len(ok)} refused for another reason: '
             + ', '.join(sorted(set(f"{o['kind']} Q{o['k']+1} {o['dir']} {o['mag']} -> {o['refusal']}" for o in B if o['refusal']))[:20]))
    L.append('controls (per sweep: dyaw deg, residual mm, base residual):')
    for o in B:
        if o['kind'] in ('rot', 'scale') and not o['refusal']:
            L.append(f"  {o['bag'][11:16]}#{o['sweep_idx']} {o['kind']} {o['mag']:+g}: dyaw {o['dyaw_deg']:+.4f}  res {o['res']:.3f} (base {o['res_base']:.3f})")
    # per mode linear fits across magnitudes and sweeps: dyaw = a*d (signed, through origin); res = b*d fitted on
    # points where the injection dominates (res > 1.5*base) -> mode slope a/b = deg per mm of residual
    modes = defaultdict(list)
    for o in ok:
        if o['kind'] == 'single':
            modes[(o['k'], o['dir'])].append(o)
    tab = {}
    L.append('\nsingle-marker modes: a = dyaw/disp (deg/mm, signed fit through 0), b = res/disp (mm/mm, on res > 1.5x base),')
    L.append('  slope = |a|/b (deg per mm of residual), ratio@m = |dyaw|/res at magnitude m (sweep median), lin = a fitted on <=1.0 vs >=2.0')
    L.append('mode       a        b      slope   ' + '  '.join(f'r@{m:g}' for m in MAGS_C) + '   lin(lo/hi)   res@3mm  dyaw@3mm(max)')
    for (k, d), os_ in sorted(modes.items()):
        dsp = np.array([o['mag'] for o in os_]); dy = np.array([o['dyaw_deg'] for o in os_])
        rs = np.array([o['res'] for o in os_]); rb = np.array([o['res_base'] for o in os_])
        a = float(np.sum(dsp * dy) / np.sum(dsp ** 2))
        dom = rs > 1.5 * rb
        b = float(np.sum(dsp[dom] * rs[dom]) / np.sum(dsp[dom] ** 2)) if dom.sum() >= 3 else float('nan')
        lo = dsp <= 1.0; hi = dsp >= 2.0
        alo = float(np.sum(dsp[lo] * dy[lo]) / np.sum(dsp[lo] ** 2)); ahi = float(np.sum(dsp[hi] * dy[hi]) / np.sum(dsp[hi] ** 2))
        ratios = [float(np.median(np.abs(dy[dsp == m]) / rs[dsp == m])) for m in MAGS_C]
        slope = abs(a) / b if b == b else float('nan')
        # residual reached at the displacement that biases yaw 0.1 deg (detection-gap figure)
        d01 = 0.1 / abs(a) if a else float('inf')
        r01 = b * d01 if b == b else float('nan')
        tab[(k, d)] = dict(a=a, b=b, slope=slope, ratios=ratios, alo=alo, ahi=ahi, d01=d01, r01=r01,
                           res3=float(np.median(rs[dsp == 3.0])), dy3=float(np.max(np.abs(dy[dsp == 3.0]))))
        L.append(f'Q{k+1} {d:5s} {a:+.4f}  {b:.3f}  {slope:.4f}  ' + '  '.join(f'{x:.3f}' for x in ratios)
                 + f'   {alo:+.4f}/{ahi:+.4f}  {tab[(k, d)]["res3"]:.2f}  {tab[(k, d)]["dy3"]:.3f}')
    # marker x direction (+/- folded: max over sign) slope table
    L.append('\nslope |a|/b deg/mm, marker x direction (max over +/-):')
    L.append('      tan      rad      z')
    for k in range(7):
        L.append(f'Q{k+1}  ' + '  '.join(f'{max(tab[(k, s + "+")]["slope"], tab[(k, s + "-")]["slope"]):.4f}' for s in ('tan', 'rad', 'z')))
    fin = {m: v for m, v in tab.items() if v['slope'] == v['slope']}
    wm = max(fin, key=lambda m: fin[m]['slope'])
    wslope = fin[wm]['slope']
    L.append(f'\nWORST-CASE single-marker slope: {wslope:.4f} deg/mm of residual, mode Q{wm[0]+1} {wm[1]} '
             f'(a {fin[wm]["a"]:+.4f} deg/mm disp, b {fin[wm]["b"]:.3f})')
    nob = [m for m, v in tab.items() if v['slope'] != v['slope']]
    if nob:
        L.append('  modes whose residual never exceeded 1.5x base up to 3 mm (slope undefined -> invisible): '
                 + ', '.join(f'Q{k+1} {d} (a {tab[(k, d)]["a"]:+.4f}, dyaw@3mm {tab[(k, d)]["dy3"]:.3f}, res@3mm {tab[(k, d)]["res3"]:.2f})' for k, d in nob))
    # direct empirical: worst |dyaw| among injections that a cap C would ACCEPT
    L.append('\ndirect: max |dyaw| over single-marker injections with res <= C (no extrapolation; magnitudes <= 3 mm)')
    for C in CAPS + (0.5, 0.9):
        acc = [o for o in ok if o['kind'] == 'single' and o['res'] <= C]
        w = max(acc, key=lambda o: abs(o['dyaw_deg']))
        L.append(f'  C {C:.2f}: max |dyaw| {abs(w["dyaw_deg"]):.3f} deg (Q{w["k"]+1} {w["dir"]} {w["mag"]} mm, res {w["res"]:.2f}, sweep #{w["sweep_idx"]})')
    # detection gap: modes that bias yaw 0.1 deg below 0.5 mm residual
    L.append('\ndetection gap: residual at which each mode has biased yaw by 0.1 deg (b * 0.1/|a|), sorted, lowest 10:')
    for m in sorted(tab, key=lambda m: (tab[m]['r01'] if tab[m]['r01'] == tab[m]['r01'] else -1))[:10]:
        v = tab[m]
        L.append(f'  Q{m[0]+1} {m[1]:5s}: disp {v["d01"]:.2f} mm -> residual {v["r01"]:.2f} mm (slope {v["slope"]:.4f})')
    # pair mode
    L.append('\nQ3/Q6 tangential pair modes (median over sweeps): mag, dyaw, res')
    pm = defaultdict(list)
    for o in ok:
        if o['kind'] == 'pair36':
            pm[(o['dir'], o['mag'])].append(o)
    for (d, m), os_ in sorted(pm.items()):
        L.append(f'  {d:9s} {m:.1f} mm: dyaw {np.median([o["dyaw_deg"] for o in os_]):+.4f} (max|| {np.max(np.abs([o["dyaw_deg"] for o in os_])):.4f})  res {np.median([o["res"] for o in os_]):.2f}  ratio {np.median([abs(o["dyaw_deg"]) / o["res"] for o in os_]):.4f}')
    # cap table
    resA = np.array([float(r['res']) for r in A])
    devA = [(float(r['res']), abs(float(r['dev']))) for r in A if r['dev'] not in ('', None)]
    L.append('\nPart C cap table (worst-case = worst slope x C; observed = Part A max |dev| at res <= C; refused = Part A rows res > C)')
    for C in CAPS:
        obs = max(d for x, d in devA if x <= C)
        nref = int(np.sum(resA > C))
        L.append(f'  C {C:.2f}: worst-case {wslope * C:.3f} deg | observed {obs:.3f} deg | refused {nref}/{len(resA)} ({100 * nref / len(resA):.1f} %)')
    L.append(f'  C keeping worst-case <= 0.20 deg: <= {0.20 / wslope:.2f} mm; <= 0.10 deg: <= {0.10 / wslope:.2f} mm')
    txt = '\n'.join(L)
    print(txt)
    open(os.path.join(OUT, 'partC.txt'), 'w').write(txt + '\n')
    # plots
    fig, ax = plt.subplots(1, 2, figsize=(15, 5.6))
    cols = {'tan': 'C0', 'rad': 'C1', 'z': 'C2'}
    for d in cols:
        s = [o for o in ok if o['kind'] == 'single' and o['dir'][:-1] == d]
        ax[0].scatter([o['res'] for o in s], [abs(o['dyaw_deg']) for o in s], s=8, color=cols[d], alpha=.5, label=f'single marker {d}')
    s = [o for o in ok if o['kind'] == 'pair36']
    ax[0].scatter([o['res'] for o in s], [abs(o['dyaw_deg']) for o in s], s=22, marker='D', color='C3', label='Q3+Q6 tangential pair')
    ax[0].scatter([x for x, d in devA], [d for x, d in devA], s=18, marker='x', color='k', label='Part A |dev| (real sweeps)')
    xx = np.linspace(0, 3.5, 10)
    ax[0].plot(xx, wslope * xx, 'k--', lw=1, label=f'worst slope {wslope:.3f} deg/mm (Q{wm[0]+1} {wm[1]})')
    ax[0].axvline(1.0, color='r', lw=1); ax[0].axvline(0.9, color='orange', ls='--', lw=1)
    for h in (0.05, 0.1, 0.2):
        ax[0].axhline(h, color='grey', ls=':', lw=.6)
    ax[0].set_xlim(0, 3.5); ax[0].set_ylim(0, max(0.45, 1.05 * max(abs(o['dyaw_deg']) for o in ok if o['res'] <= 3.5)))
    ax[0].set_xlabel('template residual (mm)'); ax[0].set_ylabel('|delta yaw| (deg)'); ax[0].legend(fontsize=7)
    ax[0].set_title('injected template perturbations (7 evening sweeps) vs real sweeps', fontsize=10)
    # linearity: dyaw vs displacement for the 6 highest-|a| modes
    top = sorted(tab, key=lambda m: -abs(tab[m]['a']))[:6] + [wm] + sorted(tab, key=lambda m: tab[m]['b'] if tab[m]['b'] == tab[m]['b'] else 0)[:2]
    for i, m in enumerate(dict.fromkeys(top)):
        s = [o for o in ok if o['kind'] == 'single' and (o['k'], o['dir']) == m]
        mg = sorted(set(o['mag'] for o in s))
        med = [np.median([o['dyaw_deg'] for o in s if o['mag'] == g]) for g in mg]
        rr = [np.median([o['res'] for o in s if o['mag'] == g]) for g in mg]
        ax[1].plot(mg, med, 'o-', label=f'Q{m[0]+1} {m[1]} (res@3mm {rr[-1]:.2f})')
    ax[1].axhline(0, color='k', lw=.5)
    ax[1].set_xlabel('displacement (mm)'); ax[1].set_ylabel('delta yaw (deg, sweep median)'); ax[1].legend(fontsize=7)
    ax[1].set_title('linearity: delta yaw vs displacement (strongest and least-visible modes)', fontsize=10)
    for a in ax:
        a.grid(alpha=.2)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'partB_injection.png'), dpi=130)


def _b_sweeps(items):
    out = []
    for it in items:
        bag, idx = it.split(':')
        out += [(bag, int(i)) for i in idx.split(',')]
    return out


def main():
    global OUT, SMALL
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('stage', choices=('decode', 'prep', 'A', 'B', 'C'))
    ap.add_argument('--replay-dir', help='bb_calibration_replay.py output dir with sweeps.csv (stage A)')
    ap.add_argument('--bags', nargs='+', default=[], help='bag dirs (decode)')
    ap.add_argument('--pickles', nargs='+', default=[], help='decoded-sweep pickles (prep)')
    ap.add_argument('--out', help='output dir (default temp/probes/bb_residual_cap_evidence/<timestamp>/)')
    ap.add_argument('--ref-group', action='append', default=[], metavar='TARGET=SOURCE',
                    help='group TARGET borrows the sitting reference of group SOURCE (replaces the built-in table)')
    ap.add_argument('--b-sweeps', nargs='+', metavar='BAG:i,j,..',
                    help='stage B sweeps (default: the 2026-10-16 clean evening sweeps)')
    ap.add_argument('--mags', default=','.join(f'{m:g}' for m in MAGS),
                    help='stage B single-marker magnitudes in mm (stage C needs the default set)')
    a = ap.parse_args()
    if a.ref_group:
        REF_FALLBACK.clear()
        for it in a.ref_group:
            t, s_ = it.split('=')
            REF_FALLBACK[t] = s_
    a.mags = tuple(float(x) for x in a.mags.split(','))
    OUT = a.out or os.path.join(REPO, 'temp', 'probes', 'bb_residual_cap_evidence',
                                datetime.datetime.now().strftime('%Y%m%d-%H%M%S'))
    OUT = os.path.abspath(OUT)
    os.makedirs(OUT, exist_ok=True)
    SMALL = os.path.join(OUT, 'evening_inputs.pkl')
    for k, v in vars(a).items():
        setattr(A, k, v)
    print('out:', OUT)
    if a.stage == 'decode':
        if not a.bags:
            ap.error('decode needs --bags')
        stage_decode(a.bags)
    elif a.stage == 'prep':
        if not a.pickles:
            ap.error('prep needs --pickles')
        stage_prep()
    elif a.stage == 'A':
        if not a.replay_dir:
            ap.error('A needs --replay-dir')
        stage_A()
    elif a.stage == 'B':
        stage_B(_b_sweeps(a.b_sweeps) if a.b_sweeps else B_SWEEPS_DEFAULT)
    else:
        stage_C()


if __name__ == '__main__':
    main()
