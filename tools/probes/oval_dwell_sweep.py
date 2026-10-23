#!/usr/bin/env python3
"""Offline dwell-ladder sweep of the oval carried cell on today's planner.

Origin: the 2026-10-23 oval P2 measure-first review
(logbook/2026-10-23-oval-p2-banking-leak-not-reproduced.md). Question: does
the tilt "banking request" clamp-saturate over the carried window and leak
into leg jerk as the dwell shortens? Answer on 2026-10-23: no -- leak = 0 at
every accepted dwell 0.65..0.45; the cone refuses at 0.40.

Cell: columns_sites(--sep), apex --apex (T = flight_s), limits 350/5000/200000
mm/s, mm/s^2, mm/s^3 and hand 3900; the oval window from
``admissible_sweep._oval_window`` (catch C, release T, land C), solved with
``_hop_throw_cell`` + ``_folded_catch_cell`` (production planner paths).
"where" of a jerk peak = (leg, t) found by re-validating the same plan with a
1 mm/s^3 jerk limit and reading ``peak_leg_index / peak_leg_t_s``.

MODES
  asis     the production code path, untouched.
  patched  PROBE-ONLY ESTIMATE of the plan's P2 proposal: while a plan that
           has a ``then_throw`` terminal is being realised, ``cup_realize.
           _banking_raw`` is monkeypatched so the banking request over the
           loaded span [catch_k:] is the receive->throw slew
           (``_smooth_slew``). This is NOT the production code path, and it is
           NOT neutral on hop-shaped (cross-site) cells: on 2026-10-23 it moved
           hop-control jerk by -106k / -76k / +7k for reasons not decomposed.
           Read patched numbers as an estimate of the oval cell only.

Sanity anchor (2026-10-23, HEAD 48fd8d5a): dwell 0.65 asis reproduces
58,742 (window) / 58,121 (chain) mm/s^3 exactly.

Run (repo root, venv):
  python tools/probes/oval_dwell_sweep.py [--dwells 0.65,0.60,0.55,0.50,0.45,0.40]
         [--sep 150] [--apex 0.95] [--mode asis|patched|both] [--controls]
         [--out temp/probes/oval_dwell_sweep_<date>.json]
Imports only the planner modules + tools/admissible_sweep; writes only --out.
"""
from __future__ import annotations

import argparse
import dataclasses
import datetime
import json
import os
import sys

for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")          # must precede the numpy import

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path[:0] = [ROOT, os.path.join(ROOT, "tools")]

import numpy as np  # noqa: E402
import admissible_sweep as A  # noqa: E402
import jugglebot.motion.trajectory.cup_realize as cr  # noqa: E402
import jugglebot.motion.trajectory.feasibility as fz  # noqa: E402
import jugglebot.motion.skills.segments as sg  # noqa: E402

geom = A.StewartGeometry()
limits = A._limits(350, 5000, 200000, 3900)
cfg = A.sg.SegmentConfig()
low = dataclasses.replace(limits, leg_vel_mmps=1e9, leg_acc_mmps2=1e9, leg_jerk_mmps3=1.0)

caps: list = []      # validate_cycle captures (plan, report, n tilt calls so far)
tcalls: list = []    # tilt_schedule captures
st = dict(patch=False, ctx=None, plan_has_then=False, raw=None)
_o_val, _o_ts, _o_br, _o_ps = fz.validate_cycle, cr.tilt_schedule, cr._banking_raw, sg.plan_segment


def _val(plan, lim, geom_, **kw):
    r = _o_val(plan, lim, geom_, **kw)
    if lim is limits:
        caps.append((plan, r, len(tcalls)))
    return r


def _ts(cup_plan, receive_tilt, throw_tilt, cfg_=None, **kw):
    st['ctx'] = dict(n=int(np.asarray(cup_plan.acc).shape[0]), catch_k=int(cup_plan.catch_k),
                     dt=float(cup_plan.dt), recv=np.asarray(receive_tilt, float).reshape(2),
                     thr=np.asarray(throw_tilt, float).reshape(2))
    st['raw'] = None
    out = _o_ts(cup_plan, receive_tilt, throw_tilt, cfg_, **kw)
    c = dict(st['ctx'])
    c.update(raw=st['raw'], out=np.array(out), has_then=st['plan_has_then'])
    tcalls.append(c)
    return out


def _br(acc, cfg_, start=None):
    r = _o_br(acc, cfg_, start)
    c = st['ctx']
    # PROBE-ONLY P2 ESTIMATE (see docstring): replace the loaded-span request.
    if st['patch'] and st['plan_has_then'] and c is not None and 0 <= c['catch_k'] < c['n'] - 1:
        r = np.array(r, float)
        k = c['catch_k']
        r[k:] = cr._smooth_slew(c['recv'], c['thr'], c['n'] - k, c['dt'], cfg_, checked=False)
    st['raw'] = np.array(r)
    return r


def _ps(kind, seed, terminal, *a, **k):
    st['plan_has_then'] = getattr(terminal, 'then_throw', None) is not None
    return _o_ps(kind, seed, terminal, *a, **k)


fz.validate_cycle, cr.tilt_schedule, cr._banking_raw, sg.plan_segment = _val, _ts, _br, _ps
A.fz.validate_cycle = _val


def where(plan):
    r = _o_val(plan, low, geom)
    return r.peak_leg_index, (None if r.peak_leg_t_s is None else round(r.peak_leg_t_s, 3))


def bank_stats(c):
    cap = np.radians(cr.RealizeConfig().max_tilt_deg)
    k = c['catch_k']
    raw, out = c['raw'][k:], c['out'][k:]
    mag = np.hypot(raw[:, 0], raw[:, 1])
    ax = raw[:, int(np.argmax(np.abs(raw).max(axis=0)))]
    sgn = np.sign(ax[np.abs(ax) > 1e-6])
    return dict(span=len(raw), atcap=int(np.sum(mag >= cap * (1 - 1e-6))),
                rev=int(np.sum(sgn[1:] != sgn[:-1])),
                raw_max_deg=round(float(np.degrees(mag.max())), 2),
                out_maxmag=round(float(np.degrees(np.hypot(out[:, 0], out[:, 1])).max()), 2))


def cell(kind, dwell, patch, site_t, site_c, T):
    caps.clear(); tcalls.clear(); st['patch'] = patch
    if kind == 'oval':
        cs, rs, ts_ = A._oval_window(site_t, site_c, A.sc.apex_m(T), dwell)
    elif kind == 'hop':       # columns cross-site hop: catch@C, release@C, land@T
        cs, rs, ts_ = site_c, site_c, site_t
    else:
        cs, rs, ts_ = site_c, site_c, site_c
    res = dict(kind=kind, dwell=dwell, patch=patch)
    ok, code, seed, tk = A._hop_throw_cell(rs, cs, T, 0.4, limits, geom, cfg)
    res['throw'] = (ok, str(code))
    if not ok:
        return res
    caps.clear(); tcalls.clear()
    ok, code, reps = A._folded_catch_cell(cs, rs, ts_, seed, tk, T, dwell, (0.0, 0.0),
                                          limits, geom, cfg, pattern_flight_s=T)
    res.update(ok=bool(ok), code=str(code))
    rr = [r for r in reps if r is not None]
    if not rr:
        return res
    w = rr[0]
    res['win'] = dict(code=str(w.code), jerk=round(w.peak_leg_jerk_mmps3), vel=round(w.peak_leg_vel_mmps),
                      acc=round(w.peak_leg_acc_mmps2), seat=w.max_cup_seating_deg,
                      close=w.catch_closing_ratio, reasons=list(w.reasons[:2]))
    if len(rr) > 1:
        c2 = rr[1]
        res['chain'] = dict(code=str(c2.code), jerk=round(c2.peak_leg_jerk_mmps3), vel=round(c2.peak_leg_vel_mmps),
                            acc=round(c2.peak_leg_acc_mmps2))
    if len(caps) >= 2 and ok:
        wp, cp = caps[-2], caps[-1]
        res['win']['where'] = where(wp[0]); res['chain']['where'] = where(cp[0])
        cand = [c for c in tcalls[:wp[2]] if c['has_then'] and 0 <= c['catch_k'] < c['n'] - 1]
        res['bank'] = bank_stats(cand[-1]) if cand and cand[-1]['raw'] is not None else None
    elif caps and not ok:
        res['win']['where'] = where(caps[-1][0])
    return res


def fmt_row(r):
    if 'exc' in r:
        return f"| {r['kind']} | {r['dwell']} | {'patched' if r['patch'] else 'asis'} | EXC {r['exc']} | | | | |"
    tag = 'patched' if r['patch'] else 'asis'
    if not r['throw'][0]:
        return f"| {r['kind']} | {r['dwell']} | {tag} | throw cell refused {r['throw'][1]} | | | | |"
    w = r.get('win')
    if not w:
        return f"| {r['kind']} | {r['dwell']} | {tag} | {r.get('code')} (no report) | | | | |"
    ch = r.get('chain', {})
    b = r.get('bank')
    bs = (f"{b['atcap']}/{b['span']} at cap, {b['rev']} rev, raw max {b['raw_max_deg']} deg, out max {b['out_maxmag']} deg"
          if b else '-')
    res = 'ok' if r.get('ok') else f"REFUSED {r.get('code')}"
    return (f"| {r['kind']} | {r['dwell']} | {tag} | {res} | {w['jerk']:,} {w.get('where', '')} | "
            f"{w['vel']} / {w['acc']} | {ch.get('jerk', '-'):,} | {bs} |"
            if ch else
            f"| {r['kind']} | {r['dwell']} | {tag} | {res} | {w['jerk']:,} {w.get('where', '')} | {w['vel']} / {w['acc']} | - | {bs} |")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--dwells', default='0.65,0.60,0.55,0.50,0.45,0.40')
    ap.add_argument('--sep', type=float, default=150.0)
    ap.add_argument('--apex', type=float, default=0.95)
    ap.add_argument('--mode', choices=('asis', 'patched', 'both'), default='both')
    ap.add_argument('--controls', action='store_true',
                    help='also run hop and same-site (columns) controls at --control-dwells')
    ap.add_argument('--control-dwells', default='0.65,0.50',
                    help='comma list of dwells for the --controls runs (default 0.65,0.50)')
    ap.add_argument('--out', default=None)
    a = ap.parse_args()
    dwells = [float(x) for x in a.dwells.split(',')]
    cdw = [float(x) for x in a.control_dwells.split(',')]
    site_t, site_c = A.st.columns_sites(a.sep)
    T = A.sc.flight_s(a.apex)
    plan = [('oval', dwells)] + ([('hop', cdw), ('columns', cdw)] if a.controls else [])
    patches = {'asis': [False], 'patched': [True], 'both': [False, True]}[a.mode]
    out = []
    print('| cell | dwell | mode | result | window jerk mm/s^3 (leg, t) | win vel / acc | chain jerk | bank request on loaded span |')
    print('|---|---|---|---|---|---|---|---|')
    for kind, dws in plan:
        for d in dws:
            for patch in patches:
                try:
                    r = cell(kind, d, patch, site_t, site_c, T)
                except Exception as e:     # a refused/odd cell must not kill the ladder
                    r = dict(kind=kind, dwell=d, patch=patch, exc=repr(e)[:300])
                print(fmt_row(r), flush=True)
                out.append(r)
    path = a.out or os.path.join(ROOT, 'temp', 'probes', f'oval_dwell_sweep_{datetime.date.today().isoformat()}.json')
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    json.dump(dict(args=vars(a), rows=out), open(path, 'w'), default=str, indent=1)
    print(f'\nwrote {path}')
    if a.mode != 'asis':
        print('NOTE: patched rows are a probe-only estimate of the P2 proposal, not the production path; '
              'not neutral on hop-shaped cells.')


if __name__ == '__main__':
    main()
