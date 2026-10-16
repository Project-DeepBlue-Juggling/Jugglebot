#!/usr/bin/env python3
"""Offline replay of the recorded handoff-CATCH timings under a longer handoff lead.

Why this exists
---------------
The 2026-10-14 columns sessions (S1 20:54, S2 21:27, sitting-8 23:12 the day
before) ended 7+5 attempts and refused 6 re-aims with SPLICE_TOO_LATE, all
recorded with HANDOFF_LEAD_EXTRA_KNOTS = 2 (lead 0.275 s).  The lead has since
been raised (now 4 extra knots).  This probe asks, from the RECORDED timings
only, what margin each handoff CATCH would have had at a longer lead and
whether the tracker fit would have existed at the earlier dispatch.  It runs no
solver: it re-uses each recorded round trip P unchanged.

Recipe (run from the repo root, venv active)
--------------------------------------------
  python tools/probes/handoff_replay.py \\
      --log S1=~/.ros/log/2026-10-14-20-54-20-258010-jetson-4122860/launch.log \\
      --log S2=~/.ros/log/2026-10-14-21-27-21-980308-jetson-4138247/launch.log \\
      --log S8=~/.ros/log/2026-10-13-23-12-50-197471-jetson-3905591/launch.log \\
      --bag S1=~/Desktop/rosbags/2026-10-14_20-54-20 \\
      --bag S2=~/Desktop/rosbags/2026-10-14_21-27-22 \\
      --bag S8=~/Desktop/rosbags/2026-10-13_23-12-50_2-Ball-32-Catches
Outputs go to temp/probes/handoff_replay/: handoff_catches.csv, summary.md and
a per-bag fit cache (fit_<tag>.json; delete it to re-read the bag).

Model (per recorded handoff CATCH first dispatch)
-------------------------------------------------
Log facts: trajectory_node `install_segment CATCH Ball n: splice_k=K, plan P ms,
event tau T s` (logged at accept time Ti, P = node service entry -> accept);
skill_node `<D> CATCH skill n: spliced CATCH at knot K` (D = executor stamp) and
`<D> CATCH-AIM skill n: source=.. t_land=..`.
* HANDOFF identification: first-dispatch (not RESEND) CATCH with a scheduled
  release-minus-dispatch R - D > 0.29 s, where R = t0 + K*dt and the plan-clock
  origin t0 = t_land - tau (the CATCH event is the aim landing; verified: t0 is
  constant across an attempt).  Handoffs measure 0.307-0.336 s, the one
  non-handoff (reload) CATCH 0.238 s: a clean gap of ~0.07 s.
* Cliff C = t0 + (K - WIRE_READ_KNOTS) * dt = R - 0.075 s, i.e. D + (R-D) - 0.075.
  This is the DERIVED cliff (about D + 0.238 s); the brief's nominal D + L0 - 0.075
  = D + 0.200 s is contradicted by accepted installs that landed 0.223-0.231 s
  after D, so the derived one is used and the nominal one is also reported.
* Refusals (no K, tau or P in the log): C - D is taken as the median of the same
  attempt's (else session's) accepted handoff CATCHes; the reply time is the
  trajectory_node refusal log line minus D.  For `solver did not answer` the
  true P is unknowable (>= the node's ~250 ms wait), so its margin is an UPPER
  bound.
* Lead L: dispatch moves earlier by s = L - L0; request time req = Ti - P.
  Serial tick: start = max(req - s, I_pred'), I' = start + P, where I_pred' is
  the previous install of the same goal (shifted too if it was a handoff
  CATCH).  margin = C - I'.  RESEND installs/refusals do not move with the lead.
* Fit availability: F = first /balls message of the ball's track (matched by the
  CATCH-AIM t_land to the nearest announced landing_time, < 0.25 s) with
  landing_from_fit True.  available_L = F <= D - s.
* Re-aim budget (F > D'): first install is prior-aimed (P_prior = recorded P if
  the recorded aim was not tracker, else --prior-solve-ms); re-aim request
  R' = max(F, I'); budget = C - R'.
* `margin_adj` swaps the recorded P for the aim-source change (tracker vs prior
  median difference of the session) when availability differs from the record.
"""
from __future__ import annotations

import argparse
import collections
import csv
import json
import os
import re
from pathlib import Path

import numpy as np

DT = 0.025
WIRE_READ_KNOTS = 3
L0 = 0.275
# New-fence model (plans/archived/handoff-solve-chaining.md P1b); mirrors
# schedule.LEAD_S (9 knots) and unified_cycle.detach_knots() -- hard-coded
# because the probe is stdlib+numpy.
LEAD_S = 9 * DT
# Mirrors executor.RESEND_PINNED_BUDGET_FLOOR_S (the raw path's minimum budget).
RESEND_FLOOR_S = LEAD_S - WIRE_READ_KNOTS * DT
N_DETACH = 2
HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
OUT = ROOT / 'temp' / 'probes' / 'handoff_replay'

RE_INST = re.compile(r'^([\d.]+) \[trajectory_node.*install_segment (\w+) Ball (\d+): splice_k=(\d+), '
                     r'plan ([\d.]+) ms, event tau ([\d.]+) s, solve ([\d.]+) ms')
RE_REF = re.compile(r'^([\d.]+) \[trajectory_node.*install_segment refused: (.*)')
RE_SP = re.compile(r'^([\d.]+) \[skill_node.*\]: ([\d.]+) (RESEND )?(?:CATCH |THROW |REST |MAGAZINE )?skill (\d+):.*spliced (\w+) at knot (\d+)')
RE_AIM = re.compile(r'^([\d.]+) \[skill_node.*\]: ([\d.]+) CATCH-AIM skill (\d+): source=(\S+) landing=\(.*t_land=([\d.]+)')
RE_END = re.compile(r'^([\d.]+) \[skill_node.*\]: ([\d.]+) END SPLICE_TOO_LATE at skill (\d+) \(CATCH\): (.*)')
RE_RRF = re.compile(r'^([\d.]+) \[skill_node.*\]: ([\d.]+) RESEND-REFUSED SPLICE_TOO_LATE at skill (\d+)')
RE_GOAL = re.compile(r"goal dispatched \(pattern='(\w+)'")


def parse_log(path, tag):
    inst, refs, ends = [], [], []
    pend = None
    last_ref = None
    goal, pat = 0, None
    for i, l in enumerate(open(os.path.expanduser(path), errors='replace')):
        m = RE_GOAL.search(l)
        if m:
            goal += 1
            pat = m.group(1)
            pend = None
            continue
        m = RE_INST.match(l)
        if m:
            pend = dict(T=float(m.group(1)), kind=m.group(2), k=int(m.group(4)), P=float(m.group(5)) / 1e3,
                        tau=float(m.group(6)), S=float(m.group(7)) / 1e3, line=i + 1)
            continue
        m = RE_REF.match(l)
        if m:
            last_ref = (float(m.group(1)), m.group(2)[:60])
            continue
        m = RE_SP.match(l)
        if m and pend:
            d = dict(pend, sess=tag, D=float(m.group(2)), resend=bool(m.group(3)), skill=int(m.group(4)),
                     goal=goal, pat=pat)
            inst.append(d)
            pend = None
            continue
        m = RE_AIM.match(l)
        if m and inst and int(m.group(3)) == inst[-1]['skill'] and abs(float(m.group(2)) - inst[-1]['D']) < 1e-6:
            inst[-1]['src'] = m.group(4)
            inst[-1]['tland'] = float(m.group(5))
            continue
        m = RE_END.match(l)
        if m and last_ref:
            ends.append(dict(sess=tag, line=i + 1, D=float(m.group(2)), skill=int(m.group(3)), goal=goal, pat=pat,
                             Tref=last_ref[0], why=m.group(4)[:30], resend=False))
            continue
        m = RE_RRF.match(l)
        if m and last_ref:
            ends.append(dict(sess=tag, line=i + 1, D=float(m.group(2)), skill=int(m.group(3)), goal=goal, pat=pat,
                             Tref=last_ref[0], why='resend', resend=True))
    for r in inst:
        if 'tland' in r:
            r['t0'] = r['tland'] - r['tau']
            r['R'] = r['t0'] + r['k'] * DT
            r['RD'] = r['R'] - r['D']
            r['C'] = r['t0'] + (r['k'] - WIRE_READ_KNOTS) * DT
    return inst, ends


def load_fits(bag, cache):
    if cache.exists():
        return json.loads(cache.read_text())
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
    mc = sorted(Path(os.path.expanduser(bag)).glob('*.mcap'))[0]
    anns = {}   # throw_time_ns -> dict(landing, F)
    with open(mc, 'rb') as fh:
        rd = make_reader(fh, decoder_factories=[DecoderFactory()])
        for _s, ch, msg, dm in rd.iter_decoded_messages(topics=['/throw_announcements', '/balls']):
            if ch.topic == '/throw_announcements':
                tt = dm.throw_time.sec * 10**9 + dm.throw_time.nanosec
                lt = dm.landing_time.sec + dm.landing_time.nanosec * 1e-9
                anns.setdefault(tt, dict(landing=lt, F=None, name=dm.thrower_name))
            else:
                for b in dm.balls:
                    tt = b.throw_time.sec * 10**9 + b.throw_time.nanosec
                    # /balls restamps throw_time by a few hundred ns: match within 1 ms
                    for k, a in anns.items():
                        if abs(k - tt) < 10**6:
                            if b.landing_from_fit and a['F'] is None:
                                a['F'] = msg.log_time * 1e-9
                            break
    out = [dict(landing=a['landing'], F=a['F'], name=a['name']) for a in anns.values()]
    cache.write_text(json.dumps(out))
    return out


def pct(v, q):
    return float(np.percentile(v, q)) if len(v) else float('nan')


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--log', action='append', required=True, metavar='TAG=PATH')
    ap.add_argument('--bag', action='append', default=[], metavar='TAG=PATH')
    ap.add_argument('--leads', default='0.325,0.375,0.425')
    ap.add_argument('--prior-solve-ms', type=float, default=90.0)
    ap.add_argument('--out', default=str(OUT))
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    leads = [float(x) for x in a.leads.split(',')]
    allL = [L0] + leads
    bags = dict(x.split('=', 1) for x in a.bag)
    rows, refus, sanity = [], [], []
    S = collections.OrderedDict()
    for spec in a.log:
        tag, path = spec.split('=', 1)
        inst, ends = parse_log(path, tag)
        fits = load_fits(bags[tag], out / ('fit_%s.json' % tag)) if tag in bags else []
        land = np.array([f['landing'] for f in fits]) if fits else np.array([])
        # session medians for refusals / aim-source adjustment
        ho = [r for r in inst if not r['resend'] and r['kind'] == 'CATCH' and r.get('RD', 0) > 0.29]
        sess_cd = float(np.median([r['C'] - r['D'] for r in ho]))
        trk = [r['P'] for r in ho if r.get('src') == 'tracker']
        pri = [r['P'] for r in ho if r.get('src') != 'tracker']
        dP = (np.median(trk) - np.median(pri)) if trk and pri else 0.04
        # predecessor chain: previous install of the same goal in log order
        order = sorted(inst, key=lambda r: r['T'])
        S[tag] = dict(ho=ho, ends=ends, cd=sess_cd, dP=dP, n_inst=len(inst))
        for idx, r in enumerate(order):
            r['key'] = idx
            r['req'] = r['T'] - r['P']
        prevmap = {}
        lastg = {}
        for r in order:
            prevmap[r['key']] = lastg.get(r['goal'])
            lastg[r['goal']] = r
        for r in ho:
            if fits and len(land):
                j = int(np.argmin(np.abs(land - r['tland'])))
                r['F'] = fits[j]['F'] if abs(land[j] - r['tland']) < 0.25 else None
                r['matched'] = abs(land[j] - r['tland']) < 0.25
            else:
                r['F'], r['matched'] = None, False
            r['req'] = r['T'] - r['P']

        def finish(r, s, memo):
            """install instant of r when handoff dispatches move s earlier."""
            if r['key'] in memo:
                return memo[r['key']]
            shift = s if (not r['resend'] and r['kind'] == 'CATCH' and r.get('RD', 0) > 0.29) else 0.0
            pred = prevmap[r['key']]
            start = r['req'] - shift
            if pred is not None:
                start = max(start, finish(pred, s, memo))
            memo[r['key']] = start + r['P']
            return memo[r['key']]
        for L in allL:
            s = L - L0
            memo = {}
            for r in ho:
                I = finish(r, s, memo)
                r.setdefault('margin', {})[L] = r['C'] - I
                rec_trk = r.get('src') == 'tracker'
                Dp = r['D'] - s
                fav = (r['F'] is not None and r['F'] <= Dp)
                r.setdefault('fav', {})[L] = fav if r['matched'] else None
                # adjusted P for the aim-source change
                P_adj = r['P'] + (dP if (fav and not rec_trk) else (-dP if (not fav and rec_trk and r['matched']) else 0.0))
                r.setdefault('margin_adj', {})[L] = r['margin'][L] - (P_adj - r['P'])
                if r['matched'] and r['F'] is not None and r['F'] > Dp:
                    start = r['req'] - s
                    pr = prevmap[r['key']]
                    if pr is not None:
                        start = max(start, finish(pr, s, memo))
                    Pp = r['P'] if not rec_trk else a.prior_solve_ms / 1e3
                    Ip = start + Pp
                    Rr = max(r['F'], Ip)
                    r.setdefault('budget', {})[L] = r['C'] - Rr
                    # New fences: the executor's cliff estimate is dt/2 earlier
                    # than C; t_rel = R = C + WIRE_READ_KNOTS*dt.  A re-send at
                    # t=Rr is PINNED while t + LEAD_S <= t_rel + n*dt - dt/2,
                    # and then needs budget_est >= the floor; past the band it
                    # takes the raw path (not a pinned budget at all).
                    t_rel = r['C'] + WIRE_READ_KNOTS * DT
                    snapped = Rr + LEAD_S <= t_rel + N_DETACH * DT - DT / 2
                    b_est = r['C'] - DT / 2 - Rr
                    r.setdefault('pinned_firable', {})[L] = bool(snapped and b_est >= RESEND_FLOOR_S - 1e-9)
                    r.setdefault('raw_path', {})[L] = bool(not snapped)
        # refusals: modelled with C - D and the reply offset
        for e in ends:
            same = [r for r in ho if r['goal'] == e['goal']]
            cd = float(np.median([r['C'] - r['D'] for r in same])) if same else sess_cd
            e['cd'] = cd
            e['off'] = e['Tref'] - e['D']
            e['margin'] = {L: cd - e['off'] + (L - L0) for L in allL}
            refus.append(e)
        for r in ho:
            rows.append(r)
    # sanity
    lines = []
    for tag, v in S.items():
        acc = [r for r in rows if r['sess'] == tag]
        neg = [r for r in acc if r['margin'][L0] < -1e-9]
        sn = [r for r in acc if r['margin'][L0] < 0]
        nom = [r for r in acc if (r['D'] + L0 - 0.075) - r['T'] < 0]
        ref = [e for e in refus if e['sess'] == tag and not e['resend']]
        rneg = [e for e in ref if e['margin'][L0] < 0]
        rs = [e for e in refus if e['sess'] == tag and e['resend']]
        lines.append((tag, len(acc), len(neg), len(nom), len(ref), len(rneg), len(rs), v['cd']))
    # csv
    with open(out / 'handoff_catches.csv', 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['sess', 'kind', 'goal', 'skill', 'D', 'T_install_or_refusal', 'P_or_offset_ms', 'splice_k',
                    'C_minus_D_ms', 'aim_src', 'F_minus_D_ms'] + ['margin_ms_L%.3f' % L for L in allL]
                   + ['fit_avail_L%.3f' % L for L in allL] + ['reaim_budget_ms_L%.3f' % L for L in leads]
                   + ['reaim_firable_new_fences_L%.3f' % L for L in leads])
        for r in rows:
            w.writerow([r['sess'], 'accepted', r['goal'], r['skill'], '%.3f' % r['D'], '%.3f' % r['T'], '%.1f' % (r['P'] * 1e3),
                        r['k'], '%.0f' % ((r['C'] - r['D']) * 1e3), r.get('src', ''),
                        '' if r['F'] is None else '%.0f' % ((r['F'] - r['D']) * 1e3)]
                       + ['%.0f' % (r['margin'][L] * 1e3) for L in allL]
                       + ['' if r['fav'][L] is None else int(r['fav'][L]) for L in allL]
                       + ['%.0f' % (r['budget'][L] * 1e3) if L in r.get('budget', {}) else '' for L in leads]
                       + [('' if L not in r.get('budget', {}) else
                           ('pinned' if r['pinned_firable'][L] else ('raw' if r['raw_path'][L] else 'declined')))
                          for L in leads])
        for e in refus:
            w.writerow([e['sess'], 'refused-resend' if e['resend'] else 'refused-END', e['goal'], e['skill'], '%.3f' % e['D'],
                        '%.3f' % e['Tref'], '%.0f' % (e['off'] * 1e3), '', '%.0f' % (e['cd'] * 1e3), '', '']
                       + ['%.0f' % (e['margin'][L] * 1e3) for L in allL] + [''] * (len(allL) + 2 * len(leads)))

    def stat(v):
        v = [x * 1e3 for x in v]
        return 'n=%d neg=%d min=%.0f p5=%.0f p50=%.0f' % (len(v), sum(x < 0 for x in v), min(v) if v else 0, pct(v, 5), pct(v, 50)) if v else 'n=0'
    md = ['# handoff_replay summary', '', 'Leads L (s): ' + ', '.join('%.3f' % L for L in allL) + ' (first = recorded L0).', '']
    md += ['## 0. Sanity at L0 (recorded outcome must reproduce)', '',
           '| sess | accepted handoff CATCH | neg margin (derived C) | neg with nominal C=D+0.200 | END refusals | of which neg | resend refusals (not lead-driven) | median C-D ms |',
           '|---|---|---|---|---|---|---|---|']
    for t in lines:
        md.append('| %s | %d | %d | %d | %d | %d | %d | %.0f |' % (t[0], t[1], t[2], t[3], t[4], t[5], t[6], t[7] * 1e3))
    md += ['', '## 1. Margin by lead (ms; negative = SPLICE_TOO_LATE)', '']
    groups = [(t, [r for r in rows if r['sess'] == t]) for t in S] + [('ALL', rows)]
    md.append('Accepted handoff CATCH installs (fixed recorded P, serial queue):')
    md += ['', '| sess | L | stats | with aim-source P adjustment |', '|---|---|---|---|']
    for t, g in groups:
        for L in allL:
            md.append('| %s | %.3f | %s | %s |' % (t, L, stat([r['margin'][L] for r in g]), stat([r['margin_adj'][L] for r in g])))
    md += ['', 'END refusals (reply offset from stamp, upper bound for `solver did not answer`):', '',
           '| sess | L | stats |', '|---|---|---|']
    ends_only = [e for e in refus if not e['resend']]
    for t in list(S) + ['ALL']:
        g = [e for e in ends_only if t == 'ALL' or e['sess'] == t]
        for L in allL:
            md.append('| %s | %.3f | %s |' % (t, L, stat([e['margin'][L] for e in g])))
    md += ['', 'Per END refusal (ms): ' + '; '.join('%s L%d sk%d off=%.0f C-D=%.0f' % (e['sess'], e['line'], e['skill'], e['off'] * 1e3, e['cd'] * 1e3) +
           ' m=' + '/'.join('%.0f' % (e['margin'][L] * 1e3) for L in allL) for e in ends_only)]
    md += ['', 'Resend refusals (lead-independent, listed): ' + '; '.join('%s L%d sk%d off=%.0f' % (e['sess'], e['line'], e['skill'], e['off'] * 1e3) for e in refus if e['resend'])]
    md += ['', '## 2. Fit availability at dispatch', '', '| sess | n matched | recorded tracker-aimed | ' + ' | '.join('F<=D\' L%.3f' % L for L in allL) + ' |',
           '|---|---|---|' + '---|' * len(allL)]
    for t, g in groups:
        m = [r for r in g if r['matched']]
        if not m:
            continue
        md.append('| %s | %d | %.0f%% | ' % (t, len(m), 100 * np.mean([r.get('src') == 'tracker' for r in m])) +
                  ' | '.join('%.0f%%' % (100 * np.mean([bool(r['fav'][L]) for r in m])) for L in allL) + ' |')
    md += ['', 'Agreement at L0 between (F<=D) and recorded source=tracker: ' +
           ', '.join('%s %.0f%%' % (t, 100 * np.mean([bool(r['fav'][L0]) == (r.get('src') == 'tracker') for r in g if r['matched']]))
                     for t, g in groups if any(r['matched'] for r in g))]
    md += ['', '## 3. Re-aim budget (ms) for catches whose fit is NOT available at dispatch', '',
           'Reference tracker-aim solve: p50 126-146, p90 ~190 ms.', '',
           '| sess | L | n (fit late at L) | p5 | p25 | p50 | p75 (of those with F before cliff) | >=126 (of all n) | >=146 | >=190 |', '|---|---|---|---|---|---|---|---|---|---|']
    for t, g in groups:
        for L in leads:
            vall = [r['budget'][L] * 1e3 for r in g if L in r.get('budget', {})]
            v = [x for x in vall if x > -1e9 and x >= 0]
            if not vall:
                continue
            md.append('| %s | %.3f | %d (%d with F before the cliff) | %.0f | %.0f | %.0f | %.0f | %.0f%% | %.0f%% | %.0f%% |' % (
                t, L, len(vall), len(v), pct(v, 5), pct(v, 25), pct(v, 50), pct(v, 75),
                *[100 * np.mean([x >= th for x in vall]) for th in (126, 146, 190)]))
    md += ['', '## 3b. Re-aim fence model, P1b (window open from first-install end while t_s(t) passes both fences and budget >= %.0f ms)' % (RESEND_FLOOR_S * 1e3), '',
           'Column `reaim_firable_new_fences` in handoff_catches.csv: pinned = the re-send at max(F, first-install end) is inside the snap band with cliff_est - t >= floor (cliff_est = C - dt/2); raw = it falls past the band (raw path, 150 ms free-standing budget, not counted as pinned-firable); declined = PINNED-BUDGET pre-decline.', '',
           '| sess | L | n (fit late at L) | pinned-firable | share | raw path | PINNED-BUDGET declined | old budget>=190 (C-based) |', '|---|---|---|---|---|---|---|---|']
    for t, g in groups:
        for L in leads:
            gg = [r for r in g if L in r.get('budget', {})]
            if not gg:
                continue
            nf = sum(r['pinned_firable'][L] for r in gg)
            nr = sum(r['raw_path'][L] for r in gg)
            nd = len(gg) - nf - nr
            no = sum(r['budget'][L] * 1e3 >= 190 for r in gg)
            md.append('| %s | %.3f | %d | %d | %.0f%% | %d | %d | %d |' % (t, L, len(gg), nf, 100.0 * nf / len(gg), nr, nd, no))
    md += ['', '## 4. Caveats', '',
           '* Solve duration P is replayed unchanged; real P rises with a tracker aim (+~40 ms) and the new node waits to the pinned cliff instead of 250 ms.',
           '* Cliff C is derived assuming the CATCH event tau sits at the aim t_land; the constant bias this implies (R-D 0.313 vs 0.275 nominal) is unexplained by the logs.',
           '* Refused requests have no solve/plan time in the log: margins use the stamp-to-refusal offset; `solver did not answer` rows are upper bounds.',
           '* F is the /balls publish time, not the executor read time; the executor sees it later by a tick or two.',
           '* Aim-source recorded share here is over ALL matched first dispatches (tracker/schedule/schedule:learned); the 74% (146/197) quoted elsewhere excludes schedule:learned.',
           '* Re-aim rows with F after the cliff can never re-aim at any lead: they are counted in n and fail every threshold.',
           '* Resends are not shifted by the lead; queueing against resend solves is modelled only through the previous-install chain.']
    (out / 'summary.md').write_text('\n'.join(md) + '\n')
    print('\n'.join(md))


if __name__ == '__main__':
    main()
