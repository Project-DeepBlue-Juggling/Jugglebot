"""Emit lead 0 vs 1: what the earlier fences and windows cost in the sim gate.

Emit lead 1 sends every setpoint frame one knot (25 ms) early, so every Catch
re-aim fence closes 25 ms earlier and every handoff is planned from a Fit 25 ms
younger (design ``emit_lead_design.md`` section 5a, logbook
``2026-11-25-emit-lead-p2-measurements.md``).  This probe runs the SKILLS GATE
(``sim/skills_gate.py``, tracker-sourced aims, the gate's default) once per lead
in its own subprocess (the constants are import-time, so the generated
``JB_TRAJ_EMIT_LEAD_KNOTS`` is set on BOTH import spellings before anything
under ``jugglebot.motion.skills`` loads) and tallies, per config, lead and seed:

* Catches made / dropped (the gate's own table);
* re-aims accepted (the re-solve path ``RESEND`` and the late-bump path
  ``BUMP``, which a Catch that carries a Throw takes outside the snap band),
  and re-aims declined by reason code (RESEND-SKIPPED / RESEND-REFUSED /
  BUMP-DECLINED lines, once per (catch, reason), the bumper's ``BUMP_STATS``
  codes, and the two SILENT fences, FREEZE and MIN-WINDOW, which close the live
  catch without a line);
* the last accepted re-aim's time before touch-down, per Catch;
* band-REST installs and their slide in knots against ``BAND_REST_SLIDE_CAP_KNOTS``;
* refused installs by code, and exceptions ("compile errors").

THE SIM SOLVE IS INSTANTANEOUS (install time = dispatch time), so this measures
fences and windows, NOT solve lateness; it knows nothing of real tracker noise,
real solve times or the Hand.

    # one config, both leads, seeds 0-2 (child processes run under nice 10):
    python tools/probes/emit_lead_admissibility.py --configs columns --seeds 0 1 2
    # the whole matrix (CPU heavy: nice, one at a time, under the test lock):
    flock /tmp/jugglebot-tests.lock python tools/probes/emit_lead_admissibility.py
    # re-print from saved JSON without running anything:
    python tools/probes/emit_lead_admissibility.py --summarise OUT_DIR

Outputs: ``--out`` (default ``temp/probes/emit_lead_admissibility/<stamp>/``):
``<config>_lead<L>_seed<S>.json`` and ``.log`` per run, and ``summary.md``.
"""
from __future__ import annotations

import argparse
import contextlib
import io
import json
import os
import re
import subprocess
import sys
import time

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))

#: config name -> skills_gate argv (without --seeds).
CONFIGS = {
    'columns': ['--planner-mode', 'pose'],
    'columns_turned45': ['--planner-mode', 'pose', '--angle-deg', '45'],
    'columns_turned90': ['--planner-mode', 'pose', '--angle-deg', '90'],
    # the one-ball oval is entered by a Ball Butler reload (a fed entry)
    'oval_1ball': ['--learn', '--pattern', 'oval_1ball',
                   '--separation-mm', '150', '--apex-m', '0.95',
                   '--dwell-s', '0.5', '--target-throws', '12'],
    'columns_1ball_fed': ['--learn', '--pattern', 'columns', '--one-ball-fed',
                          '--feed-angle-deg', '11.9',
                          '--feed-speed-mmps', '5600',
                          '--target-throws', '12'],
}


# ── child: one lead, one config, one seed ──────────────────────────────────

def _child(lead: int, config: str, seed: int, out_json: str) -> int:
    for p in (os.path.join(_ROOT, 'ros_ws', 'src', 'jugglebot'), _ROOT,
              os.path.join(_ROOT, 'config', 'generated')):
        if p not in sys.path:
            sys.path.insert(0, p)
    import hardware_config as hw_top
    import jugglebot.hardware_config as hw
    hw_top.JB_TRAJ_EMIT_LEAD_KNOTS = hw.JB_TRAJ_EMIT_LEAD_KNOTS = lead
    from jugglebot.motion.skills import executor as ex
    from sim import skills_gate as sg
    assert ex.EMIT_LEAD_KNOTS == lead, (ex.EMIT_LEAD_KNOTS, lead)
    dt = float(hw.JB_TRAJ_KNOT_DT_S)

    instances = []
    reaims = []          # per accepted re-aim: dict(idx, t_abs, to_touch_s)
    lines = []           # every resend line, for the decline census
    fences = []          # silent closes: dict(idx, fence, to_touch_s)
    orig_init = ex.SkillExecutor.__init__
    orig_resend = ex.SkillExecutor._resend_live_catch

    def init(self, *a, **k):
        orig_init(self, *a, **k)
        instances.append(self)

    def resend(self, t_abs_s):
        lc = self._live_catch
        pre = (None if lc is None else
               (lc[0], float(lc[1].t_land_s), lc[2]))
        out = orig_resend(self, t_abs_s)
        for ln in out:
            lines.append(ln)
            if ((' RESEND skill ' in ln or ' BUMP skill ' in ln)
                    and self._live_catch is not None):
                reaims.append(dict(
                    idx=self._live_catch[0], t_abs=float(t_abs_s),
                    to_touch_s=float(self._live_catch[1].t_land_s) - t_abs_s))
        if (pre is not None and not out and self._live_catch is None):
            # closed silently: FREEZE (inside catch_freeze_s) or MIN-WINDOW
            to_touch = pre[1] - float(t_abs_s)
            fences.append(dict(
                idx=pre[0], to_touch_s=to_touch,
                fence='FREEZE' if to_touch <= self.catch_freeze_s + 1e-12
                else 'MIN-WINDOW'))
        return out

    ex.SkillExecutor.__init__ = init
    ex.SkillExecutor._resend_live_catch = resend
    sg.reset_bump_stats()

    buf = io.StringIO()
    err = None
    t_start = time.time()
    try:
        with contextlib.redirect_stdout(buf):
            rc = sg.main(CONFIGS[config] + ['--seeds', str(seed)])
    except Exception as exc:                       # noqa: BLE001
        rc, err = -1, '%s: %s' % (type(exc).__name__, exc)
    wall = time.time() - t_start
    text = buf.getvalue()

    makes = drops = None
    for ln in text.splitlines():
        m = re.match(r'\[skills_gate\] %d\s+(PASS|FAIL)\s+(\d+)\s+(\d+)\s+(\d+)'
                     % seed, ln)
        if m:
            makes, drops = int(m.group(3)), int(m.group(4))
        m = re.match(r'\[skills_gate_learn\] %d\s+(PASS|FAIL)\s+(\S+)\s+(\S+)\s+'
                     r'(\S+)\s+(\S+)\s+(\d+)\s+(\d+)\s+(\d+)' % seed, ln)
        if m:
            drops, makes = int(m.group(7)), int(m.group(8))

    declined = {}
    for ln in lines:
        m = re.search(r'RESEND-SKIPPED (\S+) skill', ln)
        if m:
            declined[m.group(1)] = declined.get(m.group(1), 0) + 1
        m = re.search(r'BUMP-DECLINED (\S+) skill', ln)
        if m:
            declined['BUMP:' + m.group(1)] = \
                declined.get('BUMP:' + m.group(1), 0) + 1
        m = re.search(r'RESEND-REFUSED (\S+) at skill', ln)
        if m:
            declined['REFUSED-' + m.group(1)] = \
                declined.get('REFUSED-' + m.group(1), 0) + 1
    fence_counts = {}
    for f in fences:
        fence_counts[f['fence']] = fence_counts.get(f['fence'], 0) + 1

    band, refused, installs, resend_accepted = [], {}, 0, 0
    for inst in instances:
        for _idx, _skill, res in inst.results:
            installs += 1
            if not res.accepted:
                refused[res.code] = refused.get(res.code, 0) + 1
            if res.accepted and 'band REST' in res.message:
                m = re.search(r'slide (\d+) of at most (\d+) knots', res.message)
                band.append(int(m.group(1)) if m else 0)   # no REBASE: slide 0
            elif not res.accepted and 'slide cap' in res.message:
                band.append(99)                            # refused over the cap
    # last accepted re-aim per Catch (idx)
    last = {}
    for r in reaims:
        last[r['idx']] = r
    doc = dict(
        lead=lead, config=config, seed=seed, rc=rc, error=err, wall_s=wall,
        dt=dt, makes=makes, drops=drops, installs=installs,
        refused_installs=refused, reaims_accepted=len(reaims),
        catches_reaimed=len(last),
        last_reaim_to_touch_s=sorted(r['to_touch_s'] for r in last.values()),
        all_reaim_to_touch_s=sorted(r['to_touch_s'] for r in reaims),
        resend_declined=declined, silent_fences=fence_counts,
        silent_fence_to_touch_s=sorted(f['to_touch_s'] for f in fences),
        bump=dict(accepted=sg.BUMP_STATS['accepted'],
                  declined=dict(sg.BUMP_STATS['declined'])),
        band_rest_slides_knots=band,
        band_rest_slide_cap=ex.BAND_REST_SLIDE_CAP_KNOTS)
    with open(out_json, 'w') as fh:
        json.dump(doc, fh, indent=1, sort_keys=True)
    with open(out_json[:-5] + '.log', 'w') as fh:
        fh.write(text)
        fh.write('\n'.join(lines))
    return 0


# ── parent: spawn, then summarise ──────────────────────────────────────────

def _pct(xs, q):
    if not xs:
        return float('nan')
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(round(q * (len(xs) - 1))))]


def summarise(out_dir: str) -> str:
    runs = []
    for fn in sorted(os.listdir(out_dir)):
        if fn.endswith('.json') and '_lead' in fn:
            runs.append(json.load(open(os.path.join(out_dir, fn))))
    rows = ['| config | lead | seeds | makes | drops | re-aims acc | catches re-aimed '
            '| last re-aim to touch-down ms p10/p50/p90 (min) | declined by code '
            '| silent fences | bump acc / declined | band-REST n (slide 0..cap, max) '
            '| refused installs | errors |',
            '|' + '---|' * 14]
    per_seed = ['', '| config | lead | seed | makes | drops | re-aims acc | wall s |',
                '|---|---|---|---|---|---|---|']
    for cfg in CONFIGS:
        for lead in (0, 1):
            rs = [r for r in runs if r['config'] == cfg and r['lead'] == lead]
            if not rs:
                continue
            def tot(key):
                d = {}
                for r in rs:
                    for k, v in r[key].items():
                        d[k] = d.get(k, 0) + v
                return d
            last = [x for r in rs for x in r['last_reaim_to_touch_s']]
            slides = [x for r in rs for x in r['band_rest_slides_knots']]
            bump_dec = {}
            for r in rs:
                for k, v in r['bump']['declined'].items():
                    bump_dec[k] = bump_dec.get(k, 0) + v
            errs = [r['error'] for r in rs if r['error']]
            fmt = lambda d: ', '.join('%s %d' % kv for kv in sorted(d.items())) or '-'
            rows.append('| %s | %d | %s | %s | %s | %d | %d | %s | %s | %s | %d / %s | %s | %s | %s |' % (
                cfg, lead, ','.join(str(r['seed']) for r in rs),
                sum(r['makes'] or 0 for r in rs), sum(r['drops'] or 0 for r in rs),
                sum(r['reaims_accepted'] for r in rs),
                sum(r['catches_reaimed'] for r in rs),
                ('%.0f / %.0f / %.0f (%.0f)' % (
                    1e3 * _pct(last, .1), 1e3 * _pct(last, .5), 1e3 * _pct(last, .9),
                    1e3 * min(last))) if last else '-',
                fmt(tot('resend_declined')), fmt(tot('silent_fences')),
                sum(r['bump']['accepted'] for r in rs), fmt(bump_dec),
                ('%d (max %d / cap %d; hist %s)' % (
                    len(slides), max(slides), rs[0]['band_rest_slide_cap'],
                    fmt({k: slides.count(k) for k in sorted(set(slides))}))
                 ) if slides else '0',
                fmt(tot('refused_installs')), '; '.join(errs) or '-'))
            for r in rs:
                per_seed.append('| %s | %d | %d | %s | %s | %d | %.0f |' % (
                    cfg, lead, r['seed'], r['makes'], r['drops'],
                    r['reaims_accepted'], r['wall_s']))
    return '\n'.join(rows + per_seed) + '\n'


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--configs', nargs='+', default=list(CONFIGS),
                    choices=list(CONFIGS))
    ap.add_argument('--seeds', type=int, nargs='+', default=[0, 1, 2, 3, 4])
    ap.add_argument('--leads', type=int, nargs='+', default=[0, 1])
    ap.add_argument('--out', default=None)
    ap.add_argument('--summarise', default=None, metavar='OUT_DIR')
    ap.add_argument('--child', nargs=4, metavar=('LEAD', 'CONFIG', 'SEED', 'JSON'),
                    help=argparse.SUPPRESS)
    a = ap.parse_args(argv)
    if a.child:
        return _child(int(a.child[0]), a.child[1], int(a.child[2]), a.child[3])
    if a.summarise:
        print(summarise(a.summarise))
        return 0
    out = a.out or os.path.join(_ROOT, 'temp', 'probes', 'emit_lead_admissibility',
                                time.strftime('%Y%m%d_%H%M%S'))
    os.makedirs(out, exist_ok=True)
    for cfg in a.configs:
        for seed in a.seeds:
            for lead in a.leads:
                fn = os.path.join(out, '%s_lead%d_seed%d.json' % (cfg, lead, seed))
                if os.path.exists(fn):
                    continue                       # resume
                t0 = time.time()
                subprocess.run(['nice', '-n', '19', sys.executable, __file__,
                                '--child', str(lead), cfg, str(seed), fn],
                               check=False)
                print('%s lead %d seed %d: %.0f s%s' % (
                    cfg, lead, seed, time.time() - t0,
                    '' if os.path.exists(fn) else '  (NO OUTPUT)'), flush=True)
    text = summarise(out)
    with open(os.path.join(out, 'summary.md'), 'w') as fh:
        fh.write(text)
    print(text)
    return 0


if __name__ == '__main__':
    sys.exit(main())
