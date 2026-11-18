#!/usr/bin/env python3
"""Offline reach sweep of the human_catch Pattern's level CATCH (PWAY-11 P2, U4).

Question (``design-human-catch.md`` section 5, P2 gate probe 1): from the REST
the machine holds at the wait Site W, for a Landing at offset d from W and a
lead L (landing time minus the claim instant), does the EXISTING command-time
admissibility check -- ``executor.check_segment`` at today's Sitting limits,
``reserve_fresh_lead=True`` as ``trajectory_node`` and ``sim/skills_gate.py``'s
``_make_checker`` call it -- accept ``schedule.compile_human_catch``'s level
CATCH?  ``ok = planned and margin >= 0`` (the checker's own rule).

Two sweeps:

* FIRST PLAN: displacement 0..300 mm (``--step``) in ``--dirs`` directions x
  lead 0.45..0.80 s x arrival apex (``--apexes``, absolute metres; the vertical
  arrival speed is sqrt(2 g (apex - catch plane))).  Per cell: ok, code, worst
  margin, the LIMITING quantity (the leg vel / acc / jerk whose headroom is the
  margin's minimum, read from the gate report ``check_segment`` already builds),
  planned wall.  Reach R(lead, direction) = the farthest offset up to which
  EVERY grid offset is accepted (monotone prefix; the first refusal ends it).
* RE-AIM: the first plan is INSTALLED (``install_segment``, a fresh origin from
  rest) at displacement ``--reaim-first`` and lead ``--reaim-first-lead``; at a
  later instant with ``r`` seconds of lead left, a second terminal moved by
  ``dx`` mm (away from W, in the first plan's direction) is checked against the
  LIVE record (``check_segment`` splices at ``t + LEAD_S``, I-CATCH-4's seam).
  The executor's fences (0.25 s freeze, 0.35 s contact window, 10 mm / 10 ms,
  the lateral clamp) are NOT applied here: this measures only what the gate
  would admit, so a re-aim past the fences is shown as admissible that the node
  would never attempt.

Nothing here changes a limit, margin, gain or timing constant: the limits are
``planner_cells.sitting_limits`` at the generated launch values (600 / 8000 /
200000 mm/s, mm/s^2, mm/s^3; Hand 4500 rev/s^2).  Offline: no ROS, no hardware.

usage: python tools/probes/human_catch_reach.py [--dirs 8] [--step 25]
          [--max-mm 300] [--leads 0.45,...,0.80] [--apexes 1.9]
          [--wait-xy -50,0] [--no-reaim] [--out FILE.md]
"""
from __future__ import annotations

import argparse
import collections
import contextlib
import math
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'config', 'generated'))
sys.path.insert(0, os.path.join(ROOT, 'ros_ws', 'src', 'jugglebot'))
sys.path.insert(0, os.path.join(ROOT, 'tools', 'probes'))
import numpy as np                                                    # noqa: E402
import planner_cells as asw                                           # noqa: E402
from jugglebot.motion.geometry import StewartGeometry                 # noqa: E402
from jugglebot.motion.skills import admissible as adm                 # noqa: E402
from jugglebot.motion.skills import executor as ex                    # noqa: E402
from jugglebot.motion.skills import nominate as nom                   # noqa: E402
from jugglebot.motion.skills import schedule as sc                    # noqa: E402
from jugglebot.motion.skills import segments as sg                    # noqa: E402
from jugglebot.motion.skills import sites as st                       # noqa: E402

G_MM_S2 = 9810.0
#: ``skill_node._DISPATCH_LOOKAHEAD_S`` (1 / 40 Hz tick + one 25 ms knot).
DISPATCH_LOOKAHEAD_S = 0.05
T_NOW = 100.0
#: Default wait Site: the columns Pattern's Site 0 (P1, nearest Ball Butler).
DEFAULT_WAIT_XY = (-50.0, 0.0)
LEADS = (0.45, 0.50, 0.55, 0.60, 0.65, 0.70, 0.75, 0.80)
REAIM_LEADS = (0.35, 0.40, 0.45, 0.50, 0.55, 0.60)
REAIM_MM = (10, 20, 30, 40, 50, 70, 100)

Result = collections.namedtuple(
    'Result', 'ok planned code margin limiting peaks wall_s')


def arrival_vel_mm_s(apex_m: float) -> np.ndarray:
    """Vertical arrival at the Catch plane of a ball whose absolute apex is
    ``apex_m`` (design D9: 1.9 m -> 4.58 m/s)."""
    rise = float(apex_m) * 1000.0 - st.CATCH_CUP_Z_MM
    if not rise > 0.0:
        raise ValueError('apex %.3f m is not above the Catch plane' % apex_m)
    return np.array([0.0, 0.0, -math.sqrt(2.0 * G_MM_S2 * rise)])


@contextlib.contextmanager
def _capture_reports():
    """Record every gate report ``check_segment`` hands ``admissible.margin``
    (the executor module reads ``adm.margin`` at call time): the only way to
    see WHICH of vel / acc / jerk limits, without a second copy of the check."""
    box = []
    real = adm.margin

    def spy(report, limits, frac=adm.MARGIN_FRAC):
        box.append((report, limits, frac))
        return real(report, limits, frac)
    adm.margin = spy
    try:
        yield box
    finally:
        adm.margin = real


def _limiting(box):
    """(name, peaks) of the headroom minimum in the LAST captured report."""
    if not box:
        return '-', None
    report, limits, f = box[-1]
    if report is None:
        return '-', None
    peaks = (float(report.peak_leg_vel_mmps), float(report.peak_leg_acc_mmps2),
             float(report.peak_leg_jerk_mmps3))
    head = (
        ('vel', (f * limits.leg_vel_mmps - peaks[0]) / (f * limits.leg_vel_mmps)),
        ('acc', (f * limits.leg_acc_mmps2 - peaks[1]) / (f * limits.leg_acc_mmps2)),
        ('jerk', (f * limits.leg_jerk_mmps3 - peaks[2])
         / (f * limits.leg_jerk_mmps3)))
    return min(head, key=lambda h: h[1])[0], peaks


class _Ctx:
    """Limits, geometry, config and rest seed, built once."""

    def __init__(self, wait_xy=DEFAULT_WAIT_XY, limits=None):
        self.geom = StewartGeometry()
        self.limits = (asw.sitting_limits(600.0, 8000.0, 200000.0, 4500.0)
                       if limits is None else limits)
        self.cfg = sg.SegmentConfig(planner_mode='pose')
        self.wait_xy = np.array(wait_xy, dtype=float)
        self.wait_site = st.Site('W', np.array([wait_xy[0], wait_xy[1],
                                                st.CATCH_CUP_Z_MM]))

    def seed(self):
        return nom.rest_state(self.wait_site.rest_site_mm())

    def landing(self, dx_mm, dy_mm):
        return np.array([self.wait_xy[0] + dx_mm, self.wait_xy[1] + dy_mm,
                         st.CATCH_CUP_Z_MM])


_CTX = {}


def _ctx(wait_xy=DEFAULT_WAIT_XY):
    key = (float(wait_xy[0]), float(wait_xy[1]))
    if key not in _CTX:
        _CTX[key] = _Ctx(wait_xy)
    return _CTX[key]


def _terminal(sched):
    """The CatchTerminal the executor builds for this Schedule's only CATCH
    (``executor._catch_terminal`` with no clamp bite, no carried throw)."""
    sk = sched.skills[0]
    lp = sk.landing_prior
    return ex.CatchTerminal(
        landing_mm=lp.pos_mm, landing_vel_mm_s=lp.vel_mm_s,
        t_land_s=float(lp.t_land_abs_s),
        rest_site_mm=sk.site.rest_site_mm(), then_throw=None,
        hold_tilt=sk.hold_tilt, receive_tilt=sk.receive_tilt)


def check_reach(dx_mm: float, dy_mm: float, lead_s: float, apex_m: float = 1.9,
                wait_xy=DEFAULT_WAIT_XY) -> Result:
    """One FIRST-PLAN cell: ``compile_human_catch`` for a Landing offset
    (dx, dy) mm from the wait Site at ``lead_s``, then the executor's own
    ``check_segment`` from REST at W at the Schedule's dispatch instant."""
    c = _ctx(wait_xy)
    sched = sc.compile_human_catch(
        c.landing(dx_mm, dy_mm), arrival_vel_mm_s(apex_m), T_NOW + lead_s,
        T_NOW, DISPATCH_LOOKAHEAD_S)
    sk = sched.skills[0]
    with _capture_reports() as box:
        t0 = time.perf_counter()
        res = ex.check_segment(
            None, c.seed(), sc.CATCH, _terminal(sched), sk.dispatch_s(),
            cfg=c.cfg, limits=c.limits, geom=c.geom,
            reserve_fresh_lead=True)
        wall = time.perf_counter() - t0
        name, peaks = _limiting(box)
    margin = float(res.margin)
    if res.planned:
        ok = bool(margin >= 0.0)
        code = 'OK' if ok else 'MARGIN'
    else:
        ok, code = False, str(res.code)
    return Result(ok, bool(res.planned), code, margin, name, peaks, wall)


def check_reaim(first_mm: float, first_lead_s: float, resend_lead_s: float,
                dx_mm: float, apex_m: float = 1.9,
                wait_xy=DEFAULT_WAIT_XY):
    """One RE-AIM cell.  Install the first plan (displacement ``first_mm`` in
    +x of W, lead ``first_lead_s``), then check a terminal moved a further
    ``dx_mm`` in +x with ``resend_lead_s`` of lead left, against the live
    record.  Returns a :class:`Result`, or None when the FIRST plan itself is
    not installable (the cell does not exist)."""
    c = _ctx(wait_xy)
    vel = arrival_vel_mm_s(apex_m)
    s0 = sc.compile_human_catch(c.landing(first_mm, 0.0), vel,
                                T_NOW + first_lead_s, T_NOW,
                                DISPATCH_LOOKAHEAD_S)
    sk0 = s0.skills[0]
    rec, res0, _seg = ex.install_segment(
        None, c.seed(), sc.CATCH, _terminal(s0), sk0.dispatch_s(),
        cfg=c.cfg, limits=c.limits, geom=c.geom, reserve_fresh_lead=True)
    if not res0.accepted:
        return None
    t_land = T_NOW + first_lead_s
    t_resend = t_land - float(resend_lead_s)
    t1 = ex.CatchTerminal(
        landing_mm=c.landing(first_mm + dx_mm, 0.0), landing_vel_mm_s=vel,
        t_land_s=t_land, rest_site_mm=sk0.site.rest_site_mm(), then_throw=None,
        hold_tilt=None, receive_tilt=(0.0, 0.0))
    with _capture_reports() as box:
        t0 = time.perf_counter()
        res = ex.check_segment(rec, None, sc.CATCH, t1, t_resend, cfg=c.cfg,
                               limits=c.limits, geom=c.geom,
                               reserve_fresh_lead=True)
        wall = time.perf_counter() - t0
        name, peaks = _limiting(box)
    margin = float(res.margin)
    if res.planned:
        ok = bool(margin >= 0.0)
        code = 'OK' if ok else 'MARGIN'
    else:
        ok, code = False, str(res.code)
    return Result(ok, bool(res.planned), code, margin, name, peaks, wall)


# -- the sweeps -------------------------------------------------------------

def directions(n: int):
    return [(k * 360.0 / n) for k in range(int(n))]


def sweep_first(dirs_deg, offsets, leads, apex_m, wait_xy, log=print):
    """{(dir_deg, lead): [(offset_mm, Result), ...]} over the grid.  A row
    stops being solved after TWO consecutive refusals (the cut that keeps the
    sweep affordable on a loaded box): the remaining offsets repeat the last
    refusal, so a re-acceptance beyond two refusals would not be seen."""
    out = {}
    for d in dirs_deg:
        ca, sa = math.cos(math.radians(d)), math.sin(math.radians(d))
        for lead in leads:
            row, misses, last = [], 0, None
            for off in offsets:
                if misses >= 2:      # two refusals in a row: the rest is out
                    row.append((off, last))      # (filled, not solved)
                    continue
                last = check_reach(off * ca, off * sa, lead, apex_m, wait_xy)
                misses = 0 if last.ok else misses + 1
                row.append((off, last))
            out[(d, lead)] = row
            log('  dir %5.1f lead %.2f: %s' % (
                d, lead, ' '.join('%s' % ('.' if r.ok else 'x')
                                  for _o, r in row)))
    return out


def reach_mm(row):
    """The farthest offset up to which every grid offset is accepted (0 when
    even the first refuses)."""
    r = 0.0
    for off, res in row:
        if not res.ok:
            break
        r = float(off)
    return r


def first_refusal(row):
    for off, res in row:
        if not res.ok:
            return off, res
    return None


def sweep_reaim(first_mm, first_lead_s, resend_leads, shifts, apex_m,
                wait_xy, log=print):
    out = {}
    for rl in resend_leads:
        row = []
        for dx in shifts:
            row.append((dx, check_reaim(first_mm, first_lead_s, rl, dx, apex_m,
                                        wait_xy)))
        out[rl] = row
        log('  first %3.0f mm @ %.2f s, re-aim at lead %.2f: %s' % (
            first_mm, first_lead_s, rl,
            ' '.join('?' if r is None else ('.' if r.ok else 'x')
                      for _d, r in row)))
    return out


def format_first(res_by, dirs_deg, leads, apex_m, wait_xy):
    lines = ['### First plan, apex %.2f m (arrival %.2f m/s), wait Site (%.0f, '
             '%.0f) mm' % (apex_m, -arrival_vel_mm_s(apex_m)[2] / 1000.0,
                           wait_xy[0], wait_xy[1]), '',
             'Reach (mm) = farthest grid offset with every nearer offset '
             'accepted.  Limiting = the quantity refusing the first refused '
             'offset (leg vel / acc / jerk headroom minimum; code in '
             'brackets).', '',
             '| lead s | ' + ' | '.join('%g deg' % d for d in dirs_deg)
             + ' | min | limiting at first refusal |',
             '|---|' + '---|' * (len(dirs_deg) + 2)]
    for lead in leads:
        reaches, lim = [], collections.Counter()
        for d in dirs_deg:
            row = res_by[(d, lead)]
            reaches.append(reach_mm(row))
            fr = first_refusal(row)
            if fr is not None:
                lim['%s [%s]' % (fr[1].limiting, fr[1].code)] += 1
        lines.append('| %.2f | %s | %.0f | %s |' % (
            lead, ' | '.join('%.0f' % r for r in reaches), min(reaches),
            ', '.join('%s x%d' % kv for kv in lim.most_common())
            or 'none refused'))
    return lines


def format_cells(res_by, dirs_deg, leads, offsets):
    lines = ['', '#### Worst margin per cell (offset mm; min over directions; '
             '`x` = refused)', '',
             '| lead s | ' + ' | '.join('%g' % o for o in offsets) + ' |',
             '|---|' + '---|' * len(offsets)]
    for lead in leads:
        cells = []
        for k, off in enumerate(offsets):
            ms = [res_by[(d, lead)][k][1] for d in dirs_deg]
            worst = min(ms, key=lambda r: (-1e9 if math.isnan(r.margin)
                                           else r.margin))
            nref = sum(1 for r in ms if not r.ok)
            tag = '' if nref == 0 else ' x%d' % nref
            cells.append('%.2f%s' % (worst.margin, tag)
                         if not math.isnan(worst.margin) else 'nan' + tag)
        lines.append('| %.2f | %s |' % (lead, ' | '.join(cells)))
    return lines


def format_reaim(res_by, first_mm, first_lead_s, shifts):
    lines = ['', '### Re-aim (gate only, fences not applied), first plan %.0f '
             'mm at lead %.2f s' % (first_mm, first_lead_s), '',
             'Cell = margin of the re-solve against the live record; `x` = '
             'refused with its limiting quantity.  Reach = largest shift with '
             'every smaller shift accepted.', '',
             '| lead left s | ' + ' | '.join('%d mm' % d for d in shifts)
             + ' | reach mm | limiting at first refusal |',
             '|---|' + '---|' * (len(shifts) + 2)]
    for rl, row in sorted(res_by.items()):
        cells, reach, lim = [], 0.0, None
        broken = False
        for dx, r in row:
            if r is None:
                cells.append('n/a')
                broken = True
                continue
            cells.append('%.2f' % r.margin if r.ok else
                         'x %s [%s]' % (r.limiting, r.code))
            if not r.ok and lim is None:
                lim = '%s [%s]' % (r.limiting, r.code)
                broken = True
            if not broken:
                reach = float(dx)
        lines.append('| %.2f | %s | %.0f | %s |' % (
            rl, ' | '.join(cells), reach, lim or 'none refused'))
    return lines


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--dirs', type=int, default=8)
    ap.add_argument('--step', type=float, default=25.0)
    ap.add_argument('--max-mm', type=float, default=300.0)
    ap.add_argument('--leads', default=','.join('%.2f' % x for x in LEADS))
    ap.add_argument('--apexes', default='1.9')
    ap.add_argument('--wait-xy', default='%g,%g' % DEFAULT_WAIT_XY)
    ap.add_argument('--no-reaim', action='store_true')
    ap.add_argument('--reaim-first', default='0,100')
    ap.add_argument('--reaim-first-lead', type=float, default=0.70)
    ap.add_argument('--reaim-leads', default=','.join(
        '%.2f' % x for x in REAIM_LEADS))
    ap.add_argument('--reaim-mm', default=','.join(str(x) for x in REAIM_MM))
    ap.add_argument('--out', default=None)
    a = ap.parse_args(argv)
    wait_xy = tuple(float(x) for x in a.wait_xy.split(','))
    leads = [float(x) for x in a.leads.split(',')]
    offsets = [a.step * k for k in range(0, int(round(a.max_mm / a.step)) + 1)]
    dirs_deg = directions(a.dirs)
    md = ['# human_catch reach sweep', '',
          'Limits: leg 600 mm/s, 8000 mm/s^2, 200000 mm/s^3, Hand 4500 '
          'rev/s^2 (generated launch values; margin fraction %.2f).  Planner '
          'mode pose.  Offsets %s mm; %d directions.' % (
              adm.MARGIN_FRAC, offsets, a.dirs), '']
    t0 = time.time()
    for apex in (float(x) for x in a.apexes.split(',')):
        print('first plan, apex %.2f' % apex)
        res = sweep_first(dirs_deg, offsets, leads, apex, wait_xy)
        md += format_first(res, dirs_deg, leads, apex, wait_xy)
        md += format_cells(res, dirs_deg, leads, offsets)
        walls = [r.wall_s for row in res.values() for _o, r in row]
        md += ['', 'Check wall (in-process): p50 %.0f ms, p95 %.0f ms, max %.0f '
               'ms over %d cells.' % (1e3 * np.percentile(walls, 50),
                                      1e3 * np.percentile(walls, 95),
                                      1e3 * max(walls), len(walls)), '']
    if not a.no_reaim:
        shifts = [float(x) for x in a.reaim_mm.split(',')]
        rleads = [float(x) for x in a.reaim_leads.split(',')]
        for first in (float(x) for x in a.reaim_first.split(',')):
            print('re-aim, first plan %.0f mm' % first)
            rs = sweep_reaim(first, a.reaim_first_lead, rleads, shifts, 1.9,
                             wait_xy)
            md += format_reaim(rs, first, a.reaim_first_lead, shifts)
    md += ['', 'Sweep wall %.0f s.' % (time.time() - t0)]
    text = '\n'.join(md) + '\n'
    if a.out:
        with open(a.out, 'w') as fh:
            fh.write(text)
    print(text)
    return 0


if __name__ == '__main__':
    sys.exit(main())
