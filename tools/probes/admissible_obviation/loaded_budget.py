#!/usr/bin/env python3
"""Wall-clock budget of the command-time admissibility check, on THIS box, under
whatever load is present when it runs (learner-generalisation P3a, plan § 4).

What it measures, per condition label (L0 idle, L1 Sitting-loaded, L2 Sitting-loaded
plus a parallel gate, L3 the launch's ``cpu_partition``):

(a) ONE columns catch-with-throw plan + gate in a ``WorkerSegmentSolver`` child
    started exactly as ``trajectory_node`` starts it (``os.posix_spawn``, BLAS pinned
    to one thread, never an in-process solve). ``--kind install`` sends the production
    ``install_segment`` request (plan + gate + the install bookkeeping: an upper bound
    for a check); ``--kind check`` sends the P3a CHECK request (``SegmentSolveRequest(check=True)``:
    plan + gate, no install) and also records the gate margin.
(b) A burst of ``--burst`` solves back to back, timed end to end: the refused path
    (candidate, nominal, confirm) as the caller waits for it.
(c) Lateness of a 25 ms absolute-grid thread in THIS process while the solves run:
    ``tests/motion/test_solver_worker_lateness.py``'s metric (end of tick work minus
    the grid instant), the emitter's freedom from the solve.
(d) Load average before and after, power mode, governors, frequencies, temperatures.
(e) Per solve, the worker's own wall (``reply.solve_s``), CPU seconds and voluntary /
    involuntary context switches (the JB-76 fields on ``SegmentSolveReply``): CPU near
    wall is a slow solve, CPU far below wall with involuntary switches is a starved one.

Samples are paced (``--period``) so the worker goes idle between them, as it does
between beats; a back-to-back loop keeps caches and the governor warm and reads low.

usage: python tools/probes/admissible_obviation/loaded_budget.py --label L1
          [--n 300] [--burst 3] [--period 0.25] [--kind install|check]
          [--limits VEL ACC JERK HAND_ACC] [--mode pose] [--apex 0.95]
          [--cpus 4,5] [--rt-priority 0]

Offline: no ROS, no hardware, nothing commanded. Output under
``temp/probes/admissible_obviation/loaded_budget_<label>_<stamp>.json``.
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import os
import subprocess
import sys
import threading
import time

for _v in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS',
           'NUMEXPR_NUM_THREADS'):
    os.environ[_v] = '1'
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'ros_ws', 'src', 'jugglebot'))
sys.path.insert(0, os.path.join(ROOT, 'tools'))
import numpy as np                                                    # noqa: E402
import admissible_sweep as asw                                        # noqa: E402
from jugglebot.motion.geometry import StewartGeometry                 # noqa: E402
from jugglebot.motion.skills import executor as ex                    # noqa: E402
from jugglebot.motion.skills import schedule as sc                    # noqa: E402
from jugglebot.motion.skills import segments as sg                    # noqa: E402
from jugglebot.motion.skills import sites as st                       # noqa: E402
from jugglebot.motion.skills import solver_worker as sw               # noqa: E402
from jugglebot.motion.trajectory import ballistics_bc as bal          # noqa: E402

DWELL = 0.27
PERIOD = 0.025
FAR_S = float('inf')
#: Landing offsets (mm) cycled over the samples: the model command and the
#: Learner's authority ring, so the timing is not one plan repeated.
OFFSETS_MM = ((0.0, 0.0), (10.0, 0.0), (0.0, 10.0), (-10.0, -10.0), (20.0, 0.0),
              (0.0, -20.0), (-20.0, 20.0), (30.0, 0.0), (0.0, 30.0), (-30.0, -30.0))


def _read(path):
    try:
        with open(path) as fh:
            return fh.read().strip()
    except OSError:
        return None


def _cmd(argv):
    try:
        return subprocess.run(argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                              timeout=10).stdout.decode(errors='replace').strip()
    except Exception as exc:    # noqa: BLE001 - environment facts are best effort
        return 'unavailable: %s' % (exc,)


def environment():
    cpus = sorted(glob.glob('/sys/devices/system/cpu/cpu[0-9]*/cpufreq'))
    zones = sorted(glob.glob('/sys/class/thermal/thermal_zone*'))
    return dict(
        time=time.strftime('%Y-%m-%dT%H:%M:%S%z'),
        loadavg=_read('/proc/loadavg'),
        nvpmodel=_cmd(['nvpmodel', '-q']),
        online=_read('/sys/devices/system/cpu/online'),
        governor=[_read(os.path.join(c, 'scaling_governor')) for c in cpus],
        cur_khz=[_read(os.path.join(c, 'scaling_cur_freq')) for c in cpus],
        max_khz=[_read(os.path.join(c, 'scaling_max_freq')) for c in cpus],
        temps_mC={_read(os.path.join(z, 'type')): _read(os.path.join(z, 'temp'))
                  for z in zones},
        stack=_cmd(['pgrep', '-af',
                    'jugglebot_launch|trajectory_node|skill_node|ros2 bag|'
                    'run_tests.sh|gui_synthetic_stack']),
        python=sys.version.split()[0], numpy=np.__version__)


class Grid:
    """The 25 ms absolute-grid thread of ``test_solver_worker_lateness._measure``."""

    def __init__(self):
        self.stop = threading.Event()
        self.ticks = []
        self.th = threading.Thread(target=self._run, daemon=True)

    def _run(self):
        nd = (time.perf_counter() // PERIOD + 1) * PERIOD
        while not self.stop.is_set():
            np.polyval(np.arange(6.0), 0.3)
            self.ticks.append(time.perf_counter() - nd)
            nd += PERIOD
            dt = nd - time.perf_counter()
            if dt > 0:
                self.stop.wait(dt)
            else:
                nd = (time.perf_counter() // PERIOD + 1) * PERIOD

    def __enter__(self):
        self.th.start()
        time.sleep(0.2)
        return self

    def __exit__(self, *exc):
        time.sleep(0.1)
        self.stop.set()
        self.th.join()

    def lateness_ms(self):
        return np.array(self.ticks[2:]) * 1e3


def stats(x):
    x = np.asarray(x, dtype=float)
    if x.size == 0:
        return dict(n=0)
    return dict(n=int(x.size), p50=float(np.percentile(x, 50)),
                p90=float(np.percentile(x, 90)), p99=float(np.percentile(x, 99)),
                max=float(x.max()), mean=float(x.mean()))


class Chain:
    """The columns chain the check plans on: a launch THROW from rest at the partner
    Site (installed through the worker, so its record is the streaming Segment), then
    catch-with-throw requests at the other Site, dispatched ``HANDOFF_LEAD_S`` before
    that release exactly as the Schedule dispatches them."""

    def __init__(self, solver, a, limits, geom, cfg):
        self.solver, self.limits, self.geom, self.cfg = solver, limits, geom, cfg
        self.kind = a.kind
        self.site = st.Site('S', np.array([62.5, 0.0, st.CATCH_CUP_Z_MM]))
        self.partner = st.Site('Q', np.array([-62.5, 0.0, st.CATCH_CUP_Z_MM]))
        self.T = sc.flight_s(a.apex)
        self.tau = sc.transit_s(self.T, DWELL)
        ok, code, _seed, takeoff = asw._throw_cell(
            self.partner, self.partner, self.T, asw.SINGLE_SITE_LAUNCH_S, limits, geom,
            cfg)
        if not ok:
            raise SystemExit('seed throw refused in-process: %s' % (code,))
        self.v_arrival = bal.arrival_velocity(takeoff, self.T)
        self.t0 = time.perf_counter() + 3600.0     # far future: nothing reads late
        self.t_rel = self.t0 + asw.SINGLE_SITE_LAUNCH_S
        self.seq = 0
        first = solver.solve(sw.SegmentSolveRequest(
            seq=self._next(), record=None,
            seed_rest=asw._rest_state(self.partner.rest_site_mm()), kind=sg.THROW,
            terminal=sg.ThrowTerminal(site_mm=self.partner.throw_site_mm(),
                                      target_mm=self.partner.throw_site_mm(),
                                      flight_s=self.T, t_release_s=self.t_rel),
            t_now_s=self.t0, lead_s=ex.LEAD_S, cfg=cfg, limits=limits, geom=geom,
            warm_start=None, reserve_fresh_lead=False, deadline_s=FAR_S))
        if not first.result.accepted:
            raise SystemExit('launch THROW refused through the worker: %s %s'
                             % (first.result.code, first.result.message))
        self.record, self.warm = first.new_record, first.warm_start

    def _next(self):
        self.seq += 1
        return self.seq

    def request(self, off_mm):
        landing = self.site.catch_site_mm() + np.array([off_mm[0], off_mm[1], 0.0])
        t_land = self.t_rel + self.tau
        then = sg.ThrowAfterCatch(t_release_s=t_land + DWELL,
                                  site_mm=self.site.throw_site_mm(),
                                  target_mm=landing, flight_s=self.T)
        term = sg.CatchTerminal(landing_mm=landing, landing_vel_mm_s=self.v_arrival,
                                t_land_s=t_land, rest_site_mm=self.site.rest_site_mm(),
                                then_throw=then)
        return sw.SegmentSolveRequest(
            seq=self._next(), record=self.record, seed_rest=None, kind=sg.CATCH,
            terminal=term, t_now_s=self.t_rel - sc.HANDOFF_LEAD_S, lead_s=ex.LEAD_S,
            cfg=self.cfg, limits=self.limits, geom=self.geom, warm_start=self.warm,
            reserve_fresh_lead=False, deadline_s=FAR_S,
            check=self.kind == 'check')

    def solve(self, off_mm):
        """One request; returns (client wall ms, reply)."""
        req = self.request(off_mm)
        t = time.perf_counter()
        rep = self.solver.solve(req)
        return (time.perf_counter() - t) * 1e3, rep


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--label', required=True)
    ap.add_argument('--n', type=int, default=300)
    ap.add_argument('--burst', type=int, default=3)
    ap.add_argument('--period', type=float, default=0.25)
    ap.add_argument('--kind', default='install', choices=('install', 'check'))
    ap.add_argument('--limits', type=float, nargs=4,
                    default=[600.0, 8000.0, 200000.0, 4500.0])
    ap.add_argument('--mode', default='pose', choices=('level', 'pose'))
    ap.add_argument('--apex', type=float, default=0.95)
    ap.add_argument('--cpus', default='', help='pin the worker, e.g. 4,5 (the '
                    'launch cpu_partition solver set); default unpinned')
    ap.add_argument('--rt-priority', type=int, default=0)
    ap.add_argument('--note', default='')
    a = ap.parse_args(argv)
    geom = StewartGeometry()
    limits = asw._limits(*a.limits)
    cfg = sg.SegmentConfig(planner_mode=a.mode)
    env0 = environment()
    cpus = [int(c) for c in a.cpus.split(',') if c.strip()] or None
    solver = sw.WorkerSegmentSolver(env=dict(os.environ), rt_priority=a.rt_priority,
                                    cpus=cpus)
    out = dict(label=a.label, note=a.note, args=vars(a), env_before=env0)
    try:
        if not solver.wait_ready(30.0):
            raise SystemExit('worker never READY: %s' % (solver.state,))
        out['worker'] = dict(pid=solver.pid, ready=str(solver.ready_info),
                             sched=str(solver.sched_info),
                             affinity=sorted(os.sched_getaffinity(solver.pid)))
        chain = Chain(solver, a, limits, geom, cfg)
        for off in OFFSETS_MM[:3]:                  # warm the catch path
            _ms, rep = chain.solve(off)
            if not rep.result.accepted:
                raise SystemExit('warm-up catch-with-throw refused at %r: %s %s'
                                 % (off, rep.result.code, rep.result.message))
        single = dict(wall_ms=[], solve_ms=[], cpu_ms=[], vcsw=[], ivcsw=[])
        margins = []                                # --kind check only
        burst = dict(wall_ms=[], cpu_ms=[], ivcsw=[])
        refused = []
        with Grid() as grid:
            for i in range(a.n):                    # (a) one plan + gate
                t_next = time.perf_counter() + a.period
                ms, rep = chain.solve(OFFSETS_MM[i % len(OFFSETS_MM)])
                if not rep.result.accepted or (
                        a.kind == 'check' and not rep.margin >= 0.0):
                    refused.append((i, rep.result.code))
                if a.kind == 'check':
                    margins.append(rep.margin)
                single['wall_ms'].append(ms)
                single['solve_ms'].append(rep.solve_s * 1e3)
                single['cpu_ms'].append(rep.cpu_s * 1e3)
                single['vcsw'].append(rep.vcsw)
                single['ivcsw'].append(rep.ivcsw)
                time.sleep(max(0.0, t_next - time.perf_counter()))
            for i in range(a.n):                    # (b) the refused path
                t_next = time.perf_counter() + max(a.period, 0.5)
                t = time.perf_counter()
                cpu = 0.0
                iv = 0
                for j in range(a.burst):
                    _ms, rep = chain.solve(OFFSETS_MM[(i + 3 * j) % len(OFFSETS_MM)])
                    if not rep.result.accepted:
                        refused.append((a.n + i, rep.result.code))
                    cpu += rep.cpu_s
                    iv += rep.ivcsw
                burst['wall_ms'].append((time.perf_counter() - t) * 1e3)
                burst['cpu_ms'].append(cpu * 1e3)
                burst['ivcsw'].append(iv)
                time.sleep(max(0.0, t_next - time.perf_counter()))
        late = grid.lateness_ms()
    finally:
        solver.close()
    out['env_after'] = environment()
    out['refused'] = refused
    out['single'] = {k: stats(v) for k, v in single.items()}
    out['burst'] = {k: stats(v) for k, v in burst.items()}
    out['emitter_lateness_ms'] = stats(late)
    if a.kind == 'check':
        mg = np.asarray(margins, dtype=float)
        good = mg[~np.isnan(mg)]
        out['margin'] = dict(n=int(mg.size), nan=int(mg.size - good.size),
                             min=float(good.min()) if good.size else float('nan'),
                             p50=float(np.median(good)) if good.size else float('nan'))
    out['raw'] = dict(single=single, burst=burst)
    path = os.path.join(ROOT, 'temp', 'probes', 'admissible_obviation',
                        'loaded_budget_%s_%s.json' % (a.label,
                                                      time.strftime('%Y%m%d_%H%M%S')))
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as fh:
        json.dump(out, fh, indent=1, default=str)
    s, b, e = out['single'], out['burst'], out['emitter_lateness_ms']
    print('%s kind=%s n=%d refused=%d | one plan+gate wall p50 %.1f p99 %.1f max %.1f ms'
          ' (worker solve p99 %.1f, cpu p50 %.1f p99 %.1f, ivcsw p99 %.0f max %.0f) | '
          '%d-solve burst p50 %.1f p99 %.1f max %.1f ms | emitter lateness p99 %.2f '
          'max %.2f ms (n %d) | load %s -> %s'
          % (a.label, a.kind, a.n, len(refused), s['wall_ms']['p50'], s['wall_ms']['p99'],
             s['wall_ms']['max'], s['solve_ms']['p99'], s['cpu_ms']['p50'],
             s['cpu_ms']['p99'], s['ivcsw']['p99'], s['ivcsw']['max'], a.burst,
             b['wall_ms']['p50'], b['wall_ms']['p99'], b['wall_ms']['max'], e['p99'],
             e['max'], e['n'], (env0['loadavg'] or '').split(' ')[0:3],
             (out['env_after']['loadavg'] or '').split(' ')[0:3]))
    if a.kind == 'check':
        print('%s check margin min %.4f p50 %.4f (nan %d of %d)'
              % (a.label, out['margin']['min'], out['margin']['p50'],
                 out['margin']['nan'], out['margin']['n']))
    print('wrote', path)
    return 0 if not refused else 1


if __name__ == '__main__':
    sys.exit(main())
