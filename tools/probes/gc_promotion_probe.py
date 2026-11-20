"""BUG-58 offline probe: what a stream node's gen-2 collection walks, what
promotes objects into gen 2 during an Attempt, and how big one installed
Segment's object graph is.

Why: gen-2 pauses of 47-95 ms in ``teensy_bridge_node`` and ``trajectory_node``
precede Attempt-ending firmware STOPs (BUG-58, Sittings 2026-11-13/18/20).  In
CPython 3.8 a full collection fires only when gen-1 has run 10 times AND the
objects promoted into gen 2 since the last full pass exceed a quarter of the
gen-2 population (``long_lived_pending > long_lived_total / 4``), and it walks
the WHOLE gen-2 population.  So two numbers decide the pause: the population
(cost) and the promotion rate (cadence).  This probe measures both offline.

Modes (Python 3.8, the nodes' interpreter; ROS sourced for ``population``):

* ``population <module>`` -- import a node module (no ``rclpy.init``, no node
  built, nothing bound or opened), then report the tracked-object count, the
  type histogram, the top packages by function count, and the median
  ``gc.collect(2)`` wall time.  A lower bound on the live node (init adds
  ~3-10 k objects: compare the node's own ``gc_freeze ON: ... froze N`` line).
* ``attempt`` -- run ``sim/skills_gate.py``'s ``--learn --pattern oval_1ball``
  seed 0 (MuJoCo, about 2-6 min under tracemalloc) with a ``gc.callbacks``
  hook that counts, at every gen-1 pass, the objects it promoted into gen 2
  and tallies their allocation site (tracemalloc) and type; and wraps
  ``executor.install_segment`` to pickle each accepted install's
  ``(new_record, warm_start)`` exactly as the solver worker ships it and count
  the tracked objects the node-side ``pickle.loads`` creates.  NOTE the sim runs
  planner and emitter in ONE process; on the robot the solve runs in the
  solver-worker process and only the unpickled reply lands in
  ``trajectory_node``, so read the site table by file, not as a node total.

Output: a text report on stdout and ``temp/probes/gc_promotion_<ts>.json``.

Logbook: the BUG-58 Item (map/bug-40-hz-setpoint-stream-starved-past-the-
firmware-s-70-ms.md); no test depends on it.
"""
from __future__ import annotations

import collections
import gc
import json
import os
import pickle
import statistics
import sys
import time

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))


def _collect_ms(n=7):
    ts = []
    for _ in range(n):
        t = time.perf_counter()
        gc.collect(2)
        ts.append((time.perf_counter() - t) * 1e3)
    return statistics.median(ts)


def _pkg(obj):
    m = getattr(obj, '__module__', None) or '?'
    return m.split('.')[0] if not m.startswith('jugglebot') else '.'.join(m.split('.')[:3])


def population(mod: str) -> dict:
    t0 = time.perf_counter()
    __import__(mod)
    t_imp = time.perf_counter() - t0
    gc.collect()
    objs = gc.get_objects()
    types = collections.Counter(type(o).__name__ for o in objs)
    fn_pkgs = collections.Counter(_pkg(o) for o in objs if type(o).__name__ == 'function')
    out = {'module': mod, 'import_s': round(t_imp, 2), 'tracked': len(objs),
           'modules': len(sys.modules), 'collect2_ms_median': round(_collect_ms(), 2),
           'top_types': types.most_common(12), 'functions_by_pkg': fn_pkgs.most_common(15)}
    del objs
    return out


class _PromotionTally:
    """gc.callbacks hook: at each gen-1 pass, objects promoted into gen 2 =
    (len gen0 + len gen1 at start) - collected; they are the TAIL of the gen-2
    list after the pass (gc_list_merge appends)."""

    def __init__(self, tracemalloc_mod):
        self.tm = tracemalloc_mod
        self._young = 0
        self._t0 = 0.0
        self.promoted = 0
        self.sites = collections.Counter()
        self.types = collections.Counter()
        self.full = []          # (t, pause_ms, gen2 size)
        self.gen1 = 0
        self.timeline = []      # (t, cumulative promoted)
        self.t_origin = time.perf_counter()

    def __call__(self, phase, info):
        g = info.get('generation', -1)
        if phase == 'start':
            self._t0 = time.perf_counter()
            if g == 1:
                self._young = len(gc.get_objects(0)) + len(gc.get_objects(1))
            return
        dt_ms = (time.perf_counter() - self._t0) * 1e3
        now = time.perf_counter() - self.t_origin
        if g == 2:
            self.full.append((round(now, 2), round(dt_ms, 2), len(gc.get_objects(2))))
            self.timeline.append((round(now, 2), self.promoted, 'FULL'))
        elif g == 1:
            self.gen1 += 1
            n = max(0, self._young - int(info.get('collected', 0)))
            self.promoted += n
            self.timeline.append((round(now, 2), self.promoted, ''))
            if n:
                tail = gc.get_objects(2)[-n:]
                for o in tail:
                    self.types[type(o).__name__] += 1
                    tb = self.tm.get_object_traceback(o)
                    self.sites[_site(tb)] += 1
                del tail


def _site(tb) -> str:
    if tb is None:
        return '<untraced>'
    # most-recent-first: the first frame in repo code (not this probe)
    for fr in reversed(tb):
        fn = fr.filename
        if 'gc_promotion_probe' in fn:
            continue
        if any(k in fn for k in ('/jugglebot/', '/sim/', '/controller/', '/teensy_link/')):
            rel = fn.split('/jugglebot/jugglebot/')[-1] if '/jugglebot/jugglebot/' in fn else os.path.relpath(fn, ROOT)
            return '%s:%d' % (rel, fr.lineno)
    fr = tb[-1]
    return 'ext %s:%d' % (os.path.basename(fr.filename), fr.lineno)


def attempt(max_s: float) -> dict:
    import tracemalloc
    sys.path[:0] = [os.path.join(ROOT, 'ros_ws', 'src', 'jugglebot'), ROOT]
    from jugglebot.motion.skills import executor as ex
    from sim import skills_gate as sg

    installs = []
    orig = ex.install_segment

    def wrapped(*a, **kw):
        rec, res, seg = orig(*a, **kw)
        if rec is not None and getattr(res, 'accepted', False):
            blob = pickle.dumps((rec, getattr(seg, 'warm_start', None)),
                                protocol=pickle.HIGHEST_PROTOCOL)
            # count what pickle.loads CREATES (a referent walk would escape
            # through classes into module globals)
            enabled = gc.isenabled()
            gc.disable()
            n0 = len(gc.get_objects(0))
            t = time.perf_counter()
            back = pickle.loads(blob)
            t_load = (time.perf_counter() - t) * 1e3
            created = len(gc.get_objects(0)) - n0
            if enabled:
                gc.enable()
            installs.append((len(blob), created, round(t_load, 3),
                             int(getattr(rec.plan, 'n', 0) or len(getattr(rec.plan, 'hand_rev', ())))))
            del back
        return rec, res, seg
    ex.install_segment = wrapped
    for m in list(sys.modules.values()):          # rebind `from executor import install_segment`
        if m is not None and getattr(m, 'install_segment', None) is orig and m is not ex:
            m.install_segment = wrapped

    gc.collect()
    tracemalloc.start(12)
    tally = _PromotionTally(tracemalloc)
    gc.callbacks.append(tally)
    t0 = time.perf_counter()
    cfg = sg.SelfTossGateConfig(pattern='oval_1ball')
    try:
        sg.run_learn(cfg, seeds=(0,))
    finally:
        gc.callbacks.remove(tally)
    wall = time.perf_counter() - t0
    tracemalloc.stop()
    sizes = [i[0] for i in installs]
    graph = [i[1] for i in installs]
    return {
        'wall_s': round(wall, 1), 'gen1_passes': tally.gen1, 'promoted_total': tally.promoted,
        'promoted_per_s': round(tally.promoted / wall, 1),
        'full_collections': tally.full,
        'top_promotion_sites': tally.sites.most_common(25),
        'top_promoted_types': tally.types.most_common(15),
        'installs': len(installs),
        'install_pickle_bytes_median': statistics.median(sizes) if sizes else None,
        'install_tracked_objects_median': statistics.median(graph) if graph else None,
        'install_tracked_objects_max': max(graph) if graph else None,
        'install_unpickle_ms_median': statistics.median([i[2] for i in installs]) if installs else None,
        'install_knots_median': statistics.median([i[3] for i in installs]) if installs else None,
        'timeline_tail': tally.timeline[-5:],
    }


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if not argv or argv[0] not in ('population', 'attempt'):
        print(__doc__)
        return 2
    rep = population(argv[1]) if argv[0] == 'population' else attempt(600.0)
    out_dir = os.path.join(ROOT, 'temp', 'probes')
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'gc_promotion_%s_%s.json' % (argv[0], time.strftime('%Y%m%d-%H%M%S')))
    with open(path, 'w') as f:
        json.dump(rep, f, indent=1, default=str)
    print(json.dumps(rep, indent=1, default=str))
    print('wrote', path)
    return 0


if __name__ == '__main__':
    sys.exit(main())
