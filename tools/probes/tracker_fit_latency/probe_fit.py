"""Per-ball flight-fit convergence through the real BallTracker on a bag (full-frame replay).

Recipe (run from the repo root, project venv):
  python tools/probes/tracker_fit_latency/probe_fit.py <bag> new temp/probes/tracker_fit_latency/fit_S16_new.json
  python tools/probes/tracker_fit_latency/probe_fit.py <bag> old temp/probes/tracker_fit_latency/fit_S16_old.json
Inputs: a bag (S16 = ~/Desktop/rosbags/2026-10-16_10-50-42, S8 = ~/Desktop/rosbags/2026-10-13_23-12-50_2-Ball-32-Catches); 'new' = the working-tree matcher,
'old' = the pre-5974d8c3 tracking package, which must first be exported to
temp/probes/tracker_fit_latency/oldtrack/tracking_old (git archive 5974d8c3^ of
ros_ws/src/jugglebot/jugglebot/tracking, renamed tracking_old).
Output: JSON per announced ball (source, throw_time, announced landing, t_conf, F = first
landing_from_fit frame, fit sample log, identity events).
Result 2026-10-16: S16, 123,524 frames, 212 announcements: self-toss landing - F p10/p50/p90
768/772/780 ms, 36/191 never fitted, identical for both matchers."""
import sys, json
from pathlib import Path
import numpy as np

SCR = Path(__file__).resolve().parents[3] / 'temp' / 'probes' / 'tracker_fit_latency'
REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / 'tools' / 'probes'))
sys.path.insert(0, str(REPO / 'ros_ws' / 'src' / 'jugglebot'))
import tracker_bag_replay as tbr  # noqa
from jugglebot.tracking.ball import TrackingConfidence, BallStatus  # noqa

bag, which, out = sys.argv[1], sys.argv[2], sys.argv[3]
if which == 'old':
    sys.path.insert(0, str(SCR / 'oldtrack'))
    import importlib
    # build the old package under the jugglebot namespace so relative imports work
    import types
    import jugglebot
    spec_dir = SCR / 'oldtrack' / 'tracking_old'
    import importlib.util
    pkg_spec = importlib.util.spec_from_file_location(
        'jugglebot.tracking_old', spec_dir / '__init__.py',
        submodule_search_locations=[str(spec_dir)])
    pkg = importlib.util.module_from_spec(pkg_spec)
    sys.modules['jugglebot.tracking_old'] = pkg
    pkg_spec.loader.exec_module(pkg)
    import jugglebot.tracking_old.matcher as M
    import jugglebot.tracking_old.flight_fit as FF
    tbr.BallTracker = M.BallTracker
    from jugglebot.tracking_old.ball import TrackingConfidence, BallStatus  # noqa
else:
    import jugglebot.tracking.matcher as M
    import jugglebot.tracking.flight_fit as FF

LOG = {}   # id(fit) -> list
_orig_add = FF.FlightFit.add
def add(self, t, pos):
    n0 = len(self._t)
    _orig_add(self, t, pos)
    LOG.setdefault(id(self), []).append((float(t), float(pos[2]), len(self._t) > n0,
                                         [float(v) for v in pos]))
FF.FlightFit.add = add

tracker = tbr.make_tracker()
tbr.make_tracker = lambda: tracker
rec = {}
fitid = {}
orig_pf = tracker.process_frame
def pf(positions, t, labels=None):
    balls = orig_pf(positions, t, labels)
    for b in balls:
        r = rec.setdefault(b.id, dict(id=b.id, source=b.source, throw_time=b.throw_time,
                                      t_conf=None, F=None, first_landing=None, status=None,
                                      n_conf_frames=0, ann_landing=None))
        if b.tracking == TrackingConfidence.CONFIRMED:
            r['n_conf_frames'] += 1
            if r['t_conf'] is None:
                r['t_conf'] = t
        if r['F'] is None and getattr(b, 'landing_from_fit', False) and b.status == BallStatus.IN_FLIGHT:
            r['F'] = t
            r['F_landing'] = float(b.landing_time)
        if b.status == BallStatus.IN_FLIGHT and getattr(b, 'landing_from_fit', False):
            r['last_fit_landing'] = float(b.landing_time)
        r['status'] = b.status.name
        f = tracker._flight_fits.get(b.id)
        if f is not None:
            fitid.setdefault(b.id, set()).add(id(f))
    return balls
tracker.process_frame = pf
rows, stats, records, order, _ = tbr.replay(tbr._find_mcap(Path(bag)), 0.25)
for bid in order:
    r = rec.get(bid)
    if r is None:
        continue
    r['ann_landing'] = records[bid].ann_landing_time
    r['fits'] = [LOG.get(i, []) for i in sorted(fitid.get(bid, ()))]
ev = [dict(kind=e['kind'], ball_id=e['ball_id'], t=e['t'], dt_throw=e['dt_throw'])
      for e in getattr(tracker, 'identity_events', [])]
json.dump(dict(balls=[rec[b] for b in order if b in rec], stats=stats), open(out, 'w'))
print(stats, len(order))
