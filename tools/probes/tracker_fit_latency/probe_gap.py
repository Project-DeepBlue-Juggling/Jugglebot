"""Synthetic gap driver: a 4 m/s toss, 3.4 ms frames, one dropped run of g seconds starting at G0.

Recipe (repo root, project venv; compare against the pre-fix matcher by running it on a checkout of HEAD):
  python tools/probes/tracker_fit_latency/probe_gap.py 0.03
Prints, per gap in (20, 30, 36, 50, 80 ms), the first-fit time and final tracking state.
Result 2026-10-16 (pre-fix matcher, G0 = 0.03): first fit 0.112 s at a 20-30 ms gap, 0.238 s at 36 ms,
0.340 s at 50 ms, 0.422 s at 80 ms. (G0 value is the entry's "one dropped run from t = 30 ms"; not
separately recorded in the session transcript.)"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'ros_ws' / 'src' / 'jugglebot'))
import numpy as np
from jugglebot.tracking.ballistics import GRAVITY_MMPS2
from jugglebot.tracking.matcher import BallTracker
from jugglebot.tracking.ball import TrackingConfidence
CUP = np.array([-50.0, 0.0, 860.0]); VEL = np.array([0.0, 0.0, 4000.0])
def pos(t):
    p = CUP + VEL * t; p[2] -= 0.5 * GRAVITY_MMPS2 * t * t; return p
import sys
G0 = float(sys.argv[1])
for gap in (0.020, 0.030, 0.036, 0.050, 0.080):
    tr = BallTracker(dt=0.005, landing_z=830.0, announced_gate_mm=200.0)
    bid = tr.handle_announcement(CUP.copy(), VEL.copy(), 0.0, source='jugglebot', destination='jugglebot')
    t = 0.0; ts = []
    while t < 0.80:
        ts.append(t); t += 0.0034
    # drop frames: a single gap of `gap` s starting at 0.15 s (ball rising ~2.5 m/s)
    ts = [x for x in ts if not (G0 < x < G0 + gap)]
    first_fit = None
    for x in ts[1:]:
        tr.process_frame([pos(x)], x)
        b = tr.get_ball(bid)
        if first_fit is None and b.landing_from_fit: first_fit = x
    b = tr.get_ball(bid)
    print('gap %.3f  tracking %s  frames_wo_meas %d  fit samples %d  first fit %s  landing_from_fit(end) %s' % (
        gap, b.tracking.name, b.frames_without_measurement, len(tr._flight_fits.get(bid, [])), first_fit, b.landing_from_fit))
