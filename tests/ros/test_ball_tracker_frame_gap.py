"""A CONFIRMED ball survives a run of dropped mocap frames (2026-10-16).

The live `ball_tracker_node` processes only ~1/3 of QTM's ~290 Hz frames under
sitting load (2026-10-16 sitting: /balls published for 31-34 % of the in-flight
/mocap_data frames, gaps p90 18 ms / p99 33 ms). The Kalman predict used to be
capped at 4 x nominal dt (20 ms), so across a 36 ms gap the filter stepped 20 ms
and was left (36 - 20) ms x ~3.7 m/s ~ 60 mm behind a rising ball -- outside the
50 mm association gate. It then coasted parallel to the ball, re-associated only
where the two paths happened to cross (near the apex, if at all), and the flight
fit got no samples meanwhile. Logbook:
2026-10-16-tracker-flight-fit-late-since-matcher-fix.

Driver recipe (probed 2026-10-16, `/tmp`-style scratch probe, deterministic):
a 4 m/s vertical self-toss from z=860, frames every 3.4 ms (QTM's rate), ONE run
of frames dropped from t=0.030 s for `gap` s. Before the fix the first
`landing_from_fit` frame moved 0.112 s -> 0.238 s (gap 36 ms), 0.340 s (50 ms),
0.422 s (80 ms); a 20-30 ms gap left it at 0.112 s.
"""
from __future__ import annotations

import numpy as np
import pytest

from jugglebot.tracking.ballistics import GRAVITY_MMPS2
from jugglebot.tracking.matcher import BallTracker

CUP = np.array([-50.0, 0.0, 860.0])
VEL = np.array([0.0, 0.0, 4000.0])
FRAME_S = 0.0034          # QTM at ~290 Hz (both 2026-10 sitting bags)
GAP_START_S = 0.030       # ascending at ~3.7 m/s


def _pos(t):
    p = CUP + VEL * t
    p[2] -= 0.5 * GRAVITY_MMPS2 * t * t
    return p


def _first_fit_time(gap_s: float):
    tr = BallTracker(dt=0.005, landing_z=830.0, announced_gate_mm=200.0)
    bid = tr.handle_announcement(CUP.copy(), VEL.copy(), 0.0,
                                 source='jugglebot', destination='jugglebot')
    times = [i * FRAME_S for i in range(1, int(0.8 / FRAME_S))]
    times = [t for t in times if not (GAP_START_S < t < GAP_START_S + gap_s)]
    first_fit = None
    missed_after_gap = 0
    for t in times:
        tr.process_frame([_pos(t)], t)
        ball = tr.get_ball(bid)
        if t > GAP_START_S + gap_s and ball.frames_without_measurement > 0:
            missed_after_gap += 1
        if first_fit is None and ball.landing_from_fit:
            first_fit = t
    return first_fit, missed_after_gap


@pytest.mark.parametrize('gap_s', [0.036, 0.050, 0.080])
def test_a_dropped_run_of_frames_does_not_lose_the_confirmed_track(gap_s):
    """No missed association after the gap, and the fit converges as early as
    it does with no gap at all (~0.11 s: 12 samples above the freeze floor)."""
    first_fit, missed = _first_fit_time(gap_s)
    no_gap_fit, _ = _first_fit_time(0.0)
    assert missed == 0
    assert first_fit is not None
    assert first_fit <= no_gap_fit + gap_s + 0.005


def test_a_real_dropout_is_still_capped():
    """Past `max_frame_gap_s` the predict is capped: a replay restart or a
    multi-second QTM stall never becomes one giant leap."""
    tr = BallTracker(dt=0.005, landing_z=830.0)
    tr._last_frame_time = 10.0
    assert tr._frame_dt(12.0) == tr.max_frame_gap_s == pytest.approx(0.10)
