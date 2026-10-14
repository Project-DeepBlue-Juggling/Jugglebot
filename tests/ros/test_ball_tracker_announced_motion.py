"""The 2026-10-14 announced-ball MOTION test: a static marker never confirms.

At the 2026-10-14 two-ball columns sessions the tracker confirmed every Ball
Butler feed read (5/5) on a STATIC `Ball Butler - 7` body marker 78 mm from the
announced start; the flight fit then read that point at rest at z=1720 as an
apex and put `time_at_land` 0.34-0.36 s early and ~700 mm off, so the executor's
identity gate refused the tracker on every feed. The real ball had left BB's
hand before the announced throw_time, faster than announced, and sat 230-380 mm
from the analytic position all flight -- outside the old 200 mm sphere.

Geometry below is MEASURED from bag 2026-10-14_20-54-20 (S1, feed thrown at
unix 1791972045.098, and JB self-toss at 1791972046.261), not invented; see the
2026-10-14 two-ball columns sessions entry.
"""
from __future__ import annotations

import numpy as np

from jugglebot.tracking.ball import TrackingConfidence
from jugglebot.tracking.ballistics import GRAVITY_MMPS2
from jugglebot.tracking.matcher import BallTracker, parse_label_prefixes
import jugglebot.hardware_config as hw

DT = 1.0 / 300.0          # the 2026-10-14 mocap rate
LANDING_Z = 830.0

# Ball Butler's announcement (S1, throw 1791972045.098), verbatim.
BB_P0 = np.array([-896.0, -424.0, 1739.0])
BB_V0 = np.array([949.3, 362.4, 3131.5])
BB_LAND_T = 0.882
# Static markers in every frame of the window: BB body #7/#3/#4/#1 and two
# unlabelled ones -- 78-422 mm from the announced start.
BB_STATIC = [
    (np.array([-884.0, -350.0, 1720.0]), 'Ball Butler - 7'),
    (np.array([-790.0, -473.0, 1719.0]), 'Ball Butler - 3'),
    (np.array([-786.0, -382.0, 1721.0]), 'Ball Butler - 4'),
    (np.array([-810.0, -321.0, 1693.0]), 'Ball Butler - 1'),
    (np.array([-722.0, -378.0, 1662.0]), ''),
    (np.array([-835.0, -516.0, 2146.0]), ''),   # 174 mm from expected at +0.13 s
]


def _bb_ball(t):
    """The real ball, fitted to the bag's unlabelled marker: apex 2521 mm at
    +0.29 s, in free flight from ~-0.05 s (z 1951 there, 2123 at +0.008)."""
    return np.array([-868.0 + 898.0 * t, -300.0 + 313.0 * t,
                     2521.0 - 0.5 * GRAVITY_MMPS2 * (t - 0.29) ** 2])


def _make_tracker(**over):
    kw = dict(dt=0.005, landing_z=LANDING_Z,
              announced_gate_mm=hw.TRACKING_ANNOUNCED_GATE_MM,
              excluded_label_prefixes=parse_label_prefixes(
                  hw.TRACKING_EXCLUDED_LABEL_PREFIXES))
    kw.update(over)
    return BallTracker(**kw)


def _announce_bb(tracker):
    return tracker.handle_announcement(
        initial_position=BB_P0.copy(), initial_velocity=BB_V0.copy(),
        throw_time=0.0, source='ball_butler', destination='jugglebot',
        landing_position=np.array([-61.0, 2.0, 830.0]),
        landing_velocity=np.array([949.3, 362.4, -5500.0]),
        landing_time=BB_LAND_T)


def _bb_frame(t, with_ball=True):
    markers = [p for p, _ in BB_STATIC]
    labels = [l for _, l in BB_STATIC]
    if with_ball and t >= -0.05:
        markers.append(_bb_ball(t))
        labels.append('')
    return markers, labels


class TestBallButlerFeed:

    def test_confirms_on_the_moving_ball_never_on_a_static_marker(self):
        tracker = _make_tracker()
        bid = _announce_bb(tracker)
        statics = [p for p, _ in BB_STATIC]
        t = -0.15
        while t < 0.30:
            m, l = _bb_frame(t)
            tracker.process_frame(m, t, l)
            b = tracker.get_ball(bid)
            if b.tracking == TrackingConfidence.CONFIRMED:
                assert min(np.linalg.norm(b.position - s) for s in statics) > 100.0
            t += DT
        confirms = [e for e in tracker.identity_events if e['kind'] == 'confirm']
        assert len(confirms) == 1
        ev = confirms[0]
        # On the first in-flight frame: history built from the pre-throw frames.
        assert 0.0 <= ev['dt_throw'] < 2 * DT
        assert np.linalg.norm(ev['marker'] - _bb_ball(ev['t'])) < 1e-6
        assert not [e for e in tracker.identity_events if e['kind'] == 'demote']

    def test_the_brief_case_ball_on_the_announced_parabola_but_250mm_ahead(self):
        """Release leads the announced instant by 0.085 s, so at throw_time the
        ball is already ~250 mm along the announced parabola."""
        tracker = _make_tracker()
        bid = _announce_bb(tracker)
        lead = 0.085

        def ball(t):
            u = t + lead
            return BB_P0 + BB_V0 * u + np.array([0, 0, -0.5 * GRAVITY_MMPS2 * u * u])

        assert np.linalg.norm(ball(0.0) - BB_P0) > 240.0
        n_after = 0
        t = 0.0
        while tracker.get_ball(bid).tracking != TrackingConfidence.CONFIRMED and t < 0.2:
            m = [p for p, _ in BB_STATIC] + [ball(t)]
            l = [l for _, l in BB_STATIC] + ['']
            tracker.process_frame(m, t, l)
            n_after += 1
            t += DT
        b = tracker.get_ball(bid)
        assert b.tracking == TrackingConfidence.CONFIRMED
        assert n_after <= 3
        assert np.linalg.norm(b.position - ball(t - DT)) < 5.0

    def test_static_markers_alone_never_confirm(self):
        """A fully occluded ball: only static markers, for the whole flight."""
        tracker = _make_tracker()
        bid = _announce_bb(tracker)
        t = -0.15
        while t < BB_LAND_T:
            m, l = _bb_frame(t, with_ball=False)
            tracker.process_frame(m, t, l)
            t += DT
        assert tracker.get_ball(bid).tracking == TrackingConfidence.ANNOUNCED


class TestJugglebotSelfToss:
    """JB's own announcement: start = release site, ball ~3.8 m/s at throw_time."""

    P0 = np.array([-62.0, 0.0, 860.0])
    V0 = np.array([-21.0, -8.0, 4094.0])

    def _ball(self, t):
        # Measured (bag S1, 1791972046.261): 757 @-0.025, 798 @-0.012,
        # 824 @-0.005, 851 @+0.002, then on the announced parabola ~10 mm low.
        if t < 0.0:
            return np.array([-63.0, 4.0, 851.0 + 3900.0 * (t - 0.002)])
        return self.P0 + self.V0 * t + np.array([0, 0, -0.5 * GRAVITY_MMPS2 * t * t - 10.0])

    def test_confirms_on_the_first_in_flight_frame_as_before(self):
        tracker = _make_tracker()
        bid = tracker.handle_announcement(
            initial_position=self.P0.copy(), initial_velocity=self.V0.copy(),
            throw_time=0.0, source='jugglebot', destination='jugglebot')
        t = -0.15
        first_in_flight = None
        while t < 0.05:
            # Ball sits in the cup (static) until the stroke starts at -0.06 s.
            pos = self._ball(t) if t > -0.06 else np.array([-61.0, 2.0, 712.0])
            tracker.process_frame([pos], t, [''])
            if first_in_flight is None and t >= 0.0:
                first_in_flight = t
                assert tracker.get_ball(bid).tracking == TrackingConfidence.CONFIRMED
            t += DT
        assert first_in_flight is not None
        assert tracker.get_ball(bid).tracking == TrackingConfidence.CONFIRMED

    def test_ball_resting_in_the_cup_after_landing_is_not_demoted(self):
        tracker = _make_tracker()
        bid = tracker.handle_announcement(
            initial_position=self.P0.copy(), initial_velocity=self.V0.copy(),
            throw_time=0.0, source='jugglebot', destination='jugglebot')
        t_land = tracker.get_ball(bid).landing_time
        t = -0.05
        while t < t_land + 0.3:
            pos = self._ball(min(t, t_land))
            tracker.process_frame([pos], t, [''])
            t += DT
        assert not [e for e in tracker.identity_events if e['kind'] == 'demote']


class TestDemotionSafetyNet:

    def test_a_wrong_static_confirmation_is_released_within_60ms(self):
        """`announced_motion_steps=0` reproduces the pre-2026-10-14 one-frame
        rule, which confirms on BB #7; the still-marker demotion must release
        it, exclude #7, and the moving ball must then confirm."""
        logged = []
        tracker = _make_tracker(announced_motion_steps=0, debug_log=logged.append)
        bid = _announce_bb(tracker)
        t = 0.0
        # ball occluded for the first 0.12 s so #7 is the only candidate
        # Only #7 in view, so after its demotion nothing static can re-take
        # the one-frame rule this test switches back on.
        while t < 0.12:
            tracker.process_frame([BB_STATIC[0][0]], t, [BB_STATIC[0][1]])
            t += DT
        ev = list(tracker.identity_events)
        assert ev[0]['kind'] == 'confirm'
        np.testing.assert_allclose(ev[0]['marker'], BB_STATIC[0][0])
        demotes = [e for e in ev if e['kind'] == 'demote']
        assert len(demotes) >= 1
        assert demotes[0]['t'] - ev[0]['t'] <= 0.060 + 2 * DT
        # Every confirm after the first demotion must not be on #7 again.
        later = [e for e in ev if e['kind'] == 'confirm' and e['t'] > demotes[0]['t']]
        for e in later:
            assert np.linalg.norm(e['marker'] - BB_STATIC[0][0]) > 20.0
        # The ball reappears: it confirms and stays confirmed.
        while t < 0.30:
            tracker.process_frame([BB_STATIC[0][0], _bb_ball(t)], t,
                                  [BB_STATIC[0][1], ''])
            t += DT
        b = tracker.get_ball(bid)
        assert b.tracking == TrackingConfidence.CONFIRMED
        assert np.linalg.norm(b.position - _bb_ball(t - DT)) < 30.0
        # One DEBUG line per identity decision, never per frame.
        assert len(logged) == len(tracker.identity_events)
        assert any('demote' in s for s in logged)


def test_one_debug_line_per_confirmation():
    logged = []
    tracker = _make_tracker(debug_log=logged.append)
    _announce_bb(tracker)
    t = -0.15
    while t < 0.4:
        m, l = _bb_frame(t)
        tracker.process_frame(m, t, l)
        t += DT
    assert len(logged) == 1 and 'confirm' in logged[0]
