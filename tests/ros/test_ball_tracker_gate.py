"""The 2026-09-15 announced-ball gate: all markers in, expected-position sphere out.

Regression cover for the defect that cost the 2026-09-15 apex-ladder sitting all
13 of its catches. `ball_tracker_node._on_mocap` forwarded only markers whose
mocap label was EMPTY; QTM's AIM model had labelled the flying ball
`Ball Butler - 1`; so the matcher never saw a candidate and every announced ball
stayed ANNOUNCED until the CATCH executor gave up with `NO_LANDING`.

Every pre-existing offline test fed the matcher synthetic *unlabelled* markers,
so the whole suite was blind to it by construction. These tests feed LABELLED
markers on purpose — that is the point of the file. The end-to-end evidence lives
in `tools/probes/tracker_bag_replay.py`, which replays real bags through this
same matcher.

The numbers below are the measured geometry of the 2026-09-15 bag, not invented
thresholds: the ball marker sits in the cup at 717-742 mm (under the retired
880 mm height floor), and the platform's own markers sit 204.6 / 206.3 / 219.5 /
220.7 mm from the catch cup — inside a ±200 mm box, outside a 200 mm sphere.
"""
from __future__ import annotations

import numpy as np
import pytest

from jugglebot.tracking.ball import TrackingConfidence
# The matcher's OWN gravity constant, not a restated 9810 — a 4 mm/s^2
# disagreement shows up as a 0.27 mm gate-centre offset at t = 0.37 s, which is
# exactly the kind of drift the analytic-centre test is there to rule out.
from jugglebot.tracking.ballistics import GRAVITY_MMPS2
from jugglebot.tracking.matcher import BallTracker, parse_label_prefixes
import jugglebot.hardware_config as hw

DT = 0.005
LANDING_Z = 830.0

# The 2026-09-15 self-toss site, straight out of `/throw_announcements`.
CUP = np.array([-50.0, 0.0, 860.0])
TOSS_VEL = np.array([0.0, 0.0, 3084.5])

# `Platform - 3` and `Platform - 5` as measured at the release instant of
# armA-050: 204.6 mm and 206.3 mm from CUP — i.e. a ±200 mm axis-aligned box
# would admit both, a 200 mm sphere admits neither.
PLATFORM_3 = np.array([-24.0, -62.0, 1053.0])
PLATFORM_5 = np.array([-200.0, 105.0, 764.0])

# The PHYSICAL Ball Butler rig's own markers. They share the `Ball Butler`
# prefix with the ball's label and are therefore deliberately NOT excluded — so
# where they actually sit is load-bearing, and it was measured rather than
# assumed. Session-wide on the 2026-09-15 bag (every 20th of 238 553 frames,
# 790 samples each): `Ball Butler - 4` sits 1257.1-1257.8 mm from the cup and
# `Ball Butler - 5` sits 1287.7-1288.0 mm, both static to under 1 mm, and
# **0 samples of either fall within 200 mm of the cup**. Both are ABSENT from
# every frame within +-1.5 s of both measured release instants. The rig is
# parked ~1.26-1.29 m away: outside the gate by a factor of ~6.
BB_RIG_4_DIST_MM = 1257.1          # measured minimum over the session
BB_RIG_5_DIST_MM = 1287.7
BB_RIG_4 = CUP + np.array([BB_RIG_4_DIST_MM, 0.0, 0.0])


def _make_tracker(**over):
    """A BallTracker wired as `ball_tracker_node` wires it, with overrides."""
    kw = dict(
        dt=DT,
        landing_z=LANDING_Z,
        announced_gate_mm=hw.TRACKING_ANNOUNCED_GATE_MM,
        excluded_label_prefixes=parse_label_prefixes(
            hw.TRACKING_EXCLUDED_LABEL_PREFIXES),
        detect_human_throws=hw.TRACKING_DETECT_HUMAN_THROWS,
    )
    kw.update(over)
    return BallTracker(**kw)


def _ballistic(pos0, vel0, t):
    return np.array([
        pos0[0] + vel0[0] * t,
        pos0[1] + vel0[1] * t,
        pos0[2] + vel0[2] * t - 0.5 * GRAVITY_MMPS2 * t * t,
    ])


def _announce(tracker, throw_time=0.0):
    return tracker.handle_announcement(
        initial_position=CUP.copy(),
        initial_velocity=TOSS_VEL.copy(),
        throw_time=throw_time,
        source='jugglebot',
        destination='jugglebot',
    )


class TestLabelledBallIsSeen:
    """The headline defect: a labelled ball must still be a candidate."""

    def test_ball_labelled_as_a_rigid_body_marker_still_confirms(self):
        """`Ball Butler - 1` is what QTM called the ball on all 13 throws."""
        tracker = _make_tracker()
        bid = _announce(tracker)
        for i in range(1, 6):
            t = i * DT
            pos = _ballistic(CUP, TOSS_VEL, t)
            tracker.process_frame([pos], t, ['Ball Butler - 1'])
        assert tracker.get_ball(bid).tracking == TrackingConfidence.CONFIRMED

    def test_unlabelled_ball_still_confirms(self):
        """Unlabelled markers stay eligible — this is a widening, not a swap."""
        tracker = _make_tracker()
        bid = _announce(tracker)
        for i in range(1, 6):
            t = i * DT
            tracker.process_frame([_ballistic(CUP, TOSS_VEL, t)], t, [''])
        assert tracker.get_ball(bid).tracking == TrackingConfidence.CONFIRMED

    def test_no_labels_argument_behaves_as_all_eligible(self):
        """Omitting labels keeps the synthetic-marker tests and the analyzer working."""
        tracker = _make_tracker()
        bid = _announce(tracker)
        for i in range(1, 6):
            t = i * DT
            tracker.process_frame([_ballistic(CUP, TOSS_VEL, t)], t)
        assert tracker.get_ball(bid).tracking == TrackingConfidence.CONFIRMED

    def test_ball_marker_below_the_retired_height_floor_confirms(self):
        """The floor discarded the one true candidate at the throw instant.

        At `throw_time` the ball is still in the cup at ~742 mm, far below the
        old `landing_z + 50` = 880 mm floor. There is no floor now.
        """
        tracker = _make_tracker()
        bid = _announce(tracker)
        in_cup = np.array([-45.0, 6.0, 742.0])
        assert in_cup[2] < LANDING_Z + 50.0, 'fixture must sit under the old floor'
        tracker.process_frame([in_cup], 0.0, ['Ball Butler - 1'])
        assert tracker.get_ball(bid).tracking == TrackingConfidence.CONFIRMED


class TestExclusionIsByRigidBodyMembership:

    def test_platform_and_base_markers_are_excluded(self):
        markers = [PLATFORM_3, PLATFORM_5, np.array([0.0, 0.0, -40.0])]
        labels = ['Platform - 3', 'Platform - 5', 'Base - 1']
        tracker = _make_tracker()
        assert tracker.eligible_markers(markers, labels) == []

    def test_ball_butler_labels_are_NOT_excluded(self):
        """Excluding BB-style labels would re-create the 2026-09-15 blindness.

        The ball carries a BB label whenever QTM's AIM model claims it, so the
        exclusion set may only ever name the robot's OWN rigid bodies.
        """
        prefixes = parse_label_prefixes(hw.TRACKING_EXCLUDED_LABEL_PREFIXES)
        assert not any('Ball' in p for p in prefixes)
        assert not any('Butler' in p for p in prefixes)
        tracker = _make_tracker()
        elig = tracker.eligible_markers([CUP.copy()], ['Ball Butler - 1'])
        assert len(elig) == 1

    def test_platform_marker_at_the_cup_is_not_matched(self):
        """A platform marker inside the gate must not confirm the ball.

        The expected position of an announced ball at its throw instant IS the
        cup, and the platform's own markers are the nearest things to it, so
        identity — not distance — has to be what rules them out.
        """
        tracker = _make_tracker()
        bid = _announce(tracker)
        at_cup = CUP + np.array([5.0, 5.0, 5.0])
        for i in range(10):
            tracker.process_frame([at_cup], i * DT, ['Platform - 1'])
        assert tracker.get_ball(bid).tracking == TrackingConfidence.ANNOUNCED

    def test_ball_butler_rig_marker_at_its_measured_distance_never_confirms(self):
        """The rig sits 1257 mm from the cup — outside the gate by ~6x.

        `Ball Butler - 4/5` are the PHYSICAL Ball Butler rig's markers, and they
        share the un-excluded `Ball Butler` prefix with the ball's own label, so
        nothing keeps them out by identity. Geometry does: both measured
        1257.1-1288.0 mm from the cup across the whole 2026-09-15 session, 0
        samples inside 200 mm, and absent from every frame within +-1.5 s of
        both release instants.

        This feeds the rig marker at its MEASURED distance for a whole flight
        with NO ball marker present at all — the worst case, a fully occluded
        ball — and asserts the ball never confirms off the rig.
        """
        tracker = _make_tracker()
        bid = _announce(tracker)
        assert BB_RIG_4_DIST_MM > hw.TRACKING_ANNOUNCED_GATE_MM, (
            'fixture must sit outside the gate, as measured')
        for i in range(120):
            tracker.process_frame([BB_RIG_4], i * DT, ['Ball Butler - 4'])
        assert tracker.get_ball(bid).tracking == TrackingConfidence.ANNOUNCED

    def test_nearest_wins_picks_the_ball_over_a_rig_marker_in_the_gate(self):
        """Defence in depth for a rig the measurements say is NOT there.

        The measured rig is 1257 mm away (see above), so this geometry is
        HYPOTHETICAL: it asks what happens if the Ball Butler rig were ever
        re-parked, or a new BB-labelled marker appeared, inside the gate. Since
        BB labels are deliberately eligible, the only thing separating the ball
        from such a marker is nearest-wins — so pin it.
        """
        tracker = _make_tracker()
        bid = _announce(tracker)
        t = 10 * DT
        expected = _ballistic(CUP, TOSS_VEL, t)
        ball = expected + np.array([10.0, 0.0, 0.0])          # 10 mm out
        rig_in_gate = expected + np.array([0.0, 0.0, 150.0])  # 150 mm out
        # Order the rig FIRST so a first-match-wins bug would fail this.
        tracker.process_frame([rig_in_gate, ball], t,
                              ['Ball Butler - 4', 'Ball Butler - 1'])
        b = tracker.get_ball(bid)
        assert b.tracking == TrackingConfidence.CONFIRMED
        # The filter must have been updated toward the BALL, not the rig marker.
        assert (np.linalg.norm(b.position - ball)
                < np.linalg.norm(b.position - rig_in_gate))

    def test_empty_prefix_string_excludes_nothing(self):
        assert parse_label_prefixes('') == ()
        assert parse_label_prefixes('Platform,Base') == ('Platform', 'Base')
        assert parse_label_prefixes(' Platform , Base ,') == ('Platform', 'Base')


class TestExpectedPositionSphere:

    @pytest.mark.parametrize('offset_mm,should_match', [
        (0.0, True),
        (150.0, True),
        (199.0, True),
        (201.0, False),
        (400.0, False),
    ])
    def test_gate_is_a_sphere_around_the_expected_position(self, offset_mm, should_match):
        tracker = _make_tracker()
        bid = _announce(tracker)
        t = 10 * DT
        expected = _ballistic(CUP, TOSS_VEL, t)
        marker = expected + np.array([offset_mm, 0.0, 0.0])
        tracker.process_frame([marker], t, ['Ball Butler - 1'])
        confirmed = tracker.get_ball(bid).tracking == TrackingConfidence.CONFIRMED
        assert confirmed is should_match

    def test_gate_is_a_sphere_not_a_box(self):
        """A corner marker 200 mm out on every axis (|d| = 346 mm) must NOT match.

        This is the case that separates the two shapes, and the reason the sphere
        was chosen: three platform markers live in that corner region.
        """
        tracker = _make_tracker()
        bid = _announce(tracker)
        t = 10 * DT
        corner = _ballistic(CUP, TOSS_VEL, t) + np.array([195.0, 195.0, 195.0])
        tracker.process_frame([corner], t, ['Ball Butler - 1'])
        assert tracker.get_ball(bid).tracking == TrackingConfidence.ANNOUNCED

    def test_gate_centre_is_analytic_not_the_kalman_clock(self):
        """The centre must not drift when the frame period differs from `dt`.

        The 2026-09-15 bag delivers ~186 Hz against `dt` = 5 ms. A KF-centred
        gate walks off the real trajectory as the mismatch accumulates; the
        analytic centre cannot. Here the frames arrive at 3x `dt`, so a
        KF-centred gate would be ~2/3 of a flight behind by the time it matters.
        """
        tracker = _make_tracker()
        bid = _announce(tracker)
        step = 3 * DT
        for i in range(1, 12):
            t = i * step
            tracker.process_frame([_ballistic(CUP, TOSS_VEL, t)], t, ['Ball Butler - 1'])
        ball = tracker.get_ball(bid)
        assert ball.tracking == TrackingConfidence.CONFIRMED
        # And the gate centre itself tracks wall clock exactly.
        t = 0.37
        centre = tracker._expected_announced_position(bid, t)
        np.testing.assert_allclose(centre, _ballistic(CUP, TOSS_VEL, t), atol=1e-9)

    def test_expected_position_before_throw_falls_back(self):
        tracker = _make_tracker()
        bid = _announce(tracker, throw_time=10.0)
        fb = np.array([1.0, 2.0, 3.0])
        got = tracker._expected_announced_position(bid, 9.0, fallback=fb)
        np.testing.assert_allclose(got, fb)


class TestHumanThrowPathDisabled:

    def test_parabolic_detection_off_spawns_no_phantom_balls(self):
        """158 phantom `human_throw` tracks came off one static floor marker."""
        tracker = _make_tracker()
        floor_marker = np.array([550.0, -353.0, 95.0])
        falling = np.array([0.0, 0.0, 2000.0])
        for i in range(60):
            t = i * DT
            tracker.process_frame(
                [floor_marker, _ballistic(np.array([0.0, 0.0, 1500.0]), falling, t)],
                t, ['', ''])
        assert tracker.active_balls == []

    def test_parabolic_detection_can_be_turned_back_on(self):
        tracker = _make_tracker(detect_human_throws=True)
        start = np.array([0.0, 0.0, 1500.0])
        vel = np.array([0.0, 0.0, 2000.0])
        for i in range(8):
            t = i * DT
            tracker.process_frame([_ballistic(start, vel, t)], t, [''])
        assert tracker.active_balls, 'human-throw path must still work when enabled'

    def test_config_default_is_off(self):
        assert hw.TRACKING_DETECT_HUMAN_THROWS is False


class TestConfigWiring:

    def test_generated_params_exist_and_are_sane(self):
        assert hw.TRACKING_ANNOUNCED_GATE_MM == pytest.approx(200.0)
        assert hw.TRACKING_EXCLUDED_LABEL_PREFIXES == 'Platform,Base'
        assert hw.TRACKING_DETECT_HUMAN_THROWS is False

    def test_defaults_match_the_config(self):
        """A BallTracker built with no arguments must not disagree with the YAML."""
        bare = BallTracker()
        assert bare.announced_gate_mm == pytest.approx(hw.TRACKING_ANNOUNCED_GATE_MM)
        assert bare.detect_human_throws is bool(hw.TRACKING_DETECT_HUMAN_THROWS)
        # The prefix set is deliberately EMPTY by default: a bare tracker has no
        # business guessing which rigid bodies belong to the robot, and an
        # unlabelled-marker caller must not be silently filtered.
        assert bare.excluded_label_prefixes == ()
