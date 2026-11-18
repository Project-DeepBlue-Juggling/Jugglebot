"""Tests for human_throw_event.py and the per-flight bookkeeping it reads
(`FlightFit.flight_seq`, `start_speed_mm_s`, `frozen`; matcher plumbing).

PWAY-11 P1 (design 2026-11-18). The thresholds exercised here are the PROPOSED
owner decisions D1 (start speed 500 mm/s) and D2 (rms <= 6 mm, two Fits 20 ms
apart agreeing within 30 mm / 20 ms). Real-data basis for the phantom numbers
(perception unit, 2026-11-18 idle window and 2026-11-02 bags): static-marker
phantoms start at 13-134 mm/s and the one moving floor marker at 365 mm/s,
Throws start at several thousand mm/s. Synthetic flight: plane 830, freeze
floor 1080, release at z 450 with 4500 mm/s up (the shape the sibling matcher
tests use), 300 Hz mocap.
"""
from __future__ import annotations

import logging

import numpy as np
import pytest

from jugglebot.tracking.ball import Ball, BallStatus, TrackingConfidence
from jugglebot.tracking.ballistics import GRAVITY_MMPS2
from jugglebot.tracking.flight_fit import FlightFit
from jugglebot.tracking.human_throw_event import (
    HumanThrowEventConfig, HumanThrowEventEmitter, R_PHANTOM, R_RMS)
from jugglebot.tracking.matcher import BallTracker

PLANE = 830.0
DT = 1.0 / 300.0
P0 = np.array([0.0, 0.0, 450.0])
V0 = np.array([100.0, -50.0, 4500.0])


def _pos(p0, v0, t):
    return np.array([p0[0] + v0[0] * t, p0[1] + v0[1] * t,
                     p0[2] + v0[2] * t - 0.5 * GRAVITY_MMPS2 * t * t])


def _true_landing_t(p0, v0, t0):
    dz = PLANE - p0[2]
    return t0 + (v0[2] + np.sqrt(v0[2] ** 2 - 2 * GRAVITY_MMPS2 * dz)) / GRAVITY_MMPS2


def _ball(i=7, source='human_throw', status=BallStatus.IN_FLIGHT,
          tracking=TrackingConfidence.CONFIRMED):
    return Ball(id=i, status=status, tracking=tracking, source=source)


def _fit(start_speed=4500.0):
    f = FlightFit(throw_time=0.0, landing_z=PLANE, new_flight_rise_mm=50.0)
    f.start_speed_mm_s = start_speed
    return f


def _run(emitter, fit, ball, noise_mm=1.5, seed=3, t_end=1.2, p0=P0, v0=V0,
         t0=0.0, start_t=0.0):
    """Feed one flight at 300 Hz, stepping the emitter after every frame."""
    rng = np.random.default_rng(seed)
    events = []
    t = start_t
    while t <= t_end:
        fit.add(t, _pos(p0, v0, t - t0) + rng.normal(0.0, noise_mm, 3))
        events += emitter.step([ball], {ball.id: fit}, t)
        t += DT
    return events


class TestEventStream:
    def test_one_flight_one_stream_with_revisions_and_a_single_final(self):
        em = HumanThrowEventEmitter()
        events = _run(em, _fit(), _ball())
        assert events, 'a clean flight must be served'
        assert {(e.track_id, e.flight_seq) for e in events} == {(7, 0)}
        assert [e.revision for e in events] == list(range(len(events)))
        finals = [e for e in events if e.final]
        assert len(finals) == 1 and events[-1].final
        t_true = _true_landing_t(P0, V0, 0.0)
        for e in events:
            assert abs(e.landing_time - t_true) < 0.010
            assert e.landing_z_mm == PLANE
            assert e.landing_time > e.stamp
            assert e.start_speed_mm_s == 4500.0
            assert e.n_samples >= 12 and e.span_s >= 0.05
            assert e.rms_mm <= 6.0
            assert e.fit_age_s >= 0.0
        assert events[0].flight_start == pytest.approx(
            events[0].flight_start)  # populated
        assert events[0].flight_start > 0.15  # first sample above the 1080 floor

    def test_first_event_waits_for_the_stability_gap(self):
        """Revision 0 needs a second usable Fit at least 20 ms older."""
        fit = _fit()
        em = HumanThrowEventEmitter()
        events = _run(em, fit, _ball())
        first_usable_t = None
        # Earliest frame at which the Fit is valid on its own.
        f2 = _fit()
        rng = np.random.default_rng(3)
        t = 0.0
        while t <= 1.2:
            f2.add(t, _pos(P0, V0, t) + rng.normal(0.0, 1.5, 3))
            if f2.fit() is not None and first_usable_t is None:
                first_usable_t = t
            t += DT
        assert events[0].stamp >= first_usable_t + 0.020 - 1e-6

    def test_refinements_only_when_the_landing_moves(self):
        events = _run(HumanThrowEventEmitter(), _fit(), _ball())
        for prev, cur in zip(events, events[1:]):
            moved = (np.linalg.norm(cur.landing_position - prev.landing_position) > 5.0
                     or abs(cur.landing_time - prev.landing_time) * 1000.0 > 5.0)
            assert moved or cur.final

    def test_nothing_after_the_landing_time(self):
        fit = _fit()
        em = HumanThrowEventEmitter()
        events = _run(em, fit, _ball())
        t_land = events[-1].landing_time
        late = em.step([_ball()], {7: fit}, t_land + 0.01)
        assert late == []

    def test_frozen_fit_emits_no_second_final(self):
        fit = _fit()
        em = HumanThrowEventEmitter()
        _run(em, fit, _ball())
        assert fit.frozen
        assert em.step([_ball()], {7: fit}, 1.0) == []

    def test_only_confirmed_in_flight_human_balls_qualify(self):
        for bad in (_ball(source='ball_butler'),
                    _ball(status=BallStatus.CAUGHT),
                    _ball(tracking=TrackingConfidence.ANNOUNCED)):
            em = HumanThrowEventEmitter()
            assert _run(em, _fit(), bad) == []
            assert em.rejections == {}


class TestPhantomRule:
    def test_static_marker_start_speed_emits_nothing_and_logs_once(self, caplog):
        em = HumanThrowEventEmitter()
        with caplog.at_level(logging.INFO):
            events = _run(em, _fit(start_speed=134.0), _ball(i=41))
        assert events == []
        lines = [r.getMessage() for r in caplog.records
                 if 'HUMAN-THROW-REJECTED' in r.getMessage()]
        assert len(lines) == 1
        assert 'id 41 seq 0' in lines[0] and '134' in lines[0] and '500' in lines[0]
        assert em.rejections == {R_PHANTOM: 1}

    def test_bar_is_inclusive_at_the_default_500(self):
        assert _run(HumanThrowEventEmitter(), _fit(499.9), _ball()) == []
        assert _run(HumanThrowEventEmitter(), _fit(500.0), _ball()) != []

    def test_bar_is_a_constructor_parameter(self):
        cfg = HumanThrowEventConfig(min_start_speed_mm_s=300.0)
        assert _run(HumanThrowEventEmitter(cfg), _fit(365.0), _ball()) != []

    def test_unknown_start_speed_is_not_a_throw(self, caplog):
        em = HumanThrowEventEmitter()
        with caplog.at_level(logging.INFO):
            assert _run(em, _fit(start_speed=None), _ball()) == []
        assert sum('HUMAN-THROW-REJECTED' in r.getMessage()
                   for r in caplog.records) == 1


class TestUsableRule:
    def test_noisy_fit_emits_nothing(self):
        em = HumanThrowEventEmitter()
        events = _run(em, _fit(), _ball(), noise_mm=9.0)
        assert events == []
        assert em.rejections.get(R_RMS, 0) > 0 or em.rejections.get('fit_invalid', 0) > 0

    def test_too_few_samples_emits_nothing(self):
        fit = _fit()
        em = HumanThrowEventEmitter()
        # Stop while the flight has not yet produced 12 admitted samples.
        assert _run(em, fit, _ball(), t_end=0.19) == []
        assert fit.fit() is None

    def test_rms_bar_is_a_constructor_parameter(self):
        tight = HumanThrowEventConfig(rms_mm=0.1)
        assert _run(HumanThrowEventEmitter(tight), _fit(), _ball()) == []

    def test_instability_blocks_revision_zero(self):
        """An agreement bar tighter than two live Fits can meet never serves
        while the Fit is still changing; only the frozen Fit (identical to
        itself, so trivially stable) is served, as a final revision."""
        cfg = HumanThrowEventConfig(stable_mm=0.0001, stable_ms=20.0)
        events = _run(HumanThrowEventEmitter(cfg), _fit(), _ball())
        assert len(events) == 1 and events[0].final and events[0].revision == 0

    def test_unserved_frozen_flight_is_logged_once(self, caplog):
        cfg = HumanThrowEventConfig(rms_mm=0.1)
        with caplog.at_level(logging.INFO):
            _run(HumanThrowEventEmitter(cfg), _fit(), _ball(i=9))
        lines = [r.getMessage() for r in caplog.records
                 if 'HUMAN-THROW-UNSERVED' in r.getMessage()]
        assert len(lines) == 1 and 'id 9 seq 0' in lines[0]


class TestPerFlightSequence:
    """A kept track id re-thrown: new flight_seq, phantom judged per flight."""

    F2_P = np.array([150.0, 0.0, 450.0])
    F2_V = np.array([-400.0, 100.0, 4000.0])

    def _carry(self, fit, em, ball, speeds_ok=True):
        """Flight 1, a hand dwell below the floor, flight 2; returns events."""
        events = []
        rng = np.random.default_rng(5)
        t1_end = 2.0 * V0[2] / GRAVITY_MMPS2
        t2 = t1_end + 0.30
        t = 0.0
        while t <= t2 + 1.2:
            if t < t1_end:
                p = _pos(P0, V0, t)
            elif t < t2:
                p = np.array([75.0, 0.0, 450.0 - 300.0 * np.sin(
                    np.pi * (t - t1_end) / 0.30)])   # scoop: dips 300 mm
            else:
                p = _pos(self.F2_P, self.F2_V, t - t2)
            fit.add(t, p + rng.normal(0.0, 1.5, 3))
            if fit.start_speed_pending and len(fit) > 0:
                fit.start_speed_mm_s = 4000.0
                fit.start_speed_pending = False
            events += em.step([ball], {ball.id: fit}, t)
            t += DT
        return events, t2

    def test_rethrow_gets_a_new_flight_seq_and_its_own_events(self):
        fit, em = _fit(), HumanThrowEventEmitter()
        events, t2 = self._carry(fit, em, _ball())
        assert {e.flight_seq for e in events} == {0, 1}
        assert fit.flight_seq == 1
        seq1 = [e for e in events if e.flight_seq == 1]
        assert seq1[0].revision == 0 and seq1[-1].final
        assert seq1[0].flight_start > t2
        t_true = _true_landing_t(self.F2_P, self.F2_V, t2)
        assert abs(seq1[-1].landing_time - t_true) < 0.010
        assert seq1[0].start_speed_mm_s == 4000.0

    def test_slow_first_flight_does_not_silence_the_rethrow(self):
        fit, em = _fit(start_speed=120.0), HumanThrowEventEmitter()
        events, _ = self._carry(fit, em, _ball())
        assert {e.flight_seq for e in events} == {1}
        assert em.rejections[R_PHANTOM] == 1

    def test_start_speed_pending_blocks_a_decision_not_a_rejection(self):
        """Between the new flight opening and the Tracker recording its start
        speed the emitter stays silent and counts nothing."""
        fit, em = _fit(), HumanThrowEventEmitter()
        fit.start_speed_mm_s = None
        fit.start_speed_pending = True
        assert em.step([_ball()], {7: fit}, 0.0) == []
        assert em.rejections == {}


class TestFlightFitBookkeeping:
    def test_flight_start_and_seq_defaults(self):
        f = _fit()
        assert f.flight_seq == 0 and f.flight_start_t is None and not f.frozen
        f.add(0.5, [0.0, 0.0, 1400.0])
        assert f.flight_start_t == 0.5 and not f.frozen
        f.add(0.9, [0.0, 0.0, 1000.0])      # back under the 1080 floor
        assert f.frozen

    def test_plain_reset_does_not_advance_the_sequence(self):
        f = _fit()
        f.add(0.5, [0.0, 0.0, 1400.0])
        f.reset()
        assert f.flight_seq == 0 and f.flight_start_t is None

    def test_fit_result_reports_span(self):
        f = _fit()
        t = 0.2
        while t < 0.5:
            f.add(t, _pos(P0, V0, t))
            t += DT
        fr = f.fit()
        assert fr is not None and fr.span_s == pytest.approx(0.3, abs=0.01)


class TestTrackerPlumbing:
    def _tracker(self):
        return BallTracker(dt=DT, landing_z=PLANE, detect_human_throws=True,
                           flight_fit_new_flight_rise_mm=50.0)

    def test_promoted_flight_zero_carries_its_promotion_speed(self):
        tracker = self._tracker()
        t = 0.0
        while t < 0.6:
            tracker.process_frame([_pos(P0, V0, t)], t)
            t += DT
        (ball,) = [b for b in tracker.active_balls if b.source == 'human_throw']
        fit = tracker._flight_fits[ball.id]
        assert fit.flight_seq == 0
        assert fit.start_speed_mm_s == pytest.approx(
            np.linalg.norm(V0), rel=0.15)  # filter speed at promotion

    def test_tracker_to_emitter_end_to_end(self):
        tracker, em = self._tracker(), HumanThrowEventEmitter()
        events = []
        t = 0.0
        while t < 1.2:
            balls = tracker.process_frame([_pos(P0, V0, t)], t)
            events += em.step(balls, tracker._flight_fits, t)
            t += DT
        assert events and events[-1].final
        assert abs(events[-1].landing_time - _true_landing_t(P0, V0, 0.0)) < 0.010

    def test_static_marker_never_yields_an_event(self):
        """A marker that wobbles in place can be promoted by the matcher;
        the emitter's phantom rule keeps it out of the stream."""
        tracker, em = self._tracker(), HumanThrowEventEmitter()
        events = []
        t = 0.0
        while t < 2.0:
            marker = np.array([300.0, 200.0, 2200.0])
            balls = tracker.process_frame([marker], t)
            events += em.step(balls, tracker._flight_fits, t)
            t += DT
        assert events == []
