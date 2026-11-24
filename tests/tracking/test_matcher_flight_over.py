"""BUG-70: a CONFIRMED track whose flight is over stops taking the marker.

The 2026-11-05 fed Attempt (bag 2026-11-05_22-46-00): one physical ball is
caught, carried and thrown again, and its marker never leaves the 50 mm
association gate of the PREVIOUS Throw's filter. `_associate_confirmed_balls`
runs before `_match_announced_balls`, so the old track kept the marker through
the Carry and the whole next flight and the new Throw never confirmed (replay
on 2026-11-25: tracker ids 3, 12, 15, 16, 44, 47, 48, 49 of 49). Steady Throws
escaped only because one frame in the Carry lost the ball marker, after which
the old filter coasted out of its own gate.

The synthetic marker is one ball in the bag's geometry: the cup holds it at
z 705 mm (the bag's Carry read 705-711), 0.9 s flights from release to the
landing plane, a Throw every 1.3 s. The re-throw ramp is constant acceleration
from rest over `ramp_s`, ending at the announced release state. No frame ever
drops the marker, so the luck that saved the steady Throws is absent.
"""
from __future__ import annotations

import numpy as np
import pytest

from jugglebot.tracking.ball import BallStatus, TrackingConfidence
from jugglebot.tracking.ballistics import GRAVITY_MMPS2
from jugglebot.tracking.matcher import BallTracker

DT = 1.0 / 300.0       # the 2026-11-05 bag streams QTM at ~300 Hz
LANDING_Z = 734.3            # matcher default; the freeze floor is 984.3
CUP_Z = 705.0
FLIGHT_S = 0.9
PERIOD_S = 1.3
XY = (100.0, -50.0)
STATIC = [np.array([400.0, 300.0, 600.0]), np.array([-350.0, 250.0, 700.0])]
SLOW, FAST = 0.30, 0.06      # ramp_s


def _release(ramp_s):
    """(v0, z_release) for a ramp from rest at CUP_Z that ends at v0 and a
    free flight that reaches LANDING_Z FLIGHT_S after the release."""
    v0 = (LANDING_Z - CUP_Z + 0.5 * GRAVITY_MMPS2 * FLIGHT_S ** 2) / (0.5 * ramp_s + FLIGHT_S)
    return v0, CUP_Z + 0.5 * v0 * ramp_s


def _ball_z(t, throws, ramp_s):
    v0, zr = _release(ramp_s)
    started = [tt for tt in throws if t >= tt - ramp_s]
    if not started:
        return CUP_Z
    tt = started[-1]
    if t < tt:
        s = t - (tt - ramp_s)
        return CUP_Z + 0.5 * (v0 / ramp_s) * s * s
    tau = t - tt
    return max(zr + v0 * tau - 0.5 * GRAVITY_MMPS2 * tau * tau, CUP_Z)


def _run(ramp_s, n_throws=3, tracker=None, on_frame=None):
    """Announce each Throw 0.3 s before its release and stream frames.
    Returns (tracker, ids, throws, log); log maps id -> [(t, Ball fields)]."""
    tracker = tracker or BallTracker(dt=DT, landing_z=LANDING_Z)
    v0, zr = _release(ramp_s)
    throws = [1.0 + k * PERIOD_S for k in range(n_throws)]
    t_end = throws[-1] + FLIGHT_S + 0.6
    ids, log = {}, {}
    t = 0.0
    while t <= t_end:
        for k, tt in enumerate(throws):
            if k not in ids and t >= tt - 0.3:
                ids[k] = tracker.handle_announcement(
                    initial_position=np.array([XY[0], XY[1], zr]),
                    initial_velocity=np.array([0.0, 0.0, v0]),
                    throw_time=tt, source='jugglebot', destination='jugglebot')
        frame = [np.array([XY[0], XY[1], _ball_z(t, throws, ramp_s)])] + [p.copy() for p in STATIC]
        for b in tracker.process_frame(frame, t):
            log.setdefault(b.id, []).append(dict(
                t=t, status=b.status, tracking=b.tracking, fwm=b.frames_without_measurement,
                landing_time=float(b.landing_time), from_fit=bool(b.landing_from_fit),
                landing_pos=np.array(b.landing_position, dtype=float)))
        if on_frame is not None:
            on_frame(tracker, t)
        t = round(t + DT, 9)
    return tracker, ids, throws, log


def _confirm_dt(tracker, ball_id, throw_time):
    ev = [e for e in tracker.identity_events
          if e['kind'] == 'confirm' and e['ball_id'] == ball_id]
    return ev[0]['t'] - throw_time if ev else None


@pytest.mark.parametrize('ramp_s', [SLOW, FAST], ids=['slow-ramp', 'fast-ramp'])
def test_every_re_throw_confirms_at_release(ramp_s):
    """Fails without the release on the slow ramp (verified 2026-11-25): the
    previous Throw's filter follows the marker out of the Carry and keeps it
    through the whole next flight."""
    tracker, ids, throws, _ = _run(ramp_s)
    for k, tt in enumerate(throws):
        dt = _confirm_dt(tracker, ids[k], tt)
        assert dt is not None, 'Throw %d never confirmed' % k
        assert 0.0 <= dt <= 0.02, (k, dt)


def test_landed_track_releases_the_marker_in_the_carry():
    """The marker sits still in the cup after the Catch: the track lets it go
    once its frozen landing + caught_grace_s has passed and ends CAUGHT
    `missed_frames_to_lose` frames later, through the ordinary lifecycle.
    Without the release it stays IN_FLIGHT holding the marker (verified)."""
    tracker, ids, _, log = _run(SLOW, n_throws=1)
    rows = log[ids[0]]
    land = [r['landing_time'] for r in rows if r['from_fit']][-1]
    caught = [r['t'] for r in rows if r['status'] == BallStatus.CAUGHT]
    assert caught, 'the landed track never let the marker go'
    t_rel = land + tracker.caught_grace_s
    assert t_rel < caught[0] <= t_rel + (tracker.missed_frames_to_lose + 2) * DT


@pytest.mark.parametrize('ramp_s', [SLOW, FAST], ids=['slow-ramp', 'fast-ramp'])
def test_a_ball_in_flight_never_loses_its_marker(ramp_s):
    """The invariant: from confirmation to the landing plane, every Throw's
    track takes the marker on every frame. Guard against a weakened rule:
    dropping the frozen-fit or the grace condition from `_flight_over` fails
    this test (mutation checked 2026-11-25)."""
    tracker, ids, throws, log = _run(ramp_s)
    checked = 0
    for k, tt in enumerate(throws):
        for r in log[ids[k]]:
            if tt + 0.02 <= r['t'] <= tt + FLIGHT_S:
                assert r['tracking'] == TrackingConfidence.CONFIRMED, (k, r['t'])
                assert r['status'] == BallStatus.IN_FLIGHT, (k, r['t'])
                assert r['fwm'] == 0, (k, r['t'], r['fwm'])
                checked += 1
    assert checked > 3 * 150


def test_release_never_moves_a_landing_prediction():
    """Frame by frame, Throw 0's published landing (time, position, source)
    with the release is identical to a tracker that never releases, for as
    long as the ball is IN_FLIGHT: the release cannot move the Catch aim."""
    _, ids_a, _, log_a = _run(SLOW, n_throws=1)
    ref = BallTracker(dt=DT, landing_z=LANDING_Z)
    ref._flight_over = lambda *a, **k: False
    _, ids_b, _, log_b = _run(SLOW, n_throws=1, tracker=ref)
    b = {r['t']: r for r in log_b[ids_b[0]]}
    compared = 0
    for ra in log_a[ids_a[0]]:
        if ra['status'] == BallStatus.TO_BE_THROWN:
            continue
        if ra['status'] != BallStatus.IN_FLIGHT:
            break
        rb = b[ra['t']]
        assert ra['landing_time'] == rb['landing_time'], ra['t']
        assert ra['from_fit'] == rb['from_fit'], ra['t']
        assert np.array_equal(ra['landing_pos'], rb['landing_pos']), ra['t']
        compared += 1
    assert compared > 150
