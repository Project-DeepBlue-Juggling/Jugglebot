"""The two-ball association contract, replayed on sitting 3's own ``/balls``.

THE INVARIANT (``ball_possession``, "The release identity", 2026-10-04): a
tracker estimate is used only for the release it was announced for. The
correlator keys every release on the announcement's ``(source, throw_time)``
and resolves every schedule ball's latches against ONE claimed set.

The fixtures below are the per-id state changes of bag
``~/Desktop/rosbags/2026-10-02_18-11-14`` (R5 sitting 2/3, fed columns),
decoded by the U1 diagnosis (``u1_bag_263.txt`` / ``u1_bag_275.txt``, all
times ROS epoch). That bag predates ``BallState.throw_time``, so each track's
``throw_time`` is the instant its announcement carried (the launch log's own
announce lines, which the tracker copies verbatim): exactly what the fixed
tracker publishes. The announcement plan is skill_node's own (launch log
L1156-1174): when each release was announced and for which schedule ball.

Three replays per attempt, through the REAL ``ball_possession`` functions:

* ``head``    -- HEAD 0db600e8's rule: legacy latches, one claimed set PER
                 schedule ball (`advance_flight_latches` per ball). Must
                 reproduce the sitting's mis-association.
* ``fixed``   -- identity latches through `advance_correlation`: the fix.
* ``interim`` -- legacy latches through `advance_correlation` (the global
                 claimed set alone), the belt-and-braces layer.
"""
from __future__ import annotations

import math
from types import SimpleNamespace

import pytest

from jugglebot import ball_possession as bp

BASE = 1790929000.0
TBT, FLY, CAUGHT = 0, 1, 2
ANN, CONF = 0, 1
STEP_S = 0.005          # the replay's /balls cadence (the bag's is ~180 Hz)


class _T(object):
    def __init__(self, t_s):
        if t_s is None:
            self.sec, self.nanosec = 0, 0
        else:
            self.sec = int(math.floor(t_s))
            self.nanosec = int(round((t_s - self.sec) * 1e9))


def _snapshot(rows, throw_time, t):
    """Every track minted by ``t``, each at its latest recorded state."""
    latest = {}
    for (t_row, tid, status, tracking, src, fit, lx, ly, t_land) in rows:
        if BASE + t_row <= t + 1e-9:
            latest[tid] = (status, tracking, src, fit, lx, ly, t_land)
    out = []
    for tid in sorted(latest):
        status, tracking, src, fit, lx, ly, t_land = latest[tid]
        out.append(SimpleNamespace(
            id=tid, status=status, tracking=tracking, source=src,
            destination='jugglebot', landing_from_fit=bool(fit),
            throw_time=_T(throw_time.get(tid)),
            landing_position=SimpleNamespace(x=lx, y=ly, z=830.0),
            t_land=BASE + t_land))
    return out


def _replay(fixture, mode):
    """Drive the announcement plan and the snapshots through the real rule,
    as `skill_node` does: a latch is queued at its announcement with
    ``preexisting`` = the ids IN_FLIGHT in the PREVIOUS snapshot (the node's
    ``_raw_balls``), then every snapshot advances the correlation. Returns the
    final latches and, per probe instant, ``flight_in_progress`` per ball."""
    rows, throw_time, plan, probes = fixture
    plan = sorted(plan)
    probes = sorted(probes)
    corr, prev, seen = {}, [], {}
    t = BASE + rows[0][0]
    t_end = BASE + rows[-1][0] + 0.05
    while t <= t_end:
        balls = _snapshot(rows, throw_time, t)
        while plan and BASE + plan[0][0] <= t:
            _ta, ball_id, t_rel, thrower, reset = plan.pop(0)
            pre = tuple(sorted(b.id for b in prev if b.status == FLY))
            latch = bp.FlightLatch(
                t_release_s=BASE + t_rel, preexisting=pre,
                thrower=bp.announced_source(thrower) if mode == 'fixed' else None)
            corr[ball_id] = ((latch,) if reset
                             else tuple(corr.get(ball_id, ())) + (latch,))
        if mode == 'head':
            corr = {k: bp.advance_flight_latches(
                        balls, robot_name='jugglebot', latches=v, now_s=t,
                        in_flight_status=FLY)
                    for k, v in corr.items()}
        else:
            corr = bp.advance_correlation(
                balls, robot_name='jugglebot', correlation=corr, now_s=t,
                in_flight_status=FLY)
        while probes and BASE + probes[0] <= t:
            seen[probes.pop(0)] = {
                k: bp.flight_in_progress(balls, latches=v, now_s=t,
                                         in_flight_status=FLY,
                                         confirmed_tracking=CONF)
                for k, v in corr.items()}
        prev = balls
        t += STEP_S
    ids = {k: [l.announced_id for l in v] for k, v in corr.items()}
    return ids, seen


# Attempt 1790929263.8 (U1 § 2a). Ball 1 = B (BB feed, then re-thrown from
# P1), ball 0 = A (launched and re-thrown from P2). Rows: (t - BASE, id,
# status, tracking, source, from_fit, landing x, y, t_land - BASE).
ATTEMPT_263 = (
    [(263.776, 46, TBT, ANN, 'ball_butler', 0, -32, -1, 267.201),
     (266.322, 46, FLY, CONF, 'ball_butler', 0, -86, -40, 267.146),
     (266.353, 47, TBT, ANN, 'jugglebot', 0, 46, -20, 267.741),
     (266.711, 48, TBT, ANN, 'jugglebot', 0, -50, 5, 268.334),
     (266.925, 47, FLY, CONF, 'jugglebot', 0, 42, -21, 267.732),
     (267.073, 47, FLY, CONF, 'jugglebot', 1, 41, -5, 267.772),
     (267.246, 46, CAUGHT, CONF, 'ball_butler', 0, -109, -48, 266.999),
     (267.319, 49, TBT, ANN, 'jugglebot', 0, 45, -21, 268.918),
     (267.574, 48, FLY, CONF, 'jugglebot', 0, -55, 4, 268.312),
     (267.719, 48, FLY, CONF, 'jugglebot', 1, 6, 55, 268.405),
     (268.035, 47, CAUGHT, CONF, 'jugglebot', 1, 50, -1, 267.784),
     (268.113, 49, FLY, CONF, 'jugglebot', 0, 40, -15, 268.097),
     (268.310, 49, FLY, CONF, 'jugglebot', 1, 36, -17, 268.962),
     (268.637, 48, CAUGHT, CONF, 'jugglebot', 1, 37, 71, 268.388),
     (269.166, 49, CAUGHT, CONF, 'jugglebot', 1, 39, -14, 268.964)],
    # Each track's announced throw_time (BB's feed, then our three releases).
    {46: BASE + 266.300, 47: BASE + 266.912, 48: BASE + 267.513,
     49: BASE + 268.087},
    # (t_announce - BASE, schedule ball, t_release - BASE, thrower, reset)
    [(263.780, 1, 266.300, 'ball_butler', True),
     (266.333, 0, 266.912, 'jugglebot', False),
     (266.704, 1, 267.513, 'jugglebot', False),
     (267.298, 0, 268.087, 'jugglebot', False)],
    # 267.763: skill 3 (catch B at P1) dispatches -- the WINDOW_TOO_SHORT.
    [267.763],
)

# Attempt 1790929275.0 (U1 § 2b): ball 0's LAUNCH latched Ball Butler's ball.
ATTEMPT_275 = (
    [(275.063, 50, TBT, ANN, 'ball_butler', 0, -33, -1, 278.478),
     (277.595, 50, FLY, CONF, 'ball_butler', 0, -81, -38, 278.418),
     (277.602, 51, TBT, ANN, 'jugglebot', 0, 46, -20, 279.015),
     (277.692, 50, FLY, CONF, 'ball_butler', 1, -1005, -363, 278.056),
     (277.772, 50, FLY, CONF, 'ball_butler', 0, -558, -205, 278.257),
     (278.095, 52, TBT, ANN, 'jugglebot', 0, -50, 5, 279.609),
     (278.224, 51, FLY, CONF, 'jugglebot', 0, 42, -21, 279.005),
     (278.393, 51, FLY, CONF, 'jugglebot', 1, 65, -30, 279.043),
     (278.556, 50, CAUGHT, CONF, 'ball_butler', 0, -784, -285, 278.256),
     (278.596, 53, TBT, ANN, 'jugglebot', 0, 46, -20, 280.190),
     (278.845, 52, FLY, ANN, 'jugglebot', 0, -50, 5, 279.609),
     (278.859, 52, FLY, CONF, 'jugglebot', 0, -54, 4, 279.585),
     (279.007, 52, FLY, CONF, 'jugglebot', 1, 24, 23, 279.665),
     (279.332, 51, CAUGHT, CONF, 'jugglebot', 1, 78, -27, 279.059),
     (279.416, 53, FLY, ANN, 'jugglebot', 0, 46, -20, 280.190),
     (279.891, 52, CAUGHT, CONF, 'jugglebot', 1, 44, 43, 279.658),
     (280.335, 53, FLY, CONF, 'jugglebot', 0, 46, -20, 280.190),
     (280.402, 53, CAUGHT, CONF, 'jugglebot', 0, 46, -20, 280.190)],
    {50: BASE + 277.585, 51: BASE + 278.188, 52: BASE + 278.787,
     53: BASE + 279.363},
    [(275.055, 1, 277.585, 'ball_butler', True),
     (277.589, 0, 278.188, 'jugglebot', False),
     (278.079, 1, 278.787, 'jugglebot', False),
     (278.559, 0, 279.363, 'jugglebot', False)],
    # 278.5: ball 0's launch is in the air; skill 2 (A at P2) is about to aim.
    [278.5],
)


def test_head_rule_reproduces_the_sitting_3_wrong_latch():
    """The regression as flown: ball 1's re-release (B) latches A's flight
    (47) because 48 is still TO_BE_THROWN 50 ms before its release and A's
    47 is claimed only by ball 0's latches; ball 0's re-release takes B's 48
    the same way. At skill 3's dispatch the tracker hands B's catch A's
    landing -- the WINDOW_TOO_SHORT of every fed-columns attempt."""
    ids, seen = _replay(ATTEMPT_263, 'head')
    assert ids == {1: [46, 47], 0: [47, 48]}
    assert seen[267.763][1] == 47


def test_the_identity_key_latches_each_release_to_its_own_track():
    ids, seen = _replay(ATTEMPT_263, 'fixed')
    assert ids == {1: [46, 48], 0: [47, 49]}
    # Skill 3 now aims at B's own converged track (fit since 267.719,
    # lands 268.405 -- a +0.39 s window instead of -0.228 s).
    assert seen[267.763] == {1: 48, 0: 47}


def test_the_global_claimed_set_alone_also_closes_attempt_263():
    """Belt and braces: even a latch with no identity key cannot take an id
    another schedule ball already holds."""
    ids, _seen = _replay(ATTEMPT_263, 'interim')
    assert ids == {1: [46, 48], 0: [47, 49]}


def test_head_rule_hands_our_launch_ball_butlers_feed_ball():
    """U1 § 2b: ball 0's launch latch was made while BB's 50 was still TBT
    (``preexisting`` empty), so at its release it took 50 -- a ball_butler
    track -- and skill 2 later read its post-catch garbage (the logged
    ``AIM-LATERAL-CLAMPED skill 2: -830.8 mm``) and lost throw 1's row."""
    ids, seen = _replay(ATTEMPT_275, 'head')
    assert ids == {1: [50, 51], 0: [50, 52]}
    assert seen[278.5][0] == 50


@pytest.mark.parametrize('mode', ['fixed', 'interim'])
def test_our_launch_never_latches_the_feed_ball(mode):
    ids, seen = _replay(ATTEMPT_275, mode)
    assert ids == {1: [50, 52], 0: [51, 53]}
    # 50 is ball 1's (the feed, still in the air); ball 0 reads its own 51.
    assert seen[278.5] == {1: 50, 0: 51}


# ── The rule's own edges ──────────────────────────────────────────────────────

def _track(tid, status, source, throw_time_s, tracking=CONF):
    return SimpleNamespace(id=tid, status=status, tracking=tracking,
                           source=source, destination='jugglebot',
                           throw_time=_T(throw_time_s))


def test_an_id_is_never_claimed_by_two_schedule_balls():
    """Two legacy latches on DIFFERENT schedule balls, both due, one
    candidate: exactly one of them gets it (the earlier release), and the
    other stays unlatched rather than sharing it."""
    corr = {0: (bp.FlightLatch(10.0),), 1: (bp.FlightLatch(10.5),)}
    out = bp.advance_correlation(
        [_track(7, FLY, 'jugglebot', None)], robot_name='jugglebot',
        correlation=corr, now_s=10.6, in_flight_status=FLY)
    assert [out[0][0].announced_id, out[1][0].announced_id] == [7, None]
    # An id already held keeps excluding it on every later snapshot.
    out = bp.advance_correlation(
        [_track(7, FLY, 'jugglebot', None)], robot_name='jugglebot',
        correlation=out, now_s=11.0, in_flight_status=FLY)
    assert out[1][0].announced_id is None


def test_a_duplicated_identity_is_claimed_once():
    corr = {0: (bp.FlightLatch(10.0, thrower='jugglebot'),),
            1: (bp.FlightLatch(10.0, thrower='jugglebot'),)}
    out = bp.advance_correlation(
        [_track(7, TBT, 'jugglebot', 10.0)], robot_name='jugglebot',
        correlation=corr, now_s=9.0, in_flight_status=FLY)
    assert sorted([out[0][0].announced_id, out[1][0].announced_id],
                  key=lambda i: (i is None, i)) == [7, None]


def test_a_ball_butler_track_never_satisfies_our_latch():
    """Same throw_time, wrong source: refused (and the reverse)."""
    balls = [_track(50, FLY, 'ball_butler', 10.0)]
    ours = bp.advance_correlation(
        balls, robot_name='jugglebot',
        correlation={0: (bp.FlightLatch(10.0, thrower='jugglebot'),)},
        now_s=10.5, in_flight_status=FLY)
    assert ours[0][0].announced_id is None
    bbs = bp.advance_correlation(
        balls, robot_name='jugglebot',
        correlation={1: (bp.FlightLatch(10.0, thrower='ball_butler'),)},
        now_s=10.5, in_flight_status=FLY)
    assert bbs[1][0].announced_id == 50


def test_an_identity_latch_ignores_the_other_balls_flight_at_its_release():
    """The exact sitting-3 instant, reduced: 50 ms before our release the
    other ball (own throw_time a beat earlier) is IN_FLIGHT and unclaimed,
    our own track still TO_BE_THROWN. The identity latch takes ours."""
    beat = 0.579
    balls = [_track(47, FLY, 'jugglebot', 10.0 - beat),
             _track(48, TBT, 'jugglebot', 10.0)]
    out = bp.advance_flight_latches(
        balls, robot_name='jugglebot',
        latches=(bp.FlightLatch(10.0, thrower='jugglebot'),),
        now_s=10.0 - 0.05, in_flight_status=FLY)
    assert out[0].announced_id == 48
    # It is final: a later snapshot cannot move it.
    out = bp.advance_flight_latches(
        [_track(47, FLY, 'jugglebot', 10.0 - beat)], robot_name='jugglebot',
        latches=out, now_s=10.1, in_flight_status=FLY)
    assert out[0].announced_id == 48


def test_a_throw_time_off_by_more_than_the_tolerance_is_not_ours():
    tol = bp.THROW_TIME_MATCH_TOL_S
    assert bp.match_announced_track(
        [_track(7, FLY, 'jugglebot', 10.0 + 0.9 * tol)],
        thrower='jugglebot', t_release_s=10.0) == 7
    assert bp.match_announced_track(
        [_track(7, FLY, 'jugglebot', 10.0 + 1.5 * tol)],
        thrower='jugglebot', t_release_s=10.0) is None
    assert bp.match_announced_track(
        [_track(7, FLY, 'jugglebot', None)],
        thrower='jugglebot', t_release_s=10.0) is None
