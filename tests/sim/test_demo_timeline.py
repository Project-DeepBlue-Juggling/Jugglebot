"""Unit tests for the juggling-demo master timeline (Phase 3b).

Covers controller/demo/timeline.py and the abort extension in
controller/demo/player.py. Validates the T-U8 (event ordering and
content) and T-U9 (abort + exit transient) tests of
plans/active/bb-led-two-ball-juggle-demo.md §5.
"""
import numpy as np
import pytest

from controller.demo.pattern import JugglePattern
from controller.demo.trajectory import build_analytic_oval
from controller.demo.timeline import (
    Event, ExitTransient, MasterTimeline,
    HAND_TRAJ_THROW, HAND_TRAJ_CATCH, HAND_TRAJ_NA,
    build_exit_transient,
)
from controller.demo.player import TrajectoryPlayer


# --------------------------------------------------------------------------
# Fixture helper
# --------------------------------------------------------------------------
def _build(t0=1000.0, n_catches=10):
    pattern = JugglePattern()
    traj = build_analytic_oval(pattern)
    tl = MasterTimeline(pattern, traj, t0, n_catches=n_catches)
    return pattern, traj, tl


# --------------------------------------------------------------------------
# T-U8: event ordering and content
# --------------------------------------------------------------------------
def test_events_are_strictly_time_ordered():
    """Every event's t_wall is >= the previous one (sort stability)."""
    _, _, tl = _build()
    ts = [e.t_wall for e in tl.all_events()]
    assert ts == sorted(ts)


def test_event_counts_and_kinds():
    """1 BB priming + n_catches throws + n_catches catches."""
    _, _, tl = _build(n_catches=8)
    evs = tl.all_events()
    assert len(evs) == 1 + 2 * 8
    kinds = [e.kind for e in evs]
    assert kinds.count('bb_throw') == 1
    assert kinds.count('hand_throw') == 8
    assert kinds.count('hand_catch') == 8


def test_throws_and_catches_alternate_after_priming():
    """Past the BB priming event, the hand stream alternates throw/catch."""
    _, _, tl = _build()
    hand_only = [e for e in tl.all_events() if e.kind != 'bb_throw']
    for i, e in enumerate(hand_only):
        expected = 'hand_throw' if i % 2 == 0 else 'hand_catch'
        assert e.kind == expected, f"event {i} was {e.kind}, expected {expected}"


def test_throw_event_spacing_is_one_platform_period():
    """Successive throw events are exactly one platform period apart."""
    p, _, tl = _build()
    throws = [e for e in tl.all_events() if e.kind == 'hand_throw']
    spacings = np.diff([e.t_wall for e in throws])
    np.testing.assert_allclose(spacings, p.platform_period_s, atol=1e-9)


def test_catch_offset_defaults_to_half_platform_period():
    """Default catch_offset_s == platform_period / 2 (the analytic oval CATCH)."""
    p, _, tl = _build()
    P = p.platform_period_s
    assert tl.catch_offset_s == pytest.approx(0.5 * P)
    throws = [e for e in tl.all_events() if e.kind == 'hand_throw']
    catches = [e for e in tl.all_events() if e.kind == 'hand_catch']
    for thr, cat in zip(throws, catches):
        # Catch follows its paired throw by exactly catch_offset_s.
        assert cat.t_wall - thr.t_wall == pytest.approx(0.5 * P, abs=1e-9)


def test_catch_offset_override_is_honoured():
    """A non-default catch_offset_s shifts every catch event by the new offset."""
    p = JugglePattern()
    traj = build_analytic_oval(p)
    P = p.platform_period_s
    custom_offset = p.throw_stroke_s + p.transit_window_s   # Phase-1 hand tempo
    assert 0.0 < custom_offset < P
    tl = MasterTimeline(p, traj, t0=0.0, n_catches=6,
                        catch_offset_s=custom_offset)
    assert tl.catch_offset_s == pytest.approx(custom_offset)
    for k, cat in enumerate(e for e in tl.all_events() if e.kind == 'hand_catch'):
        assert cat.t_wall == pytest.approx(custom_offset + k * P, abs=1e-9)


def test_per_ball_throw_offset_is_platform_period():
    """The two balls' throw streams are interleaved by one platform period.

    Phase 1 / plan: "per-ball phase offset = P/2" where P = ball_period; for
    the platform-period (= ball_period / 2) the same-ball spacing is 2*P
    and consecutive throws (alternating ball) are one P apart.
    """
    p, _, tl = _build()
    P = p.platform_period_s
    throws = [e for e in tl.all_events() if e.kind == 'hand_throw']
    # Consecutive throws alternate ball and are spaced by exactly P.
    for prev, nxt in zip(throws, throws[1:]):
        assert prev.ball != nxt.ball
        assert nxt.t_wall - prev.t_wall == pytest.approx(P, abs=1e-9)
    # Same-ball throws are spaced by 2 * P.
    ball0 = [e.t_wall for e in throws if e.ball == 0]
    ball1 = [e.t_wall for e in throws if e.ball == 1]
    np.testing.assert_allclose(np.diff(ball0), 2 * P, atol=1e-9)
    np.testing.assert_allclose(np.diff(ball1), 2 * P, atol=1e-9)


def test_first_throw_is_pre_held_ball_first_catch_is_bb_ball():
    """k=0 throws ball 1 (Jugglebot's pre-held ball); k=0 catches ball 0 (BB)."""
    _, _, tl = _build()
    throws = [e for e in tl.all_events() if e.kind == 'hand_throw']
    catches = [e for e in tl.all_events() if e.kind == 'hand_catch']
    assert throws[0].ball == 1
    assert catches[0].ball == 0


def test_bb_priming_event_precedes_t0_by_bb_lead():
    """BB priming throw issues at t0 - bb_lead_s."""
    pattern = JugglePattern()
    traj = build_analytic_oval(pattern)
    tl = MasterTimeline(pattern, traj, t0=500.0, n_catches=5, bb_lead_s=2.0)
    bb = next(e for e in tl.all_events() if e.kind == 'bb_throw')
    assert bb.t_wall == pytest.approx(498.0)


def test_event_traj_types_match_can_protocol():
    """Hand throw -> 1, hand catch -> 0, BB throw -> -1 (NA)."""
    _, _, tl = _build()
    for e in tl.all_events():
        if e.kind == 'hand_throw':
            assert e.traj_type == HAND_TRAJ_THROW == 1
        elif e.kind == 'hand_catch':
            assert e.traj_type == HAND_TRAJ_CATCH == 0
        elif e.kind == 'bb_throw':
            assert e.traj_type == HAND_TRAJ_NA == -1


def test_event_velocities_equal_pattern_throw_speed():
    """Symmetric flight: release speed == arrival speed == pattern.throw_speed_mps."""
    p, _, tl = _build()
    for e in tl.all_events():
        assert e.event_vel == pytest.approx(p.throw_speed_mps)


# --------------------------------------------------------------------------
# due_events cursor
# --------------------------------------------------------------------------
def test_due_events_returns_only_passed_events_in_order():
    """Cursor walks forward; events are returned in time order, no duplicates."""
    _, _, tl = _build()
    huge = max(e.t_wall for e in tl.all_events()) + 1.0
    drained = tl.due_events(huge)
    assert [e.t_wall for e in drained] == sorted(e.t_wall for e in drained)
    assert len(drained) == tl.n_events


def test_due_events_cursor_is_idempotent_per_event():
    """Repeated calls do not re-emit already-returned events."""
    _, _, tl = _build()
    first_due = tl.due_events(tl.t0 - 0.5)   # captures the BB event only
    assert len(first_due) == 1
    assert first_due[0].kind == 'bb_throw'
    # second call with the same wall-time yields no duplicates
    assert tl.due_events(tl.t0 - 0.5) == []
    # extending t_wall yields only the new events
    next_due = tl.due_events(tl.t0 + 1e-9)
    assert len(next_due) == 1
    assert next_due[0].kind == 'hand_throw'


# --------------------------------------------------------------------------
# T-U9: abort and exit transient
# --------------------------------------------------------------------------
def test_abort_truncates_future_events_past_t_wall():
    """After abort(t_wall), no scheduled event sits at or beyond t_wall."""
    p, _, tl = _build()
    P = p.platform_period_s
    # Pull some early events to advance the cursor (verifies head/tail split).
    tl.due_events(tl.t0 + 0.5 * P)
    abort_t = tl.t0 + 2.5 * P
    tl.abort(abort_t)
    assert all(e.t_wall < abort_t for e in tl.all_events())
    # No future hand throws or BB throws past abort.
    assert not any(e.kind in ('hand_throw', 'bb_throw') and e.t_wall >= abort_t
                   for e in tl.all_events())


def test_abort_preserves_already_dispatched_events():
    """The head of the schedule (already returned) is not retroactively dropped."""
    p, _, tl = _build()
    P = p.platform_period_s
    # Pull events up through the first catch.
    pulled = tl.due_events(tl.t0 + 0.5 * P + 1e-6)
    assert len(pulled) >= 2
    abort_t = tl.t0 + 0.5 * P + 1e-6   # abort just past the first catch
    tl.abort(abort_t)
    # The pulled events are still present in all_events (head is preserved).
    assert all(e in tl.all_events() for e in pulled)


def test_abort_returns_exit_transient_landing_at_stow():
    """The exit transient ends at the stow pose with zero twist and accel."""
    _, _, tl = _build()
    ex = tl.abort(tl.t0 + 0.4)
    assert isinstance(ex, ExitTransient)
    # Far past the end, eval clamps to the stow boundary.
    pose, twist, accel = ex.eval(ex.t_rel_end + 2.0)
    np.testing.assert_allclose(pose, 0.0, atol=1e-9)
    np.testing.assert_allclose(twist, 0.0, atol=1e-9)
    np.testing.assert_allclose(accel, 0.0, atol=1e-9)


def test_exit_transient_is_c2_at_the_abort_instant():
    """At t_rel_start the transient matches the trajectory in pose/twist/accel."""
    _, traj, tl = _build()
    abort_t = tl.t0 + 0.31    # mid-period
    ex = tl.abort(abort_t)
    expected = traj.eval(ex.t_rel_start)
    got = ex.eval(ex.t_rel_start)
    for a, b in zip(got, expected):
        np.testing.assert_allclose(a, b, atol=1e-9)


def test_abort_at_t0_starts_exit_from_throw_pose():
    """abort(t0) -> exit transient starts at the THROW keyframe."""
    p, _, tl = _build()
    ex = tl.abort(tl.t0)
    pose_start, _, _ = ex.eval(0.0)
    np.testing.assert_allclose(pose_start, p.throw_pose, atol=1e-6)


def test_exit_transient_eval_clamps_before_start():
    """eval before t_rel_start clamps to the segment-start state."""
    _, _, tl = _build()
    ex = tl.abort(tl.t0 + 0.4)
    early = ex.eval(ex.t_rel_start - 1.0)
    at_start = ex.eval(ex.t_rel_start)
    for a, b in zip(early, at_start):
        np.testing.assert_allclose(a, b, atol=1e-12)


def test_abort_is_idempotent():
    """Two calls return the same ExitTransient; the second call is a no-op."""
    _, _, tl = _build()
    ex1 = tl.abort(tl.t0 + 0.4)
    ex2 = tl.abort(tl.t0 + 0.9)   # different t_wall — second call ignored
    assert ex1 is ex2
    # The exit transient's start corresponds to the FIRST abort, not the second.
    assert ex1.t_rel_start == pytest.approx(0.4)


def test_build_exit_transient_helper_matches_constructor_path():
    """The module-level helper produces an equivalent transient to MasterTimeline.abort."""
    p = JugglePattern()
    traj = build_analytic_oval(p)
    ex_direct = build_exit_transient(traj, t_rel_abort=0.5)
    tl = MasterTimeline(p, traj, t0=0.0, n_catches=4)
    ex_via_abort = tl.abort(0.5)
    # Same boundary state at start, same end pose.
    for a, b in zip(ex_direct.eval(0.5), ex_via_abort.eval(0.5)):
        np.testing.assert_allclose(a, b, atol=1e-12)
    np.testing.assert_allclose(ex_direct.stow_pose, ex_via_abort.stow_pose,
                               atol=1e-12)


# --------------------------------------------------------------------------
# Player abort-path integration
# --------------------------------------------------------------------------
def test_player_aborted_property_flips_on_abort():
    p = JugglePattern()
    traj = build_analytic_oval(p)
    tl = MasterTimeline(p, traj, t0=0.0, n_catches=4)
    player = TrajectoryPlayer(traj)
    assert player.aborted is False
    player.abort(tl.abort(0.3))
    assert player.aborted is True


def test_player_command_pre_abort_tracks_trajectory():
    """Before abort the player faithfully follows the periodic trajectory."""
    p = JugglePattern()
    traj = build_analytic_oval(p)
    player = TrajectoryPlayer(traj)
    cmd = player.command_at(0.2)
    np.testing.assert_allclose(cmd.pose, traj.eval(0.2)[0], atol=1e-12)


def test_player_command_post_abort_traces_exit_transient():
    """After abort, command_at sources from the exit transient — ends at stow."""
    p = JugglePattern()
    traj = build_analytic_oval(p)
    tl = MasterTimeline(p, traj, t0=0.0, n_catches=4)
    player = TrajectoryPlayer(traj)
    ex = tl.abort(0.3)
    player.abort(ex)
    # Past the end of the transient, the platform sits at stow with zero twist.
    far = ex.t_rel_end + 0.5
    cmd = player.command_at(far)
    np.testing.assert_allclose(cmd.pose, np.zeros(6), atol=1e-9)
    np.testing.assert_allclose(cmd.twist, np.zeros(6), atol=1e-9)
    np.testing.assert_allclose(cmd.vel_mm_s, np.zeros(6), atol=1e-6)


def test_player_lookahead_after_abort_uses_exit_transient():
    """Lookahead at t_rel + dt / + 2dt also routes through the exit transient."""
    p = JugglePattern()
    traj = build_analytic_oval(p)
    tl = MasterTimeline(p, traj, t0=0.0, n_catches=4)
    player = TrajectoryPlayer(traj, control_dt=0.025)
    ex = tl.abort(0.3)
    player.abort(ex)
    # In the middle of the transient, the lookahead poses match exit_transient.eval.
    t = ex.t_rel_start + 0.1
    cmd = player.command_at(t)
    expected_next = ex.eval(t + 0.025)[0]
    expected_next2 = ex.eval(t + 2 * 0.025)[0]
    # IK-roundtrip these to compare extensions.
    from jugglebot.motion.ik_solver import (
        pose_to_leg_lengths, rotvec_to_rot_matrix)
    from jugglebot.motion.geometry import StewartGeometry
    geom = StewartGeometry()
    ref_next = pose_to_leg_lengths(expected_next[:3],
                                   rotvec_to_rot_matrix(expected_next[3:]),
                                   geom)
    ref_next2 = pose_to_leg_lengths(expected_next2[:3],
                                    rotvec_to_rot_matrix(expected_next2[3:]),
                                    geom)
    np.testing.assert_allclose(cmd.ext_next_mm, ref_next, atol=1e-9)
    np.testing.assert_allclose(cmd.ext_next2_mm, ref_next2, atol=1e-9)


def test_player_abort_rejects_none():
    p = JugglePattern()
    traj = build_analytic_oval(p)
    player = TrajectoryPlayer(traj)
    with pytest.raises(ValueError):
        player.abort(None)


# --------------------------------------------------------------------------
# Constructor validation
# --------------------------------------------------------------------------
def test_invalid_constructor_args_raise():
    p = JugglePattern()
    traj = build_analytic_oval(p)
    with pytest.raises(ValueError):
        MasterTimeline(p, traj, 0.0, n_catches=0)
    with pytest.raises(ValueError):
        MasterTimeline(p, traj, 0.0, bb_lead_s=-1.0)
    with pytest.raises(ValueError):
        MasterTimeline(p, traj, 0.0, exit_duration_s=0.0)
    with pytest.raises(ValueError):
        MasterTimeline(p, traj, 0.0, catch_offset_s=0.0)
    with pytest.raises(ValueError):
        MasterTimeline(p, traj, 0.0, catch_offset_s=p.platform_period_s + 0.1)


def test_exit_transient_rejects_bad_inputs():
    zero6 = np.zeros(6)
    with pytest.raises(ValueError):
        ExitTransient(0.0, -1.0, zero6, zero6, zero6, zero6)
    with pytest.raises(ValueError):
        ExitTransient(0.0, 1.0, np.zeros(5), zero6, zero6, zero6)


def test_event_is_a_namedtuple_with_expected_fields():
    """Event is a public NamedTuple — the orchestrator destructures it."""
    e = Event(t_wall=1.0, kind='hand_throw', traj_type=1,
              event_vel=5.0, ball=0)
    assert e.t_wall == 1.0 and e.kind == 'hand_throw'
    assert e.traj_type == 1 and e.event_vel == 5.0 and e.ball == 0
    # iteration order matches the NamedTuple field order
    assert tuple(e) == (1.0, 'hand_throw', 1, 5.0, 0)
