"""reload_coordinator_node tests (Phase 7) — the thin ROS wrapper.

The FSM itself is exhaustively covered by test_reload_sequencer.py; here we test the
node's seams: the wiring surface (action / clients / subscriptions), observation
assembly from cached messages (fresh vs stale), and that _step_sequence executes the
FSM's requested action (call_reload / send_throw) and publishes phase feedback.

ROS 2 is mocked by tests/ros/conftest.py.
"""

from __future__ import annotations

import time
import types

import pytest

from jugglebot.reload_coordinator_node import ReloadCoordinatorNode
from jugglebot.reload_sequencer import (
    ACTION_CALL_RELOAD,
    ACTION_SEND_THROW,
    BB_STATE_IDLE,
    BB_STATE_THROWING,
    ReloadSequencer,
)


class _Hb:
    def __init__(self, connected=True, state=BB_STATE_IDLE, ball_in_hand=True):
        self.connected = connected
        self.state = state
        self.ball_in_hand = ball_in_hand


class _Pos:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


class _Ball:
    def __init__(self, status, destination='jugglebot', x=0.0, y=0.0, z=744.3, id=5):
        self.id = id
        self.status = status
        self.destination = destination
        self.position = _Pos(x, y, z)


def _node_fresh(now):
    """A node with every observation cached FRESH at `now` and preconditions met."""
    node = ReloadCoordinatorNode()
    with node._lock:
        node._hb = _Hb()
        node._hb_mono = now
        node._control_mode = 'CATCH'
        node._streaming = True
        node._mocap_mono = now
        node._balls = []
        node._balls_mono = now
    return node


# ── Wiring surface ─────────────────────────────────────────────

def test_wiring_surface():
    node = ReloadCoordinatorNode()
    assert 'bb/heartbeat' in node._subscriptions
    assert 'control_mode_topic' in node._subscriptions
    assert 'trajectory/status' in node._subscriptions
    assert 'rigid_body_poses' in node._subscriptions
    assert 'balls' in node._subscriptions
    assert 'throw_announcements' in node._subscriptions
    assert 'trajectory/target_feedback' in node._subscriptions
    assert 'bb/reload' in node._clients
    assert 'bb/throw_at_target' in node._clients
    assert 'jugglebot/reload' in node._action_servers


def test_catch_point_includes_hand_cup_offset():
    """The reload aim must be the CUP plane (platform centroid + HAND_CATCH_OFFSET_MM),
    matching throw_ballistics/_landing_z/catch_coordinator — NOT the bare centroid.
    Regression for the 2026-07-20 fix: aiming at the centroid (744.3) delivered the ball
    64.78 mm below where the hand intercepts it (809.08)."""
    import jugglebot.hardware_config as hw
    node = ReloadCoordinatorNode()
    expected_z = (hw.GEOM_INITIAL_HEIGHT_MM + hw.JB_OP_DEFAULT_ACTIVE_Z_MM
                  + hw.HAND_CATCH_OFFSET_MM)
    assert node._catch_point_mm[0] == pytest.approx(0.0)
    assert node._catch_point_mm[1] == pytest.approx(0.0)
    assert node._catch_point_mm[2] == pytest.approx(expected_z)
    assert node._catch_point_mm[2] == pytest.approx(809.08)


# ── Observation assembly ───────────────────────────────────────

def test_build_observations_fresh():
    now = 100.0
    node = _node_fresh(now)
    obs = node._build_observations(now)
    assert obs.control_mode == 'CATCH'
    assert obs.bb_connected is True
    assert obs.bb_state == BB_STATE_IDLE
    assert obs.ball_in_hand is True
    assert obs.mocap_fresh is True
    assert obs.streaming is True
    assert obs.ball_caught is False


def test_build_observations_stale_mocap_and_heartbeat():
    now = 100.0
    node = _node_fresh(now)
    # Age everything past the freshness windows.
    obs = node._build_observations(now + 1.0)
    assert obs.mocap_fresh is False
    assert obs.bb_connected is False        # stale heartbeat → not connected
    assert obs.bb_state == 0                # stale → conservative BOOT


def test_build_observations_detects_caught_ball_with_error():
    now = 100.0
    node = _node_fresh(now)
    # The tracker first puts OUR ball in flight → the coordinator latches its id.
    with node._lock:
        node._balls = [_Ball(status=1, id=5)]             # IN_FLIGHT
        node._balls_mono = now
    node._build_observations(now)                          # latches announced id = 5
    # Then it's caught, 25 mm off centre.
    with node._lock:
        node._balls = [_Ball(status=2, id=5, x=15.0, y=20.0)]   # CAUGHT
        node._balls_mono = now
    obs = node._build_observations(now)
    assert obs.ball_caught is True
    assert obs.catch_error_mm == pytest.approx(25.0, abs=1e-6)


def test_build_observations_ignores_other_robots_ball():
    now = 100.0
    node = _node_fresh(now)
    with node._lock:
        node._balls = [_Ball(status=2, id=5, destination='someone_else')]
        node._balls_mono = now
    obs = node._build_observations(now)
    assert obs.ball_caught is False


def test_build_observations_stray_caught_ball_different_id_ignored():
    """A caught ball whose id is NOT our announced ball's (e.g. a leftover from a prior
    throw) does not confirm the reload's catch (fix 9)."""
    now = 100.0
    node = _node_fresh(now)
    # Our ball (id 5) goes in flight → latched.
    with node._lock:
        node._balls = [_Ball(status=1, id=5)]
        node._balls_mono = now
    node._build_observations(now)
    # A STRAY caught ball (id 99) appears while ours is still airborne.
    with node._lock:
        node._balls = [_Ball(status=2, id=99, x=0.0, y=0.0), _Ball(status=1, id=5)]
        node._balls_mono = now
    obs = node._build_observations(now)
    assert obs.ball_caught is False


# ── _step_sequence executes actions + publishes feedback ───────

class _GoalHandle:
    def __init__(self):
        self.feedbacks = []
        self.is_cancel_requested = False

    def publish_feedback(self, fb):
        self.feedbacks.append(fb.phase)


def test_step_sequence_calls_reload_and_publishes_feedback(monkeypatch):
    now = 100.0
    node = _node_fresh(now)
    with node._lock:              # empty hand → FSM asks for a reload
        node._hb = _Hb(ball_in_hand=False)
        node._hb_mono = now
    calls = []
    monkeypatch.setattr(node, '_call_reload', lambda: calls.append('reload') or True)
    seq = ReloadSequencer(catch_point_mm=node._catch_point_mm, throw_delay_s=3.0)
    seq.start(now)
    gh = _GoalHandle()
    decision = node._step_sequence(seq, now, gh)
    assert decision.action == ACTION_CALL_RELOAD
    assert calls == ['reload']
    assert gh.feedbacks == ['CHECKING']


def test_step_sequence_sends_throw_and_feeds_result(monkeypatch):
    now = 100.0
    node = _node_fresh(now)            # ball in hand → FSM sends the throw
    sent = []
    monkeypatch.setattr(node, '_send_throw',
                        lambda s: (sent.append(s.throw_delay_s) or (True, 'ok')))
    seq = ReloadSequencer(catch_point_mm=node._catch_point_mm, throw_delay_s=3.0)
    seq.start(now)
    gh = _GoalHandle()
    decision = node._step_sequence(seq, now, gh)
    assert decision.action == ACTION_SEND_THROW
    assert sent == [3.0]
    # The BB accept was fed back into the FSM → next step is THROW_PENDING.
    d2 = node._step_sequence(seq, now + 0.1, gh)
    assert d2.phase == 'THROW_PENDING'


# ── Async event forwarding to the active sequencer ─────────────

def test_announcement_forwards_to_active_sequencer():
    node = ReloadCoordinatorNode()
    seq = ReloadSequencer(catch_point_mm=node._catch_point_mm, throw_delay_s=3.0)
    seq.start(0.0)
    seq.step(0.0, _obs_stub())        # sends throw
    seq.note_throw_result(True)
    seq.step(0.1, _obs_stub())        # → THROW_PENDING
    with node._lock:
        node._active_seq = seq
    ann = types.SimpleNamespace(target_id='jugglebot')
    node._on_announcement(ann)
    d = seq.step(0.2, _obs_stub())
    assert d.phase == 'BALL_IN_FLIGHT'


def test_announcement_ignored_when_no_active_sequence():
    node = ReloadCoordinatorNode()          # no active sequence
    ann = types.SimpleNamespace(target_id='jugglebot')
    node._on_announcement(ann)              # must not raise


def test_target_feedback_reject_forwards_infeasibility():
    node = ReloadCoordinatorNode()
    seq = ReloadSequencer(catch_point_mm=node._catch_point_mm, throw_delay_s=3.0)
    seq.start(0.0)
    seq.step(0.0, _obs_stub())
    seq.note_throw_result(True)
    seq.step(0.1, _obs_stub())
    seq.note_announcement(0.5)
    seq.step(0.5, _obs_stub())              # BALL_IN_FLIGHT
    with node._lock:
        node._active_seq = seq
    fb = types.SimpleNamespace(source='catch', accepted=False, code='WORKSPACE')
    node._on_target_feedback(fb)
    d = seq.step(1.0, _obs_stub())
    assert d.done and d.result.outcome == 'MISSED_INFEASIBLE_WORKSPACE'


def test_target_feedback_frozen_and_stale_are_ignored():
    """trajectory_node emits FROZEN for every late catch target in the reach-freeze
    window and STALE_STATE on transient races — the node must drop them so the catch
    proceeds (fix 5)."""
    node = ReloadCoordinatorNode()
    seq = ReloadSequencer(catch_point_mm=node._catch_point_mm, throw_delay_s=3.0)
    seq.start(0.0)
    seq.step(0.0, _obs_stub())
    seq.note_throw_result(True)
    seq.step(0.1, _obs_stub())
    seq.note_announcement(0.5)
    seq.step(0.5, _obs_stub())              # BALL_IN_FLIGHT
    with node._lock:
        node._active_seq = seq
    for code in ('FROZEN', 'STALE_STATE'):
        node._on_target_feedback(
            types.SimpleNamespace(source='catch', accepted=False, code=code))
    d = seq.step(1.0, _obs_stub())
    assert not d.done                       # neither latched MISSED_INFEASIBLE


# ── Concurrent-goal rejection (fix 4) ──────────────────────────

def test_second_goal_rejected_while_one_active():
    from rclpy.action import GoalResponse
    node = ReloadCoordinatorNode()
    # No active sequence → accept.
    assert node._goal_callback(object()) == GoalResponse.ACCEPT
    # A reload in progress → reject the concurrent goal (would double-throw).
    with node._lock:
        node._active_seq = ReloadSequencer(catch_point_mm=node._catch_point_mm)
    assert node._goal_callback(object()) == GoalResponse.REJECT


def _obs_stub():
    from jugglebot.reload_sequencer import ReloadObservations, BB_STATE_IDLE
    return ReloadObservations(
        now=0.0, control_mode='CATCH', bb_connected=True, bb_state=BB_STATE_IDLE,
        ball_in_hand=True, mocap_fresh=True, streaming=True)
