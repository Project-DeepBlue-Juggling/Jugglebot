"""The /robot_state honesty gate on teensy_bridge_node.

``_publish_robot_state`` runs on a 100 Hz timer; ``TELEMETRY`` arrives at ~100 Hz
from the can-bridge. The two are INDEPENDENT, so they beat: some timer periods
carry two telemetry frames and some carry none, and on the "none" periods the
publisher used to re-serialise the SAME ``_latest_telemetry`` latch under a fresh
header stamp.

WHY THAT MATTERED. The republished message is content-identical to its
predecessor, and in a bag that is indistinguishable from the robot genuinely
holding still. It is the whole explanation for the "97 % identical consecutive
samples" reading that looked like a plant observation and was in fact a property
of this publisher. An analysis that treats those rows as measurements
double-counts a sample it never took — which is precisely the class of error the
bridge-temporal arc exists to eliminate, so the publisher may not be the one
introducing it.

THE FIX AND ITS ONE CARVE-OUT. The publish is skipped when the telemetry
generation has not advanced since the last one, and the skips are counted on
``/link_status`` as ``robot_state_stale_skips`` so the suppression is visible
rather than silent. The carve-out is link loss: when the UDP link dies telemetry
stops, so the generation freezes — and a freshness gate alone would suppress
``/robot_state`` at exactly the moment ``has_fatal_can_error`` is being raised
BECAUSE the link died. No consumer watchdogs this topic's ARRIVAL (the
orchestrator's liveness input is the message content), so that would convert the
loudest failure the bridge can report into total silence: strictly worse than the
duplicate the gate removes. This module pins both halves, because a future
simplification that "tidies away" the carve-out would be silent in production
until the day the link actually drops.

ROS 2 is mocked by tests/ros/conftest.py.
"""

from __future__ import annotations

import pytest

from teensy_link import Telemetry
from teensy_link import protocol as p

from tests.ros._bridge_harness import (
    _build_paired_node,
    _teardown,
    _wait_until,
)


@pytest.fixture
def bridge():
    teensy, client, node = _build_paired_node()
    yield teensy, node
    _teardown(teensy, client, node)


def _telem(pos=None):
    if pos is None:
        pos = tuple(float(i) for i in range(p.NUM_AXES))
    return Telemetry(t_teensy_us=123, pos_rev=tuple(pos),
                     vel_rps=tuple(0.0 for _ in range(p.NUM_AXES)))


def _land(teensy, node, pos=None):
    """Send one telemetry frame and wait for the GENERATION to advance.

    Waiting on the generation rather than on ``_latest_telemetry is not None``:
    the latter short-circuits from the second call onward and never proves the
    new frame landed.
    """
    before = node._telemetry_gen
    t = _telem(pos)
    teensy.send_telemetry(pos_rev=t.pos_rev, vel_rps=t.vel_rps)
    assert _wait_until(lambda: node._telemetry_gen > before), (
        'the telemetry frame never reached the node')


def test_a_fresh_latch_publishes(bridge):
    """The normal path is untouched: one new frame, one published message."""
    teensy, node = bridge
    _land(teensy, node)
    node._publish_robot_state()
    assert len(node.robot_state_pub.published) == 1
    assert node._robot_state_stale_skips == 0


def test_a_duplicate_latch_is_skipped_and_counted(bridge):
    """A publish with no new telemetry is suppressed, and the skip is COUNTED.

    Counted, not merely suppressed: a silently-dropped publish is
    indistinguishable from a publisher that stopped working, and the skip rate is
    itself the diagnostic (a steady trickle means the two ~100 Hz rates are
    beating normally; a rate approaching the full 100/s means telemetry slowed or
    stopped).
    """
    teensy, node = bridge
    _land(teensy, node)
    node._publish_robot_state()
    assert len(node.robot_state_pub.published) == 1

    # Three more timer ticks with no new telemetry frame in between.
    for _ in range(3):
        node._publish_robot_state()
    assert len(node.robot_state_pub.published) == 1, (
        'the publisher republished an unchanged telemetry latch — that row is '
        'a fabricated sample, not a measurement')
    assert node._robot_state_stale_skips == 3


def test_each_new_frame_publishes_exactly_once(bridge):
    """N telemetry frames, many timer ticks ⇒ N messages, each with its own data.

    This is the property that makes the published series follow the TELEMETRY
    rate instead of the timer rate: the mean stays ~100 Hz while the duplicate
    rows disappear.
    """
    teensy, node = bridge
    for i in range(3):
        _land(teensy, node, pos=tuple(float(i) for _ in range(p.NUM_AXES)))
        node._publish_robot_state()
        node._publish_robot_state()      # a beat with no new frame
    published = node.robot_state_pub.published
    assert len(published) == 3
    assert node._robot_state_stale_skips == 3
    # Each message carries ITS OWN frame's data, so the gate is not just
    # dropping messages but keeping the right ones.
    assert [m.motor_states[0].pos_estimate for m in published] == [0.0, 1.0, 2.0]


def test_suppression_is_bounded_when_telemetry_dies_but_heartbeats_live(bridge):
    """THE DOUBLE-FAILURE BOUND. Telemetry dead + link nominally up ⇒ the gate
    yields after _ROBOT_STATE_MAX_CONSEC_SKIPS (~0.5 s), it does not silence
    the topic forever.

    The firmware emits TELEMETRY and HEARTBEAT_T2J from separate tasks, so a
    wedged task_telem freezes the generation while heartbeats keep link_lost
    False — and the orchestrator's fatal-fault inputs ride this topic's
    CONTENT. An unbounded gate would convert that double-failure into total
    silence on the fault channel.
    """
    from jugglebot.teensy_bridge_node import _ROBOT_STATE_MAX_CONSEC_SKIPS
    teensy, node = bridge
    _land(teensy, node)
    node._publish_robot_state()
    assert len(node.robot_state_pub.published) == 1

    # Telemetry never advances again; the link stays up. Run well past the cap.
    for _ in range(_ROBOT_STATE_MAX_CONSEC_SKIPS + 5):
        node._publish_robot_state()
    published = len(node.robot_state_pub.published)
    assert published == 2, (
        'expected exactly one bounded-suppression publish after the cap '
        f'({_ROBOT_STATE_MAX_CONSEC_SKIPS} consecutive skips), got '
        f'{published - 1} extra messages')
    assert node._robot_state_stale_skips == _ROBOT_STATE_MAX_CONSEC_SKIPS + 4


def test_a_lost_link_keeps_publishing_despite_a_frozen_latch(bridge):
    """THE CARVE-OUT. Link lost ⇒ publish anyway, so the fatal is delivered.

    When the link dies, telemetry stops and the generation freezes forever. A
    freshness gate alone would then silence ``/robot_state`` at precisely the
    moment ``has_fatal_can_error`` is being raised BECAUSE of that link loss —
    and since no consumer watchdogs this topic's arrival, the orchestrator would
    simply never learn. That is a strictly worse failure than the duplicate row
    the gate exists to remove, which is why the condition is
    "unchanged AND link up" rather than "unchanged".
    """
    teensy, node = bridge
    _land(teensy, node)
    node._publish_robot_state()
    assert len(node.robot_state_pub.published) == 1

    # Arm the latch the way _health_check would on a real outage.
    node._link_latch.first_heartbeat_seen = True
    node._link_latch.link_lost = True

    for _ in range(4):
        node._publish_robot_state()
    assert len(node.robot_state_pub.published) == 5, (
        'the honesty gate suppressed /robot_state while the link was DOWN — '
        'the fatal link-loss report would never reach the orchestrator')
    # No skips were counted: nothing was skipped.
    assert node._robot_state_stale_skips == 0
    # And the messages carry the fatal, which is the entire point of publishing.
    last = node.robot_state_pub.published[-1]
    assert last.has_fatal_can_error is True
    assert any('link lost' in e.lower() for e in last.error)


def test_the_gate_reopens_when_the_link_recovers(bridge):
    """Link restored ⇒ back to one publish per telemetry frame.

    The latch clears on reconnect (``_health_check``), so the carve-out is
    bounded to the outage. Without this the first blip would pin the publisher at
    the full timer rate for the rest of the session and the fix would silently
    stop applying.
    """
    teensy, node = bridge
    _land(teensy, node)
    node._publish_robot_state()

    node._link_latch.first_heartbeat_seen = True
    node._link_latch.link_lost = True
    node._publish_robot_state()
    node._link_latch.link_lost = False

    before = len(node.robot_state_pub.published)
    node._publish_robot_state()
    node._publish_robot_state()
    assert len(node.robot_state_pub.published) == before, (
        'the duplicate suppression did not resume after the link recovered')
    assert node._robot_state_stale_skips == 2


def test_the_gate_sits_after_the_leg_setpoint_echo(bridge):
    """The echo still publishes on a skipped robot_state tick.

    ``_publish_leg_setpoint_echo`` deliberately rides this same 100 Hz timer and
    runs BEFORE the telemetry gate, because it has its own freshness gate and
    because the GUI's quiescence detector counts echo samples within a fixed
    window — fewer messages there changes a safety-teardown decision. Placing the
    skip above that call would have silenced the echo too, which is why the
    ordering is pinned rather than left to a comment.
    """
    teensy, node = bridge
    _land(teensy, node)
    node._publish_robot_state()

    calls = []
    node._publish_leg_setpoint_echo = lambda: calls.append(1)
    node._publish_robot_state()          # this one is SKIPPED
    node._publish_robot_state()          # so is this one
    assert node._robot_state_stale_skips == 2
    assert calls == [1, 1], (
        'the leg_setpoint_echo was skipped along with robot_state — the gate '
        'must sit AFTER the echo call')


def test_pre_telemetry_publishes_are_still_suppressed_and_not_counted(bridge):
    """No telemetry ever ⇒ no message, and no skip counted either.

    The pre-telemetry suppression is a DIFFERENT rule with a different reason
    (never publish a phantom all-zero snapshot), and it returns before the gate.
    Counting those as stale skips would inflate the statistic with an unrelated
    condition and make the /link_status row unreadable at startup.
    """
    teensy, node = bridge
    node._publish_robot_state()
    node._publish_robot_state()
    assert node.robot_state_pub.published == []
    assert node._robot_state_stale_skips == 0


def test_link_status_carries_the_skip_counter(bridge):
    """The skips are surfaced, so the suppression can never be silent."""
    teensy, node = bridge
    _land(teensy, node)
    node._publish_robot_state()
    node._publish_robot_state()
    node._publish_robot_state()

    node._publish_link_status()
    msg = node.link_status_pub.published[-1]
    kv = {e.key: e.value for e in msg.values}
    assert kv['robot_state_stale_skips'] == '2'
