"""SetSlate action tests for teensy_bridge_node (clapboard downlink).

T-I9..T-I14 of ``plans/active/clapboard-can3-integration.md``, plus the CLAP_DIAG
wiring the plan defers here because this is the phase with a consumer.

Shape mirrors ``tests/ros/test_teensy_bridge_node_bb.py``'s bb/throw block: the
FakeTeensy answers the RPC on loopback, injects the terminal outcome back through
the real UDP path, and the action's callbacks are driven directly (ROS is mocked
by tests/ros/conftest.py — timers and executors do not run themselves).
"""

from __future__ import annotations

import threading
import time

import pytest

from teensy_link import MsgType, RpcMethod, RpcStatus
from teensy_link import protocol as p

from jugglebot import clapboard_slate as cs
from jugglebot.can import clapboard

from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.action import CancelResponse, GoalResponse
from jugglebot_interfaces.action import SetSlate

from tests.ros._bridge_harness import _build_paired_node, _teardown, _wait_until


# ── Harness ────────────────────────────────────────────────────

@pytest.fixture
def bridge():
    teensy, client, node = _build_paired_node()
    yield teensy, node
    _teardown(teensy, client, node)


def _cone_frame(can_id: int, data: bytes, t_bridge_us: int = 0) -> bytes:
    cf = p.ConeFrame(t_bridge_us=t_bridge_us, can_id=can_id,
                     dlc=len(data), data=tuple(data.ljust(8, b'\x00')))
    return cf.pack()


def _clap_heartbeat(state=1, flags=0x1F) -> bytes:
    """A minimal CLAP_HEARTBEAT payload — enough to make the clapboard 'present'."""
    return bytes([state, flags]) + b'\x00' * 6


def _attach_clapboard(teensy, node):
    """Make the node believe a clapboard is on the bus (10 Hz heartbeat arrived)."""
    teensy.send_to_jetson(int(MsgType.CONE_FRAME),
                          _cone_frame(clapboard.HEARTBEAT_ID, _clap_heartbeat()))
    assert _wait_until(lambda: node._clap_hb_received), 'heartbeat never arrived'


def _clap_send_requests(teensy):
    """Every CLAP_SEND request the node put on the wire.

    Filtered by RPC METHOD rather than counting raw RPC_REQUEST frames: an
    unrelated RPC from any other part of the node would otherwise make a
    "no retry was issued" assertion pass or fail for the wrong reason.
    """
    out = []
    for item in teensy.received(int(MsgType.RPC_REQUEST)):
        if len(item.payload) < p.RPC_REQUEST_SIZE:
            continue
        req = p.RpcRequest.unpack(item.payload[:p.RPC_REQUEST_SIZE])
        if int(req.method) == int(RpcMethod.CLAP_SEND):
            out.append(item)
    return out


def _ack_data(txn_id, outcome=0, state=1, render_ms=2500) -> bytes:
    return (bytes([int(txn_id), int(outcome), int(state)])
            + int(render_ms).to_bytes(2, 'little') + b'\x00\x00\x00')


def _goal(template_id=3, field_ids=None, field_values=None,
          force_full_refresh=False):
    g = SetSlate.Goal()
    g.template_id = template_id
    g.field_ids = [0, 1] if field_ids is None else field_ids
    g.field_values = (['SCENE 4', 'TAKE 12'] if field_values is None
                      else field_values)
    g.force_full_refresh = force_full_refresh
    return g


def _full_goal():
    """The worst case: 8 fields x 32 characters => 41 frames."""
    ids = list(range(8))
    return _goal(template_id=3, field_ids=ids,
                 field_values=[f'{i}' + 'X' * 31 for i in ids])


class _FakeGoalHandle:
    """Minimal goal_handle for driving _clap_slate_execute directly."""

    def __init__(self, goal):
        self.request = goal
        self.succeeded = False
        self.aborted = False
        self.feedback = []

    def succeed(self):
        self.succeeded = True

    def abort(self):
        self.aborted = True

    def publish_feedback(self, fb):
        self.feedback.append(fb.phase)


def _capture_clap_send(teensy, *, status=int(RpcStatus.OK),
                       ack_after=None, ack_delay_s=0.0):
    """Answer CLAP_SEND, record the args, and optionally inject a CLAP_ACK.

    ``ack_after`` is the outcome byte to ack with; the txn_id is read out of the
    commit frame the node actually built, so the correlation is exercised for
    real rather than assumed.
    """
    box = {}

    def handler(req_id, args):
        box['args'] = args
        arg = p.ArgClapSend.unpack(args)
        box['arg'] = arg
        box['frames'] = [
            (int(arg.can_id[i]), int(arg.len[i]),
             bytes(arg.data[i * 8:(i + 1) * 8]))
            for i in range(int(arg.count))
        ]
        box['txn_id'] = box['frames'][-1][2][3]
        if ack_after is not None:
            def _send_ack():
                if ack_delay_s:
                    time.sleep(ack_delay_s)
                teensy.send_to_jetson(
                    int(MsgType.CONE_FRAME),
                    _cone_frame(clapboard.ACK_ID,
                                _ack_data(box['txn_id'], outcome=ack_after)))
            threading.Thread(target=_send_ack, daemon=True).start()
        return (status, b'')

    teensy.on_rpc(int(RpcMethod.CLAP_SEND), handler)
    return box


# ── T-I9: happy path ───────────────────────────────────────────

def test_set_slate_happy_path(bridge):
    """One CLAP_SEND carrying 41 frames, commit LAST, succeeding on an OK ack."""
    teensy, node = bridge
    _attach_clapboard(teensy, node)
    box = _capture_clap_send(teensy, ack_after=0)

    goal = _full_goal()
    assert node._clap_slate_goal(goal) == GoalResponse.ACCEPT
    handle = _FakeGoalHandle(goal)
    result = node._clap_slate_execute(handle)

    assert len(box['frames']) == 41
    assert result.success is True
    assert result.outcome == 'OK'
    assert result.render_ms == 2500
    assert handle.succeeded is True
    assert handle.aborted is False
    assert handle.feedback == ['SENDING', 'AWAITING_ACK']
    # The single-flight slot is released in execute's finally.
    assert node._clap_slate_active is False
    assert node._clap_slate_txn is None


def test_set_slate_puts_the_commit_last_on_the_wire(bridge):
    """THE ordering contract: 40 CLAP_FIELD frames, then CLAP_COMMIT.

    Chunks may arrive in any order, but a commit that lands before its chunks is
    an INCOMPLETE rejection, and the bridge drains the RPC's list FIFO exactly as
    given — there is no ordering logic in the firmware to fall back on.
    """
    teensy, node = bridge
    _attach_clapboard(teensy, node)
    box = _capture_clap_send(teensy, ack_after=0)
    goal = _full_goal()
    assert node._clap_slate_goal(goal) == GoalResponse.ACCEPT
    node._clap_slate_execute(_FakeGoalHandle(goal))

    ids = [can_id for can_id, _len, _data in box['frames']]
    assert ids[:-1] == [clapboard.FIELD_ID] * 40
    assert ids[-1] == clapboard.COMMIT_ID
    assert clapboard.COMMIT_ID not in ids[:-1]


def test_every_dispatched_frame_is_dlc_8(bridge):
    """The clapboard silently drops anything that is not DLC 8, both directions."""
    teensy, node = bridge
    _attach_clapboard(teensy, node)
    box = _capture_clap_send(teensy, ack_after=0)
    goal = _goal(field_ids=[0], field_values=['A'])   # shortest possible text
    assert node._clap_slate_goal(goal) == GoalResponse.ACCEPT
    node._clap_slate_execute(_FakeGoalHandle(goal))
    assert [dlc for _id, dlc, _data in box['frames']] == [8] * 6


def test_set_slate_commit_carries_the_crc_of_the_padded_fields(bridge):
    """The commit's CRC is over the 32-byte NUL-padded present-field buffers."""
    teensy, node = bridge
    _attach_clapboard(teensy, node)
    box = _capture_clap_send(teensy, ack_after=0)
    goal = _goal(template_id=5, field_ids=[0, 2],
                 field_values=['SCENE 4', 'WIDE'], force_full_refresh=True)
    assert node._clap_slate_goal(goal) == GoalResponse.ACCEPT
    node._clap_slate_execute(_FakeGoalHandle(goal))

    _id, _dlc, commit = box['frames'][-1]
    assert commit[0] == 5                       # template_id
    assert commit[1] == 0b0000_0101             # fields 0 and 2
    assert commit[2] == cs.COMMIT_FLAG_FULL_REFRESH
    expected = cs.crc16_over_fields(
        cs.field_buffers([0, 2], ['SCENE 4', 'WIDE']))
    assert int.from_bytes(commit[4:6], 'little') == expected


def test_txn_ids_advance_between_transactions(bridge):
    """Each push gets a fresh txn_id, so a late ack cannot satisfy the next goal."""
    teensy, node = bridge
    _attach_clapboard(teensy, node)
    box = _capture_clap_send(teensy, ack_after=0)
    first = []
    for _ in range(2):
        goal = _goal()
        assert node._clap_slate_goal(goal) == GoalResponse.ACCEPT
        node._clap_slate_execute(_FakeGoalHandle(goal))
        first.append(box['txn_id'])
    assert first[0] != first[1]
    assert 0 not in first


# ── T-I10: ack timeout ─────────────────────────────────────────

def test_set_slate_times_out_without_retrying(bridge):
    """No CLAP_ACK => abort with TIMEOUT, and exactly ONE CLAP_SEND was issued.

    A retry racing a slow render answers BUSY and sends the operator hunting a
    fault that does not exist; recovery is a fresh goal with a fresh txn_id.
    """
    teensy, node = bridge
    _attach_clapboard(teensy, node)
    _capture_clap_send(teensy)          # acks the RPC, never acks the txn
    node._CLAP_SLATE_ACK_TIMEOUT_S = 0.2      # keep the test fast

    goal = _goal()
    assert node._clap_slate_goal(goal) == GoalResponse.ACCEPT
    handle = _FakeGoalHandle(goal)
    t0 = time.monotonic()
    result = node._clap_slate_execute(handle)
    elapsed = time.monotonic() - t0

    assert result.success is False
    assert result.outcome == cs.OUTCOME_TIMEOUT
    assert handle.aborted is True
    assert 0.15 <= elapsed < 2.0
    # Exactly one dispatch — no automatic retry anywhere in the path.
    assert len(_clap_send_requests(teensy)) == 1
    assert node._clap_slate_active is False


def test_late_ack_after_timeout_does_not_resurrect_the_goal(bridge):
    """An ack for a transaction that already timed out must not complete anything."""
    teensy, node = bridge
    _attach_clapboard(teensy, node)
    box = _capture_clap_send(teensy)
    node._CLAP_SLATE_ACK_TIMEOUT_S = 0.2
    goal = _goal()
    assert node._clap_slate_goal(goal) == GoalResponse.ACCEPT
    result = node._clap_slate_execute(_FakeGoalHandle(goal))
    assert result.outcome == cs.OUTCOME_TIMEOUT

    teensy.send_to_jetson(
        int(MsgType.CONE_FRAME),
        _cone_frame(clapboard.ACK_ID, _ack_data(box['txn_id'], outcome=0)))
    assert _wait_until(lambda: node._latest_clap_ack is not None)
    # Decoded and stashed as ever, but no transaction is in flight to satisfy.
    assert node._clap_slate_ack is None
    assert node._clap_slate_active is False


def test_ack_for_a_different_txn_is_ignored(bridge):
    """Correlation is by txn_id: a stray ack must not complete the goal early."""
    teensy, node = bridge
    _attach_clapboard(teensy, node)
    box = {}

    def handler(req_id, args):
        arg = p.ArgClapSend.unpack(args)
        commit = bytes(arg.data[(int(arg.count) - 1) * 8:int(arg.count) * 8])
        box['txn_id'] = commit[3]
        # Ack the WRONG transaction id.
        teensy.send_to_jetson(
            int(MsgType.CONE_FRAME),
            _cone_frame(clapboard.ACK_ID,
                        _ack_data((commit[3] + 1) & 0xFF, outcome=0)))
        return (int(RpcStatus.OK), b'')

    teensy.on_rpc(int(RpcMethod.CLAP_SEND), handler)
    node._CLAP_SLATE_ACK_TIMEOUT_S = 0.3
    goal = _goal()
    assert node._clap_slate_goal(goal) == GoalResponse.ACCEPT
    result = node._clap_slate_execute(_FakeGoalHandle(goal))
    assert result.outcome == cs.OUTCOME_TIMEOUT


# ── T-I11: disconnected ────────────────────────────────────────

def test_goal_rejected_when_no_clapboard_attached(bridge):
    """goal_callback rejects and NO RPC is issued — nothing reaches the wire."""
    teensy, node = bridge
    _capture_clap_send(teensy, ack_after=0)
    assert node._clap_slate_goal(_goal()) == GoalResponse.REJECT
    time.sleep(0.05)
    assert _clap_send_requests(teensy) == []
    assert node._clap_slate_active is False


def test_goal_rejected_once_the_heartbeat_goes_stale(bridge):
    """Presence is a freshness verdict, not a latch — an aged heartbeat is absent."""
    teensy, node = bridge
    _attach_clapboard(teensy, node)
    assert node._clap_slate_goal(_goal()) == GoalResponse.ACCEPT
    with node._clap_slate_lock:              # release the slot the accept claimed
        node._clap_slate_active = False
    node._clap_last_hb_mono = time.monotonic() - (node._clap_hb_timeout_s + 0.1)
    assert node._clap_slate_goal(_goal()) == GoalResponse.REJECT


def test_presence_predicate_matches_the_published_connected_flag(bridge):
    """_clapboard_present() and clapboard/heartbeat.connected are ONE verdict.

    They answer the same question — is a clapboard attached right now — and a
    second, differently-timed answer is exactly the drift this pins shut.
    """
    teensy, node = bridge
    node._publish_clapboard_heartbeat()
    assert node._clapboard_present() is False
    assert node.clapboard_heartbeat_pub.published[-1].connected is False

    _attach_clapboard(teensy, node)
    node._publish_clapboard_heartbeat()
    assert node._clapboard_present() is True
    assert node.clapboard_heartbeat_pub.published[-1].connected is True

    node._clap_last_hb_mono = time.monotonic() - (node._clap_hb_timeout_s + 0.1)
    node._publish_clapboard_heartbeat()
    assert node._clapboard_present() is False
    assert node.clapboard_heartbeat_pub.published[-1].connected is False


# ── T-I12: single flight ───────────────────────────────────────

def test_second_goal_rejected_while_one_is_in_flight(bridge):
    """One slate push at a time; the slot clears when execute finishes."""
    teensy, node = bridge
    _attach_clapboard(teensy, node)
    assert node._clap_slate_goal(_goal()) == GoalResponse.ACCEPT
    assert node._clap_slate_goal(_goal()) == GoalResponse.REJECT
    with node._clap_slate_lock:
        node._clap_slate_active = False
    assert node._clap_slate_goal(_goal()) == GoalResponse.ACCEPT


def test_a_malformed_goal_never_disturbs_a_push_in_flight(bridge):
    """An invalid goal is accepted-then-aborted, and must NOT clear the slot.

    Validation runs before the presence and single-flight checks so a typo is
    never reported as "no clapboard attached" — but that ordering means an
    invalid goal reaches goal_callback's ACCEPT without claiming the slot, and
    its execute must therefore not release one it never took.
    """
    teensy, node = bridge
    _attach_clapboard(teensy, node)
    assert node._clap_slate_goal(_goal()) == GoalResponse.ACCEPT   # slot claimed
    bad = _goal(field_ids=[9], field_values=['A'])
    assert node._clap_slate_goal(bad) == GoalResponse.ACCEPT       # slot untouched
    handle = _FakeGoalHandle(bad)
    result = node._clap_slate_execute(handle)
    assert result.outcome == cs.OUTCOME_INVALID_GOAL
    assert handle.aborted is True
    # The in-flight transaction still owns the slot.
    assert node._clap_slate_active is True


# ── Validation at the boundary (T-U7's action-level half) ──────

@pytest.mark.parametrize('field_ids,field_values,needle', [
    ([9], ['A'], 'field_ids[0]'),
    ([0], ['X' * 33], '33 bytes'),
    ([0, 0], ['A', 'B'], 'duplicate'),
    ([0, 1], ['A'], 'parallel'),
    ([0], ['TAKE\x0012'], 'NUL'),
])
def test_malformed_goal_aborts_with_the_reason_and_sends_nothing(
        bridge, field_ids, field_values, needle):
    """Refused before a single frame is built, with the offender named.

    A GoalResponse.REJECT carries no payload, so a malformed goal is accepted and
    ABORTED instead — that is the only way the caller learns which field it got
    wrong, rather than being told to go and check the wiring.
    """
    teensy, node = bridge
    _attach_clapboard(teensy, node)
    _capture_clap_send(teensy, ack_after=0)
    goal = _goal(field_ids=field_ids, field_values=field_values)
    assert node._clap_slate_goal(goal) == GoalResponse.ACCEPT
    handle = _FakeGoalHandle(goal)
    result = node._clap_slate_execute(handle)
    assert result.success is False
    assert result.outcome == cs.OUTCOME_INVALID_GOAL
    assert needle in result.message
    assert handle.aborted is True
    time.sleep(0.05)
    assert _clap_send_requests(teensy) == []


def test_template_id_out_of_range_aborts(bridge):
    teensy, node = bridge
    _attach_clapboard(teensy, node)
    _capture_clap_send(teensy, ack_after=0)
    goal = _goal(template_id=16)
    assert node._clap_slate_goal(goal) == GoalResponse.ACCEPT
    result = node._clap_slate_execute(_FakeGoalHandle(goal))
    assert result.outcome == cs.OUTCOME_INVALID_GOAL
    assert 'template_id' in result.message


# ── T-I13: dispatch refused by the bridge ──────────────────────

@pytest.mark.parametrize('status,label', [
    (int(RpcStatus.ERR_BUS_DOWN), 'ERR_BUS_DOWN'),
    (int(RpcStatus.ERR_REJECTED), 'ERR_REJECTED'),
    (int(RpcStatus.ERR_UNKNOWN_METHOD), 'ERR_UNKNOWN_METHOD'),
])
def test_dispatch_failure_aborts_immediately(bridge, status, label):
    """A refused enqueue means NOTHING reached the bus, so no ack will ever come.

    ERR_BUS_DOWN is a shut TX presence gate; ERR_REJECTED is a ring that cannot
    hold the whole burst (enqueue is all-or-nothing); ERR_UNKNOWN_METHOD is a
    pre-FW-15 board. All three abort now rather than burning the 8 s budget.
    """
    teensy, node = bridge
    _attach_clapboard(teensy, node)
    _capture_clap_send(teensy, status=status)
    node._CLAP_SLATE_ACK_TIMEOUT_S = 5.0     # would dominate if we waited

    goal = _goal()
    assert node._clap_slate_goal(goal) == GoalResponse.ACCEPT
    handle = _FakeGoalHandle(goal)
    t0 = time.monotonic()
    result = node._clap_slate_execute(handle)
    assert time.monotonic() - t0 < 2.0

    assert result.success is False
    assert result.outcome == cs.OUTCOME_DISPATCH_FAILED
    assert label in result.message
    assert handle.aborted is True
    assert node._clap_slate_active is False


# ── Non-OK panel outcomes ──────────────────────────────────────

@pytest.mark.parametrize('outcome,name', [
    (0x01, 'REJECTED'),
    (0x02, 'CRC_MISMATCH'),
    (0x03, 'INCOMPLETE'),
    (0x04, 'BUSY'),
    (0x05, 'NO_TEMPLATE'),
    (0x06, 'BAD_FIELD_ID'),
])
def test_non_ok_ack_aborts_and_names_the_outcome(bridge, outcome, name):
    teensy, node = bridge
    _attach_clapboard(teensy, node)
    _capture_clap_send(teensy, ack_after=outcome)
    goal = _goal()
    assert node._clap_slate_goal(goal) == GoalResponse.ACCEPT
    handle = _FakeGoalHandle(goal)
    result = node._clap_slate_execute(handle)
    assert result.success is False
    assert result.outcome == name
    assert handle.aborted is True


def test_unknown_ack_outcome_is_reported_not_raised(bridge):
    """An outcome byte this host predates renders as UNKNOWN(n), never a crash."""
    teensy, node = bridge
    _attach_clapboard(teensy, node)
    _capture_clap_send(teensy, ack_after=0x77)
    goal = _goal()
    assert node._clap_slate_goal(goal) == GoalResponse.ACCEPT
    result = node._clap_slate_execute(_FakeGoalHandle(goal))
    assert result.success is False
    assert result.outcome == 'UNKNOWN(119)'


# ── Cancel ─────────────────────────────────────────────────────

def test_cancel_is_rejected(bridge):
    """An e-paper refresh in flight has no abort path on either transport."""
    _teensy, node = bridge
    assert node._clap_slate_cancel(None) == CancelResponse.REJECT


# ── CLAP_DIAG wiring ───────────────────────────────────────────

def _clap_diag(queued=0, sent=0, gated=0, dropped=0, ring_hwm=0) -> bytes:
    return p.ClapDiag(queued=queued, sent=sent, gated=gated, dropped=dropped,
                      ring_hwm=ring_hwm, pad=(0, 0, 0)).pack()


def test_clap_diag_is_decoded_and_published(bridge):
    """CLAP_DIAG reaches /clap_diag with every counter rendered raw."""
    teensy, node = bridge
    teensy.send_to_jetson(int(MsgType.CLAP_DIAG),
                          _clap_diag(queued=41, sent=41, ring_hwm=12))
    assert _wait_until(lambda: len(node._clap_diag_queue) == 1)
    node._publish_clap_diag()
    msg = node.clap_diag_pub.published[-1]
    kv = {v.key: v.value for v in msg.values}
    assert kv == {'queued': '41', 'sent': '41', 'gated': '0', 'dropped': '0',
                  'ring_hwm': '12', 'lost_since_previous': '0'}
    assert msg.level == DiagnosticStatus.OK
    # Drained — a second tick publishes nothing new.
    node._publish_clap_diag()
    assert len(node.clap_diag_pub.published) == 1


def test_clap_diag_warn_is_not_sticky_across_a_cumulative_counter(bridge):
    """WARN means "losing frames NOW", not "lost one at some point since boot".

    The counters are cumulative, so a level keyed on the total would latch WARN
    for the rest of the session after one legitimate loss — a slate pushed before
    the clapboard's first heartbeat opened the TX gate, say — and teach the
    operator to ignore the row. The first sample is a BASELINE at OK, because
    losses tallied before this node started are not this session's.
    """
    teensy, node = bridge
    # 1) First sample already carries losses: BASELINE, level OK.
    teensy.send_to_jetson(int(MsgType.CLAP_DIAG),
                          _clap_diag(queued=41, sent=35, gated=6))
    assert _wait_until(lambda: len(node._clap_diag_queue) == 1)
    node._publish_clap_diag()
    first = node.clap_diag_pub.published[-1]
    assert first.level == DiagnosticStatus.OK
    assert 'baseline' in first.message
    # ...and the raw counters still carry the pre-launch losses, so nothing hid.
    assert {v.key: v.value for v in first.values}['gated'] == '6'

    # 2) A window with no NEW loss stays OK, even though the total is non-zero.
    teensy.send_to_jetson(int(MsgType.CLAP_DIAG),
                          _clap_diag(queued=82, sent=76, gated=6))
    assert _wait_until(lambda: len(node._clap_diag_queue) == 1)
    node._publish_clap_diag()
    steady = node.clap_diag_pub.published[-1]
    assert steady.level == DiagnosticStatus.OK
    assert {v.key: v.value for v in steady.values}['lost_since_previous'] == '0'

    # 3) A window that loses MORE warns.
    teensy.send_to_jetson(int(MsgType.CLAP_DIAG),
                          _clap_diag(queued=123, sent=113, gated=6, dropped=4))
    assert _wait_until(lambda: len(node._clap_diag_queue) == 1)
    node._publish_clap_diag()
    losing = node.clap_diag_pub.published[-1]
    assert losing.level == DiagnosticStatus.WARN
    assert {v.key: v.value for v in losing.values}['lost_since_previous'] == '4'


def test_truncated_clap_diag_does_not_kill_the_rx_thread(bridge):
    """A short payload raises struct.error, which must not escape the callback."""
    teensy, node = bridge
    teensy.send_to_jetson(int(MsgType.CLAP_DIAG), b'\x01\x02\x03')
    time.sleep(0.05)
    assert node._clap_diag_queue == []
    teensy.send_to_jetson(int(MsgType.CLAP_DIAG), _clap_diag(queued=1))
    assert _wait_until(lambda: len(node._clap_diag_queue) == 1)


def test_failure_message_carries_the_bridge_tx_census_delta(bridge):
    """A CRC_MISMATCH cannot say WHY frames went missing; the census can.

    Non-zero gated/dropped across the transaction means the bridge never put them
    on the wire; a flat delta points at the panel's reassembly instead. Nothing
    else separates the two.
    """
    teensy, node = bridge
    _attach_clapboard(teensy, node)
    teensy.send_to_jetson(int(MsgType.CLAP_DIAG),
                          _clap_diag(queued=10, sent=10))
    assert _wait_until(lambda: node._latest_clap_diag is not None)

    def handler(req_id, args):
        arg = p.ArgClapSend.unpack(args)
        commit = bytes(arg.data[(int(arg.count) - 1) * 8:int(arg.count) * 8])
        # The bridge gated 4 of the burst, then the panel answered CRC_MISMATCH.
        teensy.send_to_jetson(int(MsgType.CLAP_DIAG),
                              _clap_diag(queued=51, sent=47, gated=4, ring_hwm=9))
        time.sleep(0.05)
        teensy.send_to_jetson(
            int(MsgType.CONE_FRAME),
            _cone_frame(clapboard.ACK_ID, _ack_data(commit[3], outcome=0x02)))
        return (int(RpcStatus.OK), b'')

    teensy.on_rpc(int(RpcMethod.CLAP_SEND), handler)
    goal = _goal()
    assert node._clap_slate_goal(goal) == GoalResponse.ACCEPT
    result = node._clap_slate_execute(_FakeGoalHandle(goal))

    assert result.outcome == 'CRC_MISMATCH'
    assert 'gated +4' in result.message
    assert 'queued +41' in result.message
    assert 'ring_hwm 9' in result.message


def test_census_with_no_pre_dispatch_sample_is_reported_absolute(bridge):
    """With nothing to difference against, the counters are ABSOLUTE and say so.

    The first CLAP_DIAG of a session can land after the dispatch. Rendering it as
    a delta from zero would blame this one slate push for every frame the bridge
    has lost since it booted.
    """
    teensy, node = bridge
    _attach_clapboard(teensy, node)

    def handler(req_id, args):
        arg = p.ArgClapSend.unpack(args)
        commit = bytes(arg.data[(int(arg.count) - 1) * 8:int(arg.count) * 8])
        teensy.send_to_jetson(int(MsgType.CLAP_DIAG),
                              _clap_diag(queued=900, sent=880, dropped=20))
        time.sleep(0.05)
        teensy.send_to_jetson(
            int(MsgType.CONE_FRAME),
            _cone_frame(clapboard.ACK_ID, _ack_data(commit[3], outcome=0x03)))
        return (int(RpcStatus.OK), b'')

    teensy.on_rpc(int(RpcMethod.CLAP_SEND), handler)
    goal = _goal()
    assert node._clap_slate_goal(goal) == GoalResponse.ACCEPT
    result = node._clap_slate_execute(_FakeGoalHandle(goal))

    assert result.outcome == 'INCOMPLETE'
    assert 'ABSOLUTE since bridge boot' in result.message
    assert 'dropped 20' in result.message          # absolute, not '+20'
    assert 'dropped +20' not in result.message


def test_failure_message_says_so_when_no_census_exists(bridge):
    """A pre-FW-15 bridge emits no 0x93; the message must say that, not imply zero."""
    teensy, node = bridge
    _attach_clapboard(teensy, node)
    _capture_clap_send(teensy)
    node._CLAP_SLATE_ACK_TIMEOUT_S = 0.2
    goal = _goal()
    assert node._clap_slate_goal(goal) == GoalResponse.ACCEPT
    result = node._clap_slate_execute(_FakeGoalHandle(goal))
    assert result.outcome == cs.OUTCOME_TIMEOUT
    assert 'census unavailable' in result.message


# ── T-I14: the ReentrantCallbackGroup proof ────────────────────

def test_action_has_its_own_reentrant_callback_group(bridge):
    """Structural half: a DEDICATED ReentrantCallbackGroup, not the node default.

    In the node-default MutuallyExclusiveCallbackGroup a multi-second blocking
    execute_callback serializes with — and stalls — the 100 Hz telemetry timers,
    which run in that group on other MultiThreadedExecutor threads.
    """
    from rclpy.callback_groups import ReentrantCallbackGroup
    _teensy, node = bridge
    server = node._action_servers['clapboard/set_slate']
    assert server._callback_group is node._clap_slate_cbgroup
    assert isinstance(node._clap_slate_cbgroup, ReentrantCallbackGroup)
    # Its own group, not shared with the other blocking surfaces.
    assert node._clap_slate_cbgroup is not node._bb_throw_cbgroup
    assert node._clap_slate_cbgroup is not node._coldstart_cbgroup


def test_telemetry_timers_keep_publishing_during_a_render(bridge):
    """Behavioural half: the timers keep running while a slate push blocks.

    The callback group alone is not the whole guarantee — an execute_callback
    that held ``self._lock`` across the wait would stall the publishers no matter
    which group it ran in. So this drives the real blocking path on one thread
    and the real publish timers on another, and asserts they interleave.

    The render is SCALED: a real e-paper full refresh is 1.5-3.5 s, here it is
    0.4 s, because the property under test (the wait holds nothing the timers
    need) is independent of the wait's duration and a 3 s test is 3 s of gate.
    """
    teensy, node = bridge
    _attach_clapboard(teensy, node)
    _capture_clap_send(teensy, ack_after=0, ack_delay_s=0.5)
    node._CLAP_SLATE_ACK_TIMEOUT_S = 5.0

    # Give the publishers something real to publish.
    teensy.send_to_jetson(int(MsgType.HEARTBEAT_T2J), p.HeartbeatT2J(
        t_teensy_us=1, link_state=int(p.LinkState.UP),
        bus1_health=int(p.BusHealth.OK), bus2_health=int(p.BusHealth.OK),
        fault_state=int(p.FaultState.NONE), flags=0, uptime_ms=1).pack())
    assert _wait_until(lambda: node._latest_heartbeat is not None)

    goal = _goal()
    assert node._clap_slate_goal(goal) == GoalResponse.ACCEPT
    handle = _FakeGoalHandle(goal)
    box = {}

    def _run():
        box['result'] = node._clap_slate_execute(handle)

    worker = threading.Thread(target=_run, daemon=True)
    worker.start()
    # Drive the 100 Hz + 10 Hz timers on THIS thread while the action blocks.
    ticks = 0
    deadline = time.monotonic() + 1.5
    while worker.is_alive() and time.monotonic() < deadline:
        node._publish_link_status()
        node._publish_clapboard_heartbeat()
        ticks += 1
        time.sleep(0.01)
    worker.join(timeout=3.0)

    assert not worker.is_alive(), 'the action never finished'
    assert box['result'].success is True
    # The timers ran throughout, and every tick actually published.
    assert ticks >= 10
    assert len(node.link_status_pub.published) == ticks
    assert len(node.clapboard_heartbeat_pub.published) == ticks
