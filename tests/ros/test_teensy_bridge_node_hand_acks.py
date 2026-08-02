"""HAND_TRAJ_CMD ack tally on teensy_bridge_node (ERR_TIMEOUT attribution).

Commit ``20d01e9`` (2026-07-24) demoted catch_coordinator's arm-ack failure from
``warning`` to ``debug``. That was correct for log noise — the failure was an
expected epidemic (30/51 = 59 % that session) while every attempt eventually
armed — and it also deleted the evidence: at the default log level the error code
never reaches the record, so every rosbag recorded since carries NO trace that the
hand arm ack ever failed. A bag is what survives a sitting; ``~/.ros/log`` at
default level is not.

The ``hand_traj_acks`` row on ``/link_status`` puts the numbers back, host-side,
against the CURRENT bridge — no wire change, no reflash. The load-bearing part is
the split:

* **fail_teensy** — the bridge answered with a non-OK status. The firmware ran
  ``hand_ops`` and one of its three CAN sends was refused.
* **fail_host** — nothing came back at all. The RPC request or its response was
  lost on the UDP link and the *host* synthesised the error.

Those two point at completely different halves of the system, and BOTH render as
``HAND_TRAJ_CMD: ERR_TIMEOUT`` in the log text (``RpcTimeout`` subclasses
``RpcError`` and carries ``ERR_TIMEOUT`` — teensy_link/rpc.py). Separating them by
hand from log lines is exactly what the 2026-08-01 recount had to do; these tests
pin the machine doing it instead.

``HAND_TRAJ_CMD`` is in ``NON_IDEMPOTENT_METHODS`` so ``retries`` is forced to 0:
one counted call is one wire attempt, with no re-dispatch inflating the tally.

Every failure here is driven through the REAL ``RpcClient`` against the
FakeTeensy — a Teensy-returned status via an auto-responder that answers non-OK,
a host timeout via a responder held closed until the call has already given up.
Nothing is monkeypatched, so a refactor that stops routing hand commands through
``_call_rpc`` fails these tests rather than silently keeping them green.

ROS 2 is mocked by tests/ros/conftest.py.
"""

from __future__ import annotations

import threading

from teensy_link import RpcMethod, RpcStatus
from teensy_link import rpc_args

from tests.ros._bridge_harness import (
    _build_paired_node,
    _link_kv,
    _teardown,
)


def _node():
    teensy, client, node = _build_paired_node(boot_state_read=False)
    return teensy, client, node


def _args() -> bytes:
    """A realistic HAND_TRAJ_CMD arg blob (kind-1 catch, the epidemic's caller)."""
    return rpc_args.encode_hand_traj_cmd(1, 1.25, 1_700_000_000_000)


def _respond(teensy, status):
    """Auto-respond to HAND_TRAJ_CMD with ``status`` (a real RPC round trip)."""
    teensy.on_rpc(int(RpcMethod.HAND_TRAJ_CMD),
                  lambda req_id, args: (int(status), b""))


def test_hand_traj_acks_row_renders_before_any_call():
    """The row exists from construction and reads all-zero — no sentinel.

    Unlike ``can3_errors`` (whose never-seen state means "this bridge may not
    even support the frame"), these counters are host-side and therefore always
    meaningful. ``calls=0`` is a different claim from "0 failures out of many",
    so rendering the honest zero is better than an 'unknown' string an operator
    would have to interpret. The row must also be UNCONDITIONAL: a row that
    appeared only after the first heartbeat would be missing from exactly the
    bags recorded during a link problem.
    """
    teensy, client, node = _node()
    try:
        row = _link_kv(node)
        assert 'hand_traj_acks' in row, "the row must publish unconditionally"
        assert row['hand_traj_acks'] == 'calls=0 ok=0 fail_teensy=0 fail_host=0'
    finally:
        _teardown(teensy, client, node)


def test_successful_hand_traj_increments_calls_only():
    """An accepted arm moves calls and ok, and neither failure class.

    If a success leaked into either fail counter the instrument would report a
    permanent epidemic and the next investigation would chase a phantom.
    """
    teensy, client, node = _node()
    try:
        _respond(teensy, RpcStatus.OK)
        ok, msg, _ = node.teensy_hand_traj_cmd(_args())
        assert ok, msg
        row = _link_kv(node)['hand_traj_acks']
        assert row == 'calls=1 ok=1 fail_teensy=0 fail_host=0'
    finally:
        _teardown(teensy, client, node)


def test_teensy_returned_error_counts_as_fail_teensy():
    """A non-OK status from the bridge lands in fail_teensy, not fail_host.

    This is the observed epidemic's shape: the recount found ZERO
    'ERR_TIMEOUT after N retries' lines on the hand path, i.e. every hand
    failure was Teensy-RETURNED — the firmware ran ``hand_ops`` and a
    ``can_jugglebot_send`` returned false. ERR_TIMEOUT is used as the status here
    deliberately: it is the exact code whose two origins the log text pools.
    """
    teensy, client, node = _node()
    try:
        _respond(teensy, RpcStatus.ERR_TIMEOUT)
        ok, msg, _ = node.teensy_hand_traj_cmd(_args())
        assert ok is False
        assert 'ERR_TIMEOUT' in msg
        row = _link_kv(node)['hand_traj_acks']
        assert row == 'calls=1 ok=0 fail_teensy=1 fail_host=0'
    finally:
        _teardown(teensy, client, node)


def test_host_synthesised_timeout_counts_as_fail_host():
    """No response at all lands in fail_host — the UDP-link failure class.

    Driven with a responder held shut by an Event until the call has already
    raised, so the timeout is deterministic rather than a race against a sleep:
    the FakeTeensy physically cannot answer before the test releases it, and the
    test releases it only after ``_call_rpc`` has returned.

    ``timeout`` is passed explicitly to keep the test fast; ``retries`` is not,
    because ``HAND_TRAJ_CMD`` is non-idempotent and ``RpcClient.call`` forces it
    to 0 regardless — pinning that here would only restate the library's rule.
    """
    teensy, client, node = _node()
    release = threading.Event()

    def _held(req_id, args):
        release.wait(2.0)
        return int(RpcStatus.OK), b""

    teensy.on_rpc(int(RpcMethod.HAND_TRAJ_CMD), _held)
    try:
        ok, msg, _ = node._call_rpc(RpcMethod.HAND_TRAJ_CMD, _args(),
                                    timeout=0.05)
        assert ok is False
        # Same rendered text as the Teensy-returned case above — which is the
        # whole reason the counters cannot be derived from the message.
        assert 'ERR_TIMEOUT' in msg
        row = _link_kv(node)['hand_traj_acks']
        assert row == 'calls=1 ok=0 fail_teensy=0 fail_host=1'
    finally:
        release.set()
        _teardown(teensy, client, node)


def test_other_methods_do_not_touch_the_hand_counters():
    """A failing non-hand RPC moves nothing — the filter is real.

    The counters live in ``_call_rpc``, which every outbound RPC passes through,
    so an unfiltered increment would silently fold homing, activate and the
    cold-start reads into the hand tally and make the ratio meaningless. NOP has
    no FakeTeensy handler, so the fake answers ERR_UNKNOWN_METHOD: a genuine
    ``RpcError`` on a different method.
    """
    teensy, client, node = _node()
    try:
        ok, _msg, _ = node._call_rpc(RpcMethod.NOP, b"", timeout=0.2)
        assert ok is False
        row = _link_kv(node)['hand_traj_acks']
        assert row == 'calls=0 ok=0 fail_teensy=0 fail_host=0'
    finally:
        _teardown(teensy, client, node)
