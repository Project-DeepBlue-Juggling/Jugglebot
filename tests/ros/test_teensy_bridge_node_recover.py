"""Guard-recovery tests for teensy_bridge_node — the /recover one-call flow (FIX 3).

/recover reseeds trajectory_node's hold at the measured encoder, VERIFIES the
streamed u0 has collapsed to within 0.25 rev of every encoder, then — and only
then — fires CLEAR_ERRORS. This is the recovery the 2026-07-10 runaway needed: a
bare /clear_errors re-latched MAX_DEVIATION within one fault tick because the
streamed u0 was still ~0.5 rev past the frozen encoder.

The cross-process reseed client (bridge → trajectory_node) is faked here; the
CLEAR_ERRORS RPC runs through the real RpcClient + FakeTeensy so the "did the clear
actually fire?" assertions are real wire observations.
"""

from __future__ import annotations

import threading
import time
import types

from controller.teensy_link import RpcMethod, RpcStatus

from std_srvs.srv import Trigger

from tests.ros.test_teensy_bridge_node_read import _build_paired_node
from tests.ros.test_teensy_bridge_node_rpc import _capture
from tests.ros.test_teensy_bridge_node_setpoint import _bring_link_and_telem_up


def _node():
    return _build_paired_node()


def _teardown(teensy, client, node):
    node.on_shutdown()
    client.stop()
    teensy.stop()


class _DoneFuture:
    """A future that is already resolved (the reseed reply, arrived)."""

    def __init__(self, result):
        self._result = result

    def done(self):
        return True

    def result(self):
        return self._result


class _FakeReseedClient:
    """Stand-in for the bridge→trajectory_node reseed_from_measured client."""

    def __init__(self, *, ready=True, success=True, message='reseeded hold at measured'):
        self._ready = ready
        self._resp = types.SimpleNamespace(success=success, message=message)
        self.calls = 0

    def service_is_ready(self):
        return self._ready

    def call_async(self, request):
        self.calls += 1
        return _DoneFuture(self._resp)


def test_recover_happy_path_reseeds_verifies_then_clears():
    teensy, client, node = _node()
    try:
        box = _capture(teensy, RpcMethod.CLEAR_ERRORS)
        _bring_link_and_telem_up(teensy, node, leg_pos=0.1)   # encoder = 0.1 rev/leg
        # The profiled descent already converged → the streamed u0 sits on the encoder.
        node._sp_pump._prev_pos = [0.1] * 6
        node._reseed_client = _FakeReseedClient(success=True)

        res = node._svc_recover(Trigger.Request(), Trigger.Response())
        assert res.success is True, res.message
        assert 'CLEAR_ERRORS' in res.message
        assert node._reseed_client.calls == 1                 # descent was requested
        assert 'args' in box                                  # CLEAR_ERRORS actually fired
    finally:
        _teardown(teensy, client, node)


def test_recover_waits_for_descent_to_converge_then_clears():
    """/recover must WAIT for the profiled descent to walk u0 down onto the encoder
    (it collapses over ~0.5-3 s, not in one frame), not sample once. A background
    thread walks _prev_pos down mid-poll; /recover blocks until it lands, then clears."""
    teensy, client, node = _node()
    try:
        box = _capture(teensy, RpcMethod.CLEAR_ERRORS)
        _bring_link_and_telem_up(teensy, node, leg_pos=0.1)
        node._recover_verify_timeout_s = 2.0
        node._sp_pump._prev_pos = [1.0] * 6                   # 0.9 rev out at t=0
        node._reseed_client = _FakeReseedClient(success=True)

        def _descend():
            time.sleep(0.3)                                    # descent still in flight
            node._sp_pump._prev_pos = [0.1] * 6                # then converges onto encoder

        th = threading.Thread(target=_descend)
        th.start()
        res = node._svc_recover(Trigger.Request(), Trigger.Response())
        th.join()
        assert res.success is True, res.message
        assert 'CLEAR_ERRORS' in res.message
        assert 'args' in box                                  # cleared only AFTER convergence
    finally:
        _teardown(teensy, client, node)


def test_recover_times_out_when_descent_never_converges():
    """The descent never walks u0 in (stays ~0.9 rev out) → /recover refuses after the
    bounded wait and does NOT clear (a clear now would re-latch within one fault
    tick). A short verify timeout keeps the test fast."""
    teensy, client, node = _node()
    try:
        box = _capture(teensy, RpcMethod.CLEAR_ERRORS)
        _bring_link_and_telem_up(teensy, node, leg_pos=0.1)
        node._recover_verify_timeout_s = 0.3                  # keep the test fast
        node._sp_pump._prev_pos = [1.0] * 6                   # 0.9 rev from encoder, forever
        node._reseed_client = _FakeReseedClient(success=True)

        res = node._svc_recover(Trigger.Request(), Trigger.Response())
        assert res.success is False
        assert 'descent did not converge' in res.message
        assert node._mpc_active is False                      # never disarmed either
        assert 'args' not in box                              # CLEAR_ERRORS must NOT fire
    finally:
        _teardown(teensy, client, node)


def test_recover_reports_clear_errors_failure():
    """Reseed + converge succeed, but the CLEAR_ERRORS RPC itself fails → report it
    precisely (not a silent success)."""
    teensy, client, node = _node()
    try:
        # CLEAR_ERRORS responds non-OK → teensy_clear_errors returns ok=False.
        _capture(teensy, RpcMethod.CLEAR_ERRORS, status=int(RpcStatus.ERR_BUS_DOWN))
        _bring_link_and_telem_up(teensy, node, leg_pos=0.1)
        node._sp_pump._prev_pos = [0.1] * 6
        node._reseed_client = _FakeReseedClient(success=True)

        res = node._svc_recover(Trigger.Request(), Trigger.Response())
        assert res.success is False
        assert 'CLEAR_ERRORS failed' in res.message
    finally:
        _teardown(teensy, client, node)


def test_recover_refuses_when_reseed_unavailable():
    """No trajectory_node reseed service (e.g. it is not running) → refuse fast,
    without clearing (never clear onto an unknown command)."""
    teensy, client, node = _node()
    try:
        box = _capture(teensy, RpcMethod.CLEAR_ERRORS)
        _bring_link_and_telem_up(teensy, node, leg_pos=0.1)
        node._reseed_client = _FakeReseedClient(ready=False)

        res = node._svc_recover(Trigger.Request(), Trigger.Response())
        assert res.success is False
        assert 'unavailable' in res.message.lower()
        assert 'args' not in box
    finally:
        _teardown(teensy, client, node)


def test_recover_refuses_when_reseed_refused():
    """trajectory_node accepts the call but reseed fails (stale telemetry / not
    streaming) → refuse, do not clear."""
    teensy, client, node = _node()
    try:
        box = _capture(teensy, RpcMethod.CLEAR_ERRORS)
        _bring_link_and_telem_up(teensy, node, leg_pos=0.1)
        node._sp_pump._prev_pos = [0.1] * 6
        node._reseed_client = _FakeReseedClient(
            success=False, message='not streaming or telemetry stale')

        res = node._svc_recover(Trigger.Request(), Trigger.Response())
        assert res.success is False
        assert 'reseed refused' in res.message
        assert 'args' not in box
    finally:
        _teardown(teensy, client, node)
