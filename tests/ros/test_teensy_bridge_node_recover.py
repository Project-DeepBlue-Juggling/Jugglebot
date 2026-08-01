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

from controller.teensy_link import RpcMethod, RpcStatus, rpc_args

from std_srvs.srv import Trigger

from tests.ros._bridge_harness import _build_paired_node, _teardown
from tests.ros.test_teensy_bridge_node_rpc import _capture
from tests.ros.test_teensy_bridge_node_setpoint import _bring_link_and_telem_up


def _node():
    return _build_paired_node()


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


def test_recover_clears_directly_when_reseed_unavailable():
    """No trajectory_node reseed service (it is down) → converge-first is IMPOSSIBLE
    (nothing to install the descent, no live stream to jolt at re-enable), so /recover
    CLEARS DIRECTLY as the escape hatch (F5) rather than stranding the operator with a
    latched guard and no reachable converge path."""
    teensy, client, node = _node()
    try:
        box = _capture(teensy, RpcMethod.CLEAR_ERRORS)
        _bring_link_and_telem_up(teensy, node, leg_pos=0.1)
        node._reseed_client = _FakeReseedClient(ready=False)

        res = node._svc_recover(Trigger.Request(), Trigger.Response())
        assert res.success is True, res.message
        assert node._reseed_client.calls == 0        # never attempted converge-first
        assert 'directly' in res.message.lower()     # worded as the escape hatch
        assert 'args' in box                         # CLEAR_ERRORS DID fire directly
    finally:
        _teardown(teensy, client, node)


def test_recover_refusal_states_manual_recovery():
    """Every armed-recovery REFUSAL (converge-first cannot complete) must tell the
    operator the always-available disarm→clear escape (F5). Checked on the reseed-refused
    branch (a live trajectory_node that declines — distinct from it being down)."""
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
        assert 'set_setpoint_output' in res.message   # states the manual disarm→clear escape
        assert 'args' not in box                       # did NOT clear
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


# ── Tightened convergence tolerance (2026-07-11 clear-errors jolt) ────────────
# The gate dropped 0.25 → 0.03 rev: 0.25 was 2.5× the firmware 0.10 lead clamp, so a
# "converged" clear could still saturate the clamp at re-enable and inject a
# pos_gain × lead ≈ 4 rev/s kick. 0.03 is well under the clamp.

def test_recover_convergence_gate_is_tightened_to_003():
    from jugglebot.teensy_bridge_node import (_RECOVER_U0_TOL_REV,
                                              _ARM_U0_TOL_REV)
    assert _RECOVER_U0_TOL_REV == 0.03            # recovery gate tightened
    assert _ARM_U0_TOL_REV == 0.25                # arming stays generous (slew covers it)


def test_recover_refuses_residual_that_old_025_tol_would_have_passed():
    """A 0.05 rev residual (u0 0.05 rev off the encoder) — inside the OLD 0.25 gate,
    OUTSIDE the new 0.03 gate — must now REFUSE (a clear there would saturate the lead
    clamp and jolt). A 0.02 rev residual (inside 0.03) still converges + clears."""
    teensy, client, node = _node()
    try:
        box = _capture(teensy, RpcMethod.CLEAR_ERRORS)
        _bring_link_and_telem_up(teensy, node, leg_pos=0.1)   # encoder = 0.1 rev/leg
        node._recover_verify_timeout_s = 0.1                  # keep the retries fast
        node._reseed_client = _FakeReseedClient(success=True)

        node._sp_pump._prev_pos = [0.15] * 6                  # 0.05 rev off — old tol passed
        res = node._svc_recover(Trigger.Request(), Trigger.Response())
        assert res.success is False
        assert 'descent did not converge' in res.message
        assert 'args' not in box                              # did NOT clear onto 0.05 rev

        node._sp_pump._prev_pos = [0.12] * 6                  # 0.02 rev off — inside 0.03
        res = node._svc_recover(Trigger.Request(), Trigger.Response())
        assert res.success is True, res.message
        assert 'args' in box                                  # cleared within the tight gate
    finally:
        _teardown(teensy, client, node)


# ── Bounded re-descend on drift (2026-07-11 clear-errors jolt) ────────────────

class _DriftReseedClient:
    """Models the leg still settling: the first reseed's descent converges u0 onto a
    STALE encoder (still off the live one); the second reseed re-samples and lands u0
    on the live encoder — so /recover must re-descend, not refuse."""

    def __init__(self, node, live):
        self._node = node
        self._live = list(live)
        self.calls = 0

    def service_is_ready(self):
        return True

    def call_async(self, request):
        self.calls += 1
        if self.calls >= 2:                                   # the re-installed descent lands
            self._node._sp_pump._prev_pos = list(self._live)
        return _DoneFuture(types.SimpleNamespace(success=True, message='reseeded'))


def test_recover_re_descends_when_u0_plateaus_off_the_drifted_encoder():
    """The descent converges u0 onto the install-time encoder, but the leg drifted on
    (the −0.102 rev plateau), so the first verify plateaus above tol. /recover must
    re-install (re-sample the now-current encoder) and only then clear — not refuse."""
    teensy, client, node = _node()
    try:
        box = _capture(teensy, RpcMethod.CLEAR_ERRORS)
        _bring_link_and_telem_up(teensy, node, leg_pos=0.1)   # LIVE encoder = 0.1 rev/leg
        node._recover_verify_timeout_s = 0.1                  # attempt-1 fails fast
        node._sp_pump._prev_pos = [0.2] * 6                   # u0 0.1 rev off the live encoder
        node._reseed_client = _DriftReseedClient(node, live=[0.1] * 6)

        res = node._svc_recover(Trigger.Request(), Trigger.Response())
        assert res.success is True, res.message
        assert node._reseed_client.calls == 2                 # re-descended exactly once
        assert 'args' in box                                  # cleared only after re-convergence
    finally:
        _teardown(teensy, client, node)


# ── Bare /clear_errors reroute: converge-first when armed, direct when not ────
# Operator decision (2026-07-11): the armed bare /clear_errors is REROUTED through the
# converge-first sequence (no raw escape hatch); the unarmed clear passes straight
# through (the benign boot-time MPC_STALE latch — output not evaluated, no jolt).

def test_armed_bare_clear_errors_routes_through_converge_first():
    teensy, client, node = _node()
    try:
        box = _capture(teensy, RpcMethod.CLEAR_ERRORS)
        _bring_link_and_telem_up(teensy, node, leg_pos=0.1)
        node._mpc_active = True                               # ARMED
        node._sp_pump._prev_pos = [0.1] * 6                   # u0 already on the encoder
        node._reseed_client = _FakeReseedClient(success=True)

        res = node._svc_clear_errors(Trigger.Request(), Trigger.Response())
        assert res.success is True, res.message
        assert node._reseed_client.calls == 1                 # went through reseed (not a bare clear)
        assert 'CLEAR_ERRORS' in res.message                  # the /recover-shaped reply
        assert 'args' in box                                  # clear fired only after converge
    finally:
        _teardown(teensy, client, node)


def test_armed_bare_clear_errors_on_diverged_command_disarms_then_clears():
    """ARMING_CONTRACT: while armed, a bare /clear_errors onto a diverged u0 first
    routes through converge-first; when that CANNOT converge, it falls back to
    DISARM + direct clear instead of the pre-contract hard refusal (the dead end
    behind the 2026-07-15 'reseed refused' wedge). The 2026-07-11 no-jolt root
    cause is preserved: the clear lands only AFTER the disarm, when the firmware's
    guard terms are inert and the output gate is off — a jolt is impossible. The
    wire is left DISARMED; re-arming must pass the full A1 pre-check."""
    teensy, client, node = _node()
    try:
        box = _capture(teensy, RpcMethod.CLEAR_ERRORS)
        _bring_link_and_telem_up(teensy, node, leg_pos=0.1)
        node._mpc_active = True                               # ARMED
        node._recover_verify_timeout_s = 0.1
        node._sp_pump._prev_pos = [1.0] * 6                   # 0.9 rev diverged, forever
        node._reseed_client = _FakeReseedClient(success=True)

        res = node._svc_clear_errors(Trigger.Request(), Trigger.Response())
        assert res.success is True, res.message
        assert node._mpc_active is False                      # disarmed FIRST
        assert 'disarmed and cleared directly' in res.message
        assert 'DISARMED' in res.message                      # loud about the state
        assert 'args' in box                                  # clear DID land (disarmed)
    finally:
        _teardown(teensy, client, node)


def test_unarmed_bare_clear_errors_passes_straight_through():
    """When NOT armed (mpc_active=0 — the benign boot-time MPC_STALE latch), the bare
    /clear_errors clears DIRECTLY: output is not being evaluated so no jolt is possible,
    and routing through reseed would needlessly refuse (nothing is streaming)."""
    teensy, client, node = _node()
    try:
        box = _capture(teensy, RpcMethod.CLEAR_ERRORS)
        # A reseed client that would FAIL the test if touched (it must NOT be called).
        node._reseed_client = _FakeReseedClient(success=True)
        assert node._mpc_active is False                      # UNARMED (startup default)

        res = node._svc_clear_errors(Trigger.Request(), Trigger.Response())
        assert res.success is True
        assert node._reseed_client.calls == 0                 # bypassed converge-first entirely
        assert box['args'] == bytes([rpc_args.AXIS_ALL])      # direct CLEAR_ERRORS, AXIS_ALL
    finally:
        _teardown(teensy, client, node)


def test_armed_bare_clear_errors_clears_directly_when_trajectory_node_down():
    """While armed, if trajectory_node is DOWN the reroute cannot converge-first (there is
    no node to install the descent), so the armed /clear_errors clears DIRECTLY (escape
    hatch, F5) rather than refusing and stranding the operator with a latched guard."""
    teensy, client, node = _node()
    try:
        box = _capture(teensy, RpcMethod.CLEAR_ERRORS)
        _bring_link_and_telem_up(teensy, node, leg_pos=0.1)
        node._mpc_active = True                               # ARMED
        node._reseed_client = _FakeReseedClient(ready=False)  # trajectory_node down

        res = node._svc_clear_errors(Trigger.Request(), Trigger.Response())
        assert res.success is True, res.message
        assert node._reseed_client.calls == 0                 # converge-first not attempted
        assert 'args' in box                                  # cleared directly
    finally:
        _teardown(teensy, client, node)


# ── clear_errors callback-group pinning (F1) ─────────────────────────────────
# The armed bare /clear_errors reroutes INLINE through _svc_recover (up to ~18 s of
# blocking). Its service MUST live in the /recover ReentrantCallbackGroup — NOT the
# node-default MutuallyExclusiveCallbackGroup, where the multi-second block would
# serialize with (and starve) the 100 Hz telemetry timers AND the set_setpoint_output
# disarm service, making the disarm escape unreachable for the whole block.

def test_clear_errors_shares_recover_callback_group():
    teensy, client, node = _node()
    try:
        svcs = node._services            # MockNode registry: name → recorded service
        assert 'clear_errors' in svcs, sorted(svcs)
        assert 'recover' in svcs
        # clear_errors shares the SAME reentrant group as /recover.
        assert svcs['clear_errors'].callback_group is node._recover_cbgroup
        assert svcs['recover'].callback_group is node._recover_cbgroup
        # And it is NOT the node-default group: reboot_odrives (a quick RPC) stays in the
        # default MutuallyExclusiveCallbackGroup, recorded as callback_group=None here.
        assert svcs['clear_errors'].callback_group is not None
        assert svcs['reboot_odrives'].callback_group is None
    finally:
        _teardown(teensy, client, node)


def test_odrive_command_clear_errors_shares_the_fallback_path():
    """Review 2026-07-15 (HIGH): the orchestrator's 'clear_errors' rides the
    odrive_command conduit, which previously kept the OLD hard-refusal armed
    reroute — the production command path could dead-end forever in the
    armed-no-stream state while the Trigger service had the fallback. Both
    must share ONE canonical path (_svc_clear_errors), including the
    recover-failed → disarm + direct-clear fallback."""
    from tests.ros.conftest import ODriveCommandService
    teensy, client, node = _node()
    try:
        box = _capture(teensy, RpcMethod.CLEAR_ERRORS)
        _bring_link_and_telem_up(teensy, node, leg_pos=0.1)
        node._mpc_active = True                               # ARMED
        node._recover_verify_timeout_s = 0.1
        node._sp_pump._prev_pos = [1.0] * 6                   # diverged forever
        node._reseed_client = _FakeReseedClient(success=True)

        req = ODriveCommandService.Request()
        req.command = 'clear_errors'
        res = node._svc_odrive_command(req, ODriveCommandService.Response())
        assert res.success is True, res.message
        assert node._mpc_active is False                      # disarmed first
        assert 'disarmed and cleared directly' in res.message
        assert 'args' in box                                  # clear DID land
    finally:
        _teardown(teensy, client, node)
