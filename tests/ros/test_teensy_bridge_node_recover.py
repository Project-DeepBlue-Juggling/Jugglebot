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

from teensy_link import (Diagnostic, FaultState, MsgType, RpcMethod,
                         RpcStatus, rpc_args)

from jugglebot.teensy_bridge_node import _GUARD_CLEAR_WAIT_S

from std_srvs.srv import SetBool, Trigger

from tests.ros._bridge_harness import _build_paired_node, _teardown, _wait_until
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


def _hand_diag_up(teensy, node, pos_rev=0.0):
    """Give the node the hand's (axis 6) Diagnostic + encoder so the recovery
    HAND PARK can make a decision.

    Every /recover now ends with the hand-park step (2026-09-17): a guard latch
    leaves the hand mid-stroke and the next schedule's opening REST cannot walk
    it home inside the lead authority — that is the double MAX_DEVIATION latch
    of 2026-09-17. Without a hand diagnostic the park REFUSES fail-safe (an
    unknown hand position must not be TRAP_TRAJ'd at), so the harness supplies
    one. ``pos_rev`` inside _HAND_PARK_BAND_REV of the park makes it a no-op,
    which is the state these leg-recovery tests are about.
    """
    teensy.send_telemetry(pos_rev=tuple([0.1] * 6 + [pos_rev]),
                          vel_rps=tuple([0.0] * 7))
    diag = Diagnostic(axis_id=6, axis_state=8, active_errors=0, disarm_reason=0)
    teensy.send_to_jetson(int(MsgType.DIAGNOSTIC), diag.pack())
    assert _wait_until(lambda: (node._latest_telemetry is not None
                                and 6 in node._latest_diag), timeout=2.0)


def test_recover_happy_path_reseeds_verifies_then_clears():
    teensy, client, node = _node()
    try:
        box = _capture(teensy, RpcMethod.CLEAR_ERRORS)
        _bring_link_and_telem_up(teensy, node, leg_pos=0.1)
        _hand_diag_up(teensy, node)   # encoder = 0.1 rev/leg
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
        _hand_diag_up(teensy, node)
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
        _hand_diag_up(teensy, node)
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
        _hand_diag_up(teensy, node)
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
        _hand_diag_up(teensy, node)
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
        _hand_diag_up(teensy, node)
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
        _hand_diag_up(teensy, node)
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
        _bring_link_and_telem_up(teensy, node, leg_pos=0.1)
        _hand_diag_up(teensy, node)   # encoder = 0.1 rev/leg
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
        _bring_link_and_telem_up(teensy, node, leg_pos=0.1)
        _hand_diag_up(teensy, node)   # LIVE encoder = 0.1 rev/leg
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
# through (the benign boot-time SETPOINT_STALE latch — output not evaluated, no jolt).

def test_armed_bare_clear_errors_routes_through_converge_first():
    teensy, client, node = _node()
    try:
        box = _capture(teensy, RpcMethod.CLEAR_ERRORS)
        _bring_link_and_telem_up(teensy, node, leg_pos=0.1)
        _hand_diag_up(teensy, node)
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
        _hand_diag_up(teensy, node)
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
    """When NOT armed (mpc_active=0 — the benign boot-time SETPOINT_STALE latch), the bare
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
        _hand_diag_up(teensy, node)
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
        _hand_diag_up(teensy, node)
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


# ── The hand park on guard-latch recovery (2026-09-17) ───────────────────────
# CONTRACT: recovery from a guard latch parks the hand through the same profiled
# firmware path ACTIVATE uses, before streaming can command it again.
#
# The sitting this encodes (bag `2026-09-17_18-45-10`): a latch left the hand at
# 8.7382 rev with cmd and encoder agreeing to 1e-4 — so the seed reconciliation
# correctly did nothing — `/clear_errors` cleared the latch and left it there,
# and the next schedule's opening REST planned the hand home at ~5.4 rev/s
# against a hand that follows at ~0.83 rev/s. The command ran 3.07 rev BELOW the
# encoder and MAX_DEVIATION_HAND_REV E-STOPPED the machine 0.69 s in. Twice.
# What cleared it was the operator's DEACTIVATE → ACTIVATE park.

def test_recover_parks_the_hand_the_latch_left_mid_stroke():
    """The hand is at the top of the throw stroke after the latch → /recover
    fires the single-axis ACTIVATE park (legs untouched) and only reports
    success once the hand has arrived."""
    teensy, client, node = _node()
    try:
        _capture(teensy, RpcMethod.CLEAR_ERRORS)
        act = _capture(teensy, RpcMethod.ACTIVATE)
        _bring_link_and_telem_up(teensy, node, leg_pos=0.1)
        _hand_diag_up(teensy, node, pos_rev=8.7382)       # the measured 2026-09-17 position
        node._sp_pump._prev_pos = [0.1] * 6
        node._reseed_client = _FakeReseedClient(success=True)

        def _park_arrives():
            time.sleep(0.3)                                # the TRAP_TRAJ walk
            teensy.send_telemetry(pos_rev=tuple([0.1] * 6 + [0.0]),
                                  vel_rps=tuple([0.0] * 7))

        th = threading.Thread(target=_park_arrives)
        th.start()
        res = node._svc_recover(Trigger.Request(), Trigger.Response())
        th.join()
        assert res.success is True, res.message
        # A SINGLE-AXIS activate: leg_activate.cpp maps that to the hand alone,
        # so the legs are never commanded off the converged descent.
        assert act['args'] == rpc_args.encode_activate(6), act
        assert 'hand parked' in res.message
        assert '8.7382' in res.message
    finally:
        _teardown(teensy, client, node)


def test_recover_hand_park_is_a_noop_when_the_hand_is_already_parked():
    """An in-band hand needs no park: no ACTIVATE is fired at all (a pointless
    10 s-budget firmware op on every recovery would be its own hazard)."""
    teensy, client, node = _node()
    try:
        _capture(teensy, RpcMethod.CLEAR_ERRORS)
        act = _capture(teensy, RpcMethod.ACTIVATE)
        _bring_link_and_telem_up(teensy, node, leg_pos=0.1)
        _hand_diag_up(teensy, node, pos_rev=0.0)
        node._sp_pump._prev_pos = [0.1] * 6
        node._reseed_client = _FakeReseedClient(success=True)

        res = node._svc_recover(Trigger.Request(), Trigger.Response())
        assert res.success is True, res.message
        assert 'already parked' in res.message
        assert 'args' not in act, 'fired an ACTIVATE at an already-parked hand'
    finally:
        _teardown(teensy, client, node)


def test_recover_refuses_when_the_hand_position_is_unknown():
    """FAIL-SAFE: no hand diagnostic ⇒ the hand position is a guess ⇒ do NOT
    TRAP_TRAJ at it. The guard is cleared, but /recover reports the recovery as
    incomplete and names the manual park — reporting success here would send the
    operator into the E-STOP the park exists to prevent."""
    teensy, client, node = _node()
    try:
        box = _capture(teensy, RpcMethod.CLEAR_ERRORS)
        act = _capture(teensy, RpcMethod.ACTIVATE)
        _bring_link_and_telem_up(teensy, node, leg_pos=0.1)   # no hand diagnostic
        node._sp_pump._prev_pos = [0.1] * 6
        node._reseed_client = _FakeReseedClient(success=True)

        res = node._svc_recover(Trigger.Request(), Trigger.Response())
        assert res.success is False
        assert 'HAND NOT PARKED' in res.message
        assert 'DEACTIVATE then ACTIVATE' in res.message
        assert 'args' in box                                  # the clear DID fire
        assert 'args' not in act                              # but no blind park
    finally:
        _teardown(teensy, client, node)


def test_recover_escape_hatch_also_parks_the_hand():
    """trajectory_node down (the escape hatch): the clear goes direct, and the
    park obligation is the same — more so, since nothing is streaming the hand."""
    teensy, client, node = _node()
    try:
        box = _capture(teensy, RpcMethod.CLEAR_ERRORS)
        act = _capture(teensy, RpcMethod.ACTIVATE)
        _bring_link_and_telem_up(teensy, node, leg_pos=0.1)
        _hand_diag_up(teensy, node, pos_rev=7.5281)       # where clear_errors #2 left it
        node._reseed_client = _FakeReseedClient(ready=False)

        def _park_arrives():
            time.sleep(0.3)
            teensy.send_telemetry(pos_rev=tuple([0.1] * 6 + [0.0]),
                                  vel_rps=tuple([0.0] * 7))

        th = threading.Thread(target=_park_arrives)
        th.start()
        res = node._svc_recover(Trigger.Request(), Trigger.Response())
        th.join()
        assert res.success is True, res.message
        assert 'args' in box and act['args'] == rpc_args.encode_activate(6)
        assert 'hand parked' in res.message
    finally:
        _teardown(teensy, client, node)


# ── The park waits for the guard to clear (2026-09-18) ───────────────────────
# `temp/logs/launch_r2gate_20260918_1325.log` lines 977-982: the operator's GUI
# clear_errors → `CLEAR_ERRORS fired` → the park's `ACTIVATE(axis 6)` came back
# `ERR_BUS_DOWN` ("... and fault_state=MAX_DEVIATION is currently latched") →
# `HAND NOT PARKED` — and the VERY NEXT log line was `Teensy guard fault
# cleared (fault_state=NONE)`. CLEAR_ERRORS is acked by the LINK; the latch is
# released by the firmware's 10 Hz fault task, one tick later. The park lost a
# race it did not know it was in, and the refusal then escalated through the
# armed /clear_errors disarm+direct-clear fallback, which is why the operator
# had to ACTIVATE/TRAJECTORY by hand.

def test_the_park_waits_out_the_10hz_fault_task_then_parks():
    """A latch that clears two polls AFTER the clear still parks: the park
    polls the cached fault_state rather than assuming the ack released it."""
    teensy, client, node = _node()
    try:
        act = _capture(teensy, RpcMethod.ACTIVATE)
        _bring_link_and_telem_up(teensy, node, leg_pos=0.1)
        _hand_diag_up(teensy, node, pos_rev=9.6227)   # the 2026-09-18 position
        # The guard is STILL latched at the instant the park is asked for.
        teensy.send_heartbeat_t2j(fault_state=int(FaultState.MAX_DEVIATION))
        assert _wait_until(
            lambda: node._latest_heartbeat is not None
            and int(node._latest_heartbeat.fault_state)
            == int(FaultState.MAX_DEVIATION), timeout=2.0)

        def _fault_task_clears_then_park_arrives():
            time.sleep(0.12)                      # ~two 0.05 s polls
            teensy.send_heartbeat_t2j(fault_state=int(FaultState.NONE))
            time.sleep(0.3)                       # then the TRAP_TRAJ walk
            teensy.send_telemetry(pos_rev=tuple([0.1] * 6 + [0.0]),
                                  vel_rps=tuple([0.0] * 7))

        th = threading.Thread(target=_fault_task_clears_then_park_arrives)
        th.start()
        ok, msg = node._park_hand()
        th.join()
        assert ok is True, msg
        assert 'hand parked' in msg
        assert act['args'] == rpc_args.encode_activate(6), act
    finally:
        _teardown(teensy, client, node)


def test_the_park_refuses_a_latch_that_never_clears_and_names_it():
    """A latch still up after the bounded wait ⇒ REFUSE, naming the latch —
    never fire an ACTIVATE the firmware will reject as ERR_BUS_DOWN and then
    report as an unexplained park failure."""
    teensy, client, node = _node()
    try:
        act = _capture(teensy, RpcMethod.ACTIVATE)
        _bring_link_and_telem_up(teensy, node, leg_pos=0.1)
        _hand_diag_up(teensy, node, pos_rev=9.6227)
        teensy.send_heartbeat_t2j(fault_state=int(FaultState.MAX_DEVIATION))
        assert _wait_until(
            lambda: node._latest_heartbeat is not None
            and int(node._latest_heartbeat.fault_state) != 0, timeout=2.0)

        t_start = time.monotonic()
        ok, msg = node._park_hand()
        waited = time.monotonic() - t_start
        assert ok is False
        assert 'STILL latched' in msg and 'MAX_DEVIATION' in msg
        assert 'args' not in act, 'fired ACTIVATE into a latched guard'
        # Bounded: it must not block the executor thread indefinitely.
        assert waited < 3.0 * _GUARD_CLEAR_WAIT_S, waited
    finally:
        _teardown(teensy, client, node)


def test_the_park_spends_no_wait_when_nothing_is_latched():
    """The schedule path (`/park_hand`, no latch at all) pays nothing for the
    wait: fault_state already reads NONE, so the park fires immediately."""
    teensy, client, node = _node()
    try:
        _bring_link_and_telem_up(teensy, node, leg_pos=0.1)
        _hand_diag_up(teensy, node, pos_rev=0.0)      # in band ⇒ no-op park
        t_start = time.monotonic()
        ok, msg = node._park_hand()
        assert ok is True and 'already parked' in msg
        assert time.monotonic() - t_start < 0.5
    finally:
        _teardown(teensy, client, node)


def test_the_park_refuses_when_there_is_no_heartbeat_to_read_the_guard_from():
    """FAIL-SAFE, same shape as the unknown-hand-position refusal: with no
    heartbeat the guard's state is unknown, so the park does not fire."""
    teensy, client, node = _node()
    try:
        act = _capture(teensy, RpcMethod.ACTIVATE)
        ok, msg = node._park_hand()
        assert ok is False
        assert 'no Teensy heartbeat' in msg
        assert 'args' not in act
    finally:
        _teardown(teensy, client, node)


# ── /park_hand: the schedule's caller of the same op (2026-09-18) ────────────

def test_park_hand_service_parks_a_hand_left_at_the_top_of_the_stroke():
    """`/park_hand` is what `skill_node` calls before it pre-levels or streams:
    the hand the previous attempt left at 9.63 rev comes home on the profiled
    ACTIVATE path, so the opening REST only ever has the park band to cover."""
    teensy, client, node = _node()
    try:
        act = _capture(teensy, RpcMethod.ACTIVATE)
        _bring_link_and_telem_up(teensy, node, leg_pos=0.1)
        _hand_diag_up(teensy, node, pos_rev=9.6227)

        def _park_arrives():
            time.sleep(0.3)
            teensy.send_telemetry(pos_rev=tuple([0.1] * 6 + [0.0]),
                                  vel_rps=tuple([0.0] * 7))

        th = threading.Thread(target=_park_arrives)
        th.start()
        res = node._svc_park_hand(Trigger.Request(), Trigger.Response())
        th.join()
        assert res.success is True, res.message
        assert act['args'] == rpc_args.encode_activate(6), act
        assert '9.6227' in res.message
    finally:
        _teardown(teensy, client, node)


def test_park_hand_service_is_a_noop_on_an_already_parked_hand():
    """Called before EVERY schedule, so the in-band answer must be free and
    must not fire a firmware op."""
    teensy, client, node = _node()
    try:
        act = _capture(teensy, RpcMethod.ACTIVATE)
        _bring_link_and_telem_up(teensy, node, leg_pos=0.1)
        _hand_diag_up(teensy, node, pos_rev=0.0)
        res = node._svc_park_hand(Trigger.Request(), Trigger.Response())
        assert res.success is True
        assert 'already parked' in res.message
        assert 'args' not in act
    finally:
        _teardown(teensy, client, node)


def test_park_hand_service_refusal_carries_the_manual_park_hint():
    """A refusal must tell the operator the escape (DEACTIVATE → ACTIVATE), the
    same hint /recover's refusal carries — the caller turns it into the
    schedule's own named refusal."""
    teensy, client, node = _node()
    try:
        _bring_link_and_telem_up(teensy, node, leg_pos=0.1)   # no hand diagnostic
        res = node._svc_park_hand(Trigger.Request(), Trigger.Response())
        assert res.success is False
        assert 'hand position is unknown' in res.message
        assert 'DEACTIVATE then ACTIVATE' in res.message
    finally:
        _teardown(teensy, client, node)


def test_park_hand_service_refuses_to_park_under_a_live_hand_lane():
    """THE MIRROR HAZARD (firmware, 2026-09-18): the op moves the AXIS while
    the STREAM keeps commanding. A hand-less hold leaves the firmware lane
    HOLDING its last knot (leg_interp.cpp:1105-1130), the guard measures that
    raw command against the encoder (leg_interp.cpp:1226-1228), and the lead
    clamp would drag the parked hand back up to it. Only a disarm/arm edge
    clears the lane (leg_interp.cpp:885) — so with the wire ARMED, `/park_hand`
    REFUSES instead of racing, and the operator's DEACTIVATE → ACTIVATE (which
    gives the edge AND parks) is the move."""
    teensy, client, node = _node()
    try:
        act = _capture(teensy, RpcMethod.ACTIVATE)
        _bring_link_and_telem_up(teensy, node, leg_pos=0.1)
        _hand_diag_up(teensy, node, pos_rev=9.6227)
        node._mpc_active = True

        res = node._svc_park_hand(Trigger.Request(), Trigger.Response())
        assert res.success is False
        assert 'the wire is ARMED' in res.message
        assert 'DEACTIVATE then ACTIVATE' in res.message
        assert 'args' not in act, 'parked the axis under a live hand lane'
    finally:
        _teardown(teensy, client, node)


def test_park_hand_service_noop_is_free_even_on_an_armed_wire():
    """The healthy schedule start: the hand IS parked, so the armed refusal
    above must not fire — it would refuse every attempt."""
    teensy, client, node = _node()
    try:
        act = _capture(teensy, RpcMethod.ACTIVATE)
        _bring_link_and_telem_up(teensy, node, leg_pos=0.1)
        _hand_diag_up(teensy, node, pos_rev=0.0)
        node._mpc_active = True

        res = node._svc_park_hand(Trigger.Request(), Trigger.Response())
        assert res.success is True and 'already parked' in res.message
        assert 'args' not in act
    finally:
        _teardown(teensy, client, node)


# ── The recovery park DISARMS first (2026-09-23) ─────────────────────────────
# `temp/logs/launch_r2gate_20260922_2321.log` 1790084102.60-102.61: `/recover`
# fired the park's ACTIVATE(6) on the armed wire and the firmware answered
# `ERR_REJECTED` — its MPC-stream interlock (`leg_activate.cpp:129`) refuses
# any cold-start op while `s_mpc_active` is set, whatever the hand lane is
# doing. Every recovery park in every log had been rejected the same way
# (2026-09-18 ×2 as ERR_BUS_DOWN before the guard-clear wait, 2026-09-22 ×2 as
# ERR_REJECTED after it); the hand was left at 5.57 rev, and the next schedule's
# opening REST walked it home through the 1 rev/s recovery slew into a 7.3 rev
# MAX_DEVIATION latch. So the recovery park disarms the wire, confirms the
# disarm on the wire, parks, and leaves the wire disarmed for the orchestrator's
# A2 arm phase — while it runs, `/link_status` reads `fault_state=RECOVERING`
# so the orchestrator stays in FAULT and does not arm under the moving hand.

def test_recover_disarms_an_armed_wire_to_park_and_leaves_it_disarmed():
    """Armed wire at the clear: the park still fires (the firmware would have
    rejected it armed), the wire is DISARMED before the ACTIVATE goes out and
    is still disarmed afterwards, and the response says so."""
    teensy, client, node = _node()
    try:
        _capture(teensy, RpcMethod.CLEAR_ERRORS)
        act = _capture(teensy, RpcMethod.ACTIVATE)
        _bring_link_and_telem_up(teensy, node, leg_pos=0.1)
        _hand_diag_up(teensy, node, pos_rev=5.5694)   # the 2026-09-22 position
        node._sp_pump._prev_pos = [0.1] * 6
        node._reseed_client = _FakeReseedClient(success=True)
        node._mpc_active = True
        armed_at_activate = []
        real_activate = node.teensy_activate

        def _activate_spy(axis):
            armed_at_activate.append(bool(node._mpc_active))
            return real_activate(axis)

        node.teensy_activate = _activate_spy

        def _park_arrives():
            time.sleep(0.3)
            teensy.send_telemetry(pos_rev=tuple([0.1] * 6 + [0.0]),
                                  vel_rps=tuple([0.0] * 7))

        th = threading.Thread(target=_park_arrives)
        th.start()
        res = node._svc_recover(Trigger.Request(), Trigger.Response())
        th.join()
        assert res.success is True, res.message
        assert act['args'] == rpc_args.encode_activate(6), act
        assert armed_at_activate == [False], armed_at_activate
        assert node._mpc_active is False
        assert 'hand parked' in res.message
        assert 'wire DISARMED' in res.message
        assert node._recovery_park_in_progress is False
    finally:
        _teardown(teensy, client, node)


def test_park_hand_refuses_on_an_armed_wire_before_firing_the_op():
    """`_park_hand` itself never fires ACTIVATE under an armed stream, from
    ANY caller: the firmware would reject it, and the refusal names the
    interlock and the disarm."""
    teensy, client, node = _node()
    try:
        act = _capture(teensy, RpcMethod.ACTIVATE)
        _bring_link_and_telem_up(teensy, node, leg_pos=0.1)
        _hand_diag_up(teensy, node, pos_rev=5.5694)
        node._mpc_active = True
        ok, msg = node._park_hand()
        assert ok is False
        assert 'ARMED' in msg and 'ERR_REJECTED' in msg
        assert 'args' not in act
    finally:
        _teardown(teensy, client, node)


def test_clear_errors_fallback_parks_the_hand_too():
    """The armed `/clear_errors` disarm+direct-clear fallback — where the
    2026-09-22 sitting actually landed — now parks the hand as well, on the
    already-disarmed wire, instead of leaving it mid-stroke."""
    teensy, client, node = _node()
    try:
        _capture(teensy, RpcMethod.CLEAR_ERRORS)
        act = _capture(teensy, RpcMethod.ACTIVATE)
        _bring_link_and_telem_up(teensy, node, leg_pos=0.1)
        _hand_diag_up(teensy, node, pos_rev=5.5694)
        node._mpc_active = True
        node._reseed_client = _FakeReseedClient(success=False,
                                                message='not streaming')

        def _park_arrives():
            time.sleep(0.3)
            teensy.send_telemetry(pos_rev=tuple([0.1] * 6 + [0.0]),
                                  vel_rps=tuple([0.0] * 7))

        th = threading.Thread(target=_park_arrives)
        th.start()
        res = node._svc_clear_errors(Trigger.Request(), Trigger.Response())
        th.join()
        assert res.success is True, res.message
        assert act['args'] == rpc_args.encode_activate(6), act
        assert 'hand parked' in res.message
        assert node._mpc_active is False
    finally:
        _teardown(teensy, client, node)


def test_fault_state_reads_recovering_only_while_the_park_runs():
    """`/link_status` carries RECOVERING while the recovery park runs on a
    firmware that reads NONE — never over a real fault, never with no
    heartbeat."""
    teensy, client, node = _node()
    try:
        assert node._published_fault_state(None) == 'UNKNOWN'
        _bring_link_and_telem_up(teensy, node, leg_pos=0.1)
        hb = node._latest_heartbeat
        assert node._published_fault_state(hb) == 'NONE'
        node._recovery_park_in_progress = True
        assert node._published_fault_state(hb) == 'RECOVERING'
        teensy.send_heartbeat_t2j(fault_state=int(FaultState.MAX_DEVIATION))
        assert _wait_until(
            lambda: int(node._latest_heartbeat.fault_state)
            == int(FaultState.MAX_DEVIATION), timeout=2.0)
        assert node._published_fault_state(
            node._latest_heartbeat) == 'MAX_DEVIATION'
    finally:
        node._recovery_park_in_progress = False
        _teardown(teensy, client, node)


def test_arm_is_refused_while_the_recovery_park_runs():
    """ARMING_CONTRACT A1 (a4): the orchestrator's A2 retry landing mid-park
    is refused fast and by name, rather than arming under a moving hand."""
    teensy, client, node = _node()
    try:
        _bring_link_and_telem_up(teensy, node, leg_pos=0.1)
        node._recovery_park_in_progress = True
        req = SetBool.Request()
        req.data = True
        resp = node._svc_set_setpoint_output(req, SetBool.Response())
        assert resp.success is False
        assert 'hand park in progress' in resp.message
        assert node._mpc_active is False
    finally:
        node._recovery_park_in_progress = False
        _teardown(teensy, client, node)


def test_recover_disarms_even_when_the_hand_is_already_parked():
    """The most common real case — a LEG latch with an armed wire and an
    in-band hand: the recovery still disarms before the (no-op) park and
    leaves the wire disarmed for the orchestrator's A2 re-arm. Deliberate
    (one recovery sequence, not two); a future change that makes the disarm
    conditional on the park being needed must show up here as an intentional
    diff, not a silent behaviour change."""
    teensy, client, node = _node()
    try:
        _capture(teensy, RpcMethod.CLEAR_ERRORS)
        act = _capture(teensy, RpcMethod.ACTIVATE)
        _bring_link_and_telem_up(teensy, node, leg_pos=0.1)
        _hand_diag_up(teensy, node, pos_rev=0.0)          # in band ⇒ no-op park
        node._sp_pump._prev_pos = [0.1] * 6
        node._reseed_client = _FakeReseedClient(success=True)
        node._mpc_active = True
        res = node._svc_recover(Trigger.Request(), Trigger.Response())
        assert res.success is True, res.message
        assert 'args' not in act                           # no ACTIVATE fired
        assert 'already parked' in res.message
        assert 'wire DISARMED' in res.message
        assert node._mpc_active is False
    finally:
        _teardown(teensy, client, node)
