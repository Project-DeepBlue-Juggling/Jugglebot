"""The owner's Leg IDLE rule on the bridge's operator services (ARMING_CONTRACT A7).

"A Leg should never be made IDLE unless the Platform is already at the Stow
position" (owner, 2026-11-25). /reboot_odrives (and odrive_command
reboot_odrives), /encoder_search and a manual /home IDLE or move Legs; each now
refuses unless the bridge's own fresh Leg states show every present Leg Idle
(for a Home of one Leg: every OTHER present Leg). Hand-only requests are exempt.

These tests drive the real input path, the DIAGNOSTIC cache read by
TeensyBridgeNode._present_leg_states, so they build the node with
legs_at_stow=False (the harness otherwise stubs six Idle Legs). Mutation
(2026-11-25): with TeensyBridgeNode._leg_idle_rule_refusal returning None
unconditionally, every refusal test below fails (the RPC is sent).
"""
from __future__ import annotations

import time

import pytest

from tests.ros import conftest  # noqa: F401  (mock rclpy, shared class identity)
from tests.ros._bridge_harness import _build_paired_node, _teardown
from teensy_link import Diagnostic
from std_srvs.srv import Trigger

IDLE, CLOSED_LOOP = 1, 8
HB_SEEN, HB_STALE = 0x2, 0x1


@pytest.fixture
def bridge():
    teensy, client, node = _build_paired_node(legs_at_stow=False)
    try:
        yield node
    finally:
        _teardown(teensy, client, node)


def _diag(node, states, *, flags=None, age_s=0.0):
    """Put one DIAGNOSTIC per Leg in the cache. ``states[i]`` None = no
    DIAGNOSTIC for that Leg; 'absent' = a drive never heartbeated (HB_SEEN 0)."""
    now = time.monotonic()
    with node._lock:
        node._latest_diag.clear()
        node._latest_diag_mono.clear()
        for leg, s in enumerate(states):
            if s is None:
                continue
            f = (0 if s == 'absent' else HB_SEEN) if flags is None else flags
            node._latest_diag[leg] = Diagnostic(
                axis_id=leg, axis_state=IDLE if s == 'absent' else s,
                active_errors=0, flags=f)
            node._latest_diag_mono[leg] = now - age_s


def _spy(node, name):
    calls = []
    orig = getattr(node, name)

    def spy(*a, **kw):
        calls.append(a)
        return orig(*a, **kw)
    setattr(node, name, spy)
    return calls


# ── /reboot_odrives and odrive_command reboot_odrives ─────────────────────

@pytest.mark.parametrize('states', [
    [CLOSED_LOOP] * 6,                         # Platform posed
    [IDLE] + [CLOSED_LOOP] * 5,                # one dropped, five holding
    [IDLE] * 5 + [None],                       # a Leg with no DIAGNOSTIC
])
def test_reboot_refused_unless_every_present_leg_is_idle(bridge, states):
    _diag(bridge, states)
    rpc = _spy(bridge, 'teensy_reboot')
    ok, msg = bridge._reboot_odrives()
    assert not ok and 'A7' in msg and '/deactivate' in msg
    assert rpc == []                           # nothing reached the wire


def test_both_reboot_entry_points_share_the_gate(bridge):
    _diag(bridge, [CLOSED_LOOP] * 6)
    rpc = _spy(bridge, 'teensy_reboot')
    res = bridge._svc_reboot_odrives(Trigger.Request(), Trigger.Response())
    assert res.success is False and 'A7' in res.message
    req = type('R', (), {'command': 'reboot_odrives'})()
    res = bridge._svc_odrive_command(req, Trigger.Response())
    assert res.success is False and 'A7' in res.message
    assert rpc == []


def test_reboot_refused_on_stale_or_silent_leg_state(bridge):
    _diag(bridge, [IDLE] * 6, age_s=10.0)      # cache frozen (link died)
    assert 'state unknown' in bridge._reboot_odrives()[1]
    _diag(bridge, [IDLE] * 6, flags=HB_SEEN | HB_STALE)   # drives went quiet
    assert 'state unknown' in bridge._reboot_odrives()[1]


def test_reboot_passes_at_stow_and_ignores_legs_not_on_the_rig(bridge):
    for states in ([IDLE] * 6, [IDLE] + ['absent'] * 5):   # robot, one-Leg bench
        _diag(bridge, states)
        assert bridge._leg_idle_rule_refusal('reboot_odrives', range(6)) is None


# ── /encoder_search ───────────────────────────────────────────────────────

def test_encoder_search_refused_with_a_leg_closed_loop(bridge):
    _diag(bridge, [IDLE] * 5 + [CLOSED_LOOP])
    calls = []
    bridge._run_encoder_search = lambda axes, **kw: calls.append(axes) or (True, 'x')
    res = bridge._svc_encoder_search(Trigger.Request(), Trigger.Response())
    assert res.success is False and 'Leg 5 state 8' in res.message
    assert calls == []
    _diag(bridge, [IDLE] * 6)
    res = bridge._svc_encoder_search(Trigger.Request(), Trigger.Response())
    assert res.success is True and calls == [list(range(6))]


# ── /home (and the orchestrator's home_motors, the same _do_home body) ────

def test_home_refused_while_any_other_leg_is_not_idle(bridge):
    _diag(bridge, [IDLE] * 5 + [CLOSED_LOOP])
    runs = []
    bridge._run_home = lambda axes: runs.append(axes) or (True, 'homed')
    writes = _spy(bridge, '_write_is_homed')
    ok, msg = bridge._do_home()                # default home_axes: Legs + Hand
    assert not ok and 'home REFUSED' in msg
    assert runs == [] and writes == []         # nothing moved, is_homed untouched
    assert bridge._home_in_progress is False   # single-flight released


def test_home_of_one_leg_needs_only_the_others_idle(bridge):
    # A one-Leg Home (bench config [0]) may start with that Leg held; the
    # other present Legs must be Idle.
    _diag(bridge, [CLOSED_LOOP] + [IDLE] * 5)
    assert bridge._leg_idle_rule_refusal('home', [0], exempt_target=True) is None
    _diag(bridge, [CLOSED_LOOP, CLOSED_LOOP] + [IDLE] * 4)
    msg = bridge._leg_idle_rule_refusal('home', [0], exempt_target=True)
    assert msg is not None and 'Leg 1' in msg and 'Leg 0' not in msg


def test_hand_only_requests_are_exempt(bridge):
    _diag(bridge, [CLOSED_LOOP] * 6)           # Platform up, Hand-only verb
    assert bridge._leg_idle_rule_refusal('home', [6], exempt_target=True) is None
    assert bridge._leg_idle_rule_refusal('encoder_search', [6]) is None
