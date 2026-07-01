"""Lint: every RpcMethod has a firmware dispatch case, and no orphan cases exist.

The UDP protocol (`config/generate_udp_protocol.py`) is the single source for the
`RpcMethod` id space; the firmware `rpc.cpp` `dispatch()` switch is the single
enforcement point that turns an id into an action. These two can silently drift:

  * a method gets reserved in the codegen but no firmware case is ever added —
    it answers the generic `ERR_UNKNOWN_METHOD` default instead of an explicit,
    intentional status (this is exactly how the audit's reserved-but-dead
    constants accreted);
  * a `case RpcMethod::X` references an `X` the codegen no longer defines (a
    stale/renamed opcode) — which fails to compile only if someone happens to
    rebuild the firmware, not in `pytest`.

This lint closes both directions structurally, so a parallel session that adds a
wire id (or renames one) without touching the firmware fails here, in pytest,
with a clear message — not silently at runtime on the bench.

Pure stdlib + the generated module (no ROS2, no g++, no hardware).
"""

from __future__ import annotations

import re
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[2]
_RPC_CPP = _REPO_ROOT / "ros_ws" / "src" / "jugglebot" / "Teensy_code_canbridge" / "rpc.cpp"

# `config/generated` is on sys.path via tests/conftest.py (same as the xlang test).
import udp_protocol  # noqa: E402


def _enum_member_names() -> set:
    return {m.name for m in udp_protocol.RpcMethod}


def _dispatched_case_names() -> set:
    """Every `case RpcMethod::NAME:` token in rpc.cpp (incl. fall-through cases)."""
    text = _RPC_CPP.read_text()
    return set(re.findall(r"case\s+RpcMethod::(\w+)\s*:", text))


def test_every_rpc_method_has_a_dispatch_case():
    """Every reserved RpcMethod id is wired to an explicit firmware case.

    A method with no case falls through to the `default: ERR_UNKNOWN_METHOD`,
    which is indistinguishable from a genuinely unknown id — so a reserved id
    MUST carry its own case (an ERR_NOT_IMPL stub is fine; that is the explicit,
    intentional 'not in this firmware revision' answer the later phase replaces).
    """
    methods = _enum_member_names()
    dispatched = _dispatched_case_names()
    missing = methods - dispatched
    assert not missing, (
        f"RpcMethod(s) with no rpc.cpp dispatch case: {sorted(missing)}. "
        "Add an explicit `case RpcMethod::<name>:` (an ERR_NOT_IMPL stub for a "
        "reserved id) so the firmware answers intentionally, not via the generic "
        "ERR_UNKNOWN_METHOD default.")


def test_no_orphan_dispatch_cases():
    """No `case RpcMethod::X` references an id the codegen no longer defines.

    Catches a stale/renamed opcode left behind in the switch — it would only ever
    surface as a firmware compile error, never in pytest, without this guard.
    """
    methods = _enum_member_names()
    dispatched = _dispatched_case_names()
    orphans = dispatched - methods
    assert not orphans, (
        f"rpc.cpp dispatch case(s) referencing a non-existent RpcMethod: "
        f"{sorted(orphans)}. Either the id was renamed/removed in "
        "config/generate_udp_protocol.py or the case is stale — reconcile them.")


def test_reserved_methods_are_stubbed_not_unknown():
    """The still-reserved ids answer ERR_NOT_IMPL (an explicit, intentional stub),
    not ERR_UNKNOWN_METHOD — so a caller can tell 'reserved, not yet implemented in
    this firmware' apart from 'garbage id'. This pins the stub contract the later
    phases replace. (Phase 1 replaced the TILT_READ/STATE_READ/STATE_WRITE stubs
    with the real Platform-Teensy relay dispatch; Phase 3 replaced GET_AXIS_VERSIONS
    with the real version pull — see test_get_axis_versions_is_implemented; only
    HAND_TRAJ_CMD [Phase 5] remains stubbed.)"""
    reserved = ["HAND_TRAJ_CMD"]
    text = _RPC_CPP.read_text()
    # Find the dispatch() body and confirm each reserved case routes to ERR_NOT_IMPL.
    # The reserved cases fall through to one shared `return RpcStatus::ERR_NOT_IMPL;`.
    block = re.search(r"// ── Reserved.*?return RpcStatus::ERR_NOT_IMPL;", text, re.DOTALL)
    assert block, "reserved-methods ERR_NOT_IMPL stub block not found in rpc.cpp"
    for name in reserved:
        assert f"RpcMethod::{name}" in block.group(0), (
            f"reserved method {name} is not in the ERR_NOT_IMPL stub block")


def _dispatch_case_body(name: str) -> str:
    """The body of `case RpcMethod::NAME:` up to the next case/default in rpc.cpp."""
    text = _RPC_CPP.read_text()
    m = re.search(
        rf"case\s+RpcMethod::{name}\s*:(.*?)(?=\n\s*case\s+RpcMethod::|\n\s*default\s*:)",
        text, re.DOTALL)
    assert m, f"{name} dispatch case not found in rpc.cpp"
    return m.group(1)


def test_clear_reboot_gate_on_bus_transmittable_not_staleness():
    """Phase 6: CLEAR_ERRORS and REBOOT_ODRIVES must gate on the bus-transmittable
    (SYNCH) signal via the shared gate_allows() chokepoint, NOT on the raw heartbeat-
    staleness predicate jugglebot_commands_allowed() — so a recovery clear reaches a
    just-repowered bus that is electrically alive but not yet heartbeating (the
    2026-06-27 deadlock; audit rec 'carve clear/reboot out of the bus-health gate').

    This is the wiring half of the dispatch-gate mirror guard (the policy half is the
    native harness test of method_gates_on_bus_transmittable): it fails if a future
    edit silently re-gates clear/reboot back onto heartbeat-staleness — the exact
    regression Phase 6 fixed.
    """
    text = _RPC_CPP.read_text()
    assert "method_gates_on_bus_transmittable" in text, (
        "rpc.cpp lost the method→gate-basis policy predicate (Phase 6)")
    assert "jugglebot_bus_transmittable" in text, (
        "rpc.cpp lost the bus-transmittable (SYNCH) gate (Phase 6)")
    for name in ("CLEAR_ERRORS", "REBOOT_ODRIVES"):
        body = _dispatch_case_body(name)
        assert "jugglebot_commands_allowed" not in body, (
            f"{name} must NOT gate on jugglebot_commands_allowed (heartbeat-staleness); "
            "route its AXIS_ALL branch through gate_allows() so a recovery clear reaches "
            "a just-repowered bus (Phase 6 SYNCH gate)")
        assert "gate_allows" in body, (
            f"{name} AXIS_ALL branch must gate through gate_allows() (Phase 6)")


def test_clear_reboot_axis_all_include_the_hand():
    """Phase 6 / audit rows 21+40: the CLEAR_ERRORS and REBOOT_ODRIVES AXIS_ALL loops
    must iterate legs + hand (i < NUM_AXES), matching can_node's JUGGLEBOT_AXES — the
    old `i < NUM_LEGS` dropped the hand (axis 6). Guards against a regression back to
    a legs-only broadcast."""
    for name in ("CLEAR_ERRORS", "REBOOT_ODRIVES"):
        body = _dispatch_case_body(name)
        assert "i < NUM_AXES" in body, (
            f"{name} AXIS_ALL loop must iterate `i < NUM_AXES` (legs + hand), not "
            "`i < NUM_LEGS` — can_node JUGGLEBOT_AXES parity (Phase 6)")
        assert "i < NUM_LEGS" not in body, (
            f"{name} AXIS_ALL loop still uses `i < NUM_LEGS` — it drops the hand (axis 6)")


def test_reboot_arms_the_watchdog_suppression_latch():
    """Phase 6: the REBOOT_ODRIVES dispatch must arm the reboot-in-progress latch
    (fault_notify_reboot_started) so the deliberate reboot silence does not false-trip
    the CAN-loss deferred stow. CLEAR_ERRORS must NOT arm it (a clear is not a reboot).
    The per-axis arm must be LEG-guarded (`a.axis < NUM_LEGS`): the CAN-loss detector
    watches leg heartbeats only, so a hand-only reboot would blind leg-loss detection
    for the full window with no benefit (audit NOTE 1)."""
    reboot = _dispatch_case_body("REBOOT_ODRIVES")
    assert "fault_notify_reboot_started" in reboot, (
        "REBOOT_ODRIVES must arm the watchdog-suppression latch (Phase 6)")
    assert "a.axis < NUM_LEGS" in reboot, (
        "the per-axis REBOOT_ODRIVES latch-arm must be leg-guarded (a.axis < NUM_LEGS) "
        "so a hand-only reboot does not blind leg-loss detection (audit NOTE 1)")
    clear = _dispatch_case_body("CLEAR_ERRORS")
    assert "fault_notify_reboot_started" not in clear, (
        "CLEAR_ERRORS must not arm the reboot latch (only a reboot silences the bus)")


def test_get_axis_versions_is_implemented():
    """Phase 3: GET_AXIS_VERSIONS is no longer a reserved ERR_NOT_IMPL stub — it has
    a real dispatch case that returns the cached version blob (version_fill_blob).
    Guards against a regression that re-stubs it (which would re-wedge the
    orchestrator BOOT on firmware_validated=False)."""
    text = _RPC_CPP.read_text()
    # The real case calls version_fill_blob into the result buffer, then returns.
    # Capture only up to the case's first `return ...;` (NOT the following reserved
    # comment block, whose prose mentions ERR_NOT_IMPL).
    m = re.search(r"case\s+RpcMethod::GET_AXIS_VERSIONS\s*:(.*?return[^;]*;)",
                  text, re.DOTALL)
    assert m, "GET_AXIS_VERSIONS dispatch case not found in rpc.cpp"
    body = m.group(1)
    assert "version_fill_blob" in body, (
        "GET_AXIS_VERSIONS must call version_fill_blob (Phase 3), not stub ERR_NOT_IMPL")
    assert "ERR_NOT_IMPL" not in body, (
        "GET_AXIS_VERSIONS must not route to ERR_NOT_IMPL (Phase 3 implemented it)")
