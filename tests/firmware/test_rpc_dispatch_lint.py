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
