"""Pytest entry point for the native (compiled) firmware test harness.

The safety-critical firmware (`fault_machine.cpp`, `leg_interp.cpp`) had **no
compiled test** before this — its correctness rested on three hand-synced Python
transcriptions, so a C++ edit that diverged from the mirrors passed the whole
suite (the standing risk flagged in `logbook/2026-06-27-can-node-teensy-parity-
audit.md` §5). This wrapper compiles the REAL `.cpp` on the build host behind the
~10-line HAL shims + the fake HAL (`tests/firmware/native/`) and runs the binary,
so a C++ divergence fails `pytest tests/ -q`.

It builds + runs one binary per safety-critical firmware TU, all hash-cached (zero
recompile when the sources are unchanged) and g++-gated (a host without a compiler
SKIPS, not fails — the Jetson run is authoritative):

  * `test_fault_machine` — soft-reset limiter, UV gating, the deferred-stow 5
    invariants, present-axis scoping, fb-stale suppression;
  * `test_leg_interp` — lead/stroke/present-axis clamps, modes, stow descent;
  * `test_platform_relay` — Phase-1 Platform-Teensy relay: trigger frames, 0x6E0
    RobotState re-encode parity, the never-command-a-dead-bus fail-fast,
    is_platform_reply_id, the generated hand axis-6 allow-table;
  * `test_version_check` — Phase-3 Get_Version sweep + raw-version cache;
  * `test_hand_ops` — Phase-5 hand traj/smooth-move conduit + preamble-abort;
  * `test_leg_activate` / `test_leg_deactivate` / `test_leg_homing` — the Phase-11
    cold-start move ladders (request validation, SETUP preamble byte-parity, the
    active/STOW COMMAND, the homing float32 Iq-EMA trip + HAND-vs-LEG dispatch, and
    abort-to-IDLE). These close coverage gap 1: before this, the code that drives
    legs into hardstops was never compiled by any test (Fable-5 hardening [6]);
  * plus a golden-conformance check: a freshly-emitted golden must still equal the
    committed `native/fault_golden.json`, so a firmware behaviour change regenerates
    the golden deliberately (and `tests/firmware/test_fault_logic.py` then pins the
    Jetson host mirror `fault_logic.py` to that same golden).

SCOPE: the harness validates DECISION LOGIC, not FreeRTOS/ISR concurrency or the
500 Hz deadline — those remain on-hardware-replay gaps (see native/README.md).
"""

from __future__ import annotations

import importlib.util
import json
import subprocess
from pathlib import Path

import pytest

_NATIVE = Path(__file__).resolve().parent / "native"
_BUILD_PY = _NATIVE / "build.py"
_GOLDEN = _NATIVE / "fault_golden.json"


def _load_build():
    spec = importlib.util.spec_from_file_location("cb_native_build", _BUILD_PY)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


_build = _load_build()

pytestmark = pytest.mark.skipif(
    not _build.have_gpp(),
    reason="g++ not available — the native firmware harness needs a host C++ compiler "
           "(the Jetson run is authoritative)")


@pytest.fixture(scope="module")
def binaries():
    """Hash-cached build of all native test binaries (compiles only on change)."""
    return _build.build_all()


def _run(path, *args):
    return subprocess.run([str(path), *args], capture_output=True, text=True)


def test_native_fault_machine_binary_passes(binaries):
    """The compiled fault_machine.cpp passes every doctest assertion (the
    robustness contract: a C++ divergence from the safety logic fails here)."""
    r = _run(binaries["test_fault_machine"])
    assert r.returncode == 0, (
        "native test_fault_machine FAILED — fault_machine.cpp diverged from the "
        f"safety contract:\n{r.stdout}\n{r.stderr}")


def test_native_leg_interp_binary_passes(binaries):
    """The compiled leg_interp.cpp passes every behaviour assertion (lead/stroke/
    present-axis clamps, mode transitions, stow descent)."""
    r = _run(binaries["test_leg_interp"])
    assert r.returncode == 0, (
        "native test_leg_interp FAILED — leg_interp.cpp diverged from the expected "
        f"interpolator behaviour:\n{r.stdout}\n{r.stderr}")


def test_native_platform_relay_binary_passes(binaries):
    """The compiled platform_relay.cpp passes every behaviour assertion (Phase 1
    Platform-Teensy relay: trigger frames, 0x6E0 RobotState re-encode parity, the
    never-command-a-dead-bus fail-fast, is_platform_reply_id, and the generated
    hand axis-6 allow-table)."""
    r = _run(binaries["test_platform_relay"])
    assert r.returncode == 0, (
        "native test_platform_relay FAILED — platform_relay.cpp / the relay seam "
        f"diverged from the expected behaviour:\n{r.stdout}\n{r.stderr}")


def test_native_version_check_binary_passes(binaries):
    """The compiled version_check.cpp passes every behaviour assertion (Phase 3
    Get_Version sweep + raw-version cache: one frame/tick bus-paced sweep of
    present axes, absent-axis skip, the never-command-a-dead-bus gate, and the
    ResultAxisVersions blob fill via the inbound-CAN3 injection hook)."""
    r = _run(binaries["test_version_check"])
    assert r.returncode == 0, (
        "native test_version_check FAILED — version_check.cpp / the version "
        f"handshake diverged from the expected behaviour:\n{r.stdout}\n{r.stderr}")


def test_native_hand_ops_binary_passes(binaries):
    """The compiled hand_ops.cpp passes every behaviour assertion (Phase 5 hand
    conduit: the CLOSED_LOOP + POSITION/PASSTHROUGH preamble to axis 6, the 0x6D0
    payload forwarded verbatim on the firmware-owned id, the never-command-a-dead-
    bus gate, and — the safety crux — the traj TX ABORTS with no 0x6D0 frame if a
    preamble send fails)."""
    r = _run(binaries["test_hand_ops"])
    assert r.returncode == 0, (
        "native test_hand_ops FAILED — hand_ops.cpp / the hand traj conduit "
        f"diverged from the expected behaviour:\n{r.stdout}\n{r.stderr}")


def test_native_leg_activate_binary_passes(binaries):
    """The compiled leg_activate.cpp passes every behaviour assertion (Phase-11 U5
    ACTIVATE: request validation — dead-bus/E-STOP/concurrent-move/no-present-leg/
    idempotent — the 5-frame SETUP preamble byte-parity, the active-pose COMMAND, and
    the safety crux that ANY abort leaves the leg in IDLE). Coverage gap 1."""
    r = _run(binaries["test_leg_activate"])
    assert r.returncode == 0, (
        "native test_leg_activate FAILED — leg_activate.cpp diverged from the "
        f"expected activate-ladder behaviour:\n{r.stdout}\n{r.stderr}")


def test_native_leg_deactivate_binary_passes(binaries):
    """The compiled leg_deactivate.cpp passes every behaviour assertion (Phase-11 U5
    DEACTIVATE: shared request validation + SETUP preamble, plus the two behaviours
    that distinguish it — the STOW-pose COMMAND and the IDLE-on-arrival that safes
    the robot de-energised — and abort-to-IDLE). Coverage gap 1."""
    r = _run(binaries["test_leg_deactivate"])
    assert r.returncode == 0, (
        "native test_leg_deactivate FAILED — leg_deactivate.cpp diverged from the "
        f"expected deactivate-ladder behaviour:\n{r.stdout}\n{r.stderr}")


def test_native_leg_homing_binary_passes(binaries):
    """The compiled leg_homing.cpp passes every behaviour assertion (Phase-9b/Phase-5
    HOME: axis-range validation incl. the hand axis, the CLOSED_LOOP+VEL_RAMP+limit
    SETUP preamble, the HAND-vs-LEG per-axis param dispatch — the port-bug surface —
    the REAL float32 Iq-EMA trip → set_absolute_position, and abort-to-IDLE). This
    runs the real float32 EMA, strictly stronger than the retired float64 Python
    transcription. Coverage gap 1."""
    r = _run(binaries["test_leg_homing"])
    assert r.returncode == 0, (
        "native test_leg_homing FAILED — leg_homing.cpp diverged from the expected "
        f"homing-ladder behaviour:\n{r.stdout}\n{r.stderr}")


def test_native_udp_framing_binary_passes(binaries):
    """The compiled C++ UDP framing (crc16_ccitt / encode_frame / decode_frame in
    udp_protocol.h — the network trust boundary) passes every assertion: the
    canonical CRC-16/CCITT-FALSE check value, encode→decode round-trip, and every
    rejection the firmware relies on (runt, bad magic/version, length mismatch, CRC
    corruption). Coverage gap 4 — before this the C++ codec was never executed."""
    r = _run(binaries["test_udp_framing"])
    assert r.returncode == 0, (
        "native test_udp_framing FAILED — the C++ UDP framing codec diverged from "
        f"the expected behaviour:\n{r.stdout}\n{r.stderr}")


def test_native_udp_link_binary_passes(binaries):
    """The compiled udp_link.cpp passes every assertion for Tier-2 hardening [15]:
    (a) LOCK COVERAGE — the fake QNEthernet asserts Ethernet.loop() (the lwIP pump)
    AND every socket TX/RX runs under NetLock; driving udp_link_service()+send trips
    zero violations (removing the pump's NetLock wrap makes it > 0 — the regression
    witness for the NO_SYS=1 non-reentrancy hazard); (b) DRAIN BUDGET — drain_socket
    is bounded by UDP_RX_DRAIN_BUDGET(=8): 20 staged packets drain exactly 8/wake,
    drain_cap_hits increments, and the backlog survives to the next wake; (c) at
    <= 1 frame/wake drain_cap_hits stays 0. SCOPE: lock COVERAGE + the drain budget,
    NOT true preemptive reentrancy (an on-hardware-replay gap — see native/README.md)."""
    r = _run(binaries["test_udp_link"])
    assert r.returncode == 0, (
        "native test_udp_link FAILED — udp_link.cpp diverged from the NetLock-coverage "
        f"/ RX drain-budget contract (Tier-2 [15]):\n{r.stdout}\n{r.stderr}")


def test_native_rpc_dispatch_binary_passes(binaries):
    """The compiled rpc.cpp dispatch()/send_axis_frame() passes every assertion: the
    (method,axis) enforcement point (hand-axis-6 allow-table, leg-axis bounds), the
    arg-decode bounds, the Phase-6 gate-basis split (CLEAR/REBOOT on bus-transmittable
    vs everything else on staleness — the 2026-06-27 just-repowered-bus crux), the
    AXIS_ALL fan-out, the reboot-latch-arms-for-leg-not-hand branch, and BB presence +
    range-check routing. Coverage gap 3 — the dispatch was never compiled before (only
    a text-regex lint). Handlers stubbed for routing isolation."""
    r = _run(binaries["test_rpc_dispatch"])
    assert r.returncode == 0, (
        "native test_rpc_dispatch FAILED — rpc.cpp dispatch/send_axis_frame diverged "
        f"from the expected (method,axis)/gate-basis behaviour:\n{r.stdout}\n{r.stderr}")


def test_native_odrive_protocol_binary_passes(binaries):
    """The compiled odrive_protocol.h encoders pass their self-checks (arb_id packing,
    the <fhh set_input_pos round-trip, zero-length get_version). Coverage gap 8 — the
    real header is now EXECUTED; the cross-language byte parity is pinned by the
    committed-golden guard below + the odrive.py reproduction in
    tests/firmware/test_odrive_protocol_xref.py."""
    r = _run(binaries["test_odrive_protocol"])
    assert r.returncode == 0, (
        "native test_odrive_protocol FAILED — odrive_protocol.h diverged from the "
        f"expected encoder behaviour:\n{r.stdout}\n{r.stderr}")


def test_odrive_committed_golden_matches_live_firmware(binaries, tmp_path):
    """A freshly-emitted odrive golden equals committed native/odrive_protocol_golden.json
    — the C++-side drift guard (gap 8). If odrive_protocol.h's encoders change, the fresh
    emission diverges and this fails, forcing a deliberate regeneration:
        python tests/firmware/native/build.py --odrive-golden \
            tests/firmware/native/odrive_protocol_golden.json
    (and, if the change is intended, a matching odrive.py update so the Python xref
    reproduction still equals the golden)."""
    fresh = tmp_path / "odrive_protocol_golden.json"
    r = _run(binaries["test_odrive_protocol"], "--emit-golden", str(fresh))
    assert r.returncode == 0, f"odrive golden emission failed:\n{r.stdout}\n{r.stderr}"
    committed = json.loads((_NATIVE / "odrive_protocol_golden.json").read_text())
    regenerated = json.loads(fresh.read_text())
    assert regenerated == committed, (
        "tests/firmware/native/odrive_protocol_golden.json is stale vs the live "
        "odrive_protocol.h. Regenerate it:\n"
        "  python tests/firmware/native/build.py --odrive-golden "
        "tests/firmware/native/odrive_protocol_golden.json\n"
        "and update odrive.py / the xref reproduction to match if intended.")


def test_native_ball_butler_binary_passes(binaries):
    """The compiled ball_butler_protocol.h encoders pass their self-checks (encode_throw
    id/len/offsets/scaling, throw_args_valid accept + each out-of-range/NaN/Inf reject,
    the payloadless state-command ids). Coverage gap 7 — the one canbridge firmware
    encoder that was never compiled. Byte parity vs ball_butler.py:
    tests/firmware/test_ball_butler_xref.py."""
    r = _run(binaries["test_ball_butler_protocol"])
    assert r.returncode == 0, (
        "native test_ball_butler_protocol FAILED — ball_butler_protocol.h diverged "
        f"from the expected encoder behaviour:\n{r.stdout}\n{r.stderr}")


def test_bb_committed_golden_matches_live_firmware(binaries, tmp_path):
    """A freshly-emitted BB golden equals committed native/ball_butler_golden.json —
    the C++-side drift guard (gap 7). Regenerate on an intended change:
        python tests/firmware/native/build.py --bb-golden \
            tests/firmware/native/ball_butler_golden.json
    (and update ball_butler.py so the Python xref reproduction still matches)."""
    fresh = tmp_path / "ball_butler_golden.json"
    r = _run(binaries["test_ball_butler_protocol"], "--emit-golden", str(fresh))
    assert r.returncode == 0, f"BB golden emission failed:\n{r.stdout}\n{r.stderr}"
    committed = json.loads((_NATIVE / "ball_butler_golden.json").read_text())
    regenerated = json.loads(fresh.read_text())
    assert regenerated == committed, (
        "tests/firmware/native/ball_butler_golden.json is stale vs the live "
        "ball_butler_protocol.h. Regenerate:\n"
        "  python tests/firmware/native/build.py --bb-golden "
        "tests/firmware/native/ball_butler_golden.json")


def test_committed_golden_matches_live_firmware(binaries, tmp_path):
    """A freshly-emitted golden equals the committed native/fault_golden.json.

    The golden is the firmware-anchored conformance source the Jetson host mirror
    (`fault_logic.py`) is pinned to. If the fault machine's behaviour changes, the
    fresh emission diverges from the committed file and this fails — forcing a
    deliberate regeneration (and a matching `fault_logic.py` update), which is
    exactly the three-way coupling the manual transcription used to rely on, now
    enforced by a test instead of by hand.
    """
    fresh = tmp_path / "fault_golden.json"
    r = _run(binaries["test_fault_machine"], "--emit-golden", str(fresh))
    assert r.returncode == 0, f"golden emission failed:\n{r.stdout}\n{r.stderr}"
    committed = json.loads(_GOLDEN.read_text())
    regenerated = json.loads(fresh.read_text())
    assert regenerated == committed, (
        "tests/firmware/native/fault_golden.json is stale vs the live firmware "
        "(fault_machine.cpp behaviour changed). Regenerate it:\n"
        "  python tests/firmware/native/build.py --golden "
        "tests/firmware/native/fault_golden.json\n"
        "and update controller/teensy_link/fault_logic.py to match if the change "
        "was intended.")
