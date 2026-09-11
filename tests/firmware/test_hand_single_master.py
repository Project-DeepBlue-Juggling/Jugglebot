"""The hand has exactly ONE master — a source scan over both firmware images.

INVARIANT (skill-stack R1, ``plans/active/two-ball-skill-stack.md`` § 4 R1,
I-FW-15):

    **At every instant, exactly one piece of code may write a motion command to
    axis 6.**  While the robot is streaming, that writer is the 2 ms interp tick
    in ``leg_interp.cpp`` and nothing else.  The two cold-start ops that must
    also touch axis 6 — HOME (drives it onto its stop) and ACTIVATE (the one-shot
    park at ``JBOp::HAND_ACTIVATE_POSITION_REV``) — run behind
    ``leg_interp.cpp``'s ``coldstart`` interlock, which suppresses the entire
    7-frame streaming burst, hand included, for the whole duration of the op.
    So the writers are serialised by construction, not by convention.

Why a source scan and not a runtime test: the failure this closes is a NEW
producer appearing — a second conduit, a helper that "just nudges the hand", a
resurrected stroke engine.  A runtime test can only exercise the producers that
exist when it is written; the thing that has actually bitten this project twice
is a producer nobody remembered was there.  ``hand_source`` (the latch that
arbitrated between two masters) and ``hand_ops`` (a host-driven stroke conduit
on the net task, out of phase with the interp tick) were both deleted at R1
precisely because a *latch* is what you need when there is more than one writer.

The scan strips comments first: every firmware file here carries a tombstone
comment naming the deleted producers, and a raw substring search reads those
tombstones as resurrections.
"""
from __future__ import annotations

import os
import re

import pytest

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_CANBRIDGE = os.path.join(_REPO, "ros_ws", "src", "jugglebot", "Teensy_code_canbridge")
_PLATFORM = os.path.join(_REPO, "ros_ws", "src", "jugglebot", "Teensy_code_platform")

#: The ODrive codec entry points that put a MOTION command on the wire.  Every
#: other ``encode_*`` in ``odrive_protocol.h`` is configuration (gains, limits,
#: modes), a state change, or a reference set — none of them move an axis.
_MOTION_ENCODERS = (
    "encode_leg_setpoint",
    "encode_set_input_pos",
    "encode_set_input_vel",
    "encode_set_input_torque",
)

#: Translation units allowed to emit a motion command to ANY axis, with the role
#: that earns each one the right.  A new entry here is a design change and must
#: be argued against the invariant in this module's docstring — it is not a
#: bookkeeping update.
_ALLOWED_MOTION_TUS = {
    "leg_interp.cpp": "the 500 Hz streamed lane — THE hand master",
    "leg_activate.cpp": "the one-shot ACTIVATE park (behind the coldstart interlock)",
    "leg_deactivate.cpp": "the legs' profiled descent (the hand only ever IDLEs)",
    "leg_homing.cpp": "the homing drive onto the hardstop (behind the interlock)",
}


def _strip_comments(text: str) -> str:
    """Drop ``//`` and ``/* */`` comments. Tombstones are comments, not code."""
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.DOTALL)
    return re.sub(r"//[^\n]*", "", text)


def _sources(directory: str, suffixes: tuple) -> dict:
    out = {}
    for name in sorted(os.listdir(directory)):
        if name.endswith(suffixes):
            with open(os.path.join(directory, name)) as fh:
                out[name] = _strip_comments(fh.read())
    return out


@pytest.fixture(scope="module")
def canbridge():
    return _sources(_CANBRIDGE, (".cpp", ".ino"))


@pytest.fixture(scope="module")
def platform():
    return _sources(_PLATFORM, (".cpp", ".ino"))


def test_only_the_sanctioned_tus_emit_a_motion_command(canbridge):
    """No unexpected translation unit commands motion on any axis."""
    found = {
        name: [e for e in _MOTION_ENCODERS if e in body]
        for name, body in canbridge.items()
        if any(e in body for e in _MOTION_ENCODERS)
    }
    # telemetry.cpp re-encodes the hand's last commanded triple to fill the
    # HandCmdEcho uplink payload. It builds a frame and never transmits it — it
    # is a byte-formatter, not a producer — so it is excluded by construction
    # rather than allow-listed, and this assert proves the exclusion still holds.
    echo = canbridge.get("telemetry.cpp", "")
    assert "encode_leg_setpoint" in echo, "telemetry.cpp no longer builds the hand echo"
    assert "can_jugglebot_send" not in echo and "can_jugglebot_tx" not in echo, (
        "telemetry.cpp now TRANSMITS — it was a pure byte-formatter and its "
        "encode_leg_setpoint call was safe only because of that")
    found.pop("telemetry.cpp", None)

    assert set(found) == set(_ALLOWED_MOTION_TUS), (
        "the set of firmware TUs that can command motion changed.\n"
        f"  found:   {sorted(found)}\n"
        f"  allowed: {sorted(_ALLOWED_MOTION_TUS)}\n"
        "A new producer is a new writer — argue it against the single-master "
        "invariant in this module's docstring before adding it here.")


def test_leg_interp_is_the_only_streamed_axis_6_producer(canbridge):
    """Exactly one line in the whole image streams a setpoint to the hand."""
    hits = {
        name: len(re.findall(r"encode_leg_setpoint\s*\(\s*HAND_AXIS", body))
        for name, body in canbridge.items()
        if re.search(r"encode_leg_setpoint\s*\(\s*HAND_AXIS", body)
    }
    hits.pop("telemetry.cpp", None)   # the echo formatter, excluded above
    assert hits == {"leg_interp.cpp": 1}, (
        "the streamed hand setpoint must have exactly ONE producer, the interp "
        f"tick in leg_interp.cpp. Found: {hits}")


def test_the_activate_park_is_the_only_other_axis_6_setpoint(canbridge):
    """ACTIVATE's park is a one-shot, reachable only through the NUM_AXES ladder.

    ``leg_activate.cpp`` never writes ``HAND_AXIS`` into a setpoint call
    literally — it widened its loops to ``NUM_AXES`` and picks the target through
    ``activate_target_rev(i)``.  That indirection is what keeps ONE data
    structure per concept (no hand-specific copy of the ladder), so the test
    pins the indirection rather than a spelling.
    """
    body = canbridge["leg_activate.cpp"]
    assert "activate_target_rev" in body, (
        "leg_activate.cpp no longer routes its per-axis target through "
        "activate_target_rev — a second copy of the hand's park position has "
        "appeared, or the hand has been dropped from ACTIVATE")
    assert "JBOp::HAND_ACTIVATE_POSITION_REV" in body, (
        "the hand's park target must be the generated constant, never a literal")
    assert re.search(
        r"encode_set_controller_mode\s*\(\s*\n?\s*HAND_AXIS,\s*ODriveControlMode::POSITION,"
        r"\s*\n?\s*ODriveInputMode::PASSTHROUGH", body), (
        "ACTIVATE must hand axis 6 to the streamed lane in POSITION/PASSTHROUGH; "
        "leaving it in TRAP_TRAJ is the FW 18 silently-inert-lane fault")


def test_deactivate_only_ever_idles_the_hand(canbridge):
    """DEACTIVATE de-energises axis 6; it never profiles a descent on it."""
    body = canbridge["leg_deactivate.cpp"]
    assert "idle_hand_now" in body, (
        "leg_deactivate.cpp no longer routes the hand through idle_hand_now — "
        "the hand may have been put into the legs' descent ladder")
    hand_frames = re.findall(r"(encode_\w+)\s*\(\s*HAND_AXIS", body)
    assert hand_frames == ["encode_set_state"], (
        "the ONLY axis-6 frame DEACTIVATE may send is the IDLE state change. "
        f"Found: {hand_frames}")


def test_the_coldstart_interlock_names_all_three_cold_start_ops(canbridge):
    """The serialisation mechanism itself, pinned.

    ACTIVATE and HOME are allowed to write axis 6 *only* because the streamed
    burst — legs and hand alike — is suppressed for the whole duration of those
    ops.  If the ``coldstart`` predicate ever drops one of them, or the hand TX
    escapes the gate it guards, two writers overlap.
    """
    body = canbridge["leg_interp.cpp"]
    m = re.search(r"const bool coldstart\s*=\s*([^;]+);", body)
    assert m, "leg_interp.cpp no longer defines the coldstart interlock predicate"
    pred = m.group(1)
    for op in ("homing_active()", "activate_active()", "deactivate_active()"):
        assert op in pred, f"the coldstart interlock no longer covers {op}"
    assert re.search(
        r"if\s*\(\s*s_output_enabled\s*&&\s*!coldstart\s*\)[\s\S]{0,3000}?"
        r"encode_leg_setpoint\s*\(\s*HAND_AXIS", body), (
        "the hand's streamed TX has escaped the `s_output_enabled && !coldstart` "
        "gate — it would then be the one producer that ignores the interlock")


def test_the_platform_teensy_commands_no_odrive_at_all(platform):
    """The Platform Teensy's stroke engine is gone and cannot come back quietly.

    It was the original hand master (``Trajectory.h`` + the 0x6D0 TRAJ_CMD
    decode + the 0x0C9 hand-encoder cache).  R1 retired it; what survives is the
    inclinometer, the time-sync slave and the 0x6E0 cold-start state — none of
    which touch an ODrive.
    """
    assert platform, "no Platform Teensy sources found"
    for name, body in platform.items():
        for enc in _MOTION_ENCODERS:
            assert enc not in body, f"{name} commands motion via {enc}"
        for dead in ("Trajectory.h", "0x6D0", "makeSmoothMove", "TRAJ_CMD"):
            assert dead not in body, (
                f"{name} still references {dead} in CODE — the Platform stroke "
                "engine is resurrecting")
