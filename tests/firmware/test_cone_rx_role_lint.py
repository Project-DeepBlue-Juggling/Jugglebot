"""Lint: the cone bus's TX presence gate stays ID-AGNOSTIC while health does not.

WHAT THIS GUARDS
----------------
The cone-role bus (physical CAN3) carries exactly one peripheral, but which one
is a physical choice: the catching cone (``0x7E0``/``0x7E1``) or the electronic
clapboard (``0x7E8``-``0x7EF``).  ``on_cone_rx()`` therefore keeps THREE
timestamps, and the whole safety of the arrangement is that they are stamped with
different rules:

* ``s_cone_last_rx_us`` — **unconditional**.  It is the sole input to
  ``partner_recent()``, i.e. the TX presence gate, and that gate only asks "is
  SOMEONE on this bus to ACK my frame".  Moving this write behind the ID
  discriminator closes the gate for whichever device is *not* the one the branch
  names, which stops the 100 Hz ``0x7DD`` time-sync broadcast, so the clapboard
  never anchors its wall clock, never emits a fire event and never leaves its
  screensaver — **with no error anywhere on either side**.  That is the one
  regression in this phase that would be genuinely hard to attribute, which is
  why it gets a structural pin rather than a comment.
* ``s_cone_only_last_rx_us`` / ``s_clap_last_rx_us`` — **discriminated**, and
  used only for health reporting.  A wire field that confidently names the wrong
  peripheral is worse than one that reports nothing, because it will be believed.

WHY A TEXT LINT
---------------
``can_buses.cpp`` compiles in no native test binary — it pulls the 7000-line
template-heavy vendored FlexCAN_T4 against i.MXRT register maps, and building a
host shim for it has been declined twice (``can_buses.h``, the ``BusRxHealth``
coverage-gap note).  So there is no compiled route to this function at all.  A
structural source assertion is what is available, and it is enough: the property
being pinned is *where in the control flow the write sits*, which is exactly what
a brace-depth walk can see.  ``test_rpc_dispatch_lint.py`` is the precedent for a
firmware text lint that needs no compiler.

Comments and string literals are stripped before the walk, so prose mentioning a
symbol (there is plenty) can neither satisfy nor break an assertion.

Pure stdlib.  No compile, no ROS 2, no hardware.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

_REPO_ROOT = Path(__file__).resolve().parents[2]
_CAN_BUSES_CPP = (_REPO_ROOT / "ros_ws" / "src" / "jugglebot"
                  / "Teensy_code_canbridge" / "can_buses.cpp")

_ON_CONE_RX_SIG = "static void on_cone_rx(const CAN_message_t& msg)"


def _strip_comments(src: str) -> str:
    """Blank out ``//`` comments, ``/* */`` comments and string literals.

    Newlines are preserved so a failure message can still quote sensible context.
    """
    out = []
    i, n = 0, len(src)
    while i < n:
        two = src[i:i + 2]
        if two == "//":
            j = src.find("\n", i)
            if j < 0:
                break
            out.append("\n")
            i = j + 1
        elif two == "/*":
            j = src.find("*/", i + 2)
            if j < 0:
                break
            out.append("\n" * src.count("\n", i, j))
            i = j + 2
        elif src[i] == '"':
            j = i + 1
            while j < n and src[j] != '"':
                j += 2 if src[j] == "\\" else 1
            out.append('""')
            i = j + 1
        else:
            out.append(src[i])
            i += 1
    return "".join(out)


@pytest.fixture(scope="module")
def code() -> str:
    return _strip_comments(_CAN_BUSES_CPP.read_text(encoding="utf-8"))


@pytest.fixture(scope="module")
def on_cone_rx_body(code: str) -> str:
    """The body of ``on_cone_rx`` (between its outermost braces)."""
    start = code.find(_ON_CONE_RX_SIG)
    assert start >= 0, (
        f"{_ON_CONE_RX_SIG!r} not found in can_buses.cpp — the cone RX callback "
        "was renamed or its signature changed.  This lint is the ONLY guard on "
        "the TX-gate stamp; re-point it rather than deleting it.")
    open_brace = code.index("{", start)
    depth = 0
    for k in range(open_brace, len(code)):
        if code[k] == "{":
            depth += 1
        elif code[k] == "}":
            depth -= 1
            if depth == 0:
                return code[open_brace + 1:k]
    raise AssertionError("unbalanced braces in on_cone_rx")


def _depth_at(body: str, index: int) -> int:
    """Brace nesting depth of ``index`` relative to the function body (0 = top)."""
    return body.count("{", 0, index) - body.count("}", 0, index)


def _sole_match(body: str, pattern: str) -> re.Match:
    matches = list(re.finditer(pattern, body))
    assert len(matches) == 1, (
        f"expected exactly one {pattern!r} in on_cone_rx, found {len(matches)}")
    return matches[0]


# ══════════════════════════════════════════════════════════════════════════════
#  The pin: the gate stamp is unconditional, the health stamps are not
# ══════════════════════════════════════════════════════════════════════════════

def test_tx_gate_stamp_is_written_unconditionally(on_cone_rx_body):
    """``s_cone_last_rx_us`` is written at on_cone_rx's TOP LEVEL, not in a branch.

    Fails the moment the write moves inside ``if (is_cone_id(...))`` or any other
    conditional — the silent-time-sync-death regression described in the module
    docstring.  Depth is measured by brace nesting, so wrapping the write in
    anything at all (an ``if``, a ``for``, a lambda, a scope block) trips it.
    """
    m = _sole_match(on_cone_rx_body, r"atomic_write_u64\(\s*&s_cone_last_rx_us\b")
    depth = _depth_at(on_cone_rx_body, m.start())
    assert depth == 0, (
        "the s_cone_last_rx_us write is nested "
        f"{depth} brace level(s) deep inside on_cone_rx.  It is the SOLE input to "
        "partner_recent() — the cone-bus TX presence gate — and must stay "
        "ID-agnostic so ANY partner on that bus keeps the 0x7DD time-sync "
        "broadcast flowing.  Gating it stops time-sync, so an attached clapboard "
        "never anchors its wall clock, never emits a fire event and never leaves "
        "its screensaver, with no error on either side.  Discriminate the HEALTH "
        "stamps (s_cone_only_last_rx_us / s_clap_last_rx_us); never this one.")


def test_the_role_discriminator_is_actually_wired(on_cone_rx_body):
    """Both role predicates are called in on_cone_rx.

    The complement of the test above: an unconditional gate stamp is only half
    the contract.  Without the discriminator, ``cone_health`` goes back to
    reporting a catching cone whenever a clapboard heartbeats.
    """
    assert "is_clapboard_id(" in on_cone_rx_body, (
        "on_cone_rx no longer calls is_clapboard_id() — cone_health cannot tell "
        "a clapboard from a catching cone without it")
    assert "is_cone_id(" in on_cone_rx_body, (
        "on_cone_rx no longer calls is_cone_id() — the cone health stamp would "
        "then be stamped by clapboard traffic, restoring the lie")


@pytest.mark.parametrize("stamp", ["s_clap_last_rx_us", "s_cone_only_last_rx_us"])
def test_health_stamps_are_written_inside_a_branch(on_cone_rx_body, stamp):
    """The two ROLE stamps are conditional — the mirror image of the gate stamp.

    A health stamp written unconditionally would report both devices present at
    once on a segment that can only ever hold one, which is a different flavour
    of the same lie.
    """
    m = _sole_match(on_cone_rx_body, rf"atomic_write_u64\(\s*&{stamp}\b")
    depth = _depth_at(on_cone_rx_body, m.start())
    assert depth >= 1, (
        f"{stamp} is written at on_cone_rx's top level, i.e. for EVERY frame on "
        "the bus.  It exists to answer WHICH peripheral is attached, so it must "
        "sit behind its role predicate.")


def test_the_tx_gate_reads_the_shared_stamp(code):
    """``can_cone_send`` gates on ``s_cone_last_rx_us`` and on nothing narrower.

    Pins the other end of the same contract: it would be equally fatal to leave
    the write unconditional but re-point the GATE at a role stamp.
    """
    assert re.search(r"partner_recent\(\s*&s_cone_last_rx_us\b", code), (
        "can_cone_send no longer gates on the shared, ID-agnostic cone stamp — "
        "the TX presence gate must ask 'is anyone here to ACK', not 'is the "
        "device I expected here'")
    for role_stamp in ("s_clap_last_rx_us", "s_cone_only_last_rx_us"):
        assert not re.search(rf"partner_recent\(\s*&{role_stamp}\b", code), (
            f"the cone TX presence gate reads {role_stamp} — a role-discriminated "
            "stamp.  That closes the gate for the other device and stops 0x7DD.")


def test_cone_health_reads_the_discriminated_cone_stamp(code):
    """``cone_health`` is derived from the cone-ONLY stamp.

    This is the work item itself: with a clapboard attached, ``bus3_health`` must
    read UNKNOWN ("no catching cone has ever been seen"), not OK.  The wire
    field's meaning is unchanged — it still names the catching cone — which is
    what lets existing consumers keep reading it.
    """
    assert re.search(
        r"cone_health\s*=\s*health_of\(\s*atomic_read_u64\(\s*&s_cone_only_last_rx_us\b",
        code), (
        "cone_health is no longer derived from s_cone_only_last_rx_us.  Deriving "
        "it from the shared stamp makes it report a catching cone whenever ANY "
        "device — including a clapboard — is on the bus.")


def test_clapboard_presence_is_reported(code):
    """``CanStats::clapboard_present`` is populated from the clapboard stamp.

    Without it, ``cone_health == UNKNOWN`` is indistinguishable from an empty
    bus, and the honest-health change would have removed information rather than
    corrected it.
    """
    assert re.search(r"clapboard_present\s*=", code), (
        "can_buses_stats() no longer sets CanStats::clapboard_present.  CanStats "
        "is built field-by-field from a DEFAULT-constructed local, so an unset "
        "field reads as uninitialised stack — and the heartbeat's bit 6 would "
        "then flicker at random.")
    assert "s_clap_last_rx_us" in code
