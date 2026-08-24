"""Pin the vendored FlexCAN_T4 TX-deferral path that the bridge now relies on.

WHAT CHANGED, AND WHY THIS FILE EXISTS NOW
------------------------------------------
Until 2026-08-24 the software TX-deferral path in ``lib/FlexCAN_T4`` was
**dormant by construction**: every bus ran ``tx_deferred == 0`` (the jugglebot bus
since FW 10 raised it to 16 TX mailboxes; bb and cone never deferred), so
``txBuffer`` was never entered and the drain branch in ``events()`` never ran.
FW 14's ``P4`` patch — the missing ``break`` in that branch — was landed
explicitly as *"disarming a mine"* rather than as a behavioural fix, on the
standing warning that **anything re-opening the deferral path must fix that loop
first** (``can_buses.cpp``'s ``setMaxMB`` block).

The tri-state TX accounting does not re-open the path — deferral was always
possible at the FlexCAN level and the send rate is unchanged — but it does move
deferral from "an outcome nobody models" to **an outcome every caller has an
explicit ruling for, and a counter for**.  A dormant guard that the firmware now
*designs around* is a guard that has to be pinned, because the next reader will
have no reason to suspect the vendored file is load-bearing.

WHAT IS PINNED
--------------
1.  ``FCTP_OPT::write(const CAN_message_t&)`` still returns ``1`` on the mailbox
    path and ``-1`` on the queue path, and **no path in that overload returns 0**.
    That is the whole basis of ``can_buses.h``'s ``classify_tx_write()``: if a
    future library bump made ``-1`` mean "dropped", every per-caller ruling in
    this change-set would silently invert.
2.  FW 14's ``P4`` ``break`` is still inside the ``frame.mb == -1`` refill loop.
    Without it one peeked deferred frame is written into **every** free TX
    mailbox — up to 16 duplicate transmissions on the jugglebot bus — while
    ``pop_front()`` runs the same number of times, discarding the next queued
    frames unsent.  Duplicate transmission AND silent loss, from one missing
    statement, on a path this change-set now expects to be exercised.
3.  ``PROVENANCE.md`` still records both patches, so a future vendor re-copy
    cannot quietly revert them without the diff being visible.

The *behavioural* half of the same contract — that ``classify_tx_write`` maps
those returns correctly and that every caller rules on ``DEFERRED`` — is executed
by ``tests/firmware/native/test_can_tx_result.cpp`` and the per-caller native
drivers.  This file is the source scan those binaries cannot be: they compile
``can_buses.h``, never the vendored ``.tpp``.

Pure stdlib.  No compiler, no ROS 2, no hardware.
"""

from __future__ import annotations

import os
import re


_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
_FW_DIR = os.path.join(_REPO_ROOT, 'ros_ws', 'src', 'jugglebot',
                       'Teensy_code_canbridge')
_LIB_DIR = os.path.join(_FW_DIR, 'lib', 'FlexCAN_T4')
_TPP = os.path.join(_LIB_DIR, 'FlexCAN_T4.tpp')
_PROVENANCE = os.path.join(_LIB_DIR, 'PROVENANCE.md')
_CAN_BUSES_H = os.path.join(_FW_DIR, 'can_buses.h')


def _read(path: str) -> str:
    with open(path, 'r', encoding='utf-8') as fh:
        return fh.read()


def _single_arg_write_body() -> str:
    """The body of ``FCTP_OPT::write(const CAN_message_t &msg)``.

    Selected by its signature rather than by line number: the mailbox-indexed
    overload ``write(FLEXCAN_MAILBOX mb_num, ...)`` sits immediately above it and
    DOES return 0, so a line-anchored or whole-file scan would happily "prove"
    the wrong function.
    """
    text = _read(_TPP)
    m = re.search(
        r'FCTP_FUNC\s+int\s+FCTP_OPT::write\s*\(\s*const\s+CAN_message_t\s*&\s*msg\s*\)\s*\{',
        text)
    assert m is not None, (
        'no FCTP_OPT::write(const CAN_message_t&) in the vendored FlexCAN_T4.tpp — '
        'the overload can_buses.cpp send_on() calls was renamed or removed, and '
        'every TxResult ruling in this firmware is derived from its return contract')
    # Brace-match from the opening brace so a later function cannot leak in.
    i = m.end() - 1
    depth = 0
    for j in range(i, len(text)):
        if text[j] == '{':
            depth += 1
        elif text[j] == '}':
            depth -= 1
            if depth == 0:
                return text[i:j + 1]
    raise AssertionError('unbalanced braces in FCTP_OPT::write body')


# ══════════════════════════════════════════════════════════════════════════════
#  1. The return contract classify_tx_write() is built on
# ══════════════════════════════════════════════════════════════════════════════

def test_write_returns_one_for_a_mailbox_and_minus_one_for_a_queued_frame():
    body = _single_arg_write_body()
    returns = set(re.findall(r'return\s+(-?\d+)\s*;', body))
    assert returns == {'1', '-1'}, (
        'FlexCAN_T4::write(const CAN_message_t&) now returns %s. can_buses.h\'s '
        'classify_tx_write() maps >0 to MAILBOX, <0 to DEFERRED and 0 to FAILED, '
        'and EVERY per-caller ruling in this firmware (poller arms AWAIT, leg '
        'setpoints count-not-retry, hand acks truthfully, safety retries only on a '
        'true failure) rests on -1 meaning QUEUED-AND-WILL-TRANSMIT. Re-derive the '
        'rulings before touching this.' % sorted(returns))


def test_the_queue_path_really_queues_before_returning_minus_one():
    """-1 must be reached only after the frame is pushed into ``txBuffer``.

    "Deferred" is a claim about the frame's future, not about the return value:
    if a future edit returned -1 *without* the ``struct2queueTx`` push, every
    caller here would ack a frame that no longer exists.
    """
    body = _single_arg_write_body()
    for m in re.finditer(r'return\s+-1\s*;', body):
        preceding = body[:m.start()]
        assert 'struct2queueTx' in preceding, (
            'a `return -1` in FlexCAN_T4::write is no longer preceded by the '
            'struct2queueTx push — -1 would then mean DROPPED, not QUEUED')


def test_can_buses_header_states_the_contract_it_depends_on():
    """The dependency is documented where the classifier lives, not only here."""
    h = _read(_CAN_BUSES_H)
    assert 'classify_tx_write' in h
    assert 'FlexCAN_T4.tpp' in h, (
        'can_buses.h no longer cites the vendored source its TX classifier '
        'derives from — the next reader loses the only pointer to why -1 is '
        'success')


# ══════════════════════════════════════════════════════════════════════════════
#  2. FW 14 P4: the deferral-drain `break` is still armed
# ══════════════════════════════════════════════════════════════════════════════

def test_the_mb_minus_one_refill_loop_still_has_its_break():
    """The ``mb == -1`` drain writes ONE peeked frame into ONE free mailbox.

    Without the ``break``, ``writeTxMailbox`` + ``pop_front`` run once per free
    mailbox on the SAME peeked frame: it is transmitted up to (MAXMB - 8) = 16
    times on the jugglebot bus while the next 15 queued frames are discarded
    unsent.  FW 14 fixed it while the path was dormant; this change-set gives
    every caller an explicit deferral ruling, so it is no longer dormant by
    assumption.
    """
    text = _read(_TPP)
    m = re.search(r'if\s*\(\s*frame\.mb\s*==\s*-1\s*\)\s*\{', text)
    assert m is not None, (
        'no `if (frame.mb == -1)` deferral-drain branch in events() — the '
        'vendored library was re-copied or restructured; re-verify P4 by hand')
    i = m.end() - 1
    depth = 0
    end = None
    for j in range(i, len(text)):
        if text[j] == '{':
            depth += 1
        elif text[j] == '}':
            depth -= 1
            if depth == 0:
                end = j + 1
                break
    assert end is not None, 'unbalanced braces in the mb == -1 branch'
    branch = text[i:end]
    assert 'writeTxMailbox' in branch and 'pop_front' in branch, branch
    assert re.search(r'^\s*break\s*;', branch, re.M), (
        'the FlexCAN_T4 events() `mb == -1` TX-deferral refill loop has lost its '
        '`break` (FW 14, PROVENANCE.md P4). One deferred frame is now written into '
        'EVERY free TX mailbox while pop_front discards the next queued frames '
        'unsent — duplicate transmission AND silent loss. This is no longer a '
        'dormant path: the 2026-08-24 tri-state accounting gives every caller an '
        'explicit DEFERRED ruling and counts deferrals per class.')


def test_the_rx_pop_is_still_inside_the_irq_mask():
    """FW 14 P3 — the ``_available`` leak fix, adjacent to the TX window.

    Not this change-set's work, but it lives three lines from the TX drain that
    is, and a re-vendor would take both out together.  The convicted failure was
    the RX ring degrading into an uptime-ratcheting delay line (leak_jb 247-248
    at 4.03 h, e2e leg lag 19.9 ms fresh vs 252.2 ms at 3.80 h).
    """
    text = _read(_TPP)
    m = re.search(r'NVIC_DISABLE_IRQ\(nvicIrq\);\s*'
                  r'asm volatile\("dsb\\n\\tisb"[^)]*\);\s*'
                  r'rxBuffer\.pop_front', text)
    assert m is not None, (
        'the RX-ring pop is no longer inside its NVIC_DISABLE_IRQ + dsb/isb window '
        '(FW 14, PROVENANCE.md P3) — the convicted `_available` one-directional '
        'leak is back, and with it the uptime-ratcheting delivery delay')


def test_provenance_records_both_tx_and_rx_patches():
    prov = _read(_PROVENANCE)
    for token in ('P3', 'P4', 'JUGGLEBOT PATCH', 'break'):
        assert token in prov, (
            'lib/FlexCAN_T4/PROVENANCE.md no longer records %r — a future vendor '
            're-copy would revert the patches with nothing to diff against' % token)
