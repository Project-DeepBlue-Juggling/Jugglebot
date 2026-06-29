"""Fidelity tests for controller/teensy_link/fault_logic.py.

Asserts the Jetson-side Python mirror reproduces every named transition in the
fault decision tree. The authoritative spec for the C++ is the **compiled**
firmware test ``tests/firmware/native/test_fault_machine.cpp`` (it drives the
real ``fault_step()``); ``tests/firmware/test_fault_logic.py`` then pins this
mirror to the firmware-anchored golden vector that binary emits. The agreement
chain is now enforced by tests rather than hand transcription:

    fault_machine.cpp  ==(native/fault_golden.json)==  fault_logic.py

A regression in the bridge's understanding of the fault decision tree, or a
drift from the firmware, shows up as a failing scenario here or in the golden
conformance.

Also covers :class:`LinkLossLatch` — the bridge's own UDP-link deferred-stow
latch (no firmware analog: it is the Jetson↔Teensy-link half of the invariant).
"""

from __future__ import annotations

from controller.teensy_link.fault_logic import (
    FaultEvaluator,
    DeferredStowLatch,
    LinkLossLatch,
    ERR_DC_BUS_UNDER_VOLTAGE,
    CLOSED_LOOP,
    IDLE,
)


# ── FaultEvaluator — error-eval / soft-reset / undervoltage scenarios ─────────

def test_all_clean_clears_fatal():
    m = FaultEvaluator()
    m.fatal_error = True
    m.evaluate_errors()
    assert not m.fatal_error and not m.undervoltage_error


def test_active_error_is_fatal():
    m = FaultEvaluator()
    m.active[2] = 0x40            # MISSING_INPUT
    m.evaluate_errors()
    assert m.fatal_error


def test_disarm_while_closed_loop_is_fatal():
    m = FaultEvaluator()
    m.disarm[3] = 0x40
    m.state[3] = CLOSED_LOOP      # axis lost torque while active
    m.evaluate_errors()
    assert m.fatal_error


def test_soft_reset_one_shot_then_fatal():
    """One auto-clear, then fatal — the bounce-loop cap. Budget refills only on
    an explicit clear."""
    m = FaultEvaluator()
    m.disarm = [0x40] * 6
    m.state = [IDLE] * 6
    m.evaluate_errors()                       # attempt #1 → clear, attempts=1
    assert m.clear_calls == 1 and m.soft_reset_attempts == 1 and not m.fatal_error
    m.disarm = [0x40] * 6                      # disarm persists after the clear
    m.evaluate_errors()                       # budget exhausted → fatal
    assert m.clear_calls == 1 and m.fatal_error
    m.notify_clear_errors()                   # operator clear refills the budget
    assert m.soft_reset_attempts == 0
    m.disarm = [0x40] * 6
    m.evaluate_errors()                       # one more auto-clear allowed
    assert m.clear_calls == 2


def test_undervoltage_only_recovers_without_fatal():
    m = FaultEvaluator()
    m.disarm = [ERR_DC_BUS_UNDER_VOLTAGE] * 6  # bus dropped + recovered (UV disarm only)
    m.state = [IDLE] * 6
    m.evaluate_errors()
    assert not m.fatal_error and not m.undervoltage_error


def test_active_undervoltage_sets_flag():
    """An active UV error (not just disarm) latches the undervoltage flag."""
    m = FaultEvaluator()
    m.active[0] = ERR_DC_BUS_UNDER_VOLTAGE
    m.evaluate_errors()
    assert m.undervoltage_error and m.fatal_error  # active error ⇒ also fatal


def test_soft_reset_not_consumed_on_dead_bus():
    """Firmware fix: a no-op clear (bus down) does NOT consume the soft-reset
    budget — the CAN-down watchdog owns that case."""
    m = FaultEvaluator()
    m.fatal_can_error = True       # CAN bus down ⇒ clear_errors_can is a no-op
    m.disarm = [0x40] * 6
    m.state = [IDLE] * 6
    m.evaluate_errors()
    assert m.clear_calls == 0 and m.soft_reset_attempts == 0


# ── DeferredStowLatch — deferred-stow latch scenarios ─────────────────────────

def test_can_loss_never_commands_dead_bus_and_arms_latch():
    s = DeferredStowLatch()
    s.step(stale=True, fresh=False)            # bus lost
    assert s.fatal and s.stow_pending
    assert not s.output_allowed()              # never command the dead bus


def test_stow_executes_only_on_confirmed_reconnect():
    s = DeferredStowLatch()
    s.step(True, False)                        # loss → arm
    assert not s.stowing
    s.step(False, True)                        # confirmed reconnect → begin stow
    assert not s.fatal and s.stowing
    assert s.output_allowed()                  # stow descent may command (bus up)
    s.stow_complete = True
    s.step(False, True)                        # descent done
    assert not s.stowing and not s.stow_pending


def test_stow_rearms_if_bus_redrops_mid_descent():
    s = DeferredStowLatch()
    s.step(True, False)                        # loss
    s.step(False, True)                        # reconnect → stowing
    assert s.stowing
    s.step(True, False)                        # bus re-drops mid-descent
    assert s.fatal and s.stow_pending and not s.stowing   # re-armed, not commanding
    assert not s.output_allowed()


# ── LinkLossLatch — the bridge's UDP-link deferred-stow latch ─────────────────

def test_link_latch_does_not_arm_before_first_heartbeat():
    latch = LinkLossLatch()
    # stale-but-never-seen (link not yet established) must NOT arm a stow.
    latch.update(stale=True, seen=False)
    assert not latch.link_lost and not latch.stow_pending
    assert latch.command_allowed()


def test_link_latch_arms_on_loss_and_blocks_commands():
    latch = LinkLossLatch()
    latch.update(stale=False, seen=True)       # link up
    assert latch.command_allowed()
    latch.update(stale=True, seen=True)        # link lost
    assert latch.link_lost and latch.stow_pending
    assert not latch.command_allowed()         # never command a dead link


def test_link_latch_stays_pending_on_reconnect():
    latch = LinkLossLatch()
    latch.update(stale=False, seen=True)
    latch.update(stale=True, seen=True)        # loss → armed
    latch.update(stale=False, seen=True)       # reconnect
    assert not latch.link_lost                 # link back up
    assert latch.stow_pending                  # but stow is STILL pending (surfaced)
    assert latch.command_allowed()


def test_link_latch_rearms_on_redrop():
    latch = LinkLossLatch()
    latch.update(stale=False, seen=True)
    latch.update(stale=True, seen=True)        # loss
    latch.update(stale=False, seen=True)       # reconnect (pending)
    latch.acknowledge_stow()                   # operator handled the stow
    assert not latch.stow_pending
    latch.update(stale=True, seen=True)        # re-drop
    assert latch.link_lost and latch.stow_pending   # re-armed
    assert not latch.command_allowed()


def test_link_latch_acknowledge_clears_pending():
    latch = LinkLossLatch()
    latch.update(stale=False, seen=True)
    latch.update(stale=True, seen=True)
    latch.update(stale=False, seen=True)
    assert latch.stow_pending
    latch.acknowledge_stow()
    assert not latch.stow_pending
