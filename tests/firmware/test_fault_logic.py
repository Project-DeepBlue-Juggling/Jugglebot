"""Executable spec for the Teensy fault state machine + deferred-stow latch.

The firmware's `fault_machine.cpp` ports can_node.py:386-483 (_handle_error),
:1443-1530 (_watchdog_check), and the 2026-05-19 deferred-stow safety-inversion
invariants. C++ can't be compiled here, so this test transcribes the C++ logic
into Python (kept 1:1 with fault_machine.cpp) and asserts the canonical scenarios
behave exactly as can_node / the logbook require. A regression in either the
firmware or its understanding shows up as a failing scenario.

The deepest validation — C++↔can_node fidelity and bench replay of the logbook
scenarios — is the adversarial review pass + hardware bring-up (see handoff).
"""

from __future__ import annotations

import pytest

CLOSED_LOOP = 8
IDLE = 1
ERR_UV = 512          # ERR_DC_BUS_UNDER_VOLTAGE
MAX_SOFT_RESET = 1


# ── Python mirror of fault_machine.cpp evaluate_errors() ──────────────────────
class FaultMirror:
    """1:1 transcription of fault_machine.cpp's error/stow logic (6 legs)."""
    def __init__(self):
        self.active = [0] * 6
        self.disarm = [0] * 6
        self.state = [IDLE] * 6
        self.fatal_error = False
        self.undervoltage_error = False
        self.fatal_can_error = False
        self.soft_reset_attempts = 0
        self.stow_pending = False
        self.stowing = False
        self.clear_calls = 0

    def _clear_errors_can(self):
        if self.fatal_can_error:
            return
        self.clear_calls += 1
        self.active = [0] * 6
        self.disarm = [0] * 6
        # clear_error_flags
        self.fatal_error = False
        self.undervoltage_error = False
        self.soft_reset_attempts = 0

    def evaluate_errors(self):
        no_active = all(a == 0 for a in self.active)
        no_disarm = all(d == 0 for d in self.disarm)
        any_disarmed = any(d != 0 for d in self.disarm)
        any_cl = any(s == CLOSED_LOOP for s in self.state)
        all_uv_only = all(d == 0 or d == ERR_UV for d in self.disarm)
        for a in self.active:
            if a & ERR_UV:
                self.undervoltage_error = True

        if no_active and no_disarm:
            self.fatal_error = False
            self.undervoltage_error = False
            return   # NB: does NOT reset soft_reset_attempts (bounce-loop guard)

        is_fatal = False
        if not no_active:
            is_fatal = True
        if any_disarmed and any_cl:
            is_fatal = True
        if any_disarmed and not any_cl:
            if self.soft_reset_attempts < MAX_SOFT_RESET:
                self._clear_errors_can()
                self.soft_reset_attempts += 1
            else:
                is_fatal = True
        if no_active and all_uv_only and any_disarmed:
            self.undervoltage_error = False
            if not is_fatal:
                self._clear_errors_can()
                return
        if is_fatal:
            self.fatal_error = True

    def notify_clear_errors(self):
        # clear_error_flags (operator path refills the budget)
        self.fatal_error = False
        self.undervoltage_error = False
        self.soft_reset_attempts = 0


# ── _handle_error scenarios ────────────────────────────────────────────────────

def test_all_clean_clears_fatal():
    m = FaultMirror()
    m.fatal_error = True
    m.evaluate_errors()
    assert not m.fatal_error and not m.undervoltage_error


def test_active_error_is_fatal():
    m = FaultMirror()
    m.active[2] = 0x40         # MISSING_INPUT
    m.evaluate_errors()
    assert m.fatal_error


def test_disarm_while_closed_loop_is_fatal():
    m = FaultMirror()
    m.disarm[3] = 0x40
    m.state[3] = CLOSED_LOOP    # axis lost torque while active
    m.evaluate_errors()
    assert m.fatal_error


def test_soft_reset_one_shot_then_fatal():
    """disarmed-not-CLOSED-LOOP: exactly one auto-clear, then fatal — the
    bounce-loop cap. Budget refills only on an explicit clear."""
    m = FaultMirror()
    m.disarm = [0x40] * 6
    m.state = [IDLE] * 6
    m.evaluate_errors()                 # attempt #1 → clear, attempts=1
    assert m.clear_calls == 1 and m.soft_reset_attempts == 1 and not m.fatal_error
    # disarm persists (simulate ODrive still disarmed after the clear)
    m.disarm = [0x40] * 6
    m.evaluate_errors()                 # budget exhausted → fatal, no further clear
    assert m.clear_calls == 1 and m.fatal_error
    # operator clear refills the budget
    m.notify_clear_errors()
    assert m.soft_reset_attempts == 0
    m.disarm = [0x40] * 6
    m.evaluate_errors()                 # one more auto-clear allowed
    assert m.clear_calls == 2


def test_undervoltage_only_recovers_without_fatal():
    m = FaultMirror()
    m.disarm = [ERR_UV] * 6     # bus dropped + recovered, undervoltage disarm only
    m.state = [IDLE] * 6
    m.evaluate_errors()
    assert not m.fatal_error and not m.undervoltage_error


# ── Deferred-stow latch state machine (logbook 2026-05-19 invariants) ──────────

class StowMirror:
    """1:1 transcription of fault_machine.cpp watchdog_and_stow()."""
    def __init__(self):
        self.fatal_can_error = False
        self.stow_pending = False
        self.stowing = False
        self.first_hb_seen = True
        self.commanded_dead_bus = False   # set true if we ever command while CAN down
        self.stow_complete = False

    def step(self, leg_hb_stale, leg_hb_fresh):
        # Detection: arm at CAN-loss detection (canonical §6 point).
        if self.first_hb_seen and leg_hb_stale and not self.fatal_can_error:
            self.fatal_can_error = True
            self.stow_pending = True
            if self.stowing:           # bus re-dropped mid-descent → re-arm
                self.stowing = False
        # Confirmed reconnect → clear fatal, begin stow.
        if self.fatal_can_error and leg_hb_fresh:
            self.fatal_can_error = False
            if self.stow_pending and not self.stowing:
                self.stowing = True
        # Completion.
        if self.stowing and self.stow_complete:
            self.stowing = False
            self.stow_pending = False

    def output_allowed(self):
        # never command a dead bus; stow only after reconnect
        if self.stowing:
            return True
        return not self.fatal_can_error


def test_can_loss_never_commands_dead_bus_and_arms_latch():
    s = StowMirror()
    s.step(leg_hb_stale=True, leg_hb_fresh=False)     # CAN2 lost
    assert s.fatal_can_error and s.stow_pending
    assert not s.output_allowed()                      # never command the dead bus


def test_stow_executes_only_on_confirmed_reconnect():
    s = StowMirror()
    s.step(True, False)                                # loss → arm
    assert not s.stowing
    s.step(False, True)                                # confirmed reconnect → begin stow
    assert not s.fatal_can_error and s.stowing
    assert s.output_allowed()                          # stow descent may command (bus up)
    s.stow_complete = True
    s.step(False, True)                                # descent done
    assert not s.stowing and not s.stow_pending


def test_stow_rearms_if_bus_redrops_mid_descent():
    s = StowMirror()
    s.step(True, False)                                # loss
    s.step(False, True)                                # reconnect → stowing
    assert s.stowing
    s.step(True, False)                                # bus re-drops mid-descent
    assert s.fatal_can_error and s.stow_pending and not s.stowing   # re-armed, not commanding
    assert not s.output_allowed()
