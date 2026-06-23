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


# ── Phase 11: present-axis scoping (fault_machine.cpp leg_present / *_present_*) ──
# A leg is "present" iff heartbeat_seen (latched-once). The firmware must never
# STREAM setpoints to, nor require fresh heartbeats from, an absent node. These
# mirror the four enforcement points 1:1 and pin the single-leg-rig behaviour.
CAN_HEARTBEAT_TIMEOUT_US = 2_000_000   # canbridge_config.h
MAX_DEVIATION_REV = 0.5                # canbridge_config.h


def all_present_legs_fresh(present, ages_us, timeout_us=CAN_HEARTBEAT_TIMEOUT_US):
    """Mirror of fault_machine.cpp all_present_legs_fresh(): a PRESENT leg that is
    stale fails; absent legs are skipped (vacuously true with zero present)."""
    return not any(present[i] and ages_us[i] > timeout_us for i in range(6))


def all_six_legs_fresh(present, ages_us, timeout_us=CAN_HEARTBEAT_TIMEOUT_US):
    """The PRE-Phase-11 form (required all six seen AND fresh) — kept only to
    assert the contrast: it dead-locks the reconnect on a subset-populated bus."""
    return all(present[i] and ages_us[i] <= timeout_us for i in range(6))


def any_leg_heartbeat_stale(present, ages_us, timeout_us=CAN_HEARTBEAT_TIMEOUT_US):
    """Mirror of any_leg_heartbeat_stale() — already present-scoped in firmware."""
    return any(present[i] and ages_us[i] > timeout_us for i in range(6))


def setpoint_tx_legs(output_enabled, present):
    """Mirror of the interp-ISR per-leg TX gate: emit iff output_enabled && present."""
    return [i for i in range(6) if output_enabled and present[i]]


def max_deviation_estop(present, base_pos, enc_pos, limit_rev=MAX_DEVIATION_REV):
    """Mirror of evaluate_guard's MAX_DEVIATION loop, scoped to present legs."""
    return any(present[i] and abs(base_pos[i] - enc_pos[i]) > limit_rev for i in range(6))


def test_present_freshness_single_leg_is_the_dead_lock_fix():
    """odrv0-only rig: a fresh axis 0 makes the bus 'fresh' even though legs 1-5
    never report. The pre-Phase-11 all-six form returns False here — that is the
    deferred-stow reconnect dead-lock this change removes."""
    present = [True] + [False] * 5
    ages = [0] * 6                                     # axis 0 just heartbeated
    assert all_present_legs_fresh(present, ages) is True
    assert all_six_legs_fresh(present, ages) is False  # the bug the fix removes


def test_present_freshness_single_leg_stale_fails():
    present = [True] + [False] * 5
    ages = [3_000_000] + [0] * 5                       # axis 0 stale
    assert all_present_legs_fresh(present, ages) is False
    assert any_leg_heartbeat_stale(present, ages) is True


def test_present_freshness_full_robot_unchanged():
    """No-op on the full robot: with all six present the new and old forms agree."""
    present = [True] * 6
    fresh = [0] * 6
    assert all_present_legs_fresh(present, fresh) is True
    assert all_six_legs_fresh(present, fresh) is True
    one_stale = [0, 0, 0, 3_000_000, 0, 0]
    assert all_present_legs_fresh(present, one_stale) is False   # a present stale leg still fails
    assert all_six_legs_fresh(present, one_stale) is False


def test_setpoint_tx_only_to_present_legs():
    assert setpoint_tx_legs(True, [True] + [False] * 5) == [0]   # single-leg rig
    assert setpoint_tx_legs(True, [True] * 6) == [0, 1, 2, 3, 4, 5]  # full robot (no-op)
    assert setpoint_tx_legs(False, [True] * 6) == []            # output gated off → nothing


def test_max_deviation_skips_absent_legs():
    """An absent leg reads enc=0; a nonzero u0 there (production sends 6 targets)
    must NOT trip the E-STOP. A present leg that genuinely diverges still trips."""
    present = [True] + [False] * 5
    base = [0.0, 3.0, 3.0, 3.0, 3.0, 3.0]              # legs 1-5 commanded far from enc=0
    enc = [0.0] * 6
    assert max_deviation_estop(present, base, enc) is False     # absent legs skipped
    base_bad = [0.9] + [0.0] * 5                       # present leg 0 diverges > 0.5
    assert max_deviation_estop(present, base_bad, enc) is True


def test_single_leg_stow_recovery_end_to_end():
    """Deferred-stow reconnect must fire with only odrv0 present — the U3 Phase-8
    CAN-loss fault-replay depends on it. Pre-Phase-11 this dead-locked."""
    present = [True] + [False] * 5
    s = StowMirror()
    # CAN3 loss: axis 0 heartbeat goes stale.
    ages = [3_000_000] + [0] * 5
    s.step(leg_hb_stale=any_leg_heartbeat_stale(present, ages),
           leg_hb_fresh=all_present_legs_fresh(present, ages))
    assert s.fatal_can_error and s.stow_pending and not s.output_allowed()
    # Confirmed reconnect: axis 0 heartbeat fresh again → stow EXECUTES.
    ages = [0] * 6
    s.step(leg_hb_stale=any_leg_heartbeat_stale(present, ages),
           leg_hb_fresh=all_present_legs_fresh(present, ages))
    assert not s.fatal_can_error and s.stowing
