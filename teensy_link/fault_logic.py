"""Pure-Python mirror of the can-bridge Teensy fault decision tree.

This module is a faithful transcription of the firmware's
``fault_machine.cpp`` — ``evaluate_errors()`` (the port of
``can_node.py:386-483 _handle_error``) and ``watchdog_and_stow()`` (the port of
``:1443-1530 _watchdog_check`` + the 2026-05-19 deferred-stow safety inversion)
— plus the bridge-side **link-loss** deferred-stow latch the Jetson owns.

Why this lives on the Jetson side too: the firmware OWNS the authoritative fault
machine and reports its verdict in ``HeartbeatT2J.fault_state``. These Python
mirrors exist so that

  * :class:`LinkLossLatch` can drive the bridge's own UDP-link watchdog with the
    *same* "never command a dead link; arm on loss; surface/stow only on
    confirmed reconnect" invariant the CAN-loss latch uses;
  * :class:`FaultEvaluator` / :class:`DeferredStowLatch` are a tested,
    byte-faithful port of the firmware decision tree — the canonical artifact
    for the ``can_node.py`` deletion (DONE 2026-07-06 — can_node
    references below point at the last pre-deletion revision in git history).
    Their fidelity to the
    firmware is pinned to a **firmware-anchored golden vector**
    (``tests/firmware/native/fault_golden.json``, emitted by the compiled
    ``fault_step()`` — the authoritative C++ spec is now
    ``tests/firmware/native/test_fault_machine.cpp``), replayed by
    ``tests/firmware/test_fault_logic.py``. Agreement chain (enforced by tests,
    not hand transcription): ``fault_machine.cpp`` ==(golden)== this module.

In the current design the bridge is an OBSERVER: :class:`FaultEvaluator` /
:class:`DeferredStowLatch` are NOT run live to override the Teensy's
authoritative ``fault_state`` (doing so would create a second, divergent fault
authority — the exact "two nearly-identical fault sites" drift the
contracts-over-patches philosophy warns against). The live mechanism is
:class:`LinkLossLatch` on the Jetson↔Teensy link.

References:
  * ``ros_ws/src/jugglebot/Teensy_code_canbridge/fault_machine.cpp``
  * ``ros_ws/src/jugglebot/jugglebot/can_node.py`` (_handle_error, _watchdog_check)
  * ``logbook/2026-05-19-can-loss-fault-response-safety-inversion.md``
  * ``tests/firmware/native/test_fault_machine.cpp`` (the compiled executable spec)
  * ``tests/firmware/test_fault_logic.py`` (the golden-conformance replay for this module)
"""

from __future__ import annotations

from typing import List

# ── Constants (must match odrive.py / fault_machine.cpp) ───────────────────
CLOSED_LOOP = 8                      # ODrive AXIS_STATES['CLOSED_LOOP']
IDLE = 1                             # ODrive AXIS_STATES['IDLE']
ERR_DC_BUS_UNDER_VOLTAGE = 512       # odrive.ERR_DC_BUS_UNDER_VOLTAGE
MAX_SOFT_RESET_ATTEMPTS = 1          # motors.max_soft_reset_attempts
NUM_LEGS = 6


class FaultEvaluator:
    """1:1 port of ``fault_machine.cpp::evaluate_errors`` (= can_node _handle_error).

    Stateful over the soft-reset budget (the bounce-loop guard): exactly one
    auto-clear of a disarmed-not-CLOSED_LOOP fault, then fatal until an explicit
    ``notify_clear_errors`` (the operator path) refills the budget.

    ``clear_calls`` counts how many times a CAN ``clear_errors`` broadcast would
    have gone out (test instrumentation). ``_clear_errors_can`` is a no-op while
    ``fatal_can_error`` is set — **never command a dead bus** — and the budget is
    then preserved (the CAN-down watchdog owns that case), matching the firmware
    fix noted in the firmware-WIP handoff adversarial review.
    """

    def __init__(self, num_legs: int = NUM_LEGS):
        self.n = num_legs
        self.active: List[int] = [0] * num_legs
        self.disarm: List[int] = [0] * num_legs
        self.state: List[int] = [IDLE] * num_legs
        self.fatal_error = False
        self.undervoltage_error = False
        self.fatal_can_error = False
        self.soft_reset_attempts = 0
        self.clear_calls = 0

    # mirror motors.clear_error_flags
    def _clear_error_flags(self) -> None:
        self.fatal_error = False
        self.undervoltage_error = False
        self.soft_reset_attempts = 0

    # mirror clear_errors_can: never command a dead bus; True iff it "sent".
    def _clear_errors_can(self) -> bool:
        if self.fatal_can_error:
            return False
        self.clear_calls += 1
        self._clear_error_flags()
        return True

    # mirror fault_notify_clear_errors (external CLEAR_ERRORS RPC / operator path)
    def notify_clear_errors(self) -> None:
        self._clear_error_flags()

    def evaluate_errors(self) -> None:
        no_active = all(a == 0 for a in self.active)
        no_disarm = all(d == 0 for d in self.disarm)
        any_disarmed = any(d != 0 for d in self.disarm)
        any_cl = any(s == CLOSED_LOOP for s in self.state)
        all_uv_only = all(d == 0 or d == ERR_DC_BUS_UNDER_VOLTAGE
                          for d in self.disarm)
        for a in self.active:
            if a & ERR_DC_BUS_UNDER_VOLTAGE:
                self.undervoltage_error = True

        # All axes fully clean — clear everything. NB: does NOT reset the
        # soft-reset budget (only an explicit clear does — bounce-loop guard).
        if no_active and no_disarm:
            self.fatal_error = False
            self.undervoltage_error = False
            return

        is_fatal = False
        if not no_active:
            is_fatal = True                          # active errors → fatal
        if any_disarmed and any_cl:
            is_fatal = True                          # disarm while CLOSED_LOOP → fatal

        if any_disarmed and not any_cl:              # disarmed, none CL → try one clear
            if self.soft_reset_attempts < MAX_SOFT_RESET_ATTEMPTS:
                # Only consume the budget if the clear actually went out (bus up).
                if self._clear_errors_can():
                    self.soft_reset_attempts += 1
            else:
                is_fatal = True                      # budget exhausted → fatal

        # Undervoltage recovery: every disarm is 0-or-UV and there WAS a disarm
        # ⇒ bus dropped + recovered; clear UV (+ auto-clear if nothing else fatal).
        if no_active and all_uv_only and any_disarmed:
            self.undervoltage_error = False
            if not is_fatal:
                self._clear_errors_can()
                return

        if is_fatal:
            self.fatal_error = True


class DeferredStowLatch:
    """1:1 port of ``fault_machine.cpp::watchdog_and_stow`` (the CAN-loss latch).

    The canonical state machine of the 2026-05-19 deferred-stow safety
    inversion: arm at loss DETECTION, never command the dead bus, execute the
    profiled stow only on CONFIRMED reconnect, re-arm if the stow half-completes
    (link re-drops mid-descent). ``step(stale, fresh)`` takes the loss/freshness
    of the monitored link; ``stow_complete`` is set by the executor when the
    descent finishes; ``output_allowed`` is the safety output gate.

    This models the executor-bearing latch (the firmware, which owns CAN +
    ``interp_begin_stow``). The Jetson bridge's link-loss usage — which has no
    stow executor — is :class:`LinkLossLatch`.

    Reboot-suppression latch: :meth:`notify_reboot_started` (the
    ``REBOOT_ODRIVES`` RPC) arms a bounded suppression of the CAN-loss *detection*
    so a deliberate ODrive-reboot silence is not read as a real loss. The clock is
    abstracted the same way the rest of this latch abstracts it — ``step`` takes the
    ``deadline_pass`` boolean (the firmware's ``now >= s_reboot_deadline_us``); the
    ``stale``/``fresh`` inputs stand in for ``any_leg_heartbeat_stale`` /
    ``all_present_legs_fresh``. Armed ONLY via ``notify_reboot_started`` /
    ``reboot_start``, so a spontaneous loss is unaffected (the inversion is intact).
    """

    def __init__(self):
        self.fatal = False           # link/bus confirmed down (fatal_can_error analog)
        self.stow_pending = False
        self.stowing = False
        self.first_seen = False       # firmware s_first_leg_hb_seen: set once any leg hb is seen
        self.stow_complete = False
        self.reboot_in_progress = False   # reboot-suppression latch flag
        self.reboot_saw_stale = False     # legs went stale since arming (release gate)

    def notify_reboot_started(self) -> None:
        """Arm the bounded watchdog-suppression latch (mirror
        ``fault_notify_reboot_started``). Resets ``reboot_saw_stale`` so a fresh
        reboot cannot inherit a stale ``saw`` and release its latch early."""
        self.reboot_in_progress = True
        self.reboot_saw_stale = False

    def step(self, stale: bool, fresh: bool,
             reboot_start: bool = False, deadline_pass: bool = False,
             seen: bool = True) -> None:
        # Mirror watchdog_and_stow: any leg heartbeat ever seen sets s_first_leg_hb_seen,
        # which AND-gates CAN-loss detection below (the cold-start no-arm gate). Default
        # True preserves every existing caller (which established the link before stepping).
        if seen:
            self.first_seen = True
        if reboot_start:
            self.notify_reboot_started()
        # Reboot-suppression latch release (mirror watchdog_and_stow):
        # release on fresh-AFTER-stale (the ODrives came back) OR the bounded deadline.
        if self.reboot_in_progress:
            if stale:
                self.reboot_saw_stale = True
            if (self.reboot_saw_stale and fresh) or deadline_pass:
                self.reboot_in_progress = False
        # Detection: link went stale → fatal + arm the deferred stow. ANDs
        # not-reboot-suppressed so a deliberate reboot's silence does not false-trip.
        if self.first_seen and stale and not self.fatal and not self.reboot_in_progress:
            self.fatal = True
            self.stow_pending = True            # canonical arm point
            if self.stowing:                    # re-dropped mid-descent → re-arm
                self.stowing = False
        # Confirmed reconnect → clear fatal, begin the stow.
        if self.fatal and fresh:
            self.fatal = False
            if self.stow_pending and not self.stowing:
                self.stowing = True
        # Completion → clear the latch.
        if self.stowing and self.stow_complete:
            self.stowing = False
            self.stow_pending = False

    def output_allowed(self) -> bool:
        # Never command a dead bus; the stow descent may command (bus confirmed up).
        if self.stowing:
            return True
        return not self.fatal


class LinkLossLatch:
    """Bridge-side deferred-stow latch for the UDP Jetson↔Teensy link.

    Mirrors the *arming/holding* half of the 2026-05-19 deferred-stow invariant
    for the link the bridge actually owns (the UDP link, NOT the CAN bus — that
    is the Teensy's). Structurally parallel to ``can_node._watchdog_check``:

      * Arm ``stow_pending`` at link-loss DETECTION (the canonical point — does
        not depend on any orchestrator round-trip).
      * ``command_allowed()`` is False while the link is down — **never command a
        dead link** (and you physically can't: frames won't be delivered).
      * On confirmed reconnect the latch stays ``stow_pending`` and the bridge
        SURFACES it (``link_status``) for the operator/orchestrator. The
        bridge does NOT auto-execute a stow: the Teensy firmware owns the
        profiled CAN-side stow (``interp_begin_stow``), and there is no stow RPC
        on the Jetson side until the firmware implements a per-leg motion RPC. ``acknowledge_stow()`` clears
        the latch once the operator/firmware has handled it.

    The execute-on-reconnect half (``DeferredStowLatch.stowing`` →
    ``stow_complete``) is intentionally absent here: the bridge has no executor.
    """

    def __init__(self):
        self.link_lost = False
        self.stow_pending = False
        self.first_heartbeat_seen = False

    def update(self, stale: bool, seen: bool) -> None:
        """Advance the latch from the link's freshness.

        Args:
            stale: True iff the last T→J heartbeat is older than the timeout.
            seen:  True iff at least one T→J heartbeat has ever been received.
        """
        if seen:
            self.first_heartbeat_seen = True
        if not self.first_heartbeat_seen:
            return  # never arm before the link is first established
        if stale and not self.link_lost:
            self.link_lost = True
            self.stow_pending = True          # canonical arm point
        elif not stale and self.link_lost:
            self.link_lost = False            # reconnect — stow_pending stays SET

    def command_allowed(self) -> bool:
        """False while the link is down — never command into a dead link."""
        return not self.link_lost

    def acknowledge_stow(self) -> None:
        """Clear the pending latch (operator/orchestrator/firmware handled it)."""
        self.stow_pending = False
