"""Hand feedback-starvation episode detector (diagnostic only).

Pure Python (no ROS, no I/O, no logging). The bridge node feeds it from the RX
thread and logs whatever lines it returns; it changes no command, limit, guard
or timing.

WHY (2026-11-06 Sitting, BUG-50): the Hand's encoder feedback reached the
can-bridge at about 10 Hz instead of 100 Hz for 0.6 s. The firmware's Hand lead
clamp (``lead_clamp_mask`` bit 6) held a Throw back against the stale position
(0.32 m instead of 1.00 m), and when fresh feedback returned the command
stepped about 1.2 rev ahead and the Hand drive tripped CURRENT_LIMIT_VIOLATION.
The launch log said nothing. Two episode kinds are reported, one line each at
its END:

* **Hand lead clamp engaged** (bit 6 of the 10 Hz heartbeat's mask).
* **Hand encoder feedback stale**: the host has no per-frame encoder age
  (``CacheDiag`` carries only a 1 Hz window maximum), so the age is measured
  the way the 2026-11-06 diagnosis measured it: how long the Hand's
  (pos, vel) pair stayed bit-identical in the 100 Hz Telemetry stream while
  the Hand was moving (|vel| > 1 rev/s; a moving Hand cannot report the same
  position twice). Time base is the bridge's own ``t_teensy_us``, so host
  jitter cannot fake a freeze. An episode is a run of consecutive frozen gaps
  each above the gate; three consecutive in-gate frames end it.

QUIET RULE (owner/orchestrator decision 2026-11-06): at the bare 30 ms gate the
2026-11-06 Sitting bag (842 s) gave 91 lines, almost all single 30-47 ms gaps
(benign jitter). A stale episode therefore prints only if it has >= 2 frozen
gaps, OR its maximum age >= 2 x HAND_FEEDBACK_FRESH_S, OR the Hand clamp bit
was set at any time during it. Shorter ones stay silent but are counted; the
count of silent gaps since the last printed line is appended to the next line
that prints. The lead-clamp line is always printed. Times in the lines are
host wall-clock epoch seconds (``time.time()``), the same clock as the launch
log's own timestamps.

Only while the setpoint stream is running (``streaming``): the Hand is then
being commanded, which is when stale feedback matters.
"""
from __future__ import annotations

from typing import List, Optional, Tuple

#: Hand feedback freshness gate, seconds. Mirror of
#: ``HAND_DEV_FRESH_US = 30000u`` in
#: ``ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h`` (the Hand
#: deviation guard's anchor-freshness gate). The firmware header is the only
#: source (no generated/config copy exists); tests/teensy_link/
#: test_hand_feedback_watch.py pins this value against the header text.
HAND_FEEDBACK_FRESH_S = 0.030

#: Hand speed (rev/s) above which an unchanged (pos, vel) pair means a frozen
#: encoder, not a Hand at rest (same threshold as the 2026-11-06 diagnosis).
MOVING_VEL_RPS = 1.0

#: ``lead_clamp_mask`` bit for the Hand (axis 6).
HAND_CLAMP_BIT = 1 << 6

#: An open stale episode ends after this many CONSECUTIVE in-gate frames, not
#: the first one: the 2026-11-06 fault freeze was six 83-94 ms gaps with a
#: short fresh gap between some of them, and must read as ONE episode.
FRESH_FRAMES_TO_CLOSE = 3

_U32 = 0xFFFFFFFF


class HandFeedbackWatch:
    """Edge-triggered episode detector. Methods return lines to log (WARN)."""

    _NONE: Tuple[str, ...] = ()

    def __init__(self, gate_s: float = HAND_FEEDBACK_FRESH_S) -> None:
        self._gate_us = int(round(gate_s * 1e6))
        self._gate_s = gate_s
        # encoder-freeze tracker
        self._last: Optional[Tuple[float, float]] = None
        self._t_change_us = 0            # unwrapped, relative accumulator
        self._w_change = 0.0             # wall time of the last change
        self._enc_open = False
        self._enc_start_w = 0.0
        self._enc_end_w = 0.0
        self._enc_max_us = 0
        self._enc_gaps = 0
        self._enc_fresh_run = 0
        self._enc_clamped = False
        self._silent_gaps = 0
        # lead-clamp tracker
        self._clamp_open = False
        self._clamp_start_w = 0.0
        self._clamp_frames = 0
        self._clamp_last_w = 0.0
        #: episode totals since construction (for tests / a future census row)
        self.enc_episodes = 0
        self.clamp_episodes = 0

    # ── encoder feedback ──────────────────────────────────────────────
    def on_telemetry(self, t_us: int, pos: float, vel: float,
                     wall_s: float, streaming: bool) -> List[str]:
        """Feed one 100 Hz Telemetry frame's Hand (pos, vel). Usually ``()``."""
        cur = (pos, vel)
        if not streaming:
            out = self._close_enc() if self._enc_open else self._NONE
            self._last = None
            return out
        last = self._last
        if last is None:
            self._last = cur
            self._t_change_us = t_us
            self._w_change = wall_s
            return self._NONE
        if cur == last:
            return self._NONE
        gap_us = (t_us - self._t_change_us) & _U32
        out = self._NONE
        if gap_us > self._gate_us and abs(last[1]) > MOVING_VEL_RPS:
            if not self._enc_open:
                self._enc_open = True
                self._enc_start_w = self._w_change
                self._enc_max_us = 0
                self._enc_gaps = 0
                self._enc_clamped = self._clamp_open
            self._enc_end_w = wall_s
            self._enc_fresh_run = 0
            self._enc_gaps += 1
            if gap_us > self._enc_max_us:
                self._enc_max_us = gap_us
        elif self._enc_open:
            self._enc_fresh_run += 1
            if self._enc_fresh_run >= FRESH_FRAMES_TO_CLOSE:
                out = self._close_enc()
        self._last = cur
        self._t_change_us = t_us
        self._w_change = wall_s
        return out

    def _close_enc(self) -> List[str]:
        self._enc_open = False
        self._enc_fresh_run = 0
        self.enc_episodes += 1
        dur = self._enc_end_w - self._enc_start_w
        loud = (self._enc_gaps >= 2 or self._enc_clamped
                or self._enc_max_us >= 2 * self._gate_us)
        if not loud:
            self._silent_gaps += self._enc_gaps
            return []
        silent, self._silent_gaps = self._silent_gaps, 0
        tail = (f' [{silent} shorter stale gap(s) silent since the last line]'
                if silent else '')
        return [
            f'Hand encoder feedback STALE for {dur:.2f} s from host-wall t={self._enc_start_w:.2f} '
            f'({self._enc_gaps} frozen gaps, max age {self._enc_max_us / 1e3:.0f} ms, '
            f'above the {self._gate_s * 1e3:.0f} ms firmware Hand freshness gate): '
            f'feedback reached the can-bridge late while the Hand was moving; '
            f'the firmware lead clamp may have held the Hand back, so a Throw '
            f'in this window is short and the step when feedback returns can '
            f'overdrive the Hand{tail}'
        ]

    # ── lead clamp ────────────────────────────────────────────────────
    def on_heartbeat(self, lead_clamp_mask: int, wall_s: float) -> List[str]:
        """Feed one 10 Hz heartbeat's ``lead_clamp_mask``. Usually ``()``."""
        if lead_clamp_mask & HAND_CLAMP_BIT:
            if not self._clamp_open:
                self._clamp_open = True
                self._clamp_start_w = wall_s
                self._clamp_frames = 0
            if self._enc_open:
                self._enc_clamped = True
            self._clamp_frames += 1
            self._clamp_last_w = wall_s
            return self._NONE
        if not self._clamp_open:
            return self._NONE
        self._clamp_open = False
        self.clamp_episodes += 1
        # Heartbeats are 10 Hz: the clamp was on for at least first..last set
        # sample and at most first set..first clear sample.
        dur = wall_s - self._clamp_start_w
        return [
            f'Hand lead clamp ENGAGED for up to {dur:.2f} s from host-wall t={self._clamp_start_w:.2f} '
            f'({self._clamp_frames} heartbeat(s) with bit 6 set): the firmware '
            f'held the Hand back against stale feedback; a Throw in this '
            f'window is short'
        ]
