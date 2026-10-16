"""The streaming view of a FUTURE-origin install (I-INSTALL-1, 2026-10-16).

A fresh THROW/CATCH reserves its dispatch lead: ``install_segment`` plants its
origin at ``t0 = t_now + lead`` and the node swaps the PLANNING head at install,
up to a whole lead (13 knots, 0.325 s, for a THROW) before ``t0``. Until
2026-10-16 the emitter then sampled the new plan at ``tau < 0`` -- its knot 0,
the previous record's TERMINAL rest -- so a REST still streaming its tail was
cut short at install and the wire STEPPED onto its end state (measured 0.040 /
0.051 / 0.055 rev at 9 / 12 / 13 knots of lead on the opening lift, against the
0.06 rev install-continuity guard).

:class:`StreamHandoff` is the plan the wire plays instead: the running record
up to the head's origin, the head from it. It is plan-like in the HEAD's time
base (``tau = t - head_t0``) so the emitter, ``_current_state`` and
``_commanded_hand_state`` sample it exactly as they sample any plan:

* ``tau < 0``  -> ``source`` at ``tau + (head_t0 - source_t0)`` (the running
  record; past its own end a rest-terminal record holds its terminal rest,
  which IS the head's knot 0 -- so the switch carries no step by construction);
* ``tau >= 0`` -> ``head``.

The choice is made PER SAMPLE, so a frame straddling ``t0`` (the emitter ships
u0, u1, u2 at ``tau``, ``tau+dt``, ``tau+2dt``) carries the source's samples
before the origin and the head's from it -- the lookahead knots name what the
next frames will actually play.

Pure Python, no ROS. Nothing here decides WHEN a handoff exists; the node's
``_install`` does (see ``TrajectoryNode._stream_plan_locked``).
"""
from __future__ import annotations


class StreamHandoff:
    """``source`` (origin ``source_t0``) until ``head_t0``, then ``head``.

    ``source`` may itself be a :class:`StreamHandoff` (a splice into the head
    before its origin re-wraps the pending one), which is why every sample is
    delegated through the source's own plan interface rather than unwrapped.
    """

    __slots__ = ('source', 'source_t0', 'head', 'head_t0', '_offset',
                 'hand_at', 'hand_accel_at')

    def __init__(self, source, source_t0: float, head, head_t0: float):
        self.source = source
        self.source_t0 = float(source_t0)
        self.head = head
        self.head_t0 = float(head_t0)
        self._offset = self.head_t0 - self.source_t0
        # The hand lane follows the same switch. Present iff the HEAD carries a
        # hand track (the emitter's own `getattr(plan, 'hand_at', None)` test);
        # a source with no hand track (a HoldPlan) yields the head's pre-origin
        # answer (its knot 0) -- what the wire carried before this class existed.
        if getattr(head, 'hand_at', None) is not None:
            self.hand_at = self._hand_at
            self.hand_accel_at = self._hand_accel_at
        else:
            self.hand_at = None
            self.hand_accel_at = None

    # The head's identity for status/diagnostics readers.
    @property
    def kind(self):
        return getattr(self.head, 'kind', None)

    @property
    def total_duration(self) -> float:
        return float(self.head.total_duration)

    def state_at(self, tau: float):
        tau = float(tau)
        if tau < 0.0:
            return self.source.state_at(tau + self._offset)
        return self.head.state_at(tau)

    def _hand_at(self, tau: float):
        tau = float(tau)
        if tau < 0.0:
            src = getattr(self.source, 'hand_at', None)
            if src is not None:
                return src(tau + self._offset)
        return self.head.hand_at(tau)

    def _hand_accel_at(self, tau: float) -> float:
        tau = float(tau)
        if tau < 0.0 and getattr(self.source, 'hand_at', None) is not None:
            src = getattr(self.source, 'hand_accel_at', None)
            return float(src(tau + self._offset)) if src is not None else 0.0
        acc = getattr(self.head, 'hand_accel_at', None)
        return float(acc(tau)) if acc is not None else 0.0
