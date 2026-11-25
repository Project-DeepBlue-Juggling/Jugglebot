"""Whether a dropped Leg provably kept its position reference (FW 53 U6b, R3; pure Python, no ROS).

The "mixed case": the Platform is up, one Leg's drive disarmed itself on an error
and the other five hold it. Owner decision 2026-11-25: "If the dropped Leg provably
kept its reference, then yes: re-enable it where it is and stow all six together.
Otherwise, we can keep with the current remedy of 'support the Platform and press the
E-stop'." This module decides "provably". Any doubt, and any input that is missing,
means NOT proven. The rows and their reasons: logbook
2026-11-26-fw53-u6b-mixed-case-recovery.md, "The predicate as built".

:class:`LegDropWatch` is fed every /robot_state and keeps, per Leg, the evidence from
the moment the Leg stops reading Closed-loop with no active error (a "drop") until it
reads that again. The disarm reason is what explains a drop and CLEAR_ERRORS erases
it, so the FAULT handler SEALS the open records when it sends the operator's
clear_errors: from then on a reason going back to 0 is the clear, not a drive reboot.

:func:`reference_unproven` is the predicate (None = proven, else the reason).
:func:`mixed_case_verdict` applies it to every Leg that is not Closed-loop.
"""
from __future__ import annotations

import math
from typing import List, NamedTuple, Optional, Sequence, Tuple

from jugglebot.can import odrive

ODRIVE_AXIS_IDLE = 1
ODRIVE_AXIS_CLOSED_LOOP = 8
NUM_LEGS = 6

#: Largest change of a dropped Leg's measured length, from its last Closed-loop
#: length, that still counts as sag rather than a frame shift. A re-seeded or
#: rebooted drive's frame is off by whole turns (the 2026-11-20 Hand jump was 5.00);
#: 0.25 rev (about 18 mm at the Legs' 70.5 mm/rev) keeps 0.75 rev of margin under
#: one turn. A Leg that sagged further is not proven (the E-stop remedy).
REF_DRIFT_TOL_REV = 0.25

#: ODrive error bits (odrive.ERROR_CODES) whose disarm keeps the drive's position
#: estimate: power stage, thermal, the CAN watchdog and a commanded estop. Reasoned
#: from the drive's structure (the estimator does not depend on them), not measured
#: here. Every other bit, decoded or not, is "loses or unknown" and refuses:
#: INITIALIZING, MISSING_ESTIMATE and CALIBRATION_ERROR lose it; SYSTEM_LEVEL,
#: TIMING_ERROR, BAD_CONFIG, DRV_FAULT, MISSING_INPUT, DC_BUS_UNDER_VOLTAGE (a
#: brown-out reboot can follow), VELOCITY_ and POSITION_LIMIT_VIOLATION (also the
#: signature of an estimate jump) and SPINOUT_DETECTED (a disagreement between the
#: commutation and the measured position) are unknown.
KEEPS_REFERENCE = (
    256         # DC_BUS_OVER_VOLTAGE
    | 1024      # DC_BUS_OVER_CURRENT
    | 2048      # DC_BUS_OVER_REGEN_CURRENT
    | 4096      # CURRENT_LIMIT_VIOLATION
    | 8192      # MOTOR_OVER_TEMP
    | 16384     # INVERTER_OVER_TEMP
    | 16777216  # WATCHDOG_TIMER_EXPIRED
    | 33554432  # ESTOP_REQUESTED
    | 134217728  # BRAKE_RESISTOR_DISARMED
    | 268435456  # THERMISTOR_DISCONNECTED
)


def _bit_names(bits: int) -> str:
    names = []
    b = 1
    while b <= bits:
        if bits & b:
            names.append(odrive.ERROR_CODES.get(b, f'bit {b}'))
        b <<= 1
    return ', '.join(names)


class LegDrop(NamedTuple):
    """The evidence for one Leg since it last read Closed-loop with no error."""
    last_cl_pos: Optional[float]  # its length then (None: never seen Closed-loop clean)
    bits: int                     # active_errors | disarm_reason, ORed over the drop
    reason_seen: bool             # a nonzero disarm_reason was reported
    reason_lapsed: bool           # it went back to 0 before the operator's clear
    odd_state: bool               # a state other than Idle or Closed-loop was reported
    gap: bool                     # a /robot_state gap beyond the freshness bound
    max_drift: float              # max |length - last_cl_pos| (inf: not finite/unknown)
    sealed: bool                  # the operator's clear_errors was sent during it


def reference_unproven(drop: Optional[LegDrop], *, is_homed: bool,
                       state: Optional[int], fresh: bool) -> Optional[str]:
    """THE predicate. None if this dropped Leg provably kept its reference, else
    the reason it is not proven (operator wording). ``state`` is the Leg's current
    ODrive axis state and ``fresh`` whether the /robot_state carrying it is within
    LEG_STATES_MAX_AGE_S."""
    if not is_homed:
        return ('the homed flag is clear (a drive reported MISSING_ESTIMATE or a '
                'reboot since the last Homing, or the robot was never homed)')
    if drop is None or drop.last_cl_pos is None:
        return ('this orchestrator never saw it Closed-loop with no error before it '
                'dropped, so there is no length to compare')
    if not fresh or state is None:
        return 'its drive state is not known from a fresh /robot_state'
    if state != ODRIVE_AXIS_IDLE:
        return f'it reads drive state {state}, not Idle'
    if drop.gap:
        return ('/robot_state paused during the drop, which could hide a drive '
                'reboot')
    if drop.odd_state:
        return ('it reported a drive state other than Idle or Closed-loop during '
                'the drop (a drive restart?)')
    if not drop.reason_seen:
        return ('its drive reported no disarm reason (an unexplained drop: a reboot '
                'or an outside IDLE)')
    if drop.reason_lapsed:
        return ('its disarm reason cleared before the operator\'s clear_errors (a '
                'drive reboot clears it)')
    bad = drop.bits & ~KEEPS_REFERENCE
    if bad:
        return (f'its error(s) {_bit_names(bad)} are not known to keep the '
                f'position estimate')
    if not (drop.max_drift <= REF_DRIFT_TOL_REV):   # NaN/inf fail
        drift = ('unknown' if not math.isfinite(drop.max_drift)
                 else f'{drop.max_drift:.2f} rev')
        return (f'its length moved {drift} from its last Closed-loop length '
                f'(limit {REF_DRIFT_TOL_REV:.2f} rev)')
    return None


class LegDropWatch:
    """Per-Leg drop evidence, fed from every /robot_state. Not thread-safe: the
    orchestrator calls it from its single executor thread."""

    def __init__(self) -> None:
        self._clean_pos: List[Optional[float]] = [None] * NUM_LEGS
        self._drop: List[Optional[LegDrop]] = [None] * NUM_LEGS

    def drop(self, leg: int) -> Optional[LegDrop]:
        """The open record for ``leg``, or None while it reads Closed-loop clean
        (or was never seen)."""
        return self._drop[leg]

    def observe(self, states: Sequence[int], active: Sequence[int],
                disarm: Sequence[int], pos: Sequence[float], gap: bool) -> None:
        """One /robot_state (Legs 0..5). ``gap``: the previous one is older than
        LEG_STATES_MAX_AGE_S."""
        for i in range(NUM_LEGS):
            st, act, dis = int(states[i]), int(active[i]), int(disarm[i])
            p = float(pos[i])
            if st == ODRIVE_AXIS_CLOSED_LOOP and act == 0:
                self._drop[i] = None
                self._clean_pos[i] = p if math.isfinite(p) else None
                continue
            d = self._drop[i]
            if d is None:
                d = LegDrop(self._clean_pos[i], 0, False, False, False, gap,
                            0.0, False)
            lapsed = d.reason_lapsed or (d.reason_seen and dis == 0
                                         and not d.sealed)
            if d.last_cl_pos is None or not math.isfinite(p):
                drift = math.inf
            else:
                drift = max(d.max_drift, abs(p - d.last_cl_pos))
            self._drop[i] = d._replace(
                bits=d.bits | act | dis,
                reason_seen=d.reason_seen or dis != 0,
                reason_lapsed=lapsed,
                odd_state=d.odd_state or st not in (ODRIVE_AXIS_IDLE,
                                                    ODRIVE_AXIS_CLOSED_LOOP),
                gap=d.gap or gap,
                max_drift=drift)

    def seal(self) -> None:
        """The operator's clear_errors is being sent: a disarm reason going back
        to 0 after this is the clear, not a reboot."""
        for i, d in enumerate(self._drop):
            if d is not None:
                self._drop[i] = d._replace(sealed=True)


def mixed_case_verdict(leg_states: Optional[Sequence[int]], fresh: bool,
                       is_homed: bool, watch: LegDropWatch
                       ) -> Tuple[bool, List[int], Optional[str]]:
    """``(mixed, dropped, refusal)``. ``mixed``: some Legs read Closed-loop and the
    rest do not (from a full, fresh state set). ``dropped``: the Legs not
    Closed-loop. ``refusal``: None when every dropped Leg is proven, else the
    operator text naming each one that is not and why."""
    if leg_states is None or len(leg_states) != NUM_LEGS or not fresh:
        return False, [], None
    closed = [i for i, s in enumerate(leg_states) if s == ODRIVE_AXIS_CLOSED_LOOP]
    dropped = [i for i in range(NUM_LEGS) if i not in closed]
    if not closed or not dropped:
        return False, dropped, None
    why = []
    for i in dropped:
        r = reference_unproven(watch.drop(i), is_homed=is_homed,
                               state=leg_states[i], fresh=fresh)
        if r is not None:
            why.append(f'Leg {i}: {r}')
    if not why:
        return True, dropped, None
    return True, dropped, (
        'Mixed-case recovery not offered: the position reference of the dropped '
        'Leg is not proven kept (' + '; '.join(why) + '). Re-energising it could '
        'drive it in a wrong frame, so the Platform stays held (ARMING_CONTRACT '
        'A6 mixed case).')
