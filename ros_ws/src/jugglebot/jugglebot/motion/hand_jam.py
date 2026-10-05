"""Hand-jam detector and recovery step machine (pure Python, no ROS).

THE INVARIANT
    **A stalled hand at the clamp under a descending command is relieved
    before anything else and never pushed through.**

Why this exists (2026-10-02, bag ``2026-10-02_18-11-14``): a ball caught
between the descending cup and the funnel ring stalled the hand at 2.87 rev
while the streamed REST command kept descending. The hand ODrive's position
loop plus integrator sat at the 50 A clamp (~57 N on the ball, ~150 W of
copper loss) for 2.05 s from contact. The guard's MAX_DEVIATION latch caught
it by 0.027 rev, and the latch did not relieve it: output-off froze the last
lead-clamped ``input_pos`` and the ODrive kept pushing. It ended only because
the ball gave way and was forced through. A ball that stopped the hand
0.9 mm lower would never have latched at all, so ``MAX_DEVIATION_HAND_REV``
is not a jam detector. This module is the jam detector, plus the choreography
that gets the hand off the ball without pushing it through.

Split of responsibility (one enforcement point each):
    * ``JamDetector.step`` is the ONLY place a jam is recognised (P1-P6 and
      the sustain, below). It is fed from the bridge's telemetry RX path,
      never from the ROS executor (``_park_hand`` blocks executor threads for
      seconds; a detector parked behind it is no detector).
    * ``JamRecovery`` is the ONLY place the recovery ORDER is decided. It
      emits intents (relief, converge-clear, disarm, move, restore) and
      consumes telemetry; it executes nothing. Its ordering rules are the
      invariant made concrete:
        1. relief (SET_VEL_CURR_LIMITS to ``relief_curr_a``) is the FIRST
           actuator-affecting intent, before any CLEAR_ERRORS or motion;
        2. no move is emitted until the relief AND the disarm have been
           acknowledged (the firmware refuses a single-axis op under an armed
           stream, and an unconfirmed disarm races it);
        3. every move after a stall is UP (away from the ring) until the
           hand has dwelt above the stall, and the downward move is watched
           by a stall monitor that answers a re-pinch with another raise,
           never a harder push; every raise starts FROM THE HAND — an anchor
           (a HAND_MOVE_TO to the measured position, once at rest) ends any
           move in flight at the hand and snaps the ODrive setpoint to it,
           because a raise that RETARGETS a stalled move is planned from a
           setpoint that ran on below the hand and pushes down first
           (2026-10-05);
        4. the shipped current limit is restored LAST, only after the hand
           has arrived at the park, and never on an UNRECOVERED path.

The predicate (all of them, sustained >= ``sustain_s``):
    P1 the lane was descending: ``vel_ff_cmd < -descend_vel_rps`` at some
       sample in the last ``descend_window_s``, or ``pos_cmd`` has dropped by
       more than ``descend_drop_rev`` from its maximum over that window, OR
       the hand itself was descending: ``vel_meas < -stall_vel_rps`` (NOT
       ``descend_vel_rps`` — see below) at some sample in that same window
       (2026-10-04: HAND_MOVE_TO is a TRAP_TRAJ planned once, inside the
       ODrive, and the ACTIVATE park never streams — the command cache gets
       a single write at the START of the move and the window forgets it
       after ``descend_window_s``, while the real descent can run for
       seconds. The encoder witnesses the whole thing, so this alternative
       is the only one of the three that stays true for the move's full
       duration. Bounded by ``stall_vel_rps`` rather than the commanded
       alternatives' ``descend_vel_rps`` because ``vel_meas`` is a real,
       noisy encoder-derived estimate (``vel_ff_cmd`` is a clean streamed
       command and has no such noise): a single -1.418 rev/s sample on an
       otherwise-still hand — the SAME noise floor ``stall_vel_rps``'s own
       1.5-not-1.0 choice already measured, below — opened a 0.3 s window
       and fired a second, spurious HAND_JAM on bag 2026-10-02_18-11-14 when
       bounded by the lower ``descend_vel_rps``. It still opens no new
       false-positive class: it is one of five ANDed predicates, and a
       catch, the pre-stroke dip, or a ball landing on the cup still answer
       to P2/P3/P4/the sustain exactly as before — see the failure-mode
       table);
    P2 the hand is stalled: ``|vel_meas| < stall_vel_rps``;
    P3 the command is below the hand: ``pos_cmd - pos_meas < -lag_rev``;
    P4 the current is at the clamp: ``|iq_meas| >= iq_frac x curr_limit_a``
       (the SHIPPED limit, so the relieved hand does not re-trigger);
    P5 ``pos_meas`` is inside ``band_rev`` (where a ring pinch is possible);
    P6 axis 6 is CLOSED_LOOP with ``active_errors == 0`` and its diagnostic
       is younger than ``max_age_s`` (2026-10-04: 1.5 s, not 0.05 — the
       can-bridge firmware (``Teensy_code_canbridge/telemetry.cpp``,
       ``diag_changed``) sends axis 6's DIAGNOSTIC on-change
       (``DIAG_IQ_THRESH_A`` 0.5 A, ``DIAG_TEMP_THRESH_C`` 1.0 C, or a
       bus-voltage change) or forced at ``DIAG_FORCE_PERIOD_US`` = 1 Hz,
       staggered per axis. In a steady stall at the clamp iq is constant, so
       the next frame is ~1 s away and a 0.05 s bound was true for <= 50 ms
       per second — the sustain could never complete (bags
       2026-10-04_19-52-08 and _20-10-26, both silent on the live node).
       1.5 s is one forced-refresh period plus the stagger and UDP jitter, so
       it still faults a bridge that stopped sending diagnostics altogether;
       it loses nothing on the state/error side, because ``axis_state`` and
       ``active_errors`` transitions are themselves on-change and prompt).
    P1-P4 + P6 without P5 is a ``HAND_STALL``: relief only, no automatic
    motion (out of the band there is no ring to raise away from).

Failure modes the design enumerates, and what answers each:
    * Ball pinched in the band (the 2026-10-02 event)        -> HAND_JAM: relief, clear,
      disarm, anchor, raise to the clearance height (``raise_to_rev``), dwell,
      lower with the stall monitor, restore.
    * A HAND_MOVE_TO in flight when a raise is due (the recovery's own
      stalled lower; the bench lower that provoked the pinch) -> the anchor:
      a retargeted raise would plan from the setpoint that kept descending
      while the hand was stalled, and the hand would be pushed down at the
      relief current until that setpoint climbed back past it (bag
      2026-10-05_10-30-08: 0.6 s on raise 1, which passed the tracking check
      only on the ball's 0.3 rev spring-back; the whole window on raise 2,
      "did not track"). The anchor ARRIVES on the measured position at once
      and its PASSTHROUGH hand-off puts the setpoint on the hand.
    * Silent stall below the latch threshold (no guard latch) -> fires all the same:
      the predicate does not consult the latch.
    * Normal descent or catch (lag <= 0.2 rev, moving)        -> P2, P3.
    * Settling at rest or park (still, small current)         -> P4, P5.
    * Top stop at 10.7 rev (stalled at the clamp, ascending)  -> P1, P3, P5.
    * HAND_MOVE_TO or ACTIVATE-commanded descent (a TRAP_TRAJ planned once,
      inside the ODrive — the command cache sees a single write, never a
      stream) -> P1's measured-descent alternative (``vel_meas``): the
      encoder witnesses the whole move, not just its start.
    * Diagnostic cadence drops to on-change/the 1 Hz forced refresh during a
      steady clamp stall (iq constant)                        -> P6's
      ``max_age_s`` (1.5 s) still passes every sample; only a bridge that
      stops sending diagnostics altogether now faults it.
    * String/spool snag or carriage bind (identical while descending) -> fires; the
      RAISE tracking check discriminates (a ball pinch is one-sided, the hand rises
      freely): no ``raise_track_tol_rev`` of rise within ``raise_track_window_s`` ->
      retarget the move to hold at the measured position, UNRECOVERED.
    * Encoder freeze or slip                                  -> P6 (ODrive errors /
      stale diagnostic); a frozen encoder under a raise also fails the tracking check,
      and the relief current bounds any runaway.
    * Hand ODrive fault (current 0 or IDLE)                   -> P4, P6.
    * Undervoltage (lag without the clamp)                    -> P4, P6.
    * A ball landing on the cup (stall < 100 ms)              -> the sustain.
    * Operator's fingers in the funnel                        -> not excluded; the same
      reaction is the right one (relief, raise; a re-pinch on the lower is <= 14 N).
    * Ball still pinched on the lower                         -> attempt 2: anchor,
      raise to the clearance height again, dwell, lower; a stall after the
      last attempt anchors, raises there once more and STAYS raised
      (UNRECOVERED): never push a ball through, never IDLE a raised hand onto it.
    * Firmware without HAND_MOVE_TO (ERR_UNKNOWN_METHOD)      -> relief-only,
      UNRECOVERED 'firmware has no HAND_MOVE_TO'; the relief stays applied.
    * Relief, clear, disarm or restore not acknowledged; hand telemetry lost or
      stale mid-move; an exception in the executor              -> UNRECOVERED with the
      current left at the relief value; the bridge keeps RECOVERING held until the
      operator's ``/recover`` (which resumes at the lower, step 5).
"""
from __future__ import annotations

import enum
import math
from collections import deque
from dataclasses import dataclass, fields
from typing import Deque, List, Optional, Sequence, Tuple

#: ODrive AxisState.CLOSED_LOOP_CONTROL.
AXIS_STATE_CLOSED_LOOP = 8

#: The carriage's gravity hold is <= 3.6 A (design §4 step 1); a relief below
#: this would let the hand sag onto the ball, so the floor is a little above.
RELIEF_CURR_MIN_A = 4.0
#: A "relief" above ~20 A is no longer relief (>= 23 N on the ball).
RELIEF_CURR_MAX_A = 20.0

#: Status strings the executor reports for a move that cannot run at all.
STATUS_UNKNOWN_METHOD = 'ERR_UNKNOWN_METHOD'
STATUS_NO_CLIENT_METHOD = 'NO_CLIENT_METHOD'


def _finite(*xs: float) -> bool:
    return all(isinstance(x, (int, float)) and math.isfinite(x) for x in xs)


@dataclass(frozen=True)
class JamConfig:
    """Detector + recovery parameters. The first block are the bridge's
    ``hand_jam.*`` ROS parameters; the second block are the design's fixed
    numbers (§3/§4), kept here so the tests and the bridge share one copy.

    ``stall_vel_rps`` is 1.5, not the design draft's 1.0: on the 2026-10-02
    event the measured velocity read -1.243 rev/s at T-0.148 s while the ball
    was compressing (pos moved 0.007 rev in that 63 ms), which reset a 1.0
    threshold's sustain and delayed the fire to T-0.023 s. 1.5 rev/s is still
    >10x below any descent the lane streams, and P3 (lag) is what separates a
    pinch from a moving catch anyway. P1's measured-descent alternative
    (2026-10-04) reuses this same bound for the identical reason — see its
    docstring above (a -1.418 rev/s sample on the SAME bag, found by the
    replay probe once that alternative existed).

    ``max_age_s`` is 1.5, not the design draft's 0.05 (2026-10-04, R5 sitting
    4): see P6's docstring above for the firmware DIAGNOSTIC cadence that
    makes 0.05 s unfireable on the live node.

    ``raise_to_rev`` (2026-10-05, R5 sitting 5) is an ABSOLUTE height, 5.0
    rev, not the design draft's relative lifts of (1.0, 2.5) rev above the
    stall. The owner watched the first honest bench recovery: a 1.0 rev lift
    from the stall (2.81 rev, the ball squashed at 50 A) put the cup 0.56 rev
    above where the ball sits once relieved (3.25 rev) and the ball could not
    pass under the hand; 5.0 rev (163 mm above the park) clears a ball resting
    on the ring. Every raise goes there; ``raise_min_rev`` is the floor on the
    lift for a stall near or above it, so the tracking check still measures
    something. ``raise_attempts`` is the number of raise-dwell-lower cycles
    before the final raise that stays up.

    ``anchor_*`` (same sitting): the ODrive plans a HAND_MOVE_TO from its OWN
    setpoint, and a move RETARGETED while the hand is stalled plans from a
    setpoint that kept descending past the hand (bag 2026-10-05_10-30-08: the
    hand sat pressed on the ball at -8.6 A for 0.6 s after the first raise was
    commanded and for the whole 0.3 s window after the second, "raise did not
    track"). An ANCHOR -- a HAND_MOVE_TO to the measured position, issued
    once the hand has been at rest for ``anchor_rest_s`` -- ARRIVES at once
    (the firmware's arrival test is on the measured position) and its
    PASSTHROUGH hand-off snaps the setpoint to the hand, so the push stops and
    the raise that follows starts a fresh move planned from where the hand is.
    ``anchor_wait_max_s`` bounds the wait for rest (a hand that will not settle
    is anchored where it is); ``anchor_attempts`` re-anchors once if the first
    does not confirm within ``hold_wait_s``.
    """
    enabled: bool = True
    band_rev: Tuple[float, float] = (1.0, 3.6)
    stall_vel_rps: float = 1.5
    lag_rev: float = 0.5
    iq_frac: float = 0.9
    sustain_s: float = 0.10
    relief_curr_a: float = 10.0
    raise_to_rev: float = 5.0
    dwell_s: float = 0.6
    lower_stall_s: float = 0.15
    raise_track_tol_rev: float = 0.1
    # ── fixed (design §3/§4), not ROS parameters ──
    descend_window_s: float = 0.3
    descend_vel_rps: float = 1.0
    descend_drop_rev: float = 0.05
    max_age_s: float = 1.5
    max_gap_s: float = 0.05
    raise_track_window_s: float = 0.3
    raise_min_rev: float = 1.0
    raise_attempts: int = 2
    anchor_rest_s: float = 0.15
    anchor_wait_max_s: float = 1.0
    anchor_attempts: int = 2
    lower_stall_vel_rps: float = 0.3
    lower_stall_margin_rev: float = 0.5
    lower_grace_s: float = 0.5
    park_rev: float = 0.0
    park_tol_rev: float = 0.1
    move_vel_rps: float = 2.5
    raise_max_rev: float = 10.0
    bb_wait_max_s: float = 5.0
    telem_max_age_s: float = 0.25
    hold_wait_s: float = 2.0
    relief_attempts: int = 2
    restore_attempts: int = 2

    #: The names exposed as bridge ROS parameters (``hand_jam.<name>``).
    ROS_PARAMS = ('enabled', 'band_rev', 'stall_vel_rps', 'lag_rev', 'iq_frac',
                  'sustain_s', 'relief_curr_a', 'raise_to_rev', 'dwell_s',
                  'lower_stall_s', 'raise_track_tol_rev')

    @classmethod
    def from_values(cls, **kw) -> 'JamConfig':
        """Build and VALIDATE a config from (possibly ROS-typed) values.
        Raises ValueError naming the first offending parameter."""
        known = {f.name for f in fields(cls)}
        out = {}
        for k, v in kw.items():
            if k not in known:
                raise ValueError(f'{k}: unknown hand_jam parameter')
            if k == 'band_rev':
                try:
                    v = tuple(float(x) for x in v)
                except (TypeError, ValueError):
                    raise ValueError(f'{k}: must be a list of two numbers, got {v!r}')
            elif k == 'enabled':
                if not isinstance(v, bool):
                    raise ValueError(f'enabled: must be a bool, got {v!r}')
            elif k in ('relief_attempts', 'restore_attempts', 'raise_attempts',
                       'anchor_attempts'):
                v = int(v)
            else:
                try:
                    v = float(v)
                except (TypeError, ValueError):
                    raise ValueError(f'{k}: must be a number, got {v!r}')
            out[k] = v
        cfg = cls(**out)
        cfg.validate()
        return cfg

    def validate(self) -> None:
        """Raise ValueError on any parameter outside its physical range."""
        def need(cond: bool, name: str, why: str) -> None:
            if not cond:
                raise ValueError(f'{name}: {why} (got {getattr(self, name)!r})')

        for name in ('stall_vel_rps', 'lag_rev', 'iq_frac', 'sustain_s',
                     'relief_curr_a', 'dwell_s', 'lower_stall_s',
                     'raise_track_tol_rev', 'descend_window_s', 'max_age_s',
                     'raise_track_window_s', 'move_vel_rps', 'raise_max_rev',
                     'raise_to_rev', 'raise_min_rev', 'anchor_rest_s',
                     'anchor_wait_max_s'):
            need(_finite(getattr(self, name)) and getattr(self, name) > 0,
                 name, 'must be finite and > 0')
        for name in ('raise_attempts', 'anchor_attempts'):
            need(isinstance(getattr(self, name), int) and getattr(self, name) >= 1,
                 name, 'must be an int >= 1')
        need(len(self.band_rev) == 2 and _finite(*self.band_rev), 'band_rev',
             'must be two finite numbers')
        lo, hi = self.band_rev
        # A re-pinch inside the band must stay visible to the lower's stall
        # monitor, which only counts above park + lower_stall_margin_rev.
        need(self.park_rev + self.lower_stall_margin_rev <= lo < hi <= self.raise_max_rev,
             'band_rev', f'must satisfy park+{self.lower_stall_margin_rev} <= lo < hi '
             f'<= {self.raise_max_rev}')
        # The clearance height must lift a stall anywhere in the band by at
        # least the floor, and must be reachable.
        need(hi + self.raise_min_rev <= self.raise_to_rev <= self.raise_max_rev,
             'raise_to_rev', f'must satisfy band hi + {self.raise_min_rev} <= raise_to_rev '
             f'<= {self.raise_max_rev} (a raise must clear a ball from anywhere in '
             f'the band)')
        need(self.raise_min_rev > self.raise_track_tol_rev, 'raise_min_rev',
             f'must exceed raise_track_tol_rev ({self.raise_track_tol_rev}) or the '
             f'tracking check cannot pass a minimal lift')
        need(self.iq_frac <= 1.0, 'iq_frac', 'must be <= 1 (a fraction of the clamp)')
        need(self.stall_vel_rps <= 5.0, 'stall_vel_rps',
             'must be <= 5 rev/s (above that a moving hand reads as stalled)')
        need(self.lag_rev <= 2.0, 'lag_rev',
             'must be <= the 2.0 rev lead clamp (the lag can never exceed it)')
        # P1 must still hold when the sustain completes, or a stall that
        # began at the end of the descent could never fire.
        need(self.sustain_s < self.descend_window_s, 'sustain_s',
             f'must be < the {self.descend_window_s} s descent window')
        need(RELIEF_CURR_MIN_A <= self.relief_curr_a <= RELIEF_CURR_MAX_A,
             'relief_curr_a', f'must be in [{RELIEF_CURR_MIN_A}, {RELIEF_CURR_MAX_A}] A '
             '(above the carriage gravity hold, well below the 50 A clamp)')
        need(self.dwell_s >= 0.2, 'dwell_s', 'must be >= 0.2 s (a 74 mm free fall '
             'is 0.12 s plus rolling off the lip)')
        need(self.lower_stall_s <= 1.0, 'lower_stall_s', 'must be <= 1 s')
        need(self.raise_track_tol_rev <= 1.0, 'raise_track_tol_rev', 'must be <= 1 rev')
        need(self.max_age_s <= 5.0, 'max_age_s',
             'must be <= 5.0 s (above that a dead diagnostic feed should fault via '
             "P6's state/error checks, not an unbounded age)")
        need(0 < self.move_vel_rps <= 2.5, 'move_vel_rps',
             'must be in (0, 2.5] rev/s (the HAND_MOVE_TO cruise bound)')


# ═══════════════════════════════ detector ═══════════════════════════════════

class Verdict(enum.Enum):
    NONE = 'NONE'
    HAND_JAM = 'HAND_JAM'
    HAND_STALL = 'HAND_STALL'


@dataclass(frozen=True)
class HandSample:
    """One telemetry sample of axis 6. ``t`` is monotonic seconds;
    ``age_s`` is the age of the diagnostic that carried iq/state/errors;
    ``curr_limit_a`` is the SHIPPED current limit P4 is a fraction of."""
    t: float
    pos_cmd: float
    vel_ff_cmd: float
    pos_meas: float
    vel_meas: float
    iq_meas: float
    axis_state: int
    active_errors: int
    age_s: float
    curr_limit_a: float


@dataclass(frozen=True)
class Predicates:
    p1: bool = False
    p2: bool = False
    p3: bool = False
    p4: bool = False
    p5: bool = False
    p6: bool = False

    @property
    def stall(self) -> bool:
        return self.p1 and self.p2 and self.p3 and self.p4 and self.p6

    @property
    def jam(self) -> bool:
        return self.stall and self.p5

    def text(self) -> str:
        return ' '.join(f'P{i}={"T" if getattr(self, f"p{i}") else "f"}'
                        for i in range(1, 7))


class JamDetector:
    """P1-P6 + the sustain, stepped once per telemetry sample. Fires ONCE
    (the returned verdict is non-NONE on exactly the firing sample) and then
    stays latched (``fired``) until ``reset()``: one jam is one recovery."""

    def __init__(self, cfg: JamConfig):
        self.cfg = cfg
        self.reset()

    def reset(self) -> None:
        self.fired: Verdict = Verdict.NONE
        self.fire_sample: Optional[HandSample] = None
        self.last: Predicates = Predicates()
        self.last_sample: Optional[HandSample] = None
        self._prev_t: Optional[float] = None
        self._last_desc_t = -math.inf
        self._last_meas_desc_t = -math.inf
        # Monotone deque of (t, pos_cmd): the window maximum is at the head.
        self._cmd_max: Deque[Tuple[float, float]] = deque()
        self._stall_since: Optional[float] = None
        self._jam_since: Optional[float] = None

    @property
    def held_s(self) -> float:
        if self._stall_since is None or self.last_sample is None:
            return 0.0
        return self.last_sample.t - self._stall_since

    def _window(self, s: HandSample) -> Tuple[float, float, Deque[Tuple[float, float]]]:
        """P1's window state INCLUDING ``s``, without mutating the detector."""
        last_desc = self._last_desc_t
        if _finite(s.vel_ff_cmd) and s.vel_ff_cmd < -self.cfg.descend_vel_rps:
            last_desc = s.t
        # The MEASURED-descent alternative, tracked next to the commanded one:
        # a HAND_MOVE_TO/ACTIVATE target is a single write the window forgets
        # after descend_window_s, but the encoder keeps witnessing the move
        # for as long as it actually runs.
        # Bounded by stall_vel_rps, NOT descend_vel_rps (2026-10-04, found by
        # the replay probe on bag 2026-10-02_18-11-14 at T+1.401 s: a single
        # vel_meas sample of -1.418 rev/s — inside [descend_vel_rps,
        # stall_vel_rps) = [1.0, 1.5) — on an otherwise-still hand opened a
        # 0.3 s false-positive window and fired a second HAND_JAM 1.4 s after
        # the real one. That is the SAME noise floor JamConfig's docstring
        # already measured for P2 (stall_vel_rps is 1.5, not 1.0, because a
        # -1.243 rev/s creep sample reset a 1.0 threshold's sustain on this
        # same bag) — vel_ff_cmd is a clean streamed command with no such
        # noise, so descend_vel_rps stays correct for the two commanded
        # alternatives, but vel_meas is a real encoder-derived estimate and
        # must clear the same floor P2 does before it counts as "moving".
        last_meas_desc = self._last_meas_desc_t
        if _finite(s.vel_meas) and s.vel_meas < -self.cfg.stall_vel_rps:
            last_meas_desc = s.t
        dq = deque(self._cmd_max)
        if _finite(s.pos_cmd):
            while dq and dq[-1][1] <= s.pos_cmd:
                dq.pop()
            dq.append((s.t, s.pos_cmd))
        while dq and dq[0][0] < s.t - self.cfg.descend_window_s:
            dq.popleft()
        return last_desc, last_meas_desc, dq

    def _evaluate(self, s: HandSample, last_desc: float, last_meas_desc: float,
                  dq: Deque[Tuple[float, float]]) -> Predicates:
        c = self.cfg
        if not _finite(s.pos_cmd, s.pos_meas, s.vel_meas, s.iq_meas, s.age_s,
                       s.curr_limit_a):
            return Predicates()   # non-finite telemetry is never a jam (P6's job)
        drop = (dq[0][1] - s.pos_cmd) if dq else 0.0
        p1 = ((s.t - last_desc) <= c.descend_window_s or drop > c.descend_drop_rev
              or (s.t - last_meas_desc) <= c.descend_window_s)
        p2 = abs(s.vel_meas) < c.stall_vel_rps
        p3 = (s.pos_cmd - s.pos_meas) < -c.lag_rev
        p4 = abs(s.iq_meas) >= c.iq_frac * s.curr_limit_a
        p5 = c.band_rev[0] <= s.pos_meas <= c.band_rev[1]
        p6 = (int(s.axis_state) == AXIS_STATE_CLOSED_LOOP
              and int(s.active_errors) == 0 and 0.0 <= s.age_s < c.max_age_s)
        return Predicates(p1, p2, p3, p4, p5, p6)

    def peek(self, s: HandSample) -> Predicates:
        """The predicates ``s`` WOULD produce, without stepping (dry run)."""
        return self._evaluate(s, *self._window(s))

    def step(self, s: HandSample) -> Verdict:
        last_desc, last_meas_desc, dq = self._window(s)
        preds = self._evaluate(s, last_desc, last_meas_desc, dq)
        gap = (self._prev_t is not None
               and not (0.0 <= s.t - self._prev_t <= self.cfg.max_gap_s))
        self._last_desc_t, self._last_meas_desc_t = last_desc, last_meas_desc
        self._cmd_max, self._prev_t = dq, s.t
        self.last, self.last_sample = preds, s
        if self.fired is not Verdict.NONE:
            return Verdict.NONE
        if gap:
            # A telemetry gap breaks the "sustained" claim: restart it here.
            self._stall_since = self._jam_since = None
        if not preds.stall:
            self._stall_since = self._jam_since = None
            return Verdict.NONE
        if self._stall_since is None:
            self._stall_since = s.t
        if preds.p5:
            if self._jam_since is None:
                self._jam_since = s.t
        else:
            self._jam_since = None
        sustain = self.cfg.sustain_s - 1e-9
        if self._jam_since is not None and s.t - self._jam_since >= sustain:
            verdict = Verdict.HAND_JAM
        elif not preds.p5 and s.t - self._stall_since >= sustain:
            verdict = Verdict.HAND_STALL
        else:
            return Verdict.NONE
        self.fired, self.fire_sample = verdict, s
        return verdict


# ═══════════════════════════════ recovery ═══════════════════════════════════

class Outcome(enum.Enum):
    RUNNING = 'RUNNING'
    HAND_JAM_RECOVERED = 'HAND_JAM_RECOVERED'
    HAND_JAM_UNRECOVERED = 'HAND_JAM_UNRECOVERED'
    HAND_STALL_RELIEVED = 'HAND_STALL_RELIEVED'


class IntentKind(enum.Enum):
    HOLD_RECOVERING = 'HOLD_RECOVERING'   # step 0: hold fault_state=RECOVERING
    SET_CURRENT = 'SET_CURRENT'           # steps 1 and 6: SET_VEL_CURR_LIMITS(6, ...)
    CONVERGE_CLEAR = 'CONVERGE_CLEAR'     # step 2: the converge-first clear
    DISARM = 'DISARM'                     # step 2: disarm and CONFIRM on the wire
    MOVE = 'MOVE'                         # steps 3/5: HAND_MOVE_TO (asynchronous)
    WAIT = 'WAIT'                         # poll again
    END = 'END'


class Step(enum.Enum):
    TRIGGER = 0
    RELIEF = 1
    CLEAR = 2
    DISARM = 21
    ANCHOR = 22        # wait for rest, then HAND_MOVE_TO the measured position
    ANCHORING = 23     # the anchor in flight: ARRIVED snaps the setpoint to the hand
    RAISE = 3
    RAISING = 31
    DWELL = 4
    BB_GATE = 41
    LOWER = 5
    LOWERING = 51
    FINAL_RAISING = 52
    HOLDING = 53
    RESTORE = 6
    DONE = 99


@dataclass(frozen=True)
class Intent:
    kind: IntentKind
    curr_a: float = math.nan
    target_rev: float = math.nan
    vel_rps: float = math.nan
    purpose: str = ''
    outcome: Optional[Outcome] = None
    reason: str = ''


@dataclass(frozen=True)
class IntentResult:
    ok: bool
    status: str = 'OK'
    msg: str = ''


@dataclass(frozen=True)
class RecoveryObs:
    """What the machine reads each poll. ``move_*`` describe the LATEST move
    the executor started (``move_done`` False while it is in flight)."""
    t: float
    pos: float
    vel: float
    iq: float = 0.0
    telem_age_s: float = 0.0
    latched: bool = False
    armed: bool = False
    bb_pending: bool = False
    move_done: bool = False
    move_ok: bool = False
    move_status: str = ''
    move_msg: str = ''


_SYNC = (IntentKind.HOLD_RECOVERING, IntentKind.SET_CURRENT,
         IntentKind.CONVERGE_CLEAR, IntentKind.DISARM, IntentKind.MOVE)
_MONITORED = (Step.ANCHOR, Step.ANCHORING, Step.RAISING, Step.LOWERING,
              Step.FINAL_RAISING, Step.HOLDING)


class JamRecovery:
    """The §4 HAND_JAM sequence as an explicit step machine.

    Protocol: ``intent = rec.next(obs)``; for every intent kind in ``_SYNC``
    the executor carries it out and calls ``rec.result(IntentResult)`` before
    the next ``next()`` (a MOVE's result is its START acknowledgement; its
    completion arrives later through ``obs.move_*``). ``WAIT`` means poll
    again; ``END`` is terminal and repeats. ``abort(reason)`` is the
    executor's exit for an exception: UNRECOVERED, nothing restored.

    ``resume=True`` is the operator's ``/recover`` after an UNRECOVERED or
    HAND_STALL terminal: hold, relief (idempotent), clear-if-latched, disarm,
    then straight to the Ball-Butler gate and the lower (step 5). A stall on
    a resumed lower is final (anchor, raise to the clearance height and stay).

    Every raise is preceded by an ANCHOR (``Step.ANCHOR`` / ``ANCHORING``): a
    HAND_MOVE_TO to the measured position once the hand has rested
    ``anchor_rest_s``. It ends whatever HAND_MOVE_TO is in flight (the
    recovery's own stalled lower; the bench's) AT THE HAND -- the firmware
    answers ARRIVED on the measured position and hands the axis to
    PASSTHROUGH with the setpoint on the hand -- so the raise is a fresh move
    planned from where the hand is. See ``JamConfig``'s ``anchor_*`` note for
    the bag that showed a retargeted raise pushing down instead.
    """

    def __init__(self, cfg: JamConfig, *, kind: Verdict, stall_pos: float,
                 restore_curr_a: float, iq_at_trigger: float = math.nan,
                 resume: bool = False):
        if kind is Verdict.NONE:
            raise ValueError('JamRecovery needs a fired verdict')
        if not (_finite(restore_curr_a) and restore_curr_a > 0):
            raise ValueError(f'restore_curr_a must be finite and > 0, got {restore_curr_a!r}')
        self.cfg = cfg
        self.kind = kind
        self.resume = resume
        self.stall_pos = float(stall_pos)
        self.iq_at_trigger = float(iq_at_trigger)
        self.restore_curr_a = float(restore_curr_a)
        # Relief is never ABOVE the shipped limit (a low-current configure
        # must not be "relieved" upward).
        self.relief_curr_a = min(float(cfg.relief_curr_a), self.restore_curr_a)
        self.step = Step.TRIGGER
        self.outcome = Outcome.RUNNING
        self.reason = ''
        # Raise-dwell-lower cycles used so far; at ``raise_attempts`` the next
        # raise is the final one that stays up. A resume starts there.
        self.attempt = cfg.raise_attempts if resume else 0
        self.raises: List[float] = []
        self.lower_stalls = 0
        self.anchors = 0
        self.history: List[str] = []
        self._pending: Optional[Intent] = None
        self._tries = 0
        self._t0: Optional[float] = None
        self._move_t0 = 0.0
        self._move_p0 = 0.0
        self._tracked = False
        self._dwell_t0 = 0.0
        self._gate_t0 = 0.0
        self._anchor_t0: Optional[float] = None
        self._anchor_tries = 0
        self._rest_since: Optional[float] = None
        self._moving_seen = False
        self._still_since: Optional[float] = None
        self._hold_reason = ''

    # ── public ──
    @property
    def done(self) -> bool:
        return self.outcome is not Outcome.RUNNING

    @property
    def holds_recovering(self) -> bool:
        """Whether the bridge must keep fault_state=RECOVERING after this run:
        every terminal except RECOVERED (the operator acknowledges via
        ``/recover``)."""
        return self.outcome is not Outcome.HAND_JAM_RECOVERED

    def raise_target(self, from_pos: float) -> float:
        """The clearance height, lifted at least ``raise_min_rev`` above
        ``from_pos`` (the anchored hand) and never above ``raise_max_rev``."""
        c = self.cfg
        return min(max(c.raise_to_rev, float(from_pos) + c.raise_min_rev), c.raise_max_rev)

    def abort(self, reason: str) -> None:
        if not self.done:
            self._end(Outcome.HAND_JAM_UNRECOVERED, reason)

    def summary(self) -> str:
        raises = ','.join(f'{r:+.2f}' for r in self.raises) or 'none'
        s = (f'{self.outcome.value} ({self.kind.value}{", resumed" if self.resume else ""}): '
             f'stall at {self.stall_pos:+.3f} rev, iq {self.iq_at_trigger:+.1f} A, '
             f'raises [{raises}] rev, anchors {self.anchors}, lower stalls {self.lower_stalls}')
        return f'{s} — {self.reason}' if self.reason else s

    def result(self, res: IntentResult) -> None:
        it = self._pending
        if it is None:
            raise RuntimeError('result() without a pending intent')
        self._pending = None
        self._on_result(it, res)

    def next(self, obs: RecoveryObs) -> Intent:
        if self._pending is not None:
            raise RuntimeError(f'next() before result() for {self._pending.kind.value}')
        if self._t0 is None:
            self._t0 = obs.t
        for _ in range(16):   # bounded internal transitions per poll
            if self.done:
                return Intent(IntentKind.END, outcome=self.outcome, reason=self.reason)
            it = self._advance(obs)
            if it is not None:
                if it.kind in _SYNC:
                    self._pending = it
                return it
        return Intent(IntentKind.WAIT)

    # ── internals ──
    def _end(self, outcome: Outcome, reason: str = '') -> None:
        self.outcome, self.reason, self.step = outcome, reason, Step.DONE
        self.history.append(f'END {outcome.value} {reason}'.strip())

    def _move(self, obs: RecoveryObs, target: float, purpose: str, step: Step) -> Intent:
        self.step = step
        self._move_t0, self._move_p0 = obs.t, obs.pos
        self._tracked = False
        self._moving_seen = False
        self._still_since = None
        if purpose not in ('lower', 'hold', 'anchor'):
            self.raises.append(target)
        self.history.append(f'MOVE {purpose} -> {target:+.3f}')
        return Intent(IntentKind.MOVE, target_rev=target, vel_rps=self.cfg.move_vel_rps,
                      purpose=purpose)

    def _begin_anchor(self, t: Optional[float]) -> None:
        """Enter ``Step.ANCHOR``: the next raise waits for the hand to rest,
        then anchors the setpoint on it (see the class docstring). ``t`` is
        the poll time, or ``None`` from a result handler (the first ANCHOR
        poll stamps it)."""
        self.step = Step.ANCHOR
        self._anchor_t0 = t
        self._rest_since = None
        self._anchor_tries = 0

    def _move_unavailable_reason(self, status: str, msg: str) -> Optional[str]:
        if status == STATUS_UNKNOWN_METHOD:
            return 'firmware has no HAND_MOVE_TO'
        if status == STATUS_NO_CLIENT_METHOD:
            return 'teensy_link RpcClient has no hand_move_to'
        return None

    def _advance(self, obs: RecoveryObs) -> Optional[Intent]:
        c = self.cfg
        st = self.step
        if st in _MONITORED and (not _finite(obs.pos, obs.vel)
                                 or obs.telem_age_s > c.telem_max_age_s):
            self._end(Outcome.HAND_JAM_UNRECOVERED,
                      f'hand telemetry lost/stale during the {st.name.lower()} '
                      f'(age {obs.telem_age_s:.2f} s) — nothing further commanded')
            return None
        if st is Step.TRIGGER:
            return Intent(IntentKind.HOLD_RECOVERING)
        if st is Step.RELIEF:
            return Intent(IntentKind.SET_CURRENT, curr_a=self.relief_curr_a,
                          purpose='relief')
        if st is Step.CLEAR:
            if obs.latched:
                return Intent(IntentKind.CONVERGE_CLEAR)
            self.step = Step.DISARM
            return None
        if st is Step.DISARM:
            return Intent(IntentKind.DISARM)
        if st is Step.ANCHOR:
            if obs.armed:
                self._end(Outcome.HAND_JAM_UNRECOVERED,
                          'wire re-armed before the raise — refusing to co-drive the hand')
                return None
            # Rest first: an anchor at a position the hand is still leaving
            # (the ball springing back 0.3 rev as 50 A becomes 10 A) would
            # drive it back there. A hand that will not settle is anchored
            # where it is after anchor_wait_max_s; the ARRIVED wait bounds it.
            if self._anchor_t0 is None:
                self._anchor_t0 = obs.t
            if abs(obs.vel) < c.lower_stall_vel_rps:
                if self._rest_since is None:
                    self._rest_since = obs.t
            else:
                self._rest_since = None
            rested = (self._rest_since is not None
                      and obs.t - self._rest_since >= c.anchor_rest_s - 1e-9)
            if not rested and obs.t - self._anchor_t0 < c.anchor_wait_max_s:
                return Intent(IntentKind.WAIT)
            if not rested:
                self.history.append(f'ANCHOR without rest after {c.anchor_wait_max_s:g} s')
            self.anchors += 1
            return self._move(obs, obs.pos, 'anchor', Step.ANCHORING)
        if st is Step.ANCHORING:
            if obs.move_done:
                if obs.move_ok:
                    self.step = Step.RAISE
                    return None
                why = self._move_unavailable_reason(obs.move_status, obs.move_msg)
                self._end(Outcome.HAND_JAM_UNRECOVERED,
                          why or f'anchor failed: {obs.move_status} {obs.move_msg}'.strip())
                return None
            if obs.t - self._move_t0 >= c.hold_wait_s:
                self._anchor_tries += 1
                if self._anchor_tries < c.anchor_attempts:
                    # The hand crept off the anchor: anchor again where it is now.
                    self.history.append(f'ANCHOR did not confirm, re-anchoring at {obs.pos:+.3f}')
                    self.anchors += 1
                    return self._move(obs, obs.pos, 'anchor', Step.ANCHORING)
                self._end(Outcome.HAND_JAM_UNRECOVERED,
                          f'the anchor at the measured position did not confirm '
                          f'{self._anchor_tries}x (hand at {obs.pos:+.3f} rev, moving?) — '
                          f'relief stays, nothing raised')
                return None
            return Intent(IntentKind.WAIT)
        if st is Step.RAISE:
            if obs.armed:
                self._end(Outcome.HAND_JAM_UNRECOVERED,
                          'wire re-armed before the raise — refusing to co-drive the hand')
                return None
            target = self.raise_target(obs.pos)
            if self.attempt >= c.raise_attempts:
                return self._move(obs, target, 'final_raise', Step.FINAL_RAISING)
            return self._move(obs, target, f'raise{self.attempt + 1}', Step.RAISING)
        if st in (Step.RAISING, Step.FINAL_RAISING):
            if obs.move_done and not obs.move_ok:
                # Checked BEFORE tracking: a raise the firmware refused never
                # moved, and its reason (e.g. no HAND_MOVE_TO) is the outcome.
                why = self._move_unavailable_reason(obs.move_status, obs.move_msg)
                self._end(Outcome.HAND_JAM_UNRECOVERED,
                          why or f'raise failed: {obs.move_status} {obs.move_msg}'.strip())
                return None
            if not self._tracked and obs.t - self._move_t0 >= c.raise_track_window_s:
                if obs.pos - self._move_p0 < c.raise_track_tol_rev:
                    # Abort to HOLD AT MEASURED: retarget the running move to
                    # where the hand is (a newer HAND_MOVE_TO supersedes it).
                    # Never leave it to the firmware's 10 s timeout, which
                    # IDLEs the hand.
                    self._hold_reason = (
                        f'raise did not track (rose {obs.pos - self._move_p0:+.3f} rev '
                        f'in {c.raise_track_window_s:.1f} s) — not a one-sided ball '
                        f'pinch (snag/bind/encoder?)')
                    return self._move(obs, obs.pos, 'hold', Step.HOLDING)
                self._tracked = True
            if not obs.move_done:
                return Intent(IntentKind.WAIT)
            if st is Step.FINAL_RAISING:
                self._end(Outcome.HAND_JAM_UNRECOVERED,
                          f'the lower stalled {self.lower_stalls}x — hand left RAISED at '
                          f'{obs.pos:+.3f} rev at {self.relief_curr_a:.0f} A; remove the '
                          f'ball, then /recover')
                return None
            self.step, self._dwell_t0 = Step.DWELL, obs.t
            return None
        if st is Step.HOLDING:
            if obs.move_done:
                self._end(Outcome.HAND_JAM_UNRECOVERED, self._hold_reason + (
                    f'; held at measured {obs.pos:+.3f} rev' if obs.move_ok else
                    f'; the hold move failed: {obs.move_status} {obs.move_msg}'.rstrip()))
                return None
            if obs.t - self._move_t0 >= c.hold_wait_s:
                self._end(Outcome.HAND_JAM_UNRECOVERED,
                          self._hold_reason + '; the hold at measured did not confirm')
                return None
            return Intent(IntentKind.WAIT)
        if st is Step.DWELL:
            if obs.t - self._dwell_t0 < c.dwell_s:
                return Intent(IntentKind.WAIT)
            self.step, self._gate_t0 = Step.BB_GATE, obs.t
            return None
        if st is Step.BB_GATE:
            if obs.bb_pending and obs.t - self._gate_t0 < c.bb_wait_max_s:
                return Intent(IntentKind.WAIT)
            self.step = Step.LOWER
            return None
        if st is Step.LOWER:
            if obs.armed:
                self._end(Outcome.HAND_JAM_UNRECOVERED,
                          'wire re-armed before the lower — refusing to co-drive the hand')
                return None
            return self._move(obs, c.park_rev, 'lower', Step.LOWERING)
        if st is Step.LOWERING:
            return self._lowering(obs)
        if st is Step.RESTORE:
            return Intent(IntentKind.SET_CURRENT, curr_a=self.restore_curr_a,
                          purpose='restore')
        return Intent(IntentKind.WAIT)

    def _lowering(self, obs: RecoveryObs) -> Optional[Intent]:
        c = self.cfg
        if abs(obs.vel) >= c.lower_stall_vel_rps:
            self._moving_seen, self._still_since = True, None
        elif ((self._moving_seen or obs.t - self._move_t0 >= c.lower_grace_s)
              and obs.pos > c.park_rev + c.lower_stall_margin_rev):
            if self._still_since is None:
                self._still_since = obs.t
            if obs.t - self._still_since >= c.lower_stall_s - 1e-9:
                # The ball is still there. Answer with a RAISE, never a push —
                # anchored first, because the lower's setpoint has run on
                # below the stalled hand and a retargeted raise would plan
                # from there (Step.RAISE picks raise-again or the final raise).
                self.lower_stalls += 1
                self.stall_pos = obs.pos
                self.history.append(f'LOWER STALL at {obs.pos:+.3f}')
                self.attempt += 1
                self._begin_anchor(obs.t)
                return None
        else:
            self._still_since = None
        if not obs.move_done:
            return Intent(IntentKind.WAIT)
        if not obs.move_ok:
            why = self._move_unavailable_reason(obs.move_status, obs.move_msg)
            self._end(Outcome.HAND_JAM_UNRECOVERED,
                      why or f'lower failed: {obs.move_status} {obs.move_msg}'.strip())
            return None
        if abs(obs.pos - c.park_rev) > c.park_tol_rev:
            self._end(Outcome.HAND_JAM_UNRECOVERED,
                      f'lower reported complete at {obs.pos:+.3f} rev, off the park — '
                      f'shipped current NOT restored')
            return None
        self.step = Step.RESTORE
        return None

    def _on_result(self, it: Intent, res: IntentResult) -> None:
        k = it.kind
        self.history.append(f'{k.value}{"(" + it.purpose + ")" if it.purpose else ""} '
                            f'-> {"ok" if res.ok else "FAIL " + res.status}')
        if k is IntentKind.HOLD_RECOVERING:
            self.step = Step.RELIEF
        elif k is IntentKind.SET_CURRENT and it.purpose == 'relief':
            if res.ok:
                self._tries = 0
                if self.kind is Verdict.HAND_STALL and not self.resume:
                    self._end(Outcome.HAND_STALL_RELIEVED,
                              f'out-of-band stall: relieved to {self.relief_curr_a:.0f} A, '
                              f'no automatic motion; /recover lowers and restores')
                else:
                    self.step = Step.CLEAR
            else:
                self._tries += 1
                if self._tries >= self.cfg.relief_attempts:
                    self._end(Outcome.HAND_JAM_UNRECOVERED,
                              f'relief SET_VEL_CURR_LIMITS failed {self._tries}x: {res.msg}')
                # else: the step stays RELIEF, so the next poll retries it.
        elif k is IntentKind.CONVERGE_CLEAR:
            if res.ok:
                self.step = Step.DISARM
            else:
                self._end(Outcome.HAND_JAM_UNRECOVERED,
                          f'guard clear failed (relief stays): {res.msg}')
        elif k is IntentKind.DISARM:
            if res.ok:
                if self.resume:
                    self.step = Step.BB_GATE
                else:
                    self._begin_anchor(None)
            else:
                self._end(Outcome.HAND_JAM_UNRECOVERED,
                          f'disarm did not confirm on the wire (relief stays): {res.msg}')
        elif k is IntentKind.MOVE:
            if not res.ok:
                why = self._move_unavailable_reason(res.status, res.msg)
                self._end(Outcome.HAND_JAM_UNRECOVERED,
                          why or f'{it.purpose} could not start: {res.status} {res.msg}')
        elif k is IntentKind.SET_CURRENT and it.purpose == 'restore':
            if res.ok:
                self._end(Outcome.HAND_JAM_RECOVERED,
                          f'hand at the park, {self.restore_curr_a:.0f} A restored')
            else:
                self._tries += 1
                if self._tries >= self.cfg.restore_attempts:
                    self._end(Outcome.HAND_JAM_UNRECOVERED,
                              f'restore to {self.restore_curr_a:.0f} A not acknowledged — '
                              f'current left at {self.relief_curr_a:.0f} A: {res.msg}')


def planned_steps(cfg: JamConfig, *, stall_pos: float, latched: bool, armed: bool,
                  restore_curr_a: float, vel_limit: float, kind: Verdict = Verdict.HAND_JAM,
                  resume: bool = False) -> List[str]:
    """The sequence ``JamRecovery`` would run from here, as text (dry run)."""
    rec = JamRecovery(cfg, kind=kind, stall_pos=stall_pos,
                      restore_curr_a=restore_curr_a, resume=resume)
    out = ['0 hold fault_state=RECOVERING (orchestrator FAULT, trajectory_node frozen, '
           'install_segment refused)',
           f'1 SET_VEL_CURR_LIMITS(6, {vel_limit:g} rev/s, {rec.relief_curr_a:g} A) — relief first']
    if kind is Verdict.HAND_STALL and not resume:
        out.append('END HAND_STALL_RELIEVED (out of band: no automatic motion)')
        return out
    out.append('2 ' + ('converge-first clear (_svc_recover steps 1-4), then ' if latched
                       else 'not latched: ') + ('disarm + confirm on the wire' if armed
                                                 else 'wire already disarmed (confirm)'))
    r1 = rec.raise_target(stall_pos)
    anchor = (f'anchor: HAND_MOVE_TO(the measured position) once the hand has rested '
              f'{cfg.anchor_rest_s:g} s (ARRIVED hands the setpoint to the hand; the push '
              f'stops), then ')
    if not resume:
        out.append(f'3 {anchor}HAND_MOVE_TO({r1:+.3f} rev, {cfg.move_vel_rps:g} rev/s) — '
                   f'must rise >= {cfg.raise_track_tol_rev:g} rev in '
                   f'{cfg.raise_track_window_s:g} s')
        out.append(f'4 dwell {cfg.dwell_s:g} s')
    out.append(f'4b wait for any Ball Butler throw to land (+1 s, <= {cfg.bb_wait_max_s:g} s)')
    out.append(f'5 HAND_MOVE_TO({cfg.park_rev:+.3f} rev) with the stall monitor '
               f'(|v| < {cfg.lower_stall_vel_rps:g} rev/s for {cfg.lower_stall_s:g} s above '
               f'{cfg.park_rev + cfg.lower_stall_margin_rev:+.2f} rev -> {anchor}raise to '
               f'{cfg.raise_to_rev:+.3f} rev' + ('' if resume else ', dwell, lower again')
               + f'; after {cfg.raise_attempts} cycles a stall stays raised, UNRECOVERED)')
    out.append(f'6 SET_VEL_CURR_LIMITS(6, {vel_limit:g} rev/s, {restore_curr_a:g} A) at the '
               f'park — restore LAST, then RECOVERED (wire left disarmed; orchestrator re-arms)')
    return out


def kinds_text(seq: Sequence[Intent]) -> List[str]:
    """Compact rendering of an intent sequence (tests, logs)."""
    return [f'{i.kind.value}:{i.purpose}' if i.purpose else i.kind.value for i in seq]
