"""Human Throw event emitter — pure Python (PWAY-11 P1, design 2026-11-18).

Turns the Tracker's `human_throw` balls and their `FlightFit`s into discrete
EVENTS, one stream per flight, identified exactly by `(track_id, flight_seq)`:
revision 0 when the flight first becomes usable, one revision per refinement,
and one `final` revision when the Fit freezes. The Tracker owns the facts
(flight segment, first state, Fit quality), so the phantom rule and the
usable rule are applied ONCE, here, at the source; consumers (the skill
stack) decide about lead and reach. A pure function of its inputs, so the
bench probe replays the same code on bag data that the node runs live.

PROPOSED owner decisions (not yet confirmed; every number lives in
`HumanThrowEventConfig`, the ONE place to change them):
  D1  phantom rule: a flight whose start speed is below 500 mm/s is not a
      Throw. Scored in the perception unit on the 2026-11-18 idle window and
      the 2026-11-02 bags: 9/9 idle phantoms (static markers 13-134 mm/s,
      one moving floor marker 365 mm/s) rejected, Throws lost 0/11, 1/44,
      4/82. For flight_seq >= 1 (a re-Throw on a kept track id) the input is
      the Kalman speed at the first admitted sample of the new flight; that
      variant is unscored (the P1 gate scores it).
  D2  usable rule for revision 0: Fit valid (FlightFit's own n >= 12 /
      span >= 0.05 s / two-pass gate), rms <= 6 mm, and two Fits at least
      20 ms apart agree on the landing within 30 mm and 20 ms, and the
      landing is still in the future.
Refinement thresholds (5 mm / 5 ms) are design rule 5, also proposed.

Not in this module by design: the Ball Butler (announced) balls (only
`source == 'human_throw'` qualifies), the BUG-92 exclusion box (the matcher
never promotes a track starting inside it), the lead rule (the skill stack's),
and any Pattern prior (it must never enter the landing).

Every candidate flight that fails a rule is counted per reason in
`rejections`; the phantom rejection is logged once per flight at INFO
(`HUMAN-THROW-REJECTED id 41 seq 0: ...`), and a flight that froze without
ever being usable is logged once (`HUMAN-THROW-UNSERVED`).
"""
from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Dict, Iterable, List, Mapping, Optional, Tuple

import numpy as np

from .ball import Ball, BallStatus, TrackingConfidence
from .ballistics import predict_landing_state
from .flight_fit import FlightFit

_log = logging.getLogger(__name__)

# Reasons (counter keys).
R_PHANTOM = 'phantom_start_speed'
R_SPEED_UNKNOWN = 'start_speed_unknown'
R_NO_FIT = 'fit_invalid'
R_RMS = 'rms_too_high'
R_UNSTABLE = 'unstable'
R_PAST = 'landing_not_future'
R_NO_LANDING = 'no_landing'


@dataclass(frozen=True)
class HumanThrowEventConfig:
    """The proposed owner decisions, in one place (see the module doc)."""
    min_start_speed_mm_s: float = 500.0   # D1
    rms_mm: float = 6.0                   # D2
    stable_mm: float = 30.0               # D2
    stable_gap_ms: float = 20.0           # D2 PROPOSED: minimum gap between the two Fits
    stable_time_ms: float = 20.0           # D2 PROPOSED: landing-time agreement of the two Fits
    refine_mm: float = 5.0                # design rule 5
    refine_ms: float = 5.0                # design rule 5


@dataclass
class HumanThrowEvent:
    """One published revision of one flight (mirrors the U2 message)."""
    track_id: int
    flight_seq: int
    revision: int
    stamp: float                  # `now` of the frame that produced it
    flight_start: float           # first admitted sample time
    landing_position: np.ndarray  # mm, mocap world frame, at landing_z_mm
    landing_velocity: np.ndarray  # mm/s
    landing_time: float           # absolute seconds
    landing_z_mm: float
    n_samples: int
    rms_mm: float
    span_s: float
    fit_age_s: float              # stamp minus the last admitted sample time
    start_speed_mm_s: float
    final: bool


class _Flight:
    """Emitter state for one (track id, flight_seq)."""
    __slots__ = ('history', 'revision', 'last_pos', 'last_t', 'final_sent',
                 'phantom', 'last_reason', 'unserved_logged')

    def __init__(self) -> None:
        # (now, landing_pos, landing_t) of recent Fits that passed the rms bar.
        self.history: List[Tuple[float, np.ndarray, float]] = []
        self.revision = -1          # last published revision, -1 = none
        self.last_pos: Optional[np.ndarray] = None
        self.last_t = 0.0
        self.final_sent = False
        self.phantom = False
        self.last_reason = R_NO_FIT
        self.unserved_logged = False


class HumanThrowEventEmitter:
    """`step(balls, fits, now) -> [HumanThrowEvent]`, call once per frame."""

    _HISTORY_S = 0.5

    def __init__(self, config: Optional[HumanThrowEventConfig] = None,
                 logger: Optional[logging.Logger] = None) -> None:
        self.config = config or HumanThrowEventConfig()
        self._logger = logger or _log
        self._flights: Dict[Tuple[int, int], _Flight] = {}
        #: reason -> count of flights (phantom) or evaluations (others).
        self.rejections: Dict[str, int] = {}

    # ------------------------------------------------------------------
    def step(self, balls: Iterable[Ball], fits: Mapping[int, FlightFit],
             now: float) -> List[HumanThrowEvent]:
        events: List[HumanThrowEvent] = []
        live_ids = set()
        for ball in balls:
            if (ball.source != 'human_throw'
                    or ball.status != BallStatus.IN_FLIGHT
                    or ball.tracking != TrackingConfidence.CONFIRMED):
                continue
            fit = fits.get(ball.id)
            if fit is None:
                continue
            live_ids.add(ball.id)
            ev = self._step_flight(ball.id, fit, float(now))
            if ev is not None:
                events.append(ev)
        # Forget flights of tracks that are gone or have moved on.
        for key in [k for k in self._flights
                    if k[0] not in live_ids or k[1] < self._seq_of(k[0], fits)]:
            del self._flights[key]
        return events

    @staticmethod
    def _seq_of(track_id: int, fits: Mapping[int, FlightFit]) -> int:
        f = fits.get(track_id)
        return f.flight_seq if f is not None else 0

    # ------------------------------------------------------------------
    def _count(self, reason: str) -> None:
        self.rejections[reason] = self.rejections.get(reason, 0) + 1

    def _step_flight(self, track_id: int, fit: FlightFit,
                     now: float) -> Optional[HumanThrowEvent]:
        cfg = self.config
        key = (track_id, fit.flight_seq)
        fl = self._flights.get(key)
        if fl is None:
            fl = self._flights[key] = _Flight()
        if fl.phantom or fl.final_sent:
            return None

        # Phantom rule (D1): decided once, when the start speed is known.
        speed = fit.start_speed_mm_s
        if speed is None:
            if fit.start_speed_pending:
                return None  # the Tracker has not seen the first sample yet
            fl.phantom = True
            self._count(R_SPEED_UNKNOWN)
            self._logger.info(
                'HUMAN-THROW-REJECTED id %d seq %d: start speed unknown',
                track_id, fit.flight_seq)
            return None
        if speed < cfg.min_start_speed_mm_s:
            fl.phantom = True
            self._count(R_PHANTOM)
            self._logger.info(
                'HUMAN-THROW-REJECTED id %d seq %d: start speed %.0f mm/s < %.0f',
                track_id, fit.flight_seq, speed, cfg.min_start_speed_mm_s)
            return None

        frozen = fit.frozen
        usable, landing = self._evaluate(fl, fit, now)
        if landing is not None and landing[2] <= now:
            usable, fl.last_reason = False, R_PAST
            self._count(R_PAST)

        event: Optional[HumanThrowEvent] = None
        if landing is not None and usable:
            pos, vel, t_land, fr = landing
            if fl.revision < 0:
                stable = self._stable(fl, now, pos, t_land)
                self._record(fl, now, pos, t_land)
                if stable:
                    event = self._make(track_id, fit, fl, now, landing, frozen)
                else:
                    fl.last_reason = R_UNSTABLE
                    self._count(R_UNSTABLE)
            else:
                self._record(fl, now, pos, t_land)
                moved = (float(np.linalg.norm(pos - fl.last_pos)) > cfg.refine_mm
                         or abs(t_land - fl.last_t) * 1000.0 > cfg.refine_ms)
                if moved or frozen:
                    event = self._make(track_id, fit, fl, now, landing, frozen)

        if frozen and event is None and fl.revision < 0 and not fl.unserved_logged:
            fl.unserved_logged = True
            self._logger.info(
                'HUMAN-THROW-UNSERVED id %d seq %d: froze without a usable Fit (%s)',
                track_id, fit.flight_seq, fl.last_reason)
        return event

    # ------------------------------------------------------------------
    def _evaluate(self, fl: _Flight, fit: FlightFit, now: float):
        """(usable-ignoring-stability, (pos, vel, t_land, FitResult) | None)."""
        fr = fit.fit()
        if fr is None:
            fl.last_reason = R_NO_FIT
            self._count(R_NO_FIT)
            return False, None
        res = predict_landing_state(fr.pos_ref, fr.vel_ref, fit.landing_z,
                                    g_xy_mmps2=fit.gravity_xy_mmps2)
        if res is None:
            fl.last_reason = R_NO_LANDING
            self._count(R_NO_LANDING)
            return False, None
        pos, vel, ttl = res
        landing = (np.asarray(pos, dtype=np.float64),
                   np.asarray(vel, dtype=np.float64), fr.t_ref + ttl, fr)
        if fr.rms_mm > self.config.rms_mm:
            fl.last_reason = R_RMS
            self._count(R_RMS)
            return False, landing
        return True, landing

    def _record(self, fl: _Flight, now: float, pos: np.ndarray,
                t_land: float) -> None:
        fl.history.append((now, pos, t_land))
        fl.history = [h for h in fl.history if now - h[0] <= self._HISTORY_S]

    def _stable(self, fl: _Flight, now: float, pos: np.ndarray,
                t_land: float) -> bool:
        """Some usable Fit at least `stable_gap_ms` older agrees with this one
        (position within `stable_mm`, landing time within `stable_time_ms`)."""
        cfg = self.config
        gap = cfg.stable_gap_ms / 1000.0 - 1e-9
        for t_then, p_then, tl_then in fl.history:
            if now - t_then >= gap:
                if (float(np.linalg.norm(pos - p_then)) <= cfg.stable_mm
                        and abs(t_land - tl_then) * 1000.0 <= cfg.stable_time_ms):
                    return True
        return False

    def _make(self, track_id: int, fit: FlightFit, fl: _Flight, now: float,
              landing, frozen: bool) -> HumanThrowEvent:
        pos, vel, t_land, fr = landing
        fl.revision += 1
        fl.last_pos = pos
        fl.last_t = t_land
        if frozen:
            fl.final_sent = True
        last_t = fit.last_sample_t
        return HumanThrowEvent(
            track_id=track_id,
            flight_seq=fit.flight_seq,
            revision=fl.revision,
            stamp=now,
            flight_start=float(fit.flight_start_t if fit.flight_start_t is not None
                               else fr.t_ref),
            landing_position=pos.copy(),
            landing_velocity=vel.copy(),
            landing_time=float(t_land),
            landing_z_mm=float(fit.landing_z),
            n_samples=int(fr.n),
            rms_mm=float(fr.rms_mm),
            span_s=float(fr.span_s),
            fit_age_s=float(now - (last_t if last_t is not None else fr.t_ref)),
            start_speed_mm_s=float(fit.start_speed_mm_s),
            final=bool(frozen),
        )
