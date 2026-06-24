"""Hand trajectory generator — Python port of Teensy Trajectory.h.

Provides both catch and throw trajectories:

- HandCatchTrajectory / HandCatchSequence: port of makeCatch() — hand moves
  downward at CATCH_VEL_RATIO × ball_speed to reduce impact velocity.
- HandThrowTrajectory / HandThrowSequence: port of calcThrow() — hand moves
  upward to eject the ball at a desired speed.

Also provides HandSmoothMove (quintic S-curve) used by both sequences.

All public positions are in mm (matching sim/plant command_hand()).
Internally uses metres to match Trajectory.h math, converting at boundaries.

Timeline conventions:
- Catch: t=0 is the moment the ball arrives (midpoint of velocity-hold phase).
- Throw: t=0 is the moment of ball release (end of velocity-hold phase).
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Constants — from config/hardware_config.yaml  teensy_trajectory section
# ---------------------------------------------------------------------------
GRAVITY_MPS2 = 9.806
HAND_SPOOL_RADIUS_M = 0.00521
LINEAR_GAIN_FACTOR = 1.035
INERTIA_HAND_ONLY_KG = 0.281
INERTIA_RATIO = 0.747
CATCH_VEL_RATIO = 0.6  # matches hardware_config.yaml teensy_trajectory.catch_vel_ratio
                       # (the platform/Jugglebot hand). Hardware-validated as
                       # reliable; tunable later. (Was 0.9 — a stale port value.)
CATCH_VEL_HOLD_PCT = 0.10
HAND_STROKE_M = 0.355
STROKE_MARGIN_M = 0.02
END_PROFILE_HOLD_S = 0.10
MAX_SMOOTH_MOVE_HAND_ACCEL_RPS2 = 100.0
QUINTIC_S2_MAX = 5.7735027
MIN_EVENT_VEL_MPS = 0.3
MAX_EVENT_VEL_MPS = 7.0
SAFETY_GAP_S = 0.02  # 20 ms safety gap (from teensy_operational.safety_gap_us)

# Throw-specific constants
THROW_VEL_HOLD_PCT = 0.05  # 5% of effective stroke for velocity hold

# Derived
_LINEAR_GAIN = LINEAR_GAIN_FACTOR / (math.pi * HAND_SPOOL_RADIUS_M * 2.0)  # rev/m
_TOTAL_STROKE_M = HAND_STROKE_M - 2.0 * STROKE_MARGIN_M  # 0.315 m
_TOTAL_STROKE_MM = _TOTAL_STROKE_M * 1000.0  # 315 mm
STROKE_MARGIN_MM = STROKE_MARGIN_M * 1000.0  # 20 mm


# ---------------------------------------------------------------------------
# HandCatchTrajectory — port of calcCatch() + buildCatch()
# ---------------------------------------------------------------------------

class HandCatchTrajectory:
    """3-segment catch trajectory matching Teensy buildCatch().

    The trajectory moves the hand downward from ``start_pos_mm`` over the
    effective stroke (315 mm).  The hand velocity during the hold phase is
    ``-CATCH_VEL_RATIO * event_vel_mps`` (negative = downward).

    Timeline: t=0 is the midpoint of the velocity-hold phase — the instant
    the ball is expected to arrive.

    Parameters
    ----------
    event_vel_mps : float
        Ball speed at landing (m/s).  Clamped to [0.3, 7.0].
    start_pos_mm : float
        Hand position (mm from physical bottom) at the start of the catch
        trajectory.  Normally the top of effective stroke = STROKE_MARGIN_MM
        + TOTAL_STROKE_MM ≈ 335 mm.
    """

    def __init__(self, event_vel_mps: float, start_pos_mm: float | None = None):
        v_throw = max(MIN_EVENT_VEL_MPS, min(MAX_EVENT_VEL_MPS, event_vel_mps))

        # Default start = top of effective stroke
        if start_pos_mm is None:
            start_pos_mm = (STROKE_MARGIN_M + _TOTAL_STROKE_M) * 1000.0

        self._start_pos_mm = start_pos_mm

        # --- port of calcCatch() (Trajectory.h lines 117-139) ---
        vC = -CATCH_VEL_RATIO * v_throw           # m/s, negative (downward)
        irC = 1.0 / INERTIA_RATIO

        vel_hold_m = CATCH_VEL_HOLD_PCT * _TOTAL_STROKE_M     # 0.0315 m
        accel_stroke_m = _TOTAL_STROKE_M - vel_hold_m          # 0.2835 m

        # Segment durations (all positive)
        t_acc = -(2.0 / (irC + 1.0)) * accel_stroke_m / vC
        t_vel = -vel_hold_m / vC
        t_dec = t_acc * irC

        # Accelerations
        catchA = vC / t_acc     # negative (accelerate downward)
        catchD = -catchA / irC  # positive (decelerate)

        # --- Positions in "effective stroke" metres (0 = top, going negative) ---
        # Segment 1: accel from rest
        x_after_accel = 0.5 * catchA * t_acc ** 2              # negative
        # Segment 2: constant velocity hold
        x_after_hold = x_after_accel + vC * t_vel              # more negative
        # Segment 3: deceleration to rest
        # x_end = x_after_hold + vC * t_dec + 0.5 * catchD * t_dec^2

        # --- Time origin: t=0 at midpoint of velocity hold ---
        # makeCatch() shifts by -(t5-t4) where t5 is vel-hold start, t4 is
        # accel start.  In our local timeline (accel starts at local 0):
        #   vel-hold starts at t_acc
        #   midpoint of vel-hold at t_acc + 0.5*t_vel
        # So t=0 in ball-arrival coords = local (t_acc + 0.5*t_vel)
        self._t_offset = t_acc + 0.5 * t_vel  # local time of t=0

        # Store segment parameters
        self._t_acc = t_acc
        self._t_vel = t_vel
        self._t_dec = t_dec
        self._catchA = catchA
        self._catchD = catchD
        self._vC = vC

        # Segment boundary positions (metres, displacement from start, all <= 0)
        self._x1 = x_after_accel
        self._x2 = x_after_hold

        # Public timing (relative to ball arrival = t=0)
        self._t_start = -self._t_offset                               # negative
        self._t_end = (t_acc + t_vel + t_dec + END_PROFILE_HOLD_S
                       - self._t_offset)                               # positive

    @property
    def start_time(self) -> float:
        """Time before ball arrival when trajectory starts (negative)."""
        return self._t_start

    @property
    def end_time(self) -> float:
        """Time after ball arrival when trajectory ends (positive)."""
        return self._t_end

    @property
    def duration(self) -> float:
        """Total duration of the catch trajectory (seconds)."""
        return self._t_end - self._t_start

    @property
    def start_pos_mm(self) -> float:
        """Hand position at trajectory start (mm from physical bottom)."""
        return self._start_pos_mm

    @property
    def end_pos_mm(self) -> float:
        """Hand position at trajectory end (mm from physical bottom)."""
        return self._start_pos_mm + self._displacement_at_local(
            self._t_acc + self._t_vel + self._t_dec) * 1000.0

    @property
    def catch_velocity_mps(self) -> float:
        """Hand velocity during the constant-velocity hold phase (m/s, negative)."""
        return self._vC

    def sample(self, t: float) -> float:
        """Return hand position (mm) at time *t* relative to ball arrival.

        Before start_time: returns start_pos_mm.
        After end_time: returns end_pos_mm (hold at final position).
        """
        if t <= self._t_start:
            return self._start_pos_mm
        if t >= self._t_end:
            return self.end_pos_mm

        # Convert to local time (0 = accel start)
        t_local = t + self._t_offset
        disp_m = self._displacement_at_local(t_local)
        return self._start_pos_mm + disp_m * 1000.0

    def _displacement_at_local(self, t_local: float) -> float:
        """Displacement in metres from start position at local time."""
        if t_local <= 0.0:
            return 0.0

        t_acc = self._t_acc
        t_vel = self._t_vel
        t_dec = self._t_dec

        if t_local <= t_acc:
            # Segment 1: accelerate from rest
            return 0.5 * self._catchA * t_local ** 2

        if t_local <= t_acc + t_vel:
            # Segment 2: constant velocity hold
            tau = t_local - t_acc
            return self._x1 + self._vC * tau

        if t_local <= t_acc + t_vel + t_dec:
            # Segment 3: decelerate to rest
            tau = t_local - (t_acc + t_vel)
            return self._x2 + self._vC * tau + 0.5 * self._catchD * tau ** 2

        # End hold: stay at final position
        tau_end = t_dec
        return self._x2 + self._vC * tau_end + 0.5 * self._catchD * tau_end ** 2


# ---------------------------------------------------------------------------
# HandSmoothMove — port of makeSmoothMove()
# ---------------------------------------------------------------------------

class HandSmoothMove:
    """Quintic S-curve point-to-point move, matching Teensy makeSmoothMove().

    Quintic polynomial: s(τ) = 10τ³ − 15τ⁴ + 6τ⁵ where τ = t/T.
    Duration T chosen so that peak acceleration ≤ MAX_SMOOTH_MOVE_HAND_ACCEL.

    Parameters
    ----------
    start_pos_mm : float
        Starting hand position (mm).
    end_pos_mm : float
        Target hand position (mm).
    """

    def __init__(self, start_pos_mm: float, end_pos_mm: float):
        self._start = start_pos_mm
        self._end = end_pos_mm
        delta_mm = end_pos_mm - start_pos_mm

        if abs(delta_mm) < 0.001:  # effectively zero
            self._duration = 0.0
            return

        # Convert delta to revolutions for accel-limit calculation
        # (MAX_SMOOTH_MOVE_HAND_ACCEL is in rev/s², so delta must be in rev)
        delta_rev = (delta_mm / 1000.0) * _LINEAR_GAIN  # mm → m → rev

        # T = sqrt(|delta_rev| * QUINTIC_S2_MAX / MAX_ACCEL)
        T = math.sqrt(abs(delta_rev) * QUINTIC_S2_MAX
                       / MAX_SMOOTH_MOVE_HAND_ACCEL_RPS2)
        self._duration = max(T, 0.05)  # minimum 50 ms (matches Teensy guard)

    @property
    def duration(self) -> float:
        """Duration of the smooth move (seconds)."""
        return self._duration

    @property
    def start_pos_mm(self) -> float:
        return self._start

    @property
    def end_pos_mm(self) -> float:
        return self._end

    def sample(self, t: float) -> float:
        """Return hand position (mm) at time *t* from move start.

        Returns start position for t < 0, end position for t > duration.
        """
        if self._duration == 0.0:
            return self._end

        if t <= 0.0:
            return self._start
        if t >= self._duration:
            return self._end

        tau = t / self._duration
        tau2 = tau * tau
        tau3 = tau2 * tau
        tau4 = tau2 * tau2
        # s(τ) = 10τ³ − 15τ⁴ + 6τ⁵
        s = 10.0 * tau3 - 15.0 * tau4 + 6.0 * tau4 * tau

        return self._start + (self._end - self._start) * s


# ---------------------------------------------------------------------------
# HandCatchSequence — smooth-move prelude + catch trajectory + hold
# ---------------------------------------------------------------------------

@dataclass
class HandCatchResult:
    """Result of attempting to create a HandCatchSequence."""
    sequence: HandCatchSequence | None
    feasible: bool
    reason: str = ""


class HandCatchSequence:
    """Complete hand catch sequence: smooth-move to start → catch trajectory.

    The sequence is anchored to absolute sim time via ``arrival_time``.

    Parameters
    ----------
    event_vel_mps : float
        Ball speed at landing (m/s).
    arrival_time : float
        Absolute sim time when ball arrives.
    current_pos_mm : float
        Current hand position (mm from physical bottom).
    """

    def __init__(
        self,
        event_vel_mps: float,
        arrival_time: float,
        current_pos_mm: float,
    ):
        # Build catch trajectory (positions from top of effective stroke)
        self._catch = HandCatchTrajectory(event_vel_mps)
        self._arrival_time = arrival_time

        # Catch trajectory start in absolute time
        self._catch_abs_start = arrival_time + self._catch.start_time

        # Build smooth-move from current position to catch trajectory start
        self._prelude = HandSmoothMove(current_pos_mm, self._catch.start_pos_mm)

        # Smooth-move must finish before catch trajectory starts
        self._prelude_start = self._catch_abs_start - self._prelude.duration

        # Absolute end time
        self._abs_end = arrival_time + self._catch.end_time

    @property
    def prelude_start_time(self) -> float:
        """Absolute sim time when smooth-move prelude begins."""
        return self._prelude_start

    @property
    def arrival_time(self) -> float:
        """Absolute sim time of ball arrival."""
        return self._arrival_time

    @property
    def end_time(self) -> float:
        """Absolute sim time when the full sequence ends."""
        return self._abs_end

    @property
    def catch_trajectory(self) -> HandCatchTrajectory:
        """The underlying catch trajectory (for inspection/testing)."""
        return self._catch

    @property
    def prelude(self) -> HandSmoothMove:
        """The smooth-move prelude (for inspection/testing)."""
        return self._prelude

    def sample(self, sim_time: float) -> float | None:
        """Return hand position (mm) at the given absolute sim time.

        Returns ``None`` after the sequence is complete.
        Before the prelude starts, returns the prelude's start position
        (hand should already be near there from priming).
        """
        if sim_time > self._abs_end:
            return None

        if sim_time < self._catch_abs_start:
            # In prelude phase
            t_prelude = sim_time - self._prelude_start
            return self._prelude.sample(t_prelude)

        # In catch trajectory phase
        t_catch = sim_time - self._arrival_time  # relative to ball arrival
        return self._catch.sample(t_catch)

    @staticmethod
    def try_create(
        event_vel_mps: float,
        arrival_time: float,
        current_pos_mm: float,
        current_time: float,
    ) -> HandCatchResult:
        """Attempt to create a HandCatchSequence with timing budget check.

        If there isn't enough time for the smooth-move prelude to complete
        before the catch trajectory starts (plus a safety gap), the catch
        attempt is infeasible and should be aborted.

        Parameters
        ----------
        event_vel_mps : float
            Ball speed at landing (m/s).
        arrival_time : float
            Absolute sim time when ball arrives.
        current_pos_mm : float
            Current hand position (mm).
        current_time : float
            Current sim time.

        Returns
        -------
        HandCatchResult
            ``feasible=True`` with a valid ``sequence``, or
            ``feasible=False`` with ``sequence=None`` and a ``reason``.
        """
        seq = HandCatchSequence(event_vel_mps, arrival_time, current_pos_mm)

        # Check timing budget: prelude must start no earlier than now,
        # with a safety gap
        time_available = seq.prelude_start_time - current_time
        if time_available < -SAFETY_GAP_S:
            shortfall = -time_available
            return HandCatchResult(
                sequence=None,
                feasible=False,
                reason=(
                    f"Insufficient time for hand catch: need "
                    f"{seq._prelude.duration:.3f}s smooth-move + "
                    f"{-seq._catch.start_time:.3f}s catch lead, "
                    f"but only {arrival_time - current_time:.3f}s until "
                    f"arrival (shortfall {shortfall:.3f}s)"
                ),
            )

        return HandCatchResult(sequence=seq, feasible=True)


# ---------------------------------------------------------------------------
# HandThrowTrajectory — port of calcThrow() + buildThrow()
# ---------------------------------------------------------------------------

class HandThrowTrajectory:
    """3-segment throw trajectory matching Teensy buildThrow().

    Moves the hand upward: accelerate → constant-velocity hold → decelerate.
    Ball is released at the end of the velocity-hold phase (t = 0 by convention).

    Timeline: t=0 is the moment of ball release (end of velocity-hold phase).

    Parameters
    ----------
    throw_speed_mps : float
        Desired ejection speed (m/s).  Clamped to [MIN, MAX].
    start_pos_mm : float
        Hand position at trajectory start (mm from physical bottom).
        Default: bottom of effective stroke (STROKE_MARGIN_MM).
    """

    def __init__(self, throw_speed_mps: float, start_pos_mm: float | None = None):
        v_throw = max(MIN_EVENT_VEL_MPS, min(MAX_EVENT_VEL_MPS, throw_speed_mps))
        self._throw_speed_mps = v_throw

        # Default start = bottom of effective stroke
        if start_pos_mm is None:
            start_pos_mm = STROKE_MARGIN_MM
        self._start_pos_mm = start_pos_mm

        # --- port of calcThrow() ---
        accel_stroke_m = (1.0 - THROW_VEL_HOLD_PCT) * _TOTAL_STROKE_M  # 0.95 * 0.315
        vel_hold_m = THROW_VEL_HOLD_PCT * _TOTAL_STROKE_M               # 0.05 * 0.315

        # Segment durations (positive)
        t_acc = 2.0 / (INERTIA_RATIO + 1.0) * accel_stroke_m / v_throw
        t_vel = vel_hold_m / v_throw
        t_dec = t_acc * INERTIA_RATIO

        # Accelerations
        throwA = v_throw / t_acc           # positive (upward)
        throwD = -throwA / INERTIA_RATIO   # negative (deceleration)

        # Positions in metres from effective bottom (positive = upward)
        x1_m = 0.5 * throwA * t_acc ** 2           # end of accel
        x2_m = x1_m + v_throw * t_vel               # end of vel hold = release point
        # x3_m computed for end position
        x3_m = x2_m + v_throw * t_dec + 0.5 * throwD * t_dec ** 2

        # Store parameters
        self._t_acc = t_acc
        self._t_vel = t_vel
        self._t_dec = t_dec
        self._throwA = throwA
        self._throwD = throwD
        self._x1_m = x1_m
        self._x2_m = x2_m
        self._x3_m = x3_m

        # Timeline: t=0 at release (end of vel-hold)
        # Trajectory starts at -(t_acc + t_vel), ends at +t_dec
        self._t_start = -(t_acc + t_vel)
        self._t_end = t_dec

    @property
    def release_time(self) -> float:
        """Always 0.0 (by timeline convention)."""
        return 0.0

    @property
    def release_pos_mm(self) -> float:
        """Hand position at ball release (mm from physical bottom)."""
        return self._start_pos_mm + self._x2_m * 1000.0

    @property
    def release_speed_mps(self) -> float:
        """Hand speed at release = throw_speed_mps (clamped)."""
        return self._throw_speed_mps

    @property
    def start_time(self) -> float:
        """Time before release when trajectory starts (negative)."""
        return self._t_start

    @property
    def end_time(self) -> float:
        """Time after release when trajectory ends (positive)."""
        return self._t_end

    @property
    def start_pos_mm(self) -> float:
        """Hand position at trajectory start (mm from physical bottom)."""
        return self._start_pos_mm

    @property
    def end_pos_mm(self) -> float:
        """Hand position at trajectory end (mm from physical bottom)."""
        return self._start_pos_mm + self._x3_m * 1000.0

    def sample(self, t: float) -> float:
        """Hand position (mm) at time t relative to ball release.

        Before start_time: returns start_pos_mm.
        After end_time: returns end_pos_mm (hold at final position).
        """
        if t <= self._t_start:
            return self._start_pos_mm
        if t >= self._t_end:
            return self.end_pos_mm

        # Convert to local time (0 = accel start)
        t_local = t - self._t_start  # shift so accel starts at 0
        disp_m = self._displacement_at_local(t_local)
        return self._start_pos_mm + disp_m * 1000.0

    def velocity_at(self, t: float) -> float:
        """Hand velocity (m/s, positive = upward) at time t relative to release."""
        if t <= self._t_start:
            return 0.0
        if t >= self._t_end:
            return 0.0

        t_local = t - self._t_start

        if t_local <= self._t_acc:
            return self._throwA * t_local

        if t_local <= self._t_acc + self._t_vel:
            return self._throw_speed_mps

        # Deceleration phase
        tau = t_local - (self._t_acc + self._t_vel)
        return self._throw_speed_mps + self._throwD * tau

    def _displacement_at_local(self, t_local: float) -> float:
        """Displacement in metres from start position at local time."""
        if t_local <= 0.0:
            return 0.0

        if t_local <= self._t_acc:
            return 0.5 * self._throwA * t_local ** 2

        if t_local <= self._t_acc + self._t_vel:
            tau = t_local - self._t_acc
            return self._x1_m + self._throw_speed_mps * tau

        if t_local <= self._t_acc + self._t_vel + self._t_dec:
            tau = t_local - (self._t_acc + self._t_vel)
            return self._x2_m + self._throw_speed_mps * tau + 0.5 * self._throwD * tau ** 2

        return self._x3_m


# ---------------------------------------------------------------------------
# HandThrowSequence — smooth-move prelude + throw trajectory
# ---------------------------------------------------------------------------

@dataclass
class HandThrowResult:
    """Result of attempting to create a HandThrowSequence."""
    sequence: HandThrowSequence | None
    feasible: bool
    reason: str = ""


class HandThrowSequence:
    """Complete throw sequence: smooth-move to start → throw trajectory.

    Anchored to absolute sim time via ``release_time``.

    Parameters
    ----------
    throw_speed_mps : float
        Desired ejection speed (m/s).
    release_time : float
        Absolute sim time when ball should be released.
    current_pos_mm : float
        Current hand position (mm from physical bottom).
    """

    def __init__(
        self,
        throw_speed_mps: float,
        release_time: float,
        current_pos_mm: float,
    ):
        self._throw = HandThrowTrajectory(throw_speed_mps)
        self._release_time = release_time

        # Throw trajectory start in absolute time
        self._throw_abs_start = release_time + self._throw.start_time

        # Build smooth-move from current position to throw trajectory start
        self._prelude = HandSmoothMove(current_pos_mm, self._throw.start_pos_mm)

        # Smooth-move must finish before throw trajectory starts
        self._prelude_start = self._throw_abs_start - self._prelude.duration

        # Absolute end time
        self._abs_end = release_time + self._throw.end_time

    @property
    def prelude_start_time(self) -> float:
        """Absolute sim time when smooth-move prelude begins."""
        return self._prelude_start

    @property
    def release_time(self) -> float:
        """Absolute sim time of ball release."""
        return self._release_time

    @property
    def end_time(self) -> float:
        """Absolute sim time when throw trajectory ends."""
        return self._abs_end

    @property
    def throw_trajectory(self) -> HandThrowTrajectory:
        """The underlying throw trajectory (for inspection/testing)."""
        return self._throw

    @property
    def prelude(self) -> HandSmoothMove:
        """The smooth-move prelude (for inspection/testing)."""
        return self._prelude

    @property
    def end_pos_mm(self) -> float:
        """Hand position after throw completes (mm from physical bottom)."""
        return self._throw.end_pos_mm

    def sample(self, sim_time: float) -> float | None:
        """Hand position (mm) at the given absolute sim time.

        Returns ``None`` after the sequence is complete.
        Before the prelude starts, returns the prelude's start position.
        """
        if sim_time > self._abs_end:
            return None

        if sim_time < self._throw_abs_start:
            # In prelude phase
            t_prelude = sim_time - self._prelude_start
            return self._prelude.sample(t_prelude)

        # In throw trajectory phase
        t_throw = sim_time - self._release_time  # relative to ball release
        return self._throw.sample(t_throw)

    @staticmethod
    def try_create(
        throw_speed_mps: float,
        release_time: float,
        current_pos_mm: float,
        current_time: float,
    ) -> HandThrowResult:
        """Create with feasibility check (enough time for smooth-move prelude).

        Parameters
        ----------
        throw_speed_mps : float
            Desired ejection speed (m/s).
        release_time : float
            Absolute sim time when ball should be released.
        current_pos_mm : float
            Current hand position (mm).
        current_time : float
            Current sim time.

        Returns
        -------
        HandThrowResult
            ``feasible=True`` with a valid ``sequence``, or
            ``feasible=False`` with ``sequence=None`` and a ``reason``.
        """
        seq = HandThrowSequence(throw_speed_mps, release_time, current_pos_mm)

        time_available = seq.prelude_start_time - current_time
        if time_available < -SAFETY_GAP_S:
            shortfall = -time_available
            return HandThrowResult(
                sequence=None,
                feasible=False,
                reason=(
                    f"Insufficient time for hand throw: need "
                    f"{seq._prelude.duration:.3f}s smooth-move + "
                    f"{-seq._throw.start_time:.3f}s throw lead, "
                    f"but only {release_time - current_time:.3f}s until "
                    f"release (shortfall {shortfall:.3f}s)"
                ),
            )

        return HandThrowResult(sequence=seq, feasible=True)


# ---------------------------------------------------------------------------
# max_throw_speed_mps — maximum achievable throw speed
# ---------------------------------------------------------------------------

def max_throw_speed_mps() -> float:
    """Maximum ejection speed achievable within the hand's stroke and accel limits.

    The throw uses accel_stroke = (1 - THROW_VEL_HOLD_PCT) * TOTAL_STROKE for
    acceleration, with the kinematic relation v² = 2 * a * d.  However, the
    acceleration itself depends on v (t_acc = 2/(IR+1) * accel_stroke / v),
    so throwA = v * (IR+1) / (2 * accel_stroke).  This means the stroke is
    always fully consumed — the limit is actually MAX_EVENT_VEL_MPS.

    Returns the clamped maximum (MAX_EVENT_VEL_MPS = 7.0 m/s).
    """
    return MAX_EVENT_VEL_MPS
