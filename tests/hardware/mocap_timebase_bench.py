#!/usr/bin/env python3
"""Mocap time-base + vertical-scale bench — drive the bench leg, log the encoder.

Lives in ``tests/hardware/`` and not ``tools/probes/`` because it COMMANDS MOTORS;
``tools/probes/README.md`` reserves that for here ("never command the motors [...]
that's tests/hardware/ territory"). Its outputs still go to ``temp/probes/``.

WHAT THIS IS FOR
----------------
The ``juggling-analyser`` project (E:/PDJ/Github/juggling-analyser) fits parabolas
to Qualisys mocap trajectories of thrown balls and recovers ``g``. It reads **low**,
consistently: -2.50% on two 2024 clips, -5.41% on a 2026 clip. Nine hypotheses have
been eliminated with numbers (marker scale in four orientations, sensor noise, air
drag, off-centre markers on spinning balls, gap-filling, the reader, the parabola
fit, a tilted Z axis, arc-top truncation). A string pendulum with a tape-measured
1285 mm length then gave ``g = 9.8413`` (+0.35%) from the SAME session, which puts
the **capture clock 59 sigma away from being the cause**.

That leaves an arithmetic identity with exactly two free terms:

    g_apparent / g_true  =  f'  *  (dt_true / dt_assumed)^2
                            ^^      ^^^^^^^^^^^^^^^^^^^^^
                     vertical scale        clock

A ball gives one number, so it cannot separate them. **An actuator can**, because
its encoder is an independent standard for position AND time at once. Marker on the
end-effector tip, actuator mounted VERTICALLY in the band the balls fly through,
QTM recording, this script logging the encoder.

WHAT EACH SEGMENT MEASURES (this is the whole design)
-----------------------------------------------------
Four of the five observables are blind to one hypothesis each, so they separate it;
the fifth must equal the product of the others, which is the consistency check.

===================  ==================  ==========================================
segment              isolates            reads 1.000 if that hypothesis is false
===================  ==================  ==========================================
``sync`` (x2)        CLOCK only          interval between the two sync bursts,
                                         mocap seconds / encoder seconds. Model-
                                         free: no length, no fit, no profile
                                         assumption. Over 300 s a 1.3% clock error
                                         is 3.8 s of divergence against ~3 ms of
                                         edge-location noise.
``stair``            SCALE only          d(z_mocap)/d(z_encoder) as a function of
                                         HEIGHT. The screw's mm-per-rev is an
                                         unknown CONSTANT, so it cancels out of any
                                         VARIATION with height -- and variation is
                                         the entire hypothesis. Each dwell averages
                                         ~750 frames, so each landing is located to
                                         ~1 um against a 5% error worth 10 mm.
``sine_long``        CLOCK only          measured frequency / commanded frequency.
                                         Needs no length at all.
``sine_local``       SCALE only          amplitude ratio at five heights -> the same
                                         f'(z) map as ``stair``, but dynamic, so a
                                         static-only artefact cannot fake both.
``accel``            BOTH (the product)  constant-acceleration strokes, fitted with
                                         the SAME parabola code the ball flights
                                         use. Closes the loop end-to-end on a known
                                         truth. Must equal scale x clock^2.
===================  ==================  ==========================================

Up AND down staircases run, so backlash and hysteresis show up as a loop rather
than contaminating the scale.

WHAT IS AND IS NOT THE TRUTH HERE
---------------------------------
**The encoder is the reference; the commanded profile is NOT.** The bench path
streams 40 Hz position knots through the firmware's 500 Hz Hermite interpolator and
a 0.10 rev lead clamp, so what executes is never exactly what was asked for. That is
fine and by design: every comparison in the table above is mocap-vs-ENCODER, and the
commanded profile only has to put the leg in interesting places. ``cmd_rev`` is
logged anyway (sample-and-hold of the latest knot) so tracking error is visible, and
``lead_clamp`` flags any interval where the clamp engaged and the encoder is not
following the command.

THE EXTRA COLUMN THAT MATTERS
-----------------------------
``bench_leg_sysid.py``'s logger timestamps each row with the Jetson's
``perf_counter()`` at UDP RECEIPT, and discards ``Telemetry.t_teensy_us``. This probe
logs BOTH. ``t_teensy_us`` is the can-bridge Teensy's own microsecond hardware clock,
sampled at the source: no UDP jitter, no Jetson scheduling, and a *third* independent
crystal to cross-check the other two against. For a measurement whose entire subject
is whether a clock is trustworthy, throwing that away would be perverse.

(Neither clock is remotely marginal for this: a Teensy or Jetson crystal is good to
tens of ppm, and the effect under test is 12 600 ppm. They only have to not be wrong
by 1.3%, and the two of them agreeing proves that between them.)

SAFETY
------
Every knot is bounded three ways before it leaves this script, and the profile is
re-validated as a whole by ``validate_profile()`` before ANY of it is streamed:

* absolute position inside the usable stroke window (stroke cap minus both margins);
* knot-to-knot step under ``vel_frac`` x the firmware lead clamp, so the clamp never
  engages during smooth motion;
* velocity and acceleration under the bench rig's brake-resistor caps.

The firmware guards (MAX_DEVIATION E-STOP, MPC staleness) remain the backstop, not
the primary protection. Everything hardware-facing -- transport, arming, the guard
state machine, homing, bring-up, the abort paths -- is INHERITED UNCHANGED from
``BridgeSysID`` in ``tests/hardware/bench_leg_sysid.py``, which has been run on this
rig. This probe adds a profile, two CSV columns and a manifest. It reimplements no
safety logic, deliberately.

USAGE
-----
    source ~/Desktop/PDJ_venv/venv/bin/activate

    # 1. Look at the plan without touching hardware (do this first, always).
    python tests/hardware/mocap_timebase_bench.py --dry-run

    # 2. Bounds/velocity/accel self-check of the generated profile. No hardware.
    python tests/hardware/mocap_timebase_bench.py --self-test

    # 3. Live. START THE QTM CAPTURE FIRST, then run this; it prints a countdown.
    python tests/hardware/mocap_timebase_bench.py --home

    # Shorter (~2 min) version if QTM's capture length is limited:
    python tests/hardware/mocap_timebase_bench.py --quick --home

    # Subset of segments, e.g. just the clock evidence:
    python tests/hardware/mocap_timebase_bench.py --segments sync,sine_long,sync

OUTPUT
------
``temp/probes/mocap_timebase_<ts>/`` containing

* ``telemetry.csv`` -- one row per received telemetry frame (~100 Hz), columns
  ``t_s, t_teensy_us, t_wall_s, cmd_rev, pos_rev, vel_rps, iq_A, fault_state,
  lead_clamp, uptime_ms, hb_flags``. ``t_s`` = Jetson perf_counter since streaming
  began; ``t_teensy_us`` = the Teensy's own clock; ``t_wall_s`` = Jetson
  ``time.time()``, for coarse alignment against the ``.qtm`` header's recorded
  wall-clock start. Differencing ``t_teensy_us`` against ``t_s`` over the run is
  itself a clock cross-check that costs nothing.
* ``profile.csv`` -- the commanded 40 Hz knot series with its segment label, so the
  analysis can slice without re-deriving anything.
* ``manifest.json`` -- caps, bounds, gains, every segment's exact parameters
  (commanded frequencies, amplitudes, accelerations, dwell times) and its knot-index
  span, the measured telemetry rate, and the clock cross-check.

Exit codes: 0 normal, 1 pre-flight rejection, 2 mid-run fault/abort.

BEFORE YOU RUN IT
-----------------
See ``--dry-run``'s checklist. In short: the bench leg must be the SOLE wire
authority (no ROS2 ``teensy_bridge_node`` running, or the arm is overridden), the
leg must be mounted vertically with the marker on the tip and visible to QTM through
the WHOLE stroke, and the QTM capture must start before this script and stop after.
"""
from __future__ import annotations

import argparse
import csv
import datetime
import math
import os
import sys
import time
from dataclasses import asdict, dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))       # tests/hardware
_PROJECT_ROOT = os.path.dirname(os.path.dirname(_SCRIPT_DIR))  # repo root
sys.path.insert(0, _SCRIPT_DIR)
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

import sysid_lib as sid                                    # noqa: E402
from bench_leg_sysid import (                              # noqa: E402
    BASELINE_GAINS,
    BRIDGE_LEAD_MARGIN_FRAC,
    BRIDGE_MAX_LEAD_REV,
    BRIDGE_SEG_T_S,
    BRIDGE_SETPOINT_HZ,
    DEFAULT_VEL_CAP_RPS,
    HARD_ACCEL_CAP_RPS2,
    HARD_CURRENT_LIMIT_A,
    HARD_STROKE_CAP_REV,
    HARD_VEL_CAP_RPS,
    STROKE_MARGIN_REV,
    BridgeSysID,
    hw_mm_per_rev,
)

# 71.5708 — single_leg_test.py:111 (a bare constant commented "measured"; no dataset
# in the repo backs it). Used ONLY to print human-readable mm: every command and every
# clamp in this script is in rev, so a wrong value here cannot cause a bad motion. It is
# an analysis risk, not a safety one — and this run is arguably its first independent
# check, since the mocap measures the tip's travel in metres directly.
MM_PER_REV = hw_mm_per_rev()

# Fraction of the lead-clamp frame step any smooth motion is allowed to use. The
# clamp engages at 0.10 rev/knot; the harness's own approach ramps run at 0.5x that
# (2.0 rev/s at 40 Hz). Riding the clamp would make the encoder stop following the
# command, which is exactly the regime this measurement must avoid: it needs SMOOTH,
# WELL-TRACKED motion, not aggressive tracking-error excitation.
DEFAULT_VEL_FRAC = 0.8         # of the 0.5-margin ramp step => 1.6 rev/s

# Segment defaults. Frequencies are deliberately ROUND and amplitudes are derived
# from the velocity budget, not the other way round: the commanded frequency is a
# primary observable and wants to be exactly stateable, whereas the amplitude is
# only ever used as a ratio against the encoder's own amplitude.
DEFAULT_SYNC_TRAVERSES = 4     # full-width triangle traverses per sync burst
DEFAULT_SYNC_AMP_REV = 0.50
DEFAULT_SYNC_DWELL_S = 0.60    # flat either side of each corner -> a sharp kink
DEFAULT_STAIR_LEVELS = 13
DEFAULT_STAIR_DWELL_S = 2.50
DEFAULT_SINE_LONG_HZ = 0.20
DEFAULT_SINE_LONG_S = 150.0    # 30 cycles
DEFAULT_SINE_LOCAL_HZ = 0.80
DEFAULT_SINE_LOCAL_S = 20.0    # 16 cycles at each height
DEFAULT_SINE_LOCAL_CENTRES = 5
DEFAULT_ACCEL_STROKES = 10     # 5 up + 5 down
DEFAULT_SETTLE_S = 0.75        # flat hold between segments
SINE_TAPER_S = 2.0             # raised-cosine envelope in/out (excluded in analysis)

SEGMENT_ORDER = ('sync', 'stair', 'sine_long', 'sine_local', 'accel', 'sync')
KNOWN_SEGMENTS = ('sync', 'stair', 'sine_long', 'sine_local', 'accel')

QUICK_OVERRIDES = dict(stair_levels=7, stair_dwell_s=1.5, sine_long_s=45.0,
                       sine_local_s=10.0, sine_local_centres=3, accel_strokes=4)


# ---------------------------------------------------------------------------
# Pure profile generation. No hardware, no I/O — testable with --self-test.
# ---------------------------------------------------------------------------

@dataclass
class Segment:
    """One labelled span of the 40 Hz knot series, with what it measures."""
    name: str
    kind: str
    knot_start: int
    knot_end: int                      # exclusive
    params: Dict[str, float] = field(default_factory=dict)

    @property
    def n_knots(self) -> int:
        return self.knot_end - self.knot_start

    def duration_s(self, seg_t: float) -> float:
        return self.n_knots * seg_t

    def t_start_s(self, seg_t: float) -> float:
        return self.knot_start * seg_t


class ProfileBuilder:
    """Accumulates 40 Hz knots, keeping every step inside the lead-clamp budget."""

    def __init__(self, start_rev: float, frame_step: float,
                 bounds: sid.StrokeBounds, seg_t: float):
        self.frame_step = float(frame_step)
        self.bounds = bounds
        self.seg_t = float(seg_t)
        self.knots: List[float] = [float(start_rev)]
        self.segments: List[Segment] = []
        self._open: Optional[Tuple[str, str, int, Dict[str, float]]] = None

    # -- segment bracketing --------------------------------------------------

    def begin(self, name: str, kind: str, **params) -> None:
        assert self._open is None, "segment %r still open" % (self._open,)
        self._open = (name, kind, len(self.knots), dict(params))

    def end(self) -> None:
        assert self._open is not None, "no open segment"
        name, kind, start, params = self._open
        self.segments.append(Segment(name, kind, start, len(self.knots), params))
        self._open = None

    # -- primitives ----------------------------------------------------------

    @property
    def current(self) -> float:
        return self.knots[-1]

    def hold(self, seconds: float) -> None:
        for _ in range(int(round(seconds / self.seg_t))):
            self.knots.append(self.current)

    def ramp_to(self, target: float) -> None:
        """Straight-line ramp at <= frame_step per knot. Index 0 is NOT repeated."""
        target = self.bounds.clamp(float(target))
        start = self.current
        n = sid.knot_ramp_frames(start, target, self.frame_step)
        for i in range(1, n + 1):
            self.knots.append(start + (target - start) * (i / n))

    def extend(self, values: Sequence[float]) -> None:
        for v in values:
            self.knots.append(self.bounds.clamp(float(v)))

    # -- the segments themselves --------------------------------------------

    def sync_burst(self, centre: float, amp: float, traverses: int,
                   dwell_s: float, label: str) -> None:
        """Triangle traverses with a flat hold at every corner.

        The corner is the point of the exercise. A velocity reversal preceded and
        followed by a dead stop gives BOTH systems a kink they can localise by
        fitting the two straight legs and intersecting them — sub-sample in mocap
        and sub-knot in the encoder, with no dependence on either system's
        amplitude calibration. Two of these bursts, one at each end of the run,
        bracket the whole record: mocap seconds between them over encoder seconds
        between them IS the clock ratio, with no model in between.
        """
        amp = min(amp, sid.symmetric_amplitude_limit(centre, self.bounds))
        self.begin(label, 'sync', amplitude_rev=amp, traverses=traverses,
                   dwell_s=dwell_s, ramp_vel_rps=self.frame_step / self.seg_t)
        self.ramp_to(centre)
        self.hold(dwell_s)
        sign = 1.0
        for _ in range(traverses):
            self.ramp_to(centre + sign * amp)
            self.hold(dwell_s)
            sign = -sign
        self.ramp_to(centre)
        self.hold(dwell_s)
        self.end()

    def staircase(self, levels: Sequence[float], dwell_s: float,
                  label: str) -> None:
        """Dwell at each level; the ratio of mocap to encoder STEP SIZES is f'(z).

        Equal steps in encoder space are the point: an unknown constant mm-per-rev
        multiplies every step identically and so cancels from any variation across
        heights, which is the signal. Only a scale that CHANGES with height can
        make these ratios differ from each other.
        """
        self.begin(label, 'stair', n_levels=len(levels), dwell_s=dwell_s,
                   lo_rev=float(min(levels)), hi_rev=float(max(levels)),
                   step_rev=float(abs(levels[1] - levels[0])) if len(levels) > 1 else 0.0)
        for lv in levels:
            self.ramp_to(lv)
            self.hold(dwell_s)
        self.end()

    def sine(self, centre: float, freq_hz: float, v_max: float, duration_s: float,
             label: str, taper_s: float = SINE_TAPER_S) -> None:
        """Tapered sine at an EXACT commanded frequency; amplitude from the budget.

        The taper is a raised cosine over the first and last ``taper_s``, so the
        leg is never asked for a velocity step at the start (a bare sine has peak
        velocity at t=0). Analysis excludes the tapered ends — the manifest records
        ``taper_s`` so it knows where they are.
        """
        amp = min(v_max / (2.0 * math.pi * freq_hz),
                  sid.symmetric_amplitude_limit(centre, self.bounds))
        n = int(round(duration_s / self.seg_t))
        t = np.arange(n) * self.seg_t
        env = np.ones(n)
        if taper_s > 0.0:
            rise = t < taper_s
            env[rise] = 0.5 * (1.0 - np.cos(math.pi * t[rise] / taper_s))
            fall = t > (t[-1] - taper_s) if n else np.zeros(0, bool)
            env[fall] = 0.5 * (1.0 - np.cos(math.pi * (t[-1] - t[fall]) / taper_s))
        self.begin(label, 'sine', centre_rev=centre, freq_hz=freq_hz,
                   amplitude_rev=amp, duration_s=n * self.seg_t, taper_s=taper_s,
                   peak_vel_rps=2.0 * math.pi * freq_hz * amp,
                   n_cycles=freq_hz * n * self.seg_t)
        self.ramp_to(centre)
        self.extend(centre + amp * env * np.sin(2.0 * math.pi * freq_hz * t))
        self.ramp_to(centre)
        self.end()

    def accel_strokes(self, lo: float, hi: float, n_strokes: int, v_max: float,
                      label: str) -> None:
        """Constant-|acceleration| strokes: accelerate to mid-travel, decelerate.

        This is the end-to-end closure. Fitting a parabola to the interior of one
        half-stroke is arithmetically the same operation the ball pipeline performs,
        so it is corrupted by the same product of scale and clock-squared. Fit the
        encoder and the mocap over the same window and the ratio must equal
        (scale) x (clock ratio)^2 measured by the other segments. If it does not,
        the cause is something in NEITHER hypothesis.

        The commanded acceleration is ~0.7% of g — the lead clamp caps the tip at
        ~1.6 rev/s and the bench stroke is only ~2.7 rev — but absolute magnitude
        is irrelevant here: the ratio is what is measured, and mocap noise of
        0.028 mm over a 1.6 s half-stroke resolves it to far better than 0.01%.

        Nor does the servo's transport delay matter, which is the obvious
        objection. Both a fixed lag and the telemetry frame's 0-10 ms sample age
        translate the trace along the time axis, and a parabola translated in time
        is the same parabola with the same second derivative. Only a SCALE error on
        the time axis changes a fitted acceleration — which is precisely the
        quantity under test, and precisely why this segment is worth running.
        """
        travel = hi - lo
        # Pick the half-stroke's KNOT COUNT from the velocity budget first, then
        # derive the acceleration from that integer. Doing it the other way round —
        # a = v_max^2/travel, then n_half = round(sqrt(travel/a)/seg_t) — leaves the
        # accelerating half short of mid-travel while the decelerating half is
        # back-parameterised from the end, so they do not meet: a single knot step of
        # ~2x the budget appears at the join. --self-test caught exactly that, at
        # every stroke cap whose travel was not a clean multiple of the step.
        # ceil (not round) so the derived acceleration is always UNDER budget.
        n_half = int(math.ceil(travel / (v_max * self.seg_t)))
        half_t = n_half * self.seg_t
        a = travel / half_t ** 2                      # travel/2 = 1/2 a half_t^2
        v_peak = a * half_t
        self.begin(label, 'accel', lo_rev=lo, hi_rev=hi, n_strokes=n_strokes,
                   accel_rps2=a, accel_m_s2=a * MM_PER_REV / 1000.0,
                   peak_vel_rps=v_peak, stroke_s=2.0 * half_t,
                   half_stroke_s=half_t, travel_rev=travel)
        self.ramp_to(lo)
        self.hold(DEFAULT_SETTLE_S)
        up = True
        for _ in range(n_strokes):
            base, target = (lo, hi) if up else (hi, lo)
            direction = 1.0 if up else -1.0
            for i in range(1, 2 * n_half + 1):
                tt = i * self.seg_t
                if i <= n_half:
                    x = 0.5 * a * tt ** 2
                else:
                    t_rem = (2 * n_half - i) * self.seg_t
                    x = travel - 0.5 * a * t_rem ** 2
                self.knots.append(self.bounds.clamp(base + direction * x))
            self.knots[-1] = self.bounds.clamp(target)
            self.hold(DEFAULT_SETTLE_S)
            up = not up
        self.end()


@dataclass
class ProfileSpec:
    """Everything that shapes the commanded profile. Recorded in the manifest."""
    stroke_cap_rev: float = HARD_STROKE_CAP_REV
    margin_rev: float = STROKE_MARGIN_REV
    centre_rev: Optional[float] = None
    vel_frac: float = DEFAULT_VEL_FRAC
    seg_t_s: float = BRIDGE_SEG_T_S
    segments: Tuple[str, ...] = SEGMENT_ORDER
    sync_traverses: int = DEFAULT_SYNC_TRAVERSES
    sync_amp_rev: float = DEFAULT_SYNC_AMP_REV
    sync_dwell_s: float = DEFAULT_SYNC_DWELL_S
    stair_levels: int = DEFAULT_STAIR_LEVELS
    stair_dwell_s: float = DEFAULT_STAIR_DWELL_S
    sine_long_hz: float = DEFAULT_SINE_LONG_HZ
    sine_long_s: float = DEFAULT_SINE_LONG_S
    sine_local_hz: float = DEFAULT_SINE_LOCAL_HZ
    sine_local_s: float = DEFAULT_SINE_LOCAL_S
    sine_local_centres: int = DEFAULT_SINE_LOCAL_CENTRES
    accel_strokes: int = DEFAULT_ACCEL_STROKES

    @property
    def bounds(self) -> sid.StrokeBounds:
        return sid.stroke_bounds(self.stroke_cap_rev, self.margin_rev)

    @property
    def centre(self) -> float:
        return (self.stroke_cap_rev / 2.0 if self.centre_rev is None
                else self.centre_rev)

    @property
    def v_max_rps(self) -> float:
        """Velocity budget: vel_frac of the harness's 0.5-margin ramp velocity.

        Held CONSTANT across ``--knot-hz``, which is the harness's own convention
        (``BRIDGE_RAMP_VEL_TARGET_RPS``: "design velocity the 0.5 margin implies at
        40 Hz knots — held across --knot-hz"). ``lead_clamp_frame_step`` returns a
        per-knot increment quoted for 40 Hz, so treating it as a step rather than a
        velocity would make a 100 Hz stream 2.5x faster for the same numbers — and
        the sync bursts' velocity reversals would then demand 400 rev/s^2, past the
        250 rev/s^2 brake-resistor cap. Which is exactly what validate_profile said
        the first time this was tried.
        """
        ramp_step_40hz = sid.lead_clamp_frame_step(BRIDGE_MAX_LEAD_REV,
                                                   BRIDGE_LEAD_MARGIN_FRAC)
        return self.vel_frac * ramp_step_40hz / BRIDGE_SEG_T_S

    @property
    def frame_step_rev(self) -> float:
        """Max knot-to-knot increment implied by the velocity budget at this rate."""
        return self.v_max_rps * self.seg_t_s


def build_profile(spec: ProfileSpec, start_rev: float
                  ) -> Tuple[np.ndarray, List[Segment]]:
    """The commanded 40 Hz knot series and its segment map. Pure."""
    b = spec.bounds
    centre = spec.centre
    if not b.contains(centre):
        raise ValueError("centre %.3f rev outside usable bounds [%.3f, %.3f]"
                         % (centre, b.lo_rev, b.hi_rev))
    pb = ProfileBuilder(start_rev, spec.frame_step_rev, b, spec.seg_t_s)
    # Keep the extremes of every stimulus one lead-clamp step clear of the usable
    # bound, so even a full-step overshoot at a turnaround stays inside it.
    pad = spec.frame_step_rev
    lo, hi = b.lo_rev + pad, b.hi_rev - pad
    v = spec.v_max_rps
    n_sync = 0

    for name in spec.segments:
        if name == 'sync':
            n_sync += 1
            pb.sync_burst(centre, spec.sync_amp_rev, spec.sync_traverses,
                          spec.sync_dwell_s, 'sync_%d' % n_sync)
        elif name == 'stair':
            levels = list(np.linspace(lo, hi, spec.stair_levels))
            pb.staircase(levels, spec.stair_dwell_s, 'stair_up')
            pb.staircase(levels[::-1], spec.stair_dwell_s, 'stair_down')
        elif name == 'sine_long':
            pb.sine(centre, spec.sine_long_hz, v, spec.sine_long_s, 'sine_long')
        elif name == 'sine_local':
            # Amplitude is uniform across centres so the five f'(z) samples are
            # directly comparable; the outermost centres set what fits.
            k = spec.sine_local_centres
            centres = list(np.linspace(lo, hi, k + 2))[1:-1] if k > 1 else [centre]
            room = min(min(c - b.lo_rev, b.hi_rev - c) for c in centres) - pad
            amp_cap = min(v / (2.0 * math.pi * spec.sine_local_hz), room)
            v_eff = 2.0 * math.pi * spec.sine_local_hz * amp_cap
            for i, c in enumerate(centres):
                pb.sine(c, spec.sine_local_hz, v_eff, spec.sine_local_s,
                        'sine_local_%d' % (i + 1))
        elif name == 'accel':
            pb.accel_strokes(lo, hi, spec.accel_strokes, v, 'accel')
        else:
            raise ValueError("unknown segment %r (known: %s)"
                             % (name, ', '.join(KNOWN_SEGMENTS)))
        pb.hold(DEFAULT_SETTLE_S)

    pb.ramp_to(centre)
    pb.hold(DEFAULT_SETTLE_S)
    return np.asarray(pb.knots, float), pb.segments


def validate_profile(series: np.ndarray, spec: ProfileSpec,
                     vel_cap_rps: float) -> List[str]:
    """Every reason this profile must not be streamed. Empty list == safe.

    Checked as a WHOLE, after generation, rather than trusting each builder: a
    profile is a few thousand numbers and there is no excuse for not looking at
    all of them before any reach the wire.
    """
    problems: List[str] = []
    b = spec.bounds
    if series.size < 2:
        return ["profile is empty"]
    if not np.all(np.isfinite(series)):
        problems.append("profile contains %d non-finite knots"
                        % int((~np.isfinite(series)).sum()))
        return problems

    # The first knot is the leg's LIVE encoder position, which can legitimately sit
    # outside the usable window — post-``--home`` the leg rests at the retracted
    # hardstop, below ``lo``, because the margin backs off the end-stop. Commanding
    # where the leg physically is cannot crash it, so the near side is relaxed to
    # include it while the far side still caps at the stroke bound. Same reasoning
    # and same relaxation as ``sysid_lib.knot_step_ramp``; without it a run that
    # started from the hardstop would be refused for being exactly where it is.
    start = float(series[0])
    lo_ok, hi_ok = min(b.lo_rev, start), max(b.hi_rev, start)
    lo, hi = float(series.min()), float(series.max())
    if lo < lo_ok - 1e-9 or hi > hi_ok + 1e-9:
        problems.append("profile spans [%.4f, %.4f] rev, outside the usable window "
                        "[%.4f, %.4f] (start %.4f)"
                        % (lo, hi, b.lo_rev, b.hi_rev, start))

    steps = np.abs(np.diff(series))
    max_step = float(steps.max())
    if max_step > spec.frame_step_rev + 1e-9:
        i = int(np.argmax(steps))
        problems.append("max knot step %.5f rev at index %d exceeds the budget "
                        "%.5f rev (%.1f%% of the %.2f rev lead clamp)"
                        % (max_step, i, spec.frame_step_rev,
                           100.0 * max_step / BRIDGE_MAX_LEAD_REV,
                           BRIDGE_MAX_LEAD_REV))
    if max_step > BRIDGE_MAX_LEAD_REV:
        problems.append("max knot step %.5f rev EXCEEDS the firmware lead clamp "
                        "%.3f rev — the encoder cannot follow this"
                        % (max_step, BRIDGE_MAX_LEAD_REV))

    v = steps / spec.seg_t_s
    max_v = float(v.max())
    if max_v > vel_cap_rps + 1e-9:
        problems.append("peak commanded velocity %.3f rev/s exceeds the session cap "
                        "%.3f rev/s" % (max_v, vel_cap_rps))
    if max_v > HARD_VEL_CAP_RPS:
        problems.append("peak commanded velocity %.3f rev/s exceeds the HARD cap "
                        "%.1f rev/s" % (max_v, HARD_VEL_CAP_RPS))

    if v.size >= 2:
        a = np.abs(np.diff(v)) / spec.seg_t_s
        max_a = float(a.max())
        if max_a > HARD_ACCEL_CAP_RPS2:
            problems.append("peak commanded acceleration %.1f rev/s^2 exceeds the "
                            "HARD cap %.1f rev/s^2 (brake resistor)"
                            % (max_a, HARD_ACCEL_CAP_RPS2))
    return problems


# ---------------------------------------------------------------------------
# The runner. Everything hardware-facing is inherited from BridgeSysID.
# ---------------------------------------------------------------------------

CSV_COLUMNS = ('t_s', 't_teensy_us', 't_wall_s', 'cmd_rev', 'pos_rev', 'vel_rps',
               'iq_A', 'fault_state', 'lead_clamp', 'uptime_ms', 'hb_flags')

# HeartbeatT2JFlags (config/generated/udp_protocol.py:112-116).
HB_TIME_SYNCED = 1    # bit0 — the Teensy clock is anchored to the Jetson
HB_MPC_ACTIVE = 8     # bit3 — the firmware agrees this stream is armed


class MocapTimebaseProbe(BridgeSysID):
    """``BridgeSysID`` plus the Teensy timestamp and one long profiled stream.

    Overrides exactly two inherited methods, both logging-only:

    * ``_open_csv``   — the wider header.
    * ``_log_frame``  — writes ``t_teensy_us`` and wall time as well. It still
      appends the SAME 7-tuple to ``self._log_buf`` in the base class's column
      order, so the inherited ``_end_telem_log`` mapping stays correct. Do not
      "tidy" that into a 9-tuple without changing ``_end_telem_log`` too.

    No safety, transport, arming or guard behaviour is altered.
    """

    def __init__(self, *, spec: ProfileSpec, **kw):
        super().__init__(**kw)
        self.spec = spec
        self._profile: Optional[np.ndarray] = None
        self._segments: List[Segment] = []
        self._manifest['probe'] = {
            'purpose': 'mocap vertical-scale and capture-clock cross-check',
            'mm_per_rev': MM_PER_REV,
            'csv_columns': list(CSV_COLUMNS),
            'spec': asdict(spec),
            'frame_step_rev': spec.frame_step_rev,
            'v_max_rps': spec.v_max_rps,
        }

    # -- logging overrides ---------------------------------------------------

    def _open_csv(self, name: str):
        os.makedirs(self.output_dir, exist_ok=True)
        path = os.path.join(self.output_dir, name)
        f = open(path, 'w', newline='')
        w = csv.writer(f)
        w.writerow(list(CSV_COLUMNS))
        return f, w, path

    def _log_frame(self, tm, now):
        ax = self.axis_id
        t_rel = now - self._log_t0
        u0 = self._log_u0
        try:
            pos = float(tm.pos_rev[ax])
            vel = float(tm.vel_rps[ax])
        except Exception:  # noqa: BLE001 — short/garbled frame
            pos = float('nan')
            vel = float('nan')
        try:
            t_teensy = int(tm.t_teensy_us)
        except Exception:  # noqa: BLE001
            t_teensy = None
        d = self._cache['diag'].get(ax)
        iq = float(d.iq_measured) if d is not None else float('nan')
        hb = self._cache['hb']
        fs = int(hb.fault_state) if hb is not None else None
        lc = None
        uptime = None
        hb_flags = None
        if hb is not None:
            try:
                lc = int((int(hb.lead_clamp_mask) >> ax) & 1)
            except Exception:  # noqa: BLE001 — older/short heartbeat
                lc = None
            # uptime_ms and flags are logged because a timing measurement whose
            # subject is a clock cannot afford to assume the instrument's clock is
            # stationary. logbook/2026-07-18-teensy-uptime-tracking-degradation.md
            # (status OPEN) measured setpoint->encoder tracking lag growing
            # MONOTONICALLY with can-bridge Teensy uptime: 10 ms at 1.8 min, 250 ms
            # at 24 h, resetting on reboot, cause unknown. That particular lag is
            # command-to-motion and so cannot corrupt a mocap-vs-ENCODER comparison
            # — but an unexplained uptime-dependent effect in a timing-critical
            # firmware path is exactly the thing to record rather than assume away.
            # flags bit0 (TIME_SYNCED) says whether t_teensy_us is anchored at all.
            try:
                uptime = int(hb.uptime_ms)
                hb_flags = int(hb.flags)
            except Exception:  # noqa: BLE001
                uptime = hb_flags = None
        # BASE COLUMN ORDER — _end_telem_log unpacks positionally. See class docstring.
        self._log_buf.append((t_rel, u0, pos, vel, iq,
                              float('nan') if fs is None else float(fs),
                              float('nan') if lc is None else float(lc)))
        if self._log_writer is not None:
            self._log_writer.writerow([
                "%.5f" % t_rel,
                "" if t_teensy is None else t_teensy,
                "%.6f" % time.time(),
                "%.6f" % u0,
                "" if pos != pos else "%.6f" % pos,
                "" if vel != vel else "%.6f" % vel,
                "" if iq != iq else "%.4f" % iq,
                "" if fs is None else int(fs),
                "" if lc is None else int(lc),
                "" if uptime is None else uptime,
                "" if hb_flags is None else hb_flags])

    # -- the stage -----------------------------------------------------------

    def _write_profile_csv(self, series: np.ndarray,
                           segments: List[Segment]) -> str:
        os.makedirs(self.output_dir, exist_ok=True)
        path = os.path.join(self.output_dir, 'profile.csv')
        label = np.array(['transition'] * series.size, dtype=object)
        for s in segments:
            label[s.knot_start:s.knot_end] = s.name
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['knot', 't_s', 'cmd_rev', 'cmd_mm', 'segment'])
            for i, u in enumerate(series):
                w.writerow([i, "%.4f" % (i * self.seg_t_s), "%.6f" % u,
                            "%.4f" % (u * MM_PER_REV), label[i]])
        return path

    def stage_mocap(self) -> bool:
        print("\n=== Mocap time-base + vertical-scale profile ===")
        # The knot period is compile-time on the Teensy (SEGMENT_T_S) and invisible
        # from this repo: stock is 40 Hz knots / 100 Hz telemetry, the
        # BENCH_SYSID_BUILD is 100 Hz / 250 Hz. Streaming 40 Hz knots into a 100 Hz
        # build distorts the interpolated velocity profile. The parent measured the
        # telemetry rate during bring-up, which is the only observable that tells
        # them apart, so surface the inference rather than assuming.
        if self.telem_rate is not None and self.telem_rate.effective_hz > 150.0:
            print("  ** telemetry measured at %.0f Hz, well above the stock 100 Hz —\n"
                  "  ** this Teensy is probably running BENCH_SYSID_BUILD, whose knot\n"
                  "  ** period is 10 ms, not the %.0f ms this profile is built for.\n"
                  "  ** Re-run with --knot-hz 100 or reflash before trusting the\n"
                  "  ** commanded frequencies."
                  % (self.telem_rate.effective_hz, self.seg_t_s * 1000.0))
        entry = self._enter_hold_at_center()
        if entry['aborted'] or entry['guard_latched']:
            self._disarm()
            print("  ABORT during approach: %s"
                  % (self._abort_reason or 'guard latch'))
            return False

        pos, _, _, _ = self._sample()
        start = pos if pos is not None else self.center_rev
        series, segments = build_profile(self.spec, start)
        problems = validate_profile(series, self.spec, self.vel_cap_rps)
        if problems:
            # Generation happens after the approach because it starts from the LIVE
            # encoder position, so this is the last possible moment to refuse — and
            # it still refuses before a single profile knot is sent.
            self._disarm()
            for p in problems:
                print("  REJECT: %s" % p, file=sys.stderr)
            self._abort_reason = "profile failed validation"
            return False
        self._profile, self._segments = series, segments

        prof_path = self._write_profile_csv(series, segments)
        total_s = series.size * self.seg_t_s
        print("  %d knots, %.1f s (%.1f min), span %.3f-%.3f rev "
              "(%.1f-%.1f mm, %.1f mm of travel)"
              % (series.size, total_s, total_s / 60.0, series.min(), series.max(),
                 series.min() * MM_PER_REV, series.max() * MM_PER_REV,
                 (series.max() - series.min()) * MM_PER_REV))
        print("  peak commanded velocity %.3f rev/s (%.0f mm/s), max knot step "
              "%.4f rev = %.0f%% of the %.2f rev lead clamp"
              % (np.abs(np.diff(series)).max() / self.seg_t_s,
                 np.abs(np.diff(series)).max() / self.seg_t_s * MM_PER_REV,
                 np.abs(np.diff(series)).max(),
                 100.0 * np.abs(np.diff(series)).max() / BRIDGE_MAX_LEAD_REV,
                 BRIDGE_MAX_LEAD_REV))
        print("  profile → %s" % prof_path)

        f, w, path = self._open_csv('telemetry.csv')
        print("  telemetry → %s" % path)
        print("\n  >>> streaming now — leave QTM recording until this says DONE <<<\n")
        t_start_wall = time.time()
        t_start_perf = time.perf_counter()
        arrays = self._stream_and_sample(series, guard=True, writer=w,
                                         revert_gains=BASELINE_GAINS)
        t_end_perf = time.perf_counter()
        f.close()
        self._disarm()

        elapsed = t_end_perf - t_start_perf
        n_rows = int(arrays['t'].size)
        self._manifest['probe']['run'] = {
            'started_wall_iso': datetime.datetime.fromtimestamp(
                t_start_wall).isoformat(timespec='milliseconds'),
            'started_wall_unix': t_start_wall,
            'commanded_duration_s': total_s,
            'elapsed_s': elapsed,
            'telemetry_rows': n_rows,
            'telemetry_mean_hz': (n_rows / elapsed) if elapsed > 0 else None,
            'profile_csv': os.path.basename(prof_path),
            'telemetry_csv': os.path.basename(path),
            'aborted': bool(arrays['aborted']),
            'guard_latched': bool(arrays['guard_latched']),
        }
        self._manifest['probe']['segments'] = [
            dict(name=s.name, kind=s.kind, knot_start=s.knot_start,
                 knot_end=s.knot_end, t_start_s=s.t_start_s(self.seg_t_s),
                 duration_s=s.duration_s(self.seg_t_s), params=s.params)
            for s in self._segments]

        if arrays['aborted'] or arrays['guard_latched']:
            diag = self._abort_diag(arrays)
            print("  ABORT after %.1f s of %.1f s: %s"
                  % (elapsed, total_s, diag or self._abort_reason or 'guard latch'))
            print("  The partial telemetry.csv is still usable — every segment that "
                  "COMPLETED is analysable, and the manifest records where it stopped.")
            return False

        self._report(arrays, elapsed, total_s)
        return True

    def _report(self, arrays: dict, elapsed: float, commanded: float) -> None:
        """Sanity numbers the operator can eyeball before leaving the bench."""
        t, cmd, posn = arrays['t'], arrays['cmd'], arrays['pos']
        good = np.isfinite(posn)
        print("\n  --- run summary ---")
        print("  commanded %.2f s, streamed %.2f s (%+.0f ms), %d telemetry rows "
              "(%.1f Hz)" % (commanded, elapsed, (elapsed - commanded) * 1e3,
                             t.size, t.size / elapsed if elapsed > 0 else 0.0))
        if good.sum():
            travel = (np.nanmax(posn[good]) - np.nanmin(posn[good]))
            err = np.abs(cmd[good] - posn[good])
            print("  encoder span %.3f-%.3f rev (%.1f mm of travel)"
                  % (np.nanmin(posn[good]), np.nanmax(posn[good]),
                     travel * MM_PER_REV))
            print("  |cmd - encoder|: median %.4f rev (%.2f mm), p99 %.4f rev, "
                  "max %.4f rev" % (np.median(err), np.median(err) * MM_PER_REV,
                                    np.percentile(err, 99), err.max()))
        lc = arrays.get('lead_clamp')
        if lc is not None and lc.size and np.any(np.isfinite(lc)):
            frac = float(np.nanmean(lc > 0.5))
            note = "" if frac < 0.01 else "   <-- CHECK: the encoder was not following"
            print("  lead clamp engaged on %.2f%% of frames%s" % (100.0 * frac, note))
        gaps = np.diff(t)
        if gaps.size:
            print("  telemetry gaps: median %.2f ms, p99 %.2f ms, max %.2f ms"
                  % (np.median(gaps) * 1e3, np.percentile(gaps, 99) * 1e3,
                     gaps.max() * 1e3))
        print("\n  DONE — you can stop the QTM capture now.")

    def run(self, modes: Optional[List[str]] = None) -> int:   # type: ignore[override]
        """Run the mocap stage through the PARENT's bring-up and teardown.

        ``BridgeSysID.run`` owns a sequence this probe must not fork: transport
        setup, waiting for first telemetry, measuring the telemetry rate, clearing a
        sticky startup guard latch, optional homing, manifest write and teardown —
        each of which exists because it failed on this rig at least once. Its stage
        dispatch is a local dict keyed by the stage names it knows, so the mocap
        stage is bound onto this INSTANCE under ``track`` (the closest analogue: one
        continuously-moving reference, one long stream) and the parent is called
        unchanged. Instance-level, so no other object's behaviour is touched.
        """
        self.stage_track = self.stage_mocap   # type: ignore[assignment]
        return super().run(['track'])


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

PREFLIGHT = """
  PRE-FLIGHT CHECKLIST — all of these matter to the measurement, not just safety
  ---------------------------------------------------------------------------
  RIG — get these wrong and the run is wasted or the leg is damaged
  [ ] WHICH PSU? On the 21 V supply the leg is back-EMF-marginal above ~1 rev/s
      (leg-gain-tuning-methodology.md; the methodology prescribes 48 V + brake
      resistor above that). This profile's default peak is 1.6 rev/s. On 21 V it
      will simply fail to track and probably latch the guard.
          -> if 21 V, run with  --vel-frac 0.4  (0.8 rev/s).
      No logbook entry records which supply is currently on the bench, so this
      cannot be checked from here. Look at it.
  [ ] ENCODER INDEX SEARCH has run since the ODrive was last powered. The ODrive
      config has startup_encoder_index_search=false, so a cold drive reports a
      non-finite position, every knot is dropped at ingest, and the run dies
      silently on the 250 ms staleness E-STOP with nothing to show for it.
          python tests/hardware/teensy_encoder_search_bench.py 0
  [ ] The leg EXTENDS UPWARD in its current mounting. Nothing in the code or
      config encodes the direction — it rests on operator-confirmation comments
      only, and the harness's engagement check verifies MAGNITUDE, not sign. If
      extension is downward this profile drives into the rig. WATCH the leg
      during the small verify nudge at the start and abort if it goes the wrong
      way.
  [ ] Note uptime_ms from the first log rows. (The "REBOOT the can-bridge Teensy
      first" step was RETIRED 2026-08-15: tracking lag used to grow monotonically
      with its uptime — 10 ms at 1.8 min, 250 ms at 24 h — and the cause turned
      out to be the vendored FlexCAN_T4 `_available` RX-ring leak, fixed in FW 14
      and validated at 5.8 h and 15.2 h of continuous uptime; logbook
      2026-07-18 is now RESOLVED, see 2026-08-15-fw14-validated-arc-closed.)
      Keep recording uptime_ms anyway — it is the label that would expose a
      regression, and this measurement is about a clock.
  [ ] The bench leg is the SOLE wire authority. No ROS2 teensy_bridge_node, no
      run_mpc.py, nothing else streaming heartbeats — mpc_active is a single
      firmware bool with no OR-ing of sources, so a second authority silently
      disarms this one mid-run (the script detects that and aborts loudly).

  GEOMETRY — this is what makes the measurement mean anything
  [ ] The leg is mounted VERTICALLY. A horizontal mounting tests the axis that
      four length checks have already shown to be fine, and answers nothing.
  [ ] The stroke sits as HIGH in the capture volume as the rig allows — ideally
      inside z = 0.9-1.7 m, where the balls actually fly. That is where the
      anomaly is hypothesised to live. If the ~193 mm of usable stroke cannot
      cover that band, run this at two or three mounting heights: a distortion
      fixed in LAB coordinates and a screw-pitch error fixed in ENCODER
      coordinates separate completely when the actuator moves and the profile
      does not.
  [ ] One marker on the end-effector tip, and it must stay visible and LABELLED
      for the whole stroke. A dropout in the middle of a dwell is survivable; a
      swapped identity between levels is not.
  [ ] A SECOND marker on the moving assembly, ~100 mm below the first, if you
      have a spare. Their measured separation should be constant as the pair
      sweeps; any variation is a free, encoder-independent check on the same
      hypothesis.
  [ ] QTM capture STARTED BEFORE this script and STOPPED AFTER it. Overlap on
      both ends is what the two sync bursts are for.
  [ ] Note the QTM take name so the two recordings can be paired.

  AFTERWARDS
  [ ] The leg is left DISARMED but still in CLOSED_LOOP, holding position — this
      script deliberately does NOT command IDLE at teardown, because on a
      VERTICAL leg that drops holding torque and lets the assembly settle under
      gravity onto its own transmission. Power it down the usual way.
"""


def _parse_segments(s: str) -> Tuple[str, ...]:
    out = tuple(x.strip() for x in s.split(',') if x.strip())
    bad = [x for x in out if x not in KNOWN_SEGMENTS]
    if bad:
        raise argparse.ArgumentTypeError(
            "unknown segment(s) %s (known: %s)"
            % (', '.join(bad), ', '.join(KNOWN_SEGMENTS)))
    if not out:
        raise argparse.ArgumentTypeError("no segments given")
    return out


def _default_output_dir() -> str:
    ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    return os.path.join(_PROJECT_ROOT, 'temp', 'probes', 'mocap_timebase_%s' % ts)


def print_plan(spec: ProfileSpec, vel_cap: float, out_dir: str) -> int:
    """Print the profile, its safety margins and the checklist. No hardware."""
    b = spec.bounds
    series, segments = build_profile(spec, spec.centre)
    problems = validate_profile(series, spec, vel_cap)
    steps = np.abs(np.diff(series))
    total_s = series.size * spec.seg_t_s

    print("=" * 78)
    print("MOCAP TIME-BASE + VERTICAL-SCALE PROBE — PLAN")
    print("=" * 78)
    print("\n  geometry")
    print("    mm per motor rev        %.4f" % MM_PER_REV)
    print("    stroke cap              %.2f rev  (%.1f mm)"
          % (spec.stroke_cap_rev, spec.stroke_cap_rev * MM_PER_REV))
    print("    usable window           %.3f-%.3f rev  (%.1f mm of travel)"
          % (b.lo_rev, b.hi_rev, b.width * MM_PER_REV))
    print("    operating centre        %.3f rev" % spec.centre)
    print("\n  velocity budget")
    print("    firmware lead clamp     %.3f rev/knot" % BRIDGE_MAX_LEAD_REV)
    ramp_40 = sid.lead_clamp_frame_step(BRIDGE_MAX_LEAD_REV, BRIDGE_LEAD_MARGIN_FRAC)
    print("    harness ramp target     %.1f rev/s at %.0f%% margin "
          "(%.4f rev/knot quoted at 40 Hz)"
          % (ramp_40 / BRIDGE_SEG_T_S, 100.0 * BRIDGE_LEAD_MARGIN_FRAC, ramp_40))
    print("    this probe (--vel-frac) %.2f rev/s = %.0f mm/s "
          "(%.4f rev/knot at %.0f Hz)"
          % (spec.v_max_rps, spec.v_max_rps * MM_PER_REV, spec.frame_step_rev,
             1.0 / spec.seg_t_s))
    print("    session velocity cap    %.1f rev/s   (hard cap %.1f)"
          % (vel_cap, HARD_VEL_CAP_RPS))

    print("\n  profile — %d knots at %.0f Hz = %.1f s (%.2f min)"
          % (series.size, 1.0 / spec.seg_t_s, total_s, total_s / 60.0))
    print("    %-14s %8s %8s %9s   %s"
          % ('segment', 't_start', 'dur', 'knots', 'what it measures / parameters'))
    what = {
        'sync': 'CLOCK (model-free interval between the two bursts)',
        'stair': 'SCALE f\'(z), static, ~1 um per level',
        'sine': 'see below',
        'accel': 'BOTH — must equal scale x clock^2',
    }
    for s in segments:
        if s.kind == 'sine':
            desc = ("CLOCK (freq ratio)" if s.name == 'sine_long'
                    else "SCALE f'(z), dynamic")
            detail = ("%s: f=%.4f Hz exactly, A=%.3f rev (%.1f mm), %.0f cycles"
                      % (desc, s.params['freq_hz'], s.params['amplitude_rev'],
                         s.params['amplitude_rev'] * MM_PER_REV,
                         s.params['n_cycles']))
        elif s.kind == 'stair':
            detail = ("%s: %d levels, %.3f rev (%.2f mm) apart, %.1f s dwell"
                      % (what['stair'], int(s.params['n_levels']),
                         s.params['step_rev'], s.params['step_rev'] * MM_PER_REV,
                         s.params['dwell_s']))
        elif s.kind == 'accel':
            detail = ("%s: %d strokes at %.3f rev/s^2 = %.4f m/s^2 (%.2f%% of g), "
                      "%.2f s each" % (what['accel'], int(s.params['n_strokes']),
                                       s.params['accel_rps2'], s.params['accel_m_s2'],
                                       100.0 * s.params['accel_m_s2'] / 9.80665,
                                       s.params['stroke_s']))
        else:
            detail = ("%s: %d traverses, +/-%.2f rev (%.1f mm), %.2f s dwell/corner"
                      % (what['sync'], int(s.params['traverses']),
                         s.params['amplitude_rev'],
                         s.params['amplitude_rev'] * MM_PER_REV, s.params['dwell_s']))
        print("    %-14s %7.1fs %7.1fs %9d   %s"
              % (s.name, s.t_start_s(spec.seg_t_s), s.duration_s(spec.seg_t_s),
                 s.n_knots, detail))

    sync = [s for s in segments if s.kind == 'sync']
    if len(sync) >= 2:
        base = (sync[-1].t_start_s(spec.seg_t_s) - sync[0].t_start_s(spec.seg_t_s))
        print("\n  the clock lever arm: %.1f s between the two sync bursts." % base)
        print("    a 1.3%% clock error shows as %.2f s of divergence;"
              % (base * (1.0 / math.sqrt(1.0 - 0.025) - 1.0)))
        print("    a corner locates to ~3 ms, so that is a ~%.0f sigma test."
              % (base * (1.0 / math.sqrt(1.0 - 0.025) - 1.0) / 0.003))

    print("\n  safety envelope of the generated profile")
    print("    position span           %.4f-%.4f rev   (window %.3f-%.3f)"
          % (series.min(), series.max(), b.lo_rev, b.hi_rev))
    print("    max knot step           %.5f rev  = %.0f%% of the lead clamp"
          % (steps.max(), 100.0 * steps.max() / BRIDGE_MAX_LEAD_REV))
    print("    peak velocity           %.3f rev/s  = %.0f%% of the session cap"
          % (steps.max() / spec.seg_t_s, 100.0 * steps.max()
             / spec.seg_t_s / vel_cap))
    if steps.size >= 2:
        acc = np.abs(np.diff(steps / spec.seg_t_s)) / spec.seg_t_s
        print("    peak commanded accel    %.1f rev/s^2  = %.0f%% of the hard cap %.0f"
              % (acc.max(), 100.0 * acc.max() / HARD_ACCEL_CAP_RPS2,
                 HARD_ACCEL_CAP_RPS2))

    if problems:
        print("\n  ***** THIS PROFILE WOULD BE REFUSED *****")
        for p in problems:
            print("    REJECT: %s" % p)
    else:
        print("\n  validate_profile(): OK — every knot inside bounds, every step "
              "inside the budget.")

    print("\n  output → %s" % out_dir)
    print(PREFLIGHT)
    return 1 if problems else 0


def main(argv: Optional[List[str]] = None) -> int:
    p = argparse.ArgumentParser(
        description="Drive the bench leg through a profile designed to separate a "
                    "QTM capture-clock error from a local vertical-scale error, "
                    "logging encoder telemetry for comparison against a "
                    "simultaneous QTM recording.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=PREFLIGHT)
    p.add_argument('--dry-run', action='store_true',
                   help="print the full plan and safety envelope; no hardware I/O")
    p.add_argument('--self-test', action='store_true',
                   help="generate and validate profiles across the parameter range; "
                        "no hardware I/O. Run this after editing any default.")
    p.add_argument('--quick', action='store_true',
                   help="~2 min profile instead of ~6 min, for a first shakedown or "
                        "a limited QTM capture length")
    p.add_argument('--home', action='store_true',
                   help="fire the firmware HOME(axis) RPC first (do this if the "
                        "encoder reference is not fresh)")
    p.add_argument('--axis', type=int, default=0,
                   help="CAN node id of the bench leg (default 0)")
    p.add_argument('--knot-hz', type=float, default=BRIDGE_SETPOINT_HZ,
                   choices=[40.0, 100.0],
                   help="knot stream rate (default %.0f). 100 REQUIRES the "
                        "BENCH_SYSID_BUILD firmware; the stock build will not "
                        "honour it. The script warns if the measured telemetry "
                        "rate suggests a mismatch." % BRIDGE_SETPOINT_HZ)
    p.add_argument('--segments', type=_parse_segments,
                   default=','.join(SEGMENT_ORDER),
                   help="comma-separated subset, in order. Known: %s. Repeat 'sync' "
                        "at both ends to keep the model-free clock test."
                        % ', '.join(KNOWN_SEGMENTS))
    p.add_argument('--stroke-cap', type=float, default=HARD_STROKE_CAP_REV,
                   help="bench-leg stroke cap in rev (default %.1f)"
                        % HARD_STROKE_CAP_REV)
    p.add_argument('--centre', type=float, default=None,
                   help="operating centre in rev (default: mid-stroke)")
    p.add_argument('--vel-frac', type=float, default=DEFAULT_VEL_FRAC,
                   help="fraction of the harness's 0.5-margin lead-clamp ramp step "
                        "to use (default %.2f => %.2f rev/s). Raising this rides the "
                        "clamp and makes the encoder stop following the command."
                        % (DEFAULT_VEL_FRAC,
                           DEFAULT_VEL_FRAC * sid.lead_clamp_frame_step(
                               BRIDGE_MAX_LEAD_REV, BRIDGE_LEAD_MARGIN_FRAC)
                           / BRIDGE_SEG_T_S))
    p.add_argument('--vel-cap', type=float, default=DEFAULT_VEL_CAP_RPS,
                   help="ODrive velocity limit for the session (default %.1f rev/s)"
                        % DEFAULT_VEL_CAP_RPS)
    p.add_argument('--current-limit', type=float, default=HARD_CURRENT_LIMIT_A,
                   help="ODrive current limit in A (default %.1f)"
                        % HARD_CURRENT_LIMIT_A)
    p.add_argument('--sine-long-hz', type=float, default=DEFAULT_SINE_LONG_HZ)
    p.add_argument('--sine-long-secs', type=float, default=DEFAULT_SINE_LONG_S)
    p.add_argument('--sine-local-hz', type=float, default=DEFAULT_SINE_LOCAL_HZ)
    p.add_argument('--sine-local-secs', type=float, default=DEFAULT_SINE_LOCAL_S)
    p.add_argument('--stair-levels', type=int, default=DEFAULT_STAIR_LEVELS)
    p.add_argument('--stair-dwell', type=float, default=DEFAULT_STAIR_DWELL_S)
    p.add_argument('--accel-strokes', type=int, default=DEFAULT_ACCEL_STROKES)
    p.add_argument('--output-dir', default=None)
    a = p.parse_args(argv)

    segments = (a.segments if isinstance(a.segments, tuple)
                else _parse_segments(a.segments))
    spec = ProfileSpec(
        stroke_cap_rev=a.stroke_cap, centre_rev=a.centre, vel_frac=a.vel_frac,
        seg_t_s=1.0 / a.knot_hz, segments=segments, stair_levels=a.stair_levels,
        stair_dwell_s=a.stair_dwell, sine_long_hz=a.sine_long_hz,
        sine_long_s=a.sine_long_secs, sine_local_hz=a.sine_local_hz,
        sine_local_s=a.sine_local_secs, accel_strokes=a.accel_strokes)
    if a.quick:
        for k, v in QUICK_OVERRIDES.items():
            setattr(spec, k, v)

    out_dir = a.output_dir or _default_output_dir()

    if a.self_test:
        return self_test()
    if a.dry_run:
        return print_plan(spec, a.vel_cap, out_dir)

    probe = MocapTimebaseProbe(
        spec=spec, axis_id=a.axis, stroke_cap_rev=a.stroke_cap,
        center_rev=spec.centre, current_limit_a=a.current_limit,
        vel_cap_rps=a.vel_cap, output_dir=out_dir, dry_run=False,
        do_home=a.home, knot_hz=a.knot_hz,
        # Required by BridgeSysID.__init__ but unused by the mocap stage: no chirp,
        # no position steps, no ladder, no onset detection runs here. Values are the
        # harness's own defaults so nothing is silently out of range if a future
        # edit does reach them.
        chirp_f0=1.0, chirp_f1=30.0, chirp_dur=8.0, chirp_amp=0.02,
        pos_steps=[0.07], ladder_step=0.14, zeta_target=0.6, bw_clear_hz=None,
        n_vel=4, ripple_threshold=0.05, osc_threshold=0.5,
        iq_ripple_threshold=0.5)
    print(PREFLIGHT)
    print("  Starting in 5 s — QTM should already be recording. Ctrl-C to abort.\n")
    for i in range(5, 0, -1):
        print("    %d..." % i)
        time.sleep(1.0)
    return probe.run()


# ---------------------------------------------------------------------------
# Self-test: profile generation is pure, so it can be checked without hardware.
# ---------------------------------------------------------------------------

def self_test() -> int:
    """Generate profiles across the parameter range and assert the envelope holds.

    This is not a substitute for ``tests/`` — it is the thing to run at the bench
    after changing a default, when the cost of a bad profile is a damaged ballscrew
    and the cost of running it is two seconds.
    """
    failures: List[str] = []
    checked = 0

    def check(spec: ProfileSpec, vel_cap: float, label: str) -> None:
        nonlocal checked
        checked += 1
        try:
            series, segments = build_profile(spec, spec.centre)
        except Exception as exc:  # noqa: BLE001
            failures.append("%s: build raised %r" % (label, exc))
            return
        for prob in validate_profile(series, spec, vel_cap):
            failures.append("%s: %s" % (label, prob))
        # The knot series must start where it was told to and end at the centre,
        # so a run can be followed by another without a jump.
        if abs(series[0] - spec.centre) > 1e-9:
            failures.append("%s: first knot %.5f != start %.5f"
                            % (label, series[0], spec.centre))
        if abs(series[-1] - spec.centre) > 1e-6:
            failures.append("%s: last knot %.5f != centre %.5f"
                            % (label, series[-1], spec.centre))
        # Segments must tile without overlapping.
        last_end = 0
        for s in segments:
            if s.knot_start < last_end:
                failures.append("%s: segment %s overlaps the previous"
                                % (label, s.name))
            if s.knot_end <= s.knot_start:
                failures.append("%s: segment %s is empty" % (label, s.name))
            last_end = s.knot_end
        # Every commanded sine must hit the frequency it claims, since that IS the
        # clock observable — a builder that silently retunes it would be invisible.
        for s in segments:
            if s.kind != 'sine':
                continue
            body = series[s.knot_start:s.knot_end]
            n_expect = s.params['n_cycles']
            mid = body - np.median(body)
            crossings = int(np.sum((mid[:-1] <= 0) & (mid[1:] > 0)))
            if abs(crossings - n_expect) > 1.5:
                failures.append("%s: %s has %d upward crossings, expected ~%.1f"
                                % (label, s.name, crossings, n_expect))

    base = ProfileSpec()
    check(base, DEFAULT_VEL_CAP_RPS, "default")
    check(ProfileSpec(**dict(asdict(base), **QUICK_OVERRIDES)),
          DEFAULT_VEL_CAP_RPS, "quick")
    for vf in (0.4, 0.6, 0.8, 1.0):
        check(ProfileSpec(vel_frac=vf), DEFAULT_VEL_CAP_RPS, "vel_frac=%.1f" % vf)
    for cap in (2.0, 2.5, 3.0, 3.3):
        check(ProfileSpec(stroke_cap_rev=cap), DEFAULT_VEL_CAP_RPS,
              "stroke_cap=%.1f" % cap)
    for centre in (1.0, 1.5, 2.0):
        check(ProfileSpec(centre_rev=centre), DEFAULT_VEL_CAP_RPS,
              "centre=%.1f" % centre)
    for name in KNOWN_SEGMENTS:
        check(ProfileSpec(segments=(name,)), DEFAULT_VEL_CAP_RPS, "only=%s" % name)
    for f in (0.1, 0.2, 0.5):
        check(ProfileSpec(sine_long_hz=f), DEFAULT_VEL_CAP_RPS, "sine_long=%.1fHz" % f)
    for f in (0.5, 0.8, 1.5):
        check(ProfileSpec(sine_local_hz=f), DEFAULT_VEL_CAP_RPS,
              "sine_local=%.1fHz" % f)
    for n in (3, 7, 13, 21):
        check(ProfileSpec(stair_levels=n), DEFAULT_VEL_CAP_RPS, "stair=%d" % n)
    # Both knot rates: the BENCH_SYSID_BUILD firmware streams at 100 Hz, and the
    # velocity budget must hold constant across them or the sync bursts' velocity
    # reversals blow the acceleration cap.
    for hz in (40.0, 100.0):
        check(ProfileSpec(seg_t_s=1.0 / hz), DEFAULT_VEL_CAP_RPS, "knot_hz=%.0f" % hz)
        check(ProfileSpec(seg_t_s=1.0 / hz, vel_frac=1.0), DEFAULT_VEL_CAP_RPS,
              "knot_hz=%.0f,vel_frac=1.0" % hz)

    print("self-test: %d profiles generated and validated" % checked)
    if failures:
        print("\nFAILURES (%d):" % len(failures))
        for f in failures:
            print("  %s" % f)
        return 1
    print("all clear — bounds, lead-clamp budget, velocity and acceleration caps "
          "hold across the parameter range, segments tile, and every commanded "
          "sine frequency survives generation.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
