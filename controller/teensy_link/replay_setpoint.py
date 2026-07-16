"""Recorded-throw Teensy-side knot setpoint source — bench validation of the Teensy interp.

Replays a recorded MPC command stream (a
``temp/logs/mpc_*.csv`` ``cmd_ext`` trajectory, the same recordings the offline
Hermite xref uses — ``tools/probes/.../hermite_xref/xref.py``) as ``flags=0x3``
:class:`Setpoint` frames for a **single** target leg axis, so the Teensy's 40 Hz
knot Hermite interp (``leg_interp.cpp`` Mode 1) can be exercised **on hardware**
with a representative throw-shaped trajectory. This is what produces the two
measurements that gate the production cutover:

  * the **float32 interp residual** — the real Teensy float32 ladder output vs a
    float64 reference (motor_guard / ``teensy_interp.py``) for the same knots; and
  * the **motion-onset penalty** — how much later the leg breaks away on the
    friction-FF-free Teensy-side path vs a with-friction-FF baseline. The optional
    ``torque_ff_fn`` injects a per-frame friction-FF torque (dependency-injected
    so this module stays free of any ``jugglebot.motion`` import — the bench
    driver supplies the Stribeck model) for the A/B onset comparison.

Bench-leg safety: the bench leg is physically shorter than the production legs
(operator: max ≈ 3.4 rev), but the firmware ``STROKE_MAX_REV[0]`` ships the
production value (≈ 3.90), so the firmware stroke clamp will NOT protect the
bench leg. :func:`scale_to_bench` therefore enforces a **position ceiling**
(default 3.30 rev, 0.10 below the 3.4 physical limit) — affine-compressing a
recording that exceeds it, no-op for one that fits — plus a **velocity guard**
(default 3.5 rev/s, kept below the ``ODRIVE_LEG_VEL_LIMIT_RPS`` the
``--close-loop`` bring-up sets — 12.0 since 2026-07-16, see hardware_config.yaml
leg_vel_limit_rps — so the leg can actually track the replay).

Trajectory timeline (seconds since arm ``t ≥ 0``), three phases so the onset is
clean to measure:

  * ``t < approach_s`` — raised-cosine ease ``start_rev → rec[0]`` (bring the leg
    to the recording's first sample; zero velocity at both ends).
  * ``approach_s ≤ t < approach_s + settle_s`` — hold at ``rec[0]`` (let the leg
    settle so the breakaway latency is measured from a true rest).
  * ``t ≥ approach_s + settle_s`` — replay the recorded samples (linearly
    interpolated between the 40 Hz grid points), holding the last sample at the
    end.

Knots/flags/sign conventions are identical to :mod:`synthetic_setpoint` (JUGGLEBOT
convention, positive = extension; the Teensy ``odrive_protocol`` applies the sign
flip — ADR-0012). Pure: no threads/sleeps; the loader (:func:`load_recorded_axis`)
is the only I/O and is separated from the trajectory math so the scaling +
frame logic is unit-testable (``tests/teensy_link/test_replay_setpoint.py``).
"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from typing import Callable, List, Optional

from . import protocol as p
from .protocol import Setpoint

DEFAULT_SEG_T_S = 0.025          # SEGMENT_T_S — recordings are sampled at this 40 Hz grid
DEFAULT_MAX_STEP_REV = 0.15      # replay knot-step bound (historically = the pre-2026-07-10 MAX_LEAD 0.15; the live lead clamp is 0.10)
DEFAULT_CEILING_REV = 3.30       # bench-leg position ceiling (0.10 below the 3.4 physical limit)
DEFAULT_VEL_CEILING_RPS = 3.5    # kept below ODRIVE_LEG_VEL_LIMIT_RPS (see hardware_config.yaml) so the leg can track it
FLAG_HAS_U1 = 0x1
FLAG_HAS_U2 = 0x2


def load_recorded_axis(csv_path: str, axis: int, mm_to_rev: float) -> List[float]:
    """Load one axis's ``cmd_ext`` column from a recorded ``mpc_*.csv``, in motor rev.

    The recordings store commanded leg extension in mm (``cmd_ext_{i}``); multiply
    by ``mm_to_rev`` (the per-axis ``geom.mm_to_rev[axis]``) to get motor rev —
    exactly as ``xref.py`` does, so the replay matches the offline reference.
    """
    out: List[float] = []
    with open(csv_path) as f:
        for r in csv.DictReader(f):
            try:
                out.append(float(r[f"cmd_ext_{axis}"]) * mm_to_rev)
            except (KeyError, ValueError):
                continue
    if len(out) < 3:
        raise ValueError(f"{csv_path}: too few usable cmd_ext_{axis} rows ({len(out)})")
    return out


def time_stretch(samples: List[float], factor: float) -> List[float]:
    """Resample a 40 Hz position trajectory to play ``factor``× slower.

    The samples sit on the firmware ``SEGMENT_T_S`` (0.025 s) grid, so to stretch
    wall-clock time by ``factor`` (>1 = slower) we resample to ``factor``× as many
    grid points by linear interpolation along the original index. The **position
    range and shape are preserved exactly** (endpoints unchanged) — only the time
    axis dilates, so per-frame **velocity scales 1/factor**, and the per-frame
    velocity *steps* at the knots (the onset the leg must chase) also scale
    1/factor. (Note: a *continuous* time-dilation would scale acceleration by
    1/factor², but a linear resample keeps the velocity steps discrete at the
    original knots — just smaller — so the per-frame demand scales 1/factor;
    verified empirically: peak step-accel 75→38→25 rev/s² at 1×/2×/3×.)

    Why: a real-robot throw recording's *onset acceleration* can exceed what the
    (current-limited) bench leg can produce, so it under-tracks and trips the
    deviation belt before the float32 interp residual can be measured cleanly
    (the firmware lead clamp saturates → ``cmd_teensy`` is no longer the pure
    interp output). The **float32 residual is speed-independent** (it's float32-
    vs-float64 arithmetic on ~3 rev position values, identical however slowly the
    same positions are traversed), so stretching lets the leg track the trajectory
    within the lead clamp — yielding a valid residual — and gives a clean,
    measurable from-rest onset for the friction-FF A/B. ``factor == 1.0`` is a
    no-op (returns a copy).
    """
    if factor <= 0:
        raise ValueError(f"time_stretch factor must be > 0, got {factor}")
    if factor == 1.0:
        return list(samples)
    n = len(samples)
    if n < 2:
        return list(samples)
    new_n = max(2, int(round((n - 1) * factor)) + 1)
    out: List[float] = []
    for j in range(new_n):
        x = j / factor                       # fractional original index
        k = int(math.floor(x))
        if k >= n - 1:
            out.append(samples[-1])
        else:
            frac = x - k
            out.append(samples[k] + (samples[k + 1] - samples[k]) * frac)
    return out


@dataclass(frozen=True)
class ScaleInfo:
    """What :func:`scale_to_bench` did, for logging (never silently truncate)."""
    pos_scale: float          # affine gain applied about the recording min (1.0 = none)
    raw_min_rev: float
    raw_max_rev: float
    scaled_min_rev: float
    scaled_max_rev: float
    peak_vel_rps: float       # peak 40 Hz chord velocity AFTER scaling
    ceiling_rev: float


def scale_to_bench(samples: List[float], *, stroke_min_rev: float,
                   ceiling_rev: float = DEFAULT_CEILING_REV,
                   vel_ceiling_rps: float = DEFAULT_VEL_CEILING_RPS,
                   seg_t_s: float = DEFAULT_SEG_T_S):
    """Make a recorded motor-rev trajectory safe for the (shorter) bench leg.

    Position: if ``max > ceiling``, affine-compress ``[min, max] → [min, ceiling]``
    (preserves the start/onset, compresses the peak); otherwise a no-op. A final
    hard clamp to ``[stroke_min, ceiling]`` is the backstop.

    Velocity: the peak 40 Hz chord velocity after scaling MUST be ``≤
    vel_ceiling`` (kept below the ODrive's velocity limit so the leg can track
    it) — raises ``ValueError`` otherwise (loud, never a silent slow-down).

    Returns ``(scaled_samples, ScaleInfo)``.
    """
    # A floor-above-ceiling config is unusable — the affine
    # map + hard clamp would silently emit a flat/sign-inverted profile. Fail loud
    # (the module's own "never a silent slow-down" stance), never execute it.
    if stroke_min_rev > ceiling_rev:
        raise ValueError(
            f"stroke_min_rev ({stroke_min_rev:.3f}) > ceiling_rev "
            f"({ceiling_rev:.3f}) — unusable bench scaling config (would produce a "
            f"flat/inverted profile). Fix the bench floor/ceiling.")
    raw_min = min(samples)
    raw_max = max(samples)
    if raw_max > ceiling_rev:
        span = raw_max - raw_min
        gain = (ceiling_rev - raw_min) / span if span > 1e-9 else 1.0
        scaled = [raw_min + (s - raw_min) * gain for s in samples]
    else:
        gain = 1.0
        scaled = list(samples)
    # Hard safety clamp (backstop; no-op when the affine map already fits).
    scaled = [min(max(s, stroke_min_rev), ceiling_rev) for s in scaled]

    peak_vel = 0.0
    for i in range(1, len(scaled)):
        peak_vel = max(peak_vel, abs(scaled[i] - scaled[i - 1]) / seg_t_s)
    if peak_vel > vel_ceiling_rps:
        raise ValueError(
            f"recorded peak chord velocity {peak_vel:.3f} rev/s > ceiling "
            f"{vel_ceiling_rps} rev/s (the ODrive velocity limit could not track "
            f"it → tracking deviation). Pick a slower recording or lower the "
            f"velocity ceiling deliberately.")

    info = ScaleInfo(pos_scale=gain, raw_min_rev=raw_min, raw_max_rev=raw_max,
                     scaled_min_rev=min(scaled), scaled_max_rev=max(scaled),
                     peak_vel_rps=peak_vel, ceiling_rev=ceiling_rev)
    return scaled, info


class RecordedThrowSource:
    """Pure generator of Teensy-side knot :class:`Setpoint` frames replaying a recording.

    Args:
        samples: the scaled motor-rev trajectory (from :func:`scale_to_bench`).
        axis: target leg axis.
        start_rev: live encoder position at arm (the approach ramps from here to
            ``samples[0]``).
        stroke_min_rev / stroke_max_rev: firmware per-axis stroke bounds for the
            target axis (used only to validate ``samples`` sit within the hard
            stroke window).
        approach_s: raised-cosine ramp time from ``start_rev`` to ``samples[0]``.
        settle_s: hold at ``samples[0]`` before the replay (clean onset).
        seg_t_s: knot spacing; MUST equal the firmware ``SEGMENT_T_S`` and the
            recording's sample period.
        num_legs / max_step_rev / stroke_margin_rev: as in
            :class:`synthetic_setpoint.SyntheticKnotSource`.
        torque_ff_fn: optional ``velocity_rev_s -> torque_Nm`` for the target axis
            (the Stribeck friction-FF, for the friction-FF A/B). ``None`` ⇒ ``torque_ff=0``
            (the plain Teensy-side path).

    Raises:
        ValueError: axis out of range, samples outside the stroke window, the
            approach step breaches the lead clamp, or a per-frame replay step does.
    """

    def __init__(self, samples: List[float], *, axis: int, start_rev: float,
                 stroke_min_rev: float, stroke_max_rev: float,
                 approach_s: float = 3.0, settle_s: float = 1.0,
                 seg_t_s: float = DEFAULT_SEG_T_S,
                 num_legs: int = p.NUM_LEGS,
                 max_step_rev: float = DEFAULT_MAX_STEP_REV,
                 stroke_margin_rev: float = 0.02,
                 torque_ff_fn: Optional[Callable[[float], float]] = None):
        if not (0 <= axis < num_legs):
            raise ValueError(f"axis {axis} out of range [0, {num_legs})")
        if approach_s <= 0:
            raise ValueError("approach_s must be > 0")
        if settle_s < 0:
            raise ValueError("settle_s must be >= 0")
        if len(samples) < 3:
            raise ValueError("need >= 3 recorded samples")

        lo = stroke_min_rev + stroke_margin_rev
        hi = stroke_max_rev - stroke_margin_rev
        s_min, s_max = min(samples), max(samples)
        # Hard floor is the binding safety bound; the margin keep-out is advisory
        # for the replay (recordings legitimately graze the floor), so check the
        # hard stroke window, not the margined one, for the recorded body.
        if s_min < stroke_min_rev or s_max > stroke_max_rev:
            raise ValueError(
                f"recorded samples [{s_min:.4f}, {s_max:.4f}] outside stroke window "
                f"[{stroke_min_rev:.4f}, {stroke_max_rev:.4f}]")
        if not (lo <= hi):
            raise ValueError(f"stroke window empty after margin: [{lo:.4f}, {hi:.4f}]")

        self.samples = list(samples)
        self.axis = int(axis)
        self.start_rev = float(start_rev)
        self.approach_s = float(approach_s)
        self.settle_s = float(settle_s)
        self.seg_t = float(seg_t_s)
        self.n = int(num_legs)
        self.max_step_rev = float(max_step_rev)
        self._torque_ff_fn = torque_ff_fn
        self._rec0 = float(samples[0])
        self._dur = (len(samples) - 1) * self.seg_t      # replay duration

        # Lead-clamp check: the worst per-frame step across BOTH the approach ramp
        # and the replay body must be within the firmware lead clamp.
        approach_peak_v = (abs(self._rec0 - self.start_rev)
                           * math.pi / (2.0 * self.approach_s))
        replay_peak_v = 0.0
        for i in range(1, len(self.samples)):
            replay_peak_v = max(replay_peak_v,
                                abs(self.samples[i] - self.samples[i - 1]) / self.seg_t)
        peak_step = max(approach_peak_v, replay_peak_v) * self.seg_t
        if peak_step > self.max_step_rev:
            raise ValueError(
                f"peak per-frame step {peak_step:.4f} rev > lead clamp "
                f"{self.max_step_rev} (approach_peak_v={approach_peak_v:.3f}, "
                f"replay_peak_v={replay_peak_v:.3f} rev/s); increase approach_s")
        self.peak_step_rev = peak_step
        self.replay_peak_vel_rps = replay_peak_v

    @property
    def total_duration_s(self) -> float:
        """Full timeline: approach + settle + replay."""
        return self.approach_s + self.settle_s + self._dur

    @property
    def onset_t_s(self) -> float:
        """``t`` at which the replay (the throw onset) begins."""
        return self.approach_s + self.settle_s

    # ── Analytic/interpolated trajectory (single axis) ─────────────────────────
    def position(self, t: float) -> float:
        """Target position (rev) at ``t`` seconds since arm."""
        if t <= 0.0:
            return self.start_rev
        if t < self.approach_s:
            s = t / self.approach_s
            ease = 0.5 * (1.0 - math.cos(math.pi * s))        # 0→1, zero slope ends
            return self.start_rev + (self._rec0 - self.start_rev) * ease
        th = t - self.approach_s
        if th < self.settle_s:
            return self._rec0
        return self._replay_pos(th - self.settle_s)

    def velocity(self, t: float) -> float:
        """Target velocity feedforward (rev/s) at ``t`` — analytic/chord derivative."""
        if t <= 0.0:
            return 0.0
        if t < self.approach_s:
            s = t / self.approach_s
            dease_dt = 0.5 * math.sin(math.pi * s) * math.pi / self.approach_s
            return (self._rec0 - self.start_rev) * dease_dt
        th = t - self.approach_s
        if th < self.settle_s:
            return 0.0
        return self._replay_vel(th - self.settle_s)

    def _replay_pos(self, tr: float) -> float:
        if tr <= 0.0:
            return self.samples[0]
        fk = tr / self.seg_t
        k = int(math.floor(fk))
        if k >= len(self.samples) - 1:
            return self.samples[-1]
        frac = fk - k
        return self.samples[k] + (self.samples[k + 1] - self.samples[k]) * frac

    def _replay_vel(self, tr: float) -> float:
        if tr < 0.0:
            return 0.0
        k = int(math.floor(tr / self.seg_t))
        if k >= len(self.samples) - 1:
            return 0.0
        return (self.samples[k + 1] - self.samples[k]) / self.seg_t

    # ── Frame construction (identical knot/flag layout to the synthetic source) ──
    def frame(self, t: float, t_origin_us: int) -> Setpoint:
        ax = self.axis
        T = self.seg_t
        p0 = self.position(t)
        p1 = self.position(t + T)
        p2 = self.position(t + 2.0 * T)
        v0 = self.velocity(t)
        tau = self._torque_ff_fn(v0) if self._torque_ff_fn is not None else 0.0

        def vec(val_at_axis: float) -> tuple:
            return tuple(val_at_axis if i == ax else 0.0 for i in range(self.n))

        return Setpoint(
            u0=vec(p0),
            u1=vec(p1),
            u2=vec(p2),
            v0=vec(v0),
            accel=(0.0,) * self.n,
            torque_ff=vec(tau),
            flags=FLAG_HAS_U1 | FLAG_HAS_U2,
            t_origin_us=int(t_origin_us),
        )
