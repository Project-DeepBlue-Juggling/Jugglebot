"""Synthetic Teensy-side knot setpoint source — bench validation of the Teensy interp.

Generates ``flags=0x3`` :class:`Setpoint` frames (``u0``/``u1``/``u2``
lookahead knots + ``v0`` velocity feedforward) for a **single** target leg axis,
so the Teensy's 40 Hz-knot cubic-Hermite interpolator (``leg_interp.cpp`` Mode 1)
can be exercised on hardware with a **known, bounded** command — fully decoupled
from ``motor_guard`` / the MPC / the Stribeck friction-FF. This is
the *synthetic Teensy-side* path used by ``tests/hardware/teensy_setpoint_bench.py``; the
deployed production bridge is untouched and still forwards ``motor_guard``'s
500 Hz output with ``flags=0`` (the Jetson-relay path, ``setpoint_pump.py``).

Pure — no I/O, no threads, no sleeps (the ``homing.py`` / ``setpoint_pump.py``
pattern). The bench driver supplies wall-time, the live start position, and does
the transport; this module only decides *what a well-formed, in-bounds knot frame
is*. Keeping it pure makes the trajectory + bounds logic unit-testable
(``tests/teensy_link/test_synthetic_setpoint.py``).

Trajectory (single axis), seconds since arm ``t ≥ 0``:

  * ``t < approach_s`` — smooth raised-cosine ease ``start_rev → center_rev`` with
    **zero velocity at both ends**. The homed leg sits at the retracted hardstop
    (≈ −0.1 rev, *below* the firmware stroke floor 0.0709 rev), so the very first
    armed motion is necessarily a controlled extension *into* the workspace, not a
    hold-in-place. The ease keeps that onset gentle.
  * ``t ≥ approach_s`` — ``center_rev + amplitude_rev·sin(2π·freq·(t−approach_s))``.
    ``amplitude_rev == 0`` ⇒ a pure hold at ``center_rev``.

Knots: ``u0 = pos(t)``, ``u1 = pos(t+T)``, ``u2 = pos(t+2T)`` with ``T = seg_t_s``
(the firmware's fixed ``SEGMENT_T_S = 0.025 s``); ``v0 = pos'(t)``;
``flags = HAS_U1 | HAS_U2 = 0x3``. Every **non-target** axis is packed ``0.0`` —
matching an absent ODrive's ``pos_rev = 0`` so the firmware ``MAX_DEVIATION``
guard can never trip on it (belt-and-suspenders with the firmware's own
present-axis scoping).

NB the velocity feedforward ``v0`` has a small step at the approach→sinusoid
handover (the ease arrives at zero velocity; the sinusoid departs at
``amplitude·2π·freq``). Position is continuous. A ``v0`` step between frames is
exactly what the interp absorbs every 40 Hz cycle in normal MPC operation, so it
is not a discontinuity the leg can feel — but it is why the operator runs the
*hold* stage (``amplitude=0``, fully smooth) before the sinusoid.

Convention: positions/velocities are JUGGLEBOT convention (positive = extension),
identical to what ``motor_guard`` publishes; the Teensy ``odrive_protocol`` applies
the sign flip on the way to the ODrive (ADR-0012).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from . import protocol as p
from .protocol import Setpoint

# Firmware-matched constants (canbridge_config.h). Mirrored here so the generator
# is usable standalone; the bench driver passes the authoritative per-axis stroke
# bounds. SEG_T must equal the firmware's compile-time SEGMENT_T_S or the knot
# lookahead won't line up with the interp's `s = dt / SEG_T`.
DEFAULT_SEG_T_S = 0.025          # SEGMENT_T_S (the 40 Hz MPC fine step)
DEFAULT_MAX_STEP_REV = 0.15      # MAX_LEAD_REV — a single knot may never command
                                 # a step the firmware lead-clamp would reject.
FLAG_HAS_U1 = 0x1
FLAG_HAS_U2 = 0x2


@dataclass(frozen=True)
class TrajectoryParams:
    """One target axis's bounded trajectory."""
    axis: int
    start_rev: float          # live encoder position at arm (may be the sub-stroke hardstop)
    center_rev: float         # in-workspace hold/oscillation centre
    amplitude_rev: float = 0.0
    freq_hz: float = 0.0
    approach_s: float = 2.0


class SyntheticKnotSource:
    """Pure generator of Teensy-side knot :class:`Setpoint` frames for one leg axis.

    Args:
        params: the trajectory definition.
        stroke_min_rev / stroke_max_rev: the firmware per-axis stroke clamp bounds
            for ``params.axis`` (``STROKE_{MIN,MAX}_REV[axis]``). The
            *oscillation envelope* (``center ± |amplitude|``) must sit strictly
            inside ``[min+margin, max−margin]``. ``start_rev`` is NOT required to
            be in-workspace (it may be the sub-stroke hardstop).
        num_legs: width of the Setpoint leg arrays (6).
        seg_t_s: knot spacing; MUST equal the firmware ``SEGMENT_T_S``.
        max_step_rev: peak allowed per-frame position step (defaults to the
            firmware lead clamp); the worst-case ``|pos(t+T)−pos(t)|`` over the
            whole trajectory is checked against it.
        stroke_margin_rev: keep-out from each stroke limit.

    Raises:
        ValueError: axis out of range, envelope outside the workspace, or a
            per-frame step that would breach the lead clamp.
    """

    def __init__(self, params: TrajectoryParams, *,
                 stroke_min_rev: float, stroke_max_rev: float,
                 num_legs: int = p.NUM_LEGS,
                 seg_t_s: float = DEFAULT_SEG_T_S,
                 max_step_rev: float = DEFAULT_MAX_STEP_REV,
                 stroke_margin_rev: float = 0.02):
        if not (0 <= params.axis < num_legs):
            raise ValueError(f"axis {params.axis} out of range [0, {num_legs})")
        if params.amplitude_rev < 0:
            raise ValueError("amplitude_rev must be >= 0")
        if params.freq_hz < 0:
            raise ValueError("freq_hz must be >= 0")
        if params.approach_s <= 0:
            raise ValueError("approach_s must be > 0")

        amp = abs(params.amplitude_rev)
        lo = stroke_min_rev + stroke_margin_rev
        hi = stroke_max_rev - stroke_margin_rev
        if lo >= hi:
            raise ValueError(
                f"stroke window empty after margin: [{lo:.4f}, {hi:.4f}]")
        if not (lo <= params.center_rev - amp and params.center_rev + amp <= hi):
            raise ValueError(
                f"oscillation envelope [{params.center_rev - amp:.4f}, "
                f"{params.center_rev + amp:.4f}] outside workspace "
                f"[{lo:.4f}, {hi:.4f}] (center={params.center_rev}, amp={amp})")

        self.params = params
        self.n = int(num_legs)
        self.seg_t = float(seg_t_s)
        self.max_step_rev = float(max_step_rev)

        # Worst-case per-frame position step over the whole trajectory.
        #  approach: peak |v| of the raised-cosine ease = |Δ|·π/(2·approach_s)
        #  sinusoid: peak |v| = amp·2π·freq
        approach_peak_v = (abs(params.center_rev - params.start_rev)
                           * math.pi / (2.0 * params.approach_s))
        sinusoid_peak_v = amp * 2.0 * math.pi * params.freq_hz
        peak_step = max(approach_peak_v, sinusoid_peak_v) * self.seg_t
        if peak_step > self.max_step_rev:
            raise ValueError(
                f"peak per-frame step {peak_step:.4f} rev > max {self.max_step_rev} "
                f"(approach_peak_v={approach_peak_v:.3f}, "
                f"sinusoid_peak_v={sinusoid_peak_v:.3f} rev/s); "
                f"increase approach_s or reduce amplitude/freq")
        self.peak_step_rev = peak_step

    # ── Analytic trajectory (single axis) ──────────────────────────────────────
    def position(self, t: float) -> float:
        """Target position (rev) at ``t`` seconds since arm."""
        pr = self.params
        if t <= 0.0:
            return pr.start_rev
        if t < pr.approach_s:
            s = t / pr.approach_s
            ease = 0.5 * (1.0 - math.cos(math.pi * s))        # 0→1, zero slope ends
            return pr.start_rev + (pr.center_rev - pr.start_rev) * ease
        tt = t - pr.approach_s
        return pr.center_rev + pr.amplitude_rev * math.sin(2.0 * math.pi * pr.freq_hz * tt)

    def velocity(self, t: float) -> float:
        """Target velocity feedforward (rev/s) at ``t`` — analytic derivative."""
        pr = self.params
        if t <= 0.0:
            return 0.0
        if t < pr.approach_s:
            s = t / pr.approach_s
            dease_dt = 0.5 * math.sin(math.pi * s) * math.pi / pr.approach_s
            return (pr.center_rev - pr.start_rev) * dease_dt
        tt = t - pr.approach_s
        w = 2.0 * math.pi * pr.freq_hz
        return pr.amplitude_rev * w * math.cos(w * tt)

    # ── Frame construction ─────────────────────────────────────────────────────
    def frame(self, t: float, t_origin_us: int) -> Setpoint:
        """Build the Teensy-side knot Setpoint to transmit at ``t`` seconds since arm.

        ``u0/u1/u2`` are this and the next two 40 Hz knots; ``v0`` the current
        velocity FF; non-target axes packed 0.0; ``flags = 0x3``.
        """
        ax = self.params.axis
        T = self.seg_t
        p0 = self.position(t)
        p1 = self.position(t + T)
        p2 = self.position(t + 2.0 * T)
        v0 = self.velocity(t)

        def vec(val_at_axis: float) -> tuple:
            return tuple(val_at_axis if i == ax else 0.0 for i in range(self.n))

        return Setpoint(
            u0=vec(p0),
            u1=vec(p1),
            u2=vec(p2),
            v0=vec(v0),
            accel=(0.0,) * self.n,
            torque_ff=(0.0,) * self.n,
            flags=FLAG_HAS_U1 | FLAG_HAS_U2,
            t_origin_us=int(t_origin_us),
        )
