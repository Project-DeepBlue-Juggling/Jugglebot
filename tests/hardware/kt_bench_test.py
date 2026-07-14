#!/usr/bin/env python3
"""Torque-constant (Kt) measurement + torque_ff channel validation — bench leg.

Settles the **13 % torque-constant discrepancy** that currently blocks all feedforward
work on this robot, and de-risks the gravity feedforward being implemented right now.

    config/hardware_config.yaml:60                          0.0624    ("measured", Phase-2)
    config/ODrive config Files/odrive_pro_leg_config.json:152  0.055133  (== 8.27/150, ODrive's DEFAULT)
    datasheet SI                                           0.0637    (== 60/(2π·150 Kv))

The ODrive divides commanded torque by ITS ``torque_constant`` to get Iq, so any torque
feedforward we ship lands **+13 % hot** until this is settled.

Two modes
=========

``--mode kt`` — measure Kt, killing the stiction confound
---------------------------------------------------------
We suspect the historical 0.0624 is biased HIGH by stiction: if the Phase-2 fit held its
weights AT REST, Coulomb friction supported part of the load, so less current was needed
than gravity alone demands, so the inferred τ/iq was inflated. τ_c ≈ 1.1 A on this rig —
NOT small next to the 2-6 A a test mass draws.

Kill it by REVERSING DIRECTION. Drive the leg at constant velocity, first EXTENDING
(lifting the mass), then RETRACTING (lowering it), at the same speed. Friction flips sign
with velocity; gravity does not::

    (iq_up + iq_down)/2  =  τ_g / Kt      ← friction CANCELS EXACTLY.  The Kt signal.
    (iq_up − iq_down)/2  =  τ_f / Kt      ← a FREE, independent friction measurement.

Sweep ≥3 masses, fit ``iq_avg = a·m + b`` WITH an intercept (``b`` absorbs the leg's own
moving mass and cable preload — a forced-origin fit folds that into the slope and biases
Kt by exactly the amount we are trying to resolve), then ``Kt = g·r_spool / a``.

The friction cross-check is **load-bearing evidence, not a bonus**: the half-difference
must be ≈constant across masses and land near the independently-known τ_c ≈ 1.094 A
(``logbook/2026-04-27-friction-feedforward-bench-validation.md:79``, reproduced at
0.88-1.22 A by ``tools/probes/bench_leg_plant_id.py``). If it does, a quantity the Kt fit
never used corroborates the whole picture. If it does not, the harness says so LOUDLY and
the Kt number must not be believed.

``--mode torque_ff_check`` — validate the torque_ff channel end-to-end
----------------------------------------------------------------------
Arguably the more valuable half, because it de-risks the gravity FF *today*. Its #1 hazard
is a **SIGN ERROR** through the firmware's ``leg_sign`` negation
(``odrive_protocol.h:171`` negates ``torque_ff`` for legs) and the int16 ×10000 scaling.

Hold the leg at mid-stroke under normal position control with a mass on it, then step
``torque_ff`` through a small ladder. **Why a position hold makes this both safe and
exact**: the loop holds station, so the leg does not move — same position, same load,
therefore the same total mechanical torque is required, whatever it is. The ODrive sums
its controller output and the feedforward before dividing by ``torque_constant``::

    iq_total = (tau_PI + tau_ff) / Kt_odrive

so injecting ``tau_ff`` makes the position loop's own contribution fall by EXACTLY that
amount. **That shift IS the measurement** — and it is a DIFFERENTIAL one, so the
indeterminate at-rest stiction offset is common to every rung and cancels in the slope. It
is also inherently safe: the position loop actively resists any runaway, and every
commanded ``torque_ff`` is tiny against the 1.5 Nm ``torque_soft_max``.

**The ladder must stay INSIDE the static-friction band** (worst-case edge ≈ 0.049 Nm =
τ_c·Kt): "the loop holds station so the shift IS the measurement" is only true while
static friction lets the leg genuinely not move. An out-of-band rung moves the leg, the
position loop's integrator re-absorbs the feedforward at the new equilibrium, and that
rung biases the slope LOW — band-saturation model: 33–52 % for a 0.10 Nm rung;
observed 2026-07-14: full-fit 63–68 % low (the ±0.10 Nm rungs
saturated/re-absorbed while the in-band rungs showed a clean −20 to −22.5 A/Nm slope (right order vs the expected 18.14, ~17 % high)). Hence
the default ladder ±{0.010, 0.020, 0.035} Nm with 0.0 as the reference rung; rungs above
0.045 Nm draw a warning.

**Verdict quality gate**: Mode 2 refuses to state ANY sign/scale verdict unless the fit
clears R² ≥ 0.90 AND |slope|/σ ≥ 3. Below that it prints a NO CONCLUSION block naming
the specific failures and any probable-disturbance rung. (Born of the 2026-07-14 run:
0.8 kg at an odd angle, partially hand-supported, gave R² = −0.099 and a 1.8σ slope —
and the harness confidently shouted MUST-NEGATE/MISMATCH conclusions from it. The −0.05
Nm rung read iq ≈ 0.05 A — essentially unloaded — because someone was holding the mass.)

It pins down three things at once:

1. **SIGN** — does a POSITIVE wire ``torque_ff`` EXTEND (lift) or RETRACT (drop) the leg?
   Stated as a hard fact for the gravity-FF implementer.
2. **SCALE** — is the delivered current ``iq ≈ torque_ff / 0.055133``? Confirms (or
   refutes) that ``torque_ff`` is interpreted exactly like ``input_torque``, and pins the
   int16 ×10000 wire scaling.
3. **Kt cross-check** — with Mode 1's true Kt we can predict the MECHANICAL torque actually
   delivered, and hence the exact feedforward correction factor.

The sign convention trap this harness exists to not fall into
=============================================================
The can-bridge firmware reports **pos/vel and iq in OPPOSITE frames for legs**:

* ``can_buses.cpp:85-86`` — pos/vel ARE passed through ``leg_sign`` (negated) ⇒ telemetry
  ``pos_rev``/``vel_rps`` are Jugglebot **extension-positive** (the frame we stream ``u0`` in).
* ``can_buses.cpp:92`` — ``iq_measured`` is stored **RAW**, no ``leg_sign`` ⇒ it stays in the
  **ODrive** frame, in which positive == RETRACTION for a leg.

So the code-read prediction is that an EXTENDING torque reads as **NEGATIVE** iq. We do not
trust that — we MEASURE it. With the leg vertical and extending UP against a hanging mass,
the leg must produce an extending torque to hold it, so **the sign of iq at a loaded hold
IS the sign of "extending torque"**: gravity self-calibrates the frame, no wire assumption
needed. The harness asserts the measured sign against the code read and shouts on a
mismatch — either outcome is a real finding.

FIRMWARE: the BENCH_SYSID_BUILD variant is REQUIRED
===================================================
**No new firmware is needed, but the existing bench variant MUST be flashed.** Verified
against source:

* ``telemetry.cpp:52-64`` — the stock v3 DIAGNOSTIC (the ONLY carrier of ``iq_measured``) is
  **on-change gated**, and ``diag_changed`` keys on ``iq_setpoint`` (not ``iq_measured``) with
  a 0.5 A threshold, plus a 1 Hz forced refresh (``:22``). At a CONSTANT steady-state
  current — exactly what both modes produce — the setpoint does not change, so stock v3
  yields iq at **~1 Hz**. That is 2-3 samples per traverse: useless.
* ``telemetry.cpp:263-267`` — ``#if BENCH_SYSID_BUILD`` forces axis-0's DIAGNOSTIC every
  telemetry tick, and ``canbridge_config.h:93-96`` raises ``TELEM_RATE_HZ`` 100 → **250** in
  that build. That gives **250 Hz un-gated axis-0 iq**. This is what makes the measurement
  possible at all.
* ``leg_interp.cpp:185 → :218 → :366 → :499`` — ``torque_ff`` from the Setpoint frame IS
  already forwarded to the ODrive (as a zero-order hold of the base knot). No firmware
  change needed for Mode 2. Note it is ZEROED by the stroke clamp (``:407``) and during the
  re-enable recovery slew (``:479``) — so Mode 2 must sit mid-stroke and wait for the slew
  to converge, which the arm-settle already guarantees.
* ``canbridge_config.h:140-148`` — the bench build also sets ``SEGMENT_T_S`` 0.025 → **0.010**
  (100 Hz knots), so the harness MUST run ``--knot-hz 100``. That is this script's default;
  it refuses to run armed on a mismatch.
* Constant velocity needs no new primitive: a constant-slope knot ramp through the firmware's
  500 Hz Hermite IS constant velocity (``leg_interp.cpp:332-337``).

Usage
=====
::

    source ~/Desktop/PDJ_venv/venv/bin/activate

    # Plan only — no socket, no motor. Prints the mass budget + distinguishing power.
    python tests/hardware/kt_bench_test.py --mode kt --dry-run
    python tests/hardware/kt_bench_test.py --mode torque_ff_check --dry-run

    # Live (requires the BENCH_SYSID_BUILD firmware flashed):
    python tests/hardware/kt_bench_test.py --mode kt --home --masses 1.0,2.0,3.0
    python tests/hardware/kt_bench_test.py --mode torque_ff_check --hold-mass 2.0

RUN INSTRUCTIONS (operator checklist)
=====================================
* **Masses: 1.0 / 2.0 / 3.0 kg.** With only 2-3 bench weights available, three masses
  spread maximally is the best fit (span is where the slope's leverage lives — 1/2/3 kg
  beats any closer triple). Two masses would be exactly-determined (slope+intercept
  through 2 points, zero residual dof — no way to tell signal from disturbance), so the
  harness refuses fewer than 3.
* **The mass must hang FREE and VERTICAL, and NOBODY touches it (or the leg) during the
  rungs/traverses.** One supported rung poisons the whole fit: on the 2026-07-14 run the
  −0.05 Nm rung read iq ≈ 0.05 A — essentially an UNLOADED leg — because the operator
  was partially supporting the 0.8 kg mass, and that single rung helped wreck the fit
  (R² = −0.099). Hang it, step back, then press ENTER.
* **Declare the true mass** (``--masses`` / ``--hold-mass``): the current budget, the
  over-current abort margins and the sign inference all use it.
* **Bench-vs-platform SIGN context** (read before acting on any Mode-2 sign verdict):
  ODrive direction calibration is PER-DRIVE, and this bench drive's torque sign
  convention is OPPOSITE to the platform legs'. The 2026-04-27 friction bench work
  empirically needed ``--ff-sign -1`` on this rig, while the 2026-05-08 PLATFORM
  validation ran the standard un-negated chain and worked (a wrong sign would have
  doubled friction — that A/B genuinely discriminates). So "positive wire torque_ff
  RETRACTS" ON THIS BENCH is EXPECTED and does NOT mean the production gravity
  feedforward must be negated — the platform-validated sign stands.

Output: ``temp/probes/kt_bench_<ts>/`` — one CSV per traverse/rung + ``manifest.json``.

Exit codes: 0 normal · 1 pre-flight rejection · 2 mid-run fault/abort
"""
from __future__ import annotations

import argparse
import datetime
import math
import os
import signal
import sys
import time
from dataclasses import asdict
from typing import List, Optional

import numpy as np

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.dirname(os.path.dirname(_SCRIPT_DIR))
sys.path.insert(0, _SCRIPT_DIR)
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

import kt_lib as kt          # noqa: E402  — the pure logic (unit-tested in tests/motion)
import sysid_lib as sid      # noqa: E402  — shared bridge machinery (imported, never edited)
from bench_leg_sysid import (  # noqa: E402 — reuse the proven Path-BRIDGE transport
    BASELINE_GAINS,
    BRIDGE_TEENSY_IP,
    HARD_CURRENT_LIMIT_A,
    HARD_STROKE_CEIL_REV,
    BridgeSysID,
)

DEFAULT_TORQUE_FF_LADDER = ','.join(f"{t:+.3f}" for t in kt.DEFAULT_TORQUE_FF_LADDER_NM)

# ---------------------------------------------------------------------------
# Bench-build coupling. canbridge_config.h:146 sets SEGMENT_T_S = 0.010 under
# BENCH_SYSID_BUILD, so the knot stream MUST run at 100 Hz to match. Streaming 40 Hz
# knots into a 100 Hz-knot firmware starves the interpolator (it extrapolates, then
# decays velocity to zero — leg_interp.cpp Mode 2/3), which would silently corrupt the
# constant-velocity assumption the entire Kt measurement rests on.
# ---------------------------------------------------------------------------
BENCH_BUILD_KNOT_HZ = 100.0
BENCH_BUILD_TELEM_HZ = 250.0          # canbridge_config.h:94 TELEM_RATE_HZ under the variant
MIN_TELEM_HZ_FOR_KT = 150.0           # below this the flashed firmware is almost certainly
                                      # stock v3 (100 Hz telemetry, ~1 Hz gated iq)

# Traverse geometry. Deliberately LOW and SHORT: on any E-STOP/fault the motor
# de-energizes and THE MASS FALLS to the retracted hardstop — there is no software fix for
# that, so the only real mitigation is to keep the fall height small. 1.5 rev of travel is
# 107 mm; the top of the traverse sits 136 mm above the hardstop.
DEFAULT_TRAVERSE_LO_REV = 0.40
DEFAULT_TRAVERSE_HI_REV = 1.90
DEFAULT_REPS = 2                      # repeat each up/down pair; reps are averaged
DEFAULT_DWELL_S = 0.6                 # rest between traverses (let the leg settle)
DEFAULT_EDGE_DISCARD_S = 0.5          # throw away the accel/decel ends of each traverse

# Mode-2 hold. Mid-traverse, far from BOTH the harness stroke bounds and the firmware
# stroke clamp (STROKE_MIN_REV[0] ≈ 0.0709, STROKE_MAX_REV[0] ≈ 3.900) — the stroke clamp
# ZEROES torque_ff (leg_interp.cpp:407), which would silently kill the measurement.
DEFAULT_TFF_HOLD_REV = 1.20
DEFAULT_TFF_SETTLE_S = 1.2            # let the position loop's integrator absorb each rung
DEFAULT_TFF_MEASURE_S = 2.0           # steady window per rung
DEFAULT_TFF_HOLD_MASS_KG = 2.0

# Live over-current abort. The ODrive's own current_soft_max (10 A) is the hardware
# backstop; this is a software belt-and-braces that trips first and disarms cleanly.
# Debounced: kt.OVERCURRENT_TRIP_SAMPLES (3) CONSECUTIVE 250 Hz samples must exceed the
# limit — a single telemetry glitch or one-sample transient cannot abort a run.
IQ_ABORT_FRAC = 0.95

# Parking. PARK_DONE_TOL_REV: the leg counts as parked when at (or below) the bottom of
# the usable window within this — don't re-arm an already-parked leg for < 1.5 mm.
PARK_DONE_TOL_REV = 0.02
PARK_DESCENT_VEL_RPS = 0.3


class KtBench(BridgeSysID):
    """Kt + torque_ff harness over Path BRIDGE.

    Subclasses :class:`BridgeSysID` to inherit — rather than re-implement — the machinery
    that took a whole session to get right: the UDP transport, the stream-then-arm-then-
    settle dance (which closes the MPC_STALE race AND lets the firmware re-enable recovery
    slew converge before any motion), the startup guard-latch clear, the output-engagement
    verify, the guard-latch backoff state machine, the 250 Hz telemetry-frame CSV logger,
    and the manifest writer.

    Exactly two things are added:

    * ``_send_knot`` is overridden to carry a ``torque_ff`` — the base class hardcodes it to
      zero, and Mode 2 is entirely about that field.
    * ``_log_frame`` is overridden to add a live over-current abort. This is the cheapest
      correct hook: it runs on the RX thread for every telemetry frame, and setting
      ``_sigint`` routes into the base class's existing clean disarm path rather than
      inventing a second one.
    """

    def __init__(self, *, masses_kg: List[float], traverse_lo_rev: float,
                 traverse_hi_rev: float, traverse_vel_rps: float, reps: int,
                 dwell_s: float, edge_discard_s: float, tilt_deg: float,
                 tff_ladder_Nm: List[float], tff_hold_rev: float,
                 tff_hold_mass_kg: float, tff_settle_s: float, tff_measure_s: float,
                 assume_yes: bool,
                 accel_time_s: float = kt.DEFAULT_TRAVERSE_ACCEL_TIME_S, **kw):
        # Mode-1 approaches the traverse LOW end; Mode-2 holds mid-stroke. The base class
        # ramps to `center_rev` on bringup, so point it at whichever this run needs.
        super().__init__(**kw)
        self.masses_kg = list(masses_kg)
        self.traverse_lo_rev = float(traverse_lo_rev)
        self.traverse_hi_rev = float(traverse_hi_rev)
        self.traverse_vel_rps = float(traverse_vel_rps)
        self.reps = int(reps)
        self.dwell_s = float(dwell_s)
        self.edge_discard_s = float(edge_discard_s)
        self.tilt_deg = float(tilt_deg)
        self.tff_ladder_Nm = list(tff_ladder_Nm)
        self.tff_hold_rev = float(tff_hold_rev)
        self.tff_hold_mass_kg = float(tff_hold_mass_kg)
        self.tff_settle_s = float(tff_settle_s)
        self.tff_measure_s = float(tff_measure_s)
        self.assume_yes = bool(assume_yes)
        self.accel_time_s = float(accel_time_s)

        self._torque_ff_Nm = 0.0        # what _send_knot puts on the wire, this instant
        self._iq_abort_reason: Optional[str] = None
        self._iq_abort_limit = IQ_ABORT_FRAC * self.current_limit_a
        self._iq_trip = kt.OverCurrentLatch(self._iq_abort_limit)
        self._needs_park = False        # True while the leg is armed away from the bottom

        self._manifest['kt'] = {
            'purpose': 'settle the 13% torque-constant discrepancy blocking feedforward',
            'candidates': dict(kt.KT_CANDIDATES),
            'geometry': {
                'bench_leg_mm_per_rev': kt.BENCH_LEG_MM_PER_REV,
                'spool_radius_mm': kt.BENCH_SPOOL_RADIUS_MM,
                'source': 'single_leg_test.py:106 (measured on THIS rig; '
                          'platform legs are ~70.5 mm/rev — a different number)',
            },
            'masses_kg': self.masses_kg,
            'traverse': {
                'lo_rev': self.traverse_lo_rev, 'hi_rev': self.traverse_hi_rev,
                'vel_rps': self.traverse_vel_rps, 'reps': self.reps,
                'edge_discard_s': self.edge_discard_s,
                'accel_time_s': self.accel_time_s,
                'accel_rps2': (self.traverse_vel_rps / self.accel_time_s
                               if self.accel_time_s > 0 else float('inf')),
            },
            'tilt_deg': self.tilt_deg,
            'friction_reference': {
                'tau_c_A': kt.TAU_C_REF_A,
                'range_A': list(kt.TAU_C_REF_RANGE_A),
                'omega_s_rev_s': kt.OMEGA_S_REV_S,
                'source': 'logbook/2026-04-27-friction-feedforward-bench-validation.md:79,81',
            },
            'torque_ff': {
                'ladder_Nm': self.tff_ladder_Nm,
                'hold_rev': self.tff_hold_rev,
                'hold_mass_kg': self.tff_hold_mass_kg,
                'wire_scale_counts_per_Nm': kt.LEG_TOR_WIRE_SCALE,
            },
            'iq_sign_code_read_prediction': kt.IQ_EXTENSION_SIGN_PREDICTED,
        }

    # -- the two overrides ----------------------------------------------------

    def _send_knot(self, u0, u1, u2, v0, t_origin_us):
        """Identical to the base class, except it carries ``self._torque_ff_Nm``.

        The Setpoint's ``torque_ff`` is float32 **Nm** on the wire
        (``config/generated/udp_protocol.py:186``); the Teensy negates it via ``leg_sign``
        and scales it by 10000 into an int16 (``odrive_protocol.h:171-179``). We stream the
        value we WANT in the Jugglebot extension-positive frame and let the firmware do
        exactly what it does in production — the point of Mode 2 is to find out what that
        is, so the harness must not pre-compensate anything.
        """
        sp = self._Setpoint(
            u0=self._vec(u0), u1=self._vec(u1), u2=self._vec(u2), v0=self._vec(v0),
            accel=(0.0,) * self._nlegs,
            torque_ff=self._vec(self._torque_ff_Nm),
            flags=0x3, t_origin_us=int(t_origin_us))
        self._client.send_stream(int(self._MsgType.SETPOINT), sp.pack())

    def _log_frame(self, tm, now):
        """Base-class telemetry logging + a live over-current abort.

        Runs on the RX thread for every telemetry frame (250 Hz on the bench build), which
        makes it the earliest place we can see a current excursion. Setting ``_sigint``
        routes into the base class's existing clean disarm path (checked every knot in
        ``_stream_and_sample``) rather than inventing a second abort mechanism.

        Debounced via :class:`kt_lib.OverCurrentLatch`: the trip needs
        ``kt.OVERCURRENT_TRIP_SAMPLES`` (3) CONSECUTIVE over-limit samples (~12 ms), so a
        single telemetry glitch or a one-sample transient cannot kill a healthy run.
        """
        super()._log_frame(tm, now)
        d = self._cache['diag'].get(self.axis_id)
        if d is None:
            return
        iq = float(d.iq_measured)
        if self._iq_trip.observe(iq) and not self._iq_abort_reason:
            self._iq_abort_reason = (
                f"OVER-CURRENT: |iq| exceeded {self._iq_abort_limit:.2f} A "
                f"({IQ_ABORT_FRAC:.0%} of the {self.current_limit_a:.1f} A limit) on "
                f"{self._iq_trip.n_consecutive} consecutive 250 Hz samples "
                f"(last {abs(iq):.2f} A) — disarming. Is the mass heavier than "
                f"declared, or is the leg binding?")
            self._sigint = True

    def _took_iq_abort(self) -> bool:
        """Consume a live over-current trip: restore the real reason (the base class's
        stream loop overwrites ``_abort_reason`` with 'SIGINT') and report it."""
        if self._iq_abort_reason:
            self._abort_reason = self._iq_abort_reason
            return True
        return False

    # -- pre-flight -----------------------------------------------------------

    def validate_args(self) -> Optional[str]:
        err = super().validate_args()
        if err:
            return err
        if abs(self.knot_hz - BENCH_BUILD_KNOT_HZ) > 1e-6:
            return (f"--knot-hz is {self.knot_hz:.0f} but the BENCH_SYSID_BUILD firmware "
                    f"this harness REQUIRES runs SEGMENT_T_S = 0.010 s "
                    f"(canbridge_config.h:146) ⇒ 100 Hz knots. A mismatched knot rate "
                    f"starves the firmware interpolator, which extrapolates and then "
                    f"decays velocity to zero — silently destroying the constant-velocity "
                    f"assumption the whole Kt measurement rests on. Use --knot-hz 100.")
        for p in (self.traverse_lo_rev, self.traverse_hi_rev, self.tff_hold_rev):
            if not self.bounds.contains(p):
                return (f"position {p:.3f} rev is outside the usable stroke window "
                        f"[{self.bounds.lo_rev:.3f}, {self.bounds.hi_rev:.3f}]")
        if self.traverse_hi_rev <= self.traverse_lo_rev:
            return "traverse hi must be above lo"
        if self.traverse_vel_rps < kt.MIN_SAFE_TRAVERSE_VEL_RPS:
            return (f"traverse velocity {self.traverse_vel_rps} rev/s is at or below the "
                    f"stiction knee (omega_s = {kt.OMEGA_S_REV_S} rev/s, "
                    f"logbook 2026-04-27:81). The friction-cancellation method needs the "
                    f"KINETIC plateau, not the Stribeck dip where friction varies steeply "
                    f"with speed — use >= {kt.MIN_SAFE_TRAVERSE_VEL_RPS:.2f} rev/s "
                    f"(0.6 recommended).")
        if self.traverse_vel_rps > self.vel_cap_rps:
            return f"traverse velocity exceeds the session velocity cap {self.vel_cap_rps}"
        if self.accel_time_s <= 0.0:
            return "--accel-time must be > 0 (the traverse start/end accel ramp)"
        problems = kt.torque_ff_ladder_safe(self.tff_ladder_Nm)
        if problems:
            return "unsafe torque_ff ladder: " + "; ".join(problems)
        # Out-of-band rungs are SAFE but biased — warn, don't refuse (a deliberate
        # band-mapping run is a legitimate experiment).
        for w in kt.torque_ff_ladder_band_warnings(self.tff_ladder_Nm):
            print(f"  WARN: {w}")
        # The firmware stroke clamp ZEROES torque_ff (leg_interp.cpp:407), so a Mode-2 hold
        # anywhere near it would silently measure nothing.
        if not (0.30 <= self.tff_hold_rev <= 3.60):
            return (f"torque_ff hold {self.tff_hold_rev:.2f} rev is too close to the "
                    f"firmware stroke clamp (STROKE_MIN 0.071 / STROKE_MAX 3.900), which "
                    f"ZEROES torque_ff (leg_interp.cpp:407) — the measurement would "
                    f"silently read zero")
        budget = kt.current_budget(
            self.masses_kg, current_limit_A=self.current_limit_a,
            vel_rps=self.traverse_vel_rps, tilt_deg=self.tilt_deg,
            accel_rps2=self.traverse_vel_rps / self.accel_time_s)
        if not budget.ok:
            return ("mass set rejected by the current/conditioning budget "
                    "(accel phase included):\n    "
                    + "\n    ".join(budget.reasons)
                    + f"\n  Recommended: --masses "
                      f"{','.join(str(m) for m in kt.recommended_masses_kg())}")
        return None

    def _check_bench_firmware(self) -> bool:
        """Refuse to run armed on stock v3 firmware.

        The measured telemetry rate is the tell: the bench variant runs 250 Hz
        (``canbridge_config.h:94``), stock v3 runs 100 Hz (``:96``). Stock also gates
        ``iq_measured`` behind the on-change DIAGNOSTIC (``telemetry.cpp:52-64``, keyed on
        ``iq_setpoint`` with a 0.5 A threshold + 1 Hz refresh), so at the CONSTANT current
        both modes produce we would get ~1 Hz iq: 2-3 samples per traverse. The run would
        appear to work and produce a confidently wrong number, which is far worse than
        refusing — hence a hard pre-flight gate rather than a warning.
        """
        rate = self._measure_telemetry_rate()
        print(f"  telemetry: {rate.mean_hz:.0f} Hz mean / {rate.effective_hz:.0f} Hz "
              f"effective ({rate.n_intervals} intervals)")
        if rate.mean_hz < MIN_TELEM_HZ_FOR_KT:
            self._abort_reason = (
                f"telemetry is {rate.mean_hz:.0f} Hz — the BENCH_SYSID_BUILD firmware "
                f"(250 Hz, canbridge_config.h:94) does NOT appear to be flashed. Stock v3 "
                f"runs 100 Hz telemetry AND gates iq_measured behind the on-change "
                f"DIAGNOSTIC (telemetry.cpp:52-64: keyed on iq_setpoint, 0.5 A threshold, "
                f"1 Hz forced refresh) — at the constant steady-state current this test "
                f"produces, that is ~1 Hz of iq, i.e. 2-3 samples per traverse. REFUSING: "
                f"the run would look fine and give a confidently wrong Kt.\n"
                f"  Flash the bench variant:\n"
                f"    cd ros_ws/src/jugglebot/Teensy_code_canbridge && "
                f"pio run -e teensy41_bench_sysid -t upload")
            print(f"\n  ABORT: {self._abort_reason}", file=sys.stderr)
            return False
        # Confirm iq is actually arriving (a DIAGNOSTIC for our axis).
        deadline = time.perf_counter() + 2.0
        while time.perf_counter() < deadline:
            _, _, iq, _ = self._sample()
            if iq is not None and np.isfinite(iq):
                print(f"  iq channel live (iq = {iq:+.3f} A)")
                return True
            time.sleep(0.05)
        self._abort_reason = ("no DIAGNOSTIC/iq_measured for axis "
                              f"{self.axis_id} — is the bench ODrive powered and on CAN3?")
        print(f"  ABORT: {self._abort_reason}", file=sys.stderr)
        return False

    # -- operator interaction -------------------------------------------------

    def _prompt_mass(self, mass_kg: float) -> str:
        """Confirm the operator has hung THIS mass before we drive the leg.

        Returns ``'go'`` (run this mass), ``'skip'`` (skip this mass, continue with the
        next), or ``'quit'`` (abort the whole run cleanly, parking first). ``'q'``, EOF
        and Ctrl-C all mean QUIT — they used to be silently treated as skip-this-mass,
        which left an operator who wanted OUT cycling through every remaining prompt.
        """
        pred_up, pred_dn = kt.predicted_iq_up_down(
            mass_kg, kt.KT_ODRIVE_CONFIGURED, vel_rps=self.traverse_vel_rps,
            tilt_deg=self.tilt_deg)
        print(f"\n{'=' * 68}")
        print(f"  MASS: {mass_kg:.2f} kg   "
              f"(tau_g = {kt.gravity_torque_Nm(mass_kg, tilt_deg=self.tilt_deg):.4f} Nm)")
        print(f"  Expect |iq| up ~{abs(pred_up):.1f} A / down ~{abs(pred_dn):.1f} A "
              f"(worst-case Kt); limit {self.current_limit_a:.0f} A")
        print(f"  The leg will traverse {self.traverse_lo_rev:.2f} -> "
              f"{self.traverse_hi_rev:.2f} rev at {self.traverse_vel_rps:.2f} rev/s, "
              f"x{self.reps} up/down.")
        print("  Check: mass SECURE, hanging FREE and VERTICAL · nobody touches it "
              "while the leg moves ·")
        print("  padding under the mass (it FALLS on any E-STOP).")
        print("=" * 68)
        if self.assume_yes:
            print("  (--yes: proceeding without confirmation)")
            return 'go'
        try:
            ans = input(f"  Hang {mass_kg:.2f} kg and press ENTER (or 's' to skip this "
                        f"mass, 'q' to quit the run): ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print()
            return 'quit'
        if ans.startswith('q'):
            return 'quit'
        if ans.startswith('s'):
            return 'skip'
        return 'go'

    # -- MODE 1: Kt ------------------------------------------------------------

    def _park_low(self, from_rev: Optional[float] = None):
        """Lower the leg to the bottom of the usable window, then disarm — on EVERY exit
        path, including the abort ones.

        Called between masses, at the end of a stage, and from ``run()``'s cleanup. The
        point is that the operator handles the weights — and swaps them — with the leg at
        its LOWEST, so the fall distance if anything lets go is ~0.

        The abort paths need real work, not a fire-and-forget stream (the pre-fix
        version's descent was a silent no-op on every one of them, while the console
        printed "gentle descent"):

        * **the SIGINT latch is bypassed for the descent** — an over-current trip and
          Ctrl-C both latch ``self._sigint``, which makes ``_stream_and_sample`` break on
          its first knot. It is cleared here and restored after; a FRESH Ctrl-C during
          the descent re-latches it (the signal handler stays installed) and stops the
          stream — the operator's escape hatch stays live;
        * **re-arm if disarmed** — every abort path disarms, so the descent re-runs the
          bring-up + stream-then-arm-then-settle dance first;
        * **clear a guard latch (within budget)** — a latched E-STOP gates the output; a
          disarmed CLEAR_ERRORS is attempted via the same verified path used at startup;
        * **if parking is genuinely impossible** (latch will not clear, recovery budget
          exhausted, fatal fault, link down), say so EXPLICITLY — never pretend;
        * **the ORIGINAL abort diagnostic is preserved** — the descent itself sets
          ``_abort_reason`` (e.g. 'SIGINT'), which used to clobber the real diagnostic.
        """
        original_abort = self._abort_reason
        original_sigint = self._sigint
        self._torque_ff_Nm = 0.0
        self._needs_park = False
        cannot: Optional[str] = None
        target = self.bounds.lo_rev
        self._sigint = False
        try:
            pos, _, _, _ = self._sample()
            live = pos if pos is not None else from_rev
            if live is None:
                cannot = "no telemetry — the leg's position is unknown (link down?)"
            elif live <= target + PARK_DONE_TOL_REV:
                pass    # already at (or below) the bottom; just make sure we disarm
            else:
                fs = self._fault_state()
                cls = sid.classify_fault(int(fs)) if fs is not None else 'none'
                if cls in ('fatal', 'unknown'):
                    cannot = (f"fault {sid.fault_name(int(fs))} is fatal — ceding "
                              f"authority, not fighting it for a descent")
                elif cls == 'latching':
                    if self.guard.recoveries_used > self.guard.max_recoveries:
                        cannot = (f"guard latch {sid.fault_name(int(fs))} with the "
                                  f"CLEAR_ERRORS recovery budget exhausted "
                                  f"({self.guard.recoveries_used}"
                                  f"/{self.guard.max_recoveries})")
                    else:
                        print(f"  park: {sid.fault_name(int(fs))} latched — disarmed "
                              f"CLEAR_ERRORS before the descent")
                        if not self._startup_clear_latch():
                            cannot = (f"guard latch would not clear "
                                      f"({self._abort_reason})")
                if cannot is None:
                    # Re-arm (every abort path disarms; a completed stage is also
                    # descending from an armed hold — _warm_and_arm is safe either way).
                    self._bringup_closed_loop()
                    settle = self._warm_and_arm(live)
                    if settle.get('aborted') or settle.get('guard_latched'):
                        cannot = ("could not re-arm for the descent ("
                                  + (self._abort_reason or 'arm-settle latched') + ")")
                    else:
                        cur, _, _, _ = self._sample()
                        start = cur if cur is not None else live
                        print(f"  parking: {start:.2f} -> {target:.2f} rev "
                              f"(gentle descent)")
                        series = kt.shaped_constant_velocity_knots(
                            start, target,
                            vel_rps=min(PARK_DESCENT_VEL_RPS, self.traverse_vel_rps),
                            seg_t_s=self.seg_t_s,
                            lead_out_frames=int(0.3 / self.seg_t_s))
                        res = self._stream_and_sample(series, guard=True)
                        if res.get('aborted') or res.get('guard_latched'):
                            cannot = ("descent interrupted ("
                                      + (self._abort_reason or 'guard latch') + ")")
        except Exception as exc:  # noqa: BLE001 — parking must never mask the abort
            cannot = f"unexpected error during the park attempt: {exc}"
        finally:
            try:
                self._disarm()
            except Exception:  # noqa: BLE001
                pass
            # Keep a FRESH Ctrl-C latched; otherwise restore the pre-park latch state.
            self._sigint = self._sigint or original_sigint
            if cannot:
                pos, _, _, _ = self._sample()
                at = f"{pos:.2f} rev" if pos is not None else "an UNKNOWN position"
                print(f"\n  *** CANNOT PARK — {cannot} ***")
                print(f"  *** The leg is DISARMED at {at} and will NOT hold the load "
                      f"— SUPPORT THE MASS before touching anything. ***")
            # Never let the park attempt clobber the diagnostic that got us here.
            if original_abort:
                self._abort_reason = original_abort
                print(f"  (original abort diagnostic preserved: {original_abort})")

    def _run_traverse(self, mass_kg: float, rep: int, direction: str) -> Optional[dict]:
        """One constant-velocity traverse; returns the sampled arrays (or None on abort).

        The start/end are SHAPED (trapezoidal velocity, ``--accel-time`` ramps): an
        unshaped series steps 0 → 0.6 rev/s in one knot — a ~60 rev/s² accel demand
        ≈ +4.5 A of inertia current at 3 kg, enough to brush the over-current abort.
        The cruise portion (the only part the steady-window selector keeps) is identical.
        """
        up = direction == 'up'
        start = self.traverse_lo_rev if up else self.traverse_hi_rev
        end = self.traverse_hi_rev if up else self.traverse_lo_rev
        series = kt.shaped_constant_velocity_knots(
            start, end, vel_rps=self.traverse_vel_rps, seg_t_s=self.seg_t_s,
            accel_time_s=self.accel_time_s,
            lead_in_frames=int(0.3 / self.seg_t_s),
            lead_out_frames=int(0.3 / self.seg_t_s))
        v_cmd = kt.knots_achieved_velocity(series, self.seg_t_s)
        f, w, path = self._open_csv(f"kt_m{mass_kg:.2f}_rep{rep}_{direction}.csv")
        print(f"    {direction:>4}: {start:.2f} -> {end:.2f} rev  "
              f"(cmd {v_cmd:.3f} rev/s, {len(series) * self.seg_t_s:.1f} s)", end='',
              flush=True)
        res = self._stream_and_sample(series, guard=True, writer=w)
        f.close()
        if self._took_iq_abort() or res['aborted'] or res['guard_latched']:
            print("  ABORT")
            if not self._abort_reason:
                self._abort_reason = self._abort_diag(res)
            return None
        res['csv'] = os.path.basename(path)
        res['v_cmd_rps'] = v_cmd
        res['direction'] = direction
        return res

    def stage_kt(self) -> bool:
        print("\n=== MODE 1: torque constant via up/down friction cancellation ===")
        self.center_rev = self.traverse_lo_rev    # bring up AT the bottom of the traverse

        points: List[kt.MassPoint] = []
        per_mass_log: List[dict] = []

        for mass in self.masses_kg:
            choice = self._prompt_mass(mass)
            if choice == 'skip':
                print("  skipped.")
                continue
            if choice == 'quit':
                self._abort_reason = "operator quit ('q'/EOF) at the mass prompt"
                print("  quitting.")
                if self._needs_park:
                    self._park_low()
                return False

            approach = self._enter_hold_at_center()
            self._needs_park = True     # armed and moving — park on every exit from here
            if approach.get('aborted') or approach.get('guard_latched'):
                print(f"  ABORT during approach: {self._abort_reason}")
                self._park_low()
                return False

            up_stats: List[kt.TraverseStats] = []
            dn_stats: List[kt.TraverseStats] = []
            ok = True
            for rep in range(1, self.reps + 1):
                for direction in ('up', 'down'):
                    res = self._run_traverse(mass, rep, direction)
                    if res is None:
                        ok = False
                        break
                    st = kt.summarize_traverse(
                        res['t'], res['vel'], res['iq'], direction=direction,
                        v_target_rps=self.traverse_vel_rps,
                        edge_discard_s=self.edge_discard_s)
                    print(f"  iq = {st.iq_mean_A:+.3f} ± {st.iq_sem_A:.3f} A  "
                          f"(n={st.n_samples}, v={st.vel_mean_rps:+.3f} rev/s)"
                          + ("" if st.ok else "  << " + "; ".join(st.reasons)))
                    (up_stats if direction == 'up' else dn_stats).append(st)
                    if not st.ok:
                        ok = False
                    # Dwell between traverses — let the leg come fully to rest so the next
                    # traverse's acceleration transient starts from a known state.
                    self._stream_and_sample(
                        np.full(int(self.dwell_s / self.seg_t_s),
                                self.traverse_hi_rev if direction == 'up'
                                else self.traverse_lo_rev), guard=True)
                if not ok:
                    break

            self._park_low()

            if not ok or not up_stats or not dn_stats:
                print(f"  mass {mass:.2f} kg FAILED — aborting the run "
                      f"(a partial mass set cannot be fitted honestly)")
                return False

            # Average the reps, then combine up/down. Averaging reps BEFORE combining keeps
            # the up/down pairing intact, which is what makes friction cancel.
            up_mean = _mean_stats(up_stats, 'up')
            dn_mean = _mean_stats(dn_stats, 'down')
            pt = kt.combine_traverses(mass, up_mean, dn_mean)
            points.append(pt)
            print(f"  --> iq_avg = {pt.iq_avg_A:+.3f} ± {pt.iq_avg_sem_A:.3f} A "
                  f"(gravity, friction cancelled)")
            print(f"      iq_halfdiff = {pt.iq_halfdiff_A:+.3f} A "
                  f"(|{abs(pt.iq_halfdiff_A):.3f}| A vs tau_c {kt.TAU_C_REF_A:.2f} A)")
            per_mass_log.append({
                'mass_kg': mass, 'point': asdict(pt),
                'up': [asdict(s) for s in up_stats],
                'down': [asdict(s) for s in dn_stats],
            })

        if len(points) < 3:
            self._abort_reason = (f"only {len(points)} mass points completed — need >= 3 "
                                  f"to fit slope + intercept")
            print(f"\n  ABORT: {self._abort_reason}")
            return False

        self._report_kt(points, per_mass_log)
        return True

    def _report_kt(self, points: List[kt.MassPoint], per_mass_log: List[dict]):
        fit = kt.fit_kt(points, tilt_deg=self.tilt_deg)
        verdict = kt.classify_kt(fit)
        friction = kt.friction_consistency(points)
        sign = kt.infer_extension_iq_sign(fit)
        signs_agree = kt.slope_friction_sign_agree(fit, points)

        print("\n" + "=" * 72)
        print("  RESULT — torque constant")
        print("=" * 72)
        print(f"  {'mass':>6} {'iq_up':>9} {'iq_down':>9} {'iq_avg':>9} "
              f"{'halfdiff':>9} {'resid':>8}")
        for p, r in zip(points, fit.residuals_A or [float('nan')] * len(points)):
            print(f"  {p.mass_kg:6.2f} {p.iq_up_A:+9.3f} {p.iq_down_A:+9.3f} "
                  f"{p.iq_avg_A:+9.3f} {p.iq_halfdiff_A:+9.3f} {r:+8.4f}")

        print(f"\n  fit: iq_avg = a*m + b   (intercept MANDATORY — it absorbs the leg's")
        print(f"       own moving mass + cable preload; forcing through the origin would")
        print(f"       fold that into the slope and bias Kt by the amount we're measuring)")
        print(f"    a = {fit.slope_A_per_kg:+.4f} ± {fit.slope_sigma:.4f} A/kg")
        print(f"    b = {fit.intercept_A:+.4f} ± {fit.intercept_sigma:.4f} A"
              f"   <- the leg's own load offset")
        print(f"    R2 = {fit.r_squared:.5f}   (n={fit.n_points}, dof={fit.dof})")

        print(f"\n  Kt = g * r_spool / |a| = {fit.kt_nm_per_a:.5f} "
              f"± {fit.kt_sigma:.5f} Nm/A")
        print(f"     95% CI: [{fit.kt_ci95[0]:.5f}, {fit.kt_ci95[1]:.5f}]")
        print(f"\n  Distance to each candidate:")
        for name, val in kt.KT_CANDIDATES.items():
            s = verdict.sigma_to.get(name, float('nan'))
            tag = 'EXCLUDED' if name in verdict.excluded else 'consistent'
            print(f"    {name:22s} {val:.5f}  {s:6.1f} sigma  {tag}")
        print(f"\n  {verdict.summary}")

        print(f"\n  FRICTION CROSS-CHECK (load-bearing — a free, independent measurement")
        print(f"  the Kt fit never used):")
        print(f"    |(iq_up - iq_down)/2| = {friction.halfdiff_mean_A:.3f} "
              f"± {friction.halfdiff_std_A:.3f} A across masses")
        print(f"    reference tau_c        = {friction.tau_c_ref_A:.3f} A "
              f"(range {kt.TAU_C_REF_RANGE_A[0]}-{kt.TAU_C_REF_RANGE_A[1]} A)")
        print(f"    ratio = {friction.ratio_to_ref:.2f}   -> "
              f"{'COHERENT' if friction.ok else '*** INCONSISTENT ***'}")
        for r in friction.reasons:
            print(f"    !! {r}")

        print(f"\n  SIGN: an extending torque reads as {sign.iq_extension_sign:+d} iq "
              f"in the reported frame")
        print(f"    {sign.note}")
        if not signs_agree:
            print("    *** the fitted slope and the friction half-difference DISAGREE in "
                  "sign — the up/down traverses may be mislabelled. Kt is NOT trustworthy.")

        print(f"\n  WHAT THIS MEANS:")
        for line in self._kt_implications(fit, verdict, friction, signs_agree):
            print(f"    {line}")
        print("=" * 72)

        self._manifest['stages']['kt'] = {
            'points': [asdict(p) for p in points],
            'per_mass': per_mass_log,
            'fit': asdict(fit),
            'verdict': asdict(verdict),
            'friction_check': asdict(friction),
            'sign_inference': asdict(sign),
            'slope_friction_signs_agree': signs_agree,
            'implications': self._kt_implications(fit, verdict, friction, signs_agree),
        }

    @staticmethod
    def _kt_implications(fit, verdict, friction, signs_agree) -> List[str]:
        """Translate the number into the decision it drives. This is the part the operator
        actually needs — 'Kt = 0.0587' is not actionable on its own."""
        out: List[str] = []
        if not friction.ok or not signs_agree:
            out.append("The friction cross-check FAILED, so the physical model is not "
                       "confirmed. DO NOT act on this Kt. Re-run after checking: leg "
                       "vertical? mass secure? both traverses at the same speed?")
            return out
        kt_v = fit.kt_nm_per_a
        ratio = kt_v / kt.KT_ODRIVE_CONFIGURED
        if 'odrive_configured' in verdict.consistent and \
                'historical_measured' in verdict.excluded:
            out.append("Kt agrees with the ODrive's CONFIGURED torque_constant (0.055133) "
                       "and EXCLUDES the historical 0.0624.")
            out.append("=> The historical 0.0624 was WRONG — consistent with the stiction "
                       "bias hypothesis (an at-rest measurement reads high).")
            out.append("=> ACTION: fix hardware_config.yaml:60 to 0.0551. The ODrive is "
                       "already right, so the torque_ff channel is ALREADY CALIBRATED and "
                       "a commanded Nm lands as that Nm. Feedforward is unblocked as-is.")
        elif 'historical_measured' in verdict.consistent and \
                'odrive_configured' in verdict.excluded:
            out.append("Kt agrees with the historical 0.0624 and EXCLUDES the ODrive's "
                       "configured 0.055133.")
            out.append("=> The stiction-bias hypothesis is REFUTED; the historical bench "
                       "fit was right and the ODrive is simply using its untouched "
                       "8.27/Kv default.")
            out.append(f"=> ACTION: every commanded torque lands {(ratio - 1) * 100:+.0f}% "
                       f"HOT (the ODrive divides by 0.0551, the motor multiplies by "
                       f"{kt_v:.4f}). EITHER set the ODrive's torque_constant to "
                       f"{kt_v:.5f}, OR scale every torque_ff by "
                       f"{kt.KT_ODRIVE_CONFIGURED / kt_v:.4f} before it goes on the wire. "
                       f"Prefer fixing the ODrive config — one place, not every call site.")
        elif 'datasheet_si' in verdict.consistent and \
                'odrive_configured' in verdict.excluded:
            out.append("Kt agrees with the SI datasheet value (0.0637) and excludes the "
                       "ODrive's 0.055133.")
            out.append("=> The motor behaves as its 150 Kv rating predicts; the ODrive's "
                       "8.27/Kv default is the outlier (that formula carries a ~4% "
                       "constant that does not apply to this motor).")
            out.append(f"=> ACTION: set the ODrive's torque_constant to {kt_v:.5f}; until "
                       f"then torque commands land {(ratio - 1) * 100:+.0f}% hot.")
        elif len(verdict.excluded) == 3:
            out.append(f"Kt = {kt_v:.5f} EXCLUDES ALL THREE candidates. That is a real "
                       f"finding, not an error — but confirm the geometry first: is the "
                       f"leg's mm/rev really {kt.BENCH_LEG_MM_PER_REV} (it is the bench "
                       f"leg, NOT a ~70.5 mm/rev platform leg)? Is it vertical?")
            out.append(f"=> If the geometry holds, the true Kt is {kt_v:.5f} and BOTH "
                       f"hardware_config.yaml AND the ODrive config are wrong.")
        else:
            out.append(f"This run does NOT separate the candidates "
                       f"({', '.join(verdict.consistent)} all remain consistent). The "
                       f"uncertainty (±{fit.kt_sigma:.5f}) is too large.")
            out.append("=> ACTION: add masses, widen the mass span, lengthen the traverse "
                       "(more steady samples), or raise --reps.")
        out.append(f"NOTE: separating 0.0624 from 0.0637 (only 2% apart) is NOT what this "
                   f"rig can do — but it is also not what blocks feedforward. The 13% "
                   f"question (0.0551 vs 0.0624) is the one that matters, and this run "
                   f"answers it.")
        return out

    # -- MODE 2: torque_ff channel --------------------------------------------

    def stage_torque_ff_check(self) -> bool:
        print("\n=== MODE 2: torque_ff channel validation (sign + scale) ===")
        self.center_rev = self.tff_hold_rev

        print(f"\n  Hold at {self.tff_hold_rev:.2f} rev with {self.tff_hold_mass_kg:.2f} kg "
              f"on the leg, stepping torque_ff through:")
        print(f"    {', '.join(f'{t:+.3f}' for t in self.tff_ladder_Nm)} Nm")
        print(f"  The position loop holds station, so its own torque contribution must FALL")
        print(f"  by exactly what the feedforward supplies. That shift IS the measurement —")
        print(f"  and it is differential, so the indeterminate at-rest stiction offset")
        print(f"  cancels in the slope. Expect |d(iq)/d(torque_ff)| = 1/0.055133 = "
              f"{1.0 / kt.KT_ODRIVE_CONFIGURED:.2f} A/Nm.")
        print(f"  Every rung stays INSIDE the static-friction band (worst-case edge "
              f"{kt.TFF_STATIC_BAND_MIN_NM:.3f} Nm):")
        print(f"  that is what keeps the leg truly stationary — an out-of-band rung "
              f"moves the leg and the")
        print(f"  integrator re-absorbs the feedforward, biasing the slope LOW "
              f"(band model: 33-52% for a 0.10 Nm rung; observed 2026-07-14: 63-68%) "
              f"(the 2026-07-14 run).")
        for w in kt.torque_ff_ladder_band_warnings(self.tff_ladder_Nm):
            print(f"  WARN: {w}")
        if not self.assume_yes:
            print(f"\n  Check: {self.tff_hold_mass_kg:.2f} kg SECURE on the leg, hanging "
                  f"FREE and VERTICAL · leg VERTICAL ·")
            print(f"  padding under the mass · NOBODY touches the mass or the leg during "
                  f"the rungs")
            print(f"  (one supported rung poisons the fit — the 2026-07-14 run's "
                  f"hand-held -0.05 Nm rung read iq ~ 0.05 A, an unloaded leg).")
            try:
                if input("  Press ENTER to begin (or 'q' to quit): "
                         ).strip().lower().startswith('q'):
                    self._abort_reason = "operator quit ('q') at the Mode-2 prompt"
                    return False
            except (EOFError, KeyboardInterrupt):
                print()
                self._abort_reason = "operator quit (EOF/Ctrl-C) at the Mode-2 prompt"
                return False

        approach = self._enter_hold_at_center()
        self._needs_park = True         # armed and moving — park on every exit from here
        if approach.get('aborted') or approach.get('guard_latched'):
            print(f"  ABORT during approach: {self._abort_reason}")
            self._park_low()
            return False

        # The re-enable recovery slew ZEROES torque_ff while it runs (leg_interp.cpp:479).
        # _warm_and_arm's flat arm-settle already converges it, but hold flat a little
        # longer before the first rung so we can never measure a slew-suppressed zero.
        print(f"\n  settling (the firmware recovery slew zeroes torque_ff while it runs "
              f"— leg_interp.cpp:479)")
        self._stream_and_sample(
            np.full(int(1.0 / self.seg_t_s), self.tff_hold_rev), guard=True)

        points: List[kt.TorqueFfPoint] = []
        print(f"\n  {'tff(Nm)':>9} {'wire(Nm)':>9} {'iq(A)':>12} {'pos(rev)':>10} "
              f"{'d_iq vs 0':>10}")
        iq_at_zero: Optional[float] = None

        for tff in self.tff_ladder_Nm:
            wire = kt.wire_quantized_torque_Nm(tff)   # what the int16 x10000 actually sends
            self._torque_ff_Nm = tff

            # Settle: let the position loop's integrator absorb the new feedforward, so the
            # measured window is a true steady state rather than the transient.
            self._stream_and_sample(
                np.full(int(self.tff_settle_s / self.seg_t_s), self.tff_hold_rev),
                guard=True)
            if self._took_iq_abort():
                print(f"\n  ABORT: {self._abort_reason}")
                self._torque_ff_Nm = 0.0
                self._park_low()
                return False

            f, w, path = self._open_csv(f"tff_{tff:+.3f}Nm.csv")
            res = self._stream_and_sample(
                np.full(int(self.tff_measure_s / self.seg_t_s), self.tff_hold_rev),
                guard=True, writer=w)
            f.close()

            if self._took_iq_abort() or res['aborted'] or res['guard_latched']:
                if not self._abort_reason:
                    self._abort_reason = self._abort_diag(res)
                print(f"\n  ABORT at torque_ff={tff:+.3f} Nm: {self._abort_reason}")
                self._torque_ff_Nm = 0.0
                self._park_low()
                return False

            iq = np.asarray(res['iq'], float)
            pos = np.asarray(res['pos'], float)
            iq = iq[np.isfinite(iq)]
            pos = pos[np.isfinite(pos)]
            if iq.size < 20:
                self._torque_ff_Nm = 0.0
                self._park_low()
                self._abort_reason = (
                    f"only {iq.size} iq samples at torque_ff={tff:+.3f} Nm — the "
                    f"BENCH_SYSID_BUILD firmware gives 250 Hz; stock v3 gates iq to ~1 Hz")
                print(f"\n  ABORT: {self._abort_reason}")
                return False

            mean = float(np.mean(iq))
            n_eff = kt.effective_sample_size(iq)
            sem = float(np.std(iq, ddof=1)) / max(1.0, np.sqrt(n_eff))
            pt = kt.TorqueFfPoint(torque_ff_Nm=wire, iq_mean_A=mean, iq_sem_A=sem,
                                  pos_mean_rev=float(np.mean(pos)) if pos.size else float('nan'),
                                  n_samples=int(iq.size))
            points.append(pt)
            if abs(tff) < 1e-9:
                iq_at_zero = mean
            d = (mean - iq_at_zero) if iq_at_zero is not None else float('nan')
            print(f"  {tff:+9.3f} {wire:+9.4f} {mean:+8.3f}±{sem:.3f} "
                  f"{pt.pos_mean_rev:10.4f} {d:+10.3f}")

        self._torque_ff_Nm = 0.0
        self._stream_and_sample(
            np.full(int(0.5 / self.seg_t_s), self.tff_hold_rev), guard=True)
        self._park_low()

        self._report_torque_ff(points)
        return True

    def _report_torque_ff(self, points: List[kt.TorqueFfPoint]):
        fit = kt.fit_torque_ff(points)
        # THE VERDICT GATE (2026-07-14 lesson): no sign/scale claim from an
        # untrustworthy fit. R² >= 0.90 and a >= 3-sigma slope, or NO CONCLUSION.
        quality = kt.assess_tff_fit_quality(fit, points)

        # The gravity-calibrated extension sign. Prefer Mode 1's (if this run did both);
        # otherwise derive it from the loaded hold itself: the leg must produce an
        # EXTENDING torque to hold the mass up, so the sign of the hold current IS the
        # extension sign — the same self-calibrating reference, from a single hold.
        kt_stage = self._manifest['stages'].get('kt') or {}
        sign_from_kt = (kt_stage.get('sign_inference') or {}).get('iq_extension_sign', 0)
        if sign_from_kt in (1, -1):
            ext_sign = int(sign_from_kt)
            sign_src = "Mode 1's gravity-loaded slope"
        elif self.tff_hold_mass_kg > 0 and math.isfinite(fit.intercept_A) \
                and abs(fit.intercept_A) > 0.2:
            ext_sign = 1 if fit.intercept_A > 0 else -1
            sign_src = (f"the loaded hold itself (the leg must push UP to hold "
                        f"{self.tff_hold_mass_kg:.1f} kg, so the sign of the hold current "
                        f"IS the extension sign)")
        else:
            ext_sign = 0
            sign_src = "unavailable (no mass on the leg? run --mode kt first)"

        verdict = kt.classify_torque_ff(fit, ext_sign, quality=quality)

        print("\n" + "=" * 72)
        print("  RESULT — torque_ff channel")
        print("=" * 72)
        print(f"  fit: iq = s*torque_ff + c")
        print(f"    s = {fit.slope_A_per_Nm:+.3f} ± {fit.slope_sigma:.3f} A/Nm")
        print(f"    c = {fit.intercept_A:+.3f} ± {fit.intercept_sigma:.3f} A"
              f"   <- the loaded-hold current (carries the at-rest stiction offset;")
        print(f"                                     fitted out, never used)")
        print(f"    R2 = {fit.r_squared:.5f}   (n={fit.n_points}, dof={fit.dof})")
        print(f"    verdict gate (R2 >= {quality.min_r2:.2f} AND |s|/sigma >= "
              f"{quality.min_t_stat:.0f}): "
              f"{'PASS' if quality.trustworthy else '*** FAIL -> NO CONCLUSION ***'}"
              f"   (t = {quality.t_stat:.1f})")
        print(f"\n  extension iq sign = {ext_sign:+d}, from {sign_src}")
        print()
        for line in verdict.lines:
            print(f"  {line}")
        for r in fit.reasons:
            print(f"  !! {r}")

        print(f"\n  WHAT THIS MEANS FOR THE GRAVITY FEEDFORWARD:")
        for line in self._tff_implications(fit, verdict):
            print(f"    {line}")
        print("=" * 72)

        self._manifest['stages']['torque_ff_check'] = {
            'points': [asdict(p) for p in points],
            'fit': asdict(fit),
            'fit_quality': asdict(quality),
            'verdict': asdict(verdict),
            'extension_iq_sign': ext_sign,
            'extension_iq_sign_source': sign_src,
            'implications': self._tff_implications(fit, verdict),
        }

    def _tff_implications(self, fit, verdict) -> List[str]:
        out: List[str] = []
        if verdict.no_conclusion:
            out.append("NO CONCLUSION: this run says NOTHING about the torque_ff sign "
                       "or scale — do not act on it, in either direction.")
            out.append("Fix the setup and re-run: hang the declared mass FREE and "
                       "VERTICAL from the leg; NOBODY touches the mass or the leg "
                       "during the rungs (one supported rung poisons the fit — the "
                       "2026-07-14 hand-held -0.05 Nm rung read iq ~ 0.05 A, an "
                       "unloaded leg); keep every rung inside the static-friction band "
                       f"(|tff| <= {kt.TFF_BAND_WARN_NM:.3f} Nm — the default ladder).")
            return out
        if not verdict.channel_live:
            out.append("The torque_ff channel is DEAD — commanded feedforward produced no "
                       "current. DO NOT ship a gravity FF.")
            out.append("Check: is the leg mid-stroke? (leg_interp.cpp:407 — the stroke "
                       "clamp ZEROES torque_ff). Did the recovery slew still have the "
                       "output? (:479 also zeroes it).")
            return out
        if verdict.positive_tff_extends is True:
            out.append("SIGN (THIS BENCH): a POSITIVE torque_ff EXTENDS (lifts) this "
                       "bench leg — matching the naive code read (odrive_protocol.h:171)"
                       ", but the OPPOSITE of the 2026-04-27 bench friction-FF finding "
                       "(--ff-sign -1 on this rig). Re-check the drive config before "
                       "leaning on it.")
            out.append("BENCH != PLATFORM: " + kt.BENCH_VS_PLATFORM_SIGN_NOTE)
        elif verdict.positive_tff_extends is False:
            out.append("SIGN (THIS BENCH): a POSITIVE torque_ff RETRACTS (drops) this "
                       "bench leg — EXPECTED on this rig, NOT a production sign error, "
                       "and NOT a reason to negate the production gravity feedforward.")
            out.append("BENCH != PLATFORM: " + kt.BENCH_VS_PLATFORM_SIGN_NOTE)
        else:
            out.append("SIGN: not determined — put a mass on the leg, or run --mode kt "
                       "first to calibrate the extension sign from gravity.")
        if verdict.scale_ok:
            out.append(f"SCALE: the ODrive really does use its configured torque_constant "
                       f"({kt.KT_ODRIVE_CONFIGURED:.5f}) to turn commanded Nm into current, "
                       f"and the int16 x10000 wire scaling round-trips. torque_ff behaves "
                       f"exactly like input_torque, as expected.")
            out.append(f"=> The MECHANICAL torque delivered per commanded Nm is "
                       f"Kt_true/{kt.KT_ODRIVE_CONFIGURED:.5f}. With the historical "
                       f"Kt=0.0624 that is "
                       f"{kt.mechanical_torque_delivered_Nm(1.0, 0.0624):.3f} Nm per 1.0 Nm "
                       f"commanded (+13%); with Kt=0.0551 it is 1.000. RUN MODE 1 to find "
                       f"out which.")
        else:
            out.append(f"SCALE: MISMATCH — the delivered current is NOT torque_ff / "
                       f"torque_constant. A gravity FF would land "
                       f"{(abs(fit.slope_A_per_Nm) * kt.KT_ODRIVE_CONFIGURED - 1) * 100:+.0f}% "
                       f"off from this alone, before the Kt question is even considered. "
                       f"Find out why before shipping.")
        return out

    # -- dry-run plan ---------------------------------------------------------

    def print_plan(self, modes: List[str]):
        print("\n" + "=" * 72)
        print("  DRY-RUN — Kt / torque_ff bench plan (no socket, no motor)")
        print("=" * 72)
        print(f"  axis={self.axis_id}  teensy={BRIDGE_TEENSY_IP}  "
              f"knots={self.knot_hz:.0f} Hz (seg_t={self.seg_t_s * 1e3:.0f} ms)")
        print(f"  caps: stroke<={self.stroke_cap_rev} rev, vel<={self.vel_cap_rps} rev/s, "
              f"curr<={self.current_limit_a} A")
        print(f"  bounds=[{self.bounds.lo_rev:.2f}, {self.bounds.hi_rev:.2f}] rev")
        print(f"  gains at bringup: pos={BASELINE_GAINS.pos_gain} "
              f"vel={BASELINE_GAINS.vel_gain} vel_int={BASELINE_GAINS.vel_int_gain} "
              f"(production baseline)")

        print("\n  FIRMWARE: BENCH_SYSID_BUILD variant is REQUIRED.")
        print("    Stock v3 gates iq_measured behind the on-change DIAGNOSTIC")
        print("    (telemetry.cpp:52-64 — keyed on iq_setpoint, 0.5 A threshold, 1 Hz")
        print("    refresh). At the CONSTANT current this test produces that is ~1 Hz of")
        print("    iq: 2-3 samples per traverse. The bench variant forces axis-0's")
        print("    DIAGNOSTIC every telemetry tick at 250 Hz (telemetry.cpp:263-267 +")
        print("    canbridge_config.h:94). The harness measures the telemetry rate at")
        print("    startup and REFUSES to run below "
              f"{MIN_TELEM_HZ_FOR_KT:.0f} Hz.")
        print("      cd ros_ws/src/jugglebot/Teensy_code_canbridge")
        print("      pio run -e teensy41_bench_sysid -t upload")

        notes = kt.recommended_velocity_notes(
            self.traverse_vel_rps, mass_kg=max(self.masses_kg) if self.masses_kg else 3.0)
        print(f"\n  VELOCITY: {self.traverse_vel_rps:.2f} rev/s "
              f"({notes['vel_linear_mm_s']:.0f} mm/s)")
        print(f"    {notes['vel_over_omega_s']:.1f}x the stiction knee "
              f"(omega_s = {kt.OMEGA_S_REV_S} rev/s) -> on the KINETIC friction plateau,")
        print(f"    not in the Stribeck dip. Same |v| both directions, so the viscous term")
        print(f"    cancels in the average too.")
        print(f"    back-EMF {notes['back_emf_V']:.2f} V "
              f"({notes['back_emf_frac_of_bus']:.1%} of a 21 V bus); regen on the descent")
        print(f"    {notes['regen_W']:.1f} W. Neither binds: the 21 V bench PSU is FINE at")
        print(f"    this speed. (21 V is 'back-EMF marginal' near the leg's ~47 rev/s top")
        print(f"    speed, nowhere near a system-ID crawl.)")

        if 'kt' in modes:
            accel_rps2 = self.traverse_vel_rps / self.accel_time_s
            budget = kt.current_budget(
                self.masses_kg, current_limit_A=self.current_limit_a,
                vel_rps=self.traverse_vel_rps, tilt_deg=self.tilt_deg,
                accel_rps2=accel_rps2)
            print(f"\n  [MODE 1: kt]  traverse {self.traverse_lo_rev:.2f} -> "
                  f"{self.traverse_hi_rev:.2f} rev, x{self.reps} up/down per mass")
            dur = kt.traverse_duration_s(self.traverse_lo_rev, self.traverse_hi_rev,
                                         self.traverse_vel_rps)
            steady = dur - 2 * self.edge_discard_s
            print(f"    each traverse: {dur:.1f} s, of which {steady:.1f} s is steady "
                  f"(after {self.edge_discard_s}s edge discard)")
            print(f"    -> ~{int(steady * BENCH_BUILD_TELEM_HZ)} iq samples per traverse "
                  f"at {BENCH_BUILD_TELEM_HZ:.0f} Hz")
            print(f"    start/end SHAPED: {self.accel_time_s:.2f} s trapezoid ramp -> "
                  f"{accel_rps2:.1f} rev/s² accel demand")
            print(f"    (an UNSHAPED series steps to speed in one knot: ~"
                  f"{self.traverse_vel_rps / self.seg_t_s:.0f} rev/s² ≈ "
                  f"+{kt.accel_current_A(max(self.masses_kg) if self.masses_kg else 3.0, self.traverse_vel_rps / self.seg_t_s, kt_nm_per_a=budget.worst_case_kt):.1f} A "
                  f"at the heaviest mass — that brushes the abort)")
            print(f"    fall height if the drive faults at the top: "
                  f"{self.traverse_hi_rev * kt.BENCH_LEG_MM_PER_REV:.0f} mm "
                  f"-- PAD UNDER THE MASS")

            print(f"\n    CURRENT BUDGET incl. the accel phase (at the WORST-CASE "
                  f"smallest candidate Kt = {budget.worst_case_kt:.5f},")
            print(f"    because a smaller Kt draws MORE current — we cannot be surprised "
                  f"by our own answer):")
            print(f"      {'mass':>6} {'tau_g':>8} {'iq_up':>7} {'iq_dn':>7} "
                  f"{'iq_acc':>7} {'iq_pk':>7} {'headroom':>9} {'tg/tc':>6}"
                  f"   iq_avg: 0.0551 vs 0.0624")
            for r in budget.rows:
                a1 = kt.gravity_torque_Nm(r.mass_kg, tilt_deg=self.tilt_deg) / kt.KT_ODRIVE_CONFIGURED
                a2 = kt.gravity_torque_Nm(r.mass_kg, tilt_deg=self.tilt_deg) / kt.KT_HISTORICAL_MEASURED
                print(f"      {r.mass_kg:6.2f} {r.tau_g_Nm:8.4f} {r.iq_up_A:7.2f} "
                      f"{r.iq_down_A:7.2f} {r.iq_accel_A:7.2f} {r.iq_peak_A:7.2f} "
                      f"{r.headroom_A:9.2f} {r.tau_g_over_tau_c:6.2f}"
                      f"   {a1:5.2f} A vs {a2:5.2f} A  (D={a1 - a2:.2f} A)")
            for w in budget.reasons:
                print(f"      !! {w}")
            print(f"      budget: {'OK' if budget.ok else '*** REJECTED ***'}")

            # Distinguishing power. 0.05 A per-point SEM is pessimistic — at 250 Hz with a
            # ~1 s steady window we average hundreds of samples even after the
            # autocorrelation correction knocks n_eff down.
            dp = kt.distinguishing_power(self.masses_kg, iq_point_sigma_A=0.05,
                                         tilt_deg=self.tilt_deg)
            print(f"\n    DISTINGUISHING POWER (assuming a pessimistic 0.05 A SEM per mass):")
            print(f"      slope SE = {dp.slope_se_A_per_kg:.4f} A/kg")
            for pair, s in dp.pairs.items():
                mark = 'RESOLVABLE' if s >= 3.0 else 'not resolvable'
                print(f"      {pair:48s} {s:5.1f} sigma  {mark}")
            print(f"      -> the 13% question (0.0551 vs 0.0624) — the one that actually")
            print(f"         blocks feedforward — is "
                  f"{'ANSWERABLE' if dp.ok else 'NOT answerable with this mass set'}.")
            print(f"      -> 0.0624 vs 0.0637 (2% apart) is NOT resolvable here. That is")
            print(f"         honest, and it does not matter: it is not what blocks us.")

        if 'torque_ff_check' in modes:
            print(f"\n  [MODE 2: torque_ff_check]  hold {self.tff_hold_rev:.2f} rev with "
                  f"{self.tff_hold_mass_kg:.2f} kg")
            print(f"    ladder: {', '.join(f'{t:+.3f}' for t in self.tff_ladder_Nm)} Nm")
            print(f"    {self.tff_settle_s:.1f} s settle + {self.tff_measure_s:.1f} s "
                  f"measure per rung -> "
                  f"~{int(self.tff_measure_s * BENCH_BUILD_TELEM_HZ)} iq samples each")
            print(f"      {'tff(Nm)':>9} {'wire(Nm)':>9} {'expected iq shift':>18} "
                  f"{'as a fraction of tau_g':>24}")
            tau_g = kt.gravity_torque_Nm(self.tff_hold_mass_kg, tilt_deg=self.tilt_deg)
            for t in self.tff_ladder_Nm:
                shift = t / kt.KT_ODRIVE_CONFIGURED
                frac = (t / tau_g) if tau_g > 0 else float('nan')
                print(f"      {t:+9.3f} {kt.wire_quantized_torque_Nm(t):+9.4f} "
                      f"{shift:+13.2f} A     {frac:+18.1%}")
            print(f"    expected |slope| = 1/{kt.KT_ODRIVE_CONFIGURED:.6f} = "
                  f"{1.0 / kt.KT_ODRIVE_CONFIGURED:.2f} A/Nm")
            print(f"    the biggest rung ({max(abs(t) for t in self.tff_ladder_Nm):.3f} Nm) "
                  f"is {max(abs(t) for t in self.tff_ladder_Nm) / kt.ODRIVE_TORQUE_SOFT_MAX_NM:.1%} "
                  f"of the ODrive's torque_soft_max ({kt.ODRIVE_TORQUE_SOFT_MAX_NM} Nm)")
            print(f"    every rung INSIDE the static-friction band (worst-case edge "
                  f"{kt.TFF_STATIC_BAND_MIN_NM:.3f} Nm) — that")
            print(f"    is what keeps the leg truly stationary; out-of-band rungs get "
                  f"re-absorbed by the")
            print(f"    integrator and bias the slope low (model 33-52%; observed 63-68% on the 2026-07-14 "
                  f"±0.10 Nm rungs).")
            for w in kt.torque_ff_ladder_band_warnings(self.tff_ladder_Nm):
                print(f"    WARN: {w}")
            print(f"    verdict gate: R2 >= {kt.TFF_GATE_MIN_R2:.2f} AND |slope|/sigma "
                  f">= {kt.TFF_GATE_MIN_T_STAT:.0f}, else NO CONCLUSION is printed")
            print(f"    (never a sign/scale verdict from a garbage fit).")
            print(f"    the position loop holds station, so it is inherently safe: the loop")
            print(f"    resists any runaway, and the iq SHIFT is the measurement.")
            print(f"    hold is {self.tff_hold_rev:.2f} rev — clear of the firmware stroke")
            print(f"    clamp (0.071 / 3.900), which would ZERO torque_ff "
                  f"(leg_interp.cpp:407).")

        print(f"\n  SIGN CONVENTION (the trap):")
        print(f"    can_buses.cpp:85-86 negates pos/vel (-> extension-positive), but :92")
        print(f"    stores iq_measured RAW in the ODrive frame. So an EXTENDING torque is")
        print(f"    predicted to read as {kt.IQ_EXTENSION_SIGN_PREDICTED:+d} iq. The harness")
        print(f"    does not assume this — gravity calibrates it (the leg must push UP to")
        print(f"    hold a mass, so the sign of the loaded current IS the extension sign).")
        print(f"\n  BENCH != PLATFORM (sign): ODrive direction calibration is PER-DRIVE;")
        print(f"    this bench drive's torque sign convention is OPPOSITE to the platform")
        print(f"    legs' (2026-04-27 bench friction-FF needed --ff-sign -1; the 2026-05-08")
        print(f"    PLATFORM validation ran un-negated and worked). A bench sign result")
        print(f"    does NOT transfer to the platform — the platform-validated sign stands.")
        print("\n  DRY-RUN complete — validation passed, nothing commanded.")
        print("=" * 72)

    # -- top-level ------------------------------------------------------------

    def run(self, modes: List[str]) -> int:
        err = self.validate_args()
        if err:
            print(f"REJECT: {err}", file=sys.stderr)
            return 1
        if self.dry_run:
            self.print_plan(modes)
            return 0

        def sigint(signum, frame):
            print("\n[SIGINT] abort -> disarm")
            self._sigint = True
            try:
                self._disarm()
            except Exception:
                pass
        signal.signal(signal.SIGINT, sigint)

        rc = 0
        try:
            self._setup_transport()
            time.sleep(0.5)
            if not self._check_bench_firmware():
                return 1
            if not self._startup_clear_latch():
                return 2
            if self.do_home:
                if not self._home():
                    self._abort_reason = "homing did not run — refusing to continue"
                    return 2
            stage_fns = {'kt': self.stage_kt,
                         'torque_ff_check': self.stage_torque_ff_check}
            for mode in modes:
                if not stage_fns[mode]():
                    rc = 2
                    break
        except KeyboardInterrupt:
            rc = 2
            if not self._abort_reason:
                self._abort_reason = "KeyboardInterrupt"
        except Exception as exc:  # noqa: BLE001
            print(f"\nFATAL: {exc}", file=sys.stderr)
            import traceback
            traceback.print_exc()
            rc = 1
        finally:
            self._torque_ff_Nm = 0.0
            # Cover the exit paths the stages could not (SIGINT mid-stream, a FATAL
            # exception, a stage that bailed before its own park): if the leg is still
            # armed away from the bottom, park it — _park_low preserves the original
            # abort diagnostic and says so explicitly if parking is impossible.
            if self._needs_park:
                try:
                    self._park_low()
                except Exception as exc:  # noqa: BLE001
                    print(f"  (park attempt failed: {exc})", file=sys.stderr)
            try:
                self._teardown_transport()
            except Exception:
                pass
            try:
                self.write_manifest()
            except Exception as exc:  # noqa: BLE001
                print(f"  (manifest write failed: {exc})", file=sys.stderr)

        if rc == 0:
            print("\nPASS: all requested modes completed.")
        else:
            print(f"\nABORT: {self._abort_reason or 'stage fault'}", file=sys.stderr)
        return rc


def _mean_stats(stats: List[kt.TraverseStats], direction: str) -> kt.TraverseStats:
    """Average repeated traverses in one direction into a single TraverseStats.

    The rep means are averaged and their SEMs combined in quadrature (÷ n). Reps are
    averaged BEFORE the up/down combination so the directional pairing stays intact —
    that pairing is what makes friction cancel, and averaging across directions first
    would destroy it.
    """
    means = np.array([s.iq_mean_A for s in stats], float)
    sems = np.array([s.iq_sem_A for s in stats], float)
    n = means.size
    mean = float(np.mean(means))
    sem = float(np.sqrt(np.sum(sems ** 2)) / n)
    # If the reps disagree by more than their SEMs suggest, believe the SPREAD — a genuine
    # rep-to-rep inconsistency (a shifting mass, a sticky spot) must widen the error bar,
    # not hide inside an optimistic one.
    if n > 1:
        spread_sem = float(np.std(means, ddof=1) / np.sqrt(n))
        sem = max(sem, spread_sem)
    return kt.TraverseStats(
        direction=direction, n_samples=int(sum(s.n_samples for s in stats)),
        iq_mean_A=mean, iq_std_A=float(np.mean([s.iq_std_A for s in stats])),
        iq_sem_A=sem,
        vel_mean_rps=float(np.mean([s.vel_mean_rps for s in stats])),
        vel_error_frac=float(np.max([s.vel_error_frac for s in stats])),
        ok=all(s.ok for s in stats),
        reasons=[r for s in stats for r in s.reasons])


def _parse_floats(s: str) -> List[float]:
    return [float(x) for x in s.split(',') if x.strip()]


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Measure the leg torque constant (Kt) with the stiction confound "
                    "removed, and validate the torque_ff channel end-to-end.",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--mode', choices=['kt', 'torque_ff_check', 'all'], default='kt')
    ap.add_argument('--dry-run', action='store_true',
                    help="print the plan (budget, distinguishing power) — no I/O")
    ap.add_argument('--yes', action='store_true',
                    help="skip the per-mass operator confirmation prompts")
    ap.add_argument('--home', action='store_true',
                    help="run the firmware HOME(axis) RPC first")

    ap.add_argument('--masses', type=str, default=None,
                    help="comma-separated masses in kg (default: the recommended ladder)")
    ap.add_argument('--traverse-lo', type=float, default=DEFAULT_TRAVERSE_LO_REV)
    ap.add_argument('--traverse-hi', type=float, default=DEFAULT_TRAVERSE_HI_REV)
    ap.add_argument('--vel', type=float, default=kt.DEFAULT_TRAVERSE_VEL_RPS,
                    help="constant traverse velocity, rev/s (must clear the "
                         f"{kt.OMEGA_S_REV_S} rev/s stiction knee)")
    ap.add_argument('--reps', type=int, default=DEFAULT_REPS)
    ap.add_argument('--dwell', type=float, default=DEFAULT_DWELL_S)
    ap.add_argument('--edge-discard', type=float, default=DEFAULT_EDGE_DISCARD_S,
                    help="seconds to discard from each end of a traverse (accel/decel)")
    ap.add_argument('--accel-time', type=float,
                    default=kt.DEFAULT_TRAVERSE_ACCEL_TIME_S,
                    help="traverse start/end velocity-ramp duration, s (trapezoid "
                         "shaping; unshaped starts demand ~60 rev/s² ≈ +4.5 A at 3 kg)")
    ap.add_argument('--tilt-deg', type=float, default=0.0,
                    help="leg's departure from vertical; gravity torque scales by cos(tilt). "
                         "KEEP THE LEG VERTICAL — this is a correction, not a licence.")

    ap.add_argument('--tff-ladder', type=str, default=None,
                    help="comma-separated torque_ff values in Nm "
                         f"(default {DEFAULT_TORQUE_FF_LADDER})")
    ap.add_argument('--tff-hold', type=float, default=DEFAULT_TFF_HOLD_REV)
    ap.add_argument('--hold-mass', type=float, default=DEFAULT_TFF_HOLD_MASS_KG,
                    help="mass on the leg during the torque_ff check (kg). It calibrates "
                         "the extension sign AND loads the position loop.")
    ap.add_argument('--tff-settle', type=float, default=DEFAULT_TFF_SETTLE_S)
    ap.add_argument('--tff-measure', type=float, default=DEFAULT_TFF_MEASURE_S)

    ap.add_argument('--axis', type=int, default=0)
    ap.add_argument('--stroke-cap', type=float, default=3.0)
    ap.add_argument('--current-limit', type=float, default=HARD_CURRENT_LIMIT_A)
    ap.add_argument('--vel-cap', type=float, default=4.0)
    ap.add_argument('--knot-hz', type=float, default=BENCH_BUILD_KNOT_HZ,
                    help="MUST be 100 to match BENCH_SYSID_BUILD's SEGMENT_T_S = 0.010")
    ap.add_argument('--output-dir', type=str, default=None)
    args = ap.parse_args()

    if args.stroke_cap > HARD_STROKE_CEIL_REV:
        print(f"REJECT: stroke cap {args.stroke_cap} above the hard ceiling "
              f"{HARD_STROKE_CEIL_REV}", file=sys.stderr)
        return 1

    masses = _parse_floats(args.masses) if args.masses else kt.recommended_masses_kg()
    ladder = (_parse_floats(args.tff_ladder) if args.tff_ladder
              else list(kt.DEFAULT_TORQUE_FF_LADDER_NM))
    modes = ['kt', 'torque_ff_check'] if args.mode == 'all' else [args.mode]

    out = args.output_dir or os.path.join(
        _PROJECT_ROOT, 'temp', 'probes',
        f"kt_bench_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}")

    bench = KtBench(
        masses_kg=masses, traverse_lo_rev=args.traverse_lo,
        traverse_hi_rev=args.traverse_hi, traverse_vel_rps=args.vel, reps=args.reps,
        dwell_s=args.dwell, edge_discard_s=args.edge_discard, tilt_deg=args.tilt_deg,
        tff_ladder_Nm=ladder, tff_hold_rev=args.tff_hold,
        tff_hold_mass_kg=args.hold_mass, tff_settle_s=args.tff_settle,
        tff_measure_s=args.tff_measure, assume_yes=args.yes,
        accel_time_s=args.accel_time,
        # --- BridgeSysID kwargs ---
        axis_id=args.axis, stroke_cap_rev=args.stroke_cap,
        center_rev=args.traverse_lo, current_limit_a=args.current_limit,
        vel_cap_rps=args.vel_cap, output_dir=out, dry_run=args.dry_run,
        chirp_f0=1.0, chirp_f1=8.0, chirp_dur=4.0, chirp_amp=0.02,
        pos_steps=[], ladder_step=0.14, zeta_target=0.7, bw_clear_hz=None, n_vel=4,
        ripple_threshold=0.05, osc_threshold=0.5, iq_ripple_threshold=0.5,
        do_home=args.home, knot_hz=args.knot_hz, fast_iq_available=True,
    )
    return bench.run(modes)


if __name__ == '__main__':
    sys.exit(main())
