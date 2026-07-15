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

``--mode torque_ff_check`` — validate the torque_ff channel by EDGE CAPTURE
----------------------------------------------------------------------------
**Redesigned 2026-07-15.** The original rung ladder held the leg and measured a
"steady" window ~1.3 s after each ``torque_ff`` step. That design is structurally
confounded and is gone: at a genuinely SETTLED static hold the velocity-loop
integrator forces pos == cmd, where torque balance pins the TOTAL iq at the load
value — **the steady-state iq response to a constant torque_ff is exactly ZERO**.
The ladder measured the locked-state re-absorption TRANSIENT (τ ≈ 2–10 s) at a fixed
cadence, so wall-clock and tff were collinear and any slow drift aliased straight
into the "slope" (the 2026-07-14 20:55 −21 A/Nm was exactly that artifact).

EDGE CAPTURE measures the INSTANTANEOUS iq jump at each edge of a ± square wave,
before the loop can re-absorb it:

1. **Settle gate** — wait until pos is within ±0.3 mrev of cmd AND |d(iq)/dt| <
   0.05 A/s over a 3 s window (max wait ~30 s, progress printed). Preceded by a
   ``--pre-soak`` hold (default 15 s) that kills the friction-band load-transfer
   transient (~1.2–1.4 A, τ ≈ 3.2 s).
2. **Square wave** — ``torque_ff`` alternates ±X, half-period 1.75 s (1.5–2 s ok),
   X ∈ {0.005, 0.010, 0.015} Nm — all inside the 0.045 Nm static-friction band so the
   lock keeps the loop blind (±0.36 A excursion at X = 0.02 vs >3.5 A headroom);
   ≥10 cycles per amplitude, 250 Hz telemetry recorded continuously.
3. **Matched filter** — each edge located by cross-correlating a step kernel with the
   iq stream near the commanded toggle (±0.1 s); never wall-clock alone.
4. **Jump** — mean(iq[t_e+0.02 .. t_e+0.20]) − mean(iq[t_e−0.20 .. t_e−0.02]) per
   edge, decay-corrected with the fitted per-hold τ (default 8 s; uncorrected creep
   costs 2.5–9.5 % at 0.2 s). slope = jump/(2X), outlier-trimmed over ≥19 edges per
   amplitude; per-amplitude + pooled. The alternating polarity is what de-aliases
   drift — the ladder had no such defence.
5. **Verdict gate** — |slope|/σ ≥ 3 AND trimmed edge-jump CV ≤ 20 % (the edge
   analogue of the old R² gate), else NO CONCLUSION (and ``channel_live`` is recorded
   as **None**, tri-state — False is reserved for a DEMONSTRATED precise null).
   Expected |slope| = 1/torque_constant = 18.14 A/Nm, sign referred through the
   REQUIRED ``--rig-orientation`` declaration.

Nothing moves during Mode 2: a static hold plus inaudible torque toggles. It pins
down SIGN (settled 2026-07-14: positive wire tff = extension through the production
chain — a RETRACTS verdict now flags a probable ``--rig-orientation`` mix-up), SCALE
(iq per commanded Nm, wire int16 ×10000 round-trip), and with Mode 1's true Kt the
exact mechanical-torque correction factor.

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
    python tests/hardware/kt_bench_test.py --mode kt --rig-orientation inverted --dry-run
    python tests/hardware/kt_bench_test.py --mode torque_ff_check --rig-orientation inverted --dry-run

    # Live (requires the BENCH_SYSID_BUILD firmware flashed):
    python tests/hardware/kt_bench_test.py --mode kt --rig-orientation inverted \\
        --home --masses 1.0,2.0,3.0
    python tests/hardware/kt_bench_test.py --mode torque_ff_check \\
        --rig-orientation inverted --hold-mass 2.0

RUN INSTRUCTIONS (operator checklist)
=====================================
* **STATE OF KNOWLEDGE (2026-07-14 session, four masses 0.5/1.0/1.5/2.25 kg):**

  - **Kt preliminary: 0.0577 ± 0.0012 Nm/A** (offline refit of the four recovered
    single-mass runs; mass-fit R² = 0.99909). The masses were OPERATOR-RECALLED, not
    weighed — and the harness had labelled every run "1.00 kg" because ``--hold-mass``
    was silently ignored in mode kt (now a parse-time refusal). **A confirmation run
    needs WEIGHED masses + 2 reps** before anything acts on 0.0577.
  - **The tff channel SIGN is settled: positive wire torque_ff = EXTENSION**, through
    the production chain, confirmed end-to-end — no negation anywhere. A RETRACTS
    verdict from a future run should first suspect the ``--rig-orientation``
    declaration, not the wire.
  - **The 20:55 −21 A/Nm "slope" was drift-aliasing**, not a channel measurement: at
    the ladder's fixed rung cadence, wall-clock and tff were collinear, so hold drift
    mapped straight onto the fit. That is WHY the redesigned mode toggles a ± square
    wave instead — alternating polarity de-aliases drift by construction (and a
    paired-edge statistic now detects any residual drift explicitly).

* **``--rig-orientation {normal,inverted}`` is REQUIRED** (no default): ``normal`` =
  extension raises the load; ``inverted`` = contraction raises it. Every sign
  inference runs through it — the 2026-07-14 manifests recorded
  ``extension_iq_sign = −1`` wrong-way-round because the rig was inverted while the
  harness assumed holding == extension.
* **Masses: 1.0 / 2.0 / 3.0 kg, WEIGHED.** With only 2-3 bench weights available, three
  masses spread maximally is the best fit (span is where the slope's leverage lives —
  1/2/3 kg beats any closer triple). Two masses would be exactly-determined
  (slope+intercept through 2 points, zero residual dof — no way to tell signal from
  disturbance), so the harness refuses fewer than 3. At the per-mass prompt you can
  type the WEIGHED value to correct the declared mass for that point — it is recorded
  per traverse in the manifest.
* **The mass must hang FREE and VERTICAL, and NOBODY touches it (or the leg) during
  the toggles/traverses.** One supported measurement poisons the whole fit: on the
  2026-07-14 run a hand-supported rung read iq ≈ 0.05 A — an essentially UNLOADED
  leg — and helped wreck the fit (R² = −0.099). Hang it, step back, then press ENTER.
* **Declare the true mass** (``--masses`` / ``--hold-mass``): the current budget, the
  over-current abort margins and the sign inference all use it. Mode-mismatched flags
  are REFUSED at parse time (``--hold-mass`` in mode kt cost the 2026-07-14 session
  its mass labels).
* **Static holds get a ``--pre-soak`` (default 15 s)** before any measurement: the
  friction-band load-transfer transient (~1.2–1.4 A ≈ τ_c amplitude, τ ≈ 3.2 s) is
  dead in 15 s, and Mode 2 additionally gates on a settled hold (pos ±0.3 mrev,
  |d(iq)/dt| < 0.05 A/s) before toggling.
* **Bench-vs-platform SIGN context** (read before acting on any Mode-2 sign verdict):
  ODrive direction calibration is PER-DRIVE, and this bench drive's torque sign
  convention is OPPOSITE to the platform legs'. The 2026-04-27 friction bench work
  empirically needed ``--ff-sign -1`` on this rig, while the 2026-05-08 PLATFORM
  validation ran the standard un-negated chain and worked (a wrong sign would have
  doubled friction — that A/B genuinely discriminates). A bench sign result does NOT
  transfer to the platform on its own — but note the 2026-07-14 session settled the
  PRODUCTION-chain tff sign (positive = extension) end-to-end.

Output: ``temp/probes/kt_bench_<ts>/`` — one CSV per traverse / square-wave amplitude
+ ``manifest.json`` (persisted INCREMENTALLY: every completed traverse/amplitude lands
on disk immediately, so an abort loses nothing — the 2026-07-14 manifests were empty
because serialization only happened at stage end and no stage ever completed).

Exit codes: 0 normal · 1 pre-flight rejection · 2 mid-run fault/abort
"""
from __future__ import annotations

import argparse
import contextlib
import datetime
import json
import math
import os
import signal
import sys
import threading
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
    BRIDGE_ARM_VERIFY_GRACE_S,
    BRIDGE_TEENSY_IP,
    HARD_CURRENT_LIMIT_A,
    HARD_STROKE_CEIL_REV,
    BridgeSysID,
)

DEFAULT_TFF_AMPS = ','.join(f"{t:.3f}" for t in kt.DEFAULT_TFF_EDGE_AMPS_NM)

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
DEFAULT_TFF_HOLD_MASS_KG = 2.0
# Edge-capture settle-gate loop: stream the hold in chunks this long between settle
# evaluations (each chunk keeps the armed stream alive; evaluation is milliseconds).
SETTLE_CHUNK_S = 1.0
PRE_SOAK_CHUNK_S = 5.0                # progress-print granularity during the pre-soak

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

    What this subclass adds:

    * ``_send_knot`` is overridden to carry a ``torque_ff`` — the base class hardcodes it to
      zero, and Mode 2 is entirely about that field.
    * ``_log_frame`` is overridden to add a live over-current abort. This is the cheapest
      correct hook: it runs on the RX thread for every telemetry frame, and setting
      ``_sigint`` routes into the base class's existing clean disarm path rather than
      inventing a second one. The latch itself records the measured peak, so an abort
      report can never lose its actual trigger behind a NaN telemetry snapshot (A5).
    * ``_keep_stream_alive`` — a background flat-knot pump for the gaps BETWEEN armed
      foreground streams (analysis, CSV plumbing, incremental manifest writes): an
      armed stream must never go silent past the firmware's 250 ms staleness E-STOP.
    * ``_park_low`` is reworked around an armed fast path plus an explicit
      disarm→clear→bring-up→stream-then-arm slow path with a bounded retry — the
      pre-fix version ran ``_bringup_closed_loop()`` (RPCs + ``time.sleep(0.3)``)
      while armed and silent, guaranteeing the MPC_STALE latch it then tripped over
      ("CANNOT PARK", 2026-07-14).
    * Mode 2's measurement core is EDGE CAPTURE (square-wave ``torque_ff`` toggles,
      matched-filter edges, decay-corrected jumps) — see the module docstring.
    """

    def __init__(self, *, masses_kg: List[float], traverse_lo_rev: float,
                 traverse_hi_rev: float, traverse_vel_rps: float, reps: int,
                 dwell_s: float, edge_discard_s: float, tilt_deg: float,
                 rig_orientation: str,
                 tff_amps_Nm: List[float], tff_hold_rev: float,
                 tff_hold_mass_kg: float, tff_half_period_s: float,
                 tff_cycles: int, pre_soak_s: float,
                 assume_yes: bool,
                 accel_time_s: float = kt.DEFAULT_TRAVERSE_ACCEL_TIME_S, **kw):
        # Mode-1 approaches the traverse LOW end; Mode-2 holds mid-stroke. The base class
        # ramps to `center_rev` on bringup, so point it at whichever this run needs.
        self._armed = False             # tracked via the _arm/_disarm overrides below
        super().__init__(**kw)
        self.masses_kg = list(masses_kg)
        self.traverse_lo_rev = float(traverse_lo_rev)
        self.traverse_hi_rev = float(traverse_hi_rev)
        self.traverse_vel_rps = float(traverse_vel_rps)
        self.reps = int(reps)
        self.dwell_s = float(dwell_s)
        self.edge_discard_s = float(edge_discard_s)
        self.tilt_deg = float(tilt_deg)
        self.rig_orientation = str(rig_orientation)
        self.tff_amps_Nm = [float(a) for a in tff_amps_Nm]
        self.tff_hold_rev = float(tff_hold_rev)
        self.tff_hold_mass_kg = float(tff_hold_mass_kg)
        self.tff_half_period_s = float(tff_half_period_s)
        self.tff_cycles = int(tff_cycles)
        self.pre_soak_s = float(pre_soak_s)
        self.assume_yes = bool(assume_yes)
        self.accel_time_s = float(accel_time_s)

        self._torque_ff_Nm = 0.0        # what _send_knot puts on the wire, this instant
        self._iq_abort_reason: Optional[str] = None
        self._iq_abort_limit = IQ_ABORT_FRAC * self.current_limit_a
        self._iq_trip = kt.OverCurrentLatch(self._iq_abort_limit)
        self._needs_park = False        # True while the leg is armed away from the bottom
        self._last_streamed_u0: Optional[float] = None   # for the keep-alive hold stream
        self._current_mass_kg: Optional[float] = None    # for abort budget context (A5)

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
            # REQUIRED operator declaration — every sign inference runs through it
            # (the 2026-07-14 manifests recorded extension_iq_sign wrong-way-round
            # because the rig was inverted and the code assumed holding == extension).
            'rig_orientation': self.rig_orientation,
            'friction_reference': {
                'tau_c_A': kt.TAU_C_REF_A,
                'range_A': list(kt.TAU_C_REF_RANGE_A),
                'omega_s_rev_s': kt.OMEGA_S_REV_S,
                'source': 'logbook/2026-04-27-friction-feedforward-bench-validation.md:79,81',
            },
            'torque_ff': {
                'design': 'edge_capture',   # the rung ladder is dead (2026-07-15)
                'amplitudes_Nm': self.tff_amps_Nm,
                'half_period_s': self.tff_half_period_s,
                'cycles_per_amplitude': self.tff_cycles,
                'pre_soak_s': self.pre_soak_s,
                'settle_gate': {
                    'pos_tol_rev': kt.SETTLE_POS_TOL_REV,
                    'diq_dt_max_A_per_s': kt.SETTLE_DIQ_DT_MAX_A_PER_S,
                    'window_s': kt.SETTLE_WINDOW_S,
                    'max_wait_s': kt.SETTLE_MAX_WAIT_S,
                },
                'hold_rev': self.tff_hold_rev,
                'hold_mass_kg': self.tff_hold_mass_kg,
                'wire_scale_counts_per_Nm': kt.LEG_TOR_WIRE_SCALE,
            },
            'iq_sign_code_read_prediction': kt.IQ_EXTENSION_SIGN_PREDICTED,
        }

    # -- the overrides ---------------------------------------------------------

    def _send_knot(self, u0, u1, u2, v0, t_origin_us):
        """Identical to the base class, except it carries ``self._torque_ff_Nm`` and
        remembers the last streamed ``u0`` (the keep-alive hold stream resumes there).

        The Setpoint's ``torque_ff`` is float32 **Nm** on the wire
        (``config/generated/udp_protocol.py:186``); the Teensy negates it via ``leg_sign``
        and scales it by 10000 into an int16 (``odrive_protocol.h:171-179``). We stream the
        value we WANT in the Jugglebot extension-positive frame and let the firmware do
        exactly what it does in production — the point of Mode 2 is to find out what that
        is, so the harness must not pre-compensate anything.
        """
        self._last_streamed_u0 = float(u0)
        sp = self._Setpoint(
            u0=self._vec(u0), u1=self._vec(u1), u2=self._vec(u2), v0=self._vec(v0),
            accel=(0.0,) * self._nlegs,
            torque_ff=self._vec(self._torque_ff_Nm),
            flags=0x3, t_origin_us=int(t_origin_us))
        self._client.send_stream(int(self._MsgType.SETPOINT), sp.pack())

    def _arm(self):
        super()._arm()
        self._armed = True

    def _disarm(self):
        super()._disarm()
        self._armed = False

    @contextlib.contextmanager
    def _keep_stream_alive(self):
        """Stream flat hold knots from a background thread while Python does analysis /
        CSV plumbing / manifest writes between foreground streams.

        THE GAP THIS CLOSES (2026-07-14): with ``mpc_active=1`` the firmware E-STOPs
        after 250 ms without a Setpoint (``canbridge_config.h:154``). Any armed
        between-streams pause — summarizing a traverse, opening the next CSV, the
        incremental manifest write, and above all ``_park_low``'s old still-armed
        ``_bringup_closed_loop()`` with its RPC round-trips + ``time.sleep(0.3)`` —
        could starve the interpolator and latch MPC_STALE, which is exactly how that
        night's kt runs died between rep 1 and rep 2 and then "CANNOT PARK"-ed. An
        armed stream must NEVER go silent; a disarm must be explicit (mpc_active=0
        suppresses the staleness check).

        No-op while disarmed. The pump holds the last streamed knot (zero motion) and
        carries the current ``self._torque_ff_Nm`` like every other knot.
        """
        if not self._armed or self._last_streamed_u0 is None:
            yield
            return
        stop = threading.Event()
        u0 = float(self._last_streamed_u0)
        period = self.seg_t_s

        def _pump():
            i = 0
            t0 = time.perf_counter()
            while not stop.is_set():
                try:
                    self._send_knot(u0, u0, u0, 0.0, int(time.time() * 1_000_000))
                except OSError:
                    break
                i += 1
                dt = t0 + i * period - time.perf_counter()
                if dt > 0:
                    time.sleep(dt)

        th = threading.Thread(target=_pump, daemon=True,
                              name='kt-keepalive-hold')
        th.start()
        try:
            yield
        finally:
            stop.set()
            th.join(timeout=1.0)

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
                f"OVER-CURRENT ({self._iq_trip.describe_trip()}; "
                f"{IQ_ABORT_FRAC:.0%} of the {self.current_limit_a:.1f} A limit) — "
                f"disarming. Is the mass heavier than declared, or is the leg binding?")
            self._sigint = True

    def _mass_budget_context(self) -> str:
        """The per-mass current budget vs the measured peak, for an over-current abort
        report — so the operator can see at a glance whether the mass is simply too
        heavy for the rig rather than guessing (A5)."""
        m = self._current_mass_kg
        if m is None:
            return ""
        try:
            iq_up, iq_dn = kt.predicted_iq_up_down(
                m, min(kt.KT_CANDIDATES.values()), vel_rps=self.traverse_vel_rps,
                tilt_deg=self.tilt_deg)
            iq_acc = kt.accel_current_A(
                m, self.traverse_vel_rps / self.accel_time_s,
                kt_nm_per_a=min(kt.KT_CANDIDATES.values()))
            return (f" [budget @ {m:.2f} kg (worst-case Kt): steady "
                    f"{max(abs(iq_up), abs(iq_dn)):.2f} A, +accel {iq_acc:.2f} A, "
                    f"vs measured peak {self._iq_trip.max_abs_A:.2f} A and the "
                    f"{self._iq_abort_limit:.2f} A abort threshold — breakaway/static "
                    f"holds can draw well past the steady prediction]")
        except Exception:  # noqa: BLE001 — a budget print must never mask the abort
            return ""

    def _took_iq_abort(self) -> bool:
        """Consume a live over-current trip: restore the real reason (the base class's
        stream loop overwrites ``_abort_reason`` with 'SIGINT') and report it, with the
        per-mass budget context appended."""
        if self._iq_abort_reason:
            self._abort_reason = self._iq_abort_reason + self._mass_budget_context()
            return True
        return False

    def _abort_diag(self, res: dict) -> str:
        """Base diagnostic, but the ACTUAL trigger can never be lost behind the
        telemetry-snapshot fallback (A5).

        The 2026-07-14 2.75 kg approach aborts printed ``NONE @ u0=+nan enc=+nan``:
        the over-current latch had fired on the RX thread, the stream loop stamped
        'SIGINT', and the approach path rebuilt the reason from an EMPTY telemetry
        log — all NaNs, real cause gone. The trigger now rides along explicitly."""
        if self._iq_abort_reason:
            return self._iq_abort_reason + self._mass_budget_context()
        base = super()._abort_diag(res)
        if self._iq_trip.max_abs_A > 0.0:
            base += f" [{self._iq_trip.describe_trip()}]"
        return base

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
        if self.rig_orientation not in kt.RIG_ORIENTATIONS:
            return (f"--rig-orientation must be one of {kt.RIG_ORIENTATIONS} "
                    f"(got '{self.rig_orientation}')")
        if not self.tff_amps_Nm or any(a <= 0.0 for a in self.tff_amps_Nm):
            return "--tff-amps must be positive amplitudes (the wave toggles ±X itself)"
        if len(set(f"{a:.4f}" for a in self.tff_amps_Nm)) != len(self.tff_amps_Nm):
            return ("--tff-amps contains duplicate amplitudes — they would overwrite "
                    "each other's CSVs and add no information; repeat CYCLES instead "
                    "(--tff-cycles)")
        # The safety validator sees both signs of every amplitude (the wave visits both).
        signed = [s * a for a in self.tff_amps_Nm for s in (+1.0, -1.0)]
        problems = kt.torque_ff_ladder_safe(signed)
        if problems:
            return "unsafe torque_ff amplitudes: " + "; ".join(problems)
        # Out-of-band amplitudes are SAFE but leave the static-friction band — the leg
        # moves, the integrator wakes up, and the edge premise dies. Warn, don't refuse
        # (a deliberate band-mapping run is a legitimate experiment).
        for w in kt.torque_ff_ladder_band_warnings(signed):
            print(f"  WARN: {w}")
        # SWING pre-flight (2026-07-15): the square wave toggles through 2X — the
        # level check above is NOT sufficient (the ±0.035 default of the first
        # session passed it and then broke the leg loose at every toggle). Any
        # amplitude warned here WILL be excluded from the pooled verdict.
        for w in kt.torque_ff_swing_warnings(self.tff_amps_Nm):
            print(f"  WARN: {w}")
        if self.tff_cycles < kt.MIN_TFF_CYCLES:
            return (f"--tff-cycles {self.tff_cycles} is below the minimum "
                    f"{kt.MIN_TFF_CYCLES} — too few edges to even attempt statistics")
        if self.tff_cycles < kt.DEFAULT_TFF_CYCLES:
            print(f"  WARN: --tff-cycles {self.tff_cycles} < the recommended "
                  f"{kt.DEFAULT_TFF_CYCLES} — fewer edges, weaker trim/CV statistics")
        if not (0.5 <= self.tff_half_period_s <= 5.0):
            return (f"--tff-half-period {self.tff_half_period_s} s is outside the "
                    f"sane 0.5–5 s range (jump windows need "
                    f"{kt.EDGE_WINDOW_HI_S:.2f} s clear of BOTH neighbouring edges; "
                    f"very long holds re-absorb and waste session time)")
        lo, hi = kt.TFF_HALF_PERIOD_RANGE_S
        if not (lo <= self.tff_half_period_s <= hi):
            print(f"  WARN: --tff-half-period {self.tff_half_period_s} s is outside "
                  f"the recommended {lo}-{hi} s window")
        if self.pre_soak_s < 0.0:
            return "--pre-soak must be >= 0"
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
            return ("mass set rejected by the current budget "
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

    def _persist_manifest(self):
        """Write the manifest to disk NOW (A6) — quiet, and never let a serialization
        hiccup kill a live run. Called after every completed traverse / amplitude, not
        just at stage end: the 2026-07-14 manifests recorded ``stages: {}`` because the
        only write happened after a stage completed and no stage ever did."""
        try:
            os.makedirs(self.output_dir, exist_ok=True)
            if self.telem_rate is not None:
                self._manifest['telemetry_rate'] = asdict(self.telem_rate)
            self._manifest['updated'] = datetime.datetime.now().isoformat(
                timespec='seconds')
            with open(os.path.join(self.output_dir, 'manifest.json'), 'w') as f:
                json.dump(self._manifest, f, indent=2)
        except Exception as exc:  # noqa: BLE001
            print(f"  (incremental manifest write failed: {exc})", file=sys.stderr)

    # -- operator interaction -------------------------------------------------

    def _prompt_mass(self, mass_kg: float):
        """Confirm the operator has hung THIS mass before we drive the leg.

        Returns ``(action, mass_used_kg)`` where ``action`` is ``'go'`` (run this
        mass), ``'skip'`` (skip this mass, continue with the next), or ``'quit'``
        (abort the whole run cleanly, parking first). ``'q'``, EOF and Ctrl-C all mean
        QUIT — they used to be silently treated as skip-this-mass, which left an
        operator who wanted OUT cycling through every remaining prompt.

        Typing a NUMBER at the prompt overrides the declared mass for this point (a
        WEIGHED value beats a recalled one — the 2026-07-14 Kt result rests on
        operator-recalled masses and is preliminary for exactly that reason). The
        override is budget-checked before it is accepted, and the mass actually used
        is recorded per traverse in the manifest.
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
            return 'go', float(mass_kg)
        while True:
            try:
                ans = input(
                    f"  Hang {mass_kg:.2f} kg and press ENTER (or type the WEIGHED "
                    f"mass in kg to correct it, 's' to skip, 'q' to quit): "
                ).strip().lower()
            except (EOFError, KeyboardInterrupt):
                print()
                return 'quit', float(mass_kg)
            if ans.startswith('q'):
                return 'quit', float(mass_kg)
            if ans.startswith('s'):
                return 'skip', float(mass_kg)
            if not ans:
                return 'go', float(mass_kg)
            try:
                m = float(ans)
            except ValueError:
                print("  (unrecognized — ENTER to accept, a number to correct, "
                      "'s' skip, 'q' quit)")
                continue
            row = kt.current_budget(
                [m], current_limit_A=self.current_limit_a,
                vel_rps=self.traverse_vel_rps, tilt_deg=self.tilt_deg,
                accel_rps2=self.traverse_vel_rps / self.accel_time_s).rows[0]
            if not row.ok:
                print(f"  REFUSED {m:.2f} kg: " + "; ".join(row.warnings))
                continue
            print(f"  using the weighed mass {m:.2f} kg for this point "
                  f"(declared {mass_kg:.2f} kg is recorded alongside)")
            return 'go', float(m)

    # -- MODE 1: Kt ------------------------------------------------------------

    def _park_rearm(self, live_rev: float) -> Optional[str]:
        """The FULL re-arm dance for a park descent, with a bounded retry (A4).

        Sequence per attempt — mirroring the proven startup dance rather than
        reinventing it:

        1. **explicit disarm** — ``mpc_active=0`` suppresses the firmware staleness
           check, so everything below may take its time without latching MPC_STALE
           (the pre-fix park ran ``_bringup_closed_loop()`` — RPC round-trips plus a
           hard ``time.sleep(0.3)`` — while still ARMED and silent, which GUARANTEED a
           staleness latch; ``_warm_and_arm``'s settle then saw the latch and the park
           died with "could not re-arm for the descent (arm-settle latched)");
        2. **verify-and-clear any latch, disarmed** (``_startup_clear_latch``: streams
           disarmed warm-up knots to re-baseline the interp base u0 to the live
           encoder, THEN a disarmed CLEAR_ERRORS, then waits for fault NONE) — within
           the guard recovery budget;
        3. **disarmed bring-up** (the 0.3 s settle inside is now harmless);
        4. **stream-then-arm-then-settle** (``_warm_and_arm``: disarmed warm-up knots
           FIRST, arm, flat armed settle so the firmware recovery slew converges).

        Returns None on success (armed, settled, ready to descend) or the reason the
        descent is impossible. One retry: a latch that fires DURING the first
        attempt's settle (e.g. a race with the fault task) gets one fresh
        clear-and-arm before the harness declares CANNOT PARK.
        """
        reason: Optional[str] = None
        for attempt in (1, 2):
            self._disarm()      # EXPLICIT: staleness check off while we take our time
            fs = self._fault_state()
            cls = sid.classify_fault(int(fs)) if fs is not None else 'none'
            if cls in ('fatal', 'unknown'):
                return (f"fault {sid.fault_name(int(fs))} is fatal — ceding "
                        f"authority, not fighting it for a descent")
            if cls == 'latching' and \
                    self.guard.recoveries_used > self.guard.max_recoveries:
                return (f"guard latch {sid.fault_name(int(fs))} with the "
                        f"CLEAR_ERRORS recovery budget exhausted "
                        f"({self.guard.recoveries_used}"
                        f"/{self.guard.max_recoveries})")
            if cls == 'latching':
                print(f"  park: {sid.fault_name(int(fs))} latched — disarmed "
                      f"CLEAR_ERRORS before the descent (attempt {attempt})")
            if not self._startup_clear_latch():
                return (f"guard latch would not clear "
                        f"({self._abort_reason or 'unknown'})")
            # Re-sample: an E-STOP de-energized the motor and the leg/mass has
            # fallen — arm where it ACTUALLY is, not where it was.
            pos, _, _, _ = self._sample()
            live = pos if pos is not None else live_rev
            self._bringup_closed_loop()          # disarmed — the 0.3 s sleep is safe
            settle = self._warm_and_arm(live)
            if not (settle.get('aborted') or settle.get('guard_latched')):
                return None
            reason = self._abort_reason or 'arm-settle latched'
            print(f"  park: re-arm attempt {attempt}/2 failed ({reason})"
                  + (" — retrying with a fresh latch clear" if attempt == 1 else ""))
        return f"could not re-arm for the descent after 2 attempts ({reason})"

    def _park_low(self, from_rev: Optional[float] = None):
        """Lower the leg to the bottom of the usable window, then disarm — on EVERY exit
        path, including the abort ones.

        Called between masses, at the end of a stage, and from ``run()``'s cleanup. The
        point is that the operator handles the weights — and swaps them — with the leg at
        its LOWEST, so the fall distance if anything lets go is ~0.

        TWO PATHS (the 2026-07-14 fix):

        * **fast path** — still armed, fault NONE, firmware agrees: stream the descent
          DIRECTLY. No disarm (the load never drops), no bring-up, no re-arm — and the
          entry checks run under ``_keep_stream_alive`` so the armed stream is never
          silent. The pre-fix code re-ran ``_bringup_closed_loop()`` here while armed
          and silent (RPCs + ``time.sleep(0.3)`` >> the 250 ms staleness window), which
          latched MPC_STALE on every healthy stage end and then failed its own re-arm:
          the operator's "CANNOT PARK — could not re-arm for the descent (arm-settle
          latched)".
        * **slow path** — disarmed or a fault latched (the load has already dropped to
          the hardstop in that case; disarming changes nothing): the full explicit
          dance in ``_park_rearm`` — disarm, re-baseline + clear latch, bring-up,
          stream-then-arm-then-settle, bounded retry.

        The abort paths also need the care the pre-fix version lacked:

        * **the SIGINT latch is bypassed for the descent** — an over-current trip and
          Ctrl-C both latch ``self._sigint``, which makes ``_stream_and_sample`` break on
          its first knot. It is cleared here and restored after; a FRESH Ctrl-C during
          the descent re-latches it (the signal handler stays installed) and stops the
          stream — the operator's escape hatch stays live;
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
            # Entry checks under the keep-alive: if we arrive armed and healthy, the
            # hold stream keeps the staleness clock fresh while we look around.
            with self._keep_stream_alive():
                pos, _, _, _ = self._sample()
                live = pos if pos is not None else from_rev
                fs = self._fault_state()
                cls = sid.classify_fault(int(fs)) if fs is not None else 'none'
                fw_active = self._fw_mpc_active()
                fast_path = (self._armed and cls == 'none'
                             and fw_active is not False)
                series = None
                if live is not None and live > target + PARK_DONE_TOL_REV \
                        and cls not in ('fatal', 'unknown') and fast_path:
                    start = live
                    series = kt.shaped_constant_velocity_knots(
                        start, target,
                        vel_rps=min(PARK_DESCENT_VEL_RPS, self.traverse_vel_rps),
                        seg_t_s=self.seg_t_s,
                        lead_out_frames=int(0.3 / self.seg_t_s))
            if live is None:
                cannot = "no telemetry — the leg's position is unknown (link down?)"
            elif live <= target + PARK_DONE_TOL_REV:
                pass    # already at (or below) the bottom; just make sure we disarm
            elif cls in ('fatal', 'unknown'):
                cannot = (f"fault {sid.fault_name(int(fs))} is fatal — ceding "
                          f"authority, not fighting it for a descent")
            else:
                if series is None:
                    # Slow path: disarmed and/or latched — full dance, bounded retry.
                    cannot = self._park_rearm(live)
                    if cannot is None:
                        with self._keep_stream_alive():
                            cur, _, _, _ = self._sample()
                            start = cur if cur is not None else live
                            series = kt.shaped_constant_velocity_knots(
                                start, target,
                                vel_rps=min(PARK_DESCENT_VEL_RPS,
                                            self.traverse_vel_rps),
                                seg_t_s=self.seg_t_s,
                                lead_out_frames=int(0.3 / self.seg_t_s))
                else:
                    start = live
                if cannot is None and series is not None:
                    print(f"  parking: {start:.2f} -> {target:.2f} rev "
                          f"(gentle descent)")
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

    def _run_traverse(self, mass_kg: float, rep: int, direction: str, *,
                      point_idx: int) -> Optional[dict]:
        """One constant-velocity traverse; returns the sampled arrays (or None on abort).

        The start/end are SHAPED (trapezoidal velocity, ``--accel-time`` ramps): an
        unshaped series steps 0 → 0.6 rev/s in one knot — a ~60 rev/s² accel demand
        ≈ +4.5 A of inertia current at 3 kg, enough to brush the over-current abort.
        The cruise portion (the only part the steady-window selector keeps) is identical.

        The prep (series build + CSV open + print) runs under the keep-alive hold
        stream: it happens between armed foreground streams, and an armed stream must
        never go silent past the 250 ms staleness window (file I/O on a loaded SD card
        is exactly the kind of pause that occasionally blows it).
        """
        up = direction == 'up'
        start = self.traverse_lo_rev if up else self.traverse_hi_rev
        end = self.traverse_hi_rev if up else self.traverse_lo_rev
        with self._keep_stream_alive():
            series = kt.shaped_constant_velocity_knots(
                start, end, vel_rps=self.traverse_vel_rps, seg_t_s=self.seg_t_s,
                accel_time_s=self.accel_time_s,
                lead_in_frames=int(0.3 / self.seg_t_s),
                lead_out_frames=int(0.3 / self.seg_t_s))
            v_cmd = kt.knots_achieved_velocity(series, self.seg_t_s)
            # point_idx in the name: duplicate masses are LEGITIMATE (repeatability
            # points) and a weighed override can land on another declared mass — without
            # the index those traverses would silently overwrite each other's CSVs (the
            # same bug class as the 2026-07-12 sysid ladder rung collision, 0182a11).
            f, w, path = self._open_csv(
                f"kt_pt{point_idx}_m{mass_kg:.2f}_rep{rep}_{direction}.csv")
            print(f"    {direction:>4}: {start:.2f} -> {end:.2f} rev  "
                  f"(cmd {v_cmd:.3f} rev/s, {len(series) * self.seg_t_s:.1f} s)",
                  end='', flush=True)
        res = self._stream_and_sample(series, guard=True, writer=w)
        with self._keep_stream_alive():
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

        # A6: results are persisted INCREMENTALLY — each traverse lands in the manifest
        # (with a write to disk) the moment it completes, so an abort loses NOTHING.
        # The 2026-07-14 manifests contained stages: {} because serialization only
        # happened at stage end, and every run aborted before reaching it — all eight
        # good traverses had to be recomputed offline from the raw CSVs.
        stage = {
            'status': 'in_progress',
            'rig_orientation': self.rig_orientation,
            'traverses': [],     # one entry per completed traverse, written immediately
            'per_mass': [],
            'points': [],
        }
        self._manifest['stages']['kt'] = stage
        self._persist_manifest()

        points: List[kt.MassPoint] = []

        for point_idx, mass in enumerate(self.masses_kg, start=1):
            choice, mass_used = self._prompt_mass(mass)
            if choice == 'skip':
                print("  skipped.")
                continue
            if choice == 'quit':
                self._abort_reason = "operator quit ('q'/EOF) at the mass prompt"
                print("  quitting.")
                if self._needs_park:
                    self._park_low()
                return False
            self._current_mass_kg = mass_used   # abort budget context (A5)

            approach = self._enter_hold_at_center()
            self._needs_park = True     # armed and moving — park on every exit from here
            if approach.get('aborted') or approach.get('guard_latched'):
                # The ACTUAL trigger first: an over-current trip during breakaway used
                # to be swallowed by the all-NaN telemetry-snapshot fallback here
                # ("NONE @ u0=+nan enc=+nan", 2026-07-14 at 2.75 kg).
                if not self._took_iq_abort() and not self._abort_reason:
                    self._abort_reason = self._abort_diag(approach)
                print(f"  ABORT during approach: {self._abort_reason}")
                self._park_low()
                return False

            up_stats: List[kt.TraverseStats] = []
            dn_stats: List[kt.TraverseStats] = []
            ok = True
            for rep in range(1, self.reps + 1):
                for direction in ('up', 'down'):
                    res = self._run_traverse(mass_used, rep, direction,
                                             point_idx=point_idx)
                    if res is None:
                        ok = False
                        break
                    # Analysis + the incremental manifest write happen under the
                    # keep-alive hold: never leave the armed stream silent (A3).
                    with self._keep_stream_alive():
                        st = kt.summarize_traverse(
                            res['t'], res['vel'], res['pos'], res['iq'],
                            direction=direction,
                            v_target_rps=self.traverse_vel_rps,
                            traverse_lo_rev=self.traverse_lo_rev,
                            traverse_hi_rev=self.traverse_hi_rev,
                            edge_discard_s=self.edge_discard_s)
                        print(f"  iq = {st.iq_mean_A:+.3f} ± {st.iq_sem_A:.3f} A  "
                              f"(n={st.n_samples}, v={st.vel_mean_rps:+.3f} rev/s)"
                              + ("" if st.ok else "  << " + "; ".join(st.reasons)))
                        (up_stats if direction == 'up' else dn_stats).append(st)
                        if not st.ok:
                            ok = False
                        stage['traverses'].append({
                            'mass_used_kg': mass_used,        # operator-entered (A1/A6)
                            'declared_mass_kg': mass,
                            'rep': rep, 'direction': direction,
                            'iq_mean_A': st.iq_mean_A, 'iq_sem_A': st.iq_sem_A,
                            'n_steady': st.n_samples,
                            'v_mean_rps': st.vel_mean_rps,
                            'csv': res['csv'], 'ok': st.ok,
                            'reasons': list(st.reasons),
                        })
                        self._persist_manifest()
                    # Dwell between traverses — let the leg come fully to rest so the next
                    # traverse's acceleration transient starts from a known state.
                    self._stream_and_sample(
                        np.full(int(self.dwell_s / self.seg_t_s),
                                self.traverse_hi_rev if direction == 'up'
                                else self.traverse_lo_rev), guard=True)
                if not ok:
                    break

            self._park_low()
            self._current_mass_kg = None

            if not ok or not up_stats or not dn_stats:
                print(f"  mass {mass_used:.2f} kg FAILED — aborting the run "
                      f"(a partial mass set cannot be fitted honestly)")
                return False

            # Average the reps, then combine up/down. Averaging reps BEFORE combining keeps
            # the up/down pairing intact, which is what makes friction cancel.
            up_mean = _mean_stats(up_stats, 'up')
            dn_mean = _mean_stats(dn_stats, 'down')
            pt = kt.combine_traverses(mass_used, up_mean, dn_mean)
            points.append(pt)
            print(f"  --> iq_avg = {pt.iq_avg_A:+.3f} ± {pt.iq_avg_sem_A:.3f} A "
                  f"(gravity, friction cancelled)")
            print(f"      iq_halfdiff = {pt.iq_halfdiff_A:+.3f} A "
                  f"(|{abs(pt.iq_halfdiff_A):.3f}| A vs tau_c {kt.TAU_C_REF_A:.2f} A)")
            stage['per_mass'].append({
                'mass_kg': mass_used, 'declared_mass_kg': mass, 'point': asdict(pt),
                'up': [asdict(s) for s in up_stats],
                'down': [asdict(s) for s in dn_stats],
            })
            stage['points'] = [asdict(p) for p in points]
            self._persist_manifest()             # parked + disarmed here: plain write

        if len(points) < 3:
            self._abort_reason = (f"only {len(points)} mass points completed — need >= 3 "
                                  f"to fit slope + intercept")
            print(f"\n  ABORT: {self._abort_reason}")
            return False

        self._report_kt(points, stage)
        return True

    def _report_kt(self, points: List[kt.MassPoint], stage: dict):
        fit = kt.fit_kt(points, tilt_deg=self.tilt_deg)
        verdict = kt.classify_kt(fit)
        friction = kt.friction_consistency(points)
        # Both sign inferences run THROUGH the operator's --rig-orientation: the
        # 2026-07-14 manifests recorded extension_iq_sign=-1 wrong-way-round because
        # the rig was inverted and the code assumed holding == extension.
        sign = kt.infer_extension_iq_sign(fit, rig_orientation=self.rig_orientation)
        signs_agree = kt.slope_friction_sign_agree(
            fit, points, rig_orientation=self.rig_orientation)

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

        stage.update({
            'status': 'complete',
            'points': [asdict(p) for p in points],
            'fit': asdict(fit),
            'verdict': asdict(verdict),
            'friction_check': asdict(friction),
            'sign_inference': asdict(sign),
            'slope_friction_signs_agree': signs_agree,
            'implications': self._kt_implications(fit, verdict, friction, signs_agree),
        })
        self._persist_manifest()

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

    # -- MODE 2: torque_ff channel (EDGE CAPTURE) ------------------------------

    def _stream_and_sample_tff(self, u0_rev: float, tff_series, *, writer=None) -> dict:
        """A flat-position armed stream whose ``torque_ff`` follows ``tff_series`` knot
        by knot — the square-wave driver.

        Mirrors ``BridgeSysID._stream_and_sample`` (pacing, telemetry-frame CSV via
        ``_begin/_end_telem_log``, the guard machine, the SIGINT/mpc_active/OSError
        abort paths) because the base loop has no per-knot feedforward hook and this
        file must not edit ``bench_leg_sysid``. Keep the safety checks in sync with
        the base if it ever changes.
        """
        n = len(tff_series)
        aborted = False
        guard_latched = False
        guard_action = None
        period = 1.0 / self.setpoint_hz
        u0 = float(u0_rev)
        self._begin_telem_log(writer, u0)
        t0 = time.perf_counter()
        pos = None
        for i in range(n):
            if self._sigint:
                aborted = True
                self._abort_reason = 'SIGINT'
                self._disarm()
                break
            self._torque_ff_Nm = float(tff_series[i])
            now_us = int(time.time() * 1_000_000)
            self._set_log_u0(u0)
            try:
                self._send_knot(u0, u0, u0, 0.0, now_us)
            except OSError as e:
                aborted = True
                self._abort_reason = f"setpoint send failed: {e}"
                self._disarm()
                break
            pos, _vel, _iq, _ts = self._sample()
            fs = self._fault_state()
            if fs is not None:
                act = self.guard.observe(fs)
                if act.kind == 'backoff_recover':
                    guard_latched = True
                    guard_action = act
                    _enc = pos if pos is not None else float('nan')
                    print(f"    GUARD LATCH -> {act.reason} "
                          f"[u0={u0:+.4f} enc={_enc:+.4f} rev, "
                          f"lead_clamp={self._lead_clamp_bit()}]")
                    self._disarm()
                    if act.clear_errors:
                        try:
                            self._clear_errors()
                        except Exception:  # noqa: BLE001
                            pass
                    break
                if act.kind == 'abort':
                    aborted = True
                    guard_action = act
                    self._abort_reason = act.reason
                    self._disarm()
                    break
            if i * period > BRIDGE_ARM_VERIFY_GRACE_S \
                    and self._fw_mpc_active() is False:
                aborted = True
                self._abort_reason = (
                    "firmware mpc_active=0 after arming — another heartbeat "
                    "authority is overriding it (is a ROS2 teensy_bridge_node "
                    "running? this driver must be the sole wire authority)")
                self._disarm()
                break
            target = t0 + (i + 1) * period
            dt = target - time.perf_counter()
            if dt > 0:
                time.sleep(dt)
        out = self._end_telem_log()
        out['last_u0'] = u0
        out['last_pos'] = pos if pos is not None else float('nan')
        out['aborted'] = aborted
        out['guard_latched'] = guard_latched
        out['guard_action'] = guard_action
        return out

    def _hold_flat(self, dur_s: float, *, writer=None) -> dict:
        """Stream a flat armed hold at the Mode-2 hold position for ``dur_s``."""
        return self._stream_and_sample(
            np.full(max(1, int(dur_s / self.seg_t_s)), self.tff_hold_rev),
            guard=True, writer=writer)

    def _wait_hold_settled(self):
        """The settle gate (B1): hold flat, evaluating :func:`kt_lib.hold_settled` on a
        rolling window until it passes or ``SETTLE_MAX_WAIT_S`` elapses.

        Returns ``(check, hold_iq_A, waited_s)``; ``check`` is None if the underlying
        stream aborted/latched (the caller handles the abort), otherwise the final
        :class:`kt_lib.SettleCheck` — ``check.settled`` False after the timeout means
        the hold NEVER settled and the stage must refuse to measure (tonight's ladder
        measured exactly such unsettled transients and called them steady states).
        """
        buf_t: List[float] = []
        buf_pos: List[float] = []
        buf_iq: List[float] = []
        t_off = 0.0
        waited = 0.0
        chk = None
        print(f"  settle gate: pos within ±{kt.SETTLE_POS_TOL_REV * 1e3:.1f} mrev of "
              f"cmd AND |d(iq)/dt| < {kt.SETTLE_DIQ_DT_MAX_A_PER_S:.2f} A/s over "
              f"{kt.SETTLE_WINDOW_S:.0f} s (max wait {kt.SETTLE_MAX_WAIT_S:.0f} s)")
        while waited < kt.SETTLE_MAX_WAIT_S - 1e-9:
            res = self._hold_flat(SETTLE_CHUNK_S)
            if self._took_iq_abort() or res['aborted'] or res['guard_latched']:
                if not self._abort_reason:
                    self._abort_reason = self._abort_diag(res)
                return None, float('nan'), waited
            waited += SETTLE_CHUNK_S
            t = np.asarray(res['t'], float)
            if t.size:
                buf_t.extend((t + t_off).tolist())
                buf_pos.extend(np.asarray(res['pos'], float).tolist())
                buf_iq.extend(np.asarray(res['iq'], float).tolist())
                t_off = buf_t[-1] + self.seg_t_s
            # Keep ~2 windows of history — the gate only looks at the trailing window.
            max_keep = int(2.5 * kt.SETTLE_WINDOW_S * BENCH_BUILD_TELEM_HZ)
            if len(buf_t) > max_keep:
                buf_t = buf_t[-max_keep:]
                buf_pos = buf_pos[-max_keep:]
                buf_iq = buf_iq[-max_keep:]
            chk = kt.hold_settled(buf_t, buf_pos, buf_iq, cmd_rev=self.tff_hold_rev)
            if waited >= kt.SETTLE_WINDOW_S:     # need a full window before judging
                print(f"    [{waited:4.0f} s] pos_err="
                      f"{chk.pos_err_rev * 1e3 if math.isfinite(chk.pos_err_rev) else float('nan'):+7.3f} mrev  "
                      f"d(iq)/dt={chk.diq_dt_A_per_s if math.isfinite(chk.diq_dt_A_per_s) else float('nan'):+7.3f} A/s  "
                      f"{'SETTLED' if chk.settled else 'waiting'}")
                if chk.settled:
                    break
        hold_iq = float('nan')
        if buf_iq:
            arr = np.asarray(buf_iq, float)
            arr = arr[np.isfinite(arr)]
            if arr.size:
                hold_iq = float(np.mean(arr[-int(kt.SETTLE_WINDOW_S * BENCH_BUILD_TELEM_HZ):]))
        return chk, hold_iq, waited

    def stage_torque_ff_check(self) -> bool:
        print("\n=== MODE 2: torque_ff channel validation by EDGE CAPTURE ===")
        self.center_rev = self.tff_hold_rev
        self._current_mass_kg = self.tff_hold_mass_kg
        per_amp_s = 2.0 * self.tff_cycles * self.tff_half_period_s
        n_edges = 2 * self.tff_cycles - 1

        # The honest preamble: what will move (NOTHING), what the operator should see,
        # how long it takes.
        print(f"\n  NOTHING WILL MOVE: this is a static hold at "
              f"{self.tff_hold_rev:.2f} rev with {self.tff_hold_mass_kg:.2f} kg on the "
              f"leg, plus torque_ff")
        print(f"  toggles too small to break the static-friction lock (±0.36 A "
              f"excursion at X=0.020 Nm). You")
        print(f"  should see the leg hold still and hear nothing; the current trace "
              f"does the measuring.")
        print(f"  Sequence: approach -> pre-soak {self.pre_soak_s:.0f} s -> settle "
              f"gate (<= {kt.SETTLE_MAX_WAIT_S:.0f} s) -> "
              f"{len(self.tff_amps_Nm)} amplitude(s)")
        print(f"  x {per_amp_s:.0f} s each ({self.tff_cycles} cycles @ "
              f"{self.tff_half_period_s:.2f} s half-period = {n_edges} edges) -> park. "
              f"Total ~"
              f"{(self.pre_soak_s + kt.SETTLE_MAX_WAIT_S / 2 + len(self.tff_amps_Nm) * (per_amp_s + 2.0) + 30) / 60:.0f} min.")
        print(f"  Why edges, not a rung ladder: at a settled hold the steady-state iq "
              f"response to a CONSTANT")
        print(f"  torque_ff is ZERO (the integrator re-absorbs it, tau ~ 2-10 s) — "
              f"only the instantaneous JUMP")
        print(f"  at each ±X toggle carries the channel gain. Expect "
              f"|d(iq)/d(tff)| = 1/{kt.KT_ODRIVE_CONFIGURED:.6f} = "
              f"{1.0 / kt.KT_ODRIVE_CONFIGURED:.2f} A/Nm per edge.")
        swing_warns = kt.torque_ff_swing_warnings(self.tff_amps_Nm)
        if not swing_warns:
            print(f"  Amplitudes: "
                  f"{', '.join(f'{a:.3f}' for a in self.tff_amps_Nm)} Nm — every "
                  f"SWING (2X) inside the {kt.TFF_EDGE_SWING_MAX_NM:.3f} Nm in-pool "
                  f"bound (static band edge {kt.TFF_STATIC_BAND_MIN_NM:.3f} Nm).")
        else:
            print(f"  Amplitudes: "
                  f"{', '.join(f'{a:.3f}' for a in self.tff_amps_Nm)} Nm")
            for w in swing_warns:
                print(f"  WARNING: {w}")
        if not self.assume_yes:
            print(f"\n  Check: {self.tff_hold_mass_kg:.2f} kg SECURE, hanging FREE "
                  f"and VERTICAL · NOBODY touches the mass,")
            print(f"  the leg or the bench during the toggles (a knocked edge is "
                  f"trimmed, but a held mass poisons")
            print(f"  everything — the 2026-07-14 hand-supported rung read as an "
                  f"unloaded leg).")
            try:
                if input("  Press ENTER to begin (or 'q' to quit): "
                         ).strip().lower().startswith('q'):
                    self._abort_reason = "operator quit ('q') at the Mode-2 prompt"
                    return False
            except (EOFError, KeyboardInterrupt):
                print()
                self._abort_reason = "operator quit (EOF/Ctrl-C) at the Mode-2 prompt"
                return False

        stage = {
            'status': 'in_progress',
            'design': 'edge_capture',
            'rig_orientation': self.rig_orientation,
            'amplitudes': [],       # per-amplitude results, persisted as they land
        }
        self._manifest['stages']['torque_ff_check'] = stage
        self._persist_manifest()

        approach = self._enter_hold_at_center()
        self._needs_park = True         # armed and moving — park on every exit from here
        if approach.get('aborted') or approach.get('guard_latched'):
            if not self._took_iq_abort() and not self._abort_reason:
                self._abort_reason = self._abort_diag(approach)
            print(f"  ABORT during approach: {self._abort_reason}")
            self._park_low()
            return False

        # PRE-SOAK: the friction-band load-transfer transient (~1.2–1.4 A ≈ tau_c
        # amplitude, tau ≈ 3.2 s) is dead in 15 s. Soak BEFORE the settle gate so the
        # gate measures the hold, not the arrival transient. Also lets the firmware
        # recovery slew (leg_interp.cpp:479, zeroes torque_ff while it runs) converge.
        if self.pre_soak_s > 0:
            print(f"\n  pre-soak: {self.pre_soak_s:.0f} s flat hold (load-transfer "
                  f"transient ~1.2-1.4 A, tau ~ 3.2 s — dead in 15 s)")
            soaked = 0.0
            while soaked < self.pre_soak_s - 1e-9:
                chunk = min(PRE_SOAK_CHUNK_S, self.pre_soak_s - soaked)
                res = self._hold_flat(chunk)
                if self._took_iq_abort() or res['aborted'] or res['guard_latched']:
                    if not self._abort_reason:
                        self._abort_reason = self._abort_diag(res)
                    print(f"  ABORT during pre-soak: {self._abort_reason}")
                    self._park_low()
                    return False
                soaked += chunk
                print(f"    [{soaked:4.0f}/{self.pre_soak_s:.0f} s]")

        chk, hold_iq, waited = self._wait_hold_settled()
        if chk is None:
            print(f"  ABORT during the settle gate: {self._abort_reason}")
            self._park_low()
            return False
        with self._keep_stream_alive():   # armed: the manifest write must not starve
            stage['settle'] = dict(asdict(chk), waited_s=waited, hold_iq_A=hold_iq)
            self._persist_manifest()
        if not chk.settled:
            self._abort_reason = (
                f"hold NEVER settled within {kt.SETTLE_MAX_WAIT_S:.0f} s "
                f"({'; '.join(chk.reasons)}) — REFUSING to measure: an unsettled hold "
                f"is exactly what faked the 2026-07-14 ladder results. Is something "
                f"touching the rig? Try a longer --pre-soak.")
            print(f"\n  ABORT: {self._abort_reason}")
            self._park_low()
            return False
        print(f"  settled after {waited:.0f} s; hold current = {hold_iq:+.3f} A")

        # The gravity-calibrated extension sign, THROUGH the rig orientation. Prefer
        # Mode 1's slope inference when this run did both; otherwise the settled hold:
        # the hold current is the torque holding the load, and --rig-orientation says
        # whether holding is extension (normal) or contraction (inverted).
        kt_stage = self._manifest['stages'].get('kt') or {}
        sign_from_kt = (kt_stage.get('sign_inference') or {}).get('iq_extension_sign', 0)
        if sign_from_kt in (1, -1):
            ext_sign = int(sign_from_kt)
            sign_src = "Mode 1's gravity-loaded slope (through --rig-orientation)"
        else:
            ext_sign = kt.extension_sign_from_hold(hold_iq, self.rig_orientation)
            sign_src = (f"the settled hold current ({hold_iq:+.2f} A) through "
                        f"--rig-orientation {self.rig_orientation} (holding == "
                        f"{'extension' if self.rig_orientation == 'normal' else 'contraction'})")
            if ext_sign == 0:
                sign_src = (f"unavailable — |hold current| {abs(hold_iq):.2f} A is too "
                            f"small to calibrate the frame (no mass on the leg?)")

        per_amp: List[kt.EdgeCaptureAmplitude] = []
        for amp in self.tff_amps_Nm:
            tff_series, toggles = kt.square_wave_tff_series(
                amp, seg_t_s=self.seg_t_s, half_period_s=self.tff_half_period_s,
                n_cycles=self.tff_cycles)
            with self._keep_stream_alive():
                f, w, path = self._open_csv(f"tff_edge_{amp:.3f}Nm.csv")
                print(f"\n  X = {amp:.3f} Nm: ±{amp / kt.KT_ODRIVE_CONFIGURED:.2f} A "
                      f"expected excursion, {toggles.size} edges over "
                      f"{len(tff_series) * self.seg_t_s:.0f} s -> "
                      f"{os.path.basename(path)}")
            res = self._stream_and_sample_tff(self.tff_hold_rev, tff_series, writer=w)
            self._torque_ff_Nm = 0.0
            with self._keep_stream_alive():
                f.close()
            if self._took_iq_abort() or res['aborted'] or res['guard_latched']:
                if not self._abort_reason:
                    self._abort_reason = self._abort_diag(res)
                print(f"\n  ABORT at X={amp:.3f} Nm: {self._abort_reason}")
                self._park_low()
                return False
            with self._keep_stream_alive():
                ar = kt.analyze_edge_capture(
                    res['t'], res['iq'], toggles, amp,
                    half_period_s=self.tff_half_period_s)
                per_amp.append(ar)
                print(f"    slope {ar.slope_A_per_Nm:+8.2f} ± "
                      f"{ar.slope_sem_A_per_Nm:.2f} A/Nm  "
                      f"(edges {ar.n_kept}/{ar.n_toggles} kept, CV "
                      f"{ar.jump_cv * 100 if math.isfinite(ar.jump_cv) else float('inf'):.0f}%, "
                      f"tau {ar.tau_s:.1f} s{'' if ar.tau_fitted else ' (default)'}"
                      f"{', DRIFT DETECTED' if ar.drift_detected else ''})")
                for r in ar.reasons:
                    print(f"    !! {r}")
                d = asdict(ar)
                d['csv'] = os.path.basename(path)
                stage['amplitudes'].append(d)
                self._persist_manifest()
            # Short zero-tff re-settle between amplitudes.
            res = self._hold_flat(2.0)
            if self._took_iq_abort() or res['aborted'] or res['guard_latched']:
                if not self._abort_reason:
                    self._abort_reason = self._abort_diag(res)
                print(f"\n  ABORT between amplitudes: {self._abort_reason}")
                self._park_low()
                return False

        self._torque_ff_Nm = 0.0
        self._hold_flat(0.5)
        self._park_low()
        self._current_mass_kg = None

        self._report_torque_ff(per_amp, stage, ext_sign, sign_src)
        return True

    def _report_torque_ff(self, per_amp: List[kt.EdgeCaptureAmplitude], stage: dict,
                          ext_sign: int, sign_src: str):
        pooled = kt.pool_edge_capture(per_amp)
        verdict = kt.classify_edge_capture(pooled, ext_sign)

        print("\n" + "=" * 72)
        print("  RESULT — torque_ff channel (edge capture)")
        print("=" * 72)
        print(f"  {'X (Nm)':>8} {'edges':>7} {'slope (A/Nm)':>16} {'CV':>6} "
              f"{'tau (s)':>8} {'drift':>6}")
        for a in per_amp:
            print(f"  {a.amplitude_Nm:8.3f} {a.n_kept:3d}/{a.n_toggles:<3d} "
                  f"{a.slope_A_per_Nm:+9.2f} ± {a.slope_sem_A_per_Nm:<5.2f} "
                  f"{a.jump_cv * 100 if math.isfinite(a.jump_cv) else float('inf'):5.0f}% "
                  f"{a.tau_s:8.1f} {'YES' if a.drift_detected else 'no':>6}")
        # Excluded amplitudes are part of the story on EVERY path (audit
        # 2026-07-15): a trustworthy verdict printed beside a chaotic per-amp row
        # with no explanation is the console-narrative blindness this harness
        # keeps getting burned by. Not folded into failures — exclusion is
        # data-quality bookkeeping, not a verdict veto.
        for x in pooled.excluded_amplitudes:
            print(f"  {x}")
        print(f"\n  pooled slope = {pooled.slope_A_per_Nm:+.2f} ± "
              f"{pooled.slope_sem_A_per_Nm:.2f} A/Nm   "
              f"(t = {pooled.t_stat:.1f}, CV = "
              f"{pooled.jump_cv * 100 if math.isfinite(pooled.jump_cv) else float('inf'):.0f}%, "
              f"{pooled.n_edges_kept} edges over {sum(1 for a in per_amp if not any(f'{a.amplitude_Nm:.3f}' in x for x in pooled.excluded_amplitudes))} pooled amplitudes)")
        print(f"  verdict gate (t >= {kt.TFF_GATE_MIN_T_STAT:.0f} AND CV <= "
              f"{kt.TFF_EDGE_CV_MAX * 100:.0f}% AND no drift): "
              f"{'PASS' if pooled.trustworthy else '*** FAIL ***'}")
        print(f"\n  extension iq sign = {ext_sign:+d}, from {sign_src}")
        print()
        for line in verdict.lines:
            print(f"  {line}")

        print(f"\n  WHAT THIS MEANS FOR THE GRAVITY FEEDFORWARD:")
        for line in self._tff_implications(pooled, verdict):
            print(f"    {line}")
        print("=" * 72)

        pooled_d = asdict(pooled)
        pooled_d.pop('per_amplitude', None)    # already persisted per-amplitude
        stage.update({
            'status': 'complete',
            'pooled': pooled_d,
            'verdict': asdict(verdict),
            'extension_iq_sign': ext_sign,
            'extension_iq_sign_source': sign_src,
            'implications': self._tff_implications(pooled, verdict),
        })
        self._persist_manifest()

    def _tff_implications(self, pooled, verdict) -> List[str]:
        out: List[str] = []
        if verdict.no_conclusion:
            out.append("NO CONCLUSION: this run says NOTHING about the torque_ff "
                       "channel — sign, scale OR liveness (channel_live is recorded "
                       "as None, not False). Do not act on it in either direction.")
            out.append("Fix the setup and re-run: hang the declared mass FREE and "
                       "VERTICAL; NOBODY touches the rig during the toggles; let the "
                       "settle gate pass before measuring; keep every amplitude "
                       f"inside the static-friction band (X <= "
                       f"{kt.TFF_BAND_WARN_NM:.3f} Nm).")
            return out
        if verdict.channel_live is False:
            out.append("The torque_ff channel is DEAD — a demonstrated PRECISE NULL "
                       "(the pooled slope is bounded near zero from clean, drift-free "
                       "edges). DO NOT ship a gravity FF.")
            out.append("Check: is the leg mid-stroke? (leg_interp.cpp:407 — the "
                       "stroke clamp ZEROES torque_ff). Did the recovery slew still "
                       "have the output? (:479 also zeroes it).")
            return out
        if verdict.positive_tff_extends is True:
            out.append("SIGN: a POSITIVE wire torque_ff EXTENDS — CONFIRMS the "
                       "settled 2026-07-14 production-chain finding (no negation "
                       "anywhere). The gravity-FF implementer can rely on it.")
        elif verdict.positive_tff_extends is False:
            out.append("SIGN: a POSITIVE wire torque_ff RETRACTS — this CONTRADICTS "
                       "the settled 2026-07-14 production-chain finding. Before "
                       "believing a wire flip, re-check the --rig-orientation "
                       "declaration (the extension sign is inferred THROUGH it; an "
                       "inverted/normal mix-up produces exactly this).")
        else:
            out.append("SIGN: not determined — put a mass on the leg, or run "
                       "--mode kt first to calibrate the extension sign from gravity.")
        if verdict.scale_ok:
            out.append(f"SCALE: the ODrive really does use its configured "
                       f"torque_constant ({kt.KT_ODRIVE_CONFIGURED:.5f}) to turn "
                       f"commanded Nm into current, and the int16 x10000 wire scaling "
                       f"round-trips. torque_ff behaves exactly like input_torque.")
            out.append(f"=> The MECHANICAL torque delivered per commanded Nm is "
                       f"Kt_true/{kt.KT_ODRIVE_CONFIGURED:.5f}. With the preliminary "
                       f"Kt=0.0577 that is "
                       f"{kt.mechanical_torque_delivered_Nm(1.0, 0.0577):.3f} Nm per "
                       f"1.0 Nm commanded; CONFIRM Kt with weighed masses (mode kt) "
                       f"before shipping the correction.")
        else:
            out.append(f"SCALE: MISMATCH — the delivered current is NOT torque_ff / "
                       f"torque_constant. A gravity FF would land "
                       f"{(abs(pooled.slope_A_per_Nm) * kt.KT_ODRIVE_CONFIGURED - 1) * 100:+.0f}% "
                       f"off from this alone, before the Kt question is even "
                       f"considered. Find out why before shipping.")
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
            per_amp_s = 2.0 * self.tff_cycles * self.tff_half_period_s
            n_edges = 2 * self.tff_cycles - 1
            print(f"\n  [MODE 2: torque_ff_check — EDGE CAPTURE]  hold "
                  f"{self.tff_hold_rev:.2f} rev with {self.tff_hold_mass_kg:.2f} kg "
                  f"(NOTHING moves)")
            print(f"    WHY EDGES: at a settled hold the steady-state iq response to a "
                  f"CONSTANT torque_ff is ZERO")
            print(f"    (the integrator re-absorbs it, tau ~ 2-10 s); the old rung "
                  f"ladder measured that transient at a")
            print(f"    fixed cadence, so drift aliased into the slope (the 2026-07-14 "
                  f"20:55 -21 A/Nm). Only the")
            print(f"    instantaneous JUMP at a ± toggle carries the channel gain; "
                  f"alternating polarity de-aliases drift.")
            print(f"    sequence: pre-soak {self.pre_soak_s:.0f} s -> settle gate "
                  f"(pos ±{kt.SETTLE_POS_TOL_REV * 1e3:.1f} mrev, |d(iq)/dt| < "
                  f"{kt.SETTLE_DIQ_DT_MAX_A_PER_S:.2f} A/s over "
                  f"{kt.SETTLE_WINDOW_S:.0f} s, max {kt.SETTLE_MAX_WAIT_S:.0f} s)")
            print(f"    then per amplitude: {self.tff_cycles} cycles @ "
                  f"±X, half-period {self.tff_half_period_s:.2f} s = {n_edges} edges "
                  f"in {per_amp_s:.0f} s, telemetry {BENCH_BUILD_TELEM_HZ:.0f} Hz")
            print(f"      {'X (Nm)':>9} {'wire(Nm)':>9} {'iq excursion':>13} "
                  f"{'edge jump':>10}")
            for a in self.tff_amps_Nm:
                print(f"      {a:9.3f} {kt.wire_quantized_torque_Nm(a):+9.4f} "
                      f"{a / kt.KT_ODRIVE_CONFIGURED:+10.2f} A "
                      f"{2 * a / kt.KT_ODRIVE_CONFIGURED:9.2f} A")
            print(f"    expected |slope| = 1/{kt.KT_ODRIVE_CONFIGURED:.6f} = "
                  f"{1.0 / kt.KT_ODRIVE_CONFIGURED:.2f} A/Nm; sign referred through "
                  f"--rig-orientation {self.rig_orientation}")
            _sw = kt.torque_ff_swing_warnings(self.tff_amps_Nm)
            if not _sw:
                print(f"    every amplitude's SWING (2X) inside the "
                      f"{kt.TFF_EDGE_SWING_MAX_NM:.3f} Nm in-pool bound — the lock keeps the "
                      f"loop blind at every toggle")
            else:
                for w in _sw:
                    print(f"    WARNING: {w}")
            print(f"    position loop blind, so the jump is the pure channel gain "
                  f"(and the leg audibly does nothing).")
            for w in kt.torque_ff_ladder_band_warnings(
                    [s * a for a in self.tff_amps_Nm for s in (+1.0, -1.0)]):
                print(f"    WARN: {w}")
            print(f"    edge windows: [t_e+{kt.EDGE_WINDOW_LO_S:.2f}, "
                  f"t_e+{kt.EDGE_WINDOW_HI_S:.2f}] vs "
                  f"[t_e-{kt.EDGE_WINDOW_HI_S:.2f}, t_e-{kt.EDGE_WINDOW_LO_S:.2f}] s, "
                  f"edges located by matched filter (±{kt.EDGE_SEARCH_S:.2f} s)")
            print(f"    with per-hold tau decay correction (default "
                  f"{kt.DEFAULT_HOLD_TAU_S:.0f} s; creep costs 2.5-9.5% uncorrected)")
            print(f"    verdict gate: |slope|/sigma >= {kt.TFF_GATE_MIN_T_STAT:.0f} "
                  f"AND trimmed edge-jump CV <= {kt.TFF_EDGE_CV_MAX * 100:.0f}% AND "
                  f"no paired-edge drift,")
            print(f"    else NO CONCLUSION (channel_live recorded as None — False is "
                  f"reserved for a demonstrated precise null).")
            print(f"    hold is {self.tff_hold_rev:.2f} rev — clear of the firmware "
                  f"stroke clamp (0.071 / 3.900), which would ZERO")
            print(f"    torque_ff (leg_interp.cpp:407).")

        print(f"\n  SIGN CONVENTION (the trap):")
        print(f"    can_buses.cpp:85-86 negates pos/vel (-> extension-positive), but :92")
        print(f"    stores iq_measured RAW in the ODrive frame. So an EXTENDING torque is")
        print(f"    predicted to read as {kt.IQ_EXTENSION_SIGN_PREDICTED:+d} iq. The harness")
        print(f"    does not assume this — gravity calibrates it THROUGH the operator's")
        print(f"    --rig-orientation ({self.rig_orientation}: holding the load is "
              f"{'extension' if self.rig_orientation == 'normal' else 'contraction'}).")
        print(f"    The 2026-07-14 manifests recorded the sign wrong-way-round because")
        print(f"    the rig was inverted and the code assumed holding == extension.")
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
            # The abort diagnostic goes INTO the manifest (A6): tonight's manifests
            # carried no abort context at all — the console scrollback was the only
            # record of why a run died.
            if self._abort_reason:
                self._manifest['abort_reason'] = self._abort_reason
            self._manifest['exit_code'] = rc
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
                    "removed, and validate the torque_ff channel end-to-end by edge "
                    "capture.",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--mode', choices=['kt', 'torque_ff_check', 'all'], default='kt')
    ap.add_argument('--dry-run', action='store_true',
                    help="print the plan (budget, distinguishing power) — no I/O")
    ap.add_argument('--yes', action='store_true',
                    help="skip the per-mass operator confirmation prompts")
    ap.add_argument('--home', action='store_true',
                    help="run the firmware HOME(axis) RPC first")
    # REQUIRED, no default: every sign inference runs through it. The 2026-07-14
    # manifests recorded extension_iq_sign wrong-way-round because the rig was
    # inverted and the harness assumed holding == extension.
    ap.add_argument('--rig-orientation', choices=list(kt.RIG_ORIENTATIONS),
                    default=None,
                    help="REQUIRED. normal = leg EXTENSION raises the load; "
                         "inverted = leg CONTRACTION raises it.")

    # Mode-scoped flags use default=None sentinels so kt.validate_mode_flags can
    # REFUSE a mode-mismatched flag instead of silently ignoring it (--hold-mass in
    # --mode kt mislabelled every 2026-07-14 run as 1.00 kg). Real defaults are
    # filled in AFTER validation.
    # -- mode kt only --
    ap.add_argument('--masses', type=str, default=None,
                    help="comma-separated masses in kg (default: the recommended "
                         "ladder). Mode kt only.")
    ap.add_argument('--traverse-lo', type=float, default=None,
                    help=f"traverse bottom, rev (default {DEFAULT_TRAVERSE_LO_REV})")
    ap.add_argument('--traverse-hi', type=float, default=None,
                    help=f"traverse top, rev (default {DEFAULT_TRAVERSE_HI_REV})")
    ap.add_argument('--vel', type=float, default=None,
                    help=f"constant traverse velocity, rev/s (default "
                         f"{kt.DEFAULT_TRAVERSE_VEL_RPS}; must clear the "
                         f"{kt.OMEGA_S_REV_S} rev/s stiction knee)")
    ap.add_argument('--reps', type=int, default=None,
                    help=f"up/down pairs per mass (default {DEFAULT_REPS})")
    ap.add_argument('--dwell', type=float, default=None,
                    help=f"rest between traverses, s (default {DEFAULT_DWELL_S})")
    ap.add_argument('--edge-discard', type=float, default=None,
                    help=f"seconds discarded from each time-end of a traverse "
                         f"(default {DEFAULT_EDGE_DISCARD_S}; the position-margin "
                         f"gate is the primary ramp exclusion)")
    ap.add_argument('--accel-time', type=float, default=None,
                    help=f"traverse start/end velocity-ramp duration, s (default "
                         f"{kt.DEFAULT_TRAVERSE_ACCEL_TIME_S}; unshaped starts demand "
                         f"~60 rev/s² ≈ +4.5 A at 3 kg)")

    # -- mode torque_ff_check only (EDGE CAPTURE) --
    ap.add_argument('--hold-mass', type=float, default=None,
                    help=f"mass on the leg during the torque_ff check, kg (default "
                         f"{DEFAULT_TFF_HOLD_MASS_KG}). It calibrates the extension "
                         f"sign AND loads the position loop. Mode torque_ff_check "
                         f"only.")
    ap.add_argument('--tff-hold', type=float, default=None,
                    help=f"hold position, rev (default {DEFAULT_TFF_HOLD_REV})")
    ap.add_argument('--tff-amps', type=str, default=None,
                    help=f"comma-separated square-wave amplitudes X in Nm (default "
                         f"{DEFAULT_TFF_AMPS}; the wave toggles ±X). All must sit "
                         f"inside the {kt.TFF_BAND_WARN_NM:.3f} Nm static-friction "
                         f"band.")
    ap.add_argument('--tff-half-period', type=float, default=None,
                    help=f"square-wave half-period, s (default "
                         f"{kt.DEFAULT_TFF_HALF_PERIOD_S}; recommended "
                         f"{kt.TFF_HALF_PERIOD_RANGE_S[0]}-"
                         f"{kt.TFF_HALF_PERIOD_RANGE_S[1]} s)")
    ap.add_argument('--tff-cycles', type=int, default=None,
                    help=f"full ± cycles per amplitude (default "
                         f"{kt.DEFAULT_TFF_CYCLES}; 2n−1 measured edges)")
    ap.add_argument('--pre-soak', type=float, default=None,
                    help=f"flat-hold soak before any static-hold measurement, s "
                         f"(default {kt.DEFAULT_PRE_SOAK_S:.0f}; the friction-band "
                         f"load-transfer transient ~1.2-1.4 A, tau ~3.2 s, is dead "
                         f"in 15 s)")

    ap.add_argument('--tilt-deg', type=float, default=0.0,
                    help="leg's departure from vertical; gravity torque scales by cos(tilt). "
                         "KEEP THE LEG VERTICAL — this is a correction, not a licence.")
    ap.add_argument('--axis', type=int, default=0)
    ap.add_argument('--stroke-cap', type=float, default=3.0)
    ap.add_argument('--current-limit', type=float, default=HARD_CURRENT_LIMIT_A)
    ap.add_argument('--vel-cap', type=float, default=4.0)
    ap.add_argument('--knot-hz', type=float, default=BENCH_BUILD_KNOT_HZ,
                    help="MUST be 100 to match BENCH_SYSID_BUILD's SEGMENT_T_S = 0.010")
    ap.add_argument('--output-dir', type=str, default=None)
    args = ap.parse_args()

    # A1: refuse mode-mismatched flags instead of silently ignoring them, and demand
    # the rig orientation. Pure + unit-tested in kt_lib.validate_mode_flags.
    scoped = list(kt.MODE1_ONLY_FLAGS) + list(kt.MODE2_ONLY_FLAGS)
    provided = [dest for dest in scoped if getattr(args, dest) is not None]
    if args.rig_orientation is not None:
        provided.append('rig_orientation')
    problems = kt.validate_mode_flags(args.mode, provided)
    if problems:
        for p in problems:
            print(f"REJECT: {p}", file=sys.stderr)
        return 1

    if args.stroke_cap > HARD_STROKE_CEIL_REV:
        print(f"REJECT: stroke cap {args.stroke_cap} above the hard ceiling "
              f"{HARD_STROKE_CEIL_REV}", file=sys.stderr)
        return 1

    def _default(v, d):
        return d if v is None else v

    masses = _parse_floats(args.masses) if args.masses else kt.recommended_masses_kg()
    amps = (_parse_floats(args.tff_amps) if args.tff_amps
            else list(kt.DEFAULT_TFF_EDGE_AMPS_NM))
    modes = ['kt', 'torque_ff_check'] if args.mode == 'all' else [args.mode]

    out = args.output_dir or os.path.join(
        _PROJECT_ROOT, 'temp', 'probes',
        f"kt_bench_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}")

    bench = KtBench(
        masses_kg=masses,
        traverse_lo_rev=_default(args.traverse_lo, DEFAULT_TRAVERSE_LO_REV),
        traverse_hi_rev=_default(args.traverse_hi, DEFAULT_TRAVERSE_HI_REV),
        traverse_vel_rps=_default(args.vel, kt.DEFAULT_TRAVERSE_VEL_RPS),
        reps=int(_default(args.reps, DEFAULT_REPS)),
        dwell_s=_default(args.dwell, DEFAULT_DWELL_S),
        edge_discard_s=_default(args.edge_discard, DEFAULT_EDGE_DISCARD_S),
        tilt_deg=args.tilt_deg,
        rig_orientation=args.rig_orientation,
        tff_amps_Nm=amps,
        tff_hold_rev=_default(args.tff_hold, DEFAULT_TFF_HOLD_REV),
        tff_hold_mass_kg=_default(args.hold_mass, DEFAULT_TFF_HOLD_MASS_KG),
        tff_half_period_s=_default(args.tff_half_period,
                                   kt.DEFAULT_TFF_HALF_PERIOD_S),
        tff_cycles=int(_default(args.tff_cycles, kt.DEFAULT_TFF_CYCLES)),
        pre_soak_s=_default(args.pre_soak, kt.DEFAULT_PRE_SOAK_S),
        assume_yes=args.yes,
        accel_time_s=_default(args.accel_time, kt.DEFAULT_TRAVERSE_ACCEL_TIME_S),
        # --- BridgeSysID kwargs ---
        axis_id=args.axis, stroke_cap_rev=args.stroke_cap,
        center_rev=_default(args.traverse_lo, DEFAULT_TRAVERSE_LO_REV),
        current_limit_a=args.current_limit,
        vel_cap_rps=args.vel_cap, output_dir=out, dry_run=args.dry_run,
        chirp_f0=1.0, chirp_f1=8.0, chirp_dur=4.0, chirp_amp=0.02,
        pos_steps=[], ladder_step=0.14, zeta_target=0.7, bw_clear_hz=None, n_vel=4,
        ripple_threshold=0.05, osc_threshold=0.5, iq_ripple_threshold=0.5,
        do_home=args.home, knot_hz=args.knot_hz, fast_iq_available=True,
    )
    return bench.run(modes)


if __name__ == '__main__':
    sys.exit(main())
