#!/usr/bin/env python3
"""Pure logic for the torque-constant (Kt) + torque_ff-channel bench harness.

This is the hardware-free half of ``kt_bench_test.py``: force balance, the
steady-velocity window selector, the constant-velocity knot generator, the
weighted fits, the Mode-2 EDGE-CAPTURE pipeline (settle gate, square-wave
schedule, matched-filter edge location, decay-corrected jump statistics), the
current budget, the mode/flag validator, and the sign inference. No sockets,
no CAN, no ``teensy_link`` import — so every decision the harness makes about
a number is unit-tested off-hardware in ``tests/motion/test_kt_lib.py``.

Why this module exists
======================
Jugglebot has a **13 % torque-constant discrepancy** that blocks all
feedforward work:

======================================  ==========  ====================================
source                                  Kt (Nm/A)   provenance
======================================  ==========  ====================================
``config/hardware_config.yaml:60``      0.0624      "measured, Phase-2 multi-weight bench
                                                    fit R²=0.994"; canonical historical
                                                    reference ``single_leg_test.py:92``
``ODrive config Files/                  0.055133    == 8.27/150 — ODrive's DEFAULT
odrive_pro_leg_config.json:152``                    8.27/Kv formula, i.e. never measured
datasheet SI                            0.0637      == 60/(2π·150 Kv)
======================================  ==========  ====================================

The ODrive divides commanded torque by ITS ``torque_constant`` to get Iq, so any
torque feedforward we ship lands **+13 % hot** until this is settled.

**The stiction hypothesis.** The historical 0.0624 is suspected to be biased HIGH.
If the Phase-2 fit held hanging weights **at rest**, Coulomb friction supports part
of the load, so less current is needed than gravity alone demands, so the inferred
τ/iq (= Kt) is inflated. τ_c ≈ 1.09 A on this rig — that is NOT small next to the
~2-6 A a test mass draws.

The method: friction cancellation by direction reversal
=======================================================
Drive the leg at **constant velocity** (no accel term ⇒ no inertia term), first
EXTENDING (lifting the mass), then RETRACTING (lowering it), at the **same speed
magnitude**. In an extension-positive torque frame, with τ_f the friction torque
(Coulomb + viscous), which **flips sign with velocity direction**, and τ_g = m·g·r
the gravity torque, which does NOT::

    UP   (extending, +v):  τ_m = τ_g + τ_f      ⇒  iq_up   = (τ_g + τ_f)/Kt
    DOWN (retracting, −v): τ_m = τ_g − τ_f      ⇒  iq_down = (τ_g − τ_f)/Kt

Therefore::

    (iq_up + iq_down)/2  =  τ_g / Kt     ← friction CANCELS EXACTLY.  The Kt signal.
    (iq_up − iq_down)/2  =  τ_f / Kt     ← a FREE, independent friction measurement.

Sweep ≥3 masses and fit **with an intercept**::

    iq_avg(m) = a·m + b,   a = g·r_spool/Kt   ⇒   Kt = g·r_spool / a

The intercept ``b`` absorbs the leg's own moving mass and any cable preload. A
forced-through-origin fit would fold that constant into the slope and bias Kt —
which is why :func:`fit_kt` mandates the intercept.

The friction half-difference is **load-bearing evidence, not a bonus**: it must be
≈ constant across masses and land near the independently-known τ_c ≈ 1.094 A
(``logbook/2026-04-27-friction-feedforward-bench-validation.md:79``, reproduced at
0.88–1.22 A by ``tools/probes/bench_leg_plant_id.py``). If it does, the picture is
coherent and Kt is trustworthy. If it does NOT, something is wrong with the run and
:func:`friction_consistency` says so loudly.

Sign conventions — the trap this module exists to not fall into
==============================================================
The can-bridge firmware reports **pos/vel and iq in OPPOSITE frames for legs**:

* ``can_buses.cpp:85-86`` — pos/vel ARE passed through ``leg_sign`` (negated), so
  telemetry ``pos_rev``/``vel_rps`` are in the Jugglebot **extension-positive** frame
  (the same frame the harness streams ``u0`` in).
* ``can_buses.cpp:92`` — ``iq_measured`` is stored **RAW**, with no ``leg_sign``. It
  stays in the **ODrive** frame.
* ``odrive_protocol.h:49`` — ``leg_sign(axis, v) = -v`` for legs.

Consequently ODrive +iq drives ODrive +position, which is Jugglebot **RETRACTION**.
The code-read prediction is therefore: **an EXTENDING torque reads as NEGATIVE
``iq_measured``** (:data:`IQ_EXTENSION_SIGN_PREDICTED` = −1).

We do **not** trust that prediction — we *measure* it, and cross-check. With the leg
vertical and extending UP against a hanging mass, the leg must produce an EXTENDING
torque to hold the mass, so **the sign of iq at a loaded hold IS the sign of
"extending torque" in the reported frame**. That is a self-calibrating reference
requiring no assumption about the wire (see :func:`infer_extension_iq_sign`). The
harness asserts the measured sign against the code-read prediction and shouts on a
mismatch — either would be a real finding.

An internal consistency check falls out for free: friction opposes motion and gravity
opposes extension, so BOTH the fitted slope ``a`` and the friction half-difference are
positive in the extension-positive torque frame — hence in the raw reported frame they
must carry the **same sign**. :func:`fit_kt` and :func:`friction_consistency` are
sign-agnostic (they work on magnitudes) and :func:`slope_friction_sign_agree` checks
the invariant.

Geometry / physics inputs (every one cited against source)
==========================================================
* **Bench leg**: 1 motor rev extends the leg **71.5708 mm** (measured;
  ``tests/hardware/single_leg_test.py:106`` ``TEST_LEG_MM_PER_REV``) ⇒
  ``r_spool = 71.5708/(2π) = 11.3907 mm``. τ_g = m·g·r_spool.
  The PLATFORM legs are ~70.5 mm/rev — **do NOT use that here**
  (``single_leg_test.py:100-102`` keeps them separate for exactly this reason).
* **The leg is VERTICAL and extends UPWARD, pushing the mass up** (operator-confirmed
  setup). Off-vertical the gravity torque is m·g·r·cos θ — see ``tilt_deg``.
* **Stiction knee** ω_s = 0.251 rev/s (logbook 2026-04-27:81). The constant velocity
  MUST sit above it so we ride the kinetic-friction plateau, not the Stribeck dip.
  ~0.5–0.8 rev/s recommended; same |v| both directions so the viscous term cancels too.
* **Bench hard limits**: stroke 3.0 rev, accel 250 rev/s², current 10 A
  (``cogging_bench_test.py``, mirrored in ``bench_leg_sysid.py:190-203``).
* **ODrive wire**: ``torque_ff`` is float32 Nm on the UDP Setpoint
  (``config/generated/udp_protocol.py:186``); the Teensy negates it via ``leg_sign``
  and scales by ``LEG_TOR_SCALE`` = 10000 into an int16
  (``odrive_protocol.h:171-179``; ``protocol_config.yaml`` ``leg_tor: 10000.0``)
  ⇒ 0.0001 Nm/LSB, wire range ±3.2767 Nm.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

# ===========================================================================
# Physical constants — each cited against its source
# ===========================================================================

G_M_S2 = 9.80665                    # standard gravity

# Bench leg (NOT a platform leg): 1 motor rev extends the leg 71.5708 mm.
# single_leg_test.py:106 TEST_LEG_MM_PER_REV (measured on this exact rig).
BENCH_LEG_MM_PER_REV = 71.5708
BENCH_SPOOL_RADIUS_MM = BENCH_LEG_MM_PER_REV / (2.0 * math.pi)   # 11.3907 mm
BENCH_SPOOL_RADIUS_M = BENCH_SPOOL_RADIUS_MM * 1e-3              # 0.0113907 m

# The three candidate torque constants this harness exists to discriminate.
KT_ODRIVE_CONFIGURED = 0.055133331567049026  # odrive_pro_leg_config.json:152 (== 8.27/150)
KT_HISTORICAL_MEASURED = 0.0624              # hardware_config.yaml:60 / single_leg_test.py:92
KT_DATASHEET_SI = 60.0 / (2.0 * math.pi * 150.0)  # 0.063662 — SI from the 150 Kv rating

KT_CANDIDATES: Dict[str, float] = {
    'odrive_configured': KT_ODRIVE_CONFIGURED,
    'historical_measured': KT_HISTORICAL_MEASURED,
    'datasheet_si': KT_DATASHEET_SI,
}

# Friction ground truth, for the cross-check (NOT an input to the Kt fit).
# logbook/2026-04-27-friction-feedforward-bench-validation.md:79,81,215-217.
# Independently reproduced at 0.88–1.22 A by tools/probes/bench_leg_plant_id.py.
TAU_C_REF_A = 1.094           # kinetic Coulomb floor, expressed as motor current
TAU_C_REF_RANGE_A = (0.88, 1.22)
OMEGA_S_REV_S = 0.251         # Stribeck breakaway velocity scale — the stiction knee
VISCOUS_B_A_PER_REV_S = 0.0173  # viscous coefficient, in A per rev/s

# Velocity recommendation: comfortably above the stiction knee (≈2.4× ω_s), and
# low enough that back-EMF and regen are non-issues on the 21 V bench PSU
# (see recommended_velocity_notes()).
DEFAULT_TRAVERSE_VEL_RPS = 0.6
MIN_SAFE_TRAVERSE_VEL_RPS = 2.0 * OMEGA_S_REV_S   # 0.502 — below this we are in the dip

# Bench hard caps (bench_leg_sysid.py:190-203, from cogging_bench_test.py).
BENCH_CURRENT_LIMIT_A = 10.0
BENCH_STROKE_CAP_REV = 3.0

# ODrive protection independent of torque_constant (operator-confirmed fact 3):
# odrive_pro_leg_config.json:53 torque_soft_max, :129 current_soft_max.
ODRIVE_TORQUE_SOFT_MAX_NM = 1.5
ODRIVE_CURRENT_SOFT_MAX_A = 10.0

# Wire scaling for torque_ff (protocol_config.yaml input_scales.leg_tor).
LEG_TOR_WIRE_SCALE = 10000.0                       # int16 counts per Nm
LEG_TOR_WIRE_RESOLUTION_NM = 1.0 / LEG_TOR_WIRE_SCALE   # 0.0001 Nm/LSB
LEG_TOR_WIRE_MAX_NM = 32767.0 / LEG_TOR_WIRE_SCALE      # ±3.2767 Nm

# Code-read prediction of the reported-iq sign for an EXTENDING torque.
# can_buses.cpp:85-86 negate pos/vel (→ extension-positive) but :92 stores
# iq_measured RAW (ODrive frame); odrive_protocol.h:49 leg_sign = −v for legs.
# ⇒ ODrive +iq == +ODrive-position == Jugglebot RETRACTION ⇒ extension reads NEGATIVE.
# MEASURED and asserted against, never assumed — see infer_extension_iq_sign().
IQ_EXTENSION_SIGN_PREDICTED = -1

# Mode-2 torque_ff ladder safety cap. 0.15 Nm ≈ 2.7 A through the ODrive's configured
# torque_constant — 10× below torque_soft_max (1.5 Nm) and far below the point where it
# could overcome the position loop holding a loaded leg.
TORQUE_FF_HARD_CAP_NM = 0.15

# The STATIC-FRICTION BAND the Mode-2 premise lives inside. "The position loop holds
# station, so the iq shift IS the measurement" is only true while the injected torque_ff
# stays under the static friction torque: |tff| < τ_c·Kt. Outside the band the leg
# actually MOVES, the position loop's integrator re-absorbs part of the feedforward at
# the new equilibrium, and the fitted slope reads LOW — band-saturation model:
# 33–52 % for a 0.10 Nm rung; observed 63–68 % low on the
# 2026-07-14 run: the ±0.10 Nm rungs saturated at the band edge / were re-absorbed,
# while the two in-band rungs showed a clean −20 to −22.5 A/Nm slope (right order vs the expected 18.14, ~17 % high)). The worst-case (lowest)
# band edge is τ_c_lo × Kt_lo = 0.88 A × 0.055133 = 0.0485 Nm; the band spans
# ≈ 0.049–0.067 Nm across the τ_c/Kt candidate ranges.
TFF_STATIC_BAND_MIN_NM = TAU_C_REF_RANGE_A[0] * KT_ODRIVE_CONFIGURED   # ≈ 0.0485
TFF_STATIC_BAND_MAX_NM = TAU_C_REF_RANGE_A[1] * KT_ODRIVE_CONFIGURED   # ≈ 0.0673
TFF_BAND_WARN_NM = 0.045          # warn above this — within 8 % of the worst-case edge

# Default ladder: max rung 0.035 Nm, well under the 0.049 Nm worst-case band edge, with
# 0.0 kept as the reference rung. Smallest rung 0.010 Nm still shifts iq by
# 0.010/0.055133 ≈ 0.18 A — comfortably measurable at 250 Hz.
DEFAULT_TORQUE_FF_LADDER_NM = (-0.035, -0.020, -0.010, 0.0, 0.010, 0.020, 0.035)

# Mode-2 verdict quality gate: below these the fit is noise and NO sign/scale verdict
# may be printed (the 2026-07-14 run produced R² = −0.099 and a 1.8σ slope, yet the
# then-current harness shouted *** MUST NEGATE *** / *** MISMATCH −63% *** from it).
TFF_GATE_MIN_R2 = 0.90
TFF_GATE_MIN_T_STAT = 3.0         # |slope| / slope_sigma
TFF_OUTLIER_RMS_MULT = 3.0        # |residual| > 3× the leave-one-out RMS ⇒ flagged

# ---------------------------------------------------------------------------
# Mode-2 EDGE-CAPTURE (2026-07-15 redesign — the rung ladder is dead)
# ---------------------------------------------------------------------------
# WHY: at a SETTLED static hold the velocity-loop integrator forces pos == cmd, and
# torque balance pins the TOTAL iq at the load value — so the steady-state iq response
# to a CONSTANT torque_ff is exactly ZERO. What the 2026-07-14 rung ladders measured
# was the TRANSIENT sampled at ~1.3 s of a τ ≈ 2–10 s locked-state re-absorption decay:
# structurally confounded (the 20:55 run's −21 A/Nm was drift-aliasing — at a fixed
# rung cadence, wall-clock time and tff are collinear, so any slow drift maps straight
# onto the fitted slope). The replacement measures the INSTANTANEOUS iq JUMP at each
# edge of a ± square wave, before the integrator can re-absorb it; the alternating
# polarity de-aliases drift (a drift contributes the SAME signed bias to consecutive
# opposite-polarity edges, so it cancels in the slope and is separately detected by
# the paired-edge drift statistic).
DEFAULT_TFF_EDGE_AMPS_NM = (0.010, 0.020, 0.035)  # all inside the 0.045 Nm static band
DEFAULT_TFF_HALF_PERIOD_S = 1.75   # square-wave half period (1.5–2 s sensible range)
TFF_HALF_PERIOD_RANGE_S = (1.5, 2.0)
DEFAULT_TFF_CYCLES = 10            # >= 10 full ± cycles per amplitude
MIN_TFF_CYCLES = 2                 # below this there are too few edges to even try
EDGE_SEARCH_S = 0.10               # matched filter hunts within ±this of the commanded toggle
EDGE_KERNEL_S = 0.10               # step-kernel half-width for the matched filter
EDGE_WINDOW_LO_S = 0.02            # jump windows: [t_e−0.20, t_e−0.02] vs [t_e+0.02, t_e+0.20]
EDGE_WINDOW_HI_S = 0.20
DEFAULT_HOLD_TAU_S = 8.0           # per-hold re-absorption τ when unfittable
MIN_HOLD_TAU_S = 1.0               # clamp for the decay correction (τ below this ⇒ method invalid anyway)
EDGE_TRIM_FRAC = 0.10              # symmetric trim fraction on the per-edge slopes
TFF_EDGE_CV_MAX = 0.20             # trimmed edge-jump CV above this ⇒ no verdict
EDGE_DRIFT_BIAS_MIN_A = 0.03       # paired-edge drift bias must exceed this AND 3σ to fire
MIN_EDGES_FOR_VERDICT = 6          # fewer kept edges than this ⇒ no verdict

# Settle gate before ANY edge-capture measurement (tonight's holds needed ~10 s
# post-approach before d(iq)/dt fell to the noise floor).
SETTLE_POS_TOL_REV = 3.0e-4        # pos within ±0.3 mrev of cmd
SETTLE_DIQ_DT_MAX_A_PER_S = 0.05   # |d(iq)/dt| over the settle window
SETTLE_WINDOW_S = 3.0
SETTLE_MAX_WAIT_S = 30.0
# Friction-band load-transfer transient at the start of any static hold: ~1.2–1.4 A
# (≈ τ_c amplitude) with τ ≈ 3.2 s — dead in 15 s. Soak it out before measuring.
DEFAULT_PRE_SOAK_S = 15.0

# Rig orientation — REQUIRED operator input (no default). The 2026-07-14 manifests
# recorded extension_iq_sign = −1 the wrong way round because the rig was INVERTED
# (contraction raised the load) while the harness assumed holding == extending.
#   normal   — leg EXTENSION raises the load (mass hangs from the extending end)
#   inverted — leg CONTRACTION raises the load
RIG_ORIENTATIONS = ('normal', 'inverted')

# Motor rotor inertia (hardware_config.yaml:55, ODrive D6374-150Kv datasheet) — used to
# budget the ACCELERATION-phase current of a shaped traverse, not by any fit.
J_ROTOR_KGM2 = 2.75e-4

# Traverse start/end shaping: ramp 0 → v over this long (trapezoidal velocity profile).
# An UNSHAPED constant-velocity series steps 0 → 0.6 rev/s in ONE knot (~60 rev/s² accel
# demand through the firmware Hermite ≈ +4.5 A at 3 kg — enough to brush the over-current
# abort). At 0.4 s the ramp accel is 0.6/0.4 = 1.5 rev/s² ⇒ ~0.11 A at 3 kg: negligible.
DEFAULT_TRAVERSE_ACCEL_TIME_S = 0.4

# Live over-current abort debounce: require this many CONSECUTIVE over-limit samples
# (~12 ms at 250 Hz) so a single telemetry glitch / one-sample spike cannot abort a run.
OVERCURRENT_TRIP_SAMPLES = 3

# The bench-vs-platform sign context every Mode-2 sign verdict must carry.
# ODrive direction calibration is PER-DRIVE, so the BENCH drive's torque sign convention
# is OPPOSITE to the platform legs'. Empirical record: the 2026-04-27 friction-FF bench
# work needed ``--ff-sign -1`` on this rig, while the 2026-05-08 PLATFORM validation ran
# the standard un-negated chain and worked (a wrong sign would have DOUBLED friction, so
# that A/B genuinely discriminates). A "positive wire torque_ff RETRACTS" result on THIS
# BENCH is therefore EXPECTED and does NOT mean the production gravity feedforward must
# be negated — the platform-validated sign stands.
BENCH_VS_PLATFORM_SIGN_NOTE = (
    "ODrive direction calibration is PER-DRIVE: the bench drive's torque sign "
    "convention is OPPOSITE to the platform legs' (2026-04-27 friction bench needed "
    "--ff-sign -1 on this rig; the 2026-05-08 PLATFORM validation ran the standard "
    "un-negated chain and worked — a wrong sign would have doubled friction, so that "
    "A/B discriminates). A bench sign result does NOT transfer to the platform; the "
    "platform-validated production sign stands.")


# ===========================================================================
# Force balance
# ===========================================================================


def gravity_torque_Nm(mass_kg: float, *, tilt_deg: float = 0.0,
                      spool_radius_m: float = BENCH_SPOOL_RADIUS_M) -> float:
    """Motor torque needed to support ``mass_kg`` against gravity: ``m·g·r·cos θ``.

    ``tilt_deg`` is the leg's departure from vertical. The operator's setup is
    VERTICAL (θ=0, cos θ=1) and should stay that way; the correction exists so a
    known small misalignment can be compensated rather than silently biasing Kt.
    A 10° tilt costs 1.5 % — comparable to the effect we are hunting, hence the knob.
    """
    return (float(mass_kg) * G_M_S2 * float(spool_radius_m)
            * math.cos(math.radians(float(tilt_deg))))


def iq_for_torque(torque_Nm: float, kt_nm_per_a: float) -> float:
    """Motor current to deliver ``torque_Nm`` given ``kt``. Iq = τ / Kt."""
    if kt_nm_per_a <= 0.0:
        raise ValueError("kt must be positive")
    return float(torque_Nm) / float(kt_nm_per_a)


def friction_current_A(vel_rps: float, *, tau_c_A: float = TAU_C_REF_A,
                       viscous_b: float = VISCOUS_B_A_PER_REV_S) -> float:
    """Friction expressed as motor current at speed ``vel_rps`` (signed with velocity).

    ``τ_c·sgn(v) + b·v`` in the current domain — the same form
    ``tools/probes/bench_leg_plant_id.py:129`` regresses. At the recommended 0.6 rev/s
    the viscous term is 0.0173·0.6 ≈ 0.01 A, i.e. ~1 % of τ_c: negligible, and it
    cancels in the up/down average anyway (it too flips sign with velocity).
    """
    v = float(vel_rps)
    if v == 0.0:
        return 0.0
    return math.copysign(tau_c_A, v) + viscous_b * v


def predicted_iq_up_down(mass_kg: float, kt_nm_per_a: float, *,
                         vel_rps: float = DEFAULT_TRAVERSE_VEL_RPS,
                         tilt_deg: float = 0.0,
                         tau_c_A: float = TAU_C_REF_A) -> Tuple[float, float]:
    """``(iq_up, iq_down)`` predicted in the **extension-positive** current frame.

    ``iq_up = (τ_g + τ_f)/Kt`` (extending, lifting), ``iq_down = (τ_g − τ_f)/Kt``
    (retracting, lowering). Used for the current budget and the a-priori
    distinguishing-power check — NOT by the fit, which is purely empirical.

    Note ``iq_down`` goes NEGATIVE for small masses: once τ_g < τ_f the motor must
    actively PUSH the mass down against friction rather than resist it. That is
    physically fine but it means τ_g is no longer ≫ τ_c, so the fit is poorly
    conditioned there — :func:`current_budget` flags it.
    """
    tau_g = gravity_torque_Nm(mass_kg, tilt_deg=tilt_deg)
    tau_f_A = friction_current_A(abs(vel_rps), tau_c_A=tau_c_A)
    iq_g = iq_for_torque(tau_g, kt_nm_per_a)
    return iq_g + tau_f_A, iq_g - tau_f_A


# ===========================================================================
# Mass selection / current budget
# ===========================================================================
#
# Three competing constraints:
#   1. iq_up must stay under the 10 A limit WITH MARGIN. iq_up is largest under the
#      SMALLEST candidate Kt (0.0551), so the budget is computed against that —
#      the conservative choice, since we do not yet know Kt (that is the point).
#   2. τ_g must meaningfully dominate τ_c. Below ~0.55 kg iq_down actually goes
#      NEGATIVE (the motor pushes the mass down against friction) — a different
#      physical regime, and the one place the up/down symmetry could plausibly break.
#   3. ≥3 masses spanning a WIDE range, so the slope has leverage.
#
# Constraints 2 and 3 pull against each other: raising the conditioning floor pushes
# the lightest mass up, and since the 10 A limit pins the heaviest mass at ~3.4 kg,
# that SQUEEZES the span and starves the slope of leverage. Resolving the tension needs
# one observation about what the conditioning floor is actually protecting against.
#
# The friction cancellation is not perfect in practice: the up and down traverses run at
# very slightly different speeds, so a small residual τ_f term survives the average. But
# that residual is essentially MASS-INDEPENDENT (Coulomb friction does not care what is
# hanging on the leg — that is the same fact the friction cross-check verifies). A
# mass-independent error lands in the INTERCEPT, which is fitted out, and NOT in the
# slope, which is the only thing Kt is derived from.
#
# So the conditioning floor is NOT protecting the slope from friction — the intercept
# already does that. What it protects against is the qualitatively different regime near
# iq_down = 0, where the motor stops resisting the mass and starts driving it down, and
# the clean "τ_f flips sign, τ_g does not" story is at its most fragile. That regime
# starts at τ_g/τ_c ≈ 1. A floor of 1.5 keeps a 50 % margin clear of it, which is all
# that is needed — and buys back the span the slope actually wants.
CONDITIONING_HARD_FLOOR = 1.5      # τ_g/τ_c below this ⇒ too close to the iq_down<0 regime
CONDITIONING_TARGET = 1.8          # what the recommended ladder's lightest mass clears
MASS_QUANTUM_KG = 0.5              # real bench weights come in 0.5 kg steps


@dataclass
class MassBudgetRow:
    mass_kg: float
    tau_g_Nm: float
    iq_up_A: float          # worst case (smallest Kt candidate → most current)
    iq_down_A: float
    headroom_A: float       # current_limit − iq_peak (accel phase included)
    tau_g_over_tau_c: float  # conditioning: want ≫ 1
    ok: bool
    warnings: List[str] = field(default_factory=list)
    iq_accel_A: float = 0.0  # extra current during the shaped accel ramp
    iq_peak_A: float = 0.0   # iq_up + iq_accel — the number the abort actually sees


@dataclass
class MassBudget:
    rows: List[MassBudgetRow]
    current_limit_A: float
    worst_case_kt: float
    ok: bool
    reasons: List[str] = field(default_factory=list)


def current_budget(masses_kg: Sequence[float], *,
                   current_limit_A: float = BENCH_CURRENT_LIMIT_A,
                   headroom_frac: float = 0.2,
                   vel_rps: float = DEFAULT_TRAVERSE_VEL_RPS,
                   tilt_deg: float = 0.0,
                   tau_c_A: float = TAU_C_REF_A,
                   min_conditioning: float = CONDITIONING_HARD_FLOOR,
                   accel_rps2: float = DEFAULT_TRAVERSE_VEL_RPS
                   / DEFAULT_TRAVERSE_ACCEL_TIME_S) -> MassBudget:
    """Current budget + conditioning check for a proposed mass set.

    Evaluated at the **smallest** candidate Kt (``KT_ODRIVE_CONFIGURED`` = 0.0551),
    because a smaller Kt means MORE current for the same torque — so this is the
    worst case and the budget cannot be surprised by the answer we are about to
    measure.

    The budget covers the WHOLE traverse, accel phase included: ``iq_peak = iq_up +``
    :func:`accel_current_A` at ``accel_rps2`` (the shaped ramp's ``v/accel_time``;
    default 1.5 rev/s² ⇒ ~0.11 A at 3 kg). Pass the actual ramp accel so a shortened
    ``--accel-time`` is caught here rather than by the over-current abort mid-run.

    A mass fails ONLY on (a) predicted ``iq_peak`` exceeding
    ``(1 − headroom_frac)·current_limit`` — 8 A at the 10 A limit and the default
    20 % headroom.  ``τ_g/τ_c`` below ``min_conditioning`` is ADVISORY (a warning
    in ``reasons``, never a refusal) — downgraded 2026-07-15 after the 0.50 kg
    point of the 2026-07-14 session empirically refuted the hard floor (it sat
    dead on the R² = 0.99909 fit; the up/down average cancels friction, so a
    near-zero one-way traverse is expected physics, not fit ill-conditioning.
    """
    kt_worst = min(KT_CANDIDATES.values())
    usable = (1.0 - headroom_frac) * current_limit_A
    tau_c_Nm = tau_c_A * kt_worst
    rows: List[MassBudgetRow] = []
    reasons: List[str] = []
    for m in masses_kg:
        iq_up, iq_down = predicted_iq_up_down(
            m, kt_worst, vel_rps=vel_rps, tilt_deg=tilt_deg, tau_c_A=tau_c_A)
        iq_accel = accel_current_A(m, accel_rps2, kt_nm_per_a=kt_worst)
        iq_peak = iq_up + iq_accel
        tau_g = gravity_torque_Nm(m, tilt_deg=tilt_deg)
        cond = (tau_g / tau_c_Nm) if tau_c_Nm > 0 else math.inf
        w: List[str] = []
        ok = True
        if iq_up > usable:
            ok = False
            w.append(f"iq_up {iq_up:.2f} A exceeds the usable budget {usable:.2f} A "
                     f"({headroom_frac * 100:.0f}% headroom under the "
                     f"{current_limit_A:.0f} A limit)")
        elif iq_peak > usable:
            ok = False
            w.append(f"the ACCEL phase pushes the peak to {iq_peak:.2f} A "
                     f"(+{iq_accel:.2f} A at {accel_rps2:.1f} rev/s²) over the usable "
                     f"budget {usable:.2f} A — lengthen --accel-time or drop the mass")
        if cond < min_conditioning:
            # WARN, don't refuse (downgraded 2026-07-15): the 2026-07-14 four-mass
            # session EMPIRICALLY refuted the hard floor — its 0.50 kg point
            # (tau_g/tau_c = 0.93) had traverse SEMs of 0.08-0.11 A and sat dead on
            # the R^2 = 0.99909 mass fit.  The near-zero traverse at light load is
            # expected PHYSICS (gravity ~ friction), not ill-conditioning of the
            # FIT, which consumes the up/down AVERAGE where friction has already
            # cancelled.  A light point also anchors the intercept.  Over-current
            # refusals above remain hard; conditioning is advisory.
            w.append(f"tau_g/tau_c = {cond:.2f} < {min_conditioning:.1f} — gravity does "
                     f"not dominate friction at this mass (near-zero one-way traverse "
                     f"expected; fine for the fit — the up/down average cancels "
                     f"friction — but expect one |iq| near 0)")
        if iq_down < 0.0:
            w.append(f"iq_down {iq_down:.2f} A is NEGATIVE — the motor must push the "
                     f"mass DOWN against friction (physical, but a light load)")
        rows.append(MassBudgetRow(
            mass_kg=float(m), tau_g_Nm=tau_g, iq_up_A=iq_up, iq_down_A=iq_down,
            headroom_A=current_limit_A - iq_peak, tau_g_over_tau_c=cond,
            ok=ok, warnings=w, iq_accel_A=iq_accel, iq_peak_A=iq_peak))
        reasons.extend(f"m={m:.2f} kg: {x}" for x in w)

    if len(list(masses_kg)) < 3:
        reasons.append("fewer than 3 masses — the intercept + slope fit needs >= 3 "
                       "points to have any residual degrees of freedom")
    ok = all(r.ok for r in rows) and len(list(masses_kg)) >= 3
    return MassBudget(rows=rows, current_limit_A=current_limit_A,
                      worst_case_kt=kt_worst, ok=ok, reasons=reasons)


def recommended_masses_kg(*, current_limit_A: float = BENCH_CURRENT_LIMIT_A,
                          headroom_frac: float = 0.2,
                          vel_rps: float = DEFAULT_TRAVERSE_VEL_RPS,
                          tau_c_A: float = TAU_C_REF_A,
                          n: int = 5,
                          min_conditioning: float = CONDITIONING_TARGET) -> List[float]:
    """A well-conditioned, budget-safe mass ladder: ``n`` masses evenly spanning
    ``[m_lo, m_hi]``, rounded to a 0.5 kg quantum (real bench weights).

    ``m_lo`` is the smallest mass whose ``τ_g/τ_c`` clears ``min_conditioning``;
    ``m_hi`` the largest whose worst-case ``iq_up`` fits the headroom budget. Both are
    computed at the WORST-CASE (smallest) candidate Kt, so the ladder is safe and
    well-conditioned whatever the answer turns out to be.

    On this rig that yields **1.0, 1.5, 2.0, 2.5, 3.0 kg** — a 3× span, which is where
    the slope's leverage comes from. See the block comment above for why the
    conditioning floor is deliberately modest (1.8, not 3): a higher floor would push
    ``m_lo`` up against the current-limited ``m_hi`` and starve the fit of exactly the
    span it needs, while protecting against an error that the intercept already absorbs.
    """
    kt_worst = min(KT_CANDIDATES.values())
    tau_c_Nm = tau_c_A * kt_worst
    g_r = G_M_S2 * BENCH_SPOOL_RADIUS_M          # Nm per kg
    # Conditioning floor: m·g·r >= min_conditioning · tau_c
    m_lo = min_conditioning * tau_c_Nm / g_r
    # Budget ceiling: (m·g·r)/kt + tau_c_A <= (1 − headroom)·limit
    usable = (1.0 - headroom_frac) * current_limit_A
    tau_f_A = friction_current_A(abs(vel_rps), tau_c_A=tau_c_A)
    m_hi = (usable - tau_f_A) * kt_worst / g_r
    if m_hi <= m_lo:
        return []

    # Lay the ladder out ON the 0.5 kg grid rather than rounding a linspace onto it:
    # rounding collides (two linspace points can land on the same weight, silently
    # shrinking the ladder) and can round a point OUTSIDE the safe bounds it was
    # sampled inside. Walking the grid gives evenly-spaced, physically-real weights
    # that are in-bounds by construction.
    q = MASS_QUANTUM_KG
    lo_q = math.ceil(m_lo / q - 1e-9) * q
    hi_q = math.floor(m_hi / q + 1e-9) * q
    if hi_q < lo_q:
        return []
    grid = [round(lo_q + i * q, 3)
            for i in range(int(round((hi_q - lo_q) / q)) + 1)]
    if len(grid) <= n:
        return grid
    # Too many rungs — subsample evenly, always keeping both endpoints (the span is
    # where the slope's leverage lives).
    idx = np.linspace(0, len(grid) - 1, max(3, int(n)))
    return sorted({grid[int(round(i))] for i in idx})


def recommended_velocity_notes(vel_rps: float = DEFAULT_TRAVERSE_VEL_RPS,
                               *, mass_kg: float = 3.0,
                               kv: float = 150.0,
                               bus_v: float = 21.0) -> Dict[str, float]:
    """Back-EMF / regen sanity numbers for a traverse velocity, so the PSU choice is
    made on arithmetic rather than folklore.

    * ``back_emf_V`` = (rev/s · 60) / Kv — at 0.6 rev/s and 150 Kv this is **0.24 V**,
      i.e. ~1 % of a 21 V bus. The 21 V PSU is "back-EMF marginal" only near the leg's
      top speed (≈47 rev/s ⇒ 18.8 V); at a system-ID crawl it is nowhere near binding.
    * ``regen_W`` = m·g·v_linear — the power the descending mass pushes back into the
      bus. At 3 kg and 0.6 rev/s (= 42.9 mm/s) that is **1.26 W**. A brake resistor
      cares about hundreds of watts; this is negligible.

    Conclusion the harness prints: at ≤1 rev/s the 21 V bench PSU is fine for BOTH
    modes. The 48 V + brake-resistor advice applies to fast-motion work, not here.
    """
    v_lin_m_s = abs(vel_rps) * BENCH_LEG_MM_PER_REV * 1e-3
    return {
        'vel_rps': float(vel_rps),
        'vel_linear_mm_s': abs(vel_rps) * BENCH_LEG_MM_PER_REV,
        'back_emf_V': abs(vel_rps) * 60.0 / kv,
        'back_emf_frac_of_bus': (abs(vel_rps) * 60.0 / kv) / bus_v,
        'regen_W': mass_kg * G_M_S2 * v_lin_m_s,
        'omega_s_rev_s': OMEGA_S_REV_S,
        'vel_over_omega_s': abs(vel_rps) / OMEGA_S_REV_S,
    }


# ===========================================================================
# A-priori distinguishing power
# ===========================================================================


def slope_for_kt(kt_nm_per_a: float, *, tilt_deg: float = 0.0) -> float:
    """The fit slope ``a = g·r·cos θ / Kt`` (A per kg) a given Kt implies."""
    return (G_M_S2 * BENCH_SPOOL_RADIUS_M * math.cos(math.radians(tilt_deg))
            / float(kt_nm_per_a))


def slope_standard_error(masses_kg: Sequence[float], iq_point_sigma_A: float
                         ) -> float:
    """A-priori SE of the fitted slope: ``σ_pt / sqrt(Σ(m − m̄)²)``.

    The ordinary-least-squares slope variance for equal-weight points. ``iq_point_sigma_A``
    is the standard error of ONE mass's ``iq_avg`` — i.e. the uncertainty AFTER averaging
    the up and down traverse means, not the per-sample iq noise.
    """
    m = np.asarray(masses_kg, float)
    if m.size < 2:
        return math.inf
    sxx = float(np.sum((m - m.mean()) ** 2))
    if sxx <= 0.0:
        return math.inf
    return float(iq_point_sigma_A) / math.sqrt(sxx)


@dataclass
class DistinguishingPower:
    masses_kg: List[float]
    iq_point_sigma_A: float
    slope_se_A_per_kg: float
    pairs: Dict[str, float]      # "a_vs_b" -> separation in sigmas
    ok: bool                     # every candidate pair separated by >= min_sigma


def distinguishing_power(masses_kg: Sequence[float], iq_point_sigma_A: float, *,
                         candidates: Optional[Dict[str, float]] = None,
                         min_sigma: float = 3.0,
                         tilt_deg: float = 0.0) -> DistinguishingPower:
    """Can this mass set + noise level actually TELL THE CANDIDATES APART?

    Compares the slope each candidate Kt implies against the a-priori slope SE. If the
    closest pair is not separated by at least ``min_sigma``, the experiment cannot
    resolve the question and the harness must say so BEFORE the operator hangs weights.

    The hardest pair is always ``historical_measured`` (0.0624) vs ``datasheet_si``
    (0.0637) — only 2 % apart. Distinguishing 0.0551 from 0.0624 (the 13 % question
    that actually blocks feedforward) is far easier, so the harness reports the pairs
    separately rather than gating on the hardest one.
    """
    cands = candidates if candidates is not None else KT_CANDIDATES
    se = slope_standard_error(masses_kg, iq_point_sigma_A)
    pairs: Dict[str, float] = {}
    names = list(cands)
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            a_i = slope_for_kt(cands[names[i]], tilt_deg=tilt_deg)
            a_j = slope_for_kt(cands[names[j]], tilt_deg=tilt_deg)
            sep = abs(a_i - a_j) / se if se > 0 else 0.0
            pairs[f"{names[i]}_vs_{names[j]}"] = float(sep)
    # Gate on the pair that actually blocks feedforward, not the 2%-apart pair.
    key = 'odrive_configured_vs_historical_measured'
    ok = pairs.get(key, 0.0) >= min_sigma
    return DistinguishingPower(
        masses_kg=[float(x) for x in masses_kg],
        iq_point_sigma_A=float(iq_point_sigma_A),
        slope_se_A_per_kg=float(se), pairs=pairs, ok=ok)


# ===========================================================================
# Constant-velocity knot generation (Path BRIDGE)
# ===========================================================================
#
# ``sysid_lib.knot_step_ramp`` ramps at *at most* ``frame_step_rev`` per knot — it caps
# a step, it does not TARGET a velocity, and its achieved velocity is
# ``delta/(ceil(delta/frame_step)·seg_t)``, i.e. somewhat BELOW the nominal. For Kt we
# need the two traverses to run at the same |v| to a tight tolerance (the viscous term
# only cancels if they do), so we size the frame count to hit the velocity exactly and
# then verify it against telemetry anyway.


def constant_velocity_knots(start_rev: float, end_rev: float, *,
                            vel_rps: float, seg_t_s: float,
                            lead_in_frames: int = 0,
                            lead_out_frames: int = 0) -> np.ndarray:
    """Knot ``u0`` series traversing ``start → end`` at a constant ``|vel_rps|``.

    The frame count is ``round(|Δ| / (v·seg_t))`` (at least 1), so the achieved velocity
    is ``|Δ|/(n·seg_t)`` — within half a frame-step of the request, rather than the
    up-to-a-full-frame-step *under* it that a ceil-based ramp gives. Both traverses of a
    pair therefore run at the same |v| to well under a percent, which is what makes the
    viscous term cancel in the up/down average.

    ``lead_in_frames`` / ``lead_out_frames`` hold the endpoints flat, giving the firmware
    500 Hz Hermite a stationary command to settle on before and after the move — the
    steady-state window selector then discards the acceleration transients anyway, but a
    flat lead-in means the leg starts from rest at a known place rather than from
    whatever the previous traverse left behind.

    The per-knot increment is ``v·seg_t``; at the recommended 0.6 rev/s and the bench
    build's 100 Hz knots (``SEGMENT_T_S`` 0.010) that is 0.006 rev — 17× under the
    0.10 rev ``MAX_LEAD_REV`` lead clamp, so the clamp can never engage on a traverse.
    """
    if seg_t_s <= 0.0:
        raise ValueError("seg_t_s must be > 0")
    if vel_rps <= 0.0:
        raise ValueError("vel_rps must be > 0 (direction comes from start vs end)")
    if lead_in_frames < 0 or lead_out_frames < 0:
        raise ValueError("lead frames must be >= 0")
    delta = float(end_rev) - float(start_rev)
    n = max(1, int(round(abs(delta) / (vel_rps * seg_t_s))))
    seq: List[float] = [float(start_rev)] * (int(lead_in_frames) + 1)
    for i in range(1, n + 1):
        seq.append(float(start_rev) + delta * (i / n))
    seq.extend([float(end_rev)] * int(lead_out_frames))
    return np.asarray(seq, float)


def shaped_constant_velocity_knots(start_rev: float, end_rev: float, *,
                                   vel_rps: float, seg_t_s: float,
                                   accel_time_s: float = DEFAULT_TRAVERSE_ACCEL_TIME_S,
                                   lead_in_frames: int = 0,
                                   lead_out_frames: int = 0) -> np.ndarray:
    """Knot series ``start → end`` cruising at ``|vel_rps|`` with SHAPED ends.

    Trapezoidal velocity profile: ramp 0 → v over ``accel_time_s``, cruise, ramp v → 0
    over ``accel_time_s``. :func:`constant_velocity_knots` steps 0 → v in ONE knot — at
    0.6 rev/s and 100 Hz knots that is a ~60 rev/s² acceleration demand through the
    firmware's 500 Hz Hermite, ≈ +4.5 A of inertia current at 3 kg (see
    :func:`accel_current_A`): enough to brush the harness over-current abort. The shaped
    ramp caps the accel at ``v/accel_time_s`` (1.5 rev/s² at the defaults ⇒ ~0.11 A at
    3 kg — negligible).

    A move too short to reach cruise (``|Δ| < v·accel_time_s``) degrades to a triangular
    profile at the SAME ramp accel, peaking below ``vel_rps`` — it never overshoots the
    requested velocity. The steady-state window selector discards the ramps either way
    (they fail both the at-speed and the not-accelerating gates), so the CRUISE portion
    — the only part the fit uses — is identical to the unshaped series' interior.

    Lead-in/out flats and endpoint conventions match :func:`constant_velocity_knots`.
    """
    if seg_t_s <= 0.0:
        raise ValueError("seg_t_s must be > 0")
    if vel_rps <= 0.0:
        raise ValueError("vel_rps must be > 0 (direction comes from start vs end)")
    if accel_time_s <= 0.0:
        raise ValueError("accel_time_s must be > 0")
    if lead_in_frames < 0 or lead_out_frames < 0:
        raise ValueError("lead frames must be >= 0")
    delta = float(end_rev) - float(start_rev)
    d = abs(delta)
    seq: List[float] = [float(start_rev)] * (int(lead_in_frames) + 1)
    if d > 0.0:
        a = float(vel_rps) / float(accel_time_s)
        if d >= vel_rps * accel_time_s:
            t_a = float(accel_time_s)
            v_pk = float(vel_rps)
            t_c = (d - v_pk * t_a) / v_pk
        else:
            v_pk = math.sqrt(d * a)          # triangular: same accel, lower peak vel
            t_a = v_pk / a
            t_c = 0.0
        T = 2.0 * t_a + t_c
        d_ramp = 0.5 * a * t_a * t_a

        def _s(t: float) -> float:
            if t <= 0.0:
                return 0.0
            if t < t_a:
                return 0.5 * a * t * t
            if t < t_a + t_c:
                return d_ramp + v_pk * (t - t_a)
            if t < T:
                tr = T - t
                return d - 0.5 * a * tr * tr
            return d

        sgn = 1.0 if delta > 0 else -1.0
        n = max(1, int(math.ceil(T / seg_t_s - 1e-9)))
        for i in range(1, n + 1):
            seq.append(float(start_rev) + sgn * _s(i * seg_t_s))
        seq[-1] = float(end_rev)             # land exactly on the endpoint
    seq.extend([float(end_rev)] * int(lead_out_frames))
    return np.asarray(seq, float)


def series_peak_accel_rps2(series: Sequence[float], seg_t_s: float) -> float:
    """Peak commanded acceleration (rev/s²) implied by a knot series — the max
    second difference over ``seg_t²``. The series-level check that a traverse's
    accel demand is what the shaping promised (mirrors
    ``sysid_lib.series_peak_velocity``)."""
    arr = np.asarray(series, float)
    if arr.size < 3 or seg_t_s <= 0.0:
        return 0.0
    return float(np.max(np.abs(np.diff(arr, n=2)))) / (float(seg_t_s) ** 2)


def accel_current_A(mass_kg: float, accel_rps2: float, *, kt_nm_per_a: float,
                    rotor_j_kgm2: float = J_ROTOR_KGM2) -> float:
    """Extra motor current the ACCELERATION phase of a traverse demands.

    Two inertia terms: the hanging mass (``m · a_lin · r_spool``, with
    ``a_lin = a·mm_per_rev``) and the rotor's own ``J·2π·a``
    (``hardware_config.yaml:55``). At the shaped default (1.5 rev/s², 3 kg) this is
    ~0.11 A — negligible; at the UNSHAPED single-knot step (~60 rev/s²) it is ~4.5 A,
    which stacked on the ~6 A gravity+friction hold current brushes the over-current
    abort. That contrast is why :func:`shaped_constant_velocity_knots` exists.
    """
    if kt_nm_per_a <= 0.0:
        raise ValueError("kt must be positive")
    a = abs(float(accel_rps2))
    a_lin_m_s2 = a * BENCH_LEG_MM_PER_REV * 1e-3
    tau_load = float(mass_kg) * a_lin_m_s2 * BENCH_SPOOL_RADIUS_M
    tau_rotor = float(rotor_j_kgm2) * 2.0 * math.pi * a
    return (tau_load + tau_rotor) / float(kt_nm_per_a)


def knots_achieved_velocity(series: Sequence[float], seg_t_s: float) -> float:
    """Largest |knot-to-knot| slope in a series (rev/s) — the commanded velocity.

    Reported so the harness can confirm the COMMAND matches the request before
    blaming the leg for a velocity mismatch.
    """
    arr = np.asarray(series, float)
    if arr.size < 2 or seg_t_s <= 0.0:
        return 0.0
    return float(np.max(np.abs(np.diff(arr)))) / float(seg_t_s)


def traverse_duration_s(start_rev: float, end_rev: float, vel_rps: float) -> float:
    """Wall time a constant-velocity traverse takes (excluding lead-in/out)."""
    if vel_rps <= 0.0:
        raise ValueError("vel_rps must be > 0")
    return abs(float(end_rev) - float(start_rev)) / float(vel_rps)


# ===========================================================================
# Steady-velocity window selection
# ===========================================================================
#
# The force balance τ_m = τ_g ± τ_f holds ONLY at constant velocity: any residual
# acceleration adds a 2π·J·a inertia term that biases iq. The selector admits a sample
# when the (lightly smoothed) velocity is at the target speed AND the position sits in
# the middle of the stroke — the accel/decel ramps live at the stroke ENDS, so the
# position margin is what actually excludes them.
#
# HISTORY (2026-07-14): the original selector also gated on |dv/dt| < 1 rev/s² from a
# finite difference of the 250 Hz velocity telemetry. That telemetry is coarsely
# quantized and noisy (σ ≈ 0.23 rev/s at a 0.6 rev/s cruise on the real rig), so the
# differenced accel had σ ≈ 35 rev/s² — the gate starved every traverse down to 23–63
# kept samples (3–7 % of the record) and failed all four of that night's masses. Once
# the velocity is gated the accel gate is REDUNDANT (a sample can only pass |v−target|
# < 10 % while genuinely cruising or in the one-sample knife-edge of a ramp, and the
# position margin removes the ramps entirely), so it was DROPPED, not retuned.
# The replacement gate on the same CSVs kept n = 47–159 with SEMs 0.05–0.11 A and a
# mass-fit R² = 0.99909.


def boxcar_smooth(x: Sequence[float], n: int = 5) -> np.ndarray:
    """NaN-aware boxcar smoother (window ``n`` samples, centred).

    Non-finite samples are excluded from each window's average rather than poisoning
    it; a window with no finite samples yields NaN. Used to knock the coarse velocity
    quantization noise down by ~√n before the at-speed gate.
    """
    x = np.asarray(x, float)
    if n <= 1 or x.size == 0:
        return x.copy()
    finite = np.isfinite(x)
    vals = np.where(finite, x, 0.0)
    kern = np.ones(int(n), float)
    num = np.convolve(vals, kern, mode='same')
    den = np.convolve(finite.astype(float), kern, mode='same')
    with np.errstate(divide='ignore', invalid='ignore'):
        out = np.where(den > 0, num / den, np.nan)
    return out


@dataclass
class SteadyWindow:
    mask: np.ndarray             # bool, one per sample
    n_total: int
    n_kept: int
    kept_frac: float
    mean_vel_rps: float          # signed, over the kept samples
    vel_error_frac: float        # |mean|v|| vs target, as a fraction
    reasons: List[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.reasons


def steady_state_mask(t_s: Sequence[float], vel_rps: Sequence[float],
                      pos_rev: Sequence[float], *,
                      v_target_rps: float,
                      traverse_lo_rev: float,
                      traverse_hi_rev: float,
                      vel_tol_frac: float = 0.10,
                      smooth_n: int = 5,
                      pos_margin_frac: float = 0.15,
                      edge_discard_s: float = 0.0,
                      min_samples: int = 50) -> SteadyWindow:
    """Select the constant-velocity samples of one traverse.

    A sample survives when ALL of:

    * the **boxcar-smoothed** ``|v|`` (window ``smooth_n``, NaN-aware) is within
      ``vel_tol_frac`` of ``|v_target|`` — we are actually at speed. Smoothing first
      matters: the raw 250 Hz velocity estimate is coarsely quantized (σ ≈ 0.23 rev/s
      on the real rig), and gating the raw signal throws away most of a genuine cruise;
    * the **position** sits inside the middle of the stroke — the traverse span shrunk
      by ``pos_margin_frac`` at EACH end. The accel/decel ramps live at the stroke
      ends, so this is what excludes them (and any sample merely *passing through* the
      target speed while decelerating near an endpoint);
    * optionally, it is at least ``edge_discard_s`` from both time-ends of the record
      (a redundant belt-and-braces once the position margin is in place; default 0).

    There is deliberately NO acceleration gate — see the section comment above: on the
    real telemetry the differenced-velocity accel estimate was pure noise (σ ≈ 35
    rev/s² against a 1 rev/s² tolerance) and starved the selector to 3–7 % of the
    record on every 2026-07-14 traverse.

    ``reasons`` is non-empty (and ``ok`` False) when fewer than ``min_samples`` survive
    — either the traverse never reached speed or it was too short to have a steady
    middle. The harness treats that as a hard failure for that traverse rather than
    averaging garbage.
    """
    t = np.asarray(t_s, float)
    v = np.asarray(vel_rps, float)
    p = np.asarray(pos_rev, float)
    n = int(min(t.size, v.size, p.size))
    if n == 0:
        return SteadyWindow(np.zeros(0, bool), 0, 0, 0.0, 0.0, 1.0,
                            ['no samples'])
    t = t[:n]
    v = v[:n]
    p = p[:n]
    vt = abs(float(v_target_rps))
    if vt <= 0.0:
        raise ValueError("v_target_rps must be non-zero")
    lo = float(min(traverse_lo_rev, traverse_hi_rev))
    hi = float(max(traverse_lo_rev, traverse_hi_rev))
    span = hi - lo
    if span <= 0.0:
        raise ValueError("traverse bounds must span a non-zero stroke")

    keep = np.isfinite(t) & np.isfinite(v) & np.isfinite(p)

    # Optional time-edge discard, referenced to the record's own span.
    if edge_discard_s > 0.0 and np.any(np.isfinite(t)):
        t0, t1 = float(np.nanmin(t)), float(np.nanmax(t))
        keep &= (t >= t0 + edge_discard_s) & (t <= t1 - edge_discard_s)

    # Position margin: only the middle of the stroke counts as cruise.
    margin = pos_margin_frac * span
    keep &= (p >= lo + margin) & (p <= hi - margin)

    # At-speed test on the SMOOTHED magnitude (direction is the traverse's business).
    v_smooth = boxcar_smooth(v, smooth_n)
    with np.errstate(invalid='ignore'):
        keep &= np.abs(np.abs(v_smooth) - vt) <= vel_tol_frac * vt

    n_kept = int(np.count_nonzero(keep))
    kept_frac = n_kept / n if n else 0.0
    mean_v = float(np.mean(v[keep])) if n_kept else 0.0
    v_err = abs(abs(mean_v) - vt) / vt if n_kept else 1.0

    reasons: List[str] = []
    if n_kept < min_samples:
        reasons.append(f"only {n_kept} steady samples (< {min_samples}) — the traverse "
                       f"never held a constant velocity long enough to measure")
    return SteadyWindow(mask=keep, n_total=n, n_kept=n_kept, kept_frac=kept_frac,
                        mean_vel_rps=mean_v, vel_error_frac=v_err, reasons=reasons)


# ===========================================================================
# Traverse summary
# ===========================================================================


@dataclass
class TraverseStats:
    direction: str              # 'up' (extending) | 'down' (retracting)
    n_samples: int
    iq_mean_A: float
    iq_std_A: float
    iq_sem_A: float             # standard error of the mean (autocorrelation-inflated)
    vel_mean_rps: float
    vel_error_frac: float
    ok: bool
    reasons: List[str] = field(default_factory=list)


def effective_sample_size(x: Sequence[float]) -> float:
    """Autocorrelation-corrected effective N: ``n / (1 + 2·Σ ρ_k)`` over positive lags.

    A 250 Hz iq trace is HEAVILY autocorrelated (cogging ripple, current-loop dynamics,
    the mechanical time constant), so the naive ``σ/√n`` understates the standard error
    of the mean by a large factor — it would let the harness claim a false 20σ exclusion
    from what is really a few independent observations. The initial-positive-sequence
    estimator sums the autocorrelation until it first goes non-positive, the standard
    conservative truncation.
    """
    arr = np.asarray(x, float)
    arr = arr[np.isfinite(arr)]
    n = arr.size
    if n < 4:
        return float(max(n, 1))
    d = arr - arr.mean()
    denom = float(np.dot(d, d))
    if denom <= 0.0:
        return float(n)
    ac = np.correlate(d, d, mode='full')[n - 1:] / denom
    tau = 1.0
    for k in range(1, min(n // 2, 1000)):
        if ac[k] <= 0.0:
            break
        tau += 2.0 * float(ac[k])
    if tau < 1.0:
        tau = 1.0
    return float(max(1.0, n / tau))


def summarize_traverse(t_s: Sequence[float], vel_rps: Sequence[float],
                       pos_rev: Sequence[float], iq_A: Sequence[float], *,
                       direction: str, v_target_rps: float,
                       traverse_lo_rev: float, traverse_hi_rev: float,
                       **window_kw) -> TraverseStats:
    """Steady-window mean iq for one traverse, with an honest standard error.

    ``iq_A`` is the RAW reported current (``Diagnostic.iq_measured``) — this function does
    NOT apply any sign convention. Frame resolution happens once, downstream, in
    :func:`infer_extension_iq_sign`, from the physics rather than from an assumption.
    ``pos_rev`` + the traverse bounds feed the position-margin cruise gate (see
    :func:`steady_state_mask`).
    """
    win = steady_state_mask(t_s, vel_rps, pos_rev, v_target_rps=v_target_rps,
                            traverse_lo_rev=traverse_lo_rev,
                            traverse_hi_rev=traverse_hi_rev, **window_kw)
    iq = np.asarray(iq_A, float)
    n = min(iq.size, win.mask.size)
    sel = iq[:n][win.mask[:n]]
    sel = sel[np.isfinite(sel)]
    reasons = list(win.reasons)
    if sel.size == 0:
        reasons.append("no finite iq samples in the steady window — is the "
                       "BENCH_SYSID_BUILD firmware flashed? stock v3 gates iq to ~1 Hz")
        return TraverseStats(direction=direction, n_samples=0, iq_mean_A=float('nan'),
                             iq_std_A=float('nan'), iq_sem_A=float('inf'),
                             vel_mean_rps=win.mean_vel_rps,
                             vel_error_frac=win.vel_error_frac,
                             ok=False, reasons=reasons)
    mean = float(np.mean(sel))
    std = float(np.std(sel, ddof=1)) if sel.size > 1 else 0.0
    n_eff = effective_sample_size(sel)
    sem = std / math.sqrt(n_eff) if n_eff > 0 else float('inf')
    return TraverseStats(
        direction=direction, n_samples=int(sel.size), iq_mean_A=mean, iq_std_A=std,
        iq_sem_A=float(sem), vel_mean_rps=win.mean_vel_rps,
        vel_error_frac=win.vel_error_frac, ok=not reasons, reasons=reasons)


# ===========================================================================
# The friction-cancelling combination
# ===========================================================================


@dataclass
class MassPoint:
    mass_kg: float
    iq_up_A: float              # raw reported frame
    iq_down_A: float
    iq_avg_A: float             # (up + down)/2 — τ_g/Kt, friction CANCELLED
    iq_halfdiff_A: float        # (up − down)/2 — τ_f/Kt, the free friction measurement
    iq_avg_sem_A: float
    iq_halfdiff_sem_A: float


def combine_traverses(mass_kg: float, up: TraverseStats, down: TraverseStats
                      ) -> MassPoint:
    """Form the friction-cancelling average and the friction half-difference.

    ``iq_avg = (iq_up + iq_down)/2``: the friction torque flips sign with velocity while
    gravity does not, so the τ_f terms cancel EXACTLY and what remains is τ_g/Kt. This is
    the entire point of the method and the reason it is immune to the stiction bias the
    historical at-rest measurement is suspected of carrying.

    ``iq_halfdiff = (iq_up − iq_down)/2`` = τ_f/Kt: an independent measurement of friction
    that costs nothing extra and cross-checks the whole picture against the known τ_c.

    Both SEMs propagate as ``sqrt(σ_up² + σ_down²)/2`` (the traverses are independent
    records, so their errors add in quadrature).
    """
    avg = 0.5 * (up.iq_mean_A + down.iq_mean_A)
    half = 0.5 * (up.iq_mean_A - down.iq_mean_A)
    sem = 0.5 * math.sqrt(up.iq_sem_A ** 2 + down.iq_sem_A ** 2)
    return MassPoint(mass_kg=float(mass_kg), iq_up_A=up.iq_mean_A,
                     iq_down_A=down.iq_mean_A, iq_avg_A=avg, iq_halfdiff_A=half,
                     iq_avg_sem_A=sem, iq_halfdiff_sem_A=sem)


# ===========================================================================
# The Kt fit (weighted, WITH an intercept — mandatory)
# ===========================================================================


@dataclass
class KtFit:
    kt_nm_per_a: float
    kt_sigma: float
    kt_ci95: Tuple[float, float]
    slope_A_per_kg: float
    slope_sigma: float
    intercept_A: float          # absorbs the leg's own moving mass + cable preload
    intercept_sigma: float
    r_squared: float
    residuals_A: List[float]
    n_points: int
    dof: int
    reasons: List[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.reasons


def fit_kt(points: Sequence[MassPoint], *, tilt_deg: float = 0.0) -> KtFit:
    """Weighted least-squares ``iq_avg = a·m + b`` ⇒ ``Kt = g·r·cos θ / |a|``.

    **The intercept is mandatory.** ``b`` absorbs the leg's own moving mass (the carriage,
    the cable, the spool) and any cable preload — a constant current offset that has
    nothing to do with the hanging mass. Forcing the fit through the origin would fold
    that constant into the slope and bias Kt by exactly the amount we are trying to
    resolve. (A forced-origin fit is also how a stiction-contaminated at-rest measurement
    can look like a clean R²=0.994 while being 13 % wrong.)

    Weights are ``1/σ_i²`` from the per-point SEMs. The parameter covariance is scaled by
    the reduced chi-square, so an underestimated per-point σ (or genuine model misfit)
    INFLATES the reported uncertainty rather than manufacturing false confidence. With
    ``dof = n − 2``, three masses give one residual degree of freedom — enough to fit, but
    ``reasons`` warns that the uncertainty is then poorly determined.

    ``Kt`` is taken from ``|a|``: the sign of ``a`` reports the reported-iq frame (see
    :func:`infer_extension_iq_sign`) and carries no magnitude information.
    """
    reasons: List[str] = []
    n = len(points)
    if n < 3:
        return KtFit(float('nan'), float('inf'), (float('nan'), float('nan')),
                     float('nan'), float('inf'), float('nan'), float('inf'),
                     float('nan'), [], n, max(0, n - 2),
                     [f"need >= 3 mass points to fit slope + intercept, got {n}"])

    m = np.array([p.mass_kg for p in points], float)
    y = np.array([p.iq_avg_A for p in points], float)
    sig = np.array([p.iq_avg_sem_A for p in points], float)
    # A zero/non-finite SEM would blow up the weights; fall back to equal weighting.
    if not np.all(np.isfinite(sig)) or np.any(sig <= 0.0):
        sig = np.ones(n)
        reasons.append("per-point SEMs unusable — fell back to equal weights; the "
                       "reported Kt uncertainty is residual-scaled only")

    w = 1.0 / (sig ** 2)
    X = np.vstack([m, np.ones(n)]).T
    W = np.diag(w)
    xtwx = X.T @ W @ X
    try:
        xtwx_inv = np.linalg.inv(xtwx)
    except np.linalg.LinAlgError:
        return KtFit(float('nan'), float('inf'), (float('nan'), float('nan')),
                     float('nan'), float('inf'), float('nan'), float('inf'),
                     float('nan'), [], n, n - 2,
                     ["design matrix is singular — are all the masses identical?"])
    beta = xtwx_inv @ (X.T @ W @ y)
    a, b = float(beta[0]), float(beta[1])
    resid = y - X @ beta
    dof = n - 2

    # Residual-scaled covariance: inflate by the reduced chi-square so model misfit or an
    # optimistic SEM widens the interval instead of hiding inside it.
    chi2 = float(resid.T @ W @ resid)
    scale = (chi2 / dof) if dof > 0 else 1.0
    if scale < 1.0:
        scale = 1.0        # never SHRINK below the propagated measurement error
    cov = xtwx_inv * scale
    a_sig = float(math.sqrt(max(0.0, cov[0, 0])))
    b_sig = float(math.sqrt(max(0.0, cov[1, 1])))

    ss_res = float(np.sum(resid ** 2))
    ss_tot = float(np.sum((y - y.mean()) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float('nan')

    if abs(a) < 1e-9:
        reasons.append("fitted slope is ~0 — no current response to mass at all; "
                       "is the leg actually loaded? is iq being reported?")
        return KtFit(float('nan'), float('inf'), (float('nan'), float('nan')),
                     a, a_sig, b, b_sig, r2, [float(x) for x in resid], n, dof,
                     reasons)

    g_r = G_M_S2 * BENCH_SPOOL_RADIUS_M * math.cos(math.radians(tilt_deg))
    kt = g_r / abs(a)
    # Propagate: Kt = g·r/|a| ⇒ σ_Kt/Kt = σ_a/|a|.
    kt_sig = kt * (a_sig / abs(a)) if a_sig < abs(a) else float('inf')
    ci = (kt - 1.96 * kt_sig, kt + 1.96 * kt_sig) if math.isfinite(kt_sig) else (
        float('nan'), float('nan'))

    if dof < 2:
        reasons.append(f"dof = {dof} — the uncertainty is itself poorly determined; "
                       f"use >= 4 masses for a trustworthy confidence interval")
    if math.isfinite(r2) and r2 < 0.98:
        reasons.append(f"R² = {r2:.4f} is low for a linear gravity law — check for a "
                       f"non-steady traverse, a slipping mass, or a tilted leg")
    return KtFit(kt_nm_per_a=kt, kt_sigma=kt_sig, kt_ci95=ci, slope_A_per_kg=a,
                 slope_sigma=a_sig, intercept_A=b, intercept_sigma=b_sig,
                 r_squared=r2, residuals_A=[float(x) for x in resid],
                 n_points=n, dof=dof, reasons=reasons)


# ===========================================================================
# Verdict: which candidate does the measurement support?
# ===========================================================================


@dataclass
class KtVerdict:
    kt_nm_per_a: float
    kt_sigma: float
    nearest: str
    sigma_to: Dict[str, float]     # candidate name -> |Kt − cand| / σ_Kt
    excluded: List[str]            # candidates beyond `exclude_sigma`
    consistent: List[str]          # candidates within `exclude_sigma`
    summary: str


def classify_kt(fit: KtFit, *, candidates: Optional[Dict[str, float]] = None,
                exclude_sigma: float = 3.0) -> KtVerdict:
    """How many sigma does the measurement put between itself and each candidate?

    A candidate more than ``exclude_sigma`` away is EXCLUDED by this measurement. The
    headline the operator needs is not "Kt = x" but "Kt excludes 0.0551 at Nσ and is
    consistent with 0.0624" (or the reverse) — that is what decides whether the ODrive's
    ``torque_constant`` or ``hardware_config.yaml`` is the thing that has to change.
    """
    cands = candidates if candidates is not None else KT_CANDIDATES
    if not math.isfinite(fit.kt_nm_per_a):
        return KtVerdict(float('nan'), float('inf'), '?', {}, [], [],
                         "fit failed — no verdict")
    sig = fit.kt_sigma if math.isfinite(fit.kt_sigma) and fit.kt_sigma > 0 else float('inf')
    sigma_to = {name: abs(fit.kt_nm_per_a - val) / sig if math.isfinite(sig) else 0.0
                for name, val in cands.items()}
    nearest = min(cands, key=lambda k: abs(fit.kt_nm_per_a - cands[k]))
    excluded = sorted(k for k, s in sigma_to.items() if s > exclude_sigma)
    consistent = sorted(k for k, s in sigma_to.items() if s <= exclude_sigma)
    parts = [f"Kt = {fit.kt_nm_per_a:.5f} ± {fit.kt_sigma:.5f} Nm/A"]
    parts.append(f"nearest candidate: {nearest} ({cands[nearest]:.5f})")
    if excluded:
        parts.append("EXCLUDES " + ", ".join(
            f"{k} ({cands[k]:.5f}) at {sigma_to[k]:.1f}σ" for k in excluded))
    if consistent:
        parts.append("consistent with " + ", ".join(
            f"{k} ({cands[k]:.5f}) at {sigma_to[k]:.1f}σ" for k in consistent))
    return KtVerdict(kt_nm_per_a=fit.kt_nm_per_a, kt_sigma=fit.kt_sigma,
                     nearest=nearest, sigma_to=sigma_to, excluded=excluded,
                     consistent=consistent, summary="; ".join(parts))


# ===========================================================================
# The friction cross-check — load-bearing, not decorative
# ===========================================================================


@dataclass
class FrictionCheck:
    halfdiff_mean_A: float       # |(iq_up − iq_down)/2| averaged over masses
    halfdiff_std_A: float
    per_mass_A: List[float]
    tau_c_ref_A: float
    ratio_to_ref: float
    constant_across_masses: bool
    near_reference: bool
    ok: bool
    reasons: List[str] = field(default_factory=list)


def friction_consistency(points: Sequence[MassPoint], *,
                         tau_c_ref_A: float = TAU_C_REF_A,
                         ref_range_A: Tuple[float, float] = TAU_C_REF_RANGE_A,
                         constancy_tol_frac: float = 0.30,
                         tolerance_frac: float = 0.40) -> FrictionCheck:
    """Does the free friction measurement corroborate the whole picture?

    ``(iq_up − iq_down)/2`` = τ_f/Kt should be

    1. **≈ constant across masses** — Coulomb friction does not depend on the hanging
       load (the cable tension changes, so a mild trend is expected; a strong one means
       load-dependent friction and the τ_f-cancels-exactly assumption is shakier); and
    2. **≈ the independently-known τ_c ≈ 1.094 A** (logbook 2026-04-27:79, reproduced at
       0.88–1.22 A by ``bench_leg_plant_id.py``).

    If both hold, the physical model is confirmed by a quantity the Kt fit never used, and
    the Kt number is trustworthy. If they do NOT hold, the harness says so LOUDLY: a
    friction term that is not what we think it is undermines the exact cancellation the
    whole method rests on, and the Kt number must not be believed.
    """
    vals = [abs(p.iq_halfdiff_A) for p in points]
    reasons: List[str] = []
    if not vals:
        return FrictionCheck(float('nan'), float('nan'), [], tau_c_ref_A, float('nan'),
                             False, False, False, ['no mass points'])
    arr = np.array(vals, float)
    mean = float(np.mean(arr))
    std = float(np.std(arr, ddof=1)) if arr.size > 1 else 0.0
    spread = (std / mean) if mean > 0 else math.inf
    constant = spread <= constancy_tol_frac
    ratio = mean / tau_c_ref_A if tau_c_ref_A > 0 else math.inf
    lo, hi = ref_range_A
    near = (lo * (1.0 - tolerance_frac) <= mean <= hi * (1.0 + tolerance_frac))

    if not constant:
        reasons.append(
            f"friction half-difference varies {spread * 100:.0f}% across masses "
            f"(> {constancy_tol_frac * 100:.0f}%) — Coulomb friction should be roughly "
            f"load-independent; a strong trend means the tau_f-cancels-exactly assumption "
            f"is shaky and Kt is less trustworthy")
    if not near:
        reasons.append(
            f"friction half-difference {mean:.2f} A is far from the independently-known "
            f"tau_c = {tau_c_ref_A:.2f} A (reproduced range {lo:.2f}-{hi:.2f} A) — "
            f"SOMETHING IS WRONG: the traverses may not be at steady velocity, the leg "
            f"may not be vertical, or the up/down speeds may differ. Do not trust Kt.")
    return FrictionCheck(halfdiff_mean_A=mean, halfdiff_std_A=std, per_mass_A=vals,
                         tau_c_ref_A=tau_c_ref_A, ratio_to_ref=ratio,
                         constant_across_masses=constant, near_reference=near,
                         ok=not reasons, reasons=reasons)


def slope_friction_sign_agree(fit: KtFit, points: Sequence[MassPoint], *,
                              rig_orientation: str = 'normal') -> bool:
    """Internal consistency between the fitted slope and the friction half-difference,
    with the expected relation set by the RIG ORIENTATION.

    On a NORMAL rig (extension raises the load) both carry the SAME sign in the raw
    reported frame: gravity opposes extension (more mass ⇒ more extending current) and
    friction opposes motion (the extending traverse needs more extending current than
    the retracting one), and whatever sign the raw telemetry frame applies, it applies
    to both equally.

    On an INVERTED rig (contraction raises the load) gravity *assists* extension while
    friction still opposes motion, so the slope and the half-difference must carry
    OPPOSITE signs — the constant-velocity balance is ``τ_m = ±τ_f − τ_g`` in the
    extension-positive frame, giving ``iq_avg = −τ_g/Kt`` against ``halfdiff = +τ_f/Kt``.

    A violation of the orientation-appropriate relation means either the up/down
    traverses are mislabelled (which would silently invert the whole result) or the
    ``--rig-orientation`` flag is wrong — so this is checked rather than assumed.
    """
    if rig_orientation not in RIG_ORIENTATIONS:
        raise ValueError(f"rig_orientation must be one of {RIG_ORIENTATIONS}")
    if not points or not math.isfinite(fit.slope_A_per_kg):
        return False
    mean_half = float(np.mean([p.iq_halfdiff_A for p in points]))
    if mean_half == 0.0 or fit.slope_A_per_kg == 0.0:
        return False
    same = (fit.slope_A_per_kg > 0) == (mean_half > 0)
    return same if rig_orientation == 'normal' else not same


# ===========================================================================
# Sign inference — the self-calibrating gravity reference
# ===========================================================================


@dataclass
class SignInference:
    iq_extension_sign: int         # +1 or −1: sign of reported iq for an EXTENDING torque
    source: str
    matches_code_read: bool
    note: str


def infer_extension_iq_sign(fit: KtFit, *,
                            rig_orientation: str = 'normal') -> SignInference:
    """Which sign of the REPORTED ``iq_measured`` corresponds to an EXTENDING torque?

    **The self-calibrating reference.** Gravity loads the leg through the rig, so the
    sign of ``d(iq_avg)/dm`` — the fitted slope — reports the frame, PROVIDED the rig
    orientation is known:

    * ``normal`` (extension raises the load): more mass ⇒ more EXTENDING torque, so the
      slope sign IS the extension sign.
    * ``inverted`` (contraction raises the load): more mass ⇒ more CONTRACTING torque,
      so the extension sign is the NEGATED slope sign.

    The 2026-07-14 manifests recorded ``extension_iq_sign = −1`` the wrong way round
    precisely because the rig was inverted while this function assumed ``normal`` — the
    orientation is now a REQUIRED operator declaration, not an assumption.

    The code-read PREDICTION is −1 (``IQ_EXTENSION_SIGN_PREDICTED``): ``can_buses.cpp:92``
    stores ``iq_measured`` raw in the ODrive frame while ``:85-86`` negate pos/vel into the
    Jugglebot extension-positive frame, and ``odrive_protocol.h:49`` makes ODrive-positive
    == Jugglebot-retraction for legs. We report whether the measurement AGREES. A
    disagreement is a real finding either way — either the code read is wrong or the rig is
    wired differently than believed — and the harness must not paper over it.
    """
    if rig_orientation not in RIG_ORIENTATIONS:
        raise ValueError(f"rig_orientation must be one of {RIG_ORIENTATIONS}")
    if not math.isfinite(fit.slope_A_per_kg) or fit.slope_A_per_kg == 0.0:
        return SignInference(0, 'indeterminate', False,
                             "slope is zero/NaN — cannot infer the iq frame")
    s = 1 if fit.slope_A_per_kg > 0 else -1
    if rig_orientation == 'inverted':
        s = -s
    match = (s == IQ_EXTENSION_SIGN_PREDICTED)
    if match:
        note = ("measured iq sign for an extending torque MATCHES the code-read "
                "prediction (can_buses.cpp:92 reports iq RAW in the ODrive frame while "
                ":85-86 negate pos/vel — so extension reads as negative iq)")
    else:
        note = ("*** MISMATCH *** the measured iq sign for an extending torque is "
                f"{s:+d}, but the firmware code-read predicts "
                f"{IQ_EXTENSION_SIGN_PREDICTED:+d} (can_buses.cpp:85-86 vs :92). Either "
                "the code read is wrong or this rig is wired differently. RESOLVE THIS "
                "BEFORE SHIPPING ANY TORQUE FEEDFORWARD — it is a sign error waiting to "
                "happen.")
    return SignInference(iq_extension_sign=s,
                         source=f'gravity-loaded slope ({rig_orientation} rig)',
                         matches_code_read=match, note=note)


def extension_sign_from_hold(hold_iq_A: float, rig_orientation: str, *,
                             min_abs_A: float = 0.2) -> int:
    """Extension-iq sign inferred from a single loaded static hold.

    The settled hold current is the torque HOLDING the load up. On a ``normal`` rig
    (extension raises the load) that torque is EXTENDING, so the hold-current sign IS
    the extension sign; on an ``inverted`` rig (contraction raises the load) the
    holding torque is CONTRACTING, so the extension sign is the NEGATED hold sign.
    Returns 0 (indeterminate) when ``|hold_iq|`` is under ``min_abs_A`` — an unloaded
    or barely-loaded hold cannot calibrate the frame.

    This is the single-hold flavour of the gravity self-calibration; the 2026-07-14
    manifests recorded the sign wrong-way-round because the then-current code assumed
    holding == extending (i.e. a normal rig) while the rig was inverted.
    """
    if rig_orientation not in RIG_ORIENTATIONS:
        raise ValueError(f"rig_orientation must be one of {RIG_ORIENTATIONS}")
    v = float(hold_iq_A)
    if not math.isfinite(v) or abs(v) < min_abs_A:
        return 0
    s = 1 if v > 0 else -1
    return s if rig_orientation == 'normal' else -s


# ===========================================================================
# MODE 2 — the torque_ff channel, end-to-end (EDGE CAPTURE)
# ===========================================================================
#
# THE 2026-07-15 REDESIGN. The original design held the leg and stepped torque_ff
# through a RUNG LADDER, measuring a "steady" window ~1.3 s after each step. That
# design is structurally confounded and is dead, for a physics reason verified on the
# 2026-07-14 data: at a genuinely SETTLED static hold the velocity-loop integrator
# forces pos == cmd, where torque balance pins the TOTAL iq at the load value — the
# steady-state iq response to a CONSTANT torque_ff is exactly ZERO. What the ladder
# measured was the locked-state re-absorption TRANSIENT (τ ≈ 2–10 s) sampled at a
# fixed ~1.3 s cadence — and because rung index and wall-clock were collinear at that
# fixed cadence, any slow drift aliased directly into the fitted slope (the 20:55
# run's −21 A/Nm "slope" was exactly that: time ⊥ tff collinearity, not physics).
#
# EDGE CAPTURE instead measures the INSTANTANEOUS iq jump at each edge of a ± square
# wave, before the loop can re-absorb it:
#
#   1. SETTLE GATE — wait until pos is within ±0.3 mrev of cmd AND |d(iq)/dt| < 0.05
#      A/s over a 3 s window (max wait ~30 s). Measuring an unsettled hold is what
#      poisoned the ladder.
#   2. SQUARE WAVE — torque_ff alternates ±X with half-period 1.5–2 s (default 1.75),
#      X ∈ {0.010, 0.020, 0.035} Nm: all inside the 0.045 Nm static-friction band, so
#      the lock keeps the position loop blind (±0.36 A excursion at X = 0.02 vs the
#      >3.5 A abort headroom). ≥10 cycles per amplitude, telemetry recorded
#      continuously at 250 Hz.
#   3. MATCHED FILTER — each edge is located by cross-correlating a step kernel with
#      the iq stream near the commanded toggle time (±0.1 s), never by wall-clock
#      alone.
#   4. JUMP — mean(iq[t_e+0.02 .. t_e+0.20]) − mean(iq[t_e−0.20 .. t_e−0.02]) per
#      edge, with a per-edge exponential decay correction (creep costs 2.5–9.5 % at
#      0.2 s for τ = 2–10 s; corrected with the fitted per-hold τ, default 8 s).
#      slope = jump / (2X), signed by the edge polarity — so drift contributes with
#      ALTERNATING sign and cancels in the slope, and is separately detected by the
#      paired-edge drift statistic.
#   5. VERDICT — |slope|/σ ≥ 3 AND trimmed edge-jump CV ≤ 20 % (the edge-capture
#      analogue of the old R² gate), else NO CONCLUSION. Expected |slope| =
#      1/torque_constant = 18.14 A/Nm, with the sign referred through the operator's
#      --rig-orientation declaration.
#
# The ODrive sums its controller output and the feedforward before dividing by
# torque_constant, so at the instant of an edge (position loop blind inside the
# static-friction band, integrator not yet moved):
#
#     d(iq) / d(torque_ff)  =  1 / Kt_odrive        (= 18.14 A/Nm at 0.055133)
#
# It is inherently SAFE — the position loop actively resists any runaway, every
# commanded torque_ff is tiny against torque_soft_max (1.5 Nm), and nothing audibly
# moves.
#
# The LADDER-fit functions below (TorqueFfPoint / fit_torque_ff / assess_tff_fit_quality
# / classify_torque_ff) are RETAINED: the differential-shift algebra is still correct
# for transient data, classify_torque_ff doubles as the verdict container the edge
# pipeline reuses, and the 2026-07-14 regression fixtures exercise them — but the
# HARNESS measurement core is edge capture (see the EDGE CAPTURE section further down).


@dataclass
class TorqueFfPoint:
    torque_ff_Nm: float         # what we put on the wire (Jugglebot extension-positive)
    iq_mean_A: float            # raw reported frame
    iq_sem_A: float
    pos_mean_rev: float
    n_samples: int


@dataclass
class TorqueFfFit:
    slope_A_per_Nm: float       # d(iq_raw)/d(torque_ff_wire)
    slope_sigma: float
    intercept_A: float          # the loaded-hold current at torque_ff = 0
    intercept_sigma: float
    r_squared: float
    residuals_A: List[float]
    n_points: int
    dof: int
    implied_kt_nm_per_a: float  # 1/|slope| — the torque_constant the ODrive ACTUALLY used
    implied_kt_sigma: float
    reasons: List[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.reasons


def fit_torque_ff(points: Sequence[TorqueFfPoint]) -> TorqueFfFit:
    """OLS ``iq = s·torque_ff + c`` over the ladder.

    ``|s|`` should be ``1/Kt_odrive`` = 1/0.055133 = **18.14 A/Nm** if torque_ff is
    interpreted exactly like ``input_torque`` (operator fact #2) and the int16 ×10000 wire
    scaling round-trips correctly. The intercept ``c`` is the loaded-hold current at zero
    feedforward — it carries the (indeterminate) stiction offset, which is exactly why it
    is fitted out rather than used.

    ``implied_kt = 1/|s|`` is the torque_constant the ODrive ACTUALLY divided by. It is a
    property of the ODrive's CONFIG, not of the motor — compare it to
    ``KT_ODRIVE_CONFIGURED``, never to the true Kt from Mode 1.
    """
    reasons: List[str] = []
    n = len(points)
    if n < 3:
        return TorqueFfFit(float('nan'), float('inf'), float('nan'), float('inf'),
                           float('nan'), [], n, max(0, n - 2), float('nan'),
                           float('inf'),
                           [f"need >= 3 ladder points, got {n}"])
    x = np.array([p.torque_ff_Nm for p in points], float)
    y = np.array([p.iq_mean_A for p in points], float)
    sig = np.array([p.iq_sem_A for p in points], float)
    if not np.all(np.isfinite(sig)) or np.any(sig <= 0.0):
        sig = np.ones(n)
        reasons.append("per-point SEMs unusable — equal weights; uncertainty is "
                       "residual-scaled only")
    if float(np.ptp(x)) <= 0.0:
        return TorqueFfFit(float('nan'), float('inf'), float('nan'), float('inf'),
                           float('nan'), [], n, n - 2, float('nan'), float('inf'),
                           ["all ladder points have the same torque_ff — no leverage"])

    w = 1.0 / sig ** 2
    X = np.vstack([x, np.ones(n)]).T
    W = np.diag(w)
    xtwx_inv = np.linalg.inv(X.T @ W @ X)
    beta = xtwx_inv @ (X.T @ W @ y)
    s, c = float(beta[0]), float(beta[1])
    resid = y - X @ beta
    dof = n - 2
    chi2 = float(resid.T @ W @ resid)
    scale = max(1.0, (chi2 / dof) if dof > 0 else 1.0)
    cov = xtwx_inv * scale
    s_sig = float(math.sqrt(max(0.0, cov[0, 0])))
    c_sig = float(math.sqrt(max(0.0, cov[1, 1])))
    ss_res = float(np.sum(resid ** 2))
    ss_tot = float(np.sum((y - y.mean()) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float('nan')

    if abs(s) < 1e-6:
        reasons.append(
            "*** torque_ff produced NO measurable current shift *** — the channel is "
            "DEAD. The bridge is not forwarding torque_ff to the ODrive, or the "
            "interp is zeroing it (leg_interp.cpp:407 stroke clamp / :479 recovery "
            "slew), or the ODrive is ignoring it. DO NOT ship a torque feedforward.")
        implied, implied_sig = float('nan'), float('inf')
    else:
        implied = 1.0 / abs(s)
        implied_sig = implied * (s_sig / abs(s)) if s_sig < abs(s) else float('inf')
    if math.isfinite(r2) and r2 < 0.98:
        reasons.append(f"R² = {r2:.4f} — the iq response to torque_ff is not clean; "
                       f"check that the hold was settled at every rung")
    return TorqueFfFit(slope_A_per_Nm=s, slope_sigma=s_sig, intercept_A=c,
                       intercept_sigma=c_sig, r_squared=r2,
                       residuals_A=[float(v) for v in resid], n_points=n, dof=dof,
                       implied_kt_nm_per_a=implied, implied_kt_sigma=implied_sig,
                       reasons=reasons)


@dataclass
class TorqueFfVerdict:
    # TRI-STATE liveness: True = a measured response proves the channel live; False =
    # a DEMONSTRATED PRECISE NULL (slope bounded well below the expected 18.14 A/Nm at
    # 3σ) proves it dead; None = this run says NOTHING about liveness (quality gate
    # failed / no conclusion). The 2026-07-14 manifests literally recorded
    # channel_live=false from NO-CONCLUSION runs whose settled data proved the channel
    # live — a no-conclusion must never masquerade as a dead channel again.
    channel_live: Optional[bool]
    positive_tff_extends: Optional[bool]
    sign_matches_expectation: Optional[bool]
    scale_ok: bool
    implied_kt_nm_per_a: float
    sigma_to_odrive_config: float
    lines: List[str] = field(default_factory=list)
    no_conclusion: bool = False    # quality gate failed — sign/scale deliberately withheld


@dataclass
class TffFitQuality:
    """Is the Mode-2 fit trustworthy enough to state ANY sign/scale verdict?

    The gate exists because of the 2026-07-14 run: a messy setup (0.8 kg at an odd
    angle, the operator partially supporting it) produced R² = −0.099 and a slope only
    1.8σ from zero, yet the harness printed dramatic *** MUST NEGATE *** /
    *** MISMATCH −63% *** conclusions from it. A confidently wrong verdict over garbage
    data is strictly worse than refusing.
    """
    trustworthy: bool
    r_squared: float
    t_stat: float                  # |slope| / slope_sigma
    min_r2: float
    min_t_stat: float
    failures: List[str] = field(default_factory=list)
    outlier_indices: List[int] = field(default_factory=list)
    outlier_notes: List[str] = field(default_factory=list)


def assess_tff_fit_quality(fit: TorqueFfFit,
                           points: Optional[Sequence[TorqueFfPoint]] = None, *,
                           min_r2: float = TFF_GATE_MIN_R2,
                           min_t_stat: float = TFF_GATE_MIN_T_STAT,
                           outlier_rms_mult: float = TFF_OUTLIER_RMS_MULT
                           ) -> TffFitQuality:
    """Quality gate for the Mode-2 fit: trustworthy iff ``R² ≥ 0.90`` AND the slope is
    at least ``3σ`` from zero. Below either bar, NO sign/scale verdict may be stated.

    Rungs whose ``|residual|`` exceeds ``outlier_rms_mult ×`` the **leave-one-out** RMS
    of the other residuals are flagged as probable external disturbances (a hand
    touching the mass, a knock). Leave-one-out rather than the plain fit RMS because a
    single large outlier inflates the plain RMS enough to hide itself: the 2026-07-14
    human-touch rung (−0.05 Nm, iq ≈ 0.05 A ≈ unloaded) sits at only 2.1× the plain RMS
    but 3.3× the leave-one-out RMS.
    """
    failures: List[str] = []
    r2 = fit.r_squared
    s = fit.slope_A_per_Nm
    s_sig = fit.slope_sigma
    if not math.isfinite(s):
        t = 0.0
        failures.append("the fit itself failed (slope is not finite): "
                        + ("; ".join(fit.reasons) or "no reason recorded"))
    elif not math.isfinite(s_sig):
        t = 0.0
        failures.append("slope uncertainty is not finite — the fit cannot support any "
                        "statistical statement")
    elif s_sig <= 0.0:
        t = math.inf               # a perfectly determined slope passes the t gate
    else:
        t = abs(s) / s_sig
    if not math.isfinite(r2) or r2 < min_r2:
        failures.append(
            f"R² = {r2:.3f} (< {min_r2:.2f}) — the ladder response is not a line; "
            f"the data are dominated by something other than torque_ff")
    if t < min_t_stat:
        failures.append(
            f"|slope|/σ = {t:.1f} (< {min_t_stat:.0f}) — the slope "
            f"{s:+.2f} ± {s_sig:.2f} A/Nm is statistically indistinguishable from "
            f"NO response at all")

    out_idx: List[int] = []
    out_notes: List[str] = []
    resid = [float(r) for r in (fit.residuals_A or [])]
    n = len(resid)
    if n >= 4:
        ss_all = sum(r * r for r in resid)
        for i, r in enumerate(resid):
            ss_others = ss_all - r * r
            rms_others = math.sqrt(ss_others / (n - 1)) if ss_others > 0 else 0.0
            if rms_others > 0 and abs(r) > outlier_rms_mult * rms_others:
                out_idx.append(i)
                label = (f"rung {points[i].torque_ff_Nm:+.3f} Nm"
                         if points is not None and i < len(points) else f"rung #{i}")
                out_notes.append(
                    f"{label}: residual {r:+.2f} A is "
                    f"{abs(r) / rms_others:.1f}× the RMS of the other rungs — probable "
                    f"external disturbance (a hand supporting the mass, a knock); the "
                    f"mass must hang FREE with nobody touching it")
    return TffFitQuality(trustworthy=not failures, r_squared=float(r2),
                         t_stat=float(t), min_r2=float(min_r2),
                         min_t_stat=float(min_t_stat), failures=failures,
                         outlier_indices=out_idx, outlier_notes=out_notes)


def _no_conclusion_lines(fit: TorqueFfFit, quality: TffFitQuality) -> List[str]:
    lines = [
        "*** NO CONCLUSION — data quality insufficient to validate the torque_ff "
        "channel ***",
        f"    verdict gate: R² ≥ {quality.min_r2:.2f} AND |slope|/σ ≥ "
        f"{quality.min_t_stat:.0f} required before any SIGN/SCALE claim.",
    ]
    lines.extend(f"    FAIL: {f}" for f in quality.failures)
    lines.extend(f"    OUTLIER: {n}" for n in quality.outlier_notes)
    lines.append("    Refusing to state SIGN or SCALE from this fit — a verdict here "
                 "would be noise dressed as a finding.")
    lines.append("    Required setup for a valid run: the mass hangs FREE and VERTICAL "
                 "from the leg; NOBODY touches")
    lines.append("    the mass or the leg during the rungs (one supported rung poisons "
                 "the whole fit); the declared")
    lines.append(f"    --hold-mass is actually on the hook; every rung stays inside the "
                 f"static-friction band (|tff| ≤ {TFF_BAND_WARN_NM:.3f} Nm).")
    return lines


def classify_torque_ff(fit: TorqueFfFit, iq_extension_sign: int, *,
                       scale_tol_frac: float = 0.10,
                       quality: Optional[TffFitQuality] = None) -> TorqueFfVerdict:
    """The hard facts the gravity-FF implementer needs, stated honestly.

    ``iq_extension_sign`` comes from the gravity-calibrated reference (Mode 1's
    :func:`infer_extension_iq_sign`, or a loaded hold in Mode 2). Given it:

    * If ``quality`` is provided and the fit fails the gate (R² < 0.90 or the slope is
      under 3σ from zero), the verdict is **NO CONCLUSION**: no sign or scale claim is
      made, ``channel_live`` is **None** (tri-state — not False: a no-conclusion says
      nothing about liveness, and False is reserved for the demonstrated precise
      null), and the lines say exactly which quality bars failed and which rungs look
      like external disturbances. (Exception: a PRECISE null — a slope bounded well
      below the expected 18.14 A/Nm — is still a conclusion: the channel is dead.)
    * A positive wire ``torque_ff`` **EXTENDS** iff ``sign(slope) == iq_extension_sign``
      — i.e. iff adding positive feedforward pushes the current in the same direction as
      hanging more mass does.
    * SIGN verdicts are framed with the per-drive calibration context
      (:data:`BENCH_VS_PLATFORM_SIGN_NOTE`): the bench drive's torque sign convention is
      OPPOSITE to the platform legs', so a bench "positive retracts" is EXPECTED and
      does NOT transfer to the platform — the platform-validated sign stands.
    * The SCALE is right iff ``implied_kt`` matches the ODrive's CONFIGURED
      ``torque_constant`` (0.055133) within ``scale_tol_frac``. A match confirms both that
      torque_ff is interpreted exactly like ``input_torque`` (operator fact #2) and that
      the int16 ×10000 wire scaling round-trips.
    """
    lines: List[str] = []
    if not math.isfinite(fit.slope_A_per_Nm) or abs(fit.slope_A_per_Nm) < 1e-6:
        lines.append("CHANNEL DEAD — torque_ff produced no current shift. "
                     "DO NOT ship a torque feedforward until this is resolved.")
        return TorqueFfVerdict(False, None, None, False, float('nan'), float('inf'),
                               lines)

    expected = 1.0 / KT_ODRIVE_CONFIGURED
    if quality is not None and not quality.trustworthy:
        # A PRECISE null (slope + 3σ still ≪ the expected 18.14 A/Nm) is a real
        # conclusion — the channel is dead — even though R²/t fail on a flat line.
        precise_null = (math.isfinite(fit.slope_sigma)
                        and abs(fit.slope_A_per_Nm) + 3.0 * fit.slope_sigma
                        < 0.05 * expected)
        if precise_null:
            lines.append("CHANNEL DEAD — torque_ff produced no current shift (the "
                         f"slope is bounded below {0.05 * expected:.1f} A/Nm at 3σ). "
                         "DO NOT ship a torque feedforward until this is resolved.")
            return TorqueFfVerdict(False, None, None, False, float('nan'),
                                   float('inf'), lines)
        # TRI-STATE (2026-07-14 lesson): a NO-CONCLUSION run says nothing about
        # liveness. channel_live=None here — False is reserved for the demonstrated
        # precise null above. That night's manifests recorded a "dead" channel from
        # runs whose settled data proved it live, purely because this path said False.
        return TorqueFfVerdict(
            channel_live=None, positive_tff_extends=None,
            sign_matches_expectation=None, scale_ok=False,
            implied_kt_nm_per_a=fit.implied_kt_nm_per_a,
            sigma_to_odrive_config=float('inf'),
            lines=_no_conclusion_lines(fit, quality), no_conclusion=True)

    extends = None
    matches = None
    if iq_extension_sign in (1, -1):
        extends = (fit.slope_A_per_Nm > 0) == (iq_extension_sign > 0)
        matches = extends is True   # code-read expectation: positive torque_ff EXTENDS
        lines.append(
            f"SIGN: a POSITIVE torque_ff on the wire "
            f"{'EXTENDS (lifts)' if extends else 'RETRACTS (drops)'} THIS BENCH leg. "
            f"[slope {fit.slope_A_per_Nm:+.2f} A/Nm; extending torque reads as "
            f"{iq_extension_sign:+d} iq]")
        if extends:
            lines.append(
                "     matches the naive firmware code read (odrive_protocol.h:171 "
                "leg_sign negation composing with the ODrive frame) — but NOTE it is "
                "the OPPOSITE of the 2026-04-27 bench friction-FF finding (--ff-sign "
                "-1 on this rig), so re-check the drive config before leaning on it.")
            lines.append("     " + BENCH_VS_PLATFORM_SIGN_NOTE)
        else:
            lines.append(
                "     EXPECTED ON THIS BENCH RIG — not a production sign error, and "
                "NOT a reason to negate the gravity feedforward. "
                + BENCH_VS_PLATFORM_SIGN_NOTE)
            lines.append(
                "     (It does contradict the naive code read of odrive_protocol.h:171"
                " — that read does not model per-drive direction calibration.)")
    else:
        lines.append("SIGN: indeterminate — no gravity-calibrated iq extension sign "
                     "available (run --mode kt first, or load the leg in Mode 2).")

    err = abs(abs(fit.slope_A_per_Nm) - expected) / expected
    scale_ok = err <= scale_tol_frac
    sig_to = (abs(fit.implied_kt_nm_per_a - KT_ODRIVE_CONFIGURED) / fit.implied_kt_sigma
              if math.isfinite(fit.implied_kt_sigma) and fit.implied_kt_sigma > 0
              else float('inf'))
    lines.append(
        f"SCALE: |slope| = {abs(fit.slope_A_per_Nm):.2f} A/Nm vs the "
        f"{expected:.2f} A/Nm expected from the ODrive's configured torque_constant "
        f"({KT_ODRIVE_CONFIGURED:.6f}) — {err * 100:.1f}% error ⇒ "
        f"{'OK' if scale_ok else 'MISMATCH'}")
    lines.append(
        f"     implied ODrive torque_constant = {fit.implied_kt_nm_per_a:.5f} "
        f"± {fit.implied_kt_sigma:.5f} Nm/A "
        f"({sig_to:.1f}σ from the configured {KT_ODRIVE_CONFIGURED:.5f})")
    if scale_ok:
        lines.append("     ✓ confirms torque_ff is interpreted exactly like input_torque "
                     "(iq = torque / torque_constant) AND that the int16 x10000 wire "
                     "scaling round-trips correctly.")
    else:
        lines.append("     *** the delivered current does NOT match torque_ff / "
                     "torque_constant. Either the wire scaling is wrong, or torque_ff is "
                     "NOT interpreted like input_torque. A gravity FF would land "
                     f"{(abs(fit.slope_A_per_Nm) / expected - 1) * 100:+.0f}% off. ***")
    return TorqueFfVerdict(
        channel_live=True, positive_tff_extends=extends,
        sign_matches_expectation=matches, scale_ok=scale_ok,
        implied_kt_nm_per_a=fit.implied_kt_nm_per_a, sigma_to_odrive_config=sig_to,
        lines=lines)


def mechanical_torque_delivered_Nm(torque_ff_Nm: float, kt_true: float,
                                   kt_odrive: float = KT_ODRIVE_CONFIGURED) -> float:
    """The MECHANICAL torque a commanded ``torque_ff`` actually delivers.

    The ODrive converts the commanded torque to current with ITS configured constant, then
    the motor converts that current to torque with the TRUE constant::

        tau_real = (torque_ff / Kt_odrive) · Kt_true

    So the delivered torque is off by the ratio ``Kt_true / Kt_odrive``. This is the whole
    reason the discrepancy blocks feedforward work: at Kt_true = 0.0624 and
    Kt_odrive = 0.055133 a commanded 1.0 Nm actually delivers **1.13 Nm** — the +13 %.
    Mode 1 measures ``kt_true``; Mode 2 confirms ``kt_odrive`` is what the ODrive really
    uses; together they give the exact correction factor.
    """
    if kt_odrive <= 0.0:
        raise ValueError("kt_odrive must be positive")
    return float(torque_ff_Nm) / float(kt_odrive) * float(kt_true)


def torque_ff_ladder_safe(ladder_Nm: Sequence[float], *,
                          cap_Nm: float = TORQUE_FF_HARD_CAP_NM) -> List[str]:
    """Reject a torque_ff ladder that is unsafe or unrepresentable. Empty list = OK.

    Three independent limits, each with a distinct failure mode:

    * ``cap_Nm`` (0.15 Nm) — the harness's own cap. Beyond it the feedforward starts to
      be a meaningful fraction of what the position loop can resist, and the "the loop
      holds station so the shift IS the measurement" argument weakens.
    * ``ODRIVE_TORQUE_SOFT_MAX_NM`` (1.5 Nm, json:53) — the ODrive's own torque limit.
      Independent of ``torque_constant`` (operator fact #3).
    * ``LEG_TOR_WIRE_MAX_NM`` (±3.2767 Nm) — the int16 ×10000 wire range; past it the
      value SILENTLY SATURATES at the int16 clamp (``odrive_protocol.h:178``) and the
      commanded value is not the delivered one.
    """
    problems: List[str] = []
    for tff in ladder_Nm:
        a = abs(float(tff))
        if a > cap_Nm:
            problems.append(
                f"torque_ff {tff:+.3f} Nm exceeds the harness cap ±{cap_Nm} Nm — too "
                f"large to be safely absorbed by the position loop")
        if a > ODRIVE_TORQUE_SOFT_MAX_NM:
            problems.append(
                f"torque_ff {tff:+.3f} Nm exceeds the ODrive torque_soft_max "
                f"{ODRIVE_TORQUE_SOFT_MAX_NM} Nm (odrive_pro_leg_config.json:53)")
        if a > LEG_TOR_WIRE_MAX_NM:
            problems.append(
                f"torque_ff {tff:+.3f} Nm exceeds the int16 wire range "
                f"±{LEG_TOR_WIRE_MAX_NM:.4f} Nm — it would SILENTLY SATURATE "
                f"(odrive_protocol.h:178)")
    return problems


def torque_ff_ladder_band_warnings(ladder_Nm: Sequence[float], *,
                                   warn_Nm: float = TFF_BAND_WARN_NM) -> List[str]:
    """WARN (never refuse) about ladder rungs that leave the static-friction band.

    The Mode-2 premise — the position loop holds station, so the iq shift IS the
    measurement — only holds while ``|torque_ff|`` stays under the static friction
    torque (worst-case edge ``τ_c·Kt`` ≈ :data:`TFF_STATIC_BAND_MIN_NM` = 0.049 Nm):
    beyond it the leg actually moves and the position loop's integrator re-absorbs the
    feedforward, biasing the fitted slope low (band model 33–52 %; observed 63–68 % on the 2026-07-14 ±0.10 Nm rungs).
    A warning, not a rejection, because an out-of-band rung is safe — it is merely a
    biased data point — and a deliberate band-mapping run is a legitimate experiment.
    """
    warns: List[str] = []
    for tff in ladder_Nm:
        if abs(float(tff)) > warn_Nm:
            warns.append(
                f"rung {float(tff):+.3f} Nm exceeds {warn_Nm:.3f} Nm and can leave the "
                f"static-friction band (worst-case edge {TFF_STATIC_BAND_MIN_NM:.3f} "
                f"Nm): the leg moves and the position loop's integrator re-absorbs the "
                f"feedforward, so this rung will bias the slope LOW instead of "
                f"measuring the channel")
    return warns


class OverCurrentLatch:
    """Debounced over-current trip: fires only on ``n_consecutive`` FINITE samples over
    ``limit_A`` (~12 ms at 250 Hz for the default 3), then latches.

    A single 250 Hz telemetry sample over the threshold must not abort a run — the
    accel transient of a traverse and ordinary current-loop ripple both produce
    one-sample excursions, and the old single-sample trip could kill a healthy run
    (and, pre-fix, its abort path then failed to park the leg). Below-limit samples
    reset the streak; non-finite samples (dropped/NaN telemetry) neither count nor
    reset. Pure — the harness feeds it ``iq_measured`` from ``_log_frame``.

    ``max_abs_A`` tracks the largest |iq| EVER observed (tripped or not), so an abort
    report can state the measured breakaway peak against the per-mass current budget
    even when the telemetry-snapshot fallback is all-NaN — the 2026-07-14 2.75 kg
    approach aborts printed "NONE @ u0=+nan enc=+nan" because the REAL trigger (this
    latch) was swallowed by that fallback.
    """

    def __init__(self, limit_A: float, n_consecutive: int = OVERCURRENT_TRIP_SAMPLES):
        if limit_A <= 0.0:
            raise ValueError("limit_A must be positive")
        if n_consecutive < 1:
            raise ValueError("n_consecutive must be >= 1")
        self.limit_A = float(limit_A)
        self.n_consecutive = int(n_consecutive)
        self.count = 0
        self.tripped = False
        self.max_abs_A = 0.0        # largest |iq| ever observed (finite samples only)

    def observe(self, iq_A: float) -> bool:
        """Feed one sample; returns True once tripped (and stays True)."""
        v = float(iq_A)
        if math.isfinite(v) and abs(v) > self.max_abs_A:
            self.max_abs_A = abs(v)
        if self.tripped:
            return True
        if not math.isfinite(v):
            return False
        if abs(v) > self.limit_A:
            self.count += 1
            if self.count >= self.n_consecutive:
                self.tripped = True
        else:
            self.count = 0
        return self.tripped

    def describe_trip(self) -> str:
        """The actual-trigger sentence an abort report must carry: which detector
        fired, with its values — never lost behind a NaN telemetry snapshot."""
        if not self.tripped:
            return (f"over-current latch NOT tripped (max |iq| seen "
                    f"{self.max_abs_A:.2f} A vs limit {self.limit_A:.2f} A)")
        return (f"over-current latch: {self.n_consecutive} consecutive samples ≥ "
                f"{self.limit_A:.2f} A, max {self.max_abs_A:.2f} A")


def wire_quantized_torque_Nm(torque_ff_Nm: float) -> float:
    """The torque_ff value that actually reaches the ODrive after the int16 ×10000 wire
    quantization (``odrive_protocol.h:175-179``).

    At 0.0001 Nm/LSB the quantization error is ≤ 0.00005 Nm — 0.5 % of the smallest
    (0.010 Nm) ladder rung, hence negligible. Computed anyway so the fit regresses against
    what was DELIVERED rather than what was asked for, and so a future ladder that gets
    too fine to represent is caught rather than silently rounded.
    """
    counts = round(float(torque_ff_Nm) * LEG_TOR_WIRE_SCALE)
    counts = max(-32768, min(32767, int(counts)))
    return counts / LEG_TOR_WIRE_SCALE


# ===========================================================================
# Mode/flag validation (A1 — the silent --hold-mass-in-mode-kt trap)
# ===========================================================================
#
# On 2026-07-14 the operator ran ``--mode kt --hold-mass 0.5/1.0/1.5/2.25`` four times.
# ``--hold-mass`` is a Mode-2 flag; Mode 1 reads ``--masses`` and silently fell back to
# its 1.0-first recommended ladder — so every run's data was labelled "1.00 kg" while
# the real hanging mass differed. A silently-ignored flag on a hardware harness is a
# data-corruption bug, not a usability nit: the fix is to REFUSE the run at parse time.

# argparse dest → flag string, per mode. Keys are the argparse ``dest`` names.
MODE1_ONLY_FLAGS: Dict[str, str] = {
    'masses': '--masses',
    'traverse_lo': '--traverse-lo',
    'traverse_hi': '--traverse-hi',
    'vel': '--vel',
    'reps': '--reps',
    'dwell': '--dwell',
    'edge_discard': '--edge-discard',
    'accel_time': '--accel-time',
}
MODE2_ONLY_FLAGS: Dict[str, str] = {
    'hold_mass': '--hold-mass',
    'tff_hold': '--tff-hold',
    'tff_amps': '--tff-amps',
    'tff_half_period': '--tff-half-period',
    'tff_cycles': '--tff-cycles',
    'pre_soak': '--pre-soak',
}


def validate_mode_flags(mode: str, provided: Sequence[str]) -> List[str]:
    """Refuse mode-mismatched flags instead of silently ignoring them. Empty list = OK.

    ``mode`` is the ``--mode`` choice ('kt' / 'torque_ff_check' / 'all'); ``provided``
    is the collection of argparse *dest* names the user EXPLICITLY set (detected via
    ``default=None`` sentinels). ``--mode all`` accepts every flag.

    Also enforces the REQUIRED ``--rig-orientation`` declaration (no default): the
    2026-07-14 manifests recorded ``extension_iq_sign`` wrong-way-round because the rig
    was inverted and the harness assumed holding == extension. The sign inference
    cannot be trusted without the operator stating which way the rig is rigged.
    """
    if mode not in ('kt', 'torque_ff_check', 'all'):
        return [f"unknown mode '{mode}'"]
    given = set(provided)
    problems: List[str] = []
    if mode == 'kt':
        for dest in sorted(MODE2_ONLY_FLAGS):
            if dest in given:
                problems.append(
                    f"{MODE2_ONLY_FLAGS[dest]} applies only to --mode torque_ff_check "
                    f"and would be SILENTLY IGNORED by --mode kt — refusing (this exact "
                    f"silent ignore mislabelled every 2026-07-14 kt run as 1.00 kg). "
                    f"Declare the traverse masses with --masses.")
    elif mode == 'torque_ff_check':
        for dest in sorted(MODE1_ONLY_FLAGS):
            if dest in given:
                problems.append(
                    f"{MODE1_ONLY_FLAGS[dest]} applies only to --mode kt and would be "
                    f"SILENTLY IGNORED by --mode torque_ff_check — refusing. Declare "
                    f"the held mass with --hold-mass.")
    if 'rig_orientation' not in given:
        problems.append(
            "--rig-orientation {normal,inverted} is REQUIRED (no default): "
            "'normal' = leg EXTENSION raises the load; 'inverted' = leg CONTRACTION "
            "raises it. It feeds the extension_iq_sign inference — the 2026-07-14 "
            "manifests recorded the sign wrong-way-round because the rig was inverted "
            "and the harness assumed holding == extension.")
    return problems


# ===========================================================================
# EDGE CAPTURE — the Mode-2 measurement pipeline (pure math)
# ===========================================================================
#
# See the MODE 2 section comment above for the physics. Everything here is pure
# array-in / dataclass-out so the whole pipeline is exercised end-to-end on synthetic
# 250 Hz traces in tests/motion/test_kt_lib.py (known slope 18.14 + τ = 8 s decay +
# noise σ = 0.08 A must come back 18.14 ± 1; a null channel must yield a PRECISE-NULL
# verdict; a drifting hold must yield NO CONCLUSION with channel_live=None).


def square_wave_tff_series(amplitude_Nm: float, *, seg_t_s: float,
                           half_period_s: float = DEFAULT_TFF_HALF_PERIOD_S,
                           n_cycles: int = DEFAULT_TFF_CYCLES
                           ) -> Tuple[np.ndarray, np.ndarray]:
    """Per-knot torque_ff values for the ± square wave, plus the commanded toggle times.

    The wave starts at ``+amplitude`` (the 0→+X onset is NOT a measured edge — it is
    half-sized and rides the settle transient) and toggles every ``half_period_s``.
    Returns ``(values, toggle_times_s)`` where ``values[i]`` is the torque_ff to stream
    with knot ``i`` and ``toggle_times_s`` are the ``2·n_cycles − 1`` full ±2X edges,
    measured from the first knot of the wave. Toggle k (1-based) has polarity
    ``(−1)**k`` — see :func:`toggle_polarities`.
    """
    if amplitude_Nm <= 0.0:
        raise ValueError("amplitude must be positive")
    if seg_t_s <= 0.0 or half_period_s <= 0.0:
        raise ValueError("seg_t_s and half_period_s must be > 0")
    if n_cycles < 1:
        raise ValueError("n_cycles must be >= 1")
    k_half = max(1, int(round(half_period_s / seg_t_s)))
    n_half = 2 * int(n_cycles)
    vals = np.empty(k_half * n_half, float)
    for h in range(n_half):
        vals[h * k_half:(h + 1) * k_half] = (amplitude_Nm if h % 2 == 0
                                             else -amplitude_Nm)
    toggles = np.array([h * k_half * seg_t_s for h in range(1, n_half)], float)
    return vals, toggles


def toggle_polarities(n_toggles: int) -> np.ndarray:
    """Δtorque_ff sign per toggle: the wave starts at +X, so toggle 1 is +X→−X
    (Δ = −2X, polarity −1), toggle 2 is +1, alternating: polarity_k = (−1)^k."""
    return np.array([(-1) ** k for k in range(1, int(n_toggles) + 1)], int)


@dataclass
class SettleCheck:
    settled: bool
    pos_err_rev: float           # |mean(pos) − cmd| over the window
    diq_dt_A_per_s: float        # fitted linear iq drift over the window
    window_s: float
    n_samples: int
    reasons: List[str] = field(default_factory=list)


def hold_settled(t_s: Sequence[float], pos_rev: Sequence[float],
                 iq_A: Sequence[float], *, cmd_rev: float,
                 pos_tol_rev: float = SETTLE_POS_TOL_REV,
                 diq_dt_max_A_per_s: float = SETTLE_DIQ_DT_MAX_A_PER_S,
                 window_s: float = SETTLE_WINDOW_S,
                 min_samples: int = 100) -> SettleCheck:
    """The settle gate before any edge-capture measurement.

    Settled iff, over the trailing ``window_s`` of the record: the mean position is
    within ``pos_tol_rev`` of the commanded hold AND the fitted linear iq drift is
    under ``diq_dt_max_A_per_s``. The 2026-07-14 holds needed ~10 s post-approach
    before d(iq)/dt fell to the noise floor (the friction-band load-transfer transient:
    ~1.2–1.4 A amplitude, τ ≈ 3.2 s) — measuring before that poisons every edge with a
    drift bias.
    """
    t = np.asarray(t_s, float)
    p = np.asarray(pos_rev, float)
    iq = np.asarray(iq_A, float)
    n = int(min(t.size, p.size, iq.size))
    if n == 0:
        return SettleCheck(False, float('nan'), float('nan'), window_s, 0,
                           ['no samples'])
    t, p, iq = t[:n], p[:n], iq[:n]
    finite = np.isfinite(t) & np.isfinite(p) & np.isfinite(iq)
    t, p, iq = t[finite], p[finite], iq[finite]
    if t.size == 0:
        return SettleCheck(False, float('nan'), float('nan'), window_s, 0,
                           ['no finite samples'])
    sel = t >= (float(np.max(t)) - float(window_s))
    t, p, iq = t[sel], p[sel], iq[sel]
    reasons: List[str] = []
    if t.size < min_samples:
        reasons.append(f"only {t.size} samples in the {window_s:.1f} s settle window "
                       f"(< {min_samples})")
        return SettleCheck(False, float('nan'), float('nan'), window_s, int(t.size),
                           reasons)
    pos_err = abs(float(np.mean(p)) - float(cmd_rev))
    span = float(np.max(t) - np.min(t))
    if span <= 0.0:
        return SettleCheck(False, pos_err, float('nan'), window_s, int(t.size),
                           ['zero time span in the settle window'])
    diq_dt = float(np.polyfit(t, iq, 1)[0])
    if pos_err > pos_tol_rev:
        reasons.append(f"pos is {pos_err * 1e3:.2f} mrev from cmd "
                       f"(> {pos_tol_rev * 1e3:.2f} mrev) — the hold has not converged")
    if abs(diq_dt) > diq_dt_max_A_per_s:
        reasons.append(f"|d(iq)/dt| = {abs(diq_dt):.3f} A/s "
                       f"(> {diq_dt_max_A_per_s:.3f}) — the hold current is still "
                       f"re-absorbing (load-transfer / integrator transient)")
    return SettleCheck(settled=not reasons, pos_err_rev=pos_err,
                       diq_dt_A_per_s=diq_dt, window_s=window_s,
                       n_samples=int(t.size), reasons=reasons)


def locate_edge_time(t_s: Sequence[float], iq_A: Sequence[float], t_cmd_s: float, *,
                     polarity: Optional[int] = None,
                     search_s: float = EDGE_SEARCH_S,
                     kernel_s: float = EDGE_KERNEL_S) -> Optional[float]:
    """Matched-filter edge localization: find the ACTUAL toggle instant near the
    commanded one.

    Slides a step kernel (−1 before the candidate instant, +1 after, ``kernel_s`` each
    side) across candidates within ``±search_s`` of ``t_cmd_s``. With ``polarity``
    given, returns the instant maximizing ``polarity · (mean_after − mean_before)``;
    with ``polarity=None`` (what the pipeline uses) it maximizes the MAGNITUDE
    ``|mean_after − mean_before|`` — the sign of the iq response is
    ``sign(slope) · polarity`` and the slope sign is precisely what the run measures,
    so the locator must not presume it. Wall-clock alone is not trusted: the commanded
    toggle time is the send-loop's schedule, while the edge in the iq stream lands
    after knot transport + ODrive application + telemetry latency (≲ tens of ms, but
    the jump windows are only 20 ms wide at the near edge, so locating matters).

    Returns None when there is not enough finite data around the command to score any
    candidate (≥5 samples each side required).
    """
    t = np.asarray(t_s, float)
    iq = np.asarray(iq_A, float)
    n = int(min(t.size, iq.size))
    if n == 0:
        return None
    t, iq = t[:n], iq[:n]
    finite = np.isfinite(t) & np.isfinite(iq)
    t, iq = t[finite], iq[finite]
    if t.size < 10:
        return None
    order = np.argsort(t, kind='stable')
    t, iq = t[order], iq[order]
    csum = np.concatenate(([0.0], np.cumsum(iq)))

    def win_mean(a: float, b: float) -> Optional[float]:
        i0 = int(np.searchsorted(t, a, side='left'))
        i1 = int(np.searchsorted(t, b, side='right'))
        if i1 - i0 < 5:
            return None
        return (csum[i1] - csum[i0]) / (i1 - i0)

    lo = int(np.searchsorted(t, t_cmd_s - search_s, side='left'))
    hi = int(np.searchsorted(t, t_cmd_s + search_s, side='right'))
    best_t: Optional[float] = None
    best_score = -math.inf
    for i in range(lo, hi):
        tau = float(t[i])
        pre = win_mean(tau - kernel_s, tau - 1e-9)
        post = win_mean(tau + 1e-9, tau + kernel_s)
        if pre is None or post is None:
            continue
        step = post - pre
        score = float(polarity) * step if polarity is not None else abs(step)
        if score > best_score:
            best_score = score
            best_t = tau
    return best_t


def estimate_response_lag(t_s: Sequence[float], iq_A: Sequence[float],
                          toggle_times: Sequence[float], *,
                          search_s: float = EDGE_SEARCH_S,
                          kernel_s: float = EDGE_KERNEL_S) -> Optional[float]:
    """The single response lag shared by every edge of one amplitude: the MEDIAN of
    the per-edge matched-filter offsets ``t_located − t_commanded``.

    The lag is SYSTEMIC — knot transport + ODrive application + telemetry latency —
    so all edges of a hold share one value. Using the median of per-edge locations
    (rather than letting each edge keep its own argmax) buys two things the per-edge
    version cannot give:

    * **robustness** — a knocked/disturbed edge mislocates ITS OWN argmax, but cannot
      drag the median of ~19; and
    * **an unbiased null** — on a DEAD channel each per-edge argmax would lock onto
      the largest noise excursion in its ±0.1 s window, systematically inflating
      |jump| and blocking the precise-null verdict; a fixed shared offset samples the
      windows at a noise-independent instant, so null jumps stay centred on zero.

    Returns None when no edge is locatable at all.
    """
    offsets: List[float] = []
    for t_cmd in toggle_times:
        t_e = locate_edge_time(t_s, iq_A, float(t_cmd), polarity=None,
                               search_s=search_s, kernel_s=kernel_s)
        if t_e is not None:
            offsets.append(t_e - float(t_cmd))
    if not offsets:
        return None
    return float(np.median(offsets))


def edge_decay_correction(tau_s: float, *, half_period_s: float,
                          win_lo_s: float = EDGE_WINDOW_LO_S,
                          win_hi_s: float = EDGE_WINDOW_HI_S) -> float:
    """The fraction of the true edge jump the windowed raw measurement captures, under
    the periodic square-wave re-absorption model. Divide the raw jump by this.

    Model: after each edge the loop re-absorbs the injected step exponentially with
    time constant ``tau_s`` toward the load line (the steady-state response to a
    constant torque_ff is ZERO), so in periodic steady state the response is
    ``±A·e^{−u/τ}`` with ``A = ΔI / (1 + e^{−hp/τ})``. The post-edge window
    ``[win_lo, win_hi]`` reads the decayed mean ``A·g``; the pre-edge window sits
    ``hp − win_hi .. hp − win_lo`` after the PREVIOUS (opposite) edge and reads
    ``−A·h``. The raw jump is therefore ``A·(g + h)`` against a true ``ΔI``:

        correction = (g + h) / (1 + e^{−hp/τ})

    where ``g``/``h`` are the window averages of ``e^{−u/τ}``. For τ = 8 s and the
    default windows the pre-window creep almost exactly compensates the post-window
    creep (correction ≈ 0.999); at τ = 2 s it is ≈ 0.979. Uncorrected single-sided
    creep would cost 2.5–9.5 % at 0.2 s over the τ = 2–10 s range — hence the
    correction. Returns 1.0 for a non-finite/absent τ; τ is clamped to
    ``MIN_HOLD_TAU_S`` below (a hold re-absorbing faster than that invalidates the
    method outright, and an unclamped tiny τ would explode the corrected jump).
    """
    if half_period_s <= 0.0:
        raise ValueError("half_period_s must be > 0")
    if not (win_hi_s > win_lo_s >= 0.0) or win_hi_s >= half_period_s:
        raise ValueError("jump windows must satisfy 0 <= lo < hi < half_period")
    if not math.isfinite(tau_s) or tau_s <= 0.0:
        return 1.0
    tau = max(float(tau_s), MIN_HOLD_TAU_S)

    def wavg(a: float, b: float) -> float:
        return tau / (b - a) * (math.exp(-a / tau) - math.exp(-b / tau))

    g = wavg(win_lo_s, win_hi_s)
    h = wavg(half_period_s - win_hi_s, half_period_s - win_lo_s)
    corr = (g + h) / (1.0 + math.exp(-half_period_s / tau))
    return float(min(1.0, max(1e-3, corr)))


def average_edge_decay(t_s: Sequence[float], iq_A: Sequence[float],
                       edge_times: Sequence[float], polarities: Sequence[int], *,
                       dur_s: float, skip_s: float = EDGE_WINDOW_LO_S,
                       grid_dt_s: float = 0.008
                       ) -> Tuple[np.ndarray, np.ndarray]:
    """Polarity-corrected, per-segment-mean-subtracted decay curve averaged across
    edges — the input :func:`fit_hold_tau` wants.

    For each edge, iq over ``[t_e + skip, t_e + dur]`` is interpolated onto a common
    grid, multiplied by the edge polarity (so every segment decays the same way up),
    and mean-subtracted (each segment rides a different baseline; the τ estimator is
    offset-invariant so only the shape matters). Returns ``(u, ybar)``.
    """
    t = np.asarray(t_s, float)
    iq = np.asarray(iq_A, float)
    finite = np.isfinite(t) & np.isfinite(iq)
    t, iq = t[finite], iq[finite]
    u = np.arange(skip_s, dur_s, grid_dt_s)
    if t.size < 10 or u.size < 6:
        return u, np.full(u.size, np.nan)
    order = np.argsort(t, kind='stable')
    t, iq = t[order], iq[order]
    segs = []
    for t_e, pol in zip(edge_times, polarities):
        if t_e is None or not math.isfinite(float(t_e)):
            continue
        tt = float(t_e) + u
        if tt[0] < t[0] or tt[-1] > t[-1]:
            continue
        y = np.interp(tt, t, iq) * float(pol)
        segs.append(y - float(np.mean(y)))
    if not segs:
        return u, np.full(u.size, np.nan)
    return u, np.mean(np.vstack(segs), axis=0)


def fit_hold_tau(u_s: Sequence[float], y: Sequence[float], *,
                 default_tau_s: float = DEFAULT_HOLD_TAU_S
                 ) -> Tuple[float, bool]:
    """Estimate the re-absorption time constant τ of ``y ≈ A·e^{−u/τ} + c``.

    Three-window ratio estimator: split the record into three equal contiguous
    windows; for a (possibly offset) exponential the window means satisfy
    ``(m1 − m2)/(m2 − m3) = e^{w/τ}`` exactly, independent of both A and c — robust,
    closed-form, and immune to the baseline. Returns ``(tau, fitted)``;
    ``(default_tau_s, False)`` whenever the decay is not resolvable (noise-dominated,
    wrong-signed differences, absurd τ) — the decay correction is ≤2 % across the
    plausible τ range, so a conservative default beats a wild fit.
    """
    u = np.asarray(u_s, float)
    yy = np.asarray(y, float)
    n = int(min(u.size, yy.size))
    if n < 12:
        return float(default_tau_s), False
    u, yy = u[:n], yy[:n]
    finite = np.isfinite(u) & np.isfinite(yy)
    u, yy = u[finite], yy[finite]
    if u.size < 12:
        return float(default_tau_s), False
    span = float(np.max(u) - np.min(u))
    if span <= 0.0:
        return float(default_tau_s), False
    w = span / 3.0
    u0 = float(np.min(u))
    m = []
    for k in range(3):
        sel = (u >= u0 + k * w) & (u < u0 + (k + 1) * w + (1e-12 if k == 2 else 0.0))
        if int(np.count_nonzero(sel)) < 3:
            return float(default_tau_s), False
        m.append(float(np.mean(yy[sel])))
    d1 = m[0] - m[1]
    d2 = m[1] - m[2]
    if d1 <= 0.0 or d2 <= 0.0 or d1 <= d2:
        return float(default_tau_s), False
    tau = w / math.log(d1 / d2)
    if not math.isfinite(tau) or not (0.05 <= tau <= 300.0):
        return float(default_tau_s), False
    return float(tau), True


@dataclass
class EdgeJump:
    t_cmd_s: float               # commanded toggle time (stream clock)
    t_edge_s: float              # matched-filter located edge (NaN if not locatable)
    polarity: int                # sign of Δtorque_ff at this edge (±1)
    jump_raw_A: float            # windowed post − pre mean, uncorrected
    jump_A: float                # decay-corrected jump
    slope_A_per_Nm: float        # jump / Δtorque_ff — signed, comparable across edges
    ok: bool
    reason: str = ''


def measure_edge_jumps(t_s: Sequence[float], iq_A: Sequence[float],
                       toggle_times: Sequence[float], amplitude_Nm: float, *,
                       half_period_s: float, tau_s: float, lag_s: float = 0.0,
                       win_lo_s: float = EDGE_WINDOW_LO_S,
                       win_hi_s: float = EDGE_WINDOW_HI_S,
                       min_win_samples: int = 10) -> List[EdgeJump]:
    """Measure the decay-corrected jump at every commanded edge, with the windows
    placed at ``t_cmd + lag_s`` (the SHARED matched-filter lag from
    :func:`estimate_response_lag` — see there for why the lag is estimated once per
    hold rather than per edge). Per edge: ``slope = jump / (polarity · 2X)``."""
    if amplitude_Nm <= 0.0:
        raise ValueError("amplitude must be positive")
    t = np.asarray(t_s, float)
    iq = np.asarray(iq_A, float)
    n = int(min(t.size, iq.size))
    t, iq = t[:n], iq[:n]
    finite = np.isfinite(t) & np.isfinite(iq)
    t, iq = t[finite], iq[finite]
    order = np.argsort(t, kind='stable')
    t, iq = t[order], iq[order]
    csum = np.concatenate(([0.0], np.cumsum(iq)))

    def win_mean(a: float, b: float) -> Tuple[Optional[float], int]:
        i0 = int(np.searchsorted(t, a, side='left'))
        i1 = int(np.searchsorted(t, b, side='right'))
        cnt = i1 - i0
        if cnt < min_win_samples:
            return None, cnt
        return (csum[i1] - csum[i0]) / cnt, cnt

    corr = edge_decay_correction(tau_s, half_period_s=half_period_s,
                                 win_lo_s=win_lo_s, win_hi_s=win_hi_s)
    pols = toggle_polarities(len(list(toggle_times)))
    out: List[EdgeJump] = []
    for t_cmd, pol in zip(toggle_times, pols):
        t_e = float(t_cmd) + float(lag_s)
        pre, n_pre = win_mean(t_e - win_hi_s, t_e - win_lo_s)
        post, n_post = win_mean(t_e + win_lo_s, t_e + win_hi_s)
        if pre is None or post is None:
            out.append(EdgeJump(float(t_cmd), float(t_e), int(pol), float('nan'),
                                float('nan'), float('nan'), False,
                                f'too few window samples (pre {n_pre}, post {n_post})'))
            continue
        raw = post - pre
        jump = raw / corr
        slope = jump / (float(pol) * 2.0 * float(amplitude_Nm))
        out.append(EdgeJump(float(t_cmd), float(t_e), int(pol), float(raw),
                            float(jump), float(slope), True))
    return out


@dataclass
class EdgeCaptureAmplitude:
    amplitude_Nm: float
    half_period_s: float
    n_toggles: int
    n_measured: int              # edges the matched filter + windows could measure
    n_kept: int                  # after the symmetric trim
    tau_s: float
    tau_fitted: bool
    slope_A_per_Nm: float        # trimmed mean of the per-edge signed slopes
    slope_sem_A_per_Nm: float
    jump_cv: float               # trimmed std/|mean| of the per-edge slopes
    drift_bias_A: float          # paired-edge mean of SIGNED raw jumps (signal cancels)
    drift_bias_sem_A: float
    drift_detected: bool
    edges: List[EdgeJump] = field(default_factory=list)
    reasons: List[str] = field(default_factory=list)


def analyze_edge_capture(t_s: Sequence[float], iq_A: Sequence[float],
                         toggle_times: Sequence[float], amplitude_Nm: float, *,
                         half_period_s: float,
                         default_tau_s: float = DEFAULT_HOLD_TAU_S,
                         trim_frac: float = EDGE_TRIM_FRAC,
                         drift_bias_min_A: float = EDGE_DRIFT_BIAS_MIN_A
                         ) -> EdgeCaptureAmplitude:
    """The per-amplitude edge-capture analysis: locate → fit τ → jumps → statistics.

    * **Trim**: the per-edge slopes are sorted and a symmetric ``trim_frac`` is dropped
      from EACH end (at least one from each end once n ≥ 5) — a single knocked/held
      edge must not drag the mean.
    * **Drift statistic**: consecutive opposite-polarity edges are paired; in each pair
      the SIGNAL contributes equal-and-opposite raw jumps while a baseline drift
      contributes the SAME signed bias — so ``mean((j_k + j_{k+1})/2)`` isolates drift.
      This is what makes the square wave immune to the drift-aliasing that produced
      the 2026-07-14 20:55 −21 A/Nm ladder artifact (there, time ⊥ tff were collinear
      at the fixed rung cadence). Fires when the bias clears BOTH 3σ of its own SEM
      and ``drift_bias_min_A``.
    """
    toggle_times = [float(x) for x in toggle_times]
    tau = float(default_tau_s)
    fitted = False
    reasons: List[str] = []

    # One SHARED response lag for the whole hold (median matched-filter offset —
    # see estimate_response_lag for why per-edge argmaxes are not used), then τ from
    # the lag-aligned edges (alignment does not need τ; the jump correction does).
    pols = toggle_polarities(len(toggle_times))
    lag = estimate_response_lag(t_s, iq_A, toggle_times)
    if lag is not None:
        aligned = [tc + lag for tc in toggle_times]
        u, ybar = average_edge_decay(
            t_s, iq_A, aligned, pols,
            dur_s=max(0.3, half_period_s - 0.05))
        # The polarity-corrected curve decays downward for a positive slope and
        # upward for a negative one (the slope sign is what the run measures, so
        # neither orientation may be presumed) — try both.
        tau, fitted = fit_hold_tau(u, ybar, default_tau_s=default_tau_s)
        if not fitted:
            tau, fitted = fit_hold_tau(u, -np.asarray(ybar, float),
                                       default_tau_s=default_tau_s)

    edges = measure_edge_jumps(t_s, iq_A, toggle_times, amplitude_Nm,
                               half_period_s=half_period_s, tau_s=tau,
                               lag_s=lag if lag is not None else 0.0)
    ok_edges = [e for e in edges if e.ok]
    n_meas = len(ok_edges)
    if n_meas == 0:
        return EdgeCaptureAmplitude(
            amplitude_Nm=float(amplitude_Nm), half_period_s=float(half_period_s),
            n_toggles=len(toggle_times), n_measured=0, n_kept=0, tau_s=tau,
            tau_fitted=fitted, slope_A_per_Nm=float('nan'),
            slope_sem_A_per_Nm=float('inf'), jump_cv=float('inf'),
            drift_bias_A=float('nan'), drift_bias_sem_A=float('inf'),
            drift_detected=False, edges=edges,
            reasons=['no measurable edges — no telemetry around any commanded toggle'])

    slopes = np.array(sorted(e.slope_A_per_Nm for e in ok_edges), float)
    k = int(trim_frac * n_meas)
    if n_meas >= 5:
        k = max(1, k)
    trimmed = slopes[k:n_meas - k] if n_meas - 2 * k >= 2 else slopes
    n_kept = int(trimmed.size)
    slope = float(np.mean(trimmed))
    std = float(np.std(trimmed, ddof=1)) if n_kept > 1 else float('inf')
    sem = std / math.sqrt(n_kept) if n_kept > 0 else float('inf')
    cv = (std / abs(slope)) if abs(slope) > 0 and math.isfinite(std) else float('inf')

    # Paired-edge drift statistic on the SIGNED RAW jumps, in edge order.
    raw = [e.jump_raw_A for e in edges if e.ok]
    pairs = [(raw[i] + raw[i + 1]) / 2.0 for i in range(0, len(raw) - 1, 2)]
    if len(pairs) >= 3:
        bias = float(np.mean(pairs))
        bias_sem = float(np.std(pairs, ddof=1) / math.sqrt(len(pairs)))
        drift = (abs(bias) > 3.0 * bias_sem) and (abs(bias) > drift_bias_min_A)
    else:
        bias, bias_sem, drift = float('nan'), float('inf'), False
    if drift:
        reasons.append(
            f"hold DRIFTING under the square wave: paired-edge bias "
            f"{bias:+.3f} ± {bias_sem:.3f} A per edge (> {drift_bias_min_A:.3f} A and "
            f"> 3σ) — the settle gate should have caught this; the jumps are "
            f"contaminated and no verdict may be stated from this amplitude")
    if n_meas < len(toggle_times):
        reasons.append(f"only {n_meas}/{len(toggle_times)} commanded edges were "
                       f"measurable")
    return EdgeCaptureAmplitude(
        amplitude_Nm=float(amplitude_Nm), half_period_s=float(half_period_s),
        n_toggles=len(toggle_times), n_measured=n_meas, n_kept=n_kept, tau_s=tau,
        tau_fitted=fitted, slope_A_per_Nm=slope, slope_sem_A_per_Nm=float(sem),
        jump_cv=float(cv), drift_bias_A=bias, drift_bias_sem_A=bias_sem,
        drift_detected=drift, edges=edges, reasons=reasons)


@dataclass
class EdgeCaptureResult:
    per_amplitude: List[EdgeCaptureAmplitude]
    slope_A_per_Nm: float        # pooled (inverse-variance weighted across amplitudes)
    slope_sem_A_per_Nm: float
    t_stat: float                # |slope| / sem
    jump_cv: float               # CV over ALL trimmed per-edge slopes, pooled
    n_edges_kept: int
    drift_detected: bool
    trustworthy: bool            # t ≥ 3 AND cv ≤ 0.20 AND no drift AND enough edges
    failures: List[str] = field(default_factory=list)


def pool_edge_capture(per_amplitude: Sequence[EdgeCaptureAmplitude], *,
                      cv_max: float = TFF_EDGE_CV_MAX,
                      min_t_stat: float = TFF_GATE_MIN_T_STAT,
                      min_edges: int = MIN_EDGES_FOR_VERDICT) -> EdgeCaptureResult:
    """Pool the per-amplitude slopes and apply the edge-capture trustworthiness gate:
    ``|slope|/σ ≥ 3`` AND trimmed edge-jump CV ≤ 20 % (the R² analogue) AND no drift
    AND ≥ ``min_edges`` kept edges. Any failure ⇒ no verdict may be stated."""
    amps = [a for a in per_amplitude]
    failures: List[str] = []
    usable = [a for a in amps
              if a.n_kept >= 2 and math.isfinite(a.slope_A_per_Nm)
              and math.isfinite(a.slope_sem_A_per_Nm) and a.slope_sem_A_per_Nm >= 0]
    n_edges = sum(a.n_kept for a in usable)
    drift = any(a.drift_detected for a in amps)
    if drift:
        for a in amps:
            if a.drift_detected:
                failures.append(
                    f"X = {a.amplitude_Nm:.3f} Nm: hold drifting (paired-edge bias "
                    f"{a.drift_bias_A:+.3f} ± {a.drift_bias_sem_A:.3f} A) — the "
                    f"square-wave de-aliasing detected exactly the failure mode that "
                    f"faked the 2026-07-14 20:55 ladder slope")
    if not usable or n_edges < min_edges:
        failures.append(f"only {n_edges} kept edges across all amplitudes "
                        f"(< {min_edges}) — not enough to state anything")
        return EdgeCaptureResult(per_amplitude=list(amps),
                                 slope_A_per_Nm=float('nan'),
                                 slope_sem_A_per_Nm=float('inf'), t_stat=0.0,
                                 jump_cv=float('inf'), n_edges_kept=n_edges,
                                 drift_detected=drift, trustworthy=False,
                                 failures=failures)
    w = np.array([1.0 / max(a.slope_sem_A_per_Nm, 1e-9) ** 2 for a in usable], float)
    s = np.array([a.slope_A_per_Nm for a in usable], float)
    slope = float(np.sum(w * s) / np.sum(w))
    sem = float(1.0 / math.sqrt(float(np.sum(w))))
    t = abs(slope) / sem if sem > 0 else math.inf

    all_slopes: List[float] = []
    for a in usable:
        ok_sorted = np.array(sorted(e.slope_A_per_Nm for e in a.edges if e.ok), float)
        k = ok_sorted.size - a.n_kept
        lo = k // 2
        all_slopes.extend(ok_sorted[lo:lo + a.n_kept].tolist())
    arr = np.array(all_slopes, float)
    cv = (float(np.std(arr, ddof=1)) / abs(float(np.mean(arr)))
          if arr.size > 1 and abs(float(np.mean(arr))) > 0 else float('inf'))

    if t < min_t_stat:
        failures.append(f"|slope|/σ = {t:.1f} (< {min_t_stat:.0f}) — the pooled slope "
                        f"{slope:+.2f} ± {sem:.2f} A/Nm is statistically "
                        f"indistinguishable from no response")
    if not math.isfinite(cv) or cv > cv_max:
        failures.append(f"trimmed edge-jump CV = "
                        f"{cv * 100 if math.isfinite(cv) else float('inf'):.0f}% "
                        f"(> {cv_max * 100:.0f}%) — the edges do not tell one "
                        f"consistent story")
    return EdgeCaptureResult(per_amplitude=list(amps), slope_A_per_Nm=slope,
                             slope_sem_A_per_Nm=sem, t_stat=float(t),
                             jump_cv=float(cv), n_edges_kept=n_edges,
                             drift_detected=drift, trustworthy=not failures,
                             failures=failures)


def _edge_no_conclusion_lines(result: EdgeCaptureResult) -> List[str]:
    lines = [
        "*** NO CONCLUSION — edge-capture data quality insufficient to validate the "
        "torque_ff channel ***",
        f"    verdict gate: |slope|/σ ≥ {TFF_GATE_MIN_T_STAT:.0f} AND trimmed "
        f"edge-jump CV ≤ {TFF_EDGE_CV_MAX * 100:.0f}% AND no drift required before "
        f"any SIGN/SCALE claim.",
    ]
    lines.extend(f"    FAIL: {f}" for f in result.failures)
    lines.append("    Refusing to state SIGN or SCALE — a verdict here would be noise "
                 "dressed as a finding.")
    lines.append("    channel_live is recorded as None (UNKNOWN), not False: this run "
                 "proves neither liveness nor death.")
    lines.append("    Required setup: mass hangs FREE and VERTICAL; NOBODY touches "
                 "the rig during the toggles;")
    lines.append("    let the settle gate pass (pos ±0.3 mrev of cmd, |d(iq)/dt| < "
                 "0.05 A/s) before measuring.")
    return lines


def classify_edge_capture(result: EdgeCaptureResult, iq_extension_sign: int, *,
                          scale_tol_frac: float = 0.10) -> TorqueFfVerdict:
    """The edge-capture verdict, stated honestly (reuses :class:`TorqueFfVerdict`).

    Gate order matters:

    1. **Fatal data-quality failures first** (drift / too few edges / non-finite
       slope) ⇒ NO CONCLUSION with ``channel_live=None`` — a drifting hold can fake a
       precise null, so the null test must not run on poisoned data.
    2. **Precise null**: ``|slope| + 3σ`` bounded under 5 % of the expected 18.14 A/Nm
       ⇒ the channel is genuinely DEAD (``channel_live=False``) — the only path
       allowed to say so.
    3. **Trust gate** (t ≥ 3 AND CV ≤ 20 %) ⇒ full SIGN/SCALE verdict
       (``channel_live=True``); otherwise NO CONCLUSION (None).

    SIGN: the tff channel sign was SETTLED on 2026-07-14 — a positive wire torque_ff
    EXTENDS through the production chain, no negation anywhere. A RETRACTS result here
    therefore contradicts a settled finding: suspect the ``--rig-orientation``
    declaration (which the extension sign is inferred through) before the wire.
    """
    expected = 1.0 / KT_ODRIVE_CONFIGURED
    s = result.slope_A_per_Nm
    sem = result.slope_sem_A_per_Nm

    fatal = (result.drift_detected
             or result.n_edges_kept < MIN_EDGES_FOR_VERDICT
             or not math.isfinite(s))
    if fatal:
        return TorqueFfVerdict(
            channel_live=None, positive_tff_extends=None,
            sign_matches_expectation=None, scale_ok=False,
            implied_kt_nm_per_a=float('nan'), sigma_to_odrive_config=float('inf'),
            lines=_edge_no_conclusion_lines(result), no_conclusion=True)

    precise_null = (math.isfinite(sem)
                    and abs(s) + 3.0 * sem < 0.05 * expected)
    if precise_null:
        lines = [
            "CHANNEL DEAD — the square-wave edges produced no current jump (the "
            f"pooled slope is bounded below {0.05 * expected:.2f} A/Nm at 3σ, from "
            f"{result.n_edges_kept} clean edges).",
            "DO NOT ship a torque feedforward until this is resolved. Check: is the "
            "leg mid-stroke? (leg_interp.cpp:407 stroke clamp ZEROES torque_ff; :479 "
            "recovery slew too).",
        ]
        return TorqueFfVerdict(
            channel_live=False, positive_tff_extends=None,
            sign_matches_expectation=None, scale_ok=False,
            implied_kt_nm_per_a=float('nan'), sigma_to_odrive_config=float('inf'),
            lines=lines)

    if not result.trustworthy:
        return TorqueFfVerdict(
            channel_live=None, positive_tff_extends=None,
            sign_matches_expectation=None, scale_ok=False,
            implied_kt_nm_per_a=(1.0 / abs(s)) if abs(s) > 0 else float('nan'),
            sigma_to_odrive_config=float('inf'),
            lines=_edge_no_conclusion_lines(result), no_conclusion=True)

    lines: List[str] = []
    extends: Optional[bool] = None
    matches: Optional[bool] = None
    if iq_extension_sign in (1, -1):
        extends = (s > 0) == (iq_extension_sign > 0)
        matches = extends is True
        lines.append(
            f"SIGN: a POSITIVE torque_ff on the wire "
            f"{'EXTENDS' if extends else 'RETRACTS'} this leg "
            f"[pooled slope {s:+.2f} ± {sem:.2f} A/Nm; extending torque reads as "
            f"{iq_extension_sign:+d} iq].")
        if extends:
            lines.append(
                "     CONFIRMS the settled 2026-07-14 finding: positive wire "
                "torque_ff = extension through the production chain, no negation "
                "needed anywhere.")
        else:
            lines.append(
                "     *** CONTRADICTS the settled 2026-07-14 production-chain "
                "finding (positive wire tff = extension). Before believing a wire "
                "sign flip, re-check the --rig-orientation declaration — the "
                "extension sign is inferred THROUGH it, and an inverted-vs-normal "
                "mix-up produces exactly this contradiction. ***")
    else:
        lines.append("SIGN: indeterminate — no gravity-calibrated iq extension sign "
                     "available (load the leg, or run --mode kt first).")

    err = abs(abs(s) - expected) / expected
    scale_ok = err <= scale_tol_frac
    implied = 1.0 / abs(s)
    implied_sig = implied * (sem / abs(s)) if sem < abs(s) else float('inf')
    sig_to = (abs(implied - KT_ODRIVE_CONFIGURED) / implied_sig
              if math.isfinite(implied_sig) and implied_sig > 0 else float('inf'))
    lines.append(
        f"SCALE: |slope| = {abs(s):.2f} A/Nm vs the {expected:.2f} A/Nm expected from "
        f"the ODrive's configured torque_constant ({KT_ODRIVE_CONFIGURED:.6f}) — "
        f"{err * 100:.1f}% error ⇒ {'OK' if scale_ok else 'MISMATCH'}")
    lines.append(
        f"     implied ODrive torque_constant = {implied:.5f} ± {implied_sig:.5f} "
        f"Nm/A ({sig_to:.1f}σ from the configured {KT_ODRIVE_CONFIGURED:.5f})")
    lines.append(
        f"     edge stats: {result.n_edges_kept} kept edges, trimmed CV "
        f"{result.jump_cv * 100:.1f}%, t = {result.t_stat:.1f}")
    if scale_ok:
        lines.append("     confirms torque_ff is interpreted exactly like "
                     "input_torque (iq = torque / torque_constant) AND that the int16 "
                     "x10000 wire scaling round-trips correctly.")
    else:
        lines.append("     *** the delivered current does NOT match torque_ff / "
                     "torque_constant. Either the wire scaling is wrong, or torque_ff "
                     "is NOT interpreted like input_torque. A gravity FF would land "
                     f"{(abs(s) / expected - 1) * 100:+.0f}% off. ***")
    return TorqueFfVerdict(
        channel_live=True, positive_tff_extends=extends,
        sign_matches_expectation=matches, scale_ok=scale_ok,
        implied_kt_nm_per_a=implied, sigma_to_odrive_config=sig_to, lines=lines)
