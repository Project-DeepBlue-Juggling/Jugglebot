"""Session AIM trim — **layer 2** of contract C-TOSS-CAL-1, and the shared
reduction/admission core the offline fit runs on.

Normative design: ``plans/active/toss-selftuning.md`` §§ 3.2, 3.6, 3.6.1, 3.6.2,
3.6.3 and decisions D5, D6, D7. Build phase **2e**.

WHAT THIS MODULE IS
-------------------
Two things, deliberately in one file:

1. **The reduction and admission core** — one toss record becomes one
   commanded-aim estimate (:func:`reduce_to_aim`) through the *production*
   forward model, and the § 3.6.2 guards decide whether it may be learnt from
   (:func:`admit_for_aim` / :func:`admit_for_timing` / :func:`admit_for_speed`).
   These arrived in phase 2c inside ``tests/hardware/toss_fit_lib.py``; phase 2e
   moved them **here**, into the production package, and that module now imports
   them. Root cause for the move, which is the same one D11 makes for the
   labeller: the ONLINE trim and the OFFLINE fit must admit and reduce a toss
   identically, or the map an operator promotes is fitted on a different
   population from the one the machine learnt on — and nothing would ever
   surface the difference. One implementation, imported by both.
2. **The session trim itself** (:class:`SessionTrim`) — the RAM-only,
   common-mode, per-goal estimator of § 3.6, with its clamps, its admission
   guards, its CUSUM freeze and its freeze-never-zero stop criteria.

WHERE THE TRIM SITS, AND WHY IT CANNOT DOUBLE-COUNT THE MAP
-----------------------------------------------------------
* Layer 0 — ``config/tilt_calibration.yaml`` (C-LEVEL-2), inclinometer.
* Layer 1 — ``config/toss_calibration.yaml`` (:mod:`jugglebot.motion.toss_cal`),
  ball-measured, **home-referenced**: it stores ``M(P) = b(P) − mean(b(home))``,
  the stable *spatial* part, with the common mode deliberately removed.
* **Layer 2 — this module.** The common mode ``M`` left out, estimated fresh
  every goal and discarded at the end of it.

The split is § 3.2's and it is physical, not stylistic: ``level`` is one
int16-quantised SCL3300 sample with 1.2–1.7 mrad/axis of session-to-session
scatter, so the absolute aim residual moves every time the operator re-levels
while the *spatial field* does not. Persisting the absolute would bake one
session's level noise into every future session; re-estimating the spatial field
每 session would chase noise at 9–25 under-determined cells. So the map owns the
field, the trim owns the offset, and there is exactly one parameter pair between
them — which is why they cannot double-count.

THE SIGN, AND THE FIXED POINT (read this before changing a minus)
------------------------------------------------------------------
The design writes the trim law with the measurement as an *error*::

    ŷ_k = S⁻¹ · land_err_mm_k / (4·h)          [design § 3.6]
    δ  ← δ_prev + clamp(−r_n − δ_prev, ±STEP_MAX)

That form does not close once a map is installed, for exactly the reason phase
2c documented for § 3.7 item 3: with applied aim ``A`` and plant bias ``ψ`` the
landing error is ``land_err = J·(A + ψ)``, so ``J⁻¹·land_err = A + ψ`` — it
still contains the aim the machine already applied. Feeding that back makes
``δ_{k+1} = −(M + δ_k + ψ)``, which oscillates rather than converging.

This module reduces to the **commanded aim that would have landed the ball on
B**, the 2c fixed point::

    b_k = A_k − J⁻¹ · land_err_k = −ψ          (independent of A_k)

and then subtracts the map's own contribution to obtain the quantity the trim
owns::

    y_k = b_k − M_k                            [the COMMON MODE demand, rad]
    r_n = (n₀·anchor + Σ y_k) / (n₀ + n)       [shrinkage mean, D6]
    δ  ← δ_prev + clamp(r_n − δ_prev, ±STEP_MAX)

The minus is already inside ``b``, so the update carries a plus. At the fixed
point ``δ* = −ψ − M`` and the machine commands ``M + δ* = −ψ``: exactly the aim
that cancels the plant bias, reached from any starting map. **The consequence
that matters operationally**: a goal may run with the previous map installed and
the trim still converges, because ``y`` does not depend on what is currently
applied.

Two definitions in § 3.6.3 have to be restated in the same convention, and the
restatement is mechanical rather than a judgement call. The design's ``|r_n|``
means *"how much error is left"*; here the equivalent is the **residual demand**
``|r_n − δ|`` — how much the estimator is still asking for. So:

* **CONVERGED** — ``|r_n − δ| < DEADBAND`` on 3 consecutive updates at n ≥ 6.
* **STALLED** — ``|δ|`` saturated at ``TRIM_MAX`` and ``|r_n − δ|`` has not
  fallen by ≥ 20 % over 3 updates. That is the "the demand exceeds the clamp, so
  this is a plant change" case, and the clamp is **not** raised.

WHAT THIS MODULE REFUSES TO GUESS — the phase's central finding
----------------------------------------------------------------
:func:`reduce_to_aim` needs ``land_err_mm``, whose schema origin is **M**
(mined): it is produced offline by the mocap descending-branch estimator
(``tools/probes/ball_arrival_offset.py``), joined into the record by
``tools/probes/toss_record_miner.py``. **The node has no live producer of it**,
and the two candidates are both refused on their merits rather than on
convenience:

* ``TossResult.catch_error_mm`` is D5's forbidden observable — a dead-reckoned
  free-fall extrapolation, a scalar *distance* not a signed 2-vector, and
  defined only for CAUGHT balls (selection-biased toward the cup, which is the
  one direction an aim fit must not be biased in).
* ``BallState.landing_position`` is the same tracker Kalman filter's predicted
  plane crossing — the same lineage, one message earlier.

So a live cycle's record is refused by :meth:`SessionTrim.observe` with the
named reason ``no_mocap_fit``, the trim stays in ``WARMUP``, and
:meth:`SessionTrim.aim` returns exactly ``(0.0, 0.0)``. That is not a stub: it
is the honest state of the instrument, it is why ``toss_trim_enabled`` defaults
**false**, and it is why the end-of-goal artefact is a *proposal* an operator
promotes rather than an auto-written calibration (premise P1). The moment a
record carries a ``land_err_mm`` — replayed from a mined corpus today, or from a
future live arrival estimator — the same ingest point starts learning with no
further plumbing. See the logbook § Phase 2e for the full argument.

WHAT THIS MODULE MUST NOT IMPORT
---------------------------------
:mod:`jugglebot.motion.toss_cal` — ``tests/motion/test_toss_cal.py::
test_nothing_else_imports_the_loader`` pins ``reload_coordinator_node`` as the
map's *single owner* (operator decision 7), and a second importer inside the
package is a second owner. The TOTAL authority clamp therefore stays in
``toss_cal.clamp_total_aim`` and is applied by the node at the one apply seam,
over ``map + trim`` — which is what D7's *"re-clamped AT APPLY, not only at
update"* asks for. This module owns only the trim's own ``TRIM_MAX``.

Pure Python — no ROS imports, no file I/O, consumed by
``reload_coordinator_node``, by ``tests/hardware/toss_fit_lib.py`` and by
``tests/motion`` with no ROS mocking at all.
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, NamedTuple, Optional, Sequence, Tuple

import numpy as np

import jugglebot.hardware_config as hw
from jugglebot import toss_record
from jugglebot.motion.trajectory.toss_release import aim_target_offset_mm

__all__ = [
    'TossTrimError',
    'TRIM_SCHEMA',
    'N0', 'N_MIN_APPLY', 'SE_GATE', 'SE_GATE_CONFIRM',
    'DEADBAND_RAD', 'STEP_MAX_RAD', 'TRIM_MAX_RAD',
    'SIGMA_PRIOR_MM', 'OUTLIER_SIGMA', 'OUTLIER_CONSECUTIVE_FREEZE',
    'CUSUM_K', 'CUSUM_H', 'CUSUM_N_MIN',
    'CONVERGED_UPDATES', 'CONVERGED_N_MIN',
    'STALLED_UPDATES', 'STALLED_IMPROVE_FRAC',
    'AFFINE_MIN_CELLS', 'AFFINE_MIN_SPREAD_MM', 'AFFINE_MIN_N',
    'AFFINE_MIN_CONDITION',
    'SPEED_DEADBAND', 'SPEED_STEP_MAX', 'SPEED_AUTHORITY',
    'TAU_DEADBAND_MS', 'TAU_STEP_MAX_MS', 'TAU_AUTHORITY_MS',
    'G4_DIP_BELOW_X3_MAX_REV', 'G4_STROKE_PEAK_MAX_REV',
    'G6_UPTIME_TREND_RATIO_MAX',
    'STATE_WARMUP', 'STATE_ACTIVE', 'STATE_CONVERGED', 'STATES',
    'FIT_RMS_MAX_MM', 'APEX_SANITY_FRAC', 'AIM_LABELS',
    'TIMING_POLL_DT_MS_MIN', 'TIMING_POLL_DT_MS_MAX', 'TAU_SE_MAX_MS',
    'SPEED_SE_MAX', 'SPEED_TIMING_N_MIN',
    'aim_landing_jacobian', 'achieved_flight_s', 'achieved_apex_m',
    'applied_aim_rad', 'map_aim_rad', 'reduce_to_aim',
    'admit_for_aim', 'admit_for_timing', 'admit_for_speed',
    'AimObservation', 'AffineFit', 'SessionTrim',
    'fit_affine', 'clamp_trim', 'trim_offset_mm',
]


class TossTrimError(RuntimeError):
    """Any refusal to reduce, admit, fit or write.

    ``tests/hardware/toss_fit_lib.TossFitError`` is an **alias** of this class,
    not a subclass: the reduction moved into this module in phase 2e and every
    ``except TossFitError`` in the fit tool must keep catching what the moved
    functions raise. One type, two names, no half-caught failure mode.
    """


#: Wire/artefact schema for the end-of-goal proposal document.
TRIM_SCHEMA = 'toss_trim_proposal/1'


# ── § 3.6 constants table. ALL PROVISIONAL until the first corpus. ───────────
#
# The mm equivalents below are at the reference geometry (T = 0.7977 s ⇔
# h = 0.78 m, catch z = 170 mm STOW), where the production aim→landing gain is
# **3126.64 mm/rad = 54.570 mm/deg** — measured, not assumed, by
# ``/tmp/probe_toss_trim.py`` (2026-08-11) differencing
# :func:`aim_landing_jacobian`. The design's idealised ``4h`` = 3120 mm/rad is
# 0.21 % lower.

#: Shrinkage prior strength (§ 3.6, D6). The prior is ``anchor.aim_rad`` from the
#: persistent map — the absolute residual measured at the home anchor, which is
#: *not* shipped as a correction precisely because it is level-noise
#: contaminated. At n₀ = 4 the prior's weight falls below 50 % at n = 5, i.e.
#: one node's worth of data, so a session that disagrees with the anchor
#: overrides it quickly.
N0 = 4

#: Minimum admitted tosses before the trim may act at all.
#:
#: **Deviation from the design's ``n ≥ 3``, and it comes with its measurement.**
#: Probed 2026-08-11 (``/tmp/probe_toss_trim.py`` → ``…trim4.py``, 300–400
#: synthetic sessions of 60–72 tosses at σ = 20 mm/axis, landing errors generated
#: through the production forward model): the design's ``n ≥ 3 ∧ |r| ≥ 2·se``
#: gate let **45.7 %** of ZERO-bias sessions command a non-zero trim, mean
#: 2.85 mm and 8.19 mm (the clamp) at the 95th percentile. Root cause: the gate
#: is a single-look significance test re-evaluated at every update, so a
#: wandering estimate eventually clears it by chance — a sequential
#: multiple-comparison problem the design's one-shot arithmetic cannot see.
#: Three tightenings were scanned together (this constant, :data:`SE_GATE` and
#: :data:`SE_GATE_CONFIRM`); see :data:`SE_GATE` for the table that chose them.
N_MIN_APPLY = 6

#: The significance factor: apply only when the estimate exceeds this many
#: standard errors. **2.5, not the design's 2.0**, chosen on the measured
#: expected residual aim error ``E|δ − b|`` in mm at the reference geometry over
#: 300 sessions × 72 tosses at σ = 20 mm/axis (``/tmp/probe_toss_trim4.py``,
#: 2026-08-11) — i.e. on what the operator actually feels, not on the false-
#: action rate::
#:
#:     true bias   no trim   gate 2.0   gate 2.5   gate 3.0
#:      0.00°       0.00       2.01       1.03       0.53
#:      0.07°       3.82       3.74       3.79       3.77
#:      0.10°       5.46       3.02       3.30       4.12
#:      0.15°       8.19       2.89       2.95       3.78
#:      0.30°       8.19       0.22       0.11       0.06
#:
#: 2.5 halves 2.0's zero-bias cost (2.01 → 1.03 mm) for ≤ 0.3 mm at the biases
#: worth correcting, and beats 3.0 by 0.8 mm right in the band the trim exists
#: for. **Two things in that table are load-bearing beyond this constant**: the
#: trim COSTS about a millimetre when there is nothing to correct (which is why
#: ``toss_trim_enabled`` defaults false), and at the ``level`` common mode the
#: design sized it on — 1.2–1.7 mrad/axis = 0.069–0.097° = 3.8–5.3 mm — it is a
#: **WASH** at σ = 20 mm. The trim only starts paying above ~0.12°. Whether σ is
#: really 20 mm has never been measured on this machine (§ 10: NO TRACK on all 31
#: descending branches of the reference sitting), and it is the number the whole
#: value proposition turns on.
SE_GATE = 2.5

#: Consecutive updates the significance gate must hold before the trim acts —
#: the direct answer to the sequential multiple-comparison problem in
#: :data:`N_MIN_APPLY`. Per axis, and reset to zero the moment the gate fails.
SE_GATE_CONFIRM = 3

#: A correction worth commanding, rad (§ 3.6: 0.10° = 5.46 mm at the reference
#: geometry). ~1/6 of the 35 mm capture radius. Below it the loop churns the
#: commanded pose for no measurable catch benefit. Applied to the **step**, i.e.
#: to ``|r_n − δ|``, which is the quantity actually being commanded.
DEADBAND_RAD = math.radians(0.10)

#: Largest single-update change in the commanded aim, rad (§ 3.6: 0.10°).
#: Bounds the commanded-orientation discontinuity between consecutive tosses: at
#: the platform extremity that is ``219.075 mm × 1.745e-3`` = **0.38 mm** of leg
#: travel, inside a profiled ``go_to_pose``.
STEP_MAX_RAD = math.radians(0.10)

#: THE session-trim authority bound, rad (§ 3.6: 0.15° = 8.19 mm), as a
#: **magnitude**. Three things make it this number: it equals θ_acc, the tilt
#: map's own accuracy floor; it is 1.5–2σ of the ``level`` single-sample scatter
#: (1.2–1.7 mrad/axis) the trim exists to cancel; and 8.19 mm is 23 % of the
#: 35 mm capture radius, so a fully saturated WRONG-SIGNED trim cannot by itself
#: cause a miss. A demand for more than this is a plant change, and the loop
#: FREEZEs and shouts (``STALLED``) rather than integrating into it.
TRIM_MAX_RAD = math.radians(0.15)

#: Prior for the per-toss reduction noise, mm per axis, used until
#: :data:`SIGMA_N_MIN` admitted tosses exist. **PROVISIONAL and it is the number
#: this module is least able to defend**: the reference sitting
#: ``2026-08-10_16-30-44`` reports NO TRACK on all 31 descending branches (2a,
#: § 10), so the arrival scatter has never been measured on this machine. 20 mm
#: is the design's own working figure (§ 3.6's se table and § 3.7's ``N_MIN``
#: derivation both use it). It is converted to radians per toss through that
#: toss's own Jacobian, so it stays correct at any throw height.
SIGMA_PRIOR_MM = 20.0

#: Admitted tosses before the sample sd replaces :data:`SIGMA_PRIOR_MM`. Below 5
#: an ``sd`` has more spread than the quantity it estimates, which would make
#: ``se`` jump around and the significance gate meaningless in exactly the window
#: where it matters most.
SIGMA_N_MIN = 5

#: G7 — drop a reduction this many sds from the running estimate.
#: Probed 2026-08-11 (``/tmp/probe_toss_trim.py``, 24 000 Gaussian samples):
#: **0 spurious drops**, so this is an outlier guard and not a noise trap.
OUTLIER_SIGMA = 4.0

#: G7 — consecutive drops that mean "regime change, not noise" ⇒ FREEZE.
OUTLIER_CONSECUTIVE_FREEZE = 3

#: G8 — tabular-CUSUM slack, in sd units, on the ONE-STEP-AHEAD residual
#: ``(y_k − r_{k−1})/σ̂``. See :meth:`SessionTrim._cusum` for why the residual is
#: one-step-ahead.
CUSUM_K = 0.5

#: G8 — tabular-CUSUM decision interval, in sd units.
#:
#: **The design says "a shift > 3·se ⇒ FREEZE"; this is a re-derivation of that
#: threshold from what the guard PROTECTS**, in the same shape phase 2c used for
#: the timing gate — because ``3·se`` is a moving target (``se`` shrinks as
#: ``σ/√(n₀+n)``) and, measured, a 3·se shift at n = 16 is 0.75 σ, which no
#: sequential test detects reliably inside a 60-toss goal at any tolerable
#: false-alarm rate. What G8 exists to catch is the braking-clamp CLASS of
#: regime change — a *large* mid-session shift, tens of mm — so the honest spec
#: is "rare false alarms, near-certain detection of ≥ 2 σ".
#:
#: Measured over 300 sessions per cell (``/tmp/probe_toss_trim3.py``,
#: 2026-08-11), 60 tosses, shift injected at n = 20::
#:
#:     k    h     false alarm    1σ step        2σ step        3σ step
#:     0.5  5.0      35.3 %      78.0 % / 7     84.7 % / 3     84.7 % / 2
#:     0.5  6.0      17.3 %      76.7 % / 9     91.0 % / 4     91.0 % / 3
#:     0.5  7.0       6.7 %      72.0 % / 10    98.3 % / 5     98.3 % / 3
#:     0.5  8.0       2.0 %      61.3 % / 12    99.7 % / 6     99.7 % / 4
#:     1.0  5.0       2.0 %      13.0 % / 8     82.7 % / 5     98.7 % / 3
#:
#: (percentage detected / median samples to alarm). ``k = 0.5, h = 8.0`` is the
#: chosen point: 2 % false alarm over a whole goal, ≥ 99 % detection of a 2σ
#: shift within 6 tosses. A false alarm costs precision only — it freezes the
#: trim at a bounded value and shouts — so the asymmetry is deliberate.
CUSUM_H = 8.0

#: G8 — admitted tosses before the CUSUM arms. Below this there is no robust sd
#: to standardise by, so an alarm would be an artefact of the prior.
CUSUM_N_MIN = SIGMA_N_MIN

#: § 3.6.3 CONVERGED — consecutive updates with a sub-deadband residual demand.
CONVERGED_UPDATES = 3

#: § 3.6.3 CONVERGED — the ``n ≥ 6`` floor, so three quiet updates on four
#: samples cannot be mistaken for convergence.
CONVERGED_N_MIN = 6

#: § 3.6.3 STALLED — updates over which the residual demand must improve.
STALLED_UPDATES = 3

#: § 3.6.3 STALLED — the improvement that counts as progress. Below it, with
#: ``|δ|`` saturated, the loop is asking for more authority than it has and the
#: answer is an ERROR naming the suspected plant fault, never a bigger clamp.
STALLED_IMPROVE_FRAC = 0.20

#: § 3.6.1 affine opt-in — minimum distinct visited cells.
AFFINE_MIN_CELLS = 3

#: § 3.6.1 affine opt-in — minimum spread in BOTH axes, mm.
AFFINE_MIN_SPREAD_MM = 100.0

#: § 3.6.1 affine opt-in — minimum admitted tosses.
AFFINE_MIN_N = 12

#: § 3.6.1 affine opt-in — the RANK test the design asks for in words
#: ("≥3 non-collinear visited cells"). Collinearity is not a boolean at floating
#: point, so it is a conditioning bar: the centred cell matrix's smaller singular
#: value must be at least this fraction of its larger one. At 0.10 a set of cells
#: within ~6° of a line is refused. **Rank-deficient fits must be refused, not
#: silently regularised** — a ridge term would return a plausible gradient
#: pointing along the one direction the session never sampled.
AFFINE_MIN_CONDITION = 0.10

#: § 3.6.1 speed trim — deadband / step / authority, as fractions of ``k_v``.
SPEED_DEADBAND = 0.01
SPEED_STEP_MAX = 0.02
SPEED_AUTHORITY = 0.10

#: § 3.6.1 release-latency trim — deadband / step / authority, ms.
TAU_DEADBAND_MS = 10.0
TAU_STEP_MAX_MS = 25.0
TAU_AUTHORITY_MS = 150.0

#: G4 plant health — the catch-robustness **Phase 0 gate row** verbatim
#: (``plans/active/catch-robustness.md`` § phase table: *"dip_below_x3 ≤ 0.10
#: rev; peak ≤ 10.060 rev; braking iq tracks commanded"*). Reused per toss so the
#: trim never learns against the braking-clamp plant. See
#: :func:`admit_for_aim`'s G4 block for what happens when the PLANT record block
#: is null, which it is until it is wired (§ 10).
G4_DIP_BELOW_X3_MAX_REV = 0.10
G4_STROKE_PEAK_MAX_REV = 10.060

#: G6 uptime — the τ estimate is refused when the session's own
#: residual-vs-uptime trend explains more of the spread than the between-node
#: signal. Expressed as the ratio (uptime-explained sd) / (between-node sd);
#: above 1.0 the session is measuring its own bridge uptime, which is D16's
#: *"convert the threshold into a measured within-session trend test"*.
G6_UPTIME_TREND_RATIO_MAX = 1.0

STATE_WARMUP = 'WARMUP'
STATE_ACTIVE = 'ACTIVE'
STATE_CONVERGED = 'CONVERGED'
#: Every other state is ``FROZEN_<reason>`` — see :meth:`SessionTrim._freeze`.
STATES = (STATE_WARMUP, STATE_ACTIVE, STATE_CONVERGED)

_FROZEN_PREFIX = 'FROZEN_'


# ═════════════════════════════════════════════════════════════════════════════
# The reduction and admission core (moved here from tests/hardware/toss_fit_lib
# in phase 2e — see the module docstring for the root cause).
# ═════════════════════════════════════════════════════════════════════════════

#: G2 track quality — the mocap descending-branch fit RMS ceiling, mm. Same
#: number ``tools/probes/toss_record_miner._mark_usable`` already applies, kept
#: identical on purpose: two different quality bars would make
#: ``usable_for_aim_fit`` and this module disagree about the same row.
FIT_RMS_MAX_MM = 3.0

#: G3 apex sanity — fractional tolerance on ``|h_ach − h_cmd| / h_cmd``. A wrong
#: apex mis-normalises the reduction (the gain is linear in h), so a toss that
#: missed its commanded height by more than this is routed to the SPEED
#: estimator instead of the aim fit.
APEX_SANITY_FRAC = 0.10

#: Labels admitted to the AIM fit (§ 3.7 item 4). CAUGHT and BOUNCED both mean
#: the ball arrived and its offset is real. MISSED is admitted too — see
#: :func:`admit_for_aim`, which requires a non-sparse mocap fit for it — because
#: the missed tosses are the most informative aim records and excluding them
#: biases the fit toward the cup. NO_RELEASE and UNKNOWN are excluded: the first
#: has no flight, the second has no verdict (D13 — UNKNOWN never collapses).
AIM_LABELS = (toss_record.LABEL_CAUGHT, toss_record.LABEL_BOUNCED,
              toss_record.LABEL_MISSED)

#: Floor for the τ fit's poll cadence. A per-record median cadence BELOW the
#: firmware's own configured poll interval is not a fast sensor, it is a stamp
#: that is not the poll stamp — a different firmware, or a mined field that does
#: not mean what this fit thinks. Refuse rather than fit a measurand of unknown
#: provenance. (Re-derived in phase 2c; see that phase's logbook section for the
#: measurement that retired the design's ±10 %-of-configured gate.)
TIMING_POLL_DT_MS_MIN = float(hw.JB_BD_CHECK_INTERVAL_MS)

#: Ceiling, sized on REACHABILITY rather than on the plant. The τ trim applies at
#: ``se ≤ 5 ms``, and with quantisation-limited samples ``se = Δ/√(12·n)``, so
#: ``n = Δ²/300`` for Δ in ms. At Δ = 200 ms that is **n = 133** admitted tosses
#: — more than the entire 129-toss first capture (§ 6). Past this cadence a
#: timing fit is not merely imprecise, it is unreachable inside a sitting.
TIMING_POLL_DT_MS_MAX = 200.0

#: Standard-error ceiling for the release-latency (τ) estimate, ms (§ 3.6.1).
#: The deadband is 10 ms, so 5 ms is the "estimate beats half the smallest
#: correction worth making" bar.
TAU_SE_MAX_MS = 5.0

#: Standard-error ceiling for the speed gain ``k_v``, as a fraction (§ 3.6.1:
#: "apply at n ≥ 5 with se ≤ 1 %").
SPEED_SE_MAX = 0.01

#: Minimum admitted tosses for the τ / k_v estimates (§ 3.6.1).
SPEED_TIMING_N_MIN = 5

_G = 9.80665


def _num(value: Any) -> Optional[float]:
    """``float(value)`` when it is a finite number, else ``None``.

    Every threshold comparison goes through this rather than a bare ``float()``:
    a corpus row is decoded from JSON written by a machine that may predate this
    build, so a string or a null in a numeric field is a data problem to be
    *reported as a refusal*, never a ``TypeError`` five frames up that kills a
    whole fit — or, live, a per-toss estimator update.
    """
    if value is None or isinstance(value, bool):
        return None
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    return out if math.isfinite(out) else None


def _is_pair(value: Any) -> bool:
    return (isinstance(value, (list, tuple)) and len(value) == 2
            and all(isinstance(v, (int, float)) and not isinstance(v, bool)
                    and math.isfinite(float(v)) for v in value))


def _z_of(rec: Dict[str, Any]) -> Optional[float]:
    xyz = rec.get('goal_catch_xyz_stow_mm')
    if not xyz or len(xyz) < 3 or xyz[2] is None:
        return None
    try:
        return round(float(xyz[2]), 3)
    except (TypeError, ValueError):
        return None


def aim_landing_jacobian(flight_time_s: float,
                         catch_z_stow_mm: float) -> np.ndarray:
    """``J`` (2×2, mm/rad): ``d(landing offset) / d(aim rx, ry)`` at zero aim.

    Column 0 is ``∂L/∂rx``, column 1 is ``∂L/∂ry``, rows are ``(Lx, Ly)``.

    Computed by finite-differencing the **production**
    :func:`~jugglebot.motion.trajectory.toss_release.aim_target_offset_mm` — the
    same function ``reload_coordinator_node._toss_aim_for_goal`` calls to APPLY
    the map — at a 1e-6 rad step. That is deliberate and it is the single most
    important line in this module: it means the fit, the trim and the apply path
    cannot disagree about the sign or the axis pairing, because there is only one
    implementation of either. A re-derived ``S`` here would be a second place for
    a sign to be wrong, and a sign error aims the machine roughly twice as badly
    as no map at all.

    The step is far inside the model's linear regime (the map's whole authority
    is 1° = 0.0175 rad and the offset is smooth through zero), and the model is
    exactly linear at the origin, so the difference quotient is the derivative to
    ~1e-10 relative.

    Sanity: at h = 0.78 m (T = 0.7977 s), z = 170 mm this returns
    ``[[0, 3126.64], [-3126.64, 0]]`` — magnitude 0.21 % above the design's
    idealised ``4h`` = 3120 mm/rad, and a 90° rotation, NOT a scaled identity.
    """
    T = float(flight_time_s)
    if not math.isfinite(T) or T <= 0.0:
        raise TossTrimError(
            'aim_landing_jacobian needs a positive flight time, got {!r}'
            .format(flight_time_s))
    z = float(catch_z_stow_mm)
    if not math.isfinite(z):
        raise TossTrimError(
            'aim_landing_jacobian needs a finite catch z, got {!r}'
            .format(catch_z_stow_mm))
    eps = 1e-6
    col_rx = np.asarray(aim_target_offset_mm(eps, 0.0, T, z), dtype=float) / eps
    col_ry = np.asarray(aim_target_offset_mm(0.0, eps, T, z), dtype=float) / eps
    return np.column_stack([col_rx, col_ry])


def trim_offset_mm(rx: float, ry: float, flight_time_s: float,
                   catch_z_stow_mm: float) -> Tuple[float, float]:
    """The mm-at-this-apex REPORT value for an aim ``(rx, ry)``.

    Thin wrapper over the production
    :func:`~jugglebot.motion.trajectory.toss_release.aim_target_offset_mm` so
    the console line and the record's ``trim_aim_mm_at_h`` are the *same*
    conversion the apply path uses, never a ``4h`` approximation printed beside
    an exact command. Returns ``(0.0, 0.0)`` for a zero aim, exactly.
    """
    off = aim_target_offset_mm(float(rx), float(ry), float(flight_time_s),
                               float(catch_z_stow_mm))
    return (float(off[0]), float(off[1]))


def achieved_flight_s(rec: Dict[str, Any]) -> Optional[float]:
    """The flight time to normalise this toss by: mocap-achieved if it exists,
    else the commanded one.

    Mocap first because the reduction's gain is linear in the flight time, so
    normalising a short throw by its commanded time mis-scales its residual —
    which is exactly the error G3 (apex sanity) is sized to bound.
    """
    for name in ('achieved_flight_s_mocap', 'flight_time_s'):
        T = _num(rec.get(name))
        if T is not None and T > 0.0:
            return T
    return None


def achieved_apex_m(rec: Dict[str, Any]) -> Optional[float]:
    """Apex implied by the achieved flight time, ``h = g·T²/8`` — the inverse of
    ``T = √(8h/g)``. Used only by G3; the reduction itself uses ``J(T, z)``."""
    T = achieved_flight_s(rec)
    if T is None:
        return None
    return _G * T * T / 8.0


def applied_aim_rad(rec: Dict[str, Any]) -> Optional[Tuple[float, float]]:
    """The aim this toss actually commanded, radians — or ``None`` if unknown.

    ``total_aim_rad`` is the declared "what was actually commanded" field and is
    preferred; ``map_aim_rad + trim_aim_rad`` is the fallback for a record
    written before the total was carried.

    **Unknown is not zero.** A mined-only row (no ``/toss/record`` declaration —
    every row of the three 2026-08-10 bags) cannot say what aim was applied, and
    guessing zero would make the fixed point subtract a bias that was in fact
    already applied, i.e. it would re-learn a correction the machine is already
    making. :func:`admit_for_aim` refuses such a row and names it.
    """
    total = rec.get('total_aim_rad')
    if _is_pair(total):
        return (float(total[0]), float(total[1]))
    parts = []
    for name in ('map_aim_rad', 'trim_aim_rad'):
        value = rec.get(name)
        if not _is_pair(value):
            return None
        parts.append((float(value[0]), float(value[1])))
    return (parts[0][0] + parts[1][0], parts[0][1] + parts[1][1])


def map_aim_rad(rec: Dict[str, Any]) -> Optional[Tuple[float, float]]:
    """The LAYER-1 (persistent map) contribution to this toss's aim, rad.

    The trim owns the common mode and the map owns the spatial field (§ 3.2), so
    the trim's per-toss measurement is ``reduce_to_aim(rec) − map_aim_rad(rec)``.
    Reading it off the RECORD rather than re-looking-up the map is what keeps
    this module out of ``toss_cal``'s import graph (see the module docstring) —
    and it is also more correct: the record carries the aim the machine *applied*
    for that toss, which is the right thing to subtract even if the map has since
    been reloaded mid-session.

    ``None`` when the field is absent or malformed: an unknown map contribution
    makes the common-mode demand unknowable, and guessing zero would fold the
    whole spatial field into the trim.
    """
    value = rec.get('map_aim_rad')
    if not _is_pair(value):
        return None
    return (float(value[0]), float(value[1]))


def reduce_to_aim(rec: Dict[str, Any]) -> Tuple[float, float]:
    """One toss → the commanded aim ``b`` that would have landed it on B, rad.

    ``b = A − J⁻¹ · land_err``  where ``A`` is the aim that WAS applied
    (:func:`applied_aim_rad`) and ``J`` the production aim→landing Jacobian at
    this toss's flight time and height.

    Why this is a converging fixed point, and why that matters operationally:
    with ``land_err = J·(A + ψ)`` the expression is ``A − (A + ψ) = −ψ``,
    **independent of A**. So a capture may run with the previous map installed —
    records inside one node carrying different applied biases reduce to the same
    ``b`` — and the ``--force-uninstall`` dance stays an escape hatch rather than
    a step of the routine (§ 3.7 item 3).

    See the module docstring for why the design's written ``+`` is not this.
    """
    aim = applied_aim_rad(rec)
    if aim is None:
        raise TossTrimError(
            '{}: the applied aim is unknown, so this toss cannot be reduced '
            '(see applied_aim_rad)'.format(rec.get('toss_uid')))
    err = rec.get('land_err_mm')
    if not _is_pair(err):
        raise TossTrimError(
            '{}: land_err_mm is absent or malformed'.format(rec.get('toss_uid')))
    T = achieved_flight_s(rec)
    z = _z_of(rec)
    if T is None or z is None:
        raise TossTrimError(
            '{}: no usable flight time / catch z to build the Jacobian'
            .format(rec.get('toss_uid')))
    J = aim_landing_jacobian(T, z)
    delta = np.linalg.solve(J, np.array([float(err[0]), float(err[1])]))
    return (float(aim[0]) - float(delta[0]), float(aim[1]) - float(delta[1]))


def admit_for_aim(rec: Dict[str, Any]) -> Tuple[bool, str]:
    """``(admitted, reason)`` for the AIM estimate — guards G1–G5 and G9–G11, as
    far as ONE record can decide them.

    Enforces the label-inclusion rule (§ 3.7 item 4) and guards G1 (release
    evidence), G2 (track quality), G3 (apex sanity), **G4 (plant health)**,
    **G5 (levelling)**, G9 (ball evidence), G10 (post-reload settle) and G11
    (retry cycle). The guards this deliberately does NOT enforce — G6 uptime
    trend, G7 outlier and G8 change detection — need the partition and the
    running estimate, which one row does not have; :class:`SessionTrim` and
    ``toss_fit_lib.fit_nodes`` own those.

    **G4 and the null PLANT block.** ``dip_below_x3_rev`` / ``stroke_peak_rev``
    ship null until the PLANT block is wired (§ 10), so G4 is enforced
    *conditionally*: a record that CARRIES the fields and is out of band is
    refused (``plant_braking_clamp``), and a record that does not carry them is
    admitted with the gap reported by :meth:`SessionTrim.snapshot`'s
    ``g4_unenforceable`` count rather than pretended away. Refusing every record
    for a field the pipeline does not yet produce would make the guard indis-
    tinguishable from an outage; silently admitting without saying so is how the
    braking clamp hid for a whole session (§ 7 R3).

    ``rimshot`` candidates are admitted for aim and excluded from timing, exactly
    as § 3.7 item 4 states — a rim transit moves the catch TIME, not the arrival
    POSITION.
    """
    label = rec.get('label')
    if label is None or label == toss_record.LABEL_UNKNOWN:
        return False, 'label_unknown'                          # G9 / D13
    if label == toss_record.LABEL_NO_RELEASE:
        return False, 'label_no_release'
    if label not in AIM_LABELS:
        return False, 'label_{}'.format(str(label).lower())

    if rec.get('t_departure_raw_ros') is None and not rec.get(
            'ball_track_confirmed') and not rec.get('throw_stroke_seen'):
        return False, 'no_release_evidence'                    # G1

    if not _is_pair(rec.get('land_err_mm')):
        return False, 'no_mocap_fit'
    if rec.get('fit_sparse'):
        return False, 'mocap_fit_sparse'                       # G2
    rms = _num(rec.get('fit_rms_mm'))
    if rms is None:
        return False, 'mocap_fit_rms_unknown'
    if rms > FIT_RMS_MAX_MM:
        return False, 'mocap_fit_quality'                      # G2
    if label == toss_record.LABEL_MISSED and (_num(rec.get('n_fit')) or 0) < 5:
        # MISSED is admitted ONLY on a non-sparse fit: it is the most
        # informative aim record there is, and excluding it would bias the whole
        # map toward the cup — but a thin track on a missed ball is exactly the
        # case where the estimator extrapolates.
        return False, 'missed_with_thin_track'

    if rec.get('retry_of'):
        return False, 'retry_cycle'                            # G11
    if rec.get('reload_settle'):
        return False, 'reload_settle'                          # G10

    if applied_aim_rad(rec) is None:
        return False, 'applied_aim_unknown'

    h_ach = achieved_apex_m(rec)
    h_cmd = _num(rec.get('apex_height_m'))
    if h_ach is not None and h_cmd is not None and h_cmd > 0.0:
        if abs(h_ach - h_cmd) / h_cmd > APEX_SANITY_FRAC:
            return False, 'apex_out_of_band'                    # G3
    if _z_of(rec) is None or achieved_flight_s(rec) is None:
        return False, 'no_geometry'

    ok, why = admit_g4_plant(rec)
    if not ok:
        return False, why
    ok, why = admit_g5_levelling(rec)
    if not ok:
        return False, why
    return True, ''


def admit_g4_plant(rec: Dict[str, Any]) -> Tuple[bool, str]:
    """G4 — plant health, from the record's PLANT block. ``(admitted, reason)``.

    The bounds are catch-robustness Phase 0's own gate row
    (:data:`G4_DIP_BELOW_X3_MAX_REV`, :data:`G4_STROKE_PEAK_MAX_REV`), reused per
    toss. **Do not learn against the braking-clamp plant**: a clamped decel
    changes both the release velocity and the seat, so a map fitted across a
    clamp boundary describes two machines.

    Absent fields ⇒ admitted, and the caller reports the gap — see
    :func:`admit_for_aim`.
    """
    dip = _num(rec.get('dip_below_x3_rev'))
    if dip is not None and dip > G4_DIP_BELOW_X3_MAX_REV:
        return False, 'plant_dip_below_x3'
    peak = _num(rec.get('stroke_peak_rev'))
    if peak is not None and peak > G4_STROKE_PEAK_MAX_REV:
        return False, 'plant_stroke_peak'
    if rec.get('trunc') is True:
        # A truncated stroke did not run the commanded profile, so its release
        # velocity is not the commanded one and its residual is not this map's.
        return False, 'plant_stroke_truncated'
    return True, ''


def admit_g5_levelling(rec: Dict[str, Any]) -> Tuple[bool, str]:
    """G5 — no learning on top of a different layer 0. ``(admitted, reason)``.

    Requires the levelling stack to be BOTH loaded and applied for this toss:
    ``gravity_correction_loaded`` ∧ ``tilt_map_applied``. The *version* half of
    the guard ("``requires`` versions match") is a session-level check, because
    it compares one record against the others — :meth:`SessionTrim.observe` owns
    it and FREEZEs on a mid-session change.

    An **absent** flag is a refusal, not a pass. "I cannot verify which layer 0
    was underneath this toss" is not "the right one was", and the whole point of
    layering is that a residual fitted under layer 0 A double-counts layer 0 B's
    delta (D3).
    """
    if rec.get('gravity_correction_loaded') is not True:
        return False, 'no_gravity_correction'
    if rec.get('tilt_map_applied') is not True:
        return False, 'no_tilt_map'
    return True, ''


def admit_for_timing(rec: Dict[str, Any]) -> Tuple[bool, str]:
    """``(admitted, reason)`` for the release-latency (τ) estimate — the phase-2c
    RE-DERIVED gate (see :data:`TIMING_POLL_DT_MS_MIN`).

    Requires: a departure edge and an announcement to measure against; a
    wall-anchored ``ball_held_stamp`` (a bridge-boot-relative stamp mis-times
    every edge by the whole bridge uptime — the largest silent error available
    anywhere in this pipeline, § 7 R2); a measured poll cadence inside the
    reachability band; and not a rimshot candidate.

    Note what is NOT required: agreement with ``JB_BD_CHECK_INTERVAL_MS``. The
    plant runs at ~3.5× the configured interval and has done for every bag we
    have; a gate on the configured number refuses 100 % of records.
    """
    if rec.get('t_departure_raw_ros') is None:
        return False, 'no_departure_edge'
    if rec.get('announce_throw_time_ros') is None:
        return False, 'no_announcement'
    if rec.get('ball_held_stamp_wall_anchored') is False:
        return False, 'bridge_stamp_not_wall_anchored'
    if rec.get('rimshot'):
        return False, 'rimshot_candidate'
    dt = _num(rec.get('sensor_poll_dt_ms_median'))
    if dt is None:
        return False, 'poll_cadence_unmeasured'
    if dt < TIMING_POLL_DT_MS_MIN:
        return False, 'poll_cadence_below_configured_interval'
    if dt > TIMING_POLL_DT_MS_MAX:
        return False, 'poll_cadence_unreachable'
    return True, ''


def admit_for_speed(rec: Dict[str, Any]) -> Tuple[bool, str]:
    """``(admitted, reason)`` for the speed-gain (``k_v``) estimate.

    Consumes ``achieved_flight_s_mocap`` against ``flight_time_s`` (§ 3.7
    item 5), so it needs BOTH and a released ball. Deliberately does NOT apply
    G3: a toss whose apex missed by more than 10 % is precisely the record the
    speed estimator exists to consume (:func:`admit_for_aim` routes it here).

    ``achieved_flight_s_fsm`` is deliberately NOT accepted as a substitute for
    the mocap flight time even though the node can fill it live. It is the FSM's
    catch-event-derived number, defined only for CAUGHT balls and referenced to
    the *seat* rather than the plane crossing; substituting it would make the
    online and offline speed estimates two different measurands wearing one name.
    """
    label = rec.get('label')
    if label in (None, toss_record.LABEL_UNKNOWN, toss_record.LABEL_NO_RELEASE):
        return False, 'label_unusable'
    for name in ('achieved_flight_s_mocap', 'flight_time_s'):
        value = _num(rec.get(name))
        if value is None or value <= 0.0:
            return False, 'no_flight_pair'
    if rec.get('retry_of') or rec.get('reload_settle'):
        return False, 'interlude_cycle'
    return True, ''


# ═════════════════════════════════════════════════════════════════════════════
# The session trim
# ═════════════════════════════════════════════════════════════════════════════


class AimObservation(NamedTuple):
    """The verdict on ONE record offered to :meth:`SessionTrim.observe`.

    ``admitted`` false always carries a ``reason``; the reasons are the guard
    vocabulary above plus ``outlier`` (G7) and ``frozen`` (the estimator has
    stopped learning). Every observation is counted in the snapshot whether it
    was admitted or not — a census of refusals is how a session proves it had
    nothing to learn from rather than that it chose not to.
    """
    toss_uid: str
    admitted: bool
    reason: str
    y_rad: Optional[Tuple[float, float]]
    node_xy_mm: Optional[Tuple[float, float]]


class AffineFit(NamedTuple):
    """``δ(P) = c + G·(P − P_anchor)`` — opt-in, structurally refused by default.

    ``ok`` false ⇒ ``reason`` names which of § 3.6.1's four bars failed and
    ``c``/``gradient`` are ``None``. There is deliberately no partial answer: a
    rank-deficient affine fit regularised into existence returns a gradient
    pointing along the one direction the session never sampled.
    """
    ok: bool
    reason: str
    c_rad: Optional[Tuple[float, float]]
    gradient: Optional[np.ndarray]      # 2x2, d(aim)/d(P mm)
    n: int
    cells: int


def clamp_trim(rx: float, ry: float,
               limit: float = TRIM_MAX_RAD) -> Tuple[float, float, bool]:
    """Clamp a trim vector's MAGNITUDE to ``limit``, preserving direction.

    Returns ``(rx, ry, clamped)``.

    **A magnitude, not a per-axis box**, for the reason
    ``toss_cal._check_aim_magnitude`` already argues one layer up: the physical
    quantity every bound is sized on (composition regime, cup swing, landing
    shift) is the from-vertical aim, and a per-axis box permits ``hypot`` =
    1.414× the bound at the corners — 41 % more authority than the number that
    was justified. This is the one place the design's literal *"per axis
    a ∈ {x, y}"* is not followed, and it is followed everywhere it is safe: the
    ESTIMATOR (mean, sd, se, significance gate, deadband) is per axis exactly as
    written; only the two authority clamps are magnitudes.
    """
    x, y = float(rx), float(ry)
    if not (math.isfinite(x) and math.isfinite(y)):
        raise TossTrimError(
            'trim must be finite, got (rx={!r}, ry={!r})'.format(rx, ry))
    mag = math.hypot(x, y)
    if mag <= float(limit):
        return (x, y, False)
    scale = float(limit) / mag
    return (x * scale, y * scale, True)


def _history_sd(values: np.ndarray) -> Optional[np.ndarray]:
    """Per-axis sd of the ADMITTED history, or ``None`` below
    :data:`SIGMA_N_MIN` samples.

    **Where the design's "running ROBUST sd" went, and why this is the robust
    one.** The obvious reading is MAD·1.4826, and that was the first
    implementation. Measured, it broke the CUSUM: MAD·1.4826 is consistent only
    asymptotically and is biased **low** at the n = 5…10 this estimator lives in,
    so every standardised residual was inflated. Measured consequence: the G8
    false-alarm rate could not be driven below **14.8 %** over a 60-toss goal at
    ANY ``(k, h)`` in the scan (``/tmp/probe_toss_trim2.py``, 2026-08-11), and
    the pairs that got close detected a real shift in under 7 % of sessions.
    Correcting the bias means carrying a finite-sample factor table — a second
    calibrated constant defending a first.

    The robustness is already bought somewhere better: **G7 removes a > 4 σ̂
    reduction BEFORE it ever enters this history** (:meth:`SessionTrim.observe`
    returns early), so the sample this sd is computed over is outlier-free by
    construction and the plain ``ddof=1`` sd is both unbiased and far lower
    variance. One guard, one place, and the estimator that consumes it is not
    also trying to be one.
    """
    if values.shape[0] < SIGMA_N_MIN:
        return None
    return np.maximum(np.std(values, axis=0, ddof=1), 1e-12)


def fit_affine(points_mm: Sequence[Sequence[float]],
               y_rad: Sequence[Sequence[float]],
               anchor_mm: Sequence[float] = (0.0, 0.0)) -> AffineFit:
    """§ 3.6.1's opt-in affine trim, refused unless the geometry supports it.

    The four bars, in the order they are reported: ≥ :data:`AFFINE_MIN_N`
    admitted tosses, ≥ :data:`AFFINE_MIN_CELLS` distinct visited cells, spread
    ≥ :data:`AFFINE_MIN_SPREAD_MM` in BOTH axes, and a centred-cell matrix whose
    conditioning clears :data:`AFFINE_MIN_CONDITION` (the design's
    "non-collinear", made numeric).

    Never called by :class:`SessionTrim` unless the caller asks: the DEFAULT
    trim is the 2-parameter constant, because the time-varying part of the error
    is common-mode by physics (§ 3.2) and letting a session fit 9–25
    under-determined cells produces 9–25 noise chasers.
    """
    P = np.asarray(points_mm, dtype=float).reshape(-1, 2)
    Y = np.asarray(y_rad, dtype=float).reshape(-1, 2)
    n = int(P.shape[0])
    cells = {(round(float(p[0]), 3), round(float(p[1]), 3)) for p in P}
    if n < AFFINE_MIN_N:
        return AffineFit(False, 'n<{}'.format(AFFINE_MIN_N), None, None, n,
                         len(cells))
    if len(cells) < AFFINE_MIN_CELLS:
        return AffineFit(False, 'cells<{}'.format(AFFINE_MIN_CELLS), None, None,
                         n, len(cells))
    cell_arr = np.asarray(sorted(cells), dtype=float)
    spread = cell_arr.max(axis=0) - cell_arr.min(axis=0)
    if float(spread[0]) < AFFINE_MIN_SPREAD_MM or \
            float(spread[1]) < AFFINE_MIN_SPREAD_MM:
        return AffineFit(False,
                         'spread({:.0f},{:.0f})mm<{:.0f}'.format(
                             float(spread[0]), float(spread[1]),
                             AFFINE_MIN_SPREAD_MM),
                         None, None, n, len(cells))
    centred = cell_arr - cell_arr.mean(axis=0)
    sv = np.linalg.svd(centred, compute_uv=False)
    if sv.size < 2 or sv[0] <= 0.0 or (sv[1] / sv[0]) < AFFINE_MIN_CONDITION:
        return AffineFit(False, 'rank_deficient', None, None, n, len(cells))
    anchor = np.asarray(anchor_mm, dtype=float).reshape(2)
    design = np.column_stack([np.ones(n), P - anchor])
    coeffs, *_ = np.linalg.lstsq(design, Y, rcond=None)
    return AffineFit(True, '', (float(coeffs[0][0]), float(coeffs[0][1])),
                     np.asarray(coeffs[1:], dtype=float).T, n, len(cells))


class SessionTrim:
    """The layer-2 common-mode aim trim for ONE toss goal (§ 3.6).

    Lifecycle: constructed at goal start, fed one record per cycle terminal
    (:meth:`observe`), read exactly once per cycle at
    ``reload_coordinator_node._build_toss_cycle`` (:meth:`aim`), and **discarded
    at the goal's end** — its only persistent trace is the proposal document
    (:meth:`proposal`), which an operator promotes through the explicit
    calibration routine or does not. That is premise P1 enforced rather than
    documented: nothing here can write ``config/toss_calibration.yaml``.

    Thread-safety is the caller's: the node holds ``_lock`` around ``observe``
    and ``aim`` exactly as it does around the rest of its per-goal state.

    **Freeze, never zero.** Every guard path ends in :meth:`_freeze`, which
    stops the estimator and leaves ``δ`` at its current value. Zeroing would
    inject a ``TRIM_MAX``-sized step into the next commanded pose, which is
    strictly worse than holding a stale but bounded correction (§ 3.6.3).
    """

    def __init__(self, *,
                 anchor_aim_rad: Optional[Sequence[float]] = None,
                 n0: int = N0,
                 sigma_prior_mm: float = SIGMA_PRIOR_MM,
                 speed_k_v_prior: Optional[float] = None,
                 session_id: str = '',
                 goal_id: str = '') -> None:
        self.session_id = str(session_id)
        self.goal_id = str(goal_id)
        self._n0 = max(0, int(n0))
        self._sigma_prior_mm = float(sigma_prior_mm)
        prior = (np.zeros(2) if anchor_aim_rad is None
                 else np.asarray(anchor_aim_rad, dtype=float).reshape(2))
        if not np.all(np.isfinite(prior)):
            raise TossTrimError(
                'anchor_aim_rad must be finite, got {!r}'.format(anchor_aim_rad))
        self._prior = prior

        self._ys: List[np.ndarray] = []
        self._nodes: List[Tuple[float, float]] = []
        self._uptimes: List[float] = []     # paired with _ys, for G6/D16
        self._gains: List[float] = []
        self._delta = np.zeros(2)
        self._state = STATE_WARMUP
        self._reason = ''
        self._reset_reason = ''
        self._r = self._prior.copy()
        self._mean = np.zeros(2)          # the DATA's own location — G7/G8 only
        self._sd = np.full(2, float('nan'))
        self._se = np.full(2, float('nan'))
        self._updates = 0
        self._last_change_update = 0
        self._changes = 0
        self._gate_run = np.zeros(2, dtype=int)
        self._demand_history: List[float] = []
        self._outlier_run = 0
        self._cusum_p = np.zeros(2)
        self._cusum_m = np.zeros(2)
        self._clamp_hits: List[str] = []

        self._seen = 0
        self._refusals: Dict[str, int] = {}
        self._guards: Dict[str, int] = {}
        self._g4_unenforceable = 0
        self._provenance: Optional[Tuple[Any, Any]] = None

        # Scalar sub-estimators (§ 3.6.1) — their own gates, their own authority.
        self._kv_prior = (None if speed_k_v_prior is None
                          else float(speed_k_v_prior))
        self._kv_samples: List[float] = []
        self._kv = 1.0 if self._kv_prior is None else self._kv_prior
        self._kv_state = STATE_WARMUP
        self._tau_samples: List[float] = []
        self._tau_ms = 0.0
        self._tau_state = STATE_WARMUP
        self._tau_refusal = ''

    # ── state helpers ────────────────────────────────────────────────────────

    @property
    def state(self) -> str:
        return self._state

    @property
    def frozen(self) -> bool:
        """True iff ``δ`` will not move again this goal.

        ``CONVERGED`` counts: § 3.6.3 says *"freeze c, keep recording"*, and it
        is the "keep recording" half that :attr:`learning_stopped` separates out.
        """
        return self._state.startswith(_FROZEN_PREFIX) \
            or self._state == STATE_CONVERGED

    @property
    def learning_stopped(self) -> bool:
        """True iff the estimator has stopped INGESTING — a guard fired.

        Deliberately narrower than :attr:`frozen`. A ``CONVERGED`` trim keeps
        taking observations and keeps running G7 and G8 over them; it simply
        stops moving ``δ``. Otherwise convergence would blind the change detector
        precisely when it matters most: a plant that shifts *after* the trim
        settled is the braking-clamp scenario G8 exists for, and a converged
        session would sail through it reporting CONVERGED to the end.
        """
        return self._state.startswith(_FROZEN_PREFIX)

    @property
    def n(self) -> int:
        """Admitted tosses."""
        return len(self._ys)

    def _freeze(self, reason: str, detail: str = '') -> None:
        """Stop learning and HOLD ``δ``. Never zeroes — see the class docstring.

        The first freeze wins: a session that trips G7 and then G8 is frozen for
        the reason that actually stopped it, and re-labelling would lose the
        chronology an operator needs to diagnose the plant.

        ``CONVERGED`` is NOT a freeze and must not block one — the short-circuit
        is on :attr:`learning_stopped`, not :attr:`frozen`. A converged session
        that then trips G7 or G8 has to say so: convergence is a statement about
        the estimate, a guard is a statement about the plant, and the plant wins.
        (Measured: with the short-circuit on ``frozen``, three consecutive 4σ
        outliers after convergence were counted and then silently discarded.)
        """
        self._guards[reason] = self._guards.get(reason, 0) + 1
        if self.learning_stopped:
            return
        self._state = _FROZEN_PREFIX + reason
        self._reason = detail or reason

    def _count(self, reason: str) -> None:
        self._refusals[reason] = self._refusals.get(reason, 0) + 1

    # ── THE read ─────────────────────────────────────────────────────────────

    def aim(self) -> Tuple[float, float]:
        """THE commanded session trim, rad — read ONCE per goal at
        ``_build_toss_cycle`` and never anywhere else.

        Always inside :data:`TRIM_MAX_RAD` by construction (the clamp is applied
        at update, and this is a pure read of the stored value). Exactly
        ``(0.0, 0.0)`` while the estimator is in ``WARMUP``, which is what makes
        the disabled/unfed path bit-identical to the pre-2e machine.
        """
        return (float(self._delta[0]), float(self._delta[1]))

    def speed_gain(self) -> float:
        """``k_v`` — the multiplier § 3.6.1 defines on ``event_vel_mps``. 1.0
        until the estimate passes its own gate.

        **NOT WIRED as of 2026-08-11 — this is an OBSERVABLE, not a command.**
        Nothing in ``reload_coordinator_node`` calls it: the node's only trim
        consumer is :meth:`aim`, and ``event_vel_mps`` is still taken verbatim
        from ``release_cmd``. The estimator runs, the console prints it and
        :meth:`proposal` records it, so a sitting produces the *evidence* for
        wiring it — and wiring it is a deliberate later decision, because it
        would give a session-local estimator authority over launch speed on the
        hardware path. Do not read the paragraph below as "this is applied and
        it is safe"; read it as the safety argument the wiring decision starts
        from.

        ⚠ **The safety argument that used to stand here is FALSE — do not
        reinstate it.** It read: *"``x3`` is algebraically velocity-independent,
        so a speed trim cannot move the hand toward the end stop, and
        ``validate_event_vel`` still gates the result against [0.3, 7.0] m/s."*
        ``x3`` is velocity-independent; the **uncommanded coast past ``x3``** is
        not — it runs 0.074 rev at 2.742 m/s to 1.020 rev at 4.858 m/s (measured
        2026-07-27; contract **C-HAND-3**, ``ros_ws/docs/hand_throw_envelope.md``
        § B1). A +10 % trim at the derived envelope ceiling takes 4.357 →
        4.793 m/s and the modelled peak from 10.600 to ~11.05 rev, **0.25 rev
        past the 10.8 rev mechanical stop** — while sailing through the [0.3,
        7.0] wire band, which bounds nothing physical.

        **What wiring this actually requires**: the trim is applied at DISPATCH,
        after the FSM's CHECKING gate, so a wired ``speed_gain`` bypasses the
        single enforcement point outright. Wiring it means re-running
        ``throw_envelope.evaluate(flight_time_s, k_v * event_vel_mps)`` on the
        TRIMMED speed, at the point of application — not relying on a gate that
        already ran against the untrimmed value.
        """
        return float(self._kv)

    def release_latency_ms(self) -> float:
        """Session-local τ, ms (§ 3.6.1). 0.0 until the estimate passes its gate.

        **NOT WIRED as of 2026-08-11 — an OBSERVABLE, not a command**, exactly
        as :meth:`speed_gain` above: nothing reads it, ``event_delay_s`` still
        carries only the config-shipped ``release_latency_ms_applied``. It is
        measured and reported so the shift is *visible* per session.

        **Never persisted**, and that stays true whatever the wiring decision is.
        It is uptime-dependent by nature — the measured
        announcement→physical-release shift runs +12.8…+23.4 ms at a fresh boot
        and +118–133 ms at ~16 h of can-bridge uptime — so a number written into
        ``config/hardware_config.yaml`` would be a lie about a different plant.
        :meth:`proposal` records it under ``session_local`` and the loader has no
        key to read it from.
        """
        return float(self._tau_ms)

    # ── ingest ───────────────────────────────────────────────────────────────

    def observe(self, rec: Dict[str, Any]) -> AimObservation:
        """Offer ONE ``toss_record/1`` row to the estimator.

        Returns the verdict; never raises on a malformed record (a corrupt row
        is a refusal with a name, not an exception into a teardown path). The
        aim, speed and τ estimators each see the row and each apply their own
        admission — a toss can legitimately be refused for aim (its apex missed)
        and admitted for speed, which is § 3.6.2 G3's explicit routing.
        """
        if not isinstance(rec, dict):
            return AimObservation('', False, 'not_a_record', None, None)
        uid = str(rec.get('toss_uid') or '')
        self._seen += 1
        self._observe_speed(rec)
        self._observe_tau(rec)

        admitted, reason = admit_for_aim(rec)
        if admitted and rec.get('dip_below_x3_rev') is None \
                and rec.get('stroke_peak_rev') is None:
            self._g4_unenforceable += 1
        if not admitted:
            self._count(reason)
            return AimObservation(uid, False, reason, None, None)

        # G5's session half: LAYER 0 must not change mid-goal.
        #
        # Only ``tilt_map_version`` is latched, deliberately — **not**
        # ``toss_cal_version``. A layer-1 (aim map) reload mid-goal is HARMLESS
        # to this estimator by construction: the reduction subtracts the aim the
        # toss actually applied (``y = reduce_to_aim(rec) − map_aim_rad(rec)``),
        # so a record thrown under map A and one thrown under map B reduce to the
        # same common-mode demand. That invariance is the fixed-point property
        # the whole design rests on (§ 3.7 item 3), and freezing on it would
        # punish the exact workflow the capture routine uses — fit, reload,
        # verify. Layer 0 is different: it moves the physical baseline the
        # residual is measured against, and a residual fitted under tilt map A
        # double-counts tilt map B's delta (D3).
        prov = (rec.get('tilt_map_version'),)
        if self._provenance is None:
            self._provenance = prov
        elif prov != self._provenance:
            self._freeze('LEVELLING_CHANGED',
                         'the LAYER-0 tilt map changed mid-goal: {} -> {}. The '
                         'residual measured before it double-counts the delta '
                         '(D3), so this goal stops learning and holds.'
                         .format(self._provenance[0], prov[0]))
            self._count('provenance_changed')
            return AimObservation(uid, False, 'provenance_changed', None, None)

        try:
            b = reduce_to_aim(rec)
            m = map_aim_rad(rec)
        except TossTrimError:
            self._count('reduction_failed')
            return AimObservation(uid, False, 'reduction_failed', None, None)
        if m is None:
            self._count('map_aim_unknown')
            return AimObservation(uid, False, 'map_aim_unknown', None, None)
        y = np.array([b[0] - m[0], b[1] - m[1]], dtype=float)

        pose = rec.get('goal_catch_xyz_stow_mm') or []
        node = ((float(pose[0]), float(pose[1]))
                if len(pose) >= 2 and pose[0] is not None
                and pose[1] is not None else None)

        if self.learning_stopped:
            # Still counted, still recorded — the design's "freeze, keep
            # recording". The corpus of a frozen session is exactly the corpus
            # an operator needs to see WHY it froze.
            self._count('frozen')
            return AimObservation(uid, False, 'frozen', (float(y[0]),
                                                         float(y[1])), node)

        # G7 — outlier, against the DATA's own location before this sample.
        #
        # Referenced on ``_mean`` (the plain running mean of the admitted
        # reductions), NOT on ``_r`` (the shrinkage estimate). The two differ by
        # the prior's pull, ``n₀/(n₀+n)`` of the distance between prior and data
        # — which at n = 5 is 44 % of it. Referencing ``_r`` therefore makes
        # every early sample look like an outlier whenever the session's true
        # bias is far from the anchor prior, and the loop freezes on its own
        # warm start. Measured while writing the property tests: a 0.60° bias at
        # σ = 2 mm tripped ``FROZEN_OUTLIERS`` at n = 5, on the first seed tried.
        #
        # The split is the right one on its own terms: ``_r`` is what the trim
        # ACTS on and deliberately borrows strength from the prior, while G7 and
        # G8 judge whether THE DATA is behaving, and a prior is not data.
        sd = self._sd
        if np.all(np.isfinite(sd)) and self._ys:
            if np.any(np.abs(y - self._mean) > OUTLIER_SIGMA * sd):
                self._outlier_run += 1
                self._count('outlier')
                self._guards['G7'] = self._guards.get('G7', 0) + 1
                if self._outlier_run >= OUTLIER_CONSECUTIVE_FREEZE:
                    self._freeze(
                        'OUTLIERS',
                        '{} consecutive reductions past {:.0f} sd — a '
                        'regime change, not noise'.format(self._outlier_run,
                                                          OUTLIER_SIGMA))
                return AimObservation(uid, False, 'outlier',
                                      (float(y[0]), float(y[1])), node)
        self._outlier_run = 0

        # G8 — the change detector, on the ONE-STEP-AHEAD residual.
        self._cusum(y)

        self._ys.append(y)
        # _nodes and _uptimes are index-paired with _ys — G6's trend test reads
        # all three together, so an unknown node or uptime is stored as a
        # placeholder rather than skipped (a shorter list would silently
        # mis-pair every subsequent sample).
        self._nodes.append(node if node is not None else (float('nan'),
                                                          float('nan')))
        self._uptimes.append(_num(rec.get('uptime_ms_at_release'))
                             if _num(rec.get('uptime_ms_at_release')) is not None
                             else float('nan'))
        T = achieved_flight_s(rec)
        z = _z_of(rec)
        if T is not None and z is not None:
            self._gains.append(
                float(np.linalg.norm(aim_landing_jacobian(T, z)[:, 1])))
        self._update()
        return AimObservation(uid, True, '', (float(y[0]), float(y[1])), node)

    # ── the estimator ────────────────────────────────────────────────────────

    def _sigma_prior_rad(self) -> np.ndarray:
        gain = (float(np.mean(self._gains)) if self._gains
                else 3126.64)          # the reference-geometry gain, mm/rad
        return np.full(2, self._sigma_prior_mm / max(gain, 1e-9))

    def _cusum(self, y: np.ndarray) -> None:
        """Two-sided tabular CUSUM (G8). FREEZE on an alarm; never re-converge.

        Two implementation choices, both forced by measurement rather than by
        taste (``/tmp/probe_toss_trim2.py``, 2026-08-11):

        * the residual is **one-step-ahead** — ``(y_k − mean_{k−1})/σ̂``,
          referenced on the DATA's own running mean over the samples seen SO
          FAR (this one is not in it yet, because ``observe`` runs the detector
          before appending). Referenced on a mean that INCLUDES the sample the
          statistic is contaminated by the very thing it is testing; referenced
          on the SHRINKAGE estimate it inherits the prior's pull and alarms on a
          correct warm start (the same defect G7 hit — see :meth:`observe`);
        * ``σ̂`` is :func:`_history_sd` — the plain sd of the G7-protected
          history. See that function for why the design's "robust sd" is not a
          MAD here, and for the measurement that forced the change.

        Silently re-converging after a regime change is how the braking clamp
        hid for a whole session (§ 3.6.2 G8), so the alarm is terminal for the
        goal: freeze, shout, keep recording.
        """
        if len(self._ys) < CUSUM_N_MIN or not np.all(np.isfinite(self._sd)):
            return
        z = (y - self._mean) / self._sd
        self._cusum_p = np.maximum(0.0, self._cusum_p + z - CUSUM_K)
        self._cusum_m = np.minimum(0.0, self._cusum_m + z + CUSUM_K)
        if np.any(self._cusum_p > CUSUM_H) or np.any(-self._cusum_m > CUSUM_H):
            self._freeze(
                'CUSUM',
                'two-sided CUSUM alarm (S+={:.2f}, S-={:.2f}, h={:.1f}) — the '
                'aim residual SHIFTED mid-goal; this is a plant change, and the '
                'trim will not re-converge into it'
                .format(float(np.max(self._cusum_p)),
                        float(np.min(self._cusum_m)), CUSUM_H))

    def _update(self) -> None:
        """One estimator update: shrinkage mean, gates, step + trim clamps, stop
        criteria. Called only for an ADMITTED, non-outlier observation."""
        ys = np.asarray(self._ys, dtype=float)
        n = ys.shape[0]
        self._mean = ys.mean(axis=0)
        self._r = (self._n0 * self._prior + ys.sum(axis=0)) / (self._n0 + n)
        sd = _history_sd(ys)
        self._sd = self._sigma_prior_rad() if sd is None else sd
        self._se = self._sd / math.sqrt(self._n0 + n)
        self._updates += 1

        demand = self._r - self._delta
        self._demand_history.append(float(np.hypot(*demand)))

        # ── the two gates, per axis exactly as § 3.6 writes them ──
        #
        # The DEADBAND is on ``|r_n|`` — the ESTIMATE — and not on the step. That
        # is the design's placement and it is load-bearing: a deadband on the
        # step creates a dead ZONE between ``DEADBAND`` (0.10°) and ``TRIM_MAX``
        # (0.15°) in which no correction can ever be commanded, because the first
        # move toward any target below 0.10° is itself below 0.10°. Written the
        # design's way the deadband asks *"is this bias worth correcting at
        # all?"* and ``STEP_MAX`` bounds how fast the answer is applied — two
        # different questions with two different constants. (Found by the
        # property tests: with the deadband on the step, a real 0.12° bias never
        # produced a single command.)
        #
        # Churn is not a physical hazard at this scale: ``STEP_MAX`` is 0.38 mm
        # of leg travel at the platform extremity, inside a profiled
        # ``go_to_pose``, so the design's deadband rationale is a benefit
        # argument and it belongs on the benefit's own quantity.
        candidate = self._delta.copy()
        for axis in (0, 1):
            if abs(float(self._r[axis])) >= SE_GATE * float(self._se[axis]):
                self._gate_run[axis] += 1
            else:
                self._gate_run[axis] = 0
            # THREE conditions, and the third is the one that makes the loop
            # self-terminating: move ``δ`` only when what is already commanded is
            # SIGNIFICANTLY wrong. Without it ``δ`` is re-assigned to ``r`` at
            # every update and therefore tracks ``r``'s own random walk forever —
            # measured at σ = 20 mm, the commanded trim never went 20 updates
            # without a nudge in 240 tosses, which is precisely the churn the
            # deadband exists to prevent and which the deadband cannot prevent
            # once it is (correctly) placed on the estimate.
            #
            # It needs no new constant: it is the SAME significance test as the
            # first condition, asked about the residual ``r − δ`` instead of
            # about ``r``. Read together: *act when the estimate is significantly
            # non-zero, worth correcting, and significantly different from what
            # is already commanded.* The loop then stops when ``δ`` is inside the
            # estimator's own precision and re-opens only if ``r`` drifts out of
            # it — which is also what makes CONVERGED and "the pose stopped
            # moving" the same event instead of two.
            if (n >= N_MIN_APPLY
                    and self._gate_run[axis] >= SE_GATE_CONFIRM
                    and abs(float(self._r[axis])) >= DEADBAND_RAD
                    and abs(float(self._r[axis]) - float(self._delta[axis]))
                    >= SE_GATE * float(self._se[axis])):
                candidate[axis] = float(self._r[axis])

        step = candidate - self._delta
        step_mag = float(np.hypot(*step))
        hits: List[str] = []
        if step_mag > 0.0 and not self.learning_stopped:
            if step_mag > STEP_MAX_RAD:
                step = step * (STEP_MAX_RAD / step_mag)
                hits.append('trim_step')
            new = self._delta + step
            rx, ry, clamped = clamp_trim(float(new[0]), float(new[1]))
            if clamped:
                hits.append('trim_max')
            self._delta = np.array([rx, ry], dtype=float)
            # CONVERGED is DESCRIPTIVE, not a latch — a step re-opens it. See
            # _check_stop_criteria for the root cause (the shrinkage estimand is
            # non-stationary while the prior's weight decays, so an early CI is
            # not evidence that the estimate has stopped moving).
            self._state = STATE_ACTIVE
            self._last_change_update = self._updates
            self._changes += 1
        self._clamp_hits = hits

        self._check_stop_criteria()

    def _check_stop_criteria(self) -> None:
        """§ 3.6.3 CONVERGED / STALLED. Both restated in the fixed-point
        convention — see the module docstring.

        **CONVERGED is DESCRIPTIVE and re-evaluated every update; the FROZEN_*
        states are terminal.** The design's word is "freeze c", and for a
        terminal guard (STALLED, OUTLIERS, CUSUM, LEVELLING_CHANGED) freezing is
        exactly right — those say the data cannot be trusted. Convergence says
        the opposite: *the loop has nothing left to ask for*, which is a
        statement about the current estimate and not about the plant.

        Latching it is wrong for a measurable reason. ``r_n`` is a SHRINKAGE mean
        whose estimand moves as the prior's weight ``n₀/(n₀+n)`` decays, so its
        standard error is not the whole story about how far it can still travel
        — and a session warm-started from an anchor that disagrees with the plant
        passes through ``r ≈ 0`` on its way to the truth. Latched, the trim
        freezes at zero exactly there. Measured while writing the property tests:
        a +0.20° prior against a −0.12° plant reported CONVERGED at n = 6 with
        ``r`` = 0.004° and then rode ``r`` to −0.108° without ever commanding
        anything.
        """
        if self.learning_stopped:
            return
        hist = self._demand_history
        n = len(self._ys)
        saturated = float(np.hypot(*self._delta)) >= TRIM_MAX_RAD * (1.0 - 1e-9)
        # CONVERGED requires the trim NOT to be saturated. A clamped ``δ`` with a
        # small remaining demand looks identical to convergence from the demand
        # alone — and it is the opposite: the estimator is asking for more
        # authority than it has and is being refused. Calling that CONVERGED
        # would report "the loop settled" for the exact case § 3.6.3 wants an
        # ERROR on. Found by the property tests: a session warm-started from a
        # deliberately wrong anchor saturated at TRIM_MAX and reported CONVERGED
        # at n = 11 while ``r`` was still moving.
        if not saturated and n >= CONVERGED_N_MIN and \
                len(hist) >= CONVERGED_UPDATES:
            # The demand's UPPER CONFIDENCE BOUND must be inside the deadband,
            # not just its point estimate. Without the ``+ SE_GATE·|se|`` term
            # this criterion cannot tell *"I have evidence of no bias"* from
            # *"I have no evidence of a bias"* — and it latches, so the second
            # case would freeze the loop at zero on a barely-formed estimate and
            # never let it act when the estimate sharpened. Measured while
            # writing the property tests: a real 0.12° bias at σ = 4 mm reported
            # CONVERGED at n = 6 with |r| = 0.079° and se = 0.019°, and never
            # moved again while |r| went on past 0.11°.
            se_mag = float(np.hypot(*self._se))
            if all(d + SE_GATE * se_mag < DEADBAND_RAD
                   for d in hist[-CONVERGED_UPDATES:]):
                self._state = STATE_CONVERGED
                self._reason = (
                    'residual demand + {:.1f}·se < {:.4f} deg on {} consecutive '
                    'updates at n = {}'.format(
                        SE_GATE, math.degrees(DEADBAND_RAD), CONVERGED_UPDATES,
                        n))
                return
        if saturated and len(hist) > STALLED_UPDATES:
            window = hist[-(STALLED_UPDATES + 1):]
            first, last = window[0], window[-1]
            if first > 0.0 and (first - last) / first < STALLED_IMPROVE_FRAC:
                self._freeze(
                    'STALLED',
                    'the trim is SATURATED at {:.3f} deg and the residual demand '
                    'has not fallen {:.0f} % over {} updates ({:.4f} -> {:.4f} '
                    'deg). That is a plant change, not a calibration — do NOT '
                    'raise the clamp; check the hand braking clamp, the level, '
                    'and the bridge uptime.'
                    .format(math.degrees(TRIM_MAX_RAD),
                            100.0 * STALLED_IMPROVE_FRAC, STALLED_UPDATES,
                            math.degrees(first), math.degrees(last)))

    # ── the scalar sub-estimators (§ 3.6.1) ──────────────────────────────────

    def _observe_speed(self, rec: Dict[str, Any]) -> None:
        """``k̂_v = √(mean(h_cmd/h_ach))``, applied at n ≥ 5, se ≤ 1 %."""
        ok, _ = admit_for_speed(rec)
        if not ok:
            return
        t_cmd = _num(rec.get('flight_time_s'))
        t_ach = _num(rec.get('achieved_flight_s_mocap'))
        if t_cmd is None or t_ach is None or t_ach <= 0.0 or t_cmd <= 0.0:
            return
        # h ∝ T², so h_cmd/h_ach = (T_cmd/T_ach)² and k_v = T_cmd/T_ach.
        self._kv_samples.append(t_cmd / t_ach)
        n = len(self._kv_samples)
        if n < SPEED_TIMING_N_MIN:
            return
        arr = np.asarray(self._kv_samples, dtype=float)
        mean = float(np.mean(arr))
        se = float(np.std(arr, ddof=1) / math.sqrt(n)) if n > 1 else float('inf')
        if se > SPEED_SE_MAX:
            self._kv_state = 'REFUSED_SE'
            return
        base = 1.0 if self._kv_prior is None else self._kv_prior
        target = mean if self._kv_prior is None else self._kv_prior * mean
        if abs(target - self._kv) < SPEED_DEADBAND * max(base, 1e-9):
            self._kv_state = STATE_CONVERGED
            return
        step = float(np.clip(target - self._kv, -SPEED_STEP_MAX,
                             SPEED_STEP_MAX))
        self._kv = float(np.clip(self._kv + step, base * (1.0 - SPEED_AUTHORITY),
                                 base * (1.0 + SPEED_AUTHORITY)))
        self._kv_state = STATE_ACTIVE

    def _observe_tau(self, rec: Dict[str, Any]) -> None:
        """``τ = median(t_rel_phys − t_rel_cmd)``, applied at n ≥ 5, se ≤ 5 ms.

        G6 (uptime) is enforced HERE and only here for the τ channel, which is
        what § 3.6.2 asks for: *"spatial estimates admitted, τ and all
        label-based scoring refused when the session's own residual-vs-uptime
        trend exceeds the between-node signal"*.
        """
        ok, why = admit_for_timing(rec)
        if not ok:
            self._tau_refusal = why
            return
        t_dep = _num(rec.get('t_departure_raw_ros'))
        t_ann = _num(rec.get('announce_throw_time_ros'))
        if t_dep is None or t_ann is None:
            return
        self._tau_samples.append(1000.0 * (t_dep - t_ann))
        n = len(self._tau_samples)
        if n < SPEED_TIMING_N_MIN:
            return
        arr = np.asarray(self._tau_samples, dtype=float)
        se = float(np.std(arr, ddof=1) / math.sqrt(n))
        if se > TAU_SE_MAX_MS:
            self._tau_state = 'REFUSED_SE'
            return
        ratio = self._uptime_trend_ratio()
        if ratio is not None and ratio > G6_UPTIME_TREND_RATIO_MAX:
            self._tau_state = 'REFUSED_G6_UPTIME'
            self._guards['G6'] = self._guards.get('G6', 0) + 1
            return
        target = float(np.median(arr))
        if abs(target - self._tau_ms) < TAU_DEADBAND_MS:
            self._tau_state = STATE_CONVERGED
            return
        step = float(np.clip(target - self._tau_ms, -TAU_STEP_MAX_MS,
                             TAU_STEP_MAX_MS))
        self._tau_ms = float(np.clip(self._tau_ms + step, -TAU_AUTHORITY_MS,
                                     TAU_AUTHORITY_MS))
        self._tau_state = STATE_ACTIVE

    def _uptime_trend_ratio(self) -> Optional[float]:
        """G6 / D16, measured: (uptime-explained sd) / (between-node sd) of the
        AIM reductions. ``None`` when the session cannot answer.

        D16 replaced an invented uptime *ceiling* with a **measured
        within-session trend test**, and this is it, computed over the same
        quantity the map is fitted from so both sides of the ratio are in the
        same units and neither needs a scale factor:

        * numerator — the sd of ``ŷ`` explained by a linear fit on
          ``uptime_ms_at_release``, i.e. ``|slope| · sd(uptime)``;
        * denominator — the sd of the per-node MEANS of ``ŷ``, i.e. the
          between-node signal the map exists to describe.

        Above 1.0 the session is measuring its own bridge uptime more than it is
        measuring its own geometry, and the τ estimate (the channel uptime moves
        directly) is refused. ``None`` — fewer than 5 paired samples, no uptime
        span, or fewer than two visited nodes — is reported as UNKNOWN rather
        than assumed benign, which is the whole point of turning a ceiling into
        a measurement.
        """
        if len(self._ys) < SPEED_TIMING_N_MIN or not self._uptimes:
            return None
        ys = np.asarray(self._ys, dtype=float)
        up = np.asarray(self._uptimes, dtype=float)
        if up.shape[0] != ys.shape[0]:
            return None
        good = np.isfinite(up)
        if int(good.sum()) < SPEED_TIMING_N_MIN:
            return None
        up, ys = up[good], ys[good]
        if float(np.ptp(up)) <= 0.0:
            return None
        nodes = [self._nodes[i] for i, keep in enumerate(good)
                 if keep and i < len(self._nodes)
                 and math.isfinite(self._nodes[i][0])]
        by_node: Dict[Tuple[float, float], List[int]] = {}
        for index, node in enumerate(nodes):
            by_node.setdefault((round(node[0], 1), round(node[1], 1)),
                               []).append(index)
        if len(by_node) < 2 or len(nodes) != ys.shape[0]:
            return None
        means = np.asarray([ys[idx].mean(axis=0) for idx in by_node.values()])
        between = float(np.linalg.norm(means.std(axis=0)))
        if between <= 0.0:
            return None
        slopes = np.asarray([np.polyfit(up, ys[:, axis], 1)[0]
                             for axis in (0, 1)])
        explained = float(np.linalg.norm(slopes) * np.std(up))
        return explained / between

    # ── reporting ────────────────────────────────────────────────────────────

    def snapshot(self) -> Dict[str, Any]:
        """Everything the record, the console and the proposal read. No I/O."""
        nodes = sorted({(round(x, 1), round(y, 1)) for x, y in self._nodes
                        if math.isfinite(x) and math.isfinite(y)})
        return {
            'schema': TRIM_SCHEMA,
            'state': self._state,
            'reason': self._reason,
            'reset_reason': self._reset_reason,
            'aim_rad': self.aim(),
            'r_rad': (float(self._r[0]), float(self._r[1])),
            'se_rad': (float(self._se[0]), float(self._se[1])),
            'sd_rad': (float(self._sd[0]), float(self._sd[1])),
            'n': self.n,
            'n_seen': self._seen,
            'n0': self._n0,
            'prior_rad': (float(self._prior[0]), float(self._prior[1])),
            'updates': self._updates,
            # How long the COMMANDED trim has been still. The operational
            # meaning of "the loop stopped", and the one that survives a σ at
            # which the CONVERGED label is not reachable inside a sitting.
            'updates_since_change': self._updates - self._last_change_update,
            # How MANY times the commanded trim moved all goal. A settled loop
            # moves a handful of times and then stops; a churning one moves on
            # every update, which is what the deadband and the "significantly
            # different from what is commanded" test exist to prevent.
            'changes': self._changes,
            'nodes': [list(node) for node in nodes],
            'refusals': dict(sorted(self._refusals.items())),
            'guards_tripped': dict(sorted(self._guards.items())),
            'g4_unenforceable': self._g4_unenforceable,
            'clamp_hits': list(self._clamp_hits),
            'speed_k_v': self.speed_gain(),
            'speed_state': self._kv_state,
            'speed_n': len(self._kv_samples),
            'tau_ms': self.release_latency_ms(),
            'tau_state': self._tau_state,
            'tau_n': len(self._tau_samples),
            'tau_refusal': self._tau_refusal,
            'uptime_trend_ratio': self._uptime_trend_ratio(),
        }

    def console_lines(self, *, node_xy_mm: Optional[Sequence[float]] = None,
                      map_aim_rad: Optional[Sequence[float]] = None,
                      flight_time_s: Optional[float] = None,
                      catch_z_stow_mm: Optional[float] = None,
                      uptime_ms: Optional[float] = None,
                      elapsed_s: Optional[float] = None,
                      applied: bool = False,
                      cycle_index: Optional[int] = None) -> List[str]:
        """The § 6 ``TRIM`` console block — three lines, one stream.

        **``SESSION-ONLY`` vs ``PERSISTENT`` is on the line from day one.** That
        is the loaded-vs-applied distinction the tilt-cal review had to go back
        and fix in four documents afterwards: a number that is never written to
        disk and a number that is must never look the same in a log. So must
        ``NOT APPLIED``, when the estimator has a value the goal is not using
        because ``toss_trim_enabled`` is false.

        **Every line carries a clock**, because the plant changes underneath a
        long session: the tracking lag grows with can-bridge uptime (+118–133 ms
        at ~16 h against a 40 ms arm-suppression margin), so a line without one
        cannot be compared with a line from an hour earlier.

        ``uptime_ms`` is the can-bridge uptime and is what the design asks for —
        but ``reload_coordinator_node`` does not subscribe to ``/link_status``
        and 2a's argument against adding that subscription to the node that owns
        the hand, the latch and the abort ladder still stands, so the live caller
        passes ``None`` and the line says ``uptime UNMEASURED`` in as many words
        rather than implying a number it does not have. ``elapsed_s`` (seconds
        into this goal) is the within-session monotonic that IS available, and
        the bag carries ``/link_status`` at 5 Hz for the offline join.
        """
        rx, ry = self.aim()
        if uptime_ms is not None and math.isfinite(float(uptime_ms)):
            up = 'uptime {:.0f} s'.format(float(uptime_ms) / 1000.0)
        else:
            up = 'uptime UNMEASURED'
        if elapsed_s is not None and math.isfinite(float(elapsed_s)):
            up += ' (+{:.0f} s into goal)'.format(float(elapsed_s))
        where = ('node({:+.0f},{:+.0f})'.format(float(node_xy_mm[0]),
                                                float(node_xy_mm[1]))
                 if node_xy_mm is not None else 'node(?)')
        mm = ''
        if flight_time_s is not None and catch_z_stow_mm is not None:
            try:
                ox, oy = trim_offset_mm(rx, ry, flight_time_s, catch_z_stow_mm)
                mm = ' = ({:+.1f},{:+.1f}) mm'.format(ox, oy)
            except (ValueError, TossTrimError):
                mm = ''
        head = ('TRIM {}: aim ({:+.3f},{:+.3f})deg{}  [clamp +-{:.3f} deg] {} '
                '{}'.format(where, math.degrees(rx), math.degrees(ry), mm,
                            math.degrees(TRIM_MAX_RAD),
                            'APPLIED' if applied else 'NOT APPLIED', up))
        mid = ('     n={}/{}  se ({:.3f},{:.3f})deg  state {}  SESSION-ONLY '
               '(discarded at goal end; a proposal is written, never a map) {}'
               .format(self.n, self._seen,
                       math.degrees(float(self._se[0]))
                       if math.isfinite(float(self._se[0])) else float('nan'),
                       math.degrees(float(self._se[1]))
                       if math.isfinite(float(self._se[1])) else float('nan'),
                       self._state, up))
        persistent = ('(persistent map: ({:+.3f},{:+.3f}) deg)'
                      .format(math.degrees(float(map_aim_rad[0])),
                              math.degrees(float(map_aim_rad[1])))
                      if map_aim_rad is not None
                      else '(persistent map: none loaded)')
        tail = '     {} PERSISTENT{} {}'.format(
            persistent,
            '' if cycle_index is None else ' — cycle {}'.format(int(cycle_index)),
            up)
        lines = [head, mid, tail]
        if self._reason:
            lines.append('     {} {}'.format(self._reason, up))
        return lines

    def proposal(self, **meta: Any) -> Dict[str, Any]:
        """The end-of-goal artefact for ``temp/logs/<session>_trim_proposal.yaml``.

        **A proposal, never a calibration.** Premise P1 says promotion into
        ``config/toss_calibration.yaml`` requires the explicit routine and its
        acceptance gates; enforcing that means this method produces a document
        the loader cannot read — different top-level ``schema``, no ``grid``, no
        ``aim_rad`` — so a mistaken ``cp`` into ``config/`` is refused loudly by
        ``parse_toss_cal`` rather than silently applied.

        ``session_local`` holds the two numbers that must NEVER be promoted at
        all: τ (uptime-dependent by nature) is in there, and the loader has no
        key for it.
        """
        snap = self.snapshot()
        doc: Dict[str, Any] = {
            'schema': TRIM_SCHEMA,
            'promotion': ('PROPOSAL ONLY — premise P1. Promote through '
                          'tests/hardware/toss_cal_fit.py and its acceptance '
                          'gates; never copy this file into config/.'),
            'session_id': self.session_id,
            'goal_id': self.goal_id,
            # The RECORD schema, not the arrival-offset estimator's version:
            # that identity lives in ``toss_cal.ESTIMATOR_VERSION`` and this
            # module may not import that loader (operator decision 7), so the
            # NODE passes it in as ``estimator_version=`` — see
            # ``reload_coordinator_node._toss_trim_end``.
            'record_schema': toss_record.SCHEMA,
            'trim': {
                'c_rad': list(snap['aim_rad']),
                'c_deg': [math.degrees(v) for v in snap['aim_rad']],
                'r_rad': list(snap['r_rad']),
                'se_rad': list(snap['se_rad']),
                'n': snap['n'],
                'n_seen': snap['n_seen'],
                'n0': snap['n0'],
                'prior_rad': list(snap['prior_rad']),
                'state': snap['state'],
                'reason': snap['reason'],
                'node_coverage': snap['nodes'],
                'guards_tripped': snap['guards_tripped'],
                'refusals': snap['refusals'],
                'g4_unenforceable': snap['g4_unenforceable'],
            },
            'session_local': {
                'speed_k_v': snap['speed_k_v'],
                'speed_state': snap['speed_state'],
                'speed_n': snap['speed_n'],
                'release_latency_ms': snap['tau_ms'],
                'release_latency_state': snap['tau_state'],
                'release_latency_n': snap['tau_n'],
                'release_latency_note': (
                    'NEVER persisted — uptime-dependent by nature (§ 3.6.1).'),
                'uptime_trend_ratio': snap['uptime_trend_ratio'],
            },
        }
        doc.update({str(k): v for k, v in meta.items()})
        return doc


def replay(records: Sequence[Dict[str, Any]], **kwargs: Any) -> SessionTrim:
    """Run a whole corpus through a fresh :class:`SessionTrim`, in order.

    The offline counterpart of the live ingest, and the ONLY way the loop closes
    today: the live record's ``land_err_mm`` is mined, not declared (see the
    module docstring), so a session's own trim converges when its bag has been
    through ``tools/probes/toss_record_miner.py`` and the mined corpus is
    replayed here. Used by the property tests and available to the analyser.
    """
    trim = SessionTrim(**kwargs)
    for rec in records:
        trim.observe(rec)
    return trim
