"""Pure fit core for the critical-point ILC — throw-side sensitivity + update law.

Normative design: ``plans/active/critical-point-ilc.md``, **Phase 1**. This module
is the hardware-free half of ``tests/hardware/ilc_fit.py`` (a thin CLI over it)
and is imported unchanged by ``tests/motion/test_ilc_fit.py``. Nothing here opens
a socket, launches a node, or imports ``rclpy`` — it reads a mined JSONL corpus
and returns numbers, so the whole fit runs in the ordinary pytest suite. It is
the sibling of ``tests/hardware/toss_fit_lib.py`` (the aim map's fit core) and
deliberately borrows its structure, its refusal discipline and its
partition-before-you-pool rule.

WHAT THIS MODULE OWNS
---------------------
1. **The command vector** ``u`` (§ Command vector) — four throw-side channels,
   each with its unit, its production seam and its authority bound
   (:data:`U_CHANNELS`); the four catch-side channels are DECLARED and
   NOT IMPLEMENTED (:data:`CATCH_CHANNELS`), and asking for one raises.
2. **The forward model** :func:`e_model` — ``u`` → release state → ballistic
   flight → the five-channel task error ``e`` (§ Command vector and task error).
3. **The sensitivity** :func:`sensitivity` — central finite differences of
   :func:`e_model`, plus the conditioning report and the **v1 channel screen**
   (:func:`conditioning`, :func:`screen_channels`).
4. **The update law** :func:`solve_step` — damped least squares with a box trust
   region, in closed form — and the **exact-gate re-validation** loop
   (:func:`admit_command`, :func:`propose_step`).
5. **The corpus side** — admission, the measured ``e``, the per-goal key, and
   the pre-registered repeatability decision (:func:`repeatability`).

DESIGN CONSTRAINT 1 IS ABSOLUTE, AND HERE IS THE WHOLE OF WHAT IT COSTS
----------------------------------------------------------------------
Every mapping in :func:`e_model` is a call into the shipped planner. The chain,
in the order ``reload_coordinator_node._build_toss_cycle`` runs it:

    ``toss_cal.clamp_total_aim`` → ``toss_release.aim_target_offset_mm``
    → virtual target → ``toss_release.compute_release_state_tilted``
    → (event_vel trim) → ``ballistics_bc.arrival_state_at_z``
    → ``toss_record_miner._lean_rad``

``_lean_rad`` is imported from the **miner** rather than from the production
package because the miner is the definition point of the mined
``arrival_dir_err_rad`` field: the model's direction channel and the measured one
must be the same five lines of code or the difference between them is not an
error, it is a convention mismatch. Everything else is production.

**Exactly one arithmetic operation in this file is not executed by a production
call**, and it is named so it cannot hide: the ``event_vel`` trim multiplies the
commanded launch magnitude by ``(1 + δv)``. That is
``toss_trim.SessionTrim.speed_gain``'s documented semantic (*"``k_v`` — the
multiplier § 3.6.1 defines on ``event_vel_mps``"*), and that method's own
docstring says in capitals that it is **NOT WIRED**: nothing in
``reload_coordinator_node`` calls it, ``event_vel_mps`` is still taken verbatim
from ``release_cmd``. So the seam is *specified* in production and *unwired* in
production, and wiring it is Phase 2's job. Modelling it here as a multiply is
not a second copy of any live code — there is no live code to copy — and the
multiply is applied to the production ``launch_vel_mms`` vector, so the
direction still comes from ``compute_release_state_tilted``'s tilt geometry and
never from a re-derived cup axis.

THE FOUR THROW-SIDE CHANNELS, AND WHY ONE OF THEM IS REFUSED
------------------------------------------------------------
``release_timing_offset`` (δt) has a real production seam —
``reload_coordinator_node._dispatch_toss_throw`` shifts ``event_delay`` by
``hw.JB_OP_TOSS_RELEASE_LATENCY_MS`` (*"while the announced landing stays
un-shifted"*) — and it is still **refused from v1**, for a reason stronger than
"no seam": its column of ``F`` is zero on all five task channels.

**Be precise about what that zero is worth, because it is easy to over-claim.**
The column is **structurally zero by construction**, not measured through the
production seam: δt is never passed to any production call — no chain in this
module takes a dispatch time — it enters :func:`e_model` only as a rigid
translation of the release instant (``t_release = δt``, ``t_arrival = t_release +
t_flight``), and every landing quantity is a difference of instants that both
carry it, so it cancels algebraically. The four spatial channels never reference
it at all. The invariance of the landing point therefore rests on the **physical
argument** that the platform holds its pose through the flight — the cup does not
move, the arc is unchanged — and *this model does not test that argument*: it
assumes it, by having no term through which a dispatch shift could reach the
geometry. Routing δt through the real seam is deferred; until it is, read this
column as "the model has no δt dependence" and not as "the machine has none".

What δt *does* move — the announcement-versus-physical release shift, mined as
``release_time_err_ms`` — is a **scheduling** error, not a task error: it sizes
the catch-arm window (``hand_stroke.stroke_clear_time``,
``required_arm_lead_s``) and it is already owned by another lane
(``toss_trim.SessionTrim.release_latency_ms``, itself NOT WIRED, and the bridge
temporal-trustworthiness arc behind gate G-1). Putting it in ``e`` would import
that lane's problem into this one. The column is still **computed rather than
asserted** — :func:`sensitivity` differences it like every other channel and gets
0.0, so an accidental δt dependence would show — and :func:`screen_channels`
excludes it by the same rank test that would exclude any other degenerate
channel. No private model of a stroke-phase-dependent release was written; there
is no production model of one to call.

E-1: CLOSED 2026-08-13, AND IT WAS A MASK CHANGE
-------------------------------------------------
Phase 1's entry condition E-1 (opened 2026-08-12 from 0c) was a ±100 mm/s
repeatable branch-to-branch velocity artefact sitting on exactly the lateral
channels. Between those two dates :data:`E1_BLOCKED` masked ``land_err_x``,
``land_err_y``, ``arrival_dir_x``, ``arrival_dir_y`` out of every residual this
module fitted, while ``F`` stayed full-size — ``F`` is model-only, computed by
differencing the planner, and no mocap artefact can reach it. That separation is
what made this a mask change rather than a re-derivation, and it is why the
closure edit is :data:`DEFAULT_MASK` and nothing else.

The mechanism is a **measurement bias, confirmed by parity about the apex**: the
tracked point is the centroid of the visible retroreflective cap, so it carries a
height-locked ``b(z)``; height is EVEN in ``tau = t − t_apex``, so the bias
enters a lateral channel as an even function while any aerodynamic force is ODD.
The fix lives in the MINER (``toss_record_miner.mine_arc`` takes every lateral
velocity from a whole-arc fit, which is bias-immune by parity, and mines
``coverage_asym_s`` with a refusal past ``COVERAGE_ASYM_MAX_S``), and the
evidence is reproducible from ``tools/probes/mocap_parity_bias.py``. See
:data:`DEFAULT_MASK` for the numbers and :func:`lateral_admissible` for the
admission requirement that keeps a pre-E-1 corpus from re-opening it.

What the artefact was worth, kept for scale (n = 19 usable rows, mined
2026-08-12, PER-BRANCH estimators): the release-branch lean ran ``+0.0181 rad``
in y against the arrival-branch ``−0.0024`` rad, a branch-to-branch delta of
0.0205 rad = **91 mm/s** at the 4436 mm/s release — three times the aim
channels' own scatter (sd 0.0030 / 0.0020 rad), and repeatable, so it would not
have averaged away.

THE AIM BLOCK'S CROSS-CHECK IS AN IDENTITY, NOT A TOLERANCE
------------------------------------------------------------
The plan asks for ``F``'s aim/landing block to reproduce ``b = 4.h.theta``
"within tolerance". It does better than that, and the exact statement is worth
having because it turns V1 from a percentage into an equation. The production
offset is ``(Δz + g.T²/2).tan(theta)`` with the tilted-release drop O(theta²),
and ``T = sqrt(8h/g)`` makes ``g.T²/2`` exactly ``4h``, so the derivative at zero
is::

    dL/dtheta = 4h + Δz        Δz = HAND_CATCH_OFFSET_MM - HAND_THROW_OFFSET_MM
                               = 64.78 - 58.044 = 6.736 mm

Measured through this module: 3126.736 mm/rad at h = 0.78 against 4h = 3120.0,
and 4006.736 at h = 1.00 against 4000.0 — the excess is 6.736 in both, to the
last digit. The "0.21 % / 0.23 % above 4h" quoted in several places in this
codebase is therefore exactly ``Δz/4h``, which is height-dependent; the constant
is the 6.736.

**THE CANONICAL STATEMENT OF THIS GAIN (D3, resolved 2026-08-21).** Every other
site in the tree now points here. Three facts, all measured
(``/tmp/probe_d3_gain.py``, run 2026-08-21):

1. the excess over ``4h`` is the CONSTANT ``Δz`` = 6.7360 mm at every h, so
   "0.21 % above 4h" is ``Δz/4h`` and is height-dependent — quote the constant,
   not the percentage;
2. the gain does **not depend on the catch z** at all (identical to 4 decimals
   over z ∈ {0, 100, 170, 250} mm). ``aim_landing_jacobian(T, z)`` takes z
   because the production seam does, not because the answer moves;
3. so the three numbers in the tree are **three geometries of one exact rule**,
   not three roundings of one number:

   * **3126.736** mm/rad — h = *exactly* 0.78 m (T = 0.79771241 s). This module.
   * **3126.639** mm/rad — T = *exactly* 0.7977 s (h = 0.779976 m). ``toss_trim``
     (which quoted it as "3126.64"); its reference geometry rounds T, not h.
   * **3126.5 / 3126.53** — ``toss_fit_lib`` / ``toss_cal_grid``, a 4-s.f.
     rounding at "h = 0.78" with no T. Reproduces no geometry exactly; those
     headers now say so, and nothing there turns on the fifth digit.

A fourth number, ``aim_target_offset_mm``'s **54.578 mm/deg**, is a different
QUANTITY altogether — the SECANT gain to a full 1° aim, larger by
``tan(1°)/1°`` (1.0001016) plus the tilted-release drop, ratio measured
**1.0001044** — against this module's derivative-at-zero **54.5718 mm/deg** at
the same h = 0.78. Both are correct; they are not interchangeable. A sizing
argument about a clamp wants the secant (``toss_cal.TOTAL_MAX_RAD``'s "55 mm at
1°"); a linearised update law wants the derivative, and :func:`sensitivity`
reports the derivative because that is what a Jacobian is.

WHAT THE MEASURED CORPUS SAYS, IN ONE PLACE
-------------------------------------------
19 rows under ``usable_for_release_fit`` across the three mocap-bearing bags
(re-mined 2026-08-12 22:0x). One commanded flight time, 0.9032 s (apex exactly
1.000 m), three resolvable goal cells and two rows whose cup is unrecoverable.

* ``release_speed_err_mms``  mean **+455.4**, sd 58.3, se 13.4 (+10.3 % of 4436)
* ``flight_time_err_s``      mean **+0.0975**, sd 0.0139, se 0.0032 (+10.8 % of 0.903)
* the two agree on the **bias** to 5 % through the production ``2/g``
  relation (+0.0929 s predicted from the speed error against +0.0975 measured)
  and yet their per-toss **scatter** correlates at only **r = 0.23** — so
  roughly three quarters of the toss-to-toss spread in each is that channel's
  own measurement noise, not plant variation. That decomposition is where
  :data:`SIGMA_E`'s numbers come from, and it independently reproduces the
  miner's own admission gate: 58.3·√0.77 = **51 mm/s** against
  ``RELEASE_FIT_MAX_SE_MMS`` = 50.

**The provenance caveat, stated once and loudly**: 16 of those 19 rows come from
``2026-08-10_16-30-44``, recorded at **16.7 h of can-bridge uptime** — deep in
the regime gate G-1 exists to close. This module therefore ships
:func:`uptime_census` and an OPTIONAL ``uptime_max_ms`` refusal, defaulting to
**no refusal**, because inventing a healthy-uptime threshold here would be
inventing a safety bound that belongs to G-1. At any threshold below 16.7 h the
present corpus loses 16 of 19 rows; that is a fact about the corpus, and the
fit prints it rather than deciding it.

THE PARTITION RULE, INHERITED
-----------------------------
:func:`partition_census` / :func:`select_partition` are ``toss_fit_lib``'s rule
re-used verbatim in intent: a corpus pooling two plants yields a correction that
is the weighted average of two machines and it *looks* fine. Pooling is refused
without an explicit flag and the census is always reported.
"""

from __future__ import annotations

import functools
import json
import math
import os
import sys
from typing import Any, Dict, Iterable, List, NamedTuple, Optional, Sequence, Tuple

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.normpath(os.path.join(_HERE, os.pardir, os.pardir))
for _p in (os.path.join(_REPO, 'ros_ws', 'src', 'jugglebot'),
           os.path.join(_REPO, 'config', 'generated'),
           os.path.join(_REPO, 'tools', 'probes')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import jugglebot.hardware_config as hw                              # noqa: E402
from jugglebot import toss_trim                                     # noqa: E402
from jugglebot.motion import toss_cal                               # noqa: E402
from jugglebot.motion.trajectory import ballistics_bc               # noqa: E402
from jugglebot.motion.trajectory import throw_envelope              # noqa: E402
from jugglebot.motion.trajectory.toss_release import (              # noqa: E402
    ThrowTiltInfeasible,
    aim_target_offset_mm,
    apex_height_from_flight_time,
    compute_release_state,
    compute_release_state_tilted,
    validate_event_vel,
)
from jugglebot import toss_sequencer                                # noqa: E402
from jugglebot.toss_sequencer import (                              # noqa: E402
    FLIGHT_TIME_MAX_S,
    FLIGHT_TIME_MIN_S,
    TOSS_ACTIVE_Z_MM,
    TOSS_XY_LIMIT_MM,
    TOSS_Z_BAND_MM,
)
# The MINER, for the one definition it owns: the per-axis lean that the mined
# ``arrival_dir_err_rad`` / ``release_dir_err_rad`` fields are differences of.
# Re-deriving those five lines here would make the model's direction channel and
# the measured one two conventions instead of one — the exact failure design
# constraint 1 names.
import toss_record_miner as _miner                                  # noqa: E402

#: The E-1 lateral-admission threshold, RE-EXPORTED from the miner rather than
#: copied. The miner owns it because the miner is what measures
#: ``coverage_asym_s``; this module needs the name so a CLI and a test can print
#: and pin the same number without reaching into a private import.
COVERAGE_ASYM_MAX_S = _miner.COVERAGE_ASYM_MAX_S

__all__ = [
    'IlcFitError', 'TossGoal',
    'U_CHANNELS', 'U_LABELS', 'N_U', 'CATCH_CHANNELS',
    'E_LABELS', 'N_E', 'E1_BLOCKED', 'E1_MASK', 'DEFAULT_MASK', 'SIGMA_E',
    'AIM_CHANNELS', 'MONITOR_CHANNELS', 'MONITOR_MASK', 'FULL_MASK',
    'GUARD_SCOPE', 'GUARD_ROW_REASONS', 'GUARD_LAND_ERR_REASONS',
    'GUARD_DECLARATION_GAP_REASONS', 'GUARD_SELF_BLINDING_REASONS',
    'GuardVerdict', 'guard_verdict', 'land_err_admissible',
    'channel_disagreement', 'disagreement_census',
    'evidence_gate', 'EVIDENCE_PASS', 'EVIDENCE_THIN', 'EVIDENCE_INSIDE_SE',
    'EVIDENCE_FROZEN',
    'TIER_8B', 'THROW_SITE_KEY_TOL_MM', 'throw_site_xy_of',
    'throw_site_admissible',
    'FD_STEPS', 'TAU0', 'AUTHORITY', 'ILC_SPEED_AUTHORITY', 'RHO_DAMPING',
    'speed_authority_band', 'speed_authority_band_for_goal',
    'MAX_TRUST_SHRINKS', 'lateral_admissible', 'COVERAGE_ASYM_MAX_S',
    'REPEATABILITY_MIN', 'zero_command', 'catch_channel',
    'release_state_for_command', 'e_model', 'release_speed_err_model',
    'sensitivity', 'conditioning', 'screen_channels',
    'weight_matrix', 'solve_step', 'admit_command', 'propose_step', 'iterate',
    'load_corpus', 'admit_record', 'measured_error', 'goal_key', 'goal_of',
    'partition_key', 'partition_census', 'census_lines', 'select_partition',
    'uptime_census', 'group_by_goal', 'repeatability', 'held_out_prediction',
    'channel_sensitivity', 'required_command', 'authority_report',
    'measurement_plane_mm',
    'cross_check_replay', 'pooled_error', 'fit_corpus',
    'synthetic_corpus', 'PARTITION_KEYS', 'TOOL_NAME',
]

TOOL_NAME = 'tests/hardware/ilc_fit.py'


class IlcFitError(RuntimeError):
    """Any refusal to model, fit, step or write. Carries the operator-facing reason.

    A distinct type from ``toss_fit_lib.TossFitError`` (which is an alias of
    ``jugglebot.toss_trim.TossTrimError``) because nothing here is caught by that
    module's handlers and conflating them would make an ILC refusal look like an
    aim-map refusal in a traceback.
    """


# ═════════════════════════════════════════════════════════════════════════════
# The command vector u (§ Command vector and task error)
# ═════════════════════════════════════════════════════════════════════════════


class UChannel(NamedTuple):
    """One command channel: what it is, where it enters the machine, how far it
    may go, and how big a step the finite difference takes."""
    name: str
    unit: str
    seam: str            # the production call/line the channel rides
    authority: float     # accumulated |u_j| bound, in the channel's unit
    authority_src: str   # where that bound comes from
    fd_step: float       # central-difference half-step
    tau0: float          # proposed per-iteration trust region (PROVISIONAL)


#: **The ILC's own accumulated-``event_vel_trim`` authority. OWNER DECISION,
#: 2026-08-13, Gate 1** (``plans/active/critical-point-ilc.md``, "Gate 1 CLOSED
#: 2026-08-13 (owner decisions): (1) the speed-authority question below is
#: DECIDED as option (a) — an ILC-specific ±0.15 authority").
#:
#: It exists because the measured corpus asks for more than ``toss_trim``'s
#: ±0.10: the pooled vertical residual requires ``event_vel_trim = −0.1076``,
#: and that is not a pooling artefact — per goal cell it is −0.096 / −0.112 /
#: −0.124 (n = 8/6/3), so two of three cells exceed ±0.10 on their own.
#:
#: **``toss_trim.SPEED_AUTHORITY`` IS NOT CHANGED and must not be.** That
#: constant bounds the SESSION TRIM, a different loop with a different update
#: law, a different measurand and a different operator model; widening it here
#: would silently widen that one. This is a second, named bound, owned by this
#: module, applied to this module's channel.
#:
#: ⚠ **DEMOTED 2026-08-21 to an OUTER CEILING. It is no longer the authority.**
#: The authority is :func:`speed_authority_band`, derived per flight time from
#: ``throw_envelope.evaluate`` — contradiction **C2** of the ILC-primary fold-in
#: (``plans/active/critical-point-ilc.md`` § "The 2026-08-21 fold-in", build
#: step 1). This constant survives as the ceiling the band is intersected with,
#: so a future envelope that opened up cannot silently widen a learned trim past
#: the number the owner actually approved.
#:
#: **The safety argument that used to stand here is FALSE — do not reinstate
#: it.** It read, in its third bullet: *"the [0.3, 7.0] m/s wire band is
#: unreachable anywhere in the sequencer's flight-time band, measured by rail
#: sweep at T = 0.55 s (2.30 m/s, 7.7× clear of the floor) and T = 1.10 s
#: (6.21 m/s, 1.13× inside the ceiling)"*. Both halves of its premise moved:
#:
#: * the flight-time band is no longer the hand-picked [0.55, 1.10] s — it is
#:   **derived**, ``[0.4949, 1.1485] s`` (``throw_envelope.MIN/MAX_FLIGHT_TIME_S``,
#:   contract C-HAND-3); and
#: * the wire band **bounds nothing physical** (``toss_trim.SessionTrim.
#:   speed_gain``'s own ⚠ block). The real gate is ``throw_envelope.evaluate``,
#:   seven bounds, and it refuses long before [0.3, 7.0] does.
#:
#: Measured against it (``tools/probes/ilc_speed_band.py``, run 2026-08-21), the
#: admissible ``k_v − 1`` is ``[+0.000, ≥+0.5]`` at T = 0.4949 s (negative side
#: bounded by ``ARM_WINDOW``), ``[≤−0.5, +0.270]`` at 0.9032, ``+0.148`` at 1.00,
#: ``+0.043`` at 1.10 and ``+0.000`` at 1.1485 (positive side bounded by
#: ``DECEL_FF_HEADROOM``). So ±0.15 is inadmissible near BOTH ends, and Gate 1's
#: "neither rail is reachable anywhere in the band" was false on this machine.
#:
#: What survives of the Gate-1 argument, and why the ceiling is still worth
#: keeping: the measured corpus asks for ``event_vel_trim = −0.1076`` and per
#: cell −0.096 / −0.112 / −0.124 (n = 8/6/3), so two of three cells exceed
#: ``toss_trim``'s ±0.10 alone. **``toss_trim.SPEED_AUTHORITY`` IS NOT CHANGED
#: and must not be** — that constant bounds the SESSION TRIM, a different loop
#: with a different update law and a different measurand.
ILC_SPEED_AUTHORITY = 0.15

#: Bisection depth for :func:`speed_authority_band`. 60 halvings of a 0.15-wide
#: bracket is ~1.3e-19 — far past double precision, i.e. exact — and the cost is
#: paid once per (T, v) pair thanks to the cache below.
_BAND_ITERS = 60


@functools.lru_cache(maxsize=512)
def speed_authority_band(flight_time_s: float, nominal_speed_mps: float,
                         ceiling: float = ILC_SPEED_AUTHORITY
                         ) -> Tuple[float, float]:
    """``(lo, hi)`` — the ADMISSIBLE ``event_vel_trim`` interval at this goal.

    **This is the ILC speed channel's authority** (C2). It is the connected
    interval of ``k_v − 1`` around the nominal command for which the REAL gate,
    ``throw_envelope.evaluate(T, v_nominal·(1+k))``, says yes — intersected with
    the :data:`ILC_SPEED_AUTHORITY` outer ceiling. Both returned edges SATISFY
    the gate (the bisection returns the admitted side, ``throw_envelope._bisect``'s
    own doctrine and for the same reason: an edge a float epsilon on the refused
    side is an edge every consumer that samples it then fails on).

    Why an interval search rather than a formula: the admitted release-speed set
    is an interval, but its two ends close for DIFFERENT reasons — ``END_STOP`` /
    ``DECEL_FF_HEADROOM`` / ``ACCEL_AUTHORITY`` / ``REGEN`` above, and
    ``ARM_WINDOW`` below — and ``ARM_WINDOW``'s dependence on speed is the
    counter-intuitive one: ``earliest = throw_decel_s(v) + ARM_SUPPRESS_MARGIN_S``
    and ``throw_decel_s`` GROWS as the release speed falls, so a *slow-down* trim
    narrows the catch-arm window. At the derived band floor that window is
    exactly ``ARM_WINDOW_MARGIN_S`` wide by construction, so the negative side is
    ``+0.000`` there. That mechanism was probed, not argued
    (``tools/probes/ilc_speed_band.py``); the first reading of it — "a slower
    throw shortens the achieved flight" — was wrong, and ``evaluate`` never
    re-derives a flight time from a speed.

    **Fails closed**: a non-finite or non-positive input, or a nominal command
    the envelope already refuses, returns ``(0.0, 0.0)`` — no trim authority at
    all. A goal the machine cannot fly untrimmed is not one a learned trim gets
    to reach into.
    """
    ceil = abs(float(ceiling))
    T = float(flight_time_s)
    v0 = float(nominal_speed_mps)
    if not (math.isfinite(T) and T > 0.0 and math.isfinite(v0) and v0 > 0.0):
        return (0.0, 0.0)
    if not throw_envelope.evaluate(T, v0).ok:
        return (0.0, 0.0)
    return (-_band_edge(T, v0, -1.0, ceil), _band_edge(T, v0, +1.0, ceil))


def _band_edge(T: float, v0: float, sign: float, ceil: float) -> float:
    """Largest ``s`` in ``[0, ceil]`` with ``evaluate(T, v0·(1+sign·s))`` ok."""
    if ceil <= 0.0:
        return 0.0
    if throw_envelope.evaluate(T, v0 * (1.0 + sign * ceil)).ok:
        return ceil
    lo, hi = 0.0, ceil
    for _ in range(_BAND_ITERS):
        mid = 0.5 * (lo + hi)
        if throw_envelope.evaluate(T, v0 * (1.0 + sign * mid)).ok:
            lo = mid
        else:
            hi = mid
    return lo


def speed_authority_band_for_goal(goal: 'TossGoal') -> Tuple[float, float]:
    """:func:`speed_authority_band` at ``goal``'s OWN nominal commanded speed.

    Not the Tier-8a vertical projection: an aimed or displaced goal releases
    faster at the same flight time, and the band is a property of the speed the
    machine will actually command. Zero aim and zero trim, so this is the
    untrimmed command the trim is a multiple of.
    """
    _n, _c, _l, v_nom = release_state_for_command(zero_command(), goal)
    return speed_authority_band(float(goal.flight_time_s), float(v_nom))

#: THE throw-side command vector. Order is load-bearing: it indexes every ``u``
#: array, every column of ``F`` and every screen verdict in this module.
#:
#: **aim_rx / aim_ry** ride the aim map's own seam exactly — the value is a
#: commanded tilt in radians, converted to a VIRTUAL TARGET displacement by
#: ``aim_target_offset_mm`` and handed to ``compute_release_state_tilted``, which
#: is what ``reload_coordinator_node._build_toss_cycle`` does at lines 2459-2483.
#: The ILC correction is therefore *the same kind of object* as an aim-map cell,
#: which is what makes the plan's eventual "absorb the aim map" decision a
#: question about update laws rather than about representations.
#:
#: **event_vel_trim** is ``k_v − 1`` in ``toss_trim``'s parametrisation. Its
#: authority was that module's ``SPEED_AUTHORITY`` verbatim until Gate 1 closed;
#: it is now :data:`ILC_SPEED_AUTHORITY`, an ILC-specific ±0.15 the owner decided
#: on 2026-08-13 with the rail sweep in front of them. See that constant for the
#: whole argument, and the module docstring for why the multiply is the one
#: non-production operation in the file.
#:
#: **release_timing_offset** is seconds of shift on the dispatch's
#: ``event_delay`` (``hw.JB_OP_TOSS_RELEASE_LATENCY_MS`` is the shipped
#: constant, 0.0). REFUSED from v1 — see the module docstring; its column of
#: ``F`` is computed and comes back zero, but STRUCTURALLY so (the model has no
#: δt dependence to difference), which is a weaker statement than a measurement.
U_CHANNELS: Tuple[UChannel, ...] = (
    UChannel(
        name='aim_rx', unit='rad',
        seam=('toss_cal.clamp_total_aim -> toss_release.aim_target_offset_mm '
              '-> virtual target -> compute_release_state_tilted '
              '(reload_coordinator_node._toss_aim_for_goal / '
              '_build_toss_cycle)'),
        authority=toss_cal.TOTAL_MAX_RAD,
        authority_src='toss_cal.TOTAL_MAX_RAD (1.0 deg, D7 apply-time clamp)',
        # 1e-5 rad = 0.57 mdeg, 1/3000 of the authority. The offset is analytic
        # and odd through zero, so the central difference is exact to ~1e-10
        # relative at any step this small; toss_trim.aim_landing_jacobian uses a
        # 1e-6 FORWARD step for the same reason.
        fd_step=1e-5,
        # TOTAL_MAX_RAD / 3: full authority reachable in the k <= 3 iterations
        # Phase 3's success criterion is written against.
        tau0=toss_cal.TOTAL_MAX_RAD / 3.0),
    UChannel(
        name='aim_ry', unit='rad',
        seam='as aim_rx',
        authority=toss_cal.TOTAL_MAX_RAD,
        authority_src='toss_cal.TOTAL_MAX_RAD (1.0 deg, D7 apply-time clamp)',
        fd_step=1e-5,
        tau0=toss_cal.TOTAL_MAX_RAD / 3.0),
    UChannel(
        name='event_vel_trim', unit='dimensionless (k_v - 1)',
        seam=('multiplies release_cmd.event_vel_mps — '
              'toss_trim.SessionTrim.speed_gain\'s documented semantic; '
              'NOT WIRED in production (Phase 2 wires it)'),
        authority=ILC_SPEED_AUTHORITY,
        authority_src=('ilc_fit_lib.ILC_SPEED_AUTHORITY (+-15 %, owner '
                       'decision 2026-08-13 Gate 1; toss_trim.SPEED_AUTHORITY '
                       'is +-10 % and is deliberately NOT changed)'),
        # 1e-4 = 0.01 %, 1/1500 of the authority and ~4x the double-precision
        # noise floor of the two ballistic solves it passes through.
        fd_step=1e-4,
        # 0.040, and it is a bracket rather than a taste — APPROVED as proposed
        # in the Gate-1 sizing memo (owner, 2026-08-13). FLOOR: never chase
        # noise — 2x the noise-equivalent command (2 * se(flight-time mean)
        # 0.0032 s / 0.906 s per unit) = 0.007. CEILING: never let one step
        # spend the whole authority — 0.15 since the memo, and 0.040 was already
        # inside the tighter 0.10 the bracket was drawn against, so the widening
        # does not move it. INSIDE that, the binding constraint is Phase 3's
        # "k <= 3 iterations" criterion against the MEASURED requirement of
        # 0.1076, i.e. tau >= 0.0359. 0.040 is the smallest round value that
        # meets it, and smallest is the right end of the bracket to sit on: a
        # smaller step is less exposure to a wrong F.
        tau0=0.040),
    UChannel(
        name='release_timing_offset', unit='s',
        seam=('reload_coordinator_node._dispatch_toss_throw: event_delay -= '
              'hw.JB_OP_TOSS_RELEASE_LATENCY_MS/1000 (ships 0.0) — NOT routed '
              'through this model; see the module docstring'),
        authority=toss_trim.TAU_AUTHORITY_MS / 1e3,
        authority_src='toss_trim.TAU_AUTHORITY_MS (+-150 ms)',
        # 1 ms — 1/900 of the flight time, 1/150 of the authority.
        fd_step=1e-3,
        tau0=toss_trim.TAU_STEP_MAX_MS / 1e3),
)

U_LABELS: Tuple[str, ...] = tuple(c.name for c in U_CHANNELS)
N_U = len(U_CHANNELS)
FD_STEPS = np.array([c.fd_step for c in U_CHANNELS], dtype=float)
TAU0 = np.array([c.tau0 for c in U_CHANNELS], dtype=float)
AUTHORITY = np.array([c.authority for c in U_CHANNELS], dtype=float)

#: The catch-side half of § Command vector, **declared and not implemented**.
#: v1 is throw-side by the phase's own scope, and two prerequisites are open, not
#: merely unbuilt: **G-2** (the hand-ODrive braking clamp — ~-10 A delivered
#: against ~-26 A commanded — sits directly in the catch-softness path, so
#: learning against it fits the clamp), and **0b outcome (ii)**: the measured
#: contact transient did NOT separate from its matched cross-label control
#: (1.03x on ``vel_meas``, 1.45x on the ``iq`` impulse), so the *validation*
#: channel for a modelled contact-velocity surrogate does not exist yet.
#: :func:`catch_channel` raises rather than returning a zero, because a silently
#: zero catch channel is a fit that reports it optimised something it never
#: touched.
#:
#: **``catch_timing_offset`` was considered as the first catch channel on
#: 2026-08-21 (owner decision 5) and DECLINED, with the residual written down.**
#: G-2 has since closed (``b084f98``, the hand's braking clamp was the legs' Kt
#: in ``torque_soft_min``), so the decision hangs on the other prerequisite, and
#: three things have to be true for a channel to be a channel here:
#:
#: 1. **a measured residual.** This one exists and is clean: the catch edge is
#:    DEBOUNCE-FREE (``toss_record``'s own header — the 241 ms asymmetry is on
#:    the DEPARTURE edge), so ``t_catch_deb_ros`` is a real instant, and the
#:    residual is
#:
#:        ``catch_time_err_s = (t_catch_deb_ros − t_land_bag) − event_delay_s``
#:
#:    i.e. measured seat instant minus plane-crossing instant, minus the delay
#:    the catch was ARMED with at ``catch_coordinator_node._arm_hand_catch``.
#:    Every one of those three fields is already mined on every row.
#: 2. **an analytic ``∂e/∂u`` through the production chain** (design constraint
#:    1: never a symbolic twin). This is the one that fails. ``e_model`` is
#:    release-side and analytic to the plane — ballistic flight plus the
#:    ``hand_stroke`` closed form — and it stops there. The seat instant is a
#:    function of the hand's DESCENT profile, which lives in the Teensy's
#:    ``calcCatch`` geometry; differentiating it would mean importing or
#:    re-deriving firmware, and re-deriving it is exactly the twin the constraint
#:    forbids. Note the shape of the failure: the column would not be
#:    structurally zero the way ``release_timing_offset``'s is — it would be
#:    *unmodelled*, which is worse, because a finite-difference through a chain
#:    the model does not contain returns a confident wrong number instead of a
#:    zero the screen can exclude.
#: 3. **a σ for the new ``e`` row.** Never measured; and the only corpus that
#:    exists cannot legitimately supply it, because ``partition_key`` includes
#:    ``bridge_fw_version`` and every row predates FW 14 (C6).
#:
#: And the cadence work makes (2) worse, not better: the true 0.25 s dwell (R6)
#: is a DEFERRED FIRMWARE FORK of ``calcCatch``, so the very geometry this
#: channel would differentiate is the one scheduled to change. Building the
#: channel now would fit a descent profile the fork replaces.
#:
#: So: declared, unimplemented, residual definition recorded above, and the
#: unblocking condition is a modelled descent — not more hardware time.
CATCH_CHANNELS: Tuple[Tuple[str, str, str], ...] = (
    ('catch_pose_dx', 'mm',
     'catch_coordinator._compute_catch_command reach target xy'),
    ('catch_pose_dy', 'mm',
     'catch_coordinator._compute_catch_command reach target xy'),
    ('catch_event_vel_trim', 'dimensionless',
     'catch_coordinator_node._arm_hand_catch req.event_vel'),
    ('catch_timing_offset', 's',
     'catch_coordinator_node._arm_hand_catch event_delay'),
)


def catch_channel(name: str):
    """Refuse a catch-side channel, loudly and by name (v1 is throw-side)."""
    known = {c[0] for c in CATCH_CHANNELS}
    if name not in known:
        raise IlcFitError(
            'unknown command channel {!r} — throw side is {}, catch side is {}'
            .format(name, list(U_LABELS), sorted(known)))
    extra = ''
    if name == 'catch_timing_offset':
        extra = (' This one was re-examined on 2026-08-21 (owner decision 5) '
                 'and DECLINED for a NAMED reason, not for time: its residual '
                 'is clean and already mined — catch_time_err_s = '
                 '(t_catch_deb_ros - t_land_bag) - event_delay_s — but there is '
                 'no analytic d(seat instant)/du, because the seat is a '
                 'function of the Teensy calcCatch descent geometry that '
                 'e_model does not contain, and finite-differencing through a '
                 'chain the model lacks returns a confident wrong column rather '
                 'than a zero the screen can exclude. See CATCH_CHANNELS.')
    raise NotImplementedError(
        'catch-side channel {!r} is DECLARED, NOT IMPLEMENTED in v1. The '
        'prerequisite is Phase 0b outcome (ii) — the measured impact transient '
        'did not separate from its matched cross-label control, so the modelled '
        'contact-velocity surrogate has no validation channel yet. (G-2, the '
        'hand-ODrive braking clamp, was the other one and CLOSED 2026-08-18, '
        'b084f98.) Seam, for whoever implements it: {}.{}'
        .format(name, dict((c[0], c[2]) for c in CATCH_CHANNELS)[name], extra))


def zero_command() -> np.ndarray:
    """The nominal command — today's machine, exactly (all four channels 0)."""
    return np.zeros(N_U, dtype=float)


# ═════════════════════════════════════════════════════════════════════════════
# The task error e (§ Command vector and task error, throw critical point)
# ═════════════════════════════════════════════════════════════════════════════

#: The five throw-critical-point channels, in the plan's own order. Each is
#: defined to be **bit-for-bit the same measurand** as the mined field beside it,
#: because the fit differences a model against a measurement and any convention
#: gap between them is indistinguishable from plant error:
#:
#:   0,1  ``land_err_mm``          landing xy minus the announced cup xy
#:   2,3  ``arrival_dir_err_rad``  lean(arrival) minus lean(nominal arrival)
#:   4    ``flight_time_err_s``    achieved minus commanded flight time
E_LABELS: Tuple[str, ...] = ('land_err_x', 'land_err_y',
                             'arrival_dir_x', 'arrival_dir_y',
                             'flight_time_err')
N_E = len(E_LABELS)

#: **HISTORICAL** — the channels entry condition E-1 excluded from FITTING
#: between 2026-08-12 and 2026-08-13, kept by name because the plan, the logbook
#: and this module's own screen all refer to it, and because a reader needs to be
#: able to reproduce the masked answer from the same corpus.
E1_BLOCKED: Tuple[str, ...] = ('land_err_x', 'land_err_y',
                               'arrival_dir_x', 'arrival_dir_y')
#: 1.0 where a channel was fitted under E-1, 0.0 where E-1 blocked it.
E1_MASK = np.array([0.0 if lbl in E1_BLOCKED else 1.0 for lbl in E_LABELS],
                   dtype=float)

#: The two channels that DRIVE the aim columns of ``u``. Owner decision 6 of the
#: 2026-08-21 ILC-primary fold-in: **the whole-arc arrival direction is the
#: primary and only aim residual.**
AIM_CHANNELS: Tuple[str, ...] = ('arrival_dir_x', 'arrival_dir_y')

#: The plane-position channels — **MONITOR ONLY since 2026-08-21** (decision 6,
#: closing contradiction **C3**). Mined, recorded, reported and disagreement-
#: logged on every toss; weighted **zero** in the update law.
#:
#: Root cause, not the decision by name. ``land_err_mm`` is a POSITION fit at the
#: catch plane, so it carries the mocap visible-centroid bias ``b(z)``
#: ABSOLUTELY; ``arrival_dir_err_rad`` is a whole-arc VELOCITY and is bias-immune
#: by parity (a position bias is EVEN in ``tau = t − t_apex``, any aerodynamic
#: force is ODD). E-1 measured only the bias GRADIENT — ``b_y`` −23.3 mm at
#: z = 882 falling to −2.5 mm at z = 1882, r = 0.998 over 19 arcs — never the
#: absolute offset, and the two channels therefore disagree SYSTEMATICALLY:
#: +18.0 mm in y pooled (+17.4 / +19.5 / +15.4 across the three goal cells)
#: against 0.85 mm in x. ``weight_matrix``'s ``Q`` arbitrated that as though it
#: were noise, which is the one thing it is not.
#:
#: **Why the discriminator was not run.** The planned resolution was H2, a 20 min
#: static fixtured-ball capture with conventional markers. Owner pushback
#: 2026-08-21: markers on the ball corrupt the very trackable surface being
#: measured, so H2 cannot measure the bias of the unmarked ball. The mechanism
#: the owner reads instead — platform-frame occlusion near the bottom of the
#: stroke — is what the arc's own data already says: the bias is large at
#: z ≈ 880 (the catch-plane height) and has vanished by z ≈ 1880, and it is
#: parity-EVEN. So C3 is resolved BY DECISION on the evidence in hand, and the
#: bias-immune channel takes the loop.
#:
#: **What closes ABSOLUTE centering, since the direction channel cannot.** An
#: arrival-direction loop converges the ball onto the cup the *arc* points at; a
#: constant registration offset between the mocap frame and the physical cup is
#: invisible to it. That closes through CATCH OUTCOMES — the penalty loop is the
#: ground truth for "centered on the cup" — and a residual ~10 mm registration
#: bias against the 35 mm capture radius is tolerable and visible in the penalty
#: trend.
#:
#: **The standing validation** is :func:`channel_disagreement`, logged per toss:
#: if the arrival_dir-driven loop converges while the plane residual holds the
#: known ``b(z)`` profile shape, the model is confirmed. If catch rate plateaus
#: with a converged aim, the decision is wrong and gets revisited.
MONITOR_CHANNELS: Tuple[str, ...] = ('land_err_x', 'land_err_y')

#: 1.0 on a monitor channel, 0.0 elsewhere — the complement of the default mask.
MONITOR_MASK = np.array([1.0 if lbl in MONITOR_CHANNELS else 0.0
                         for lbl in E_LABELS], dtype=float)

#: All five channels weighted. **HISTORICAL / diagnostic only** since decision 6
#: — it is what :data:`DEFAULT_MASK` was between 2026-08-13 and 2026-08-21, kept
#: by name so the pre-decision answer stays reproducible from the same corpus
#: (the same service :data:`E1_MASK` performs for the pre-2026-08-13 answer).
#: **Do not pass it to a fit that will be written to an artifact.**
FULL_MASK = np.ones(N_E, dtype=float)

#: **THE MODULE-WIDE DEFAULT MASK.** ``[0, 0, 1, 1, 1]`` since 2026-08-21 —
#: full-size between 2026-08-13 (E-1 closed) and then.
#:
#: E-1 CLOSED 2026-08-13. Owner-adopted resolution, evidence in
#: ``plans/active/critical-point-ilc.md`` § Phase 1 E-1 and reproducible from
#: ``tools/probes/mocap_parity_bias.py``: the lateral branch-to-branch artefact
#: was a height-locked **measurement** bias (the tracked point is the centroid of
#: the visible retroreflective cap), CONFIRMED by parity decomposition about the
#: apex — a position bias is EVEN in ``tau = t - t_apex`` while any aerodynamic
#: force is ODD, and the measured even part repeats across 19 arcs, three object
#: positions and two sittings at 1.45 mm cross-arc sd (pairwise r = 0.998). A
#: bias profile fitted on OTHER arcs collapses the branch delta from −104.4 mm/s
#: to 13.1 mm/s leave-one-out. Magnus was refuted twice (parity, and a
#: data-derived aero bound); spin was refuted by residual periodicity 40–60× too
#: small.
#:
#: The fix is in the MINER, not here: ``toss_record_miner.mine_arc`` takes every
#: lateral velocity from a WHOLE-ARC fit, which is bias-immune by parity, and
#: mines ``coverage_asym_s`` — the residual leak's driver — with a refusal past
#: ``COVERAGE_ASYM_MAX_S``. So closing E-1 IS a mask change here, exactly as the
#: plan said it would be, and this constant is that change.
#:
#: **The admission requirement that makes it safe.** A corpus glob
#: (``temp/probes/toss_records_*.jsonl``) happily matches files mined BEFORE the
#: whole-arc estimator, whose lateral channels are the artefact itself — 10.9
#: mrad ≈ 44 mm of phantom aim per toss (0.0109 rad through the 4007 mm/rad
#: ``4h + dz`` landing gain at the corpus goal), repeatable, so it would not
#: average away. :func:`admit_record` therefore requires a present, in-bound
#: ``coverage_asym_s`` before any lateral channel is read, and
#: :func:`measured_error` returns ``nan`` in the lateral channels of a row that
#: fails it. One enforcement point each, and a stale file loses its lateral
#: channels rather than the whole corpus.
#:
#: **The 2026-08-21 change, and the one safety question it had to answer.**
#: Zeroing two of the four lateral entries removes SNR from the aim columns, and
#: an aim column that falls below :data:`SCREEN_SNR_MIN` would be EXCLUDED by
#: :func:`screen_channels` — i.e. the decision would silently switch the aim
#: channel off rather than re-source it. Measured at the corpus goal
#: (``/tmp/probe_mask_design.py``, run 2026-08-21): the whitened aim-column norm
#: is 2.9207 under :data:`FULL_MASK`, of which ``land_err`` contributes 1.5857
#: and ``arrival_dir`` 2.4527. Under this mask it is **2.4527**, and under this
#: mask WITH the D2-corrected sigma (0.00302) it is **1.9330** — both far above
#: the 1.0 floor, and the retained set is unchanged
#: (``aim_rx``, ``aim_ry``, ``event_vel_trim``). ``release_timing_offset`` stays
#: EXCLUDED for the reason it always was: its column is structurally zero.
DEFAULT_MASK = FULL_MASK - MONITOR_MASK

#: Per-channel measurement noise, in each channel's own unit. **PROVISIONAL**
#: until a corpus larger than 19 rows exists, in the convention
#: ``toss_fit_lib.N_MIN`` already uses. Every number is measured on the
#: 2026-08-12 corpus, not assumed:
#:
#: * ``land_err``  — per-axis sd 15.7 / 13.6 mm over the 17 rows with a landing
#:   fit; pooled to one isotropic 14.7 mm rather than split, because the physics
#:   is isotropic and two numbers at n = 17 is fitting the axes.
#: * ``arrival_dir`` — per-axis sd 0.00296 / 0.00308 rad, pooled to **0.00302**.
#:   (Pre-E-1-re-mine it was 0.00267 / 0.00203 → 0.00238; see the D2 block
#:   below for why the number moved and why it is now the one in this array.)
#: * ``flight_time`` — sd 0.0139 s. Deliberately the RAW scatter and not the
#:   0.0122 s the 23 %-plant / 77 %-noise decomposition implies: a Q weight that
#:   under-states the noise over-trusts the channel, and the decomposition rests
#:   on one correlation coefficient at n = 19.
#:
#: **D2 — RE-DERIVED 2026-08-21. ``arrival_dir`` is 0.00302, not 0.00238.**
#: The E-1 re-mine changed the lateral estimator (per-branch → whole-arc), so the
#: channel's measured scatter moved with it: on the re-mined corpus the per-axis
#: sd is 0.00296 / 0.00308, pooled **0.00301993 rad**, 27 % above the 0.00238
#: this array carried. ``land_err`` (14.7313 vs 14.7) and ``flight_time``
#: (0.0138473 vs 0.0139) are unchanged, because their estimators were.
#: Reproduce: ``/tmp/probe_ilc_sigma.py``, run 2026-08-21 over the three
#: newest-mine corpus files, and re-derivable from the committed
#: ``tests/hardware/ilc_corpus_fixture.py`` since C8 landed.
#:
#: **The population it belongs to, which is not the population it is now applied
#: over.** 0.00302 is the pooled sd over the **17** rows carrying BOTH lateral
#: channels, because ``lateral_admissible`` required ``land_err_mm`` until
#: 2026-08-21. Now that the primary channel no longer depends on a monitor one
#: (decision 6), **19** arrival directions are readable and the same statistic
#: reads **0.00286** — 5.4 % lower. This array keeps 0.00302, i.e. it slightly
#: OVER-states the noise, which is the conservative direction and this array's
#: own stated doctrine one bullet up: a ``Q`` weight that under-states the noise
#: over-trusts the channel. Both numbers are pinned by
#: ``test_the_committed_fixture_reproduces_the_headline_numbers``.
#:
#: It was left stale on 2026-08-13 because Gate 1 had approved
#: ``Q = diag(1/sigma^2)`` with the old numbers and re-deriving an approved
#: weight is an owner decision. That decision was taken on 2026-08-21 (fold-in
#: decision 6, D2), and **the cost it was weighed against has since collapsed to
#: zero in the aim channels** — which is the part worth writing down:
#:
#: * the bounded cost of the stale entry was 0.00997 → 0.01044 rad of pooled aim
#:   requirement (57.1 % → 59.8 % of the D7 authority). Every millimetre of that
#:   was ``Q`` ARBITRATING ``arrival_dir`` against ``land_err`` — the sigma ratio
#:   is what decides how a systematic +18 mm disagreement is split.
#: * Decision 6 removes the arbitration: with :data:`MONITOR_MASK` zeroed out of
#:   the update, ``aim_rx`` is driven by ``arrival_dir_y`` alone and ``aim_ry``
#:   by ``arrival_dir_x`` alone (``F`` is exactly this sparse — see
#:   :func:`sensitivity`), so a common scale on the only two channels driving
#:   those columns cancels out of ``(FᵀQF)⁻¹FᵀQe`` entirely. Measured: the pooled
#:   aim requirement is **0.008717 rad at BOTH sigmas**, bit-identical.
#: * What the value still changes is the DAMPED step, because ``R = diag(ρ/τ²)``
#:   is in scaled coordinates and does not cancel, and the SNR the screen reads
#:   (2.4527 → 1.9330, both above the 1.0 floor). So the number matters; the
#:   57.1/59.8 framing does not survive decision 6 and is recorded here as
#:   superseded rather than deleted.
#:
#: **The gap the weights WERE arbitrating is NOT noise — and since 2026-08-21
#: they no longer arbitrate it** (:data:`MONITOR_CHANNELS`, decision 6). Kept
#: verbatim because it is the measurement the decision rests on.
#: The two lateral channels disagree SYSTEMATICALLY, in y only,
#: by +18.0 mm at the plane (pooled; +17.4 / +19.5 / +15.4 across the three goal
#: cells) while agreeing in x to under 1.3 mm. ``land_err_mm`` is a POSITION fit
#: at the catch plane and so carries the centroid bias b_y(z_plane) absolutely;
#: ``arrival_dir_err_rad`` is a whole-arc VELOCITY and is bias-immune. So the gap
#: is a direct read-out of the standing E-1 caveat — only the bias GRADIENT was
#: measured, never the absolute offset — and its size (~18 mm) is the same order
#: as the measured b_y profile's own span (21 mm). Until the fixtured no-robot
#: capture closes that, a mocap-closed aim loop converges to the measurement's
#: cup, not the world's, and these two channels are how far apart those are.
SIGMA_E = np.array([14.7, 14.7, 0.00302, 0.00302, 0.0139], dtype=float)

#: The cross-check channel (NOT part of ``e``). ``release_speed_err_mms`` is the
#: same physical quantity as ``flight_time_err`` seen through the other branch of
#: the arc — in the model they are exactly proportional (``dT/dv = 2/g``), so
#: putting both in ``e`` would double-weight one measurement. It earns its keep
#: as an INDEPENDENT confirmation of the fitted correction (V2b), which is a
#: different job from being a fitted channel.
#:
#: Its noise floor is the miner's own admission gate, ``RELEASE_FIT_MAX_SE_MMS``
#: = 50 mm/s — independently corroborated at 51 mm/s by the correlation
#: decomposition in the module docstring.
SIGMA_RELEASE_SPEED_MMS = float(_miner.RELEASE_FIT_MAX_SE_MMS)


# ═════════════════════════════════════════════════════════════════════════════
# The goal
# ═════════════════════════════════════════════════════════════════════════════


class TossGoal(NamedTuple):
    """One toss goal — everything the forward model needs, in production frames.

    ``catch_pose_stow_mm`` is the ``Toss.action`` nominated catch pose (STOW mm,
    z = 170 is ACTIVE), ``flight_time_s`` the commanded flight time, and
    ``throw_site_xy_mm`` the platform's xy at release (Tier 8a: the catch xy, per
    ``_build_toss_cycle``'s ``aim_site``). ``plane_mm`` is the GLOBAL z the
    landing error is measured at; ``None`` means "the goal's own cup plane",
    which is what makes ``e_model(0) == 0`` exactly.
    """
    catch_pose_stow_mm: Tuple[float, float, float]
    flight_time_s: float
    throw_site_xy_mm: Optional[Tuple[float, float]] = None
    plane_mm: Optional[float] = None

    @property
    def site_xy(self) -> Tuple[float, float]:
        if self.throw_site_xy_mm is not None:
            return (float(self.throw_site_xy_mm[0]),
                    float(self.throw_site_xy_mm[1]))
        return (float(self.catch_pose_stow_mm[0]),
                float(self.catch_pose_stow_mm[1]))

    @property
    def apex_m(self) -> float:
        return float(apex_height_from_flight_time(self.flight_time_s))


#: The goal the 2026-08-12 corpus was thrown at, for probes and defaults:
#: 0.9032 s (apex exactly 1.000 m) at the ACTIVE plane. The xy is per-record.
CORPUS_FLIGHT_TIME_S = 0.9032314457914598


# ═════════════════════════════════════════════════════════════════════════════
# The forward model
# ═════════════════════════════════════════════════════════════════════════════


def release_state_for_command(u, goal: TossGoal):
    """``(nominal, commanded, launch_vel_mms, event_vel_mps)`` for command ``u``.

    A faithful replay of ``reload_coordinator_node._build_toss_cycle`` lines
    2449-2492, including its branch:

    * ``nominal`` is the UNCORRECTED ``compute_release_state`` — what ANNOUNCE
      carries (D4), and therefore what every mined error is differenced against;
    * with a zero aim the commanded state IS the nominal one (the byte-identical
      path the disabled aim map produces);
    * with a non-zero aim the target is displaced by ``aim_target_offset_mm`` and
      the state comes from ``compute_release_state_tilted`` at the same throw
      site the node uses.

    The branch is safe to finite-difference across because the two paths agree
    **bitwise** at zero tilt — ``compute_release_state_tilted``'s documented
    "degenerate identity", pinned by ``tests/motion/test_toss_release.py`` and
    re-pinned here — so a central difference straddling ``aim = 0`` is not
    straddling a discontinuity.

    The ``event_vel`` trim is applied last, as a multiply on the commanded
    magnitude (see the module docstring for why that multiply is the one
    non-production operation here). The returned ``launch_vel_mms`` keeps the
    production direction exactly: it is ``release_cmd.launch_vel_mms * (1 + dv)``.
    """
    arr = np.asarray(u, dtype=float).reshape(N_U)
    rx, ry, dv = float(arr[0]), float(arr[1]), float(arr[2])
    pose = tuple(float(v) for v in goal.catch_pose_stow_mm)
    T = float(goal.flight_time_s)

    nominal = compute_release_state(pose, T)
    if (rx, ry) == (0.0, 0.0):
        commanded = nominal
    else:
        offset = aim_target_offset_mm(rx, ry, T, pose[2])
        virtual_b = (pose[0] + float(offset[0]), pose[1] + float(offset[1]),
                     pose[2])
        commanded = compute_release_state_tilted(
            virtual_b, T, throw_site_xy_mm=goal.site_xy)

    k = 1.0 + dv
    event_vel = float(commanded.event_vel_mps) * k
    launch = np.asarray(commanded.launch_vel_mms, dtype=float) * k
    return nominal, commanded, launch, event_vel


def _plane_of(goal: TossGoal, nominal) -> float:
    if goal.plane_mm is not None:
        return float(goal.plane_mm)
    return float(np.asarray(nominal.catch_point_global_mm, dtype=float)[2])


def measurement_plane_mm(goal: TossGoal) -> float:
    """The GLOBAL z the landing error is measured at for this goal.

    The goal's own cup plane unless one was nominated. Public because the CLI
    and any future report want to print it beside the mined ``land_plane_mm``:
    the miner REFUSES a declaration more than
    ``toss_record_miner.PLANE_MISMATCH_TOL_MM`` = 5 mm from the plane it fitted
    at, and a fit whose model plane disagreed with the corpus's would produce a
    landing error that is a plane offset wearing a plant residual's clothes.
    """
    return _plane_of(goal, compute_release_state(goal.catch_pose_stow_mm,
                                                 goal.flight_time_s))


def e_model(u, goal: TossGoal) -> np.ndarray:
    """The five-channel task error the PRODUCTION chain predicts for command ``u``.

    ``e_model(0, goal)`` is exactly zero (to float round-off) for any goal whose
    measurement plane is its own cup plane — the planner solves the boundary-value
    problem it is being scored on, which is precisely why the residual worth
    learning is the *measured* one and the model is only ever a direction.

    The fourth channel of ``u`` — ``release_timing_offset`` — enters as a rigid
    shift of the release instant and NOWHERE ELSE: it is not passed to any
    production call here. It is written out rather than dropped so that
    :func:`sensitivity` computes its column instead of asserting it, but the
    resulting zero is **structural** — the arrival instant is defined as the
    release instant plus the flight time, so the shift cancels in the difference,
    and the four spatial channels never see δt at all. That the cup does not move
    under a dispatch shift is a *physical* claim about the platform holding its
    pose through the flight; this model assumes it rather than testing it. See
    the module docstring.

    Raises :class:`IlcFitError` when the production chain refuses the command
    (``ThrowTiltInfeasible``, an aim past the 12° ceiling, a ball that never
    reaches the plane) — the caller's trust-region loop shrinks and re-solves.
    """
    arr = np.asarray(u, dtype=float).reshape(N_U)
    dt = float(arr[3])
    try:
        nominal, _cmd, launch, _ev = release_state_for_command(arr, goal)
        plane = _plane_of(goal, nominal)
        pos, vel, t_flight = ballistics_bc.arrival_state_at_z(
            _cmd.release_pos_global_mm, launch, plane, descending=True)
    except (ThrowTiltInfeasible, ValueError) as exc:
        raise IlcFitError(
            'the production chain refuses u={} at goal {}: {}'
            .format(np.array2string(arr, precision=6), tuple(
                goal.catch_pose_stow_mm), exc))

    cup = np.asarray(nominal.catch_point_global_mm, dtype=float)
    # The RELEASE instant carries the dispatch shift; the ARRIVAL instant rides
    # it. Both appear in the measured flight time, which is the difference — so
    # the cancellation below is ALGEBRAIC, not a result routed through the
    # production seam (dt reaches no production call in this module). Written out
    # so the column is differenced like any other; read the zero as "the model
    # has no dt dependence", never as "the machine has none".
    t_release = dt
    t_arrival = t_release + float(t_flight)

    nominal_arrival = ballistics_bc.arrival_velocity(
        nominal.launch_vel_mms, float(goal.flight_time_s))
    lean_meas = _miner._lean_rad(vel)
    lean_nom = _miner._lean_rad(nominal_arrival)
    return np.array([
        float(pos[0]) - float(cup[0]),
        float(pos[1]) - float(cup[1]),
        lean_meas[0] - lean_nom[0],
        lean_meas[1] - lean_nom[1],
        (t_arrival - t_release) - float(goal.flight_time_s),
    ], dtype=float)


def release_speed_err_model(u, goal: TossGoal) -> float:
    """The CROSS-CHECK channel: modelled ``release_speed_err_mms`` for ``u``.

    ``|launch(u)| − |launch(0)|``, the same difference
    ``toss_record_miner.mine_arc`` takes between the backcast release velocity
    and ``cmd_launch_vel_mms``. Not part of ``e`` — see
    :data:`SIGMA_RELEASE_SPEED_MMS` for why one measurement must not be weighted
    twice — and used by V2b to confirm the fitted correction against a channel
    the fit never saw.
    """
    _nom, _cmd, launch, _ev = release_state_for_command(u, goal)
    nominal, _c2, base, _e2 = release_state_for_command(zero_command(), goal)
    del nominal, _c2, _e2
    return float(np.linalg.norm(launch) - np.linalg.norm(base))


# ═════════════════════════════════════════════════════════════════════════════
# Sensitivity, conditioning, and the v1 channel screen
# ═════════════════════════════════════════════════════════════════════════════


def sensitivity(u=None, goal: Optional[TossGoal] = None, *,
                steps=None) -> np.ndarray:
    """``F = de_model/du`` (5x4) by CENTRAL finite differences at ``u``.

    Central, not forward, and the reason is the whole of design constraint 1's
    argument turned around: the chain is a smooth closed form, so the leading
    error term of a central difference is ``O(h^2 f''')`` and vanishes exactly on
    the linear-through-zero aim block — the block that has an independent
    analytic answer (``b = 4h.theta``) and is therefore the one that must be
    right to machine precision, not to 1e-6.

    Step sizes are per-channel and justified at :data:`U_CHANNELS`; each is
    ~1/1000 of its channel's authority, which is the dimensionless form of "far
    inside the linear regime and far above the round-off floor".
    """
    if goal is None:
        raise IlcFitError('sensitivity() needs a goal')
    base = zero_command() if u is None else np.asarray(u, dtype=float).reshape(N_U)
    h = FD_STEPS if steps is None else np.asarray(steps, dtype=float).reshape(N_U)
    F = np.zeros((N_E, N_U), dtype=float)
    for j in range(N_U):
        up = base.copy()
        um = base.copy()
        up[j] += h[j]
        um[j] -= h[j]
        F[:, j] = (e_model(up, goal) - e_model(um, goal)) / (2.0 * h[j])
    return F


def weight_matrix(sigma_e=None, mask=None) -> np.ndarray:
    """``Q = diag(mask / sigma^2)`` — measurement-noise whitening.

    ``Q`` is a chi-square weighting, which is the honest default when the
    channels carry different units (mm, rad and s cannot be summed without one).
    The alternative — task-priority weighting, where a task-critical but noisy
    channel keeps its weight — is a REAL decision this module deliberately does
    not make, and Gate 1 recorded it as deferred. **It stopped being free on
    2026-08-13.** Under E-1 exactly one channel survived the mask, so the two
    weightings produced the identical step; with the mask lifted, the aim answer
    is a chi-square compromise between ``land_err`` (whitened weight
    ``|F|/sigma`` = 273) and ``arrival_dir`` (422), and the two do NOT agree —
    on the 2026-08-13 corpus ``land_err`` alone asks for ``aim_rx`` = +0.01249
    rad and ``arrival_dir`` alone for +0.00798. See :data:`SIGMA_E`'s note on
    what that gap is, and do not read the pooled answer as though the channels
    corroborated each other.
    """
    s = SIGMA_E if sigma_e is None else np.asarray(sigma_e, dtype=float)
    m = DEFAULT_MASK if mask is None else np.asarray(mask, dtype=float)
    return np.diag(m / (s ** 2))


def conditioning(F, *, sigma_e=None, tau=None, mask=None) -> Dict[str, Any]:
    """SVD of the COLUMN-SCALED, noise-whitened ``F`` — the v1 screen's evidence.

    The scaling is what makes the singular values mean something. Raw ``F`` mixes
    mm/rad (4.0e3) with s (0.91) and its singular values are an artefact of the
    unit system. Scaled::

        F_hat = diag(1/sigma_e) . F . diag(tau)

    each entry is *"measurement-noise sigmas of task error moved per full trust
    region of that command channel"*, so a singular value is directly readable as
    a signal-to-noise ratio at one iteration and the screen threshold is 1.0 by
    construction rather than by taste.

    Returns the raw and scaled matrices, the singular values, the condition
    number over the retained directions, and the per-channel scaled column norm
    (which, for a block-diagonal ``F`` like this one, IS the channel's singular
    value — reported separately because that coincidence is a property of this
    plant, not of the method).

    ``mask=None`` means :data:`DEFAULT_MASK`, the SAME default as
    :func:`weight_matrix`, :func:`solve_step`, :func:`required_command`,
    :func:`iterate` and :func:`fit_corpus`. One default across the whole module,
    because the alternative — this family defaulting to one mask and that family
    to another — is how a report and the step it justifies come to describe two
    different systems. It went full size when E-1 closed (2026-08-13) and became
    ``[0, 0, 1, 1, 1]`` on 2026-08-21 when owner decision 6 demoted ``land_err``
    to a monitor; both historical answers stay reachable by name, as an EXPLICIT
    ``mask=FULL_MASK`` or ``mask=E1_MASK``.
    """
    F = np.asarray(F, dtype=float).reshape(N_E, N_U)
    s = SIGMA_E if sigma_e is None else np.asarray(sigma_e, dtype=float)
    t = TAU0 if tau is None else np.asarray(tau, dtype=float)
    m = DEFAULT_MASK if mask is None else np.asarray(mask, dtype=float)
    F_hat = (np.diag(m / s) @ F) @ np.diag(t)
    U, sv, Vt = np.linalg.svd(F_hat)
    keep = sv > (sv[0] * 1e-12 if sv.size and sv[0] > 0 else 0.0)
    cond = (float(sv[keep].max() / sv[keep].min()) if keep.any()
            else float('inf'))
    return {
        'F': F,
        'F_scaled': F_hat,
        'singular_values': sv,
        'rank': int(keep.sum()),
        'condition_retained': cond,
        'column_norms': np.linalg.norm(F_hat, axis=0),
        'right_vectors': Vt,
        'left_vectors': U,
        'sigma_e': np.asarray(s, dtype=float),
        'tau': np.asarray(t, dtype=float),
        'mask': np.asarray(m, dtype=float),
    }


#: Scaled-column-norm floor for admission to v1. A channel whose full trust
#: region moves the task error by less than ONE measurement sigma cannot be
#: distinguished from noise in a single iteration, so learning on it is learning
#: on the estimator. 1.0 is not a tuned number — it is the definition of the
#: noise floor in the units :func:`conditioning` puts the matrix into.
SCREEN_SNR_MIN = 1.0

#: Cosine above which two scaled columns are called degenerate. At 0.99 the two
#: channels' error responses differ by under 8 degrees, i.e. the fit cannot tell
#: which of them moved the residual; the plan is explicit that such a pair is
#: EXCLUDED rather than regularised into noise (risk 4).
SCREEN_COS_MAX = 0.99


def screen_channels(F, *, sigma_e=None, tau=None, mask=None) -> Dict[str, Any]:
    """The **v1 channel screen**: which channels survive, and why not the others.

    Four exclusion rules, applied in this order (a channel is reported with the
    FIRST that fires, because a degenerate-and-silent channel is more usefully
    named silent):

    1. **blocked by the MASK** — the channel clears the floor on the UNMASKED
       matrix but not under ``mask``. That is a different fact from "this channel
       is weak", and keeping the two apart is what made closing E-1 a mask change
       rather than a re-derivation. Since 2026-08-13 the default mask is full
       size, so this rule fires only for a caller who passes :data:`E1_MASK`
       explicitly — which is exactly how the historical answer is reproduced.
    2. **below the noise floor** — scaled column norm < :data:`SCREEN_SNR_MIN`.
       A structurally-zero column (``release_timing_offset``) lands here with a
       norm of exactly 0.0 whether or not a mask is applied.
    3. **degenerate with another channel** — |cos| between scaled columns above
       :data:`SCREEN_COS_MAX`.
    4. otherwise **retained**.

    ``mask=None`` means :data:`DEFAULT_MASK`, the module-wide default (see
    :func:`conditioning`), which is ``[0, 0, 1, 1, 1]`` since owner decision 6
    demoted ``land_err`` to a monitor (2026-08-21). Rule 1 still needs the
    full-size matrix internally and computes it regardless of what was asked for,
    so both historical reports — ``mask=FULL_MASK`` and ``mask=E1_MASK`` — are
    unchanged.

    **The demotion does not screen the aim channels out, and that was checked
    before it landed**: the whitened aim-column norm falls 2.9207 → 2.4527 (and
    → 1.9330 once D2's corrected sigma is in), against a
    :data:`SCREEN_SNR_MIN` of 1.0.
    """
    m = DEFAULT_MASK if mask is None else np.asarray(mask, dtype=float)
    full = np.ones(N_E)
    rep = conditioning(F, sigma_e=sigma_e, tau=tau, mask=m)
    unmasked = (rep if np.array_equal(m, full)
                else conditioning(F, sigma_e=sigma_e, tau=tau, mask=full))
    cols = rep['F_scaled']
    norms = rep['column_norms']
    verdicts: List[Dict[str, Any]] = []
    retained: List[int] = []
    for j in range(N_U):
        if not np.isfinite(norms[j]) or norms[j] < SCREEN_SNR_MIN:
            blocked = (np.isfinite(unmasked['column_norms'][j])
                       and unmasked['column_norms'][j] >= SCREEN_SNR_MIN)
            verdicts.append({
                'channel': U_LABELS[j], 'index': j, 'snr': float(norms[j]),
                'snr_unmasked': float(unmasked['column_norms'][j]),
                'verdict': 'EXCLUDED',
                'reason': 'e1_blocked' if blocked else 'below_noise_floor',
                'detail': (
                    ('observable at {:.3g} sigma on the UNMASKED channels, but '
                     'every one of them is an E-1 blocked channel ({}) — this '
                     'is an entry-condition exclusion, not a weak channel'
                     .format(float(unmasked['column_norms'][j]),
                             ', '.join(E1_BLOCKED)))
                    if blocked else
                    ('full trust region {:.4g} {} moves the task error by '
                     '{:.3g} sigma (< {:.1f})'
                     .format(rep['tau'][j], U_CHANNELS[j].unit,
                             float(norms[j]), SCREEN_SNR_MIN)))})
            continue
        twin = None
        for i in retained:
            denom = norms[i] * norms[j]
            if denom <= 0.0:
                continue
            cos = abs(float(cols[:, i] @ cols[:, j]) / denom)
            if cos > SCREEN_COS_MAX:
                twin = (i, cos)
                break
        if twin is not None:
            verdicts.append({
                'channel': U_LABELS[j], 'index': j, 'snr': float(norms[j]),
                'verdict': 'EXCLUDED', 'reason': 'degenerate',
                'detail': ('|cos| = {:.4f} with {} (> {:.2f}) — the fit cannot '
                           'attribute a residual between them'
                           .format(twin[1], U_LABELS[twin[0]],
                                   SCREEN_COS_MAX))})
            continue
        retained.append(j)
        verdicts.append({'channel': U_LABELS[j], 'index': j,
                         'snr': float(norms[j]), 'verdict': 'RETAINED',
                         'reason': '', 'detail': ''})
    return {'retained': tuple(retained),
            'retained_labels': tuple(U_LABELS[j] for j in retained),
            'verdicts': verdicts, 'report': rep}


# ═════════════════════════════════════════════════════════════════════════════
# The update law + the EXACT-GATE re-validation loop (design constraint 3)
# ═════════════════════════════════════════════════════════════════════════════

#: Damping ``rho`` for ``R = rho.I`` in the SCALED coordinates
#: :func:`conditioning` defines. **PROVISIONAL.** Derivation, not taste: in
#: scaled units a direction with singular value ``s`` takes a step of
#: ``s/(s^2 + rho)`` per sigma of residual, so ``rho`` fixes where damping bites.
#: At ``rho = 0.25`` a direction that moves the error by a full sigma at full
#: trust region (``s = 1``) keeps 80 % of its step, while a half-noise-floor
#: direction (``s = 0.5``) keeps 50 % — damping is a soft continuation of the
#: screen's hard 1.0 threshold rather than an independent knob. v1's retained
#: channel runs ``s = 2.61``, where the cost is 3.5 %.
RHO_DAMPING = 0.25

#: How many times :func:`propose_step` halves the trust region before refusing.
#: 6 halvings is a 64x shrink; past that a "feasible" step is smaller than the
#: measurement noise it was computed from and the honest answer is a refusal
#: naming the gate that bound.
MAX_TRUST_SHRINKS = 6


def solve_step(F, e_meas, *, Q=None, rho: float = RHO_DAMPING,
               tau=None, mask=None, sigma_e=None) -> np.ndarray:
    """``argmin ||F.du + e||^2_Q + ||du||^2_R`` s.t. ``||du||_inf <= tau``.

    Closed form — normal equations, then clip — exactly as § Sensitivity and
    update law specifies: *"solved in closed form (normal equations + clip) — no
    QP-solver dependency at this dimensionality"*. The clip is a PROJECTION, not
    the constrained argmin; at four dimensions with a box that is the plan's
    deliberate choice and it is stated here so nobody later reads the clip as an
    optimiser bug. (If a constraint ever has to bind *inside* the solve, the plan
    says that is when to revisit — not before.)

    ``R`` is built in the SCALED coordinates so that ``rho`` is dimensionless:
    ``R = diag(rho / tau_j^2)``, i.e. ``rho`` units of penalty at one full trust
    region. Building it in raw units instead would make the same ``rho`` mean
    "0.25 per radian" on one channel and "0.25 per second" on another, which is
    how a damping constant silently becomes six.
    """
    F = np.asarray(F, dtype=float).reshape(N_E, N_U)
    e = np.asarray(e_meas, dtype=float).reshape(N_E)
    t = TAU0 if tau is None else np.asarray(tau, dtype=float).reshape(N_U)
    if Q is None:
        Q = weight_matrix(sigma_e=sigma_e, mask=mask)
    Q = np.asarray(Q, dtype=float).reshape(N_E, N_E)
    R = np.diag(float(rho) / np.maximum(t, 1e-300) ** 2)
    A = F.T @ Q @ F + R
    b = -(F.T @ Q @ e)
    try:
        du = np.linalg.solve(A, b)
    except np.linalg.LinAlgError as exc:                    # pragma: no cover
        raise IlcFitError(
            'the damped normal equations are singular ({}) — with R = rho/tau^2 '
            'and rho > 0 this cannot happen for finite F, so it means F carries '
            'a NaN'.format(exc))
    if not np.all(np.isfinite(du)):
        raise IlcFitError(
            'the update is non-finite (du={}) — F or e carries a NaN; refusing '
            'rather than clipping a NaN to the trust region'.format(du))
    return np.clip(du, -t, t)


def admit_command(u, goal: TossGoal) -> Tuple[bool, str]:
    """**The exact-gate re-validation** (design constraint 3): would the machine
    actually accept ``u`` at this goal?

    Runs the REAL production admission, in the order the shipped code runs it —
    never a linearised surrogate, and never a second copy of a bound:

    1. ``toss_sequencer`` CHECKING's static goal gates: the flight-time band
       (``FLIGHT_TIME_MIN/MAX_S``) and the workspace box
       (``TOSS_XY_LIMIT_MM`` / ``TOSS_Z_BAND_MM`` about ``TOSS_ACTIVE_Z_MM``).
       Invariant to a throw-side ``u``, and run anyway: a fit against an
       inadmissible goal is a fit nobody can fly.
    2. **per-channel authority** — ``|u_j| <= AUTHORITY[j]``, every bound
       imported from the module that owns it (``toss_cal.TOTAL_MAX_RAD``,
       ``toss_trim.SPEED_AUTHORITY``, ``toss_trim.TAU_AUTHORITY_MS``). This
       module does NOT widen a safety bound to make its own answer fit; when the
       measured requirement exceeds a bound, the refusal says so and the decision
       goes to the operator (see the Gate-1 memo).
    2b. **the speed channel's T-dependent band** —
       :func:`speed_authority_band`, derived from ``throw_envelope.evaluate`` at
       this goal's own nominal release speed and intersected with the
       :data:`ILC_SPEED_AUTHORITY` ceiling. This is the ILC speed authority
       (C2); the flat ceiling in step 2 is only its outer bound.
    3. ``toss_cal.clamp_total_aim`` — the D7 apply-time total-aim clamp. A clamp
       HIT is a refusal here, not a truncation: risk 5 in the plan is precisely
       that a partially truncated ``du`` desynchronises applied-u from
       recorded-u, and the cheap fix at four dimensions is to shrink the trust
       region until the step fits inside the authority instead of letting the
       clamp choose a different step than the one that was solved for.
    4. ``aim_target_offset_mm`` — raises past the 12° tilt ceiling.
    5. ``compute_release_state_tilted`` — ``ThrowTiltInfeasible``, the gate the
       FSM maps to ``REJECTED_TILT_CLAMP``.
    6. ``validate_event_vel`` — the bridge's ``[0.3, 7.0]`` m/s acceptance band,
       which the FSM checks as ``REJECTED_EVENT_VEL``.
    6b. ``throw_envelope.evaluate(T, event_vel)`` — contract **C-HAND-3**, the
       gate the FSM checks as ``REJECTED_THROW_ENVELOPE``. Added 2026-08-21 (C2):
       the wire band in step 6 "bounds nothing physical", and this is the one
       that does — seven bounds, END_STOP first.
    7. the forward model must be computable at ``u`` (a ball that never reaches
       the plane is not a toss).
    """
    arr = np.asarray(u, dtype=float).reshape(N_U)
    if not np.all(np.isfinite(arr)):
        return False, 'non-finite command {}'.format(arr)

    T = float(goal.flight_time_s)
    if not (FLIGHT_TIME_MIN_S <= T <= FLIGHT_TIME_MAX_S):
        return False, ('goal flight time {:.3f} s is outside the sequencer band '
                       '[{:.2f}, {:.2f}] (REJECTED_FLIGHT_TIME)'
                       .format(T, FLIGHT_TIME_MIN_S, FLIGHT_TIME_MAX_S))
    x, y, z = (float(v) for v in goal.catch_pose_stow_mm)
    if (abs(x) > TOSS_XY_LIMIT_MM or abs(y) > TOSS_XY_LIMIT_MM
            or abs(z - TOSS_ACTIVE_Z_MM) > TOSS_Z_BAND_MM):
        return False, ('goal pose ({:.1f}, {:.1f}, {:.1f}) is outside the '
                       'sequencer workspace box (REJECTED_WORKSPACE)'
                       .format(x, y, z))

    # Per-axis for the SCALAR channels only. The aim pair is bounded JOINTLY, on
    # its magnitude, by toss_cal.clamp_total_aim immediately below — that is the
    # D7 single enforcement point, and adding a per-axis copy of it here would be
    # a second bound to drift (a per-axis bound also admits |aim| = sqrt(2).MAX
    # on the diagonal, which is a different and wrong authority).
    for j in (2, 3):
        if abs(arr[j]) > AUTHORITY[j] * (1.0 + 1e-12):
            return False, ('{} = {:+.6g} {} exceeds its authority {:.6g} — '
                           'bound owned by {}. This module refuses rather than '
                           'widening it; widening is an operator decision.'
                           .format(U_LABELS[j], arr[j], U_CHANNELS[j].unit,
                                   AUTHORITY[j], U_CHANNELS[j].authority_src))

    # 2b. The SPEED channel's real authority: the T-dependent envelope band (C2).
    # AUTHORITY[2] above is now only the outer CEILING — it is flat in T, and the
    # admissible set is not. Checked here, before the aim clamp, because it is a
    # per-channel bound like the loop above and its refusal names a different
    # owner (throw_envelope / C-HAND-3, not the Gate-1 memo).
    try:
        v_nom = float(release_state_for_command(zero_command(), goal)[3])
    except (ThrowTiltInfeasible, ValueError) as exc:
        return False, ('the UNTRIMMED command at this goal is already refused '
                       'by the production release chain, so no band exists to '
                       'admit a trim into: {}'.format(exc))
    lo_dv, hi_dv = speed_authority_band(T, v_nom)
    if not (lo_dv - 1e-12 <= arr[2] <= hi_dv + 1e-12):
        beyond = throw_envelope.evaluate(T, v_nom * (1.0 + float(arr[2])))
        return False, (
            'event_vel_trim = {:+.6g} is outside the ADMISSIBLE band '
            '[{:+.4f}, {:+.4f}] at T = {:.4f} s — the band is derived per flight '
            'time from throw_envelope.evaluate (contract C-HAND-3), not from the '
            'flat {:+.2f} ILC_SPEED_AUTHORITY ceiling, because the admitted set '
            'is not flat in T. Bound just outside: {}'
            .format(arr[2], lo_dv, hi_dv, T, ILC_SPEED_AUTHORITY,
                    beyond.message or 'ILC_SPEED_AUTHORITY (the outer ceiling)'))

    rx, ry = float(arr[0]), float(arr[1])
    try:
        crx, cry, hits = toss_cal.clamp_total_aim(rx, ry)
    except toss_cal.TossCalError as exc:
        return False, 'toss_cal.clamp_total_aim refused the aim: {}'.format(exc)
    if hits:
        return False, ('the D7 apply-time clamp {} would TRUNCATE this aim '
                       '(|aim| = {:.6f} rad > {:.6f}) — a truncated step is not '
                       'the step that was solved for (risk 5); shrink the trust '
                       'region instead'
                       .format(hits, math.hypot(rx, ry), toss_cal.TOTAL_MAX_RAD))
    del crx, cry

    try:
        _nom, _cmd, _launch, event_vel = release_state_for_command(arr, goal)
    except ThrowTiltInfeasible as exc:
        return False, 'REJECTED_TILT_CLAMP — {}'.format(exc)
    except ValueError as exc:
        return False, 'the production release chain refused it: {}'.format(exc)

    if not validate_event_vel(event_vel):
        return False, ('event_vel {:.4f} m/s is outside the bridge band '
                       '[{:.2f}, {:.2f}] (REJECTED_EVENT_VEL)'
                       .format(event_vel, hw.TEENSY_TRAJ_MIN_EVENT_VEL_MPS,
                               hw.TEENSY_TRAJ_MAX_EVENT_VEL_MPS))
    # 6b. THE hardware gate — contract C-HAND-3's single enforcement point, run
    # on the COMMANDED speed exactly as `toss_sequencer` CHECKING runs it. The
    # band check above is derived from this function at zero aim; this call is
    # the same function on the actual command, so it also sees what the band
    # cannot: the 1/cos(aim) growth an aim adds, and a Tier-8b displaced goal's
    # faster release. Belt AND braces, deliberately — the band is the authority
    # the trust region is sized against, this is the verdict the machine gives.
    envelope = throw_envelope.evaluate(T, event_vel)
    if not envelope.ok:
        return False, 'REJECTED_THROW_ENVELOPE({})'.format(envelope.message)
    try:
        e_model(arr, goal)
    except IlcFitError as exc:
        return False, str(exc)
    return True, ''


def propose_step(F, e_meas, goal: TossGoal, *, u_current=None, tau=None,
                 rho: float = RHO_DAMPING, mask=None, sigma_e=None,
                 max_shrinks: int = MAX_TRUST_SHRINKS) -> Dict[str, Any]:
    """Solve, re-validate against the REAL gates, halve ``tau`` on refusal, repeat.

    Returns ``{'du', 'u_next', 'tau', 'shrinks', 'refusals', 'admitted'}``.
    Raises :class:`IlcFitError` when no admissible step exists after
    ``max_shrinks`` halvings — loudly, naming every gate that bound on the way
    down, because "the learner emitted nothing this iteration" is a result the
    operator must see rather than a silent zero.

    A zero step is admitted without shrinking (``u_current`` was already
    admissible, so is ``u_current + 0``) — that is convergence, not a refusal.
    """
    u0 = zero_command() if u_current is None else np.asarray(
        u_current, dtype=float).reshape(N_U)
    ok, why = admit_command(u0, goal)
    if not ok:
        raise IlcFitError(
            'the CURRENT command is already inadmissible, so no step from it '
            'can be validated: {}'.format(why))
    t = (TAU0 if tau is None else np.asarray(tau, dtype=float)).astype(float)
    refusals: List[str] = []
    for shrink in range(int(max_shrinks) + 1):
        du = solve_step(F, e_meas, tau=t, rho=rho, mask=mask, sigma_e=sigma_e)
        u_next = u0 + du
        ok, why = admit_command(u_next, goal)
        if ok:
            return {'du': du, 'u_next': u_next, 'tau': t, 'shrinks': shrink,
                    'refusals': refusals, 'admitted': True}
        refusals.append('tau={} refused: {}'.format(
            np.array2string(t, precision=6), why))
        t = t / 2.0
    raise IlcFitError(
        'NO ADMISSIBLE STEP: {} attempts ({} trust-region halvings, a {}x '
        'shrink) all refused by the real production gates. This is not a solver '
        'failure — it is the machine saying the correction the residual asks for '
        'cannot be commanded. Refusals, largest trust region first:\n  {}'
        .format(int(max_shrinks) + 1, int(max_shrinks), 2 ** int(max_shrinks),
                '\n  '.join(refusals)))


def required_command(F, e_meas, *, mask=None, sigma_e=None) -> np.ndarray:
    """The UNDAMPED, UNBOUNDED minimum-norm solution of ``F.du = −e``.

    The correction the residual actually asks for, with no trust region and no
    ridge — i.e. what the loop converges to if it is allowed to. Computed by
    pseudo-inverse on the whitened, masked system, so a rank-deficient ``F``
    returns the minimum-norm member of the solution set instead of raising.
    """
    F = np.asarray(F, dtype=float).reshape(N_E, N_U)
    e = np.asarray(e_meas, dtype=float).reshape(N_E)
    s = SIGMA_E if sigma_e is None else np.asarray(sigma_e, dtype=float)
    m = DEFAULT_MASK if mask is None else np.asarray(mask, dtype=float)
    W = np.diag(m / s)
    return np.linalg.pinv(W @ F) @ (-(W @ e))


def authority_report(F, e_meas, *, mask=None, sigma_e=None) -> Dict[str, Any]:
    """Can the correction the residual asks for be COMMANDED at all?

    Compares :func:`required_command` against :data:`AUTHORITY`, **in the shape
    each bound actually has**: the two scalar channels per-axis, and the aim pair
    JOINTLY on its magnitude, because ``toss_cal.clamp_total_aim`` bounds
    ``hypot(rx, ry)`` and nothing bounds an axis on its own. A per-axis report
    here would have said ``exceeds=False`` for a required aim of
    ``(0.9·MAX, 0.9·MAX)`` — magnitude 1.27·MAX — that :func:`admit_command`
    refuses outright, i.e. the report would have cleared a correction the machine
    will not fly. The aim rows therefore carry ``joint_required`` /
    ``joint_fraction`` beside their per-axis ``required`` / ``fraction``, and
    their ``exceeds`` is the joint verdict.

    This is the question the trust-region loop cannot answer — it asymptotes
    toward a bound rather than reporting it — and it is the one Gate 1 had to
    decide. On the present corpus the pooled vertical residual asks for
    ``event_vel_trim`` = −0.1076, which exceeded ``toss_trim.SPEED_AUTHORITY``
    (±0.10) by 7.6 %. **The operator decided it on 2026-08-13**, on the rail
    sweep this module supplied: an ILC-SPECIFIC :data:`ILC_SPEED_AUTHORITY` of
    ±0.15, leaving ``toss_trim``'s bound untouched. The required command is now
    72 % of the bound and this report says ``ok``.

    That is what a widening looks like when it is done properly, and the shape is
    the point: this module still refuses rather than widening, the widened bound
    is a NEW named constant owned by the loop that needed it, and the physics
    argument is attached (``STROKE_TOP_REV`` is algebraically
    velocity-independent, so a speed trim cannot walk the hand into its end stop;
    ``validate_event_vel`` still gates the result; and neither rail is reachable
    anywhere in the sequencer's flight-time band). A fit library does not decide
    this about itself.
    """
    need = required_command(F, e_meas, mask=mask, sigma_e=sigma_e)
    # The aim pair's bound is on its MAGNITUDE (toss_cal.clamp_total_aim), so its
    # exceedance is a joint verdict — the same test admit_command runs, not a
    # per-axis paraphrase of it that disagrees on the diagonal.
    aim_mag = math.hypot(float(need[0]), float(need[1]))
    rows = []
    for j in range(N_U):
        joint = aim_mag if j in (0, 1) else None
        magnitude = abs(float(need[j])) if joint is None else joint
        over = magnitude > AUTHORITY[j]
        row = {
            'channel': U_LABELS[j], 'required': float(need[j]),
            'authority': float(AUTHORITY[j]),
            'fraction': (float(abs(need[j]) / AUTHORITY[j])
                         if AUTHORITY[j] else float('inf')),
            'exceeds': bool(over),
            'joint_required': joint,
            'joint_fraction': (None if joint is None else
                               (float(joint / AUTHORITY[j]) if AUTHORITY[j]
                                else float('inf'))),
            'source': U_CHANNELS[j].authority_src}
        rows.append(row)
    return {'required': need, 'channels': rows, 'aim_magnitude': float(aim_mag),
            'any_exceeds': any(r['exceeds'] for r in rows),
            'iterations_at_tau': [
                (float('nan') if TAU0[j] <= 0
                 else float(math.ceil(abs(need[j]) / TAU0[j])))
                for j in range(N_U)]}


def cross_check_replay(du, goal: TossGoal,
                       records: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    """V2b: does ``du`` reduce a channel the fit NEVER SAW?

    The fit consumes ``flight_time_err_s``. This replays the proposed ``du``
    through :func:`release_speed_err_model` and scores it against the measured
    ``release_speed_err_mms`` — the *other* branch of the same arc, mined by a
    separate ballistic fit over a disjoint set of mocap samples. A correction
    fitted on one branch that cancels the other branch's error is evidence about
    the plant; a correction that only cancels the channel it was fitted on is
    evidence about arithmetic.

    Returns before/after RMS and the per-row residual, so a sign flip shows up as
    a DOUBLING rather than as a failed inequality.
    """
    vals = np.asarray([_num(r.get('release_speed_err_mms')) for r in records
                       if admit_record(r)[0]
                       and _num(r.get('release_speed_err_mms')) is not None],
                      dtype=float)
    if vals.size == 0:
        raise IlcFitError('no admitted row carries release_speed_err_mms')
    predicted = release_speed_err_model(du, goal)
    after = vals + predicted
    return {'n': int(vals.size), 'predicted_change_mms': float(predicted),
            'mean_before_mms': float(vals.mean()),
            'mean_after_mms': float(after.mean()),
            'rms_before_mms': float(np.sqrt((vals ** 2).mean())),
            'rms_after_mms': float(np.sqrt((after ** 2).mean())),
            'reduction': float(1.0 - np.sqrt((after ** 2).mean())
                               / np.sqrt((vals ** 2).mean()))}


def iterate(goal: TossGoal, e_meas, *, n_iter: int = 3, u0=None, tau=None,
            rho: float = RHO_DAMPING, mask=None, sigma_e=None
            ) -> List[Dict[str, Any]]:
    """Run ``n_iter`` MODEL-side iterations from a fixed measured residual.

    Each iteration re-linearises ``F`` at the current command (the plan's *"F ...
    at the current command"*) and predicts the residual as ``e + F.du``; the
    measured residual is NOT re-measured, because measuring it again means flying
    the robot. This is the offline convergence check and the trust-region sizing
    instrument, never a substitute for the Phase-3 hardware loop.

    Reports authority saturation explicitly: a run that ends against a bound has
    not converged, it has run out of authority, and those are different failures
    with different owners.
    """
    u = zero_command() if u0 is None else np.asarray(u0, dtype=float).reshape(N_U)
    e = np.asarray(e_meas, dtype=float).reshape(N_E)
    m = DEFAULT_MASK if mask is None else np.asarray(mask, dtype=float)
    out: List[Dict[str, Any]] = []
    for k in range(int(n_iter)):
        F = sensitivity(u, goal)
        step = propose_step(F, e, goal, u_current=u, tau=tau, rho=rho, mask=m,
                            sigma_e=sigma_e)
        u = step['u_next']
        e = e + F @ step['du']
        # Saturation is tested in the shape each bound has: the aim pair on its
        # MAGNITUDE (clamp_total_aim's joint bound), the scalars per-axis. A
        # per-axis aim test would call (0.9, 0.9)·MAX unsaturated while the D7
        # clamp is already refusing it.
        aim_mag = math.hypot(float(u[0]), float(u[1]))
        sat = [U_LABELS[j] for j in range(N_U)
               if (aim_mag if j in (0, 1) else abs(u[j]))
               >= AUTHORITY[j] * (1.0 - 1e-9)]
        out.append({'iter': k, 'u': u.copy(), 'du': step['du'],
                    'e_pred': e.copy(),
                    'norm_masked': float(np.linalg.norm(
                        m * e / (SIGMA_E if sigma_e is None
                                 else np.asarray(sigma_e, dtype=float)))),
                    'saturated': tuple(sat), 'shrinks': step['shrinks']})
    return out


# ═════════════════════════════════════════════════════════════════════════════
# The corpus
# ═════════════════════════════════════════════════════════════════════════════

#: Partition keys, ``toss_fit_lib.PARTITION_KEYS`` minus the two the ILC corpus
#: cannot supply (``toss_tier`` and ``hand_odrive_config_sha`` are declaration
#: fields and every mined-only row has them null) and plus ``cmd_flight_time_s``,
#: which the aim map does not need (it normalises by height) and this fit does:
#: a single-operating-point corpus cannot tell a launch-speed GAIN from an
#: OFFSET, so pooling two flight times would fit a compromise between two
#: hypotheses it has no power to separate.
PARTITION_KEYS: Tuple[str, ...] = (
    'tilt_map_version', 'bridge_fw_version', 'platform_fw_version',
    'cmd_flight_time_s', 'land_plane_mm',
)

#: Quantisation of ``cmd_flight_time_s`` inside the partition key and the
#: artifact key, seconds. **PROVISIONAL.** Derivation: the gain-vs-offset
#: ambiguity above means the two hypotheses' corrections diverge across a cell by
#: ``|dv|.(eps/T)``; setting that under the per-goal noise-equivalent command
#: (se 0.0032 s / 0.906 s = 0.0035) at the measured ``|dv|`` = 0.112 gives
#: ``eps/T <= 0.031``, i.e. ±28 ms at T = 0.903. 50 ms cells (±25 ms) sit inside
#: that with margin and give 11 cells over the sequencer's [0.55, 1.10] band.
FLIGHT_TIME_CELL_S = 0.050

#: Pose-cell pitch for the artifact key, mm. **PROVISIONAL**, and deliberately
#: the aim map's own grid: ``tilt_cal_grid.build_axis(DEFAULT_BOX_MM = 150,
#: nodes = 3)`` gives [-150, 0, +150], so a 150 mm pitch means an ILC cell and an
#: aim-map node name the same physical pose. That matters more than any
#: resolution argument: the plan's eventual absorb-or-keep decision compares the
#: two corrections cell by cell, and it cannot if their grids disagree.
POSE_CELL_MM = 150.0

#: Cell size for the nominated catch **z**, mm. NOT the xy pitch: quantising a
#: 170 mm ACTIVE plane onto a 150 mm grid would label it "150" and, worse, pool
#: it with a genuinely different plane. 10 mm is chosen against the miner's own
#: refusal tolerance — ``toss_record_miner.PLANE_MISMATCH_TOL_MM`` = 5.0 — so two
#: rows sharing a z cell agree on the measurement plane to inside the tolerance
#: the miner would have refused them over. The correction itself is z-insensitive
#: in the model (``F[flight_time, event_vel_trim]`` does not depend on z at all),
#: so the cell exists to prevent silent pooling, not to resolve a gradient.
POSE_Z_CELL_MM = 10.0


def _mine_stamp(path: str) -> str:
    """The MINE timestamp in a ``toss_record_miner`` output filename, or ``''``.

    ``toss_records_<bag>_<YYYYMMDD>_<HHMMSS>.jsonl`` — the bag name followed by
    ``datetime.now().strftime('%Y%m%d_%H%M%S')`` (``toss_record_miner``'s
    ``write_outputs``). Lexicographic order on that stamp IS chronological order,
    which is what makes "the newer mine wins" a comparison rather than a guess.
    An unparseable name returns ``''`` and therefore never supersedes a
    parseable one.
    """
    base = os.path.basename(path)
    if not (base.startswith('toss_records_') and base.endswith('.jsonl')):
        return ''
    parts = base[len('toss_records_'):-len('.jsonl')].split('_')
    if len(parts) < 4:
        return ''
    stamp = '_'.join(parts[-2:])
    return stamp if stamp.replace('_', '').isdigit() else ''


def load_corpus(paths: Iterable[str]) -> List[Dict[str, Any]]:
    """Read mined ``toss_record/1`` JSONL rows, DE-DUPLICATED by ``toss_uid``.

    Plain ``json.loads``, not ``toss_record.decode``: the ILC fields this module
    reads (``release_vel_track_mms``, ``arrival_dir_err_rad``, ...) are origin-'M'
    additive fields, and a decode that silently drops what this build does not
    know is the right behaviour for a map the NODE will load and the wrong one
    for a fit that must fail loudly on a corpus it cannot read. A row that will
    not parse is reported and skipped — losing a sitting because a writer was
    killed mid-line is worse than fitting the other 52.

    **The de-duplication is not housekeeping, it is a correctness gate.** The
    miner stamps every run, so a bag re-mined after a miner fix leaves BOTH files
    in ``temp/probes`` and the documented glob
    (``--corpus temp/probes/toss_records_*.jsonl``) matches all of them: on the
    2026-08-12 corpus that is 6 files, 106 rows, 53 distinct tosses. Pooling them
    would double every row and halve every standard error **without moving a
    single mean** — a corpus bug that looks exactly like better data, and the one
    failure mode a fit cannot detect from its own output.

    A duplicate is resolved by :func:`_mine_stamp`: **the newer mine supersedes
    the older**, whatever order the paths arrive in. First-seen-wins would be
    wrong rather than merely arbitrary here — the 2026-08-10 bag's earlier mine
    predates the miner fix that set ``usable_for_release_fit``, so first-seen on
    a sorted glob keeps 53 rows of which **0** are admissible, and the fit refuses
    a corpus it can in fact read.
    """
    out: List[Dict[str, Any]] = []
    index: Dict[str, Tuple[str, int]] = {}      # toss_uid -> (mine stamp, pos)
    # (dropped path, kept path) -> the toss_uids dropped. Reported per PAIR and
    # not per row: duplicates arrive a whole re-mine at a time (53 of them on the
    # 2026-08-12 corpus), and 53 lines naming the same two files would bury the
    # corpus summary they are supposed to qualify.
    dropped: Dict[Tuple[str, str], List[str]] = {}
    for path in paths:
        stamp = _mine_stamp(path)
        with open(path, 'r') as handle:
            for lineno, line in enumerate(handle, 1):
                line = line.strip()
                if not line:
                    continue
                try:
                    rec = json.loads(line)
                except ValueError as exc:
                    print('  SKIP {}:{}: {}'.format(path, lineno, exc))
                    continue
                if not isinstance(rec, dict):
                    print('  SKIP {}:{}: not an object'.format(path, lineno))
                    continue
                rec['_source_file'] = path
                uid = rec.get('toss_uid')
                if isinstance(uid, str) and uid:
                    prev = index.get(uid)
                    if prev is not None:
                        prev_stamp, pos = prev
                        if stamp > prev_stamp:
                            pair = (out[pos]['_source_file'], path)
                            out[pos] = rec
                            index[uid] = (stamp, pos)
                        else:
                            pair = (path, out[pos]['_source_file'])
                        dropped.setdefault(pair, []).append(uid)
                        continue
                    index[uid] = (stamp, len(out))
                out.append(rec)
    total = sum(len(v) for v in dropped.values())
    for (lost, kept), uids in sorted(dropped.items()):
        print('  SKIP {}: {} duplicate toss_uid(s) already read from the newer '
              'mine {} (first: {})'.format(lost, len(uids), kept, uids[0]))
    if total:
        print('  de-duplicated {} row(s) by toss_uid — a re-mined bag leaves '
              'BOTH files behind and the documented glob matches both'
              .format(total))
    return out


def _num(value: Any) -> Optional[float]:
    try:
        f = float(value)
    except (TypeError, ValueError):
        return None
    return f if math.isfinite(f) else None


def _pair(value: Any) -> Optional[Tuple[float, float]]:
    if not isinstance(value, (list, tuple)) or len(value) < 2:
        return None
    a, b = _num(value[0]), _num(value[1])
    return None if a is None or b is None else (a, b)


# ═════════════════════════════════════════════════════════════════════════════
# The guard port (build step 3) — G1–G11 applied to the ILC corpus
# ═════════════════════════════════════════════════════════════════════════════

#: Refusal reasons whose subject is THE TOSS. A row earning one of these is not
#: evidence about the plant on any channel, so the whole row is refused.
GUARD_ROW_REASONS = frozenset({
    'label_unknown', 'label_no_release',          # G9 — possession / label
    'label_unusable',                             # G9, speed guard's spelling
    'no_release_evidence',                        # G1
    'retry_cycle', 'reload_settle',               # G11 / G10
    'interlude_cycle',                            # G11+G10, speed guard's
    'no_gravity_correction', 'no_tilt_map',       # G5 — layer 0 identity
    'plant_dip_below_x3', 'plant_stroke_peak',    # G4 — plant health
    'plant_stroke_truncated',
})

#: Refusal reasons whose subject is the LANDING-PLANE POSITION FIT, i.e. exactly
#: and only the ``land_err_x/y`` monitor columns (G2). Scoped to the channel by
#: :func:`land_err_admissible`; the row keeps its arrival direction and its
#: flight time.
GUARD_LAND_ERR_REASONS = frozenset({
    'no_mocap_fit', 'mocap_fit_sparse', 'mocap_fit_rms_unknown',
    'mocap_fit_quality', 'missed_with_thin_track',
})

#: Refusal reasons that are preconditions of the SESSION TRIM'S OWN ESTIMATOR
#: and not of this one. Counted and reported, never applied.
#:
#: * ``applied_aim_unknown`` — ``toss_trim.reduce_to_aim`` computes
#:   ``b = A − J⁻¹·land_err`` and needs ``A = applied_aim_rad`` to subtract. The
#:   ILC update law never forms ``A``: it accumulates ``u`` in its own artifact
#:   and re-validates the sum through :func:`admit_command`. So the field is a
#:   precondition of a different estimator, and it is a DECLARATION ('D') field
#:   that no mined-only row can carry — on the 2026-08-12 corpus it alone would
#:   refuse 6 of 19.
#: * ``no_flight_pair`` — needs the DECLARED ``flight_time_s``. The miner
#:   produces the same measurand as ``cmd_flight_time_s`` (origin 'M', from the
#:   announcement's ``predicted_tof_sec``), and :func:`goal_of` already builds
#:   this module's entire geometry on it. Requiring the declaration would refuse
#:   16 of 19 rows for a number the fit is already using.
#: * ``no_geometry`` — ``toss_trim._z_of`` reads the goal z out of
#:   ``goal_catch_xyz_stow_mm``, another 'D' field that is null on every
#:   mined-only row. :func:`goal_of` recovers the same z from ``land_plane_mm``
#:   walked back down the two GENERATED offsets, which is this module's geometry
#:   precondition and is enforced where it belongs — a row whose goal cannot be
#:   recovered gets no goal cell and is not fitted at one. Applying the trim's
#:   version alone refused **16 of 19** rows (measured 2026-08-21) for a pose the
#:   fit had already reconstructed.
GUARD_DECLARATION_GAP_REASONS = frozenset({
    'applied_aim_unknown', 'no_flight_pair', 'no_geometry',
})

#: Refusal reasons that are SELF-BLINDING for this estimator — waived by name,
#: with the root cause, and counted.
#:
#: ``apex_out_of_band`` is G3: refuse a toss whose achieved apex missed the
#: commanded one by more than ``toss_trim.APEX_SANITY_FRAC`` (10 %). That guard
#: is correct for the trim, which fits AIM ONLY and cannot model the vertical
#: miss. **This module fits the vertical channel** — ``flight_time_err`` is
#: ``E_LABELS[4]`` and ``event_vel_trim`` is the column that corrects it — so a
#: toss whose apex missed is precisely the record the fit exists to consume.
#: ``toss_trim`` makes this exact argument itself, one estimator over:
#: :func:`~jugglebot.toss_trim.admit_for_speed` *"deliberately does NOT apply G3
#: … a toss whose apex missed by more than 10 % is precisely the record the speed
#: estimator exists to consume"*.
#:
#: It is not a theoretical concern. The corpus headline is a hand that throws
#: **+11 % fast**, and ``h = gT²/8`` turns +11 % of flight time into **+23 % of
#: apex** — so on any row carrying the declaration, G3 refuses the machine's own
#: dominant, known, correctable error. The guard would refuse the evidence needed
#: to clear the guard: a deadlock, not a conservative default.
GUARD_SELF_BLINDING_REASONS = frozenset({'apex_out_of_band'})

#: Every ``toss_trim`` refusal reason, mapped to the scope this module applies it
#: at. Pinned COMPLETE against ``toss_trim.AIM_REFUSAL_REASONS`` and
#: ``SPEED_REFUSAL_REASONS`` by ``test_every_toss_trim_refusal_reason_is_scoped``
#: — a guard added upstream must be scoped here or the suite goes red, rather
#: than defaulting to "admitted", which is how a guard that looks enforced is not.
GUARD_SCOPE: Dict[str, str] = {}
for _r in GUARD_ROW_REASONS:
    GUARD_SCOPE[_r] = 'ROW'
for _r in GUARD_LAND_ERR_REASONS:
    GUARD_SCOPE[_r] = 'LAND_ERR'
for _r in GUARD_DECLARATION_GAP_REASONS:
    GUARD_SCOPE[_r] = 'DECLARATION_GAP'
for _r in GUARD_SELF_BLINDING_REASONS:
    GUARD_SCOPE[_r] = 'SELF_BLINDING'
del _r


class GuardVerdict(NamedTuple):
    """What ``toss_trim``'s G1–G11 say about one row, sorted by scope.

    ``row_reasons`` refuses the record outright; ``land_err_reasons`` refuses
    only the two monitor columns; ``waived`` is reported, never applied.
    """
    row_reasons: Tuple[str, ...]
    land_err_reasons: Tuple[str, ...]
    waived: Tuple[str, ...]

    @property
    def row_ok(self) -> bool:
        return not self.row_reasons


def guard_verdict(rec: Dict[str, Any]) -> GuardVerdict:
    """Run ``toss_trim``'s aim AND speed guards over one row and scope the result.

    **This is design constraint 4's possession gate, and until 2026-08-21 it did
    not exist.** ``admit_record`` imported ``toss_trim`` for constants only and
    called none of its guards, so G1–G11 were not applied to the ILC corpus at
    all — including G9, the label/possession gate the plan makes an admission
    requirement. A synthetic corpus carried ``'label': 'CAUGHT'`` and nothing
    read it.

    The guards are CALLED, never re-implemented (one definition, or the online
    trim and the offline fit disagree about which rows are in the population).
    Both non-short-circuiting forms are used —
    :func:`~jugglebot.toss_trim.aim_refusals` and
    :func:`~jugglebot.toss_trim.speed_refusals` — because a classifier that only
    ever sees the FIRST reason would admit a row whose first refusal is
    channel-scoped and whose second refuses the whole toss.

    Every returned reason is looked up in :data:`GUARD_SCOPE`. An unrecognised
    reason is treated as :data:`GUARD_ROW_REASONS` — **fail closed**, so a guard
    added to ``toss_trim`` without a scope entry here refuses rows loudly instead
    of passing silently, and the completeness test says which one.
    """
    reasons: List[str] = []
    for reason in toss_trim.aim_refusals(rec):
        if reason not in reasons:
            reasons.append(reason)
    for reason in toss_trim.speed_refusals(rec):
        if reason not in reasons:
            reasons.append(reason)

    row: List[str] = []
    land: List[str] = []
    waived: List[str] = []
    for reason in reasons:
        scope = GUARD_SCOPE.get(reason)
        if scope is None and reason.startswith(toss_trim.LABEL_REFUSAL_PREFIX):
            # The open `label_<name>` family — a label that is neither unknown,
            # nor NO_RELEASE, nor in AIM_LABELS. G9, and it refuses the row.
            scope = 'ROW'
        if scope == 'LAND_ERR':
            land.append(reason)
        elif scope in ('DECLARATION_GAP', 'SELF_BLINDING'):
            waived.append(reason)
        else:
            row.append(reason)
    return GuardVerdict(tuple(row), tuple(land), tuple(waived))


def lateral_admissible(rec: Dict[str, Any]) -> Tuple[bool, str]:
    """May this row's LATERAL channels be read at all? ``(ok, reason)``.

    **THE E-1 admission requirement, and the single enforcement point for it.**
    Both :func:`admit_record` (``need_lateral``) and :func:`measured_error` go
    through here, so there is exactly one definition of "this row's lateral
    numbers are trustworthy" and a caller cannot get the channels by a different
    door.

    Two conditions, in this order:

    1. ``coverage_asym_s`` present, and within
       ``toss_record_miner.COVERAGE_ASYM_MAX_S``. **Absence is a refusal, not a
       default-pass**, and that is the whole point: the field only exists on rows
       mined after the whole-arc estimator landed (2026-08-13). A row mined
       before it carries PER-BRANCH lateral velocities, i.e. the E-1 artefact
       itself — 10.9 mrad of phantom arrival direction, ≈44 mm of phantom aim,
       and REPEATABLE, so it does not average away. The documented corpus glob
       matches both mines (:func:`load_corpus`'s de-duplication is by
       ``toss_uid``, not by miner version), so without this check lifting the
       mask would silently re-open E-1 on any corpus with a stale file in it.
    2. the channel itself present — ``arrival_dir_err_rad``.

    The threshold is IMPORTED from the miner rather than restated, for the same
    reason ``_lean_rad`` is: one definition of the gate, or the corpus and the
    fit disagree about which rows are in it.

    **``land_err_mm``'s presence left this gate on 2026-08-21** and moved to
    :func:`land_err_admissible`. Root cause: decision 6 made ``arrival_dir`` the
    PRIMARY aim channel and ``land_err`` a monitor, and requiring a monitor
    channel in order to read the primary one throws away the measurement the loop
    runs on to protect a column nobody fits. It is the same asymmetry
    :func:`admit_record` already argues for the vertical channel, one level down;
    it costs 2 of the 19 rows in the only corpus that exists.
    """
    asym = _num(rec.get('coverage_asym_s'))
    if asym is None:
        return False, ('no coverage_asym_s — mined before the E-1 whole-arc '
                       'estimator, so its lateral channels ARE the artefact')
    if abs(asym) > COVERAGE_ASYM_MAX_S:
        return False, ('coverage_asym_s {:+.4f} s exceeds {:.2f} s — a '
                       'half-seen arc, where the whole-arc fit\'s parity '
                       'cancellation no longer holds'
                       .format(asym, COVERAGE_ASYM_MAX_S))
    if _pair(rec.get('arrival_dir_err_rad')) is None:
        return False, 'no arrival_dir_err_rad'
    return True, ''


def land_err_admissible(rec: Dict[str, Any]) -> Tuple[bool, str]:
    """May this row's MONITOR channels (``land_err_x/y``) be read? ``(ok, why)``.

    Three conditions, and the third is the guard port (build step 3):

    1. :func:`lateral_admissible` — the E-1 whole-arc gate, because a plane
       residual mined before it is as artefactual as an arrival direction;
    2. ``land_err_mm`` present;
    3. **``toss_trim``'s G2 track-quality guard**, via :func:`guard_verdict` —
       ``no_mocap_fit`` / ``mocap_fit_sparse`` / ``mocap_fit_rms_unknown`` /
       ``mocap_fit_quality`` / ``missed_with_thin_track``. Those five reasons all
       speak about the LANDING-PLANE POSITION FIT, which is exactly and only what
       ``land_err_mm`` is, so they gate this channel and not the row: a toss with
       a clean arc and a poor plane fit is a perfectly good arrival-direction
       measurement, and refusing it would discard the primary channel to protect
       a monitor.

    On the 2026-08-12 corpus this refuses 10 of the 19 admitted rows'
    ``land_err`` columns (8 ``mocap_fit_quality`` + 2 ``no_mocap_fit``) while
    keeping every one of their arrival directions.
    """
    ok, why = lateral_admissible(rec)
    if not ok:
        return False, why
    if _pair(rec.get('land_err_mm')) is None:
        return False, 'no land_err_mm'
    verdict = guard_verdict(rec)
    if verdict.land_err_reasons:
        return False, ('toss_trim G2: {}'
                       .format(', '.join(verdict.land_err_reasons)))
    return True, ''


def admit_record(rec: Dict[str, Any], *, need_lateral: bool = False,
                 uptime_max_ms: Optional[float] = None) -> Tuple[bool, str]:
    """Is this row admissible to the ILC fit? ``(ok, reason)``.

    The gate is the MINER's own ``usable_for_release_fit`` — ``backcast_fit_n``
    and ``arrival_fit_n`` >= 20 with ``release_vel_se_mms`` <= 50 — imported as a
    flag rather than re-derived, for the reason ``toss_fit_lib`` gives about
    ``admit_for_aim``: two quality bars for one population is how a headline
    becomes unreproducible. On top of it:

    * the vertical channels must be present (``flight_time_err_s``);
    * ``need_lateral`` additionally requires :func:`lateral_admissible`.
      **It is NOT the default**, and that asymmetry is deliberate: a row with a
      clean flight time and no landing fit is a perfectly good vertical-channel
      measurement, and :func:`pooled_error` averages per CHANNEL, so refusing the
      whole row would throw away vertical data to protect a lateral channel that
      :func:`measured_error` has already nulled. Pass ``need_lateral=True`` when
      a caller needs a row that can answer on every channel;
    * ``uptime_max_ms`` is the plan's G-1 defence-in-depth refusal
      (*"the learner refuses records whose uptime_ms exceeds the healthy
      threshold"*) and it defaults to **None = no refusal**. Not an oversight:
      the healthy threshold is G-1's to fix, and 16 of the 19 usable rows in the
      only corpus that exists sit at 16.7 h. :func:`uptime_census` reports the
      distribution so the choice is made with the cost visible.
    * **``toss_trim``'s G1–G11, via :func:`guard_verdict`** (2026-08-21, build
      step 3). This is the possession gate design constraint 4 asks for and that
      this function did not have. Only the ROW-scoped reasons refuse here; the
      G2 landing-fit reasons scope to the ``land_err`` monitor columns
      (:func:`land_err_admissible`) and the trim-estimator preconditions are
      waived and counted. See :data:`GUARD_SCOPE` for the whole table and the
      root cause of every waiver.
    * ``throw_site_xy_mm`` under **tier 8b** (C7). Under 8a the field is the
      ``TossSequencer`` class DEFAULT ``(0.0, 0.0)`` and means "unset", not
      "threw from the origin" — reading it as a site would move the model 192 mm
      on the corpus's own displaced cells. So it is consumed only when the row
      declares 8b, an 8b row that carries no site is refused (under 8b every
      field of the release state is a function of A, and there is no correct 8a
      fallback), and an 8b row whose site is DISPLACED from its cup is refused
      because the v1 artifact key ``(x, y, z, T)`` has no site component and
      would apply that correction to every future toss to the same cup.
    """
    if not rec.get('usable_for_release_fit'):
        return False, 'not usable_for_release_fit (the miner\'s own gate)'
    if _num(rec.get('flight_time_err_s')) is None:
        return False, 'no flight_time_err_s'
    verdict = guard_verdict(rec)
    if verdict.row_reasons:
        return False, ('toss_trim guard: {} (G1-G11, see GUARD_SCOPE)'
                       .format(', '.join(verdict.row_reasons)))
    ok, why = throw_site_admissible(rec)
    if not ok:
        return False, why
    if need_lateral:
        ok, why = lateral_admissible(rec)
        if not ok:
            return False, why
    if uptime_max_ms is not None:
        up = _num(rec.get('uptime_ms_at_release'))
        if up is None:
            return False, 'no uptime_ms_at_release (G-1 refusal is armed)'
        if up > float(uptime_max_ms):
            return False, ('uptime {:.2f} h exceeds the healthy bound {:.2f} h '
                           '(G-1)'.format(up / 3.6e6,
                                          float(uptime_max_ms) / 3.6e6))
    return True, ''


def measured_error(rec: Dict[str, Any]) -> np.ndarray:
    """The measured ``e`` (5,) for one row — ``nan`` in a channel the row lacks.

    Straight reads of the mined fields, in :data:`E_LABELS` order. No arithmetic:
    the miner already differenced each channel against the production nominal
    (``mine_arc``), so recomputing anything here would be a second definition of
    the measurand.

    **One gate, and it is not arithmetic either**: a row that fails
    :func:`lateral_admissible` comes back ``nan`` in the four lateral channels
    and keeps its vertical one. This is where the E-1 refusal has to live — the
    channels are pooled with a per-channel ``nanmean`` (:func:`pooled_error`), so
    a stale row that merely failed ``admit_record(need_lateral=True)`` would
    still contribute its contaminated lateral numbers through every path that
    admits on the vertical channel alone. Refusing at the read is the only place
    that closes all of them at once.

    **Two gates since 2026-08-21, not one**, because the two lateral channels are
    no longer the same kind of thing (decision 6): ``arrival_dir`` is the primary
    aim measurand and gates on :func:`lateral_admissible` (E-1), while
    ``land_err`` is a monitor and additionally gates on
    :func:`land_err_admissible` (E-1 + presence + ``toss_trim``'s G2). A row with
    a clean arc and a poor plane fit therefore comes back with its arrival
    direction intact and ``nan`` in the monitor columns, instead of losing both.
    """
    out = np.full(N_E, np.nan, dtype=float)
    if lateral_admissible(rec)[0]:
        out[2], out[3] = _pair(rec.get('arrival_dir_err_rad'))
    if land_err_admissible(rec)[0]:
        out[0], out[1] = _pair(rec.get('land_err_mm'))
    ft = _num(rec.get('flight_time_err_s'))
    if ft is not None:
        out[4] = ft
    return out


def channel_disagreement(rec: Dict[str, Any],
                         goal: Optional[TossGoal] = None
                         ) -> Optional[Dict[str, Any]]:
    """THE per-toss C3 validation log, or ``None`` when the row cannot answer.

    ``(land_err − arrival_dir · gain)`` per axis, in mm at the measurement plane,
    where ``gain`` is the model's OWN ratio ``∂land_err/∂aim ÷ ∂arrival_dir/∂aim``
    read off :func:`sensitivity` — never a re-derived ``4h + Δz``, because the
    whole point of the number is that it compares two channels *through the model
    that relates them*, and a second copy of that relation would make a model
    disagreement look like a plant one.

    **Why it is logged on every toss.** Decision 6 resolves C3 by DECISION rather
    than by the H2 measurement (which the owner retired: conventional markers on
    the ball corrupt the trackable surface being measured). This is the standing
    replacement validation, and it has two readings:

    * the arrival_dir-driven loop converges **while** this residual holds the
      known ``b(z)`` profile shape ⇒ the centroid-bias model is confirmed and the
      monitor channel is behaving exactly as the model says it must;
    * catch rate plateaus **with** a converged aim ⇒ the decision is wrong, the
      loop has centred on the measurement's cup rather than the world's, and C3
      re-opens.

    Baseline, measured on the 2026-08-12 corpus (2026-08-21, over the three
    newest-mine files, model gain 3993.264 mm/rad at the corpus goal). Two
    populations, and both are quoted because they answer different questions:

    * **E-1-admissible only, n = 17** — the population the C3 finding was taken
      on: pooled ``(+0.90, +18.10) mm``, per cell y = ``+17.45 / +19.57 /
      +15.46`` at n = 6 / 8 / 3, x under 1.35 mm everywhere.
    * **G2-gated, n = 9** — what :func:`disagreement_census` reports now that
      :func:`land_err_admissible` gates the monitor columns on the landing-plane
      fit's own quality: ``(−2.90, +19.77) mm``, sd ``(3.87, 10.24)``. The y
      disagreement SURVIVES the quality gate and tightens, which is the reading
      that matters: it is not an artefact of poor plane fits.
    """
    if goal is None:
        goal = goal_of(rec)
    if goal is None:
        return None
    e = measured_error(rec)
    if np.isnan(e[:4]).any():
        return None
    F = sensitivity(zero_command(), goal)
    # aim_rx drives land_err_y and arrival_dir_y; aim_ry drives land_err_x and
    # arrival_dir_x. The per-axis ratio is the model's mm-per-rad-of-lean.
    with np.errstate(divide='ignore', invalid='ignore'):
        gain_x = float(F[0, 1] / F[2, 1]) if F[2, 1] else float('nan')
        gain_y = float(F[1, 0] / F[3, 0]) if F[3, 0] else float('nan')
    return {'toss_uid': rec.get('toss_uid'),
            'goal_cell': goal_key(rec),
            'land_err_mm': (float(e[0]), float(e[1])),
            'arrival_dir_rad': (float(e[2]), float(e[3])),
            'gain_mm_per_rad': (gain_x, gain_y),
            'arrival_dir_as_plane_mm': (float(e[2]) * gain_x,
                                        float(e[3]) * gain_y),
            'disagreement_mm': (float(e[0]) - float(e[2]) * gain_x,
                                float(e[1]) - float(e[3]) * gain_y)}


def disagreement_census(records: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    """Pooled :func:`channel_disagreement` over rows that can answer it."""
    rows = [d for d in (channel_disagreement(r) for r in records)
            if d is not None]
    if not rows:
        return {'n': 0, 'mean_mm': (float('nan'), float('nan')),
                'sd_mm': (float('nan'), float('nan')), 'rows': []}
    arr = np.array([d['disagreement_mm'] for d in rows], dtype=float)
    sd = arr.std(axis=0, ddof=1) if arr.shape[0] > 1 else np.zeros(2)
    return {'n': int(arr.shape[0]),
            'mean_mm': (float(arr[:, 0].mean()), float(arr[:, 1].mean())),
            'sd_mm': (float(sd[0]), float(sd[1])),
            'rows': rows}


#: The tier string that makes ``throw_site_xy_mm`` mean something (C7). Imported
#: from the sequencer rather than restated — the record's ``toss_tier`` field is
#: stamped from the same constant.
TIER_8B = str(toss_sequencer.TIER_8B)

#: How far an 8b throw site may sit from its own cup xy and still be written into
#: a v1 artifact cell, mm — the plan's **zero-displacement admission gate**
#: (``plans/active/critical-point-ilc.md``, contradiction ledger C7).
#:
#: **This is not a physical tolerance — it is the width of "the key can still
#: name this geometry".** The v1 key is ``(x, y, z, T)`` on the CATCH pose
#: (``toss_ilc.goal_key``) with no site component, so two goals that share a cup
#: and differ in throw site share a cell and their corrections pool. Under 8a,
#: and under 8b thrown from the cup, the cup determines the geometry exactly.
#:
#: It is :data:`POSE_CELL_MM`, the key's own xy quantisation, and that choice is
#: the plan's — deliberately NOT the 1 mm "the site IS the cup" reading my first
#: pass used. Root cause, and the plan states it in one line: **8b is the SHIPPED
#: default** (``config/hardware_config.yaml: toss_tier: "8b"``), so a gate at
#: float slack refuses every row a real 8b session produces — the machine throws
#: from wherever it is and aims at displaced cups, and refusing that is refusing
#: the tier. The symmetry that makes 150 mm defensible rather than merely
#: convenient: the key ALREADY tolerates 150 mm of variation in the cup xy
#: itself, since a cup anywhere inside a cell shares that cell. Tolerating the
#: same of site variation is the existing tolerance applied to the other end of
#: the same geometry, not a new one.
#:
#: What it does NOT establish, and this is the open item rather than a silence:
#: two points 150 mm apart can still fall in DIFFERENT cells (74 mm and 224 mm
#: quantise to 0 and 150), so "inside the gate" does not prove "the aim site and
#: the cup name one cell". The plan's full C7 resolution pairs this gate with
#: keying the cell on the AIM SITE — catch xy under 8a, throw site under 8b —
#: which needs ``toss_ilc.goal_key`` and the node's lookup to move together and
#: is a separate, node-touching change.
THROW_SITE_KEY_TOL_MM = POSE_CELL_MM


def throw_site_xy_of(rec: Dict[str, Any]) -> Optional[Tuple[float, float]]:
    """The row's MEANINGFUL 8b throw site, or ``None``. **Tier-gated** (C7).

    ``throw_site_xy_mm`` is a DECLARATION field filled from
    ``getattr(seq, 'throw_site_xy_mm', (0.0, 0.0))``, and ``TossSequencer``'s own
    class default for it is ``(0.0, 0.0)``. Under tier 8a nothing ever assigns
    it, so ``[0.0, 0.0]`` on an 8a row means **UNSET**, not "threw from the
    origin" — and the 2026-08-12 corpus contains exactly that: three admitted
    rows declaring ``toss_tier = '8a'``, ``throw_site_xy_mm = [0.0, 0.0]`` and a
    cup at ``(±150, −120)``. Reading the field as a site there moves the modelled
    release **192.094 mm**, which is why C7 says the fit ignores it and why the
    fix is a tier gate rather than a plain read.
    """
    if str(rec.get('toss_tier') or '') != TIER_8B:
        return None
    return _pair(rec.get('throw_site_xy_mm'))


def throw_site_admissible(rec: Dict[str, Any]) -> Tuple[bool, str]:
    """C7 — may this row be fitted and written under the v1 artifact key?

    Three refusals, all only reachable under tier 8b:

    * **``throw_site_unknown``** — the row declares 8b and carries no usable
      site. Under 8b every field of the release state is a function of A
      (``reload_coordinator_node``'s own ``elif tier == TIER_8B`` branch returns
      ``release = None`` for exactly this reason), so there is no 8a fallback
      that is merely approximate: the model would be evaluated at a pose the
      machine never occupied.
    * **``throw_site_not_in_key``** — the row declares 8b and its site is more
      than :data:`THROW_SITE_KEY_TOL_MM` from its own cup xy. The geometry is
      recoverable and :func:`goal_of` models it correctly; what cannot hold it is
      the ARTIFACT KEY, which names the cup and not the site. Writing such a
      row's correction into that cell would apply a far-displaced-throw aim to
      every future toss to the same cup from anywhere. The bound is the key's own
      xy quantisation, NOT float slack — 8b is the shipped default and a gate at
      float slack refuses the tier; see :data:`THROW_SITE_KEY_TOL_MM`.

    Zero rows in the 2026-08-12 corpus are affected: all 19 admitted rows are
    tier 8a or tier-unknown.
    """
    if str(rec.get('toss_tier') or '') != TIER_8B:
        return True, ''
    site = _pair(rec.get('throw_site_xy_mm'))
    if site is None:
        return False, ('throw_site_unknown — the row declares tier 8b and every '
                       'field of an 8b release state is a function of the throw '
                       'site A; there is no 8a fallback that is merely '
                       'approximate')
    cup = _cup_xy_of(rec)
    if cup is None:
        # The SITE is known here; the CUP is not, so the displacement the gate
        # bounds cannot be formed — and without a cup :func:`goal_of` returns
        # None, so the row has no artifact cell to be written to in the first
        # place. Named apart from `throw_site_unknown` so the two are not
        # diagnosed as one fault.
        return False, ('throw_site_unplaceable — tier 8b with a site but no '
                       'recoverable cup xy, so the displacement the v1 key '
                       'tolerance bounds cannot be computed')
    offset = math.hypot(site[0] - cup[0], site[1] - cup[1])
    if offset > THROW_SITE_KEY_TOL_MM:
        return False, ('throw_site_not_in_key — an 8b throw site {:.1f} mm from '
                       'its cup, and the v1 artifact key (x, y, z, T) names the '
                       'cup only, so this correction would be applied to every '
                       'toss to the same cup regardless of throw site'
                       .format(offset))
    return True, ''


def _cup_xy_of(rec: Dict[str, Any]) -> Optional[Tuple[float, float]]:
    """``land_xy_global_mm − land_err_mm`` — the announced cup xy, or ``None``."""
    land = _pair(rec.get('land_xy_global_mm'))
    err = _pair(rec.get('land_err_mm'))
    if land is None or err is None:
        return None
    return (land[0] - err[0], land[1] - err[1])


def goal_of(rec: Dict[str, Any]) -> Optional[TossGoal]:
    """The :class:`TossGoal` a mined row was thrown at, or ``None``.

    ``goal_catch_xyz_stow_mm`` is a DECLARATION field and every mined-only row
    has it null, so the pose is recovered from the two mined fields that bracket
    it: ``land_xy_global_mm − land_err_mm`` is the announced cup xy by the
    miner's own definition of ``land_err_mm``, and the cup xy IS the goal xy
    (``stow_to_global_mm`` adds only z). The z comes from ``land_plane_mm``
    walked back down the two GENERATED offsets — never a third copy of the
    geometry.

    **The throw site is carried since 2026-08-21 (C7).** It used to be dropped,
    on the stated assumption that the toss was tier 8a; under 8b that assumption
    silently evaluates the forward model at the cup instead of at A. It is read
    through :func:`throw_site_xy_of`, which is tier-gated — see there for why a
    plain read of ``throw_site_xy_mm`` is a 192 mm trap on this very corpus.
    ``TossGoal.site_xy`` still falls back to the cup xy when it is ``None``,
    which is correct for 8a and is now a documented tier fact rather than an
    assumption about a corpus.
    """
    T = _num(rec.get('cmd_flight_time_s'))
    cup = _cup_xy_of(rec)
    plane = _num(rec.get('land_plane_mm'))
    if T is None or cup is None or plane is None:
        return None
    z = plane - float(hw.GEOM_INITIAL_HEIGHT_MM) - float(hw.HAND_CATCH_OFFSET_MM)
    return TossGoal(catch_pose_stow_mm=(cup[0], cup[1], z),
                    flight_time_s=T, plane_mm=plane,
                    throw_site_xy_mm=throw_site_xy_of(rec))


def goal_key(rec: Dict[str, Any], *, pose_cell_mm: float = POSE_CELL_MM,
             z_cell_mm: float = POSE_Z_CELL_MM,
             flight_cell_s: float = FLIGHT_TIME_CELL_S):
    """The quantised artifact key for one row, or ``None`` when unrecoverable.

    ``(x_cell_mm, y_cell_mm, z_cell_mm, flight_time_cell_s)``. The xy cells are
    centred on the aim map's own node values (multiples of :data:`POSE_CELL_MM`
    about the origin) so the ILC key and an aim-map node name the same pose; z
    and the flight time get their own, finer cells — see :data:`POSE_Z_CELL_MM`
    and :data:`FLIGHT_TIME_CELL_S`.
    """
    goal = goal_of(rec)
    if goal is None:
        return None
    x, y, z = goal.catch_pose_stow_mm
    q = float(pose_cell_mm)
    qz = float(z_cell_mm)
    return (round(float(np.round(x / q)) * q, 3),
            round(float(np.round(y / q)) * q, 3),
            round(float(np.round(z / qz)) * qz, 3),
            round(float(np.round(goal.flight_time_s / float(flight_cell_s)))
                  * float(flight_cell_s), 4))


def partition_key(rec: Dict[str, Any]) -> Tuple[Any, ...]:
    """The plant-identity tuple. A ``None`` component stays ``None`` — an unknown
    key is a KNOWN GAP, never an assumed match (``toss_fit_lib``'s § 8 rule)."""
    values: List[Any] = []
    for key in PARTITION_KEYS:
        value = rec.get(key)
        if key == 'cmd_flight_time_s':
            num = _num(value)
            values.append(None if num is None
                          else round(float(np.round(num / FLIGHT_TIME_CELL_S))
                                     * FLIGHT_TIME_CELL_S, 4))
        elif key == 'land_plane_mm':
            num = _num(value)
            values.append(None if num is None else round(num, 2))
        else:
            values.append(None if value is None else str(value))
    return tuple(values)


def partition_census(records: Sequence[Dict[str, Any]]
                     ) -> Dict[Tuple[Any, ...], int]:
    census: Dict[Tuple[Any, ...], int] = {}
    for rec in records:
        key = partition_key(rec)
        census[key] = census.get(key, 0) + 1
    return census


def census_lines(census: Dict[Tuple[Any, ...], int]) -> List[str]:
    lines = []
    for key, n in sorted(census.items(), key=lambda kv: (-kv[1], str(kv[0]))):
        parts = ['{}={}'.format(name, '?' if value is None else value)
                 for name, value in zip(PARTITION_KEYS, key)]
        lines.append('  n={:<4d} {}'.format(n, '  '.join(parts)))
    return lines


def select_partition(records: Sequence[Dict[str, Any]], *,
                     allow_cross: bool = False,
                     key: Optional[Tuple[Any, ...]] = None):
    """Pick the partition to fit; REFUSE a cross-partition fit without a flag.

    ``toss_fit_lib.select_partition``'s rule, and it bites harder here: this
    corpus's three bags differ in can-bridge uptime by two and a half orders of
    magnitude (0.05 h, 1.3 h, 16.7 h) while agreeing on every partition key, so
    the key list is necessary and NOT sufficient — read :func:`uptime_census`
    beside the partition census, always.
    """
    census = partition_census(records)
    warnings: List[str] = []
    if not records:
        return [], census, None, warnings
    missing = sorted({name for k in census
                      for name, v in zip(PARTITION_KEYS, k) if v is None})
    if missing:
        warnings.append(
            'partition keys UNKNOWN in this corpus: {} — a null key is a known '
            'gap, never an assumed match.'.format(', '.join(missing)))
    if key is not None:
        rows = [r for r in records if partition_key(r) == key]
        if not rows:
            raise IlcFitError('no records in the requested partition {}'
                              .format(key))
        return rows, census, key, warnings
    if len(census) == 1:
        return list(records), census, next(iter(census)), warnings
    if not allow_cross:
        raise IlcFitError(
            'refusing to fit across {} partitions — a correction pooled over '
            'two plants is the weighted average of two machines and it LOOKS '
            'fine. Census:\n{}\nFit one partition, or cross deliberately with '
            'allow_cross=True.'.format(len(census), '\n'.join(
                census_lines(census))))
    warnings.append('CROSSING {} partitions by explicit request.'
                    .format(len(census)))
    return list(records), census, None, warnings


def uptime_census(records: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    """Can-bridge uptime distribution over a record set — the G-1 evidence.

    Risk 1 in the plan is *"learning a moving plant"*: the transport latency
    drifts 10 → ~240 ms with bridge uptime, so a corpus dominated by one long-
    uptime sitting is a corpus of one plant state. This function does not
    refuse anything; it makes the cost of any threshold visible before it is
    chosen.
    """
    hours = [(_num(r.get('uptime_ms_at_release')) or float('nan')) / 3.6e6
             for r in records]
    finite = [h for h in hours if math.isfinite(h)]
    buckets = {'<=2h': 0, '2-6h': 0, '6-12h': 0, '>12h': 0, 'unknown': 0}
    for h in hours:
        if not math.isfinite(h):
            buckets['unknown'] += 1
        elif h <= 2.0:
            buckets['<=2h'] += 1
        elif h <= 6.0:
            buckets['2-6h'] += 1
        elif h <= 12.0:
            buckets['6-12h'] += 1
        else:
            buckets['>12h'] += 1
    return {'n': len(records), 'buckets': buckets,
            'min_h': min(finite) if finite else None,
            'max_h': max(finite) if finite else None,
            'median_h': float(np.median(finite)) if finite else None}


def group_by_goal(records: Sequence[Dict[str, Any]], **kw
                  ) -> Dict[Any, List[Dict[str, Any]]]:
    """``{goal_key: rows}``. Rows whose goal cannot be recovered group under
    ``None`` and are reported, never silently merged into a real cell."""
    groups: Dict[Any, List[Dict[str, Any]]] = {}
    for rec in records:
        groups.setdefault(goal_key(rec, **kw), []).append(rec)
    return groups


# ═════════════════════════════════════════════════════════════════════════════
# V4 — the pre-registered repeatability decision (NULL-exit)
# ═════════════════════════════════════════════════════════════════════════════

#: **The pre-registered threshold.** A per-goal constant correction must remove
#: at least half the mean-square vertical residual, measured OUT OF SAMPLE
#: (leave-one-out within each goal cell). Below it the plan NULL-exits.
#:
#: Why 0.5, derived rather than chosen:
#:
#: * The null value is NEGATIVE, not zero. Under "no repeatable structure"
#:   (zero-mean noise) the leave-one-out mean is an independent draw, so
#:   ``E[MSE_after] = sigma^2.(1 + 1/(n-1)) > MSE_before`` and ``R_rep ->
#:   -1/(n-1)`` — about -0.14 at the 8-row cell. Over SEVERAL cells the statistic
#:   is pooled, so the null is the row-weighted sum
#:   ``-sum_k n_k/((n_k-1).N)`` — **-0.226** on the 2026-08-12 corpus's 6/8/3
#:   cells, which :func:`repeatability` reports as ``null_expectation``. (The
#:   pooled ``-1/(N-1)`` = -0.0625 is the null of a DIFFERENT statistic — one
#:   global leave-one-out mean — and quoting it here would understate the null
#:   by 3.6x.) Any positive value is already evidence; the threshold is not
#:   fighting a null at 0.
#: * 0.5 is the point where the correction removes more than it leaves. Below
#:   it, the post-correction task error is still dominated by the part learning
#:   cannot touch, and the Phase-3 A/B — whose criterion is a composite error
#:   reduction at k <= 3 — could not resolve the difference against the measured
#:   scatter.
#: * It is 10x the strongest spurious value a mean can manufacture at this n
#:   (``1/(1+n)`` = 0.05 for the in-sample form at n = 19).
REPEATABILITY_MIN = 0.5


def repeatability(records: Sequence[Dict[str, Any]], *, channel: str = 'flight_time_err_s',
                  min_per_goal: int = 2, **kw) -> Dict[str, Any]:
    """The V4 statistic: out-of-sample repeatable fraction of a scalar residual.

    ``R_rep = 1 − MSE_loo / MSE_raw`` where ``MSE_loo`` uses each row's own goal
    cell mean computed WITHOUT that row. Leave-one-out and not in-sample,
    because an in-sample per-cell mean at n = 3 removes a third of the variance
    by construction and would turn "we fitted 4 numbers to 19 points" into
    evidence.

    ``verdict`` is ``PASS`` or ``NULL_EXIT`` against
    :data:`REPEATABILITY_MIN` — pre-registered, and reported even when it passes
    comfortably, because the point of a pre-registered exit is that it is
    readable after the fact.
    """
    groups: Dict[Any, List[float]] = {}
    skipped = 0
    unkeyed = 0
    for rec in records:
        ok, _why = admit_record(rec)
        if not ok:
            skipped += 1
            continue
        value = _num(rec.get(channel))
        if value is None:
            skipped += 1
            continue
        key = goal_key(rec, **kw)
        if key is None:
            # A row whose goal cannot be recovered is NOT a goal cell of its
            # own: grouping the unkeyed rows together would let two tosses at
            # two different poses vouch for each other's repeatability, which is
            # the exact claim this statistic exists to make.
            unkeyed += 1
            continue
        groups.setdefault(key, []).append(value)

    per_goal: List[Dict[str, Any]] = []
    raw: List[float] = []
    loo: List[float] = []
    for key in sorted(groups, key=str):
        vals = np.asarray(groups[key], dtype=float)
        if vals.size < int(min_per_goal):
            per_goal.append({'goal': key, 'n': int(vals.size), 'skipped': True})
            continue
        resid = np.array([vals[i] - np.delete(vals, i).mean()
                          for i in range(vals.size)])
        per_goal.append({
            'goal': key, 'n': int(vals.size), 'skipped': False,
            'mean': float(vals.mean()), 'sd': float(vals.std(ddof=1)),
            'rms_raw': float(np.sqrt((vals ** 2).mean())),
            'rms_loo': float(np.sqrt((resid ** 2).mean()))})
        raw.extend(vals.tolist())
        loo.extend(resid.tolist())

    if not raw:
        raise IlcFitError(
            'repeatability: no goal cell has {} admitted rows — the statistic '
            'is undefined and that is a REFUSAL, not a null result (a null '
            'needs a measurement)'.format(int(min_per_goal)))
    raw_a = np.asarray(raw)
    loo_a = np.asarray(loo)
    mse_raw = float((raw_a ** 2).mean())
    r_rep = 1.0 - float((loo_a ** 2).mean()) / mse_raw if mse_raw > 0 else 0.0
    n_cells = sum(1 for g in per_goal if not g['skipped'])
    # The null is the PER-CELL leave-one-out null, not the pooled -1/(N-1).
    # Under zero-mean noise E[r_i^2] = sigma^2.n_k/(n_k-1) inside a cell of n_k
    # rows, so E[R_rep] = 1 - sum_k n_k^2/((n_k-1).N) = -sum_k n_k/((n_k-1).N).
    # Pooling instead would quote -1/(N-1) = -0.0625 on the 2026-08-12 corpus's
    # 6/8/3 cells where the statistic's own null is -0.226 — understating by 3.6x
    # exactly where the reader is asked to judge a small positive R_rep.
    null = -sum(float(g['n']) / max(1, g['n'] - 1)
                for g in per_goal if not g['skipped']) / float(raw_a.size)
    return {
        'channel': channel, 'n': int(raw_a.size), 'n_goal_cells': n_cells,
        'n_skipped_rows': skipped, 'n_unkeyed_rows': unkeyed,
        'mean': float(raw_a.mean()), 'sd': float(raw_a.std(ddof=1)),
        'rms_raw': float(np.sqrt(mse_raw)),
        'rms_loo': float(np.sqrt((loo_a ** 2).mean())),
        'R_rep': float(r_rep), 'threshold': REPEATABILITY_MIN,
        'null_expectation': float(null),
        'verdict': 'PASS' if r_rep >= REPEATABILITY_MIN else 'NULL_EXIT',
        'per_goal': per_goal,
    }


def channel_sensitivity(channel: str, goal: TossGoal) -> float:
    """``d(channel)/d(event_vel_trim)`` at this goal, through the production chain.

    The v1 command channel is one-dimensional, so V3's model prediction needs one
    number per scalar observable. ``flight_time_err_s`` reads it straight out of
    ``F``; ``release_speed_err_mms`` — the cross-check channel, deliberately not
    in ``e`` — gets it from :func:`release_speed_err_model`, which is the same
    difference the miner takes. Both are finite differences of production calls;
    neither is ``2/g`` written out by hand.
    """
    if channel == 'flight_time_err_s':
        return float(sensitivity(zero_command(), goal)[4, 2])
    if channel == 'release_speed_err_mms':
        h = FD_STEPS[2]
        plus = release_speed_err_model(np.array([0.0, 0.0, h, 0.0]), goal)
        minus = release_speed_err_model(np.array([0.0, 0.0, -h, 0.0]), goal)
        return float((plus - minus) / (2.0 * h))
    raise IlcFitError(
        'no production sensitivity for channel {!r} — v1 knows {}'
        .format(channel, ['flight_time_err_s', 'release_speed_err_mms']))


def held_out_prediction(records: Sequence[Dict[str, Any]], *,
                        channel: str = 'flight_time_err_s',
                        n_train: Optional[int] = None,
                        seed: int = 20260812, **kw) -> Dict[str, Any]:
    """V3: train the correction on a SUBSET, predict the held-out rows' residual.

    The correction is fitted **as a command**, not as a channel offset: the
    training rows' mean residual is divided by :func:`channel_sensitivity` at the
    training geometry to give ``du``, and each held-out row's residual is then
    predicted as ``e_i + s_i·du`` with ``s_i`` evaluated at THAT row's own goal.
    On this corpus the two geometries give the same ``s`` to 12 digits (the
    vertical sensitivity does not depend on pose), so ``predicted`` and the
    naive channel-mean subtraction coincide — but the code path is the one that
    would DIVERGE on a corpus spanning flight times, which is exactly where a
    wrong ``F`` would first show, and the return value reports both so the gap is
    visible rather than assumed away.

    Two splits, because at n = 19 both are worth having:

    * ``split`` — one deterministic train/test partition (the plan's own wording);
    * ``loo`` — leave-one-out over the whole corpus, the same estimator with n
      times the resolution, and the number to quote.
    """
    rows = [r for r in records if admit_record(r)[0]
            and _num(r.get(channel)) is not None and goal_of(r) is not None]
    if len(rows) < 4:
        raise IlcFitError(
            'held_out_prediction needs at least 4 admitted rows with a '
            'recoverable goal, got {}'.format(len(rows)))
    rng = np.random.default_rng(int(seed))
    order = rng.permutation(len(rows))
    k = int(n_train) if n_train is not None else int(round(0.7 * len(rows)))
    k = max(2, min(len(rows) - 1, k))
    train = [rows[i] for i in order[:k]]
    test = [rows[i] for i in order[k:]]

    def _du(subset) -> float:
        """The COMMAND the training subset asks for (event_vel_trim units)."""
        mean = float(np.mean([_num(r.get(channel)) for r in subset]))
        s = channel_sensitivity(channel, _goal_from_rows(subset))
        if s == 0.0:                                        # pragma: no cover
            raise IlcFitError('channel {} has zero sensitivity'.format(channel))
        return -mean / s

    def _predict(subset, du):
        vals = np.asarray([_num(r.get(channel)) for r in subset], dtype=float)
        sens = np.asarray([channel_sensitivity(channel, goal_of(r))
                           for r in subset], dtype=float)
        return vals, vals + sens * du

    du_train = _du(train)
    before, after = _predict(test, du_train)
    rms_before = float(np.sqrt((before ** 2).mean()))
    rms_after = float(np.sqrt((after ** 2).mean()))
    # PREDICTED vs ACTUAL, stated as a prediction with an ACTUAL to check it
    # against — the only such pair available offline. The model says the
    # residual is a per-goal constant, so the training set PREDICTS the held-out
    # set's mean; the held-out set MEASURES it. ``z`` is the miss in units of the
    # held-out mean's own standard error, so "predicted 0.0970 s, actual
    # 0.0955 s" is readable as agreement or not without a second table.
    actual_mean = float(before.mean())
    s_train = channel_sensitivity(channel, _goal_from_rows(train))
    predicted_mean = -du_train * s_train
    se_test = (float(before.std(ddof=1) / math.sqrt(before.size))
               if before.size > 1 else float('nan'))
    z = ((predicted_mean - actual_mean) / se_test
         if se_test and math.isfinite(se_test) and se_test > 0
         else float('nan'))

    loo_before: List[float] = []
    loo_after: List[float] = []
    for i, rec in enumerate(rows):
        others = [r for j, r in enumerate(rows) if j != i]
        b, a = _predict([rec], _du(others))
        loo_before.append(float(b[0]))
        loo_after.append(float(a[0]))
    lb = np.asarray(loo_before)
    la = np.asarray(loo_after)
    return {
        'channel': channel, 'n': len(rows),
        'split': {'n_train': len(train), 'n_test': len(test),
                  'du': du_train, 'correction': predicted_mean,
                  'predicted_test_mean': predicted_mean,
                  'actual_test_mean': actual_mean,
                  'se_test_mean': se_test, 'z': z,
                  'rms_before': rms_before, 'rms_after': rms_after,
                  'reduction': (1.0 - rms_after / rms_before) if rms_before else 0.0},
        'loo': {'rms_before': float(np.sqrt((lb ** 2).mean())),
                'rms_after': float(np.sqrt((la ** 2).mean())),
                'reduction': float(1.0 - np.sqrt((la ** 2).mean())
                                   / np.sqrt((lb ** 2).mean()))},
    }


# ═════════════════════════════════════════════════════════════════════════════
# Synthetic corpus — the V2 closed-loop driver
# ═════════════════════════════════════════════════════════════════════════════


def synthetic_corpus(goal: TossGoal, *, u_plant, n: int = 12,
                     sigma_e=None, seed: int = 20260812,
                     t0: float = 1786343595.0, dt_s: float = 6.0,
                     uptime_ms: int = 60000) -> List[Dict[str, Any]]:
    """A replayable corpus whose error is generated by the FORWARD model.

    ``u_plant`` is a KNOWN command-space perturbation of the plant: the machine
    behaves as if it had been commanded ``u_plant`` when it was commanded zero.
    The rows carry ``e_model(u_plant, goal)`` plus per-channel Gaussian noise, in
    the mined field names, so the whole fit — admission, key, residual, step —
    runs on them unchanged.

    The closed-loop property this exists to test (V2, the ``toss_fit_lib`` 2c
    pattern): fitting these rows must return ``du ≈ −u_plant``. A sign error does
    not merely fail that assertion, it fails it by DOUBLING the residual, which
    is what makes the test worth more than restating the update law.
    """
    e_true = e_model(u_plant, goal)
    v_true = release_speed_err_model(u_plant, goal)
    sigma = SIGMA_E if sigma_e is None else np.asarray(sigma_e, dtype=float)
    rng = np.random.default_rng(int(seed))
    x, y, z = (float(v) for v in goal.catch_pose_stow_mm)
    plane = _plane_of(goal, compute_release_state(goal.catch_pose_stow_mm,
                                                  goal.flight_time_s))
    rows: List[Dict[str, Any]] = []
    for i in range(int(n)):
        noise = rng.normal(0.0, 1.0, N_E) * sigma
        e = e_true + noise
        rows.append({
            'schema': 'toss_record/1',
            'toss_uid': 'syn-ilc-{}'.format(i),
            'announce_throw_time_ros': t0 + i * dt_s,
            'cmd_flight_time_s': float(goal.flight_time_s),
            'cmd_launch_vel_mms': [0.0, 0.0, float(np.linalg.norm(
                compute_release_state(goal.catch_pose_stow_mm,
                                      goal.flight_time_s).launch_vel_mms))],
            'cmd_release_source': 'announcement',
            'land_plane_mm': plane,
            'land_err_mm': [float(e[0]), float(e[1])],
            'land_xy_global_mm': [x + float(e[0]), y + float(e[1])],
            'arrival_dir_err_rad': [float(e[2]), float(e[3])],
            'flight_time_err_s': float(e[4]),
            'release_speed_err_mms': float(
                v_true + rng.normal(0.0, SIGMA_RELEASE_SPEED_MMS)),
            'backcast_fit_n': 60, 'arrival_fit_n': 60,
            'release_vel_se_mms': 7.5, 'arrival_vel_se_mms': 11.0,
            'usable_for_release_fit': True,
            # E-1's admission requirement, satisfied EXPLICITLY rather than by
            # omission: a synthetic corpus that skipped it would exercise the
            # lateral-channel refusal instead of the closed loop, and the V2a
            # aim channels would silently come back zero.
            'arc_fit_n': 120, 'arc_fit_rms_mm': 2.0,
            'arc_lateral_vel_se_mms': 1.5,
            'coverage_asym_s': 0.0, 'usable_for_lateral_fit': True,
            'uptime_ms_at_release': int(uptime_ms + i * dt_s * 1e3),
            'tilt_map_version': 'synthetic-tilt/1',
            'bridge_fw_version': '10 (proto 5)',
            'platform_fw_version': '2',
            'record_provenance': 'mined-only',
            # ── toss_trim's G1–G11, satisfied EXPLICITLY (2026-08-21) ──
            # Same argument as the E-1 block above, one guard family over: since
            # `admit_record` now RUNS those guards (build step 3), a synthetic
            # corpus that left these fields absent would exercise the refusal
            # path instead of the closed loop, and V2a's channels would come
            # back zero for a reason that has nothing to do with the fit. Each
            # value is what a healthy machine's record carries.
            'label': 'CAUGHT',                        # G9 — possession
            't_departure_raw_ros': t0 + i * dt_s + 0.17,   # G1 — release seen
            'ball_track_confirmed': True,
            'throw_stroke_seen': True,
            'gravity_correction_loaded': True,        # G5 — layer 0 identity
            'tilt_map_applied': True,
            'retry_of': None, 'reload_settle': None,  # G10 / G11 — not an
            #                                           interlude cycle
            'n_fit': 40,                              # G2 — landing-plane fit
            'fit_rms_mm': 0.5, 'fit_sparse': False,
            'dip_below_x3_rev': 0.0,                  # G4 — plant health
            'stroke_peak_rev': 9.0, 'trunc': False,
            # C7: tier 8a, so `throw_site_xy_mm` is meaningless and the goal xy
            # IS the cup xy — which is what this generator builds.
            'toss_tier': '8a',
        })
    del z
    return rows


def _goal_from_rows(rows: Sequence[Dict[str, Any]]) -> TossGoal:
    """The cell's representative goal: the centroid of its members' poses.

    Not the cell's nominal centre — the cell is a KEY, the fit is evaluated at
    the geometry the tosses were actually thrown at. On a corpus whose declared
    poses are recovered rather than declared, those differ by up to half a cell
    and the sensitivity is evaluated where the machine was.
    """
    goals = [goal_of(r) for r in rows]
    goals = [g for g in goals if g is not None]
    if not goals:
        raise IlcFitError(
            'no row in this group carries a recoverable goal — pass goal= '
            'explicitly rather than letting the fit invent one')
    pose = np.mean([g.catch_pose_stow_mm for g in goals], axis=0)
    # The throw site, when the cell has one (C7). Every 8b row admitted into a
    # cell sits within THROW_SITE_KEY_TOL_MM of its own cup — `admit_record`
    # refuses the rest by name — so a cell either has no site at all (8a, or
    # tier-unknown: `site_xy` then falls back to the cup, which is what 8a means)
    # or one that is the cup to float slack. Averaging is therefore exact, not a
    # compromise; the mean is taken so the code says WHICH site rather than
    # picking a row.
    sites = [g.throw_site_xy_mm for g in goals if g.throw_site_xy_mm is not None]
    site = (tuple(float(v) for v in np.mean(sites, axis=0)) if sites else None)
    return TossGoal(
        catch_pose_stow_mm=tuple(float(v) for v in pose),
        flight_time_s=float(np.mean([g.flight_time_s for g in goals])),
        throw_site_xy_mm=site,
        plane_mm=float(np.mean([g.plane_mm for g in goals])))


def pooled_error(rows: Sequence[Dict[str, Any]]) -> Tuple[np.ndarray, np.ndarray]:
    """``(e_meas, per-channel count)`` over rows — per-channel NANMEAN.

    Per channel, not per row: a row with a clean flight time and no landing fit
    is a perfectly good vertical-channel measurement, and dropping it because a
    channel the E-1 mask blocks anyway is missing would throw away 2 of 19 rows
    for nothing. A channel with no measurements at all comes back 0.0 with a
    count of 0 — the caller's mask is what decides whether that matters.
    """
    stack = np.vstack([measured_error(r) for r in rows])
    with np.errstate(invalid='ignore'):
        mean = np.nanmean(stack, axis=0)
    counts = np.sum(~np.isnan(stack), axis=0)
    return np.where(np.isnan(mean), 0.0, mean), counts


#: Evidence-gate verdicts, in increasing order of "and it will not come back".
EVIDENCE_PASS = 'PASS'
EVIDENCE_THIN = 'THIN'
EVIDENCE_INSIDE_SE = 'INSIDE_SE_GATE'
EVIDENCE_FROZEN = 'FROZEN_CUSUM'


def evidence_gate(rows: Sequence[Dict[str, Any]], *, mask=None) -> Dict[str, Any]:
    """The per-cell evidence gate — build step 3's second half. **Per channel.**

    Returns ``{'mask': (N_E,), 'channels': [...], 'frozen': bool,
    'frozen_monitor': bool, 'passed': (...), 'gated': (...)}`` where the mask is
    1.0 on a channel this cell may write a step from and 0.0 on one it may not.
    ``frozen`` is about the FITTED channels; a CUSUM alarm on a monitor column
    sets ``frozen_monitor`` instead, because nothing was going to be learnt from
    that column and telling an operator the loop froze when it did not is worse
    than saying nothing. Three refusals, each ``toss_trim``'s and each imported rather than
    re-derived, because every one of those constants carries a measurement that
    nobody is going to re-run:

    * **THIN** — fewer than ``toss_trim.N_MIN_APPLY`` = 6 measurements in the
      channel. Probed 2026-08-11 over 300–400 synthetic sessions: the design's
      ``n ≥ 3`` let **45.7 %** of ZERO-bias sessions command a non-zero
      correction.
    * **INSIDE_SE_GATE** — ``|mean| < toss_trim.SE_GATE · se`` = 2.5 standard
      errors. Chosen on the measured expected residual error in mm, not on the
      false-action rate: 2.5 halves 2.0's zero-bias cost (2.01 → 1.03 mm) for
      ≤ 0.3 mm at the biases worth correcting.
    * **FROZEN_CUSUM** — a two-sided tabular CUSUM (``k = 0.5``, ``h = 8.0``,
      armed at ``CUSUM_N_MIN`` = 5) alarms on the channel's series. 2 % false
      alarm over a 60-toss goal, 99.7 % detection of a 2σ shift within six
      tosses. The recursion is ``toss_trim.cusum_step`` — extracted 2026-08-21
      so the online trim and this batch scan drive ONE implementation of the
      arithmetic those numbers were measured against.

    **Why this module needed the gate at all.** ILC's only noise defence was the
    model-side SNR screen and ``ρ``: a three-row cell with wide scatter produced
    a confident-looking step, because the screen asks *"can this COMMAND move the
    task error above the noise?"* — a property of ``F`` and ``σ``, identical for
    every cell — and never *"does THIS CELL's measurement resolve above its own
    standard error?"*. Those are different questions and only the second one can
    see a thin, noisy cell.

    **``se`` is ``sd/√n``, not ``toss_trim``'s ``sd/√(n₀+n)``.** The trim shrinks
    toward a prior with ``n₀ = 4`` and its standard error is the shrunken
    estimator's; :func:`pooled_error` is a plain per-channel ``nanmean`` with no
    prior, so ``n₀`` would understate the error of the quantity actually being
    tested. Same gate, applied to the estimator that exists here.

    **Freeze-never-zero.** A gated channel contributes zero to the STEP, which is
    not the same as zeroing the correction: the update law is ``u_next = u_prev +
    du`` (:func:`propose_step`), so ``du_j = 0`` HOLDS the accumulated command at
    whatever the last admitted fit put there. The artifact keeps its value; only
    learning stops. That is ``toss_trim._freeze``'s rule — *"stop learning and
    HOLD δ; never zeroes"* — transposed to an accumulating law, and it is pinned
    by ``test_a_frozen_channel_holds_u_prev_rather_than_zeroing_it``.

    The CUSUM scans rows in the order given. :func:`load_corpus` sorts by
    ``toss_uid``, i.e. chronologically within a bag, which is what a change
    detector needs; a caller that re-orders a cell gets a detector reading a
    scrambled series and this docstring is the notice.
    """
    m = DEFAULT_MASK if mask is None else np.asarray(mask, dtype=float)
    stack = (np.vstack([measured_error(r) for r in rows])
             if len(rows) else np.zeros((0, N_E), dtype=float))
    out_mask = np.zeros(N_E, dtype=float)
    channels: List[Dict[str, Any]] = []
    frozen = False
    frozen_monitor = False
    for j, label in enumerate(E_LABELS):
        col = stack[:, j] if stack.shape[0] else np.array([])
        col = col[~np.isnan(col)]
        n = int(col.shape[0])
        entry: Dict[str, Any] = {
            'channel': label, 'index': j, 'n': n,
            'requested': bool(m[j] > 0.0),
            'mean': float(col.mean()) if n else float('nan'),
            'sd': float('nan'), 'se': float('nan'),
            'significance': float('nan'),
            'cusum_max': float('nan'), 'cusum_index': None,
            'verdict': EVIDENCE_THIN, 'detail': ''}
        if n >= 2:
            sd = float(max(col.std(ddof=1), 1e-12))
            se = sd / math.sqrt(n)
            entry['sd'] = sd
            entry['se'] = se
            entry['significance'] = abs(entry['mean']) / se if se else float('inf')
        # G8 first: a shift makes the mean meaningless, so a frozen channel is
        # frozen whatever its significance says.
        alarm_at, cusum_max = _cusum_scan(col)
        entry['cusum_max'] = cusum_max
        entry['cusum_index'] = alarm_at
        if alarm_at is not None:
            entry['verdict'] = EVIDENCE_FROZEN
            entry['detail'] = (
                'two-sided CUSUM alarm at sample {} (max |S| = {:.2f} against '
                'h = {:.1f}) — this channel SHIFTED inside the cell, so its '
                'pooled mean describes two plants. Learning stops and the '
                'accumulated command is HELD, never zeroed.'
                .format(alarm_at, cusum_max, toss_trim.CUSUM_H))
            # A MONITOR channel can alarm too, and it is worth shouting about --
            # a plane residual that shifts mid-cell is a plant statement. But it
            # is not a FIT freeze: nothing was going to be learnt from that
            # column, so there is no accumulated command being held. Reported
            # apart so an operator is not told the loop froze when it did not.
            if m[j] > 0.0:
                frozen = True
            else:
                frozen_monitor = True
        elif n < toss_trim.N_MIN_APPLY:
            entry['detail'] = ('{} measurements, below toss_trim.N_MIN_APPLY = {}'
                               .format(n, toss_trim.N_MIN_APPLY))
        elif entry['significance'] < toss_trim.SE_GATE:
            entry['verdict'] = EVIDENCE_INSIDE_SE
            entry['detail'] = ('|mean| = {:.6g} is {:.2f} se, inside '
                               'toss_trim.SE_GATE = {} — this cell has not '
                               'resolved a residual above its own noise'
                               .format(abs(entry['mean']),
                                       entry['significance'], toss_trim.SE_GATE))
        else:
            entry['verdict'] = EVIDENCE_PASS
            out_mask[j] = 1.0
        channels.append(entry)
    return {'mask': out_mask * (m > 0.0).astype(float),
            'channels': channels, 'frozen': frozen,
            'frozen_monitor': frozen_monitor,
            'passed': tuple(c['channel'] for c in channels
                            if c['verdict'] == EVIDENCE_PASS and m[c['index']]),
            'gated': tuple((c['channel'], c['verdict']) for c in channels
                           if c['verdict'] != EVIDENCE_PASS and m[c['index']])}


def _cusum_scan(col) -> Tuple[Optional[int], float]:
    """``(alarm_index, max|S|)`` for one channel's series. ``toss_trim``'s G8.

    One-step-ahead residual, exactly as :meth:`toss_trim.SessionTrim._cusum`
    defines it and for its measured reason: standardise sample ``i`` against the
    mean and sd of samples ``[:i]``, so the statistic is not referenced on a mean
    that already contains the shift it is testing for. Arms at
    ``toss_trim.CUSUM_N_MIN``; ``σ̂`` is ``toss_trim.history_sd``, the plain
    ``ddof=1`` sd whose docstring records why the design's MAD·1.4826 could not
    drive the false-alarm rate below 14.8 % at ANY ``(k, h)``.
    """
    col = np.asarray(col, dtype=float)
    s_p = 0.0
    s_m = 0.0
    peak = 0.0
    for i in range(int(col.shape[0])):
        if i < toss_trim.CUSUM_N_MIN:
            continue
        history = col[:i].reshape(-1, 1)
        sd = toss_trim.history_sd(history)
        if sd is None or not np.all(np.isfinite(sd)):
            continue
        z = (float(col[i]) - float(history.mean())) / float(sd[0])
        s_p, s_m = toss_trim.cusum_step(s_p, s_m, z)
        peak = max(peak, float(abs(s_p)), float(abs(s_m)))
        if toss_trim.cusum_alarmed(s_p, s_m):
            return i, peak
    return None, peak


def fit_corpus(records: Sequence[Dict[str, Any]], *, goal: Optional[TossGoal] = None,
               goal_cell: Any = '__modal__', pool_across_goals: bool = False,
               allow_cross_partition: bool = False,
               tau=None, rho: float = RHO_DAMPING, mask=None,
               uptime_max_ms: Optional[float] = None,
               require_evidence: bool = True) -> Dict[str, Any]:
    """One goal cell's fit: admitted rows → pooled ``e_meas`` → ``F`` → ``du``.

    **Per goal cell by default**, because the artifact is per-goal keyed: pooling
    two cells fits a correction to a pose the machine never threw from. Set
    ``pool_across_goals=True`` to pool deliberately — the return value records
    that it happened, and on the 2026-08-12 corpus it is a defensible thing to
    do (the three cells' vertical means, 0.087 / 0.101 / 0.112 s, are not
    resolved above their own standard errors) but it must be a decision, not a
    default.

    The **partition rule runs first** (:func:`select_partition`), so a corpus
    spanning two plants is refused before a single number is computed. On the
    2026-08-12 corpus that bites immediately and usefully: the three bags split
    16 / 3 on ``bridge_fw_version`` (10 vs 12).

    Passing ``goal=`` explicitly bypasses the cell selection entirely and fits
    every admitted row at that geometry — the shape a synthetic single-goal
    corpus wants, and the caller's responsibility on a real one.

    **``cell_goals`` is returned for every recoverable cell, not just the fitted
    one**, and it is the reason a POOLED caller can still gate per cell. A pooled
    fit has ONE ``goal`` — the modal cell's — but its ``du`` is written to every
    cell the corpus visited, and those cells differ in pose AND flight time. The
    production gates (:func:`admit_command`) are goal-dependent in both: the
    sequencer's flight-time band and workspace box, the tilt feasibility and the
    bridge's ``event_vel`` acceptance band all move with the goal. Validating
    every cell against the modal cell's geometry is therefore a gate that can
    pass for a command the machine would reject at the cell it is written to, so
    the geometry each cell would be fitted at is returned rather than left to a
    caller to re-derive (a second definition of "this cell's goal" is a second
    thing to drift).

    **``require_evidence`` (default True, 2026-08-21)** runs
    :func:`evidence_gate` over the fitted rows and intersects its per-channel
    verdict into the mask the step is solved under, so no step is written from a
    channel whose pooled residual is inside ``2.5·se``, or that carries fewer
    than 6 measurements, or that a CUSUM says shifted mid-cell. ``'evidence'`` is
    returned whether or not it was applied, so a caller that turns it off still
    reports what it would have said. ``'disagreement'`` is the C3 monitor census
    (:func:`disagreement_census`) and is always computed — it is a validation
    channel, never a gate.
    """
    admitted = [r for r in records
                if admit_record(r, uptime_max_ms=uptime_max_ms)[0]]
    if not admitted:
        raise IlcFitError('no admitted rows in this corpus')
    admitted, census, part_key, warnings = select_partition(
        admitted, allow_cross=allow_cross_partition)

    groups = group_by_goal(admitted)
    if goal is not None:
        rows = admitted
        cell = None
    else:
        if goal_cell == '__modal__':
            keyed = {k: v for k, v in groups.items() if k is not None}
            if not keyed:
                raise IlcFitError(
                    'no admitted row carries a recoverable goal cell')
            cell = max(keyed, key=lambda k: len(keyed[k]))
        else:
            # A CLI passes the cell as its printed string; accept either.
            matches = [k for k in groups
                       if k == goal_cell or str(k) == str(goal_cell)]
            if not matches:
                raise IlcFitError('goal cell {} is not in this corpus (cells: '
                                  '{})'.format(goal_cell,
                                               sorted(groups, key=str)))
            cell = matches[0]
        rows = admitted if pool_across_goals else groups[cell]
        goal = _goal_from_rows(groups[cell])

    e_meas, counts = pooled_error(rows)
    m = DEFAULT_MASK if mask is None else np.asarray(mask, dtype=float)
    # THE per-cell evidence gate (build step 3). ON by default: it is a safety
    # gate, and the corpus that is quiet enough not to need it is also the corpus
    # it costs nothing on. `require_evidence=False` is for the synthetic closed
    # loops (V2) whose whole job is to recover an INJECTED perturbation from a
    # noiseless corpus, where "this cell has not resolved a residual above its
    # own noise" is not a statement about the plant.
    gate = evidence_gate(rows, mask=m)
    # A channel with NO measurements comes back 0.0 from `pooled_error` with a
    # count of 0, and a 0.0 residual under a non-zero weight is not "no
    # information" — it is the assertion "this channel is already perfect",
    # which drives the step toward cancelling any correction the other channels
    # ask for on a coupled column. Invisible while the E-1 mask zeroed the
    # lateral channels anyway; reachable the moment it was lifted. So the mask
    # the step actually runs under is intersected with the counts, and BOTH are
    # returned so a report can say which channels were fitted rather than which
    # were requested.
    m_eff = m * (counts > 0).astype(float)
    if require_evidence:
        m_eff = m_eff * gate['mask']
    F = sensitivity(zero_command(), goal)
    step = propose_step(F, e_meas, goal, tau=tau, rho=rho, mask=m_eff)
    # Every recoverable cell's own geometry. `goal_key` returns non-None only
    # when `goal_of` did, so a non-None group can always produce one.
    cell_goals = {k: _goal_from_rows(v) for k, v in groups.items()
                  if k is not None}
    return {'goal': goal, 'goal_cell': cell, 'cell_goals': cell_goals,
            'pooled': bool(pool_across_goals),
            'n_admitted': len(admitted), 'n_rows': len(rows),
            'e_meas': e_meas, 'channel_counts': counts, 'F': F,
            'mask': m, 'mask_effective': m_eff,
            'du': step['du'], 'u_next': step['u_next'],
            'shrinks': step['shrinks'], 'refusals': step['refusals'],
            'screen': screen_channels(F, mask=m_eff),
            'evidence': gate, 'evidence_applied': bool(require_evidence),
            'disagreement': disagreement_census(rows),
            'partition_census': census, 'partition_key': part_key,
            'warnings': warnings,
            'goal_cells': dict((k, len(v)) for k, v in groups.items()),
            'uptime': uptime_census(rows)}
