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

Two numbers in the tree round this differently and neither is wrong so much as
stale: ``toss_fit_lib``'s header says 3126.5 and ``toss_trim``'s says 3126.64.
A third, ``aim_target_offset_mm``'s **54.578 mm/deg**, is a different quantity
altogether — the SECANT gain at a full 1° aim, where ``tan(1°)/1°`` adds 0.011 %
— against this module's derivative-at-zero 54.5718 mm/deg. Both are correct;
they are not interchangeable, and :func:`sensitivity` reports the derivative
because that is what a Jacobian is.

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
    raise NotImplementedError(
        'catch-side channel {!r} is DECLARED, NOT IMPLEMENTED in v1. Two open '
        'prerequisites, neither of them "we ran out of time": G-2 (the '
        'hand-ODrive braking clamp is in the catch-softness path, so a fit '
        'against it fits the clamp), and Phase 0b outcome (ii) — the measured '
        'impact transient did not separate from its matched cross-label '
        'control, so the modelled contact-velocity surrogate has no validation '
        'channel yet. Seam, for whoever implements it: {}'
        .format(name, dict((c[0], c[2]) for c in CATCH_CHANNELS)[name]))


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

#: **THE MODULE-WIDE DEFAULT MASK — full-size since 2026-08-13.**
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
DEFAULT_MASK = np.ones(N_E, dtype=float)

#: Per-channel measurement noise, in each channel's own unit. **PROVISIONAL**
#: until a corpus larger than 19 rows exists, in the convention
#: ``toss_fit_lib.N_MIN`` already uses. Every number is measured on the
#: 2026-08-12 corpus, not assumed:
#:
#: * ``land_err``  — per-axis sd 15.7 / 13.6 mm over the 17 rows with a landing
#:   fit; pooled to one isotropic 14.7 mm rather than split, because the physics
#:   is isotropic and two numbers at n = 17 is fitting the axes.
#: * ``arrival_dir`` — per-axis sd 0.00267 / 0.00203 rad, pooled to 0.00238.
#: * ``flight_time`` — sd 0.0139 s. Deliberately the RAW scatter and not the
#:   0.0122 s the 23 %-plant / 77 %-noise decomposition implies: a Q weight that
#:   under-states the noise over-trusts the channel, and the decomposition rests
#:   on one correlation coefficient at n = 19.
#:
#: **STALE IN ONE CHANNEL, deliberately not fixed here (2026-08-13).** The E-1
#: re-mine changed the lateral estimator, so ``arrival_dir``'s measured scatter
#: moved: on the re-mined corpus the per-axis sd is 0.00296 / 0.00308, pooled
#: **0.00302 rad**, i.e. 27 % above the 0.00238 sitting in this array. ``land_err``
#: (14.73 vs 14.7) and ``flight_time`` (0.01385 vs 0.0139) are unchanged, because
#: their estimators were. This array is NOT updated because Gate 1 approved
#: ``Q = diag(1/sigma^2)`` with these numbers on 2026-08-13 and re-deriving an
#: approved weight is an owner decision, not a tidy-up. The cost of leaving it is
#: bounded and measured: it over-trusts ``arrival_dir`` relative to ``land_err``,
#: moving the pooled aim requirement from |aim| = 0.00997 to 0.01044 rad
#: (57.1 % -> 59.8 % of the D7 authority) — inside the authority either way.
#:
#: **The gap the weights are arbitrating is NOT noise, and this is the finding to
#: carry forward.** The two lateral channels disagree SYSTEMATICALLY, in y only,
#: by +18.0 mm at the plane (pooled; +17.4 / +19.5 / +15.4 across the three goal
#: cells) while agreeing in x to under 1.3 mm. ``land_err_mm`` is a POSITION fit
#: at the catch plane and so carries the centroid bias b_y(z_plane) absolutely;
#: ``arrival_dir_err_rad`` is a whole-arc VELOCITY and is bias-immune. So the gap
#: is a direct read-out of the standing E-1 caveat — only the bias GRADIENT was
#: measured, never the absolute offset — and its size (~18 mm) is the same order
#: as the measured b_y profile's own span (21 mm). Until the fixtured no-robot
#: capture closes that, a mocap-closed aim loop converges to the measurement's
#: cup, not the world's, and these two channels are how far apart those are.
SIGMA_E = np.array([14.7, 14.7, 0.00238, 0.00238, 0.0139], dtype=float)

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
    different systems. Since E-1 closed (2026-08-13) that default is FULL SIZE;
    the historical masked answer is an EXPLICIT ``mask=E1_MASK``.
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
    :func:`conditioning`), which is FULL SIZE since E-1 closed. Rule 1 still
    needs the full-size matrix internally and computes it regardless of what was
    asked for, so the historical ``mask=E1_MASK`` report is unchanged.
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
    2. the channels themselves present — ``land_err_mm`` and
       ``arrival_dir_err_rad``.

    The threshold is IMPORTED from the miner rather than restated, for the same
    reason ``_lean_rad`` is: one definition of the gate, or the corpus and the
    fit disagree about which rows are in it.
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
    if _pair(rec.get('land_err_mm')) is None:
        return False, 'no land_err_mm'
    if _pair(rec.get('arrival_dir_err_rad')) is None:
        return False, 'no arrival_dir_err_rad'
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
    """
    if not rec.get('usable_for_release_fit'):
        return False, 'not usable_for_release_fit (the miner\'s own gate)'
    if _num(rec.get('flight_time_err_s')) is None:
        return False, 'no flight_time_err_s'
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
    """
    out = np.full(N_E, np.nan, dtype=float)
    if lateral_admissible(rec)[0]:
        out[0], out[1] = _pair(rec.get('land_err_mm'))
        out[2], out[3] = _pair(rec.get('arrival_dir_err_rad'))
    ft = _num(rec.get('flight_time_err_s'))
    if ft is not None:
        out[4] = ft
    return out


def goal_of(rec: Dict[str, Any]) -> Optional[TossGoal]:
    """The :class:`TossGoal` a mined row was thrown at, or ``None``.

    ``goal_catch_xyz_stow_mm`` is a DECLARATION field and every mined-only row
    has it null, so the pose is recovered from the two mined fields that bracket
    it: ``land_xy_global_mm − land_err_mm`` is the announced cup xy by the
    miner's own definition of ``land_err_mm``, and the cup xy IS the goal xy for
    an 8a toss (``stow_to_global_mm`` adds only z). The z comes from
    ``land_plane_mm`` walked back down the two GENERATED offsets — never a third
    copy of the geometry.
    """
    T = _num(rec.get('cmd_flight_time_s'))
    land = _pair(rec.get('land_xy_global_mm'))
    err = _pair(rec.get('land_err_mm'))
    plane = _num(rec.get('land_plane_mm'))
    if T is None or land is None or err is None or plane is None:
        return None
    z = plane - float(hw.GEOM_INITIAL_HEIGHT_MM) - float(hw.HAND_CATCH_OFFSET_MM)
    return TossGoal(catch_pose_stow_mm=(land[0] - err[0], land[1] - err[1], z),
                    flight_time_s=T, plane_mm=plane)


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
            'label': 'CAUGHT',
            'record_provenance': 'mined-only',
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
    return TossGoal(
        catch_pose_stow_mm=tuple(float(v) for v in pose),
        flight_time_s=float(np.mean([g.flight_time_s for g in goals])),
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


def fit_corpus(records: Sequence[Dict[str, Any]], *, goal: Optional[TossGoal] = None,
               goal_cell: Any = '__modal__', pool_across_goals: bool = False,
               allow_cross_partition: bool = False,
               tau=None, rho: float = RHO_DAMPING, mask=None,
               uptime_max_ms: Optional[float] = None) -> Dict[str, Any]:
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
            'partition_census': census, 'partition_key': part_key,
            'warnings': warnings,
            'goal_cells': dict((k, len(v)) for k, v in groups.items()),
            'uptime': uptime_census(rows)}
