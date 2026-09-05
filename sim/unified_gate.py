"""Unified-mode sim gate — the Phase-4 production-chain-in-the-loop harness.

Plan: ``plans/active/unified-7dof-planner.md`` § 4 Phase 4 ("Sim gate") and
§ 5 **T-I2**.  This is the unified-mode variant of ``sim/toss_gate.py``: where
that gate drives the LEGACY stack (``toss_release`` + ``planner.build_catch`` +
the firmware stroke engine's Python mirror), this one drives the UNIFIED one,
and it drives every hop of it for real::

    unified_cycle.plan_launch / plan_steady / plan_landing / extend
        │                     the planner — one 7-channel CyclePlan per window,
        │                     chained at the release instants, gated by
        │                     feasibility.validate_cycle
        ▼
    motion.trajectory.emitter.KnotEmitter.frame()          40 Hz
        │                     the REAL emitter, sampling (τ, τ+dt, τ+2·dt) and
        │                     assembling the real make_mpc_command dict
        ▼
    teensy_link.setpoint_pump.SetpointPump.build()
        │                     the REAL bridge pump: per-channel step gates, the
        │                     all-or-nothing hand key rule, HAS_HAND / HAS_V1
        ▼
    Setpoint.pack() → bytes → Setpoint.unpack()
        │                     the REAL v6 wire (208 B, float32 lanes) — packed
        │                     and unpacked, so every number the mirror sees has
        │                     been through the same quantisation the Teensy sees
        ▼
    teensy_interp.TeensyLegInterp.tick() / .tick_hand()    500 Hz
        │                     the firmware mirror: 6 leg lanes (the motor_guard-
        │                     validated ladder) + the FW 17 hand lane, both with
        │                     the transmitted-v1 Mode-1 rule
        ▼
    sim.plant.mujoco_plant.MuJoCoPlant
                              legs commanded from the mirror's per-tick rev
                              targets, the hand from the mirror's hand lane

Nothing between the planner and the plant is re-implemented here.  That is the
point: a harness that re-derived the emitter's field set, the pump's gates or
the interpolator's ladder would be gating a different machine than the one
Phase 5 flies.

THE CAPTURE MODEL, AND WHAT IS ADVISORY
---------------------------------------
The acceptance authority is ``toss_gate``'s **kinematic capture model** — a
``contact_carry=False`` plant (``sim/toss_gate.py``'s own plant, constructed the
same way): MuJoCo contact triggers the make, and the carry is then a kinematic
hold rather than contact physics.  Contact PHYSICS (``contact_carry=True``,
``toss_gate``'s ``--diag-release`` column) is not run here at all, and the
per-tick contact detail that is reported — ``capture_dist_mm``, the make/drop
counts — is reported, never gated on (owner resolution, 2026-08-29; the same
resolution ``sim/cycle_gate.py`` records).  The reasons are that gate's:
hardware catches are already smooth, and the MuJoCo contact model is the
low-fidelity element.  Phase 5's UH ladder is the seating authority.

THE BANDS (``core_clean``)
--------------------------
Mirrors ``toss_gate``'s vocabulary, with the two reactive-arm terms REMOVED and
four chain terms added:

* ``caught`` / ``held_at_end`` — every planned catch made, and the ball still
  held when the plan ends (``toss_gate``'s terms, same plant, same mechanism).
* ``validate_ok`` — ``feasibility.validate_cycle`` returned ``OK`` on the whole
  chained plan at the session limits.
* ``pump_clean`` — ``pump_rejects == 0`` **and** ``accepted == emitted``.  The
  second half is the non-vacuous form: a gate that only counted rejects would
  pass on a stream that emitted nothing (``toss_gate``'s invariant, verbatim).
* ``mirror_ok`` — the firmware mirror's 500 Hz reconstruction of the plan is
  within :data:`MIRROR_TOL_HAND_REV` on the hand lane and
  :data:`MIRROR_TOL_LEG_REV` on the six leg lanes (the two bounds have
  different provenance — see their docstrings; the hand is exactly
  reconstructible and the legs are not).  This is the band that makes the wire
  and the interpolator load-bearing rather than decorative.
* ``flags_ok`` — every accepted frame carried ``HAS_U1|HAS_U2|HAS_HAND|HAS_V1``.
  A silently hand-less frame is a hand that free-runs on the firmware's decay
  ladder, which is exactly the failure the flag discipline exists to prevent.
* ``hold_travel`` / ``hold_tilt`` / ``separation_ms`` — ``gate_common``'s
  quiescence thresholds over the post-plan settle, and ``toss_gate``'s
  post-capture separation bound.
* ``preposition_err`` — the settle onto the plan's knot 0 before streaming
  starts, at ``toss_gate``'s 2 mm tolerance.
* ``no_slam`` / ``runway_active`` / ``seat_ok`` / ``capture_ok`` —
  ``cycle_gate``'s four plan-domain bands, imported thresholds and all.

**DROPPED, deliberately: ``catch_armed`` and ``hand_arm_infeasible``.**
``toss_gate`` requires both because its catch is REACTIVE — an arm-and-forget
``HandCatchSequence`` dispatched from the tracked flight — so "a ball fell into
a statically-parked cup" has to be excluded from a pass.  Under unified mode
there is no reactive arm at all: the catch stroke is *in the plan*, sampled from
the same clock as the platform, and ``catch_coordinator_node``'s dispatch is
gated off.  There is therefore no arm to have fired, and a band asserting one
did would be unfalsifiable.  What replaces it as the silent-dead-hand guard is
``flags_ok`` + ``mirror_ok``: the hand lane must have been commanded, on the
wire, every frame, and the firmware must have reconstructed it.

THE TWO CYCLE SETS
-------------------
1. **The single-toss set** — ``LAUNCH`` then ``LANDING``, chained with
   ``extend``, on the band ``toss_gate`` pins.  See :func:`default_grid` for how
   the legacy (xy, height, flight, displacement) tiers map onto ``CycleGoals``.
   Acceptance: ``core_clean >= ceil(0.9 n)`` over the binding points.
2. **The two-pose constant-beat set** — ``LAUNCH`` + ``STEADY``×k +
   ``LANDING``, the cup alternating between two sites, one ball riding the whole
   ring.  Acceptance: EVERY cycle clean and the beat exact
   (:data:`BEAT_TOL_S`), which is a strictly harder bar than the 90 % band and
   is meant to be: a ring that drops one cycle in ten is not a ring.

Run headless (default); writes a JSON report under ``temp/reports/``.
Pure Python + MuJoCo; no ROS2.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import math
import os
import sys
import time

import numpy as np

# Single path bootstrap (repo root, ros_ws pkg, config/generated);
# see sim/_paths.py.  Runnable entry scripts only — library modules under
# sim/ never touch sys.path.
_repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _repo_root not in sys.path:
    sys.path.insert(0, _repo_root)
from sim._paths import bootstrap_paths  # noqa: E402
bootstrap_paths()

# The firmware mirror lives with the xref that validates it against motor_guard
# (tools/probes/.../hermite_xref).  Imported from there rather than copied here:
# a second transcription of the interpolation ladder under sim/ would be a
# second thing to keep in step with leg_interp.cpp, and the xref would not be
# watching it.
#
# Loaded BY FILE PATH rather than by putting its directory on ``sys.path``.  The
# import-style contract (``tests/sim/test_sim_import_style.py``) allows an entry
# script exactly ONE mutation — the repo-root preamble above that
# ``bootstrap_paths()`` needs — and the root cause it protects is hand-derived
# roots drifting apart from the four ``sim/_paths.py`` owns.  A second root here
# is exactly that shape.  Naming the FILE creates no root to drift: there is one
# path, it is derived from this file's own location, and it points at the
# module the xref validates.  Registered in ``sys.modules`` under its own name so
# the xref's other consumers (which run with that directory on the path for their
# own reasons) still see one module identity rather than two.
_XREF_DIR = os.path.join(_repo_root, 'tools', 'probes',
                         'teensy_link_profiling', 'hermite_xref')


def _load_teensy_interp():
    import importlib.util
    if 'teensy_interp' in sys.modules:
        return sys.modules['teensy_interp']
    spec = importlib.util.spec_from_file_location(
        'teensy_interp', os.path.join(_XREF_DIR, 'teensy_interp.py'))
    module = importlib.util.module_from_spec(spec)
    sys.modules['teensy_interp'] = module
    spec.loader.exec_module(module)
    return module


import mujoco                                                     # noqa: E402

import jugglebot.hardware_config as hw                            # noqa: E402
from jugglebot.motion.geometry import StewartGeometry             # noqa: E402
from jugglebot.motion.ik_solver import (                          # noqa: E402
    pose_to_leg_lengths, rotvec_to_rot_matrix,
)
from jugglebot.motion import unified_cycle as uc                  # noqa: E402
from jugglebot.motion.trajectory import KnotEmitter               # noqa: E402
from jugglebot.motion.trajectory import TrajectoryLimits          # noqa: E402
from jugglebot.motion.trajectory import ballistics_bc as bal      # noqa: E402
from jugglebot.motion.trajectory import cup_cycle as cc           # noqa: E402
from jugglebot.motion.trajectory import feasibility as fz         # noqa: E402
from jugglebot.motion.trajectory import throw_envelope            # noqa: E402
from jugglebot.motion.trajectory import tilt_geometry as tg       # noqa: E402
from jugglebot.motion.trajectory.hand_stroke import (             # noqa: E402
    LINEAR_GAIN_REV_PER_M,
)

from teensy_link.protocol import Setpoint                         # noqa: E402
from teensy_link.setpoint_pump import (                           # noqa: E402
    FLAG_HAS_HAND, FLAG_HAS_U1, FLAG_HAS_U2, FLAG_HAS_V1, SetpointPump,
)

ti = _load_teensy_interp()

from sim.plant.mujoco_plant import MuJoCoPlant                    # noqa: E402
from sim.gate_common import (                                     # noqa: E402
    HOLD_TILT_DEG, HOLD_TRAVEL_MM, KNOT_DT_S, SEPARATION_MS, ViewerClosed,
    attach_viewer, tilt_change_deg, travel_mm,
)
# cycle_gate's plan-domain thresholds and its exact-integer pass threshold are
# IMPORTED, never restated: the two gates must agree on what a clean cycle is,
# and a second copy of 2.0 mm is a number that drifts.
from sim.cycle_gate import (                                      # noqa: E402
    CAPTURE_TOL_MM, SEAT_CONE_DEG, SEAT_FORCE_FLOOR_MPS2, _pass_threshold,
)

# ---------------------------------------------------------------------------
# The operating point
# ---------------------------------------------------------------------------

#: Cup-opening heights (mm, GLOBAL) at the release, the catch and at rest.  The
#: first two are ``cycle_gate``'s ``THROW_CUP_Z_M`` / ``CATCH_CUP_Z_M`` in this
#: module's mm frame — the same measured optimum (release height sets the
#: slider runway underneath it, which is what bounds flight time under the z
#: pin).  ``REST`` is ``tests/motion/test_unified_cycle.py``'s reference rest
#: site: low enough that the pre-launch dip fits, high enough to be inside the
#: cup-z box.  It is where each set's cycle STARTS from; where it comes to REST
#: is :func:`_settle_site`, which is a different height and deliberately so.
THROW_CUP_Z_MM = 860.0
CATCH_CUP_Z_MM = 830.0
REST_CUP_Z_MM = 750.0


def _settle_site(catch_site_mm):
    """The rest site a window is aimed at — the PRODUCTION rule, not a gate one.

    ``reload_coordinator_node._unified_cycle_request`` settles every window at
    *the catch xy, at the parked cup height*, and this gate exists to run the
    choreography the coordinator actually asks for.  The z half is the load-
    bearing one: a cycle that stops at the 830 mm catch height leaves the hand at
    4.755 rev, and ``toss_sequencer``'s CHECKING gate then refuses the NEXT cycle
    ``REJECTED_HAND_NOT_PARKED`` (band 0.5 rev) before it plans — so a gate that
    settled at the catch would be scoring a cycle the machine can only fly once.

    ``unified_cycle.SETTLE_CUP_Z_MM`` (689.6 mm = 0.316 rev) is that parked height
    clamped up into the planner's own cup box; its derivation lives there.
    """
    c = np.asarray(catch_site_mm, dtype=float)
    return np.array([float(c[0]), float(c[1]), float(uc.SETTLE_CUP_Z_MM)])

#: LAUNCH window length (s) — rest → release.  The reference value
#: ``tests/motion/test_unified_cycle.py`` plans its launch fixture at.
LAUNCH_PERIOD_S = 0.6

#: Every other window's length (s) — ``cycle_gate``'s ``PERIOD_S``, so the two
#: gates describe the same beat.  Measured 2026-09-04: a LANDING window of 1.0 s
#: cannot absorb a 50 mm displaced catch (``LIMIT_JERK`` at 155-198 k mm/s³
#: against the 150 k session limit) while 1.4 s carries the whole ring at
#: 81-90 k, so the displaced band would otherwise be refused for the window
#: length rather than for the reach.
CYCLE_PERIOD_S = 1.4

#: Firmware interpolation tick (s) — 500 Hz, ``leg_interp.cpp``'s ISR rate.
TICK_S = 0.002

#: How long the host keeps quiet after the last frame, in seconds.  Covers the
#: firmware's whole wind-down ladder (``SEGMENT_T_S`` 0.025 + ``MAX_EXTRAP_DT_S``
#: 0.05 + ``EXTRAP_DECAY_DT_S`` 0.06 = 0.135 s) with room to settle, and gives
#: the hold-quiescence window something to measure.
QUIET_TAIL_S = 0.25

#: Worst |mirror − plan| (rev) the HAND reconstruction band allows.
#:
#: PROVENANCE — a wire-quantisation bound, not a tuned one, and the hand is the
#: channel where that is the WHOLE story.  ``CyclePlan.hand_at`` is a cubic
#: Hermite over ``(hand_rev, hand_vel_rps)`` on the 25 ms knot grid; the emitter
#: samples exactly those knots; the pump forwards them; the firmware's Mode 1
#: rebuilds the same Hermite from the same four numbers under ``HAS_V1``.  The
#: two are therefore the SAME curve, and the only difference is that the four
#: numbers went through float32 on the wire.  At the hand's ~10 rev operating
#: point a float32 half-ulp is **4.77e-7 rev** (``2^3 · 2^-24``), and the Hermite
#: basis is a PARTITION OF UNITY over the two position terms plus two
#: velocity terms scaled by ``T`` (~2.5 rev at a 100 rev/s knot, half-ulp
#: 1.2e-7) — so the quantisation errors are averaged, not summed, and the
#: arithmetic floor is ~5e-7 rev rather than the ~2e-6 an earlier reading of this
#: comment claimed.  MEASURED over the full grid (``python sim/unified_gate.py
#: --no-viewer``, run 2026-09-05, with the production settle site): worst
#: **4.766e-07 rev** across all 27 trials —
#: one half-ulp, as predicted.  4e-6 rev (this band) is therefore ~8× the floor
#: and 1.3e-4 mm of slider; ``tests/sim/test_unified_gate.py`` additionally holds
#: the observed worst to HALF the band, which is 4× the floor.
MIRROR_TOL_HAND_REV = 4.0e-6

#: Worst |mirror − plan| (rev) the LEG reconstruction band allows.
#:
#: PROVENANCE — the legs are NOT exactly reconstructible, by construction, and
#: this number is that difference measured rather than a tolerance chosen.  The
#: plan is a cubic Hermite in POSE space; the emitter runs the IK at the three
#: knots and the firmware rebuilds a cubic Hermite in REV space between them.
#: The IK is nonlinear, so a rev-space cubic through IK(pose) at the knots is
#: not IK of a pose-space cubic in between — the two agree AT every knot and
#: differ by the IK's curvature over one 25 ms segment in between.  That
#: difference is a property of the real chain, present on hardware, and no
#: amount of firmware fidelity removes it.  MEASURED
#: (``python sim/unified_gate.py --no-viewer``, run 2026-09-05, with the
#: production settle site): 9.891e-08 rev on a co-located toss (where the
#: platform barely moves), 9.46e-07 on the 50 mm displaced ring and 1.065e-05 rev
#: on the 60 mm two-pose ring (where it moves most) — i.e. 7.5e-4 mm of leg
#: extension at the worst point on the grid.  5e-4 rev (35 µm) is ~47x the worst
#: observed and still two orders below the 0.10 rev firmware lead clamp.
#:
#: Those three numbers were 9.891e-08 / 2.94e-05 / 8.086e-05 before the settle
#: site became the production one (2026-09-05).  The gate used to bring the cup
#: back to the THROW xy at 750 mm after every catch, so every displaced point
#: paid a lateral traverse the coordinator never asks for; settling at the CATCH
#: xy removes it, and with it most of the platform motion this residual is
#: proportional to.  The band is unchanged — it was never near either value —
#: but ``tests/sim/test_unified_gate.py``'s non-vacuity check now carries more of
#: the weight, and its own ring numbers are measured there.
#:
#: HOW FAR OUTSIDE A FAULT LANDS, measured rather than asserted
#: (``/tmp/probe_f2_mirror.py``, 2026-09-04, run twice bit-identically).  A
#: dropped lane or a stale latch lands orders out.  The ``v1`` rule does NOT,
#: and the difference matters: masking ``HAS_V1`` moves the 60 mm ring's leg
#: reconstruction to 5.836e-04 rev — 1.17× the band — and moves a CO-LOCATED
#: toss's by nothing at all (9.891e-08 rev either way), because with the
#: platform nearly still the leg extension is near-linear over a 25 ms knot and
#: the ``(u2−u1)/T`` fallback reproduces the transmitted ``v1``.  So this band
#: catches a v1 regression only where there is platform motion to catch it in.
#: ``tests/sim/test_unified_gate.py::test_the_mirror_band_is_non_vacuous`` pins
#: both halves — the ring failing, and the toss's silence — so neither is
#: rediscovered the hard way.  The HAND band, by contrast, moves four orders
#: past its bound on every plan.
MIRROR_TOL_LEG_REV = 5.0e-4

#: Beat tolerance (s) for the constant-beat set.  The release instants are sums
#: of window durations, each an exact multiple of the 25 ms knot grid, so the
#: only spread is float summation: measured **4.441e-16 s** over a five-release
#: ring (``python sim/unified_gate.py --set beat``, run 2026-09-04).  1e-9 s is six orders above that and still 5 000× finer
#: than one 500 Hz tick.
BEAT_TOL_S = 1.0e-9

#: Pre-stream settle tolerance (mm) onto the plan's knot 0 — ``toss_gate``'s
#: ``preposition_tol_mm``.  Measured 0.028 mm on every point of the grid
#: (2026-09-04) — the settle is 1 s of hold at the plan's own knot 0.
PREPOSITION_TOL_MM = 2.0

#: Session limits — ``cycle_gate`` / ``toss_gate`` / ``reload_gate`` parity.
_SESSION_LEG_VEL_MMPS = 250.0
_SESSION_LEG_ACC_MMPS2 = 3000.0
_SESSION_LEG_JERK_MMPS3 = 150000.0

_ADVISORY_NOTE = (
    "An ADVISORY point is run, scored and reported in full but never gated. "
    "The one advisory point in the shipped grid is the legacy z = 200 tier at "
    "T = 0.80 s: mapped onto the cup it asks for a release 30 mm higher, which "
    "takes 30 mm out of the slider runway underneath it, and the LANDING is "
    "then refused with HAND_LIMIT_ACC (4666 rev/s^2 against the 3500 cap, "
    "measured 2026-09-04). That is the cap doing its job on a physically "
    "harder throw, not a defect, and cycle_gate's _ADVISORY_FLIGHTS_S sets the "
    "precedent for saying so rather than burning the 90 % band on it.")

_DROPPED_BANDS_NOTE = (
    "catch_armed and hand_arm_infeasible are toss_gate bands that do NOT "
    "appear here: under unified mode there is no reactive hand arm to fire "
    "(the catch stroke is in the plan and catch_coordinator's SetHandTrajCmd "
    "dispatch is gated off), so a band asserting one fired would be "
    "unfalsifiable. flags_ok + mirror_ok replace them as the silent-dead-hand "
    "guard: the hand lane must be on the wire every frame and the firmware "
    "must reconstruct it.")

_LEG_CLAMP_NOTE = (
    "The LEG lane in this gate's firmware mirror is motor_guard's clamp, NOT "
    "FW 17's. teensy_interp.py deliberately keeps motor_guard's leg numbers "
    "(MAX_LEAD_REV 0.15 rev, and a lead-clamped leg has its vel_ff ZEROED) "
    "because that file exists to be cross-checked against motor_guard "
    "(tools/probes/.../hermite_xref) and porting the firmware's leg clamp into "
    "it would break the xref it serves. leg_interp.cpp:62-63,641-646 ships "
    "LEAD 0.10 rev with VELFF_CAP 3.5 rev/s and never zeroes the feedforward. "
    "So total_leg_lead_clamp_ticks below is motor_guard's verdict on the "
    "stream, and a real board would clamp SOONER (0.10 < 0.15) and more gently "
    "(bounded vel_ff, not zero). The HAND lane is the firmware's own, "
    "constants and all (MAX_LEAD_HAND_REV 2.0), and is pinned to "
    "canbridge_config.h by tests/firmware/test_hermite_xref.py.")

_CONTACT_NOTE = (
    "The gating capture model is toss_gate's contact->kinematic-hold "
    "(contact_carry=False). capture_dist_mm and the per-catch contact detail "
    "below are REPORTED, never gated (owner resolution 2026-08-29): hardware "
    "catches are already smooth and the MuJoCo contact model is the "
    "low-fidelity element. Phase 5's UH ladder is the seating authority.")


# ---------------------------------------------------------------------------
# The grid — how the legacy tiers map onto CycleGoals
# ---------------------------------------------------------------------------

#: ``toss_gate``'s five xy sweep poses (``_XY_POINTS_MM``).
_XY_POINTS_MM = ((0.0, 0.0), (60.0, 60.0), (60.0, -60.0),
                 (-60.0, 60.0), (-60.0, -60.0))
#: ``toss_gate``'s 2-3 m/s binding band: the derived envelope floor and 0.60 s.
_SPEED_BAND_FLIGHTS_S = (throw_envelope.MIN_FLIGHT_TIME_S, 0.60)
#: ``toss_gate``'s orchestrator-amendment binding band.
_T080_S = 0.80
#: ``toss_gate``'s z sweep, as platform-z offsets from the active plane.
_Z_OFFSETS_MM = (-30.0, +30.0)
_Z_SWEEP_FLIGHTS_S = (0.60, 0.80)
#: ``toss_gate``'s Tier-8b BINDING displaced ring (the Rung-2a reliable box).
_RING_MM = 50.0
_R2 = math.sqrt(0.5)
_RING_8DIR = ((1.0, 0.0), (_R2, _R2), (0.0, 1.0), (-_R2, _R2),
              (-1.0, 0.0), (-_R2, -_R2), (0.0, -1.0), (_R2, -_R2))

#: Two-pose constant-beat set: site separation (mm), flight (s), and how many
#: STEADY cycles ride between the LAUNCH and the LANDING.  60 mm is inside
#: ``gate_common.REACH_ENVELOPE_MM`` (80) with margin; 4 STEADY windows give the
#: five releases and five catches the "≥ 4 cycles" requirement asks for.
BEAT_DISPLACEMENT_MM = 60.0
BEAT_FLIGHT_S = 0.60
BEAT_STEADY_CYCLES = 4


def _point_id(point: dict) -> str:
    if point['kind'] == 'ring':
        return 'ring_d%.0f_T%.2f_n%d' % (point['displacement_mm'],
                                         point['flight_s'],
                                         point['steady_cycles'])
    dx, dy = point['catch_dxy_mm']
    return 'x%g_y%g_dz%+g_T%.3f_d%+g%+g' % (
        point['xy_mm'][0], point['xy_mm'][1], point['z_offset_mm'],
        point['flight_s'], dx, dy)


def single_toss_grid() -> list:
    """The single-toss set, mapped point-for-point off ``toss_gate``'s grid.

    **THE MAPPING.**  ``toss_gate`` sweeps ``(x, y, z_platform, T)``: a platform
    POSE to throw and catch at, plus a flight time, with Tier 8b adding a catch
    site displaced from the throw site.  ``CycleGoals`` wants CUP-OPENING sites.
    The four tiers map like this, and the mapping is exact rather than
    approximate in three of the four:

    * **xy** — identity.  At level tilt the cup opening sits directly over the
      platform centroid (``cup_xy = centroid_xy + arm·cup_axis_xy`` and
      ``cup_axis_xy`` is zero at level), so a legacy xy pose and a cup xy site
      are the same two numbers.  The unified planner reaches them by translating
      the platform, exactly as the legacy pre-position did.
    * **flight T** — identity.  Both are the ballistic time of flight, and both
      sides derive the take-off velocity from it under the same 9806 mm/s²
      (``ballistics_bc`` on one side, ``cup_cycle.takeoff_velocity`` on the
      other; ``unified_cycle`` imports the first so the two cannot drift).
      ``catch_t_s`` is pinned EQUAL to the flight time: one ball, thrown at the
      end of the LAUNCH window and caught ``T`` later, so the catch instant is
      not a free parameter here the way it is in ``cycle_gate``'s two-ball
      steady state.
    * **displacement (Tier 8b)** — identity, applied to the catch site (and to
      the throw TARGET, which is the same point: the ball has to land where the
      cup will be).
    * **height z — NOT identity, and this is the one that needs saying.**  The
      legacy gate moves the PLATFORM in z.  Under unified mode z is pinned
      (``unified_z_float_enabled`` false ⇒ every emitted pose carries
      ``z == JB_OP_DEFAULT_ACTIVE_Z_MM`` exactly), so the platform cannot go
      there.  What CAN is the slider: ``cup_z = CUP_Z_BASE_MM + slider +
      (pose_z − active_z)`` is linear in both terms with the same unit
      coefficient, so a legacy platform-z offset Δz and a cup-z offset Δz put
      the cup opening in the *same place*.  The tier is therefore mapped onto
      the cup height and the machine reaches it differently — same cup, same
      ball, different actuator.  The cost of the substitution is real and shows
      up in the grid: the slider spends Δz of its stroke doing what the legs
      used to, so the +30 mm point at T = 0.80 s runs out of runway under the
      release and is refused (see :data:`_ADVISORY_NOTE`).

    ``toss_gate``'s long-flight rungs (``_FLIGHT_HIGH_S`` 0.974 and
    ``_FLIGHT_MAX_S`` 1.148) are NOT in this grid at all: the unified planner's
    measured ceiling is 0.80 s under the 3500 rev/s² hand cap with z pinned
    (``cycle_gate.max_plannable_flight_s``), so those rungs are not a band this
    machine has, and running them would only re-measure a known refusal.
    """
    pts = []
    for T in _SPEED_BAND_FLIGHTS_S:               # the 2-3 m/s binding band
        for xy in _XY_POINTS_MM:
            pts.append(dict(kind='toss', xy_mm=xy, z_offset_mm=0.0,
                            flight_s=float(T), catch_dxy_mm=(0.0, 0.0),
                            advisory=False))
    for xy in _XY_POINTS_MM:                      # the orchestrator amendment
        pts.append(dict(kind='toss', xy_mm=xy, z_offset_mm=0.0,
                        flight_s=_T080_S, catch_dxy_mm=(0.0, 0.0),
                        advisory=False))
    for dz in _Z_OFFSETS_MM:                      # the z sweep, on the cup
        for T in _Z_SWEEP_FLIGHTS_S:
            pts.append(dict(kind='toss', xy_mm=(0.0, 0.0), z_offset_mm=dz,
                            flight_s=float(T), catch_dxy_mm=(0.0, 0.0),
                            advisory=bool(dz > 0.0 and T >= _T080_S)))
    for ux, uy in _RING_8DIR:                     # the Tier-8b binding ring
        pts.append(dict(kind='toss', xy_mm=(0.0, 0.0), z_offset_mm=0.0,
                        flight_s=_T080_S,
                        catch_dxy_mm=(_RING_MM * ux, _RING_MM * uy),
                        advisory=False))
    return pts


def beat_grid() -> list:
    """The two-pose constant-beat set."""
    return [dict(kind='ring', displacement_mm=BEAT_DISPLACEMENT_MM,
                 flight_s=BEAT_FLIGHT_S, steady_cycles=BEAT_STEADY_CYCLES,
                 advisory=False)]


def default_grid() -> list:
    return single_toss_grid() + beat_grid()


# ---------------------------------------------------------------------------
# Plan-domain scoring (cycle_gate's bands, on this gate's multi-event clock)
# ---------------------------------------------------------------------------

def seat_angles(cup, tilts, spans):
    """``(max_angle_deg, n_scored)`` — ``cycle_gate.seat_angles``'s criterion.

    Identical physics and identical thresholds; the only difference is the
    WINDOW.  ``cycle_gate.seat_angles`` reads its carry window from a module
    constant (one catch, at ``PERIOD_S × CATCH_FRAC``), which a chained plan
    with up to five catches has no equivalent of — so the spans are passed in
    as ``[(k_start, k_end), ...]`` and the worst over all of them is returned.

    The criterion itself, restated because it is the load-bearing part: a seated
    ball feels the specific force ``g − a_cup`` and stays in the cup while that
    vector lies inside the cup's cone about the cup axis.  Knots where the field
    is weaker than :data:`~sim.cycle_gate.SEAT_FORCE_FLOOR_MPS2` are skipped —
    in free fall the ball is weightless and the angle to a vanishing field is
    both numerically arbitrary and physically irrelevant.
    """
    g = np.array([0.0, 0.0, -float(hw.GRAVITY_MPS2)])
    worst = 0.0
    scored = 0
    for k0, k1 in spans:
        for k in range(max(0, k0), min(int(k1) + 1, cup.acc.shape[0])):
            f = g - cup.acc[k]
            mag = float(np.linalg.norm(f))
            if mag < SEAT_FORCE_FLOOR_MPS2:
                continue
            up = -f / mag
            ax = tg.cup_axis(*tilts[k])
            c = float(np.clip(np.dot(ax, up), -1.0, 1.0))
            worst = max(worst, float(np.degrees(np.arccos(c))))
            scored += 1
    return worst, scored


def _carry_spans(meta, n_knots):
    """``[(k_catch, k_next_release)]`` — the knots a ball is in the cup for."""
    dt = float(meta.dt)
    rel = sorted(float(m.t_s) for m in meta.releases)
    spans = []
    for cm in meta.catches:
        k0 = int(min(math.ceil(float(cm.t_s) / dt), n_knots - 1))
        nxt = [r for r in rel if r > float(cm.t_s) + 1e-9]
        k1 = (int(min(math.floor(nxt[0] / dt), n_knots - 1)) if nxt
              else n_knots - 1)
        if k1 >= k0:
            spans.append((k0, k1))
    return spans


def runway_margin_m(catch_vel_mm_s, catch_cup_z_mm, cup_cfg) -> float:
    """``cycle_gate``'s runway band, read back from the planner's own requirement.

    A boolean echo of ``catch_runway_enabled`` would pass just as happily on a
    program whose runway row had stopped being assembled; this asks
    ``cup_cycle.catch_runway_requirement`` what the arrival actually needs and
    checks the catch leaves that much stroke under it.
    """
    floor_m, need_m = cc.catch_runway_requirement(
        np.asarray(catch_vel_mm_s, dtype=float) / 1000.0, cup_cfg)
    return float(catch_cup_z_mm / 1000.0 - floor_m - need_m)


# ---------------------------------------------------------------------------
# The falling-edge decay probe (pure mirror, no plant)
# ---------------------------------------------------------------------------

def hand_decay_probe(plan, geom) -> dict:
    """Cut the stream mid-stroke and watch the hand lane wind down.

    **Why this is a separate probe and not a column of the main run.**  Every
    plan in both sets ENDS at rest, so the falling edge at plan expiry is a
    falling edge on a hand that is already stationary — it observes nothing.
    The firmware rule is about the other case: HAS_HAND falling while the hand
    is MOVING (a host gap, a backstop freeze to a hand-less plan), where holding
    Mode 1's ``s = 1`` endpoint would keep commanding it with ``vel_ff = v1``,
    i.e. hold-at-last-command from up to 200 rev/s.  So the probe cuts the
    stream at the knot of peak hand speed and ticks the mirror forward with no
    further frames.

    Returns the observed ladder: the velocity at the moment of the cut, the age
    at which the lane leaves Mode 1, the age at which the velocity first reaches
    EXACTLY zero, whether it is monotone non-increasing in magnitude after the
    Taylor phase, and the total travel the wind-down adds.
    """
    emitter = KnotEmitter(geom)
    pump = _make_pump()
    mirror = _make_mirror(geom)
    dt = float(plan.dt)
    k_cut = int(np.argmax(np.abs(np.asarray(plan.hand_vel_rps))))
    k_cut = max(1, min(k_cut, int(plan.n_knots) - 3))

    fb = [0.0] * 6
    hand_fb = float(plan.hand_rev[0])
    for seq in range(k_cut + 1):
        tau = seq * dt
        sp, _ = pump.build(emitter.frame(plan, tau, seq),
                           t_origin_us=int(seq * 25000))
        if sp is None:
            return {'ok': False, 'reason': 'pump rejected the pre-cut stream'}
        _latch(mirror, Setpoint.unpack(sp.pack()), tau)
        hand_fb = float(sp.u0[6])
    # A generous, motionless feedback anchor: the probe is about the LADDER, so
    # the lead clamp must not be what stops the lane.  ±MAX_LEAD_HAND_REV around
    # a frozen encoder is 2.0 rev of room, and the wind-down travels far less.
    t_cut = k_cut * dt
    v_at_cut = float('nan')
    pos_at_cut = None
    cmd_at_cut = None
    t_leave_mode1 = None
    t_zero = None
    peak_abs_after = 0.0
    monotone = True
    prev_abs = None
    trace = []
    cmd_last = 0.0
    lead_ticks_at_cut = int(mirror.hand_lead_clamp_ticks)
    t = t_cut
    while t <= t_cut + 0.40 + 1e-12:
        mirror.tick(t, fb)
        out = mirror.tick_hand(t, hand_fb)
        if out is None:
            # A silent lane means the probe measured nothing at all — report it
            # as a failed probe rather than crashing the whole gate run, the
            # same shape the pump-reject exit above uses.
            return {'ok': False,
                    'reason': 'the hand lane went silent mid-probe (age %.4f s)'
                              % (t - mirror.hand_ts)}
        cmd_last = float(out[0])
        age = t - mirror.hand_ts
        v = float(mirror.hand_raw_vel)
        if pos_at_cut is None:
            pos_at_cut = float(mirror.hand_raw_pos)
            cmd_at_cut = cmd_last
            v_at_cut = v
        # WHEN the lane left Mode 1, read off the LANE's own rung rather than
        # inferred from the age.  ``age > SEGMENT_T_S`` is the same test the
        # ladder uses to decide, so recording it here would be recording the
        # question, not the answer: it could not fail even on a lane that held
        # Mode 1 forever.  ``hand_mode`` is what the lane actually did.
        if t_leave_mode1 is None and int(mirror.hand_mode) != 1:
            t_leave_mode1 = age
        if int(mirror.hand_mode) != 1:
            if prev_abs is not None and abs(v) > prev_abs + 1e-9:
                monotone = False
            prev_abs = abs(v)
            peak_abs_after = max(peak_abs_after, abs(v))
            if t_zero is None and v == 0.0:
                t_zero = age
        trace.append((age, v, float(mirror.hand_raw_pos)))
        t += TICK_S
    return {
        'ok': True,
        'cut_knot': k_cut,
        'v_at_cut_rps': v_at_cut,
        'age_left_mode1_s': t_leave_mode1,
        'age_velocity_zero_s': t_zero,
        'decay_deadline_s': (ti.SEGMENT_T_S + ti.MAX_EXTRAP_DT_S
                             + ti.EXTRAP_DECAY_DT_S),
        # The deadline is a firmware age; the probe samples it on the 500 Hz
        # tick grid, so the first tick AT OR AFTER the deadline is the earliest
        # observation that can report zero.  One tick of slack, not a fudge.
        'sampling_slack_s': TICK_S,
        'monotone_after_mode1': bool(monotone),
        'travel_after_cut_rev': float(trace[-1][2] - pos_at_cut),
        # What the AXIS would actually have been commanded: the same wind-down
        # after the hand lead clamp against a (deliberately frozen) encoder.
        # The gap between the two is the point — the raw ladder coasts a long
        # way from a fast cut and MAX_LEAD_HAND_REV is the only thing that
        # bounds it, which is why that constant is 2.0 rev and not 5.0.
        'clamped_travel_after_cut_rev': float(cmd_last - cmd_at_cut),
        'max_lead_hand_rev': ti.MAX_LEAD_HAND_REV,
        # The clamp's own witness.  A wind-down travel merely LARGER than 1 rev
        # would not say the clamp engaged (the band is 2.0 rev); this counts the
        # ticks on which it actually did, over the probe window alone.
        'lead_clamp_ticks': int(mirror.hand_lead_clamp_ticks) - lead_ticks_at_cut,
        'final_vel_rps': float(trace[-1][1]),
        'peak_abs_vel_after_mode1_rps': peak_abs_after,
    }


# ---------------------------------------------------------------------------
# Chain plumbing
# ---------------------------------------------------------------------------

def _make_pump() -> SetpointPump:
    """A ``SetpointPump`` built from the SAME config chain the bridge uses.

    ``teensy_bridge_node`` passes ``max_step_hand_rev`` as
    ``JB_TRAJ_HAND_VEL_LIMIT_RPS × JB_TRAJ_KNOT_DT_S`` rather than leaving it on
    the module default; re-deriving it the same way here is what makes a pump
    reject in this gate mean the same thing it would mean on the bridge.
    """
    return SetpointPump(
        mm_to_rev=hw.GEOM_MM_TO_REV,
        max_step_rev=hw.JB_OP_MAX_POSITION_STEP_REV,
        max_step_hand_rev=(float(hw.JB_TRAJ_HAND_VEL_LIMIT_RPS)
                           * float(hw.JB_TRAJ_KNOT_DT_S)),
        torque_ff_enabled=bool(hw.DYNAMICS_TORQUE_FF_ENABLED),
        torque_ff_max_nm=float(hw.DYNAMICS_TORQUE_FF_MAX_NM),
        torque_wire_scale=float(hw.ODRIVE_LEG_TORQUE_WIRE_SCALE),
        torque_ff_ramp_frames=int(math.ceil(
            float(hw.DYNAMICS_TORQUE_FF_RAMP_S)
            / float(hw.JB_TRAJ_KNOT_DT_S))))


class _DummyIPC:
    """No-op IPC so ``MotorGuard`` needs no ZMQ broker (``xref._DummyIPC``)."""

    def recv_all(self):
        return []

    def send_telemetry(self, msg):
        pass

    @property
    def seconds_since_last_recv(self):
        return 0.0

    def close(self):
        pass


def _make_mirror(geom) -> 'ti.TeensyLegInterp':
    """The firmware mirror, with the stroke bounds the firmware actually ships.

    ``MotorGuard``'s ``_stroke_{min,max}_rev`` are the same numbers as
    ``canbridge_config.h``'s ``STROKE_{MIN,MAX}_REV`` — pinned by
    ``tests/firmware/test_hermite_xref.py::test_firmware_stroke_bounds_match_motor_guard``
    — so taking them from the guard keeps ONE source rather than a third copy.
    """
    import jugglebot.motion.motor_guard as mg              # noqa: PLC0415
    guard = mg.MotorGuard(geom=geom, ipc=_DummyIPC())
    return ti.TeensyLegInterp(guard._stroke_min_rev, guard._stroke_max_rev)


def _latch(mirror, sp: Setpoint, t_latch: float) -> None:
    """Latch one DECODED wire frame into the mirror, flags and all."""
    has_u1 = bool(sp.flags & FLAG_HAS_U1)
    has_u2 = bool(sp.flags & FLAG_HAS_U2)
    has_v1 = bool(sp.flags & FLAG_HAS_V1)
    mirror.latch_setpoint(
        sp.u0[:6], sp.v0[:6], sp.accel[:6], sp.torque_ff[:6], t_latch,
        u1=(sp.u1[:6] if has_u1 else None),
        u2=(sp.u2[:6] if has_u2 else None),
        v1=(sp.v1[:6] if has_v1 else None))
    if sp.flags & FLAG_HAS_HAND:
        mirror.latch_hand(
            sp.u0[6], sp.v0[6], t_latch,
            u1=(sp.u1[6] if has_u1 else None),
            u2=(sp.u2[6] if has_u2 else None),
            v1=(sp.v1[6] if has_v1 else None),
            accel=sp.accel[6])


def slider_mm_of_rev(rev: float, cfg) -> float:
    """Hand motor rev → sim slider mm (``cup_realize``'s relation, inverted)."""
    return (float(rev) / LINEAR_GAIN_REV_PER_M * 1000.0
            + float(cfg.slider_rev_zero_mm))


def rev_of_slider_mm(mm: float, cfg) -> float:
    return (float(mm) - float(cfg.slider_rev_zero_mm)) / 1000.0 \
        * LINEAR_GAIN_REV_PER_M


# ---------------------------------------------------------------------------
# Config + result records
# ---------------------------------------------------------------------------

@dataclasses.dataclass
class UnifiedGateConfig:
    points: list = None
    seed: int = 0
    leg_vel_mmps: float = _SESSION_LEG_VEL_MMPS
    leg_acc_mmps2: float = _SESSION_LEG_ACC_MMPS2
    leg_jerk_mmps3: float = _SESSION_LEG_JERK_MMPS3
    #: Run the MuJoCo column (the plant, the ball, the execution metrics).  With
    #: it off the gate scores the plan + chain only — used by the unit tests to
    #: keep the per-commit subset cheap.
    plant_column: bool = True
    #: Run the falling-edge decay probe on the first accepted plan.
    decay_probe: bool = True
    report_path: str = None


@dataclasses.dataclass
class UnifiedTrialResult:
    """One cycle set, scored.  NaN for unavailable, never ``None``."""

    idx: int
    point_id: str
    kind: str
    advisory: bool
    flight_s: float
    displacement_mm: float

    accepted: bool = False
    reject_code: str = None

    # ── gating bands ──
    validate_ok: bool = False
    pump_clean: bool = False
    mirror_ok: bool = False
    flags_ok: bool = False
    caught: bool = False
    held_at_end: bool = False
    capture_ok: bool = False
    seat_ok: bool = False
    no_slam: bool = False
    runway_active: bool = False
    quiescent: bool = False
    prepositioned: bool = False
    beat_exact: bool = True
    core_clean: bool = False

    # ── measured, reported ──
    n_knots: int = 0
    duration_s: float = float('nan')
    n_releases: int = 0
    n_catches: int = 0
    #: ``unified_cycle.latest_supersede_time_s`` of the plan that was streamed:
    #: the last plan time at which a frame off it still tells the truth about the
    #: next knot.  ``inf`` for a rest-terminal plan (both shipped sets end at
    #: rest, so this is the expected value here); a finite number would mean the
    #: gate streamed a plan with a release-terminal cliff.  REPORTED, never
    #: gated — the deadline belongs to whoever installs the next window.
    latest_supersede_s: float = float('nan')
    plan_wall_s: float = float('nan')
    stream_wall_s: float = float('nan')
    validate_code: str = ''
    peak_leg_vel_mmps: float = float('nan')
    peak_leg_acc_mmps2: float = float('nan')
    peak_leg_jerk_mmps3: float = float('nan')
    peak_hand_vel_rps: float = float('nan')
    peak_hand_acc_rps2: float = float('nan')
    peak_hand_rev: float = float('nan')
    pump_frames_emitted: int = 0
    pump_frames_accepted: int = 0
    pump_rejects: int = 0
    flags_seen: tuple = ()
    mirror_leg_worst_rev: float = float('nan')
    mirror_hand_worst_rev: float = float('nan')
    stream_leg_phase_worst_rev: float = float('nan')
    stream_hand_phase_worst_rev: float = float('nan')
    leg_lead_clamp_ticks: int = 0
    leg_stroke_clamp_ticks: int = 0
    hand_lead_clamp_ticks: int = 0
    hand_clip_ticks: int = 0
    hand_unseen_skips: int = 0
    hand_dev_max_rev: float = float('nan')
    ticks: int = 0
    makes: int = 0
    drops: int = 0
    capture_dist_mm: float = float('nan')
    exec_cup_err_mm: float = float('nan')
    plan_capture_dist_mm: float = float('nan')
    separation_ms: float = float('nan')
    hold_travel_mm: float = float('nan')
    hold_tilt_deg: float = float('nan')
    preposition_err_mm: float = float('nan')
    max_seat_angle_deg: float = float('nan')
    seat_scored_knots: int = 0
    pre_catch_hand_max_rev: float = float('nan')
    worst_runway_margin_m: float = float('nan')
    worst_runway_margin_rev: float = float('nan')
    max_tilt_deg: float = float('nan')
    beat_period_s: float = float('nan')
    beat_worst_dev_s: float = float('nan')

    def to_dict(self) -> dict:
        d = dataclasses.asdict(self)
        d['flags_seen'] = [hex(f) for f in self.flags_seen]
        return d


# ---------------------------------------------------------------------------
# The gate
# ---------------------------------------------------------------------------

class UnifiedGate:
    """Runs :func:`default_grid` (or ``cfg.points``) through the whole chain."""

    def __init__(self, cfg: UnifiedGateConfig = None):
        self.cfg = UnifiedGateConfig() if cfg is None else cfg
        self.geom = StewartGeometry()
        self.limits = TrajectoryLimits.from_config(hw).with_session_limits(
            leg_vel_mmps=self.cfg.leg_vel_mmps,
            leg_acc_mmps2=self.cfg.leg_acc_mmps2,
            leg_jerk_mmps3=self.cfg.leg_jerk_mmps3)
        self.rcfg = uc.build_realize_config(self.limits)
        self.cup_cfg = uc.build_cup_config()
        self.mm_to_rev = np.asarray(hw.GEOM_MM_TO_REV, dtype=float)
        self.emitter = KnotEmitter(self.geom)
        self.viewer = None
        self._plant = None
        self._site_id = None

    @property
    def plant(self):
        """The gating plant — ``toss_gate``'s, constructed the same way.

        ``contact_carry=False``: contact triggers the make, the carry is a
        kinematic hold.  See :data:`_CONTACT_NOTE`.
        """
        if self._plant is None:
            self._plant = MuJoCoPlant(geom=self.geom)
            self._site_id = mujoco.mj_name2id(
                self._plant.model, mujoco.mjtObj.mjOBJ_SITE, 'hand_opening')
        return self._plant

    # ── planning ──────────────────────────────────────────────────────────

    def _rest_state(self, xy, cup_z_mm) -> uc.CycleState:
        """A stationary state whose cup opening sits at ``(xy, cup_z_mm)``.

        The POSE and the SLIDER are derived through the level realisation
        (``slider = cup_z − base``), so the inputs owe nothing to the forward
        map.  ``CycleState.at_rest`` then calls
        ``unified_cycle.cup_state_from_platform`` to fill the state's
        ``cup_pos_mm``, so the resulting CUP POSITION does go through that map —
        which is the right thing here (the planner is what this gate is gating)
        and is separately pinned to ~1e-13 mm by
        ``tests/motion/test_unified_cycle.py``'s round-trip tests.
        """
        cfg = self.rcfg
        slider_mm = float(cup_z_mm) - float(cfg.cup_z_base_mm)
        rev = ((slider_mm - float(cfg.slider_rev_zero_mm)) / 1000.0
               * LINEAR_GAIN_REV_PER_M)
        pose = np.array([float(xy[0]), float(xy[1]), float(cfg.active_z_mm),
                         0.0, 0.0, 0.0])
        return uc.CycleState.at_rest(pose, rev, cfg)

    def plan_single_toss(self, point: dict):
        """``LAUNCH`` then ``LANDING``, chained — the single-toss cycle set."""
        x, y = point['xy_mm']
        dz = float(point['z_offset_mm'])
        dx, dy = point['catch_dxy_mm']
        T = float(point['flight_s'])
        throw = np.array([x, y, THROW_CUP_Z_MM + dz])
        catch = np.array([x + dx, y + dy, CATCH_CUP_Z_MM + dz])
        start = np.array([x, y, REST_CUP_Z_MM + dz])
        settle = _settle_site(catch)

        state = self._rest_state((x, y), start[2])
        plan, meta = uc.plan_launch(
            uc.CycleGoals(period_s=LAUNCH_PERIOD_S, throw_site_mm=throw,
                          throw_target_mm=catch, flight_s=T,
                          settle_site_mm=settle),
            state, self.limits, self.geom)
        catch_vel = bal.arrival_velocity(meta.release_vel_mm_s, T)
        plan_b, meta_b = uc.plan_landing(
            uc.CycleGoals(period_s=CYCLE_PERIOD_S, catch_site_mm=catch,
                          catch_vel_mm_s=catch_vel, catch_t_s=T,
                          settle_site_mm=settle),
            uc.release_state_from_meta(meta, plan), self.limits, self.geom)
        return uc.extend(plan, meta, plan_b, meta_b, self.limits, self.geom)

    def plan_ring(self, point: dict):
        """``LAUNCH`` + ``STEADY``×k + ``LANDING`` over two alternating sites.

        The ring is SELF-CONSISTENT rather than nominally periodic, and that is
        what makes the beat a claim about physics instead of about bookkeeping:
        each window's catch site is the previous window's throw TARGET, each
        window's catch velocity is ``ballistics_bc.arrival_velocity`` of the
        previous window's PLANNED release velocity, and the catch instant is the
        flight time.  One ball rides the whole thing.

        Site ``S_{i+1}`` is where window ``i`` both catches and throws — the cup
        travels ``S_i → S_{i+1}`` during the ball's flight, dwells, and releases
        toward ``S_i`` again.  With two sites that is the alternation.
        """
        d = float(point['displacement_mm'])
        T = float(point['flight_s'])
        n = int(point['steady_cycles'])
        throw_sites = [np.array([0.0, 0.0, THROW_CUP_Z_MM]),
                       np.array([d, 0.0, THROW_CUP_Z_MM])]
        catch_sites = [np.array([0.0, 0.0, CATCH_CUP_Z_MM]),
                       np.array([d, 0.0, CATCH_CUP_Z_MM])]
        start = np.array([0.0, 0.0, REST_CUP_Z_MM])

        state = self._rest_state((0.0, 0.0), start[2])
        plan, meta = uc.plan_launch(
            uc.CycleGoals(period_s=LAUNCH_PERIOD_S,
                          throw_site_mm=throw_sites[0],
                          throw_target_mm=catch_sites[1], flight_s=T,
                          settle_site_mm=_settle_site(catch_sites[1])),
            state, self.limits, self.geom)
        for i in range(n):
            here, there = (i + 1) % 2, i % 2
            goals = uc.CycleGoals(
                period_s=CYCLE_PERIOD_S,
                throw_site_mm=throw_sites[here],
                throw_target_mm=catch_sites[there], flight_s=T,
                catch_site_mm=catch_sites[here],
                catch_vel_mm_s=bal.arrival_velocity(meta.release_vel_mm_s, T),
                catch_t_s=T)
            p2, m2 = uc.plan_steady(
                goals, uc.release_state_from_meta(meta, plan),
                self.limits, self.geom)
            plan, meta = uc.extend(plan, meta, p2, m2, self.limits, self.geom)
        last = (n + 1) % 2
        p2, m2 = uc.plan_landing(
            uc.CycleGoals(period_s=CYCLE_PERIOD_S,
                          catch_site_mm=catch_sites[last],
                          catch_vel_mm_s=bal.arrival_velocity(
                              meta.release_vel_mm_s, T),
                          catch_t_s=T,
                          settle_site_mm=_settle_site(catch_sites[last])),
            uc.release_state_from_meta(meta, plan), self.limits, self.geom)
        return uc.extend(plan, meta, p2, m2, self.limits, self.geom)

    # ── the stream ────────────────────────────────────────────────────────

    def _stream(self, plan, meta) -> dict:
        """Drive the plan through emitter → pump → wire → mirror → plant.

        Returns the measured chain + execution metrics.  The 500 Hz tick loop is
        the firmware's, not the plan's: frames are latched when their 25 ms
        instant ARRIVES, and the mirror runs off its OWN latched phase in
        between.  The two grids do not divide (12.5 ticks per knot), so every
        other frame lands a tick after its nominal instant — which is the real
        board's situation, where a UDP arrival is asynchronous to the
        interpolation ISR, and it is why the reconstruction is scored at the
        mirror's phase and the wall-clock offset is reported separately.
        """
        plant = self.plant
        cfg = self.rcfg
        mirror = _make_mirror(self.geom)
        pump = _make_pump()
        dt = float(plan.dt)
        rel_t = [float(m.t_s) for m in meta.releases]
        rel_v = [np.asarray(m.vel_mm_s, dtype=float) for m in meta.releases]
        cat_t = [float(m.t_s) for m in meta.catches]

        # 1. Settle onto the plan's knot 0, then seat the ball.
        pose0 = np.asarray(plan.pose[0], dtype=float)
        plant.reset(pose0)
        plant.command(plant.pose_to_extensions(pose0))
        plant.command_hand(slider_mm_of_rev(float(plan.hand_rev[0]), cfg))
        for _ in range(40):
            plant.step(KNOT_DT_S)
            if self.viewer is not None:
                self.viewer.sync()
        st = plant.get_state()
        preposition_err_mm = float(np.linalg.norm(
            np.asarray(st.platform_pos_mm) - pose0[:3]))
        plant.ball_manager.ball(0).spawn_in_hand()
        for _ in range(4):
            plant.step(plant.timestep)

        # 2. Stream.
        t_start = plant.data.time
        t_end = t_start + float(plan.total_duration) + QUIET_TAIL_S
        n_frames = int(math.floor(float(plan.total_duration) / dt)) + 1
        next_frame_t = t_start
        seq = 0
        latch_tau = 0.0
        emitted = accepted = 0
        flags_seen = set()
        worst_leg = worst_hand = 0.0
        worst_leg_phase = worst_hand_phase = 0.0
        ticks = 0
        n_rel = n_cat = 0
        makes = drops = 0
        capture_dist = float('nan')
        exec_cup_err = float('nan')
        separation_ms = 0.0
        lost_since = None
        expect_held = False
        hold_pos, hold_rot = [], []
        t_wall = time.time()

        while plant.data.time < t_end:
            t = plant.data.time
            tau_wall = t - t_start

            if seq < n_frames and t >= next_frame_t - 1e-9:
                tau = seq * dt
                frame = self.emitter.frame(plan, tau, seq)
                sp, _reason = pump.build(frame, t_origin_us=int(seq * 25000))
                emitted += 1
                if sp is not None:
                    accepted += 1
                    flags_seen.add(int(sp.flags))
                    _latch(mirror, Setpoint.unpack(sp.pack()), t)
                    latch_tau = tau
                seq += 1
                next_frame_t += dt

            st = plant.get_state()
            fb_rev = list(np.asarray(st.leg_extensions_mm) * self.mm_to_rev)
            fb_hand = rev_of_slider_mm(float(st.hand_pos_mm), cfg)
            cmd_pos, _cmd_vel, _ = mirror.tick(t, fb_rev)
            hand_out = mirror.tick_hand(t, fb_hand)
            plant.command(np.asarray(cmd_pos) / self.mm_to_rev)
            if hand_out is not None:
                plant.command_hand(slider_mm_of_rev(hand_out[0], cfg))

            # ── reconstruction, scored two ways ──
            # (a) LADDER fidelity: the mirror against the plan at the mirror's
            #     OWN trajectory phase.  This isolates the interpolator + the
            #     wire from the arrival jitter, and is the band.
            # (b) STREAM phase error: the mirror against the plan at wall time.
            #     Reported: it is the 40 Hz-arrival vs 500 Hz-tick offset times
            #     the channel speed, a real property of the link and not a
            #     defect in anything.
            tau_phase = latch_tau + (t - mirror.base_timestamp)
            if 0.0 <= tau_phase <= plan.total_duration:
                d_leg, d_hand = self._recon(plan, mirror, tau_phase)
                worst_leg = max(worst_leg, d_leg)
                worst_hand = max(worst_hand, d_hand)
            if 0.0 <= tau_wall <= plan.total_duration:
                d_leg, d_hand = self._recon(plan, mirror, tau_wall)
                worst_leg_phase = max(worst_leg_phase, d_leg)
                worst_hand_phase = max(worst_hand_phase, d_hand)

            # ── ball lifecycle: release at each planned release, capture after ──
            if n_rel < len(rel_t) and tau_wall >= rel_t[n_rel]:
                if plant.has_ball and plant.get_ball_state().held:
                    plant.release_ball(rel_v[n_rel])
                expect_held = False
                lost_since = None
                n_rel += 1
            # Polled UNCONDITIONALLY, not behind a ``not held`` guard:
            # ``check_and_capture`` harvests a one-shot latch that ``step()``
            # already set, and by the time it is polled the ball IS held — a
            # guard on ``held`` therefore counts zero makes on a run that
            # caught everything (measured 2026-09-04, before this line).
            if plant.has_ball and plant.check_and_capture():
                makes += 1
                expect_held = True
                bs = plant.get_ball_state()
                site = plant.data.site_xpos[self._site_id] * 1000.0
                d_cap = float(np.linalg.norm(
                    np.asarray(bs.position_mm) - site))
                capture_dist = (d_cap if not np.isfinite(capture_dist)
                                else max(capture_dist, d_cap))
            if n_cat < len(cat_t) and tau_wall >= cat_t[n_cat]:
                st2 = plant.get_state()
                pose_m = np.concatenate([np.asarray(st2.platform_pos_mm),
                                         np.asarray(st2.platform_rot)])
                cup_m = uc.cup_state_from_platform(
                    pose_m, rev_of_slider_mm(float(st2.hand_pos_mm), cfg), cfg)
                err = float(np.linalg.norm(
                    cup_m - np.asarray(meta.catches[n_cat].site_mm)))
                exec_cup_err = (err if not np.isfinite(exec_cup_err)
                                else max(exec_cup_err, err))
                n_cat += 1
            if expect_held and plant.has_ball:
                if not plant.get_ball_state().held:
                    if lost_since is None:
                        lost_since = t
                        drops += 1
                    separation_ms = max(separation_ms, (t - lost_since) * 1000.0)
                else:
                    lost_since = None
            if tau_wall > float(plan.total_duration):
                # Re-read rather than reuse ``st``: ``get_state`` returns the
                # SAME PlantState instance mutated in place (the P1 contract),
                # so a reference taken earlier in this tick is only valid until
                # the next call — and the catch block above makes one.
                sth = plant.get_state()
                hold_pos.append(np.asarray(sth.platform_pos_mm).copy())
                hold_rot.append(np.asarray(sth.platform_rot).copy())

            plant.step(TICK_S)
            ticks += 1
            if self.viewer is not None:
                self.viewer.sync()

        bs = plant.get_ball_state()
        return dict(
            preposition_err_mm=preposition_err_mm,
            emitted=emitted, accepted=accepted,
            rejects=int(pump.frames_rejected),
            flags_seen=tuple(sorted(flags_seen)),
            mirror_leg=worst_leg, mirror_hand=worst_hand,
            phase_leg=worst_leg_phase, phase_hand=worst_hand_phase,
            leg_lead=int(mirror.lead_clamp_ticks),
            leg_stroke=int(mirror.stroke_clamp_ticks),
            hand_lead=int(mirror.hand_lead_clamp_ticks),
            hand_clip=int(mirror.hand_clip_ticks),
            hand_unseen=int(mirror.hand_unseen_skips),
            hand_dev_max=float(mirror.hand_dev_max),
            ticks=ticks, makes=makes, drops=drops,
            capture_dist_mm=capture_dist, exec_cup_err_mm=exec_cup_err,
            separation_ms=separation_ms,
            held_at_end=bool(bs is not None and bs.held),
            hold_travel_mm=travel_mm(hold_pos),
            hold_tilt_deg=tilt_change_deg(hold_rot),
            stream_wall_s=time.time() - t_wall)

    def _recon(self, plan, mirror, tau):
        """``(|Δleg|, |Δhand|)`` in rev between the mirror's RAW ladder and the plan."""
        pose_p, _, _ = plan.state_at(tau)
        rot = rotvec_to_rot_matrix(np.asarray(pose_p[3:6], dtype=float))
        ext = pose_to_leg_lengths(np.asarray(pose_p[:3], dtype=float), rot,
                                  self.geom)
        d_leg = float(np.max(np.abs(np.asarray(mirror.raw_pos)
                                    - ext * self.mm_to_rev)))
        hr, _ = plan.hand_at(tau)
        return d_leg, abs(float(mirror.hand_raw_pos) - float(hr))

    # ── one trial ─────────────────────────────────────────────────────────

    def run_trial(self, idx: int, point: dict) -> UnifiedTrialResult:
        base = dict(idx=idx, point_id=_point_id(point), kind=point['kind'],
                    advisory=bool(point.get('advisory', False)),
                    flight_s=float(point['flight_s']),
                    displacement_mm=(
                        float(point['displacement_mm'])
                        if point['kind'] == 'ring'
                        else float(np.hypot(*point['catch_dxy_mm']))))
        t_plan = time.time()
        try:
            plan, meta = (self.plan_ring(point) if point['kind'] == 'ring'
                          else self.plan_single_toss(point))
        except uc.CycleInfeasible as exc:
            return UnifiedTrialResult(accepted=False, reject_code=exc.outcome(),
                                      plan_wall_s=time.time() - t_plan, **base)
        plan_wall = time.time() - t_plan

        rep = meta.report
        cup = meta.cup_plan
        tilts = np.asarray(meta.tilts, dtype=float)

        # ── plan-domain bands (cycle_gate's) ──
        spans = _carry_spans(meta, int(plan.n_knots))
        seat_deg, seat_n = seat_angles(cup, tilts, spans)
        prime = float(hw.JB_OP_HAND_CATCH_PRIME_REV)
        k_last_catch = spans[-1][0] if spans else int(plan.n_knots) - 1
        pre = np.asarray(plan.hand_rev)[:k_last_catch + 1]
        pre_max = float(np.max(pre)) if pre.size else float('nan')
        margins_m = [runway_margin_m(m.vel_mm_s, float(m.site_mm[2]),
                                     self.cup_cfg) for m in meta.catches]
        margins_rev = [float(m.runway_margin_rev) for m in meta.catches
                       if m.runway_margin_rev is not None]
        # The plan's own capture: the cup track at the interpolated touch-down
        # against the requested site.  The QP pins it by hard equality, so this
        # is a numerical-noise check on the chain that built the goal, and it is
        # scored at the INTERPOLATED instant for the reason cycle_gate.cup_state_at
        # documents (reading cup.catch_k instead measures the cup mid-approach).
        plan_cap = 0.0
        for m in meta.catches:
            k = max(0, min(int(math.floor(float(m.t_s) / float(cup.dt))),
                           cup.jerk.shape[0] - 1))
            d = float(m.t_s) - k * float(cup.dt)
            pos = (cup.pos[k] + cup.vel[k] * d + 0.5 * cup.acc[k] * d ** 2
                   + cup.jerk[k] * d ** 3 / 6.0)
            plan_cap = max(plan_cap, float(np.linalg.norm(
                pos * 1000.0 - np.asarray(m.site_mm))))

        rel_t = sorted(float(m.t_s) for m in meta.releases)
        beat_dev = (float(np.max(np.abs(np.diff(rel_t) - CYCLE_PERIOD_S)))
                    if len(rel_t) > 1 else 0.0)

        res = UnifiedTrialResult(
            accepted=True, reject_code=None,
            n_knots=int(plan.n_knots), duration_s=float(plan.total_duration),
            n_releases=len(meta.releases), n_catches=len(meta.catches),
            latest_supersede_s=float(uc.latest_supersede_time_s(meta)),
            plan_wall_s=plan_wall,
            validate_ok=bool(rep.code == fz.OK), validate_code=str(rep.code),
            peak_leg_vel_mmps=float(rep.peak_leg_vel_mmps),
            peak_leg_acc_mmps2=float(rep.peak_leg_acc_mmps2),
            peak_leg_jerk_mmps3=float(rep.peak_leg_jerk_mmps3),
            peak_hand_vel_rps=float(rep.peak_hand_vel_rps),
            peak_hand_acc_rps2=float(rep.peak_hand_acc_rps2),
            peak_hand_rev=float(rep.peak_hand_rev),
            seat_ok=bool(seat_n > 0 and seat_deg <= SEAT_CONE_DEG),
            max_seat_angle_deg=seat_deg, seat_scored_knots=seat_n,
            no_slam=bool(np.isfinite(pre_max) and pre_max <= prime),
            pre_catch_hand_max_rev=pre_max,
            runway_active=bool(self.cup_cfg.catch_runway_enabled
                               and margins_m
                               and min(margins_m) >= -1e-12),
            worst_runway_margin_m=float(min(margins_m)) if margins_m
            else float('nan'),
            worst_runway_margin_rev=float(min(margins_rev)) if margins_rev
            else float('nan'),
            capture_ok=bool(plan_cap <= CAPTURE_TOL_MM),
            plan_capture_dist_mm=plan_cap,
            max_tilt_deg=float(np.degrees(np.linalg.norm(tilts, axis=1)).max()),
            beat_period_s=CYCLE_PERIOD_S, beat_worst_dev_s=beat_dev,
            beat_exact=bool(beat_dev <= BEAT_TOL_S),
            **base)

        if not self.cfg.plant_column:
            res.core_clean = bool(
                res.validate_ok and res.seat_ok and res.no_slam
                and res.runway_active and res.capture_ok and res.beat_exact)
            return res

        s = self._stream(plan, meta)
        want_flags = FLAG_HAS_U1 | FLAG_HAS_U2 | FLAG_HAS_HAND | FLAG_HAS_V1
        res.pump_frames_emitted = int(s['emitted'])
        res.pump_frames_accepted = int(s['accepted'])
        res.pump_rejects = int(s['rejects'])
        res.flags_seen = s['flags_seen']
        res.mirror_leg_worst_rev = float(s['mirror_leg'])
        res.mirror_hand_worst_rev = float(s['mirror_hand'])
        res.stream_leg_phase_worst_rev = float(s['phase_leg'])
        res.stream_hand_phase_worst_rev = float(s['phase_hand'])
        res.leg_lead_clamp_ticks = int(s['leg_lead'])
        res.leg_stroke_clamp_ticks = int(s['leg_stroke'])
        res.hand_lead_clamp_ticks = int(s['hand_lead'])
        res.hand_clip_ticks = int(s['hand_clip'])
        res.hand_unseen_skips = int(s['hand_unseen'])
        res.hand_dev_max_rev = float(s['hand_dev_max'])
        res.ticks = int(s['ticks'])
        res.makes = int(s['makes'])
        res.drops = int(s['drops'])
        res.capture_dist_mm = float(s['capture_dist_mm'])
        res.exec_cup_err_mm = float(s['exec_cup_err_mm'])
        res.separation_ms = float(s['separation_ms'])
        res.held_at_end = bool(s['held_at_end'])
        res.hold_travel_mm = float(s['hold_travel_mm'])
        res.hold_tilt_deg = float(s['hold_tilt_deg'])
        res.preposition_err_mm = float(s['preposition_err_mm'])
        res.stream_wall_s = float(s['stream_wall_s'])

        res.pump_clean = bool(res.pump_rejects == 0
                              and res.pump_frames_accepted
                              == res.pump_frames_emitted
                              and res.pump_frames_emitted > 0)
        res.mirror_ok = bool(res.mirror_leg_worst_rev <= MIRROR_TOL_LEG_REV
                             and res.mirror_hand_worst_rev
                             <= MIRROR_TOL_HAND_REV)
        res.flags_ok = bool(res.flags_seen == (want_flags,))
        res.caught = bool(res.makes >= len(meta.catches) > 0)
        res.quiescent = bool(res.hold_travel_mm < HOLD_TRAVEL_MM
                             and res.hold_tilt_deg < HOLD_TILT_DEG
                             and res.separation_ms <= SEPARATION_MS)
        res.prepositioned = bool(res.preposition_err_mm <= PREPOSITION_TOL_MM)
        res.core_clean = bool(
            res.caught and res.held_at_end
            and res.validate_ok and res.pump_clean and res.mirror_ok
            and res.flags_ok and res.quiescent and res.prepositioned
            and res.no_slam and res.runway_active and res.seat_ok
            and res.capture_ok and res.beat_exact)
        return res

    # ── run + summarise ───────────────────────────────────────────────────

    def run(self) -> dict:
        points = (default_grid() if self.cfg.points is None
                  else list(self.cfg.points))
        results = [self.run_trial(i, p) for i, p in enumerate(points)]
        decay = None
        if self.cfg.decay_probe:
            for p in points:
                try:
                    plan, _meta = (self.plan_ring(p) if p['kind'] == 'ring'
                                   else self.plan_single_toss(p))
                except uc.CycleInfeasible:
                    continue
                decay = hand_decay_probe(plan, self.geom)
                break
        return self._summarise(points, results, decay)

    def _summarise(self, points, results, decay) -> dict:
        toss = [r for r in results if r.kind == 'toss']
        ring = [r for r in results if r.kind == 'ring']
        toss_binding = [r for r in toss if not r.advisory]
        n_toss = len(toss_binding)
        clean_toss = sum(1 for r in toss_binding if r.core_clean)
        thr_toss = _pass_threshold(n_toss) if n_toss else 0
        # A set with no points is NOT RUN, not failed: ``--set beat`` is a
        # legitimate invocation and a vacuous ``all()`` must not report PASS
        # either, so the run requires at least one set to have run and every
        # set that DID run to have passed.
        toss_ran = bool(n_toss)
        toss_passed = bool(clean_toss >= thr_toss) if toss_ran else None

        ring_binding = [r for r in ring if not r.advisory]
        ring_clean = sum(1 for r in ring_binding if r.core_clean)
        ring_beat = all(r.beat_exact for r in ring_binding)
        ring_ran = bool(ring_binding)
        ring_passed = (bool(ring_clean == len(ring_binding) and ring_beat)
                       if ring_ran else None)

        accepted = [r for r in results if r.accepted]
        # Invariants that must hold on EVERY accepted trial, advisory included:
        # a systemic chain break must not be able to hide behind a 9-of-10 band.
        # With the plant column off (``--no-plant``) NO frame was streamed, so
        # the three chain invariants have nothing to say and are NOT RUN — the
        # same treatment an empty cycle set gets above, and for the same reason:
        # a vacuous ``all()`` must not read as evidence.
        chain = [r for r in accepted if r.pump_frames_emitted > 0]
        streamed = bool(self.cfg.plant_column)
        pump_ok = (bool(chain) and all(r.pump_clean for r in chain)
                   if streamed else None)
        mirror_ok = (bool(chain) and all(r.mirror_ok for r in chain)
                     if streamed else None)
        flags_ok = (bool(chain) and all(r.flags_ok for r in chain)
                    if streamed else None)
        decay_ok = bool(decay and decay.get('ok')
                        and decay.get('age_velocity_zero_s') is not None
                        and decay['age_velocity_zero_s']
                        <= (decay['decay_deadline_s']
                            + decay['sampling_slack_s'] + 1e-9)
                        and decay['monotone_after_mode1']
                        and decay['final_vel_rps'] == 0.0)
        checked = [v for v in (pump_ok, mirror_ok, flags_ok,
                               decay_ok if self.cfg.decay_probe else None)
                   if v is not None]
        invariants_ok = bool(all(checked))
        ran = [v for v in (toss_passed, ring_passed) if v is not None]
        passed = bool(ran and all(ran) and invariants_ok)

        def _worst(rows, attr, best=max):
            vals = [getattr(r, attr) for r in rows
                    if np.isfinite(getattr(r, attr))]
            return float(best(vals)) if vals else float('nan')

        return {
            'gate': 'unified',
            'passed': passed,
            'single_toss': {
                'n': n_toss, 'core_clean': clean_toss,
                'pass_threshold': thr_toss, 'ran': toss_ran,
                'passed': toss_passed,
                'advisory_n': len(toss) - n_toss,
                'advisory_clean': sum(1 for r in toss
                                      if r.advisory and r.core_clean),
            },
            'constant_beat': {
                'n': len(ring_binding), 'core_clean': ring_clean,
                'ran': ring_ran, 'passed': ring_passed,
                'beat_exact': bool(ring_beat),
                'worst_beat_dev_s': _worst(ring_binding, 'beat_worst_dev_s'),
                'beat_tol_s': BEAT_TOL_S,
                'period_s': CYCLE_PERIOD_S,
                'releases': [r.n_releases for r in ring_binding],
                'catches': [r.n_catches for r in ring_binding],
            },
            'invariants': {
                'pump_clean_everywhere': pump_ok,
                'mirror_within_tol_everywhere': mirror_ok,
                'flags_full_everywhere': flags_ok,
                'hand_decay_observed': (decay_ok if self.cfg.decay_probe
                                        else None),
                'all_ok': invariants_ok,
            },
            'trials': len(results),
            'accepted': len(accepted),
            'core_clean': sum(1 for r in results if r.core_clean),
            'rejected': [{'point_id': r.point_id, 'reject_code': r.reject_code,
                          'advisory': r.advisory}
                         for r in results if not r.accepted],
            'worst_mirror_leg_rev': _worst(accepted, 'mirror_leg_worst_rev'),
            'worst_mirror_hand_rev': _worst(accepted, 'mirror_hand_worst_rev'),
            'mirror_tol_leg_rev': MIRROR_TOL_LEG_REV,
            'mirror_tol_hand_rev': MIRROR_TOL_HAND_REV,
            'worst_stream_phase_leg_rev': _worst(
                accepted, 'stream_leg_phase_worst_rev'),
            'worst_stream_phase_hand_rev': _worst(
                accepted, 'stream_hand_phase_worst_rev'),
            'worst_capture_dist_mm': _worst(accepted, 'capture_dist_mm'),
            'worst_exec_cup_err_mm': _worst(accepted, 'exec_cup_err_mm'),
            'worst_plan_capture_dist_mm': _worst(accepted,
                                                 'plan_capture_dist_mm'),
            'worst_hold_travel_mm': _worst(accepted, 'hold_travel_mm'),
            'worst_hold_tilt_deg': _worst(accepted, 'hold_tilt_deg'),
            'worst_separation_ms': _worst(accepted, 'separation_ms'),
            'worst_preposition_err_mm': _worst(accepted, 'preposition_err_mm'),
            'worst_seat_angle_deg': _worst(accepted, 'max_seat_angle_deg'),
            'worst_hand_dev_max_rev': _worst(accepted, 'hand_dev_max_rev'),
            'worst_leg_jerk_mmps3': _worst(accepted, 'peak_leg_jerk_mmps3'),
            'worst_hand_acc_rps2': _worst(accepted, 'peak_hand_acc_rps2'),
            # Split per lane: the two counters do NOT mean the same thing.  The
            # leg half is motor_guard's clamp, the hand half is the firmware's —
            # see ``leg_clamp_note``.  Summing them hid that.
            'total_leg_lead_clamp_ticks': sum(r.leg_lead_clamp_ticks
                                              for r in accepted),
            'total_hand_lead_clamp_ticks': sum(r.hand_lead_clamp_ticks
                                               for r in accepted),
            'total_hand_unseen_skips': sum(r.hand_unseen_skips
                                           for r in accepted),
            'hand_decay': decay,
            'contact_note': _CONTACT_NOTE,
            'leg_clamp_note': _LEG_CLAMP_NOTE,
            'advisory_note': _ADVISORY_NOTE,
            'dropped_bands_note': _DROPPED_BANDS_NOTE,
            'thresholds': {
                'mirror_tol_leg_rev': MIRROR_TOL_LEG_REV,
                'mirror_tol_hand_rev': MIRROR_TOL_HAND_REV,
                'beat_tol_s': BEAT_TOL_S,
                'capture_tol_mm': CAPTURE_TOL_MM,
                'hold_travel_mm': HOLD_TRAVEL_MM,
                'hold_tilt_deg': HOLD_TILT_DEG,
                'separation_ms': SEPARATION_MS,
                'preposition_tol_mm': PREPOSITION_TOL_MM,
                'seat_cone_deg': SEAT_CONE_DEG,
                'hand_catch_prime_rev': float(hw.JB_OP_HAND_CATCH_PRIME_REV),
                'session_leg_vel_mmps': self.cfg.leg_vel_mmps,
                'session_leg_acc_mmps2': self.cfg.leg_acc_mmps2,
                'session_leg_jerk_mmps3': self.cfg.leg_jerk_mmps3,
                'tick_s': TICK_S,
                'launch_period_s': LAUNCH_PERIOD_S,
                'cycle_period_s': CYCLE_PERIOD_S,
            },
            'config': dataclasses.asdict(self.cfg),
            'results': [r.to_dict() for r in results],
        }


# ---------------------------------------------------------------------------
# Entry points
# ---------------------------------------------------------------------------

def run_gate(cfg: UnifiedGateConfig = None, viewer_speed: float = None) -> dict:
    """Run the gate, write the JSON report, return it."""
    cfg = UnifiedGateConfig() if cfg is None else cfg
    gate = UnifiedGate(cfg)
    t0 = time.time()
    try:
        if viewer_speed is not None:
            attach_viewer(gate, viewer_speed, tag='unified_gate')
        report = gate.run()
    except ViewerClosed:
        print('[unified_gate] viewer closed by the operator — no report written.')
        raise SystemExit(130)
    finally:
        if gate.viewer is not None:
            gate.viewer.close()
    report['wall_s'] = time.time() - t0

    path = cfg.report_path
    if path is None:
        out_dir = os.path.join(_repo_root, 'temp', 'reports')
        os.makedirs(out_dir, exist_ok=True)
        path = os.path.join(out_dir, 'unified_gate_seed%d.json' % cfg.seed)
    with open(path, 'w') as fh:
        json.dump(report, fh, indent=2)
    report['report_path'] = path
    return report


def _verdict(flag) -> str:
    return 'NOT RUN' if flag is None else ('PASS' if flag else 'FAIL')


def _print_table(rep: dict) -> None:
    print('[unified_gate] %-28s %-9s %8s %8s %10s %10s %9s'
          % ('point', 'verdict', 'mir_leg', 'mir_hand', 'capture', 'exec_cup',
             'hold_mm'))
    for r in rep['results']:
        if not r['accepted']:
            print('[unified_gate] %-28s %-9s  REJECTED %s'
                  % (r['point_id'], 'ADVISORY' if r['advisory'] else 'FAIL',
                     r['reject_code']))
            continue
        verdict = ('ADVISORY' if r['advisory']
                   else ('clean' if r['core_clean'] else 'DIRTY'))
        print('[unified_gate] %-28s %-9s %8.2e %8.2e %9.2f %9.2f %9.4f'
              % (r['point_id'], verdict, r['mirror_leg_worst_rev'],
                 r['mirror_hand_worst_rev'], r['capture_dist_mm'],
                 r['exec_cup_err_mm'], r['hold_travel_mm']))
    st = rep['single_toss']
    cb = rep['constant_beat']
    print('[unified_gate] SET 1 single-toss : core_clean %d/%d  threshold %d  '
          '=> %s   (advisory %d, clean %d)'
          % (st['core_clean'], st['n'], st['pass_threshold'],
             _verdict(st['passed']), st['advisory_n'], st['advisory_clean']))
    print('[unified_gate] SET 2 two-pose beat: core_clean %d/%d  beat %s '
          '(worst dev %.3e s, tol %.0e)  => %s'
          % (cb['core_clean'], cb['n'], 'EXACT' if cb['beat_exact'] else 'OFF',
             cb['worst_beat_dev_s'], cb['beat_tol_s'],
             _verdict(cb['passed'])))
    inv = rep['invariants']
    def _inv(flag):
        return 'NOT RUN' if flag is None else ('OK' if flag else 'BROKEN')
    print('[unified_gate] invariants: pump %s  mirror %s  flags %s  decay %s'
          % (_inv(inv['pump_clean_everywhere']),
             _inv(inv['mirror_within_tol_everywhere']),
             _inv(inv['flags_full_everywhere']),
             _inv(inv['hand_decay_observed'])))
    d = rep.get('hand_decay') or {}
    if d.get('ok'):
        print('[unified_gate] hand falling edge: cut at %.1f rev/s, leaves '
              'Mode 1 at %.3f s, velocity EXACTLY 0 at %.3f s (deadline %.3f '
              '+ %.3f sampling), monotone %s, wind-down travel %.4f rev raw / '
              '%.4f rev after the %.1f rev lead clamp (%d clamped ticks)'
              % (d['v_at_cut_rps'], d['age_left_mode1_s'],
                 d['age_velocity_zero_s'], d['decay_deadline_s'],
                 d['sampling_slack_s'], d['monotone_after_mode1'],
                 d['travel_after_cut_rev'],
                 d['clamped_travel_after_cut_rev'], d['max_lead_hand_rev'],
                 d['lead_clamp_ticks']))
    print('[unified_gate] worst: mirror leg %.3e / hand %.3e rev (leg tol %.0e) | '
          'stream phase leg %.3e / hand %.3e rev'
          % (rep['worst_mirror_leg_rev'], rep['worst_mirror_hand_rev'],
             rep['mirror_tol_leg_rev'], rep['worst_stream_phase_leg_rev'],
             rep['worst_stream_phase_hand_rev']))
    print('[unified_gate] worst: capture %.2f mm  exec-cup %.2f mm  plan-cap '
          '%.2e mm  hold %.4f mm / %.5f deg  sep %.1f ms  seat %.2f deg  '
          'hand dev %.3f rev'
          % (rep['worst_capture_dist_mm'], rep['worst_exec_cup_err_mm'],
             rep['worst_plan_capture_dist_mm'], rep['worst_hold_travel_mm'],
             rep['worst_hold_tilt_deg'], rep['worst_separation_ms'],
             rep['worst_seat_angle_deg'], rep['worst_hand_dev_max_rev']))
    print('[unified_gate] clamps: %d LEG lead ticks (motor_guard 0.15 rev, not '
          'FW 0.10), %d HAND lead ticks (FW 2.0 rev), %d hand unseen skips  |  '
          '%s  (wall %.1f s) -> %s'
          % (rep['total_leg_lead_clamp_ticks'],
             rep['total_hand_lead_clamp_ticks'],
             rep['total_hand_unseen_skips'],
             'PASS' if rep['passed'] else 'FAIL', rep['wall_s'],
             rep.get('report_path')))


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--set', choices=('all', 'toss', 'beat'), default='all',
                   help='which cycle set(s) to run')
    p.add_argument('--no-plant', action='store_true',
                   help='plan + score only; skip the MuJoCo/chain column')
    p.add_argument('--no-decay-probe', action='store_true')
    p.add_argument('--viewer', action='store_true')
    p.add_argument('--no-viewer', action='store_true',
                   help='headless (the default; accepted for symmetry)')
    p.add_argument('--viewer-speed', type=float, default=1.0)
    p.add_argument('--report', default=None)
    args = p.parse_args(argv)

    points = None
    if args.set == 'toss':
        points = single_toss_grid()
    elif args.set == 'beat':
        points = beat_grid()

    cfg = UnifiedGateConfig(points=points, seed=args.seed,
                            plant_column=not args.no_plant,
                            decay_probe=not args.no_decay_probe,
                            report_path=args.report)
    rep = run_gate(cfg, viewer_speed=(args.viewer_speed if args.viewer
                                      else None))
    _print_table(rep)
    return 0 if rep['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
