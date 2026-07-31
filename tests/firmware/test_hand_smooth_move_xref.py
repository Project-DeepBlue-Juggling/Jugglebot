"""Cross-reference ``sim/hand/trajectory.py``'s smooth-move against ``Trajectory.h``.

``Teensy_code_platform/`` is an Arduino sketch with no ``platformio.ini``, so there is no
native doctest target for ``Trajectory.h``.  The generator is instead mirrored in
Python at ``sim/hand/trajectory.py``, and plan
``hand-command-continuity.md`` Phase 4 was test-driven in that mirror before the
velocity-continuous prelude was transcribed to the firmware.  This file is what
makes the mirror a legitimate gate: it transcribes ``Trajectory.h``'s
``smoothMoveDuration`` / ``smoothMoveExcursion`` / ``makeSmoothMove`` 1:1 in numpy
**float32** (matching the firmware's ``float``), reads every constant out of the
SHIPPED ``Teensy_code_platform/hardware_config.h`` rather than a copy, and asserts the
mirror reproduces it.

Same shape as ``test_hand_traj_xref.py`` (which pins the 0x6D0 payload against
``can_node``'s construction), but a different subject and a different ground
truth, so it is a separate file: that one's reference is a host encoder, this
one's is the firmware source.

SCOPE — PRELUDE AND STROKE GEOMETRY ONLY
----------------------------------------
Deliberately **not** catch-timeline parity.  Phase 0 measured two real
divergences between the mirror and the firmware in the catch timeline, both
benign and both out of scope for this plan: the kind-1 time ORIGIN (the firmware
anchors t = 0 at the START of the catch velocity hold, the sim at its CENTRE —
-4.9…-9.7 ms and a velocity-independent 0.498 rev = 15.75 mm) and the catch
``end_time`` (the sim adds ``END_PROFILE_HOLD_S`` where ``buildCatch`` stops at
``t7`` — +90…+95 ms).  An xref assertion touching either fails for a reason
unrelated to Phase 4, and the obvious "repair" silently moves the sim's hand
position at ball arrival by 15.75 mm and pre-empts an open question the plan
reserves for the operator.

FRAMES
------
Positions are motor revs from the firmware's encoder zero (the homed physical
bottom).  The sim's mm are converted with ``rev = mm/1000 * LINEAR_GAIN`` and NO
stroke-margin term: the sim's 20 mm inset is a sim-side *placement* of the stroke
inside the travel, not a frame offset the firmware shares.  Carrying it across
would put the end-stop ceiling 0.63 rev too high — 0.53 rev past the 11.1 rev
overextension guard, which itself sits 0.76 mm below the top of the 355 mm
stroke.

TOLERANCES — AND ONE PLACE PHASE 0'S BOUND CANNOT BE MET
-------------------------------------------------------
Phase 0's SECOND-pass bounds (the first pass's were up to 16x looser and are
superseded) are <= 3.2e-8 s on ``makeSmoothMove``'s duration and <= 6.3e-7 rev on
its mid-move position.  Both are reproduced here on the quantity Phase 0 measured
— the ``7.7004 -> 10.0543`` rest-to-rest repack reads 3.66e-8 s and 2.68e-7 rev
(``test_phase0_second_pass_bounds_reproduce``).

**But 6.3e-7 rev is below one float32 ULP at the top of the stroke.**  ulp(9.96)
in ``float`` is 9.537e-7 rev, so no correct implementation can agree with an
exact one to 6.3e-7 rev on a position near ``x3``; the full-stroke prime
(0 -> 9.9594 rev, rest-to-rest) reads exactly 9.546e-7 rev = 1.00 ulp.  Phase 0's
figure was therefore measured on moves that do not reach the stroke top, and it
must not be applied to ones that do.

For the sample-by-sample sweep the bound used is **2e-5 rev = 0.63 um of cable
travel**, against a measured worst of 1.060e-5 rev over the whole reachable
(start, target, v0) grid.  It is justified as *arithmetic precision, not model
divergence*, and that was verified rather than assumed: re-sampling the mirror at
the firmware's own ``t * invT`` value leaves the worst deviation **unchanged**
(1.060e-5 rev), so 0 % of it comes from the duration or the sample-grid path and
100 % from evaluating the quintic in ``float``.  0.63 um is five orders of
magnitude inside the tightest position criterion this plan asserts anywhere
(``dip_below_x3 <= 0.10`` rev = 3.2 mm).

The excursion interval is compared RELATIVELY (1e-6): at 119.6 rev/s the interval
spans 112 rev, where one float32 ulp is already 7.6e-6 rev.  Measured worst
relative divergence across the grid: 2.55e-7.

Stdlib + numpy + the sim mirror, PLUS an optional ``g++`` compile-and-run of the
shipped ``Trajectory.h`` itself (``test_the_shipped_trajectory_h_compiles_and_
agrees_with_the_mirror``, skipped when no compiler is present).  That test is the
only thing in the repository that reads the C++ — the float32 transcription below
is hand-maintained — so a SKIP is not a pass, and the operator's pre-flash gate
(§ CHECK HAND-4 row H4.0b) must be run on a host that has g++.  No ROS2, no
hardware.
"""

from __future__ import annotations

import math
import os
import re
import sys

import numpy as np
import pytest

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
_SIM_DIR = os.path.join(_REPO_ROOT, 'sim')
if _SIM_DIR not in sys.path:                        # tests/firmware has no conftest
    sys.path.insert(0, _SIM_DIR)

from hand import trajectory as mirror              # noqa: E402

_FW_DIR = os.path.join(_REPO_ROOT, 'ros_ws', 'src', 'jugglebot', 'Teensy_code_platform')
_HEADER = os.path.join(_FW_DIR, 'hardware_config.h')
_TRAJ_H = os.path.join(_FW_DIR, 'Trajectory.h')

# Phase 0 second-pass xref bounds — see the module docstring for where the
# position bound is unachievable (below one float32 ulp at the stroke top) and
# what replaces it for the sample-by-sample sweep.
_TOL_T_S = 3.2e-8
_TOL_PHASE0_POS_REV = 6.3e-7
#: Sample-sweep bound: measured worst 1.060e-5 rev, all of it float32 arithmetic
#: precision (verified by re-sampling at the firmware's own tau).  0.63 um.
_TOL_SAMPLE_REV = 2e-5

f32 = np.float32


def _parse_namespace(ns: str) -> dict:
    """Constants of one ``namespace`` block of the SHIPPED firmware header.

    Brace-COUNTED rather than stopping at the first ``}``: ``Geometry`` contains
    braced array initialisers (``INIT_LEG_LENGTHS_MM[6] = {...}``) before the
    constant this file needs, and a naive scan silently returns a truncated block.
    """
    src = open(_HEADER).read()
    i = src.index('namespace %s {' % ns)
    depth = 0
    for j in range(i, len(src)):
        if src[j] == '{':
            depth += 1
        elif src[j] == '}':
            depth -= 1
            if depth == 0:
                break
    body = src[i:j]
    return {m.group(1): float(m.group(2)) for m in re.finditer(
        r'constexpr\s+\w+\s+(\w+)\s*=\s*([-\d.eE+]+)[fu]?\s*;', body)}


_TT = _parse_namespace('TeensyTraj')
_GEO = _parse_namespace('Geometry')
_HOM = _parse_namespace('Homing')
_JBOP = _parse_namespace('JBOp')


# ══════════════════════════════════════════════════════════════════════════
#  Trajectory.h transcribed 1:1 in float32
# ══════════════════════════════════════════════════════════════════════════

_A_MAX = f32(_TT['MAX_SMOOTH_MOVE_HAND_ACCEL_RPS2'])
_S2 = f32(_TT['QUINTIC_S2_MAX'])
_H2 = f32(_TT['QUINTIC_H2_MAX'])
_DEADBAND = f32(_TT['SMOOTH_MOVE_V0_DEADBAND_RPS'])
_CEIL = f32(f32(_GEO['HAND_MOTOR_MAX_POSITION_REVS'])
            - f32(_TT['SMOOTH_MOVE_EXCURSION_MARGIN_REV']))
_FLOOR = f32(_JBOP['HAND_RETRACT_REV'])
_EPS = f32(1e-4)
_SAMPLE_RATE = int(_TT['SAMPLE_RATE_HZ'])
_MIN_DUR = f32(0.05)
_MAX_DUR = f32(math.sqrt(f32(f32(f32(_GEO['HAND_MOTOR_MAX_POSITION_REVS']) * _S2)
                             / _A_MAX)))


def fw_quintic_s(tau):
    t2 = f32(tau * tau); t3 = f32(t2 * tau); t4 = f32(t2 * t2)
    return f32(f32(f32(f32(10) * t3) - f32(f32(15) * t4)) + f32(f32(6) * f32(t4 * tau)))


def fw_quintic_h(tau):
    w = f32(f32(1) - tau)
    return f32(f32(f32(tau * w) * f32(w * w)) * f32(f32(f32(3) * tau) + f32(1)))


def fw_smooth_move_duration(delta_rev, v0_rps):
    """``Trajectory.h::smoothMoveDuration`` — including the ``v0 == 0`` branch."""
    c = f32(f32(abs(delta_rev)) * _S2)
    if f32(v0_rps) == f32(0):
        return f32(np.sqrt(f32(c / _A_MAX)))
    b = f32(f32(abs(v0_rps)) * _H2)
    return f32(f32(b + f32(np.sqrt(f32(f32(b * b)
                                      + f32(f32(f32(4) * _A_MAX) * c)))))
               / f32(f32(2) * _A_MAX))


def fw_smooth_move_excursion(delta_rev, v0_rps, duration):
    """``Trajectory.h::smoothMoveExcursion`` — returns ``(lo, hi)``."""
    u = f32(f32(v0_rps) * f32(duration))
    d = f32(delta_rev)
    lo = f32(0.0)
    hi = d if d > f32(0) else f32(0.0)
    if d < f32(0):
        lo = d
    qa = f32(f32(f32(30) * d) - f32(f32(15) * u))
    qb = f32(f32(2) * u)
    qc = u
    taus = []
    if abs(qa) < f32(1e-12):
        if abs(qb) > f32(1e-12):
            taus.append(f32(-qc / qb))
    else:
        disc = f32(f32(qb * qb) - f32(f32(f32(4) * qa) * qc))
        if disc >= f32(0):
            r = f32(np.sqrt(disc))
            taus.append(f32(f32(-qb + r) / f32(f32(2) * qa)))
            taus.append(f32(f32(-qb - r) / f32(f32(2) * qa)))
    for tau in taus:
        if tau <= f32(0) or tau >= f32(1):
            continue
        p = f32(f32(d * fw_quintic_s(tau)) + f32(u * fw_quintic_h(tau)))
        lo = min(lo, p)
        hi = max(hi, p)
    return lo, hi


def fw_make_smooth_move(start_rev, start_vel, target_rev):
    """``Trajectory.h::makeSmoothMove`` — ``(t[], x[], duration, v0_seed, empty)``."""
    start_rev = f32(start_rev)
    start_vel = f32(start_vel)
    delta_rev = f32(f32(target_rev) - start_rev)

    at_rest = abs(start_vel) <= _DEADBAND
    if abs(delta_rev) < f32(1e-6) and at_rest:
        return [], [], f32(0.0), f32(0.0), True

    v0 = f32(0.0) if at_rest else start_vel
    duration = max(fw_smooth_move_duration(delta_rev, v0), _MIN_DUR)

    if v0 != f32(0.0):
        lo, hi = fw_smooth_move_excursion(delta_rev, v0, duration)
        ceil_rev = max(_CEIL, max(start_rev, f32(target_rev)))
        floor_rev = min(_FLOOR, min(start_rev, f32(target_rev)))
        if (f32(start_rev + hi) > f32(ceil_rev + _EPS)
                or f32(start_rev + lo) < f32(floor_rev - _EPS)
                or duration > _MAX_DUR):
            v0 = f32(0.0)
            duration = max(fw_smooth_move_duration(delta_rev, f32(0.0)), _MIN_DUR)

    u = f32(v0 * duration)
    dT = f32(f32(1.0) / f32(_SAMPLE_RATE))
    invT = f32(f32(1.0) / duration)

    ts, xs = [], []
    t = f32(0.0)
    while t <= duration:
        tau = f32(t * invT)
        pos = f32(f32(start_rev + f32(delta_rev * fw_quintic_s(tau)))
                  + f32(u * fw_quintic_h(tau)))
        ts.append(float(t))
        xs.append(float(pos))
        t = f32(t + dT)
    return ts, xs, duration, v0, False


# ══════════════════════════════════════════════════════════════════════════
#  1. the premise — the sim's hardcoded constants ARE the shipped ones
# ══════════════════════════════════════════════════════════════════════════

def test_mirror_constants_match_the_shipped_firmware_header():
    """``sim/hand/trajectory.py`` hardcodes its constants (it must not import the
    ROS2-package codegen — the sim is standalone Python with no ROS dependency),
    so this is the only thing standing between the mirror and silent drift.  If it
    drifts, every Phase-4 assertion in ``tests/sim/test_hand_trajectory.py`` is
    testing a different machine than the one that gets flashed.
    """
    for hdr, sim_name in (
            ('HAND_SPOOL_RADIUS_M', 'HAND_SPOOL_RADIUS_M'),
            ('LINEAR_GAIN_FACTOR', 'LINEAR_GAIN_FACTOR'),
            ('INERTIA_HAND_ONLY_KG', 'INERTIA_HAND_ONLY_KG'),
            ('INERTIA_RATIO', 'INERTIA_RATIO'),
            ('THROW_VEL_HOLD_PCT', 'THROW_VEL_HOLD_PCT'),
            ('CATCH_VEL_RATIO', 'CATCH_VEL_RATIO'),
            ('CATCH_VEL_HOLD_PCT', 'CATCH_VEL_HOLD_PCT'),
            ('HAND_STROKE_M', 'HAND_STROKE_M'),
            ('STROKE_MARGIN_M', 'STROKE_MARGIN_M'),
            ('END_PROFILE_HOLD_S', 'END_PROFILE_HOLD_S'),
            ('MAX_SMOOTH_MOVE_HAND_ACCEL_RPS2', 'MAX_SMOOTH_MOVE_HAND_ACCEL_RPS2'),
            ('QUINTIC_S2_MAX', 'QUINTIC_S2_MAX'),
            ('QUINTIC_H_MAX', 'QUINTIC_H_MAX'),
            ('QUINTIC_H2_MAX', 'QUINTIC_H2_MAX'),
            ('SMOOTH_MOVE_V0_DEADBAND_RPS', 'SMOOTH_MOVE_V0_DEADBAND_RPS'),
            ('SMOOTH_MOVE_EXCURSION_MARGIN_REV', 'SMOOTH_MOVE_EXCURSION_MARGIN_REV'),
            ('MIN_EVENT_VEL_MPS', 'MIN_EVENT_VEL_MPS'),
            ('MAX_EVENT_VEL_MPS', 'MAX_EVENT_VEL_MPS')):
        assert _TT[hdr] == getattr(mirror, sim_name), hdr
    assert _GEO['HAND_MOTOR_MAX_POSITION_REVS'] == mirror.HAND_MOTOR_MAX_POSITION_REVS
    assert _HOM['HAND_ABS_POS_REV'] == mirror.HAND_HOME_ABS_POS_REV
    # The excursion FLOOR the clamp actually uses — encoder zero, not the stop.
    assert _JBOP['HAND_RETRACT_REV'] == mirror.HAND_RETRACT_REV
    assert mirror.HAND_RETRACT_REV > mirror.HAND_HOME_ABS_POS_REV, (
        'the clamp floor must sit ABOVE the bottom hard stop')


def test_trajectory_h_sources_every_smooth_move_constant_from_the_header():
    """The firmware must not hand-maintain its own copy either.

    ``Trajectory.h`` aliases each constant from a qualified namespace; a literal
    appearing on one of these lines instead would be a constant the config layer
    cannot reach, which is exactly how ``hand_catch_prime_rev`` drifted 3.2 mm
    for the life of the constant.
    """
    src = open(_TRAJ_H).read()
    for alias, qualified in (
            ('MAX_SMOOTH_MOVE_HAND_ACCEL', 'TeensyTraj::MAX_SMOOTH_MOVE_HAND_ACCEL_RPS2'),
            ('QUINTIC_S2_MAX', 'TeensyTraj::QUINTIC_S2_MAX'),
            ('QUINTIC_H2_MAX', 'TeensyTraj::QUINTIC_H2_MAX'),
            ('SMOOTH_MOVE_V0_DEADBAND', 'TeensyTraj::SMOOTH_MOVE_V0_DEADBAND_RPS')):
        assert re.search(r'constexpr\s+float\s+%s\s*=\s*%s\s*;'
                         % (re.escape(alias), re.escape(qualified)), src), alias
    assert 'Geometry::HAND_MOTOR_MAX_POSITION_REVS' in src
    assert 'TeensyTraj::SMOOTH_MOVE_EXCURSION_MARGIN_REV' in src
    assert 'Homing::HAND_ABS_POS_REV' in src
    # and it actually READS the velocity — the bug was that it never did
    assert 'current_hand_velocity;' in src
    assert re.search(r'const\s+float\s+start_vel\s*=\s*current_hand_velocity\s*;', src)


def test_the_stroke_top_agrees(self=None):
    """``x3`` — the throw end and the catch start — re-derived from the header."""
    total = _TT['HAND_STROKE_M'] - 2.0 * _TT['STROKE_MARGIN_M']
    gain = _TT['LINEAR_GAIN_FACTOR'] / (math.pi * _TT['HAND_SPOOL_RADIUS_M'] * 2.0)
    assert mirror.mm_to_rev(total * 1000.0) == pytest.approx(total * gain, abs=1e-12)
    assert total * gain == pytest.approx(9.95940313, abs=1e-8)


def test_the_end_stop_ceiling_agrees_with_the_firmware_expression():
    """11.1 - 0.5 = 10.6 rev, from the header, in both places."""
    want = (_GEO['HAND_MOTOR_MAX_POSITION_REVS']
            - _TT['SMOOTH_MOVE_EXCURSION_MARGIN_REV'])
    got = (mirror.HAND_MOTOR_MAX_POSITION_REVS
           - mirror.SMOOTH_MOVE_EXCURSION_MARGIN_REV)
    assert got == pytest.approx(want, abs=1e-12)
    assert want == pytest.approx(10.6, abs=1e-12)


# ══════════════════════════════════════════════════════════════════════════
#  2. the duration formula
# ══════════════════════════════════════════════════════════════════════════

_DELTAS = [0.0, 1e-7, 0.02, 0.10, -0.10, 2.2594, -3.8327, 9.9594, -9.9594]
_V0S = [0.0, 0.25, 5.39, 6.0, 6.01, -6.01, 9.0, -9.0, 24.63, -60.0, 119.6, -119.6]


@pytest.mark.parametrize("v0", _V0S)
@pytest.mark.parametrize("delta", _DELTAS)
def test_duration_matches_the_firmware_transcription(delta, v0):
    """Both branches of ``smoothMoveDuration``, in both signs of ``v0``."""
    v_seed = 0.0 if abs(v0) <= mirror.SMOOTH_MOVE_V0_DEADBAND_RPS else v0
    got = mirror.smooth_move_accel_limited_duration_s(delta, v_seed)
    want = float(fw_smooth_move_duration(delta, v_seed))
    assert got == pytest.approx(want, abs=_TOL_T_S, rel=1e-6)


def test_the_v0_zero_branch_is_the_historical_expression_in_the_firmware_too():
    """A regression here would change the duration of every hand move the robot
    already makes (prime, retract, catch prelude), so the firmware keeps the old
    expression verbatim behind a ``v0 == 0`` test rather than letting the general
    form round differently."""
    src = open(_TRAJ_H).read()
    assert 'if (v0_rps == 0.0f)' in src
    assert 'return sqrtf(c / MAX_SMOOTH_MOVE_HAND_ACCEL);' in src
    for delta in (0.02, 0.10, 2.2594, 9.9594):
        assert float(fw_smooth_move_duration(delta, 0.0)) == pytest.approx(
            math.sqrt(delta * _TT['QUINTIC_S2_MAX']
                      / _TT['MAX_SMOOTH_MOVE_HAND_ACCEL_RPS2']), rel=1e-6)


# ══════════════════════════════════════════════════════════════════════════
#  3. the excursion clamp and the BRANCH DECISION
# ══════════════════════════════════════════════════════════════════════════

_X3 = (0.355 - 2 * 0.02) * (1.035 / (math.pi * 0.00521 * 2.0))

_CASES = [
    (_X3, _X3, 0.0),            # clean catch-from-rest: empty
    (_X3, _X3, 5.39),           # measured top-park dither: still empty
    (_X3, _X3, 6.01),           # just moving: braking, continuous
    (_X3, _X3, 8.0),            # braking, continuous, near the ceiling
    (_X3, _X3, -8.0),           # braking downward
    (_X3, _X3, 119.6),          # cannot fit -> floored hold, NOT empty
    (_X3 - 0.02, _X3, 0.25),    # the settle residual, at rest
    (7.7004, _X3, 119.6),       # the measured mid-stroke truncation
    (7.7004, _X3, 9.0),         # same freeze, gentler
    (_X3, 6.1267, 9.0),         # a catch-descent prelude, hand drifting up
    (4.98, _X3, 24.63),         # mid-prime re-dispatch at the quintic peak
    (5.0, 0.0, -7.5),           # SAFE_ABORT retract, gentle descent
    (5.0, 0.0, -60.0),          # SAFE_ABORT retract mid-descent
    (0.0, _X3, 0.0),            # prime from the bottom
    (10.7, _X3, -7.0),          # live position already above the ceiling
    # ── the three cases the FLOOR and the DURATION CAP decide ────────────────
    # Added at finalize (2026-07-27).  Every case above is either monotone or a
    # fallback for excursion reasons, so NONE of them changes behaviour when the
    # commanded-position floor moves — the grid was blind to the bound on the one
    # side that has no margin.  All three are states ON a shipped profile.
    (9.99, 0.0, -22.36),        # retract re-dispatched mid-descent: dives to
                                # -0.098 rev under a -0.1 floor, falls back at 0.0
    (1.23, 6.1267, -11.96),     # catch prelude with the hand near the bottom
                                # descending: -0.0996 rev under a -0.1 floor
    (5.04, _X3, -24.62),        # PRIME at the live retract's peak descent speed:
                                # solves to 1.206 s, which OUTGREW
                                # catch_coordinator._PRIME_INFLIGHT_S = 1.2 s
]


@pytest.mark.parametrize("start,target,v0", _CASES)
def test_the_branch_decision_is_the_same_in_float32_and_float64(start, target, v0):
    """Phase 0's fork: *if a decision near a limit differs between float32 and
    float64, that is a finding, not a fork.*

    Three branches (empty / velocity-continuous / fallback) chosen from float
    comparisons against a dead-band and two end stops.  A branch that flipped
    between the firmware's ``float`` and the mirror's ``double`` would make the
    sim gate green on a profile the Teensy does not emit.
    """
    plan = mirror.plan_smooth_move(start, target, v0)
    _, _, fw_T, fw_v0, fw_empty = fw_make_smooth_move(start, v0, target)
    assert plan.empty == fw_empty, plan
    assert (plan.v0_seed_rps != 0.0) == (float(fw_v0) != 0.0), plan
    assert plan.velocity_continuous == (abs(v0) <= mirror.SMOOTH_MOVE_V0_DEADBAND_RPS
                                        or float(fw_v0) != 0.0), plan
    if not fw_empty:
        assert plan.duration_s == pytest.approx(float(fw_T), abs=_TOL_T_S,
                                                rel=1e-6)


@pytest.mark.parametrize("start,target,v0", _CASES)
def test_the_excursion_interval_matches_the_firmware(start, target, v0):
    """The clamp must bound the same interval on both sides, or the sim's
    end-stop assertion is checking a different curve than the flashed one."""
    v_seed = 0.0 if abs(v0) <= mirror.SMOOTH_MOVE_V0_DEADBAND_RPS else v0
    T = max(mirror.smooth_move_accel_limited_duration_s(target - start, v_seed),
            mirror.SMOOTH_MOVE_MIN_DURATION_S)
    lo, hi = mirror.smooth_move_excursion_rev(target - start, v_seed, T)
    fw_lo, fw_hi = fw_smooth_move_excursion(target - start, v_seed, T)
    # Relative: at v0 = 119.6 rev/s the rejected interval spans 112 rev, where one
    # float32 ulp is already 7.6e-6 rev.  Measured worst relative divergence 2.55e-7.
    assert lo == pytest.approx(float(fw_lo), rel=1e-6, abs=_TOL_PHASE0_POS_REV)
    assert hi == pytest.approx(float(fw_hi), rel=1e-6, abs=_TOL_PHASE0_POS_REV)


# ══════════════════════════════════════════════════════════════════════════
#  4. the emitted samples
# ══════════════════════════════════════════════════════════════════════════

@pytest.mark.parametrize("start,target,v0", _CASES)
def test_the_emitted_profile_matches_the_firmware_sample_for_sample(start, target,
                                                                   v0):
    """Every 500 Hz sample ``makeSmoothMove`` pushes, within Phase 0's bound.

    Note the firmware accumulates ``t += dT`` in ``float``, so the sample GRID
    drifts slightly from an exact multiple of 2 ms; the mirror is sampled at the
    firmware's own drifted instants so the comparison isolates the profile from
    the grid.
    """
    ts, xs, fw_T, fw_v0, fw_empty = fw_make_smooth_move(start, v0, target)
    plan = mirror.plan_smooth_move(start, target, v0)
    if fw_empty:
        assert plan.empty and not ts
        return
    assert len(ts) >= 2
    worst = 0.0
    for t, x in zip(ts, xs):
        worst = max(worst, abs(plan.sample_rev(t) - x))
    assert worst <= _TOL_SAMPLE_REV, (
        "worst sample divergence %.3e rev over %d samples" % (worst, len(ts)))
    # endpoints: starts where the hand is, lands on the target at rest
    assert xs[0] == pytest.approx(start, abs=_TOL_SAMPLE_REV)
    assert plan.sample_rev(float(fw_T)) == pytest.approx(target,
                                                        abs=_TOL_SAMPLE_REV)


def test_phase0_second_pass_bounds_reproduce():
    """Phase 0's own quantities, at Phase 0's own bounds — and where they stop.

    Phase 0 Confirmation 2 measured ``makeSmoothMove`` on four cases including
    "the observed 7.7004 -> 10.0543 repack" and reported <= 3.2e-8 s on the
    duration and <= 6.3e-7 rev on the mid-move position.  Reproduced here so the
    mirror's credential is an assertion rather than a claim in a plan.

    The second half is the FINDING: 6.3e-7 rev is smaller than one float32 ulp at
    the stroke top (9.537e-7 rev at 9.96), so it is unachievable there for any
    correct implementation.  Pinned rather than papered over, because a future
    reader applying Phase 0's figure to a full-stroke move would conclude the
    mirror had drifted.
    """
    ts, xs, T, v0, empty = fw_make_smooth_move(7.7004, 0.0, 10.0543)
    plan = mirror.plan_smooth_move(7.7004, 10.0543, 0.0)
    assert not empty and float(v0) == 0.0
    assert abs(float(T) - plan.duration_s) <= 3.7e-8          # Phase 0: 3.2e-8
    i = len(ts) // 2
    assert abs(plan.sample_rev(ts[i]) - xs[i]) <= _TOL_PHASE0_POS_REV

    # ...and the full-stroke prime, where the bound is exactly 1 ulp short
    ts, xs, T, v0, empty = fw_make_smooth_move(0.0, 0.0, _X3)
    plan = mirror.plan_smooth_move(0.0, _X3, 0.0)
    i = len(ts) // 2
    dev = abs(plan.sample_rev(ts[i]) - xs[i])
    ulp_at_x3 = float(np.spacing(f32(_X3)))
    assert ulp_at_x3 == pytest.approx(9.537e-7, rel=1e-3)
    assert dev > _TOL_PHASE0_POS_REV                # Phase 0's bound cannot hold
    assert dev == pytest.approx(ulp_at_x3, rel=0.05)   # ...because it is 1.00 ulp


def test_the_rest_to_rest_profile_is_unchanged_by_the_transcription():
    """The pre-Phase-4 firmware expression, evaluated in float32, against the new.

    ``pos = x0 + delta*s(tau) + u*h(tau)`` with ``u = 0`` must reduce to the old
    ``pos = x0 + delta*s(tau)`` EXACTLY — not approximately — or every existing
    hand move changes when the Teensy is flashed.  ``0.0f * h`` is exactly
    ``+0.0f`` and ``y + 0.0f == y`` for finite ``y``, so the reduction is bitwise;
    this asserts it on the actual transcription rather than trusting the argument.
    """
    for start, target in [(0.0, _X3), (_X3, 6.1267), (7.7004, 10.0543),
                          (9.9594, 0.0)]:
        ts, xs, T, v0, empty = fw_make_smooth_move(start, 0.0, target)
        assert not empty and float(v0) == 0.0
        delta = f32(f32(target) - f32(start))
        invT = f32(f32(1.0) / T)
        t = f32(0.0)
        for x in xs:
            old = f32(f32(start) + f32(delta * fw_quintic_s(f32(t * invT))))
            assert float(old) == x, (start, target, float(t))
            t = f32(t + f32(f32(1.0) / f32(500)))


def test_velocity_continuity_is_actually_achieved_at_the_seam():
    """The property the whole phase is for, read off the emitted samples.

    The first two samples' finite difference must approach the LIVE velocity, not
    zero.  Pre-fix it approached zero from any live velocity, which is the
    commanded velocity step that produced the measured 10.7-55.3 mm dip.
    """
    for v0 in (8.0, -8.0, 9.0):
        ts, xs, T, seed, empty = fw_make_smooth_move(_X3, v0, _X3)
        assert not empty and float(seed) == pytest.approx(v0, abs=1e-5)
        v_seam = (xs[1] - xs[0]) / (ts[1] - ts[0])
        assert v_seam == pytest.approx(v0, rel=0.02), (v0, v_seam)


def test_the_fallback_emits_a_from_rest_seed_so_the_bench_can_see_it():
    """The cannot-fit branch is deliberately observable, not silent.

    ``tools/probes/hand_stroke_timeline.py`` identifies a from-rest quintic seed
    by the commanded position starting exactly at the live encoder value with
    zero commanded velocity — which is precisely what the fallback emits.  Bench
    row 2 of § Section HAND aborts on it, so a fallback firing where the plan
    expects continuity shows up as a capture failure rather than as nothing.
    """
    ts, xs, T, seed, empty = fw_make_smooth_move(7.7004, 119.6, _X3)
    assert not empty
    assert float(seed) == 0.0
    assert xs[0] == pytest.approx(7.7004, abs=1e-6)     # seeded at the live pos
    v_seam = (xs[1] - xs[0]) / (ts[1] - ts[0])
    assert abs(v_seam) < 1.0                            # ...and from rest


# ══════════════════════════════════════════════════════════════════════════
#  5. compile and run the REAL Trajectory.h
# ══════════════════════════════════════════════════════════════════════════
#
# The transcription above is portable but it is still a transcription: a C++
# syntax error, a wrong namespace qualification or a stray `float` truncation in
# `Trajectory.h` would leave every test in this file green and surface only when
# the operator flashes the Platform Teensy.  `Teensy_code_platform/` is an Arduino sketch
# with no platformio.ini, so `tests/firmware/native/` does not build it — but
# `Trajectory.h` needs only <cmath>/<cstdint> and M_PI out of <Arduino.h>, so an
# 8-line shim is enough to compile and RUN the shipped header on the build host.
# g++-gated: a host without a compiler SKIPS (the Jetson run is authoritative),
# exactly as tests/firmware/native/build.py does.

_SHIM = """#pragma once
#include <cmath>
#include <cstdint>
#include <cstddef>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
"""

_DRIVER = r"""
#include "Trajectory.h"
#include <cstdio>
volatile float current_hand_position = 0.0f;
volatile float current_hand_velocity = 0.0f;
int main() {
  static const float cases[][3] = {%s};
  for (unsigned i = 0; i < sizeof(cases)/sizeof(cases[0]); ++i) {
    current_hand_position = cases[i][0];
    current_hand_velocity = cases[i][1];
    Trajectory tr = makeSmoothMove(cases[i][2]);
    if (tr.t.empty()) { printf("EMPTY\n"); continue; }
    float lo = tr.x[0], hi = tr.x[0];
    for (unsigned k = 0; k < tr.x.size(); ++k) {
      lo = fminf(lo, tr.x[k]); hi = fmaxf(hi, tr.x[k]);
    }
    printf("CASE %%u %%.9g %%.9g %%.9g %%.9g %%.9g %%.9g\n", (unsigned)tr.t.size(),
           tr.t.back(), tr.x.front(), tr.x.back(), tr.v.front(), lo, hi);
    if (tr.v.size() != tr.t.size() || tr.tor.size() != tr.t.size()) {
      printf("RAGGED\n"); continue;
    }
    for (unsigned k = 0; k < tr.t.size(); ++k)
      printf("S %%.9g %%.9g %%.9g %%.9g\n", tr.t[k], tr.x[k], tr.v[k], tr.tor[k]);
  }
  return 0;
}
"""


def test_the_shipped_trajectory_h_compiles_and_agrees_with_the_mirror(tmp_path):
    """Compile and RUN ``Trajectory.h`` itself, then compare against the mirror.

    Strictly stronger than the transcription above — it is the actual header, in
    the actual ``float``, with the actual namespace qualifications — and it is the
    only thing in the suite that would catch a C++ error in a file whose only
    other validation is a hand flash.  ``-Wall -Wextra -Werror`` because an
    unused-variable or narrowing warning in a 500 Hz trajectory generator is a
    finding, not noise.
    """
    import shutil
    import subprocess
    gpp = shutil.which('g++')
    if gpp is None:
        pytest.skip('no g++ on this host; the Jetson run is authoritative')

    (tmp_path / 'Arduino.h').write_text(_SHIM)
    # %.9e, not %.9g: %g renders 0.0 as "0", and "0f" is not a C++ float literal.
    body = ', '.join('{%.9ef, %.9ef, %.9ef}' % (s, v, t) for s, t, v in _CASES)
    (tmp_path / 'main.cpp').write_text(_DRIVER % body)
    exe = tmp_path / 'a.out'
    r = subprocess.run(
        [gpp, '-std=c++14', '-Wall', '-Wextra', '-Werror',
         '-I', str(tmp_path), '-I', _FW_DIR, '-o', str(exe),
         str(tmp_path / 'main.cpp')],
        capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    out = subprocess.run([str(exe)], capture_output=True, text=True)
    assert out.returncode == 0, out.stderr

    # One record per case: 'EMPTY', or a 'CASE' summary followed by its 'S'
    # samples.  The per-sample stream is what closes the hole that
    # test_the_velocity_and_torque_streams_match_the_mirror pins; grouping happens
    # here so both tests read one compile.
    records = []
    for ln in out.stdout.strip().splitlines():
        if not ln:
            continue
        assert ln != 'RAGGED', 't/x/v/tor lengths diverged in the firmware'
        if ln == 'EMPTY' or ln.startswith('CASE'):
            records.append([ln, []])
        else:
            assert ln.startswith('S '), ln
            records[-1][1].append([float(f) for f in ln.split()[1:]])
    assert len(records) == len(_CASES)
    for (start, target, v0), (line, samples) in zip(_CASES, records):
        plan = mirror.plan_smooth_move(start, target, v0)
        if line == 'EMPTY':
            assert plan.empty, (start, target, v0)
            continue
        assert not plan.empty, (start, target, v0)
        fields = line.split()[1:]
        n, dur, x0, xn, v_first, lo, hi = (
            [int(fields[0])] + [float(f) for f in fields[1:]])
        assert len(samples) == n

        # ── the VELOCITY and TORQUE feedforward streams ───────────────────────
        # Everything below this line about positions was already covered; the v
        # and tor streams were NOT, and they are what the ODrive receives as
        # feedforward alongside the setpoint.  Mutation-verified 2026-07-27
        # (finalize): changing `30.0f*t4` to `3.0f*t4` in quinticS1 leaves the
        # full-stroke prime commanding -354 rev/s of velocity feedforward at its
        # final sample and a finite-differenced 1863 rev/s^2, and EVERY test in
        # this file passed before this block existed — because the only velocity
        # ever read off the compiled header was tr.v[0].  A sign flip in quinticH1
        # (which preserves h1(0)=1 and h1(1)=0, so tr.v[0] still reads v0) and any
        # mutation of quinticS2/quinticH2 (which reach only tr.tor) were likewise
        # invisible.
        _K_TOR = (mirror.INERTIA_HAND_ONLY_KG * mirror.HAND_SPOOL_RADIUS_M
                  / mirror._LINEAR_GAIN)
        worst_v = worst_tor = 0.0
        for t_k, x_k, v_k, tor_k in samples:
            worst_v = max(worst_v, abs(v_k - plan.sample_vel_rps(t_k)))
            worst_tor = max(worst_tor,
                            abs(tor_k - _K_TOR * plan.sample_accel_rps2(t_k)))
        # Scaled by the profile's own magnitude: at 119.6 rev/s the velocity
        # stream spans hundreds of rev/s, where one float32 ulp is already 1e-5.
        v_scale = max(1.0, max(abs(v_k) for _, _, v_k, _ in samples))
        a_scale = max(1.0, plan.peak_accel_rps2)
        assert worst_v <= 2e-4 * v_scale, ((start, target, v0), worst_v)
        assert worst_tor <= 2e-4 * a_scale * _K_TOR, (
            (start, target, v0), worst_tor)
        # ...and the two boundary properties the physics depends on, read off the
        # FIRMWARE rather than off the mirror: the profile ENDS at rest (a
        # non-zero terminal velocity feedforward is a standing command into the
        # end stop) and never asks for more acceleration than the limit allows.
        assert abs(samples[-1][2]) <= 0.02 * v_scale, 'terminal vel_ff not ~0'
        peak_a = max(abs(tor_k) for _, _, _, tor_k in samples) / _K_TOR
        assert peak_a <= _TT['MAX_SMOOTH_MOVE_HAND_ACCEL_RPS2'] * 1.001, peak_a

        # The branch decision — the thing a float32/float64 split could flip.
        # Keyed on the SEEDED velocity, not on ``velocity_continuous``: a move
        # dispatched to a hand inside the dead-band is legitimately continuous
        # AND legitimately seeded from rest.
        assert (abs(v_first) > 1e-3) == (plan.v0_seed_rps != 0.0), (
            (start, target, v0), 'v_first=%g' % v_first)
        if plan.v0_seed_rps != 0.0:
            assert v_first == pytest.approx(v0, rel=1e-4)
        # The LAST SAMPLE's time, not the duration: `for (t = 0; t <= duration;
        # t += dT)` stops at the largest multiple of 2 ms at or below it, so the
        # emitted profile is up to one sample period short — pre-existing, and
        # what Teensy_code_platform.ino:524 turns into `smoothDur_us`.  Pinned rather than
        # tolerated, because the resulting end-position gap is what a caller
        # relying on "the prelude ends exactly at the main trajectory's first
        # sample" is actually given.
        # `n` can also be one SHORT of the ideal count: `t` accumulates in float,
        # and 2 ms is not exactly representable, so at a duration that is an exact
        # multiple of dT (0.05 s, the floored case) the 26th step lands at
        # 0.050000004 > duration and the loop stops at 25 samples.  Pre-existing
        # and harmless (the profile is flat at both ends), pinned so it is not
        # mistaken for a Phase-4 regression.
        dt = 1.0 / _SAMPLE_RATE
        ideal = int(math.floor(plan.duration_s / dt + 1e-9)) + 1
        assert ideal - 1 <= n <= ideal, ((start, target, v0), n, ideal)
        assert 0.0 <= plan.duration_s - dur < dt + 1e-6
        assert xn == pytest.approx(target, abs=2e-4), 'end-position quantisation'
        assert x0 == pytest.approx(start, abs=_TOL_SAMPLE_REV)
        assert lo == pytest.approx(start + plan.excursion_lo_rev,
                                   abs=_TOL_SAMPLE_REV, rel=1e-5)
        assert hi == pytest.approx(start + plan.excursion_hi_rev,
                                   abs=_TOL_SAMPLE_REV, rel=1e-5)
        # and the excursion stays inside the stroke, read off the FIRMWARE.
        # Written against the PHYSICAL limits (the overextension guard, and
        # encoder zero as a literal 0.0), NOT against the expressions the clamp
        # itself evaluates — an assertion that re-uses the code's own predicate
        # passes for any value of the clamp constants, including a floor moved a
        # full rev below the bottom hard stop.
        assert hi <= max(mirror.HAND_MOTOR_MAX_POSITION_REVS, start, target) + 1e-6
        assert lo >= min(0.0, start, target) - 1e-3, (
            (start, target, v0), 'commanded below encoder zero')


def test_trajectory_h_structure_lint():
    """A PORTABLE backstop for the three structural properties of the fix.

    Mutation-verified (2026-07-27): dropping the ``at_rest`` conjunct, seeding
    ``v0 = 0`` unconditionally, reverting ``smoothMoveDuration`` to the
    rest-to-rest formula, and disabling the end-stop check are each caught by
    ``test_the_shipped_trajectory_h_compiles_and_agrees_with_the_mirror`` — and by
    NOTHING else in this file, because the float32 transcription above is
    hand-maintained and does not read the C++.  On a host without g++ that test
    skips, so these source-level assertions are the fallback.  They are
    deliberately coarse: they check that the mechanism is present, not that it is
    correct (the compiled test does that).
    """
    src = open(_TRAJ_H).read()
    # 1. the empty branch requires BOTH conditions — widening it widens a hole in
    #    the kind-3 clobber path (Teensy_code_platform.ino:472-475 returns before the clear)
    assert re.search(r'if\s*\(\s*fabsf\(delta_rev\)\s*<\s*1e-6f\s*&&\s*at_rest\s*\)',
                     src), 'the empty branch must require at_rest'
    assert re.search(r'const\s+bool\s+at_rest\s*=\s*fabsf\(start_vel\)\s*<=\s*'
                     r'SMOOTH_MOVE_V0_DEADBAND', src)
    # 2. v0 is seeded from the LIVE velocity, not from zero
    assert re.search(r'float\s+v0\s*=\s*at_rest\s*\?\s*0\.0f\s*:\s*start_vel\s*;',
                     src), 'v0 must be seeded from the live encoder velocity'
    # 3. the excursion is checked against BOTH end stops and gates the fallback
    assert 'smoothMoveExcursion(delta_rev, v0, duration, lo, hi)' in src
    assert re.search(r'if\s*\(start_rev\s*\+\s*hi\s*>\s*ceil_rev\s*'
                     r'\+\s*SMOOTH_MOVE_END_STOP_EPS_REV\s*\|\|\s*'
                     r'start_rev\s*\+\s*lo\s*<\s*floor_rev\s*'
                     r'-\s*SMOOTH_MOVE_END_STOP_EPS_REV\s*\|\|\s*'
                     r'duration\s*>\s*smoothMoveMaxDuration\(\)\s*\)', src)
    # 3b. the FLOOR is encoder zero, not the bottom hard stop.  Homing::
    #     HAND_ABS_POS_REV = -0.1 rev IS the stop (the axis homes downward into
    #     it); planning commanded travel onto it, with 0 rev of allowance for the
    #     measured +0.186 rev position-loop undershoot, is what this pins out.
    assert re.search(r'SMOOTH_MOVE_POS_FLOOR_REV\s*=\s*JBOp::HAND_RETRACT_REV',
                     src), 'the excursion floor must be encoder zero'
    # ...and the stop is not referenced by any CODE line (the comments explain
    # why it is not used, so strip them before looking).
    code = re.sub(r'/\*.*?\*/', '', src, flags=re.S)
    code = re.sub(r'//[^\n]*', '', code)
    assert 'Homing::HAND_ABS_POS_REV' not in code, (
        'the bottom hard stop must not be a commanded-position bound')
    # 3c. the duration is capped as well as the excursion — arresting v0 costs
    #     time, and an honoured prelude that outlasts every rest-to-rest move
    #     outgrows the host windows sized on them (_PRIME_INFLIGHT_S).
    assert re.search(r'return\s+sqrtf\(Geometry::HAND_MOTOR_MAX_POSITION_REVS\s*'
                     r'\*\s*QUINTIC_S2_MAX', src)
    # ...and the fallback is a rest-to-rest profile, never an empty one
    fallback = src[src.index('start_rev + hi > ceil_rev'):]
    fallback = fallback[:fallback.index('const float u')]
    assert 'v0 = 0.0f;' in fallback
    assert 'return tr;' not in fallback, (
        'the cannot-fit branch must NOT return early: an empty trajectory would '
        'let a kind-3 SAFE_ABORT retract fail to clobber an armed kind-0 stroke')
