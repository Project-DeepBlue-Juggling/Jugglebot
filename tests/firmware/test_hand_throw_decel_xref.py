"""Cross-reference the SHIPPED ``Trajectory.h`` throw against the sim mirror.

Contract: ``ros_ws/docs/hand_decel_feedforward.md`` (**C-HAND-2**).
Mirror-side properties: ``tests/sim/test_hand_throw_decel_ff.py``.

Sibling of ``test_hand_smooth_move_xref.py``, and deliberately a separate file:
that one's subject is ``makeSmoothMove`` (the prelude), this one's is
``buildThrow`` (the stroke).  Both compile and RUN the real header — for a file
whose only other validation is a hand flash, a Python transcription is not
evidence.

WHAT IT PINS, AND WHY EACH ONE MATTERS
--------------------------------------
1. **The decel segment's torque is the corrected conversion** — the change
   itself.  Mutating ``throwDecelToTorque`` back to ``accelToTorque`` restores
   the ~30 % under-torque that put the hand into its end stop on 2026-07-27.
2. **The accel and velocity-hold torques are NOT** — the scoping.  Correcting
   the ascent feedforward would raise the achieved release velocity and
   re-calibrate every throw height the machine has flown.
3. **The position and velocity streams are bit-unchanged** — asserted against
   closed forms re-derived here, not read off the mirror.  Every host window
   C-HAND-1 and Phase 3 sized is derived from these streams.
4. **kind-1 (``buildCatch``) still uses the legacy conversion** — the catch and
   the kind-3 clobber path are outside this contract entirely.
5. **The int16 wire quantisation cannot turn the feedforward into a visible
   undershoot** — half an LSB is 0.005 N.m, which at the BOTTOM of the flight
   band is 9 % of the whole command.

``g++``-gated exactly as the smooth-move xref is: a host without a compiler
SKIPS, and the operator's pre-flash gate must be run on a host that has one.
"""

from __future__ import annotations

import math
import os
import re
import shutil
import subprocess
import sys

import pytest

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:                  # tests/firmware has no conftest
    sys.path.insert(0, _REPO_ROOT)

from sim.hand import trajectory as mirror           # noqa: E402

_FW_DIR = os.path.join(_REPO_ROOT, 'ros_ws', 'src', 'jugglebot', 'Teensy_code_platform')
_HEADER = os.path.join(_FW_DIR, 'hardware_config.h')

#: Release speeds spanning the flight band THIS FILE WAS WRITTEN AGAINST
#: (0.55 -> 1.10 s is v = 2.698 -> 5.396 m/s). Deliberately literal, and
#: deliberately WIDER than the derived envelope that superseded it on 2026-08-18
#: (contract C-HAND-3: 0.4949 -> 0.8871 s = 2.440 -> 4.357 m/s) — the
#: feedforward being sized over more speeds than the gate admits is the safe
#: direction. Plus the 4.858 m/s tier
#: that made physical contact with the end stop on 2026-07-27.
_VELS = (2.698, 3.440, 3.969, 4.858, 5.396)

#: Sample-stream tolerance.  Same justification as the smooth-move xref's:
#: float32 arithmetic, not model divergence.  2e-5 rev = 0.63 um of cable.
_TOL_REV = 2e-5
#: Torque tolerance, relative to the profile's own magnitude.  The firmware
#: computes in float32 and the mirror in float64.
_TOL_TOR_REL = 5e-6

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
  static const float vels[] = {%s};
  printf("K %%.9g %%.9g\n", (double)THROW_DECEL_TORQUE_K,
         (double)(INERTIA_HAND_ONLY * HAND_SPOOL_R));
  for (unsigned i = 0; i < sizeof(vels)/sizeof(vels[0]); ++i) {
    HandTrajGenerator gen(vels[i]);
    Trajectory thr = gen.makeThrow();
    Trajectory cat = gen.makeCatch();
    if (thr.v.size() != thr.t.size() || thr.tor.size() != thr.t.size()) {
      printf("RAGGED\n"); continue;
    }
    printf("THROW %%u %%.9g\n", (unsigned)thr.t.size(), (double)vels[i]);
    for (unsigned k = 0; k < thr.t.size(); ++k)
      printf("S %%.9g %%.9g %%.9g %%.9g\n", (double)thr.t[k], (double)thr.x[k],
             (double)thr.v[k], (double)thr.tor[k]);
    printf("CATCH %%u\n", (unsigned)cat.t.size());
    for (unsigned k = 0; k < cat.t.size(); ++k)
      printf("C %%.9g %%.9g\n", (double)cat.t[k], (double)cat.tor[k]);
  }
  return 0;
}
"""


def _parse_namespace(ns: str) -> dict:
    """Constants of one ``namespace`` block of the SHIPPED firmware header.

    Brace-COUNTED, for the same reason the smooth-move xref counts: ``Geometry``
    carries braced array initialisers and a naive scan silently truncates.
    """
    with open(_HEADER) as fh:
        src = fh.read()
    start = src.index('namespace %s {' % ns)
    depth, i = 0, start
    while True:
        if src[i] == '{':
            depth += 1
        elif src[i] == '}':
            depth -= 1
            if depth == 0:
                break
        i += 1
    body = src[start:i]
    out = {}
    for m in re.finditer(
            r'constexpr\s+\w+\s+(\w+)\s*=\s*([-+0-9.eE]+)[fu]?\s*;', body):
        out[m.group(1)] = float(m.group(2))
    return out


_TT = _parse_namespace('TeensyTraj')


def _model(v: float) -> dict:
    """``calcThrow`` + ``calcCatch``, re-derived from the SHIPPED header.

    Deliberately not imported from the mirror: this is the independent route,
    so a change made in both the mirror and the firmware still has to face a
    third derivation from the constants that are actually flashed.
    """
    r = _TT['HAND_SPOOL_RADIUS_M']
    gain = _TT['LINEAR_GAIN_FACTOR'] / (math.pi * r * 2.0)
    ir = _TT['INERTIA_RATIO']
    total = _TT['HAND_STROKE_M'] - 2.0 * _TT['STROKE_MARGIN_M']
    vel_hold = _TT['THROW_VEL_HOLD_PCT'] * total
    accel_st = total - vel_hold
    t_acc = 2.0 / (ir + 1.0) * accel_st / v
    t_vel = vel_hold / v
    t_dec = t_acc * ir
    a_acc = v / t_acc
    a_dec = -a_acc / ir
    x1 = 0.5 * a_acc * t_acc ** 2
    x2 = x1 + v * t_vel
    x3 = x2 + v * t_dec + 0.5 * a_dec * t_dec ** 2
    k_legacy = _TT['INERTIA_HAND_ONLY_KG'] * r
    k_decel = (_TT['THROW_DECEL_REFLECTED_INERTIA_KGM2']
               * 2.0 * math.pi * gain)
    # calcCatch
    vC = -_TT['CATCH_VEL_RATIO'] * v
    irC = 1.0 / ir
    velH = _TT['CATCH_VEL_HOLD_PCT'] * total
    accS = total - velH
    ct_acc = -(2.0 / (irC + 1.0)) * accS / vC
    catchA = vC / ct_acc
    catchD = -catchA / irC
    return dict(gain=gain, t_acc=t_acc, t_vel=t_vel, t_dec=t_dec,
                a_acc=a_acc, a_dec=a_dec, x1=x1, x2=x2, x3=x3,
                k_legacy=k_legacy, k_decel=k_decel,
                catchA=catchA, catchD=catchD, v=v)


def _sample(m: dict, t_rel: float):
    """(pos_m, vel_mps, tor_Nm) at ``t_rel`` seconds relative to release.

    Segment selection mirrors ``buildSegment``'s ``while (t > tA[idx+1]-start)``
    — strictly greater, so a sample landing exactly on a boundary belongs to the
    EARLIER segment.
    """
    t_local = t_rel + m['t_acc'] + m['t_vel']
    if t_local <= m['t_acc']:
        return (0.5 * m['a_acc'] * t_local ** 2,
                m['a_acc'] * t_local,
                m['k_legacy'] * m['a_acc'])
    if t_local <= m['t_acc'] + m['t_vel']:
        tau = t_local - m['t_acc']
        return m['x1'] + m['v'] * tau, m['v'], 0.0
    tau = t_local - m['t_acc'] - m['t_vel']
    return (m['x2'] + m['v'] * tau + 0.5 * m['a_dec'] * tau ** 2,
            m['v'] + m['a_dec'] * tau,
            m['k_decel'] * m['a_dec'])


@pytest.fixture(scope='module')
def fw_run(tmp_path_factory):
    """Compile and run the shipped ``Trajectory.h``; return its parsed output."""
    gpp = shutil.which('g++')
    if gpp is None:
        pytest.skip('no g++ on this host; the Jetson run is authoritative')
    tmp_path = tmp_path_factory.mktemp('throwxref')
    (tmp_path / 'Arduino.h').write_text(_SHIM)
    body = ', '.join('%.9ef' % v for v in _VELS)
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

    k_decel = k_legacy = None
    cases = []                      # [(v, [throw samples], [catch (t,tor)])]
    mode = None
    for ln in out.stdout.strip().splitlines():
        if not ln:
            continue
        assert ln != 'RAGGED', 't/x/v/tor lengths diverged in the firmware'
        f = ln.split()
        if f[0] == 'K':
            k_decel, k_legacy = float(f[1]), float(f[2])
        elif f[0] == 'THROW':
            cases.append((float(f[2]), [], []))
            mode = 'T'
        elif f[0] == 'CATCH':
            mode = 'C'
        elif f[0] == 'S':
            assert mode == 'T'
            cases[-1][1].append([float(x) for x in f[1:]])
        elif f[0] == 'C':
            assert mode == 'C'
            cases[-1][2].append([float(x) for x in f[1:]])
    assert len(cases) == len(_VELS)
    return dict(k_decel=k_decel, k_legacy=k_legacy, cases=cases)


# ══════════════════════════════════════════════════════════════════════════
#  the two conversions, read off the compiled firmware
# ══════════════════════════════════════════════════════════════════════════

def test_the_firmware_declares_two_distinct_torque_conversions(fw_run):
    """``THROW_DECEL_TORQUE_K`` != ``INERTIA_HAND_ONLY * HAND_SPOOL_R``.

    Both are read out of the compiled header, so this fails if the constant is
    hand-edited in the firmware's own ``hardware_config.h`` past codegen.
    """
    m = _model(_VELS[0])
    assert fw_run['k_legacy'] == pytest.approx(m['k_legacy'], rel=1e-6)
    assert fw_run['k_decel'] == pytest.approx(m['k_decel'], rel=1e-6)
    assert fw_run['k_decel'] / fw_run['k_legacy'] == pytest.approx(
        1.2891, rel=1e-3)
    # ...and the mirror agrees with the compiled firmware, not just with itself
    assert fw_run['k_decel'] == pytest.approx(
        mirror._TORQUE_K_THROW_DECEL, rel=1e-6)
    assert fw_run['k_legacy'] == pytest.approx(
        mirror._TORQUE_K_LEGACY, rel=1e-6)


# ══════════════════════════════════════════════════════════════════════════
#  the throw stream: position + velocity unchanged, torque corrected
# ══════════════════════════════════════════════════════════════════════════

def test_the_shipped_throw_matches_the_model_on_every_stream(fw_run):
    """Sample-by-sample over the whole flight band.

    Segment boundaries are excluded from the TORQUE comparison only, and only
    within one 500 Hz sample period: the torque stream is a step at the segment
    seams, so a sample straddling one legitimately disagrees on which side it
    belongs to under float32 vs float64 time accumulation.  Position and
    velocity are continuous there and get no such exemption.  The number of
    exempted samples is asserted, so the exemption cannot quietly grow to cover
    the whole stream.
    """
    dt = 1.0 / _TT['SAMPLE_RATE_HZ']
    for v, samples, _catch in fw_run['cases']:
        m = _model(v)
        assert len(samples) > 40          # 57 at the band ceiling, 116 at 2.698
        exempt = 0
        worst_x = worst_v = worst_tor = 0.0
        for t_k, x_k, v_k, tor_k in samples:
            want_x, want_v, want_tor = _sample(m, t_k)
            worst_x = max(worst_x, abs(x_k - want_x * m['gain']))
            worst_v = max(worst_v, abs(v_k - want_v * m['gain']))
            t_local = t_k + m['t_acc'] + m['t_vel']
            near_seam = (abs(t_local - m['t_acc']) < dt
                         or abs(t_local - m['t_acc'] - m['t_vel']) < dt)
            if near_seam:
                exempt += 1
                continue
            worst_tor = max(worst_tor, abs(tor_k - want_tor))
        a_scale = max(abs(m['a_acc']), abs(m['a_dec']))
        assert worst_x <= _TOL_REV, (v, worst_x)
        assert worst_v <= _TOL_REV * max(1.0, v * m['gain']), (v, worst_v)
        assert worst_tor <= _TOL_TOR_REL * a_scale * m['k_decel'], (v, worst_tor)
        assert exempt <= 4, (v, exempt)


def test_the_throw_torque_stream_is_exactly_three_values(fw_run):
    """accel (legacy conversion), zero, decel (corrected) — in that order.

    A run-structure assertion rather than a per-sample one, because it is the
    SCOPING that has to hold: if a future edit applies the corrected conversion
    to the accel segment too, every per-sample tolerance still passes against a
    model updated in the same commit, but this test names the three values
    independently and fails.
    """
    for v, samples, _catch in fw_run['cases']:
        m = _model(v)
        want_acc = m['k_legacy'] * m['a_acc']
        want_dec = m['k_decel'] * m['a_dec']
        runs = []
        for _t, _x, _v, tor in samples:
            if not runs or abs(runs[-1][0] - tor) > 1e-9:
                runs.append([tor, 0])
            runs[-1][1] += 1
        assert len(runs) == 3, (v, [r[0] for r in runs])
        assert runs[0][0] == pytest.approx(want_acc, rel=1e-5)
        assert runs[1][0] == pytest.approx(0.0, abs=1e-9)
        assert runs[2][0] == pytest.approx(want_dec, rel=1e-5)
        # the ACCEL torque must NOT have moved — that is the scoping half
        assert runs[0][0] == pytest.approx(
            m['a_acc'] * _TT['INERTIA_HAND_ONLY_KG']
            * _TT['HAND_SPOOL_RADIUS_M'], rel=1e-5)
        # run lengths track the commanded segment durations
        dt = 1.0 / _TT['SAMPLE_RATE_HZ']
        assert runs[0][1] == pytest.approx(m['t_acc'] / dt, abs=2)
        assert runs[1][1] == pytest.approx(m['t_vel'] / dt, abs=2)
        assert runs[2][1] == pytest.approx(m['t_dec'] / dt, abs=2)


def test_the_throw_starts_at_zero_and_ends_within_one_sample_of_the_stroke_top(fw_run):
    """``x3`` is the catch trajectory's first sample and the prime target.

    ``buildSegment`` emits ``while (t < end)``, so the LAST sample lands up to
    one 500 Hz period short of ``t3`` rather than on it.  The residual distance
    is exactly the decel's own ``0.5*|a_dec|*dt^2`` — 0.0072 rev = 0.23 mm at the
    band ceiling, and 3.1e-4 rev at the 4.858 m/s tier (which is why the
    2026-07-27 trace reads a commanded hold at ``9.9592`` against a nominal
    ``9.9594``).  Bounded exactly here rather than waved through with a round
    tolerance, so a change that made it grow would fail.
    """
    total = _TT['HAND_STROKE_M'] - 2.0 * _TT['STROKE_MARGIN_M']
    dt = 1.0 / _TT['SAMPLE_RATE_HZ']
    for v, samples, _catch in fw_run['cases']:
        m = _model(v)
        x3_rev = total * m['gain']
        shortfall = x3_rev - samples[-1][1]
        bound = 0.5 * abs(m['a_dec']) * dt * dt * m['gain']
        assert 0.0 <= shortfall <= bound * 1.02 + 1e-6, (v, shortfall, bound)
        assert samples[0][1] == pytest.approx(0.0, abs=1e-6)


def test_the_throw_ends_with_a_residual_velocity_feedforward(fw_run):
    """Pinned as a KNOWN, PRE-EXISTING effect — NOT introduced by C-HAND-2.

    The same one-sample shortfall leaves the final frame carrying a non-zero
    Vel_FF: ``|a_dec| * (t3 - t_last)``, up to ``|a_dec| * dt`` = **7.19 rev/s at
    the shipped band ceiling**.  ``Set_Input_Pos`` is a LATCHED setpoint, so the
    drive keeps that feedforward until the NEXT hand command overwrites it, and
    while it is held the position loop balances it only at ``Vel_FF / pos_gain``
    of position error — i.e. it biases the hand UPWARD, toward the end stop.

    HOW BIG IT ACTUALLY IS, measured rather than assumed
    ----------------------------------------------------
    Read off ``temp/logs/toss_trace_2026-07-27_15-39-50.jsonl``, the sitting's
    own capture, over the post-stroke hold window of all 17 self-tosses:

    * ``vel_ff_cmd`` peaks at **2.02 / 0.82 / 1.41 / 1.14 rev/s** at the 2.742 /
      3.440 / 3.969 / 4.858 m/s tiers respectively — note it does NOT rise with
      speed, so it is the sampled tail of the residual rather than the residual
      itself — and its MEDIAN through the same windows is **0.00-0.02 rev/s**.
    * So the exposure is **bounded, not indefinite**, and the bound that carries
      the argument is the MAGNITUDE rather than the window.  The steady-state
      bias the residual would reach if it were held indefinitely is
      ``2.02/35 = 0.058`` rev = 1.8 mm at the observed maxima, and
      ``7.19/35 = 0.205`` rev = 6.5 mm at the band ceiling.  Small either way.

      The window itself is longer than this docstring used to say.  It read
      "measured 28.0-61.6 ms after the commanded stroke end" — the 2026-07-28
      sitting's hand measurement, anchored on the sample where ``pos_cmd`` hit x3
      EXACTLY and taken over one session's tosses.  ``hand_stroke_timeline.py``
      now reports the same quantity as ``stroke_end_hold_ms`` and measures
      **30.6-128.3 ms** over the sitting's 44 post-stroke commands — and on 25 of
      69 clean throws NO command lands before the catch descent at all, so the
      residual can be held for the whole inter-command interval.  That widening
      does not change the conclusion, because the conclusion rests on the
      1.8-6.5 mm ceiling above, not on the window being short.  Cross-reference:
      ``logbook/2026-08-18-trunc-criterion-stroke-end.md``.

    A HYPOTHESIS THIS TEST WITHDREW
    -------------------------------
    The measured post-stroke offset was first attributed to this residual.  The
    trace refutes it: the median ``vel_ff_cmd`` through those same windows is
    0.00-0.02 rev/s, which predicts an offset of 0.0006 rev — two orders of
    magnitude too small.  The settled offset therefore has ANOTHER cause
    (velocity-integrator residual and/or friction holding the hand above its
    setpoint) and is recorded as an open question, not as a finding.

    The offset itself, per tier, straight off the probe (all 17 tosses; quoted
    per tier because ONE span hides the point).  Corrected 2026-07-29 — this
    docstring previously read "+0.044 to +0.078 rev", an upper bound that appears
    nowhere in the data and a lower bound that silently dropped the 4.858 m/s
    tier, which is the tier this whole contract exists for:

    ======== =====================
    tier     ``pos_meas - pos_cmd``
    ======== =====================
    2.742    +0.0669 .. +0.0671
    3.440    +0.0429 .. +0.0663
    3.969    +0.0190 .. +0.0412
    4.858    **-0.0043 .. +0.0069**
    ======== =====================

    Note it SHRINKS with speed and reaches ~0 at the tier that touched the stop,
    which is itself an argument against the withdrawn hypothesis.

    WHY IT IS NOT FIXED HERE
    ------------------------
    The residual is a property of ``buildSegment``, shared with ``buildCatch``,
    so removing it changes the commanded terminal frame of the kind-1 catch
    descent that meets the ball — a safety-relevant surface, and an operator
    decision.  ``tests/firmware/test_hand_smooth_move_xref.py`` already asserts
    the OPPOSITE for ``makeSmoothMove`` ("a non-zero terminal velocity
    feedforward is a standing command into the end stop"), which is why this one
    is pinned with its exact bound rather than left unstated: it must not grow,
    and the next person to read the two files must not think they disagree by
    accident.  Bench row **HAND-7.6** measures it.
    """
    dt = 1.0 / _TT['SAMPLE_RATE_HZ']
    pos_gain = 35.0                              # ODRIVE_HAND_POS_GAIN
    worst_rev = 0.0
    for v, samples, _catch in fw_run['cases']:
        m = _model(v)
        v_term = samples[-1][2]
        bound = abs(m['a_dec']) * dt * m['gain']
        assert 0.0 <= v_term <= bound * 1.02 + 1e-6, (v, v_term, bound)
        worst_rev = max(worst_rev, v_term / pos_gain)
    # The worst-case steady bias this phase accepts, as a number so it cannot
    # creep: 0.205 rev at the band ceiling, and only if nothing overwrites it.
    assert worst_rev <= 0.21, worst_rev


# ══════════════════════════════════════════════════════════════════════════
#  kind-1 is outside this contract
# ══════════════════════════════════════════════════════════════════════════

def test_the_catch_torque_stream_still_uses_the_legacy_conversion(fw_run):
    """``buildCatch`` untouched — the catch and kind-3 are outside C-HAND-2.

    Failure mode this closes: "make the two builders consistent" applied to the
    catch, which changes commanded braking on the descent that meets the ball.
    """
    for v, _samples, catch in fw_run['cases']:
        m = _model(v)
        vals = sorted({round(tor, 9) for _t, tor in catch})
        assert len(vals) == 3, (v, vals)
        assert vals[0] == pytest.approx(m['k_legacy'] * m['catchA'], rel=1e-5)
        assert vals[1] == pytest.approx(0.0, abs=1e-9)
        assert vals[2] == pytest.approx(m['k_legacy'] * m['catchD'], rel=1e-5)


# ══════════════════════════════════════════════════════════════════════════
#  the wire
# ══════════════════════════════════════════════════════════════════════════

def test_wire_quantisation_cannot_produce_a_visible_undershoot(fw_run):
    """Half an LSB is 0.005 N.m — 9 % of the command at the band floor.

    ``packTrajectory`` sends ``int16(lrintf(tor * InputScale::hand_tor))`` with
    ``hand_tor = 100``, so the wire resolution is 0.01 N.m.  Rounding UP is one
    of the two directions that can break C-HAND-2's one-sided-safety half.

    GRAVITY IS THE OTHER ONE, AND IT IS BIGGER
    ------------------------------------------
    Corrected 2026-07-29.  This test used to bound the wire rounding ALONE
    against 0.10 rev and pass with 0.09.  That understated the case: on an
    UPWARD deceleration **gravity brakes in the same direction as the
    feedforward**, worth ``tau_grav/(2*pi*a_cmd)`` of effective inertia —
    +1.46e-6 at the band floor against a declared 9.5e-6, i.e. far more than the
    +0.89e-6 the wire rounding contributes there.  With both terms the OPEN-LOOP
    commanded undershoot at the band floor is ~0.58 rev, not 0.09.

    So the honest set of claims, all asserted below:

    1. **At the band CEILING — where the ladder's new R5 rung lives — there is
       no open-loop over-brake at all**, even with gravity and worst-case
       rounding: ``9.5e-6 + 0.37e-6 + 0.22e-6 = 1.009e-5`` is still under the
       ``1.0126e-5`` decel-side bound.  This is the statement that matters for
       the high tiers, and it is the one the phase exists for.
    2. **At the band FLOOR there IS an open-loop over-brake**, and it is bounded
       here rather than denied.  It is not fixable by choosing a different
       declared inertia: meeting the gravity-inclusive inequality at the floor
       needs ``J_ff <= 8.69e-6``, which puts the pessimistic peak at 10.80 rev,
       past the 10.60 hard-abort line.  The two requirements are incompatible.
    3. **The physical dip is what H7.4 gates, and the loop attenuates it.** The
       measured loop attenuation at the band-floor tier is 4.3 % of the open-loop
       excursion (0.0739 rev measured against a 1.7185 rev open-loop bracket at
       2.742 m/s), and the gravity term is largest exactly where the decel ramp
       is longest and the loop most effective.  0.58 x 0.043 = 0.025 rev against
       the 0.100 rev gate.

    The remaining enforcement is the bench: runbook row **H7.4**, and the new
    band-floor rung **R0**, which exists because that is where over-braking bites
    first and the ladder previously never went there.
    """
    tor_scale = 100.0
    #: Binding DECEL-side lower bound; see the sim test module's docstring.
    j_true_min = 1.0126e-5
    tau_grav = 1.50 * 0.00551333324983716
    #: Measured loop attenuation of an open-loop excursion at the band-floor
    #: tier: 0.0739 rev measured / 1.7185 rev open-loop, 2026-07-27 capture.
    loop_attenuation = 0.043
    j_ff = _TT['THROW_DECEL_REFLECTED_INERTIA_KGM2']
    total = _TT['HAND_STROKE_M'] - 2.0 * _TT['STROKE_MARGIN_M']
    worst_open_loop = 0.0
    for v, samples, _catch in fw_run['cases']:
        m = _model(v)
        tor = min(s[3] for s in samples)               # the decel value
        assert abs(tor * tor_scale) < 32767            # int16 headroom
        a_dec_rad = abs(m['a_dec']) * m['gain'] * 2.0 * math.pi
        # worst-case effective braking inertia: declared + wire round-up + gravity
        j_eff_max = (j_ff * (abs(tor) + 0.5 / tor_scale) / abs(tor)
                     + tau_grav / a_dec_rad)
        d_dec = 0.5 * v * m['t_dec'] * m['gain']
        assert d_dec == pytest.approx(
            _TT['INERTIA_RATIO'] * (total - _TT['THROW_VEL_HOLD_PCT'] * total)
            / (1.0 + _TT['INERTIA_RATIO']) * m['gain'], rel=1e-9)
        if j_eff_max > j_true_min:
            worst_open_loop = max(worst_open_loop,
                                  d_dec * (1.0 - j_true_min / j_eff_max))

        # Claim 1 — nothing over-brakes at the top of the band, gravity included.
        # _VELS[-1] = 5.396 m/s is the 1.10 s flight this file was written
        # against — the band ceiling AT THE TIME, and runbook rung R5. Since
        # 2026-08-18 the band is DERIVED (C-HAND-3) and its ceiling is
        # 4.357 m/s, so 5.396 is now OUT of contract in both directions: the
        # feedforward is still sized there (deliberately wider than the
        # admitted set) but no goal can reach it.
        if v == max(_VELS):
            assert j_eff_max <= j_true_min, (
                f'open-loop over-brake at the band ceiling: j_eff {j_eff_max:.4e} '
                f'> {j_true_min:.4e}')

    # Claim 2 — the floor's open-loop over-brake, bounded rather than denied.
    assert worst_open_loop <= 0.60, (
        f'worst OPEN-LOOP commanded undershoot {worst_open_loop:.4f} rev')
    # Claim 3 — what H7.4 will actually see, after the loop opposes it.
    assert worst_open_loop * loop_attenuation <= 0.10, (
        f'predicted physical dip {worst_open_loop*loop_attenuation:.4f} rev '
        'exceeds the H7.4 gate')
