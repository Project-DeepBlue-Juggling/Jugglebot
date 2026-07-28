"""Contract C-HAND-2 — the throw's post-release deceleration feedforward.

Normative statement: ``ros_ws/docs/hand_decel_feedforward.md``.
Enforcement point: ``Teensy_code/Trajectory.h::throwDecelToTorque``, consumed by
exactly one caller (``buildThrow``'s ``torA[2]``), mirrored in
``sim/hand/trajectory.py``.
Firmware cross-reference (compiles and runs the shipped header):
``tests/firmware/test_hand_throw_decel_xref.py``.

WHAT THIS FILE IS FOR
---------------------
The hand made **light physical contact with its end stop** on ~1.2 m throws
during the 2026-07-27 validation sitting
(``logbook/2026-07-28-anomaly-fixes-validation-sitting.md``).  The commanded
profile never left ``x3 = 9.9594`` rev; the excess was position-loop coast, and
the root cause is that the braking torque feedforward was sized from a
hand-mass-on-a-spool model that omits the motor's rotor.

Three properties are asserted here, and each of them is what stops a *different*
way of getting this wrong:

1. **The declared inertia is at or below the measured one.**  This is the
   one-sided-safety half of the contract: the feedforward alone produces
   ``a_cmd * J_ff/J_true``, so while ``J_ff <= J_true`` it can only reduce the
   overshoot and can never brake the hand to a stop SHORT of ``x3`` and then
   drag it back up — which would re-create the operator-visible dip that
   Phases 1-4 of ``plans/active/hand-command-continuity.md`` removed.
2. **The correction is scoped to the decel segment.**  Correcting the ASCENT
   feedforward would raise the achieved release velocity and re-calibrate every
   throw height the machine has ever flown.
3. **The commanded position and velocity streams did not move.**  C-HAND-1's
   stroke-busy window, ``MIN_THROW_EVENT_DELAY_S``, ``_PRIME_INFLIGHT_S`` and
   ``tools/probes/hand_stroke_timeline.py``'s whole verdict model are all sized
   on the commanded profile.  A torque-only change leaves every one of them
   valid *without moving*; a profile-shaping change would not have.

EMPIRICAL ANCHORS — every number below traces to a measurement
-------------------------------------------------------------
Probe: ``tools/probes/hand_decel_authority.py``, run against
``temp/logs/toss_trace_2026-07-27_15-39-50.jsonl`` (the sitting's own capture).

* reflected inertia, DECEL-side torque balance (binding) . >= 1.0126e-5 kg m^2
* reflected inertia, decel regression over four tiers .... 1.050e-5 kg m^2
* geometric load-only floor (a hard lower bound) ......... 7.120e-6 kg m^2
* whole-session max |iq| ................................. 25.67 A (limit 50.0)

The *lower* of the two post-release numbers is what the safety bound uses,
because a lower ``J_true`` is the case in which a given ``J_ff`` is closest to
over-braking.

WHY THE DECEL-SIDE BOUND AND NOT THE ACCEL ONE
----------------------------------------------
An earlier draft anchored on an ACCEL-phase torque balance (1.015e-5).  That is
not a post-release inertia: release is at ``x2``, the end of the velocity hold,
so the ball is in the cup for the whole ascent.  ``INERTIA_RATIO = 0.747`` is
``m_hand/(m_hand+m_ball)`` (it is what makes ``throwD = -throwA/IR`` a
constant-motor-torque design), so ``m_ball = 0.0952 kg`` reflects
``2.412e-6 kg m^2`` — 24 % of that figure.  Ball-corrected it gives **7.74e-6**,
BELOW the declared value, i.e. read as a decel identification it would say the
shipped feedforward over-brakes.  It is refuted in that direction too: at
``J_decel = 7.74e-6`` the legacy feedforward would have delivered 95 % of the
commanded decel at every tier, so eta would be flat ~0.95, where 0.799 was
measured at 4.858 m/s.  The accel method simply is not measuring this quantity.

The decel-side bound is a genuine BOUND rather than a fit:
``J >= (tau_FF_wire + tau_grav) / (2*pi * a_achieved)``, valid because the hand
is AHEAD of ``pos_cmd`` through the whole overshoot (so the position loop brakes
too, ``tau_loop >= 0``) and friction only adds.  It uses no ``iq`` measurement,
so the telemetry aliasing cannot corrupt it.  Printed per tier by
``tools/probes/hand_decel_authority.py``.
"""

from __future__ import annotations

import math

import pytest

from hand import trajectory as mirror
import hardware_config as hw
from jugglebot.toss_sequencer import FLIGHT_TIME_MIN_S, FLIGHT_TIME_MAX_S

# ── measured anchors (see the module docstring for provenance) ──────────────
#: Binding DECEL-SIDE (post-release) lower bound on the hand axis's total
#: reflected inertia at the motor: the largest of the four per-tier bounds
#: ``(tau_FF_wire + tau_grav)/(2*pi*a_achieved)``, from the 4.858 m/s tier that
#: touched the end stop.  The declared feedforward inertia must not exceed it —
#: see ``test_declared_inertia_cannot_over_brake``.  Tightened 2026-07-29 from a
#: ball-inclusive ACCEL-phase figure of 1.015e-5; see the module docstring.
_MEASURED_REFLECTED_INERTIA_MIN_KGM2 = 1.0126e-5
#: Upper identification.  Used only to state the pessimistic peak bracket.
_MEASURED_REFLECTED_INERTIA_MAX_KGM2 = 1.050e-5
#: Measured ball-free quiescent hold torque of the hand axis (1.50 A x Kt).  On
#: an UPWARD deceleration gravity brakes in the SAME direction as the
#: feedforward, so it belongs in any statement about over-braking.
_GRAVITY_HOLD_NM = 1.50 * 0.00551333324983716
#: The Teensy's builder clamp on release speed.  NOT an operating point — the
#: host's flight-time band is far below it; see
#: ``test_the_feedforward_is_only_sized_over_the_flight_band``.
_MAX_EVENT_VEL_MPS = 7.0
#: The hand ODrive's flashed ``torque_constant`` (Nm/A) —
#: ``config/ODrive config Files/odrive_pro_hand_config.json``.  TEN TIMES
#: smaller than the legs' (this is a ~1500 Kv motor), which is why the hand's
#: current numbers look nothing like the legs'.
_HAND_KT_NM_PER_A = 0.00551333324983716
#: Runbook § CHECK HAND-4/HAND-7 hard-abort line on measured hand position.
_RUNBOOK_PEAK_ABORT_REV = 10.60

_GRAVITY = 9.81


def _v_for_flight(T: float) -> float:
    """Commanded release speed for a flight time (ballistic, up and back)."""
    return _GRAVITY * T / 2.0


def _band():
    return (FLIGHT_TIME_MIN_S, 0.80, FLIGHT_TIME_MAX_S)


# ══════════════════════════════════════════════════════════════════════════
#  1. what the historical expression actually meant
# ══════════════════════════════════════════════════════════════════════════

def test_legacy_conversion_implies_a_reflected_inertia_below_the_load_alone():
    """``a*m*r`` implies 7.3695e-6 kg m^2 — and that is the whole defect.

    Two errors compound: the rotor is missing entirely, and the RAW spool
    radius is used where the effective radius (one motor rev moves
    ``2*pi*r/LINEAR_GAIN_FACTOR`` of cable) belongs.  The second error is small
    and in the *helpful* direction, which is exactly why it hid the first.
    """
    implied = mirror.LEGACY_IMPLIED_INERTIA_KGM2
    assert implied == pytest.approx(7.3695e-6, rel=1e-4)

    # Re-derived independently of the mirror's own expression.
    m = hw.TEENSY_TRAJ_INERTIA_HAND_ONLY_KG
    r = hw.TEENSY_TRAJ_HAND_SPOOL_RADIUS_M
    gain = hw.TEENSY_TRAJ_LINEAR_GAIN_FACTOR / (math.pi * r * 2.0)
    assert implied == pytest.approx(m * r / (2.0 * math.pi * gain), rel=1e-12)

    # The load alone, with the EFFECTIVE radius.  The legacy value sits 3.5 %
    # above it — that is the radius error, and nothing else.
    r_eff = 1.0 / (2.0 * math.pi * gain)
    j_load = m * r_eff ** 2
    assert j_load == pytest.approx(7.120e-6, rel=1e-3)
    assert implied / j_load == pytest.approx(
        hw.TEENSY_TRAJ_LINEAR_GAIN_FACTOR, rel=1e-9)


# ══════════════════════════════════════════════════════════════════════════
#  2. the contract's two halves
# ══════════════════════════════════════════════════════════════════════════

def test_declared_inertia_cannot_over_brake():
    """C-HAND-2's one-sided-safety obligation: ``J_ff <= J_true``.

    Failure mode this closes: a feedforward sized ABOVE the true reflected
    inertia decelerates the hand harder than the commanded profile, so the hand
    stops short of ``x3`` and the position loop then drags it back up.  That is
    a commanded-profile dip — indistinguishable at the bench from the
    queue-clobber dip C-HAND-1 exists to detect, because
    ``tools/probes/hand_stroke_timeline.py``'s ``dip_below_x3`` row scores the
    hand's position, not its cause.
    """
    declared = hw.TEENSY_TRAJ_THROW_DECEL_REFLECTED_INERTIA_KGM2
    assert declared <= _MEASURED_REFLECTED_INERTIA_MIN_KGM2, (
        'declared feedforward inertia exceeds the LOWER measured '
        'identification — the feedforward can now over-brake')
    # ...and it is a deliberate under-estimate, not a coincidence: at least 5 %.
    assert declared <= 0.95 * _MEASURED_REFLECTED_INERTIA_MIN_KGM2


def test_declared_inertia_is_above_the_legacy_implied_one():
    """...otherwise the change is a no-op (or a regression).

    Failure mode this closes: a well-meaning edit that "simplifies" the
    declared inertia back to the load-only geometric value, which would restore
    the ~30 % under-torque this contract exists to remove while leaving every
    other test green.
    """
    declared = hw.TEENSY_TRAJ_THROW_DECEL_REFLECTED_INERTIA_KGM2
    assert declared > mirror.LEGACY_IMPLIED_INERTIA_KGM2
    boost = declared / mirror.LEGACY_IMPLIED_INERTIA_KGM2
    assert boost == pytest.approx(1.2891, rel=1e-3)


def test_the_declared_inertia_reaches_the_mirror_and_the_shipped_firmware():
    """Three routes to the same number: YAML codegen, mirror, shipped header.

    The header route is the only one that survives a hand-edit of
    ``Teensy_code/hardware_config.h`` that bypasses ``generate_config.py`` — and
    that header is what actually gets flashed.
    """
    import os
    import re
    declared = hw.TEENSY_TRAJ_THROW_DECEL_REFLECTED_INERTIA_KGM2
    assert mirror.THROW_DECEL_REFLECTED_INERTIA_KGM2 == pytest.approx(
        declared, rel=1e-12)

    repo = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))))
    header = os.path.join(repo, 'ros_ws', 'src', 'jugglebot', 'Teensy_code',
                          'hardware_config.h')
    with open(header) as fh:
        src = fh.read()
    block = re.search(r'namespace TeensyTraj\s*\{(.*?)\n\}', src, re.S)
    assert block, 'TeensyTraj namespace not found in the shipped header'
    hit = re.search(r'THROW_DECEL_REFLECTED_INERTIA_KGM2\s*=\s*([0-9eE.+-]+)f',
                    block.group(1))
    assert hit, 'the shipped firmware header does not declare the constant'
    assert float(hit.group(1)) == pytest.approx(declared, rel=1e-9)


# ══════════════════════════════════════════════════════════════════════════
#  3. scoping — only the decel segment moved
# ══════════════════════════════════════════════════════════════════════════

@pytest.mark.parametrize('flight_s', _band())
def test_only_the_decel_segment_uses_the_corrected_conversion(flight_s):
    """Accel on the legacy conversion, hold at zero, decel on the corrected one.

    Failure mode this closes: applying the corrected inertia to the ASCENT.
    That makes the hand track the accel ramp better and therefore raises the
    achieved release velocity — silently re-calibrating every throw height on a
    machine whose hand already reaches its end stop.
    """
    thr = mirror.HandThrowTrajectory(_v_for_flight(flight_s))
    a_acc = thr._throwA           # m/s^2, positive
    a_dec = thr._throwD           # m/s^2, negative

    assert thr.accel_torque_nm == pytest.approx(
        a_acc * mirror.INERTIA_HAND_ONLY_KG * mirror.HAND_SPOOL_RADIUS_M,
        rel=1e-12)
    assert thr.decel_torque_nm == pytest.approx(
        a_dec * mirror.THROW_DECEL_REFLECTED_INERTIA_KGM2
        * 2.0 * math.pi * mirror._LINEAR_GAIN, rel=1e-12)

    # sampled through the profile: +, 0, - and zero outside
    t_rel_start = thr.start_time + thr._t_acc              # end of accel
    assert thr.torque_at(thr.start_time - 1e-3) == 0.0
    assert thr.torque_at(thr.start_time + 0.5 * thr._t_acc) == pytest.approx(
        thr.accel_torque_nm)
    assert thr.torque_at(t_rel_start + 0.5 * thr._t_vel) == 0.0
    assert thr.torque_at(0.5 * thr._t_dec) == pytest.approx(thr.decel_torque_nm)
    assert thr.torque_at(thr.end_time + 1e-3) == 0.0

    assert thr.decel_torque_nm < 0.0 < thr.accel_torque_nm


@pytest.mark.parametrize('flight_s', _band())
def test_the_commanded_position_and_velocity_streams_did_not_move(flight_s):
    """The change is torque-only — re-derived here, not read off the mirror.

    Failure mode this closes: a future "improvement" that reaches the same
    braking by reshaping the profile.  Every host window that C-HAND-1 and
    Phase 3 sized (``stroke_clear_time``, ``MIN_THROW_EVENT_DELAY_S``,
    ``_PRIME_INFLIGHT_S``, ``hand_catch_prime_rev``) is derived from THESE
    streams, and the timeline probe's whole verdict model is calibrated on
    them.  If they move, all of that has to move in the same commit.
    """
    v = _v_for_flight(flight_s)
    thr = mirror.HandThrowTrajectory(v)

    ir = mirror.INERTIA_RATIO
    total = mirror.HAND_STROKE_M - 2.0 * mirror.STROKE_MARGIN_M
    vel_hold = mirror.THROW_VEL_HOLD_PCT * total
    accel_st = total - vel_hold
    t_acc = 2.0 / (ir + 1.0) * accel_st / v
    t_vel = vel_hold / v
    t_dec = t_acc * ir
    a_acc = v / t_acc
    a_dec = -a_acc / ir
    x1 = 0.5 * a_acc * t_acc ** 2
    x2 = x1 + v * t_vel
    x3 = x2 + v * t_dec + 0.5 * a_dec * t_dec ** 2

    assert thr._t_acc == pytest.approx(t_acc, rel=1e-12)
    assert thr._t_vel == pytest.approx(t_vel, rel=1e-12)
    assert thr._t_dec == pytest.approx(t_dec, rel=1e-12)
    assert x3 == pytest.approx(total, rel=1e-12)     # ends at the stroke top
    assert mirror.mm_to_rev(thr.end_pos_mm - thr.start_pos_mm) == pytest.approx(
        hw.HAND_STROKE_TOP_REV, rel=1e-9)

    for frac in (0.0, 0.13, 0.37, 0.51, 0.76, 0.99):
        t = thr.start_time + frac * (thr.end_time - thr.start_time)
        tl = t - thr.start_time
        if tl <= t_acc:
            want_x, want_v = 0.5 * a_acc * tl ** 2, a_acc * tl
        elif tl <= t_acc + t_vel:
            tau = tl - t_acc
            want_x, want_v = x1 + v * tau, v
        else:
            tau = tl - t_acc - t_vel
            want_x = x2 + v * tau + 0.5 * a_dec * tau ** 2
            want_v = v + a_dec * tau
        assert thr.sample(t) - thr.start_pos_mm == pytest.approx(
            want_x * 1000.0, abs=1e-9)
        assert thr.velocity_at(t) == pytest.approx(want_v, abs=1e-9)


def test_the_catch_and_smooth_move_conversions_are_untouched():
    """kind-1 and every ``makeSmoothMove`` prelude keep ``accelToTorque``.

    Failure mode this closes: widening the corrected conversion to the
    smooth-move path, whose torque stream is separately pinned by
    ``tests/firmware/test_hand_smooth_move_xref.py``'s ``_K_TOR`` — and whose
    kind-3 form is the ONLY un-arm mechanism the Teensy offers.
    """
    k_legacy = mirror.INERTIA_HAND_ONLY_KG * mirror.HAND_SPOOL_RADIUS_M
    assert mirror.accel_to_torque_nm(1.0) == pytest.approx(k_legacy, rel=1e-12)
    assert mirror.accel_to_torque_nm(-3.5) == pytest.approx(
        -3.5 * k_legacy, rel=1e-12)
    # and the two conversions are genuinely different, or the scoping is moot
    assert mirror.throw_decel_to_torque_nm(1.0) != pytest.approx(
        k_legacy, rel=1e-6)


# ══════════════════════════════════════════════════════════════════════════
#  4. the geometry the whole contract rests on
# ══════════════════════════════════════════════════════════════════════════

def test_the_commanded_decel_distance_is_velocity_independent():
    """4.046 rev at every speed — the lever arm the tracking error acts on.

    ``0.5*v*t_dec`` with ``t_dec = IR*t_acc`` and ``t_acc = 2*accelSt/((1+IR)*v)``
    reduces to ``IR*accelSt/(1+IR)``: no ``v``.  So ``calcThrow`` allocates the
    exact ideal stopping distance with ZERO allowance for tracking error at
    every speed, ending on the top of the usable stroke — which is why a
    fractional tracking shortfall converts straight into end-stop travel:
    ``peak = x3 + 4.046*(1/eta - 1)``.
    """
    dists = []
    for flight_s in (FLIGHT_TIME_MIN_S, 0.7, 0.9, FLIGHT_TIME_MAX_S):
        v = _v_for_flight(flight_s)
        thr = mirror.HandThrowTrajectory(v)
        dists.append(0.5 * v * thr._t_dec * mirror._LINEAR_GAIN)
    assert max(dists) - min(dists) < 1e-9
    assert dists[0] == pytest.approx(4.046, abs=5e-3)


# ══════════════════════════════════════════════════════════════════════════
#  5. authority — the reason steepening was rejected
# ══════════════════════════════════════════════════════════════════════════

@pytest.mark.parametrize('flight_s', _band())
def test_peak_decel_feedforward_current_stays_inside_the_shipped_limit(flight_s):
    """The corrected feedforward must fit in ``hand_curr_limit_a`` with headroom.

    Failure mode this closes: a feedforward that saturates the drive leaves the
    position/velocity loop NO authority to correct anything on top of it, and
    raising ``hand_curr_limit_a`` is an operator decision, not an implementer's.
    Measured: 31.6 A at the 4.858 m/s tier that touched the end stop, 38.9 A at
    the shipped band ceiling, against a 50.0 A limit.
    """
    thr = mirror.HandThrowTrajectory(_v_for_flight(flight_s))
    amps = thr.peak_decel_current_a(_HAND_KT_NM_PER_A)
    assert amps <= 0.85 * hw.ODRIVE_HAND_CURR_LIMIT_A, (
        f'decel feedforward wants {amps:.1f} A of a '
        f'{hw.ODRIVE_HAND_CURR_LIMIT_A:.0f} A limit at T={flight_s} s')


def test_the_feedforward_is_only_sized_over_the_flight_band():
    """C-HAND-2 sizes the feedforward over the FLIGHT BAND, not the builder clamp.

    Failure mode this closes: reading ``MAX_EVENT_VEL_MPS = 7.0`` as an operating
    point.  ``mirror.HandThrowTrajectory(7.0)`` is a perfectly legal object — the
    builder clamps to that speed, not to ``FLIGHT_TIME_MAX_S`` — and at 7.0 m/s
    the corrected decel feedforward ALONE wants **65.5 A of a 50 A limit**, which
    is the exact saturation this contract's headroom test says it prevents.  (The
    legacy feedforward already wanted 50.8 A there, so the gap is pre-existing;
    the correction widens it by 29 %.)

    The only thing keeping callers inside the band is the host's
    ``toss_sequencer`` flight-time check, so the boundary is asserted here rather
    than assumed: inside the band the feedforward fits with headroom, and at the
    builder clamp it provably does not.  If a future change makes 7.0 m/s
    reachable, this test is where that has to be confronted.
    """
    v_ceiling = _v_for_flight(FLIGHT_TIME_MAX_S)
    assert v_ceiling < _MAX_EVENT_VEL_MPS, (
        'the flight band now reaches the builder clamp; the feedforward is not '
        'sized there')

    inside = mirror.HandThrowTrajectory(v_ceiling).peak_decel_current_a(
        _HAND_KT_NM_PER_A)
    at_clamp = mirror.HandThrowTrajectory(
        _MAX_EVENT_VEL_MPS).peak_decel_current_a(_HAND_KT_NM_PER_A)

    assert inside <= 0.85 * hw.ODRIVE_HAND_CURR_LIMIT_A
    # Recorded as an executable fact, not a wish: the clamp is out of contract.
    assert at_clamp > hw.ODRIVE_HAND_CURR_LIMIT_A, (
        f'{at_clamp:.1f} A at the {_MAX_EVENT_VEL_MPS} m/s builder clamp')


def test_the_band_ceiling_is_already_near_the_axis_torque_ceiling():
    """Why the commanded ramp cannot simply be steepened.

    At ``FLIGHT_TIME_MAX_S`` the profile's OWN commanded deceleration needs
    75-90 % of everything the axis can produce at ``hand_curr_limit_a``.  The
    steepest commandable ramp therefore buys ~15 % of decel distance, i.e. ~15 %
    of the overshoot — 1.02 -> 0.88 rev on the tier that touched the stop.  This
    test is the recorded reason the fork went to the feedforward instead, and it
    fails if a future config change opens (or closes) that door.
    """
    v = _v_for_flight(FLIGHT_TIME_MAX_S)
    thr = mirror.HandThrowTrajectory(v)
    a_cmd_rev = abs(thr._throwD) * mirror._LINEAR_GAIN
    assert a_cmd_rev == pytest.approx(3597.0, rel=0.02)

    for j_true in (_MEASURED_REFLECTED_INERTIA_MIN_KGM2,
                   _MEASURED_REFLECTED_INERTIA_MAX_KGM2):
        ceiling = (hw.ODRIVE_HAND_CURR_LIMIT_A * _HAND_KT_NM_PER_A
                   / (j_true * 2.0 * math.pi))
        assert 0.75 <= a_cmd_rev / ceiling <= 0.90


# ══════════════════════════════════════════════════════════════════════════
#  6. the requirement this phase was set
# ══════════════════════════════════════════════════════════════════════════

@pytest.mark.parametrize('flight_s', _band())
def test_pessimistic_peak_bracket_clears_the_runbook_abort_line(flight_s):
    """peak <= 10.60 rev across the WHOLE band, in the pessimistic bracket.

    Pessimistic = the feedforward is the only braking and the closed loop
    contributes nothing.  Then the achieved decel is ``a_cmd * J_ff/J_true`` and

        over = d_dec * (J_true/J_ff - 1)

    which is **velocity-independent**, because ``d_dec`` is.  0.267 rev at the
    lower bound, 0.426 at the upper identification — peak 10.226 / 10.385 rev.  That
    velocity-independence is the property that matters: it is what removes the
    superlinear growth (+0.074 -> +1.020 rev over 2.74 -> 4.86 m/s) that made
    the band ceiling unflyable.

    The bracket is genuinely pessimistic — evaluated on the PRE-change
    feedforward it predicts +1.72 rev at 4.86 m/s where +1.02 was measured, so
    the loop closes ~40 % of it in practice.  The bench ladder
    (``tests/hardware/session_anomaly_fixes.md`` § CHECK HAND-7) is what turns
    the bracket into a measurement; nothing here has run on hardware.

    WHAT THIS ASSERTION DOES **NOT** COVER — release-speed scatter
    -------------------------------------------------------------
    Everything here is at the COMMANDED release speed.  The overshoot goes as
    ``v^2``, so with a release-speed error ``eps`` the bracket is
    ``d_dec * ((1+eps)^2 * J_true/J_ff - 1)``.  Measured ``v_pk`` against the
    commanded 153.4 rev/s at the ~1.2 m tier was +1.8 / -7.1 / +1.9 / -0.3 /
    -4.0 %, so at ``eps = +1.9 %`` the pessimistic peak is 10.56 rev and at
    ``eps = +2.6 %`` it reaches the 10.60 line.  That is why runbook rung R5
    carries a ``v_pk`` cross-check: a DEBRIEF-band peak there may be scatter
    rather than a feedforward failure, and the two have opposite responses.
    Deliberately NOT folded into this assertion — ``eps`` is a plant property,
    not a config one, and this test guards the config.
    """
    thr = mirror.HandThrowTrajectory(_v_for_flight(flight_s))
    d_dec = 0.5 * thr._throw_speed_mps * thr._t_dec * mirror._LINEAR_GAIN
    j_ff = hw.TEENSY_TRAJ_THROW_DECEL_REFLECTED_INERTIA_KGM2
    x3 = hw.HAND_STROKE_TOP_REV

    worst = x3 + d_dec * (_MEASURED_REFLECTED_INERTIA_MAX_KGM2 / j_ff - 1.0)
    assert worst <= _RUNBOOK_PEAK_ABORT_REV, (
        f'pessimistic peak {worst:.3f} rev exceeds the {_RUNBOOK_PEAK_ABORT_REV} '
        'rev runbook abort line')
    # ...with a stated margin, so a later inertia edit that eats it fails here.
    assert _RUNBOOK_PEAK_ABORT_REV - worst >= 0.15


def test_the_pessimistic_bracket_reproduces_the_pre_change_overshoot_order():
    """Sanity floor on the bracket: it must not be optimistic about the past.

    Evaluated with the LEGACY implied inertia it must predict *at least* the
    +1.020 rev that was actually measured at 4.858 m/s — a bracket that
    under-predicts a measured overshoot is not a bound, it is a wish.
    """
    thr = mirror.HandThrowTrajectory(4.858)
    d_dec = 0.5 * 4.858 * thr._t_dec * mirror._LINEAR_GAIN
    over = d_dec * (_MEASURED_REFLECTED_INERTIA_MAX_KGM2
                    / mirror.LEGACY_IMPLIED_INERTIA_KGM2 - 1.0)
    assert over >= 1.020
