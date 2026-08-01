"""Leg torque feedforward — sign, magnitude, units, clamp, ramp, and the OFF path.

This feature drives a 1.2 kg platform on six legs with an OPEN-LOOP torque. A sign
error does not degrade gracefully — it commands the legs to help gravity instead of
fight it, and the position loop then has to overcome twice the load. So the sign is
pinned here by a *physical* assertion, not by a golden vector, and the pin is
reproduced independently of the code path under test wherever that is possible.

The chain these tests cover, end to end:

    LegTorqueFeedforward  → TRUE Nm, extension-positive
      → KnotEmitter frame['torque_Nm']  (:5557)
      → SetpointPump      → clamp (TRUE Nm) → ramp → × Kt wire scale → ODrive-Nm
      → UDP Setpoint.torque_ff
      → firmware encode_leg_setpoint: leg_sign (NEGATE) → ×10000 → int16
      → ODrive: iq = input_torque / torque_constant

The firmware half is not importable from Python, so the two firmware constants that
close the loop (``leg_sign`` = negate-for-legs, ``LEG_TOR_SCALE`` = 10000) are
re-implemented here in ``_firmware_encode_torque`` and pinned against their sources.
"""

from __future__ import annotations

import json
import math
import os

import numpy as np
import pytest
import yaml

import jugglebot.hardware_config as hw
from jugglebot.motion.conversions import leg_forces_to_motor_torques
from jugglebot.motion.dynamics import DynamicsParams
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.ik_solver import (
    compute_jacobian,
    compute_leg_vectors,
    rotvec_to_rot_matrix,
)
from jugglebot.motion.torque_ff import LegTorqueFeedforward
from jugglebot.motion.trajectory import HoldPlan, KnotEmitter

from teensy_link.setpoint_pump import (
    DEFAULT_TORQUE_FF_MAX_NM,
    DEFAULT_TORQUE_WIRE_SCALE,
    SetpointPump,
)

import protocol_config as pc

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))

ACTIVE_POSE = np.array([0.0, 0.0, float(hw.JB_OP_DEFAULT_ACTIVE_Z_MM), 0.0, 0.0, 0.0])
ZERO6 = np.zeros(6)

# The ODrive's flashed torque_constant — the number the drive DIVIDES BY.
KT_ODRIVE = float(hw.DYNAMICS_MOTOR_KT_ODRIVE_CONFIG_NM_PER_A)
# The motor's bench-measured torque constant — the number physics OBEYS.
KT_TRUE = float(hw.DYNAMICS_MOTOR_KT_NM_PER_A)


# ── helpers ────────────────────────────────────────────────────────────────

def _geom():
    return StewartGeometry()


def _ff(**kw):
    kw.setdefault('enabled', True)
    kw.setdefault('gravity', True)
    kw.setdefault('platform_inertia', False)
    return LegTorqueFeedforward(_geom(), **kw)


def _pump(**kw):
    kw.setdefault('mm_to_rev', hw.GEOM_MM_TO_REV)
    kw.setdefault('max_step_rev', hw.JB_OP_MAX_POSITION_STEP_REV)
    return SetpointPump(**kw)


def _pump_ff(ramp_frames: int = 0, **kw):
    """A pump with the feedforward ON (production constants, ramp off by default)."""
    kw.setdefault('torque_ff_enabled', True)
    kw.setdefault('torque_ff_max_nm', float(hw.DYNAMICS_TORQUE_FF_MAX_NM))
    kw.setdefault('torque_wire_scale', float(hw.ODRIVE_LEG_TORQUE_WIRE_SCALE))
    kw.setdefault('torque_ff_ramp_frames', ramp_frames)
    return _pump(**kw)


def _cmd(torque_Nm=None, u0_scale: float = 1.0):
    """A minimal, pump-acceptable mpc_cmd dict at the active pose."""
    ext = np.asarray(hw.JB_OP_ACTIVATE_POSITION_REVS) / np.asarray(hw.GEOM_MM_TO_REV)
    msg = {
        'type': 'mpc_cmd',
        'ext_mm': list(ext),
        'motor_rev': [float(r) * u0_scale for r in hw.JB_OP_ACTIVATE_POSITION_REVS],
        'vel_mm_s': [0.0] * 6,
        'seq': 1,
    }
    if torque_Nm is not None:
        msg['torque_Nm'] = list(torque_Nm)
    return msg


def _cmd_raw_torque(bad):
    """A pump-acceptable cmd whose ``torque_Nm`` is deliberately malformed.

    Separate from ``_cmd`` because ``_cmd`` coerces via ``list()`` — which a scalar or a
    string would either reject or silently reshape. This one sets the field verbatim.
    """
    msg = _cmd()
    msg['torque_Nm'] = bad
    return msg


def _firmware_encode_torque(axis: int, torque_ff_wire: float) -> int:
    """Re-implementation of the can-bridge's torque encoding, for sign pinning.

    Mirrors ``odrive_protocol.h::encode_leg_setpoint`` exactly:

        float tff = leg_sign(axis, torque_ff_Nm);     // legs: NEGATE
        long  ti  = lroundf(tff * LEG_TOR_SCALE);     // 10000 counts/Nm
        clamp ti to int16

    Returns the int16 the ODrive receives in ``Set_Input_Pos.Torque_FF``.
    """
    sign = -1.0 if 0 <= axis <= 5 else 1.0     # is_leg(axis) → negate
    ti = int(round(sign * torque_ff_wire * float(pc.INPUT_SCALE_LEG_TOR)))
    return max(-32768, min(32767, ti))


# ═══════════════════════════════════════════════════════════════════════════
# 1. SIGN — the assertion this whole file exists for
# ═══════════════════════════════════════════════════════════════════════════

def test_gravity_ff_sign_holds_platform_up():
    """PHYSICAL FACT PINNED: at a level pose, EVERY leg's gravity feedforward is POSITIVE.

    Why that is the correct sign, from first principles and independent of any code:

      * The platform hangs on six legs. Gravity pulls it DOWN.
      * On the Jugglebot side of the firmware's ``leg_sign``, positive is the
        EXTENSION direction: a positive leg force acts along +extension, and
        ``leg_forces_to_motor_torques`` is a multiply by the (positive) spool radius,
        so a positive leg force gives a positive motor torque.
      * Extending all six legs RAISES the platform.
      * Therefore the torque that HOLDS THE PLATFORM UP against gravity is POSITIVE
        on every leg. There is no level pose at which a leg must push the platform
        down to hold it against gravity.

    If this test ever fails with all-negative torques, the feedforward is commanding
    the legs to ACCELERATE the platform downward at ~2 g while the position loop
    fights it. Do not "fix" it by flipping a sign somewhere downstream — find which
    of the three sign carriers (``-W_gravity``, ``solve(J.T, W)``, the spool radius)
    changed.
    """
    ff = _ff()
    rot = rotvec_to_rot_matrix(ACTIVE_POSE[3:6])
    tau = ff.compute(ACTIVE_POSE[:3], rot, ZERO6, ZERO6)

    assert np.all(tau > 0.0), (
        f'gravity feedforward must be extension-positive on every leg; got {tau}. '
        'A negative value here means the FF is helping gravity pull the platform DOWN.')


@pytest.mark.parametrize('z', [0.0, 50.0, 120.0, 170.0, 220.0])
def test_gravity_ff_sign_is_positive_across_the_z_range(z):
    """The sign fact is a property of the pose class, not of one lucky height."""
    ff = _ff()
    pose = np.array([0.0, 0.0, z, 0.0, 0.0, 0.0])
    rot = rotvec_to_rot_matrix(pose[3:6])
    tau = ff.compute(pose[:3], rot, ZERO6, ZERO6)
    assert np.all(tau > 0.0), f'z={z}: expected all-positive FF, got {tau}'


def test_gravity_ff_legs_actually_support_the_weight():
    """The forces must RECONSTRUCT a +m·g support wrench from the raw leg geometry.

    This is the non-tautological guard against the classic ``f = J.T @ W`` blunder
    (the force map is ``f = J^{-T}·W``, i.e. ``np.linalg.solve(J.T, W)``). We take the
    per-leg forces the FF implies, resolve each along its leg's unit vector using
    ``compute_leg_vectors`` — geometry, NOT the Jacobian — and sum. The resulting
    force on the platform must be +m·g upward, i.e. exactly cancelling gravity.

    With ``J.T @ W`` the reconstruction is off by the (badly conditioned) J·J^T and
    this fails by orders of magnitude.
    """
    geom = _geom()
    params = DynamicsParams.from_config()
    ff = _ff()
    rot = rotvec_to_rot_matrix(ACTIVE_POSE[3:6])
    pos = ACTIVE_POSE[:3]

    tau = ff.compute(pos, rot, ZERO6, ZERO6)
    # motor torque (Nm) → leg force (N): inverse of the spool-radius scale.
    f_legs = tau / (geom.spool_radius_mm / 1000.0)

    leg_vecs = compute_leg_vectors(pos, rot, geom)
    unit = leg_vecs / np.linalg.norm(leg_vecs, axis=1, keepdims=True)

    # Total force the six legs apply to the platform (N).
    F_total = (f_legs[:, None] * unit).sum(axis=0)
    weight_N = params.mass_kg * params.gravity_mps2

    assert F_total[2] == pytest.approx(weight_N, rel=1e-9), (
        f'legs must lift exactly the platform weight ({weight_N:.4f} N); '
        f'reconstructed Fz = {F_total[2]:.4f} N')
    assert F_total[0] == pytest.approx(0.0, abs=1e-9)
    assert F_total[1] == pytest.approx(0.0, abs=1e-9)


def test_wire_sign_reaches_the_odrive_as_platform_up():
    """END-TO-END SIGN: extension-positive on the wire → negative ODrive torque.

    The firmware negates for leg axes (``leg_sign``), and ODrive position is negated
    the same way (``clip_position`` clamps the Jugglebot setpoint to [0, max] and the
    caller negates), so ODrive-negative *is* leg-extension. A positive wire torque
    therefore arrives at the drive as a NEGATIVE ``Torque_FF``, which drives ODrive
    position negative, which extends the leg, which pushes the platform UP.

    This pins that the Jetson must NOT pre-negate — the firmware already does.
    """
    ff = _ff()
    rot = rotvec_to_rot_matrix(ACTIVE_POSE[3:6])
    tau = ff.compute(ACTIVE_POSE[:3], rot, ZERO6, ZERO6)

    pump = _pump_ff()
    sp, reason = pump.build(_cmd(torque_Nm=tau), 0)
    assert reason is None and sp is not None

    assert all(t > 0.0 for t in sp.torque_ff), (
        f'the UDP wire carries extension-positive torque; got {sp.torque_ff}')

    for axis in range(6):
        counts = _firmware_encode_torque(axis, sp.torque_ff[axis])
        assert counts < 0, (
            f'leg {axis}: the ODrive must receive a NEGATIVE Torque_FF (= extension = '
            f'platform up); got {counts} counts from wire {sp.torque_ff[axis]:.6f}')


# ═══════════════════════════════════════════════════════════════════════════
# 2. MAGNITUDE — is it the right size?
# ═══════════════════════════════════════════════════════════════════════════

def test_gravity_ff_magnitude_matches_the_documented_range():
    """0.013–0.041 Nm/leg at the active pose.

    Cross-checks two independent prior sources:
      * ``plans/archived/2026-05-08 friction-ff-motor-guard-integration.md:37`` —
        "0.013–0.041 Nm/leg, non-zero in 99.6 % of samples" (the MPC-path measurement).
      * ``docs/motion_planner/dynamics.md`` — "~0.018 Nm" per leg. That figure was
        computed at the OLD platform mass of 0.96 kg; at today's 1.2 kg it scales to
        ~0.023 Nm, which is where the mean lands.

    A feedforward an order of magnitude out would sail through every other test in
    this file (they are all sign/units/plumbing); this is the one that would catch a
    mass, spool-radius or mm↔m slip.
    """
    ff = _ff()
    rot = rotvec_to_rot_matrix(ACTIVE_POSE[3:6])
    tau = ff.compute(ACTIVE_POSE[:3], rot, ZERO6, ZERO6)

    assert tau.min() > 0.010, f'min leg torque {tau.min():.4f} Nm is below the documented band'
    assert tau.max() < 0.045, f'max leg torque {tau.max():.4f} Nm is above the documented band'
    assert 0.015 < tau.mean() < 0.035, (
        f'mean leg torque {tau.mean():.4f} Nm — expected ~0.023 Nm '
        '(0.018 Nm at the docs\' old 0.96 kg mass, scaled to today\'s 1.2 kg)')

    # And in the units that actually matter on the bench: amps.
    iq = tau / KT_TRUE
    assert iq.max() < 1.0, (
        f'gravity FF should be well under 1 A/leg (got {iq.max():.3f} A); it is '
        'smaller than the 1.09 A Coulomb friction floor, which is why gravity FF '
        'alone cannot make a held platform move.')


def test_gravity_ff_is_independent_of_velocity_and_acceleration():
    """Gravity-only mode must be a purely STATIC term.

    This is a safety property, not a nicety. The can-bridge holds the last
    ``torque_ff`` UNDECAYED through the stale-link extrapolation window
    (``leg_interp.cpp:366`` sits after the velocity-decay block). A static term is
    safe to hold — gravity is still there. An acceleration-proportional term is not.
    So with ``platform_inertia=False`` the FF must not depend on twist or accel at all.
    """
    ff = _ff(platform_inertia=False)
    rot = rotvec_to_rot_matrix(ACTIVE_POSE[3:6])
    quiet = ff.compute(ACTIVE_POSE[:3], rot, ZERO6, ZERO6)

    twist = np.array([200.0, -150.0, 100.0, 0.5, -0.3, 0.2])
    accel = np.array([3000.0, -2000.0, 4000.0, 8.0, -6.0, 4.0])
    moving = ff.compute(ACTIVE_POSE[:3], rot, twist, accel)

    assert np.allclose(quiet, moving, atol=0.0, rtol=0.0), (
        'gravity-only FF must be identical at rest and in motion')


def test_platform_inertia_term_adds_torque_when_accelerating_upward():
    """Accelerating the platform UP must ask the legs for MORE torque, not less.

    Sign pin for the optional inertia term: ``W_inertia`` is ADDED to the support
    wrench (``F = m·a_com``), so a positive +z acceleration increases every leg's
    torque above the pure-gravity value.
    """
    rot = rotvec_to_rot_matrix(ACTIVE_POSE[3:6])
    grav = _ff(platform_inertia=False).compute(ACTIVE_POSE[:3], rot, ZERO6, ZERO6)

    ff = _ff(platform_inertia=True)
    up = np.array([0.0, 0.0, 4000.0, 0.0, 0.0, 0.0])     # +4 m/s² (the hard ceiling)
    down = np.array([0.0, 0.0, -4000.0, 0.0, 0.0, 0.0])
    tau_up = ff.compute(ACTIVE_POSE[:3], rot, ZERO6, up)
    tau_down = ff.compute(ACTIVE_POSE[:3], rot, ZERO6, down)

    assert np.all(tau_up > grav), (
        f'accelerating up must need MORE leg torque than a static hold\n'
        f'  static: {grav}\n  +4 m/s²: {tau_up}')
    assert np.all(tau_down < grav), (
        f'accelerating down must need LESS leg torque than a static hold\n'
        f'  static: {grav}\n  -4 m/s²: {tau_down}')
    # a = 0 must reduce exactly to the gravity term (no spurious offset).
    assert np.allclose(ff.compute(ACTIVE_POSE[:3], rot, ZERO6, ZERO6), grav)


# ═══════════════════════════════════════════════════════════════════════════
# 3. FLAG OFF ⇒ EXACTLY ZEROS (byte-identical to the pre-feature behaviour)
# ═══════════════════════════════════════════════════════════════════════════

def test_shipped_config_has_the_feature_on():
    """The shipped default is ON since the 2026-07-16 hardware arming session.

    History: this test shipped as `test_shipped_config_has_the_feature_off` —
    the tripwire that made arming an operator bench action, not a config
    commit. The bench action happened: the operator ran the A/B arming
    sessions per tests/hardware/session_torque_ff.md on 2026-07-16 (sign
    verified NO INVERSION on hardware — per-leg hold-iq delta leg-mean
    -0.115 A, the ideal falls-or-flat signature; zero motion at the arm edge
    through the 2 s ramp; every setpoint carried FF; the firmware clamp never
    bound) and chose to ship the feature enabled. If this fails, someone
    DISARMED the gravity FF via a commit — that reopens the
    integrator-carries-gravity margin problem and must be a deliberate,
    logged decision (see logbook/2026-07-16-gravity-ff-armed.md), not a
    drive-by config edit."""
    assert hw.DYNAMICS_TORQUE_FF_ENABLED is True, (
        'dynamics.torque_ff_enabled shipped as true after the 2026-07-16 '
        'hardware arming session — turning it OFF is a deliberate, logged '
        'decision, not a drive-by config edit.')


def test_ff_disabled_returns_exact_zeros():
    ff = _ff(enabled=False)
    rot = rotvec_to_rot_matrix(ACTIVE_POSE[3:6])
    tau = ff.compute(ACTIVE_POSE[:3], rot, ZERO6,
                     np.array([1000.0, 0.0, 2000.0, 1.0, 0.0, 0.0]))
    assert not ff.active
    assert np.array_equal(tau, np.zeros(6))


def test_emitter_with_shipped_config_emits_live_gravity_torque():
    """The default-constructed emitter (production path) publishes the LIVE
    gravity feedforward since the shipped flag went true (2026-07-16): at the
    active pose, per-leg torques in the platform's known gravity range
    (0.013-0.041 Nm, all extension-positive — the legs support the platform).
    (Pre-arming this test pinned exact zeros; that behaviour is still covered
    for the flag-off path by test_ff_disabled_returns_exact_zeros and the
    byte-identical pump test below.)"""
    frame = KnotEmitter(_geom()).frame(HoldPlan(ACTIVE_POSE), 0.0, 0)
    tau = np.asarray(frame['torque_Nm'])
    assert tau.shape == (6,)
    assert np.all(tau > 0.0), f'gravity support must be extension-positive: {tau}'
    assert np.all(tau > 0.010) and np.all(tau < 0.045), (
        f'per-leg gravity FF at the active pose must sit in the known '
        f'0.013-0.041 Nm band: {tau}')


def test_pump_with_ff_off_is_byte_identical_to_the_pre_feature_frame():
    """OFF ⇒ the packed UDP bytes are exactly what they were before this feature.

    Same cmd, once carrying a live torque_Nm and once with the field absent (the
    pre-feature producer). With the FF off the pump must not even look at the field,
    so the two frames must pack to identical bytes.
    """
    tau = _ff().compute(ACTIVE_POSE[:3], rotvec_to_rot_matrix(ACTIVE_POSE[3:6]),
                        ZERO6, ZERO6)

    with_tq, r1 = _pump().build(_cmd(torque_Nm=tau), 12345)
    without_tq, r2 = _pump().build(_cmd(), 12345)

    assert r1 is None and r2 is None
    assert with_tq.torque_ff == (0.0,) * 6
    assert with_tq.pack() == without_tq.pack(), (
        'with the FF off, a frame carrying torque_Nm must pack byte-identically to '
        'one without it')


def test_pump_with_ff_off_ignores_a_malformed_torque_vector():
    """OFF must be inert even against garbage — no new reject path appears."""
    for bad in ([float('nan')] * 6, [1.0, 2.0], 'not-a-vector', [None] * 6, 3.0):
        sp, reason = _pump().build(_cmd_raw_torque(bad), 0)
        assert reason is None, f'FF off must not reject on torque_Nm={bad!r}'
        assert sp.torque_ff == (0.0,) * 6


# ═══════════════════════════════════════════════════════════════════════════
# 4. Kt SCALING — the whole point of the wire scale
# ═══════════════════════════════════════════════════════════════════════════

def test_wire_scale_delivers_the_physically_correct_current():
    """The round trip that justifies the entire prescale design.

    The drive computes ``iq = input_torque / torque_constant`` with its FLASHED
    ``torque_constant`` = 0.055133 (ODrive's uncalibrated 8.27/Kv nameplate default).
    The motor actually produces ``Kt_true × iq`` with Kt_true = 0.0624 Nm/A. So to
    deliver a TRUE torque T we must put ``T × (0.055133 / 0.0624)`` on the wire.

    Assert the loop closes: the current the drive ends up commanding equals the
    current physics needs, ``T / Kt_true``.
    """
    tau_true = np.array([0.02, 0.03, 0.04, 0.01, 0.025, 0.035])
    sp, reason = _pump_ff().build(_cmd(torque_Nm=tau_true), 0)
    assert reason is None

    wire = np.array(sp.torque_ff)
    assert np.allclose(wire, tau_true * (KT_ODRIVE / KT_TRUE))

    iq_delivered = wire / KT_ODRIVE          # what the ODrive firmware computes
    iq_wanted = tau_true / KT_TRUE           # what physics needs for tau_true
    assert np.allclose(iq_delivered, iq_wanted, rtol=1e-12), (
        'the wire scale must make the DELIVERED CURRENT correct — that is the only '
        'thing a torque feedforward needs')


def test_wire_scale_matches_the_generated_constant():
    assert hw.ODRIVE_LEG_TORQUE_WIRE_SCALE == pytest.approx(KT_ODRIVE / KT_TRUE, rel=1e-12)
    # A 13.2% mismatch is what this constant exists to absorb — sanity-bound it so a
    # future Kt re-measurement can't silently produce an absurd scale.
    assert 0.5 < hw.ODRIVE_LEG_TORQUE_WIRE_SCALE < 1.5


def test_pump_defaults_match_generated_config():
    """The pure module's standalone defaults must not drift from the generated config.

    ``SetpointPump`` deliberately has no ``jugglebot.hardware_config`` import (it is a
    pure module the bridge injects into). That purity costs a drift risk, which this
    test closes — exactly as ``DEFAULT_MAX_STEP_REV`` mirrors ``JB_OP_MAX_POSITION_STEP_REV``.
    """
    assert DEFAULT_TORQUE_WIRE_SCALE == pytest.approx(
        hw.ODRIVE_LEG_TORQUE_WIRE_SCALE, rel=1e-12)
    assert DEFAULT_TORQUE_FF_MAX_NM == pytest.approx(hw.DYNAMICS_TORQUE_FF_MAX_NM)


def test_yaml_kt_odrive_config_matches_the_flashed_odrive_json():
    """The YAML's copy of the drive's torque_constant must equal what is FLASHED.

    ``dynamics.motor_kt_odrive_config_nm_per_a`` is a MIRROR of
    ``config/ODrive config Files/odrive_pro_leg_config.json``
    → ``axis0.config.motor.torque_constant``. If they diverge, the wire scale is wrong
    and every leg feedforward is delivered at the wrong current — silently, because
    nothing in the system ever reads torque_constant back off a drive.
    """
    json_path = os.path.join(_REPO_ROOT, 'config', 'ODrive config Files',
                             'odrive_pro_leg_config.json')
    with open(json_path) as f:
        cfg = json.load(f)
    flashed = float(cfg['axis0']['config']['motor']['torque_constant'])

    yaml_path = os.path.join(_REPO_ROOT, 'config', 'hardware_config.yaml')
    with open(yaml_path) as f:
        yml = yaml.safe_load(f)
    mirrored = float(yml['dynamics']['motor_kt_odrive_config_nm_per_a'])

    assert mirrored == pytest.approx(flashed, rel=1e-12), (
        f'hardware_config.yaml says the drives carry torque_constant={mirrored} but '
        f'odrive_pro_leg_config.json flashes {flashed}')
    # It is the ODrive 8.27/Kv nameplate DEFAULT, not a measurement — and provably so:
    # the flashed value is bit-exactly float32(8.27 / 150), i.e. ODrive's documented
    # heuristic for a 150 Kv motor, stored in the drive's float32 config and never
    # touched (git confirms torque_constant was never edited in the leg config).
    # Pinning this stops anyone "reconciling" the 13 % gap by editing the YAML: this
    # number is a property of the DRIVE, and changing it here without re-flashing all
    # six drives (and rescaling vel_gain / vel_integrator_gain by Kt_true/Kt_config to
    # keep the validated velocity loop where it is) silently breaks the wire scale.
    assert flashed == float(np.float32(8.27 / 150.0)), (
        f'torque_constant={flashed!r} is no longer float32(8.27/150) — a drive was '
        're-flashed with a different torque_constant. STOP: the velocity loop\'s '
        'authority is vel_gain/torque_constant, so this also moved the shipping, '
        'bench-validated leg tune. See hardware_config.yaml:dynamics.')


def test_odrive_input_torque_scale_matches_the_firmware_wire_scale():
    """``input_torque_scale`` on the drive must equal the can-bridge's LEG_TOR_SCALE.

    The CAN frame carries ``Torque_FF`` as an int16 of counts, and the drive recovers
    ``input_torque = Torque_FF / input_torque_scale``. Our drives are flashed with
    10000 (NOT ODrive's default of 1000), and the bridge encodes with the same 10000
    (``canbridge_config.h`` ``LEG_TOR_SCALE = InputScale::leg_tor``). A drive restored
    to vendor defaults would use 1000 and deliver **10× the intended feedforward
    torque** — silently. This is the pin that makes that drift loud.
    """
    json_path = os.path.join(_REPO_ROOT, 'config', 'ODrive config Files',
                             'odrive_pro_leg_config.json')
    with open(json_path) as f:
        cfg = json.load(f)
    flashed = float(cfg['axis0']['config']['can']['input_torque_scale'])
    assert flashed == pytest.approx(float(pc.INPUT_SCALE_LEG_TOR)), (
        f'ODrive input_torque_scale={flashed} but the bridge encodes at '
        f'{pc.INPUT_SCALE_LEG_TOR} counts/Nm')


# ═══════════════════════════════════════════════════════════════════════════
# 5. int16 QUANTISATION AND RANGE
# ═══════════════════════════════════════════════════════════════════════════

def test_clamped_torque_always_fits_in_int16_on_the_wire():
    """The clamp must keep the encoded counts inside int16 with margin.

    Encoding saturates at ±32767 counts = ±3.2767 ODrive-Nm = ±59 A of demand
    (÷0.055133) — about 6× ``current_soft_max``. Saturation is therefore NOT a safety
    net; the Jetson clamp is. Verify the clamp bound lands far inside int16 so the
    firmware's saturation can never be the operative limit.
    """
    lim = float(hw.DYNAMICS_TORQUE_FF_MAX_NM)
    max_counts = lim * float(hw.ODRIVE_LEG_TORQUE_WIRE_SCALE) * float(pc.INPUT_SCALE_LEG_TOR)
    assert max_counts < 32767, 'the clamp must be reachable without int16 saturation'
    assert max_counts < 32767 / 10.0, (
        f'the clamp ({lim} Nm → {max_counts:.0f} counts) should sit an order of '
        'magnitude inside int16 — if it does not, the clamp is far too permissive')

    # And in amps: the clamp must leave the velocity loop real authority under the
    # 10 A current_soft_max (ODrive ADDS the FF to the loop output BEFORE the limit).
    iq_at_clamp = lim * float(hw.ODRIVE_LEG_TORQUE_WIRE_SCALE) / KT_ODRIVE
    assert iq_at_clamp < 5.0, (
        f'at the clamp the FF alone would draw {iq_at_clamp:.2f} A of the 10 A '
        'current_soft_max — that leaves too little for disturbance rejection')


def test_int16_quantisation_error_is_negligible_against_the_signal():
    """0.5-count rounding = 5e-5 ODrive-Nm ≈ 0.9 mA — four orders below the FF."""
    tau = _ff().compute(ACTIVE_POSE[:3], rotvec_to_rot_matrix(ACTIVE_POSE[3:6]),
                        ZERO6, ZERO6)
    sp, _ = _pump_ff().build(_cmd(torque_Nm=tau), 0)

    for axis in range(6):
        wire = sp.torque_ff[axis]
        counts = _firmware_encode_torque(axis, wire)
        # Recover what the drive will actually apply, back in TRUE Nm.
        recovered_odrive_nm = -counts / float(pc.INPUT_SCALE_LEG_TOR)   # undo leg_sign
        recovered_true_nm = recovered_odrive_nm / float(hw.ODRIVE_LEG_TORQUE_WIRE_SCALE)
        err_A = abs(recovered_true_nm - tau[axis]) / KT_TRUE
        assert err_A < 1e-3, (
            f'leg {axis}: quantisation costs {err_A * 1000:.3f} mA — expected < 1 mA')
        # Non-zero counts: the FF must survive quantisation at all (it does: ~100-350).
        assert abs(counts) > 50


# ═══════════════════════════════════════════════════════════════════════════
# 6. THE CLAMP — the only bound between a mis-modelled FF and open-loop runaway
# ═══════════════════════════════════════════════════════════════════════════

@pytest.mark.parametrize('bad', [5.0, 50.0, 3.2767, -5.0, -50.0])
def test_absurd_torque_is_clamped(bad):
    """A model blow-up must be bounded, not merely int16-saturated.

    Without this clamp a 3.2767 Nm wire value = 59 A of demand. The ODrive adds
    ``input_torque`` to the velocity loop's output BEFORE the torque limit, so the
    current clamp saturates and the position loop loses ALL authority — the leg runs
    open-loop at full current until MAX_DEVIATION E-STOPs it mid-motion.
    """
    lim = float(hw.DYNAMICS_TORQUE_FF_MAX_NM)
    scale = float(hw.ODRIVE_LEG_TORQUE_WIRE_SCALE)
    sp, reason = _pump_ff().build(_cmd(torque_Nm=[bad] * 6), 0)
    assert reason is None
    expected = math.copysign(lim, bad) * scale
    assert all(t == pytest.approx(expected) for t in sp.torque_ff), (
        f'torque_Nm={bad} must clamp to ±{lim} Nm true (±{expected:.5f} on the wire); '
        f'got {sp.torque_ff}')


def test_clamp_is_applied_per_leg_not_to_the_vector():
    """One bad leg must not scale down the other five (that would be a silent detune)."""
    lim = float(hw.DYNAMICS_TORQUE_FF_MAX_NM)
    scale = float(hw.ODRIVE_LEG_TORQUE_WIRE_SCALE)
    tau = [0.02, 99.0, 0.03, -99.0, 0.01, 0.04]
    sp, _ = _pump_ff().build(_cmd(torque_Nm=tau), 0)
    assert sp.torque_ff[0] == pytest.approx(0.02 * scale)
    assert sp.torque_ff[1] == pytest.approx(lim * scale)
    assert sp.torque_ff[2] == pytest.approx(0.03 * scale)
    assert sp.torque_ff[3] == pytest.approx(-lim * scale)
    assert sp.torque_ff[4] == pytest.approx(0.01 * scale)
    assert sp.torque_ff[5] == pytest.approx(0.04 * scale)


def test_gravity_ff_never_reaches_the_clamp_anywhere_in_the_reachable_workspace():
    """Headroom check: the clamp must be a backstop, not an operating limit.

    If the shipped clamp ever bit during normal motion it would silently distort the
    feedforward (and, worse, do so pose-dependently). Sweep a coarse grid of poses
    and assert the gravity FF stays comfortably under it.
    """
    ff = _ff()
    lim = float(hw.DYNAMICS_TORQUE_FF_MAX_NM)
    worst = 0.0
    for x in (-60.0, 0.0, 60.0):
        for y in (-60.0, 0.0, 60.0):
            for z in (0.0, 100.0, 170.0, 220.0):
                for tilt in (0.0, 0.15):
                    pose = np.array([x, y, z, tilt, -tilt, 0.0])
                    rot = rotvec_to_rot_matrix(pose[3:6])
                    tau = ff.compute(pose[:3], rot, ZERO6, ZERO6)
                    worst = max(worst, float(np.max(np.abs(tau))))
    assert worst < 0.5 * lim, (
        f'worst gravity FF over the pose sweep is {worst:.4f} Nm against a clamp of '
        f'{lim} Nm — less than 2x headroom means the clamp is likely to bite in normal use')


# ═══════════════════════════════════════════════════════════════════════════
# 7. THE RAMP — the integrator-transient hazard
# ═══════════════════════════════════════════════════════════════════════════

def test_ramp_starts_at_zero_and_climbs_monotonically_to_full():
    """THE HAZARD: when streaming starts, the ODrive's velocity INTEGRATOR is already
    carrying the full gravity load (it wound up during the activate hold). Adding the
    gravity feedforward as a step would briefly command ~2× gravity until the
    integrator unwinds — an unforced upward kick on every leg at the moment of arming.

    So: the FIRST frame must carry ZERO feedforward, and the FF must climb smoothly
    from there while the integrator bleeds off.
    """
    tau = [0.03] * 6
    pump = _pump_ff(ramp_frames=80)          # 2.0 s at 40 Hz
    scale = float(hw.ODRIVE_LEG_TORQUE_WIRE_SCALE)

    seen = []
    for _ in range(120):
        sp, reason = pump.build(_cmd(torque_Nm=tau), 0)
        assert reason is None
        seen.append(sp.torque_ff[0])

    assert seen[0] == 0.0, 'the first frame after arming must carry ZERO feedforward'
    assert all(b >= a for a, b in zip(seen, seen[1:])), 'the ramp must be monotonic'
    assert seen[80] == pytest.approx(0.03 * scale), 'full FF after ramp_frames'
    assert seen[-1] == pytest.approx(0.03 * scale)
    # No frame anywhere in the ramp exceeds the steady value.
    assert max(seen) == pytest.approx(0.03 * scale)
    # And the per-frame increment is small (1/80 of full ≈ 1.25 %).
    steps = [b - a for a, b in zip(seen, seen[1:81])]
    assert max(steps) == pytest.approx(0.03 * scale / 80.0, rel=1e-9)


def test_reset_restarts_the_ramp():
    """After a link loss the integrator has re-absorbed gravity on its own, so
    re-applying the full FF as a step would double-count it exactly as at first arm.
    ``reset()`` (called on re-arm / link re-establish) must therefore restart the ramp.
    """
    tau = [0.03] * 6
    pump = _pump_ff(ramp_frames=40)
    for _ in range(200):
        pump.build(_cmd(torque_Nm=tau), 0)
    assert pump.torque_ff_ramp_scale() == 1.0

    pump.reset()
    assert pump.torque_ff_ramp_scale() == 0.0
    sp, _ = pump.build(_cmd(torque_Nm=tau), 0)
    assert sp.torque_ff == (0.0,) * 6


def test_rejected_frames_do_not_advance_the_ramp():
    """The ramp counts ACCEPTED frames. A rejected frame commands nothing, so it must
    not buy ramp credit — otherwise a burst of rejects would let the FF snap to full
    the moment a good frame arrives."""
    tau = [0.03] * 6
    pump = _pump_ff(ramp_frames=40)
    pump.build(_cmd(torque_Nm=tau), 0)                       # accepted → _ff_frames 1
    before = pump.torque_ff_ramp_scale()

    # A step-gate violation (u0 jumps far beyond max_step_rev) → reject.
    sp, reason = pump.build(_cmd(torque_Nm=tau, u0_scale=5.0), 0)
    assert sp is None and reason is not None
    assert pump.torque_ff_ramp_scale() == before


def test_a_frame_without_feedforward_restarts_the_ramp():
    """A producer that streams position-only and THEN starts feeding forward must be
    ramped, not stepped.

    While no feedforward was arriving, the ODrive's velocity integrator was winding up
    to carry gravity on its own — so the moment feedforward starts is, physically, a
    first-arm all over again. Without this, a restarted emitter (or a source swap)
    would snap the FF straight to full: the exact transient the ramp exists to prevent,
    reintroduced through the back door.
    """
    tau = [0.03] * 6
    pump = _pump_ff(ramp_frames=40)

    for _ in range(100):                       # ramp reaches full
        pump.build(_cmd(torque_Nm=tau), 0)
    assert pump.torque_ff_ramp_scale() == 1.0

    sp, reason = pump.build(_cmd(), 0)         # a frame with NO torque_Nm
    assert reason is None
    assert sp.torque_ff == (0.0,) * 6
    assert pump.torque_ff_ramp_scale() == 0.0, 'the ramp must restart'

    sp, _ = pump.build(_cmd(torque_Nm=tau), 0)  # feedforward resumes
    assert sp.torque_ff == (0.0,) * 6, 'and the resumed FF must start from zero again'


def test_ramp_disabled_gives_full_ff_on_the_first_frame():
    """ramp_frames=0 is the no-ramp escape hatch (tests / a deliberate operator choice)."""
    scale = float(hw.ODRIVE_LEG_TORQUE_WIRE_SCALE)
    sp, _ = _pump_ff(ramp_frames=0).build(_cmd(torque_Nm=[0.02] * 6), 0)
    assert sp.torque_ff[0] == pytest.approx(0.02 * scale)


# ═══════════════════════════════════════════════════════════════════════════
# 8. MALFORMED INPUT WITH THE FF ON
# ═══════════════════════════════════════════════════════════════════════════

@pytest.mark.parametrize('bad,frag', [
    ([float('nan')] * 6, 'non-finite'),
    ([float('inf')] * 6, 'non-finite'),
    ([0.01] * 5, 'wrong-length'),
    ([0.01] * 7, 'wrong-length'),
    (['x'] * 6, 'non-numeric'),
    (3.0, 'not a sequence'),
])
def test_malformed_torque_with_ff_on_is_a_safety_reject(bad, frag):
    """With the FF live, a malformed torque vector must REJECT the frame, not silently
    zero the feedforward.

    Silently zeroing would manufacture a step discontinuity in an open-loop torque
    injected into a closed loop — precisely the class of transient that bootstrapped
    the 2026-05-08 5 Hz platform limit cycle. Rejecting instead leaves the firmware
    holding the last good setpoint AND its torque (continuous), and if the condition
    persists the 250 ms MPC-staleness E-STOP fires. Loud beats quiet.
    """
    pump = _pump_ff()
    sp, reason = pump.build(_cmd_raw_torque(bad), 0)
    assert sp is None
    assert reason is not None and frag in reason
    assert pump.frames_rejected == 1


def test_missing_torque_with_ff_on_is_not_a_reject():
    """A producer that simply does no feedforward (a bench source) is not a fault."""
    pump = _pump_ff()
    sp, reason = pump.build(_cmd(), 0)
    assert reason is None
    assert sp.torque_ff == (0.0,) * 6
    assert pump.frames_without_ff == 1
    assert pump.frames_rejected == 0


def test_singular_jacobian_yields_zero_ff_not_a_crash():
    """A degenerate pose must degrade to no-feedforward, never to a NaN on the wire.

    A NaN in torque_ff makes the firmware drop the WHOLE setpoint frame
    (``leg_interp.cpp:164`` validates isfinite) — i.e. a lost position command, which
    is strictly worse than a lost feedforward.
    """
    geom = _geom()
    ff = _ff()
    rot = rotvec_to_rot_matrix(ACTIVE_POSE[3:6])
    J_singular = np.zeros((6, 6))
    tau = ff.compute(ACTIVE_POSE[:3], rot, ZERO6, ZERO6, J=J_singular)
    assert np.array_equal(tau, np.zeros(6))
    assert np.all(np.isfinite(tau))


# ═══════════════════════════════════════════════════════════════════════════
# 9. THE CROSS-CUTTING INVARIANT: every emitted frame is pump-acceptable
# ═══════════════════════════════════════════════════════════════════════════

def test_emitter_frames_with_ff_on_are_accepted_by_a_real_pump():
    """The MVP command path's headline invariant, re-asserted with the FF live."""
    geom = _geom()
    emit = KnotEmitter(geom, torque_ff=_ff())
    pump = _pump_ff(ramp_frames=80)
    plan = HoldPlan(ACTIVE_POSE)

    for k in range(200):
        frame = emit.frame(plan, k * 0.025, k)
        sp, reason = pump.build(frame, k * 25_000)
        assert reason is None, f'frame {k} rejected: {reason}'
        assert all(math.isfinite(t) for t in sp.torque_ff)

    assert pump.frames_rejected == 0
    assert pump.frames_built == 200
    # Steady state reached, and it is the real gravity load.
    assert pump.torque_ff_ramp_scale() == 1.0
    tau = _ff().compute(ACTIVE_POSE[:3], rotvec_to_rot_matrix(ACTIVE_POSE[3:6]),
                        ZERO6, ZERO6)
    assert np.allclose(np.array(sp.torque_ff),
                       tau * float(hw.ODRIVE_LEG_TORQUE_WIRE_SCALE))


def test_emitter_reuses_the_callers_jacobian_and_leg_accels():
    """The FF must not silently pay for a second Jacobian / J̇ at 40 Hz.

    Pinned by behaviour: passing J explicitly must give the same answer as letting the
    FF recompute it (so the emitter's reuse is safe), including for the reflected-rotor
    term which needs the leg accelerations.
    """
    geom = _geom()
    ff = _ff(platform_inertia=True)
    rot = rotvec_to_rot_matrix(ACTIVE_POSE[3:6])
    pos = ACTIVE_POSE[:3]
    twist = np.array([50.0, 20.0, -30.0, 0.1, 0.0, -0.05])
    accel = np.array([500.0, -200.0, 800.0, 1.0, -0.5, 0.2])

    J = compute_jacobian(pos, rot, geom)
    a = ff.compute(pos, rot, twist, accel)                  # recompute internally
    b = ff.compute(pos, rot, twist, accel, J=J)             # caller-supplied J
    assert np.allclose(a, b, rtol=1e-12, atol=1e-15)


# ═══════════════════════════════════════════════════════════════════════════
# 10. dynamics.py reuse — no re-implementation of the physics
# ═══════════════════════════════════════════════════════════════════════════

def test_ff_agrees_with_dynamics_gravity_to_motor_torques():
    """The FF module must be a config gate over ``dynamics.py``, not a second model."""
    from jugglebot.motion.dynamics import gravity_to_motor_torques

    geom = _geom()
    params = DynamicsParams.from_config()
    rot = rotvec_to_rot_matrix(ACTIVE_POSE[3:6])
    reference = gravity_to_motor_torques(ACTIVE_POSE[:3], rot, geom, params)
    mine = _ff().compute(ACTIVE_POSE[:3], rot, ZERO6, ZERO6)
    assert np.allclose(mine, reference, rtol=1e-12, atol=1e-15)


def test_force_decomposition_uses_a_solve_not_a_transpose_product():
    """Guard the ``f = J^{-T}·W`` convention against a regression to ``J^T·W``.

    These two differ by J·J^T, whose condition number on this platform is ~10-60 after
    normalisation — so the wrong one is wrong by a large, pose-dependent factor while
    still *looking* like a plausible 6-vector of torques. Assert the FF matches the
    solve and does NOT match the product.
    """
    geom = _geom()
    params = DynamicsParams.from_config()
    from jugglebot.motion.dynamics import compute_gravity_wrench

    rot = rotvec_to_rot_matrix(ACTIVE_POSE[3:6])
    J = compute_jacobian(ACTIVE_POSE[:3], rot, geom)
    W_support = -compute_gravity_wrench(rot, params)

    correct = leg_forces_to_motor_torques(np.linalg.solve(J.T, W_support), geom)
    wrong = leg_forces_to_motor_torques(J.T @ W_support, geom)

    mine = _ff().compute(ACTIVE_POSE[:3], rot, ZERO6, ZERO6)
    assert np.allclose(mine, correct, rtol=1e-12)
    assert not np.allclose(mine, wrong, rtol=1e-3)


# ── Adversarial-review fixes, 2026-07-14 ───────────────────────────────────
# (logbook/2026-07-14-kt-reconciliation-and-gravity-ff.md — review findings)

def test_reflected_rotor_magnitude_pin_kills_2pi_and_mm_to_rev_mutations():
    """Review finding: dropping the 2*pi from the reflected-rotor term (the
    rev/s^2-vs-rad/s^2 confusion — this codebase's recurring unit-error class)
    survived the ENTIRE motion suite. This pins the exact magnitude:
    tau_reflected = J_rotor * (leg_acc_mm_s2 * mm_to_rev * 2*pi), per leg."""
    import jugglebot.motion.torque_ff as tff_mod
    geom = _geom()
    params = DynamicsParams(mass_kg=0.0,           # zero mass: no wrench terms
                            com_offset_mm=np.zeros(3),
                            gravity_mps2=9.806,
                            inertia_tensor_kgmm2=np.zeros((3, 3)),
                            motor_rotor_inertia_kgm2=2.75e-4)
    ff = LegTorqueFeedforward(geom, params, enabled=True, gravity=False,
                              platform_inertia=True)
    pose = np.array([0.0, 0.0, 170.0])
    rot = np.eye(3)
    accel = np.array([0.0, 0.0, 4000.0, 0.0, 0.0, 0.0])   # 4 m/s^2 up
    leg_acc = np.full(6, 4000.0)                          # synthetic, mm/s^2
    tau = ff.compute(pose, rot, np.zeros(6), accel, leg_acc_mm_s2=leg_acc)
    expected = 2.75e-4 * (4000.0 * np.asarray(geom.mm_to_rev) * 2.0 * np.pi)
    np.testing.assert_allclose(tau, expected, rtol=1e-12)
    # And the magnitude is meaningful: dropping 2*pi would be a 6.28x error.
    assert np.all(expected > 0.02), "pin must be large enough to catch a 6.28x drop"


def test_inertia_wrench_transport_moment_virtual_work():
    """Review finding: compute_inertia_wrench omitted the Newton-Euler
    transport moment r_com x (m*a_com) about the platform geometric centre.
    Independent check by rotational virtual work: for a pure linear
    acceleration `a` and CoM offset r, the wrench's moment row must equal
    r x (m*a) — the power delivered through a virtual rotation about the
    centre must match the CoM's displacement rate against the inertial force."""
    from jugglebot.motion.dynamics import compute_inertia_wrench
    params = DynamicsParams(mass_kg=1.2,
                            com_offset_mm=np.array([-9.68, -68.64, 52.73]),
                            gravity_mps2=9.806,
                            inertia_tensor_kgmm2=np.zeros((3, 3)))
    rot = np.eye(3)
    accel = np.array([0.0, 0.0, 4000.0, 0.0, 0.0, 0.0])   # pure linear, 4 m/s^2
    W = compute_inertia_wrench(rot, np.zeros(6), accel, params)
    F = W[:3]                                             # N
    np.testing.assert_allclose(F, [0.0, 0.0, 1.2 * 4.0], rtol=1e-12)
    # Transport moment, computed independently: r (mm) x F (N) = N*mm.
    expected_moment = np.cross(params.com_offset_mm, F)
    np.testing.assert_allclose(W[3:], expected_moment, rtol=1e-12)
    # And it is NOT zero — the pre-fix code returned exactly zeros here.
    assert np.linalg.norm(W[3:]) > 100.0                  # ~330 N*mm at this accel


def test_singular_jacobian_holds_then_decays_never_steps_to_zero():
    """Review finding: a singular/ill-conditioned Jacobian used to step a live
    feedforward to exact zeros in one tick — the same discontinuity class as
    the 2026-05-08 friction-FF limit cycle trigger. Now: hold the last good
    torque, decay linearly to zero over _DECAY_CALLS, recover instantly on the
    next good tick."""
    import jugglebot.motion.torque_ff as tff_mod
    ff = _ff()
    pose = np.array([0.0, 0.0, 170.0])
    rot = np.eye(3)
    z6 = np.zeros(6)
    good = ff.compute(pose, rot, z6, z6)
    assert np.all(good > 0)
    # Feed a singular Jacobian: exactly-singular J makes solve raise.
    J_sing = np.zeros((6, 6))
    t1 = ff.compute(pose, rot, z6, z6, J=J_sing)
    # First bad tick: held value, slightly decayed — NOT zeros, NOT a step.
    expected_1 = good * (1.0 - 1.0 / tff_mod._DECAY_CALLS)
    np.testing.assert_allclose(t1, expected_1, rtol=1e-12)
    # Decays monotonically to exact zeros by _DECAY_CALLS bad ticks.
    last = t1
    for _ in range(tff_mod._DECAY_CALLS):
        t = ff.compute(pose, rot, z6, z6, J=J_sing)
        assert np.all(np.abs(t) <= np.abs(last) + 1e-15)
        last = t
    np.testing.assert_array_equal(last, np.zeros(6))
    # Recovery on the next good tick is immediate and exact.
    np.testing.assert_allclose(ff.compute(pose, rot, z6, z6), good, rtol=1e-12)


def test_ill_conditioned_jacobian_triggers_sanity_bound():
    """np.linalg.solve does NOT raise on an ill-conditioned J — it returns huge
    FINITE forces. The producer must catch those via the physical sanity bound
    (no leg's gravity+inertia FF can exceed _SANE_MAX_NM at any valid pose)."""
    import jugglebot.motion.torque_ff as tff_mod
    ff = _ff()
    pose = np.array([0.0, 0.0, 170.0])
    rot = np.eye(3)
    z6 = np.zeros(6)
    good = ff.compute(pose, rot, z6, z6)
    # Near-singular (not exactly singular) J: solve succeeds, forces explode.
    J_ill = np.eye(6) * 1e-9
    t = ff.compute(pose, rot, z6, z6, J=J_ill)
    assert np.all(np.abs(t) <= tff_mod._SANE_MAX_NM), \
        "implausible torques must never leave the producer"
    # It took the degrade path (held+decayed), not the explode path.
    np.testing.assert_allclose(
        t, good * (1.0 - 1.0 / tff_mod._DECAY_CALLS), rtol=1e-12)


def test_pump_restart_torque_ramp_restarts_ff_only_not_step_gate():
    """Review finding: the guard-latch -> /recover path (mpc_active stays 1)
    never fired reset(), so the FF returned as a full-magnitude step after the
    firmware recovery slew. restart_torque_ramp() must restart the ramp WITHOUT
    waiving the per-frame position-step gate (unlike reset())."""
    pump = _pump(torque_ff_enabled=True, torque_ff_ramp_frames=10)
    cmd = dict(motor_rev=[1.0] * 6, vel_mm_s=[10.0] * 6,
               cmd_next_mm=[71.0] * 6, cmd_next2_mm=[72.0] * 6,
               ext_mm=[70.0] * 6, torque_Nm=[0.04] * 6, seq=1)
    for _ in range(10):
        sp, _ = pump.build(cmd, t_origin_us=0)
    assert pump.torque_ff_ramp_scale() == 1.0
    full = sp.torque_ff[0]
    assert full > 0.0
    pump.restart_torque_ramp()
    # Ramp is back at zero...
    assert pump.torque_ff_ramp_scale() == 0.0
    sp2, _ = pump.build(cmd, t_origin_us=0)
    assert sp2.torque_ff[0] < full
    # ...but the step gate baseline SURVIVED (unlike reset()): a step-violating
    # frame is still rejected on the very next build.
    bad = dict(cmd)
    bad['motor_rev'] = [1.0 + 2.0 * pump.max_step_rev] * 6
    sp3, reason = pump.build(bad, t_origin_us=0)
    assert sp3 is None and reason


def test_pump_torque_ff_max_nm_ceiling():
    """Review finding: the only clamp in the chain had no ceiling — a config
    typo could set it above the ~0.55 wire-Nm point where the FF consumes the
    whole current budget and the position loop loses all authority."""
    with pytest.raises(ValueError, match='ceiling'):
        _pump(torque_ff_enabled=True, torque_ff_max_nm=0.6)
    # The shipped value and the boundary are accepted.
    _pump(torque_ff_enabled=True, torque_ff_max_nm=0.15)
    _pump(torque_ff_enabled=True, torque_ff_max_nm=0.30)


def test_hardware_plant_honours_config_term_flags(monkeypatch):
    """Review finding (the third producer): HardwarePlant read NONE of the
    config flags — enabling the master switch would have armed the MPC path's
    acceleration-proportional platform-inertia FF that
    torque_ff_platform_inertia=false exists to gate. The skip flags must flow
    from the generated config into compute_full_feedforward_torques."""
    from jugglebot.motion.motor_commands import cartesian_to_motor_commands
    from jugglebot.motion.dynamics import DynamicsParams as DP
    geom = _geom()
    params = DP.from_config()
    pose = np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0])
    accel = np.array([0.0, 0.0, 4000.0, 0.0, 0.0, 0.0])
    z6 = np.zeros(6)
    # gravity-only (the shipped per-term config): accel must NOT change tau.
    _, _, tau_static, _ = cartesian_to_motor_commands(
        pose, z6, z6, geom, params, feedforward_enabled=True,
        skip_reflected_inertia=True, skip_platform_inertia=True)
    _, _, tau_accel, _ = cartesian_to_motor_commands(
        pose, z6, accel, geom, params, feedforward_enabled=True,
        skip_reflected_inertia=True, skip_platform_inertia=True)
    np.testing.assert_allclose(tau_accel, tau_static, rtol=1e-12)
    # with the inertia term allowed, the same accel MUST change tau.
    _, _, tau_inertial, _ = cartesian_to_motor_commands(
        pose, z6, accel, geom, params, feedforward_enabled=True,
        skip_reflected_inertia=True, skip_platform_inertia=False)
    assert float(np.max(np.abs(tau_inertial - tau_static))) > 1e-3
    # and skip_gravity kills the static term entirely.
    _, _, tau_none, _ = cartesian_to_motor_commands(
        pose, z6, z6, geom, params, feedforward_enabled=True,
        skip_reflected_inertia=True, skip_gravity=True,
        skip_platform_inertia=True)
    np.testing.assert_array_equal(tau_none, np.zeros(6))
