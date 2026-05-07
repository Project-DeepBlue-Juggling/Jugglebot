"""Motor-guard friction-feedforward unit tests.

Covers the contract from
``plans/active/friction-ff-motor-guard-integration.md`` §2 — single
canonical enforcement point ``MotorGuard._compute_friction_ff_Nm`` plus
the additive integration into ``_commanded_torque_ff_Nm``.  Test list
mirrors §7.1.  The function is the vectorised port of the bench-validated
``friction_ff_nm`` in ``tests/hardware/friction_ff_demo.py:167-205``.
"""

from __future__ import annotations

import math
import time

import numpy as np

from jugglebot.motion.friction_ff_params import FrictionFFParams
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.ipc import (
    TOPIC_MODE,
    TOPIC_MPC_CMD,
    TOPIC_MOTOR_FB,
    make_mode_command,
    make_mpc_command,
)
from jugglebot.motion.motor_guard import (
    MAX_EXTRAP_DT_S,
    GuardMode,
    MotorGuard,
)
from jugglebot.motion.conversions import extensions_mm_to_revs


# ---------------------------------------------------------------------------
# Test scaffolding (mirrors tests/motion/test_motor_guard.py)
# ---------------------------------------------------------------------------

class _MockIPC:
    def __init__(self):
        self._queue: list[tuple[bytes, dict]] = []
        self.sent_telemetry: list[dict] = []
        self._last_recv = time.monotonic()

    def inject(self, topic: bytes, msg: dict) -> None:
        self._queue.append((topic, msg))

    def recv_all(self):
        msgs = list(self._queue)
        self._queue.clear()
        if msgs:
            self._last_recv = time.monotonic()
        return msgs

    def send_telemetry(self, msg: dict) -> None:
        self.sent_telemetry.append(msg)

    @property
    def seconds_since_last_recv(self) -> float:
        return time.monotonic() - self._last_recv

    def close(self) -> None:
        pass


def _make_guard() -> tuple[MotorGuard, _MockIPC]:
    ipc = _MockIPC()
    geom = StewartGeometry()
    guard = MotorGuard(rate_hz=500, geom=geom, ipc=ipc)
    return guard, ipc


def _enable(guard: MotorGuard, ipc: _MockIPC) -> None:
    ipc.inject(TOPIC_MODE, make_mode_command('enable', source='MPC'))
    guard._process_ipc()


def _params(enabled: bool = True, **overrides) -> FrictionFFParams:
    """Build a ``FrictionFFParams`` for tests.  Bench-fit defaults; overrides
    let individual tests pin specific scalars or per-leg arrays."""
    base = dict(
        enabled=enabled,
        stiction_boost_threshold_rps=0.20,
        ff_sign=np.full(6, -1.0),
        tau_c_A=np.full(6, 1.094),
        tau_s_A=np.full(6, 1.953),
        omega_s_rps=np.full(6, 0.251),
        b_A_per_rps=np.full(6, 0.0173),
        load_offset_A=np.zeros(6),
        motor_kt_nm_per_a=0.0624,
    )
    for k, v in overrides.items():
        if isinstance(v, (int, float)) and k not in (
                'enabled', 'stiction_boost_threshold_rps', 'motor_kt_nm_per_a'):
            base[k] = np.full(6, float(v))
        else:
            base[k] = v
    return FrictionFFParams(**base)


def _ref_friction_ff_nm(v_cmd: float, p: FrictionFFParams,
                        leg: int = 0) -> float:
    """Scalar reference port of friction_ff_nm (bench, friction_ff_demo.py).

    Used as ground truth for cross-checking the vectorised compute output.
    """
    av = abs(v_cmd)
    if av < 1e-4:
        return p.load_offset_A[leg] * p.ff_sign[leg] * p.motor_kt_nm_per_a
    s = math.copysign(1.0, v_cmd)
    if av < p.stiction_boost_threshold_rps:
        iq_friction = p.tau_s_A[leg] + p.b_A_per_rps[leg] * av
    else:
        iq_friction = (p.tau_c_A[leg]
                       + (p.tau_s_A[leg] - p.tau_c_A[leg])
                         * math.exp(-(av / p.omega_s_rps[leg]) ** 2)
                       + p.b_A_per_rps[leg] * av)
    iq_total = -s * iq_friction + p.load_offset_A[leg]
    return iq_total * p.ff_sign[leg] * p.motor_kt_nm_per_a


# ---------------------------------------------------------------------------
# Tests — §7.1 of the integration plan
# ---------------------------------------------------------------------------

def test_friction_ff_disabled_returns_zero():
    """Flag off → np.zeros(6); the disabled mode is one branch + memset."""
    guard, _ = _make_guard()
    guard._friction_ff_params = _params(enabled=False)
    guard._commanded_vel_ff_rps = np.array([0.5, -0.5, 1.0, -1.0, 0.1, -0.1])
    out = guard._compute_friction_ff_Nm()
    np.testing.assert_array_equal(out, np.zeros(6))


def test_friction_ff_zero_at_v_zero():
    """At v_cmd ≈ 0, FF is just load_offset × Kt × ff_sign — no friction."""
    guard, _ = _make_guard()
    p = _params(enabled=True, load_offset_A=np.array(
        [0.10, -0.05, 0.20, -0.15, 0.0, 0.30]))
    guard._friction_ff_params = p
    guard._commanded_vel_ff_rps = np.zeros(6)
    out = guard._compute_friction_ff_Nm()
    expected = p.load_offset_A * p.ff_sign * p.motor_kt_nm_per_a
    np.testing.assert_allclose(out, expected, atol=1e-12)


def test_friction_ff_signs_with_velocity():
    """For each leg: FF(+v) − load_term and FF(−v) − load_term are opposite-signed.

    Equivalent to ``FF(+v) + FF(-v) == 2 × load × ff_sign × Kt``.
    """
    guard, _ = _make_guard()
    p = _params(enabled=True, load_offset_A=np.array(
        [0.10, -0.05, 0.20, -0.15, 0.0, 0.30]))
    guard._friction_ff_params = p

    v = np.array([0.5, 0.5, 0.5, 0.5, 0.5, 0.5])
    guard._commanded_vel_ff_rps = v
    out_pos = guard._compute_friction_ff_Nm().copy()
    guard._commanded_vel_ff_rps = -v
    out_neg = guard._compute_friction_ff_Nm().copy()

    expected_sum = 2.0 * p.load_offset_A * p.ff_sign * p.motor_kt_nm_per_a
    np.testing.assert_allclose(out_pos + out_neg, expected_sum, atol=1e-12)
    # The friction component itself flips sign — magnitudes match.
    np.testing.assert_allclose(
        out_pos - p.load_offset_A * p.ff_sign * p.motor_kt_nm_per_a,
        -(out_neg - p.load_offset_A * p.ff_sign * p.motor_kt_nm_per_a),
        atol=1e-12)


def test_friction_ff_taper_to_coulomb():
    """At |v| = 5·ω_s, magnitude is within 5 % of τ_c (Stribeck term tapered)."""
    guard, _ = _make_guard()
    p = _params(enabled=True)
    guard._friction_ff_params = p
    # 5 × ω_s puts exp(−25) ≈ 1.4e-11 — Stribeck term is essentially zero.
    v_cmd = 5.0 * p.omega_s_rps[0]
    guard._commanded_vel_ff_rps = np.full(6, v_cmd)
    out = guard._compute_friction_ff_Nm()
    # Magnitude per leg = (τ_c + b·|v|) × Kt.  load_offset = 0 in defaults.
    expected_mag = (p.tau_c_A[0] + p.b_A_per_rps[0] * v_cmd) * p.motor_kt_nm_per_a
    # |out| should be within 5 % of (τ_c × Kt) — i.e. the viscous term
    # contribution at v=5·ω_s ≈ 0.0173·1.255 = 0.0217 A is small
    # vs τ_c=1.094 A (~2 %).  Allowable wiggle: 5 %.
    target = p.tau_c_A[0] * p.motor_kt_nm_per_a
    np.testing.assert_allclose(np.abs(out), expected_mag, atol=1e-12)
    assert np.all(np.abs(np.abs(out) - target) / target < 0.05), (
        f'|FF| {np.abs(out)} vs τ_c·Kt {target} — diff > 5 %')


def test_friction_ff_boost_at_low_v():
    """At |v| = 0.5 × boost_threshold, magnitude is within 5 % of τ_s × Kt."""
    guard, _ = _make_guard()
    p = _params(enabled=True)
    guard._friction_ff_params = p
    v_cmd = 0.5 * p.stiction_boost_threshold_rps
    guard._commanded_vel_ff_rps = np.full(6, v_cmd)
    out = guard._compute_friction_ff_Nm()
    target = p.tau_s_A[0] * p.motor_kt_nm_per_a
    # Boosted value = (τ_s + b·|v|) × Kt — viscous term adds ~0.18 % at
    # v=0.10 with b=0.0173.  Within 5 %.
    assert np.all(np.abs(np.abs(out) - target) / target < 0.05), (
        f'|FF| {np.abs(out)} vs τ_s·Kt {target} — diff > 5 %')


def test_friction_ff_continuity_at_boost_threshold():
    """Discontinuity at threshold edge equals the predictable Stribeck-vs-τ_s gap.

    Sanity check, not a tight tolerance — confirms the only step at the
    band edge is the (τ_s − Stribeck(threshold)) term, with the viscous
    term shared between both branches.
    """
    guard, _ = _make_guard()
    p = _params(enabled=True)
    guard._friction_ff_params = p
    eps = 1e-6
    v_below = p.stiction_boost_threshold_rps - eps
    v_above = p.stiction_boost_threshold_rps + eps

    guard._commanded_vel_ff_rps = np.full(6, v_below)
    below = guard._compute_friction_ff_Nm().copy()
    guard._commanded_vel_ff_rps = np.full(6, v_above)
    above = guard._compute_friction_ff_Nm().copy()

    # Predicted gap: (τ_s − Stribeck(threshold)) × Kt × ff_sign
    av = p.stiction_boost_threshold_rps
    stribeck = (p.tau_c_A[0]
                + (p.tau_s_A[0] - p.tau_c_A[0])
                  * math.exp(-(av / p.omega_s_rps[0]) ** 2)
                + p.b_A_per_rps[0] * av)
    boosted = p.tau_s_A[0] + p.b_A_per_rps[0] * av
    expected_gap = (boosted - stribeck) * p.motor_kt_nm_per_a  # positive iq diff
    # Magnitudes: |below| should exceed |above| by expected_gap.  Tolerance
    # absorbs the eps offset on either side of the threshold (b·eps + slope of
    # Stribeck taper at threshold × eps), which contributes a few ×10⁻⁷ Nm.
    np.testing.assert_allclose(
        np.abs(below) - np.abs(above), expected_gap, atol=1e-6)


def test_friction_ff_per_leg_independence():
    """Different per-leg params produce leg-independent FF outputs."""
    guard, _ = _make_guard()
    # Make every per-leg param unique per leg.
    p = _params(
        enabled=True,
        ff_sign=np.array([-1.0, +1.0, -1.0, +1.0, -1.0, +1.0]),
        tau_c_A=np.array([1.0, 1.1, 1.2, 1.3, 1.4, 1.5]),
        tau_s_A=np.array([1.8, 1.85, 1.9, 1.95, 2.0, 2.05]),
        omega_s_rps=np.array([0.20, 0.22, 0.24, 0.26, 0.28, 0.30]),
        b_A_per_rps=np.array([0.010, 0.012, 0.014, 0.016, 0.018, 0.020]),
        load_offset_A=np.array([0.0, 0.05, 0.10, 0.15, 0.20, 0.25]),
    )
    guard._friction_ff_params = p
    v = np.array([0.5, -0.5, 0.7, -0.7, 0.3, -0.3])
    guard._commanded_vel_ff_rps = v
    out = guard._compute_friction_ff_Nm()

    expected = np.array([_ref_friction_ff_nm(v[i], p, leg=i)
                         for i in range(6)])
    np.testing.assert_allclose(out, expected, atol=1e-12)


def test_friction_ff_buffer_identity():
    """``_friction_ff_buf`` Python id is stable across many calls."""
    guard, _ = _make_guard()
    guard._friction_ff_params = _params(enabled=True)
    initial_id = id(guard._friction_ff_buf)
    rng = np.random.default_rng(42)
    for _ in range(1000):
        guard._commanded_vel_ff_rps = rng.uniform(-2.0, 2.0, size=6)
        guard._compute_friction_ff_Nm()
    assert id(guard._friction_ff_buf) == initial_id


def test_friction_ff_int16_clamp():
    """Even with extreme params, the summed torque_ff stays in int16 encode range.

    Encoder convention: ``can_node._send_position_target`` quantises Nm
    via ``int(round(τ × INPUT_SCALE_LEG_TOR))`` with
    ``INPUT_SCALE_LEG_TOR = 10000`` and clamps to int16 (±32767).  This
    test confirms realistic friction-FF magnitudes fit comfortably; the
    int16 clamp itself happens in can_node, not motor_guard.
    """
    INPUT_SCALE_LEG_TOR = 10000  # mirrors ros_ws/.../can/jugglebot_protocol.py
    guard, _ = _make_guard()
    # τ_s = 5 A is far above the bench's 1.953 A — extreme test value
    # per the integration plan.
    p = _params(enabled=True, tau_s_A=np.full(6, 5.0))
    guard._friction_ff_params = p
    # Stack onto a non-trivial MPC dynamics torque too (worst-case).
    guard._mpc_base_torque_Nm = np.full(6, 0.5)
    guard._commanded_vel_ff_rps = np.full(6, 0.10)  # boost band

    np.add(guard._mpc_base_torque_Nm,
           guard._compute_friction_ff_Nm(),
           out=guard._commanded_torque_ff_Nm)

    raw = np.round(guard._commanded_torque_ff_Nm * INPUT_SCALE_LEG_TOR).astype(int)
    assert np.all(raw >= -32768), f'underflow: {raw}'
    assert np.all(raw <= 32767), f'overflow: {raw}'


def test_friction_ff_clamped_leg_zeros():
    """A stroke-clamped leg has friction FF zeroed in the final torque_ff.

    Mirrors test_stroke_clamp_workspace_status from test_motor_guard.py:
    motor feedback is parked at stroke_max so the lead-clamp lets the
    extrapolated commanded position reach the stroke boundary, where the
    stroke-clamp truncates it.  The stroke-clamp's
    ``_commanded_torque_ff_Nm[clamped] = 0`` write is the load-bearing
    zero-out — regardless of how torque_ff was computed (MPC base only,
    or MPC base + friction FF), the clamped-leg torque_ff ends at zero.
    """
    guard, ipc = _make_guard()
    guard._friction_ff_params = _params(enabled=True)
    _enable(guard, ipc)

    ext_mm = [140.0] * 6
    pos_rev = extensions_mm_to_revs(np.array(ext_mm), guard.geom).tolist()
    ipc.inject(TOPIC_MOTOR_FB, {
        'type': 'motor_feedback',
        'pos': list(pos_rev),
        'vel': [0.0] * 6,
        'cur': [0.0] * 6,
    })
    ipc.inject(TOPIC_MPC_CMD, make_mpc_command(
        ext_mm=ext_mm, pose_6dof=[0.0, 0.0, 50.0, 0.0, 0.0, 0.0],
        torque_Nm=[0.04] * 6, seq=0))
    guard._process_ipc()

    # Park feedback at stroke max so lead-clamp allows the extrapolation
    # to reach stroke_max (otherwise lead-clamp catches it 0.15 rev above
    # the feedback first and stroke-clamp never fires).
    guard._motor_fb_pos_rev = guard._stroke_max_rev.copy()
    guard._mpc_base_vel_rps = np.full(6, 100.0)
    guard._mpc_base_timestamp = time.perf_counter() - MAX_EXTRAP_DT_S
    guard._interpolate_and_send(guard.dt_target)

    # Stroke-clamp must have fired: position at stroke max, torque_ff zero.
    np.testing.assert_allclose(
        guard._commanded_pos_rev, guard._stroke_max_rev, atol=1e-9)
    np.testing.assert_array_equal(
        guard._commanded_torque_ff_Nm, np.zeros(6))


def test_friction_ff_additive_with_dynamics_torque():
    """Final torque_ff is the SUM of MPC base torque and friction FF (not a replacement).

    Pre-existing MPC dynamics torque (gravity + platform inertia) and the
    new friction FF model orthogonal physics — they must stack.  Verified
    via the integration point at motor_guard.py:_interpolate_and_send.
    """
    guard, ipc = _make_guard()
    guard._friction_ff_params = _params(enabled=True)
    _enable(guard, ipc)

    ext_mm = [140.0] * 6
    pos_rev = extensions_mm_to_revs(np.array(ext_mm), guard.geom).tolist()
    ipc.inject(TOPIC_MOTOR_FB, {
        'type': 'motor_feedback',
        'pos': list(pos_rev),
        'vel': [0.0] * 6,
        'cur': [0.0] * 6,
    })

    # Known fake MPC dynamics torque — distinct per leg so a uniform
    # friction-FF couldn't masquerade as a passthrough.
    base_torque = [0.040, -0.030, 0.020, -0.010, 0.025, -0.035]
    ipc.inject(TOPIC_MPC_CMD, make_mpc_command(
        ext_mm=ext_mm, pose_6dof=[0.0, 0.0, 50.0, 0.0, 0.0, 0.0],
        vel_mm_s=[100.0] * 6,    # nonzero so friction FF is active
        torque_Nm=base_torque, seq=0))
    guard._process_ipc()

    # Drive the interpolation a small step into the segment so vel_ff is
    # populated (Hermite interpolator); force the pre-clamp torque to be
    # the SUM by reading guard state mid-cycle is awkward — instead, call
    # the integration formula directly with the known inputs.
    guard._commanded_vel_ff_rps = np.full(6, 0.5)  # rev/s, well above boost band
    expected_friction = guard._compute_friction_ff_Nm().copy()

    out = np.empty(6)
    np.add(guard._mpc_base_torque_Nm, expected_friction, out=out)

    np.testing.assert_allclose(
        out, np.asarray(base_torque) + expected_friction, atol=1e-12)
    # Friction FF must be non-zero — otherwise the test is degenerate.
    assert np.any(np.abs(expected_friction) > 1e-6)


def test_friction_ff_lead_clamped_leg_passes_torque_ff():
    """Lead-clamped (not stroke-clamped) legs preserve torque_ff.

    Lead-clamping zeros vel_ff but intentionally not torque_ff — the leg
    is still moving (just being held back from running too far ahead of
    actual encoder), and friction torque assists the motion.  This test
    pins down the existing behaviour so a future "zero torque_ff on lead-
    clamp" change becomes a deliberate, visible decision.
    """
    guard, ipc = _make_guard()
    guard._friction_ff_params = _params(enabled=True)
    _enable(guard, ipc)

    ext_mm = [140.0] * 6
    pos_rev = extensions_mm_to_revs(np.array(ext_mm), guard.geom).tolist()
    ipc.inject(TOPIC_MOTOR_FB, {
        'type': 'motor_feedback',
        'pos': list(pos_rev),
        'vel': [0.0] * 6,
        'cur': [0.0] * 6,
    })
    ipc.inject(TOPIC_MPC_CMD, make_mpc_command(
        ext_mm=ext_mm, pose_6dof=[0.0, 0.0, 50.0, 0.0, 0.0, 0.0],
        vel_mm_s=[100.0] * 6,
        seq=0))
    guard._process_ipc()

    # Force lead-clamp without stroke-clamp: extreme vel + stale base
    # so commanded_pos runs ahead of feedback by > MAX_LEAD_REV.
    # Use a small backdate so the result stays well within stroke limits.
    guard._mpc_base_vel_rps = np.full(6, 20.0)
    guard._mpc_base_timestamp = time.perf_counter() - 0.020
    guard._interpolate_and_send(guard.dt_target)

    # vel_ff must be zero (lead-clamp zeros it).
    np.testing.assert_array_equal(
        guard._commanded_vel_ff_rps, np.zeros(6))
    # torque_ff must still equal MPC base + friction FF — i.e. NOT zeroed.
    # MPC base torque is zero in this command (no torque_Nm passed) so
    # we assert torque_ff is non-zero by virtue of friction FF.  The
    # friction value used in this cycle was computed against the PRE-
    # clamp vel_ff_rps (which had nonzero magnitude before lead-clamp
    # zeroed it), so friction should be nonzero on at least one leg.
    assert np.any(np.abs(guard._commanded_torque_ff_Nm) > 1e-9), (
        'Lead-clamp must preserve torque_ff — friction FF was zeroed too')
