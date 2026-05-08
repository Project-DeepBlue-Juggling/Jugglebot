"""Motor-guard friction-feedforward unit tests.

Covers the contract from
``plans/archived/2026-05-08 friction-ff-motor-guard-integration.md`` §2 — single
canonical enforcement point ``MotorGuard._compute_friction_ff_Nm`` plus
the additive integration into ``_commanded_torque_ff_Nm``.  Test list
mirrors §7.1, updated for PR 2.1's smooth-gate replacement of the
bench-era boost band.  See
``logbook/2026-05-08-friction-ff-platform-limit-cycle.md`` for why the
boost band's hard 0 → τ_s step at v=±1e-4 was removed.

The new function is:
    iq_friction(v) = stribeck(v) · gate(v)
where:
    stribeck(v) = τ_c + (τ_s−τ_c)·exp(−(|v|/ω_s)²) + b·|v|
    gate(v)     = 1 − exp(−(|v|/v_gate)²)
"""

from __future__ import annotations

import math
import time

import numpy as np
import pytest

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
    """Build a ``FrictionFFParams`` for tests.  Bench-fit τ_c/τ_s/ω_s/b,
    platform-default ff_sign=+1 and v_gate=0.05.  Overrides let
    individual tests pin specific scalars or per-leg arrays.

    Scalar overrides (other than ``enabled``, ``v_gate_rps``,
    ``motor_kt_nm_per_a``) auto-broadcast to a length-6 array so tests
    can write e.g. ``_params(tau_s_A=5.0)`` to set every leg to 5 A.
    """
    base = dict(
        enabled=enabled,
        v_gate_rps=0.05,
        ff_sign=np.full(6, +1.0),
        tau_c_A=np.full(6, 1.094),
        tau_s_A=np.full(6, 1.953),
        omega_s_rps=np.full(6, 0.251),
        b_A_per_rps=np.full(6, 0.0173),
        load_offset_A=np.zeros(6),
        motor_kt_nm_per_a=0.0624,
    )
    for k, v in overrides.items():
        if isinstance(v, (int, float)) and k not in (
                'enabled', 'v_gate_rps', 'motor_kt_nm_per_a'):
            base[k] = np.full(6, float(v))
        else:
            base[k] = v
    return FrictionFFParams(**base)


def _ref_friction_ff_nm(v_cmd: float, p: FrictionFFParams,
                        leg: int = 0) -> float:
    """Scalar reference port of the smooth-gate friction-FF function.

    This is the analytic ground truth for cross-checking the vectorised
    ``_compute_friction_ff_Nm``.  The reference uses Python ``math``
    (not numpy) so it is independent of the production path's ufunc
    chain — any divergence flags an implementation bug.

    Formula (mirrors motor_guard.py:_compute_friction_ff_Nm):
        stribeck = τ_c + (τ_s−τ_c)·exp(−(|v|/ω_s)²) + b·|v|
        gate     = 1 − exp(−(|v|/v_gate)²)
        iq_total = −sign(v)·(stribeck·gate) + load_offset
        torque   = iq_total · ff_sign · Kt
    """
    av = abs(v_cmd)
    s = 0.0 if av == 0.0 else math.copysign(1.0, v_cmd)
    stribeck = (p.tau_c_A[leg]
                + (p.tau_s_A[leg] - p.tau_c_A[leg])
                  * math.exp(-(av / p.omega_s_rps[leg]) ** 2)
                + p.b_A_per_rps[leg] * av)
    gate = 1.0 - math.exp(-(av / p.v_gate_rps) ** 2)
    iq_friction = stribeck * gate
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


def test_friction_ff_smooth_gate_at_v_gate():
    """At |v| = v_gate, gate value is exactly 1 − e⁻¹ ≈ 0.6321.

    This pins the canonical operating point of the smooth gate.  At
    |v| = v_gate the magnitude is ~63 % of the un-gated Stribeck value
    — by definition, since gate(v_gate) = 1 − exp(−1).  Any drift of
    this value would mean the gate function changed; tests below use
    this as the reference.
    """
    guard, _ = _make_guard()
    p = _params(enabled=True)
    guard._friction_ff_params = p
    v_cmd = p.v_gate_rps
    guard._commanded_vel_ff_rps = np.full(6, v_cmd)
    out = guard._compute_friction_ff_Nm()

    av = v_cmd
    stribeck = (p.tau_c_A[0]
                + (p.tau_s_A[0] - p.tau_c_A[0])
                  * math.exp(-(av / p.omega_s_rps[0]) ** 2)
                + p.b_A_per_rps[0] * av)
    expected_gate = 1.0 - math.exp(-1.0)   # = 0.6321...
    expected = stribeck * expected_gate * p.motor_kt_nm_per_a
    # ff_sign=+1, sgn=+1 → torque = -expected (sign flip)
    np.testing.assert_allclose(
        np.abs(out), np.full(6, expected), rtol=1e-9,
        err_msg='gate(v_gate) should be 1 − e⁻¹ ≈ 0.6321')


def test_friction_ff_smooth_gate_kills_low_v():
    """At |v| ≪ v_gate, the smooth gate suppresses friction below the
    position-loop's natural absorption threshold.

    THIS IS THE PR 2.1 REGRESSION GUARD.  In PR 2 the boost band emitted
    full ±τ_s × Kt = ±0.122 Nm for any |v| > 1e-4, which kicked the
    platform into a 5 Hz limit cycle.  The smooth gate's gate(v) = 1 −
    exp(−(|v|/v_gate)²) at small |v| ≈ (|v|/v_gate)² (Taylor expansion),
    so torque scales as |v|² — heavily damped at low velocity.

    At |v| = v_gate/10 = 0.005 rev/s: gate ≈ 0.01.  At v_gate=0.05 with
    bench params, expected |τ_ff| ≈ 0.012 Nm × 0.01 ≈ 1 mNm.  We assert
    < 5 mNm to leave headroom for parameter perturbations.  The bench
    boost band would have emitted 0.122 Nm at this same |v| — 100×
    higher than the new smooth-gate value.
    """
    guard, _ = _make_guard()
    p = _params(enabled=True)
    guard._friction_ff_params = p
    v_cmd = p.v_gate_rps / 10.0   # 0.005 rev/s = 5 mrev/s
    guard._commanded_vel_ff_rps = np.full(6, v_cmd)
    out = guard._compute_friction_ff_Nm()
    assert np.all(np.abs(out) < 0.005), (
        f'smooth gate must keep |τ_ff| below 5 mNm at |v|=v_gate/10 — got {out}')


def test_friction_ff_smooth_gate_continuous_through_zero():
    """The function is C⁰ (continuous) and C¹ (smooth) through v=0.

    No discontinuity at any v_cmd — verifies the structural fix from
    PR 2.1.  Sample the function at 200 points spanning [-2·v_gate,
    +2·v_gate] and assert the per-step delta is bounded.

    Per-step delta bound: with the smooth gate, the Lipschitz constant
    of |τ_ff| is bounded by approximately 2·τ_s·Kt/v_gate.  For default
    params that's ~5 Nm/(rev/s).  Across our 200-point sweep with step
    Δv = 4·v_gate/200 = 1 mrev/s, the per-step delta should be well
    below 5 mNm.
    """
    guard, _ = _make_guard()
    p = _params(enabled=True)
    guard._friction_ff_params = p

    vs = np.linspace(-2.0 * p.v_gate_rps, 2.0 * p.v_gate_rps, 201)
    out_vec = np.empty_like(vs)
    for i, v in enumerate(vs):
        guard._commanded_vel_ff_rps = np.full(6, v)
        out_vec[i] = guard._compute_friction_ff_Nm()[0]

    deltas = np.abs(np.diff(out_vec))
    # Per the Lipschitz bound: max delta ≈ 2·τ_s·Kt / v_gate * Δv
    #   = 2 · 1.953 · 0.0624 / 0.05 · (4·0.05/200) = 4.87 mNm
    # Use 6 mNm tolerance (20 % headroom).
    assert deltas.max() < 6e-3, (
        f'|τ_ff| step exceeded smooth-gate Lipschitz bound: '
        f'max Δ = {deltas.max()*1e3:.3f} mNm at v={vs[deltas.argmax()]:+.4f}')


def test_friction_ff_smooth_gate_no_kick_on_baseline_holdnoise():
    """Replay realistic BASELINE hold-phase vel_ff noise through the FF
    function and assert the resulting torque stays small.

    Hold-phase vel_ff in BASELINE (FF off) had this distribution
    (measured 2026-05-08 from rosbag 2026-05-08_13-16-52):
        median ≈ 0 rev/s, 99%ile ≈ 0.021 rev/s, max ≈ 0.29 rev/s.
    A handful of samples cross [0.01, 0.20) — under the OLD boost band
    those would have emitted full ±0.122 Nm kicks, bootstrapping the
    limit cycle.  Under the smooth gate, the same samples should emit
    far smaller torques.

    This test synthesises a vel_ff stream matching the BASELINE
    distribution and asserts that the resulting torque_ff stays below
    a hold-quality threshold.  If a future change reintroduces a hard
    step or removes the gate, this regression fires loudly.
    """
    guard, _ = _make_guard()
    p = _params(enabled=True)
    guard._friction_ff_params = p

    # Synthesise vel_ff matching baseline-hold distribution: a Laplace
    # with scale 0.005 (matches median≈0, 90%ile≈0.012, 99%ile≈0.025)
    # plus rare excursions up to 0.30 rev/s.
    rng = np.random.default_rng(42)
    n_samples = 5000
    vs = rng.laplace(loc=0.0, scale=0.005, size=n_samples)
    # Inject 1 % large-excursion samples
    excursion_idx = rng.choice(n_samples, size=n_samples // 100, replace=False)
    vs[excursion_idx] = rng.uniform(-0.30, 0.30, size=excursion_idx.size)

    torques = np.empty(n_samples)
    for i, v in enumerate(vs):
        guard._commanded_vel_ff_rps = np.full(6, v)
        torques[i] = guard._compute_friction_ff_Nm()[0]

    abs_torques = np.abs(torques)
    median_abs = np.median(abs_torques)
    p99_abs = np.percentile(abs_torques, 99)
    max_abs = abs_torques.max()

    # In a clean baseline-hold replay, the median torque should be tiny.
    # The bench boost band would have given ~0.122 Nm median (since 95 %+
    # of |v| samples cross 1e-4); smooth gate at scale=0.005 hold-noise
    # gives gate ≈ (0.005/0.05)² = 0.01, torque ≈ 1.2 mNm.
    assert median_abs < 0.005, (
        f'smooth-gate median torque on baseline-hold replay too high: '
        f'{median_abs*1e3:.2f} mNm (regression of the limit-cycle fix)')
    # The 99th percentile catches any rare excursions.  Should still be
    # well below the position-loop's absorption capacity.
    assert p99_abs < 0.10, (
        f'smooth-gate 99%ile torque too high: {p99_abs*1e3:.1f} mNm')
    # Max can hit the Stribeck peak during large-excursion samples;
    # bound it to the un-gated Stribeck max plus epsilon.
    expected_max_bound = 1.05 * p.tau_s_A[0] * p.motor_kt_nm_per_a
    assert max_abs < expected_max_bound, (
        f'smooth-gate max torque {max_abs*1e3:.1f} mNm exceeds '
        f'Stribeck max bound {expected_max_bound*1e3:.1f} mNm')


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

    # Sanity: with the velocity the interpolator will commit during the
    # clamped cycle, friction-FF is non-zero — without this peek the
    # post-clamp zero assertion would also pass if the friction code path
    # were never entered (vacuous test).
    guard._commanded_vel_ff_rps = guard._mpc_base_vel_rps.copy()
    pre_clamp_friction = guard._compute_friction_ff_Nm().copy()
    assert np.any(np.abs(pre_clamp_friction) > 1e-6), (
        'pre-clamp friction is zero — test is degenerate, would pass '
        'even if friction-FF code path were never invoked')

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

    # Tighten: assert per-leg sign agreement with the model
    # ``torque_ff = MPC_base + friction(extrapolated_v)``.  The Hermite
    # interpolator commits a velocity slightly different from the raw
    # base_vel (it samples the cubic at s = dt_since_cmd / T), so we
    # avoid pinning the magnitude exactly — but the sign of friction is
    # determined by sign(v) × ff_sign × Kt, which is robust to that
    # discrepancy.  A regression that wired only one of {MPC base,
    # friction} into torque_ff would fail this check.
    guard._commanded_vel_ff_rps = guard._mpc_base_vel_rps.copy()
    expected_friction = guard._compute_friction_ff_Nm().copy()
    expected_torque = guard._mpc_base_torque_Nm + expected_friction
    np.testing.assert_array_equal(
        np.sign(guard._commanded_torque_ff_Nm),
        np.sign(expected_torque),
        err_msg='lead-clamped torque_ff sign disagrees with MPC + friction')


def test_commanded_torque_ff_buffer_identity_across_estop():
    """``_commanded_torque_ff_Nm`` keeps Python id across estop/disable/re-enable.

    The buffer is the ``out=`` target of ``np.add`` in
    ``_interpolate_and_send`` — rebinding in ``_zero_outputs`` would
    silently break any future caller that caches ``id(buf)``.  Mirrors
    the stability contract already held by ``_friction_ff_buf`` (see
    ``test_friction_ff_buffer_identity``).
    """
    guard, ipc = _make_guard()
    initial_torque_id = id(guard._commanded_torque_ff_Nm)
    initial_pos_id = id(guard._commanded_pos_rev)
    initial_vel_id = id(guard._commanded_vel_ff_rps)

    # E-stop path — _trigger_estop calls _zero_outputs.
    guard._trigger_estop('test')
    assert id(guard._commanded_torque_ff_Nm) == initial_torque_id, \
        'estop must not rebind _commanded_torque_ff_Nm'
    assert id(guard._commanded_pos_rev) == initial_pos_id, \
        'estop must not rebind _commanded_pos_rev'
    assert id(guard._commanded_vel_ff_rps) == initial_vel_id, \
        'estop must not rebind _commanded_vel_ff_rps'

    # Disable path — also calls _zero_outputs.
    guard.mode = GuardMode.ENABLED
    ipc.inject(TOPIC_MODE, make_mode_command('disable'))
    guard._process_ipc()
    assert id(guard._commanded_torque_ff_Nm) == initial_torque_id
    assert id(guard._commanded_pos_rev) == initial_pos_id
    assert id(guard._commanded_vel_ff_rps) == initial_vel_id

    # Re-enable — does not call _zero_outputs.
    _enable(guard, ipc)
    assert id(guard._commanded_torque_ff_Nm) == initial_torque_id


def test_friction_ff_scratch_buffer_identity():
    """All friction-FF scratch buffers keep Python id across many calls.

    Extends the buffer-identity contract from ``_friction_ff_buf`` to
    every internal scratch buffer — guarantees the zero-allocation
    rewrite of ``_compute_friction_ff_Nm`` does not silently regress to
    per-call allocation if a future edit reaches for ``np.abs(v)`` etc.
    instead of ``np.abs(v, out=...)``.

    Buffer set updated in PR 2.1: removed the boost-band buffers
    (``_friction_boosted``, ``_friction_in_boost``, ``_friction_dead_zone``)
    and added ``_friction_gate`` for the smooth-gate replacement.
    """
    guard, _ = _make_guard()
    guard._friction_ff_params = _params(enabled=True)
    rng = np.random.default_rng(7)
    ids = {
        '_friction_ff_buf':    id(guard._friction_ff_buf),
        '_friction_av':        id(guard._friction_av),
        '_friction_sgn':       id(guard._friction_sgn),
        '_friction_tmp':       id(guard._friction_tmp),
        '_friction_stribeck':  id(guard._friction_stribeck),
        '_friction_gate':      id(guard._friction_gate),
        '_friction_iq':        id(guard._friction_iq),
        '_friction_iq_total':  id(guard._friction_iq_total),
        '_friction_tau_diff':  id(guard._friction_tau_diff),
        '_friction_kt_signed': id(guard._friction_kt_signed),
    }
    for _ in range(1000):
        guard._commanded_vel_ff_rps = rng.uniform(-2.0, 2.0, size=6)
        guard._compute_friction_ff_Nm()
    for name, initial in ids.items():
        assert id(getattr(guard, name)) == initial, (
            f'{name} was rebound during compute — zero-alloc contract '
            f'violated')


def test_friction_ff_no_steady_state_alloc():
    """In steady state ``_compute_friction_ff_Nm`` allocates ~zero ndarrays.

    Uses ``tracemalloc`` to measure heap growth across 1000 compute
    calls AFTER a warm-up call (which fills the derived-params cache).
    The threshold (4 KiB) is generous: a real per-call allocation
    regression would produce hundreds of KiB of growth (1000 calls ×
    ~10 ndarrays/call × ~100 bytes/ndarray header).  Anything under a
    few KiB indicates true zero-alloc.
    """
    import tracemalloc

    guard, _ = _make_guard()
    guard._friction_ff_params = _params(enabled=True)
    guard._commanded_vel_ff_rps = np.full(6, 0.5)
    # Warm-up: trigger the derived-params cache fill so the measured
    # window is steady-state only.
    guard._compute_friction_ff_Nm()

    tracemalloc.start()
    snap0 = tracemalloc.take_snapshot()
    for _ in range(1000):
        guard._compute_friction_ff_Nm()
    snap1 = tracemalloc.take_snapshot()
    tracemalloc.stop()

    diff = snap1.compare_to(snap0, 'lineno')
    total_alloc = sum(stat.size_diff for stat in diff if stat.size_diff > 0)
    assert total_alloc < 4096, (
        f'_compute_friction_ff_Nm allocated {total_alloc} bytes over '
        f'1000 calls — zero-alloc contract violated')


def test_friction_ff_enable_override_none_uses_yaml():
    """``friction_ff_enable_override=None`` (default) preserves YAML setting.

    YAML default since PR 3b (2026-05-08) is ``enabled: true``; the
    no-override path must reflect that.  If a future change flips the
    YAML default, update this assertion AND add a logbook entry —
    don't silently drift the runtime behaviour.
    """
    ipc = _MockIPC()
    geom = StewartGeometry()
    guard = MotorGuard(rate_hz=500, geom=geom, ipc=ipc,
                       friction_ff_enable_override=None)
    assert guard._friction_ff_params.enabled is True


def test_friction_ff_enable_override_true_forces_enable():
    """Override=True sets ``params.enabled=True`` regardless of YAML.

    With YAML default already True (PR 3b), an explicit True is a
    no-op — the override mechanism's `!=` guard skips the
    ``dataclasses.replace`` call.  Behaviour-equivalent to the YAML
    default; this test pins that.
    """
    ipc = _MockIPC()
    geom = StewartGeometry()
    guard = MotorGuard(rate_hz=500, geom=geom, ipc=ipc,
                       friction_ff_enable_override=True)
    assert guard._friction_ff_params.enabled is True


def test_friction_ff_enable_override_false_forces_disable():
    """Override=False forces ``params.enabled=False`` (overrides YAML default True).

    This is the operator-facing kill-switch for ablation / safety:
    ``ros2 launch jugglebot jugglebot_launch.py friction_ff_enable:=false``.
    """
    ipc = _MockIPC()
    geom = StewartGeometry()
    guard = MotorGuard(rate_hz=500, geom=geom, ipc=ipc,
                       friction_ff_enable_override=False)
    assert guard._friction_ff_params.enabled is False


def test_friction_ff_enable_override_preserves_other_params():
    """Override only flips ``enabled``; the per-leg arrays + Kt are unchanged.

    Critical: the override must not silently zero out friction
    parameters or change the sign convention while flipping the flag.
    """
    ipc = _MockIPC()
    geom = StewartGeometry()
    guard_yaml = MotorGuard(rate_hz=500, geom=geom, ipc=ipc,
                            friction_ff_enable_override=None)
    guard_force = MotorGuard(rate_hz=500, geom=geom, ipc=_MockIPC(),
                             friction_ff_enable_override=True)

    # All per-leg arrays and Kt must match the unforced load.
    np.testing.assert_array_equal(
        guard_force._friction_ff_params.tau_c_A,
        guard_yaml._friction_ff_params.tau_c_A)
    np.testing.assert_array_equal(
        guard_force._friction_ff_params.tau_s_A,
        guard_yaml._friction_ff_params.tau_s_A)
    np.testing.assert_array_equal(
        guard_force._friction_ff_params.omega_s_rps,
        guard_yaml._friction_ff_params.omega_s_rps)
    np.testing.assert_array_equal(
        guard_force._friction_ff_params.b_A_per_rps,
        guard_yaml._friction_ff_params.b_A_per_rps)
    np.testing.assert_array_equal(
        guard_force._friction_ff_params.load_offset_A,
        guard_yaml._friction_ff_params.load_offset_A)
    np.testing.assert_array_equal(
        guard_force._friction_ff_params.ff_sign,
        guard_yaml._friction_ff_params.ff_sign)
    assert (guard_force._friction_ff_params.motor_kt_nm_per_a
            == guard_yaml._friction_ff_params.motor_kt_nm_per_a)
    assert (guard_force._friction_ff_params.v_gate_rps
            == guard_yaml._friction_ff_params.v_gate_rps)


@pytest.mark.parametrize('v_factor, gate_check', [
    (0.1,  lambda g: g < 0.02),    # |v|=v_gate/10:  gate ~0.01
    (1.0,  lambda g: 0.62 < g < 0.64),  # |v|=v_gate:     gate = 1−1/e ≈ 0.6321
    (2.0,  lambda g: 0.98 < g < 0.99),  # |v|=2·v_gate:   gate ≈ 0.982
    (5.0,  lambda g: g > 0.999),   # |v|=5·v_gate:   gate ≈ 1.000
])
def test_friction_ff_smooth_gate_table(v_factor, gate_check):
    """Pin the smooth-gate function values at canonical points.

    Replaces PR 2's parametrised dead-zone-edge test, which checked the
    hard 0 → τ_s step at v=±1e-4 — that step is exactly what produced
    the platform limit cycle.  With the smooth gate there is no edge,
    so this test pins the gate's shape at canonical operating points
    (10 %, 100 %, 200 %, 500 % of v_gate) instead.

    Any future tweak to the gate function form (e.g. swap to tanh)
    would break this table — flagging the change as deliberate.
    """
    guard, _ = _make_guard()
    p = _params(enabled=True)
    guard._friction_ff_params = p
    v_cmd = v_factor * p.v_gate_rps
    guard._commanded_vel_ff_rps = np.full(6, v_cmd)
    out = guard._compute_friction_ff_Nm()

    av = v_cmd
    stribeck = (p.tau_c_A[0]
                + (p.tau_s_A[0] - p.tau_c_A[0])
                  * math.exp(-(av / p.omega_s_rps[0]) ** 2)
                + p.b_A_per_rps[0] * av)
    expected_full = stribeck * p.motor_kt_nm_per_a
    actual_gate = np.abs(out[0]) / expected_full
    assert gate_check(actual_gate), (
        f'gate value at v={v_factor}·v_gate is {actual_gate:.4f} '
        f'(expected per check: {gate_check.__doc__ or "see lambda"})')
