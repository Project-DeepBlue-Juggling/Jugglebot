"""FW 22 hand acceleration torque feedforward (unit U2b) — twin scenarios.

The twin (``tools/probes/.../hermite_xref/teensy_interp.py``) mirrors the
firmware block ``tau = fade * sat(Ks * J * 2pi * a_cmd + bias)``. These tests
drive it at 500 Hz through every transition the torque path must survive
without a torque STEP: stream start, K slew, late arrivals + a dropped frame
(the grace rejoin), cover exhaustion -> C2 stop -> hold, lead-clamp engage and
clear, stroke clip, recovery slew — plus saturation and the legacy tau == 0
guarantee. The firmware-side twins (wire bytes, drain, heartbeat bit 14) are
native cases in ``tests/firmware/native/test_leg_interp.cpp``.
"""

from __future__ import annotations

import math
import os
import re
import sys

_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(_REPO, "tools", "probes", "teensy_link_profiling", "hermite_xref"))

import teensy_interp as ti  # noqa: E402

T = 0.025
DT_US = 2000
J2PI = ti.HAND_TORQUE_FF_J_2PI
KSTEP = ti.HAND_FF_GAIN_SLEW_PER_S * ti.HAND_TORQUE_TICK_S
FSTEP = ti.HAND_TORQUE_FADE_PER_S * ti.HAND_TORQUE_TICK_S   # 0.1 of |raw| per tick
# K = 1 on the 1500 rev/s^2 plan: curve term J2pi*62 + fade term FSTEP*J2pi*1500 (~4.1e-3 + 9.9e-3)
FADE_CURVE_BOUND = J2PI * 62.1 + FSTEP * J2PI * 1500.0 + 1e-4


def _plan(amp, period, p0, n=90):
    a = [amp * math.sin(2 * math.pi * k / period) for k in range(n)]
    v = [0.0] * n
    p = [0.0] * n
    v[0] = -amp * period * T / (2 * math.pi)
    p[0] = p0
    for k in range(n - 1):
        v[k + 1] = v[k] + T * (a[k] + a[k + 1]) / 2
        p[k + 1] = p[k] + T * v[k] + T * T * (a[k] / 3 + a[k + 1] / 6)
    return p, v, a


def _plan_pos(P, t0_us, now_us):
    p, v, _ = P
    tau = (now_us - t0_us) / (T * 1e6)
    k = int(math.floor(tau))
    s = tau - k
    d = p[k + 1] - p[k]
    return (p[k] + (s ** 3 - 2 * s ** 2 + s) * T * v[k] + (-2 * s ** 3 + 3 * s ** 2) * d
            + (s ** 3 - s ** 2) * T * v[k + 1])


def _run(P, nframes, t_end_us, K=0.5, bias=0.0, arrival=None, drop=(), fb_fn=None,
         slew_fn=None, k_fn=None):
    tw = ti.TeensyLegInterp([0.0] * 6, [10.0] * 6)
    t0 = 100_000
    arrival = arrival or (lambda k, stamp: stamp - 24_000)
    evs = sorted((arrival(k, t0 + k * 25_000), k) for k in range(nframes) if k not in drop)
    p, v, a = P
    z = [0.0] * 6
    legs = [0.5] * 6
    out = []
    i = 0
    fb = p[0]
    now = 0
    while now < t_end_us:
        while i < len(evs) and evs[i][0] <= now:
            rk, k = evs[i]
            kk = k_fn(k) if k_fn else K
            tw.sched_on_setpoint(t0 + k * 25_000, rk, legs + [p[k]], legs + [p[k + 1]], legs + [p[k + 2]],
                                 z + [v[k]], z + [v[k + 1]], z + [v[k + 2]], has_hand=True,
                                 torque=z + [bias], accel=z + [a[k]], hand_ff_gain=kk)
            i += 1
        if slew_fn is not None:
            tw.hand_recover_slewing = bool(slew_fn(now))
        fbr = fb_fn(now, fb) if fb_fn else fb
        r = tw.tick_hand(now / 1e6, fbr, 0.0, 0.0)
        if r is not None:
            fb = r[0]
        out.append((now, tw.hand_cmd_tau, tw.hand_cmd_acc, tw.hand_ff_fade, tw.hand_ff_ks))
        now += DT_US
    return tw, t0, out


def _max_step(out, col=1):
    return max(abs(out[i][col] - out[i - 1][col]) for i in range(1, len(out)))


def _max_da(out):
    return max(abs(out[i][2] - out[i - 1][2]) for i in range(1, len(out)))


def test_play_tau_is_k_j_2pi_a_and_rate_bounded_from_zero():
    P = _plan(1500.0, 12, 5.0)
    tw, t0, out = _run(P, 44, 100_000 + 42 * 25_000, K=0.5)
    n = 0
    for now, tau, a, fade, ks in out:
        if now >= t0 + 300_000:
            assert fade == 1.0 and ks == 0.5
            assert abs(tau - 0.5 * J2PI * a) < 1e-12
            n += 1
    assert n > 300
    # K*J2pi*(plan jerk*dt ~ 63) + J2pi*1500*kstep
    assert _max_step(out) < 4e-3
    assert tw.hand_tau_clamp_ticks == 0


def test_gain_change_slews_ks_never_steps():
    P = _plan(1500.0, 12, 5.0)
    tw, t0, out = _run(P, 44, 100_000 + 42 * 25_000, k_fn=lambda k: 0.0 if k < 16 else 1.0)
    assert max(abs(out[i][4] - out[i - 1][4]) for i in range(1, len(out))) <= KSTEP + 1e-12
    assert out[-1][4] == 1.0
    assert _max_step(out) < J2PI * (63.0 + 1500.0 * KSTEP / 1.0) + 1e-3


def test_late_arrivals_and_dropped_frame_add_no_step_beyond_the_curve():
    P = _plan(1500.0, 12, 5.0)
    tw, t0, out = _run(P, 44, 100_000 + 40 * 25_000, K=0.5, drop=(17,),
                       arrival=lambda k, stamp: stamp + (1 + (k * 7919) % 15) * 1000)
    live = [o for o in out if o[0] >= t0 + 200_000]
    # The torque path itself adds at most the fade/K/bias rate terms on top of
    # K*J2pi*|delta a_cmd| (the grace rejoin's a-step is the curve's, U2a residual 3/4).
    for i in range(1, len(live)):
        da = abs(live[i][2] - live[i - 1][2])
        assert abs(live[i][1] - live[i - 1][1]) <= 0.5 * J2PI * da + J2PI * 3000 * KSTEP + 1e-9
    assert _max_step(live) < 0.04
    assert tw.sched_stops == 0 if hasattr(tw, "sched_stops") else True


def test_exhaustion_stop_and_hold_keep_tau_continuous_and_end_at_zero():
    P = _plan(1500.0, 12, 5.0)
    tw, t0, out = _run(P, 20, 100_000 + 20 * 25_000 + 900_000, K=1.0)
    bound = J2PI * ti.STREAM_STOP_HAND_JERK_RPS3 * 0.002 + 5e-3
    assert _max_step(out) < bound
    assert tw.sched_phase(1) == 3          # HOLD
    assert out[-1][1] == 0.0


def test_lead_clamp_engage_fades_to_zero_and_clear_ramps_back():
    P = _plan(1500.0, 12, 5.0)
    e0, e1 = 100_000 + 400_000, 100_000 + 500_000
    fb_fn = (lambda now, fb: _plan_pos(P, 100_000, now) - 3.0 if e0 <= now < e1 else fb)
    tw, t0, out = _run(P, 44, 100_000 + 42 * 25_000, K=1.0, fb_fn=fb_fn)
    assert tw.hand_lead_clamp_ticks > 0
    assert _max_step(out) < FADE_CURVE_BOUND
    for now, tau, a, fade, ks in out:
        if e0 + 22_000 <= now < e1:
            assert tau == 0.0
        if e1 + 30_000 <= now < t0 + 40 * 25_000:
            assert abs(tau - J2PI * a) < 1e-12


def test_stroke_clip_fades_the_feedforward():
    P = _plan(1500.0, 12, 3.0)            # dips ~0.4 rev below the retract stop
    tw, t0, out = _run(P, 44, 100_000 + 42 * 25_000, K=1.0)
    assert tw.hand_clip_ticks > 0
    assert tw.hand_tau_fade_ticks > 0
    assert _max_step(out) < FADE_CURVE_BOUND


def test_recovery_slew_flag_fades_the_feedforward():
    P = _plan(1500.0, 12, 5.0)
    w0, w1 = 100_000 + 400_000, 100_000 + 450_000
    tw, t0, out = _run(P, 44, 100_000 + 42 * 25_000, K=1.0, slew_fn=lambda now: w0 <= now < w1)
    assert _max_step(out) < FADE_CURVE_BOUND
    assert any(tau == 0.0 and fade == 0.0 for now, tau, a, fade, ks in out if w0 + 22_000 <= now < w1)


def test_saturation_bounds_the_sum():
    P = _plan(3000.0, 8, 5.4)
    tw, t0, out = _run(P, 40, 100_000 + 38 * 25_000, K=1.5)
    peak = max(abs(o[1]) for o in out)
    assert peak <= ti.HAND_TORQUE_FF_CLAMP_NM + 1e-12
    assert peak >= ti.HAND_TORQUE_FF_CLAMP_NM - 1e-12
    assert tw.hand_tau_clamp_ticks > 0


def test_bias_is_clamped_and_rate_limited():
    P = _plan(0.0, 12, 5.0)               # a == 0: tau is the bias alone
    tw, t0, out = _run(P, 44, 100_000 + 42 * 25_000, K=0.0, bias=0.2)
    assert abs(out[-1][1] - ti.HAND_TORQUE_BIAS_CLAMP_NM) < 1e-12
    assert _max_step(out) <= ti.HAND_TORQUE_BIAS_RATE_NM_PER_S * 0.002 + FSTEP * 0.05 + 1e-12


def test_legacy_stream_tau_is_exactly_zero():
    tw = ti.TeensyLegInterp([0.0] * 6, [10.0] * 6)
    fb = 3.0
    for n in range(80):
        tw.latch_hand(3.0 + 0.2 * n, 8.0, n * 0.025, u1=3.2 + 0.2 * n, v1=8.0)
        for k in range(12):
            r = tw.tick_hand(n * 0.025 + k * 0.002, fb, 0.0, 0.0)
            assert tw.hand_cmd_tau == 0.0
            if r is not None:
                fb = r[0]


def test_twin_constants_pinned_to_config_and_firmware():
    sys.path.insert(0, os.path.join(_REPO, "config", "generated"))
    import hardware_config as hw  # noqa: E402
    assert ti.HAND_TORQUE_FF_CLAMP_NM == hw.JB_TRAJ_STREAM_HAND_TORQUE_FF_CLAMP_NM
    assert ti.HAND_FF_GAIN_SLEW_PER_S == hw.JB_TRAJ_STREAM_HAND_FF_GAIN_SLEW_PER_S
    assert ti.HAND_TORQUE_BIAS_CLAMP_NM == hw.JB_TRAJ_STREAM_HAND_TORQUE_BIAS_CLAMP_NM
    assert ti.HAND_TORQUE_BIAS_RATE_NM_PER_S == hw.JB_TRAJ_STREAM_HAND_TORQUE_BIAS_RATE_NM_PER_S
    assert ti.HAND_TORQUE_FADE_PER_S == hw.JB_TRAJ_STREAM_HAND_TORQUE_FADE_PER_S
    assert ti.HAND_MEASURED_REFLECTED_INERTIA_KGM2 == hw.HAND_ENV_MEASURED_REFLECTED_INERTIA_KGM2
    cfg = open(os.path.join(_REPO, "ros_ws", "src", "jugglebot", "Teensy_code_canbridge",
                            "canbridge_config.h")).read()
    for name in ("STREAM_HAND_TORQUE_FF_CLAMP_NM", "STREAM_HAND_FF_GAIN_SLEW_PER_S",
                 "STREAM_HAND_TORQUE_BIAS_CLAMP_NM", "STREAM_HAND_TORQUE_BIAS_RATE_NM_PER_S",
                 "STREAM_HAND_TORQUE_FADE_PER_S"):
        assert f"TrajOp::{name}" in cfg
    assert re.search(r"HAND_TORQUE_FF_J_2PI\s*=\s*HandEnv::MEASURED_REFLECTED_INERTIA_KGM2", cfg)
    # the ODrive json the operator re-applies must match the generated wire scale
    import json
    j = json.load(open(os.path.join(_REPO, "config", "ODrive config Files", "odrive_pro_hand_config.json")))
    proto = open(os.path.join(_REPO, "config", "generated", "protocol_config.py")).read()
    m = re.search(r"INPUT_SCALE_HAND_TOR\s*=\s*([0-9.]+)", proto)
    assert m and float(m.group(1)) == 1000.0
    assert j["axis0"]["config"]["can"]["input_torque_scale"] == 1000
