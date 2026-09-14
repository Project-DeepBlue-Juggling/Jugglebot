"""FW 22 scheduled playback — 500 Hz continuity of the Python twin.

``teensy_interp.py``'s ``sched_*`` block mirrors ``leg_interp.cpp``'s scheduled
path line for line (the native harness, ``native/test_leg_interp.cpp``, drives the
compiled firmware through the same scenarios). These tests pin the C2 claims of
the hand C2 spec (2026-09-14) at the interpolator's own 500 Hz tick:

* knot-aligned stamped frames with random arrival lateness 0-40 ms, a single
  dropped frame, a late burst and the first frame of a stream all replay the
  plan EXACTLY (float64), so acceleration is continuous by construction;
* cover exhaustion runs the C2 stop (finite-difference jerk bounded, no
  position/velocity step, peak |a| <= max(A_stop, |a_entry|)) and then holds;
* a discontinuous resume out of the hold is refused while armed (no position
  step) and accepted once the output is disabled;
* the stop's peak acceleration and jerk hold across a grid of entry states up
  to 200 rev/s and 3500 rev/s^2;
* a legacy (unstamped) stream is untouched, and the arrival-phase legacy path
  FAILS the same continuity check (the check has teeth).

The C2 test plan is a piecewise-linear-acceleration knot sequence, so the plan
itself is exactly C2 and the cubic Hermite over (p, v) knots reproduces it.
"""

from __future__ import annotations

import math
import os
import random
import re
import sys

import pytest

_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(_REPO, "tools", "probes", "teensy_link_profiling", "hermite_xref"))

import teensy_interp as ti  # noqa: E402

T_US = ti.SCHED_SEG_US
TICK_US = 2000
T0_US = 1_000_000_000          # stamp of knot 0 (bridge wall == mono in the twin)
LEG_SCALE = 0.01               # legs ride the same plan, scaled (exercises the leg group)


def c2_plan(n, amp_a=2500.0, period_knots=12, p0=5.0):
    """Knot (p, v, a) of a C2 piecewise cubic: accel linear between knots."""
    T = ti.SEGMENT_T_S
    a = [amp_a * math.sin(2.0 * math.pi * k / period_knots) for k in range(n)]
    v = [-amp_a * period_knots * T / (2.0 * math.pi)]          # zero-mean velocity
    p = [p0]
    for k in range(n - 1):
        v.append(v[k] + T * (a[k] + a[k + 1]) / 2.0)
        p.append(p[k] + T * v[k] + T * T * (a[k] / 3.0 + a[k + 1] / 6.0))
    return p, v, a


def plan_jerk(a):
    return max(abs(a[k + 1] - a[k]) for k in range(len(a) - 1)) / ti.SEGMENT_T_S


def ref(P, V, now_us):
    tau = (now_us - T0_US) / T_US
    k = int(math.floor(tau))
    return ti.span_eval(P[k], V[k], P[k + 1], V[k + 1], tau - k)


class Driver:
    def __init__(self, P, V, A):
        self.P, self.V, self.A = P, V, A
        self.it = ti.TeensyLegInterp([-1e9] * 6, [1e9] * 6)
        self.hand = []      # (now_us, p, v, a) raw hand ladder, from the first transmitted tick
        self.leg = []
        self.fb = 0.0

    def send(self, k, recv_us, **kw):
        P, V, A = self.P, self.V, self.A
        lane = lambda X, j: [LEG_SCALE * X[j]] * 6 + [X[j]]  # noqa: E731
        return self.it.sched_on_setpoint(
            T0_US + k * T_US, recv_us, lane(P, k), lane(P, k + 1), lane(P, k + 2),
            lane(V, k), lane(V, k + 1), lane(V, k + 2), has_hand=True,
            accel=[0.0] * 6 + [A[k]], **kw)

    def run(self, events, t_from_us, t_to_us, phase_us=700):
        """events: list of (arrival_us, k), ingested before the tick they precede."""
        events = sorted(events)
        ei = 0
        now = t_from_us + phase_us
        while now < t_to_us:
            while ei < len(events) and events[ei][0] <= now:
                self.send(events[ei][1], events[ei][0])
                ei += 1
            self.it.tick(now / 1e6, [0.0] * 6)
            out = self.it.tick_hand(now / 1e6, fb_rev=self.fb)
            if out is not None:
                self.hand.append((now, self.it.hand_raw_pos, self.it.hand_raw_vel, self.it.hand_raw_acc))
                self.leg.append((now, self.it.raw_pos[0], self.it.raw_vel[0], self.it.cmd_acc[0]))
                self.fb = self.it.hand_raw_pos
            now += TICK_US


def c2_violations(samples, jerk_bound):
    """500 Hz finite-difference C2 check: accel steps bounded by jerk·h, and no
    velocity or position step beyond what that jerk can produce in one tick."""
    bad = []
    for (t0, p0, v0, a0), (t1, p1, v1, a1) in zip(samples, samples[1:]):
        h = (t1 - t0) * 1e-6
        if abs(a1 - a0) > jerk_bound * h + 1e-6:
            bad.append(("acc", t1, a1 - a0))
        if abs(v1 - v0 - h * (a0 + a1) / 2.0) > jerk_bound * h * h / 4.0 + 1e-7:
            bad.append(("vel", t1, v1 - v0))
        if abs(p1 - p0 - h * (v0 + v1) / 2.0 - h * h * (a0 - a1) / 12.0) > jerk_bound * h ** 3 / 6.0 + 1e-9:
            bad.append(("pos", t1, p1 - p0))
    return bad


def assert_exact_plan(d, samples=None, legs=True):
    samples = d.hand if samples is None else samples
    assert samples, "the hand lane never transmitted"
    for now, p, v, a in samples:
        pr, vr, ar = ref(d.P, d.V, now)
        assert abs(p - pr) < 1e-9 and abs(v - vr) < 1e-7 and abs(a - ar) < 1e-5, (now, p - pr, v - vr, a - ar)
    if legs:
        for now, p, v, a in d.leg:
            pr, vr, ar = ref(d.P, d.V, now)
            assert abs(p - LEG_SCALE * pr) < 1e-11 and abs(a - LEG_SCALE * ar) < 1e-7


def nominal_events(n_frames, lateness_ms=lambda k: 0.0, drop=(), lead_us=0):
    """Arrivals of frames 0..n-1. The host emit lead is 0 knots (a frame for knot
    T is sent at ~T), so ``lateness`` is measured from the frame's OWN stamp."""
    ev, prev = [], 0
    for k in range(n_frames):
        if k in drop:
            continue
        arr = max(prev, T0_US + k * T_US - lead_us + int(lateness_ms(k) * 1000))
        ev.append((arr, k))
        prev = arr
    return ev


# ── exact replay under timing faults ─────────────────────────────────────────
@pytest.mark.parametrize("lead_us,late_lo,late_hi", [
    (0, 1.0, 15.0),        # THE normal path: emit lead 0, frames land 1-15 ms after their stamp
    (T_US, 0.0, 40.0),     # stress: a 1-knot lead absorbs 0-40 ms of lateness exactly
])
@pytest.mark.parametrize("seed", range(5))
def test_knot_aligned_frames_with_random_lateness_replay_the_plan_exactly(seed, lead_us, late_lo, late_hi):
    P, V, A = c2_plan(64)
    rng = random.Random(seed)
    d = Driver(P, V, A)
    d.run(nominal_events(60, lambda k: rng.uniform(late_lo, late_hi), lead_us=lead_us),
          T0_US - 40_000, T0_US + 58 * T_US)
    assert_exact_plan(d)
    assert not c2_violations(d.hand, plan_jerk(A) * 1.01)
    it = d.it
    assert it.sched_stops == 0 and it.sched_expired == 0 and it.sched_refused == 0
    assert it._sg[1].dp_max < 1e-9 and it._sg[1].da_max < 1e-5
    assert it._sg[1].promo_over == 0 and it._sg[0].promo_over == 0


def test_a_single_dropped_frame_is_covered_by_a_one_knot_lead():
    P, V, A = c2_plan(64)
    d = Driver(P, V, A)
    d.run(nominal_events(60, lambda k: 3.0, drop={20}, lead_us=T_US), T0_US - 40_000, T0_US + 58 * T_US)
    assert_exact_plan(d)
    assert d.it.sched_stops == 0


@pytest.mark.parametrize("late_ms", [1.0, 3.0, 8.0, 15.0])
def test_a_single_dropped_frame_at_emit_lead_0_rides_the_grace_and_rejoins_the_plan(late_ms):
    # Lead 0: frame 19 covers [19T, 21T); frame 20 is dropped and frame 21 lands
    # late_ms after 21T — inside the grace window, where frame 19's span-2 cubic
    # continues. No stop; the rejoin step is the plan's own knot jerk change x δ.
    P, V, A = c2_plan(64)
    d = Driver(P, V, A)
    d.run(nominal_events(60, lambda k: late_ms, drop={20}), T0_US - 40_000, T0_US + 58 * T_US)
    it = d.it
    assert it.sched_stops == 0 and it.sched_refused == 0
    rp, rv, ra = ti.SCHED_RESUME_TOL_HAND
    assert it._sg[1].dp_max <= rp and it._sg[1].dv_max <= rv and it._sg[1].da_max <= ra
    rejoin = T0_US + 21 * T_US + int(late_ms * 1000) + TICK_US
    bad = c2_violations(d.hand, plan_jerk(A) * 1.01)
    assert all(T0_US + 21 * T_US <= b[1] <= rejoin for b in bad), bad
    assert_exact_plan(d, samples=[s for s in d.hand if s[0] >= rejoin], legs=False)
    assert_exact_plan(d, samples=[s for s in d.hand if s[0] < T0_US + 21 * T_US], legs=False)


def test_a_late_burst_is_played_at_its_true_phase():
    P, V, A = c2_plan(64)
    d = Driver(P, V, A)
    ev = [e for e in nominal_events(60, lambda k: 2.0) if e[1] not in (20, 21)]
    burst = T0_US + 20 * T_US + 20_000               # lead 0: 20 lands 20 ms late, 21 early, together
    ev += [(burst, 20), (burst, 21)]
    d.run(ev, T0_US - 40_000, T0_US + 58 * T_US)
    assert_exact_plan(d)
    assert d.it.sched_stops == 0 and d.it.sched_expired == 0


def test_the_first_frame_of_a_stream_starts_at_its_true_phase():
    P, V, A = c2_plan(64)
    d = Driver(P, V, A)
    ev = [(T0_US + 30 * T_US + 10_000, 30)] + [e for e in nominal_events(60, lambda k: 4.0) if e[1] > 30]
    d.run(ev, T0_US + 29 * T_US, T0_US + 58 * T_US)
    assert d.hand[0][0] >= T0_US + 30 * T_US + 10_000     # silent until the first frame
    assert_exact_plan(d)
    assert d.it._sg[1].promo_over == 0


def test_expired_and_stale_frames_are_dropped_and_counted():
    P, V, A = c2_plan(64)
    d = Driver(P, V, A)
    d.run(nominal_events(12, lead_us=T_US), T0_US - 40_000, T0_US + 11 * T_US)
    d.send(3, T0_US + 11 * T_US)                       # cover long gone
    d.run([], T0_US + 11 * T_US, T0_US + 11 * T_US + 4000)
    assert d.it.sched_expired == 1
    playing = d.it._sg[1].frame_t_start_us
    d.send(10, T0_US + 11 * T_US + 5000)              # due, within its cover, but older than frame 11
    d.run([], T0_US + 11 * T_US + 4000, T0_US + 11 * T_US + 10_000)
    # counted per lane group: the frame carries the legs AND the hand
    assert d.it.sched_stale == 2 and d.it._sg[1].frame_t_start_us == playing


@pytest.mark.parametrize("late_hi_ms", [40.0, 80.0])
def test_stress_lead0_random_lateness_never_steps(late_hi_ms):
    """Emit lead 0 with 0-40 ms lateness (the stress case: cover + grace holds it)
    and 0-80 ms (forces exhaustion, stops and refusals). Whatever the resume rule
    decides, the emitted curve never steps beyond the stop jerk plus the resume
    tolerances, and every stop is accounted for."""
    P, V, A = c2_plan(64)
    J = max(plan_jerk(A), ti.STREAM_STOP_HAND_JERK_RPS3)
    rp, rv, ra = ti.SCHED_RESUME_TOL_HAND
    for seed in range(5):
        rng = random.Random(100 + seed)
        d = Driver(P, V, A)
        d.run(nominal_events(60, lambda k: rng.uniform(0.0, late_hi_ms)), T0_US - 40_000, T0_US + 58 * T_US)
        for (t0, p0, v0, a0), (t1, p1, v1, a1) in zip(d.hand, d.hand[1:]):
            h = (t1 - t0) * 1e-6
            assert abs(a1 - a0) <= J * h + ra
            assert abs(v1 - v0 - h * (a0 + a1) / 2.0) <= J * h * h / 4.0 + rv
            assert abs(p1 - p0 - h * (v0 + v1) / 2.0 - h * h * (a0 - a1) / 12.0) <= J * h ** 3 / 6.0 + rp
        it = d.it
        if late_hi_ms <= 40.0:
            assert it.sched_stops == 0 and it.sched_refused == 0
        else:
            assert it.sched_stops >= 1


def test_a_frame_inside_the_grace_window_is_promoted_without_a_stop():
    P, V, A = c2_plan(64)
    d = Driver(P, V, A)
    ev = [e for e in nominal_events(60, lambda k: 3.0) if e[1] != 21]
    ev.append((T0_US + 21 * T_US + 40_000, 21))       # 40 ms late: 15 ms into frame 20's grace
    d.run(ev, T0_US - 40_000, T0_US + 58 * T_US)
    it = d.it
    assert it.sched_stops == 0 and it.sched_refused == 0 and it.sched_phase(1) == 1
    assert_exact_plan(d, samples=[s for s in d.hand if s[0] >= T0_US + 21 * T_US + 43_000], legs=False)


def test_a_frame_past_the_grace_window_meets_the_stop_and_is_refused_while_armed():
    P, V, A = c2_plan(64)
    d = Driver(P, V, A)
    ev = [e for e in nominal_events(60, lambda k: 3.0) if e[1] not in (21, 22)]
    ev.append((T0_US + 22 * T_US + 30_000, 22))       # frame 20's cover + grace ended at 22T+20 ms
    d.run(ev, T0_US - 40_000, T0_US + 30 * T_US)
    it = d.it
    assert it.sched_stops == 2 and it.sched_refused >= 1 and it._sg[1].latched
    assert it.sched_phase(1) in (2, 3)


# ── cover exhaustion: the C2 stop, the hold, the resume rule ─────────────────
@pytest.mark.parametrize("last_frame", [12, 14, 17])
def test_cover_exhaustion_runs_a_c2_stop_then_holds_and_refuses_a_step(last_frame):
    P, V, A = c2_plan(80)
    d = Driver(P, V, A)
    K = last_frame
    t_exh = T0_US + (K + 2) * T_US
    t_exh = T0_US + K * T_US + ti.SCHED_EXHAUST_US     # last stamp + 2T + grace
    d.run(nominal_events(K + 1, lambda k: 5.0), T0_US - 40_000, t_exh + 600_000)
    it, g = d.it, d.it._sg[1]
    assert it.sched_stops == 2                          # legs group + hand group
    assert it.sched_stop_capped == 0
    assert it.sched_phase(1) == 3 and it.sched_phase(0) == 3
    p_entry, v_entry, a_entry = ti.span_eval(P[K + 1], V[K + 1], P[K + 2], V[K + 2], ti.SCHED_S_MAX)
    stop = [s for s in d.hand if s[0] >= t_exh]
    assert abs(stop[0][3] - ti.stop_eval(p_entry, v_entry, a_entry, g.stop_D,
                                         (stop[0][0] - t_exh) * 1e-6)[2]) < 1e-6
    peak = max(abs(s[3]) for s in stop)
    assert peak <= max(ti.STREAM_STOP_HAND_ACCEL_RPS2, abs(a_entry)) * (1.0 + 1e-3)
    jb = max(plan_jerk(A), ti.STREAM_STOP_HAND_JERK_RPS3) * 1.001
    assert not c2_violations(d.hand, jb), c2_violations(d.hand, jb)[:3]
    assert not c2_violations(d.leg, max(plan_jerk(A) * LEG_SCALE, ti.STREAM_STOP_LEG_JERK_RPS3) * 1.001)
    hold_p = d.hand[-1][1]
    assert d.hand[-1][2] == 0.0 and d.hand[-1][3] == 0.0

    # The plan resumes somewhere else while ARMED: refused, latched, no step.
    t1 = t_exh + 600_000
    k_res = (t1 - T0_US) // T_US + 1
    n_before = len(d.hand)
    d.run([(t1 + 1000, k_res), (t1 + 1000 + T_US, k_res + 1)], t1, t1 + 80_000)
    assert it.sched_refused >= 1 and g.latched
    assert all(s[1] == hold_p and s[2] == 0.0 for s in d.hand[n_before:])
    assert it.hand_base_pos == P[k_res + 1]            # the guard now reads the refused command
    # Disarmed: nothing reaches the wire, so the discontinuous frame is accepted.
    it.sched_out_en = False
    d.run([(t1 + 81_000, k_res + 3)], t1 + 80_000, t1 + 120_000)
    assert not g.latched and it.sched_phase(1) == 1


def test_a_continuous_resume_out_of_a_hold_is_accepted():
    n = 40
    P, V, A = [3.0] * n, [0.0] * n, [0.0] * n            # a rest-terminal plan, stationary
    d = Driver(P, V, A)
    ev = nominal_events(6) + [e for e in nominal_events(30) if e[1] >= 20]
    d.run(ev, T0_US - 40_000, T0_US + 28 * T_US)
    it = d.it
    assert it.sched_stops == 2 and it.sched_refused == 0 and it.sched_resumes >= 1
    assert not it._sg[1].latched and it.sched_phase(1) == 1
    assert all(s[1] == 3.0 for s in d.hand)


def test_arm_edge_restarts_a_held_leg_group_and_clears_the_hand_group():
    P, V, A = c2_plan(40)
    d = Driver(P, V, A)
    d.run(nominal_events(10), T0_US - 40_000, T0_US + 20 * T_US)
    it = d.it
    assert it.sched_phase(0) == 3 and it.sched_phase(1) == 3
    it.arm_edge()
    assert it.sched_phase(1) == 0 and not it.hand_active and it._sg[0].restart
    d.run([(T0_US + 20 * T_US + 1000, 21)], T0_US + 20 * T_US, T0_US + 21 * T_US + 4000)
    assert it.sched_slew_rearms == 1 and it.sched_refused == 0 and it.sched_phase(0) == 1


# ── the stop profile itself ──────────────────────────────────────────────────
@pytest.mark.parametrize("lanes", [
    ("hand", 200.0, 3500.0, ti.STREAM_STOP_HAND_ACCEL_RPS2, ti.STREAM_STOP_HAND_JERK_RPS3),
    ("leg", 16.5, 500.0, ti.STREAM_STOP_LEG_ACCEL_RPS2, ti.STREAM_STOP_LEG_JERK_RPS3),
])
def test_stop_peak_acceleration_and_jerk_across_the_entry_grid(lanes):
    _, vmax, amax, A_lim, J_lim = lanes
    n_tau = 4001
    worst_D = 0.0
    for iv in range(-8, 9):
        for ia in range(-7, 8):
            v0, a0 = vmax * iv / 8.0, amax * ia / 7.0
            D, capped = ti.stop_duration([v0], [a0], A_lim, J_lim)
            assert not capped, (v0, a0)
            if v0 == 0.0 and a0 == 0.0:
                assert D == 0.0
                continue
            worst_D = max(worst_D, D)
            alim = max(A_lim, abs(a0)) * (1.0 + 2e-4)
            p_prev = None
            for j in range(n_tau):
                p, v, a = ti.stop_eval(0.0, v0, a0, D, D * j / (n_tau - 1) * (1.0 - 1e-12))
                assert abs(a) <= alim, (v0, a0, D, j, a)
                if p_prev is not None:
                    assert abs(p - p_prev) <= (abs(v0) + abs(a0) * D) * D / (n_tau - 1) * 2.0 + 1e-12
                p_prev = p
            # jerk is linear in tau: its endpoints bound it
            x = v0 / D
            assert abs(-6.0 * x - 4.0 * a0) / D <= J_lim * (1.0 + 1e-6)
            assert abs(6.0 * x + 2.0 * a0) / D <= J_lim * (1.0 + 1e-6)
            p_end, v_end, a_end = ti.stop_eval(0.0, v0, a0, D, D)
            assert v_end == 0.0 and a_end == 0.0
            _, v_near, a_near = ti.stop_eval(0.0, v0, a0, D, D * (1.0 - 1e-9))
            assert abs(v_near) < 1e-5 and abs(a_near) < 1e-2
            assert abs(p_end - D * (v0 * 0.5 + a0 * D / 12.0)) < 1e-12
            # minimality: a 1 % shorter stop violates a bound
            assert not ti.stop_feasible([v0], [a0], D * 0.99, A_lim, J_lim)
    assert worst_D < ti.SCHED_STOP_MAX_S


# ── legacy untouched; the arrival-phase path fails the same check ────────────
def test_demoted_frames_and_the_sched_machinery_leave_a_legacy_stream_bit_identical():
    P, V, A = c2_plan(40)

    def legacy(with_demoted):
        it = ti.TeensyLegInterp([-1e9] * 6, [1e9] * 6)
        out = []
        for j in range(400):
            now = T0_US + 700 + j * TICK_US
            if j % 12 == 0:
                k = j // 12
                if with_demoted:
                    assert it.sched_on_setpoint(T0_US, now, [0.0] * 7, [0.0] * 7, [0.0] * 7,
                                                [0.0] * 7, [0.0] * 7, [0.0] * 7, full=False) == "demoted"
                it.latch_setpoint([P[k]] * 6, [V[k]] * 6, [0.0] * 6, [0.0] * 6, now / 1e6,
                                  u1=[P[k + 1]] * 6, u2=[P[k + 2]] * 6, v1=[V[k + 1]] * 6)
                it.latch_hand(P[k], V[k], now / 1e6, u1=P[k + 1], u2=P[k + 2], v1=V[k + 1])
            out.append((it.tick(now / 1e6, [P[0]] * 6), it.tick_hand(now / 1e6, fb_rev=P[0]),
                        it.hand_raw_pos, it.hand_raw_vel))
        return out, it

    a, _ = legacy(False)
    b, it = legacy(True)
    assert a == b
    assert it.sched_demoted > 0 and it.sched_phase(0) == 0 and it.sched_phase(1) == 0


def test_negative_control_arrival_phase_legacy_playback_breaks_c2():
    P, V, A = c2_plan(64)
    rng = random.Random(7)
    it = ti.TeensyLegInterp([-1e9] * 6, [1e9] * 6)
    arrivals = sorted((T0_US + k * T_US + int(rng.uniform(0, 3000)), k) for k in range(60))
    samples, ai = [], 0
    for j in range(700):
        now = T0_US + 700 + j * TICK_US
        while ai < len(arrivals) and arrivals[ai][0] <= now:
            _, k = arrivals[ai]
            it.latch_hand(P[k], V[k], arrivals[ai][0] / 1e6, u1=P[k + 1], u2=P[k + 2], v1=V[k + 1])
            ai += 1
        if it.tick_hand(now / 1e6, fb_rev=5.0) is not None:
            samples.append((now, it.hand_raw_pos, it.hand_raw_vel, it.hand_raw_acc))
    assert len(c2_violations(samples, plan_jerk(A) * 1.01)) > 20


# ── constants pinned to the firmware ─────────────────────────────────────────
def test_sched_constants_match_the_firmware():
    cb = open(os.path.join(_REPO, "ros_ws", "src", "jugglebot", "Teensy_code_canbridge",
                           "canbridge_config.h"), encoding="utf-8").read()
    hw = open(os.path.join(_REPO, "ros_ws", "src", "jugglebot", "Teensy_code_canbridge",
                           "hardware_config.h"), encoding="utf-8").read()

    def c(name, text=cb):
        m = re.search(r"\b" + name + r"\s*=\s*([-0-9.eE]+)[fu]?\s*;", text)
        assert m, name
        return float(m.group(1))

    assert c("SCHED_QUEUE_LEN") == ti.SCHED_QUEUE_LEN
    assert c("SCHED_MAX_FUTURE_US") == ti.SCHED_MAX_FUTURE_US
    assert (c("SCHED_PROMO_TOL_POS_LEG_REV"), c("SCHED_PROMO_TOL_VEL_LEG_RPS"),
            c("SCHED_PROMO_TOL_ACC_LEG_RPS2")) == pytest.approx(ti.SCHED_PROMO_TOL_LEG)
    assert (c("SCHED_PROMO_TOL_POS_HAND_REV"), c("SCHED_PROMO_TOL_VEL_HAND_RPS"),
            c("SCHED_PROMO_TOL_ACC_HAND_RPS2")) == pytest.approx(ti.SCHED_PROMO_TOL_HAND)
    assert (c("SCHED_RESUME_TOL_POS_LEG_REV"), c("SCHED_RESUME_TOL_VEL_LEG_RPS"),
            c("SCHED_RESUME_TOL_ACC_LEG_RPS2")) == pytest.approx(ti.SCHED_RESUME_TOL_LEG)
    assert (c("SCHED_RESUME_TOL_POS_HAND_REV"), c("SCHED_RESUME_TOL_VEL_HAND_RPS"),
            c("SCHED_RESUME_TOL_ACC_HAND_RPS2")) == pytest.approx(ti.SCHED_RESUME_TOL_HAND)
    assert c("SCHED_STOP_BRACKET_S") == pytest.approx(ti.SCHED_STOP_BRACKET_S)
    assert c("SCHED_STOP_MAX_S") == pytest.approx(ti.SCHED_STOP_MAX_S)
    assert c("SCHED_STOP_BISECT_ITERS") == ti.SCHED_STOP_BISECT_ITERS
    assert c("SCHED_STOP_REL_EPS") == pytest.approx(ti.SCHED_STOP_REL_EPS)
    assert c("HAND_FF_GAIN_MAX") == pytest.approx(ti.HAND_FF_GAIN_MAX)
    assert c("SCHED_GRACE_S") == pytest.approx(ti.SCHED_GRACE_S)
    assert ti.SCHED_GRACE_US == round(ti.SCHED_GRACE_S * 1e6)
    assert c("STREAM_STOP_HAND_ACCEL_RPS2", hw) == ti.STREAM_STOP_HAND_ACCEL_RPS2
    assert c("STREAM_STOP_HAND_JERK_RPS3", hw) == ti.STREAM_STOP_HAND_JERK_RPS3
    assert c("STREAM_STOP_LEG_ACCEL_RPS2", hw) == ti.STREAM_STOP_LEG_ACCEL_RPS2
    assert c("STREAM_STOP_LEG_JERK_RPS3", hw) == ti.STREAM_STOP_LEG_JERK_RPS3
    assert ti.SCHED_SEG_US == round(ti.SEGMENT_T_S * 1e6)
