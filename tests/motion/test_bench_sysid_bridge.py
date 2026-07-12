"""Unit tests for the Path-BRIDGE pure logic in ``sysid_lib``.

Covers the bridge-specific half of ``tests/hardware/bench_leg_sysid.py`` — the
position-domain knot-ramp stimulus generators (bounded by the lead clamp), the
telemetry-rate estimator + honest-chirp-frequency bounds, and the guard-latch
backoff state machine — none of which touch a socket, so they run in the ordinary
pytest suite. Sibling to ``test_bench_sysid_logic.py`` (which covers the shared
onset/ladder/step-metric logic); this file is split out because it also
cross-checks the duplicated FaultState wire codes against the generated enum.

``sysid_lib`` lives in ``tests/hardware/`` (excluded from collection via
``--ignore=tests/hardware``), so it goes on ``sys.path`` explicitly here, mirroring
``test_bench_sysid_logic.py``.
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np
import pytest

_HW_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'hardware')
if _HW_DIR not in sys.path:
    sys.path.insert(0, _HW_DIR)

import sysid_lib as sid  # noqa: E402


# ===========================================================================
# Lead-clamp sizing + knot-ramp stimulus generators
# ===========================================================================

def test_lead_clamp_frame_step_2x_margin():
    # 0.5 × the 0.10 rev clamp = 0.05 rev/frame → 2.0 rev/s at 40 Hz (the operating
    # point). The default margin_frac is 0.5 (the methodology's ≥2× lead margin).
    assert sid.lead_clamp_frame_step(0.10) == pytest.approx(0.05)
    assert sid.lead_clamp_frame_step(0.10, margin_frac=0.5) / 0.025 == pytest.approx(2.0)
    assert sid.lead_clamp_frame_step(0.10, margin_frac=1.0) == pytest.approx(0.10)


def test_lead_clamp_frame_step_rejects_bad_margin():
    for bad in (0.0, -0.1, 1.5):
        with pytest.raises(ValueError):
            sid.lead_clamp_frame_step(0.10, margin_frac=bad)


def test_knot_ramp_frames_ceils_and_zero():
    assert sid.knot_ramp_frames(1.5, 1.5, 0.05) == 0          # already there
    assert sid.knot_ramp_frames(1.5, 1.64, 0.05) == 3         # 0.14/0.05 → ceil 2.8 = 3
    assert sid.knot_ramp_frames(1.5, 1.55, 0.05) == 1         # 0.05/0.05 → 1
    assert sid.knot_ramp_frames(1.5, 1.56, 0.05) == 2         # just over one step
    with pytest.raises(ValueError):
        sid.knot_ramp_frames(0.0, 1.0, 0.0)


def test_knot_step_ramp_endpoints_hold_and_under_clamp():
    b = sid.stroke_bounds(3.0, margin_rev=0.15)
    step = sid.lead_clamp_frame_step(0.10)   # 0.05
    ramp = sid.knot_step_ramp(1.5, 1.78, frame_step_rev=step, hold_frames=10, bounds=b)
    assert ramp[0] == pytest.approx(1.5)                  # first knot = start (no jump at arm)
    assert ramp[-1] == pytest.approx(1.78)                # ends holding the target
    # every knot-to-knot step is under the lead clamp (the whole point).
    assert sid.max_frame_step(ramp) <= step + 1e-12
    # holds the target for hold_frames after the ramp.
    n_ramp = sid.knot_ramp_frames(1.5, 1.78, step)
    assert len(ramp) == n_ramp + 1 + 10
    assert np.allclose(ramp[-10:], 1.78)


def test_knot_step_ramp_retracting_step():
    step = sid.lead_clamp_frame_step(0.10)
    ramp = sid.knot_step_ramp(1.5, 1.36, frame_step_rev=step, hold_frames=0)
    assert ramp[0] == pytest.approx(1.5)
    assert ramp[-1] == pytest.approx(1.36)
    assert sid.max_frame_step(ramp) <= step + 1e-12


def test_knot_step_ramp_clamps_to_bounds():
    b = sid.stroke_bounds(3.0, margin_rev=0.15)   # (0.15, 2.85)
    # A target past the hi bound is clipped defence-in-depth (callers pass in-bounds).
    ramp = sid.knot_step_ramp(2.8, 3.5, frame_step_rev=0.05, hold_frames=5, bounds=b)
    assert ramp.max() <= 2.85 + 1e-12


def test_knot_step_ramp_out_of_bounds_start_no_jump_at_arm():
    # Post --home the leg sits AT the ~0.0 end-stop — BELOW the usable lo=0.15 (the
    # margin backs the window off the end-stop). The approach ramp to centre must
    # (a) start on the true encoder pos so there is NO jump at arm, and (b) keep
    # every knot-to-knot step under the lead-clamp budget while it climbs into the
    # window. The old blanket clip(lo, hi) flattened the sub-lo knots up to 0.15,
    # so the very first streamed knot led the encoder by 0.15 rev — 3x the 0.05 rev
    # lead-clamp step the ramp is sized for (defeating the "index 0 = start" invariant
    # and leaning on the firmware lead clamp / re-enable slew to absorb the jump).
    b = sid.stroke_bounds(3.0, margin_rev=0.15)     # (0.15, 2.85)
    step = sid.lead_clamp_frame_step(0.10)          # 0.05 rev/knot
    ramp = sid.knot_step_ramp(0.0, 1.5, frame_step_rev=step, hold_frames=5, bounds=b)
    assert ramp[0] == pytest.approx(0.0)            # first knot = true start (no jump)
    assert sid.max_frame_step(ramp) <= step + 1e-12  # every knot step under the lead clamp
    assert ramp[-1] == pytest.approx(1.5)           # still ends holding the (in-bounds) target
    # Far (target) side is STILL capped at the stroke bound — near-side relaxation
    # costs no protection (a caller-bug target past hi is still clipped down).
    over = sid.knot_step_ramp(0.0, 3.5, frame_step_rev=step, hold_frames=2, bounds=b)
    assert over.max() <= b.hi_rev + 1e-12


def test_knot_step_ramp_high_end_stop_start_symmetric():
    # Symmetric case: leg parked just above the hi bound (upper margin). The
    # descending approach ramp must likewise start on the true pos (no jump) and
    # never command ABOVE where the leg already sits (which is inside the physical
    # stroke), so it cannot drive into the upper end-stop.
    b = sid.stroke_bounds(3.0, margin_rev=0.15)     # (0.15, 2.85)
    step = sid.lead_clamp_frame_step(0.10)
    ramp = sid.knot_step_ramp(2.95, 1.5, frame_step_rev=step, hold_frames=3, bounds=b)
    assert ramp[0] == pytest.approx(2.95)           # true start, no downward jump
    assert ramp.max() <= 2.95 + 1e-12               # never commands above the start pos
    assert sid.max_frame_step(ramp) <= step + 1e-12
    assert ramp[-1] == pytest.approx(1.5)


def test_knot_step_ramp_rejects_negative_hold():
    with pytest.raises(ValueError):
        sid.knot_step_ramp(1.5, 1.6, frame_step_rev=0.05, hold_frames=-1)


def test_max_frame_step_short_series():
    assert sid.max_frame_step([]) == 0.0
    assert sid.max_frame_step([1.0]) == 0.0
    assert sid.max_frame_step([1.0, 1.03, 1.0]) == pytest.approx(0.03)


def test_bridge_step_plan_reuses_position_step_series_clamp():
    b = sid.stroke_bounds(3.0, margin_rev=0.15)   # (0.15, 2.85)
    step = sid.lead_clamp_frame_step(0.10)        # 0.05
    plan = sid.bridge_step_plan(2.8, [0.05, 0.3, -0.1], b,
                                frame_step_rev=step, seg_t_s=0.025)
    # step +0.05 fits: target 2.85, 1 frame.
    assert plan[0].target_rev == pytest.approx(2.85)
    assert plan[0].clamped is False
    assert plan[0].ramp_frames == 1
    # step +0.3 hits the hi bound → clamped to 2.85, so the *effective* step is 0.05.
    assert plan[1].target_rev == pytest.approx(2.85)
    assert plan[1].clamped is True
    # step -0.1 → target 2.70, 2 frames (0.1/0.05).
    assert plan[2].target_rev == pytest.approx(2.70)
    assert plan[2].ramp_frames == 2


def test_bridge_step_plan_peak_frame_step_under_clamp():
    b = sid.stroke_bounds(3.0, margin_rev=0.15)
    step = sid.lead_clamp_frame_step(0.10)
    plan = sid.bridge_step_plan(1.5, [0.07, 0.14, 0.28, -0.14], b,
                                frame_step_rev=step, seg_t_s=0.025)
    for sp in plan:
        # The even-spaced ramp guarantees each knot step is at or under the lead step.
        assert sp.peak_frame_step_rev <= step + 1e-12
        # And the reported peak matches the actual ramp's max step.
        ramp = sid.knot_step_ramp(1.5, sp.target_rev, frame_step_rev=step, hold_frames=0)
        assert sid.max_frame_step(ramp) == pytest.approx(sp.peak_frame_step_rev, abs=1e-12)
        assert sp.ramp_duration_s == pytest.approx(sp.ramp_frames * 0.025)


# ===========================================================================
# Telemetry-rate estimator + honest chirp-frequency bounds
# ===========================================================================

def test_estimate_telemetry_rate_regular_100hz():
    ts = np.arange(200) / 100.0    # a clean 100 Hz stream
    r = sid.estimate_telemetry_rate(ts)
    assert r.mean_hz == pytest.approx(100.0, rel=1e-6)
    assert r.median_hz == pytest.approx(100.0, rel=1e-6)
    assert r.jitter_rms_s == pytest.approx(0.0, abs=1e-9)
    assert r.effective_hz == pytest.approx(100.0, rel=1e-6)
    assert r.n_intervals == 199


def test_estimate_telemetry_rate_single_blip_is_robust():
    # A lone 50 ms stall in a 100 Hz stream must NOT tank the effective rate — the
    # p95 gap ignores a single 1% blip (one gap in an 8 s chirp is negligible for a
    # lock-in). effective stays ~100 Hz; the blip only shows up in max/jitter.
    dt = [0.01] * 100
    dt[50] = 0.05
    ts = np.concatenate([[0.0], np.cumsum(dt)])
    r = sid.estimate_telemetry_rate(ts)
    assert r.max_interval_s == pytest.approx(0.05)
    assert r.effective_hz == pytest.approx(100.0, rel=1e-6)   # blip ignored by p95
    assert r.jitter_rms_s > 0.0


def test_estimate_telemetry_rate_sustained_irregularity_lowers_effective():
    # SUSTAINED irregularity (10% of gaps are 3× longer) DOES drop the effective
    # rate below the mean — the honest number to bound a chirp against.
    dt = [0.01] * 90 + [0.03] * 10
    ts = np.concatenate([[0.0], np.cumsum(dt)])
    r = sid.estimate_telemetry_rate(ts)
    assert r.effective_hz == pytest.approx(1.0 / 0.03, rel=1e-6)   # p95 gap = 0.03 s
    assert r.effective_hz < r.mean_hz


def test_estimate_telemetry_rate_degenerate():
    assert sid.estimate_telemetry_rate([]).n_intervals == 0
    assert sid.estimate_telemetry_rate([1.0]).mean_hz == 0.0
    # Non-monotone / duplicate stamps (dt <= 0) are dropped, not turned into inf.
    r = sid.estimate_telemetry_rate([0.0, 0.0, 0.0])
    assert r.n_intervals == 0
    assert r.effective_hz == 0.0


def test_honest_chirp_top_freq():
    assert sid.honest_chirp_top_freq(100.0) == pytest.approx(12.5)   # 100/8
    assert sid.honest_chirp_top_freq(100.0, min_samples_per_period=10.0) == pytest.approx(10.0)
    assert sid.honest_chirp_top_freq(0.0) == 0.0


def test_knot_stream_top_freq():
    # 40 Hz knots / 5 samples-per-period = 8 Hz — the bridge honestly reaches the
    # 6 Hz pos-loop question but not 30 Hz.
    assert sid.knot_stream_top_freq(0.025) == pytest.approx(8.0)
    assert sid.knot_stream_top_freq(0.0) == 0.0


def test_bridge_chirp_top_freq_binds_to_the_tightest():
    # Nominal 100 Hz telemetry (12.5) vs 40 Hz knots (8.0): the knot bound wins.
    assert sid.bridge_chirp_top_freq(30.0, 100.0, 0.025) == pytest.approx(8.0)
    # A small operator request binds instead.
    assert sid.bridge_chirp_top_freq(5.0, 100.0, 0.025) == pytest.approx(5.0)
    # A very slow/irregular uplink (effective 20 Hz) makes telemetry (2.5) the binder.
    assert sid.bridge_chirp_top_freq(30.0, 20.0, 0.025) == pytest.approx(2.5)


def test_chirp_amplitude_cap_for_lead_clamp_bounds_per_knot_step():
    # The bridge chirp must ALSO stay under the lead-clamp frame step, which the vel/accel
    # caps miss: the per-knot step of A·sin(2πf·t) sampled at seg_t is ~A·2πf·seg_t, so
    # A ≤ frame_step/(2πf·seg_t) keeps the sweep under the 0.05 rev/knot lead step.
    seg_t, frame_step, f1 = 0.025, 0.05, 8.0
    cap = sid.chirp_amplitude_cap_for_lead_clamp(
        f1, frame_step_rev=frame_step, seg_t_s=seg_t)
    assert cap == pytest.approx(frame_step / (2 * math.pi * f1 * seg_t))
    # At the cap the implied peak per-knot displacement equals the frame step (2× margin).
    peak_vel, _ = sid.chirp_peak_kinematics(cap, f1)
    assert peak_vel * seg_t == pytest.approx(frame_step, rel=1e-9)
    # Higher f1 ⇒ tighter cap (monotone decreasing in frequency).
    assert sid.chirp_amplitude_cap_for_lead_clamp(
        16.0, frame_step_rev=frame_step, seg_t_s=seg_t) < cap


def test_lead_clamp_cap_binds_below_vel_cap_at_knot_nyquist():
    # WHY the lead-clamp bound is needed on top of the kinematic caps: at the default
    # DEFAULT_VEL_CAP 4.0 rev/s and MAX_LEAD/seg_t = 0.10/0.025 = 4.0, the vel-bound
    # amplitude rides the lead clamp exactly (peak per-knot step == MAX_LEAD), so the
    # lead-clamp cap (which targets frame_step = 0.5×MAX_LEAD) is HALF the vel cap — i.e.
    # it binds first and restores the 2× margin the steps have.
    seg_t, frame_step, f1 = 0.025, 0.05, 8.0
    lead = sid.chirp_amplitude_cap_for_lead_clamp(
        f1, frame_step_rev=frame_step, seg_t_s=seg_t)
    kin = sid.chirp_amplitude_cap_for_kinematics(f1, vel_cap_rps=4.0, accel_cap_rps2=250.0)
    assert lead < kin              # lead-clamp bound is the binding one at 4.0 rev/s vel cap
    # And it is exactly half the vel-only bound (frame_step = 0.5 × MAX_LEAD).
    vel_bound = 4.0 / (2 * math.pi * f1)
    assert lead == pytest.approx(0.5 * vel_bound, rel=1e-9)


# ===========================================================================
# Guard-latch backoff state machine + FaultState cross-check
# ===========================================================================

def test_fault_codes_match_generated_enum():
    # The plain-int FaultState codes duplicated in sysid_lib (so it stays
    # dependency-free) MUST equal the generated wire enum — else a firmware fault
    # would be misclassified. This is the guard against silent drift.
    from controller.teensy_link import FaultState
    assert sid.FAULT_NONE == int(FaultState.NONE)
    assert sid.FAULT_MPC_STALE == int(FaultState.MPC_STALE)
    assert sid.FAULT_LINK_LOST == int(FaultState.LINK_LOST)
    assert sid.FAULT_MOTOR_OVERSPEED == int(FaultState.MOTOR_OVERSPEED)
    assert sid.FAULT_MAX_DEVIATION == int(FaultState.MAX_DEVIATION)
    assert sid.FAULT_ODRIVE_FATAL == int(FaultState.ODRIVE_FATAL)
    assert sid.FAULT_CAN_BUS_DOWN == int(FaultState.CAN_BUS_DOWN)
    assert sid.FAULT_MOTOR_FB_STALE == int(FaultState.MOTOR_FB_STALE)


def test_classify_fault_buckets():
    # The recoverable-vs-latching PARTITION must match the firmware latch set, not just
    # the enum values (test_fault_codes_match_generated_enum covers those). fault_machine
    # .cpp:69-80,403-418 makes MOTOR_OVERSPEED / MPC_STALE / MAX_DEVIATION all sticky guard
    # E-STOPs (s_estop_latched, released only by fault_notify_clear_errors), while
    # MOTOR_FB_STALE (fb_stale, :388-419) and LINK_LOST (instantaneous jetson_link_up,
    # :420) self-recover. Pinning the partition here means a future firmware drift that
    # re-buckets one of these breaks this test instead of silently mis-recovering.
    assert sid.classify_fault(sid.FAULT_NONE) == 'none'
    # Only MOTOR_FB_STALE and LINK_LOST self-recover.
    for f in (sid.FAULT_LINK_LOST, sid.FAULT_MOTOR_FB_STALE):
        assert sid.classify_fault(f) == 'recoverable'
    # MPC_STALE LATCHES alongside MAX_DEVIATION / MOTOR_OVERSPEED (it is NOT recoverable) —
    # so a latch runs the revert + CLEAR_ERRORS recovery, not a passive watch.
    for f in (sid.FAULT_MPC_STALE, sid.FAULT_MAX_DEVIATION, sid.FAULT_MOTOR_OVERSPEED):
        assert sid.classify_fault(f) == 'latching'
    for f in (sid.FAULT_ODRIVE_FATAL, sid.FAULT_CAN_BUS_DOWN):
        assert sid.classify_fault(f) == 'fatal'
    assert sid.classify_fault(99) == 'fatal'     # unknown → conservative


def test_guard_none_continues():
    g = sid.GuardLatchBackoff()
    act = g.observe(sid.FAULT_NONE)
    assert act.kind == 'continue'
    assert not act.revert and not act.clear_errors


def test_guard_latch_rising_edge_backoff_recover():
    g = sid.GuardLatchBackoff(max_recoveries=2)
    act = g.observe(sid.FAULT_MAX_DEVIATION)
    assert act.kind == 'backoff_recover'
    assert act.revert is True and act.clear_errors is True
    assert g.recoveries_used == 1


def test_guard_latch_level_held_does_not_double_count():
    g = sid.GuardLatchBackoff(max_recoveries=2)
    g.observe(sid.FAULT_MAX_DEVIATION)            # rising edge → recover (uses 1)
    act = g.observe(sid.FAULT_MAX_DEVIATION)      # still latched, awaiting our clear
    assert act.kind == 'watch'
    assert g.recoveries_used == 1                 # NOT double-counted


def test_guard_latch_budget_exhaustion_aborts():
    g = sid.GuardLatchBackoff(max_recoveries=2)
    # Three distinct latches (each returns to NONE in between): 1, 2 recover; 3 aborts.
    for expect, used in [('backoff_recover', 1), ('backoff_recover', 2)]:
        assert g.observe(sid.FAULT_MAX_DEVIATION).kind == expect
        assert g.recoveries_used == used
        assert g.observe(sid.FAULT_NONE).kind == 'continue'   # cleared → back to nominal
    act = g.observe(sid.FAULT_MAX_DEVIATION)      # the 3rd latch — over budget
    assert act.kind == 'abort'
    assert act.revert is True and act.clear_errors is False


def test_guard_fatal_aborts_and_cedes():
    g = sid.GuardLatchBackoff()
    act = g.observe(sid.FAULT_ODRIVE_FATAL)
    assert act.kind == 'abort'
    assert act.revert is True
    assert act.clear_errors is False             # cede authority — no clear
    assert act.classification == 'fatal'


def test_guard_mpc_stale_latches_and_recovers():
    # MPC_STALE is a LATCHING firmware guard E-STOP (fault_machine.cpp:69-80,403-418),
    # reachable when a blocking RPC gain-apply straddles the 250 ms staleness window while
    # armed. On the rising edge it must drive the revert + CLEAR_ERRORS recovery — NOT the
    # passive 'watch' the pre-fix (mis)classification produced.
    g = sid.GuardLatchBackoff(max_recoveries=2)
    act = g.observe(sid.FAULT_MPC_STALE)
    assert act.kind == 'backoff_recover'
    assert act.revert is True and act.clear_errors is True
    assert act.classification == 'latching'
    assert g.recoveries_used == 1
    # Held latched awaiting our clear → watch, not a second recovery.
    assert g.observe(sid.FAULT_MPC_STALE).kind == 'watch'
    assert g.recoveries_used == 1


def test_guard_recoverable_watches_then_aborts_on_persistence():
    # MOTOR_FB_STALE is a genuinely self-recovering fault (fault_machine.cpp fb_stale
    # re-enables output when feedback returns) — the correct fault to drive the watch-then-
    # abort path. MPC_STALE must NOT be used here: it latches (see the test above).
    g = sid.GuardLatchBackoff(recoverable_grace=5)
    for _ in range(5):
        assert g.observe(sid.FAULT_MOTOR_FB_STALE).kind == 'watch'   # firmware self-recovers
    # The 6th consecutive tick of the same recoverable fault → the link is broken.
    assert g.observe(sid.FAULT_MOTOR_FB_STALE).kind == 'abort'


def test_guard_recoverable_run_resets_on_clear():
    g = sid.GuardLatchBackoff(recoverable_grace=3)
    g.observe(sid.FAULT_MOTOR_FB_STALE)
    g.observe(sid.FAULT_MOTOR_FB_STALE)
    g.observe(sid.FAULT_NONE)                    # cleared → run resets
    for _ in range(3):
        assert g.observe(sid.FAULT_MOTOR_FB_STALE).kind == 'watch'
    assert g.observe(sid.FAULT_MOTOR_FB_STALE).kind == 'abort'


def test_guard_history_records_every_observation():
    g = sid.GuardLatchBackoff()
    g.observe(sid.FAULT_NONE)
    g.observe(sid.FAULT_MAX_DEVIATION)
    assert len(g.history) == 2
    assert g.history[1]['classification'] == 'latching'
    assert g.history[1]['action'] == 'backoff_recover'


# ===========================================================================
# Stream-then-arm warmup + startup guard-latch clearance
# ===========================================================================

def test_warmup_knot_series_is_flat_at_pos_and_right_length():
    # The DISARMED stream-then-arm warmup is a flat hold at the measured position:
    # every knot equals the position (so the interp base re-baselines to the encoder
    # and no knot leads it), for exactly n_frames knots. Streamed with mpc_active=0 it
    # freshens the staleness clock without commanding motion (output is gated).
    s = sid.warmup_knot_series(0.2344, n_frames=16)
    assert len(s) == 16
    assert s[0] == pytest.approx(0.2344)
    assert np.all(s == pytest.approx(0.2344))
    assert sid.max_frame_step(s) == pytest.approx(0.0)   # flat: no knot-to-knot step at all
    # Default frame count is the module constant (0.4 s at 40 Hz).
    assert len(sid.warmup_knot_series(1.5)) == sid.BRIDGE_ARM_WARMUP_FRAMES


def test_warmup_knot_series_rejects_zero_frames():
    for bad in (0, -1):
        with pytest.raises(ValueError):
            sid.warmup_knot_series(1.5, n_frames=bad)


def test_deviation_within_clear_tol_is_sign_agnostic():
    # u0≈enc gate before a startup CLEAR_ERRORS: within tol either sign is safe,
    # beyond tol either sign refuses. Default tol is the 0.10 rev lead clamp.
    assert sid.deviation_within_clear_tol(0.0)
    assert sid.deviation_within_clear_tol(0.09)
    assert sid.deviation_within_clear_tol(-0.09)
    assert not sid.deviation_within_clear_tol(0.5229)     # the measured MAX_DEVIATION latch dev
    assert not sid.deviation_within_clear_tol(-0.5229)
    # Custom tolerance is honoured.
    assert sid.deviation_within_clear_tol(0.2, tol_rev=0.25)
    assert not sid.deviation_within_clear_tol(0.2, tol_rev=0.1)


def test_plan_startup_latch_none_proceeds():
    plan = sid.plan_startup_latch(sid.FAULT_NONE)
    assert plan.action == 'proceed'
    assert plan.classification == 'none'


def test_plan_startup_latch_latching_clears():
    # A sticky guard E-STOP (MAX_DEVIATION / MPC_STALE / MOTOR_OVERSPEED) must drive
    # the verify-u0≈enc + CLEAR_ERRORS path — NOT abort, NOT proceed. This is the
    # operator's case: the persisted MAX_DEVIATION latch that blocked every run.
    for f in (sid.FAULT_MAX_DEVIATION, sid.FAULT_MPC_STALE, sid.FAULT_MOTOR_OVERSPEED):
        plan = sid.plan_startup_latch(f)
        assert plan.action == 'clear', sid.fault_name(f)
        assert plan.classification == 'latching'


def test_plan_startup_latch_recoverable_waits():
    for f in (sid.FAULT_LINK_LOST, sid.FAULT_MOTOR_FB_STALE):
        plan = sid.plan_startup_latch(f)
        assert plan.action == 'wait_recover'
        assert plan.classification == 'recoverable'


def test_plan_startup_latch_fatal_aborts_without_clearing():
    # ODRIVE_FATAL (the measured live state while the bench ODrive is unpowered) and
    # CAN_BUS_DOWN must ABORT with guidance — the harness never auto-clears a fatal.
    # Unknown codes are conservatively fatal too.
    for f in (sid.FAULT_ODRIVE_FATAL, sid.FAULT_CAN_BUS_DOWN, 99):
        plan = sid.plan_startup_latch(f)
        assert plan.action == 'abort'
        assert plan.classification == 'fatal'


def test_plan_startup_latch_action_matches_classification():
    # The action partition must track classify_fault exactly — a future firmware
    # re-bucket that breaks classify_fault must also break this, not silently
    # mis-route a startup clear (e.g. auto-clearing a newly-fatal code).
    expect = {'none': 'proceed', 'latching': 'clear',
              'recoverable': 'wait_recover', 'fatal': 'abort'}
    for f in (sid.FAULT_NONE, sid.FAULT_MPC_STALE, sid.FAULT_LINK_LOST,
              sid.FAULT_MOTOR_OVERSPEED, sid.FAULT_MAX_DEVIATION,
              sid.FAULT_ODRIVE_FATAL, sid.FAULT_CAN_BUS_DOWN, sid.FAULT_MOTOR_FB_STALE):
        plan = sid.plan_startup_latch(f)
        assert plan.action == expect[sid.classify_fault(f)]


# ===========================================================================
# Arm-settle + output-engagement verify (2026-07-12 approach-latch fix)
# ===========================================================================

def test_arm_settle_series_is_flat_and_right_length():
    # The FLAT ARMED settle held after the arm so the firmware re-enable recovery slew
    # converges at the encoder before any climb: every knot equals the arm position (no
    # knot leads the encoder → the 1.0 rev/s slew's target == the encoder → it converges
    # in one tick), for exactly n_frames knots.
    s = sid.arm_settle_series(1.5, n_frames=24)
    assert len(s) == 24
    assert np.all(s == pytest.approx(1.5))
    assert sid.max_frame_step(s) == pytest.approx(0.0)   # flat: no knot-to-knot step
    assert len(sid.arm_settle_series(0.3)) == sid.BRIDGE_ARM_SETTLE_FRAMES


def test_arm_settle_series_rejects_zero_frames():
    for bad in (0, -1):
        with pytest.raises(ValueError):
            sid.arm_settle_series(1.5, n_frames=bad)


def test_arm_settle_covers_the_enable_latency_at_the_ramp_rate():
    # The core guarantee: the flat settle must outlast the firmware output-enable latency
    # (10 Hz J→T heartbeat + 10 Hz fault task ≈ 200 ms) so the slew converges with a
    # STATIONARY command. At 40 Hz that latency is ~8 knots; the settle must exceed it
    # with margin (else the same race that produced the operator's latch survives).
    seg_t_s = 0.025
    enable_latency_s = 0.2   # 100 ms heartbeat + 100 ms fault task (FAULT_TASK_HZ=10)
    latency_frames = enable_latency_s / seg_t_s
    assert sid.BRIDGE_ARM_SETTLE_FRAMES > latency_frames * 1.5


def test_engage_test_increment_is_safe_under_both_clamps():
    # The engagement probe must never itself latch the guard: its increment has to sit
    # well under BOTH the lead clamp (0.10 rev) and MAX_DEVIATION (0.5 rev), and be
    # reachable by the 1.0 rev/s recovery slew within a couple of 40 Hz knots.
    assert sid.BRIDGE_ENGAGE_TEST_REV < 0.10          # under the lead clamp
    assert sid.BRIDGE_ENGAGE_TEST_REV < 0.5           # under MAX_DEVIATION
    reach_per_knot = 1.0 * 0.025                       # slew vel × seg_t
    assert sid.BRIDGE_ENGAGE_TEST_REV <= 2 * reach_per_knot


def test_classify_output_engagement_engaged_when_encoder_follows():
    chk = sid.classify_output_engagement(
        commanded_delta_rev=0.03, encoder_delta_rev=0.028,
        fault_state=sid.FAULT_NONE, fw_mpc_active=True)
    assert chk.engaged
    assert 'engaged' in chk.reason


def test_classify_output_engagement_mpc_active_false_is_most_specific():
    # A competing heartbeat authority (arm never took) is diagnosed FIRST — even if the
    # encoder happened to move and fault is NONE, mpc_active=0 is the load-bearing cause.
    chk = sid.classify_output_engagement(
        commanded_delta_rev=0.03, encoder_delta_rev=0.03,
        fault_state=sid.FAULT_NONE, fw_mpc_active=False)
    assert not chk.engaged
    assert 'mpc_active=0' in chk.reason


def test_classify_output_engagement_fault_gates_output():
    # A non-NONE fault_state (e.g. MOTOR_FB_STALE frozen feedback, or ODRIVE_FATAL) is
    # named as the gating condition even though the encoder did not move.
    for f in (sid.FAULT_MOTOR_FB_STALE, sid.FAULT_ODRIVE_FATAL, sid.FAULT_MAX_DEVIATION):
        chk = sid.classify_output_engagement(
            commanded_delta_rev=0.03, encoder_delta_rev=0.0,
            fault_state=f, fw_mpc_active=True)
        assert not chk.engaged
        assert sid.fault_name(f) in chk.reason


def test_classify_output_engagement_armed_but_not_following():
    # The subtle case the operator's original "never moves" report pointed at: armed,
    # fault NONE, but the encoder does not follow the command → the leg is powered-down /
    # not truly CLOSED_LOOP / mechanically bound. Not a fault code, still not engaged.
    chk = sid.classify_output_engagement(
        commanded_delta_rev=0.03, encoder_delta_rev=0.001,
        fault_state=sid.FAULT_NONE, fw_mpc_active=True)
    assert not chk.engaged
    assert 'not following' in chk.reason


def test_classify_output_engagement_tracks_at_the_frac_boundary():
    # The track fraction is a loose "did it MOVE" gate, not a tracking-accuracy check.
    # Exactly at the boundary counts as engaged; just under does not.
    frac = sid.BRIDGE_ENGAGE_TRACK_FRAC
    at = sid.classify_output_engagement(
        commanded_delta_rev=0.03, encoder_delta_rev=frac * 0.03,
        fault_state=sid.FAULT_NONE, fw_mpc_active=True)
    assert at.engaged
    under = sid.classify_output_engagement(
        commanded_delta_rev=0.03, encoder_delta_rev=frac * 0.03 - 1e-4,
        fault_state=sid.FAULT_NONE, fw_mpc_active=True)
    assert not under.engaged


def test_classify_output_engagement_mpc_active_none_is_not_a_false_negative():
    # An older/short heartbeat with no MPC_ACTIVE readout (None) must NOT be read as
    # "arm did not take" — only an explicit False does. With fault NONE + the encoder
    # following, None still classifies engaged.
    chk = sid.classify_output_engagement(
        commanded_delta_rev=0.03, encoder_delta_rev=0.03,
        fault_state=sid.FAULT_NONE, fw_mpc_active=None)
    assert chk.engaged


def test_approach_abort_diagnostic_names_fault_and_live_quantities():
    # The self-diagnosing abort string must carry the fault name and the live
    # u0/enc/deviation so the operator's next report is self-explanatory (the whole
    # point of replacing the opaque 'guard latch' line).
    s = sid.approach_abort_diagnostic(
        fault_state=sid.FAULT_MAX_DEVIATION, u0_rev=1.20, enc_rev=0.68,
        guard_reason="MAX_DEVIATION latch → back off", lead_clamp=True, mpc_active=True)
    assert 'MAX_DEVIATION' in s
    assert '+1.2000' in s and '+0.6800' in s
    assert '+0.5200' in s                      # dev = u0 - enc = 0.52 (past the 0.5 latch)
    assert 'lead_clamp=engaged' in s
    assert 'fw_mpc_active=True' in s
    assert 'back off' in s


def test_approach_abort_diagnostic_omits_optional_fields_when_absent():
    s = sid.approach_abort_diagnostic(
        fault_state=sid.FAULT_MPC_STALE, u0_rev=0.5, enc_rev=0.5)
    assert 'MPC_STALE' in s
    assert 'dev(u0-enc)=+0.0000' in s
    assert 'lead_clamp' not in s
    assert 'fw_mpc_active' not in s


# ---------------------------------------------------------------------------
# Engagement-probe target: live-encoder baseline + high-side-only clamp
# (2026-07-12 false 'not-following' abort — a leg homed below the firmware
# STROKE_MIN clamp is driven up to STROKE_MIN by the settle; the engagement
# check must reference where the leg ACTUALLY is, not the stale home pos).
# ---------------------------------------------------------------------------

def test_engagement_probe_target_tiny_increment_within_window():
    # Leg comfortably inside the window: the target is just the small UP increment; the
    # high-side clamp is inert. Commanded delta == the design increment.
    tgt = sid.engagement_probe_target(1.50, sid.BRIDGE_ENGAGE_TEST_REV, hi_rev=2.85)
    assert tgt == pytest.approx(1.50 + sid.BRIDGE_ENGAGE_TEST_REV)


def test_engagement_probe_target_below_window_stays_tiny_not_clamped_up():
    # THE REGRESSION: a leg sitting below the harness window (homed to STROKE_MIN ≈ 0.071,
    # window low bound 0.15) must NOT have its nudge inflated up to the low bound. A
    # bounds.clamp() would return 0.15 (a 0.079-rev climb); the high-side-only clamp keeps
    # it a true 0.03-rev nudge from the live encoder.
    enc0 = 0.0709
    tgt = sid.engagement_probe_target(enc0, sid.BRIDGE_ENGAGE_TEST_REV, hi_rev=2.85)
    assert tgt == pytest.approx(enc0 + sid.BRIDGE_ENGAGE_TEST_REV)
    assert (tgt - enc0) == pytest.approx(sid.BRIDGE_ENGAGE_TEST_REV)
    # Contrast with the OLD behaviour that produced the false abort.
    bounds = sid.stroke_bounds(3.0, 0.15)              # window [0.15, 2.85]
    assert bounds.clamp(enc0 + sid.BRIDGE_ENGAGE_TEST_REV) == pytest.approx(0.15)


def test_engagement_probe_target_high_side_clamp_binds_near_ceiling():
    # Near the top of stroke the increment is clamped to hi so it never over-commands.
    tgt = sid.engagement_probe_target(2.84, sid.BRIDGE_ENGAGE_TEST_REV, hi_rev=2.85)
    assert tgt == pytest.approx(2.85)
    assert (tgt - 2.84) <= sid.BRIDGE_ENGAGE_TEST_REV + 1e-12


def test_engagement_shared_baseline_reads_the_operator_run_as_engaged():
    # End-to-end numeric regression on the operator's own 2026-07-12 numbers. The leg
    # homed to -0.069 (below STROKE_MIN 0.0709); the flat settle drove it UP to STROKE_MIN,
    # so at the probe start the encoder is ~0.0727. It then tracked the nudge to ~0.1477
    # (base reached 0.15; live_deviation +0.0023). The engagement verdict MUST be computed
    # on ONE baseline (the live post-settle encoder).
    enc0 = 0.0727                       # post-settle encoder (driven to STROKE_MIN)
    enc1 = 0.1477                       # tracked to ~the target (0.075 rev of motion)
    hi = 2.85
    tgt = sid.engagement_probe_target(enc0, sid.BRIDGE_ENGAGE_TEST_REV, hi_rev=hi)
    # Shared-baseline (fixed) verdict: commanded and encoder deltas both from enc0.
    fixed = sid.classify_output_engagement(
        commanded_delta_rev=tgt - enc0, encoder_delta_rev=enc1 - enc0,
        fault_state=sid.FAULT_NONE, fw_mpc_active=True)
    assert fixed.engaged                                # the leg DID follow → engaged
    # Mixed-baseline (the pre-fix bug): commanded measured from the stale home pos -0.069.
    stale_home = -0.069
    inflated_tgt = sid.stroke_bounds(3.0, 0.15).clamp(stale_home + sid.BRIDGE_ENGAGE_TEST_REV)
    buggy = sid.classify_output_engagement(
        commanded_delta_rev=inflated_tgt - stale_home,  # 0.219 rev (mismatched baseline)
        encoder_delta_rev=enc1 - enc0,                  # 0.075 rev (live baseline)
        fault_state=sid.FAULT_NONE, fw_mpc_active=True)
    assert not buggy.engaged                            # the false 'not following' abort
    assert 'not following' in buggy.reason
