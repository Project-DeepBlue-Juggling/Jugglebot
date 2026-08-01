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
    from teensy_link import FaultState
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


# ===========================================================================
# Run-A upgrades (2026-07-12): estimator hardening, extended ladder, buzz check,
# servo-limited step ramp, sustained-tracking discriminant, 100 Hz logging.
# ===========================================================================

# --- Feature 2: servo-limited measurement step frame-step (0.9 margin) ------

def test_step_frame_step_0p9_margin_sizing():
    # The measurement step ramps at 0.9× the lead clamp = 0.09 rev/frame = 3.6 rev/s
    # (still < MAX_LEAD 0.10, < vel_limit 4.0), so rise time reflects the servo not the
    # gentle 0.5-margin approach ramp. Contrast the approach ramp at 0.05 rev/frame.
    step = sid.lead_clamp_frame_step(0.10, margin_frac=0.9)
    assert step == pytest.approx(0.09)
    assert step / 0.025 == pytest.approx(3.6)                 # rev/s at 40 Hz knots
    approach = sid.lead_clamp_frame_step(0.10, margin_frac=0.5)
    assert step > approach                                    # measurement ramp is faster
    assert step < 0.10                                        # still under the lead clamp


# --- Feature 3: Welch H1 + magnitude-squared coherence ----------------------

def test_welch_h1_recovers_known_gain_high_coherence():
    # A noiseless pure-gain plant y = k·u: H1 must recover the gain with coherence ~1
    # (the output is fully explained by the input at every bin).
    fs, n = 100.0, 4096
    rng = np.random.default_rng(1)
    u = rng.standard_normal(n)
    y = 0.5 * u
    g, ph, coh = sid.welch_h1_coherence(u, y, fs, [5.0, 10.0, 20.0])
    assert np.allclose(g, 0.5, atol=1e-6)
    assert np.allclose(coh, 1.0, atol=1e-6)
    assert np.allclose(ph, 0.0, atol=1e-6)


def test_welch_coherence_low_for_uncorrelated_output():
    # Independent noise output → coherence near 0 (the aliased/low-SNR flag the harness
    # uses to reject a bin's gain/phase).
    fs, n = 100.0, 4096
    rng = np.random.default_rng(2)
    u = rng.standard_normal(n)
    y = rng.standard_normal(n)              # unrelated to u
    _, _, coh = sid.welch_h1_coherence(u, y, fs, [5.0, 10.0, 20.0])
    assert np.all(coh < 0.5)


def test_welch_h1_degenerate_short_input():
    g, ph, coh = sid.welch_h1_coherence([1.0, 2.0], [1.0, 2.0], 100.0, [5.0])
    assert np.isnan(g[0])
    assert coh[0] == 0.0


def test_single_freq_response_mean_removal_is_zero_effect_on_grid():
    # The defensive mean-removal must be bit-identical on an integer-period grid (the
    # verified zero-effect claim), even with a large DC offset added to both signals.
    fs, freq, n = 1000.0, 5.0, 1000
    t = np.arange(n) / fs
    u = np.sin(2 * math.pi * freq * t)
    y = 2.0 * np.sin(2 * math.pi * freq * t + math.radians(30.0))
    g0, p0 = sid.single_freq_response(t, u, y, freq)
    g1, p1 = sid.single_freq_response(t, u + 1.5, y + 1.5, freq)   # DC offset added
    assert g1 == pytest.approx(g0, rel=1e-9)
    assert p1 == pytest.approx(p0, abs=1e-9)
    assert g1 == pytest.approx(2.0, rel=1e-3)


# --- Feature 4: extended ladder (frozen vint + scaled vel_gain candidates) ---

def test_high_rung_vel_gain_candidates_scale_and_shape():
    # lo = round(0.45·√(pg/90)) to 0.05; candidates [lo, 1.25·lo, 1.6·lo]. At pg=90 the
    # base lo (0.45) is recovered; lo climbs with √pg so damping rises with pos_gain.
    c90 = sid.high_rung_vel_gain_candidates(90.0)
    assert c90[0] == pytest.approx(0.45)
    assert c90 == pytest.approx([0.45, 1.25 * 0.45, 1.6 * 0.45])
    c210 = sid.high_rung_vel_gain_candidates(210.0)
    assert c210[0] == pytest.approx(round(0.45 * math.sqrt(210.0 / 90.0) / 0.05) * 0.05)
    assert c210[0] > c90[0]                                   # climbs with pos_gain
    # Exactly three candidates, ascending (first-stable-wins searches low→high).
    assert len(c210) == 3 and c210[0] < c210[1] < c210[2]


def test_high_rung_table_freezes_vint_by_default():
    rungs = sid.high_rung_table()
    assert [r.pos_gain for r in rungs] == list(sid.HIGH_RUNG_POS_GAINS)
    for r in rungs:
        assert r.vel_int_gain == pytest.approx(sid.FROZEN_VEL_INT_GAIN)   # frozen, not ratio
        # vel_gain_lo/hi bracket the explicit candidate set.
        cands = sid.high_rung_vel_gain_candidates(r.pos_gain)
        assert r.vel_gain_lo == pytest.approx(cands[0])
        assert r.vel_gain_hi == pytest.approx(cands[-1])


def test_high_rung_table_vint_ratio_variant_restores_scaling():
    # --vint-ratio restores vel_int = pos_gain / ratio on the high rungs (the operator
    # opt-out of the freeze).
    rungs = sid.high_rung_table(vint_ratio=125.0)
    for r in rungs:
        assert r.vel_int_gain == pytest.approx(r.pos_gain / 125.0)
    assert rungs[-1].vel_int_gain == pytest.approx(210.0 / 125.0)


def test_extended_ladder_appends_high_rungs_and_from_pg_filters():
    full = sid.extended_ladder()
    pgs = [r.pos_gain for r in full]
    assert pgs == [25.0, 40.0, 55.0, 70.0, 90.0, 110.0, 130.0, 155.0, 180.0, 210.0]
    # --from-pg drops the low rungs (skip re-proving them).
    hi = sid.extended_ladder(from_pos_gain=110.0)
    assert [r.pos_gain for r in hi] == [110.0, 130.0, 155.0, 180.0, 210.0]
    assert sid.is_high_rung(110.0) and not sid.is_high_rung(90.0)


def test_ladder_revert_target_baseline_above_130_last_good_below():
    baseline = sid.GainTriple(40.0, 0.20, 0.32)
    last_good = sid.GainTriple(90.0, 0.45, 0.72)
    # At/above pos_gain 130 the recovery slew must run against SOFT baseline gains.
    assert sid.ladder_revert_target(130.0, last_good, baseline) is baseline
    assert sid.ladder_revert_target(210.0, last_good, baseline) is baseline
    # Below 130 the last-good triple is the fallback.
    assert sid.ladder_revert_target(110.0, last_good, baseline) is last_good
    # No last-good yet → baseline even below 130.
    assert sid.ladder_revert_target(55.0, None, baseline) is baseline


def test_extended_ladder_high_rungs_gracefully_stop_via_zeta_or_climb():
    # Sanity that the extended table drops into GainLadder and escalates: a stable rung
    # with zeta ≥ target escalates to the next high rung rather than stopping early.
    lad = sid.GainLadder(rungs=sid.extended_ladder(from_pos_gain=110.0))
    dec = lad.record(sid.GainTriple(110.0, 0.5, 0.72),
                     sid.OnsetResult(False, 0.0, 0.0, []), zeta=0.8)
    assert dec.action == 'escalate'
    assert lad.current_rung().pos_gain == 130.0


# --- Feature 4: quiescent-hold buzz classifier ------------------------------

def test_classify_quiescent_buzz_vel_gate():
    # vel-ripple above the onset is a buzz; below is clean. iq is reported but NOT gated
    # by default (stock iq is on-change-aliased → an iq gate would false-safe).
    assert sid.classify_quiescent_buzz(0.05).buzz            # 0.05 > 0.03 onset
    assert not sid.classify_quiescent_buzz(0.01).buzz
    # A big iq ripple does NOT trip the buzz unless fast-iq firmware is present.
    q = sid.classify_quiescent_buzz(0.01, iq_rms=5.0, iq_onset=0.3)
    assert not q.buzz
    assert q.iq_rms == pytest.approx(5.0)                    # still reported


def test_classify_quiescent_buzz_iq_gate_only_with_fast_iq():
    q = sid.classify_quiescent_buzz(0.01, iq_rms=5.0, iq_onset=0.3,
                                    fast_iq_available=True)
    assert q.buzz                                            # now the iq gate applies
    assert any('iq ripple' in r for r in q.reasons)
    # Custom vel onset is honoured.
    assert sid.classify_quiescent_buzz(0.02, vel_onset=0.015).buzz


# --- Feature 5: sustained-tracking waveform generator -----------------------

def test_track_sine_is_c1_at_ends_and_within_bounds():
    b = sid.stroke_bounds(3.0, 0.15)                         # (0.15, 2.85)
    seg_t, center, amp, pv, dur = 0.025, 1.5, 0.4, 2.0, 6.0
    s = sid.track_reference_series(center_rev=center, amplitude_rev=amp,
                                   peak_vel_rps=pv, duration_s=dur, seg_t_s=seg_t,
                                   wave='sine', bounds=b)
    assert len(s) == int(round(dur / seg_t))
    assert s[0] == pytest.approx(center)                     # starts at centre
    # C1: the raised-cosine amplitude ramp makes the sine start (and end) at rest — the
    # first/last knot-to-knot step is far below the mid-sweep peak step.
    peak_step = pv * seg_t
    assert abs(s[1] - s[0]) < 0.1 * peak_step                # near-zero velocity at start
    assert abs(s[-1] - center) < 0.05 * amp                  # ends back near centre
    # Bounds respected and per-knot step ≤ peak_vel·seg_t (lead-clamp sizing).
    assert s.min() >= b.lo_rev - 1e-9 and s.max() <= b.hi_rev + 1e-9
    assert sid.max_frame_step(s) <= peak_step + 1e-9


def test_track_sine_clips_to_bounds_when_amplitude_would_exceed():
    b = sid.stroke_bounds(3.0, 0.15)
    # An oversized amplitude near a bound is clipped defence-in-depth by the generator.
    s = sid.track_reference_series(center_rev=0.30, amplitude_rev=0.5,
                                   peak_vel_rps=2.0, duration_s=2.0, seg_t_s=0.025,
                                   wave='sine', bounds=b)
    assert s.min() >= b.lo_rev - 1e-9


def test_track_triangle_reversal_count_and_step():
    # Triangle: slope = peak_vel (sharp reversals), period T = 4A/peak_vel; reversals
    # every T/2 → ~ duration·peak_vel/(2A) of them. Per-knot step == peak_vel·seg_t.
    seg_t, center, amp, pv, dur = 0.025, 1.5, 0.3, 2.0, 6.0
    s = sid.track_reference_series(center_rev=center, amplitude_rev=amp,
                                   peak_vel_rps=pv, duration_s=dur, seg_t_s=seg_t,
                                   wave='triangle')
    assert sid.max_frame_step(s) <= pv * seg_t + 1e-9
    expected = int(round(dur * pv / (2.0 * amp)))            # 6·2/(0.6) = 20
    rev = sid.count_velocity_reversals(np.diff(s))
    assert abs(rev - expected) <= 1


def test_track_sine_frequency_matches_peak_velocity():
    # f chosen so A·2πf == peak_vel (per-knot step at the peak hits the lead-clamp size).
    f = sid.track_sine_frequency(2.0, 0.4)
    assert 2.0 == pytest.approx(0.4 * 2 * math.pi * f)


# --- Feature 5: residual periodogram peak extraction ------------------------

def test_periodogram_peaks_recover_6hz_limit_cycle_and_harmonic():
    # THE discriminant regression: a synthetic 6 Hz limit cycle + 12 Hz 2nd harmonic at
    # the true 100 Hz telemetry rate must come out at 6.0 ± 0.2 Hz (dominant) with the
    # 12 Hz harmonic present — directly comparable to the S4 forensics (5.9-6.1 Hz + 12.3).
    fs = 100.0
    t = np.arange(int(6.0 * fs)) / fs                        # 6 s window → df ≈ 0.17 Hz
    x = 1.0 * np.sin(2 * math.pi * 6.0 * t) + 0.3 * np.sin(2 * math.pi * 12.0 * t)
    peaks = sid.periodogram_peaks(x, fs, top_n=3)
    assert peaks[0][0] == pytest.approx(6.0, abs=0.2)        # dominant fundamental
    assert peaks[0][1] == pytest.approx(1.0, rel=0.1)        # amplitude ~1.0
    harmonic = [f for f, _ in peaks if abs(f - 12.0) <= 0.2]
    assert harmonic, f"12 Hz harmonic missing from {peaks}"


def test_periodogram_peaks_degenerate():
    assert sid.periodogram_peaks([], 100.0) == []
    assert sid.periodogram_peaks([1.0, 2.0, 3.0], 0.0) == []


def test_count_velocity_reversals_deadband():
    # Clean alternating velocity → a reversal every sample. A deadband drops dither.
    v = np.array([1.0, -1.0, 1.0, -1.0])
    assert sid.count_velocity_reversals(v) == 3
    dither = np.array([0.001, -0.001, 0.001, -0.001])
    assert sid.count_velocity_reversals(dither, deadband_rps=0.01) == 0


def test_analyze_tracking_residual_end_to_end():
    # The pure discriminant assembler on a synthetic run: residual = cmd - pos carries a
    # 6 Hz cycle; dominant freq recovered, iq stats + clamp fraction populated.
    fs = 100.0
    t = np.arange(int(6.0 * fs)) / fs
    cmd = 1.5 + 0.4 * np.sin(2 * math.pi * 0.8 * t)          # the tracking reference
    resid = 0.02 * np.sin(2 * math.pi * 6.0 * t)             # a 6 Hz limit cycle on top
    pos = cmd - resid
    vel = np.gradient(pos, t)
    iq = 2.0 + 0.5 * np.sin(2 * math.pi * 6.0 * t)
    lead_clamp = np.where(np.abs(vel) > np.max(np.abs(vel)) * 0.8, 1.0, 0.0)
    r = sid.analyze_tracking_residual(t, cmd, pos, vel=vel, iq=iq, lead_clamp=lead_clamp)
    assert r.fs_hz == pytest.approx(100.0, rel=1e-3)
    assert r.dominant_freq_hz == pytest.approx(6.0, abs=0.2)
    assert r.iq_max == pytest.approx(2.5, rel=0.05)
    assert 0.0 <= r.clamp_engaged_frac <= 1.0
    assert r.reversals > 0
    # No lead_clamp column → clamp fraction is None (the "if the column exists" contract).
    r2 = sid.analyze_tracking_residual(t, cmd, pos, vel=vel, iq=iq)
    assert r2.clamp_engaged_frac is None


def test_median_sample_rate_reads_100hz_not_knot_rate():
    # Feature 1 relies on this: a 100 Hz telemetry log must read ~100 Hz, not the 40 Hz
    # knot rate the old per-knot CSV decimated to.
    t = np.arange(300) / 100.0
    assert sid.median_sample_rate(t) == pytest.approx(100.0, rel=1e-6)
    assert sid.median_sample_rate([0.0]) == 0.0
    assert sid.median_sample_rate([1.0, 1.0, 1.0]) == 0.0    # no positive gaps


# --- Feature 1: fit_step_response on 10 ms-spaced (100 Hz) data --------------

def test_fit_step_response_on_10ms_spaced_data_uses_t_not_fixed_dt():
    # REGRESSION for the 100 Hz CSV change: the step metrics must key off t_s, not a
    # hardcoded 25 ms knot spacing. A monotone first-order step sampled at 100 Hz (dt=10
    # ms) must give the analytic 10-90% rise (ln 9 · tau) and zero overshoot.
    tau = 0.05
    t = np.arange(0.0, 1.0, 0.01)                            # 100 Hz, dt = 10 ms
    y = 1.0 - np.exp(-t / tau)
    m = sid.fit_step_response(t, y, y0=0.0, y_final=1.0)
    assert m.rise_time_s == pytest.approx(math.log(9.0) * tau, rel=0.05)
    assert m.overshoot == pytest.approx(0.0, abs=1e-6)
    assert m.zeta == pytest.approx(1.0)                      # no overshoot → critically damped
    # The SAME curve at 25 ms spacing yields the same rise time (rate-invariant) — the
    # guarantee the decoupled 100 Hz logging depends on.
    t25 = np.arange(0.0, 1.0, 0.025)
    m25 = sid.fit_step_response(t25, 1.0 - np.exp(-t25 / tau), 0.0, 1.0)
    assert m25.rise_time_s == pytest.approx(m.rise_time_s, rel=0.05)


# --- review fixes: velocity-anchored frame steps + chirp amplitude dedupe ----

def test_velocity_anchored_frame_step_identity_at_40hz():
    # At the 40 Hz design point the velocity cap equals the margin sizing exactly
    # (2.0·0.025 = 0.05, 3.6·0.025 = 0.09) — behaviour unchanged vs lead_clamp_frame_step.
    assert sid.velocity_anchored_frame_step(0.10, 0.5, 2.0, 0.025) == pytest.approx(0.05)
    assert sid.velocity_anchored_frame_step(0.10, 0.9, 3.6, 0.025) == pytest.approx(0.09)


def test_velocity_anchored_frame_step_velocity_binds_at_100hz():
    # REGRESSION for the --knot-hz HIGH finding: a fixed 0.09 rev/frame at 100 Hz knots
    # would command 9.0 rev/s (past vel_limit 4.0); the anchor must hold the design
    # velocity instead (0.020 / 0.036 rev/frame ⇒ still 2.0 / 3.6 rev/s).
    seg_100 = 0.010
    gentle = sid.velocity_anchored_frame_step(0.10, 0.5, 2.0, seg_100)
    step = sid.velocity_anchored_frame_step(0.10, 0.9, 3.6, seg_100)
    assert gentle == pytest.approx(0.020)
    assert step == pytest.approx(0.036)
    assert gentle / seg_100 == pytest.approx(2.0)
    assert step / seg_100 == pytest.approx(3.6)


def test_velocity_anchored_frame_step_lead_clamp_binds_for_fast_targets():
    # A hypothetical high v_target must still respect the lead-clamp margin.
    assert sid.velocity_anchored_frame_step(0.10, 0.5, 100.0, 0.025) == pytest.approx(0.05)


def test_velocity_anchored_frame_step_rejects_bad_inputs():
    for v, seg in ((0.0, 0.025), (-1.0, 0.025), (2.0, 0.0), (2.0, -0.01)):
        with pytest.raises(ValueError):
            sid.velocity_anchored_frame_step(0.10, 0.5, v, seg)


def test_dedupe_amplitudes_collapses_lead_clamp_collisions():
    # The default sweep 0.02/0.06/0.12 at the knot-bounded f1≈8 Hz reduces 0.06 AND 0.12
    # to the same ~0.0398 rev cap — the duplicate must be skipped (one CSV, one armed run).
    cap = 0.05 / (2.0 * math.pi * 8.0 * 0.025)               # ≈ 0.03979
    kept, skipped = sid.dedupe_amplitudes([0.02, 0.06, 0.12], lambda a: min(a, cap))
    assert [round(r, 3) for r, _ in kept] == [0.02, 0.06]
    assert kept[1][1] == pytest.approx(round(cap, 4))
    assert len(skipped) == 1
    assert skipped[0][0] == pytest.approx(0.12)
    assert skipped[0][1] == pytest.approx(round(cap, 4))


def test_dedupe_amplitudes_all_distinct_passthrough():
    kept, skipped = sid.dedupe_amplitudes([0.01, 0.02, 0.03], lambda a: a)
    assert [r for r, _ in kept] == [0.01, 0.02, 0.03]
    assert skipped == []


# --- gap-fill knobs: custom rungs + explicit candidate selection -------------

def test_custom_rung_table_builds_explicit_single_candidate_rungs():
    pts = [sid.GainTriple(130.0, 0.50, 0.72), sid.GainTriple(110.0, 0.55, 0.72)]
    rungs = sid.custom_rung_table(pts)
    assert len(rungs) == 2
    for rung, pt in zip(rungs, pts):
        assert rung.explicit is True
        assert rung.pos_gain == pt.pos_gain
        assert rung.vel_int_gain == pt.vel_int_gain
        assert rung.vel_gain_lo == rung.vel_gain_hi == pt.vel_gain


def test_custom_rung_table_rejects_empty():
    with pytest.raises(ValueError):
        sid.custom_rung_table([])


def test_rung_vel_gain_candidates_explicit_overrides_high_rung_scaling():
    # THE point of --rungs: an explicit (130, 0.50) must test exactly 0.50 — the
    # high-rung sqrt scaling would force lo=0.55 there and the off-diagonal point
    # would be untestable.
    explicit = sid.custom_rung_table([sid.GainTriple(130.0, 0.50, 0.72)])[0]
    assert sid.rung_vel_gain_candidates(explicit) == [0.50]
    # Table rungs keep their existing behaviour.
    table_high = sid.LadderRung(130.0, 0.72, 0.55, 0.88)
    assert sid.rung_vel_gain_candidates(table_high) == \
        sid.high_rung_vel_gain_candidates(130.0)
    table_base = sid.LadderRung(40.0, 0.32, 0.20, 0.50)
    assert sid.rung_vel_gain_candidates(table_base, 4) == \
        sid.vel_gain_candidates(table_base, 4)


def test_ladder_rung_default_not_explicit():
    # Backward compatibility: all existing 4-arg constructions are table rungs.
    assert sid.LadderRung(90.0, 0.72, 0.45, 0.90).explicit is False


# --- survey-mode GainLadder: unstable points never abort the gap-fill --------

def _survey_ladder(points):
    return sid.GainLadder(rungs=sid.custom_rung_table(points), survey=True)


def test_survey_unstable_point_advances_not_backoff():
    # THE survey guarantee: (130, 0.50) buzzing must not abort (110, 0.55) —
    # comparing outcomes ACROSS points is the whole attribution experiment.
    lad = _survey_ladder([sid.GainTriple(130.0, 0.50, 0.72),
                          sid.GainTriple(110.0, 0.55, 0.72)])
    dec = lad.record(sid.GainTriple(130.0, 0.50, 0.72),
                     sid.OnsetResult(True, 0.2, 0.9, ['buzz']), None)
    assert dec.action == 'escalate'
    assert not lad.stopped
    assert lad.current_rung().pos_gain == 110.0


def test_survey_completes_all_points_and_has_no_winner():
    pts = [sid.GainTriple(130.0, 0.50, 0.72), sid.GainTriple(110.0, 0.55, 0.72),
           sid.GainTriple(120.0, 0.50, 0.72)]
    lad = _survey_ladder(pts)
    ok = sid.OnsetResult(False, 0.0, 0.0, [])
    bad = sid.OnsetResult(True, 0.2, 0.9, ['buzz'])
    lad.record(pts[0], bad, None)                       # unstable → continue
    lad.record(pts[1], ok, 0.80)                        # stable → continue
    dec = lad.record(pts[2], ok, 0.75)                  # last point
    assert dec.action == 'stop_ok'
    assert dec.gains is None                            # survey has no winner
    assert lad.stopped and lad.stop_reason == 'survey_complete'
    assert lad.last_good == pts[2]                      # stable points still banked


def test_survey_zeta_below_target_recorded_not_backoff():
    lad = _survey_ladder([sid.GainTriple(130.0, 0.45, 0.72),
                          sid.GainTriple(120.0, 0.50, 0.72)])
    dec = lad.record(sid.GainTriple(130.0, 0.45, 0.72),
                     sid.OnsetResult(False, 0.0, 0.0, []), 0.55)   # ζ < 0.7
    assert dec.action == 'escalate'
    assert 'zeta_below_target' in dec.reason


def test_non_survey_ladder_still_backs_off():
    # Regression: the escalation ladder's stop-on-onset semantics are unchanged.
    lad = sid.GainLadder(rungs=sid.custom_rung_table(
        [sid.GainTriple(130.0, 0.50, 0.72)]), survey=False)
    dec = lad.record(sid.GainTriple(130.0, 0.50, 0.72),
                     sid.OnsetResult(True, 0.2, 0.9, ['buzz']), None)
    assert dec.action == 'backoff'
    assert lad.stopped and lad.stop_reason == 'instability_onset'


# ===========================================================================
# Feature 1 — motion-excited HF onset detector
# ===========================================================================

def test_hf_band_knot40_notch_clips_top_of_band():
    # At 40 Hz knots the staircase [34, 46] sits INSIDE the 20-45 band → the effective
    # band shrinks to ~20-34 Hz and the notch is absorbed by the clip.
    lo, hi, notch = sid.hf_band(250.0, 40.0)
    assert lo == pytest.approx(20.0)
    assert hi == pytest.approx(34.0)
    assert notch is None


def test_hf_band_knot100_notch_outside_full_band():
    # At 100 Hz knots the notch [94, 106] is above the band → full 20-45 Hz.
    lo, hi, notch = sid.hf_band(250.0, 100.0)
    assert (lo, hi) == pytest.approx((20.0, 45.0))
    assert notch is None


def test_hf_band_caps_at_0p45_fs():
    # A low telemetry rate caps the band below Nyquist: 0.45·60 = 27 Hz.
    lo, hi, notch = sid.hf_band(60.0, 100.0)
    assert lo == pytest.approx(20.0)
    assert hi == pytest.approx(27.0)
    assert notch is None


def test_hf_band_interior_notch_kept_explicit():
    # A knot rate whose notch sits fully inside the band is returned as an explicit
    # exclusion (not a clip), so the band-RMS masks it out.
    lo, hi, notch = sid.hf_band(250.0, 30.0)          # notch [24, 36] interior to [20, 45]
    assert (lo, hi) == pytest.approx((20.0, 45.0))
    assert notch == pytest.approx((24.0, 36.0))


def test_motion_hf_metrics_30hz_tone_trips_onset():
    # A 30 Hz tone (inside the 100-knot 20-45 band) with RMS above the 0.12 rps onset must
    # trip. Amplitude 0.3 → RMS 0.212 rps.
    fs = 250.0
    t = np.arange(int(2.0 * fs)) / fs
    vel = 0.3 * np.sin(2 * math.pi * 30.0 * t)
    m = sid.motion_hf_metrics(t, vel, iq=None, knot_hz=100.0)
    assert m.fs_hz == pytest.approx(250.0, rel=1e-3)
    assert m.hf_vel_rms == pytest.approx(0.3 / math.sqrt(2.0), rel=0.05)
    assert m.onset


def test_motion_hf_metrics_staircase_at_knot_rate_does_not_trip():
    # A big tone AT the knot rate (40 Hz) must NOT trip at 40-knot: the notch clips the
    # band to 20-34 so the 40 Hz staircase energy is excluded.
    fs = 250.0
    t = np.arange(int(2.0 * fs)) / fs
    vel = 0.5 * np.sin(2 * math.pi * 40.0 * t)        # pure staircase-rate tone
    m = sid.motion_hf_metrics(t, vel, iq=None, knot_hz=40.0)
    assert m.band[1] == pytest.approx(34.0)
    assert m.hf_vel_rms < 0.02                        # 40 Hz is outside the clipped band
    assert not m.onset


def test_motion_hf_metrics_rail_duty_and_iq_onset():
    # iq rail duties count |iq| over 9.0 / 9.5 A; a 30 Hz iq tone above the 0.8 A onset
    # trips via the iq branch.
    fs = 250.0
    t = np.arange(int(2.0 * fs)) / fs
    vel = np.zeros_like(t)
    iq = 2.0 * np.sin(2 * math.pi * 30.0 * t)         # HF iq RMS ≈ 1.414 A > 0.8
    iq[:20] = 9.7                                       # 20 samples railed past 9.5
    m = sid.motion_hf_metrics(t, vel, iq, knot_hz=100.0)
    assert m.hf_iq_rms > 0.8
    assert m.onset
    assert m.rail_duty_9 == pytest.approx(20 / t.size, rel=0.05)
    assert m.rail_duty_9_5 == pytest.approx(20 / t.size, rel=0.05)


def test_motion_hf_metrics_short_window_is_safe():
    m = sid.motion_hf_metrics([0.0, 0.01], [0.1, 0.2], iq=[1.0, 1.0], knot_hz=100.0)
    assert m.hf_vel_rms == 0.0 and m.hf_iq_rms == 0.0
    assert not m.onset


# ===========================================================================
# Feature 2 — min-jerk strokes + battery builder
# ===========================================================================

def test_minjerk_peak_velocity_and_accel_match_formulas():
    # Peak velocity = 1.875·A/T, peak accel = 5.77·A/T², measured off the series.
    A, T, seg_t = 0.5, 0.4, 0.001
    s = sid.minjerk_stroke_series(1.5, 1.5 + A, T, seg_t)
    v = np.diff(s) / seg_t
    a = np.diff(v) / seg_t
    assert np.max(np.abs(v)) == pytest.approx(sid.MINJERK_VEL_COEF * A / T, rel=0.02)
    assert np.max(np.abs(a)) == pytest.approx(sid.MINJERK_ACCEL_COEF * A / (T * T), rel=0.03)
    assert sid.MINJERK_VEL_COEF == pytest.approx(1.875)
    assert sid.MINJERK_ACCEL_COEF == pytest.approx(5.7735, abs=1e-3)


def test_minjerk_boundary_conditions_zero_end_vel_and_accel():
    # Zero velocity AND acceleration at both ends (the defining min-jerk property).
    A, T, seg_t = 0.4, 0.5, 0.001
    s = sid.minjerk_stroke_series(0.0, A, T, seg_t)
    v = np.diff(s) / seg_t
    a = np.diff(v) / seg_t
    assert s[0] == pytest.approx(0.0) and s[-1] == pytest.approx(A)
    assert abs(v[0]) < 1e-3 * sid.MINJERK_VEL_COEF * A / T      # ~0 start velocity
    assert abs(v[-1]) < 1e-3 * sid.MINJERK_VEL_COEF * A / T     # ~0 end velocity
    # accel is analytically 0 at both ends; the first/last discrete second-difference
    # sits a couple % of peak off zero (it samples ~1.5·seg_t in), far below peak.
    assert abs(a[0]) < 0.05 * sid.MINJERK_ACCEL_COEF * A / (T * T)   # ~0 start accel
    assert abs(a[-1]) < 0.05 * sid.MINJERK_ACCEL_COEF * A / (T * T)  # ~0 end accel


def test_minjerk_rejects_bad_inputs():
    for T, seg in ((0.0, 0.01), (-1.0, 0.01), (0.5, 0.0), (0.5, -0.01)):
        with pytest.raises(ValueError):
            sid.minjerk_stroke_series(0.0, 1.0, T, seg)


def test_stroke_battery_default_grid_shape_and_kinds():
    b = sid.stroke_bounds(3.0, 0.15)                            # (0.15, 2.85), sym limit 1.35
    specs = sid.stroke_battery(sid.STROKE_AMPS_DEFAULT, sid.STROKE_VELS_DEFAULT,
                               vel_cap=4.0, accel_cap=250.0, bounds=b, center=1.5)
    kinds = [s.kind for s in specs]
    # 4 amps × 4 vels symmetric + 4 asymmetric + 1 sustained = 21.
    assert kinds.count('symmetric') == 16
    assert kinds.count('asymmetric') == 4
    assert kinds.count('sustained') == 1
    sustained = [s for s in specs if s.kind == 'sustained'][0]
    assert sustained.n_strokes == sid.STROKE_SUSTAINED_N
    assert sustained.amplitude_rev == pytest.approx(1.0)       # largest feasible amp


def test_stroke_battery_accel_cap_stretches_duration():
    # A fast large stroke is accel-limited: T stretched, realized peak vel drops, peak
    # accel pinned at the cap.
    b = sid.stroke_bounds(3.0, 0.15)
    specs = sid.stroke_battery([1.0], [3.8], vel_cap=4.0, accel_cap=10.0,
                               bounds=b, center=1.5)
    sym = [s for s in specs if s.kind == 'symmetric'][0]
    assert sym.accel_limited
    assert sym.peak_accel_out_rps2 == pytest.approx(10.0, rel=1e-6)
    assert sym.out_peak_vel_rps < 3.8                          # stretched below the request
    # Duration = √(5.77·A/cap).
    assert sym.T_out_s == pytest.approx(math.sqrt(sid.MINJERK_ACCEL_COEF * 1.0 / 10.0), rel=1e-6)


def test_stroke_battery_drops_amplitudes_that_dont_fit_bounds():
    # Centre near a bound: only small amplitudes fit symmetric.
    b = sid.stroke_bounds(3.0, 0.15)                            # (0.15, 2.85)
    specs = sid.stroke_battery([0.2, 0.4, 0.7, 1.0], [2.0], vel_cap=4.0, accel_cap=250.0,
                               bounds=b, center=0.5)            # sym limit = 0.35
    amps = sorted({round(s.amplitude_rev, 3) for s in specs})
    assert amps == [0.2]                                       # 0.4/0.7/1.0 dropped
    assert all(s.amplitude_rev <= 0.35 + 1e-9 for s in specs)


def test_stroke_battery_asymmetric_is_fast_out_slow_return():
    b = sid.stroke_bounds(3.0, 0.15)
    specs = sid.stroke_battery([0.4], [1.0, 3.8], vel_cap=4.0, accel_cap=250.0,
                               bounds=b, center=1.5)
    asym = [s for s in specs if s.kind == 'asymmetric'][0]
    # Fast out (max grid vel 3.8) → short T_out; slow return (1.0) → long T_return.
    assert asym.out_peak_vel_rps > asym.return_peak_vel_rps
    assert asym.T_out_s < asym.T_return_s


def test_stroke_series_symmetric_shape_and_bounds():
    b = sid.stroke_bounds(3.0, 0.15)
    specs = sid.stroke_battery([0.4], [2.0], vel_cap=4.0, accel_cap=250.0,
                               bounds=b, center=1.5)
    sym = [s for s in specs if s.kind == 'symmetric'][0]
    s = sid.stroke_series(sym, center=1.5, seg_t=0.01, bounds=b)
    assert s[0] == pytest.approx(1.5)                          # starts at centre (no jump)
    assert s[-1] == pytest.approx(1.5)                         # ends back at centre
    assert s.max() == pytest.approx(1.9, abs=1e-6)             # reaches center+A
    assert s.min() >= b.lo_rev - 1e-9 and s.max() <= b.hi_rev + 1e-9


def test_stroke_series_asymmetric_out_steeper_than_return():
    b = sid.stroke_bounds(3.0, 0.15)
    specs = sid.stroke_battery([0.4], [1.0, 3.8], vel_cap=4.0, accel_cap=250.0,
                               bounds=b, center=1.5)
    asym = [s for s in specs if s.kind == 'asymmetric'][0]
    s = sid.stroke_series(asym, center=1.5, seg_t=0.005, bounds=b)
    d = np.diff(s)
    # Fastest up-step (out throw) far exceeds the fastest down-step magnitude (slow catch).
    assert d.max() > abs(d.min())


def test_stroke_series_sustained_alternates_about_centre():
    b = sid.stroke_bounds(3.0, 0.15)
    specs = sid.stroke_battery([0.4], [2.0], vel_cap=4.0, accel_cap=250.0,
                               bounds=b, center=1.5)
    sus = sid.stroke_battery([0.4], [2.0], vel_cap=4.0, accel_cap=250.0,
                             bounds=b, center=1.5)
    sustained = [s for s in sus if s.kind == 'sustained'][0]
    s = sid.stroke_series(sustained, center=1.5, seg_t=0.01, bounds=b)
    assert s[0] == pytest.approx(1.5) and s[-1] == pytest.approx(1.5)
    assert s.max() == pytest.approx(1.9, abs=1e-6)             # +A extreme
    assert s.min() == pytest.approx(1.1, abs=1e-6)             # −A extreme (alternates)


# ===========================================================================
# Feature 3 — teleop mapping, slew planner, command grammar, gate
# ===========================================================================

def test_teleop_target_deadband_holds_centre():
    b = sid.stroke_bounds(3.0, 0.15)
    assert sid.teleop_target(0.0, 1.5, 0.9, b) == pytest.approx(1.5)
    assert sid.teleop_target(0.03, 1.5, 0.9, b) == pytest.approx(1.5)      # inside deadband
    assert sid.teleop_target(-0.04, 1.5, 0.9, b) == pytest.approx(1.5)


def test_teleop_target_maps_and_clamps():
    b = sid.stroke_bounds(3.0, 0.15)                            # (0.15, 2.85)
    assert sid.teleop_target(1.0, 1.5, 0.9, b) == pytest.approx(2.4)       # center + range
    assert sid.teleop_target(-1.0, 1.5, 0.9, b) == pytest.approx(0.6)
    # A huge range is clamped to bounds minus margin.
    hi = b.hi_rev - sid.TELEOP_CLAMP_MARGIN_REV
    assert sid.teleop_target(1.0, 1.5, 5.0, b) == pytest.approx(hi)
    lo = b.lo_rev + sid.TELEOP_CLAMP_MARGIN_REV
    assert sid.teleop_target(-1.0, 1.5, 5.0, b) == pytest.approx(lo)


def test_slew_toward_rate_limited_convergence_no_overshoot():
    seg_t, vel_cap, accel_cap = 0.025, 2.0, 250.0
    cmd, v = 1.5, 0.0
    target = 2.3
    prev = cmd
    max_v = 0.0
    max_a = 0.0
    for _ in range(2000):
        nxt, v2 = sid.slew_toward(cmd, v, target, vel_cap, accel_cap, seg_t)
        step_v = abs(nxt - cmd) / seg_t
        step_a = abs(v2 - v) / seg_t
        max_v = max(max_v, step_v)
        max_a = max(max_a, step_a)
        assert nxt <= target + 1e-9                            # never overshoots the target
        cmd, v = nxt, v2
        if abs(cmd - target) < 1e-6 and abs(v) < 1e-6:
            break
    assert cmd == pytest.approx(target, abs=1e-4)              # converged
    assert max_v <= vel_cap + 1e-6                             # velocity cap respected
    assert max_a <= accel_cap + 1e-6                           # accel cap respected


def test_slew_toward_stays_within_bounds_for_in_bounds_target():
    b = sid.stroke_bounds(3.0, 0.15)
    seg_t = 0.025
    cmd, v = 1.5, 0.0
    target = sid.teleop_target(1.0, 1.5, 0.9, b)               # 2.4, inside bounds
    for _ in range(1000):
        cmd, v = sid.slew_toward(cmd, v, target, 2.0, 250.0, seg_t)
        assert b.lo_rev - 1e-9 <= cmd <= b.hi_rev + 1e-9
        if abs(cmd - target) < 1e-6 and abs(v) < 1e-6:
            break


def test_slew_toward_rejects_bad_seg_t():
    with pytest.raises(ValueError):
        sid.slew_toward(1.5, 0.0, 2.0, 2.0, 250.0, 0.0)


def test_gains_swap_allowed_gate():
    assert sid.gains_swap_allowed(0.0, 0.0)
    assert sid.gains_swap_allowed(0.04, 0.01)
    assert not sid.gains_swap_allowed(0.10, 0.0)               # moving
    assert not sid.gains_swap_allowed(0.0, 0.05)               # command leads the encoder
    # Custom tolerances honoured.
    assert sid.gains_swap_allowed(0.08, 0.03, vel_tol=0.1, dev_tol=0.05)


def test_parse_teleop_command_grammar():
    g = sid.parse_teleop_command('g 130 0.5 0.72')
    assert g.kind == 'gains'
    assert g.gains == sid.GainTriple(130.0, 0.5, 0.72)
    assert sid.parse_teleop_command('b').kind == 'baseline'
    assert sid.parse_teleop_command('r').kind == 'rearm'
    assert sid.parse_teleop_command('q').kind == 'quit'
    assert sid.parse_teleop_command('  Q  ').kind == 'quit'    # case + whitespace tolerant
    assert sid.parse_teleop_command('').kind == 'empty'
    assert sid.parse_teleop_command('g 1 2').kind == 'invalid'  # too few args
    assert sid.parse_teleop_command('g a b c').kind == 'invalid'  # non-numeric
    assert sid.parse_teleop_command('zzz').kind == 'invalid'


# --- review fix: stroke HF metrics exclude the stationary holds --------------

def test_stroke_moving_windows_cover_strokes_not_holds():
    # REGRESSION (review LOW): whole-window band-RMS over a sym stroke includes two
    # 0.4 s holds, deflating the RMS ~0.7x and masking the onset flag. The moving
    # windows must span exactly out-stroke and return (+ ring-down pad), not holds.
    spec = sid.StrokeSpec('symmetric', 0.4, 2.0, 2.0, 0.375, 0.375, 0.4,
                          123.0, 123.0, False, 1, 'sym_A0.40_v2.0')
    wins = sid.stroke_moving_windows(spec, ring_pad_s=0.15)
    assert wins == [(0.0, pytest.approx(0.525)),
                    (pytest.approx(0.775), pytest.approx(1.30))]
    # Second window starts at T_out + hold (return-stroke start), ends after
    # T_return + pad; the middle hold (0.525..0.775 s) is excluded.
    assert wins[0][1] < wins[1][0]


def test_stroke_moving_windows_none_for_holdless():
    sus = sid.StrokeSpec('sustained', 1.0, 3.8, 3.8, 0.49, 0.49, 0.0,
                         200.0, 200.0, False, 4, 'sustained_A1.00_v3.8_x4')
    assert sid.stroke_moving_windows(sus) is None
    nohold = sid.StrokeSpec('symmetric', 0.4, 2.0, 2.0, 0.375, 0.375, 0.0,
                            123.0, 123.0, False, 1, 'sym')
    assert sid.stroke_moving_windows(nohold) is None


# --- fix: sustained strokes sized by the 2A swing + series-level velocity gate

def test_sustained_stroke_series_respects_peak_velocity():
    # REGRESSION for the 2026-07-12 triple-latch: sustained swings travel 2A
    # (center+A -> center-A) but were timed for A, commanding 2x the intended
    # peak velocity (7.6 rev/s at A=1.0/v=3.8 >> vel_limit 4.0) and latching
    # MAX_DEVIATION at every gain point. Post-fix the commanded peak must stay
    # at the requested velocity for EVERY segment.
    bounds = sid.stroke_bounds(3.0, 0.15)
    battery = sid.stroke_battery([1.0], [3.8], vel_cap=4.0, accel_cap=250.0,
                                 bounds=bounds, center=1.5)
    sus = [s for s in battery if s.kind == 'sustained'][0]
    series = sid.stroke_series(sus, 1.5, 0.01, bounds)
    pk = sid.series_peak_velocity(series, 0.01)
    assert pk <= 3.8 * 1.05
    assert pk > 3.0                       # still an aggressive stroke, not degenerate
    # Swing duration is sized for distance 2A: T ~= 1.875*2A/v.
    assert sus.T_out_s == pytest.approx(1.875 * 2.0 / 3.8, rel=1e-6)


def test_series_peak_velocity_basic_and_degenerate():
    assert sid.series_peak_velocity(np.array([0.0, 0.05, 0.10]), 0.025) == \
        pytest.approx(2.0)
    assert sid.series_peak_velocity(np.array([1.0]), 0.025) == 0.0
    assert sid.series_peak_velocity(np.array([1.0, 1.1]), 0.0) == 0.0


def test_sym_asym_strokes_unaffected_by_sustained_fix():
    bounds = sid.stroke_bounds(3.0, 0.15)
    battery = sid.stroke_battery([0.4], [2.0], vel_cap=4.0, accel_cap=250.0,
                                 bounds=bounds, center=1.5)
    for s in battery:
        series = sid.stroke_series(s, 1.5, 0.01, bounds)
        assert sid.series_peak_velocity(series, 0.01) <= 4.0 * 1.05, s.label


# --- Stroke tracking-error units (the 2026-07-12 14.2x mm/milli-rev bug) -----

def test_format_stroke_error_scales_rev_to_mm_not_milli_rev():
    """The 2026-07-12 regression: err (in REV) was printed as ``err * 1e3`` under
    an "mm" label — milli-REVOLUTIONS, 14.2x too large. It reported a real
    0.72 mm tracking error as "10.3 mm" and manufactured the retracted "pos 70
    accuracy knee". The only correct scale is the leg's measured mm/rev."""
    mm_per_rev = 71.5708                     # bench leg (single_leg_test.py:106)
    err_rev = 0.01034                        # the real pos-40 battery mean (222801/091748)
    s = sid.format_stroke_error(err_rev, 2.0 * err_rev, mm_per_rev)
    # 0.01034 rev x 71.5708 mm/rev = 0.74 mm -- NOT the "10.3 mm" the 1e3 scale printed.
    assert 'errRMS=0.74 mm' in s
    assert 'peak=1.48 mm' in s
    assert '10.3' not in s and '10.2' not in s


def test_format_stroke_error_is_linear_in_mm_per_rev():
    """Guards the specific failure mode: a hard-coded scale factor. The output
    must track mm_per_rev, so substituting 1e3 (or any constant) fails here."""
    a = sid.format_stroke_error(0.02, 0.02, 70.5)     # platform leg
    b = sid.format_stroke_error(0.02, 0.02, 71.5708)  # bench leg
    assert 'errRMS=1.41 mm' in a
    assert 'errRMS=1.43 mm' in b
    assert a != b


def test_format_stroke_error_zero_and_sign_agnostic():
    assert 'errRMS=0.00 mm' in sid.format_stroke_error(0.0, 0.0, 71.5708)
    # err arrays are pre-abs'd by the caller, but the formatter must not lie if
    # a signed value reaches it.
    assert 'errRMS=-0.72 mm' in sid.format_stroke_error(-0.01, 0.01, 71.5708)
