"""Pure-python half of ``hand_stream_bench.py --stage stroke_cut`` (BUG-55).

The stage COMMANDS THE HAND MOTOR and lives in tests/hardware (``--ignore``d);
its profile, cut selection, stop model, refusals and frame builder live in
``tests/hardware/hand_stroke_cut.py`` so they import with no link and no I/O.
The stop model is checked two ways: against the probe that sized FW 37
(``tools/probes/hand_stall_stop.py --option 2 --grace-ms <twin grace>``) and end to end
through the real SetpointPump and the firmware twin's scheduled lane.
"""
from __future__ import annotations

import argparse
import os
import sys

import pytest

_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
for _p in (os.path.join(_REPO, "tests", "hardware"),
           os.path.join(_REPO, "tools", "probes")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import hand_stroke_cut as SC  # noqa: E402
import teensy_interp as ti  # noqa: E402  (on sys.path via hand_stroke_cut)

START, TOP = 0.2, 8.5
# The grace the tool (and the firmware twin) uses; never a literal.
GRACE_MS = ti.HAND_C1_STOP_GRACE_US * 1e-3


def _fine(traj, dt=1e-4):
    n = int(traj.duration / dt) + 1
    return [(i * dt,) + tuple(traj.sample(i * dt)) for i in range(n)]


# ── the stroke profile ──────────────────────────────────────────────────────

@pytest.mark.parametrize("v_pk,accel", [(70.0, 800.0), (40.0, 800.0), (64.0, 1000.0)])
def test_stroke_profile_bounds_peak_and_c2(v_pk, accel):
    traj = SC.stroke_profile(START, TOP, v_pk, accel)
    s = _fine(traj)
    ps = [x[1] for x in s]
    vs = [x[2] for x in s]
    accs = [x[3] for x in s]
    assert min(ps) >= START - 1e-9 and max(ps) <= TOP + 1e-9
    assert max(ps) == pytest.approx(TOP, abs=1e-9)
    assert max(vs) == pytest.approx(v_pk, rel=1e-9)
    assert min(vs) == pytest.approx(-v_pk, rel=1e-9)
    assert max(abs(a) for a in accs) <= accel * (1 + 1e-9)
    end = traj.end
    assert end[0] == pytest.approx(START, abs=1e-9)
    assert abs(end[1]) < 1e-9 and abs(end[2]) < 1e-9
    # C2: acceleration continuous (bounded by the jerk across every step),
    # velocity and position continuous.
    jerk = accel / SC.STROKE_JERK_T
    dt = s[1][0] - s[0][0]
    for a, b in zip(s, s[1:]):
        assert abs(b[3] - a[3]) <= jerk * dt * (1 + 1e-6) + 1e-9
        assert abs(b[2] - a[2]) <= accel * dt * (1 + 1e-6) + 1e-9
        assert abs(b[1] - a[1]) <= v_pk * dt * (1 + 1e-6) + 1e-9


def test_stroke_too_short_for_the_speed_is_refused():
    with pytest.raises(ValueError, match="cannot reach"):
        SC.scurve_pieces(5.0, 70.0, 800.0, SC.STROKE_JERK_T)
    with pytest.raises(SC.StrokeCutRefusal, match="park the hand lower"):
        SC.plan_stroke_cut(3.0, TOP)


# ── the cut frame ───────────────────────────────────────────────────────────

@pytest.mark.parametrize("mode,peak,cut", [("upstroke", 70.0, 55.0),
                                          ("upstroke", 70.0, 64.0),
                                          ("downstroke", 70.0, 55.0),
                                          ("slow", 40.0, 36.0)])
def test_cut_is_the_first_frame_past_the_threshold(mode, peak, cut):
    plan = SC.plan_stroke_cut(START, TOP, mode, peak, cut)
    vd = SC.decision_velocity(SC.frame_knots(plan.traj, plan.cut))
    vprev = SC.decision_velocity(SC.frame_knots(plan.traj, plan.cut - 1))
    if mode == "downstroke":
        assert vd < -cut <= vprev
    else:
        assert vd > cut >= vprev
    assert plan.stop["path"] == ("C1" if mode == "upstroke" else "C2")
    assert plan.stop["entry"][1] == pytest.approx(vd) if mode == "upstroke" else True


def test_upstroke_default_numbers():
    """The figures the logbook and the Sitting quote (start 0.2, top 8.5)."""
    plan = SC.plan_stroke_cut(START, TOP)
    assert plan.cut == 21
    # FW 39 (HAND_C1_STOP_GRACE_US 14000): entry 63.2 rev/s, end 3.185 rev. History: at
    # FW 37 (6 ms grace) these were 56.8 rev/s and 2.619 rev; fw36_end is grace-independent.
    assert plan.stop["entry"][1] == pytest.approx(63.2, abs=0.05)
    assert plan.stop["end_rev"] == pytest.approx(3.185, abs=0.001)
    assert plan.fw36_end == pytest.approx(4.531, abs=0.001)
    assert plan.stop["end_rev"] + SC.CMD_TOL_REV < 0.5 * (plan.stop["end_rev"] + plan.fw36_end)


# ── the stop model vs the probe that sized FW 37 ────────────────────────────

def _probe_args(grace_ms):
    return argparse.Namespace(grace_ms=grace_ms, fast_up_thresh=50.0, stop_accel=4500.0,
                              accel=4500.0, jerk=350000.0, clamp_entry=False)


@pytest.mark.parametrize("cut", [53.0, 55.0, 60.0, 64.0])
def test_c1_stop_end_matches_the_probe_option_2(cut):
    hs = pytest.importorskip("hand_stall_stop")
    plan = SC.plan_stroke_cut(START, TOP, "upstroke", 70.0, cut)
    _, _, p1, v1, p2, v2, _ = plan.knots
    end, v_in, _a, where = hs.cover_end_stop("2", (p1, v1, p2, v2), _probe_args(GRACE_MS))
    assert where == "cover end +%g ms" % GRACE_MS
    assert v_in == pytest.approx(plan.stop["entry"][1], abs=1e-9)
    assert plan.stop["end_rev"] == pytest.approx(end, abs=1e-9)


@pytest.mark.parametrize("mode,peak,cut", [("slow", 40.0, 36.0), ("downstroke", 70.0, 55.0)])
def test_c2_stop_end_matches_the_probe_baseline(mode, peak, cut):
    hs = pytest.importorskip("hand_stall_stop")
    plan = SC.plan_stroke_cut(START, TOP, mode, peak, cut)
    _, _, p1, v1, p2, v2, _ = plan.knots
    if mode == "downstroke":
        # the probe's C2 travel only models an upward stop: mirror the frame
        end, *_ = hs.cover_end_stop("baseline", (-p1, -v1, -p2, -v2), _probe_args(0.0))
        end = -end
    else:
        end, *_ = hs.cover_end_stop("baseline", (p1, v1, p2, v2), _probe_args(0.0))
    assert plan.stop["end_rev"] == pytest.approx(end, abs=0.01)   # probe bisects on a grid


# ── refusals ────────────────────────────────────────────────────────────────

def test_top_too_high_is_refused():
    with pytest.raises(SC.StrokeCutRefusal) as e:
        SC.plan_stroke_cut(START, 9.8)
    assert any("under the 10.501 clip" in r for r in e.value.reasons)


def test_fw_too_old_is_refused():
    assert SC.stroke_cut_fw_refusal(38) is None
    assert SC.stroke_cut_fw_refusal(39) is None
    assert "37 < 38" in SC.stroke_cut_fw_refusal(37)
    assert "unknown" in SC.stroke_cut_fw_refusal(None)


def test_control_and_threshold_refusals():
    with pytest.raises(SC.StrokeCutRefusal, match="must stay below 45"):
        SC.plan_stroke_cut(START, TOP, "slow", 48.0, 40.0)
    with pytest.raises(SC.StrokeCutRefusal, match="clearly on one side"):
        SC.plan_stroke_cut(START, TOP, "upstroke", 70.0, 51.0)


# ── end to end: the real pump, the wire struct, the firmware twin ───────────

def _run_twin(plan, resume=True, resume_offset=0.0):
    from teensy_link.setpoint_pump import SetpointPump, FLAG_HAS_SCHED, FLAG_HAS_HAND
    from teensy_link import protocol as p
    pump = SetpointPump(mm_to_rev=[1.0] * 6, num_legs=6, max_step_hand_rev=5.0)
    it = ti.TeensyLegInterp([-1e9] * 6, [1e9] * 6)
    T_US = int(SC.SEG_T * 1e6)
    T0 = 1_000_000_000
    frames = []                                  # (stamp_us, kn or None)
    for k in range(plan.cut + 1):
        frames.append(SC.frame_knots(plan.traj, k))
    frames += [None] * int(round(SC.SETTLE_S / SC.SEG_T))
    out = {"hand": []}

    def feed(idx, kn):
        stamp = T0 + idx * T_US
        sp, why = pump.build(SC.sched_setpoint_cmd([0.0] * 6, [0.0] * 6, stamp, kn),
                             t_origin_us=stamp)
        assert sp is not None, why
        assert sp.flags & FLAG_HAS_SCHED
        sp = p.Setpoint.unpack(sp.pack())            # through the wire struct
        verdict = it.sched_on_setpoint(
            sp.t_origin_us, stamp - T_US, list(sp.u0), list(sp.u1), list(sp.u2),
            list(sp.v0), list(sp.v1), list(sp.v2),
            has_hand=bool(sp.flags & FLAG_HAS_HAND), accel=list(sp.accel))
        assert verdict == "sched"

    clock = {"now": T0 - T_US + 700}             # off the 2 ms grid, as on hardware

    def run(frames, idx0):
        events = [(T0 + (idx0 + i) * T_US - T_US, idx0 + i, kn)   # sent one knot early
                  for i, kn in enumerate(frames)]
        t_to = events[-1][0] + 2 * T_US
        ei = 0
        while clock["now"] < t_to:
            now = clock["now"]
            while ei < len(events) and events[ei][0] <= now:
                feed(events[ei][1], events[ei][2])
                ei += 1
            it.tick(now / 1e6, [0.0] * 6)
            if it.tick_hand(now / 1e6, fb_rev=it.hand_raw_pos) is not None:
                out["hand"].append((now, it.hand_raw_pos))
            clock["now"] += 2000
        return idx0 + len(frames)

    nxt = run(frames, 0)
    t_cut = T0 + plan.cut * T_US
    after = [h for t, h in out["hand"] if t >= t_cut]
    hold = out["hand"][-1][1]
    res = {"hold": hold, "peak": max(after), "low": min(after),
           "c1": it.sched_hand_c1_stops, "stops": it.sched_stops}
    if resume:
        hold_f32 = p.Setpoint.unpack(p.Setpoint(u0=(0.0,) * 6 + (hold,)).pack()).u0[6]
        ret = SC.return_profile(hold_f32 + resume_offset, plan.start)
        run([SC.frame_knots(ret, k) for k in range(int(ret.duration / SC.SEG_T) + 1)], nxt)
        res.update(refused=it.sched_refused, resumes=it.sched_resumes,
                   final=out["hand"][-1][1])
    return res


@pytest.mark.parametrize("mode,peak,cut", [("upstroke", 70.0, 55.0),
                                          ("downstroke", 70.0, 55.0),
                                          ("slow", 40.0, 36.0)])
def test_twin_runs_the_modelled_stop_and_accepts_the_resume(mode, peak, cut):
    plan = SC.plan_stroke_cut(START, TOP, mode, peak, cut)
    r = _run_twin(plan)
    assert r["hold"] == pytest.approx(plan.stop["end_rev"], abs=2e-3)
    if mode == "downstroke":
        assert r["low"] == pytest.approx(plan.stop["end_rev"], abs=2e-3)
    else:
        assert r["peak"] == pytest.approx(plan.stop["end_rev"], abs=2e-3)
    assert r["c1"] == (1 if mode == "upstroke" else 0)
    assert r["stops"] == 1
    assert r["refused"] == 0 and r["resumes"] == 1
    assert r["final"] == pytest.approx(START, abs=1e-3)


def test_verdict_scores_a_twin_shaped_run():
    plan = SC.plan_stroke_cut(START, TOP)
    end = plan.stop["end_rev"]
    t_end = 10.0
    echo = [(t_end - 0.05, end - 0.4), (t_end, end), (t_end + 0.3, end)]
    enc = [(t_end - 0.01, end - 0.2), (t_end + 0.03, end + 0.6), (t_end + 0.08, end + 0.04),
           (t_end + 0.3, end + 0.01)]
    v, crit, summ = SC.evaluate_stroke_cut(plan, echo, enc, t_end, 1, 1, 0)
    assert v == "PASS", crit
    assert summ["overshoot"] == pytest.approx(0.6)
    assert summ["settle_s"] == pytest.approx(0.08)
    v2, crit2, _ = SC.evaluate_stroke_cut(plan, echo, enc, t_end, 0, 1, 0)
    assert v2 == "FAIL" and [c["id"] for c in crit2 if c["verdict"] == "FAIL"] == ["S2"]
    fw36 = [(t_end, plan.fw36_end)]
    v3, crit3, _ = SC.evaluate_stroke_cut(plan, fw36, enc, t_end, 1, 1, 0)
    assert v3 == "FAIL" and crit3[0]["verdict"] == "FAIL"


def test_twin_refuses_a_resume_away_from_the_hold():
    """Negative control for the stage's resume: 0.2 rev off the hold (resume
    tolerance 0.05) is refused and latched, which the driver reads as
    sched_refused and answers by disarming at the hold."""
    plan = SC.plan_stroke_cut(START, TOP)
    r = _run_twin(plan, resume_offset=0.2)
    assert r["refused"] >= 1 and r["resumes"] == 0
    assert r["final"] == pytest.approx(r["hold"], abs=1e-6)


def test_slow_control_discriminates_the_full_grace():
    """The slow control's S1 separates the 20 ms grace from a stop at cover +
    6 ms by more than twice the tolerance, and scores each correctly."""
    plan = SC.plan_stroke_cut(START, TOP, "slow", 40.0, 36.0)
    end, early = plan.stop["end_rev"], plan.early_end
    assert end - early > 2 * SC.CMD_TOL_REV
    enc = [(10.0, end), (10.3, end)]
    ok, crit, _ = SC.evaluate_stroke_cut(plan, [(10.0, end)], enc, 10.0, 0, 1, 0)
    assert ok == "PASS", crit
    bad, crit, _ = SC.evaluate_stroke_cut(plan, [(10.0, early)], enc, 10.0, 0, 1, 0)
    assert bad == "FAIL" and crit[0]["verdict"] == "FAIL"
