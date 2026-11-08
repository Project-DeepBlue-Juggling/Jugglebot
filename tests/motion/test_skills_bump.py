"""I-CATCH-5 — the late catch re-aim BUMP (``motion/skills/bump.py::apply_catch_bump``).

Plan ``plans/archived/late-catch-reaim-bump.md``; design entry
``logbook/2026-10-16-late-catch-reaim-bump-design.md`` (§ Implementation G1).

The record every test bumps is a REAL catch-with-throw installed here by
``executor.install_segment`` (a THROW from rest, then the CATCH carrying a
throw spliced at the release, R5 sitting limits 350 / 5000 / 200 000, hand
3900) — not the probe's gitignored ``temp/probes/late_reaim/records_columns_0.pkl``,
so the gate never depends on a regenerated artefact.  The numbers each test
pins were confirmed by a probe on this exact construction (2026-10-16):
Δ (20, 0) 0.45 s out installs with b(t_rel) 16.27 mm / v_b -18.98 mm/s;
Δ (60, 0) 0.30 s out is refused LIMIT_VEL (384.8 > 350 mm/s); Δ (20, 0)
0.25 s out LIMIT_ACC; 0.20 s out BUMP-FREEZE; Δ (60, 0) at 0.45 s has
|v_b| 57.0 mm/s > the 50 mm/s owner bound.
"""
from __future__ import annotations

import dataclasses
import math

import numpy as np
import pytest

import jugglebot.hardware_config as hw
from jugglebot.motion import unified_cycle as uc
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.skills import bump
from jugglebot.motion.skills import executor as ex
from jugglebot.motion.skills import schedule as sc
from jugglebot.motion.skills import segments as sg
from jugglebot.motion.skills import sites as si
from jugglebot.motion.skills.schedule import Schedule, Skill
from jugglebot.motion.trajectory import cup_realize as cr
from jugglebot.motion.trajectory.limits import TrajectoryLimits

FLIGHT_S = 0.857
APEX_M = sc.apex_m(FLIGHT_S)
LAUNCH_S = 0.4
T0_ABS = 100.0
DWELL_S = 0.30
T_REL0 = T0_ABS + LAUNCH_S
T_LAND = T_REL0 + FLIGHT_S
LAND_VEL = np.array([0.0, 0.0, -FLIGHT_S / 2.0 * 9806.0])


@pytest.fixture(scope='module')
def geom():
    return StewartGeometry()


@pytest.fixture(scope='module')
def limits():
    return TrajectoryLimits.from_config(hw).with_sitting_limits(
        leg_vel_mmps=350.0, leg_acc_mmps2=5000.0, leg_jerk_mmps3=200000.0,
        hand_acc_rps2=float(hw.JB_TRAJ_HAND_ACC_LIMIT_RPS2))


@pytest.fixture(scope='module')
def sites():
    return si.columns_sites(100.0)


def _rest_state(cup_mm) -> uc.CycleState:
    cfg = cr.RealizeConfig()
    rev = ((float(cup_mm[2]) - cfg.cup_z_base_mm - cfg.slider_rev_zero_mm)
           / 1000.0 * cr.HAND_REV_PER_M)
    pose = np.array([cup_mm[0], cup_mm[1], cfg.active_z_mm, 0.0, 0.0, 0.0])
    return uc.CycleState.at_rest(pose, rev, cfg)


def _installer_over(box, limits, geom):
    """The sim gate's installer shape: ``install_segment`` over a local record."""
    def installer(kind, terminal, t_now_s, ball_id=None):
        rec, res, _seg = ex.install_segment(
            box.get('rec'), box.get('seed'), kind, terminal, t_now_s,
            limits=limits, geom=geom)
        if res.accepted:
            box['rec'] = rec
            box['seed'] = None
        return res
    return installer


@pytest.fixture(scope='module')
def record(limits, geom, sites):
    """THROW from rest at P1, then P2's CATCH carrying a throw (installed)."""
    p1, p2 = sites
    rec, res, _ = ex.install_segment(
        None, _rest_state(p1.rest_site_mm()), sg.THROW,
        sg.ThrowTerminal(site_mm=p1.throw_site_mm(), target_mm=p1.catch_site_mm(),
                         flight_s=FLIGHT_S, t_release_s=T_REL0),
        T0_ABS, limits=limits, geom=geom)
    assert res.accepted, res.message
    terminal = sg.CatchTerminal(
        landing_mm=p2.catch_site_mm(), landing_vel_mm_s=LAND_VEL,
        t_land_s=T_LAND, rest_site_mm=p2.rest_site_mm(),
        then_throw=sg.ThrowAfterCatch(t_release_s=T_LAND + DWELL_S,
                                      site_mm=p2.throw_site_mm(),
                                      target_mm=p2.catch_site_mm(),
                                      flight_s=FLIGHT_S))
    rec2, res2, _ = ex.install_segment(rec, None, sg.CATCH, terminal,
                                       T_REL0 - ex.LEAD_S, limits=limits, geom=geom)
    assert res2.accepted, res2.message
    return rec2


def _k_wire(rec, to_land_s):
    return bump.k_wire_at(rec.t0_s, T_LAND - to_land_s, rec.plan.dt)


def _bump(rec, limits, geom, to_land_s, delta, **kw):
    return bump.apply_catch_bump(rec.plan, rec.meta, _k_wire(rec, to_land_s),
                                 delta, T_LAND - rec.t0_s, limits, geom, **kw)


def _carried(rec):
    return [r for r in rec.meta.releases if float(r.t_s) > T_LAND - rec.t0_s][0]


def test_every_knot_through_k_wire_plus_2_and_the_hand_are_bit_identical(
        record, limits, geom):
    res = _bump(record, limits, geom, 0.45, (20.0, 0.0))
    assert res.accepted, res.message
    k_wire = _k_wire(record, 0.45)
    assert res.k_b == k_wire + bump.BUMP_EXTRA_KNOTS == k_wire + 2
    kb = res.k_b
    assert np.array_equal(res.plan.pose[:kb + 1], record.plan.pose[:kb + 1])
    assert np.array_equal(res.plan.pose_vel[:kb + 1], record.plan.pose_vel[:kb + 1])
    # The first changed knot is k_b + 1, and only x/y ever change.
    assert not np.array_equal(res.plan.pose[kb + 1], record.plan.pose[kb + 1])
    assert np.array_equal(res.plan.pose[:, 2:], record.plan.pose[:, 2:])
    assert np.array_equal(res.plan.pose_vel[:, 2:], record.plan.pose_vel[:, 2:])
    assert np.array_equal(res.plan.hand_rev, record.plan.hand_rev)
    assert np.array_equal(res.plan.hand_vel_rps, record.plan.hand_vel_rps)
    assert res.plan.n_knots == record.plan.n_knots
    assert res.plan.contact_knots == record.plan.contact_knots


def test_the_carried_throw_is_retargeted_so_the_landing_offset_is_zero(
        record, limits, geom):
    res = _bump(record, limits, geom, 0.45, (20.0, 0.0))
    assert res.accepted
    rm = _carried(record)
    k_rel = int(round(float(rm.t_s) / record.plan.dt))
    b = res.plan.pose[:, 0:2] - record.plan.pose[:, 0:2]
    bd = res.plan.pose_vel[:, 0:2] - record.plan.pose_vel[:, 0:2]
    # Realised release offset = the closed form, and NOT delta.
    assert np.allclose(b[k_rel], res.release_offset_xy_mm, atol=1e-9)
    assert res.release_offset_xy_mm[0] == pytest.approx(17.0802, abs=1e-3)  # 16.2696 until 2026-10-31: pre-release hold default 0.100 -> 0.000 (hold-pinned-0.1 restores it)
    assert np.allclose(bd[k_rel], res.launch_bias_xy_mm_s, atol=1e-9)
    land = res.release_offset_xy_mm + res.launch_bias_xy_mm_s * float(rm.flight_s)
    assert np.allclose(land, 0.0, atol=1e-6)
    # Held at delta at the catch (the pre-hold), and the meta follows the plan.
    k_c = int(math.floor((T_LAND - record.t0_s) / record.plan.dt))
    assert np.allclose(b[k_c], (20.0, 0.0), atol=1e-9)
    rm2 = [r for r in res.meta.releases if abs(float(r.t_s) - float(rm.t_s)) < 1e-9][0]
    assert np.allclose(np.asarray(rm2.site_mm)[:2] - np.asarray(rm.site_mm)[:2],
                       res.release_offset_xy_mm, atol=1e-9)
    assert np.allclose(np.asarray(rm2.vel_mm_s)[:2] - np.asarray(rm.vel_mm_s)[:2],
                       res.launch_bias_xy_mm_s, atol=1e-9)


def test_the_record_ends_at_its_own_rest_site_with_zero_velocity(
        record, limits, geom):
    res = _bump(record, limits, geom, 0.45, (20.0, 20.0))
    assert res.accepted
    assert np.allclose(res.plan.pose[-1], record.plan.pose[-1], atol=1e-9)
    assert np.allclose(res.plan.pose_vel[-1], record.plan.pose_vel[-1], atol=1e-9)
    assert np.allclose(res.plan.pose_vel[-1, 0:2], 0.0, atol=1e-9)


def test_a_bump_beyond_the_limits_is_declined_with_the_gates_code_never_clipped(
        record, limits, geom):
    pose0 = record.plan.pose.copy()
    res = _bump(record, limits, geom, 0.30, (60.0, 0.0),
                max_launch_lateral_mm_s=float('inf'))
    assert not res.accepted and res.code == 'LIMIT_VEL' and not res.terminal
    assert res.plan is None and res.report is not None and res.report.code == 'LIMIT_VEL'
    assert np.array_equal(record.plan.pose, pose0)
    res = _bump(record, limits, geom, 0.25, (20.0, 0.0))
    assert not res.accepted and res.code == 'LIMIT_ACC' and res.plan is None


def test_bump_freeze_once_the_ramp_has_no_runway(record, limits, geom):
    res = _bump(record, limits, geom, 0.20, (20.0, 0.0))
    assert not res.accepted and res.code == bump.BUMP_FREEZE and res.terminal
    assert bump.BUMP_FREEZE_LEAD_S == pytest.approx(0.225)


def test_the_50_mm_s_launch_bound_declines_above_it(record, limits, geom):
    assert bump.BUMP_MAX_LAUNCH_LATERAL_MM_S == 50.0
    res = _bump(record, limits, geom, 0.45, (60.0, 0.0))
    assert not res.accepted and res.code == bump.BUMP_LAUNCH and not res.terminal
    assert float(np.hypot(*res.launch_bias_xy_mm_s)) > 50.0
    # The same delta without the bound passes the gate: the bound declined it.
    assert _bump(record, limits, geom, 0.45, (60.0, 0.0),
                 max_launch_lateral_mm_s=float('inf')).accepted


def test_a_ramp_start_before_k_b_is_a_contract_violation(record, limits, geom):
    k_wire = _k_wire(record, 0.45)
    with pytest.raises(ValueError):
        bump.apply_catch_bump(record.plan, record.meta, k_wire, (20.0, 0.0),
                              T_LAND - record.t0_s, limits, geom,
                              t_ramp_start_s=(k_wire + 1) * record.plan.dt)


def test_the_node_side_splice_check_is_non_terminal(record, limits, geom):
    t_disp = T_LAND - 0.45
    res, k_wire = bump.bump_record(record.plan, record.meta, record.t0_s, t_disp,
                                   (20.0, 0.0), T_LAND, limits, geom,
                                   t_apply_s=t_disp + 3 * record.plan.dt)
    assert res.code == bump.BUMP_SPLICE_TOO_LATE and not res.terminal
    res, _ = bump.bump_record(record.plan, record.meta, record.t0_s, t_disp,
                              (20.0, 0.0), T_LAND, limits, geom,
                              t_apply_s=t_disp + 0.010)
    assert res.accepted


def test_the_per_pattern_switch_is_columns_only_and_off_by_default(sites):
    assert bump.bump_enabled_for('columns')
    assert bump.bump_enabled_for('columns_1ball_fed')
    for name in ('hop', 'reload', 'magazine', 'self_toss', None):
        assert not bump.bump_enabled_for(name)
    x = ex.SkillExecutor(_schedule(sites), lambda *a, **k: None)
    assert x.catch_bump_enabled is False


# ── the executor's pure layer (G1): catch_bump + commit_catch_bump ─────────

def _schedule(sites) -> Schedule:
    p1, p2 = sites
    tt = sc.ThenThrow(t_release_abs_s=T_LAND + DWELL_S,
                      y_d=(np.zeros(2), APEX_M), target=p2)   # columns = self-toss
    skills = (
        Skill(kind=sg.THROW, ball_id=0, site=p1, t_abs_s=T_REL0,
              window_s=LAUNCH_S, y_d=(np.zeros(2), APEX_M), target=p1),
        Skill(kind=sg.CATCH, ball_id=0, site=p2, t_abs_s=T_LAND,
              window_s=0.278, then_throw=tt),
    )
    return Schedule(pattern='columns', skills=skills, flight_s=FLIGHT_S,
                    beat_s=0.578, transit_s=0.278, dwell_s=DWELL_S, t0_abs_s=T0_ABS)


def _live(sites, limits, geom, enabled=True, on_experience=None):
    p1, _p2 = sites
    box = {'seed': _rest_state(p1.rest_site_mm())}
    sch = _schedule(sites)
    land = {'l': ex.Landing(pos_mm=sites[1].catch_site_mm(), vel_mm_s=LAND_VEL,
                            t_land_abs_s=T_LAND, from_fit=True)}
    x = ex.SkillExecutor(sch, _installer_over(box, limits, geom),
                         tracker=lambda b: land['l'], catch_bump_enabled=enabled,
                         on_experience=on_experience)
    x.tick(sch.skills[0].dispatch_s())
    x.tick(sch.skills[1].dispatch_s())
    assert x._live_catch is not None and box['rec'].meta.catches
    return x, box, land


def test_an_accepted_executor_bump_records_the_realised_release_not_delta(
        sites, limits, geom):
    x, box, _land = _live(sites, limits, geom)
    rec = box['rec']
    site = x.schedule.skills[1].site
    x0 = x._seat_offset_m(1).copy()
    rel0 = x._carried_release_mm[1].copy()
    moved = ex.Landing(pos_mm=sites[1].catch_site_mm() + np.array([20.0, 0.0, 0.0]),
                       vel_mm_s=LAND_VEL, t_land_abs_s=T_LAND, from_fit=True)
    t = T_LAND - 0.45
    d = x.catch_bump(rec.plan, rec.meta, rec.t0_s, t, limits, geom, landing=moved)
    assert d.reason == 'OK', d
    assert np.allclose(d.delta_xy_mm, (20.0, 0.0))
    # catch_bump is pure: nothing recorded until commit.
    assert np.array_equal(x._carried_release_mm[1], rel0)
    lines = x.commit_catch_bump(d, t)
    assert lines and 'BUMP skill 1' in lines[-1]
    b_rel = d.result.release_offset_xy_mm
    assert np.allclose(x._carried_release_mm[1][:2] - rel0[:2], b_rel, atol=1e-9)
    assert not np.allclose(b_rel, d.delta_xy_mm)
    # The learner row's x[2:4] is the release offset the bumped plan executes.
    assert np.allclose(x._seat_offset_m(1) - x0, b_rel / 1000.0, atol=1e-12)
    assert x._resend_counts[1] == 1
    assert np.allclose(x._live_catch[1].landing_mm[:2],
                       sites[1].catch_site_mm()[:2] + (20.0, 0.0))


def test_an_accepted_bump_moves_the_pending_memory_rows_release(
        sites, limits, geom):
    """BUG-81 F1 on the BUMP path: the carried Throw's Memory row was
    registered at the Catch's dispatch; an accepted bump moves the release by
    b(t_rel), and the pending row must record that release (rel and seat),
    not the dispatch aim. The release velocity (the Learner's kernel key)
    stays the query's."""
    x, box, _land = _live(sites, limits, geom, on_experience=lambda e: None)
    rec = box['rec']
    pend, = [p for p in x._pending_outcomes
             if p.t_release_s == pytest.approx(T_LAND + DWELL_S)]
    rel0 = np.asarray(pend.ctx.release_m, dtype=float).copy()
    vel0 = np.asarray(pend.ctx.release_vel_mps, dtype=float).copy()
    moved = ex.Landing(pos_mm=sites[1].catch_site_mm() + np.array([20.0, 0.0, 0.0]),
                       vel_mm_s=LAND_VEL, t_land_abs_s=T_LAND, from_fit=True)
    t = T_LAND - 0.45
    d = x.catch_bump(rec.plan, rec.meta, rec.t0_s, t, limits, geom, landing=moved)
    assert d.reason == 'OK', d
    x.commit_catch_bump(d, t)
    b_rel = np.asarray(d.result.release_offset_xy_mm, dtype=float)
    assert float(np.hypot(*b_rel)) > 1.0
    np.testing.assert_allclose(np.asarray(pend.ctx.release_m)[:2] - rel0[:2],
                               b_rel / 1000.0, atol=1e-12)
    assert pend.ctx.release_m[2] == rel0[2]
    np.testing.assert_allclose(pend.ctx.release_m,
                               x._carried_release_mm[1] / 1000.0, atol=1e-12)
    np.testing.assert_allclose(pend.ctx.seat_offset_m, x._seat_offset_m(1))
    np.testing.assert_allclose(pend.x[2:4], x._seat_offset_m(1))
    np.testing.assert_allclose(pend.ctx.release_vel_mps, vel0)


def test_executor_fences_off_freeze_and_the_window_retry_rule(sites, limits, geom):
    x, box, _land = _live(sites, limits, geom, enabled=False)
    rec = box['rec']
    moved = ex.Landing(pos_mm=sites[1].catch_site_mm() + np.array([20.0, 0.0, 0.0]),
                       vel_mm_s=LAND_VEL, t_land_abs_s=T_LAND, from_fit=True)
    assert x.catch_bump(rec.plan, rec.meta, rec.t0_s, T_LAND - 0.45, limits, geom,
                        landing=moved).reason == 'BUMP-OFF'
    x, box, _land = _live(sites, limits, geom)
    rec = box['rec']
    unfitted = dataclasses.replace(moved, from_fit=False)
    assert x.catch_bump(rec.plan, rec.meta, rec.t0_s, T_LAND - 0.45, limits, geom,
                        landing=unfitted).reason == 'NO-CONVERGED-FIT'
    small = dataclasses.replace(
        moved, pos_mm=sites[1].catch_site_mm() + np.array([5.0, 0.0, 0.0]))
    assert x.catch_bump(rec.plan, rec.meta, rec.t0_s, T_LAND - 0.45, limits, geom,
                        landing=small).reason == 'WITHIN-TOLERANCE'
    # A gate refusal is BUMP-WINDOW: non-terminal, no cap spent, and the same
    # delta is not re-gated until it has shrunk by resend_pos_tol_mm.
    d = x.catch_bump(rec.plan, rec.meta, rec.t0_s, T_LAND - 0.25, limits, geom,
                     landing=moved)
    assert d.reason == 'BUMP-WINDOW' and not d.terminal and d.result.code == 'LIMIT_ACC'
    x.commit_catch_bump(d, T_LAND - 0.25)
    assert x._live_catch is not None and x._resend_counts.get(1, 0) == 0
    d2 = x.catch_bump(rec.plan, rec.meta, rec.t0_s, T_LAND - 0.24, limits, geom,
                      landing=moved)
    assert d2.reason == 'BUMP-WINDOW' and d2.result is None
    # Past t_c - 0.225: BUMP-FREEZE, terminal — re-aiming ends.
    d3 = x.catch_bump(rec.plan, rec.meta, rec.t0_s, T_LAND - 0.22, limits, geom,
                      landing=moved)
    assert d3.reason == bump.BUMP_FREEZE and d3.terminal
    x.commit_catch_bump(d3, T_LAND - 0.22)
    assert x._live_catch is None


# ── G2: the live route in _resend_live_catch (2026-10-16) ───────────────────

def _routed(sites, limits, geom, enabled=True):
    """``_live`` plus a counting bumper over the local record (the sim gate's
    shape) and a counting installer wrapper."""
    x, box, land = _live(sites, limits, geom, enabled=enabled)
    calls = {'bump': 0, 'install': 0}
    inner = x.installer

    def installer(*a, **k):
        calls['install'] += 1
        return inner(*a, **k)

    def bumper(idx, ball_id, delta, t_c_abs, t_disp):
        calls['bump'] += 1
        rec = box['rec']
        res, _k = bump.bump_record(rec.plan, rec.meta, rec.t0_s, t_disp, delta,
                                   t_c_abs, limits, geom, t_apply_s=t_disp)
        if res.accepted:
            box['rec'] = dataclasses.replace(rec, plan=res.plan, meta=res.meta)
        return res

    x.installer = installer
    x.bumper = bumper
    land['l'] = ex.Landing(pos_mm=sites[1].catch_site_mm() + np.array([20.0, 0.0, 0.0]),
                           vel_mm_s=LAND_VEL, t_land_abs_s=T_LAND, from_fit=True)
    return x, box, land, calls


def test_past_the_snap_band_a_carried_catch_reaim_reaches_the_bumper(
        sites, limits, geom):
    x, box, _land, calls = _routed(sites, limits, geom)
    t = T_LAND - 0.45
    assert not x._resend_splice_instant(1, t, T_LAND)[3], 'must be past the band'
    site = x.schedule.skills[1].site
    x0 = x._seat_offset_m(1).copy()
    rec0 = box['rec']
    lines = x._resend_live_catch(t)
    assert calls == {'bump': 1, 'install': 0}, calls
    assert any(' BUMP skill 1:' in ln and 'installed b(t_rel)' in ln
               and 'v_b' in ln for ln in lines), lines
    assert box['rec'] is not rec0 and box['rec'].t0_s == rec0.t0_s
    # The learner row's x[2:4] moves by the REALISED b(t_rel), not by delta.
    moved = (x._seat_offset_m(1) - x0) * 1000.0
    assert 0.0 < float(np.hypot(*moved)) < 20.0 - 1e-6
    assert x._resend_counts[1] == 1


def test_catch_freeze_does_not_pre_empt_bump_freeze_on_the_bump_path(
        sites, limits, geom):
    # t_c - 0.24: inside catch_freeze_s (0.25) but before BUMP-FREEZE (0.225).
    t = T_LAND - 0.24
    x, _box, _land, calls = _routed(sites, limits, geom)
    x._resend_live_catch(t)
    assert calls['bump'] == 1 and calls['install'] == 0
    assert x._live_catch is not None          # a gate refusal is non-terminal
    # The same tick with the bump OFF is the re-solve path's freeze: re-aiming ends.
    x2, _b2, _l2, calls2 = _routed(sites, limits, geom, enabled=False)
    assert x2._resend_live_catch(t) == []
    assert x2._live_catch is None and calls2 == {'bump': 0, 'install': 0}
    # Past t_c - 0.225: BUMP-FREEZE, terminal, the bumper is never asked.
    x3, _b3, _l3, calls3 = _routed(sites, limits, geom)
    lines = x3._resend_live_catch(T_LAND - 0.22)
    assert calls3['bump'] == 0 and x3._live_catch is None
    assert any('BUMP-DECLINED BUMP-FREEZE skill 1' in ln for ln in lines), lines


def test_a_catch_without_a_carried_throw_keeps_the_raw_resolve_path(
        sites, limits, geom):
    """BUMP-NO-CARRY decision (G2): the attempt's last catch carries no
    release, so it falls back to today's raw re-solve (installer)."""
    x, _box, _land, calls = _routed(sites, limits, geom)
    idx, term, t_last = x._live_catch
    x._live_catch = (idx, dataclasses.replace(term, then_throw=None), t_last)
    x._resend_live_catch(T_LAND - 0.45)
    assert calls['bump'] == 0 and calls['install'] == 1


def test_a_non_gate_retry_code_is_retried_and_launch_joins_the_shrink_rule(
        sites, limits, geom):
    x, _box, _land, _calls = _routed(sites, limits, geom)
    seq = []

    def bumper(idx, ball_id, delta, t_c_abs, t_disp):
        code = bump.BUMP_HANDOFF if not seq else bump.BUMP_LAUNCH
        seq.append(code)
        return bump.BumpResult(False, code, False, code, 0,
                               delta_xy_mm=np.asarray(delta, float))
    x.bumper = bumper
    lines = x._resend_live_catch(T_LAND - 0.45)
    assert any('BUMP-DECLINED BUMP_HANDOFF' in ln for ln in lines)
    assert x._live_catch is not None and 1 not in x._bump_refused_mm
    x._resend_live_catch(T_LAND - 0.42)
    assert seq == [bump.BUMP_HANDOFF, bump.BUMP_LAUNCH]
    assert x._bump_refused_mm.get(1) is not None and x._live_catch is not None
    x._resend_live_catch(T_LAND - 0.39)       # same delta: not re-asked
    assert len(seq) == 2


def test_the_launch_bound_is_on_the_total_velocity_plan_plus_bias(
        record, limits, geom):
    """Bias alone passes the 50 mm/s bound, bias + the plan's own release
    lateral velocity does not -> BUMP-LAUNCH (the bound is on the TOTAL)."""
    ok = _bump(record, limits, geom, 0.45, (40.0, 0.0),
               max_launch_lateral_mm_s=float('inf'))
    assert ok.accepted, ok.message
    v_b = np.asarray(ok.launch_bias_xy_mm_s, float)
    nb = float(np.hypot(*v_b))
    assert 0.0 < nb < 50.0
    # Give the unbumped record's carried release a lateral velocity along the
    # bias, so the total exceeds the bound while the bias alone does not.
    rm = _carried(record)
    kick = np.zeros(3)
    kick[:2] = v_b / nb * (50.0 - nb + 5.0)
    rm2 = dataclasses.replace(rm, vel_mm_s=np.asarray(rm.vel_mm_s, float) + kick)
    rels = [rm2 if r is rm else r for r in record.meta.releases]
    meta2 = dataclasses.replace(record.meta, releases=rels)
    res = bump.apply_catch_bump(record.plan, meta2, _k_wire(record, 0.45),
                                (40.0, 0.0), T_LAND - record.t0_s, limits, geom)
    assert not res.accepted and res.code == bump.BUMP_LAUNCH and not res.terminal
    assert float(np.hypot(*res.launch_bias_xy_mm_s)) < 50.0   # the bias alone passes


# ---------------------------------------------------------------------------
# I-PLAN-10 — the bump updates the derived cup track, not only the pose
# ---------------------------------------------------------------------------

def test_a_bumped_meta_cup_track_decomposes_to_the_bumped_pose(
        record, limits, geom):
    """I-PLAN-10: the bump is a pure xy overlay on the pose, so ``meta.cup_plan``
    must carry the same overlay or ``splice_at`` re-derives the seam velocity
    from a stale track (the 2026-10-19 17:47 / 15:23 LIMIT_JERK).

    Recipe (probed 2026-10-19 on this ``record``, delta (20, 0), 0.45 s before
    landing, v_b -18.98 mm/s; also delta (12, 0) v_b 11.39): on every knot
    ``decompose(meta.cup_plan, meta.tilts).pose[:, :2]`` equals ``plan.pose[:,
    :2]`` (measured 7e-15 mm; asserted <= 1e-9) and ``meta.tilts`` equals
    ``plan.pose[:, 3:5]``.  The cup track's vel/acc overlay equals the analytic
    b_dot/b_ddot.  Acc is compared to a FINE central FD (h = 1e-6 s) of
    ``retarget_profile``'s own b_dot, never a 25 ms FD of the knots: the 25 ms
    central FD is 278-500 mm/s^2 off on this fixture (the quintic ramp is far
    from quadratic over 50 ms), which would either hide a wrong acc or force a
    tolerance too loose to mean anything.  A control (the unbumped record) is
    coherent too, so the check is not vacuous.
    """
    cfg = uc.build_realize_config(limits)
    base = cr.decompose(record.meta.cup_plan, record.meta.tilts, cfg)
    assert np.abs(base.pose[:, :2] - record.plan.pose[:, :2]).max() <= 1e-9

    for delta in ((20.0, 0.0), (12.0, 0.0)):
        res = _bump(record, limits, geom, 0.45, delta)
        assert res.accepted, res.message
        got = cr.decompose(res.meta.cup_plan, res.meta.tilts, cfg)
        assert np.abs(got.pose[:, :2] - res.plan.pose[:, :2]).max() <= 1e-9
        assert np.abs(res.meta.tilts - res.plan.pose[:, 3:5]).max() <= 1e-9

        d_pos = (res.meta.cup_plan.pos - record.meta.cup_plan.pos)[:, :2] * 1000.0
        d_vel = (res.meta.cup_plan.vel - record.meta.cup_plan.vel)[:, :2] * 1000.0
        d_acc = (res.meta.cup_plan.acc - record.meta.cup_plan.acc)[:, :2] * 1000.0
        # position overlay == the pose overlay; velocity overlay == the plan's
        assert np.abs(d_pos - (res.plan.pose - record.plan.pose)[:, :2]).max() <= 1e-9
        assert np.abs(d_vel - (res.plan.pose_vel - record.plan.pose_vel)[:, :2]).max() <= 1e-6
        assert np.abs(d_vel).max() > 5.0       # non-vacuous: v_b is ~11-19 mm/s
        assert np.abs(d_acc).max() > 100.0

        # acc overlay vs a fine FD of retarget_profile's own b_dot
        t_land = T_LAND - record.t0_s
        rel = _carried(record)
        k_b = res.k_b
        dt = float(record.plan.dt)
        t = np.arange(record.plan.n_knots) * dt
        t_b = res.k_b * dt
        pre_s = int(rel.pre_hold_knots) * dt
        post_s = uc.post_release_hold_knots() * dt
        args = (t_b, t_land - bump.BUMP_PRE_HOLD_S, t_land, float(rel.t_s),
                pre_s, post_s, float(record.plan.t[-1]), float(rel.flight_s),
                np.asarray(delta, float))
        h = 1e-6
        _b, bd_p = bump.retarget_profile(t + h, *args)
        _b, bd_m = bump.retarget_profile(t - h, *args)
        fd = (bd_p - bd_m) / (2 * h)
        fd[:k_b + 1] = 0.0
        assert np.abs(d_acc[k_b + 1:] - fd[k_b + 1:]).max() < 1.0, (
            np.abs(d_acc - fd).max())


def _splice_catch_at_the_bumped_release(rec_b, sites, limits, geom):
    """``install_segment`` a CATCH at the snapped carried release of ``rec_b``
    (the 17:47 / 15:23 sequence: bump, then the next catch splices there)."""
    _p1, p2 = sites
    rel2 = _carried(rec_b)
    t_rel_abs = rec_b.t0_s + float(rel2.t_s)
    term = sg.CatchTerminal(
        landing_mm=p2.catch_site_mm(), landing_vel_mm_s=LAND_VEL,
        t_land_s=t_rel_abs + FLIGHT_S, rest_site_mm=p2.rest_site_mm())
    t_now = t_rel_abs - ex.LEAD_S - 0.02
    k_s, _post = ex.splice_knot_for(rec_b, sg.CATCH, term, t_now,
                                    sc.install_lead_s(sg.CATCH))
    new, res, _seg = ex.install_segment(rec_b, None, sg.CATCH, term, t_now,
                                        limits=limits, geom=geom,
                                        t_install_s=t_now)
    return k_s, new, res


def test_a_splice_at_a_bumped_release_keeps_the_bump_velocity_at_the_seam(
        record, limits, geom, sites):
    """I-PLAN-10 end to end: bump 0.45 s before landing, then install the next
    CATCH at the bumped carried release (snapped knot K = 62).

    Recipe (probed 2026-10-19, ``repro_unit1_fixed.py``): control (no bump) is
    accepted, V[K] -0.43 mm/s.  Bumped by delta (12|20|36, 0) -> v_b 11.39 /
    18.98 / 34.17 mm/s (11.96 / 19.93 / 35.87 since 2026-10-31: all x1.050, the
    hold-0 default retargets a 5 % larger release offset): ALL ACCEPTED, and the seam velocity (the spliced plan's
    ``pose_vel[K]``, re-derived from the joined cup track) is within 2.5 mm/s
    of the bumped plan's ``pose_vel[K + 1]`` -- the healthy re-derivation
    baseline is ~1.9 mm/s (the FD of a coherent cup track against the stored
    velocity), so a missing v_b (11-34 mm/s) cannot hide inside it.  Pre-fix
    (stale ``meta.cup_plan`` = the unbumped track, injected here) the same
    install is refused CHAIN_DISCONTINUITY naming ``meta.cup_plan`` ("STALE at
    knot 61 ... |diff| 16.7" at v_b 19); before the I-PLAN-10 refusal existed it
    surfaced as LIMIT_JERK 288k-609k mm/s^3.
    """
    seam_tol = 2.5      # mm/s; healthy baseline ~1.9
    k_ctl, _new, res0 = _splice_catch_at_the_bumped_release(record, sites, limits, geom)
    assert res0.accepted, res0.message

    for delta, v_b_expect in (((12.0, 0.0), 11.96), ((20.0, 0.0), 19.93),
                              ((36.0, 0.0), 35.87)):
        r = _bump(record, limits, geom, 0.45, delta)
        assert r.accepted, r.message
        assert float(np.hypot(*r.launch_bias_xy_mm_s)) == pytest.approx(v_b_expect, abs=0.01)
        rec_b = ex.PlanRecord(plan=r.plan, meta=r.meta, t0_s=record.t0_s)

        k_s, new, res = _splice_catch_at_the_bumped_release(rec_b, sites, limits, geom)
        assert k_s == k_ctl == 62
        assert res.accepted, (delta, res.code, res.message)
        seam = np.asarray(new.plan.pose_vel[k_s, :2])
        want = np.asarray(rec_b.plan.pose_vel[k_s + 1, :2])
        assert np.abs(seam - want).max() <= seam_tol, (delta, seam, want)
        # the bias is IN the seam velocity, not merely close to a small number
        assert abs(seam[0]) > 0.5 * v_b_expect

        # Pre-fix shape: the unbumped (stale) cup track under the bumped pose.
        stale = dataclasses.replace(r.meta, cup_plan=record.meta.cup_plan)
        _k, _n, bad = _splice_catch_at_the_bumped_release(
            ex.PlanRecord(plan=r.plan, meta=stale, t0_s=record.t0_s),
            sites, limits, geom)
        assert not bad.accepted
        assert bad.code == uc.CHAIN_DISCONTINUITY, (delta, bad.code, bad.message)
        assert 'meta.cup_plan' in bad.message, bad.message


# ---------------------------------------------------------------------------
# C-HAND-5 x I-CATCH-5 — the bump re-derives the stepped release's brake seed
# ---------------------------------------------------------------------------

def _brake_seed_for(mark, cup_plan, dt, limits):
    """The C-HAND-5 brake seed ``_meta_for`` records, recomputed for ``mark``:
    ``release_brake_accel_mm_s2`` of the mark's take-off velocity and the cup
    track's release-knot acceleration ``a_L`` (the launch window's last knot,
    which ``_concat_cup`` keeps at a seam), to the production box top."""
    k_rel = int(round(float(mark.t_s) / float(dt)))
    return uc.release_brake_accel_mm_s2(
        np.asarray(mark.vel_mm_s, float),
        np.asarray(cup_plan.acc[k_rel], float) * 1000.0,
        float(mark.site_mm[2]), uc.build_cup_config().z_max_m * 1000.0, limits)


def test_a_bumped_stepped_release_reseeds_its_brake_for_the_bumped_takeoff(
        record, limits, geom):
    """The bump moves the carried release's velocity by ``v_b``; the C-HAND-5
    brake seed on that mark must be the one the un-bumped computation gives for
    the BUMPED take-off (and the bumped plan's own release acceleration
    ``a_L + b̈(t_rel)``), not the pre-bump one.

    Why it matters (2026-11-02 Sittings, debrief ``hand_acc.md``): the next
    catch is seeded from this mark (``release_state_at_knot``), and its detach
    rows pin knots 1-2's lateral acceleration to the brake through a near-
    vertical axis, so a seed whose lateral part ignores ``v_b`` is steered by
    the BRAKE MAGNITUDE: S2 L1489's successor peaked 4696 rev/s^2 offline
    (4509 live) against 3407 with the refreshed seed.

    Control: on the un-bumped record the stored seed IS this computation, so
    the helper is the production recipe, not a second formula.  Non-vacuous:
    probed 2026-11-02 on this record, the lateral seed moves by more than
    100 mm/s^2 at these deltas.
    """
    i = list(record.meta.releases).index(_carried(record))
    rm0 = record.meta.releases[i]
    assert rm0.brake_accel_mm_s2 is not None        # production steps the release
    dt = float(record.meta.dt)
    np.testing.assert_allclose(
        rm0.brake_accel_mm_s2,
        _brake_seed_for(rm0, record.meta.cup_plan, dt, limits), rtol=0, atol=1e-6)

    for delta in ((20.0, 0.0), (36.0, 0.0)):
        res = _bump(record, limits, geom, 0.45, delta)
        assert res.accepted, res.message
        rm = res.meta.releases[i]
        want = _brake_seed_for(rm, res.meta.cup_plan, dt, limits)
        got = np.asarray(rm.brake_accel_mm_s2, float)
        np.testing.assert_allclose(got, want, rtol=0, atol=1e-6)
        assert np.abs(got[:2] - np.asarray(rm0.brake_accel_mm_s2)[:2]).max() > 100.0
        # the step a_R - a_L is along the bumped take-off direction (C-HAND-5)
        k_rel = int(round(float(rm.t_s) / dt))
        step = got - np.asarray(res.meta.cup_plan.acc[k_rel], float) * 1000.0
        u = np.asarray(rm.vel_mm_s, float) / np.linalg.norm(rm.vel_mm_s)
        assert np.linalg.norm(step - np.dot(step, u) * u) <= 1e-9 * np.linalg.norm(step)


def test_a_continuous_release_keeps_no_brake_seed_through_a_bump(
        record, limits, geom):
    """With the step off the mark carries ``None`` (the next window starts
    from ``g``); the bump must not invent a seed."""
    i = list(record.meta.releases).index(_carried(record))
    rels = list(record.meta.releases)
    rels[i] = dataclasses.replace(rels[i], brake_accel_mm_s2=None)
    meta = dataclasses.replace(record.meta, releases=tuple(rels))
    res = bump.apply_catch_bump(record.plan, meta, _k_wire(record, 0.45),
                                (20.0, 0.0), T_LAND - record.t0_s, limits, geom)
    assert res.accepted, res.message
    assert res.meta.releases[i].brake_accel_mm_s2 is None


def test_a_stepped_release_without_a_cup_track_declines_the_bump(
        record, limits, geom):
    """``a_L`` comes off ``meta.cup_plan``; without it the seed cannot be
    re-derived, and a stale seed is the defect, so the bump is DECLINED
    (terminal, the old record keeps streaming) rather than accepted stale."""
    meta = dataclasses.replace(record.meta, cup_plan=None)
    res = bump.apply_catch_bump(record.plan, meta, _k_wire(record, 0.45),
                                (20.0, 0.0), T_LAND - record.t0_s, limits, geom)
    assert not res.accepted
    assert res.code == bump.BUMP_NO_CARRY and res.terminal
    assert 'brake seed' in res.message


def test_s2_l1489_the_bumped_successor_plans_inside_the_hand_limit(geom, monkeypatch):
    """Regression, 2026-11-02 Sitting 2 L1489 (Columns sep 125, Apex 0.95,
    dwell 0.27, pose, hold 0, hand 4500): Ball 1's catch at P2 was bumped
    (-30.5, -1.3) mm and Ball 2's next catch at P1, seeded from the bumped
    release, was refused HAND_LIMIT_ACC 4509 live.

    Recipe (``tools/probes/bump_seed_replay.py``'s construction with the logged
    learner commands, probed 2026-11-02): with the pre-fix STALE seed injected
    the successor refuses HAND_LIMIT_ACC (4696 offline); with the bump's own
    seed it plans at 3407 rev/s^2.  Asserted with margin: < 4000.
    """
    from jugglebot.motion.trajectory import ballistics_bc as bal
    # C-HAND-6 margin OFF: this pins the bump's brake re-seed against the
    # shape it was measured on (2026-11-02, no margin). With the margin on the
    # stale seed no longer drives the successor past 4500, so the fail-before
    # half would be vacuous; the re-seed logic itself is margin-independent.
    monkeypatch.setattr(uc, 'POST_RELEASE_TOP_MARGIN', False)
    apex, dwell, sep = 0.95, 0.27, 125.0
    lim = TrajectoryLimits.from_config(hw).with_sitting_limits(
        leg_vel_mmps=float(hw.JB_TRAJ_LEG_VEL_LIMIT_MMPS),
        leg_acc_mmps2=float(hw.JB_TRAJ_LEG_ACC_LIMIT_MMPS2),
        leg_jerk_mmps3=float(hw.JB_TRAJ_LEG_JERK_LIMIT_MMPS3),
        hand_acc_rps2=4500.0)
    p1, p2 = si.columns_sites(sep)
    tp = sc.flight_s(apex)
    tau = sc.transit_s(tp, dwell)
    cfg = dataclasses.replace(sg.SegmentConfig(planner_mode='pose'),
                              pre_release_hold_s=0.0)

    def mm(v):
        return np.array([float(v[0]), float(v[1]), 0.0])

    def cterm(site, u, ua):
        landing = site.catch_site_mm()
        release = np.array([landing[0], landing[1], site.throw_site_mm()[2]])
        r = release[:2] - np.asarray(site.cup_mm, float)[:2]
        gain = min(max(apex / ua, 0.5), 2.0)
        target = site.catch_site_mm() + mm(u) + mm(r * (1 - 1 / gain))
        vl = bal.launch_velocity(site.throw_site_mm(), landing, tp)
        return sg.CatchTerminal(
            landing_mm=landing, landing_vel_mm_s=bal.arrival_velocity(vl, tp),
            t_land_s=tau, rest_site_mm=site.rest_site_mm(),
            then_throw=sg.ThrowAfterCatch(t_release_s=tau + dwell, site_mm=release,
                                          target_mm=target,
                                          flight_s=sc.flight_s(ua)))

    launch = sg.plan_segment(
        sg.THROW, _rest_state(p1.rest_site_mm()),
        sg.ThrowTerminal(site_mm=p1.throw_site_mm(), target_mm=p1.throw_site_mm(),
                         flight_s=tp, t_release_s=0.4),
        cfg, lim, geom).release_state
    seg = sg.plan_segment(sg.CATCH, launch, cterm(p2, (7.8, -4.0), 0.868),
                          cfg, lim, geom)
    k_rel = int(round(seg.release_t_s / float(seg.plan.dt)))
    r = bump.apply_catch_bump(seg.plan, seg.meta, -1, (-30.5, -1.3),
                              float(seg.event_t_s), lim, geom)
    assert r.accepted, r.message
    nxt = cterm(p1, (-20.0, -16.6), 0.892)

    ok = sg.plan_segment(sg.CATCH, uc.release_state_at_knot(r.plan, r.meta, k_rel),
                         nxt, cfg, lim, geom)
    assert ok.meta.report.peak_hand_acc_rps2 < 4000.0

    # the pre-fix shape: the pre-bump seed on the bumped mark
    i = [j for j, m in enumerate(r.meta.releases)
         if int(round(float(m.t_s) / float(seg.plan.dt))) == k_rel][0]
    rels = list(r.meta.releases)
    rels[i] = dataclasses.replace(
        rels[i], brake_accel_mm_s2=seg.meta.releases[i].brake_accel_mm_s2)
    stale = dataclasses.replace(r.meta, releases=tuple(rels))
    with pytest.raises(uc.CycleInfeasible) as exc:
        sg.plan_segment(sg.CATCH, uc.release_state_at_knot(r.plan, stale, k_rel),
                        nxt, cfg, lim, geom)
    assert exc.value.code == 'HAND_LIMIT_ACC'


# ── I-CATCH-7 (BUG-42): the executor logs and commits the next-catch gate's call ──

def test_a_shrunk_bump_commits_the_applied_delta_and_logs_requested_and_applied(
        sites, limits, geom):
    """The gate here is a stand-in (refuses |Δ| > 12 mm) because this file's record
    is the R5 construction; the real planner's refusals are pinned in
    ``test_skills_bump_next_catch.py``.  What this pins is the executor's half:
    the BUMP line names requested and applied, the live terminal and the learner's
    carried release follow the APPLIED step, and the cap counts one re-aim."""
    x, box, _land = _live(sites, limits, geom)
    rec = box['rec']
    rel0 = x._carried_release_mm[1].copy()
    moved = ex.Landing(pos_mm=sites[1].catch_site_mm() + np.array([20.0, 0.0, 0.0]),
                       vel_mm_s=LAND_VEL, t_land_abs_s=T_LAND, from_fit=True)
    t = T_LAND - 0.45

    def gate(res):
        return (None if np.hypot(*res.delta_xy_mm) <= 12.0
                else 'HAND_LIMIT_ACC (stand-in)')
    d = x.catch_bump(rec.plan, rec.meta, rec.t0_s, t, limits, geom, landing=moved,
                     next_catch_gate=gate)
    assert d.reason == 'OK', d
    assert np.allclose(d.delta_xy_mm, (20.0, 0.0))          # the request
    assert np.allclose(d.result.delta_xy_mm, (10.0, 0.0))   # the applied step
    lines = x.commit_catch_bump(d, t)
    assert ('BUMP-SHRUNK step 1: requested (20.0, 0.0) -> applied (10.0, 0.0) mm'
            in lines[-1]), lines
    assert 'BUMP skill 1' in lines[-1]
    assert np.allclose(x._live_catch[1].landing_mm[:2],
                       sites[1].catch_site_mm()[:2] + (10.0, 0.0))
    assert np.allclose(x._carried_release_mm[1][:2] - rel0[:2],
                       d.result.release_offset_xy_mm, atol=1e-9)
    assert x._resend_counts[1] == 1


def test_a_bump_the_next_catch_refuses_at_every_step_is_declined_and_logged(
        sites, limits, geom):
    x, box, _land = _live(sites, limits, geom)
    rec = box['rec']
    moved = ex.Landing(pos_mm=sites[1].catch_site_mm() + np.array([20.0, 0.0, 0.0]),
                       vel_mm_s=LAND_VEL, t_land_abs_s=T_LAND, from_fit=True)
    t = T_LAND - 0.45
    d = x.catch_bump(rec.plan, rec.meta, rec.t0_s, t, limits, geom, landing=moved,
                     next_catch_gate=lambda res: 'HAND_LIMIT_ACC (stand-in)')
    assert d.reason == bump.BUMP_NEXT_CATCH and not d.terminal
    lines = x.commit_catch_bump(d, t)
    assert 'BUMP-DECLINED BUMP-NEXT-CATCH skill 1' in lines[-1], lines
    assert x._live_catch is not None                 # non-terminal: re-aim stays open
    assert x._bump_refused_mm[1] == pytest.approx(20.0)   # the shrink-retry rule
    assert x._resend_counts.get(1, 0) == 0
    assert np.allclose(x._live_catch[1].landing_mm[:2], sites[1].catch_site_mm()[:2])
