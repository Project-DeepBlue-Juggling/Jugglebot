"""I-CATCH-7 — a BUMP is gated by the next catch (JB-120,
``motion/skills/bump.py::bump_with_next_catch_gate``).

Entry: ``logbook/2026-11-03-jb120-bump-gated-by-the-next-catch.md``.

The construction is ``tools/probes/bump_seed_replay.py``'s, the Columns pair of the
2026-11-01/02 Sittings (sep 125, Apex 0.95, dwell 0.27, planner pose, hold 0,
learner commands from the 2026-11-01 memory rows): Ball 2's CATCH at P1 carries a
throw, it is bumped along x, and Ball 1's next CATCH at P2 is solved from the
bumped release with the production ``plan_segment``.  The gate the tests inject is
that solve, so every pass / refuse below is the real planner's answer.

Probed on main 2026-11-03 (hand limit lifted to read the peak): the next catch's
peak Hand acceleration is 2477 / 3463 / 3661 rev/s^2 after a bump of 5 / 10 / 20 mm
in x, 2225 at 2.5 mm.  Nothing on main reaches the live 4500 (the 2026-11-02
brake-seed fix), so the gate limit here is :data:`GATE_HAND_RPS2` = 3600, between
the 10 and 20 mm successors: a real seed over a real gate, not a mock.
"""
from __future__ import annotations

import dataclasses

import numpy as np
import pytest

import jugglebot.hardware_config as hw
from jugglebot.motion import unified_cycle as uc
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.skills import bump
from jugglebot.motion.skills import schedule as sc
from jugglebot.motion.skills import segments as sg
from jugglebot.motion.skills import sites as si
from jugglebot.motion.trajectory import ballistics_bc as bal
from jugglebot.motion.trajectory import cup_realize as cr
from jugglebot.motion.trajectory.limits import TrajectoryLimits

APEX, DWELL, SEP = 0.95, 0.27, 125.0
U_APEX_B2, U_APEX_B1 = 0.8727, 0.8816
U_B2, U_B1 = (-11.0, -20.0), (-7.0, -10.0)
GATE_HAND_RPS2 = 3600.0


def _limits(hand):
    return TrajectoryLimits.from_config(hw).with_sitting_limits(
        leg_vel_mmps=float(hw.JB_TRAJ_LEG_VEL_LIMIT_MMPS),
        leg_acc_mmps2=float(hw.JB_TRAJ_LEG_ACC_LIMIT_MMPS2),
        leg_jerk_mmps3=float(hw.JB_TRAJ_LEG_JERK_LIMIT_MMPS3),
        hand_acc_rps2=float(hand))


def _mm(v):
    return np.array([float(v[0]), float(v[1]), 0.0])


@pytest.fixture(scope='module')
def case():
    geom = StewartGeometry()
    lim = _limits(4500.0)
    p1, p2 = si.columns_sites(SEP)
    tp = sc.flight_s(APEX)
    tau = sc.transit_s(tp, DWELL)
    cfg = dataclasses.replace(sg.SegmentConfig(planner_mode='pose'),
                              pre_release_hold_s=0.0)

    def cterm(site, u_mm, u_apex):
        landing = site.catch_site_mm()
        release = np.array([landing[0], landing[1], site.throw_site_mm()[2]])
        r = release[:2] - np.asarray(site.cup_mm, float)[:2]
        gain = min(max(APEX / u_apex, 0.5), 2.0)
        target = site.catch_site_mm() + _mm(u_mm) + _mm(r * (1.0 - 1.0 / gain))
        vl = bal.launch_velocity(site.throw_site_mm(), landing, tp)
        return sg.CatchTerminal(
            landing_mm=landing, landing_vel_mm_s=bal.arrival_velocity(vl, tp),
            t_land_s=tau, rest_site_mm=site.rest_site_mm(),
            then_throw=sg.ThrowAfterCatch(t_release_s=tau + DWELL, site_mm=release,
                                          target_mm=target,
                                          flight_s=sc.flight_s(u_apex)))

    rcfg = cr.RealizeConfig()
    rest = p2.rest_site_mm()
    rev = (((float(rest[2]) - rcfg.cup_z_base_mm) - rcfg.slider_rev_zero_mm)
           / 1000.0) * cr.HAND_REV_PER_M
    at_rest = uc.CycleState.at_rest(
        np.array([rest[0], rest[1], rcfg.active_z_mm, 0.0, 0.0, 0.0]), rev, rcfg)
    launch = sg.plan_segment(
        sg.THROW, at_rest,
        sg.ThrowTerminal(site_mm=p2.throw_site_mm(), target_mm=p2.throw_site_mm(),
                         flight_s=tp, t_release_s=0.4),
        cfg, lim, geom).release_state
    seg = sg.plan_segment(sg.CATCH, launch, cterm(p1, U_B2, U_APEX_B2),
                          cfg, lim, geom)
    nxt = cterm(p2, U_B1, U_APEX_B1)
    calls = []

    def gate_at(hand):
        glim = _limits(hand)

        def gate(res):
            calls.append(np.asarray(res.delta_xy_mm, float).copy())
            seed = uc.release_state_at_knot(res.plan, res.meta, res.carried_release_k)
            try:
                sg.plan_segment(sg.CATCH, seed, nxt, cfg, glim, geom)
            except uc.CycleInfeasible as exc:
                return '%s %s' % (exc.code, exc)
            return None
        return gate

    return dict(seg=seg, lim=lim, geom=geom, gate_at=gate_at, calls=calls,
                t_c=float(seg.event_t_s))


def _bump(case, delta, gate):
    del case['calls'][:]
    return bump.bump_with_next_catch_gate(
        case['seg'].plan, case['seg'].meta, -1, delta, case['t_c'], case['lim'],
        case['geom'], next_catch_gate=gate)


def test_a_bump_whose_next_catch_plans_is_applied_unchanged(case):
    res = _bump(case, (5.0, 0.0), case['gate_at'](GATE_HAND_RPS2))
    assert res.accepted, res.message
    assert res.shrink_steps == 0
    assert np.allclose(res.delta_xy_mm, (5.0, 0.0))
    assert np.allclose(res.requested_delta_xy_mm, (5.0, 0.0))
    assert bump.BUMP_SHRUNK not in res.message
    assert len(case['calls']) == 1
    plain = bump.apply_catch_bump(case['seg'].plan, case['seg'].meta, -1, (5.0, 0.0),
                                  case['t_c'], case['lim'], case['geom'])
    assert np.array_equal(res.plan.pose, plain.plan.pose)
    assert np.array_equal(res.plan.pose_vel, plain.plan.pose_vel)


def test_a_bump_the_next_catch_refuses_is_shrunk_to_the_largest_passing_step(case):
    gate = case['gate_at'](GATE_HAND_RPS2)
    # The gap, on the pre-I-CATCH-7 path: apply_catch_bump accepts the 20 mm bump
    # (its own gate cannot see the successor) and the seed it leaves refuses.
    plain = bump.apply_catch_bump(case['seg'].plan, case['seg'].meta, -1, (20.0, 0.0),
                                  case['t_c'], case['lim'], case['geom'])
    assert plain.accepted, plain.message
    why = gate(plain)
    assert why is not None and why.startswith('HAND_LIMIT_ACC'), why

    res = _bump(case, (20.0, 0.0), gate)
    assert res.accepted, res.message
    assert res.shrink_steps == 1
    assert np.allclose(res.delta_xy_mm, (10.0, 0.0))
    assert np.allclose(res.requested_delta_xy_mm, (20.0, 0.0))
    assert [tuple(c) for c in case['calls']] == [(20.0, 0.0), (10.0, 0.0)]
    assert res.message.startswith(bump.BUMP_SHRUNK + ' step 1: requested (20.0, 0.0)'
                                  ' -> applied (10.0, 0.0) mm; next catch refused '
                                  '(20.0, 0.0) HAND_LIMIT_ACC'), res.message
    # The applied step is a whole bump of the smaller delta, never an edited seed.
    ten = bump.apply_catch_bump(case['seg'].plan, case['seg'].meta, -1, (10.0, 0.0),
                                case['t_c'], case['lim'], case['geom'])
    assert np.array_equal(res.plan.pose, ten.plan.pose)
    assert np.array_equal(res.release_offset_xy_mm, ten.release_offset_xy_mm)
    k = res.carried_release_k
    assert np.array_equal(res.meta.releases[0].brake_accel_mm_s2
                          if res.meta.releases[0].brake_accel_mm_s2 is not None
                          else np.zeros(3),
                          ten.meta.releases[0].brake_accel_mm_s2
                          if ten.meta.releases[0].brake_accel_mm_s2 is not None
                          else np.zeros(3))
    assert gate(res) is None and k == ten.carried_release_k


def test_a_bump_with_no_passing_step_is_declined_by_name(case):
    # 2000 rev/s^2: below every successor of the ladder (2225 at 2.5 mm).
    res = _bump(case, (20.0, 0.0), case['gate_at'](2000.0))
    assert not res.accepted
    assert res.code == bump.BUMP_NEXT_CATCH and not res.terminal
    assert np.allclose(res.delta_xy_mm, (20.0, 0.0))
    assert [tuple(c) for c in case['calls']] == [(20.0, 0.0), (10.0, 0.0),
                                                (5.0, 0.0), (2.5, 0.0)]
    assert res.message.count(') HAND_LIMIT_ACC') == 4, res.message
    assert res.plan is None


def test_without_a_gate_the_bump_is_i_catch_5_unchanged(case):
    seg = case['seg']
    # Dispatch 3.5 knots before t0: k_wire = -1, the other tests' ramp start.
    t_disp = -3.5 * float(seg.plan.dt)
    res, k_wire = bump.bump_record(seg.plan, seg.meta, 0.0, t_disp, (20.0, 0.0),
                                   case['t_c'], case['lim'], case['geom'])
    assert k_wire == -1
    plain = bump.apply_catch_bump(seg.plan, seg.meta, -1, (20.0, 0.0), case['t_c'],
                                  case['lim'], case['geom'])
    assert res.accepted and res.shrink_steps == 0 and res.requested_delta_xy_mm is None
    assert np.array_equal(res.plan.pose, plain.plan.pose)
