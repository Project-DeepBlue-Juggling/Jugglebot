"""BUG-146: the Hand's tilt-drop term carries clamped cubic-spline knot rates.

``cup_realize.decompose`` raises the slider by ``drop = arm * (1 - axis_z)`` so a
tilted cup still reaches its planned height.  The drop's knot POSITIONS are
exact; its knot RATES used to be ``vz * (1 - axis_z) - arm * axis_dot_z`` with
``axis_dot`` a second-order central difference.  ``validate_cycle`` Pass 5 (and
the firmware's torque feedforward) rebuild each span as a Hermite cubic from
(position, rate), so a rate error dv became a knot-acceleration step of about
320 * dv: ~0.116 * theta^2 rev/s^2 (theta in deg) in the SETTLE tail, which
refused every aimed THROW from a level rest past ~6.2 deg of release tilt
``HAND_LIMIT_C2`` (Apex 1.8 m, 450 mm, window 0.6 s: 5.384 > 4.5).

The fix (owner, TKT-98 option 1): the drop's interior knot rates are the
clamped cubic spline through its own positions, the two end rates pinned to
the old formula's values, so every seam's Hand rate and every Hand position is
unchanged.  Measured 2026-11-28 (``/tmp/probe_bug146b.py``, Apex 1.8 m, 0 deg
direction, window 0.6 s): each planned window's interior Hand C2 jump falls
from 1.04 / 4.17 / 9.34 rev/s^2 at 3 / 6 / 9 deg to below 1e-6 (float noise);
what is left on the JOINED plan is the LAUNCH -> SETTLE extend seam at the
release knot + 1, 0.21 / 0.87 / 2.29 rev/s^2, which is 80 x the pre-existing
mismatch between the two windows' one-sided end rates at the release knot
(the clamps keep it, by design).  The joined bound below is 0.04 * theta^2,
about a third of the pre-fix law and 1.07-1.65x the measured values.
"""
from __future__ import annotations

import dataclasses
import math

import numpy as np
import pytest

from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.skills import nominate as nom
from jugglebot.motion.skills import segments as sg
from jugglebot.motion.skills import sites as st
from jugglebot.motion.trajectory import cup_realize as cr
from jugglebot.motion.trajectory import tilt_geometry as tg

G = 9.81
DT = 0.025

#: Below this an interior Hand knot-acceleration jump is float noise (rev/s^2).
#: Measured < 1e-9 on every window of the cases below; the gate's tolerance is
#: 4.5 (1e-3 x the Sitting Hand limit).
WINDOW_C2_NOISE = 1e-6
#: Joined-plan bound coefficient (rev/s^2 per deg^2); see the module docstring.
JOINED_C2_PER_DEG2 = 0.04


def _hermite_jump(p, v, h=DT):
    """``a_right - a_left`` at interior knots 1..n-2 (validate_cycle Pass 5)."""
    p = np.asarray(p, dtype=float)
    v = np.asarray(v, dtype=float)
    return (6.0 * (p[2:] - p[:-2]) / h ** 2
            - (2.0 * v[:-2] + 8.0 * v[1:-1] + 2.0 * v[2:]) / h)


def _sitting_limits():
    """The PWAY-11 P4 Sitting limits the Item was found at (leg 600 / 8000 /
    200000, Hand 4500 rev/s^2)."""
    return nom.sitting_limits(600.0, 8000.0, 200000.0, 4500.0)


def _throw(apex_abs_m, d_mm, dir_deg, window_s):
    """An aimed THROW from a level REST at the columns Site W (BUG-146's case)."""
    site = st.Site('W', np.array([-50.0, 0.0, st.CATCH_CUP_Z_MM]))
    seed = nom.rest_state(site.rest_site_mm())
    rel = site.throw_site_mm()
    vz = math.sqrt(2.0 * G * (apex_abs_m - rel[2] / 1000.0))
    flight = 2.0 * vz / G
    c, s = math.cos(math.radians(dir_deg)), math.sin(math.radians(dir_deg))
    target = rel + np.array([d_mm * c, d_mm * s, 0.0])
    term = sg.ThrowTerminal(site_mm=rel, target_mm=target, flight_s=flight,
                            t_release_s=window_s)
    tilt_deg = math.degrees(math.atan2(d_mm / 1000.0 / flight, vz))
    return seed, term, tilt_deg


def _d_for_tilt(apex_abs_m, tilt_deg):
    vz = math.sqrt(2.0 * G * (apex_abs_m - st.RELEASE_CUP_Z_MM / 1000.0))
    return 1000.0 * (2.0 * vz / G) * vz * math.tan(math.radians(tilt_deg))


def _plan(monkeypatch, apex_abs_m, d_mm, dir_deg=0.0, window_s=0.6):
    """Plan the THROW, recording every window ``decompose`` realises."""
    windows = []
    real = cr.decompose

    def spy(cup, tilts, cfg=None):
        r = real(cup, tilts, cfg)
        windows.append(r)
        return r
    monkeypatch.setattr(cr, 'decompose', spy)
    seed, term, tilt_deg = _throw(apex_abs_m, d_mm, dir_deg, window_s)
    cfg = sg.SegmentConfig(planner_mode='pose')
    seg = sg.plan_segment(sg.THROW, seed, term, cfg, _sitting_limits(),
                          StewartGeometry())
    return seg, windows, tilt_deg


def _joined_peak(plan):
    jump = _hermite_jump(plan.hand_rev, plan.hand_vel_rps)
    for k in (getattr(plan, 'release_step_knots', None) or ()):
        jump[k - 1] = 0.0          # C-HAND-5: the declared release step
    return float(np.max(np.abs(jump)))


def test_the_bug146_throw_at_6_8_deg_is_accepted(monkeypatch):
    """Apex 1.8 m, 450 mm reach (6.82 deg), window 0.6 s: refused
    ``HAND_LIMIT_C2`` 5.384 > 4.5 before the fix, accepted after."""
    seg, windows, tilt_deg = _plan(monkeypatch, 1.8, 450.0)
    assert tilt_deg == pytest.approx(6.82, abs=0.01)
    assert seg.plan.n_knots > 0
    assert _joined_peak(seg.plan) < 4.5
    assert max(float(np.max(np.abs(_hermite_jump(w.slider_rev,
                                                 w.slider_vel_rev_s))))
               for w in windows if w.pose.shape[0] >= 3) < WINDOW_C2_NOISE


@pytest.mark.parametrize('tilt_deg', [3.0, 6.0, 9.0])
def test_the_hand_knot_acceleration_is_continuous_at_every_tilt(monkeypatch,
                                                                tilt_deg):
    """Every realised window is C2 in the Hand to float noise; the joined plan
    keeps only the release-seam residual, under 0.04 * theta^2."""
    seg, windows, got = _plan(monkeypatch, 1.8, _d_for_tilt(1.8, tilt_deg))
    assert got == pytest.approx(tilt_deg, abs=0.01)
    assert len(windows) >= 2
    for w in windows:
        if w.pose.shape[0] < 3:
            continue
        jump = _hermite_jump(w.slider_rev, w.slider_vel_rev_s)
        assert float(np.max(np.abs(jump))) < WINDOW_C2_NOISE
    assert _joined_peak(seg.plan) < JOINED_C2_PER_DEG2 * tilt_deg ** 2


# -- decompose-level pins (synthetic plan) -----------------------------------

@dataclasses.dataclass
class _CupPlan:
    pos: np.ndarray
    vel: np.ndarray
    dt: float
    catch_k: int = -1
    t: np.ndarray = None


def _synthetic(n=31, slew_deg=8.0):
    """A cup riding the stroke (sinusoidal z) under a quintic tilt slew."""
    t = np.arange(n) * DT
    w = 2.0 * math.pi / (t[-1] * 1.3)
    z = 0.80 + 0.06 * np.sin(w * t)
    vz = 0.06 * w * np.cos(w * t)
    pos = np.column_stack([np.zeros(n), np.zeros(n), z])
    vel = np.column_stack([np.zeros(n), np.zeros(n), vz])
    u = t / t[-1]
    s = 10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5
    tilts = np.column_stack([np.zeros(n), math.radians(slew_deg) * s])
    return _CupPlan(pos=pos, vel=vel, dt=DT, t=t), tilts


def _reference(cup, tilts, cfg):
    """The PRE-FIX slider positions and rates, statement for statement."""
    n = cup.pos.shape[0]
    axis = np.zeros((n, 3))
    arm = np.zeros(n)
    slider_mm = np.zeros(n)
    for k in range(n):
        cup_z_mm = float(cup.pos[k, 2]) * 1000.0
        a = tg.cup_axis(float(tilts[k, 0]), float(tilts[k, 1]))
        arm_k = tg.cup_lever_arm_mm(cup_z_mm)
        drop_k = arm_k * (1.0 - float(a[2]))
        s = cup_z_mm - cfg.cup_z_base_mm + drop_k
        slider_mm[k] = min(max(s, 0.0), float(cfg.slider_stroke_mm))
        axis[k] = a
        arm[k] = arm_k
    vz_mm = cup.vel[:, 2] * 1000.0
    axis_dot = cr._knot_derivative(axis, cup.dt)
    drop_dot = vz_mm * (1.0 - axis[:, 2]) - arm * axis_dot[:, 2]
    rev = ((slider_mm - float(cfg.slider_rev_zero_mm)) / 1000.0
           * cr.HAND_REV_PER_M)
    rate = (vz_mm + drop_dot) / 1000.0 * cr.HAND_REV_PER_M
    return rev, rate


def test_the_seam_rates_and_every_position_are_unchanged():
    """Knot 0 (a splice/extend seed, I-PLAN-11) and the last knot keep the
    pre-fix Hand rate to 1e-12; every Hand position is bitwise unchanged; the
    interior satisfies the spline relation (fails pre-fix: ~1e1 rev/s^2)."""
    cup, tilts = _synthetic()
    cfg = cr.RealizeConfig()
    out = cr.decompose(cup, tilts, cfg)
    assert not out.slider_saturated.any()
    ref_rev, ref_rate = _reference(cup, tilts, cfg)
    assert np.array_equal(out.slider_rev, ref_rev)
    assert abs(out.slider_vel_rev_s[0] - ref_rate[0]) < 1e-12
    assert abs(out.slider_vel_rev_s[-1] - ref_rate[-1]) < 1e-12
    # The DROP term's own rates satisfy the spline relation at every interior
    # knot (the synthetic cup is not itself spline-consistent, so test the
    # drop channel: slider minus the cup's exact part).
    k_rev = cr.HAND_REV_PER_M / 1000.0
    cup_rev = cup.pos[:, 2] * 1000.0 * k_rev
    cup_rate = cup.vel[:, 2] * 1000.0 * k_rev
    drop_jump = _hermite_jump(out.slider_rev - cup_rev,
                              out.slider_vel_rev_s - cup_rate)
    assert float(np.max(np.abs(drop_jump))) < WINDOW_C2_NOISE
    # ... and the move is small: the rates shift by far less than they are.
    assert float(np.max(np.abs(out.slider_vel_rev_s - ref_rate))) < 0.05


def test_a_saturated_knot_still_reports_zero_hand_rate():
    """The stroke clamp's zero rate still wins over the spline."""
    cup, tilts = _synthetic()
    cfg = dataclasses.replace(cr.RealizeConfig(), slider_stroke_mm=60.0)
    out = cr.decompose(cup, tilts, cfg)
    assert out.slider_saturated.any()
    assert np.all(out.slider_vel_rev_s[out.slider_saturated] == 0.0)


def test_a_level_plan_keeps_todays_hand_rates_exactly(monkeypatch):
    """Zero tilt: the drop is identically 0, the spline is never solved, and
    the Hand rates are the pre-fix ones element for element."""
    cup, tilts = _synthetic(slew_deg=0.0)
    cfg = cr.RealizeConfig()

    def boom(*a, **k):
        raise AssertionError('a level plan must not solve the drop spline')
    monkeypatch.setattr(cr, '_clamped_spline_rates', boom, raising=False)
    out = cr.decompose(cup, tilts, cfg)
    ref_rev, ref_rate = _reference(cup, tilts, cfg)
    assert np.array_equal(out.slider_rev, ref_rev)
    assert np.array_equal(out.slider_vel_rev_s, ref_rate)
