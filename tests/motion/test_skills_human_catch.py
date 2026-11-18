"""``schedule.compile_human_catch`` -- the per-Throw Schedule of the human_catch
Pattern (PWAY-11 P2, U4; design ``design-human-catch.md`` section 2.1).

Unmarked and parallel-safe except the two ``check_segment`` tests, which solve
one real Segment each (about 0.1 s idle).  The admissibility check is the
executor's own ``check_segment`` at the Sitting limits, driven by the reach
probe's ``check_reach`` (``tools/probes/human_catch_reach.py``) so the test and
the sweep cannot drift apart.
"""

from __future__ import annotations

import importlib.util
import os

import numpy as np
import pytest

import jugglebot.hardware_config as hw
from jugglebot.motion.skills import schedule as sc
from jugglebot.motion.skills import sites as st

_PROBE = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..',
                      'tools', 'probes', 'human_catch_reach.py')

LOOKAHEAD_S = 0.05          # skill_node._DISPATCH_LOOKAHEAD_S (1/40 + one knot)
ARRIVAL_VEL = np.array([0.0, 0.0, -4580.0])


def _probe():
    spec = importlib.util.spec_from_file_location('human_catch_reach', _PROBE)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _landing(x=-50.0, y=0.0):
    return np.array([x, y, st.CATCH_CUP_Z_MM])


def _compile(lead_s=0.70, t_now=100.0, landing=None, vel=ARRIVAL_VEL):
    landing = _landing() if landing is None else landing
    return sc.compile_human_catch(landing, vel, t_now + lead_s, t_now,
                                  LOOKAHEAD_S)


def test_the_schedule_is_one_level_catch_labelled_human_catch():
    s = _compile()
    assert s.pattern == 'human_catch'
    assert len(s.skills) == 1
    sk = s.skills[0]
    assert sk.kind == sc.CATCH
    assert sk.then_throw is None            # a standalone LANDING: rest-terminal
    assert sk.hold_tilt is None
    assert sk.receive_tilt == (0.0, 0.0)    # level pinned
    assert sk.ball_id == 0
    assert s.phantom_balls == ()
    assert s.t0_abs_s == 100.0


def test_window_arithmetic_dispatches_one_lookahead_after_now():
    """window = t_land - (t_now + lookahead) - LEAD_S; the executor's dispatch
    instant (t_abs - window - lead) is therefore t_now + lookahead."""
    s = _compile(lead_s=0.70)
    sk = s.skills[0]
    assert sk.window_s == pytest.approx(0.70 - LOOKAHEAD_S - sc.LEAD_S)
    assert sk.t_abs_s == pytest.approx(100.70)
    assert sk.dispatch_s() == pytest.approx(100.0 + LOOKAHEAD_S)
    assert sk.landing_prior.t_land_abs_s == 100.70


def test_a_window_below_the_floor_is_refused_by_check_window():
    # the shortest lead that still leaves MIN_WINDOW_S
    lead_ok = LOOKAHEAD_S + sc.LEAD_S + sc.MIN_WINDOW_S
    _compile(lead_s=lead_ok + 1e-6)
    with pytest.raises(ValueError, match='floor'):
        _compile(lead_s=lead_ok - 0.01)


def test_the_catch_xy_is_the_landing_xy_bit_for_bit():
    """The Pattern prior never enters the aim: the compiled CATCH's Site, its
    rest terminal and its landing prior are the event's Landing xy exactly."""
    x, y = -50.0 + 123.456789012345, 87.654321098765
    s = _compile(landing=_landing(x, y))
    sk = s.skills[0]
    assert sk.landing_prior.pos_mm[0] == x and sk.landing_prior.pos_mm[1] == y
    assert sk.site.cup_mm[0] == x and sk.site.cup_mm[1] == y
    rest = sk.site.rest_site_mm()
    assert rest[0] == x and rest[1] == y


def test_the_landing_prior_carries_the_event_velocity_and_time():
    v = np.array([120.0, -80.0, -4400.0])
    s = _compile(vel=v)
    np.testing.assert_array_equal(s.skills[0].landing_prior.vel_mm_s, v)


@pytest.mark.parametrize('bad', [np.nan, np.inf])
def test_a_non_finite_landing_time_is_refused(bad):
    with pytest.raises(ValueError):
        sc.compile_human_catch(_landing(), ARRIVAL_VEL, bad, 100.0,
                               LOOKAHEAD_S)


def test_a_landing_before_now_is_refused():
    with pytest.raises(ValueError):
        sc.compile_human_catch(_landing(), ARRIVAL_VEL, 99.0, 100.0,
                               LOOKAHEAD_S)


# -- the admissibility check (real planner) --------------------------------

def test_a_landing_at_the_wait_site_passes_the_check():
    hc = _probe()
    r = hc.check_reach(0.0, 0.0, 0.70)
    assert r.ok, (r.code, r.margin)
    assert r.margin >= 0.0


def test_a_landing_far_outside_reach_is_refused_with_its_code():
    """300 mm in a 0.45 s motion: refused inside the solve; nothing to send."""
    hc = _probe()
    r = hc.check_reach(300.0, 0.0, 0.70)
    assert not r.ok
    assert r.code != 'OK'
    # the refusal names the bound (I-PLAN-1/2) or is a gate margin
    assert r.code in ('LIMIT_VEL', 'LIMIT_ACC', 'LIMIT_JERK', 'MARGIN',
                      'WORKSPACE', 'UNREACHABLE', 'CATCH_CLOSING_SPEED',
                      'CATCH_AXIS') or r.code.isupper()
