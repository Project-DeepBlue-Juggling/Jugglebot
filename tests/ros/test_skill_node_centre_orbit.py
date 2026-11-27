"""JP-14 ``centre_orbit`` through the node: its label, its parameter, the
``skills/nominate`` key and the out-of-range refusal (the compile is
``tests/motion/test_skills_centre_orbit.py``; the goal-accept path is covered
by ``test_every_pattern_entry_opens_on_a_rest``, which parametrises over every
cell of ``_CUP_RULE``)."""
from __future__ import annotations

from jugglebot import juggle_request as jr
from jugglebot import records as rec
from jugglebot import skill_node as sn
from jugglebot.motion.skills import sites as st
from tests.ros.test_skill_node_nominate import _nominate_req, _ready_node


def test_centre_orbit_has_its_own_record_label_and_is_not_a_feed():
    assert ('centre_orbit', 'Centre and orbit', 2) in rec.PATTERNS
    assert 'centre_orbit' in sn.SkillNode._PATTERNS
    assert 'centre_orbit' not in rec.FEED_PATTERNS
    assert sn._CUP_RULE['centre_orbit'] == sn._CUP_RULE['columns']


def test_orbit_n_is_a_nominate_only_key_and_a_node_parameter():
    assert jr.NOMINATE_EXTRA_FIELDS['orbit_n'] is int
    pattern, vals = jr.parse_juggle_request(
        'centre_orbit,separation_mm=100,orbit_n=6',
        extra_keys=jr.NOMINATE_EXTRA_FIELDS)
    assert pattern == 'centre_orbit' and vals['orbit_n'] == 6
    node, _ = _ready_node('orbit_param')
    cfg = node._nomination_config('centre_orbit', 0.95, 100.0, 0.27)
    assert cfg.orbit_n == 8 and cfg.entry == 'hop'
    node._params['orbit_n'] = 12
    assert node._nomination_config('centre_orbit', 0.95, 100.0,
                                   0.27).orbit_n == 12


def test_an_out_of_range_radius_is_refused_by_name_not_as_workspace():
    node, _ = _ready_node('orbit_range')
    refused = _nominate_req(node, 'centre_orbit,apex_m=0.95,'
                                  'separation_mm=120,dwell_s=0.27,orbit_n=8')
    assert refused.success is False
    assert st.ORBIT_RADIUS_OUT_OF_RANGE in refused.message
    assert 'REJECTED_SITE_OUTSIDE_WORKSPACE' not in refused.message
    refused = _nominate_req(node, 'centre_orbit,apex_m=0.95,'
                                  'separation_mm=100,dwell_s=0.27,orbit_n=4')
    assert refused.success is False
    assert st.ORBIT_N_OUT_OF_RANGE in refused.message


import pytest  # noqa: E402


@pytest.mark.parametrize('r,n,named', [
    (90, 8, st.ORBIT_RADIUS_OUT_OF_RANGE), (110, 8, st.ORBIT_RADIUS_OUT_OF_RANGE),
    (100, 5, st.ORBIT_N_OUT_OF_RANGE), (100, 13, st.ORBIT_N_OUT_OF_RANGE)])
def test_just_outside_the_range_is_refused_by_name_never_clamped(r, n, named):
    node, _ = _ready_node('orbit_edge_%d_%d' % (r, n))
    refused = _nominate_req(node, 'centre_orbit,apex_m=0.95,separation_mm=%d,'
                                  'dwell_s=0.27,orbit_n=%d' % (r, n))
    assert refused.success is False and named in refused.message
    assert 'nearest viable' not in refused.message   # not clamped / re-aimed


def test_an_odd_goal_count_is_refused_by_name_before_anything_moves():
    node, _ = _ready_node('orbit_odd')
    res = node._run_columns(0.95, 100.0, 0.27, 17, orbit_n=8)
    assert res.success is False and st.ORBIT_ODD_THROW_COUNT in res.message
