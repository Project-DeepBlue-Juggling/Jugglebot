"""JP-11 rotating Columns, the nomination wiring (build unit U2): the config
field, the full-turn span rule, the builder's refusals by name and the
request grammar.  The compile itself is ``test_skills_columns_rotating.py``;
the node is ``tests/ros/test_skill_node_columns_rotating.py``.

    pytest tests/motion/test_skills_columns_rotating_wiring.py -q -p no:cacheprovider
"""
from __future__ import annotations

import pytest

from jugglebot import juggle_request as jr
from jugglebot import records as rec
from jugglebot.motion.skills import nominate as nm
from jugglebot.motion.skills import schedule as sc
from jugglebot.motion.skills import sites as st


def _cfg(step=10.0, **kw):
    kw.setdefault('entry', 'hop')
    return nm.NominationConfig('columns_rotating', 0.95, 125.0, 0.27,
                               step_deg=step, **kw)


def test_the_pattern_is_registered_everywhere_a_nomination_looks():
    assert 'columns_rotating' in nm.NOMINATED_PATTERNS
    assert 'columns_rotating' in nm.COLUMNS_FED
    assert ('columns_rotating', 'Columns — rotating', 2) in rec.PATTERNS
    assert 'columns_rotating' not in rec.FEED_PATTERNS


@pytest.mark.parametrize('step,span', [(10.0, 38), (5.0, 74), (20.0, 20),
                                       (7.0, 54), (0.5, 722)])
def test_span_is_a_full_turn_plus_two(step, span):
    assert nm.rotating_span(step) == span


def test_the_short_nomination_is_one_full_turn_and_an_operator_count_caps_it():
    assert nm.build_pattern(_cfg(10.0)).n_throws == 38
    assert nm.build_pattern(_cfg(5.0)).n_throws == 74
    assert nm.build_pattern(_cfg(10.0, n_throws=12)).n_throws == 12
    assert nm.build_pattern(_cfg(10.0, n_throws=500)).n_throws == 38
    # the Attempt's own build (n_throws passed) is never capped
    assert nm.build_pattern(_cfg(10.0), 60).n_throws == 60


def test_the_builder_makes_columns_fed_pair_with_the_step_and_feed_at_p1():
    pat = nm.build_pattern(_cfg(10.0, feed_site='P2'))   # P2 is NOT honoured
    assert pat.step_deg == 10.0
    assert [s.name for s in pat.sites] == ['P2', 'P1']
    assert pat.apex_m_ball_b is None and not pat.phantom_balls
    sched = nm.compile_schedule(_cfg(10.0, n_throws=6))
    assert sched.pattern == sc.COLUMNS_ROTATING


@pytest.mark.parametrize('step', [0.0, -5.0, 20.5, float('nan')])
def test_a_step_outside_zero_to_twenty_is_refused_by_name(step):
    with pytest.raises(ValueError, match=st.ROTATING_STEP_OUT_OF_RANGE):
        nm.build_pattern(_cfg(step))


def test_zero_step_is_never_read_as_plain_columns():
    # the field default (0.0) on this pattern must refuse, not fly Columns
    with pytest.raises(ValueError, match=st.ROTATING_STEP_OUT_OF_RANGE):
        nm.build_pattern(nm.NominationConfig('columns_rotating', 0.95, 125.0,
                                             0.27, entry='hop'))


def test_an_unequal_apex_pair_is_refused_by_name():
    with pytest.raises(ValueError, match=st.ROTATING_APEX_PAIR_UNEQUAL):
        nm.build_pattern(_cfg(10.0, apex_m_ball2=0.9))
    assert nm.build_pattern(_cfg(10.0, apex_m_ball2=0.95)).n_throws == 38


def test_a_t0_entry_refuses_by_name_as_a_compile_refusal():
    with pytest.raises(ValueError, match=st.ROTATING_NEEDS_HOP_ENTRY):
        nm.compile_schedule(_cfg(10.0, entry='t0', n_throws=6))


def test_step_deg_is_a_nominate_only_key():
    assert jr.NOMINATE_EXTRA_FIELDS['step_deg'] is float
    assert 'step_deg' not in jr.JUGGLE_NUMERIC_FIELDS
    pattern, vals = jr.parse_juggle_request(
        'columns_rotating,apex_m=0.95,separation_mm=125,step_deg=10',
        extra_keys=jr.NOMINATE_EXTRA_FIELDS)
    assert pattern == 'columns_rotating' and vals['step_deg'] == 10.0
    with pytest.raises(ValueError):          # the goal relay refuses it
        jr.parse_juggle_request('columns_rotating,step_deg=10')
