"""JP-5 ``polygon_walk`` through ``skills/nominate`` and the node's parameters
(the compile and the margins are ``tests/motion/test_skills_polygon_walk.py``;
the goal-accept path is covered by ``test_every_pattern_entry_opens_on_a_rest``,
which parametrises over every cell of ``_CUP_RULE``)."""
from __future__ import annotations

from jugglebot import juggle_request as jr
from jugglebot import records as rec
from jugglebot import skill_node as sn
from tests.ros.test_skill_node_nominate import _nominate_req, _ready_node


def test_the_service_nominates_a_square_and_reads_polygon_n():
    node, _ = _ready_node('polygon')
    ok = _nominate_req(node, 'polygon_walk,apex_m=0.9,separation_mm=100,'
                             'dwell_s=0.27,polygon_n=4')
    assert ok.success is True, ok.message
    assert 'polygon_walk' in ok.message and 'separation 100 mm' in ok.message
    refused = _nominate_req(node, 'polygon_walk,apex_m=0.9,separation_mm=250,'
                                  'dwell_s=0.27')
    # 250 mm: the node's existing workspace check refuses first (circumradius
    # 177 mm > the 150 mm cup-cycle workspace); 200 mm (141 mm out) reaches
    # the nomination and is refused there (probe 2026-11-18: LIMIT_JERK)
    assert refused.success is False
    assert 'REJECTED_SITE_OUTSIDE_WORKSPACE' in refused.message
    refused = _nominate_req(node, 'polygon_walk,apex_m=0.9,separation_mm=200,'
                                  'dwell_s=0.27')
    assert refused.success is False
    assert 'REJECTED_SITE_OUTSIDE_WORKSPACE' not in refused.message


def test_polygon_n_is_a_nominate_only_key_and_a_node_parameter():
    assert jr.NOMINATE_EXTRA_FIELDS['polygon_n'] is int
    pattern, vals = jr.parse_juggle_request(
        'polygon_walk,separation_mm=100,polygon_n=5',
        extra_keys=jr.NOMINATE_EXTRA_FIELDS)
    assert pattern == 'polygon_walk' and vals['polygon_n'] == 5
    node, _ = _ready_node('polygon_param')
    assert node._nomination_config('polygon_walk', 0.9, 100.0, 0.27).polygon_n == 4
    node._params['polygon_n'] = 5
    assert node._nomination_config('polygon_walk', 0.9, 100.0, 0.27).polygon_n == 5


def test_polygon_walk_has_its_own_record_label_and_is_not_a_feed():
    assert ('polygon_walk', 'Polygon walk', 1) in rec.PATTERNS
    assert 'polygon_walk' in sn.SkillNode._PATTERNS
    assert 'polygon_walk' not in rec.FEED_PATTERNS
