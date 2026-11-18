"""``human_catch`` at the node: nomination at accept, the records ledger, and
how a human Catch is BOOKED (PWAY-11 P2, U5b).

* I-ADMIT-2: the goal is nominated at accept like every other Pattern (the
  skip the node lifecycle, U5a, put there is gone); a refusal rejects the goal
  and nothing moves.
* The ledger: ``human_catch`` has a ``records.PATTERNS`` row; its row counts
  human Throws planned for and Catches confirmed seated, and books NO drops.
* A human Catch is its own thing: the executor carries ``landing_prior``, but a
  caught Ball is not a ``feed`` line and a miss is not a Feed miss.
* Memory: a full Attempt, caught and missed Throws, writes no row and offers
  the Learner no experience.

Helpers are the lifecycle file's (``test_skill_node_human_catch.py``).
"""
from __future__ import annotations

from unittest.mock import MagicMock, patch

import pytest

from jugglebot import records as rec
from jugglebot import skill_node as sn
from jugglebot.motion.skills import executor as ex
from jugglebot.motion.skills import human_catch as hcp
from jugglebot.motion.skills import nominate as nom
from tests.ros.test_skill_node import _goal
from tests.ros.test_skill_node_human_catch import (
    _EMPTY, _SEATED, _catch_land, _catches, _ctx, _event, _fill_cup, _hc_node,
    _phase, _plan_one, _rest_to_rest, _start, _tick, _tick_until, _texts,
    _lines)


# -- I-ADMIT-2: nominated at accept ------------------------------------------

def test_the_nominator_and_the_node_share_the_dispatch_look_ahead():
    assert nom.HUMAN_CATCH_DISPATCH_LEAD_S == pytest.approx(
        sn._DISPATCH_LOOKAHEAD_S)


def test_the_goal_is_nominated_at_accept_with_the_node_parameters():
    node, client, clock, _cup = _hc_node()
    node._params['human_catch_home_x_mm'] = -40.0
    node._params['human_catch_nominal_apex_m'] = 2.0
    node._params['human_catch_first_lead_s'] = 0.58
    seen = []
    real = nom.nominate

    def spy(config, *a, **kw):
        seen.append(config)
        return real(config, *a, **kw)
    with patch.object(sn.nom, 'nominate', spy):
        resp = node._start_pattern(_goal('human_catch'))
    assert resp.success is True, resp.message
    assert [c.pattern for c in seen] == ['human_catch']
    c = seen[0]
    assert tuple(c.hc_home_xy_mm) == (-40.0, 0.0)
    assert c.hc_nominal_apex_m == 2.0 and c.hc_first_lead_s == 0.58
    # the opening REST goes to the nomination's wait Site
    assert node._executor.schedule.skills[0].site.cup_mm[0] == pytest.approx(-40.0)
    assert len(client.calls) == 0          # nothing is sent before the tick


def test_a_refused_nomination_rejects_the_goal_and_moves_nothing():
    node, client, clock, _cup = _hc_node()
    refused = SimpleNamespaceRefusal()
    with patch.object(sn.SkillNode, '_nominate', return_value=refused):
        resp = node._start_pattern(_goal('human_catch'))
    assert resp.success is False and 'nomination REFUSED' in resp.message
    assert node._executor is None and node._reload_ctx is None
    assert client.calls == []


def SimpleNamespaceRefusal():
    from types import SimpleNamespace
    return SimpleNamespace(ok=False, lines=['CATCH: LIMIT_VEL'], seconds=0.0,
                           summary='nomination REFUSED: human_catch ...',
                           report=None)


def test_a_new_pattern_name_is_nominated_by_the_service_too():
    node, _client, _clock, _cup = _hc_node()
    resp = MagicMock()
    req = MagicMock()
    req.data = 'human_catch'
    out = node._svc_nominate(req, resp)
    assert out.success is True, out.message
    assert 'human_catch' in out.message and 'PROPOSED D9' in out.message


# -- the ledger --------------------------------------------------------------

def test_the_records_ledger_has_a_human_catch_row_and_books_no_drops(tmp_path):
    assert 'human_catch' in {p for p, _l, _b in rec.PATTERNS}
    node, _client, _clock, _cup = _hc_node()
    path = tmp_path / 'records.md'
    node._params['records_path'] = str(path)
    beaten, best = node._offer_record('human_catch', 5, 4, 'COMPLETED')
    assert beaten is True and best == '4'
    led = rec.load(str(path))
    [row] = led.table('human_catch').rows
    assert (row.throws, row.catches, row.end_code) == (5, 4, 'COMPLETED')
    assert led.drops is None or led.drops.live == 0


# -- how a human Catch is booked --------------------------------------------

def _book_two_throws(node, client, clock, cup):
    """Throw 1 caught and Throw 2 missed, both by the real cup verdict."""
    _start(node, clock)
    t_plan = clock.t
    _plan_one(node, clock, client, x=-50.0)
    L = _catch_land(node)
    _fill_cup(node, t_plan - 0.05, L + 3.0, seated_from=L + 0.02)
    cup[0] = _SEATED
    t_dec = (L + ex.MISSED_CATCH_CLOSE_AFTER_LANDING_S
             + ex.MISSED_CATCH_DECIDE_LAG_S)
    _tick_until(node, clock, t_dec + 0.05)
    assert _phase(node) == hcp.HOLDING
    cup[0] = _EMPTY
    _tick(node, clock, clock.t + 0.025)
    _tick(node, clock, clock.t + 0.6)
    assert _phase(node) == hcp.RETURNING
    _rest_to_rest(node, clock)
    assert _phase(node) == hcp.WAITING
    t_plan2 = clock.t
    _plan_one(node, clock, client, x=-40.0, track=8)
    L2 = _catch_land(node)
    _fill_cup(node, t_plan2 - 0.05, L2 + 3.0)               # never SEATED
    t_dec2 = (L2 + ex.MISSED_CATCH_CLOSE_AFTER_LANDING_S
              + ex.MISSED_CATCH_DECIDE_LAG_S)
    _tick_until(node, clock,
                t_dec2 + ex.MISSED_CATCH_FAULT_SETTLE_S + 0.06)
    assert _phase(node) == hcp.RETURNING


def test_a_caught_and_a_missed_human_ball_book_as_human_not_as_feeds(tmp_path):
    node, client, clock, cup = _hc_node()
    log = _lines(node)
    path = tmp_path / 'records.md'
    node._params['records_path'] = str(path)
    _book_two_throws(node, client, clock, cup)
    assert (node._goal_human_claimed, node._goal_human_caught,
            node._goal_human_missed) == (2, 1, 1)
    assert node._goal_feed_missed == 0 and node._goal_feeds_caught == 0
    assert node._goal_missed_catches == 0 and node._goal_feed_scores == []
    texts = _texts(log)
    assert not any(t.startswith('feed') or ' feed ' in t for t in texts)
    assert any('HUMAN-CATCH-CAUGHT:' in t for t in texts)
    assert not any('not confirmed by the cup' in t for t in texts)
    # the end of the Attempt: the Result and the ledger carry the human tally
    node._stop_attempt()
    node._on_tick()
    gh = MagicMock(is_cancel_requested=False)
    gh.request.pattern = 'human_catch'
    result = node._juggle_execute(gh)
    assert (result.throws, result.caught) == (2, 1)
    assert result.outcome == 'STOPPED'
    assert result.per_throw[-1].startswith('human_catch result: 1 of 2 human '
                                           'Throws caught, 1 missed, 0 not '
                                           'confirmed')
    assert 'not counted as Feeds' in result.per_throw[-1]
    [row] = rec.load(str(path)).table('human_catch').rows
    assert (row.throws, row.catches) == (2, 1)
    assert rec.load(str(path)).drops is None or \
        rec.load(str(path)).drops.live == 0


# -- Memory ------------------------------------------------------------------

def test_a_full_attempt_writes_no_memory_row_and_offers_no_experience():
    node, client, clock, cup = _hc_node()
    seen = []
    real_build = sn.SkillNode._build_hc_executor

    def spy(self, sched, policy):
        e = real_build(self, sched, policy)
        seen.append(e)
        return e
    with patch.object(sn.SkillNode, '_make_learner') as make_learner, \
            patch.object(sn.SkillNode, '_open_memory') as open_memory, \
            patch.object(sn.SkillNode, '_build_hc_executor', spy):
        _book_two_throws(node, client, clock, cup)
        node._stop_attempt()
        node._on_tick()
    make_learner.assert_not_called()
    open_memory.assert_not_called()
    # a REST, the caught CATCH, a REST, the missed CATCH, a REST: none has a
    # Learner or an experience sink, and none produced an experience
    assert len(seen) >= 5
    for e in seen:
        assert e.learner is None and e.on_experience is None
        assert getattr(e, 'experiences', []) == []
    assert node._goal_reports == []             # no ThrowReport, no row


# -- a Feed is booked exactly as before (node side) -------------------------

def test_the_node_still_counts_a_feed_miss_for_a_feed_executor():
    """The retire capture adds `missed_feed_catches` as it always did; the
    human counters stay zero for it."""
    node, _client, _clock, _cup = _hc_node()
    node._start_pattern(_goal('human_catch'))
    e = node._executor
    e.missed_feed_catches = 2
    e.end_code = ex.MISSED_CATCH
    e.attempt_ended = True
    node._stop_attempt()
    node._on_tick()
    assert node._goal_feed_missed == 2
    assert node._goal_human_caught == 0 and node._goal_human_missed == 0
