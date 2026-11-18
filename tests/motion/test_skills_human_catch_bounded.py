"""PWAY-11 P2 audit findings 5 and 6 on the policy."""
from __future__ import annotations

from jugglebot.motion.skills import human_catch as hc
from tests.motion.test_skills_human_catch_policy import _evt, _waiting


def test_declines_stay_bounded_over_an_until_stop_attempt():
    p = _waiting()
    for i in range(1000):                       # each one far from the robot
        d = p.on_event(_evt(-50.0 + 450.0, 0.0, 10.0, tid=i), now_s=0.0)
        assert d.kind == hc.IGNORE
    assert len(p.declines) == 256
    assert p.declines[-1].key == (999, 0)       # the newest are kept


def test_finished_counts_the_nodes_compiled_plans_when_given():
    p = _waiting(n_throws=2)
    p.on_event(_evt(-50, 0, 0.8, tid=1), now_s=0.0)
    p.on_executor_done('LIMIT_JERK', moved=False)
    p.on_event(_evt(-50, 0, 0.8, tid=2), now_s=0.0)
    p.on_executor_done('LIMIT_JERK', moved=False)
    assert p.claimed == 2 and p.finished()      # default: the policy's claims
    assert not p.finished(1) and p.finished(2)  # the node's compiled count
