"""The feed-catch bag probe's REJECTED_NO_BALL regex reads both the old
`self_toss (reload) ENDED (...)` log shape and JB-190's
`self_toss ENDED (...) ... entered by a Feed` shape. Compiled from the
file's own text so nothing hardware-side is imported."""
from __future__ import annotations

import re
from pathlib import Path

_SRC = (Path(__file__).resolve().parents[2] / 'tools' / 'probes'
        / 'feed_catch_bag_probe.py').read_text()


def _regex():
    m = re.search(r'^_REJECTED_NO_BALL_RE = (re\.compile\(.*?\))\n(?=\n|\w)',
                  _SRC, re.S | re.M)
    assert m, 'regex definition not found'
    return eval(m.group(1), {'re': re})


def test_both_end_line_shapes_match():
    rx = _regex()
    assert rx.search('self_toss (reload) ENDED (REJECTED_NO_BALL)')
    assert rx.search('self_toss ENDED (REJECTED_NO_BALL): no ball · 0/0 '
                     'caught · entered by a Feed')
    assert not rx.search('self_toss ENDED (MISSED_CATCH)')
