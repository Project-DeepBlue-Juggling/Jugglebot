"""JB-21: the pure helper behind the `bb/throw_outcome` throw identity."""
from __future__ import annotations

from jugglebot import throw_identity as ti

BASE = 'THROW_REJECTED_BAD_STATE (axis=n/a, detail1=0)'


def test_tag_appends_a_suffix_and_keeps_the_prefix_byte_for_byte():
    tagged = ti.tag_message(BASE, 1761957123.456789)
    assert tagged.startswith(BASE)
    assert tagged == BASE + ' release_wall_s=1761957123.456789'
    assert tagged.split(' ', 1)[0] == 'THROW_REJECTED_BAD_STATE'


def test_an_aim_without_a_release_is_not_tagged():
    assert ti.tag_message(BASE, 0.0) == BASE


def test_parse_round_trips_and_is_none_without_a_suffix():
    assert ti.parse_release_wall_s(ti.tag_message(BASE, 12.5)) == 12.5
    assert ti.parse_release_wall_s(BASE) is None
    assert ti.parse_release_wall_s('OK release_wall_s=abc') is None
    assert ti.parse_release_wall_s('') is None


def test_verdict_matching_rules():
    win = ti.release_window(100.0, 3.0)        # release expected near 103.0
    assert ti.verdict_matches(ti.tag_message(BASE, 102.956), win)
    assert not ti.verdict_matches(ti.tag_message(BASE, 95.0), win)
    assert not ti.verdict_matches(ti.tag_message(BASE, 110.0), win)
    # No suffix (older producer) or nothing recorded: today's behaviour.
    assert ti.verdict_matches(BASE, win)
    assert ti.verdict_matches(ti.tag_message(BASE, 95.0), None)
