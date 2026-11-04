"""JB-150: the round-b rotating sample of tools/admissible_sweep_run.py."""
from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'tools'))
import admissible_sweep_run as run  # noqa: E402


def _names(which='committed'):
    return [n for n, _ in run.families(which)]


def test_family_sets_are_unique_and_sized():
    c, h, old = _names('committed'), _names('h70'), _names('h4500')
    # carry-ring-zvd P1c (JB-102): every pose family gains a `<name>_zvd` twin
    # (carry_zvd true) appended after the pre-ZVD order, so each set is its old
    # base (26 / 30 / 33) plus one twin per pose family (15 / 19 / 22).
    assert len(old) == 41 and len(set(old)) == 41
    assert len(h) == 49 and len(set(h)) == 49
    # the pre-ZVD prefix of each set is unchanged (merge order preserved)
    base = lambda ns: [n for n in ns if not n.endswith('_zvd')]
    assert len(base(old)) == 26 and old[:26] == base(old)
    assert len(base(h)) == 30 and h[:30] == base(h) and base(h)[:26] == base(old)
    # P4 (2026-11-03): the committed set is h70 plus the three one-ball oval dwells.
    assert len(c) == 55 and len(set(c)) == 55
    assert len(base(c)) == 33 and c[:33] == base(c) and base(c)[:30] == base(h)
    assert base(c)[30:] == ['ovalx040_pose', 'ovalx035_pose', 'ovalx030_pose']
    assert len([n for n in c if n.endswith('_zvd')]) == 22
    assert c[-3:] == ['ovalx040_pose_zvd', 'ovalx035_pose_zvd', 'ovalx030_pose_zvd']


def test_sample_is_deterministic_and_sized():
    names = _names()
    a = run.round_b_sample(names, '242c56b683b5', 3)
    assert a == run.round_b_sample(names, '242c56b683b5', 3)
    assert len(a) == 3 and len(set(a)) == 3 and set(a) <= set(names)


def test_sample_rotates_and_covers_every_family():
    names = _names()
    seen = set()
    triples = set()
    for k in range(len(names)):          # successive hashes, int(hash,16) % n == k
        s = run.round_b_sample(names, '%x' % k, 3)
        seen.update(s)
        triples.add(tuple(s))
    assert seen == set(names)
    assert len(triples) == len(names)    # a different triple for every start


def test_sample_all_restores_full_round_b():
    names = _names('h70')
    assert run.round_b_sample(names, 'abc', None) == names
    assert run.round_b_sample(names, 'abc', len(names) + 5) == names
