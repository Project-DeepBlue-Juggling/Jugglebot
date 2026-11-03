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
    assert len(old) == 26 and len(set(old)) == 26
    assert len(c) == 30 and len(set(c)) == 30 and c[:26] == old
    assert h == c


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
