"""GLOSSARY.md parser and the CLAUDE.md term-block currency test."""
from __future__ import annotations

import os
from collections import Counter

import pytest

from mapkit.gen_glossary import (Entry, END_MARKER, START_MARKER, extract_block, load_glossary,
                                 parse_glossary, render_term_block)

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

SAMPLE = """# T

preamble **Not**: an entry

## Language

### One

**Alpha**:
First def
continues here.
_Avoid_: beta, gamma (see note, with comma), "quoted, thing"

**Delta**: inline def

### Two

**Eps**:
Another.
_Avoid_: zeta

## Other

**Ignored**:
nope
"""


def test_parse_sample():
    es = parse_glossary(SAMPLE)
    assert [e.term for e in es] == ["Alpha", "Delta", "Eps"]
    a = es[0]
    assert a.cluster == "One"
    assert a.definition == "First def continues here."
    assert a.avoid == ["beta", "gamma (see note, with comma)", '"quoted, thing"']
    assert es[1].avoid == [] and es[1].definition == "inline def"
    assert es[2].cluster == "Two" and es[2].avoid == ["zeta"]


def test_real_glossary_parses():
    es = load_glossary(REPO)
    assert es
    for e in es:
        assert e.cluster, e.term
        assert e.definition.strip(), e.term
    dupes = [t for t, n in Counter(e.term for e in es).items() if n > 1]
    assert not dupes, dupes
    by = {e.term: e for e in es}
    assert "leg 6" in by["Hand"].avoid
    assert by["Hand"].cluster == "The machine"
    assert "Sitting" in by and "Rung" in by
    assert by["Sitting"].cluster and by["Rung"].cluster


def _es():
    return [Entry("A", "d", [], "C1"), Entry("B", "d", [], "C1"), Entry("C", "d", [], "C2")]


def test_render_with_cluster_names():
    assert render_term_block(_es()) == "**C1**: A, B\n**C2**: C"


def test_render_drops_cluster_names_over_limit():
    out = render_term_block(_es(), limit=len("A, B\nC") + 1)
    assert out == "A, B\nC"


def test_render_raises_when_still_over():
    with pytest.raises(ValueError):
        render_term_block(_es(), limit=3)


def test_claude_md_term_block_current():
    with open(os.path.join(REPO, "CLAUDE.md"), encoding="utf-8") as fh:
        text = fh.read()
    assert START_MARKER in text and END_MARKER in text
    block = extract_block(text)
    assert len(block) < 1200, "term block is %d chars; ceiling 1200" % len(block)
    assert block == render_term_block(load_glossary(REPO)), \
        "CLAUDE.md glossary term block is stale: run `python -m mapkit gen`"
