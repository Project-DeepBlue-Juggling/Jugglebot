"""mapkit.frontmatter: the split between a leading YAML block and the body."""
from __future__ import annotations

import datetime

import pytest

from mapkit.frontmatter import FrontmatterError, split_frontmatter


def test_block_and_body_split():
    """A leading '---' block is safe_load-ed; the body is everything after the closing '---'."""
    meta, body = split_frontmatter("---\ntitle: X\ncreated: 2026-10-24\n---\n# H\ntext\n")
    assert meta == {"title": "X", "created": datetime.date(2026, 10, 24)}
    assert body == "# H\ntext\n"


def test_no_frontmatter_returns_empty_meta_and_whole_text():
    """A file without a leading '---' line yields ({}, text) unchanged."""
    text = "# Title\n---\nnot: frontmatter\n"
    assert split_frontmatter(text) == ({}, text)


def test_empty_block_is_empty_mapping():
    """An empty frontmatter block yields {} rather than None."""
    assert split_frontmatter("---\n---\nbody") == ({}, "body")


def test_invalid_yaml_raises_frontmatter_error_without_path():
    """Invalid YAML raises FrontmatterError, a ValueError, with a path-less message."""
    with pytest.raises(FrontmatterError) as ei:
        split_frontmatter("---\ntitle: a: b\n---\n")
    assert isinstance(ei.value, ValueError)
    assert "invalid YAML" in str(ei.value)


def test_unclosed_and_non_mapping_blocks_rejected():
    """An unclosed block and a non-mapping block (a list) are both FrontmatterErrors."""
    with pytest.raises(FrontmatterError):
        split_frontmatter("---\ntitle: x\n")
    with pytest.raises(FrontmatterError):
        split_frontmatter("---\n- a\n- b\n---\n")


def test_crlf_delimiters_accepted():
    """CRLF line endings around the delimiters still split."""
    meta, body = split_frontmatter("---\r\ntitle: x\r\n---\r\nbody")
    assert meta == {"title": "x"}
    assert body == "body"
