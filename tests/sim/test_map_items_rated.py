"""Every map item is rated: the 2026-10-28 backfill set the six metric fields on all
existing items (five on non-bugs; `diagnosis` is bugs-only), and a new item is rated at
triage.  This test pins that no item slips back to unrated.  The metric vocabulary and the
optional-on-new rule live in DOCUMENTATION_GUIDE.md § 2.10 and `mapkit/items.py`."""
from __future__ import annotations

from pathlib import Path

import pytest

from mapkit.items import METRIC_FIELDS

REPO = Path(__file__).resolve().parents[2]
ITEMS = sorted((REPO / "map").glob("*.md"))


def _frontmatter(path: Path) -> dict:
    text = path.read_text(encoding="utf-8")
    assert text.startswith("---\n"), path
    body = text.split("\n---", 1)[0][4:]
    out = {}
    for line in body.splitlines():
        if line and not line.startswith(" ") and ":" in line:
            k, v = line.split(":", 1)
            out[k.strip()] = v.strip()
    return out


@pytest.mark.parametrize("path", ITEMS, ids=[p.stem for p in ITEMS])
def test_every_item_is_rated(path: Path) -> None:
    fm = _frontmatter(path)
    required = [f for f in METRIC_FIELDS if f != "diagnosis" or fm.get("kind") == "bug"]
    missing = [f for f in required if not fm.get(f)]
    assert not missing, f"{path.name} is unrated: {missing}"
