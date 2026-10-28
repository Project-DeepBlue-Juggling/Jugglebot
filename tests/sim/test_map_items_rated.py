"""Every triaged map item is rated: the 2026-10-28 backfill set the six metric fields on all
existing items (five on non-bugs; `diagnosis` is bugs-only).  Metrics are OPTIONAL on a new
item so quick capture stays one line, and the rating happens at triage, i.e. when the item
leaves `status: inbox`.  This test therefore exempts `inbox` items and pins that nothing
past triage slips back to unrated.  The metric vocabulary and the
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
    if fm.get("status") == "inbox":
        pytest.skip("inbox items are rated at triage (DOCUMENTATION_GUIDE.md § 2.10)")
    required = [f for f in METRIC_FIELDS if f != "diagnosis" or fm.get("kind") == "bug"]
    missing = [f for f in required if not fm.get(f)]
    assert not missing, f"{path.name} is unrated: {missing}"
