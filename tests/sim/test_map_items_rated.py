"""Every triaged map item is rated: the 2026-10-28 backfill set the six metric fields on all
existing items (five on non-bugs; `diagnosis` is bugs-only).  Metrics are OPTIONAL on a new
item so quick capture stays one line, and the rating happens at triage, i.e. when the item
leaves `status: inbox`.  This test therefore exempts `inbox` items and pins that nothing
past triage slips back to unrated.  The metric vocabulary and the
optional-on-new rule live in DOCUMENTATION_GUIDE.md § 2.10 and `mapkit/items.py`."""
from __future__ import annotations

from pathlib import Path

import pytest

from mapkit.checks import is_inbox, unrated_problems

REPO = Path(__file__).resolve().parents[2]
ITEMS = sorted((REPO / "map").glob("*.md"))


@pytest.mark.parametrize("path", ITEMS, ids=[p.stem for p in ITEMS])
def test_every_item_is_rated(path: Path) -> None:
    if is_inbox(str(path)):
        pytest.skip("inbox items are rated at triage (DOCUMENTATION_GUIDE.md § 2.10)")
    # mapkit.checks.unrated_problems: the same check the commit path makes
    bad = unrated_problems(str(path))
    assert not bad, "\n".join(bad)
