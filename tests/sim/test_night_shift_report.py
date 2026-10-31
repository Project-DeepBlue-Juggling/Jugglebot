"""T-U7: the Night Shift report parser and layout checker."""
from __future__ import annotations

import os
import re

from mapkit.night_shift_report import SECTIONS, parse_report, report_errors

_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))

VALID = """# Night Shift 2026-10-31

## Nightly
GREEN 1200 passed 2026-10-31

## Merged to main
JB-12 | Fix the thing | abc1234

## Held
JB-13 | night/jb-13 | gate refused: tests | answer JB-14

## Questions for the Operator
JB-14 | Which one? (parent JB-13) -- answer in the app under ## Answer

## Unfinished
night/jb-15 | stopped at step 2

## Skipped
JB-16 | needs hardware

## Unaddressed Sitting results
none recorded

## Cost
units: 3 (sonnet); items touched: 4; wall clock: 04:45 to 06:10
total API cost: USD 12.34 (runner, total_cost_usd)
"""


def _swap(text, old, new):
    assert old in text
    return text.replace(old, new, 1)


def test_valid_report_parses_and_has_no_errors():
    assert report_errors(VALID) == []
    parsed = parse_report(VALID)
    assert tuple(parsed) == SECTIONS
    assert parsed["Merged to main"] == ["JB-12 | Fix the thing | abc1234"]
    assert parsed["Unaddressed Sitting results"] == ["none recorded"]
    # the orchestrator's line plus the runner's appended total API cost
    assert len(parsed["Cost"]) == 2
    assert parsed["Cost"][-1].startswith("total API cost: USD ")


def test_missing_section():
    errs = report_errors(_swap(VALID, "## Skipped\nJB-16 | needs hardware\n\n", ""))
    assert len(errs) == 1 and "missing section '## Skipped'" in errs[0]


def test_misordered_sections():
    text = _swap(VALID, "## Held", "## TMP")
    text = _swap(text, "## Skipped", "## Held")
    text = _swap(text, "## TMP", "## Skipped")
    errs = report_errors(text)
    assert len(errs) == 1 and "out of order" in errs[0]


def test_bad_first_tokens():
    for bad in ("`JB-3` | x | y", "[JB-3] | x | y", "bug-x | x | y", "JB-3x | x | y"):
        errs = report_errors(_swap(VALID, "JB-12 | Fix the thing | abc1234", bad))
        assert len(errs) == 1 and "bare JB-n" in errs[0], bad


def test_wrong_title():
    errs = report_errors(_swap(VALID, "# Night Shift 2026-10-31", "# Report"))
    assert len(errs) == 1 and "title" in errs[0]


def test_readme_layout_block_matches_sections():
    text = open(os.path.join(_REPO, "docs", "night-shift", "README.md")).read()
    block = re.search(r"```\n(# Night Shift.*?)\n```", text, re.S).group(1)
    heads = [l[3:].strip() for l in block.splitlines() if l.startswith("## ")]
    assert tuple(heads) == SECTIONS
