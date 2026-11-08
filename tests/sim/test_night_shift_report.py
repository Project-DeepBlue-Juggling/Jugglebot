"""T-U7: the Night Shift report parser and layout checker."""
from __future__ import annotations

import os
import re

from mapkit.night_shift_report import SECTIONS, parse_report, report_errors

_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))

VALID = """# Night Shift 2026-10-31

**Summary.** 1 merged, 1 held, 1 question.

## Nightly
GREEN 1200 passed 2026-10-31

## Merged to main
| Item | What it was | What changed | Commits |
|---|---|---|---|
| BUG-12 | Fix the thing | The thing is fixed. | abc1234 |

## Held
| Item | What it is | Branch | Why the gate held it | What unblocks it | Respond in |
|---|---|---|---|---|---|
| BUG-13 | Wobble | night/bug-13 | (a) gate refused: tests | answer TKT-14 | TKT-14 |
| BUG-17 | Bench thing | night/bug-17 | (b) hardware | a Sitting | none |

## Questions for the Operator
| Ticket | Question (answer in the app under its `## Answer`) | Blocks | Note |
|---|---|---|---|
| TKT-14 | Which one? | BUG-13 | |

## Unfinished
night/bug-15 | stopped at step 2

## Skipped
BUG-16 | needs hardware

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
    assert parsed["Merged to main"][-1].startswith("| BUG-12 |")
    assert parsed["Unaddressed Sitting results"] == ["none recorded"]
    # the orchestrator's line(s) plus the runner's appended total API cost
    assert len(parsed["Cost"]) >= 2
    assert parsed["Cost"][-1].startswith("total API cost: USD ")


def test_missing_section():
    errs = report_errors(_swap(VALID, "## Skipped\nBUG-16 | needs hardware\n\n", ""))
    assert len(errs) == 1 and "missing section '## Skipped'" in errs[0]


def test_misordered_sections():
    text = _swap(VALID, "## Held", "## TMP")
    text = _swap(text, "## Skipped", "## Held")
    text = _swap(text, "## TMP", "## Skipped")
    errs = report_errors(text)
    assert len(errs) == 1 and "out of order" in errs[0]


def test_bad_first_tokens():
    for bad in ("`BUG-3` | x | y", "[BUG-3] | x | y", "bug-x | x | y", "BUG-3x | x | y"):
        errs = report_errors(_swap(VALID, "BUG-12 | Fix the thing | The thing is fixed. | abc1234", bad))
        assert len(errs) == 1 and "bare Item ID" in errs[0], bad


def test_table_header_and_rule_are_layout_not_rows():
    """A GFM table's header row and |---| rule never need an Item ID; a data row does."""
    assert report_errors(VALID) == []
    # a header row renamed to anything still passes (it is identified by the rule below it)
    assert report_errors(_swap(VALID, "| Item | What it was |", "| Thing | Was |")) == []
    # a data row that is not an Item fails, even inside a table
    errs = report_errors(_swap(VALID, "| BUG-16 |", "| row |")) if "| BUG-16 |" in VALID else \
        report_errors(_swap(VALID, "BUG-16 | needs hardware", "| nothing | here |"))
    assert len(errs) == 1 and "bare Item ID" in errs[0]


def test_legacy_pipe_rows_still_pass():
    text = _swap(VALID, """| Item | What it was | What changed | Commits |
|---|---|---|---|
| BUG-12 | Fix the thing | The thing is fixed. | abc1234 |""", "BUG-12 | Fix the thing | abc1234")
    assert report_errors(text) == []


def test_the_live_report_passes_the_checker():
    import glob
    for path in sorted(glob.glob(os.path.join(_REPO, "docs", "night-shift", "2???-??-??.md"))):
        assert report_errors(open(path).read()) == [], path


def test_wrong_title():
    errs = report_errors(_swap(VALID, "# Night Shift 2026-10-31", "# Report"))
    assert len(errs) == 1 and "title" in errs[0]


def test_readme_layout_block_matches_sections():
    text = open(os.path.join(_REPO, "docs", "night-shift", "README.md")).read()
    block = re.search(r"```\n(# Night Shift.*?)\n```", text, re.S).group(1)
    heads = [l[3:].strip() for l in block.splitlines() if l.startswith("## ")]
    assert tuple(heads) == SECTIONS


def test_held_respond_in_must_be_a_bare_id_or_none():
    for bad in ("`TKT-14`", "[TKT-14]", "TKT-14x", "", "later", "TKT-14 and TKT-15"):
        errs = report_errors(_swap(VALID, "answer TKT-14 | TKT-14 |", "answer TKT-14 | %s |" % bad))
        assert len(errs) == 1 and "Respond in" in errs[0], (bad, errs)
    # a short row (cell missing) is refused too
    errs = report_errors(_swap(VALID, "| answer TKT-14 | TKT-14 |", "| answer TKT-14 |"))
    assert len(errs) == 1 and "Respond in" in errs[0]


def test_held_table_without_respond_in_still_passes_as_history():
    old = _swap(VALID, " | Respond in |", " |")
    old = _swap(old, "|---|---|---|---|---|---|", "|---|---|---|---|---|")
    old = _swap(old, "| answer TKT-14 | TKT-14 |", "| answer TKT-14 |")
    old = _swap(old, "| a Sitting | none |", "| a Sitting |")
    assert report_errors(old) == []


def test_item_rows_accept_per_kind_ids_and_still_reject_garbage():  # legacy form: retired at P5
    """P1 consumer, plan § 2.3: a bare per-kind ID is a legal Item cell; garbage stays loud."""
    for good in ("BUG-12", "FEAT-3", "IDEA-1", "PWAY-4", "TKT-31", "JB-12"):
        assert report_errors(_swap(VALID, "BUG-12 | Fix the thing", good + " | Fix the thing")) == [], good
    for bad in ("BUG-x", "BUG-", "`BUG-3`", "bug-3", "FOO-3", "BUG-03x"):
        errs = report_errors(_swap(VALID, "BUG-12 | Fix the thing", bad + " | Fix the thing"))
        assert len(errs) == 1 and "bare Item ID" in errs[0], bad
