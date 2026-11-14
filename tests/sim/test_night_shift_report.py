"""T-U7: the Night Shift report parser and layout checker."""
from __future__ import annotations

import os
import re

from mapkit.night_shift_report import (INPUTS_CUTOVER, INPUTS_SECTION, LEGACY_SECTIONS, SECTIONS,
                                       parse_report, report_errors)

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
    assert tuple(parsed) == LEGACY_SECTIONS
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


def test_item_rows_accept_per_kind_ids_and_still_reject_garbage():
    """P1 consumer, plan § 2.3: a bare per-kind ID is a legal Item cell; garbage stays loud."""
    for good in ("BUG-12", "FEAT-3", "IDEA-1", "PWAY-4", "TKT-31"):
        assert report_errors(_swap(VALID, "BUG-12 | Fix the thing", good + " | Fix the thing")) == [], good
    for bad in ("BUG-x", "BUG-", "`BUG-3`", "bug-3", "FOO-3", "BUG-03x", "OLD-12", "old-12"):
        errs = report_errors(_swap(VALID, "BUG-12 | Fix the thing", bad + " | Fix the thing"))
        assert len(errs) == 1 and "bare Item ID" in errs[0], bad


def test_bare_id_prefixes_are_mapkit_ids_all_prefixes():
    """``_BARE_ID`` keeps its prefixes literal (the checker stays stdlib-only); this pins
    them to the one enforcement point, ``mapkit.ids.ALL_PREFIXES`` (item-id-per-kind P5)."""
    from mapkit.ids import ALL_PREFIXES
    from mapkit.night_shift_report import _BARE_ID
    inner = re.search(r"\(\?:([A-Z|]+)\)", _BARE_ID.pattern).group(1)
    assert set(inner.split("|")) == set(ALL_PREFIXES)
    for p in ALL_PREFIXES:
        assert _BARE_ID.match("%s-12" % p) and not _BARE_ID.match("%s-012" % p)


# --- close-out C5: the ninth section and the Skipped trailer -----------------------------

INPUT_HEAD = ("| Item | What it is | Input | Exactly what is needed | What the night then does | Respond in |\n"
              "|---|---|---|---|---|---|\n")
INPUT_ROW = "| BUG-13 | Wobble | decision | Pick A or B | Implements the pick | TKT-14 |\n"
TRAILER = "unchanged: 97 Items not re-examined (reasons on each Item: night_skip)"


def _nine(date="2026-11-15", inputs="none\n"):
    text = _swap(VALID, "2026-10-31", date)
    return _swap(text, "## Unfinished", "## %s\n%s\n## Unfinished" % (INPUTS_SECTION, inputs))


def test_the_cutover_date_and_section_counts():
    assert INPUTS_CUTOVER == "2026-11-15" and len(SECTIONS) == 9 and len(LEGACY_SECTIONS) == 8
    assert report_errors(VALID) == []                       # 8 sections, dated before the cutover
    assert report_errors(_nine()) == []                     # 9 sections on the cutover date
    assert report_errors(_nine("2026-11-01")) == []         # a pre-cutover report may carry it too
    errs = report_errors(_swap(VALID, "2026-10-31", "2026-11-15"))
    assert len(errs) == 1 and INPUTS_SECTION in errs[0] and "missing" in errs[0]


def test_ninth_section_must_follow_questions():
    text = _nine()
    text = _swap(text, "## %s\nnone\n\n## Unfinished" % INPUTS_SECTION, "## Unfinished")
    text = _swap(text, "## Skipped", "## %s\nnone\n\n## Skipped" % INPUTS_SECTION)
    errs = report_errors(text)
    assert len(errs) == 1 and "out of order" in errs[0]


def test_inputs_table_header_and_rows_are_checked():
    assert report_errors(_nine(inputs=INPUT_HEAD + INPUT_ROW)) == []
    assert report_errors(_nine(inputs="none recorded\n")) == []
    bad_head = INPUT_HEAD.replace("Input |", "Kind |")
    assert any("must be 'none' or a table" in e for e in report_errors(_nine(inputs=bad_head + INPUT_ROW)))
    assert any("must be 'none' or a table" in e for e in report_errors(_nine(inputs="BUG-13 | x\n")))
    for bad, why in ((INPUT_ROW.replace("BUG-13", "`BUG-13`"), "bare Item ID"),
                     (INPUT_ROW.replace("decision", "maybe"), "Input must be"),
                     (INPUT_ROW.replace("TKT-14", "later"), "Respond in"),
                     (INPUT_ROW.replace(" | TKT-14 |", " |"), "cells")):
        errs = report_errors(_nine(inputs=INPUT_HEAD + bad))
        assert len(errs) == 1 and why in errs[0], (bad, errs)
    for ok in ("sheet", "none"):
        assert report_errors(_nine(inputs=INPUT_HEAD + INPUT_ROW.replace("TKT-14", ok))) == []


def test_skipped_trailer_is_allowed_only_as_the_last_skipped_line():
    with_trailer = _swap(VALID, "BUG-16 | needs hardware", "BUG-16 | needs hardware\n" + TRAILER)
    assert report_errors(with_trailer) == []
    assert report_errors(_swap(VALID, "BUG-16 | needs hardware", "none\n" + TRAILER)) == []
    # not last: a row after it
    errs = report_errors(_swap(VALID, "BUG-16 | needs hardware", TRAILER + "\nBUG-16 | needs hardware"))
    assert len(errs) == 1 and "bare Item ID" in errs[0]
    # in another section
    errs = report_errors(_swap(VALID, "night/bug-15 | stopped at step 2", "night/bug-15 | x\n" + TRAILER))
    assert errs == []  # Unfinished is not an Item-row section; the trailer is simply text there
    errs = report_errors(_swap(VALID, "| BUG-12 | Fix the thing | The thing is fixed. | abc1234 |",
                               "| BUG-12 | Fix the thing | The thing is fixed. | abc1234 |\n" + TRAILER))
    assert len(errs) == 1 and "'## Merged to main'" in errs[0]
    # a malformed trailer is a bad row
    errs = report_errors(_swap(VALID, "BUG-16 | needs hardware", "BUG-16 | x\nunchanged: many Items"))
    assert len(errs) == 1 and "bare Item ID" in errs[0]


def test_inputs_rows_are_not_item_rows_for_the_carry_rule():
    """A standing generated list of every input-* Item must not mark them walked tonight."""
    import sys
    sys.path.insert(0, os.path.join(_REPO, "tools"))
    import night_shift_preflight as pf
    from mapkit.night_shift_report import ITEM_ROW_SECTIONS
    assert INPUTS_SECTION not in ITEM_ROW_SECTIONS
    text = _nine(inputs=INPUT_HEAD + INPUT_ROW.replace("BUG-13", "BUG-99"))
    assert "BUG-99" not in pf.report_item_ids(text)
    assert "BUG-12" in pf.report_item_ids(text)             # a real row still counts


def test_stub_report_writes_the_inputs_section_and_matches_the_constant(tmp_path):
    import subprocess
    import sys
    sys.path.insert(0, os.path.join(_REPO, "tools"))
    import night_shift_stub_report as stub
    assert stub._INPUTS_SECTION == INPUTS_SECTION
    out = tmp_path / "stub.md"
    subprocess.run([sys.executable, os.path.join(_REPO, "tools", "night_shift_stub_report.py"),
                    "--date", "2026-11-16", "--status", "CLAUDE_FAILED", "--clone", str(tmp_path),
                    "--out", str(out), "--started", "a", "--finished", "b"], check=True)
    text = out.read_text()
    assert report_errors(text) == []
    assert parse_report(text)[INPUTS_SECTION] == ["none recorded"]
    assert tuple(parse_report(text)) == SECTIONS


EARLIER = "earlier: 0 open questions from earlier nights, not re-listed"


def test_questions_earlier_line_only_last_in_nine_section_reports():
    q_row = "| TKT-14 | Which one? | BUG-13 | |"
    ok = _swap(_nine(), q_row, q_row + "\n" + EARLIER.replace("0", "12"))
    assert report_errors(ok) == []
    # not last
    errs = report_errors(_swap(_nine(), q_row, EARLIER + "\n" + q_row))
    assert len(errs) == 1 and "bare Item ID" in errs[0]
    # malformed
    errs = report_errors(_swap(_nine(), q_row, q_row + "\nearlier: many open questions"))
    assert len(errs) == 1 and "bare Item ID" in errs[0]
    # eight-section (pre-cutover) report: not allowed
    errs = report_errors(_swap(VALID, "| TKT-14 | Which one? | BUG-13 | |", "| TKT-14 | Which one? | BUG-13 | |\n" + EARLIER))
    assert len(errs) == 1 and "'## Questions" in errs[0]
    # other sections
    errs = report_errors(_swap(_nine(), "| BUG-12 | Fix the thing | The thing is fixed. | abc1234 |",
                               "| BUG-12 | Fix the thing | The thing is fixed. | abc1234 |\n" + EARLIER))
    assert len(errs) == 1 and "Merged" in errs[0]


def test_earlier_line_is_not_a_carry_row():
    import sys
    sys.path.insert(0, os.path.join(_REPO, "tools"))
    import night_shift_preflight as pf
    text = _swap(_nine(), "| TKT-14 | Which one? | BUG-13 | |", "| TKT-14 | Which one? | BUG-13 | |\n" + EARLIER)
    assert pf.report_item_ids(text) >= {"TKT-14"}
    assert not any(i.startswith("earlier") for i in pf.report_item_ids(text))
