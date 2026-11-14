"""Pins on the Night Shift protocol's text (``.claude/commands/night-shift.md``, closeout C5).

The protocol is prose an unattended run follows, so a later edit can silently drop a promise
the report layout, the preflight and the gate depend on. Each pin names one promise and the
reader that relies on it. Pure text reads; no robot, no network.
"""
from __future__ import annotations

import os
import re

import pytest

REPO = os.path.join(os.path.dirname(__file__), "..", "..")
PROTOCOL = os.path.join(REPO, ".claude", "commands", "night-shift.md")
REASONS = ("sitting", "input-fact", "input-decision", "input-sitting", "owner-session",
           "too-large", "frozen-path", "out-of-reach", "waiting-trigger", "low-priority")


@pytest.fixture(scope="module")
def text():
    with open(PROTOCOL, encoding="utf-8") as fh:
        return fh.read()


def _section(text, start, end):
    i = text.index(start)
    return text[i:text.index(end, i)]


# ---- promises that already held: never deleted ----------------------------------------

def test_gate_alone_decides_merge(text):
    assert "python tools/night_shift_gate.py" in text
    assert "only it decides MERGE" in text
    assert "a HOLD is never promoted" in text


def test_every_test_and_probe_runs_under_the_nonet_wrapper(text):
    assert "tools/night_shift_nonet.sh" in text
    assert "Every test command, probe and bag read" in text


def test_never_list_and_held_only_work_survive(text):
    assert "The never-list" in text
    assert "`.claude/commands/night-shift.md`" in text
    assert "SELF_EDIT_BRANCH" in text


# ---- C5: the delta, the skip batch, the trailer, the inputs section, the conflict rule -

def test_walks_delta_not_items(text):
    s1 = _section(text, "## 1. Start", "## 2. The work loop")
    assert "walk **`delta`, in its order, never `items`**" in s1
    assert "`delta_why[<ID>]`" in s1
    for why in ("answered", "carry", "unclassified", "unblocked", "changed", "added", "full-walk"):
        assert "`%s`:" % why in s1, why
    assert "not opened, not re-rated, and not reported individually" in s1
    assert "Take the Items of `delta` in its order." in text


def test_every_delta_item_gets_a_row_and_only_delta_items(text):
    inv = _section(text, "8. **Every step ends in a status", "9. Stop starting")
    assert "every Item in the `delta`" in inv
    assert "not reported individually" in inv


def test_skip_reasons_are_written_in_one_batch(text):
    assert "python -m mapkit item skip --batch D/skips-<date>.json --commit --no-push" in text
    assert "ONE batch at the end of the walk" in text
    assert '{"item": "<ID>", "clear": true}' in text
    assert "A **Merge or a Hold clears the Item's" in text
    assert "On a full walk every Item is re-examined" in text
    for reason in REASONS:
        assert "`%s`" % reason in text, reason


def test_input_reasons_need_input_and_a_what_the_night_will_do_note(text):
    assert "An `input-*` reason needs `input`" in text
    assert "phrased as what the night WILL do" in text
    assert "ONCE, when the skip is first set" in text
    assert "`input-sitting` files\nno ticket" in text


def test_skipped_section_ends_with_the_unchanged_trailer(text):
    line = "`unchanged: <n> Items not re-examined (reasons on each Item: night_skip)`"
    assert line in text
    assert "ONLY the delta Items skipped tonight" in text
    assert "`unchanged` in `--candidates`" in text


def test_answered_questions_are_acted_on_once_and_not_relisted(text):
    assert "The ticket is never listed again" in text
    assert "not re-listed row by row" in text
    assert "`earlier: <n> open questions from earlier nights, not re-listed`" in text


def test_inputs_section_is_generated_never_hand_written(text):
    assert "## Items the Night Shift could resolve with a small amount of input from you" in text
    assert "Never write its\nrows: run `python -m mapkit night inputs` and paste its output verbatim" in text
    assert "the nine sections in order" in text


def test_merge_conflict_rule(text):
    s = _section(text, "### 2i. MERGE", "### 2j. HOLD")
    assert "on ANY conflict, `git merge --abort`" in s
    assert "a\n   hand-resolved merge is a tree the gate never saw" in s
    assert "git merge -q origin/main" in s
    for gen in ("logbook/INDEX.md", "plans/*/INDEX.md", "docs/project-map.md", "docs/glossary.md",
                "docs/runbook.md"):
        assert gen in s, gen
    assert "python -m mapkit gen" in s
    assert "Held `merge conflict with main in <paths>`" in s
    assert "2e-2g on the NEW tip" in s
    assert "merge only the tip the gate recorded `MERGE` for" in s
    # The rule's merge happens on the branch (the gated tip), never on main.
    assert re.search(r"Never resolve it on `main`", s)
