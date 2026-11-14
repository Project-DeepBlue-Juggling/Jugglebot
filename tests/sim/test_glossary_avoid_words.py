"""No GLOSSARY.md _Avoid_ word in live plans, recent logbook entries or map/ items.

One looping test lists every offender as file:line: phrase. Exempt: text inside
backticks (inline or fenced), text inside double quotes on the same line, lines
ending ``<!-- avoid-ok -->``, and path-like tokens (anything containing ``/`` or
ending in a file extension) -- a filename such as
``2026-10-19-held-axis-reload-catch-refuses-tracker-reaim.md`` or a service name
such as ``bb/reload`` is an identifier even when it is not backticked, and a
filename never changes. Phrases live in ``avoid_words_checked.txt``.
"""
from __future__ import annotations

import os

from mapkit.checks import (CHECKED_REL, LOGBOOK_FROM, OK_TAG, check_avoid_words,  # noqa: F401
                           find_offenders, load_phrases, scoped_files)

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
CHECKED = os.path.join(os.path.dirname(os.path.abspath(__file__)), "avoid_words_checked.txt")


def test_no_avoid_words_in_live_docs():
    # mapkit.checks.check_avoid_words: the same call the commit path makes
    offenders = check_avoid_words(REPO)
    assert not offenders, "Avoid-words found (use the glossary term, or backtick/quote/avoid-ok):\n" + \
        "\n".join(offenders)


def test_find_offenders_negative_case(tmp_path):
    plan = tmp_path / "plan.md"
    plan.write_text(
        "clean line\n"
        "the leg 6 motor is hot\n"
        "backticked `leg 6` is fine\n"
        'quoted "leg 6" is fine\n'
        "tagged leg 6 line <!-- avoid-ok -->\n"
        "```\nleg 6 in a fence\n```\n"
        "tosses are not toss-the-word\n"
        "  - 2026-10-19-held-axis-reload-catch.md and bb/reload are paths\n"
        "but a bare reload here is not\n",
        encoding="utf-8")
    assert find_offenders([str(plan)], ["leg 6", "toss", "reload"], str(tmp_path)) == [
        "plan.md:2: leg 6", "plan.md:9: toss", "plan.md:11: reload"]


def test_frontmatter_reference_lines_are_identifiers(tmp_path):
    """BUG-45 (2026-11-01): inside the frontmatter block a `parent:` /
    `promoted_to:` stem and a bare `- <stem>` list item are identifiers like
    the filenames above; a single bare word there is not a stem; body text,
    a body `- reload` bullet and a body `---` rule are scanned as before, and
    a file that does not open with `---` gets no exemption."""
    item = tmp_path / "item.md"
    item.write_text(
        "---\n"
        "parent: bug-sim-reload-gate-limit-jerk-regression\n"
        "links:\n"
        "- 2026-10-28-jb76-solve-tail-is-environment-starvation-instrumented\n"
        "tags:\n"
        "- reload\n"
        "---\n"
        "\n"
        "- reload\n"
        "---\n"
        "a bare reload here\n",
        encoding="utf-8")
    assert find_offenders([str(item)], ["reload"], str(tmp_path)) == [
        "item.md:6: reload", "item.md:9: reload", "item.md:11: reload"]
    plain = tmp_path / "plain.md"
    plain.write_text("parent: bug-sim-reload-x\n---\n- reload\n", encoding="utf-8")
    assert find_offenders([str(plain)], ["reload"], str(tmp_path)) == [
        "plain.md:1: reload", "plain.md:3: reload"]


def test_checked_list_seed():
    assert load_phrases(CHECKED) == ["leg 6", "can-hub", "session limits", "guard E-STOP",
                              "runsheet", "toss", "reload", "ticket number"]
