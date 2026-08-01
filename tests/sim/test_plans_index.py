"""Contract test for ``plans/active/INDEX.md``.

The index is the one place that answers "what is actually in flight?".  It only
stays useful if it cannot drift, so this test pins both directions:

* **forward** — every active plan (``plans/active/*.md``) has a row in the
  index, i.e. its filename appears as a markdown link target;
* **reverse** — every ``*.md`` filename mentioned anywhere in the index exists
  in ``plans/active/``, so an archived plan left behind in the table is a
  failure rather than a stale row nobody notices.

Two names are exempt from the forward direction: ``INDEX.md`` itself, and the
``PROMPT-*.md`` session prompts (self-contained phase-runner prompts, not
plans).  They are still subject to the reverse direction if mentioned.

Pure text, no fixtures, no temp files, no network — safe under xdist.
"""

from __future__ import annotations

import os
import re

import pytest

_TESTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_REPO_ROOT = os.path.dirname(_TESTS_DIR)
_ACTIVE_DIR = os.path.join(_REPO_ROOT, 'plans', 'active')
_INDEX_PATH = os.path.join(_ACTIVE_DIR, 'INDEX.md')

_INDEX_NAME = 'INDEX.md'
_PROMPT_PREFIX = 'PROMPT-'

# Any bare markdown filename: "accel-ff-inertia.md", "refactor-2026-07.md".
# Deliberately does NOT match names preceded by a directory component, so a
# reference like "logbook/2026-08-01-foo.md" is ignored -- only plan-level
# filenames are policed.
_MD_NAME_RE = re.compile(r'(?<![\w/.-])([A-Za-z0-9][A-Za-z0-9._+-]*\.md)\b')

# Markdown link target form used by the table rows: "](accel-ff-inertia.md)".
_LINK_TARGET_RE = re.compile(r'\]\(([A-Za-z0-9][A-Za-z0-9._+-]*\.md)\)')


def _active_plan_files():
    return sorted(
        name for name in os.listdir(_ACTIVE_DIR)
        if name.endswith('.md') and os.path.isfile(os.path.join(_ACTIVE_DIR, name))
    )


def _index_text():
    with open(_INDEX_PATH, encoding='utf-8') as handle:
        return handle.read()


def test_index_file_exists():
    assert os.path.isfile(_INDEX_PATH), (
        'plans/active/INDEX.md is missing. Every active plan must be listed '
        'there; see the header of that file for the format.'
    )


def test_every_active_plan_has_an_index_row():
    """Forward direction: adding a plan without an index row fails here."""
    expected = {
        name for name in _active_plan_files()
        if name != _INDEX_NAME and not name.startswith(_PROMPT_PREFIX)
    }
    linked = set(_LINK_TARGET_RE.findall(_index_text()))
    missing = sorted(expected - linked)
    assert not missing, (
        'These plans are in plans/active/ but have no row in '
        'plans/active/INDEX.md: {}. Add a row (or archive the plan).'.format(missing)
    )


def test_index_mentions_no_plan_that_left_plans_active():
    """Reverse direction: archiving a plan without touching the index fails here."""
    present = set(_active_plan_files())
    mentioned = set(_MD_NAME_RE.findall(_index_text()))
    mentioned.discard(_INDEX_NAME)
    stale = sorted(name for name in mentioned if name not in present)
    assert not stale, (
        'plans/active/INDEX.md names these files, but they are not in '
        'plans/active/: {}. Remove the row (the plan was archived or renamed).'
        .format(stale)
    )


def test_index_does_not_list_itself():
    linked = set(_LINK_TARGET_RE.findall(_index_text()))
    assert _INDEX_NAME not in linked, (
        'plans/active/INDEX.md should not carry a row for itself.'
    )


@pytest.mark.parametrize('name', [
    n for n in _active_plan_files()
    if n != _INDEX_NAME and not n.startswith(_PROMPT_PREFIX)
])
def test_index_row_has_a_status_and_a_scope(name):
    """Each row is `| [plan](plan) | status | date | scope |` -- four filled cells."""
    row = None
    for line in _index_text().splitlines():
        if ']({})'.format(name) in line and line.lstrip().startswith('|'):
            row = line
            break
    assert row is not None, (
        '{} has no table row in plans/active/INDEX.md (found only a prose '
        'mention, or a link outside the table).'.format(name)
    )
    cells = [cell.strip() for cell in row.strip().strip('|').split('|')]
    assert len(cells) >= 4, (
        'Row for {} has {} cells, expected 4 '
        '(plan | status | last touched | scope): {!r}'.format(name, len(cells), row)
    )
    assert all(cells[1:4]), (
        'Row for {} has an empty status/last-touched/scope cell: {!r}'.format(name, row)
    )
