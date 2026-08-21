"""Contract test for the three plan boards and their ``INDEX.md`` files.

The plans tree is a three-way split, and each directory answers a different
question:

* ``plans/active/``   — what is actually schedulable now;
* ``plans/parked/``   — deliberately not now, each row naming what would unpark it;
* ``plans/archived/`` — done or superseded.

An index is the one place that answers "what is in flight?", so it only stays
useful if it cannot drift.  This test pins both directions for each board:

* **forward** — every plan in the directory has a row in that directory's index,
  i.e. its filename appears as a markdown link target;
* **reverse** — every ``*.md`` filename mentioned anywhere in an index exists in
  *that* index's own directory, so a plan that moved out (parked, archived, or
  renamed) is a failure rather than a stale row nobody notices.

The reverse direction is what makes ``plans/active/INDEX.md`` an honest board:
an active row naming a file that now lives in ``parked/`` or ``archived/`` fails
here.

One further rule, and it is the one that would have caught the problem this
three-way split was built to fix: **every archived plan filename is bare**, with
no ``YYYY-MM-DD `` prefix.  Archival used to rename the file, which broke every
inbound cross-reference at exactly the moment the plan stopped being editable —
``related_plan:`` is filename-only by contract (``DOCUMENTATION_GUIDE.md`` § 2.6)
and resolves by searching the plan directories, so a renamed file resolves
nowhere.  The filename now never changes for the life of a plan; the date lives
in the ``archived:`` frontmatter field and in the archived index.  Without this
assertion the convention can silently regress on the next archival.

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
_PLANS_DIR = os.path.join(_REPO_ROOT, 'plans')
_ACTIVE_DIR = os.path.join(_PLANS_DIR, 'active')
_PARKED_DIR = os.path.join(_PLANS_DIR, 'parked')
_ARCHIVED_DIR = os.path.join(_PLANS_DIR, 'archived')

_INDEX_NAME = 'INDEX.md'
_PROMPT_PREFIX = 'PROMPT-'

# The three boards, keyed by the directory name used in failure messages.
_BOARDS = {
    'active': _ACTIVE_DIR,
    'parked': _PARKED_DIR,
    'archived': _ARCHIVED_DIR,
}

# Any bare markdown filename: "accel-ff-inertia.md", "refactor-2026-07.md".
# Deliberately does NOT match names preceded by a directory component, so a
# reference like "logbook/2026-08-01-foo.md" or a cross-board reference like
# "plans/parked/accel-ff-inertia.md" is ignored -- only plan-level filenames
# are policed.
_MD_NAME_RE = re.compile(r'(?<![\w/.-])([A-Za-z0-9][A-Za-z0-9._+-]*\.md)\b')

# Markdown link target form used by the table rows: "](accel-ff-inertia.md)".
_LINK_TARGET_RE = re.compile(r'\]\(([A-Za-z0-9][A-Za-z0-9._+-]*\.md)\)')

# The archival naming that this split retired: "2026-08-15 some-plan.md".
_DATE_PREFIX_RE = re.compile(r'^\d{4}-\d{2}-\d{2}[ _-]')


def _plan_files(board):
    directory = _BOARDS[board]
    return sorted(
        name for name in os.listdir(directory)
        if name.endswith('.md') and os.path.isfile(os.path.join(directory, name))
    )


def _plannable(board):
    """Plan files subject to the forward rule (index + PROMPTs exempt)."""
    return {
        name for name in _plan_files(board)
        if name != _INDEX_NAME and not name.startswith(_PROMPT_PREFIX)
    }


def _index_path(board):
    return os.path.join(_BOARDS[board], _INDEX_NAME)


def _index_text(board='active'):
    with open(_index_path(board), encoding='utf-8') as handle:
        return handle.read()


@pytest.mark.parametrize('board', sorted(_BOARDS))
def test_index_file_exists(board):
    assert os.path.isfile(_index_path(board)), (
        'plans/{}/INDEX.md is missing. Every {} plan must be listed there; '
        'see the header of that file for the format.'.format(board, board)
    )


@pytest.mark.parametrize('board', sorted(_BOARDS))
def test_every_plan_has_an_index_row(board):
    """Forward direction: adding a plan without an index row fails here."""
    expected = _plannable(board)
    linked = set(_LINK_TARGET_RE.findall(_index_text(board)))
    missing = sorted(expected - linked)
    assert not missing, (
        'These plans are in plans/{}/ but have no row in '
        'plans/{}/INDEX.md: {}. Add a row (or move the plan).'
        .format(board, board, missing)
    )


@pytest.mark.parametrize('board', sorted(_BOARDS))
def test_index_mentions_no_plan_that_left_its_directory(board):
    """Reverse direction: parking or archiving a plan without touching the index fails here.

    This is the staleness protection.  For ``plans/active/INDEX.md`` it is what
    keeps the board honest: a row naming a file that now lives in ``parked/`` or
    ``archived/`` is a failure, not a row nobody notices.
    """
    present = set(_plan_files(board))
    mentioned = set(_MD_NAME_RE.findall(_index_text(board)))
    mentioned.discard(_INDEX_NAME)
    stale = sorted(name for name in mentioned if name not in present)

    def _where(name):
        for other in sorted(_BOARDS):
            if other != board and name in _plan_files(other):
                return '{} (now in plans/{}/)'.format(name, other)
        return name

    assert not stale, (
        'plans/{}/INDEX.md names these files, but they are not in '
        'plans/{}/: {}. Remove the row (the plan was parked, archived or '
        'renamed).'.format(board, board, [_where(n) for n in stale])
    )


@pytest.mark.parametrize('board', sorted(_BOARDS))
def test_index_does_not_list_itself(board):
    linked = set(_LINK_TARGET_RE.findall(_index_text(board)))
    assert _INDEX_NAME not in linked, (
        'plans/{}/INDEX.md should not carry a row for itself.'.format(board)
    )


def test_archived_plan_filenames_are_bare():
    """A plan's filename never changes for its life -- including at archival.

    Archival used to prefix the completion date, which renamed the file and so
    broke every inbound cross-reference (``related_plan:`` is filename-only and
    resolves by searching the plan directories).  The date belongs in the
    ``archived:`` frontmatter field, not in the name.
    """
    prefixed = sorted(
        name for name in _plan_files('archived')
        if _DATE_PREFIX_RE.match(name)
    )
    assert not prefixed, (
        'These archived plans carry a date prefix in the filename: {}. '
        'Archival must not rename the file -- git mv it unchanged and record '
        'the date in an `archived: YYYY-MM-DD` frontmatter field instead. '
        'Renaming breaks every inbound `related_plan:` and prose reference '
        '(see DOCUMENTATION_GUIDE.md 2.6).'.format(prefixed)
    )


def test_no_plan_filename_is_claimed_by_two_boards():
    """A bare filename must identify exactly one plan, in exactly one directory.

    ``related_plan:`` resolves by searching all three directories, so a name
    present in two of them is ambiguous by construction.
    """
    seen = {}
    clashes = []
    for board in sorted(_BOARDS):
        for name in _plannable(board):
            if name in seen:
                clashes.append('{} (plans/{}/ and plans/{}/)'.format(
                    name, seen[name], board))
            else:
                seen[name] = board
    assert not clashes, (
        'These plan filenames exist in more than one plans/ directory, so a '
        'filename-only reference cannot resolve: {}. Rename one, or finish the '
        'move.'.format(sorted(clashes))
    )


@pytest.mark.parametrize('name', sorted(_plannable('active')))
def test_active_index_row_has_a_status_and_a_scope(name):
    """Each row is `| [plan](plan) | status | date | scope |` -- four filled cells."""
    row = None
    for line in _index_text('active').splitlines():
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


@pytest.mark.parametrize('name', sorted(_plannable('parked')))
def test_parked_index_row_names_what_would_unpark_it(name):
    """Each row is `| [plan](plan) | parked | why | what would unpark it |`.

    The last cell is the reason this directory exists: a parked plan whose row
    does not say what would unpark it is a plan nobody can pick up.
    """
    row = None
    for line in _index_text('parked').splitlines():
        if ']({})'.format(name) in line and line.lstrip().startswith('|'):
            row = line
            break
    assert row is not None, (
        '{} has no table row in plans/parked/INDEX.md (found only a prose '
        'mention, or a link outside the table).'.format(name)
    )
    cells = [cell.strip() for cell in row.strip().strip('|').split('|')]
    assert len(cells) >= 4, (
        'Row for {} has {} cells, expected 4 '
        '(plan | parked | why parked | what would unpark it): {!r}'
        .format(name, len(cells), row)
    )
    assert all(cells[1:4]), (
        'Row for {} has an empty parked-date/why/unpark-gate cell: {!r}'
        .format(name, row)
    )


@pytest.mark.parametrize('name', sorted(_plannable('parked')))
def test_parked_plan_says_why_it_is_parked(name):
    """The plan itself carries its own parked note -- the index is not the only copy.

    A reader who opens the plan directly must learn it is parked without having
    to find the index.
    """
    with open(os.path.join(_PARKED_DIR, name), encoding='utf-8') as handle:
        head = handle.read(4000)
    assert re.search(r'(?i)\bparked\b', head), (
        'plans/parked/{} does not say it is parked in its first 4000 chars. '
        'Keep a `status: parked   # YYYY-MM-DD -- why` frontmatter note (or an '
        'equivalent banner) on the plan itself.'.format(name)
    )


@pytest.mark.parametrize('name', sorted(_plannable('archived')))
def test_archived_plan_records_its_archival_date(name):
    """The date the filename used to carry now lives in frontmatter."""
    with open(os.path.join(_ARCHIVED_DIR, name), encoding='utf-8') as handle:
        head = handle.read(4000)
    assert re.search(r'^archived:\s*\d{4}-\d{2}-\d{2}\s*$', head, re.MULTILINE), (
        'plans/archived/{} has no `archived: YYYY-MM-DD` frontmatter field. '
        'That field is now the only in-file record of when the plan closed -- '
        'the filename no longer carries the date.'.format(name)
    )
