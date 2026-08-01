# -*- coding: utf-8 -*-
"""Front-matter validation for EVERY real logbook entry.

``sim/analysis/logbook_search.py`` is what a fix-proposer consults for prior art
before proposing a fix.  It used to skip an entry with malformed or missing
front matter *silently*, which makes "this investigation was never done" and
"this investigation is unreadable" look identical to the caller.  The loader now
warns; this file is the other half of the contract — it fails if any committed
entry would produce such a warning.

Scope: every ``logbook/*.md`` except the structural files (INDEX.md, README.md,
TEMPLATE.md).  ``logbook/artifacts/`` is not scanned (the loader is flat).
"""

from __future__ import annotations

import os
import re

import pytest

# Import path setup handled by tests/conftest.py
from sim.analysis.logbook_search import (
    REQUIRED_FRONT_MATTER,
    _SKIP_FILES,
    _parse_frontmatter,
    load_entries,
    scan_entries,
)

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
_LOGBOOK_DIR = os.path.join(_REPO_ROOT, 'logbook')

_DATE_RE = re.compile(r'^\d{4}-\d{2}-\d{2}$')


def _entry_files():
    if not os.path.isdir(_LOGBOOK_DIR):
        return []
    return sorted(f for f in os.listdir(_LOGBOOK_DIR)
                  if f.endswith('.md') and f not in _SKIP_FILES)


def test_logbook_dir_exists_and_is_populated():
    """Guard the guard: an empty file list must not vacuously pass everything."""
    assert os.path.isdir(_LOGBOOK_DIR), _LOGBOOK_DIR
    files = _entry_files()
    assert len(files) >= 100, (
        'only %d entries found — did the scan break?' % len(files))


def test_every_entry_has_required_front_matter():
    """Every entry parses, carries title/type/date/status, and dates in ISO.

    Deliberately ONE test that loops rather than ``parametrize(_entry_files())``.
    A parametrized list is built at *collection* time, and CLAUDE.md documents
    parallel Claude sessions sharing this working tree as a normal workflow — an
    entry created between two xdist workers' collection phases aborts the whole
    run with "Different tests were collected between gw0 and gw1" instead of
    producing a clean red.  Looping costs the per-file test id, so the failure
    message lists every offender explicitly instead.
    """
    offenders = []
    for fname in _entry_files():
        path = os.path.join(_LOGBOOK_DIR, fname)
        with open(path, 'r', encoding='utf-8') as handle:
            text = handle.read()

        fm = _parse_frontmatter(text)
        if not fm:
            offenders.append(
                '%s: no parseable YAML front matter — the file must open with '
                'a "---" block that a later line closes with "---"' % fname)
            continue

        for key in REQUIRED_FRONT_MATTER:
            value = fm.get(key)
            if not (isinstance(value, str) and value.strip()):
                offenders.append(
                    '%s: front matter key %r is missing or blank (got %r)'
                    % (fname, key, value))

        # ``date`` is the sort key in find_similar(), compared as a string.
        # Note: a trailing YAML comment (``date: 2026-01-01  # backfilled``) is
        # NOT stripped by the parser and lands here — logbook/README.md says so.
        if isinstance(fm.get('date'), str) and not _DATE_RE.match(fm['date']):
            offenders.append(
                '%s: date %r is not YYYY-MM-DD' % (fname, fm['date']))

    assert not offenders, (
        '%d logbook entries have bad front matter:\n  %s'
        % (len(offenders), '\n  '.join(offenders)))


def test_every_entry_file_is_loaded():
    """No entry is dropped: the loader returns one record per committed file."""
    expected = _entry_files()
    entries, warnings = scan_entries(_LOGBOOK_DIR)
    loaded = sorted(e['filename'] for e in entries)
    assert loaded == expected, (
        'dropped entries: %r' % sorted(set(expected) - set(loaded)))
    assert warnings == [], 'loader warnings on the committed logbook:\n' + \
        '\n'.join(warnings)


# ---------------------------------------------------------------------------
# The warn-on-skip machinery itself
# ---------------------------------------------------------------------------

_GOOD = (
    '---\n'
    'title: Good entry\n'
    'type: bugfix\n'
    'date: 2026-01-01\n'
    'status: resolved\n'
    '---\n'
    '# Good entry\n'
)


def test_missing_title_is_dropped_with_a_warning(tmp_path):
    (tmp_path / '2026-01-01-good.md').write_text(_GOOD, encoding='utf-8')
    (tmp_path / '2026-01-02-bad.md').write_text(
        '---\ntype: bugfix\ndate: 2026-01-02\nstatus: open\n---\n', encoding='utf-8')

    entries, warnings = scan_entries(str(tmp_path))
    assert [e['filename'] for e in entries] == ['2026-01-01-good.md']
    assert len(warnings) == 1
    assert '2026-01-02-bad.md' in warnings[0]
    assert 'DROPPED' in warnings[0] and 'title' in warnings[0]


def test_no_front_matter_is_dropped_with_a_warning(tmp_path):
    (tmp_path / '2026-01-03-plain.md').write_text('# No front matter\n',
                                                  encoding='utf-8')
    entries, warnings = scan_entries(str(tmp_path))
    assert entries == []
    assert len(warnings) == 1
    assert 'no parseable YAML front matter' in warnings[0]


def test_incomplete_front_matter_warns_but_still_loads(tmp_path):
    """A titled entry with a missing 'status' is still prior art — keep it, say so."""
    (tmp_path / '2026-01-04-partial.md').write_text(
        '---\ntitle: Partial\ntype: bugfix\ndate: 2026-01-04\n---\n',
        encoding='utf-8')
    entries, warnings = scan_entries(str(tmp_path))
    assert [e['filename'] for e in entries] == ['2026-01-04-partial.md']
    assert len(warnings) == 1
    assert 'incomplete front matter' in warnings[0]
    assert 'status' in warnings[0]


def test_structural_files_are_skipped_without_warning(tmp_path):
    for name in _SKIP_FILES:
        (tmp_path / name).write_text('# structural\n', encoding='utf-8')
    (tmp_path / '2026-01-01-good.md').write_text(_GOOD, encoding='utf-8')

    entries, warnings = scan_entries(str(tmp_path))
    assert [e['filename'] for e in entries] == ['2026-01-01-good.md']
    assert warnings == []


def test_load_entries_collects_warnings_when_given_a_list(tmp_path):
    (tmp_path / '2026-01-05-bad.md').write_text('# nope\n', encoding='utf-8')
    collected = []
    entries = load_entries(str(tmp_path), warnings=collected)
    assert entries == []
    assert len(collected) == 1


def test_load_entries_prints_warnings_by_default(tmp_path, capsys):
    (tmp_path / '2026-01-05-bad.md').write_text('# nope\n', encoding='utf-8')
    load_entries(str(tmp_path))
    captured = capsys.readouterr()
    assert 'logbook_search: WARNING' in captured.err
    assert captured.out == ''


def test_load_entries_can_be_silenced(tmp_path, capsys):
    (tmp_path / '2026-01-05-bad.md').write_text('# nope\n', encoding='utf-8')
    load_entries(str(tmp_path), warn_stream=False)
    captured = capsys.readouterr()
    assert captured.err == ''
