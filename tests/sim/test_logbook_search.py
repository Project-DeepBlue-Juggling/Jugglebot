"""Tests for sim/analysis/logbook_search.py — logbook prior-art search."""

import json
import os
import textwrap

import pytest

# Import path setup handled by tests/conftest.py
from sim.analysis.logbook_search import (
    _parse_frontmatter,
    _extract_section,
    load_entries,
    find_similar,
)


# ---------------------------------------------------------------------------
# Frontmatter parsing
# ---------------------------------------------------------------------------

class TestParseFrontmatter:
    def test_simple_key_value(self):
        text = textwrap.dedent("""\
            ---
            title: My Entry
            type: investigation
            date: 2026-04-01
            status: resolved
            ---
            # Body
        """)
        fm = _parse_frontmatter(text)
        assert fm['title'] == 'My Entry'
        assert fm['type'] == 'investigation'
        assert fm['date'] == '2026-04-01'
        assert fm['status'] == 'resolved'

    def test_quoted_values(self):
        text = textwrap.dedent("""\
            ---
            title: "Quoted title"
            phase: "3.1"
            ---
        """)
        fm = _parse_frontmatter(text)
        assert fm['title'] == 'Quoted title'
        assert fm['phase'] == '3.1'

    def test_list_values(self):
        text = textwrap.dedent("""\
            ---
            title: Test
            related_issues:
              - VEL_FF_BUG
              - COLD_HOLD_STROKE_MIN
            subsystem:
              - mpc
              - motion
            ---
        """)
        fm = _parse_frontmatter(text)
        assert fm['related_issues'] == ['VEL_FF_BUG', 'COLD_HOLD_STROKE_MIN']
        assert fm['subsystem'] == ['mpc', 'motion']

    def test_single_item_list(self):
        text = textwrap.dedent("""\
            ---
            title: Test
            commits:
              - abc1234
            ---
        """)
        fm = _parse_frontmatter(text)
        assert fm['commits'] == ['abc1234']

    def test_no_frontmatter(self):
        text = "# Just a heading\nSome body text."
        fm = _parse_frontmatter(text)
        assert fm == {}

    def test_empty_frontmatter(self):
        text = "---\n---\n# Body"
        fm = _parse_frontmatter(text)
        assert fm == {}

    def test_mixed_key_types(self):
        text = textwrap.dedent("""\
            ---
            title: Test Entry
            type: bugfix
            sessions:
              - mpc_20260401_143845.csv
            tags:
              - safety
              - performance
            ---
        """)
        fm = _parse_frontmatter(text)
        assert fm['title'] == 'Test Entry'
        assert fm['type'] == 'bugfix'
        assert fm['sessions'] == ['mpc_20260401_143845.csv']
        assert fm['tags'] == ['safety', 'performance']


# ---------------------------------------------------------------------------
# Section extraction
# ---------------------------------------------------------------------------

class TestExtractSection:
    SAMPLE = textwrap.dedent("""\
        ---
        title: Test
        ---
        # Title

        ## Summary

        This is the summary.

        ## Fix

        Changed foo to bar in `controller/mpc.py`.
        Also updated tests.

        ## Outcome

        All tests pass. Tracking error improved.

        ## Open Questions

        None.
    """)

    def test_extract_fix(self):
        fix = _extract_section(self.SAMPLE, 'Fix')
        assert 'Changed foo to bar' in fix
        assert 'Also updated tests' in fix

    def test_extract_outcome(self):
        outcome = _extract_section(self.SAMPLE, 'Outcome')
        assert 'All tests pass' in outcome

    def test_extract_summary(self):
        summary = _extract_section(self.SAMPLE, 'Summary')
        assert 'This is the summary' in summary

    def test_missing_section(self):
        result = _extract_section(self.SAMPLE, 'Nonexistent')
        assert result is None

    def test_last_section(self):
        last = _extract_section(self.SAMPLE, 'Open Questions')
        assert 'None' in last


# ---------------------------------------------------------------------------
# Entry loading (uses real logbook if available)
# ---------------------------------------------------------------------------

class TestLoadEntries:
    def test_load_real_logbook(self):
        """Load entries from the actual logbook directory."""
        repo_root = os.path.dirname(os.path.dirname(os.path.dirname(
            os.path.abspath(__file__))))
        logbook_dir = os.path.join(repo_root, 'logbook')

        if not os.path.isdir(logbook_dir):
            pytest.skip('logbook directory not found')

        entries = load_entries(logbook_dir)
        # We know there are at least 2 entries
        assert len(entries) >= 2

        # Check structure of first entry
        entry = entries[0]
        assert 'filename' in entry
        assert 'frontmatter' in entry
        assert entry['frontmatter'].get('title')
        assert entry['frontmatter'].get('type')
        assert entry['frontmatter'].get('date')

    def test_load_empty_dir(self, tmp_path):
        entries = load_entries(str(tmp_path))
        assert entries == []

    def test_load_nonexistent_dir(self):
        entries = load_entries('/nonexistent/path')
        assert entries == []

    def test_skips_index_and_readme(self, tmp_path):
        """INDEX.md, README.md, and TEMPLATE.md should be skipped."""
        (tmp_path / 'INDEX.md').write_text('# Index')
        (tmp_path / 'README.md').write_text('# Readme')
        (tmp_path / 'TEMPLATE.md').write_text('# Template')
        (tmp_path / '2026-01-01-real-entry.md').write_text(textwrap.dedent("""\
            ---
            title: Real Entry
            type: bugfix
            date: 2026-01-01
            status: resolved
            ---
            # Real Entry
        """))

        entries = load_entries(str(tmp_path))
        assert len(entries) == 1
        assert entries[0]['filename'] == '2026-01-01-real-entry.md'


# ---------------------------------------------------------------------------
# Similarity matching
# ---------------------------------------------------------------------------

class TestFindSimilar:
    @pytest.fixture
    def logbook_dir(self, tmp_path):
        """Create a temporary logbook with test entries."""
        entry_a = textwrap.dedent("""\
            ---
            title: Oscillation bug
            type: investigation
            date: 2026-03-30
            status: resolved
            related_issues:
              - VEL_FF_BUG
            subsystem:
              - mpc
              - motion
            tags:
              - safety
            ---
            # Oscillation bug

            ## Summary
            Platform oscillated violently.

            ## Fix
            Fixed velocity formula.

            ## Outcome
            Resolved completely.
        """)

        entry_b = textwrap.dedent("""\
            ---
            title: Cold hold discontinuity
            type: investigation
            date: 2026-04-01
            status: resolved
            related_issues:
              - COLD_HOLD_STROKE_MIN
            subsystem:
              - mpc
              - controller
            tags:
              - safety
              - performance
            ---
            # Cold hold discontinuity

            ## Summary
            Cold hold returned wrong position.

            ## Fix
            Changed fallback to use current position.

            ## Outcome
            Discontinuity resolved.
        """)

        (tmp_path / '2026-03-30-oscillation.md').write_text(entry_a)
        (tmp_path / '2026-04-01-cold-hold.md').write_text(entry_b)
        return str(tmp_path)

    def test_match_by_flag_type(self, logbook_dir):
        matches = find_similar(
            flag_types=['oscillation'],
            logbook_dir=logbook_dir,
        )
        assert len(matches) >= 1
        assert matches[0]['title'] == 'Oscillation bug'

    def test_match_by_subsystem(self, logbook_dir):
        matches = find_similar(
            subsystems=['controller'],
            logbook_dir=logbook_dir,
        )
        assert len(matches) >= 1
        # Cold hold has 'controller' subsystem
        titles = [m['title'] for m in matches]
        assert 'Cold hold discontinuity' in titles

    def test_match_by_issue_id(self, logbook_dir):
        matches = find_similar(
            issue_ids=['VEL_FF_BUG'],
            logbook_dir=logbook_dir,
        )
        assert len(matches) >= 1
        assert matches[0]['title'] == 'Oscillation bug'

    def test_combined_scoring(self, logbook_dir):
        """Flag type + subsystem should score higher than subsystem alone."""
        matches = find_similar(
            flag_types=['oscillation'],
            subsystems=['mpc'],
            logbook_dir=logbook_dir,
        )
        # Oscillation entry should rank first (flag_type match + subsystem)
        assert matches[0]['title'] == 'Oscillation bug'
        assert matches[0]['score'] > matches[1]['score']

    def test_no_matches(self, logbook_dir):
        matches = find_similar(
            flag_types=['workspace'],
            subsystems=['tracking'],
            logbook_dir=logbook_dir,
        )
        assert matches == []

    def test_empty_logbook(self, tmp_path):
        matches = find_similar(
            flag_types=['oscillation'],
            logbook_dir=str(tmp_path),
        )
        assert matches == []

    def test_match_includes_fix_and_outcome(self, logbook_dir):
        matches = find_similar(
            flag_types=['oscillation'],
            logbook_dir=logbook_dir,
        )
        assert matches[0]['fix_summary'] != ''
        assert 'velocity' in matches[0]['fix_summary'].lower()
        assert matches[0]['outcome_summary'] != ''

    def test_match_includes_metadata(self, logbook_dir):
        matches = find_similar(
            issue_ids=['COLD_HOLD_STROKE_MIN'],
            logbook_dir=logbook_dir,
        )
        m = matches[0]
        assert m['date'] == '2026-04-01'
        assert m['status'] == 'resolved'
        assert len(m['match_reasons']) > 0
