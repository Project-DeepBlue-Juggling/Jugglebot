# -*- coding: utf-8 -*-
"""Search engineering logbook entries for past investigations with similar symptoms.

Parses YAML frontmatter from logbook/*.md entries and matches against
diagnosis flags and subsystem tags.  Designed to be called by the
fix-proposer agent before proposing new fixes.

No pyyaml dependency — uses simple string parsing for frontmatter.

Usage:
    python sim/analysis/logbook_search.py --flags '["oscillation"]' --subsystems '["mpc"]'
    python sim/analysis/logbook_search.py --flags '["discontinuity", "solve_time"]'
    python sim/analysis/logbook_search.py --issue-ids '["VEL_FF_BUG"]'
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
from typing import Any, Dict, List, Optional, Sequence

# Repo root (two levels up from sim/analysis/)
_ANALYSIS_DIR = os.path.dirname(os.path.abspath(__file__))
_SIM_DIR = os.path.dirname(_ANALYSIS_DIR)
_REPO_ROOT = os.path.dirname(_SIM_DIR)
_LOGBOOK_DIR = os.path.join(_REPO_ROOT, 'logbook')

# Files to skip when scanning logbook/ — structural, by design, never warned about.
_SKIP_FILES = {'INDEX.md', 'README.md', 'TEMPLATE.md'}

#: Front-matter keys every logbook entry must carry.  ``title`` is load-bearing
#: (an entry without one cannot be listed, so it is dropped); the rest are
#: warned about but the entry still loads.  Pinned for the real logbook by
#: ``tests/sim/test_logbook_front_matter.py``.
REQUIRED_FRONT_MATTER = ('title', 'type', 'date', 'status')

# Map diagnosis flag messages to flag types for matching
_FLAG_TYPE_PATTERNS = {
    'oscillation': re.compile(r'(?i)oscillat|chatter'),
    'discontinuity': re.compile(r'(?i)discontinuit|jump'),
    'solve_time': re.compile(r'(?i)solve.*budget|solve.*time|staleness'),
    'tracking': re.compile(r'(?i)tracking.*error|steady.state.*error'),
    'workspace': re.compile(r'(?i)workspace|stroke.*limit'),
    'torque': re.compile(r'(?i)torque|feedforward'),
}


# ---------------------------------------------------------------------------
# Frontmatter parsing (no pyyaml dependency)
# ---------------------------------------------------------------------------

def _parse_frontmatter(text: str) -> Dict[str, Any]:
    """Parse YAML frontmatter from a markdown file's text.

    Handles simple key: value pairs and key: [list] / key:\\n  - item syntax.
    """
    if not text.startswith('---'):
        return {}

    # Find closing ---
    end = text.find('\n---', 3)
    if end == -1:
        return {}

    fm_text = text[4:end].strip()
    result: Dict[str, Any] = {}
    current_key: Optional[str] = None
    current_list: Optional[List[str]] = None

    for line in fm_text.split('\n'):
        stripped = line.strip()
        if not stripped:
            continue

        # List item continuation
        if stripped.startswith('- ') and current_key is not None:
            if current_list is None:
                current_list = []
            current_list.append(stripped[2:].strip().strip('"').strip("'"))
            result[current_key] = current_list
            continue

        # Key: value pair
        colon_idx = stripped.find(':')
        if colon_idx > 0:
            # Flush previous list
            current_key = stripped[:colon_idx].strip()
            value = stripped[colon_idx + 1:].strip()

            if value:
                # Inline value — strip quotes
                result[current_key] = value.strip('"').strip("'")
                current_list = None
            else:
                # Value on next lines (list)
                current_list = []
                result[current_key] = current_list

    return result


def _extract_section(text: str, heading: str) -> Optional[str]:
    """Extract the text of a ## section from markdown."""
    pattern = re.compile(r'^## ' + re.escape(heading) + r'\s*$', re.MULTILINE)
    match = pattern.search(text)
    if not match:
        return None

    start = match.end()
    # Find next ## heading or end of file
    next_heading = re.search(r'^## ', text[start:], re.MULTILINE)
    if next_heading:
        end = start + next_heading.start()
    else:
        end = len(text)

    return text[start:end].strip()


# ---------------------------------------------------------------------------
# Entry loading
# ---------------------------------------------------------------------------

def scan_entries(logbook_dir: str = _LOGBOOK_DIR
                 ) -> tuple[List[Dict[str, Any]], List[str]]:
    """Load logbook entries AND the problems found while loading them.

    Returns ``(entries, warnings)``.  A warning is emitted for every entry that
    is dropped (no parseable front matter, or no ``title`` — nothing downstream
    can display such an entry) and for every entry that loads but is missing one
    of :data:`REQUIRED_FRONT_MATTER`.  The structural skips in
    :data:`_SKIP_FILES` are by design and are never warned about.

    Why warn at all: this search is what a fix-proposer consults *before*
    proposing a fix, so an entry silently dropped for a typo'd front-matter
    block is prior art that a future session will never see.  Silence made the
    drop indistinguishable from "no such investigation exists".
    """
    entries: List[Dict[str, Any]] = []
    warnings: List[str] = []

    if not os.path.isdir(logbook_dir):
        return entries, warnings

    for fname in sorted(os.listdir(logbook_dir)):
        if fname in _SKIP_FILES or not fname.endswith('.md'):
            continue

        path = os.path.join(logbook_dir, fname)
        with open(path, 'r', encoding='utf-8') as f:
            text = f.read()

        fm = _parse_frontmatter(text)
        if not fm:
            warnings.append(
                '%s: DROPPED — no parseable YAML front matter (need a leading '
                '"---" block, closed by "---", holding at least one key)' % fname)
            continue
        if not fm.get('title'):
            warnings.append(
                '%s: DROPPED — front matter has no "title"' % fname)
            continue

        missing = [key for key in REQUIRED_FRONT_MATTER
                   if not _nonempty_str(fm.get(key))]
        if missing:
            warnings.append(
                '%s: incomplete front matter — missing/blank: %s'
                % (fname, ', '.join(missing)))

        # Normalize list fields
        for key in ('related_issues', 'subsystem', 'tags', 'sessions',
                     'files_changed', 'commits'):
            val = fm.get(key)
            if val is None:
                fm[key] = []
            elif isinstance(val, str):
                fm[key] = [val]

        # Extract key sections for the fix-proposer
        entry = {
            'filename': fname,
            'frontmatter': fm,
            'summary': _extract_section(text, 'Summary'),
            'fix': _extract_section(text, 'Fix'),
            'outcome': _extract_section(text, 'Outcome'),
        }
        entries.append(entry)

    return entries, warnings


def _nonempty_str(value: Any) -> bool:
    """True for a non-blank string (list-valued front matter does not count)."""
    return isinstance(value, str) and bool(value.strip())


def load_entries(logbook_dir: str = _LOGBOOK_DIR,
                 warnings: Optional[List[str]] = None,
                 warn_stream: Any = None) -> List[Dict[str, Any]]:
    """Load all logbook entries with parsed frontmatter and key sections.

    Parameters
    ----------
    warnings : optional list.  When given, load problems are appended to it and
        nothing is printed — the caller owns the surfacing.
    warn_stream : stream for the default surfacing (``sys.stderr`` when
        omitted).  Pass ``False`` to silence.  Ignored when *warnings* is given.
    """
    entries, found = scan_entries(logbook_dir)

    if warnings is not None:
        warnings.extend(found)
        return entries

    stream = sys.stderr if warn_stream is None else warn_stream
    if found and stream:
        for warning in found:
            print('logbook_search: WARNING %s' % warning, file=stream)

    return entries


# ---------------------------------------------------------------------------
# Similarity matching
# ---------------------------------------------------------------------------

def _classify_flags(flags: Sequence[str]) -> set[str]:
    """Classify flag message strings into flag types."""
    types = set()
    for flag in flags:
        for flag_type, pattern in _FLAG_TYPE_PATTERNS.items():
            if pattern.search(flag):
                types.add(flag_type)
    return types


def find_similar(
    flag_types: Sequence[str] | None = None,
    subsystems: Sequence[str] | None = None,
    issue_ids: Sequence[str] | None = None,
    logbook_dir: str = _LOGBOOK_DIR,
    warnings: Optional[List[str]] = None,
) -> List[Dict[str, Any]]:
    """Find logbook entries with similar symptoms.

    Matching criteria (scored):
      - Direct issue ID match (strong: +3)
      - Flag-type overlap with related_issues patterns (medium: +2 per match)
      - Subsystem overlap (weak: +1 per match)

    Returns entries sorted by score (highest first), with match reasons.

    Load problems (dropped or incomplete entries) go to *warnings* when a list
    is supplied, otherwise to stderr — never nowhere.
    """
    entries = load_entries(logbook_dir, warnings=warnings)
    if not entries:
        return []

    flag_types_set = set(flag_types or [])
    subsystems_set = set(subsystems or [])
    issue_ids_set = set(issue_ids or [])

    # Build a map from known issue IDs to their flag types
    # (based on known_issues.yaml signature types)
    _ISSUE_FLAG_TYPES = {
        'VEL_FF_BUG': {'oscillation'},
        'COLD_HOLD_STROKE_MIN': {'discontinuity', 'solve_time'},
        'ASSIGNMENT_ORDER': {'discontinuity'},
        'MPC_STALENESS': {'solve_time'},
        'LEG2_TRACKING': {'tracking'},
        'CAN_BUS_WATCHDOG': {'ros2_log'},
        'ODRIVE_DISARM': {'ros2_log'},
        'BOOT_TIMEOUT': {'ros2_log'},
        'MOTOR_FEEDBACK_STALE': {'ros2_log'},
        'CAN_STEP_REJECT': {'ros2_log'},
        'FIRMWARE_MISMATCH': {'ros2_log'},
        'TIMING_FIRST_SAMPLE': {'solve_time'},
    }

    matches = []

    for entry in entries:
        fm = entry['frontmatter']
        score = 0
        reasons = []

        entry_issues = set(fm.get('related_issues', []))
        entry_subsystems = set(fm.get('subsystem', []))

        # 1. Direct issue ID match
        if issue_ids_set:
            overlap = issue_ids_set & entry_issues
            if overlap:
                score += 3 * len(overlap)
                reasons.append(f"direct issue match: {', '.join(sorted(overlap))}")

        # 2. Flag-type overlap via related_issues
        if flag_types_set:
            for issue_id in entry_issues:
                issue_flags = _ISSUE_FLAG_TYPES.get(issue_id, set())
                overlap = flag_types_set & issue_flags
                if overlap:
                    score += 2 * len(overlap)
                    reasons.append(
                        f"flag-type match via {issue_id}: "
                        f"{', '.join(sorted(overlap))}"
                    )

        # 3. Subsystem overlap
        if subsystems_set:
            overlap = subsystems_set & entry_subsystems
            if overlap:
                score += len(overlap)
                reasons.append(f"subsystem overlap: {', '.join(sorted(overlap))}")

        if score > 0:
            matches.append({
                'filename': entry['filename'],
                'title': fm.get('title', ''),
                'date': fm.get('date', ''),
                'status': fm.get('status', ''),
                'score': score,
                'match_reasons': reasons,
                'related_issues': fm.get('related_issues', []),
                'subsystem': fm.get('subsystem', []),
                'summary': entry['summary'] or '',
                'fix_summary': entry['fix'] or '',
                'outcome_summary': entry['outcome'] or '',
            })

    # Sort by score descending, then by date descending (stable sort)
    matches.sort(key=lambda m: m['date'], reverse=True)
    matches.sort(key=lambda m: -m['score'])

    return matches


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description='Search logbook for past investigations with similar symptoms')
    parser.add_argument('--flags', type=str, default='[]',
                        help='JSON array of flag type strings '
                             '(oscillation, discontinuity, solve_time, '
                             'tracking, workspace, torque)')
    parser.add_argument('--subsystems', type=str, default='[]',
                        help='JSON array of subsystem tag strings '
                             '(mpc, controller, motion, can, tracking, etc.)')
    parser.add_argument('--issue-ids', type=str, default='[]',
                        help='JSON array of known_issues.yaml IDs '
                             '(e.g., VEL_FF_BUG)')
    parser.add_argument('--logbook-dir', type=str, default=_LOGBOOK_DIR,
                        help='Path to logbook directory')
    parser.add_argument('--strict', action='store_true',
                        help='Exit 1 if any entry was dropped or is missing '
                             'required front matter')
    args = parser.parse_args()

    flag_types = json.loads(args.flags)
    subsystems = json.loads(args.subsystems)
    issue_ids = json.loads(args.issue_ids)

    warnings: List[str] = []
    matches = find_similar(
        flag_types=flag_types,
        subsystems=subsystems,
        issue_ids=issue_ids,
        logbook_dir=args.logbook_dir,
        warnings=warnings,
    )

    # stdout stays pure JSON so callers can pipe it; warnings go to stderr.
    for warning in warnings:
        print('logbook_search: WARNING %s' % warning, file=sys.stderr)

    print(json.dumps(matches, indent=2))
    return 1 if (args.strict and warnings) else 0


if __name__ == '__main__':
    sys.exit(main())
