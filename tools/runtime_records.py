#!/usr/bin/env python3
"""Copy a Sitting's runtime records and learner Memory back into the tracked files.

Phase C8 (``plans/active/night-shift-closeout.md``): the robot writes the
records ledger and every plant's Memory under the gitignored ``temp/runtime/``
(``ros_ws/src/jugglebot/jugglebot/runtime_paths.py``, whose docstring defines
the ``<file>.seed`` JSON this tool reads). The Sitting debrief runs ``merge``
and then commits ``docs/records.md`` and ``temp/learn/jugglebot/memory_v2.csv``.

    python tools/runtime_records.py status      # what merge would do; exit 0
    python tools/runtime_records.py merge       # do it

Files covered: ``temp/runtime/records.md`` -> ``docs/records.md``, and every
``temp/runtime/learn/<plant_id>/memory_v2.csv`` ->
``temp/learn/<plant_id>/memory_v2.csv`` (a plant other than ``jugglebot`` has
a gitignored, local tracked-side file; the rule is the same).

**The rules, per file** (T = tracked bytes, R = runtime bytes, S = the seed):

* R absent: nothing (``absent``).
* R present, no readable seed: REFUSE (exit 2). Which rows are new cannot be
  told, and this tool never guesses.
* sha256(T) == S.sha256 (null == absent tracked file): the tracked file has
  not moved since seeding, so R is T plus the Sitting's writes: copy R over T.
* Memory, T changed since the seed (an auto-pull brought rows recorded
  elsewhere, a quarantine commit): the Memory is append-only, so the Sitting's
  rows are R's data rows after ``S.rows``. They are appended byte for byte onto
  T, with ``Memory.append``'s rules: a header when T is absent or empty, a
  newline first when T's last byte is not one (a torn row). Before that, R
  must be provably "the seeded bytes plus appended rows": some line-boundary
  prefix of R hashes to S.sha256 and holds S.rows data rows, and R and T carry
  the same header. Anything else REFUSES (exit 2) -- including a tracked file
  that was deleted since a non-null seed.
* records.md, T changed since the seed: REFUSE (exit 2) with what differs.
  The ledger is a rendered page, not an append-only log, so the owner decides
  (the debrief asks), then reruns with ``--resolve-records keep-runtime``
  (R over T; T's changes since the seed are dropped) or ``keep-tracked`` (T
  stays, R is re-seeded from it; a hand-merged T goes this way).

After a merge, T and R both hold the merged bytes M and the seed is rewritten
for M, so a rerun finds every file ``in step`` and writes nothing.

**Write order and crashes.** Each file is written with
``runtime_paths.write_atomic`` (temp file, fsync, rename), so no single file
is ever half-written; the order makes the PAIR recoverable:

1. journal ``<runtime>.merging`` = {tracked sha before, merged sha, rows};
2. R := M;
3. T := M;
4. seed := seed(M);
5. remove the journal.

A rerun that finds a journal finishes or discards it before anything else:
if R does not start with M (sha check at line boundaries), step 2 never
happened and T is untouched, so the journal is dropped and the merge is
recomputed from scratch; if R starts with M, steps 3-5 are redone from R's
prefix, provided T is still either the pre-merge or the merged bytes
(otherwise REFUSE: something changed T after the crash). Why not T first: a
crash after T := M with the old seed makes the rerun see "tracked changed"
and append the same rows a second time. Why not the seed first: a crash
after it leaves a seed whose row count describes M while R is still the old
bytes, so the rerun slices R at the wrong row.

**Exit codes**: 0 merged or nothing to do; 2 at least one file refused (the
others were still merged; the message says what to decide); 3 a Sitting
looks live (``tools/lib/live_session.sh``), nothing written. A live Sitting is
appending to these runtime files, and a merge that rewrites R under it can
drop a row written between the read and the rename. The live check matches
process command lines, so a pytest naming a node test file also reads as
live (BUG-85); the refusal lists the matching processes, and
``--merge-during-live-sitting-may-drop-rows`` overrides it.
"""
from __future__ import annotations

import argparse
import difflib
import hashlib
import json
import os
import subprocess
import sys
from typing import Callable, List, NamedTuple, Optional

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, 'ros_ws', 'src', 'jugglebot'))

from jugglebot import runtime_paths as rtp  # noqa: E402

EXIT_OK = 0
EXIT_REFUSED = 2
EXIT_LIVE = 3

LIVE_OVERRIDE = '--merge-during-live-sitting-may-drop-rows'
JOURNAL_SUFFIX = '.merging'
#: The git-tracked plant (``temp/learn/jugglebot/memory_v2.csv``).
DEFAULT_PLANT = 'jugglebot'


class Entry(NamedTuple):
    label: str           # 'records' or 'memory <plant_id>'
    runtime: str
    tracked: str
    memory: bool


class Refusal(Exception):
    """One file cannot be merged without the owner; the message says why."""


class Plan(NamedTuple):
    state: str            # the status word(s)
    merged: Optional[bytes]   # None = nothing to write
    detail: str = ''


# ── small helpers ─────────────────────────────────────────────────────────

def _read(path: str) -> Optional[bytes]:
    if not os.path.isfile(path):
        return None
    with open(path, 'rb') as handle:
        return handle.read()


def _sha(data: Optional[bytes]) -> Optional[str]:
    return None if data is None else hashlib.sha256(data).hexdigest()


def _rows(data: Optional[bytes]) -> int:
    return 0 if not data else len(rtp.csv_data_rows(data))


def _rel(root: str, path: str) -> str:
    return os.path.relpath(path, root)


def journal_path(runtime: str) -> str:
    return runtime + JOURNAL_SUFFIX


def entries(root: str) -> List[Entry]:
    """records first, then every plant under temp/runtime/learn/, sorted."""
    out = [Entry('records', rtp.runtime_records_path(root),
                 rtp.tracked_records_path(root), False)]
    learn = os.path.join(root, rtp.RUNTIME_DIR, 'learn')
    found = os.listdir(learn) if os.path.isdir(learn) else []
    found = sorted(p for p in found if os.path.isdir(os.path.join(learn, p)))
    # The robot's own plant is always listed, so status names it even
    # before its first runtime copy exists.
    plants = [DEFAULT_PLANT] + [p for p in found if p != DEFAULT_PLANT]
    for plant_id in plants:
        try:
            out.append(Entry('memory %s' % plant_id,
                             rtp.runtime_memory_path(root, plant_id),
                             rtp.tracked_memory_path(root, plant_id), True))
        except ValueError as exc:   # an unsafe plant_id directory name
            out.append(Entry('memory %s' % plant_id,
                             os.path.join(learn, plant_id), '', True))
            sys.stderr.write('runtime_records: %s\n' % exc)
    return out


# ── the Memory append ─────────────────────────────────────────────────────

def _lines(data: bytes) -> List[bytes]:
    return data.splitlines(keepends=True)


def prefix_end(data: bytes, sha: str, rows: Optional[int],
               lines_only: bool = True) -> Optional[int]:
    """The offset in ``data`` just after the line-boundary prefix whose
    SHA-256 is ``sha`` (and which holds ``rows`` data rows, unless None), or
    None. A prefix may also end mid-row: bytes that ended in a torn row,
    which ``Memory.append`` later completed with a bare newline, match
    without that newline and the offset is after it. ``lines_only`` False
    (the records ledger) matches the whole of ``data`` only."""
    if hashlib.sha256(data).hexdigest() == sha and (
            rows is None or _rows(data) == rows):
        return len(data)
    if not lines_only:
        return None
    hasher = hashlib.sha256()
    if hasher.hexdigest() == sha and not rows:
        return 0
    pos = 0
    data_rows = 0
    for index, line in enumerate(_lines(data)):
        if index > 0 and line.strip():
            data_rows += 1
        ok_rows = rows is None or data_rows == rows
        if line.endswith(b"\n") and not line.endswith(b"\r\n"):
            torn = hasher.copy()
            torn.update(line[:-1])
            if ok_rows and torn.hexdigest() == sha:
                return pos + len(line)
        hasher.update(line)
        pos += len(line)
        if ok_rows and hasher.hexdigest() == sha:
            return pos
    return None


def seeded_prefix_end(runtime: bytes, seed_sha: Optional[str],
                      seed_rows: int) -> int:
    """Where the seeded content ends in ``runtime`` (the new rows start),
    proving runtime = seeded bytes + appended rows (:func:`prefix_end`).
    Raises Refusal when no prefix matches."""
    if seed_sha is None:
        return 0
    end = prefix_end(runtime, seed_sha, seed_rows)
    if end is None:
        raise Refusal(
            "the runtime copy is not the seeded bytes plus appended rows (no "
            "prefix hashes to the seed sha256 %s with %d rows): it was edited "
            "after seeding, so which rows are new cannot be told"
            % (seed_sha, seed_rows))
    return end


def append_memory(tracked: Optional[bytes], runtime: bytes,
                  seed: dict) -> bytes:
    """T plus R's post-seed rows (module docstring). Raises Refusal."""
    seed_sha = seed.get('sha256')
    seed_rows = seed.get('rows')
    if not isinstance(seed_rows, int) or seed_rows < 0:
        raise Refusal('the seed has no valid "rows" (%r)' % (seed_rows,))
    if tracked is None and seed_sha is not None:
        raise Refusal(
            'the tracked file was deleted after seeding (seed sha256 %s): '
            'restore it or decide what the Memory should hold' % seed_sha)
    offset = seeded_prefix_end(runtime, seed_sha, seed_rows)
    lines = _lines(runtime)
    if not lines:
        raise Refusal('the runtime copy is empty')
    header = lines[0]
    tail = _lines(runtime[offset:])
    if offset == 0 and tail:
        tail = tail[1:]     # an empty seed: R's first line is its header
    new = [line for line in tail if line.strip()]
    expected_new = _rows(runtime) - seed_rows
    if len(new) != expected_new:
        raise Refusal(
            'the runtime copy has %d rows after the seed by line but %d by '
            'CSV record: a row spans lines, refusing to split it'
            % (len(new), expected_new))
    if tracked is None or not tracked.strip():
        merged = header + b''.join(new)
    else:
        if _lines(tracked)[0].strip() != header.strip():
            raise Refusal('the runtime header %r is not the tracked header '
                          '%r' % (header.strip(), _lines(tracked)[0].strip()))
        sep = b'' if tracked.endswith(b'\n') else b'\n'
        merged = tracked + sep + b''.join(new)
    if _rows(merged) != _rows(tracked) + len(new):
        raise Refusal('the merged file would hold %d rows, not tracked %d + '
                      'runtime-new %d' % (_rows(merged), _rows(tracked),
                                          len(new)))
    return merged


# ── per-file planning ─────────────────────────────────────────────────────

def _records_diff(root: str, entry: Entry, tracked: Optional[bytes],
                  runtime: bytes, limit: int = 40) -> str:
    diff = list(difflib.unified_diff(
        (tracked or b'').decode('utf-8', 'replace').splitlines(),
        runtime.decode('utf-8', 'replace').splitlines(),
        fromfile=_rel(root, entry.tracked) + ' (tracked)',
        tofile=_rel(root, entry.runtime) + ' (runtime)', lineterm='', n=0))
    shown = diff[:limit]
    if len(diff) > limit:
        shown.append('... %d more diff lines' % (len(diff) - limit))
    return '\n'.join(shown)


def plan(root: str, entry: Entry, resolve_records: Optional[str] = None
         ) -> Plan:
    """What merge would do for ``entry`` (no journal handling). Raises
    Refusal."""
    if not entry.tracked:
        raise Refusal('unsafe plant_id directory %s' % entry.runtime)
    runtime = _read(entry.runtime)
    if runtime is None:
        return Plan('absent', None)
    seed = rtp.read_seed(entry.runtime)
    if seed is None:
        raise Refusal(
            'no readable seed %s: which rows the Sitting added cannot be '
            'told. Inspect it; to keep the tracked file, move the runtime '
            'copy aside (the next launch re-seeds)'
            % _rel(root, rtp.seed_path(entry.runtime)))
    tracked = _read(entry.tracked)
    if tracked is not None and tracked == runtime:
        if _sha(tracked) == seed.get('sha256'):
            return Plan('in step', None)
        return Plan('in step; seed refreshed', runtime)
    if _sha(tracked) == seed.get('sha256'):
        if tracked == runtime:
            return Plan('in step', None)
        if entry.memory:
            ahead = _rows(runtime) - (seed.get('rows') or 0)
            return Plan('runtime ahead by %d rows' % ahead, runtime)
        return Plan('runtime ahead', runtime)
    # The tracked file changed since seeding.
    if entry.memory:
        merged = append_memory(tracked, runtime, seed)
        return Plan('tracked changed since seed; appending %d runtime rows'
                    % (_rows(merged) - _rows(tracked)), merged)
    if resolve_records == 'keep-runtime':
        return Plan('tracked changed since seed; keeping runtime', runtime)
    if resolve_records == 'keep-tracked':
        if tracked is None:
            raise Refusal('--resolve-records keep-tracked, but %s is absent'
                          % _rel(root, entry.tracked))
        return Plan('tracked changed since seed; keeping tracked', tracked)
    raise Refusal(
        '%s changed since the runtime copy was seeded at %s (sha256 %s -> '
        '%s), and the ledger is not append-only. Ask the owner, then rerun '
        'with --resolve-records keep-runtime (the runtime ledger replaces '
        'the tracked one) or keep-tracked (hand-merge into the tracked file '
        'first if both sides matter). Tracked vs runtime:\n%s'
        % (_rel(root, entry.tracked), seed.get('seeded_at', '?'),
           seed.get('sha256'), _sha(tracked),
           _records_diff(root, entry, tracked, runtime)))


# ── the write sequence ────────────────────────────────────────────────────

def commit(entry: Entry, merged: bytes, tracked_before: Optional[bytes],
           crash_after: Optional[int] = None) -> None:
    """Steps 1-5 of the module docstring. ``crash_after`` (tests only)
    raises after that step, to drive the recovery path."""
    def step(n: int) -> None:
        if crash_after == n:
            raise RuntimeError('injected crash after step %d' % n)
    rtp.write_atomic(journal_path(entry.runtime), (json.dumps({
        'tracked_before_sha256': _sha(tracked_before),
        'merged_sha256': _sha(merged),
        'merged_rows': _rows(merged) if entry.memory else None,
    }, indent=2, sort_keys=True) + '\n').encode('utf-8'))
    step(1)
    if _read(entry.runtime) != merged:
        rtp.write_atomic(entry.runtime, merged)
    step(2)
    if tracked_before != merged:
        rtp.write_atomic(entry.tracked, merged)
    step(3)
    rtp.write_seed(entry.runtime, entry.tracked, merged, entry.memory)
    step(4)
    os.unlink(journal_path(entry.runtime))


def recover(root: str, entry: Entry) -> str:
    """Finish or drop an interrupted merge's journal. Returns a note, '' if
    there was no journal. Raises Refusal."""
    jpath = journal_path(entry.runtime)
    if not os.path.exists(jpath):
        return ''
    try:
        with open(jpath) as handle:
            journal = json.load(handle)
        merged_sha = journal['merged_sha256']
        before_sha = journal['tracked_before_sha256']
    except (OSError, ValueError, KeyError, TypeError) as exc:
        raise Refusal('unreadable merge journal %s (%s)'
                      % (_rel(root, jpath), exc))
    runtime = _read(entry.runtime)
    end = (None if runtime is None
           else prefix_end(runtime, merged_sha, None, entry.memory))
    tracked_sha = rtp.sha256_of(entry.tracked)
    if end is None and tracked_sha == merged_sha != before_sha:
        raise Refusal(
            "an interrupted merge left %s: %s holds the merged bytes but the "
            "runtime copy does not start with them: resolve by hand"
            % (_rel(root, jpath), _rel(root, entry.tracked)))
    if end is None:
        # Step 2 never happened, so step 3 did not either: T is untouched.
        os.unlink(jpath)
        return 'dropped an interrupted merge journal (nothing was written)'
    merged = runtime[:end]
    if tracked_sha not in (before_sha, merged_sha):
        raise Refusal(
            'an interrupted merge left %s, and %s changed since (sha256 %s, '
            'expected %s or %s): resolve by hand'
            % (_rel(root, jpath), _rel(root, entry.tracked), tracked_sha,
               before_sha, merged_sha))
    if tracked_sha != merged_sha:
        rtp.write_atomic(entry.tracked, merged)
    rtp.write_seed(entry.runtime, entry.tracked, merged, entry.memory)
    os.unlink(jpath)
    return 'finished an interrupted merge'


# ── live check ────────────────────────────────────────────────────────────

def live_sitting(root: str) -> str:
    """'' if no Sitting looks live, else the matching processes
    (``tools/lib/live_session.sh``, the runners' one definition)."""
    lib = os.path.join(root, 'tools', 'lib', 'live_session.sh')
    if not os.path.isfile(lib):
        return ''
    script = ('source "$1" || exit 1; live_session || exit 1; '
              'pgrep -af "$LIVE_SESSION_RE"; exit 0')
    proc = subprocess.run(['bash', '-c', script, 'runtime_records', lib],
                          stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                          universal_newlines=True)
    if proc.returncode != 0:
        return ''
    return proc.stdout.strip() or '(a live process, exited since)'


# ── commands ──────────────────────────────────────────────────────────────

def cmd_status(root: str, out=None) -> int:
    """One line per file; plants other than ``jugglebot`` whose runtime copy
    is absent (a seed-only directory a test or a cold goal left) are counted
    on one line instead."""
    out = out or sys.stdout
    quiet_absent = 0
    for entry in entries(root):
        if (entry.memory and entry.label != 'memory jugglebot'
                and not os.path.exists(entry.runtime)
                and not os.path.exists(journal_path(entry.runtime))):
            quiet_absent += 1
            continue
        name = '%-18s %s' % (entry.label, _rel(root, entry.tracked)
                             if entry.tracked else entry.runtime)
        if os.path.exists(journal_path(entry.runtime)):
            out.write('%s: interrupted merge (journal present; merge '
                      'finishes it)\n' % name)
            continue
        try:
            state = plan(root, entry).state
        except Refusal as exc:
            first = str(exc).split('\n', 1)[0]
            if 'no readable seed' in first:
                state = 'no readable seed'
            elif 'changed since' in first:
                state = 'tracked changed since seed (merge refuses: %s)' % (
                    first,)
            else:
                state = 'merge refuses: %s' % first
        out.write('%s: %s\n' % (name, state))
    if quiet_absent:
        out.write('%d other plant director%s: absent (no runtime copy)\n'
                  % (quiet_absent, 'y' if quiet_absent == 1 else 'ies'))
    return EXIT_OK


def cmd_merge(root: str, resolve_records: Optional[str] = None,
              live_check: Callable[[str], str] = live_sitting,
              allow_live: bool = False, out=None, err=None) -> int:
    out = out or sys.stdout
    err = err or sys.stderr
    live = live_check(root)
    if live and not allow_live:
        err.write(
            'runtime_records: refusing to merge: a Sitting looks live, and '
            'the robot appends to these runtime files (a row written during '
            'the merge could be dropped). Matching processes:\n%s\n'
            'If none of them is a launch (BUG-85: a pytest naming a node test '
            'file matches too), rerun with %s.\n' % (live, LIVE_OVERRIDE))
        return EXIT_LIVE
    refused = 0
    for entry in entries(root):
        name = '%s (%s)' % (entry.label, _rel(root, entry.tracked)
                            if entry.tracked else entry.runtime)
        try:
            note = recover(root, entry) if entry.tracked else ''
            if note:
                out.write('%s: %s\n' % (name, note))
            p = plan(root, entry, resolve_records)
        except Refusal as exc:
            refused += 1
            err.write('runtime_records: REFUSED %s: %s\n' % (name, exc))
            continue
        if p.merged is None:
            out.write('%s: %s\n' % (name, p.state))
            continue
        tracked_before = _read(entry.tracked)
        commit(entry, p.merged, tracked_before)
        rows = (' (%d rows)' % _rows(p.merged)) if entry.memory else ''
        out.write('%s: %s -> merged%s\n' % (name, p.state, rows))
    return EXIT_REFUSED if refused else EXIT_OK


def main(argv: Optional[List[str]] = None,
         live_check: Callable[[str], str] = live_sitting) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n', 1)[0])
    ap.add_argument('--repo', default=REPO,
                    help='repo root (default: this checkout)')
    sub = ap.add_subparsers(dest='cmd', required=True)
    sub.add_parser('status', help='per file: what merge would do (exit 0)')
    mp = sub.add_parser('merge', help='copy the runtime files back')
    mp.add_argument('--resolve-records', choices=('keep-runtime',
                                                   'keep-tracked'),
                    help='the owner\'s answer when docs/records.md changed '
                         'since the seed')
    mp.add_argument(LIVE_OVERRIDE, dest='allow_live', action='store_true',
                    help='merge although a Sitting looks live (BUG-85 false '
                         'positives only)')
    args = ap.parse_args(argv)
    root = os.path.abspath(args.repo)
    if args.cmd == 'status':
        return cmd_status(root)
    return cmd_merge(root, args.resolve_records, live_check, args.allow_live)


if __name__ == '__main__':
    sys.exit(main())
