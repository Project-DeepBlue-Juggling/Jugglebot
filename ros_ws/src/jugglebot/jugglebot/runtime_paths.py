"""Where the running robot writes: the ONE path source for runtime files.

**The invariant** (``plans/active/night-shift-closeout.md`` Phase C8): a
running launch leaves ``git status`` of the live checkout empty. The robot
used to append to two TRACKED files during a Sitting -- the records ledger
``docs/records.md`` and the learner's Memory
``temp/learn/<plant_id>/memory_v2.csv`` -- which dirtied the tree, so the
2-minute auto-pull skipped for the whole Sitting. Now every runtime write
goes to a gitignored copy under ``temp/runtime/`` (``temp/*`` is ignored and
nothing re-includes ``temp/runtime/``), and the Sitting debrief copies the
runtime files back into the tracked ones (``tools/runtime_records.py``).

Enforcement point: this module. ``skill_node`` (Memory and records),
``jugglebot_launch.py`` (the ``records_path`` parameters) and
``ros_ws/gui/gui_server.py`` (the records panel, which loads this file BY
PATH, so the top level is stdlib only) take every runtime path from here.
Test: ``tests/ros/test_runtime_writes_untracked.py`` drives ``Memory.append``
and ``records.record_attempt`` through these paths in a temp git repo that
uses the repo's real ``.gitignore`` and asserts ``git status --porcelain`` is
empty.

**Paths** (``root`` is the repo root):

* records: tracked ``<root>/docs/records.md`` ->
  runtime ``<root>/temp/runtime/records.md``
* Memory: tracked ``<root>/temp/learn/<plant_id>/memory_v2.csv``
  (``memory.memory_path``) -> runtime
  ``<root>/temp/runtime/learn/<plant_id>/memory_v2.csv``. Every plant_id goes
  through the runtime copy, not only the git-tracked ``jugglebot`` one, so
  there is one rule (an untracked plant's tracked-side file is simply a local
  file the merge also writes back).

**Seeding** (:func:`ensure_seeded`, called on first load and before every
write): if the runtime file is ABSENT, the tracked file's bytes are read once
and written to ``<runtime>.seed`` and then to the runtime file, each
atomically (a temp file in the same directory, fsync, ``os.replace``), so a
crash mid-copy leaves either no runtime file (re-seeded next time) or a
complete one -- never a short Memory. Seed first, then the copy: a crash
between the two leaves a seed with no runtime file, which the next call
simply overwrites. If the tracked file is absent too, only the seed is
written (``sha256`` null, ``rows`` 0) and the runtime file is created by the
first write exactly as a missing file always was (a new plant_id starts
cold; the ledger starts empty). An EXISTING runtime file is never truncated
or rewritten here.

**The seed format** (``<runtime file>.seed``, JSON, one object; the merge
tool's contract):

* ``tracked`` (str): absolute path of the tracked file the copy came from.
* ``sha256`` (str or null): hex SHA-256 of the tracked file's bytes AT
  SEEDING (= the runtime file's bytes at that moment); null = the tracked
  file did not exist.
* ``rows`` (int, Memory only, absent for records): the tracked file's data
  rows at seeding, counted by :func:`csv_data_rows` (non-empty CSV records
  after the header). The Memory is append-only, so the runtime rows a
  Sitting added are ``csv_data_rows(runtime)[rows:]``.
* ``seeded_at`` (str): local ISO-8601 time of seeding, with UTC offset.

**A stale seed** (the runtime file exists and the tracked file's SHA-256 no
longer equals the seed's: an auto-pull brought rows recorded elsewhere, a
hand edit, a quarantine commit) KEEPS THE RUNTIME COPY UNCHANGED and returns
one warning for the caller to log at WARN. Neither side is lost: the runtime
rows stay where they are, the tracked rows stay in git, and the debrief merge
reconciles them (Memory: appends the runtime's post-seed rows to the
tracked file; records: refuses and asks). The robot does NOT see the new
tracked content until that merge re-seeds -- so a deliberate edit of the
tracked Memory (a quarantine) only reaches the robot if the runtime copy and
its seed are removed (or the edit is made to the runtime copy). Re-seeding or
merging on load was rejected: it would rewrite a live Memory on a goal start,
and an append-merge is wrong for an edit that REMOVED rows.
A runtime file with no readable seed is likewise kept, with a warning.
"""

from __future__ import annotations

import csv
import datetime
import hashlib
import io
import json
import os
import tempfile
from typing import List, NamedTuple, Optional, Tuple

#: Runtime files live under ``<root>/temp/runtime`` (gitignored by ``temp/*``).
RUNTIME_DIR = os.path.join('temp', 'runtime')

#: The seed JSON sits beside its runtime file: ``<runtime file>.seed``.
SEED_SUFFIX = '.seed'

#: :class:`Seeding` actions.
SEEDED = 'seeded'               # runtime was absent; tracked copied in now
SEEDED_EMPTY = 'seeded_empty'   # runtime and tracked absent; seed only
KEPT = 'kept'                   # runtime present, seed matches tracked
STALE = 'stale'                 # runtime present, tracked changed since seed
UNSEEDED = 'unseeded'           # runtime present, no readable seed


class Seeding(NamedTuple):
    """What :func:`ensure_seeded` found or did. ``warning`` is '' or the one
    line the caller logs at WARN; ``rows`` is the seed's ``rows`` (Memory)
    or None."""
    runtime: str
    tracked: str
    action: str
    rows: Optional[int]
    seeded_at: str
    warning: str


# ── paths ─────────────────────────────────────────────────────────────────

def tracked_records_path(root: str) -> str:
    """``<root>/docs/records.md``: the committed records ledger."""
    return os.path.join(root, 'docs', 'records.md')


def runtime_records_path(root: str) -> str:
    """``<root>/temp/runtime/records.md``: the ledger the robot writes."""
    return os.path.join(root, RUNTIME_DIR, 'records.md')


def tracked_memory_path(root: str, plant_id: str) -> str:
    """``memory.memory_path``: ``<root>/temp/learn/<plant_id>/memory_v2.csv``.
    Raises ValueError on an unsafe ``plant_id``. (Imported lazily: the GUI
    loads this module by path and never reaches the Memory.)"""
    from jugglebot.motion.skills import memory as _mem
    return _mem.memory_path(root, plant_id)


def runtime_memory_path(root: str, plant_id: str) -> str:
    """``<root>/temp/runtime/learn/<plant_id>/memory_v2.csv``."""
    tracked = tracked_memory_path(root, plant_id)   # validates plant_id
    return os.path.join(root, RUNTIME_DIR, 'learn', plant_id,
                        os.path.basename(tracked))


def seed_path(runtime: str) -> str:
    return runtime + SEED_SUFFIX


# ── seed I/O ──────────────────────────────────────────────────────────────

def csv_data_rows(data: bytes) -> List[List[str]]:
    """The data rows of a Memory CSV's bytes: every non-empty CSV record
    after the header (the seed's ``rows`` is ``len`` of this)."""
    records = [r for r in csv.reader(io.StringIO(data.decode('utf-8'),
                                                 newline='')) if r]
    return records[1:]


def sha256_of(path: str) -> Optional[str]:
    """Hex SHA-256 of a regular file's bytes, None if it is not one."""
    if not os.path.isfile(path):
        return None
    with open(path, 'rb') as handle:
        return hashlib.sha256(handle.read()).hexdigest()


def read_seed(runtime: str) -> Optional[dict]:
    """The seed of ``runtime`` as a dict, None if absent or unreadable."""
    try:
        with open(seed_path(runtime), 'r') as handle:
            seed = json.load(handle)
    except (OSError, ValueError):
        return None
    return seed if isinstance(seed, dict) and 'sha256' in seed else None


def write_atomic(path: str, data: bytes) -> None:
    """``data`` to ``path`` via a same-directory temp file + fsync +
    ``os.replace``: a reader (or a crash) sees the old file or the whole new
    one, never a short one."""
    out_dir = os.path.dirname(path) or '.'
    os.makedirs(out_dir, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix='.%s.' % os.path.basename(path),
                               suffix='.tmp', dir=out_dir)
    try:
        with os.fdopen(fd, 'wb') as handle:
            handle.write(data)
            handle.flush()
            os.fsync(handle.fileno())
        os.chmod(tmp, 0o644)   # mkstemp's 0600 would outlive the rename
        os.replace(tmp, path)
    except BaseException:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise


def write_seed(runtime: str, tracked: str, data: Optional[bytes],
               memory: bool) -> dict:
    """Write ``<runtime>.seed`` for the tracked bytes ``data`` (None = the
    tracked file is absent). Also the merge tool's re-seed after a merge."""
    seed = {
        'tracked': os.path.abspath(tracked),
        'sha256': None if data is None else hashlib.sha256(data).hexdigest(),
        'seeded_at': datetime.datetime.now().astimezone().isoformat(
            timespec='seconds'),
    }
    if memory:
        seed['rows'] = 0 if data is None else len(csv_data_rows(data))
    write_atomic(seed_path(runtime),
                 (json.dumps(seed, indent=2, sort_keys=True) + '\n')
                 .encode('utf-8'))
    return seed


# ── seeding ───────────────────────────────────────────────────────────────

def ensure_seeded(runtime: str, tracked: str, memory: bool = False) -> Seeding:
    """Make ``runtime`` ready to load or write (module docstring, Seeding):
    seed it from ``tracked`` if absent; never touch it if present. Raises
    OSError if the tracked file exists but cannot be read (a directory, no
    permission): the caller's existing refusal path, as for the old load."""
    if os.path.exists(runtime):
        seed = read_seed(runtime)
        if seed is None:
            return Seeding(runtime, tracked, UNSEEDED, None, '', (
                'runtime file %s has no readable seed %s: loading it as is; '
                'the debrief merge cannot tell which rows are new'
                % (runtime, seed_path(runtime))))
        rows = seed.get('rows')
        seeded_at = str(seed.get('seeded_at', '?'))
        if sha256_of(tracked) != seed.get('sha256'):
            return Seeding(runtime, tracked, STALE, rows, seeded_at, (
                'tracked %s changed since the runtime copy was seeded at %s '
                '(an auto-pull or an edit): the robot keeps using the runtime '
                'copy %s unchanged and does NOT see the new tracked content '
                'until the debrief merge (tools/runtime_records.py); to load '
                'the tracked file instead, move the runtime copy and its '
                '.seed aside' % (tracked, seeded_at, runtime)))
        return Seeding(runtime, tracked, KEPT, rows, seeded_at, '')
    if os.path.exists(tracked):
        with open(tracked, 'rb') as handle:
            data = handle.read()
        seed = write_seed(runtime, tracked, data, memory)
        write_atomic(runtime, data)
        return Seeding(runtime, tracked, SEEDED, seed.get('rows'),
                       seed['seeded_at'], '')
    seed = write_seed(runtime, tracked, None, memory)
    return Seeding(runtime, tracked, SEEDED_EMPTY, seed.get('rows'),
                   seed['seeded_at'], '')


def live_memory_path(root: str, plant_id: str) -> Tuple[str, Seeding]:
    """The Memory file the robot loads and appends to for ``plant_id``, seeded
    from the tracked one (:func:`ensure_seeded`). Raises ValueError for an
    unsafe plant_id or an unmigrated schema-v1 ``memory.csv`` beside an
    absent tracked v2 file (``Memory``'s own refusal, checked on the TRACKED
    directory because the runtime directory never holds a v1 file)."""
    from jugglebot.motion.skills import memory as _mem
    tracked = tracked_memory_path(root, plant_id)
    runtime = runtime_memory_path(root, plant_id)
    if not os.path.exists(runtime) and not os.path.exists(tracked):
        _mem.refuse_unmigrated_v1(tracked)
    return runtime, ensure_seeded(runtime, tracked, memory=True)


def live_records_path(root: str) -> Tuple[str, Seeding]:
    """The records ledger the robot writes, seeded from ``docs/records.md``."""
    runtime = runtime_records_path(root)
    return runtime, ensure_seeded(runtime, tracked_records_path(root))


def readable_records_path(root: str) -> str:
    """For a READER (the GUI): the runtime ledger if present, else the
    tracked one. Never seeds (a reader does not write)."""
    runtime = runtime_records_path(root)
    return runtime if os.path.exists(runtime) else tracked_records_path(root)


def describe(seeding: Seeding) -> str:
    """One short clause for the load log line."""
    if seeding.action == SEEDED:
        tail = '' if seeding.rows is None else ', %d rows' % seeding.rows
        return 'seeded now from %s%s' % (seeding.tracked, tail)
    if seeding.action == SEEDED_EMPTY:
        return 'seeded now: no tracked file %s, starting empty' % (
            seeding.tracked,)
    tail = '' if seeding.rows is None else ' with %d rows' % seeding.rows
    return 'runtime copy %s, seeded %s%s' % (seeding.action,
                                             seeding.seeded_at, tail)
