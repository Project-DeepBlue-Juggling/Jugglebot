"""BUG-129 experiment: Fast DDS shared-memory housekeeping at launch.

Pure Python, no ROS imports: ``jugglebot_launch.py`` calls ``clean_stale_shm``
once, before any node starts, and logs ``summary_line`` of the result.

Why not ``fastdds shm clean``: the copy shipped with Foxy (Fast DDS 2.1.4,
/opt/ros/foxy/tools/fastdds/shm/clean.py) has three faults on this box.
(1) Its in-use branch increments ``self.segments_in_use`` while the counter was
created as ``self.__segments_in_use`` (name-mangled), so the first segment a
live process holds (the ``ros2 daemon`` always holds one) raises
AttributeError and the run stops partway, in ``os.listdir`` order. (2) It finds
ports only through their ``_el`` lock file; a stale port from an earlier launch
has none, so the stale ports here are never seen. (3) It probes liveness by
taking ``flock(LOCK_EX | LOCK_NB)`` on a live participant's lock file (and
``open(..., 'ab')`` recreates a lock file that vanished in between), which can
race a participant that is starting at that moment.

This version never locks or opens a Fast DDS file. A file group is stale, and
removed, only when ALL of these hold for every file in it:

- owned by this user (other users' files are never touched);
- no flock/POSIX lock on its inode in /proc/locks (any user's lock counts);
- not mapped (/proc/<pid>/maps) or open (/proc/<pid>/fd) by any process this
  user can read, which covers every ROS process run as this user. A port file
  the ``ros2 daemon`` still maps is therefore KEPT even when its owner is dead:
  unlinking it would leave the daemon writing into an orphaned copy once a new
  participant recreates that port number;
- last modified more than ``min_age_s`` ago (a participant starting right now).

Groups: a segment is ``fastrtps_<16 hex>`` + ``_el``; a port is
``fastrtps_port<N>`` + ``_el`` + ``_sl`` + ``sem.fastrtps_port<N>_mutex``. Any
other name is ignored. The scan is bounded by ``deadline_s``; if it runs out,
or /proc cannot be read, nothing is removed. The function never raises.
"""
from __future__ import annotations

import os
import re
import time
from typing import Dict, List, NamedTuple, Optional, Set, Tuple

SHM_DIR = '/dev/shm'

_SEGMENT_RE = re.compile(r'^fastrtps_([0-9a-f]{16})(?:_el)?$')
_PORT_RE = re.compile(r'^fastrtps_port(\d{1,5})(?:_el|_sl)?$')
_PORT_MUTEX_RE = re.compile(r'^sem\.fastrtps_port(\d{1,5})_mutex$')
# /proc/locks: "15: FLOCK  ADVISORY  WRITE 2619632 00:19:141689 0 EOF"
_LOCK_INODE_RE = re.compile(r'\s[0-9a-f]+:[0-9a-f]+:(\d+)\s')


class CleanResult(NamedTuple):
    removed: int          # files unlinked
    kept_in_use: int      # Fast DDS files kept: locked, mapped or open by a live process
    kept_other: int       # Fast DDS files kept: another user's, or too recent
    errors: int           # unlink failures
    elapsed_ms: float
    note: str             # '' on a normal run; why nothing was removed otherwise


def _group_key(name: str) -> Optional[Tuple[str, str]]:
    m = _SEGMENT_RE.match(name)
    if m:
        return ('segment', m.group(1))
    m = _PORT_RE.match(name) or _PORT_MUTEX_RE.match(name)
    if m:
        return ('port', m.group(1))
    return None


def _locked_inodes(proc: str) -> Set[int]:
    """Inodes with any lock in /proc/locks. Raises OSError if unreadable."""
    with open(os.path.join(proc, 'locks')) as f:
        return {int(m.group(1)) for m in _LOCK_INODE_RE.finditer(' ' + f.read())}


def _held_paths(proc: str, shm_dir: str, deadline: float) -> Optional[Set[str]]:
    """Paths under ``shm_dir`` mapped or open by any process we can read.

    None if the deadline passed before every process was scanned.
    """
    prefix = shm_dir.rstrip('/') + '/'
    held = set()  # type: Set[str]
    for pid in os.listdir(proc):
        if not pid.isdigit():
            continue
        if time.monotonic() > deadline:
            return None
        base = os.path.join(proc, pid)
        try:
            with open(os.path.join(base, 'maps')) as f:
                for line in f:
                    i = line.find(prefix)
                    if i >= 0:
                        held.add(line[i:].rstrip('\n'))
        except OSError:
            pass   # another user's process, or it exited
        try:
            fd_dir = os.path.join(base, 'fd')
            for fd in os.listdir(fd_dir):
                try:
                    target = os.readlink(os.path.join(fd_dir, fd))
                except OSError:
                    continue
                if target.startswith(prefix):
                    held.add(target)
        except OSError:
            pass
    return held


def clean_stale_shm(shm_dir: str = SHM_DIR, proc: str = '/proc',
                    min_age_s: float = 30.0, deadline_s: float = 2.0,
                    dry_run: bool = False) -> CleanResult:
    """Remove Fast DDS shared-memory files no live process holds. Never raises."""
    t0 = time.monotonic()
    deadline = t0 + deadline_s

    def _result(removed=0, in_use=0, other=0, errors=0, note=''):
        return CleanResult(removed, in_use, other, errors,
                           (time.monotonic() - t0) * 1e3, note)

    try:
        groups = {}  # type: Dict[Tuple[str, str], List[str]]
        for name in os.listdir(shm_dir):
            key = _group_key(name)
            if key is not None:
                groups.setdefault(key, []).append(name)
        if not groups:
            return _result()
        try:
            locked = _locked_inodes(proc)
        except OSError as exc:
            return _result(other=sum(map(len, groups.values())),
                           note='/proc/locks unreadable (%s), nothing removed' % exc)
        held = _held_paths(proc, shm_dir, deadline)
        if held is None:
            return _result(other=sum(map(len, groups.values())),
                           note='scan over %.1f s, nothing removed' % deadline_s)

        uid = os.getuid()
        now = time.time()
        removed = in_use = other = errors = 0
        for names in groups.values():
            verdict = 'stale'
            for name in names:
                path = os.path.join(shm_dir, name)
                try:
                    st = os.lstat(path)
                except OSError:
                    continue   # vanished: nothing to keep or remove
                if path in held or st.st_ino in locked:
                    verdict = 'in_use'
                    break
                if st.st_uid != uid or now - st.st_mtime < min_age_s:
                    verdict = 'other'
            if verdict == 'in_use':
                in_use += len(names)
                continue
            if verdict == 'other':
                other += len(names)
                continue
            for name in names:
                if dry_run:
                    removed += 1
                    continue
                try:
                    os.unlink(os.path.join(shm_dir, name))
                    removed += 1
                except FileNotFoundError:
                    pass
                except OSError:
                    errors += 1
        return _result(removed, in_use, other, errors)
    except Exception as exc:  # noqa: BLE001 - a launch step must never fail the launch
        return _result(note='%s: %s, stopped' % (type(exc).__name__, exc))


def summary_line(result: CleanResult, dry_run: bool = False) -> str:
    """The one INFO line the launch logs."""
    line = 'shm clean: %s %d stale Fast DDS files from %s (%d kept in use by a live ' \
           'process, %d kept as recent or not ours, %d errors) in %.0f ms' % (
               'would remove' if dry_run else 'removed', result.removed, SHM_DIR,
               result.kept_in_use, result.kept_other, result.errors, result.elapsed_ms)
    if result.note:
        line += ': ' + result.note
    return line


if __name__ == '__main__':   # dry run by hand: python3 -m jugglebot.dds_shm
    print(summary_line(clean_stale_shm(dry_run=True), dry_run=True))
