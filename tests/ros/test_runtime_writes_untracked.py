"""Phase C8 (``plans/active/night-shift-closeout.md``): the running robot never
writes a tracked file.

The invariant: a running launch leaves ``git status`` of the live checkout
empty. Enforcement point: ``jugglebot/runtime_paths.py``, the one source of
the paths ``skill_node`` writes (the learner's Memory, the records ledger).
This file drives the two real writers -- ``Memory.append`` and
``records.record_attempt`` -- through those paths inside a temp git repo that
uses the repo's REAL ``.gitignore`` and asserts ``git status --porcelain`` is
empty; then pins the seeding contract the debrief merge relies on (byte-
identical copy, the seed JSON, the row count the robot logs at load, never
rewriting a runtime file on load, the stale-seed rule, atomicity).

Parallel-safe: every repo lives under ``tmp_path``.
"""
from __future__ import annotations

import hashlib
import os
import re
import shutil
import subprocess
from unittest.mock import patch

import pytest

from jugglebot import records as rec
from jugglebot import runtime_paths as rtp
from jugglebot.motion.skills import memory as mem

_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))


def _exp(i: int) -> mem.Experience:
    return mem.Experience(
        site_xy_m=[0.01 * i, -0.02 * i], release_m=[0.01 * i, -0.02 * i, 0.86],
        release_vel_mps=[0.1 * i, 0.0, 2.0], dwell_s=0.3,
        seat_offset_m=[0.0, 0.0], u=[0.0, 0.0, 0.85],
        y=[0.001 * i, -0.001 * i, 0.85], t_abs_s=1_700_000_000.0 + i,
        ball_id=i % 2, release_kind='free_cup', caught=True, migrated=False)


def _memory_text(n: int) -> str:
    return ''.join([','.join(mem.HEADER) + '\n']
                   + [','.join(mem.format_row(_exp(i))) + '\n'
                      for i in range(n)])


def _git(repo, *args) -> str:
    env = dict(os.environ, GIT_CONFIG_GLOBAL=os.devnull,
               GIT_CONFIG_NOSYSTEM='1')
    return subprocess.run(
        ['git', '-C', str(repo)] + list(args), check=True, env=env,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        universal_newlines=True).stdout


def _sha(path) -> str:
    with open(str(path), 'rb') as handle:
        return hashlib.sha256(handle.read()).hexdigest()


@pytest.fixture
def repo(tmp_path):
    """A git repo with the real ``.gitignore`` and the two tracked files
    committed: ``docs/records.md`` (one record) and the live plant's
    ``temp/learn/jugglebot/memory_v2.csv`` (5 rows)."""
    root = tmp_path / 'repo'
    root.mkdir()
    _git(root, 'init', '-q')
    _git(root, 'config', 'user.email', 'test@example.invalid')
    _git(root, 'config', 'user.name', 'test')
    shutil.copy(os.path.join(_REPO, '.gitignore'), str(root / '.gitignore'))
    (root / 'docs').mkdir()
    rec.write_atomic(str(root / 'docs' / 'records.md'), rec.render(rec.rebuild(
        [rec.Attempt('2026-11-01 10:00', 'columns', 12, 11, 'STOPPED')])))
    tracked_mem = root / 'temp' / 'learn' / 'jugglebot' / 'memory_v2.csv'
    tracked_mem.parent.mkdir(parents=True)
    tracked_mem.write_text(_memory_text(5))
    _git(root, 'add', '.gitignore', 'docs/records.md',
         'temp/learn/jugglebot/memory_v2.csv')
    _git(root, 'commit', '-q', '-m', 'tracked state')
    assert _git(root, 'ls-files').split() == [
        '.gitignore', 'docs/records.md', 'temp/learn/jugglebot/memory_v2.csv']
    return root


# ── the invariant ─────────────────────────────────────────────────────────

def test_a_sittings_writes_leave_git_status_empty(repo):
    """Every runtime write the robot makes -- Memory rows for the tracked
    plant and for a brand-new one, an Attempt that beats a record, one that
    books drops -- leaves the porcelain status empty, untracked files
    included."""
    root = str(repo)
    before = {p: _sha(repo / p) for p in
              ('docs/records.md', 'temp/learn/jugglebot/memory_v2.csv')}
    for plant_id in ('jugglebot', 'oval-h050'):
        path, _s = rtp.live_memory_path(root, plant_id)
        memory = mem.Memory(path)
        for i in range(3):
            memory.append(_exp(10 + i))
    path, _s = rtp.live_records_path(root)
    assert rec.record_attempt(path, rec.Attempt(
        '2026-11-14 10:00', 'columns', 31, 30, 'STOPPED'))[0] is True
    rec.record_attempt(path, rec.Attempt(
        '2026-11-14 10:05', 'hop', 4, 3, 'STOPPED', drops=1, caught=3))

    assert _git(repo, 'status', '--porcelain', '--untracked-files=all') == ''
    assert {p: _sha(repo / p) for p in before} == before
    assert len(mem.Memory(rtp.runtime_memory_path(root, 'jugglebot'))) == 8
    assert len(mem.Memory(rtp.runtime_memory_path(root, 'oval-h050'))) == 3
    assert rec.load(rtp.runtime_records_path(root)) != rec.load(
        rtp.tracked_records_path(root))


def test_runtime_paths_live_under_temp_runtime(tmp_path):
    root = str(tmp_path)
    assert rtp.runtime_records_path(root) == os.path.join(
        root, 'temp', 'runtime', 'records.md')
    assert rtp.tracked_records_path(root) == os.path.join(
        root, 'docs', 'records.md')
    assert rtp.runtime_memory_path(root, 'P1') == os.path.join(
        root, 'temp', 'runtime', 'learn', 'P1', 'memory_v2.csv')
    assert rtp.tracked_memory_path(root, 'P1') == mem.memory_path(root, 'P1')
    with pytest.raises(ValueError):
        rtp.runtime_memory_path(root, '../escape')


# ── seeding ───────────────────────────────────────────────────────────────

def test_a_fresh_seed_is_byte_identical_and_its_row_count_is_the_tracked_one(
        repo):
    """The dress-rehearsal check, pinned: on a fresh seed the Memory the robot
    loads has exactly the tracked file's rows, and the seed says so."""
    root = str(repo)
    tracked = rtp.tracked_memory_path(root, 'jugglebot')
    path, seeding = rtp.live_memory_path(root, 'jugglebot')
    assert seeding.action == rtp.SEEDED and seeding.warning == ''
    with open(tracked, 'rb') as a, open(path, 'rb') as b:
        assert a.read() == b.read()
    seed = rtp.read_seed(path)
    assert set(seed) == {'tracked', 'sha256', 'rows', 'seeded_at'}
    assert seed['tracked'] == os.path.abspath(tracked)
    assert seed['sha256'] == _sha(tracked)
    assert seed['rows'] == 5 == seeding.rows
    assert len(mem.Memory(path)) == len(mem.Memory(tracked)) == 5


def test_the_records_seed_has_no_rows_key(repo):
    path, seeding = rtp.live_records_path(str(repo))
    assert seeding.action == rtp.SEEDED
    assert set(rtp.read_seed(path)) == {'tracked', 'sha256', 'seeded_at'}
    assert _sha(path) == _sha(repo / 'docs' / 'records.md')


def test_skill_node_logs_the_loaded_row_count(repo):
    """``SkillNode._open_memory``'s DEBUG line ``memory loaded N rows`` and
    the Learner's INFO goal-start line ``rows N`` (the operator's check) both
    carry the tracked count on a fresh seed."""
    from jugglebot import skill_node as sn
    node = sn.SkillNode()
    lines, info, warnings = [], [], []
    node.get_logger().debug = lines.append
    node.get_logger().info = info.append
    node.get_logger().warning = warnings.append
    with patch.object(sn, '_REPO_ROOT', str(repo)):
        memory = node._open_memory('jugglebot')
    assert len(memory) == 5
    assert info == []          # the console's one-learner-line contract
    node._make_learner(memory)
    assert len(info) == 1 and info[0].startswith('learner: ')
    assert '; rows 5 (' in info[0]
    loaded = [l for l in lines if l.startswith('memory loaded ')]
    assert len(loaded) == 1
    assert loaded[0].startswith('memory loaded 5 rows from %s (seeded now '
                                % rtp.runtime_memory_path(str(repo),
                                                          'jugglebot'))
    assert warnings == []


def test_a_second_load_never_rewrites_the_runtime_copy(repo):
    root = str(repo)
    path, _s = rtp.live_memory_path(root, 'jugglebot')
    mem.Memory(path).append(_exp(20))
    seed_before = rtp.read_seed(path)
    stat_before, bytes_before = os.stat(path), open(path, 'rb').read()
    path2, seeding = rtp.live_memory_path(root, 'jugglebot')
    assert path2 == path
    assert seeding.action == rtp.KEPT and seeding.warning == ''
    assert open(path, 'rb').read() == bytes_before
    assert os.stat(path).st_mtime_ns == stat_before.st_mtime_ns
    assert rtp.read_seed(path) == seed_before
    assert len(mem.Memory(path)) == 6


def test_a_stale_seed_keeps_the_runtime_copy_and_warns(repo):
    """The tracked file changed after seeding (an auto-pull brought rows
    from elsewhere): the runtime copy is kept untouched, the tracked file is
    untouched, and one warning names both and the way out."""
    root = str(repo)
    path, _s = rtp.live_memory_path(root, 'jugglebot')
    mem.Memory(path).append(_exp(30))
    runtime_bytes = open(path, 'rb').read()
    tracked = rtp.tracked_memory_path(root, 'jugglebot')
    with open(tracked, 'a') as handle:
        handle.write(','.join(mem.format_row(_exp(40))) + '\n')
    tracked_bytes = open(tracked, 'rb').read()
    _p, seeding = rtp.live_memory_path(root, 'jugglebot')
    assert seeding.action == rtp.STALE
    assert tracked in seeding.warning and path in seeding.warning
    assert 'runtime_records.py' in seeding.warning
    assert open(path, 'rb').read() == runtime_bytes
    assert open(tracked, 'rb').read() == tracked_bytes
    assert rtp.read_seed(path)['rows'] == 5


def test_a_runtime_file_without_a_seed_is_kept_and_warned(repo):
    root = str(repo)
    path = rtp.runtime_records_path(root)
    os.makedirs(os.path.dirname(path))
    with open(path, 'w') as handle:
        handle.write('hand-made\n')
    seeding = rtp.ensure_seeded(path, rtp.tracked_records_path(root))
    assert seeding.action == rtp.UNSEEDED and 'no readable seed' in \
        seeding.warning
    assert open(path).read() == 'hand-made\n'


def test_no_tracked_file_starts_cold_exactly_as_before(tmp_path):
    """A new plant_id: no runtime file until the first append (which writes
    the header), an empty Memory, and a seed recording the absence."""
    root = str(tmp_path)
    path, seeding = rtp.live_memory_path(root, 'brand_new')
    assert seeding.action == rtp.SEEDED_EMPTY
    assert not os.path.exists(path)
    assert rtp.read_seed(path) == dict(rtp.read_seed(path), sha256=None,
                                       rows=0)
    memory = mem.Memory(path)
    assert len(memory) == 0
    memory.append(_exp(1))
    assert open(path).readline().strip() == ','.join(mem.HEADER)
    _p, again = rtp.live_memory_path(root, 'brand_new')
    assert again.action == rtp.KEPT and again.warning == ''


def test_an_unmigrated_v1_memory_is_still_refused(tmp_path):
    d = tmp_path / 'temp' / 'learn' / 'v1_only'
    d.mkdir(parents=True)
    (d / 'memory.csv').write_text(','.join(mem.HEADER_V1) + '\n')
    with pytest.raises(ValueError, match='migrate_memory_schema_v2'):
        rtp.live_memory_path(str(tmp_path), 'v1_only')
    assert not (tmp_path / 'temp' / 'runtime').exists()


def test_a_crash_mid_copy_leaves_no_short_memory(repo, monkeypatch):
    """The copy is temp-file + rename: if the rename of the runtime file
    fails, no runtime file (and no temp litter) is left, and the next load
    seeds cleanly."""
    root = str(repo)
    runtime = rtp.runtime_memory_path(root, 'jugglebot')
    real_replace = os.replace

    def failing_replace(src, dst):
        if dst == runtime:
            raise OSError('simulated crash')
        return real_replace(src, dst)

    monkeypatch.setattr(rtp.os, 'replace', failing_replace)
    with pytest.raises(OSError):
        rtp.live_memory_path(root, 'jugglebot')
    assert not os.path.exists(runtime)
    assert [n for n in os.listdir(os.path.dirname(runtime))
            if n.endswith('.tmp')] == []
    monkeypatch.setattr(rtp.os, 'replace', real_replace)
    _p, seeding = rtp.live_memory_path(root, 'jugglebot')
    assert seeding.action == rtp.SEEDED
    assert len(mem.Memory(runtime)) == 5


def test_runtime_files_are_world_readable(repo):
    path, _s = rtp.live_records_path(str(repo))
    assert os.stat(path).st_mode & 0o777 == 0o644
    assert os.stat(rtp.seed_path(path)).st_mode & 0o777 == 0o644


def test_the_gui_reader_prefers_runtime_and_falls_back_to_tracked(repo):
    root = str(repo)
    assert rtp.readable_records_path(root) == rtp.tracked_records_path(root)
    assert not os.path.exists(rtp.runtime_records_path(root))   # never seeds
    rtp.live_records_path(root)
    assert rtp.readable_records_path(root) == rtp.runtime_records_path(root)


def test_the_launch_points_skill_node_at_the_runtime_ledger():
    """The launch's ``records_path`` is the runtime copy and its
    ``records_tracked_path`` the tracked ledger, both from runtime_paths."""
    src = open(os.path.join(_REPO, 'ros_ws', 'src', 'jugglebot', 'launch',
                            'jugglebot_launch.py')).read()
    assert "'records_path': _rtp.runtime_records_path(_repo_root())" in src
    assert ("'records_tracked_path': _rtp.tracked_records_path(_repo_root())"
            in src)
    assert "'docs', 'records.md'" not in src


def test_skill_node_never_builds_a_memory_on_the_tracked_path():
    src = open(os.path.join(_REPO, 'ros_ws', 'src', 'jugglebot', 'jugglebot',
                            'skill_node.py')).read()
    assert re.search(r'(?<![\w.])memory_path\(', src) is None
    assert src.count('self._open_memory(plant_id)') == 4
