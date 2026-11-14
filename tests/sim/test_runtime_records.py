"""``tools/runtime_records.py``: the debrief merge of the runtime records and Memory.

Phase C8 unit 2. Every rule of the module docstring is one table row below,
driven through ``main([...])`` on a throwaway repo under ``tmp_path`` whose
runtime copies are seeded by unit 1's own ``runtime_paths.ensure_seeded``
(so the seed format is the real one). The live-Sitting check is injected:
nothing here depends on what is running on the box (BUG-85).
"""
from __future__ import annotations

import csv
import io
import json
import os
import sys

import pytest

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, "tools"))

import runtime_records as rr  # noqa: E402

rtp = rr.rtp
HEADER = ["site_x", "site_y", "c2", "c3"]   # the merge never parses values


def _row(tag: str) -> bytes:
    buf = io.StringIO()
    csv.writer(buf).writerow([tag, "1.0", "2.0", "3.0"])   # \r\n, as Memory
    return buf.getvalue().encode()


def _memory(*tags: str, header: bool = True) -> bytes:
    head = _row("x").replace(b"x,1.0,2.0,3.0", ",".join(HEADER).encode())
    return (head if header else b"") + b"".join(_row(t) for t in tags)


def _w(path: str, data: bytes) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "wb") as handle:
        handle.write(data)


def _r(path: str):
    if not os.path.exists(path):
        return None
    with open(path, "rb") as handle:
        return handle.read()


def _append_like_memory(path: str, data: bytes) -> None:
    """``Memory.append``'s torn-row rule for raw rows: a bare newline first
    when the file's last byte is not one."""
    old = _r(path) or b""
    sep = b"\n" if old and not old.endswith(b"\n") else b""
    _w(path, old + sep + data)


class Repo:
    def __init__(self, root: str):
        self.root = root
        self.tm = rtp.tracked_memory_path(root, "jugglebot")
        self.rm = rtp.runtime_memory_path(root, "jugglebot")
        self.tr = rtp.tracked_records_path(root)
        self.rr = rtp.runtime_records_path(root)

    def seed_memory(self):
        return rtp.ensure_seeded(self.rm, self.tm, memory=True)

    def seed_records(self):
        return rtp.ensure_seeded(self.rr, self.tr)

    def merge(self, *extra, live=""):
        return rr.main(["--repo", self.root, "merge"] + list(extra),
                       live_check=lambda _root: live)


@pytest.fixture
def repo(tmp_path):
    return Repo(str(tmp_path))


def _rows(data):
    return 0 if not data else len(rtp.csv_data_rows(data))


# ── the rule table ─────────────────────────────────────────────────────────
# Each setup leaves the repo just before the debrief and returns the tracked
# bytes the merge must produce (None = the tracked file must stay as it was).

def m_runtime_absent(r):
    _w(r.tm, _memory("a", "b"))
    return None


def m_seed_matches_copy(r):
    _w(r.tm, _memory("a", "b"))
    r.seed_memory()
    _append_like_memory(r.rm, _row("n1") + _row("n2") + _row("n3"))
    return _memory("a", "b", "n1", "n2", "n3")


def m_tracked_changed_append(r):
    _w(r.tm, _memory("a", "b"))
    r.seed_memory()
    _append_like_memory(r.rm, _row("n1") + _row("n2") + _row("n3"))
    _w(r.tm, _memory("a", "b", "p1", "p2"))      # an auto-pull's rows
    return _memory("a", "b", "p1", "p2", "n1", "n2", "n3")


def m_no_seed(r):
    _w(r.tm, _memory("a"))
    r.seed_memory()
    _append_like_memory(r.rm, _row("n1"))
    os.unlink(rtp.seed_path(r.rm))
    return None


def m_unreadable_seed(r):
    _w(r.tm, _memory("a"))
    r.seed_memory()
    _w(rtp.seed_path(r.rm), b"{not json")
    return None


def m_seed_null_tracked_still_absent(r):
    r.seed_memory()                                  # sha256 null, rows 0
    _append_like_memory(r.rm, _memory("n1", "n2"))   # the first append's header
    return _memory("n1", "n2")


def m_seed_null_tracked_appeared(r):
    r.seed_memory()
    _append_like_memory(r.rm, _memory("n1", "n2"))
    _w(r.tm, _memory("p1"))
    return _memory("p1", "n1", "n2")


def m_runtime_edited_and_tracked_changed(r):
    _w(r.tm, _memory("a", "b"))
    r.seed_memory()
    _w(r.rm, _memory("a", "n1"))                     # "b" removed by hand
    _w(r.tm, _memory("a", "b", "p1"))
    return None


def m_tracked_deleted(r):
    _w(r.tm, _memory("a"))
    r.seed_memory()
    _append_like_memory(r.rm, _row("n1"))
    os.unlink(r.tm)
    return None


def m_tracked_torn_last_row(r):
    _w(r.tm, _memory("a"))
    r.seed_memory()
    _append_like_memory(r.rm, _row("n1"))
    _w(r.tm, _memory("a") + b"p1,1.0")              # a torn row arrived
    return _memory("a") + b"p1,1.0\n" + _row("n1")


def m_seeded_torn_row(r):
    _w(r.tm, _memory("a") + b"t,1.0")               # torn at seeding
    r.seed_memory()
    _append_like_memory(r.rm, _row("n1"))           # Memory adds the "\n"
    _w(r.tm, _memory("a") + b"t,1.0" + b"\n" + _row("p1"))
    return _memory("a") + b"t,1.0\n" + _row("p1") + _row("n1")


def m_header_mismatch(r):
    _w(r.tm, _memory("a"))
    r.seed_memory()
    _append_like_memory(r.rm, _row("n1"))
    _w(r.tm, b"other,header\r\n" + _row("p1"))
    return None


def m_in_step(r):
    _w(r.tm, _memory("a"))
    r.seed_memory()
    return None


def r_absent(r):
    _w(r.tr, b"# ledger\n")
    return None


def r_seed_matches_copy(r):
    _w(r.tr, b"# ledger\nold\n")
    r.seed_records()
    _w(r.rr, b"# ledger\nold\nnew record\n")
    return b"# ledger\nold\nnew record\n"


def r_tracked_changed(r):
    _w(r.tr, b"# ledger\nold\n")
    r.seed_records()
    _w(r.rr, b"# ledger\nold\nnew record\n")
    _w(r.tr, b"# ledger\nold\nedited elsewhere\n")
    return None


def r_seed_null_tracked_still_absent(r):
    r.seed_records()
    _w(r.rr, b"# ledger\nfirst\n")
    return b"# ledger\nfirst\n"


def r_seed_null_tracked_appeared(r):
    r.seed_records()
    _w(r.rr, b"# ledger\nfirst\n")
    _w(r.tr, b"# ledger\npulled\n")
    return None


CASES = [
    # (setup, exit code, status word expected in `status`)
    (m_runtime_absent, 0, "absent"),
    (m_in_step, 0, "in step"),
    (m_seed_matches_copy, 0, "runtime ahead by 3 rows"),
    (m_tracked_changed_append, 0, "tracked changed since seed"),
    (m_no_seed, 2, "no readable seed"),
    (m_unreadable_seed, 2, "no readable seed"),
    (m_seed_null_tracked_still_absent, 0, "runtime ahead by 2 rows"),
    (m_seed_null_tracked_appeared, 0, "tracked changed since seed"),
    (m_runtime_edited_and_tracked_changed, 2, "merge refuses"),
    (m_tracked_deleted, 2, "merge refuses"),
    (m_tracked_torn_last_row, 0, "tracked changed since seed"),
    (m_seeded_torn_row, 0, "tracked changed since seed"),
    (m_header_mismatch, 2, "merge refuses"),
    (r_absent, 0, "absent"),
    (r_seed_matches_copy, 0, "runtime ahead"),
    (r_tracked_changed, 2, "tracked changed since seed"),
    (r_seed_null_tracked_still_absent, 0, "runtime ahead"),
    (r_seed_null_tracked_appeared, 2, "tracked changed since seed"),
]


@pytest.mark.parametrize("setup,code,word", CASES,
                         ids=[c[0].__name__ for c in CASES])
def test_the_rule_table(repo, capsys, setup, code, word):
    memory = setup.__name__.startswith("m_")
    tracked_path = repo.tm if memory else repo.tr
    runtime_path = repo.rm if memory else repo.rr
    want = setup(repo)
    before = _r(tracked_path)
    runtime_before = _r(runtime_path)
    seed_before = _r(rtp.seed_path(runtime_path))

    assert rr.main(["--repo", repo.root, "status"]) == 0
    label = "memory jugglebot" if memory else "records"
    line = [ln for ln in capsys.readouterr().out.splitlines()
            if ln.startswith(label)]
    assert line and word in line[0], line

    assert repo.merge() == code
    after = _r(tracked_path)
    if want is None:
        assert after == before            # nothing written to the tracked file
        if code == 2:
            assert _r(runtime_path) == runtime_before
            assert _r(rtp.seed_path(runtime_path)) == seed_before
        if runtime_before is None:
            assert not os.path.exists(runtime_path)
        return
    assert after == want
    assert _r(runtime_path) == want       # runtime rewritten to the merge
    seed = rtp.read_seed(runtime_path)
    assert seed["sha256"] == rr._sha(want)
    if memory:
        assert seed["rows"] == _rows(want)
        # tracked-before + runtime-new (the runtime rows after the seed's)
        seed_rows = json.loads(seed_before)["rows"]
        assert _rows(after) == _rows(before) + _rows(runtime_before) - seed_rows
    assert not os.path.exists(rr.journal_path(runtime_path))


def test_records_refusal_says_what_differs(repo, capsys):
    r_tracked_changed(repo)
    assert repo.merge() == 2
    err = capsys.readouterr().err
    assert "docs/records.md changed since" in err
    assert "+new record" in err and "-edited elsewhere" in err
    assert "--resolve-records" in err


@pytest.mark.parametrize("choice", ["keep-runtime", "keep-tracked"])
def test_the_owner_resolves_records(repo, choice):
    r_tracked_changed(repo)
    runtime, tracked = _r(repo.rr), _r(repo.tr)
    assert repo.merge("--resolve-records", choice) == 0
    want = runtime if choice == "keep-runtime" else tracked
    assert _r(repo.tr) == want and _r(repo.rr) == want
    assert repo.merge() == 0 and _r(repo.tr) == want


@pytest.mark.parametrize("setup", [m_seed_matches_copy, m_tracked_changed_append,
                                   m_seed_null_tracked_appeared,
                                   m_seeded_torn_row, r_seed_matches_copy])
def test_merge_then_merge_again_is_a_no_op(repo, setup):
    setup(repo)
    assert repo.merge() == 0
    paths = [repo.tm, repo.rm, rtp.seed_path(repo.rm),
             repo.tr, repo.rr, rtp.seed_path(repo.rr)]
    first = [_r(p) for p in paths]
    stamps = [os.stat(p).st_mtime_ns if os.path.exists(p) else None
              for p in paths]
    assert repo.merge() == 0
    assert [_r(p) for p in paths] == first           # byte-identical, same seed
    assert [os.stat(p).st_mtime_ns if os.path.exists(p) else None
            for p in paths] == stamps                # and nothing rewritten


def test_a_memory_merge_keeps_both_sets_exactly_once(repo):
    _w(repo.tm, _memory(*["s%d" % i for i in range(50)]))
    repo.seed_memory()
    sitting = ["n%d" % i for i in range(7)]
    for tag in sitting:                              # one append per throw
        _append_like_memory(repo.rm, _row(tag))
    pulled = ["p%d" % i for i in range(4)]
    _w(repo.tm, _memory(*(["s%d" % i for i in range(50)] + pulled)))
    tracked_before = _rows(_r(repo.tm))
    assert repo.merge() == 0
    rows = [r[0] for r in rtp.csv_data_rows(_r(repo.tm))]
    assert len(rows) == tracked_before + len(sitting)
    for tag in sitting + pulled:
        assert rows.count(tag) == 1, tag
    assert rows[-len(sitting):] == sitting           # appended, in order


def test_every_plant_is_covered_and_copies_back_to_its_local_file(repo):
    other_t = rtp.tracked_memory_path(repo.root, "bench2")
    other_r = rtp.runtime_memory_path(repo.root, "bench2")
    _w(other_t, _memory("a"))
    rtp.ensure_seeded(other_r, other_t, memory=True)
    _append_like_memory(other_r, _row("n1"))
    m_seed_matches_copy(repo)
    assert repo.merge() == 0
    assert _r(other_t) == _memory("a", "n1")
    assert _r(repo.tm) == _memory("a", "b", "n1", "n2", "n3")


def test_a_refused_file_does_not_block_the_others(repo):
    r_tracked_changed(repo)
    m_seed_matches_copy(repo)
    assert repo.merge() == 2
    assert _r(repo.tm) == _memory("a", "b", "n1", "n2", "n3")
    assert _r(repo.tr) == b"# ledger\nold\nedited elsewhere\n"


def test_a_live_sitting_refuses_unless_overridden(repo, capsys):
    m_seed_matches_copy(repo)
    before = _r(repo.tm)
    live = "4242 ros2 launch jugglebot jugglebot_launch.py"
    assert repo.merge(live=live) == rr.EXIT_LIVE == 3
    err = capsys.readouterr().err
    assert live in err and rr.LIVE_OVERRIDE in err and "BUG-85" in err
    assert _r(repo.tm) == before
    assert repo.merge(rr.LIVE_OVERRIDE, live=live) == 0
    assert _r(repo.tm) == _r(repo.rm) != before


def test_status_always_exits_zero_even_on_refusals(repo, capsys):
    m_no_seed(repo)
    r_tracked_changed(repo)
    assert rr.main(["--repo", repo.root, "status"]) == 0
    out = capsys.readouterr().out
    assert "no readable seed" in out and "tracked changed since seed" in out


# ── crash recovery: every step of the write order ────────────────────────────

@pytest.mark.parametrize("setup", [m_seed_matches_copy, m_tracked_changed_append])
@pytest.mark.parametrize("crash_after", [1, 2, 3, 4])
def test_a_crash_at_any_step_is_recovered_by_a_rerun(tmp_path, setup, crash_after):
    clean = Repo(str(tmp_path / "clean"))
    want = setup(clean)
    assert clean.merge() == 0

    r = Repo(str(tmp_path / "crash"))
    setup(r)
    entry = [e for e in rr.entries(r.root) if e.label == "memory jugglebot"][0]
    p = rr.plan(r.root, entry)
    with pytest.raises(RuntimeError):
        rr.commit(entry, p.merged, _r(r.tm), crash_after=crash_after)
    assert os.path.exists(rr.journal_path(r.rm))
    assert r.merge() == 0
    assert _r(r.tm) == want == _r(clean.tm)          # no row lost or doubled
    assert _r(r.rm) == want
    assert rtp.read_seed(r.rm)["rows"] == _rows(want)
    assert not os.path.exists(rr.journal_path(r.rm))
    assert r.merge() == 0 and _r(r.tm) == want


def test_a_sitting_after_a_crashed_merge_still_merges_once(repo):
    m_tracked_changed_append(repo)
    entry = [e for e in rr.entries(repo.root) if e.label == "memory jugglebot"][0]
    p = rr.plan(repo.root, entry)
    with pytest.raises(RuntimeError):
        rr.commit(entry, p.merged, _r(repo.tm), crash_after=3)
    _append_like_memory(repo.rm, _row("later"))     # the next Sitting
    assert repo.merge() == 0
    want = _memory("a", "b", "p1", "p2", "n1", "n2", "n3", "later")
    assert _r(repo.tm) == want and _r(repo.rm) == want


# ── unit 1 follow-up: the goal-start check is stats only ─────────────────────

def test_the_present_runtime_check_hashes_only_when_a_file_changed(repo, monkeypatch):
    _w(repo.tm, _memory("a"))
    repo.seed_memory()
    calls = []
    real = rtp.sha256_of
    monkeypatch.setattr(rtp, "sha256_of", lambda p: calls.append(p) or real(p))
    for _ in range(5):
        assert repo.seed_memory().action == rtp.KEPT
    assert len(calls) == 1
    _w(repo.tm, _memory("a", "p1"))                  # a pull: size changes
    assert repo.seed_memory().action == rtp.STALE
    assert len(calls) == 2
    assert repo.seed_memory().action == rtp.STALE
    assert len(calls) == 2


def test_an_empty_seed_is_not_rewritten_at_every_goal_start(repo):
    first = repo.seed_memory()
    assert first.action == rtp.SEEDED_EMPTY
    stamp = os.stat(rtp.seed_path(repo.rm)).st_mtime_ns
    again = repo.seed_memory()
    assert again.action == rtp.SEEDED_EMPTY and again.rows == 0
    assert os.stat(rtp.seed_path(repo.rm)).st_mtime_ns == stamp
