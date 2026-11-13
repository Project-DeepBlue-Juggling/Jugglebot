"""tools/night_shift_landed.py: every commit a night lands on main is gated (closeout C1).

The landed check is the single enforcement point binding a push to ``main`` to a MERGE
verdict record (``night_shift_gate.py --record``). Each test builds a repository under
tmp_path, lands one shape of commit between FROM and TO, and asserts the rule's
answer: the four failures it exists for (a merge after a HOLD, a merge with no gate
run, a log from another tip, an evil merge) and the shapes a normal night produces
(exempt map/report writes, a gated fast-forward, a ``--no-ff`` merge, a branch that
merged ``main`` in for its re-gate). The EXEMPT list is pinned against what
``mapkit gen`` writes. No ports, no fixed paths, safe under xdist.
"""
from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
TOOL = os.path.join(ROOT, "tools", "night_shift_landed.py")
sys.path.insert(0, os.path.join(ROOT, "tools"))
import night_shift_gate as gate  # noqa: E402
import night_shift_landed as landed  # noqa: E402


@pytest.fixture(autouse=True)
def _git_env(tmp_path, monkeypatch):
    monkeypatch.setenv("HOME", str(tmp_path))
    monkeypatch.setenv("GIT_CONFIG_GLOBAL", str(tmp_path / "gitconfig"))
    monkeypatch.setenv("GIT_CONFIG_NOSYSTEM", "1")
    for k in ("GIT_AUTHOR_NAME", "GIT_COMMITTER_NAME"):
        monkeypatch.setenv(k, "t")
    for k in ("GIT_AUTHOR_EMAIL", "GIT_COMMITTER_EMAIL"):
        monkeypatch.setenv(k, "t@example.com")


def _git(cwd, *args):
    return subprocess.run(["git"] + list(args), cwd=str(cwd), check=True,
                          capture_output=True, text=True).stdout.strip()


class Repo:
    def __init__(self, tmp_path):
        self.root = tmp_path / "repo"
        self.root.mkdir()
        self.verdicts = tmp_path / "verdicts"
        _git(self.root, "init", "-q")
        _git(self.root, "checkout", "-q", "-b", "main")
        self.write("src/a.py", "A = 1\n")
        self.write("map/bug-x.md", "x\n")
        self.start = self.commit("seed")

    def write(self, rel, text):
        p = self.root / rel
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(text)

    def commit(self, msg="edit", allow_empty=False):
        _git(self.root, "add", "-A")
        _git(self.root, "commit", "-q", "-m", msg, *(["--allow-empty"] if allow_empty else []))
        return _git(self.root, "rev-parse", "HEAD")

    def head(self):
        return _git(self.root, "rev-parse", "HEAD")

    def record(self, tip, verdict="MERGE", **over):
        self.verdicts.mkdir(exist_ok=True)
        rec = {"schema": 1, "verdict": verdict, "item": "BUG-7", "branch": "b", "tip": tip}
        rec.update(over)
        (self.verdicts / ("%s.%s.json" % (tip, verdict))).write_text(json.dumps(rec))

    def branch(self, name="night-shift/x"):
        _git(self.root, "checkout", "-q", "-b", name)

    def main(self):
        _git(self.root, "checkout", "-q", "main")

    def check(self, to=None):
        return landed.check(str(self.root), self.start, to or "main", str(self.verdicts))

    def cli(self, *extra):
        return subprocess.run([sys.executable, TOOL, "--repo", str(self.root), "--from", self.start,
                               "--to", "main", "--verdicts", str(self.verdicts)] + list(extra),
                              capture_output=True, text=True)


@pytest.fixture
def repo(tmp_path):
    return Repo(tmp_path)


def ungated(res):
    return [u["why"] for u in res["ungated"]]


def test_nothing_landed_passes(repo):
    res = repo.check()
    assert (res["gated"], res["exempt"], res["ungated"]) == (0, 0, [])
    r = repo.cli()
    assert r.returncode == 0 and r.stdout.strip() == "landed: gated=0 exempt=0 ungated=0"


def test_exempt_and_empty_commits_pass(repo):
    for rel in ("map/bug-x.md", "docs/night-shift/2026-11-13.md", "logbook/2026-11-13-x.md",
                "plans/active/INDEX.md", "docs/runbook.md", "docs/project-map.md", "docs/glossary.md"):
        repo.write(rel, "new\n")
        repo.commit("docs")
    repo.commit("empty", allow_empty=True)
    res = repo.check()
    assert res["exempt"] == 8 and res["ungated"] == []


def test_a_code_commit_with_no_record_is_ungated(repo):
    """A merge with no gate run: the commit, author and paths are named; exit 1."""
    repo.write("src/a.py", "A = 2\n")
    repo.write("map/bug-x.md", "closed\n")
    c = repo.commit("fix")
    res = repo.check()
    assert res["ungated"] == [{"commit": c, "author": "t <t@example.com>",
                               "why": "no MERGE record covers it", "paths": ["src/a.py"]}]
    r = repo.cli()
    assert r.returncode == 1, r.stdout + r.stderr
    lines = r.stdout.splitlines()
    assert lines[0] == "landed: gated=0 exempt=0 ungated=1"
    assert lines[1] == "UNGATED %s t <t@example.com> no MERGE record covers it: src/a.py" % c[:12]


def test_a_recorded_fast_forward_merge_passes(repo):
    repo.branch()
    repo.write("src/a.py", "A = 2\n")
    repo.commit("fix")
    repo.write("src/a.py", "A = 3\n")
    tip = repo.commit("chore(audit)")
    repo.record(tip)
    repo.main()
    _git(repo.root, "merge", "-q", "--ff-only", "night-shift/x")
    res = repo.check()
    assert res["gated"] == 2 and res["ungated"] == []
    assert repo.cli().returncode == 0


def test_merge_after_a_hold_is_ungated(repo):
    repo.branch()
    repo.write("src/a.py", "A = 2\n")
    tip = repo.commit("fix")
    repo.record(tip, verdict="HOLD")
    repo.main()
    _git(repo.root, "merge", "-q", "--ff-only", "night-shift/x")
    assert ungated(repo.check()) == ["no MERGE record covers it"]


def test_a_record_for_another_tip_does_not_cover_a_later_commit(repo):
    """An EXIT=0 log (and so a MERGE) for an earlier tip; a commit after it merged."""
    repo.branch()
    repo.write("src/a.py", "A = 2\n")
    gated_tip = repo.commit("fix")
    repo.record(gated_tip)
    repo.write("src/b.py", "B = 1\n")
    late = repo.commit("after the gate")
    repo.main()
    _git(repo.root, "merge", "-q", "--ff-only", "night-shift/x")
    res = repo.check()
    assert res["gated"] == 1 and [u["commit"] for u in res["ungated"]] == [late]


def test_a_hold_after_the_merge_does_not_unmerge(repo):
    """The gate re-run on a merged tip (a resume, a re-check) HOLDs; the MERGE record stands."""
    import night_shift_gate as g
    repo.branch()
    repo.write("src/a.py", "A = 2\n")
    tip = repo.commit("fix")
    repo.main()
    _git(repo.root, "merge", "-q", "--ff-only", "night-shift/x")
    for verdict, reasons in (("MERGE", []), ("HOLD", ["[a] stale"])):
        res = g.GateResult(verdict=verdict, item="BUG-7", branch="night-shift/x", tip=tip, reasons=reasons)
        g.write_record(str(repo.verdicts), res, str(repo.root), "main", None, None)
    assert sorted(os.listdir(str(repo.verdicts))) == [tip + ".HOLD.json", tip + ".MERGE.json"]
    assert repo.check()["ungated"] == [] and repo.cli().returncode == 0


def test_a_recorded_no_ff_merge_passes(repo):
    repo.branch()
    repo.write("src/a.py", "A = 2\n")
    tip = repo.commit("fix")
    repo.record(tip)
    repo.main()
    repo.write("map/bug-x.md", "night's map write on main\n")
    repo.commit("map: x")
    _git(repo.root, "merge", "-q", "--no-ff", "-m", "merge(night-shift)", "night-shift/x")
    res = repo.check()
    assert (res["gated"], res["exempt"], res["ungated"]) == (2, 1, [])


def test_an_evil_merge_is_flagged(repo):
    """A merge commit edited by hand: content no gate saw rides in with a gated tip."""
    repo.branch()
    repo.write("src/a.py", "A = 2\n")
    tip = repo.commit("fix")
    repo.record(tip)
    repo.main()
    repo.write("map/bug-x.md", "moved\n")
    repo.commit("map: x")
    _git(repo.root, "merge", "-q", "--no-ff", "--no-commit", "night-shift/x")
    repo.write("src/evil.py", "EVIL = 1\n")
    m = repo.commit("merge(night-shift)")
    res = repo.check()
    assert res["ungated"] == [{"commit": m, "author": "t <t@example.com>",
                               "why": "merge carries content of its own (evil merge)",
                               "paths": ["src/evil.py"]}]


def test_a_merge_of_an_unrecorded_branch_is_flagged_twice(repo):
    repo.branch()
    repo.write("src/a.py", "A = 2\n")
    repo.commit("fix")
    repo.main()
    repo.write("map/bug-x.md", "moved\n")
    repo.commit("map: x")
    _git(repo.root, "merge", "-q", "--no-ff", "-m", "merge", "night-shift/x")
    assert sorted(ungated(repo.check())) == ["merge of no MERGE-recorded tip", "no MERGE record covers it"]


def test_a_branch_that_merged_main_for_its_regate_passes(repo):
    """The re-gate (protocol 2l): the branch merges origin/main, the gate judges that tip."""
    repo.branch()
    repo.write("src/a.py", "A = 2\n")
    repo.commit("fix")
    repo.main()
    repo.write("src/c.py", "C = 1\n")
    repo.commit("a day's work")
    _git(repo.root, "checkout", "-q", "night-shift/x")
    _git(repo.root, "merge", "-q", "--no-edit", "main")
    repo.record(repo.head())
    repo.main()
    _git(repo.root, "merge", "-q", "--ff-only", "night-shift/x")
    res = repo.check()
    # The day's commit is covered too: it is inside the tree the gate judged (the suite ran
    # on it), though the gate's (f) diff starts after it. A documented gap, not a pass by luck.
    assert res["ungated"] == [] and res["gated"] == 3


def test_malformed_records_are_ignored(repo):
    repo.write("src/a.py", "A = 2\n")
    c = repo.commit("fix")
    repo.record(c, schema=2)
    assert len(repo.check()["ungated"]) == 1
    (repo.verdicts / ("%s.MERGE.json" % c)).write_text("{not json")
    assert len(repo.check()["ungated"]) == 1
    (repo.verdicts / ("%s.MERGE.json" % c)).unlink()
    repo.record(c)
    os.rename(str(repo.verdicts / ("%s.MERGE.json" % c)), str(repo.verdicts / "other.json"))
    assert len(repo.check()["ungated"]) == 1
    os.rename(str(repo.verdicts / "other.json"), str(repo.verdicts / ("%s.MERGE.json" % c)))
    assert repo.check()["ungated"] == []


def test_an_unresolvable_range_is_an_error_not_a_pass(repo):
    r = subprocess.run([sys.executable, TOOL, "--repo", str(repo.root), "--from", "0" * 40,
                        "--to", "main", "--verdicts", str(repo.verdicts)], capture_output=True, text=True)
    assert r.returncode == 2 and r.stdout.startswith("landed: error"), r.stdout


# --- the EXEMPT pin ----------------------------------------------------------
def test_exempt_is_pinned():
    assert landed.EXEMPT == (
        "map/*", "docs/night-shift/*", "logbook/*",
        "plans/active/INDEX.md", "plans/parked/INDEX.md", "plans/archived/INDEX.md",
        "docs/project-map.md", "docs/glossary.md", "docs/runbook.md",
    )
    assert landed.GEN_NOT_EXEMPT == ("CLAUDE.md",)


def test_exempt_covers_every_path_mapkit_gen_writes(tmp_path):
    """Run ``mapkit gen --check`` on a copy whose every output is stale: what it would
    write is exactly the generated part of EXEMPT plus GEN_NOT_EXEMPT."""
    from mapkit import gen_glossary
    work = tmp_path / "gen"
    work.mkdir()
    for d in ("map", "plans"):
        shutil.copytree(os.path.join(ROOT, d), str(work / d), ignore=shutil.ignore_patterns("INDEX.md"))
    (work / "docs").mkdir()
    shutil.copy(os.path.join(ROOT, "GLOSSARY.md"), str(work / "GLOSSARY.md"))
    text = open(os.path.join(ROOT, "CLAUDE.md"), encoding="utf-8").read()
    s = text.index(gen_glossary.START_MARKER) + len(gen_glossary.START_MARKER)
    e = text.index(gen_glossary.END_MARKER, s)
    (work / "CLAUDE.md").write_text(text[:s] + "\nstale\n" + text[e:], encoding="utf-8")
    r = subprocess.run([sys.executable, "-m", "mapkit", "gen", "--repo", str(work), "--check"],
                       capture_output=True, text=True, cwd=ROOT)
    written = sorted(l[len("would change "):] for l in r.stdout.splitlines()
                     if l.startswith("would change "))
    assert written, r.stdout + r.stderr
    generated = [p for p in landed.EXEMPT if "*" not in p]
    assert written == sorted(generated + list(landed.GEN_NOT_EXEMPT))
    assert not any(landed.is_exempt(p) for p in landed.GEN_NOT_EXEMPT)


def test_no_exempt_path_is_one_the_gate_forbids():
    """An exempt path skips the gate; it must never be one the gate would refuse."""
    tracked = _git(ROOT, "ls-files").splitlines()
    import fnmatch
    both = [p for p in tracked if landed.is_exempt(p)
            and any(fnmatch.fnmatchcase(p, f) for f in gate.FORBIDDEN_PATHS)]
    assert both == []
    for p in landed.GEN_NOT_EXEMPT:
        assert any(fnmatch.fnmatchcase(p, f) for f in gate.FORBIDDEN_PATHS)
