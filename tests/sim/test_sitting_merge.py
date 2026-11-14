"""tools/sitting_merge.sh: the debrief lands a passed Item's held branch only through the gate.

The script is the one path from a held ``night-shift/<date>-<ID>`` branch to ``main`` after a
Sitting (closeout C10): ``prepare`` brings the branch up to date with ``main`` in its own
worktree and runs the full suite; ``land`` runs the base's gate with ``--waive c`` and pushes
exactly the gated tip. Every test drives the real script, the real gate and the real ``mapkit``
against a temporary bare ``origin``, a seed clone that plays the other writers and a "live"
clone the script works from; the suite is a stub ``run_tests.sh`` whose exit code the test
sets. tmp_path only: no ports, no fixed paths (the suite lock is under tmp_path too, so a
test run inside ``flock /tmp/jugglebot-tests.lock`` cannot deadlock on itself).
"""
from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

from mapkit.items import write_item

REPO = Path(__file__).resolve().parents[2]
SCRIPT = REPO / "tools" / "sitting_merge.sh"
ITEM = "BUG-7"
BRANCH = "night-shift/2026-11-13-BUG-7"
BUG = "map/bug-wobble.md"
SECTION = "## Sitting\n\n- setup: level\n- why: w\n- do: d\n- pass: p\n- record: r\n- minutes: 5\n"


def _meta(kind, title, n, **extra):
    meta = {"id": "%s-%d" % ({"bug": "BUG", "pathway": "PWAY"}[kind], n), "kind": kind,
            "title": title, "status": "in_progress" if kind == "bug" else "active",
            "created": "2026-10-01", "updated": "2026-10-01T00:00:00Z", "priority": "medium"}
    meta.update(extra)
    return meta


def write_bug(root, body="Body.\n\n" + SECTION):
    # hardware: robot plus a ## Sitting section: condition (c) fails, so only --waive c merges it.
    write_item(str(root / BUG), _meta("bug", "Wobble", 7, hardware="robot", radius="local",
                                      effort="s", owner="none", diagnosis="confirmed"), body)


def logbook_entry(root, slug, title):
    (root / "logbook" / (slug + ".md")).write_text(
        "---\ntitle: %s\ndate: %s\nsubsystem: tools\ntype: feature\nstatus: complete\n---\n\nx\n"
        % (title, slug[:10]))


class Fx:
    def __init__(self, tmp_path):
        self.tmp = tmp_path
        for name in ("gitconfig", ".gitconfig"):  # this git (2.25) reads HOME, not GIT_CONFIG_GLOBAL
            (tmp_path / name).write_text("[user]\n\tname = t\n\temail = t@t\n")
        self.verdicts = tmp_path / "verdicts"
        self.work = tmp_path / "work"
        self.env = dict(os.environ, HOME=str(tmp_path), GIT_CONFIG_GLOBAL=str(tmp_path / "gitconfig"),
                        GIT_CONFIG_NOSYSTEM="1", NIGHT_SHIFT_VERDICTS=str(self.verdicts),
                        SITTING_MERGE_LOCK=str(tmp_path / "suite.lock"),
                        SITTING_MERGE_PYTHON=sys.executable, STUB_SUITE_EXIT="0",
                        LIVE_SESSION_RE="^no-such-process-sitting-merge-test$",
                        ROS_DISTRO="test", VIRTUAL_ENV=os.environ.get("VIRTUAL_ENV", "test"))
        self.env.pop("JUGGLEBOT_REPO", None)
        self.origin = tmp_path / "origin.git"
        self.seed = tmp_path / "seed"
        self.live = tmp_path / "live"
        self.git(tmp_path, "init", "-q", "--bare", str(self.origin))
        self.git(self.origin, "symbolic-ref", "HEAD", "refs/heads/main")
        s = self.seed
        s.mkdir()
        self.git(s, "init", "-q")
        self.git(s, "checkout", "-q", "-b", "main")
        self.git(s, "remote", "add", "origin", str(self.origin))
        for d in ("map", "src", "logbook", "docs", "config", "tools/lib",
                  "plans/active", "plans/parked", "plans/archived"):
            (s / d).mkdir(parents=True)
        write_item(str(s / "map" / "pathway-steady.md"),
                   _meta("pathway", "Steady", 1, destination="It is steady."), "Body\n")
        write_bug(s)
        (s / "src" / "thing.py").write_text("X = 1\n")
        (s / "config" / "hardware_config.yaml").write_text("a: 1\n")
        logbook_entry(s, "2026-11-01-a", "Entry A")
        shutil.copytree(str(REPO / "mapkit"), str(s / "mapkit"),
                        ignore=shutil.ignore_patterns("__pycache__"))
        shutil.copy(str(REPO / "tools" / "night_shift_gate.py"), str(s / "tools"))
        for name in ("GLOSSARY.md", "CLAUDE.md"):  # mapkit gen reads the glossary, writes the term block
            shutil.copy(str(REPO / name), str(s / name))
        shutil.copy(str(REPO / "tools" / "lib" / "live_session.sh"), str(s / "tools" / "lib"))
        # The base's network fence and the suite are stubs; the gate, mapkit and guard are real.
        (s / "tools" / "night_shift_nonet.sh").write_text('#!/usr/bin/env bash\necho "nonet(base)"\nexec "$@"\n')
        (s / "run_tests.sh").write_text(
            '#!/usr/bin/env bash\necho "stub suite $*"\nexit "${STUB_SUITE_EXIT:-0}"\n')
        for f in ("tools/night_shift_nonet.sh", "run_tests.sh"):
            os.chmod(str(s / f), 0o755)
        (s / ".gitignore").write_text("__pycache__/\n")
        self.gen(s)
        self.commit(s, "seed")
        self.git(s, "push", "-q", "origin", "main")
        self.git(tmp_path, "clone", "-q", str(self.origin), str(self.live))

    # --- plumbing ---------------------------------------------------------------
    def git(self, cwd, *args):
        return subprocess.run(["git"] + list(args), cwd=str(cwd), env=self.env, check=True,
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                              universal_newlines=True).stdout.strip()

    def gen(self, root):
        subprocess.run([sys.executable, "-m", "mapkit", "gen", "--repo", str(root)], cwd=str(root),
                       env=self.env, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    def commit(self, root, msg):
        self.git(root, "add", "-A")
        self.git(root, "commit", "-q", "-m", msg)
        return self.git(root, "rev-parse", "HEAD")

    def branch(self, edits, name=BRANCH, regen=False):
        """Push a held branch off the current origin/main with these {path: text} edits."""
        self.git(self.seed, "checkout", "-q", "-B", name, "main")
        for path, text in edits.items():
            (self.seed / path).write_text(text)
        if regen:
            self.gen(self.seed)
        self.commit(self.seed, "work " + name)
        self.git(self.seed, "push", "-q", "-f", "origin", name)
        self.git(self.seed, "checkout", "-q", "main")

    def main_moves(self, edits, regen=False, msg="main moves"):
        self.git(self.seed, "checkout", "-q", "main")
        self.git(self.seed, "pull", "-q", "--ff-only", "origin", "main")
        for path, text in edits.items():
            (self.seed / path).write_text(text)
        if regen:
            self.gen(self.seed)
        self.commit(self.seed, msg)
        self.git(self.seed, "push", "-q", "origin", "main")

    def run(self, cmd, *extra, branch=BRANCH):
        return subprocess.run(["bash", str(SCRIPT), cmd, "--branch", branch, "--item", ITEM,
                               "--work", str(self.work), "--repo", str(self.live)] + list(extra),
                              env=self.env, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                              universal_newlines=True)

    def origin_sha(self, ref="main"):
        return self.git(self.origin, "rev-parse", ref)

    def remote_has(self, name):
        return bool(self.git(self.origin, "branch", "--list", name))

    def gate_log(self):
        return (self.work / ("gate-%s.log" % ITEM)).read_text()

    def logged_head(self):
        return [ln[5:] for ln in self.gate_log().splitlines() if ln.startswith("HEAD=")][0]

    def audit(self, open_findings=0, head=None):
        (self.work / ("audit-%s.md" % ITEM)).write_text(
            "No findings.\n\nAudited HEAD: %s\nBehaviour-affecting findings open: %d\n"
            % (head or self.logged_head(), open_findings))

    def wt(self):
        return self.work / ("wt-%s" % ITEM)

    def jsonl(self):
        p = self.live / "temp" / "reports" / "sitting-merge.jsonl"
        return [json.loads(x) for x in p.read_text().splitlines()] if p.exists() else []


def refused(r, code):
    assert r.returncode == code, (r.returncode, r.stdout, r.stderr)
    assert ("REFUSED %d " % code) in r.stderr, r.stderr


@pytest.fixture
def fx(tmp_path):
    return Fx(tmp_path)


def test_script_is_valid_bash_and_strict():
    assert subprocess.run(["bash", "-n", str(SCRIPT)]).returncode == 0
    assert "set -euo pipefail" in SCRIPT.read_text()


def test_happy_path_lands_exactly_the_gated_tip(fx):
    fx.branch({"src/thing.py": "X = 2\n"})
    fx.main_moves({"src/other.py": "Y = 1\n"})  # main moved since the night: prepare merges it in
    base = fx.origin_sha()
    r = fx.run("prepare")
    assert r.returncode == 0, r.stderr
    lines = fx.gate_log().splitlines()
    tip = fx.logged_head()
    assert lines[0] == "HEAD=" + tip and lines[1] == "BRANCH=" + BRANCH
    assert lines[-3:] == ["HEAD=" + tip, "BRANCH=" + BRANCH, "EXIT=0"]
    assert "stub suite --full" in lines and "nonet(base)" in lines  # under the base's fence
    assert fx.git(fx.wt(), "merge-base", "--is-ancestor", base, "HEAD") == ""
    assert (fx.work / ("diff-%s.patch" % ITEM)).read_text().count("X = 2") == 1
    assert fx.origin_sha("refs/heads/" + BRANCH) != tip  # the flown branch is untouched
    fx.audit()
    r = fx.run("land")
    assert r.returncode == 0, r.stderr
    assert "MERGE (c waived)" in r.stdout and ("LANDED %s %s" % (ITEM, tip)) in r.stdout
    assert fx.origin_sha() == tip  # a fast-forward: main IS the gated tree
    record = json.loads((fx.verdicts / ("%s.MERGE.json" % tip)).read_text())
    assert record["verdict"] == "MERGE" and record["waived"] == ["c"] and record["base_sha"] == base
    assert not fx.remote_has(BRANCH)
    assert not fx.wt().exists()
    assert not fx.git(fx.live, "branch", "--list", BRANCH)
    rows = fx.jsonl()
    assert len(rows) == 1 and rows[0]["tip"] == tip and rows[0]["item"] == ITEM
    # The night's landed check counts every commit main gained as gated by the record.
    out = subprocess.run([sys.executable, str(REPO / "tools" / "night_shift_landed.py"),
                          "--repo", str(fx.live), "--from", base, "--to", tip,
                          "--verdicts", str(fx.verdicts)], env=fx.env, stdout=subprocess.PIPE,
                         stderr=subprocess.PIPE, universal_newlines=True)
    assert out.returncode == 0 and "ungated=0" in out.stdout, out.stdout + out.stderr

    # Re-running land is a no-op that says so.
    r2 = fx.run("land")
    assert r2.returncode == 0, r2.stderr
    assert ("ALREADY LANDED %s %s" % (ITEM, tip)) in r2.stdout
    assert fx.origin_sha() == tip and len(fx.jsonl()) == 1


def test_hold_keeps_the_branch_and_the_worktree(fx):
    fx.branch({"src/thing.py": "X = 2\n"})
    base = fx.origin_sha()
    assert fx.run("prepare").returncode == 0
    fx.audit(open_findings=1)
    r = fx.run("land")
    refused(r, 13)
    assert "HOLD" in r.stdout and "[b]" in r.stdout
    assert fx.origin_sha() == base
    assert fx.remote_has(BRANCH) and fx.wt().is_dir()
    assert fx.jsonl() == []
    assert (fx.verdicts / ("%s.HOLD.json" % fx.logged_head())).exists()


def test_a_commit_after_the_suite_is_refused(fx):
    fx.branch({"src/thing.py": "X = 2\n"})
    base = fx.origin_sha()
    assert fx.run("prepare").returncode == 0
    fx.audit()
    (fx.wt() / "src" / "thing.py").write_text("X = 3\n")
    refused(fx.run("land"), 10)  # uncommitted
    fx.commit(fx.wt(), "an edit after the suite")
    fx.audit(head=fx.git(fx.wt(), "rev-parse", "HEAD"))  # even an audit of the new tip
    refused(fx.run("land"), 9)
    assert fx.origin_sha() == base and fx.remote_has(BRANCH)


def test_a_missing_audit_is_refused(fx):
    fx.branch({"src/thing.py": "X = 2\n"})
    base = fx.origin_sha()
    assert fx.run("prepare").returncode == 0
    refused(fx.run("land"), 11)
    assert fx.origin_sha() == base


def test_a_red_suite_is_refused_at_prepare_and_land(fx):
    fx.branch({"src/thing.py": "X = 2\n"})
    fx.env["STUB_SUITE_EXIT"] = "3"
    refused(fx.run("prepare"), 7)
    assert fx.gate_log().splitlines()[-1] == "EXIT=3"
    fx.audit()
    refused(fx.run("land"), 7)


def test_a_frozen_path_branch_is_refused_with_no_override(fx):
    fx.branch({"config/hardware_config.yaml": "a: 2\n", "src/thing.py": "X = 2\n"})
    r = fx.run("prepare")
    refused(r, 4)
    assert "config/hardware_config.yaml" in r.stderr and "day Session" in r.stderr
    assert not fx.wt().exists() and not (fx.work / ("gate-%s.log" % ITEM)).exists()
    for flag in ("--force", "--waive", "--allow-frozen"):
        refused(fx.run("prepare", flag, "f"), 2)


def test_main_moved_since_prepare_is_refused(fx):
    fx.branch({"src/thing.py": "X = 2\n"})
    assert fx.run("prepare").returncode == 0
    fx.audit()
    fx.main_moves({"src/other.py": "Y = 1\n"})
    moved = fx.origin_sha()
    r = fx.run("land")
    refused(r, 12)
    assert "re-run prepare" in r.stderr
    assert fx.origin_sha() == moved and fx.remote_has(BRANCH)
    assert not list(fx.tmp.glob("verdicts/*.MERGE.json"))


def test_the_items_own_map_conflict_takes_mains_copy_and_lands(fx):
    """Main is the authority for the Item's map state; both sides touch it (its updated: line)."""
    fx.branch({BUG: (fx.seed / BUG).read_text().replace("Body.", "Body, branch."),
               "src/thing.py": "X = 2\n"})
    fx.main_moves({BUG: (fx.seed / BUG).read_text().replace("Body.", "Body, main.")})
    r = fx.run("prepare")
    assert r.returncode == 0, r.stderr
    assert ("TOOK MAIN's copy of %s" % BUG) in r.stdout
    assert ("TOOK MAIN's copy of %s" % BUG) in fx.gate_log().splitlines()[2]  # in the header
    assert "Body, main." in (fx.wt() / BUG).read_text()
    assert "Body, branch." in (fx.work / ("branch-copy-%s.md" % ITEM)).read_text()
    fx.audit()
    assert fx.run("land").returncode == 0
    assert fx.origin_sha() == fx.logged_head()
    assert "Body, main." in fx.git(fx.origin, "show", "main:" + BUG)


def test_a_map_conflict_stops_prepare(fx):
    """Another Item's map file in conflict is no mechanical answer: exit 5."""
    other = "map/bug-other.md"
    write_item(str(fx.seed / other), _meta("bug", "Other", 8), "Other body.\n")
    fx.main_moves({}, msg="add BUG-8")
    fx.branch({other: (fx.seed / other).read_text().replace("Other body.", "Branch body.")})
    fx.main_moves({other: (fx.seed / other).read_text().replace("Other body.", "Main body.")})
    r = fx.run("prepare")
    refused(r, 5)
    assert other in r.stderr and "TOOK MAIN" not in r.stdout
    assert not fx.wt().exists() and not (fx.work / ("gate-%s.log" % ITEM)).exists()


def test_a_generated_page_conflict_is_regenerated(fx):
    logbook_entry(fx.seed, "2026-11-02-b", "Entry B")
    fx.branch({"src/thing.py": "X = 2\n"}, regen=True)  # the branch adds B and its INDEX row
    assert not (fx.seed / "logbook" / "2026-11-02-b.md").exists()  # B lives on the branch only
    logbook_entry(fx.seed, "2026-11-03-c", "Entry C")
    fx.main_moves({}, regen=True)  # main adds C and its INDEX row: logbook/INDEX.md conflicts
    r = fx.run("prepare")
    assert r.returncode == 0, r.stderr
    assert "regenerated: logbook/INDEX.md" in r.stderr
    index = (fx.wt() / "logbook" / "INDEX.md").read_text()
    assert "Entry B" in index and "Entry C" in index
    fx.audit()
    assert fx.run("land").returncode == 0
    assert fx.origin_sha() == fx.logged_head()


def test_an_empty_branch_and_the_candidate_branch_are_refused(fx):
    fx.git(fx.seed, "push", "-q", "origin", "main:refs/heads/" + BRANCH)  # nothing beyond main
    refused(fx.run("prepare"), 6)
    refused(fx.run("prepare", branch="night-shift/sitting-2026-11-13"), 2)
    refused(fx.run("prepare", branch="night-shift/2026-11-13-BUG-8"), 2)  # another Item's branch


def test_land_without_prepare_is_refused(fx):
    fx.branch({"src/thing.py": "X = 2\n"})
    refused(fx.run("land"), 16)


def test_machinery_comes_from_the_base(fx):
    """The gate, the network fence and the live-session guard run as extracted from origin/main,
    never from the live checkout's working tree (nor the branch's, whose edit to them is frozen)."""
    for rel, text in (("tools/night_shift_gate.py", "print('MERGE')\n"),
                      ("tools/night_shift_nonet.sh", '#!/usr/bin/env bash\necho LIVE-NONET\nexec "$@"\n'),
                      ("tools/lib/live_session.sh", "live_session() { return 0; }\n")):
        (fx.live / rel).write_text(text)
    fx.branch({"src/thing.py": "X = 2\n"})
    base = fx.origin_sha()
    assert fx.run("prepare").returncode == 0
    assert "nonet(base)" in fx.gate_log() and "LIVE-NONET" not in fx.gate_log()
    fx.audit(open_findings=1)
    refused(fx.run("land"), 13)
    assert fx.origin_sha() == base
