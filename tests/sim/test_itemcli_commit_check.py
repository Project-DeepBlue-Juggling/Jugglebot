"""``mapkit item * --commit`` runs the map-write check before it commits (close-out C13).

The four checks live in ``mapkit.checks`` (the four map tests call the same
functions). Contract pinned here: a commit that would break any of them is
refused with exit 4, nothing is committed, and the write stays in the working
tree; a clean one commits; a ticket filed with ``--parent`` is exempt from the
similar-title refusal only; the batch skip commit goes through the check too.
Everything runs in a ``tmp_path`` git repo (no remote: ``--no-push``).
"""
from __future__ import annotations

import json
import os
import subprocess

import pytest

from mapkit import checks
from mapkit.__main__ import main
from mapkit.items import load_item, write_item

RATED = {"effort": "s", "hardware": "none", "owner": "none", "radius": "local", "area": "docs"}
UPD = "2026-10-01T00:00:00Z"


def _git(cwd, *a):
    return subprocess.run(["git"] + list(a), cwd=str(cwd), check=True, stdout=subprocess.PIPE,
                          stderr=subprocess.DEVNULL, universal_newlines=True).stdout


def _put(root, kind, slug, status="backlog", body="Body text\n", title=None, **extra):
    meta = {"kind": kind, "title": title or ("T " + slug), "status": status,
            "created": "2026-10-01", "updated": UPD, "priority": "medium", "filed_by": "claude"}
    if status != "inbox":
        meta.update(RATED)
        if kind == "bug":
            meta["diagnosis"] = "unknown"
    meta.update(extra)
    write_item(str(root / "map" / ("%s-%s.md" % (kind, slug))), meta, body)


@pytest.fixture
def repo(tmp_path, monkeypatch):
    monkeypatch.setenv("HOME", str(tmp_path))
    monkeypatch.setenv("GIT_CONFIG_GLOBAL", str(tmp_path / "gitconfig"))
    monkeypatch.setenv("GIT_CONFIG_NOSYSTEM", "1")
    root = tmp_path / "work"
    (root / "map").mkdir(parents=True)
    (root / "docs").mkdir()
    (root / "tests" / "sim").mkdir(parents=True)
    (root / "tests" / "sim" / "avoid_words_checked.txt").write_text("reload\n")
    (root / "GLOSSARY.md").write_text("# G\n\n## Language\n\n### Robot\n\n**Hand**: The cup.\n")
    for b in ("active", "parked", "archived"):
        (root / "plans" / b).mkdir(parents=True)
    write_item(str(root / "map" / "pathway-p.md"),
               {"kind": "pathway", "title": "P", "status": "active", "created": "2026-10-01",
                "destination": "reach the goal", **RATED}, "Body\n")
    _put(root, "bug", "b")
    _put(root, "bug", "inb", status="inbox")
    for cmd in (["init", "-q"], ["config", "user.name", "T"], ["config", "user.email", "t@e.x"],
                ["add", "."], ["commit", "-qm", "seed"]):
        _git(root, *cmd)
    assert checks.run_all(str(root)) == []
    return root


def _run(root, *argv):
    return main(list(argv) + ["--repo", str(root)])


def _refused(root, capsys, *argv):
    head = _git(root, "rev-parse", "HEAD")
    assert _run(root, *argv) == 4
    err = capsys.readouterr().err
    assert _git(root, "rev-parse", "HEAD") == head, "a refused commit must not commit"
    assert "left in the working tree" in err
    return err


def test_clean_commit_succeeds(repo):
    assert _run(repo, "item", "set-status", "bug-b", "in_progress", "--note", "started",
                "--owner", "al", "--commit", "--no-push") == 0
    assert _git(repo, "log", "-1", "--format=%s").startswith("map: in_progress")
    assert _git(repo, "status", "--porcelain", "map", "docs/project-map.md") == ""


def test_whole_map_load_failure_is_refused(repo, capsys, monkeypatch):
    # gen_docs loads the map too, so a broken file usually dies earlier with exit 2; pin that
    # the commit path itself still runs check_map_loads by making the file fail only there.
    monkeypatch.setattr(checks, "check_map_loads", lambda r: ["map/bug-x.md: unreadable"])
    err = _refused(repo, capsys, "item", "set-status", "bug-b", "in_progress", "--note", "n",
                   "--owner", "al", "--commit", "--no-push")
    assert "[map-loads] map/bug-x.md: unreadable" in err


def test_real_malformed_sibling_is_never_committed(repo, capsys):
    (repo / "map" / "bug-broken.md").write_text("---\nkind: bug\n---\n")
    head = _git(repo, "rev-parse", "HEAD")
    assert _run(repo, "item", "set-status", "bug-b", "in_progress", "--note", "n",
                "--owner", "al", "--commit", "--no-push") == 2
    assert _git(repo, "rev-parse", "HEAD") == head


def test_leaving_inbox_unrated_is_refused(repo, capsys):
    err = _refused(repo, capsys, "item", "set-status", "bug-inb", "backlog", "--commit", "--no-push")
    assert "[rated] bug-inb.md is unrated" in err
    assert load_item(str(repo / "map" / "bug-inb.md")).status == "backlog"  # change left in place


def test_over_long_progress_line_is_refused(repo, capsys):
    err = _refused(repo, capsys, "item", "set-status", "bug-b", "in_progress", "--owner", "al",
                   "--note", "word " * 100, "--commit", "--no-push")
    assert "[readability]" in err


def test_avoid_word_is_refused(repo, capsys):
    err = _refused(repo, capsys, "item", "set-status", "bug-b", "in_progress", "--owner", "al",
                   "--note", "needs a reload first", "--commit", "--no-push")
    assert "[avoid-words] map/bug-b.md" in err


def test_rate_commit_also_goes_through_the_check(repo, capsys):
    # a rated Item whose body already breaks the cap: the rate commit is refused
    p = repo / "map" / "bug-b.md"
    it = load_item(str(p))
    write_item(str(p), it.meta, "x" * 600 + "\n")
    _refused(repo, capsys, "item", "rate", "bug-b", "--effort", "m", "--commit", "--no-push")


def test_new_commit_goes_through_the_check_and_leaves_the_file(repo, capsys):
    err = _refused(repo, capsys, "item", "new", "bug", "A reload problem", "--commit", "--no-push")
    assert "[avoid-words]" in err
    assert (repo / "map" / "bug-a-reload-problem.md").exists()


def test_clean_new_commits(repo):
    assert _run(repo, "item", "new", "bug", "A new thing", "--commit", "--no-push") == 0
    assert _git(repo, "log", "-1", "--format=%s").startswith("map: file bug")


def test_batch_skip_commit_goes_through_the_check(repo, capsys, tmp_path):
    _put(repo, "bug", "c", body="reload every time\n")  # an offender already in the tree
    rows = tmp_path / "rows.json"
    rows.write_text(json.dumps([{"item": "bug-b", "reason": "too-large", "note": "Needs a plan."}]))
    err = _refused(repo, capsys, "item", "skip", "--batch", str(rows), "--commit", "--no-push")
    assert "[avoid-words] map/bug-c.md" in err


def test_batch_skip_clean_commits(repo, tmp_path):
    rows = tmp_path / "rows.json"
    rows.write_text(json.dumps([{"item": "bug-b", "reason": "too-large", "note": "Needs a plan."}]))
    assert _run(repo, "item", "skip", "--batch", str(rows), "--commit", "--no-push") == 0
    assert _git(repo, "log", "-1", "--format=%s").startswith("map: night skip reasons")


def test_parent_ticket_is_exempt_from_similar_title_only(repo, capsys):
    _put(repo, "bug", "g", title="Hand stroke limit stalls the catch move")
    title = "Hand stroke limit stalls catch moves"
    flags = ["--type", "task", "--effort", "s", "--hardware", "none", "--owner", "none",
             "--radius", "local", "--area", "docs"]
    # no parent: refused as similar (exit 2)
    assert _run(repo, "item", "new", "ticket", title, *flags, "--no-push") == 2
    assert "similar Items exist" in capsys.readouterr().err
    # with --parent: filed and committed
    assert _run(repo, "item", "new", "ticket", title, "--parent", "p", *flags,
                "--commit", "--no-push") == 0
    # an exact open duplicate is still refused with a parent
    assert _run(repo, "item", "new", "ticket", title, "--parent", "p", *flags, "--no-push") == 2
    assert "duplicate of" in capsys.readouterr().err


def test_check_command_reports_and_exits_4(repo, capsys):
    assert _run(repo, "check") == 0
    _put(repo, "bug", "c", body="reload every time\n")
    assert _run(repo, "check") == 4
    assert "[avoid-words]" in capsys.readouterr().err
