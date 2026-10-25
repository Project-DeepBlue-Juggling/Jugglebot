"""mapkit item set-status / show / items."""
from __future__ import annotations

import argparse
import json
import subprocess

import pytest

from mapkit import itemcli
from mapkit.items import load_item, write_item


@pytest.fixture(autouse=True)
def _git_env(tmp_path, monkeypatch):
    monkeypatch.setenv("HOME", str(tmp_path))
    monkeypatch.setenv("GIT_CONFIG_GLOBAL", str(tmp_path / "gitconfig"))
    monkeypatch.setenv("GIT_CONFIG_NOSYSTEM", "1")


def _item(root, kind, slug, status="inbox", **extra):
    meta = {"kind": kind, "title": "T " + slug, "status": status,
            "created": "2026-10-01", "updated": "2026-10-01T00:00:00Z",
            "priority": "medium", "parent": "p"}
    if kind == "ticket":
        meta["type"] = "task"
    meta.update(extra)
    write_item(str(root / "map" / ("%s-%s.md" % (kind, slug))), meta, "Body text\n")


def _tree(root):
    (root / "map").mkdir(parents=True, exist_ok=True)
    (root / "docs").mkdir(exist_ok=True)
    (root / "GLOSSARY.md").write_text("# G\n\n## Language\n\n### Robot\n\n**Hand**: The cup.\n")
    for b in ("active", "parked", "archived"):
        (root / "plans" / b).mkdir(parents=True, exist_ok=True)
    write_item(str(root / "map" / "pathway-p.md"),
               {"kind": "pathway", "title": "P", "status": "active",
                "created": "2026-10-01", "destination": "reach the goal"}, "Body\n")
    (root / "plans" / "active" / "plan-a.md").write_text(
        "---\ntitle: Plan A\nstatus: active\npathway: p\nsteps:\n"
        "  - {id: P1, title: First, status: complete}\n"
        "  - {id: P2, title: Second, status: not_started}\n---\n\nbody\n")


def test_progress_section_created_then_appended(tmp_path):
    _tree(tmp_path)
    _item(tmp_path, "bug", "b")
    itemcli.set_status(str(tmp_path), "b", "in_progress", note="started", owner="al")
    itemcli.set_status(str(tmp_path), "bug-b", "in_progress", note="more", owner="al")
    it = load_item(str(tmp_path / "map" / "bug-b.md"))
    assert it.body.count("## Progress") == 1
    lines = [l for l in it.body.splitlines() if l.startswith("- 20")]
    assert len(lines) == 2 and lines[0].endswith(": started") and lines[1].endswith(": more")
    assert it.meta["owners"] == ["al"] and it.status == "in_progress"


def test_progress_appends_before_next_section(tmp_path):
    _tree(tmp_path)
    _item(tmp_path, "bug", "b")
    path = tmp_path / "map" / "bug-b.md"
    it = load_item(str(path))
    write_item(str(path), it.meta, "Body\n\n## Progress\n\n- 2026-10-01: a\n\n## Other\n\nx\n")
    itemcli.set_status(str(tmp_path), "b", "backlog", note="b")
    body = load_item(str(path)).body
    assert body.index(": b\n") < body.index("## Other")


def test_refuses_reopen_complete_unless_force(tmp_path):
    _tree(tmp_path)
    _item(tmp_path, "bug", "b", status="complete")
    with pytest.raises(itemcli.ItemCliError):
        itemcli.set_status(str(tmp_path), "b", "in_progress")
    itemcli.set_status(str(tmp_path), "b", "in_progress", force=True)
    assert load_item(str(tmp_path / "map" / "bug-b.md")).status == "in_progress"


def test_owner_clash_and_invalid_status(tmp_path):
    _tree(tmp_path)
    _item(tmp_path, "bug", "b", owners=["other"])
    with pytest.raises(itemcli.ItemCliError):
        itemcli.set_status(str(tmp_path), "b", "in_progress", owner="me")
    with pytest.raises(itemcli.ItemCliError):
        itemcli.set_status(str(tmp_path), "b", "open")  # ticket status on a bug
    itemcli.set_status(str(tmp_path), "b", "in_progress", owner="me", force=True)
    assert load_item(str(tmp_path / "map" / "bug-b.md")).meta["owners"] == ["other", "me"]
    # the holder itself may start it
    _item(tmp_path, "bug", "c", owners=["me"])
    itemcli.set_status(str(tmp_path), "c", "in_progress", owner="me")


def test_link_dedupe(tmp_path):
    _tree(tmp_path)
    _item(tmp_path, "bug", "b", links=["logbook/x.md"])
    itemcli.set_status(str(tmp_path), "b", "backlog", links=["logbook/x.md", "logbook/y.md", "logbook/y.md"])
    assert load_item(str(tmp_path / "map" / "bug-b.md")).meta["links"] == ["logbook/x.md", "logbook/y.md"]


def test_ambiguous_and_missing_slug(tmp_path):
    _tree(tmp_path)
    _item(tmp_path, "bug", "same")
    _item(tmp_path, "idea", "same")
    with pytest.raises(itemcli.ItemCliError):
        itemcli.find_item(str(tmp_path), "same")
    assert itemcli.find_item(str(tmp_path), "idea-same").kind == "idea"
    with pytest.raises(itemcli.ItemCliError):
        itemcli.find_item(str(tmp_path), "ghost")


def _git(cwd, *a):
    subprocess.run(["git"] + list(a), cwd=str(cwd), check=True,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def _out(cwd, *a):
    return subprocess.run(["git"] + list(a), cwd=str(cwd), check=True,
                          stdout=subprocess.PIPE, universal_newlines=True).stdout


def _git_repo(tmp_path):
    remote = tmp_path / "remote.git"
    work = tmp_path / "work"
    subprocess.run(["git", "init", "--bare", str(remote)], check=True,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["git", "--git-dir", str(remote), "symbolic-ref", "HEAD",
                    "refs/heads/main"], check=True)
    work.mkdir()
    _tree(work)
    _item(work, "bug", "b")
    for cmd in (["init"], ["symbolic-ref", "HEAD", "refs/heads/main"],
                ["config", "user.name", "T"], ["config", "user.email", "t@e.x"],
                ["remote", "add", "origin", str(remote)],
                ["add", "."], ["commit", "-m", "seed"],
                ["push", "-u", "origin", "main"]):
        _git(work, *cmd)
    return work, remote


def test_commit_path_commits_only_item_and_page_and_pushes(tmp_path):
    work, remote = _git_repo(tmp_path)
    (work / "unrelated.txt").write_text("x")
    _git(work, "add", "unrelated.txt")  # staged by someone else: must not be swept in
    itemcli.set_status(str(work), "b", "in_progress", note="n", commit=True)
    files = _out(work, "show", "--name-only", "--format=", "HEAD").split()
    assert "map/bug-b.md" in files and "unrelated.txt" not in files
    assert set(files) <= {"map/bug-b.md", "docs/project-map.md"}
    msg = _out(work, "log", "-1", "--format=%B").strip()
    assert msg == "map: in_progress T b" and "Logbook-Entry" not in msg
    assert _out(remote, "log", "-1", "--format=%B", "main").strip() == msg


def test_no_commit_stages_nothing(tmp_path, capsys):
    work, _ = _git_repo(tmp_path)
    ns = argparse.Namespace(repo=str(work), slug="b", status="complete", note="done",
                            link=[], owner=None, force=False, commit=False, no_push=True)
    assert itemcli._run_set_status(ns) == 0
    out = capsys.readouterr().out
    assert "stage: map/bug-b.md" in out and "stage: docs/project-map.md" in out
    assert _out(work, "diff", "--cached", "--name-only").strip() == ""
    ns.status = "bogus"
    assert itemcli._run_set_status(ns) == 2


def test_show_and_items(tmp_path, capsys):
    _tree(tmp_path)
    _item(tmp_path, "bug", "b1", priority="high")
    _item(tmp_path, "bug", "b2", status="complete")
    _item(tmp_path, "feature", "f1")
    _item(tmp_path, "ticket", "t1", status="open", rank=1)
    _item(tmp_path, "idea", "elsewhere", parent="other")
    d = itemcli.show(str(tmp_path), "p")
    assert d["destination"] == "reach the goal"
    assert [s["id"] for s in d["plans"][0]["steps"]] == ["P1", "P2"]
    assert [r["slug"] for r in d["children"]["bug"]] == ["b1"]
    assert list(d["children"]) == ["bug", "feature", "ticket"]
    ns = argparse.Namespace(repo=str(tmp_path), pathway="p", json=False)
    assert itemcli._run_show(ns) == 0
    txt = capsys.readouterr().out
    assert "reach the goal" in txt and "P2" in txt and "b1" in txt and "b2" not in txt
    ns.json = True
    itemcli._run_show(ns)
    assert json.loads(capsys.readouterr().out)["slug"] == "p"
    assert itemcli._run_show(argparse.Namespace(repo=str(tmp_path), pathway="zz", json=False)) == 2

    def items(**k):
        base = dict(repo=str(tmp_path), pathway=None, kind=None, status=None,
                    open=False, json=True)
        base.update(k)
        itemcli._run_items(argparse.Namespace(**base))
        return json.loads(capsys.readouterr().out)

    assert {r["slug"] for r in items(pathway="p")} == {"p", "b1", "b2", "f1", "t1"}
    assert [r["slug"] for r in items(kind="bug", open=True)] == ["b1"]
    assert [r["slug"] for r in items(status="complete")] == ["b2"]
    row = items(kind="bug", open=True)[0]
    assert row["priority"] == "high" and row["pathway"] == "p" and row["age_days"] >= 24
