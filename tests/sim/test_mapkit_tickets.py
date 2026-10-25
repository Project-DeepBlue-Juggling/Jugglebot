"""mapkit tickets: frontier / claim / resolve and the item writer."""
from __future__ import annotations

import argparse
import os
import subprocess

import pytest

from mapkit import tickets
from mapkit.items import dump_item, load_item, write_item


@pytest.fixture(autouse=True)
def _git_env(tmp_path, monkeypatch):
    monkeypatch.setenv("HOME", str(tmp_path))
    monkeypatch.setenv("GIT_CONFIG_GLOBAL", str(tmp_path / "gitconfig"))
    monkeypatch.setenv("GIT_CONFIG_NOSYSTEM", "1")


def _ticket(root, slug, rank, status="open", blocked_by=(), owners=(),
            parent="p"):
    meta = {"kind": "ticket", "title": "T " + slug, "type": "task",
            "parent": parent, "blocked_by": list(blocked_by),
            "owners": list(owners), "rank": rank, "status": status,
            "links": [], "created": "2026-10-23", "updated": "2026-10-23T00:00:00Z",
            "priority": "medium"}
    write_item(str(root / "map" / ("ticket-%s.md" % slug)), meta, "Question?\n")


def _plan(root, board, name):
    d = root / "plans" / board
    d.mkdir(parents=True, exist_ok=True)
    (d / name).write_text("---\ntitle: x\nstatus: active\n---\n\nbody\n")


def _tree(root):
    (root / "map").mkdir(parents=True, exist_ok=True)
    for b in ("active", "parked", "archived"):
        (root / "plans" / b).mkdir(parents=True, exist_ok=True)
    write_item(str(root / "map" / "pathway-p.md"),
               {"kind": "pathway", "title": "P", "status": "active",
                "created": "2026-10-23", "destination": "d"}, "Body\n")


def _slugs(root, pathway="p"):
    return [i.slug for i in tickets.frontier(str(root), pathway)]


def test_round_trip_byte_identical(tmp_path):
    p = tmp_path / "ticket-a.md"
    meta = {"kind": "ticket", "title": "Dash – colon: x", "type": "task",
            "parent": "p", "created": "2026-10-23", "status": "open"}
    write_item(str(p), meta, "\nline\n")
    first = p.read_text(encoding="utf-8")
    it = load_item(str(p))
    write_item(str(p), it.meta, it.body)
    assert p.read_text(encoding="utf-8") == first
    assert dump_item(it.meta, it.body) == first


def test_frontier_ordering_and_filters(tmp_path):
    _tree(tmp_path)
    _ticket(tmp_path, "c", 3)
    _ticket(tmp_path, "a", 1)
    _ticket(tmp_path, "b", 2, owners=["x"])
    _ticket(tmp_path, "d", 4, blocked_by=["ticket-c"])
    _ticket(tmp_path, "e", 5, status="complete")
    _ticket(tmp_path, "f", 6, parent="other")
    assert _slugs(tmp_path) == ["a", "c"]


def test_frontier_blockers(tmp_path):
    _tree(tmp_path)
    _ticket(tmp_path, "done", 1, status="complete")
    _ticket(tmp_path, "open1", 2)
    _plan(tmp_path, "active", "act.md")
    _plan(tmp_path, "archived", "arc.md")
    _ticket(tmp_path, "by-done", 3, blocked_by=["ticket-done"])
    _ticket(tmp_path, "by-open", 4, blocked_by=["ticket-open1"])
    _ticket(tmp_path, "by-act", 5, blocked_by=["act.md"])
    _ticket(tmp_path, "by-arc", 6, blocked_by=["arc.md"])
    _ticket(tmp_path, "by-ghost", 7, blocked_by=["nothing.md"])
    assert _slugs(tmp_path) == ["open1", "by-done", "by-arc"]


def _git_repo(tmp_path):
    remote = tmp_path / "remote.git"
    work = tmp_path / "work"
    subprocess.run(["git", "init", "--bare", str(remote)], check=True,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["git", "--git-dir", str(remote), "symbolic-ref", "HEAD",
                    "refs/heads/main"], check=True)
    work.mkdir()
    _tree(work)
    _ticket(work, "q", 1)
    for cmd in (["init"], ["symbolic-ref", "HEAD", "refs/heads/main"],
                ["config", "user.name", "T"], ["config", "user.email", "t@e.x"],
                ["remote", "add", "origin", str(remote)],
                ["add", "."], ["commit", "-m", "seed"],
                ["push", "-u", "origin", "main"]):
        subprocess.run(["git"] + cmd, cwd=str(work), check=True,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return work, remote


def _log(repo, ref="HEAD"):
    return subprocess.run(["git", "log", "-1", "--format=%B", ref], cwd=str(repo),
                          check=True, stdout=subprocess.PIPE,
                          universal_newlines=True).stdout.strip()


def _files(repo):
    return subprocess.run(["git", "show", "--name-only", "--format=", "HEAD"],
                          cwd=str(repo), check=True, stdout=subprocess.PIPE,
                          universal_newlines=True).stdout.split()


def test_claim_commits_and_pushes(tmp_path):
    work, remote = _git_repo(tmp_path)
    tickets.claim(str(work), "q", "alice")
    tickets.claim(str(work), "q", "alice")  # idempotent owners
    it = load_item(str(work / "map" / "ticket-q.md"))
    assert it.meta["owners"] == ["alice"]
    assert _log(work) == "map: claim q"
    assert "Logbook-Entry" not in _log(work)
    assert _files(work) == ["map/ticket-q.md"]
    assert _log(remote, "main") == "map: claim q"
    assert _slugs(work) == []


def test_resolve_appends_answer(tmp_path):
    work, remote = _git_repo(tmp_path)
    tickets.resolve(str(work), "q", "It is 42.")
    it = load_item(str(work / "map" / "ticket-q.md"))
    assert it.status == "complete"
    assert it.body.rstrip().endswith("## Answer\n\nIt is 42.")
    assert _log(work) == "map: resolve q"
    assert _log(remote, "main") == "map: resolve q"


def test_cli_refusals_exit_2(tmp_path):
    work, _ = _git_repo(tmp_path)
    _plan(work, "active", "act.md")
    ns = lambda **k: argparse.Namespace(repo=str(work), no_push=True, **k)
    assert tickets._run_claim(ns(slug="nope", user="a")) == 2
    assert tickets._run_claim(ns(slug="q", user="a")) == 0
    assert tickets._run_resolve(ns(slug="q", answer="x", answer_file=None)) == 0
    assert tickets._run_resolve(ns(slug="q", answer="y", answer_file=None)) == 2
    assert tickets._run_claim(ns(slug="q", user="b")) == 2
    # a pathway is not a ticket
    assert tickets._run_claim(ns(slug="p", user="a")) == 2
