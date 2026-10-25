"""Item IDs through the CLI (plan map-item-ids.md, P3): resolution, ``item new``, the push loop.

Fixture repos live under tmp_path with a bare remote; no ports, safe under xdist.
"""
from __future__ import annotations

import json
import subprocess

import pytest

from mapkit import ids, itemcli, tickets
from mapkit.items import load_item, write_item


@pytest.fixture(autouse=True)
def _git_env(tmp_path, monkeypatch):
    monkeypatch.setenv("HOME", str(tmp_path))
    monkeypatch.setenv("GIT_CONFIG_GLOBAL", str(tmp_path / "gitconfig"))
    monkeypatch.setenv("GIT_CONFIG_NOSYSTEM", "1")


def _git(cwd, *a):
    subprocess.run(["git"] + list(a), cwd=str(cwd), check=True,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def _out(cwd, *a):
    return subprocess.run(["git"] + list(a), cwd=str(cwd), check=True,
                          stdout=subprocess.PIPE, universal_newlines=True).stdout


def _write(root, kind, slug, sid=None, **extra):
    meta = {"kind": kind, "title": "T " + slug, "status": "inbox", "created": "2026-10-01",
            "updated": "2026-10-01T00:00:00Z", "priority": "medium", "parent": "p"}
    if sid:
        meta = dict([("id", sid)] + list(meta.items()))
    if kind == "ticket":
        meta.update(type="task", status="open")
    if kind == "pathway":
        meta.update(status="active", destination="reach the goal")
        meta.pop("parent")
    meta.update(extra)
    write_item(str(root / "map" / ("%s-%s.md" % (kind, slug))), meta, "Body\n")


def _tree(root):
    (root / "map").mkdir(parents=True, exist_ok=True)
    (root / "docs").mkdir(exist_ok=True)
    (root / "GLOSSARY.md").write_text("# G\n\n## Language\n\n### Robot\n\n**Hand**: The cup.\n")
    for b in ("active", "parked", "archived"):
        (root / "plans" / b).mkdir(parents=True, exist_ok=True)
    _write(root, "pathway", "p", "JB-1")
    _write(root, "bug", "cone-margin", "JB-2")
    _write(root, "ticket", "q", "JB-3")
    _write(root, "idea", "12", "JB-4")  # an all-digit slug


def _repo(tmp_path, name="work"):
    remote = tmp_path / "remote.git"
    work = tmp_path / name
    _git(tmp_path, "init", "--bare", str(remote))
    _git(remote, "symbolic-ref", "HEAD", "refs/heads/main")
    work.mkdir()
    _tree(work)
    for cmd in (["init"], ["symbolic-ref", "HEAD", "refs/heads/main"],
                ["config", "user.name", "T"], ["config", "user.email", "t@e.x"],
                ["remote", "add", "origin", str(remote)], ["add", "."],
                ["commit", "-m", "seed"], ["push", "-u", "origin", "main"]):
        _git(work, *cmd)
    return work, remote


def _other_clone_files(tmp_path, remote, slug, sid):
    other = tmp_path / ("other-" + slug)
    _git(tmp_path, "clone", str(remote), str(other))
    _git(other, "config", "user.name", "O")
    _git(other, "config", "user.email", "o@e.x")
    _write(other, "bug", slug, sid)
    _git(other, "add", ".")
    _git(other, "commit", "-m", "map: file bug %s T %s" % (sid, slug))
    _git(other, "push", "origin", "HEAD:main")


# --- resolution -------------------------------------------------------------
def test_reads_take_every_form_writes_refuse_bare_digits(tmp_path, capsys):
    _tree(tmp_path)
    root = str(tmp_path)
    for text in ("JB-2", "jb2", "Jb-2", "bug-cone-margin", "cone-margin"):
        assert itemcli.find_item(root, text).slug == "cone-margin"
    assert itemcli.find_item(root, "2", allow_bare=True).slug == "cone-margin"
    with pytest.raises(itemcli.ItemCliError, match="write commands need"):
        itemcli.find_item(root, "2")
    _write(tmp_path, "feature", "x", "JB-12")
    assert itemcli.find_item(root, "12").kind == "idea"  # digits are only a slug to a write
    with pytest.raises(itemcli.ItemCliError, match="both the slug"):
        itemcli.find_item(root, "12", allow_bare=True)
    with pytest.raises(itemcli.ItemCliError, match="no Item"):
        itemcli.find_item(root, "JB-99")
    assert itemcli.show(root, "1")["slug"] == "p" and itemcli.show(root, "jb1")["id"] == "JB-1"
    assert {i.slug for i in itemcli.list_items(root, pathway="JB-1")} >= {"p", "cone-margin"}


def test_set_status_by_id_echoes_id_stem_title_and_cites_it(tmp_path, capsys):
    from mapkit.__main__ import main
    _tree(tmp_path)
    assert main(["item", "set-status", "jb-2", "backlog", "--repo", str(tmp_path)]) == 0
    assert capsys.readouterr().out.splitlines()[0] == "JB-2 bug-cone-margin -> backlog T cone-margin"
    assert main(["item", "set-status", "2", "backlog", "--repo", str(tmp_path)]) == 2
    assert itemcli._subject("in_progress", load_item(str(tmp_path / "map" / "bug-cone-margin.md"))) \
        == "map: in_progress JB-2 T cone-margin"


def test_items_and_show_carry_the_id(tmp_path, capsys):
    from mapkit.__main__ import main
    _tree(tmp_path)
    assert main(["items", "--json", "--repo", str(tmp_path)]) == 0
    rows = json.loads(capsys.readouterr().out)
    assert {r["slug"]: r["id"] for r in rows}["cone-margin"] == "JB-2"
    assert main(["items", "--repo", str(tmp_path)]) == 0
    assert any(l.split()[:3] == ["JB-2", "bug", "cone-margin"]
               for l in capsys.readouterr().out.splitlines())
    assert main(["show", "JB-1", "--repo", str(tmp_path)]) == 0
    out = capsys.readouterr().out
    assert out.startswith("JB-1 T p [active] (p)") and "JB-2" in out


def test_tickets_resolve_by_id_and_check_the_kind(tmp_path, capsys):
    work, _ = _repo(tmp_path)
    tickets.claim(str(work), "JB-3", "al")
    assert _out(work, "log", "-1", "--format=%s").strip() == "map: claim JB-3 q"
    with pytest.raises(tickets.TicketError, match="not a ticket"):
        tickets.claim(str(work), "JB-2", "al", push=False)
    from mapkit.__main__ import main
    assert main(["frontier", "p", "--repo", str(work)]) == 0  # claimed: frontier is empty
    _write(work, "ticket", "r", "JB-5", rank=1)
    assert main(["frontier", "p", "--repo", str(work)]) == 0
    assert capsys.readouterr().out.split("\t")[1] == "JB-5"


# --- item new -----------------------------------------------------------------
def test_item_new_without_commit_allocates_and_normalises_refs(tmp_path, capsys):
    _tree(tmp_path)
    it = itemcli.new_item(str(tmp_path), "bug", "Cup skids", parent="JB-1",
                          links=["jb2", "logbook/x.md"])
    assert it.short_id == "JB-5" and it.slug == "cup-skids" and it.status == "inbox"
    assert it.meta["parent"] == "p" and it.meta["links"] == ["bug-cone-margin", "logbook/x.md"]
    assert it.meta["filed_by"] == "claude"
    assert open(it.path).read().startswith("---\nid: JB-5\nkind: bug\n")
    # a same slug under another kind gets -2 (the app's uuid is slug-keyed)
    assert itemcli.new_item(str(tmp_path), "idea", "Cone margin").slug == "cone-margin-2"
    with pytest.raises(ids.IdError):
        itemcli.new_item(str(tmp_path), "bug", "x", parent="1")  # bare digits: not on a write


def test_item_new_commit_happy_path(tmp_path):
    work, remote = _repo(tmp_path)
    it = itemcli.new_item(str(work), "feature", "Short ids", commit=True)
    assert it.short_id == "JB-5"
    assert _out(remote, "log", "-1", "--format=%s", "main").strip() == "map: file feature JB-5 Short ids"


def test_item_new_repairs_when_the_remote_advances_between_check_and_push(tmp_path, monkeypatch):
    work, remote = _repo(tmp_path)
    real = itemcli._git_out
    state = {"pushes": 0}

    def racing(repo_root, *args):
        if args and args[0] == "push":
            state["pushes"] += 1
            if state["pushes"] == 1:  # after our rebase + check, before our push lands
                _other_clone_files(tmp_path, remote, "phone", "JB-5")
        return real(repo_root, *args)

    monkeypatch.setattr(itemcli, "_git_out", racing)
    it = itemcli.new_item(str(work), "bug", "Mine", commit=True)
    assert state["pushes"] == 2  # the first push was rejected, the loop went round once
    assert it.short_id == "JB-6"
    subjects = _out(remote, "log", "--format=%s", "main").splitlines()
    assert subjects[:3] == ["map: renumber JB-5 -> JB-6 Mine", "map: file bug JB-5 Mine",
                            "map: file bug JB-5 T phone"]
    _git(work, "fetch")
    assert not ids.check(str(work))["duplicate"]


def test_item_new_gives_up_after_three_rejected_pushes(tmp_path, monkeypatch):
    work, _ = _repo(tmp_path)
    real = itemcli._git_out

    def rejecting(repo_root, *args):
        if args and args[0] == "push":
            return subprocess.CompletedProcess(args, 1, "", "rejected")
        return real(repo_root, *args)

    monkeypatch.setattr(itemcli, "_git_out", rejecting)
    with pytest.raises(itemcli.ItemCliError, match="push rejected 3 times"):
        itemcli.new_item(str(work), "bug", "Stuck", commit=True)


# --- audit fixes (2026-10-26): upstream-aware repair, rebase abort, validate before write ----
def _push_from_other(tmp_path, remote, branch, slug, sid, tag):
    other = tmp_path / ("other-%s-%s" % (tag, slug))
    _git(tmp_path, "clone", "-b", branch, str(remote), str(other))
    _git(other, "config", "user.name", "O")
    _git(other, "config", "user.email", "o@e.x")
    # same created date as ours, so only the own-set rule (not created) can pick the keeper
    _write(other, "bug", slug, sid, created=itemcli._today())
    _git(other, "add", ".")
    _git(other, "commit", "-m", "map: file bug %s T %s" % (sid, slug))
    _git(other, "push", "origin", "HEAD:%s" % branch)


def _inject_before_first_pull(monkeypatch, action):
    real = itemcli._git_out
    state = {"done": False}

    def racing(repo_root, *args):
        if args[:2] == ("pull", "--rebase") and not state["done"]:
            state["done"] = True
            action()
        return real(repo_root, *args)

    monkeypatch.setattr(itemcli, "_git_out", racing)


@pytest.mark.parametrize("mine,theirs", [("aardvark", "zulu"), ("zulu", "aardvark")])
def test_repair_on_a_feature_upstream_renumbers_ours_whatever_the_stem_order(
        tmp_path, monkeypatch, mine, theirs):
    # The audit repro: the upstream is origin/feat (ahead of origin/main), so the
    # Item the rebase brings in is no holder on origin/main; ours must still lose.
    work, remote = _repo(tmp_path)
    _git(work, "checkout", "-b", "feat")
    _write(work, "bug", "on-feat", "JB-5")  # upstream ahead of main
    _git(work, "add", ".")
    _git(work, "commit", "-m", "feat item")
    _git(work, "push", "-u", "origin", "feat")
    _inject_before_first_pull(monkeypatch, lambda: _push_from_other(
        tmp_path, remote, "feat", theirs, "JB-6", mine))
    it = itemcli.new_item(str(work), "bug", mine, commit=True)
    assert it.short_id == "JB-7"
    assert load_item(str(work / "map" / ("bug-%s.md" % theirs))).short_id == "JB-6"
    _git(work, "fetch")
    assert not ids.check(str(work), "origin/feat")["duplicate"]
    assert _out(remote, "log", "-1", "--format=%s", "feat").strip() == "map: renumber JB-6 -> JB-7 " + mine
    from mapkit import gen_docs
    assert gen_docs.write_docs(str(work), check=True) == []  # docs regenerated before the push


def test_slug_clash_aborts_the_rebase_and_exits_cleanly(tmp_path, monkeypatch):
    work, remote = _repo(tmp_path)
    _inject_before_first_pull(monkeypatch, lambda: _push_from_other(
        tmp_path, remote, "main", "same-title", "JB-5", "clash"))
    with pytest.raises(itemcli.ItemCliError, match="rebase was aborted") as exc:
        itemcli.new_item(str(work), "bug", "Same title", commit=True)
    assert "map/bug-same-title.md" in str(exc.value)
    status = _out(work, "status")
    assert "rebase" not in status and "Unmerged" not in status
    assert _out(work, "log", "-1", "--format=%s").strip() == "map: file bug JB-5 Same title"


def test_invalid_new_item_leaves_no_file(tmp_path):
    _tree(tmp_path)
    before = sorted(p.name for p in (tmp_path / "map").iterdir())
    with pytest.raises(itemcli.ItemCliError, match="destination"):
        itemcli.new_item(str(tmp_path), "pathway", "No destination")
    with pytest.raises(itemcli.ItemCliError, match="parent"):
        itemcli.new_item(str(tmp_path), "ticket", "No parent")
    assert sorted(p.name for p in (tmp_path / "map").iterdir()) == before


def test_cli_new_reports_a_git_failure_without_a_traceback(tmp_path, capsys):
    from mapkit.__main__ import main
    _tree(tmp_path)  # not a git repo: the add fails
    assert main(["item", "new", "bug", "x", "--commit", "--no-push", "--repo", str(tmp_path)]) == 2
    assert "git add" in capsys.readouterr().err
