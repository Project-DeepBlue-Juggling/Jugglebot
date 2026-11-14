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
    if meta["status"] != "inbox":  # the commit path's map-write check wants a triaged Item rated
        meta.update(effort="s", hardware="none", owner="none", radius="local", area="docs")
        if kind == "bug":
            meta["diagnosis"] = "unknown"
    write_item(str(root / "map" / ("%s-%s.md" % (kind, slug))), meta, "Body\n")


def _tree(root):
    """The live tree: per-kind IDs and a crosswalk file with a FICTIONAL old prefix.

    Numbers are distinct across kinds so the bare-digit tests still find one Item;
    the crosswalk rows are ``OLD-n`` for each Item (the code knows no real old prefix).
    """
    (root / "map").mkdir(parents=True, exist_ok=True)
    (root / "docs").mkdir(exist_ok=True)
    (root / "GLOSSARY.md").write_text("# G\n\n## Language\n\n### Robot\n\n**Hand**: The cup.\n")
    for b in ("active", "parked", "archived"):
        (root / "plans" / b).mkdir(parents=True, exist_ok=True)
    rows = (("OLD-1", "PWAY-1", "pathway", "p"), ("OLD-2", "BUG-2", "bug", "cone-margin"),
            ("OLD-3", "TKT-3", "ticket", "q"), ("OLD-4", "IDEA-4", "idea", "12"))  # idea-12: an all-digit slug
    for _old, new, kind, slug in rows:
        _write(root, kind, slug, new)
    (root / "mapkit").mkdir(exist_ok=True)
    (root / "mapkit" / "legacy_ids.csv").write_text(
        "old_id,new_id,kind,stem\n" + "".join("%s,%s,%s,%s-%s\n" % (o, n, k, k, sl) for o, n, k, sl in rows))


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
    for text in ("BUG-2", "bug2", "Bug-2", "bug-cone-margin", "cone-margin"):
        assert itemcli.find_item(root, text).slug == "cone-margin"
    assert itemcli.find_item(root, "2", allow_bare=True).slug == "cone-margin"
    with pytest.raises(itemcli.ItemCliError, match="write commands need"):
        itemcli.find_item(root, "2")
    _write(tmp_path, "feature", "x", "FEAT-12")
    assert itemcli.find_item(root, "12").kind == "idea"  # digits are only a slug to a write
    with pytest.raises(itemcli.ItemCliError, match="both the slug"):
        itemcli.find_item(root, "12", allow_bare=True)
    with pytest.raises(itemcli.ItemCliError, match="no Item"):
        itemcli.find_item(root, "BUG-99")
    assert itemcli.show(root, "1")["slug"] == "p" and itemcli.show(root, "pway1")["id"] == "PWAY-1"
    assert {i.slug for i in itemcli.list_items(root, pathway="PWAY-1")} >= {"p", "cone-margin"}


def test_set_status_by_id_echoes_id_stem_title_and_cites_it(tmp_path, capsys):
    from mapkit.__main__ import main
    _tree(tmp_path)
    assert main(["item", "set-status", "bug-2", "backlog", "--repo", str(tmp_path)]) == 0
    assert capsys.readouterr().out.splitlines()[0] == "BUG-2 bug-cone-margin -> backlog T cone-margin"
    assert main(["item", "set-status", "2", "backlog", "--repo", str(tmp_path)]) == 2
    assert itemcli._subject("in_progress", load_item(str(tmp_path / "map" / "bug-cone-margin.md"))) \
        == "map: in_progress BUG-2 T cone-margin"


def test_items_and_show_carry_the_id(tmp_path, capsys):
    from mapkit.__main__ import main
    _tree(tmp_path)
    assert main(["items", "--json", "--repo", str(tmp_path)]) == 0
    rows = json.loads(capsys.readouterr().out)
    assert {r["slug"]: r["id"] for r in rows}["cone-margin"] == "BUG-2"
    assert main(["items", "--repo", str(tmp_path)]) == 0
    assert any(l.split()[:3] == ["BUG-2", "bug", "cone-margin"]
               for l in capsys.readouterr().out.splitlines())
    assert main(["show", "PWAY-1", "--repo", str(tmp_path)]) == 0
    out = capsys.readouterr().out
    assert out.startswith("PWAY-1 T p [active] (p)") and "BUG-2" in out


def test_tickets_resolve_by_id_and_check_the_kind(tmp_path, capsys):
    work, _ = _repo(tmp_path)
    tickets.claim(str(work), "TKT-3", "al")
    assert _out(work, "log", "-1", "--format=%s").strip() == "map: claim TKT-3 q"
    with pytest.raises(tickets.TicketError, match="not a ticket"):
        tickets.claim(str(work), "BUG-2", "al", push=False)
    from mapkit.__main__ import main
    assert main(["frontier", "p", "--repo", str(work)]) == 0  # claimed: frontier is empty
    _write(work, "ticket", "r", "TKT-5", rank=1)
    assert main(["frontier", "p", "--repo", str(work)]) == 0
    assert capsys.readouterr().out.split("\t")[1] == "TKT-5"


# --- item new -----------------------------------------------------------------
def test_item_new_without_commit_allocates_and_normalises_refs(tmp_path, capsys):
    _tree(tmp_path)
    it = itemcli.new_item(str(tmp_path), "bug", "Cup skids", parent="PWAY-1",
                          links=["bug2", "logbook/x.md"])
    assert it.short_id == "BUG-3" and it.slug == "cup-skids" and it.status == "inbox"
    assert it.meta["parent"] == "p" and it.meta["links"] == ["bug-cone-margin", "logbook/x.md"]
    assert it.meta["filed_by"] == "claude"
    assert open(it.path).read().startswith("---\nid: BUG-3\nkind: bug\n")
    # a same slug under another kind gets -2 (the app's uuid is slug-keyed)
    assert itemcli.new_item(str(tmp_path), "idea", "Cone margin",
                            allow_similar=True).slug == "cone-margin-2"
    with pytest.raises(ids.IdError):
        itemcli.new_item(str(tmp_path), "bug", "x", parent="1")  # bare digits: not on a write


def test_item_new_commit_happy_path(tmp_path):
    work, remote = _repo(tmp_path)
    it = itemcli.new_item(str(work), "feature", "Short ids", commit=True)
    assert it.short_id == "FEAT-1"
    assert _out(remote, "log", "-1", "--format=%s", "main").strip() == "map: file feature FEAT-1 Short ids"


def test_item_new_repairs_when_the_remote_advances_between_check_and_push(tmp_path, monkeypatch):
    work, remote = _repo(tmp_path)
    real = itemcli._git_out
    state = {"pushes": 0}

    def racing(repo_root, *args):
        if args and args[0] == "push":
            state["pushes"] += 1
            if state["pushes"] == 1:  # after our rebase + check, before our push lands
                _other_clone_files(tmp_path, remote, "phone", "BUG-3")
        return real(repo_root, *args)

    monkeypatch.setattr(itemcli, "_git_out", racing)
    it = itemcli.new_item(str(work), "bug", "Mine", commit=True)
    assert state["pushes"] == 2  # the first push was rejected, the loop went round once
    assert it.short_id == "BUG-4"
    subjects = _out(remote, "log", "--format=%s", "main").splitlines()
    # the renumber is folded into our unpushed file commit: no subject cites BUG-3 for Mine
    assert subjects[:2] == ["map: file bug BUG-4 Mine", "map: file bug BUG-3 T phone"]
    assert "map: renumber" not in "\n".join(subjects)
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
    _write(work, "bug", "on-feat", "BUG-3")  # upstream ahead of main
    _git(work, "add", ".")
    _git(work, "commit", "-m", "feat item")
    _git(work, "push", "-u", "origin", "feat")
    _inject_before_first_pull(monkeypatch, lambda: _push_from_other(
        tmp_path, remote, "feat", theirs, "BUG-4", mine))
    it = itemcli.new_item(str(work), "bug", mine, commit=True)
    assert it.short_id == "BUG-5"
    assert load_item(str(work / "map" / ("bug-%s.md" % theirs))).short_id == "BUG-4"
    _git(work, "fetch")
    assert not ids.check(str(work), "origin/feat")["duplicate"]
    assert _out(remote, "log", "-1", "--format=%s", "feat").strip() == "map: file bug BUG-5 " + mine
    from mapkit import gen_docs
    assert gen_docs.write_docs(str(work), check=True) == []  # docs regenerated before the push


def test_slug_clash_aborts_the_rebase_and_exits_cleanly(tmp_path, monkeypatch):
    work, remote = _repo(tmp_path)
    _inject_before_first_pull(monkeypatch, lambda: _push_from_other(
        tmp_path, remote, "main", "same-title", "BUG-3", "clash"))
    with pytest.raises(itemcli.ItemCliError, match="rebase was aborted") as exc:
        itemcli.new_item(str(work), "bug", "Same title", commit=True)
    assert "map/bug-same-title.md" in str(exc.value)
    status = _out(work, "status")
    assert "rebase" not in status and "Unmerged" not in status
    assert _out(work, "log", "-1", "--format=%s").strip() == "map: file bug BUG-3 Same title"


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


# --- build-review fixes (2026-10-26): amend-on-renumber, lenient allocation, waiting commit, item push ----
def test_renumber_is_a_separate_commit_when_the_file_commit_is_not_head(tmp_path, monkeypatch):
    work, remote = _repo(tmp_path)
    itemcli.new_item(str(work), "bug", "Mine", commit=True, push=False)  # BUG-3, local only
    (work / "unrelated.txt").write_text("x\n")
    _git(work, "add", ".")
    _git(work, "commit", "-m", "later local commit")
    _other_clone_files(tmp_path, remote, "phone", "BUG-3")
    it = itemcli.push_item(str(work), "bug-mine")
    assert it.short_id == "BUG-4"
    subjects = _out(remote, "log", "--format=%s", "main").splitlines()
    assert subjects[0] == "map: renumber BUG-3 -> BUG-4 Mine"  # HEAD was the unrelated commit
    assert "later local commit" in subjects


def test_item_push_after_no_push_runs_the_same_loop(tmp_path):
    work, remote = _repo(tmp_path)
    itemcli.new_item(str(work), "idea", "Check first", commit=True, push=False)
    assert _out(remote, "log", "-1", "--format=%s", "main").strip() == "seed"
    from mapkit.__main__ import main
    assert main(["item", "push", "idea-check-first", "--repo", str(work)]) == 0
    assert _out(remote, "log", "-1", "--format=%s", "main").strip() == "map: file idea IDEA-5 Check first"


def test_failure_after_the_commit_says_a_local_commit_is_waiting(tmp_path, monkeypatch):
    work, _ = _repo(tmp_path)
    real = itemcli._git_out

    def rejecting(repo_root, *args):
        if args and args[0] == "push":
            return subprocess.CompletedProcess(args, 1, "", "rejected")
        return real(repo_root, *args)

    monkeypatch.setattr(itemcli, "_git_out", rejecting)
    with pytest.raises(itemcli.ItemCliError) as exc:
        itemcli.new_item(str(work), "bug", "Stuck", commit=True)
    msg = str(exc.value)
    assert "local commit for bug-stuck is waiting" in msg
    assert "item push bug-stuck" in msg and "git reset --soft HEAD~1" in msg and "--hard" not in msg


def test_a_malformed_sibling_is_skipped_for_allocation_and_named_when_gen_refuses(tmp_path, capsys):
    work, _ = _repo(tmp_path)
    (work / "map" / "bug-broken.md").write_text("---\nid: BUG-9\nkind: bug\nstatus: bogus\n---\n\nb\n")
    before = sorted(p.name for p in (work / "map").iterdir())
    with pytest.raises(itemcli.ItemCliError, match="bug-broken.md"):
        itemcli.new_item(str(work), "bug", "Fine", commit=True)
    assert sorted(p.name for p in (work / "map").iterdir()) == before  # nothing half-filed
    assert "skipping unreadable Item map/bug-broken.md" in capsys.readouterr().err
    assert ids.allocate(str(work), kind="bug") == "BUG-10"  # the broken file's BUG-9 is never reused


# --- ids-p6-audit-fixes (2026-10-26) -------------------------------------------------------
def test_item_push_unknown_name_refuses_before_any_git_call(tmp_path, monkeypatch, capsys):
    work, remote = _repo(tmp_path)
    itemcli.new_item(str(work), "bug", "Other", commit=True, push=False)  # an unrelated unpushed commit
    calls = []
    real = itemcli._git_out
    monkeypatch.setattr(itemcli, "_git_out", lambda r, *a: calls.append(a) or real(r, *a))
    from mapkit.__main__ import main
    assert main(["item", "push", "BUG-64", "--repo", str(work)]) == 2
    err = capsys.readouterr().err
    assert "BUG-64" in err and "waiting" not in err
    assert calls == []  # not even the first pull
    assert _out(remote, "log", "-1", "--format=%s", "main").strip() == "seed"  # nothing went out


def test_item_push_accepts_an_id(tmp_path):
    work, remote = _repo(tmp_path)
    itemcli.new_item(str(work), "idea", "By id", commit=True, push=False)  # IDEA-5
    from mapkit.__main__ import main
    assert main(["item", "push", "IDEA-5", "--repo", str(work)]) == 0
    assert _out(remote, "log", "-1", "--format=%s", "main").strip() == "map: file idea IDEA-5 By id"


def test_rerunning_item_push_never_amends_a_published_commit(tmp_path):
    work, remote = _repo(tmp_path)
    itemcli.new_item(str(work), "bug", "Done", commit=True, push=False)
    page = work / "docs" / "project-map.md"
    page.write_text(page.read_text() + "\nstale\n")  # the commit carries a stale generated page
    _git(work, "commit", "-q", "-a", "--amend", "--no-edit")
    _git(work, "push")
    sha = _out(remote, "rev-parse", "main").strip()
    itemcli.push_item(str(work), "bug-done")  # re-run: regenerating dirties the page again
    assert _out(work, "log", "--format=%H").split().count(sha) == 1  # the published commit survives
    assert _out(work, "rev-parse", "HEAD") == _out(remote, "rev-parse", "main")  # and the push landed
    assert _out(work, "log", "-1", "--format=%s").strip() == "map: regenerate project-map page"


# --- per-kind IDs (plan item-id-per-kind, P1 unit 3) ----------------------------
def test_link_and_pathway_filter_take_per_kind_ids(tmp_path):
    """T-U4/T-U5 (P1 unit 3): ``--link BUG-2`` was stored as plain text; ``--pathway PWAY-1`` ignored.

    The is-an-ID test at itemcli ``new_item`` links and ``list_items`` pathway
    used the old-form-only ``ids.parse``, so a per-kind reference fell through silently.
    """
    _tree(tmp_path)
    got = itemcli.list_items(str(tmp_path), pathway="PWAY-1")
    assert {i.slug for i in got} >= {"q"}
    assert [i.slug for i in itemcli.list_items(str(tmp_path), pathway="1")] == [i.slug for i in got]
    it = itemcli.new_item(str(tmp_path), "bug", "Cup skids", links=["BUG-2", "bug2", "logbook/x.md"],
                          allow_similar=True)
    assert it.meta["links"] == ["bug-cone-margin", "logbook/x.md"]


def test_new_item_allocates_in_its_own_kind(tmp_path):
    """T-U2/T-U3 (P1 unit 3): ``item new`` passes ``kind=`` to allocate; always per kind, file or no file."""
    _tree(tmp_path)
    a = itemcli.new_item(str(tmp_path), "bug", "Cup skids")
    b = itemcli.new_item(str(tmp_path), "feature", "Wider stroke")
    assert a.short_id == "BUG-3" and b.short_id == "FEAT-1"
    assert open(a.path).read().startswith("---\nid: BUG-3\nkind: bug\n")
    (tmp_path / "mapkit" / "legacy_ids.csv").unlink()
    c = itemcli.new_item(str(tmp_path), "idea", "Brand new thing")
    assert c.short_id == "IDEA-5"


def test_old_id_spellings_at_link_and_pathway(tmp_path):
    """Any spelling of an old ID: ``--pathway`` reads it as the new Pathway; ``--link`` refuses naming the new ID."""
    _tree(tmp_path)
    by_id = itemcli.list_items(str(tmp_path), pathway="PWAY-1")
    assert by_id
    for text in ("OLD-1", "old-01", "old1", "OLD-001"):
        assert [i.slug for i in itemcli.list_items(str(tmp_path), pathway=text)] == [i.slug for i in by_id]
    for text in ("OLD-2", "old-02", "old2"):
        with pytest.raises(itemcli.ItemCliError, match="BUG-2"):
            itemcli.new_item(str(tmp_path), "bug", "Cup skids", links=[text], allow_similar=True)
    assert not (tmp_path / "map" / "bug-cup-skids.md").exists()  # the refusal wrote nothing


# --- P4 unit 1 (D-3): an old ID after the cutover ---------------------------
def test_legacy_id_reads_translate_with_a_stderr_line(tmp_path, capsys):
    from mapkit.__main__ import main
    _tree(tmp_path)
    capsys.readouterr()
    assert main(["items", "--pathway", "OLD-1", "--repo", str(tmp_path)]) == 0
    cap = capsys.readouterr()
    assert "OLD-1 -> PWAY-1" in cap.err.splitlines()
    assert "cone-margin" in cap.out
    assert main(["show", "OLD-1", "--repo", str(tmp_path)]) == 0
    assert "OLD-1 -> PWAY-1" in capsys.readouterr().err.splitlines()
    assert main(["show", "old1", "--repo", str(tmp_path)]) == 0
    assert "old1 -> PWAY-1" in capsys.readouterr().err.splitlines()
    assert main(["items", "--pathway", "PWAY-1", "--repo", str(tmp_path)]) == 0
    assert capsys.readouterr().err == ""  # the new ID prints no translation


def test_legacy_id_writes_refuse_and_name_the_new_id(tmp_path, capsys):
    from mapkit.__main__ import main
    _tree(tmp_path)
    bug = tmp_path / "map" / "bug-cone-margin.md"
    before = bug.read_text()
    capsys.readouterr()
    assert main(["item", "set-status", "OLD-2", "backlog", "--repo", str(tmp_path)]) == 2
    assert "OLD-2 is now BUG-2 (bug-cone-margin)" in capsys.readouterr().err
    assert main(["item", "rate", "old2", "--effort", "s", "--repo", str(tmp_path)]) == 2
    assert "BUG-2" in capsys.readouterr().err
    assert bug.read_text() == before  # both refusals changed nothing
    assert main(["item", "set-status", "BUG-2", "backlog", "--repo", str(tmp_path)]) == 0
    assert bug.read_text() != before
    with pytest.raises(ids.LegacyIdError, match="PWAY-1"):  # _run_new turns IdError into exit 2
        itemcli.new_item(str(tmp_path), "bug", "Another thing", parent="OLD-1", allow_similar=True)
    with pytest.raises(itemcli.ItemCliError, match="TKT-3"):
        itemcli.new_item(str(tmp_path), "bug", "Third thing", links=["OLD-3"], allow_similar=True)
    with pytest.raises(itemcli.ItemCliError, match="BUG-2"):
        itemcli.new_item(str(tmp_path), "ticket", "Q two", ticket_type="question",
                         parent="OLD-2", allow_similar=True)
    assert main(["item", "new", "bug", "Via cli", "--parent", "OLD-1", "--repo", str(tmp_path)]) == 2


def test_unknown_legacy_id_is_the_normal_not_found(tmp_path):
    _tree(tmp_path)
    with pytest.raises(itemcli.ItemCliError, match="no Item"):
        itemcli.find_item(str(tmp_path), "OLD-99")


@pytest.mark.parametrize("argv", [
    ["item", "set-status", "OLD-2", "complete"],
    ["item", "set-status", "bug-nonexistent", "complete"],
    ["item", "new", "bug", "A new title", "--link", "OLD-2"],
    ["items", "--pathway", "OLD-1"],
    ["show", "OLD-1"],
])
def test_a_malformed_crosswalk_is_one_error_line_not_a_traceback(tmp_path, capsys, argv):
    """P5 audit 1 (P4 audit B3): ``ids.legacy`` raises ItemError naming file and line on a
    malformed crosswalk; every command reports it as one ``mapkit:`` line, exit 2, writing nothing."""
    from mapkit.__main__ import main
    _tree(tmp_path)
    (tmp_path / "mapkit" / "legacy_ids.csv").write_text("old,new\nOLD-2,BUG-2\n")
    before = sorted(p.name for p in (tmp_path / "map").iterdir())
    assert main(argv + ["--repo", str(tmp_path)]) == 2
    err = capsys.readouterr().err
    assert "mapkit/legacy_ids.csv:1:" in err and "Traceback" not in err
    assert sorted(p.name for p in (tmp_path / "map").iterdir()) == before


def test_items_with_an_unknown_pathway_id_is_one_error_line(tmp_path, capsys):
    """Found during P5: ``items --pathway <unknown ID>`` raised ItemCliError out of
    ``_run_items`` (which caught only ItemError) as a traceback, exit 1."""
    from mapkit.__main__ import main
    _tree(tmp_path)
    assert main(["items", "--pathway", "PWAY-999", "--repo", str(tmp_path)]) == 2
    err = capsys.readouterr().err
    assert "no pathway 'PWAY-999'" in err and "Traceback" not in err
