"""mapkit.ids: format, allocate, duplicates, check, repair, assign_missing, resolve.

Each git test builds a working clone of a bare remote under tmp_path, so
``origin/main`` is a real published ref. No ports; safe under xdist.
"""
from __future__ import annotations

import subprocess

import pytest

from mapkit import ids
from mapkit.items import load_item, load_items, write_item


@pytest.fixture(autouse=True)
def _git_env(tmp_path, monkeypatch):
    monkeypatch.setenv("HOME", str(tmp_path))
    monkeypatch.setenv("GIT_CONFIG_GLOBAL", str(tmp_path / "gitconfig"))
    monkeypatch.setenv("GIT_CONFIG_NOSYSTEM", "1")


def _git(cwd, *args):
    return subprocess.run(["git"] + list(args), cwd=str(cwd), check=True,
                          capture_output=True, text=True).stdout


def _item(root, kind, slug, created="2026-10-01", **extra):
    meta = {"kind": kind, "title": "T " + slug, "status": "inbox",
            "created": created, "updated": "2026-10-01T00:00:00Z", "priority": "medium"}
    if kind == "ticket":
        meta.update(type="task", parent="p", status="open")
    meta.update(extra)
    path = root / "map" / ("%s-%s.md" % (kind, slug))
    write_item(str(path), meta, "Body text\n")
    return path


def _repo(tmp_path):
    """(working clone, bare remote); one commit on main with an empty map/."""
    remote = tmp_path / "remote.git"
    _git(tmp_path, "init", "-q", "--bare", str(remote))  # this git has no init -b
    _git(remote, "symbolic-ref", "HEAD", "refs/heads/main")
    work = tmp_path / "work"
    _git(tmp_path, "clone", "-q", str(remote), str(work))
    _git(work, "config", "user.email", "t@example.com")
    _git(work, "config", "user.name", "t")
    _git(work, "checkout", "-q", "-b", "main")
    (work / "map").mkdir()
    (work / "map" / "config.yaml").write_text("kinds: []\n")
    _commit_push(work, "seed")
    return work, remote


def _commit_push(work, msg):
    _git(work, "add", "-A")
    _git(work, "commit", "-q", "-m", msg)
    _git(work, "push", "-q", "origin", "HEAD:main")
    _git(work, "fetch", "-q", "origin")


def _items(work):
    return load_items(str(work / "map"))


# --- format ----------------------------------------------------------------
def test_format_and_parse():
    assert ids.format_id(7) == "JB-7"
    for bad in (0, -1, True):
        with pytest.raises(ValueError):
            ids.format_id(bad)
    for text in ("JB-12", "jb12", "Jb-12", " jb-12 "):
        assert ids.parse(text) == 12
    assert ids.parse("12") is None and ids.parse("12", allow_bare=True) == 12
    for bad in ("JB-0", "XB-3", "JB-", "jb-8a", ""):
        assert ids.parse(bad, allow_bare=True) is None


def test_canonical_rejects_loose_forms():
    assert ids.canonical("JB-3") == 3
    for bad in ("JB-01", "jb-3", "JB3", 3, None, "JB-0"):
        assert ids.canonical(bad) is None


def test_malformed_id_still_loads(tmp_path):
    (tmp_path / "map").mkdir()
    p = _item(tmp_path, "bug", "b", id=12)  # YAML int, not the canonical string
    it = load_item(str(p))
    assert it.short_id is None and ids.malformed([it]) == [it]


def test_write_item_puts_id_first(tmp_path):
    (tmp_path / "map").mkdir()
    p = _item(tmp_path, "bug", "b", id="JB-4")
    assert p.read_text().startswith("---\nid: JB-4\nkind: bug\n")
    assert load_item(str(p)).short_id == "JB-4"


# --- allocate / duplicates --------------------------------------------------
def test_allocate_empty_map_is_one(tmp_path):
    work, _ = _repo(tmp_path)
    assert ids.allocate(str(work)) == "JB-1"


def test_allocate_takes_max_over_tree_and_ref(tmp_path):
    work, _ = _repo(tmp_path)
    _item(work, "bug", "a", id="JB-3")
    _commit_push(work, "a")
    _git(work, "rm", "-q", "map/bug-a.md")  # gone locally, still on origin/main
    _item(work, "idea", "b", id="JB-2")
    assert ids.allocate(str(work)) == "JB-4"
    _item(work, "idea", "c", id="JB-9")
    assert ids.allocate(str(work)) == "JB-10"


def test_duplicates_lists_every_member(tmp_path):
    (tmp_path / "map").mkdir()
    for s in ("a", "b", "c"):
        _item(tmp_path, "bug", s, id="JB-1" if s != "c" else "JB-2")
    d = ids.duplicates(load_items(str(tmp_path / "map")))
    assert list(d) == ["JB-1"] and [i.slug for i in d["JB-1"]] == ["a", "b"]


# --- repair -----------------------------------------------------------------
def test_repair_renumbers_the_unpushed_item(tmp_path):
    work, _ = _repo(tmp_path)
    _item(work, "bug", "pub", created="2026-10-05", id="JB-1")
    _commit_push(work, "pub")
    _item(work, "bug", "local", created="2026-10-01", id="JB-1")  # older, but unpublished
    before = (work / "map" / "bug-local.md").read_text()
    rows = ids.repair(str(work))
    assert rows == [("JB-1", "JB-2", "bug-local")]
    after = (work / "map" / "bug-local.md").read_text()
    assert after == before.replace("id: JB-1", "id: JB-2")  # nothing else moved
    assert not ids.duplicates(_items(work))


def test_repair_between_two_published_later_created_loses(tmp_path):
    work, _ = _repo(tmp_path)
    _item(work, "bug", "x", created="2026-10-02", id="JB-1")
    _item(work, "bug", "y", created="2026-10-01", id="JB-2")
    _commit_push(work, "both")
    # a merge left both holding JB-5 on the ref
    for s, c in (("x", "2026-10-02"), ("y", "2026-10-01")):
        _item(work, "bug", s, created=c, id="JB-5")
    _commit_push(work, "collide")
    rows = ids.repair(str(work))
    assert rows == [("JB-5", "JB-6", "bug-x")]


def test_repair_own_limits_to_the_writers_files(tmp_path):
    work, _ = _repo(tmp_path)
    _item(work, "bug", "theirs", id="JB-1")
    _item(work, "bug", "mine", created="2026-10-09", id="JB-1")
    assert ids.repair(str(work), own=["bug-theirs-not"]) == []
    assert ids.repair(str(work), own=["bug-mine"]) == [("JB-1", "JB-2", "bug-mine")]


# --- assign_missing ---------------------------------------------------------
def test_assign_missing_order_and_untouched_fields(tmp_path):
    work, _ = _repo(tmp_path)
    _item(work, "bug", "z-old", created="2026-09-01")
    _item(work, "idea", "b-same-day-first", created="2026-10-01")
    _commit_push(work, "first")
    _item(work, "feature", "a-same-day-later", created="2026-10-01")
    _commit_push(work, "second")
    _item(work, "ticket", "unpushed", created="2026-10-01")
    _item(work, "pathway", "has", created="2026-08-01", id="JB-3", destination="d",
          status="active")
    snap = {p.name: p.read_text() for p in (work / "map").glob("*.md")}
    rows = ids.assign_missing(str(work))
    assert rows == [("JB-4", "bug-z-old"), ("JB-5", "idea-b-same-day-first"),
                    ("JB-6", "feature-a-same-day-later"), ("JB-7", "ticket-unpushed")]
    for p in (work / "map").glob("*.md"):
        text = p.read_text()
        sid = load_item(str(p)).short_id
        assert text == "---\nid: %s\n" % sid + snap[p.name][len("---\n"):] or p.name == "pathway-has.md"
        assert load_item(str(p)).meta["updated"] == "2026-10-01T00:00:00Z"
    assert ids.assign_missing(str(work)) == []


# --- check ------------------------------------------------------------------
def test_check_reports_each_class(tmp_path):
    work, _ = _repo(tmp_path)
    _item(work, "bug", "a", id="JB-1")
    _item(work, "bug", "renamed", id="JB-2")
    _item(work, "bug", "handedit", id="JB-3")
    _commit_push(work, "pub")
    _item(work, "bug", "dup", id="JB-1")
    _item(work, "bug", "bad", id="jb-9")
    _item(work, "bug", "none")
    _item(work, "bug", "handedit", id="JB-30")  # changed with no renumber commit
    _item(work, "bug", "renamed", id="JB-20")
    _git(work, "add", "map/bug-renamed.md")  # only this file rides the renumber commit
    _git(work, "commit", "-q", "-m", "map: renumber JB-2 -> JB-20 T renamed")
    p = ids.check(str(work))
    assert p["duplicate"] == ["JB-1: bug-a, bug-dup"]
    assert len(p["malformed"]) == 1 and p["malformed"][0].startswith("bug-bad")
    assert len(p["changed"]) == 1 and p["changed"][0].startswith("bug-handedit")
    assert p["missing"] == ["bug-none"]


def test_cli_check_exit_codes(tmp_path):
    from mapkit.__main__ import main
    work, _ = _repo(tmp_path)
    _item(work, "bug", "a", id="JB-1")
    _item(work, "bug", "none")
    assert main(["ids", "--check", "--repo", str(work)]) == 0  # missing only: informational
    _item(work, "bug", "b", id="JB-1")
    assert main(["ids", "--repo", str(work)]) == 1


# --- resolve ----------------------------------------------------------------
def test_resolve_precedence_and_ambiguity(tmp_path):
    (tmp_path / "map").mkdir()
    _item(tmp_path, "bug", "cone-margin", id="JB-4")
    _item(tmp_path, "idea", "12", id="JB-5")       # an all-digit slug
    _item(tmp_path, "feature", "x", id="JB-12")
    _item(tmp_path, "bug", "shared", id="JB-7")
    _item(tmp_path, "idea", "shared", id="JB-8")
    items = load_items(str(tmp_path / "map"))
    r = lambda t, **k: ids.stem(ids.resolve(items, t, **k))  # noqa: E731
    assert r("bug-cone-margin") == r("cone-margin") == r("JB-4") == r("jb4") == "bug-cone-margin"
    assert r("bug-cone-margin.md") == "bug-cone-margin"
    with pytest.raises(ids.IdError, match="write commands need"):
        ids.resolve(items, "4")
    assert r("4", allow_bare=True) == "bug-cone-margin"
    with pytest.raises(ids.IdError, match="both the slug"):
        ids.resolve(items, "12", allow_bare=True)
    assert r("12") == "idea-12"  # a write: the digits are only a slug
    with pytest.raises(ids.IdError, match="ambiguous"):
        ids.resolve(items, "shared")
    with pytest.raises(ids.IdError, match="no Item"):
        ids.resolve(items, "JB-99")


def test_resolve_refuses_a_duplicated_id(tmp_path):
    (tmp_path / "map").mkdir()
    _item(tmp_path, "bug", "a", id="JB-1")
    _item(tmp_path, "bug", "b", id="JB-1")
    with pytest.raises(ids.IdError, match="--repair"):
        ids.resolve(load_items(str(tmp_path / "map")), "JB-1")


def test_set_id_is_a_one_line_text_edit(tmp_path):
    # Hand- or app-written files keep their quoting and gain no loader defaults.
    (tmp_path / "map").mkdir()
    p = tmp_path / "map" / "pathway-q.md"
    text = ("---\nkind: pathway\ntitle: 'Quoted: title'\nstatus: active\n"
            "created: '2026-10-01'\ndestination: >-\n  line one\n\n  id: not a key\n---\n\nBody\r\nCRLF\r\n")
    p.write_bytes(text.encode())
    ids.set_id(load_item(str(p)), "JB-5")
    assert p.read_bytes().decode() == "---\nid: JB-5\n" + text[4:]
    ids.set_id(load_item(str(p)), "JB-6")  # renumber replaces, never duplicates
    assert p.read_bytes().decode() == "---\nid: JB-6\n" + text[4:]
    assert load_item(str(p)).short_id == "JB-6"


def test_ref_ids_reads_only_the_frontmatter_key(tmp_path):
    # A column-0 'id: JB-n' in a body (a fenced example) must not inflate allocate.
    work, _ = _repo(tmp_path)
    p = _item(work, "bug", "a", id="JB-2")
    p.write_text(p.read_text() + "\n```yaml\nid: JB-900\n---\n```\n")
    _item(work, "bug", "noid")
    q = work / "map" / "bug-noid.md"
    q.write_text(q.read_text() + "\nid: JB-800\n")
    _commit_push(work, "bodies")
    assert ids.ref_ids(str(work)) == {"bug-a": "JB-2"}
    assert ids.allocate(str(work)) == "JB-3"


# --- build-review fix 4 (2026-10-26): one malformed Item never stops an ID write ----
def _broken(root, name="bug-broken", sid="JB-9"):
    path = root / "map" / (name + ".md")
    path.write_text("---\nid: %s\nkind: bug\nstatus: bogus\n---\n\nbody\n" % sid)
    return path


def test_allocate_skips_a_malformed_file_but_counts_its_id(tmp_path, capsys):
    work, _ = _repo(tmp_path)
    _item(work, "bug", "ok", id="JB-2")
    _broken(work)
    assert ids.allocate(str(work)) == "JB-10"
    assert "skipping unreadable Item map/bug-broken.md" in capsys.readouterr().err


def test_repair_and_assign_missing_survive_a_malformed_sibling(tmp_path):
    work, _ = _repo(tmp_path)
    _item(work, "bug", "a", id="JB-1")
    _item(work, "bug", "b", id="JB-1", created="2026-10-05")
    _item(work, "bug", "c")  # no id
    _broken(work, sid="JB-7")
    rows = ids.repair(str(work))
    assert [(r[0], r[2]) for r in rows] == [("JB-1", "bug-b")] and rows[0][1] == "JB-8"
    assert ids.assign_missing(str(work)) == [("JB-9", "bug-c")]
    assert "id: JB-7" in (work / "map" / "bug-broken.md").read_text()  # untouched


def test_check_stays_strict_about_a_malformed_item(tmp_path):
    work, _ = _repo(tmp_path)
    _broken(work)
    from mapkit.items import ItemError
    with pytest.raises(ItemError):
        ids.check(str(work))
