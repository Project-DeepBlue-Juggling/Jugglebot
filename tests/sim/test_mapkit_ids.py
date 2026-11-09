"""mapkit.ids: format, allocate, duplicates, check, repair, assign_missing, resolve, the frozen crosswalk.

Each git test builds a working clone of a bare remote under tmp_path, so
``origin/main`` is a real published ref. No ports; safe under xdist.
"""
from __future__ import annotations

import csv
import subprocess
from pathlib import Path

import pytest

from mapkit import ids
from mapkit.items import ItemError, load_item, load_items, write_item


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



def test_malformed_id_still_loads(tmp_path):
    (tmp_path / "map").mkdir()
    p = _item(tmp_path, "bug", "b", id=12)  # YAML int, not the canonical string
    it = load_item(str(p))
    assert it.short_id is None and ids.malformed([it]) == [it]


def test_write_item_puts_id_first(tmp_path):
    (tmp_path / "map").mkdir()
    p = _item(tmp_path, "bug", "b", id="BUG-4")
    assert p.read_text().startswith("---\nid: BUG-4\nkind: bug\n")
    assert load_item(str(p)).short_id == "BUG-4"


# --- allocate / duplicates --------------------------------------------------
def test_allocate_empty_map_is_one(tmp_path):
    work, _ = _repo(tmp_path)
    _crosswalk(work)
    assert ids.allocate(str(work), kind="bug") == "BUG-1"


def test_allocate_takes_max_over_tree_and_ref(tmp_path):
    work, _ = _repo(tmp_path)
    _crosswalk(work)
    _item(work, "bug", "a", id="BUG-3")
    _commit_push(work, "a")
    _git(work, "rm", "-q", "map/bug-a.md")  # gone locally, still on origin/main
    _item(work, "bug", "b", id="BUG-2")
    assert ids.allocate(str(work), kind="bug") == "BUG-4"
    _item(work, "bug", "c", id="BUG-9")
    assert ids.allocate(str(work), kind="bug") == "BUG-10"


def test_duplicates_lists_every_member(tmp_path):
    (tmp_path / "map").mkdir()
    for s in ("a", "b", "c"):
        _item(tmp_path, "bug", s, id="BUG-1" if s != "c" else "BUG-2")
    d = ids.duplicates(load_items(str(tmp_path / "map")))
    assert list(d) == ["BUG-1"] and [i.slug for i in d["BUG-1"]] == ["a", "b"]


# --- repair -----------------------------------------------------------------
def test_repair_renumbers_the_unpushed_item(tmp_path):
    work, _ = _repo(tmp_path)
    _crosswalk(work)
    _item(work, "bug", "pub", created="2026-10-05", id="BUG-1")
    _commit_push(work, "pub")
    _item(work, "bug", "local", created="2026-10-01", id="BUG-1")  # older, but unpublished
    before = (work / "map" / "bug-local.md").read_text()
    rows = ids.repair(str(work))
    assert rows == [("BUG-1", "BUG-2", "bug-local")]
    after = (work / "map" / "bug-local.md").read_text()
    assert after == before.replace("id: BUG-1", "id: BUG-2")  # nothing else moved
    assert not ids.duplicates(_items(work))


def test_repair_between_two_published_later_created_loses(tmp_path):
    work, _ = _repo(tmp_path)
    _crosswalk(work)
    _item(work, "bug", "x", created="2026-10-02", id="BUG-1")
    _item(work, "bug", "y", created="2026-10-01", id="BUG-2")
    _commit_push(work, "both")
    # a merge left both holding BUG-5 on the ref
    for s, c in (("x", "2026-10-02"), ("y", "2026-10-01")):
        _item(work, "bug", s, created=c, id="BUG-5")
    _commit_push(work, "collide")
    rows = ids.repair(str(work))
    assert rows == [("BUG-5", "BUG-6", "bug-x")]


def test_repair_own_limits_to_the_writers_files(tmp_path):
    work, _ = _repo(tmp_path)
    _crosswalk(work)
    _item(work, "bug", "theirs", id="BUG-1")
    _item(work, "bug", "mine", created="2026-10-09", id="BUG-1")
    assert ids.repair(str(work), own=["bug-theirs-not"]) == []
    assert ids.repair(str(work), own=["bug-mine"]) == [("BUG-1", "BUG-2", "bug-mine")]


# --- assign_missing ---------------------------------------------------------
def test_assign_missing_order_and_untouched_fields(tmp_path):
    work, _ = _repo(tmp_path)
    _crosswalk(work)
    _item(work, "bug", "z-old", created="2026-09-01")
    _item(work, "idea", "b-same-day-first", created="2026-10-01")
    _commit_push(work, "first")
    _item(work, "feature", "a-same-day-later", created="2026-10-01")
    _commit_push(work, "second")
    _item(work, "ticket", "unpushed", created="2026-10-01")
    _item(work, "pathway", "has", created="2026-08-01", id="PWAY-1", destination="d",
          status="active")
    snap = {p.name: p.read_text() for p in (work / "map").glob("*.md")}
    rows = ids.assign_missing(str(work))
    assert rows == [("BUG-1", "bug-z-old"), ("IDEA-1", "idea-b-same-day-first"),
                    ("FEAT-1", "feature-a-same-day-later"), ("TKT-1", "ticket-unpushed")]
    for p in (work / "map").glob("*.md"):
        text = p.read_text()
        sid = load_item(str(p)).short_id
        assert text == "---\nid: %s\n" % sid + snap[p.name][len("---\n"):] or p.name == "pathway-has.md"
        assert load_item(str(p)).meta["updated"] == "2026-10-01T00:00:00Z"
    assert ids.assign_missing(str(work)) == []


# --- check ------------------------------------------------------------------
def test_check_reports_each_class(tmp_path):
    work, _ = _repo(tmp_path)
    _crosswalk(work)
    _item(work, "bug", "a", id="BUG-1")
    _item(work, "bug", "renamed", id="BUG-2")
    _item(work, "bug", "handedit", id="BUG-3")
    _commit_push(work, "pub")
    _item(work, "bug", "dup", id="BUG-1")
    _item(work, "bug", "bad", id="bug-9")
    _item(work, "bug", "none")
    _item(work, "bug", "handedit", id="BUG-30")  # changed with no renumber commit
    _item(work, "bug", "renamed", id="BUG-20")
    _git(work, "add", "map/bug-renamed.md")  # only this file rides the renumber commit
    _git(work, "commit", "-q", "-m", "map: renumber BUG-2 -> BUG-20 T renamed")
    p = ids.check(str(work))
    assert p["duplicate"] == ["BUG-1: bug-a, bug-dup"]
    assert len(p["malformed"]) == 1 and p["malformed"][0].startswith("bug-bad")
    assert len(p["changed"]) == 1 and p["changed"][0].startswith("bug-handedit")
    assert p["missing"] == ["bug-none"]


def test_cli_check_exit_codes(tmp_path):
    from mapkit.__main__ import main
    work, _ = _repo(tmp_path)
    _crosswalk(work)
    _item(work, "bug", "a", id="BUG-1")
    _item(work, "bug", "none")
    assert main(["ids", "--check", "--repo", str(work)]) == 0  # missing only: informational
    _item(work, "bug", "b", id="BUG-1")
    assert main(["ids", "--repo", str(work)]) == 1


# --- resolve ----------------------------------------------------------------
def test_resolve_precedence_and_ambiguity(tmp_path):
    (tmp_path / "map").mkdir()
    _item(tmp_path, "bug", "cone-margin", id="BUG-4")
    _item(tmp_path, "idea", "12", id="IDEA-5")     # an all-digit slug
    _item(tmp_path, "feature", "x", id="FEAT-12")
    _item(tmp_path, "bug", "shared", id="BUG-7")
    _item(tmp_path, "idea", "shared", id="IDEA-8")
    items = load_items(str(tmp_path / "map"))
    r = lambda t, **k: ids.stem(ids.resolve(items, t, **k))  # noqa: E731
    assert r("bug-cone-margin") == r("cone-margin") == r("BUG-4") == r("bug4") == "bug-cone-margin"
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
        ids.resolve(items, "BUG-99")


def test_resolve_refuses_a_duplicated_id(tmp_path):
    (tmp_path / "map").mkdir()
    _item(tmp_path, "bug", "a", id="BUG-1")
    _item(tmp_path, "bug", "b", id="BUG-1")
    with pytest.raises(ids.IdError, match="--repair"):
        ids.resolve(load_items(str(tmp_path / "map")), "BUG-1")


def test_set_id_is_a_one_line_text_edit(tmp_path):
    # Hand- or app-written files keep their quoting and gain no loader defaults.
    (tmp_path / "map").mkdir()
    p = tmp_path / "map" / "pathway-q.md"
    text = ("---\nkind: pathway\ntitle: 'Quoted: title'\nstatus: active\n"
            "created: '2026-10-01'\ndestination: >-\n  line one\n\n  id: not a key\n---\n\nBody\r\nCRLF\r\n")
    p.write_bytes(text.encode())
    ids.set_id(load_item(str(p)), "PWAY-5")
    assert p.read_bytes().decode() == "---\nid: PWAY-5\n" + text[4:]
    ids.set_id(load_item(str(p)), "PWAY-6")  # renumber replaces, never duplicates
    assert p.read_bytes().decode() == "---\nid: PWAY-6\n" + text[4:]
    assert load_item(str(p)).short_id == "PWAY-6"


def test_ref_ids_reads_only_the_frontmatter_key(tmp_path):
    # A column-0 'id: BUG-n' in a body (a fenced example) must not inflate allocate.
    work, _ = _repo(tmp_path)
    _crosswalk(work)
    p = _item(work, "bug", "a", id="BUG-2")
    p.write_text(p.read_text() + "\n```yaml\nid: BUG-900\n---\n```\n")
    _item(work, "bug", "noid")
    q = work / "map" / "bug-noid.md"
    q.write_text(q.read_text() + "\nid: BUG-800\n")
    _commit_push(work, "bodies")
    assert ids.ref_ids(str(work)) == {"bug-a": "BUG-2"}
    assert ids.allocate(str(work), kind="bug") == "BUG-3"


# --- build-review fix 4 (2026-10-26): one malformed Item never stops an ID write ----
def _broken(root, name="bug-broken", sid="BUG-9"):
    path = root / "map" / (name + ".md")
    path.write_text("---\nid: %s\nkind: bug\nstatus: bogus\n---\n\nbody\n" % sid)
    return path


def test_allocate_skips_a_malformed_file_but_counts_its_id(tmp_path, capsys):
    work, _ = _repo(tmp_path)
    _crosswalk(work)
    _item(work, "bug", "ok", id="BUG-2")
    _broken(work)
    assert ids.allocate(str(work), kind="bug") == "BUG-10"
    assert "skipping unreadable Item map/bug-broken.md" in capsys.readouterr().err


def test_repair_and_assign_missing_survive_a_malformed_sibling(tmp_path):
    work, _ = _repo(tmp_path)
    _crosswalk(work)
    _item(work, "bug", "a", id="BUG-1")
    _item(work, "bug", "b", id="BUG-1", created="2026-10-05")
    _item(work, "bug", "c")  # no id
    _broken(work, sid="BUG-7")
    rows = ids.repair(str(work))
    assert [(r[0], r[2]) for r in rows] == [("BUG-1", "bug-b")] and rows[0][1] == "BUG-8"
    assert ids.assign_missing(str(work)) == [("BUG-9", "bug-c")]
    assert "id: BUG-7" in (work / "map" / "bug-broken.md").read_text()  # untouched


def test_check_stays_strict_about_a_malformed_item(tmp_path):
    work, _ = _repo(tmp_path)
    _broken(work)
    with pytest.raises(ItemError):
        ids.check(str(work))



# --- item-id-per-kind: per-kind IDs only (P5) ----------------------------------
# Plan plans/archived/item-id-per-kind.md § 2.1 and § 5. The retired global form is
# known only to the frozen crosswalk; synthetic crosswalks here use a FICTIONAL old
# prefix (OLD) so the code is proved to hold no knowledge of the real one.
_KIND_PREFIX = (("bug", "BUG"), ("feature", "FEAT"), ("idea", "IDEA"),
                ("pathway", "PWAY"), ("ticket", "TKT"))
_REPO_ROOT = str(Path(__file__).resolve().parents[2])
_XW_HEADER = "old_id,new_id,kind,stem\n"


def _crosswalk(work):
    """Put a header-only crosswalk file in ``work`` (allocation no longer reads it)."""
    (work / "mapkit").mkdir(exist_ok=True)
    path = work / "mapkit" / "legacy_ids.csv"
    path.write_text(_XW_HEADER)
    return path


def _write_xw(root, rows, header=_XW_HEADER):
    """Write ``rows`` (4-tuples or raw lists) as the crosswalk CSV under ``root``."""
    (root / "mapkit").mkdir(parents=True, exist_ok=True)
    path = root / "mapkit" / "legacy_ids.csv"
    with open(path, "w", newline="", encoding="utf-8") as fh:
        fh.write(header)
        csv.writer(fh, lineterminator="\n").writerows(rows)
    return path


_SYNTH_XW = [("OLD-2", "BUG-1", "bug", "bug-b-early"),
             ("OLD-9", "BUG-2", "bug", "bug-b-late"),
             ("OLD-3", "FEAT-1", "feature", "feature-f-a"),
             ("OLD-5", "IDEA-1", "idea", "idea-i-a"),
             ("OLD-1", "PWAY-1", "pathway", "pathway-pw"),
             ("OLD-7", "TKT-1", "ticket", "ticket-t-a")]


def _perkind_tree(tmp_path):
    """Six per-kind Items plus the synthetic crosswalk naming them."""
    (tmp_path / "map").mkdir(parents=True, exist_ok=True)
    _item(tmp_path, "idea", "i-a", id="IDEA-1")
    _item(tmp_path, "bug", "b-late", id="BUG-2")
    _item(tmp_path, "bug", "b-early", id="BUG-1")
    _item(tmp_path, "pathway", "pw", id="PWAY-1", destination="d", status="active")
    _item(tmp_path, "ticket", "t-a", id="TKT-1")
    _item(tmp_path, "feature", "f-a", id="FEAT-1")
    _write_xw(tmp_path, _SYNTH_XW)
    return tmp_path


def test_removed_names_are_absent():
    """The global-form API is gone, not merely unused."""
    for name in ("PREFIX", "format_id", "canonical", "parse", "typed_ref", "per_kind",
                 "make_crosswalk", "CrosswalkError"):
        assert not hasattr(ids, name), name
    assert ids.ALL_PREFIXES == ("BUG", "FEAT", "IDEA", "PWAY", "TKT")


def test_prefixes_cover_every_kind():
    """T-U1: one prefix per Item kind, exactly the § 2.1 table."""
    from mapkit.items import KINDS
    assert ids.PREFIXES == dict(_KIND_PREFIX)
    assert set(ids.PREFIXES) == set(KINDS)


def test_parse_ref_canonical_ref_format_ref_per_kind():
    """T-U1: all five prefixes, any case, hyphen optional; BUG-0, BUG-01,
    FEAT3x and a foreign prefix rejected; stored forms are canonical and case-sensitive."""
    for kind, prefix in _KIND_PREFIX:
        want = "%s-12" % prefix
        for text in (want, prefix.lower() + "-12", prefix.lower() + "12",
                     prefix.title() + "-12", " %s12 " % prefix):
            assert ids.parse_ref(text) == want, text
        assert ids.canonical_ref(want) == want
        for loose in (prefix.lower() + "-12", prefix + "12", prefix + "-012"):
            assert ids.canonical_ref(loose) is None, loose
        assert ids.format_ref(kind, 12) == want
    for bad in ("BUG-0", "BUG-01", "FEAT3x", "XB-3", "BUG-", "12", "", "BUGS-3",
                "bug-12-x", "xBUG-9", "FEATURE-3", "OLD-12", "old12", "OLD-012"):
        assert ids.parse_ref(bad) is None, bad
    for bad in ("BUG-0", 12, None, "XB-3", "FEAT-8a", "OLD-12"):
        assert ids.canonical_ref(bad) is None, bad
    assert ids.split_ref("TKT-31") == ("TKT", 31) and ids.split_ref("OLD-149") is None
    assert ids.split_ref("tkt-31") is None
    for kind, n in (("bug", 0), ("bug", True), ("bug", -2), ("bogus", 3), ("Bug", 3), ("feat", 1)):
        with pytest.raises(ValueError):
            ids.format_ref(kind, n)


def test_short_id_and_malformed_are_per_kind_only(tmp_path):
    """T-U5: a per-kind ID is a well-formed short_id; a loose or foreign-prefix one is malformed."""
    (tmp_path / "map").mkdir()
    _item(tmp_path, "bug", "new", id="BUG-3")
    _item(tmp_path, "feature", "other", id="FEAT-3")
    _item(tmp_path, "idea", "loose", id="idea-3")
    _item(tmp_path, "pathway", "foreign", id="OLD-3", destination="d", status="active")
    items = load_items(str(tmp_path / "map"))
    assert [i.short_id for i in items] == ["BUG-3", "FEAT-3", None, None]
    assert sorted(i.slug for i in ids.malformed(items)) == ["foreign", "loose"]


def test_duplicates_are_per_id_not_per_number(tmp_path):
    """T-U5: BUG-3, FEAT-3 and IDEA-3 are three IDs, not one duplicate."""
    (tmp_path / "map").mkdir()
    _item(tmp_path, "bug", "a", id="BUG-3")
    _item(tmp_path, "feature", "b", id="FEAT-3")
    _item(tmp_path, "idea", "c", id="IDEA-3")
    assert ids.duplicates(load_items(str(tmp_path / "map"))) == {}
    _item(tmp_path, "bug", "d", id="BUG-3")
    d = ids.duplicates(load_items(str(tmp_path / "map")))
    assert list(d) == ["BUG-3"] and [i.slug for i in d["BUG-3"]] == ["a", "d"]


def test_mismatch_flags_a_prefix_that_contradicts_the_kind(tmp_path):
    """T-U5: mismatch flags BUG-3 on a feature; no prefix is a wildcard; a
    foreign-prefix value is the malformed class's, not this one;
    ``ids --check`` reports the class and fails on it."""
    for kind, prefix in _KIND_PREFIX:
        assert not ids.mismatch("%s-3" % prefix, kind)
        others = [p for k, p in _KIND_PREFIX if k != kind]
        assert all(ids.mismatch("%s-3" % p, kind) for p in others)
        assert not ids.mismatch("OLD-3", kind)  # not canonical: malformed's, never a wildcard
    # no prefix matches every kind
    for _, prefix in _KIND_PREFIX:
        assert sum(not ids.mismatch("%s-3" % prefix, k) for k, _ in _KIND_PREFIX) == 1
    assert not ids.mismatch("bug-3", "feature")  # not canonical: the malformed class's
    assert not ids.mismatch(None, "bug")
    from mapkit.__main__ import main
    work, _ = _repo(tmp_path)
    _item(work, "feature", "f", id="BUG-3")
    _item(work, "bug", "b", id="BUG-4")
    p = ids.check(str(work))
    assert len(p["mismatch"]) == 1 and p["mismatch"][0].startswith("feature-f")
    assert "BUG-3" in p["mismatch"][0] and "feature" in p["mismatch"][0]
    assert p["malformed"] == [] and p["duplicate"] == []
    assert main(["ids", "--check", "--repo", str(work)]) == 1


@pytest.mark.parametrize("with_crosswalk", [False, True])
def test_allocate_needs_a_kind_with_or_without_a_crosswalk(tmp_path, with_crosswalk):
    """T-U3: always per kind. No ``kind=`` is a ValueError naming the kinds, whether or
    not the crosswalk file exists; a foreign-prefix ``id:`` reserves nothing; every
    result carries one of the five prefixes."""
    work, _ = _repo(tmp_path)
    if with_crosswalk:
        _crosswalk(work)
    _item(work, "bug", "a", id="OLD-4")
    _item(work, "bug", "b", id="BUG-2")
    with pytest.raises(ValueError, match="kind.*bug"):
        ids.allocate(str(work))
    with pytest.raises(ValueError):
        ids.allocate(str(work), ids.DEFAULT_REF, _items(work))
    with pytest.raises(ValueError):
        ids.allocate(str(work), kind="bogus")
    with pytest.raises(ValueError):
        ids.allocate(str(work), kind=None)
    assert ids.allocate(str(work), kind="bug") == "BUG-3"
    for kind in ids.PREFIXES:
        got = ids.allocate(str(work), kind=kind)
        assert got.split("-")[0] in ids.ALL_PREFIXES
        assert ids.split_ref(got) is not None


def test_allocate_per_kind_scopes_the_max(tmp_path):
    """T-U2: filing a bug leaves FEAT's next number unchanged; the max spans the
    tree and origin/main; 1 on an empty kind; no kind is refused."""
    work, _ = _repo(tmp_path)
    _crosswalk(work)
    _item(work, "bug", "a", id="BUG-7")
    _item(work, "feature", "f", id="FEAT-2")
    _commit_push(work, "pub")
    _git(work, "rm", "-q", "map/bug-a.md")  # gone locally, still on origin/main
    _item(work, "bug", "b", id="BUG-3")
    assert ids.allocate(str(work), kind="feature") == "FEAT-3"
    assert ids.allocate(str(work), kind="bug") == "BUG-8"
    _item(work, "bug", "c", id="BUG-8")  # filing a bug ...
    assert ids.allocate(str(work), kind="feature") == "FEAT-3"  # ... moves no other kind
    assert ids.allocate(str(work), ids.DEFAULT_REF, _items(work), kind="bug") == "BUG-9"
    for kind in ("idea", "pathway", "ticket"):
        assert ids.allocate(str(work), kind=kind) == ids.format_ref(kind, 1)
    with pytest.raises(ValueError, match="kind"):
        ids.allocate(str(work))
    with pytest.raises(ValueError):
        ids.allocate(str(work), kind="bogus")


def test_repair_and_assign_missing_stay_within_the_kind(tmp_path):
    """T-U5: two BUG-3 repair within the kind; a backfill numbers per kind."""
    work, _ = _repo(tmp_path)
    _crosswalk(work)
    _item(work, "bug", "pub", created="2026-10-05", id="BUG-3")
    _item(work, "feature", "f", id="FEAT-9")
    _commit_push(work, "pub")
    _item(work, "bug", "local", created="2026-10-01", id="BUG-3")
    _item(work, "idea", "none")
    assert ids.repair(str(work)) == [("BUG-3", "BUG-4", "bug-local")]
    assert ids.assign_missing(str(work)) == [("IDEA-1", "idea-none")]


def test_resolve_takes_typed_per_kind_ids_and_lists_a_shared_bare_number(tmp_path):
    """T-U4: stem, slug, ID order; typed per-kind IDs resolve; bare digits on a read
    return the one match or list every match; bare on a write is refused."""
    (tmp_path / "map").mkdir()
    _item(tmp_path, "bug", "cone", id="BUG-12")
    _item(tmp_path, "feature", "fan", id="FEAT-12")
    _item(tmp_path, "idea", "solo", id="IDEA-4")
    _item(tmp_path, "pathway", "big", id="PWAY-149", destination="d", status="active")
    items = load_items(str(tmp_path / "map"))
    r = lambda t, **k: ids.stem(ids.resolve(items, t, **k))  # noqa: E731
    assert r("BUG-12") == r("bug12") == r("Bug-12") == "bug-cone"
    assert r("FEAT-12") == r("feat12") == "feature-fan"
    assert r("PWAY-149") == r("pway149") == "pathway-big"
    assert r("4", allow_bare=True) == "idea-solo"
    assert r("149", allow_bare=True) == "pathway-big"
    with pytest.raises(ids.IdError, match="BUG-12.*FEAT-12") as exc:
        ids.resolve(items, "12", allow_bare=True)
    assert [ids.stem(i) for i in exc.value.matches] == ["bug-cone", "feature-fan"]
    with pytest.raises(ids.IdError, match="write commands need.*e.g. BUG-12"):
        ids.resolve(items, "4")
    with pytest.raises(ids.IdError, match="no Item"):
        ids.resolve(items, "TKT-12")


def test_resolve_refuses_a_stem_that_is_another_items_id(tmp_path):
    """T-U4 / § 2.1: ``bug-12`` is a legal stem shape and a typed ID; when the
    stem and the ID name different Items, resolve errors instead of guessing."""
    (tmp_path / "map").mkdir()
    _item(tmp_path, "bug", "12", id="BUG-40")   # stem bug-12
    _item(tmp_path, "bug", "other", id="BUG-12")
    _item(tmp_path, "idea", "3", id="IDEA-3")   # stem idea-3 IS IDEA-3: no conflict
    items = load_items(str(tmp_path / "map"))
    with pytest.raises(ids.IdError, match="stem of bug-12.*ID of bug-other"):
        ids.resolve(items, "bug-12")
    assert ids.stem(ids.resolve(items, "BUG-12")) == "bug-other"  # not a stem: case-sensitive
    assert ids.stem(ids.resolve(items, "idea-3")) == "idea-3"
    assert ids.stem(ids.resolve(items, "bug-other")) == "bug-other"


# --- the frozen crosswalk and D-3 legacy resolution --------------------------
def test_legacy_is_empty_without_the_file_and_full_with_it(tmp_path):
    root = _perkind_tree(tmp_path)
    assert ids.legacy(str(tmp_path / "nowhere")) == {}
    (root / "mapkit" / "legacy_ids.csv").unlink()
    assert ids.legacy(str(root)) == {}
    _write_xw(root, _SYNTH_XW)
    assert ids.legacy(str(root)) == {"OLD-2": "BUG-1", "OLD-9": "BUG-2", "OLD-3": "FEAT-1",
                                     "OLD-5": "IDEA-1", "OLD-1": "PWAY-1", "OLD-7": "TKT-1"}


@pytest.mark.parametrize("what,header,rows,line,match", [
    ("header", "old_id,new_id\n", [], 1, "header"),
    ("empty-file", "", [], 1, "header"),
    ("short-row", _XW_HEADER, [("OLD-1", "BUG-1", "bug")], 2, "4 non-empty cells"),
    ("long-row", _XW_HEADER, [("OLD-1", "BUG-1", "bug", "bug-x", "extra")], 2, "4 non-empty cells"),
    ("empty-cell", _XW_HEADER, [("OLD-1", "BUG-1", "bug", "")], 2, "4 non-empty cells"),
    ("wrong-kind", _XW_HEADER, [("OLD-1", "BUG-1", "feature", "feature-x")], 2, "not a canonical ID of kind"),
    ("non-canonical", _XW_HEADER, [("OLD-1", "BUG-01", "bug", "bug-x")], 2, "not a canonical ID of kind"),
    ("foreign-new", _XW_HEADER, [("OLD-1", "OLD-2", "bug", "bug-x")], 2, "not a canonical ID of kind"),
    ("duplicate-old", _XW_HEADER,
     [("OLD-1", "BUG-1", "bug", "bug-x"), ("OLD-1", "BUG-2", "bug", "bug-y")], 3, "listed twice"),
])
def test_legacy_refuses_a_malformed_file_naming_file_and_line(tmp_path, what, header, rows, line, match):
    _write_xw(tmp_path, rows, header=header)
    with pytest.raises(ItemError) as exc:
        ids.legacy(str(tmp_path))
    msg = str(exc.value)
    assert msg.startswith("mapkit/legacy_ids.csv:%d:" % line), msg
    assert match in msg, msg


def test_legacy_ref_takes_any_spelling_of_an_old_id():
    lg = {"OLD-12": "BUG-1", "OLD-3": "FEAT-1"}
    for text in ("OLD-12", "old-12", "old12", "OLD-012", "Old12", " old-12 ", "OLD0012"):
        assert ids.legacy_ref(text, lg) == "BUG-1", text
    assert ids.legacy_ref("old3", lg) == "FEAT-1"
    for text in ("OLD-13", "OLD-0", "OLD-", "12", "", "BUG-1", "bug1", "OLD-12x", "XYZ-12"):
        assert ids.legacy_ref(text, lg) is None, text
    # a current ID is in no row, even when the crosswalk is full of them as values
    assert ids.legacy_ref("BUG-1", lg) is None
    assert ids.legacy_ref("OLD-12", {}) is None
    assert ids.legacy_ref("OLD-12", None) is None
    for not_text in (None, 12, b"OLD-12", ["OLD-12"]):
        assert ids.legacy_ref(not_text, lg) is None


def test_legacy_ref_knows_no_prefix_of_its_own():
    """Whatever letters the crosswalk's keys carry are the old form."""
    assert ids.legacy_ref("zq-5", {"ZQ-5": "TKT-2"}) == "TKT-2"
    assert ids.legacy_ref("OLD-5", {"ZQ-5": "TKT-2"}) is None


def test_check_is_clean_on_a_per_kind_tree_with_the_crosswalk(tmp_path):
    from mapkit.__main__ import main
    root = _perkind_tree(tmp_path)
    p = ids.check(str(root), items=load_items(str(root / "map")))
    assert p["duplicate"] == p["malformed"] == p["mismatch"] == []
    assert main(["ids", "--check", "--repo", str(root), "--ref", "HEAD"]) in (0, 1)  # no git: ref absent
    assert ids.legacy(str(root))["OLD-9"] == "BUG-2"


def test_resolve_legacy_reads_translate_and_writes_refuse(tmp_path):
    root = _perkind_tree(tmp_path)
    items = load_items(str(root / "map"))
    lg = ids.legacy(str(root))
    for spelling in ("OLD-9", "old-9", "old9", "OLD-009"):
        assert ids.stem(ids.resolve(items, spelling, allow_bare=True, legacy=lg)) == "bug-b-late"
        with pytest.raises(ids.LegacyIdError,
                           match=r"%s is now BUG-2 \(bug-b-late\); writes take the new ID" % spelling) as exc:
            ids.resolve(items, spelling, legacy=lg)
        assert exc.value.new_id == "BUG-2"
    assert ids.stem(ids.resolve(items, "BUG-2", legacy=lg)) == "bug-b-late"
    assert ids.stem(ids.resolve(items, "bug-b-late", legacy=lg)) == "bug-b-late"
    with pytest.raises(ids.IdError, match="no Item"):
        ids.resolve(items, "OLD-77", allow_bare=True, legacy=lg)
    with pytest.raises(ids.IdError, match="no Item"):
        ids.resolve(items, "OLD-77", legacy=lg)


def test_resolve_without_a_legacy_map_is_no_item(tmp_path):
    root = _perkind_tree(tmp_path)
    items = load_items(str(root / "map"))
    for kw in ({}, {"legacy": None}, {"legacy": {}}):
        with pytest.raises(ids.IdError, match="no Item"):
            ids.resolve(items, "OLD-9", allow_bare=True, **kw)
        with pytest.raises(ids.IdError, match="no Item"):
            ids.resolve(items, "OLD-9", **kw)


def test_ids_cli_no_longer_has_make_crosswalk(tmp_path):
    from mapkit.__main__ import main
    for argv in (["ids", "--make-crosswalk", "--repo", str(tmp_path)],
                 ["ids", "--out", str(tmp_path / "x.csv"), "--repo", str(tmp_path)]):
        with pytest.raises(SystemExit) as exc:
            main(argv)
        assert exc.value.code == 2
    assert not (tmp_path / "x.csv").exists()


# --- real data: the frozen file is the record of the retired form ------------
def test_the_real_crosswalk_loads_clean():
    lg = ids.legacy(_REPO_ROOT)
    assert len(lg) == 212
    for old, new in lg.items():
        assert ids.canonical_ref(new) == new, (old, new)


def test_every_real_old_id_is_refused_as_an_id_and_found_as_a_legacy_one():
    lg = ids.legacy(_REPO_ROOT)
    assert lg
    for old, new in lg.items():
        spellings = (old, old.lower(), old.replace("-", ""), old.lower().replace("-", ""))
        for s in spellings:
            assert ids.parse_ref(s) is None, s
            assert ids.canonical_ref(s) is None, s
            assert ids.split_ref(s) is None, s
            assert ids.legacy_ref(s, lg) == new, s
        assert ids.legacy_ref(new, lg) is None, new
        assert ids.parse_ref(new) == new
        assert not ids.mismatch(old, "bug")
    assert {o.split("-")[0] for o in lg}.isdisjoint(ids.ALL_PREFIXES)


def test_a_real_old_id_is_no_item_without_the_crosswalk(tmp_path):
    lg = ids.legacy(_REPO_ROOT)
    (tmp_path / "map").mkdir()
    _item(tmp_path, "bug", "a", id="BUG-1")
    items = load_items(str(tmp_path / "map"))
    for old in lg:
        for kw in ({}, {"legacy": None}, {"legacy": {}}):
            with pytest.raises(ids.IdError, match="no Item"):
                ids.resolve(items, old, allow_bare=True, **kw)
            with pytest.raises(ids.IdError, match="no Item"):
                ids.resolve(items, old.lower(), **kw)


def test_a_real_old_id_on_an_item_is_malformed_and_never_allocated_over(tmp_path):
    old = sorted(ids.legacy(_REPO_ROOT))[0]
    work, _ = _repo(tmp_path)
    _item(work, "bug", "a", id=old)
    items = _items(work)
    assert [i.short_id for i in items] == [None]
    assert ids.malformed(items) == items
    assert ids.allocate(str(work), kind="bug") == "BUG-1"
