"""mapkit.items: map/<kind>-<slug>.md loading and validation."""
from __future__ import annotations

import pytest

from mapkit.items import (KINDS, REQUIRED_FIELDS, STATUS_SETS, ItemError, item_uuid,
                          load_item, load_items)

FIXTURES = {
    "pathway-two-ball-oval.md": "kind: pathway\ntitle: Two-ball oval\nstatus: active\n"
                                "created: 2026-10-14\nupdated: 2026-10-24T10:00:00+11:00\n"
                                "destination: two balls in an oval\nowners: [Harrison]\n",
    "idea-shake-test.md": "kind: idea\ntitle: Shake test\nstatus: inbox\ncreated: 2026-10-20\n"
                          "filed_by: claude\npriority: low\n",
    "feature-trail-overlay.md": "kind: feature\ntitle: Trails\nstatus: backlog\n"
                                "created: 2026-10-20\nparent: two-ball-oval\n"
                                "blocked_by: [two-ball-oval.md]\n",
    "bug-feed-y-bias.md": "kind: bug\ntitle: Feed y bias\nstatus: in_progress\n"
                          "created: 2026-10-21\nparent: two-ball-oval\npriority: high\n",
    "ticket-p3-design.md": "kind: ticket\ntitle: P3 design\nstatus: open\ncreated: 2026-10-24\n"
                           "parent: two-ball-oval\ntype: grilling\nrank: 1\n",
}


def _write(d, name, meta, body="Body text.\n"):
    p = d / name
    p.write_text("---\n" + meta + "---\n" + body, encoding="utf-8")
    return p


def test_every_kind_loads(tmp_path):
    """One fixture per kind loads; slug strips the kind prefix; dates become ISO strings;
    a pathway defaults branch to main; non-markdown files are ignored."""
    for name, meta in FIXTURES.items():
        _write(tmp_path, name, meta)
    (tmp_path / "notes.txt").write_text("ignored", encoding="utf-8")
    (tmp_path / "config.yaml").write_text("a: 1\n", encoding="utf-8")
    items = load_items(str(tmp_path))
    assert sorted(i.kind for i in items) == sorted(KINDS)
    by = {i.slug: i for i in items}
    assert set(by) == {"two-ball-oval", "shake-test", "trail-overlay", "feed-y-bias",
                       "p3-design"}
    oval = by["two-ball-oval"]
    assert oval.meta["created"] == "2026-10-14"
    assert isinstance(oval.meta["updated"], str) and oval.meta["updated"].startswith("2026-10-24T10")
    assert oval.meta["branch"] == "main"
    assert oval.body == "Body text.\n"


def test_kind_filename_mismatch_rejected(tmp_path):
    """A frontmatter kind that differs from the filename prefix is rejected."""
    p = _write(tmp_path, "bug-x.md", FIXTURES["idea-shake-test.md"])
    with pytest.raises(ItemError, match="does not match filename prefix"):
        load_item(str(p))


def test_unknown_prefix_rejected(tmp_path):
    """A filename that starts with no known kind is rejected."""
    p = _write(tmp_path, "epic-x.md", "kind: epic\ntitle: x\nstatus: open\ncreated: 2026-10-24\n")
    with pytest.raises(ItemError):
        load_item(str(p))


def test_bad_status_rejected(tmp_path):
    """A status outside the kind's set is rejected (ticket statuses are not inbox statuses)."""
    p = _write(tmp_path, "ticket-x.md", FIXTURES["ticket-p3-design.md"].replace(
        "status: open", "status: backlog"))
    with pytest.raises(ItemError, match="status"):
        load_item(str(p))


def test_missing_required_and_bad_typed_fields_rejected(tmp_path):
    """Missing required fields, a bad priority, ticket type or non-int rank are rejected."""
    cases = [
        ("pathway-x.md", FIXTURES["pathway-two-ball-oval.md"].replace(
            "destination: two balls in an oval\n", "")),
        ("idea-x.md", FIXTURES["idea-shake-test.md"].replace("priority: low", "priority: urgent")),
        ("ticket-x.md", FIXTURES["ticket-p3-design.md"].replace("type: grilling", "type: chore")),
        ("ticket-y.md", FIXTURES["ticket-p3-design.md"].replace("rank: 1", "rank: first")),
        ("bug-x.md", FIXTURES["bug-feed-y-bias.md"] + "owners: Harrison\n"),
    ]
    survivors = []
    for name, meta in cases:
        p = _write(tmp_path, name, meta)
        try:
            load_item(str(p))
            survivors.append(name)
        except ItemError:
            pass
    assert not survivors, survivors


def test_uuid_stable():
    """item_uuid is uuid5 of the slug under a fixed namespace: deterministic and slug-distinct."""
    assert item_uuid("two-ball-oval") == item_uuid("two-ball-oval")
    assert item_uuid("two-ball-oval") != item_uuid("two-ball-ovals")
    assert item_uuid("two-ball-oval") == "%s" % item_uuid("two-ball-oval")
    import uuid
    from mapkit.items import ITEM_NAMESPACE
    assert item_uuid("a") == str(uuid.uuid5(ITEM_NAMESPACE, "a"))
    assert str(ITEM_NAMESPACE) == "6f1c3b0e-5d7a-5e2b-9a61-4a1f2c8d9e30"


def test_constants_cover_every_kind():
    """REQUIRED_FIELDS and STATUS_SETS have one entry per kind and every kind requires
    kind/title/status/created."""
    assert set(REQUIRED_FIELDS) == set(KINDS) == set(STATUS_SETS)
    for k in KINDS:
        assert {"kind", "title", "status", "created"} <= set(REQUIRED_FIELDS[k])
    assert "destination" in REQUIRED_FIELDS["pathway"]


def test_missing_map_dir_is_empty(tmp_path):
    """A map directory that does not exist yields no items."""
    assert load_items(str(tmp_path / "map")) == []


def test_readability_rule_clauses():
    from mapkit.items import (BLOCK_CAP, HEADINGLESS_CAP, PROGRESS_CAP,
                              readability_violations)

    word = "x" * 9 + " "  # 10 chars
    long_para = word * (BLOCK_CAP // 10 + 1)
    assert readability_violations("Short.\n\n- a\n- b\n") == []
    # a paragraph wrapped over several lines is still one block
    assert readability_violations("\n".join([word * 20] * 3))
    # a list item's continuation lines count with it; separate items do not add up
    assert readability_violations("- " + word * 30 + "\n  " + word * 30 + "\n")
    assert readability_violations("- " + word * 30 + "\n- " + word * 30 + "\n") == []
    # tables and fenced code are exempt
    assert readability_violations("| " + long_para + " |\n") == []
    assert readability_violations("```\n" + long_para + "\n```\n") == []
    assert readability_violations("~~~\n" + long_para + "\n~~~\n") == []
    # Progress lines have their own cap and do not count toward the body
    prog = "## Progress\n\n- 2026-10-30: " + "y" * PROGRESS_CAP + "\n"
    assert any("Progress line" in v for v in readability_violations(prog))
    assert readability_violations("## Progress\n\n- 2026-10-30: " + word * 30 + "\n") == []
    # a sub-heading stays inside Progress (as for _progress_dates)
    assert any("Progress line" in v for v in readability_violations(
        "## Progress\n\n### Sitting\n\n- 2026-10-30: " + "y" * (PROGRESS_CAP + 50) + "\n"))
    # a long body needs a ## heading
    paras = "\n\n".join([word * 40] * (HEADINGLESS_CAP // 400 + 1))
    assert any("no ## heading" in v for v in readability_violations(paras))
    assert readability_violations("Intro.\n\n## Detail\n\n" + paras) == []
    # only ## counts as the heading; ### alone does not
    assert any("no ## heading" in v for v in readability_violations("Intro.\n\n### Detail\n\n" + paras))


# --- atomic Item writes ------------------------------------------------------
# The map app re-indexes map/ on every change and several writers share it, so a
# truncate-then-write lets a reader see an empty or half-written Item, and a crash
# mid-write destroys it. Writes go to a same-directory temp file, then os.replace.

def _bug(tmp_path, body="Body text.\n"):
    return _write(tmp_path, "bug-feed-y-bias.md", FIXTURES["bug-feed-y-bias.md"], body)


def _leftovers(d, keep):
    return sorted(p.name for p in d.iterdir() if p.name not in keep)


def test_write_text_atomic_round_trips_and_leaves_no_temp(tmp_path):
    from mapkit.atomic import write_text_atomic
    p = tmp_path / "bug-x.md"
    write_text_atomic(str(p), "a\nb\n")
    assert p.read_bytes() == b"a\nb\n"
    write_text_atomic(str(p), "crlf\r\nkept\n", newline="")   # set_id's mode: bytes as given
    assert p.read_bytes() == b"crlf\r\nkept\n"
    write_text_atomic(str(p), "é\n")
    assert p.read_bytes() == "é\n".encode("utf-8")
    assert _leftovers(tmp_path, {"bug-x.md"}) == []


def test_write_text_atomic_keeps_mode_bits(tmp_path):
    import os
    import stat
    from mapkit.atomic import write_text_atomic
    p = tmp_path / "bug-x.md"
    p.write_text("old\n")
    os.chmod(str(p), 0o640)
    write_text_atomic(str(p), "new\n")
    assert stat.S_IMODE(os.stat(str(p)).st_mode) == 0o640
    # a NEW file gets what open(path, "w") would give, not mkstemp's 0600
    q = tmp_path / "bug-new.md"
    umask = os.umask(0o022)
    os.umask(umask)
    write_text_atomic(str(q), "x\n")
    assert stat.S_IMODE(os.stat(str(q)).st_mode) == 0o666 & ~umask


def test_failed_replace_keeps_the_original_and_no_temp(tmp_path, monkeypatch):
    import os
    from mapkit.atomic import write_text_atomic
    p = tmp_path / "bug-x.md"
    p.write_text("original\n")
    seen = []

    def boom(src, dst):
        seen.append(os.path.basename(src))
        with open(src) as fh:
            assert fh.read() == "replacement\n"   # fully written before the swap
        raise OSError("disk gone")
    monkeypatch.setattr(os, "replace", boom)
    with pytest.raises(OSError):
        write_text_atomic(str(p), "replacement\n")
    assert p.read_text() == "original\n"
    assert _leftovers(tmp_path, {"bug-x.md"}) == []
    # the temp name can never be loaded as an Item: every map/ loader takes *.md only
    assert seen and seen[0].startswith(".") and not seen[0].endswith(".md")


def test_write_item_and_set_id_are_atomic(tmp_path, monkeypatch):
    import os
    from mapkit.ids import set_id
    from mapkit.items import write_item
    p = _bug(tmp_path)
    before = p.read_text()
    item = load_item(str(p))

    def boom(src, dst):
        raise OSError("disk gone")
    monkeypatch.setattr(os, "replace", boom)
    with pytest.raises(OSError):
        write_item(str(p), dict(item.meta, title="Changed"), item.body)
    with pytest.raises(OSError):
        set_id(item, "BUG-9")
    assert p.read_text() == before
    assert _leftovers(tmp_path, {p.name}) == []
    monkeypatch.undo()
    set_id(item, "BUG-9")
    assert p.read_text().startswith("---\nid: BUG-9\n")
    assert write_item(str(p), dict(load_item(str(p)).meta, title="Changed"), item.body)
    assert load_item(str(p)).meta["title"] == "Changed"
    assert _leftovers(tmp_path, {p.name}) == []
