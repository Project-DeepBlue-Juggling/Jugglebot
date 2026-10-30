"""Duplicate / near-duplicate Item titles (``mapkit.similar``) and the ``item new`` refusal.

The fixture titles are the 2026-10-30 cases: JB-103 filed four times from the
Robot tab (one truncated), and a short reword of the sentence-length JB-101.
"""
from __future__ import annotations

import pytest

from mapkit import itemcli, similar
from mapkit.items import load_items, write_item

RECORDS = ("With STATE MACHINE window minimized, the Records window doesn't snap to the "
           "left; it remains floating")
TICKS = ("Some GUI chart zoom levels result in only  a single tick mark being visible. "
         "There should always be at least two, to give scale")


def _write(root, kind, slug, sid, title, status="inbox"):
    meta = {"id": sid, "kind": kind, "title": title, "status": status, "created": "2026-10-01",
            "updated": "2026-10-01T00:00:00Z", "priority": "medium"}
    if kind == "pathway":
        meta.update(status="active", destination="reach the goal")
    write_item(str(root / "map" / ("%s-%s.md" % (kind, slug))), meta, "Body\n")


@pytest.fixture
def repo(tmp_path):
    (tmp_path / "map").mkdir()
    (tmp_path / "docs").mkdir()
    for b in ("active", "parked", "archived"):
        (tmp_path / "plans" / b).mkdir(parents=True)
    (tmp_path / "GLOSSARY.md").write_text("# G\n\n## Language\n\n### Robot\n\n**Hand**: The cup.\n")
    _write(tmp_path, "bug", "records", "JB-103", RECORDS)
    _write(tmp_path, "bug", "ticks", "JB-101", TICKS)
    _write(tmp_path, "pathway", "oval", "JB-40", "Two-ball oval")
    _write(tmp_path, "pathway", "stack", "JB-41", "Two-ball skill stack", )
    _write(tmp_path, "bug", "old", "JB-7", "Hand clock drifts", status="complete")
    return tmp_path


def _items(root):
    return load_items(str(root / "map"))


@pytest.mark.parametrize("title", [
    RECORDS,
    RECORDS.upper(),
    "  with state machine window minimized -- the records window doesn't snap to the left, "
    "it remains floating!",
])
def test_same_title_as_an_open_item_is_a_duplicate(repo, title):
    dup, near = similar.screen(_items(repo), title)
    assert dup is not None and dup.item.short_id == "JB-103" and dup.exact
    assert all(m.item.short_id != "JB-103" for m in near)


@pytest.mark.parametrize("title,expect", [
    (RECORDS[:-1], "JB-103"),  # the truncated first filing ("floatin")
    ("Records window stays floating when the state machine window is minimized", "JB-103"),
    ("Minimizing the state machine window leaves Records floating", "JB-103"),
    ("Chart shows a single tick mark at some zoom levels", "JB-101"),  # containment
])
def test_near_titles_are_similar_not_duplicates(repo, title, expect):
    dup, near = similar.screen(_items(repo), title)
    assert dup is None
    assert [m.item.short_id for m in near][:1] == [expect]


@pytest.mark.parametrize("title", [
    "Two-ball columns",              # shares two words with JB-40/41: below every bar
    "Records page loads slowly",
    "Wave camera drops the first clip",
])
def test_unrelated_titles_pass(repo, title):
    assert similar.screen(_items(repo), title) == (None, [])


def test_same_title_as_a_closed_item_is_only_similar(repo):
    dup, near = similar.screen(_items(repo), "Hand clock drifts")
    assert dup is None
    assert [(m.item.short_id, m.exact, m.closed) for m in near] == [("JB-7", True, True)]


def test_exclude_skips_the_item_itself(repo):
    assert similar.screen(_items(repo), RECORDS, exclude="bug-records") == (None, [])


def test_as_dict_is_the_wire_shape(repo):
    _, near = similar.screen(_items(repo), RECORDS[:-1])
    assert similar.as_dict(near[0]) == {
        "short_id": "JB-103", "stem": "bug-records", "kind": "bug", "title": RECORDS,
        "status": "inbox", "exact": False, "score": near[0].ratio.__round__(2)}


def test_item_new_refuses_a_duplicate_even_with_allow_similar(repo):
    with pytest.raises(itemcli.ItemCliError, match="duplicate of JB-103"):
        itemcli.new_item(str(repo), "idea", RECORDS.lower(), allow_similar=True)
    assert len(_items(repo)) == 5  # nothing written


def test_item_new_refuses_similar_unless_allowed(repo):
    with pytest.raises(itemcli.ItemCliError, match=r"similar Items exist.*\n  JB-103 \(inbox\)"):
        itemcli.new_item(str(repo), "bug", RECORDS[:-1])
    assert len(_items(repo)) == 5
    it = itemcli.new_item(str(repo), "bug", RECORDS[:-1], allow_similar=True)
    assert it.short_id == "JB-104"


def test_cli_exit_code_and_flag(repo, capsys):
    import argparse
    parser = argparse.ArgumentParser()
    itemcli.register(parser.add_subparsers())
    args = parser.parse_args(["item", "new", "bug", RECORDS[:-1], "--repo", str(repo)])
    assert args.func(args) == 2
    assert "similar Items exist" in capsys.readouterr().err
    args = parser.parse_args(["item", "new", "bug", RECORDS[:-1], "--repo", str(repo),
                              "--allow-similar"])
    assert args.func(args) == 0
