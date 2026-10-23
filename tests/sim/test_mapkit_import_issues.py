"""mapkit import-issues against a fake fetch (no network)."""
from __future__ import annotations

import datetime

import pytest

from mapkit.import_issues import import_issues, slugify
from mapkit.items import load_items, write_item

NOW = datetime.datetime(2026, 10, 24, 1, 2, 3)


def _issue(n, title, state, labels=(), comments=(), body="Q body"):
    return {"number": n, "title": title, "body": body, "state": state,
            "labels": [{"name": l} for l in labels],
            "comments": [{"body": c} for c in comments],
            "url": "https://github.com/o/r/issues/%d" % n,
            "createdAt": "2026-10-2%dT05:06:07Z" % (n % 10)}


ISSUES = {
    12: _issue(12, "Second: the thing", "OPEN"),
    10: _issue(10, "First – grill", "CLOSED", ["wayfinder:grilling", "x"],
               ["early", "final answer"]),
    11: _issue(11, "Third", "CLOSED", [], ["ans"]),
}
DEPS = {12: [10, 99], 10: [], 11: []}


def fake_fetch(args):
    if args[0] == "api" and args[1].endswith("/sub_issues"):
        return [{"number": n} for n in (12, 10, 11)]
    if args[0] == "api" and args[1].endswith("/dependencies/blocked_by"):
        k = int(args[1].split("/")[-3])
        return [{"number": n} for n in DEPS[k]]
    if args[0] == "issue":
        return ISSUES[int(args[2])]
    raise AssertionError(args)


def _setup(tmp_path):
    (tmp_path / "map").mkdir()
    write_item(str(tmp_path / "map" / "pathway-p.md"),
               {"kind": "pathway", "title": "P", "status": "active",
                "created": "2026-10-23", "destination": "d"}, "B\n")


def test_import_maps_fields(tmp_path):
    _setup(tmp_path)
    written = import_issues(str(tmp_path), 21, "p", fake_fetch, now=NOW)
    assert [w.name for w in written] == [
        "ticket-first-grill.md", "ticket-third.md", "ticket-second-the-thing.md"]
    items = {i.slug: i for i in load_items(str(tmp_path / "map"))
             if i.kind == "ticket"}
    first, third, second = (items["first-grill"], items["third"],
                            items["second-the-thing"])
    assert first.meta["type"] == "grilling"
    assert first.meta["rank"] == 1 and third.meta["rank"] == 2
    assert second.meta["rank"] == 3
    assert first.status == "complete" and second.status == "open"
    assert third.meta["type"] == "task"
    assert first.body.rstrip().endswith("## Answer\n\nfinal answer")
    assert "## Answer" not in second.body
    assert second.meta["blocked_by"] == ["ticket-first-grill"]
    assert "https://github.com/Project-DeepBlue-Juggling/Jugglebot/issues/99" \
        in second.meta["links"]
    assert second.meta["links"][0] == "https://github.com/o/r/issues/12"
    assert first.meta["created"] == "2026-10-20"
    assert first.meta["updated"] == "2026-10-24T01:02:03Z"
    assert first.meta["owners"] == [] and first.meta["priority"] == "medium"
    assert first.meta["parent"] == "p"


def test_rerun_skips(tmp_path, capsys):
    _setup(tmp_path)
    import_issues(str(tmp_path), 21, "p", fake_fetch, now=NOW)
    p = tmp_path / "map" / "ticket-third.md"
    p.write_text(p.read_text() + "edited\n")
    before = p.read_text()
    assert import_issues(str(tmp_path), 21, "p", fake_fetch, now=NOW) == []
    assert p.read_text() == before
    assert "skip" in capsys.readouterr().err


def test_refuses_without_pathway(tmp_path):
    (tmp_path / "map").mkdir()
    with pytest.raises(SystemExit):
        import_issues(str(tmp_path), 21, "p", fake_fetch, now=NOW)


def test_slugify():
    assert slugify("A – B: C") == "a-b-c"
    assert slugify("  --Hello, Wörld!  ") == "hello-world"
    long = slugify("word " * 30)
    assert len(long) <= 60 and not long.endswith("-") and not long.startswith("-")
    assert slugify("——") == "ticket"


def test_slug_collision(tmp_path):
    _setup(tmp_path)

    def fetch(args):
        if args[0] == "api" and args[1].endswith("/sub_issues"):
            return [{"number": 1}, {"number": 2}]
        if args[0] == "api":
            return []
        return _issue(int(args[2]), "Same title", "OPEN")
    written = import_issues(str(tmp_path), 21, "p", fetch, now=NOW)
    assert [w.name for w in written] == ["ticket-same-title.md",
                                         "ticket-same-title-2.md"]
