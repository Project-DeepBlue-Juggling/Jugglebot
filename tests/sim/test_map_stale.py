"""Staleness of map items: the shared fixture, ``items --stale`` and no-op writes.

The table is the shared fixture (rows numbered; the app's frontend test
encodes the same rows). tmp_path only.
"""
from __future__ import annotations

import datetime
import os

import pytest

from mapkit import itemcli
from mapkit.items import STALE_DAYS, dump_item, is_stale, load_item, stale_clock, write_item

TODAY = datetime.date(2026, 10, 25)

# (row, status, created, progress dates, updated, stale, age)
ROWS = [
    (1, "inbox", "2026-10-01", [], "2026-10-01", False, 24),
    (2, "inbox", "2026-09-20", [], "2026-09-20", True, 35),
    (3, "in_progress", "2026-09-01", ["2026-10-15"], "2026-10-15", False, 10),
    (4, "in_progress", "2026-09-01", ["2026-09-10", "2026-10-01"], "2026-10-01", True, 24),
    (5, "backlog", "2026-08-01", ["2026-08-20"], "2026-08-20", True, 66),
    (6, "backlog", "2026-08-30", [], "2026-08-30", False, 56),
    (7, "complete", "2026-01-01", [], "2026-01-01", False, None),
    (8, "inbox", "2026-09-01", ["2026-10-20"], "2026-09-02", False, 5),
    (9, "inbox", "2026-09-01", [], "2026-10-24", True, 54),
    (10, "inbox", "2026-09-25", [], "2026-09-25", False, 30),
]


def _write(map_dir, row, status, created, progress, updated):
    body = "## Problem\n\nx\n"
    if progress:
        body += "\n## Progress\n\n" + "".join("%s - note\n" % d for d in progress)
    meta = {"kind": "bug", "title": "row %d" % row, "status": status,
            "created": created, "updated": updated + "T00:00:00Z"}
    path = os.path.join(str(map_dir), "bug-row%d.md" % row)
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(dump_item(meta, body))
    return load_item(path)


@pytest.mark.parametrize("row,status,created,progress,updated,stale,age", ROWS,
                         ids=["row%d" % r[0] for r in ROWS])
def test_fixture_row(tmp_path, row, status, created, progress, updated, stale, age):
    item = _write(tmp_path, row, status, created, progress, updated)
    assert is_stale(item, TODAY) is stale
    if age is not None:
        assert (TODAY - stale_clock(item)).days == age


def test_thresholds_are_the_documented_ones():
    assert STALE_DAYS == {"inbox": 30, "backlog": 60, "in_progress": 14}


def test_progress_bullet_and_colon_forms_count(tmp_path):
    # real items write "2026-10-24: ..." and set-status writes "- 2026-10-25: ..."
    body = "## Progress\n\n2026-10-01: a\n- 2026-10-20: b\n"
    path = os.path.join(str(tmp_path), "bug-x.md")
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(dump_item({"kind": "bug", "title": "t", "status": "inbox",
                            "created": "2026-01-01"}, body))
    assert stale_clock(load_item(path)) == datetime.date(2026, 10, 20)


def test_items_stale_lists_exactly_the_stale_rows(tmp_path, monkeypatch, capsys):
    map_dir = tmp_path / "map"
    map_dir.mkdir()
    for r in ROWS:
        _write(map_dir, *r[:5])
    monkeypatch.setattr(itemcli.datetime, "date",
                        type("D", (datetime.date,), {"today": classmethod(lambda c: TODAY)}))
    from mapkit.__main__ import main
    assert main(["items", "--stale", "--repo", str(tmp_path)]) == 0
    out = capsys.readouterr().out.strip().splitlines()
    got = sorted(l.split()[1] for l in out)
    assert got == sorted("row%d" % r[0] for r in ROWS if r[5])
    row2 = next(l for l in out if " row2 " in l)
    assert row2.endswith("stale 35d since 2026-09-20")


def test_noop_write_leaves_file_and_updated_untouched(tmp_path):
    path = os.path.join(str(tmp_path), "bug-x.md")
    meta = {"kind": "bug", "title": "t", "status": "inbox", "created": "2026-01-01",
            "updated": "2026-01-02T00:00:00Z"}
    assert write_item(path, meta, "## Problem\n\nx\n") is True  # first write
    before = open(path, encoding="utf-8").read()
    os.utime(path, (1, 1))
    again = dict(meta, updated="2030-01-01T00:00:00Z")  # a caller bumps to "now"
    assert write_item(path, again, "## Problem\n\nx\n") is False
    assert open(path, encoding="utf-8").read() == before
    assert os.stat(path).st_mtime == 1
    changed = dict(meta, status="backlog", updated="2030-01-01T00:00:00Z")
    assert write_item(path, changed, "## Problem\n\nx\n") is True
    assert load_item(path).meta["updated"] == "2030-01-01T00:00:00Z"
    assert load_item(path).status == "backlog"


def test_set_status_same_status_reports_unchanged(tmp_path, capsys):
    map_dir = tmp_path / "map"
    map_dir.mkdir()
    _write(map_dir, 1, "inbox", "2026-10-01", [], "2026-10-01")
    from mapkit.__main__ import main
    before = (map_dir / "bug-row1.md").read_text()
    assert main(["item", "set-status", "row1", "inbox", "--repo", str(tmp_path)]) == 0
    assert "unchanged" in capsys.readouterr().out
    assert (map_dir / "bug-row1.md").read_text() == before
