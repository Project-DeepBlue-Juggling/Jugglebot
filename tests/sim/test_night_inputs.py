"""``python -m mapkit night inputs``: the generated input section (close-out C5)."""
from __future__ import annotations

import os

from mapkit import night_inputs
from mapkit.__main__ import main
from mapkit.items import write_item
from mapkit.night_shift_report import INPUTS_HEADER, INPUTS_SECTION, cells, report_errors

_PRE = {"bug": "BUG", "feature": "FEAT", "idea": "IDEA", "ticket": "TKT", "pathway": "PWAY"}


def _put(tmp, kind, slug, n, status=None, **extra):
    meta = {"id": "%s-%d" % (_PRE[kind], n), "kind": kind, "title": "T " + slug, "status": status or "inbox",
            "created": "2026-10-01", "updated": "2026-10-01T00:00:00Z"}
    if kind == "ticket":
        meta.update(parent="p", type="task", status=status or "open")
    meta.update(extra)
    (tmp / "map").mkdir(exist_ok=True)
    write_item(str(tmp / "map" / ("%s-%s.md" % (kind, slug))), meta, "Body\n")


def _skip(reason, input_="Need X", note="Then it does Y"):
    d = {"night_skip": reason, "night_skip_on": "2026-11-14", "night_skip_note": note}
    if reason.startswith("input-"):
        d["night_skip_input"] = input_
    return d


def _body(tmp):
    return night_inputs.render(str(tmp / "map"))


def test_no_rows_is_none(tmp_path):
    assert _body(tmp_path) == "none"                       # no map at all
    _put(tmp_path, "bug", "a", 1)
    _put(tmp_path, "bug", "b", 2, **_skip("sitting", note="n"))
    assert _body(tmp_path) == "none"                       # non-input reasons are excluded


def test_each_input_type_and_respond_in(tmp_path):
    _put(tmp_path, "bug", "fact", 5, **_skip("input-fact"))
    _put(tmp_path, "feature", "sit", 2, **_skip("input-sitting"))
    _put(tmp_path, "idea", "dec", 3, **_skip("input-decision"))
    _put(tmp_path, "idea", "dec2", 1, **_skip("input-decision"))
    # open question tickets on bug-fact: the lowest ID wins; a complete one is ignored
    _put(tmp_path, "ticket", "q9", 9, type="question", parent="bug-fact")
    _put(tmp_path, "ticket", "q7", 7, type="question", parent="bug-fact")
    _put(tmp_path, "ticket", "q3", 3, type="question", parent="bug-fact", status="complete")
    # a non-question ticket does not count
    _put(tmp_path, "ticket", "t4", 4, parent="idea-dec", type="task")
    out = _body(tmp_path).splitlines()
    assert cells(out[0]) == list(INPUTS_HEADER)
    rows = [cells(r) for r in out[2:]]
    assert [r[0] for r in rows] == ["BUG-5", "FEAT-2", "IDEA-1", "IDEA-3"]   # kind, then number
    by = {r[0]: r for r in rows}
    assert by["BUG-5"][2:] == ["fact", "Need X", "Then it does Y", "TKT-7"]
    assert by["FEAT-2"][2] == "sitting" and by["FEAT-2"][5] == "sheet"
    assert by["IDEA-3"][2] == "decision" and by["IDEA-3"][5] == "none"
    assert by["IDEA-1"][5] == "none"


def test_closed_items_pathways_and_tickets_are_excluded(tmp_path):
    _put(tmp_path, "bug", "done", 1, status="complete", **_skip("input-fact"))
    _put(tmp_path, "bug", "gone", 2, status="dropped", **_skip("input-fact"))
    _put(tmp_path, "bug", "live", 3, status="in_progress", **_skip("input-fact"))
    out = _body(tmp_path).splitlines()
    assert [cells(r)[0] for r in out[2:]] == ["BUG-3"]


def test_a_pipe_or_newline_cannot_break_the_table_and_the_report_passes(tmp_path):
    _put(tmp_path, "bug", "p", 1, title="a | b", **_skip("input-fact", input_="x | y", note="do | this"))
    body = _body(tmp_path)
    row = body.splitlines()[2]
    assert len(cells(row)) == len(INPUTS_HEADER)
    assert "a / b" in row and "x / y" in row and "do / this" in row
    report = ("# Night Shift 2026-11-15\n\n## Nightly\nGREEN\n\n## Merged to main\nnone\n\n## Held\nnone\n\n"
              "## Questions for the Operator\nnone\n\n## %s\n%s\n\n## Unfinished\nnone\n\n## Skipped\nnone\n\n"
              "## Unaddressed Sitting results\nnone\n\n## Cost\nunits: 0\n" % (INPUTS_SECTION, body))
    assert report_errors(report) == []


def test_cli_prints_the_body(tmp_path, capsys):
    _put(tmp_path, "bug", "a", 1, **_skip("input-sitting"))
    assert main(["night", "inputs", "--map", str(tmp_path / "map")]) == 0
    out = capsys.readouterr().out
    assert out.startswith("| Item | What it is | Input |") and "| BUG-1 |" in out
    assert main(["night", "inputs", "--map", str(tmp_path / "empty")]) == 0
    assert capsys.readouterr().out.strip() == "none"
