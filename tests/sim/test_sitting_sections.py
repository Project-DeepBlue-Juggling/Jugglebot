"""T-U5: ``## Sitting`` parsing, Runbook ordering/grouping and the currency of docs/runbook.md."""
from __future__ import annotations

import os
import re

import pytest

from mapkit import gen_docs, sitting
from mapkit.items import write_item

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def _sec(setup="level", minutes="10", extra="", do="  - move it\n  - stop it\n", drop=()):
    lines = {"setup": "- setup: %s\n" % setup, "why": "- why: because.\n",
             "do": "- do:\n" + do, "pass": "- pass: it moves\n",
             "record": "- record: the lag\n", "minutes": "- minutes: %s\n" % minutes}
    return "Intro.\n\n## Sitting\n\n" + "".join(v for k, v in lines.items() if k not in drop) + extra


def test_parse_all_keys_and_nested_do():
    spec, errs = sitting.parse_sitting(_sec("level, armed", "12", "- branch: `night-shift/x`\n"))
    assert errs == []
    assert spec.setup == ("level", "armed") and spec.minutes == 12
    assert spec.do == ["move it", "stop it"] and spec.branch == "night-shift/x"
    assert spec.why == "because." and spec.pass_ == "it moves" and spec.record == "the lag"


def test_single_line_do_and_stops_at_next_heading():
    body = _sec(do="").replace("- do:\n", "- do: press go\n") + "\n## Progress\n\n- 2026-10-01: x\n"
    spec, errs = sitting.parse_sitting(body)
    assert errs == [] and spec.do == ["press go"] and spec.branch is None


def test_missing_do_names_do():
    spec, errs = sitting.parse_sitting(_sec(drop=("do",)))
    assert spec is None and any("'do'" in e for e in errs)


@pytest.mark.parametrize("bad", ["ten", "0", "-3"])
def test_bad_minutes(bad):
    spec, errs = sitting.parse_sitting(_sec(minutes=bad))
    assert spec is None and any("minutes" in e for e in errs)


def test_unknown_setup_and_key():
    spec, errs = sitting.parse_sitting(_sec(setup="level, magic"))
    assert spec is None and any("magic" in e for e in errs)
    spec, errs = sitting.parse_sitting(_sec(extra="- colour: red\n"))
    assert spec is None and any("colour" in e for e in errs)


def test_no_section():
    assert not sitting.has_sitting_section("## Other\n")
    assert sitting.parse_sitting("## Other\n")[0] is None


def _item(root, kind, slug, body="Body.\n", **kw):
    meta = {"kind": kind, "title": kw.pop("title", slug), "status": kw.pop("status", "inbox"),
            "created": "2026-10-20"}
    meta.update(kw)
    write_item(str(root / "map" / ("%s-%s.md" % (kind, slug))), meta, body)


@pytest.fixture()
def repo(tmp_path):
    (tmp_path / "map").mkdir()
    spec = [  # id, setup, minutes, priority, extra
        ("1", "level", "10", "high", ""),                      # 0.3
        ("2", "level, armed", "10", "medium", ""),             # 0.2
        ("3", "armed, level", "20", "high", ""),               # 0.15
        ("4", "bb-fed", "5", "low", ""),                       # 0.2
        ("5", "bridge-cycled", "30", "high", "- branch: night-shift/b\n"),  # 0.2
        ("6", "level", "40", "low", ""),                       # 0.025
    ]
    for n, setup, mins, pri, extra in spec:
        _item(tmp_path, "bug", "row%s" % n, _sec(setup, mins, extra), id="JB-%s" % n,
              title="Row %s" % n, priority=pri, hardware="robot", parent="pw")
    return tmp_path


def test_order_and_blocks(repo):
    data = sitting.collect(str(repo))
    assert [r.item_id for r in data["rows"]] == ["JB-1", "JB-2", "JB-4", "JB-5", "JB-3", "JB-6"]
    assert [[r.item_id for r in b] for b in data["blocks"]] == [
        ["JB-1", "JB-6"], ["JB-2", "JB-3"], ["JB-4"], ["JB-5"]]
    assert data["needs_section"] == [] and data["questions"] == []


def test_needs_section_and_hardware_none_ignored(repo):
    _item(repo, "bug", "nosec", id="JB-7", title="No sec", hardware="robot", priority="high")
    _item(repo, "bug", "soft", _sec(), id="JB-8", title="Soft", hardware="none")
    _item(repo, "bug", "done", _sec(), id="JB-9", title="Done", hardware="robot", status="complete")
    data = sitting.collect(str(repo))
    assert [n[0] for n in data["needs_section"]] == ["JB-7"]
    assert "JB-8" not in [r.item_id for r in data["rows"]]
    assert "JB-9" not in [r.item_id for r in data["rows"]]
    assert "JB-8" not in sitting.render_runbook(str(repo))


def test_questions_under_hardware_only(repo):
    _item(repo, "bug", "soft", id="JB-8", title="Soft", hardware="none")
    _item(repo, "ticket", "q1", id="JB-20", title="Which gain?", status="open", type="question", parent="JB-1")
    _item(repo, "ticket", "q2", id="JB-21", title="Soft q", status="open", type="question", parent="JB-8")
    assert sitting.collect(str(repo))["questions"] == [("JB-20", "Which gain?", "JB-1")]
    assert "- JB-20 Which gain? (parent JB-1)" in sitting.render_runbook(str(repo))


def test_render_has_no_date_and_is_stable(repo):
    text = sitting.render_runbook(str(repo))
    assert not re.search(r"20\d\d-\d\d-\d\d", text)
    assert text == sitting.render_runbook(str(repo))
    assert "## Block 1: level  (2 rows, 50 min)" in text
    assert "### JB-1 Row 1  (10 min, high, pw)" in text
    assert "- branch: `night-shift/b`" in text
    assert "`JB-" not in text


def test_append_result_placement_and_error():
    out = sitting.append_sitting_result("Body.\n\n## Progress\n\n- 2026-10-01: x\n", "2026-11-02", "pass", "ok")
    assert out.index("## Sitting result (2026-11-02)") < out.index("## Progress")
    assert "- verdict: pass\n- notes: ok\n" in out
    out = sitting.append_sitting_result("Body.\n", "2026-11-02", "fail", "bad")
    assert out.endswith("## Sitting result (2026-11-02)\n\n- verdict: fail\n- notes: bad\n")
    with pytest.raises(ValueError):
        sitting.append_sitting_result("Body.\n", "2026-11-02", "maybe", "")


def test_runbook_page_current():
    with open(os.path.join(REPO, "docs", "runbook.md"), encoding="utf-8") as fh:
        assert fh.read() == sitting.render_runbook(REPO)
    assert gen_docs.write_docs(REPO, check=True) == []
