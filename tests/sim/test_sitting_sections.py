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


def test_first_pins_a_row_ahead_of_the_score_order(repo):
    """``first: <reason>`` (2026-11-06): the row is flown before every scored row and its
    setup set opens the rig's first block; without the key the order is score only."""
    _item(repo, "bug", "row7", _sec("bb-fed", "60", "- first: the others fly on it\n"), id="JB-7",
          title="Row 7", priority="low", hardware="robot", parent="pw")  # score 1/60, last unpinned
    data = sitting.collect(str(repo))
    assert [r.item_id for r in data["rows"]] == ["JB-7", "JB-1", "JB-2", "JB-4", "JB-5", "JB-3", "JB-6"]
    assert [[r.item_id for r in b] for b in data["blocks"]] == [
        ["JB-7", "JB-4"], ["JB-1", "JB-6"], ["JB-2", "JB-3"], ["JB-5"]]
    assert data["rows"][0].spec.first == "the others fly on it"
    text = sitting.render_runbook(str(repo))
    assert "### Block 1: bb-fed  (2 rows, 65 min)" in text
    assert "- first: the others fly on it" in text


def test_first_without_a_reason_is_a_parse_error():
    spec, errs = sitting.parse_sitting(_sec(extra="- first:\n"))
    assert spec is None and any("first needs a reason" in e for e in errs)
    spec, errs = sitting.parse_sitting(_sec())
    assert errs == [] and spec.first is None


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
    assert "### Block 1: level  (2 rows, 50 min)" in text
    assert "#### JB-1 Row 1  (10 min, high, pw)" in text
    assert "- branch: `night-shift/b`" in text
    assert "`JB-" not in text


def _rig_repo(tmp_path):
    (tmp_path / "map").mkdir()
    for n, rig, mins, pri in (("1", "bench-hand", "5", "high"), ("2", "bench-leg", "10", "medium"),
                              ("3", "robot", "40", "low"), ("4", "bench-leg", "10", "low")):
        _item(tmp_path, "bug", "rig%s" % n, _sec("level", mins), id="JB-%s" % n, title="Rig %s" % n,
              priority=pri, hardware=rig, parent="pw")
    _item(tmp_path, "bug", "nosec", id="JB-5", title="No sec", hardware="bench-leg", priority="high")
    return tmp_path


def test_rigs_never_share_a_block_and_follow_rig_order(tmp_path):
    repo = _rig_repo(tmp_path)
    data = sitting.collect(str(repo))
    # same setup everywhere, but one block per rig; robot first despite the lowest score
    assert [(h, [[r.item_id for r in b] for b in bs]) for h, bs in data["rigs"]] == [
        ("robot", [["JB-3"]]), ("bench-leg", [["JB-2", "JB-4"]]), ("bench-hand", [["JB-1"]])]
    assert [[r.item_id for r in b] for b in data["blocks"]] == [["JB-3"], ["JB-2", "JB-4"], ["JB-1"]]
    assert {r.item_id: r.hardware for r in data["rows"]}["JB-1"] == "bench-hand"
    assert data["needs_section"][0][0] == "JB-5" and data["needs_section"][0][3] == "bench-leg"


def test_rendered_rig_headings_summary_and_needs_label(tmp_path):
    text = sitting.render_runbook(str(_rig_repo(tmp_path)))
    lines = text.splitlines()
    heads = [l for l in lines if l.startswith("#")]
    assert heads.index("## Robot  (1 block, 40 min)") < heads.index("## Bench leg  (1 block, 20 min)") \
        < heads.index("## Bench hand  (1 block, 5 min)")
    assert "### Block 1: level  (1 row, 40 min)" in lines
    assert "### Block 3: level  (1 row, 5 min)" in lines
    assert "#### JB-1 Rig 1  (5 min, high, pw)" in lines
    assert "Rigs: Robot 1 block (40 min) \u00b7 Bench leg 1 block (20 min) \u00b7 Bench hand 1 block (5 min)" in lines
    assert "- JB-5 [Bench leg] No sec: no ## Sitting section" in lines
    assert "## Needs a Sitting section" in lines and "## Close-out" in lines


def test_bench_hand_is_a_valid_hardware_value():
    from mapkit.items import METRIC_ENUMS
    assert "bench-hand" in METRIC_ENUMS["hardware"] and "bench-hand" in sitting.HARDWARE


def test_append_result_placement_and_error():
    out = sitting.append_sitting_result("Body.\n\n## Progress\n\n- 2026-10-01: x\n", "2026-11-02", "pass", "ok")
    assert out.index("## Sitting result (2026-11-02)") < out.index("## Progress")
    assert "- verdict: pass\n- notes: ok\n" in out
    out = sitting.append_sitting_result("Body.\n", "2026-11-02", "fail", "bad")
    assert out.endswith("## Sitting result (2026-11-02)\n\n- verdict: fail\n- notes: bad\n")
    out = sitting.append_sitting_result("Body.\n", "2026-11-02", "pending", "check the bag")
    assert out.endswith("- verdict: pending\n- notes: check the bag\n")
    with pytest.raises(ValueError):
        sitting.append_sitting_result("Body.\n", "2026-11-02", "maybe", "")


def test_append_result_timed_and_validation():
    out = sitting.append_sitting_result("Body.\n", "2026-11-02", "pass", "ok", time="14:05")
    assert out.endswith("## Sitting result (2026-11-02 14:05)\n\n- verdict: pass\n- notes: ok\n")
    untimed = sitting.append_sitting_result("Body.\n", "2026-11-02", "pass", "ok")
    assert untimed == "Body.\n\n## Sitting result (2026-11-02)\n\n- verdict: pass\n- notes: ok\n"
    for bad in ("24:00", "12:60", "9:05", "1405", ""):
        with pytest.raises(ValueError):
            sitting.append_sitting_result("Body.\n", "2026-11-02", "pass", "ok", time=bad)
    with pytest.raises(ValueError):
        sitting.append_sitting_result("Body.\n", "11/02", "pass", "ok")


def test_parse_sitting_results_both_forms():
    body = sitting.append_sitting_result("Body.\n", "2026-11-02", "fail", "bad")
    body = sitting.append_sitting_result(body, "2026-11-03", "pending", "check the bag", time="09:30")
    body += "\n## Progress\n\n- verdict: pass\n"
    assert sitting.parse_sitting_results(body) == [
        {"date": "2026-11-02", "time": None, "verdict": "fail", "notes": "bad"},
        {"date": "2026-11-03", "time": "09:30", "verdict": "pending", "notes": "check the bag"},
    ]
    assert sitting.parse_sitting_results("Body.\n") == []


def test_append_result_rejects_trailing_newline_and_impossible_dates():
    # ``$`` would accept "2026-11-02\n" and write a heading no reader matches; fullmatch does not.
    for bad_date in ("2026-11-02\n", "2026-13-01", "2026-11-32"):
        with pytest.raises(ValueError):
            sitting.append_sitting_result("Body.\n", bad_date, "pass", "ok")
    with pytest.raises(ValueError):
        sitting.append_sitting_result("Body.\n", "2026-11-02", "pass", "ok", time="14:05\n")


def test_parse_sitting_results_reads_hand_split_notes():
    # The debrief re-flows an over-long owner box into ``- notes (...):`` plus indented bullets.
    body = ("## Sitting result (2026-11-03)\n\n- verdict: fail\n"
            "- notes (the owner's box, split into bullets):\n"
            "  - At 1.35 the hand isn't using the full stroke.\n"
            "  - At 1.45 the first attempt was uncatchable.\n\n## Progress\n\n- 2026-11-03: x\n")
    assert sitting.parse_sitting_results(body) == [{
        "date": "2026-11-03", "time": None, "verdict": "fail",
        "notes": "At 1.35 the hand isn't using the full stroke. At 1.45 the first attempt was uncatchable."}]


def test_runbook_page_current():
    with open(os.path.join(REPO, "docs", "runbook.md"), encoding="utf-8") as fh:
        assert fh.read() == sitting.render_runbook(REPO)
    assert gen_docs.write_docs(REPO, check=True) == []
