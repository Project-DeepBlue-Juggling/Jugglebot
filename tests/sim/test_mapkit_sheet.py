"""mapkit/sheet.py: check_sheet, validate_shape, enrich, results and naming (IDEA-19)."""
from __future__ import annotations

import copy
import glob
import os

import pytest
import yaml

from mapkit import sheet as sh
from mapkit.items import write_item

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def _body(setup, do, pass_):
    return ("Intro.\n\n## Sitting\n\n- setup: %s\n- why: because.\n- do:\n%s- pass: %s\n"
            "- record: the lag\n- minutes: 10\n") % (
        setup, "".join("  - %s\n" % d for d in do), pass_)


def _item(root, slug, id_, priority, body, status="inbox"):
    meta = {"id": id_, "kind": "bug", "title": slug, "status": status, "created": "2026-10-20",
            "priority": priority, "hardware": "robot", "parent": "pw"}
    write_item(str(root / "map" / ("bug-%s.md" % slug)), meta, body)


@pytest.fixture()
def repo(tmp_path):
    (tmp_path / "map").mkdir()
    _item(tmp_path, "a", "BUG-1", "low", _body("level", [
        "launch `ros2 launch jugglebot jugglebot_launch.py`", "level it",
        "run `python tools/x.py --date <date>`"], "it moves"))
    _item(tmp_path, "b", "BUG-2", "medium", _body("level", ["level it", "flick it"], "it flicks"))
    return tmp_path


def _step(n, kind, frm, to, text, serves, **kw):
    d = {"n": n, "kind": kind, "title": "t%d" % n, "from": frm, "to": to,
         "actions": [{"where": "laptop", "text": text}], "see": "ok", "stop_if": "smoke",
         "serves": serves}
    d.update(kw)
    return d


@pytest.fixture()
def good():
    return {"schema": 1, "date": "2026-11-10", "rig": "robot", "omitted": [], "steps": [
        _step(1, "setup", [], ["launched"], "run ros2 launch jugglebot jugglebot_launch.py",
              [{"item": "BUG-1", "do": [0]}], new_launch=True),
        _step(2, "setup", ["launched"], ["launched", "level"], "level it",
              [{"item": "BUG-1", "do": [1]}, {"item": "BUG-2", "do": [0]}]),
        _step(3, "run", ["launched", "level"], ["launched", "level"],
              "python tools/x.py --date 2026-11-10 then flick it",
              [{"item": "BUG-1", "do": [2], "pass": True}, {"item": "BUG-2", "do": [1], "pass": True}]),
        _step(4, "restore", ["launched", "level"], [], "shut down", []),
    ]}


def _has(errs, text):
    assert any(text in e for e in errs), errs


def test_good_sheet_clean(good, repo):
    assert sh.check_sheet(good, str(repo)) == []


# --- check_sheet ---

def test_do_step_unplaced(good, repo):
    good["steps"][1]["serves"][0]["do"] = []
    _has(sh.check_sheet(good, str(repo)), "BUG-1: do step 1 is on no step and not omitted")


def test_omitted_do_index_with_reason(good, repo):
    good["steps"][1]["serves"][0]["do"] = []
    good["omitted"] = [{"item": "BUG-1", "do": [1], "reason": "done earlier"}]
    assert sh.check_sheet(good, str(repo)) == []


def test_omitted_whole_item(good, repo):
    good["steps"] = [s for s in good["steps"]]
    for st in good["steps"]:
        st["serves"] = [sv for sv in st["serves"] if sv["item"] != "BUG-2"]
    good["steps"][2]["to"] = ["launched", "level"]
    good["omitted"] = [{"item": "BUG-2", "reason": "not today"}]
    assert sh.check_sheet(good, str(repo)) == []


def test_omitted_missing_reason(good, repo):
    good["omitted"] = [{"item": "BUG-2", "do": [1]}]
    _has(sh.check_sheet(good, str(repo)), "each entry needs 'item' and a 'reason'")


def test_command_absent(good, repo):
    good["steps"][0]["actions"][0]["text"] = "launch the thing ros2 launch jugglebot jugglebot_launch.py"
    assert sh.check_sheet(good, str(repo)) == []
    good["steps"][0]["actions"][0]["text"] = "start the launch"
    good["steps"][0].pop("new_launch")
    _has(sh.check_sheet(good, str(repo)), "do step 0's command is not on the sheet verbatim")


def test_command_retyped(good, repo):
    good["steps"][0]["actions"][0]["text"] = "ros2 launch jugglebot jugglebot-launch.py"
    _has(sh.check_sheet(good, str(repo)), "command is not on the sheet verbatim")


def test_placeholder_matches_concrete(good, repo):
    assert sh.check_sheet(good, str(repo)) == []  # <date> matched 2026-11-10
    good["steps"][2]["actions"][0]["text"] = "python tools/y.py --date 2026-11-10"
    _has(sh.check_sheet(good, str(repo)), "BUG-1: do step 2's command")


def test_do_out_of_order(good, repo):
    good["steps"][1]["serves"][0]["do"] = [2]
    good["steps"][2]["serves"][0]["do"] = [1]
    _has(sh.check_sheet(good, str(repo)), "BUG-1: do steps out of order at step 3")


def test_no_pass_and_two_pass(good, repo):
    good["steps"][2]["serves"][1].pop("pass")
    _has(sh.check_sheet(good, str(repo)), "BUG-2: needs exactly one step with pass: true, has 0")
    good["steps"][2]["serves"][1]["pass"] = True
    good["steps"][1]["serves"][1]["pass"] = True
    good["steps"][1]["kind"] = "run"
    _has(sh.check_sheet(good, str(repo)), "BUG-2: needs exactly one step with pass: true, has 2")


def test_verdict_from_lacks_setup(good, repo):
    good["steps"][1]["to"] = ["launched"]
    good["steps"][2]["from"] = ["launched"]
    _has(sh.check_sheet(good, str(repo)), "does not start from its setup (missing level)")


def test_item_not_in_scope(good, repo):
    good["steps"][1]["serves"].append({"item": "BUG-99", "do": []})
    _has(sh.check_sheet(good, str(repo)), "BUG-99 is not an in-scope Item")
    good["omitted"] = [{"item": "BUG-98", "reason": "x"}]
    _has(sh.check_sheet(good, str(repo)), "omitted: BUG-98 is not an in-scope Item")


# --- validate_shape ---

def test_shape_good(good):
    assert sh.validate_shape(good) == []


def test_shape_n_gap(good):
    good["steps"][2]["n"] = 5
    _has(sh.validate_shape(good), "n must be 3")


def test_shape_chain_break(good):
    good["steps"][1]["from"] = []
    _has(sh.validate_shape(good), "step 2: starts from [] but step 1 left [launched]")


def test_shape_ends_armed(good):
    good["steps"][3]["to"] = ["armed"]
    _has(sh.validate_shape(good), "the sheet must not end armed")


def test_shape_run_serves_nothing(good):
    good["steps"][2]["serves"] = []
    _has(sh.validate_shape(good), "a run step must serve at least one Item")


def test_shape_pass_on_setup(good):
    good["steps"][0]["serves"][0]["pass"] = True
    _has(sh.validate_shape(good), "only a run step can carry a verdict")


def test_shape_new_launch_needs_command(good):
    good["steps"][0]["actions"][0]["text"] = "do something"
    _has(sh.validate_shape(good), "new_launch needs its 'ros2 launch' command")


def test_shape_unknown_state_word(good):
    good["steps"][0]["to"] = ["launched", "flying"]
    good["steps"][1]["from"] = ["launched", "flying"]
    _has(sh.validate_shape(good), "must list state words")


def test_shape_empty_stop_if(good):
    good["steps"][1]["stop_if"] = "  "
    _has(sh.validate_shape(good), "'stop_if' is empty")


# --- enrich ---

def test_enrich_optional_rules(good, repo):
    out = sh.enrich(good, str(repo))
    by_n = {s["n"]: s for s in out["steps"]}
    assert by_n[1]["optional"] is True            # BUG-1 low only
    assert by_n[2]["optional"] is False           # low + medium shared
    assert by_n[3]["optional"] is False
    assert by_n[4]["optional"] is False           # serves nothing
    assert good["steps"][0]["from"] == []         # input untouched
    assert "from_" in by_n[1]


def test_enrich_pass_text_from_item(good, repo):
    out = sh.enrich(good, str(repo))
    sv = {s["item"]: s for s in out["steps"][2]["serves"]}
    assert sv["BUG-1"]["pass_text"] == "it moves" and sv["BUG-2"]["pass_text"] == "it flicks"
    assert sv["BUG-1"]["verdict"] is True
    assert out["steps"][0]["serves"][0]["pass_text"] is None


def test_enrich_unknown_item(good, repo):
    good["steps"][0]["serves"].append({"item": "BUG-99", "do": []})
    out = sh.enrich(good, str(repo))
    row = out["steps"][0]["serves"][1]
    assert row["title"] == "" and row["is_open"] is False and row["stem"] is None


# --- apply_step_result ---

def _apply(good, n, status=None, comments="", verdicts=None, results=None):
    return sh.apply_step_result(good, results or sh.empty_results(), n, status, comments,
                                verdicts or {}, "2026-11-10T10:00:00")


def test_apply_status_plain_step(good):
    out = _apply(good, 1, status="done", comments="  all  fine ")
    assert out["steps"][1] == {"status": "done", "comments": "all fine", "saved_at": "2026-11-10T10:00:00"}


def test_apply_verdicts(good):
    out = _apply(good, 3, verdicts={"BUG-1": {"verdict": "pass", "notes": "ok"},
                                    "BUG-2": {"verdict": "fail", "notes": "x"}})
    assert out["verdicts"][3]["BUG-1"]["verdict"] == "pass"
    assert out["verdicts"][3]["BUG-2"]["verdict"] == "fail"


def test_apply_errors(good):
    with pytest.raises(sh.SheetError, match="takes a verdict per Item, not a status"):
        _apply(good, 3, status="done", verdicts={"BUG-1": {"verdict": "pass"}})
    with pytest.raises(sh.SheetError, match="carries no verdicts"):
        _apply(good, 1, status="done", verdicts={"BUG-1": {"verdict": "pass"}})
    with pytest.raises(sh.SheetError, match="verdict must be one of"):
        _apply(good, 3, verdicts={"BUG-1": {"verdict": "great"}})
    with pytest.raises(sh.SheetError, match="no step 9"):
        _apply(good, 9, status="done")
    with pytest.raises(sh.SheetError, match="status must be one of"):
        _apply(good, 1, status="bogus")


def test_apply_does_not_mutate(good):
    res = {"schema": 1, "steps": {1: {"status": "done", "comments": "", "saved_at": "t"}},
           "verdicts": {3: {"BUG-1": {"verdict": "pass", "notes": "", "saved_at": "t"}}}}
    before = copy.deepcopy(res)
    _apply(good, 3, verdicts={"BUG-1": {"verdict": "fail"}}, results=res)
    _apply(good, 2, status="problem", results=res)
    assert res == before



def test_a_juggle_goal_needs_launched_level_and_armed(good, repo):
    goal = 'ros2 action send_goal /jugglebot/juggle jugglebot_interfaces/action/Juggle "{pattern: x}"'
    good["steps"][2]["actions"].append({"where": "Terminal 2", "command": goal})
    _has(sh.check_sheet(good, str(repo)), "step 3: sends a Juggle goal but does not start from armed")
    good["steps"][1]["actions"].append({"where": "Terminal 2", "command": goal})
    _has(sh.check_sheet(good, str(repo)), "step 2: sends a Juggle goal but does not start from level, armed")
    nominate = "ros2 service call /skills/nominate jugglebot_interfaces/srv/SetString \"{data: 'x'}\""
    good["steps"][1]["actions"][-1]["command"] = nominate
    assert not [e for e in sh.check_sheet(good, str(repo)) if e.startswith("step 2: sends")]


def test_skipped_is_a_step_status(good):
    out = _apply(good, 1, status="skipped")
    assert out["steps"][1]["status"] == "skipped"
    with pytest.raises(sh.SheetError):
        _apply(good, 1, status="later")


# --- results, listing, names ---

def test_results_round_trip(good, tmp_path):
    out = _apply(good, 1, status="done")
    sh.write_results(str(tmp_path), "2026-11-10-robot", out)
    back = sh.load_results(str(tmp_path), "2026-11-10-robot")
    assert list(back["steps"]) == [1] and back["steps"][1]["status"] == "done"
    assert sh.saved_steps(back) == [1]
    assert sh.load_results(str(tmp_path), "2026-11-11-robot") == sh.empty_results()


def test_list_sheets_order(good, tmp_path):
    d = tmp_path / "map" / "sheets"
    d.mkdir(parents=True)
    for name, date in (("2026-11-10-robot", "2026-11-10"), ("2026-11-10-robot-2", "2026-11-10"),
                       ("2026-11-09-robot", "2026-11-09")):
        s = dict(good, date=date)
        (d / (name + ".yaml")).write_text(yaml.safe_dump(s))
    sh.write_results(str(tmp_path), "2026-11-10-robot", _apply(good, 1, status="done"))
    rows = sh.list_sheets(str(tmp_path))
    assert [r["name"] for r in rows] == ["2026-11-10-robot-2", "2026-11-10-robot", "2026-11-09-robot"]
    assert rows[1]["saved"] == 1 and rows[0]["steps"] == 4


def test_next_name(tmp_path):
    assert sh.next_name(str(tmp_path), "2026-11-10", "robot") == "2026-11-10-robot"
    d = tmp_path / "map" / "sheets"
    d.mkdir(parents=True)
    (d / "2026-11-10-robot.yaml").write_text("a: 1")
    assert sh.next_name(str(tmp_path), "2026-11-10", "robot") == "2026-11-10-robot-2"
    (d / "2026-11-10-robot-2.yaml").write_text("a: 1")
    assert sh.next_name(str(tmp_path), "2026-11-10", "robot") == "2026-11-10-robot-3"


@pytest.mark.parametrize("bad", ["x", "2026-11-10-arm", "2026-1-10-robot", "../2026-11-10-robot"])
def test_invalid_names(tmp_path, bad):
    assert not sh.valid_name(bad)
    for fn in (sh.sheet_path, sh.results_path):
        with pytest.raises(sh.SheetError):
            fn(str(tmp_path), bad)
    with pytest.raises(sh.SheetError):
        sh.next_name(str(tmp_path), "bad", "robot")


# --- guard on the real repo ---

def test_committed_sheets_valid():
    paths = [p for p in glob.glob(os.path.join(REPO, "map", "sheets", "*.yaml"))
             if not p.endswith(".results.yaml")]
    for p in paths:
        assert sh.validate_shape(sh.load_sheet(p)) == [], p


def test_first_step_starts_from_nothing(good, repo):
    good["steps"][0]["from"] = ["armed"]
    _has(sh.validate_shape(good), "step 1: must start from nothing")


def test_whole_omitted_item_cannot_be_served(good, repo):
    good["omitted"] = [{"item": "BUG-1", "reason": "not today"}]
    _has(sh.check_sheet(good, str(repo)), "BUG-1 is omitted whole but served here")


def test_non_list_state_is_an_error_not_a_crash(good):
    good["steps"][0]["from"] = 5
    _has(sh.validate_shape(good), "'from' must list state words")


def test_corrupt_results_sections_read_as_empty(repo):
    path = sh.results_path(str(repo), "2026-11-10-robot")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("schema: 1\nsteps: null\nverdicts: null\n")
    assert sh.saved_steps(sh.load_results(str(repo), "2026-11-10-robot")) == []


def test_live_check_accepts_an_item_closed_since_the_sheet_was_written(good, repo):
    _item(repo, "b", "BUG-2", "medium", _body("level", ["level it", "flick it"], "it flicks"),
          status="complete")
    _has(sh.check_sheet(good, str(repo)), "BUG-2 is not an in-scope Item")
    assert sh.check_sheet(good, str(repo), live=True) == []
    good["steps"][2]["serves"][1]["item"] = "BUG-999"  # an Item that never existed still fails
    _has(sh.check_sheet(good, str(repo), live=True), "BUG-999 is not an in-scope Item")


def test_every_state_word_has_help_and_enrich_sends_it(good, repo):
    assert set(sh.STATE_HELP) == set(sh.STATE_WORDS)
    assert all(text.strip().endswith(".") for text in sh.STATE_HELP.values())
    assert sh.enrich(good, str(repo))["state_help"] == sh.STATE_HELP


def test_enrich_gives_a_serve_without_do_an_empty_list(good, repo):
    good["steps"][2]["serves"][0].pop("do", None)
    for st in sh.enrich(good, str(repo))["steps"]:
        for sv in st["serves"]:
            assert isinstance(sv["do"], list)


def test_firmware_named_on_the_sheet_must_match_the_tree(good, repo):
    os.makedirs(os.path.join(str(repo), "teensy_link"), exist_ok=True)
    with open(os.path.join(str(repo), "teensy_link", "rpc_args.py"), "w", encoding="utf-8") as fh:
        fh.write("EXPECTED_BRIDGE_FW_VERSION = 43\n")
    good["steps"][0]["see"] = "BRIDGE_FW_CHECK: OK - can-bridge v42"
    _has(sh.check_sheet(good, str(repo)), "names can-bridge v42, but the tree expects v43")
    good["steps"][0]["see"] = "BRIDGE_FW_CHECK: OK - can-bridge v43"
    assert not [e for e in sh.check_sheet(good, str(repo)) if "can-bridge" in e]


def test_actions_must_suit_zsh_and_any_working_directory(good, repo):
    """2026-11-13: step 24 sourced install/setup.bash in zsh (sibling scripts resolved against the
    working directory: "no such file ... local_setup.sh") and step 40 tee'd to ../temp/logs from the
    repo root (launches 3 and 4 lost their logs)."""
    acts = good["steps"][1]["actions"]
    acts.append({"where": "Terminal 2", "command": "cd ~/Desktop/Jugglebot/ros_ws && source install/setup.bash"})
    _has(sh.check_sheet(good, str(repo)), "step 2: 'source install/setup.bash' is bash-only")
    acts[-1]["command"] = "cd ~/Desktop/Jugglebot/ros_ws && source /opt/ros/foxy/setup.zsh && source install/setup.zsh"
    assert not [e for e in sh.check_sheet(good, str(repo)) if "bash-only" in e]
    acts[-1]["command"] = "ros2 launch jugglebot jugglebot_launch.py 2>&1 | tee -i ../temp/logs/l.log"
    _has(sh.check_sheet(good, str(repo)), "tee to the relative path ../temp/logs/l.log")
    acts[-1]["command"] = "ros2 launch jugglebot jugglebot_launch.py 2>&1 | tee -i ~/Desktop/Jugglebot/temp/logs/l.log"
    assert sh.check_sheet(good, str(repo)) == []
    # The app's live re-check of an already-flown sheet is not bannered by authoring rules.
    acts[-1]["command"] = "source install/setup.bash && x | tee ../l.log"
    assert not [e for e in sh.check_sheet(good, str(repo), live=True) if "bash-only" in e or "tee to" in e]


def test_setup_waived_needs_a_reason(good, repo):
    run = next(st for st in good["steps"] if any(sv.get("pass") for sv in st["serves"]))
    run["from"] = []
    for st in good["steps"]:  # keep the chain whole
        if st["n"] == run["n"] - 1:
            st["to"] = []
        if st["n"] == run["n"] + 1:
            st["from"] = list(run["to"])
    errs = sh.check_sheet(good, str(repo))
    _has(errs, "does not start from its setup")
    for sv in run["serves"]:
        if sv.get("pass"):
            sv["setup_waived"] = "the Item judges a state restored by a relaunch"
    assert not [e for e in sh.check_sheet(good, str(repo)) if "does not start from its setup" in e]


# --- the two exceptions a real Sitting needed (2026-11-10) ---

def test_reorder_reason_allows_out_of_order(good, repo):
    steps = good["steps"]
    served = [(st, sv) for st in steps for sv in st["serves"] if sv["item"] == "BUG-1" and sv.get("do")]
    assert len(served) >= 2
    (_, first), (_, second) = served[0], served[1]
    first["do"], second["do"] = second["do"], first["do"]
    _has(sh.check_sheet(good, str(repo)), "BUG-1: do steps out of order")
    second["reorder"] = "the do list states this precondition late"
    assert not [e for e in sh.check_sheet(good, str(repo)) if "out of order" in e]


def test_bare_command_name_is_not_a_command():
    assert sh._commands("read it from the GUI or `ros2 param get`") == []
    assert sh._commands("`ros2 param get /skill_node x`") == ["ros2 param get /skill_node x"]
