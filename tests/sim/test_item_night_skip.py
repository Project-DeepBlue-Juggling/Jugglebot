"""Night Shift skip reasons live on the Item (close-out plan C3).

Contract: the four ``night_skip*`` keys are validated once, in ``mapkit.items``; the CLI
(`item skip`) writes them without bumping ``updated`` (C4's delta walk reads ``updated``
and ``status`` as "changed", and a skip written by the night must not look like a change);
closing an Item removes them. Everything runs in ``tmp_path``.
"""
from __future__ import annotations

import json
import subprocess

import pytest

from mapkit import itemcli
from mapkit.items import ItemError, load_item, write_item

UPD = "2026-10-01T00:00:00Z"


@pytest.fixture(autouse=True)
def _git_env(tmp_path, monkeypatch):
    monkeypatch.setenv("HOME", str(tmp_path))
    monkeypatch.setenv("GIT_CONFIG_GLOBAL", str(tmp_path / "gitconfig"))
    monkeypatch.setenv("GIT_CONFIG_NOSYSTEM", "1")


@pytest.fixture(autouse=True)
def _tree(tmp_path):
    (tmp_path / "map").mkdir()
    (tmp_path / "docs").mkdir()
    (tmp_path / "GLOSSARY.md").write_text("# G\n\n## Language\n\n### Robot\n\n**Hand**: The cup.\n")
    for b in ("active", "parked", "archived"):
        (tmp_path / "plans" / b).mkdir(parents=True)


def _put(root, kind, slug, status="inbox", body="Body\n", **extra):
    meta = {"kind": kind, "title": "T " + slug, "status": status, "created": "2026-10-01",
            "updated": UPD, "priority": "medium"}
    if kind == "pathway":
        meta = {"kind": kind, "title": "T", "status": "active", "created": "2026-10-01",
                "destination": "goal"}
    elif kind == "ticket":
        meta.update(parent="p", type="task", status="open")
    meta.update(extra)
    path = root / "map" / ("%s-%s.md" % (kind, slug))
    path.parent.mkdir(parents=True, exist_ok=True)
    write_item(str(path), meta, body)
    return path


SKIP = {"night_skip": "too-large", "night_skip_on": "2026-11-13", "night_skip_note": "Needs a plan first."}
INP = {"night_skip": "input-fact", "night_skip_on": "2026-11-13", "night_skip_note": "Blocked on a value.",
       "night_skip_input": "The measured stroke in mm."}


def _load(root, kind, slug, **extra):
    return load_item(str(_put(root, kind, slug, **extra)))


def test_valid_skip_loads_on_every_non_pathway_kind(tmp_path):
    for kind in ("bug", "feature", "idea", "ticket"):
        assert _load(tmp_path, kind, "ok-" + kind, **SKIP).meta["night_skip"] == "too-large"
    assert _load(tmp_path, "bug", "inp", **INP).meta["night_skip_input"].startswith("The measured")


def test_no_skip_keys_is_fine(tmp_path):
    assert "night_skip" not in _load(tmp_path, "bug", "plain").meta


@pytest.mark.parametrize("reason", ["sitting", "input-fact", "input-decision", "input-sitting",
                                    "owner-session", "too-large", "frozen-path", "out-of-reach",
                                    "waiting-trigger", "low-priority"])
def test_every_enum_reason_is_accepted(tmp_path, reason):
    extra = dict(SKIP, night_skip=reason)
    if reason.startswith("input-"):
        extra["night_skip_input"] = "x"
    assert _load(tmp_path, "bug", "r", **extra).meta["night_skip"] == reason


@pytest.mark.parametrize("patch,frag", [
    ({"night_skip": "lazy"}, "not in"),
    ({"night_skip_on": "13-11-2026"}, "YYYY-MM-DD"),
    ({"night_skip_on": "2026-02-30"}, "calendar"),
    ({"night_skip_on": None}, "YYYY-MM-DD"),
    ({"night_skip_note": "x" * 161}, "max 160"),
    ({"night_skip_note": None}, "required"),
    ({"night_skip_note": "  "}, "non-empty"),
    ({"night_skip_input": "unexpected"}, "only allowed for an input-"),
])
def test_invalid_skip_is_refused(tmp_path, patch, frag):
    extra = dict(SKIP)
    extra.update(patch)
    extra = {k: v for k, v in extra.items() if v is not None}
    with pytest.raises(ItemError, match=frag):
        _load(tmp_path, "bug", "bad", **extra)


def test_note_160_chars_is_the_boundary(tmp_path):
    assert _load(tmp_path, "bug", "edge", **dict(SKIP, night_skip_note="x" * 160))


def test_input_reason_requires_input_and_caps_it(tmp_path):
    no_input = {k: v for k, v in INP.items() if k != "night_skip_input"}
    with pytest.raises(ItemError, match="night_skip_input is required"):
        _load(tmp_path, "bug", "noinp", **no_input)
    with pytest.raises(ItemError, match="max 200"):
        _load(tmp_path, "bug", "biginp", **dict(INP, night_skip_input="y" * 201))
    assert _load(tmp_path, "bug", "okinp", **dict(INP, night_skip_input="y" * 200))


def test_stray_keys_without_reason_refused(tmp_path):
    with pytest.raises(ItemError, match="without night_skip"):
        _load(tmp_path, "bug", "stray", night_skip_on="2026-11-13")


def test_refused_on_pathway_and_question_ticket(tmp_path):
    with pytest.raises(ItemError, match="pathway"):
        _load(tmp_path, "pathway", "pw", **SKIP)
    with pytest.raises(ItemError, match="question ticket"):
        _load(tmp_path, "ticket", "q", type="question", **SKIP)
    assert _load(tmp_path, "ticket", "task-ok", type="task", **SKIP)


def test_unquoted_yaml_date_is_accepted_and_normalised(tmp_path):
    p = _put(tmp_path, "bug", "yd", **SKIP)
    p.write_text(p.read_text().replace("night_skip_on: '2026-11-13'", "night_skip_on: 2026-11-13"))
    assert load_item(str(p)).meta["night_skip_on"] == "2026-11-13"


# ---- CLI ---------------------------------------------------------------------------


def test_skip_sets_without_bumping_updated_then_clear_restores(tmp_path):
    p = _put(tmp_path, "bug", "b")
    before = p.read_text()
    it = itemcli.skip_item(str(tmp_path), "bug-b", reason="sitting", note="Needs the robot.",
                           on="2026-11-14")
    assert it.changed
    got = load_item(str(p))
    assert got.meta["updated"] == UPD  # THE pinned decision: a skip is not a change (C4)
    assert got.status == "inbox"
    assert (got.meta["night_skip"], got.meta["night_skip_on"]) == ("sitting", "2026-11-14")
    again = itemcli.skip_item(str(tmp_path), "bug-b", reason="sitting", note="Needs the robot.",
                              on="2026-11-14")
    assert not again.changed  # idempotent
    itemcli.skip_item(str(tmp_path), "bug-b", clear=True)
    assert p.read_text() == before


def test_skip_rejects_bad_input_and_leaves_file(tmp_path):
    p = _put(tmp_path, "bug", "b")
    before = p.read_text()
    with pytest.raises(itemcli.ItemCliError, match="input-"):
        itemcli.skip_item(str(tmp_path), "bug-b", reason="input-fact", note="n")
    with pytest.raises(itemcli.ItemCliError, match="exactly one"):
        itemcli.skip_item(str(tmp_path), "bug-b")
    with pytest.raises(itemcli.ItemCliError, match="not in"):
        itemcli.skip_item(str(tmp_path), "bug-b", reason="nope", note="n")
    _put(tmp_path, "ticket", "q", type="question")
    with pytest.raises(itemcli.ItemCliError, match="question ticket"):
        itemcli.skip_item(str(tmp_path), "ticket-q", reason="sitting", note="n")
    assert p.read_text() == before


@pytest.mark.parametrize("status", ["complete", "dropped"])
def test_set_status_close_removes_all_four_keys(tmp_path, status):
    kind = "bug"
    p = _put(tmp_path, kind, "c", **INP)
    itemcli.set_status(str(tmp_path), "bug-c", status)
    text = p.read_text()
    assert "night_skip" not in text
    assert load_item(str(p)).status == status


def test_set_status_other_moves_keep_the_skip(tmp_path):
    p = _put(tmp_path, "bug", "k", **SKIP)
    itemcli.set_status(str(tmp_path), "bug-k", "backlog")
    assert load_item(str(p)).meta["night_skip"] == "too-large"


def _git_repo(root):
    for args in (["init", "-q"], ["config", "user.email", "t@t"], ["config", "user.name", "t"]):
        subprocess.run(["git"] + args, cwd=root, check=True)


def test_batch_one_commit_and_no_updated_bump(tmp_path):
    a, b = _put(tmp_path, "bug", "a"), _put(tmp_path, "idea", "b")
    _put(tmp_path, "feature", "c")
    _git_repo(tmp_path)
    subprocess.run(["git", "add", "-A"], cwd=tmp_path, check=True)
    subprocess.run(["git", "commit", "-qm", "base"], cwd=tmp_path, check=True)
    f = tmp_path / "rows.json"
    f.write_text(json.dumps([
        {"item": "bug-a", "reason": "input-decision", "note": "Owner picks.", "input": "A or B?"},
        {"item": "idea-b", "reason": "low-priority", "note": "Not now."}]))
    changed = itemcli.skip_batch(str(tmp_path), json.loads(f.read_text()), on="2026-11-14",
                                 commit=True, push=False)
    assert len(changed) == 2
    log = subprocess.run(["git", "log", "--format=%s%n%b", "-n", "1"], cwd=tmp_path,
                         capture_output=True, text=True, check=True).stdout
    assert log.startswith("map: night skip reasons 2026-11-14 (2 Items)")
    assert "Logbook-Entry" not in log
    assert subprocess.run(["git", "rev-list", "--count", "HEAD"], cwd=tmp_path,
                          capture_output=True, text=True).stdout.strip() == "2"
    for p in (a, b):
        assert load_item(str(p)).meta["updated"] == UPD


def test_batch_is_all_or_nothing(tmp_path):
    a = _put(tmp_path, "bug", "a")
    before = a.read_text()
    rows = [{"item": "bug-a", "reason": "sitting", "note": "ok"},
            {"item": "bug-a-missing", "reason": "sitting", "note": "x"},
            {"item": "bug-a", "reason": "input-fact", "note": "no input"}]
    with pytest.raises(itemcli.ItemCliError, match="nothing written"):
        itemcli.skip_batch(str(tmp_path), rows, on="2026-11-14")
    assert a.read_text() == before


def test_cli_main_paths(tmp_path, capsys):
    from mapkit.__main__ import main
    _put(tmp_path, "bug", "m")
    rc = main(["item", "skip", "bug-m", "--reason", "sitting", "--note", "Robot.",
               "--repo", str(tmp_path)])
    assert rc == 0 and load_item(str(tmp_path / "map" / "bug-m.md")).meta["night_skip"] == "sitting"
    assert main(["item", "skip", "bug-m", "--clear", "--repo", str(tmp_path)]) == 0
    assert main(["item", "skip", "--repo", str(tmp_path)]) == 2
    f = tmp_path / "r.json"
    f.write_text(json.dumps([{"item": "bug-m", "reason": "sitting", "note": "n"}]))
    assert main(["item", "skip", "--batch", str(f), "--repo", str(tmp_path)]) == 0
