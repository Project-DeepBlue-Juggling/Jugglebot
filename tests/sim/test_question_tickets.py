"""T-U4: question tickets (type: question), has_answer, question_tickets, parent rules."""
from __future__ import annotations

import pytest

from mapkit import itemcli, tickets
from mapkit.items import (ItemError, has_answer, load_item, load_items, open_questions,
                          question_tickets, write_item)


@pytest.fixture(autouse=True)
def _git_env(tmp_path, monkeypatch):
    monkeypatch.setenv("HOME", str(tmp_path))
    monkeypatch.setenv("GIT_CONFIG_GLOBAL", str(tmp_path / "gitconfig"))
    monkeypatch.setenv("GIT_CONFIG_NOSYSTEM", "1")


def _write(root, kind, slug, n, status="inbox", body="Body\n", **extra):
    meta = {"id": "JB-%d" % n, "kind": kind, "title": "T " + slug, "status": status,
            "created": "2026-10-01", "updated": "2026-10-01T00:00:00Z", "priority": "medium"}
    if kind == "ticket":
        meta["type"] = "task"
    if kind == "pathway":
        meta["destination"] = "reach the goal"
    meta.update(extra)
    write_item(str(root / "map" / ("%s-%s.md" % (kind, slug))), meta, body)


def _tree(root):
    (root / "map").mkdir(parents=True)
    (root / "docs").mkdir()
    (root / "GLOSSARY.md").write_text("# G\n\n## Language\n\n### Robot\n\n**Hand**: The cup.\n")
    for b in ("active", "parked", "archived"):
        (root / "plans" / b).mkdir(parents=True)
    _write(root, "pathway", "p", 1, status="active")
    _write(root, "bug", "foo", 2)
    _write(root, "ticket", "t", 3, status="open", parent="p")


def _q(root, parent, title="Which way?"):
    return itemcli.new_item(str(root), "ticket", title, parent=parent, ticket_type="question",
                            commit=False)


def test_question_type_loads_and_unknown_fails(tmp_path):
    _tree(tmp_path)
    _write(tmp_path, "ticket", "q", 4, status="open", parent="bug-foo", type="question")
    assert load_item(str(tmp_path / "map" / "ticket-q.md")).meta["type"] == "question"
    _write(tmp_path, "ticket", "bad", 5, status="open", parent="p", type="riddle")
    with pytest.raises(ItemError):
        load_item(str(tmp_path / "map" / "ticket-bad.md"))


@pytest.mark.parametrize("body,want", [
    ("text\n\n## Answer\nUse the left one.\n", True),
    ("text\n", False),
    ("text\n\n## Answer\n\n   \n", False),
    ("## Answer\n\n## Progress\n- x\n", False),
    ("a\n\n## Answer  \n\nyes\n\n## Progress\n- x\n", True),
    ("## answer\nyes\n", False),
    ("## Answered\nyes\n", False),
])
def test_has_answer(body, want):
    assert has_answer(body) is want


@pytest.mark.parametrize("parent", ["bug-foo", "foo", "JB-2"])
def test_new_question_stores_parent_stem(tmp_path, parent):
    _tree(tmp_path)
    it = _q(tmp_path, parent)
    assert it.meta["parent"] == "bug-foo" and it.meta["type"] == "question"


def test_new_question_refuses_bad_parents(tmp_path):
    _tree(tmp_path)
    with pytest.raises(itemcli.ItemCliError, match="bug, feature or idea"):
        _q(tmp_path, "p")
    with pytest.raises(itemcli.ItemCliError, match="bug, feature or idea"):
        _q(tmp_path, "JB-3")
    with pytest.raises(itemcli.ItemCliError, match="not found"):
        _q(tmp_path, "bug-nope")


def test_non_question_ticket_keeps_pathway_rule(tmp_path):
    _tree(tmp_path)
    with pytest.raises(itemcli.ItemCliError, match="not a pathway"):
        itemcli.new_item(str(tmp_path), "ticket", "Do a thing", parent="bug-foo", ticket_type="task")


def test_question_tickets_and_open_questions_resolve_three_forms(tmp_path):
    _tree(tmp_path)
    q1 = _q(tmp_path, "JB-2", "First question?")
    q2 = _q(tmp_path, "bug-foo", "Second question?")
    itemcli.set_status(str(tmp_path), q2.slug, "complete", push=False)
    items = load_items(str(tmp_path / "map"))
    for form in ("bug-foo", "foo", "JB-2"):
        assert {t.slug for t in question_tickets(items, form)} == {q1.slug, q2.slug}
        assert [t.slug for t in open_questions(items, form)] == [q1.slug]
    assert question_tickets(items, "JB-1") == []


def test_complete_without_answer_warns_but_succeeds(tmp_path, capsys):
    _tree(tmp_path)
    q = _q(tmp_path, "bug-foo")
    itemcli.set_status(str(tmp_path), q.slug, "complete", push=False)
    assert "completed without an ## Answer" in capsys.readouterr().err
    assert load_item(q.path).status == "complete"


def test_complete_with_answer_is_silent(tmp_path, capsys):
    _tree(tmp_path)
    q = _q(tmp_path, "bug-foo")
    write_item(q.path, q.meta, q.body + "\n## Answer\nLeft.\n")
    itemcli.set_status(str(tmp_path), q.slug, "complete", push=False)
    assert "Answer" not in capsys.readouterr().err


def test_frontier_ignores_question_tickets(tmp_path):
    _tree(tmp_path)
    q = _q(tmp_path, "bug-foo")
    assert [t.slug for t in tickets.frontier(str(tmp_path), "p")] == ["t"]
    assert q.slug not in [t.slug for t in tickets.frontier(str(tmp_path), "p")]
