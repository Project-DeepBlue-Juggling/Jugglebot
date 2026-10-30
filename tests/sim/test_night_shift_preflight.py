"""Night Shift preflight (T-U2, pure part): candidates, order, compare, CLI, head."""
from __future__ import annotations

import json
import os
import subprocess
import sys

import pytest

from mapkit.items import write_item

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
TOOL = os.path.join(ROOT, "tools", "night_shift_preflight.py")
sys.path.insert(0, os.path.join(ROOT, "tools"))
import night_shift_preflight as pf  # noqa: E402


def _item(root, n, kind, slug, status="inbox", body="Body\n", **extra):
    meta = {"id": "JB-%d" % n, "kind": kind, "title": "T " + slug, "status": status,
            "created": "2026-10-01", "updated": "2026-10-01T00:00:00Z"}
    if kind == "ticket":
        meta.update({"parent": "p", "type": "task"})
    meta.update(extra)
    write_item(str(root / "map" / ("%s-%s.md" % (kind, slug))), meta, body)


def _tree(root, question=False):
    (root / "map").mkdir(parents=True)
    write_item(str(root / "map" / "pathway-p.md"),
               {"id": "JB-1", "kind": "pathway", "title": "P", "status": "active",
                "created": "2026-10-01", "destination": "goal"}, "Body\n")
    _item(root, 2, "idea", "i1", priority="high")
    _item(root, 3, "feature", "f1", status="backlog", priority="low")
    _item(root, 4, "bug", "b-unconf", priority="high", diagnosis="hypothesis")
    _item(root, 5, "bug", "b-conf", priority="low", diagnosis="confirmed")
    _item(root, 6, "ticket", "t-task", status="open")
    if question:  # mapkit learns the `question` type in a later step; see _with_question
        _item(root, 7, "ticket", "t-q", status="complete", type="question")
    _item(root, 8, "bug", "b-done", status="complete")
    _item(root, 9, "idea", "i-drop", status="dropped")
    _item(root, 10, "bug", "b-early", priority="low", created="2026-09-01")


@pytest.fixture
def _with_question(monkeypatch):
    """Teach the loader the `question` ticket type (not in mapkit yet) in-process."""
    import mapkit.items as mi
    monkeypatch.setattr(mi, "TICKET_TYPES", mi.TICKET_TYPES + ("question",))


def _ids(rec):
    return [e["id"] for e in rec["items"]]


def test_candidates_and_order(tmp_path, _with_question):
    _tree(tmp_path, question=True)
    rec = pf.build_record(str(tmp_path), "2026-10-31")
    # confirmed bug, other bugs (high first, then low by created), feature, tickets, idea
    assert _ids(rec) == ["JB-5", "JB-4", "JB-10", "JB-3", "JB-6", "JB-7", "JB-2"]
    assert [e["rank"] for e in rec["items"]] == [1, 2, 3, 4, 5, 6, 7]
    first = rec["items"][0]
    assert first["stem"] == "bug-b-conf" and first["diagnosis"] == "confirmed"
    assert first["parent"] is None and first["effort"] is None
    assert set(first) == set(pf._FIELDS) | {"rank"}
    assert rec["head"] is None and rec["date"] == "2026-10-31"


def test_compare(tmp_path, _with_question):
    _tree(tmp_path, question=True)
    prev = pf.build_record(str(tmp_path), "d1")
    assert pf.compare(prev, pf.build_record(str(tmp_path), "d2"))["nothing_new"]
    _item(tmp_path, 11, "feature", "new")
    c = pf.compare(prev, pf.build_record(str(tmp_path), "d2"))
    assert c["added"] == ["JB-11"] and not c["nothing_new"]
    os.remove(str(tmp_path / "map" / "feature-new.md"))
    _item(tmp_path, 2, "idea", "i1", priority="high", updated="2026-10-02T00:00:00Z",
          body="Edited\n")
    c = pf.compare(prev, pf.build_record(str(tmp_path), "d2"))
    assert c["changed"] == ["JB-2"] and not c["added"]
    _item(tmp_path, 2, "idea", "i1", priority="high", body="Body\n")
    _item(tmp_path, 3, "feature", "f1", status="complete", priority="low")
    c = pf.compare(prev, pf.build_record(str(tmp_path), "d2"))
    assert c["removed"] == ["JB-3"]
    c = pf.compare(None, prev)
    assert c["added"] == sorted(_ids(prev)) and not c["removed"]


def _run(*args):
    return subprocess.run([sys.executable, TOOL] + list(args), stdout=subprocess.PIPE,
                          stderr=subprocess.PIPE, universal_newlines=True)


def test_cli(tmp_path):
    _tree(tmp_path)
    out1, out2 = str(tmp_path / "c1.json"), str(tmp_path / "c2.json")
    r = _run("--repo", str(tmp_path), "--out", out1, "--date", "2026-10-31")
    assert r.returncode == 0 and r.stdout.startswith("NEW items=6 added=6")
    text = open(out1).read()
    assert text.endswith("\n") and json.loads(text)["items"][0]["id"] == "JB-5"
    r = _run("--repo", str(tmp_path), "--out", out2, "--previous", out1)
    assert r.returncode == 3 and r.stdout.strip() == "NOTHING_NEW items=6"
    assert os.path.exists(out2) and not os.path.exists(out2 + ".tmp")
    _item(tmp_path, 2, "idea", "i1", priority="high", updated="2026-10-05T00:00:00Z",
          body="Edited\n")
    r = _run("--repo", str(tmp_path), "--out", out2, "--previous", out1)
    assert r.returncode == 0 and "changed=1" in r.stdout
    # unreadable previous -> everything added
    bad = tmp_path / "bad.json"
    bad.write_text("{nope")
    r = _run("--repo", str(tmp_path), "--out", out2, "--previous", str(bad))
    assert r.returncode == 0 and "added=6" in r.stdout


def test_malformed_item_exits_2_no_out(tmp_path):
    _tree(tmp_path)
    (tmp_path / "map" / "bug-broken.md").write_text(
        "---\nkind: bug\nstatus: inbox\ncreated: 2026-10-01\n---\n\nbody\n")
    out = str(tmp_path / "c.json")
    r = _run("--repo", str(tmp_path), "--out", out)
    assert r.returncode == 2 and "title" in r.stdout and not os.path.exists(out)
    r = _run("--repo", str(tmp_path / "nonexistent"), "--out", out)
    assert r.returncode == 2 and not os.path.exists(out)


def test_head_is_git_sha_or_null(tmp_path, monkeypatch):
    _tree(tmp_path)
    assert pf.build_record(str(tmp_path), "d")["head"] is None
    env = {"GIT_AUTHOR_NAME": "t", "GIT_AUTHOR_EMAIL": "t@t", "GIT_COMMITTER_NAME": "t",
           "GIT_COMMITTER_EMAIL": "t@t", "GIT_CONFIG_GLOBAL": str(tmp_path / "gc"),
           "GIT_CONFIG_NOSYSTEM": "1", "HOME": str(tmp_path), "PATH": os.environ["PATH"]}
    for cmd in (["init", "-q"], ["add", "."], ["commit", "-q", "-m", "x"]):
        subprocess.run(["git", "-C", str(tmp_path)] + cmd, env=env, check=True)
    sha = subprocess.run(["git", "-C", str(tmp_path), "rev-parse", "HEAD"], env=env,
                         stdout=subprocess.PIPE, universal_newlines=True).stdout.strip()
    assert pf.build_record(str(tmp_path), "d")["head"] == sha
