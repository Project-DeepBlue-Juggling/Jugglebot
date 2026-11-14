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
    pre = {"bug": "BUG", "feature": "FEAT", "idea": "IDEA", "ticket": "TKT"}[kind]
    meta = {"id": "%s-%d" % (pre, n), "kind": kind, "title": "T " + slug, "status": status,
            "created": "2026-10-01", "updated": "2026-10-01T00:00:00Z"}
    if kind == "ticket":
        meta.update({"parent": "p", "type": "task"})
    if not (kind == "ticket" and extra.get("type") == "question"):
        meta.update({"night_skip": "out-of-reach", "night_skip_on": "2026-11-14",
                     "night_skip_note": "fixture"})   # classified: not unclassified
    meta.update(extra)
    write_item(str(root / "map" / ("%s-%s.md" % (kind, slug))), meta, body)


def _tree(root, question=False):
    (root / "map").mkdir(parents=True)
    write_item(str(root / "map" / "pathway-p.md"),
               {"id": "PWAY-1", "kind": "pathway", "title": "P", "status": "active",
                "created": "2026-10-01", "destination": "goal"}, "Body\n")
    _item(root, 1, "idea", "i1", priority="high")
    _item(root, 1, "feature", "f1", status="backlog", priority="low")
    _item(root, 1, "bug", "b-unconf", priority="high", diagnosis="hypothesis")
    _item(root, 2, "bug", "b-conf", priority="low", diagnosis="confirmed")
    _item(root, 1, "ticket", "t-task", status="open")
    if question:  # mapkit learns the `question` type in a later step; see _with_question
        _item(root, 2, "ticket", "t-q", status="complete", type="question")
    _item(root, 3, "bug", "b-done", status="complete")
    _item(root, 2, "idea", "i-drop", status="dropped")
    _item(root, 4, "bug", "b-early", priority="low", created="2026-09-01")


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
    assert _ids(rec) == ["BUG-2", "BUG-1", "BUG-4", "FEAT-1", "TKT-1", "TKT-2", "IDEA-1"]
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
    _item(tmp_path, 2, "feature", "new")
    c = pf.compare(prev, pf.build_record(str(tmp_path), "d2"))
    assert c["added"] == ["FEAT-2"] and not c["nothing_new"]
    os.remove(str(tmp_path / "map" / "feature-new.md"))
    _item(tmp_path, 1, "idea", "i1", priority="high", updated="2026-10-02T00:00:00Z",
          body="Edited\n")
    c = pf.compare(prev, pf.build_record(str(tmp_path), "d2"))
    assert c["changed"] == ["IDEA-1"] and not c["added"]
    _item(tmp_path, 1, "idea", "i1", priority="high", body="Body\n")
    _item(tmp_path, 1, "feature", "f1", status="complete", priority="low")
    c = pf.compare(prev, pf.build_record(str(tmp_path), "d2"))
    assert c["removed"] == ["FEAT-1"] and c["nothing_new"]   # removed is not news (C4)
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
    assert text.endswith("\n") and json.loads(text)["items"][0]["id"] == "BUG-2"
    r = _run("--repo", str(tmp_path), "--out", out2, "--previous", out1)
    assert r.returncode == 3 and r.stdout.strip() == "NOTHING_NEW items=6 unchanged=6"
    assert os.path.exists(out2) and not os.path.exists(out2 + ".tmp")
    _item(tmp_path, 1, "idea", "i1", priority="high", updated="2026-10-05T00:00:00Z",
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


# --- item-id-per-kind P1 (plan section 5, T-U7): filing order under both ID forms ---

def _bug(root, sid, slug, created="2026-10-01", kind="bug", **extra):
    meta = {"id": sid, "kind": kind, "title": "T " + slug, "status": "inbox",
            "created": created, "updated": "2026-10-01T00:00:00Z", "priority": "low"}
    meta.update(extra)
    write_item(str(root / "map" / ("%s-%s.md" % (kind, slug))), meta, "Body\n")


def test_per_kind_ids_keep_filing_order_not_a_sentinel(tmp_path):
    """T-U7 (silent failure 1): a per-kind ID must not reach the old ``10**9`` fallback.

    On the unfixed preflight every ``BUG-<n>`` keyed ``10**9``, so a same-day tie fell
    through to slug order: ``bug-a-ten`` (BUG-10) ran before ``bug-z-two`` (BUG-2).
    Filing order is ``created``, then the number within the kind, then the stem.
    """
    (tmp_path / "map").mkdir()
    _bug(tmp_path, "BUG-10", "a-ten")
    _bug(tmp_path, "BUG-2", "z-two")
    _bug(tmp_path, "BUG-3", "m-early", created="2026-09-01")
    _bug(tmp_path, "FEAT-2", "f-early", created="2026-09-01", kind="feature")
    assert _ids(pf.build_record(str(tmp_path), "d")) == ["BUG-3", "BUG-2", "BUG-10", "FEAT-2"]


def test_order_is_created_then_the_number_within_a_kind(tmp_path):
    """T-U7: per-kind IDs restart at 1 per kind; the order is created, then kind and number, then stem."""
    rows = [("bug", "z-a", "2026-10-01"), ("bug", "a-b", "2026-10-01"),
            ("feature", "f-a", "2026-10-01"), ("bug", "q-c", "2026-09-01"),
            ("feature", "f-b", "2026-10-01"), ("idea", "i-a", "2026-10-01")]
    (tmp_path / "map").mkdir(parents=True)
    seen = {}
    for kind, slug, created in rows:
        seen[kind] = seen.get(kind, 0) + 1
        sid = "%s-%d" % ({"bug": "BUG", "feature": "FEAT", "idea": "IDEA"}[kind], seen[kind])
        _bug(tmp_path, sid, slug, created=created, kind=kind)
    order = [e["stem"] for e in pf.build_record(str(tmp_path), "d")["items"]]
    assert order[:3] == ["bug-q-c", "bug-z-a", "bug-a-b"]
    assert sorted(order) == sorted("%s-%s" % (k, s_) for k, s_, _c in rows)


@pytest.mark.parametrize("bad", ["BUG-01", "XB-3", "FEAT-4"])
def test_unusable_id_is_loud_not_a_sentinel(tmp_path, bad):
    """T-U7: a malformed or kind-contradicting ID stops the preflight (exit 2)."""
    (tmp_path / "map").mkdir()
    _bug(tmp_path, "BUG-1", "fine")
    _bug(tmp_path, bad, "broken")
    with pytest.raises(pf.ItemError, match="bug-broken"):
        pf.build_record(str(tmp_path), "d")
    out = str(tmp_path / "c.json")
    r = _run("--repo", str(tmp_path), "--out", out)
    assert r.returncode == 2 and "bug-broken" in r.stdout and not os.path.exists(out)


def test_id_less_item_sorts_after_every_id_of_its_day_and_does_not_raise(tmp_path):
    """T-U7: ``ids --check`` calls a missing ID informational, so preflight keys it, never aborts.

    Key ``(created, "~", 0, stem)``: after every ID-bearing Item of the same ``created``,
    before a later day, ties broken by stem.
    """
    (tmp_path / "map").mkdir()
    _bug(tmp_path, "BUG-9", "nine")
    _bug(tmp_path, None, "aa-no-id")
    _bug(tmp_path, "BUG-2", "two")
    _bug(tmp_path, None, "zz-no-id")
    _bug(tmp_path, "BUG-1", "later", created="2026-10-02")
    got = [e["stem"] for e in pf.build_record(str(tmp_path), "d")["items"]]
    assert got == ["bug-two", "bug-nine", "bug-aa-no-id", "bug-zz-no-id", "bug-later"]


# --- closeout C4: the delta (table-driven over synthetic records) ---

def _e(id_, kind="bug", status="inbox", updated="u1", type_=None, blocked_by=None,
       night_skip=None):
    return {"id": id_, "stem": "%s-%s" % (kind, id_.lower()), "kind": kind, "status": status,
            "updated": updated, "type": type_, "blocked_by": blocked_by or [],
            "night_skip": night_skip}


def _rec(*entries):
    return {"items": list(entries)}


_Q_OPEN = dict(kind="ticket", type_="question", status="open")
_Q_DONE = dict(kind="ticket", type_="question", status="complete")

DELTA_CASES = [
    # (name, previous, current, previous carry, branches, full_walk, expected delta_why)
    ("no previous: everything added", None, [_e("BUG-1", night_skip="sitting")],
     [], set(), False, {"BUG-1": "added"}),
    ("unchanged and classified: nothing", [_e("BUG-1", night_skip="sitting")],
     [_e("BUG-1", night_skip="sitting")], [], set(), False, {}),
    ("updated moved: changed", [_e("BUG-1", night_skip="sitting")],
     [_e("BUG-1", updated="u2", night_skip="sitting")], [], set(), False, {"BUG-1": "changed"}),
    ("status moved: changed", [_e("BUG-1", night_skip="sitting")],
     [_e("BUG-1", status="in_progress", night_skip="sitting")], [], set(), False,
     {"BUG-1": "changed"}),
    ("unclassified: no skip, no branch", [_e("BUG-1")], [_e("BUG-1")], [], set(), False,
     {"BUG-1": "unclassified"}),
    ("a held branch classifies it", [_e("BUG-1")], [_e("BUG-1")], [], {"BUG-1"}, False, {}),
    ("carry", [_e("BUG-1", night_skip="sitting")], [_e("BUG-1", night_skip="sitting")],
     ["BUG-1"], set(), False, {"BUG-1": "carry"}),
    ("carry of a gone Item is dropped", [_e("BUG-1", night_skip="sitting")], [],
     ["BUG-1"], set(), False, {}),
    ("unblocked: blocker changed",
     [_e("BUG-1", night_skip="sitting", blocked_by=["bug-bug-2"]), _e("BUG-2", night_skip="x")],
     [_e("BUG-1", night_skip="sitting", blocked_by=["bug-bug-2"]),
      _e("BUG-2", updated="u2", night_skip="x")], [], set(), False,
     {"BUG-2": "changed", "BUG-1": "unblocked"}),
    ("unblocked: blocker closed (left the candidates)",
     [_e("BUG-1", night_skip="sitting", blocked_by=["BUG-2"]), _e("BUG-2", night_skip="x")],
     [_e("BUG-1", night_skip="sitting", blocked_by=["BUG-2"])], [], set(), False,
     {"BUG-1": "unblocked"}),
    ("removed alone is not news", [_e("BUG-1", night_skip="s"), _e("BUG-2", night_skip="s")],
     [_e("BUG-1", night_skip="s")], [], set(), False, {}),
    ("answered question: walked once", [_e("TKT-1", **_Q_OPEN)], [_e("TKT-1", updated="u2", **_Q_DONE)],
     [], set(), False, {"TKT-1": "answered"}),
    ("answered question next night: silent", [_e("TKT-1", **_Q_DONE)], [_e("TKT-1", **_Q_DONE)],
     [], set(), False, {}),
    ("question never unclassified or carried", [_e("TKT-1", **_Q_DONE)], [_e("TKT-1", **_Q_DONE)],
     ["TKT-1"], set(), False, {}),
    ("added question enters", [], [_e("TKT-1", **_Q_OPEN)], [], set(), False, {"TKT-1": "added"}),
    ("open question changed: changed", [_e("TKT-1", **_Q_OPEN)],
     [_e("TKT-1", updated="u2", **_Q_OPEN)], [], set(), False, {"TKT-1": "changed"}),
    ("full walk ignores night_skip, skips an answered question",
     [_e("BUG-1", night_skip="sitting"), _e("TKT-1", **_Q_DONE), _e("TKT-2", **_Q_OPEN)],
     [_e("BUG-1", night_skip="sitting"), _e("TKT-1", **_Q_DONE), _e("TKT-2", **_Q_OPEN)],
     [], set(), True, {"BUG-1": "full-walk", "TKT-2": "full-walk"}),
    ("precedence: added beats unclassified and carry", [], [_e("BUG-1")], ["BUG-1"], set(),
     False, {"BUG-1": "added"}),
    ("precedence: changed beats unclassified", [_e("BUG-1")], [_e("BUG-1", updated="u2")], [],
     set(), False, {"BUG-1": "changed"}),
    ("precedence: carry beats unclassified", [_e("BUG-1")], [_e("BUG-1")], ["BUG-1"], set(),
     False, {"BUG-1": "carry"}),
]


@pytest.mark.parametrize("name,prev,cur,carry,branches,full,want", DELTA_CASES,
                         ids=[c[0] for c in DELTA_CASES])
def test_delta_table(name, prev, cur, carry, branches, full, want):
    previous = None if prev is None else dict(_rec(*prev), carry=carry)
    got = pf.compute_delta(previous, _rec(*cur), branches, full)
    assert got["delta_why"] == want
    assert set(got["delta"]) == set(want)
    assert got["unchanged"] == len(cur) - len(want)


def test_delta_keeps_walk_order():
    cur = _rec(_e("BUG-9"), _e("BUG-2"), _e("FEAT-1", kind="feature"))
    assert pf.compute_delta(None, cur, set())["delta"] == ["BUG-9", "BUG-2", "FEAT-1"]


def test_cli_delta_keys_and_snapshot_carry(tmp_path):
    _tree(tmp_path)
    out1, last, out2 = (str(tmp_path / n) for n in ("c1.json", "last.json", "c2.json"))
    assert _run("--repo", str(tmp_path), "--out", out1).returncode == 0
    rec = json.load(open(out1))
    assert len(rec["delta"]) == 6 and set(rec["delta_why"].values()) == {"added"}
    assert rec["unchanged"] == 0 and rec["full_walk"] is False and len(rec["items"]) == 6
    report = tmp_path / "r.md"
    report.write_text("# Night Shift 2026-11-14\n\n## Merged to main\n\n"
                      "| ID | what |\n|---|---|\n| BUG-2 | fixed |\n\n## Held\nnone\n")
    r = _run("--repo", str(tmp_path), "--out", last, "--walked", out1, "--report", str(report))
    assert r.stdout.startswith("SNAPSHOT items=6 carry=5")
    carry = json.load(open(last))["carry"]
    assert "BUG-2" not in carry and sorted(carry) == sorted(set(rec["delta"]) - {"BUG-2"})
    r = _run("--repo", str(tmp_path), "--out", out2, "--previous", last)
    assert r.returncode == 0
    assert set(json.load(open(out2))["delta_why"].values()) == {"carry"}
    # no readable report: nothing is proven walked, everything carries
    _run("--repo", str(tmp_path), "--out", last, "--walked", out1,
         "--report", str(tmp_path / "missing.md"))
    assert len(json.load(open(last))["carry"]) == 6
    # --full-walk
    r = _run("--repo", str(tmp_path), "--out", out2, "--previous", out1, "--full-walk")
    assert r.returncode == 0 and "full_walk=1" in r.stdout
    assert set(json.load(open(out2))["delta_why"].values()) == {"full-walk"}


def test_branch_ids_classify_an_item(tmp_path):
    env = {"GIT_AUTHOR_NAME": "t", "GIT_AUTHOR_EMAIL": "t@t", "GIT_COMMITTER_NAME": "t",
           "GIT_COMMITTER_EMAIL": "t@t", "HOME": str(tmp_path), "PATH": os.environ["PATH"],
           "GIT_CONFIG_NOSYSTEM": "1"}
    subprocess.run(["git", "init", "-q", str(tmp_path)], env=env, check=True)
    subprocess.run(["git", "-C", str(tmp_path), "commit", "-q", "--allow-empty", "-m", "x"],
                   env=env, check=True)
    for ref in ("night-shift/2026-11-13-BUG-12", "night-shift/2026-11-13-salvage-0100", "other/x-BUG-3"):
        subprocess.run(["git", "-C", str(tmp_path), "update-ref", "refs/remotes/origin/" + ref, "HEAD"],
                       env=env, check=True)
    assert pf.branch_ids(str(tmp_path)) == {"BUG-12"}
