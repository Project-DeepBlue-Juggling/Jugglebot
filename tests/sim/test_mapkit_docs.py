"""mapkit derive + generated docs pages (project-map.md, glossary.md)."""
from __future__ import annotations

import os
import re

import pytest

from mapkit import derive, gen_docs
from mapkit.items import load_items, write_item
from mapkit.plans import load_plans

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def _item(root, kind, slug, **kw):
    meta = {"kind": kind, "title": kw.pop("title", slug), "status": kw.pop("status", "inbox"),
            "created": "2026-10-20"}
    meta.update(kw)
    write_item(str(root / "map" / ("%s-%s.md" % (kind, slug))), meta, "Body.\n")


def _plan(root, board, name, fm, body="body\n"):
    d = root / "plans" / board
    d.mkdir(parents=True, exist_ok=True)
    (d / name).write_text("---\n" + fm + "---\n\n" + body)


@pytest.fixture()
def repo(tmp_path):
    (tmp_path / "map").mkdir()
    (tmp_path / "GLOSSARY.md").write_text(
        "# G\n\n## Language\n\n### Robot\n\n**Hand**: The cup.\n_Avoid_: leg 6, \"cup\"\n\n"
        "### Tooling\n\n**Sitting**: A session.\n")
    _item(tmp_path, "pathway", "alpha", title="Alpha", status="active", priority="high",
          destination="Reach A.", owners=["Harrison"])
    _item(tmp_path, "pathway", "beta", title="Beta", status="active", priority="medium",
          destination="Reach B.")
    _item(tmp_path, "pathway", "gamma", title="Gamma", status="complete", priority="high",
          destination="Reached G.")
    _plan(tmp_path, "active", "a-plan.md",
          "title: A plan\nstatus: active\nowner: Harrison\npathway: alpha\nsteps:\n"
          "  - {id: P1, title: One, kind: phase, status: complete}\n"
          "  - {id: P2, title: Two, kind: phase, status: retired}\n"
          "  - {id: P3, title: Three, kind: phase, status: in_progress}\n"
          "  - {id: P4, title: Four, kind: phase, status: not_started}\n",
          "## Phase 1: One\n\n## Phase 2: Two\n\n## Phase 3: Three\n\n## Phase 4: Four\n")
    _plan(tmp_path, "parked", "b-plan.md",
          "title: B plan\nstatus: active\npathway: beta\n")
    _item(tmp_path, "idea", "spark", title="A spark", priority="low", parent="alpha")
    _item(tmp_path, "bug", "stray", title="Stray | bug", status="backlog")
    _item(tmp_path, "feature", "old", title="Old", status="complete", parent="alpha")
    _item(tmp_path, "ticket", "tk1", title="Ticket one", status="open", parent="alpha",
          type="task", rank=2, owners=["Claude"])
    return tmp_path


def _load(root):
    items = load_items(str(root / "map"))
    return items, load_plans(str(root)), {i.slug: i for i in items if i.kind == "pathway"}


def test_pathway_order(repo):
    items, _, _ = _load(repo)
    # priority first (alpha, gamma high; beta medium), then active before complete
    assert [p.slug for p in derive.pathway_order(items)] == ["alpha", "gamma", "beta"]


def test_progress_excludes_retired(repo):
    items, plans, pw = _load(repo)
    assert derive.progress(pw["alpha"], plans) == (1, 3)
    assert derive.progress(pw["beta"], plans) == (0, 1)  # plan as one unit


def test_next_and_lanes(repo):
    items, plans, pw = _load(repo)
    nxt = derive.next_stop(pw["alpha"], plans, items)
    assert nxt.step_id == "P3"  # in-progress step is the first not-done stop
    ln = derive.lanes(pw["alpha"], plans, items)
    assert [u.step_id for u in ln["now"]] == ["P3"]
    assert ln["next"] == []  # already in Now
    assert [getattr(e, "step_id", None) or e.slug for e in ln["later"]] == ["P4", "spark"]
    assert [u.step_id for u in ln["done"]] == ["P1"]
    assert derive.next_stop(pw["gamma"], plans, items) is None


def test_next_waits_on_blocker(repo):
    _plan(repo, "active", "c-plan.md",
          "title: C plan\nstatus: active\npathway: delta\nblocked_by: [ticket-gate]\n"
          "steps:\n  - {id: P1, title: C one, kind: phase, status: not_started}\n",
          "## Phase 1: C one\n")
    _item(repo, "pathway", "delta", title="Delta", status="active", destination="D.")
    _item(repo, "ticket", "gate", title="The gate", status="open", parent="delta",
          type="task", rank=1)
    items, plans, pw = _load(repo)
    nxt = derive.next_stop(pw["delta"], plans, items)
    assert nxt.kind == "blocker" and nxt.label == "The gate"
    # blocker resolved: Next is the plan's step
    _item(repo, "ticket", "gate", title="The gate", status="complete", parent="delta",
          type="task", rank=1)
    items, plans, pw = _load(repo)
    assert derive.next_stop(pw["delta"], plans, items).step_id == "P1"


def test_render_sections_links_and_determinism(repo):
    text = gen_docs.render_project_map(str(repo))
    assert text == gen_docs.render_project_map(str(repo))
    heads = re.findall(r"^## (.+)$", text, re.M)
    assert heads == ["Alpha", "Gamma", "Beta", "Now / Next / Later", "Inbox", "Open tickets"]
    assert "1 of 3 stops done" in text
    assert "| Pathway | Now | Next | Later | Done |" in text
    assert "| ID | Kind | Title | Status | Priority | Effort | HW | Owner | Radius | Area | Pathway | Age |" in text
    assert "filed 2026-10-20" in text
    assert "Stray \\| bug" in text
    assert "### Alpha" in text and "Ticket one" in text
    for target in re.findall(r"\]\(([^)]+)\)", text):
        assert target.startswith(gen_docs.REPO_URL), target
    gl = gen_docs.render_glossary_page(str(repo))
    assert "### Robot" in gl and "**Hand**" in gl and "_Avoid: leg 6, \"cup\"_" in gl
    assert gl == gen_docs.render_glossary_page(str(repo))


def test_write_docs_roundtrip_and_noop(repo, tmp_path_factory):
    (repo / "docs").mkdir()
    assert len(gen_docs.write_docs(str(repo))) == 2
    assert gen_docs.write_docs(str(repo), check=True) == []
    empty = tmp_path_factory.mktemp("nomap")
    assert gen_docs.write_docs(str(empty)) == []


@pytest.mark.skipif(not os.path.isdir(os.path.join(REPO, "map")),
                    reason="map/ does not exist yet; the currency check starts with the map seed")
@pytest.mark.parametrize("rel,render", [
    ("project-map.md", gen_docs.render_project_map),
    ("glossary.md", gen_docs.render_glossary_page)])
def test_committed_pages_are_current(rel, render):
    with open(os.path.join(REPO, "docs", rel), encoding="utf-8") as fh:
        committed = fh.read()
    assert committed == render(REPO), "docs/%s is stale: run `python -m mapkit gen`" % rel


def test_project_map_shows_ids_and_warns_on_a_duplicate(repo):
    _item(repo, "idea", "spark2", title="Second spark", priority="low", parent="alpha", id="JB-5")
    text = gen_docs.render_project_map(str(repo))
    assert re.search(r"^\| JB-5 \| idea \| \[Second spark\]", text, re.MULTILINE)
    assert re.search(r"^\| — \| idea \| \[A spark\]", text, re.MULTILINE)  # ID-less: a dash
    assert "- idea: JB-5 [Second spark]" in text
    assert "duplicate Item IDs" not in text
    _item(repo, "bug", "twin", title="Twin", id="JB-5")
    text = gen_docs.render_project_map(str(repo))
    assert "> **Warning: duplicate Item IDs** — JB-5: bug-twin, idea-spark2" in text
