"""mapkit.plans: plan loading over the three boards and meta normalisation."""
from __future__ import annotations

import os

from mapkit.plans import (PlanError, load_plan, load_plans, normalise_plan_meta,
                          plan_paths)

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))

FORMERLY_INVALID = (
    "plans/archived/follower-cadence-and-divergence.md",
    "plans/archived/PROMPT-canbridge-rx-drain-throughput.md",
    "plans/archived/shaped-planning-efficiency.md",
)


def test_every_real_plan_parses():
    """Every plan on the three boards (INDEX.md skipped) parses; all offenders are listed."""
    paths = plan_paths(REPO_ROOT)
    assert len(paths) >= 69
    offenders = []
    for p in paths:
        try:
            load_plan(p)
        except PlanError as exc:
            offenders.append(str(exc).splitlines()[0])
    assert not offenders, "unparsable plans:\n" + "\n".join(offenders)


def test_formerly_invalid_archived_plans_parse():
    """The three archived files whose unquoted titles were invalid YAML now load with a str title."""
    for rel in FORMERLY_INVALID:
        plan = load_plan(os.path.join(REPO_ROOT, rel))
        assert isinstance(plan.meta.get("title"), str) and plan.meta["title"]
        assert plan.board == "archived"


def test_plan_paths_skip_index_and_include_prompts():
    """plan_paths covers active/parked/archived, skips INDEX.md, includes PROMPT-*.md."""
    names = [os.path.basename(p) for p in plan_paths(REPO_ROOT)]
    assert "INDEX.md" not in names
    assert any(n.startswith("PROMPT-") for n in names)
    boards = {os.path.basename(os.path.dirname(p)) for p in plan_paths(REPO_ROOT)}
    assert boards <= {"active", "parked", "archived"}


def test_normalisation_rules():
    """complete -> completed, str related_plan -> list, dates -> ISO strings (steps too); input untouched."""
    import datetime
    raw = {
        "status": "complete", "related_plan": "a.md",
        "created": datetime.date(2026, 1, 2), "archived": datetime.date(2026, 1, 3),
        "completed": datetime.date(2026, 1, 4), "last_updated": datetime.date(2026, 1, 5),
        "parked": datetime.datetime(2026, 1, 6, 7, 8),
        "steps": [{"id": "P1", "title": "t", "date": datetime.date(2026, 1, 7)}],
    }
    out = normalise_plan_meta(raw)
    assert out["status"] == "completed"
    assert out["related_plan"] == ["a.md"]
    assert (out["created"], out["archived"], out["completed"], out["last_updated"],
            out["parked"]) == ("2026-01-02", "2026-01-03", "2026-01-04", "2026-01-05",
                               "2026-01-06")
    assert out["steps"][0]["date"] == "2026-01-07"
    assert raw["status"] == "complete" and raw["related_plan"] == "a.md"
    assert normalise_plan_meta({"related_plan": ["x.md"]})["related_plan"] == ["x.md"]


def test_real_corpus_is_normalised():
    """Across the real corpus no plan reports status 'complete', related_plan is always a
    list, and every date field is a str."""
    bad = []
    for path in plan_paths(REPO_ROOT):
        try:
            plan = load_plan(path)
        except PlanError:
            continue  # reported by test_every_real_plan_parses
        m = plan.meta
        if m.get("status") == "complete":
            bad.append("%s: status complete" % plan.filename)
        if "related_plan" in m and not isinstance(m["related_plan"], list):
            bad.append("%s: related_plan not a list" % plan.filename)
        for k in ("created", "archived", "completed", "last_updated", "parked"):
            if m.get(k) is not None and not isinstance(m[k], str):
                bad.append("%s: %s is %s" % (plan.filename, k, type(m[k]).__name__))
    assert not bad, "\n".join(bad)


def test_load_plan_fixture(tmp_path):
    """load_plan fills board from the directory, parses steps:, and file-numbers headings."""
    d = tmp_path / "plans" / "active"
    d.mkdir(parents=True)
    f = d / "x.md"
    f.write_text("---\ntitle: X\nsteps:\n  - {id: Phase 1, title: One, kind: phase,"
                 " status: complete, date: 2026-10-18}\n---\n\n## P1 — One — COMPLETE\n\nSummary"
                 " line.\nSecond line.\n\nNext para.\n", encoding="utf-8")
    plan = load_plan(str(f))
    assert plan.board == "active" and plan.filename == "x.md" and plan.has_steps
    assert plan.steps[0].norm_id == "P1" and plan.steps[0].date == "2026-10-18"
    h = plan.headings[0]
    assert (h.norm_id, h.title, h.line_no) == ("P1", "One", 7)
    assert h.summary == "Summary line.\nSecond line."
    assert [p.filename for p in load_plans(str(tmp_path))] == ["x.md"]


def test_bad_yaml_raises_plan_error_with_path(tmp_path):
    """An invalid frontmatter raises PlanError naming the path."""
    f = tmp_path / "bad.md"
    f.write_text("---\ntitle: a: b\n---\n", encoding="utf-8")
    try:
        load_plan(str(f))
    except PlanError as exc:
        assert str(f) in str(exc)
    else:
        raise AssertionError("PlanError not raised")
