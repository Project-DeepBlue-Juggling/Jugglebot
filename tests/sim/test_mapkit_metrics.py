"""Item metrics (effort, hardware, owner, radius, diagnosis, area): schema, validation,
CLI flags, `item rate`, canonical key order, the project-map columns, plan-step metrics.

The spec is the owner-approved 2026-10-28 metrics table in DOCUMENTATION_GUIDE.md 2.10;
`mapkit.items.METRIC_ENUMS` is the single in-code copy.
"""
from __future__ import annotations

import argparse

import pytest

from mapkit import itemcli
from mapkit.items import METRIC_ENUMS, METRIC_FIELDS, ItemError, load_item, write_item
from mapkit.plans import load_plan
from mapkit.steps import check_step_vocab


def _write(d, name, meta, body="Body.\n"):
    p = d / name
    p.write_text("---\n" + meta + "---\n" + body, encoding="utf-8")
    return str(p)


_BUG = "kind: bug\ntitle: Feed y bias\nstatus: inbox\ncreated: 2026-10-21\npriority: high\n"
_OTHERS = {
    "idea-x.md": "kind: idea\ntitle: X\nstatus: inbox\ncreated: 2026-10-20\n",
    "feature-x.md": "kind: feature\ntitle: X\nstatus: inbox\ncreated: 2026-10-20\n",
    "ticket-x.md": "kind: ticket\ntitle: X\nstatus: open\ncreated: 2026-10-24\n"
                   "parent: p\ntype: task\n",
    "pathway-x.md": "kind: pathway\ntitle: X\nstatus: active\ncreated: 2026-10-14\n"
                    "destination: d\n",
}


def test_spec_shape():
    assert METRIC_FIELDS == ("effort", "hardware", "owner", "radius", "diagnosis", "area")
    assert METRIC_ENUMS["effort"] == ("xs", "s", "m", "l", "xl")
    assert METRIC_ENUMS["hardware"] == ("none", "bench-leg", "robot")
    assert METRIC_ENUMS["owner"] == ("none", "decision", "hands")
    assert METRIC_ENUMS["radius"] == ("local", "subsystem", "cross-cutting", "normative")
    assert METRIC_ENUMS["diagnosis"] == ("unknown", "hypothesis", "confirmed")
    assert len(METRIC_ENUMS["area"]) == 14


def test_every_value_of_every_enum_loads(tmp_path):
    for f in METRIC_FIELDS:
        for v in METRIC_ENUMS[f]:
            it = load_item(_write(tmp_path, "bug-b.md", _BUG + "%s: %s\n" % (f, v)))
            assert it.meta[f] == v


@pytest.mark.parametrize("field,value", [
    ("effort", "huge"), ("effort", "S"), ("hardware", "bench"), ("owner", "claude"),
    ("radius", "global"), ("diagnosis", "maybe"), ("area", "everywhere")])
def test_metric_outside_its_enum_is_rejected(tmp_path, field, value):
    with pytest.raises(ItemError, match=field):
        load_item(_write(tmp_path, "bug-b.md", _BUG + "%s: %s\n" % (field, value)))


@pytest.mark.parametrize("name", sorted(_OTHERS))
def test_diagnosis_on_a_non_bug_is_rejected(tmp_path, name):
    with pytest.raises(ItemError, match="diagnosis"):
        load_item(_write(tmp_path, name, _OTHERS[name] + "diagnosis: unknown\n"))


def test_metrics_are_optional_on_every_kind(tmp_path):
    for name, meta in dict(_OTHERS, **{"bug-b.md": _BUG}).items():
        it = load_item(_write(tmp_path, name, meta))
        assert not any(f in it.meta for f in METRIC_FIELDS)


# --- plan steps ------------------------------------------------------------------------------

def _plan(tmp_path, steps_yaml):
    f = tmp_path / "plan.md"
    f.write_text("---\ntitle: T\n" + steps_yaml + "---\n## P1 - One\n", encoding="utf-8")
    return load_plan(str(f))


def test_step_effort_and_hardware_validated(tmp_path):
    ok = _plan(tmp_path, "steps:\n  - {id: P1, title: One, effort: m, hardware: robot}\n")
    assert ok.steps[0].effort == "m" and ok.steps[0].hardware == "robot"
    assert check_step_vocab(ok) == []
    bad = _plan(tmp_path, "steps:\n  - {id: P1, title: One, effort: huge, hardware: lab}\n")
    msgs = check_step_vocab(bad)
    assert len(msgs) == 2
    assert any("effort" in m for m in msgs) and any("hardware" in m for m in msgs)


# --- CLI -------------------------------------------------------------------------------------

def _tree(root):
    (root / "map").mkdir(parents=True, exist_ok=True)
    (root / "docs").mkdir(exist_ok=True)
    (root / "GLOSSARY.md").write_text("# G\n\n## Language\n\n### Robot\n\n**Hand**: The cup.\n")
    for b in ("active", "parked", "archived"):
        (root / "plans" / b).mkdir(parents=True, exist_ok=True)
    write_item(str(root / "map" / "pathway-p.md"),
               {"kind": "pathway", "title": "P", "status": "active",
                "created": "2026-10-01", "destination": "reach the goal"}, "Body\n")


def _item(root, kind, slug):
    meta = {"kind": kind, "title": "T " + slug, "status": "inbox", "created": "2026-10-01",
            "updated": "2026-10-01T00:00:00Z", "priority": "medium", "parent": "p"}
    write_item(str(root / "map" / ("%s-%s.md" % (kind, slug))), meta, "Body text\n")


def test_new_with_metrics_orders_keys_after_priority_and_page_has_columns(tmp_path):
    _tree(tmp_path)
    item = itemcli.new_item(str(tmp_path), "bug", "Metric bug", effort="s", hardware="none",
                            owner="none", radius="local", diagnosis="confirmed", area="gui")
    keys = list(load_item(item.path).meta)
    i = keys.index("priority")
    assert tuple(keys[i + 1:i + 7]) == METRIC_FIELDS
    page = (tmp_path / "docs" / "project-map.md").read_text()
    assert "| Effort | HW | Owner | Radius | Area |" in page
    row = [ln for ln in page.splitlines() if "Metric bug" in ln][0]
    assert "| s | none | none | local | gui |" in row


def test_new_without_metrics_writes_none(tmp_path):
    _tree(tmp_path)
    item = itemcli.new_item(str(tmp_path), "idea", "Plain idea")
    assert not any(f in load_item(item.path).meta for f in METRIC_FIELDS)
    page = (tmp_path / "docs" / "project-map.md").read_text()
    row = [ln for ln in page.splitlines() if "Plain idea" in ln][0]
    assert "| — | — | — | — | — |" in row


def test_new_rejects_bad_metrics_before_writing(tmp_path):
    _tree(tmp_path)
    with pytest.raises(itemcli.ItemCliError, match="effort"):
        itemcli.new_item(str(tmp_path), "idea", "X", effort="huge")
    with pytest.raises(itemcli.ItemCliError, match="bugs only"):
        itemcli.new_item(str(tmp_path), "idea", "X", diagnosis="unknown")
    assert [p.name for p in (tmp_path / "map").iterdir()] == ["pathway-p.md"]


def test_rate_sets_metrics_in_canonical_order_and_validates(tmp_path):
    _tree(tmp_path)
    _item(tmp_path, "bug", "b")
    itemcli.rate_item(str(tmp_path), "b", {"area": "gui", "effort": "xs"})
    itemcli.rate_item(str(tmp_path), "b", {"diagnosis": "hypothesis", "hardware": "robot"})
    meta = load_item(str(tmp_path / "map" / "bug-b.md")).meta
    assert [k for k in meta if k in METRIC_FIELDS] == ["effort", "hardware", "diagnosis", "area"]
    assert list(meta).index("effort") == list(meta).index("priority") + 1
    with pytest.raises(itemcli.ItemCliError, match="nothing to set"):
        itemcli.rate_item(str(tmp_path), "b", {})
    _item(tmp_path, "idea", "i")
    with pytest.raises(itemcli.ItemCliError, match="bugs only"):
        itemcli.rate_item(str(tmp_path), "i", {"diagnosis": "unknown"})


def test_rate_cli_entry_and_noop(tmp_path, capsys):
    _tree(tmp_path)
    _item(tmp_path, "bug", "b")
    ns = argparse.Namespace(repo=str(tmp_path), item="b", commit=False, no_push=True,
                            effort="m", hardware=None, owner=None, radius=None,
                            diagnosis=None, area=None)
    assert itemcli._run_rate(ns) == 0
    assert "effort=m" in capsys.readouterr().out
    assert itemcli._run_rate(ns) == 0 and "unchanged" in capsys.readouterr().out
    ns.effort = "huge"
    assert itemcli._run_rate(ns) == 2
