"""Contract for the committed ``map/`` items and ``map/config.yaml``.

One looping test per concern; each lists every offender. Pure text, no ports,
safe under xdist.
"""
from __future__ import annotations

import glob
import os

import yaml

from mapkit.items import (INBOX_KINDS, KINDS, LIST_FIELDS, PRIORITIES,
                          REQUIRED_FIELDS, STATUS_SETS, ItemError, load_item,
                          load_items, split_item_filename)
from mapkit.plans import load_plans

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
MAP = os.path.join(REPO, "map")


def _items():
    return load_items(MAP)


def test_every_item_loads():
    bad = []
    for p in sorted(glob.glob(os.path.join(MAP, "*.md"))):
        try:
            load_item(p)
        except ItemError as exc:
            bad.append(str(exc))
    assert not bad, "\n".join(bad)
    assert _items()


def test_fields_statuses_and_types():
    bad = []
    for it in _items():
        m = it.meta
        for f in REQUIRED_FIELDS[it.kind]:
            if m.get(f) in (None, ""):
                bad.append("%s: missing %s" % (it.filename, f))
        if m["status"] not in STATUS_SETS[it.kind]:
            bad.append("%s: status %r" % (it.filename, m["status"]))
        if m.get("priority") not in PRIORITIES:
            bad.append("%s: priority %r" % (it.filename, m.get("priority")))
        for f in LIST_FIELDS:
            if not isinstance(m.get(f), list):
                bad.append("%s: %s is not a list" % (it.filename, f))
        if it.kind in INBOX_KINDS and not m.get("filed_by"):
            bad.append("%s: inbox kind without filed_by" % it.filename)
    assert not bad, "\n".join(bad)


def test_references_resolve():
    items = _items()
    slugs = {i.slug for i in items}
    stems = {i.filename[:-3] for i in items}  # blocked_by targets are <kind>-<slug> stems
    pathways = {i.slug for i in items if i.kind == "pathway"}
    plans = {p.filename for p in load_plans(REPO)}
    bad = []
    for it in items:
        parent = it.meta.get("parent")
        if parent and parent not in pathways:
            bad.append("%s: parent %r is not a pathway" % (it.filename, parent))
        for t in it.meta.get("blocked_by") or []:
            if t not in stems and t not in plans:
                bad.append("%s: blocked_by %r unresolved" % (it.filename, t))
        pt = it.meta.get("promoted_to")
        if pt and pt not in plans and pt not in pathways:
            bad.append("%s: promoted_to %r unresolved" % (it.filename, pt))
    assert not bad, "\n".join(bad)


def test_slug_matches_filename():
    bad = []
    for p in sorted(glob.glob(os.path.join(MAP, "*.md"))):
        kind, slug = split_item_filename(os.path.basename(p))
        it = load_item(p)
        if (it.kind, it.slug) != (kind, slug):
            bad.append(os.path.basename(p))
    assert not bad, bad


def test_config_matches_mapkit():
    with open(os.path.join(MAP, "config.yaml"), encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)
    assert tuple(cfg["kinds"]) == KINDS
    assert set(cfg["statuses"]) == set(STATUS_SETS)
    for k, v in cfg["statuses"].items():
        assert set(v) == set(STATUS_SETS[k]) and len(v) == len(set(v)), k
    missing = []
    for key in ("items", "glossary", "logbook"):
        if not os.path.exists(os.path.join(REPO, cfg[key])):
            missing.append(cfg[key])
    for d in cfg["plans"]:
        if not os.path.isdir(os.path.join(REPO, d)):
            missing.append(d)
    assert not missing, missing


def test_seed_pathways():
    by = {i.slug: i for i in _items() if i.kind == "pathway"}
    expect = {"two-ball-oval": "active", "learner-generalisation": "active",
              "wave-camera-can3": "active", "ball-butler": "active",
              "project-map": "active", "two-ball-skill-stack": "complete"}
    wrong = {s: (by[s].status if s in by else None) for s, st in expect.items()
             if s not in by or by[s].status != st}
    assert not wrong, wrong


def test_plan_pathways_exist():
    pathways = {i.slug for i in _items() if i.kind == "pathway"}
    bad = []
    for p in load_plans(REPO):
        if p.board in ("active", "parked") and p.meta.get("pathway") \
                and p.meta["pathway"] not in pathways:
            bad.append("%s: pathway %r" % (p.filename, p.meta["pathway"]))
    assert not bad, "\n".join(bad)
