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
    # The six seed Pathways still exist. Their status is live data (the app and
    # `mapkit item set-status` move it), so only the closed one is pinned.
    by = {i.slug: i for i in _items() if i.kind == "pathway"}
    seed = ("two-ball-oval", "learner-generalisation", "wave-camera-can3",
            "ball-butler", "project-map", "two-ball-skill-stack")
    assert not [s for s in seed if s not in by], sorted(by)
    assert by["two-ball-skill-stack"].status == "complete"


def test_plan_pathways_exist():
    pathways = {i.slug for i in _items() if i.kind == "pathway"}
    bad = []
    for p in load_plans(REPO):
        if p.board in ("active", "parked") and p.meta.get("pathway") \
                and p.meta["pathway"] not in pathways:
            bad.append("%s: pathway %r" % (p.filename, p.meta["pathway"]))
    assert not bad, "\n".join(bad)


def test_item_ids_unique():
    # Plan map-item-ids.md, P2: every PRESENT id is the canonical JB-<n> and no
    # two Items share one. Presence is test_every_item_has_an_id (P6).
    from mapkit import ids

    items = _items()
    bad = ["%s: id %r is not JB-<n>" % (i.filename, i.meta.get("id"))
           for i in ids.malformed(items)]
    bad += ["%s shared by %s (python -m mapkit ids --repair)"
            % (k, ", ".join(i.filename for i in v))
            for k, v in sorted(ids.duplicates(items).items())]
    assert not bad, "\n".join(bad)


def test_every_item_has_an_id():
    # Plan map-item-ids.md, P6: every writer allocates (mapkit item new, the app's
    # create, /inbox), so an ID-less Item means a hand-written file or a stale
    # session. Fix: python -m mapkit ids --assign-missing, in its own map: commit.
    from mapkit import ids

    bad = [i.filename for i in ids.missing(_items())]
    assert not bad, ("Items without an id (python -m mapkit ids --assign-missing): "
                     + ", ".join(bad))
