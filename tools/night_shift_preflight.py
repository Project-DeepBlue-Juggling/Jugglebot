#!/usr/bin/env python3
"""Night Shift preflight: list the board's workable Items, say if anything is new.

Why: the 04:45 Night Shift run must cost nothing on a night when the board did not
change, so `claude` is only invoked when this tool exits 0. It writes the candidate
record (`candidates.json`) and compares it with the previous night's.

Candidates (Pathways are never candidates):
  (a) bug/feature/idea with status inbox, backlog or in_progress;
  (b) ticket with status open;
  (c) ticket of type "question" at ANY status (an answered question's `updated`
      stamp moves, which re-opens its parent for the next night).

Order (the order the orchestrator works them): bugs with `diagnosis: confirmed`,
then other bugs, then features, then tickets, then ideas; within a group priority
high > medium > low (missing = low), then `created` oldest first, then Item ID.

Exit codes: 0 NEW, 3 NOTHING_NEW, 2 error (nothing written). The --out file is
always written (atomically) on 0 and 3.

Usage:
  python tools/night_shift_preflight.py --repo <checkout> --out <candidates.json>
      [--previous <path>] [--date YYYY-MM-DD]
"""
from __future__ import annotations

import argparse
import datetime
import json
import os
import subprocess
import sys
from typing import Any, Dict, List, Optional

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mapkit.items import Item, ItemError, load_items  # noqa: E402

EXIT_NEW = 0
EXIT_ERROR = 2
EXIT_NOTHING_NEW = 3

_WORKABLE_INBOX = ("inbox", "backlog", "in_progress")
_PRIORITY_RANK = {"high": 0, "medium": 1, "low": 2}
_FIELDS = ("id", "stem", "kind", "status", "title", "priority", "created", "updated",
           "parent", "type", "effort", "hardware", "owner", "radius", "diagnosis", "area")


def is_candidate(item: Item) -> bool:
    if item.kind == "pathway":
        return False
    if item.kind in ("bug", "feature", "idea"):
        return item.status in _WORKABLE_INBOX
    if item.kind == "ticket":
        return item.status == "open" or item.meta.get("type") == "question"
    return False


def _group(item: Item) -> int:
    if item.kind == "bug":
        return 0 if item.meta.get("diagnosis") == "confirmed" else 1
    return {"feature": 2, "ticket": 3, "idea": 4}[item.kind]


def _id_num(item: Item) -> int:
    sid = item.short_id or ""
    return int(sid[3:]) if sid.startswith("JB-") and sid[3:].isdigit() else 10 ** 9


def _sort_key(item: Item):
    return (_group(item), _PRIORITY_RANK.get(item.meta.get("priority"), 2),
            str(item.meta.get("created") or ""), _id_num(item), item.slug)


def _entry(item: Item, rank: int) -> Dict[str, Any]:
    m = item.meta
    d = {"id": item.short_id, "stem": "%s-%s" % (item.kind, item.slug), "kind": item.kind,
         "status": item.status, "title": item.title}
    for f in _FIELDS[5:]:
        d[f] = m.get(f)
    d["rank"] = rank
    return d


def _head(repo_root: str) -> Optional[str]:
    try:
        r = subprocess.run(["git", "-C", repo_root, "rev-parse", "HEAD"],
                           stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                           universal_newlines=True)
    except OSError:
        return None
    return r.stdout.strip() if r.returncode == 0 and r.stdout.strip() else None


def build_record(repo_root: str, date: str) -> Dict[str, Any]:
    """The candidate record; raises ItemError on a malformed Item."""
    map_dir = os.path.join(repo_root, "map")
    if not os.path.isdir(map_dir):
        raise ItemError("%s: no map/ directory" % repo_root)
    cands = sorted((i for i in load_items(map_dir) if is_candidate(i)), key=_sort_key)
    return {"date": date, "head": _head(repo_root),
            "items": [_entry(i, n) for n, i in enumerate(cands, 1)]}


def _key(entry: Dict[str, Any]) -> str:
    return entry.get("id") or entry.get("stem") or ""


def compare(previous: Optional[Dict[str, Any]], current: Dict[str, Any]) -> Dict[str, Any]:
    prev = {_key(e): e for e in (previous or {}).get("items", [])}
    cur = {_key(e): e for e in current.get("items", [])}
    added = sorted(k for k in cur if k not in prev)
    removed = sorted(k for k in prev if k not in cur)
    changed = sorted(k for k in cur if k in prev and (
        cur[k].get("updated") != prev[k].get("updated")
        or cur[k].get("status") != prev[k].get("status")))
    return {"nothing_new": not (added or removed or changed),
            "added": added, "removed": removed, "changed": changed}


def _load_previous(path: Optional[str]) -> Optional[Dict[str, Any]]:
    if not path:
        return None
    try:
        with open(path, encoding="utf-8") as fh:
            data = json.load(fh)
    except (OSError, ValueError):
        return None
    return data if isinstance(data, dict) else None


def _write_atomic(path: str, record: Dict[str, Any]) -> None:
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as fh:
        fh.write(json.dumps(record, sort_keys=True, indent=1) + "\n")
    os.replace(tmp, path)


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="Night Shift preflight")
    ap.add_argument("--repo", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--previous")
    ap.add_argument("--date", default=None)
    args = ap.parse_args(argv)
    date = args.date or datetime.date.today().isoformat()
    try:
        record = build_record(os.path.abspath(args.repo), date)
    except (ItemError, OSError) as exc:
        print("ERROR %s" % exc)
        return EXIT_ERROR
    result = compare(_load_previous(args.previous), record)
    _write_atomic(args.out, record)
    n = len(record["items"])
    if result["nothing_new"]:
        print("NOTHING_NEW items=%d" % n)
        return EXIT_NOTHING_NEW
    print("NEW items=%d added=%d changed=%d removed=%d" % (
        n, len(result["added"]), len(result["changed"]), len(result["removed"])))
    return EXIT_NEW


if __name__ == "__main__":
    sys.exit(main())
