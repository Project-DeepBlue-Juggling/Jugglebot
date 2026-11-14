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
high > medium > low (missing = low), then `created` oldest first, then Item ID (the
number within its prefix), then stem. An Item whose ID is missing,
malformed or names another kind is an error (exit 2), never a sort-last fallback.

THE INVARIANT (closeout C4, owner 2026-11-14): the night examines only the DELTA.
The delta is a subset of the candidates, in the same walk order, and `delta_why`
names ONE reason per member, the first that applies:

  full-walk   --full-walk (Sundays; `$REPORTS/full-walk.request`): every open Item,
              ignoring night_skip. An answered question ticket is not open and falls
              through to the rules below.
  added       not in the previous snapshot (no snapshot: everything is added).
  answered    a question ticket whose `updated`/`status` moved and whose status is
              not open: the owner just answered it. Walked once, never reported again.
  changed     `updated` or `status` differs from the previous snapshot.
  unblocked   `blocked_by` names an Item that changed, or that left the candidates
              (closed or dropped: the blocker is resolved).
  carry       named in the previous snapshot's `carry[]`: tonight's delta minus the IDs
              in the report's Item rows (`--walked` + `--report` on the post-run
              snapshot). A failed night writes no snapshot, so nothing is lost.
  unclassified  workable, no `night_skip`, and no `origin/night-shift/*-<ID>` branch:
              nobody has said why the night leaves it alone.

A question ticket enters ONLY as added, answered or changed (never carry, unblocked or
unclassified), so an answered ticket cannot recur. `removed` is no longer news.
candidates.json keeps the full `items` list and gains `delta[]`, `delta_why{}`,
`unchanged` (count) and `full_walk`. A post-run snapshot (`--walked`) instead gains
`carry[]`.

Exit codes: 0 NEW (delta non-empty), 3 NOTHING_NEW (delta empty), 2 error (nothing
written). The --out file is always written (atomically) on 0 and 3.

Usage:
  python tools/night_shift_preflight.py --repo <checkout> --out <candidates.json>
      [--previous <path>] [--date YYYY-MM-DD] [--full-walk]
  post-run snapshot: ... --out candidates-last.json --walked candidates-<date>.json
      --report docs/night-shift/<date>.md
"""
from __future__ import annotations

import argparse
import datetime
import json
import os
import re
import subprocess
import sys
from typing import Any, Dict, List, Optional, Tuple

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mapkit.ids import KIND_OF_PREFIX, PREFIXES, mismatch, split_ref  # noqa: E402
from mapkit.items import Item, ItemError, load_items  # noqa: E402
from mapkit.night_shift_report import ITEM_ROW_SECTIONS, cells, item_rows, parse_report  # noqa: E402

EXIT_NEW = 0
EXIT_ERROR = 2
EXIT_NOTHING_NEW = 3

_WORKABLE_INBOX = ("inbox", "backlog", "in_progress")
_PRIORITY_RANK = {"high": 0, "medium": 1, "low": 2}
_FIELDS = ("id", "stem", "kind", "status", "title", "priority", "created", "updated",
           "parent", "type", "effort", "hardware", "owner", "radius", "diagnosis", "area",
           "blocked_by", "night_skip")


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


def _filing_key(item: Item) -> Tuple[str, str, int, str]:
    """Filing order: ``created``, then the ID within its prefix, then the stem.

    A group holds one kind, so within it the per-kind number is filing order (the
    2026-11-08 renumber kept each kind's filing order). A malformed or
    kind-contradicting ID raises (preflight exits 2) instead of sorting last behind a
    sentinel, which is how per-kind IDs once tied silently. A MISSING ID is legal
    (``ids --check`` calls it informational): it sorts after every ID of its
    ``created`` day, then by stem.
    """
    stem = "%s-%s" % (item.kind, item.slug)
    raw = item.meta.get("id")
    if raw in (None, ""):
        return (str(item.meta.get("created") or ""), "~", 0, stem)
    sp = split_ref(raw)
    if sp is None:
        raise ItemError("%s: id %r is not a canonical Item ID (want %s-<n>)"
                        % (stem, raw, PREFIXES.get(item.kind, "?")))
    if mismatch(raw, item.kind):
        raise ItemError("%s: id %s names a %s, but the Item is a %s"
                        % (stem, raw, KIND_OF_PREFIX[sp[0]], item.kind))
    return (str(item.meta.get("created") or ""), sp[0], sp[1], stem)


def _sort_key(item: Item):
    return ((_group(item), _PRIORITY_RANK.get(item.meta.get("priority"), 2))
            + _filing_key(item))


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
    # `removed` (closed, dropped) is not news since closeout C4.
    return {"nothing_new": not (added or changed),
            "added": added, "removed": removed, "changed": changed}


_BRANCH_ID = re.compile(r"-([A-Z]+-[1-9]\d*)$")


def branch_ids(repo_root: str) -> set:
    """Item IDs that have an ``origin/night-shift/*-<ID>`` branch (held or in flight)."""
    try:
        r = subprocess.run(["git", "-C", repo_root, "for-each-ref", "--format=%(refname)",
                            "refs/remotes/origin/night-shift/"],
                           stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                           universal_newlines=True)
    except OSError:
        return set()
    out = set()
    for ref in (r.stdout.splitlines() if r.returncode == 0 else []):
        m = _BRANCH_ID.search(ref.strip())
        if m:
            out.add(m.group(1))
    return out


def _blocker_keys(entry: Dict[str, Any], names: Dict[str, str]) -> List[str]:
    return [names[b] for b in (entry.get("blocked_by") or []) if b in names]


def compute_delta(previous: Optional[Dict[str, Any]], record: Dict[str, Any],
                  branches: set, full_walk: bool = False) -> Dict[str, Any]:
    """The delta: ``{"delta": [key...], "delta_why": {key: why}, "unchanged": n}``.

    Pure. ``branches`` is the set of Item IDs with a night-shift branch on origin.
    Precedence and definitions: the module docstring.
    """
    cmp_ = compare(previous, record)
    added, changed, removed = set(cmp_["added"]), set(cmp_["changed"]), set(cmp_["removed"])
    carry = set((previous or {}).get("carry") or [])
    names: Dict[str, str] = {}
    for e in list((previous or {}).get("items", [])) + list(record["items"]):
        names[_key(e)] = _key(e)
        if e.get("stem"):
            names[e["stem"]] = _key(e)
    moved = changed | removed
    delta: List[str] = []
    why: Dict[str, str] = {}
    for e in record["items"]:
        k = _key(e)
        question = e.get("type") == "question"
        answered_q = question and e.get("status") != "open"
        reason = None
        if full_walk and not answered_q:
            reason = "full-walk"
        elif k in added:
            reason = "added"
        elif k in changed and answered_q:
            reason = "answered"
        elif k in changed:
            reason = "changed"
        elif question:
            reason = None
        elif any(b in moved for b in _blocker_keys(e, names)):
            reason = "unblocked"
        elif k in carry:
            reason = "carry"
        elif not e.get("night_skip") and k not in branches:
            reason = "unclassified"
        if reason:
            delta.append(k)
            why[k] = reason
    return {"delta": delta, "delta_why": why, "unchanged": len(record["items"]) - len(delta)}


def report_item_ids(text: str) -> set:
    """Item IDs in the Item rows of a night report (the sections ``parse_report`` knows)."""
    parsed = parse_report(text)
    out = set()
    for sec in ITEM_ROW_SECTIONS:
        for row in item_rows(parsed.get(sec, [])):
            toks = cells(row)[0].split()
            if toks and split_ref(toks[0]):
                out.add(toks[0])
    return out


def compute_carry(walked: Optional[Dict[str, Any]], report_text: Optional[str],
                  record: Dict[str, Any]) -> List[str]:
    """Tonight's delta minus the IDs the report has a row for, kept only while the Item is
    still a candidate and not a question ticket. No report: nothing proven walked."""
    if not walked:
        return []
    seen = report_item_ids(report_text) if report_text is not None else set()
    live = {_key(e) for e in record["items"] if e.get("type") != "question"}
    return [k for k in walked.get("delta", []) if k not in seen and k in live]


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
    ap.add_argument("--full-walk", action="store_true",
                    help="every open Item is in the delta (Sundays, full-walk.request)")
    ap.add_argument("--walked", help="post-run snapshot: tonight's candidates json")
    ap.add_argument("--report", help="post-run snapshot: tonight's report (with --walked)")
    args = ap.parse_args(argv)
    date = args.date or datetime.date.today().isoformat()
    repo = os.path.abspath(args.repo)
    try:
        record = build_record(repo, date)
    except (ItemError, OSError) as exc:
        print("ERROR %s" % exc)
        return EXIT_ERROR
    n = len(record["items"])
    if args.walked:
        text = None
        if args.report:
            try:
                with open(args.report, encoding="utf-8") as fh:
                    text = fh.read()
            except OSError:
                text = None
        record["carry"] = compute_carry(_load_previous(args.walked), text, record)
        _write_atomic(args.out, record)
        print("SNAPSHOT items=%d carry=%d" % (n, len(record["carry"])))
        return EXIT_NEW
    previous = _load_previous(args.previous)
    result = compare(previous, record)
    record.update(compute_delta(previous, record, branch_ids(repo), args.full_walk))
    record["full_walk"] = bool(args.full_walk)
    _write_atomic(args.out, record)
    if not record["delta"]:
        print("NOTHING_NEW items=%d unchanged=%d" % (n, record["unchanged"]))
        return EXIT_NOTHING_NEW
    print("NEW items=%d added=%d changed=%d removed=%d delta=%d unchanged=%d%s" % (
        n, len(result["added"]), len(result["changed"]), len(result["removed"]),
        len(record["delta"]), record["unchanged"], " full_walk=1" if args.full_walk else ""))
    return EXIT_NEW


if __name__ == "__main__":
    sys.exit(main())
