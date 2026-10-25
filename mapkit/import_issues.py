"""``mapkit import-issues``: wayfinder map sub-issues -> ``map/ticket-*.md``.

The network is behind one injectable ``fetch(cmd_args) -> parsed json``;
``cmd_args`` is the argument list after the ``gh`` binary.
"""
from __future__ import annotations

import datetime
import json
import os
import re
import shutil
import subprocess
import sys
import unicodedata
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

from mapkit.items import write_item

REPO_SLUG = "Project-DeepBlue-Juggling/Jugglebot"
MAX_SLUG = 60


def slugify(title: str) -> str:
    s = unicodedata.normalize("NFKD", title)
    s = s.encode("ascii", "ignore").decode("ascii").lower()
    s = re.sub(r"[^a-z0-9]+", "-", s).strip("-")
    if len(s) > MAX_SLUG:
        # cut at a word boundary, never mid-word (filenames are permanent)
        cut = s[:MAX_SLUG + 1].rfind("-")
        s = s[:cut if cut > 0 else MAX_SLUG]
    s = s.strip("-")
    return s or "ticket"


def find_gh(explicit: Optional[str] = None) -> str:
    for cand in (explicit, os.environ.get("GH_BIN"),
                 os.path.expanduser("~/bin/gh")):
        if cand and os.path.isfile(cand):
            return cand
    found = shutil.which("gh")
    if not found:
        raise RuntimeError("gh binary not found (--gh, $GH_BIN, ~/bin/gh, PATH)")
    return found


def make_fetch(gh: str) -> Callable[[List[str]], Any]:
    def fetch(cmd_args: List[str]) -> Any:
        out = subprocess.run([gh] + list(cmd_args), check=True,
                             stdout=subprocess.PIPE,
                             universal_newlines=True).stdout
        return json.loads(out) if out.strip() else []
    return fetch


def _label_type(labels: Any) -> str:
    for lab in labels or []:
        name = lab.get("name") if isinstance(lab, dict) else str(lab)
        if name and name.startswith("wayfinder:"):
            t = name.split(":", 1)[1]
            if t in ("research", "prototype", "grilling", "task"):
                return t
    return "task"


def import_issues(repo_root: str, map_number: int, pathway: str,
                  fetch: Callable[[List[str]], Any],
                  now: Optional[datetime.datetime] = None) -> List[Path]:
    map_dir = Path(repo_root) / "map"
    if not (map_dir / ("pathway-%s.md" % pathway)).is_file():
        raise SystemExit("mapkit: map/pathway-%s.md does not exist" % pathway)
    now = now or datetime.datetime.utcnow()
    stamp = now.replace(microsecond=0).isoformat() + "Z"

    subs = fetch(["api", "repos/%s/issues/%d/sub_issues" % (REPO_SLUG, map_number)])
    numbers = sorted(int(s["number"]) for s in subs)
    issues: Dict[int, Any] = {}
    blockers: Dict[int, List[int]] = {}
    for k in numbers:
        issues[k] = fetch(["issue", "view", str(k), "--json",
                           "number,title,body,state,labels,comments,url,createdAt"])
        deps = fetch(["api", "repos/%s/issues/%d/dependencies/blocked_by"
                      % (REPO_SLUG, k)])
        blockers[k] = [int(d["number"]) for d in deps]

    slugs: Dict[int, str] = {}
    used = set()
    for k in numbers:
        base = slugify(issues[k]["title"])
        slug, n = base, 1
        while slug in used:
            n += 1
            slug = "%s-%d" % (base, n)
        used.add(slug)
        slugs[k] = slug

    written: List[Path] = []
    for rank, k in enumerate(numbers, start=1):
        slug = slugs[k]
        path = map_dir / ("ticket-%s.md" % slug)
        if path.exists():
            print("mapkit: skip %s (exists)" % path.name, file=sys.stderr)
            continue
        iss = issues[k]
        inside = [b for b in blockers[k] if b in slugs]
        outside = [b for b in blockers[k] if b not in slugs]
        links = [iss["url"]] + [
            "https://github.com/%s/issues/%d" % (REPO_SLUG, b) for b in outside]
        from mapkit import ids  # local: ids imports items, which this module's readers avoid
        meta = {
            "id": ids.allocate(repo_root),  # written files count, so each import gets the next
            "kind": "ticket",
            "title": iss["title"],
            "type": _label_type(iss.get("labels")),
            "parent": pathway,
            "blocked_by": ["ticket-%s" % slugs[b] for b in inside],
            "owners": [],
            "rank": rank,
            "status": "complete" if str(iss["state"]).upper() == "CLOSED" else "open",
            "links": links,
            "created": (iss.get("createdAt") or "")[:10],
            "updated": stamp,
            "priority": "medium",
        }
        body = (iss.get("body") or "").strip("\n") + "\n"
        comments = iss.get("comments") or []
        if comments:
            last = comments[-1]
            text = last.get("body", "") if isinstance(last, dict) else str(last)
            body += "\n## Answer\n\n" + text.strip("\n") + "\n"
        write_item(str(path), meta, body)
        written.append(path)
    return written


def _run(args) -> int:
    fetch = make_fetch(find_gh(args.gh))
    written = import_issues(os.path.abspath(args.repo), args.map_number,
                            args.pathway, fetch)
    for p in written:
        print(p)
    return 0


def register(subparsers) -> None:
    p = subparsers.add_parser("import-issues",
                              help="import a wayfinder map's sub-issues as tickets")
    p.add_argument("map_number", type=int)
    p.add_argument("--pathway", required=True)
    p.add_argument("--repo", default=".")
    p.add_argument("--gh", default=None)
    p.set_defaults(func=_run)
