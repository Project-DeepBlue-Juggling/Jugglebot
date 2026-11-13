#!/usr/bin/env python3
"""The report a Night Shift writes for itself when the orchestrator could not.

`tools/night_shift.sh` calls this when a real run that reached Claude ends in any
status but OK with no `docs/night-shift/<date>.md`. It writes the eight-section
layout of `docs/night-shift/README.md`, so the morning finds a page in the usual
place that says the night failed and where its work went, and the map app and
`mapkit.night_shift_report` read it like any other report.

It claims nothing it does not know. Tonight's Items were never tallied, so the
Item sections say `none recorded`, not `none`. The two facts it does state come
from git and the board: what reached `main`, and which question tickets are open.
"""
from __future__ import annotations

import argparse
import glob
import os
import subprocess
import sys
from typing import List

_NOT_RECORDED = "none recorded (the orchestrator stopped before reporting)"
#: Ticket statuses that no longer want an answer.
_CLOSED = ("complete", "wontfix", "archived", "cancelled")


def main_commits(clone: str, head_before: str) -> List[str]:
    """`<short sha> <subject>` for each commit tonight put on origin/main."""
    if not head_before:
        return []
    done = subprocess.run(
        ["git", "-C", clone, "log", "--format=%h %s", head_before + "..origin/main"],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, universal_newlines=True)
    return [line for line in done.stdout.splitlines() if line.strip()]


def open_questions(clone: str) -> List[str]:
    """One GFM row per open question ticket on the board, for the Questions section."""
    sys.path.insert(0, clone)
    try:
        from mapkit.items import load_item
    except ImportError:
        return []
    rows: List[str] = []
    for path in sorted(glob.glob(os.path.join(clone, "map", "ticket-*.md"))):
        try:
            meta = load_item(path).meta
        except (OSError, ValueError):
            continue
        if meta.get("type") != "question" or meta.get("status") in _CLOSED:
            continue
        title = str(meta.get("title", "")).replace("|", "/")
        rows.append("| %s | %s | %s | |" % (meta.get("id"), title, meta.get("parent") or ""))
    return rows


def _one_line(text: str) -> str:
    """A runner string flattened: stderr tails are multi-line, and a line starting
    ``## `` would become a real section and break the layout."""
    return " ".join(text.split()).replace("|", "/")


def render(args: argparse.Namespace) -> str:
    commits = main_commits(args.clone, args.head_before)
    questions = open_questions(args.clone)
    summary = ("**Summary.** The night did not finish: status %s. The runner wrote this "
               "page because the orchestrator wrote no report; %d commit(s) reached main."
               % (args.status, len(commits)))
    out = ["# Night Shift %s" % args.date, "", summary, "", "## Nightly",
           _one_line(args.nightly) or "not recorded", ""]
    out += ["## Merged to main", _NOT_RECORDED, "", "## Held", _NOT_RECORDED, ""]
    out += ["## Questions for the Operator"]
    if questions:
        out += ["| Ticket | Question (answer in the app under its `## Answer`) | Blocks | Note |",
                "|---|---|---|---|"] + questions
    else:
        out += ["none"]
    out += ["", "## Unfinished"]
    if args.salvaged:
        branch = args.salvaged.split()[0]
        out += ["%s | Saved by the runner (%s): not reviewed and not gated, so a Session "
                "must check it before any merge" % (branch, args.salvaged)]
    else:
        out += ["none saved by the runner"]
    out += ["runner detail | %s" % _one_line(args.detail)]
    out += ["main | %s" % commit.replace("|", "/") for commit in commits]
    out += ["", "## Skipped", _NOT_RECORDED, "", "## Unaddressed Sitting results",
            _NOT_RECORDED, "", "## Cost", "units: not recorded",
            "wall clock: %s to %s" % (args.started, args.finished)]
    out += [_one_line(line) for line in (args.deploy_line, args.cost_line) if line]
    return "\n".join(out) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    for flag in ("date", "status", "clone", "out", "started", "finished"):
        parser.add_argument("--" + flag, required=True)
    for flag in ("detail", "nightly", "head-before", "salvaged", "deploy-line", "cost-line"):
        parser.add_argument("--" + flag, default="")
    args = parser.parse_args()
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, "w") as handle:
        handle.write(render(args))
    return 0


if __name__ == "__main__":
    sys.exit(main())
