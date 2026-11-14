"""``python -m mapkit night inputs``: the Night Shift report's generated input section.

The body of ``## Items the Night Shift could resolve with a small amount of input from
you`` is printed from the board, never written by the model: every open Item whose
``night_skip`` starts with ``input-`` is one row. See ``docs/night-shift/README.md``.
"""
from __future__ import annotations

import argparse
import os
import sys
from typing import Dict, List, Tuple

from mapkit.ids import split_ref
from mapkit.items import ItemError, load_items
from mapkit.night_shift_report import INPUTS_HEADER

#: Statuses that no longer want work (the inbox kinds; a ticket's are below).
_CLOSED = ("complete", "dropped", "promoted")
_KIND_ORDER = {"BUG": 0, "FEAT": 1, "IDEA": 2, "PWAY": 3, "TKT": 4}


def _cell(text: object) -> str:
    """One table cell: single line, and a pipe becomes ``/`` (the report checker splits rows on
    every ``|``, so an escaped pipe would still break the row)."""
    return " ".join(str(text if text is not None else "").split()).replace("|", "/")


def _sort_key(entry: Tuple[str, int]) -> Tuple[int, int]:
    return (_KIND_ORDER.get(entry[0], 9), entry[1])


def rows(map_dir: str) -> List[List[str]]:
    """One six-cell row per open input-* Item, ordered by ID kind then number."""
    items = load_items(map_dir)
    open_questions: Dict[str, List[Tuple[int, str]]] = {}
    for t in items:
        if t.kind != "ticket" or t.meta.get("type") != "question" or t.status != "open":
            continue
        ref = split_ref(t.meta.get("id"))
        if ref:
            open_questions.setdefault(str(t.meta.get("parent")), []).append((ref[1], str(t.meta["id"])))
    out: List[Tuple[Tuple[int, int], List[str]]] = []
    for it in items:
        reason = str(it.meta.get("night_skip") or "")
        if not reason.startswith("input-") or it.kind in ("pathway", "ticket") or it.status in _CLOSED:
            continue
        ref = split_ref(it.meta.get("id"))
        if not ref:
            continue
        kind = reason[len("input-"):]
        stem = "%s-%s" % (it.kind, it.slug)
        asked = sorted(open_questions.get(stem, []))
        respond = asked[0][1] if asked else ("sheet" if kind == "sitting" else "none")
        out.append((_sort_key(ref), [str(it.meta["id"]), it.meta.get("title"), kind,
                                      it.meta.get("night_skip_input"), it.meta.get("night_skip_note"),
                                      respond]))
    out.sort(key=lambda e: e[0])
    return [r for _, r in out]


def render(map_dir: str) -> str:
    rs = rows(map_dir)
    if not rs:
        return "none"
    lines = ["| " + " | ".join(INPUTS_HEADER) + " |", "|" + "---|" * len(INPUTS_HEADER)]
    lines += ["| " + " | ".join(_cell(c) for c in r) + " |" for r in rs]
    return "\n".join(lines)


def _run(args: argparse.Namespace) -> int:
    map_dir = args.map or os.path.join(os.getcwd(), "map")
    try:
        print(render(map_dir))
    except (ItemError, OSError) as exc:
        print("mapkit: %s" % exc, file=sys.stderr)
        return 2
    return 0


def register(subparsers) -> None:
    night = subparsers.add_parser("night", help="Night Shift report helpers")
    nsub = night.add_subparsers(dest="night_command")
    p = nsub.add_parser("inputs", help="print the report's generated input section body")
    p.add_argument("--map", default=None, help="map directory (default ./map)")
    p.set_defaults(func=_run)
