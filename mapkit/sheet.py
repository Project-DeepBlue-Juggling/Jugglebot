"""Sitting sheets: one ordered, step-by-step page for one Sitting on one rig (IDEA-19).

``docs/runbook.md`` keeps each Item's steps together and cannot merge them, because a
``do:`` step is free text. A sheet is the merge: ``/sitting-sheet <rig>`` has Claude write
``map/sheets/<date>-<rig>[-N].yaml`` from the open Items' ``## Sitting`` sections, and the map
app's Next Sitting page steps through it. The app saves into a sibling
``<name>.results.yaml``, never into the sheet, so a sheet is immutable once written.

An ordered merge by a model can drop or paraphrase a step, so ``check_sheet`` holds it to what
the Items say. Everything a tool can verify is verified here; what it cannot (whether a step
needs something a free-text ``do:`` never states) is the command's read-through.

- every ``do:`` step of every in-scope Item is on the sheet or ``omitted`` with a reason;
- every command an Item's ``do:`` quotes appears on the sheet, placeholders aside, so a
  ``ros2 param set`` cannot be retyped wrongly;
- an Item's ``do:`` steps keep their order;
- each step starts in the state the previous step left, and the sheet does not end Armed;
- an Item's verdict sits on one run step, whose start state includes the Item's ``setup:``.

A step with ``pass: true`` on a served Item takes one verdict per such Item, because Items
sharing a flight can come out differently and ``/sitting-debrief`` merges a held branch on a
pass. The verdict lands on the Item as today's ``## Sitting result``.
"""
from __future__ import annotations

import os
import re
import tempfile
from typing import Any, Dict, List, Optional

import yaml

from mapkit import sitting
from mapkit.items import load_items

SHEET_DIR = os.path.join("map", "sheets")
SCHEMA = 1
KINDS = ("setup", "run", "restore")
STATE_WORDS = ("launched", "level", "tilt", "armed", "bb-fed", "bridge-cycled")
STEP_STATUSES = ("done", "problem")
CMD_PREFIXES = ("ros2 ", "python ", "python3 ", "tools/", "colcon ", "./")

_NAME = re.compile(r"^\d{4}-\d{2}-\d{2}-(robot|bench-leg|bench-hand)(-\d+)?$")
_DATE = re.compile(r"^\d{4}-\d{2}-\d{2}$")
_TICKED = re.compile(r"`([^`]+)`")
_PLACEHOLDER = re.compile(r"<[^<>]+>")
_STEP_KEYS = ("n", "kind", "title", "from", "to", "actions", "see", "stop_if", "serves")


class SheetError(Exception):
    """A sheet or results file that cannot be read, or a result that does not fit its sheet."""


def valid_name(name: str) -> bool:
    return bool(_NAME.match(name))


def sheet_path(repo_root: str, name: str) -> str:
    if not valid_name(name):
        raise SheetError("bad sheet name %r" % (name,))
    return os.path.join(repo_root, SHEET_DIR, name + ".yaml")


def results_path(repo_root: str, name: str) -> str:
    if not valid_name(name):
        raise SheetError("bad sheet name %r" % (name,))
    return os.path.join(repo_root, SHEET_DIR, name + ".results.yaml")


def load_sheet(path: str) -> Dict[str, Any]:
    try:
        with open(path, encoding="utf-8") as fh:
            data = yaml.safe_load(fh)
    except (OSError, yaml.YAMLError) as exc:
        raise SheetError("%s: %s" % (path, exc))
    if not isinstance(data, dict):
        raise SheetError("%s: not a mapping" % (path,))
    return data


def list_sheets(repo_root: str) -> List[Dict[str, Any]]:
    """Every sheet, newest first: name, date, rig, step count, steps saved so far."""
    root = os.path.join(repo_root, SHEET_DIR)
    if not os.path.isdir(root):
        return []
    out: List[Dict[str, Any]] = []
    for fn in os.listdir(root):
        stem = fn[:-len(".yaml")] if fn.endswith(".yaml") else ""
        if not valid_name(stem):
            continue
        try:
            sheet = load_sheet(os.path.join(root, fn))
            res = load_results(repo_root, stem)
        except (SheetError, TypeError, ValueError, AttributeError):
            continue  # one bad file must not take the whole page down
        out.append({"name": stem, "date": str(sheet.get("date", "")), "rig": str(sheet.get("rig", "")),
                    "steps": len(sheet.get("steps") or []), "saved": len(saved_steps(res))})
    # A later ``-N`` of a date and rig is the newer sheet, so the suffix sorts numerically.
    def key(row: Dict[str, Any]) -> Any:
        m = re.search(r"-(\d+)$", row["name"])
        return (row["date"], int(m.group(1)) if m else 1, row["name"])
    return sorted(out, key=key, reverse=True)


# --- the shape of a sheet ------------------------------------------------------------------


def validate_shape(sheet: Dict[str, Any]) -> List[str]:
    """Errors in the sheet's own structure and state chain; needs no Items."""
    errs: List[str] = []
    if sheet.get("schema") != SCHEMA:
        errs.append("schema must be %d" % SCHEMA)
    if not _DATE.match(str(sheet.get("date", ""))):
        errs.append("date must be YYYY-MM-DD (quote it)")
    if sheet.get("rig") not in sitting.HARDWARE:
        errs.append("rig must be one of %s" % ", ".join(sitting.HARDWARE))
    steps = sheet.get("steps")
    if not isinstance(steps, list) or not steps:
        return errs + ["steps must be a non-empty list"]
    for pos, st in enumerate(steps, 1):
        if not isinstance(st, dict):
            errs.append("step %d: not a mapping" % pos)
            continue
        where = "step %s" % st.get("n", pos)
        for key in _STEP_KEYS:
            if key not in st:
                errs.append("%s: missing '%s'" % (where, key))
        if st.get("n") != pos:
            errs.append("%s: n must be %d (numbered from 1 with no gaps)" % (where, pos))
        if st.get("kind") not in KINDS:
            errs.append("%s: kind must be one of %s" % (where, ", ".join(KINDS)))
        for key in ("title", "see", "stop_if"):
            if key in st and not str(st.get(key) or "").strip():
                errs.append("%s: '%s' is empty" % (where, key))
        for key in ("from", "to"):
            if key in st:
                val = st.get(key) or []
                if not isinstance(val, list) or [w for w in val if w not in STATE_WORDS]:
                    errs.append("%s: '%s' must list state words from %s" % (
                        where, key, ", ".join(STATE_WORDS)))
        acts = st.get("actions")
        if "actions" in st and (not isinstance(acts, list) or not acts):
            errs.append("%s: actions must be a non-empty list" % where)
        for act in acts if isinstance(acts, list) else []:
            if not isinstance(act, dict) or not str(act.get("where") or "").strip() \
                    or not (act.get("text") or act.get("command")):
                errs.append("%s: each action needs 'where' and 'text' or 'command'" % where)
        if st.get("new_launch") and "ros2 launch" not in _step_text(st):
            errs.append("%s: new_launch needs its 'ros2 launch' command" % where)
        errs += _shape_serves(st, where)
    errs += _shape_chain(steps)
    if isinstance(steps[0], dict) and steps[0].get("from"):
        errs.append("step 1: must start from nothing (from: [])")
    for om in sheet.get("omitted") or []:
        if not isinstance(om, dict) or not om.get("item") or not str(om.get("reason") or "").strip():
            errs.append("omitted: each entry needs 'item' and a 'reason'")
    return errs


def _shape_serves(st: Dict[str, Any], where: str) -> List[str]:
    errs: List[str] = []
    serves = st.get("serves")
    if not isinstance(serves, list):
        return ["%s: serves must be a list" % where] if "serves" in st else []
    if st.get("kind") == "run" and not serves:
        errs.append("%s: a run step must serve at least one Item" % where)
    for sv in serves:
        if not isinstance(sv, dict) or not sv.get("item"):
            errs.append("%s: each serves entry needs 'item'" % where)
            continue
        do = sv.get("do", [])
        if not isinstance(do, list) or not all(isinstance(i, int) and not isinstance(i, bool) for i in do):
            errs.append("%s: %s: do must list step indices" % (where, sv.get("item")))
        if sv.get("pass") and st.get("kind") != "run":
            errs.append("%s: %s: only a run step can carry a verdict" % (where, sv.get("item")))
    return errs


def _shape_chain(steps: List[Any]) -> List[str]:
    errs: List[str] = []
    prev: Optional[Dict[str, Any]] = None
    for st in steps:
        if not isinstance(st, dict):
            prev = None
            continue
        if prev is not None and set(prev.get("to") or []) != set(st.get("from") or []):
            errs.append("step %s: starts from [%s] but step %s left [%s]" % (
                st.get("n"), ", ".join(st.get("from") or []), prev.get("n"),
                ", ".join(prev.get("to") or [])))
        prev = st
    last = steps[-1]
    if isinstance(last, dict) and "armed" in (last.get("to") or []):
        errs.append("step %s: the sheet must not end armed" % last.get("n"))
    return errs


def _step_text(st: Dict[str, Any]) -> str:
    parts: List[str] = []
    for act in st.get("actions") or []:
        if isinstance(act, dict):
            parts += [str(act.get("command") or ""), str(act.get("text") or "")]
    return " ".join(" ".join(parts).split())


# --- the sheet against the Items -----------------------------------------------------------


def scope_rows(repo_root: str, rig: str) -> List[sitting.RunbookRow]:
    """The open Items with a valid ``## Sitting`` section on ``rig``, in Runbook order."""
    return [r for r in sitting.collect(repo_root)["rows"] if r.hardware == rig]


def _commands(do_text: str) -> List[str]:
    # A bare "ros2 param get" names a kind of command, not a command to copy.
    return [t.strip() for t in _TICKED.findall(do_text)
            if t.strip().startswith(CMD_PREFIXES)
            and not (t.strip().startswith("ros2 ") and len(t.split()) < 4)]


def _pattern(command: str) -> "re.Pattern[str]":
    text = " ".join(command.split())
    parts = _PLACEHOLDER.split(text)
    return re.compile(".+?".join(re.escape(p) for p in parts))


def check_sheet(sheet: Dict[str, Any], repo_root: str) -> List[str]:
    """Every error: ``validate_shape`` plus the coverage rules in the module docstring."""
    errs = validate_shape(sheet)
    steps = [s for s in (sheet.get("steps") or []) if isinstance(s, dict)]
    rows = {r.item_id: r for r in scope_rows(repo_root, str(sheet.get("rig")))}
    omitted: Dict[str, Optional[List[int]]] = {}
    for om in sheet.get("omitted") or []:
        if isinstance(om, dict) and om.get("item"):
            if om["item"] not in rows:
                errs.append("omitted: %s is not an in-scope Item" % om["item"])
            omitted[str(om["item"])] = om.get("do")  # None = the whole Item
    for st in steps:
        for sv in st.get("serves") or []:
            if not isinstance(sv, dict):
                continue
            item = str(sv.get("item"))
            if item not in rows:
                errs.append("step %s: %s is not an in-scope Item" % (st.get("n"), item))
            elif item in omitted and omitted[item] is None:
                errs.append("step %s: %s is omitted whole but served here" % (st.get("n"), item))
    for item, row in sorted(rows.items()):
        skip = omitted.get(item, [])
        if item in omitted and skip is None:
            continue
        errs += _check_item(item, row, steps, list(skip or []))
    return errs


def _check_item(item: str, row: sitting.RunbookRow, steps: List[Dict[str, Any]],
                skip: List[int]) -> List[str]:
    errs: List[str] = []
    serving = [(st, sv) for st in steps for sv in (st.get("serves") or [])
               if isinstance(sv, dict) and sv.get("item") == item]
    covered = sorted({i for _, sv in serving for i in (sv.get("do") or [])})
    for i in range(len(row.spec.do)):
        if i not in covered and i not in skip:
            errs.append("%s: do step %d is on no step and not omitted: %s" % (
                item, i, row.spec.do[i][:80]))
    for i in covered:
        if not 0 <= i < len(row.spec.do):
            errs.append("%s: do index %d does not exist (the Item has %d)" % (item, i, len(row.spec.do)))
    text = " ".join(_step_text(st) for st, _ in serving)
    for i, step in enumerate(row.spec.do):
        if i in skip:
            continue
        for cmd in _commands(step):
            if not _pattern(cmd).search(text):
                errs.append("%s: do step %d's command is not on the sheet verbatim: %s" % (item, i, cmd))
    high = -1
    for st, sv in serving:
        do = [i for i in (sv.get("do") or []) if isinstance(i, int)]
        # A ``do:`` list can state a precondition after the steps it governs ("before each
        # angle's first goal"); ``reorder: <why>`` on the serves entry says so on the sheet.
        if do and min(do) < high and not str(sv.get("reorder") or "").strip():
            errs.append("%s: do steps out of order at step %s" % (item, st.get("n")))
        high = max([high] + do)
    verdicts = [st for st, sv in serving if sv.get("pass")]
    if len(verdicts) != 1:
        errs.append("%s: needs exactly one step with pass: true, has %d" % (item, len(verdicts)))
    else:
        need = set(row.spec.setup) - set(verdicts[0].get("from") or [])
        if need:
            errs.append("%s: its verdict step %s does not start from its setup (missing %s)" % (
                item, verdicts[0].get("n"), ", ".join(sorted(need))))
    return errs


# --- what the app shows and saves ----------------------------------------------------------


def enrich(sheet: Dict[str, Any], repo_root: str) -> Dict[str, Any]:
    """A copy of ``sheet`` with each served Item's live fields and each step's ``optional``.

    ``optional`` is computed here, never written by the sheet's author: a step is optional when
    every Item it serves is priority low (TKT-53), so a shared step stays required if any of its
    Items is not. ``pass_text`` comes from the Item, so a verdict is judged on the Item's own
    criterion.
    """
    items = {(i.short_id or i.slug): i for i in load_items(os.path.join(repo_root, "map"))}
    out = dict(sheet)
    out_steps: List[Dict[str, Any]] = []
    for st in sheet.get("steps") or []:
        copy = dict(st)
        served: List[Dict[str, Any]] = []
        for sv in st.get("serves") or []:
            item = items.get(sv.get("item"))
            row = dict(sv)
            row["verdict"] = bool(row.pop("pass", False))
            row.update({"title": "", "priority": "", "is_open": False, "stem": None, "kind": None,
                        "pass_text": None})
            if item is not None:
                row.update({"title": item.title, "priority": str(item.meta.get("priority") or ""),
                            "is_open": item.status in sitting.OPEN_STATUSES,
                            "stem": "%s-%s" % (item.kind, item.slug), "kind": item.kind})
                spec, _ = sitting.parse_sitting(item.body)
                if spec is not None and row["verdict"]:
                    row["pass_text"] = spec.pass_
            served.append(row)
        copy["serves"] = served
        copy["from_"] = copy.pop("from", [])
        copy["optional"] = bool(served) and all(s["priority"] == "low" for s in served)
        out_steps.append(copy)
    out["steps"] = out_steps
    return out


def empty_results() -> Dict[str, Any]:
    return {"schema": SCHEMA, "steps": {}, "verdicts": {}}


def load_results(repo_root: str, name: str) -> Dict[str, Any]:
    path = results_path(repo_root, name)
    if not os.path.exists(path):
        return empty_results()
    data = load_sheet(path)
    for key in ("steps", "verdicts"):
        if not isinstance(data.get(key), dict):
            data[key] = {}
    return data


def saved_steps(results: Dict[str, Any]) -> List[int]:
    return sorted({int(n) for n in results.get("steps", {})} | {int(n) for n in results.get("verdicts", {})})


def apply_step_result(sheet: Dict[str, Any], results: Dict[str, Any], n: int,
                      status: Optional[str], comments: str, verdicts: Dict[str, Dict[str, str]],
                      saved_at: str) -> Dict[str, Any]:
    """``results`` with step ``n``'s boxes saved; raises SheetError if they do not fit the step.

    A step that carries verdicts takes those, one per Item it carries; any other step takes
    ``status``. ``comments`` is free on both. The Item's own ``## Sitting result`` is the
    caller's write, so this stays pure.
    """
    step = next((s for s in sheet.get("steps") or [] if s.get("n") == n), None)
    if step is None:
        raise SheetError("no step %s" % (n,))
    carried = {sv["item"] for sv in step.get("serves") or [] if sv.get("pass")}
    if carried:
        unknown = set(verdicts) - carried
        if unknown:
            raise SheetError("step %d does not carry a verdict for %s" % (n, ", ".join(sorted(unknown))))
        for item, box in verdicts.items():
            if box.get("verdict") not in sitting.VERDICTS:
                raise SheetError("%s: verdict must be one of %s" % (item, ", ".join(sitting.VERDICTS)))
        if status is not None:
            raise SheetError("step %d takes a verdict per Item, not a status" % n)
        if not verdicts and not comments.strip():
            raise SheetError("nothing to save on step %d" % n)
    else:
        if verdicts:
            raise SheetError("step %d carries no verdicts" % n)
        if status not in STEP_STATUSES:
            raise SheetError("status must be one of %s" % ", ".join(STEP_STATUSES))
    out = {"schema": SCHEMA, "steps": dict(results.get("steps", {})),
           "verdicts": dict(results.get("verdicts", {}))}
    if status is not None or comments.strip():
        out["steps"][n] = {"status": status, "comments": " ".join(comments.split()), "saved_at": saved_at}
    if verdicts:
        merged = dict(out["verdicts"].get(n, {}))
        for item, box in verdicts.items():
            merged[item] = {"verdict": box["verdict"], "notes": " ".join(str(box.get("notes", "")).split()),
                            "saved_at": saved_at}
        out["verdicts"][n] = merged
    return out


def write_results(repo_root: str, name: str, results: Dict[str, Any]) -> str:
    """Atomically write ``results``; returns the path, for the app's git writer."""
    path = results_path(repo_root, name)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=os.path.dirname(path), suffix=".tmp")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as fh:
            yaml.safe_dump(results, fh, sort_keys=True, allow_unicode=True)
        os.replace(tmp, path)
    except BaseException:
        if os.path.exists(tmp):
            os.unlink(tmp)
        raise
    return path


def next_name(repo_root: str, date: str, rig: str) -> str:
    """A free sheet name: a date and rig that already have a sheet get ``-2``, ``-3``, ..."""
    base = "%s-%s" % (date, rig)
    if not os.path.exists(sheet_path(repo_root, base)):
        return base
    n = 2
    while os.path.exists(sheet_path(repo_root, "%s-%d" % (base, n))):
        n += 1
    return "%s-%d" % (base, n)


# --- command line --------------------------------------------------------------------------


def register(sub: Any) -> None:
    parser = sub.add_parser("sheet", help="Sitting sheets: the Items in scope, a sheet's check")
    verbs = parser.add_subparsers(dest="verb", required=True)
    scope = verbs.add_parser("scope", help="print one rig's in-scope Items as YAML, do steps numbered")
    scope.add_argument("rig", choices=sitting.HARDWARE)
    scope.add_argument("--repo", default=None)
    scope.set_defaults(func=_cmd_scope)
    check = verbs.add_parser("check", help="check a sheet against the Items; exit 1 on any error")
    check.add_argument("path")
    check.add_argument("--repo", default=None)
    check.set_defaults(func=_cmd_check)
    name = verbs.add_parser("next-name", help="print the free sheet name for a date and rig")
    name.add_argument("date")
    name.add_argument("rig", choices=sitting.HARDWARE)
    name.add_argument("--repo", default=None)
    name.set_defaults(func=_cmd_next_name)


def _root(args: Any) -> str:
    from mapkit.__main__ import default_repo
    return os.path.abspath(args.repo or default_repo())


def _cmd_scope(args: Any) -> int:
    rows = [{"item": r.item_id, "title": r.title, "priority": r.priority,
             "setup": list(r.spec.setup), "why": r.spec.why,
             "do": {i: s for i, s in enumerate(r.spec.do)}, "pass": r.spec.pass_,
             "record": r.spec.record, "minutes": r.spec.minutes, "branch": r.spec.branch,
             "first": r.spec.first} for r in scope_rows(_root(args), args.rig)]
    print(yaml.safe_dump(rows, sort_keys=False, allow_unicode=True, width=100000))
    return 0


def _cmd_check(args: Any) -> int:
    try:
        errs = check_sheet(load_sheet(args.path), _root(args))
    except (SheetError, TypeError, ValueError, AttributeError) as exc:
        errs = ["malformed sheet: %s" % (exc,)]
    for e in errs:
        print(e)
    print("sheet check: %s" % ("OK" if not errs else "%d error(s)" % len(errs)))
    return 1 if errs else 0


def _cmd_next_name(args: Any) -> int:
    print(next_name(_root(args), args.date, args.rig))
    return 0
