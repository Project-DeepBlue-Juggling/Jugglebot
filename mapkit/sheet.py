"""Sitting sheets: one ordered, step-by-step page for one Sitting on one rig (IDEA-19).

An Item's ``## Sitting`` section keeps its steps together and cannot merge with another's, because a
``do:`` step is free text (``docs/runbook.md`` is only an index of the waiting Items; this sheet is
the one ordering authority). A sheet is the merge: ``/sitting-sheet <rig>`` has Claude write
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
- an Item's verdict sits on one run step, whose start state includes the Item's ``setup:``;
- an action's shell is the operator's zsh and any working directory: no ``source ...setup.bash``
  (``setup.zsh`` instead) and no ``tee`` to a relative path (2026-11-13: both failed at the robot).

A step with ``pass: true`` on a served Item is where that Item's verdict is derived: one per such
Item, because Items sharing a flight can come out differently and ``/sitting-debrief`` merges a
held branch on a pass. The operator saves notes only; the verdict lands on the Item as
``## Sitting result`` when the debrief closes the sheet.

Results schema 2 (C9, 2026-11-14): the app records NOTES ONLY, ``verdicts[n][ID] = {notes,
saved_at}``, because the operator's live pass/fail was wrong, missing or contradicted by the logs on
seven Sittings. During the Sitting the results file is the only store; nothing is written into an
Item. The debrief then adds ``derived[n][ID] = {verdict, evidence, criterion, by}`` from the logs
(``derive_results``), the owner confirms once (``confirm_results``: ``confirmed = {at, changed}``),
and ``close_sheet`` writes each Item's ``## Sitting result`` exactly once, only after ``confirmed``.
A schema 1 file still loads, and the verdicts it carries are kept (they were written to the Items
live, so ``close_sheet`` refuses it).
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
SCHEMA = 1  # the SHEET file; results have their own RESULTS_SCHEMA
RESULTS_SCHEMA = 2
READABLE_RESULTS = (1, 2)
KINDS = ("setup", "run", "restore")
STATE_WORDS = ("launched", "level", "tilt", "armed", "bb-fed", "bridge-cycled")
#: One sentence per state word, shown by the app as the tooltip on a step's start-state chips.
STATE_HELP = {
    "launched": "A ros2 launch from this sheet is running in Terminal 1 and has finished Homing.",
    "level": "The Level action has run in this launch, so the Platform's normal is aligned with gravity.",
    "tilt": "The Platform is held at a commanded Tilt, a non-zero angle about x and/or y, instead of Level.",
    "armed": "Activate has run: the Legs are Closed-loop at the Active pose and streaming motion is permitted.",
    "bb-fed": "Ball Butler is up and loaded, ready to Feed.",
    "bridge-cycled": "The can-bridge has been power-cycled since it last ran a launch, so its uptime is fresh.",
}
#: What must already hold before a step may send a Juggle goal: Boot > Home > Level > Activate.
#: A goal sent earlier is no rehearsal: with the Cup empty skill_node accepts it and moves the
#: Platform and the Hand (2026-11-10). The no-motion rehearsal is the /skills/nominate service.
GOAL_COMMAND = "send_goal /jugglebot/juggle"
GOAL_NEEDS = ("launched", "level", "armed")
STEP_STATUSES = ("done", "problem", "skipped")  # skipped: not flown this Sitting (owner, 2026-11-10)
CMD_PREFIXES = ("ros2 ", "python ", "python3 ", "tools/", "colcon ", "./")

_NAME = re.compile(r"^\d{4}-\d{2}-\d{2}-(robot|bench-leg|bench-hand)(-\d+)?$")
_DATE = re.compile(r"^\d{4}-\d{2}-\d{2}$")
_TICKED = re.compile(r"`([^`]+)`")
_PLACEHOLDER = re.compile(r"<[^<>]+>")
_STEP_KEYS = ("n", "kind", "title", "from", "to", "actions", "see", "stop_if", "serves")
#: The firmware the Jetson tree expects; a sheet that names another can-bridge version is stale
#: (2026-11-10: the bridge went from 42 to 43 between writing a sheet and flying it).
FW_SOURCE = os.path.join("teensy_link", "rpc_args.py")
_FW_EXPECTED = re.compile(r"^EXPECTED_BRIDGE_FW_VERSION\s*=\s*(\d+)", re.M)
_FW_NAMED = re.compile(r"can-bridge v(\d+)")


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
    """The open Items with a valid ``## Sitting`` section on ``rig``, in Runbook order (priority, then ID; the order of a sheet is the author's)."""
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


def check_sheet(sheet: Dict[str, Any], repo_root: str, live: bool = False) -> List[str]:
    """Every error: ``validate_shape`` plus the coverage rules in the module docstring.

    ``live`` is the app's check at the robot, after the sheet was written: an Item that has been
    closed since is not an error there (2026-11-10: BUG-7 closed mid-Sitting and the sheet read
    as stale); the page shows it as closed. The author's check passes ``live=False``.
    """
    errs = validate_shape(sheet)
    closed = set()
    if live:
        closed = {(i.short_id or i.slug) for i in load_items(os.path.join(repo_root, "map"))
                  if i.status not in sitting.OPEN_STATUSES}
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
                if item not in closed:
                    errs.append("step %s: %s is not an in-scope Item" % (st.get("n"), item))
            elif item in omitted and omitted[item] is None:
                errs.append("step %s: %s is omitted whole but served here" % (st.get("n"), item))
    errs += _check_firmware(sheet, repo_root)
    if not live:  # a flown sheet is a record: the app's re-check must not banner it for authoring rules
        errs += _check_shell(steps)
    for st in steps:
        if GOAL_COMMAND in _step_text(st) and isinstance(st.get("from"), list):
            lacks = [w for w in GOAL_NEEDS if w not in st["from"]]
            if lacks:
                errs.append("step %s: sends a Juggle goal but does not start from %s (a goal moves "
                            "the robot; rehearse with /skills/nominate)" % (st.get("n"), ", ".join(lacks)))
    for item, row in sorted(rows.items()):
        skip = omitted.get(item, [])
        if item in omitted and skip is None:
            continue
        errs += _check_item(item, row, steps, list(skip or []))
    return errs


#: The operator's terminals run zsh. colcon's ``setup.bash`` finds its sibling scripts through
#: ``BASH_SOURCE``, which zsh lacks, so it looks in the working directory and prints "no such file"
#: (2026-11-13, step 24). ``setup.zsh`` is the same file for zsh.
_SOURCE_BASH = re.compile(r"\b(?:source|\.)\s+\S*setup\.bash\b")
#: ``tee [-flags] <path>``; a relative path breaks when the shell is not where the author assumed
#: (2026-11-13: ``tee -i ../temp/logs/...`` from the repo root, launches 3 and 4 lost their logs).
_TEE_PATH = re.compile(r"\btee\b(?:\s+-\w+)*\s+(\S+)")


def _check_shell(steps: List[Dict[str, Any]]) -> List[str]:
    errs: List[str] = []
    for st in steps:
        text = _step_text(st)
        for m in _SOURCE_BASH.finditer(text):
            errs.append("step %s: '%s' is bash-only and fails in the operator's zsh; source "
                        "setup.zsh instead" % (st.get("n"), m.group(0)))
        for m in _TEE_PATH.finditer(text):
            target = m.group(1).strip("\"'")
            if not target.startswith(("/", "~", "$")):
                errs.append("step %s: tee to the relative path %s depends on the working directory; "
                            "use an absolute path (~/Desktop/Jugglebot/temp/logs/...)" % (st.get("n"), target))
    return errs


def _check_firmware(sheet: Dict[str, Any], repo_root: str) -> List[str]:
    try:
        with open(os.path.join(repo_root, FW_SOURCE), encoding="utf-8") as fh:
            m = _FW_EXPECTED.search(fh.read())
    except OSError:
        return []
    if m is None:
        return []
    want = m.group(1)
    errs: List[str] = []
    for st in sheet.get("steps") or []:
        if not isinstance(st, dict):
            continue
        text = " ".join(str(st.get(k) or "") for k in ("see", "stop_if", "record")) + " " + _step_text(st)
        for got in sorted(set(_FW_NAMED.findall(text)) - {want}):
            errs.append("step %s: names can-bridge v%s, but the tree expects v%s (%s)" % (
                st.get("n"), got, want, FW_SOURCE))
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
        waived = [sv for st, sv in serving if sv.get("pass") and str(sv.get("setup_waived") or "").strip()]
        if need and not waived:
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
            if not isinstance(row.get("do"), list):
                row["do"] = []  # a serve may cite no do step (a ride-along judged from the logs)
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
    out["state_help"] = dict(STATE_HELP)
    return out


def empty_results() -> Dict[str, Any]:
    return {"schema": RESULTS_SCHEMA, "steps": {}, "verdicts": {}}


def load_results(repo_root: str, name: str) -> Dict[str, Any]:
    path = results_path(repo_root, name)
    if not os.path.exists(path):
        return empty_results()
    data = load_sheet(path)
    if data.get("schema") not in READABLE_RESULTS:
        raise SheetError("%s: results schema must be one of %s" % (path, READABLE_RESULTS))
    for key in ("steps", "verdicts"):
        if not isinstance(data.get(key), dict):
            data[key] = {}
    return data


def saved_steps(results: Dict[str, Any]) -> List[int]:
    return sorted({int(n) for n in results.get("steps", {})} | {int(n) for n in results.get("verdicts", {})})


def apply_step_result(sheet: Dict[str, Any], results: Dict[str, Any], n: int,
                      status: Optional[str], comments: str, verdicts: Dict[str, Dict[str, str]],
                      saved_at: str) -> Dict[str, Any]:
    """``results`` (as schema 2) with step ``n``'s boxes saved; raises SheetError if they do not fit.

    A step that carries a verdict takes notes per Item (``verdicts[ID] = {notes}``; a ``verdict``
    key is refused, the debrief derives it) and may also take a ``status`` (the way to mark it done
    with no notes). Any other step takes ``status``. ``comments`` is free on both. Pure: the
    caller writes the file. Entries a schema 1 file already holds, verdicts included, are kept.
    """
    if results.get("confirmed"):
        raise SheetError("these results are confirmed; the debrief has the Sitting")
    step = next((s for s in sheet.get("steps") or [] if s.get("n") == n), None)
    if step is None:
        raise SheetError("no step %s" % (n,))
    carried = {sv["item"] for sv in step.get("serves") or [] if sv.get("pass")}
    if status is not None and status not in STEP_STATUSES:
        raise SheetError("status must be one of %s" % ", ".join(STEP_STATUSES))
    if carried:
        unknown = set(verdicts) - carried
        if unknown:
            raise SheetError("step %d does not carry notes for %s" % (n, ", ".join(sorted(unknown))))
        for item, box in verdicts.items():
            if "verdict" in box:
                raise SheetError("%s: the sheet records notes only; the debrief derives the verdict" % item)
        if not verdicts and status is None and not comments.strip():
            raise SheetError("nothing to save on step %d" % n)
    else:
        if verdicts:
            raise SheetError("step %d carries no verdicts" % n)
        if status is None:
            raise SheetError("status must be one of %s" % ", ".join(STEP_STATUSES))
    out = {k: v for k, v in results.items() if k not in ("steps", "verdicts")}
    out.update({"schema": RESULTS_SCHEMA, "steps": dict(results.get("steps", {})),
                "verdicts": dict(results.get("verdicts", {}))})
    if status is not None or comments.strip():
        prior = out["steps"].get(n) or {}
        out["steps"][n] = {"status": status if status is not None else prior.get("status"),
                           "comments": " ".join(comments.split()), "saved_at": saved_at}
    if verdicts:
        merged = dict(out["verdicts"].get(n, {}))
        for item, box in verdicts.items():
            merged[item] = {"notes": " ".join(str(box.get("notes", "")).split()), "saved_at": saved_at}
        out["verdicts"][n] = merged
    return out


# --- the debrief: derive, confirm, close ----------------------------------------------------


def verdict_items(sheet: Dict[str, Any]) -> Dict[str, int]:
    """``{Item ID: n}`` of the one step per Item that carries its verdict (``pass: true``)."""
    out: Dict[str, int] = {}
    for st in sheet.get("steps") or []:
        for sv in st.get("serves") or []:
            if isinstance(sv, dict) and sv.get("pass"):
                out[str(sv["item"])] = int(st["n"])
    return out


def _int(n: Any) -> Optional[int]:
    try:
        return int(n)
    except (TypeError, ValueError):
        return None


def validate_derived(sheet: Dict[str, Any], derived: Any) -> List[str]:
    """Errors in ``derived[n][ID] = {verdict, evidence, criterion, by}`` against the sheet."""
    if not isinstance(derived, dict):
        return ["derived must map a step number to {Item ID: entry}"]
    errs: List[str] = []
    want = verdict_items(sheet)
    for n, boxes in derived.items():
        if not isinstance(boxes, dict):
            errs.append("derived[%s]: must map an Item ID to an entry" % (n,))
            continue
        for item, box in boxes.items():
            where = "derived[%s][%s]" % (n, item)
            if want.get(str(item)) != _int(n):
                errs.append("%s: %s's verdict is not on step %s" % (where, item, n))
            if not isinstance(box, dict):
                errs.append("%s: must be a mapping" % where)
                continue
            if box.get("verdict") not in sitting.VERDICTS:
                errs.append("%s: verdict must be one of %s" % (where, ", ".join(sitting.VERDICTS)))
            for key in ("evidence", "criterion", "by"):
                if not str(box.get(key) or "").strip():
                    errs.append("%s: '%s' is required" % (where, key))
    return errs


def derive_results(sheet: Dict[str, Any], results: Dict[str, Any],
                   derived: Dict[Any, Dict[str, Dict[str, str]]]) -> Dict[str, Any]:
    """``results`` with ``derived`` set (replacing any earlier one); refuses a bad shape or a
    confirmed file. The debrief shows these as one table, then calls :func:`confirm_results`."""
    if results.get("confirmed"):
        raise SheetError("these results are confirmed already")
    errs = validate_derived(sheet, derived)
    if errs:
        raise SheetError("; ".join(errs))
    out = dict(results)
    out["schema"] = RESULTS_SCHEMA
    out["derived"] = {int(n): {str(i): {k: str(b[k]) for k in ("verdict", "evidence", "criterion", "by")}
                               for i, b in boxes.items()} for n, boxes in derived.items()}
    return out


def _derived_for(results: Dict[str, Any], n: int) -> Dict[str, Any]:
    d = results.get("derived") or {}
    return d.get(n) or d.get(str(n)) or {}


def _missing_derived(sheet: Dict[str, Any], results: Dict[str, Any]) -> List[str]:
    return sorted(item for item, n in verdict_items(sheet).items()
                  if item not in _derived_for(results, n))


def confirm_results(sheet: Dict[str, Any], results: Dict[str, Any], at: str,
                    changed: Optional[List[str]] = None) -> Dict[str, Any]:
    """``results`` with ``confirmed = {at, changed}``: the owner's one confirmation of ``derived``.

    ``changed`` lists the Item IDs whose derived verdict the owner flipped (already edited in
    ``derived``). Refuses until every verdict-carrying Item has a derived entry."""
    missing = _missing_derived(sheet, results)
    if missing:
        raise SheetError("no derived verdict for %s" % ", ".join(missing))
    out = dict(results)
    out["confirmed"] = {"at": str(at), "changed": sorted(set(changed or []))}
    return out


def validate_results(sheet: Dict[str, Any], results: Dict[str, Any]) -> List[str]:
    """Shape errors in a results file, ``derived`` and ``confirmed`` included."""
    errs: List[str] = []
    if results.get("schema") not in READABLE_RESULTS:
        errs.append("schema must be one of %s" % (READABLE_RESULTS,))
    if "derived" in results:
        errs += validate_derived(sheet, results["derived"])
    conf = results.get("confirmed")
    if conf is not None and not (isinstance(conf, dict) and conf.get("at")
                                 and isinstance(conf.get("changed"), list)):
        errs.append("confirmed must be {at, changed: [Item IDs]}")
    if conf is not None and "derived" not in results:
        errs.append("confirmed without derived")
    return errs


def close_sheet(repo_root: str, name: str, now: Optional[str] = None) -> List[str]:
    """Write each verdict-carrying Item's ``## Sitting result`` from the confirmed derived
    verdicts, exactly once; returns the Item IDs written.

    Refuses (every reason at once) a schema 1 file (its verdicts went into the Items live), a file
    without ``confirmed``, a missing derived verdict, and a file already closed. The results file
    gets ``closed = {at, items}`` last, so a second call is refused; an Item that already holds
    this sheet's result for its step is skipped, so a run that died halfway can be repeated.
    """
    from mapkit.items import load_items, write_item
    sheet = load_sheet(sheet_path(repo_root, name))
    results = load_results(repo_root, name)
    errs: List[str] = []
    if results.get("schema") != RESULTS_SCHEMA:
        errs.append("schema 1 results were written into the Items live; there is nothing to close")
    if results.get("closed"):
        errs.append("already closed at %s" % (results["closed"].get("at"),))
    if not results.get("confirmed"):
        errs.append("not confirmed: the owner confirms the derived verdicts first")
    errs += validate_results(sheet, results)
    errs += ["no derived verdict for %s" % i for i in _missing_derived(sheet, results)]
    want = verdict_items(sheet)
    by_id = {(i.short_id or i.slug): i for i in load_items(os.path.join(repo_root, "map"))}
    errs += ["%s is not an Item" % i for i in sorted(want) if i not in by_id]
    if errs:
        raise SheetError("; ".join(errs))
    date = str(sheet.get("date"))
    at = str(results["confirmed"]["at"])
    m = re.match(r"^%s (\d{2}:\d{2})" % re.escape(date), at)
    time = m.group(1) if m else None
    written: List[str] = []
    for item, n in sorted(want.items(), key=lambda kv: kv[1]):
        box = _derived_for(results, n)[item]
        notes_box = (results["verdicts"].get(n) or results["verdicts"].get(str(n)) or {}).get(item) or {}
        marker = "(sheet %s, step %d)" % (name, n)
        it = by_id[item]
        if marker in it.body:
            continue
        text = ("%s %s [derived: %s]" % (str(notes_box.get("notes", "")).strip(), marker,
                                         box["evidence"])).strip()
        write_item(it.path, dict(it.meta), sitting.append_sitting_result(
            it.body, date, box["verdict"], text, time=time))
        written.append(item)
    out = dict(results)
    out["closed"] = {"at": now or at, "items": sorted(want)}
    write_results(repo_root, name, out)
    return written


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
    derive = verbs.add_parser("derive", help="set derived verdicts from a YAML file ({step: {ID: entry}})")
    derive.add_argument("name")
    derive.add_argument("file", help="YAML: step number -> Item ID -> {verdict, evidence, criterion, by}")
    derive.add_argument("--repo", default=None)
    derive.set_defaults(func=_cmd_derive)
    confirm = verbs.add_parser("confirm", help="record the owner's one confirmation of the derived verdicts")
    confirm.add_argument("name")
    confirm.add_argument("--at", required=True, help="'YYYY-MM-DD HH:MM'")
    confirm.add_argument("--changed", nargs="*", default=[], help="Item IDs the owner flipped")
    confirm.add_argument("--repo", default=None)
    confirm.set_defaults(func=_cmd_confirm)
    close = verbs.add_parser("close", help="write each Item's Sitting result once, from confirmed verdicts")
    close.add_argument("name")
    close.add_argument("--repo", default=None)
    close.set_defaults(func=_cmd_close)
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


def _results_edit(args: Any, edit: Any) -> int:
    root = _root(args)
    try:
        sheet = load_sheet(sheet_path(root, args.name))
        write_results(root, args.name, edit(sheet, load_results(root, args.name)))
    except (SheetError, OSError, yaml.YAMLError) as exc:
        print("sheet: %s" % (exc,))
        return 1
    return 0


def _cmd_derive(args: Any) -> int:
    try:
        with open(args.file, encoding="utf-8") as fh:
            derived = yaml.safe_load(fh)
    except (OSError, yaml.YAMLError) as exc:
        print("sheet: %s" % (exc,))
        return 1
    return _results_edit(args, lambda sh, res: derive_results(sh, res, derived))


def _cmd_confirm(args: Any) -> int:
    return _results_edit(args, lambda sh, res: confirm_results(sh, res, args.at, args.changed))


def _cmd_close(args: Any) -> int:
    try:
        written = close_sheet(_root(args), args.name)
    except SheetError as exc:
        print("sheet close: %s" % (exc,))
        return 1
    print("sheet close: wrote %d Sitting result(s): %s" % (len(written), ", ".join(written) or "-"))
    return 0
