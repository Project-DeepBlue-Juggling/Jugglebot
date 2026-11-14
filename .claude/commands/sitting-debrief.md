---
description: Debrief the Sitting(s) flown since the last debrief: first flush the Sitting sheet's results from the map app, then ANALYSE the new launch logs and rosbags against the open Items and plans (inventory, derived verdicts the owner confirms once, per-killer diagnosis, crosswalk, write-backs), then close the paperwork - write each Item's ## Sitting result from the confirmed verdicts, merge the passed Items' held branches into main one by one, return the live checkout to main, rebuild if ros_ws changed, commit docs/records.md, file follow-ups from the notes, write the logbook entry, regenerate the Runbook index. Invoke with /sitting-debrief [<date>] [--dry-run]. Runs in ~/Desktop/Jugglebot under the single-writer rule.
---

# Sitting debrief

(Formerly `/sitting-wrapup`; that command's paperwork is sections 2 to 6 below, unchanged.)

The morning or post-Sitting Session's one prompt. The owner's share of the Sitting was the
hardware steps and the result boxes in the map app; everything that is git, tests and
paperwork is this command. The owner's share of a flown sheet is notes only (done, problem or
skipped, plus a few words per step); the verdicts are this command's: derived from the logs,
confirmed by the owner once. The Next Sitting Runbook (`docs/runbook.md`) is only an index of the
waiting Items and holds no steps and no close-out; the step-by-step page is the Sitting sheet
(`map/sheets/<name>.yaml`, results beside it in `<name>.results.yaml`).

Arguments: `$ARGUMENTS` (`<date>` = the Sitting's date, the one in the `## Sitting result
(<date>)` headings (optionally `(<date> HH:MM)`, time informational) and in `night-shift/sitting-<date>`; with no date, every Sitting since the
last debrief is in scope; `--dry-run` = run the analysis (section 1) and the derived-verdict table
(section 1.C), write nothing to the repo or the map, and stop after the verdict table).

Two phases. **Analysis (section 1)** reads the new test data against the ongoing research,
derives the verdicts, and ends with write-backs to Items, plans and the logbook. **Paperwork
(sections 2 to 6)** is the close-out, unchanged from `/sitting-wrapup` apart from where the
results come from.

## 0. Where you are, and the rules

- You run in `~/Desktop/Jugglebot`, the live checkout, on `main`. **Single-writer rule**: a
  peer Session may be live here. `git fetch && git status -sb` first; a working tree with
  changes you did not make, or `origin/main` ahead of local, means **pause and say what you
  see** before any git write. Never `git stash`, never `git branch -f`, never a force-push.
- **Merges never happen in this working tree.** `tools/sitting_merge.sh` (section 3) makes a
  scratch worktree per Item, `temp/debrief/wt-<ID>` (gitignored), merges and runs the suite
  there, and pushes the gated tip to `origin/main`; this checkout only follows with
  `git pull --ff-only`. A failed merge leaves the branch held with the conflict named; it is
  never resolved by guesswork and never by taking either side wholesale. The two exceptions
  are the pages `python -m mapkit gen` writes and the Item's own `map/` file (`main` is the
  authority for map state), which the script takes from `main` before it regenerates.
- **Commit the Sitting's records and learner Memory first**, before the first `prepare`, so the
  suite runs on a `main` that already holds them (the 2026-11-13 debrief went red on `main`
  only after its records commit). Whatever the Sitting wrote to tracked files (`docs/records.md`,
  `temp/learn/jugglebot/memory*.csv`; once closeout C8 lands, the output of
  `python tools/runtime_records.py merge`) is committed on `main` and pushed, on its own,
  with the `Logbook-Entry: <date>-sitting-debrief` trailer.
- Nothing here powers hardware. The Sitting is over.

## 0a. Flush the Sitting sheet's results

The operator's notes live in the map app's clone until the app pushes them. Make that happen
first, so they reach `main`: for each sheet in scope (`ls map/sheets/<date>-*.yaml`; `<name>` is the
file name without `.yaml`)

```
curl -sS -X POST http://localhost:8765/api/v1/sheets/<name>/close
```

It answers `flushed: true` only after the push; then `git pull --ff-only` (this checkout's
single-writer check from section 0 has already passed). Anything else means the notes are not on
`main`: **do not hand-copy the results file** from the app's clone
(`~/map-live/Jugglebot/map/sheets/<name>.results.yaml` holds it on disk, so nothing is lost).
App down (connection refused): start it (`systemctl --user start jugglebot-map`) and call again,
or ask the owner. An error body or a non-`true` `flushed`: report it verbatim and ask the owner;
do not go on to the analysis on a results file you could not flush.

## 1. Analyse the data (before any paperwork)

Writing-for-agents rule: you orchestrate and synthesise; sub-agents read the logs. Scope is
every launch log under `~/.ros/log/` (`temp/logs/launch_<date>_<n>.log` exists only when the
operator's tee worked, so `~/.ros/log/<date>-*/launch.log` is the complete source) and every rosbag under `~/Desktop/rosbags/` newer than
the last debrief entry (or the day of the date argument). The dirty `docs/records.md` and
`temp/learn/jugglebot/memory.csv` are the Sitting's expected output, not a surprise. No
robot, no GUI writes: this is reading.

**A. Inventory (you, not an agent).** In a scratchpad `context.md` list, per launch log, with
line numbers: every Attempt start (`grep -n ' started:' launch.log`: the line is `<pattern> ... started:`),
every end (`grep -n ' ENDED' launch.log`: reason) and every success (`grep -nE ' done.*: [0-9]+/[0-9]+ caught'`:
`<pattern> done...: N/M caught`; the older `goal dispatched (pattern=` and `ENDED (.*caught` greps
match nothing or too little on current logs), plus the bag path and the sheet the Sitting flew
(`map/sheets/<name>.yaml` and `<name>.results.yaml`: its steps, each verdict-carrying Item's
step and the Item's `pass:` criterion, the owner's notes per step). A Sitting flown with no sheet
has no such file: use the Items' own `## Sitting` sections (`do:`, `pass:`, `record:`). Then ONE
`AskUserQuestion`: (1) the verdict source is not asked: on a schema 2 sheet you derive every
verdict from the logs and the owner confirms the table once (a sheet-less or schema 1 Sitting,
whose results were typed live, keeps its typed verdicts, section 2), so question 1 is the Sitting's
date scope if the `<date>` argument was absent, else dropped; (2) fix scope, default = diagnose and fix non-threshold bugs, while any
limit, margin or guard change comes back as an owner decision with the numbers; (3) physical
observations the logs cannot hold (sounds, a ball that looked wrong, a rig change). Put the
answers in `context.md`; every unit below reads it.

**B. Parallel units** (one `Agent` each, Sonnet by default; Opus only for diagnosis that is
safety- or control-bearing, one line of justification in its brief; each under ~80 tool
calls, writes `<unit>.md` to the scratchpad, reports only its path and a verdict line):
1. *Sheet verdicts*: for every verdict-carrying Item of every step (`pass: true` on the step's
   `serves`, `python -c` over `mapkit.sheet.verdict_items`), extract exactly what the step's
   `record:` line and the Item's `pass:` criterion ask, from the logs and bags, and judge the
   criterion literally. Output per Item: `verdict` (`pass` | `fail` | `data` | `inconclusive`),
   `evidence` (the log or bag path plus the measured value) and `criterion` (the step's pass
   text VERBATIM). The owner's notes are leads to where to look, never the verdict. A criterion
   that needs a number the logs lack is `inconclusive`, never `pass`; `data` is a flight that
   produced readings but has no pass/fail criterion. A sheet-less Sitting does the same per
   Item from its `## Sitting` section's `pass:`.
2. *One diagnosis unit per attempt-killer class*: group the `ENDED` reasons (guard latches,
   limit-margin trips, missed catches, timeouts...). Each unit first tests the open Item's
   hypothesis against the new data (confirm, refute, or cannot tell, with the evidence lines)
   before proposing a new one. A refuted hypothesis is stated as withdrawn, not rescued.
3. *Crosswalk*: which open plan steps, plan Questions and Items (`python -m mapkit items
   --open --json`, `docs/patterns.md` rungs) this data can advance, and what each still needs.
4. *Trapped-Ball scan* (you, standing, every bag, before the Hand-stall and missed-Catch units start):
   `source ~/Desktop/PDJ_venv/venv/bin/activate; for b in <each bag>; do python tools/trapped_ball_finder.py $b [--hand-csv <Hand capture csv>] > temp/logs/trapped_$(basename $b).txt; done`
   (5-12 min per bag, run them in parallel). Copy each report's section (b) and the UNDER_HAND dwells into
   the scratchpad `context.md` and hand every Hand stall and every under-Hand dwell to the diagnosis units.
   A stall that says TRAPPED MARKER COINCIDES is BUG-84 confirmed for that event (marker position and the
   Ball it traces to); one that says "cannot see" is stated as such and never read as "refuted"; a
   Ball-origin question in a missed-Catch unit reads the dwell list for that Ball (a Ball the Tracker dropped
   can still sit under the Cup). A Hand stall is never judged from the Tracker's ball tracks alone: `/balls`
   drops a Ball once it jostles on the Platform.

**C. Synthesis and the one confirmation (you).** Read the unit files, not the logs.

1. *Derive.* Write the verdicts into the results file (not under `--dry-run`): a scratchpad YAML
   `{<step n>: {<Item ID>: {verdict, evidence, criterion, by: sitting-debrief <date>}}}` for every
   verdict-carrying Item, then `python -m mapkit sheet derive <name> <file.yaml>`. Never guess a pass:
   `inconclusive` when the logs cannot decide.
2. *One table.* Show the owner ONE table, `Item | step | criterion | measured | derived`, with the
   killer class and proposed write-back per row, and ask the owner to confirm or flip rows, once.
   The owner's notes were leads, never the verdict. `--dry-run` stops here (it wrote nothing,
   so it shows the table from your unit files only).
3. *Apply flips,* if any: edit the YAML's `verdict` (keep the criterion and say in `evidence`
   that the owner flipped it), re-run `derive` (it replaces the earlier set), then
   `python -m mapkit sheet confirm <name> --at "YYYY-MM-DD HH:MM" [--changed <ID> ...]` (the
   owner's confirmation time; `--changed` names the flipped Items). It refuses until every
   verdict-carrying Item has a derived entry.
4. *Close.* `python -m mapkit sheet close <name>` writes each Item's `## Sitting result` exactly
   once, from the confirmed verdicts (it refuses an unconfirmed or already closed file, and a
   schema 1 file, whose verdicts already went into the Items live). Those Item files and the
   results file ride in the debrief commit. Several sheets: derive, confirm and close each.

Then the write-backs, in this order:
- Item progress notes and closes: `python -m mapkit item set-status <ID> <status> --note "..."`
  (no `--commit`; the files ride in the debrief commit);
- new Items for new facts: `python -m mapkit item new <bug|feature|idea> ...`;
- plan status lines and the `docs/patterns.md` rung status the data moves;
- non-threshold fixes the owner's scope allows, each its own gate-scoped change; threshold
  changes (limits, margins, guards) are listed as owner decisions with the numbers and not made;
- the logbook entry `logbook/<date>-sitting-debrief.md` (written at section 5): full
  investigation form with a Discussion if any hypothesis was withdrawn or reframed, short form
  otherwise.
Then go on to sections 2 to 6.

## 2. Read the results

1. **From a sheet (schema 2 results, the default).** The confirmed table of section 1.C is the
   result set: after `sheet close` each verdict-carrying Item holds one `## Sitting result
   (<date> HH:MM)` section whose text is the owner's note plus `(sheet <name>, step <n>)` and
   `[derived: <evidence>]`. The results file also holds what exists nowhere else: a `problem` or
   `skipped` step and any step note (a `skipped` step was not flown, so it is no evidence either
   way for the Items it serves). An earlier typed or derived result on an Item is reported in the
   table, never acted on.
2. **Fallback: a Sitting flown without a sheet, or from a schema 1 sheet** (results typed
   straight onto Items, verdicts saved live). `python -m mapkit items --open --json` and, for
   every hardware Item (`hardware` bench-leg, bench-hand or robot), read its body for
   `## Sitting result (<date>)` sections (the heading may carry a trailing `HH:MM`, informational;
   scope by the date part; parser: `mapkit.sitting.parse_sitting_results`) newer than the last
   debrief (the last debrief's date is the newest `logbook/*-sitting-debrief.md` or older
   `logbook/*-sitting-wrapup.md` entry; none = all of them). A schema 1 sheet's
   `map/sheets/<name>.results.yaml` is read alongside: a `problem` or `skipped` step and any
   step comment exist nowhere else. **An Item with several results from one Sitting: the last in
   file order is its verdict** (a re-save appends, so an earlier `pass` may have been
   corrected). Report the earlier ones in the table, never act on them; if the last differs from
   the results file's verdict for that Item, stop and ask. These typed verdicts are reconciled
   against section 1's log-derived ones in the C table, and the owner confirms that table once.
3. For each result: Item, verdict (`pass`, `fail`, `data`, `inconclusive`, or the typed
   `pending`), notes, and the held branch the Item's `## Sitting` section names (if any). List
   them in a table before touching anything.
4. A `pending` (typed, fallback path only) or `inconclusive` result means the logs and the owner
   could not judge the criterion: it is neither closed nor merged, and gets a Progress line
   `Flown at the Sitting of <date>, verdict unresolved: <what is missing>` so the next sheet
   keeps it. `data` closes nothing either: a Progress line with the readings. The section stays
   on the Item as the record of what was flown.

## 3. Merge the passed Items, one at a time

For each result whose confirmed verdict is `pass` (`fail`, `data` and `inconclusive` never merge;
on the fallback path a typed `pending` counts once resolved to pass) AND a held branch `night-shift/<d>-<ID>` (the
`branch:` line of its `## Sitting` section; if the section has none, still look for
`git branch -r --list 'origin/night-shift/*-<ID>'` before treating the pass as a plain check,
so held software work is never dropped), land it with `tools/sitting_merge.sh`, the only path
from a held branch to `main` (closeout C10; its header has every rule and exit code, and
`tests/sim/test_sitting_merge.py` pins them). The branch was gated against an older `main`, so
`prepare` brings it up to date and runs the full suite again (owner, 2026-11-07): a Sitting pass
says the flown code works; only this says it still works beside what has landed since. The
flown branch stays as it is on the remote until it lands. Be on `main` first, with the records
and Memory already committed (section 0). One Item at a time, `prepare` to `land`: a land moves
`main`, and the next Item's `land` refuses a `main` its `prepare` did not see.

1. `tools/sitting_merge.sh prepare --branch night-shift/<d>-<ID> --item <ID> --work ~/Desktop/Jugglebot/temp/debrief`
   with `run_in_background` (the suite can outlast a foreground call; it takes the suite lock
   itself, so never wrap it in `flock`). It ends `PREPARED <ID> <tip>`, or `REFUSED <code> <reason>`.
2. One `audit-reporter` agent (`model: sonnet`) on `temp/debrief/diff-<ID>.patch`; you write its
   result to `temp/debrief/audit-<ID>.md`, findings marked as in the night's 2e, ending with
   `Audited HEAD: <tip>` and `Behaviour-affecting findings open: <n>`. Narrative findings are
   recorded, not applied: any commit after the suite makes `land` refuse. A branch that touches
   `mapkit/` also needs `tools/night_shift_nonet.sh tools/night_shift_app_suite.sh --app ~/night-shift/jugglebot-map --mapkit temp/debrief/wt-<ID> --log temp/debrief/app-<ID>.log` (condition (h)).
3. `tools/sitting_merge.sh land --branch night-shift/<d>-<ID> --item <ID> --work ~/Desktop/Jugglebot/temp/debrief`.
   It runs the base's gate with `--waive c`: the hardware pass overrides ONLY the hardware
   condition. `LANDED` means `main` is now exactly the gated tip. A `HOLD` (open audit findings,
   a normative radius, an open question, a changed `owner` rating: condition (g), which no waiver
   clears) keeps the branch held with the reasons in the summary; the Sitting result stays on the Item.

- `TOOK MAIN's copy of map/<file>` (also in the gate log header): the Item's own file conflicted
  and `main`'s copy was kept; the branch's copy is `temp/debrief/branch-copy-<ID>.md`. Anything
  its body adds that `main` lacks goes into the Item when you close it, under a
  `## From the branch (<date>)` heading.
- A refusal is never worked around. `5` (a conflict outside the generated pages and the Item's
  own file, another Item's `map/` file included): the Item stays `in_progress`, the branch
  stays held, the summary names the files. `12` (`main`
  moved): `prepare` again, once; if it moves again the branch stays held: `main kept moving`.
  `4` (a frozen path, gate condition (f)): the script has no flag for it; such a branch is merged
  by hand in a day Session at the owner's say-so, named in the summary as not gated. Go on to the
  next Item (per-Item merges keep one failure from blocking the rest; never merge the candidate
  branch `night-shift/sitting-<date>` itself, it is a build product).
- A merge that lands: close the Item with the Sitting as evidence:
  `python -m mapkit item set-status <ID> complete --note "Passed at the Sitting of <date>: <one line from the notes>" --link logbook/<date>-sitting-debrief.md`
  plus `--link` for the branch's own logbook entry (no `--commit`; its paths ride in the debrief commit below).
- A `fail` verdict: the Item stays `in_progress`; append the headline of the notes as a
  Progress line (`set-status <ID> in_progress --note "Failed at the Sitting of <date>: <headline>"`,
  no `--commit`); its branch stays held and its `## Sitting` section is read again by the next `/sitting-sheet`
  (edit the section if the notes say the `do:` or `pass:` was wrong).
- A `pass` with no branch anywhere (a plain check): close the Item the same way, no merge.
- The candidate branch `night-shift/sitting-<date>` is built before the Sitting by the owner or
  the preparing Session with `tools/night_shift_candidate.sh --date <date> --push`; it is only
  what the robot ran, never what merges.

## 4. Return the live checkout and rebuild

1. `git fetch && git pull --ff-only` (the lands moved `origin/main`; the auto-pull may already
   have followed). `git status -sb` must read `## main...origin/main` plus only the files this
   command changed. If the checkout is somehow still on `night-shift/sitting-<date>`,
   `git checkout main`.
2. If any merged branch touched `ros_ws/`: `cd ros_ws && colcon build --packages-select
   jugglebot_interfaces jugglebot` (both, always: `jugglebot` alone installs new node code over an
   old interface, and the mismatch only shows at the first service call) and say so.
3. `git add docs/records.md` if it changed again since section 0's records commit (skill_node
   writes it live during a Sitting; it is committed at every Sitting close-out).

## 5. Follow-ups, logbook, Runbook, commit

1. For every note that names a new problem or idea, file it:
   `python -m mapkit item new <bug|feature|idea> "<title>" --body "<the note, with the Item and date>" --link map/<stem>.md --priority <p>` (no `--commit`; the files ride in the debrief commit).
   Rate it when you know the answer.
2. Write `logbook/<date>-sitting-debrief.md` (short form: the results table, what merged with
   SHAs, what stayed held and why, follow-ups filed, the rebuild if any); step 3 gives it its
   `logbook/INDEX.md` row.
3. `python -m mapkit gen` (`logbook/INDEX.md` gains the entry's row; the Runbook index `docs/runbook.md` drops the passed Items and keeps
   the failed ones; the project-map page follows the status changes).
4. `./run_tests.sh` (each merged branch passed the full gate on its updated tip; the merged
   result gets the per-commit gate here, the nightly covers the rest tonight). Red: stop, say
   which test, do not commit, and name the landed SHAs (already on `origin/main`, each the tip
   its gate recorded MERGE for) for the owner to decide. Never revert or reset them away.
5. One commit: `docs(sitting): debrief <date> - <n> passed, <m> merged, <k> held` with the
   trailers `Logbook-Entry: <date>-sitting-debrief` and
   `Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>` (the commit holds the Items' new
   `## Sitting result` sections and each closed sheet's `<name>.results.yaml`); `git fetch && git status -sb`
   again; `git push origin main`. Then push nothing else. If `origin/main` has moved and the push is
   rejected: section 0 applies, so pause and say what you see.

## 6. Finish

Say, in three lines: results read (passed / failed), merged and closed (with SHAs), held
(with why). Offer the owner `/sitting-sheet <rig>` for the next Sitting (the Runbook index lists what waits) and nothing else.

Pinned by `tests/sim/test_sitting_debrief_protocol.py`: this file names every close-out step
(the flush call first, results read, derive, one owner confirmation, confirm, close, per-Item
merge, return to `main`, rebuild if `ros_ws` changed, `docs/records.md`, follow-ups filed, the
logbook entry, `mapkit gen`, the single-writer check, no force-push, the records first, the merge
through `tools/sitting_merge.sh`), and that it reads nothing from the Runbook's tables. What the script does
(brought up to date, suite, base gate with `--waive c`, the gated tip only, the refusals) is
pinned by `tests/sim/test_sitting_merge.py` against temp repos; derive, confirm and close by
`tests/sim/test_mapkit_sheet.py`.
