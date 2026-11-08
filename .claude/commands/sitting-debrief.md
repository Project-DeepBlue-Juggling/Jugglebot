---
description: Debrief the Sitting(s) flown from the Next Sitting Runbook since the last debrief: first ANALYSE the new launch logs and rosbags against the open Items and plans (inventory, verdicts, per-killer diagnosis, crosswalk, write-backs), then close the paperwork - read every new ## Sitting result, merge the passed Items' held branches into main one by one, return the live checkout to main, rebuild if ros_ws changed, commit docs/records.md, file follow-ups from the notes, write the logbook entry, regenerate the Runbook. Invoke with /sitting-debrief [<date>] [--dry-run]. Runs in ~/Desktop/Jugglebot under the single-writer rule.
---

# Sitting debrief

(Formerly `/sitting-wrapup`; that command's paperwork is sections 2 to 6 below, unchanged.)

The morning or post-Sitting Session's one prompt. The owner's share of the Sitting was the
hardware steps and the result boxes in the map app; everything that is git, tests and
paperwork is this command. It is the close-out the Next Sitting Runbook (`docs/runbook.md`,
`## Close-out`) promises, step for step.

Arguments: `$ARGUMENTS` (`<date>` = the Sitting's date, the one in the `## Sitting result
(<date>)` headings (optionally `(<date> HH:MM)`, time informational) and in `night-shift/sitting-<date>`; with no date, every Sitting since the
last debrief is in scope; `--dry-run` = run the analysis (section 1) and the results table
(section 2.2), write nothing to the repo or the map, and stop after the verdict table).

Two phases. **Analysis (section 1)** reads the new test data against the ongoing research and
ends with write-backs to Items, plans and the logbook. **Paperwork (sections 2 to 6)** is the
close-out the Runbook promises, unchanged from `/sitting-wrapup`.

## 0. Where you are, and the rules

- You run in `~/Desktop/Jugglebot`, the live checkout, on `main`. **Single-writer rule**: a
  peer Session may be live here. `git fetch && git status -sb` first; a working tree with
  changes you did not make, or `origin/main` ahead of local, means **pause and say what you
  see** before any git write. Never `git stash`, never `git branch -f`, never a force-push.
- A failed merge leaves the branch held with the conflict named; it is never resolved by
  guesswork and never by taking either side wholesale. The one exception is the three
  paperwork files listed in section 3, step 1, whose right answer is mechanical.
- Nothing here powers hardware. The Sitting is over.

## 1. Analyse the data (before any paperwork)

Writing-for-agents rule: you orchestrate and synthesise; sub-agents read the logs. Scope is
every launch log under `~/.ros/log/` and every rosbag under `~/Desktop/rosbags/` newer than
the last debrief entry (or the day of the date argument). The dirty `docs/records.md` and
`temp/learn/jugglebot/memory.csv` are the Sitting's expected output, not a surprise. No
robot, no GUI writes: this is reading.

**A. Inventory (you, not an agent).** In a scratchpad `context.md` list, per launch log, with
line numbers: every goal (`grep -n 'Juggle requested.*goal dispatched (pattern='`) and every
attempt end (`grep -n 'ENDED (.*caught'`: reason, `N/M caught`), plus the bag path and the
`## Sitting` rows of the Runbook the Sitting flew. Then ONE `AskUserQuestion`: (1) verdict
source (the owner's typed `## Sitting result` boxes, or you derive verdicts from the logs and
the owner confirms); (2) fix scope, default = diagnose and fix non-threshold bugs, while any
limit, margin or guard change comes back as an owner decision with the numbers; (3) physical
observations the logs cannot hold (sounds, a ball that looked wrong, a rig change). Put the
answers in `context.md`; every unit below reads it.

**B. Parallel units** (one `Agent` each, Sonnet by default; Opus only for diagnosis that is
safety- or control-bearing, one line of justification in its brief; each under ~80 tool
calls, writes `<unit>.md` to the scratchpad, reports only its path and a verdict line):
1. *Runbook verdicts*: for each Runbook row, extract exactly what its `record:` line asks,
   from the logs and bags, and judge `pass:` literally. A pass that needs a number the logs
   lack is `unverified`, never `pass`.
2. *One diagnosis unit per attempt-killer class*: group the `ENDED` reasons (guard latches,
   limit-margin trips, missed catches, timeouts...). Each unit first tests the open Item's
   hypothesis against the new data (confirm, refute, or cannot tell, with the evidence lines)
   before proposing a new one. A refuted hypothesis is stated as withdrawn, not rescued.
3. *Crosswalk*: which open plan steps, plan Questions and Items (`python -m mapkit items
   --open --json`, `docs/patterns.md` rungs) this data can advance, and what each still needs.

**C. Synthesis (you).** Read the unit files, not the logs. Show the owner one verdict table
(row, verdict, evidence line, killer class, proposed write-back) and wait for confirmation
or correction. `--dry-run` stops here. Then the write-backs, in this order:
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

1. `python -m mapkit items --open --json` and, for every hardware Item (`hardware` bench-leg, bench-hand or
   robot) plus every Item named by a `branch:` in `docs/runbook.md`, read its body for
   `## Sitting result (<date>)` sections (the heading may carry a trailing `HH:MM`, informational; scope by the date part; parser: `mapkit.sitting.parse_sitting_results`) newer than the last debrief (the last debrief's date
   is the newest `logbook/*-sitting-debrief.md` or older `logbook/*-sitting-wrapup.md` entry; none = all of them).
2. For each result: Item, verdict (`pass`, `fail` or `pending`), notes, and the held branch its `## Sitting`
   section names (if any). List them in a table before touching anything. (Section 1 has already produced the verdicts; this table is the Items' typed results reconciled against them.)
3. A `pending` result means the row was flown but the owner could not judge its `pass:` live: its
   verdict is section 1's log-derived one (`unverified` if the logs cannot settle it), confirmed
   by the owner in the C table. Sections 3 onward act on that resolved verdict; an `unverified`
   row is neither closed nor merged, and gets a Progress line `Flown at the Sitting of <date>,
   verdict unresolved: <what is missing>` so the next Runbook keeps it. The typed `pending`
   section stays on the Item as the record of what was flown.

## 3. Merge the passed Items, one at a time

For each result with verdict `pass` (typed, or a `pending` resolved to pass) AND a held branch `night-shift/<d>-<ID>` (the
`branch:` line of its `## Sitting` section; if the section has none, still look for
`git branch -r --list 'origin/night-shift/*-<ID>'` before treating the pass as a plain check,
so held software work is never dropped):

The branch was gated against an older `main`, and `main` moves many times a day, so it is brought
up to date and gated again before it merges (owner, 2026-11-07). A Sitting pass says the flown
code works; only this says it still works beside what has landed since. Budget about 10 minutes
of full suite per branch, plus an audit. Do not start while a launch is up: the suite loads the box.
Be on `main` first: `git checkout main` if the Sitting left the checkout on the candidate branch.
Shell variables do not survive between tool calls, so `BASE` is written to a file and every block below
reads it back. Never recompute it after a merge: it must stay the base this Item's suite ran on.

1. **Update, in a throwaway worktree.** The branch stays as it is on the remote, because that is
   what flew. Later Items build on earlier ones, because the base is local `main`:
   ```
   mkdir -p temp/reports/debrief && git rev-parse main > temp/reports/debrief/base-<ID>
   BASE=$(cat temp/reports/debrief/base-<ID>)
   git worktree remove --force temp/debrief-<ID> 2>/dev/null; git worktree prune
   git worktree add -q -B night-shift/<d>-<ID> temp/debrief-<ID> origin/night-shift/<d>-<ID>
   git -C temp/debrief-<ID> merge --no-edit $BASE
   ```
   A held branch nearly always conflicts here, on files only the paperwork touches. These three,
   and no others, have a mechanical answer (the exception to section 0's rule):
   - the Item's own `map/<stem>.md`: `main`'s copy, which holds the Sitting result. First read the
     branch's copy: anything its body adds that `main` lacks goes into the Item on `main` when you
     close it, under a `## From the branch (<date>)` heading;
   - `logbook/INDEX.md`: keep both sides' rows, dropping only the conflict markers;
   - generated pages (`docs/project-map.md`, `docs/runbook.md`, `docs/glossary.md`, `plans/*/INDEX.md`):
     `main`'s copy, then `python -m mapkit gen` in the worktree.

   Take `main`'s copy by naming the commit, `git -C temp/debrief-<ID> checkout $BASE -- <paths>`, never
   `--ours` or `--theirs`: in this merge direction `--ours` is the branch. Anything else conflicting:
   `git -C temp/debrief-<ID> merge --abort` and treat it as the conflict below. Otherwise finish with
   `git -C temp/debrief-<ID> add -A`, then `git -C temp/debrief-<ID> diff --cached --quiet || git -C temp/debrief-<ID> commit --no-edit`
   (a clean merge has already committed).
   Never edit the flown code here. If `origin/main` was already inside the branch and `BASE` adds
   nothing (`git merge` says up to date), the tip has not moved: step 3 can use the night's gate log
   and audit from `~/night-shift/Jugglebot/temp/reports/night-shift/`.
2. **Audit, then the full suite, on the updated tip.** The gate only accepts results for that tip.
   Save the diff first and check it is not empty, because an audit of an empty diff finds nothing and
   the gate cannot tell: `git -C temp/debrief-<ID> diff "$(cat temp/reports/debrief/base-<ID>)...HEAD" > temp/reports/debrief/diff-<ID>.patch`.
   Empty means the branch adds nothing: stop and say so. One `audit-reporter` agent (`model: sonnet`) on
   that file; you write its result to `temp/reports/debrief/audit-<ID>.md`, findings marked as in the
   night's 2f and ending with `Audited HEAD: <sha>` and `Behaviour-affecting findings open: <n>`.
   Narrative findings are recorded, not applied. A behaviour-affecting one keeps the branch held and
   skips the suite. Then, with Foxy and the venv sourced, and with `run_in_background` because the
   suite can outlast a foreground call (wait for the `EXIT=` line):
   ```
   cd temp/debrief-<ID>
   printf 'HEAD=%s\nBRANCH=%s\n' "$(git rev-parse HEAD)" "$(git rev-parse --abbrev-ref HEAD)" > ../reports/debrief/gate-<ID>.log
   tools/night_shift_nonet.sh ./run_tests.sh --full >> ../reports/debrief/gate-<ID>.log 2>&1; echo "EXIT=$?" >> ../reports/debrief/gate-<ID>.log
   cd ../..
   ```
   A branch that touches `mapkit/` also needs the app's suite against it (condition (h)):
   `tools/night_shift_nonet.sh tools/night_shift_app_suite.sh --app ~/night-shift/jugglebot-map --mapkit temp/debrief-<ID> --log temp/reports/debrief/app-<ID>.log`.
3. **The gate.** The hardware pass overrides ONLY the gate's hardware condition, and the gate is
   an extract of the base, never the working tree:
   ```
   BASE=$(cat temp/reports/debrief/base-<ID>)
   rm -rf temp/gate-base && mkdir -p temp/gate-base
   git archive $BASE tools/night_shift_gate.py mapkit | tar -x -C temp/gate-base
   python temp/gate-base/tools/night_shift_gate.py --repo . --base $BASE --branch night-shift/<d>-<ID> --item <ID> --gate-log temp/reports/debrief/gate-<ID>.log --audit temp/reports/debrief/audit-<ID>.md --waive c [--app-log temp/reports/debrief/app-<ID>.log]
   ```
   Anything but `MERGE (c waived)` (a red or missing gate log, open audit findings, a
   normative radius, an open question, a forbidden path, a changed `owner` rating: condition
   (g), which no waiver clears) keeps the branch held with the
   reasons in the summary; the Sitting result stays on the Item. A forbidden path is a day merge,
   made by the owner's say-so in this Session and named in the summary as not gated.
4. **Merge.** `main` is still at the base unless the auto-pull moved it:
   ```
   git merge -q --ff-only night-shift/<d>-<ID>
   ```
   If `main` moved and `git diff --name-only "$(cat temp/reports/debrief/base-<ID>)" main` lists only `map/`, `docs/`, `logbook/` and
   `plans/` paths, no code changed under the result:
   `git merge -q --no-ff -m "merge(sitting <date>): <ID> <title>" night-shift/<d>-<ID>`.
   If code moved, go back to step 1 once. If it moves again, the branch stays held: `main kept moving`.
5. **Clean up**, whatever the outcome: `git worktree remove --force temp/debrief-<ID>` and
   `git branch -D night-shift/<d>-<ID>`. That deletes the local copy only; the remote branch stays.

- A conflict outside step 1's three: `git -C temp/debrief-<ID> merge --abort`; the Item stays `in_progress`, the branch stays held, and
  the summary names the conflicting files. Go on to the next Item (per-Item merges are what
  keep one failure from blocking the rest; never merge the candidate branch
  `night-shift/sitting-<date>` itself, it is a build product).
- A merge that lands: close the Item with the Sitting as evidence:
  `python -m mapkit item set-status <ID> complete --note "Passed at the Sitting of <date>: <one line from the notes>" --link logbook/<date>-sitting-debrief.md`
  plus `--link` for the branch's own logbook entry (no `--commit`; its paths ride in the debrief commit below).
- A `fail` verdict: the Item stays `in_progress`; append the headline of the notes as a
  Progress line (`set-status <ID> in_progress --note "Failed at the Sitting of <date>: <headline>"`,
  no `--commit`); its branch stays held and its `## Sitting` section is re-read on the next
  Runbook regeneration (edit the section if the notes say the `do:` or `pass:` was wrong).
- A `pass` with no branch anywhere (a plain check): close the Item the same way, no merge.
- The candidate branch `night-shift/sitting-<date>` is built before the Sitting by the owner or
  the preparing Session with `tools/night_shift_candidate.sh --date <date> --push`; it is only
  what the robot ran, never what merges.

## 4. Return the live checkout and rebuild

1. `git status -sb` must read `## main...origin/main` plus only the files this command changed.
   If the checkout is somehow still on `night-shift/sitting-<date>`, `git checkout main`
   (section 3 began by switching, because its merges happen on `main`).
2. If any merged branch touched `ros_ws/`: `cd ros_ws && colcon build --packages-select
   jugglebot_interfaces jugglebot` (both, always: `jugglebot` alone installs new node code over an
   old interface, and the mismatch only shows at the first service call) and say so.
3. `git add docs/records.md` if it changed (skill_node writes it live during the Sitting; it
   is committed at every Sitting close-out).

## 5. Follow-ups, logbook, Runbook, commit

1. For every note that names a new problem or idea, file it:
   `python -m mapkit item new <bug|feature|idea> "<title>" --body "<the note, with the Item and date>" --link map/<stem>.md --priority <p>` (no `--commit`; the files ride in the debrief commit).
   Rate it when you know the answer.
2. Write `logbook/<date>-sitting-debrief.md` (short form: the results table, what merged with
   SHAs, what stayed held and why, follow-ups filed, the rebuild if any) and its
   `logbook/INDEX.md` row.
3. `python -m mapkit gen` (the Runbook page `docs/runbook.md` drops the passed rows and keeps
   the failed ones; the project-map page follows the status changes).
4. `./run_tests.sh` (each merged branch passed the full gate on its updated tip; the merged
   result gets the per-commit gate here, the nightly covers the rest tonight). Red: stop, say
   which test, do not commit, and name the merged SHAs: they are on local `main`, unpushed, and
   stay there for the owner to decide. Never reset them away.
5. One commit: `docs(sitting): debrief <date> - <n> passed, <m> merged, <k> held` with the
   trailers `Logbook-Entry: <date>-sitting-debrief` and
   `Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>`; `git fetch && git status -sb`
   again; `git push origin main`. Then push nothing else. If `origin/main` has moved and the push is
   rejected: section 0 applies, so pause and say what you see, with the merged SHAs named.

## 6. Finish

Say, in three lines: results read (passed / failed), merged and closed (with SHAs), held
(with why). Offer the owner the Runbook link for the next Sitting and nothing else.

Pinned by `tests/sim/test_sitting_debrief_protocol.py`: this file names every step the
Runbook's `## Close-out` promises (results read, per-Item merge, return to `main`, rebuild if
`ros_ws` changed, `docs/records.md`, follow-ups filed, the logbook entry, `mapkit gen`, the
single-writer check, no force-push, the branch brought up to date and the suite, audit and gate
re-run on it with `--waive c`).
