---
description: Close a Sitting flown from the Next Sitting Runbook - read every new ## Sitting result, merge the passed Items' held branches into main one by one, return the live checkout to main, rebuild if ros_ws changed, commit docs/records.md, file follow-ups from the notes, write the logbook entry, regenerate the Runbook. Invoke with /sitting-wrapup <date> [--dry-run]. Runs in ~/Desktop/Jugglebot under the single-writer rule.
---

# Sitting wrap-up

The morning or post-Sitting Session's one prompt. The owner's share of the Sitting was the
hardware steps and the result boxes in the map app; everything that is git, tests and
paperwork is this command. It is the close-out the Next Sitting Runbook (`docs/runbook.md`,
`## Close-out`) promises, step for step.

Arguments: `$ARGUMENTS` (`<date>` = the Sitting's date, the one in the `## Sitting result
(<date>)` headings and in `night-shift/sitting-<date>`; `--dry-run` = list what would happen
and change nothing).

## 0. Where you are, and the rules

- You run in `~/Desktop/Jugglebot`, the live checkout, on `main`. **Single-writer rule**: a
  peer Session may be live here. `git fetch && git status -sb` first; a working tree with
  changes you did not make, or `origin/main` ahead of local, means **pause and say what you
  see** before any git write. Never `git stash`, never `git branch -f`, never a force-push.
- A failed merge leaves the branch held with the conflict named; it is never resolved by
  guesswork and never by taking either side wholesale.
- Nothing here powers hardware. The Sitting is over.

## 1. Read the results

1. `python -m mapkit items --open --json` and, for every hardware Item (`hardware` bench-leg or
   robot) plus every Item named by a `branch:` in `docs/runbook.md`, read its body for
   `## Sitting result (<date>)` sections newer than the last wrap-up (the last wrap-up's date
   is the newest `logbook/*-sitting-wrapup.md` entry; none = all of them).
2. For each result: Item, verdict (`pass` or `fail`), notes, and the held branch its `## Sitting`
   section names (if any). List them in a table before touching anything. `--dry-run` stops here.

## 2. Merge the passed Items, one at a time

For each result with verdict `pass` AND a held branch `night-shift/<d>-JB-n` (the
`branch:` line of its `## Sitting` section; if the section has none, still look for
`git branch -r --list 'origin/night-shift/*-JB-n'` before treating the pass as a plain check,
so held software work is never dropped):

1. The hardware pass overrides ONLY the gate's hardware condition. Re-run the gate with the
   night's own artefacts (the tip has not moved, so they still match):
   ```
   python tools/night_shift_gate.py --repo . --base origin/main --branch origin/night-shift/<d>-JB-n --item JB-n \
       --gate-log ~/night-shift/Jugglebot/temp/reports/night-shift/gate-JB-n.log \
       --audit ~/night-shift/Jugglebot/temp/reports/night-shift/audit-JB-n.md --waive c
   ```
   Anything but `MERGE (c waived)` (a red or missing gate log, open audit findings, a
   normative radius, an open question, a forbidden path) keeps the branch held with the
   reasons in the summary; the Sitting result stays on the Item.
2. ```
   git fetch -q origin
   git merge -q --ff-only origin/night-shift/<d>-JB-n || git merge -q --no-ff -m "merge(sitting <date>): JB-n <title>" origin/night-shift/<d>-JB-n
   ```

- A conflict: `git merge --abort`; the Item stays `in_progress`, the branch stays held, and
  the summary names the conflicting files. Go on to the next Item (per-Item merges are what
  keep one failure from blocking the rest; never merge the candidate branch
  `night-shift/sitting-<date>` itself, it is a build product).
- A merge that lands: close the Item with the Sitting as evidence:
  `python -m mapkit item set-status JB-n complete --note "Passed at the Sitting of <date>: <one line from the notes>" --link logbook/<date>-sitting-wrapup.md`
  (no `--commit`; its paths ride in the wrap-up commit below).
- A `fail` verdict: the Item stays `in_progress`; append the headline of the notes as a
  Progress line (`set-status JB-n in_progress --note "Failed at the Sitting of <date>: <headline>"`,
  no `--commit`); its branch stays held and its `## Sitting` section is re-read on the next
  Runbook regeneration (edit the section if the notes say the `do:` or `pass:` was wrong).
- A `pass` with no branch anywhere (a plain check): close the Item the same way, no merge.
- The candidate branch `night-shift/sitting-<date>` is built before the Sitting by the owner or
  the preparing Session with `tools/night_shift_candidate.sh --date <date> --push`; it is only
  what the robot ran, never what merges.

## 3. Return the live checkout and rebuild

1. `git status -sb` must read `## main...origin/main` plus only the files this command changed.
   If the Sitting left the checkout on `night-shift/sitting-<date>`, `git checkout main` first
   (after step 2's merges, which were done on `main`; if you find yourself on the candidate
   branch at the start, switch before step 2).
2. If any merged branch touched `ros_ws/`: `cd ros_ws && colcon build --packages-select
   jugglebot` (the interfaces and launch need the rebuild before the next Sitting) and say so.
3. `git add docs/records.md` if it changed (skill_node writes it live during the Sitting; it
   is committed at every Sitting close-out).

## 4. Follow-ups, logbook, Runbook, commit

1. For every note that names a new problem or idea, file it:
   `python -m mapkit item new <bug|feature|idea> "<title>" --body "<the note, with the Item and date>" --link map/<stem>.md --priority <p>` (no `--commit`; the files ride in the wrap-up commit).
   Rate it when you know the answer.
2. Write `logbook/<date>-sitting-wrapup.md` (short form: the results table, what merged with
   SHAs, what stayed held and why, follow-ups filed, the rebuild if any) and its
   `logbook/INDEX.md` row.
3. `python -m mapkit gen` (the Runbook page `docs/runbook.md` drops the passed rows and keeps
   the failed ones; the project-map page follows the status changes).
4. `./run_tests.sh` (a merged branch already passed the full gate on its own; the merged
   result gets the per-commit gate here, the nightly covers the rest tonight). Red: stop, say
   which test, do not commit.
5. One commit: `docs(sitting): wrap-up <date> - <n> passed, <m> merged, <k> held` with the
   trailers `Logbook-Entry: <date>-sitting-wrapup` and
   `Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>`; `git fetch && git status -sb`
   again; `git push origin main`. Then push nothing else.

## 5. Finish

Say, in three lines: results read (passed / failed), merged and closed (with SHAs), held
(with why). Offer the owner the Runbook link for the next Sitting and nothing else.

Pinned by `tests/sim/test_sitting_wrapup_protocol.py`: this file names every step the
Runbook's `## Close-out` promises (results read, per-Item merge, return to `main`, rebuild if
`ros_ws` changed, `docs/records.md`, follow-ups filed, the logbook entry, `mapkit gen`, the
single-writer check, no force-push, the gate re-run with `--waive c`).
