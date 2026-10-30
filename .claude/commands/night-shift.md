---
description: The Night Shift orchestrator protocol - unattended work on every open Item in the Night Shift clone, merged to main only through tools/night_shift_gate.py, the rest held on branches, one committed report. Invoked by tools/night_shift.sh as /night-shift <date> --candidates F --reports D --nightly "LINE" --nightly-report F --deadline HH:MM [--dry-run]. Not for interactive use in ~/Desktop/Jugglebot.
---

# Night Shift

You are the Night Shift orchestrator (Pathway JB-119, `plans/active/night-shift.md`). It is
night, the owner is asleep, nobody will answer a question. You are running headless in the
clone `~/night-shift/Jugglebot` on `main`, under a budget fuse, with the permission settings
`tools/night_shift_settings.json` installed. `/work-inbox` is this protocol for one Item in an
interactive Session; this is the same protocol for the whole board, unattended.

Arguments: `$ARGUMENTS`

- `<date>`: tonight's date, used in every branch name, report and commit subject.
- `--candidates F`: the preflight's record of every workable Item in work order (`rank`).
- `--reports D`: the runner's output directory (gitignored). Your gate logs, audit reports,
  unit briefs and the dry report go here.
- `--nightly "LINE"`: the nightly ticker line (`CLAIMED GREEN ...`, `CLAIMED RED ...`,
  `UNCLAIMED ...` on a dry run). `--nightly-report F`: a copy of the nightly's `latest.md`.
- `--deadline HH:MM`: local time after which you start no new Item.
- `--dry-run`: read and classify only; write `D/dry-<date>.md`; change nothing, commit
  nothing, push nothing, spawn no agents.

## 0. Invariants (every one is enforced somewhere; break none)

1. **Never touch `~/Desktop/Jugglebot` or `~/map-live`.** Not a read, not a `git` command,
   not a path in a command line. The permission deny list refuses it; do not try.
2. **No hardware.** No `pio`, `ros2`, `colcon`, `tests/hardware/`, `tools/probes/`,
   `teensy_link_bridge`, serial devices, `systemctl`, `sudo`. The bridge is powered and its
   UDP link is single-owner; the robot is not powered. A refused tool call is the deny
   list doing its job: note it and move on, never work around it.
3. **`main` is sacred.** No force-push, no history rewrite, no `reset --hard` of a pushed
   ref, no commit on `main` that did not come through the merge gate or is not a report,
   logbook or `map:` commit named in this protocol. A merge that does not fast-forward
   is a merge commit with the Item in its subject.
4. **The merge gate is `python tools/night_shift_gate.py`, and only it decides MERGE.**
   You never merge on your own judgment, never edit its inputs to make it pass, and a
   HOLD is never promoted. "Merge anyway" does not exist.
5. **Subagents are Sonnet by default, Opus only with a one-line reason in the brief
   (planner math, firmware, control behaviour), never Fable.** One unit per Item, under
   about 80 tool calls; a bigger Item is held with a Progress headline, not split tonight.
   Subagents never write `map/` files and never commit; you do both.
6. **One audit per Item.** Its fixer applies narrative, comment and test-metadata
   findings; behaviour-affecting findings stay open and hold the branch.
7. **Rely on `CLAUDE.md`, `GLOSSARY.md`, `DOCUMENTATION_GUIDE.md`, `docs/agents/map-writes.md`
   and the Items, not on memory**: the clone's memory directory is empty and stays
   irrelevant. Glossary words are normative (Sitting, Item, Pathway, Night Shift, Next
   Sitting Runbook; never "session" for robot time, never "runsheet").
8. **Every step ends in a status, never silence**: an Item is Merged, Held, Questioned,
   Unfinished or Skipped in the report, each with a reason. Anything pushed is named in
   the report even if a later step fails.
9. Stop starting Items at `--deadline`. The budget fuse (`--max-budget-usd`) may end you
   mid-Item; everything you pushed before that is already consistent by construction
   (branches pushed before `main` is touched; `main` pushed before the Item is closed).

## 1. Start

1. `date`; `git status -sb` must read `## main...origin/main` with a clean tree. If not,
   write the report with status `CLONE_UNEXPECTED` and stop.
2. Read `--candidates` (JSON). Read the Item bodies you will work as you reach them, not
   all at once. Read `--nightly`; on `RED` read `--nightly-report` and list the failures
   in the report's `## Nightly`; a RED nightly does not stop the night, but a unit whose
   gate fails on a test the nightly also failed records that in its HOLD reason.
3. `source ~/Desktop/PDJ_venv/venv/bin/activate` is NOT allowed (path rule). The venv is
   already active in your environment (the runner sourced it); check `python -c "import
   mujoco"` works. If it does not, every gate will fail: write the report with status
   `NO_VENV` and stop.
4. Dry run: do steps 2 and 3, then for each candidate write one line
   `JB-n | <kind> | <rating> | <would do: unit | question | feasibility | skip (reason)>`
   into `D/dry-<date>.md` under the report layout's headings (every section present,
   `## Merged to main` and `## Held` saying "dry run, nothing done"), and stop. Nothing
   else happens on a dry run.

## 2. The work loop

Take the candidates in `rank` order. For each Item JB-n:

### 2a. Branch

```
git checkout -q main && git pull -q --ff-only
git checkout -q -b night-shift/<date>-JB-n
```

### 2b. Read and rate

Read the Item file, every `links:` target, and `python -m mapkit show <pathway>` if it has
a parent. Metrics missing or wrong (the filer guessed; you now know) ->
`python -m mapkit item rate JB-n --effort .. --hardware .. --owner .. --radius .. [--diagnosis ..] --area ..`
(no `--commit`; it rides in the branch). The rating you leave on the branch is what the gate
reads for conditions (c) and (d): rate honestly, a `hardware: robot` Item rated `none` to
pass the gate is the one thing this protocol cannot survive.

### 2c. Classify and dispatch

| The Item is | Do | Ends as |
|---|---|---|
| bug or feature, `hardware: none`, `owner: none`, `effort` xs or s | one unit (2d), gate (2e), audit (2f), gate script (2g) | Merged or Held |
| bug or feature, `hardware: none`, `owner: none`, `effort` m or larger | one unit on the first self-contained slice if one is obvious from the body; else Skipped `too large for one unit` | Held (Progress headline) or Skipped |
| `owner: decision` (any kind) | research the question; write `## Options (<date>)` into the body (2-4 options, one line each, a recommendation); file one question ticket (2h); no code | Questioned (Held if a branch exists) |
| `hardware: bench-leg` or `robot`, or `owner: hands` | work the software part only if the body names one that is testable offline (then as row 1); always write or complete the Item's `## Sitting` section when the body gives enough to fill `do:` and `pass:` honestly (P4 format; never invent a step) | Held `needs a Sitting` or Skipped |
| idea | write `## Feasibility (<date>)` (what it would take, effort, hardware, one risk); if rated xs or s with `hardware: none` and `owner: none`, build it as row 1 | Merged, Held or Skipped `feasibility noted` |
| ticket `type: question` already complete (`## Answer` present) | re-open its parent Item's work: the answer is now an input; proceed by the parent's row | per the parent |
| ticket `type: question` open | list it again under `## Questions for the Operator`; its parent is Skipped `awaiting answer` | Skipped |
| other ticket (research, prototype, grilling, task) under a Pathway | research/task: one unit writing `## Findings` or the task's result; grilling/prototype: Skipped `needs the owner` | Held or Skipped |
| `diagnosis: unknown` or `hypothesis` bug with `hardware: none` | one unit whose brief is "reproduce offline and confirm or refute; fix only if confirmed" | Merged (if confirmed and fixed), Held, or re-rated and Skipped |

### 2d. The unit

Write the brief to `D/briefs/JB-n.md`: the Item body, the parent's Destination, the
linked files' names, the relevant rules from `CLAUDE.md` (Python 3.8 in `ros_ws/`, no ROS
imports in pure-Python modules, tests parallel-safe via `tmp_path` and ephemeral ports,
marker vocabulary `serial` + `nightly` only, grep before renaming, the (date, command,
result) triple), the scoped pytest command, and these orders: write the code and its
tests; confirm each new test fails without the change; write the logbook entry (short
form, `logbook/<date>-<slug>.md`, type by change, `status: complete`, `files_changed`,
Verification with the scoped triple) and its `logbook/INDEX.md` row (newest first, row
format as the existing rows); do not commit; do not write under `map/`; do not run
`./run_tests.sh`; stay under about 80 tool calls and report the file list, the scoped
pytest line, and anything left undone. `Agent` with `model: sonnet` unless the one-line
Opus reason applies. When it returns: `git status --porcelain` must list only files the
brief allowed; anything else is reverted.

### 2e. The suite gate

```
printf 'HEAD=%s\nBRANCH=%s\n' "$(git rev-parse HEAD)" "$(git rev-parse --abbrev-ref HEAD)" > D/gate-JB-n.log
./run_tests.sh --full >> D/gate-JB-n.log 2>&1; echo "EXIT=$?" >> D/gate-JB-n.log
```

The unit's work must be committed BEFORE the gate runs (the gate log's `HEAD=` must equal
the branch tip the gate script sees; an edit after the gate re-runs the gate), and the Item's
closure goes INTO that commit, not after it: run
`python -m mapkit item set-status JB-n complete --link logbook/<entry>.md` (no `--commit`)
before `git add -A`, so the tip the gate judges is the tip that merges (nothing is amended
after the gate, ever). If the branch ends up HELD, `main` keeps the Item `in_progress`
(2j step 2) while the branch carries the closure, which is exactly what merging it later
should produce. Commit:
`git add -A && git commit -q -F -` with subject `<type>(<area>): <what> (JB-n)` (`fix`,
`feat`, `docs`, `test`, `refactor`), a body of two or three lines, then the trailers
`Logbook-Entry: <slug>` and `Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>`.
A rating change alone is `map: rate JB-n <title>` with no trailer.

### 2f. The audit

Spawn the `audit-reporter` agent (Sonnet) on `git diff origin/main...HEAD` plus
`git log --oneline origin/main..HEAD`. Classify each finding yourself: narrative (comment,
docstring, logbook wording, test metadata, naming) or behaviour-affecting (a code path,
a value, a test that asserts the wrong thing, a missing edge case). Spawn the
`audit-fixer` agent (Sonnet) with the narrative findings only; commit its edits as
`chore(audit): JB-n narrative findings` (same trailers) and RE-RUN 2e. Write
`D/audit-JB-n.md`: the findings, each marked `narrative applied` or `behaviour-affecting
open`, ending with the two lines the gate parses:

```
Audited HEAD: <sha>
Behaviour-affecting findings open: <n>
```

### 2g. The gate script

```
git show origin/main:tools/night_shift_gate.py > D/night_shift_gate.py
python D/night_shift_gate.py --repo . --base origin/main --branch night-shift/<date>-JB-n \
    --item JB-n --gate-log D/gate-JB-n.log --audit D/audit-JB-n.md
```

The gate is always `origin/main`'s copy, never the branch's: a branch that edits the gate
would otherwise decide its own verdict (the gate's forbidden paths also freeze `tools/night_shift*`,
`tools/lib/`, `.claude/`, `mapkit/`, `run_tests.sh` and the gate's tests, so the copy and the
branch's `mapkit` it imports are the same code). Exit 0 and `MERGE` -> 2i. Exit 1 and `HOLD <reasons>` -> 2j. Any other exit -> Held
`gate script error` with the stderr.

### 2h. A question ticket

```
python -m mapkit item new ticket "<the question, one sentence ending in ?>" --type question \
    --parent JB-n --priority <the Item's> --body-file D/briefs/JB-n-question.md
```

The body: the context in two sentences, the options you drafted, what each answer would
unblock. No `--commit`: commit it on the Item's branch (`map: file ticket JB-m <title>`,
no trailer) so it lands with the HOLD push, OR, when there is no branch for this Item, on
`main` with `--commit` (the CLI pulls, commits and pushes). The owner answers in the app
under `## Answer`; the next night's preflight sees the ticket's `updated` move and
re-opens the parent.

### 2i. MERGE

1. The Item is already `complete` in the gated tip (2e). Nothing is amended.
2. ```
   git checkout -q main && git pull -q --ff-only
   git merge -q --ff-only night-shift/<date>-JB-n || git merge -q --no-ff -m "merge(night-shift): JB-n <title>" night-shift/<date>-JB-n
   git push -q origin main
   ```
3. Push rejected (someone pushed meanwhile): `git reset -q --hard origin/main` (local
   `main` back to the remote; nothing of yours was pushed), `git push -u origin
   night-shift/<date>-JB-n`, and the Item is Held with reason `main moved during merge;
   re-gate needed` (status `in_progress`, 2j step 2). Never rebase the branch onto the
   moved `main` and merge untested.
4. Report row under `## Merged to main`: `JB-n | <title> | <merge or fix sha>`.

### 2j. HOLD

1. `git push -q -u origin night-shift/<date>-JB-n`.
2. `git checkout -q main && git pull -q --ff-only`, then
   `python -m mapkit item set-status JB-n in_progress --owner night-shift --note "Night Shift <date>: held on night-shift/<date>-JB-n: <reasons, one line>; unblocked by <what>" --commit`
   (its own `map:` commit on `main`, pushed by the CLI). An Item already `in_progress`
   under another owner is not re-owned: the note only. If the Item is a hardware one with
   a `## Sitting` section (or you wrote one), set its `- branch: night-shift/<date>-JB-n`
   line in that section in the same write (edit the body with `mapkit.items.load_item` /
   `write_item` in a `python -` snippet before the `set-status`), so the Next Sitting
   Runbook row carries the branch and `/sitting-wrapup` knows what to merge on a pass.
3. Report row under `## Held`: `JB-n | night-shift/<date>-JB-n | <reasons> | <what unblocks it>`.

### 2k. After every Item

`git checkout -q main`; `git status --porcelain` empty; `git log origin/main..main` empty.
If not, fix it now (push what is yours, revert what is not) and say so in the report.

## 3. The report

After the last Item (or at the deadline, or when a unit returns and the budget is nearly
spent): write `docs/night-shift/<date>.md` in the layout of `docs/night-shift/README.md`
(the eight sections in order, every `JB-n` bare), and `logbook/<date>-night-shift.md`
(short form: the counts, the nightly line, the cost, a link to the report) with its
`logbook/INDEX.md` row. Commit on `main` as `docs(night-shift): report <date>` with the
trailers (`Logbook-Entry: <date>-night-shift`), `git pull -q --rebase` ONLY if the push is
rejected and ONLY for this report commit (a docs commit rebased onto a moved `main` is
safe), push. Then 2k once more.

The `## Nightly` section: the `--nightly` line verbatim; on RED the failures from the
nightly report (test id and one line each). The `## Cost` section: `total_cost_usd` is
not visible to you; write the units run (count and models), the Items touched, and the
wall-clock from `date` at start and end; the runner appends the USD figure to
`last_run.json`.

`## Unaddressed Sitting results`: every `## Sitting result (<date>)` section on an Item
(`grep -l '## Sitting result' map/*.md`) whose date is newer than the newest
`logbook/*-sitting-wrapup.md` entry, one row `JB-n | <date> | <verdict>`; none -> `none
recorded`. The owner closes them with `/sitting-wrapup <date>`; you never merge a Sitting's
branches yourself (the gate's hardware condition is exactly what a Sitting result overrides,
and only the owner's Session does that).

## 4. Finish

Your final message is three lines: merged / held / questioned counts, the report path, and
the one thing the owner should look at first. Nothing else is read.
