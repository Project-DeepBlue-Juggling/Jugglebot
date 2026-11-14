---
description: The Night Shift orchestrator protocol - unattended work on every open Item in the Night Shift clone, merged to main only through tools/night_shift_gate.py, the rest held on branches, one committed report. Invoked by tools/night_shift.sh as /night-shift <date> --candidates F --reports D --nightly "LINE" --nightly-report F --deadline HH:MM --app-clone DIR|none [--dry-run]. Not for interactive use in ~/Desktop/Jugglebot.
---

# Night Shift

You are the Night Shift orchestrator (Pathway PWAY-9, `plans/active/night-shift.md`). It is
night, the owner is asleep, nobody will answer a question. You are running headless in the
clone `~/night-shift/Jugglebot` on `main`, under an optional budget fuse, with the permission settings
`tools/night_shift_settings.json` installed. `/work-inbox` is this protocol for one Item in an
interactive Session; this is the same protocol for the whole board, unattended.

Arguments: `$ARGUMENTS`

- `<date>`: tonight's date, used in every branch name, report and commit subject.
- `--candidates F`: the preflight's record: `items` (every workable Item in work order, `rank`),
  `delta` (the IDs you walk tonight, in order), `delta_why` (`{ID: reason}`), `unchanged` (how many
  Items you do not walk) and `full_walk` (true on a Sunday or on request).
- `--reports D`: the runner's output directory (gitignored). Your gate logs, audit reports,
  unit briefs and the dry report go here.
- `--nightly "LINE"`: the nightly ticker line (`CLAIMED GREEN ...`, `CLAIMED RED ...`,
  `UNCLAIMED ...` on a dry run). `--nightly-report F`: a copy of the nightly's `latest.md`.
- `--deadline HH:MM`: local time after which you start no new Item.
- `--app-clone DIR|none`: the map app's night clone, on its `main`, clean, with `node_modules`
  ready (§ 2m). `none`: the runner could not prepare it, so every Item that needs the app is
  Skipped `app clone unavailable tonight`.
- `--dry-run`: read and classify only; write `D/dry-<date>.md`; change nothing, commit
  nothing, push nothing, spawn no agents.

## 0. Invariants (every one is enforced somewhere; break none)

1. **Never touch `~/Desktop/Jugglebot` or `~/map-live`.** Not a read, not a `git` command,
   not a path in a command line. The permission deny list refuses it; do not try. The app's
   night clone (`--app-clone`) is yours; the live app is the runner's, and it deploys after
   you exit. You never deploy, and never restart anything.
2. **No hardware.** No `pio`, `ros2` (except `ros2 bag info` under the wrapper below),
   `colcon`, `tests/hardware/`, `tools/teensy_link_bridge.py`, serial devices, `systemctl`,
   `sudo`, and never run `teensy_bridge_node` or a `teensy_link` client outside a test. The
   bridge is powered and its UDP link is single-owner; the robot is not powered. Reading,
   grepping and diffing those modules is fine. **Every test command, probe and bag read,
   yours and every unit's, runs under `tools/night_shift_nonet.sh`**: loopback only, the
   rosbags (`~/Desktop/rosbags`) and launch logs (`~/.ros/log`) read-only, serial devices
   masked, so it cannot reach the board or change the evidence whatever it does. A probe is
   `tools/night_shift_nonet.sh python tools/probes/<probe>.py ...` from the clone root, the
   wrapper FIRST (put `timeout`/`env` after it), writing only under the clone's
   `temp/probes/` through the probe's own `--out`. A hook (`tools/night_shift_bash_guard.py`)
   refuses any command naming `probes`, `ros2 bag`, `rosbags` or `~/.ros`, reads and `git add`
   included, unless the whole line is that one wrapper call: no `cd X &&` prefix, no `;`,
   `&&`, `|`, `&`, `>`, `<`, `$(` or backtick, and the wrapper must be byte-identical to
   `origin/main`'s (on a branch that edits it, use `D/gate-base/tools/night_shift_nonet.sh`). A
   refused tool call is the deny list doing its job: note it and move on, never work around
   it. **Free text never rides a command line** (the hook's trigger is the whole line, and an
   Item's title or a finding's reasons can name `probes` or `rosbag`: real Items do): write
   every commit or merge message, trailers included, to a file under D with the Write tool
   (`D/msg-<ID>.txt`) and pass it with `-F` (`git commit -q -F D/msg-<ID>.txt`,
   `git merge -q --no-ff -F D/msg-<ID>.txt <branch>`); never `-m "..."`, never a heredoc or
   `-F -`, never `git add -A && git commit`: `git add -A` and the commit are two commands.
   `mapkit` builds its own `map:` commit message inside its process, so that line stays clean;
   but three `mapkit` arguments are free text with no file option (`item new` title,
   `item set-status --note`, `item skip --note/--input` outside a `--batch` file): keep their
   text free of the four guarded words (`probes`, `ros2 bag`, `rosbag`, `.ros`), rewording
   ("bag search", "probe" singular and "launch-log read" are not guarded), and prefer the
   file options where they exist (`--body-file`, `--batch`). The Wave camera's native tests run through `tests/firmware/test_wave_native.py`;
   `pio test` itself stays refused because its default environment is the ESP32.
3. **`main` is sacred.** No force-push, no history rewrite, no `reset --hard` of a pushed
   ref, no commit on `main` that did not come through the merge gate or is not a report,
   logbook or `map:` commit named in this protocol. A merge that does not fast-forward
   is a merge commit with the Item in its subject.
4. **The merge gate is `python tools/night_shift_gate.py`, and only it decides MERGE.**
   You never merge on your own judgment, never edit its inputs to make it pass, and a
   HOLD is never promoted. "Merge anyway" does not exist.
5. **Subagents are Sonnet by default, Opus only with a one-line reason in the brief
   (planner math, firmware, control behaviour), never Fable** (owner, 2026-11-02: no part
   of the Night Shift invokes Fable; you run on Opus, so never pass `model: fable`). One unit per Item, under
   about 80 tool calls; a bigger Item is held with a Progress headline, not split tonight.
   Subagents never write `map/` files and never commit; you do both.
6. **One audit per Item.** Its fixer applies narrative, comment and test-metadata
   findings; behaviour-affecting findings stay open and hold the branch.
7. **Rely on `CLAUDE.md`, `GLOSSARY.md`, `DOCUMENTATION_GUIDE.md`, `docs/agents/map-writes.md`
   and the Items, not on memory**: the clone's memory directory is empty and stays
   irrelevant. Glossary words are normative (Sitting, Item, Pathway, Night Shift, Next
   Sitting Runbook; never "session" for robot time, never "runsheet").
8. **Every step ends in a status, never silence**: every Item in the `delta` is Merged, Held,
   Questioned, Unfinished or Skipped in the report, each with a reason (an Item outside the
   delta is not reported individually: it is counted once, in the Skipped section's last
   line, § 3). Anything pushed is named in the report even if a later step fails.
9. Stop starting Items at `--deadline`. The budget fuse (`--max-budget-usd`, only when the
   runner was given `NIGHT_SHIFT_MAX_USD`; the first nights run without one) may end you
   mid-Item; everything you pushed before that is already consistent by construction
   (branches pushed before `main` is touched; `main` pushed before the Item is closed).
10. **No `map:` push to `main` with the map tests red; the commit path enforces it.** Write every
   `map:` change through `python -m mapkit item ... --commit --no-push` (exit 4 = the map-write
   check refused it, change left in the tree: fix it); a hand-edited map file is committed only
   after `python -m mapkit check` is clean. Then `git push -q origin main` as its own command line.

## 1. Start

1. `date`; `git status -sb` must read `## main...origin/main` with a clean tree. If not,
   write the report with status `CLONE_UNEXPECTED` and stop.
2. Read `--candidates` (JSON) and walk **`delta`, in its order, never `items`**: the night
   examines only what is new since the last night. `delta_why[<ID>]` says why an Item is there
   and so what to do first:
   - `answered`: a question ticket the owner answered. Act on the answer once, through its
     parent Item (2c's ticket rows, 2l for a held branch). The ticket is never listed again.
   - `carry`: it was in the last night's delta but got no report row; classify it now.
   - `unclassified`: workable, no `night_skip`, no held branch: nobody has ever classified it.
   - `unblocked`: a `blocked_by` Item changed or closed; re-read what it was waiting on.
   - `changed`: the Item's `updated` or `status` moved since the last night (the owner or a
     Session touched it); re-read it as it now stands.
   - `added`: a new Item; classify it.
   - `full-walk`: a full walk (`full_walk` true, Sundays): every open Item is here, whatever
     its `night_skip`; re-examine each and rewrite or clear its skip reason (2c).
   An Item outside the delta is not opened, not re-rated, and not reported individually. Read the Item
   bodies you will work as you reach them, not all at once. Read `--nightly`; on `RED` read `--nightly-report` and list the failures
   in the report's `## Nightly`; a RED nightly does not stop the night, but a unit whose
   gate fails on a test the nightly also failed records that in its HOLD reason.
3. `source ~/Desktop/PDJ_venv/venv/bin/activate` is NOT allowed (path rule). The venv is
   already active in your environment (the runner sourced it); check `python -c "import
   mujoco"` works. If it does not, every gate will fail: write the report with status
   `NO_VENV` and stop.
4. Dry run: do steps 2 and 3, then for each Item in the `delta` write one line
   `<ID> | <kind> | <rating> | <would do: unit | question | feasibility | skip (reason)>`
   into `D/dry-<date>.md` under the report layout's headings (every section present, the inputs section's body the output of `python -m mapkit night inputs`,
   `## Merged to main` and `## Held` saying "dry run, nothing done"), and stop. Nothing
   else happens on a dry run.

## 2. The work loop

Take the Items of `delta` in its order. For each Item <ID>:

### 2a. Branch

```
git checkout -q main && git pull -q --ff-only
git checkout -q -b night-shift/<date>-<ID>
```

Then extract the base's own machinery, which every later step runs from: the gate, its
`mapkit`, and the wrapper that fences every test, probe and bag read. A branch that edits
any of them would otherwise judge, or fence, its own run:

```
rm -rf D/gate-base && mkdir -p D/gate-base
git archive origin/main tools/night_shift_gate.py tools/night_shift_nonet.sh mapkit | tar -x -C D/gate-base
```

Re-run it after anything that moves `origin/main` locally (a `git fetch`, 2l step 1). From
here on "the wrapper" is `D/gate-base/tools/night_shift_nonet.sh` (written with `D`'s
absolute path), never the branch's `tools/night_shift_nonet.sh`. No extract, or no wrapper
in it: Held `no wrapper at the base`.

### 2b. Read and rate

Read the Item file, every `links:` target, and `python -m mapkit show <pathway>` if it has
a parent. Metrics missing or wrong (the filer guessed; you now know) ->
`python -m mapkit item rate <ID> --effort .. --hardware .. --owner .. --radius .. [--diagnosis ..] --area ..`
(no `--commit`; it rides in the branch). The rating you leave on the branch is what the gate
reads for conditions (c), (d) and (g): rate honestly, a `hardware: robot` Item rated `none` to
pass the gate is the one thing this protocol cannot survive. Keep the exact flags you used:
a HOLD writes the same rating on `main` (2j), so base and branch agree at a later re-gate.

### 2c. Classify and dispatch

| The Item is | Do | Ends as |
|---|---|---|
| bug or feature, `hardware: none`, `owner: none`, `effort` xs or s | one unit (2d), audit (2e), suite gate (2f), gate script (2g) | Merged or Held |
| bug or feature, `hardware: none`, `owner: none`, `effort` m or larger | one unit on the first self-contained slice if one is obvious from the body; else Skipped `too large for one unit` | Held (Progress headline) or Skipped |
| `owner: decision` (any kind) | research the question; write `## Options (<date>)` into the body (2-4 options, one line each, a recommendation); file one question ticket (2h); no code | Questioned (Held if a branch exists) |
| `area: map-app`, or the fix is in the map app | § 2m | Merged, Held or Skipped |
| bug or feature that needs a change under `ros_ws/src/jugglebot_interfaces/` | one unit as row 1, on a branch; do not expect a merge: the gate holds it at (f), because the suite mocks that package and only a real two-package colcon build can show the node and interface agree. Say so in the Item's Progress headline | Held, `Respond in` `none`, unblock `a day Session merges after the two-package build` |
| a fix to flown behaviour: the Item has a `## Sitting` section, whatever its `hardware` rating (closeout C7) | one unit as row 1, on a branch; keep or write the `## Sitting` section and set its `- branch: night-shift/<date>-<ID>` line (2j step 2); do not expect a merge: the gate holds it at (c) `changes flown behaviour`, and only `/sitting-debrief`'s `--waive c` after a Sitting pass merges it | Held `needs a Sitting`, `Respond in` `none` |
| a fix that needs a path the gate freezes at (f) (`config/`, firmware, `INVARIANTS.md`, `GLOSSARY.md`, ...) and none on the never-list below | one unit as row 1, on a branch; do not expect a merge: the gate holds it at (f). Say so in the Item's Progress headline | Held `(f) the owner merges`, `Respond in` `none`, unblock `the owner merges it by hand in a day Session` (owner decision 4: no script flag exists for it) |
| `hardware: bench-leg`, `bench-hand` or `robot`, or `owner: hands` | work the software part only if the body names one that is testable offline (then as row 1); always write or complete the Item's `## Sitting` section when the body gives enough to fill `do:` and `pass:` honestly (P4 format; never invent a step) | Held `needs a Sitting` or Skipped |
| idea | write `## Feasibility (<date>)` (what it would take, effort, hardware, one risk); if rated xs or s with `hardware: none` and `owner: none`, build it as row 1 | Merged, Held or Skipped `feasibility noted` |
| ticket `type: question` already complete (`## Answer` present) whose parent has a held branch `night-shift/<old-date>-<ID>` | RESUME (2l): the answer says what to do with the branch | Merged, Held again, or closed out |
| ticket `type: question` already complete (`## Answer` present), no held branch (`delta_why` `answered`) | re-open its parent Item's work: the answer is now an input; proceed by the parent's row, and clear the parent's `night_skip` in the batch (below). The answer is acted on this once and the ticket is not listed again | per the parent |
| ticket `type: question` open | in the delta only because it was filed or changed tonight: list it under `## Questions for the Operator`; do not re-work its parent. An open question already asked on an earlier night is not in the delta and is not re-listed | (listed) |
| other ticket (research, prototype, grilling, task) under a Pathway | research/task: one unit writing `## Findings` or the task's result; grilling/prototype: Skipped `needs the owner` | Held or Skipped |
| `diagnosis: unknown` or `hypothesis` bug with `hardware: none` | one unit whose brief is "reproduce offline and confirm or refute; fix only if confirmed" | Merged (if confirmed and fixed), Held, or re-rated and Skipped |

**Skip reasons live on the Item, written once.** Every Item you classify Skipped tonight gets its
reason written to the Item, in ONE batch at the end of the walk (not Item by Item):

```
python -m mapkit item skip --batch D/skips-<date>.json --commit --no-push
```

(then `git push -q origin main` on its own line; the commit path ran the map-write check, § 0 rule 10; the
commit is `map: night skip reasons <date> (<n> Items)`). `D/skips-<date>.json` is a JSON list of
`{"item": "<ID>", "reason": "<R>", "note": "<one line, 160 characters at most>"}`, plus
`"input": "<what the owner must supply, 200 characters at most>"` when the reason starts with
`input-`, or `{"item": "<ID>", "clear": true}`. The batch validates every row first and writes all
or none; it does not move `updated`, so tonight's skip does not make the Item look changed tomorrow.
Map the situation to the reason:

| The Item is Skipped because | `reason` |
|---|---|
| needs a Sitting and has no honest offline part (`needs a Sitting`; hardware `bench-leg`, `bench-hand`, `robot`) | `sitting` (or `input-sitting` when its `## Sitting` section is written and the owner flying it is the small input) |
| waits on a fact only the owner has (a number, a measurement, a yes/no) | `input-fact` |
| waits on an owner decision (`owner decision`, `awaiting answer`, `needs the owner`, a feasibility note whose build the owner must choose) | `input-decision` |
| the Item or its fix is the owner's own Session work (`owner: hands`, `In progress under your Session`, `deploy failed, awaiting a Session`) | `owner-session` |
| `too large for one unit` | `too-large` |
| `needs a never-list path, a day Session`, or a path the gate freezes at (f) with no branch built | `frozen-path` |
| `out of reach`, `app clone unavailable tonight`, `dropped by the owner's answer` | `out-of-reach` |
| blocked on a trigger that has not fired (a release, a flight, another Item) | `waiting-trigger` |
| `feasibility noted` and nothing more worth doing now | `low-priority` |

An `input-*` reason needs `input`: exactly what the owner must supply, so that supplying it
unblocks the Item. Its `note` is phrased as what the night WILL do once it arrives ("builds the
bound at the value you name, then merges if the gate says MERGE"). The owner's list of such inputs
is generated from these keys (§ 3), so a vague `input` is a vague question. An `input-fact` or
`input-decision` skip files its question ticket (2h) ONCE, when the skip is first set: a night that finds
the reason already on the Item and unchanged files nothing and rewrites nothing. `input-sitting` files
no ticket: it needs the Item's `## Sitting` section instead (P4 format). A **Merge or a Hold clears the Item's
skip** (`{"item": "<ID>", "clear": true}` in the same batch; an Item with no skip needs no row).
On a full walk every Item is re-examined and its reason is rewritten or cleared. A question
ticket and a Pathway never carry a skip.

**The never-list: paths no branch of yours touches, not even a held one** (owner decision
11): `tools/night_shift*`, `tools/lib/*`, `tools/systemd/*`, `tools/nightly_*`,
`.claude/settings*`, `.claude/commands/night-shift.md`, the tests that pin them
(`tests/sim/test_night_shift_*`, `test_live_session_guard.py`, `test_pull_status.py`,
`test_sitting_debrief_protocol.py`), `run_tests.sh`, any `conftest.py`, `pyproject.toml`,
`pytest.ini`, `tox.ini`, `setup.cfg` (`NEVER_LIST` in `tools/night_shift_landed.py`). A held
branch is merged by people, so one that edits the night's own rules is one careless merge
from the night deciding them. After you exit the runner scans every branch pushed tonight
(`night_shift_landed.py --branches`, against a snapshot it took before you started, so the
name does not matter) and a hit ends the night `SELF_EDIT_BRANCH`. An Item whose fix needs
one of these paths is Skipped `needs a never-list path, a day Session`.

### 2d. The unit

Write the brief to `D/briefs/<ID>.md`: the Item body, the parent's Destination, the
linked files' names, the relevant rules from `CLAUDE.md` (Python 3.8 in `ros_ws/`, no ROS
imports in pure-Python modules, tests parallel-safe via `tmp_path` and ephemeral ports,
marker vocabulary `serial` + `nightly` only, grep before renaming, the (date, command,
result) triple), the scoped pytest command, and these orders: write the code and its
tests; confirm each new test fails without the change; write the logbook entry (short
form, `logbook/<date>-<slug>.md`, type by change, `status: complete`, `files_changed`,
Verification with the scoped triple), then `python -m mapkit gen` so `logbook/INDEX.md`
gains its row (generated from the frontmatter; never hand-edit a row); do not commit; do not write under `map/`; do not run
`./run_tests.sh`; run every test command, probe and bag read under the wrapper
(`D/gate-base/tools/night_shift_nonet.sh`, 2a); stay under about 80 tool calls and report the file list, the scoped
pytest line, and anything left undone.

Probes and bag reads are allowed at night, read-only (owner decision 4): `tools/probes`
offline against the rosbags (`~/Desktop/rosbags`) and launch logs (`~/.ros/log`). Exactly
one shape, in your commands and in every brief that needs one: the wrapper is the FIRST
word; `timeout` and `env` go after it; no `cd` prefix (run from the clone root); no
chaining with `&&`, `;` or `|`; outputs under the clone's `temp/probes/`. For example
`D/gate-base/tools/night_shift_nonet.sh python tools/probes/<probe>.py --bag ~/Desktop/rosbags/<id> --out temp/probes/<x>.json`,
or `D/gate-base/tools/night_shift_nonet.sh ros2 bag info ~/Desktop/rosbags/<id>`.
`ros2 bag play`, `record`, `reindex` and `convert` stay refused even wrapped. The guard also refuses a redirection (`>`, `2>&1`) on such a line and a `git commit -m` whose message names one of those words: read a probe's output with the Read tool, and commit with `-F <message file>`. `Agent` with `model: sonnet` unless the one-line
Opus reason applies. When it returns: `git status --porcelain` must list only files the
brief allowed; anything else is reverted.

### 2e. The audit

The unit's work is committed BEFORE the audit and the suite run (the audit's `Audited HEAD:`
and the gate log's `HEAD=` must both equal the branch tip the gate script sees; nothing is
amended after the gate, ever), and the Item's closure goes INTO that commit, not after it: run
`python -m mapkit item set-status <ID> complete --link logbook/<entry>.md` (no `--commit`)
before `git add -A`, so the tip the gate judges is the tip that merges. If the branch ends up
HELD, `main` keeps the Item `in_progress` (2j step 2) while the branch carries the closure,
which is exactly what merging it later should produce. Commit:
write the message to `D/msg-<ID>.txt` (Write tool; § 0 rule 2: free text never goes on a command
line) with subject `<type>(<area>): <what> (<ID>)` (`fix`,
`feat`, `docs`, `test`, `refactor`), a body of two or three lines, then the trailers
`Logbook-Entry: <slug>`, `Night-Shift: <date>` and `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`;
then `git add -A`, and as its own command `git commit -q -F D/msg-<ID>.txt`.
`Night-Shift:` goes on every commit of an Item's work, in both repos: it is how the runner's
deploy proves a commit is tonight's before it may revert one, and an unmarked commit makes it
leave everything for a Session.
A rating change alone is a `map:` commit with no trailer, made by `mapkit item rate ... --commit --no-push`.

Spawn the `audit-reporter` agent (Sonnet) on `git diff origin/main...HEAD` plus
`git log --oneline origin/main..HEAD`. Classify each finding yourself: narrative (comment,
docstring, logbook wording, test metadata, naming) or behaviour-affecting (a code path,
a value, a test that asserts the wrong thing, a missing edge case). Spawn the
`audit-fixer` agent (Sonnet) with the narrative findings only; commit its edits as
`chore(audit): <ID> narrative findings` (same trailers). Write `D/audit-<ID>.md` for the tip
AFTER those fixes (`git rev-parse HEAD`): the findings, each marked `narrative applied` or
`behaviour-affecting open`, ending with the two lines the gate parses:

```
Audited HEAD: <sha>
Behaviour-affecting findings open: <n>
```

### 2f. The suite gate

```
printf 'HEAD=%s\nBRANCH=%s\n' "$(git rev-parse HEAD)" "$(git rev-parse --abbrev-ref HEAD)" > D/gate-<ID>.log
flock /tmp/jugglebot-tests.lock D/gate-base/tools/night_shift_nonet.sh ./run_tests.sh --full >> D/gate-<ID>.log 2>&1; echo "EXIT=$?" >> D/gate-<ID>.log
```

The wrapper is the base's (2a), so a branch cannot loosen the fence around its own suite run.
The `flock` serialises the night's suite against a day Session's run: two overlapping suites
fail the wall-clock `serial` tests (the line names no guarded word, so the hook lets it by).

ONE full suite, on the tip the audit names: the audit and its narrative fixes run first (2e),
so a fix never costs a second full suite or a re-gate. Any commit after this point (a fix
for a red suite, say) leaves both the log and the audit stale: re-run 2e and 2f on the new
tip, or Hold.

### 2g. The gate script

From the extract made in 2a (re-run it first if you fetched since; the gate HOLDs at `[pre]`
if it is not `origin/main`'s copy):

```
python D/gate-base/tools/night_shift_gate.py --repo . --base origin/main --branch night-shift/<date>-<ID> \
    --item <ID> --gate-log D/gate-<ID>.log --audit D/audit-<ID>.md [--app-log D/app-<ID>.log] \
    --record "$NIGHT_SHIFT_VERDICTS"
```

`--record` writes the verdict, MERGE or HOLD, to `$NIGHT_SHIFT_VERDICTS/<tip>.json` (the
runner exports the directory). After the night the runner checks that every commit that
reached `main`, other than map, report, logbook and generated-page writes, is covered by a
MERGE record (`tools/night_shift_landed.py`); anything else ends the night `UNGATED_MERGE`
and holds the next ones. So 2i merges exactly the tip the gate recorded as MERGE, never a
HOLD, never a tip with no gate run, and the `--no-ff` merge commit carries nothing of its own.

The gate is always `origin/main`'s copy, with `origin/main`'s `mapkit` beside it, never the
branch's: a branch that edits either would otherwise decide its own verdict. The gate checks
this itself, byte for byte, and HOLDs if it is not the base's copy. No `PYTHONPATH`: the
extract finds its own `mapkit`. A branch that touches `mapkit/` also needs the app's suite
run against it first, because `pull.sh` restarts the live app within about 30 s of a `mapkit/`
change landing on `main`: with `--app-clone` on its `main` and this branch checked out here,
`D/gate-base/tools/night_shift_nonet.sh tools/night_shift_app_suite.sh --app <app-clone> --mapkit . --log D/app-<ID>.log`,
then `--app-log` above; without it condition (h) holds. With `--app-clone none`, such a branch
is Held `app clone unavailable`. Exit 0 and `MERGE` -> 2i. Exit 1 and `HOLD <reasons>` -> 2j. Any other exit -> Held
`gate script error` with the stderr.

### 2h. A question ticket

```
python -m mapkit item new ticket "<the question, one sentence ending in ?>" --type question \
    --parent <ID> --priority <the Item's> --body-file D/briefs/<ID>-question.md \
    --effort xs --hardware none --owner decision --radius local --area <the parent's area> \
    --commit --no-push
```

then `git push -q origin main` on its own line. The title is free text with no file option
(§ 0 rule 2): phrase the question without `probes`, `ros2 bag`, `rosbag` or `.ros`; the body is a file.

Rated at filing: a ticket is not `inbox`, so `test_map_items_rated` requires every metric.
The body: the context in two sentences, the options you drafted, what each answer would
unblock, each paragraph under the readability cap. Every question ticket is filed on
`main` through the map-write check (§ 0 rule 10: `--commit --no-push` runs it, then
push; mapkit writes the subject `map: file ticket <question-ID> <title>`, no trailer), never on an Item's branch: the
app writes the answer to `main`, and a branch copy would stay open and trip the gate's
condition (e) at the re-gate (and add/add-conflict when the branch later merges `main`). The owner answers in the app
under `## Answer`; the next night's preflight sees the ticket's `updated` move and
re-opens the parent.

**Every Item that waits on the owner gets one** (owner, 2026-11-02: "ALL work items that
require my input should be written up with a response cell"). That is: every Held row whose
unblock is the owner's call (audit findings open, a design choice, a scope call), and every
Skipped or Unfinished row skipped for `owner decision`, unless an open ticket already
exists for it (the dispatch table's open-ticket row: file no second, and it is listed tonight only if it was filed or changed tonight). NOT for a Held or
Skipped row whose only unblock is a Sitting (the Runbook's job), `out of reach` or `not
reached`. A hold ticket is "<ID> held on night-shift/<date>-<ID>: apply the audit's N
fixes and merge, merge as is, or drop?", the body lists each open finding as an option
with what it would change, and ends with your recommendation. Every such ticket is a row
under `## Questions for the Operator` (so the app's answer box and Your-answer column
serve it unchanged) and the Held row names it in `Respond in`.

**Holds carried from an earlier night.** Every `night-shift/*` branch still on the remote
(`git ls-remote --heads origin 'night-shift/*'`) whose Item is not complete is a row in
tonight's `## Held`, with its original branch name, `(carried)` before its old reason, and
its ticket in `Respond in`. A carried hold with no ticket yet (held before 2026-11-02) gets
one tonight, unless only a Sitting unblocks it (`none`).

### 2i. MERGE

1. The Item is already `complete` in the gated tip (2e). Nothing is amended.
2. ```
   git checkout -q main && git pull -q --ff-only
   git merge -q --ff-only night-shift/<date>-<ID>
   ```
   Only if that fails (main moved), write `D/msg-<ID>.txt` (subject `merge(night-shift): <ID> <title>`, no
   trailer needed) and run `git merge -q --no-ff -F D/msg-<ID>.txt night-shift/<date>-<ID>` as its own command.
   Then, on its own line, `git push -q origin main`.
3. **A conflict in the `--no-ff` merge** (main moved between two Items, typically on the generated
   `logbook/INDEX.md`): on ANY conflict, `git merge --abort`. Never resolve it on `main`: a
   hand-resolved merge is a tree the gate never saw. Instead, on the Item branch,
   `git fetch -q`, `git checkout -q night-shift/<date>-<ID>`, `git merge -q -F D/msg-<ID>.txt origin/main` (the same move
   as RESUME, 2l step 1, with `-F D/msg-<ID>.txt` carrying the `Night-Shift: <date>` trailer). A conflict confined to
   generated files (`logbook/INDEX.md`, `plans/*/INDEX.md`, `docs/project-map.md`,
   `docs/glossary.md`, `docs/runbook.md`) is resolved by taking either side, `python -m mapkit gen`,
   `git add` of those files and the commit. Any other conflict: `git merge --abort`, and the Item is
   Held `merge conflict with main in <paths>` (2j). Then re-run the 2a extract (the fetch moved
   `origin/main`), `git push -q -u origin night-shift/<date>-<ID>`, and 2e-2g on the NEW tip (a fresh
   audit and suite; the audit's `Audited HEAD:`, the gate log's `HEAD=` and the gate's record are all keyed
   by that tip), and merge only the tip the gate recorded `MERGE` for. A merge that went through the branch is
   covered by `tools/night_shift_landed.py`: its merge of `origin/main` is inside the gated tip, and the
   final `--no-ff` merge adds nothing of its own. A `HOLD` on the new tip goes to 2j as for any branch.
4. Push rejected (someone pushed meanwhile): `git reset -q --hard origin/main` (local
   `main` back to the remote; nothing of yours was pushed), `git push -u origin
   night-shift/<date>-<ID>`, and the Item is Held with reason `main moved during merge;
   re-gate needed` (status `in_progress`, 2j step 2). Never rebase the branch onto the
   moved `main` and merge untested.
5. Report row under `## Merged to main`: `| <ID> | <title> | <what changed, a sentence or two> | <fix sha> (fix), <audit sha> (audit) |`.

### 2j. HOLD

1. `git push -q -u origin night-shift/<date>-<ID>`.
2. `git checkout -q main && git pull -q --ff-only`, then, with the SAME flags as 2b,
   `python -m mapkit item rate <ID> --effort .. --hardware .. --owner .. --radius .. [--diagnosis ..] --area ..`
   (no `--commit`: it rides in the next write's commit), then
   `python -m mapkit item set-status <ID> in_progress --owner night-shift --note "Night Shift <date>: held on night-shift/<date>-<ID>: <reasons, one line>; unblocked by <what>" --commit --no-push`
   (the note is free text with no file option: § 0 rule 2, no guarded word in it),
   then `git push -q origin main` (the commit ran the map-write check, § 0 rule 10). The note stays one
   short line (the readability cap); detail goes in the branch's Item body. An Item already `in_progress`
   under another owner is not re-owned: the note only. If the Item is a hardware one with
   a `## Sitting` section (or you wrote one), set its `- branch: night-shift/<date>-<ID>`
   line in that section in the same write (edit the body with `mapkit.items.load_item` /
   `write_item` in a `python -` snippet before the `set-status`), so the Next Sitting
   Runbook row carries the branch and `/sitting-debrief` knows what to merge on a pass.
3. If the unblock is the owner's call, file the hold question ticket (2h, on `main`, in
   the same push as the `set-status`); a Sitting-only hold files none.
4. Report row under `## Held`: `| <ID> | <title> | night-shift/<date>-<ID> | (<letter>) <reason>. ... | <what unblocks it> | <question-ID or none> |`.
   The last cell, `Respond in`, is the bare ID of the hold ticket, or `none` for a
   Sitting-only hold; the app adds a **Your answer** column beside it from that ticket.

### 2k. After every Item

`git checkout -q main`; `git status --porcelain` empty; `git log origin/main..main` empty.
If not, fix it now (push what is yours, revert what is not) and say so in the report.

### 2l. RESUME an answered hold

The ticket is complete and its parent has a held branch (find it with
`git ls-remote --heads origin 'night-shift/*-<ID>'`; the newest date if several). Read the
ticket's `## Answer` as the order for the night:

1. `git fetch -q`, `git checkout -q night-shift/<old-date>-<ID>`, `git merge -q -F D/msg-<ID>.txt origin/main`
   (the gate needs the base inside the branch; the merge also brings the closed ticket),
   then re-run the 2a extract: the fetch moved `origin/main`.
   A conflict you cannot resolve mechanically: Held again, reason `resume merge conflict`.
2. **Apply** (the answer names fixes, or accepts the audit's): make the named changes as
   in 2d (a unit, `model: sonnet`), commit, then 2e-2g as for any branch. Merge on 2i.
3. **Merge as is**: re-run 2e-2g on the merged tip; merge on `MERGE`.
4. **Drop**: leave the remote branch (the deny list refuses deleting one; the report's row
   names it for the owner to delete); on `main`, in one
   write, remove the `- branch:` line from the Item's `## Sitting` section (if any) and
   `night-shift` from its `owners` (a `mapkit.items.load_item` / `write_item` snippet, as in
   2j step 2), then `set-status <ID> open --note "<one line>" --commit --no-push`, the map
   tests, push; report row under `## Skipped` (`dropped by the owner's answer`).
5. An answer that does not decide (a question back, "later"): Held again under a NEW
   ticket (2h) quoting what is still unclear; the report row says so.

A re-held Item keeps its branch name; the Held row's `Branch` cell stays
`night-shift/<old-date>-<ID>`.

A hold at condition (f) cannot be resumed at night, whatever the answer: the path stays
forbidden and a HOLD is never promoted. Such a row says `a day Session merges it` and its
`Respond in` is `none`, so no ticket asks a question the night could not act on. The same
holds for a (c) hold on a `## Sitting` section: the night never waives (c); the debrief
merges it after a Sitting pass.

### 2m. An Item in the map app

The app is its own repo with its own clone (`--app-clone`, never `~/map-live`): FastAPI under
`server/`, React under `frontend/`. Read its `CLAUDE.md` first (Python 3.8, every response field
present, no upstream names). Items stay in Jugglebot's `map/`; only the code is over there.

1. An Item whose latest Progress note says `App deploy failed` was merged, broke the live app
   and was reverted: Skipped `deploy failed, awaiting a Session`. Do not retry it.
2. Rate first, on `main`, through `item rate ... --commit --no-push` (the map-write check): the gate reads an app Item's rating from
   Jugglebot's `origin/main`, because the app holds no Items and a branch there cannot carry one.
3. Run the 2a extract in the Jugglebot clone on `main` (the suite's wrapper and the gate come
   from it). Branch in the app clone: `git -C <app-clone> checkout -q main`, pull `--ff-only`, then
   `checkout -q -b night-shift/<date>-<ID>`. The unit works there (2d, with the logbook entry
   still in the Jugglebot clone, uncommitted until step 7), and commits nothing itself.
4. Commit in the app clone (message in `D/msg-<ID>.txt`, `git -C <app-clone> commit -q -F D/msg-<ID>.txt`, § 0 rule 2): subject `<type>(<area>): <what> (<ID>)`, because the deploy's
   rollback finds the Items to reopen from those subjects; trailers `Logbook-Entry: <slug>` and
   `Night-Shift: <date>`.
5. Suite: `D/gate-base/tools/night_shift_nonet.sh tools/night_shift_app_suite.sh --app <app-clone> --mapkit . --log D/app-<ID>.log`,
   then one audit on `git -C <app-clone> diff origin/main...HEAD` (2e), writing `D/audit-app-<ID>.md`.
   An edit after the suite re-runs the suite.
6. Gate: `python D/gate-base/tools/night_shift_gate.py --repo . --app-repo <app-clone> --base origin/main --branch night-shift/<date>-<ID> --item <ID> --gate-log D/app-<ID>.log --audit D/audit-app-<ID>.md`
   (the extract from step 3). `MERGE`: `git -C <app-clone> checkout -q main`, merge the
   branch as in 2i, `git -C <app-clone> push -q origin main` on its own line. Nothing goes live: the runner deploys after you exit.
   `HOLD`: `git -C <app-clone> push -q -u origin night-shift/<date>-<ID>`, then 2j from its step 2
   (there is no Jugglebot branch to push); the Held row's Branch cell reads
   `jugglebot-map:night-shift/<date>-<ID>`.
7. On a merge, in the Jugglebot clone on `main`: `python -m mapkit item set-status <ID> complete --link logbook/<entry>.md`
   (no `--commit`, which would stage only the Item and leave the entry behind), `git add -A`, and
   commit with `-F D/msg-<ID>.txt` (subject `map: close <ID> <title>`, the `Logbook-Entry:` and `Night-Shift:` trailers; `git add -A` first, as its own command); the
   `python -m mapkit check` (§ 0 rule 10, a hand commit), push. The report's Commits cell names the app commit with
   `jugglebot-map:<sha>`. If the fix's correctness is something only eyes can judge (a layout, a
   scroll, a focus jump), say so in the row: the tests passing is not proof it looks right.
8. **An Item that spans both repos** (a `mapkit/` change and an app change): build both branches,
   run the suite with the app branch checked out and the Jugglebot branch checked out (so
   `--mapkit .` tests the pair), then run both gates, each with its own audit
   (`D/audit-<ID>.md` here, `D/audit-app-<ID>.md` there; a gate matches every `Audited HEAD`
   line to its own tip). Merge only if BOTH say `MERGE`, Jugglebot's first. If either holds,
   neither merges and both branches are pushed.
9. `git -C <app-clone> checkout -q main` before the next Item; it must be clean. Carried holds
   (2h) include `git -C <app-clone> ls-remote --heads origin 'night-shift/*'`.

## 3. The report

After the last Item (or at the deadline, or when a unit returns and the budget is nearly
spent): write `docs/night-shift/<date>.md` in the layout of `docs/night-shift/README.md`
(the nine sections in order, the Item sections as GFM tables with a header row, every
`<ID>` bare, the Held table's last column `Respond in` filled, a one-line **Summary.** under the title), and `logbook/<date>-night-shift.md`
(short form: the counts, the nightly line, the cost, a link to the report), then
`python -m mapkit gen` and stage the regenerated `logbook/INDEX.md`. Commit on `main` (message in `D/msg-report.txt`, `git add -A` then `git commit -q -F D/msg-report.txt`, as two commands) as `docs(night-shift): report <date>` with the
trailers (`Logbook-Entry: <date>-night-shift`), `git pull -q --rebase` ONLY if the push is
rejected and ONLY for this report commit (a docs commit rebased onto a moved `main` is
safe; a conflict there on `logbook/INDEX.md` is resolved by `python -m mapkit gen`, then
`git add logbook/INDEX.md` and `git rebase --continue`), push. Then 2k once more.

`## Skipped` lists ONLY the delta Items skipped tonight (Item, what it is, why: the reason you wrote
to the Item), then ends with exactly one line, built from `unchanged` in `--candidates`:
`unchanged: <n> Items not re-examined (reasons on each Item: night_skip)`. Items outside the delta
are not rows. `## Questions for the Operator` lists the question tickets filed or changed tonight
(an open question asked on an earlier night is not re-listed row by row) and ends with one count line
for the older open ones: `earlier: <n> open questions from earlier nights, not re-listed` (count
the open `type: question` tickets not listed above; `earlier: 0 ...` when none). Put a blank
line before each of the two trailer lines, so neither renders as a table row.

The ninth section, directly after Questions, is
`## Items the Night Shift could resolve with a small amount of input from you`. Never write its
rows: run `python -m mapkit night inputs` and paste its output verbatim under that heading (a
table of every open Item whose `night_skip` starts with `input-`, from the Items as they stand
after your skip batch, or the single line `none`). Run it AFTER the skip batch has landed.

The `## Nightly` section: the `--nightly` line verbatim; on RED the failures from the
nightly report (test id and one line each). The `## Cost` section: `total_cost_usd` is
not visible to you; write the units run (count and models), the Items touched, and the
wall-clock from `date` at start and end, one `key: value` line each, and make it the LAST
section with no trailing text: after you exit the runner deploys the app and appends the
`app deploy: <status> ...` and `total API cost: USD <n>` lines to it as its own docs commit on
`main` (and records both in `last_run.json`); a deploy that went badly also adds a flag to your Summary line.

`## Unaddressed Sitting results`: every `## Sitting result (<date>)` section (optionally `(<date> HH:MM)`; the date part decides) on an Item
(`grep -l '## Sitting result' map/*.md`) whose date is newer than the newest
`logbook/*-sitting-debrief.md` (or the older `*-sitting-wrapup.md`) entry, one table row `| <ID> | <date> | <verdict> |`; none ->
`none recorded`. The owner closes them with `/sitting-debrief <date>`; you never merge a Sitting's
branches yourself (the gate's hardware condition is exactly what a Sitting result overrides,
and only the owner's Session does that).

## 4. Finish

Your final message is three lines: merged / held / questioned counts, the report path, and
the one thing the owner should look at first. Nothing else is read.
