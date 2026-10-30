---
title: Night Shift - unattended overnight work on the board, and the Next Sitting Runbook
created: 2026-10-30
status: active
owner: Harrison
last_updated: 2026-10-30
pathway: night-shift
branch: main
scope: >-
  An unattended Claude Code run after the 04:00 nightly suite that walks every open Item, works what
  Claude can finish alone, merges only what passes a deterministic merge gate, holds the rest on
  branches and writes one committed report with the operator's questions as answerable tickets; then
  the Next Sitting Runbook, one ordered page of every hardware check waiting across all Pathways,
  generated from the Items, with results typed in the map app and a wrap-up Session that merges.
steps:
  - {id: P1, title: Runner skeleton and dry run, kind: phase, status: not_started, effort: m, hardware: none}
  - {id: P2, title: "Work loop, merge gate, report and ticker", kind: phase, status: not_started, effort: m, hardware: none}
  - {id: P3, title: Night Shift page in the map app, kind: phase, status: not_started, effort: m, hardware: none}
  - {id: P4, title: Next Sitting Runbook, kind: phase, status: not_started, effort: m, hardware: none}
  - {id: P5, title: Sitting wrap-up command, kind: phase, status: not_started, effort: s, hardware: none}
  - {id: R6, title: First Runbook-driven Sitting, kind: rung, status: not_started, effort: s, hardware: robot}
related_logbook:
  - 2026-10-30-night-shift-design-and-plan.md
related_code:
  - tools/nightly_suite.sh (live-session guard, status line, ticker raise)
  - tools/nightly_ticker.sh
  - mapkit/itemcli.py, mapkit/items.py, mapkit/gen_docs.py
  - .claude/commands/work-inbox.md (the per-Item protocol Night Shift reuses)
  - docs/agents/map-writes.md
  - ~/map-live/jugglebot-map (server/app.py, server/gitwriter.py, server/nightly.py, frontend/src/features/nightly)
---

# Night Shift

Branch `main`. The schedulable form of Pathway JB-119; the design decisions are recorded in that Item's body and in `logbook/2026-10-30-night-shift-design-and-plan.md`. Steps P1 to P5 are software only and are meant to land in one orchestrated Session so the first run happens the following night; R6 is the first Sitting flown from the Runbook.

## 1. Context

- The board holds 57 open Items (2026-10-30); 10 fit the break-sized query the owner approved on 2026-10-28 (effort XS or S, hardware none, owner none, bugs confirmed), 27 need a Sitting or the bench leg, 4 wait on an owner decision, 6 are unrated. Items are filed faster than Sessions clear them.
- The owner has API credits unused by anything else. Claude Code 2.1.314 is installed; its `--help` lists `-p`, `--max-budget-usd`, `--model`, `--effort`, `--output-format json` and `--dangerously-skip-permissions` (checked 2026-10-30). Whether an API key bills the Console credits is confirmed in P1.
- A Sitting today costs the owner non-hardware steps (suite runs, git) that a Session can do. Hardware checks waiting across Pathways have no single page; each plan carries its own Runbook under `tests/hardware/`.
- Prerequisites in place: the metrics on every Item, the `## Answer` convention on tickets, the map app's git writer (commits `map/` edits to `main` from its own clone), the nightly's live-session guard and ticker, the suite flock shared with the auto-pull.

## 2. Architecture

### Current

```
04:00 jugglebot-nightly.timer -> nightly_suite.sh (main checkout, Nice 10)
        -> ./run_tests.sh --full --hypothesis-profile=ci-deep
        -> temp/reports/nightly/{status,latest.md}; ticker RAISED
first Session of the day: nightly_ticker.sh check -> CLAIMED -> reads latest.md
owner: /work-inbox by hand, one Item at a time, in an interactive Session
map app (:8765, own clone ~/map-live/Jugglebot, pulls every 30 s): board, drawer, nightly chip
```

### Proposed

```
04:00 nightly (unchanged) ---------------------------------------------+
04:45 jugglebot-night-shift.timer -> tools/night_shift.sh              |
   clone ~/night-shift/Jugglebot (own clone, main, ff from origin)      |
   guards: live_session() from nightly_suite.sh; temp/night-shift.hold; |
           suite flock via run_tests.sh                                  |
   preflight: python -m mapkit items --open --json -> candidates.json;  |
           exit 0 with "nothing new" if the set equals last night's      |
   claude -p "/night-shift <date>" --model <fable> --max-budget-usd N    |
       --dangerously-skip-permissions --output-format json              |
       orchestrator: claims ticker <--------------------------------------+
                     per Item: branch night-shift/<date>-JB-n, re-rate, one unit
                     (Sonnet default, Opus when the brief says why), ./run_tests.sh --full,
                     /audit, python tools/night_shift_gate.py -> MERGE | HOLD
                     MERGE: ff/merge into main, push; HOLD: push the branch
                     questions: mapkit item new ticket --type question --parent JB-n
                     report: docs/night-shift/<date>.md (committed, pushed)
   auto-pull (2 min) -> live checkout; map-pull (30 s) -> app clone
map app: /night-shift page (reports, open question tickets with inline answer boxes)
         /next-sitting page (docs/runbook.md rows, result boxes -> Item body)
Sitting: checkout night-shift/sitting-<date> -> fly rows -> /sitting-wrapup Session merges
```

### New files, conventions and formats

| Artefact | Form |
|---|---|
| `tools/night_shift.sh` | runner: guards, preflight, `claude -p`, exit summary to `temp/reports/night-shift/last_run.json` |
| `tools/systemd/jugglebot-night-shift.{service,timer}` | user units; `OnCalendar=*-*-* 04:45`, `After=jugglebot-nightly.service`, `Nice=10`, `EnvironmentFile=%h/.config/jugglebot/night-shift.env` (mode 600, holds `ANTHROPIC_API_KEY`) |
| `.claude/commands/night-shift.md` | the orchestrator protocol (what `/work-inbox` is for one Item, for the whole board, unattended) |
| `tools/night_shift_gate.py` | deterministic merge gate; input: branch, Item ID, gate log path, audit report path; output `MERGE` or `HOLD <reasons>`; exit 0/1 |
| `tools/night_shift_candidate.sh` | stacks held hardware branches into `night-shift/sitting-<date>` |
| `docs/night-shift/<date>.md` | the report (committed) |
| `mapkit/sitting.py`, `python -m mapkit runbook` | `## Sitting` section parser and `docs/runbook.md` generator (P4) |
| `.claude/commands/sitting-wrapup.md` | the morning Session's command (P5) |
| ticket `type: question` | new enum value; answered under `## Answer` |

Report layout (`docs/night-shift/<date>.md`), sections in this order: `# Night Shift <date>`; `## Nightly` (the ticker line and, on RED, the failures from `latest.md`); `## Merged to main` (JB-n, title, commit); `## Held` (JB-n, branch, reason, what unblocks it); `## Questions for the Operator` (ticket JB-m, one line each, answered in the app); `## Unfinished` (branch, Progress headline); `## Skipped` (JB-n, one-line reason); `## Cost` (USD, duration, units run). Every `JB-n` is bare so the app's drawer links it.

Candidate record (`candidates.json`, written by the preflight, read by the orchestrator and diffed against last night's): `{"date": "2026-10-31", "items": [{"id": "JB-21", "stem": "bug-...", "status": "inbox", "updated": "...", "effort": "s", "hardware": "none", "owner": "none", "radius": "subsystem", "diagnosis": "confirmed"}]}`. "Nothing new" means the same `id` set with the same `updated` stamps and no open question ticket answered since the last run.

Merge gate rule (code, not judgment): MERGE requires all of (a) the full suite gate exit 0 on the branch, logged; (b) an audit report with no behaviour-affecting finding left open; (c) the orchestrator's re-rating `hardware: none`; (d) `radius` not `normative`; (e) no open `question` ticket whose `parent` is the Item; (f) the branch touches none of `Teensy_code_canbridge/`, `config/*.yaml`, `GLOSSARY.md`, `CLAUDE.md`, `DOCUMENTATION_GUIDE.md`. Anything else is HOLD with every failing reason listed.

### What stays the same

The nightly suite and its status line; the ticker mechanism (Night Shift becomes its usual claimant); `/work-inbox` for interactive use; the map-writes recipe; the single-writer rule for `~/Desktop/Jugglebot` (Night Shift never touches it); `tests/hardware/` Runbooks as reference documents.

## 3. Implementation Phase Summary

| Phase | Scope | Status | Date | Risk | Validates |
|-------|-------|--------|------|------|-----------|
| P1 | Runner skeleton and dry run | NOT STARTED | | low | timer fires, guards hold, billing confirmed, a report with no edits |
| P2 | Work loop, merge gate, report and ticker | NOT STARTED | | medium | first real night: merges only through the gate |
| P3 | Night Shift page in the map app | NOT STARTED | | low | reports render, a question is answered in two clicks |
| P4 | Next Sitting Runbook | NOT STARTED | | medium | `## Sitting` rows generate one ordered page; results write back |
| P5 | Sitting wrap-up command | NOT STARTED | | low | a Sitting ends with one prompt and no owner git |
| R6 | First Runbook-driven Sitting | NOT STARTED | | medium | a held branch is flown, passed and merged by the wrap-up |

## 4. Implementation Phases

### P1: Runner skeleton and dry run

- **Files**: `tools/night_shift.sh`, `tools/systemd/jugglebot-night-shift.service`, `tools/systemd/jugglebot-night-shift.timer`, `tools/systemd/README.md` (install block, env file), `.claude/commands/night-shift.md` (dry-run protocol only), `GLOSSARY.md` (Night Shift, Next Sitting Runbook, via the domain-modeling skill without a grilling round, owner 2026-10-30), `tests/sim/test_night_shift_runner.py`.
- **Scope**: the clone at `~/night-shift/Jugglebot` (created by the runner if absent, `git pull --ff-only` each run, never a branch switch); the live-session guard factored out of `nightly_suite.sh` into `tools/lib/live_session.sh` and sourced by both (the regex and the DEFER semantics unchanged); the hold file `~/Desktop/Jugglebot/temp/night-shift.hold` (present means exit 0 with `HELD`); the preflight and `candidates.json`; `--dry-run` which runs `claude -p` with the protocol's dry-run mode: read, classify, write the report to `temp/reports/night-shift/dry-<date>.md`, change nothing, push nothing.
- **Critical details**: the robot is never powered overnight but the bridge is, so the clone's `.claude/settings.local.json` denies `Bash(pio run*)`, `Bash(ros2 launch*)`, `Bash(python tests/hardware/*)` and `Bash(*teensy_link_bridge*)`; P1 verifies those deny rules hold under `--dangerously-skip-permissions`, else the runner uses `--permission-mode acceptEdits` plus an allowlist and the protocol says so. Billing: the first dry run's `total_cost_usd` in the JSON result is checked against the Console usage page. Memory: the clone has its own memory directory (keyed by path); the protocol tells the orchestrator to rely on CLAUDE.md and the Items, not on memory. Budget fuse: `NIGHT_SHIFT_MAX_USD` default 90, passed as `--max-budget-usd`; not an Item cap.
- **Dependencies**: none.

### P2: Work loop, merge gate, report and ticker

- **Files**: `.claude/commands/night-shift.md` (full protocol), `tools/night_shift_gate.py`, `tests/sim/test_night_shift_gate.py`, `mapkit/items.py` and `DOCUMENTATION_GUIDE.md` § 2.10 (ticket `type: question`), `docs/agents/map-writes.md` (question tickets, `## Answer`), `CLAUDE.md` (session-start rule: Night Shift is the usual ticker claimant; the morning Session reads `docs/night-shift/<date>.md` on LOWERED by night-shift), `docs/night-shift/README.md` (report layout).
- **Scope**: per Item, in candidate order (bugs confirmed first, then features, decisions, ideas; priority then age): read the Item and its links; re-rate with `mapkit item rate` on the branch when the filer's rating is wrong; one unit per Item as an `Agent` capped at about 80 calls, Sonnet unless the brief names why Opus (planner math, firmware, control behaviour); `./run_tests.sh --full` on the branch; `/audit` on the branch diff, its fixer applying narrative findings and leaving behaviour-affecting ones as HOLD reasons; `tools/night_shift_gate.py`; MERGE pushes `main` (ff when possible) and closes the Item in the same commit as the fix and its logbook entry; HOLD pushes the branch and leaves the Item `in_progress` with a Progress headline. Decision-gated Items: research, draft `## Options` into the body, file one question ticket, stop. Ideas: a `## Feasibility` note; an idea rated XS or S with hardware none may be built and goes through the same gate. Unrated Items are rated first. The ticker: `nightly_ticker.sh check --who "night-shift <date>"` at start, the line into `## Nightly`. Unanswered question tickets from earlier nights are listed again; answered ones re-open their Item for the next unit.
- **Critical details**: Night Shift never edits `~/Desktop/Jugglebot`, never force-pushes, never rewrites `main` history; a merge that does not fast-forward is a merge commit with the Item in the subject. A Night Shift commit carries `Logbook-Entry:` like any other. Two runs never overlap (`flock` on `temp/reports/night-shift/run.lock`). Any failure after a push leaves the report naming the pushed refs.
- **Dependencies**: P1.

### P3: Night Shift page in the map app

- **Files** (repo `jugglebot-map`): `server/nightshift.py` (list and read `docs/night-shift/*.md` from the data clone; list open `question` tickets), routes `GET /night-shift/reports`, `GET /night-shift/reports/{date}`, `GET /night-shift/questions`; `frontend/src/pages/NightShift/`, a nav entry beside the nightly chip, a badge with the open-question count; `tests/test_nightshift.py`.
- **Scope**: the latest report rendered with the existing Markdown component; each open question ticket shown with its parent Item, the question, and a textarea whose save `PATCH`es the ticket body appending `## Answer` (and `status: complete` on the ticket) through the existing store and git writer; one click to answer, one to save.
- **Dependencies**: P2 (ticket type, report layout). Deploy with `deploy-jetson.sh` between Sittings.

### P4: Next Sitting Runbook

- **Files**: `mapkit/sitting.py`, `mapkit/__main__.py` (`runbook` subcommand, also run by `gen`), `docs/runbook.md` (generated), `tools/night_shift_candidate.sh`, `docs/agents/map-writes.md` (the `## Sitting` section), `CLAUDE.md` (one line: a Session that files or progresses a hardware Item writes its `## Sitting` section), `tests/sim/test_sitting_sections.py`, `tests/sim/test_mapkit_docs.py` (currency of `docs/runbook.md`); app: `frontend/src/pages/NextSitting/`, `GET /runbook`, result writes via the existing PATCH.
- **Scope**: `## Sitting` on an Item body is a bullet list with fixed keys: `setup:` one of `level`, `tilt`, `armed`, `bb-fed`, `bridge-cycled` (several allowed, comma separated); `why:`; `do:` (hardware steps only, imperative); `pass:`; `record:`; `minutes:` integer; optional `branch:` a held branch. The generator orders rows by `priority weight (high 3, medium 2, low 1) x (2 if branch else 1) / minutes` and groups them into blocks sharing a `setup:` set so transitions are minimised, and writes `docs/runbook.md` with a header (date, candidate branch, setup block 0 from the standing facts: bridge power-cycle, `hand_source` latch), the rows as the sketch in JB-119's body, a `## Questions a Sitting can answer` block from open question tickets whose parent has hardware robot, and a `## Close-out` block ending in the draft wrap-up prompt. `night_shift_candidate.sh` merges every held branch whose Item has a `## Sitting` section into `night-shift/sitting-<date>` and reports conflicts as HOLD rows. The app page renders the rows with a pass or fail toggle and a notes box per row; save appends `## Sitting result (<date>)` with the verdict and notes to the Item. Pathway chips filter the same list.
- **Critical details**: the generator never invents a `do:` step; a hardware Item without a `## Sitting` section appears in a `## Needs a Sitting section` list, not as a row. `docs/runbook.md` is generated and never hand-edited (the currency test fails otherwise). `tests/hardware/*.md` Runbooks are linked from rows, never duplicated.
- **Dependencies**: P2 (held branches), P3 (page scaffolding).

### P5: Sitting wrap-up command

- **Files**: `.claude/commands/sitting-wrapup.md`, `tests/sim/test_sitting_wrapup_protocol.py` (the command file names every step the Runbook's close-out promises).
- **Scope**: the morning or post-Sitting Session runs `/sitting-wrapup`: reads every `## Sitting result` newer than the last wrap-up, merges the candidate branch's passed Items into `main` (per-Item merge so a failed Item's branch stays held), returns the live checkout to `main`, rebuilds if `ros_ws` changed, commits `docs/records.md`, files follow-up Items from the notes, writes the logbook entry, regenerates the Runbook. Night Shift's protocol gains one step: list unaddressed `## Sitting result` sections in the report.
- **Critical details**: the wrap-up runs in `~/Desktop/Jugglebot` under the single-writer rule, so it checks for a peer Session first. Never a force-push; a failed merge leaves the branch held with the conflict named.
- **Dependencies**: P4.

### R6: First Runbook-driven Sitting

- **Rung gate**: one Sitting flown from `docs/runbook.md`, at least one held branch passed and merged by `/sitting-wrapup`, the owner's typed steps limited to hardware actions and result boxes. Logged with the (date, command, result) triple.
- **Dependencies**: P5, a held hardware branch from a real night.

## 5. Testing Plan

Unit (offline):
- T-U1 `tools/night_shift_gate.py`: each of the six gate conditions failing alone yields HOLD naming that condition; all passing yields MERGE; a missing audit report is HOLD, never MERGE.
- T-U2 preflight "nothing new": same ids and stamps -> exit 0 and no `claude` invocation (fake `claude` on PATH records calls); one new Item, one changed `updated`, or one answered ticket -> invocation.
- T-U3 runner guards: hold file present -> `HELD`; `live_session()` true -> wait then `DEFERRED`; both runners source the one guard file (a grep test that `nightly_suite.sh` no longer defines its own regex).
- T-U4 ticket `type: question` validated; `## Answer` detection; a `question` ticket on a non-existent parent is refused.
- T-U5 `## Sitting` parser: every key, missing `do:` -> listed under needs-a-section, bad `minutes` -> refused; ordering and setup grouping on a fixture of six Items; `docs/runbook.md` currency.
- T-U6 `night_shift_candidate.sh` on temporary git repositories: clean stack, one conflicting branch reported and skipped, no change to `main`.
- T-U7 report layout: a report fixture parses into the seven sections; bare `JB-n` in every row.
- T-U8 (app) routes return the report list, a report, the open questions; a PATCH answer appends `## Answer` once and sets the ticket complete.

Integration (this Jetson, no robot):
- T-I1 `systemctl --user start jugglebot-night-shift.service` with `NIGHT_SHIFT_DRY_RUN=1`: a dry report appears under `temp/reports/night-shift/`, nothing pushed, `total_cost_usd` logged.
- T-I2 deny rules under `--dangerously-skip-permissions`: a probe prompt that tries `pio run` is refused (else the fallback permission mode is adopted and documented).
- T-I3 the first real night: every merged commit's branch has a gate log and audit report; `git log origin/main` shows only gate-passed merges; the map app shows the report and its questions within one minute of the push.
- T-I4 timer ordering: Night Shift waits on the suite flock while the nightly runs; neither paints the other's status.

Hardware:
- T-H1 = R6.

Regression:
- T-R1 the nightly's status line, ticker and `latest.md` are byte-identical in form after the guard refactor (fixture run of `nightly_suite.sh` with a fake `run_tests.sh`).
- T-R2 `/work-inbox` unchanged for interactive use (its tests pass).

## 6. Notes for Collaborators

- **Safety invariants**: the clone never runs `tests/hardware/`, `ros2 launch`, or any flash; the bridge is powered at night and the UDP link is single-owner (`project_hardware_bench_facts`). Night Shift never writes to `~/Desktop/Jugglebot`. No force-push, no history rewrite on `main`. A HOLD is never silently promoted to MERGE; the gate script is the single enforcement point and `tests/sim/test_night_shift_gate.py` pins it.
- **Architecture decisions**: headless Claude Code over a cloud routine or Managed Agents because the trustworthy gate is this box (venv, MuJoCo, serial timing tests, teensy_link wire tests); over a hand-rolled orchestrator because it would reimplement the harness and every repo rule. Fable orchestrates because the role is judgment per call with a small context; units are Sonnet by default per the 2026-09-07 token rule. Questions are tickets because the drawer already edits and links them. The Runbook is generated from Items so it cannot drift from the board.
- **Startup ordering**: nightly 04:00 -> Night Shift 04:45 (waits on the suite flock) -> auto-pull -> map-pull -> page. A Sitting that starts before Night Shift finishes is detected by the guard, which waits and then defers.
- **Files affected**: listed per step above; the `jugglebot-map` changes live in that repository and deploy with `deploy-jetson.sh`.
- **Rollback**: `systemctl --user disable --now jugglebot-night-shift.timer`; held branches are deleted or kept; merges to `main` revert like any commit and the bundled `complete` re-opens textually; the guard refactor reverts by restoring the regex into `nightly_suite.sh`.
