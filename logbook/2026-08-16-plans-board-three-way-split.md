---
title: "Plans board goes three-way (active / parked / archived), and archival stops renaming plans"
type: refactor
date: 2026-08-16
status: resolved
files_changed:
  - plans/parked/INDEX.md
  - plans/archived/INDEX.md
  - plans/active/INDEX.md
  - tests/sim/test_plans_index.py
  - DOCUMENTATION_GUIDE.md
  - CLAUDE.md
  - .claude/commands/archive-plan.md
  - config/generate_udp_protocol.py
  - config/hardware_config.yaml
  - teensy_link/client.py
  - teensy_link/tod_server.py
subsystem:
  - tools
  - config
tags:
  - docs
  - testing
---

# Plans board — three-way split, and the filename that stops moving

## Summary

Two owner-directed changes to the plans layer, landed in one pass.

**Change 1 — `plans/parked/`.** A parked plan used to keep living in
`plans/active/` with `status: parked`, so the active board answered "what should
I tackle next?" only after the reader mentally filtered six of fifteen rows. The
six plans parked on 2026-08-15 (`accel-ff-inertia`,
`catch-reach-degenerate-overshoot`, `hand-trajectory-generator-overhaul`,
`learned-ff-residuals`, `levelling-frame-contract`, `refactor-2026-07`) moved to
`plans/parked/` with their existing `status: parked` notes intact. The new
`plans/parked/INDEX.md` carries the column that makes the directory useful:
**what would unpark it** — the concrete gate, prerequisite or decision, derived
from each plan's own text. One plan (`hand-trajectory-generator-overhaul.md`)
states no gate at all; its row says so plainly rather than inventing one.

**Change 2 — archival stops renaming.** `plans/archived/` used a
`YYYY-MM-DD <name>.md` prefix. `DOCUMENTATION_GUIDE.md` mandates `related_plan:`
be **filename-only**, resolved by searching the plan directories — so the prefix
broke every inbound reference at exactly the moment the plan stopped being
editable. Twelve logbook entries named `bridge-temporal-trustworthiness.md` and
resolved to nothing within a day of its archival, and the same breakage had
recurred at every prior archival. No code consumer validates the field
(`tests/sim/test_logbook_front_matter.py` does not), which is precisely why it
went unnoticed for months: the damage was silent, cumulative, and visible only
to a human or agent actually trying to follow a link. All 28 prefixed archived
plans were renamed back to bare names, each gaining `archived: YYYY-MM-DD`
carrying the date the filename used to hold.

**The rule now: a plan's filename never changes for its life.** Parking and
archival are both a `git mv` of the file unchanged. The date stays sortable from
frontmatter, from the new `plans/archived/INDEX.md`, and from git's rename
record.

## Why the filename, and not something cleverer

The alternative — teach consumers to strip a date prefix — was rejected because
there is no consumer to teach. `related_plan:` is read by humans and agents, not
by code, so the "fix" would have to live in every reader's head forever. Making
the filename immutable puts the invariant in one place (the name) and makes it
testable (see below). It also means the *directory* becomes the only thing that
moves, which is why the guide now says prose should prefer the bare filename and
treat a path-qualified reference as a thing that will need re-pointing.

`plans/archived/INDEX.md` is new and was not literally asked for. It exists
because dropping the prefix removes chronology from `ls` — the archived board
would otherwise have had no date ordering anywhere. It is generated from the
`archived:` fields and pinned by the same forward/reverse rules as the other two.

## Migration and the reference sweep

- **28 archived plans renamed** (`git mv`), no bare-name collisions.
  `shaped-planning-efficiency.md` was already bare; its `archived:` date
  (2026-07-17) came from `git log --diff-filter=A`.
- **6 pre-frontmatter archived plans** got a minimal YAML block (`title`,
  `status` where the file states one verbatim, `archived`). Without it their
  dates would have been lost with the prefix. `PROMPT-canbridge-rx-drain-throughput.md`
  states no status, so none was invented.
- **258 path references re-pointed across 134 files** (`plans/archived/<date> <name>.md`
  → `plans/archived/<name>.md`), including 6 `%20`-escaped markdown links and 6
  relative link targets inside archived plans.
- **248 more re-pointed across 146 files** for the directory moves: 146
  `plans/active/` → `plans/parked/` (caused by this change) and 102
  `plans/active/` → `plans/archived/` (pre-existing rot from earlier archivals,
  swept because it is the same defect class).

Both grep traps the task warned about were live:

1. **`rg` is not installed on this box**, so a ripgrep survey returns a silent
   zero. Everything here used `git grep` / `grep -rn`.
2. **Five paths were split across two source lines** by string concatenation or
   prose wrapping, so neither the full path nor the bare filename matched:
   `config/generate_udp_protocol.py:746`, `config/hardware_config.yaml:99`,
   `teensy_link/client.py:23`, `teensy_link/tod_server.py:16`, and
   `DOCUMENTATION_GUIDE.md:240` (rewritten wholesale anyway). The generator one
   is the dangerous one — it emits **five** artifacts, so fixing the artifacts
   without the generator would have restored the stale path at the next codegen.
   They were found by grepping for lines *ending* in a partial path — a
   `plans/archived/` followed by a bare date and then end-of-line, optionally
   through a closing quote — not by grepping for the path itself.

## Deliberately not swept

- **Three historical narrative mentions** of the old convention stay as written:
  `logbook/2026-08-01-analysis-safety-nets.md:203,238` and
  `logbook/2026-08-15-arc-closure-cleanup-r7-health-gate.md:57`. These describe
  what the naming convention *was* when that work happened (one of them is
  `plans/archived/2026-08-15 <name>.md`, a placeholder, not a resolvable path).
  Rewriting them would falsify the record. Live *pointers* inside logbook
  entries were re-pointed; narration of the convention was not.
- **17 references to deleted `PROMPT-*.md` files** (deleted per the 2026-08-09
  owner convention) have nothing to point at and were left.
- **Two files owned by a parallel session** — `ros_ws/docs/levelling_frame.md`
  and `tests/hardware/session_anomaly_fixes.md` — were left untouched. They
  carry 9 stale plan paths between them; listed in the handoff report for
  routing.

## What now makes the two changes stick

`tests/sim/test_plans_index.py` extended from 1 board to 3, keeping the original
tests' spirit and error messages. Rules, in order of how much they would have
saved:

1. **Every archived plan filename is bare** — no date prefix. This is the one
   that would have caught the whole problem; the convention can no longer
   silently regress.
2. **No index row names a file that left its directory**, for all three boards
   (the staleness protection that already existed for `active` → `archived`,
   extended to `parked`). The failure message now says which directory the file
   moved to.
3. **Forward + reverse** for all three: every plan has a row, every named row
   has a plan.
4. **Every parked row's four cells are filled**, the fourth being the unpark
   gate — a parked plan with a blank gate is a plan nobody can pick up.
5. **Every parked plan says it is parked on the plan itself**, so a reader who
   opens the file directly does not need the board.
6. **Every archived plan carries `archived: YYYY-MM-DD`** — now the only in-file
   record of when it closed.
7. **No bare filename is claimed by two boards**, since a filename-only
   reference could not resolve if one were.

Documentation followed: `DOCUMENTATION_GUIDE.md` § 2.6 rewritten (three-way
split with a schedulability test per directory, the never-rename rule with its
root cause, `archived:` and the parked note, and the grep cautions);
`.claude/commands/archive-plan.md` moved to the no-rename flow and gained a
manual **Parking a plan** section; `CLAUDE.md`'s INDEX rule now covers all three
directories. **No `/park-plan` command was built** — parking is documented as a
manual flow inside `/archive-plan`, and a proper counterpart command is worth
adding but was not in scope.

## Verification

`config/generate_udp_protocol.py` changed, so all five generated artifacts were
regenerated rather than hand-edited, and `config/generate_config.py` was re-run
after the `hardware_config.yaml` comment edit (no generated output changed —
that comment does not propagate).

**The firmware image is byte-identical**, so this change implies **no reflash**:

| | text | data | bss | `firmware.hex` md5 |
|---|---|---|---|---|
| before | 232768 | 35520 | 107872 | `ea705b4bb4026047318c0361750c87ab` |
| after  | 232768 | 35520 | 107872 | `ea705b4bb4026047318c0361750c87ab` |

Both measured 2026-08-16 with
`cd ros_ws/src/jugglebot/Teensy_code_canbridge && pio run -e teensy41`.

Scoped suites, all run 2026-08-16 in the project venv:

- `python -m pytest tests/sim/ -q` — **1417 passed, 3 xfailed in 1005.35 s**
  (the plans and logbook gates live here, including the extended
  `test_plans_index.py`).
- `python -m pytest tests/firmware/ -q` — **399 passed in 19.68 s** (the
  regenerated protocol artifacts and their xlang byte-diff).
- `python -m pytest tests/teensy_link/ -q` — **238 passed in 9.57 s**
  (`client.py` / `tod_server.py` docstrings were among the split-line fixes).
- `python -m pytest tests/sim/test_plans_index.py -q` — **63 passed in 0.17 s**,
  re-run after the last three documentation edits landed.

Gate (`./run_tests.sh`, run 2026-08-16, box confirmed quiet — a concurrent `pytest tests/sim` had corrupted an allocation baseline during scoped verification): **5241 passed in 241.24 s, RESULT: PASS.**
