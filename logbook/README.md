# Engineering Logbook

A structured, searchable record of every investigation, bugfix, refactor, feature, and optimization in the Jugglebot project. Every code change gets a logbook entry that captures *why* the change was made, not just *what* changed.

## Why This Exists

Git history tells you *what* changed. Commit messages tell you *what* was intended. But neither captures the full story: what symptoms were observed, what hypotheses were considered and rejected, what tradeoffs were made, or what side effects were discovered. This logbook fills that gap.

Future collaborators (human or AI) can search the logbook to understand:
- Why a particular file looks the way it does
- What investigations have touched a subsystem
- What issues were encountered and how they were resolved
- What's still open or partially addressed

## Quick Reference

| Command | Purpose |
|---------|---------|
| `/investigate` | Full hardware diagnosis-to-fix pipeline (gated) |
| `/log <type> <title>` | Log a non-hardware code change |
| `/logbook` | Browse, search, and filter entries |
| `/archive-plan <name>` | Archive a completed plan (with critical review) |

## Workflows

### After a hardware test session

```
/investigate                          # latest unanalysed session
/investigate mpc_20260401_152101.csv  # specific session
/investigate --dry-run                # diagnose + log, skip fix steps
/investigate --resume <entry>         # resume a paused investigation
```

The `/investigate` pipeline walks through 9 gated steps:

1. **Determine scope** — find target CSVs
2. **Diagnose** — run analysis engine, cross-reference known issues
3. **Create logbook entry** — via logbook-updater agent
4. **Propose fixes** — via fix-proposer agent (1-3 options with risk assessment)
5. **Implement** — make code changes
6. **Test** — run `./run_tests.sh` (the full-suite gate)
7. **Commit** — with `Logbook-Entry:` trailer
8. **Push** — optional
9. **Update outcome** — fill in results, set status

Each step is gated — you can stop at any point and resume later.

### After a code change (non-hardware)

```
/log refactor Extract MPC orchestration into controller/
/log bugfix Fix race condition in IPC handshake
/log feature Add dashboard auto-reconnect
/log optimization Reduce solver warm-start overhead
/log --from-commits a32bf27,3a3381f    # retroactive
```

Subsystem tags are auto-detected from file paths. The entry uses sections appropriate to the type (e.g., a refactor gets Motivation/Changes/Verification instead of Symptoms/Diagnosis).

### Browsing the logbook

```
/logbook                        # full index with status counts
/logbook --summary              # digest of recent entries
/logbook --timeline             # chronological view with plan milestones
/logbook --search velocity      # keyword search across all entries
/logbook --file motor_guard.py  # reverse lookup: what entries touched this file?
/logbook --subsystem mpc        # filter by subsystem
/logbook --status open          # show unfinished entries
/logbook <entry-name>           # read a specific entry
/logbook --new "my title"       # create a blank entry manually
```

### Archiving a plan

```
/archive-plan hardware-bringup
```

The plan-reviewer agent critically checks every milestone against the actual codebase before allowing archival. Plans move from `plans/active/` to `plans/archived/` with a completion date.

## Entry Length — short form is the default

**Most entries are short form: 10–30 lines.** What changed, why, and the
(date, command, result) verification triple. Front matter still carries
`title` / `type` / `date` / `status` / `phase` / `files_changed` /
`subsystem`. That is the whole obligation for a routine bugfix, refactor,
feature, or process change.

**Escalate to the full investigation form — with a real Discussion section —
when any of these hold** (verbatim from CLAUDE.md's Engineering Philosophy):

> (a) a hypothesis was withdrawn or reframed mid-investigation, (b) a
> non-obvious tradeoff was accepted, (c) the chosen approach beat another
> reasonable approach for reasons future-readers wouldn't infer from the
> code alone.

Under those triggers the Discussion is non-negotiable, and it is written
*before* the Fix section. Hardware investigations almost always hit at least
one trigger; a docs or plumbing change almost never does. When in doubt, ask
whether a future session would be able to reconstruct *why* from the code and
the commit alone — if not, write the Discussion.

## Entry Format

Every entry is a markdown file with YAML frontmatter:

```yaml
---
title: Velocity feedforward semantic mismatch causing violent oscillation
type: investigation          # investigation | bugfix | refactor | feature | optimization
date: 2026-03-30
status: resolved             # open | in-progress | tuned | resolved
phase: "3.1"                 # bringup phase (optional)
related_plan: hardware-bringup.md
related_issues:
  - VEL_FF_BUG
sessions:
  - mpc_20260330_171932.csv
files_changed:               # enables reverse lookups
  - controller/mpc.py
  - ros_ws/src/jugglebot/jugglebot/motion/motor_guard.py
subsystem:                   # controlled vocabulary
  - mpc
  - motion
tags:
  - safety
---
```

> **Legacy field:** entries written before 2026-08-01 also carry a
> `commits:` list of short SHAs. New entries do **not** — see
> [Commit Traceability](#commit-traceability) below.

### Entry types and their sections

| Type | Use when... | Sections |
|------|-------------|----------|
| **investigation** | Diagnosing a hardware anomaly | Symptoms, Diagnosis, Discussion, Fix, Outcome |
| **bugfix** | Fixing a software bug | Problem, Root Cause, Fix, Verification |
| **refactor** | Restructuring code | Motivation, Changes, Verification |
| **feature** | Adding new functionality | Motivation, Design, Implementation, Verification |
| **optimization** | Improving performance | Motivation, Approach, Benchmarks, Verification |

All types share: **Summary** (at top), **Withdrawn claims** (investigations), and **Open Questions** (at bottom).

### Status ladder

| Status | When to use |
|--------|-------------|
| `open` | Nothing done yet. |
| `in-progress` | Diagnosis is done, fix or verification is still ongoing. |
| `tuned` | The specific symptom scoped to this entry is addressed and verified on hardware, but the entry intentionally has an open sibling investigation (tracked elsewhere or in Open Questions). Use this rather than leaving a real ship at `in-progress` forever. |
| `resolved` | Every symptom in this entry's scope is addressed and verified; no open follow-ups inside this entry's scope. |

### Withdrawn claims

Investigation entries have a **Withdrawn claims** section near the bottom.
When an earlier interpretation in the entry (or in the investigating
conversation) turns out to be wrong, document the retraction there rather
than silently editing the wrong claim out of the Diagnosis or Verification
sections. This makes incorrect conclusions part of the investigation
history so the next person — or the next LLM — doesn't repeat the mistake.
Format and an example are in `TEMPLATE.md`.

### Tag taxonomy

**Subsystem tags** (auto-detected from file paths):

| Tag | Covers |
|-----|--------|
| `mpc` | `controller/mpc.py`, `controller/params.py` |
| `controller` | `controller/` (non-MPC) |
| `motion` | `ros_ws/.../motion/` |
| `can` | `ros_ws/.../can/`, `can_node.py` |
| `tracking` | `ros_ws/.../tracking/` |
| `ros` | orchestrator, state machine, bridge nodes |
| `gui` | `ros_ws/gui/` |
| `sim` | `sim/` |
| `config` | `config/` |
| `tools` | `tools/` |

**Content tags** (applied manually):
`safety`, `performance`, `IPC`, `kinematics`, `dynamics`, `testing`, `docs`

## Commit Traceability

Code-change commits include a `Logbook-Entry:` trailer:

```
fix: cold-hold fallback holds at current position instead of stroke minimum

Logbook-Entry: 2026-04-01-cold-hold-fallback-stroke-minimum
Co-Authored-By: Claude Opus 4.6 (1M context) <noreply@anthropic.com>
```

**The trailer is the canonical link, and it works in both directions:**

- **code → entry**: **code line** → `git blame` → **commit** → `Logbook-Entry`
  trailer → **logbook entry** (symptoms, discussion, alternatives, outcome)
- **entry → code**: `git log --grep "Logbook-Entry: <slug>"` lists every commit
  that belongs to the entry — including follow-ups the entry never knew about

Because the reverse direction is a one-command query, **entries do not carry
commit SHAs**. The old convention (a `commits:` frontmatter list, backfilled by
a small follow-up commit right after the real one) was retired on 2026-08-01: it
cost a second commit on ~30% of commits and duplicated what the trailer already
provides. Do not add `commits:` to new entries and do not write a backfill
commit. Historical entries keep their `commits:` blocks — of 175 entries dated
2026, 156 are reachable by `git log --grep` and a further 16 by their recorded
SHAs (checked 2026-08-01).

## What the logbook tests actually check

`pytest tests/sim/test_logbook_search.py -q` (run 2026-07-27: **24 passed in
0.19 s**) *does* parse the real `logbook/` directory, so a logbook edit is not
automatically outside the test surface — **but it would not catch the two
failures you would most expect it to**:

- `sim/analysis/logbook_search.py` skips `INDEX.md` outright (`_SKIP_FILES`),
  so a broken or missing INDEX row passes green.
- `load_entries` silently `continue`s past any entry whose front matter lacks a
  `title`, and the test asserts only `len(entries) >= 2` plus the shape of the
  alphabetically-first entry (a 2026-03-30 file) — so a malformed new entry is
  dropped, not flagged.

This is the worked example behind CLAUDE.md's rule that a "docs-only, so no
tests needed" exemption must name the tests that read the path you changed *and*
trace what they assert. Trace the coverage; never infer it from a passing count.
(Hardening `logbook_search` to warn-on-skip and validate front matter is a
tracked item in `plans/active/refactor-2026-07.md` Phase 6.)

## Interactive Diagnosis Reports

The diagnosis engine generates interactive HTML reports (Plotly) by default:

```bash
python3 sim/analysis/diagnose.py temp/logs/mpc_20260401_152101.csv --json
```

**Features:** scroll-zoom, box-select zoom, pan, hover tooltips, legend toggle (click to show/hide series).

**8 plot categories:** legs, pose, tracking, solver, velocity, hand, workspace, chatter. Auto-selected based on detected anomalies.

**Fallback:** `--static-plots` for matplotlib PNGs when Plotly isn't installed.

## Directory Structure

```
logbook/
  INDEX.md              ← summary table (auto-maintained)
  TEMPLATE.md           ← entry format reference
  README.md             ← this file
  YYYY-MM-DD-slug.md    ← individual entries

plans/
  active/                         ← in-progress plans
    <name>.md
  archived/                       ← completed/superseded plans
    YYYY-MM-DD <name>.md          ← prefixed with completion date

sim/analysis/
  diagnose.py           ← analysis engine (MPC telemetry + rosbag)
  compare_sessions.py   ← compare two hardware sessions (before/after)
  logbook_search.py     ← search logbook by flag types/subsystems (prior art)
  plot_interactive.py   ← Plotly interactive reports
  plot_diagnosis.py     ← matplotlib static plots (fallback)
  compare.py            ← sim-vs-hardware accuracy comparison
  report_html.py        ← HTML report generation
  known_issues.yaml     ← signature catalog for auto-detection
  log_index.json        ← per-session metadata

.claude/commands/       ← slash commands
.claude/agents/         ← dedicated agents
```

## Agents

The system uses dedicated agents to keep each task focused:

| Agent | Purpose | Used by |
|-------|---------|---------|
| **fix-proposer** | Reads diagnosis + source code, proposes 1-3 fixes with risk assessment | `/investigate` |
| **logbook-updater** | Creates/updates entries, maintains INDEX.md, cross-references | `/investigate`, `/log` |
| **plan-reviewer** | Critically checks plan completeness before archiving | `/archive-plan` |
