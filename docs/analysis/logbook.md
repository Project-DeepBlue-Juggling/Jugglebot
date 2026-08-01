# Engineering Logbook

The engineering logbook captures the *why* behind code changes -- symptoms, hypotheses, rejected alternatives, and outcomes that don't belong in git commits or code comments.

## Why a Logbook?

Git history tells you **what** changed. Code comments tell you **how** it works. But neither captures:

- The symptoms that triggered the investigation
- Hypotheses that were considered and rejected
- Side-effects discovered during debugging
- Before/after metrics proving the fix worked
- Open questions for future sessions

The logbook fills this gap with structured, searchable entries linked to commits.

## Commands

### `/investigate` -- Full Hardware Diagnosis Pipeline

The primary workflow for hardware issues. Orchestrates a 9-step gated pipeline:

```
/investigate                              # Most recent unanalysed session
/investigate mpc_20260401_143845.csv      # Specific session
/investigate --resume <entry>             # Resume a paused investigation
/investigate --dry-run                    # Diagnose + log only, skip fixes
```

**Pipeline steps** (each gated by user approval):

1. **Determine scope** -- find target CSV(s), auto-group related sessions
2. **Diagnose** -- run `/diagnose`, cross-reference known issues
3. **Create logbook entry** -- via logbook-updater agent
4. **Propose fixes** -- via fix-proposer agent (with prior art search)
5. **Implement** -- make code changes
6. **Test** -- run `pytest tests/ -v`
7. **Commit** -- with `Logbook-Entry:` trailer
8. **Push** -- optional
9. **Update outcome** -- fill in results, suggest `--compare` for before/after

### `/log` -- Log Code Changes

For non-hardware changes (refactors, features, optimizations):

```
/log refactor Extract MPC orchestration from sim/ into controller/
/log bugfix Fix race condition in IPC handshake
/log feature Add dashboard auto-reconnect
/log --from-commits a32bf27,3a3381f     # Retroactive logging
```

### `/logbook` -- Browse and Search

```
/logbook                           # Show full index
/logbook --summary 7               # Digest of last 7 days
/logbook --search oscillation      # Keyword search across all entries
/logbook --file motor_guard.py     # Which entries touched this file?
/logbook --subsystem mpc           # Filter by subsystem tag
/logbook --status open             # Show unfinished investigations
/logbook 2026-03-30-velocity-feedforward-oscillation  # Read specific entry
```

---

## Entry Format

### Frontmatter

Every entry has YAML frontmatter for structured metadata:

```yaml
---
title: Velocity feedforward semantic mismatch causing violent oscillation
type: investigation
date: 2026-03-30
status: resolved
phase: "3.1"
related_plan: hardware-bringup.md
related_issues:
  - VEL_FF_BUG
sessions:
  - mpc_20260330_171932.csv
files_changed:
  - controller/mpc.py
  - ros_ws/src/jugglebot/jugglebot/motion/motor_guard.py
subsystem:
  - mpc
  - motion
tags:
  - safety
  - dynamics
---
```

### Sections by Entry Type

| Type | Sections |
|------|----------|
| **investigation** | Summary, Symptoms, Diagnosis, Discussion, Fix, Outcome, Open Questions |
| **bugfix** | Summary, Problem, Root Cause, Fix, Verification, Outcome |
| **refactor** | Summary, Motivation, Changes, Verification, Outcome |
| **feature** | Summary, Motivation, Design, Implementation, Verification, Outcome |
| **optimization** | Summary, Motivation, Approach, Benchmarks, Verification, Outcome |

### Taxonomy

**Subsystem tags** (auto-detected from file paths):

`mpc` `controller` `motion` `can` `tracking` `ros` `gui` `sim` `config` `tools`

**Content tags** (manual):

`safety` `performance` `IPC` `kinematics` `dynamics` `testing` `docs`

---

## Prior Art Search

When proposing fixes for a new issue, the fix-proposer agent searches past logbook entries for similar symptoms using `sim/analysis/logbook_search.py`.

### How It Works

Matching criteria (scored by relevance):

1. **Direct issue ID match** (+3) -- diagnosis flags reference a `known_issues.yaml` ID that appears in a past entry's `related_issues`
2. **Flag-type overlap** (+2) -- e.g., both the current diagnosis and a past entry involve oscillation flags
3. **Subsystem overlap** (+1) -- entries sharing subsystem tags with the flagged code

### Example Output

```
### Prior Art

Found 1 related past investigation:
- **2026-03-30 Velocity feedforward oscillation** (resolved)
  Match: oscillation flags with amplitude growth, subsystem overlap: mpc
  Fix applied: use (cmd - prev_cmd)/dt for velocity feedforward
  Outcome: resolved, <0.5mm steady-state error
```

### Standalone Usage

```bash
# Search by flag types
python sim/analysis/logbook_search.py --flags '["oscillation"]' --subsystems '["mpc"]'

# Search by known issue IDs
python sim/analysis/logbook_search.py --issue-ids '["VEL_FF_BUG"]'
```

---

## Commit Traceability

Code-change commits include a `Logbook-Entry:` trailer. **The trailer is the
canonical bidirectional link** — entries carry no `commits:` frontmatter field
(convention retired 2026-08-01; run `git log --grep "Logbook-Entry: <slug>"` for
the reverse direction, which finds *every* commit for an entry rather than the
one SHA a manual backfill happened to capture):

```
fix: cold-hold fallback holds at current position instead of stroke minimum

Logbook-Entry: 2026-04-01-cold-hold-fallback-stroke-minimum
Co-Authored-By: Claude Opus 4.6 (1M context) <noreply@anthropic.com>
```

This creates a one-hop path from any line of code to its full investigation:

```
code line  ->  git blame  ->  commit  ->  Logbook-Entry trailer  ->  logbook entry
```

The logbook entry contains the full arc: symptoms observed, hypotheses considered, alternatives rejected, fix applied, and outcome verified.

---

## Agents

### Fix-Proposer

Analyses a diagnosis report and proposes 1-3 actionable fixes:

- Searches logbook for **prior art** before proposing
- Traces the full data path (MPC solver -> runner -> plant -> IPC -> motor_guard -> CAN)
- Assesses **risk** and **control-system implications**
- Specifies which hardware re-test to run

### Logbook-Updater

Handles mechanical file operations:

- Creates/updates entry files with proper frontmatter
- Maintains `logbook/INDEX.md` (sorted by date)
- Cross-references `sim/analysis/log_index.json` with logbook entry pointers
