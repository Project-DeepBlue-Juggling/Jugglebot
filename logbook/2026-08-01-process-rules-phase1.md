---
title: "Refactor Phase 1 — process rules: SHA backfill retired, short-form logbook default, audit gate scoped, CLAUDE.md diet, plans archived, memory compacted"
type: refactor
date: 2026-08-01
status: resolved
phase: "refactor-2026-07 Phase 1"
related_plan: refactor-2026-07.md
files_changed:
  - CLAUDE.md
  - DOCUMENTATION_GUIDE.md
  - logbook/README.md
  - logbook/TEMPLATE.md
  - docs/analysis/logbook.md
  - tools/probes/README.md
  - .claude/commands/investigate.md
  - .claude/commands/diagnose.md
  - .claude/commands/log.md
  - .claude/commands/commit.md
  - .claude/commands/next-phase-prompt.md
  - .claude/agents/logbook-updater.md
  - .claude/workflows/bb-tilt-phases.js
  - plans/active/INDEX.md
  - plans/active/refactor-2026-07.md
  - plans/active/PROMPT-anomaly-fixes-orchestration.md
  - plans/active/bb-online-juggle-tilt-rearchitecture.md
  - tests/sim/test_plans_index.py
  - tests/hardware/mvp_bench_runbook.md
  - plans/active/mvp-trajectory-bringup.md
  - plans/active/teensy-can-offload.md
  - plans/active/PROMPT-single-ball-toss-software-run.md
  - ros_ws/src/jugglebot_interfaces/action/Reload.action
  - plans/archived/2026-08-01 follower-cadence-and-divergence.md
  - plans/archived/2026-08-01 reload-action-catch-latch.md
  - plans/archived/2026-08-01 hardware-bringup.md
  - plans/archived/2026-08-01 dashboard-3d-mesh-and-sim-port.md
  - plans/archived/2026-08-01 always-on-telemetry-daemon.md
subsystem:
  - docs
tags:
  - process
  - docs
  - testing
---

# Refactor Phase 1 — process rules

## What changed and why

All six owner-approved items of `plans/active/refactor-2026-07.md` § Phase 1.
The programme's premise is that process ceremony, not architecture, is the drag.

1. **SHA backfill retired.** The `Logbook-Entry: <slug>` trailer is now the
   canonical *bidirectional* link (`git log --grep` is the reverse direction),
   so entries carry no `commits:` frontmatter and no follow-up backfill commit
   (~30% of July commits were backfills). Checked before deleting the
   convention: of 175 entries dated 2026, **156 are reachable by
   `git log --grep "Logbook-Entry: <slug>"`, 16 more by their recorded SHAs,
   3 by neither** (all pre-convention; nothing regresses). Rippled through
   every artifact that *generates* an entry, not just the ones that describe
   one — `logbook/README.md`, `TEMPLATE.md`, `DOCUMENTATION_GUIDE.md`,
   `docs/analysis/logbook.md`, `.claude/commands/{investigate,log,next-phase-prompt}.md`,
   `.claude/agents/logbook-updater.md`, `.claude/workflows/bb-tilt-phases.js`.
   The first pass missed `/log`, `/next-phase-prompt` and `bb-tilt-phases.js`,
   which is the whole failure mode: a convention retired only in the docs that
   *describe* it gets re-injected by the next generator run. A follow-up
   `git ls-files | xargs grep` then caught two more live instruction sites —
   `plans/active/PROMPT-anomaly-fixes-orchestration.md` and
   `plans/active/bb-online-juggle-tilt-rearchitecture.md` — also fixed.
   Historical *records* of past backfill commits (in
   `catch-reach-degenerate-overshoot.md`, `hand-command-continuity.md`, older
   logbook entries) are left verbatim: they say what happened, not what to do.
2. **Short-form entries are the default** (10–30 lines: what/why + the
   verification triple). The full investigation form with a real Discussion
   stays MANDATORY under the three existing triggers, quoted verbatim in both
   CLAUDE.md and `logbook/README.md`.
3. **`/audit` gate scoped** to ≥2 narrative files or any normative doc; a single
   logbook entry landing with its code no longer triggers it.
4. **Double-suite wording clarified**: one `./run_tests.sh` run satisfies both
   "after the change" and "before the commit" when no edit intervenes.
5. **CLAUDE.md diet**: 25,414 → 22,089 bytes (−13.1%) with every rule retained. The three
   giant inline worked examples became one-line rules plus links — the
   logbook-search coverage trace moved into `logbook/README.md` § "What the
   logbook tests actually check" (it existed nowhere else), the probe-placement
   history now points at `tools/probes/README.md`, and the two 2026-05-11
   canonical examples point at the tier1a/tier1c entries that already tell them.
6. **Five plans archived** with archival notes (follower-cadence,
   reload-action-catch-latch, hardware-bringup, dashboard-3d-mesh,
   telemetry-daemon); `leg-gain-tuning-methodology.md` was deliberately left
   active — the gain *hunt* closed, but the document is the live normative
   methodology cited by production code, `hardware_config.yaml` and the active
   accel-FF plan. New `plans/active/INDEX.md` + `tests/sim/test_plans_index.py`
   pin the index in both directions. Memory compacted 53 → 18 topic files
   (index 13,678 → 5,727 bytes) by verbatim merge; a script verified all 49
   merged bodies survive byte-for-byte, and the 4 tombstoned arcs had their live
   payloads migrated first.

## Review findings applied at finalize

Two adversarial review passes ran before the commit and served as this change's
`/audit` gate (both approved; all WARNINGs below were verified against the tree
and fixed, none rejected):

- **Retirement didn't reach the generators.** `/log`, `/next-phase-prompt` and
  the tracked `bb-tilt-phases.js` skill still taught `commits: <pending>` +
  SHA backfill, and `/next-phase-prompt` cited the memory file the same stage
  had merged away. Fixed; `/log` and `/next-phase-prompt` also gained the
  short-form default and the three-trigger Discussion gate.
- **Audit-scope narrowing was too tight.** The rewritten rule closed the
  normative-doc list to four names in the same commit that delegated CLAUDE.md
  obligations *into* `logbook/README.md` and `tools/probes/README.md` — a solo
  edit to either would have escaped the gate. The list is open-ended again and
  names the delegation targets explicitly.
- **One delegated sub-obligation didn't survive the move**: "every committed
  probe gets a README row" was dropped from CLAUDE.md and never landed in
  `tools/probes/README.md`. Added there.
- **The memory compaction broke its own cross-reference graph** — 46 dangling
  `[[wikilink]]`s across 30 merged-away topics, plus a *new* stale pointer to
  the `always-on-telemetry-daemon` plan this stage archived. Every body
  survived the verbatim merge; only the links rotted. All now repoint at
  `<file> § "<section>"`, self-references read as "(this file)", and three
  pre-existing stale plan paths were fixed too.
- `.claude/commands/commit.md` pointed at the deleted
  `feedback_verify_staged_before_commit` memory file. Repointed.
- Gate-command consistency: `/next-phase-prompt` and `bb-tilt-phases.js` told
  agents to run bare `pytest tests/ -q`. That is not just slower — it takes no
  flock, so an agent's gate collides with a concurrent session's instead of
  queueing. Both now say `./run_tests.sh`. (The one `pytest tests/ -q
  --hypothesis-profile=ci-deep` occurrence is a quoted 2026-05-11 citation
  illustrating the triple format and was left verbatim.)

**Deliberate non-changes:** `Reload.action`'s one-line comment fix stays (see
Verification); `leg-gain-tuning-methodology.md` stays active; the four untracked
`.claude/workflows/*-runner.js` files still teach SHA backfill but belong to
other sessions and were not touched.

## Memory layer (outside git)

`~/.claude/projects/-home-jetson-Desktop-Jugglebot/memory/` went 53 topic files
→ 18 (19 files including `MEMORY.md`; index 13,678 → 5,727 B). Not a tracked path, so `files_changed` above
cannot cover it. Pre-compaction backup at `memory_backup_20260801/` (54 files);
pre-link-rewrite backup at `memory_prelinkfix_20260801/`.

## Verification

- Full gate: `./run_tests.sh`, run 2026-08-01 on the Jetson under
  `~/Desktop/PDJ_venv/venv` — **parallel phase 4312 passed, 3 xfailed in
  435.18 s; serial phase 9 passed in 39.75 s; total 481 s; RESULT: PASS**
  (exit 0). Combined **4321 passed, 3 xfailed**; the 3 xfails are the
  pre-existing set.
- Scoped: `pytest tests/sim/test_plans_index.py tests/sim/test_logbook_search.py -q`,
  run 2026-08-01: **44 passed in 0.25 s** (20 new plans-index tests + the 24
  pre-existing logbook-search tests). Re-run after writing this Verification
  section, since recording a gate result is necessarily an edit *after* the
  gate: same result.
- Both logbook-touching test surfaces were traced rather than assumed —
  `test_logbook_search.py` parses the real `logbook/` directory but skips
  `INDEX.md` and silently drops entries with no `title`, so it does **not**
  cover the INDEX row or malformed front matter (§ "What the logbook tests
  actually check" in `logbook/README.md`). The INDEX row and this entry's front
  matter were checked by reading, not by relying on a passing count.
- Index contract negative-checked out-of-band: injecting an archived plan name
  is reported as stale, deleting a row is reported as missing.
- `ros_ws/src/jugglebot_interfaces/action/Reload.action` changed by one comment
  line only (a plan path that would have rotted). Behaviour is unchanged, but
  the launch runs the **installed** copy — pick the change up at the next
  routine `colcon build --packages-select jugglebot_interfaces jugglebot`.
