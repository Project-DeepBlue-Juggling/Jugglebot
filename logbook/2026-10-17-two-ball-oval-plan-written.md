---
title: Two-ball oval plan written - the wayfinder map's 58 decisions become plans/active/two-ball-oval.md, rung-1 runsheet promoted
type: feature
date: 2026-10-17
status: resolved
phase: "two-ball-oval — charting (pre-P1)"
related_plan: two-ball-oval.md
related:
  - 2026-10-13-skill-stack-r6-close-out.md
files_changed:
  - plans/active/two-ball-oval.md (new, 371 lines)
  - plans/active/INDEX.md (row)
  - logbook/INDEX.md (row)
  - tests/hardware/session_oval_rung1.md (new, promoted from the map's draft)
subsystem:
  - skill-stack
tags:
  - planning
---

# Two-ball oval plan written

## What
The wayfinder map for the two-ball oval (`.scratch/two-ball-oval/` in worktree
`~/Desktop/Jugglebot-oval`, gitignored; charted 2026-10-14, 14 tickets, 58 owner decisions
2026-10-14 to 2026-10-17) is clear. Its destination is now `plans/active/two-ball-oval.md`:
§ 1.3 restates every decision by number so the plan stands without the map; § 2 fixes the
architecture (symmetric sites with T = P1 near Ball Butler and C = P2, one carry-folded STEADY
window, the level catch with a receive-pin tilt hold, C-CUP-4 as normative text, the
`level | pose` planner mode, box/workspace/dwell rules, learner key on the release site, the
aim principle, fed entry and exit); § 3/§ 4 are five phases each gated by a hardware rung
(P1 pattern dispatchable → rung 1 go/no-go on the loaded carry at 150 mm / dwell 0.50 /
apex 0.95 at today's limits; P2 banking leak fix; P3 pose planner → decision 24 gate; P4 one
working-point move 600/8000/200k + 30 A + dwell ladder; P5 two-ball → 30 consecutive).
The rung-1 runsheet is promoted to `tests/hardware/session_oval_rung1.md` with the draft's
site rows corrected (BB feeds T through the reload; the first carry follows the first steady
catch at C).

## Why
Owner decision 1 (2026-10-14): the map's destination is a phased plan with every decision
locked so the build runs through `/next-phase-prompt`. Nothing was left to decide after
tickets 05 and 06 closed on 2026-10-17.

## Verification
Docs only. The tests that read these paths: `tests/sim/test_plans_index.py` (forward: the new
plan has an active-board row; reverse: every md name in the active index exists there) and
`tests/sim/test_logbook_front_matter.py` (this entry's `related_plan` resolves by bare
filename across the three plan directories). Run 2026-10-17:
`pytest tests/sim/test_plans_index.py tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_search.py -q`
- **113 passed in 0.89 s**. `/audit --unstaged` run the same day: 0 blocking, 3 warnings,
3 notes, all narrative, all applied before the commit.
