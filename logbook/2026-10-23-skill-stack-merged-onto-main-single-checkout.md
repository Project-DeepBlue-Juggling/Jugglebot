---
title: "skill-stack merged onto main: one checkout, one branch; the three worktrees and the merged branches retired"
type: refactor
date: 2026-10-23
status: resolved
phase: "two-ball-oval - before rung 2"
related_plan: two-ball-oval.md
files_changed:
  - CLAUDE.md
  - plans/active/two-ball-oval.md
  - plans/active/learner-generalisation.md
  - tests/hardware/session_oval_rung1.md
  - tests/hardware/session_skill_stack_rehearsals.md
  - logbook/INDEX.md
subsystem:
  - infrastructure
---

## What changed

Owner, 2026-10-23: the skill-stack design "has been very conclusively validated" (row 47 and
48 PASSED, the 48-catch columns run, oval rung 1 at 28/28), so the branch is merged onto
`main` and the tree tidied before the oval advances.

- `main` had no commit that `skill-stack` lacked (1592 behind, last moved 2026-02-17), so
  the merge is a pure fast-forward (`git push origin skill-stack:main`), twice today: once
  after the sitting records and once after the planner commits.
- `mvp-trajectory-bringup`, the branch the main checkout had been on, carried seven unique
  commits: the firmware/GUI ones were already ported (FW 25, the BB manual aim), and the two
  2026-09-21 docs commits were ported this morning (`CONTEXT.md` glossary, `docs/agents`,
  the docs-link contract). Nothing functional was left behind.
- `~/Desktop/Jugglebot` is now the single checkout, on `main`, with a fresh `ros_ws/install`.
  The 04:00 nightly, `~/.zshrc` and the 2-minute auto-pull timer already pointed there, so
  the nightly now tests the live code (it had been testing `mvp-trajectory-bringup`). The
  GUI service unit (`~/System_Scripts/jugglebot-gui.service`, port 8081) is repointed from
  the skills worktree to this checkout and restarted.
- The worktrees `~/Desktop/Jugglebot-skills`, `~/Desktop/Jugglebot-oval` and
  `~/Desktop/Jugglebot-bbcal` are removed. Their gitignored material was copied into the
  main checkout first with `rsync --ignore-existing`: `temp/logs` (the paths the 2026-10
  entries cite), `temp/sweeps` (the admissible-box sweep scripts), `temp/probes`,
  `temp/reports`, and the oval wayfinder map to `.scratch/two-ball-oval/`.
- Branches: every branch except `main` and `two-ball-oval` is deleted locally and on
  origin (owner's choice). Each deleted tip is kept reachable under a lightweight tag
  `archive/<branch>` so nothing becomes unreachable: the parked clapboard arc, the June
  circuit-diagram update on `refactor`, the hand-geometry-correction record, the demo
  branch, `ros2_first_approach`, `handoff-robustness`, `main_backup`, `bb-calibration`,
  `mvp-trajectory-bringup` and `skill-stack` itself. `two-ball-oval` is fast-forwarded to
  `main` and kept for the oval work.
- Current-state docs repointed from the branch/worktree to `main` in the single checkout
  (CLAUDE.md's live-stack bullet and parallel-session bullet, the oval plan's header and
  gate, the learner-generalisation plan, the two live runbooks' step rows). Dated runbooks
  and logbook entries keep their original paths as history.

## Verification

- 2026-10-23, `git rev-list --left-right --count origin/main...skill-stack` before each
  push -> `0 N` (fast-forward only); after: `main == skill-stack == daa495e0`.
- 2026-10-23, `pytest tests/sim/test_plans_index.py tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py -q` -> 99 passed (docs repoint).
- 2026-10-23, `colcon build --packages-select jugglebot_interfaces jugglebot` in
  `~/Desktop/Jugglebot/ros_ws` -> both packages built, `dwell_s` present in the installed
  `Juggle.action`.
- GUI: `systemctl --user restart jugglebot-gui.service` -> the server process runs from
  `~/Desktop/Jugglebot/ros_ws/gui` (checked with `ps`).
