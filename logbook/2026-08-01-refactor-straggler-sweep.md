---
title: Refactor straggler sweep — safety.md re-framed live-first, plan paths refreshed, runner scripts rippled, docker sim stack deleted, ERR_TIMEOUT prompt authored
type: refactor
date: 2026-08-01
status: resolved
phase: "refactor-2026-07 — post-programme stragglers 2-3"
files_changed:
  - ros_ws/docs/safety.md
  - plans/archived/bb-led-two-ball-juggle-demo.md
  - plans/active/bb-online-juggle-tilt-rearchitecture.md
  - plans/active/mvp-trajectory-bringup.md
  - plans/active/PROMPT-err-timeout-hand-path.md
  - plans/active/INDEX.md
  - CLAUDE.md
  - sim/Dockerfile
  - sim/compose.yaml
  - sim/.dockerignore
subsystem:
  - repo
  - ros
---

# Refactor straggler sweep

**What** (owner-directed, 2026-08-01):

1. `ros_ws/docs/safety.md` rewritten live-first: the four live leg-path
   layers (arming contract; `setpoint_pump` per-frame clamp; the firmware
   `MAX_DEVIATION` guard as the owner-confirmed leg-path authority; the
   sustained-confinement command gate) each cite the single place their
   thresholds live instead of duplicating numbers — the drift class that
   staled the old file. The parked MPC chain (motor_guard/motion_bridge/
   can_node) moved to an explicit dormant-record section, tables kept for
   the revival.
2. `controller/demo` → `sim/juggle_planner` path refresh in the three
   active plans still naming the old location (36 refs: bb-led 31,
   bb-online-tilt 4, mvp-bringup 1). `refactor-2026-07.md`'s own mentions
   are the record of the move and stay.
3. The four untracked workflow runner scripts (the anomaly/canbridge/mvp
   phase-runners + sitting-analysis)
   rippled for the SHA-backfill retirement (no more `commits: <pending>`
   frontmatter or backfill steps; Logbook-Entry trailer stated as
   canonical) and `anomaly-phase-runner.js`'s pointer to the merged-away
   `project_hand_sensor_recon` memory re-aimed at its surviving section.
   Untracked files: edited in place, not committed.
4. `sim/Dockerfile` + `compose.yaml` + `.dockerignore` deleted (owner
   decision; pre-existing broken — the compose stack never survived the
   sim import-root change and is unused). CLAUDE.md's docker command line
   removed with it, and its stale "CAN encoding must match can_node.py"
   convention re-pointed at the firmware + protocol_config authority
   (audit catch — can_node.py was deleted 2026-07-06).
5. `plans/active/PROMPT-err-timeout-hand-path.md` authored: self-contained
   prompt for a fresh session to attribute (offline pattern analysis →
   `tx_write_fail` counter flash → bench stream A/B) and then fix the
   ~50% hand-dispatch ERR_TIMEOUT epidemic, with the catch-latch
   reconciliation fence stated as hard.

**Verification**: docs/config-only sweep; the tests reading these paths
(`test_plans_index`, `test_logbook_search`, `test_logbook_front_matter`,
`test_choreography_map`) plus the full gate — triple in the commit
messages. `/audit --unstaged` run before commit per the narrative-change
rule.
