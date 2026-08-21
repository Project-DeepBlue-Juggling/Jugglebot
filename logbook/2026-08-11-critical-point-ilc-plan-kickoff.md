---
title: "Critical-point ILC arc opened — task-level iterative learning on the throw/catch events, in its own worktree"
type: refactor
date: 2026-08-11
status: in-progress
phase: "critical-point-ilc — plan kickoff (pre-Phase-0)"
related_plan: critical-point-ilc.md
files_changed:
  - plans/active/critical-point-ilc.md
  - plans/active/INDEX.md
  - logbook/2026-08-11-critical-point-ilc-plan-kickoff.md
  - logbook/INDEX.md
subsystem:
  - motion
  - tracking
tags:
  - docs
---

# Critical-point ILC arc opened

Planning change only, no code. `plans/active/critical-point-ilc.md` is created:
task-level iterative learning control on the throw and catch critical points,
adapted from Suresh & Atkeson's flying-knots method (arXiv:2602.21302, reference
code `github.com/krish-suresh/flying_knots_public`). The owner brought the paper
to the 2026-08-11 session; the analysis that led here concluded the toss aim map
is the method's 2-DOF special case, and the general form is the growth path for
coupled task errors — landing position + arrival-velocity direction +
catch-contact softness responding jointly to aim, stroke velocity and timing.

**Why a separate worktree.** Owner direction: develop in parallel with the
in-flight arcs without touching their tree. Branch `critical-point-ilc`,
worktree `/home/jetson/Desktop/Jugglebot-ilc`, base `5e046cc` — the tilt-cal
arc wrap-up, the last commit before both the catch-robustness/toss-selftuning
arc (`2328d0a`) and the bridge-temporal arc (`9099d1e`) began. Consequence,
recorded in the plan's collaborator notes: the toss-record substrate this plan
rides (record schema, miner, aim map, trim) is NOT in this tree — merging it
in is gate G-3, before Phase-0 implementation.

**Owner decisions captured in the plan (2026-08-11).** (a) Possession truth is
the hand ball sensor tri-state, never the mocap caught/dropped verdict — the
owner distrusts it. (b) Catch softness via hand-drive channels
(`hand_telemetry` `vel_meas`/`iq_meas`), probe-first; hand mocap
markers are third preference behind the ballistic backcast. (c)
Arrival-velocity direction joins the task-error vector. (c′) The softness
channels are cadence-split by the pre-commit audit: `vel_meas` is truly
100 Hz, but `iq_meas` reaches the bag through the on-change/1 Hz DIAGNOSTIC
path republished from cache, so Phase 0b opens with a cadence census before
any noise question. (d) The arc follows
bridge-temporal-trustworthiness (hardware learning needs a repeatable plant)
but is NOT sequenced behind the toss-selftuning capture campaign — the
substrate is required, the map values are not, since applied aim is recorded
per toss.

**The one technical commitment worth flagging**: sensitivities come from
finite differences through the production planning chain, never a symbolic
re-derivation — the `JB_OP_HAND_CATCH_PRIME_REV` drift (3.2 mm, silent for
the life of the constant) is the canonical cost of a second copy of stroke
math, and a model built for learning would be exactly that.

## Verification

Docs-only change, but inside the test surface: `tests/sim/test_plans_index.py`
pins the plans INDEX ↔ `plans/active/` correspondence (both directions — the
new row's link target and the reverse md-name scan), and the logbook
front-matter tests parse this entry. Gate (`./run_tests.sh -q`, run
2026-08-11 in the worktree at this change-set): **RESULT: PASS — 4270
passed, 5 skipped, 0 failed; parallel 195 s rc=0, serial 8 s rc=0 (the
phase collects no tests — every serial-marked test is `nightly`), total
203 s.** The wrapper's `-q` mode prints no count line, so the
counts are tallied from the captured per-test progress output of the same
run; the ~700-test gap to the `mvp-trajectory-bringup` gate
(`./run_tests.sh`, run 2026-08-11: 4974 passed in 232 s, cited from
`logbook/2026-08-11-bridge-temporal-trustworthiness-kickoff.md` on that
branch) is the toss-selftuning/possession build that postdates this branch's
base. Scoped re-runs after this entry's final edit
(`pytest tests/sim/test_plans_index.py tests/sim/test_logbook_front_matter.py
tests/sim/test_logbook_search.py -q`, 2026-08-11): recorded in the commit
message. `/audit --unstaged` ran before the commit (multi-document narrative
gate, four narrative files); findings and dispositions are recorded in the
commit message.
