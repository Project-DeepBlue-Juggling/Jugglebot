---
title: "The OUTCOME line reports the catch's CONTACT PHASE (seat − scheduled landing) — the one number that separated smooth catches from double-contact ones"
type: change
date: 2026-09-18
status: done
phase: "two-ball-skill-stack — R3"
related_plan: two-ball-skill-stack.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
---

## What changed

`_PendingOutcome.t_seat_s` records the tick on which the SEATED latch fired, and
`_finalise_outcome` appends `seat=+0.104 s vs scheduled landing` to the OUTCOME line. Reported,
never learned (`caught` stays report-only too; the learner regresses on the landing alone).

## Why

The 2026-09-17 23:49 sitting (apex 0.6 m, 4-throw chains, bag `2026-09-17_23-49-23`): the operator
called attempt 1 "by far the smoothest" and every later attempt "the ball hits the hand before it
moves, bounces off, then the hand moves down and the ball follows". `scratchpad/late_catch_probe.py`
over the bag: the four smooth catches seated at **+0.104 s** after the scheduled landing (cup at
the bottom of its dive, 0.3 m/s); the bad ones at **+0.015 s** (cup still at 8.6 rev, then diving
away at 2.3 m/s) and **+0.338 s** (the bounce re-seating while the cup was already rising). One
number, perfect separation — and the executor had it and threw it away. It is the quantity the
operator judges; it now lands in the log for every throw. The decision on what to DO about the
phase (the learner's parameterisation, the catch aim source, the cup's downward-acceleration
bound) is the owner's, filed in the plan's R3 carried items and the paper-comparison review.

## Verification

(2026-09-18, `pytest tests/motion/test_skills_executor.py tests/ros/test_skill_node.py -q`):
163 passed. (2026-09-18, `./run_tests.sh`, `temp/logs/contact_phase_gate_20260918.log`):
**PASS — 6219 passed, 9 skipped, 1 xfailed in 224.29 s; serial 3 passed.**
