---
id: FEAT-53
kind: feature
title: Extract the pure install layer from executor.py and split the rest along its seams
status: inbox
created: '2026-11-14'
updated: '2026-11-13T23:09:35Z'
owners: []
priority: medium
effort: l
area: skill-stack
blocked_by: []
links: []
filed_by: claude
---

Split `motion/skills/executor.py` along the seams the 2026-11-13 maintainer review found, starting with the pure install layer. The file is 8,016 lines, of which the reviewer counted about 3,400 as code; the rest is docstrings, comments and 220 dated references.

## Why the install layer first

`install_segment`, `check_segment`, `_rebase_fresh_rest` and the splice-lead arithmetic (roughly lines 1619-2450) are the safety-bearing part: they decide what reaches the wire. `trajectory_node`, `solver_worker` and `nominate` use only that part, yet the solver worker process imports the whole module with its learner, bump and report dependencies. As its own module of about 800 lines it can be reviewed and tested alone.

## Seams

| Lines (approx.) | Concern |
| --- | --- |
| 66-935 | Constants and refusal codes, mostly history |
| 936-1073 | Precondition ladder (`Observations`, `precondition_refusals`) |
| 1619-2450 | Pure install layer |
| 2908-3842 | Command, learner, admissibility |
| 3842-4812 | Aim, landing, lateral clamp |
| 4954-5582 | Tick, dispatch, closing |
| 5582-5760, 6353-7299 | Outcome and evidence bookkeeping |
| 5821-6353 | Stop, survivor, Beat change |
| 7299-7835 | Re-send and bump |

## Smaller findings to fold in

- Index-keyed executor state has no single enforcement point: `_apply_beat_change` audits a 15-row table of containers, `_install_survivor_tail` (`:5958-5976`) renumbers skills and checks none of it. One per-skill state object closes the class.
- One tick takes several `Observations` snapshots (`:4988`, `:6433`, `:5118`, `:5270`); the comment at `:6452` works around it. One snapshot per tick.
- `SkillExecutor.__init__` takes about 30 parameters and sets 74 attributes.

## Risks

`tests/motion/test_skills_executor.py` is 8,919 lines; a move must keep the import surface (`ex.install_segment` and friends) so the tests do not all change in the same commit.
