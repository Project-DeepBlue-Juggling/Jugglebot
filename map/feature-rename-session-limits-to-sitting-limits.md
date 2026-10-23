---
kind: feature
title: Rename "session limits" to Sitting limits (glossary term)
status: inbox
created: '2026-09-21'
updated: '2026-10-24T13:05:00Z'
owners: []
priority: medium
blocked_by: []
links:
- https://github.com/Project-DeepBlue-Juggling/Jugglebot/issues/17
filed_by: harrison
---

`CONTEXT.md` defines **Sitting** (a powered period at the robot) and reserves **Session** for a Claude Code conversation, so the live velocity / acceleration / jerk limits are **Sitting limits**. The code still says "session limits". The glossary is deliberately ahead of the code here.

## Scope (surveyed 2026-09-21)

Roughly 235 hits on `mvp-trajectory-bringup` and more on `skill-stack`: the identifier `with_session_limits`, comments in `config/hardware_config.yaml` (regenerate with `python config/generate_config.py`), `trajectory_node.py`, `motion/trajectory/limits.py`, `motion/trajectory/feasibility.py`, `motion/skills/admissible.py`, `sim/skills_gate.py`, the probes, and about 30 test files.

## Constraints

- Do it on `skill-stack`, not `mvp-trajectory-bringup`: skill-stack is 41 commits ahead and the rename would conflict across all of them.
- `feasibility.py` is touched. Any edit to `feasibility.py` or `segments.py` requires the admissible-box re-sweep (about 29 min), so bundle this rename with the next change that forces a re-sweep anyway.
- Wait until no other session has uncommitted edits to those files.
- Grep-counted rename per CLAUDE.md: list every reference, rename, verify the count of the old name is zero outside history.
- Historical logbook entries and archived plans are not rewritten.
- `./run_tests.sh --full` before the commit (the change is under `sim/` and the planner path).
