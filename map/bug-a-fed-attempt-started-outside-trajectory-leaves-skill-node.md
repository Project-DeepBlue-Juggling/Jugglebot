---
id: BUG-113
kind: bug
title: A fed Attempt started outside TRAJECTORY leaves skill_node stuck with an Attempt running, and Stop is ignored
status: in_progress
created: '2026-11-18'
updated: '2026-11-18T03:42:04Z'
owners: []
priority: high
effort: s
hardware: none
owner: decision
radius: local
diagnosis: confirmed
area: skill-stack
blocked_by: []
links:
- logbook/2026-11-18-sitting-debrief.md
filed_by: sitting-debrief 2026-11-18
---

Sitting of 2026-11-18, launch 3, 11:55:29-11:58:00 (temp/logs/launch_2026-11-18_3.log lines 3738-3778). Both defects are in main (the candidate changed only a parameter in skill_node).

1. A fed Attempt was started while the orchestrator was still in STANDBY after a guard-latch recovery. The executor ended itself (ABORTED_MODE_CHANGED) after the Ball Butler announcement had claimed the Feed context (`_reload_ctx`); the compile then declined the executor swap and that branch never releases the claimed context.
   - The Feed timeout skips claimed contexts and the end line is silent while a context exists, so no ENDED line was logged, every new Juggle was refused (an attempt is already running, 6 times) and the Juggle action never returned. The operator had to Ctrl-C the launch.
2. orchestrator_node: the rejection of a NEW goal clears the stored goal handle, wiping the handle of the still-live goal, so Stop answered: Juggle stop ignored: no attempt is running.

Owner question, not decided: should a fed Attempt be refused at accept when the orchestrator is not in TRAJECTORY, instead of starting and ending itself?

## Progress

- 2026-11-18: Both defects fixed at the 2026-11-18 debrief (declined swap releases the claimed context; a rejection no longer clears the live goal handle), with a failing-first test each. Open: the owner question on refusing a fed Attempt outside TRAJECTORY.
