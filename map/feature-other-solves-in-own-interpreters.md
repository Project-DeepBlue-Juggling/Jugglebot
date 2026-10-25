---
kind: feature
title: Move the other solves (skill_node 43 ms, go_home / go_to_pose / hold) into their own interpreters
status: inbox
created: '2026-10-24'
updated: '2026-10-24T13:05:00Z'
owners: []
priority: medium
blocked_by: []
links:
- plans/archived/skill-stack-dress-rehearsals-and-residue.md
filed_by: claude
---

The skill node's 43 ms solve and the `go_home` / `go_to_pose` / hold solves still run in the shared interpreter; the worker seam can take them (wave-3 open 7, design R8). Trigger: a stall traced to those solves.
