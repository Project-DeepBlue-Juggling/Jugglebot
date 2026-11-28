---
id: BUG-155
kind: bug
title: A full-turn rotating Columns goal can block the goal accept for minutes while it nominates
status: inbox
created: '2026-11-28'
updated: '2026-11-28T00:15:11Z'
owners: []
priority: low
effort: s
hardware: none
owner: none
radius: local
diagnosis: confirmed
area: skill-stack
blocked_by: []
links: []
parent: pattern-progression-columns-to-centre-and-orbit
filed_by: claude
---

Found 2026-11-27 building JP-11 (IDEA-12). The goal accept nominates the whole Attempt in the goal callback. A 38-Throw rotating Columns nomination took 6 min 48 s on a loaded box (about a minute idle; one 74-Throw case took 47 min overnight, not looked into). The accept is instant only when /skills/nominate ran first with the same fields and count, which the Sitting section requires. Possible fixes: nominate a bounded prefix at the accept, or split the three flight rungs across processes.
