---
id: IDEA-31
kind: idea
title: 'The oval on the shared Memory: compile the oval on compile_beats, then fly it on plant jugglebot'
status: inbox
created: '2026-11-28'
updated: '2026-11-28T16:27:31Z'
owners: []
priority: low
effort: l
hardware: robot
owner: none
radius: subsystem
area: learner
blocked_by: []
links: []
parent: learner-generalisation
filed_by: claude-e7
---

Left over from the learner-generalisation plan's P5 item 4 and carried by learner-efficacy E7. The oval is not flown on the one shared Memory under that plan: it needs compile_oval built on compile_beats, which does not exist. BUG-138 (lateral law per release kind) is the precondition and is merged 2026-11-28. Once compile_oval exists, fly the oval on plant jugglebot and check that carried rows no longer move the Columns command.

## Feasibility (2026-11-29)

- What it would take: a `compile_oval` built on `compile_beats` (the oval's per-Beat schedule expressed the way Columns and JP-11 are compiled), then a Sitting flying the oval on plant `jugglebot` (the shared Memory) and checking that carried rows are accepted and its commands stay inside the Learner's cap.
- Preconditions: BUG-138 (lateral law per release kind) is on main since 2026-11-28; BUG-153 (a cold release kind cannot learn the oval's 179 mm offset) matters here only if the carried kind starts cold.
- Effort: l (a compiler and its sim gate, then a Sitting). Hardware: robot. Owner: none for the compiler, hands for the flight.
- One risk: the oval's carried Throws are the largest lateral offsets the shared Memory has seen, so a compile that is correct in sim can still move the Learner's carried-kind command for Columns. The Sitting has to read the Columns margins after the oval flies.
