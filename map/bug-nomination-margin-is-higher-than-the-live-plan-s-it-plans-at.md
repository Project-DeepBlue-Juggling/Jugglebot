---
id: JB-185
kind: bug
title: 'Nomination margin is higher than the live plan''s: it plans at nominal Sites without the re-aim travel (0.194 nominated, 0.075 live at Sep. 170)'
status: complete
created: '2026-11-06'
updated: '2026-11-06T06:14:51Z'
owners: []
priority: medium
effort: m
hardware: none
owner: decision
radius: subsystem
diagnosis: confirmed
area: skill-stack
blocked_by: []
links:
- logbook/2026-11-06-sitting-debrief.md
- plans/active/learner-generalisation.md
- logbook/2026-11-06-nomination-reaim-figure.md
parent: learner-generalisation
filed_by: claude
---

The goal-accept nomination (P3b of plan `learner-generalisation.md`) reports a margin that the live plan does not have, in two measured cases.

## Evidence
- Hardware, 2026-11-06, Columns Apex 1.00 / Sep. 170: nominated worst margin 0.194; the live admissibility checks on the same Segments read 0.075 to 0.258. The nomination plans every Segment at the nominal Sites, and the live Catch aims carried the 10 mm outward re-aim at each end (transits of 187 mm, not 170).
- Sim, 2026-11-06: a standing Columns at Apex 0.80 nominates OK at margin 0.06 and then fails an install on all five seeds (`logbook/2026-11-06-learner-generalisation-p4-beat-event-stream.md`).
- The nomination's own docstring says it plans at the identity command and does not see the Learner.

## Wanted
- The nomination walks each Segment at the extremes of the lateral aim authority as well as at the nominal Sites, so its margin is a floor on what the live check will see.
- This moves the acceptance edge (a thin-margin goal accepted today could be refused), so it is an owner decision, not a silent fix. On 2026-11-06 the owner declined a new bound on transit aggressiveness (see the Leg separation Item); this Item is about the printed margin being honest.

## Progress

- 2026-11-06: Closed 2026-11-06 by option C (owner): acceptance unchanged on the nominal margin; the skills/nominate reply and the Juggle panel also show the worst margin with every Catch aim at the clamp corner (10 mm outward, 30 mm across, both signs). Goal accept costs what it did. Sep. 170: 0.211 nominal, 0.016 at the limit (live minimum 0.075).
