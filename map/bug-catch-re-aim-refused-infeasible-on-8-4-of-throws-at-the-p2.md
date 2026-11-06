---
id: JB-188
kind: bug
title: Catch re-aim refused INFEASIBLE on 8.4 % of Throws at the P2 Sitting, 15 of 69 on the turned pair
status: inbox
created: '2026-11-06'
updated: '2026-11-06T05:58:57Z'
owners: []
priority: medium
diagnosis: unknown
area: skill-stack
blocked_by: []
links:
- logbook/2026-11-06-sitting-debrief-2.md
filed_by: claude
---

`RESEND-REFUSED INFEASIBLE ... QP infeasible: unbounded dual step` on 29 of 346 Throws at the Sitting of 2026-11-06 15:53, against 2.3 % at the 13:19 Sitting and 0 to 0.4 % in the endurance logs the same afternoon.

## Findings

- Concentrated on `columns_turned` at 90 degrees (15 of 69 Throws).
- The one-ball goal at Apex 0.70 refused 12 of 20 as WINDOW_TOO_SHORT and one at Apex 1.00 refused 8 as SPLICE_TOO_LATE.
- The refused Throws did not land or seat worse, and none preceded a miss in the one-ball goals; the Hand stays on the earlier aim.
- The reason string exists since 2026-09-22.

## Open questions

Not diagnosed: why the turned pair's re-aim QP is infeasible more often.

## Evidence

`temp/probes/sitting_debrief_2026-11-06b/unit_stack.md` section 5 (P2 log L657, L879, L1114, L7428).
