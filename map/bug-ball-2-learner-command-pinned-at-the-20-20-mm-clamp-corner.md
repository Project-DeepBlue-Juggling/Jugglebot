---
id: JB-139
kind: bug
title: Ball 2 learner command pinned at the (-20,-20) mm clamp corner in 37 of 40 straight Columns rows (BOX-EXPECTED-LANDING-UNMET u=(-20,-20))
status: inbox
created: '2026-11-01'
updated: '2026-11-01T12:19:19Z'
owners: []
priority: medium
diagnosis: confirmed
area: learner
blocked_by: []
links:
- map/pathway-learner-generalisation.md
filed_by: claude
---

Found at the 2026-11-01 Sitting: the Ball 2 learner command sits at the (-20, -20) mm clamp corner in 37 of 40 straight Columns rows (BOX-EXPECTED-LANDING-UNMET u=(-20,-20)).

## Findings

- u1 (y) = -0.020 in 37/40 rows, u0 (x) = -0.020 in 13/40, both pinned in 12/40.
- It does not predict the HAND_LIMIT_ACC ends (JB-120): lateral u moves the Hand peak by under 1 point.
- Relevant to the learner generalisation work (JB-38).
