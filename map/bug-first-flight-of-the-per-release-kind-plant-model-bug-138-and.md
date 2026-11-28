---
id: BUG-158
kind: bug
title: First flight of the per-release-kind plant model (BUG-138) and process noise on b (BUG-139)
status: inbox
created: '2026-11-28'
updated: '2026-11-28T01:28:17Z'
owners: []
priority: high
hardware: robot
blocked_by: []
links: []
parent: learner-generalisation
filed_by: claude-e7
---

First flight of the BUG-138 plant-model change (lateral law per release kind, K shared) with BUG-139 (process noise on b), merged 2026-11-28. Nothing has flown it. On the live Memory the next launch's command moves by at most 3.9 mm, so this is a check that the Learner loads the Memory and the Attempts run, not an accuracy trial.

Fly it before the 15 mm outward Catch authority trial in BUG-78, so the two changes are not confounded.

## Sitting

- setup: level, armed, bb-fed
- why: BUG-138 and BUG-139 are on main and unflown; the owner wants a normal Columns Attempt and a nominate rehearsal against the live Memory before any other Learner-side change flies.
- do:
  - Before arming: `ros2 service call /skills/nominate jugglebot_interfaces/srv/SetString "{data: 'columns,apex_m=0.95,separation_mm=125,dwell_s=0.27'}"` and read the margins.
  - Fly 2 normal Columns Attempts (Sep. 125, Apex 0.95, Dwell 0.27, Throws 21), with the Catch authority at its default 10 mm.
- pass: the launch log's `learner:` line reports the Memory loaded with 0 unusable rows, and both Attempts run with no refusal attributable to the Learner.
- record: that `learner:` line, and caught of thrown per Attempt.
- minutes: 10

## Sitting result (2026-11-28 12:28)

- verdict: pending
- notes: First, third, fourth, fifth, sixth attempts ended with Ball2 colliding with the Platform frame during its feed throw. Second attempt worked perfectly. Seventh ended with MAX_DEVIATION. The next few all ended with Ball2's feed hitting the Platform. Final attempt ended with a mid-air collision.
