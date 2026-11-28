---
id: BUG-158
kind: bug
title: First flight of the per-release-kind plant model (BUG-138) and process noise on b (BUG-139)
status: complete
created: '2026-11-28'
updated: '2026-11-28T02:08:43Z'
owners: []
priority: high
effort: s
hardware: robot
owner: none
radius: subsystem
diagnosis: confirmed
area: learner
blocked_by: []
links:
- logbook/2026-11-28-first-flight-of-the-learner-merge-and-the-15-mm-catch-authority-trial.md
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

- verdict: pass
- notes: First, third, fourth, fifth, sixth attempts ended with Ball2 colliding with the Platform frame during its feed throw. Second attempt worked perfectly. Seventh ended with MAX_DEVIATION. The next few all ended with Ball2's feed hitting the Platform. Final attempt ended with a mid-air collision.

Analysis (bags 2026-11-28_12-21-52 and 2026-11-27_19-13-16; owner verdict pass 2026-11-28):

- Ten `learner:` lines, one per Attempt, all "unusable 0", "rejected 44"; the first reads K 1.0820 and 5811 rows (4684 free_cup, 1083 carried). No ADMISSIBILITY refusal and no NoAdmissible. The refusals in the log (RESEND-REFUSED CATCH_AXIS, LIMIT_JERK, TRACKER-IDENTITY-REFUSED, BUMP-DECLINED) are tracker re-aims of a Catch, not Learner commands.
- Replaying the Memory through the merged Learner reproduces the logged commands to about 1 mm. Largest offset 17.4 mm lateral and 6.9 mm Apex (caps 30 and 20). The merged and pre-merge code differ by under 0.5 mm in command on the same Memory.
- 52 rows written (23 at Site -62.5, 29 at +62.5), all accepted. Landing error per Site is the same as before the merge within the sample's resolution (standard error about 4 mm): sd about 19 to 24 mm per axis in both bags.
- The Attempts that ended early did so on Ball 2's Feed (BUG-160, BUG-161) and on a Hand MAX_DEVIATION (Attempt 7, the known bridge-stall race, see FEAT-40 and BUG-89).

## Progress

- 2026-11-28: First flight passed (owner verdict pass): Memory loaded with 0 unusable rows, 10 Attempts ran, no Learner refusal.
