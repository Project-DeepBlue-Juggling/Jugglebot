---
id: JB-162
kind: feature
title: 'P4 ramp Sitting: Columns canary at 600 / 8000 and 20 A, then the one-ball oval dwell ladder 0.40 / 0.35 / 0.30'
status: in_progress
created: '2026-11-03'
updated: '2026-11-03T21:54:15Z'
owners: []
priority: high
effort: m
hardware: robot
owner: hands
radius: subsystem
area: skill-stack
blocked_by: []
links:
- plans/active/two-ball-oval.md
- logbook/2026-11-04-sitting-debrief.md
parent: two-ball-oval
filed_by: claude
---

P4 ramp of plan `plans/active/two-ball-oval.md`: the first Sitting at the P4 working point (600 / 8000 / 200000, legs 20 A under the 30 A trip; box 113 cells, gate_hash 2605fc1bd25e; logbook `2026-11-03-p4-working-point-600-8000-20a`). Five Columns canary Attempts, then the one-ball oval dwell ladder 0.40 / 0.35 / 0.30. Closes when the plan's P4 gate is met.

## Sitting

- setup: armed, bb-fed, bridge-cycled
- why: P4 ramp of plan two-ball-oval.md: the first flight at the 600 / 8000 working point with the legs at 20 A (box gate_hash 2605fc1bd25e); five Columns canary Attempts show nothing regressed, then the one-ball oval dwell ladder finds the shortest dwell that holds.
- do:
  - Fly this row FIRST, before every other robot row: since cf35526c every goal runs at 600 / 8000 and 20 A. The 2026-11-03 Sitting flew 1 of the 5 canary Attempts (20/20, peak leg iq 11.7 A) and the oval to dwell 0.35 without a pass (debrief 2026-11-04: carried throws land long, the carry's deceleration ends 25-30 ms before release).
  - Before arming: confirm the launch log shows BridgeIdentity FW 38 (proto 11), the box line gate_hash 2605fc1bd25e, and the Sitting limits 600 / 8000 / 200000 (none of the three was in the 2026-11-03 launch log; read them from the GUI or `ros2 param get` and note where they came from).
  - Rehearse before arming: cup empty, Ball Butler unpowered, send `ros2 action send_goal /jugglebot/juggle jugglebot_interfaces/action/Juggle "{pattern: oval_1ball_fed, apex_m: 1.00, separation_mm: 150.0, dwell_s: 0.40, num_cycles: 4}" --feedback` once. The accept message may name only the cup and Ball Butler refusals (no box, dwell or limit refusal) and nothing moves.
  - Columns canary: GUI Columns, Sep. 125, Apex 0.95, Dwell 0.27, Throws 21. Five Attempts (the 2026-11-03 one does not count toward this Sitting's five).
  - Watch the leg current throughout: cancel the goal if any leg iq sample reaches 19 A (1 A under the 20 A clamp; proposed 2026-11-03, owner to confirm; the 2026-11-03 peak was 11.7 A).
  - One-ball oval, on its own plant: `ros2 param set /skill_node plant_id p4-oval-<date>` (refused while a goal runs) and read it back from the goal start line.
  - FIRST one Attempt at dwell 0.50: `{pattern: oval_1ball_fed, apex_m: 1.00, separation_mm: 150.0, dwell_s: 0.50, num_cycles: 32}` (cup empty, Ball Butler loaded). It separates the new leg point from the dwell: dwell 0.50 landed 46 +/- 10 mm long at 350 / 5000 on 2026-10-31; about the same here clears P4 of the long bias.
  - Then re-fly the 2026-11-03 ladder unchanged (owner 2026-11-04): dwell 0.40 at Apex 1.00 (oval_1ball_fed, then oval_1ball), then dwell 0.35 at Apex 1.00 after one Attempt with no leg LIMIT, guard or HAND_LIMIT end.
  - Fly dwell 0.30 at Apex 1.10, not 1.00, and only if 0.35 holds: the box leaves 1 mm of +x landing room at 1.00 / 0.30 and 25 mm at 1.10 / 0.30.
- pass: the five Columns Attempts each read `COMPLETED (20/20 caught)` with no leg LIMIT end and peak leg iq under 20 A; then the one-ball oval sustains 30 consecutive Catches in one Attempt at the shortest dwell flown (plan P4 gate).
- record: caught of thrown and the end code per Attempt; the CATCH leg-velocity refusal count across the canary (JB-24 expects none at 600 mm/s); peak leg iq per leg per Attempt from the bag (the 20 A measurement); the shortest dwell reaching 30; the raw long bias per dwell (landed minus the command u), from rest and carried; the LIMIT_JERK refusal count; the oval plant_id; the bag and launch log paths; commit docs/records.md.
- minutes: 50

## Sitting result (2026-11-03 22:47)

- verdict: pending
- notes: Many attempts of the patterns described in 6 and 7. Didn't run 8, as 6 and 7 had no successful runs, thrown to completion. There were a few LANE_REFUSED, MAX_DEVIATION, and FRAME_OFFSET errors in the attempt, but most attempts ended due to a messy throw being uncatchable. No Lef LIMIT ends, with peak leg current of ~10 A. 'Pending' because we didn't technically pass, but we also didn't observe the fail conditions.

## Progress

- 2026-11-04: Flown at the Sitting of 2026-11-03 (22:29), not passed: Columns canary 1 of 5 Attempts flown (COMPLETED 20/20, no leg LIMIT end, peak leg iq 11.7 A on leg 2, 0 LIMIT_VEL/ACC refusals, 1 LIMIT_JERK re-aim refusal 510456 vs 200000 that did not matter); one-ball oval best 16/16 at dwell 0.40, 0.35 best 2/2, 0.30 not flown.
  - Dominant end MISSED_CATCH: carried throws land long (+76 mm raw at 0.40, +122 at 0.35; +48 from rest), the carry's deceleration ending 25-30 ms before release with the ~6 Hz Platform ring swinging forward at separation; learner pinned at its 30 mm authority from goal 4. 3 ends were firmware scheduler STOPs on a clean host stream.
  - Next: one Attempt at dwell 0.50 first, then the same 0.40 / 0.35 ladder (owner 2026-11-04).
