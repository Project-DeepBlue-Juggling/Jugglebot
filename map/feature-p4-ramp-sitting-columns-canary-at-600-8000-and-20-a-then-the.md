---
id: JB-162
kind: feature
title: 'P4 ramp Sitting: Columns canary at 600 / 8000 and 20 A, then the one-ball oval dwell ladder 0.40 / 0.35 / 0.30'
status: inbox
created: '2026-11-03'
updated: '2026-11-03T11:47:33Z'
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
parent: two-ball-oval
filed_by: claude
---

P4 ramp of plan `plans/active/two-ball-oval.md`: the first Sitting at the P4 working point (600 / 8000 / 200000, legs 20 A under the 30 A trip; box 113 cells, gate_hash 2605fc1bd25e; logbook `2026-11-03-p4-working-point-600-8000-20a`). Five Columns canary Attempts, then the one-ball oval dwell ladder 0.40 / 0.35 / 0.30. Closes when the plan's P4 gate is met.

## Sitting

- setup: armed, bb-fed, bridge-cycled
- why: P4 ramp of plan two-ball-oval.md: the first flight at the 600 / 8000 working point with the legs at 20 A (box gate_hash 2605fc1bd25e); five Columns canary Attempts show nothing regressed, then the one-ball oval dwell ladder finds the shortest dwell that holds.
- do:
  - Fly this row FIRST, before every other robot row: since cf35526c every goal runs at 600 / 8000 and 20 A, so the Columns canary is the first evidence the new point is safe.
  - Before arming: confirm the launch log shows BridgeIdentity FW 38 (proto 11), the box line gate_hash 2605fc1bd25e, and the Sitting limits 600 / 8000 / 200000 (the launch default since cf35526c).
  - Rehearse before arming: cup empty, Ball Butler unpowered, send `ros2 action send_goal /jugglebot/juggle jugglebot_interfaces/action/Juggle "{pattern: oval_1ball_fed, apex_m: 1.00, separation_mm: 150.0, dwell_s: 0.40, num_cycles: 4}" --feedback` once. The accept message may name only the cup and Ball Butler refusals (no box, dwell or limit refusal) and nothing moves.
  - Columns canary: GUI Columns, Sep. 125, Apex 0.95, Dwell 0.27, Throws 21. Five Attempts.
  - Watch the leg current throughout: cancel the goal if any leg iq sample reaches 19 A (1 A under the 20 A clamp; proposed 2026-11-03, owner to confirm).
  - One-ball oval ladder, on its own plant: `ros2 param set /skill_node plant_id p4-oval-<date>` (refused while a goal runs) and read it back from the goal start line.
  - Fly `{pattern: oval_1ball_fed, apex_m: 1.00, separation_mm: 150.0, dwell_s: 0.40, num_cycles: 32}` (cup empty, Ball Butler loaded). Move to dwell 0.35 at Apex 1.00 after one Attempt with no leg LIMIT, guard or HAND_LIMIT end.
  - Fly dwell 0.30 at Apex 1.10, not 1.00: the box leaves 1 mm of +x landing room at 1.00 / 0.30 and 25 mm at 1.10 / 0.30.
- pass: the five Columns Attempts each read `COMPLETED (20/20 caught)` with no leg LIMIT end and peak leg iq under 20 A; then the one-ball oval sustains 30 consecutive Catches in one Attempt at the shortest dwell flown (plan P4 gate).
- record: caught of thrown and the end code per Attempt; the CATCH leg-velocity refusal count across the canary (JB-24 expects none at 600 mm/s); peak leg iq per leg per Attempt from the bag (the 20 A measurement); the shortest dwell reaching 30; the oval plant_id; the bag and launch log paths; commit docs/records.md.
- minutes: 50

## Sitting result (2026-11-03 22:47)

- verdict: pending
- notes: Many attempts of the patterns described in 6 and 7. Didn't run 8, as 6 and 7 had no successful runs, thrown to completion. There were a few LANE_REFUSED, MAX_DEVIATION, and FRAME_OFFSET errors in the attempt, but most attempts ended due to a messy throw being uncatchable. No Lef LIMIT ends, with peak leg current of ~10 A. 'Pending' because we didn't technically pass, but we also didn't observe the fail conditions.
