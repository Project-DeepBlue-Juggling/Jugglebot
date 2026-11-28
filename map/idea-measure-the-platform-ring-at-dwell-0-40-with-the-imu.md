---
id: IDEA-27
kind: idea
title: Measure the Platform ring at Dwell 0.40 with the IMU
status: in_progress
created: '2026-11-23'
updated: '2026-11-27T04:11:20Z'
owners: []
priority: low
effort: s
hardware: robot
owner: hands
radius: local
area: skill-stack
blocked_by: []
links:
- logbook/2026-11-23-ring-study-release-timing-redone-on-the-bridge-clock.md
- plans/parked/carry-ring-zvd.md
filed_by: claude
night_skip: sitting
night_skip_on: '2026-11-28'
night_skip_note: One Dwell 0.40 Attempt rang and one did not; the Sitting section now flies four.
---

Decides whether plans/parked/carry-ring-zvd.md comes back or is archived (owner, 2026-11-23). At Dwell 0.50 the IMU, on the bridge clock, shows the x ring quiet at the measured separation (about 4 ms after the planned release) and surging to about 19 mm/s 42 ms after it (logbook 2026-11-23-ring-study-release-timing-redone-on-the-bridge-clock, which corrects the earlier 43 mm/s). Dwell 0.40 is where the old 76 mm long landings were seen and is unmeasured.

The owner doubts Dwell 0.40 is reachable without other changes. A refusal at the rehearsal is itself the answer: the plan is then archived.

Fly it on Platform FW 15 (BUG-132 fixed) and analyse on the bridge clock: check the capture with `tools/probes/imu_clock_check.py` and time the other streams with `tools/probes/_bag_time.py`.

## Sitting

- setup: level, armed, bb-fed
- why: Attempts at Dwell 0.40 with the ZVD rows off tell whether the ring after release grows as the old landings suggested. It rides on any robot Sitting.
- do:
  - After Level, rehearse (moves nothing): `ros2 service call /skills/nominate jugglebot_interfaces/srv/SetString "{data: 'oval_1ball_fed,apex_m=1.00,separation_mm=150.0,dwell_s=0.40'}"`. If it reports a refusal, write the refusal down and skip the next two steps.
  - `ros2 param set /skill_node plant_id ring-dwell040-<date>`, then read it back.
  - Fly four Attempts, the last one after a pause of two minutes, waiting for "recording" between them: `ros2 action send_goal /jugglebot/juggle jugglebot_interfaces/action/Juggle "{pattern: oval_1ball_fed, apex_m: 1.00, separation_mm: 150.0, dwell_s: 0.40, num_cycles: 8}" --feedback`. Wait for "recording" on the GUI IMU panel.
  - `ros2 param set /skill_node plant_id jugglebot` and read it back.
- pass:
  - Either the rehearsal refuses Dwell 0.40, or the Attempt's "Platform IMU upload finished" line reads "complete yes, crc ok". The ring numbers are computed at the debrief.
- record:
  - The rehearsal's answer, and the caught count. The upload line is in the launch log.
- minutes: 15

## Sitting result (2026-11-27 15:08)

- verdict: pass
- notes (split into bullets):
  - First attempt ended with a missed throw. Second worked perfectly. (sheet 2026-11-27-robot, step 16) [derived: Both Dwell 0.40 Attempts (13:57:49, 1 of 2 caught; 13:58:24, 8 of 8) ended with an upload line reading complete yes, crc ok (captures 33D1 and 33D2).
  - The rehearsal answer is not in the log and ring numbers: the 8 of 8 Attempt shows no growth (peak 15 +/- 9 mm/s against 13 +/- 3 at Dwell 0.50 the same day, n 7), the 1 of 2 Attempt rang 58 to 74 mm/s at release.]

## Progress

- 2026-11-27: Sitting 2026-11-27 ring numbers: the 8 of 8 Attempt at Dwell 0.40 shows no growth (peak 15 +/- 9 mm/s against 13 +/- 3 at Dwell 0.50 the same day, n 7), but the 1 of 2 Attempt rang 58 to 74 mm/s at release and missed both Catches. One Attempt cannot decide; the Sitting section now flies four, then unpark or archive carry-ring-zvd.
