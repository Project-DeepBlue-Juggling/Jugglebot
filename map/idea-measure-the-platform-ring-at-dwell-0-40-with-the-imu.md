---
id: IDEA-27
kind: idea
title: Measure the Platform ring at Dwell 0.40 with the IMU
status: inbox
created: '2026-11-23'
updated: '2026-11-23T05:07:19Z'
owners: []
priority: low
effort: s
hardware: robot
owner: hands
radius: local
area: skill-stack
blocked_by: []
links:
- plans/parked/carry-ring-zvd.md
filed_by: claude
---

Decides whether plans/parked/carry-ring-zvd.md comes back or is archived (owner, 2026-11-23). At Dwell 0.50 the IMU shows the x ring surging to about 43 mm/s 45 ms after the planned release, with 7 to 8 mm/s of spread (logbook 2026-11-23-platform-ring-identification-from-the-imu). Dwell 0.40 is where the old 76 mm long landings were seen and is unmeasured.

The owner doubts Dwell 0.40 is reachable without other changes. A refusal at the rehearsal is itself the answer: the plan is then archived.

Analyse only after BUG-132 (IMU timestamps early) is fixed, or with the shift estimated per capture.

## Sitting

- setup: level, armed, bb-fed
- why: one Attempt at Dwell 0.40 with the ZVD rows off tells whether the ring after release grows as the old landings suggested. It rides on any robot Sitting.
- do:
  - After Level, rehearse (moves nothing): `ros2 service call /skills/nominate jugglebot_interfaces/srv/SetString "{data: 'oval_1ball_fed,apex_m=1.00,separation_mm=150.0,dwell_s=0.40'}"`. If it reports a refusal, write the refusal down and skip the next two steps.
  - `ros2 param set /skill_node plant_id ring-dwell040-<date>`, then read it back.
  - Fly one Attempt: `ros2 action send_goal /jugglebot/juggle jugglebot_interfaces/action/Juggle "{pattern: oval_1ball_fed, apex_m: 1.00, separation_mm: 150.0, dwell_s: 0.40, num_cycles: 8}" --feedback`. Wait for "recording" on the GUI IMU panel.
  - `ros2 param set /skill_node plant_id jugglebot` and read it back.
- pass:
  - Either the rehearsal refuses Dwell 0.40, or the Attempt's "Platform IMU upload finished" line reads "complete yes, crc ok". The ring numbers are computed at the debrief.
- record:
  - The rehearsal's answer, and the caught count. The upload line is in the launch log.
- minutes: 8
