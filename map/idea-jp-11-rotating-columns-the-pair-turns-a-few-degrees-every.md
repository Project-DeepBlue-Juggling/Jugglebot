---
id: IDEA-12
kind: idea
title: 'JP-11 Rotating columns: the pair turns a few degrees every Beat'
status: in_progress
created: '2026-10-30'
updated: '2026-11-28T00:15:05Z'
owners: []
priority: medium
effort: l
hardware: robot
owner: hands
radius: cross-cutting
area: skill-stack
blocked_by: []
links:
- logbook/2026-11-27-jp11-rotating-columns-compile.md
- logbook/2026-11-27-jp11-rotating-standin-rule.md
filed_by: claude (jugglebot-f5, pattern exploration 2026-10-30)
parent: pattern-progression-columns-to-centre-and-orbit
night_skip: owner-session
night_skip_on: '2026-11-28'
night_skip_note: The 2026-11-27 day Session proposed a four-unit build for you to decide; BUG-152 blocks its sim gate.
---

Columns whose Site pair rotates about +z continuously. Pose-only.

## Numbers

- Per-Beat displacement 2r sin(d_theta): 11 / 22 / 43 mm per Beat at 5 / 10 / 20 deg with separation 125 mm, aim under 0.7 deg.
- A full turn takes 41 / 21 / 10 s.
- Offline probe: feasible under pose up to about 45 deg per Beat; ball clearance collides at 90 deg per Beat.
- Reads as rotating at 20 deg per Beat or less.

## Blocker and routes

Blocked on per-Beat Sites: the executor, admissible box and learner all assume a Pattern's two Sites are fixed for the Attempt. Two routes:

- The learner-generalisation plan P3 and P4 (online admissibility, per-Beat BeatEvent stream).
- The cheaper pattern-relative admissible-box key proved by JP-10.

## Sitting

- setup: level, armed, bb-fed
- why: plan pattern-progression.md step P8, the JP-11 Rung: does two-ball Columns keep catching while its Site pair turns about the Platform centre every Beat. First flight of the `columns_rotating` Pattern and of its command-time check (I-ADMIT-1 rule iv).
- do:
  - Rehearse each step, nothing moves, and wait for each reply (up to several minutes on a loaded box): `ros2 service call /skills/nominate jugglebot_interfaces/srv/SetString "{data: 'columns_rotating,apex_m=0.95,separation_mm=125,dwell_s=0.27,num_cycles=38,step_deg=10'}"`, then the same with `num_cycles=74,step_deg=5` and with `num_cycles=20,step_deg=20`. Each must reply success: True. The goal's accept reuses these, so the counts below must match.
  - Fly one straight Columns control: GUI Columns, Sep. 125, Apex 0.95, Dwell 0.27, Throws 21.
  - `ros2 param set /skill_node columns_angle_deg 0.0` then `ros2 param set /skill_node columns_step_deg 5.0`
  - `ros2 action send_goal /jugglebot/juggle jugglebot_interfaces/action/Juggle "{pattern: columns_rotating, apex_m: 0.95, separation_mm: 125.0, dwell_s: 0.27, num_cycles: 74}" --feedback`, up to 3 Attempts.
  - `ros2 param set /skill_node columns_step_deg 10.0`, then the same goal with `num_cycles: 38`, up to 5 Attempts (the gate).
  - Only if 10 passed: `ros2 param set /skill_node columns_step_deg 20.0`, then the same goal with `num_cycles: 20`, up to 3 Attempts.
  - Restore: `ros2 param set /skill_node columns_step_deg 0.0`
- pass: at 10 deg per Beat, one Attempt with 36 consecutive Catches (one full turn of the pair) and no install refusal.
- record: per Attempt the step, Catches of Throws, the end reason and the Site name at the end; every `rotating stand-in` line that says stepped back; every AIM-LATERAL-CLAMPED line; the rehearsal's worst margin; the Hand iq peak; the bag directory.
- minutes: 60

## Progress

- 2026-11-27: 2026-11-27 design (offline): JP-11 needs no yaw support; the rotation can be explicit per-Beat Sites on compile_beats, as the held Centre and orbit branch did. Every rotating Throw is cross-site and gets no command-time admissibility check; a new rule is proposed. The Learner follows about 40 % of the turning pull (estimate). Build of four units proposed; owner to decide.
- 2026-11-28: 2026-11-28: built and merged to main (compile, wiring, command-time check rule iv, I-ADMIT-1 amended with the owner's wording). Sim at the live limits: 40 of 40 full-turn trials with every Catch at 5, 10 and 20 deg per Beat; no Learner in the sim. Full-turn nominations viable, worst margin 0.31 to 0.33 (jerk). Ready for its Ladder; the Sitting section is above.
