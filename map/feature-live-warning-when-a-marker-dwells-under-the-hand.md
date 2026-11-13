---
id: FEAT-50
kind: feature
title: Live warning when a marker dwells under the Hand
status: complete
created: '2026-11-13'
updated: '2026-11-13T17:52:57Z'
owners: []
priority: medium
effort: m
hardware: robot
owner: none
radius: subsystem
area: tracking
blocked_by: []
links:
- map/bug-hand-stalls-at-the-70-a-clamp-mid-dive-probably-on-a-ball.md
- logbook/2026-11-14-trapped-ball-live-warn.md
filed_by: claude
---

BUG-84's three Hand stalls on 2026-11-13 were each a Ball trapped under the Hand that the Tracker had dropped; tools/trapped_ball_finder.py finds them offline from the raw unlabelled markers on /mocap_data. This asks for the live version: a log-only WARN at the moment it happens ("marker dwelling under the Hand at (x,y,z) for N ms, Hand at R rev"), repeated at 1 Hz while it persists.

Where: ball_tracker_node or ball_possession.py (pure Python, no ROS imports) already has the markers and robot frame; Hand position is on /hand_telemetry and the Platform pose on /rigid_body_poses. Move the finder's pure region_of function and its dwell state machine into a shared module under ros_ws/.../tracking/ that the finder imports.

Thresholds from the tool, fitted on one Sitting: a dwell is within 15 mm for at least 0.30 s (0.08 s when a stall is already seen); UNDER_HAND is below the Hand marker (z = -47 + 32.567 x rev) and not a Ball seated in the Cup (z = -25 + 32.567 x rev, x,y = 8,23). Store the Hand-marker fit in config, not as literals. Resting Balls in the rim pocket at about (-55,57) are common and harmless (21 stall-free dwells), so the WARN is a hint, not a stall predictor.

Log only: no action on the Hand path; the Hand safety authority stays the Teensy guard and the jam detector. It would later let skill_node or the operator hold the next dive; that is a separate owner decision and no motion change is proposed here.

## Sitting

- setup: armed
- why: NOT flyable until the WARN is built (no branch yet). Then: confirm the Hand-marker fit and the dwell thresholds still hold after any Hand or marker re-mount, and see the WARN fire in a dive over a deliberately placed Ball.
- do:
  - With the Platform level and the Hand at Active pose, place a Ball on the Platform top and roll it into the tube under the Cup by hand, then watch the launch log.
  - Read the WARN line and compare its position with the Ball.
- pass: a WARN naming the dwell position within 15 mm of the Ball, within 1 s of the Ball stopping, and no WARN with the Cup empty and the Platform clear.
- record: the WARN text, its time, and any false WARN count over the Sitting.
- minutes: 10
