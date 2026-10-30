---
id: JB-110
kind: idea
title: 'JP-13 Rising and sinking columns: the whole Pattern rides the Platform in z'
status: inbox
created: '2026-10-30'
updated: '2026-10-30T09:07:32Z'
owners: []
priority: medium
effort: l
hardware: robot
owner: hands
radius: cross-cutting
area: skill-stack
blocked_by: []
links: []
filed_by: claude (jugglebot-f5, pattern exploration 2026-10-30)
---

Owner framing 2026-10-30: the release and catch planes stay fixed RELATIVE to the Platform centroid, so the hand stroke is untouched and the limits are the leg lengths. Leg IK probe (pose_to_leg_lengths, hard margin 5 mm, 2026-10-30): with a columns-sized reserve (xy 65 mm, tilt 5 deg) the Platform can sit from 104 mm below to 68 mm above the active z 170; with xy 75 and tilt 8 deg, -90 / +50 mm. The range is asymmetric: the active pose is near the top of the z workspace, so this is mostly a SINKING pattern. A z step per Beat adds to the transit vector (sqrt(125^2 + 50^2) is 8 percent longer than 125), so 50 mm per Beat fits today's leg velocity. Software: today z is a global (CATCH_CUP_Z_MM 830, RELEASE_CUP_Z_MM 860, default_active_z_mm 170 held at every knot, unified_z_band_mm 30); a Site needs a z and the two planes become offsets from it, the admissible box gains a z axis, the tracker's landing plane follows the Site. Pose-only. Not the same as moving the catch plane relative to the Platform, which costs hand acceleration (sites.py table) and was refused at 930.
