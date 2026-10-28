---
id: JB-52
kind: bug
title: 'Wave camera: CLIP_MISSING on the first start after a camera power-cycle'
status: inbox
created: '2026-10-25'
updated: '2026-10-25T09:00:00Z'
owners: []
priority: low
effort: m
hardware: robot
owner: hands
radius: subsystem
diagnosis: unknown
area: tracking
blocked_by: []
links:
- logbook/2026-10-19-wave-camera-can3-bridge-and-node.md
- temp/logs/wave_row7_autostop_fix.log
parent: wave-camera-can3
filed_by: claude
---

2026-10-25 16:07, the row 7 repeat: `record_start` returned CLIP_MISSING (reason 5, fault 0x04), no "Created new clip." UART line inside the window, about three minutes after a camera power-cycle. The owner is fairly sure a clip was recorded, so it reads as a missed or late UART line at the adapter. For the bench PC; see logbook `2026-10-19-wave-camera-can3-bridge-and-node.md` § Jetson bring-up.
