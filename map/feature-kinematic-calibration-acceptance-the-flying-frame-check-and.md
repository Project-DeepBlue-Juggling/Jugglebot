---
id: JB-85
kind: feature
title: 'Kinematic calibration acceptance: the flying frame check, and a Fit that assumes gravity along -z'
status: inbox
created: '2026-10-28'
updated: '2026-10-28T07:26:53Z'
owners: []
priority: medium
blocked_by: []
links: []
parent: learner-generalisation
filed_by: claude
---

Kinematic-calibration acceptance was never finished.

- The flying frame check and the tilt-map recapture after Level (kincal steps 6-7) were never done; `_frame_offset_check` is still live (`skill_node.py:605`).
- `flight_fit` assumes gravity along -z, which gives a +62 mm/s^2 y bias.
- The early Fit lands about 9 mm short in x.
- The Pathway's P2 open questions (centre-pull cause, apex drift) are candidate symptoms of an uncorrected frame tilt; a global residual would otherwise absorb a geometry error.

Acceptance (Destination): the kinematic model, the Robot frame and the Tracker's Fit agree closely enough that the flying frame check passes and `_frame_offset_check` is deleted.

Source entries: [logbook/2026-09-23-kinematic-calibration-design-and-fit-tool.md](logbook/2026-09-23-kinematic-calibration-design-and-fit-tool.md), [logbook/2026-09-23-cup-contact-first-sitting.md](logbook/2026-09-23-cup-contact-first-sitting.md), [logbook/2026-09-29-skill-stack-r4-sitting-3-analysis.md](logbook/2026-09-29-skill-stack-r4-sitting-3-analysis.md)
