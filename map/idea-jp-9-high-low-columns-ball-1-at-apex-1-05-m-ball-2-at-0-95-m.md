---
id: JB-107
kind: idea
title: 'JP-9 High-low columns: Ball 1 at apex 1.05 m, Ball 2 at 0.95 m'
status: inbox
created: '2026-10-30'
updated: '2026-10-30T09:07:30Z'
owners: []
priority: medium
effort: m
hardware: robot
owner: hands
radius: subsystem
area: skill-stack
blocked_by: []
links: []
filed_by: claude (jugglebot-f5, pattern exploration 2026-10-30)
---

Owner wants to see it (2026-10-30). Two-ball columns with unequal apexes. The 0.045 s flight difference is absorbed by the Dwells (the low ball holds longer) and shortens one of the two empty transits; the offline probe planned 1.05 over 0.95 at today's limits under pose. Binding constraint: hand acceleration at the high ball (3843 of 3900 rev/s2 at 1.05). Software: Pattern carries one apex_m for both Sites and the box keys on one apex band, so the compiler needs a per-Site apex and the admissible box a two-band cell; the learner's apex outcome is already per throw. Visually subtle: the two apexes differ by 100 mm.
