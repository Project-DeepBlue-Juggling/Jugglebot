---
id: FEAT-47
kind: feature
title: Add "Total Drops To Date" counter
status: in_progress
created: '2026-11-08'
updated: '2026-11-10T19:11:08Z'
owners:
- night-shift
priority: medium
effort: m
hardware: none
owner: none
radius: subsystem
area: gui
blocked_by: []
links: []
filed_by: harrison
---

where a 'drop' is strictly when a ball that was thrown with the intention of being caught by Jugglebot is not caught.

It would be neat to see the counter atop "Records" in the GUI, with it incrementing each time a ball is dropped.

This Item will require going back through all  Sitting logs to date to find drops, but that should be achievable with enough Sonnet agents, and it only needs to be done once.

Filed from the Robot tab at 2026-11-08T21:40:24+11:00.

## Options (2026-11-09)

The live count is cheap (the records ledger has Throws and Catches per Attempt). The history is about 1200 launch logs and 570 bags since January, in several log formats.

1. Live counter only, from the first Attempt after it lands.
2. Live counter plus a one-off base figure from the launch logs' Attempt end lines, and the page says the date it counts from.
3. Live counter plus the full history, by agents over older logs and bags.

Recommendation: 2. The definition's edge cases are in the ticket.

## Progress

- 2026-11-09: Night Shift 2026-11-09: rated, options written; awaiting the owner's answer
- 2026-11-10: Night Shift 2026-11-10: TKT-55 answered (option 3, definitions settled); building the live counter tonight, the history backfill is for a day Session
- 2026-11-10: Night Shift 2026-11-10: held on night-shift/2026-11-10-FEAT-47: (b) 3 audit findings, a false magazine drop first; unblocked by the owner's answer in TKT-61
- 2026-11-11: Night Shift 2026-11-11: TKT-61's three fixes merged (f87faa19); still open: Feed misses (required before closing) and the history backfill
- 2026-11-11: Night Shift 2026-11-11: Feed-miss slice held on night-shift/2026-11-11-FEAT-47: (b) magazine Feed misses still uncounted; unblocked by your answer
