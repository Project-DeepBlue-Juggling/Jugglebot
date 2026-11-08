---
id: TKT-35
kind: ticket
title: 'BUG-64 CONSOLE search: pin the startup lines, show the ring''s window, backfill from launch.log, or leave it?'
status: complete
created: '2026-11-04'
updated: '2026-11-03T21:55:31Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
radius: local
area: gui
blocked_by: []
links: []
parent: bug-console-search-missing-some-entries
type: question
---

BUG-64 asks why a CONSOLE search for "v38" or "v7" finds no BRIDGE or PLATFORM firmware-check line. The search works on those exact texts. The lines are simply not in the CONSOLE: the 5000-line ring evicts them after about 8-17 min, and `/rosout` on Foxy is volatile, so a page that connects after them never sees them (Findings (2026-11-04) on BUG-64).

## Options

1. Pin one-shot startup lines (`*_FW_CHECK`, identity) so the ring cap never evicts them. GUI-only, s.
2. Show the ring's window at the top of the CONSOLE ("lines since HH:MM:SS, N dropped"). GUI-only, xs.
3. Backfill the CONSOLE from the current run's `launch.log` through a `gui_server.py` endpoint. Covers a late connect too, m.
4. Leave it: the hardware-versions sidebar already shows the versions.

Recommendation: 2 then 1.

## What each answer unblocks

- 1 or 2 (or both): a GUI unit the Night Shift can build, test under node and gate.
- 3: a larger unit that adds a server endpoint, still offline-testable.
- 4: BUG-64 closes as explained.

## Answer

I'm not sure this was an eviction problem; I searched for these terms during a live Sitting that had only just begun, so  there were fewer than 5,000 messages that had been sent.

I agree with your recommendation: do (2), then (1)
