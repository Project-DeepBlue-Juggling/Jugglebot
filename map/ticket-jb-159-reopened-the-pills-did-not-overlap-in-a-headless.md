---
id: JB-170
kind: ticket
title: 'JB-159 reopened: the pills did not overlap in a headless repro; re-check on a recording now, or send a screenshot naming the chart?'
status: complete
created: '2026-11-05'
updated: '2026-11-04T21:22:21Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
radius: local
area: gui
blocked_by: []
links: []
parent: bug-gui-chart-hover-over-callouts-overlap
type: question
---

You reopened JB-159: after the 2026-11-04 fix (298f2a38) the hover pills still overlapped on a recording. The Night Shift could not reproduce it: a headless Chromium ran the real chart modules with position and motor temperature in the same pixel band, live and replay paths, DPR 1 and 2, and the pills stacked edge to edge with no overlap. The GUI server sends `no-cache`, so a hard refresh does load the current JS.

## Options
1. Send a screenshot of the overlap and say which chart it was on (a leg chart, the Hand, the CAN traffic chart). The night then reproduces that exact case.
2. Re-check first: open a recording again now (the live checkout must have been on `main` at or after 05:46 on 2026-11-04) and say whether it still overlaps.
3. Close JB-159 if 2 shows it fixed.

Recommendation: 2, then 1 if it still overlaps.

## What each answer unblocks
- 1: a unit that reproduces your case in the headless harness and fixes it.
- 2 or 3: JB-159 closes, or goes to 1 with the chart named.

## Answer

Yes, this must have been an incorrect-checkout problem; this is working well now.
