---
id: BUG-104
kind: bug
title: GUI command buttons have no in-flight guard
status: complete
created: '2026-11-14'
updated: '2026-11-16T18:59:56Z'
owners: []
priority: low
effort: s
hardware: none
owner: none
radius: local
diagnosis: hypothesis
area: gui
blocked_by: []
links:
- logbook/2026-11-16-gui-command-inflight-guard.md
parent: gui-improvements
filed_by: claude
---

The GUI's orchestrator command buttons publish on every click with no in-flight guard. From the 2026-11-13 maintainer review; read by the reviewer, not re-checked.

## Findings

- `ros_ws/gui/js/commands.js:55-80` publishes on each click. Enable state comes from the last orchestrator state received, so a double click inside one state-update latency publishes twice.
- Only the clear-errors command has hold-to-confirm. Level and Activate have none.
- The Juggle panel's Start button already has the right pattern: a busy flag and a cooldown (`dispatchJuggle` in `juggle-panel.js`).
- Whether the orchestrator treats a repeated command as a no-op was not checked; if it does, this is cosmetic.

## Wanted

The same busy-and-cooldown guard on the command buttons, and a decision on whether Activate deserves a confirm.

## Progress

- 2026-11-16: Night Shift 2026-11-16: held on night-shift/2026-11-16-BUG-104: command buttons latch until the state moves; 1 audit finding open (Deactivate shares the latch); unblocked by TKT-78
- 2026-11-17: Night Shift 2026-11-17: held on night-shift/2026-11-16-BUG-104: Deactivate and Clear Errors exempt (TKT-78); 2 test gaps open; gate [pre] and (a) BUG-109; unblocked by TKT-81
