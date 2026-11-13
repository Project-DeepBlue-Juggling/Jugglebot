---
id: BUG-104
kind: bug
title: GUI command buttons have no in-flight guard
status: inbox
created: '2026-11-14'
updated: '2026-11-13T23:10:03Z'
owners: []
priority: low
diagnosis: hypothesis
area: gui
blocked_by: []
links: []
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
