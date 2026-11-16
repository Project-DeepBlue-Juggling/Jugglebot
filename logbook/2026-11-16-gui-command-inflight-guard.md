---
title: "GUI orchestrator command buttons gain an in-flight guard (BUG-104)"
type: bugfix
date: 2026-11-16
status: complete
phase: night-shift
subsystem: gui
related_items: [BUG-104]
files_changed:
  - ros_ws/gui/js/commands.js
  - tests/ros/test_gui_command_inflight.py
  - tests/ros/js/command_inflight_harness.js
  - logbook/2026-11-16-gui-command-inflight-guard.md
  - logbook/INDEX.md
---

## What and why

`commands.js` published on every click and took its enable state from the last `orchestrator_state`, so a double click
inside one state-update latency published twice. Each dispatch now latches all five command buttons inert (and a
second click handler call returns early) until the orchestrator state CHANGES from the one the click saw, or
`COMMAND_COOLDOWN_MS` (1500 ms) expires. Release is on a state change, not on "the next message", because
`orchestrator_state` re-publishes continuously and the next message would release on the next tick. 1500 ms rather
than the Juggle Start cooldown (4000 ms): a refused command never changes the state, so the latch must clear quickly.
Commands and payloads are unchanged, and the clear-errors hold-to-confirm still runs through the same shared `dispatch` (by reading; the harness never presses Clear Errors). One dispatch latches all five buttons, Deactivate and Clear Errors included, for up to 1.5 s when the state does not move (the audit finding, resolved 2026-11-17 below).

## Exemption: Deactivate and Clear Errors (2026-11-17, owner answer to TKT-78, option 1)

Deactivate is the operator's way out of Active and must never wait on another button's latch, so Deactivate and Clear
Errors are exempt from the shared latch: another button's click never disables them, and their own click does not latch
the others. Each keeps a private latch of the same kind (state change or 1500 ms), so its own double click publishes
once. `commands.js` now holds latches keyed `shared` or by exempt command name. Click-and-hold: of the five buttons only
Clear Errors is hold-to-confirm (`DESTRUCTIVE_COMMANDS`); `holdToConfirm` never uses click, it fires the same `dispatch`
once after the 800 ms hold and skips a disabled button, so a completed hold publishes once and composes with the private
latch (the harness now drives it with pointerdown plus the hold timer). The owner notes the double-click risk may be small
for held buttons; only Home, Level, Activate and Deactivate are plain clicks.

## Is a repeat a no-op in the orchestrator? Partly

`orchestrator_node.py::_on_command` (l. 267-295) enqueues every command unconditionally (`enqueue_command`, l. 278);
a command the state does not accept only draws a WARN (l. 279-290). Per its comment (l. 58-67) the handlers consume
`activate/home/level` in IDLE only and BOOT/HOMING/LEVELLING clear their queue every tick, so a duplicate is
normally discarded, but the node does not guarantee it; the GUI guard is the cheap fix either way.

## Left for the owner

Whether Activate deserves a hold-to-confirm: not added (owner decision, per the Item).

## Verification

2026-11-16 (before the exemption), `night_shift_nonet.sh nice -n 19 env OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 python -m pytest -q tests/ros/test_gui_command_inflight.py tests/ros/test_gui_replay_fence.py`: 8 passed in 0.58 s. With `commands.js` reverted, 4 of the 5 new tests fail (double click publishes twice, etc.); the control test passes either way.

2026-11-17, `night_shift_nonet.sh nice -n 19 env OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 python -m pytest -q tests/ros/test_gui_command_inflight.py tests/ros/test_gui_replay_fence.py`: 10 passed in 0.52 s. With `commands.js` reverted to the pre-exemption version, the 2 new exemption tests and the all-buttons latch test fail (3 failed, 4 passed).
