---
title: Catching-cone catches invisible in the GUI — catch_correlation_node was a Phase-13 launch orphan; node restored + defensive heartbeat UX
type: bugfix
date: 2026-07-12
status: resolved
phase: GUI + catching cone
related_entries:
  - 2026-05-23-throw-director-and-cone-live-integration
  - 2026-07-06-phase13-socketcan-decommission
  - 2026-07-11-gui-leg-setpoint-echo-poscmd
files_changed:
  - ros_ws/src/jugglebot/launch/jugglebot_launch.py
  - ros_ws/gui/js/panels.js
  - ros_ws/gui/js/main.js
  - tests/ros/test_launch_nodes.py
  - tools/probes/gui_dom_probe.py
  - tools/probes/gui_synthetic_stack.py
  - tools/probes/README.md
commits:
  - pending-backfill
subsystem:
  - gui
  - ros
tags:
  - observability
  - IPC
  - testing
---

## Summary

Operator report (live GUI testing, 2026-07-12): a piezo contact on the catching
cone produces **nothing** in the GUI, even though `cone/heartbeat`'s
`last_catch_seq` increments correctly on every hit. Not a GUI bug: the GUI's
**only** catch-surfacing input is `cone/timing_result`, published exclusively by
`catch_correlation_node` — and that node was **orphaned by the Phase-13
SocketCAN decommission** ([[2026-07-06-phase13-socketcan-decommission]],
commit 7c7f61b). Its sole launcher, `catching_cone_test.launch.py`, was deleted
as "the last launcher of `can_node`" (operator chose delete over rewire), and
the node was never added to `jugglebot_launch.py`. The **transport half**
migrated correctly (`teensy_bridge_node` publishes `cone/catch_event` and
`cone/heartbeat` from the relayed CAN frames); the **correlation half** has been
silently dead since 7c7f61b. The pipeline was validated live end-to-end on
2026-05-23 ([[2026-05-23-throw-director-and-cone-live-integration]]:167,
417-419: cone → `cone/catch_event` → `catch_correlation_node` →
`cone/timing_result` → GUI sound bar). Fix, two parts: (1) restore
`catch_correlation_node` to `jugglebot_launch.py` as a bare declaration exactly
as the deleted launch had it, with an in-place comment telling the whole story;
(2) a **defensive heartbeat UX** — `panels.js`'s cone panel now surfaces
`last_catch_seq` directly (per-tap feedback independent of the correlation
pipeline). A string-level launch tripwire (`tests/ros/test_launch_nodes.py`)
pins the node declared **and** listed in the `LaunchDescription`, closing the
specific regression class.

## Problem

On the live GUI, tapping the catching cone's piezo surface produces no visible
response anywhere in the GUI — no sound bar, no timing readout, no panel change.
Meanwhile `/cone/heartbeat`'s `last_catch_seq` increments cleanly on each hit,
so the cone hardware, the CAN relay, and the bridge's heartbeat path are all
demonstrably alive. The catches are being *counted* but never *surfaced*.

## Root Cause

**A Phase-13 launch orphan — a regression, not a feature gap.** The GUI's only
catch-surfacing datasource is `cone/timing_result` (`CatchTimingResult`),
published *exclusively* by `catch_correlation_node`. That node subscribes
`cone/catch_event`, matches each catch against `throw_announcements` landings
within a 0.5 s window, and emits matched/unmatched `CatchTimingResult` records.

`catch_correlation_node`'s only launcher was `catching_cone_test.launch.py`,
**deleted in the Phase-13 SocketCAN decommission** (commit 7c7f61b) as "the last
launch file that still started `can_node`" — the operator chose delete over
rewire ([[2026-07-06-phase13-socketcan-decommission]] Discussion). The node was
**never added to `jugglebot_launch.py`**. So after 7c7f61b, nothing launches
`catch_correlation_node`, `cone/timing_result` has zero publishers, and every
consumer stayed wired to a dead topic.

The transport half of the cone pipeline migrated correctly:
`teensy_bridge_node` publishes `cone/catch_event` (from the relayed 0x7E0 CAN
frames) and `cone/heartbeat` (with `last_catch_seq`). Only the **correlation
half** was orphaned.

**Evidence chain:**

- `git show 7c7f61b -- .../catching_cone_test.launch.py`: the deletion removed
  both the `catch_correlation_node = Node(package='jugglebot',
  executable='catch_correlation_node')` declaration **and** its
  `LaunchDescription([...])` membership.
- `git log -S catch_correlation_node -- .../jugglebot_launch.py`: **empty** —
  the symbol never existed in the production launch's history; it was never
  migrated there, only ever launched by the test launch.
- The GUI subscribes `cone/timing_result` (`main.js:212`) and the production
  rosbag record list includes `/cone/timing_result`
  (`jugglebot_launch.py:263`) — consumers stayed wired to a publisher-less
  topic, so bags silently recorded it empty too. Dead topics look identical to
  quiet topics: no error is raised anywhere.

## Discussion

### The silent-orphan failure class — and its two-day recurrence

This is the **same failure class** fixed yesterday in
[[2026-07-11-gui-leg-setpoint-echo-poscmd]]: a migration moves or deletes a
*producer* while its *consumers* stay wired, and it fails **silently** because a
dead topic is indistinguishable from a quiet one. Yesterday it was
`leg_lengths_topic` losing its publisher when `can_node` was deleted and the
run_mpc stack went dormant; today it is `cone/timing_result` losing its
publisher when the same 7c7f61b decommission deleted the only launcher of
`catch_correlation_node`. Two instances in two days, both rooted in the
Phase-13 migration, both invisible until an operator noticed a topic that was
supposed to move but didn't.

The mitigations landed so far target this class directly: the pos_cmd fix added
consumer⊆producer tripwires (the KeyValue drift-pin tests), and this fix adds a
**launch-node tripwire** — `tests/ros/test_launch_nodes.py`, a string-level
check (in the regex style of `test_gui_geometry.py`) that pins
`catch_correlation_node` both **declared** (`executable='...'`) and **membered**
in the returned `LaunchDescription([...])`. Its docstring names the regression
class explicitly: *a node dropped from the production launch (or whose last
launcher is removed) while its downstream consumers stay wired.* It is honest
about its limits — it cannot prove the node starts under a real ROS2 runtime; it
catches exactly the delete-the-producer-leave-the-consumer class that bit us.

Worth stating as an open question (below): a future **systematic sweep**
asserting every GUI-subscribed topic has a live producer somewhere in the launch
graph would close the *whole* class in one contract, rather than pinning nodes
one at a time as regressions surface. That is a bigger change than this fix
warrants; it is noted, not implemented here.

### Why the heartbeat UX addition is in-scope for a "minimal" fix

Restoring the node alone would fix the reported symptom. The `panels.js`
`updateConeHeartbeat` addition — surfacing `last_catch_seq` directly as a
`last catch #<seq> · <N>s ago` line with an 800 ms accent flash on change — is
included deliberately, on two grounds:

- **Defense in depth against exactly the failure mode that just bit.** The
  heartbeat path (`cone/heartbeat` → bridge) is structurally independent of the
  correlation pipeline (`cone/catch_event` → `catch_correlation_node` →
  `cone/timing_result`). If correlation ever dies again — a node dropped, a
  crash, a `throw_announcements` outage — the operator still gets **per-tap
  feedback** from the heartbeat. Catch visibility no longer has a single point
  of silent failure.
- **Operator mental-model alignment.** The operator's instinctive expectation
  is "I tapped the cone, the GUI should acknowledge the tap." The correlation
  pipeline only surfaces *matched* or *unmatched-within-a-window* results, which
  is a semantically richer but strictly narrower signal than "a catch happened."
  The heartbeat line answers the instinctive question directly and immediately.

The addition reuses only existing classes/idioms in `panels.js`; it is a
low-risk, high-leverage complement to the node restore, not scope creep.

### Why NOT an event-store CATCH type

The tempting "clean" design would emit a catch into the GUI's event store so it
flows through the existing chart overlays and filters. Rejected:
`EVENT_TYPES` is a **frozen 4-type enum** feeding chart overlays and filter
controls. Adding a `CATCH` type ripples into overlay rendering, filter UI, and
the event-store schema — a materially bigger change than the reported bug
warrants, and out of scope for a minimal regression fix. The heartbeat line is
a self-contained panel update that touches none of that machinery.

### First-message baseline (no phantom, reconnect-safe, wrap-safe)

The seq-change flash is guarded so it never fires spuriously: the seq baseline
is established on the **first connected heartbeat** (no phantom flash on panel
init), and a reconnect **re-baselines** (catches that occurred while the panel
was disconnected don't retroactively flash). The baseline resets on **both
disconnect axes**: a cone-reported-offline heartbeat (`connected=false`) AND a
GUI↔rosbridge websocket drop (the pre-commit audit caught that the first
implementation covered only the cone axis — a catch during a rosbridge outage
would have flashed on reconnect; `setCatchingConeDisconnected()` is now also
called from main.js's disconnected branch, alongside its existing latch
resets). The change test is an inequality, not `>`, so the `uint8`
`last_catch_seq` wrapping 255→0 still registers as a catch. `have_any_catch`
gates the whole line so the placeholder seq/ms fields are never shown as real.

## Fix

**Launch** (`ros_ws/src/jugglebot/launch/jugglebot_launch.py`):
`catch_correlation_node` restored — a **bare declaration** (`package='jugglebot',
executable='catch_correlation_node'`, no params, no remaps), exactly matching
the form 7c7f61b deleted (verified against `git show`; the deleted launch's
cone-related params all belonged to `ball_butler_node`, not this node). Declared
as a `Node(...)` and added to the returned `LaunchDescription([...])`, with an
in-place comment block recording the orphan story so the next reader sees *why*
the node lives here.

**GUI** (`ros_ws/gui/js/panels.js`, `updateConeHeartbeat`): the cone panel now
surfaces `last_catch_seq` directly — a `last catch #<seq> · <N>s ago` line
sourced from the `CatchingConeHeartbeat` fields `have_any_catch`,
`last_catch_seq`, `ms_since_last_catch` — with an 800 ms accent flash on
seq change. Baseline established on the first *connected* heartbeat (no
init phantom); reconnect re-baselines; change test is a wrap-safe inequality.
Reuses existing panel classes/idioms only. Deliberately **not** an event-store
emission (see Discussion).

**Test** (`tests/ros/test_launch_nodes.py`, new): string-level tripwire pinning
`catch_correlation_node` both declared and membered in the launch's
`LaunchDescription`. Docstring names the regression class and cross-references
this entry and the 2026-05-23 validated-working reference wiring.

**What the operator sees post-fix:**

- Every piezo tap → the heartbeat line updates and flashes, **independent of
  correlation** (works even if `catch_correlation_node` is down).
- With the stack **relaunched** (to pick up the restored node): an **unmatched**
  tap → an `Actual` clock reading + an `unmatched catch` footer; a **matched**
  catch (a throw announced within 0.5 s) → a predicted-vs-actual delta + a
  sound-bar bar + a `last: <thrower>` footer.
- Honest note: the **sound bar remains matched-only by design** — it visualises
  the predicted-vs-actual timing delta, which only exists for matched catches.

## Verification

- **Scoped suites** (`pytest tests/ros/test_launch_nodes.py
  tests/ros/test_gui_geometry.py -q`, run 2026-07-12): **56 passed** (2 new
  launch tripwires + 54 existing geometry checks).
- **Syntax**: node-module syntax check on `panels.js` — clean; `py_compile` on
  `jugglebot_launch.py` — clean.
- **Headless-chromium harness** against the real `panels.js` module (temporary,
  deleted after use): first-heartbeat baseline establishes with **no phantom
  flash**; reconnect with a pre-existing catch **re-baselines silently**; a
  seq change renders `last catch #8 · 0s ago` — all as designed.
- **Synthetic-stack DOM probe, new cone scenario** (`python3
  tools/probes/gui_dom_probe.py --scenario cone`, run 2026-07-12): **11/11
  assertions PASS** (10/10 twice pre-audit-fix, then 11/11 with the added
  GUI-disconnect reset assertion C7 — cone heartbeats kept flowing while only
  the websocket died, so the baseline wipe to `—` can only come from the new
  main.js disconnect branch) — connected badge + `no catches yet` baseline with a
  zero-flash 1.2 s sample window; disconnect re-baseline; first-heartbeat with
  a pre-existing catch baselines to `last catch #7 · 12s ago` with **zero
  phantom flash**; seq 7→8 flashes accent-green and updates; unmatched
  `cone/timing_result` → Actual clock + `unmatched catch` footer; matched →
  `+12.5 ms` delta (amber band) + `last: ball_butler`. Regression runs the same
  day: scenario1 **17/17**, minimap **33/33**; teardown flags clean on all.
- **Full suite pre-commit gate** (`pytest tests/ -q`, run 2026-07-12): **2533
  passed, 1 xfailed in 620.89 s** (count includes the parallel bench-sysid
  session's newly-landed tests plus this fix's 2 launch tripwires; post-suite
  edits were markdown-only, outside pytest collection).
- **PLACEHOLDER — operator live confirmation** (piezo tap → panel feedback):
  **remaining hardware validation.** Requires a **stack relaunch** to pick up
  the restored `catch_correlation_node`; the heartbeat-line half can be
  confirmed without a relaunch, but matched/unmatched timing readouts need the
  node running. Not yet performed.

## Outcome

The catching-cone catch pipeline is wired end-to-end again: `catch_correlation_node`
is back in the production launch (restoring `cone/timing_result` for the GUI and
the rosbag), and the cone panel now gives per-tap heartbeat feedback that
survives any future correlation-half outage. The launch tripwire pins the
node against silent re-orphaning. Remaining: the operator live confirmation
(relaunch required) — the one placeholder verification item above.

## Open Questions

- **Systematic producer-liveness sweep.** A test asserting that every
  GUI-subscribed topic has a live producer somewhere in the launch graph would
  close the entire silent-orphan class in one contract, rather than pinning
  nodes one at a time as regressions surface. Noted, not implemented here —
  it is a materially larger change than this fix.
- **Sound bar is matched-only by design.** If operators want a visual cue for
  *unmatched* catches too (beyond the footer text), that is a follow-up UX
  decision, not a regression.

## Related

- [[2026-05-23-throw-director-and-cone-live-integration]] (:167, 417-419) — the
  live end-to-end validation of the cone → `cone/catch_event` →
  `catch_correlation_node` → `cone/timing_result` → GUI sound-bar pipeline that
  7c7f61b silently broke. The reference wiring this fix restores.
- [[2026-07-06-phase13-socketcan-decommission]] — commit 7c7f61b, which deleted
  `catching_cone_test.launch.py` (the sole launcher of `catch_correlation_node`)
  as the last launcher of `can_node`; the node was never migrated to
  `jugglebot_launch.py`.
- [[2026-07-11-gui-leg-setpoint-echo-poscmd]] — the same silent-orphan failure
  class fixed one day earlier (`leg_lengths_topic` producer orphaned by the same
  migration); the consumer⊆producer tripwire strategy this fix extends to the
  launch graph.
