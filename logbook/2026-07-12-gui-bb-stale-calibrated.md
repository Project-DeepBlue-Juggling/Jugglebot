---
title: BB panel stale "Calibrated" across ROS sessions — latched calibration_result never reset when the GUI outlives the session; disconnect-reset added
type: bugfix
date: 2026-07-12
status: resolved
phase: GUI + ball-butler observability
related_entries:
  - 2026-07-12-gui-cone-catch-correlation-regression
  - 2026-07-11-gui-leg-setpoint-echo-poscmd
files_changed:
  - ros_ws/gui/js/panels.js
  - ros_ws/gui/js/main.js
  - tests/ros/test_gui_geometry.py
commits:
  - 946b0a9
subsystem:
  - gui
  - ros
tags:
  - observability
  - testing
---

## Summary

Operator report: the BB panel shows a **stale "Calibrated"** from a previous
session when the GUI — a long-lived browser tab — outlives ROS sessions; the
operator asked whether calibration state can be read from a topic directly.
Investigation ground truth: the Calibrated display is driven by the **latched**
`bb/calibration_result` from `mocap_node` (published only on calibration
attempts, never on startup); `bb/heartbeat` (`teensy_bridge_node`, relayed from
the BB device's CAN heartbeat) carries **no** calibration flag — and adding one
was **rejected** on root-cause grounds (see Discussion). The latched topic
**is** the right source; the gap was that nothing reset the indicator on
session death. Fix: `resetBBCalibration()` in `panels.js`, called from
`main.js`'s `onConnectionStateChange` `'disconnected'` branch — the same
pattern as the cone entry's `setCatchingConeDisconnected`, making this the
**third instance this week of the "GUI outlives the ROS session"
state-latching class** (pos_cmd flatline cache, cone catch baseline, now BB
Calibrated). Pinned by 4 new string-level tripwires
(`TestBBCalibrationStaleLatchReset`).

## Problem

The GUI runs headless-served in a browser tab that routinely outlives ROS
sessions (launch, test, tear down, relaunch — the tab stays open). After a
session in which BB calibration succeeded, every subsequent session shows the
BB panel as "Calibrated" **even though the fresh session has performed no
calibration** — the operator has no honest way to tell a calibrated session
from an uncalibrated one without re-calibrating. The operator's suggested
direction: read calibration state directly from a topic (e.g. a flag on the BB
heartbeat).

## Root Cause

The Calibrated indicator is set by a **latched event, not a state stream**:
`mocap_node` publishes `bb/calibration_result` with transient-local
(latched) QoS **only when a calibration attempt completes** — never on
startup, and nothing ever re-publishes it to signal that a session ended. The
browser DOM persists across ROS sessions, so the previous session's
`success=true` render simply stays on screen. A fresh `mocap_node` has no
latched sample to deliver, so nothing corrects the display until the *next*
calibration attempt.

Notably, `bb/heartbeat` cannot be the fix's source: it is relayed by
`teensy_bridge_node` from the BB device's CAN heartbeat and carries no
calibration field — calibration is not device state at all; it lives in
`mocap_node` (see Discussion for why it should stay there).

## Discussion

### Third instance of the "GUI outlives the ROS session" state-latching class

This is the same failure class as this week's two prior fixes: GUI state
seeded by ROS messages persists in the browser DOM after the ROS session dies,
silently misrepresenting a fresh session. Instances: the pos_cmd flatline
cache ([[2026-07-11-gui-leg-setpoint-echo-poscmd]] — `latestCommandedLegs`
never staleness-cleared), the cone catch-counter baseline
([[2026-07-12-gui-cone-catch-correlation-regression]] — reconnect
re-baselining), and now the BB Calibrated latch. The common remedy is now a
pattern: **reset session-scoped GUI state on the rosbridge `'disconnected'`
transition** — the one signal the browser reliably gets when the ROS session
goes away.

### Why NOT a calibration flag on bb/heartbeat (operator's suggestion, rejected)

The concrete failure modes and costs, not an appeal to convention:

- **Wrong process.** Calibration state lives in `mocap_node`; `bb/heartbeat`
  is published by `teensy_bridge_node` from the BB device's CAN heartbeat.
  Wiring calibration into the bridge heartbeat means a new cross-node
  subscription, a `.msg` change, and a lockstep interface rebuild — real
  coupling and rebuild cost for a display fix.
- **Category error.** The heartbeat reports **device liveness** (the CAN link
  to the BB). Calibration is **mocap-software state** — it can be valid while
  the BB link is down and stale while the link is up. Merging them makes both
  signals harder to reason about.
- **It doesn't even fix the bug.** The stale render is in the browser DOM; a
  heartbeat flag would still display stale until the next message after
  reconnect, and a dead session publishes nothing at all. The DOM needs a
  disconnect reset **regardless** of the data source.

The latched topic is already the architecturally right source — publish-once,
re-delivered automatically to late subscribers. The gap was purely the missing
reset.

### The self-healing property worth stating

Resetting on `'disconnected'` composes correctly with the latched QoS in both
directions:

- **Same-session network blip**: the websocket drops, the indicator resets to
  Not Calibrated; on reconnect, rosbridge's resubscribe **re-delivers the
  still-latched `success=true`** and the Calibrated display restores itself —
  no operator action, no false negative that persists.
- **Fresh session**: the new `mocap_node` has no latched sample, so nothing is
  re-delivered and the panel **honestly stays Not Calibrated** until a real
  calibration runs.

### Other alternatives rejected

- **Reset on BB-heartbeat `connected=false`**: mocap calibration is
  independent of the BB CAN link — a transient BB dropout would flicker the
  calibration display for no reason, and a session death without a BB
  heartbeat transition wouldn't reset it at all. Wrong axis.
- **`mocap_node` publishes `success=false` on startup**: manufactures a
  spurious calibration-failure event with no discriminator from a genuinely
  failed calibration — every downstream consumer would have to learn to tell
  them apart.

### Residual (documented, accepted)

A **lone `mocap_node` restart while rosbridge stays up** is not covered: the
websocket never drops, so no reset fires, and the old latched sample is gone
from the new node — the stale render would persist until the next calibration.
Accepted: that is not the deployment topology (nodes launch and die together
via `jugglebot_launch.py`; rosbridge dies with the session).

## Fix

**GUI** (`ros_ws/gui/js/panels.js`): new exported `resetBBCalibration()` —
returns the BB panel's calibration display to the Not Calibrated /
uncalibrated-dot baseline.

**GUI** (`ros_ws/gui/js/main.js`): `resetBBCalibration()` called from
`onConnectionStateChange`'s `'disconnected'` branch — alongside the cone
panel's `setCatchingConeDisconnected()`, with an in-place comment recording
the self-healing rationale (resubscribe re-delivers the still-latched sample
on a same-session blip).

**Tests** (`tests/ros/test_gui_geometry.py`): 4 new string-level tripwires
(`TestBBCalibrationStaleLatchReset`) pinning that the reset exists, is
exported, is invoked from the `'disconnected'` branch, and that the latch
semantics documented above survive refactors.

## Verification

- **Scoped suites** (`pytest tests/ros/test_gui_geometry.py
  tests/ros/test_ball_butler_node.py tests/ros/test_ball_butler.py
  tests/ros/test_teensy_bridge_node_bb.py -q`, run 2026-07-12): **138 passed**,
  including the 4 new `TestBBCalibrationStaleLatchReset` tripwires.
- **GUI JS**: node-module syntax checks on `panels.js` / `main.js` — clean.
- **Headless-chromium baseline**: a fresh page renders **Not Calibrated** with
  the uncalibrated dot (the post-reset state is the honest default).
- **Full suite, immediately pre-commit** (`pytest tests/ -q`, run 2026-07-12):
  **2545 passed, 1 xfailed in 590.43 s** (tree includes the parallel
  bench-sysid session's concurrent work; post-suite edits were markdown-only,
  outside pytest collection).

## Outcome

The BB Calibrated display is now session-honest: it resets on rosbridge
disconnect, self-restores from the latched sample on a same-session blip, and
stays Not Calibrated in a fresh uncalibrated session. No message, node, or
interface changes — the latched topic was the right source all along; the GUI
just needed to stop trusting a DOM render across session boundaries. Third
strike for the session-latching class; the disconnect-reset pattern is now
established in `main.js` for future session-scoped state.

## Related

- [[2026-07-12-gui-cone-catch-correlation-regression]] — the disconnect-reset
  pattern this reuses (`setCatchingConeDisconnected`, the cone catch-counter
  re-baseline) and the sibling instance of the session-latching class.
- [[2026-07-11-gui-leg-setpoint-echo-poscmd]] — class origin: the pos_cmd
  flatline cache (`latestCommandedLegs` never staleness-cleared), the first of
  the three instances this week.
