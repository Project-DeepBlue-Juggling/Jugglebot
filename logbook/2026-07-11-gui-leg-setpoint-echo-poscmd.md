---
title: GUI "Pos (cmd)" leg series restored — teensy_bridge setpoint echo replaces the orphaned leg_lengths_topic datasource
type: bugfix
date: 2026-07-11
status: resolved
phase: GUI + can-bridge observability
related_plan: mvp-trajectory-bringup.md
related_entries:
  - 2026-07-08-mvp-autonomous-build-run
  - 2026-07-06-phase13-socketcan-decommission
  - 2026-07-01-canbridge-phase5-hand-conduit
files_changed:
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/src/jugglebot/launch/jugglebot_launch.py
  - ros_ws/gui/js/main.js
  - ros_ws/gui/js/panels.js
  - ros_ws/gui/js/telemetry-charts.js
  - tests/ros/test_teensy_bridge_node_setpoint.py
  - tools/probes/uplot_nan_gap_probe.js
  - tools/probes/README.md
commits: []  # PENDING — backfill immediately after commit (feedback_logbook_sha_backfill)
subsystem:
  - gui
  - ros
  - can
tags:
  - observability
  - IPC
  - testing
---

## Summary

The GUI motor charts' **"Pos (cmd)" series was missing for legs 0–5** on all
recent runs. Not a chart bug: the series' datasource (`leg_lengths_topic`) was
**orphaned by the architecture migration** — in the current topology no ROS
topic carries commanded leg positions at all, so commanded-vs-measured was
unobservable for every current-topology run. Fix: `teensy_bridge_node` now
**echoes the last accepted per-leg setpoint** from its setpoint funnel on a new
topic **`leg_setpoint_echo`** (Float64MultiArray, 6 floats, motor revs),
covering *both* command sources (trajectory_node and run_mpc both feed :5557);
the GUI subscribes to it, staleness-clears its cache, and — via a custom uPlot
gaps hook — renders stale windows as true pen-up gaps instead of false bridging
ramps. A multi-agent adversarial review confirmed 2 HIGH findings (both fixed
in-session, plus 4 more accepted-and-fixed), and the swap also killed a latent
GUI bug that computed hand tracking error from garbage
(`latestCommandedLegs[6]` = leg-0 vel_ff in the old 18-value layout). The GUI's
last dependency on `motion_bridge_node`'s **leg output** is gone — but one GUI
consumer of the node remains (`motion/diagnostics` → the motion panel,
MPC-stack-only data), so the MVP plan's motion_bridge retirement item's
"verify GUI topic consumption" check must still disposition that topic before
removal; this change clears the leg-output half only.

## Problem

Operator report: on recent runs the GUI motor charts show every datasource
*except* "Pos (cmd)" for legs 0–5. The series' toggle pill stays lit; no error
is surfaced anywhere — the series is simply absent from the plot. (uPlot
renders an all-NaN series as silently invisible, so the GUI gave no hint that
the datasource was dead rather than merely flat.)

## Root Cause

**An architecture-migration orphan, not a chart bug.** In the current
production topology, commanded leg positions flow

```
trajectory_node ──ZMQ :5557──▶ teensy_bridge_node SetpointPump ──UDP SETPOINT──▶ can-hub Teensy
```

and **no ROS topic carries them anywhere on that path**. The GUI's datasource,
`leg_lengths_topic`, is only published by `motion_bridge_node` *while the
run_mpc/motor_guard stack streams* — and that stack has been dormant since the
MVP trajectory bringup removed MPC from the leg path
([[2026-07-08-mvp-autonomous-build-run]]). The topic's original consumer,
`can_node`, was deleted in the 2026-07-06 SocketCAN decommission
([[2026-07-06-phase13-socketcan-decommission]]), so nothing in the migration
noticed the GUI was still reading a topic no current run publishes.

Strictly, the failure is data-dependent — the series would reappear if
`run_mpc` were running — but the production path is dead: commanded leg
position was unobservable for **all** current-topology runs.

## Discussion

*(Hypothesis reframed mid-investigation: this started as "chart bug" and ended
as "architecture-migration observability orphan" — the GUI code was doing
exactly what it was told with a datasource the migration had silently
orphaned.)*

### Why the bridge echoes, not trajectory_node publishing (user-approved design)

**Chosen**: `teensy_bridge_node` echoes the last **accepted** per-leg u0 from
the setpoint funnel on the new `leg_setpoint_echo` topic. Concrete reasons:

- **Covers both sources.** trajectory_node *and* run_mpc both feed :5557; the
  bridge's funnel is the one place every commanded leg setpoint passes through.
  An echo there is topology-proof — it survives the MPC-replanner
  re-architecture direction too.
- **"Accepted" is the honest signal.** Echoing post-funnel means the chart
  shows what the robot was actually commanded, not what a producer *wished* —
  pump-rejected frames don't appear.
- **Hot-path discipline holds.** The setpoint thread only *stashes* — three
  plain assignments under `self._lock`. Message construction + `publish()` ride
  the existing 100 Hz `robot_state` timer, gated on freshness (0.5 s → the echo
  goes silent when the stream stops) and seq-deduped (~40 Hz effective). No
  allocation, no ROS work on the UDP send path.
- **Stash-before-send is deliberate**: the stash happens before the UDP send,
  so a transient send `OSError` doesn't retract the echo. The link watchdog
  owns real outages; the echo reports command intent, and a one-packet blip
  should not punch a hole in the chart.

**Rejected — trajectory_node publishes alongside its ZMQ send**: covers only
trajectory runs (run_mpc's commands would stay invisible); adds work to the
S3-emit-gap-sensitive hot path; and re-opens a dual-publisher hazard on
`leg_lengths_topic` whenever run_mpc runs.

**Rejected — reuse the `leg_lengths_topic` name/layout**: dual-publisher
conflict with `motion_bridge_node` the moment run_mpc runs; the name misleads
(these are motor revs of *commanded setpoint*, not leg lengths); and the old
18-value layout would re-activate a latent GUI bug reading index 6 as a hand
command (see Fix — it was garbage: leg-0 vel_ff).

### Adversarial review — 2 HIGH confirmed, all findings fixed in-session

A multi-agent adversarial review of the diff confirmed:

1. **HIGH — uPlot false bridging ramp.** The vendored uPlot 1.6.31 gap scanner
   only honours strict `null`; `Float64Array` columns can only carry NaN. So
   after a stale window, the dashed series would draw a **false bridging ramp
   across un-commanded time** on stream resume — fabricated data on an
   operator-facing chart. Fixed with a custom per-series gaps hook (`nanGaps`,
   `telemetry-charts.js` ~617–693) that scans the drawn range for non-finite
   runs, wired into every series. Validated by driving the **real** vendored
   uPlot in node with the shipped hook: 21/21 assertions, including edge runs,
   all-NaN, single-point NaN, and null+NaN merge.
2. **HIGH — production trigger unpinned.** The echo call sits inside
   `_publish_robot_state`, deliberately **before** the telemetry-suppression
   gate — but every test called the echo method directly, leaving the
   production choreography untested (exactly the mocked-ROS blind spot recorded
   in memory `feedback_mocked_ros_blind_to_choreography`). Fixed: a test now
   drives `_publish_robot_state()` with no telemetry cached and asserts the
   echo publishes once while `robot_state` stays silent.
3. **MEDIUM** — `/leg_setpoint_echo` added to the rosbag record list
   (post-session forensics parity with the GUI).
4. **MEDIUM ×2** — tracking-error panel decoupled from the leg-echo gate (no
   more freezing at last values when the echo is stale; the hand slot renders
   `--` unknown instead of a false-green hard 0 when hand telemetry is absent);
   tracking sparklines made NaN-safe (pen-up holes).
5. **LOW** — echo call moved inside `_publish_robot_state`'s `try` (containment);
   stash-before-send pinned by a fault-injection test (send `OSError` → echo
   still updates).

### Accepted tradeoffs (documented, not fixed)

- The echo rides the node-default MutuallyExclusive callback group shared with
  blocking services — a long service call can gap the "Pos (cmd)" series while
  the real stream keeps flowing. Observability-only artifact; accepted.
- The GUI staleness watchdog is `setTimeout`-based — background-tab throttling
  can extend the recorded pre-gap flatline. Accepted.
- `hand_telemetry.pos_cmd` is 0.0 until the first `HAND_CMD_ECHO` sniff, so a
  bridge restart shows a phantom hand tracking error briefly — a defensible
  improvement over the previous garbage read of vel_ff.
- `latestHandTelemetry` has no staleness watchdog — pre-existing, out of scope.
- This is the first-ever coupling of the setpoint thread to `self._lock`;
  guarded by convention — all ~40 existing holders were traced and confirmed
  to be microsecond-scale stashes.

## Fix

**Bridge** (`ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py`):

- New topic **`leg_setpoint_echo`** (Float64MultiArray, 6 floats, motor revs):
  the setpoint thread stashes the last **accepted** per-leg u0 (three plain
  assignments under `self._lock`, before the UDP send); the 100 Hz
  `robot_state` timer builds + publishes, gated on 0.5 s freshness and
  seq-deduped (~40 Hz effective). The echo call lives inside
  `_publish_robot_state`'s `try`, before the telemetry-suppression gate — the
  echo publishes even when robot_state telemetry is absent.

**Launch** (`ros_ws/src/jugglebot/launch/jugglebot_launch.py`):
`/leg_setpoint_echo` added to the rosbag record list.

**GUI** (`ros_ws/gui/js/`):

- `main.js` — subscription swapped `leg_lengths_topic` → `leg_setpoint_echo`
  (50 ms throttle); `latestCommandedLegs` now staleness-cleared after 1 s (was
  never cleared → a misleading flatline at the last commanded value); **latent
  bug fixed**: hand tracking error (~line 319) was computed from
  `latestCommandedLegs[6]` — leg-0 vel_ff in the old 18-value layout, i.e.
  garbage — now sourced from `hand_telemetry.pos_cmd` (the Phase-5
  `HAND_CMD_ECHO` value, [[2026-07-01-canbridge-phase5-hand-conduit]]).
- `telemetry-charts.js` — custom per-series `nanGaps` hook (~617–693) so NaN
  runs render as true pen-up gaps in the vendored uPlot 1.6.31 (see Discussion
  HIGH #1), wired into every series.
- `panels.js` — tracking-error panel decoupled from the leg-echo gate; hand
  slot renders `--` when hand telemetry is absent; sparklines NaN-safe.

**Tests** (`tests/ros/test_teensy_bridge_node_setpoint.py`): production-trigger
choreography test (echo publishes via `_publish_robot_state` with robot_state
silent), fault-injection stash-before-send test (send `OSError` → echo still
updates), plus echo freshness/dedup coverage.

The GUI's last dependency on `motion_bridge_node`'s **leg output** is removed.
One GUI consumer of the node remains — `motion/diagnostics` → the motion panel
(MPC-stack-only data; sole publisher is motion_bridge_node) — so the retirement
item's "verify GUI topic consumption" check must still disposition that topic
before the node is removed; this change clears the leg-output half only.

## Verification

- **Scoped suites** (`pytest tests/ros/test_teensy_bridge_node_setpoint.py
  tests/ros/test_gui_geometry.py -q`, run 2026-07-11): **73 passed**.
- **Full ros battery** (`pytest tests/ros/ -q`, run 2026-07-11): **853 passed**.
- **Full suite** (`pytest tests/ -q --ignore=tests/motion/test_bench_sysid_bridge.py`,
  run 2026-07-11): **2437 passed, 1 xfailed in 571.35 s**. (The `--ignore`
  excluded a *parallel session's* then-untracked in-progress test file, which is
  not part of this change; that file has since been committed by its own session
  in f6b8439. The run waited for that session's own concurrent suite to finish
  first — two overlapping suites load-flake the timing tests.)
- **Canonical full suite, immediately pre-commit** (`pytest tests/ -q`, no
  ignore, run 2026-07-11 after f6b8439 landed the parallel session's tests):
  **2469 passed, 1 xfailed in 574.97 s**.
- **GUI JS**: node-module syntax checks on `main.js` / `panels.js` /
  `telemetry-charts.js` all clean.
- **uPlot gaps-hook validation probe** (`node tools/probes/uplot_nan_gap_probe.js`,
  run 2026-07-11 from its committed home): drives the REAL vendored uPlot 1.6.31
  with the shipped `nanGaps` hook (extracted verbatim at run time) — **21/21
  assertions pass** (edge runs, all-NaN, single-point NaN, null+NaN merge,
  dashed-stroke invariance). Promoted to `tools/probes/` (reusable-probe rule:
  it is the regression recipe for any future uPlot upgrade).

## Open Questions

- `latestHandTelemetry` staleness watchdog (pre-existing gap, out of scope
  here) — worth adding if the phantom-freshness pattern ever misleads an
  operator.
- The MutuallyExclusive-callback-group echo gap and the `setTimeout`-based GUI
  watchdog are accepted observability-only artifacts (see Discussion); revisit
  only if operators start misreading the gaps.

## Related

- Plan: [`mvp-trajectory-bringup.md`](../plans/active/mvp-trajectory-bringup.md)
  — the migration that dormanted run_mpc (orphaning the datasource) and the
  deferred motion_bridge retirement item this unblocks.
- [[2026-07-08-mvp-autonomous-build-run]] — MPC removed from the leg path (why
  `motion_bridge_node` no longer publishes `leg_lengths_topic` in production).
- [[2026-07-06-phase13-socketcan-decommission]] — `can_node` (the topic's old
  consumer) deleted.
- [[2026-07-01-canbridge-phase5-hand-conduit]] — `HAND_CMD_ECHO`, the source of
  the `hand_telemetry.pos_cmd` now feeding the hand tracking error.
- Memory: `feedback_mocked_ros_blind_to_choreography` — the blind spot behind
  review HIGH #2.
