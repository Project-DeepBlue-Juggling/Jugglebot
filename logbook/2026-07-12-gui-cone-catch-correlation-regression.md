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
  - eedcbed
  - 4f774b0  # investigation addendum
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
- **Operator live confirmation** (2026-07-12, evidence: the session 18-37-27
  rosbag — the first post-rebuild session with real taps): **5 piezo taps →
  5 `/cone/catch_event` → 5 matched `/cone/timing_result` → the GUI cone panel
  updated on every tap. CONFIRMED.** Note the pickup requirement was stronger
  than this entry originally stated: a **colcon rebuild + relaunch** (`ros2
  launch` runs the *installed* launch copy), not a relaunch alone — see the
  Addendum for the same-day "unreliability" detour this caused.

## Outcome

The catching-cone catch pipeline is wired end-to-end again: `catch_correlation_node`
is back in the production launch (restoring `cone/timing_result` for the GUI and
the rosbag), and the cone panel now gives per-tap heartbeat feedback that
survives any future correlation-half outage. The launch tripwire pins the
node against silent re-orphaning. Live-confirmed the same day (session
18-37-27: 5 taps → 5 matched timing_results → the GUI updated on every tap)
after the operator's colcon rebuild — see the Addendum for the stale-install
detour and the two flagged residuals.

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

## Addendum (2026-07-12, same day) — "unreliability" investigation: transport refuted, stale-install root cause

Operator report, a few hours after the fix landed: catch events **displayed
only once across several sessions**; the operator hypothesized
`cone/catch_event` topic interference. A **read-only investigation** over
`~/Desktop/rosbags/*` (mcap) plus the `~/.ros/log` node rosters, covering all
10 of the day's sessions, **refuted the transport hypothesis** and found a
stale-install root cause instead.

**Transport refuted.** In every session with genuine taps, the bagged
`/cone/catch_event` message count **equals** the genuine tap count (the
heartbeat `last_catch_seq` delta): 10=10, 19=19, 6=6, 5=5 — **zero drops**
anywhere. There is no interference on the topic.

**Root cause: `ros2 launch` runs the INSTALLED launch copy.** The restored
`catch_correlation_node` (committed in eedcbed at ~18:20) only entered the
node roster after the operator's **colcon rebuild**, between the 18-30-05 and
18-34-09 sessions. Every pre-rebuild session had `catch_event` flowing with
**zero** `timing_result`s — the node simply wasn't running. 18-37-27, the
first post-rebuild session with real taps, **worked perfectly** (5 taps → 5
catch_events → 5 matched timing_results → the GUI green on every tap) — that
session is the fix's live confirmation, now cited in Verification.

| session | synced | taps | catch_event msgs | corr node running | timing_results | GUI |
|---|---|---|---|---|---|---|
| 10-37-19 | never | 0 | 0 | no | 0 | — |
| 10-42-51 | never | 0 | 0 | no | 0 | — |
| 15-57-59 | yes | 10 | 10 | **NO** | 0 | dead |
| 16-06-23 | yes | 0 | 0 | no | 0 | — |
| 16-08-47 | yes | 19 | 19 | **NO** | 0 | dead |
| 18-30-05 | yes | 6 | 6 | **NO** | 0 | dead |
| 18-34-09 | reboot | 0 | 0 | yes | 0 | — |
| 18-37-27 | yes | 5 | 5 | **YES** | **5 (all matched)** | **WORKS** |
| 18-42-22 | yes | 0 | 0 | yes | 0 | — |
| 18-44-56 | reboot | 0 | 0 | yes | 0 | — |

The operator's "only once did I observe catch_event publishing" observation
was almost certainly `ros2 topic echo` **false-negatives** — the known Foxy
flakiness on this box (memory `reference_ros2_topic_echo_flaky_foxy`); the
bags are authoritative, and they show the topic publishing in every tap
session.

**Two residuals flagged for the operator:**

1. **A stale top-level `install/` tree exists beside `ros_ws/install/`** —
   its installed `jugglebot_launch.py` is dated 2026-07-09 and contains **no**
   `catch_correlation_node` (verified: zero references, vs 3 in the current
   `ros_ws/install/` copy). Sourcing the wrong tree silently resurrects this
   exact regression. Recommend deleting it; operator's call.
2. **Suspected cone MCU restarts mid-testing**: the 18-34-09 and 18-44-56
   sessions show `last_catch_seq` resetting to 0 and `have_any_catch`
   clearing (the "reboot" rows above). If the operator did NOT deliberately
   power-cycle the cone at those times, that is a separate power/wiring
   intermittency lead — independent of everything else in this entry.

**Process lesson**: the fix's operator guidance said "relaunch required" when
the actual requirement was "**colcon build + relaunch**" (`ros2 launch` reads
the installed copy, not the source tree) — captured as memory
`feedback_ros_ws_changes_need_colcon_guidance`.
