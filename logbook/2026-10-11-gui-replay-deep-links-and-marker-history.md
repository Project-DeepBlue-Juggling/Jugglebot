---
title: GUI replay Phase 6 - deep links to a replay moment and labelled marker trails past 5 s
type: feature
date: 2026-10-11
status: resolved
phase: "gui-rosbag-replay - Phase 6 (deep links, long marker trails)"
related_plan: gui-rosbag-replay.md
files_changed:
  - logbook/2026-10-11-gui-replay-deep-links-and-marker-history.md
  - logbook/INDEX.md
  - plans/active/gui-rosbag-replay.md
  - ros_ws/gui/js/ball-history-layer.js
  - ros_ws/gui/js/replay/ball-history.js
  - ros_ws/gui/js/replay/ui/deeplink.js
  - ros_ws/gui/js/replay/ui/index.js
  - ros_ws/gui/js/replay/ui/trackbar.js
  - tests/ros/js/ball_history_harness.js
  - tests/ros/js/ball_history_layer_harness.js
  - tests/ros/js/replay_ui_deeplink_harness.js
  - tests/ros/test_gui_ball_history_layer.py
  - tests/ros/test_gui_replay_ball_history.py
  - tests/ros/test_gui_replay_ui_deeplink.py
  - tests/ros/test_gui_replay_ui_trackbar.py
---

# GUI replay Phase 6: deep links and long marker trails

Owner decisions of 2026-10-11 resolved map tickets 07–10 (`.scratch/gui-replay/issues/`). The plan's § Phase 6 now
carries them.

## What changed

- **Deep links (ticket 07).** The URL form is `index.html?recording=<id>&t=<t>`.
  - `t` is either a bare number, meaning seconds from the recording's start, or `HH:MM:SS(.s)`, meaning local
    wall-clock time of day. Wall clock is resolved against the recording's own date and handles a recording that
    crosses midnight.
  - The link lands PAUSED at `t`, and is honoured once per page load.
  - While a rosbridge session is connected the link is refused with a notice and is not queued.
  - An unknown recording gives "Recording … not found" and stays in the lobby.
  - A `t` outside the recording is clamped with a notice; an unreadable `t` starts at the beginning with a notice.
  - The address bar never changes on its own. Links come only from the new "Copy link" button in the transport. It
    tries `navigator.clipboard`, then a textarea with `execCommand('copy')`. If both refuse, it shows a `prompt()`
    holding the URL, because the toast is not selectable.
  - Code: new pure module `js/replay/ui/deeplink.js`, wired in `js/replay/ui/index.js` and `trackbar.js`.
- **Marker trails follow the slider (ticket 10.2).** Phase 5b's "markers never extend past 5 s" rule is lifted in
  replay; live is unchanged, because its maximum is 5 s.
  - Above 5 s the far tier (`js/replay/ball-history.js` + `js/ball-history-layer.js`) also draws LABELLED mocap markers.
  - Each label is decimated to one point per 50 ms and drawn in its `labelToColour` palette colour, with the same age
    fade as balls.
  - Labels are interned per history instance, so a colour is stable across slots.
  - Cap policy: past the layer's 1,000,000-vertex cap, the oldest slots' marker rows are dropped first and balls are
    kept.
- **Not built, by owner decision.** Recorded skill and juggle state stays hidden (ticket 08). Latency hardening
  (ticket 09) is deferred until after Phase 6.

## Discussion

- **Labelled markers only, at 50 ms.** Unlabelled markers are keyed by a nearest-neighbour guess with 0.5 s memory.
  Over minutes those keys chain into wrong trails, and the ball's own unlabelled marker would only duplicate the ball
  line. At 10 ms per label, the busy bag would cross the vertex cap. The owner chose this option over "labelled at
  10 ms" and "all markers".
- **Balls-first two-sweep fill: built, measured, withdrawn.** Adding markers made the whole-recording fill of the busy
  bag (18 min) take ~160 s. The owner approved a balls-first sweep, on the Phase 5b premise that a `/balls`-only load is
  cheap (17 ms per slot). End to end it lost: the balls finished at ~130 s instead of ~160 s, but the whole fill took
  ~285 s, because every slot was loaded twice. So the code keeps ONE combined load per slot, nearest-first from the
  playhead.
  - **Hypothesis withdrawn.** My first explanation was that the Range fetch plus whole-chunk decompression dominates, so
    the topic filter cannot help. The committed probe (`tools/probes/replay_slot_load_timing.mjs`, which reports the
    message counts each load returned) does not support it. Single loads range ~20–560 ms with no consistent
    dependence on the topic filter or the content:
    - a `/balls`-only load returning 1,018 balls took 73 ms;
    - empty `/balls`-only loads took 21–386 ms;
    - `/mocap_data`-only loads took 159–529 ms.
  - **Suspected, unverified:** queueing behind other work on the shared low-priority lane, chiefly the chart digester
    that fills ±330 s of 1 s digests after an open or a seek. 17 ms is the `/balls` DECODE alone.
    *(Pointer added 2026-10-11, ticket 09: also withdrawn. Queue waits were ≤ 30 ms; the main thread was blocked in
    WebGL `bufferSubData` by the layer's whole-buffer re-uploads, and the load times above are that delay. See
    [2026-10-11-gui-replay-far-tier-append-only-uploads](2026-10-11-gui-replay-far-tier-append-only-uploads.md).)*
  - **Phase 5b's "whole busy bag in ~8 s"** stays unexplained against the ~130 s balls sweep measured here.
  - **Follow-up:** a faster long-tail fill belongs with the deferred latency work (ticket 09). Measure the lane
    contention first, then decide between a second reader and a filtered-row cache.
- **Near/far overlap.** The far tier draws the whole tail as thin lines under the near-tier ribbons, exactly as balls
  already did; it is not clipped to start 5 s back.

## Verification

- **Scoped pytest, 2026-10-11, in the build worktree:**
  - `python -m pytest tests/ros/test_gui_replay_ball_history.py tests/ros/test_gui_ball_history_layer.py tests/ros/test_gui_replay_mode.py tests/ros/test_gui_trails_wiring.py -q`:
    48 passed (before the two-sweep revert).
  - `... test_gui_replay_ball_history.py test_gui_ball_history_layer.py -q`: 22 passed (after it).
  - `python -m pytest tests/ros/test_gui_replay_ui_trackbar.py tests/ros/test_gui_replay_ui_deeplink.py -q`:
    28 passed.
- **Full gate, 2026-10-11 (`./run_tests.sh`, `temp/logs/gate_phase6_1421.log`):** 6644 passed, 1 failed. The failure
  was the overview listing race, fixed in
  [2026-10-11-gui-replay-overview-listing-race](2026-10-11-gui-replay-overview-listing-race.md). The final pre-commit
  gate is cited in the commit message.
- **Headless smoke, 2026-10-11** (worktree server on :8099, headless Chromium, busy bag `2026-10-02_12-43-28`; drivers
  in the session scratchpad, uncommitted):
  - `t=83.4` landed paused at 83.4 s and stayed paused.
  - `t=12:44:00` landed at 28.6 s (the first message is ~3.4 s after the id's time).
  - `t=99999` clamped to 1086.2 s with a notice; `t=junk` started at 0 with a notice.
  - `recording=no_such_rec` gave the not-found notice and stayed in the lobby.
  - "Copy link" in headless (no user gesture, clipboard denied) opened the `prompt()` holding
    `…?recording=2026-10-02_12-43-28&t=83.4`.
  - With the tail at the recording length: 109 slots, 715,702 vertices, 0 dropped, JS heap 16.5 MB after GC. No JS
    errors other than the expected rosbridge connection refusals.
- **Slot-load probe, 2026-10-11** (same server and browser setup):
  `node tools/probes/replay_slot_load_timing.mjs 2026-10-02_12-43-28 8099 9333 10,30,60,70,80` gave single loads of
  21–556 ms with no consistent topic dependence (JSON under `temp/probes/`).
- **Unverified:**
  - refuse-while-connected in a browser (node harness only);
  - a real-gesture copy in the owner's Chrome over plain http;
  - the owner's first look at Phases 5, 5b and 6.
