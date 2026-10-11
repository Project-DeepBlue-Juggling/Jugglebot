---
title: GUI replay Phase 5b - always-visible trail-length overlay and recording-length ball history
type: feature
date: 2026-10-11
status: resolved
phase: "gui-rosbag-replay - Phase 5b (trail overlay, long ball history)"
related_plan: gui-rosbag-replay.md
files_changed:
  - logbook/2026-10-11-gui-replay-trail-overlay-and-ball-history.md
  - logbook/INDEX.md
  - plans/active/gui-rosbag-replay.md
  - ros_ws/gui/css/viewer.css
  - ros_ws/gui/js/ball-history-layer.js
  - ros_ws/gui/js/main.js
  - ros_ws/gui/js/replay/ball-history.js
  - ros_ws/gui/js/replay/mcap-decode.js
  - ros_ws/gui/js/replay/mcap-worker.js
  - ros_ws/gui/js/replay/mode.js
  - ros_ws/gui/js/replay/sources.js
  - ros_ws/gui/js/replay/wiring.js
  - ros_ws/gui/js/trail-feed.js
  - ros_ws/gui/js/trail-settings-ui.js
  - ros_ws/gui/js/trail-settings.js
  - ros_ws/gui/js/trails-scene.js
  - tests/ros/js/ball_history_harness.js
  - tests/ros/js/ball_history_layer_harness.js
  - tests/ros/js/mcap_oracle_harness.js
  - tests/ros/js/replay_digest_lane_harness.js
  - tests/ros/js/replay_mcap_source_harness.js
  - tests/ros/js/replay_mode_harness.js
  - tests/ros/js/replay_test_support.js
  - tests/ros/js/trail_layer_harness.js
  - tests/ros/test_gui_ball_history_layer.py
  - tests/ros/test_gui_replay_ball_history.py
  - tests/ros/test_gui_replay_digest.py
  - tests/ros/test_gui_replay_mcap_source.py
  - tests/ros/test_gui_replay_mode.py
  - tests/ros/test_gui_trail_layer.py
  - tests/ros/test_gui_trails_wiring.py
  - tests/ros/test_replay_mcap_oracle.py
subsystem:
  - gui
tags:
  - feature
  - testing
---

# GUI replay Phase 5b - always-visible trail-length overlay and recording-length ball history

## Summary

Owner ask after Phase 5: an always-visible trail slider at the bottom right of the 3D scene, its maximum shown, and a
range up to the recording's length "to review throw consistency". The slider is now a compact overlay above the
replay dock; its maximum is 5 s live and the recording length in replay, on a linear-then-logarithmic scale. Past 5 s
only the BALL trails extend: every flight in `[p - tail, p]` is drawn as thin lines faded by age, loaded in the
background from `/balls` alone. Design: § Phase 5b of `plans/active/gui-rosbag-replay.md`.

## Discussion

Marker trails stay capped at 5 s. Minutes of marker history smear into blobs, and the near tier's rings, pool and
rebuild cost are sized for 5 s; the far tier is balls-only and cheap because `/balls` is 44 Hz on average. The far
data rides the existing low-priority worker lane with a new topic filter, so a slot decodes `/balls` only (17 ms
against 331 ms per slot on the busy bag). The review of the far-tier layer replaced an eager 1,000,000-vertex
allocation (about 28 MB at page load, live included, re-uploaded whole on every slot arrival) with buffers sized to
the loaded flights and released on exit.

The phase's one audit (2026-10-11) found three behaviour issues, all fixed: the slider's right end re-snapped a
non-round max (1,081,300 to 1,080,000 ms, the tail stopping 1.3 s short of the recording's start), so an exact request
for the max is now stored unsnapped; the loader could miss the oldest 10 s of the tail because its window followed a
centre that re-targets only every 10 s, so the window now reaches 10 s further back (matching its eviction bound); and
the idle far tier scheduled a timer every frame.

## Verification

- Scoped (`pytest tests/ros -q -k "gui or replay"`, run 2026-10-11 after the Unit D review edits): 491 passed, 1 skipped.
- Unit E scoped set plus the lazy-buffer test (`pytest tests/ros/test_gui_ball_history_layer.py tests/ros/test_gui_replay_ball_history.py tests/ros/test_gui_trails_wiring.py tests/ros/test_gui_replay_mode.py -q`, run 2026-10-11): 42 passed.
- Headless Chromium on the busy bag `2026-10-02_12-43-28` (run 2026-10-11): the overlay clears the replay dock (481 px
  against 501 px) in both themes; a whole-recording tail loads 50 slots and 63,480 vertices in about 8 s with nothing
  dropped; the screenshot shows the overlaid Ball Butler flights in per-ball colours, older ones fainter.
  *(Pointer added 2026-10-11, Phase 6: 50 slots is fewer than the bag's 109, and Phase 6's whole-bag fill took ~160 s
  with markers and ~130 s for a balls-only sweep, so this ~8 s is not a whole-recording figure;
  see [2026-10-11-gui-replay-deep-links-and-marker-history](2026-10-11-gui-replay-deep-links-and-marker-history.md).)*
- After the audit fixes (`pytest tests/ros -q -k "gui or replay"`, run 2026-10-11): 514 passed, 1 skipped.
- Browser re-smoke of the lazy buffers on the busy bag (run 2026-10-11): capacity 0 to 4,096 to 65,536 vertices for
  the whole recording (63,480 used), 0 after exit, no console errors, the same overlaid-flights picture.
- Full gate after the audit fixes (`./run_tests.sh`, run 2026-10-11 in the `Jugglebot-replay` worktree, log `temp/logs/gate_phase5b_final_2026-10-11.log`): **6629 passed, 9 skipped; serial 3 passed; RESULT PASS** (353 s).

## Outcome

Software-complete. The owner's look in their own browser is pending.
