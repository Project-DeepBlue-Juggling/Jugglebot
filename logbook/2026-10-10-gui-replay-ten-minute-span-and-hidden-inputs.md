---
title: GUI replay - ten-minute chart span with digest tier, and hidden input panels in replay
type: feature
date: 2026-10-10
status: resolved
phase: "gui-rosbag-replay - Phase 4 follow-up"
related_plan: gui-rosbag-replay.md
files_changed:
  - logbook/INDEX.md
  - plans/active/gui-rosbag-replay.md
  - ros_ws/gui/css/replay.css
  - ros_ws/gui/js/juggle-panel.js
  - ros_ws/gui/js/replay/cache.js
  - ros_ws/gui/js/replay/chart-store.js
  - ros_ws/gui/js/replay/mcap-worker.js
  - ros_ws/gui/js/replay/mode.js
  - ros_ws/gui/js/replay/sources.js
  - ros_ws/gui/js/replay/ui/index.js
  - ros_ws/gui/js/replay/wiring.js
  - ros_ws/gui/js/telemetry-charts.js
  - tests/ros/js/replay_chart_harness.js
  - tests/ros/js/replay_mcap_source_harness.js
  - tests/ros/js/replay_mode_harness.js
  - tests/ros/js/replay_test_support.js
  - tests/ros/test_gui_replay_chart.py
  - tests/ros/test_gui_replay_mcap_source.py
  - tests/ros/test_gui_replay_mode.py
  - logbook/2026-10-10-gui-replay-ten-minute-span-and-hidden-inputs.md
  - ros_ws/gui/js/replay/digest.js
  - ros_ws/gui/js/replay/ui/hide.js
  - tests/ros/js/replay_digest_harness.js
  - tests/ros/js/replay_digest_lane_harness.js
  - tests/ros/js/replay_ui_hide_harness.js
  - tests/ros/test_gui_replay_digest.py
  - tests/ros/test_gui_replay_ui_hide.py
subsystem:
  - gui
tags:
  - performance
  - testing
  - docs
---

# GUI replay: ten-minute chart span and hidden input panels

## Summary

After the first direct-MCAP session the owner reported replay as "very snappy" and asked for (1) chart data viewable out to 10 minutes, at lower resolution further from the playhead, and (2) the JUGGLE panel and other input surfaces (e.g. BB's target) hidden in replay. Both are done: a two-tier chart store (full rate in the resident window, 1 s min/max/mean digests out to +-330 s) with `REPLAY_MAX_SPAN_SEC` raised 120 -> 600, and a `ui/hide.js` module that hides the input regions in REPLAY and stops the juggle panel's rAF loop. Software complete; gate pending; real GPU and a real 13 h bag under the digester unverified.

## Motivation

Charts were capped at a 120 s window because every chart point lives in a resident slot and one resident recorded second costs about 1 MB of heap (measured), so 600 s at full rate would be roughly 600 MB. Input panels (jog, speed limits, BB target, the JUGGLE panel) are controls with nothing to control in a recording, and the juggle panel kept an animation loop running.

## Discussion

Why two tiers rather than a longer resident window: heap scales with resident seconds, so a 10 min window at full rate is the cost we measured as unacceptable. A chart over 600 s has far fewer pixels than samples, so far from the playhead a per-second summary loses nothing visible. Root causes behind each choice:

- **Digest the DERIVED chart signals, on the main thread.** The derivation (the vector each chart actually plots) lives on the main thread, so the digester must run there. Digests are 1 s bins `{t, n, min, mean, max}` per axis. Raw-topic digests in the worker would have had to duplicate the derivation and could drift from what the full-rate tier draws.
- **Low-priority worker lane, never memoised.** Far slots are fetched with `load(i, {lite:true})`: the worker serves all full loads before any lite load, a lite result is not memoised, and a resident slot is served free. If far slots were memoised they would stay resident and reproduce the 1 MB/s heap cost the digest tier exists to avoid. Concurrent lite loads of one slot share a single decode.
- **Digester pause rule: playback never waits on it.** It pauses while the engine is buffering or the cache has a load in flight, and takes a by-product digest of any chunk that is resident anyway (no load). A failed slot is not retried in a loop. Ring is bounded (<= 68 slots), filled nearest-first around the playhead ([5,6,4,7,3,8]), re-targeted after a > 30 s move, evicted beyond the span.
- **Per-bin clipping at the resident edge.** Digest bins overlapping the full-rate window are clipped per bin (zero-copy subarrays) so the two tiers never double-draw the same instant.
- **Gaps stay gaps.** A missing digest is one all-NaN point where neighbours are > `GAP_S` (1.5 s) apart, so an undigested stretch is not drawn as a bridged line that would assert data we do not have.
- **Envelope draw hook.** The min/max band draws between overlays and the playhead (`drawReplayEnvelope`, installed and removed with replay); `envelope(key)` is cached so there is no per-frame allocation. Digest-only rebuilds coalesce to one repaint per animation frame.
- **Hidden inputs.** `createHide({document, mode, setJuggleHidden})` toggles a `replay-hidden` class (`display:none !important`) on `#minimap-juggle`, `#bb-throw-content`, `#panel-jog`, `#panel-speed-limits`, re-applied while active so the late-built juggle panel is caught, and restores inline display on exit. The juggle panel gets `setJugglePanelReplayHidden(on)`: cancel the rAF, make `renderJugglePanel` a no-op while keeping the last snapshot, re-render and `ensureAnim()` on exit. `#panel-bb` and `#bb-content` stay visible (they show recorded state).

Accepted tradeoff: far-from-playhead data is a summary (min/mean/max per second), and filling the ring costs worker time; both are bounded by the pause rule.

## Implementation

- Unit A1: `js/replay/digest.js` (new, `createDigester`), `chart-store.js` (`digestChunk`, `setDigests`, `onInvalidate`, `residentRange`, `envelope`; only sealed chunks digested), `mcap-worker.js` (two queues), `sources.js`, `cache.js` (`loading()`).
- Unit A2: `telemetry-charts.js` (`REPLAY_MAX_SPAN_SEC` 600, initial 30 s window unchanged, envelope hook), `chart-store.js` (clipping, gap points, cached envelope, `windowFor` cap 600), `mode.js` (digester created after the entry seek, disposed first on exit; `store()`/`digester()`), `wiring.js`.
- Unit C: `js/replay/ui/hide.js` (new), `ui/index.js`, `css/replay.css`, `js/juggle-panel.js`.
- Tests: `tests/ros/test_gui_replay_digest.py` (12), `test_gui_replay_ui_hide.py` (9), chart/mode/mcap-source additions (`test_two_tier_composed_data_has_gaps_for_missing_digests`, `test_digest_bins_are_clipped_per_bin_at_the_resident_edge`, `test_envelope_hook_draws_only_over_digest_regions`, `test_digest_installs_repaint_once_per_frame`, `test_digester_lives_exactly_as_long_as_the_replay`), plus harnesses; span/wheel assertions moved to 600.

## Verification

All 2026-10-10, venv, headless Chromium with software GL for smokes.

- Unit A1: `python -m pytest tests/ros -k "gui or replay" -q` -> 440 passed, 1 skipped.
- Unit A2: same command -> 445 passed, 1 skipped. Smoke (`smoke_report_7.md`, 13 h bag `2026-06-23_20-31-20`, playhead at t0, playback running): ring of 34 slots, 22 digested at 5 s and all 34 by 11 s; buffering transitions over 10 s of playback: 0 with the digester and 0 without; heap after GC 7.2 MB at open -> 8.4 MB ring full -> 8.7 MB (no-digester run 6.9 / 7.2 / 7.5, so the ring costs about 1.2 MB); 9-chart forced redraw 1.84 ms at a 131 s span vs 2.59 ms at 600 s (software-GL rAF frames of ~450-550 ms dominate and cannot resolve this).
- Unit C: same command -> 444 passed, 1 skipped. Smoke (`smoke7.mjs`, `smoke_report_6.md`): all four regions `display:none` in REPLAY, juggle rAF count frozen at 24 over 1.5 s, regions restored and rAF resumed after exit, `#panel-bb`/`#bb-content` visible. ROS was UP, so Chromium ran with `--host-resolver-rules` refusing :9090.
- Branch gate (`./run_tests.sh`, run 2026-10-10 in the replay worktree): parallel 6536 passed, 9 skipped, **1 failed** = `tests/firmware/test_bb_fw_update_xref.py::test_bb_fw_version_matches_the_host_expectation` (the BallButler repo on this box is at FW 8 and `BB_FW_VERSION_EXPECTED` was 6 on this branch; `skill-stack` `b02c44e0` bumps it to 8 — nothing in this block touches either side); serial 3 passed.
- MERGED GATE: pending

## Outcome

Chart span is 600 s with a bounded-cost far tier and no added buffering in the smoke; replay hides the input panels. Unverified: real GPU visual behaviour; the three input panels (jog, speed limits, BB target) are already hidden when disconnected, so only the juggle panel's hide was strongly observed; left-side clip and the envelope on a real canvas (node tests only); a mid-bag playhead with at most 67 slots (68 is the loose bound); the digester under the 13 h bag beyond the 10 s smoke.

Owner latency asks carried to Phase 6 (plan): the overview strip arrives only after the niced pass; slot buffering is serial.
