---
title: GUI rosbag replay UI, Phase 3 - lobby, picker, docked trackbar, overview, absent-state and DOM fence
type: feature
date: 2026-10-10
status: resolved
phase: "gui-rosbag-replay - Phase 3 (replay UI)"
related_plan: gui-rosbag-replay.md
files_changed:
  - ros_ws/gui/index.html
  - ros_ws/gui/css/replay.css
  - ros_ws/gui/js/main.js
  - ros_ws/gui/js/replay/sources.js
  - ros_ws/gui/js/replay/ui/dom.js
  - ros_ws/gui/js/replay/ui/format.js
  - ros_ws/gui/js/replay/ui/toast.js
  - ros_ws/gui/js/replay/ui/picker.js
  - ros_ws/gui/js/replay/ui/lobby.js
  - ros_ws/gui/js/replay/ui/index.js
  - ros_ws/gui/js/replay/ui/overview.js
  - ros_ws/gui/js/replay/ui/trackbar.js
  - ros_ws/gui/js/replay/ui/absent.js
  - ros_ws/gui/js/replay/ui/fence-dom.js
  - ros_ws/gui/test_replay_engine.html
  - tests/ros/js/replay_ui_lobby_harness.js
  - tests/ros/js/replay_ui_trackbar_harness.js
  - tests/ros/js/replay_ui_absent_harness.js
  - tests/ros/js/replay_ui_fence_dom_harness.js
  - tests/ros/test_gui_replay_ui_lobby.py
  - tests/ros/test_gui_replay_ui_trackbar.py
  - tests/ros/test_gui_replay_ui_absent.py
  - tests/ros/test_gui_replay_ui_fence_dom.py
  - tests/ros/test_gui_replay_format_contract.py
  - plans/active/gui-rosbag-replay.md
  - logbook/2026-10-10-gui-replay-browser-engine-phase2.md
  - logbook/2026-10-10-gui-replay-ui-phase3.md
  - logbook/INDEX.md
subsystem:
  - gui
tags:
  - testing
  - docs
---

# GUI rosbag replay UI, Phase 3

## Summary

Phase 3 of `plans/active/gui-rosbag-replay.md`: the real replay UI on top of Phase 2's engine, replacing the throwaway dev page. Software complete; the owner's browser session is pending. The owner resolved wayfinder ticket 03 on the mock (`ros_ws/gui/test_replay_mock.html`, kept as the ticket's artifact): layout A (trackbar and transport docked at the bottom where the command overlay sits), compact overview (state bands plus ticks, no per-topic rows), lobby and picker as mocked. The orchestrator's calls on top of that: which ticks exist (below) and "not in this recording" is a dimmed element, never a frozen value.

## Motivation

Phase 2 left the engine reachable only from `test_replay_engine.html`; nothing in the real GUI called `getReplayMode`. Phase 3 binds it to the lobby (the disconnected command overlay), a picker modal, a trackbar with hotkeys, and the two DOM contracts the charting decisions require: every command surface visibly disabled in replay (decision 10) and absent topics visibly absent (decision 18).

## Design and Implementation

Three units, all new modules under `ros_ws/gui/js/replay/ui/` plus `css/replay.css`.

- **Unit 1, lobby and picker.** `index.js` (entry; `main.js` step 6d calls `initReplayUi(getReplayMode({...}))` before `ros.init()`; main imports the UI, never the reverse), `lobby.js` (owns the disconnected overlay buttons, the `REPLAY YYYY-MM-DD` header text and routing of mode `notice` events), `picker.js` (modal: date, duration, size, topic chips, text filter, ticket 05 states and 409 messages, backend-unavailable banner), `toast.js`, `dom.js` (`h()` builder; every module takes `document` through a factory), `format.js`. `index.html` gains `#replay-lobby`, `#replay-dock`, `#replay-banner`, `#replay-backend-banner`.
- **Unit 2, trackbar.** `overview.js` (layout and zoom math, strip drawn once per recording and zoom), `trackbar.js` (one rAF loop driven by `engine.state()`, hotkeys, drag and scrub, FF/RW ladder, mount/unmount on `mode.on('state')`).
- **Unit 3, contracts.** `absent.js` (`ABSENT_TABLE`: topic to region; class `replay-absent` plus tooltip "not in this recording"; only judges an authoritative topic set, i.e. a session snapshot or a complete recording's manifest; re-evaluates on conversion completion via `source.onChange`). `fence-dom.js` (`FENCE_SURFACES` = the six surfaces of decision 10; `disabled` plus "unavailable in replay"; a MutationObserver re-fences anything a live panel re-enables on the next recorded message; exit restores prior state). `sources.js` session source gained `topicSet()`. `test_replay_engine.html` deleted.

Deviations a future reader should know:

- **Picker per-row fetches.** The list endpoint carries no topics or progress, so the picker fetches `/manifest` and `/status` per row, best effort and serially. Follow-up candidate: extend the list response.
- **`ABSENT_TABLE` is a judgement mapping** (a region is dimmed only when NONE of its topics is present). Mapped: `#panel-flags`/`#bus-voltage-value` <- `/robot_state`; `#state-badge`/`#state-sub-mode` <- `/orchestrator_state`; `#panel-bb` <- `/bb/heartbeat` + `/bb/calibration_result` + `/bb/calibration_attempt`; `#panel-catching-cone` <- `/cone/heartbeat` + `/cone/timing_result`; `#panel-motion` <- `/motion/diagnostics`; `#panel-tracking` <- `/robot_state` + `/leg_setpoint_echo` + `/hand_telemetry`; `#panel-can` <- `/profile` + `/link_status` (audit W4 corrected the first draft, which mapped tracking to `/mocap_data` and CAN to `/udp_diag`). Unmapped subscribed topics (no region is dimmed for them): `/clock_diag`, `/control_mode_topic`, `/rigid_body_poses`, `/skills/attempt`, `/mocap_data`, `/udp_diag`.
- **Sentinel fix (smoke).** Empty bags report `start_ns = INT64_MAX` and sorted to the top of the picker; `format.js` `nsOk` now rejects `start_ns >= 4e18` and falls back to `mtime - duration_s`. Regression test: the `r_empty` row in the lobby harness.
- **Phase-end audit fixes (2026-10-10, all eleven applied):** W1 trackbar keydown is capture-phase and `stopImmediatePropagation`s only the keys it handles (Space no longer also pauses the live charts); W2 `holdToConfirm` narrowed to `.cmd-btn.hold-fillable` (the chart signal-toggle pills carry `.hold-fillable` and stay enabled); W3 `/overview` retry is frame-count backoff (200 frames), no new wall-clock reads; W4 `ABSENT_TABLE` topics corrected (above); W5 `initReplayUi` wrapped in try/catch in `main.js` step 6d; N1 fence observer watches only the surface roots and prunes detached controls; N2 the lobby shows only after a real `disconnected` edge, never on the initial default; N3 picker guards double-open, `probe()` no longer bumps `loadToken`, enrichment skips already-enriched rows, batches renders and aborts on close; N4 sentinel regression test; N5 `test_gui_clock_contract.py` now scans `js/**/*.js` recursively (one site surfaced: `replay/cache.js` retry-backoff `Date.now()`, marked wall-clock: it paces real HTTP failures).
- **`bb_throw` tick dropped.** Unit 2 had guessed a `bb_throw` overview kind; `schema.OVERVIEW_TICK_KINDS` has none and no recorded topic carries a BB throw outcome. The overview draws exactly fault, skill_attempt, catch_event, bb_calibration (homed/levelled are not ticks). A contract test pins the drawn kinds to the schema tuple. Phase 4 may derive a throw tick from `/balls`.
- **FF/RW ladder.** Speed ladder x0.25 ... x8; RW from forward enters reverse at x1. Seeks (keys, overview click, scrub) are clamped to the converted frontier.
- **Dev page retired.** `test_replay_engine.html` is deleted; the real UI replaces it.
- **Related, committed separately:** `d69a0547`, the reconnect-edge flicker fix (`2026-10-10-gui-replay-reconnect-edge-flicker.md`).

## Verification

- Scoped, run 2026-10-10: `python -m pytest tests/ros -k "gui or replay" -q` gave **359 passed, 1 skipped**.
- Scoped, run 2026-10-10: `python -m pytest tests/ros/test_replay_*.py -q` gave **41 passed**.
- New tests: `tests/ros/test_gui_replay_ui_{lobby,trackbar,absent,fence_dom}.py` (node harnesses in `tests/ros/js/replay_ui_*_harness.js`) and two contract tests in `tests/ros/test_gui_replay_format_contract.py` (`test_overview_tick_kinds_drawn_by_ui_are_in_schema`, `test_picker_key_topics_are_in_allowlist`).
- Full gate (`./run_tests.sh`, run 2026-10-10 in the replay worktree, after the audit fixes): parallel **6386 passed, 9 skipped in 282.88 s**; serial 3 passed in 10.39 s; PASS.
- Headless browser smoke (2026-10-10, Chromium headless over the DevTools protocol against the worktree served on :8099 with ROS down): steps load / lobby / picker (469 rows) / open (22 MB recording, 19.6 s cold conversion to REPLAY, 1.3 s re-open from cache) / replay transport + hotkeys + 56 fenced controls / exit all PASS; no value→'--' reversal during 5 s of play (the flicker fix holds). One bug found and fixed: empty bags report start_ns = INT64_MAX and sorted to the top of the picker (format.js sentinel, regression test added).

## Outcome

The replay path exists in the real GUI: lobby, picker, docked transport, overview, dimmed absent regions and a visibly fenced command surface. On the live path the only changes are the mount points; the lobby stays hidden until a real connect attempt has failed (a `disconnected` edge), so a cold page load does not flash it, and a throw in `initReplayUi` is caught and logged without affecting the live GUI.

## Next

Owner follow-ups: no `jugglebot-gui.service` restart is needed for this phase (`git diff --stat` shows no change to `ros_ws/gui/replay/*.py` or `gui_server.py`; the static files are served fresh). With the ROS stack down, open the GUI and try lobby, picker, replay. Phase 4 (trails) waits on ticket 04 (prototype `ros_ws/gui/test_replay_trails.html`, committed on skill-stack).
