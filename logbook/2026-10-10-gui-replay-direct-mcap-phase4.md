---
title: GUI replay Phase 4 - direct MCAP source, browser-side decode, converter cache retired
type: feature
date: 2026-10-10
status: resolved
phase: "gui-rosbag-replay - Phase 4 (direct MCAP)"
related_plan: gui-rosbag-replay.md
files_changed:
  - logbook/2026-10-10-gui-replay-direct-mcap-phase4.md
  - logbook/INDEX.md
  - plans/active/gui-rosbag-replay.md
  - ros_ws/gui/css/replay.css
  - ros_ws/gui/gui_server.py
  - ros_ws/gui/index.html
  - ros_ws/gui/js/replay/allowlist.js
  - ros_ws/gui/js/replay/chunk.js
  - ros_ws/gui/js/replay/mcap-decode.js
  - ros_ws/gui/js/replay/mcap-worker.js
  - ros_ws/gui/js/replay/mode.js
  - ros_ws/gui/js/replay/slot.js
  - ros_ws/gui/js/replay/sources.js
  - ros_ws/gui/js/replay/ui/absent.js
  - ros_ws/gui/js/replay/ui/format.js
  - ros_ws/gui/js/replay/ui/lobby.js
  - ros_ws/gui/js/replay/ui/picker.js
  - ros_ws/gui/js/replay/ui/trackbar.js
  - ros_ws/gui/js/replay/wiring.js
  - ros_ws/gui/lib/mcap-bundle.min.js
  - ros_ws/gui/lib/msgpack.min.js
  - ros_ws/gui/lib/VENDORED.md
  - ros_ws/gui/replay/api.py
  - ros_ws/gui/replay/cache.py
  - ros_ws/gui/replay/convert.py
  - ros_ws/gui/replay/decode.py
  - ros_ws/gui/replay/__init__.py
  - ros_ws/gui/replay/overview.py
  - ros_ws/gui/replay/recordings.py
  - ros_ws/gui/replay/schema.py
  - tests/ros/js/mcap_oracle_harness.js
  - tests/ros/js/replay_chart_harness.js
  - tests/ros/js/replay_feed_harness.js
  - tests/ros/js/replay_mcap_source_harness.js
  - tests/ros/js/replay_mode_harness.js
  - tests/ros/js/replay_slot_pin_harness.js
  - tests/ros/js/replay_test_support.js
  - tests/ros/js/replay_ui_absent_harness.js
  - tests/ros/js/replay_ui_lobby_harness.js
  - tests/ros/js/replay_ui_trackbar_harness.js
  - tests/ros/_replay_chunks.py
  - tests/ros/_replay_fixture.py
  - tests/ros/_replay_srv.py
  - tests/ros/test_gui_replay_bridge.py
  - tests/ros/test_gui_replay_chart.py
  - tests/ros/test_gui_replay_feed.py
  - tests/ros/test_gui_replay_mcap_source.py
  - tests/ros/test_gui_replay_mode.py
  - tests/ros/test_gui_replay_ui_absent.py
  - tests/ros/test_gui_replay_ui_lobby.py
  - tests/ros/test_gui_replay_ui_trackbar.py
  - tests/ros/test_gui_vendor_pin.py
  - tests/ros/test_replay_api.py
  - tests/ros/test_replay_convert.py
  - tests/ros/test_replay_e2e.py
  - tests/ros/test_replay_mcap_oracle.py
  - tests/ros/test_replay_overview.py
  - tests/ros/test_replay_range.py
  - tests/ros/test_replay_stdlib.py
  - tools/gui_vendor/build.sh
  - tools/gui_vendor/entry.mjs
  - tools/gui_vendor/.gitignore
  - tools/gui_vendor/package.json
  - tools/gui_vendor/package-lock.json
  - tools/systemd/jugglebot-gui.service
  - tools/systemd/README.md
subsystem:
  - gui
tags:
  - testing
  - docs
  - performance
---

# GUI replay Phase 4, direct MCAP source

## Summary

Phase 4 of `plans/active/gui-rosbag-replay.md` (design: wayfinder ticket 06). The browser now reads the `.mcap` itself over HTTP Range and decodes it in a module Web Worker; the converter cache of Phase 1 (convert once into `temp/replay_cache/`, serve msgpack chunks) is deleted. The server keeps a listing, a Range-capable `file` route and a background overview pass for the timeline strip. Software complete; the gate is pending and the owner's browser is unverified (see Outcome).

## Motivation

Ticket 05 chose convert-once because decoding dominated (1.0-1.3 k msg/s with `mcap_ros2`). The cost of that choice: the only wait that scaled with recording length was the conversion itself, about 4x real time on the Jetson (a 114 s recording took 18 s; a 13 h bag would take hours). Opening a recording should cost one slot, not the recording.

## Discussion

Decision reversed: ticket 05's convert-once cache gave way to ticket 06's direct MCAP. Root causes, in order of weight:

- **The wait.** Conversion at ~4x real time was the only cost proportional to recording length. A browser worker decodes only the slots the playhead touches. The spike measured 31x real time cold and 57-82x steady under node on the Jetson, with 0 mismatches against the Python oracle, so a slot costs a fraction of a second.
- **The byte cost is noise.** The converted cache was about 5x smaller than the raw MCAP, but over Ethernet to the desktop browser that difference does not show; the Range reads fetch only the touched slots anyway.
- **The Source seam made the swap local.** Phase 2's `Source` interface (`range/chunkIndex/status/load/window/latestBefore/timeline`) meant `McapSource` replaced `RecordingSource` without touching the engine, charts or dispatch.
- **Less state to keep correct.** No cache means no eviction, no stale-format discard, no partial-cache rebuild, no disk guard, no per-row manifest/status fetches. `convert.py` survives as the Python oracle the JS decode is pinned against, so one contract (`schema.py`) still has an independent second implementation.

Accepted tradeoffs: the browser needs module workers (Chrome/Edge 80+, Firefox 114+), the unindexed (killed) recordings are refused `no_index` instead of converted, and a 119 kB vendored bundle is now a pinned artefact.

## Design and Implementation

Five units (design section 7), dependency order U1, U2 and U3a, U3b, U4, U5.

- **U1 vendoring.** `tools/gui_vendor/{package.json,package-lock.json,entry.mjs,build.sh,.gitignore}` build `ros_ws/gui/lib/mcap-bundle.min.js` (119,335 B, sha256 `bcb05709938b86ae5dba7c97b3ba53084c211c3915dd9e9e3d61e23c7c4bf36d`, identical across two builds). Exports `McapIndexedReader, MessageReader, parse`. Pins `@mcap/core` 2.3.0, `@foxglove/rosmsg` 5.0.5, `@foxglove/rosmsg2-serialization` 3.1.2, esbuild 0.28.2. `lib/VENDORED.md` updated.
- **U2 decode and oracle.** `js/replay/mcap-decode.js` (`openRecording, decodeSlot, latestRow, flatten, slotOf, httpReadable, McapError`), `js/replay/allowlist.js`; fixture changes in `tests/ros/_replay_fixture.py`; `tests/ros/js/mcap_oracle_harness.js`.
- **U3a backend routes.** New stdlib `replay/recordings.py` (struct footer read, `metadata.yaml` parse, memoised per path/size/mtime); `api.py` rewritten; `gui_server.py` drops `--cache-dir/--cache-cap-gb/--min-free-gb`, adds `--overview-dir`; `cache.py` deleted; `tools/systemd/jugglebot-gui.service` and `README.md` updated; `tests/ros/_replay_srv.py`.
- **U3b overview.** `replay/decode.py` (extracted from `convert.py`), `replay/overview.py`, `convert.py` (oracle; `--bulk` gone), `schema.py` docstring (`OPENED_MARK` removed), `test_replay_overview.py`; `test_replay_e2e.py` deleted.
- **U4 browser source.** `js/replay/mcap-worker.js` (async FIFO; `open/load/latest/close` to `opened/slot/row/error`; `t` arrays transferred; `// wall-clock:` marked), `slot.js`, `sources.js` `McapSource`, `chunk.js` `chunkFromRecord` (msgpack gone: lib, script tag, VENDORED row), `wiring.js` module-worker factory, `mode.js` hold.
- **U5 UI cut.** `ui/picker.js`, `ui/format.js` (listing-only rows, no per-row fetches, dimmed no-index / in-progress, new refusal vocabulary, amber "timeline overview unavailable" note vs the red unreachable-listing banner), `ui/lobby.js` ("REPLAY  opening..."), `ui/trackbar.js` (frontier and converted-span removed; `subscribeSource` on `onChange`), `ui/absent.js`, `css/replay.css`, `replay/__init__.py` docstring.

### Backend cut

Routes under `/api/replay/`: `GET recordings` (row: `id, size_bytes, mtime, closed, indexed, in_progress, duration_s, message_count, start_ns, topics{counts}, overview`; `overview_available` is a top-level key beside `recordings`); `GET|HEAD recordings/<id>/file` (200/206/416, 412 on `If-Match` mismatch, 409 `recording_in_progress`|`no_index`, 400/404; `Accept-Ranges`, ETag, `no-store`, 1 MiB streaming); `GET recordings/<id>/overview` (200, 202 queued|computing, 409, 503, 500; a failure sticks until the bag changes). Deleted: `cache.py` (18 cache tests, names `test_open_convert_serve_and_reopen` through `test_old_format_cache_chunk_and_files_404`), the open/status/manifest/chunks routes, the three cache flags, `lib/msgpack.min.js`, `test_replay_e2e.py`, four `RecordingSource` tests. `convert.py` is now the oracle.

### Contract tests

- JS decode == Python oracle on the fixture: 4 slots, 446,741 values, 0 mismatches (`test_replay_mcap_oracle.py`, 7 tests). Decode on the Jetson: open 22-24 ms, 4 slots 463-585 ms.
- Range handling, file route vs the server (`test_replay_range.py`); overview pass == oracle overview (`test_replay_overview.py`, 9 tests).
- Allowlist JS pin; vendored-bundle sha256 pin (`test_gui_vendor_pin.py`, 4 tests).
- Slot math pinned equal across the main thread (`slot.js`), the worker (`mcap-decode.js`) and Python (`schema.py`).
- `test_gui_replay_mcap_source.py` (11 tests) plus a mode hold test; feed and chart tests now read oracle JSON (`tests/ros/_replay_chunks.py`, `tests/ros/js/replay_test_support.js`).

### Fixture findings

- Dependency headers in the embedded schema must read `MSG: pkg/Type`; the old `pkg/msg/Type` made the JS parser skip 6 topics.
- The MCAP writer's zstd default is not decodable by the vendored reader without a decompressor, so the fixture defaults to `compression="none"` (zstd/lz4 opt-in).

### Overview-pass timings

44 MB / 157 s bag: 3.6 s (10 bb_calibration ticks, 2 bands). 843 MB / 13.1 h bag: 265 s, niced and under contention (0 ticks, a BOOT to FAULT band). `/robot_state` is read by a CDR flag walk self-checked against `rosbags` every 20,000th message; only `/orchestrator_state`, `/cone/catch_event`, `/skills/attempt`, `/bb/calibration_attempt` are fully decoded.

### U4 deviations from the design

- **`slot.js` twin.** The main-thread `slotOf` lives in its own small module so the 119 kB bundle stays off the main thread; a test pins it equal to the worker copy and to Python.
- **Hold.** `mode.js` does an explicit slot-0 load BEFORE the ordered entry; a refusal therefore leaves the live GUI untouched, then enters REPLAY paused.

### U5 smoke (`smoke5.mjs`, headless Chromium, ROS down, 2026-10-10)

24 PASS / 0 FAIL. 473 rows listed; 3 killed recordings dimmed "no index"; the newest 45 MB bag opens in 0.93-0.99 s in an isolated probe page (4-4.8 s under the software-GL smoke load); the 843 MB / 13 h bag opens in 0.25 s with seek-to-end 2.1-2.7 s in-page; the overview strip drew 2 bands + 8 ticks when `/overview` landed; opening a killed recording is refused `no_index`. Two defects the smoke found and fixed: the 409 reason now also travels in an `X-Replay-Reason` header (HEAD has no body; `api.py` and `mcap-decode.js`), and the overview strip now redraws on `source.onChange`.

## Verification

All 2026-10-10, venv.

- U1: `pytest tests/ros/test_gui_vendor_pin.py -q` -> 4 passed.
- U2: `pytest tests/ros/test_replay_mcap_oracle.py -q` -> 6 passed (0 mismatches over 446,741 values).
- U3a/U3b: `pytest tests/ros/test_replay_*.py tests/ros/test_gui_vendor_pin.py -q` -> 69 passed.
- U4/U5: `python -m pytest tests/ros -k "gui or replay" -q` -> 416 passed, 1 skipped (3 runs green); `pytest tests/ros/test_replay_*.py -q` -> 68 passed.
- Smoke: `smoke5.mjs` headless Chromium -> 24 PASS / 0 FAIL.
- Phase-end audit fixes (2 tests added) then `python -m pytest tests/ros -k "gui or replay" -q` → 418 passed, 1 skipped (run 2026-10-10).
- Full gate (`./run_tests.sh`, run 2026-10-10 in the replay worktree, after the audit fixes): parallel **6498 passed, 9 skipped in 272.25 s**; serial 3 passed in 12.63 s; PASS.

## Migration / owner steps

1. `rm -rf temp/replay_cache/` in the skills worktree (gitignored, no longer read).
2. Reinstall the unit: `sudo cp tools/systemd/jugglebot-gui.service /etc/systemd/system/ && sudo systemctl daemon-reload && sudo systemctl restart jugglebot-gui`. Never while another session is using the GUI.
3. `temp/replay_overview/` fills on demand (first overview GET per recording).
4. Module workers need Chrome/Edge 80+ or Firefox 114+ (design question Q3: which browser runs on the Win10 desktop is unconfirmed).

## Outcome

Software complete and merged-ready pending the gate. Unverified: real GPU / visual behaviour, and the owner's browser (the smoke ran headless Chromium on the Jetson with software GL). Open owner question carried from the design: auto-play after the hold (default stays paused).
