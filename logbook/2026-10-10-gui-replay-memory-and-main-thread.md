---
title: GUI replay - memory and main-thread cost (session buffer removed, typed per-leaf slots, derive-once charts)
type: bugfix
date: 2026-10-10
status: resolved
phase: "gui-rosbag-replay - Phase 4 follow-up (memory)"
related_plan: gui-rosbag-replay.md
files_changed:
  - logbook/2026-10-10-gui-replay-memory-and-main-thread.md
  - logbook/INDEX.md
  - plans/active/gui-rosbag-replay.md
  - ros_ws/gui/css/replay.css
  - ros_ws/gui/js/catching-cone-model.js
  - ros_ws/gui/js/replay/cache.js
  - ros_ws/gui/js/replay/chart-store.js
  - ros_ws/gui/js/replay/chunk.js
  - ros_ws/gui/js/replay/digest.js
  - ros_ws/gui/js/replay/engine.js
  - ros_ws/gui/js/replay/mcap-decode.js
  - ros_ws/gui/js/replay/mcap-worker.js
  - ros_ws/gui/js/replay/mode.js
  - ros_ws/gui/js/replay/session.js
  - ros_ws/gui/js/replay/sources.js
  - ros_ws/gui/js/replay/ui/absent.js
  - ros_ws/gui/js/replay/ui/format.js
  - ros_ws/gui/js/replay/ui/index.js
  - ros_ws/gui/js/replay/ui/lobby.js
  - ros_ws/gui/js/replay/ui/picker.js
  - ros_ws/gui/js/replay/wiring.js
  - ros_ws/gui/js/ros-bridge.js
  - ros_ws/gui/test_robot_models.html
  - tests/ros/js/mcap_oracle_harness.js
  - tests/ros/js/mcap_worker_transfer_harness.js
  - tests/ros/js/replay_cache_harness.js
  - tests/ros/js/replay_chart_harness.js
  - tests/ros/js/replay_digest_harness.js
  - tests/ros/js/replay_feed_harness.js
  - tests/ros/js/replay_mcap_source_harness.js
  - tests/ros/js/replay_mode_harness.js
  - tests/ros/js/replay_test_support.js
  - tests/ros/js/replay_typed_columns_harness.js
  - tests/ros/js/replay_ui_absent_harness.js
  - tests/ros/js/replay_ui_lobby_harness.js
  - tests/ros/test_gui_replay_bridge.py
  - tests/ros/test_gui_replay_cache.py
  - tests/ros/test_gui_replay_chart.py
  - tests/ros/test_gui_replay_digest.py
  - tests/ros/test_gui_replay_feed.py
  - tests/ros/test_gui_replay_mcap_source.py
  - tests/ros/test_gui_replay_memory_contract.py
  - tests/ros/test_gui_replay_mode.py
  - tests/ros/test_gui_replay_ui_absent.py
  - tests/ros/test_gui_replay_ui_lobby.py
  - tests/ros/test_replay_mcap_oracle.py
  - tests/ros/test_replay_typed_columns.py
subsystem:
  - gui
tags:
  - performance
  - testing
  - docs
---

# GUI replay: memory and main-thread cost

## Summary

Owner, 2026-10-10 evening: Win10 Chrome 154, tab at 3 GB, "Aw, Snap! Out of Memory" after panning and zooming a ~7 min recording, with lag and slow buffering. Three units landed. M1 deleted the live session buffer end to end and halved the residency (8 slots, 10 s ahead). M2a typed the slot records per leaf and transferred them instead of cloning. M2b made the chart store derive once per chunk from the typed columns and coalesced resident changes to one chart rebuild per animation frame. The first hypothesis (the session buffer at about 1 MB per recorded second dominates) was reframed by measurement; see Discussion. Software complete; the owner's Win10 run is unverified.

## Symptom

Pan/zoom on a ~7 min recording, 3 GB tab, OOM crash. Before the crash: lag, slow buffering, long main-thread stalls.

## Hypothesis

The orchestrator's first reading: the live session buffer (the 600 s ring built at message arrival in `ros-bridge.js`) costs about 1 MB per recorded second and dominates; a scalar-typed slot layout would then fix the replay slots.

## Measurement

Headless Chromium on the Jetson (heap after forced GC unless noted), scripts in the session scratchpad. Bags: `2026-10-02_12-43-28` (busy, 109 slots, ~7 300 msg per 10 s slot), `2026-06-17_15-13-42` (7 min, the owner's case), `2026-06-23_20-31-20` (the 13 h bag: low traffic, so a poor case for memory; used only for the ring-fill smoke).

| Quantity | Measured |
|---|---|
| Live session buffer, record path (120 s, 87 620 msgs) | +8.21 MB, i.e. 41 MB projected at 600 s, about 94 B per message; hydrate garbage about 1.5 kB per message (about 1.1 MB/s) |
| Live idle heap, 60 s | 6.0 MB |
| Replay slot, busy bag, decode + structuredClone | 16.5-17.9 MB per slot (about 1.7 MB per recorded second: the "1 MB/s" figure was this hydrated replay-slot shape, not the session buffer) |
| Scalar-only typed layout (variant A/B) | 16.7 of 17.0 MB (2 % saved) |
| Per-leaf CSR typing of arrays of objects (variant C) | 5.4 of 17.0 MB (68 % saved) |
| Main thread per slot arrival (clone + derive twice + nine chart rebuilds) | about 600 ms vs 14 ms worker decode work |
| 7 min bag, 90 s seek storm | floor 52.9 MB (peaks 63.4), 287 long tasks, 93.7 s total, worst 839 ms |

## Discussion

The reframing. The session buffer is 41 MB per 600 s plus about 1.1 MB/s of garbage, not a gigabyte. The "about 1 MB per recorded second" the owner and orchestrator had in mind was the cost of a hydrated replay slot. A scalar-only typed layout saves 2 %, because the slot cost is arrays of OBJECTS (`/balls`, `/rigid_body_poses`, `/mocap_data` bodies), not scalar columns: each element is a heap object with its own fields. Per-leaf CSR typing (an `offsets` Uint32Array plus one typed column per leaf) removes the objects and saves 68 %. Had we shipped the scalar layout first we would have spent a unit for 2 %.

Bytes were not the whole story. The lag is main-thread work per slot arrival: the structured clone of a 17 MB heap graph, deriving the chart signals twice (once for the resident store, once for the digest), and nine chart rebuilds, about 600 ms against a 14 ms worker decode. So transfer (typed buffers detach, no clone), derive-once (one per-chunk cache shared by `setResident` and the digest) and one `setData` per animation frame matter as much as the bytes. Crash risk and stalls share one cause (per-row object churn), which is why one typed shape fixes both.

Why the session buffer was dropped although it was cheap (41 MB). Low value: the newest recording covers "replay the last session". It added churn on every live message (the tap, hydrate garbage), and removing it removes a second feed source, one code path fewer in `mode.js`, `wiring.js` and the lobby. Cost of removal: the last few seconds before a live disconnect are not replayable until the bag closes. Accepted.

Residency cut. `maxResident` 16 to 8 and `aheadSec` 20 to 10: with the slot at 5.6 MB the worst-case resident set is about 45 MB, and the prefetch only needs to cover one decode latency (about 340 ms including HTTP) at the speeds offered. Cache expectations were re-derived rather than loosened.

Not done, and why. Charts at 20 Hz: the digest and the two-tier store already bound the draw cost and the owner's complaint was not chart rate. A worker-side digest: it would move the by-product digest off the main thread but needs the derivation code in the worker (a second copy to keep in sync); after derive-once the digest is a by-product of work already done, so it was deferred. The 3 GB crash itself was never reproduced headless: the busy-bag floor was 70 MB before, so the owner's 3 GB must involve Win10 Chrome allocator or GPU behaviour we cannot see here; the change removes the largest per-row churn but cannot prove the crash gone.

## Fix

M1 (session buffer removed, residency, cone): deleted `js/replay/session.js`; in `sources.js` removed `SessionBufferSource`/`createSessionBuffer`/`SESSION_BUFFER_SEC` (only `McapSource` exported); removed the `getSessionBuffer` tap in `ros-bridge.js`, the `{kind:'session'}` branch and the 'no session buffer' refusal in `mode.js`/`wiring.js`, the "Replay last session" button (`ui/lobby.js`), the picker session row and `sessionInfo` (`ui/picker.js`), the 'memory' state cell and session `buildRows` argument (`ui/format.js`), `ui/index.js` and `ui/absent.js` session branches, a css rule; `cache.js` `maxResident` 8, `aheadSec` 10. Peer hand-off: `js/catching-cone-model.js` staleness now on `clock.now()` (virtual, frozen while paused), marker dropped; `test_robot_models.html` call sites updated.

M2a (typed slots): `mcap-decode.js` `decodeSlot` returns `{type, n, t, cols, kinds}` (scalars Float64Array, bools Uint8Array, strings plain, arrays of objects CSR with dotted leaf names, arrays of primitives CSR with one `flat`, irregular as 'any' and cloned); `mcap-worker.js` transfers every typed buffer (deduped); `chunk.js` `makeTypedHydrator`/`isTyped`, `makeTopic`, `chunkFromRecord` adopts without copy.

M2b (readers): `chunk.js` gains `buildColumn(s)` (moved from `mcap-decode.js`, re-exported), `typedColumns(topic)` (memoised), `leafFiller`; `chart-store.js` `derive()` reads `motor_states` CSR leaves into preallocated per-axis Float64Arrays, `latestBefore` returns `{t, tp, k}`, `derivedFor` is the one per-chunk cache; `mode.js` coalesces `syncResident` to one `store.setResident` per animation frame (flushed at entry, cancelled at exit); `engine.js` `dataOf` reads the raw column (fixes a latent true/false vs 1/0 mismatch in the on-change gate); `digest.js` comments.

## Verification

- 2026-10-10, M1, `pytest tests/ros -k 'gui or replay' -q`: 442 passed, 1 skipped. Record path cost 0 (was 8.21 MB per 120 s); live idle heap 6.0 MB before and after. New `tests/ros/test_gui_replay_memory_contract.py` (no tap, no session symbol in js, cone uses clock) and lobby `test_no_session_button_or_row`.
- 2026-10-10, M2a, same scoped command: 448 passed, 1 skipped. Oracle: 4 slots, 446 741 values, 0 mismatches (hydrate every row, re-flatten by value; null == NaN/Inf). New `test_replay_typed_columns.py` (6 tests, including transfer detach and the real worker transferring every buffer).
- 2026-10-10, M2b, same scoped command: 451 passed, 1 skipped. New tests: typed derivation hydrates nothing and derives each chunk once; resident changes rebuild the chart store once per frame; `chunkFromRecord` adopts buffers without copying.
- Before (HEAD 65fc0161) to after, headless Chromium:

| Measure | Before | After |
|---|---|---|
| Bytes per slot, busy bag (decode + clone + `chunkFromRecord`) | 17.86 MB | 5.64 MB (2.43 heap + 3.21 typed buffers) |
| Load arrival on main thread, p50 / max | 240 / 251 ms (long tasks 126-174 ms per load) | 26 / 29 ms (no long tasks); `chunkFromRecord` <= 0.8 ms |
| Slot-arrival clone + GC self time | about 146 ms | about 22 ms per load |
| Replay heap, busy bag, 10 min ring filled | 69.9 MB | 18.0 MB |
| Five re-enter cycles | about 63.5 MB | 14.8-18.0 MB |
| Heap max while playing + digester filling | 262.7 MB | 140.7 MB |
| 7 min bag, 90 s seek storm: floor at 90 s | 52.9 MB (peaks 63.4) | 16.8 MB |
| Same, sampled max | 151.1 MB | 82.6 MB |
| Same, long-task total / worst stall | 93.7 s / 839 ms | 93.5 s / 887 ms |
| Busy-bag smoke heap at open / filled / after zoom | 47 / 48.8 / 60.5 MB | 12.1 / 13.8 / 16.9 MB |
| Page open | 6.7 s | 4.6 s |

Long-task totals did not move because headless software rendering puts 28-29 s of every 30 s in native `(program)`; our JS plus GC fell from 4.3 s to 1.0 s per 30 s on the 7 min bag. The largest remaining main-thread JS cost is NOT replay code: `getTrackingSparklineCtx` in `js/panels.js`, about 4.7-7.2 s per 30 s on the busy bag (follow-up).

- Full gate (`./run_tests.sh`, run 2026-10-10 in the replay worktree, after the audit fixes): parallel **6544 passed, 9 skipped in 285.12 s**; serial 3 passed in 10.78 s; PASS.

## Outcome

Resolved at software level: the replay working set fell about 3-4x (busy-bag ring-filled heap 69.9 to 18.0 MB; slot 17.9 to 5.6 MB), arrival work on the main thread about 10x, and the session buffer is gone. Unverified: the owner's real Win10 Chrome 154 run; worker and GPU memory (not measured); a literal 600 s ring on a long busy bag; the 3 GB crash was never reproduced headless, so it is not proven fixed. Follow-ups: the `getTrackingSparklineCtx` cost in `panels.js`; a worker-side digest if the main-thread digest shows up in the owner's trace.
