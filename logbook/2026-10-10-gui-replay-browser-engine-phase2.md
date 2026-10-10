---
title: GUI rosbag replay browser engine, Phase 2 - the live handlers driven from recorded chunks on a virtual clock
type: feature
date: 2026-10-10
status: resolved
phase: "gui-rosbag-replay - Phase 2 (browser engine)"
related_plan: gui-rosbag-replay.md
files_changed:
  - ros_ws/gui/js/replay/chunk.js
  - ros_ws/gui/js/replay/sources.js
  - ros_ws/gui/js/replay/policy.js
  - ros_ws/gui/js/replay/engine.js
  - ros_ws/gui/js/replay/fence.js
  - ros_ws/gui/js/replay/chart-store.js
  - ros_ws/gui/js/replay/cache.js
  - ros_ws/gui/js/replay/mode.js
  - ros_ws/gui/js/replay/wiring.js
  - ros_ws/gui/js/replay/session.js
  - ros_ws/gui/js/clock.js
  - ros_ws/gui/js/ros-bridge.js
  - ros_ws/gui/js/main.js
  - ros_ws/gui/lib/msgpack.min.js
  - ros_ws/gui/lib/VENDORED.md
  - ros_ws/gui/test_replay_engine.html
  - ros_ws/gui/replay/convert.py
  - ros_ws/gui/replay/cache.py
  - ros_ws/gui/replay/api.py
  - ros_ws/gui/replay/schema.py
  - tests/ros/test_gui_clock_contract.py
  - tests/ros/test_gui_replay_format_contract.py
  - tests/ros/test_gui_replay_chart.py
  - tests/ros/test_replay_api.py
  - tests/ros/test_replay_convert.py
  - logbook/2026-10-10-gui-replay-browser-engine-phase2.md
  - logbook/INDEX.md
subsystem:
  - gui
tags:
  - testing
  - performance
---

# GUI rosbag replay browser engine, Phase 2

## Summary

Phase 2 of `plans/active/gui-rosbag-replay.md` (Browser engine, adopted from wayfinder ticket 02's design proposal on 2026-10-10): the browser side that replays Phase 1's chunks through the GUI's own live handlers. Software complete; the owner's on-box session with the throwaway dev page is pending.

## Motivation

Phase 1 serves columnar chunks; nothing in the browser consumed them. The engine must make recorded data indistinguishable from live data to every panel, without a second handler table, without wall-clock reads that a pause or seek would break, and without any path to the robot.

## Design and Implementation

Six units, all in `ros_ws/gui/js/replay/` unless noted:

- **U1 feed.** `chunk.js` decodes msgpack, provides `indexLatestBefore`, and hydrates rows lazily (the inverse of `replay/schema.py`'s flattening; non-finite floats hydrate to `null`, the rule read from the installed rosbridge `message_conversion.py`). `sources.js`: `RecordingSource` over the Phase 1 API (1 s status polling, growing frontier) and `SessionBufferSource` (epoch-aligned 10 s chunks, 600 s ring, per-topic last-message sidecars). `@msgpack/msgpack` 2.8.0 vendored as `lib/msgpack.min.js` (31,572 bytes, sha256 in `lib/VENDORED.md`).
- **U2 clock.** `js/clock.js`: `now()` plus a virtual `setTimeout` queue advanced by playhead travel, frozen when paused, cleared on seek, scaled by speed. 24 FOLLOWS sites moved to `clock.now()`, 8 watchdog timers to `clock.setTimeout`; every remaining wall-clock read carries a `// wall-clock:` marker pinned by `tests/ros/test_gui_clock_contract.py`.
- **U3 engine.** `policy.js` (19-topic class/gate table pinned to `schema.SUBSCRIBED + PLANNED` and to `subscribeAll()`'s throttles); `engine.js` (merged `t`-order dispatch; rosbridge throttle emulated in playhead time scaled by speed; events never dropped; seek = reset latches, muted baseline, at most 30 s pre-roll, latest-before; reverse mutes events and ends with a seek; `buffering` at gaps and the frontier); `fence.js` plus early returns in commands/jog/bb-aim/panels; `ros-bridge.js` `dispatchLocal`, transport refusal in replay and `setBeforeConnectedHook`; `main.js` `resetForSeek()` / `blankDisconnectedState()`.
- **U4 charts.** `telemetrySample()` extracted from `onTelemetryData` (no live behaviour change); `chart-store.js` keeps immutable per-chunk derived columns rebuilt into fresh arrays on every residency change; span clamped to 120 s. 245 k derived values matched the live path NaN-aware.
- **U5 cache.** `cache.js`: resident set = window + 20 s ahead + 10 s behind in the play direction, at most 16 chunks, farthest-first eviction, strictly serial fetch, frontier-aware, `bufferedRanges()`.
- **U6 mode.** `mode.js` (LIVE, LOBBY, OPENING, REPLAY, LOBBY; ordered entry/exit; a reconnect exits BEFORE main's listener via the pre-notify hook), `wiring.js`, `session.js`, the session-buffer tap in `ros-bridge.js`, event-store snapshot/restore with `(t, type, label)` dedup, and the dev page `ros_ws/gui/test_replay_engine.html` (real GUI plus a transport strip; heap readout for the session-buffer measurement). CAN/UDP traffic rings gained `resetTrafficRing()` for seek/exit.

Phase 1 defects found by Phase 2: the converter wrote DiagnosticStatus IDL constants (OK/WARN/ERROR/STALE) as columns (rosbags gives them as dataclass fields WITH defaults; real fields have none), fixed in `convert.py`; the server never enforced the manifest `format`, so a complete cache with an older format is now listed `stale` and reconverted on open. `FORMAT_VERSION` is 2 and `chunk.js` `CHUNK_FORMAT` is pinned to it by `tests/ros/test_gui_replay_format_contract.py`.

## Discussion

Three non-obvious choices, each over a reasonable alternative. (1) Dispatch through the live handlers, not a replay-side handler table: a second table drifts from `subscribeAll()`, whereas the allowlist and policy tests pin a single set. (2) Throttle emulation in playhead time (scaled by speed) rather than latest-per-tick (frame-rate-dependent, non-deterministic tests) or every-Nth (drifts with rate jitter); handlers therefore never run faster than live. (3) An immutable chart buffer rebuilt per chunk boundary rather than the live shift-on-push ring, because the shifting ring is the root cause of the 2026-09-07 cursor-drift class, which cannot recur this way. The full alternatives are in the design proposal, `.scratch/gui-replay/issues/02-design-proposal.md` in the skills worktree (gitignored, not in the repo).

## Audit addendum

The end-of-phase `/audit` on the staged branch found no blocking issue; 6 WARNINGs and 5 NOTEs were applied before the commit. Replay entry rolls back cleanly if a step throws and exit is exception-safe (clock and fence cannot stay stuck in replay). A throwing virtual-timer callback or seek handler no longer wedges the engine (try/catch in `clock._travel` with a 10 000-timer fuse; `seekBusy` cleared in `finally`). Virtual timer ids are negative so they never cancel a real timer after exit. A unit toggle during replay no longer leaves the parked live chart ring in the wrong unit. Command buttons are disabled on entry even before any `orchestrator_state`, and a fenced click emits no COMMAND event. `robot-meshes.js` freshness is bypassed in replay (a paused replay no longer shows a stale robot). The `bbCalibrationInitialSkipped` latch is saved on entry and restored on exit.

Server side, the flaky `test_source_changed_discards_complete_cache` was a real product bug: a worker's manifest reaches `complete` before `proc.wait()` returns, so a re-open in that window served a stale cache. Staleness checks now run before the queued/running check and join the exiting worker (409 `busy` after 5 s); chunk, manifest and overview routes 404 on an old-format cache; the bulk CLI reconverts old-format caches; a test pins `_field_names` to the `.msg` definitions. Tests were added for each (fence, clock ids, fuse, throwing steps, scrub-supersedes-seek, unit toggle, stale worker).

## Verification

- Default gate on the branch: `./run_tests.sh`, run 2026-10-10 in the worktree (log `temp/logs/gate_replay_phase2_fixed_2026-10-10.log`): **parallel 6296 passed, 9 skipped, 1 failed in 258.20 s; serial 3 passed in 9.71 s; RESULT: FAIL, exit 1** — the one failure is `tests/sim/test_ball_butler_sim.py::TestConvenience::test_scatter_reproducible_with_seeded_rng`, a load flake outside this branch (passes 2/2 alone in the worktree and 1/1 in the live tree at `c4015417`; the branch touches nothing under `sim/`). The merged state is gated again before the push (see the merge commit's entry line below).
- Merged state: merge commit `6f834a14` brought in `skill-stack` `c4015417` (the peer's `/bb/calibration_attempt` subscription) and follow-up `ca4b6781` added its replay policy entry, re-keyed the overview `bb_calibration` tick to that per-sweep topic (`ok` / `failed` from `success`) and extended the fixture. Gate on the merged branch, `./run_tests.sh`, run 2026-10-10 (log `temp/logs/gate_replay_phase2_merged_2026-10-10.log`): **parallel 6331 passed, 9 skipped in 263.67 s; serial 3 passed in 9.64 s; RESULT: PASS, exit 0** — the pre-push run.
- Scoped, run 2026-10-10, after the audit fixes: `pytest tests/ros/ -k "gui or replay" -q -p no:cacheprovider` gave 309 passed, 1 skipped in 91.85 s; `pytest tests/ros/test_gui_replay_chart.py tests/ros/test_replay_api.py tests/ros/test_replay_convert.py tests/ros/test_replay_e2e.py tests/ros/test_replay_stdlib.py -q` gave 46 passed in 50.89 s.
- Known load flake seen in the pre-fix gate and NOT caused by this branch: `tests/ros/test_skill_node_resend_param.py::test_the_default_cap_reaches_the_executor` (`_executor` is None after `_start_pattern` under xdist load; passes 3/3 alone in the worktree and in the live tree; the branch touches nothing under `ros_ws/src`).
- `--full` was not run: nothing under `controller/` or `sim/` changed.

## Outcome

Replay engine, clock, charts, cache and mode switching are in place behind the dev page; no live-path behaviour change beyond the extracted `telemetrySample()`.

## Next

The owner opens `http://<jetson>:8081/test_replay_engine.html` with the ROS stack down after the merge (steps in the plan, Phase 2, and on the dev page). Phase 3 (the real UI) waits on the ticket 03 pick; Phase 4 on ticket 04.

Update 2026-10-10: Phase 3 retired the dev page (`test_replay_engine.html` deleted); the real UI is the path now (`2026-10-10-gui-replay-ui-phase3.md`).
