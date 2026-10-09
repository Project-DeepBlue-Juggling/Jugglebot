---
title: GUI rosbag replay backend, Phase 1 - MCAP converted once to columnar chunks and served by the stdlib GUI server
type: feature
date: 2026-10-10
status: resolved
phase: "gui-rosbag-replay - Phase 1 (backend)"
related_plan: gui-rosbag-replay.md
files_changed:
  - plans/active/gui-rosbag-replay.md
  - plans/active/INDEX.md
  - ros_ws/gui/gui_server.py
  - ros_ws/gui/replay/__init__.py
  - ros_ws/gui/replay/schema.py
  - ros_ws/gui/replay/convert.py
  - ros_ws/gui/replay/cache.py
  - ros_ws/gui/replay/api.py
  - tools/systemd/jugglebot-gui.service
  - tools/systemd/README.md
  - tests/ros/_replay_fixture.py
  - tests/ros/test_replay_convert.py
  - tests/ros/test_replay_allowlist.py
  - tests/ros/test_replay_api.py
  - tests/ros/test_replay_e2e.py
  - tests/ros/test_replay_stdlib.py
  - logbook/2026-10-10-gui-replay-backend-phase1.md
  - logbook/INDEX.md
subsystem:
  - gui
  - tools
tags:
  - testing
  - performance
---

# GUI rosbag replay backend, Phase 1

## Summary

Phase 1 of `plans/active/gui-rosbag-replay.md` (new plan, same commit): the backend that lets the browser GUI replay MCAP recordings. Software complete; installing the systemd unit and restarting the service is an owner step (the running service still serves the old code).

## Motivation

Decided in the wayfinder map's ticket 05 (`.scratch/gui-replay/`, gitignored) and recorded in the plan, Architecture. Decoding dominated (ticket 01: `mcap_ros2` needs 3.9 s for a 10 s full-rate window and 29 s for 60 s; `rosbags` is 6-7x faster), so the design converts once and serves files instead of decoding per seek. The server stays stdlib so boot never depends on the venv and a decode crash cannot take the static GUI down. Round 1 was owner-confirmed; round 2 was delegated to Claude's judgment by the owner.

## Implementation

- `ros_ws/gui/replay/schema.py` is the cache contract (one enforcement point): `temp/replay_cache/<id>/` holds `manifest.json`, `overview.json` and fixed 10 s `chunk-NNNNN.msgpack.gz` chunks (every allow-listed topic columnar, flattened field paths, gzip-stored so a seek is a file read). Allowlist = the GUI's live subscribe set (17 topics) plus `/balls` and `/cone/catch_event`.
- `ros_ws/gui/replay/convert.py` is the worker (venv: `mcap` reader, `rosbags` typestore decoder, `msgpack`): `python -m replay.convert --bag/--out` and `--bulk`. Two-chunk reorder buffer, manifest rewritten at most 1/s, overview (orchestrator band; fault/homed/levelled/catch/skill/calibration ticks; presence) built in the same pass; unindexed (killed) recordings are read in file order.
- `cache.py`, `api.py` and `gui_server.py` are stdlib only (the systemd unit runs the server under `/usr/bin/python3`). The server is now a `ThreadingHTTPServer` with `GET /api/replay/recordings`, `POST /api/replay/recordings/<id>/open` and `GET .../status|manifest|overview|chunks/<i>`. One niced worker subprocess at a time under `--worker-python`; progressive open; LRU eviction at 10 GB with a 5 GB free-disk guard; a footer-less file younger than 10 s is refused as a recording in progress.
- `tools/systemd/jugglebot-gui.service` lands in the repo (it existed only on the box) with explicit arguments; `tools/systemd/README.md` has the install lines. Not installed or restarted.
- Tests: `test_replay_convert.py` (11), `test_replay_allowlist.py` (2, pins the allowlist to the JS source), `test_replay_api.py` (19, real server on an ephemeral port with a stub worker), `test_replay_e2e.py` (1, real worker through the API), `test_replay_stdlib.py` (2, pins the stdlib-only boundary of the server half: an isolated-interpreter import plus an AST scan of top-level imports). `_replay_fixture.py` writes a synthetic rosbag2-style MCAP from the repo's `.msg` files (embedded `ros2msg` schemas, unindexed and late-message variants).

One correction during the build: the server agent implemented the per-recording routes without the `recordings/` segment (the brief had abbreviated the plan's table). Fixed to the plan's shape before the gate; the plan is normative and Phase 2 builds against it. The end-of-phase audit (`/audit`, staged diff) found no blocking issue; six low-risk findings were applied before the commit: `open` is serialised by a lock (two concurrent opens could `rmtree` a running worker's cache), the worker probe's 3 s boot timeout became 30 s plus a lazy re-probe on `open` (a slow cold boot would otherwise have disabled replay for the server's lifetime), the chunk index is parsed with `[0-9]+` (a non-ASCII digit crashed the request instead of returning 400), `preexec_fn` gave way to a `nice -n 10` prefix (unsafe in a threaded server; only where the binary exists), a `failed` manifest keeps `chunks: []` as the contract says, and the stdlib-boundary test above was added.

## Verification

- Default gate on the committed tree: `./run_tests.sh`, run 2026-10-10 (log `temp/logs/gate_replay_phase1_fixed_2026-10-10.log`): **parallel 6201 passed, 9 skipped in 252.75 s; serial 3 passed in 9.85 s; total 269 s; RESULT: PASS, exit 0**. An earlier run on the pre-audit tree (same day, `temp/logs/gate_replay_phase1_2026-10-10.log`) was also PASS: 6195 passed, 9 skipped in 298 s; serial 3 passed.
- Scoped, after the audit fixes, run 2026-10-10: `pytest tests/ros/test_replay_api.py tests/ros/test_replay_e2e.py tests/ros/test_replay_convert.py tests/ros/test_replay_stdlib.py -q -p no:cacheprovider` gave 33 passed in 37.62 s; `pytest tests/ros/test_replay_allowlist.py -q` is 2 passed (inside the gate run below).
- Conversion measured 2026-10-10 on the newest real bag (50 MB, 149 s) with another session's full suite running: 35.1 s wall, 15 chunks, 7.9 MB cache, `dropped_late` 0 (roughly 4x real time under load).
- `--full` was not run: no file under `controller/` or `sim/` changed, and the nightly tier does not cover `ros_ws/gui/`.

## Outcome

**On-box validation (2026-10-10, after the owner installed the unit and restarted the service):** `GET /api/replay/recordings` on the live :8081 reported `worker_available: true` and 468 recordings; `POST .../2026-10-10_00-24-06/open` (24 MB, 114 s) went queued → complete in 18 s (12 chunks, 3.2 MB under `temp/replay_cache/`), and `chunks/0` came back `application/msgpack` with `Content-Encoding: gzip`. Phase 1 is therefore complete on the box, not only in software.

Backend complete and tested; not yet live. Next: the owner installs the unit and restarts the GUI service when nobody is using the GUI; Phases 2-4 wait on wayfinder tickets 02 (engine), 03 (UI), 04 (trails).
