---
title: GUI rosbag replay — one playhead over any MCAP recording or the last live session
created: 2026-10-10
status: active
owner: Harrison
last_updated: 2026-10-10
related_plan: two-ball-skill-stack.md
related_code:
  - ros_ws/gui/replay/schema.py (the cache contract: layout, chunk record, manifest, overview, allowlist)
  - ros_ws/gui/replay/convert.py (the venv worker)
  - ros_ws/gui/replay/api.py + cache.py (stdlib routes under gui_server.py)
  - ros_ws/gui/gui_server.py
  - tools/systemd/jugglebot-gui.service
---

# GUI rosbag replay

The browser GUI (`ros_ws/gui/`) will replay any MCAP recording on the Jetson
disk, or the last live session still in memory, through **one playhead** that
drives the charts, the 3D scene and every panel whose topic was recorded, with
a trackbar, transport controls and hotkeys in place of the command buttons,
available only while no live ROS2 session is connected.

**Decision record.** The design was charted with `/wayfinder` on 2026-10-09;
the map and its tickets live in `.scratch/gui-replay/` (gitignored, local to
the `Jugglebot-skills` worktree). Ticket 00 holds the 21 charting decisions,
ticket 01 the decode / payload measurements, ticket 05 the backend decisions.
Ticket 02 (engine and feed contract) was resolved 2026-10-10 and is Phase 2
below. **Tickets 03 (replay UI prototype — mock built, owner's pick pending)
and 04 (3D trails prototype) are still open**: Phases 3–4 are outlines that
each ticket's resolution will turn into a detailed phase.

## Context

- The GUI is vanilla JS over rosbridge; every topic handler lives in
  `main.js subscribeAll()` and fans out by direct calls; charts stamp samples
  with `Date.now()` at arrival; 42 wall-clock call sites across 12 files.
- Recordings: `jugglebot_launch.py` records an explicit topic list as MCAP to
  `~/Desktop/rosbags/YYYY-MM-DD_HH-MM-SS/<id>_0.mcap` (one file, never
  split). 487 recordings, 29.5 GB, median 17 MB, p90 170 MB, largest indexed
  843 MB / 13.1 h; 3 unindexed (killed recordings, 2026-06-08).
- Measured 2026-10-09 (ticket 01): decoding dominates. `mcap_ros2` decodes the
  array-bearing GUI topics at ~1.0–1.3 k msg/s (a 10 s full-rate window in
  3.9 s, 60 s in 29 s); the `rosbags` typestore is 6–7× faster. The reader
  floor is ~8 ms per recorded second. A 60 s window held as Python objects
  peaks at ~2 GB RSS on a box with ~2.3 GB free. Columnar msgpack+gzip is
  52–63 KB per recorded second.
- The GUI server (`gui_server.py`, 59 lines) is a single-threaded stdlib
  `HTTPServer` run by a systemd unit under `/usr/bin/python3`, which has
  neither `mcap` nor `rosbags`; the unit chose stdlib-only so boot never
  depends on the venv. The unit file was not in the repo before this plan.

## Architecture

Backend decisions (ticket 05, 2026-10-10; round 1 owner-confirmed, round 2
delegated to Claude's judgment):

1. **Decoder: `rosbags` typestore** (`get_typestore(Stores.ROS2_FOXY)`, types
   registered from the embedded `ros2msg` schemas, `deserialize_cdr`). The
   reader stays `mcap` (`SeekingReader` for indexed bags, `NonSeekingReader`
   for unindexed). `tools/probes/` keeps `mcap_ros2`; no shared code.
2. **Convert once, serve files.** First open streams the bag once, in the
   background, into `temp/replay_cache/<id>/`: `manifest.json`,
   `overview.json` and fixed **10 s** chunks `chunk-NNNNN.msgpack.gz`, every
   allow-listed topic columnar (flattened field paths) in one file, stored
   gzipped and served with `Content-Encoding: gzip`, so a seek costs zero
   decode CPU. The converter holds one chunk plus a one-chunk reorder buffer,
   so memory is bounded by a chunk, not the window. `format` in the manifest
   invalidates every cache when the chunk schema changes (enforced at `open`: a complete cache with an older format is discarded and reconverted; listed as `stale`).
3. **Allowlist = the GUI's subscribe set** plus the two topics the plan adds
   (`/balls`, `/cone/catch_event`), pinned by a contract test against the JS
   source. Other recorded topics are counted in the manifest, not converted.
4. **Backend inside `gui_server.py`, still stdlib**: `ThreadingHTTPServer`;
   the `/api/replay/` routes are file reads; the conversion is a subprocess
   under the venv interpreter (`--worker-python`, nice 10, one at a time).
   Venv missing → the lobby shows "replay backend unavailable"; the static
   GUI is unaffected. The unit file lands in `tools/systemd/`.
5. **Open is progressive**: replay opens on the first chunk; a status endpoint
   reports progress; the converted span fills in under the trackbar. Convert
   on open only, plus a bulk CLI run by hand; no automatic sweep (the server
   is always up and a sweep would compete with a live session). A partial
   cache (worker killed) is discarded and rebuilt, never resumed.
6. **Eviction**: LRU by last-open, 10 GB cap, and refuse to start a conversion
   below 5 GB free (the guard protects the recorder; the cap is incidental).
7. **Session buffer = the same chunk shape**, built in the browser at arrival
   (ring of 60 × 10 s): one feed implementation for both sources (Phase 2).
8. **Served rate**: chunks carry the full recorded rate. The browser never
   calls handlers faster than the live rate, whatever the speed; charts ingest
   columns, not messages. The decimation rule above 1× is ticket 02's.
9. **Unindexed bags** convert uniformly through `NonSeekingReader` (duration
   unknown until the pass ends). A footer-less file still growing is a
   *recording in progress* and is refused until its mtime is ≥ 10 s old.
10. **No duration cap**; a zoomable trackbar is a ticket 03 item.

**The contract has one enforcement point:** `ros_ws/gui/replay/schema.py`
defines the layout, the chunk record, the flattening rule, the manifest, the
overview and the allowlist; the worker writes it, the server serves it, the
browser (Phase 2) unflattens by it. Changing the shape bumps `FORMAT_VERSION`.

### HTTP API (served by `gui_server.py`, all `Cache-Control: no-cache`)

| route | returns |
|---|---|
| `GET /api/replay/recordings` | `{"recordings": [...newest first], "cache": {"bytes", "cap_bytes", "disk_free_bytes"}, "worker_available": bool}`; each recording: `id`, `path`, `size_bytes`, `mtime`, `closed` (MCAP footer magic present), `in_progress`, `duration_s` / `message_count` (from `metadata.yaml` when present, regex-parsed, else `null`), `cache` (`none`/`converting`/`complete`/`failed`) |
| `POST /api/replay/recordings/<id>/open` | `200 {"status": "complete"|"converting"|"queued"}` or `409 {"status": "refused", "reason": "recording_in_progress"|"disk_low"|"worker_unavailable"}`; touches `.opened`, evicts LRU, discards a stale partial cache with no live worker, starts or queues the worker; `404` unknown id |
| `GET /api/replay/recordings/<id>/status` | `{"status": "none"|"queued"|"converting"|"complete"|"failed", "chunks_done", "chunks_total", "t0", "t1", "error"}` |
| `GET /api/replay/recordings/<id>/manifest` | `manifest.json` (`404` if none) |
| `GET /api/replay/recordings/<id>/overview` | `overview.json` (`404` until complete) |
| `GET /api/replay/recordings/<id>/chunks/<i>` | the stored bytes, `Content-Type: application/msgpack`, `Content-Encoding: gzip`; `404` if `i >= chunks_done` |

Recordings are the top-level `YYYY-MM-DD_HH-MM-SS` directories of the
recordings root holding exactly one `.mcap`; the nested
`Old naming scheme/` folder is out of scope.

## Vocabulary

- *Live mode* — rosbridge connected, data streams in. *Replay mode* — the
  exclusive in-page state driven by the playhead; reachable only while
  rosbridge is disconnected. *Lobby* — the disconnected command overlay
  offering the two replay entries.
- *Playhead* — the single replay time, wall-clock seconds.
- *Recording* — one bag directory. *Session buffer* — the chunks retained in
  the browser during live mode (600 s). *Source* — a recording or the session
  buffer; *feed* — the time-ordered records a source yields.
- *Chunk* — a fixed 10 s slice of a source's feed, every allow-listed topic
  columnar in one record; the unit of fetch, of the cache and of the session
  buffer. *Window* — a contiguous run of chunks around the playhead.
- *Cache* — a recording's converted form under `temp/replay_cache/<id>/`.
  *Conversion* — the one-off backend pass, run by the *worker* (a venv
  subprocess of the GUI server). *Allowlist* — the topics the backend
  converts.
- *Timeline overview* — per-recording bands, ticks and presence drawn under
  the trackbar. *Trail* — the `tail_length_ms` of history behind a ball or
  marker.

## Implementation Phase Summary

| Phase | Scope | Status |
|---|---|---|
| 1a | Converter worker (`replay/convert.py`): rosbags decode, chunking with reorder buffer, manifest, overview, bulk CLI; synthetic MCAP fixture; tests | done 2026-10-10 (software) |
| 1b | Server (`replay/cache.py`, `replay/api.py`, `gui_server.py`): threaded, routes, worker queue, eviction, refusals; unit file in `tools/systemd/`; tests | done 2026-10-10 (software; unit not yet installed on the box) |
| 1c | End-to-end test (real worker through the API on the fixture); bulk conversion of the newest recordings by hand; unit installed on the box | done 2026-10-10: e2e test; unit installed and the service restarted by the owner; live open path smoke-tested (24 MB / 114 s recording queued → complete in 18 s, 12 chunks, 3.2 MB, chunk served gzip). Bulk pre-conversion remains optional |
| 2 | Browser engine: feed over chunks (recording via the API, session buffer in memory), playhead clock, dispatch through the live handlers, seek semantics, prefetch, chart-store swap | decided 2026-10-10 (ticket 02); building in worktree `gui-replay-phase2` |
| 3 | Replay UI: lobby, picker, trackbar + transport + hotkeys, timeline overview, zoom; replay-only-while-disconnected gating | **pending ticket 03** |
| 4 | Balls in the 3D scene with trails (live and replay), `tail_length_ms` input | **pending ticket 04** |
| 5 | Hardening and fog: session-buffer memory ceiling, deep links `?recording=<id>&t=`, read-only minimap / juggle panel in replay | after 2–4 |

## Phase 1 — Backend

### 1a Converter worker — `ros_ws/gui/replay/convert.py`

- CLI: `python -m replay.convert --bag <file.mcap> --out <cache_dir>
  [--chunk-s 10]` (run with `cwd=ros_ws/gui`), and `--bulk <recordings_root>
  --cache-root <dir> [--newest N]` for the by-hand sweep. Exit 0 on complete,
  1 on failure (manifest `status: failed`, `error` set).
- Reader: `mcap` `make_reader`; `SeekingReader` when the file ends with the
  MCAP footer magic, `NonSeekingReader` otherwise (`chunks_total` unknown
  until the end). Types: the `rosbags` typestore, registered once from the
  embedded `ros2msg` schemas (the concatenated rosbag2 format with
  `MSG: pkg/msg/Name` separators). Decode only allow-listed channels; count
  the others into `skipped_topics`.
- Chunking: `t0` = first message's log time; chunk `i` covers
  `[t0 + 10 i, t0 + 10 (i + 1))`. Two chunks stay open (reorder buffer);
  chunk `i` is sealed when a message with `t >= t0 + 10 (i + 2)` arrives, or at
  the end. A message for an already-sealed chunk increments `dropped_late`.
  Every index gets a file, empty ones included. Columns per
  `schema.py`'s flattening rule; `msgpack.packb` + `gzip` level 6; written
  to a temp name and renamed into place.
- Manifest: written at start (status converting), rewritten at most once per
  second while converting (`chunks_done`, running `topics` counts), finalised
  at completion (`t1`, `chunks` index, `completed_at`, status complete).
  Atomic rename on every write.
- Overview at completion: `/orchestrator_state` `.data` value segments as the
  band; ticks per `OVERVIEW_TICK_KINDS`; presence = merged runs of chunks
  where a topic has `n >= 1`.
- Memory: never more than the two open chunks decoded; a 60 s window is
  never materialised.

### 1b Server — `replay/cache.py`, `replay/api.py`, `gui_server.py`

- `cache.py` (stdlib): list recordings (dir scan, `closed` by footer magic,
  `in_progress` = not closed and mtime < 10 s old, `metadata.yaml` regex
  parse for duration / message count), read manifests, cache size, LRU
  eviction (`.opened` mtime, else `completed_at`), disk guard
  (`shutil.disk_usage`), stale-partial detection.
- `api.py` (stdlib): route table above; a worker queue thread running one
  `subprocess.Popen(["nice", "-n", "10", worker_python, "-m",
  "replay.convert", ...], cwd=gui_dir)` at a time (the `nice` prefix only
  where the binary exists; `preexec_fn` is unsafe in a threaded server);
  `worker_available` probed at start (`worker_python -c "import mcap,
  rosbags, msgpack"`, 30 s timeout) and re-probed lazily on `open` so a slow
  cold boot cannot disable replay for the server's lifetime. `open` is
  serialised by a lock so two concurrent opens cannot discard a running
  worker's cache.
- `gui_server.py`: `ThreadingHTTPServer`; new args `--rosbags-dir`
  (default `~/Desktop/rosbags`), `--cache-dir` (default
  `<repo>/temp/replay_cache`), `--worker-python` (default
  `~/Desktop/PDJ_venv/venv/bin/python`), `--cache-cap-gb 10`,
  `--min-free-gb 5`; `/api/replay/` delegated to `api.py`; static serving
  and CORS unchanged.
- `tools/systemd/jugglebot-gui.service` lands in the repo with the explicit
  arguments; `tools/systemd/README.md` gains the install / restart lines
  (`sudo cp`, `daemon-reload`, `restart`). Installing is an owner action: the
  running service keeps the old code until restarted.

### 1c Integration

- `tests/ros/test_replay_e2e.py`: the API with `sys.executable` as the
  worker converts the synthetic fixture end to end; status progresses;
  chunks decode in Python to the written values.
- Bulk-convert the newest recordings by hand; record the measured
  conversion time per MB in the logbook entry.

## Phase 2 — Browser engine

Settled by wayfinder ticket 02 (`.scratch/gui-replay/issues/02-design-proposal.md`). Six units, in order U1 → U2 → U3
∥ U4 → U5 → U6; U3 is the design-bearing unit.

- **Feed.** A `Source` (recording or session buffer) exposes sync `range()`, `chunkIndex()`, `status()` and async
  `load(i)`, `window(t0, t1)`, `latestBefore(t, topic)`, `timeline()`. The engine reads resident chunks synchronously
  inside a tick and never awaits mid-dispatch. A decoded chunk keeps per-topic `t` (Float64Array) and the schema
  columns; rows are hydrated lazily, one per dispatched record, by the inverse of the `schema.py` flattening rule;
  non-finite floats follow the representation live rosbridge delivers (probed in U1). The recording source drives the
  Phase 1 API (open, manifest, 1 s status polling while converting, chunk fetch, msgpack decode). The session buffer
  is built in the browser from the subscription wrapper in `ros-bridge.js`: epoch-aligned 10 s chunks over the last
  600 s plus a per-topic last-message sidecar; its ceiling (Phase 5) shortens the horizon, never drops topics.
- **Clock.** `js/clock.js` provides `now()`, `setTimeout`/`clearTimeout` and `isReplay()`. In replay `now()` returns
  the dispatched record's `t` during its handlers and the playhead otherwise; timers run on a virtual queue driven by
  playhead travel, frozen while paused, cleared on seek and scaled by the speed. Every remaining
  `Date.now()`/`performance.now()`/`new Date()` read carries a `// wall-clock:` marker, pinned by a contract test.
- **Dispatch.** Replay calls the handlers registered through `ros.subscribe()` via `ros.dispatchLocal()`, merged
  across topics in time order. Publishing and service calls are refused by `ros-bridge.js` while replay is active, and
  every command affordance is disabled under `body.replay`. The reconnect loop keeps running;
  `setConnectionState('connected')` exits replay before any listener is notified.
- **Seek.** A seek resets every handler latch (`main.js resetForSeek()`), the CAN/UDP rings and the replay event
  store, dispatches a muted latest-before baseline at `p − W`, pre-rolls `(p − W, p]` at 1× rates for edge-, history-
  and event-bearing topics, then latest-before at `p` for the rest (`W` = visible span, at most 30 s). A drag-scrub
  skips the pre-roll until release. Pausing settles every state topic to its latest record before the playhead.
- **Rates.** Each topic is dispatched no faster than its live rate: rosbridge-throttle emulation in playhead time with
  the gate scaled by the speed; event topics never drop a record; `orchestrator_state` and `control_mode_topic` also
  pass on change. Reverse playback dispatches state topics only, mutes event emission and ends with a seek.
- **Charts.** Replay swaps in an immutable `ReplayChartStore` built from per-chunk derived columns through the same
  `telemetrySample()` the live path uses; the live ring is restored on exit. The buffer is rebuilt on every swap or
  chunk append, never shifted in place; the span is clamped to 120 s.
- **Cache.** Resident chunks cover the chart window plus 20 s ahead and 10 s behind in the play direction, at most 16,
  evicted farthest-first, fetched serially. A missing chunk or the converted frontier puts the engine in `buffering`
  with the playhead held.
- **Mode.** LIVE → LOBBY → OPENING → REPLAY{paused, playing, buffering} → LOBBY; a reconnect exits to LIVE with a
  toast. Entry and exit are single ordered functions; exit ends with the live disconnect blanking.
- **Decoder.** `@msgpack/msgpack` 2.8.0, vendored under `ros_ws/gui/lib/` with its size and hash.
- **Tests.** Node harnesses under `tests/ros/js/` with pytest wrappers per unit (feed round-trip against a
  Python-written chunk, clock contract, engine rate/order/seek/fence properties, chart derivation parity, cache
  residency, mode transitions). DOM wiring is exercised by hand through `ros_ws/gui/test_replay_engine.html` until
  Phase 3 replaces it.

Owner-level choices settled by delegation (2026-10-10): the replay chart span
is clamped to 120 s; a seek pre-rolls 30 s of events rather than building a
per-recording event index; above 1× the stale indicators flag only gaps of at
least speed × timeout; "Replay last session" opens paused at the end minus one
span; the session-buffer horizon stays 600 s by default (`SESSION_BUFFER_SEC`),
with U6 measuring the heap on the box so the owner can cap it at 300 s if the
browser runs on the Jetson.

## Phases 3–5 (outline, pending tickets)

- **Phase 3 UI** (ticket 03): lobby in `#command-overlay`, picker modal
  (date, duration, size, cache state, "converting n %", "duration unknown",
  "recording in progress"), trackbar with date header and HH:MM:SS readout,
  speed ladder, step / FF / RW, hotkeys, overview bands and ticks, zoom.
- **Phase 4 balls and trails** (ticket 04): `/balls` in the 3D scene, trails
  behind balls and markers, `tail_length_ms`.
- **Phase 5**: the fog items on the map.

## Testing Plan

- Phase 1 tests run under the venv in the default gate (`./run_tests.sh`):
  `tests/ros/test_replay_convert.py`, `test_replay_api.py`,
  `test_replay_allowlist.py`, `test_replay_e2e.py`. The fixture
  (`tests/ros/_replay_fixture.py`) writes a small synthetic MCAP from the
  repo's `.msg` definitions with `rosbags` + the `mcap` writer, embedding the
  schemas in rosbag2's format, with an "unindexed" variant (footer removed).
- Contract test: the allowlist equals the JS subscribe set plus `PLANNED`.
- Server tests bind ephemeral ports and use `tmp_path`; the worker in tests is
  `sys.executable` or a stub script; no test touches `~/Desktop/rosbags`.
- Phase 2–4 tests follow the node-harness standard (`tests/ros/js/`).

## Notes for Collaborators

- The map (`.scratch/gui-replay/map.md`) is the decision index; it is
  gitignored, so this plan carries the decisions that matter to the build.
  Continue the map with `/wayfinder .scratch/gui-replay/map.md`, one ticket
  per session.
- Never restart `jugglebot-gui.service` while another session is using the
  GUI; the new server code is inert until the restart.
- `temp/replay_cache/` is rebuildable; deleting it costs one conversion per
  recording opened afterwards.
