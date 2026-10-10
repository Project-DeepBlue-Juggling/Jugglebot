---
title: GUI rosbag replay — one playhead over any MCAP recording or the last live session
created: 2026-10-10
status: active
owner: Harrison
last_updated: 2026-10-10
related_plan: two-ball-skill-stack.md
related_code:
  - ros_ws/gui/replay/schema.py (the contract: chunk record, flatten rule, slot math, overview, allowlist)
  - ros_ws/gui/replay/{recordings,api}.py (stdlib routes under gui_server.py), decode.py + overview.py (venv overview pass), convert.py (test oracle)
  - ros_ws/gui/js/replay/{mcap-decode,mcap-worker,slot,allowlist}.js + sources.js McapSource
  - tools/gui_vendor/ (vendored MCAP bundle build)
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
below. Ticket 03 (replay UI) was resolved 2026-10-10 and is Phase 3 below;
Ticket 06 (direct MCAP) is Phase 4 below; **ticket 04 (3D trails prototype) is
still open**: Phase 5 is an outline that its resolution will turn into a detailed phase.

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

Backend decisions (ticket 05, 2026-10-10; items 2, 4, 5, 6 and 9 revised by ticket 06 / Phase 4):

1. **Decoder: `rosbags` typestore** (`get_typestore(Stores.ROS2_FOXY)`, types registered from the embedded `ros2msg`
   schemas, `deserialize_cdr`), kept for the oracle (`convert.py`) and the overview pass. The reader stays `mcap`
   (`SeekingReader` for indexed bags). `tools/probes/` keeps `mcap_ros2`; no shared code.
2. **Direct MCAP, decoded in the browser worker.** The server hands out the `.mcap` bytes over HTTP Range; a module Web
   Worker (vendored `@mcap/core` + rosmsg2 bundle) decodes fixed **10 s** slots on demand, every allow-listed topic
   columnar (flattened field paths) in one record. No converted cache exists. Reversed from ticket 05's convert-once
   cache because the only wait that scaled with recording length was the conversion (about 4x real time on the Jetson).
3. **Allowlist = the GUI's subscribe set** plus the two topics the plan adds (`/balls`, `/cone/catch_event`), pinned by a
   contract test against the JS source. Other recorded topics are counted, not decoded.
4. **Backend inside `gui_server.py`, still stdlib**: `ThreadingHTTPServer`; the `/api/replay/` routes are listing and
   file reads; the venv subprocess (`--worker-python`, nice 10, one at a time) runs the **overview pass only**. Venv
   missing → the overview is `unavailable`, replay itself still works. The unit file lives in `tools/systemd/`.
5. **Open holds until slot 0 is resident**, then REPLAY paused. The overview pass runs on the first GET of a
   recording's overview, never as a sweep (the server is always up and a sweep would compete with a live session).
6. *(Eviction deleted: there is no cache.)*
7. **Session buffer = the same chunk shape**, built in the browser at arrival (ring of 60 × 10 s): one feed
   implementation for both sources (Phase 2).
8. **Served rate**: slots carry the full recorded rate. The browser never calls handlers faster than the live rate,
   whatever the speed; charts ingest columns, not messages. The decimation rule above 1× is ticket 02's.
9. **Unindexed bags are refused `no_index`** (no summary section to seek by); a footer-less file still growing is
   `recording_in_progress`.
10. **No duration cap**; a zoomable trackbar is a ticket 03 item.

**The contract has one enforcement point:** `ros_ws/gui/replay/schema.py`
defines the chunk record, the flattening rule, slot math, the overview and the
allowlist; the oracle and the overview pass write it, the server serves it, the
browser worker builds slots by it. Changing the shape bumps `FORMAT_VERSION`.

### HTTP API (served by `gui_server.py`)

| route | returns |
|---|---|
| `GET /api/replay/recordings` | `{"recordings": [...newest first], "overview_available": bool}`; row: `id, size_bytes, mtime, closed, indexed, in_progress, duration_s, message_count, start_ns, topics{counts}, overview` (`overview` = `ready|none|queued|computing|failed`) |
| `GET\|HEAD /api/replay/recordings/<id>/file` | the `.mcap` bytes; Range → `206`, unsatisfiable/multi-range `416`, `If-Match` mismatch `412`, `409 recording_in_progress`\|`no_index` (also `X-Replay-Reason`), `404` unknown id, `400 bad recording id`; `Accept-Ranges`, ETag, `no-store` |
| `GET /api/replay/recordings/<id>/overview` | `200` overview JSON, `202 {queued\|computing}`, `409` refused, `503 worker_unavailable`, `500 failed` |

Recordings are the top-level `YYYY-MM-DD_HH-MM-SS` directories of the recordings root holding exactly one `.mcap`; the
nested `Old naming scheme/` folder is out of scope.

## Vocabulary

- *Live mode* — rosbridge connected, data streams in. *Replay mode* — the
  exclusive in-page state driven by the playhead; reachable only while
  rosbridge is disconnected. *Lobby* — the disconnected command overlay
  offering the two replay entries.
- *Playhead* — the single replay time, wall-clock seconds.
- *Recording* — one bag directory. *Session buffer* — the chunks retained in
  the browser during live mode (600 s). *Source* — a recording or the session
  buffer; *feed* — the time-ordered records a source yields.
- *Chunk* — a fixed 10 s slot of a source's feed built by the worker, every allow-listed topic
  columnar in one record; the unit of decode, residency and of the session
  buffer. *Window* — a contiguous run of chunks around the playhead.
- *Overview pass* — the one-off venv pass over a recording that yields its
  timeline overview, cached under `temp/replay_overview/<id>.json`. *Allowlist* —
  the topics the decoder keeps.
- *Timeline overview* — per-recording bands, ticks and presence drawn under
  the trackbar. *Trail* — the `tail_length_ms` of history behind a ball or
  marker.

## Implementation Phase Summary

| Phase | Scope | Status |
|---|---|---|
| 1a | (superseded by 4) Converter worker (`replay/convert.py`): rosbags decode, chunking with reorder buffer, manifest, overview, bulk CLI; synthetic MCAP fixture; tests | done 2026-10-10 (software) |
| 1b | Server (`replay/cache.py`, `replay/api.py`, `gui_server.py`): threaded, routes, worker queue, eviction, refusals; unit file in `tools/systemd/`; tests | done 2026-10-10 (software; unit not yet installed on the box) |
| 1c | End-to-end test (real worker through the API on the fixture); bulk conversion of the newest recordings by hand; unit installed on the box | done 2026-10-10: e2e test; unit installed and the service restarted by the owner; live open path smoke-tested (24 MB / 114 s recording queued → complete in 18 s, 12 chunks, 3.2 MB, chunk served gzip). Bulk pre-conversion remains optional |
| 2 | Browser engine: feed over chunks (recording via the API, session buffer in memory), playhead clock, dispatch through the live handlers, seek semantics, prefetch, chart-store swap | software-complete 2026-10-10 (`81355621`…`ca4b6781`, merged to `skill-stack`); the real GUI calls `getReplayMode` only in Phase 3; the dev page was retired in Phase 3 |
| 3 | Replay UI: lobby, picker, trackbar + transport + hotkeys, timeline overview, zoom; replay-only-while-disconnected gating | done 2026-10-10 (`2026-10-10-gui-replay-ui-phase3.md`); owner browser session held 2026-10-10, feedback in `2026-10-10-gui-replay-phase3-owner-feedback.md` |
| 4 | Direct MCAP source: vendored @mcap/core + rosmsg2 bundle, decode in a module Web Worker over HTTP Range, McapSource behind the Source seam, hold-until-slot-0 open; backend = listing + Range + cached overview pass; converter cache deleted, convert.py kept as the test oracle | software-complete 2026-10-10 (`2026-10-10-gui-replay-direct-mcap-phase4.md`); gate pending; owner browser unverified |
| 5 | Balls in the 3D scene with trails (live and replay), `tail_length_ms` input | **pending the owner's pick on ticket 04** (prototype `ros_ws/gui/test_replay_trails.html` built 2026-10-10, committed on skill-stack) |
| 6 | Hardening and fog: session-buffer memory ceiling, deep links `?recording=<id>&t=`, read-only minimap / juggle panel in replay | after 2–5; session-buffer ceiling: owner measured ≈ 1 MB heap per recorded second resident on 2026-10-10 (600 s ≈ 600 MB) → decide 300 s / topic cut |

## Phase 1 — Backend (historical)

Built 2026-10-10 as a convert-once cache; **superseded by Phase 4**, which deleted `replay/cache.py`, the
open/status/manifest/chunks routes, the cache flags (`--cache-dir`, `--cache-cap-gb`, `--min-free-gb`) and the msgpack
wire format. What survives: `schema.py` (layout, chunk record, flattening rule, allowlist, overview shape,
`FORMAT_VERSION`), `convert.py` as the test oracle (`--bulk` removed), the threaded stdlib server, the venv worker
(now the overview pass only), and `tools/systemd/jugglebot-gui.service` (ExecStart without the cache flags). The
original narrative is in the Phase 1 logbook entries.

## Phase 2 — Browser engine

Settled by wayfinder ticket 02 (`.scratch/gui-replay/issues/02-design-proposal.md`). Six units, in order U1 → U2 → U3
∥ U4 → U5 → U6; U3 is the design-bearing unit.

- **Feed.** A `Source` (recording or session buffer) exposes sync `range()`, `chunkIndex()`, `status()` and async
  `load(i)`, `window(t0, t1)`, `latestBefore(t, topic)`, `timeline()`. The engine reads resident chunks synchronously
  inside a tick and never awaits mid-dispatch. A decoded chunk keeps per-topic `t` (Float64Array) and the schema
  columns; rows are hydrated lazily, one per dispatched record, by the inverse of the `schema.py` flattening rule;
  non-finite floats follow the representation live rosbridge delivers (probed in U1). The recording source (Phase 4: `McapSource`) reads the
  `.mcap` over HTTP Range and decodes in a worker. The session buffer
  is built in the browser from the subscription wrapper in `ros-bridge.js`: epoch-aligned 10 s chunks over the last
  600 s plus a per-topic last-message sidecar; its ceiling (Phase 6) shortens the horizon, never drops topics.
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
  chunk append, never shifted in place; the span is clamped to 120 s (raised to 600 by the two-tier store, see Phase 4).
- **Residency.** Resident chunks cover the chart window plus 20 s ahead and 10 s behind in the play direction, at most 16,
  evicted farthest-first, fetched serially. A missing chunk puts the engine in `buffering`
  with the playhead held.
- **Mode.** LIVE → LOBBY → OPENING → REPLAY{paused, playing, buffering} → LOBBY; a reconnect exits to LIVE with a
  toast. Entry and exit are single ordered functions; exit ends with the live disconnect blanking.
- **Decoder.** Phase 2 used `@msgpack/msgpack` 2.8.0 (removed in Phase 4; the vendored MCAP bundle replaces it).
- **Tests.** Node harnesses under `tests/ros/js/` with pytest wrappers per unit (feed round-trip against a
  Python-written chunk, clock contract, engine rate/order/seek/fence properties, chart derivation parity, cache
  residency, mode transitions). DOM wiring is covered by the real UI (Phase 3), which retired the dev page.

Owner-level choices settled by delegation (2026-10-10): the replay chart span
is clamped to 120 s (raised to 600 by the two-tier store, see Phase 4); a seek pre-rolls 30 s of events rather than building a
per-recording event index; above 1× the stale indicators flag only gaps of at
least speed × timeout; "Replay last session" opens paused at the end minus one
span; the session-buffer horizon stays 600 s by default (`SESSION_BUFFER_SEC`),
with U6 measuring the heap on the box so the owner can cap it at 300 s if the
browser runs on the Jetson.

## Phase 3 — Replay UI

Built 2026-10-10; narrative and verification in `logbook/2026-10-10-gui-replay-ui-phase3.md`. Ticket 03 resolved.

**Decisions.** Owner (on the mock `ros_ws/gui/test_replay_mock.html`): layout A, trackbar + transport docked at the
bottom where the command overlay sits; compact overview (state bands + ticks, no per-topic rows); lobby and picker as
mocked. Orchestrator: the ticks drawn are exactly fault, skill_attempt, catch_event, bb_calibration (the recorded
vocabulary, `schema.OVERVIEW_TICK_KINDS`); a BB throw tick is deferred (no recorded topic carries it); absent topics
dim rather than hide; seeks clamped to the converted frontier (gone in Phase 4: the status is always complete); FF/RW are the x0.25..x8 ladder, RW from forward = reverse x1.

**Modules** (`ros_ws/gui/js/replay/ui/`, styles in `css/replay.css`; each takes `document` through a factory):
- `index.js` entry, `main.js` calls `initReplayUi(getReplayMode({...}))` before `ros.init()`; `getReplayUi()` returns
  `{toaster, picker, lobby, trackbar, absent, fenceDom, mode, dock}`.
- `lobby.js` disconnected overlay buttons, header text, `notice` routing. `picker.js` modal.
- `trackbar.js` bar, transport, hotkeys, one rAF loop; `overview.js` layout/zoom math and the strip.
- `absent.js` and `fence-dom.js` the two DOM contracts; `toast.js`, `dom.js`, `format.js` helpers.

**Hooks.** Dock is `#replay-dock` inside `#command-overlay` (class `replay-active` hides the live buttons). Mode
singleton via `getReplayMode()` (no args after init). Toasts via `getReplayUi().toaster.show(text, ms)`. `lobby.js`
owns `#conn-text`/`#conn-dot` while replay is active. The trackbar mounts and unmounts on `mode.on('state')`.
Main imports the UI, never the reverse.

**Contracts and tests.** DOM fence (decision 10): `FENCE_SURFACES` = command overlay, jog panel, juggle panel,
minimap sequencer, Ball Butler, hold-to-confirm; a MutationObserver re-fences re-enabled controls; selectors are
checked against the real GUI sources (`test_gui_replay_ui_fence_dom.py`). Absent state (decision 18): `ABSENT_TABLE`
judges only an authoritative topic set (session snapshot or complete recording) and re-evaluates on
`source.onChange` (`test_gui_replay_ui_absent.py`). Also `test_gui_replay_ui_{lobby,trackbar}.py` and two schema contracts
in `test_gui_replay_format_contract.py` (drawn tick kinds and picker key topics are in the schema).

**Known follow-up** (closed by Phase 4): the picker's per-row `/manifest` and `/status` fetches are gone; the listing
row carries `topics` and `overview`.

**Phase 5 hand-offs surfaced here.**
(a) Trail history feed: `policy.js` classes `/mocap_data` as state-render and `/balls` as columns-only, so a trail
needs a *columns window* `[p − tail_length_ms, p]` read from the resident chunks on every seek, scrub and reverse step
and appended from columns in forward play, ungated by speed. Pre-roll must cover at least the tail; `resetForSeek`
must reset the trail layer; a ball that is also an unlabelled mocap marker needs a duplicate-trail rule.
(b) A BB throw tick is derivable from `/balls`.


## Phase 4 — Direct MCAP source

Built 2026-10-10; narrative and verification in `logbook/2026-10-10-gui-replay-direct-mcap-phase4.md`. Settled by ticket
06 (`.scratch/gui-replay/issues/06-design-proposal.md`), which reverses ticket 05's convert-once cache.

**Decisions.** The browser reads the `.mcap` itself over HTTP Range and decodes in a module Web Worker; the only wait
that scaled with recording length was the conversion (about 4x real time on the Jetson), and over Ethernet the 5x byte
difference is noise. Spike: 31x real time cold, 57-82x steady under node on the Jetson, 0 mismatches against the Python
oracle. Open holds until slot 0 is resident. Unindexed bags are refused `no_index`. The Source seam made the swap local.

**Browser modules** (`ros_ws/gui/js/replay/`): `mcap-decode.js` (open, decodeSlot, latestRow, flatten, slotOf,
httpReadable), `mcap-worker.js`, `slot.js` (main-thread `slotOf` twin; keeps the 119 kB bundle off the main thread),
`allowlist.js`, and `sources.js` `McapSource`. Vendored `lib/mcap-bundle.min.js` is built by `tools/gui_vendor/build.sh`
(pinned versions, reproducible, sha256 pinned by `test_gui_vendor_pin.py`).

**Backend** (`ros_ws/gui/replay/`, stdlib server): `recordings.py` (listing, footer and metadata parse, memoised),
`api.py` (routes below), `decode.py` + `overview.py` (venv overview pass, niced, one at a time, atomic write to
`temp/replay_overview/<id>.json` with a source stamp), `convert.py` kept as the test oracle, `schema.py` still the one
contract. Routes: `GET recordings`; `GET|HEAD recordings/<id>/file` (200/206/416, 412 on `If-Match` mismatch, 409
`recording_in_progress`|`no_index` with an `X-Replay-Reason` header, `Accept-Ranges`, ETag, `no-store`, 1 MiB
streaming); `GET recordings/<id>/overview` (200, 202 queued|computing, 409, 503, 500). No manifest route: the worker's
`open` reply is the manifest. Listing row: `id, size_bytes, mtime, closed, indexed, in_progress, duration_s,
message_count, start_ns, topics, overview`.

**Worker protocol.** Requests `open|load|latest|close`, replies `opened|slot|row|error`, one FIFO, `t` arrays
transferred; the worker builds a 10 s slot in memory with the `schema.py` flatten rule.

**Hold rule.** `mode.js` loads slot 0 explicitly BEFORE the ordered entry; a refusal leaves the live GUI untouched; then
REPLAY paused (not auto-play).

**Contract tests.** JS decode == Python oracle on the fixture; Range file-vs-server; overview pass == oracle overview;
allowlist JS pin; vendored-bundle sha256; slot math pinned across main thread, worker and Python.

**Two-tier chart store** (owner ask, `logbook/2026-10-10-gui-replay-ten-minute-span-and-hidden-inputs.md`): `REPLAY_MAX_SPAN_SEC` 600; full rate in the resident window, 1 s min/max/mean digests of the derived signals out to +-330 s (`js/replay/digest.js`, low-priority never-memoised `lite` worker loads, paused while playback buffers), drawn by the `drawReplayEnvelope` hook.
**Replay-hidden input regions** (`js/replay/ui/hide.js`): juggle minimap, BB target, jog and speed-limit panels get `replay-hidden` in REPLAY; the juggle rAF stops.

**Migration.** `rm -rf temp/replay_cache/`; reinstall the unit (new ExecStart, `--overview-dir`); module workers need
Chrome/Edge 80+ or Firefox 114+.

## Phases 5-6 (outline)
- **Phase 5 balls and trails** (ticket 04, pending the owner's pick): `/balls` in the 3D scene, trails behind balls and
  markers, `tail_length_ms`.
- **Phase 6**: the fog items on the map. Owner's remaining latency asks: the overview strip arrives only after the niced pass (pre-compute overviews for all recordings in the background at service start), and slot buffering is serial (one worker/reader; add a second reader for prefetch).

## Testing Plan

- Backend tests run under the venv in the default gate (`./run_tests.sh`): `tests/ros/test_replay_{convert,api,range,
  stdlib,overview,mcap_oracle}.py`, `test_gui_vendor_pin.py`. The fixture (`tests/ros/_replay_fixture.py`) writes a small
  synthetic MCAP from the repo's `.msg` definitions with `rosbags` + the `mcap` writer (`compression="none"` default,
  opt-in zstd/lz4), embedding the schemas in rosbag2's format, with an "unindexed" variant (footer removed).
- Contract test: the allowlist equals the JS subscribe set plus `PLANNED`.
- Server tests bind ephemeral ports and use `tmp_path`; no test touches `~/Desktop/rosbags` except the footer/listing
  test over the real bags, which skips without them.
- Phase 2-5 tests follow the node-harness standard (`tests/ros/js/`).

## Notes for Collaborators

- The map (`.scratch/gui-replay/map.md`) is the decision index; it is gitignored, so this plan carries the decisions
  that matter to the build. Continue the map with `/wayfinder .scratch/gui-replay/map.md`, one ticket per session.
- Never restart `jugglebot-gui.service` while another session is using the GUI; the new server code is inert until the
  restart.
- `temp/replay_overview/` is rebuildable (one overview pass per recording opened afterwards); `temp/replay_cache/` is
  dead since Phase 4 and may be deleted.
