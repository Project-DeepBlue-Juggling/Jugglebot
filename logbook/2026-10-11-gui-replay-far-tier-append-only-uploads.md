---
title: GUI replay - the slow long-tail fill was whole-buffer WebGL re-uploads, not the worker lane
type: investigation
date: 2026-10-11
status: resolved
phase: "gui-rosbag-replay - ticket 09 (latency, post Phase 6)"
related_plan: gui-rosbag-replay.md
files_changed:
  - ros_ws/gui/js/ball-history-layer.js
  - ros_ws/gui/js/replay/ball-history.js
  - tests/ros/js/ball_history_layer_harness.js
  - tests/ros/test_gui_ball_history_layer.py
  - tools/probes/replay_lane_trace.mjs
  - tools/probes/replay_slot_load_timing.mjs
  - tools/probes/README.md
  - plans/active/gui-rosbag-replay.md
  - logbook/2026-10-11-gui-replay-deep-links-and-marker-history.md
  - logbook/2026-10-11-gui-replay-trail-overlay-and-ball-history.md
  - logbook/2026-10-11-gui-replay-far-tier-append-only-uploads.md
  - logbook/INDEX.md
---

# GUI replay: the slow long-tail fill was whole-buffer WebGL re-uploads

## Symptom

On the busy 18-minute bag (`2026-10-02_12-43-28`), with the trail tail set to the whole recording, the far-tier fill
took ~160 s on the Jetson in headless Chromium. Single filtered slot loads appeared to take ~20–560 ms, with no
dependence on the topic filter. Phase 6 left the cause "suspected, unverified: queueing on the shared low-priority
worker lane" ([2026-10-11-gui-replay-deep-links-and-marker-history](2026-10-11-gui-replay-deep-links-and-marker-history.md)).
Ticket 09 (latency) was to start by measuring that contention.

## Diagnosis

`tools/probes/replay_lane_trace.mjs` wraps `Worker` so it can time every request from post to reply on the main
thread, against the worker's own `ms`. Busy bag, playhead at the end, tail = the whole recording:

- **The lane is not the bottleneck.** Queue waits on both lanes were ≤ 30 ms, and the worker was busy ~21 s of a
  ~240 s fill. A filtered (`/balls` + `/mocap_data`) load is ~150–190 ms of worker time; a full lite load is ~330 ms.
- **The fill came in bursts.** About 22 slots in ~6 s, then a ~110 s gap during which one reply took 113 s to arrive
  although the worker spent 158 ms on it. During those gaps even CDP evaluations took ~10 s, so the main thread was
  blocked.
- **The profile names the cause.** Over 136 s, the CPU profile spent 122 s in WebGL `bufferSubData`. On every
  arrival, `ball-history-layer.js` rebuilt the vertex arrays and set `needsUpdate` with no update range, and Three
  r170 then uploads the attribute's WHOLE CAPACITY (up to 1M vertices × 28 B). That is 1.5 GB over 456 calls per fill,
  O(slots²). Headless GL here is software (llvmpipe, even with ANGLE/EGL flags; snap Chromium cannot reach the Tegra
  GPU), so each upload blocked the main thread. The worker's replies sat undelivered in the meantime, and that delay
  is what the 20–560 ms "load times" were.
- **Two "before" figures.** The traced run, with CDP polling and a profiler attached, was still filling at ~240 s,
  against ~160 s unprofiled in Phase 6. The 136 s profile covers only part of that run.
- **The control run.** With `bufferSubData` stubbed to a no-op, the same fill reached all 109 slots in 27.4 s.

## Discussion

- **Two hypotheses withdrawn.** Phase 6 first blamed Range fetch plus decompression; its own probe disproved that.
  It then suspected lane contention with the chart digester, and this trace disproves that too. Both were inferred
  from wall times taken on the main thread, which included a stall on that same thread. The trace separates queue,
  service and delivery time; future latency claims should come from it, not from caller-side wall time.
- **Why append-only, and not cheaper publishes.** Throttling publishes (e.g. at most one per 500 ms) would cut the
  number of uploads about threefold but keep each one at full capacity, so the cost stays quadratic. Append-only makes
  the cost linear and needs no timing constant.
- **Why joins live in a second geometry.** Slots arrive nearest-first, i.e. OLDER slots land after newer ones. A
  segment across a slot boundary belongs to the later slot, so prepending an older slot changes the newer slot's
  join count. Kept in one buffer, every boundary change would shift data that is already uploaded. With intra-slot
  segments in an append-only main buffer (LineSegments order is irrelevant) and joins (≤ one per key per boundary,
  a few thousand) in a small geometry rewritten on every call, uploads stay proportional to what arrived.
- **Accepted tradeoffs.**
  - An append keeps the old `aT` epoch, so an older slot's times are negative. float32 keeps ms resolution over a day.
  - An eviction, a change to the cap-driven marker drop, or a buffer grow rebuilds in full. Growth doubles, so total
    upload stays ≤ ~2× the final size. Once a recording is cap-bound (over 1M vertices), every arrival drops the
    oldest row and so rebuilds: O(used) per arrival there, as before. The busy bag (715,702 vertices) never reaches it.
  - A fresh allocation is marked dirty without an update range. Its first upload is a whole-capacity `bufferData`,
    and r170 does not clear pending ranges at that point, so a range set then would be re-sent with the next append.
  - Two draw calls instead of one.
- **What this does NOT show.** On the owner's Win10 GPU the old re-uploads were far cheaper than under software GL,
  so that machine may not have had a 160 s fill. The owner's look is still owed. The balls-first two-sweep was
  measured under the same artefact and has not been re-measured.

## Fix

- **`js/ball-history-layer.js`:**
  - Rows are matched by object across `setSlots`. Each slot's intra-slot segments go into the main buffer in arrival
    order, and only that range is uploaded (`addUpdateRange`). A full rebuild uploads only the used prefix.
  - Boundary joins go in a second `LineSegments` that shares the material.
  - The cap policy is unchanged, and so is the per-slot count that it uses (intra-slot plus joins into the slot).
  - `stats()` gains `fullUploads` and `uploadedVertices`.
- **Tests (`ball_history_layer_harness.js`):**
  - an append of a newer and then an OLDER slot uploads only the slot plus the joins, with no full rebuild, and
    draws exactly the segment multiset of a from-scratch build;
  - the pending Three update ranges are element-unit and cover only the appended slot;
  - evicting the head row rebuilds and re-bases the epoch.
- **Docs.** The probe docstrings, the `ball-history.js` comments, the plan's Phase 5b text and both earlier entries
  now carry the measured cause.

## Verification

- **Scoped tests:** `pytest tests/ros/test_gui_ball_history_layer.py tests/ros/test_gui_replay_ball_history.py -q`
  (run 2026-10-11, before the audit fixes): **24 passed**.
- **Headless, Jetson, busy bag:** `node tools/probes/replay_lane_trace.mjs 2026-10-02_12-43-28 120 tail 99999`
  (run 2026-10-11, after the fix): **109 slots in 26.1 s**, `bufferSubData` 20 ms in total. Before the fix, slots were
  still arriving ~240 s in, with 122 s of a 136 s profile in `bufferSubData`.
- **Gate:** `./run_tests.sh` (run 2026-10-11, after the audit fixes): **6652 passed, 9 skipped** in 294 s, plus the
  serial tail **3 passed**.
- **Unverified:** the fill time on the owner's Win10 Chrome with a real GPU.
