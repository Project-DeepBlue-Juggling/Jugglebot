---
title: mocap_node publishes every QTM frame from a bounded queue (150 frames, 0.5 s) instead of a latest-frame snapshot; per-frame work cut so all ~300 frames/s cost 23-24 % of a core against 43 % for 200 before; frame loss now logged (1 Hz counters, throttled WARN); built and gated, not deployed
type: bugfix
date: 2026-10-10
status: in-progress
phase: "two-ball-skill-stack — R5 (Ball Butler calibration) / catching"
related_plan: two-ball-skill-stack.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/mocap_interface.py
  - ros_ws/src/jugglebot/jugglebot/mocap_node.py
  - tests/ros/test_mocap_frame_queue.py
  - tests/ros/test_mocap_interface.py
  - tests/ros/test_mocap_node.py
  - tests/ros/test_mocap_node_base_frame.py
  - tests/ros/conftest.py
  - logbook/2026-10-10-mocap-frame-queue.md
  - logbook/2026-10-10-mocap-frame-loss-under-load.md
  - logbook/INDEX.md
subsystem:
  - tracking
  - ros
tags:
  - mocap
  - performance
---

# mocap_node: a bounded frame queue instead of the latest-frame snapshot

## Problem

From the investigation `2026-10-10-mocap-frame-loss-under-load`: QTM streams every frame at 300 Hz over TCP.
`MocapInterface.on_packet` overwrote one latest-frame snapshot, and a 5 ms timer in `mocap_node` published it.
When the process was CPU-starved, the receive thread and the timer stalled together. The packets that then arrived
back to back overwrote each other, so a delay became a loss. The result was 180 unique frames/s on an idle box and
70-116 under load, with gaps up to 157 ms. The node spent about 41 % of a core per second, about 80 % of it building
Python ROS messages. QTM's drop counter was parsed but never used, so the loss never showed in any log.

## Change

- **Queue (`mocap_interface.py`).**
  - `on_packet` builds one `MocapFrame` namedtuple per QTM frame and appends it to
    `collections.deque(maxlen=FRAME_QUEUE_MAXLEN)`, all under one `data_lock`. The tuple holds: frame number, QTM µs,
    the ROS stamp, the ROS receive time, aligned, labelled `[(label, x, y, z, r)]`, unlabelled `[(x, y, z, r)]`,
    bodies `(name, frame_id, x, y, z, qx, qy, qz, qw)` and the 7 BB rows.
  - The ROS stamp is converted once, at receive, in `_update_qtm_clock_sync`, which now returns
    `(receive_ns, stamp_ns)` from the same min-latency sync.
  - `maxlen` is 150, which is 0.5 s at 300 Hz. That is about 3× the longest gap in the loaded sittings (157 ms).
    Only a stall longer than 0.5 s loses frames: the oldest go first and are counted as `overflow_drops`. The same
    bound caps the latency a starved node can build up at 0.5 s.
  - A packet that repeats the last frame number is not queued (counted as a duplicate). A jump forward is counted as
    QTM gap frames. A frame number that goes backwards is counted as a QTM restart, not a gap. A disconnect clears the
    queue and the counter.
  - `drain_frames()` / `latest_frame()` / `take_stream_stats()` replace the snapshot getters and `clear_*`. These
    are removed: `get_all_markers_base_frame`, `get_labelled_markers`, `get_body_poses`, `clear_markers`,
    `clear_body_poses`. `latest_frame_ros_ns()` stays, reading the last record. `ball_butler_markers` (used by
    mocap/status) is still the latest frame's.
- **Publisher (`mocap_node.py`).**
  - Each tick of the unchanged 5 ms timer drains the queue and publishes **every** frame, oldest first, through
    `_publish_frame`: `/mocap_data`, `/rigid_body_poses` (for frames that carry bodies) and `/bb/markers`, each with
    the frame's own stamp. It also feeds the base frame and the calibration window.
  - Message types, topics and stamp semantics are unchanged. For rigid bodies, the header is stamped at publish time
    and each pose at its frame's receive time, as before.
  - An empty tick now publishes nothing. Before, it re-published the last frame with no markers: about a third of
    `/mocap_data` in the loaded bags.
  - The tick stays at `hw.TRACKING_MOCAP_DT_S` = 5 ms. A drain-all tick only sets latency (0-5 ms; consumers fit
    against the QTM stamps). A faster tick would cost executor wake-ups to save at most 2.5 ms of mean latency, and
    that constant lives in a gated file anyway.
- **Cheaper per-frame work.** I measured each change and took the hot spots from the profile:
  - `PoseStamped` is built only when publishing, once per body per frame. The receive path keeps tuples.
  - `RigidBodyPose`'s default `PoseStamped` is filled in place instead of building a second one; that alone was about
    1/5 of the publish time.
  - `math.isnan` replaces numpy per marker.
  - Index→name lists are rebuilt only when `marker_dict`/`body_dict` is replaced (this was an O(L²) scan per packet).
  - The quaternion is computed with plain floats (`quaternion_from_rotation_list`, pinned branch for branch against
    the old numpy version).
  - The alignment check uses `math`.
  - The base-candidate array is built only when `_feed_base_frame` will use it (`_base_feed_due`: every frame of a
    calibration window, otherwise once per 0.2 s of frame time).
  - Markers are built by `_marker_fast`, which writes rosidl's `_x`/`_position`… slots directly. It is enabled only
    if both classes have exactly that slot layout and an import-time self-check builds a message equal to the
    checked builder's; otherwise the old setter path (`_marker_checked`) is used. The checked path is what runs
    under the tests' message stand-ins.
- **Loss visibility.**
  - `_check_stream_health` runs on the 1 Hz clock-offset timer. It appends to the existing
    `qtm clock sync: …` DEBUG line: `stream N frames/s, queue max d/150, lost: node overflow x (total X), QTM gaps
    y frames (total Y), duplicates, QTM restarts, QTM 2D drop ‰, out of sync ‰`. The ‰ figures are QTM's 3D-header
    2D drop and out-of-sync rates, the highest since the last line.
  - On any overflow or QTM gap it logs one WARN: `mocap: N QTM frames lost in the last T s — a dropped by
    mocap_node (frame queue full …), b never received from QTM (k frame-number gaps); QTM 2D drop …‰ — check the
    Jetson load …`. The WARN is limited to one per `MOCAP_LOSS_WARN_PERIOD_S` = 10 s; losses inside that window are
    summed into the next WARN, so none goes unreported.
- **Side finding fixed: stamps going backwards.** In the 10-09 and 10-10 bag stats, **every** backward step
  (164-490 per bag, 0.2-21 µs) is an empty message right after a real one. The old empty tick re-published the
  previous frame's QTM time through an offset that had moved down a few µs. With one conversion per frame and no
  re-publishing, this cannot happen any more. A clock-sync re-anchor larger than one frame period can still step the
  stamps back; that comes from the sync, not the queue.

## Verification

Benchmark: `~/bb_calibration_sessions/mocap_queue_20261010/` (`bench2.py`, `run_bench2.sh`, `bench2_summary.txt`).
It is the investigation's `bench.py` adapted:
- the same 3000 recorded frames per dataset, rebuilt into real QTM packets and parsed by `qtm_rt`;
- the real `on_packet` / `_publish_mocap_data`, with the real Foxy message classes and `serialize_message` standing
  in for publish;
- a tick after 2 of every 3 packets (200 ticks/s against 300 packets/s);
- 5 interleaved repetitions of old (`590a1d13`), new and new with the checked builder, under `nice -n 10`;
- run 2026-10-10 22:13-22:22 with other work on the box (load average 2-4).

| µs, mean ± SD over 5 reps | old FW5 | new FW5 | old 19:23 | new 19:23 | old 14:13 | new 14:13 |
|---|---|---|---|---|---|---|
| receive per packet (parse + `on_packet` incl. sync) | 676 ± 9 | 173 ± 2 | 795 ± 58 | 200 ± 9 | 802 ± 16 | 204 ± 13 |
| publish per published frame | 776 ± 11 | 515 ± 3 | 938 ± 93 | 575 ± 21 | 946 ± 12 | 592 ± 39 |
| frames published per 300 packets | 200 | 300 | 200 | 300 | 200 | 300 |
| CPU per s at 300 packets/s | 358 ms (36 %) | 206 ms (21 %) | 426 ms (43 %) | 233 ms (23 %) | 430 ms (43 %) | 239 ms (24 %) |
| new/old CPU, paired by rep | | 0.576 ± 0.004 | | 0.547 ± 0.024 | | 0.555 ± 0.032 |

- The new code publishes **every** frame (300/s, against at most 200 of which about 180 were unique) at 55-58 % of the
  old CPU.
- The checked-setter variant costs 25 / 29 / 30 % of a core. The slot-writing builder saves about 5-6 points and
  builds messages field-identical to the checked builder on all 1000 frames checked per dataset (also after a
  serialize/deserialize round trip).
- The old 19:23 figure (43 %) reproduces the investigation's 41 %, which is within this run's load.
- **Equivalence** (`lockstep_compare.sh`, one tick after every packet, old against new, 3000 frames × 3 datasets):
  every `/mocap_data`, `/bb/markers` and `/rigid_body_poses` message is field-identical after deserialization,
  including stamps, labels, NaN BB rows and quaternions. The raw CDR bytes differ only in uninitialised padding.
- **Tests** (`tests/ros/test_mocap_frame_queue.py`, 21 new tests):
  - A 45-frame stall followed by one tick publishes 45 messages per topic, with strictly increasing, distinct stamps
    equal to each frame's own conversion, and each message carries its own frame's markers.
  - Overflow drops the oldest and is counted.
  - An empty tick publishes nothing.
  - Duplicates, gaps and QTM restarts are counted correctly, and a disconnect clears the queue.
  - Stamps never go backwards under a moving offset with jittered latency.
  - Name lookups follow a replaced dict; bodies carry the right frames and offset; rigid-body stamps and values are
    right.
  - The calibration collector still takes each frame once (per distinct stamp).
  - The base monitor still runs 5 times per second of frame time, and candidates are built only then.
  - The 1 Hz line carries the counters; the WARN fires at once, is throttled, sums the counts and stays quiet with no
    loss.
  - The float quaternion matches numpy on every branch, NaN included.
  - The fast-builder self-check accepts rosidl-style classes and rejects others.

  Existing tests moved to the frame API: `test_mocap_interface.py`, `test_mocap_node.py`,
  `test_mocap_node_base_frame.py`. The conftest `RigidBodyPose` stand-in now default-constructs a full
  `PoseStamped`, as the real message does.
- 2026-10-10, scoped: every `tests/ros/test_*.py` that imports mocap_interface/mocap_node, plus the three logbook
  tests. Result: 961 passed, 1 skipped.
- 2026-10-10, full gate (`./run_tests.sh --full` from the worktree): see the commit's `Tests:` line.

## Not verified live

- `/mocap_data` unique frames/s on an idle box: expect about 300, up from 180. Also check `/rigid_body_poses` and
  `/bb/markers` at the same rate.
- The 1 Hz `qtm clock sync: … stream …` DEBUG line in a real log (rosout/bag), and QTM's real drop / out-of-sync
  figures.
- The WARN under a deliberate load: for example, a scoped pytest during an idle stack. Expect no loss for stalls
  under 0.5 s and the queue high-water mark to rise in the DEBUG line. Only a starvation longer than 0.5 s should
  warn "dropped by mocap_node".
- Real `publish` cost with DDS and subscribers at 300 Hz (the benchmark used `serialize_message`), and the executor
  cost of a catch-up tick. After a 150 ms stall one tick publishes about 45 frames, roughly 25 ms, and delays the other
  callbacks in the node (heartbeat, `bb/axis_estimates`) by that much.
- Downstream at 300 Hz:
  - The ball tracker's frame-count thresholds (`missed_frames_to_lose` = 10, `TRACKING_MAX_FRAMES_WITHOUT_MEASUREMENT`
    = 200) now mean 33 ms / 0.67 s of frames, against about 50 ms / 1 s of messages before (which included empty
    re-publishes).
  - skill_node's frame check now gets about 300 Platform samples/s (deque maxlen 400 still covers its 1 s window).
  - Bags get 1.5× the `/mocap_data`, `/rigid_body_poses` and `/bb/markers` messages.
  - A BB calibration window collects about 300 frames/s, without the duplicate snapshots that went into the arc fit
    before.

## Follow-ups (out of scope)

- Priority or core pinning for mocap_node in the launch file.
- Base-frame identification rate / vectorising `identify_base_markers` (4-7 ms GIL pieces).
- The `PoseStamped` header of each rigid body still carries the receive time, not the QTM-derived frame stamp.
  Switching it would give `/rigid_body_poses` consumers a capture-time stamp (no node reads it; check the offline probes under `tools/probes/` before changing it).

**Reviewer's adjustment (merge, 2026-10-10 23:05):** `ball_tracker_node.py` `missed_frames_to_lose` 10 → 15 so the
lose-track window stays ~50 ms at ~300 msgs/s (it would have shrunk to ~33 ms). `TRACKING_MAX_FRAMES_WITHOUT_MEASUREMENT`
= 200 stays (gated `hardware_config.py`; now ~0.67 s of coasting instead of ~1 s) — revisit at the next admissible re-sweep.

**Merged-state gate (skill-stack 7500b425, 2026-10-10 22:38–22:44, alone on the box, log
`temp/logs/full_gate_mocap_queue_merged_20261010.log`):** PASS — 6596 passed, 9 skipped, 1 xfailed; serial 6 passed.
Installed with `colcon build --packages-select jugglebot` at 22:46; live verification (≈300 unique frames/s on `/mocap_data`
on an idle box, the 1 Hz stream line, a loss WARN only under real starvation) is the owner's next launch.
