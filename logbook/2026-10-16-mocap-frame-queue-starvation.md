---
title: "mocap_node's 30-52 % frame-loss bursts are the BB calibration sweep's post-processing (run_calibration + estimate_base_pose, 4.3-7.2 s) running synchronously inside the bb/heartbeat callback on the single-threaded executor; drops = 300 Hz x solve time - 150 queued frames to within 15 frames on 13 of the 14 solved sweeps of the five named launches (the 14th, 16:23, is 16 off); no burst at idle, at goal accept or during juggling; the tracker's ~1/3 frame share is NOT this; fix options for the owner, none built"
type: investigation
date: 2026-10-16
status: resolved
phase: "tracker fit latency (follow-up to handoff-solve-chaining P2)"
related_plan: handoff-solve-chaining.md
related:
  - 2026-10-10-mocap-frame-queue.md
  - 2026-10-10-mocap-frame-loss-under-load.md
  - 2026-10-16-tracker-flight-fit-late-since-matcher-fix.md
files_changed:
  - logbook/2026-10-16-mocap-frame-queue-starvation.md
  - logbook/INDEX.md
  - tools/probes/mocap_starvation/bag_stalls.py
  - tools/probes/mocap_starvation/log_stalls.py
  - tools/probes/mocap_starvation/solve_vs_drain.py
  - tools/probes/README.md
subsystem:
  - tracking
  - ros
tags:
  - mocap
  - performance
  - bb-calibration
---

# mocap_node frame-queue starvation: the BB calibration solve blocks the executor

## Symptoms

Every launch log since 2026-10-13 carries mocap_node WARNs such as `1739 of 3302 expected QTM frames lost
(52.7 %) in the last 11 s — 1728 dropped by mocap_node (frame queue full: starved for over 0.5 s)`
(`mocap_node.py:572`; 0.5 s = `FRAME_QUEUE_MAXLEN` 150 / 300 Hz, `mocap_interface.py:24,154,595`). The
tracker-fit entry of the same day left the question open: are these drops the reason `ball_tracker_node` sees only
~1/3 of QTM's frames, and do they hit flights?

## Diagnosis

1. **When.** In the five named launches (2026-10-13 23:12, 10-14 20:54, 10-16 10:50, 16:10, 16:23) every
   node-overflow WARN follows a mocap_node `Calibration sweep complete ... Processing...` line (the BB calibration
   sweep's end edge) by 3-10 s. None follows a goal accept, a `reload started`, a throw or idle time. Across all 23
   launch logs from 2026-10-11 to 10-16: 55 node-overflow WARNs, 51 within 20 s of a sweep's `Processing...`; of the
   other 4, one is a 27 s refused-sweep solve on 10-11 15:14 (8126 frames, same mechanism, outside the 20 s window),
   one is 259 frames (7.4 %) in that launch's collection window, 8-12 s after node start (see the exception under item 2), and two are in the 2026-10-16 15:05 launch (see 5).
   The peer note quoted in the tracker entry ("after every calibration sweep **and at idle**") is wrong about idle.
2. **How long the executor stops.** mocap_node's 1 Hz `qtm clock sync` DEBUG line comes from an executor timer, so
   a gap in it is an executor stall. On every sweep the gap starts at `Processing...` and ends at the `BB calibrated`
   / refusal line. Taking T = (first 1 Hz line after the stall) − (`Processing...`), the predicted loss 300·T − 150
   matches the logged `dropped by mocap_node` count (`log_stalls.py`):

   | launch | T (s) per sweep | predicted / logged drops |
   |---|---|---|
   | 10-13 23:12 | 6.20, 5.56, 7.19, 5.28 | 1710/1700, 1518/1519, 2007/1998, 1434/1435 |
   | 10-14 20:54 | 4.32, 4.39, 4.40, 5.13, 5.57, 6.25 | 1146/1145, 1167/1170, 1170/1170, 1389/1391, 1521/1522, 1725/1725 |
   | 10-16 10:50 | 5.06, 5.78 | 1368/1362, 1584/1585 |
   | 10-16 16:10 | 4.45 | 1185/1178 |
   | 10-16 16:23 | 6.21 | 1712/1728 (16 off) |

   (A seventh 10-14 sweep, 724.2, was refused in under 1.5 s and raised no WARN.) `log_stalls.py` reproduces the
   table; one cell differs from the first write-up: 16:23 predicts 1712 (T = 6.21 s), 16 frames from the logged 1728, one
   over the 15-frame tolerance this entry states (the other 13 solved sweeps are within 7). The 7 s **collection** loses
   nothing in the five named launches: the whole count is accounted for by the solve. The one exception is the
   2026-10-11 15:14 launch, whose 259-frame WARN (ending 0.65 s before that sweep's `Processing...`) falls in the
   collection: the queue sat at 150/150 on five consecutive 1 Hz lines with the lines 1.0-1.4 s apart and no single
   gap, which is intermittent executor slowness 6-12 s after node start, the item-5 class, not the solve; unexplained. This corrects the 2026-10-11 inference in
   `2026-10-10-mocap-frame-loss-under-load` ("the sweep blocks mocap_node's executor").
3. **Bags agree** (`bag_stalls.py`): 16:10 has exactly one `/mocap_data` publish gap of 100 ms or more in 114 s,
   4464 ms starting at the `Processing...` instant, with ~1183 frames never published and a 242-frame catch-up
   burst after it (the 150 queued frames plus the ~90 that arrive during the ~0.3 s the drain takes to catch up; the
   burst is defined as stamp > 20 ms older than publish time, so frames arriving after the solve ends count too); outside it the per-second count is p5/p50/p95 279/299/307. The 4464 ms publish gap includes ~0.5 s of queued frames that were published later; the stamps lost are 3947 ms (3.9466 s x 300 = 1184, matching the 1183 here and the logged 1178). 16:23: one 6214 ms gap (~1732 frames);
   10:50: 5072 and 5791 ms (~1376, ~1589). The only other gaps are a single 146 ms and a single 347 ms one.
4. **Mechanism.** `_on_bb_heartbeat` (`mocap_node.py:931`) calls `_finalize_calibration()` (`:985`) inline on the
   CALIBRATING→not edge; the node spins the default single-threaded executor (`:1444`), so the 5 ms drain timer
   (`:343`) cannot run until the solve returns. The asyncio receive thread keeps queuing; the 150-frame deque
   overflows 0.5 s in, and everything after that until the solve ends is dropped. Offline on the 16:10 sweep (2097
   frames): `run_calibration` 1.38-1.55 s, `estimate_base_pose` over the sweep's 2097 base-candidate frames
   1.67-1.72 s (cProfile: 4202 `kabsch` + 2100 `_pose_from_pairs` calls, ~813k Python calls), 3.1 s together on one
   thread. The node took 4.45 s because the same GIL serves its 300 Hz receive thread. The state-file writes and the
   publish take tens of ms (`BB calibrated` follows `Yaw offset (sweep)` by ~10 ms).
5. **A separate class, n = 1, unexplained.** The 2026-10-16 15:05 launch ran for 31 s before it was stopped with
   ctrl-c, with no sweep: `/mocap_data` ran at 115 Hz throughout (no discrete gap), publish − QTM-stamp latency p50
   840 ms, the 1 Hz line firing every 1.3-1.8 s, 2037 and 2091 frames dropped. That is a slow publish side or a
   starved process (H2/H4), not a blocked callback. The box load at that minute was not recorded; the 15:07
   relaunch shows only the sweep-shaped stalls (3.9, 4.9 and 4.9 s).

### Hypotheses

- **H1 (long synchronous callback): CONFIRMED** for every burst except item 5. Specifically the post-sweep solve,
  not the collection, the 1 Hz timers or a state-file write.
- **H2 (whole-process CPU starvation): KILLED** for these bursts. They line up with sweep ends, never with goal
  accept, solver-worker load or replays, and the stream holds 299 frames/s p50 through the juggling. Possibly item 5.
- **H3 (receive thread stalls): KILLED.** "never received from QTM" stays ≤ 19 frames per window. In the bags
  the publish gap is longer than the stamp gap (4464 against 3947 ms) and is followed by a catch-up burst of old
  stamps, which means the frames were received and queued but not published.
- **H4 (slow publish, gradual fill): KILLED** for these bursts: the loss is a step (one gap of the solve's
  length), not a ramp. Item 5 is the one H4-shaped episode.

### Consequence for catches

The burst is ~4-7 s long and starts at a sweep's end. A flight overlaps it only if a throw is dispatched within
~7 s of a calibration, and none was in these launches (closest: 16:23, WARN 9 s before the first attempt). The
tracker's ~1/3 frame share in `2026-10-16-tracker-flight-fit-late-since-matcher-fix` is **not** caused by this:
mocap_node publishes ~299 frames/s through the juggling, so that share is lost downstream (ball_tracker_node's
own processing, subscription or QoS). That question is still open and belongs to the tracker.

## Discussion

The finding reframes the brief. The cost is limited to the ~5 s after each sweep, when nothing tracks balls. The
owner already accepted this remainder on 2026-10-11, under the wrong mechanism ("the sweep blocks the executor").
The real cost today is smaller and different: the WARN raises a false alarm in every sitting, it sent a diagnosis
to the wrong node, and it blinds the node to heartbeats at the sweep's end. On 10-14 a sweep started at 759.6, 3.6 s
after a solve released the executor at 756.0, so the BB's state changes during a solve are only seen late. The
QTM clock sync also logs slew clamps right after the stall (16:10: 2 → 25 over the next 10 s); that was not chased.

Options, none built:

- **A. Solve in a worker thread** (snapshot the sweep's inputs on the executor thread, run `run_calibration` +
  `estimate_base_pose` off it, apply the result, write state and publish back on the executor thread). The probe
  shows the GIL is not the obstacle: with the solve in a thread, an imitation drain loop at the node's cost keeps
  max tick gaps of 37-74 ms, a queue peak of 11-22 of 150 and 0 drops, and the solve slows from 3.1 to 3.7-4.0 s.
  Design work: `_finalize_calibration` reads ~10 `self._calib_*` / base-monitor attributes that a new sweep's start
  edge rebinds, so the inputs must be snapshotted. A sweep that starts while a solve is in flight needs a rule
  (refuse the new one, or drop the stale result). The eight `tests/ros/` files that drive `_on_bb_heartbeat` /
  `_finalize_calibration` synchronously need a synchronous seam (an inline-executor hook). `_finalize_calibration` is also the function the
  live `bb-calibration` branch (worktree `~/Desktop/Jugglebot-bbcal`, Phase 2 next) is editing, so building A here
  would conflict with it.
- **B. Make the solve cheaper.** Vectorise or subsample `estimate_base_pose` (it fits all 2097 frames; the base
  is static, so a few hundred would do) and `track_constellation`. This shortens the stall but does not remove
  it: getting under the 0.5 s queue horizon would need a ~6x cut on a loaded box. It is a complement to A, not a
  replacement.
- **C. Accept and re-word.** Keep the stall and name it in the WARN (e.g. "during the BB calibration solve"), so
  sittings stop reading it as starvation. Free, honest, and it fixes nothing.
- **D. MultiThreadedExecutor + reentrant groups.** Rejected: every callback in the node would then need to be
  thread-safe. A gives the same benefit and keeps all node state on one thread.

Recommendation: A, scheduled into the `bb-calibration` plan (it owns `_finalize_calibration`). Do C now if the false
alarm costs more than a one-line change. Item 5 needs a CPU log (`pidstat`/`mpstat`, proposed fix 6 of the
2026-10-10 entry) the next time it happens. It is n = 1 and was not chased.

## Fix

**Resolved 2026-10-16 by option A (owner): `2026-10-16-mocap-calibration-solve-off-executor.md`** (worker-thread solve, executor-thread apply; hardware receipt row 30b of `tests/hardware/session_skill_stack_rehearsals.md`). Original text follows.

None. The diagnosis is clear, but the fix needs a design decision (in-flight overlap rule, snapshot seam) and
touches a function another live branch is editing. The owner decides between A/B/C. The hardware rehearsal for A:
over a sitting with ≥ 2 sweeps, zero `dropped by mocap_node` WARNs, `bag_stalls.py` shows no `/mocap_data`
publish gap of 100 ms or more at the sweep ends, and the `BB calibrated` values are unchanged.

## Verification

- 2026-10-16, `python tools/probes/mocap_starvation/log_stalls.py` (default glob `~/.ros/log/2026-10-1[1-6]-*/launch.log`;
  gaps in the 1 Hz `qtm clock sync` line vs `Processing...` stamps): the Diagnosis 2 table (13 of 14 sweeps of the five
  named launches within 15 frames, 16:23 at 16) and 55 node-overflow WARNs across 23 logs, 51 within 20 s of a
  sweep end; the 4 outside are 259 and 8126 (10-11 15:14) and 2037 and 2091 (10-16 15:05), as in Diagnosis 1.
- 2026-10-16, `python tools/probes/mocap_starvation/bag_stalls.py ~/Desktop/rosbags/<bag>` for 2026-10-16_16-10-37,
  _16-23-01, _10-50-42, _15-05-39, _15-07-58: the numbers in Diagnosis 3 and 5.
- 2026-10-16, `python tools/probes/bb_calibration_replay.py --bags ~/Desktop/rosbags/2026-10-16_16-10-37
  --emit-fixture 2026-10-16_16-10-37:0 --out temp/probes/mocap_starvation/fixture_2026-10-16_16-10-37_0.json`:
  2097 frames, replayed yaw −11.5251° (logged −11.5253°).
- 2026-10-16 16:45 (load average 2.4-2.7, live stack up), `OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 python
  tools/probes/mocap_starvation/solve_vs_drain.py temp/probes/mocap_starvation/fixture_2026-10-16_16-10-37_0.json`
  (plus two earlier scratchpad runs of the same code): `run_calibration` 1.38 s, `estimate_base_pose` 1.72 s; solve
  in a worker thread 3.72-3.99 s, drain tick-gap max 37-74 ms, queue peak 11-22 of 150, 0 dropped.
- No code changed, so no tests were run.
