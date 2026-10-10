---
title: mocap_node loses QTM frames when the Jetson is loaded — 180 unique frames/s on an idle box, 70–116 in the loaded sittings of 2026-10-10; the latest-snapshot publisher turns any stall into loss; the node's per-frame cost grew 22 % this week (secondary); fixes proposed, not built
type: investigation
date: 2026-10-10
status: open
phase: "two-ball-skill-stack — R5 (Ball Butler calibration) / catching"
related_plan: two-ball-skill-stack.md
files_changed:
  - logbook/2026-10-10-mocap-frame-loss-under-load.md
  - logbook/INDEX.md
subsystem:
  - tracking
  - ros
tags:
  - mocap
  - performance
---

# mocap_node frame loss under load

## Symptoms

The BB accuracy sitting of 2026-10-10 19:23 (session `20261010T082544_262394Z`, BB FW 7) had 12 of 40 landings
rejected by the extractor (3 "mocap source-stamp gap > 0.1 s", 9 "no ballistic track passed quality gates"), where
the 2026-10-09 FW 5 session rejected 0 of 110. The ball was visible in nearly every frame that existed; there were too
few frames. `/mocap_data` carried 180 distinct QTM frames/s on 2026-10-09 (the 200 Hz publish timer's ceiling against
a 300 Hz source) and 104–116 on every 2026-10-10 sitting from 12:26 on, 70 in the known-loaded 00:24 bag, with gaps up
to 157 ms. Catch-time SD (6.6 → 9.2 ms) and cross-range landing spread grew with it.

## Diagnosis (`~/bb_calibration_sessions/mocap_rate_20261010/REPORT.md`, scripts alongside; Opus agent, 2026-10-10)

- **The loss is real and happens inside the mocap_node process**, not in stamping or de-duplication:
  `/rigid_body_poses` (published per fresh packet, no stamps) matches the unique-stamp count within 0.3–1.3 % in every
  bag; the 200 Hz publish timer itself slowed (199.5 → 164–171 Hz) and stamp gaps > 20 ms contain a median of one
  publish where ~4 were due, so the receive thread and the timer stalled together. A QTM-side drop would have left the
  timer publishing duplicates. Every bag carries the same 4 bodies in the 6D stream, so re-enabling bodies in QTM did
  not change the packets.
- **Primary cause: CPU starvation by work outside the node.** 14:49 bag: 169 Hz for 10 s, then the drop; it stepped
  back to 171–173 Hz at 15:01:10, 90 s after the session, which is when this session's analysis agent finished its bag
  replays and scoped pytest run (its outputs are stamped 14:55, its idle wait began 15:00). 19:23 bag: already 147–162
  Hz before the session (another Claude session was working on the box), stepping down at session start and again 220
  s in, never recovering. 00:24 (a gate was running): 70 Hz. The onset lies between the 10:32 bag (~180 Hz) and 12:26
  (104 Hz), i.e. when the day's merges, gates and analyses began, not with any one code change: 12:26 ran the
  min-latency clock sync but not the base-frame code.
- **Secondary: the node's own per-frame cost grew 22 %** (34 → 41 % of one core, measured on recorded frames, 5
  interleaved repetitions, spread ≤ 2 %): the larger scene (14 → 19 markers) adds ~13 points (receive 609 → 699 µs
  per packet, publish 732 → 816 µs), the new code ~9 points (clock sync 9 → 41 µs per packet; base-frame candidate
  building ~95 µs per publish). The 5 Hz base-frame update averages 1.4 ms (max 6.7 ms) because tracking held in only
  63 % of updates and the full identification (0.8 / 3.6 / 4.6 ms at 13 / 19 / 27 candidates) ran often. ~80 % of
  the per-packet cost is building Python ROS messages (`on_packet` builds 4 `PoseStamped` per 300 Hz packet). HEAD
  sustains 171.5 Hz on an idle box (the 14:49 tail), so this is fragility, not the loss itself.
- **Amplifier: the latest-snapshot design.** QTM arrives over TCP with every frame; `on_packet` keeps only the latest
  snapshot and the 5 ms timer publishes it, so packets arriving in a burst after a stall overwrite each other: delay
  becomes loss. Nothing grows over time; buffers are bounded.
- The "receive latency 1.9 → 8.9 ms" figure is mostly a change of definition with the min-latency sync (excess over
  the fastest packets instead of a mean): compare only bags from 12:26 on (4.5 ms clean, 9–13 ms loaded).
- Side finding, not chased: stamps go backwards 164–490 times per bag, including the healthy 10-09 session; the
  publisher may read markers and the frame timestamp under separate lock acquisitions.

## Discussion

The rule "no test suites or heavy jobs during a sitting" (logbook `2026-10-10-mocap-min-latency-clock-sync`) was
written for the clock sync; it also governs frame supply, which the catch prediction and the landing extractor both
live on. The node runs with no priority or core reservation and ~40 % of a core of Python per frame on a shared box.

## Proposed fixes (not implemented; pointers in the report)

1. Queue every frame in a bounded deque in `mocap_interface.on_packet` (`mocap_interface.py:471-480`) and have
   `_publish_mocap_data` (`mocap_node.py:606-660`) publish each with its own stamp, so delay becomes latency, not
   loss, and the 180 Hz ceiling goes.
2. Priority / core for mocap_node (`launch/jugglebot_launch.py:363`); keep tests, builds and replays off the box
   while the stack is up.
3. Cut per-packet cost: build `PoseStamped` only when publishing (`mocap_interface.py:407-436`), `math.isnan`
   instead of numpy per marker (:381, :468), precomputed index→name lookup (:673).
4. Log QTM's own drop counter (parsed, unused, :384-386) and frame-number gaps in the 1 Hz debug line.
5. Base identification at ≤ 1 Hz once tracked, or vectorised (`bb_base_frame.py:221-258`); find out why tracking
   fails 37 % of updates.
6. Live check for the next sitting: `pidstat -u -t -p $(pgrep -f lib/jugglebot/mocap_node) 1` and `mpstat -P ALL 1`
   into files; node threads well under 100 % with total CPU high = load; threads at ~100 % = the node's own cost.

## Open

- **Fix 1, 3 and 4 built 2026-10-10** (bounded frame queue, cheaper per-frame work, loss counters + WARN): logbook
  `2026-10-10-mocap-frame-queue`, branch `mocap-frame-queue-2026-10-10`, not merged or deployed; this entry stays open
  until `/mocap_data` is verified at ~300 unique frames/s live.

- n = 1 bag per condition around the onset; the load at 12:26 and 19:23 is inferred (no CPU log).
- Decide whether to build fixes 1–4 before relying on catches (two-ball juggling).
