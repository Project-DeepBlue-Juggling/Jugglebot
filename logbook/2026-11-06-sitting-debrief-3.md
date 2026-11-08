---
title: "Sitting 2026-11-06 20:20 debrief: the Hand at encoder period 9 ms passes (BUG-50 closed); its frame losses recur at half the period, which confirms the clock beat, but the losses are spread, not removed"
type: investigation
date: 2026-11-06
status: complete
phase: "BUG-50 9 ms trial"
related_plan: learner-generalisation.md
related_issues: []
files_changed:
  - logbook/2026-11-06-sitting-debrief-3.md
  - logbook/INDEX.md
  - config/ODrive config Files/odrive_pro_hand_config.json
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - tests/ros/test_teensy_bridge_node_read.py
  - temp/learn/jugglebot/memory_v2.csv
  - map/bug-hand-odrive-feedback-starvation-701-ms-heartbeat-dropout.md
  - map/bug-mid-air-ball-collisions-end-half-of-two-ball-columns.md
  - map/bug-two-ball-columns-runs-end-on-a-missed-catch-of-ball-2-with.md
  - map/bug-leg-encoder-dropouts-of-80-to-99-ms-continue-at-the-clock.md
  - docs/project-map.md
  - docs/runbook.md
subsystem: [can, hand, tracking]
tags: [safety, testing]
---

# Sitting 2026-11-06 20:20 debrief

Launch log `~/.ros/log/2026-11-06-20-20-46-584215-jetson-1805214/launch.log`, bag `~/Desktop/rosbags/2026-11-06_20-20-46`.
Unit reports (gitignored): `temp/probes/sitting_debrief_2026-11-06c/unit_9ms.md`, `unit_ends.md`, `context.md`.

## What was flown

The BUG-50 row: the Hand drive's `axis0.config.can.encoder_msg_rate_ms` from 10 to 9, saved (owner) and kept. Armed hold of
1303 s with the stream running, then Columns. The check goal was four Columns goals (Apex 1.00, Dwell 0.27, Sep. 120)
instead of the Runbook's `Self-toss`; the owner amended the clause to "no Hand fault, no `Hand lead clamp ENGAGED`".
The can-bridge was not power-cycled (it needs the Jetson power-cycled; owner's choice). The BUG-56 hazard behind that
step did not arise: the launch homed all seven axes after the save (L229, L268).

| Row | Verdict | Evidence |
|---|---|---|
| Hand under 25 ms over the hold | pass | `age_max_us_6` peak 23.1 ms, 0 of 1299 windows over 25 ms |
| A Leg over 50 ms (control) | pass | all six Legs, 4 to 8 episodes each, up to 99.1 ms |
| No Hand fault or clamp in the goals | pass | 0 lines; `active_errors` 0 on all 8608 rows; `lead_clamp_mask` bit 6 0 in 14 673 samples |
| Goal 1, 10/10 | collision (BUG-74) | 71 mm, both balls kink, L944 |
| Goal 2, 5/5 | Ball 2 deflected at Cup height (BUG-78) | sideways 1.8 m/s within 30 ms, cup raw 0, L5477 |
| Goal 3, 12/12 | collision (BUG-74) | 72 mm, both balls kink, L5880 |
| Goal 4, 7/7 | Ball 2 deflected at Cup height (BUG-78) | sideways 0.3 m/s, fit 13 mm from the aim, cup raw 0, L6165 |

## Discussion

**The count did not decide it; the period did.** Zero Hand episodes in this hold has a probability of about 8 % if 9 ms
changed nothing (5 to 16 % across the reasonable variants), so the Runbook's pass rule alone proves little. The stronger
result: the Hand's frame-loss passages through the vulnerable phase came at half their 10 ms period, 17 of 17 expected
slots filled, spaced 90.5 falling to 78.4 s, while the six Legs (still at 10 ms) and the Hand's own heartbeats (still at
100 ms, 156 to 162 s) did not move. A change on one drive moved that drive's encoder recurrence by a factor of 2.0 and
nothing else, which noise, bus load, Throw load or temperature would not do. Per passage, 0 of 17 collapsed into a run of
lost frames against 5 of 16 at 10 ms (one-sided Fisher p = 0.018). One passage fell inside goal 3 during 128 to 130 rev/s
strokes: age 16.2 ms, no clamp, all caught (n = 1; the same event at 10 ms was the afternoon's fault).

**Two parts of the earlier model are withdrawn.**
- "At most every other frame is lost, worst gap about 18 ms" is not strictly true. Eleven frozen runs on the bridge clock
  show a gap over 20 ms (bound 20 to 33 ms; two consecutive lost frames, 27 ms, is the natural reading but is inference),
  all with the Hand parked and none on a main passage. They sit close to the firmware's 30 ms gate.
- "At 10 ms every frame sits at one phase, so each passage takes a run" is wrong for the Hand: 11 of its 16 passages at
  10 ms lost alternate frames, with no run. What 9 ms changes is that the two frame classes sit 1 ms apart, far wider than
  the 6 to 30 us window, so they cannot be in it together.

**The loss is spread, not removed.** 0.60 % of Hand frames are still lost (0.38 to 0.59 % at 10 ms). The owner asked whether
the dropouts could be stopped entirely: the long gaps have not recurred so far, the lost frames continue. Which side drops
the frame is still unknown.

**Why keep 9 with a 27 ms probable worst gap against a 30 ms gate.** Owner decision: 9 is better than 10, keep it. No
Hand gap has reached 40 ms in 1429 s, and the sample is short. 7 ms would give 14 to 21 ms for +43 frames/s per axis; not
taken. One thing per trial: the heartbeat and error rates stay as they are (owner took the recommendation).

**Runs of 10, 5, 12 and 7 against about 22.** 11.8 ends per 100 Throws against 4.5 earlier, P(4 or more ends) about 0.07. Nothing
ties it to 9 ms (no Hand involvement in any end; release and Apex windows unchanged) or to Sep. 120 (collisions at 71 and
72 mm match the 70 to 77 mm at every earlier Sep.). n = 4: cannot be told. Unattributed and to watch: iq per commanded
acceleration 13 % above the P2 goals, while the two 10 ms sets differ from each other by 19 %.

**An earlier line of evidence re-read.** The afternoon's report cited 14-55 log L6754, `0/80 encoder frames, encoder age
max 10 ms`, for "both lost". Those log numbers are unreliable (below), but the census confirms the conclusion: the Hand's
windows at that moment read +62 and +25 frames with age max 98 ms.

## Fix

The heartbeat-dropout log line differenced a 1 Hz census over episodes under a couple of seconds and assumed 100 frames/s,
so it printed `0/80` and `107/70` on a Hand sending 111 per s. An episode under 3 s now says it is too short for a count;
a longer one prints a measured rate with no nominal (`teensy_bridge_node.py`, `_HB_STALE_ENC_COUNT_MIN_MS`; diagnostic
only, no command or guard). `config/ODrive config Files/odrive_pro_hand_config.json` now says 9, so re-applying it keeps the setting.

## Follow-ups and owner decisions

BUG-82 filed with its Runbook row: Leg 4 at 9 ms (P(zero by chance) about 5e-4), then all Legs if it passes. It carries
the undecided items with their numbers: the Hand lead-clamp anchor and the 70 / 80 A gap. BUG-78 narrowed: every smooth
no-collision end so far (15 of 15) is Ball 2, and all 4 Ball 1 ends are collisions; a false EMPTY is out for the two ends read here.
BUG-79's Dwell check was not flown and stays on the Runbook. Three firmware scheduler STOPs in this Sitting, none in a goal
(L1329 after a 63.3 ms generation-2 collection in `teensy_bridge_node` during the hold, L6219, L6236; for BUG-58 and BUG-66,
whose claims this entry does not write to). Memory: 30 new rows from 34 caught; the other four are Feed Throws, which write no row by design.

## Verification

(2026-11-06, `./run_tests.sh`, finished 23:02: 8790 passed, 51 skipped, 4 failed, plus 7 serial passed.) All four failures were document checks on this debrief's own writes: BUG-74 and BUG-78 unrated on leaving the inbox, one over-long block in BUG-82, one bare avoid-word in this entry. Each was fixed and no code changed after that run.

(2026-11-06, `pytest -q tests/sim/test_map_items_rated.py tests/sim/test_map_items.py tests/sim/test_glossary_avoid_words.py tests/sim/test_mapkit_docs.py tests/sim/test_plans_index.py tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py tests/sim/test_sitting_debrief_protocol.py`: 226 passed, 47 skipped.)

The log-line fix is pinned by `tests/ros/test_teensy_bridge_node_read.py` (the rewritten clearing-edge test and the new `test_hb_stale_long_episode_reports_a_measured_rate_not_a_nominal`), inside the gate run above. No live hardware check of the new line yet.
