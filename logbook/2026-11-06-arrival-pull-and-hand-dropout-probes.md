---
title: "Two offline probes, 2026-11-06 evening: the two-ball arrival pull is not a planner defect (BUG-79, position half withdrawn), and the Hand encoder dropouts are a clock beat that a drive setting should stop (BUG-50)"
type: investigation
date: 2026-11-06
status: complete
phase: "learner-generalisation P2 follow-up; BUG-50 diagnosis"
related_plan: learner-generalisation.md
related_issues: []
files_changed:
  - logbook/2026-11-06-arrival-pull-and-hand-dropout-probes.md
  - logbook/2026-11-06-sitting-debrief-2.md
  - logbook/INDEX.md
  - plans/active/learner-generalisation.md
  - map/bug-hand-odrive-feedback-starvation-701-ms-heartbeat-dropout.md
  - map/bug-two-ball-throws-land-about-9-mm-toward-the-partner-site-an.md
  - map/bug-the-memory-records-the-dispatch-aim-as-the-release-position.md
  - docs/runbook.md
  - docs/project-map.md
subsystem: [motion, can, ros]
tags: [safety, dynamics, diagnosis]
---

# Two offline probes, 2026-11-06 evening

This entry follows up `logbook/2026-11-06-sitting-debrief-2.md`. Two hypotheses from that debrief were withdrawn or reframed the same day, which is the trigger for the full form. Both probes ran offline against the bags of the 14:55 to 15:53 launches. Nothing was changed on the robot, the drives or the code. The full unit reports are `temp/probes/sitting_debrief_2026-11-06b/unit_creep.md` and `unit_dropout.md` (gitignored; every number below comes from them).

## Symptoms

- BUG-79: the debrief recorded the two-ball pull toward the partner Site as half position (a commanded Carry creep of 5 to 6 mm, possibly a planner defect) and half velocity.
- BUG-50: the Hand encoder frame loss that faulted the Hand during the P2 Sitting. The owner asked: "Is it possible to stop these dropouts entirely?"

## Part A. The two-ball arrival pull (BUG-79)

### What the probe did

It planned Columns Schedules offline on the production `segments.plan_segment` with the live limits (600 mm/s, 8000 mm/s^2, 200000 mm/s^3, Hand 4500 rev/s^2, `pre_release_hold_s` 0.000), and compared them with the 98 carried Throws of goals 13 to 19 of the P2 bag. It parsed the launch log for every re-aim (RESEND, BUMP, refused RESEND) and joined each to its Memory row.

### What it found

- Offline, the planned Cup at the release knot equals the requested release to 0.00 mm in every case tried. The Platform differs from the Cup only by the tilt lever (115.7 mm at the release), 0.2 to 0.6 mm. On the bag, the 6 carried Throws that were never re-aimed sit at -0.40 +/- 0.45 mm. No planner defect.
- The offset is by design. 92 of 98 carried Throws were re-aimed by the tracker (73 RESEND, 19 BUMP), and the release follows the caught position. The catch clamp is asymmetric (10 mm outward, 35 mm inward). On a landing scatter of 24 mm SD it predicts a mean release 5.3 mm toward the partner, against 6.0 observed. The planner Banks for that release and flies the offset back.
- The "3.5 mm creep" between touch-down and release is the plan's own arrival overshoot (+3.3 mm at -0.20 s, back within 0.5 mm by -0.10 s). Referenced to the release the plan flew, the bag matches it to about 1 mm.
- Bookkeeping gap: `_register_outcome` runs once, at dispatch. A later re-aim moves `_carried_release_mm` but the pending row is not refreshed, so the Memory's `rel` holds the dispatch aim (the Site, to 0.05 mm on all 98 rows), not the release that flew. No Learner input reads it today, so no command was corrupted; the diagnosis was.
- Small defect: a refused re-send leaves `_carried_release_mm` and `_carried_catch_xy_mm` at the refused aim while the installed plan flies the old one. 23 of 98 rows had a refused re-aim. It skews the next Catch's dispatch prior, not the row.

### The reframing

Measured against the release that really flew, the position part of the pull is +1.1 mm (plain pair, 38 Throws) and +1.3 mm (turned pair, 60), about the 1.3 mm encoder lag. The velocity part is -10.7 and -11.1 mm. The plan asks for +16.6 and +13.3 mm/s of lateral launch; the plant delivers +4.2 and +0.4 mm/s. So the pull is almost entirely a lateral launch velocity the Platform does not deliver after an arrival, not a position error.

### Two fixes proposed, not applied

- F1: refresh the pending row's release and seat after each accepted re-aim (`_resend_live_catch`, `_resend_hand_corrected_catch`, the accepted branch of `commit_catch_bump`), so the row records the Throw that flew.
- F2: snapshot and restore `_carried_release_mm` and `_carried_catch_xy_mm` when a re-send is refused.

Neither changes a command by itself, so neither shrinks the pull. F2 changes the dispatch prior of that ball's next Catch by at most the refused re-aim delta (35 mm), which still passes the same clamp and the same online admissibility check. They wait for the owner's go-ahead because they change what the Memory records and the next Catch's prior. Filed as BUG-81.

### What it means for BUG-79's options

"Fix at source" no longer includes a planner fix. It means the ring after arrival (the subject of the carry-ring-zvd plan) or the lateral launch delivery. The Dwell A/B hardware check stands.

## Part B. The Hand encoder dropouts (BUG-50)

### The mechanism and its evidence

The long dropouts follow a clock beat. Each drive's free-running 10 ms encoder schedule drifts against the can-bridge's 2 ms setpoint tick. An episode starts when a drive's frame instant slides into one narrow phase of that tick (6 to 30 microseconds wide) and lasts 0.6 to 2 s while the drift crosses it. Inside an episode the surviving frames arrive about every 90 to 100 ms.

Evidence, 78 episodes in five bags of one bridge boot (uptime 50 630 to 54 919 s, FW 40, PROTOCOL 12):

| axis | slip (us/s) | predicted period (s) | episodes |
|---|---|---|---|
| 0 | 8.7 | 229 | 11 |
| 1 | 10.4 | 193 | 12 |
| 2 | 7.7 | 259 | 8 |
| 3 | 7.4 | 269 | 8 |
| 4 | 14.4 | 139 | 19 |
| 5 | 14.9 | 134 | 15 |
| 6 (Hand) | 10.0 | 200 | 5 |

- Onsets on each axis recur at whole multiples of 2000 us divided by the drive's measured slip, matched within 4 to 14 % (ratio 0.86 to 1.04). The period is the 2 ms tick, not 1 ms or 10 ms, and successive onsets step the quantised phase by 2 ms each time.
- 76 of 78 episodes came while the setpoint stream ran; the other two are the P2 fault's aftermath. Idle windows show none.
- Axes are independent: 3 windows with two axes at once, against 3.4 expected by chance.
- No CAN-priority gradient. The Hand, with the lowest priority, has the fewest episodes, and 4 of its 5 came with the Hand parked.
- A prediction confirmed after the fact: the Hand's recurrence due at log time about 1404.7 s showed as a Hand heartbeat dropout there.

### Discussion

Why a beat, not noise: noise would not recur at multiples of a period that matches each drive's own crystal slip, and would not stay gated by the stream. The convincing evidence is the two independent measures of the period agreeing per axis.

Withdrawn, each with its reason:

- Bus saturation or arbitration starvation. The bus is 62 to 75 % loaded with idle time every tick, arbitration retries and never drops, there is no ID gradient, and node 6's encoder was lost while its iq frames kept arriving.
- Bridge RX overflow, ring leak or missed FIFO service. `fifo_overflows_jb`, `fifo_warns`, `leak_hwm_jb` and `rx_cap_hits_jb` read 0 in all five bags.
- The sensor poller starving node 6. Legs 0 to 5 are never polled and hold 73 of the 78 episodes.
- An encoder read fault at high current, or EMI. Episodes fall on parked axes and take heartbeats too, with no Throw correlation.
- An intermittent electrical fault as the primary cause. Wire-error deltas were 0 across the P2 bag, and a wiring fault would not recur on per-drive drift periods.
- The August argument "zero ACK errors means the frame was never on the wire" is invalid: every node on the bus ACKs a frame, not only the bridge.

Why a drive setting over firmware as the first move: it needs no firmware and no change to the live interp ISR timing, it is reversible from the drive, and it breaks the lock rather than hiding one phase. Its cost is that the drive reboots on save and the Hand must be re-homed.

### What is not known

Which side drops the frame: the drive does not send it, or the bridge's FlexCAN does not accept it. Every bridge counter reads 0, which favours the drive side, but a hardware-silent bridge loss is not excluded. Whether the drive applies a new rate live without a save is also unverified.

### The likely fix (not applied)

Set the Hand drive's `encoder_msg_rate_ms` from 10 to 9. Nine ms is 4.5 ticks, so successive frames alternate between two phases 1 ms apart, and a window 6 to 30 us wide can catch at most every other frame. The worst gap should fall from 80 to 100 ms to about 18 ms, under the firmware's 30 ms freshness gate. Cost: 11 % more encoder frames (about +77 frames/s, about +1 % bus load). Risks: `save_configuration` reboots the drive on 0.6.11 (reboot the bridge afterwards, re-home the Hand); host diagnostics that assume 100 encoder frames/s would read about 11 % surplus; the firmware's age-based staleness logic was not checked line by line. The same reasoning applies to `error_msg_rate_ms` (20 to 19 or 21) and `heartbeat_msg_rate_ms` (100 to 99).

Firmware alternative (owner decision): dither the start of the bridge's 7-frame burst by about 0 to 200 us per tick. It covers every message and drive without a drive save, at up to 0.2 ms of setpoint jitter (about 1 A of transient on the Hand by the shipped gains) and a change to the live ISR, so it needs an offline replay and an owner ruling. The Hand lead-clamp anchor decision stays open as defence in depth.

### The confirming test and what would refute the model

Hand at 9 ms, parked, setpoint stream running at least 20 minutes (six or more Hand periods). Prediction: `/cache_diag` `age_max_us_6` stays under about 25 ms while Legs 0 to 5 keep their periodic episodes over 50 ms as the in-run control. One Hand episode over 50 ms refutes the model.

### Before "entirely" can be claimed

The beat is the trigger, but removing every single lost frame depends on which side drops it. That needs a passive second receiver (a USB-CAN adapter, or the Platform Teensy with a sniff build) logging per-ID timestamps beside the bridge census through a few episodes. Frames the sniffer sees and the bridge misses mean the bridge side; frames neither sees mean the drive side, which only an odd rate or dither can make harmless. The answer to the owner is therefore: the harmful dropouts can very probably be stopped, and "entirely" is not yet shown.

### The 89 short episodes

The 89 short (30 to 47 ms) episodes the host detector counted on the bag replay are almost all artefacts of the bag's republished `/robot_state`, which is missing 12 to 15 % of samples in the large bags: of 99 Hand frozen-value gaps of 30 ms or more, 94 span a missing sample. The firmware census shows 0 of 838 windows with a Hand cache age between 30 and 50 ms. The live detector reads the bridge's Telemetry callback per UDP frame, not that topic, so it should print fewer lines live than the 7 the replay gave. That is inference, not yet seen live.

### Owner answers, 2026-11-06 evening

The Hand drive's hard current limit is 80 A (correct as found). The Hand was up, at the top, at the fault. No change to the wiring.

## Fix

None applied. Documentation only. Nothing changed on the robot, the drives, the code or the Memory.

## Verification

This entry lands with documentation only (the plan trim, the Items and the regenerated pages ride in the same commit). Doc tests below.

- 2026-11-06 19:04, on the whole change set after the audit's fixes, `python -m pytest tests/sim/test_plans_index.py tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py tests/sim/test_glossary_avoid_words.py tests/sim/test_sitting_debrief_protocol.py tests/sim/test_sitting_sections.py tests/sim/test_night_shift_gate.py -q`: 159 passed; `python -m pytest tests/sim -k "mapkit or map_items" -q`: 310 passed, 49 skipped.
- 2026-11-06, `python -m pytest tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py tests/sim/test_glossary_avoid_words.py tests/sim/test_map_items.py tests/sim/test_map_items_rated.py -q -p no:cacheprovider`: 176 passed, 49 skipped in 3.34 s.
- The two probes are offline reads of existing bags; their scripts and outputs are in the gitignored `temp/probes/sitting_debrief_2026-11-06b/`.

## Open

- Owner go-ahead for F1 and F2 (BUG-81).
- The drive-setting trial and the passive-sniffer measurement for BUG-50.
- The velocity shortfall after an arrival (the real pull): cheapest next check is an offline per-row regression of ball lateral velocity on planned velocity, encoder velocity at release and measured tilt, split by RESEND and BUMP.
