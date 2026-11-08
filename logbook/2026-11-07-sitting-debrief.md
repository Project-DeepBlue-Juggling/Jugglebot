---
title: "Sitting 2026-11-07 16:15 debrief: Legs 1 and 2 separated three times (the third unnoticed), no planned Leg peak tells safe transits from unsafe ones, and the third re-seat left the Platform tilted enough to explain every hop miss"
type: investigation
date: 2026-11-07
status: complete
phase: "Sitting debrief"
related_plan: pattern-progression.md
related_issues: []
files_changed:
  - logbook/2026-11-07-sitting-debrief.md
  - logbook/INDEX.md
  - map/bug-two-ball-throws-land-about-9-mm-toward-the-partner-site-an.md
  - map/bug-legs-1-and-2-left-their-magnetic-base-joints-on-columns-at.md
  - map/bug-leg-encoder-dropouts-of-80-to-99-ms-continue-at-the-clock.md
  - map/bug-ball-2-s-first-throw-after-the-feed-catch-lands-about-180-mm.md
  - map/bug-two-ball-columns-runs-end-on-a-missed-catch-of-ball-2-with.md
  - map/bug-mid-air-ball-collisions-end-half-of-two-ball-columns.md
  - map/bug-catch-re-aim-refused-infeasible-on-8-4-of-throws-at-the-p2.md
  - map/bug-aborted-no-release-three-goals-ended-because-the-throw-never.md
  - map/bug-nomination-s-re-aim-figure-read-0-049-while-the-live-feed.md
  - docs/project-map.md
  - docs/runbook.md
subsystem: [motion, hand, tracking]
tags: [safety, dynamics, testing]
---

# Sitting 2026-11-07 16:15 debrief

Launch log `~/.ros/log/2026-11-07-16-15-21-645789-jetson-2503132/launch.log` (L = line), bag `~/Desktop/rosbags/2026-11-07_16-15-21`.
Clock: times like 856.52 are 1794028000 + seconds (856.52 is 16:20:56.5). Unit reports and scripts (gitignored):
`temp/probes/sitting_debrief_2026-11-07/` (`verdicts.md` is the table the owner confirmed; `unit_*.md`; `joint.py`,
`sep2.py`, `transit.py`, `tilt.py`, `pull.py`, `enc_age.py`). The Hand ran at `encoder_msg_rate_ms` 9 and so did Leg 4 (owner).

## What was flown

48 Attempts in 40 minutes: Columns at Apex 1.10 (4), Apex 0.93 at Dwell 0.27 (2), 0.32 (4) and 0.30 (3); turned Columns
at 45 deg (11, three Memory plants) and 90 deg (13, plant `columns90-2`); high-low at Sep. 100 (3) and 125 (4); hop at
Sep. 250 (4). Ends: 35 ball-caused (7 collisions, 19 Landings beyond the Catch authority, 4 inside reach with the Cup
empty, 4 other, 1 cannot tell), 6 NO_ADMISSIBLE_COMMAND, 2 LANE_REFUSED, 1 LIMIT_JERK, 1 ABORTED_MODE_CHANGED (Hand
SPINOUT), 1 REJECTED_BB (Ball Butler empty), 2 COMPLETED.

| Row | Typed | Verdict (owner confirmed) | Evidence |
|---|---|---|---|
| BUG-57 | pass 16:19 | pass; Item kept open (owner) | 4 Attempts at Apex 1.10, 0 SPINOUT. At 639.57 the Hand stopped at 1.97 rev while its command went on to 0.04 rev, setpoint -69.9 A (measured -72.3) for about 0.12 s, past the row's 66 A stop; cause unknown. Off-row SPINOUT L8157 below |
| BUG-79 | fail 16:26 | fail; row off the Runbook (owner) | Separations at 856.52 and 1028.45; no answer to the Dwell question (pull n 34 / 5 / 4) |
| IDEA-11 | fail 16:42 | fail | 45 deg best 20/20 (L9950); 90 deg best 9/9 in 13 Attempts; `columns90` flew at 45 deg (L10458) |
| BUG-58 | pending 16:46 | pass, literal; row stays a ride-along | L7727 and L14166 with GC_PROBE from both nodes within 0.6 s (n=0) and full WARNs |
| IDEA-10 | no box | fail (owner accepted the log) | Sep. 100: 11/11, best 5/5, Apex difference 75 mm; the third separation was at Sep. 125 |
| BUG-63 | none | not flown | data only, below |
| BUG-82 and other rows | none | not flown | |

## Findings

**Three separations, not two.** Encoder forward kinematics against the Platform mocap body, expressed in the Base body's
frame, sits at 1 to 2 mm median in every other Attempt and leaves it at 856.52 (Dwell 0.32, at x -70, y -26),
1028.45 (Dwell 0.30, x +63) and 2403.56 (high-low Sep. 125, L15545, x +84, y -33). The owner had not seen the third
("one of the legs had separated without me realising"). Each came 0.02 to 0.14 s after a transit's arrival.

**The third re-seat left the Platform off, and that explains the hop.** Platform in the Base frame at rest, rotation
vector: (0.05, -0.45, 0.66) deg before (the value at every earlier rest, re-seats 1 and 2 included), (0.04, 0.85, 3.05)
after it, (-0.7 to -0.95, 1.19 to 1.30, 3.4) at the hop Site, and about 15 mm off (frame check +15.0 to +16.9 mm from
L17228). That tilt at 4.0 m/s over 0.87 s predicts a Landing about +103 mm in x and +52 in y; the four hop first Throws
landed +112 to +133 and +52 to +70 (L17571, L17698, L17818, L17993). The frame check reads position only, so it passed.
The owner is re-seating the Platform now.

**NO_ADMISSIBLE_COMMAND was the Feed, not the joints.** Ball 2's first Throw flew in all five refused Attempts (L4262,
L4390, ...); the refused Segment is the next one, the Catch of Ball 1 (skill 2). In all five the Feed Catch aim was
clamped to -30 mm laterally (fits -31 to -54 mm) and the margin read -0.208, -0.155, -0.074, -0.034, -0.029; both first
Attempts had an unclamped Feed and planned +0.116 and +0.054. The same clamp costs Dwell 0.27 about 0.11 (0.294 to
0.181), where there is room. Feed aims were clamped in 25 of 32 Attempts. The refusals are a planner result on a
planner input; the Platform sat within 2 mm of its encoder pose in those Attempts.

**The Base walks.** The Base body moved from (3.3, 34.2) mm at L625 to (-53.3, -15.4) at L15716 in 27 minutes, in 16
logged moves, rigidly with the machine (Platform-in-Base stays under 2 mm, so the coordinator's world-frame "resting
residual walking 0.8 to 9.9 mm" was the Base, not the joints). Per Attempt: Apex 1.10 0.5 to 2.6 mm; Apex 0.93 Dwell
0.27 4.4 and 5.0; turned 45 up to 9.4 (L6351, a 9/9 Attempt, no separation); high-low up to 7.6. At the second
separation the Base stepped about 11.5 mm and 1.75 deg at 1029.5 to 1030.0, inside that Attempt, 1.0 s after the joints let
go; the frame check after it was unchanged, so the Base moved rather than its markers. The owner did not expect 10 mm
in one Attempt; the data show 9.4 mm without a separation and 11.5 mm after one. It bounds a Landing bias within one
Attempt by that step (small against 17 to 24 mm sd) and does not explain the separations. The machine was also moved
201 mm at 16:52:15, 141 mm back at 16:52:57 and 216 mm at 16:54:54, during and after the third re-seat.

**Other ends.** L14166: a Hand C1 stop, then MAX_DEVIATION +5.779 rev 11 ms later with the Hand still at 2.21 rev: the
streamed target walked away from a held Hand; the hold followed 113 ms after the STOP. L7727: a firmware STOP
(`refused +4`) in a Feed, no latch. Downlink ledger lost 0 of 83,340 setpoints, so FEAT-41's queue fix holds; only 3 of
23 STOPs have a host signature. L8157 SPINOUT (turned 45, Apex 0.95): the brake fell away from about 87 rev/s at 8.93
rev to 58.6 rev/s at 10.74 rev (clip 10.501) at -36 A against a 70 A limit, no STOP or C1 stop first, Hand rest 0.061 rev
before and after. L17420 LIMIT_JERK: install gate 203,074 against 200,000 mm/s^3, and L17560 at 200,003. BUG-74: 7
collisions in about 300 Throws. BUG-33: L2489 is a missed Catch reported as no release. BUG-75: 24 of 282 Throws (8.5 %),
turned 15 %. BUG-63 (unflown): Ball 2's first Throw landed within 45 mm in all 32 Attempts, clamped Feed +4.2 mm outward
(n 25), unclamped +9.6 (n 7). Turned 90 deg: 7 of 9 Ball 2 ends landed 21 to 76 mm outward against 10 mm authority,
scatter rather than bias (caught Ball 2 mean +5 mm, sd 20, n 13), so PWAY-2 P2(b) stays open. Owner: the Cup rim is 85
mm in diameter; at 90 deg every strike hit the Platform top (the top of the Hand's stroke), elsewhere some misses hit
the Hand and some other Platform parts. Encoder ages over the whole launch: Leg 4 peak 17.1 ms and Hand 18.1 ms, 0 of
2396 windows over 25 ms each; Legs 0 to 3 and 5 peak 99 to 100 ms with 20 to 30 windows over 25 ms.

## Discussion

**The Runbook row's premise was wrong, and the metric behind it does not work.** The BUG-79 row said both Dwells were
"inside what has flown": 353 mm/s and 5180 mm/s^2 planned at Dwell 0.32 against 371 and 5488 flown. The first pair was
planned at nominal aims and the second was a live install peak that includes the Catch re-aim, so they were not
comparable; the same Sitting's Dwell 0.27 planned 310 / 4076 and flew 350 to 383 / 4912 to 5120, and Dwell 0.32 flew
404 / 5872. The deeper problem is that no planned peak separates the events:

| Attempt | Plan Leg mm/s, mm/s^2 | Transits | Separated | Worst transit: executed command accel Leg 1 / 2 (mm/s^2) | Est. tension L1 / L2 (N) |
|---|---|---|---|---|---|
| L3265 Dwell 0.27 | 383, 5120 | 19 | no | 4497 / 4113 | -49 / -46 |
| L3814 Dwell 0.32 | 404, 5872 | 4 | yes | 8974 / 9633 | -52 / -41 |
| L4827 Dwell 0.30 | 406, 5968 | 3 | yes | 9737 / 6873 | -54 / -42 |
| L15097 high-low 125 | 403, 5936 | 12 | no | 8316 / 4447 | -53 / -34 |
| L15545 high-low 125 | 417, 6792 | 3 | yes | 10789 / 9617 | -47 / -49 |
| L14789 high-low 125 | 430, 7104 | 3 | no (ended 1 s later) | 11925 / 11566 | -55 / -40 |
| 2026-11-06 Sep. 170 | 486, 6296 | 2 | yes | 14450 / 13696 | -59 / -49 |

5872 separated after 4 transits and 5936 flew 12; 7104 held for 3. Estimated tension and lead-clamp tick counts do not
separate either. What the four events share: a transit whose executed command reached 8974 to 14450 mm/s^2 on Leg 1
(far above the plan; earlier work reads it as the lead clamp releasing), then the joints let go at a later arrival 0.5
to 1.2 s on. That supports BUG-72's "the first transit unseats, the second finishes" over "one long fast arrival". It is
n = 4 against L14789 (ended too early to count), L15097 (8316 flew on) and a clean 6685 / 9141 on 2026-11-06: a lead,
not a bound. The rehearsal should have used one metric on both sides (at the re-aim corners), carried the plan through
the bridge's feedforward cap and lead clamp to the executed command on Legs 1 and 2, and said that nothing yet separates
the events instead of "inside what has flown". The owner took the row off the Runbook and plans a hardware modification
against separation first; no bound (BUG-72 decision stands, TKT-52 holds the options).

**Two readings withdrawn.** "Every later Attempt was refused because the joints had let go" (the owner's note read that
way, and the timing invited it) does not survive the Feed table above: the planner does not read joint state, the
Platform was attached in those Attempts, and the refusals track the Feed clamp one for one. "The resting residual walks
because the joints creep" was the Base moving under a world-frame fit.

**What BUG-79 learnt.** Nothing about its question: the long Dwells gave 5 and 4 rows, mostly first Throws from rest,
which cannot test the time from arrival to release (pull toward the partner +13.4 mm, sd 22.6, n 34 at Dwell 0.27;
about +12 at both long Dwells). A redesign needs a Dwell contrast that stays inside what has flown on the executed
command.

## Owner decisions listed, none made

- LIMIT_JERK install tolerance: 203,074 and 200,003 refused against 200,000 mm/s^3 (L17420, L17560).
- The hold after a Hand C1 stop arrives 113 ms after the STOP; MAX_DEVIATION latched in 11 ms (L14119 to L14136).
- The Feed lateral clamp costs about 0.11 of margin at Dwell 0.27 and turns Dwell 0.30 / 0.32 negative (above).
- A print-only tilt and yaw line in the frame check (no gate): it would have shown the third re-seat at 16:53.

## Follow-ups and Item writes

Progress notes on BUG-79 (with its `## Sitting` section replaced by a note: off the Runbook until redesigned), BUG-72
(two notes), BUG-82, BUG-63, BUG-78, BUG-74, BUG-75, BUG-33. BUG-87 filed: the nomination's re-aim figure read -0.049
while the live Feed-Catch Segment was refused at -0.208. Four Items are claimed by other owners and `set-status`
refused them without `--force`, so their notes live here only:
- BUG-57 (jugglebot-3d): owner keeps it open; the -69.9 A blocked downstroke at 639.57 and the L8157 SPINOUT above.
- BUG-58 (jugglebot-50): row passed literally (L7727, L14166, GC_PROBE within 0.6 s, n=0); both STOPs host-clean, lost 0
  of 83,340; row stays a ride-along.
- IDEA-11 (jugglebot-f5): fail as above; PWAY-2 P2(b) still open; owner's 85 mm rim and Platform-top note.
- IDEA-10 (jugglebot-f5): fail as above, owner accepted the log; Legs 1 and 2 separated in its third Sep. 125 Attempt.

BUG-82's pass clause, read literally: the row (a 20-minute armed hold with no Juggle request) was not flown. The in-run
numbers would meet its first two clauses (Leg 4 never over 25 ms; other Legs over 50 ms), the spacing clause was not
measured: unverified, status and row left to the owner. No merges (no held branch for the passed rows), no rebuild
(nothing under `ros_ws/` changed), `docs/records.md` unchanged, Memory already committed (eee4323d).

## Verification

(2026-11-08 11:15, `pytest -q tests/sim/test_map_items.py tests/sim/test_map_items_rated.py tests/sim/test_glossary_avoid_words.py tests/sim/test_plans_index.py tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py tests/sim/test_sitting_debrief_protocol.py tests/sim/test_sitting_sections.py tests/sim/test_mapkit*.py`: 447 passed, 40 skipped.) Tier 3 (documentation and map files only, no code): these are the tests that read `map/`, `logbook/`, the Runbook and the generated pages. The first run caught an over-long block in BUG-87's body, restructured before this run.
