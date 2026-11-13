---
id: BUG-87
kind: bug
title: Nomination's re-aim figure read -0.049 while the live Feed-Catch Segment was refused at -0.208 (Dwell 0.32)
status: backlog
created: '2026-11-08'
updated: '2026-11-10T18:31:38Z'
owners: []
priority: medium
effort: m
hardware: none
owner: none
radius: local
diagnosis: hypothesis
area: skill-stack
blocked_by: []
links:
- logbook/2026-11-07-sitting-debrief.md
- map/bug-two-ball-throws-land-about-9-mm-toward-the-partner-site-an.md
filed_by: claude
---

At the Sitting of 2026-11-07 (Columns Apex 0.93 Sep. 125) the goal was accepted at nominal margin 0.189 and the printed re-aim figure was -0.049.

## What flew
- Live, the Segment after Ball 2's first Throw (the Catch of Ball 1, skill 2) was refused at -0.208, -0.155, -0.074 (Dwell 0.32) and -0.034, -0.029 (Dwell 0.30).
- Each refusal followed a Feed Catch whose lateral aim was clamped to -30 mm. The two Attempts with an unclamped Feed planned +0.116 and +0.054.
- So the corner walk in `nominate.py` (`_AimShifts`) reads above the live Feed case.

## Not settled
- Whether the fed schedule's first Segments are walked at all, or whether the splice timing of a late Feed re-aim adds the rest.
- Figure only: acceptance never uses it, so this is about what the operator is told, not a guard.

## Findings (2026-11-09)

Night Shift, one unit, offline replay with `nominate.py` (throwaway probes, deleted). Columns Apex 0.93, Sep. 125, Dwell 0.32, hop entry, launch-default limits. **Not settled; no code changed.**

### What reproduces
- The printed -0.049 reproduces exactly: across sign -1, Segment idx 2 (the Catch of Ball 1, then its Throw). The other across sign reads +0.061, and the nominal is 0.189, as in the log.
- The fed Segments are walked: idx 0 (hop-entry Throw), idx 1 (the Feed Catch and its folded Throw) and idx 2. The "not walked at all" reading in this Item is refuted.

### What does not reproduce
| Input | Offline margin | Live margin |
|---|---|---|
| Feed clamped at (-10, -30), `u` (+15.3, +10.0), Apex 0.862 | +0.181 | -0.208 |
| Feed unclamped (+11.4, +5.3), `u` (+11.3, +10.0) | +0.198 | +0.116 |

- The clamp costs about 0.32 live and about 0.02 offline.
- A grid over Feed and second-Catch corners at both Dwells never reads below -0.049.
- Feed landing time (+-40 ms) and arrival velocity (+-200 mm/s) move idx 2 by nothing at 3 decimals: it depends only on the Feed's release state.

### Lead (not proven)
- The walk seems the wrong place to fix this. The gap looks to be in how the live executor builds the Feed Catch's release state, which the next Segment is seeded from. The log shows the Feed spliced at Knot 17 after a rebased Fresh-origin.

### Next
- A bag-reading unit: take skill 1's live release state from the 2026-11-07 16:15 bag, plan skill 2 from it, compare with the nominal idx 1 release state. Effort rated up to m for that reason.

## Findings (2026-11-10)

Night Shift research unit, offline, no code changed. Columns Apex 0.93, Sep. 125, Dwell 0.32, hop entry, launch-default limits, the production planner via `nominate.plan_nominal` with a wrapper on `segments.plan_segment` (throwaway probes in /tmp, not committed).

### Sources
- Bag `/home/jetson/Desktop/rosbags/2026-11-07_16-15-21` (mcap): lists topics, but holds no plan record. The refused check (log line 4241, 896.41) runs 0.66 s BEFORE the Feed's folded Throw releases (about 897.07), so no live release state exists in the bag to read.
- `launch.log` lines 4197-4247 (the -0.208 / -0.165 Attempt) and 1010-1075 (an accepted Attempt, for timing).
- Files at origin/main: `motion/skills/{nominate,executor,segments}.py`, `motion/unified_cycle.py`.

### Verdict
- **Refuted** for the lead as stated ("the live Feed release state differs from the walk's and accounts for most of -0.208").
- **Not found** what does account for it: the log of the refused Attempt does not carry skill 2's catch landing time or position.
- Best surviving candidate: skill 2's OWN catch inputs (landing time and xy), not the Feed's release.

### Evidence
- Same function. The check seeds skill 2 with `uc.release_state_at_knot(record.plan, record.meta, k_rel)` (`executor._segment_origin`, snap in `splice_knot_for`). The walk seeds with `Segment.release_state` = `release_state_from_meta`, which calls that same function (`segments._plan_catch_throw`).
- Same numbers. On the walk's Feed segment (clamped (-72.5, -30), live u (-13.0, +8.7), Apex 0.856), `seg.release_state` and `release_state_at_knot(seg.plan, seg.meta, 24)` differ by exactly 0.0 in pose, velocities, hand, cup state and detach axis.
- Live inputs rebuilt from the log (skill 0 Apex 0.862 and u (-6.8, +3.0); Feed landing (-72.5, -30.0, 830.1); Feed u (-13.0, +8.7) at Apex 0.856; fold released at the caught xy, as `_catch_terminal` does) give idx 2 = +0.102, not -0.208.
- Bounding the Feed: 32 corner cases (Feed aim (-10 or +35, +-30), Feed u (+-30, +-30), Apex 0.80 or 0.93) move idx 2 over -0.013 .. +0.452. The worst is the clamped Feed with u opposed across, which the walk already covers. No Feed state inside the authority box reaches -0.2.
- Idx 2's own inputs do reach it (table below, Feed held at the live values).

| Skill 2 catch input (Feed fixed at live) | idx 2 margin |
|---|---|
| nominal landing and time | +0.102 |
| landing 35 mm off at the sitting edge, time nominal | -0.17 to -0.23 (LIMIT_ACC) |
| landing 0 mm off, 20 ms earlier | -0.17 (LIMIT_ACC) |
| 35 mm off, 30 ms earlier | -0.28 (LIMIT_JERK) |
| 5 ms earlier, nominal xy | -0.001 |

- The margin falls off a cliff with the catch time: 5 ms earlier takes +0.102 to about 0, 30 ms gives INFEASIBLE or HAND_LIMIT_ACC. The seed-to-catch transit shrinks, which is the jerk / acceleration the log reports.

### What differs between the walk and the live skill 2 (not proven to be the cause)
- The walk keeps the hop-entry Throw at the pattern flight (`_cmd_frac` bypass, `plan_nominal` idx 0). Live, `_hop_command` flew it at Apex 0.862 (the log: "apex 0.930 -> 0.862"), 32 ms shorter (`schedule.flight_s`: 0.8386 s vs 0.8710 s), so Ball 1 arrives earlier than the walk's idx 2 `t_land`.
- Whether the live prior uses that earlier time is open: in the accepted Attempt the logged t_land (632.786) sits 0.946 s after the release, matching `flight_s(1.1)` = 0.947 s, not the commanded Apex 1.017 (0.911 s). So this candidate is weakened, not excluded (a tracker re-aim after 896.52 was not visible).
- Smaller: the live THROW's event is at 0.425 s (knot 17), the walk's at 0.400 s (knot 16, `sk.window_s`); the folded Throw's fly-back shift (`_offset_flyback_mm`) is about (-0.8, -2.4) mm and is not in the walk. Both are far below 0.3 of margin.

### Not done
- No live release state read: it does not exist in the bag for the refused Attempts (see Sources).
- Did not reproduce -0.208 exactly; skill 2's real prior landing (time, xy, velocity) at the check is not in the log.
- Did not run the executor's `check_segment` on a rebuilt `PlanRecord`; only the seed function was compared.
- Did not cover the Dwell 0.30 pair or the +0.116 / +0.054 unclamped Attempts.

### Next
- Log (or have `skill_node` print at the check) skill 2's candidate landing: `pos_mm`, `t_land_abs_s`, aim source, for every `ADMISSIBILITY` line. One DEBUG line answers this.
- Offline, rebuild the live check for skill 2: `check_segment` on the Feed's real `PlanRecord` with `_prior_landing`, and compare its terminal field by field with `_landing` in `plan_nominal`.
- If the 32 ms hop-Apex difference is it, a fix touches `nominate._landing` / `plan_nominal` (walk the hop Throw at its `_hop_command` flight, as `executor._predicted_landing` does), not the Feed seed. No code written here.

## Progress

- 2026-11-08: 2026-11-08 owner decision: fix this before anything else on the Feed clamp. The clamped Feed Catch aim (25 of 32 Attempts on 2026-11-07) costs about 0.11 of margin; above Dwell 0.27 it refused every later Attempt. Then check whether Feed misses lean one way (columns_feed_bb_bias_mm). Acceptance stays nominal.
- 2026-11-09: Night Shift 2026-11-09: -0.049 reproduces and the fed Segments are walked; the live -0.208 does not, so no code changed; next is a bag read of the Feed's release state
- 2026-11-10: Night Shift 2026-11-10: the Feed-seed lead is refuted (walk and check seed alike); skill 2's own Catch time is the lever; next is one log line at the check
- 2026-11-10: Night Shift 2026-11-10: merged the ADMISSIBILITY-LANDING log line; after the next ros_ws rebuild, a refused fed Segment shows the Catch landing its check used
- 2026-11-11: Night Shift 2026-11-11: ADMISSIBILITY-LANDING is live in the 2026-11-10 bags (316 and 185 lines); no fed Segment was refused (skill 2 margins 0.40 to 0.41); waits on a refused one
