---
id: BUG-75
kind: bug
title: Catch re-aim refused INFEASIBLE on 8.4 % of Throws at the P2 Sitting, 15 of 69 on the turned pair
status: backlog
created: '2026-11-06'
updated: '2026-11-10T18:01:34Z'
owners: []
priority: medium
effort: s
hardware: robot
owner: hands
radius: local
diagnosis: confirmed
area: skill-stack
blocked_by: []
links:
- logbook/2026-11-06-sitting-debrief-2.md
filed_by: claude
---

`RESEND-REFUSED INFEASIBLE ... QP infeasible: unbounded dual step` on 29 of 346 Throws at the Sitting of 2026-11-06 15:53, against 2.3 % at the 13:19 Sitting and 0 to 0.4 % in the endurance logs the same afternoon.

## Findings

- Concentrated on `columns_turned` at 90 degrees (15 of 69 Throws).
- The one-ball goal at Apex 0.70 refused 12 of 20 as WINDOW_TOO_SHORT and one at Apex 1.00 refused 8 as SPLICE_TOO_LATE.
- The refused Throws did not land or seat worse, and none preceded a miss in the one-ball goals; the Hand stays on the earlier aim.
- The reason string exists since 2026-09-22.

## Open questions

Not diagnosed: why the turned pair's re-aim QP is infeasible more often.

## Evidence

`temp/probes/sitting_debrief_2026-11-06b/unit_stack.md` section 5 (P2 log L657, L879, L1114, L7428).

## Findings (2026-11-07)

Source: launch.log of the 15:53 Sitting (`~/.ros/log/2026-11-06-15-53-05-334499-jetson-1580225/launch.log`), parsed by a throwaway script (deleted). The bag was not needed.

### Verdict
- True infeasibility of a LATE SECOND re-aim, not a planner defect and not a turned-pair property.
- Harmless by design: the Hand stays on the first, accepted re-aim. Rate low.

### What the log shows (measured, n = 29 refusals)
- All 29 INFEASIBLE refusals (14 one-ball, 15 turned) are the SECOND ask of a Catch whose first re-aim was accepted. None is a first ask.
- The ask itself does not separate them. Refused: 10.1 to 25.5 mm, median 12.8 to 13.7. Accepted second asks: 10.0 to 32.7 mm, median 14.6.
- Time shift is about 0 in all (-0.005 to 0.000 s). Ball does not separate them: 9 Ball 1, 6 Ball 2 on the turned pair.
- Direction in the Robot frame is not in the log line, so it was not tested (gap).

### The variable that does separate: time left after the splice
| Set | n | splice to landing, min / median / max |
|---|---|---|
| One-ball, second ask refused | 14 | 0.116 / 0.151 / 0.176 s |
| Turned, second ask refused | 15 | 0.109 / 0.161 / 0.172 s |
| One-ball, second ask accepted | 126 | 0.113 / 0.341 / 0.418 s |
| Turned, second ask accepted | 2 | 0.236 / 0.287 / 0.338 s |

- Refused with 0.18 s or more of window: 0 of 29.
- Accepted with under 0.18 s: 6 of 128, so it is a band, not a clean threshold.

### Why the turned pair refuses more often
- Its FIRST ask comes later: window to landing median 0.324 s (n=70), against 0.459 s (n=263) for one-ball and straight columns.
- A second ask, one resend interval later, therefore falls in the 0.11 to 0.18 s band.
- Turned: 17 second asks, 15 refused. One-ball: 140 second asks, 14 refused. Straight `columns` (4 goals): 51 first asks, 0 second asks, 0 refusals.
- Why the turned first ask is about 0.13 s later is a HYPOTHESIS (Fit convergence on the turned geometry). Not tested.

### Binding row (hypothesis, not replayed)
- The cup-contact floor `CUP_CONTACT_ACC` applies in the last 0.125 s before landing (`executor.py` lines 282-294, `hw.JB_TRAJ_CUP_CONTACT_WINDOW_LEAD_S`).
- A splice 0.11 to 0.18 s out leaves 0 to 0.05 s of runway to reach that floor from a state already moving on the first re-aim. The executor comment says this solve can only refuse.
- Inequality indices recur (253 x9, 238 x7, 223 x3), consistent with one block at a similar window length. Not named: no row-label helper exists in `cup_cycle.py`, and a replay did not fit the budget.
- The pre-solve CONTACT-WINDOW fence tests the estimated splice instant, so asks just inside the band still reach the QP.

### Effect on Catches (approximate)
- Throws following a refusal: 22 of 22 CAUGHT, landing radius median 15.3 mm, max 55.4 mm.
- Other throws with a summary line: 62 of 62 CAUGHT, median 19.5 mm, max 64.0 mm.
- Attribution of summary line to refusal is by log order, so treat the numbers as indicative.

### Rating and not done
- Rating: effort small, hardware none, diagnosis done to the window variable; binding row unnamed.
- No limit change proposed. A cheaper route, if the 8 % refusal count is ever a nuisance, is to decline in the pre-solve fence (no cap count, no solve cost). That is a design choice for the owner, not made here.
- Not done: Robot-frame direction of the ask versus the turned axis, a QP replay.

## Findings (2026-11-08)

Sources: bag `/home/jetson/Desktop/rosbags/2026-11-06_15-53-05` (`/balls`: per-Ball throw_time, first confirmed, first `landing_from_fit`) and the 15:53 `launch.log`. Code read at origin/main (`executor.py` `_resend_live_catch`, `_resend_splice_instant`). Probe scripts deleted.

### Verdict
- The premise does not hold in this bag. The turned pair's first ask is NOT later than plain Columns: ask to landing is 0.628 s against 0.633 s, and the splice to landing is 0.317 s for both.
- The 0.324 against 0.459 s gap compares turned with one-ball self_toss. It is a splice-rule difference (pinned versus raw), not Fit convergence.

### Stages per Catch of a Hand-thrown Ball (seconds, medians)
| Stage | Columns (n) | Turned (n) |
|---|---|---|
| (a) release to Tracker confirm | 0.046 (59) | 0.048 (72) |
| (b) release to first converged Fit | 0.161 (59) | 0.161 (72) |
| catch dispatched, after release | 0.133 (59) | 0.134 (72) |
| (c) first Fit to first ask | 0.106 (50) | 0.115 (70) |
| (d) release to first ask | 0.265 (50) | 0.273 (70) |
| first ask to landing | 0.633 (50) | 0.626 (70) |
| release to landing (flight) | 0.904 (59) | 0.903 (72) |

- Turned minus Columns is +0.008 s at (d), not +0.13 s. The spreads overlap (p25 to p75: 0.244 to 0.288 against 0.262 to 0.292).
- Flight is identical, so there is no Apex or Dwell difference to normalise.
- Ball 1 against Ball 2 turned: 35 each, first-ask offset median 20.0 against 19.2 mm. No Ball effect.

### Why the splice window differs from one-ball
- A first ask while the Hand's other Throw is still ahead splices PINNED at that release (`_resend_splice_instant`). Splice to landing is then `t_land - t_rel`.
- Measured `t_land - t_rel`: Columns 0.317 s (n=57), turned 0.317 s (n=72). First asks pinned: 46 of 46 and 67 of 70.
- Self_toss (one ball) has no such release, so it splices raw at ask + 0.225 s: first ask to landing 0.701 s (n=221), splice about 0.476 s.
- So 0.317 against 0.459 is the two-ball beat (release 0.575 s after the Ball's own release), a property of the schedule.

### What does differ: second asks
| Set | Catches with a first ask | Second asks | Refused |
|---|---|---|---|
| Columns | 51 | 3 | 3 |
| Turned | 72 | 27 | 25 |

- The refused count covers every code. Launch log, turned: INFEASIBLE 15, WINDOW_TOO_SHORT 8, LIMIT_JERK 2, SINGULAR 1, UNVERIFIED 1; 27 lines, of which 2 are first asks. Columns: LIMIT_JERK 3. Recounted by the orchestrator.
- The second ask is never pinned: its splice is ask + 0.225 s, leaving 0.167 s median (turned, n=27), in the refusing band.
- Why turned asks again more often is NOT explained. Hypothesis only: after the first re-aim the Fit still drifts more than the 10 mm tolerance in the turned frame.
- Tested and ruled out: first ask at the 30 mm authority clamp (17 of 50 against 17 of 70), Ball (15 and 12 of 35), first-ask size (2-ask 19.2 mm against 1-ask 20.0 mm).
- Fit drift first to last, x median 27.1 against 24.9 mm. It does not separate them.
- Gap: the bag does not say why the unsaturated turned Catches (19 of 53, against 3 of 33) ask twice. The Fit drift is in the tracker frame and the aim is in the Robot frame, so I did not trust a per-axis comparison.

### Cost in Catch authority (measured, log summary lines)
| Turned, by re-aim outcome | n | Landing radius median, max |
|---|---|---|
| One re-aim, no second ask | 40 | 25.0, 64.6 mm |
| Second ask refused | 26 | 25.1, 41.2 mm |
| Two re-aims accepted | 2 | 27.5, 44.1 mm |

- All 68 caught. No measurable cost: a refused second ask leaves the Hand on the accepted first re-aim, which lands as well as one that never asked.
- Columns: one re-aim 31.0 mm (n=26 caught), two re-aims 27.5 mm (n=23). A 3.5 mm median, small and unproven.

### Recommendation
- Change nothing in timing or limits. The "later first ask" is not real, and the refusals are free.
- Correct BUG-75's wording: the turned pair is not later, it asks twice more often. If the count is a nuisance, the cheap route is the pre-solve fence (no cap count). I do not propose it; the control consequence at 40 Hz is not walked here and no cost justifies it.

### Rating and not done
- Rating: effort small, hardware none, diagnosis done for the premise; the cause of the extra second asks is open.
- Not done: Robot-frame per-axis Fit drift (needs the tracker-to-robot transform), a QP replay, and the stage split of (c) into Fit-published against gate-opened (the log does not record the gate opening; the bag has Fit publication and the ask time only).

## Findings (2026-11-09)

Your Option 2 on TKT-51. Sources: bag `/home/jetson/Desktop/rosbags/2026-11-06_15-53-05`, the 15:53 `launch.log`, `executor.py`, `bump.py`, `skill_node.py` at origin/main. Throwaway scripts, deleted.

### Verdict
- **The premise was wrong.** Columns asks a second time as often as the turned pair. Last night's "3 against 27" counted only `RESEND` lines and missed the Columns second asks, which go to the BUMP.
- **What differs is the route.** `BUMP_ENABLED_PATTERNS` (`bump.py`) is `columns` and `columns_1ball_fed`, so the turned pair's second asks fall through to the raw QP re-solve.
- Nothing in the Fit separates them.

### Second asks, both routes counted
| Set | Catches | Catches with 2+ re-aims | Second ask by QP re-solve | BUMP installed |
|---|---|---|---|---|
| Columns | 65 | 27 | 3 (all LIMIT_JERK) | 29, none refused |
| Turned | 79 | 27 | 27, of which 25 refused | 0 (route off) |

### Fit drift between the asks, Robot frame
| Set | n | Gap, median s | Along pair, abs median mm | Across pair, abs median mm |
|---|---|---|---|---|
| Columns | 27 | 0.275 | 6.1 | 11.7 |
| Turned | 27 | 0.228 | 6.3 | 6.5 |

- The drift has the same sign in the Robot frame on both axes whatever the pair axis, so it is frame-fixed, not a pair-axis effect.
- The ask test is per-axis at 10 mm or 10 ms. A norm test would fire more on the turned pair, not less. Cadence and the 30 mm clamp do not separate them either.

### Not done
- No replay of the BUMP on turned Catches, so whether it would cut the refusals is inference.
- Your 2026-10-16 condition for other Patterns (a matching-record capture passing the replay) has no capture for `columns_turned` in the files read.
- The 2026-11-07 log and the one-ball goals were not recounted.

## Findings (2026-11-10): what the capture involves

Your question on TKT-56. Read from `plans/archived/late-catch-reaim-bump.md` § 2 and § 5 and the three scripts in `tools/probes/late_reaim/`. Steps 1 to 4 are offline; only step 5 flies.

1. **Capture.** `capture_records.py` runs the sim Columns gate (`sim/skills_gate.py`) at the live Sitting limits and saves every plan record the executor installs. The gate already turns the pair (`angle_deg`); the script needs one flag to pass it. One run each at 45 and 90 deg, minutes each.
2. **Rows.** `parse_resends.py` collects the RESEND and RESEND-REFUSED lines (delta, time to landing, refusing limit). Its log paths are fixed to 2026-10-16; point it at the 2026-11-06 and 2026-11-07 launch logs to get the refused turned second asks.
3. **Replay.** `bump_replay.py --retarget --hold-s 0.05 --patterns columns_turned` runs the production `apply_catch_bump` on each row against the matching record, through the feasibility gate. Columns passed 86.9 % (diagonal) and 94.7 % (axis) on 2026-10-16; that is the bar.
4. **Enable.** Add `columns_turned` to `BUMP_ENABLED_PATTERNS` in `bump.py` with its test, then the full gate. No interface change: the bump request exists since 2026-10-16.
5. **Fly.** One turned Columns Sitting, judged by the archived plan's § 5 grep: every post-band re-aim is a `BUMP` line, no bump-caused `LIMIT_*`, landings no worse than unbumped.

- The Night Shift cannot run steps 1 to 3: its permission list refuses `tools/probes/`. A day Session can, in about an hour.
- Step 4 is then one small branch, held for step 5's Sitting.
- One risk: the level planner is direction-sensitive at 45 deg (IDEA-11: LIMIT_JERK 205k to 207k), so 45 deg may pass the replay less often than 90 deg.

## Decision (2026-11-11)

From TKT-59. You chose option 1 and will run it yourself: a day Session runs the capture and replay, steps 1 to 3 of Findings (2026-11-10). If the turned refusals pass at about the Columns rate (87 to 95 %), the next night builds step 4 (`columns_turned` in `BUMP_ENABLED_PATTERNS`, with its test) on a branch held for step 5's Sitting.

- The Next Sitting Runbook takes hardware steps only, and steps 1 to 3 are offline: they need no power, only `tools/probes/late_reaim/`.
- So BUG-75 is rated `hardware: robot` for step 5. Until then the Runbook lists it under "Needs a Sitting section" as your reminder.
- The Night Shift writes the `## Sitting` section when it builds step 4.

## Progress

- 2026-11-08: Night Shift 2026-11-08: the turned pair's first ask is not later; it asks twice more often, at no measurable cost (detail below)
- 2026-11-08: 2026-11-07: RESEND-REFUSED INFEASIBLE on 24 of 282 Throws (8.5 %); turned Columns 22 of 143 (15 %).
- 2026-11-09: Night Shift 2026-11-09: the turned pair does not ask twice more often; its second asks take the re-solve because the BUMP is off for it; awaiting the owner's answer
- 2026-11-10: Night Shift 2026-11-10: the capture is offline (sim records, log rows, replay), then a one-line enable and a Sitting; see TKT-59
- 2026-11-11: Night Shift 2026-11-11: TKT-59 answered (option 1); your day Session runs the capture and replay, then the night builds the enable for a Sitting; rated robot so the Runbook lists it
