---
id: JB-188
kind: bug
title: Catch re-aim refused INFEASIBLE on 8.4 % of Throws at the P2 Sitting, 15 of 69 on the turned pair
status: inbox
created: '2026-11-06'
updated: '2026-11-06T19:54:17Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
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
