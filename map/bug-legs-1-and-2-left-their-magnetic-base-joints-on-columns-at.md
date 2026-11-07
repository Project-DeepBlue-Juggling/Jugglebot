---
id: JB-184
kind: bug
title: 'Legs 1 and 2 left their magnetic base joints on Columns at Sep. 170: acceptance bounds Leg kinematics only and does not see the bridge feedforward cap'
status: backlog
created: '2026-11-06'
updated: '2026-11-07T19:12:26Z'
owners: []
priority: high
effort: m
hardware: none
owner: decision
radius: cross-cutting
diagnosis: hypothesis
area: legs
blocked_by: []
links:
- logbook/2026-11-06-sitting-debrief.md
- logbook/2026-11-06-sitting-debrief-2.md
filed_by: claude
---

At the Sitting of 2026-11-06 a two-ball Columns goal at Apex 1.00, Sep. 170, Dwell 0.27 (limits 600 / 8000 / 200000) pulled Legs 1 and 2 out of their magnetic base joints about 9 s after Start. The owner pressed the E-stop button; nothing was damaged.

**Owner decision (2026-11-06): no bound on transit aggressiveness for now.** The consequence was minor and a conservative limit has its own cost; the owner will be mindful of the Patterns requested. Do not add a limit, margin or guard under this Item without a new owner decision.

## Evidence
- Each transit at Sep. 170 was about 187 mm in 0.3 s (170 plus the 10 mm outward re-aim at each end). The install lines asked Legs 1 and 2 for 486 mm/s and 6296 mm/s^2 (81.0 % and 78.7 % of the limits); the clean Attempts of the same launch (Sep. 100, 125, 135) asked for at most 361 mm/s and 4672 mm/s^2.
- The can-bridge caps each Leg's velocity feedforward at 3.5 rev/s, which is 249 mm/s. The rest of a faster plan is made up by position error: Legs 1 and 2 logged 55 and 65 lead-clamp ticks in two transits, against at most 36 in 21 transits in a clean Attempt.
- The joints let go at the end of the second transit (the Platform mocap pose left the pose implied by the Leg encoders from 1793931867.53). That transit's own loads were ordinary; the first one carried the highest estimated Leg tension of the Sitting (59 N on Leg 1 against a clean worst of 47 N; the estimate was not checked against a load cell).
- The gate and the nomination bound Leg velocity, acceleration and jerk only. Nothing bounds Leg tension, Platform lateral acceleration or joint pull-off, and nothing knows the bridge's feedforward cap.
- Width alone is not the cause: the hop at Sep. 250 flew on 2026-10-19 to 2026-10-22 at limits 350 / 5000 without separation.

## Not settled
- Whether the first transit unseated the joints and the second finished it, or the arrival of a long fast transit rolled the joint out with little axial preload. A bench pull-off test of one magnetic joint (axial force, and how it falls off-axis) would decide it.

## What would reopen this
- A second separation, or a wish to fly Columns wider than Sep. 135 at Apex 1.00. Numbers for choosing a bound are in `logbook/2026-11-06-sitting-debrief.md`.

## Options (2026-11-08)

The reopen rule fired: the 2026-11-07 Sitting separated Legs 1 and 2 twice more, at 5872 and 5968 mm/s^2. Full tables in `## Findings (2026-11-08)` below. Nothing has been added: no limit, margin or guard.

1. Keep no bound, and take Dwell 0.30 and 0.32 off the Runbook. Dwell 0.27 held at 5120.
2. Bench pull-off test of one magnetic joint first: axial force and how it falls off-axis. This decides whether a planned peak is the right measure at all.
3. A bound on planned Leg acceleration at goal accept, inside the 5488 to 5872 gap. It changes no per-tick Segment, but the gap is 7 % on three cases, and the cause is not shown.
4. Hold the joints mechanically, so a separation stops being the failure.

Recommendation: 1 and 2 together, and no limit value yet. Both separations began about 0.1 s after a transit ended near rest, which a planned peak does not describe.

Whatever you choose: Runbook rehearsals leave out the Catch re-aim, so they read low. That is 353 / 5180 rehearsed against 404 / 5872 flown.

## Findings (2026-11-08)

Sources: launch log `2026-11-07-16-15-21-645789-jetson-2503132` (the only 16:xx launch) and bag `2026-11-07_16-15-21`. Leg percentages in install lines are of 600 mm/s and 8000 mm/s^2 (81.0 % / 78.7 % = 486 / 6296 on 2026-11-06 confirms it).

### 1. Attempts and planned Leg peaks

Peak = largest `leg vel / acc` over every install line of the Attempt (buffer max, so it includes re-aims and the stop REST). Apex 0.93, Sep. 125 throughout.

| Start | Dwell | Ended | Planned peak mm/s, mm/s^2 | Joints |
|---|---|---|---|---|
| 16:16:47 to 16:20:13 (6 Attempts) | 0.27 | various | worst 384 / 5120 (63.9 % / 64.0 %) | held |
| 16:20:50 | 0.32 | +6.9 s NO_ADMISSIBLE | 404 / 5872 (67.4 % / 73.4 %) | let go +5.96 s |
| 16:21:23, 16:21:50, 16:22:11 | 0.32 | +14.8, +5.1, +5.1 s NO_ADMISSIBLE | 97 to 103 / 1470 to 1630 (ended early) | seated at Start |
| 16:23:43 | 0.30 | +7.6 s MISSED_CATCH | 406 / 5968 (67.6 % / 74.6 %) | let go +5.4 s |
| 16:25:42, 16:26:07 | 0.30 | NO_ADMISSIBLE | 90 / 1480 (ended early) | seated at Start |

- The early-ended Attempts are not comparators: they ended before the big transits.
- Rehearsal said 353 / 5180 for Dwell 0.32; flown plan 404 / 5872, which is 14 % and 13 % higher.
- Rehearsal for 0.27 said 310 / 4076; flown 384 / 5120 (24 % and 26 % higher). Same cause as 2026-11-06: the Runbook rehearsal omits the Catch re-aim (measured as a gap, cause per JB-184's own note).

### 2. When the joints let go (measured)

Method: encoder FK of `/robot_state` (production `leg_lengths_to_pose`) against the mocap Platform z. Z is used because the xy frames differ by the base yaw. Offset 4.06 mm taken from the 16:20:13 clean Attempt. n = 2 separations.

| Attempt | First dip | Clear separation | After arrival of |
|---|---|---|---|
| 0.32 #1 | z -6 mm at +5.35 s, then -4.5 mm for 0.3 s | z -7 mm at +5.96 s, -108 mm by +6.35 s, mocap lost at +7.0 s | 3rd transit (+5.27 s), then the stop REST's arrival (+5.86 s) |
| 0.30 #1 | z -10.8 mm at +5.39 s | -22 mm at +5.95 s, -72 mm at +6.27 s | 3rd transit, arrival at x = +62 (+5.26 s) |

- Both dips start about 0.1 s after the third transit ended, with the Platform nearly at rest.
- Clean baseline: z residual within 1.6 mm over the 14 s of the clean Attempt.
- The 0.32 #1 end-of-Attempt stop REST (103 mm, 0.24 s) is a braking move after the refusal at +5.66 s, not a planned transit.
- Not computed: lead-clamp ticks and Leg tension (budget). The 2026-11-06 method needs `/link_status` counters and a load model I did not rebuild.

### 3. What differs from the clean Attempts (mostly hypotheses)

- Measured: planned acceleration 5872 and 5968 against clean maxima 5120 (today) and 5488 (Sep. 120, 2026-11-06); speed 404 and 406 against 384 and 371.
- Measured: the owner's re-seat worked each time. The z residual before the next Start is 0.1 mm or less (-0.4 for 0.32 #4), so no Attempt started with joints still displaced. Who re-seated is not in the log.
- Measured: transits here were 80 to 130 mm chords (FK, |vx| > 300 mm/s), not the 187 mm of Sep. 170. Transit length does not explain these two.
- Hypothesis: the trigger is the third transit's arrival after two prior fast reversals. Two cases cannot tell this from "accumulated unseating".
- Hypothesis: both separations came with the Catch re-aim clamped at its limit (AIM-LATERAL-CLAMPED at -44.8 mm in y, authority 30), which adds acceleration the rehearsal omits.
- Weak: `mocap_node` logged "Robot frame MOVED 10.3 mm / 2.0 deg" at 16:23:52, after the 0.30 fall. The base was moved or the Platform fell on it. Not at the 0.32 fall.

### 4. NO_ADMISSIBLE_COMMAND after the first Attempt

- First Attempt line (L3980): `END NO_ADMISSIBLE_COMMAND at skill 6 (CATCH): ... candidate (11.3, 10.0) mm, apex 0.860 m refused MARGIN (margin -0.026)`.
- Later Attempts (L4245): `at skill 2 (CATCH): ... candidate (15.3, 10.0) mm ... refused LIMIT_JERK (margin -0.208)`. Same at L4373 and L4528.
- The refusal is the goal's own headroom, not the separated Platform. The nomination printed for Dwell 0.32 reads margin 0.189 nominal and -0.049 with the Catch re-aim at its limit; Dwell 0.30 reads -0.031.
- The Ball Butler feed already lands 31 to 45 mm off the Site in y before the separation (AIM-LATERAL-CLAMPED -44.8 mm in Attempt 1). Later Attempts carried the candidate's x from 11.3 to 15.3 mm (a 4 mm difference in the feed; cause not established, hypothesis: Ball Butler aim noise).
- No Site or pose is read from the separated Platform. Verdict: unrelated to the separation, and predicted by the nomination before flight.
- The Runbook row says to stop after three refusals. Four 0.32 and three 0.30 Attempts ended this way.

### Can a bound on planned Leg acceleration separate these?

- Numerically yes, narrowly. Separated cases: 5872, 5968 (today), 6296 (2026-11-06). Clean maxima seen: 5120, 5488. The gap is 384 mm/s^2 (7 %) with n = 3 separations and two clean maxima. Speed: separated 404, 406, 486; clean 384, 371.
- Not shown: that planned peak is the cause. Separation came about 0.1 s after a transit ended at near-zero speed, which fits tension or arrival loading, not peak plan acceleration. Sep. 120 reached 5488 clean, 7 % under.
- A bound would also bite goals the owner may want: any bound inside that 7 % gap refuses the 0.32 and 0.30 goals the owner asked for, which the existing Dwell 0.27 flies at 5120.
- Walking one 40 Hz tick (hypothetical): a refusal at goal accept changes no per-tick Segment, so no feedforward effect. A bound applied as a planner clamp would change Segment timing and needs its own walk. I recommend no limit; the owner decides.
- Per the reopen rule: two further separations since 2026-11-06, both at Dwell 0.30 / 0.32, so JB-184 qualifies for the reopen decision.

## Progress

- 2026-11-06: 2026-11-06 P2 Sitting: two-ball goals at Sep. 120 planned Leg peaks of 321 to 371 mm/s and 3656 to 5488 mm/s^2 (goal 17, L9651), above the previous clean envelope (361 / 4672) and under the separating transit (486 / 6296). Owner: base joints stayed attached throughout. The Runbook's offline prediction (264 / 3011) read low: it did not include the Catch re-aim.
- 2026-11-08: Night Shift 2026-11-08: the reopen rule fired: two more separations on 2026-11-07 at 5872 and 5968 mm/s^2; options and findings below, no limit added
