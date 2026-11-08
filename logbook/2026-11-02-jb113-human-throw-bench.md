---
title: "IDEA-16: two human-only Sittings scored against the tracker's human-throw detector; tracks carry across throws (stale-fit cause unconfirmed), phantoms 12.5 and 27 per minute (upper bounds), usable landing estimate 0.3 s ahead on 24 of 42 and 41 of 71, landing scatter 75-130 mm sd at the robot-spot hand; the pattern prior gates association and pre-positions the cup, never the final aim (owner)"
type: investigation
date: 2026-11-02
status: complete
phase: "IDEA-16 human-partner cascade, perception first rung"
subsystem: tracking
files_changed:
  - tools/probes/human_throw_bench.py
  - tools/probes/README.md
  - map/idea-jp-17-three-ball-cascade-with-the-owner-as-the-second-hand.md
  - docs/runbook.md
  - logbook/INDEX.md
---

# IDEA-16: the human-throw bench

**Data.** The owner flew two human-only Sittings on 2026-11-02 with the robot moved aside
and `detect_human_throws=False` (so `/balls` is empty; the raw `/mocap_data` is what counts):
`2026-11-02_20-09-16_Human-single-3-tosses-near-Jugglebot` (91 s, one ball passed between a
hand near x 0.75 m and a hand where the robot's Cup lives) and
`2026-11-02_21-04-06_Human-3-ball-juggling-near-Jugglebot` (61 s, a three-ball cascade at
varying heights with one hand where the robot lives, reverse cascade at the end).

**Method.** New probe `tools/probes/human_throw_bench.py`: an independent ground truth
(gravity-aware linking of every eligible marker, fixed-g free-flight cuts, rising flights
as Throws, release points clustered into hands), then the same frames through the real
`BallTracker` (`tracker_bag_replay.make_tracker()`) with human detection forced on, scored
per Throw. The ball is unlabelled in every flight (the 09-15 AIM-label failure does not recur).

**Result** (`python tools/probes/human_throw_bench.py <both bags>`, 2026-11-03 09:33):

| | passes (91 s) | cascade (61 s) |
|---|---|---|
| Throws in the truth (crossing the 830 mm plane) | 44 (42) | 82 (71) |
| Served by a tracker ball | 44 | 81 |
| Within 50 mm / 30 ms at 0.30 s before the plane (usable for a Catch) | **24 of 42** | **41 of 71** |
| First estimate within 50 mm / 30 ms, at any lead | 30 of 42, lead median 0.65 s, p10 0.06 s | 55 of 71, lead median 0.56 s, p10 0.05 s |
| Phantoms (upper bound) | 19 (12.5 per minute) | 28 (27.3 per minute) |
| of which within 1.5 s of a truth release | 7 (4.6 per minute) | 22 (21.5 per minute) |
| Landing sd at the robot-spot hand (x, y) | 75, 125 mm | 129, 90 mm |
| Landing vs mean of the previous 5 there (median, p90) | 140, 278 mm | 108, 223 mm |
| Flight to the plane | 0.63-1.07 s | 0.22-1.04 s |

**Findings.**
- **Tracks carry across throws** (23 and 43 Throws were served by a track already on the
  ball at release). Many failing rows carry a landing time 0.5-1.5 s early: the hypothesis
  is that a carried track keeps the previous flight's frozen fit. Unconfirmed; the next step.
- **Phantoms** start on Ball Butler markers (about (-0.81, -0.12, 1.70) m, eligible because
  AIM sometimes labels a real ball as Ball Butler), on balls handled low, and near the Cup
  position at z 0.8 m (a ball being handled where the robot's own ball will sit). The rate is
  an upper bound: a flight the truth lost counts as a phantom (the cascade's last 12 s,
  mostly reverse cascade, are nearly absent from the truth). The 1.5 s split bounds it well
  for the passes (4.6 to 12.5 per minute) and barely for the cascade, where releases are dense.
- **Hand-region gate what-if** (a track's state carried back along its parabola to release
  height, within 200 mm of a hand): passes 0 of 19 and 3 of 28 phantoms, but 17 and 21 of
  the rejections are tracks that never rise (v_z <= 0 at their first state), so the rising-start
  test does most of the work and the hand geometry adds 2 and 4. It keeps only 19 of 44 and
  36 of 81 served Throws, because a carried track's state 30 ms in is still the hand-held
  state. The gate's retention is unmeasurable until the carried-track defect is fixed.

## Discussion

The owner asked whether knowing the Pattern lets the tracker focus on the ball's expected
place and time. Two uses were weighed. **As a measurement** (a prior pulling the landing
estimate toward the Pattern's mean) it would bias the aim: the owner's landings sit a median
108-140 mm (p90 223-278 mm) from the running mean, far outside the roughly 35 mm the Columns
catch corrects, and the fit converges to about 10-20 mm once it has the ball. **As a gate**
(which marker is the ball, when to expect it, and an earlier confirm through the announced-ball
path the tracker already has for Ball Butler throws) it removes the phantoms above without
touching the aim. Decision (owner, 2026-11-02): the prior gates association and confirms
earlier; the Cup moves to the running Pattern-mean landing before the release and re-aims as
the fit arrives; the final aim comes from the fit alone. A gate that rejects a wild Throw
fails safe (no catch). The owner expects tighter throws when juggling with the robot (better
lighting, one hand to manage); the bench re-measures that at the next data Sitting.

## Next

1. Confirm or refute the carried-track hypothesis on the bench's failing rows; fix in the matcher.
2. Phantom rejection and the gate against the bench (retention re-measured after 1).
3. The predicted-Throw announcement (the Pattern prior) through the announced-ball path.
4. Offline reach check: the Cup re-aiming over plus or minus 150-300 mm in the last 0.5-0.8 s.
5. A data Sitting with the robot in place, unarmed (IDEA-16's `## Sitting` section).

**Verification.** No reader imports the probe (`grep -rn human_throw_bench` hits only its
README row, this entry and the Item). Docs and map tests: see the commit message.
