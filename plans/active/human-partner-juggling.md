---
title: Human-partner juggling - from one caught pass to a shared three-ball cascade
created: 2026-11-18
status: active
owner: Harrison
last_updated: 2026-11-18
pathway: human-partner-juggling
branch: main
scope: >-
  Take the robot from never having caught a human Throw to a three-ball cascade shared with the owner
  (JP-17), one capability per rung: catch one pass, catch and throw back, hold the Platform tilted
  toward the owner, then three Balls. Eight steps, four of them offline.
steps:
  - {id: P1, title: Human Throw event, kind: phase, status: in_progress, effort: m, hardware: none}
  - {id: P2, title: Reactive Catch, kind: phase, status: in_progress, effort: l, hardware: none}
  - {id: P3, title: Catch one pass, kind: rung, status: not_started, effort: s, hardware: robot}
  - {id: P4, title: Throw back to a fixed hand point, kind: phase, status: not_started, effort: m, hardware: none}
  - {id: P5, title: Catch and throw back, kind: rung, status: not_started, effort: s, hardware: robot}
  - {id: P6, title: Held tilt, kind: phase, status: not_started, effort: l, hardware: none}
  - {id: P7, title: Exchange at a held tilt, kind: rung, status: not_started, effort: s, hardware: robot}
  - {id: P8, title: Three-ball cascade, kind: rung, status: not_started, effort: l, hardware: robot}
related_logbook:
  - 2026-11-02-jb113-human-throw-bench.md
  - 2026-11-03-jb113-human-ball-fit-segments-flights.md
  - 2026-11-15-human-throw-ball-butler-zone.md
  - 2026-11-18-sitting-debrief.md
  - 2026-11-18-human-throw-event.md
  - 2026-11-18-human-catch-compile.md
related_code:
  - ros_ws/src/jugglebot/jugglebot/tracking/matcher.py
  - ros_ws/src/jugglebot/jugglebot/tracking/flight_fit.py
  - ros_ws/src/jugglebot/jugglebot/ball_possession.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - tools/probes/human_throw_bench.py
---

# Human-partner juggling

The schedulable form of the Pathway item for JP-17 (IDEA-16 holds the physics: timing, pass distance at a held tilt, the bench history). Owner, 2026-11-18: work the ladder below, do as much as possible offline before the first Sitting.

## 1. Context

- IDEA-16 was closed by mistake on 2026-11-13 (its bench row passed) and reopened 2026-11-18. No robot Catch of a human Throw has been attempted.
- Perception is close for high Throws. On the 2026-11-18 bag (11 cascade Throws beside the robot, apex 1.85 to 2.07 m) 10 of 11 had a landing estimate within 50 mm and 30 ms at 0.30 s before the plane; the first usable estimate arrives a median 0.71 s ahead. On 2026-11-02 it was 51 of 71 (cascade) and 37 of 42 (passes).
- The Catch path cannot take a human Throw today. It is fed only by a Ball Butler announcement (`skill_node.py:6225`); a human track has no `throw_time`, so the announced-track latch (`ball_possession.py:2050`) never resolves it.
- Re-aim authority is far too small for a human: 35 mm inward, 10 mm outward, 30 mm across, two re-sends, none inside 0.25 s of touch-down, and a re-aim needs at least 0.35 s of lead. The owner's landings sit a median 101 to 163 mm (p90 122 to 387 mm) from the mean of the previous five.
- Rough reach, rest to rest under today's Sitting limits (600 mm/s, 8000 mm/s^2, jerk 200000) less the 0.225 s splice lead: 96 mm at 0.50 s of lead, 156 mm at 0.60 s, 216 mm at 0.70 s. It ignores the Jacobian, the contact window and the skid bound.
- Source for these numbers: the perception unit of 2026-11-18 (its tables are copied into IDEA-16).

## 2. Decisions

- Owner, 2026-11-18: no safety rules are added for a person inside the Platform's reach; the environment is safe and nothing changes on that account. This answers the precondition PWAY-8 recorded for this thread.
- Owner, 2026-11-18: the ladder is one Ball caught, then caught and thrown back, then the held tilt, then three Balls.
- Owner, 2026-11-02 (IDEA-16): the Pattern prior gates association and confirms earlier; it never pulls the landing estimate. The Cup moves to the running Pattern-mean landing before the release and re-aims as the Fit arrives; the final aim is the Fit's.
- Proposed, owner to confirm: the first rung flies high lobs (apex about 1.8 m or more), which is what the Tracker already fits well; the low-Throw floor (`flight_fit.py:141`, 1080 mm) is then not on the critical path. Lowering it for human balls to 900 mm would recover at most 8 of the 19 low Throws missed on 2026-11-02.
- Proposed, owner to confirm: phantom rejection by first-state speed of at least 500 mm/s (D1). It rejects all 9 idle phantoms outside the Ball Butler box and loses 0 of 11, 1 of 44 and 4 of 82 real Throws on the three bags; "must rise" and a height floor lose far more. The P1 gate (2026-11-18) leaves 0 phantom event streams on the 11-18 slice.
- Confirmed by measurement (2026-11-18, `human-catch-reach.md`, `p2-gate.md`; these replace the rough reach in section 1): a first plan needs at least about 0.51 s of lead; 20 to 50 mm of move at 0.55 s; 100 mm at 0.575 to 0.60 s; beyond that the +-150 mm cup workspace box limits, not the Legs; re-aims with 0.45 s of lead or less are refused for jerk. So the first aim is final and the correction must come before the plan: the policy plans once, from the latest revision, when the lead falls to the plan window (PROPOSED 0.62 s).
- Decided in code, not by the owner: a human Throw is a new message from the Tracker, `HumanThrowEvent` on `human_throws`, keyed `(track_id, flight_seq)` (the Ball Butler announcement and `/balls` were rejected for named failure modes, entry `2026-11-18-human-throw-event.md`); a human Catch is booked as its own count and never as a Feed; Memory is never opened for it.
- Still PROPOSED, owner to decide (full list in `2026-11-18-human-catch-compile.md` "Owner decisions open"): D2 usable bar (rms 6 mm, 30 mm / 20 ms); D3 first lead 0.55 s (the sweep suggests 0.60 s); plan window 0.62 s; D4 association radius 400 mm; D5 re-aim authority 50 mm; D6 re-sends 2; D7 wait Site (mean of 5, clamp 150 mm, deadband 20 mm); D8 clear 0.5 s and return 1.0 s; D9 nominal arrival Apex 1.9 m; D10 thresholds (P1 first-event criterion fails on the 11-18 bag, 2/11); D11 the 11-18 bag scored per direction; the holding timeout; the pending-key rule; the wording of three INVARIANTS.md rows (drafted, pending approval, not edited).
- Open: the Destination's count (the Pathway item proposes 30 consecutive Throws of the shared cascade).
- Owner, 2026-11-18 (evening): the person is assumed not to drop. A Ball counts as dropped only once it has neither been tracked nor felt by the Hand; a missed human Catch is otherwise not a drop.
- Owner, 2026-11-18 (evening): the three INVARIANTS.md rows are approved and landed with the code (I-PLAN-8's sixth entry, I-ADMIT-2's `human_catch` clause, the new I-CATCH-8).

## 3. Steps

Every Rung needs its Sitting section dress-rehearsed on the loaded Jetson, `./run_tests.sh --full` green before the Sitting, and its own machine label in `records.py::PATTERNS`.

### P1: Human Throw event

- Adds: a message the skill stack can anchor a Catch to, published by the Tracker for a human ball once its Fit is usable: track id, fitted landing position and time at the Catch plane, the Fit's age and sample count, and a quality flag. Plus the phantom rule decided in section 2, and the Ball Butler box of BUG-92.
- Status: built (`2026-11-18-human-throw-event.md`). Kept `in_progress`, not `complete`: the Phase check passes every D10 criterion except the first-event accuracy on the 2026-11-18 bag (2 of 11 within 50 mm / 30 ms against the 70 % bar), and the owner has not yet chosen between revising the criterion to "first good revision lead at least 0.60 s" (met: 0.70 / 0.72 / 0.59 s, the 11-02 cascade marginal), waiting on a better first estimate, or holding the first aim for the first revision (about 0.05 s).
- Gate (Phase check): (2026-11-18, `python tools/probes/human_throw_bench.py --events --out-dir temp/probes/p1 [--t-start 11:34:18 --t-end 11:37:30] <bag>` on the three bags, BUG-92 box on, proposed D1/D2) events for 11/11, 39/44 and 58/82 real Throws; first-event lead p50 0.75, 0.73 and 0.60 s; first-event xy error p50 70, 22 and 28 mm; first event within 50 mm / 30 ms 2/11, 33/37, 49/53; phantom event streams 0.0, 0.0 and 1.0 per minute. Verdict: first-event criterion fails on 11-18, all others pass (`p1-gate.md`).
- Depends on: nothing. Items consumed: BUG-92.

### P2: Reactive Catch

- Adds: a Pattern, provisional label `human_catch`, Platform level, Cup waiting at a wait Site. On a human Throw event with enough lead it plans a Catch at the fitted landing through the existing command-time admissibility check, re-aims as the Fit refines, and does nothing when no admissible Catch exists (the Ball falls; the Attempt goes on). The Cup's wait Site follows the running mean of the owner's landings.
- Design questions for the unit that builds it: the re-aim authority for this Pattern (the 35/10/30 mm clamps are sized for a Learner correction, not a 150 mm human scatter) as an owner decision with the reach numbers; the latest lead at which a first plan is accepted; what "caught" means with no announced Throw (the Cup sensor); how Memory is kept out of it (a human Throw must not train the Learner's plant).
- Status: built and audited (`2026-11-18-human-catch-compile.md`): `compile_human_catch`, `HumanCatchPolicy`, the node lifecycle, nomination, its own records row and Catch booking; one audit pass, nine new-Pattern items fixed in `c11137ba`. Its offline gate is not met on the existing bags, hence `in_progress`.
- Gate (Phase check): (2026-11-18, `python -u tools/probes/human_catch_replay.py <bag> --frames-pkl <cached> --sweep` on the three bags, `hand` frame, plan window 0.62 s, authority 50 mm) plans admitted 5/11, 6/44, 13/82 Throws; planned Cup vs real crossing xy p50 / p90 28 / 52, 18 / 33 and 30 / 46 mm; D10 good (30 mm, 20 ms) over in-reach Throws 1/11, 4/21, 5/46 against the 80 % bar: no-go as written; 0 plans on phantoms. The bags are multi-Ball cascades and passes and cannot score a one-Ball Pattern; every re-aim at the plan window was refused (15 of 15). Transport delay and skid not modelled (`p2-gate.md`).
- Depends on: P1.

### P3: Catch one pass

- Adds: the first robot Catch of a human Throw. One Ball, the owner lobs it at the Cup, the robot Catches and holds; the owner takes the Ball back by hand.
- Starts with a data row, before any live `human_catch` Attempt: single Balls lobbed at the Cup (the wait Site), one Ball in the air at a time, Platform Level and NOT armed, `ros2 param set /ball_tracker_node detect_human_throws true`, about five minutes (roughly 15 to 20 lobs at lead 0.7 s or more); the bag is replayed through `tools/probes/human_catch_replay.py` and scored against D10 first. This is the population the three existing bags lack.
- Checks the first live Attempt needs: the Cup sensor reporting before the first lob (every verdict abstains and reads as caught when its history is blind); both ROS packages rebuilt (`jugglebot_interfaces` and `jugglebot`) because of the new message; the Sitting section dress-rehearsed on the loaded Jetson per the standing rule.
- Rung gate (proposed): 10 Catches of 15 lobs in one Attempt, with no Catch planned on a phantom.
- Depends on: P2. Plus the three standing Rung preconditions.

### P4: Throw back to a fixed hand point

- Adds: after the Catch and a Dwell, a Throw aimed at a fixed point where the owner's hand waits. Platform level first: the aimed-throw reach inside a catch-and-throw window was measured at 250 to 300 mm (IDEA-16, untraced), so the hand point sits beside the Cup.
- Gate (Phase check): (date, `sim/skills_gate.py` on the label plus the landing scatter of the aimed Throw in sim, result) - to be filled.
- Depends on: P3.

### P5: Catch and throw back

- Adds: a continuous one-Ball exchange between the owner and the robot.
- Rung gate (proposed): 10 consecutive exchanges in one Attempt.
- Depends on: P4. Plus the three standing Rung preconditions.

### P6: Held tilt

- Adds: a planner mode that holds the Platform tilted toward the owner for the whole Attempt, so the Throw carries 0.4 to 0.75 m (9 to 12 deg, IDEA-16) and the Cup faces the incoming Ball.
- Gate (Phase check): (date, nomination margins for the Catch and the Throw at 9 and 12 deg, plus `./run_tests.sh --full`, result) - to be filled.
- Depends on: P5.

### P7: Exchange at a held tilt

- Adds: the one-Ball exchange at juggling distance.
- Rung gate (proposed): 10 consecutive exchanges at a pass distance of 0.4 m or more.
- Depends on: P6. Plus the three standing Rung preconditions.

### P8: Three-ball cascade

- Adds: the robot's Throw time taken from the owner's tempo (each Ball's period is 3 Beats; the robot's Dwell equals the owner's), three Balls in the Pattern.
- Rung gate: the Pathway's Destination.
- Depends on: P7. Plus the three standing Rung preconditions.

## 4. Not in this plan

- The low-Throw floor change, unless P3 shows the owner's lobs fall under it.
- Any change to the Ball Butler Feed path or to Columns and the oval: `detect_human_throws` stays off by default and the new Pattern is its own label.
