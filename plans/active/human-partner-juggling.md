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
  - {id: P1, title: Human Throw event, kind: phase, status: not_started, effort: m, hardware: none}
  - {id: P2, title: Reactive Catch, kind: phase, status: not_started, effort: l, hardware: none}
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
- Proposed, owner to confirm: phantom rejection by first-state speed of at least 500 mm/s. It rejects all 9 idle phantoms outside the Ball Butler box and loses 0 of 11, 1 of 44 and 4 of 82 real Throws on the three bags; "must rise" and a height floor lose far more.
- Open: the Destination's count (the Pathway item proposes 30 consecutive Throws of the shared cascade).

## 3. Steps

Every Rung needs its Sitting section dress-rehearsed on the loaded Jetson, `./run_tests.sh --full` green before the Sitting, and its own machine label in `records.py::PATTERNS`.

### P1: Human Throw event

- Adds: a message the skill stack can anchor a Catch to, published by the Tracker for a human ball once its Fit is usable: track id, fitted landing position and time at the Catch plane, the Fit's age and sample count, and a quality flag. Plus the phantom rule decided in section 2, and the Ball Butler box of BUG-92.
- Gate (Phase check): (date, `tools/probes/human_throw_bench.py` replay of the three bags through the event path, result: events per real Throw, lead at first event, phantoms per minute) - to be filled.
- Depends on: nothing. Items consumed: BUG-92.

### P2: Reactive Catch

- Adds: a Pattern, provisional label `human_catch`, Platform level, Cup waiting at a wait Site. On a human Throw event with enough lead it plans a Catch at the fitted landing through the existing command-time admissibility check, re-aims as the Fit refines, and does nothing when no admissible Catch exists (the Ball falls; the Attempt goes on). The Cup's wait Site follows the running mean of the owner's landings.
- Design questions for the unit that builds it: the re-aim authority for this Pattern (the 35/10/30 mm clamps are sized for a Learner correction, not a 150 mm human scatter) as an owner decision with the reach numbers; the latest lead at which a first plan is accepted; what "caught" means with no announced Throw (the Cup sensor); how Memory is kept out of it (a human Throw must not train the Learner's plant).
- Gate (Phase check): (date, offline replay of the three bags through the reactive path under `sim/skills_gate.py` limits, result: fraction of Throws with an admissible Catch planned, and the planned Cup position against the Ball's real crossing) - to be filled.
- Depends on: P1.

### P3: Catch one pass

- Adds: the first robot Catch of a human Throw. One Ball, the owner lobs it at the Cup, the robot Catches and holds; the owner takes the Ball back by hand.
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
