---
kind: ticket
title: 'Resurrect CONTEXT.md: define the undefined coinages and resolve the conflicting terms'
type: grilling
parent: project-map
blocked_by:
- ticket-glossary-md-entry-format-the-claude-md-term-name-list-and
owners: []
rank: 8
status: complete
links:
- https://github.com/Project-DeepBlue-Juggling/Jugglebot/issues/29
created: '2026-10-23'
updated: '2026-10-23T14:37:58Z'
priority: medium
---

Part of #21

## Question

CORRECTION (2026-10-23): the file is on main (245 lines) and was softened on three claims in commit 84d40b27. Re-run the undefined/conflict lists from research/jugglebot-jargon-survey.md against the LIVE file before grilling; expect the list to shrink.

With the format from 07 fixed, grill the owner term by term on: (a) the ~9 terms defined nowhere: hop / hop-entry catch, braking knot (catch_k+1), lane / lane-latch, oval, "wave" of work, capture-move, robot frame fit, UH-n, plus any the jargon survey flags as undefined (research/jugglebot-jargon-survey.md, 67 rows); (b) the ~12 multiply-used terms: gate (suite gate / rung gate / feasibility gate), session vs sitting, "reload" vs feed, "toss" vs throw, "runsheet" vs runbook, armed, and the rest of the table's conflict flags; (c) the R0-R6 gate names and what each meant.

Output: the completed GLOSSARY.md content, written inline as terms resolve (domain-modeling), ready to land on main with the implementation. The two stale pointers (logbook 2026-09-30-gui-view-menu-triad-toggles.md:25 and 2026-10-04-skill-stack-r5-sitting-4.md:417) are fixed by the plan, not here.

## Answer

## Answer (grilling, owner, 2026-10-23, one round off an agent-built worksheet)

Method: a Sonnet agent re-checked the jargon survey's 25 flagged rows against the live `CONTEXT.md` (`research/glossary-coinages-worksheet.md`): 11 groups were already defined (Sitting vs Session and Sitting limits; Rung / Phase / the three gates; Armed; Cup vs Cone; Active / Stow / Homing; Guard vs motor_guard; Soft E-stop vs E-stop button; Feed; Throw; Runbook; Banking / Level / Tilt map) and 14 remained, each drafted from two or three usage excerpts. The owner confirmed 12 (fresh-origin on 2026-10-24), rewrote one (Wave) and struck one (UH-n). Entries are in the skill's format (#28) for the `GLOSSARY.md` landing in #35; the cluster heading each belongs under is given in brackets. The nine map-vocabulary entries from #25 land in the same commit.

### Entries

```
[The machine]

**Lane**:
One streamed command channel in the can-bridge: the legs' lane or the Hand's lane, each fed Knots by the emitter. A lane-latch is the firmware holding a lane after a refusal, cleared by lane recovery.
_Avoid_: channel, stream (for the per-group lane)

**Robot frame**:
The Mocap-measured pose of Jugglebot's base in the world frame, fitted from marker positions (the robot frame fit) and re-fitted when the robot is moved (a robot frame move). An Attempt is refused while it is settling.
_Avoid_: base frame

**Wave**:
The Freefly Wave camera on CAN3 (ADR 0014). Never a unit of work: a plan has Phases and Rungs and nothing below them.
_Avoid_: wave (for a batch of implementation units), "wave N unit X"

[Motion]

**Braking knot**:
A Knot of a CATCH Segment after touch-down while the Cup decelerates to the end of the catch stroke. The seating-cone check exempts braking knots; the seating-floor check does not.
_Avoid_: catch_k+1 (as a name)

**Carry**:
The motion of the Cup between a Catch and the next Throw while it holds the ball, within the Dwell.

**Capture move**:
The approach part of a CATCH Segment that reaches the Landing and takes the ball, including any re-aim when the Platform is already tilted off the new Landing.

**Catch plane**:
The fixed horizontal reference height (configured; 830 mm today) at which a Landing is defined and Apex is measured. Not the Platform's height.

[Skills]

**Hop**:
A one-ball Pattern in which the Hand throws from one Site and catches at the other, so the Cup crosses between Sites once per Throw. The hop entry is a Hop used as the opener of a fed Columns Attempt.
_Avoid_: transit (for the whole pattern)

**Oval**:
The Pattern in which each caught ball is carried back in the Cup to the throw Site inside the Dwell and thrown again; the first Pattern where the Cup translates while loaded. Columns carries nothing.

**Ball 1, Ball 2**:
Balls are numbered in the order Jugglebot's Hand first throws them: Ball 1 is in the Hand at the start, Ball 2 is fed or waits at the second Site. Operator-facing text uses these names; code prose may keep ball A / ball B for the same ids.
_Avoid_: ball A, ball B (in operator-facing text)

**Handoff lead**:
How early, in Knots, a Skill is dispatched ahead of its absolute time so the planner has solve budget to Splice it onto the Segment still streaming. A first-dispatch budget only.

**Fresh-origin**:
A Segment that opens from rest with no preceding Segment to Splice onto, so its origin is chosen at dispatch and its solve budget is a lead rather than a splice Knot.

[Tooling]

**Emit gap**:
The interval between consecutive Knot emissions to the can-bridge, nominal 25 ms at 40 Hz, published as `max_emit_gap_ms`. A large gap means the emitter stalled.
```

### Struck

- **UH-n**: vestigial (owner); no entry. The identifiers stay in `unified_cycle.py` and `CLAUDE.md:128` as history; the Avoid-word test does not police them.
- **R0–R6, P1–P5**: no entries; they are step ids carried by their plans' `steps:` lists (#27).

### Ripple noted for #35

- `CLAUDE.md:128` says "runsheet" (Avoid-listed) and "UH-3"; the rename commit fixes the first, the second stays as history.
- Code and plans still say "wave 2 unit A" in comments (`learner.py:352`, `executor.py:2201`, the dress-rehearsal plan): historical, not rewritten; new text uses "Phase N" or the step id.
- The seed Idea "rename the one-ball Patterns to Ball1/Ball2 terminology" (#26) is now backed by a glossary entry and stays an Idea (mass renames are out of scope).
