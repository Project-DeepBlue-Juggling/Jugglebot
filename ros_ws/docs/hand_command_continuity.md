# Hand Command Continuity — contract C-HAND-1

**Normative.** This document specifies when a hand trajectory command may be sent
to the Platform Teensy. It is the written half of the repo's contract pattern
(normative statement + one canonical enforcement point + a test that fails on the
omission); the other two halves are
`jugglebot/catch_coordinator_node._throw_stroke_gate_ok` and
`tests/ros/test_catch_coordinator_node.py` /
`tests/motion/test_hand_stroke.py`.

Scope: the ROS 2 package `ros_ws/src/jugglebot/`, plus the Platform Teensy
firmware in `ros_ws/src/jugglebot/Teensy_code/`. The hand is the only actuator
this contract governs; leg and platform motion are unaffected.

## The invariant

> **No hand command may create a discontinuity — in position or in velocity —
> between the live hand state and the newly commanded trajectory.**

Two obligations follow, and they sit on opposite sides of the CAN bus:

| # | Obligation | Owner | Status |
|---|---|---|---|
| **H** | A **scheduled** kind-0/1/2 stroke is not dispatched while another stroke is physically executing. | Host (`catch_coordinator_node`) | **Landed** 2026-07-26 |
| **F** | The smooth-move prelude is continuous with the live hand **velocity**, not seeded at `v = 0`. | Firmware (`Trajectory.h`) | **Open** — plan Phase 4, needs a Platform Teensy flash |

Obligation H is a mitigation, not a closure. It removes the one dispatch path
that was reliably violating the invariant. Obligation F is what actually closes
the class, because every command that can land mid-motion — a prime, a retract
ladder rung, a `SAFE_ABORT` — re-preludes from `v = 0` today and H does not gate
any of them.

## Why this exists — the failure it closes

The Platform Teensy holds **one** packed trajectory queue. Any kind-0/1/2 command
rebuilds it from scratch: `Teensy_code.ino:539` calls `packedMsgs.clear()` and
`Trajectory.h:242-301` seeds the replacement prelude from `current_hand_position`
with `v = 0, a = 0`. The live velocity is available — `current_hand_velocity` is
declared `extern` at `Trajectory.h:47` — and is **never read**.

That is harmless when the hand is at rest, which is why it survived a year of
reload operation: a reload's catch arm lands with the hand parked at the top, so
the "rest-to-rest" quintic is seeded from a position the hand is genuinely
standing on.

It is not harmless during a self-toss. Measured 2026-07-25 across seven tosses in
three sessions: the kind-1 catch arm landed **8-18 ms after ball release**, inside
the throw's 65 ms deceleration ramp. The queue was cleared while the hand was
travelling through ~120 rev/s, and replaced by a rest-to-rest quintic computed
from that instant's position. Consequences, all measured:

* the hand **overshot to 10.17-10.33 rev**, leaving 0.775 rev of headroom against
  the 11.1 rev overextension guard;
* it was then **yanked 0.34-1.75 rev below the stroke end** (10.7 to 55.3 mm, up
  to 20.5 % of the usable stroke) and recovered over ~300 ms;
* the throw's own deceleration ramp was discarded, replaced by the position
  loop's reaction to a frozen setpoint.

The defect is not "the arm was late". It is that **a single-queue, last-writer-
wins actuator was being written to while it was moving**, and nothing in the
system forbade it.

## Enforcement

Obligation H has exactly one enforcement point:

```
catch_coordinator_node._throw_stroke_gate_ok(event_delay, event_vel_mps) -> bool
```

called from `_arm_hand_catch`, which is the node's only kind-0/1/2 dispatch. The
gate sits at the **dispatch**, not at its call site in `_on_balls`, so a second
caller added later (a re-arm timer, a recovery path) cannot bypass it silently —
and the defect it would re-introduce is invisible in ROS, because the Teensy
prints nothing when it clears its queue.

The stroke-busy window is **derived, not fixed**:

```
clear_at = announced throw_time + t_dec(|initial_velocity|) + ARM_SUPPRESS_MARGIN_S
```

Both inputs are already on the wire (`ThrowAnnouncement`). A fixed conservative
delay was rejected because `t_dec` spans 94.5 ms at the 0.55 s flight to 47.4 ms
at 1.10 s — sized for the short end it wastes half the window at the long end;
sized for the long end it lands back inside the ramp at the short end, where the
momentum is ~1.9x and the end-stop headroom is smallest.

The timing model itself lives in **one** place,
`jugglebot/motion/trajectory/hand_stroke.py` — the host-side copy of
`Trajectory.h`'s closed form. `tools/probes/hand_stroke_timeline.py`, which turns
a bench capture into the PASS/ABORT verdict, imports the same module. If the
verdict instrument and the shipped window ever disagreed about `t_dec`, the bench
would score the fix against a different model than the one that shipped.

## Deliberate exemptions — do not "fix" these

* **kind-3 smooth-moves are NOT gated.** A kind-3 replacing whatever is queued is
  the **only un-arm mechanism the Teensy offers**, and a pre-release `SAFE_ABORT`'s
  retract depends on it clobbering an armed kind-0 throw stroke
  (`toss_sequencer`'s ORDERING PRINCIPLE). Gating it would make the abort path
  unable to stop a queued throw. The toss's own prime-during-stroke hazard is
  owned by a separate, already-enforced gate: `catch/prime_hold`, raised for the
  whole PREPARE→terminal span. Pinned by
  `test_kind3_smooth_move_is_not_gated_by_the_stroke_window`.
* **The window keys on `thrower_name`, not `target_id`.** A BallButler throw
  aimed at us carries `target_id == 'jugglebot'` too, so a `target_id` gate would
  delay every reload's catch arm — which has no throw stroke to protect and needs
  its lead. A reload announcement leaves the window `None` and the whole gate
  inert.
* **A closed window dispatches rather than defers.** `Teensy_code.ino:533` refuses
  the whole command when it will not fit and prints to serial **only** (`:534`),
  so an arm deferred past that point is not a late catch — it is a silently
  missing one with no ROS-visible signal. A dip is recoverable; a dropped ball is
  not.

## Known limits of obligation H

Stated so a bench session scores it correctly:

1. **The forced (window-closed) branch promises an attempt, not a catch.** Its fit
   check budgets the at-rest prelude; on that branch the hand is mid-stroke, so
   the firmware's live-encoder prelude is 0.37-0.76 s and `:533` may refuse the
   dispatch. This is exactly the pre-fix arithmetic, and `:533` returns **before**
   `packedMsgs.clear()`, so a refusal leaves the live stroke intact — the cost is
   a lost catch, never a clobbered stroke.
2. **The deferral is tick-dependent.** The gate is reached only from `_on_balls`;
   nothing re-enters it on a timer. A track that drops out for the whole remaining
   span, or a landing-time revision that pushes `event_delay` under
   `_MIN_EVENT_DELAY_S`, bypasses the gate and the arm is never dispatched.
   Instrumented at the bench (runbook row H1.7) rather than guessed at.
3. **An armed stroke produces no observable until its event time**, so a kind-1
   arm cannot be telemetry-verified the way the hand ladders were. What *can* be
   verified from a capture is the harm the guard prevents: a repack that clobbers
   a live stroke leaves a from-rest quintic seed at the live position, which the
   probe counts as `n_seeds`.
4. **`v = 0` at the window's opening is exact for the commanded profile and
   assumed-within-tolerance for the measured one.** No post-fix settle has been
   measured; runbook row H1.1's `dip_below_x3 <= 0.100 rev` is the measurement.

## Related

* `plans/active/hand-command-continuity.md` — the plan, its Phase 0 evidence, and
  the Phase 4 firmware work that closes obligation F.
* `logbook/2026-07-26-hand-command-continuity-arm-gating.md` — the phase that
  landed obligation H.
* `logbook/2026-07-26-hand-stroke-timeline-probe.md` — Phase 0, the measurement
  the contract is built on.
* `tests/hardware/session_anomaly_fixes.md` § Section HAND — the bench checks.
* `ros_ws/docs/levelling_frame.md` — sibling contract C-LEVEL-1, same pattern.
