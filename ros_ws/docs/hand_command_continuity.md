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

**Sibling contract, added 2026-07-28:** `ros_ws/docs/hand_decel_feedforward.md`
(**C-HAND-2**) governs how hard the throw stroke *brakes* once a command has
been legally dispatched. The two do not overlap — C-HAND-1 is about WHEN a
command may be sent, C-HAND-2 about the torque feedforward of the segment
after ball release — but they share a failure surface worth knowing about: a
C-HAND-2 violation (a feedforward sized ABOVE the true reflected inertia)
produces a commanded dip below `x3` that this contract's own
`dip_below_x3 <= 0.10 rev` bench row cannot distinguish from the
queue-clobber defect C-HAND-1 exists to detect. That is why C-HAND-2's
declared inertia is required to be an under-estimate.

## The invariant

> **No hand command may create a discontinuity — in position or in velocity —
> between the live hand state and the newly commanded trajectory.**

Two obligations follow, and they sit on opposite sides of the CAN bus:

| # | Obligation | Owner | Status |
|---|---|---|---|
| **H** | A **scheduled** kind-0/1/2 stroke is not dispatched while another stroke is physically executing. | Host (`catch_coordinator_node`) | **Landed** 2026-07-26 |
| **F** | The smooth-move prelude is continuous with the live hand **velocity**, not seeded at `v = 0`. | Firmware (`Trajectory.h`) | **Landed in source** 2026-07-27 — **NOT LIVE until the Platform Teensy is flashed.** Whether it is live is now READABLE: `link_status/platform_fw_version` must show **`>= 1`**, not `0 (PRE-VERSIONING)` — `ros_ws/docs/platform_fw_version.md` (contract C-PLATFW-1) carries the current expected value, which moved to **2** on 2026-07-28 with C-HAND-2. Read the version against *that* file, never against a number restated here |

Obligation H is a mitigation, not a closure. It removes the one dispatch path
that was reliably violating the invariant. Obligation F is what closes
the class, because every command that can land mid-motion — a prime, a retract
ladder rung, a `SAFE_ABORT` — used to re-prelude from `v = 0` and H does not gate
any of them.

**Obligation F is satisfied CONDITIONALLY, and the condition is a number.**
`makeSmoothMove` now seeds the quintic `(x0, v0, a = 0) -> (target, 0, 0)`, but a
velocity-continuous profile must overshoot to come back, and the overshoot is
`|v0| * T * 16/81` with `T` set by the acceleration limit — so it costs
`0.00778 * v0^2` rev of travel. Against the stroke that buys continuity only up to
**~9.1 rev/s** at the stroke top and **~20.9 rev/s** from a mid-stroke freeze —
and mid-stroke the duration bound below binds first, at **~20.3 rev/s**.
Above that the profile falls back to the rest-to-rest quintic (today's exact
behaviour), because arresting sooner needs an acceleration nothing in the firmware
declares: a `SAFE_ABORT` retract dispatched with the hand essentially *at*
`hand_retract_rev = 0.0` and still descending at the measured -60 rev/s would need
**28 000 rev/s²** to keep its overshoot above encoder zero. (Far from the target
the same descent is served fine — 5.0 → 0.0 rev at -7.5 rev/s is honoured — so the
28 000 figure bounds the near-target corner, not every retract.) A second, weaker
bound applies as well: an honoured prelude may not take longer than the longest
rest-to-rest move the stroke admits (0.8005 s), so it never outlasts a profile the
firmware could already emit. So the honest statement of F is:

> **F is closed for every `|v0|` whose arrest fits inside the stroke at the
> declared acceleration limit, and explicitly, observably declined above it.**

The residual is not an implementation gap — it is a consequence of
`MAX_SMOOTH_MOVE_HAND_ACCEL_RPS2 = 100` rev/s² being a *comfort* limit 19-36×
below what the throw profile itself commands (**1902 rev/s²** at a 0.80 s flight,
**3597 rev/s²** at the band top, `FLIGHT_TIME_MAX_S = 1.10 s`). Widening F means
raising that limit, which changes what the machine can physically do at the bench
— an operator envelope decision, recorded as an open question rather than taken
here. **And it is now bounded from above by physics, not just by comfort:**
C-HAND-2 measures the axis's own deceleration ceiling at `hand_curr_limit_a = 50`
as 4178–4333 rev/s², so the band top already sits at 83–86 % of it. The headroom
argument for widening F is 36×, not 60×, and it runs out at ~4200 rev/s².

*(Corrected 2026-07-29. This passage previously read "19-60x" and "6055 rev/s² at
the band top". 6055 is the decel at the Teensy's `MAX_EVENT_VEL_MPS = 7.0`
builder clamp — a 1.43 s flight — not at `FLIGHT_TIME_MAX_S`; re-derived from the
shipped header, `|throwD| = 123.55·v²` rev/s². The old figure asserted the machine
routinely commands a deceleration above its own physical ceiling. See
`ros_ws/docs/hand_decel_feedforward.md`.)*

**Also closed by F, and worth naming separately:** the empty-trajectory branch of
`makeSmoothMove` now requires the hand to be **at rest** as well as at the target.
`Teensy_code.ino:472-475` returns from the kind-3 handler *before*
`packedMsgs.clear()` when the move comes back empty, so the old
position-only condition was a latent hole in the only un-arm mechanism the Teensy
offers. The new condition is strictly narrower: at the target but moving now
yields a braking profile, and the cannot-fit fallback emits a floored hold rather
than nothing.

## Why this exists — the failure it closes

The Platform Teensy holds **one** packed trajectory queue. Any kind-0/1/2 command
rebuilds it from scratch: `Teensy_code.ino:539` calls `packedMsgs.clear()` and
`makeSmoothMove` seeds the replacement prelude from `current_hand_position`.
Until 2026-07-27 it seeded `v = 0, a = 0`: the live velocity was available —
`current_hand_velocity` declared `extern volatile` two lines above the function —
and **never read**. That unread `extern` *was* the defect.

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
   assumed-within-tolerance for the measured one.** Partly closed 2026-07-27: the
   settle tail of every completed commanded hand move in the three 2026-07-25
   traces reads `|vel| <= 0.25` rev/s in the +20…+70 ms window the gated arm lands
   in, well inside the 6.0 rev/s dead-band. Runbook row H1.1's
   `dip_below_x3 <= 0.100 rev` remains the post-flash measurement.

## Known limits of obligation F

1. **A velocity-continuous prelude takes LONGER, and the arm-fit budget does not
   model it.** `required_arm_lead_s` budgets `PRELUDE_ALLOWANCE_S = 76 ms` — the
   REST-TO-REST duration of the 0.10 rev settle band — because that is the case
   the arm gate exists to produce. A continuous prelude over the same tiny `delta`
   takes **0.24 s at the 6.0 rev/s dead-band edge and 0.32 s at 8 rev/s**, since
   the duration is set by arresting `v0`, not by covering `delta`. Reachability
   and consequence, both bounded:
   * *Reachable only if the hand is drifting 6-9 rev/s when a kind-1 arm lands*,
     which the measured settle tail (≤ 0.25 rev/s) says does not happen — the
     6-9 rev/s figures come from the 2026-07-24 top-park dither p99 (5.39) and the
     slowest observed genuine traverse (9.2), not from an arm instant.
   * *If it did*: `t_acc_catch + 0.32 + 0.02` = 0.47 s at the nominal armed
     3.13 m/s, against the caller's `_MIN_EVENT_DELAY_S = 0.3` s floor. At an
     0.80 s flight the arm dispatches with ~0.55 s of `event_delay` and fits; at
     `FLIGHT_TIME_MIN_S = 0.55` the window's right edge IS the 0.3 s floor, so
     `Teensy_code.ino:533` could refuse the command — a lost catch, with the live
     stroke intact (`:533` returns before `packedMsgs.clear()`).
   **Instrumented, not fixed.** Widening `PRELUDE_ALLOWANCE_S` to the continuous
   duration would make the gate defer far more and could close the arm window at
   the band floor outright — a change to arming timing on the strength of a trigger
   that has never been observed. Runbook row **H4.7** is the guard: a
   `Not enough time for smooth-move` that appears only after the flash, on a toss
   whose `first_neg_cmd` shows a brake, is this limit firing.
2. **The affordable velocity band is narrow** — see the note under the obligation
   table. Above ~9.1 rev/s at the stroke top / ~20.3 rev/s mid-stroke (where the
   0.8005 s duration bound binds before the excursion clamp's ~20.9) the profile
   falls back to rest-to-rest, i.e. F is declined rather than met. The fallback is
   observable as a `n_seeds` row and is today's exact behaviour, so it is a
   *bounded* residual, not a regression.
3. **Continuity is C¹, not C².** The profile matches the live position and the
   live velocity but sets `a(0) = 0`, so it does *not* match the live
   acceleration. At the case that motivated the fix — a command landing on the
   throw's deceleration ramp — the hand is decelerating at ~1908 rev/s², and the
   prelude commands zero acceleration at its first sample. The commanded TORQUE
   feedforward (`accelToTorque`) is therefore still continuous *with itself*
   (`s''(0) = h''(0) = 0`, so there is no torque step, exactly as before), but it
   does not continue the ramp the hand was on. Matching `a(0)` would need a live
   acceleration estimate, and the Teensy has none: `current_hand_position` and
   `current_hand_velocity` come from the ODrive's 0x009 `Pos_Estimate`/
   `Vel_Estimate` frame (`Teensy_code.ino:439-441`) and there is no third field.
   `a0 = 0` is what the plan specifies and it removes the first-order
   discontinuity, which is the one the measured defect was caused by.
4. **~~There is no version handshake on the Platform Teensy.~~ CLOSED 2026-07-27
   — see `ros_ws/docs/platform_fw_version.md` (contract C-PLATFW-1).** As written
   here the board carried no `FW_VERSION`, so host/firmware skew on the hand path
   was undetectable from the Jetson: an un-flashed board behaved exactly like the
   pre-fix one with no log line, no `link_status` field and no refusal, and runbook
   row **H4.0**'s procedural chain was the only guard. It now declares
   `FW_VERSION` and reports it in bytes 5-6 of the 0x6E0 RobotState reply it
   already sends, surfaced as `robot_state.platform_fw_version` /
   `link_status/platform_fw_version` / a `PLATFORM_FW_CHECK` log line. **Obligation
   F's "NOT LIVE until flashed" row above is now directly observable** rather than
   inferred from a four-link evidence chain. The skew WARNS and never refuses —
   this path carries the kind-3 retract, the only un-arm mechanism the Teensy
   offers; see that document's § Warn, never refuse.
5. **The dead-band's anchor is a p99, not a maximum.** `smooth_move_v0_deadband_rps
   = 6.0` sits above the 2026-07-24 top-park dither p99 of 5.39 rev/s, whose
   maximum was never published — so roughly 1 % of parked-top samples are above the
   anchor and a single tail sample can seed a prelude on a hand nobody moved
   (0.280 rev = 8.9 mm at the dead-band edge, growing as `v0²`). An independent
   re-read of the three 2026-07-25 traces filtered for *stationarity* (position
   spread < 0.02 rev over ±0.3 s) rather than for a position band gives `|vel|`
   p99 = 0.134-0.144 and max 0.96 rev/s, which suggests 5.39 is a position-band
   artefact that swallowed transit samples. If so the dead-band is ~6× above the
   real noise floor — conservative, but resting on a number that wants
   re-measuring at the top park before anyone tightens it.
6. **The excursion floor is encoder zero, and the bottom stop is 0.1 rev below
   it.** `Homing::HAND_ABS_POS_REV = -0.1` rev *is* the bottom hard stop (the axis
   homes downward into it), and the host declares 0 the floor for this axis
   (`teensy_bridge_node` rejects a smooth-move target below 0, `can/odrive.py`
   clips a hand setpoint below 0 and warns). The clamp therefore uses
   `JBOp::HAND_RETRACT_REV = 0.0`, relaxed to `min(FLOOR, start, target)` so a
   legal target and a live sub-zero reading both stay servable. Unlike the
   ceiling, the floor carries no *margin* for the position loop's measured
   +0.186 rev undershoot — the 0.1 rev to the stop is the whole allowance, and
   that asymmetry is deliberate rather than derived.

## Related

* `plans/active/hand-command-continuity.md` — the plan, its Phase 0 evidence, and
  the Phase 4 firmware work that closes obligation F.
* `logbook/2026-07-26-hand-command-continuity-arm-gating.md` — the phase that
  landed obligation H.
* **Enforcement point for F**: `Teensy_code/Trajectory.h`'s `makeSmoothMove`,
  mirrored at `sim/hand/trajectory.py`'s `plan_smooth_move`, host model at
  `jugglebot/motion/trajectory/hand_stroke.py`. Pinned by
  `tests/firmware/test_hand_smooth_move_xref.py` (which **compiles and runs the
  shipped header**) and `tests/sim/test_hand_trajectory.py`. Bench:
  `tests/hardware/session_anomaly_fixes.md` § CHECK HAND-4 — **firmware flash, not
  a colcon build.**
* `logbook/2026-07-26-hand-stroke-timeline-probe.md` — Phase 0, the measurement
  the contract is built on.
* `tests/hardware/session_anomaly_fixes.md` § Section HAND — the bench checks.
* `ros_ws/docs/levelling_frame.md` — sibling contract C-LEVEL-1, same pattern.
