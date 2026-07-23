---
title: Phase-7 re-test — catch strokes erased by a prime-vs-arm race on the Teensy queue; tracker split-track corruption fakes CAUGHT; pre-tilt + catch_vel_scale land
type: investigation
date: 2026-07-23
status: resolved
phase: "MVP trajectory bringup — Phase 7 reload: second hardware session (re-test of the same-day fixes)"
related_plan: mvp-trajectory-bringup.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/catch_coordinator.py
  - ros_ws/src/jugglebot/jugglebot/catch_coordinator_node.py
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py
  - ros_ws/src/jugglebot_interfaces/action/Reload.action
  - tests/ros/conftest.py
  - tests/ros/test_catch_coordinator.py
  - tests/ros/test_catch_coordinator_node.py
  - tests/ros/test_reload_coordinator_node.py
  - tests/hardware/session_phase7_reload.md
commits: []
subsystem:
  - ros
tags:
  - reload
  - catch
  - hand
  - tracking
  - race-condition
---

# Phase-7 re-test: the morning fixes worked; the next layer of failures, diagnosed and fixed

## Summary

Re-test session, bag `~/Desktop/rosbags/2026-07-23_11-37-57` (7 reload attempts:
1 pre-throw abort + 6 throws, 0 real catches). The morning's z-frame fix **worked**:
the platform reached + tilted on every throw and every ball met the hand. The
session surfaced the next layer:

1. **Catch strokes erased on 3 of 6 attempts by a prime-vs-arm race.** The catch
   stroke fired on attempts 1/4/6 and was silently erased on 2/3/5.
2. **5 of 6 reloads spuriously reported CAUGHT** while the real ball bounced out —
   tracker split-track corruption.
3. **Attempt 0 aborted on a transient prime-dispatch failure** (correct behavior,
   wasted attempt).
4. Even fired strokes bounced out: the cup was already descending at contact (real
   arrivals ran 0.10–0.14 s later than announced) — the tuning space the operator's
   new speed knob addresses.

## Root cause 1 — the prime-vs-arm race (strokes 2/3/5)

`catch_coordinator_node`'s `_hand_primed` flag never latched all session (the edge
prime's service confirmation never came back success — unresolved from the bag; a
first-attempt symptom of the same transient class as attempt 0). So on the first
balls tick of every flight, the gated **re-prime** fired in the *same tick* as the
catch **arm** — two async service calls racing to the Platform Teensy, whose kind-3
(smooth-move) and kind-1 (catch) handlers each do `packedMsgs.clear()` and repack:
**last writer wins** on a single packed queue. Re-prime last → armed catch erased
(2/3/5); arm last → stroke survived (1/4/6). Telemetry proof: the catch prelude
(cmd 9.858 → 9.875) starts on ALL six attempts at throw+0.13–0.15 s; on 2/3/5 it is
smoothly walked back to 9.858 (the re-prime's own profile) and no stroke ever fires.

**Fix — never send a smooth-move while a catch sequence is live.** The prime is
REMOVED from the balls path entirely. Priming lives on the `catch/armed` rising edge
plus a slow off-path retry timer (0.5 s) that is suppressed for 1.5 s after any catch
command (`_PRIME_RETRY_QUIET_S`). Two adjacent hardening fixes from the same trace:
the arm one-shot (`_hand_traj_armed_for_ball`) now latches **only when the arm was
actually dispatched** (a service-not-ready early-return used to latch it permanently,
killing all retries), and the silent `event_delay < 0.3 s` drop now logs once per
ball.

## Root cause 2 — tracker split-track corruption fakes CAUGHT (and re-fakes the "drift")

Each announced ball confirms on launch detections near BB, then its Kalman landing
estimate jumps ~200 mm and drifts toward BB at ~(−480, −255) mm/s while the state
coasts BELOW THE FLOOR (z < −400 mm at 8–9 m/s) — and still flips CAUGHT 0.25–0.71 s
after landing. The REAL ball spawns as a separate untagged (`destination=''`) track
predicting landing 29–39 mm from centre (the operator's "spot-on"). Consequences in
this bag: (a) all post-confirm reactive targets rejected WORKSPACE 201–227 mm past
the 80 mm envelope — **the envelope did exactly its job**, holding the accepted
announcement-based reach; (b) 5 of 6 reloads reported SUCCESS `CAUGHT` off the
corrupt track; (c) attempt 5 latched a pre-existing phantom untagged track as "our"
ball (`not b.destination` clause) and rode it to a wrong MISSED.

**Mitigations here (the tracker fix is its own investigation, still OPEN):** a
CAUGHT **plausibility gate** in `reload_coordinator_node` — a caught ball confirms
only within 200 mm horizontal / 150 mm vertical of the catch point (the corrupt
tracks flip CAUGHT ~630 mm out and 1.3 m low; a real cup catch is tens of mm) — and
the announced-ball latch now **excludes ids already in flight at throw-accept**,
prefers a destination match over the untagged fallback, and treats an untagged latch
as provisional (a destination-tagged candidate displaces it — a phantom spawning
during the countdown would otherwise grab the latch exactly as attempt 5's did).
Reload outcomes are now honest: a bounce-out reads MISSED. **Known cost while the
corruption stands: a REAL catch may also read MISSED** — the latched announced track
is the corrupt one, so its CAUGHT is rightly rejected as implausible and the real
ball's separate untagged track is never consulted. Containment is deliberate (a
false CAUGHT misleads tuning worse than a false MISSED); the runbook tells the
operator to judge seating by eye/hand telemetry until the tracker investigation
lands.

## Root cause 3 — transient prime dispatch failure (attempt 0)

The reload's first `smooth_move_hand` dispatch failed once (the same service executed
the retract seconds later), aborting `ABORTED_PRIME_FAILED` before any throw —
correct, but a wasted attempt. **Fix**: one immediate retry on prime-dispatch
failure (free — the prime is pre-throw); a double failure still aborts.

## New capability 1 — pre-tilt at announcement

The reactive reach only starts at throw detection (the coordinator filters
`IN_FLIGHT`), leaving ~0.58 s of usable motion and a ~95 %-settled platform at
contact. The `ThrowAnnouncement` — published inside the throw call, ~3.9 s before
landing — already carries the solver-consistent landing position/velocity/time. While
armed, `catch_coordinator_node` now synthesizes ONE predicted catch target from it
(`CatchCoordinator.predicted_catch_command`, single-sourced with the reactive pose
math), so the platform settles into the receive tilt during the countdown. The
receive-tilt direction is the BB→target azimuth (flight-invariant) and saturates the
12° clamp for real BB arrivals — which is what makes pre-tilting safe. Mid-flight
refinements supersede as C2 replans; each accept re-anchors the reach freeze, so the
final 0.3 s commits to the last refined target. The predicted target does NOT touch
the per-ball correlation state (no blacklist feed) and NEVER arms the hand (the
Teensy fire is one-shot on the first event time — arming at announcement would bake
~3.9 s of countdown jitter into it; the release-time arm from the same announced
landing time is strictly tighter). Excursion ≈ 14 mm ≪ the 80 mm envelope.

## New capability 2 — the operator's catch-speed knob

`Reload.action` goal gains `catch_vel_scale` (0 ⇒ 1.0), relayed on a new
`catch/vel_scale` topic at PREPARE (before the armed edge) and applied by
`catch_coordinator_node` to the event velocity the hand catch is armed with —
mathematically identical to re-tuning the flash-gated firmware `CATCH_VEL_RATIO`
(0.6), per-attempt, no reflash: the kind-1 trajectory depends on the event velocity
only through `0.6·v` (stroke length and start are constant; timing `t_acc = 0.404/v`,
decel ∝ v²). Clamped to [0.3, 1.5]: below, the scaled velocity falls under the Teensy
windup budget and the stroke is silently dropped; above, the 7 m/s ceiling binds.
Reset to 1.0 on the disarm edge so one attempt's tuning never leaks into the next.
Usage: `ros2 action send_goal /jugglebot/reload jugglebot_interfaces/action/Reload
"{throw_delay_s: 3.0, catch_vel_scale: 0.8}" --feedback`, or publish
`catch/vel_scale` manually for bench throws.

## Verification

- Scoped (2026-07-23): `pytest tests/ros/test_catch_coordinator_node.py
  tests/ros/test_reload_coordinator_node.py tests/ros/test_catch_coordinator.py
  tests/ros/test_reload_sequencer.py tests/ros/test_reload_integration.py -q` —
  all pass (27 + 30 + 5 + 32 + 4).
- Broader (2026-07-23): `pytest tests/ros tests/motion -q` = **1675 passed in
  350.99 s**.
- Full pre-commit gate, run twice on 2026-07-23: `pytest tests/ -q` = **2921
  passed, 1 xfailed, 0 failed in 774.49 s** (pre-audit-fix tree), then on the
  final tree (audit fixes + their tests applied) **2925 passed, 1 xfailed,
  0 failed in 743.89 s**.
- New tests: no-prime-from-balls-path + retry-timer quiet-window suppression (the
  race); arm one-shot latches only on dispatch; vel-scale scaling/clamping/reset +
  re-clamp to Teensy bounds; announcement pre-tilt publish, gating, and
  correlation-state isolation; predicted-vs-reactive pose-math identity; CAUGHT
  plausibility gate (below-floor CAUGHT rejected, near-cup CAUGHT counted);
  pre-existing-flight exclusion + destination preference; prime retry-once +
  double-failure abort.

## Discussion

**The operator's symptom readings vs the mechanisms.** "The hand didn't always move
down" was NOT a prime-timing problem this time — the hand was primed at top at all
six contacts; the strokes were *erased after arming* by our own re-prime. And "no
balls were successfully caught" was worse than it looked: five of those attempts
*reported* CAUGHT. Both misreads were only resolvable from the bag (hand telemetry +
the corrupt tracks' below-floor positions) — neither the action results nor the
node logs carried the truth. The plausibility gate exists precisely so the action's
own verdicts stop lying while the tracker investigation is open.

**Why remove the balls-path prime instead of ordering the two calls.** Ordering
(prime, then arm next tick) still races: both are async service calls into a bridge
queue and a 5 ms balls tick is no spacing at all against CAN scheduling. The
invariant that actually closes the class is *no kind-3 while a catch sequence is
live* — enforceable Jetson-side with one timestamp (`_last_cmd_mono`), testable, and
it survives future re-ordering of the pipeline. The Teensy-side alternative (make
kind-1/kind-3 compose instead of clobber) is better engineering but is firmware —
flash-gated — and the Jetson invariant is sufficient.

**Why the pre-tilt hooks at the coordinator node, not the tracker or the FSM.** The
tracker's announced ball is `TO_BE_THROWN` until release by design (flipping the
coordinator's status filter would perturb blacklist bookkeeping and the hand-arm
gate); the reload FSM shouldn't compute poses (it owns sequencing, not geometry).
The coordinator node already owns the announcement→pose math via
`compute_catch_orientation` — one public wrapper keeps the pose math single-sourced,
and the `_catch_armed` gate on both ends makes the worst-case ordering race (the
announcement beating the armed Bool) a benign skip back to today's behavior.

**What was deliberately NOT done here.** (a) The tracker split-track fix — the
matcher is mis-associating launch detections with the announced track; that is a
real investigation (its own session) and this entry's mitigations are containment,
not cure. (b) Any timing tuning from this bag's numbers: the can-bridge Teensy was
at **67.6 h uptime** (deep in the known degradation regime, reboot experiment STILL
outstanding) — the 0.10–0.14 s arrival lateness and any stroke-timing conclusions
must be re-measured on a fresh-boot plant before being trusted. (c) The
prime-confirmation mystery (why `_hand_primed` never latched) — the retry timer
makes the system robust to it, and the next session's logs will show the
`_on_prime_done` failure message if it recurs.

## Related

- `logbook/2026-07-23-phase7-reload-first-hardware-session.md` — the morning session (z fix, envelope, arming-before-throw)
- Rosbag: `~/Desktop/rosbags/2026-07-23_11-37-57` (7 attempts, 6 throws, hand telemetry at 100 Hz)
- OPEN: tracker split-track corruption (fakes CAUGHT, drives the envelope rejects) — needs its own session
- OPEN: can-bridge Teensy uptime lag — reboot BEFORE the next session (67.6 h at this bag)
- `tests/hardware/session_phase7_reload.md` — runbook updated for the next sitting
