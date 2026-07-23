---
title: Phase-7 third sitting — 8/10 caught at 0.8; false MISSED_INFEASIBLE verdicts, prime-retry stutter, dispatch-failure epidemic, just-in-time pre-tilt — diagnosed and fixed
type: investigation
date: 2026-07-23
status: resolved
phase: "MVP trajectory bringup — Phase 7 reload: third hardware session (catch_vel_scale sweep)"
related_plan: mvp-trajectory-bringup.md
files_changed:
  - config/hardware_config.yaml
  - ros_ws/src/jugglebot/jugglebot/catch_coordinator_node.py
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py
  - ros_ws/src/jugglebot/jugglebot/reload_sequencer.py
  - ros_ws/src/jugglebot/jugglebot/orchestrator_node.py
  - ros_ws/src/jugglebot_interfaces/action/Reload.action
  - ros_ws/gui/js/state-minimap.js
  - ros_ws/gui/css/state-minimap.css
  - tests/ros/test_catch_coordinator_node.py
  - tests/ros/test_reload_coordinator_node.py
  - tests/ros/test_reload_sequencer.py
  - tests/ros/test_orchestrator_node.py
  - tests/ros/test_orchestrator_conduit_contract.py
  - tests/hardware/session_phase7_reload.md
commits: []
subsystem:
  - ros
  - gui
tags:
  - reload
  - catch
  - hand
  - verdicts
  - pre-tilt
  - can-dispatch
---

# Phase-7 third sitting: first real catches (8/10 at scale 0.8); the reporting and smoothness layer, diagnosed and fixed

## Summary

Third sitting, bag `~/Desktop/rosbags/2026-07-23_17-51-09` (13 reload goals: 1
pre-throw abort + 12 sequences; **can-bridge Teensy + Jetson freshly rebooted**
before the session). All throws used `catch_vel_scale: 0.8`. **The catching now
works**: of the 12 sequences, 2 throws were aborted by BB itself (yaw
`THROW_ABORTED_NOT_SETTLED` — the ball never flew), and of the 10 real throws
**8 were physically caught** (hand holding-current step +0.8 A over the empty
baseline) — the operator's "only two missed" count exactly. The session surfaced
four anomalies, all root-caused from the bag + node logs by a parallel forensics
workflow (three code tracers + a rosbag forensics agent + an adversarial
verifier) and fixed this session:

1. **All 12 goals resolved `MISSED_INFEASIBLE_WORKSPACE`** — including the 8
   catches.
2. **The hand stuttered on 5/12 prime ascents** (stalls with velocity reversals
   to −4 rev/s mid-move).
3. **Goal 1 aborted `ABORTED_PRIME_FAILED`** (the operator's "hand not priming
   in time" — again).
4. **The platform arrived at the receive tilt essentially AT landing** (tilt
   error still >1° until 0.24–0.49 s before contact on all 12 attempts) instead
   of being seated well before the ball arrives.

Also locked in: **`catch_vel_scale` default = 0.8** (config-backed,
`JB_OP_CATCH_VEL_SCALE_DEFAULT`), and a **GUI RELOAD button** in the STATE
MACHINE panel (Trigger-service relay in `orchestrator_node` → one `Reload` goal;
rosbridge on Foxy has no ROS2 action transport, so a relay is structurally
required).

## Root causes (verified against bag + node logs + code)

### 1. False `MISSED_INFEASIBLE_WORKSPACE` — two mechanisms composing

Per attempt the feedback stream was: 1 pre-tilt **accept** at announce+0.1 s,
then 4–6 accepts at announce+3.0..3.2 s, then **27–46 WORKSPACE rejects**
(verbatim: "catch target N mm from the armed hold pose exceeds the 80 mm reach
envelope", N walking 81→481 mm) as the **corrupt split-track's** landing
estimate drifted off-platform — the known-open tracker corruption
([2026-07-23-phase7-retest-stroke-race-tracker-corruption](2026-07-23-phase7-retest-stroke-race-tracker-corruption.md)).
`ReloadSequencer.note_catch_feasibility` was **last-writer-wins**, so the reject
stream always ended latched; and the CAUGHT plausibility gate (correctly) vetoed
the corrupt track's below-floor CAUGHT (11 IMPLAUSIBLE warnings across the 12
sequences) while the real ball rode untagged tracks the verdict never consults.
Settle order then produced `MISSED_INFEASIBLE_WORKSPACE` on 12/12. The envelope
was doing its job — the platform held its accepted pose *because* the garbage
was rejected — but the verdict claimed the opposite.

### 2. Prime-ascent stutter — the 0.5 s retry tick restarting a live ascent

All five stalls are **phase-locked to the retry-timer's 0.5 s grid** (mod-0.5 =
0.430–0.451 across A01/A04/A07/A08/A12) and each is preceded 10–20 ms by a
re-dispatch. Mechanism: the edge-prime's **dispatch ack failed**
(`HAND_TRAJ_CMD: ERR_TIMEOUT`, on exactly and only the 5 stuttered attempts), so
`_hand_primed` never latched and `_prime_retry_tick` re-dispatched 0.5 s into a
0.68–1.05 s ascent. A kind-3 re-dispatch rebuilds the Teensy profile from the
live hand position at v(0)=0 — the observed stall + pullback (0.06–0.13 rev) +
velocity reversal. A second, deterministic onset variant: the reload
coordinator's CHECKING prime and this node's catch/armed edge prime restarted a
just-started ascent ~60–90 ms in on 3/12 attempts (the two nodes cannot see each
other's service calls).

### 3. Goal-1 abort — a dispatch-failure epidemic, NOT the ODrive settle window

The operator's hypothesis (ODrive needs ~100 ms after entering
CLOSED_LOOP_CONTROL before honouring commands) is **refuted for this abort**,
three ways: (a) the hand axis was **IDLE from t=8.0 s until t=106.479 s** — no
closed-loop entry preceded the failed primes, (b) a settle-window swallow
returns dispatch *success*, and the abort requires two dispatch *failures*, (c)
the only hand profile in the goal-1 window is the SAFE_ABORT **retract**
(−0.113→0.0 rev), not a prime (target 9.858). Actual cause: **both**
`smooth_move_hand` dispatches returned the firmware-replied
`HAND_TRAJ_CMD: ERR_TIMEOUT` — a `can_jugglebot_send` CAN3-enqueue/partner-gate
failure on the bridge Teensy. This failure class ran **~40–60 % per call all
session** (despite the fresh reboot): 10/13 first prime dispatches failed
(retry-once saved 9), 5 catch-side prime acks failed, and **7/12 SAFE_ABORT hand
retracts failed outright** (the hand silently stayed at top — benign only
because the next goal re-primes). Goal 1 was simply the both-attempts-failed
tail. The settle-window insight stays true of ODrives and is a real latent
first-command-of-session hazard (the retract's first ~60 ms streamed while the
axis was still IDLE), but it lives on the firmware side — deferred to the
epidemic investigation below.

### 4. Just-in-time pre-tilt — an arrival-time policy, not actuator speed

`_on_throw_announcement` set the pre-tilt target's arrival to the predicted
**landing time exactly** (bag: arrival − landing = +0.000..+0.005 s on 12/12),
and `build_catch`'s fixed-lead contract spans the whole lead with ONE min-jerk
quintic — so the ~10.5° receive tilt was a single ~3.9 s crawl completing AT
contact, defeating the pre-tilt's own stated intent.

## Fix

1. **Honest verdicts** (`reload_sequencer.py`): new `_catch_accepted` latch;
   `note_catch_feasibility` now gates on `_throw_sent` (not phase — the pre-tilt
   accept lands while the FSM is still AIMING) and latches infeasibility **only
   while no target was ever accepted**. `MISSED_INFEASIBLE_<code>` now means
   exactly "the platform never had a reachable catch pose".
2. **Anti-stutter in-flight window** (`catch_coordinator_node.py`):
   `_PRIME_INFLIGHT_S = 1.2` — no re-prime (edge or tick) may be dispatched
   within 1.2 s of the last prime dispatch, from EITHER owner: the reload
   coordinator announces every ACTION_PRIME_HAND dispatch on the new
   `catch/prime_dispatched` topic. The window is anchored to **dispatch, not
   ack** (failed acks were observed with the frame still transmitted and the
   hand moving). Post-window re-dispatch at top is a Teensy no-op, so a
   genuinely lost dispatch still recovers on the next tick.
3. **Dispatch retry ladders** (`reload_coordinator_node.py`):
   `_HAND_DISPATCH_ATTEMPTS = 4` with 0.15 s gaps for the prime (drops the
   goal-abort probability from ~16 % to ~3 % at the ~40 % end of the observed
   failure band) and — because it is the safing path — the same ladder for the
   SAFE_ABORT retract, whose catch/armed disarm now goes out FIRST so the
   prime-retry tick cannot erase an in-flight retract (audit finding).
4. **Early pre-tilt** (`catch_coordinator_node.py`): pre-tilt arrival =
   landing − `_PRETILT_EARLY_S` (1.5 s), clamped to ≥ now + 1.0 s and ≤ landing.
   Verified interactions: the reach-freeze window re-anchors to the refinement's
   own arrival after the settle-hold release; the envelope center (captured at
   the PREPARE latch raise, before the announcement) is unaffected; mid-flight
   refinements stay just-in-time as designed.
5. **`catch_vel_scale` locked at 0.8** (`hardware_config.yaml` →
   `JB_OP_CATCH_VEL_SCALE_DEFAULT`): goal field 0/unset resolves to 0.8, the
   disarm-edge reset restores 0.8, negative-typo fallback uses it, and the
   per-goal override still applies.
6. **GUI RELOAD button** (`state-minimap.js/.css` + `orchestrator_node.py`):
   `jugglebot/reload_request` (std_srvs/Trigger) relay → one fire-and-forget
   `Reload` goal (`throw_delay_s=3.0`, `catch_vel_scale=0.0` ⇒ config default).
   Enabled only when connected + ACTIVE; outcome logged by the relay callbacks.
   The reload coordinator remains the sole authority on preconditions and
   outcomes — an ill-timed click is rejected with an honest code.

## Discussion

**A hypothesis was withdrawn.** The operator attributed goal-1's abort to the
ODrive mode-change settle window; the bag refuted it (axis IDLE throughout — no
mode change had happened) and replaced it with the dispatch epidemic, which the
fresh-reboot context makes *more* alarming, not less: this is not the uptime-lag
regime, it is a distinct CAN TX-path failure. We did NOT implement a 100 ms
settle wait anywhere: the only place it would matter is bridge/Platform-Teensy
firmware (flash-gated), and it would not have saved any observed failure this
session. It is queued into the epidemic investigation instead — implementing it
Python-side today would have been action-bias theatre against the evidence.

**Why verdict option "accepted-target wins" over the alternatives.** (i) "only
the LAST pre-landing decision counts" is fragile against reject-stream timing
(the corrupt track keeps rejecting right through landing on most attempts — the
last decision before landing was WORKSPACE on 12/12, so it would have changed
nothing); (iii) "wait for the ball-held sensor" leaves three known-false verdict
classes standing for an unknown number of sittings. Option (ii) states a truth
the trajectory layer already enforces: an accepted target means the platform IS
holding a reachable catch pose, so later rejects cannot mean "the platform never
reached". Its known cost: a flight whose *refinements* were all garbage but
whose pre-tilt was accepted reads MISSED rather than MISSED_INFEASIBLE — accep-
table, because the pre-tilt pose is a genuine catch attempt (all 8 of today's
catches were won by the feed-forward pose plus its 4–6 early accepted
refinements — all within the 80 mm envelope of the hold pose — while every LATE
refinement was rejected as the corrupt track drifted out).

**Why the anti-stutter window keys off dispatch, not ack.** The forensics
surfaced acks failing while the frame still went out (A04's edge prime: ack
failed at 155.748, profile restart at ~155.75; A05's ascent begins at the
instant of its *failed* first dispatch). An ack-anchored window would re-open
the exact stutter it exists to close whenever the ack lies. Cost: a truly-lost
dispatch waits out the 1.2 s window before the tick recovers it — inside the
3 s countdown with margin. The honest fix for the ack channel itself (and a
telemetry-based `_hand_primed` latch) belongs to the epidemic investigation.

**Deliberately deferred, with owners:**
- **HAND_TRAJ_CMD ERR_TIMEOUT epidemic** — its own investigation next session
  (bridge `can_jugglebot_send`: partner_recent staleness gate vs FlexCAN TX
  contention with the 500 Hz leg-interp stream; the `[canhealth] tx_gated`
  counter disambiguates; also bench-pin the failed-ack-but-frame-sent race
  before trusting the ack channel for anything).
- **BB yaw `THROW_ABORTED_NOT_SETTLED` on 2/12 throws** — BB-side; the throw
  COMPLETION result is logged by the bridge but not consumed by the reload FSM,
  so those goals read MISSED instead of a truthful "BB aborted the throw".
  Wiring that signal into the FSM is a candidate for the next code round.
- **Envelope-center subtlety** — the 80 mm envelope is centred on the
  pre-pre-tilt hold pose, so even REAL-ball refinements at genuinely off-centre
  crossings get rejected (A07's real ball crossed 124 mm off-centre and was
  still caught by the feed-forward pose). Decide deliberately (keep / re-centre
  at announcement / widen) when the tracker fix lands.
- **Tracker split-track corruption** — still the root-cause blocker for honest
  CAUGHT verdicts; unchanged priority. The operator's imminent **ball-held
  sensor** (switch on a hand-ODrive GPIO, BB-pattern) is the clean long-term
  verdict source and will subsume most of the plausibility-gate machinery.

## Verification

- Scoped suites during development: `pytest tests/ros/test_reload_sequencer.py`
  (35 passed), `tests/ros/test_catch_coordinator_node.py` (35 passed), the four
  reload/catch files together (107 passed in 4.39 s), all run 2026-07-23.
- GUI relay: `pytest tests/ros -q` (run 2026-07-23, by the GUI implementation
  agent): **973 passed in 52.71 s**.
- Full pre-commit gate, run twice: `pytest tests/ -q` (run 2026-07-23):
  **2948 passed, 1 xfailed, 0 failed in 921.00 s** (pre-audit-fix tree), then
  after the audit's safe-abort reorder: **2948 passed, 1 xfailed, 0 failed in
  770.60 s** on the final tree (same run date, same invocation).
- Forensics artifacts: workflow run `wf_79d6f5fb-5bf` (4 investigators + 1
  adversarial verifier over bag `2026-07-23_17-51-09` + node logs); per-agent
  results in the session's workflow journal.

## Related

- [2026-07-23-phase7-retest-stroke-race-tracker-corruption](2026-07-23-phase7-retest-stroke-race-tracker-corruption.md) — the prior sitting (race + tracker corruption; pre-tilt + vel-scale introduced)
- [2026-07-23-phase7-reload-first-hardware-session](2026-07-23-phase7-reload-first-hardware-session.md) — the z double-add + arming-before-throw round
- `tests/hardware/session_phase7_reload.md` — fourth-sitting runbook
