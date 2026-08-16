---
title: The hand ball sensor becomes the possession source of truth — tri-state ARRIVAL, a source merge, and a live ball-evidence gate
type: feature
date: 2026-08-10
status: resolved
phase: "catch-robustness Phase 1"
related_plan: catch-robustness.md
files_changed:
  - ros_ws/docs/ball_possession_contract.md
  - ros_ws/src/jugglebot/jugglebot/ball_possession.py
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py
  - ros_ws/src/jugglebot/jugglebot/toss_sequencer.py
  - ros_ws/src/jugglebot/jugglebot/reload_sequencer.py
  - ros_ws/src/jugglebot_interfaces/action/Toss.action
  - config/hardware_config.yaml
  - tools/probes/hand_sensor_verdict_replay.py
  - tools/probes/data/hand_sensor_replay_fixture.json
  - tools/probes/README.md
  - tests/ros/test_ball_possession.py
  - tests/ros/test_hand_sensor_replay.py
  - tests/ros/test_toss_coordinator.py
  - tests/ros/test_toss_sequencer.py
  - tests/ros/test_reload_coordinator_node.py
  - plans/active/catch-robustness.md
  - plans/archived/hand-ball-sensor.md
  - plans/active/single-ball-toss.md
  - tests/hardware/session_phase8_toss_hardware.md
  - tests/hardware/session_phase8_toss_trace.md
  - tests/hardware/session_anomaly_fixes.md
  - tests/hardware/session_phase7_reload.md
  - logbook/INDEX.md
subsystem:
  - ros
tags:
  - contract
  - testing
  - safety
---

# The hand ball sensor becomes the possession source of truth

## Summary

The ball-in-cup hand sensor (G02 on the hand ODrive, wired and published since
2026-07-29) is now the **PRIMARY** possession source. `TrackerArrivalSource` is
demoted to the arrival corroborator, the ball-evidence CHECKING gate defaults ON
and reads the sensor **live**, and the machine has, for the first time, a direct
measurement of *when* a ball entered the cup.

Three things landed together, per the repo's contract pattern:

1. **Normative** — `ros_ws/docs/ball_possession_contract.md` gains § 2.1
   (ARRIVAL is tri-state), § 3.2 (the tick-driven source kind + the merge rules)
   and § 3.3 (the ball-evidence precondition is a live read). § 7's accepted
   bounce-out trap and § 7.1's two named coordinator edits are closed.
2. **Enforcement** — `ball_possession.HandBallSensorSource` (pure Python) and
   `merge_possession`, reached only through
   `ReloadCoordinatorNode._possession_confirmed`; the live `evidence(now)` read
   in `_build_toss_observations`.
3. **Tests** — 31 new pure-verdict/merge tests, the CHECKING refusals at both FSM
   and node level, and a bag-replay test on a committed fixture cut.

Operator authorisation for the flip (2026-08-10, recorded in
`plans/active/catch-robustness.md` § Owner decisions): the sensor is validated in
situ, so the hand-ball-sensor plan's *"flip forbidden until Phase 7 validates"*
gate is **superseded**; Phase 7 steps 4–5 stay open as bench tuning, not blockers.
The supersession is recorded at that plan's own statement of the gate.

## The measurement the windows are built on

Three retest bags — `~/Desktop/rosbags/2026-08-10_{16-04-26,16-13-48,16-30-44}`,
**203,922 `/hand_telemetry` samples, 100 % `ball_held_valid`**, 81 announcements —
replayed through the production source by

```
python tools/probes/hand_sensor_verdict_replay.py \
  --bag 2026-08-10_16-04-26 --bag 2026-08-10_16-13-48 --bag 2026-08-10_16-30-44 \
  --merge-tracker refuse --json
```

| bag | samples | valid | empty→held | held→empty | quick-drops | CAUGHT | MISSED |
|---|---|---|---|---|---|---|---|
| 16-04-26 | 47,124 | 100 % | 5 | 6 | 0 | 4 | 10 |
| 16-13-48 | 86,132 | 100 % | 24 | 23 | 0 | 6 | 21 |
| 16-30-44 | 70,666 | 100 % | 38 | 39 | 3 | 25 | 15 |

The transition counts reconcile **exactly** with the orchestrator's independent
hand count on all three bags. Announcement count is not throw count — BB reload
throws announce too, and both throwers' catches land in the same cup, so all 81
are scored and none is a catch *rate*.

**ARRIVAL band.** Over the 35 announcements whose next empty→held edge opened a
segment ≥ 1.5 s, that edge landed **+137 … +798 ms** after the announced
`landing_time` (median +399 ms) and **never before it**. The nearest non-catch
edge is **+3194 ms**. Shipped: `arrival_lead_s 0.20`, `arrival_window_s 1.50` —
1.9× above the band top, 2.1× below the re-seat floor. The window is deliberately
not trimmed to the band: the physical release lags its announcement by a
can-bridge-**uptime**-dependent amount (+54–63 ms fresh on 2026-07-27, **+118–133
ms at ~16 h** on these bags), so a window fitted to a fresh-boot session starts
refusing real catches as a sitting wears on.

**RETENTION window.** The longest seat-then-leave the sensor resolved is
**0.999 s** (three of them: 0.571 / 0.989 / 0.999 s). Shipped 1.50 s = 1.5×.
The upper constraint is the machine's own cadence: `MIN_TOSS_THROW_DELAY_S` is
3.5 s, so the window closes 2.3× before the earliest legitimate departure a throw
can produce.

> **The provenance of those three, stated honestly because the phase brief called
> them "bounce-out ground truth".** They all sit in bag `16-30-44` at
> t ≈ 292/298/299 s with `pos_meas ≈ 0` (hand parked at the bottom) inside a 54 s
> window containing **no announcement** — i.e. the operator hand-loading the cup,
> not post-catch bounce-outs. They are therefore ground truth for the property the
> retention window actually rests on — that the sensor *resolves* a sub-second
> seat-then-leave — and are **not** evidence about how a real bounce-out behaves.
> The three bags contain no post-catch bounce-out at all (0 BOUNCE labels), so the
> retention rule ships un-exercised against its target failure mode. That is a
> named residual, not a silent one.

## Discussion

### Retention deliberately does NOT gate the catch verdict

The obvious design — hold the catch verdict until `RETENTION_CONFIRMED` — was
written out and rejected. C-POSSESS-1 § 3 already records that *a source cannot
answer late*: `toss_sequencer._step_in_flight` finishes the goal on the first
confirmed tick. Gating on retention would hold the toss's terminal open for the
whole 1.5 s window, which is a real **actuation-timing** change on the
`RECENTER`/`STAY` path — the one path whose safety argument (§ 5 of the contract:
the hand is already parked, `go_home` fires with the ball seated) was written
against today's timing, and the one the plan's own constraint forbids re-tuning
until the hand-drive braking clamp is fixed (Phase 0).

So a fresh arrival mints `RETENTION_UNKNOWN`, which § 2 consequence 3 forbids from
vetoing, and retention binds where it can still *act*: the next cycle's
precondition and the possession latch. The consequence, stated so nobody thinks
the trap is fully gone: the one tick that reports CAUGHT over a ball that bounced
out immediately afterwards **survives as a reporting error**. What is gone is the
actuation consequence — cycle N+1 firing an empty stroke, which is what § 7.1 was
about.

### The sensor overrules the release-evidence latch clear, not the other way round

`_build_toss_observations` already cleared `_ball_possession` on release evidence
(the throw-stroke telemetry signature after dispatch). The sensor's `EMPTY`/`SEATED`
edits are applied **after** that, so on a tick where both speak, the sensor wins.
That ordering matters and it is not incidental: the stroke signature fires during
the *ascent*, and the ball does not leave the cup until the release point near the
top — so at that instant a healthy sensor still reads HELD, and it is right. The
release clear is a prediction; the sensor is an observation. Letting the prediction
win would put the machine back to believing a model over the cup, which is the
whole failure the sensor was made primary to end.

### `arrival_ok` became a property, and the bool was a latent instance of § 1's defect

`PossessionVerdict.arrival_ok` was a `bool`, which quietly violated the contract it
enforces: § 2 requires `UNKNOWN` for any part a source does not positively observe,
and a bare bool forces *"I could not look"* and *"I looked and it did not arrive"*
onto one value. Inert while the only source always had an estimate in hand;
reachable the moment a source can be blind. Rather than adding a tri-state field
*beside* the bool — two representations of one truth, exactly how contracts rot —
the field was **replaced** by `arrival: str` with `arrival_ok` as a derived
property. Dual truth is then unrepresentable rather than merely discouraged, and
`test_arrival_ok_is_a_projection_not_a_field` asserts the constructor rejects the
old spelling. Blast radius was four construction sites (grep-verified), all
same-arity.

### The arrival window is passed per query, never latched

The first design gave the source a `note_expected_landing()` arm/disarm pair. It
was deleted before it shipped: a latched window outlives its goal, and a window
that outlives its goal can veto the *next* ball's tracker CAUGHT with a stale
"nothing arrived". Rather than guard the lifecycle in three teardown paths
(`_build_toss_cycle`, `_clear_toss_cycle_state`, the reload reset), `observe(now,
landing_t)` takes the window from the caller every time and the whole bug class
stops existing. The node reads it from `seq.landing_perf` — a new public property
on **both** FSMs, NaN until a release is stamped, so a query before anything is in
the air answers `ARRIVAL_UNKNOWN` instead of looking around t = 0.

### Both FSMs are wired, including the reload

The reload path is wired to the same merge, which makes its verdict reachable for
the first time (§ 4: every destination-tagged reload track in the reference capture
is a split track 204.9–752.9 mm out, so the tracker refuses catches the operator
watched land). This turns on the reload's `CAUGHT` terminal, which has been dead
code on hardware — the same class of newly-live path § 5 analysed for the toss, and
the same terminal shape (`RECENTER`: lower the latch, `go_home`, no retract). It is
listed as a bench-watch item rather than argued away here. The measured BB catches
in the bags (+137…+422 ms) sit inside the same arrival band as the self-tosses, so
no separate window is needed.

### `REJECTED_BALL_UNKNOWN` is a second code, not a reuse of `NO_BALL`

An operator who reads `NO_BALL` goes hunting for a ball; the fault in the UNKNOWN
case is the sensor. One code for both would send them to the wrong subsystem and —
worse — would hide that UNKNOWN **refuses**, which is the fail-closed choice this
project made explicitly against BallButler's fail-open `ball_in_hand_ = true` boot
default (a recorded live bug in that repo). `toss_require_ball_evidence: false`
remains the operator's total-bypass escape hatch, and is tested as one.

## Fix

- `ball_possession.py`: `ARRIVAL_*` / `EVIDENCE_*` vocabularies;
  `arrival_ok` → derived property; `HandBallSensorSource` (blind-span tracking, a
  bounded edge log, tri-state `evidence`/`observe`, `arrival_time`);
  `merge_possession`; `describe` gains an UNKNOWN shape and quotes the reason.
- `reload_coordinator_node.py`: the sensor is fed from `_on_hand_telemetry`;
  `_possession_confirmed` becomes the merge point and keys its once-per-verdict log
  on the arrival **state**; `_build_toss_observations` takes ONE live evidence
  read and derives `ball_seated` / `ball_evidence` / `catch_event_dt_s` from it;
  `_expected_landing_perf()`; `_log_toss_outcome` prints `catch_dt`.
- `toss_sequencer.py`: `ball_evidence` + `catch_event_dt_s` observations,
  `catch_event_dt_s` on `TossResult`, the `REJECTED_BALL_UNKNOWN` branch,
  `landing_perf`. `reload_sequencer.py`: `landing_perf`.
- `config/hardware_config.yaml`: `toss_require_ball_evidence: true`; three new
  `jugglebot_ball_detect` keys (`arrival_lead_s`, `arrival_window_s`,
  `retention_window_s`) + regenerated artifacts.
- `tools/probes/hand_sensor_verdict_replay.py` + its committed fixture cut and
  README row.

## Audit fixes (2026-08-10, same day, post-merge)

An independent audit of the three phase commits (`9caf6bc` / `2716e3b` /
`6920e88`) found the logic sound — the gate refuses on EMPTY **and** on UNKNOWN,
`RETENTION_UNKNOWN` never collapses into success, no lock-order inversion, the
arm/catch latch and `_MAX_ARM_DISPATCHES` untouched — and the real exposure in
**reachability, not logic**. What was fixed:

1. **The operator scoring rows were inverted by this very phase** (the HIGH one).
   `tests/hardware/session_anomaly_fixes.md` row **POSS-1** listed *"a reload
   reading `CAUGHT`"* in its ABORT column and its reload PASS criterion read
   *"gate `CAUGHT` count 0 … every attempt logs one `possession REFUSED` line
   (that IS the pass)"* — i.e. the phase's headline capability scored as a
   failure. Rewritten in place, along with `POSS-1.3`, standing rule 3's reload
   bullet, `POSS-1.6`'s "all refused" wording, and the same stale expectation in
   `tests/hardware/session_phase7_reload.md` item 5. Two rows added: **POSS-1.7**
   (the reload's `CAUGHT` terminal, REPORT and deliberately ungated — see Open)
   and **POSS-1.8** (the UNKNOWN paths, with the *"kill the SDO poller, not the
   link"* recipe, because `hand_fresh` gates before `ball_seated`). A section
   banner now states the inversion before any row is scored, and the deployment
   pre-flight gained the sensor-half greps + a print of the four live constants.
   The `/audit --unstaged` pass over those edits then found **five more instances
   of the same inversion** that the finding had not named, all corrected: three
   surviving *"reload catches still read `MISSED`, correctly"* sentences (two in
   `session_anomaly_fixes.md` §§ CCATCH-2t / LVL-5, one in
   `session_phase8_toss_hardware.md`); `CONT-STEP-1`'s preamble, which still
   opened *"Because `toss_require_ball_evidence` is `false` … it does not
   refuse"* two lines above the banner saying the opposite; and **`CONT-1.7`**,
   whose PASS (`COMPLETED`, 3 throws, 3 MISSED cycles on an **empty cup**) is
   unreachable under the shipped gate — an empty cup now gives
   `ABORTED_CYCLE_REJECTED_NO_BALL` at cycle 1 and the whole dry-trace step
   returns no data. The § POSS command block also gained the `SENSOR_BLIND`,
   `Reload CAUGHT` and `REJECTED_BALL_UNKNOWN` greps and the sensor-half probe
   invocation, because the old block only counted lines whose meaning changed.
2. **`describe()`'s REFUSED branch attributed every refusal to the tracker** —
   the exact mirror of the wart the CONFIRMED branch was fixed for in this same
   phase, and on the path the phase was *built* to enable. A sensor veto of a
   tracker CAUGHT (§ 3.2 rule 2 — the headline capability) printed
   `arrival 3 mm > 70 mm from the catch point`: arithmetically false, and it
   routes the operator to the wrong subsystem on the row they score the sitting
   with. Both branches now share `_tracker_cross_check`, which decides
   agrees/DISAGREES by comparing what the tracker *would* have said with what the
   verdict *does* say, so the two cannot drift apart again. Pinned by
   `test_describe_attributes_a_sensor_veto_to_the_sensor` (the mirror of the
   existing CONFIRMED-side test).
3. **The retention window shipped with no widen guard.** Every retention case in
   the probe's self-check was written *relative* to `RETENTION_WINDOW_S`
   (`hold_s=RETENTION_WINDOW_S + 0.2`), so it followed the constant wherever it
   went and only a *narrowing* could fail. Added an absolute twin (a 1.55 s hold
   is a keeper at the shipped 1.50 s window, a BOUNCE at ≥1.56 s) — 10 cases now,
   counted programmatically rather than hard-coded — plus a parametrised mutation
   test over 1.6 / 5.0 / 0.9 s. Widening is the direction that matters: it invents
   bounce-outs on the one rule with no real bounce-out behind it.
4. **`test_the_three_measured_seat_then_leaves_are_in_the_ledger` sat on its own
   boundary.** `RETENTION_WINDOW_S / max(durations)` = `1.5 / 1.000` = exactly
   1.5 against a `>= 1.5` assert, while the `approx(…, abs=0.002)` above it admits
   1.002 — green today, red on a rounding wobble nobody caused. The margin is now
   asserted against the *pinned* longest, and the pin owns the re-baseline
   question.
5. **Four doc corrections**: `ReloadSequencer.landing_perf`'s *"the sensor window
   is ~10x the gap between the two"* is **~2x** (the gap is the ball's whole
   time of flight, ~0.6–0.7 s, against a 1.50 s window) — restated with the
   failure direction (a real catch read MISSED, never a false CAUGHT) and pointed
   at the bench-watch item; the probe header claimed to print *"the verdict the
   coordinator would have minted"* while defining `CAUGHT` as arrival **and**
   retention CONFIRMED, so a `BOUNCE` row is really a live `CAUGHT` (§ 7
   residual) — stated, in the module and in `tools/probes/README.md`;
   C-POSSESS-1.B's normative signature gained the `landing_t` parameter that is
   the clause's load-bearing design element; `Toss.action`'s header documents
   `REJECTED_BALL_UNKNOWN` beside `REJECTED_NO_BALL` (comment-only — no IDL
   surface change, so no `jugglebot_interfaces` rebuild is implied, and the
   runbook's mandatory two-package build covers it anyway).
6. **Two undocumented behaviours in `HandBallSensorSource`**: the `_edges` (64) /
   `_blind_spans` (32) eviction — a `deque` drops oldest-first, so overflow can
   only discard evidence older than the live windows, but an evicted blind span
   fails **OPEN** in the retention direction, so size up not down; and the
   two-sample warm-up (the first `note_sample` has no predecessor, so its gap is
   infinite and it opens a blind span — `evidence()` needs two samples, ~10–20 ms
   at 100 Hz, in the conservative direction).

Three findings were **not** fixed and are carried as open items below: the reload
`CAUGHT` terminal's first hardware execution (operator decision, not a code fix),
the `stale_s` comment vs the real ball-evidence age bound, and `_ball_possession`
being write-only state.

## Verification

- Probe self-check (`python tools/probes/hand_sensor_verdict_replay.py
  --self-check`, run 2026-08-10): **10/10 cases pass, exit 0** (was 9/9 before the
  audit added the absolute retention widen guard).
- Three-bag replay (command above, run 2026-08-10): **35 CAUGHT / 46 MISSED /
  0 BOUNCE / 0 UNKNOWN** over 81 announcements; catch band +137…+798 ms; ledger
  reconciles exactly with the independent transition counts (table above).
- Full gate (`./run_tests.sh`, run 2026-08-10): see the commit messages for the
  count triple of each commit.

## Open

- **Every UNKNOWN path is test-only.** 203,922 real samples were 100 % valid, so
  the blind/stale branches — and `REJECTED_BALL_UNKNOWN` itself — have never fired
  on hardware. First bench session with the gate live is what validates them.
- **The retention rule has never seen a real post-catch bounce-out** (0 BOUNCE
  labels in three bags). Sized against seat-then-leave events of the right shape
  but the wrong provenance; see the § note above.
- **The reload's `CAUGHT` terminal is newly live.** Same class as the toss change
  § 5 analysed, but not separately measured — watch it at the next sitting. The
  audit rated this the second HIGH and left it deliberately unfixed: making the
  sensor primary turns `ReloadResult.success` True for the first time, so
  `ACTION_RECENTER` (lower latch + `go_home`, **no** hand retract) executes on
  hardware for the first time in the machine's history. Whether to gate it —
  and on what — is an **operator decision**, not a code fix; runbook row
  **POSS-1.7** now makes the path visible and records the three things to watch
  without inventing a threshold. Its window anchor is also weaker than the
  toss's: `ReloadSequencer.landing_perf` falls back to `throw_time + throw_delay`
  before BB's announcement lands, ~2x inside the window rather than the toss's
  exact release-derived instant (docstring corrected — see Audit fixes 5).
- **The `stale_s` comment does not survive the second hop.** The constructor
  passes `stale_s=_HAND_STATE_STALE_S` (0.5 s) to answer *"is the Jetson still
  hearing the sensor"*, but `/hand_telemetry` is a free-running 100 Hz timer over
  a **cache** in `teensy_bridge_node`, so the gap term almost never fires and the
  real bound on ball-evidence age is the bridge's `_HAND_SENSOR_RX_FRESH_S` =
  **3.0 s** plus the firmware's 240 ms `REPLY_STALE_US`. Left unfixed on purpose:
  the fix is a design choice (publish the RX age, or tighten the bridge's own
  bound), not a comment edit, and the current behaviour is not wrong — only its
  stated justification is.
- **`_ball_possession` is now write-only state.** With the gate ON,
  `ball_seated` is derived from the live `evidence()` read and the latch has no
  consumer; C-POSSESS-1 § 3.3 edit 2 ("a valid EMPTY clears the latch, a valid
  SEATED sets it") is therefore enforced on state nothing reads, and the write
  only runs inside `_build_toss_observations`, which is not ticked during the
  `TossContinuous` DWELL that § 3.3 names as the failure it fixes. Nothing
  misbehaves today; the contract's third leg is only nominally present for that
  clause. Decision needed: give the latch a consumer, or amend § 3.3 to record it
  as diagnostic-only.
- **The one-tick CAUGHT-over-a-bounced-ball reporting residual** stands
  (Discussion § 1). Runbook row POSS-1.2 stays a REPORT row.
- **BallButler's generated `hardware_config.h` is stale in that repo** — it was
  already stale before this session (the 2026-08-10 tier-8a / catch-vel-scale /
  levelling-settle changes were never propagated), and any `generate_config.py`
  run dirties it. Not committed here: no BallButler firmware consumes the changed
  symbols, and a sister-repo commit is not this session's to make.
