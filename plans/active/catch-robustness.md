---
title: Catch robustness — sensor-truth verdicts, hand-drive restoration, toss self-tuning
created: 2026-08-10
status: active
related_logbook:
  - 2026-08-10-hand-drive-braking-clamp-diagnosis.md
related_code:
  - ros_ws/src/jugglebot/jugglebot/ball_possession.py
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py
  - ros_ws/src/jugglebot/jugglebot/toss_sequencer.py
  - tools/probes/hand_stroke_timeline.py
  - plans/active/hand-ball-sensor.md
  - plans/active/single-ball-toss.md
  - plans/active/hand-command-continuity.md
---

# Plan — Catch robustness programme

**Branch:** `mvp-trajectory-bringup`
**Goal:** highly repeatable single-ball tosses across most of the workspace at
z = 170 mm — catch success approaching 100 %, with the hand ball sensor as the
ground truth for whether (and when) a ball was caught.
**Owner decisions (2026-08-10, in-session):** hand sensor becomes the possession
source of truth AND the ball-evidence gate (`toss_require_ball_evidence` → true);
learning shape = (c) persistent calibration map updated only by an explicit
calibration routine, plus a session-local real-time trim layered on it;
`TossContinuous` should auto-reload from BB on drops. The hand-ball-sensor
plan's "flip forbidden until Phase 7 validates" gate is SUPERSEDED by this
decision — the sensor is operator-validated in situ (Phase 7 steps 4–5 remain
open as bench work, not blockers).

## Context — the 2026-08-10 diagnosis

The 8a retest (tilt map live, catch_vel_scale 0.9) caught most throws but showed
a returned post-throw dip on every 1.0 m throw, end-stop bumps, and ~60 %
catch rate at 1.0 m. Offline diagnosis from the three retest bags
(16-04-26 / 16-13-48 / 16-30-44, ~16 h can-bridge uptime):

- **The dip is NOT the 2026-07-25 clobber class returning.** 54/54 throws at
  4.44 m/s show `trunc = -`, `seeds = 0` — the Phase-1 arm gate holds. The
  FW-2 decel FF **is** commanded (`tor_ff_cmd` −0.1500 Nm = `throwDecelToTorque`
  at this tier; legacy would read −0.11).
- **The drive under-delivers braking**: over the 54 decel ramps `iq_meas` reads
  −10.52/−7.60/−3.85 A (min/med/max) against the **−27.2 A** the commanded
  −0.1500 N·m asks for at the hand's Kt 0.0055133, and the whole-bag negative
  floor is −11.8 A while the positive side runs to +49.6 A. One-sided, with a
  hard floor ⇒ drive-side braking limit on the hand ODrive — swapped during the
  2026-07-31 CAN3 elimination, on a machine whose ODrives occasionally need
  config resets. **The leading candidate is named and pre-registered**:
  `axis0.config.torque_soft_min = −0.055133 N·m` = **exactly −10.00 A**
  (asymmetric against `torque_soft_max` +0.5 N·m) — C-HAND-2 § *The negative
  torque clamp*, bench pre-flight **H7.0c**. Its 2026-07-27 counter-evidence
  was measured on the *pre-swap* drive and does not transfer. Regen
  (`dc_max_negative_current`) and a reset `torque_constant` stay in the class
  until the dump rules them out.
- Consequences: coast +0.51…+0.81 rev past x3, position-loop pullback +
  FW-2 braking prelude ⇒ dip below x3 0.27–1.67 rev (median 0.90) on 54/54;
  peaks to 10.766 rev. **The same clamp plausibly degrades the catch seat
  (also regen-direction) — catch knobs must not be tuned against this plant.**
- Dispatch shift +118–133 ms at ~16 h uptime (fresh-boot discipline stands).
- The sensor ledger in the bags is 100 % valid; bag 16-30-44 alone yields
  39 departures / 38 catches / 3 quick-drops (<1.5 s) — automatic bounce-out
  labels, mined offline before any wiring existed.

## Phases

| Phase | Scope | Gate | Status |
|---|---|---|---|
| 0 | **Bench (operator): hand-drive restoration.** **H7.0c first** — read `axis0.config.torque_soft_min` off the LIVE hand axis (30 s, `odrivetool`; PASS is `≤ −0.20 N·m`, anything above that is the finding); then dump + diff the whole hand-ODrive config vs `config/ODrive config Files/odrive_pro_hand_config.json` (regen / `dc_max_negative_current`, `torque_constant`, `gpio2_mode`); restore; fresh can-bridge boot; HAND-7 R0–R5 decel ladder; 1.0 m retest | dip_below_x3 ≤ 0.10 rev; peak ≤ 10.060 rev; braking iq tracks commanded | PENDING (next sitting) |
| 1 | **Sensor-truth possession + ball-evidence gate.** `HandBallSensorSource` behind the C-POSSESS-1 seam (tri-state honest; UNKNOWN never collapses to True); sensor-primary verdicts on toss + reload; catch-event time surfaced; `toss_require_ball_evidence` → true (UNKNOWN refuses, loudly); bag-replay validation vs the three 2026-08-10 bags | full gate + bag-replay reproduces the mined labels | **SOFTWARE COMPLETE + AUDITED 2026-08-10** — contract extended (C-POSSESS-1 §§ 2.1/3.2/3.3), `HandBallSensorSource` + `merge_possession` landed, gate flipped, `tools/probes/hand_sensor_verdict_replay.py` reconciles EXACTLY with the independent transition counts on all three bags (35 CAUGHT / 46 MISSED / 0 UNKNOWN over 81 announcements). `logbook/2026-08-10-sensor-truth-possession.md`. **Audit fixes landed same day** (entry § Audit fixes): the operator scoring rows this phase inverted are rewritten (runbook `POSS-1`/`POSS-1.3` had a reload `CAUGHT` — the headline capability — in their ABORT column), `describe()` stops attributing a SENSOR veto to the tracker, and the retention window gains the absolute widen guard it shipped without. **Three items carried to the bench, unfixed on purpose**: (a) the reload's `ACTION_RECENTER` terminal executes on hardware for the first time — ungated REPORT row `POSS-1.7`, gating it is an operator call; (b) `stale_s`'s stated justification vs the real 3.0 s bridge bound (design fork); (c) `_ball_possession` is write-only state, so § 3.3 edit 2 has no consumer. **Bench validation of the UNKNOWN paths is outstanding** — 203,922 real samples were 100 % valid, so every blind/stale path is test-only (row `POSS-1.8`: kill the SDO poller, not the link) |
| 2 | **Toss self-tuning loop (design → operator review → build).** Per-toss record schema; session-local trim + persistent calibration map (explicit calibration routine only); TossContinuous auto-reload-on-drop; catch-knob A/B ladders scored by sensor labels | design reviewed by operator before build | DESIGN IN PROGRESS |
| 3 | **Bench: re-baseline + tuning sittings.** Catch-rate baseline on the restored plant, then workspace grid tosses with the loop | ≥ target catch rate across the grid | PENDING (after 0–2) |
| — | **Open:** recurring ODrive config-drift guard (launch-time SDO config assertion vs session-runbook probe — design fork, not yet chosen); bridge-uptime lag root cause (owned by refactor-2026-07 Phase 7); **if Phase 0 finds the clamp was live, C-HAND-2's `J ≥ 1.0126e-5 kg·m²` was measured through it** and `throw_decel_reflected_inertia_kgm2` must be re-derived on the restored drive (discriminator: does achieved decel become tier-independent?) | | |

## Constraints

- **No catch-parameter tuning until Phase 0 lands** — the 60 % baseline was
  measured on a degraded plant (braking clamp + 16 h uptime).
- Safety forks stay closed: `MIN_TOSS_THROW_DELAY_S = 3.5` floor untouched;
  hand ladders + `_MAX_ARM_DISPATCHES` retained; kind-3 abort clobber rights
  untouched (C-HAND-1 "What must NOT change").
- Fresh can-bridge boot before every sitting; `uptime_ms` logged with every
  timing-sensitive number.
- All tuning data collection under tier 8a at z = 170 mm, ±150 mm workspace.
