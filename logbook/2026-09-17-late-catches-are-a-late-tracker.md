---
title: "Late catches are a late tracker — the guard chain, the step gate, and the ballistic landing fit"
type: investigation
date: 2026-09-17
status: done
phase: "two-ball-skill-stack — R3"
related_plan: two-ball-skill-stack.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/tracking/flight_fit.py (new — FlightFit gravity-fixed batch ballistic landing estimator, freeze-on-descent admission, two-pass outlier gate)
  - ros_ws/src/jugglebot/jugglebot/tracking/tests/test_flight_fit.py (new, 10 tests)
  - ros_ws/src/jugglebot/jugglebot/tracking/matcher.py (feeds FlightFit the raw matched marker; landing prediction prefers the fit, falls back to the KF unconverged)
  - ros_ws/src/jugglebot/jugglebot/ball_tracker_node.py (passes the 3 new flight_fit config keys through, logs at startup)
  - config/hardware_config.yaml (+ generated artifacts below — flight_fit_min_samples/residual_mm/freeze_above_plane_mm)
  - config/generated/hardware_config.h
  - config/generated/hardware_config.py
  - ros_ws/src/jugglebot/CatchingCone_code/hardware_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/hardware_config.h
  - ros_ws/src/jugglebot/Teensy_code_platform/hardware_config.h
  - ros_ws/src/jugglebot/jugglebot/hardware_config.py
  - tests/ros/test_ball_tracker_flight_fit.py (new, 5 tests)
  - tools/probes/tracker_bag_replay.py (--validate-landing, independent ground-truth fit, mocap crossing comparison)
  - tools/probes/README.md
  - teensy_link/setpoint_pump.py (step gate rewritten as a velocity bound with a non-absorbing rejection)
  - tests/teensy_link/test_setpoint_pump.py (10 new tests)
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py (guard-latch hand park; hb-stale WARN reshaped to one line per dropout episode)
  - tests/ros/test_teensy_bridge_node_read.py (WARN-text test replaced by 6)
  - tests/ros/test_teensy_bridge_node_recover.py (4 new park tests)
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py (CAUGHT_WINDOW_S 0.35 -> 0.70 s, reporting floor)
subsystem:
  - tracking
  - ros
  - motion
  - can
  - config
  - tools
tags:
  - safety
  - performance
  - testing
---

## Symptom

Two sittings (S1 `temp/logs/launch_r2gate_20260917_1845.log` + bag `2026-09-17_18-45-10`, 16
throws; S2 `..._1850.log` + bag `2026-09-17_18-50-45`, 21 throws, self-toss 1/3/5 chains; K=0.7
torque FF confirmed live). Operator's three observations: late catches "settling in" after the
ball had already landed; learner rows reading `caught=False` on throws the operator watched land
in the cup; and repeating `heartbeats stale >500 ms` WARNs alongside two guard latches and one
mid-throw SETPOINT_STALE E-STOP.

## Diagnosis

### 1 — the ball is on time, the hand is late (`findings_20260917.md` §1)

At the 0.5 m class (S2): the ball returns to release height at +0.63..+0.69 s, repeatable to
±0.03 s — the physics is boring. Contact happens at the TOP of the hand's stroke (170 mm above
the plan's touch-down height) because the hand's descent starts at `t_land − 0.03 s`; command
velocity at contact is −0.2..−0.4 m/s against a 3.1 m/s ball, and the cup accelerates downward
at ~15 m/s² (> g) during the run-up that follows, so a ball that has already landed loses contact
— the debounced sensor reads HELD then EMPTY for 0.12–0.25 s on half the throws, and on the other
half it never reads HELD at contact at all, seating only once the hand stops (+0.85..+1.02 s
after the throw). At 0.9 m/0.7 m-command (S1) the sign flips: the plant flew 0.924 s true against
a 0.857 s aim, so the hand had already descended to rest before the ball arrived 0.07–0.12 s late
— hard landing into a stationary cup, one drop (throw 2, no seat).

### 2 — why the learner settles there: the tracker's landing estimate is late (`findings` §2)

The learner is asked to drive the recorded flight `y` to `y_d`. Row-by-row against a raw-marker
gravity-fixed parabola (`flight_truth2.txt`, rms 4–7 mm), the recorded flight overshoots the true
knot-to-830 flight by +0.001..+0.122 s (n=17, median +0.05, 12/17 ≥ +0.03), so the learner
converges the machine to `y_true ≈ y_d − bias`, i.e. early. S1 shows the transient: identity 0.857
→ true 0.924 s (ball late, hand early); the tracker read 1.017 (+0.093), so the learner cut the
command to 0.736/0.722/0.711/0.703 across four throws while the TRUE flight fell to 0.801/0.774
(ball now early, hand late) and the tracker still reported ≥ y_d — the learner believed it had
converged.

**Withdrawn — "the hand-lane splice stepped" (this session's own first read of the SETPOINT_STALE
E-STOP).** Refuted by `opus_guard_hb_report.md` §B(1): `/hand_telemetry pos_cmd` crosses the CATCH
splice at knot 101 as a smooth ~85 rev/s ascent (3.4004@107.8176 → 4.2088@107.8256 →
5.0807@107.8379); "8.7989 is simply where that same lane sits ~61 ms later," no knot stepped. The
real mechanism is §5/Fix 2 below: a ~55–64 ms Jetson pump scheduling hole racing a per-frame
DISPLACEMENT gate whose ~57 ms budget (at the stroke's 88 rev/s peak) is smaller than the measured
jitter, and which freezes its own baseline on the first reject.

**Withdrawn — 2026-09-16's implicit premise that the beat-late estimates were only the host's
per-ball correlation latch.** That day's fix
([2026-09-16-tracker-correlation-follows-the-flight-in-progress](2026-09-16-tracker-correlation-follows-the-flight-in-progress.md))
closed a real bug (flight *n*'s row reading flight *n+1*'s landing) and reads as if lateness were
a bookkeeping artifact. `tracker_lag_probe.py` shows otherwise: samples correctly correlated to
their OWN flight are still 40–75 ms late at the crossing (growing with descent speed, KF velocity
~15% low) — because `matcher.py`'s `process_frame` calls `kf.predict()` with no `dt` every frame,
so the filter always integrates at the nominal 5 ms/200 Hz instead of these bags' actual ~5.2
ms/192 Hz frame period. Correlation and lag are two different bugs; fixing the first left the
second untouched, and the executor's freeze rule (2026-09-16) keeps the LAST admissible estimate —
the most lagged one — making the second bug worse, not better.

### 3 — the `caught=False` verdicts (`findings` §3)

`caught` is recorded, not learned from (no term in `learner.command`). Verdict = a debounced
SEATED sample inside `[landing − 0.10, anchor + CAUGHT_WINDOW_S]`, closed early at the next
release − 10 ms. S2's 4 False rows: seats landing 5–18 ms after the window closed by the
next-release bound (×3), plus one genuine drop (`SPLICE_TOO_LATE` on the 5-throw chain, no seat).
S1's 6 False rows: seats 0.375–0.61 s after the landing on catches that settled late — inside the
physical event, outside the old 0.35 s window. **Operator count: 2 genuine drops across 37 throws**
(S1 throw 2, S2 throw 15); every other False row was a real catch the window missed.

## Discussion

**Why a batch ballistic fit beats retuning the Kalman filter.** The physics of a thrown ball is
two numbers per axis (initial position, initial velocity) and one constant (g) — a batch
least-squares fit over the free-flight samples estimates exactly those and extrapolates the
landing from ground truth, not from state. A recursive filter lags by construction near the
crossing: it is still incorporating position/velocity information as the ball approaches the
plane, and any Q/R retune trades noise rejection for lag rather than removing it. The freeze rule
then keeps the LAST admissible in-flight estimate, which is the most lagged one available — so a
KF retune fights the freeze rule it feeds, while a fit that converges early in the flight and
holds is already the answer the freeze rule wants to keep.

**Why the window is widened, not the verdict redefined.** `CAUGHT_WINDOW_S` 0.35 → 0.70 (S3 in the
Fix list) does not fix late catches — it stops mis-*reporting* ones the operator watched succeed.
Widening is monotone: it can only add a catch verdict, never invent a landing, and on a chained
catch `_bound_by_next_release` still closes the window at the next release regardless of the
constant. The late SEAT is the symptom of §1 (hand descent timed off a late-arriving ball); the
tracker fit (Fix 1) is what should shrink the true settle time and make 0.70 s unnecessary again.
Recorded as a reporting floor, not a fix, so a future session does not read the wider window as
evidence the timing problem is solved.

**Why the step gate is a rate.** A fixed displacement budget (5 rev) is only ever correct at one
cadence; expressed as a rate (`v_max · Δt_since_last_accepted`, floored at one knot period) the
same nominal-cadence budget survives, but a late producer gets proportionally more room instead of
none. The complementary fix — the baseline must not freeze on a reject — is what makes one
scheduling hiccup non-absorbing; a rate bound alone would still latch forever if the time base
never advanced.

**Why the hand park lives in `/recover`, not a hand-rate feasibility gate.** §B(2)'s root cause
(the opening REST planned a homing lane at ~5.4 rev/s against a hand that tracks post-latch
recovery at ~0.83 rev/s — the `RECOVER_SLEW_VEL_RPS = 1.0 rev/s` cap named in
`plans/active/two-ball-skill-stack.md`'s carried item (f), `leg_interp.cpp:777`) is a genuine
missing feasibility check at `install_segment`. It is not implemented here because its threshold
is unmeasured: the achievable hand rate under the bridge's lead authority off an arbitrary seed is
exactly what a bench test would need to pin, and a feasibility gate built on a guessed number would
either refuse the good case or pass the bad one. The park is the fail-safe available today without
that measurement — it removes the specific 8.7 rev seed that made this lane unfollowable — and is
explicitly named as a stopgap for the feasibility check, not a replacement for it.

**The aim-source question, left to the owner.** With `flight_fit`'s landing bias at −2.3 ms (committed tree; the first, descent-only run measured −4.8 ms)
(within the ±10 ms criterion) `catch_aim_source=tracker` would close the timing loop directly off
the observed ball rather than the schedule's commanded landing — potentially fixing §1 without
touching the learner at all. The default stays `schedule`: the fit's sd (45 ms) and dispatch bias
(−53 ms) still fail this brief's own criteria (Verification), so switching the live aim source is
not yet warranted by this session's evidence.

## Fix

1. **`ros_ws/.../tracking/flight_fit.py`** (new) — `FlightFit`: gravity-fixed batch least-squares
   fit over one flight's `(t, pos)` samples; excludes pre-release samples; freezes admission once
   descending and `z ≤ freeze_above_plane_mm` above the landing plane; two-pass 12 mm-residual
   outlier gate; `landing()` reuses `ballistics.predict_landing_state`. `t_ref` anchors at the LAST
   admitted sample (bug found via bag validation: anchoring at the FIRST sample let real drag put
   the fitted z spuriously below `landing_z` while still airborne, snapping `landing_time` to
   ~10 ms after throw — fixed same session).
2. **`matcher.py`** — feeds `FlightFit` the RAW matched marker (not KF state); landing prediction
   prefers the fit once converged, falls back to the KF otherwise; `_flight_fits` cleaned up on
   fresh/terminal ids.
3. **`config/hardware_config.yaml`** `ball_tracking:` — `flight_fit_min_samples=12`,
   `flight_fit_residual_mm=12.0`, `flight_fit_freeze_above_plane_mm=250.0` → regenerated →
   `hw.TRACKING_FLIGHT_FIT_*`; `ball_tracker_node.py` wires them through and logs at startup.
4. **`tools/probes/tracker_bag_replay.py`** — `--validate-landing`: an INDEPENDENT ground-truth fit
   (no import of `flight_fit.py`), `validate_landing()`, `--out-md`.
5. **`teensy_link/setpoint_pump.py`** (hardware-safety) — the per-frame step gate is now a
   VELOCITY bound: reject iff `|Δu| > v_max_axis · Δt_since_last_ACCEPTED_frame` (Δt floored at one
   knot period), rate = axis displacement limit ÷ `knot_dt_s` (200 rev/s hand, 12 rev/s leg — kept
   above `feasibility.py`'s `STEP_BOUND_MARGIN 0.80 × hand_vel_limit_rps` = 160 rev/s so the wire
   gate is never tighter than a validated plan). Time base advances only on an accepted frame, so a
   reject burst grows the budget rather than freezing it (`DEFAULT_GATE_REARM_STALE_S = 0.25 s`
   clears the baseline instead of widening it unbounded past the firmware's own
   `SETPOINT_STALENESS_US`). Legs get the same form for the same reason.
6. **`teensy_bridge_node.py`** (hardware-safety) — `_park_hand_after_guard_latch`, called from
   `_svc_recover`: a single-axis `ACTIVATE(HAND_AXIS)` TRAP_TRAJ to `HAND_ACTIVATE_POSITION_REV`
   before the next schedule can stream the hand. No-op inside `_HAND_PARK_BAND_REV = 0.10 rev`.
   Fail-safe: no hand telemetry / non-finite `pos_rev[6]` ⇒ refuses, `/recover` returns
   `success=False` naming the required manual DEACTIVATE→ACTIVATE — clearing the guard without
   parking the hand would send the operator into the E-STOP the park exists to prevent.
7. **`teensy_bridge_node.py`** (telemetry) — the hb-stale WARN is now one line per dropout episode
   (axis, measured gap, co-dropped encoder count, per-axis tally) on the clearing edge, plus an
   onset WARN past `CAN_AXIS_SILENCE_TIMEOUT_US`; the 500 ms threshold is UNCHANGED (the class is
   real, `plans/active/leg-bus-frame-drops.md` owns it — see Carried).
8. **`executor.py`** — `CAUGHT_WINDOW_S` 0.35 → 0.70 s (Discussion, above).

## Verification

| what | date, command | result |
|---|---|---|
| tracker fit unit + integration | 2026-09-17, `pytest ros_ws/src/jugglebot/jugglebot/tracking/tests tests/ros/test_ball_tracker_gate.py tests/ros/test_toss_integration.py tests/ros/test_reload_integration.py tests/ros/test_ball_tracker_flight_fit.py -q` | **102 passed** (15 new, 87 pre-existing) |
| step gate | 2026-09-17, `pytest tests/teensy_link/test_setpoint_pump.py -q` | **99/99 pass in 0.40 s** |
| step gate vs wire fixtures | 2026-09-17, `pytest tests/teensy_link/test_setpoint_pump.py tests/teensy_link/test_v5_wire_regression.py -q` | **99 pass** (wire bytes unchanged) |
| torque-FF / setpoint adjacency | 2026-09-17, `pytest tests/ros/test_teensy_bridge_node_setpoint.py tests/motion/test_leg_torque_ff.py -q` | **89/89 pass in 7.70 s** |
| read + recover (final state, all 3 bridge fixes) | 2026-09-17, `pytest tests/ros/test_teensy_bridge_node_read.py tests/ros/test_teensy_bridge_node_recover.py -q -p no:randomly` | **83/83 pass in 17.47 s** |
| recover alone | 2026-09-17, `pytest tests/ros/test_teensy_bridge_node_recover.py -q -p no:randomly` | **20/20 pass** |
| read alone | 2026-09-17, `pytest tests/ros/test_teensy_bridge_node_read.py -q -p no:randomly` | **63/63 pass** |
| full `tests/ros/` mid-work snapshot | 2026-09-17, `pytest tests/ros/ -q` | **2908 pass, 6 failed** — all 6 are the tests this work updated (old WARN text + 5 recover tests needing a hand diagnostic), all green in the two rows above |

**Tracker landing-estimate bag validation** (`landing_z=830.0 mm`, criterion: last-in-flight
`\|bias\|≤10 ms AND sd≤15 ms`, dispatch `\|bias\|≤20 ms`; bags `2026-09-17_18-45-10` +
`2026-09-17_18-50-45`, n=17 rows with an independent ground-truth fit):

| run | last-in-flight bias | last-in-flight sd | dispatch bias | dispatch sd | verdict |
|---|---|---|---|---|---|
| 1st (2026-09-17, `brief_tracker_fit_report.md`) | −4.79 ms | 45.26 ms | −52.83 ms | 34.64 ms | bias PASS / sd **FAIL**; dispatch **FAIL** |
| 2nd (2026-09-17 20:20, `temp/probes/tracker_fit_validation_20260917.md`, re-run with the cup-floor gate on BOTH legs — the announced `throw_time` precedes the physical release by 5–55 ms, so the pushed ascent samples were in the first run's fits) | −2.28 ms | 44.60 ms | −49.29 ms | 34.62 ms | bias PASS / sd **FAIL** on the headline; see below |

(An intermediate re-run at 20:05 reproduced the 1st run's numbers exactly: it had started against a
copy of `flight_fit.py` that the implementation agent had reverted to descent-only after filing its
report — the main session re-applied the two-leg gate at 20:08 and re-ran; the 20:20 table is the
tree that was committed.)

Per throw at the last in-flight sample (2nd run, 17 rows): 15 rows lie in **−24 .. +4 ms** (median
−9 ms, sd ≈ 9 ms). The two outside are not fit errors: ball 8 (+162 ms) never reached
`flight_fit_min_samples` — 11 admitted samples on a low arc with marker gaps — so its landing stayed
on the KF fallback path by design; ball 21 (−75 ms) has a 13-sample ground-truth fit, too weak to
judge by. The headline sd therefore fails on those two rows alone; the fit itself clears the
criterion. The **dispatch-instant bias (−49 ms) is not the fit either**: the probe's dispatch
instant is `announced landing − 0.278 − 0.200` = ~0.12 s after the throw, at which point only
~7 samples sit above the 1080 mm floor, so every row is still on the KF fallback — whose state is
the ANNOUNCEMENT's seed, i.e. the learner's commanded flight (0.595 s) against a true ~0.66 s.
A negative −50..−60 ms is exactly that prior showing through. Consequence, carried below: the
tracker cannot yet aim a CATCH at its dispatch instant; the closed timing loop today is the
schedule aim plus the now-unbiased learner.

(2026-09-17, `./run_tests.sh --full`, log `temp/logs/sitting0917_full_20260917.log`, the final tree of this entry): **PASS — parallel 6257 passed, 9 skipped, 2 xfailed in 284.42 s; serial 6 passed in 18.97 s.**
(2026-09-17, `./run_tests.sh`, log `temp/logs/sitting0917_gate2_20260917.log`, after the phase audit's three narrative/dead-code fixes — the yaml comment, the unused descend bookkeeping in `flight_fit.py`, two stale numbers): **PASS — parallel 6219 passed, 9 skipped, 1 xfailed in 222.23 s; serial 3 passed in 7.76 s.**

## Carried

* **`catch_aim_source=tracker` is not yet a closed timing loop.** At the CATCH dispatch instant
  the fit has not converged (too few samples above the cup floor) and the KF fallback carries the
  announcement prior (−50 ms measured). Options: a lower `min_samples` for the ascent-only fit with
  a residual-quality gate, or re-aiming from the converged fit at the first re-send.
* **KF frame-spacing** (`matcher.py`'s `kf.predict()` called with no `dt`, always integrating at
  nominal 5 ms/200 Hz against these bags' actual ~5.2 ms/192 Hz) — a real, independent lag
  contributor, still live for the KF fallback path and the published position/velocity. No fix
  landed.
* **Leg-bus per-axis frame drops** — the 2026-09-17 heartbeat/encoder census (`opus_guard_hb_report.md`
  §A) is the same phenomenon `plans/active/leg-bus-frame-drops.md` (status `proposed`) already
  covers, measured live with FW 23 instrumentation the plan didn't have; promote to `active`.
* **`MAX_LEAD_HAND_REV` (2.0) vs `MAX_DEVIATION_HAND_REV` (2.5)** — 0.5 rev gap between "clamp
  saturated" and "E-STOP"; owner sign-off item, not touched.
* **The 55–64 ms Jetson scheduling hole's origin** — Fix 5 makes it non-fatal, does not explain it.
  Prime suspect is the 10 Hz `teensy_bridge_node` diag/link_status callback; needs a
  callback-duration probe.
* **`SPLICE_TOO_LATE` on the S2 5-throw chain** (throw 15, one of the two genuine drops) — not
  investigated under this entry.
* **`REJECTED_HAND_LANE_TOO_FAST` feasibility check at `trajectory_node.install_segment`** — the
  Discussion's "unmeasured threshold" gate; the park (Fix 6) is a stopgap, not a replacement.
* **Same-class defect in `trajectory_node`'s emitter backstop** (`trajectory_node.py:1273-1311`) —
  identical displacement-vs-rate gate against the LAST EMITTED frame, no elapsed-time term; it
  survived 2026-09-17 only because its bound tracks the LIVE session limit while the pump's was
  frozen at launch. Same fix shape as Fix 5, not applied there.
* **Live-vs-static hand rate mismatch** — the pump's 200 rev/s is captured at construction from the
  static YAML ceiling; `trajectory/set_limits` can raise the LIVE session limit toward
  `JB_TRAJ_HAND_VEL_CEILING_RPS = 300`, above which a validated plan could be refused on the wire.
  Pre-existing, now explicit; owner call on whether the pump should learn the live limit.
