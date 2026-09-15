---
title: "The tracker was blind to a labelled ball — all markers in, gated to the expected ball"
type: bugfix
date: 2026-09-15
status: done
phase: "two-ball-skill-stack — R3"
related_plan: two-ball-skill-stack.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/ball_tracker_node.py (forward EVERY marker with its label; read the three new params; new parse via tracking.matcher)
  - ros_ws/src/jugglebot/jugglebot/tracking/matcher.py (parse_label_prefixes; eligible_markers; process_frame takes labels; announced gate = 200 mm sphere around the ANALYTIC expected position; height floor retired; _expected_announced_position; parabolic path behind detect_human_throws)
  - config/hardware_config.yaml (+announced_gate_mm, excluded_label_prefixes, detect_human_throws; -min_height_above_landing_mm) + regenerated artifacts
  - tools/probes/tracker_bag_replay.py (NEW — production-faithful bag replay through the real BallTracker)
  - tests/ros/test_ball_tracker_gate.py (NEW — 21 tests: labelled ball, identity exclusion, sphere-not-box, analytic centre, parabolic off, config wiring)
  - tests/ros/test_toss_integration.py, tests/ros/test_reload_integration.py (retire the height-floor constant, wire the new params)
  - ros_ws/src/jugglebot/jugglebot/tracking/tests/test_matcher.py (parabolic tests opt in explicitly)
  - tests/hardware/session_skills_r3_apex_ladder.md (stream check row 23 — the tracker must see the ball)
  - plans/active/two-ball-skill-stack.md, tools/probes/README.md
---

## What / Why

At the 2026-09-15 apex-ladder sitting the robot threw 13 self-tosses, the
operator watched every one of them fly and be caught, and the tracker
CONFIRMED **none** of them. Every catch ended `END NO_LANDING`.

`ball_tracker_node._on_mocap` forwarded to the matcher only those
`/mocap_data` markers whose `label` field was **empty**:

```python
for data in msg.markers:
    if not data.label:          # Unlabelled markers only
        markers.append(...)
```

The ball was not unlabelled. QTM's AIM model had claimed the flying ball as a
marker of the **`Ball_Butler` rigid body**, and published it every frame as
`Ball Butler - 1`. So the one true candidate was discarded at the node's front
door, on every frame, for the whole session — and `_match_announced_balls`,
having nothing to match, left all 13 balls at `ANNOUNCED` until the CATCH
executor's deadline expired.

The tracker now forwards **every** marker with its label, and drops only those
belonging to the robot's own rigid bodies.

## Evidence

Bag `~/Desktop/rosbags/2026-09-15_18-51-37`, log
`temp/logs/launch_r2gate_20260915_1851.log`.

Markers whose z rises by more than 300 mm within 1.2 s of release — the whole
frame, labelled and unlabelled (probe `step1.py`, scratch, 2026-09-15):

| throw | release (abs) | label | z before | peak z | rise | t@peak |
|---|---|---|---|---|---|---|
| armA-050 | 1789462880.965 | **`Ball Butler - 1`** | 711–742 | **1491** | 775 mm | +0.409 s |
| armA-090 | 1789463078.400 | **`Ball Butler - 1`** | 715–780 | **2002** | 1286 mm | +0.499 s |

Exactly one label flies, on both throws, and it is the same one. The other
nine labels present in those windows move by 0–7 mm: `Platform - 1/3/4/5` and
`Base - 1/2/3/4/6`. No unlabelled marker rises at all — the only unlabelled
returns in the window are three static clutter points, one of them the floor
marker at (550, −353, 95) mm.

`/rigid_body_poses` carries four bodies: **`Ball_Butler`, `Base`,
`Catching_Cone`, `Platform`**. Whole-session label census (every 40th of
238 553 frames): `Base - 1..6`, `Platform - 1..5`, and `Ball Butler - 1`
(4 515 samples, present ~76 % of frames) plus `Ball Butler - 4/5` (392 each).
The ball marker is continuously present — resting in the cup at z ≈ 717–719,
lifting through the throw, and back to 717 after the catch.

Distances from the announcement's own `initial_position` (−50, 0, 860) at the
release instant — the decisive geometry:

| label | distance | inside a 200 mm sphere? | inside a ±200 mm box? |
|---|---|---|---|
| `Ball Butler - 1` (armA-090) | **8.1 mm** | yes | yes |
| `Ball Butler - 1` (armA-050) | **103.2 mm** | yes | yes |
| `Platform - 3` | 204.6 mm | no | **yes** |
| `Platform - 5` | 206.3 mm | no | **yes** |
| `Platform - 4` | 219.5 mm | no | no |
| `Platform - 1` | 220.7 mm | no | **yes** |
| `Base - *` | 863–960 mm | no | no |
| `Ball Butler - 4` | **1257.1–1257.8 mm** | no | no |
| `Ball Butler - 5` | **1287.7–1288.0 mm** | no | no |

The last two rows are the **physical Ball Butler rig's own markers**, and they
matter because they share the un-excluded `Ball Butler` prefix with the ball's
label — nothing keeps them out by identity, so their geometry is the only thing
that does. Measured session-wide (every 20th of 238 553 frames, 790 samples
each) rather than at the release instants, because **both are ABSENT from every
frame within ±1.5 s of both measured releases**; they appear in ~6.6 % of the
session, elsewhere. When present they are static to under 1 mm at 1257–1288 mm
from the cup — the rig parked ~1.26–1.29 m away, **outside the 200 mm gate by a
factor of ~6, with 0 samples inside it**. So the un-excluded BB prefix is safe
here as a matter of measured fact, not of assumption; if the rig were ever
re-parked inside the gate, the only thing separating it from the ball would be
nearest-wins, which is now pinned by
`test_nearest_wins_picks_the_ball_over_a_rig_marker_in_the_gate`.

## Discussion

### Why "all markers in", and why the exclusion cannot be by label shape

The owner's framing was right and the important half of it is the second half:
*consider all mocap markers as possible balls, but only pay attention to an
area around the expected ball*. The first half is what fixes the sitting.

The tempting cheap fix — keep filtering by label, but also admit "ball-looking"
labels — is the trap. The ball's label was `Ball Butler - 1`: a **Ball Butler
rig label**. Had the exclusion set been written as "drop the robot's markers
and the BB rig's markers", as the obvious reading of "exclude the machines"
suggests, it would have reproduced the very blindness it was meant to fix. The
label a marker carries is a property of QTM's AIM model file, not of the
object: re-run the calibration, add a marker to a rig, move a ball into a
volume where a different body's model claims it, and the label changes while
the physics does not. So eligibility is decided by **rigid-body membership of
the robot** (`Platform`, `Base`) and by nothing else. Everything else —
unlabelled markers, BB-labelled markers, cone-labelled markers — stays
eligible. `Catching_Cone` is deliberately *not* excluded: it had no labelled
markers in this session, it is nowhere near the cup, and adding it would buy
nothing while adding one more way for a future ball to be silently dropped.

That reframes the defect one level up, which is the reason to write this down.
It is not "the node had the wrong filter". It is: **the tracker's candidate set
was defined by a property of the observer rather than by the physics**, and the
whole class of failures that follows — a re-labelled ball, a re-calibrated
volume, a new rig, a second ball claimed by a different model — is closed only
by making membership the criterion and geometry the gate.

### Why gate-to-expected replaces the adaptive threshold and the height floor

The old announced gate had three parts, and two of them were wrong:

1. **Unlabelled-only** (in the node) — the headline defect above.
2. **Height floor**: `if mpos[2] < landing_z + min_height_above_landing_mm:
   continue`, i.e. 830 + 50 = **880 mm**. The ball marker sits in the cup at
   **717–742 mm** through the announced throw instant and only clears 880 mm at
   about +0.056 s. The floor therefore discarded the one true candidate at
   exactly the frame the gate first ran. It is retired outright, not relaxed:
   a height threshold cannot distinguish a ball from a platform marker
   (`Platform - 1` sits at 763 mm, `Platform - 3` at 1053 mm — one under the
   floor and one over it), so it was never doing the job its name implies.
   Identity exclusion does that job properly.
3. **Adaptive KF-distance threshold**: `base + speed*0.05 +
   time_since_throw*50`, capped at 400 mm, measured from `kf.position`. The
   *centre* is the real problem here, not the radius. `KalmanFilter.predict()`
   advances by a fixed `self.dt` (5 ms) once per `process_frame` call, so the
   filter's clock is only correct if frames arrive at exactly 200 Hz. This bag
   delivers ~186 Hz. A KF-centred gate therefore walks backwards along the
   trajectory at ~7 % of elapsed flight — hundreds of mm of z error half a
   second into a 1.5 m throw — and the error grows exactly when the ball is
   moving fastest. The new gate is centred on the **analytic** ballistic
   position from the announcement's `(initial_position, initial_velocity,
   throw_time)`, evaluated at the frame's own timestamp. Closed form, no
   accumulation, no dependence on frame rate. (The matcher already had this
   construction inline in `_is_near_existing_ball`, with a comment saying it is
   "far more accurate than the KF"; it is now one helper,
   `_expected_announced_position`, used by both.)

A **sphere of 200 mm, not a ±200 mm box.** This is the one place where the
choice is forced by measurement rather than taste: the platform's own markers
sit 204.6 / 206.3 / 219.5 / 220.7 mm from the cup, and **three of the four are
inside a ±200 mm axis-aligned box** while none is inside a 200 mm sphere. The
box would have put a platform marker in the candidate set at precisely the
frame where the expected ball position *is* the cup. The sphere is also the
honest shape: the quantity being gated is a distance, and the isotropy of the
prediction error does not know about coordinate axes. Identity exclusion is
still the primary defence — the margin between the ball (8–103 mm) and the
nearest platform marker (204.6 mm) is only ~100 mm, too thin to be a safety
argument on its own — but the two together mean a mislabelled platform marker
still has to be within 200 mm of the ball's predicted position to do damage.

### What the human-throw disable costs

`_detect_parabolic` is now behind `detect_human_throws`, default **False**. The
cost is real but currently zero: nothing on the skill stack consumes a
`human_throw` ball, and a human cannot be caught from at present. The benefit
is measurable in the baseline replay — under the old code the 13 announced
balls were assigned tracker ids 80, 82, 83, 97, 99, 107, 109, 111, 135, 137,
141, 146, 150, and under the new code they are ids 1–13. Those gaps are the
158 phantom tracks the parabolic detector spawned off the static floor marker
at (550, −353, 95) mm over the session: every one of them a Ball object, a
Kalman filter, and a row in `/balls` that `ball_possession` had to consider and
reject. The path is preserved, tested (`tests/tracking` opts in explicitly) and
one parameter away.

### Failure modes considered

- **A platform marker at the cup.** The expected position of an announced ball
  at its throw instant *is* the cup, and the platform's markers are the nearest
  things to it. Ruled out by identity, not distance — and pinned by
  `test_platform_marker_at_the_cup_is_not_matched`, which feeds a
  `Platform - 1`-labelled marker 8.7 mm from the cup for 10 frames and asserts
  the ball stays `ANNOUNCED`.
- **A platform marker crossing the flight path or the catch.**
  `_associate_confirmed_balls` runs *before* the announced gate and uses a
  50 mm gate around the KF position of an already-CONFIRMED ball, with `used`
  markers removed from the announced stage. A platform marker would have to be
  within 50 mm of the tracked ball to steal the association, and it is excluded
  by identity before it gets there.
- **Two balls, one announcement.** The nearest-marker-wins rule plus the shared
  `used` set means one marker feeds at most one ball per frame, and the
  announced stage only ever sees markers the confirmed stage did not take.
- **Two announcements, one ball** (the 2026-09-13 chained-throw bag). The first
  ball confirms and holds the marker; the second and third announcements find no
  free candidate and expire to `UNKNOWN`. That is the correct answer — there was
  one physical ball — and the replay shows it unchanged from the old code.
- **The ball resting in the cup confirms the instant the announcement fires,
  before any real motion.** True, and intended: the ball *is* there, the gate
  is a position gate, and the alternative (a height floor, or a minimum speed)
  is what caused this entry. If a throw is announced and never physically
  happens, the ball confirms in the cup and the KF's velocity estimate is then
  pulled toward zero by the stationary measurements, so the landing prediction
  degrades rather than staying at the announcement's fiction — which is the
  safer direction, but it is not a *guarantee*, and it is the one residual the
  owner should be aware of (see Outstanding).
- **The Ball Butler rig's own markers sharing the eligible prefix.** Measured:
  1257.1–1288.0 mm from the cup, static, 0 samples inside the gate, absent
  around every release. Geometry handles them; two tests pin both the measured
  case and the hypothetical re-parked-inside-the-gate case.
- **`excluded_label_prefixes` as a prefix match.** `Platform` also prefixes a
  hypothetical `Platform_Ball`. Accepted: the QTM model's naming is
  `<Body> - <n>`, the set is three words long and lives in config, and a
  substring or regex match would be harder to reason about for no gain here.

## Fix

- `ball_tracker_node._on_mocap` builds a parallel `labels` list and passes
  every marker through: `self._tracker.process_frame(markers, t, labels)`.
- `BallTracker.eligible_markers(positions, labels)` is the single enforcement
  point for eligibility; `process_frame` calls it once and all three stages
  (confirmed association, announced gate, parabolic) see only its output. With
  `labels=None` every marker is eligible, which keeps the synthetic-marker
  tests and `tools/tracking_analyzer.py` working unchanged.
- `_match_announced_balls` gates on `‖marker − expected‖ ≤ announced_gate_mm`
  with `expected = _expected_announced_position(...)`. The height floor and the
  adaptive threshold are gone.
- `parse_label_prefixes` lives in `tracking/matcher.py` (pure Python), not in
  the node: the node stays a thin wrapper, and tests and the replay probe reach
  it without importing rclpy.
- Config (`ball_tracking`): `+announced_gate_mm: 200.0`,
  `+excluded_label_prefixes: "Platform,Base"`, `+detect_human_throws: false`,
  `−min_height_above_landing_mm`. The prefix list is ONE comma-separated string
  because `generate_config.py` emits this section into the firmware header too
  and its C++ emitter has no representation for a list of strings.

## Verification

**The repro, and the fix, on the sitting's own bag.**
`tools/probes/tracker_bag_replay.py` replays a bag's `/mocap_data` (labels
included) and `/throw_announcements` through the same `BallTracker` the node
builds from the same generated config, frame by frame, using each frame's bag
timestamp as `current_time`.

Pre-change baseline (HEAD's `matcher.py` + HEAD's unlabelled-only node filter,
run 2026-09-15):

- `2026-09-15_18-51-37`: **announced 13, CONFIRMED 0** — every ball `NEVER`
  confirmed, final status `UNKNOWN`. This reproduces the sitting exactly.
- `2026-09-13_22-57-18`: **announced 3, CONFIRMED 1** — ball 81 at
  `t_conf = 0.027 s`, final `CAUGHT`; balls 83 and 84 never.

After the change (`python tools/probes/tracker_bag_replay.py <bag>`, run
2026-09-15):

- `2026-09-15_18-51-37` — **13/13 PASS**, 238 553 mocap frames, 2 542 013
  markers, 886 586 eligible after label exclusion:

| ball | t_conf (s) | t_land_est (s) | land_err at deadline (s) | land xy err at deadline (mm) | frames | final |
|---|---|---|---|---|---|---|
| 1 | 0.005 | 0.005 | +0.050 | 5 | 49 | CAUGHT |
| 2 | 0.002 | 0.002 | +0.051 | 10 | 119 | CAUGHT |
| 3 | 0.002 | 0.002 | +0.053 | 10 | 128 | CAUGHT |
| 4 | 0.009 | 0.009 | +0.042 | 11 | 111 | CAUGHT |
| 5 | 0.001 | 0.001 | +0.063 | 12 | 121 | CAUGHT |
| 6 | 0.001 | 0.001 | +0.065 | 11 | 126 | CAUGHT |
| 7 | 0.006 | 0.006 | +0.128 | 15 | 134 | CAUGHT |
| 8 | 0.006 | 0.006 | +0.073 | 8 | 34 | CAUGHT |
| 9 | 0.006 | 0.006 | +0.061 | 7 | 22 | CAUGHT |
| 10 | 0.009 | 0.009 | +0.087 | 9 | 34 | CAUGHT |
| 11 | 0.004 | 0.004 | +0.111 | 16 | 128 | CAUGHT |
| 12 | 0.001 | 0.001 | +0.057 | 14 | 137 | CAUGHT |
| 13 | 0.002 | 0.002 | +0.071 | 13 | 115 | CAUGHT |

  Every ball CONFIRMS within **9 ms** of release with a landing estimate, against
  the 0.25 s criterion (the CATCH deadline is ~0.47–0.49 s before landing at
  this operating point). The `land_err at deadline` column is the number the
  executor would actually consume: **+0.042 to +0.128 s later** than the
  announcement said, i.e. the tracker corrects the plant's known ~25 %-fast
  throw in the right direction (armA-050's marker crosses z = 830 descending at
  +0.787 s against an announced +0.685 s). The *first* landing estimate's
  −0.006 to −0.017 s agreement with the announcement is near-zero by
  construction and is only a sanity check.

- `2026-09-13_22-57-18` — **1/3 PASS**, unchanged from the baseline: ball 1 at
  `t_conf = 0.001 s` (26 ms earlier than the old code's 0.027 s), final
  `CAUGHT`, deadline landing +0.134 s / 246 mm; balls 2 and 3 never confirm,
  because only one physical ball existed for three chained announcements. **No
  regression.**

**Tests** (all 2026-09-15):

- `pytest tests/ros/test_ball_tracker_gate.py tests/sim/test_logbook_front_matter.py -q`
  → **33 passed in 0.58 s** (23 gate tests after the two Ball-Butler-rig tests
  were added 2026-09-16, plus the 10 front-matter tests). The gate file alone
  was **21 passed in 0.26 s** before those two.
- `pytest tests/ros/ -q` → **2881 passed, 4 skipped in 197.17 s**
- `pytest ros_ws/src/jugglebot/jugglebot/tracking/tests/ -q` → **54 passed in
  0.76 s**
- `pytest tests/sim tests/motion tests/firmware -q --deselect tests/motion/test_unified_cycle_budget.py --deselect tests/motion/test_validate_cycle_budget.py`
  → **2889 passed, 5 skipped, 3 deselected, 2 xfailed in 976.85 s** (the two
  deselected files are the `serial`-marked wall-clock budget tests, which
  measure the machine rather than this change)
- `pytest tests/motion -q --deselect …` on its own → **1819 passed, 3
  deselected in 312.92 s**
- `pytest tests/firmware -q` → **258 passed, 1 skipped in 209.49 s** (the
  `ball_tracking` section is emitted into the firmware header, and
  `excluded_label_prefixes` is carried as ONE comma-separated string precisely
  so the C++ emitter stays valid: `constexpr const char*
  EXCLUDED_LABEL_PREFIXES = "Platform,Base";`)
- `pytest tests/sim/test_plans_index.py tests/sim/test_logbook_front_matter.py -q`
  → **83 passed in 0.45 s**

`./run_tests.sh` was NOT run by this unit (out of scope per the brief). Since
this change touches `config/` and the ROS tracker, **`./run_tests.sh --full` is
owed before the commit and before the sitting.**

(2026-09-16 00:xx local, `./run_tests.sh --full`, log `temp/logs/tracker_full2_20260916.log`, the final staged tree): **PASS — parallel 6169 passed, 9 skipped, 2 xfailed in 284.90 s; serial 6 passed in 18.57 s.**

## Outstanding — for the owner

1. **`catch_aim_source` still defaults to `schedule`.** The parallel workaround
   landed today
   ([2026-09-15-open-loop-catch-from-throw-state](2026-09-15-open-loop-catch-from-throw-state.md))
   made the catch open-loop precisely *because* the tracker was blind. The
   tracker now works, so the choice is live again: leave the catch open-loop,
   move it back to `tracker`, or keep `schedule` for the ladder and re-evaluate
   at R4. Nothing in this change touches that default.
2. **A throw that is announced but never released** will confirm the ball
   sitting in the cup. Harmless today (the KF then drags the landing estimate
   away from the announcement's fiction rather than toward it) and out of scope
   here, but if it ever needs to be refused, the honest gate is "the marker has
   left the cup", not a height floor.
3. **QTM's labelling is now a diagnostic, not a dependency** — but it is worth
   knowing that the ball was labelled at all. If the intent is for the ball to
   be unlabelled, the AIM model needs the fix; the tracker no longer cares
   either way.
