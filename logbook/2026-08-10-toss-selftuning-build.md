---
title: Toss self-tuning loop — the build
type: feature
date: 2026-08-10
status: in-progress
phase: "catch-robustness Phase 2 / toss-selftuning build"
related_plan: toss-selftuning.md
subsystem:
  - ros
  - motion
tags:
  - testing
  - instrumentation
---

# Toss self-tuning loop — the build

**One entry for the whole build.** `plans/active/toss-selftuning.md` § 5 lands in
six phases (2a–2f); each appends its own section here rather than minting a
separate entry, because the phases share one design, one corpus format and one
acceptance story, and a reader reconstructing "how did the toss calibration loop
get built?" wants them adjacent. Every section carries its own (date, command,
result) verification triple.

Everything in 2a–2f is **desk-side** — no hardware is needed or touched
(operator decision 1). The operator runs the capture, after catch-robustness
Phase 0.

---

## Phase 2a — the instrument (2026-08-10)

**Zero new control authority.** No map, no trim, nothing applied. If every line
of this phase were deleted the machine would command exactly the same motion; the
only thing that changes is that a sitting now produces a corpus.

### What landed

| Deliverable | Where |
|---|---|
| The pure record module — `FIELDS`, `encode`/`decode`/`validate`, `label_from_sensor`, `latch_announced_ball`, `join` | `ros_ws/src/jugglebot/jugglebot/toss_record.py` |
| `/toss/record` declaration at the FSM terminal + best-effort JSONL belt | `reload_coordinator_node._publish_toss_record` |
| The offline miner, with `--self-check` and `--emit-fixture` | `tools/probes/toss_record_miner.py` |
| **THE ONE bag-record list** (D18) | `jugglebot_launch.py` + `tests/hardware/session_anomaly_fixes.md` |
| Layer-1.5 dwell-tilt covariate **schema** (nullable until 2d wires the reads) | `toss_record.FIELDS`, block `dwell_tilt` |
| Tests | `tests/motion/test_toss_record.py`, `tests/ros/test_toss_record_miner.py`, `tests/ros/toss_record_fixtures.py` |

### Discussion

Several things in this phase did not go the way the plan wrote them, and three of
them are measurements that contradicted a design assumption. They are here rather
than in the Fix section because the *reasoning* is the part a future session
needs.

#### 1. The plan's catch-search window was wrong, and the bag says so

§ 3.3 specifies `t_catch_raw_ros` as the first `empty→held` edge in
`[landing − 0.30, landing + 0.70]`, with **0.70 = `toss_sequencer::CATCH_CONFIRM_WINDOW_S`**.

That constant is the **FSM's terminal deadline** — how long a toss waits before
declaring MISSED — not a sensor search window. Phase 1 measured and shipped the
sensor window separately as `JB_BD_ARRIVAL_LEAD_S` / `JB_BD_ARRIVAL_WINDOW_S`
= `[−0.20, +1.50] s`, sized on three bags of real arrivals.

Scored against the reference bag (25 catches), the draft window relabels
**exactly one** row MISSED: the **+798 ms** arrival. That single row is not a
rounding case — it is the population **maximum**, and it is the row
`JB_BD_ARRIVAL_WINDOW_S` was sized on in the first place. The runner-up
(+675 ms) survives with 25 ms to spare, which is the honest statement of how
thin the margin was: one session's variation.

So `label_from_sensor` takes the **shipped** windows, injected. That also buys
the D11 property for free — the offline labeller and the live
`HandBallSensorSource` now read the same constants by construction, and
`test_the_miner_mirrors_the_nodes_windows` pins it.

*(An earlier draft of this entry and of the module docstring claimed the draft
window cost **two** rows. It costs one. The claim was checked against the bag and
corrected rather than left as a plausible-sounding number — which is the entire
reason the empirical-probe rule exists.)*

#### 2. The debounce lag is 2.4× what the plan estimated, and it is asymmetric

D12 argues raw-for-times / debounced-for-verdict from *"the 5-sample debounce at
50 Hz is a 100 ms window"*. Measured on the reference bag (70,666
`/hand_telemetry` samples, `ball_held_valid` 100 %):

| edge | population | min | median | max |
|---|---|---|---|---|
| `empty→held` (the CATCH) | all 38 | 0 ms | 0 ms | 0 ms |
| `held→empty` (the DEPARTURE) | all 39 | 232 ms | **241 ms** | 295 ms |
| `held→empty`, **self-toss release edges only** | 31 | 232.5 ms | **240.1 ms** | 250.5 ms |

(The third row is the one committed to `tests/ros/toss_record_fixtures.py` as
`DEBOUNCE_FALL_LAG_MS`, because it is the population a *release-timing* fit would
draw from. The 295 ms outlier in the all-edges row is a non-toss departure — the
operator lifting the ball out — and it is worth keeping visible precisely because
it shows the lag is not a fixed constant.)

Both halves matter. The asymmetry is *documented firmware behaviour* —
`plans/active/hand-ball-sensor.md` § "Debounce asymmetry": five consecutive EMPTY
good replies to drop, any single HELD restores — so this is a confirmation, not a
discovery. What is new is the magnitude on the departure side and what it implies:
**241 ms is larger than the +118–133 ms uptime dispatch shift the whole fresh-boot
discipline exists to control.** A timing fit taken off the debounced departure
edge would carry a systematic lag bigger than the effect it was measuring, and it
would look like real physics.

The reason the plan's estimate was low compounds with finding 3.

#### 3. The sensor poll runs at ~71 ms, not the configured 20 ms

`JB_BD_CHECK_INTERVAL_MS = 20` (50 Hz). Measured cadence of `ball_held_stamp`
advances on the same bag: **p5 32 / median 71 / p95 111 ms** — a 3.5× gap,
spread wide rather than quantised, which is the signature of a poll being
deferred rather than one running at a different fixed rate. Five samples at
71 ms is ~355 ms, which is the right order for the 241 ms fall lag above.

This is **not diagnosed here** — it is a can-bridge question and it belongs to its
own investigation. What this phase does is make it *impossible to assume away*:
`sensor_poll_dt_ms_median` is a MEASURED field on every record, and
`test_the_fixture_records_the_measured_poll_cadence_gap` fails if the corpus ever
stops carrying the discrepancy. The field earned its place on the first bag it
was pointed at, which is the strongest argument available for keeping
measured-not-assumed fields in a schema.

It also has a live consequence for the plan: § 3.7 item 5 gates the timing fit on
`sensor_poll_dt_ms_median` being *"within 10 % of 20 ms"*. On this plant that
gate refuses **100 %** of records. Carried in Open below; 2c owns the fix.

#### 4. The reference bag cannot support an aim fit at all, and that is a finding

The miner produces a complete corpus from `2026-08-10_16-30-44` — 31 self-tosses,
19 CAUGHT / 12 MISSED, every row `mined-only` — but **`usable_for_aim_fit` is
0/31**, because the mocap block is empty.

Investigated rather than accepted, and then **cross-checked against the shipped
instrument rather than trusted**: `tools/probes/ball_arrival_offset.py --thrower
jugglebot --plane 809.08` reports **`NO TRACK` on all 31**, independently. So the
empty block is the capture, not the miner.

What the bag actually contains, in ±1.5 s of a landing: exactly **four**
unlabelled markers — three static rig markers outside the ±300 mm lateral gate
and one slow-drifting marker at z ≈ 695 mm, below the 809.08 mm cup plane and
settling like a platform-mounted marker. The precise statement is therefore
narrower than "no ball in mocap": **the ball is absent from the descending fit
band `[cup_plane, cup_plane + 300 mm]` within the ±300 mm lateral gate.** It is
*not* absent everywhere — the shipped probe's floor census counts 1–4 new floor
arrivals after five of the misses, so the ball is tracked once it is on the
ground. This is a coverage/occlusion gap over the cup, not a marker-labelling
problem.

Two consequences worth writing down. First, the miner is behaving correctly — it
reports absence as absence, and `excluded_reason: no_mocap_fit` is the honest
row. Second, and more useful: **the recording gap D18 fixes is not the only gap.**
A sitting can carry every topic on the list and still be unable to answer the aim
question, because the ball has to be visible to QTM *in the descending band over
the cup* for `ball_arrival_offset` to work at all. That is a P2-preflight check
the plan does not currently have, and it is cheap: one throw, one miner run, look
for a non-null `land_xy_global_mm`. Carried in Open.

(`floor_arrival` is in the schema and ships null in 2a: the floor census lives in
`ball_arrival_offset.read_bag`, which the miner does not reuse — it imports the
*estimator*, not the reader. Wiring it is a small follow-on, and this sitting is
the argument for doing it.)

#### 5. Three deliberate deviations, and why

**`uptime_ms_at_release` is mined, not declared.** The schema marks it `D+M`. The
coordinator does not subscribe to `/link_status`, and adding a subscription to
the node that owns the hand, the latch and the abort ladder — to obtain a pure
covariate — spends exactly the surface D10 argues for keeping out of that node.
`/link_status` is in the bag at 5 Hz, so the miner recovers the number to
~200 ms, three orders finer than the hours-scale drift it exists to partition on.
The `D` side is null and `record_provenance` already says so.

**`cycle_index` and `action` are left NULL on the mined side.** The schema's `M`
source for `cycle_index` is the action-feedback stream, which is exactly what
D18 adds to the record list and which no existing bag has. The tempting
substitute — the announcement's position in the bag — is a *different quantity*:
"the 7th announcement in this sitting" is not "cycle 7 of this session". Filling
one with the other would manufacture a spurious `D+M` agreement on every
single-toss bag and a spurious `D+M` **conflict** on every session bag, which
would flood the one field (`disagreement`) whose value is that it is normally
empty.

**The miner test lives in `tests/ros/`, not `tests/sim/`.** The plan's 2a row
says `tests/sim/test_toss_record_miner.py`. It sits next to its fixture and next
to `tests/ros/test_hand_sensor_replay.py`, which is the same shape (probe +
committed fixture + graceful bag skip) and the precedent the phase spec named.
Splitting a test from the fixture it imports across two directories buys nothing.

#### 6. The plant block is null, and that was the safer choice

§ 3.3's Plant block (`stroke_peak_rev`, `dip_below_x3_rev`, `dispatch_shift_ms`,
`iq_brake_min_a`) ships **null**, and `plant_block_source` with it — a source
string on an empty block asserts a provenance that does not exist.

§ 3.4 is explicit that the row builder must be **imported** from
`hand_stroke_timeline`, not re-implemented, and that is right. But wiring it
means mapping its *session-relative* clock back onto the record's ROS instants,
and a mistake there writes **plausible wrong plant numbers** into a corpus that
guard G4 would then police tosses with. Nothing in 2a can validate the result:
there is no independent plant reference for this bag. Writing a confident wrong
number is the exact failure class this phase exists to prevent, so the block
stays null and `_mark_usable` says so — G4 is listed among the guards that belong
to the FIT rather than being silently marked as enforced.

The field mapping is worked out and recorded in the plan's § 10 so the next
phase pays nothing for the deferral. One genuine gap surfaced while doing it:
**`iq_brake_min_a` has no shipped builder at all** — `hand_stroke_timeline`'s
`HandSample` does not carry `iq_meas` — so it needs a new computation *and* a
pinned braking window, and a window pinned without measurement is exactly what
the empirical-probe rule forbids.

#### 7. What was NOT built, deliberately

The Layer-1.5 dwell inclinometer **reads** are not wired — only the schema
fields, all nullable. The phase spec assigns the wiring to 2d. This is not a
placeholder in the pejorative sense: § 3.10's degrade-never-delay rule already
makes `dwell_tilt_n = 0` a legal record on real hardware, so every analyser has
to tolerate an absent block from day one anyway. Landing the schema now means the
2d wiring is a producer change with no consumer churn.

### Fix

**`jugglebot/toss_record.py`** — pure (no ROS, no file I/O, no config imports,
the `ball_possession.py` posture). Windows are *injected*, never imported, so the
node and the miner cannot drift onto different constants. `FIELDS` is a tuple of
typed `Field` records carrying block, origin (`D`/`M`/`D+M`/`X`) and kind;
`encode` maps non-finite floats to `null` and refuses to emit a bare `NaN` token
(`allow_nan=False`); `validate` returns problems rather than raising, because a
miner that dies on one malformed row loses a whole capture.

`latch_announced_ball` is **extracted verbatim** from
`reload_coordinator_node._update_announced_ball_latch`; the node now calls it and
keeps only the locking. That is D11's "one rule, two callers" made structural
rather than aspirational.

**`reload_coordinator_node`** — publishes one `std_msgs/String` JSON declaration
per cycle terminal from `_log_toss_outcome`, the single authoritative outcome
line, so the census includes the `REJECTED_BAD_GOAL` path. The record context is
opened *before* the goal-numerics gate, so a rejected goal declares its own
identity instead of inheriting the previous goal's. The whole publish path is
guarded and the belt writes outside `self._lock`, warning once per goal and then
going quiet: a full disk is a lost measurement, never a stalled abort ladder.
The node also gained a 30 s median-filtered `perf − ros` offset **for the record
only** — the FSM keeps its single instantaneous read, and carrying both numbers
is what turns `_announcement_landing_perf`'s documented open reconciliation
question into a measurement.

**`tools/probes/toss_record_miner.py`** — reads a bag in **two passes**, and that
is not premature optimisation: a sitting carries ~137k `/mocap_data` and ~93k
`/balls` messages, and decoding them in full costs minutes and gigabytes while
the miner only ever looks at ±1.2 s around each landing. Pass 1 is cheap and
yields the announcement instants; pass 2 decodes the heavy streams only inside
those windows. The first (single-pass) implementation was killed after 7 minutes
without finishing.

**`tools/probes/ball_arrival_offset.py`** — gained `fit_plane_crossing_full`,
which returns the fit residual RMS (guard G2) alongside the existing five-tuple;
`fit_plane_crossing` now delegates to it. One least-squares, two views — a second
fit in the miner would be a second place for the band selection to drift.

**`motion/tilt_map.find_repo_root`** — the private `_find_repo_root` promoted to
public (5 occurrences, all updated) so the record belt resolves `temp/logs/` by
the same **marker walk** rather than a fixed number of `dirname` hops. The module
runs from the colcon install tree in production and from `ros_ws/src` under
pytest, and those are different depths; a fixed walk is the tilt-cal Phase-2
finding, which produced a path that could not exist in production while looking
perfectly reasonable in the source tree.

**THE ONE bag-record list (D18)** — `jugglebot_launch.py` records the union;
`session_anomaly_fixes.md` § Recording now says `record:=true` and its eight
per-section "append these topics" instructions are retired. The old runbook
command also named `/platform_target`, which has had no publisher since the
SocketCAN decommission; it is not in the union.

### Verification

**The gate**, run 2026-08-10 on the Jetson, immediately before the commits and
with no code edit in between:

- `./run_tests.sh --full` → **RESULT: PASS**, parallel **4834 passed, 3 xfailed
  in 470.98 s**, serial **9 passed, 4837 deselected in 40.52 s**, total 517 s.

**The acceptance**, run 2026-08-10:

- `python tools/probes/toss_record_miner.py --bag 2026-08-10_16-30-44` →
  `ledger: 70666 samples, ball_held_valid 70666/70666 (100.0 %), 38 catches
  (empty->held), 39 departures (held->empty), 39 held segments`, three
  quick-drops at 0.569 / 0.999 / 0.988 s, `labels: CAUGHT=19, MISSED=12` over
  31 self-tosses, every row `mined-only`. **That is the hand-mined ground truth
  exactly** — 39 departures / 38 catches / 3 seat-then-leaves — from a bag that
  predates every line of this phase.
- Independent reconciliation: those 31 labels match Phase 1's
  `hand_sensor_verdict_replay.py` on the same bag's `jugglebot` subset
  (19 CAUGHT / 12 MISSED), through a completely different code path — the live
  `HandBallSensorSource` rather than `label_from_sensor`.
- `python tools/probes/toss_record_miner.py --self-check` → **22/22 cases pass**,
  exit 0.
- Cross-check of the empty mocap block:
  `python tools/probes/ball_arrival_offset.py --bag ~/Desktop/rosbags/2026-08-10_16-30-44
  --thrower jugglebot --plane 809.08` → **`NO TRACK` on all 31**, i.e. the
  shipped estimator agrees the ball is not there.

**Scoped runs** (2026-08-10), for the pieces the gate covers but does not name:

- `pytest tests/motion/test_toss_record.py -q` → **47 passed in 0.28 s**.
- `pytest tests/ros/test_toss_record_publisher.py tests/ros/test_toss_record_miner.py
  tests/motion/test_toss_record.py -q` → **86 passed in 31.00 s** (the miner's
  bag-backed tests read `sensor_only`, which is what keeps them off the gate's
  critical path).
- `cd ros_ws && colcon build --packages-select jugglebot` → `Finished <<<
  jugglebot [2.61s]`, 1 package. (`jugglebot_interfaces` is untouched — this
  phase adds no `.msg`/`.srv`/`.action`, which is the whole point of
  JSON-in-`String`.)

**The one edit after the gate** is this Verification block itself. Naming its
test surface rather than appealing to the file extension, per CLAUDE.md and
`logbook/README.md` § "What the logbook tests actually check":
`tests/sim/test_logbook_front_matter.py` parses every committed entry and
asserts the required front-matter keys and an ISO `date` (this block touches
neither), and `tests/sim/test_logbook_search.py` indexes the real `logbook/`
directory but **skips `INDEX.md` outright** and silently drops any entry lacking
a `title` — so it would not catch a malformed entry, and the front-matter test is
the one carrying the coverage. Both re-run after this edit:
`pytest tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_search.py
tests/sim/test_plans_index.py -q` → **57 passed in 0.53 s**.

### Open

- **§ 3.7 item 5's timing gate is unsatisfiable on this plant.** It requires
  `sensor_poll_dt_ms_median` within 10 % of 20 ms; the measured median is 71 ms,
  so the gate refuses every record. 2c owns re-deriving it from the measured
  distribution rather than from the configured interval.
- **The measured 71 ms poll cadence has no diagnosis.** Configured 20 ms,
  achieved p5 32 / p50 71 / p95 111. Own investigation, can-bridge side.
- **Ball visibility in `/mocap_data` is an unchecked capture precondition.** The
  reference sitting has no unlabelled ball marker near the cup, so it cannot
  support an aim fit at any record-list completeness. Proposal: a P2 preflight —
  one throw, one miner run, require a non-null `land_xy_global_mm`.
- **SCL3300 async-read firmware follow-on** (registered by operator decision 2,
  not built here): restructure the Platform-Teensy `get_platform_tilt` path to
  timer-driven background sampling into a cache, so a tilt read never blocks the
  loop that streams hand moves. Also carried in the plan's § 10 and
  `catch-robustness.md`'s Open row.
- **`uptime_ms_at_release` is mined-only** in 2a. If a later phase needs it
  declared, that is a `/link_status` subscription on the coordinator and should
  be argued on its own merits, not slipped in.

---

## Phase 2b — the map, applied at zero (2026-08-11)

**The authority lands with its disable path proved rather than argued.** The aim
map is loadable, gated, observable and wired into the one seam that builds a toss
goal — and because no `config/toss_calibration.yaml` exists yet, every commanded
number is bit-for-bit what the machine did before this phase.

### What landed

| Deliverable | Where |
|---|---|
| The pure loader — schema v1, all-or-nothing validation, `map_version`, hull-clamped `lookup`, apply-time `clamp_total_aim` | `ros_ws/src/jugglebot/jugglebot/motion/toss_cal.py` |
| THE aim-angle → virtual-target conversion | `motion/trajectory/toss_release.aim_target_offset_mm` |
| The ONE per-goal lookup + apply | `reload_coordinator_node._toss_aim_for_goal`, called only from `_build_toss_cycle` |
| `toss/reload_calibration` (`std_srvs/Trigger`) with a version readback | `reload_coordinator_node._svc_reload_toss_calibration` |
| `toss/calibration_status` — latched JSON `loaded`/`applied`/`version` | `reload_coordinator_node._publish_toss_cal_status`, + the one bag-record list |
| The three `tier == TIER_8B` branches re-keyed on "the commanded release carries a non-zero tilt" | `_position_platform_for_toss`, `_toss_positioning_xyz`, the `catch/pretilt_hold` raise |
| Record fields `map_aim_rad` / `total_aim_rad` / `map_aim_mm_at_h` / `clamp_hits` / `toss_cal_*` filled for real | `_toss_record_fields` |
| Tests | `tests/motion/test_toss_cal.py`, `tests/ros/test_toss_calibration.py`, additions to `tests/motion/test_toss_release.py` |

### Discussion

#### 1. What the map STORES was the first decision, and it decides where the sign can be wrong

`aim_rad` stores the **commanded aim tilt**, in the production
`TiltedReleaseState.tilt_rx`/`tilt_ry` convention — not the measured plant bias.

The design's § 3.7 fit reduces each toss to
`b_i = map_aim_i + trim_aim_i + S⁻¹·land_err_i/(4h)` and calls it "the bias that
would have landed it on B", and § 3.6's trim update is `δ ← δ_prev + clamp(−r_n −
δ_prev, …)` — a *negated* estimate. Read together, both `map_aim_rad` and
`trim_aim_rad` are **corrections**, i.e. things the machine commands. Adopting
that as the storage convention has one consequence worth stating plainly: **the
runtime apply path contains no sign flip at all**, so there is no runtime place
for a sign to be wrong. Every sign question collapses into the fit tool (2c) and
into rung SC-0, which measures the sign on hardware instead of assuming it. It
also makes the corpus self-checking: for an 8a toss `aim_tilt_rx_rad` should
equal `map_aim_rad[0]`, and the node writes both from different objects.

The alternative — store the measured residual and negate at apply — was rejected
because it puts a sign in the one place nobody re-derives: the live path, at
10 goals/min, where a flip is invisible until a whole sitting has been thrown
against it.

#### 2. The aim is applied as a VIRTUAL TARGET, and the round trip is exact

D1 says the aim rides the existing tilted release path "with a virtual aim
target", and D2 says the three `TIER_8B` branches re-key on a non-zero tilt. What
the design does not spell out is the conversion: given a commanded aim
`(rx, ry)`, *how far* must the target move?

The exact form, derived from the same geometry `compute_release_state_tilted`
uses internally (the ball leaves ALONG the cup axis, so `v = |v|·a`, and
`v_z = Δz/T + g·T/2`):

```
d = (Δz + g·T²/2) · a_xy / a_z ,   a = cup_axis(rx, ry),
Δz = HAND_CATCH_OFFSET − HAND_THROW_OFFSET + arm·(1 − a_z)
```

**Probed before it was coded** (`/tmp/probe_toss_cal_aim.py`, 2026-08-11), over
h ∈ {0.45, 0.60, 0.78, 1.00} m × |aim| ∈ {0, 0.05, 0.10, 0.15, 0.5, 1.0}° × 5
azimuths × 3 goal poses: the production path derives back the requested aim to a
worst error of **8.35e-13 rad (4.8e-11°)**, and at zero aim the offset is
**exactly `[0.0, 0.0]`**, so `compute_release_state_tilted` reproduces
`compute_release_state` bitwise. The same probe pins the gain against the
design's own `b = 4h·θ` sizing model — **54.578 vs 54.454 mm/deg at h = 0.78,
+0.23 %** — which is the check that the model every constant in § 3.6 was sized
with still describes the code that applies them. Both numbers are now driven
round-trip tests, not restated residuals; a restated residual drifts in lockstep
with the algebra error it is supposed to catch.

The alternative — invert `tilt_to_throw` numerically to find the target — was
rejected as a second implementation of the aim geometry, which is precisely what
D2 exists to avoid.

#### 3. Two release states, because D4 means what it says

This is the non-obvious tradeoff of the phase. D4 requires the announcement to
carry the **uncorrected** landing: the announcement is the *prediction of where
the ball goes*, and after a correct aim correction that is B. Keeping it
uncorrected is what leaves the correlation → catch path, the receive-tilt
computation and the possession plausibility bound bitwise unchanged.

But the tilted path's `catch_point_global_mm` is the *virtual* target, and its
`launch_vel_mms` carries the aim's lateral component — so announcing straight
off the corrected state would move the announced landing by up to 55 mm and tilt
the announced arrival by up to 1°, changing the receive tilt on every aimed toss.

So the node now carries two states:

- `_toss_release_state` — **uncorrected**. Read by `_announce_toss`,
  `_toss_landing_global_mm` (possession plausibility) and the record's
  `catch_point_global_mm`. Every existing reader is untouched.
- `_toss_release_cmd` — **commanded**. Read by exactly two seams:
  `_position_platform_for_toss` (orientation + the swing-compensated pre-tilt
  xyz, via the single `_toss_positioning_xyz`) and the sequencer's `event_vel`.

The cost is a second object and a second thing to keep in step. What buys it back
is that **when the aim is zero the two are the same OBJECT, not merely equal** —
the disabled path executes not one extra floating-point operation, so "no map" is
identical to today by construction rather than by float luck. That is asserted
with `is`, not `==`, in `test_absent_map_leaves_the_release_state_untouched`.

A cheaper-looking option was to build the corrected state and then
`dataclasses.replace` its `catch_point_global_mm` back to B. It was rejected
because the *velocity* fields would still carry the aim, so the receive tilt
would change anyway — a fix that looks like D4 and is not.

#### 4. `pretilt_hold` re-keyed, and a deliberate 8b behaviour change

The three branches are now keyed on `_release_is_tilted(commanded release)`
rather than `tier == TIER_8B`. That is the design's instruction, and it is also
the honest predicate: the thing that matters is "is the commanded release
orientation non-level?", of which the tier was only ever a proxy.

The proxy and the predicate disagree in exactly one case: a **Tier-8b goal with
zero displacement**, where the tilt is exactly zero. Today that raises
`pretilt_hold`; after the re-key it does not. Accepted deliberately — with zero
displacement the stock pre-tilt's A→B translate is zero and its un-tilt is to
level, which is the pose already held, so it is the same argument F3 makes for
today's 8a. Recorded here because it is the one behavioural difference the re-key
introduces, and a future reader tracing "why did 8b stop holding the pre-tilt at
A == B" should find it named rather than inferred.

#### 5. Where the status fields went, and why not `TrajectoryStatus`

§ 3.7 asks for `toss_cal_loaded` / `toss_cal_applied` / `toss_cal_version`
"published on status", and § 6's P2 preflight greps them off
`/trajectory/status`. **That grep is not implementable.** `TrajectoryStatus` is
published by `trajectory_node`, which does not own this map (operator decision 7
puts it in `reload_coordinator_node`, because the map rewrites a GOAL while the
tilt map rewrites POSES at ingest) and therefore cannot know whether it is
applied. Cross-node field-filling would be a lie with a plausible-looking value
in it — the exact failure class the loaded-vs-applied distinction exists to kill.

The minimal faithful deviation: a **latched `std_msgs/String` JSON topic**,
`toss/calibration_status`, owned by the node that owns the map, carrying
`loaded`, `applied`, `version`, `path`, `dormant_reason` and both tilt-map
versions. Same D10 argument the `/toss/record` publisher already makes — a typed
field forces a `jugglebot_interfaces` rebuild, and a partial two-package colcon
build takes down every ball-op action. `TRANSIENT_LOCAL` because the map changes
only on load and reload, so an unlatched topic would be silent exactly when the
operator asks. The operator's one-liner becomes
`ros2 topic echo /toss/calibration_status --once`, and it is on the bag-record
list so a sitting's DORMANT-vs-applied verdict is recoverable afterwards.

The plan's § 6 P2 row is now wrong as written; it is carried in Open.

#### 6. The version hash is one step wider than the design's literal text

§ 3.7 says the hash covers "schema version, both axes, the bias grids". It covers
those plus `units.aim`, `anchor.aim_rad` and `speed.k_v`.

Root cause, which is the design's own stated intent: *two files whose applied
numbers are identical report the same version; an edit to a single applied node
changes it.* `units.aim` decides what the grids MEAN. `anchor.aim_rad` is the
session trim's warm-start prior and `speed.k_v` the speed prior — phase 2e
**acts on both**, so under the literal rule an edit to either would change the
machine's behaviour without changing the version, which is precisely the property
the version string exists to deny. Everything genuinely provenance (`captured`,
`requires`, `jacobian`, `stats`, `grid.z_mm`, `grid.orientation`) stays out, so
the fit tool's re-emit still cannot churn the version.

Both of the tilt map's own audit findings are inherited verbatim and re-tested:
float-normalisation (so `170` and `170.0` agree) and `ndarray → tolist()`
recursion (so an ndarray does not hash as its truncated repr and collide across
large grids).

#### 7. The authority bound is a magnitude, and it is checked twice

`TOTAL_MAX_RAD = 1.0°` per D7, applied to `hypot(rx, ry)` — **not** per axis. A
per-axis box would admit `hypot = 1.414°` at the corners, 41 % past the authority
that the composition-regime argument (C-LEVEL-2's 1°×1° cross term, 1.523e-4
rad), the cup-swing cap (1.131 mm, measured) and the landing-shift number
(54.578 mm at h = 0.78, measured) are all sized on.

Checked at parse time per node, and again at apply via `clamp_total_aim`. The
apply-time clamp is *provably* a no-op in 2b — a bilinear blend of bounded
vectors is a convex combination, and there is a test that sweeps a saturated map
to say so — but D7 requires the total to be re-clamped at apply rather than only
at update, and 2e's trim adds to this number. Landing the enforcement point with
the seam it protects is cheaper than bolting one on beside it later.

#### 8. Dormancy fails CLOSED on an unknown tilt-map version

D3 makes a `requires.tilt_map_version` mismatch dormant. The case the design does
not name is an **empty** live version — no `/trajectory/status` heard yet, or a
build that publishes none. That is treated as a mismatch: *"I cannot verify which
levelling layer is underneath me"* is not *"the right one is underneath me"*, and
applying an aim residual on an unverified layer 0 is exactly the double-count D3
exists to prevent. It costs only precision, and it self-heals — a
`/trajectory/status` arrival carrying the matching version flips the map to
applied live, with no reload, and republishes the status topic. Tested.

The WARN is loud but fires **once per (map, live tilt-version) pairing**: a
dormant map at 10 goals/min would otherwise bury every other console line, the
4091-ERROR-lines-in-41 s failure mode the seed gate already exists to prevent.

### Fix

- **`motion/toss_cal.py`** — new pure module, C-LEVEL-2 loader shape: candidate
  search via `find_repo_root` (imported from `tilt_map`, **never** a fixed
  `__file__` walk), authoritative `$JUGGLEBOT_TOSS_CAL` override, source tree
  before ament share, all-or-nothing `parse_toss_cal`, hull-clamped `lookup`,
  `map_version` over float-normalised applied numbers only, `clamp_total_aim`,
  and `TossCal.provenance_mismatch` for D3.
- **`motion/trajectory/toss_release.aim_target_offset_mm`** — THE aim-angle →
  virtual-target conversion, refusing an aim past the same 12° ceiling
  `compute_release_state_tilted` enforces.
- **`reload_coordinator_node`** — owns the map (decision 7): loads in `__init__`
  and on `toss/reload_calibration`, publishes `toss/calibration_status`,
  evaluates `_toss_aim_for_goal` **once per goal** in `_build_toss_cycle`,
  carries `_toss_release_cmd` beside `_toss_release_state`, re-keys the three
  tilt branches, and fills the record's applied-calibration block.
- **`setup.py`** — conditional `config/toss_calibration.yaml` share install,
  mirroring the tilt map's row and its `--force-uninstall` caveat.
- **`jugglebot_launch.py`** — `/toss/calibration_status` joins THE ONE list.
- **`ros_ws/docs/choreography.md`** — regenerated (the new topic + service).

### Verification

- `./run_tests.sh --full` (run 2026-08-11) → **5009 + 9 passed, 3 xfailed in
  515 s** (parallel 471 s, serial 44 s). `--full` because a plan-phase closure
  gets every tier.
- `pytest tests/motion/test_toss_cal.py tests/motion/test_toss_release.py
  tests/ros/test_toss_calibration.py -q` (run 2026-08-11) → **203 passed in
  6.64 s** — the scoped iteration record.
- `/tmp/probe_toss_cal_aim.py` (run 2026-08-11) → worst virtual-target
  round-trip error **8.35e-13 rad**; zero-aim offset exactly `[0.0, 0.0]`; gain
  **54.578 mm/deg at h = 0.78** vs the design's idealised `4h` = 54.454.
- `cd ros_ws && colcon build --packages-select jugglebot` (run 2026-08-11) →
  `Finished <<< jugglebot`, 1 package. `jugglebot_interfaces` is untouched:
  `Trigger` + `String` were chosen precisely so this phase needs no two-package
  build.
- The gate's own catch: `tests/ros/test_choreography_map.py` failed on the first
  gate run because the new topic and service made `ros_ws/docs/choreography.md`
  stale. Regenerated with `python tools/gen_choreography_map.py`.

**Three defects in this phase's own work, found by the pre-commit audit and
fixed before the commit** — recorded because two of them are test-integrity
faults, the kind that are invisible precisely because the test passes:
`test_the_repo_ships_no_toss_calibration_yet` walked one directory too few and
asserted against `<repo>/tests/config/`, so it could never fail (now asserts the
walk landed on the repo root first); a `textwrap.dedent` over a whole module in
the `__file__`-walk guard did nothing; and 2a's `FIELDS` notes still read "null
until phase 2b" for the three `toss_cal_*` fields this phase fills.

**The one edit after the gate** is this Verification block. Naming its test
surface rather than appealing to the file extension, per `logbook/README.md`
§ "What the logbook tests actually check":
`tests/sim/test_logbook_front_matter.py` parses every committed entry and asserts
the required front-matter keys and an ISO `date` (this block touches neither),
`tests/sim/test_logbook_search.py` indexes `logbook/` but skips `INDEX.md` and
drops any entry lacking a `title`, and `tests/sim/test_plans_index.py` pins the
plan/INDEX pairing. Re-run after this edit: `pytest
tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_search.py
tests/sim/test_plans_index.py -q` (run 2026-08-11) → **57 passed in 0.50 s**.

### Open

- **The plan's § 6 P2 preflight row is wrong.** It greps `toss_cal_*` off
  `/trajectory/status`; the fields live on `/toss/calibration_status` because a
  different node owns the map. The plan text is corrected in the same commit;
  the operator runbooks that copy that row have not been touched and should be
  when 2f lands the tool that reads it.
- **`ESTIMATOR_VERSION = 'arrival-offset/1'` is asserted, not yet earned.**
  `tools/probes/ball_arrival_offset.py` carries no version of its own, so 2b
  declares the string and gates on it. 2c must write the same string from the
  fit tool AND bump it whenever the fit-plane rule, band width, lateral gate or
  minimum sample count changes — otherwise the gate is decorative.
- **`aim_rad` is stored in the commanded convention, so SC-0 is the only thing
  that can prove the sign.** Nothing in 2b can: with no map loaded there is no
  aim to be wrong about. The 2c closed-loop sign test and the P5.4
  doubled-bias-on-one-node first application remain the two gates that matter.
- **Tier 8b composes the aim in DISPLACEMENT, not in tilt.** The virtual target
  moves by the aim's own ballistic offset, so a displaced 8b throw gets the aim
  added to its target rather than to its angle — physically the right
  composition (lateral displacements add; tilts only approximately do), but the
  map is fitted at 8a and R6 refuses a non-8a capture, so an 8b aim is
  extrapolation. It applies uniformly because the aim is a property of the
  PLANT, not of the tier; if that turns out to be wrong, the fix is a tier gate
  in `_toss_aim_for_goal`, one line.

---

## Phase 2c — the fit, the analyser, and the closed loop that proves the sign (2026-08-11)

**Still zero control authority, and still desk-side.** Nothing in this phase runs
on the robot or changes what the machine commands. What it adds is the ability to
turn a corpus into `config/toss_calibration.yaml` — and, more importantly, the
ability to find out **offline, before a sitting**, whether that map would aim the
machine or fight it.

### What landed

| Deliverable | Where |
|---|---|
| The pure fit core — partition rule, reduction, admission, gates, document build | `tests/hardware/toss_fit_lib.py` |
| The thin desk-side CLI over it (`--dry-run` / `--no-apply` / `--group` / `--reload` + version readback) | `tests/hardware/toss_cal_fit.py` |
| The analyser — heat map + quiver, per-node n/sd, anchor series, residual-vs-uptime scatter, map-vs-map diff, `--group` A/B, HTML+PNG to `temp/reports/`, `--json` | `tools/toss_cal_analyse.py` |
| The synthetic replay corpus (also the CLI's own driver) | `toss_fit_lib.synthetic_corpus` |
| Tests | `tests/motion/test_toss_cal_fit.py` (61), `tests/motion/test_toss_cal_analyse.py` (14) |

### Discussion

#### 1. The design's reduction has a sign error, and it is the one that matters

§ 3.7 item 3 writes the per-toss reduction as

```
b_i = map_aim_i + trim_aim_i + S⁻¹ · land_err_i / (4·h_i)          ← a PLUS
```

That expression does not close. With applied aim `A` and plant bias `ψ`, the
landing error is `land_err = J·(A + ψ)`, so `J⁻¹·land_err = A + ψ` and the design's
formula evaluates to **`2A + ψ`**. At `A = 0` it returns the plant bias
*uncancelled* — and `aim_rad` is defined as the **commanded** aim (`toss_cal.py`'s
docstring, 2b), so shipping `+ψ` commands the machine to tilt *into* its own error.
Applied once it roughly doubles the landing error; applied again on the next
capture it diverges.

The fixed point is the minus:

```
b_i = A_i − J⁻¹ · land_err_i  =  A − (A + ψ)  =  −ψ      independent of A
```

and that independence is precisely the property § 3.7 item 3 is *arguing for* two
sentences later ("captures do not have to run with the map uninstalled … a
converging fixed point rather than a one-shot measurement"). So the design's prose
and its formula disagree with each other; the prose is right.

**Why this was worth catching in a build phase rather than at SC-0.** SC-0 measures
`S` on hardware and blocks the grid capture, so a wrong sign would eventually have
surfaced — after a power-up, a level, an arm and 25 tosses. It would have surfaced
as *"the gain is right but every sign is inverted"*, which reads exactly like a
frame-convention problem in the geometry and would have sent the session hunting
`tilt_geometry` rather than the fit. Catching it offline costs nothing and leaves
SC-0 doing what it is actually for: confirming that the *plant* obeys the
production model.

#### 2. The sign is never written down — it is differentiated out of the apply path

The obvious implementation of `S` is to write the 2×2 matrix into the fit. That is
a second place for a sign to be wrong, and R1 lists exactly that class as one of
the four silent routes to a map that aims worse than none.

Instead `aim_landing_jacobian(T, z)` **finite-differences the production
`toss_release.aim_target_offset_mm`** — the same function
`reload_coordinator_node._toss_aim_for_goal` calls to apply the map — at a 1e-6 rad
step. There is consequently one implementation of the aim geometry in the repo, and
the fit's inverse is that implementation read backwards. If the apply path's
convention ever changes, the fit's changes with it, in the same direction, in the
same commit.

Doing it that way immediately produced a result worth stating plainly, because it
falsifies the intuitive guess:

```
J(h = 0.78 m, z = 170 mm)  =  [[    0.0,  3126.5],
                               [-3126.5,     0.0]]  mm/rad
```

`S = [[0, 1], [−1, 0]]` — a **90° rotation**, not a scaled identity. A tilt about
`+rx` moves the ball in `−y`; a tilt about `+ry` moves it in `+x` (right-hand rule
on the cup axis). An identity `S` — which is what "correct an x error with an x
tilt" writes down — would push the ball sideways instead of back, and would look
like a plausible field on the analyser's quiver. The design's insistence that `S`
be *measured* rather than assumed is vindicated by the production geometry before a
single ball flies; the same rotation is why the analyser draws its quiver in
landing space (`J·aim`) rather than in tilt space.

The magnitude also pins the design's idealised `4h`: 3126.53 vs 3120.0 mm/rad, a
**0.209 %** difference carried by the `Δz` and drop terms the exact form keeps.
`4h` stays valid for back-of-envelope work and a test holds the two within 1 %.

(Reconciling with 2b's number, which is the same physical fact reported
differently: 3126.53 mm/rad = **54.568 mm/deg**, the *derivative at zero aim*,
against 2b's probe figure of 54.578 mm/deg, a *secant at finite aim*. The model
has real curvature — the secant at 1° is 3126.86 mm/rad = 54.574 — so the two
differ in the fourth significant figure by construction, not by disagreement.)

#### 3. The acceptance test is a closed loop, not a restated residual

The plan asked for *"inject a known synthetic aim bias into a replayed corpus, run
the REAL fit, assert it recovers sign AND magnitude"*, and warned that a residual
restated as an assertion drifts in lockstep with a sign flip. The shipped test goes
one step further than recovering the number:

1. `synthetic_corpus` builds `land_err_mm` from the **forward** production model
   `aim_target_offset_mm(A + ψ)`;
2. the **real** fit runs — `fit_nodes` → `anchor_estimate` → `build_map_document`
   → the production `toss_cal.parse_toss_cal`;
3. a second corpus is replayed with that map installed and applied **through the
   production apply path** (`toss_cal.lookup` → `clamp_total_aim` →
   `aim_target_offset_mm`);
4. the assertion is on the *landing error*: 8+ mm uncorrected, under 1 mm
   corrected.

So the round trip exercises the production sign convention in both directions
instead of restating one of them, and a sign flip does not merely fail the
assertion — `test_a_sign_flipped_fit_doubles_the_error` pins that it fails it by
**>1.8×** the uncorrected error, which is the consequence the whole SC-0 blocking
rung exists to prevent.

A companion test drives the same loop with a **spatially varying** bias and checks
the shipped grid node by node through `toss_cal.lookup`, on a deliberately
asymmetric field — a transposed `[iy][ix]` is invisible on a symmetric one.

#### 4. The timing gate the design handed to this phase, re-derived from the bag

§ 10 recorded that § 3.7 item 5's timing-fit gate — `sensor_poll_dt_ms_median`
within 10 % of the configured `JB_BD_CHECK_INTERVAL_MS` (20 ms) — is unsatisfiable,
and gave 2c the job of re-deriving it. Measured per-record on
`2026-08-10_16-30-44` (31 self-tosses, mined 2026-08-11):

| statistic | min | p5 | median | p95 | max |
|---|---|---|---|---|---|
| per-record `sensor_poll_dt_ms_median` | 60 | 63 | **70** | 80 | 87 |

The shipped gate refuses 100 % of them. It is not a gate, it is an outage.

The re-derivation started from what the gate *protects* rather than from the
constant. The measurand is `t_departure_raw_ros − announce_throw_time_ros`, and a
raw sensor edge is quantised by the poll cadence Δ, so its per-sample sd should be
`Δ/√12`. Over the reference bag's 31 departures:

```
measured sd  20.51 ms
predicted    20.50 ms   (Δ/√12 at Δ = 70.998 ms)
ratio        1.001
```

**The entire observed dispersion of the release-shift measurand is the
instrument's quantisation.** The release timing itself is more repeatable than the
sensor can see. That reframes the gate from "does the cadence match a number" to
"is the cadence precise enough for the estimate to gate", which is what shipped:

- **floor = `JB_BD_CHECK_INTERVAL_MS` (20 ms)** — a per-record median *below* the
  firmware's own poll interval is not a fast sensor, it is a stamp that is not the
  poll stamp (different firmware, or a mined field that does not mean what the fit
  thinks). Refuse a measurand of unknown provenance rather than fit it.
- **ceiling = 200 ms**, sized on *reachability*: with quantisation-limited samples
  `se = Δ/√(12·n)`, so the τ trim's `se ≤ 5 ms` needs `n = Δ²/300`. At Δ = 200 ms
  that is **133** admitted tosses — more than the entire 129-toss first capture
  (§ 6). Past that cadence a timing fit is not merely imprecise, it is unreachable
  inside a sitting, and saying so is more honest than accumulating a fit that will
  never gate. It is 2.3× the worst per-record median measured, so it does not bind
  on today's plant.

The underlying 3.5× poll-cadence gap still has no diagnosis and is still a
can-bridge question, not this plan's.

#### 5. Two smaller deviations, both weakenings of a claim the design overstated

**(a) The home-referencing invariance is not byte-identical here.**
`tilt_cal_grid`'s equivalent property is exact because its reduction is a plain
subtraction of inclinometer readings. This reduction runs through the ballistic
model, which is linear only to second order in the aim, so a 2.4 mrad common
shift — the worst observed level-to-level jump — perturbs each shipped node by at
most **2.6e-7 rad** on the test fixture: **8e-4 mm** of landing shift at h = 0.78 m,
**four** orders below the 0.15° (2.618e-3 rad) flat-field floor. The test asserts
1e-6 rad rather than equality, and a real failure of home-referencing would show
up at the size of the shift itself (2.4e-3), four orders larger.

**(b) The flat-field guard refuses, but with a documented override.** § 3.8 says
"WARN and refuse to write" and that is what ships, plus `--allow-flat-field`, which
is stamped into `captured.flat_field_override`. Root cause for having an override
at all: a *common-mode-only* plant produces a flat grid legitimately — all of the
bias lives in `anchor.aim_rad` — and on that plant a refusal with no escape would
make the tool unable to ship the one map its own § 3.2 architecture predicts.

#### 6. What the fit does when a node has nothing (D15, and the hole D15 leaves)

A **thin** node (some tosses, fewer than `N_MIN` = 8) keeps its previous value,
marked `stale: true` with its `n` and source — the deliberate deviation from the
tilt tool's all-or-nothing write, because this map is an incremental refinement and
refusing would block 24 good nodes over one thin week. It is never interpolated
from its neighbours.

D15 does not cover a node that was **never** measured and has no previous value.
Shipping a zero there is not neutral: bilinear interpolation would drag its
measured neighbours toward zero across half a cell, inventing calibration between
them. So the ball-actually-flew guard refuses the write and names the nodes. The
same guard refuses a corpus in which **nothing flew anywhere** even when a previous
map exists — the toss analogue of the 2026-07-15 DISARMED tilt capture that wrote a
plausible all-zeros map because the platform never moved.

Related: a previous map on a **different grid** is not carried by index. `[iy][ix]`
on a different grid reads a different physical pose; that is the transposed-index
error dressed up as a refresh.

#### 7. The only real corpus that exists still cannot support a fit, and the tool says so

Run against the reference bag's mined corpus, the fit refuses:

```
EXCLUSIONS
  no_mocap_fit                     31
REFUSED: BALL NEVER FLEW: not one of the 9 grid nodes has a single admitted toss.
```

That is the § 10 finding from 2a — the mocap cannot see the ball over the cup on
that sitting, 0/31 usable — surfacing through the fit rather than through a
reader's memory. The refusal ordering was changed for this: the ball-flew guard
runs **before** the anchor estimate, because on such a corpus the anchor also fails
and its message ("no admitted toss at the home node") is the first symptom to
raise, not the root cause.

### Fix

- **`tests/hardware/toss_fit_lib.py`** (new, pure, importable, no ROS): the
  partition rule and census; `aim_landing_jacobian`; `reduce_to_aim`;
  `admit_for_aim` / `_timing` / `_speed`; `fit_nodes` with the D15 thin-node rule;
  `anchor_visits` / `anchor_estimate` (mean over VISITS, not tosses — a visit with
  12 tosses must not outvote three with 4 each); `flat_field_verdict` and
  `ball_flew_verdict`; `sigma_land_mm`, `r_eff_mm`, `speed_fit`, `timing_fit`,
  `score_groups`; `build_map_document` / `validate_map_document` (through the
  production loader) / `dump_map_yaml` / `diff_documents` / `write_target_path`;
  `synthetic_corpus`.
- **`tests/hardware/toss_cal_fit.py`** (new): argument parsing, ordering and
  printing only. Refusals hoisted ahead of any work. `rclpy` is imported inside the
  `--reload` function, so the desk-side default path never needs a graph and
  `tests/motion/` can import the CLI on a box with no ROS.
- **`tools/toss_cal_analyse.py`** (new): the four panels, the diff, the A/B rows.
- **Tests**: `tests/motion/test_toss_cal_fit.py`,
  `tests/motion/test_toss_cal_analyse.py`.

### Verification

- `./run_tests.sh` (run 2026-08-11) → **4647 passed in 210.23 s** (before the
  analyser commit) and **4661 passed in 210.77 s** (after it).
- `./run_tests.sh --full` (run 2026-08-11) → **5084 + 9 passed, 3 xfailed in
  518 s** (parallel 474 s, serial 44 s). `--full` because a plan-phase closure
  gets every tier.
- `pytest tests/motion/test_toss_cal_fit.py -q` (run 2026-08-11) → **61 passed in
  3.05 s**. `pytest tests/motion/test_toss_cal_analyse.py -q` (run 2026-08-11) →
  **14 passed in 4.47 s**.
- `python tools/probes/toss_record_miner.py --bag 2026-08-10_16-30-44 --jsonl`
  (run 2026-08-11) → 31 rows, `usable_for_aim_fit: 0/31`, per-record poll cadence
  60/63/70/80/87 ms (min/p5/median/p95/max) — the measurement the timing gate was
  re-derived from.
- `python tests/hardware/toss_cal_fit.py --corpus <that corpus> --dry-run`
  (run 2026-08-11) → exit 1, `REFUSED: BALL NEVER FLEW`, exclusions
  `no_mocap_fit 31`.
- A **200-seed** probe of the noisy closed loop (2026-08-11) put the empirical
  pooled-error sd at **1.016 / 0.993 se** per axis, the worst error over 400
  axis-draws at **3.27 se** and the 99.9th percentile at 3.04 — which is where
  the test's 4-se bound comes from. (An earlier 20-seed run read 1.19 se and was
  used to justify the bound as "trimmed-mean variance inflation"; at 20 seeds an
  sd estimate carries ~16 % relative uncertainty, so that was noise, and the
  trimmed mean's real cost at n = 30 is 2.7 % before pooling washes it out. The
  bound did not change; the reason for it did, and the wrong reason was caught by
  the pre-commit audit.)
- **No `colcon` build**: nothing under `ros_ws/` changed. The fit imports
  `jugglebot.motion.toss_cal` and `jugglebot.toss_record` but adds nothing to the
  package.

### Open

- **`--group` A/B scoring lives in the analyser AND the fit**, sharing one
  implementation (`toss_fit_lib.score_groups`). § 3.7 item 8 put it on the fit;
  the analyser is where an operator will actually look. One function, two front
  doors.
- **`ESTIMATOR_VERSION` is now written by the fit** into `requires.estimator_version`
  and the estimator's own parameters are pulled from
  `tools/probes/ball_arrival_offset` at write time rather than copied. The 2b open
  item — *bump it whenever the fit-plane rule, band width, lateral gate or minimum
  sample count changes* — is now enforceable by inspection of the artefact, but is
  still a human obligation: `ball_arrival_offset.py` carries no version of its own.
- **G4 (plant health) still cannot be enforced from a record.** The PLANT block
  ships null (2a, § 10), so `iq_brake_min_a` is unavailable and § 7 R3's
  *"toss_cal_fit prints a REFUSE when the braking-clamp median sits outside the
  post-restore band"* is **not implemented**. `admit_for_aim` names the guards it
  does enforce and does not pretend to this one.
- **G6/G8 are reported, not enforced.** The residual-vs-uptime trend and the
  anchor peak-to-peak are printed by both tools and carried in `--json`; the
  design's D16 refusal (*"the analyser refuses a timing fit whose within-session
  trend exceeds the between-node signal"*) needs a comparison the fit has and the
  operator currently makes by eye. Wiring it is a small follow-on and belongs with
  the first real corpus that has an hours-long uptime span.
- **`N_MIN` (8), `TRIM_FRACTION` (0.10), `NODE_SNAP_TOL_MM` (1.0) and the
  200 ms timing ceiling stay PROVISIONAL** in the tilt tool's convention, so the
  commit that pins each is easy to find.
---

## Phase 2d — auto-reload, the reopened retry, and a covariate that does not fit (2026-08-11)

**The first phase of this build that commands motion.** 2a–2c were an
instrument, a lookup applied at zero, and an offline fit; 2d gives a
`TossContinuous` session one motion-bearing interlude — `go_home` plus the
shipping reload sequence — so a sitting survives a dropped ball instead of ending
on one. Everything it commands is an existing, hardware-validated mechanism; what
is new is *when* they run and *what refuses them*.

### What landed

| Deliverable | Where |
|---|---|
| `on_empty_cup` (IDL default `"STOP"`) + `max_reloads` (0 ⇒ config 3) + `reloads_used` on the result | `jugglebot_interfaces/action/TossContinuous.action` |
| The whitelist resolver — anything that is not exactly `RELOAD` is `STOP` | `toss_session.resolve_on_empty_cup` |
| `SESSION_ACTION_RELOAD` / `SESSION_PHASE_RELOAD`, the budget + floor counters, the interlude handshake | `toss_session.py` |
| The § 3.9 ladder: precondition gate → verified-arrival recentre → the reload FSM → settle → `RELOAD_SETTLE` | `reload_coordinator_node._run_reload_interlude` and friends |
| BB fail-open boot fence (`_bb_ball_in_hand_observed_false`) | `reload_coordinator_node._on_heartbeat` |
| The BB terminal-outcome relay, and the targeted `THROW_ABORTED_NOT_SETTLED` retry | `ball_butler_node` → `bb/throw_outcome` → `_bb_throw_outcome_since` |
| `ABORTED_NO_RELEASE` single retry, gated on a valid-HELD cup (operator decision 6) | `toss_session.note_cycle_result(..., ball_evidence=)` |
| Layer 1.5 — N reads at a fixed gap in the QUIESCENT dwell, recorded as a covariate | `_maybe_read_dwell_tilt`, `_dwell_tilt_fields` |
| `reload_settle` / `retry_of` / `goal_on_empty_cup` / `goal_max_reloads` / the `dwell_tilt` block, written | `_open_toss_record`, `_toss_record_fields` |
| Four config keys | `jugglebot_operational.toss_session_{max_reloads,floor_pause_every,dwell_tilt_reads,dwell_tilt_gap_s}` |

### Discussion

#### The design's Layer-1.5 read budget does not fit in the dwell, and the arithmetic says so

§ 3.10 asks for **N = 8 reads at a 0.15 s gap, ~1.2 s of a 6.0 s dwell**. That
reads as 20 % of the window. It is not: the *quiescent* dwell — the only place
§ 3.10 permits a read — is

    quiescent = dwell_time_s − throw_delay_s − (CAUGHT-verdict latency)

because a cycle's release is its own accept + `throw_delay`, so cycle N+1 STARTS
at `landing + dwell − throw_delay` and everything after that instant is the next
cycle's own countdown, PREPARE and THROW included. At the shipped defaults
(dwell 6.0, delay 5.0) that is **1.0 s minus the 0.202–0.442 s verdict latency ≈
0.7 s**, not 6.0. Eight reads do not fit; one or two do.

This was not visible from the design text and it is the kind of error that would
have shipped as a mystery ("why is `dwell_tilt_n` always 1?"). Three responses
were considered:

1. **Read during cycle N+1's CHECKING/POSITIONING**, which is before PREPARE.
   Rejected: § 3.10's rule 1 is *reads never overlap PREPARE→THROW*, and the only
   thing that currently makes that rule STRUCTURAL rather than a check is that
   `_run_toss_cycle` blocks the session loop for a cycle's whole life. Putting a
   read inside the cycle tick converts a structural guarantee into a conditional
   one, on a service that blocks the Platform-Teensy loop streaming hand moves.
   Wrong trade for a covariate with zero authority.
2. **Raise the default dwell** so the schedule fits. Rejected here: operator
   decision 3 fixed the cadence at 6.0 s, and this build does not get to move it
   to make an instrument comfortable.
3. **Ship the schedule, let the degrade rule do exactly what it was written
   for**, and state the arithmetic where the operator will meet it — in the
   config comment, the method docstring and this entry. Chosen. `dwell_tilt_n`
   of 1–2 with `dwell_tilt_degraded` true is the *correct* reading at the shipped
   cadence, and an operator who wants the full N buys it with `dwell_time_s ≈
   7.5 s+` on the goal. That is one number in the capture tool, not a firmware
   change.

The follow-on the design already registered — timer-driven background SCL3300
sampling into a cache, so a tilt read is a cache read — dissolves the whole
constraint. Until it lands, the dwell-only schedule and degrade-never-delay are
the fence.

#### `THROW_ABORTED_NOT_SETTLED` exists, but nothing could see it

§ 3.9 makes the BB not-positioned-in-time retry conditional: *"the retry must be
targeted at the identifiable code … if no distinct code exists, the retry is not
shipped."* The code exists — `BallButlerCommandOutcome.THROW_ABORTED_NOT_SETTLED`
= 41, observed twice on hardware (2026-07-23 and 2026-07-24 sittings, both
`axis=YAW`). But tracing it from firmware to consumer showed it could not reach
one: `bb/throw_at_target` is **fire-and-forget** — `ball_butler_node` publishes
the announcement and returns `success=True` as soon as the goal is dispatched,
and the firmware's terminal `CMD_RESULT` arrives later on a callback that only
**logs** it. From the reload FSM's seat a NOT_SETTLED throw is indistinguishable
from an ordinary `MISSED`: the announcement landed, no ball came.

So the design's binary — *ship the targeted retry, or ship nothing* — had a third
option it did not anticipate: **make the code observable, then target it.**
`ball_butler_node` now relays `result.message` on `bb/throw_outcome`
(`std_msgs/String`; its leading token is the enum member name, so the consumer
compares a named code rather than pattern-matching a sentence), and the
coordinator caches it with an arrival stamp.

The alternative rejected explicitly: infer NOT_SETTLED from the *absence of a
tracked ball* (no `/balls` entry ever went IN_FLIGHT with `destination == us`).
That proxy is available today and needs no wire — and it is wrong, because it
also fires on every tracker failure, which is a different fault with a different
remedy. The design's own words rule it out: a blanket retry "would swallow the
fail-open boot bug and every real BB fault." A retry that asks an unwell machine
to throw more real balls has to be keyed on the machine saying what is wrong.

Two guards make the relay safe to consume: the cached outcome is ignored unless
it is stamped **after this attempt began** (a stale NOT_SETTLED must not licence
retrying a reload that failed for another reason), and every retry is charged to
`max_reloads` — the retry re-enters the reload FSM and each re-entry is a real
ball, so it cannot be free.

#### The completion test moved from cycles to throws, and that is why a drop costs no data point

Before this phase, `note_cycle_result` ended the session at
`_cycle_index >= num_throws`. With auto-reload, a `REJECTED_NO_BALL` cycle
consumes an index while flying nothing — so a 10-throw session with three drops
would silently deliver seven data points. The completion test is now
`_throws >= num_throws`.

The two are **identical for every session the pre-2026-08-11 machine could run**,
and that is the argument for making the change here rather than treating it as a
behaviour edit: the only outcomes that do not increment `_throws` are the
`REJECTED_*`/`ABORTED_*` family, and before this phase every one of those stopped
the session before the check was reached. Nothing that could previously happen
lands differently. The runaway that the cycle counter used to fence is now fenced
by `max_reloads` and by `NO_RELEASE_MAX_CONSECUTIVE`, which are the fences that
actually bound the new loops — a cycle counter never did.

#### One sensor rung, two codes

§ 3.9 names a single sensor code (`STOPPED_SENSOR_UNKNOWN`) for the rung "hand
sensor `ball_held_valid` and reads empty". Tri-state means that rung can fail two
ways, and they are not the same failure. UNKNOWN is blindness: refuse, because a
dead sensor must not licence an autonomous BB throw at a cup nobody can see into
— the fail-open default this project declined to copy from BallButler. SEATED is
a **contradiction**: the interlude is only entered from `REJECTED_NO_BALL`, which
is minted on a valid EMPTY read moments earlier, so a SEATED read here means
something is wrong with the belief — and acting on it throws a real ball at a
hand that already holds one. Shipping `STOPPED_CUP_NOT_EMPTY` is a deviation from
the design's letter in the fail-closed direction, and it names the state the
operator has to go and look at.

#### `_execute_reload` was not refactored, deliberately

The interlude drives the reload FSM through the node's own `_step_sequence` and
its own `_safe_on_early_exit` — the mechanism is shared, so a rung added to the
reload ladder lands in one place and both callers see it. What is duplicated is
the ~15-line loop shell, because the rest of `_execute_reload` is goal-handle
bookkeeping the interlude does not have: it owns no handle, and a session's
interlude terminal is not the session's terminal. Refactoring would have edited a
path with four sittings of hardware evidence behind it to serve a caller with
none. The drift risk that matters (the ladder) is closed; the drift risk that
remains (a loop-shell edit) is visible in two adjacent methods.

#### S2 is amended, not quietly broken

`toss_session`'s S2 said *"the session commands NO motion of its own."* That is
now false, and the module docstring says so in those words rather than letting a
future reader discover it. The amendment is bounded three ways: the interlude is
entered from `REJECTED_NO_BALL` **only** — the one toss terminal where the FSM
provably commanded nothing (minted in CHECKING, `_terminal_action` returns
`ACTION_NONE`), so the machine is quiescent; every rung is an existing validated
mechanism; and it is fenced by the budget and the floor tally. A session that
omits `on_empty_cup` is bit-unchanged.

#### The recentre pad is measured, not chosen

`_go_home()` returns on the service ACK at plan-**install**, not on arrival — the
same trap the MISS-cleanup floor already documents — so the interlude waits the
profile out and then confirms a **fresh** commanded position inside
`_RELOAD_CENTERED_TOL_MM` (66.53 mm). The profile is deterministic:
`planner.build_return_to_neutral` takes `max(go_home_duration_s 2.0,
min_move_duration_s 0.20)` and an infeasible move is refused at the service call.
The only unmodelled terms are the ack→install latency and the publish period of
the channel being read, so the pad was sized on the latter: across five bags of
2026-08-10 (`12-06-49`, `15-16-03`, `16-04-26`, `16-13-48`, `16-30-44`),
**15,409 inter-sample gaps** on `/trajectory/status` — the SAME 5 Hz timer that
publishes `/trajectory/commanded_position` — ran **median 0.200 s, p99 ≤ 0.252 s,
worst 0.643 s**. `_RECENTRE_VERIFY_PAD_S = 1.5` is 2.3× that worst gap and 1.5×
the `_TRAJ_STATUS_STALE_S` freshness window the read applies. Erring long is
free: by then the platform is parked, and the only cost of a longer window is
time on a path that is already failing.

### Fix

Nine surfaces, in dependency order.

1. **`TossContinuous.action`** — `string on_empty_cup "STOP"`, `int32
   max_reloads`, `int32 reloads_used`. The string default is load-bearing and the
   node re-applies it through `resolve_on_empty_cup`, which whitelists the one
   dangerous value; a negative `max_reloads` is `REJECTED_BAD_GOAL(max_reloads)`
   rather than coerced.
2. **`config/hardware_config.yaml`** — `toss_session_max_reloads: 3`,
   `toss_session_floor_pause_every: 5`, `toss_session_dwell_tilt_reads: 8`,
   `toss_session_dwell_tilt_gap_s: 0.15`; regenerated and re-run for determinism.
3. **`toss_session.py`** — the resolver, the two counters, `SESSION_ACTION_RELOAD`
   / `SESSION_PHASE_RELOAD`, `note_reload_result`, the `ball_evidence`-gated
   `ABORTED_NO_RELEASE` retry with its consecutive gauge, the two inherited flags
   (each consumed at `START_CYCLE`, so exactly one cycle wears each), and the
   completion test.
4. **`reload_coordinator_node.py`** — `_reload_interlude_gate`,
   `_recentre_for_reload`, `_run_reload_interlude`, `_run_one_reload_attempt`,
   `_settle_after_reload`, the heartbeat fence, `_on_bb_throw_outcome` /
   `_bb_throw_outcome_since`, `_maybe_read_dwell_tilt` / `_read_platform_tilt` /
   `_dwell_tilt_fields`, the `get_platform_tilt` client, and the session-loop
   wiring.
5. **`ball_butler_node.py`** — the `bb/throw_outcome` relay (telemetry only;
   guarded so it can never break the result chain).
6. **The record** — `goal_on_empty_cup`, `goal_max_reloads`, `reload_settle`,
   `retry_of`, and the `dwell_tilt` block, all written by `_toss_record_fields`.
   `_toss_uid` is now one formula in one place, because `retry_of` names a uid it
   minted.
7. **`ros_ws/docs/choreography.md`** — regenerated; it picks up both new wires.
8. **Tests** — 34 new (see Verification).
9. **`plans/active/toss-selftuning.md`** — the 2d row marked landed, § 10 gains
   the dwell-window finding.

### Verification

- **The gate** (`./run_tests.sh`, run 2026-08-11): **RESULT PASS — 4752 passed
  of 5187 collected (435 deselected: the `nightly` tier), 221 s total**
  (parallel 213 s, serial phase empty as the wrapper reports).
- **`colcon build --packages-select jugglebot_interfaces jugglebot`**, run
  2026-08-11: **2 packages finished, 0 failed** (interfaces 2 min 9 s, jugglebot
  2.49 s). Interfaces FIRST — the `.action` changed, and a partial build takes
  down every ball-op action. Verified against the BUILT IDL rather than the
  source: `TossContinuous.Goal().on_empty_cup == 'STOP'`,
  `max_reloads == 0`, `Result().reloads_used == 0`, so the mock in
  `tests/ros/conftest.py` mirrors the wire rather than merely agreeing with the
  file.
- **73 new test functions** (29 FSM, 40 node, 4 record), several parametrised.
- **Config determinism**: `python config/generate_config.py` run twice, second
  run produced no further diff (2026-08-11).
- **The recentre pad probe** (`/tmp/probe_go_home_settle.py`, run 2026-08-11):
  15,409 `/trajectory/status` inter-sample gaps across five 2026-08-10 bags —
  median 0.200 s, p99 ≤ 0.252 s, worst 0.643 s.

New tests, by gate:

| Gate (build spec) | Test |
|---|---|
| a node test for every named stop code | `test_gate_refuses_*` (×6), `test_the_interlude_cannot_be_entered_from_an_off_centre_park`, `test_a_not_settled_run_that_exhausts_the_budget_stops_with_the_budget_code`, `test_the_retry_is_targeted_at_that_code_and_nothing_else` |
| the interlude cannot be entered from an off-centre park | `test_the_interlude_cannot_be_entered_from_an_off_centre_park`, `test_a_stale_commanded_position_reads_as_not_centred` |
| an omitted `on_empty_cup` STOPS | `test_an_omitted_on_empty_cup_stops_the_session` + `test_on_empty_cup_wire_default_is_stop` + the 9-case whitelist parametrisation |
| `stop_on_miss` semantics unchanged | `test_stop_on_miss_semantics_are_unchanged_by_the_reload_policy` |
| NO_RELEASE retry fires ONLY on valid-HELD | `test_no_release_retry_is_gated_on_the_live_cup_read` (tri-state), `test_two_consecutive_no_releases_stop_the_session`, `test_the_no_release_streak_is_CONSECUTIVE_not_cumulative` |
| a live `false` `toss_require_ball_evidence` refuses to arm | `test_gate_refuses_when_ball_evidence_is_disabled` |
| dwell reads provably absent between PREPARE and THROW | `test_dwell_tilt_reads_have_exactly_one_call_site_and_it_is_the_dwell` (structural), `test_a_tight_dwell_degrades_the_read_count_never_the_throw` |

### Open

- **Layer 1.5 is thin at the shipped cadence.** At dwell 6.0 / delay 5.0 the
  quiescent window admits ~1–2 reads, not 8, and the record says so
  (`dwell_tilt_degraded`). The operator's lever is `dwell_time_s ≈ 7.5 s+` on the
  capture goal; the structural fix is the registered SCL3300 async-cache firmware
  follow-on. **D17 — "is arrival repeatability the dominant σ term?" — cannot be
  answered from a 6.0 s-cadence corpus.**
- **`STOPPED_FLOOR_CLEAR_REQUIRED` is unreachable at the shipped defaults**:
  `max_reloads` 3 binds before `floor_pause_every` 5. It is a fence for a
  deliberately long-budget session, and it is tested at the FSM level with an
  explicit threshold rather than the default.
- **The BB relay is best-effort telemetry.** `bb/throw_outcome` is a plain
  `String` on a depth-10 QoS; a dropped message costs a retry (the session stops
  by name instead), never a safety property. If the retry proves load-bearing on
  hardware it wants a typed message and a sequence number.
- **`_run_one_reload_attempt` duplicates `_execute_reload`'s loop shell.** By
  choice (see the Discussion). If a third caller appears, extract it.
- **A harness lesson, not a code one: never start the gate on its RESULT line.**
  A run launched the instant `./run_tests.sh` printed `RESULT: PASS` — and a
  concurrent `pytest --collect-only` against the same tree — produced a run that
  collected **5096 instead of 5187** and passed 4661 instead of 4752, with no
  failure and no error. Two pytest processes share `__pycache__` and the
  assertion-rewrite cache, and `run_tests.sh`'s lock only serialises gates, not
  ad-hoc pytest. Re-run alone it reproduces exactly (4752 / 5187, twice). A
  silently SHORT collection that still says PASS is the worst shape a gate can
  take, so: one pytest at a time, and wait for the process to exit, not for its
  last line.
- **The interlude's reload uses the reload FSM's OWN default throw delay**, not
  the session's `throw_delay_s`. That is deliberate — the session's delay is a
  toss parameter and the reload's is a BB countdown floor — but it means a
  session's cadence and its interlude's are set by different numbers, which is
  worth an operator's eye on the first sitting.
- **Nothing measures how long an interlude actually takes.** The budget
  (`_reload_interlude_budget_s`) is derived from constants; the first sitting
  should compare it against the wall clock before anyone leans on the session
  ceiling.
