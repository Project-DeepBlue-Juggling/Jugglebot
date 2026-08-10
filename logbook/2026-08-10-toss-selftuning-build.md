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
