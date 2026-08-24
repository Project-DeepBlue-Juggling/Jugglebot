---
title: The three 2026-08-24 change-sets get recorded where the operator reads them, and the poll-cadence floor stops refusing healthy records
type: refactor
date: 2026-08-24
status: resolved
related_plan: toss-selftuning.md
files_changed:
  - logbook/INDEX.md
  - logbook/2026-08-24-hand-sensor-poll-cadence.md
  - tests/hardware/session_cadence_ladder.md
  - tests/hardware/session_anomaly_fixes.md
  - plans/active/toss-selftuning.md
  - ros_ws/docs/ball_possession_contract.md
  - ros_ws/src/jugglebot/jugglebot/toss_trim.py
  - ros_ws/src/jugglebot/jugglebot/ball_possession.py
  - tools/probes/hand_sensor_settle.py
  - tools/probes/README.md
  - tests/motion/test_toss_cal_fit.py
  - tests/ros/test_teensy_bridge_node_hand_acks.py
subsystem:
  - ros
  - can
tags:
  - docs
  - testing
  - sensor
---

# The three 2026-08-24 change-sets get recorded where the operator reads them, and the poll-cadence floor stops refusing healthy records

## What and why

`61aea25` (the 71 ms poll-cadence diagnosis), `2995855` (the can-bridge poller
+ tri-state TX change-set, **unflashed** — committed as FW 15, renumbered
**FW 16** later the same day; see Discussion) and `145484c` (the arrival band re-measured to
0.087 / 0.56) all landed on 2026-08-24 with their INDEX rows and their runbook
cross-recordings **deferred**, because a parallel workstream held the shared
files. That workstream landed at `c0dfa77`. This is the pass that pays the debt,
plus the two code items those entries deferred as Open Questions. The operator
is starting continuous toss testing, so the runbook had to be right tonight.

**Narrative.** Four INDEX rows. The cadence ladder's § 3 preamble now says
**R3 is UNBLOCKED** — both pre-R3 measurements are resolved — and § 3.1 carries
the measured band plus the correction that matters most for a sitting: the
re-measure bought **zero** R0–R3 dwell, because `handoff_margin_s` is a
`max()` already won by the hand's park re-entry (0.1416 s on the binding
aim-armed column, above even the *old* 0.137 s arrival term). 16.6 ms accrues
only with the aim disarmed. The published dwell floor at the default delay is
therefore **5.1416 s**, not 5.137, and it is a *park* number, not an arrival
one — swept at all three sites that published it. § 3.2 gains the mechanism
(pre-FW-14 ring leak, already fixed) without gaining the prior that entry
deliberately refused to write. C-POSSESS-1 § 3.4's reachability table is
re-derived at `ARRIVAL_BAND_MAX_S` 0.56 by calling the shipped
`arrival_boundary_t`: **nothing amputates anywhere on the table**, the R5′ clamp
pin closes 15.1 ms earlier (+0.7849), R6 closes at +0.5600 against a +0.560
ceiling — a pass at *exactly* equality, recorded as reachable and never as
cleared — and the two thresholds fall 1.000 → **0.760** and 0.800 → **0.560** s
while the 200 ms between them (which is `arrival_lead_s`) does not move.

**A THIRD stale citation, found by sweeping rather than by list.** The
poll-cadence entry's census named two documents still describing the 71 ms as a
standing property. There were three: `ball_possession.HandBallSensorSource`'s
own class docstring (`ball_possession.py:~488`) told every future reader that
*"the poll cadence itself measures ~71 ms, not the configured 20 ms, and that gap
has no diagnosis yet"* — a production docstring, in the file that owns the
possession contract, and both clauses of it false. Corrected in place, keeping
the part that still stands: the 232/241/295 ms fall lag is NOT re-measured, and
losing the cadence as its candidate explanation leaves it *without* a mechanism,
which makes the raw/debounced split more load-bearing rather than less. The
lesson is small and cheap: a census that greps for a *number* misses the sites
that spell it in prose — sweep for the claim, not the digits.

**Code.** `hand_sensor_settle.poll_cadence` was the last private copy of the
step rule; it now imports `toss_record.poll_dt_steps_ms` and defines only the
reduction. `TIMING_POLL_DT_MS_MIN` was a live false-refusal — see below.

## The `TIMING_POLL_DT_MS_MIN` incidence probe (measure before you move a threshold)

The floor was `1.0 x JB_BD_CHECK_INTERVAL_MS` = 20.0 exactly, against a strict
`<`. Measured over the population the gate actually reads — every post-FW-14
per-toss record carrying a cadence, mined under `temp/probes/` by
`tools/probes/toss_record_miner.py --sensor-only --jsonl` (four bags,
2026-08-18 → 2026-08-23):

| | value |
|---|---|
| records with a median | **39** |
| min / median / max | 19.999980927 / 20.000934601 / 20.003080368 ms |
| total spread | **3.1 µs** |
| **strictly < 20.0 (refused today)** | **9 (23 %)** |
| < 15.0 | **0** |
| pre-FW-14 contrast (n = 31) | 60 – 87 ms, far above the floor |

So the whole healthy distribution is one spike at the configured interval whose
low mode sits **19.1 ns** under it — a floating-point representation artefact of
the stamp arithmetic, not a sensor polling faster than its own interval — and it
was answering `poll_cadence_below_configured_interval` on nearly a quarter of a
healthy sitting. The floor is now **`0.75 x` configured = 15.0 ms**, and the
fraction comes from the data rather than from taste: the guard exists to catch a
measurand of *impossible provenance*, and the impossible measurand with a name
is the `/hand_telemetry` **100 Hz republish** stamp, which would mine as
**10.0 ms**. 15.0 is the midpoint of the only gap in the corpus — 5 ms clear of
the rate it must still refuse, 5 ms below every healthy reading, and ~1600x the
observed spread. `tests/motion/test_toss_cal_fit.py::test_the_timing_floor_admits_the_healthy_plant_and_still_refuses_the_republish`
pins both halves against the verbatim measured value.

## Discussion — one contradiction, reported and then resolved the same day

§ 3.5 of `ball_possession_contract.md` attributes the 2026-08-23 ladder bag to
**can-bridge FW 15**, while the standing hardware note says FW 15 has never been
flashed and the bridge runs FW 14. The instruction for this pass was to correct
the doc *unless* the bag really says 15 — and it does. The text was therefore
**left exactly as it stands**, and the contradiction is surfaced instead.

The evidence, and why it is decisive rather than suggestive: `bridge_fw_version`
is mined from `/link_status`, which `teensy_bridge_node._bridge_fw_version_str`
renders from the **`BridgeIdentity` frame the board itself sends**, and that
renderer appends an explicit `(SKEW — expected vN)` suffix whenever the reported
value disagrees with `EXPECTED_BRIDGE_FW_VERSION` (the poll-cadence entry quotes
a real one: `10 (SKEW — expected v11)`). The mined rows read a clean
`15 (proto 5)`. A skew-free 15 cannot be produced by a board reporting 14.
Per bag: `2026-08-18_18-42-19` → `14 (proto 5)`, then `2026-08-20_21-51-39`,
`2026-08-21_10-11-42` and `2026-08-23_19-14-54` → `15 (proto 5)`.

The reconciliation — offered as the likely reading, not asserted — is that
**"FW 15" now names two different images**. `3760daa` (2026-08-18) bumped
`FW_VERSION` 14 → 15 for the hand end-stop clamp, and
`logbook/2026-08-18-hand-end-stop-corrected.md:172` records *"✅ BOTH FLASHES ARE
ABOARD — updated 2026-08-21"*. The board has been reporting 15 since somewhere
between 18 Aug 18:42 and 20 Aug 21:51, which matches. What is genuinely
**unflashed** is the *second* half folded into that same version number on
2026-08-24 (the 50 Hz poller + tri-state TX), because the owner chose a fold
over a bump to 16. So the version number no longer identifies the image, and
`bridge_fw_version` cannot distinguish them — exactly the failure mode
`canbridge_config.h` warns about eleven times over ("a healthy link is NOT
evidence this build is aboard"). Nothing here is safe to decide from a
document; the operator owns it.

**RESOLVED the same day — the owner bumped rather than folded.** The reading
offered above was the right one, and the operator confirmed it: the end-stop
image has been aboard since ~2026-08-20 and the board legitimately reports 15.
So `2995855`'s premise ("15 is unflashed, one upload carries both") was false,
and its commit message's *"the live bridge stays on FW 14"* is wrong — pushed and
unamendable, corrected in
`logbook/2026-08-24-poller-cadence-and-tristate-tx.md` § "Why FW 16, and why the
fold was withdrawn" instead. Owner's re-decision (2026-08-24): the poller +
tri-state image becomes **FW 16**, with `EXPECTED_BRIDGE_FW_VERSION` bumped in
the same commit, so `bridge_fw_version` discriminates the two images again and
the advisory `15 (SKEW — expected v16)` row stands until the flash. Nothing in
§ 3.5 of `ball_possession_contract.md` needed changing after all: that bag really
was FW 15, and FW 15 really is the end-stop clamp alone.

## Verification

All 2026-08-24, venv `~/Desktop/PDJ_venv/venv`.

* `python tools/probes/hand_sensor_settle.py --self-check` → **38/38 self-check
  cases pass** (was 34/34; the four new cases pin the two holes the shared step
  rule closes — a transient low stamp the private rule reported as a **571.0 ms**
  poll, and a wall-anchor epoch change it reported as **1.786e12 ms**, both
  confirmed against a local re-implementation of the old rule before the checks
  were written).
* `python -m pytest tests/ros/test_ilc_measurement_probes.py -q -p no:randomly`
  → **30 passed in 26.68 s** — the consuming test the step-rule change was
  deferred over, unchanged.
* `python -m pytest tests/motion/test_toss_cal_fit.py -q -p no:randomly` →
  **62 passed in 3.27 s**.
* `python -m pytest tests/motion/test_toss_trim.py tests/motion/test_toss_record.py
  tests/ros/test_toss_record_miner.py -q -p no:randomly` → **186 passed in
  45.62 s**.
* THE GATE (`./run_tests.sh`, run 2026-08-24) against this change-set's tree
  (before the FW 15→16 renumber landed in the same working tree — that
  change-set carries its own gate run) →
  **PASS — 5759 passed, 3 skipped, 5 warnings in 257.71 s**; serial phase 6197
  deselected in 6.57 s (empty by the 2026-08-01 carve-out, as `run_tests.sh`
  says); `Summary [gate]: parallel 261s (rc=0) | serial 10s (rc=0) | total
  271s`, `RESULT: PASS`. An earlier identical run on the same tree minus the
  `ball_possession.py` docstring correction gave 5759 passed / 258.49 s / PASS.
* The doc surface, run LAST so it covers this entry itself
  (`python -m pytest tests/sim/test_logbook_front_matter.py
  tests/sim/test_logbook_search.py tests/sim/test_plans_index.py -q
  -p no:randomly`, 2026-08-24) → **103 passed in 0.64 s**. Those three files are
  the whole test surface that reads `logbook/` and `plans/`, so this entry and
  its four INDEX rows are covered on this change-set's tree (before the
  FW 15→16 renumber landed in the same working tree — that change-set carries
  its own gate run).

> ⚠ **`./run_tests.sh --full` WEDGED and was killed — it is NOT part of this
> verification, and the pre-sitting `--full` obligation is UNMET.** It ran 35
> minutes without leaving the parallel phase (the 04:00 nightly, same tiers at
> ci-deep, takes **21m 35s** and was GREEN today: 6174/6180). Diagnosed rather
> than assumed: `load average 0.01` with all four xdist workers asleep in
> `futex_wait_queue_me` and **no test file open on any of them**, one worker a
> `<defunct>` zombie with a replacement spawned 7 minutes in — and **swap at
> 3191 MB of 3668 (87 %)** with 1.1 GB of RAM free. Killing it returned 2.5 GB.
> The reading is memory pressure, not a test defect: `-n 4` plus the nightly
> tier's heavier fixtures does not fit on this 8 GB box **while a GUI server, an
> agent session and jtop are resident**, which is exactly what is NOT resident at
> 04:00. Nothing in this change-set plausibly causes it — the default gate
> passes on the same tree, and the tier was green at 04:00 before these edits.
> **Before the sitting, re-run `--full` on a quiet box** (or accept tonight's
> nightly), and consider `-n 2` if it recurs.

## Outcome

R3 is unblocked in the runbook the operator will actually read, the three
2026-08-24 entries are indexed, and the toss sessions about to run can no longer
lose a quarter of their timing records to a floating-point edge. Two Open
Questions on `logbook/2026-08-24-hand-sensor-poll-cadence.md` are closed in
place (the floor and the sibling census); and the third — the FW-15 identity
collision — was resolved the same day by the owner's re-decision to bump the
poller + tri-state image to FW 16 rather than fold it into 15 (see
Discussion).
