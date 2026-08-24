---
title: The 71 ms hand-sensor poll cadence was an already-fixed bridge defect, not a property of the sensor path
type: investigation
date: 2026-08-24
status: resolved-with-open-items
related_plan: toss-selftuning.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/toss_record.py
  - tools/probes/hand_poll_cadence_census.py
  - tools/probes/README.md
  - tools/probes/toss_record_miner.py
  - tests/ros/toss_record_fixtures.py
  - tests/ros/test_toss_record_miner.py
  - tests/motion/test_toss_record.py
  - tests/motion/test_toss_cal_fit.py
  - logbook/2026-08-10-toss-selftuning-build.md
subsystem:
  - ros
  - can
tags:
  - testing
  - docs
  - sensor
---

# The 71 ms hand-sensor poll cadence was an already-fixed bridge defect, not a property of the sensor path

## Summary

`logbook/2026-08-10-toss-selftuning-build.md` § Open carried *"the measured 71 ms
poll cadence has no diagnosis — own investigation, can-bridge side"* for two
weeks, and at commit `24c7551` (2026-08-22) **two** documents still repeated the
71 ms as a standing property of this robot: the hardware runbook
(`tests/hardware/session_cadence_ladder.md` § 3.2, headed "⬜ NOT DIAGNOSED") and
`plans/active/toss-selftuning.md`'s open row. It is not one. A census of the
existing bags settles it offline, with no new sittings: the pre-FW-14 FlexCAN_T4
RX-ring `_available` leak inflated the hand-sensor SDO round trip with bridge
uptime, and the bridge's **one-in-flight** poll loop converts round-trip latency
directly into lost cadence. The FW 14 fix landed 2026-08-15
(`logbook/2026-08-14-fw14-ring-leak-fix.md`, validated in
`logbook/2026-08-15-fw14-validated-arc-closed.md`); all thirteen decodable bags
captured since measure the configured 20 ms. The phenomenon is closed.

Five defects found while re-deriving the numbers are fixed here — four in the
measurement path (a cadence statistic that folded backwards steps into the
median; that, once those were merely *dropped*, minted a forward step of the
whole re-anchor distance out of the next sample; that spanned blind gaps; and
that could inherit the bridge's wall-anchor discontinuity — plus a miner that
crashed on `--jsonl` with an absolute `--bag`) and one in the tests (the
100 Hz-republish dedupe, the single behaviour separating publish rate from poll
rate, was never exercised). The census itself is now a committed probe, so the
tables below are re-derivable rather than trusted.

## Symptoms

- `sensor_poll_dt_ms_median` mined **70.998 ms** on the reference bag
  `2026-08-10_16-30-44` against a configured `JB_BD_CHECK_INTERVAL_MS` of 20 —
  a 3.5x gap, spread wide (p5 32 / p95 111) rather than quantised at a different
  fixed rate.
- The gap was durable enough across the 2026-08-12 sittings to look structural,
  and it was written into a test name
  (`test_the_fixture_records_the_measured_poll_cadence_gap`) as a property of the
  plant. Two documents still read that way at `24c7551`: the hardware runbook
  (`tests/hardware/session_cadence_ladder.md` § 3.2) and
  `plans/active/toss-selftuning.md`'s open row. Two others had said it and then
  withdrawn it on their own evidence — see Open Questions — so the framing was
  already half-retracted; what was missing was the mechanism and the closure.

## Diagnosis

### The transfer function: why a slow round trip *is* a slow cadence

`gpio_poll_step()` runs on `task_homing` at `HOMING_RATE_HZ = 100`
(`Teensy_code_canbridge/canbridge_config.h:106`), and it is a **strictly
one-in-flight** state machine: while `s_phase == PPhase::AWAIT` it does nothing
but test for the reply and return; only from IDLE, and only once the poll
interval (20 ms) has elapsed, does it send the next request. (Cited by symbol
rather than by line: at commit `24c7551` (2026-08-22) that is
`gpio_poll.cpp:296-309`, but a parallel workstream is rewriting exactly this
pacing as FW 16 — see Open Questions.) Because the reply is *observed* on a tick
and the next attempt is taken on a later tick, the achieved cadence quantises to
the 10 ms task grid:

    C = 10 · max(2, ceil(RTT / 10) + 1)   ms

`ball_held_stamp` is the reply's arrival wall stamp (`apply_good_reply`'s
`p.wall_us`, `gpio_poll.cpp:208` at that same commit), so consecutive stamps are
separated by exactly that C. The measurement inverts cleanly:

| measured C | implied RTT | regime |
|---|---|---|
| 20 ms | ≤ 10 ms | healthy — the configured interval is the binding constraint |
| 70 ms | 50–60 ms | the round trip, not the interval, is setting the rate |

So a 20 ms *configuration* and a 71 ms *measurement* were never in
contradiction. There is no missing poll and no re-rating: the poller is waiting.

### The census: the cadence tracks bridge uptime, and the FW 14 fix removed it

The 2026-07-18 arc (closed 2026-08-15;
`logbook/2026-08-14-s3-conviction-ring-leak-measured.md`) convicted the
FlexCAN_T4 `_available` one-way leak: the RX ring becomes an uptime-ratcheting
delay line, ~40 ms/h, capped at 256 slots ≈ 135 ms. Everything that comes back
over the jugglebot bus is delayed by it — the SDO reply included. Three
independent cuts of the bag corpus:

1. **A same-day reboot A/B.** `2026-08-12_14-52-00` measures **p50 72.001 ms**
   (n = 1674 steps) at a bridge uptime of **63.06 h**; `2026-08-12_14-55-18`,
   three minutes later across a can-bridge Teensy reboot, measures **p50
   20.0 ms** (n = 3636, p95 36.0, max 37.0) at **0.00 h**. The reboot is
   witnessed by the covariate rather than assumed. (2026-08-24, `python
   tools/probes/hand_poll_cadence_census.py --bag 2026-08-12_14-52-00 --bag
   2026-08-12_14-55-18`, result as quoted.)

   **One confound, named because re-running the census surfaced it.** That reboot
   also brought the S1 Arm C flash into service: `bridge_fw_version` reads
   `10 (SKEW — expected v11)` on the first bag and `11` on the second. So the
   pair is not a clean uptime-only A/B, and an earlier draft of this entry said
   "nothing else changed", which was wrong. What it still cannot be is the *ring*
   fix — that is FW 14, three days later — and the uptime reading rests on cut 3's
   corpus-wide correlation rather than on this pair alone.
2. **Every post-FW-14 bag that decodes runs at the configured rate.** 13 of the
   18 date-named bags from 2026-08-15 on carry `/hand_telemetry` and decode —
   four have no hand topic, and `2026-08-15_00-42-12` is truncated. **All 13
   measure p50 20.0 / p95 30.0 ms**, at bridge uptimes from 0.0 h to 116.7 h —
   the exact soak range that used to produce the drift. (The session's own first
   census counted 16 bags over a slightly wider date net; the verdict is the same
   either way.) (2026-08-24, `python tools/probes/hand_poll_cadence_census.py
   --all --match '2026-08-*' --json` — 52 bags surveyed, 47 measured, 5
   unreadable; table in Verification. The run exits **1** because those five
   would not read, which is the probe's documented per-bag reporting and not a
   failed census.)
3. **Pre-FW-14, the per-bag median tracks bridge uptime.** Over the 34
   pre-FW-14 bags of the same corpus run, per-bag `p50_ms` against
   `uptime_h_open` gives **Spearman ρ = +0.803** (Pearson r = +0.479 on the same
   pairs — the gap is what a cadence quantised onto 20 / 40 / 50 / 70 / 100 ms
   rungs does to a *linear* coefficient, which is why the rank statistic is the
   one quoted). The reduction is one line over the `--json` report cut 2 wrote:
   take `[(b['uptime_h_open'], b['p50_ms']) for b in report['bags'] if
   b.get('p50_ms') is not None and b['bag'] < '2026-08-15']` and correlate the
   two columns' ranks (2026-08-24).

   **A number moved here, and it is worth saying which.** The session's own
   uncommitted analysis reported ρ = +0.839. Re-derived from this probe's rows it
   is **+0.803** — same sign, same order of magnitude, same verdict, but the
   earlier figure came from a slightly different bag set and does not reproduce
   exactly, so +0.803 is what this entry stands behind.

   The **`leak_jb` half of this cut is deliberately not re-derivable here**: the
   census reads no `/ring_diag` (Fix item 5). The ring's uptime-ratcheting
   occupancy is convicted on its own evidence in
   `logbook/2026-08-14-s3-conviction-ring-leak-measured.md` — `leak_jb` **247–248
   of 256 slots** against `avail_reported_jb` 0 at 4.03 h, high-water 249 — and
   this entry does not re-measure it. Cuts 1 and 2 above were re-run
   independently for this entry.

Uptime → ring delay → SDO RTT → poll cadence. Each link is measured, and the
last one is arithmetic.

## Discussion

**The framing pivot.** This was carried as *"an open can-bridge investigation"* —
a hardware question needing a sitting, a scope, an `/investigate`. It was
neither: the answer was already in the corpus, and it was already fixed. Two
things had hidden that. First, the 2026-08-10 phase deliberately (and correctly)
refused to diagnose the cadence, making it a MEASURED field so nobody could
assume it away — but "not diagnosed here" then aged into "undiagnosed", and the
number aged into a property. Second, the FW 14 fix closed the ring-leak arc on
its own evidence (e2e latency, `leak_jb`, clamp duty) without anyone re-checking
which *other* open items that arc's mechanism silently answered. The cost of the
miss was small only because nobody spent a sitting on it; the general lesson is
that closing an arc should include a sweep of open items that name the same
plant.

**Why the fix is not a wider guard.** The tempting move on finding a stale number
spread across several documents is to add a rule — a test that pins the cadence to 20 ms, or a
health gate that flags an elevated median. Both are wrong here. Cadence is a
*measurement of the session in front of you*; 71 ms was a true measurement and
so is 20 ms. What failed was not the measurement but its **status**: a
one-sitting number was cited as a standing property. So the fix is to make every
citation say which bag it came from and when — and to make the instrument that
produces it harder to fool.

**The wording decision, and why it must not be quietly reverted.** It is very
tempting to write *"71 ms is the signature of the RX-ring leak — if you ever see
an elevated cadence again, look there first."* That sentence is deliberately
**not** anywhere in this change-set, on the owner's direction, and it should not
be added later. The reason is that it converts one confirmed mechanism into a
prior for every future observation of the same symptom, and the symptom is
generic: `C = 10·max(2, ceil(RTT/10)+1)` is elevated by *anything* that slows the
round trip — bus load, an ODrive servicing slowly, a task-priority change, a
future TX-deferral path, or a defect nobody has met yet. A future reader who
inherits "it's the ring leak" will check `leak_jb`, find it zero, and be *less*
likely to look further than if they had inherited nothing at all. What the
documents now say instead is bounded and true: an elevated measured cadence means
**investigate; the mechanism is undetermined until measured**. The transfer
function above is the useful inheritance, because it tells the next reader what
to measure (the RTT) rather than what to believe.

**Why the measurement fixes are in the same change-set.** Re-deriving the census
is what exposed them, and each one would have corrupted the very numbers this
entry cites — a folded-in backwards step, a step spanning a blind gap, or the
wall-epoch discontinuity would each have moved a median or a max. Landing the
narrative without the instrument fixes would leave the next census reproducing
the numbers only by luck. (The anchor guard is the clearest case: on
`2026-08-12_14-55-18` the +1.79e12 ms anchor step is excluded today *only*
because that bag's thirteen leading pre-anchor samples happen to carry
`ball_held_valid` false — see Verification.)

## Fix

1. **`ros_ws/src/jugglebot/jugglebot/toss_record.py:871-980`** — one definition
   of a poll step. New `poll_dt_steps_ms()` returns a `PollSteps` NamedTuple
   (`steps_ms`, `n_backwards`, `n_domain_breaks`) and `poll_dt_ms_median()` is
   the median of it. Three rules, each closing a way a clock artefact became a
   cadence: a **backwards** step is counted and **resets the tracker** (folded in
   as a negative it dragged the median toward zero — the sibling census in
   `tools/probes/hand_sensor_settle.poll_cadence` had always dropped-and-counted
   it — but dropping the negative is only half of it: carrying `prev` onto the
   LOW stamp makes the very next sample a step of the whole re-anchor distance in
   the POSITIVE direction, so a 0.5 s re-anchor mints a 500 ms "poll". `prev`
   resets, and the one interval spanning the re-anchor is lost rather than
   invented); `prev` **resets on any invalid sample** too (the bridge may poll
   many times while `ball_held_valid` is false, so the difference across the gap
   is not one interval); and a step that crosses the bridge's **wall anchor** is
   refused and counted, using `STAMP_WALL_TOL_S` — the same discriminator that
   sets the record's `ball_held_stamp_wall_anchored` flag, now one constant
   shared with the miner instead of two copies of `60.0`. Module docstring
   (`:75-87`) and the field comment reframed: the 71 ms is a measurement of one
   sitting.
2. **`tools/probes/toss_record_miner.py`** — `--jsonl` with an absolute `--bag`
   (which `~` expansion silently produces) spliced the whole path into the output
   filename and died `FileNotFoundError` *after* the full mine and report, losing
   the corpus. New `bag_label()` reduces `--bag` to the bag's own basename at the
   one place the bag is opened, so the report header, the jsonl name and the
   fixture's `REFERENCE_BAG` agree for either spelling; the *path* still keeps
   whatever was given.
3. **`tests/ros/toss_record_fixtures.py` + `tests/ros/test_toss_record_miner.py`**
   — `POLL_DT_MS_MEDIAN = 70.998` **stays** (it is the true mined median of the
   reference bag and pins the miner against it), but the test is now a pure
   miner-regression pin,
   `test_the_fixture_pins_the_reference_bags_mined_poll_cadence`, asserting the
   value rather than a gap against the configured interval. The fixture comment
   is regenerated-identical in the emitter, so `--emit-fixture` stays idempotent.
   Same discipline applied to
   `test_the_timing_gate_admits_the_cadence_the_reference_bag_measured`
   (`tests/motion/test_toss_cal_fit.py:648`, renamed from
   `..._this_plant_actually_runs_at`): the re-derived band gate stands, but its
   justification is now "the cadence varies between sittings", not "the plant
   runs slow".
4. **`tests/motion/test_toss_record.py:535-698`** — the dedupe is finally
   exercised. Every synthetic stream in the file advanced the stamp on *every*
   sample, so the consecutive-distinct-stamp rule — the one behaviour separating
   the 100 Hz republish from the true poll cadence — was untested. New
   `poll_stream()` republishes each poll N times (integer division, so the repeat
   count cannot drift), and five tests drive it: the poll rate is reported not
   the publish rate (parametrised over 20 / 50 / 71 ms, all measured on this
   robot); a backwards step is counted and reseeds the tracker; an invalid span
   resets it; the wall-anchor step is refused (stamps verbatim from
   `2026-08-12_14-55-18`); and a never-advancing stream reports `None` rather
   than a 0 ms poll that would sail through the admission floor. The
   backwards-step case pins the **max**, not just the sign — `min(...) > 0` is
   satisfied by the very 500 ms fake forward step the old tracker minted, so the
   assertion that catches it is `max(steps_ms) == 20 ms`, with the step count at
   38 (39 unbroken, minus the one interval the re-anchor costs).
5. **`tools/probes/hand_poll_cadence_census.py`** (new, + its
   `tools/probes/README.md` row) — the census this entry's tables are made of,
   committed rather than left in a scratch directory. Every number below is a
   run of it. It defines only the **reduction** (percentiles, the per-bag row,
   the corpus split): the step rule is imported from
   `toss_record.poll_dt_steps_ms` and the reader from
   `toss_record_miner.read_bag(..., sensor_only=True)`, so it cannot drift from
   the `sensor_poll_dt_ms_median` the miner writes onto a record. `--bag` takes
   a name or a path, `--all --match '<glob>'` sweeps a corpus, `--json` writes
   to `temp/probes/`, and `--self-check` is bag-free (8 groups). Two scope
   decisions worth naming: it reads no `/ring_diag`, so ring occupancy is out
   (see Diagnosis cut 3); and a bag that will not decode is **reported and
   skipped**, not fatal — a truncated final chunk surfaces as `struct.error`
   from inside the CDR reader, and "one of these bags is truncated" is a census
   result, not a reason to lose the other fifty-one.

## Verification

All 2026-08-24, on the venv at `~/Desktop/PDJ_venv/venv`.

**The median of a clean-era bag does not move.** `python
tools/probes/hand_poll_cadence_census.py --bag 2026-08-10_16-30-44` → **n = 9845,
p5 32.002, p50 70.998, p95 110.997, max 155.001 ms, n_backwards 0,
n_domain_breaks 0**, over 70,666 samples at `valid_frac` 1.0. (The console table
rounds to 1 dp; the three-decimal figures quoted here are the `--json` report's.)
Identical to the same census run against the pre-fix function: `poll_dt_ms_median`
returns 70.99795341491699 before and after, so
`tests/ros/toss_record_fixtures.py`'s 70.998 is unchanged.

**The anchor step.** On `2026-08-12_14-55-18` the step is real, and its size is
arithmetic on the two stamps either side: (1786510522.065801 − 17.251004) s =
**1.786511e+12 ms**. It is excluded today by *luck*, not by rule — the bag's
**thirteen** leading samples, carrying six distinct boot-relative stamps, all
have `ball_held_valid` false, and the pre-change-set function skipped invalid
samples outright. As bagged the max is **37.0 ms** either way (2026-08-24,
`python tools/probes/hand_poll_cadence_census.py --bag 2026-08-12_14-55-18` →
**n 3636, p5 14.0, p50 20.0, p95 36.0, max 37.0 ms, back 0, dom 0**).

Driving the **reachable** case — the same stream with `valid` forced true, which
is exactly a record with `ball_held_stamp_wall_anchored == false` — the old logic
returns a max of **1.786511e+12 ms** and the new one **37.000 ms with
`n_domain_breaks` 1**. Over the committed modules, so it needs no scratch script
(2026-08-24):

```python
import sys; sys.path[:0] = ['tools/probes', 'ros_ws/src/jugglebot']
import toss_record_miner as m
from jugglebot.toss_record import poll_dt_steps_ms as P
d = m.read_bag('/home/jetson/Desktop/rosbags/2026-08-12_14-55-18', sensor_only=True)
s = P([x._replace(valid=True) for x in d.hand])
print(len(s.steps_ms), max(s.steps_ms), s.n_domain_breaks)   # -> 3641 37.0 1
```

The same case, driven on those verbatim stamps, is *asserted* by
`tests/motion/test_toss_record.py::test_the_wall_anchor_discontinuity_is_not_a_poll_interval`
and by the census probe's `--self-check` group 5 — so it is pinned, not measured
once.

**The miner crash.** Before:
`python tools/probes/toss_record_miner.py --bag ~/Desktop/rosbags/2026-08-12_14-55-18
--sensor-only --jsonl` → `FileNotFoundError: .../temp/probes/toss_records_/home/jetson/Desktop/rosbags/2026-08-12_14-55-18_20260824_112139.jsonl`
after the whole report printed. After: `wrote
/home/jetson/Desktop/Jugglebot/temp/probes/toss_records_2026-08-12_14-55-18_20260824_112613.jsonl`.

**The post-FW-14 census** (`python tools/probes/hand_poll_cadence_census.py
--all --match '2026-08-*' --json`), p50 / p95 in ms against the bridge uptime the
bag opened at:

| bag | steps | p50 | p95 | max | uptime at open |
|---|---|---|---|---|---|
| 2026-08-15_00-44-59 | 3440 | 20.0 | 30.0 | 31.0 | 5.8 h |
| 2026-08-15_00-46-37 | 795 | 20.0 | 30.0 | 30.0 | 5.8 h |
| 2026-08-15_00-47-44 | 2303 | 20.0 | 30.0 | 32.0 | 5.8 h |
| 2026-08-15_10-04-52 | 303 | 20.0 | 30.0 | 31.0 | 15.1 h |
| 2026-08-15_10-06-14 | 930 | 20.0 | 30.0 | 30.0 | 15.1 h |
| 2026-08-15_10-08-06 | 3323 | 20.0 | 30.0 | 31.0 | 15.2 h |
| 2026-08-15_10-13-12 | 252 | 20.0 | 30.0 | 30.0 | 15.2 h |
| 2026-08-18_00-12-27 | 2606 | 20.0 | 30.0 | 473.6 | 77.2 h |
| 2026-08-18_18-42-19 | 21453 | 20.0 | 30.0 | 50.0 | 95.7 h |
| 2026-08-18_22-32-42 | 784 | 20.0 | 30.0 | 31.0 | 0.0 h |
| 2026-08-20_21-51-39 | 7172 | 20.0 | 30.0 | 50.0 | 47.3 h |
| 2026-08-21_10-11-42 | 13414 | 20.0 | 30.0 | 31.0 | 59.7 h |
| 2026-08-23_19-14-54 | 11462 | 20.0 | 30.0 | 160.0 | 116.7 h |

**13/13 at p50 20.0 / p95 30.0**, `n_backwards` and `n_domain_breaks` zero
throughout. The isolated `max` outliers (473.6 / 160 / 50 ms) are single gaps in
a stream that medians at the configured interval either side of them — a stalled
round trip, not a cadence — and they are exactly why a p50 rather than a max is
the field the record carries. The same run's 34 pre-FW-14 rows are cut 3's
population; one of them (`2026-08-09_16-32-37`) carries `n_domain_breaks` 1, so
the anchor guard fires on real corpus data and not only under test.

**Probe self-checks**, all 2026-08-24 and all after the fixes above: `python
tools/probes/toss_record_miner.py --self-check` → **63/63 pass**; `python
tools/probes/hand_sensor_settle.py --self-check` → **34/34 pass**; `python
tools/probes/hand_poll_cadence_census.py --self-check` → **PASS, 8 groups, 0
failures**.

**Scoped suites.** `pytest tests/motion/ -q` → **1937 passed, 3 skipped in
319.33 s**. `pytest tests/ros/ -q` → 2176 passed, 1 failed in 305.40 s; the
failure is `test_teensy_bridge_node_coldstart.py::test_a_skew_does_not_gate_the_hand_dispatch_path`,
untouched by this change-set and green scoped (`pytest
tests/ros/test_teensy_bridge_node_coldstart.py -q` → **26 passed in 7.33 s**) —
a load flake under a concurrently running suite on the other worktree.

**The gate.** `./run_tests.sh` (2026-08-24) → parallel phase **5756 passed,
3 skipped, 4 warnings in 259.70 s**; serial phase 6194 deselected (every
`serial`-marked test is also `nightly`, so the gate's serial phase is empty by
design); `Summary [gate]: parallel 262s (rc=0) | serial 10s (rc=0) | total 272s`,
**RESULT: PASS**. The `tests/ros` flake above did not reappear. `--full` was not
run: nothing under `controller/` or `sim/` is touched, so the default gate covers
this change-set.

**Re-gated after the audit-fix pass** — the backwards-step reset, the census
probe, the narrowed cross-recording claims and the restated citations all landed
after the run above, so it was re-run against the final tree. Scoped first
(2026-08-24, `pytest tests/motion/test_toss_record.py
tests/ros/test_toss_record_miner.py -q -p no:randomly`) → **103 passed in
31.92 s**; then the gate (2026-08-24, `./run_tests.sh`) → parallel **5758 passed,
3 skipped, 5 warnings in 255.75 s**, serial 6196 deselected, `Summary [gate]:
parallel 258s (rc=0) | serial 10s (rc=0) | total 268s`, **RESULT: PASS**
(`GATE_RC=0`). The tree at that moment also carried a parallel workstream's
unstaged edits to `toss_record.py`, `test_toss_record.py` and
`toss_record_miner.py` (the `ARRIVAL_BAND_MAX_S` re-measure), which is why the
parallel count is 5758 rather than 5756 — the gate covers the union, and passed
on it. The `--emit-fixture` idempotence check was re-run against that same tree:
rendering the emitter template from the fixture's own constants and diffing
against `tests/ros/toss_record_fixtures.py` gives **0 diff lines**.

One shared-tree artefact worth recording, since a future reader will meet it
again: an intermediate gate run came back FAIL with all 17
`tests/firmware/test_native_firmware.py` cases in ERROR. It was not this
change-set — no C++ or firmware test is touched here, an earlier run of the
identical Python tree passed (5749/5749), the same firmware case passed scoped
immediately after (`pytest
tests/firmware/test_native_firmware.py::test_native_gpio_poll_binary_passes -q`
→ **1 passed in 190.76 s**), and the next full gate passed. A parallel
workstream was rewriting thirteen `Teensy_code_canbridge/` sources at that
moment; the native-firmware harness compiles those sources, so a build taken
mid-edit errors out. On this box the gate is not hermetic against a concurrent
session editing firmware — re-run before believing a firmware ERROR block.

## Outcome

The 2026-08-10 Open item *"the measured 71 ms poll cadence has no diagnosis"* is
closed, with an addendum on that entry pointing here. No sitting was spent: the
corpus already held the answer, and the FW 14 fix had already shipped it. The
cadence field keeps doing its job — it is measured per record, and on the current
plant it measures 20 ms.

## Open Questions

Deliberately deferred, each with what would settle it:

- ~~**The `TIMING_POLL_DT_MS_MIN` strict-`<` edge.** `toss_trim.py:519` sets the
  floor to `JB_BD_CHECK_INTERVAL_MS` = 20.0 exactly, and `:1065` refuses on
  `dt < TIMING_POLL_DT_MS_MIN`. A healthy plant now medians *at* the floor, and
  quantisation puts real medians at 19.99x — those would be refused
  `poll_cadence_below_configured_interval`. Not changed here: the right fix
  depends on the incidence, which needs a probe over the post-FW-14 corpus
  (every bag in the census above medians at 20.0, but the census reports the
  whole bag, not the per-record decisive window the gate actually reads).~~
  **CLOSED 2026-08-24, and the concern was right**: the incidence probe this
  item asked for was run over the per-record population the gate actually reads
  — **9 of 39** post-FW-14 records (23 %) sat 19.1 ns under 20.0 and were being
  refused. The floor is now `0.75 x` configured = **15.0 ms**, sized as the
  midpoint between the measured spike (20.000 ± 0.003 ms) and the 10 ms
  `/hand_telemetry` republish rate that is the one measurand of impossible
  provenance this guard exists to catch.
  `logbook/2026-08-24-cadence-crossrecording-closeout.md`.
- **The 42-vs-50 Hz residual — being closed elsewhere, concurrently.** The
  transfer function predicts a floor of 20 ms (50 Hz) and the corpus agrees at
  p50, but p95 is 30 ms, so the *mean* cadence is ~24 ms, ~42 Hz. While this
  entry was being written a parallel workstream landed the mechanism in
  `gpio_poll.cpp` (uncommitted at the time of writing, FW 16, dated 2026-08-24):
  the old pacing restarted the interval from `now` at each send, so with the
  poll interval an exact multiple of the task period the next send needed this
  wake's jitter to beat the one two ticks earlier — a coin flip that costs a
  whole task period when lost, and re-anchors the grid on the late tick when it
  does. Their fix is an absolute schedule plus a half-tick early-fire band.
  Noted here rather than merged in: it is their change to verify, it does not
  touch this entry's diagnosis (a 50-60 ms round trip forces a >= 70 ms cadence
  under either pacing), and the two arrived at the same numbers independently —
  which is the useful cross-check.
- **The `send_on` `write() == -1` deferral misread**, and **the `defer_jb = 64`
  census anomaly** — both noticed while reading the bridge TX path for this
  entry, both outside the sensor path, neither reproduced. Left as observations,
  not claims.
- **Cross-recordings held back — two documents, not four.** At `24c7551` only
  two still describe the 71 ms as a standing property: the runbook
  (`tests/hardware/session_cadence_ladder.md` § 3.2, headed "⬜ NOT DIAGNOSED")
  and `plans/active/toss-selftuning.md`'s open row. Both are under active
  modification by a parallel workstream, so they were not touched here; updating
  them is a one-paragraph follow-up once that work lands. **DONE 2026-08-24**:
  both now carry the mechanism and the closure, and the runbook's § 3 preamble
  says R3 is unblocked — `logbook/2026-08-24-cadence-crossrecording-closeout.md`.
  (The runbook heading that item quotes had already been flipped to
  "✅ MEASURED AWAY 2026-08-23" by the parallel workstream before this pass
  ran; what was missing was the mechanism, not the status.) The other two need **no
  edit**, and nothing here should be read as claiming otherwise:
  `plans/active/critical-point-ilc.md` retracted the framing itself on 2026-08-12
  off the Phase-0d eleven-bag census — `:161-163` *"the sensor poll cadence is
  session-dependent (per-bag p50 spans 20–76 ms; the reference bag's 71 ms is not
  a standing property — read `sensor_poll_dt_ms_median` per record)"* and
  `:538-541` *"Cadence correction to this plan's own text: the poll's p50 is NOT
  a standing 71 ms"* — and `tools/probes/README.md:101` records that same census
  as having *"refuted them as a standing plant property"*. Both already tell a
  reader to read the field per record; what they lacked, and what this entry
  supplies, is the mechanism and the closure.
- **The sibling census in `tools/probes/hand_sensor_settle.poll_cadence`**
  (`:135-158`) still derives its own steps — and it is now the *only* copy that
  does, since `hand_poll_cadence_census.py` imports the shared rule. It already
  drops-and-counts backwards steps and its `max_ms` is unaffected on every bag
  measured here, but it carries the same re-anchor hole this change-set closed
  (`prev` moves onto the LOW stamp) and it would inherit the anchor
  discontinuity in the reachable case. Adopting `toss_record.poll_dt_steps_ms`
  would close both; it was left alone to keep this change-set to the measurement
  path the record actually carries, and because its own consuming test
  (`tests/ros/test_ilc_measurement_probes.py`) pins numbers that a step-rule
  change would have to be re-derived against.
  **CLOSED 2026-08-24**: `poll_cadence` now imports `poll_dt_steps_ms` and
  defines only the reduction. The re-derivation the deferral worried about cost
  nothing — `tests/ros/test_ilc_measurement_probes.py` passes unchanged (30/30)
  and the probe's `--self-check` went 34/34 → **38/38**, the four new cases
  pinning exactly the two holes: a transient low stamp that the private rule
  reported as a **571 ms** poll, and a wall-anchor epoch change it reported as a
  **1.786e12 ms** one. `logbook/2026-08-24-cadence-crossrecording-closeout.md`.
