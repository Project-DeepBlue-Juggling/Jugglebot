---
title: "The toss loops stop sleeping a tick and start keeping a schedule"
type: optimization
date: 2026-08-27
status: resolved
phase: "toss-pipelined-preamble — Phase B5"
related_plan: toss-pipelined-preamble.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py
  - ros_ws/src/jugglebot/jugglebot/toss_sequencer.py
  - tests/ros/test_toss_coordinator.py
  - tests/ros/test_toss_continuous_node.py
subsystem:
  - ros
tags:
  - cadence
  - performance
  - toss
  - timing
---

# The toss loops stop sleeping a tick and start keeping a schedule

## Summary

Phase B5 lever 1 of `plans/active/toss-pipelined-preamble.md`. Both toss loops —
`_run_toss_cycle` and the session loop in `_execute_toss_continuous`, which is
what actually drives `_tick_toss_pipeline` — ended in a fixed
`time.sleep(_TICK_S)`. An iteration therefore cost `work + 0.020 + scheduler
overshoot`, and the loop **period was an output**. Every budget in this stack is
denominated in `NODE_LOOP_PERIOD_S` as an *input*: `pre_dispatch_budget_s`
charges 4, `commit_budget_s` charges 1, `SAFE_ABORT_LADDER_S` charges 4,
`DEFAULT_SESSION_MISS_CLEANUP_S` charges 2 of the **session** loop's own poll.

They now pace to an absolute grid: `next_due += NODE_LOOP_PERIOD_S; sleep(next_due
− now − slop)`, with a `_PACE_SLOP_S = 0.002` early-fire band and a re-anchor
whenever the loop falls more than one period behind.

Against the B0/P1 corpus (73 chained cycles), re-scored as `max(PERIOD, work)`:
worst-per-cycle period **p50 0.0447 → 0.0400, p90 0.0519 → 0.0400, max 0.0626 →
0.0418**, and the cycles whose worst pre-dispatch iteration exceeds the charge go
**53/73 → 2/73**. `NODE_LOOP_PERIOD_S` is untouched at 0.040 — the re-cut is
B6's, off a post-B3/B4 sitting that does not exist yet.

Levers 2 and 3 were measured, not written: lever 2 de-scoped on B0's numbers
(and on a stale premise, below), lever 3 verified clean.

## Discussion

### Why the paced period is `NODE_LOOP_PERIOD_S` (0.040) and not `_TICK_S` (0.020)

Three arguments, in increasing order of how load-bearing they are.

**1. A target below the typical work is not a target.** The pacer sleeps `period
− work`. Chained `loop_work_max_pre_s` is p50 0.0237 / p90 0.0307 (B0/P1), so
pacing at 0.020 leaves the sleep degenerate on **49 of 73 cycles** — the period
goes straight back to being `work`, and it degenerates on exactly the loaded
ticks that need the yield most. This is the failure the census entry pre-named
when it refused to choose a TARGET from 28 samples
(`2026-08-26-toss-loop-period-census.md` § "Why measure first"). At 0.040 the
same re-score projects p50/p90 0.0400, max 0.0418.

**2. The charge and the period become the same number.** `commit_budget_s` charges
one loop period for the polled COMMIT tick, and `pre_dispatch_budget_s` charges
four for the serial ladder. Today those charges are wrong on 53/73 cycles and
wrong by up to 22.6 ms each; four ticks charged at 0.160 could really cost 0.250,
which is precisely how the two `ABORTED_CANT_MAKE_RELEASE` cycles of
`2026-08-26_14-25-16` happened. Pacing does not make the charge generous — it
makes it *true*.

**3. It makes the machine match the model that certifies it.** This is the one
that settled it. `tools/probes/cadence_rung_check.py` — the probe every published
rung's accept gate is checked against, and the probe B0/P3 used to red the
pipelined ladder — drives the real FSM on `now = t0 + tick * NODE_LOOP_PERIOD_S`
(`:326`). It has **always** modelled a perfectly paced 0.040 grid. The machine
did not run one. Pacing closes that gap; the alternative was to keep widening the
probe's fiction, and a probe whose clock is not the machine's clock is a probe
that certifies rungs the machine cannot fly.

**The margin, named honestly.** `loop_work_max_pre_s` maxes at **0.0418** — the
worst measured pre-dispatch iteration does more work than a 0.040 period can
hold, so on it the pacer degenerates by 1.8 ms. Two cycles in 73. That is not
hidden: `loop_n_over_pre` stops being dominated by "sleep + work happened to
exceed the charge" and becomes a work-overrun signal. It does **not** become a
*pure* one — a paced period is `PERIOD + (wake_k − wake_{k−1})`, so a wake-latency
spike still trips the threshold, by a millisecond or two where the unpaced loop
tripped it by twenty. B6's acceptance ("`loop_n_over_pre` is 0 on every successful
cycle") was written against the unpaced loop and needs re-reading in that light.

**What it costs, stated.** The session loop's between-cycle poll coarsens from
~0.023 s to ~0.040 s, so a *serial* cycle's START is detected a median ~7.5 ms
later — `~7.5 ms = (0.040 − _PACE_SLOP_S)/2 − 0.023/2`, the shift in the mean
wait of a uniformly-arriving event between the two poll granularities — and the
achieved period grows by that much. It does not accumulate —
`_next_cycle_at` is re-derived from the previous cycle's **scheduled** landing
every cycle — and it is the same quantity `DEFAULT_SESSION_MISS_CLEANUP_S` has
charged at `2 × NODE_LOOP_PERIOD_S` since D3, so the poll is finally costing what
the budget already pays for. Cancel-between-cycles latency moves by the same
amount, against a 0.25 s `TOSS_CANCEL_CUTOFF_S` and a documented deferral of one
whole cycle.

### The early-fire band: the poller's transposition has a trap in it

The plan says "half-tick early-fire band" and points at the FW 16 poller fix
(`2026-08-24-poller-cadence-and-tristate-tx.md` § Feature 1). **Read literally
against this loop, "half a tick" would destroy the pacing.** In the poller the
band is `TICK_PERIOD_US / 2` — half the **wake granularity** (the 100 Hz
`task_homing` tick — `TICK_PERIOD_US = 1e6 / HOMING_RATE_HZ`,
`HOMING_RATE_HZ = 100`), not half the 20 ms poll interval it paces. Half of `_PACE_PERIOD_S` here
is 0.020 s, and firing 20 ms early on a 40 ms grid hands the period straight back
to `work`. The band's job is to absorb the wake granularity, so it is sized to
the wake granularity: **0.002 s**.

And the band is not cosmetic here, for a reason specific to this stack.
`time.sleep` returns **late**. Sleeping the full `due − now` lands every iteration
a wake latency past its grid instant, and four of those come straight out of
`pre_dispatch_budget_s`. At a `throw_delay_s` sitting exactly on the accept floor
the runtime guard's comparison is an *equality* — that is what
`FLOOR_REPRESENTATION_SLACK_S` (1e-6) exists for — so a systematic +1.5 ms would
abort `ABORTED_CANT_MAKE_RELEASE` on every at-the-floor rung. **Sleeping short by
the band lets the scheduler's own latency carry the wake *to* the due instant
instead of past it.** Late is the failure direction; early is free margin. That
is the poller's "rounding the due instant to the nearest tick", transposed
correctly.

Sized from measurement at both ends: idle `time.sleep` overshoot on this Jetson
is p50 0.1 ms / p99 0.1 ms / max 0.9 ms (400 samples at 0.014 s and 0.020 s
targets, `/tmp/probe_sleep_granularity.py`, 2026-08-27); under the live executor
`loop_sleep_max_pre_s − 0.020` is p50 1.5 ms / max 6.8 ms (B0/P1). 0.002 s covers
the idle maximum and the live median. The 6.8 ms tail is deliberately **not** a
sizing target — a band that large would systematically shorten the period by
17 %, and an outlier wake is what the runtime guard and `loop_n_over_pre` are for.
Ceiling: the band must stay under `PERIOD / 2`, the poller's own bound (the
largest that cannot pull an iteration onto its predecessor's slot). 0.002 is a
tenth of that ceiling (a twentieth of the period).

### Recovery: re-anchor, not catch-up — and where the line is

Two behaviours, and the split matters:

* **Mild overrun keeps the grid.** An iteration that runs past its due instant
  does not sleep, and the next one is short by exactly the overrun. This is
  deliberate: `pre_dispatch_budget_s` charges the **sum** of four ticks, and a
  preserved grid makes that sum exact even when one member overran. The implied
  catch-up is bounded to one period by construction.
* **More than one period behind re-anchors on `now`.** Same call the poller took
  ("a sample not taken has no backlog"), plus two reasons that are ours:
  replaying missed slots would fire several iterations at nearly the same `now`
  doing no useful FSM work — every guard here is level-triggered
  (`now >= commit_at`) — and it would charge `LoopPeriodCensus` several near-zero
  periods on exactly the cycle that just overran, diluting p50 and
  `loop_n_over_pre` precisely where the instrument matters most. It would also
  spend the burst starving the executor of a yield immediately after a heavy tick.

The trigger is measured, not hypothetical: the one B0/P1 cycle that commanded a
positioning move spent **0.3022 s** in a single iteration (0.2774 of it inside a
blocking `go_to_pose`) — 7.5 periods of backlog. And the session loop's two
`continue` statements (the reload interlude, the blocking serial cycle) skip the
pacer entirely and return seconds behind; re-anchoring is what makes that a single
fresh period rather than a burst.

`test_a_loop_more_than_one_period_behind_reanchors_instead_of_bursting` drives
exactly the 0.3022 s case and asserts one positive sleep per iteration — a
catch-up pacer would answer it with seven zero-length ones.

### Lever 2 — de-scoped on the census read, and on a premise that had gone stale

The plan pre-authorised this: *"`loop_obs_max_pre_s` from B0/P1 says whether this
is the dominant term before any of it is written."* It says no. Chained
`loop_obs_max_pre_s` is **p50 0.0026** against `body`'s **0.0233**, and the
per-cycle argmax is `body × 40, sleep × 32, obs × 1` over 73 cycles — obs is ~6 %
of the cost and the *smallest* of the three levers.

Worse for the lever, its premise has drifted. The plan describes
`_build_toss_observations` as rebuilding *"~25 fields per tick under the lock,
including a numpy `hypot` and a sensor query"*. On the **shipped default the numpy
call does not run at all**: `np.linalg.norm` at `:2590` is inside the `else` of
`if not mocap_body`, and `_TOSS_MOCAP_BODY_PARAM` is unconfigured by default, so
the builder takes the `platform_at_target = True` branch. The sensor query is one
`self._ball_sensor.evidence(now)` call, and caching it is *forbidden*, not merely
unhelpful — C-POSSESS-1 § 3.3 edit 1 requires the whole snapshot to be built from
one instant's cup state, and § 2.4.4 requires the read that admits the commit to
be the read that terminalises the upstream cycle. There is no trivially cheap win
left in there, so none was taken.

One thing worth carrying to B6: `loop_obs_max_pre_s` has a **heavy tail** —
p50 0.0026 but p90 0.0113, p95 0.0165, max 0.0400. The median says "not the
lever"; the tail says something occasionally makes the observation build 15× its
median, and the census cannot say what. That is a question for the post-B3/B4
census read, not a reason to cache the cup.

### Lever 3 — verified, and the plan's list is one entry short

Verified by AST call-graph over `ReloadCoordinatorNode`, not by eye
(`/tmp/probe_b5_lever3.py`, 2026-08-27): every `_wait_future` / `call_async`
reachable from `_step_toss_sequence`, `_tick_toss_pipeline` and
`_build_toss_observations`, with terminal-action handlers pruned. Five sites, and
the runtime guards on each read by hand:

| site | reached via | status |
|---|---|---|
| `_dispatch_toss_throw` `:5553` | the COMMIT tick | **necessary — leave it** (plan agrees) |
| `_position_platform_for_toss` `:4824` | POSITIONING, only when a move is commanded | **the plan's lever-3 text does not name this one.** Necessary, and it is the 0.2774 s `body` of the one moving cycle in the corpus. Never reached by a *staged* cycle (§ 2.4.1: a cycle only stages if its positioning decision is SKIP); on the serial path it is the first cycle of a sitting. Charged: `pre_dispatch_budget_s(True)` carries `TOSS_POSITION_MIN_MOVE_S + TOSS_POSITION_SETTLE_PAD_S` |
| `_arm_catch` `:8070`, `_set_soft_catch_gains` `:5441` | `_prepare_toss_catch` | **off the per-cycle path since B3.** `staged=True` returns before both (guards only); the chained serial path routes through `_arm_session`, which is `if self._toss_session_armed: return True` after cycle 1 |
| `_go_home` `:8176` | `_step_toss_sequence` `:4725`, the `_TOSS_POSITION_UNKNOWN_TERMINALS` zombie-superseder | teardown — it only runs on `decision.done`, i.e. the terminal iteration the census deliberately never commits. Covered by the plan's "teardown paths" wording, but reached from `_step_toss_sequence` rather than from a terminal action handler, which is worth knowing |

`_build_toss_observations` reaches **none**. `_read_platform_tilt` (the dwell-tilt
covariate) is not reachable from a tick at all: its single call site is the
session loop's quiescent-dwell branch, and B4's `committed_live` belt makes it
unreachable under the pipeline.

The plan's claim stands, with the positioning `go_to_pose` added to the list of
per-tick blocking calls that are *necessary and charged* rather than absent.

## Implementation

* `reload_coordinator_node.py` — `_PACE_PERIOD_S` (an **alias** of
  `TOSS_LOOP_PERIOD_S`, so the two cannot drift), `_PACE_SLOP_S`, and
  `_pace_to_next_tick(next_due, period, slop)`. A module function, not a method:
  it touches no node state, so it is testable against a fake clock without a
  node, and it looks `time` up on the module so the node tests' `_Clock`
  namespace patches it like every other clock read. Both loops initialise
  `next_due = t_start` and replace their `time.sleep(_TICK_S)` with
  `next_due = _pace_to_next_tick(next_due)`. `_TICK_S`'s own comment is amended:
  seven sleep sites became **five**, all reload-side.
* `toss_sequencer.py` — documentation only, and both edits are about keeping a
  field honest rather than changing one. `NODE_LOOP_PERIOD_S` gains a paragraph
  saying it is now a **set-point as well as a bound** (raise it and every toss
  loop slows; drop it below `loop_work_max_pre_s` and the pacer degenerates), and
  its stale "the loop is `work; time.sleep(_TICK_S)`" / "a numpy norm" sentences
  are corrected. `LoopPeriodCensus` gains the interpretation flip for
  `loop_sleep_max_pre_s`: the arithmetic is unchanged (`next_now − t_pre_sleep`,
  still measured, never inferred) but it now reports **headroom** — a value near
  zero is the interesting one, meaning the tick's work consumed the whole period.
  `tools/probes/toss_loop_census.py` reads the field by name and needs **no
  change**: the measurement is the same, and its "dominant term" argmax stays
  meaningful (a `sleep`-dominant cycle is now one with headroom to spare).

**Not paced, deliberately:** the four reload-side waits. They feed different
budgets (`SAFE_ABORT_LADDER_S`, `DEFAULT_SESSION_MISS_CLEANUP_S`,
`JB_BD_ARRIVAL_WINDOW_S`), the census has never measured them, and pacing a loop
whose period nothing has bounded is the guess D3's "measure, then choose" refused.

**Not touched:** `NODE_LOOP_PERIOD_S`'s value.
`test_the_census_never_feeds_a_budget` inspects the compiled identifier set of
`pre_dispatch_budget_s` / `min_throw_delay_for_release_s` and still passes; the
pacer reads the constant, the constant reads nothing.

## Verification

Scoped, run **2026-08-27**: `pytest tests/ros/test_toss_coordinator.py
tests/ros/test_toss_continuous_node.py -q -p no:randomly` — with the source edit
in and before the test edits, **312 passed, 1 failed in 99.05 s**; the single
failure was the stated-intent assertion below and nothing else.

Scoped sweep, run **2026-08-27**: `pytest tests/ros/ -q -p no:randomly` —
**2518 passed, 1 skipped in 332.26 s**, exit 0.

Scoped, run **2026-08-27** (final, after a comment reorder):
`pytest tests/ros/test_toss_coordinator.py tests/ros/test_toss_continuous_node.py
tests/ros/test_toss_sequencer.py -q -p no:randomly` — **470 passed in 101.35 s**.
`test_the_census_never_feeds_a_budget` is in that set and passes
(`pytest tests/ros/test_toss_sequencer.py -q -p no:randomly -k census_never_feeds`
— **1 passed in 0.29 s**, run 2026-08-27), which is the pin the plan names by
line for this phase.

Full gate, run by the orchestrator after this entry was drafted and before the
commit (`./run_tests.sh --full`, **2026-08-27**): parallel **6555 selected,
rc=0, in 524 s** + serial **9 passed (rc=0) in 45 s**, total 569 s, **RESULT: PASS** — the
(date, command, result) triple § 5.7 requires. The commit (`b316429`) carries
the same triple.

Logbook parse, run **2026-08-27**: `pytest tests/sim/test_logbook_search.py -q
-p no:randomly` — **24 passed in 0.26 s**. Run because that suite parses the real
`logbook/` directory and *silently drops* an entry whose front matter it cannot
read (`logbook/README.md` § "What the logbook tests actually check"), so a green
count is not evidence on its own: `load_entries()` was called directly and this
entry confirmed present with all eight front-matter keys populated.

Probe re-runs, run **2026-08-27** — both READERS, both unmodified, both to prove
the field semantics survived:
`python tools/probes/toss_loop_census.py` — 73 chained cycles, the same table as
B0/P1, exit 0;
`python tools/probes/cadence_rung_check.py --solve --frontier --pipeline --grid`
— **0 violations** pipelined, **196** on the counter-check, frontier
**56.3 throws/min**, exit 0 (unchanged, as it must be: `NODE_LOOP_PERIOD_S` did
not move).

Probe re-score, run **2026-08-27**: `/tmp/probe_pace_rescore.py` over the B0/P1
corpus (73 chained cycles) — projected worst-per-cycle period at PERIOD 0.040:
p50 0.0400 / p90 0.0400 / max 0.0418, cycles with work over PERIOD **2/73**; at
0.020, **49/73**; at 0.050, 0/73.

Probe, run **2026-08-27**: `/tmp/probe_sleep_granularity.py` — idle `time.sleep`
overshoot p50 0.1 ms / p99 0.1 ms / max 0.9 ms over 400 samples.

Probe, run **2026-08-27**: `/tmp/probe_b5_lever3.py` — the AST call-graph table
in § Discussion.

**Eight new tests** (the commit message's "nine" is a miscount) in
`tests/ros/test_toss_coordinator.py`: the constant drift
guard, the absolute grid over 20 varying-work iterations, the early-fire band,
the bounded catch-up, the re-anchor, the never-negative sweep, the structural pin
that both loops call the pacer and neither sleeps a fixed interval, and the
census truthfulness check.

**One changed assertion**, `tests/ros/test_toss_continuous_node.py:564`:
`DWELL <= achieved < DWELL + rcn._TICK_S` → `... + rcn._PACE_PERIOD_S`. The
session loop's START_CYCLE poll can now be up to a whole loop period late instead
of up to a whole sleep late — the cost this phase accepted and named at the
constant, and the bound is now denominated in the unit
`DEFAULT_SESSION_MISS_CLEANUP_S` already charges the same poll in.

**Not verified on hardware.** The mocked-ROS suite has no executor, no GIL
contention and a fake clock with zero wake latency, so it is structurally blind to
the quantity the pacing exists to bound — the same limitation the census entry
recorded about itself. The first real read is B6's sitting.

## Deploy

`cd ros_ws && colcon build --packages-select jugglebot && source install/setup.bash`,
then relaunch.

## Outcome

**What B6's re-cut looks like, mechanically.** The re-cut of `NODE_LOOP_PERIOD_S` is **not** taken here, and the reason is the
plan's own: it is re-cut *"by a human reading the census, never by the census"*,
and the census it needs is a **post-B3/B4/B5 sitting**, which does not exist. The
2026-08-26 corpus is a pre-B3 unpaced loop; re-cutting off it would size a
constant against a machine that no longer exists.

When that sitting lands, B6 executes four mechanical steps:

1. **Read.** `python tools/probes/toss_loop_census.py --csv --json` against the
   new corpus. The number is `max(loop_period_max_pre_s)` over the **chained**
   class, ceiled to the next 10 ms (the `ARRIVAL_BAND_MAX_S` sizing discipline —
   a constant is a bound, not a datum). The probe prints exactly this as
   `chained period bound`. Sanity check it against `loop_work_max_pre_s`: under
   pacing the period cannot fall below the paced value, so a bound *above* 0.040
   means work is overrunning and the answer is to trim work, not to raise the
   constant.
2. **Cut one constant.** `toss_sequencer.NODE_LOOP_PERIOD_S`. Nothing else:
   `_PACE_PERIOD_S` is an alias, `pre_dispatch_budget_s` / `commit_budget_s` /
   `SAFE_ABORT_LADDER_S` / `DEFAULT_SESSION_MISS_CLEANUP_S` are all derived, and
   `cadence_rung_check.py` imports it. Leave `NODE_TICK_S` alone — it is the
   reload-side sleep and its drift guard, not an arithmetic unit.
3. **Re-cut the two tables.** Plan § 2.7's pipelined-floor table and § 6.2's rung
   ladder, both reproduced by
   `python tools/probes/cadence_rung_check.py --solve --frontier --pipeline --grid`.
   At 0.025 s the plan predicts the `h = 1.0` floor falls 0.4170 → 0.4020 and the
   clearance triples to 32.9 ms; at the B0-measured 0.070 it is 0.4470 and rung
   P5 is refused outright. The probe reports which.
4. **Re-run and re-publish.** `tests/hardware/session_cadence_ladder.md`'s
   clearance table, and the plan's § 3 B5 row.

Two things B6 should carry in from here:

* the acceptance criterion **"`loop_n_over_pre` is 0 on every successful cycle"**
  predates pacing. A paced period still carries `wake_k − wake_{k−1}`, so a
  handful of near-threshold periods with `loop_work_max_pre_s` well under the
  constant is scheduler jitter, not a finding. Work over the constant is the
  finding;
* `loop_obs_max_pre_s`'s tail (p50 0.0026, max 0.0400) is unexplained and the
  census cannot explain it. If lever 2 is ever re-opened, that tail — not the
  median — is what would justify it.
