---
title: Handoff-catch SPLICE_TOO_LATE - a longer pinned lead, the pinned re-aim as the late adjustment (solve chaining conditional)
created: 2026-10-15
status: completed
completed: 2026-10-16
archived: 2026-10-16
owner: harrison
last_updated: 2026-10-16
related_logbook:
  - 2026-10-14-two-ball-columns-sessions-why-attempts-end.md
related_code:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/schedule.py::HANDOFF_LEAD_EXTRA_KNOTS
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py::_snap_to_release
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py::SkillExecutor._resend_live_catch
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py::SkillExecutor._catch_aim
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py::_svc_install_segment
  - ros_ws/src/jugglebot/jugglebot/motion/skills/solver_worker.py::WorkerSegmentSolver
---

# Handoff-catch SPLICE_TOO_LATE: longer pinned lead + pinned re-aim (chaining conditional)

The filename is the first draft's; it was reframed 2026-10-15 (§ 0.2, § 0.4). Branch `handoff-robustness` off
`skill-stack` 4f236485. Absorbs `skill-stack-dress-rehearsals-and-residue.md` § 2 (b) `SPLICE_TOO_LATE`.

**If your physical intuition disagrees with the framing - especially the re-aim timeline in § 1 - that is
load-bearing signal; say so.**

## 0. Root cause, findings, decisions

### 0.1 The failure

A CATCH that follows a release splices AT that release knot (`executor._snap_to_release`, executor.py:1250-1316),
and its wire cliff is `t0 + (k_rel - WIRE_READ_KNOTS)*dt` (`splice_cliff_s`, :1336). The solve is a dense QP that
either converges or refuses, with no time cap. Accepted handoff installs take p50 137-139 / p90 188-198 / max
223-231 ms from the dispatch stamp; the seven attempt-ending `SPLICE_TOO_LATE`s replied 215-272 ms after it, which
is the right tail of the same distribution. The seam is pinned, so a late reply has no graceful alternative and
ends the attempt. Fix 1 (7f590d41, extra knots 2 -> 4, 250 ms of budget) still leaves five of the six recorded
failures within 20 ms of the cliff. Owner target: ZERO self-ends on the recorded timings, with margin.

Each extra knot buys the first install 25 ms, because the cliff is pinned to the release and does not move with the
dispatch. It costs 25 ms of dispatch gap: the minimum gap is `0.400 - 0.025 * EXTRA` s (probe 2026-10-15, the same
for apex 0.80-1.10, dwell 0.27/0.50, sep 125/150, columns and one-ball), which is 0.250 s at 6 and 0.200 s at 8.

### 0.2 Findings that reframed the first draft (accepted by the owner 2026-10-15)

- **F-A: a longer lead QUEUES; it does not read a stale record.** The skill_node tick blocks in `_wait_future`
  for the whole install (skill_node.py:2140, 2301), and `install_segment` runs in trajectory_node's
  mutually-exclusive group (:1005, :5031). So a CATCH that comes due mid-solve is sent the moment its predecessor
  installs (executor.py:3260-3272) and reads the installed head. The cost is at most 231 - 200 = 31 ms of budget
  at the minimum gap. Chaining would win back only ~25-50 ms, and only with pipelining: that is § 2.C.
- **F-B: the first aim is usually the tracker.** `_catch_aim` ranks a converged fit first (executor.py:2913-2967).
  The dispatch-time `CATCH-AIM ... landing=` lines of the 2026-10-13/14 logs show 146 tracker / 51 schedule
  (74 %). An earlier dispatch moves these onto the prior; § 1 is the answer.

### 0.3 Decisions

- **D1 Lead.** `HANDOFF_LEAD_EXTRA_KNOTS` 4 -> **8** (owner, 2026-10-15, on the § 0.4 numbers). `LEAD_S`,
  `CATCH_FREEZE_S` and `SOLVE_BUDGET_KNOTS` are unchanged.
- **D2 Late adjustment = the pinned re-aim, with SNAP-AWARE fences** (owner, 2026-10-15; P1b). No new solve path.
- **D3 Acceptance offline:** a replay of every recorded handoff catch -> zero first-install `SPLICE_TOO_LATE` with
  >= 25 ms margin, and the fit-loss rule of P0 met. Hardware then confirms (P2).
- **D4 Chaining is conditional** (§ 2.C, trigger stated there). The solve-cost cuts and the cheap xy bump are § 4.
- **D5 Fix 1 facts.** The node deadline for a pinned splice is already `max(t_wall + LEAD_S + dt,
  splice_cliff_s - REPLY_TRANSIT_ALLOW_S)`, with `splice_knot_for` computed BEFORE the solve
  (trajectory_node.py:4640-4664), so the deadline follows the cliff for first installs AND re-sends. Only the
  fixed-wait literal and `test_install_segment.py`'s 255/262 ms cases (:1743, :1759) move.

### 0.4 Phase 0 outcome, 2026-10-15

- **Probe and data.** Probe `tools/probes/handoff_replay.py`; outputs in
  `temp/probes/handoff_replay/{handoff_catches.csv,summary.md}`. 299 handoff catches (S1 65 / S2 97 / S8 137),
  identified as first-dispatch, non-RESEND CATCHes whose scheduled release minus dispatch is > 0.29 s.
- **Cliff derivation.** The cliff is derived as `t0 + (K - 3)*dt` from the logged splice knot. The nominal
  `D + 0.200` was contradicted by accepted installs landing 0.223-0.231 s after dispatch. The derived cliff sits at
  about `D + 0.238`. **RESOLVED at P2 (2026-10-16):** the node's own `cliff +X ms` fields over 67 refusals put the
  real cliff +0 to +25 ms (median +21) past nominal — the ceil-to-knot quantisation of the splice knot, never
  negative, so every nominal budget here is a lower bound; the probe's own derivation reads +27 ms on the same data, so the
  lead-13 38 ms was that derivation within one knot of the node's arithmetic, not a clock offset.
- **Sanity at the recorded lead.** 0 negative accepted margins (one of -0.4 ms is rounding); 11 of the 12 END
  refusals come out negative.
- **First-install margin, min/median.** On the 12 ENDs: 13/28 ms at 0.325 s (upper bounds for the two no-answer
  rows), 63/78 at 0.375, 113/128 at 0.425. Accepted minima: 50 / 100 / 150 ms. Rule (a) passes at 6 and 8.
- **Fit available at dispatch.** 61 % as recorded, 45 % @4, 10 % @6, 5 % @8. Live fits converge mostly 0.2-0.3 s
  after release, NOT at the 0.11 s offline figure.
- **Re-aim budget** for prior-aimed catches, pinned, = cliff - max(fit, first-install end). Share with >= 190 ms:
  16 % @4, 68 % @6, 75 % @8. About 16 % of catches have their fit after the cliff at every lead, so no pinned
  re-aim is possible for them at any lead.
- **Rule (b) FAILS at every lead**, because nearly every catch becomes prior-aimed. The owner chose **8 + the
  snap-aware fences**, accepting the consequence: with the fences fixed, about **20 %** of catches end on the
  prior, against ~39 % today; without the fence fix, ~95 % would.

## 1. The late adjustment: the pinned re-aim

### 1.1 Why a re-send gets the same cliff

`_snap_to_release` snaps any THROW/CATCH whose raw splice knot satisfies `k_s <= k_rel + n_detach` (n_detach = 2,
cup_cycle.py:229). Here `k_rel` is the last release the head carries BEFORE the segment's own event. For a re-send
of catch-A that release is still B's: A's own carried re-throw lies after A's landing and is filtered out by
`event_k`. The node computes the raw knot with `LEAD_S` (0.225 s). So a re-send fired at `t <= k_rel + 2*dt -
LEAD_S` splices at the same release knot as the first install and has the same cliff. Fix 1's deadline already
waits to that cliff (D5). After that instant a re-send splices at its raw knot with the free-standing 150 ms budget.

### 1.2 RESEND rules: what P1b changes and what it keeps

`_resend_live_catch` (executor.py:4656-4758).

**UNCHANGED, and load-bearing:**
- the first-install gate (`_live_catch` is set only after an accepted dispatch, :3496);
- the converged-fit requirement (:4706);
- the 10 mm / 10 ms tolerance, the 25 ms interval and the cap of 2 (:1757-1760);
- `CATCH_FREEZE_S`;
- the 2026-09-20 terminal rule (:4735-4750): a SOLVED re-send that is refused ends re-aiming.

**CHANGED by P1b:**
- **The defect.** `MIN_WINDOW` (:4690) and the `CONTACT-WINDOW` decline (:4726) both measure the splice as
  `t + skill.lead_s`. For a handoff catch that is `HANDOFF_LEAD_S`, which is neither where the node splices (the
  release knot inside the snap band, `t + LEAD_S` after it) nor what the node uses.
- **What happens today.** Re-aiming closes at `t_land - 0.125 - HANDOFF_LEAD_S` (0.330 at 8). A snapped re-send
  near the cliff is SOLVED and refused `SPLICE_TOO_LATE`, and that refusal spends the catch's re-aim (terminal).
- **The fix.** Both fences take the real splice instant (spec in P1b). A new NON-TERMINAL decline stops a snapped
  re-send whose budget to the cliff is below the floor.

### 1.3 Timeline: apex 0.95, dwell 0.27, sep 125 (after P1b)

A is released at 0 and B at 0.575, and catch-A is pinned at B's release (`k_rel` 0.575, cliff 0.500 nominal,
§ 0.4 bias). A touches down at 0.880. A's live fit converges 0.2-0.3 s after release. Installs take p50 139 ms. A
re-send is snapped while `t + 0.225 <= 0.625`, i.e. `t <= 0.400`.

| | 4 extra | 6 extra | 8 extra |
|---|---|---|---|
| first dispatch | 0.250 | 0.200 | 0.150 |
| first-install budget | 250 ms | 300 ms | 350 ms |
| fit available at dispatch (P0) | 45 % | 10 % | 5 % |
| first install done (p50) | 0.389 | 0.339 | 0.289 |
| pinned re-aim open (floor 150 ms -> `t <= 0.350`) | never | never | 0.289-0.350 |
| raw re-aim (`t > 0.400`, 150 ms budget) open until CONTACT-WINDOW `t + 0.225 >= 0.755` | 0.400-0.530 | 0.400-0.530 | 0.400-0.530 |

At 8, the pinned re-aim runs from first-install end to the floor; P0's per-catch budget is >= 190 ms for 75 % (shipped floor: 150 ms). From 0.350 to 0.400 a re-send is pre-declined, not spent. After that the raw path re-aims as today, with a 150 ms
budget, tight against tracker-aim solves (p50 126-146). Without P1b the window closes at 0.330, and a near-cliff
pinned re-send is terminal.

### 1.4 One control cycle at 40 Hz

- **The wire** changes only at an install. The emitter samples the installed record through every solve.
- **First installs and pinned re-sends** splice at `k_rel` from the same seed under the unchanged transit guard.
- **Feedforward** is per-knot from the installed `CyclePlan` and is untouched.
- **A refusal** installs nothing, and the rest tail runs out.
- **The one new timing fact** is the tick blocking up to ~31 ms longer (F-A).

## 2. Phases

Each phase is one Sonnet unit, under ~80 tool calls, its own commit with a `Logbook-Entry:` trailer.

**P0 - replay harness: DONE 2026-10-15** (slug `2026-10-15-handoff-replay-baseline`; outcome in § 0.4).
For each recorded handoff catch at leads 4/6/8 it reported:
- the first-install margin (serial queue, pinned cliff);
- whether a fit existed at dispatch;
- the pinned re-aim budget.

**Phase 0 decision rules (pre-registered 2026-10-15):**
> The lead is **8** unless **(a)** any uncensored recorded pair has < 25 ms first-install margin at 8, or **(b)**
> more than **5 %** of recorded handoff first dispatches lose their converged fit at the 8-knot dispatch instant
> AND their re-aim budget at 8 would be below the tracker-aim p90 (190 ms) or have no window. If (a) or (b) holds
> the lead is **6**, and the same two tests are re-applied at 6. If 6 also fails (a), the conditional § 2.C is
> triggered. If 6 fails only (b), the owner chooses between 6 and 4 on the printed numbers.

*Why 5 %:* it is the top of the SPLICE_TOO_LATE self-end rate being bought out (2.7-5.1 % per dispatch).
Above it, unrecoverable prior-aimed catches would outnumber the self-ends removed. Outcome: (a) passes at 6 and 8,
(b) fails at every lead. The owner overrode toward 8 + P1b (§ 0.4).

**P1 (P1a + P1b) - IMPLEMENTED 2026-10-16, FLOWN 2026-10-16 (P2)** (one commit, slug `2026-10-16-handoff-lead-8-knots-and-snap-aware-resend-fences`).
**P1a - the lead change (8).** Slug `2026-10-16-handoff-lead-8-knots-and-snap-aware-resend-fences`.
- `schedule.py:126` 4 -> 8, with the docstring history (2 -> 4 -> 8, the § 0.4 numbers).
- Fix 1's fixed-wait literal.
- `tests/ros/test_install_segment.py` :1743/:1759 re-pinned to the 8-knot cliff.
- `tests/ros/test_skills_plan_bench.py:175` renamed and re-pinned.
- `tests/motion/test_skills_schedule.py`: the monotone cases unchanged, plus
  `test_the_handoff_min_gap_is_400_ms_less_a_knot_per_extra_knot`.
- Gate: `handoff_replay.py --extra 8` (0 SPLICE_TOO_LATE, worst margin >= 25 ms), then `./run_tests.sh`.

**P1b - snap-aware re-aim fences** (`SkillExecutor._resend_live_catch`, executor.py ~4688-4727).
Slug `2026-10-16-handoff-lead-8-knots-and-snap-aware-resend-fences`.
- **The real splice instant `t_s(t)`**, computed as the node will compute it:
  - `t_rel` is the last release instant BEFORE this catch's landing among the schedule's earlier skills. Use
    `schedule._release_instants` (a THROW's `t_abs_s`; a CATCH's `then_throw.t_release_abs_s`), which is the same
    `last_release_s` `_assign_leads` computes. For a handoff catch it equals `skill.t_abs_s - skill.window_s`.
  - `t_s = t_rel` if `t + LEAD_S <= t_rel + n_detach*dt - dt/2`; otherwise `t_s = t + LEAD_S`. Use the
    module-level `LEAD_S`, NOT `skill.lead_s`.
- **Why the schedule's instant and not the record's knot:**
  - The executor ticks on the ROS clock, while `t0` / `InstallResult.t0_s` are on the wire's `perf_counter`
    clock (skill_node.py:2149-2151, 2349-2352). The offset is not a multiple of dt, so `k_rel` cannot be
    reproduced; the error is at most dt/2.
  - The `- dt/2` errs toward "raw", which has later, tighter fences. The cliff estimate is
    `t_rel - WIRE_READ_KNOTS*dt - dt/2`.
- **Fences:**
  - `MIN_WINDOW`: `t_land - t_s < MIN_WINDOW_S`.
  - `CONTACT-WINDOW`: `t_s >= t_land - CUP_CONTACT_WINDOW_LEAD_S`.
  - Both keep today's terminal / non-terminal character.
- **New pre-decline:** `PINNED-BUDGET`, non-terminal. It does not count toward the cap, does not clear
  `_live_catch`, and logs once per catch through `_resend_declined`.
  - Fires when the re-send is snapped and `cliff - t < RESEND_PINNED_BUDGET_FLOOR_S` = `LEAD_S - WIRE_READ_KNOTS*dt` = **0.150 s**.
  - Why 0.150: it is the raw path's minimum budget (raw budget is in [0.150, 0.175)). Declining hands the catch to
    the raw path, whose refusal is equally terminal, so never decline onto a path with no more budget. Above the
    floor the pinned path is at least as good on budget and keeps the seam at the release.
  - Tested on `maybe_snapped` (`+ dt/2`, node may still snap); window fences stay on `snapped` (`- dt/2`).
- **After the band:** the raw path at `t + LEAD_S` with the free-standing 150 ms budget, exactly as today.
- **Tests** (`tests/motion/test_skills_executor.py`):
  - `test_a_pinned_resend_window_opens_at_first_install_end_and_closes_at_the_floor[4|6|8]`
  - `test_the_pinned_budget_decline_is_not_terminal` (a later raw re-send still fires)
  - `test_a_resend_after_the_snap_band_splices_raw`, `test_the_fences_use_LEAD_S_not_the_handoff_lead`
- **Control implication:**
  - The executor's fences only decide whether to ASK for a re-send. The node's `SPLICE_TOO_LATE`, transit and
    limits refusals are unchanged and remain the safety layer, with the firmware `MAX_DEVIATION` behind them.
  - A pinned re-send re-solves the same window, from the same release seed (`release_state_at_knot` at `k_rel`),
    that a first tracker-aimed install would have used. On the wire it is indistinguishable from today's accepted
    pinned re-sends.
- **Gate:**
  - `handoff_replay.py` re-run with the fence model must reproduce the § 1.3 / § 0.4 budget table.
  - `./run_tests.sh --full` closes P1 (P1a + P1b), followed by colcon build and a full launch restart.
- **Also in P1, or P2 if the logs are needed:** resolve the 38 ms cliff bias (§ 0.4) from `_refusal_timing`
  lines.

**P2 - dress rehearsal row + close-out: FLOWN 2026-10-16 10:50, TARGET MET** (zero handoff `SPLICE_TOO_LATE` ENDs in 17 attempts / 175 handoff catches, worst first-install margin 70 ms, three installs over Fix 1's budget; § 2.C's trigger did not fire; rows 23-29 read retrospectively in `logbook/2026-10-16-handoff-lead-flown-zero-splice-too-late-tracker-fit-late.md` — the tracker fit arrived ~90 ms later than in the recorded sessions, so the first aim was the prior on 175/175 and only 30 re-aims installed; follow-ups in § 4). The runsheet (rows 23-29) was written for the loaded Jetson with all refusals reported at once; the operator did
not run rows 26-29 live, so every read is retrospective from the launch log and the bag. Read: schedules compiled
monotone (row 24, offline), the `CATCH-AIM` source mix 175/175 prior, every installed re-send `splice_k == k_rel`,
`PINNED-BUDGET` declined at most once per catch and never followed by a budget END, zero handoff `SPLICE_TOO_LATE`
ENDs. Next: `/archive-plan`.

### 2.C CONDITIONAL - worker chaining + pipelining + async dispatch

**Trigger (either):** P0 shows < 25 ms first-install margin on any recorded pair at the chosen lead, OR the next
sitting still produces a `SPLICE_TOO_LATE` END on a handoff catch. Otherwise this section is never scheduled, and
it is archived with the plan as the recorded design. Two units: C1 worker + client (slug `...-handoff-worker-chaining`),
C2 node contract + async dispatch (slug `...-handoff-chain-contract`).

- **Request/reply** (solver_worker.py:79-111):
  - `SegmentSolveRequest.chain`: when set, `record=None` and the worker uses its retained head, `(seq, record,
    warm_start)` of its last ACCEPTED solve (cleared only by a restart).
  - `SegmentSolveReply.chained_on_seq`.
  - A chain with no retained head is refused `PREDECESSOR_NOT_INSTALLED`.
  - Only a pinned handoff with an outstanding predecessor chains. Fresh origins, REST, MAGAZINE and RESEND ship
    the record.
- **Client:** per-seq reply slots, <= 2 in flight. Today `_await` drops non-matching replies (:520-525). One
  serial worker.
- **Node:** `install_segment` gets its own reentrant group, an admission semaphore of 2, an install-order gate, 4
  threads, and chained `deadline_s = t_wall + HANDOFF_LEAD_S`.
- **Executor:** an async `PendingInstall`, finalised in dispatch order. While an install is pending, only a
  chained CATCH dispatches. A predecessor fallback -> `CHAIN-CANCELLED`, then re-dispatch.
- **Contract, INVARIANTS.md § 7, row I-CHAIN-1:**
  > A segment solved on a seed the node did not hand over MUST be installed only onto the record that seed came
  > from: a chained reply is installed iff the node's installed head was produced by solve `reply.chained_on_seq`
  > and no install of any kind has happened since; otherwise it is refused `PREDECESSOR_NOT_INSTALLED`, nothing
  > is installed, and the attempt ends. Enforcement: `trajectory_node._install(require_head_seq=)` under
  > `_plan_lock`, in the same acquisition as `require_epoch` (:2822). Test:
  > `tests/ros/test_install_segment.py::test_a_chained_reply_installs_only_onto_the_head_it_was_solved_on`.
  > Parity: `tests/motion/test_solver_worker.py::test_a_chained_seed_equals_the_installed_record_seed`.

  Root cause it closes: `splice_at` (unified_cycle.py:2980) keeps knots `[0, k_s)` of the record it was given. If
  that record is not the wire's head, the six legs step at the next sample, and `_seam_check` (:2404) cannot see
  it.
- **Stale-seed rows (test ids):**
  1. predecessor refused by the worker (`pred_worker_refused`);
  2. predecessor refused at the node (`pred_node_refused`);
  3. hold between the two; `SUPERSEDED_BY_HOLD` wins (`hold_between`);
  4. predecessor RESEND after the chain (`resend_after_chain`). Unreachable: the predecessor's re-aim window
     closes before the CATCH dispatch at any dwell. Belt-and-braces.
  5. predecessor RESEND accepted by the worker, refused at the node (`resend_node_refused`);
  6. worker restart (`worker_restart`);
  7. predecessor fallback -> `CHAIN-CANCELLED` (`fallback_cancels_chain`).

  A RESEND that the worker refuses is not stale.
## 3. Risks, with the control implication of each

- **First aims fall back to the prior.** Expect about 20 % of catches to end on the prior (§ 0.4); this was
  accepted. There is no wire effect. P2 checks the source mix.
- **The tick blocks up to ~31 ms longer** at the minimum gap, while a due CATCH waits for its predecessor's install.
  Re-aims, catch evidence and the mode / lane-refused observation wait with it. The emitter is unaffected.
- **A re-aim refused on limits is still terminal** (`LIMIT_*`, 2026-09-20 rule). P1b only stops spending the re-aim
  on a refusal the executor could predict (the budget). About 16 % of catches have no fit before the cliff at any
  lead in the recorded sessions (48 % at the 2026-10-16 sitting, see § 4).
- **The cliff bias is resolved (P2, § 0.4):** +0 to +25 ms of knot quantisation, never negative, so the 150 ms
  floor and the budget table are lower bounds.
- **§ 2.C only:** a wrong-head install (I-CHAIN-1), Foxy thread-pool exhaustion (semaphore 2), and async END
  timing.

## 4. Deferred (named, out of scope)

- Solve-cost cuts:
  - warm start through `plan_settle`, keyed to survive window-length changes (segments.py:402);
  - one gate pass over the joined range;
  - no per-solve `inv(H)` (cup_cycle.py:1678);
  - `stage_wall_s` on the install log line.
- The cheap late re-aim: a jerk-bounded cup-xy bump on the installed tail.


**Recorded at P2 (2026-10-16), not this rung's to do:**
- **Tracker fit latency since the matcher fix 5974d8c3** (first flown today): jugglebot-throw fits converged 540 ms
  before landing at the median vs 630-645 in S1/S2/S8, p10 142 vs 566, and 36 % of flights never converged (vs
  13-23 %); `TRACKER-IDENTITY-REFUSED` 51 vs 12-15 per session; learner rows 64 % of catches vs 85 %. Confirmation
  itself is 2-3 steps, so the delay is downstream of it. Bag `~/Desktop/rosbags/2026-10-16_10-50-42`.
- **A fresh-origin lead for the reload THROW:** `LEAD_S` gives it 150 ms; today's THROW plans ran 44-165 ms (recorded
  41-88) and one attempt ended `ORIGIN_TOO_LATE` on 188 ms. Same class as this rung; a per-kind lead in
  `_assign_leads`, replayed first.
- **The raw-path re-aim after the snap band never installs** (53 fired, 53 refused on `LIMIT_*`/`INFEASIBLE`, 0
  installed): a re-send at D + 0.30 s splices 100 ms after the release knot with ~240 ms to cup contact. It is
  spent on a terminal refusal; a physics fence (or a non-terminal limit refusal for raw re-sends) would keep the
  re-aim count honest. Harmless to the catch today.

## 5. Done means

- P0: done 2026-10-15 (§ 0.4); recorded in the P0 logbook entry.
- P1a: done 2026-10-16 (`handoff_replay.py --extra 8`: 0 SPLICE_TOO_LATE, worst recorded failure +113 ms; P1 entry).
- P1b: done 2026-10-16 (fence model reproduced the § 1.3 / § 0.4 table; P1 entry).
- P1 close: done 2026-10-16 (`./run_tests.sh --full`, counts in the P1 entry); the 38 ms bias carried to P2 and resolved there (§ 0.4).
- P2: DONE 2026-10-16 — rows 23-29 read (retrospectively), 0 handoff `SPLICE_TOO_LATE` ENDs of 175, source mix
  175/175 prior, 30 re-aims installed; closing entry written; § 2.C's trigger did not fire. Next: `/archive-plan`.

## Archival note (2026-10-16)

**Shipped:** `HANDOFF_LEAD_EXTRA_KNOTS` 4 -> 8 (handoff lead 17 knots, 350 ms of first-install solve budget; min dispatch
gap 0.200 s at the columns point) and the catch re-aim fences re-keyed on the real splice instant with the non-terminal
`PINNED-BUDGET` decline (floor = the raw path's own 0.150 s; invariant I-CATCH-4). Commits 1f4a3cd0 (P0), ecc21d68 (P1),
466878eb (P2 runsheet), 3b3b90d0 (P2 close). **Why it closed:** flown 2026-10-16 10:50 — zero handoff `SPLICE_TOO_LATE`
ENDs in 17 attempts / 175 handoff catches, worst first-install margin 70 ms, three installs over Fix 1's budget; the cliff
bias is knot quantisation (+0..+25 ms). § 2.C (worker chaining) never triggered and is archived here as the recorded design.
**Residue re-homed:** the three § 4 follow-ups found at P2 — the tracker flight fit converging ~90 ms later since the matcher
fix 5974d8c3, a fresh-origin lead for the reload THROW (one `ORIGIN_TOO_LATE` on 188 ms), and the raw-path re-aim that never
installs after the snap band — are opened as their own work (2026-10-16 evening, Opus units on branches
`tracker-fit-latency` and `throw-fresh-lead`; the raw-path fence deferred until the tracker fix shows whether it matters).
The sitting read-out is `logbook/2026-10-16-handoff-lead-flown-zero-splice-too-late-tracker-fit-late.md`.
