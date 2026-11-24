---
title: "BUG-95 finished (TKT-76 option 1): the band REST slides by whole knots only while the wire has read past its anchor, is capped at 8 knots (0.200 s) and then refused SPLICE_TOO_LATE as before, stays on the knot grid, and INVARIANTS.md K1 and I-INSTALL-1 now say so"
type: bugfix
date: 2026-11-25
status: complete
phase: "BUG-95 day session"
subsystem: skill-stack
related_items: [BUG-95, TKT-76]
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - tests/motion/test_skills_executor.py
  - tests/motion/test_magazine_throw.py
  - logbook/2026-11-25-bug-95-band-rest-slide-cap.md
---

# BUG-95 finished: the band REST's slide is whole knots, capped, on the grid

The Night Shift's fix (`logbook/2026-11-16-bug-95-rest-band-installs-fresh-at-the-records-end.md`, cherry-picked onto main without code conflicts) made the fresh decision read `splice_lead_s` and started a REST in the band at the running record's end. The owner answered TKT-76 with option 1: accept it, cap the rebase slide, amend the invariants. This entry closes that entry's five open findings.

## What changed

- **INVARIANTS.md first.** K1 gains a "one lead for the fresh decision, and the band REST" clause naming `install_is_fresh`, the anchor, the whole-knot slide, the cap, its enforcement point and five tests. The REST-splice-lead clause now says "never a fresh REST, band REST included". I-INSTALL-1's exemption is narrowed to a fresh REST whose origin is now; a band REST is not exempt and names its enforcement (`_svc_install_segment`, `stream_handoff` armed when `rest_waits_for_end`) and its node test.
- **(a) The cap.** `executor.BAND_REST_SLIDE_CAP_KNOTS = 8` (0.200 s), one enforcement point `band_rest_slide_refusal(slide_knots)`, read by `install_segment`'s fresh branch. Past the cap the refusal is `SPLICE_TOO_LATE`.
- **The slide rule (changed from the night's).** A band REST now takes the splice's own wire test with `k_s = 0` on its anchor `E`: `k_wire = floor((t_inst - E)/dt) + WIRE_READ_KNOTS`. If `k_wire < 0` it installs at `E`. Otherwise it slides `k_wire + 1` whole knots, through `_rebase_fresh_rest`'s held branch, so the plan is untouched. The night's version used the generic fresh-REST rule (`solve - 75 ms > 0`, then `t_inst + 75 ms`). See the Discussion.
- **(b)** `install_is_fresh` and `_magazine_splice_refusal` now run inside the `try` in both `install_segment` and `check_segment`. A NaN `t_rest_s` raised `ValueError` out of `math.ceil` in `splice_lead_s`. It now returns `UNREACHABLE`.
- **(c)** A band REST onto a MAGAZINE record is accepted as fresh, which is intended. It is pinned by a test in `tests/motion/test_magazine_throw.py`.
- **(d)** The band anchor is `_ceil_to_knot_grid(record.end_s)`, a twin of the node's `_ceil_to_grid` (the executor cannot import the node). It returns an on-grid end unchanged, bit for bit. A slide is whole knots from an on-grid anchor, so the REBASED origin stays on the grid too.
- Messages: an accepted band install says `band REST anchored at the running record's end` (plus `installed ±x s from it, slide n of at most 8 knots` when REBASED). The existing prefixes `fresh origin:` and `fresh origin REBASED +x s` are unchanged.
- The branch's own slow-solve test moved from `E + 0.20` (now over the cap) to `E + 0.09` (7 knots).

## Discussion

**The cap's number, against the data.** The brief's premise was that the largest slides ever measured were +0.16 to +0.18 s. That premise is too narrow. I parsed every `fresh origin REBASED` line in `~/.ros/log` and `$MAIN/temp/logs`, deduplicated by timestamp, into the scratchpad's `out/bug95/rebase_slides.csv`. 200 REBASEs were logged from 2026-09-28 to 11-20:
- **At-rest seed (n = 185):** min +0.150, p50 +0.162, p90 +0.243, p99 +0.693, max +1.015 s. 30 of the 185 are over 0.200 s.
- **In-motion seed (n = 15):** +0.175 to +0.225 s.

Every large slide is a long non-band PRE-TILT REST of 3.4 to 7.3 s. For example, 2026-10-19 15:27 slid +0.693 s on a 6.9 s REST, and the Feed CATCH then spliced and caught. One large slide failed: on 2026-10-28 09:53 a slide of +0.673 s on a 4.575 s REST ended the Attempt `TILT_PIN` at the Feed CATCH ("seed attitude off the held receive attitude … 0.0853 deg > 0.0573 deg"). That failure is the class the cap exists for. A slid PRE-TILT REST is still settling into its pre-tilt attitude when the Feed CATCH (absolute landing) splices into its tail.

So 0.200 s does not sit at the edge of flown data. It sits on the conservative side of it: above the median and above R5 sitting 2's +0.16 to +0.18 s, and far below the one known failure. The cap cannot add a new failure, because every refused slide was refused by the old clamped splice too. I implemented the orchestrator's 0.200 s. The data supports loosening it later if a Sitting shows band refusals. The uncapped non-band PRE-TILT slides, and that TILT_PIN, are outside this Item (see Findings below).

**Why the slide rule changed (one approach beat another).** In the walk of the night's rule, a band REST whose solve lands in `(E, E + 75 ms]` installed at slide 0 with its origin in the past. The emitter's next tick then sampled the plan at τ = 75 ms, so its first knots were skipped. In the test fixture (a 100 mm traverse over 2.8 s) the first emitted knot was 0.228 mm and 7.19 mm/s off a held rest. That gives the firmware a 25 ms Hermite segment with about 1600 mm/s² at its start, where the plan's own acceleration is about 120 mm/s². That is the generic fresh-REST rule's accepted behaviour, but for a band REST the old code refused those solves. "The cap can only restore yesterday's behaviour" was therefore not quite true of the night's version.

The `k_s = 0` wire test makes that sentence exact. In knots, the old clamped splice refused iff `floor((t_inst - E)/dt) >= -3`, which is `k_wire >= 0`. The new rule installs at `E` for exactly the solves the old one accepted, with the same plan on the wire from `E`. It slides 1 to 8 knots where the old one refused, and refuses beyond 8. No knot of the band REST is ever behind the wire, and every accepted slide is whole knots on the grid.

**(d) Fixed, not proved harmless.** An off-grid origin is sampled off-knot by the 40 Hz emitter at every tick of the plan, not only at its start. The C2FF measurement (the trajectory_node "Unified 7-DoF cycle" comment block) put that at 299 to 679 rev/s² Hand acceleration steps at every frame boundary. That is a torque-feedforward discontinuity, which is exactly why fresh origins are snapped. A band REST skips the node's snap, so it inherits the record's phase. If the record was an at-rest REBASED fresh REST (`t0 = t_inst + 75 ms`, off-grid by construction), the band REST would stream off-knot for its whole window. Rounding the anchor up costs less than one knot of holding the record's terminal rest, which is exactly knot 0, so the snap introduces no step.

**(c) The MAGAZINE acceptance is intended and safe.** `MAGAZINE_NEEDS_REST` exists because a SPLICE cannot cut a magazine plan: its meta has no `cup_plan`. A band REST splices nothing. The magazine plays out whole, and the wire hands over at `t0` (I-INSTALL-1). The seed reads only `plan.pose[-1]`, `hand_rev[-1]` and `meta.levelling_correction`, all of which the magazine meta carries. At the magazine's end the Hand holds the magazine's own terminal rest: the test asserts zero platform twist and zero Hand velocity there, and the module docstring says "the plan ends at REST". That is the same seed a fresh THROW dispatched within `LEAD_S` of a streaming magazine already uses.

**One control cycle, the capped path.** Fixture: a 3.2 s REST record ending at `E` (on the grid). The band REST to the other Site is dispatched at `E - 0.4 s`: install lead 0.225 s, splice lead 0.475 s, plan 2.8 s. Numbers are from the scratchpad's `out/bug95/probe_walk.py` (output `probe_walk.out`), run 2026-11-25. The emitter sends one knot per 25 ms tick `t_k`, stamped `t_k` (`emit_lead_knots: 0`). Until a new plan's origin, the wire plays the running record, which holds its terminal rest past `E` (`CyclePlan.state_at` clamps). The node arms the I-INSTALL-1 handoff for every accepted band install, because its origin is in the future at install by construction (`k_wire < 0` means `t0 > t_inst + 75 ms`).

| tick | slide 0 (`t_inst = E - 80 ms`) | slide 1 (`E - 60 ms`) | at the cap (`E + 120 ms`) | one knot over (`E + 140 ms`) |
|---|---|---|---|---|
| E - 25 ms | record, last approach | record | record | record |
| E | REST knot 0 = record's terminal rest (0 step; knot-0 velocity 0.23 mm/s is the planner's own) | record terminal rest (held) | held | held |
| E + 25 ms | REST knot 1 (0.012 mm, 1.50 mm/s) | REST knot 0 (0 step) | held | held |
| E + 50 ms | knot 2 (0.083 mm, 4.34 mm/s) | knot 1 | held | held |
| E + 75 to 175 ms | knots 3-7 | knots 2-6 | held | held; the worker refuses `SPLICE_TOO_LATE` at `t_inst`, the record stays active, the Attempt ends as before BUG-95 |
| E + 200 ms | knot 8 | knot 7 | REST knot 0 (0 step), then knot 1 at +225 ms | held |
| REST end | E + 2.8 s | E + 2.825 s | E + 3.0 s | not installed |

The pre-BUG-95 code gives the same wire in the slide-0 column. It refused every other column `SPLICE_TOO_LATE`.

What the slid columns cost downstream: the PRE-TILT REST ends up to 0.200 s late, and the Feed CATCH, dispatched at its nominal end, splices into its last ≤ 8 knots. R5 sitting 2 flew that at +0.16 to +0.18 s. Whether `TILT_PIN` holds at +0.200 s is checked only by the Sitting below.

## Findings outside this Item (not fixed)

- A non-band fresh REST's REBASE is still uncapped. It slid up to +1.015 s in the logs, and +0.673 s ended an Attempt `TILT_PIN` at the Feed CATCH on 2026-10-28 (`python3_3246325_1793141374725.log`).
- A non-band at-rest REBASE still lands off the knot grid (`t_inst + 75 ms`). The 2026-10-23 entry accepted this ("bit-identical to the old rule, including the off-grid `t0`"), but the C2FF reasoning above says it streams off-knot for its whole window.

## Verification

All runs 2026-11-25, from the worktree, under `flock /tmp/jugglebot-tests.lock`, `nice -n 19 env OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 python -m pytest -p no:cacheprovider`:

- `-q -n 3 --dist loadfile` over the 2026-11-16 entry's 22 files plus `tests/motion/test_skills_human_catch.py` (a reader of `install_segment` that the grep found) and the doc readers `tests/sim/test_no_legacy_item_ids.py`, `test_logbook_front_matter.py` and `test_docs_links.py` (26 files): **1741 passed, 1 failed in 229.42 s**. The one failure was the first version of the malformed-terminal test: `RestTerminal`'s constructor refuses NaN. The test was rewritten to corrupt a built terminal to a non-number. Re-run `-q tests/motion/test_skills_executor.py tests/motion/test_magazine_throw.py`: **341 passed in 17.33 s**.
- The serial wall-clock readers, `-q tests/motion/test_check_segment_budget.py tests/motion/test_unified_cycle_budget.py`: **4 passed in 10.26 s**.
- `tests/sim/test_logbook_index.py` was not run. `logbook/INDEX.md` is regenerated by `python -m mapkit gen` at integration, and that test fails until then.
- **Mutations**, each with `executor.py` restored afterwards (`cmp` against the saved copy). Each run is `-k 'band or bug95 or slide or malformed or clamps or magazine_record'` over the two test files:
  - Cap set to 10**6: 3 fail (cap value, cap boundary, one over).
  - Grid snap removed: 1 fails (the off-grid record test).
  - Slide reverted to the generic `t_inst + 75 ms`: 7 fail (the slow-solve test, the whole-knot slide table, cap accepted, off-grid).
  - Decision reverted to the bare lead (`t_now + lead_s >= end_s`): 14 fail, including the MAGAZINE test.
  - `install_is_fresh` called outside the `try`: the malformed-terminal test fails with `ValueError: could not convert string to float` (run alone with `-k malformed`).
