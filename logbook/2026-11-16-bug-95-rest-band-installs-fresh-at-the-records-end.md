---
title: "BUG-95: a long REST whose running record ends inside its splice lead installs fresh at the record's end, not a clamped splice that ends the Attempt SPLICE_TOO_LATE"
type: bugfix
date: 2026-11-16
status: complete
phase: night-shift
subsystem: skill-stack
related_items: [BUG-95]
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - tests/motion/test_skills_executor.py
  - tests/ros/test_install_segment.py
---

# BUG-95: one lead for the fresh decision and the splice Knot

**Confirmed.** `install_segment`, `check_segment` and `trajectory_node` decided fresh with the bare `install_lead_s` (0.225 s for a REST), while `splice_knot_for` placed the Knot with the window-scaled `splice_lead_s`. A REST dispatched with the running record ending between the two leads spliced at the record's clamped last Knot on a budget of `end_s - t_now - 75 ms`, and a slow solve ended the Attempt `SPLICE_TOO_LATE`.

**Probe** (`/tmp/probe_bug95.py`, uncommitted): every REST of `compile_columns` (plain, Feed, hop entry; n = 1/3/8), `compile_rest_columns`, `compile_one_ball` (self, hop; lift 1.5/3.2/6.5 s), `compile_reload_wait`, `compile_reload_columns` and `compile_reload` after a `compile_reload_wait` bridge (the announcement swept over the bridge's last 1.2 s in 5 ms steps). Running record = the previous Segment (THROW/CATCH ends `rest_tail_s` after its last event, REST at its `t_abs_s`); `t_now` in `[dispatch_s - 50 ms, dispatch_s]` (skill_node look-ahead).

| Pattern | REST | window (s) | install lead | splice lead | end - t_now (s) | in band |
|---|---|---|---|---|---|---|
| columns / rest_columns / one_ball, all variants | closing | 0.300 | 0.225 | 0.225 | 0.175..0.225 | no |
| reload_columns (lift 1.5/3.2/6.5) | DECAY, closing | 1.000, 0.300 | 0.225 | 0.225 | 0.175..0.225 | no |
| Feed after bridge, lift 1.5 | PRE-TILT (0) | 1.500 | 0.225 | 0.225 | swept | no (0/250) |
| Feed after bridge, lift 3.2 | PRE-TILT (0) | 3.200 | 0.225 | 0.550 | swept | **yes**, 75/250: dispatch 0.18-0.55 s before bridge end |
| Feed after bridge, lift 6.5 | PRE-TILT (0) | 6.500 | 0.225 | 2.000 | swept | **yes**, 205/250: dispatch 0.18-1.2 s (to 2.0 s) before bridge end |

So the band is reached live by the Feed's PRE-TILT REST onto a bridge REST of about 2.2 s or more (a Hand far from rest: goal 13's lift was 3.2 s), whenever Ball Butler's announcement lands in the bridge's tail. Every compiled-in REST is fresh by construction.

**Fix.** `executor.install_is_fresh(record, kind, terminal, t_now, lead)` = `t_now + splice_lead_s(...) >= record.end_s`, read at all three sites; `splice_knot_for` already reads `splice_lead_s`, so a splice's Knot never clamps. For THROW/CATCH/MAGAZINE `splice_lead_s` is the bare lead, so their decision is unchanged. A REST in the band (`fresh_rest_waits_for_end`) gets origin `t0 = record.end_s` and the record's terminal-rest seed, which are the clamped splice's own origin and seed. The node skips the live seed and grid snap for it and arms the I-INSTALL-1 stream handoff.

**Not the Item's literal "fresh from now"** (decision, made at night): a fresh REST today starts at `t_now` from the live, moving state. Widening that from 0.225 s to up to 2.0 s before the running REST's end would pre-empt a still-moving Hand by up to 2 s, and in the in-motion REBASE branch `_rebase_fresh_rest`'s own docstring says the residual against the running record is unmeasured. Anchoring at the record's end gives the REBASE rule an exact-rest seed (the held branch) with no step.

*(Superseded slide rule: the REBASE described in this paragraph was replaced on 2026-11-25 by a whole-knot slide on the grid, capped at 8 knots; see `logbook/2026-11-25-bug-95-band-rest-slide-cap.md`.)*

**One control cycle (40 Hz), band case `t_now = end - 0.4 s`.** Before: the splice Knot was `n-1`, the cliff `end - 75 ms`, so 325 ms of budget, and a 600 ms solve gave SPLICE_TOO_LATE. After, each 25 ms tick: the emitter keeps sampling the running record, holding its terminal rest past `end_s` while the solve runs. A solve shorter than 475 ms installs `t0 = end_s` with the same plan the clamped splice would have streamed, so there is zero step at the switch. A longer one REBASES to `t_inst + 75 ms` (same motion later, the wire holds the terminal rest until then, zero step). No install starts earlier than today, and no budget that works today shrinks. Residual: a REBASE shifts the PRE-TILT REST's end later, which the Feed CATCH meets with the existing REBASED-pre-tilt handling (R5 sitting 2). That handling is unverified for a slide above about 0.2 s.

**Owner follow-up:** `motion/skills/INVARIANTS.md` (not edited, by the Night Shift rules) needs two text changes. The K1 row ("a REST SPLICE (never a fresh REST)") and I-INSTALL-1's last cell ("A fresh REST is exempt") should both say that a band REST is fresh, anchored at the record's end, and not exempt.

**Open after the audit** (named, not fixed; all five closed 2026-11-25 by `logbook/2026-11-25-bug-95-band-rest-slide-cap.md`, which also replaces this entry's slide rule with whole-knot slides):
- (a) A band REST's REBASE slide is bounded only by the solve deadline, so a slow solve can move the PRE-TILT REST's end, and the Feed Catch, later than the +0.16-0.18 s ever measured.
- (b) `install_is_fresh` runs outside the `try` that maps a malformed REST terminal's ValueError to UNREACHABLE.
- (c) A REST onto a MAGAZINE record in the band is now fresh instead of refused MAGAZINE_NEEDS_REST (untested).
- (d) The band REST skips the grid snap, so it starts off-grid if the running record was itself a REBASED fresh REST.
- (e) `INVARIANTS.md` K1 (line ~45) and I-INSTALL-1 (line ~39) now contradict the code and need the owner's amendment.

## Verification

2026-11-16, from the worktree, `night_shift_nonet.sh nice -n 19 env OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 python -m pytest -q` over the 22 files that call the install path (tests/motion: magazine_throw, skills_admissible_online, skills_bump, skills_capture_replay, skills_executor_learner, skills_executor_online, skills_executor, skills_install_origin, skills_rebase_continuity, skills_schedule, solver_worker, splice_tilt_rate_seam, validate_cycle; tests/ros: install_segment, levelling_frame, skill_node_check, skill_node, skills_plan_bench, trajectory_node, unified_cycle_levelling; tests/sim: sched_probe, skills_gate): **1650 passed in 390.66 s**. All six new tests failed with the source change reverted (node: `splice_k` 128, the clamped splice; executor: SPLICE_TOO_LATE, "from the splice at knot 128", missing `install_is_fresh`).
