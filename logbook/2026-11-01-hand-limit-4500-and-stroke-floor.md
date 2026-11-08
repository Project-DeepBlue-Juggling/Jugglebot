---
title: "Hand limit 3900 to 4500 rev/s^2 at 60 A, HAND_STROKE between-knot rows (I-PLAN-15), bridge gap instrumentation and trajectory observability"
type: investigation
date: 2026-11-01
status: in-progress
phase: "pattern progression"
related_plan: pattern-progression.md
related_issues: []
files_changed:
  - config/hardware_config.yaml
  - config/generated/hardware_config.py
  - config/generated/hardware_config.h
  - ros_ws/src/jugglebot/jugglebot/hardware_config.py
  - ros_ws/src/jugglebot/Teensy_code_canbridge/hardware_config.h
  - ros_ws/src/jugglebot/Teensy_code_platform/hardware_config.h
  - ros_ws/src/jugglebot/CatchingCone_code/hardware_config.h
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/cup_cycle.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/feasibility.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/throw_envelope.py
  - ros_ws/src/jugglebot/jugglebot/motion/unified_cycle.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/report.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - ros_ws/docs/hand_throw_envelope.md
  - sim/skills_gate.py
  - teensy_link/gap_monitor.py
  - tools/probes/bump_seed_replay.py
  - tools/probes/README.md
  - tests/motion/test_cup_cycle.py
  - tests/motion/test_magazine_throw.py
  - tests/motion/test_skills_executor.py
  - tests/motion/test_throw_envelope.py
  - tests/motion/test_validate_cycle.py
  - tests/motion/test_validate_cycle_vectorised.py
  - tests/ros/test_teensy_bridge_node_bridge_diag.py
  - tests/ros/test_teensy_bridge_node_hand_jam.py
  - tests/ros/test_teensy_bridge_node_read.py
  - tests/teensy_link/test_gap_monitor.py
  - config/generated/admissible_box.yaml
  - tests/fixtures/admissible_box_pre_h4500.yaml
subsystem: [motion, ros, config, tools, sim]
tags: [safety, performance, kinematics, dynamics, testing, docs]
---

# Hand limit 4500 at 60 A, and the HAND_STROKE between-knot floor

Code half of `2026-11-01-sitting-debrief.md`, which attributed the three largest end classes of the Sitting (HAND_STROKE 7, HAND_LIMIT_ACC 6, and the starved-bridge latches) to causes other than the Items that tracked them. Items: BUG-49 (fix landed), BUG-42 (decision), BUG-34 and BUG-50 (instrumented), BUG-4.

## Summary

| Change | What | Evidence |
|---|---|---|
| HAND_STROKE | the planner bounds the same 4-per-knot samples the gate checks: floor rows always, ceiling rows opt-in and OFF in production (I-PLAN-15) | hop-start lowest point 0.0078 -> 0.0202 rev |
| Hand limit | `hand_acc_limit_rps2` 3900 -> 4500, `hand_curr_limit_a` 50 -> 60 A | decel-FF headroom bound 4710.6 at 60 A |
| Instrumentation | forward and loop gap in the bridge, `/link_status` fields, one WARN per episode | 3 new bridge tests, 6 gap_monitor tests |
| Observability | peaks on every accepted install line, one more digit in LIMIT_* text, identical-aim fallback skipped, `bump_seed_replay.py` | 6 of 6 replayed peaks within 2 % of the log |
| Box | re-swept at 4500 | 82 cells (82 before), 32 changed vs the pre-tonight box, gate_hash 15c6d0ff8a84; hop cells traded landing x for apex band (BUG-54); hop cells traded landing x for apex band (BUG-54) |

## Discussion

**Why raise the limit instead of fixing the bump seed (accepted tradeoff).** All six HAND_LIMIT_ACC ends were first installs seeded from a BUMPED predecessor with a span-1 peak of 4000-4100 (BUG-42). The structural fix (a bump that does not hand the successor a hot seed) is filed, not built. The owner decided to raise the limit to the decel-feedforward headroom bound. The runway `v^2/(2a)` shrinks by 1/1.154, so the QP can brake later and harder, which is what admits the bump-seeded successor. The cost is a higher authority demand on the Hand (below) and the loss of REGEN margin in the throw envelope. The seed defect is still there; the limit now hides it with 4500 vs 4100 of margin rather than removing it.

**Why 60 A.** At 4500 the ball-free hand needs 0.297 N m = 53.8 A (42.4 A from the torque feedforward, about 11.4 A from the loop). That cannot fit under 50 A, so the limit and the current go up together. The owner was open to 60-80 A; 70-80 A is not available because the drive's `current_hard_max` is 70 A, so soft = hard trips on any overshoot (BUG-51 measured 53.3 A against the 50 A soft limit). 60 A leaves a 10 A margin.

**Why the HAND_STROKE rows sample the stroke level, not the box floor.** Every refusal was the hop-start skill-2 CATCH in the middle of a 25 ms span: the cup QP bounded the slider at knots 1..n only, while the gate samples the continuum. Rebuilt offline, the knot minimum sat on the 0.0614 rev box floor while the cubic at t = 0.488 s dipped to 0.0078 rev (a*dt^2/8 at about 680 rev/s^2); the hardware seed went 0.012 further, to -0.004.

| Alternative | Why not |
|---|---|
| Gate tolerance only (0.01 rev) | hides the dip instead of bounding it; kept only as a stopgap for in-band plans (`HAND_STROKE_FLOOR_TOL_REV`), parked floors keep the 1e-4 tolerance |
| Larger knot inset | first try held interior samples at the BOX floor (0.0614 rev): it bound most oval rung-1 carries and moved 13 characterisation pins (seating cone 34.7 -> 30.1 deg, contact floor 0.254 -> 0.207 g) for no refusal avoided |
| Half-knot samples only | h = 12.5 ms sags 0.076 rev at 3900, more than the 2 mm inset; the gate's pass-2 check is closed-form, so the planner needs the rigorous quarter-mesh bound (0.019 rev at 3900, 0.022 at 4500) |

Final design: knots keep the box insets; interior samples are held at the stroke bottom (or top) plus or minus `a*(dt/4)^2/8`, never inside the knot bound. The rows are linear columns on the existing jerk variables, so the QP stays convex (H and the variable count are unchanged). They are slack on every plan clear of the stroke level and the active-set solver never adds a slack row, so those plans come out bit-identical. Span 0 is skipped (its one free variable would make the row judge the seed). The ceiling mirror (stroke top = catch prime 10.45 rev, sag 0.022 rev at 4500) needs each inset >= sag; the 10 mm and 2 mm insets exceed 0.72 mm. With a 0.5 mm inset the rows-on overshoot was +0.002 rev. The ceiling rows exist in code but are opt-in (below).

**Why the ceiling rows are off in production (added 2026-11-02, after the first `--full`).** The first `./run_tests.sh --full` (08:17-08:24, 7947 passed, 3 failed + 1 serial failed) failed the owner pose budget, `test_pose_mode_steady_fits_the_same_budget`: pose core 50.4 ms > 50.0. Attribution on the quiet box, then the pre-registered criterion (keep the ceiling only if floor+ceiling core <= 46.0 ms):

| Variant | Pose core (ms) |
|---|---|
| no rows | 43.6-44.7 (42.67 min after vectorising) |
| floor only, per-row loop | 47.7-48.4 |
| floor + ceiling, per-row loop | 50.2-50.5 |
| floor + ceiling, vectorised | 47.75 min |
| floor only, vectorised | 45.59 min |
| production after (floor only) | 45.81-46.01 |

Vectorising removed about 2/3 of the cost (profile: per-row loop 2.4 ms, hstack/normalisation 1.3 ms; the remaining solve +1.9 ms is `C.T @ x` over 360 more columns, inherent). Criterion not met (47.75 > 46.0), so the ceiling is opt-in (`stroke_z_top_m` is not passed by `build_cup_config`). Reason it is safe to drop: no swept or flown top reversal comes near the prime (0.02-0.13 rev above the ceiling knots, >= 0.17 rev under the prime), the 10 mm knot inset covers the sag to 3930 rev/s^2, and beyond that the gate refuses (the safe side). The first sweep (gate_hash dd4ae01a93a0) predates these gated edits and is superseded.

## Fix

### 1. HAND_STROKE (I-PLAN-15)

| Item | Value |
|---|---|
| `Z_BOX_SAMPLES_PER_KNOT` | 4, equal to the gate's `_CYCLE_SAMPLES_PER_KNOT` (pinned by test); 1 disables both blocks |
| Floor rows | `z_span_lower`, tagged `pzm`, moved onto the slider in `_pose_augment` like the knot rows |
| Ceiling rows | `z_span_upper` after the floor block, only when `CupCycleConfig.stroke_z_top_m` is set; `build_cup_config` does NOT pass it (OFF in production) |
| Row assembly | vectorised: `_z_span_rows` (one broadcast) and `_slider_rewrite` (per block), bit-identical to the old per-row code (`test_the_between_knot_rows_are_bit_identical_to_the_per_row_loop`, 3 windows; mutation check: a 1-ulp perturbation fails all 4 runs) |
| Gate tolerance | `HAND_STROKE_FLOOR_TOL_REV = 0.01`, in-band floors only |
| Hop-start window (pose, apex 0.95, sep 125, dwell 0.27) | continuum minimum 0.0078 -> 0.0202 rev, quarter minimum 0.0079 -> 0.0220; knots unchanged 0.0614; peak hand acc 3421.7 -> 3421.6 rev/s^2 |
| QP time (n = 23, BLAS 1 thread, min of 8) | 3.34 / 5.17 ms -> 3.44 / 4.52 ms (floor); whole `plan_segment` 32.0 -> 33.1 ms with floor + ceiling (first draft; production is now floor only, pose core 45.81-46.01 ms) |
| Ceiling reachable? | no natural case: knot tops are the post-release brake at 400-1700 rev/s^2, at least 0.17 rev under the prime; residual beyond 3930 rev/s^2 is refused by the gate |

Residual: the tilt caveat (cup z vs slider, up to about 0.003 rev past the prime in a contrived worst case at the full cap). The exact hardware seed was not replayed (the launch log does not print the splice state); the regression test uses the reconstructed window.

### 2. Hand limit and current

| Quantity | 50 A | 60 A |
|---|---|---|
| Decel-FF headroom `0.85*I*Kt/(2 pi J_ff 9.5e-6)` | 3925.5 | **4710.6** (4500 = 95.5 %) |
| Full-current line `I*Kt/(2 pi J 1.05e-5)` | 4178.4 | 5014.1 |
| Ball-aboard ascent authority | 3225.2 | 3897.3 |
| Ball-free current at the session limit | 46.7 A @3900 | 53.8 A @4500 |
| Ball-aboard ascent current at the session limit | 60.0 A @3900 | 69.0 A @4500 |
| Throw-envelope release speed bound | 5.720 m/s (FF headroom) | 5.641 m/s (REGEN) |
| Apex ceiling | 1.665 m | 1.619 m |

Behaviour changes:
- REGEN now binds the throw envelope (flat-out brake against the unchanged 360 W), so a higher current limit lowers the ceiling 5.720 -> 5.641 m/s. No flown or planned Pattern is near it (columns 0.85-1.10 m, ceiling Apex 1.42); END_STOP stays last. Fencing the commanded rather than the limit torque would be a C-HAND-3 contract change, not made.
- The magazine (Feed-back) throw ascent follows the authority 3225 -> 3897 (about 21 % harder).
- Hand-jam trigger `0.9 x curr_limit`: 45 -> 54 A. A stalled hand now heats at up to 60 A for `sustain_s`.
- `sim/skills_gate.py` `_SESSION_HAND_ACC_RPS2` now derives from the config (was a literal 3900).
- The current is live at the next configure (`DEFAULT_VEL_CURR['hand_curr']`): no flash, no ODrive save.

Hardware facts and open decisions:

| Fact | State |
|---|---|
| Hand ODrive `current_hard_max` | 70 A; 60 A leaves 10 A; 70-80 A needs the drive config raised first |
| Hand motor rated/peak current | undocumented in the repo; demagnetisation peak unconfirmed |
| ODrive backup JSON | still `current_soft_max 50` (a transcription configure overrides; not edited) |
| `stream_stop_hand_accel_rps2` 3500 | left: stops at max(3500, entry accel), so a 4500 entry is honoured; changing it needs a flash |
| Firmware FF clamp 0.234 N m | left; reaching 0.85*60*Kt = 0.281 needs a flash |
| Ball-aboard ascent wants 69 A vs 60 A | BUG-40 gap still open (BUG-41 measured brake currents about half the model, so probably pessimistic) |

### 3. Instrumentation (BUG-34, BUG-50)

- `teensy_link/gap_monitor.py`: `GapTracker` (rolling 1 s max and p99, idle cutoff), `EpisodeLatch`, `GapStats`, `SCHED_COVER_MS = 70` (2x25 + 20; the only Python mirror, firmware source not in this checkout).
- `/link_status` gains `max_forward_gap_ms`, `forward_gap_p99_ms`, `max_loop_gap_ms` (1 s window, 10 Hz).
- WARN once per episode when the forward or loop gap exceeds the 70 ms cover.
- The STOP warning names both candidates (emitter gap vs bridge forwarding gap) and prints both maxima. The loop gap splits them: loop gap about forward gap means the bridge stalled; loop gap small means the stream was late.
- The heartbeat census prints the encoder age max (a lower bound; the 1 Hz cache_diag window can land after the clearing edge).
- Recording costs a lock and a deque append; no I/O in the setpoint loop.

### 4. Observability

- Every accepted `install_segment` debug line carries `peaks hand acc N rev/s^2 (P% of limit), leg vel V% acc A%` (worst leg per quantity; per-leg peaks would need new sampling).
- LIMIT_* messages print one more digit (leg `.1f` -> `.2f`, hand acc 1 -> 2, hand vel 2 -> 3; skill END lines `%.2f` with zeros stripped).
- The executor skips the identical-aim fallback (`np.allclose`, atol 1e-6): same problem, same refusal; only the `CATCH-AIM-FALLBACK` log line is absent.
- `tools/probes/bump_seed_replay.py` promoted (launch log in, `temp/probes/bump_seed_replay/<ts>/replay.csv` out). Run 2026-11-01 on the Sitting log: 25 BUMP lines, 6 replayed, **6 of 6 peaks within 2.0 % of the log**. The two Ball 1 cases replay at 99 % of 3900 (no offline refusal): the peak reproduces, the verdict at the margin does not.

## Verification

| Date | Command | Result |
|---|---|---|
| 2026-11-01 | `OPENBLAS_NUM_THREADS=1 python -m pytest tests/motion/ -q -m "not nightly" -n 3` (unit F, before the 4500 and precision pins landed) | 11 failed, 2752 passed, 2 skipped in 702.78 s; all 11 = stale box, precision-pin strings, magazine 5.3 pin, or load-bound serial budget tests |
| 2026-11-02 | `OPENBLAS_NUM_THREADS=1 python -m pytest tests/motion/test_validate_cycle.py tests/motion/test_validate_cycle_vectorised.py -q -p no:cacheprovider` (precision pins) | 94 passed in 21.59 s |
| 2026-11-02 | `pytest tests/motion/test_validate_cycle.py ... test_cup_cycle.py test_pose_planner.py test_cup_cycle_row_labels.py -q -p no:cacheprovider` (ceiling unit, before the pins) | 4 failed (the precision strings the pins fixed), 211 passed in 14.16 s |
| 2026-11-02 | `pytest tests/motion/test_unified_cycle.py test_unified_cycle_splice.py test_skills_segments.py test_skills_oval.py test_skills_executor.py -q -n 3 -m "not nightly"` | 457 passed, 2 skipped in 36.96 s |
| 2026-11-01 | `pytest tests/teensy_link/test_gap_monitor.py tests/ros/test_teensy_bridge_node_bridge_diag.py tests/ros/test_teensy_bridge_node_read.py -q` | 85 passed in 16 s |
| 2026-11-02 | `pytest tests/ros/test_teensy_bridge_node_hand_jam.py tests/motion/test_magazine_throw.py tests/motion/test_throw_envelope.py` + the two re-pinned validate_cycle/cup_cycle tests `-q` | 62 passed |
| 2026-11-02 | targeted 15-file run of every reader of a changed value (`-n 4 -m "not nightly"`) | 32 failed, 1192 passed in 1496.93 s; 26 of the 32 were the stale-box LimitsMismatch the re-sweep clears, the rest the precision strings since fixed |
| (sweep) | `tools/admissible_sweep.py --hand-acc 4500`, second full two-round sweep after the gated edits (first sweep, 2026-11-02 00:40-02:41, superseded) | 2026-11-02 08:41-10:4x AEDT, after the final gated edits: all 26 families IDENTICAL across rounds, `CMP_STATUS 0`, loader accepts 82 cells, gate_hash 15c6d0ff8a84 = live; all 82 cells equal the superseded first sweep (dd4ae01a93a0) |
| 2026-11-02 | `./run_tests.sh --full` (first, 08:17-08:24) | 7947 passed, 3 failed + 1 serial failed; the pose budget at 50.4 ms > 50.0 (above) |
| 2026-11-02 | `OPENBLAS_NUM_THREADS=1 python -m pytest tests/motion/test_cup_cycle.py test_validate_cycle.py test_validate_cycle_vectorised.py test_pose_planner.py test_cup_cycle_row_labels.py test_unified_cycle_budget.py test_unified_cycle.py test_skills_segments.py -q -p no:cacheprovider` (floor-only, ceiling opt-in) | 351 passed, 2 skipped in 34.24 s |
| 2026-11-02 | box-history tests re-anchored on the frozen `tests/fixtures/admissible_box_pre_h4500.yaml` (no assertion loosened) plus `test_h4500_resweep_kept_cells_and_columns_values_and_changed_only_hand_acc` | in the gate run below |
| (gate) | `./run_tests.sh --full` | 2026-11-02, on the commit merged onto origin/main (Night Shift FEAT-7/FEAT-35 included): **PASS**, parallel 7961 passed, 53 skipped in 386.22 s; serial 8 passed in 30.61 s (total 423 s). The first merged run failed FEAT-7's new `test_a_biased_plant_fires_the_forward_clip_in_a_small_hop_run`: the sim's hop cell widened to +/-20 mm, so a +30 mm bias no longer leaves the box; the test now runs on `tests/fixtures/admissible_box_pre_h4500.yaml` (frozen geometry) and reproduces its original clip exactly |

Not sourced: a clean-HEAD A/B of the bridge tests (no stash allowed in the shared tree); the serial budget tests (`test_unified_cycle_budget.py`, `test_validate_cycle_budget.py`) failed at load 24 under xdist and must be judged from the serial tail of the gate run; the offline sim `skills_gate.py --learn` run (PASS 3/3) predates the fix and was not re-run.

## Outcome

Landed, awaiting the sweep and gate results above. Not yet shown on hardware: that 4500 at 60 A admits the bump-seeded successor in flight, that no HAND_STROKE end recurs, and the ball-aboard ascent current (69 A wanted vs 60 A, BUG-40). Hand-jam and the 55 A watch (IDEA-10, BUG-51) are the first things to read after the next Sitting.

## Open Questions

- **BUG-54, the hop cells (owner decision).** The re-swept box keeps 82 cells; 32 changed. Columns, turned and high-low cells are unchanged or wider. The hop cells gained apex band at the bottom, and the apex 0.92-0.97 hop cell (the one Columns at 0.95 flies) shrank in landing x from +/-10 to +/-4 mm. Cause: `_flight_band_and_rect` grows the band while the zero-offset throw passes, then fits one rectangle over the whole band; at 4500 the 0.65-0.73 s flights pass at zero offset but fail HAND_LIMIT_C2 at large x. At the operating flights every grid offset passes, so it is a selection artefact.
- **Audit residue accepted (2026-11-02, `/audit --unstaged`, no blocking findings).** The gap tracker builds its percentile under the lock (about 50-100 us against a 70 ms cover); a 70 ms to 1 s pause between goals can raise one advisory gap WARN. Applied: the executor fallback-skip comment says "normally identical", the probe constant is labelled as the Sitting-time cap, and `_peaks_suffix` has a test.

- Stale 3900 claims owned elsewhere: `plans/active/two-ball-oval.md` line 72 (dated history, kept; line 557 was corrected in 2e35db2d); `sites.py` L54 notes release plane 880 needed 4416 rev/s^2, now under 4500, so the refused release-plane raise may reopen.
- Tests with explicit `hand_acc_rps2=3900.0` that touch the box (`test_skills_executor.py` `limits_r5`, `test_skills_admissible.py:1659`, `test_install_segment.py:2151`, others) may need the live value after the sweep; re-pin, never loosen.
- Owner/flash decisions: FF clamp 0.234 -> 0.281 N m; raising `current_hard_max` before any soft limit past about 65 A; the ODrive backup JSON; the hand motor datasheet peak.
- The ceiling rows are dormant: reopen if a top reversal above 3930 rev/s^2 is ever planned (the gate refuses it today) or if the pose core budget gains >= 2 ms of room.
- The bump seed itself (BUG-42 structural fix) is filed, not built.
- Colcon rebuild needed for `ros_ws` (`cd ros_ws && colcon build --packages-select jugglebot`); `teensy_link` is live-tree.
