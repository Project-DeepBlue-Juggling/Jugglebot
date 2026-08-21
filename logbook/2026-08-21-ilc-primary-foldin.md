---
title: Critical-point ILC becomes the primary toss learning architecture — the arc is unparked and folded onto the mainline
type: refactor
date: 2026-08-21
status: in-progress
files_changed:
  - plans/active/critical-point-ilc.md
  - plans/active/INDEX.md
  - logbook/INDEX.md
  - tools/probes/README.md
  - config/hardware_config.yaml
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py
  - ros_ws/src/jugglebot/jugglebot/motion/toss_ilc.py
  - tests/hardware/ilc_fit_lib.py
subsystem:
  - motion
  - ros
  - plans
tags:
  - ilc
  - toss
  - self-tuning
  - merge
  - architecture
related_plan: critical-point-ilc.md
---

# Critical-point ILC becomes the primary toss learning architecture

## Summary

The `critical-point-ilc` arc was parked on 2026-08-14 with Phases 0–2 complete,
audited and shipped dormant, behind two hard hardware gates. Both have since
closed — **G-1** (the bridge-latency-grows-with-uptime leak) on 2026-08-15 with
FW 14's IRQ-guarded ring pop (`9cd2bee`), and **G-2** (the hand ODrive's
asymmetric −10.00 A braking clamp) on 2026-08-18 (`b084f98`). On 2026-08-21 the
owner took the architecture decision the closures unblocked: **critical-point
ILC is THE primary toss learning law**, not a second opinion beside the layer-2
session trim.

This entry records the fold-in. Phase A (this section) is the mechanical half:
the arc branch catches up to the mainline, the plan moves onto the main
schedulable board, and the arc's resume prompt is retired.

## The decision being recorded

Six owner decisions, taken 2026-08-21, define what the fold-in means. They are
written here because none of them is recoverable from the code alone:

1. **ILC is always applied.** `JB_OP_TOSS_ILC_ENABLED` defaulting true is
   acceptable *because an absent artifact is exactly zero* — a machine with no
   `config/toss_ilc.yaml` is bitwise the machine we fly today. Every dormancy,
   provenance and clamp gate stays. The layer-2 trim's **aim** estimator demotes
   to **MONITOR-ONLY** (zero authority; it keeps observing and logging so
   divergence stays visible). Revert-to-trim only if ILC hits blocks.
2. **C1 (the common mode) is resolved inside ILC.** Session levelling noise is
   1.2–1.7 mrad/axis against per-cell |aim| of 9–10.5 mrad — 11–19 % of every
   persisted cell would otherwise be one session's `level()` draw. ILC gains a
   **session-local common-mode component** (RAM, discarded at goal end, its own
   evidence gate); the persisted per-cell artifact carries **only the spatial
   residual**. This transposes the home/anchor-mean referencing doctrine of
   `toss-selftuning.md` § 3.2 rather than inventing a new one. No persisted cell
   may absorb one session's levelling draw.
3. **Cadence target is R5-prime**: dwell 0.49 s at flight T 0.4949 s
   (~61 throws/min), reached via the census ladder R0→R5. The true 0.25 s dwell
   (R6) is a **deferred firmware fork** (calcCatch geometry) and is not being
   built. `MIN_TOSS_THROW_DELAY_S` retires as a floor; a ~0.1 s dispatch
   debounce plus derived state-based interlocks own the protection.
4. **SC-0…SC-3, the `toss_cal_grid` acquisition campaign and the `toss_fit_lib`
   per-node map fit are RETIRED as designed**, superseded by ILC. The decisive
   fact: `config/toss_calibration.yaml` **has never existed in either tree**, so
   the aim map was never captured and Phase 3's stated baseline ("aim map
   alone") was never constructible — the real baseline is *no correction*. The
   seams ILC rides are KEPT (`toss_cal.clamp_total_aim`, `aim_target_offset_mm`,
   the loader/dormancy patterns), as is any session/refusal machinery reusable
   for the ILC hardware runbook. Plans get supersession notes and board moves;
   nothing is deleted. **Still owed** — Phase A did the board move and the index
   rows only; the supersession notes inside `toss-selftuning.md`,
   `catch-robustness.md` and `critical-point-ilc.md` § D4 have NOT been written
   yet and belong to the build ladder.
5. **`k_v` lives in ILC** (`event_vel_trim`); the trim's `speed_gain` retires
   with the aim estimator. Release-latency `tau` stays trim-side, unwired, noted.
   `catch_timing_offset` lands as the first catch channel **only if** the seam
   and the sensor catch-event residual make it a clean gated addition —
   otherwise it stays declared-unimplemented with the residual definition
   written down.
6. **H2 (the fixtured-ball centroid capture) is RETIRED.** Owner pushback:
   conventional markers on the ball would corrupt the very trackable surface
   being measured, and the platform-frame occlusion reading is consistent with
   the arc's own data (bias large at z≈880 = catch-plane height, vanishing by
   z≈1880, parity-EVEN). So **C3 is resolved by decision, not measurement**:
   `arrival_dir` (whole-arc, bias-immune) becomes the PRIMARY and ONLY aim
   residual driving the ILC update; plane-level `land_err_x/y` demote to
   MONITOR-ONLY and are masked out of the aim update — never Q-arbitrated
   against `arrival_dir`. Absolute centering closes through **catch outcomes**;
   a ~10 mm registration bias against the 35 mm capture radius is tolerable and
   visible in the penalty trend. The per-toss channel-disagreement log is the
   standing replacement validation.

## Phase A — the refresh-merge (§ 4.5 build step 1, first half)

`origin/mvp-trajectory-bringup` merged into `critical-point-ilc`: **26 commits**,
merge base `e75badd` (the mainline commit that gate G-3's `712bcee` merged).
Three text conflicts, all in index/registry files; **no code conflict at all**.

- **`plans/active/INDEX.md`** — the ILC side still listed six plans the mainline
  has since parked or archived. Resolved to the mainline's board plus a single
  refreshed `critical-point-ilc.md` row. The prompt section gained the deletion
  note for the resume prompt.
- **`logbook/INDEX.md`** — date-ordered union of both sides' new rows. One row
  (`2026-08-12-odrive-fleet-reflash-tool`) existed on **both** sides and had
  diverged; the mainline's copy is a strict superset (it carries the same-day
  `FULL_CALIBRATION_SEQUENCE` addendum), so the mainline row was kept and the
  arc's dropped.
- **`tools/probes/README.md`** — resolved **row by row against the merge base**,
  not by side. `toss_record_miner.py` took the arc's text (the arc is what added
  the E-1 whole-arc estimators); `ball_arrival_offset.py`,
  `hand_stroke_timeline.py`, `displaced_reach_frontier.py` and
  `link_status_health_scan.py` took the mainline's (mainline-only edits);
  `hand_decel_authority.py` was identical. The arc's three new probes
  (`hand_contact_softness.py`, `hand_sensor_settle.py`, `mocap_parity_bias.py`)
  were added; the `canbridge_reboot_latch_probe.py` row was **dropped**, because
  the mainline archived that probe file and re-homed its row under `archived/`.

`config/hardware_config.yaml` auto-merged to exactly *mainline + the arc's one
`toss_ilc_enabled` key*, and re-running `python config/generate_config.py`
produced **no change** to any of the six generated artifacts — the auto-merge
was already codegen-exact, which is also the determinism proof.

`reload_coordinator_node.py` was the one file the digest flagged as a real
conflict, and git auto-merged it. It was checked by hand rather than trusted:
the two sides' edits are *adjacent, not overlapping* in the `TossSequencer`
construction — the arc rewrites `event_vel_mps=` to carry the ILC speed trim,
the mainline inserts `workspace_xy_mm=` two lines below. Both are present and
correct in the merged file. The mainline's removal of the `TOSS_XY_LIMIT_MM`
import from that node is also safe: the constant still lives in
`toss_sequencer.py`, which is where `ilc_fit_lib` imports it from.

## Discussion — what was deliberately NOT fixed here

This was scoped to merge fallout. Three things the merge legitimately surfaces
were left for the § 4.5 build ladder, and they are named here so the next
session does not mistake silence for absence:

- **The flight-time band widened for free, and that is correct.**
  `ilc_fit_lib.admit_command` imports `FLIGHT_TIME_MIN_S/MAX_S` from
  `toss_sequencer`, which now derives them from `throw_envelope`
  (`[0.4949, 1.1485]` s, replacing the hand-picked `[0.55, 1.10]`). The
  exact-gate re-validation therefore picked up the derived envelope with **zero
  code change** — design constraint 1 (never a symbolic twin) paying off exactly
  as intended.
- **`TOSS_XY_LIMIT_MM` is now a stale mirror.** `ilc_fit_lib`'s workspace gate
  still uses the module constant `150.0`, while the production sequencer now
  takes `workspace_xy_mm` from `hw.JB_OP_TOSS_WORKSPACE_XY_MM` (`160.0`). This
  is a fidelity gap, **not a safety one** — 150 is strictly tighter, so the fit
  can only refuse steps the machine would have allowed, never admit one it would
  refuse. Fixing it is a design call about which value the fit should mirror,
  which belongs to build step 1, not to a merge.
- **C2 is real but still latent, and the rail-sweep test correctly still
  passes.** `test_the_event_vel_band_is_unreachable_inside_the_speed_authority`
  was predicted to fail; it does not, and that is right. The node's apply seam
  gates only on `validate_event_vel` (the bridge's `[0.3, 7.0]` m/s wire band),
  and the merge did not add the `throw_envelope` gate — that *is* build step 1.
  The contradiction (±0.15 speed authority is inadmissible near both ends of the
  derived flight-time band, and `+0.000` at `T = 0.4949` s, which is exactly the
  R5-prime cadence target) bites the moment the envelope gate lands. **Decisions
  3 and 5 above collide here**: the chosen cadence sits precisely on the flight
  time where the negative speed-trim authority goes to zero.

`toss_ilc_enabled` is deliberately left **`false`** in this merge. Decision 1
makes `true` acceptable, but flipping it is feature work with a named tripwire
(`tests/motion/test_toss_ilc.py::test_shipped_config_has_the_feature_off`), and
a merge commit is the wrong place to arm a learned correction.

The arc's resume prompt `PROMPT-critical-point-ilc-resume.md` is **deleted**,
per the owner's 2026-08-09 convention that a completed prompt's arc lives in the
logbook. It was written to resume a *parked* arc and was superseded the moment
the arc was unparked; its Done-means list is answered by the plan and by this
entry. (It was found already moved to the worktree root, uncommitted and
byte-identical to its committed blob; nothing was lost.)

## Verification

- Worktree gate on the resolved merge (before this entry was written), run
  2026-08-21: `./run_tests.sh` — **5576 passed, 5 skipped, 0 failed in
  260.38 s** (RESULT: PASS). Green first try; no code fallout to fix.
- Worktree pre-commit gate on the final tree (merge + board + this entry), run
  2026-08-21: `./run_tests.sh` — **5576 passed, 5 skipped, 0 failed in
  249.77 s** (parallel 252 s; the gate's serial phase is empty by design, 6016
  deselected; total 262 s, RESULT: PASS).
- Codegen determinism, run 2026-08-21: `python config/generate_config.py`
  followed by `git diff --stat` on the generated set — **no change**.
- Scoped confirmations, run 2026-08-21:
  `pytest tests/sim/test_plans_index.py -q` — **69 passed in 0.16 s**;
  `pytest tests/motion/test_ilc_fit.py -q -k "event_vel_band or unreachable or
  speed_authority"` — **2 passed, 48 deselected in 0.24 s**.
