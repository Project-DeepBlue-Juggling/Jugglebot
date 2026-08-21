---
title: Critical-point ILC becomes the primary toss learning architecture — the arc is unparked and folded onto the mainline
type: refactor
date: 2026-08-21
status: resolved
files_changed:
  - plans/active/critical-point-ilc.md
  - plans/active/toss-selftuning.md
  - plans/active/catch-robustness.md
  - plans/active/INDEX.md
  - logbook/INDEX.md
  - tools/probes/README.md
  - tools/probes/ilc_speed_band.py
  - config/hardware_config.yaml
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py
  - ros_ws/src/jugglebot/jugglebot/motion/toss_ilc.py
  - ros_ws/src/jugglebot/jugglebot/toss_trim.py
  - ros_ws/src/jugglebot/jugglebot/toss_record.py
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

This entry records the fold-in in three phases. **Phase A** is the mechanical
half: the arc branch catches up to the mainline, the plan moves onto the main
schedulable board, and the arc's resume prompt is retired. **Phase B** merges
the refreshed arc onto `mvp-trajectory-bringup`. **Phase C** writes the
decisions through the plan surface — the supersession map, the contradiction
ledger, the re-frozen Phase-3 criteria and the cadence census — and carries the
evidence the decisions rest on.

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
   nothing is deleted. *(Phase A did the board move and the index rows only; the
   supersession notes inside `toss-selftuning.md`, `catch-robustness.md` and
   this plan's Phase 3 were **written in Phase C below**, so this obligation is
   discharged.)*
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

## Phase B — the fold-in onto `mvp-trajectory-bringup`

With the arc branch refreshed, `critical-point-ilc` merged into
`mvp-trajectory-bringup` on the main tree. Because Phase A made the arc branch a
strict superset of the mainline, this was fast-forwardable; it was taken as a
**real merge commit (`--no-ff`)** anyway, so the fold-in is a single identifiable
point in history rather than a silent replay of 37 commits.

**Zero conflicts.** The delta onto the mainline is exactly the arc and nothing
else — 33 paths:

- the plan `plans/active/critical-point-ilc.md` and its row on the main board;
- 5 logbook entries (the 4 arc entries plus this one) and the `logbook/INDEX.md`
  rows;
- `ros_ws/.../motion/toss_ilc.py` (the layer-3 artifact + loader) and the one
  apply seam in `reload_coordinator_node.py`;
- `tests/hardware/ilc_fit_lib.py` + `ilc_fit.py` (the offline fit core and CLI);
- 4 test files (`test_ilc_fit.py`, `test_toss_ilc.py`, `test_toss_ilc_node.py`,
  `test_ilc_measurement_probes.py`) plus edits to 3 existing ones;
- 3 probes (`hand_contact_softness.py`, `hand_sensor_settle.py`,
  `mocap_parity_bias.py`) and the miner's E-1 whole-arc estimators;
- `config/hardware_config.yaml`'s `toss_ilc_enabled` key and the six regenerated
  artifacts.

`python config/generate_config.py` re-run on the main tree produced **no change**
to the merged generated artifacts, confirming determinism on this tree too.

⚠ **Cross-repo side effect worth knowing about**: `config/generate_config.py`
writes unconditionally into the sibling **BallButler** repo
(`~/Desktop/BallButler/ball_butler_main/hardware_config.h`). Running it from
*any* Jugglebot tree silently dirties that repo — and running it from a feature
worktree will inject that feature's keys into it (here, `TOSS_ILC_ENABLED`).
Both regen runs were reverted there with `git checkout`, so BallButler is
untouched by this work. Separately: BallButler's committed header is genuinely
**stale** against the current Jugglebot config (it still carries the old
`HAND_STROKE_MM = 355.0`, `HAND_MOTOR_MAX_POSITION_REVS = 11.1`, tier `8a`-era
`CATCH_VEL_SCALE_DEFAULT` and `TOSS_REQUIRE_BALL_EVIDENCE`), which is a real
finding for that repo's owner and deliberately not fixed from here.

## The one real bug the fold-in surfaced: a presence-only corpus guard

The `--full` gate on main failed one test —
`tests/motion/test_ilc_fit.py::test_cli_validate_runs_all_four_validations_and_exits_zero`
— and it was **not** a flake. It is `nightly`-marked, so neither tree's default
gate had ever run it; the arc's own last full gate ran in the worktree, where
the environment happened to be favourable.

**Mechanism.** These tests read a mined corpus from `temp/probes/`, which is
gitignored and therefore *per-worktree*. The ILC worktree holds corpora mined
2026-08-12/13 by the arc's own extended miner. The main tree holds one mined
**2026-08-11 by the PRE-E-1 miner** — 31 rows that carry no
`usable_for_release_fit` field, so every one is refused by the miner's own gate,
`fit_corpus` REFUSES with "no admitted rows in this corpus" (**correctly**), and
the CLI exits non-zero. The test asserted exit zero, so it failed.

**The actual defect is a guard inconsistency inside one file.** The `corpus`
fixture was already admission-aware — it skips below 10 admitted rows, which is
why 11 sibling tests skipped cleanly on main. The two CLI tests bypassed the
fixture and guarded on `_corpus_paths()`, i.e. on **file presence** alone. Two
different answers to "is there a corpus here?" in the same module.

The mirror case is worse than the failure, and is why this was fixed as a class
rather than patched: `test_cli_refuses_a_cross_partition_fit_without_the_flag`
asserts `rc == 1` and **passed on main for the wrong reason** — the CLI returned
1 for want of admissible rows, not for the cross-partition pooling the test
exists to pin. A vacuous pass, the same failure mode the arc's own Phase-2 audit
already caught once in the byte-identical-OFF test.

**Fix** (test-only): one shared `_corpus_or_skip_reason()` helper that answers
"does this tree hold a corpus the fit can actually use?", used by the fixture
and by both CLI tests, with a skip reason that names the pre-E-1-mine cause.
Verified to discriminate rather than to blanket-skip:

- main tree (stale pre-E-1 corpus): `pytest tests/motion/test_ilc_fit.py -q`
  → **37 passed, 13 skipped** — the two CLI tests now skip honestly;
- ILC worktree (good corpus), same file copied in: `-q` → **50 passed, 0
  skipped**, and the two CLI tests **run and pass** rather than skipping.
  The worktree was restored to its committed state immediately afterwards.

This is C8 ("the evidence for the headline numbers has no committed provenance")
biting in its cheapest possible form. It does not close C8 — a corpus fixture
with committed provenance still does not exist — but it does stop a stale
gitignored directory from deciding whether the suite is green.

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
- Main-tree gate at plan-phase closure, `mvp-trajectory-bringup` with the
  fold-in merge staged, run 2026-08-21: `./run_tests.sh --full` — **1 failed,
  5992 passed, 11 skipped, 3 xfailed in 509.12 s** (serial phase 9 passed;
  RESULT: FAIL). The single failure was the presence-only corpus guard
  described above, diagnosed and fixed in this session.
- Main-tree gate after the guard fix, run 2026-08-21: `./run_tests.sh --full` —
  **5991 passed, 13 skipped, 3 xfailed, 0 failed in 498.66 s** parallel, plus
  **9 passed, 6007 deselected in 41.53 s** serial; total 547 s, **RESULT: PASS**.
  The delta against the failing run is exactly the two guarded tests: one
  failure and one vacuous pass both became honest skips (5992→5991 passed,
  11→13 skipped), and nothing else moved.
- For scale, the pre-fold-in mainline nightly of the same morning
  (`temp/reports/nightly/status`, 2026-08-21T04:03): **GREEN 5738/5741 passed**.
  The fold-in adds 156 new test functions across four ILC test files.
- Codegen determinism, run 2026-08-21 on **both** trees:
  `python config/generate_config.py` followed by `git diff --stat` on the
  generated set — **no change** on either.
- Scoped confirmations, run 2026-08-21:
  `pytest tests/sim/test_plans_index.py -q` — **69 passed in 0.16 s**;
  `pytest tests/motion/test_ilc_fit.py -q -k "event_vel_band or unreachable or
  speed_authority"` — **2 passed, 48 deselected in 0.24 s**.

## Phase C — the decisions rippled through the plan surface

Phases A and B moved the code and the board. This phase writes the decisions
into the three plans they change, so that a future session reading the plan
surface alone reaches the same conclusions without this entry.

| Document | What landed |
|---|---|
| `plans/active/critical-point-ilc.md` | Status → **THE PRIMARY** learning architecture; a new **§ The 2026-08-21 fold-in** carrying the six decisions with the failure each one prevents, the **C1–C8 contradiction ledger** with each resolution and where it lands, C1's design spelled out in three parts, **two derived consequences** (below), and the five-step build ladder. Phase 3 **re-frozen** against a no-correction baseline with five pre-registered criteria (P-1…P-5). H2's retirement written where the E-1 caveat proposed it. Risk 3 sharpened — it was live, in both directions, and provenance keys were never going to catch it |
| `plans/active/toss-selftuning.md` | A **SUPERSESSION NOTICE** at the top: what retires *as an update law* (the § 3.6 aim estimator → monitor-only, `speed_gain` → ILC, § 3.7's never-captured map, § 3.8's SC-0…SC-3, § 5's 2c per-node fit), what is **retained as substrate** (the record, the miner, G1–G11, the CUSUM/freeze machinery, auto-reload, Layer 1.5, the D7 clamp, § 3.2's doctrine), and what § 3.2 becomes under ILC. Plus a new **§ 11** — the whole cadence census, the hard-floor table, the R0→R5 ladder to R5-prime, and the most-dangerous-change warning |
| `plans/active/catch-robustness.md` | A 2026-08-21 programme-status section: Phase 2's substrate untouched, its update laws retired, Phase 3 **re-scoped** (the re-baseline sitting is *also* ILC's no-correction A/B baseline, ≥2 flight-time cells, R0→R5 cadence), the catch-channel condition restated, and the `MIN_TOSS_THROW_DELAY_S` clause of § Constraints superseded — with the rest of that bullet (hand ladders, `_MAX_ARM_DISPATCHES`, kind-3 clobber rights) explicitly left standing |
| `plans/active/INDEX.md` | All three rows rewritten; `catch-robustness` and `toss-selftuning` re-dated to 2026-08-21 |
| `tools/probes/ilc_speed_band.py` (+ its README row) | **New committed probe**, promoted out of `/tmp` because build step 1 has to re-run it: it sweeps `event_vel_trim` against the real `throw_envelope` gate and reports the admitted band and the bound that closes each side. It is also what refuted this ripple's own first-draft mechanism — see the Discussion |

**No board moves.** Every candidate was checked and none qualifies: the ILC plan
was already unparked onto the active board in Phase A, and neither
`toss-selftuning` nor `catch-robustness` is superseded *as a whole* — each still
carries live schedulable work (the cadence ladder and its § 11.4 prerequisites;
the unfixed reload-interlude-cancel HIGH; the Phase-3 re-baseline sitting).
Archiving a document because part of it retired would take the live half off the
schedulable board, which is the failure the three-way split exists to prevent.
Recorded here so a reader does not mistake the absence of a move for an
oversight.

## The evidence the decisions rest on

Two read-only digests were commissioned before the decisions were taken, and
they are the specification this phase transcribed. Their headline findings:

**ILC's own evidence is strong and was independently validated.** V1 — `F`'s aim
block equals the production `aim_landing_jacobian` to 3.4e-11 relative, and the
exact identity `dL/dθ = 4h + Δz` (Δz = 6.736 mm) is pinned. V2 — a synthetic
closed loop recovers an injected 3-channel perturbation within 10 % (a
sign-flipped `F` is pinned to FAIL at ×2 residual), and the real corpus, fitted
on flight time **alone**, cancels **86.8 %** of the release-speed rms
out-of-channel (459.0 → 60.8 mm/s). V3 — leave-one-out **84.9 % / 86.4 %** rms
reduction against a pure-noise control pinned below 5 %. V4 — `R_rep`
**0.9858 / 0.9790** against a derived `REPEATABILITY_MIN = 0.5` and a per-cell
LOO null of −0.226: **no NULL-exit**. Conditioning: singular values
`[2.921, 2.921, 2.608, 0]`, condition 1.12 over retained channels, with
`release_timing_offset` refused because its column is structurally zero.

**Eight genuine contradictions, and none of them was papered over.** C1 the
common mode (owner-resolved inside ILC); C2 the speed authority (±0.15 is not
admissible near either end of the derived band — and is **+0.000 on the negative
side at exactly the cadence target**); C3 the two lateral channels disagreeing
by **+18.0 mm in y in every cell** while agreeing in x to 0.85 mm
(owner-resolved by decision 6); C4 the two-directional double-count
(owner-resolved by demoting the trim's aim estimator, plus a monitor-arithmetic
fix); C5 clamp starvation (dissolved — no map is being captured); C6 the corpus
being historical (accepted: Phase 3 starts from a fresh capture); C7 tier 8b
(resolved by keying the cell on the **aim site**, which is the release pose, plus
a zero-displacement admission gate); C8 reproducibility (partially closed by this
session's corpus-guard fix; a committed corpus fixture still does not exist).

**The cadence census found that the operator's stated target was the wrong
variable.** The hard floor is not `MIN_TOSS_THROW_DELAY_S` — it is hand-stroke
geometry: the catch stroke's post-contact tail, made un-hideable by C-HAND-1's
no-overlap rule, plus the throw's prelude, the Teensy build gap and the windup.
Across the whole C-HAND-3 admitted band that floor runs **0.2508 s** (at the
1.1485 s ceiling, apex 1.62 m, zero margin on everything else simultaneously) to
**0.4901 s** at the 0.4949 s band floor. A 0.25 s dwell is therefore unreachable
at **every** admitted flight time — it misses by 0.8 ms at the single most
extreme point and by 2× at a juggling-realistic flight. But cycle *period* is
nearly flat across the band and bottoms at **0.985 s ⇒ ~61 throws/min**, so the
6× throughput gain the operator wanted is available — bought by shortening the
*flight*, not the dwell. Hence R5-prime, and hence the standing advice to ask
for "≥ 50 throws/min" rather than for a dwell number. `MIN_TOSS_THROW_DELAY_S`
at 3.5 s is fencing a sequence that measurably costs 0.70 s.

⚠ **The census's most dangerous change, transcribed verbatim into the plan
because it is a safety finding, not a performance one**: the hazard is *not*
lowering `MIN_TOSS_THROW_DELAY_S` — it is lowering the dwell **without** the
possession-semantics work. That combination leaves the `ball_seated` gate
**fail-OPEN** (the measured `held→empty` debounce is 232–295 ms, so cycle N+1
reads a `_held` bit still true from the previous ball — a direct inversion of
C-POSSESS-1's "a dead sensor refuses, it does not pass"), labels **every good
cycle `BOUNCED`** (the retention window's justification is written against the
3.5 s delay and dies with it), and routes good cycles into the **auto-reload
interlude** — which asks BallButler to throw a second ball at a cup that already
holds one.

## Discussion

### Why the H2 pushback is decisive, and why "by decision" is not a shortcut

The arc had a clean plan for C3: a ~20-minute no-robot capture with the taped
ball fixtured and carrying 3+ conventional point markers, giving `b(x, y, z)`
absolutely. The owner refused it on a mechanism argument, and the argument holds:
**markers on the ball corrupt the very trackable surface being measured.** QTM
sees one blob because the ball is fully tape-covered; a marker cluster changes
that blob, so the capture would measure the centroid of a different object than
the one that flies. There is no version of H2 that measures the flying ball.

What makes the decision safe rather than merely unavoidable is that the arc's own
data already convicts the mechanism the owner named. The fitted bias is
**large at z ≈ 880 — the catch-plane height — and vanishes by z ≈ 1880**, and it
is **parity-EVEN** in time about the apex, which is the signature of a position
bias and not of any aerodynamic force. Platform-frame occlusion near the bottom
of the stroke predicts exactly that shape. So the channel that never carried the
bias — `arrival_dir`, a whole-arc velocity — was already the trustworthy one, and
the decision is to stop asking Q to arbitrate between it and a channel we now
understand to be systematically offset.

The cost is stated rather than hidden: with `land_err` masked out, **absolute**
centering is no longer measured by mocap at all. It closes through catch
outcomes instead, which is defensible because the penalty loop is the ground
truth for "centered on the cup" and a ~10 mm registration bias against a 35 mm
capture radius is tolerable and visible in the trend. The standing replacement
validation is the per-toss channel-disagreement log: if the `arrival_dir`-driven
loop converges while the plane residual holds the known `b(z)` profile shape,
the model is confirmed; **if catch rate plateaus with converged aim, C3 re-opens.**

### Two consequences the ripple derived, neither of which was in the digests

**(a) `SIGMA_E`'s stale `arrival_dir` σ stops being a micro-decision.** While Q
arbitrated two lateral channels, 0.00238 vs the measured 0.00302 rad was a
bounded 27 % over-trust that moved the pooled aim requirement 0.00997 → 0.01044
rad and nothing else — which is why the arc deliberately left it alone (Gate 1
approved the weights; re-deriving an approved weight is an owner call). Under
decision 6 `arrival_dir` is the **only** aim channel, so its σ no longer trades
off against anything — it sets the aim block's weight against `R = diag(ρ/τ²)`
directly, and understating it by 27 % takes correspondingly larger steps. It is
now a prerequisite before the first artifact is written, together with re-running
`conditioning()` / `screen_channels()` under the new mask: the aim columns
cleared `SCREEN_SNR_MIN` at 2.92σ with *both* lateral channels feeding them, and
a mask that drops two of five rows moves that number.

**(b) Decisions 3 and 5 collide at exactly the cadence target — and the fix is
one level up.** At T = 0.4949 s the admissible `k_v − 1` on the negative side is
`+0.000`, bounded by `ARM_WINDOW`. The corpus's measured demand is `−0.1076` —
a slow-down. So at R5-prime the ILC speed channel has **zero authority in the
only direction the plant asks for**.

The mechanism is worth writing down, because the obvious reading is wrong and I
wrote the wrong one first. It is *not* that a slower throw shortens the achieved
flight past `ARM_WINDOW_CLOSES_AT_S` — `throw_envelope.evaluate` takes `(T, v)`
as independent arguments and never re-derives a flight time from `v`. The real
chain, probed rather than argued (`tools/probes/ilc_speed_band.py`, run
2026-08-21): `arm_window_s = latest − earliest`, `latest` depends on **T alone**,
`earliest = throw_decel_s(v) + ARM_SUPPRESS_MARGIN_S`, and
`throw_decel_s = INERTIA_RATIO · t_acc` **grows as release speed falls** —
0.10488 s at v = 2.440 m/s against 0.11654 s at v = 2.196. A slow-down trim
therefore pushes the *earliest* arm instant later and narrows the window; at the
band floor the window is exactly `ARM_WINDOW_MARGIN_S` = 0.0500 s wide by
construction, so any negative trim at all breaks it. The probe also re-measures
the whole band and reproduces the digest's sweep to the grid resolution
(`[+0.000, ≥+0.5]` at 0.4949, `[−0.409, ≥+0.5]` at 0.55, `+0.270` at 0.9032,
`+0.148` at 1.00, `+0.043` at 1.10, `+0.000` at 1.1485).

The resolution is not to widen the authority. That demand is not a per-cell
learned residual at all: the hand throws ~11 % fast **consistently, at every
cell** (−0.096 / −0.112 / −0.124 at n = 8/6/3), which is a plant *gain*. A
cell-keyed artifact cannot express a T-independent gain, and — the part that
matters — `throw_envelope.evaluate` is a **model-space** gate, so an 11 %
uncalibrated gain means the gate's verdict is 11 % wrong about the real machine.
The pre-registered recommendation written into the plan is therefore to fold the
re-measured gain into the **model** (the nominal release-speed map), after which
model and plant agree, the envelope gate becomes honest, and the ILC trim demand
collapses toward zero everywhere — at which point the zero negative authority at
the band floor stops mattering, because nothing is asking for it. **Prerequisite
either way (C6): re-measure the gain on the restored drive first.** The +11 % was
measured through the −10.00 A braking clamp, and probe 0c's own caveat says its
verdict is re-checked after restoration.

### Why C1's resolution persists an anchor term at all

Decision 2 says the persisted per-cell artifact carries **only** the spatial
residual, and that no persisted cell may absorb one session's `level()` draw.
Taken alone, that would discard most of the correction: the measured per-cell
|aim| is 9.1–10.5 mrad and is **consistent across all three goal cells**, i.e.
the correction ILC found is dominated by common mode, worth ~36–42 mm at the
corpus operating point against a 35 mm capture radius.

The decision names its own way out — *transpose the § 3.2 doctrine* — and § 3.2
has three parts, not one: home-referenced spatial cells, **plus** a persisted
`anchor_aim_rad` acting as the **warm-start prior** for the session-local
component (not as a hard correction), **plus** a session estimator that owns the
common mode. Taking all three keeps the noise out *and* the correction in,
because each session contributes one independent `level()` draw to a shrinkage
mean, so a 1.2–1.7 mrad draw enters at `1/(n₀+S)` weight instead of being frozen
at one sample. That is the reading written into the plan, and it is flagged as
the one place this ripple interprets rather than transcribes — worth an
explicit owner confirmation, because "per-cell carries only the spatial
residual" and "a top-level anchor term is persisted" are consistent only under
that reading.

The honest limitation is stated in the plan rather than buried: the session
component cannot update *within* a session, because there is no live-admissible
observable — `land_err_mm` is mined and both live candidates are D5-forbidden,
which is the same `no_mocap_fit` wall the trim hit. Today it is
seeded-and-held: applied from the prior, updated between sessions by the fit.

## Verification — Phase C

- **Full gate on the docs ripple + the new probe**, run 2026-08-21:
  `./run_tests.sh --full` — **5991 passed, 13 skipped, 3 xfailed in 505.98 s**
  parallel, plus **9 passed, 6007 deselected in 41.48 s** serial; total 553 s,
  **RESULT: PASS**. Bit-identical counts to the post-guard-fix run in the Phase-B
  verification above, which is the expected answer: this phase changed markdown
  and added a probe nothing imports.
- **The probe itself**, run 2026-08-21: `python tools/probes/ilc_speed_band.py`
  — band `[0.4949, 1.1485] s`, arm window closes at 0.4542 s, and the six-row
  sweep quoted in the Discussion (`[+0.000, ≥+0.5]` at the floor with
  `neg=ARM_WINDOW` … `[≤−0.5, +0.000]` at the ceiling with
  `pos=DECEL_FF_HEADROOM`), plus the `throw_decel_s`-vs-`v` mechanism table.
- **The doc-surface tests, traced rather than inferred** (`logbook/README.md`
  § "What the logbook tests actually check" is explicit that a passing count is
  not coverage): `pytest tests/sim/test_plans_index.py
  tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_search.py -q`,
  run 2026-08-21 — **103 passed in 0.65 s**. What they actually assert:
  `test_plans_index` pins both directions of all three boards (every plan has a
  row; **no index may name a plan that left its directory**; four filled cells
  per active row; archived filenames bare), which is what makes the three
  edited rows honest. `test_logbook_front_matter` parses every entry and pins
  title/type/date/status present and the date ISO — it is what catches a
  malformed front matter here. `test_logbook_search` parses the real directory
  but **skips `INDEX.md` outright** and silently drops a title-less entry, so it
  does **not** cover the `logbook/INDEX.md` row this phase rewrote — that row
  was checked by hand, and by the table-cell scan below.
- **Mechanical cross-document pass** over the whole changed set, run
  2026-08-21: every markdown table's data rows re-counted against its header
  (the three plans, both INDEXes and this entry are clean; the 15 flagged rows
  are pre-existing `logbook/INDEX.md` rows with unescaped pipes inside code
  spans, none of them this arc's); every relative link target and every
  `plans/…`/`logbook/…` path resolved on disk. That pass found and fixed four
  dangling references: a `§ 4.2` that only exists in the read-only digest, two
  `§ C4`/`§ C7` shorthands that name no heading, a `reload_coordinator_node:2492`
  line number that is the ILC worktree's numbering (`aim_site` is in
  `_build_toss_cycle` on this tree), and — pre-existing, in a file this phase was
  already rewriting — three relative links in `toss-selftuning.md` still pointing
  at `tilt-calibration-grid.md`, `hand-ball-sensor.md` and `refactor-2026-07.md`
  as siblings, months after those plans moved to `archived/` and `parked/`.

## Phase D — build step 1: the speed channel's authority becomes T-dependent (C2)

**What changed.** `ILC_SPEED_AUTHORITY = 0.15` is demoted from *the authority*
to *an outer ceiling*, and the authority becomes
`ilc_fit_lib.speed_authority_band(T, v_nominal)` — the connected interval of
`k_v − 1` around the nominal command that `throw_envelope.evaluate` admits,
intersected with that ceiling. It is bisected rather than scanned, so the two
band edges are exact where `tools/probes/ilc_speed_band.py`'s 0.001 scan could
only report `+0.000`; both edges are returned on the *admitted* side, which is
`throw_envelope._bisect`'s own doctrine and for the same reason.

Three enforcement points, not one:

1. `ilc_fit_lib.admit_command` gains **step 2b** (the T-dependent band, replacing
   the flat per-channel check for `event_vel_trim`) and **step 6b**
   (`throw_envelope.evaluate` on the commanded speed, beside `validate_event_vel`).
   6b is not redundant with 2b: the band is derived at zero aim, so only 6b sees
   the `1/cos(aim)` growth an aim adds and a Tier-8b displaced goal's faster
   release.
2. `reload_coordinator_node._ilc_vel_trim_refusal` — the apply seam, which had
   **no** envelope gate at all. This is the half of C2 that was a live defect
   rather than a sizing error: a trim clearing the wire band and breaking
   C-HAND-3 reached `TossSequencer` CHECKING, which minted
   `REJECTED_THROW_ENVELOPE` and killed the goal — layer 3 acting as a gate,
   which is the one thing its own contract forbids.

**A third check the build ladder did not ask for.** The seam also refuses the
trim when the **untrimmed** goal is outside the envelope. Root cause: at long
flight times a *slow-down* trim is admissible where the nominal is not (at
T = 1.25 s the nominal 6.134 m/s is refused on `DECEL_AUTHORITY` and −0.12 brings
it to an admitted 5.398 m/s), so without this check whether a goal flies at all
would depend on whether an artifact happened to be loaded. "Byte-identical with
layer 3 off" has to hold for the machine's *verdict*, not only for its
arithmetic.

**The measured band** (`tools/probes/ilc_speed_band.py`, re-run 2026-08-21 on
this tree, unchanged from the fold-in's first run): `[+0.000, ≥+0.5]` at
T = 0.4949 s with the negative side bounded by `ARM_WINDOW`; `[≤−0.5, +0.270]` at
0.9032; `+0.148` at 1.00; `+0.043` at 1.10; `+0.000` at 1.1485 with the positive
side bounded by `DECEL_FF_HEADROOM`. Bisected against the exact derived edges the
binding side is `−2.8e-16` and `+1.1e-16` — zero to double precision, which is
what "by construction" means here: the band edge *is* the flight time at which
that bound reaches equality.

**The test that failed as written, and what replaced it.**
`test_the_event_vel_band_is_unreachable_inside_the_speed_authority` pinned Gate
1's safety argument — *"over the whole flight-time band and the whole ±0.15
authority, `event_vel` stays inside [0.3, 7.0] m/s"*. That statement is still
**true** and is now **irrelevant**: the rails do stay inside the wire band and
are refused anyway, because the wire band bounds nothing physical. It is replaced
by `test_the_scalar_speed_authority_is_inadmissible_near_both_band_ends` (both
ends, each naming the bound that closes it, plus the corpus's own −0.1076 demand
refused at exactly the R5-prime cadence target),
`test_the_speed_authority_band_is_derived_per_flight_time` (the T-sweep, against
the probe's published numbers, plus monotonicity and the fail-closed cases) and
`test_the_throw_envelope_is_wired_into_admit_command`.

At the node, the new gate is driven by a **real artifact** rather than by
injection: at T = 1.10 s the admissible trim is `+0.043` while the artifact's
parse-time ceiling is still ±0.15, so a `+0.10` cell is a perfectly legal
artifact this machine cannot fly at that flight time. That gap between "legal to
persist" and "flyable here" is the argument for gating at apply and not only at
parse — and it is why the parse-time ceiling was deliberately **not** made
T-dependent: a cell's key carries a *quantised* flight time (50 ms cells), so a
parse-time band would refuse or admit against a T the goal may not have.

### Verification — Phase D

- `python tools/probes/ilc_speed_band.py`, run 2026-08-21: reproduces the six-row
  band table above and the `throw_decel_s`-vs-speed mechanism block.
- `pytest tests/motion/test_ilc_fit.py tests/motion/test_toss_ilc.py -q
  -p no:randomly`, run 2026-08-21: **183 passed, 13 skipped in 2.21 s** (the 13
  skips are the corpus-backed V2b/V3/V4 arms — `temp/probes/` is gitignored).
- `pytest tests/ros/test_toss_ilc_node.py -q -p no:randomly`, run 2026-08-21:
  **30 passed in 5.89 s**.

## Phase E — build step 2a: the layer-2 aim estimator loses its authority (C4)

**What changed.** `toss_trim.AIM_AUTHORITY = 'MONITOR'`. The layer-2 aim
estimator keeps observing, keeps running G1–G11, keeps its CUSUM and its
freeze-never-zero, keeps printing and keeps writing its end-of-goal proposal —
and contributes exactly nothing to the commanded aim. The composition at
`reload_coordinator_node._toss_aim_for_goal` is now
`clamp_total_aim(map + ilc)`.

Root cause rather than the decision by name: two converging estimators of one
quantity double-count in both directions. Forward, the machine over-aims by the
ILC's contribution *while the trim reports CONVERGED* — at the corpus operating
point that is ~36–42 mm of landing shift against a 35 mm capture radius, i.e.
the difference between catching and not. Mirrored, a converged trim makes the
ILC's measured residual read `J·ILC_prev`, so the next fit returns
`du ≈ −ILC_prev` and the persisted artifact unlearns itself to zero. ILC keeps
the authority because its seam is the goal build, *upstream* of the FSM's
CHECKING gate, which is the only place a correction can be validated before it
becomes a command.

**The monitor's arithmetic had to change too, and that is the half that is easy
to miss.** `reduce_to_aim` reduces from `total_aim_rad`, which carries layer 3;
subtracting `map_aim_rad` alone made the monitor report layer 3's own converged
correction as demand that was still outstanding. `SessionTrim.observe` now
subtracts `map_aim_rad + ilc_aim_rad`, via a new `toss_trim.ilc_aim_rad`
accessor whose absent/null rule is deliberately **not** `map_aim_rad`'s:

* **absent or null ⇒ exactly zero.** Layer 3 writes the field on every path it
  runs — miss, dormant, disabled, clamp-refused — in the same `rec.update` that
  writes `total_aim_rad`. A record with a total but no ILC field is from a build
  where layer 3 did not exist, and there the contribution genuinely was zero.
* **present but not a pair ⇒ unknown, and the row is refused by name**
  (`ilc_aim_unknown`). That is a corrupt record, not an old one.
* `map_aim_rad` keeps its stricter `None`-on-absent rule, and the asymmetry is
  real: a mined-only row cannot say what MAP was applied, because the map is a
  file on disk the miner never saw and a non-zero one is already inside
  `land_err` — guessing zero there would re-learn a correction the machine is
  already making. Layer 3 not being in the record means layer 3 was not in the
  *build*.

**The record gains a key rather than repurposing one.** `trim_aim_rad` still
means "what layer 2 CONTRIBUTED to the commanded aim" and is now a structural
zero; the estimate rides beside it as `trim_monitor_aim_rad`, with
`trim_authority` recorded so a corpus can prove which build's rows carry a
commanded trim. Folding the monitor value into `trim_aim_rad` would have been
the C4 double-count wearing a different hat — `toss_trim.applied_aim_rad`'s
map+trim fallback, the miner and `ilc_fit_lib` all read that key as *applied*.

**Why the monitor is kept at all**, since a demotion implemented by never
feeding the estimator would satisfy every no-double-count assertion: the trim
reduces `land_err_mm` while layer 3's aim update is driven by `arrival_dir`
(decision 6), so the two are independent read-outs of one physical quantity and
their per-toss **disagreement is the standing validation** that C3's
by-decision resolution was the right call. A test pins that the estimator is
still fed and that the console says `MONITOR (NOT APPLIED)`.

### One consequence the ladder did not anticipate

**The D7 clamp's TRUNCATION branch is now unreachable without layer 3.**
`parse_toss_cal` refuses any map node past `TOTAL_MAX_RAD` *on the magnitude* —
the same quantity and the same number `clamp_total_aim` bounds — and the map's
lookup interpolates within the convex hull of admitted nodes, so with layer 2 at
zero authority every loadable map is already inside the clamp. The old test
`test_a_clamp_that_binds_on_map_plus_trim_ALONE_still_truncates` drove its
over-authority sum with a saturated map plus a saturated trim, and that sum no
longer exists. Rather than delete the coverage, the test is rewritten to assert
both halves of the new truth: a map saturated exactly AT the authority is
commanded verbatim with no clamp hit, and a map past it is refused at PARSE. The
truncation code stays — it is layer 1's semantics on paper — but the fact that
nothing can reach it is now written down instead of waiting to be rediscovered.

### Six tests changed, and why each one had to

Every one of them pinned layer 2's authority, so each is rewritten to pin the
demotion rather than deleted — and each keeps the root cause its predecessor was
about:

| Was | Is |
|---|---|
| `test_the_total_is_reclamped_at_apply_over_map_plus_trim` (node) | `test_the_layer_2_aim_has_ZERO_authority_over_the_commanded_aim` — a saturated trim does not even reach the clamp |
| `test_the_virtual_target_is_built_from_the_TOTAL_aim` | `test_the_virtual_target_is_NOT_moved_by_the_monitor_only_trim` — the mirror, and the more important direction now: a build that left the estimate in `trim_aim_rad` while not composing it would be the same silent-wrong, inverted |
| `test_pretilt_hold_is_raised_for_a_trim_only_aim` | `test_pretilt_hold_is_raised_for_a_map_only_aim` + `test_a_trim_only_aim_raises_NO_pretilt_hold` — D3's rule keyed on the commanded release state, so the coverage is re-pointed at a layer that commands, and the converse is asserted |
| `test_the_record_carries_map_trim_and_total_separately` | `..._map_trim_monitor_and_total_separately` — the total is the map alone, 0.2° not 0.3° |
| `test_a_clamp_that_binds_on_map_plus_trim_ALONE_still_truncates` | `test_the_D7_clamp_can_no_longer_BIND_without_layer_3` — see above |
| `test_the_total_is_reclamped_at_apply_over_map_plus_trim` (pure) | `test_the_total_clamp_bounds_a_sum_no_single_layer_can_see` — same arithmetic property of `clamp_total_aim`, illustrated with `map + ilc`, because the old illustration would have gone on passing while describing a composition that no longer happens |

Plus the `_pre_phase2_aim_block` **oracle**, which is a hand-transcribed copy of
the pre-Phase-2 arithmetic. It carries the C4 demotion, and the reason is worth
recording: the claim that oracle checks is *"with layer 3 inactive the numbers
are the pre-Phase-2 numbers"*, so leaving layer 2's authority in it would make
it fail for a reason that has nothing to do with layer 3. What it still holds
independently — and is the whole point of keeping a second copy — is the
pre-Phase-2 *shape*: `trim_offset_mm = offset − map_offset`, against the live
code's `base_offset − map_offset`.

### The four new tests (the digest's two failure narratives)

* `tests/ros/test_toss_ilc_node.py::test_both_layers_live_the_machine_does_NOT_
  over_aim_by_the_trim` — narrative 1, with the counterfactual computed rather
  than assumed: the total that the old composition *would* have commanded is
  formed and shown to differ by a real (> 1 mm) displacement, so a stub trim
  reading zero cannot pass it.
* `..._node.py::test_the_trim_keeps_OBSERVING_while_it_commands_nothing` — the
  half a lazy demotion would break.
* `tests/motion/test_toss_trim.py::test_a_converged_ILC_leaves_the_monitor_
  demanding_NOTHING` — narrative 2, and it evaluates the OLD expression on the
  same rows to show the bias removed (mean demand ≈ the ILC's own correction,
  outside the deadband) rather than only showing the new number is zero.
* `..._test_toss_trim.py::test_the_monitor_still_sees_a_residual_layer_3_has_
  NOT_corrected` — the converse, so "subtract the ILC too" cannot be satisfied
  by an estimator that was simply switched off. Its expectation carries the
  shrinkage factor `n/(n₀+n)` explicitly: a tolerance wide enough to swallow the
  prior's pull would also swallow a sign error on the ILC subtraction.

### Verification — Phase E

- `pytest tests/motion/test_toss_trim.py tests/ros/test_toss_trim_node.py
  tests/ros/test_toss_ilc_node.py tests/motion/test_toss_ilc.py
  tests/motion/test_ilc_fit.py -q -p no:randomly`, run 2026-08-21:
  **318 passed, 13 skipped in 28.44 s**.

#### Salvage note — Phase E was written by a session that closed before it committed

Phase E's nine-file working set sat **uncommitted** in the main tree from
2026-08-21 19:55 until it was picked up on 2026-08-22 by the session that wrote
Phase F below. The authoring session had closed. It is credited here rather than
silently absorbed: the design above, the six rewritten tests, the four new ones
and this section's prose are its work.

**Salvage-verified**, and the verification found the gap the house rule
predicts — *never trust an unrun test assertion*. The scoped run cited
immediately above covers five files; the two additive record fields
(`trim_monitor_aim_rad`, `trim_authority`) are also pinned by
`tests/motion/test_toss_record.py::test_fields_are_pinned`, the record's own
name-order drift guard, which that scope never reached and which was
**failing**:

```
At index 51 diff: 'trim_monitor_aim_rad' != 'total_aim_rad'
```

That is the drift guard working exactly as designed — it is the one test whose
whole job is to notice a field list changing, and it is in a sixth file. Fixed
by adding both names to `EXPECTED_FIELDS` in calibration-block order. Nothing
else in the salvaged diff needed changing; every symbol it referenced
(`_aimed_node`, `toss_trim.replay`, `_is_pair`'s string rejection, the `'s'`
field kind) was checked against the tree before the run rather than assumed.

- `./run_tests.sh`, run 2026-08-22: **5579 passed, 13 skipped in 254.85 s**
  (parallel 258 s, serial phase empty, total 267 s, `RESULT: PASS`).
