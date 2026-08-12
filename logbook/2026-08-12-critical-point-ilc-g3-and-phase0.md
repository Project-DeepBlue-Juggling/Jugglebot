---
title: "Critical-point ILC — G-3 merged, Phase 0 complete: the arrival/backcast fields, two probes, a working messy-catch score, and a hand that throws 11 % fast"
type: feature
date: 2026-08-12
status: resolved
phase: "critical-point-ilc — G-3 + Phase 0 (0a–0d)"
related_plan: critical-point-ilc.md
files_changed:
  - plans/active/critical-point-ilc.md
  - plans/active/INDEX.md
  - logbook/INDEX.md
  - logbook/2026-08-10-toss-selftuning-build.md
  - ros_ws/src/jugglebot/jugglebot/toss_record.py
  - tools/probes/toss_record_miner.py
  - tools/probes/hand_contact_softness.py
  - tools/probes/hand_sensor_settle.py
  - tools/probes/README.md
  - tests/motion/test_toss_record.py
  - tests/motion/test_toss_trim.py
  - tests/ros/test_toss_record_miner.py
  - tests/ros/test_ilc_measurement_probes.py
subsystem:
  - motion
  - tracking
  - tools
tags:
  - testing
  - kinematics
---

# Critical-point ILC — G-3 merged, Phase 0 complete

Gate G-3 closed by merging `mvp-trajectory-bringup` at `e75badd` into
`critical-point-ilc` (merge `712bcee`; conflicts were the two index tables
only). The owner's messy-catch metric (sensor bounce = messy, 2026-08-12)
was added to the plan as catch-error channel three and Phase 0d (`b246856`).
Phase 0 then ran desk-side as two parallel Opus implementation agents — 0a/0c
extending the toss-record miner, 0b/0d as two new committed probes — against
the real corpus: the 2026-08-10 reference bag plus the ten 2026-08-12 bags
(three of the eleven carry mocap + announcements — the reference and two from
2026-08-12; the other eight are hand-telemetry-only).

**What landed.** 24 additive origin-'M' mined fields (`command_ref`,
`arrival`, `backcast`, `split` blocks; no schema bump — the record's
additive rule) plus the `usable_for_release_fit` selection flag (origin
'X', added at audit so the headline population is expressible from the
corpus), `mine_arc` as the single definition point, whole-branch ballistic
fits carrying their own standard errors;
`tools/probes/hand_contact_softness.py` (0b) and
`tools/probes/hand_sensor_settle.py` (0d); 44 new scoped tests + 85 new
self-check cases across the three tools.

**Phase outcomes, against the pre-registered menus.**

- **0b → outcome (ii)**: the catch-contact transient is NOT resolvable in
  either hand-drive channel. The decisive control is cross-label at matched
  phase — CAUGHT vs MISSED under identical commanded choreography (ball vs
  no ball): `vel_meas` 1.03×, iq impulse 1.45× (post-audit shared disjoint
  baseline; both under the significance bar) — the apparent 8.8×-over-floor
  transient is the commanded stroke. Softness stays modeled-surrogate-only
  in v1; a firmware high-rate capture is the follow-on. Caveat: the whole
  census ran through the clamped hand drive (G-2 unrestored); re-check after
  restoration.
- **0d → outcome (i)**: the messy-catch score works. Raw `ball_held_raw`
  flips within 0.75 s of the arrival edge, threshold ≥ 1: recall 3/3 on the
  reference bag's known quick-drops, false-clean 0 %, false-messy 3/55
  (all three scoring exactly 2 with 9–12 s possession — plausibly genuinely
  rattly-but-kept). Joins the mined fields and the Phase-3 criteria.
- **0a/0c**: the vertical channels are clean and carry the headline —
  **the hand throws ~11 % fast, consistently** (+478 mm/s pooled median on
  a commanded 4436, n = 19 under `usable_for_release_fit` across the three
  mocap-bearing bags; reference bag +473, n = 16; flight time +101 ms on a
  commanded 903 ms; apex cross-check 4924 vs 4900 mm/s). This is exactly the
  repeatable, model-invisible bias the ILC exists to null, and a Phase-1 v1
  restricted to the vertical channels can start on it immediately.

## Discussion

**The most important negative result: the release-vs-flight split is not
clean.** The lateral channels — exactly the aim channels — carry a
~±100 mm/s repeatable branch-to-branch velocity artefact (per-axis
release/flight correlation −0.38/−0.46; `v_y` +93 → −7 mm/s between
branches on every toss). Magnitude and whole-arc average both fit a mocap
marker ~30 mm off the ball centre on a ball spinning ~0.5 rev/s; genuine
Magnus is unseparated. Phase 1 gained entry condition E-1: no fit trusts
the direction-error channels until this is resolved. Trusting them today
would have fed a plausible-looking, systematically wrong aim gradient
straight into the learner — found only because 0c fit both branches and
compared.

**Two 2a claims withdrawn/reframed** (addenda in
`logbook/2026-08-10-toss-selftuning-build.md` § Withdrawn claims): the
reference bag DOES support an aim fit — the descending-branch selector was
cutting at the self toss's own resting ball ~110 mm below the plane, so
`land_xy_global_mm` had been null on every self toss ever mined
(`usable_for_aim_fit` 0/31 → 7/31 after the miner-side fix); and "the
release runs late" is a sensor-cadence artefact — the mocap backcast puts
release at −4.6 ms vs the announcement while the sensor departure edge lags
by an amount that tracks the poll cadence (+172 ms at 71 ms poll, +95 ms at
54 ms).

**Plan-text corrections from the censuses**, folded into the plan the same
day: `vel_meas` is not genuinely 100 Hz (~10 % duplicates, ~50 Hz effective
through fast strokes); the sensor poll p50 is session-dependent (20–76 ms
across bags — read per record, never assume 71 ms); 0b's pre-registered
outcome (iii) had named the wrong direction, and the observed fourth cell
is reported UNREGISTERED by the probe rather than rounded into a box.

**Judgement call flagged for review**: 0a populated the existing
`achieved_flight_s_mocap` field (present in the schema with an empty doc
and no producer) instead of adding a duplicate name — read as "no prior
meaning to change"; reversible. The audit sharpened the consequence: "no
producer" is not "no consumer" — populating it makes
`toss_trim.admit_for_speed` live (the `k_v` speed-gain estimator was
structurally dead without it) and `flight_time_reference` prefer the mocap
value, so the newly-live path is now pinned by
`test_a_mined_mocap_flight_time_ACTIVATES_the_speed_gate`.

**Pre-commit audit (Opus audit-reporter, 2026-08-12): 4 BLOCKING /
9 WARNING / 4 NOTE — all applied same-day by an Opus fix agent, two
applied-differently with evidence.** The two that mattered most: the
messy-score flip counter seeded its state from the first sample *inside*
the window, so a flip between the last pre-window and first in-window poll
was invisible — miss direction false-CLEAN, confirmed real (a drop on the
first post-arrival poll scored 0, now 1, `usable` recall unchanged at 3/3);
and the within-cycle iq "signal" (3.46×, p = 1.4e-9) was a baseline
artefact — both integrals subtracted the control window's own median — and
collapsed to 0.96×, p = 0.68 under a shared disjoint quiet-held baseline,
while the cross-label headline survived (1.03×/1.45×, outcome (ii)
unchanged). The two evidence-based deviations: the README's 1.715/8.8×
figures are correct for the *fixed* code (the audit's 1.704/8.7× described
the pre-fix state — the plan and this entry now carry 8.8×), and the
`usable_for_release_fit` flag ships origin 'X' beside its three siblings
rather than the suggested 'M', keeping the origin partition meaningful.

## Verification

- Full gate on the merged tree before the merge commit (`./run_tests.sh -q`,
  run 2026-08-12 in the worktree): **PASS — 5073 passed, 5 skipped,
  0 failed** (parallel 236 s rc=0, serial 9 s rc=0, total 245 s; counts
  tallied from the captured run log).
- Combined scoped run over both agents' test files after both landed
  (`pytest tests/motion/test_toss_record.py tests/ros/test_toss_record_miner.py
  tests/ros/test_ilc_measurement_probes.py -q -p no:cacheprovider`, run
  2026-08-12): **102 passed in 33.07 s**.
- Full gate on the pre-audit Phase-0 tree (`./run_tests.sh -q`, run
  2026-08-12): **PASS — 5105 passed, 5 skipped, 0 failed** (parallel 230 s
  rc=0, serial 10 s rc=0, total 240 s; counts tallied from the captured run
  log).
- `/audit --unstaged` (multi-document narrative gate + code review, Opus):
  verdict NOT CLEAN, 4 BLOCKING / 9 WARNING / 4 NOTE — dispositions in the
  Discussion above; all fixes verified by the runs below.
- Post-audit-fix, final tree, all run 2026-08-12:
  `toss_record_miner.py --self-check` **45/45**;
  `hand_contact_softness.py --self-check` **28/28**;
  `hand_sensor_settle.py --self-check` **34/34**;
  `pytest tests/motion/test_toss_record.py tests/ros/test_toss_record_miner.py
  tests/ros/test_ilc_measurement_probes.py tests/motion/test_toss_trim.py -q
  -p no:cacheprovider` → **191 passed in 72.96 s**;
  full gate (`./run_tests.sh`, run 2026-08-12): **PASS — 5116 passed,
  5 skipped, 0 failed in 245 s**.
