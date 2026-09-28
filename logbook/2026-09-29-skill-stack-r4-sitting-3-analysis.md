---
title: "R4 sitting 3: the self-toss residual is release-side scatter, not the owner's well-positioned-platform premise; the hop overshoot is pre-release translation lag, not the post-release return sitting 1 blamed; the reload's catch-seed dead zone is fixed"
type: investigation
date: 2026-09-29
status: in-progress
phase: "two-ball-skill-stack — R4"
related_plan: two-ball-skill-stack.md
sessions:
  - temp/logs/skills_r4_20260927_2237.log
  - temp/logs/skills_r4_20260928_2141.log
  - temp/logs/skills_r4_20260928_2355.log
  - ~/Desktop/rosbags/2026-09-27_22-37-26 (mcap — not in the repo)
  - ~/Desktop/rosbags/2026-09-28_21-41-10 (mcap — not in the repo)
  - ~/Desktop/rosbags/2026-09-28_23-55-19 (mcap — not in the repo)
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/admissible.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/schedule.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/segments.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/cup_cycle.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/feasibility.py
  - ros_ws/src/jugglebot/jugglebot/motion/unified_cycle.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - ros_ws/src/jugglebot/launch/jugglebot_launch.py
  - sim/skills_gate.py
  - tests/motion/test_cup_cycle.py
  - tests/motion/test_skills_schedule.py
  - tests/motion/test_skills_segments.py
  - tests/motion/test_validate_cycle_vectorised.py
  - tests/ros/test_skill_node.py
  - tests/ros/test_trajectory_node.py
  - tools/probes/selftoss_landing_decomposition.py
  - tools/admissible_sweep.py
  - config/generated/admissible_box.yaml
  - tests/motion/test_skills_admissible.py
  - tests/hardware/session_skills_r4_diag.md
subsystem:
  - motion
  - ros
  - sim
  - tools
tags:
  - performance
  - kinematics
  - testing
---

# R4 sitting 3 (2026-09-28 23:55)

## Symptom

Owner's report, one launch (log `temp/logs/skills_r4_20260928_2355.log`, bag
`~/Desktop/rosbags/2026-09-28_23-55-19`):

1. Self-toss unchanged from sitting 2: every catch is still re-aimed by the full 20 mm lateral
   authority.
2. Every reload left the hand parked at the bottom of its stroke; the inbound ball was caught hard;
   the refusal behind it (`CATCH_AXIS`) was cryptic to the operator.
3. Hops landed +88 / +110 mm long on the far site.
4. A 0.95 m hop was refused, and the box-refusal message read like an apex problem when the real
   mismatch was the swept separation.

## Diagnosis

### 1. Self-toss: the owner's premise does not survive — the residual is release-side scatter

43 fitted self-tosses across sittings 1–3 (new probe `tools/probes/selftoss_landing_decomposition.py`,
evidence `.scratch/r4-throw-precision/evidence/result_t01.md`): rest throws (first throw of an
attempt, platform stationary, nothing caught before it) give a landing 1σ of **(15.8, 23.0) mm**,
1.6–2.3× the ±10 mm target. The mean is (+6, +2) mm with no direction that repeats across sittings —
**scatter, not bias.**

The error enters as the ball's lateral launch velocity: `dv` (fitted velocity minus the velocity
needed to land on the announced site) has sd (18, 26) mm/s, which over T≈0.86 s reproduces the
landing scatter almost exactly. It is not a platform pose error — platform tilt varies only
0.02–0.09° and release position by 2.6 mm across rest throws, and the root-sum-square of every
platform term (tilt, xy velocity, release position) is **≤ ~7 mm** against an observed 13–28 mm.

The reconciliation in `result_t02.md` § "Reconciliation with ticket 01" re-samples the platform at
+15 ms (rather than at the release knot) and reaches the same number: rest throws' residual after
subtracting the platform-velocity transfer law is **19 / 25 mm/s, i.e. 17 / 22 mm of landing**,
uncorrelated with the platform's own y velocity (r = 0.00). That residual is release-side: how the
ball sits in the cup, sideways play or rolling under the stroke, cup-lip contact at separation, or
the hand axis not being coaxial with the platform normal. The bag has no cup or hand marker to
separate those candidates.

### 2. Hop: translation lag, not tilt — and the sitting-1 hold attribution was wrong

Tilt at release matches the plan within 0.03–0.14° on all six hops instrumented (three from sitting 1,
no post-release hold; three from sitting 3, with the 50 ms post-release hold landed 2026-09-28).
Orientation is **refuted** as the mechanism.

The plan swings the platform +x at about +106 mm/s at −75 ms and asks it to stop at the release
knot. The legs lag the command by ~25 ms on the encoders and ~40–50 ms in mocap, so at ball
separation (+10..+20 ms after the knot) the platform is still translating at **+98..+129 mm/s**. The
ball inherits that velocity one-for-one: regressing the ball's perpendicular launch-velocity surplus
on the mocap platform vx at +10 ms gives slope 0.83, R² 0.70 (n = 38, self-tosses and hops, both
sittings).

**Sitting 1 (no hold) and sitting 3 (with the hold) are indistinguishable**: +98..130 mm/s at t_rel
and +87..104 mm landing error, both sittings. The hold shows up in the **commanded** trajectory (cmd
vx ≤ 9 mm/s over the +25..+50 ms window) but never in the **realised** one — the platform is still
decelerating from the pre-release swing through the entire hold window (mocap vx 120→94→69→47→32→14
mm/s over 0..+50 ms, reaching 0 at ~+60 ms, then undershooting to −20..−30 mm/s at +75..+100 ms).

This corrects the sitting-1 attribution. That entry read the ~+95 mm/s surplus as the plan's
post-release re-acceleration (the "return" after the release knot), which is why the fix landed as a
50 ms **post**-release hold. In fact the realised platform never re-accelerates within 80 ms in
sitting 1 either — there is nothing for a post-release hold to hold against, because the surplus
velocity is entirely the tail of the pre-release forward swing, arriving late at the knot because the
legs are still catching up to a command that has already gone to zero.

### 3. Reload: the fresh-predicate dead zone

`schedule.compile_reload` places the CATCH's dispatch instant at exactly `pretilt_end_rel − LEAD_S`
(0.225 s / 9 knots) — "by construction", per its own docstring — so that the schedule's arithmetic
and `_svc_install_segment`'s fresh-vs-splice test (`t_now + LEAD_S >= record.end_s`) agree that a
fresh install is safe to sample the machine's **live** pose at that same instant. `_cycle_start_state`
is always asked for `kind=uc.SETTLE`, whose `_KIND_SHAPE` entry carries `post_release=True` (added
2026-09-16 for hand-seed reconciliation on every fresh install) — that branch never refuses on a
moving machine, it just reports whatever the live pose currently is.

Net effect: the CATCH's very first dispatch attempt samples the PRE-TILT REST **225 ms before** its
S-curve has physically reached the held-axis target, generically off that axis by whatever lateral
distance remains — 1.2 mm reproduced offline (`probe_t03.py`, scenario E), 5.681 / 1.065 mm on the
sitting's two occurrences (the residual varies with 40 Hz tick and ROS2/DDS service-call jitter, not
a fixed knot count, which is why the two occurrences differ).

### 4. The box message conflated separation with apex

The 0.95 m hop's refusal named the apex as out of range when the real mismatch was the swept
separation (the box is only swept at 100/250 mm separations; 0.95 m sits at an untested
apex/separation pairing). `admissible.describe_miss()` now distinguishes "apex out of range" from
"sites don't match the swept separation".

## Discussion

**Why a pre-release hold, and not acceleration feedforward alone.** Accel FF
(`plans/active/accel-ff-inertia.md`, parked) would cut the 25–50 ms leg tracking lag that is the root
of the residual velocity, but it reduces the lag, it doesn't remove the plan's own dependence on it
being small. A pre-release hold is structural instead: it forces the QP to plan the cup at ~0 lateral
velocity for the last N knots, so whatever residual tracking lag remains, there is no commanded
velocity left for it to be late catching up to. The feasibility probe
(`result_prehold_probe.md`) confirms this is not just cheaper but more robust: N = 4 (100 ms) survives
an effective delay up to ~100 ms with the carrier velocity at +15 ms staying ≤ 1 mm/s, comfortably
above the measured 45 ms (mocap) + 30 ms (fitted lag τ) + 20 ms (separation) ≈ 95 ms budget. Accel FF
remains the right complementary fix for the lag itself — it stays parked and is not a substitute here,
per ticket 02's own framing.

**Why the tilt is not pinned.** The natural-seeming companion fix — freeze the tilt schedule over the
same N knots the cup velocity is held — was tried and rejected on two independent grounds. It is
**infeasible**: pinning `tilt_schedule` knots n−1−N..n−1 refused `LIMIT_ACC` at 15,345–23,431 mm/s²
(N=2..4) against a 5000 limit, because the accel-bounded smoother cannot move the tilt ramp earlier to
honour consecutive pins without blowing the leg acceleration budget. And it is **unnecessary**: the
cup QP already plans the cup's own velocity directly, so decompose (the cup→platform split) places the
residual tilt rate (3.59→4.01° over the last 100 ms, ~0 at the release knot) onto the centroid, which
absorbs it without touching the carrier velocity the ball actually rides. The remaining platform
acceleration at the release knot (`kappa·g` from the terminal `acc == g` row, ≈690 mm/s²) gives a
within-knot residual of ≤ 2 mm/s — negligible against the ~100 mm/s it replaces.

**Refuse, not clip, on the pre-release hold's edge cases.** The hold has real preconditions: a catch
inside the same window must land strictly before the hold starts (`k_td < n − N`, or its
catch-position equality fights the hold rows on rank), the window needs `n ≥ N + 2` knots to exist at
all, and it needs a positive `v_takeoff_z` (a hold on a cup that isn't launching upward has no
well-defined `kappa = v_xy/v_z` to hold to). The fix adds these as a new `HOLD_WINDOW` refusal family
in `cup_cycle.py` rather than silently shrinking N, clamping the window, or falling back to no hold
when a case doesn't fit. A clipped hold is the more dangerous failure mode here: it would still *look*
like a hold in the schedule (a nonzero `pre_release_hold_knots`) while giving a margin below the
95 ms budget this fix is built on, and nothing in the log would distinguish "held for 100 ms" from
"held for 20 ms because the window was tight." Refusing makes an under-provisioned hold visible at
plan time instead of showing up later as an unexplained partial overshoot — the same reasoning that
governs `HOLD_WINDOW`'s sibling `CATCH_AXIS` and the rest of the gated-refusal family.

**Why the fix is in `compile_reload`, not in the node's seed logic.** The obvious alternative is to
make `_cycle_start_state`'s `SETTLE` branch refuse on a moving machine, the way the non-`post_release`
branch already does. That was rejected: `post_release=True` on `SETTLE` is deliberate, load-bearing
behaviour from 2026-09-16 (the hand-seed reconciliation that runs on every fresh install, not just a
reload's), and every other fresh install in the system relies on it never refusing there. Changing it
would touch a shared code path used by every skill's fresh install, with no way to scope the change to
just this one dead zone. The actual defect is local to the reload: `compile_reload` is the one place
that places a CATCH's dispatch instant exactly at the earliest instant the fresh check can fire,
before the preceding REST has actually arrived. That is a scheduling-geometry bug with a
scheduling-geometry fix — subtract `LEAD_S` from the window so the dispatch instant moves to
`pretilt_end_rel` (the REST's real completion), not before it.

**The fresh-predicate's dead zone `[end_s − lead, end_s)` is a carried class, not closed.** The reload
fix closes the one place this cost a refused reload, but the dead zone itself is generic:
`_svc_install_segment` can seed any fresh install from a live, possibly mid-slew, commanded state
whenever that install's earliest dispatch attempt falls inside it. THROW 0, dispatched right after the
opening REST, lands in the same zone by construction — but it is benign there, because the opening
REST's own target is on-axis in every direction that matters to a THROW's seed (unlike the reload's
CATCH, which needs the seed on a specific held-axis line). Whether this deserves a class-level fix at
the node (for example, refusing any fresh install inside the dead zone unless the caller declares its
seed axis-insensitive) is left in `.scratch/r4-throw-precision/map.md` § "Not yet specified" rather
than decided here.

**What is still unexplained.** The self-toss release-side residual (17/22 mm) has no instrument that
separates ball-in-cup motion from carriage compliance from a non-coaxial hand axis — that needs the
diagnostic sitting's instrumented block (ticket 05), not more bag analysis. And the learner's lateral
command wanders ±10 mm on noise with no lateral bias to correct (rest-throw mean (+6, +2), SEM (3.6,
5.3)) — ticket 01 recommends freezing lateral learning, but that is an owner decision, not yet made,
and no code change follows from it in this entry.

## Fix

- `schedule.compile_reload` (`ros_ws/.../motion/skills/schedule.py`): `catch_window` is now
  `t_land_rel − pretilt_end_rel − LEAD_S` instead of `t_land_rel − pretilt_end_rel`, which moves the
  CATCH's `dispatch_s()` out to exactly `pretilt_end_rel` — the REST's real completion — instead of
  `LEAD_S` before it. The pre-tilt REST's end moves back by the same `LEAD_S`, so
  the CATCH's own window stays exactly `RELOAD_CATCH_WINDOW_S` (0.5 s) and its motion keeps the
  0.725 s it actually ran. Cost: BB's announcement must lead the landing by about 225 ms more (2.45 s
  in total; `skill_node`'s 2.5 s BB throw-delay floor still clears it). New test
  `test_compile_reload_catch_dispatches_only_once_the_pretilt_rest_has_ended` fails before, passes
  after.
- `PRE_RELEASE_HOLD_S = 0.100` (`unified_cycle.py`), the mirror of `POST_RELEASE_HOLD_S`: plumbed
  through `CycleGoals.pre_release_hold_knots` and a new `pre_release_hold_knots()` twin of
  `post_release_hold_knots()`, `segments.SegmentConfig.pre_release_hold_s` (default
  `uc.PRE_RELEASE_HOLD_S`), and `cup_cycle._assemble`, which adds two equality rows per knot on the
  velocity integrator (`vel_xy[k] == kappa·vel_z[k]` for `k ∈ [n−N, n−1]`, `kappa = v_takeoff_xy /
  v_takeoff_z`) — the same structure as the existing post-release hold block, no tilt pin (rejected;
  see Discussion). `trajectory_node.py` declares `pre_release_hold_s` as a live-settable parameter
  (bounded `[0, 0.25]` s, validated in the parameter callback, `ros2 param set /trajectory_node
  pre_release_hold_s 0.0` turns it off for an A/B); `jugglebot_launch.py` adds the matching launch
  argument, default `0.100`.
- New `HOLD_WINDOW` refusal family in `cup_cycle.py`: refuses (does not clip) a pre-release hold
  request when there aren't enough knots for it, when a catch in the same window lands inside the
  hold rather than strictly before it, or when `v_takeoff_z` isn't positive.
- Plain-language refusal messages: the `CATCH_AXIS` message in `cup_cycle.py` is rewritten to state
  the seed offset's magnitude in plain words; a new `admissible.describe_miss()` (used by both
  `skill_node.py` and `executor.py`'s box refusals) says whether the apex is out of range or the
  sites don't match the swept separation. New test
  `test_hop_refusal_names_the_separation_mismatch_not_just_the_apex` fails before, passes after. This
  edit changes the `cup_cycle.py`/`feasibility.py` gate hash, so `config/generated/admissible_box.yaml`
  needs re-sweeping (batched with the pre-release-hold edits below — same gated files, one sweep).
  Remaining jargon-heavy messages (other `CATCH_AXIS` variants, `CATCH_TOO_EARLY`, `CUP_CONTACT_ACC`,
  `SINGULAR`, the feasibility Jacobian condition) are deferred to the next gated batch.
- New reusable probe `tools/probes/selftoss_landing_decomposition.py`: decomposes a bag's self-toss
  landings into `p_rel − p_ann` / `needed` / `dv` / `cup` terms against a gravity-fixed parabola fit to
  raw mocap; writes `temp/probes/selftoss_decomp_*.txt` plus a per-bag `.npz` raw cache.

## Verification

* 2026-09-29 00:57, `python sim/skills_gate.py --reload` (ticket 03's timing fix alone) →
  **PASS 5/5** (`temp/logs/skills_gate_reload_20260929.log`: seeds 0–4, 2 makes / 0 drops / 0
  pump_rejects each, installs 7/7 or 8/8, wall 20.3 s).
* 2026-09-29 ~01:18, a scoped pytest run over the touched skills/ros test modules
  (`temp/logs/prehold_scoped.txt`; the invocation line itself was not captured in the log, only the
  xdist worker output) → **595 passed, 2 skipped, 19 FAILED**. All 19 failures are the same cause —
  `admissible box refused: ... swept against gate_hash='bf095653d422' but the live gate ... hashes to
  'da8ee3afc541'` — i.e. the plain-language-message edit moved the gate hash and the box has not been
  re-swept yet (see the re-sweep below, still running). None of the 19 are a functional regression
  from this session's changes.
* 2026-09-29 01:18:59, `python sim/skills_gate.py --reload` (re-run after the pre-release-hold edits
  landed, to check no regression) → **PASS 5/5** (`temp/logs/prehold_reload.txt`, wall 22.9 s).
* 2026-09-29 01:19:34, `python sim/skills_gate.py --seeds 0 1 2 3 4 --no-viewer` (columns pattern) →
  **PASS**, 20/20 makes / 0 drops every seed, all 5 seeds (`temp/logs/prehold_columns.txt`, wall
  58.2 s).
* 2026-09-29 01:21:22, `python sim/skills_gate.py --learn --pattern hop --seeds 0 1 2 3 4
  --no-viewer` → **FAIL** on `band_xy`/`band_apex` (`None`/`None`) all 5 seeds, 25 makes / 0 drops
  each (`temp/logs/prehold_hop.txt`, wall 166.0 s). This is the known, pre-existing box-clipped
  verdict (the hop learner's authority is smaller than the box bound it's chasing) — unchanged by this
  session's fixes, and the same `None` verdict as R4 sitting 1's entry.
* 2026-09-29 01:21:33, `python sim/skills_gate.py --learn --pattern self_toss --seeds 0 1 2 3 4
  --no-viewer` → **PASS 5/5** (`temp/logs/prehold_self_toss.txt`: band_xy 3, band_apex 5, mono_xy/
  mono_apex True, 25 makes / 0 drops each seed, wall 177.3 s).
* 2026-09-29 01:51–02:25, `OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 python tools/admissible_sweep.py
  --site-pairs all --single-apex 0.5 0.6 0.7 0.8 0.9 --out temp/probes/admissible_box_r4d_run{1,2}.yaml`,
  two parallel runs (`temp/logs/admissible_sweep_r4d_run{1,2}_20260929.log`, 34.3 / 33.8 min) →
  **bit-identical** (`cmp`), `gate_hash` **`ad36fa53aab2`**, installed as
  `config/generated/admissible_box.yaml`. What moved against `bf095653d422`:
  - **hop boxes WIDER**: P1→P2 x upper +2 → +10 mm, and P2→P1 x lower −0.5 → −10 mm (the hold lowers the hop's leg peaks);
  - **self_toss boxes narrower**: ±40 → ±30 mm at 0.6–0.9 m, and y ±20 → ±10 mm at 0.5 m. This still covers the 20 mm learner/re-aim authority;
  - **columns boxes EMPTY**. The sweep's columns THROW cell (0.3 s dwell) cannot fit the 100 ms hold: hold 0 → OK, 0.05 → MARGIN, 0.1 → INFEASIBLE (2026-09-29 probe of `_throw_cell`). The real chained columns pattern still passes the sim gate with the hold (above). Columns is outside this map's destination; it flies with `pre_release_hold_s:=0`. It is pinned by `test_the_default_pre_release_hold_empties_the_columns_cell`, and the tiny columns sweep tests now pass `pre_release_hold_s=0.0` (a new test-only `sweep()` override).
* 2026-09-29 09:03–09:08, `./run_tests.sh --full` with the new box → **FAIL, 2 failed / 5567 passed / 8
  skipped / 1 xfailed + serial 6/6** (`temp/logs/gate_full_r4d_20260929.log`). The two failures were
  `test_tiny_sweep_produces_one_box_inside_the_swept_grid` and its round-trip twin: the columns
  incompatibility above, and nothing else. Fixed as described, then
  `pytest tests/motion/test_skills_admissible.py -q` → 57 passed.
* 2026-09-29 09:30–09:35, `./run_tests.sh --full` (pre-commit, after the audit fixes) → **PASS**,
  parallel 5572 passed / 8 skipped / 1 xfailed + serial 6/6, 306 s
  (`temp/logs/gate_full_r4d_final_20260929.log`).

## Open Questions

- **The diagnostic sitting** is designed (ticket 05, answered with the owner 2026-09-29) as
  `tests/hardware/session_skills_r4_diag.md`: A. hop hold A/B (the owner contests the pre-release
  mechanism — the visible windup is −x; the executed commands show a fast +140 mm/s +x swing at
  −75 ms after it; the A/B decides); B. open-loop self-toss singles (true spread, re-aim off); C.
  carried-chain dwell 0.3 vs 0.6 s (owner: a wonky catch doesn't settle in 0.3 s); D. reload. A cup
  marker body is off the table for now (owner: hard to fit); offline release-side discriminators
  (scatter vs launch speed; is the ball visible in the cup pre-release) are running instead.
- **No fallback on a refused CATCH.** A refused reload CATCH leaves the hand parked at the PRE-TILT
  REST's own terminal (bottom of stroke, receive tilt) for an inbound ball the schedule already knows
  is coming (`Skill.landing_prior`). Should it instead fall back to a receive-height REST that at
  least lets the stroke absorb the impact? Raised in `result_t03.md` § 5, not decided.
- **Freeze lateral learning?** Ticket 01's evidence shows no lateral bias to learn on rest throws
  (mean (+6, +2), SEM (3.6, 5.3)) while the learner's `u_xy` wanders ±10 mm on noise and, at least
  once (S3 run 1 throw 4), commanded a correction that dropped a ball. Freezing `u_xy ≡ 0` (or strong
  shrinkage) while keeping apex learning is the implied fix. Owner (2026-09-29): frozen
  (`learner_lateral_authority_mm:=0`) for the diagnostic sitting; the launch default is decided after it.
- **One solver refusal still speaks jargon**: `cup_cycle._solve_qp`'s "QP infeasible: unbounded dual
  step admitting inequality %d (working set size %d)" (no `reason=`, so it reports `INFEASIBLE` where its
  two rewritten siblings report `SINGULAR`). Audit 2026-09-29; deferred to the next gated batch
  because a `cup_cycle.py` edit costs a 35-min box re-sweep.
- **Release-side discriminators, offline** (`.scratch/r4-throw-precision/evidence/result_release_side.md`,
  2026-09-29): QTM tracks the ball IN the cup at 152 Hz and over the last 150 ms of the stroke it
  drifts only 2.2–6.1 mm relative to the Platform body (5 throws, sitting 3), so the ~17/22 mm kick is
  concentrated at separation. On 2026-09-22 (the cleanest data, n=4–5 per apex) the lateral velocity σ is flat at
  17.4 → 18.0 mm/s from 0.6 to 0.9 m while the angle σ shrinks, which leans toward a fixed-size kick
  (lip or ball–cup contact) over axis wobble. The diag runsheet's block B2 decides it.
- **The early tracker fit reads about 9 mm short in x**, which drives some of the 20 mm re-aims that
  chase real error but with a biased trigger. De-biasing the early fit, or gating re-aims on a later
  or converged fit, would halve the noise in each re-aim decision — not attempted here.
