# Ticket 02 result: hop +x overshoot. The platform is still sliding +x under the ball when it separates

Date: 2026-09-29. Bags: sitting 1 `2026-09-27_22-37-26` (3 hops, no post-release hold) and sitting 3 `2026-09-28_23-55-19` (3 hops, 50 ms hold). Sitting 2 `2026-09-28_21-41-10` has **no hop announcement**: all 21 are self-tosses, because the hop THROW was refused. Scripts are in this scratchpad: `cache.py`, `decomp.py`, `sep.py`, `planfine.py`, `legfk.py` and `regr.py`. No production code was changed.

## Verdict (one line)

**This is not a tilt error.** At the planned release the platform is **translating at +98..+129 mm/s in +x** (mocap). The plan asks for ~0 mm/s there. The ball inherits that velocity at separation, so the ball's velocity perpendicular to the cup axis is +44..+98 mm/s against a planned 0. That velocity times ~0.9 s of flight is the ~+50..+100 mm overshoot. The platform tilt at release matches the plan to within 0.07°.

## Per-throw table

Column definitions:
- **ann v**: the announced (commanded) cup velocity. Its angle from vertical is the planned axis tilt, because the plan holds the platform at 0 at the release knot.
- **tilt**: the mocap `Platform` body tilt at t_rel.
- **plat vx**: the mocap Platform origin velocity in x (quadratic fit, ±20 ms) at t_rel, +10 ms and +20 ms. It is on the same clock as the ball.
- **ball ⊥**: the ball's launch-velocity surplus perpendicular to the realised axis, (Δvx·cosθ − Δvz·sinθ), from a gravity-fixed fit over 0.03–0.45 s.
- **Δapex**: the realised apex above the release point minus the commanded apex.
- **land err x**: the skill_node OUTCOME value (sitting 3) or the logbook value (sitting 1); my 0.03–0.45 s fit is in brackets.
- **lag instr.**: the probe's parabola-vs-release-point at t_rel (negative means late).

| sitting / t_rel | ann v (x,z) mm/s | ann angle | tilt @t_rel | plat vx @0 / +10 / +20 | ball v fit (x,z) | ball ⊥ | Δapex | land err x | lag instr. |
|---|---|---|---|---|---|---|---|---|---|
| S1 13416.686 (no hold) | 294, 4130 | 4.07° | 3.93° | 130 / 103 / 71 | 387, 4640 | +58 | +55 mm | +87 (+87) | −172 |
| S1 13430.536 (no hold) | 273, 4100 | 3.80° | 3.69° | 98 / 84 / 69 | 389, 4387 | +97 | +41 | +102 (+101) | −82 |
| S1 13450.486 (no hold) | 276, 4047 | 3.90° | 3.77° | 129 / 97 / 60 | 390, 4292 | +98 | +55 | +104 (+104) | −48 |
| S3 03856.036 (hold) | 284, 4282 | 3.79° | 3.72° | 118 / 94 / 71 | 390, 4562 | +88 | +59 | n/a (+102) | −67 |
| S3 04148.911 (hold) | 274, 4258 | 3.68° | 3.61° | 120 / 94 / 69 | 336, 4539 | +44 | +40 | **+88** (+62) | −87 |
| S3 04157.811 (hold) | 280, 4167 | 3.84° | 3.81° | 129 / 102 / 73 | 396, 4550 | +90 | +32 | **+110** (+107) | −148 |

**Planned values at the release knot.** The offline hop from `planfine.py` gives these values at the release knot, with centroid-frame values from `plan.pose`:
- centroid vx is +0.3 mm/s and the tilt is 4.006°;
- the pre-release swing is +106 mm/s at −75 ms, +95 at −50 and +34 at −25;
- the hold is −0.5 and −2.7 mm/s at +25 and +50 ms;
- the return is +36 at +75 and +88 at +100.

In the FK/mocap origin frame, which is a different reference point from the plan's pose point, the executed leg commands (`/leg_cmd_executed`) FK'd give a commanded origin vx of +140 at −75 ms, +60 at −25 and **+22 at 0** (a 25 ms central difference). They give **+9 / +7 over the hold** and +34 / +56 at +75 / +100.

**Tracking.** The same instants FK'd from `/robot_state` encoders give a measured origin vx of **+89..+91 mm/s at t_rel**, peaking ~25 ms after the commanded peak. Mocap reads +118..+129 at t_rel and peaks ~40–50 ms after the command. The extra lag against the encoder FK is the QTM-vs-Jetson clock offset plus mocap latency. Measured FK and mocap positions agree to within ~2 mm.

## Answers to the key questions

- **Did the hold appear in the realised platform motion?** It appears in the **commands** (cmd vx ≤ 9 mm/s over +25..+50 ms), but not in the realised motion. The realised platform is still decelerating from the pre-release swing through the entire hold window: mocap vx is 120 → 94 → 69 → 47 → 32 → 14 mm/s over 0..+50 ms, and reaches 0 at about +60 ms. It then undershoots to −20..−30 mm/s at +75..+100 ms.

  **Sitting 1 (no hold) is indistinguishable**: +98..+130 mm/s at t_rel and +87..+104 mm landings. The hold changed nothing, because the surplus was never the post-release return. It is the **tail of the pre-release forward swing, delayed by leg tracking lag**. This corrects the sitting-1 attribution, which read the ~+95 mm/s surplus as the plan's post-release re-acceleration. In fact the realised platform never re-accelerated within 80 ms in sitting 1 either.
- **How late is the release?** Across 38 throws, the best transfer (below) puts separation at about **+10..+20 ms after the planned knot on the mocap clock**. The release-lag instrument reads −48..−172 mm (late) on every hop. Early ball samples are too timestamp-jittery (several ms) to pin separation per throw. Hand telemetry peaks ~20 ms after the plan on the log clock for hops and self-tosses alike, which does not separate transport latency from real hand lag.
- **Does the ball follow the cup velocity or the cup axis?** It follows the **realised cup velocity**. I regressed the ball's perpendicular velocity surplus on mocap platform vx·cosθ over **38 throws**: 32 self-tosses, 6 hops, both sittings, with and without the hold.
  - At +10 ms the slope is **0.83**, the intercept **+1.3 mm/s** and **R² 0.70**.
  - At +20 ms the slope is **1.17**, the intercept −3.8 and R² 0.65.
  - The self-toss-only slope is 1.18 at +10 ms and 0.98 at +20 ms.

  The ball inherits the platform's translational velocity at separation, one for one, with no offset. The self-tosses carry the same law: S1's two re-aimed releases with the platform at +55 / +47 mm/s landed +40 / +30 mm.
- **Is the extra apex the hand overspeeding, and does the x error scale with it?** The apex surplus is real (+32..+59 mm, 3.5–6 %). It is **the same on self-tosses**: learner rows show y_apex − u_apex = +0.08 m on self-toss and +0.086..+0.09 m on hops. It does **not** scale with the x error: the hop with the smallest Δapex (+32) had the largest overshoot (+110).

  An overspeed along the 3.7° axis adds only Δ|v|·sinθ ≈ 190·0.065 ≈ **+12 mm/s** of vx, or ~11 mm of landing. The longer time of flight from the higher apex (~+0.02 s × 280 mm/s) adds ~6 mm. Together (c) accounts for ~15–20 mm of the ~100.

  **Budget**: 85 (⊥ platform) + 11 (axis overspeed) + 6 (tof) ≈ 102, against a measured +102..+110. Hop 2 is the outlier: ⊥ is only +44 in my short-window fit but +88 in the production fit. That points to fit-window sensitivity, or a later separation on that throw (~+25 ms). Ball stamps are too jittery to resolve which.
- **Learner rows 10 → 11.** Row 10 (the evening's first hop, u = (0, 0, 0.95)) landed at +0.117 m. The learner then commanded **u_x = −0.010 m (−10 mm) on both later hops**. That is the clipped lower bound of the hop box, so it pushed in the right direction and hit its authority limit. It also lowered the commanded apex from 0.95 to 0.939 to 0.9005. Row 11 (+0.088) did not change x, which was already clipped. The learner neither caused the overshoot nor can absorb it: 10 mm of authority against a 100 mm error.

## Mechanism verdict against (a)–(e)

- **(a) Realised tilt ≠ planned tilt: REFUTED as stated.** The mocap tilt at t_rel is within 0.03–0.14° of the announced axis angle on all six hops, with a mean of −0.09°. The error would need ~+1° the other way. What the owner sees as "tilted" is real: the ball leaves at 4.2–5.2° against a 3.7–4.1° axis. That +0.5..+1.3° velocity-direction error comes from **platform translation**, not orientation. Tilt rate at release is ±1–2 °/s, so its lever term is ±10–20 mm/s, second order.

  **(a) as a translation-tracking lag: CONFIRMED.** This is the mechanism. The legs lag the commanded pre-release forward swing: encoder FK lags ~25 ms, mocap ~40–50 ms. The plan decelerates the centroid from +106 mm/s to 0 in the last 75 ms before the knot, so the realised platform is still at ~+90..+130 mm/s at the knot and ~+70..+100 at separation.
- **(b) The ball follows cup velocity and the plan's velocity is wrong: HALF.** The ball does follow the cup velocity (slope ≈ 1, R² 0.7, n = 38). The plan's velocity is right (0 at the knot). The **realisation** is wrong.
- **(c) Hand overspeed: minor, ~15–20 mm of ~100.** It is common to the self-tosses, where it only moves the apex. It does not scale with the x error.
- **(d) Levelling or frame transform on the lateral component: REFUTED.** The world-frame tilt matches the plan, and the commanded-leg FK and mocap agree. The 977cbb9 hold-axis change cannot matter: sitting 1 (before it) and sitting 3 (after it) are identical.
- **(e) Release timing: CONTRIBUTING, not independent.** Separation is ~10–20 ms after the knot. On a stationary cup that costs nothing (self-toss). Here it only enlarges (a): it samples the decaying platform velocity a little later.

## Implied fix

**The platform must be still before the ball separates, not only after it.** Add a **pre-release platform hold**: the platform pose (translation and tilt) is frozen over the last N knots of the THROW stroke, so the hand alone does the final acceleration. This mirrors `POST_RELEASE_HOLD_S`, and the two holds together bracket the release as [t_rel − N·25 ms, t_rel + 50 ms].

**Sizing from the bag:**
- the realised mocap vx takes ~50–60 ms after the commanded vx reaches ~0 to fall below ~15 mm/s;
- separation is +10..+20 ms after the knot;
- so the commanded platform must be at rest **≥ ~75–100 ms before the knot: N = 3–4 knots**.

**Cost:**
- the windup and tilt must finish earlier, with higher leg acc and jerk in the windup;
- the hop THROW already plans at 129k of the 150k mm/s³ jerk limit;
- so start the tilt earlier in the 0.62 s THROW window rather than compress it;
- this needs an offline feasibility check at the operating point and the box re-sweep (feasibility/segments edits).

**Complementary, not a substitute:** leg acceleration FF (`accel-ff-inertia.md`) would cut the ~25–50 ms leg lag, which is the root of the residual velocity.

**Rejected:** widening the hop box to absorb +100 mm. It aims around a plant velocity that varies with lag (+44..+98 mm/s ⊥ spread) and would hide the class. The same class also hits self-tosses whose platform is still moving at release (re-aimed catches), which is relevant to ticket 01's ±10 mm target.

## Robot test that decides it

This is one block of the diagnostic sitting: 5 hops at the current code (arm A) and 5 hops with a pre-release hold of N = 4 (arm B). It needs no extra instrumentation: the same bag topics, `/rigid_body_poses` and `/balls`, then `decomp.py` + `regr.py`.

- **Deciding number:** the mocap Platform vx at t_rel + 10 ms.
  - Arm A is predicted at 84..103 mm/s, as now.
  - **Arm B is predicted at ≤ 20 mm/s, with the ball's perpendicular surplus ≤ 25 mm/s and a mean hop landing x error within ±25 mm** (now +87..+110).
- **Refutation criterion:** if arm B's platform vx at +10 ms is ≤ 20 mm/s but the landing error stays ≥ +60 mm, this mechanism is wrong. The next suspect is then cup-ball contact in the tilted cup, and the 38-throw transfer law would need re-reading.
- **Zero-code pre-check before the sitting:** the prediction for the current code is already testable offline on any new hop bag. The ball's perpendicular surplus should equal 0.8–1.2 × the mocap platform vx at +10..+20 ms.

## (date, command, result) triples

- 2026-09-29, `python tools/probes/release_motion_bag_probe.py ~/Desktop/rosbags/2026-09-28_23-55-19 --out <scratch>/rm_s3.txt`: 15 announcements. The hops at 04148.911 and 04157.811 had platform tilt 3.61 / 3.83° at t_rel and landing error on the probe's plane of +61.8 / +106.6 mm. The release-lag instrument read −86.8 / −147.9 mm.
- 2026-09-29, `OPENBLAS_NUM_THREADS=1 python tools/probes/planned_release_motion.py` (and `<scratch>/planfine.py`, which dumps every knot): the hop plan's centroid vx was +106.1 at −75 ms, +95.2 at −50, +33.9 at −25 and +0.3 at the knot. Tilt at the knot was 4.006°. The HOLD verdict was 2.74 mm/s, and the return was +36.2 at +75 ms.
- 2026-09-29, `python <scratch>/decomp.py <scratch>/s3.npz` and `… s1.npz` (bags cached by `<scratch>/cache.py`): mocap platform vx at t_rel was +118 / +120 / +129 mm/s (S3) and +130 / +98 / +129 (S1). Self-tosses were −20..+55.
- 2026-09-29, `python <scratch>/legfk.py ~/Desktop/rosbags/2026-09-28_23-55-19 <scratch>/s3.npz 1790604148.911,1790604157.811,1790603856.036`: commanded-leg FK vx at t_rel was +22 / +21 / +23 mm/s, and the hold knots read +9 / +7. Encoder FK read +90 / +91 / +89 and mocap +120 / +125 / +118. The encoder-FK peak lags the command by ~25 ms.
- 2026-09-29, `python <scratch>/regr.py`: n = 38. At +10 ms the ball ⊥ surplus vs platform vx gave slope 0.83, intercept +1.3 mm/s and R² 0.70. At +20 ms the slope was 1.17 and R² 0.65. The self-toss-only slope was 1.18 (+10 ms) and 0.98 (+20 ms).
- 2026-09-29, `grep "memory row appended" temp/logs/skills_r4_20260928_2355.log`: the hop rows commanded u_x = −0.010 on both later hops (clipped), with u_apex 0.939 / 0.9005. The results were y = (+0.088, +0.028, 1.025) and (+0.110, −0.017, 0.987).

`<scratch>` = `/tmp/claude-1000/-home-jetson-Desktop-Jugglebot-skills/157654d5-f8cc-42f2-b835-1fa13ecee62a/scratchpad`. `decomp.py` and `regr.py` are candidates for promotion to `tools/probes/release_velocity_budget_probe.py` if the fix lands.

## Reconciliation with ticket 01 (2026-09-29, `python <scratch>/recon.py`, self-tosses from all three bags S1/S2/S3)

Rest = the first throw of an attempt (announced < 0.8 s ahead). Carried = a throw from a catch (announced ~1.3 s ahead). Method:
- The ball's perpendicular surplus is (Δv_xy − n_xy/n_z·Δv_z) against the realised mocap axis n.
- The platform velocity is the mocap quadratic fit at **t_rel + 15 ms**.
- Landing-equivalent = velocity 1σ × the mean time of flight.

| | n | landing 1σ x / y (mm) | ball ⊥ surplus 1σ x / y (mm/s) | platform v@+15 ms mean ± 1σ x / y (mm/s) | residual after transfer law 1σ x / y (mm/s → mm) | corr(ball ⊥, platform) x / y |
|---|---|---|---|---|---|---|
| rest | 19 | 13.8 / 20.5 | 19.1 / 25.2 | +8.3 ± 3.5 / +5.7 ± 2.2 (median \|v\| 9.7, max 17.8) | 19.4 / 25.3 → **16.7 / 21.7 mm** | 0.03 / 0.00 |
| carried | 34 | 28.8 / 32.1 | 25.5 / 46.6 | +11.6 ± 9.7 / +13.1 ± 17.2 (max 65.6) | 23.0 / 45.7 → 17.5 / 34.9 mm | 0.44 (slope 1.16) / 0.24 (slope 0.64) |

1. **Rest self-toss.** After subtracting the transfer law at +15 ms, the residual is **19 / 25 mm/s, or 17 / 22 mm of landing**. That is essentially all of the scatter: the transfer law removes nothing, because the platform barely moves (1σ 3.5 / 2.2 mm/s, which is ≤ 3 mm of landing). Sampling the platform at +15 ms instead of at the knot does not change ticket 01's conclusion.
2. **The y error is not platform y.** On rest throws corr(ball ⊥y, platform vy) = 0.00. On carried throws the correlation is 0.24 (slope 0.64): the platform y motion from lateral re-aims adds some, but most of the 46 mm/s y scatter is not platform.
3. **Is the platform moving at release on rest throws?** Not meaningfully. It shows a small consistent bias (+8 / +6 mm/s mean, |v| median 9.7, max 18 at +15 ms; max 31 at the knot) with a tiny spread. That bias is at most ~7 mm of a mean landing offset, and the learner absorbs it. It cannot produce the 1σ.
4. **Fit noise does not explain the residual.** The lateral-velocity noise of a gravity-fixed fit over 0.03–0.45 s is about 5 mm/s (≈ 4 mm of landing) against a residual of 19–25 mm/s. My rest landing 1σ (13.8 / 20.5) agrees with ticket 01's production 1σ (15.8 / 23.0).

**Answer: a still platform at separation does not get the self-toss to ±10 mm.** It fixes the hop's +100 mm bias and the carried-throw platform term. On rest throws, which already have a still platform, a release-side residual of ~17 / 22 mm (1σ) remains that is uncorrelated with platform motion, and it is larger in y. The candidates lie in the ball–cup release: the ball's seat in the cup, lateral play or rolling under the ~100 m/s² stroke, cup-lip contact at separation, and the hand axis not being coaxial with the platform normal. They need a release-side instrument, for example the ball's lateral position in the cup before the stroke against the lateral ⊥ velocity. Platform timing does not reach them.
