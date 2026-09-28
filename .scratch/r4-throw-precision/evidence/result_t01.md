# Ticket 01: self-toss landing error, decomposed (2026-09-29)

Bags: S1 `2026-09-27_22-37-26`, S2 `2026-09-28_21-41-10`, S3 `2026-09-28_23-55-19`. The `2026-09-28_19-53-37` bag holds no jugglebot throws.
There are 43 self-toss flights with a raw-mocap free-flight fit and 7 with none (chain ended or ball dropped). Hops are excluded here; they are ticket 02.
Probe: `tools/probes/selftoss_landing_decomposition.py`, which is new, reusable and uncommitted. It writes `temp/probes/selftoss_decomp_*.txt` plus an `.npz` raw cache per bag.

## Method

- **Ground truth L** comes from a gravity-fixed least-squares parabola. It is fitted to the raw `/mocap_data` ball markers (gated to the `/balls` track) from t_rel + 80 ms until 60 mm above the landing plane. The fit rms is 0.2 to 5.6 mm. This truth is independent of the tracker. It agrees with the log's OUTCOME `y` to 2 or 3 mm (for example, S3 #1 OUTCOME (-10.6, +16.7) against the probe's (-13.1, +14.6)).
- **Commanded A** is the announced landing, which is the schedule site plus the learner's `u`. `u` is joined from the `memory row appended` line.
- **E = L - A** is split as follows:
  - `p_rel - p_ann`: where the ball actually left from.
  - `needed = (A - p_rel)/T`: the lateral velocity that would have landed the ball on A from there.
  - `dv = v_fit - needed`, so E ≈ dv·T (T ≈ 0.86 s).
  - `cup`: what the platform could have given the ball, v_axial·tilt(mocap Platform body) + Platform xy velocity (-40..0 ms).
- The re-aim audit looks up the tracker's landing at the log time of each RESEND and compares it with both the truth and the committed aim.

## Per-throw table

"rest" is the first throw of an attempt: the platform is stationary and nothing was caught before it. "carried" is a throw released from the position where the previous ball was caught. All values are in the schedule/mocap frame. Tilt is at t_rel.

| sit | # | kind | u_xy mm, u_apex | E=L-A x,y mm | p_rel-p_ann mm | v_fit xy mm/s | needed xy | cup = v_ax*tilt+plat_v | tilt deg | speed ratio | apex m | tracker@1st re-aim - truth mm | re-aims (log) | caught |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| S1 | 1 | rest | -1.2,+20.0, 0.808 | **+28,-5** | -7,-4 | +41,+22 | +7,+28 | -15,+16 | -0.18,+0.27 | 1.061 | 0.883 | -16,-2 | 14.2 | True |
| S1 | 2 | rest | -1.2,+20.0, 0.808 | **-1,-14** | -2,-1 | -1,+9 | +1,+25 | -17,+17 | -0.24,+0.12 | 1.061 | 0.884 | -3,+1 | 8.7 | True |
| S1 | 3 | rest | -1.4,+20.0, 0.808 | **+29,+5** | -9,-7 | +43,+38 | +9,+32 | -17,+12 | -0.24,+0.12 | 1.063 | 0.885 | -27,-3 | 20.0 ; LIMIT_JERK 11.1 | True |
| S1 | 4 | rest | -1.4,+20.0, 0.808 | **+1,+0** | -3,-2 | +3,+26 | +2,+26 | -16,+19 | -0.23,+0.18 | 1.064 | 0.888 | -6,+1 | 20.0 | True |
| S1 | 5 | rest | -1.4,+20.0, 0.808 | **+1,-6** | -2,-3 | +2,+20 | +1,+27 | -16,+20 | -0.24,+0.16 | 1.065 | 0.887 | -2,-0 | 15.6 | True |
| S1 | 6 | carried | -1.4,+20.0, 0.808 | **+15,+93** | -5,-13 | +22,+147 | +5,+39 | +1,+11 | -0.17,+0.23 | 1.073 | 0.897 | -17,-31 | 20.0 | False |
| S1 | 7 | carried | | no free flight (chain ended / ball dropped) | | | | | | | | | | |
| S1 | 8 | carried | | no free flight (chain ended / ball dropped) | | | | | | | | | | |
| S1 | 9 | carried | | no free flight (chain ended / ball dropped) | | | | | | | | | | |
| S1 | 10 | rest | -1.9,+20.0, 0.809 | **-2,-23** | -2,-4 | -2,+1 | +0,+28 | -17,+22 | -0.25,+0.21 | 1.056 | 0.883 | -6,+1 | 9.0 | True |
| S1 | 11 | carried | -1.9,+20.0, 0.809 | **+20,-20** | -4,-1 | +26,+2 | +3,+25 | +36,+38 | -0.14,+0.29 | 1.067 | 0.891 | -13,+4 | 5.6 | True |
| S1 | 12 | carried | -1.9,+20.0, 0.809 | **-12,+42** | -2,-8 | -15,+82 | -0,+32 | -21,+9 | -0.21,+0.15 | 1.069 | 0.894 | +2,-13 | 20.0 | True |
| S1 | 13 | rest | -1.9,+20.0, 0.810 | **+3,-28** | -5,-2 | +7,-7 | +3,+26 | -16,+21 | -0.24,+0.19 | 1.071 | 0.900 | -14,+5 | 12.7 ; LIMIT_JERK 11.5 | True |
| S1 | 14 | carried | -1.9,+20.0, 0.810 | **+47,+7** | -6,-2 | +60,+34 | +5,+26 | +52,+31 | -0.13,+0.30 | 1.076 | 0.900 | -16,+4 | 20.0 | False |
| S1 | 15 | carried | -1.9,+20.0, 0.810 | **-2,+27** | -3,-5 | -2,+61 | +1,+29 | -34,+19 | -0.15,+0.28 | 1.070 | 0.897 | -1,+3 | 20.0 | False |
| S1 | 16 | carried | | no free flight (chain ended / ball dropped) | | | | | | | | | | |
| S1 | 17 | carried | | no free flight (chain ended / ball dropped) | | | | | | | | | | |
| S1 | 18 | rest | -2.0,+20.0, 0.810 | **+17,+14** | -6,-5 | +25,+46 | +5,+30 | -15,+23 | -0.21,+0.20 | 1.076 | 0.891 | -13,+2 | 20.0 | True |
| S1 | 19 | carried | -2.0,+20.0, 0.810 | **-11,+33** | +1,-6 | -16,+69 | -3,+31 | -13,+29 | -0.16,+0.22 | 1.065 | 0.889 | +8,-8 | 20.0 | True |
| S1 | 20 | carried | -2.0,+20.0, 0.810 | **-13,+16** | +1,-4 | -18,+47 | -3,+28 | +25,-17 | -0.16,+0.27 | 1.069 | 0.896 | +1,+0 | 20.0 | True |
| S1 | 21 | carried | -2.0,+20.0, 0.810 | **+43,+20** | -5,-4 | +54,+51 | +3,+28 | +57,-0 | -0.16,+0.32 | 1.067 | 0.896 | -12,+6 | 20.0 | False |
| S1 | 22 | carried | | no free flight (chain ended / ball dropped) | | | | | | | | | | |
| S2 | 1 | rest | +0.0,+0.0, 0.900 | **+9,-4** | -5,-6 | +15,+2 | +5,+6 | -17,-7 | -0.21,-0.09 | 1.080 | 0.988 | -10,-15 | 18.5 | True |
| S2 | 2 | rest | +0.0,+0.0, 0.900 | **+32,+48** | -8,-5 | +44,+58 | +9,+6 | -22,-3 | -0.25,-0.04 | 1.083 | 0.998 | -33,-12 | 20.0 ; 16.2 | True |
| S2 | 3 | rest | -9.2,-3.8, 0.826 | **-4,-7** | -0,-3 | -15,-9 | -10,-1 | -35,-9 | -0.42,-0.14 | 1.082 | 0.914 | -3,-3 | 17.0 | True |
| S2 | 4 | rest | -1.0,+3.0, 0.820 | **-21,+10** | -3,-5 | -22,+22 | +3,+10 | -20,-1 | -0.26,-0.02 | 1.087 | 0.910 | -6,-7 | 20.0 | True |
| S2 | 5 | rest | +6.9,-0.6, 0.817 | **-7,+28** | -1,-2 | +2,+34 | +9,+2 | -11,-7 | -0.13,-0.09 | 1.083 | 0.907 | -4,-4 | 20.0 | True |
| S2 | 6 | rest | +7.0,-7.0, 0.815 | **+37,-22** | -5,-4 | +57,-29 | +14,-4 | -2,-15 | -0.12,-0.24 | 1.084 | 0.902 | -20,-5 | 20.0 ; LIMIT_JERK 0.0 | True |
| S2 | 7 | rest | +3.0,-4.0, 0.815 | **-3,-34** | -1,+4 | +1,-48 | +4,-9 | -19,-8 | -0.25,-0.12 | 1.078 | 0.900 | -1,+9 | 20.0 | True |
| S2 | 8 | carried | +3.0,-4.0, 0.815 | **+36,-7** | -7,-22 | +53,+13 | +12,+21 | -18,+24 | -0.16,+0.24 | 1.084 | 0.905 | -13,-1 | 20.0 | True |
| S2 | 9 | carried | +3.0,-4.0, 0.815 | **+26,+4** | +16,-14 | +15,+17 | -15,+11 | -42,-6 | -0.61,-0.06 | 1.089 | 0.909 | -15,-7 | 12.3 | True |
| S2 | 10 | rest | -3.9,+0.9, 0.813 | **+5,+32** | -2,-3 | +3,+42 | -3,+4 | -23,-2 | -0.29,-0.05 | 1.089 | 0.904 | -0,-2 | 20.0 | True |
| S2 | 11 | carried | -3.9,+0.9, 0.813 | **+28,-25** | -2,+18 | +30,-49 | -3,-20 | -15,-57 | -0.32,-0.53 | 1.082 | 0.900 | -5,-5 | 20.0 | True |
| S2 | 12 | carried | -3.9,+0.9, 0.813 | **-38,+13** | +16,-28 | -68,+48 | -23,+33 | -52,+48 | -0.63,+0.48 | 1.081 | 0.905 | -7,-18 | 20.0 | True |
| S2 | 13 | carried | -3.9,-2.3, 0.813 | **+38,+43** | -27,-15 | +71,+64 | +27,+15 | +21,-11 | +0.15,+0.03 | 1.084 | 0.899 | -3,-6 | LIMIT_JERK 20.0 | False |
| S2 | 14 | carried | | no free flight (chain ended / ball dropped) | | | | | | | | | | |
| S2 | 15 | carried | | no free flight (chain ended / ball dropped) | | | | | | | | | | |
| S2 | 16 | carried | | no free flight (chain ended / ball dropped) | | | | | | | | | | |
| S2 | 17 | rest | -4.7,-2.5, 0.813 | **-5,+37** | -3,-4 | -8,+46 | -2,+2 | -20,-5 | -0.28,-0.08 | 1.075 | 0.889 | +1,-6 | 20.0 | True |
| S2 | 18 | carried | -4.7,-2.5, 0.813 | **-23,+31** | -8,+15 | -23,+15 | +3,-21 | -8,-51 | -0.19,-0.45 | 1.086 | 0.910 | +5,-10 | 20.0 | True |
| S2 | 19 | carried | -4.7,-2.5, 0.813 | **+13,-8** | -27,+12 | +41,-26 | +26,-17 | +16,-26 | +0.10,-0.38 | 1.083 | 0.899 | -10,-5 | 16.6 | True |
| S2 | 20 | carried | -3.9,-4.7, 0.814 | **+10,+16** | -8,-22 | +16,+37 | +4,+19 | -30,+21 | -0.28,+0.18 | 1.085 | 0.901 | - | - | True |
| S2 | 21 | carried | -2.5,-6.2, 0.813 | **+1,+2** | -3,-0 | +1,-5 | +1,-7 | -28,-18 | -0.33,-0.27 | 1.087 | 0.904 | -8,+7 | 10.8 ; LIMIT_JERK 10.1 | True |
| S3 | 1 | rest | +0.0,+0.0, 0.900 | **-13,+15** | -2,-6 | -12,+24 | +3,+7 | -12,-9 | -0.09,-0.12 | 1.055 | 0.978 | -7,-7 | 20.0 | True |
| S3 | 2 | carried | +0.0,+0.0, 0.900 | **-1,+8** | -21,+4 | +22,+4 | +23,-4 | +23,-21 | +0.24,-0.28 | 1.058 | 0.985 | +2,+0 | 6.8 | True |
| S3 | 3 | carried | +0.0,+0.0, 0.900 | **+7,-21** | -2,+3 | +9,-26 | +2,-3 | -10,-36 | -0.21,-0.25 | 1.056 | 0.983 | -13,-3 | 20.0 | True |
| S3 | 4 | carried | +8.7,-13.6, 0.835 | **+56,+27** | -14,-21 | +90,+39 | +26,+9 | +35,+29 | +0.26,+0.21 | 1.063 | 0.923 | -29,-0 | 20.0 | False |
| S3 | 5 | carried | | no free flight (chain ended / ball dropped) | | | | | | | | | | |
| S3 | 6 | rest | -6.2,-4.8, 0.830 | **+7,-16** | -4,-0 | +6,-24 | -3,-5 | -16,-14 | -0.15,-0.20 | 1.050 | 0.900 | -12,+6 | 16.3 | True |
| S3 | 7 | carried | -6.2,-4.8, 0.830 | **+23,-24** | -13,-16 | +35,-15 | +8,+14 | +8,+10 | +0.02,+0.05 | 1.054 | 0.907 | -6,+6 | 20.0 | True |
| S3 | 8 | carried | -6.2,-4.8, 0.830 | **+29,-2** | +6,-23 | +20,+19 | -14,+21 | -31,-14 | -0.48,+0.12 | 1.060 | 0.913 | -12,-3 | 10.6 | True |
| S3 | 9 | carried | -7.1,+1.8, 0.830 | **+10,+5** | +7,-11 | -4,+21 | -16,+15 | -34,+9 | -0.44,+0.16 | 1.065 | 0.913 | -10,+3 | 9.0 ; LIMIT_JERK 10.1 | True |
| S3 | 10 | carried | -10.6,+6.4, 0.828 | **-7,+12** | -11,+8 | -8,+12 | -0,-1 | -28,-11 | +0.10,-0.07 | 1.063 | 0.913 | -2,-3 | 19.6 | True |

## Verdict: scatter, not bias. The dominant channel is the ball's lateral launch velocity, which the platform does not explain.

**Size of the error.**

| set | n | mean E (x, y) mm | 1σ (x, y) mm | radial 1σ |
|---|---|---|---|---|
| rest (no catch before; nothing re-aimed can have touched it) | 19 | +6.0, +1.7 (SEM 3.6, 5.3) | **15.8, 23.0** | 27.9 |
| carried (released from the caught xy) | 24 | +12.3, +12.2 | 23.5, 26.1 | 35.1 |
| all | 43 | +9.5, +7.6 | 20.5, 25.1 | 32.4 |

Per sitting, rest throws only:
- S1: (+9.7, -7.1), σ (13.1, 14.1).
- S2: (+4.7, +10.0), σ (18.6, 28.4).
- S3 has only 2 rest throws.

The means sit within about 1–2 SEM of zero and do not repeat in direction across sittings. **The rest-throw 1σ of 16–23 mm per axis is the error with re-aim contributions removed.** It is 1.6–2.3× the ±10 mm target.

**The error is a velocity error, not a position error.**
- For rest throws, `p_rel - p_ann` has σ (2.6, 2.5) mm, so where the ball leaves from barely varies.
- The launch velocity error dv has σ (18, 26) mm/s. Over T ≈ 0.86 s that reproduces E.

**The platform does not explain it.** This is the rest-throw landing 1σ that each platform term would produce, measured per sitting:

| term | contribution to landing 1σ |
|---|---|
| mocap Platform tilt variation (sd 0.02–0.09°) | 1.4–5.8 mm |
| Platform xy velocity at release | 0.4–3.4 mm |
| release position | 1.0–4.3 mm |
| **root-sum-square of the three** | **≤ ~7 mm** |
| observed | 13–28 mm |

- A regression of v_fit on the cup velocity the platform could have given (v_axial·tilt + platform velocity) gives r = 0.46 in x and 0.29 in y for rest throws. The residual sd is 19 and 27 mm/s, against 22 and 28 mm/s raw. The platform state explains little of the ball's lateral velocity.
- **So at least about 20 mm of the 1σ enters between the platform and the ball: ball–cup contact at separation, or lateral compliance of the hand carriage during the roughly 3 g stroke.** The bag cannot separate those two, because there is no cup or hand marker.
- The owner's premise ("a well-positioned platform throws repeatably") is therefore only half right. The platform *is* already well positioned on rest throws, to 7 mm or better, and the balls still scatter by about 20 mm.

**Terms ruled out or secondary:**
- **Separation lag.** The ball leaves 51 ± 14 mm below the planned release point, a late, soft separation on every throw. It is uncorrelated with |dv| (r = -0.05), so it is not the scatter source by itself.
- **Frame offset.** The tracker-to-schedule correction is ≤ 2 mm, and the rest `p_rel - p_ann` mean is (-3.7, -3.4) mm.
- **Mocap tilt reading.** The mocap Platform body reads a constant -0.12..-0.25° x tilt at rest. The balls do not show it: it would give a -9..-19 mm x bias, and the measured bias is +6. It is a marker-definition offset, not a real tilt.
- **Hand speed ratio.** The ratio is 1.065 ± 0.006 (S1), 1.084 ± 0.004 (S2) and 1.058 ± 0.005 (S3). It is an apex bias with almost no within-sitting scatter, so the apex is not a precision problem once learned.
- **Carried throws.** These add scatter (radial 35 against 28 mm) through two channels:
  - **The platform is still translating toward the site at release.** Cup velocity against release offset has a slope of -1.67 mm/s per mm (r = -0.85). The plan's fly-back needs about -1.1, and the plan claims a residual centroid speed of 7 mm/s or less. The measured Platform velocity is up to 17 mm/s, for example S2 carried: plat v_y -17 at an offset of +18 mm. The result is a weak over-fly-back: E against E_pos has slope -0.51 in x and -0.26 in y (r -0.28 and -0.21, n = 16).
  - **The just-caught ball is presumably still moving in the cup.** This is the S1 mechanism: the S1 carried mean y is +27 mm, before fix A.

## Re-aims chase real error (though the fit that triggers them is biased by about 9 mm)

Across 48 RESENDs, tracker landing at the re-aim instant minus truth is **(-8.8, -3.6) mm mean, sd (8.4, 7.4)**:
- The early fit is systematically about 9 mm short in x. The converged last fit comes within 1–6 mm.
- Of the 26 re-aims clamped at 20 mm, the ball's true error exceeded 20 mm on **22**.
- The median |fit - truth| is 9.9 mm, against a median |truth - A| of 28.1 mm.

So the 20 mm clamp is being hit by real 20–55 mm landing errors, and fit noise is not what fills the authority. With re-aim OFF at today's scatter, about a third of the balls would land more than 30 mm off the cup.

## Learner memory: it compensates the apex and compounds the lateral error

- **Apex: it compensates correctly.**
  - S2: u_apex 0.900 → 0.813 brought the apex from 0.99–1.00 m to 0.90–0.91 m.
  - S3: 0.900 → 0.830 brought it from 0.98 m to 0.90–0.913 m.
  - The speed ratio drifts from sitting to sitting (1.058–1.084, about ±5 % in apex gain), so apex rows should not carry across sittings. The next sitting starts cold anyway.
- **Lateral: it compounds.** There is no lateral bias to learn (rest mean (+6, +2), SEM (3.6, 5.3)). Yet u_xy wanders by about ±10 mm on noise:
  - S2: u_x runs -9.2 → -1.0 → +6.9 → +3.0 → -3.9 → -4.7 across one sitting.
  - S3 run 1, throw 4: the learner commanded (+8.7, -13.6) after three rows whose y mean was +0.4. That ball landed (+64, +12) and was dropped.
  - This matches a local fit driven by 20–25 mm row noise with few rows. I did not trace the learner code.
  - S1's u_y = +20 was the stale pre-kincal bias, already quarantined.

## The fix the evidence implies

1. **Freeze lateral learning** (u_xy ≡ 0, or strong shrinkage) and keep apex learning. This removes the ±10 mm the learner adds and costs nothing, because no lateral bias exists.
2. **Keep the in-flight re-aim** until the release channel is fixed; it is catching real error. De-bias the tracker's early fit (-9 mm x) or gate re-aims on a later or converged fit. That halves the noise in each re-aim decision and should reduce the jerk-refused re-sends.
3. **Carried throws:**
   - Hold the platform stationary at release. The measured translation at release is up to 17 mm/s, against the plan's 7 mm/s or less.
   - Give the caught ball settle time before the stroke.
4. **Precision to ±10 mm needs a release-side fix, not a platform-side one.** Candidates, ranked for the sitting to decide:
   - (a) ball seating and in-cup motion, fixed by settle time, cup geometry, or a crisper separation (a stronger deceleration after the release knot);
   - (b) lateral compliance of the hand carriage, fixed by stiffening, or a feedforward if it is repeatable per stroke.

## What only a sitting can settle (about 30 min of the diagnostic sitting)

| test | protocol | deciding number |
|---|---|---|
| A: seating | 12 rest self-tosses at 0.9 m. Re-aim OFF (lateral authority 0), lateral learner frozen (u_xy = 0), apex warm-started in the same sitting. The operator centres the ball and waits 3 s before each throw. | rest 1σ per axis from this probe. **≤ 10 mm**: the platform is enough and today's scatter was seating; fix seating and settle time. **≥ 15 mm**: the scatter is in the release dynamics; go to B. |
| B: channel | Put a 3-marker "Hand" rigid body on the cup or carriage; 12 more rest throws. | Compare the sd of the cup's lateral velocity at separation (from mocap, with the ball fit's separation instant) with the ball's sd of about 20–25 mm/s. **Cup sd ≥ 15 mm/s**: carriage compliance, a mechanical fix. **Cup sd ≤ 5 mm/s**: ball-in-cup, fix the cup geometry or separation profile. |
| C: carried | 3 chains × 5 throws with re-aim OFF. | Carried 1σ against A's rest 1σ. **More than 5 mm worse**: post-catch settling and the release-time translation are the carried-throw lever. |

## Probe commands run: (date, command, result)

- **2026-09-29**, `python tools/probes/release_motion_bag_probe.py ~/Desktop/rosbags/2026-09-28_23-55-19 --out temp/probes/release_motion_20260928_2355.txt`
  - 15 announcements in 97 s. Self-toss platform tilt at release was 0.05–0.7°.
  - The fitted |v| was 4.0–4.5 m/s. The parabola sat 48–94 mm below the announced release point.
  - It showed carried releases at the caught xy, for example (-61, -15) against the site's (-50, 0).
- **2026-09-29**, `python tools/probes/selftoss_landing_decomposition.py ~/Desktop/rosbags/<bag>`, run in parallel for each of the 4 bags to build the `.npz` caches.
  - OK. `2026-09-28_19-53-37` holds 0 jugglebot throws.
  - S1 speed ratio 1.067 ± 0.006, S2 1.084 ± 0.004, S3 1.058 ± 0.005.
- **2026-09-29**, `python tools/probes/selftoss_landing_decomposition.py ~/Desktop/rosbags/2026-09-27_22-37-26 ~/Desktop/rosbags/2026-09-28_21-41-10 ~/Desktop/rosbags/2026-09-28_23-55-19 --log temp/logs/skills_r4_20260927_2237.log --log temp/logs/skills_r4_20260928_2141.log --log temp/logs/skills_r4_20260928_2355.log`
  - Output: `temp/probes/selftoss_decomp_2026-09-27_22-37_2026-09-28_21-41_2026-09-28_23-55.txt`.
  - 43 fitted self-tosses. E 1σ was rest (15.8, 23.0), carried (23.5, 26.1), all (20.5, 25.1) mm.
- **2026-09-29**, `python <scratchpad>/t01_stats.py`
  - Output: `scratchpad/t01_stats.out`.
  - Rest v_fit against cup: r 0.46 / 0.29, residual sd 19.4 / 27.2 mm/s.
  - Carried cup against E_pos: slope -1.67 / -1.65 (r -0.85 / -0.87).
  - Fit at RESEND minus truth: (-8.8, -3.6) sd (8.4, 7.4), n = 48. 22 of 26 clamped re-aims had a true error above 20 mm.
- **2026-09-29**, `python <scratchpad>/t01_extra.py`
  - Output: `scratchpad/t01_extra.out`.
  - Separation-lag z offset -51.0 ± 14.4 mm, corr with |dv| -0.05.
  - The rest platform-term budget is ≤ 5.8 mm per term.
- **2026-09-29**, `python <scratchpad>/t01_table.py`
  - Output: `scratchpad/t01_table.md`, which is the table above.

Caveats:
- The mocap Platform state is sampled at t_rel, not at the true separation (+10–40 ms). The platform is stationary on rest throws, so this matters mainly for carried ones.
- The hand encoder and FK were not used. The Platform body is the realised pose, and the hand's axial speed enters only through the fitted launch velocity.
- The planned release tilt of carried throws was inferred (fly-back ≈ -r/T), not replanned. `planned_release_motion.py` can confirm it if the -1.67 against -1.1 gap matters.
