# R4 diagnostic sitting: the hold A/B, the true self-toss spread, and the reload catch

Wayfinder map `.scratch/r4-throw-precision/map.md`, ticket 05 (designed 2026-09-29 with the owner).
Evidence and reasoning: `logbook/2026-09-29-skill-stack-r4-sitting-3-analysis.md`.

This sitting **gathers data**; it does not attempt the R4 gate. Every block has a hypothesis and the
number that decides it. **If a block misses its deciding number, stop that block and move on: no
on-robot debugging.** The deciding numbers marked (offline) are read by Claude from the bag after the
sitting (`tools/probes/release_motion_bag_probe.py`, `tools/probes/selftoss_landing_decomposition.py`).
The ones marked (live) you can read from the `OUTCOME` lines as you go.

**If your physical intuition disagrees with a block's framing, say so: that is load-bearing.** The
hop hypothesis especially is contested (owner, 2026-09-29: the visible windup is −x, so how can
pre-release motion push the ball +x?). Block A is the test that settles it either way.

Budget: about 50 min after bring-up.

## 0. Bring-up

Run `session_skills_r4.md` §§ 1–3 unchanged (colcon build, `--full`, gate-hash check, bring-up,
`level` first, dress rehearsal), with these differences:

| # | Step | Expect |
|---|---|---|
| 0.1 | § 1 row 3: the live gate hash and the box's | Both `ad36fa53aab2`. The box was re-swept 2026-09-29 for the pre-release hold. It has **no columns boxes** (known, recorded); hop and self_toss are present. |
| 0.2 | Launch as usual: `pre_release_hold_s` defaults to **0.100** | `trajectory_node` logs `segment config: pre_release_hold_s=0.100 s, post_release_hold_s=0.050 s`. |
| 0.3 | § 2 row 10 with a FRESH `plant_id r4diag-$(date +%Y%m%d)` | Cold learner memory (owner Q3). |
| 0.4 | `ros2 param set /skill_node learner_lateral_authority_mm 0.0` | Lateral learning frozen for the whole sitting; apex learning stays on (owner Q2). |
| 0.5 | `ros2 param set /skill_node catch_resend_max 0` ; `ros2 param set /skill_node catch_aim_source schedule` | Every catch aims open-loop at the schedule landing, with no in-flight re-aim: the destination's own criterion. Expect some drops in Block B. That is the measurement, not a failure. |
| 0.6 | Reload listening check (`session_skills_r4.md` step 27) | `ros2 service call /bb/aim …` answers. |

## A. Hop A/B: does holding the platform still before release remove the +x overshoot? (≈ 12 min)

Hypothesis (ticket 02): the platform is still sliding +x at about +100 mm/s when the ball separates,
the tail of a fast +x swing in the last ~120 ms, smeared by 25–50 ms of leg lag, and the ball
inherits it. The pre-release hold (platform still for the last 100 ms, hand does the final push)
should remove it. Owner's alternative: orientation, or post-throw motion bleeding in.

`ros2 param set /skill_node separation_mm 250.0` first.

| # | Step | Expect / deciding number |
|---|---|---|
| A1 | `ros2 param get /trajectory_node pre_release_hold_s` | `0.1`. |
| A2 | 5 × `ros2 action send_goal /jugglebot/juggle jugglebot_interfaces/action/Juggle "{pattern: hop, apex_m: 0.9, separation_mm: 250.0, num_cycles: 1}"`. Reset the ball to P1 between throws. | (live) OUTCOME landing x. **Hold works: mean within ±25 mm of 0** (before: +88/+110). (offline) mocap platform vx 10 ms after release ≤ 20 mm/s (before: +84..+103). |
| A3 | `ros2 param set /trajectory_node pre_release_hold_s 0.0` | `Set parameter successful`; the node logs `pre_release_hold_s set to 0.000 s`. |
| A4 | 5 × the same hop | (live) The control: should reproduce **+85..+110 mm**. If it does not, the overshoot has moved; note it and stop. |
| A5 | `ros2 param set /trajectory_node pre_release_hold_s 0.1` | Back to the default for the rest of the sitting. |

How to read it:

| A2 (hold) | A4 (no hold) | Verdict |
|---|---|---|
| within ±25 mm | +85..+110 | Pre-release motion was the cause; the hold is the fix. |
| still ≥ +60, with the platform still at release (offline) | +85..+110 | Pre-release motion is NOT the cause. Owner's hypotheses (orientation, post-throw) come back; next is a smaller separation. |
| between | +85..+110 | Partial: read the offline platform velocity before deciding. |

A HOLD_WINDOW refusal is a plain-language message naming the timing it could not fit. Record it
and move on.

## B. Self-toss, open-loop: the true landing spread (≈ 12 min)

Hypothesis (ticket 01): first-of-attempt throws (ball fully settled) spread about 16 × 23 mm (1σ)
because of something between platform and ball at release, not the platform's position. With
re-aim off, this is the spread the catch has to live with.

| # | Step | Expect / deciding number |
|---|---|---|
| B1 | 12 × `ros2 action send_goal /jugglebot/juggle jugglebot_interfaces/action/Juggle "{pattern: self_toss, num_cycles: 1}"`. Let the ball settle between attempts; it does so by itself. | (live) OUTCOME `y=(x, y)` and `caught`. (offline) landing 1σ per axis. **≤ 10 mm: the platform-still hold also fixed the self-toss (unexpected; would reopen ticket 01). About 15–23 mm: the release-side residual is confirmed**, and the fix is at the cup/hand. |

| B2 | `ros2 param set /skill_node apex_m 0.6` ; 8 × the same single ; then `ros2 param set /skill_node apex_m 0.9` | (offline) lateral launch-velocity σ at 0.6 m against B1's 0.9 m. **Same σ in mm/s (the angle shrinks with speed): a fixed-size kick at separation** (cup lip, ball–cup contact; the 2026-09-22 data leans this way, n=4–5). **Same σ in degrees: the launch axis itself wobbles** (carriage/cup compliance). |

B2 needs no markers. QTM already tracks the ball in the cup at 152 Hz, and over the last 150 ms of
the stroke it drifts only 2–6 mm relative to the platform (offline, 2026-09-29), so the kick is
concentrated at separation. That is where the offline analysis will look.

## C. Carried chains: dwell A/B, does settle time matter? (≈ 10 min)

Hypothesis (owner, Q4): a ball caught slightly wonky doesn't settle in a 0.3 s dwell, which adds
spread on carried throws (ticket 01: carried 23.5 × 26.1 mm against rest 15.8 × 23.0 mm).

| # | Step | Expect / deciding number |
|---|---|---|
| C1 | `ros2 param set /skill_node dwell_s 0.30` ; 3 × `{pattern: self_toss, num_cycles: 5}` | (offline) carried-throw landing 1σ at 0.3 s. Chains may end early on a drop; that is data. |
| C2 | `ros2 param set /skill_node dwell_s 0.60` ; 3 × the same | (offline) **carried 1σ at 0.6 s ≤ rest 1σ + 3 mm: settling is the lever.** No change: it isn't. |
| C3 | `ros2 param set /skill_node dwell_s 0.30` | Restore. |

If a dwell of 0.6 s is refused (a HOLD_WINDOW or box message), record the message verbatim and skip C2.

## D. Reload: does the hand now rise to receive? (≈ 8 min)

Hypothesis (ticket 03, fixed): the catch now dispatches only after the pre-tilt has finished, so its
plan starts on the catch line.

| # | Step | Expect / deciding number |
|---|---|---|
| D1 | `session_skills_r4.md` § 6 reload from the GUI, `self_toss` with `num_cycles: 1`, 3 times | (live) **No `CATCH_AXIS` refusal; the hand visibly rises and meets the ball softly.** Any refusal message is now plain language: copy it verbatim into § R. |

## R. Results (fill in)

| Block | Result |
|---|---|
| A2 hold on: landing x (5) | |
| A4 hold off: landing x (5) | |
| B open-loop singles: caught / 12; landings | |
| C1 dwell 0.3: chains | |
| C2 dwell 0.6: chains | |
| D reload: refusals / hand rose? | |
| Bag directory | |
