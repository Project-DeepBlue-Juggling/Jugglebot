---
title: "Handoff lead flown (sitting 2026-10-16 10:50): zero SPLICE_TOO_LATE ends in 17 attempts / 175 handoff catches, three first installs over Fix 1's budget; but every first aim rode the prior because the tracker fit now converges ~90 ms later than in the recorded sessions and never converges on 36 % of flights, so only 30 catches got a correction; the fresh THROW's 150 ms origin budget is the same failure class and was hit once"
type: investigation
date: 2026-10-16
status: resolved
phase: "handoff-solve-chaining — P2 (flown; close-out)"
related_plan: handoff-solve-chaining.md
related:
  - 2026-10-16-handoff-lead-8-knots-and-snap-aware-resend-fences.md
  - 2026-10-15-handoff-replay-baseline.md
  - 2026-10-14-two-ball-columns-sessions-why-attempts-end.md
files_changed:
  - plans/active/handoff-solve-chaining.md
  - logbook/2026-10-16-handoff-lead-8-knots-and-snap-aware-resend-fences.md
  - logbook/2026-10-15-handoff-replay-baseline.md
---

# Handoff lead flown: zero SPLICE_TOO_LATE ends; the correction now rides a tracker fit that arrives late

**Data:** `~/.ros/log/2026-10-16-10-50-42-798495-jetson-1033523/launch.log`, bag `~/Desktop/rosbags/2026-10-16_10-50-42`.
HEAD at the sitting 466878eb (`HANDOFF_LEAD_EXTRA_KNOTS = 8`, install receipt `= 8`). 17 columns attempts
(Ball-Butler-fed hop entry), 191 catches dispatched (175 handoff + 16 attempt-opening); the log names an outcome for
151 of them, 136 caught / 15 missed (the rest are catches the attempt ended before or that the outcome line does not
name), ends: 15 `MISSED_CATCH`, 1 `ORIGIN_TOO_LATE` (the sitting's
first reload THROW), 1 `ABORTED_NO_RELEASE`. The operator did not run runsheet § 1b rows 26-29 live; all reads
below are retrospective (parser `scratchpad/sitting_parse.py` + `sitting_stats.py`, session-local; the replay probe
re-run on the pair: `python tools/probes/handoff_replay.py --log S16=<launch.log> --bag S16=<bag>`, 2026-10-16).

## Runsheet § 1b reads (rows 23-29)

| row | check | result |
|---|---|---|
| 23 | install carries `HANDOFF_LEAD_EXTRA_KNOTS = 8` | PASS (grep on `ros_ws/install/.../schedule.py`) |
| 26 | `CATCH-AIM` source mix at first dispatch, handoff catches | 175/175 `schedule:learned`, 0 `tracker` (recorded sessions: 31-55 % tracker; the plan predicted ~5 % at this lead) |
| 27 | re-send inside the band splices at the first install's knot | PASS: 30/30 installed snapped re-aims have `splice_k == k_rel`; 0 raw re-aims installed |
| 28 | `PINNED-BUDGET` at most once per catch, never followed by a budget END | PASS: 38 catches declined once, 0 twice; 0 budget ENDs |
| 29a | handoff `SPLICE_TOO_LATE` ENDs | **0** (2026-10-14 baseline 2.7-5.1 % per dispatch). The 4 `SPLICE_TOO_LATE` strings in the log are `RESEND-REFUSED` lines (3 transit, 1 worker) on re-aims whose first install held; all four balls were caught |
| 29b | every refusal carries `_refusal_timing` | PASS: 76/76 post-solve refusals (39 LIMIT_JERK, 13 LIMIT_ACC, 12 INFEASIBLE, 3 CATCH_AXIS, 3 transit, 2 LIMIT_VEL, 1 each WINDOW_TOO_SHORT / SPLICE_TOO_LATE / SINGULAR / ORIGIN_TOO_LATE); the 77th `install_segment refused` line is a pre-solve `commanded state moved during planning` retry, which carries no timing by design |
| 29c | cliff bias vs nominal | **resolved: +0 to +25 ms, median +21 ms** over 67 refusals with a `cliff +X ms` field (snapped re-sends vs `D + 0.350`, raw vs `R + 0.150`); the replay's derived median `C - D` is 377 ms. The bias is the ceil-to-knot quantisation of the splice knot and is never negative, so every nominal budget in the plan is a lower bound. The P0 derivation (the same probe) reads +27 ms on today's data against the node's +21 median, so the lead-13 "38 ms" was that derivation within one knot of the node's arithmetic, not a clock offset |

**First-install budget, handoff catches (n = 175):** node plan p50 156 / p90 215 / **max 280 ms** against the 350 ms
nominal budget (worst margin 70 ms, 90 ms with the measured quantisation). Three first installs exceeded Fix 1's
250 ms and 26 exceeded the original 200 ms, i.e. three attempts (and the catches behind them) that the lead bought
in one sitting. Solves ran about 20 % slower than in the recorded sessions (recorded accepted p50 128 / max 224 ms)
although every one was prior-aimed; the margin is thinner than the replay's 113 ms projection but ample.

## Discussion

**The plan's premise about the tracker did not hold on the day.** The design (plan § 0.2 Finding B, § 1.3) assumed
the flight fit converges 0.2-0.3 s after release, so that at 17 knots the pinned re-aim window
(`[first-install end, cliff - 0.150]` = roughly `[D + 0.16, D + 0.20]`) would carry the correction for 76 % of
late-fit catches. From today's bag (same method as P0, `fit_S16.json`): the jugglebot-throw fit converged
**540 ms before landing at the median vs 630-645 ms in S1/S2/S8**, with p10 142 ms (vs 566) and **77 of 212
flights never converging (36 %, vs 13-23 %)**. In log terms the first converged-fit event came at D + 226 ms
median (p10 164), i.e. release + 0.37 s, after the pinned window had closed for most catches. The full partition of
the 175: 30 re-aims installed (17 %, all snapped), 57 catches whose re-aim was refused (53 raw-path re-sends fired
after the snap band and refused on physics — `LIMIT_JERK` 35, `LIMIT_ACC` 13, `INFEASIBLE` 12, other 3 — plus 4
snapped re-sends refused, 3 `SPLICE_TOO_LATE` and 1 `LIMIT_VEL`), 2 `PINNED-BUDGET`-declined with nothing after,
2 within tolerance, 84 that never saw a converged fit. The 38 `PINNED-BUDGET` declines sit inside those classes:
36 went on to a refused raw re-send, 2 are the declined-only pair. No raw re-send installed, because a raw re-send at D + 0.30 s splices 100 ms after the release knot, mid-swing, with ~240 ms to
the cup contact; a refused re-send is terminal (2026-09-20 rule), so those catches stayed on the prior.

**This is the first sitting with the matcher fix (5974d8c3, 2026-10-15).** Confirmation itself is fast (183
`tracker confirm` lines at 2-3 steps, +0.001 to +0.028 s from throw_time), so the delay sits downstream of
confirmation, in when the per-flight `FlightFit` starts receiving samples or passes its 12-sample / 50 ms /
12 mm gate. Not diagnosed here; the `TRACKER-IDENTITY-REFUSED` count also rose from 12-15 per session to 51 and
the learner got 87 rows for 136 catches (64 %, vs 85 % at S8). Both point at the same change and belong to one
tracking investigation with this bag.

**Did it cost catches?** The miss rate did not move: 15/151 (10 %) today vs 4/49, 10/89, 10/124 (8-11 %) in the
three recorded sessions, which had the tracker at first dispatch 31-55 % of the time. So the first-dispatch aim
source is not what decides a miss at this spread. Within today's sitting the split is suggestive, not
conclusive: the 30 catches with an installed re-aim had 0 misses (expected ~2.7 at the sitting rate, p ~ 0.06),
the 55 with a refused re-aim had 10 (18 %), the 84 with no fit had 6. Every missed handoff catch had a
tracker-vs-aim delta of 18-40 mm that never got in; caught landings run p50 22-26 / p90 43-50 / max 63 mm, so
the cup's authority is about 50-60 mm and the misses are the balls beyond it. Shrinking that spread (the
"messy throws") is worth more than any timing change; getting the correction in is second.

**The fresh THROW origin is the same failure class and is unprotected.** The reload THROW is a fresh origin with
`LEAD_S` (0.225 s) and therefore a 150 ms solve budget (`reserve_fresh_lead`). Today's 16 THROW plan times ran
44-165 ms (recorded sessions: 41-88 ms) and the sitting's first attempt ended `ORIGIN_TOO_LATE` on a 188 ms
solve (0/0 caught; not the first solve of the session, so not a cold start). A THROW-specific fresh-origin lead
(the schedule already assigns per-kind leads in `_assign_leads`; from rest there is no seam to move) would close
it the same way the handoff lead did. Not done here: it is a schedule change with its own ripple on the
hop-entry timing and needs its own replay.

## Decision

- § 2.C's trigger did NOT fire (worst first-install margin 70 ms >= 25 ms; zero handoff `SPLICE_TOO_LATE` ENDs).
  Worker chaining stays dropped. The lead stays at 8 extra knots; a further knot or two is available (min dispatch
  gap 0.400 - 0.025 x EXTRA s stays positive to 15) but nothing today asks for it.
- The rung's own target is met; the plan can be archived. Three follow-ups are recorded in the plan § 4, none of
  them this rung's: (1) tracker fit latency since 5974d8c3 (bag in hand), (2) a fresh-origin lead for the reload
  THROW, (3) the raw-path re-aim after the snap band never installs and could be fenced on physics instead of
  being spent on a terminal refusal.

## Verification

- 2026-10-16, `grep -E 'END .*SPLICE_TOO_LATE' <launch.log>`: no lines; `grep -oE 'ENDED \((\w+)\)' | sort | uniq -c`:
  15 MISSED_CATCH, 1 ORIGIN_TOO_LATE, 1 ABORTED_NO_RELEASE.
- 2026-10-16, `python tools/probes/handoff_replay.py --log S16=... --bag S16=...`: 191 first-dispatch CATCHes replayed
  (the 175 handoff catches plus the 16 attempt-opening catches, which the probe's release-minus-dispatch criterion
  admits), 0 negative margins at every lead, median derived `C - D` 377 ms; fit available at the old dispatch instant 1 %.
- Docs-only commit; the tests that read these paths are `tests/sim/test_logbook_front_matter.py`,
  `tests/sim/test_logbook_search.py` (front matter shape; INDEX.md is skipped by the loader) and
  `tests/sim/test_plans_index.py` (plan <-> INDEX pinning). Result in the commit message.
