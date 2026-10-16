---
title: "The tracker fit converged late on 2026-10-16 not because of the matcher fix but because the live node skips ~2/3 of QTM's frames and the Kalman predict was capped at 20 ms: one 30-60 ms processed-frame gap left a CONFIRMED filter ~65 mm behind the ball, outside its 50 mm gate, and it coasted unfed for the rest of the flight; the cap now follows the real (QTM-stamped) gap up to 0.10 s"
type: investigation
date: 2026-10-16
status: resolved
phase: "tracker fit latency (follow-up to handoff-solve-chaining P2)"
related_plan: handoff-solve-chaining.md
related:
  - 2026-10-16-handoff-lead-flown-zero-splice-too-late-tracker-fit-late.md
  - 2026-10-14-two-ball-columns-sessions-why-attempts-end.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/tracking/matcher.py
  - ros_ws/src/jugglebot/jugglebot/tracking/tests/test_matcher_frame_dt.py
  - tests/ros/test_ball_tracker_frame_gap.py
---

# Tracker fit late since 2026-10-16: frame starvation in the live node + a 20 ms predict cap, not the matcher fix

**Data:** S16 = launch log `~/.ros/log/2026-10-16-10-50-42-798495-jetson-1033523/launch.log`, bag
`~/Desktop/rosbags/2026-10-16_10-50-42` (first sitting with 5974d8c3); S8 = launch log
`~/.ros/log/2026-10-13-23-12-50-197471-jetson-3905591/launch.log`, bag
`~/Desktop/rosbags/2026-10-13_23-12-50_2-Ball-32-Catches` (before it). Probes are committed in `tools/probes/tracker_fit_latency/`
(they wrap `tools/probes/tracker_bag_replay.py`'s `make_tracker`/`replay`); every headline number below is tied
to its command in Verification. Bag shorthand: S16 = `~/Desktop/rosbags/2026-10-16_10-50-42`, S8 =
`~/Desktop/rosbags/2026-10-13_23-12-50_2-Ball-32-Catches`.

## Symptoms

From the bags' `/balls` (`handoff_replay.load_fits`), landing minus the first `landing_from_fit` instant, all
announcements: S8 p10/p50/p90 566/633/671 ms with 25/145 never; S16 142/540/614 ms with 77/212 never. In S16's
log: 0 of 175 handoff first aims from the tracker, 84 `NO-CONVERGED-FIT` resend skips, 47
`TRACKER-IDENTITY-REFUSED throw` lines all at +1.15..+1.20 s (S8: 1). Confirmation itself was fast (183
`tracker confirm` lines, +0.001..+0.028 s; 1 demote). The obvious suspect was 5974d8c3, the only tracking
commit between the two sittings.

## Diagnosis

1. **The tracker logic reproduces neither the symptom nor a difference between matchers.** Full-frame replay of
   S16 (all 123,524 `/mocap_data` frames through the real `BallTracker`), self-tosses: landing - F p10/p50/p90
   **768/772/780 ms, 36/191 never**, with the current matcher AND with 5974d8c3^'s matcher (identical); confirm
   p50 +0.002 s. Per announcement, live F trails replay F by p10/p50/p90 113/184/536 ms (132 matched), and 44
   flights the live node never fitted converge in replay. Offline `process_frame` cost is 0.153 ms/frame (new)
   vs 0.177 ms (old) over 30,000 frames — the matcher's compute is not it either.
2. **The live node processes a fraction of the frames.** `/mocap_data` arrives at ~290 Hz in both bags
   (identical 1.17 kB messages), but during the first 0.6 s of each flight `/balls` was published for p50
   **0.45 (S8) / 0.33 (S16)** of the mocap frames; in-flight `/balls` gaps p90/p99 12.6/25.5 ms (S8) vs
   **18.1/33.4 ms (S16)**, 18 % vs 37 % of in-flight time spent in gaps > 15 ms. The live `CONFIRMED` instant
   trails `throw_time` by p50 47 ms (S8) / 83 ms (S16) against replay's +2 ms.
3. **A decimated replay — only the newest mocap frame at each live `/balls` publish instant — reproduces the
   live numbers on both bags:** S16 157/547/631 ms, 70/212 never (live 142/540/614, 77); S8 509/638/678 ms,
   17/145 never (live 566/633/671, 25). So the symptom is a property of the frames the node got, not of the
   matcher revision.
4. **Mechanism of the never-converged flights (frame-by-frame trace of S16 balls 4, 15, 18 in the decimated
   replay):** right after confirmation the nearest marker to the CONFIRMED filter's position jumps to 63-118 mm
   and stays there; `frames_without_measurement` climbs for the rest of the flight; the fit holds 0-2 samples.
   Ball 18 sits at a near-constant 63-70 mm for 0.7 s (filter parallel to the ball), ball 4 is re-acquired only
   at the apex where the two paths cross (fit converges at +0.51 s). The cause is `_frame_dt`'s upper clamp,
   4 x nominal dt = **20 ms**: across a 36 ms processed-frame gap the filter steps 20 ms and is left
   (36 - 20) ms x ~4 m/s ~ 65 mm behind, outside the 50 mm association gate (`gate_radius_mm` default; the node
   does not override it). Synthetic driver (4 m/s toss, 3.4 ms frames, one dropped run from t = 30 ms): first
   fit at 0.112 s with a 20-30 ms gap, **0.238 s at 36 ms, 0.340 s at 50 ms, 0.422 s at 80 ms**.
5. The +1.18 s identity refusals are downstream: with no fit before the same ball's next release,
   `flight_in_progress` hands the executor the NEXT flight's fit, which `_release_landing` correctly refuses.

**Hypotheses (pre-registered in the brief):** H1 (fit starts late / popped and recreated after confirmation) —
killed: full-frame replay converges at +0.14 s, no ball ever had a second `FlightFit`, 1 demote in the log.
H2 (`throw_time` / freeze-floor admission) — killed: identical admission converges early at full rate. H3
(identity split, two tracks per flight) — killed as a cause: one fit per ball id; the +1.18 s refusals are
item 5. H4 (residual gate rejecting a non-ball marker) — killed: the starved fits held 0-2 samples, not
rejected ones; associated samples sit 0.2-2 mm from the filter. The actual mechanism (frame starvation + the
predict cap) was not on the list.

## Discussion

**Why 5974d8c3 looked guilty and isn't.** It was the only tracking change between the sittings, and its motion
test does need two consecutive steps, which a starved stream delays. But the matcher A/B on identical frames is
identical, and S8 — before the fix — already ran at 0.45 of the frames, with 14 confirmed-but-never-fitted
self-tosses in its decimated replay. S16 crossed a threshold that S8 sat near: more processed-frame gaps beyond
~33 ms. Why the node got less CPU on 2026-10-16 is **not established** (candidates: the handoff rung's larger
segment solves in the worker process, other load on the box); the node's own tracker work is ~0.15 ms of a
3.4 ms frame budget, so the cost is in rclpy (deserialising ~26 labelled markers, building and publishing
`BallStateArray`) and in scheduling.

**Why fix the cap and not the node first.** Two independent defects compound: (a) the node drops frames, so the
fit's 12 samples take ~3x as long to collect — this alone moves p50 from 772 to ~578 ms; (b) the cap turns
a dropped run into permanent track loss, which is what produced the never-converged third. (b) is a
one-constant, well-understood fix whose correctness does not depend on the load; (a) needs a design choice
(see Outcome). The cap existed for "a replay restart never becomes one giant leap" (2026-09-20). Since the same
date frames carry QTM's capture time, so a measured 36 ms gap IS 36 ms of flight, and the gravity-exact
predict across it is correct; a giant leap (restart, multi-second stall) is still capped, at 0.10 s.

**Rejected alternatives.** Widening `gate_radius_mm` (50 mm) would also re-acquire, but trades identity
safety near the cup and the other ball for a symptom of the predict error. Scaling the gate with the gap is
the same trade with more machinery. Lowering `flight_fit_min_samples` would speed convergence under starvation
but loosens the accuracy contract the fit was validated on (2026-09-17).

**Not changed, pre-existing.** A gap above 0.10 s is clamped, not reset (the track steps 0.10 s and the
gate decides re-acquisition), and an out-of-order stamp overwrites `_last_frame_time`; both behaviours predate
this change and are unchanged. The Kalman process noise Q (`kalman.py`) is added once per predict regardless of
dt, so after a long gap the covariance is slightly under-inflated; association uses a fixed 50 mm gate, not a
Mahalanobis one, so the effect is confined to the Kalman gain; scaling Q with dt is a separate change, not made
here.

**Tradeoff accepted.** The announced-ball motion test's "consecutive" window was `4 x dt + 0.02` = 40 ms; it now
uses the same 0.10 s. A step's speed is measured over the real interval, so a static marker still fails the
0.4-1.8x speed ratio — 5974d8c3's purpose is kept (`test_ball_tracker_announced_motion.py`,
`test_ball_tracker_gate.py` green). Control-path implication: the published KF position/velocity and the
pre-fit fallback landing become more correct, the fit's landing math is unchanged, and more catches will now
re-aim from the tracker inside the executor's existing clamps (10 mm outward / 35 mm inward / 20 mm across)
and feasibility gate — a behaviour change the next sitting should watch, not a new authority.

## Fix

`tracking/matcher.py`: new `BallTracker(max_frame_gap_s=0.10)` (floored at 4 x dt); `_frame_dt` clamps to
`[0.4 x dt, max_frame_gap_s]` instead of `[0.4, 4] x dt`; the announced-motion continuity check uses the same
bound. No config or node change; the ROS package needs `colcon build` + a launch restart to take effect.
Tests: new `tests/ros/test_ball_tracker_frame_gap.py` (the synthetic driver above, gaps 36/50/80 ms: zero missed
associations after the gap, first fit no later than the no-gap time + the gap; plus the 0.10 s cap) — 4 failed
against HEAD's matcher, 4 pass with the fix; `tracking/tests/test_matcher_frame_dt.py`'s cap assertion updated
to the new bound with a 36 ms case added.

## Outcome

Decimated (live-faithful) replay, self-tosses, landing - F p10/p50/p90 and never-fitted:

| | before | after |
|---|---|---|
| S16 | 157/547/631 ms, 69/191 never (39 confirmed-but-unfitted) | **496/578/635 ms, 34/191 never (4 confirmed-but-unfitted)** |
| S8 | 509/638/678 ms, 17/131 never (14) | **585/640/679 ms, 4/131 never (1)** |
| S16 full-frame | 768/772/780 ms, 36/191 never | 768/772/780 ms, 36/191 never (unchanged) |

Denominators: Diagnosis item 3's "70/212" and "17/145" count ALL announcements; this table's "/191" and "/131"
count self-tosses only. Separately, ~30 S16 self-tosses never confirm at all, even in a full-rate replay with
either matcher (a different set from the "36/191 never fitted" of Diagnosis item 1, which counts flights that
never got a fit); that class is open and undiagnosed, next step = replay one at full rate and read the
`tracker confirm` / `NO-CONVERGED-FIT` lines. **Not yet flown; takes effect after colcon build + launch restart.**
**Still open:** the node's frame starvation (the remaining ~190 ms of p50 against full
rate). A peer session's BB-calibration sitting analysis (2026-10-16, not measured here) found mocap_node
logging "frame queue full" drops of 31-64 % of QTM frames after every calibration sweep and at idle, present in
the 2026-10-13 and 10-14 logs too, so the drop may sit upstream in mocap_node's frame queue rather than in
ball_tracker_node's own CPU; the next step is to read the mocap_node "frame queue full" lines against the
/balls publish share before profiling the tracker. Options, for the owner: (1) find what took the CPU on 2026-10-16 (profile the live node under a sitting's
load; check the solver worker's niceness/affinity); (2) cut the node's per-frame rclpy cost (publish `/balls`
at a fixed lower rate while processing every frame, or move the matcher into the mocap interface's process);
(3) lower QTM's frame rate toward the tracker's 200 Hz nominal. Each changes a different thing; none is in this
entry.

## Verification

All commands run 2026-10-16 from the repo root with the project venv; probe outputs under
`temp/probes/tracker_fit_latency/`. The probes were promoted from scratch scripts after the runs (paths were
only made repo-relative; logic unchanged), so the commands below are the promoted-path equivalents of what was
run, and the numbers are those recorded in this entry and the session hand-off.

- Symptoms S8 566/633/671 ms & 25/145 never, S16 142/540/614 ms & 77/212 never: from the bags' `/balls` via
  `handoff_replay.load_fits` (live numbers; no probe command recorded for this read beyond that loader).
- Diagnosis 1 (full-frame replay), `python tools/probes/tracker_fit_latency/probe_fit.py ~/Desktop/rosbags/2026-10-16_10-50-42 new|old temp/probes/tracker_fit_latency/fit_S16_<new|old>.json`:
  212 announcements, 123,524 frames; self-toss landing - F 768/772/780 ms, 36/191 never for both matchers.
- Diagnosis 1 per-frame cost (0.153 ms new vs 0.177 ms old, 30,000 frames): from a scratch `probe_cost.py`,
  NOT promoted (results in the session scratchpad `cost_new.txt` / `cost_old.txt`); no repo-recorded command.
- Diagnosis 2 (rates), `python tools/probes/tracker_fit_latency/bagrate.py <bag> temp/probes/tracker_fit_latency/rate_<tag>.json`:
  mocap 286.8 Hz (S8) / 292.0 Hz (S16); in-flight `/balls`-per-mocap ratios and gap percentiles as in Diagnosis item 2.
- Diagnosis 3 and the Outcome table (decimated replay), `python tools/probes/tracker_fit_latency/probe_decim.py <bag> temp/probes/tracker_fit_latency/rate_<tag>.json 0.0 temp/probes/tracker_fit_latency/dec_<tag>_0.json`
  (before = pre-fix matcher, after = working tree): the numbers in those two sections.
- Diagnosis 4 ball 4/15/18 traces, `python tools/probes/tracker_fit_latency/probe_trace.py ~/Desktop/rosbags/2026-10-16_10-50-42 temp/probes/tracker_fit_latency/rate_S16.json 0.0 temp/probes/tracker_fit_latency/trace_S16.json`
  (pre-fix matcher; `TRACE` set edited in the script for the balls of interest).
- Diagnosis 4 synthetic 0.112/0.238/0.340/0.422 s, `python tools/probes/tracker_fit_latency/probe_gap.py 0.03` against the pre-fix matcher. The
  G0 = 0.03 argument is inferred from "one dropped run from t = 30 ms"; the exact value passed was not recorded.
- `pytest tests/ros/test_ball_tracker_frame_gap.py -q -p no:cacheprovider` against HEAD's `matcher.py`:
  **4 failed**; with the fix:
  `pytest tests/ros/test_ball_tracker_*.py ros_ws/src/jugglebot/jugglebot/tracking/tests -q -p no:cacheprovider`
  **94 passed in 33.60 s**. `./run_tests.sh` not run by the entry's author (the coordinator gates the commit).
