---
title: "Stop after a drop: a spliced REST before the next throw replaces the legacy hold, and only a converged fit counts as tracker release evidence"
type: investigation
date: 2026-09-29
status: in-progress
phase: "two-ball-skill-stack — R4 carry (pre-R5)"
related_plan: two-ball-skill-stack.md
sessions:
  - temp/logs/skills_r4_20260929_1911.log
  - ~/Desktop/rosbags/2026-09-29_19-11-49 (mcap — not in the repo)
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - tests/motion/test_skills_executor.py
  - tests/ros/test_skill_node.py
  - plans/active/two-ball-skill-stack.md
subsystem:
  - motion
  - ros
---

# Stop after a drop

## Symptom

At the R4 gate sitting (2026-09-29 19:11, `logbook/2026-09-29-skill-stack-r4-gate-met.md` § 7)
one drop was followed by up to three empty throws, and in the last attempt four:
- the first empty throw was taken as released;
- the attempt ended only after the second;
- the stop (`trajectory/hold`) was refused `LIMIT_JERK` 157–158 k > 150 k on all three
  attempt-ending drops, so the plan already installed ran its next throw too.

The owner's rule (2026-09-29): **always make the first throw after a catch**, because slow-settling
balls that did not register as held were often thrown well. Then stop before the second empty
throw, and rest the hand at home. The owner confirmed the design below before it was built.

## Diagnosis

1. **The first empty throw passed the release check.** `_advance_release_evidence` accepted any
   tracker landing estimate sampled after the release. The dropped ball bounced beside the cup,
   and its Kalman estimate confirmed the throw.
2. **The stop was a pre-skill-stack move.** `planner.build_hold` is a platform-only quintic back
   to the pose held when the stop was asked for, at rest, in `min_move_duration_s` = 0.2 s. It
   carries no hand track. From mid-swing that U-turn breaks the jerk limit.

## Discussion

**Release evidence: a converged fit near the schedule.** Measured on the sitting's bag
(`/throw_announcements` from Jugglebot against `/balls.landing_from_fit`; scratchpad
`probe_fit_latency.py`):
- 117 releases; 106 had a converged fit before landing;
- release → first fit: p50 0.176 s, p95 0.227 s, max 0.406 s, so all inside `RELEASE_GRACE_S`
  (0.5 s, unchanged);
- all 11 releases with no fit were empty throws after drops, among them the run of four at
  19:23:47–50.

So "converged fit" separates real throws from empty ones on this data.

The added time check, `RELEASE_FIT_TOL_S` 0.2 s around the scheduled touch-down, keeps a bounce
that flies a ballistic arc of its own from converging into evidence. It is sized on its own
quantity, not on the latency above. Over the 106 real releases' 18,219 fit samples, fitted
touch-down minus scheduled is +0.027 to +0.066 s (p50 +0.044 s; the plant throws a little high).
So 0.2 s is three times the worst real case, and the next flight is a beat away.

The hand-sensor path (SEATED then EMPTY after the release) is unchanged and stays an independent
confirmation.

The failure this could introduce is a real throw whose fit is slower than 0.5 s and whose sensor
also misses the release. That stops a good chain. It is benign: the ball lands on a resting cup.
It has not been observed (max 0.406 s).

**The stop: a REST spliced into the running plan, two candidates, then the old hold.** A REST
asked for now would normally be pulled forward to the pending release by `_snap_to_release`, the
rule that makes a catch's splice start at the ball's release, and the throw would still run. So
the first candidate is at rest `STOP_BEFORE_MARGIN_S` (two knots) BEFORE that release. Its own
event then precedes the release, there is nothing to snap to, and the splice cuts the throw out.

The alternative was a new "cut before the release" flag on `InstallSegment.srv`. It was rejected:
an interface change and a `colcon build` of `jugglebot_interfaces`, for a case the existing
timing rule already expresses.

The second candidate is at rest `STOP_AFTER_S` (0.6 s) after the release. It snaps to the release,
is seeded post-release, and lets that throw run, but nothing after it.

The legacy hold stays last. The HAND_LANE_REFUSED stop is unchanged: it is deliberately
hand-less, and never tries a REST.

Rest site: the site of the skill carrying the pending release, i.e. where the platform is already
heading, level, hand at home (the site's rest point every schedule REST uses).

Probe on the real planner and executor at the R4 limits (300 / 5000 / 150 000 / 3500),
schedule-aimed catches, 7 releases per cell (scratchpad `probe_stop_rest.py`):

| Asked after the previous release | 0.45 s | 0.50 s | 0.55 s | 0.60 s | 0.65 s |
|---|---|---|---|---|---|
| hop 250 mm, 'before' REST | refused (5 `CUP_CONTACT_ACC`, 2 `LIMIT_JERK`) | 7/7 | 7/7 | 7/7 | refused (`LIMIT_JERK`, window too short) |
| self_toss, 'before' REST | 2/7 | 7/7 | 7/7 | 7/7 | 7/7 |
| either pattern, 'after' REST (0.6 or 1.0 s) | 7/7 | 7/7 | 7/7 | 7/7 | 7/7 |

Plans took 12–18 ms. Asked earlier (0.30–0.40 s) the 'before' REST refuses on the hop because the
platform is still mid-traverse between the sites. In the first probe pass, which tried both
sites at 0.50 s, each site was accepted only as often as it was the platform's destination (3/7
and 4/7), and the others refused `LIMIT_VEL`. That is consistent with the destination rule;
it was not checked snapshot by snapshot.

The empty-throw END fires at t_release + 0.5 s, inside the working band. So after a drop:
- the first empty throw runs (the owner's rule);
- the END fires 0.5 s later;
- the 'before' REST stops the machine ahead of the second.

The margin at the far edge is thin: 0.60 s works on the hop and 0.65 s does not. A tick or solve
running more than ~100 ms late falls through to the 'after' REST, which is one empty throw worse
but still bounded.

**Hand at home, not at the top** (owner, 2026-09-29). Every attempt's opening REST already homes
the hand. The top only pays off once the reload also starts from the top, which belongs with the
reload hand-rise item.

## Fix

- `executor.py`:
  - `RELEASE_FIT_TOL_S`, `STOP_BEFORE_MARGIN_S` and `STOP_AFTER_S`;
  - `_advance_release_evidence`'s tracker branch requires `Landing.from_fit` and a touch-down
    within `RELEASE_FIT_TOL_S` of the scheduled one;
  - `SkillExecutor.stop_terminals(t_now)` returns the ordered `('before' | 'after',
    RestTerminal)` candidates for the earliest pending release among the ACCEPTED installs, or
    `[]` when none is pending. The 'before' candidate is omitted when `LEAD_S + MIN_WINDOW_S`
    cannot fit ahead of it.
- `skill_node.py`: `_maybe_hold_pending_event` calls `_stop_with_rest` first for a pending release
  (not for the forced HAND_LANE_REFUSED hold). That installs each candidate through the ordinary
  `_installer` until one is accepted, then falls back to `trajectory/hold`. There is one INFO
  line per stop and a WARN per refused candidate.
- `skill_node._on_tick` runs the stop AFTER releasing `_tick_lock`, the same as `_stop_attempt` and
  `_end_attempt` always did, with the executor passed in. The stop can now be three blocking round
  trips, which would outlast the `_STOP_LOCK_WAIT_S` an operator Stop waits for (phase-end
  audit).

## Verification

- 2026-09-29, the new tests before the change:
  - `tests/motion/test_skills_executor.py -k "unfitted_tracker_landing or far_from_the_schedule or
    stop_rests_come or hop_stop_rest"`: **4 failed**;
  - `tests/ros/test_skill_node.py -k "stopped_with_a_rest_not or refused_stop_rest_tries or
    refused_hand_lane_holds_without"`: **2 failed, 1 passed** (the hand-lane regression guard).
- After the change, 2026-09-29:
  - `python -m pytest tests/motion/test_skills_executor.py -q`: **137 passed in 5.01 s**. This
    includes `test_a_hop_stop_rest_installs_before_the_next_throw_on_the_real_planner`: a 250 mm
    hop chain on the real planner, stopped `RELEASE_GRACE_S` after a release, with the 'before'
    REST accepted and no release left after its splice.
  - `python -m pytest tests/ros/test_skill_node.py -q`: **143 passed in 3.50 s**.
- Gate, run 2026-09-29 22:50–22:56: `./run_tests.sh --full` → **PASS, 5616 passed, 9 skipped, 1
  xfailed in 354.26 s; serial 6 passed in 20.17 s** (`temp/logs/gate_full_stop_rest_20260929.log`).
  Against the R4 close-out's 5581: +7 are this entry's tests, and +28 are
  `tests/ros/test_launch_console.py` from the console commit `ddc3d0a9`.
- Gate after the audit fixes, run 2026-09-29 23:12–23:16: `./run_tests.sh` → **PASS, 5580 passed, 9
  skipped in 213.66 s; serial 3 passed in 8.76 s** (`temp/logs/gate_stop_rest2_20260929.log`).
- Phase-end audit, two items adjudicated and taken:
  - the lock finding: `test_the_stop_after_an_ended_tick_runs_outside_the_tick_lock`, which
    failed before, with the REST install seen under the lock;
  - the margin note: `test_the_before_stop_splice_clears_the_pre_release_hold_band` pins
    `MIN_WINDOW_KNOTS + margin >= pre-hold knots + 2`, exactly 6 = 6 at the defaults.
  The tolerance citation was corrected, as above. One finding did not apply: the held-axis
  `rest_site_mm` never reaches `stop_terminals`, because the reload's held-axis CATCH carries no
  throw (see its docstring). After these, `tests/ros/test_skill_node.py` 144 passed, and the
  stop and evidence tests in `tests/motion/test_skills_executor.py` 9 passed.
- Hardware: not yet flown. At the next sitting, a drop should end with at most ONE empty throw
  and an `END ... stopped with a REST ... BEFORE the next release` line.

## Withdrawn claims

## Open Questions

- A live `pre_release_hold_s` above 0.100 s can refuse the 'before' REST near its offer edge
  (the pinned margin is exact at the default); the stop then takes the 'after' REST.
- The 'before' band is narrow on the hop (0.50–0.60 s after the release). If hardware tick or
  solve lateness pushes the END past it, the 'after' REST catches it, at one more empty throw.
  Watch for the WARN line.
- Columns (R5) has two balls. A drop there leaves the other ball in flight, so "stop before the
  next throw" is not obviously the right policy. That is an R5 decision.
