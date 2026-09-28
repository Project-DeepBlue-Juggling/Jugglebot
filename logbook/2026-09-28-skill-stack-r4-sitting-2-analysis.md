---
title: "R4 sitting 2: the self-toss held, the hop THROW refused on the post-release hold's axis under a real level offset, and the reload met a Ball Butler node that answered nothing"
type: investigation
date: 2026-09-28
status: in-progress
phase: "two-ball-skill-stack — R4"
related_plan: two-ball-skill-stack.md
sessions:
  - temp/logs/skills_r4_20260928_2141.log
  - ~/.ros/log/2026-09-28-21-41-09-858686-jetson-1498526/launch.log
  - ~/Desktop/rosbags/2026-09-28_21-41-10 (mcap — not in the repo)
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/unified_cycle.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - ros_ws/src/jugglebot/jugglebot/rosbridge_websocket_lean.py
  - ros_ws/gui/js/bb-aim.js
  - ros_ws/gui/js/ros-bridge.js
  - tests/ros/test_rosbridge_websocket_lean.py
  - tests/ros/test_gui_bb_aim_timeout.py
  - tests/ros/test_unified_cycle_levelling.py
  - tests/motion/test_unified_cycle.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - config/generated/admissible_box.yaml
  - tests/motion/test_skills_segments.py
  - tests/ros/test_skill_node.py
  - tests/hardware/session_skills_r4.md
---

# R4 sitting 2 (2026-09-28 21:41)

## Symptom

Owner's report, one launch:

1. Self-toss from a cold memory "fairly well": stable to `num_cycles: 5`; the one
   `num_cycles: 10` dropped after ~5–7 throws (log: `END ABORTED_NO_RELEASE`).
2. Every Ball Butler reload: `ABORTED_BB_THROW_TIMEOUT: reload refused: bb/throw_at_target did not
   answer in 2.0 s`. Owner: why does BB run its ball check before feeding at all? It should only
   run on NO_BALL; BB can feed as soon as its axes are in position.
3. The hop gate (`pattern: hop, apex 0.9, separation 250`): the platform moved to the throw pose,
   then `install_segment refused: REJECTED_CYCLE_INFEASIBLE(HAND_LIMIT_ACC: peak hand acceleration
   4969.7 rev/s^2 > 3500.0)` at skill 1 (THROW), three times. Owner's hypothesis: the hand's
   cogging jitter at the lower position (~0.8 mm) makes the plan snap it to the start at high jerk.
4. The GUI's BB yaw/pitch fields (orchestrator IDLE) do nothing and freeze the GUI until refresh.

## Diagnosis

### 1. The hop refusal is the post-release hold's axis under the level correction (not the jitter)

The refusal number was **4969.7 on all three attempts**, while the THROW-install seed differed each
time (hand −0.0016 / +0.0016 / −0.0027 rev/s; one attempt seeded IN MOTION at 27 mm/s). A
jitter-driven snap would move the number with the seed; a number identical to 0.1 is the plan
itself. The seed is also spliced from the REST's own record, not the encoder.

Offline replay through the real install chain (scratchpad probes; the reusable one is
`tools/probes/planned_release_motion.py`):

| Seed / planner | hop THROW peak hand acc (rev/s²) |
|---|---|
| no level correction (every sim gate) | 2980.2 |
| tonight's memory + boxes, no correction | accepted (learner exonerated, u_apex 0.8865) |
| the sitting's level offset (−3.857, +5.809) mrad | **REFUSED 5235.9** |
| y half only (+5.8 mrad, across the hop) | REFUSED 5229.2 |
| x half only (±5.8 mrad, along the hop) | 2980.2 |
| offset, hold off (`post_release_hold_s = 0`) | 3016.6 |
| offset, commit 9ef8d39 (before the hold) | 3016.6 |

So it is a regression from `b16561e` (the 50 ms hold), reachable only with a level correction that
has a component across the throw. The hold rows pin `v_xy = kappa·v_z` with `kappa` the cup axis
of the release tilt shifted into the plan frame. The release knot hands the tail the ballistic
launch velocity, whose direction is the **gravity-frame** axis; the correction rotates the held
line away from it by the correction angle. Along the hop that is a small change in an already large
`v_x`. Across it, the row demands `v_y = 0.0058·v_z ≈ 24 mm/s` one knot after a release with
`v_y = 0`; lateral velocity cannot jump under the jerk rows, so the QP satisfies the row by
collapsing `v_z`, which is the hand. The sim gates never set a level correction, which is why the
hop gate passed 25/25 twice. The on-axis self-toss is unaffected under the same offset at both commits
(offline, three throws: 3009.9 rev/s² at `b16561e`, 3008.8 with the fix), which matches the sitting.

### 2. The reload: a deaf Ball Butler node, and a race behind it

`ball_butler_node` started ("BallButler node ready") and then processed **no callback for the whole
session**: no "BB calibration received" after mocap published the calibration (sitting 1 logged
it), no service answer (`bb/throw_at_target` 3/3 timeouts; the GUI's `bb/aim` timed out at rosbridge's
50 s bound three times), exit clean at SIGINT. The bag recorder matched its `/bb/throw_outcome`
publisher, so its participant was discovered. No `ball_butler_node` code changed since sitting 1.
Not reproduced: run alone on domain 77, on domain 0 three times, and with the sitting's own bag
replayed into it (`/rigid_body_poses`, `/bb/heartbeat`, `/orchestrator_state`), it answers every
call. Candidates not ruled out: a Fast DDS participant that discovers but receives nothing
(`/dev/shm` holds 188 stale `fastrtps_*` segments back to 09-11), or an executor wedge that needs
the live bridge's action server. Neither can be separated without a stack dump of the live process.

Behind it, a second defect the dead node hid: the bag's `/bb/heartbeat` shows `bb/reload` → the
next heartbeat still IDLE (the state from **before** the check) → CHECKING_BALL 73–121 ms later →
IDLE 1.0 s after that, on all three reloads. The gate fired `bb/throw_at_target` ~20 ms after the
reload, on that stale IDLE. A responsive node would have drawn `THROW_REJECTED_BAD_STATE` again.
And `bb/reload` was never needed: every reload in both R4 sittings found the ball already in hand
(no RELOADING state in any 2026-09 bag), so it only ever bought the ~1 s ball check.

### 3. GUI freeze

The trigger is 2 (`bb/aim` never answered). The freeze is rosbridge's: stock 1.3.1
`CallService.call_service` runs `ServiceCaller(...).run()` on the browser tab's ONE
`IncomingQueue` thread, so every later message from the tab waits behind an unanswered call
until the lean rosbridge's 50 s bound (`CALL_SERVICE_TIMEOUT_S`, item 3 of that module) fires —
three such lines in the log. On the GUI side, `bb-aim.js` set no client timeout and a fresh
submit could stack a second call behind the first (a Sonnet agent's read, handoff in the session
scratchpad). When `bb/aim` does answer, refusals included, the GUI already handled it.

## Discussion

**Why the launch line and not the plan-frame tilt axis.** The hold exists so the ball, which
separates 20–40 ms late, rides a cup whose velocity stays on the launch line. The cup velocity is
what the ball feels; the platform's own velocity is how the planner realises it. Under a correction
the two cannot both be held: the release knot fixes the cup velocity to the launch velocity, and a
frozen platform would move the cup along the plan-frame axis. The first version chose "platform
frozen" and turned an 0.33° disagreement into an infeasible plan. Holding the launch line keeps the
cup continuous through the release knot and leaves the platform translating at
`v_z·(kappa_launch − axis_plan_xy)`, the same small velocity it already has at the release knot
(24 mm/s at 5.8 mrad and 4.2 m/s), decaying with `v_z`. With no correction the two lines agree to
the sin/tan difference of the 4° release tilt (0.48 mm/s off the old line in the hop test), so the
self-toss plans are unchanged to the printed digit and the hop's hold verdict reads 2.74 mm/s.
Ruled out: widening the hand limit (the plan was not asking for more hand; the constraint was
inconsistent), and dropping the hold under a correction (every real sitting has one).

**Why the reload skips `bb/reload` on a ball in hand, and what still guards the no-ball case.** The
owner's physical point stands: the check is for an empty hand. The decision reads only a heartbeat
fresher than 0.5 s (10 Hz measured); anything older is not evidence of a ball and falls through to
`bb/reload`. After a `bb/reload`, only an IDLE that follows a non-IDLE state counts, which closes the
race directly rather than by a delay. It assumes BB's non-IDLE interval outlasts one heartbeat period
(100 ms); the measured CHECKING_BALL dwell is 1.0 s. A shorter one would read as a 10 s
`ABORTED_BB_NOT_READY`, not a wrong throw. The 10 s wait after `bb/reload` is a ceiling, not a
measurement: no bag holds a physical fetch.

**The deaf node is carried, not fixed.** A code fix would be a guess. The runsheet now makes the
next sitting prove the node is listening before the reload block, and says to capture a `py-spy`
dump if it is not.

## Fix

- `motion/unified_cycle.py`: the post-release hold's line is the seed cup velocity's own
  `v_xy / v_z` (the launch line); the plan-frame tilt axis is kept only as a fallback below 0.5 m/s
  of `v_z`, which no real release reaches.
- `skill_node.py`: `bb/reload` only when a fresh heartbeat does not report a ball in hand
  (`RELOAD_BB_FETCH_TIMEOUT_S` 10 s after one, `RELOAD_BB_READY_TIMEOUT_S` 3 s otherwise); the
  ready gate needs a fresh heartbeat and, after a `bb/reload`, BB having left IDLE
  (`_on_bb_heartbeat` sets `bb_left_idle`); `ABORTED_BB_THROW_TIMEOUT` names the node, not BB.
- `rosbridge_websocket_lean.py` item 5: each `call_service` runs on its own daemon thread
  (a shim whose `run()` calls the stock caller's `start()`), so one unanswered call no longer
  stalls the tab. Replies still go out through `IOLoop.add_callback` (thread-safe) and still pass
  item 1's client destroy and item 3's bound.
- GUI (`ros_ws/gui/js/bb-aim.js`, `ros-bridge.js`, Sonnet agent): at most one `bb/aim` in flight,
  a 3 s client timeout (`withTimeout`), the operator told within 3 s. A judgement value, not
  bench-measured.
- `tests/ros/test_unified_cycle_levelling.py`: the five tests that install from a fresh origin
  now freeze `perf_counter` (the install tests' own `_frozen_perf`). Since this morning's
  `ORIGIN_TOO_LATE` they compared a real solve time against the 75 ms wire lead and all five
  refused under the evening's sweep load (0.15–0.55 s solves), which is load, not the frame
  logic they test.
- `INVARIANTS.md` `ABORTED_BB_NOT_READY` row, runsheet § "since sitting 2", step 27 listening
  check, § 7 `ABORTED_BB_THROW_TIMEOUT` row, § 11 sitting 2.
- `config/generated/admissible_box.yaml` re-swept (`unified_cycle.py` is in the gate hash).

## Verification

* 2026-09-28, `pytest tests/motion/test_skills_segments.py -k level_correction` against the
  committed `unified_cycle.py` → **FAIL** `CycleInfeasible HAND_LIMIT_ACC 5235.7`; with the fix →
  **PASS**. The reload trio (`-k "ball_already_in_hand or before_the_ball_check or
  stale_bb_heartbeat"`) against the committed `skill_node.py` → **3 FAIL** (a `bb/reload` call
  recorded; the 3.0 s deadline; `AttributeError` on the heartbeat age) → **3 PASS** after.
  `tests/ros/test_rosbridge_websocket_lean.py` → 32 passed (2 new); the shim also checked against
  the real Foxy `ServiceCaller` (`run()` returned in 0.000 s, callback delivered).
  `tests/ros/test_gui_bb_aim_timeout.py` → 3 passed (agent: fail-before shown by running the
  harness against the committed JS).
* 2026-09-28 22:21–23:03 (`temp/logs/admissible_sweep_r4c_run{1,2}_20260928_2221.log`),
  `python tools/admissible_sweep.py --site-pairs all --single-apex 0.5 0.6 0.7 0.8 0.9 --out
  temp/probes/admissible_box_r4c_run<n>.yaml`, two runs in parallel, `OMP/OPENBLAS_NUM_THREADS=1`
  (42.2 min) → **bit-identical** (`swept_at` excluded), **gate_hash `bf095653d422`**; one bound
  moved: hop P2→P1 x lower −1.0 → −0.5 mm.
* 2026-09-28 23:07–23:18 (`temp/logs/r4c_gates_20260928_2307.log`), `python sim/skills_gate.py
  --learn --pattern self_toss --seeds 0 1 2 3 4 --no-viewer` twice → **PASS 5/5 both runs, 25
  makes / 0 drops / band entry 3 per seed, bit-identical with timing stripped**; `--pattern hop`
  twice → **25 makes / 0 drops every seed, bit-identical**; band verdict `None` as at R4 and in
  sitting 1's entry (the hop learner is box-clipped).
* 2026-09-28 23:18 (`temp/logs/gate_full_r4c_20260928_2318.log`), `./run_tests.sh --full` →
  **PASS 5554 passed, 8 skipped, 1 xfailed in 282.68 s; serial 6 passed in 19.83 s.**
* 2026-09-28 23:24 (`temp/logs/gate_r4c_precommit_20260928_2324.log`), `./run_tests.sh` after the
  audit's docstring edit → **PASS 5516 passed, 8 skipped in 216.37 s; serial 3 passed in 8.77 s.**
* Audit (`/audit --unstaged`, one Sonnet reporter): no behaviour findings; two narrative notes
  applied (the test's fail-before figure names its seed; the left-IDLE sampling assumption).
